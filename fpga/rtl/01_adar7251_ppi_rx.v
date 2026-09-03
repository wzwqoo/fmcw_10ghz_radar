// =============================================================================
// adar7251_ppi_rx.v   *** 3-RX VARIANT ***
// Single ADAR7251 chip, PPI byte-wide mode.
//
// The ADAR7251 always streams all 4 of its physical channels in PPI byte mode
// (8 bytes per sample set).  On the 3-RX board only channels 1..3 are wired to
// antennas, so this variant still receives all 8 bytes (preserving SCLK/DATA_READY
// timing and the byte-7 valid pulse) but exposes only ch0_out..ch2_out.  The 4th
// channel's bytes are clocked through for timing but discarded.
//
// PPI byte-wide mode timing (from datasheet Figure 46):
//
//   CONV_START ──────┐ (active low, pulse from FPGA to start conversion)
//                    │
//   tCSDR = 1.215µs  │
//                    ▼
//   DATA_READY ──────────┐ (goes HIGH when data is ready)
//                        │
//   SCLK_ADC  ─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─
//               0   1   2   3   4   5   6   7
//
//   DOUT[7:0]: [CH1_HI][CH1_LO][CH2_HI][CH2_LO]
//              [CH3_HI][CH3_LO][CH4_HI][CH4_LO]
//
//   8 SCLK_ADC cycles deliver all 4 channels (2 bytes each).
//   Data is valid on RISING edge of SCLK_ADC.
//   High byte output first (PAR_ENDIAN=0 in register 0x1C1).
//
// ADAR7251 register setup for PPI byte mode:
//   0x1C2: write 0x0001  → OUTPUT_MODE=1 (parallel), CS_OVERRIDE=0
//   0x1C1: write 0x0004  → PAR_NIBBLE=1 (byte), PAR_ENDIAN=0 (HI first),
//                           PAR_CHANNELS=0 (4 channels)
//   0x042: POUT_EN=1, SOUT_EN=0 (enable parallel, disable serial block)
//
// At 1.2 MSPS, 4 channels, byte mode:
//   SCLK_ADC = 4 channels × 2 bytes × 1.2MSPS = 9.6 MHz
//   DATA_READY period ≈ 1/1.2MHz = 833 ns
//   8 SCLK cycles × (1/9.6MHz) = 833 ns → back-to-back, no gap
//
// FPGA interface:
//   sclk_adc    : input  9.6 MHz (from ADAR7251 SCLK_ADC pin, route via BUFG)
//   data_ready  : input  (from ADAR7251 DATA_READY pin)
//   dout        : input  [7:0] (ADAR7251 ADC_DOUT0..7)
//   conv_start  : output (active-low pulse to ADAR7251 CONV_START)
//
//   ch_out[0..3]: output [15:0] channels 1..4, signed 16-bit
//   valid_out   : output pulses ONE sclk_adc cycle after 8th byte received
//
// Reset domains (BUG_AUDIT MAJOR #1 fix):
//   `rst` is a free-running async reset.  We synchronise it independently
//   into each clock domain (clk_sys, sclk_adc) before use.  Never reference
//   `rst` directly in sequential logic — use rst_clk_sys or rst_sclk.
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module adar7251_ppi_rx (
    // ADAR7251 interface
    input  wire        sclk_adc,       // 9.6 MHz (route through BUFG)
    input  wire        data_ready,     // HIGH when conversion data available
    input  wire [7:0]  dout,           // 8-bit parallel data bus

    // FPGA drives CONV_START
    output reg         conv_start,     // active-low to ADAR7251

    // Recovered 3-channel samples (CH4 received but discarded on 3-RX board)
    output reg  signed [15:0] ch0_out,
    output reg  signed [15:0] ch1_out,
    output reg  signed [15:0] ch2_out,
    output reg         valid_out,      // one sclk_adc pulse

    // System clock for CONV_START timing
    input  wire        clk_sys,        // 100 MHz system clock
    input  wire        rst             // async — synchronised internally
);

// ── Per-domain reset synchronisers (BUG_AUDIT MAJOR #1) ───────────────────
// Standard 2-FF rule: capture async input, give one settle stage, then use.
reg [1:0] rst_sync_clk_sys;
reg [1:0] rst_sync_sclk;

always @(posedge clk_sys or posedge rst) begin
    if (rst) rst_sync_clk_sys <= 2'b11;
    else     rst_sync_clk_sys <= {rst_sync_clk_sys[0], 1'b0};
end
always @(posedge sclk_adc or posedge rst) begin
    if (rst) rst_sync_sclk <= 2'b11;
    else     rst_sync_sclk <= {rst_sync_sclk[0], 1'b0};
end

wire rst_clk_sys = rst_sync_clk_sys[1];
wire rst_sclk    = rst_sync_sclk[1];

// ---------------------------------------------------------------------------
// CONV_START generator (system clock domain)
// Pulse conv_start low for one clk_sys cycle every 833 ns (1.2 MSPS)
// 833 ns × 100 MHz = 83 cycles between conversions
// ---------------------------------------------------------------------------
localparam CONV_PERIOD = 83;   // clk_sys cycles between conversions

reg [$clog2(CONV_PERIOD):0] conv_cnt;

always @(posedge clk_sys) begin
    if (rst_clk_sys) begin
        conv_cnt   <= {($clog2(CONV_PERIOD)+1){1'b0}};
        conv_start <= 1'b1;     // idle high
    end else begin
        conv_start <= 1'b1;
        conv_cnt   <= conv_cnt + 1;
        if (conv_cnt == CONV_PERIOD - 1) begin
            conv_cnt   <= {($clog2(CONV_PERIOD)+1){1'b0}};
            conv_start <= 1'b0;   // pulse low for one cycle
        end
    end
end

// ---------------------------------------------------------------------------
// PPI data capture (sclk_adc domain, 9.6 MHz)
// ---------------------------------------------------------------------------
reg [2:0]  byte_cnt;      // 0..7 counts 8 bytes per sample set
reg        dr_prev;       // previous DATA_READY for edge detect
reg        capturing;

// Temporary high-byte storage
reg [7:0]  hi0, hi1, hi2, hi3;

always @(posedge sclk_adc) begin
    if (rst_sclk) begin
        byte_cnt  <= 3'd0;
        dr_prev   <= 1'b0;
        capturing <= 1'b0;
        valid_out <= 1'b0;
        ch0_out   <= 16'sd0;
        ch1_out   <= 16'sd0;
        ch2_out   <= 16'sd0;
        hi0       <= 8'h00;
        hi1       <= 8'h00;
        hi2       <= 8'h00;
        hi3       <= 8'h00;
    end else begin
        valid_out <= 1'b0;
        dr_prev   <= data_ready;

        // Rising edge of DATA_READY: conversion complete, data streaming starts
        if (data_ready && !dr_prev) begin
            byte_cnt  <= 3'd0;
            capturing <= 1'b1;
        end

        if (capturing && data_ready) begin
            // Bytes arrive in order: CH1_HI, CH1_LO, CH2_HI, CH2_LO,
            //                        CH3_HI, CH3_LO, CH4_HI, CH4_LO
            case (byte_cnt)
                3'd0: hi0 <= dout;                          // CH1 high byte
                3'd1: ch0_out <= $signed({hi0, dout});      // CH1 complete
                3'd2: hi1 <= dout;                          // CH2 high byte
                3'd3: ch1_out <= $signed({hi1, dout});      // CH2 complete
                3'd4: hi2 <= dout;                          // CH3 high byte
                3'd5: ch2_out <= $signed({hi2, dout});      // CH3 complete
                3'd6: hi3 <= dout;                          // CH4 high byte (discarded)
                3'd7: begin
                    // CH4 low byte arrives here but is discarded on 3-RX board.
                    valid_out <= 1'b1;  // channels 0..2 ready
                    capturing <= 1'b0;
                end
            endcase

            byte_cnt <= byte_cnt + 3'd1;
        end

        // Safety: if DATA_READY goes low mid-frame, abort
        if (!data_ready && dr_prev && capturing) begin
            capturing <= 1'b0;
            byte_cnt  <= 3'd0;
        end
    end
end

// ---------------------------------------------------------------------------
// Sanity check: verify SCLK_ADC is 9.6 MHz
// At 100 MHz system clock: 100MHz / 9.6MHz = 10.4 cycles per SCLK period
// This is only for simulation verification, synthesis ignores it.
// ---------------------------------------------------------------------------
// synthesis translate_off
initial begin
    $display("adar7251_ppi_rx: PPI byte mode, 3 channels used (CH4 discarded)");
    $display("  Expected SCLK_ADC = 9.6 MHz");
    $display("  Expected DATA_READY period = 833 ns (1.2 MSPS)");
    $display("  8 bytes per sample set, all channels valid on byte 7");
end
// synthesis translate_on

endmodule
`default_nettype wire
