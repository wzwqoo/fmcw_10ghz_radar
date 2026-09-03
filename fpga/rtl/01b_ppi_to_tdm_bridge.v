// =============================================================================
//  ppi_to_tdm_bridge.v
//
//  Adapter between adar7251_ppi_rx (3 channels simultaneously valid for one
//  sclk_adc cycle, every ~833 ns) and tdm_mux_3to1 (3 channels staggered,
//  one per fast clock cycle).
//
//  Adds one register per channel and a 3-cycle valid window on the TDM side
//  so the downstream tdm_mux_3to1 can select all three channels
//  with its own round-robin counter.
//
//  Fix (BUG_AUDIT MAJOR #2):
//  Previously the PPI receiver fed all four channels valid on the same
//  cycle.  The TDM mux's free-running counter would sample one channel and
//  miss the other three until the next sample set.  This bridge latches the
//  parallel sample and holds all four channel-valid flags high for four TDM
//  clocks, letting the downstream mux's own counter select one valid channel
//  per clock.
//
//  Domain handling:
//    * Inputs (ch*_in_data, in_valid) are in the sclk_adc domain (9.6 MHz).
//    * Outputs are in the TDM clock domain (clk_tdm, typically 48 MHz =
//      4 × ADC sample rate).  in_valid → tdm pulse uses a CDC handshake:
//      because tdm clock is FASTER than the producer's burst rate, simple
//      2-FF synchronisation of a level signal works (pulse stretched by
//      latching ch*_in_data into an "armed" register).
//
//  Throughput: at fs=1.2 MSPS per channel, the producer emits one full
//  set every 100 MHz / 1.2 MHz ≈ 83 cycles of clk_sys.  The TDM consumer
//  needs 4 cycles per set, so plenty of headroom; the latch never overruns.
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module ppi_to_tdm_bridge #(
    parameter DATA_W = 16
)(
    // Source side (sclk_adc domain)
    input  wire                       sclk_adc,
    input  wire                       rst_n_adc,
    input  wire signed [DATA_W-1:0]   ch0_in_data,
    input  wire signed [DATA_W-1:0]   ch1_in_data,
    input  wire signed [DATA_W-1:0]   ch2_in_data,
    input  wire                       in_valid,    // one-cycle pulse, all 3 chs valid

    // Sink side (TDM clock domain)
    input  wire                       clk_tdm,
    input  wire                       rst_n_tdm,
    output reg  signed [DATA_W-1:0]   ch0_data,
    output reg  signed [DATA_W-1:0]   ch1_data,
    output reg  signed [DATA_W-1:0]   ch2_data,
    output reg                        ch0_vld,
    output reg                        ch1_vld,
    output reg                        ch2_vld
);

    // ── Source-side latch ────────────────────────────────────────────────
    reg signed [DATA_W-1:0] ch0_lat, ch1_lat, ch2_lat;
    reg                     handshake_adc;    // toggle on each new sample set

    always @(posedge sclk_adc) begin
        if (!rst_n_adc) begin
            handshake_adc <= 1'b0;
            ch0_lat <= {DATA_W{1'b0}};
            ch1_lat <= {DATA_W{1'b0}};
            ch2_lat <= {DATA_W{1'b0}};
        end else if (in_valid) begin
            ch0_lat <= ch0_in_data;
            ch1_lat <= ch1_in_data;
            ch2_lat <= ch2_in_data;
            handshake_adc <= ~handshake_adc;   // toggle = new data available
        end
    end

    // ── CDC: synchronise toggle into clk_tdm domain ──────────────────────
    reg [2:0] hs_sync;
    always @(posedge clk_tdm) begin
        if (!rst_n_tdm) hs_sync <= 3'b0;
        else            hs_sync <= {hs_sync[1:0], handshake_adc};
    end
    wire new_sample_tdm = hs_sync[2] ^ hs_sync[1];   // edge detect

    // ── Three-cycle output window for downstream tdm_mux_3to1 ────────────
    reg [1:0] dist_state;
    localparam D_IDLE = 2'd0,
               D_CH0  = 2'd1,
               D_CH1  = 2'd2,
               D_CH2  = 2'd3;

    reg signed [DATA_W-1:0] ch0_held, ch1_held, ch2_held;

    always @(posedge clk_tdm) begin
        if (!rst_n_tdm) begin
            dist_state <= D_IDLE;
            ch0_vld <= 1'b0; ch1_vld <= 1'b0; ch2_vld <= 1'b0;
            ch0_data <= {DATA_W{1'b0}};
            ch1_data <= {DATA_W{1'b0}};
            ch2_data <= {DATA_W{1'b0}};
        end else begin
            ch0_vld <= 1'b0; ch1_vld <= 1'b0; ch2_vld <= 1'b0;

            case (dist_state)
                D_IDLE: begin
                    if (new_sample_tdm) begin
                        // Re-latch into held regs to cross the CDC boundary safely:
                        // hs_sync[2]=hs_sync[1] would no longer be 'edge' next cycle,
                        // so we capture ch*_lat now.  (Holding pattern means
                        // 3 producer cycles in clk_tdm domain — safe at 48 vs 9.6 MHz.)
                        ch0_held <= ch0_lat;
                        ch1_held <= ch1_lat;
                        ch2_held <= ch2_lat;
                        dist_state <= D_CH0;
                    end
                end
                D_CH0: begin
                    ch0_data <= ch0_held; ch1_data <= ch1_held; ch2_data <= ch2_held;
                    ch0_vld <= 1'b1; ch1_vld <= 1'b1; ch2_vld <= 1'b1;
                    dist_state <= D_CH1;
                end
                D_CH1: begin
                    ch0_data <= ch0_held; ch1_data <= ch1_held; ch2_data <= ch2_held;
                    ch0_vld <= 1'b1; ch1_vld <= 1'b1; ch2_vld <= 1'b1;
                    dist_state <= D_CH2;
                end
                D_CH2: begin
                    ch0_data <= ch0_held; ch1_data <= ch1_held; ch2_data <= ch2_held;
                    ch0_vld <= 1'b1; ch1_vld <= 1'b1; ch2_vld <= 1'b1;
                    dist_state <= D_IDLE;
                end
                default: dist_state <= D_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
