// =============================================================================
//  tdm_mux_3to1.v   *** 3-RX VARIANT ***
//  Routes 3 slow-rate channels into a single TDM stream at 3x the input rate.
//
//  Usage model
//  -----------
//  All logic runs on a single fast clock (clk = 4×fs).
//  Each input channel is expected to present valid data every 3rd clock cycle,
//  staggered by 1 clock:
//    ch0 valid at t = 0, 3, 6, ...
//    ch1 valid at t = 1, 4, 7, ...
//    ch2 valid at t = 2, 5, 8, ...
//
//  The 2-bit counter (cnt) round-robins the channel selection, wrapping 2->0.
//  Both tdm_data and tdm_ch_id are registered for clean timing closure.
//
//  Vivado FIR Compiler port mapping (s_axis side):
//    .s_axis_data_tdata  (tdm_data)
//    .s_axis_data_tvalid (tdm_valid)
//
//  Parameters
//  ----------
//  DATA_W : data bus width in bits (match your ADC / FIR Compiler tdata width)
// =============================================================================

`default_nettype none

module tdm_mux_3to1 #(
    parameter DATA_W = 16
)(
    input  wire             clk,
    input  wire             rst_n,

    // ── 3 input channels (data registered in slow domain, sampled here) ────
    input  wire [DATA_W-1:0] ch0_data,
    input  wire [DATA_W-1:0] ch1_data,
    input  wire [DATA_W-1:0] ch2_data,
    input  wire              ch0_vld,
    input  wire              ch1_vld,
    input  wire              ch2_vld,

    // ── TDM output stream ──────────────────────────────────────────────────
    output reg  [DATA_W-1:0] tdm_data,
    output reg  [1:0]        tdm_ch_id,  // channel tag — must travel downstream
    output reg               tdm_valid   // high every cycle when any ch is valid
);

    // ── Modulo-3 counter (wraps 2 -> 0) ────────────────────────────────────
    reg [1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            cnt <= 2'd0;
        else if (cnt == 2'd2)  cnt <= 2'd0;
        else                   cnt <= cnt + 1'b1;
    end

    // ── Combinational mux (registered 1 cycle below) ───────────────────────
    reg [DATA_W-1:0] mux_d;
    reg              mux_v;

    always @(*) begin
        case (cnt)
            2'd0: begin mux_d = ch0_data; mux_v = ch0_vld; end
            2'd1: begin mux_d = ch1_data; mux_v = ch1_vld; end
            2'd2: begin mux_d = ch2_data; mux_v = ch2_vld; end
            default: begin mux_d = {DATA_W{1'b0}}; mux_v = 1'b0; end
        endcase
    end

    // ── Output register (ensures clean timing into FIR tdata/tvalid) ───────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tdm_data  <= {DATA_W{1'b0}};
            tdm_ch_id <= 2'd0;
            tdm_valid <= 1'b0;
        end else begin
            tdm_data  <= mux_d;
            tdm_ch_id <= cnt;      // tag = cnt at this clock = which ch is on bus
            tdm_valid <= mux_v;
        end
    end

endmodule

`default_nettype wire
