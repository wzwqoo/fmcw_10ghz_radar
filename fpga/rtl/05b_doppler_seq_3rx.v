// =============================================================================
//  doppler_seq_3rx.v   *** 3-RX VARIANT ***
//
//  Three-lane wrapper for doppler_seq_ctrl.  Use this at the block-design
//  boundary when the design has three Doppler FFT IPs, one per RX channel.
//
//  Each lane reads one RX slice from DDR A ping/pong and streams one
//  N_CHIRPS-beat frame per range bin into its matching Doppler FFT.
//
//      rd0_buf_base/valid -> lane 0 -> Doppler FFT 0
//      rd1_buf_base/valid -> lane 1 -> Doppler FFT 1
//      rd2_buf_base/valid -> lane 2 -> Doppler FFT 2
//
//  The single-lane doppler_seq_ctrl remains the primitive so its existing
//  unit test and timing behaviour stay isolated.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module doppler_seq_3rx #(
    parameter DATA_W       = 32,
    parameter ADDR_W       = 32,
    parameter N_RANGE_BINS = `RADAR_RANGE_BINS,
    parameter N_CHIRPS     = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES = `RADAR_SAMPLE_BYTES
)(
    input  wire clk,
    input  wire rst_n,

    // ── From scatter_write_3rx ping-pong read-side status ─────────────────
    input  wire [ADDR_W-1:0] rd0_buf_base,
    input  wire [ADDR_W-1:0] rd1_buf_base,
    input  wire [ADDR_W-1:0] rd2_buf_base,
    input  wire [2:0]        rd_buf_valid,
    output wire [2:0]        doppler_frame_done,

    // ── AXI4 read master 0 ────────────────────────────────────────────────
    output wire [ADDR_W-1:0] m0_araddr,
    output wire [7:0]        m0_arlen,
    output wire [2:0]        m0_arsize,
    output wire [1:0]        m0_arburst,
    output wire              m0_arvalid,
    input  wire              m0_arready,
    input  wire [DATA_W-1:0] m0_rdata,
    input  wire [1:0]        m0_rresp,
    input  wire              m0_rlast,
    input  wire              m0_rvalid,
    output wire              m0_rready,

    // ── AXI4 read master 1 ────────────────────────────────────────────────
    output wire [ADDR_W-1:0] m1_araddr,
    output wire [7:0]        m1_arlen,
    output wire [2:0]        m1_arsize,
    output wire [1:0]        m1_arburst,
    output wire              m1_arvalid,
    input  wire              m1_arready,
    input  wire [DATA_W-1:0] m1_rdata,
    input  wire [1:0]        m1_rresp,
    input  wire              m1_rlast,
    input  wire              m1_rvalid,
    output wire              m1_rready,

    // ── AXI4 read master 2 ────────────────────────────────────────────────
    output wire [ADDR_W-1:0] m2_araddr,
    output wire [7:0]        m2_arlen,
    output wire [2:0]        m2_arsize,
    output wire [1:0]        m2_arburst,
    output wire              m2_arvalid,
    input  wire              m2_arready,
    input  wire [DATA_W-1:0] m2_rdata,
    input  wire [1:0]        m2_rresp,
    input  wire              m2_rlast,
    input  wire              m2_rvalid,
    output wire              m2_rready,

    // ── AXI-Stream outputs to Doppler FFT s_axis_data ─────────────────────
    output wire [DATA_W-1:0] m0_axis_tdata,
    output wire              m0_axis_tvalid,
    output wire              m0_axis_tlast,
    input  wire              m0_axis_tready,

    output wire [DATA_W-1:0] m1_axis_tdata,
    output wire              m1_axis_tvalid,
    output wire              m1_axis_tlast,
    input  wire              m1_axis_tready,

    output wire [DATA_W-1:0] m2_axis_tdata,
    output wire              m2_axis_tvalid,
    output wire              m2_axis_tlast,
    input  wire              m2_axis_tready,

    // ── Matching Doppler FFT output completion pulses ─────────────────────
    input  wire              fft0_out_valid,
    input  wire              fft0_out_last,
    input  wire              fft1_out_valid,
    input  wire              fft1_out_last,
    input  wire              fft2_out_valid,
    input  wire              fft2_out_last,

    output wire [2:0]        busy
);

    doppler_seq_ctrl #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES)
    ) u_rx0 (
        .clk(clk), .rst_n(rst_n),
        .buf_base(rd0_buf_base), .buf_valid(rd_buf_valid[0]),
        .doppler_frame_done(doppler_frame_done[0]),
        .m_axi_araddr(m0_araddr), .m_axi_arlen(m0_arlen),
        .m_axi_arsize(m0_arsize), .m_axi_arburst(m0_arburst),
        .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
        .m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp),
        .m_axi_rlast(m0_rlast), .m_axi_rvalid(m0_rvalid),
        .m_axi_rready(m0_rready),
        .m_axis_tdata(m0_axis_tdata), .m_axis_tvalid(m0_axis_tvalid),
        .m_axis_tlast(m0_axis_tlast), .m_axis_tready(m0_axis_tready),
        .fft_out_valid(fft0_out_valid), .fft_out_last(fft0_out_last),
        .busy(busy[0])
    );

    doppler_seq_ctrl #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES)
    ) u_rx1 (
        .clk(clk), .rst_n(rst_n),
        .buf_base(rd1_buf_base), .buf_valid(rd_buf_valid[1]),
        .doppler_frame_done(doppler_frame_done[1]),
        .m_axi_araddr(m1_araddr), .m_axi_arlen(m1_arlen),
        .m_axi_arsize(m1_arsize), .m_axi_arburst(m1_arburst),
        .m_axi_arvalid(m1_arvalid), .m_axi_arready(m1_arready),
        .m_axi_rdata(m1_rdata), .m_axi_rresp(m1_rresp),
        .m_axi_rlast(m1_rlast), .m_axi_rvalid(m1_rvalid),
        .m_axi_rready(m1_rready),
        .m_axis_tdata(m1_axis_tdata), .m_axis_tvalid(m1_axis_tvalid),
        .m_axis_tlast(m1_axis_tlast), .m_axis_tready(m1_axis_tready),
        .fft_out_valid(fft1_out_valid), .fft_out_last(fft1_out_last),
        .busy(busy[1])
    );

    doppler_seq_ctrl #(
        .DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES)
    ) u_rx2 (
        .clk(clk), .rst_n(rst_n),
        .buf_base(rd2_buf_base), .buf_valid(rd_buf_valid[2]),
        .doppler_frame_done(doppler_frame_done[2]),
        .m_axi_araddr(m2_araddr), .m_axi_arlen(m2_arlen),
        .m_axi_arsize(m2_arsize), .m_axi_arburst(m2_arburst),
        .m_axi_arvalid(m2_arvalid), .m_axi_arready(m2_arready),
        .m_axi_rdata(m2_rdata), .m_axi_rresp(m2_rresp),
        .m_axi_rlast(m2_rlast), .m_axi_rvalid(m2_rvalid),
        .m_axi_rready(m2_rready),
        .m_axis_tdata(m2_axis_tdata), .m_axis_tvalid(m2_axis_tvalid),
        .m_axis_tlast(m2_axis_tlast), .m_axis_tready(m2_axis_tready),
        .fft_out_valid(fft2_out_valid), .fft_out_last(fft2_out_last),
        .busy(busy[2])
    );

endmodule

`default_nettype wire
