// =============================================================================
//  scatter_write_3rx.v   *** 3-RX VARIANT ***
//
//  Wrapper for three transposed range-FFT scatter writers.  Connect one Range
//  FFT m_axis_data output to each sN_* input.  Each writer stores its RX map at
//  A ping/pong base plus RX_INDEX * RADAR_RD_MAP_BYTES.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module scatter_write_3rx #(
    parameter DATA_W          = 32,
    parameter ADDR_W          = 32,
    parameter N_RANGE_BINS    = `RADAR_RANGE_BINS,
    parameter N_CHIRPS        = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES    = `RADAR_SAMPLE_BYTES,
    parameter MAX_OUTSTANDING = 8,
    parameter BUF_A_BASE      = `RADAR_DDR_A_PING,
    parameter BUF_B_BASE      = `RADAR_DDR_A_PONG,
    parameter RX_STRIDE       = N_RANGE_BINS * N_CHIRPS * SAMPLE_BYTES
)(
    input  wire clk,
    input  wire rst_n,

    input  wire [DATA_W-1:0] s0_tdata, input  wire s0_tvalid, output wire s0_tready,
    input  wire [DATA_W-1:0] s1_tdata, input  wire s1_tvalid, output wire s1_tready,
    input  wire [DATA_W-1:0] s2_tdata, input  wire s2_tvalid, output wire s2_tready,

    output wire [ADDR_W-1:0] m0_awaddr, output wire [7:0] m0_awlen, output wire [2:0] m0_awsize,
    output wire [1:0] m0_awburst, output wire m0_awvalid, input wire m0_awready,
    output wire [DATA_W-1:0] m0_wdata, output wire [DATA_W/8-1:0] m0_wstrb,
    output wire m0_wlast, output wire m0_wvalid, input wire m0_wready,
    input wire [1:0] m0_bresp, input wire m0_bvalid, output wire m0_bready,

    output wire [ADDR_W-1:0] m1_awaddr, output wire [7:0] m1_awlen, output wire [2:0] m1_awsize,
    output wire [1:0] m1_awburst, output wire m1_awvalid, input wire m1_awready,
    output wire [DATA_W-1:0] m1_wdata, output wire [DATA_W/8-1:0] m1_wstrb,
    output wire m1_wlast, output wire m1_wvalid, input wire m1_wready,
    input wire [1:0] m1_bresp, input wire m1_bvalid, output wire m1_bready,

    output wire [ADDR_W-1:0] m2_awaddr, output wire [7:0] m2_awlen, output wire [2:0] m2_awsize,
    output wire [1:0] m2_awburst, output wire m2_awvalid, input wire m2_awready,
    output wire [DATA_W-1:0] m2_wdata, output wire [DATA_W/8-1:0] m2_wstrb,
    output wire m2_wlast, output wire m2_wvalid, input wire m2_wready,
    input wire [1:0] m2_bresp, input wire m2_bvalid, output wire m2_bready,

    output wire [ADDR_W-1:0] rd0_buf_base,
    output wire [ADDR_W-1:0] rd1_buf_base,
    output wire [ADDR_W-1:0] rd2_buf_base,
    output wire [2:0]        rd_buf_valid,
    input  wire [2:0]        doppler_frame_done,

    output wire [2:0]        busy,
    output wire [2:0]        stall
);

    wire [1:0] pp_state_unused [0:2];

    scatter_write_master #(.DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES), .MAX_OUTSTANDING(MAX_OUTSTANDING),
        .RX_INDEX(0), .RX_STRIDE(RX_STRIDE),
        .BUF_A_BASE(BUF_A_BASE), .BUF_B_BASE(BUF_B_BASE))
    u_rx0 (.clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s0_tdata), .s_axis_tvalid(s0_tvalid), .s_axis_tready(s0_tready),
        .m_axi_awaddr(m0_awaddr), .m_axi_awlen(m0_awlen), .m_axi_awsize(m0_awsize),
        .m_axi_awburst(m0_awburst), .m_axi_awvalid(m0_awvalid), .m_axi_awready(m0_awready),
        .m_axi_wdata(m0_wdata), .m_axi_wstrb(m0_wstrb), .m_axi_wlast(m0_wlast),
        .m_axi_wvalid(m0_wvalid), .m_axi_wready(m0_wready),
        .m_axi_bresp(m0_bresp), .m_axi_bvalid(m0_bvalid), .m_axi_bready(m0_bready),
        .rd_buf_base(rd0_buf_base), .rd_buf_valid(rd_buf_valid[0]),
        .doppler_frame_done(doppler_frame_done[0]),
        .busy(busy[0]), .stall(stall[0]), .pp_state_out(pp_state_unused[0]));

    scatter_write_master #(.DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES), .MAX_OUTSTANDING(MAX_OUTSTANDING),
        .RX_INDEX(1), .RX_STRIDE(RX_STRIDE),
        .BUF_A_BASE(BUF_A_BASE), .BUF_B_BASE(BUF_B_BASE))
    u_rx1 (.clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s1_tdata), .s_axis_tvalid(s1_tvalid), .s_axis_tready(s1_tready),
        .m_axi_awaddr(m1_awaddr), .m_axi_awlen(m1_awlen), .m_axi_awsize(m1_awsize),
        .m_axi_awburst(m1_awburst), .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready),
        .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb), .m_axi_wlast(m1_wlast),
        .m_axi_wvalid(m1_wvalid), .m_axi_wready(m1_wready),
        .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid), .m_axi_bready(m1_bready),
        .rd_buf_base(rd1_buf_base), .rd_buf_valid(rd_buf_valid[1]),
        .doppler_frame_done(doppler_frame_done[1]),
        .busy(busy[1]), .stall(stall[1]), .pp_state_out(pp_state_unused[1]));

    scatter_write_master #(.DATA_W(DATA_W), .ADDR_W(ADDR_W),
        .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
        .SAMPLE_BYTES(SAMPLE_BYTES), .MAX_OUTSTANDING(MAX_OUTSTANDING),
        .RX_INDEX(2), .RX_STRIDE(RX_STRIDE),
        .BUF_A_BASE(BUF_A_BASE), .BUF_B_BASE(BUF_B_BASE))
    u_rx2 (.clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s2_tdata), .s_axis_tvalid(s2_tvalid), .s_axis_tready(s2_tready),
        .m_axi_awaddr(m2_awaddr), .m_axi_awlen(m2_awlen), .m_axi_awsize(m2_awsize),
        .m_axi_awburst(m2_awburst), .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready),
        .m_axi_wdata(m2_wdata), .m_axi_wstrb(m2_wstrb), .m_axi_wlast(m2_wlast),
        .m_axi_wvalid(m2_wvalid), .m_axi_wready(m2_wready),
        .m_axi_bresp(m2_bresp), .m_axi_bvalid(m2_bvalid), .m_axi_bready(m2_bready),
        .rd_buf_base(rd2_buf_base), .rd_buf_valid(rd_buf_valid[2]),
        .doppler_frame_done(doppler_frame_done[2]),
        .busy(busy[2]), .stall(stall[2]), .pp_state_out(pp_state_unused[2]));

endmodule

`default_nettype wire
