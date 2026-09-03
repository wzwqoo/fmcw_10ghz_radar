// =============================================================================
//  cplx_cell_cache_top.v   *** 3-RX VARIANT ***
//
//  Wraps three cplx_cell_writer instances, one per RX antenna.  All three write
//  per-receiver complex range-Doppler maps in DDR3 region B through separate
//  AXI4 master ports.  The AXI SmartConnect / Interconnect arbitrates between
//  them on the MIG side.
//
//  Why four masters instead of one TDM master
//  ------------------------------------------
//  - Lowest-risk path: each writer is identical to rd_map_collector_summed,
//    a proven pattern.  No on-chip BRAM reshuffling.
//  - Each Doppler-FFT IP instance is already an independent AXI-Stream
//    source, so per-RX writers map 1:1 onto IP outputs.
//  - AXI throughput cost is negligible — at the radar's chirp rate the four
//    writers combined consume < 5% of typical MIG bandwidth.
//
//  Collapse to a single shared writer later if MIG bandwidth becomes a
//  bottleneck (e.g. with higher PRF or wider IQ).
//
//  Memory layout:
//  --------------
//  The complex RD cache is region B of the B/C/D frame bundle.  It is now
//  ping-ponged as B0/B1 so MicroBlaze can read one complete frame bundle while
//  PL writes the next bundle.  This wrapper does not decide ownership; it only
//  forwards frame_buf_sel to all four per-RX writers.
//
//      frame_buf_sel = 0 -> B0 at RADAR_DDR_B0_BASE
//      frame_buf_sel = 1 -> B1 at RADAR_DDR_B1_BASE
//
//  For the selected B set:
//      RX0 map base = Bx + 0 * RADAR_RD_MAP_BYTES
//      RX1 map base = Bx + 1 * RADAR_RD_MAP_BYTES
//      RX2 map base = Bx + 2 * RADAR_RD_MAP_BYTES
//
//  The same frame_buf_sel must also select Cx in rd_map_collector_summed and
//  Dx in cfar_streaming_axi_verilog.  Top-level/BD logic must publish a frame
//  to software only after Bx, Cx, and Dx are all complete.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module cplx_cell_cache_top #(
    parameter DATA_W       = 32,
    parameter ADDR_W       = 32,
    parameter N_RANGE_BINS = `RADAR_RANGE_BINS,
    parameter N_CHIRPS     = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES = `RADAR_SAMPLE_BYTES,
    parameter COMPLEX_BASE0 = `RADAR_DDR_B0_BASE,
    parameter COMPLEX_BASE1 = `RADAR_DDR_B1_BASE
)(
    input  wire clk,
    input  wire rst_n,

    // Selects B0/B1 for the complete 3-RX complex RD frame.  This must match
    // the C and D buffer selection for the same radar frame.
    input  wire frame_buf_sel,

    // ── 3 RX AXI-Stream inputs (complex IQ from Doppler FFTs) ─────────────
    input  wire [DATA_W-1:0] s0_tdata, input  wire s0_tvalid, input  wire s0_tlast, output wire s0_tready,
    input  wire [DATA_W-1:0] s1_tdata, input  wire s1_tvalid, input  wire s1_tlast, output wire s1_tready,
    input  wire [DATA_W-1:0] s2_tdata, input  wire s2_tvalid, input  wire s2_tlast, output wire s2_tready,

    // ── 3 AXI4 write masters (flatten if your interconnect prefers it) ────
    output wire [ADDR_W-1:0]   m0_awaddr,  output wire [7:0] m0_awlen,  output wire [2:0] m0_awsize,
    output wire [1:0]          m0_awburst, output wire       m0_awvalid, input  wire       m0_awready,
    output wire [DATA_W-1:0]   m0_wdata,   output wire [DATA_W/8-1:0] m0_wstrb,
    output wire                m0_wlast,   output wire       m0_wvalid,  input  wire       m0_wready,
    input  wire [1:0]          m0_bresp,   input  wire       m0_bvalid,  output wire       m0_bready,

    output wire [ADDR_W-1:0]   m1_awaddr,  output wire [7:0] m1_awlen,  output wire [2:0] m1_awsize,
    output wire [1:0]          m1_awburst, output wire       m1_awvalid, input  wire       m1_awready,
    output wire [DATA_W-1:0]   m1_wdata,   output wire [DATA_W/8-1:0] m1_wstrb,
    output wire                m1_wlast,   output wire       m1_wvalid,  input  wire       m1_wready,
    input  wire [1:0]          m1_bresp,   input  wire       m1_bvalid,  output wire       m1_bready,

    output wire [ADDR_W-1:0]   m2_awaddr,  output wire [7:0] m2_awlen,  output wire [2:0] m2_awsize,
    output wire [1:0]          m2_awburst, output wire       m2_awvalid, input  wire       m2_awready,
    output wire [DATA_W-1:0]   m2_wdata,   output wire [DATA_W/8-1:0] m2_wstrb,
    output wire                m2_wlast,   output wire       m2_wvalid,  input  wire       m2_wready,
    input  wire [1:0]          m2_bresp,   input  wire       m2_bvalid,  output wire       m2_bready,

    // ── Aggregate status ──────────────────────────────────────────────────
    output wire              busy,        // OR of per-writer busy
    output reg               cache_ready  // pulses 1 cycle after all 3 done
);

    wire [2:0] busy_v;
    wire [2:0] done_v;

    assign busy = |busy_v;

    // cache_ready: pulse when all 3 done strobes have fired since the last
    // pulse.  A simple "frame done counter" handles arbitrary skew between
    // writers (they finish independently because of AXI interconnect timing).
    reg [2:0] done_sticky;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_sticky <= 3'd0;
            cache_ready <= 1'b0;
        end else begin
            cache_ready <= 1'b0;
            if (&(done_sticky | done_v)) begin
                cache_ready <= 1'b1;
                done_sticky <= 3'd0;
            end else begin
                done_sticky <= done_sticky | done_v;
            end
        end
    end

    cplx_cell_writer #(.RX_INDEX(0), .COMPLEX_BASE0(COMPLEX_BASE0), .COMPLEX_BASE1(COMPLEX_BASE1),
                       .DATA_W(DATA_W), .ADDR_W(ADDR_W),
                       .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
                       .SAMPLE_BYTES(SAMPLE_BYTES))
    u_w0 (.clk(clk), .rst_n(rst_n), .frame_buf_sel(frame_buf_sel),
          .s_axis_tdata(s0_tdata), .s_axis_tvalid(s0_tvalid),
          .s_axis_tlast(s0_tlast), .s_axis_tready(s0_tready),
          .m_axi_awaddr(m0_awaddr), .m_axi_awlen(m0_awlen),
          .m_axi_awsize(m0_awsize), .m_axi_awburst(m0_awburst),
          .m_axi_awvalid(m0_awvalid), .m_axi_awready(m0_awready),
          .m_axi_wdata(m0_wdata), .m_axi_wstrb(m0_wstrb),
          .m_axi_wlast(m0_wlast), .m_axi_wvalid(m0_wvalid),
          .m_axi_wready(m0_wready),
          .m_axi_bresp(m0_bresp), .m_axi_bvalid(m0_bvalid),
          .m_axi_bready(m0_bready),
          .busy(busy_v[0]), .frame_done(done_v[0]));

    cplx_cell_writer #(.RX_INDEX(1), .COMPLEX_BASE0(COMPLEX_BASE0), .COMPLEX_BASE1(COMPLEX_BASE1),
                       .DATA_W(DATA_W), .ADDR_W(ADDR_W),
                       .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
                       .SAMPLE_BYTES(SAMPLE_BYTES))
    u_w1 (.clk(clk), .rst_n(rst_n), .frame_buf_sel(frame_buf_sel),
          .s_axis_tdata(s1_tdata), .s_axis_tvalid(s1_tvalid),
          .s_axis_tlast(s1_tlast), .s_axis_tready(s1_tready),
          .m_axi_awaddr(m1_awaddr), .m_axi_awlen(m1_awlen),
          .m_axi_awsize(m1_awsize), .m_axi_awburst(m1_awburst),
          .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready),
          .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb),
          .m_axi_wlast(m1_wlast), .m_axi_wvalid(m1_wvalid),
          .m_axi_wready(m1_wready),
          .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid),
          .m_axi_bready(m1_bready),
          .busy(busy_v[1]), .frame_done(done_v[1]));

    cplx_cell_writer #(.RX_INDEX(2), .COMPLEX_BASE0(COMPLEX_BASE0), .COMPLEX_BASE1(COMPLEX_BASE1),
                       .DATA_W(DATA_W), .ADDR_W(ADDR_W),
                       .N_RANGE_BINS(N_RANGE_BINS), .N_CHIRPS(N_CHIRPS),
                       .SAMPLE_BYTES(SAMPLE_BYTES))
    u_w2 (.clk(clk), .rst_n(rst_n), .frame_buf_sel(frame_buf_sel),
          .s_axis_tdata(s2_tdata), .s_axis_tvalid(s2_tvalid),
          .s_axis_tlast(s2_tlast), .s_axis_tready(s2_tready),
          .m_axi_awaddr(m2_awaddr), .m_axi_awlen(m2_awlen),
          .m_axi_awsize(m2_awsize), .m_axi_awburst(m2_awburst),
          .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready),
          .m_axi_wdata(m2_wdata), .m_axi_wstrb(m2_wstrb),
          .m_axi_wlast(m2_wlast), .m_axi_wvalid(m2_wvalid),
          .m_axi_wready(m2_wready),
          .m_axi_bresp(m2_bresp), .m_axi_bvalid(m2_bvalid),
          .m_axi_bready(m2_bready),
          .busy(busy_v[2]), .frame_done(done_v[2]));

endmodule

`default_nettype wire
