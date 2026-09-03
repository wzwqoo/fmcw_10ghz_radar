// =============================================================================
//  rd_map_collector_summed.v
//
//  Modified version of rd_map_collector.v for the new pipeline.
//
//  WHAT CHANGED vs rd_map_collector.v
//  ----------------------------------
//  The original module accepted a complex (I,Q) Doppler FFT stream and wrote
//  it to DDR3.  In the new pipeline this role is split:
//
//      cplx_cell_cache_top      → writes 4× per-RX RD maps to DDR3.B (for AoA)
//      noncoh_integrator        → folds 4 RX into one |X|^2 stream
//      rd_map_collector_summed  → writes that single |X|^2 stream to DDR3.C
//
//  So the input here is now UNSIGNED magnitude-squared, not complex.  The AXI
//  beat layout is unchanged (DATA_W bits / sample, one sample per beat, N_CHIRPS beats
//  per range-bin burst), so the FSM is identical to the original.  Only the
//  semantics, the comments, and the cell_t in cfar_detector.h change.
//
//  cell_t in cfar_detector.h
//  -------------------------
//  Original was ap_uint<16>.  After summing 4 channels of (I^2 + Q^2) the
//  worst case is 4 * 2 * (2^15)^2 = 2^33, which does NOT fit in 16 bits.
//  Two options:
//    (a) Widen cell_t to ap_uint<32> and let noncoh_integrator output 32-bit
//        saturated values.  Easiest path — DDR3 stays 32-bit per sample.
//        Re-check acc_t and thresh_t headroom in cfar_detector.h (the file
//        already documents the recalculation).
//    (b) Pre-scale by right-shifting inside noncoh_integrator to keep the
//        16-bit interface.  Loses 16 LSBs of dynamic range.
//  Default below is option (a): SAMPLE_BYTES from radar_params.vh, DATA_W=32, unsigned.
//
//  Memory layout: C0/C1 summed RD power map
//  -----------------------------------------
//  This collector writes region C of the B/C/D frame bundle.  It supports two
//  C buffers selected by frame_buf_sel:
//
//      frame_buf_sel = 0 -> C0 at RD_MAP_BASE0 = RADAR_DDR_C0_BASE
//      frame_buf_sel = 1 -> C1 at RD_MAP_BASE1 = RADAR_DDR_C1_BASE
//
//      addr(buf, k, n) = C_BASE[buf]
//                      + k * (N_CHIRPS * SAMPLE_BYTES)
//                      + n * SAMPLE_BYTES
//
//  C is the summed noncoherent power map:
//      C[k,n] = Σ over RX of (I_rx[k,n]^2 + Q_rx[k,n]^2)
//
//  CFAR reads C[buf] and writes D[buf].  MicroBlaze later reuses C[buf] for
//  detection ranking and sub-bin range/Doppler refinement.  C is not a CFAR
//  output; it is the CFAR input image.
//
//  cfar_start handshake:
//      cfar_start pulses only after the full C[buf] map is written.
//      cfar_frame_buf_sel exposes the same latched buffer index used for the
//      C write addresses, so CFAR reads C[buf] and writes D[buf].
//
//  Notes
//  -----
//    1. m_axi_wlast is registered with m_axi_wdata.  The beat counter is
//       advanced when a stream sample is accepted, so deriving WLAST directly
//       from the live counter asserts it one beat early at row boundaries.
//    2. Address generation is combinational from the latched C0/C1 base.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module rd_map_collector_summed #(
    parameter DATA_W       = 32,           // matches noncoh_integrator OUT_W
    parameter ADDR_W       = 32,
    parameter N_RANGE_BINS = `RADAR_RANGE_BINS,
    parameter N_CHIRPS     = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES = `RADAR_SAMPLE_BYTES, // DATA_W/8 — keep consistent
    parameter RD_MAP_BASE0 = `RADAR_DDR_C0_BASE,  // DDR3 C0: summed power map
    parameter RD_MAP_BASE1 = `RADAR_DDR_C1_BASE   // DDR3 C1: summed power map
)(
    input  wire clk,
    input  wire rst_n,

    // Selects C0 or C1 for the complete summed RD frame.  The value is latched
    // when a frame starts, then forwarded to CFAR with cfar_start.
    input  wire frame_buf_sel,

    // ── AXI-Stream input — noncoh_integrator m_axis ────────────────────────
    //   tdata is UNSIGNED magnitude-squared (was complex IQ in the original)
    input  wire [DATA_W-1:0] s_axis_tdata,
    input  wire              s_axis_tvalid,
    input  wire              s_axis_tlast,
    output reg               s_axis_tready,

    // ── AXI4 master write address channel (AW) ────────────────────────────
    output reg  [ADDR_W-1:0] m_axi_awaddr,
    output reg  [7:0]        m_axi_awlen,
    output wire [2:0]        m_axi_awsize,
    output wire [1:0]        m_axi_awburst,
    output reg               m_axi_awvalid,
    input  wire              m_axi_awready,

    // ── AXI4 master write data channel (W) ────────────────────────────────
    output reg  [DATA_W-1:0]   m_axi_wdata,
    output wire [DATA_W/8-1:0] m_axi_wstrb,
    output wire                m_axi_wlast,
    output reg                 m_axi_wvalid,
    input  wire                m_axi_wready,

    // ── AXI4 master write response channel (B) ────────────────────────────
    input  wire [1:0]        m_axi_bresp,
    input  wire              m_axi_bvalid,
    output wire              m_axi_bready,

    // ── CFAR HLS IP handshake ─────────────────────────────────────────────
    output reg               cfar_start,
    output wire              cfar_frame_buf_sel,
    input  wire              cfar_done,

    // ── Status ────────────────────────────────────────────────────────────
    output reg               busy,
    output reg               frame_complete
);

    // ── Tied constant AXI4 signals ────────────────────────────────────────
    assign m_axi_awsize  = 3'd2;          // 4 bytes per beat (DATA_W=32)
    assign m_axi_awburst = 2'b01;         // INCR
    assign m_axi_wstrb   = {(DATA_W/8){1'b1}};
    assign m_axi_bready  = 1'b1;

    // ── State encoding ────────────────────────────────────────────────────
    localparam S_IDLE      = 3'd0;
    localparam S_ISSUE_AW  = 3'd1;
    localparam S_WAIT_AW   = 3'd2;
    localparam S_WRITE     = 3'd3;
    localparam S_WAIT_B    = 3'd4;
    localparam S_NEXT_BIN  = 3'd5;
    localparam S_KICK_CFAR = 3'd6;
    localparam S_WAIT_CFAR = 3'd7;

    localparam K_W  = $clog2(N_RANGE_BINS);
    localparam BT_W = $clog2(N_CHIRPS);

    localparam [K_W-1:0]    K_MAX        = N_RANGE_BINS - 1;
    localparam [BT_W-1:0]   BEAT_LAST    = N_CHIRPS - 1;
    localparam [ADDR_W-1:0] BURST_STRIDE = N_CHIRPS * SAMPLE_BYTES;

    reg                    active_buf_sel;
    wire [ADDR_W-1:0]      selected_rd_base;
    wire [ADDR_W-1:0]      wr_addr_w;
    reg                    m_axi_wlast_r;

    assign selected_rd_base = active_buf_sel ? RD_MAP_BASE1 : RD_MAP_BASE0;
    assign cfar_frame_buf_sel = active_buf_sel;
    assign m_axi_wlast = m_axi_wlast_r;

    reg [2:0]      state;
    reg [K_W-1:0]  k;
    reg [BT_W-1:0] beat_cnt;

    assign wr_addr_w   = selected_rd_base
                       + ( {{(ADDR_W-K_W){1'b0}}, k} * BURST_STRIDE );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            k              <= {K_W{1'b0}};
            beat_cnt       <= {BT_W{1'b0}};
            busy           <= 1'b0;
            cfar_start     <= 1'b0;
            frame_complete <= 1'b0;
            s_axis_tready  <= 1'b0;
            m_axi_awvalid  <= 1'b0;
            m_axi_awaddr   <= {ADDR_W{1'b0}};
            m_axi_awlen    <= 8'd0;
            m_axi_wvalid   <= 1'b0;
            m_axi_wdata    <= {DATA_W{1'b0}};
            m_axi_wlast_r  <= 1'b0;
            active_buf_sel <= 1'b0;
        end else begin
            cfar_start     <= 1'b0;
            frame_complete <= 1'b0;

            case (state)
                S_IDLE: begin
                    k             <= {K_W{1'b0}};
                    beat_cnt      <= {BT_W{1'b0}};
                    busy          <= 1'b0;
                    s_axis_tready <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_wlast_r <= 1'b0;
                    if (s_axis_tvalid) begin
                        active_buf_sel <= frame_buf_sel;
                        state          <= S_ISSUE_AW;
                        busy           <= 1'b1;
                    end
                end

                S_ISSUE_AW: begin
                    m_axi_awaddr  <= wr_addr_w;
                    m_axi_awlen   <= N_CHIRPS - 1;
                    m_axi_awvalid <= 1'b1;
                    beat_cnt      <= {BT_W{1'b0}};
                    state         <= S_WAIT_AW;
                end

                S_WAIT_AW: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        s_axis_tready <= 1'b1;
                        state         <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    if (s_axis_tvalid && s_axis_tready
                        && (!m_axi_wvalid || m_axi_wready)) begin

                        m_axi_wdata  <= s_axis_tdata;
                        m_axi_wvalid <= 1'b1;
                        m_axi_wlast_r <= (beat_cnt == BEAT_LAST);
                        beat_cnt     <= beat_cnt + 1'b1;

                        if (beat_cnt == BEAT_LAST) begin
                            s_axis_tready <= 1'b0;
                            state         <= S_WAIT_B;
                        end
                    end else if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid  <= 1'b0;
                        m_axi_wlast_r <= 1'b0;
                    end
                end

                S_WAIT_B: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid  <= 1'b0;
                        m_axi_wlast_r <= 1'b0;
                    end
                    if (m_axi_bvalid)
                        state <= S_NEXT_BIN;
                end

                S_NEXT_BIN: begin
                    if (k < K_MAX) begin
                        k     <= k + 1'b1;
                        state <= S_ISSUE_AW;
                    end else begin
                        k     <= {K_W{1'b0}};
                        state <= S_KICK_CFAR;
                    end
                end

                S_KICK_CFAR: begin
                    cfar_start <= 1'b1;
                    state      <= S_WAIT_CFAR;
                end

                S_WAIT_CFAR: begin
                    if (cfar_done) begin
                        frame_complete <= 1'b1;
                        busy           <= 1'b0;
                        state          <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
