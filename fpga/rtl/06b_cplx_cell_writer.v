// =============================================================================
//  cplx_cell_writer.v
//
//  Writes one RX channel of complex Doppler-FFT output to DDR3 region B.
//  Identical N_CHIRPS-beat-burst-per-range-bin pattern to rd_map_collector, but
//  without the CFAR handshake.  Four instances of this module (one per RX)
//  populate the per-receiver complex range-Doppler maps that the AoA path
//  later reads at detection-peak addresses.
//
//  Memory layout (region B0/B1):
//  -----------------------------
//  This writer now supports two complete B/C/D post-Doppler frame sets.
//  frame_buf_sel is latched at the first stream sample of a frame and selects
//  which complex-map set receives the complete frame:
//
//      frame_buf_sel = 0 -> COMPLEX_BASE0 = RADAR_DDR_B0_BASE
//      frame_buf_sel = 1 -> COMPLEX_BASE1 = RADAR_DDR_B1_BASE
//
//      addr(buf, rx, k, n) = COMPLEX_BASE[buf]
//                          + rx * RX_STRIDE
//                          + k  * BURST_STRIDE
//                          + n  * SAMPLE_BYTES
//      where:
//          buf         : 0 or 1, selected for the full frame
//          rx          : 0..3
//          k           : 0..N_RANGE_BINS-1
//          n           : 0..N_CHIRPS-1
//          BURST_STRIDE = N_CHIRPS * SAMPLE_BYTES                 (one range row)
//          RX_STRIDE    = N_RANGE_BINS * N_CHIRPS * SAMPLE_BYTES  (one full map)
//
//  Each beat carries one complex sample: I in upper IQ_W bits, Q in lower IQ_W.
//
//  Synchronization rule:
//      This module only writes B[buf].  The top-level/BD control must use the
//      same buf for C[buf] and D[buf], publish the frame only after all three
//      products are complete, and hold/release ownership with MicroBlaze.
//
//  Frame handshake
//  ---------------
//  The instance asserts frame_done for one cycle after its last B response,
//  then returns to S_IDLE.  cplx_cell_cache_top collects all four frame_done
//  pulses to produce a single cache_ready signal for downstream.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module cplx_cell_writer #(
    parameter DATA_W       = 32,
    parameter ADDR_W       = 32,
    parameter N_RANGE_BINS = `RADAR_RANGE_BINS,
    parameter N_CHIRPS     = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES  = `RADAR_SAMPLE_BYTES,
    parameter RX_INDEX      = 0,                     // 0..3
    parameter COMPLEX_BASE0 = `RADAR_DDR_B0_BASE,    // DDR3 B0: per-RX RD map base
    parameter COMPLEX_BASE1 = `RADAR_DDR_B1_BASE,    // DDR3 B1: per-RX RD map base
    parameter RX_STRIDE     = N_RANGE_BINS * N_CHIRPS * SAMPLE_BYTES
)(
    input  wire clk,
    input  wire rst_n,

    // Selects B0 or B1 for the complete frame.  The value is latched when a
    // frame starts so a top-level change cannot split one frame across sets.
    input  wire frame_buf_sel,

    // ── AXI-Stream input — one RX of Doppler FFT (complex IQ) ─────────────
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
    output reg                 m_axi_wlast,
    output reg                 m_axi_wvalid,
    input  wire                m_axi_wready,

    // ── AXI4 master write response channel (B) ────────────────────────────
    input  wire [1:0]        m_axi_bresp,
    input  wire              m_axi_bvalid,
    output wire              m_axi_bready,

    // ── Status ────────────────────────────────────────────────────────────
    output reg               busy,
    output reg               frame_done
);

    assign m_axi_awsize  = 3'd2;
    assign m_axi_awburst = 2'b01;
    assign m_axi_wstrb   = {(DATA_W/8){1'b1}};
    assign m_axi_bready  = 1'b1;

    localparam S_IDLE     = 3'd0;
    localparam S_ISSUE_AW = 3'd1;
    localparam S_WAIT_AW  = 3'd2;
    localparam S_WRITE    = 3'd3;
    localparam S_WAIT_B   = 3'd4;
    localparam S_NEXT_BIN = 3'd5;

    localparam K_W  = $clog2(N_RANGE_BINS);
    localparam BT_W = $clog2(N_CHIRPS);

    localparam [K_W-1:0]    K_MAX        = N_RANGE_BINS - 1;
    localparam [BT_W-1:0]   BEAT_LAST    = N_CHIRPS - 1;
    localparam [ADDR_W-1:0] BURST_STRIDE = N_CHIRPS * SAMPLE_BYTES;
    localparam [ADDR_W-1:0] RX_OFFSET    = RX_INDEX * RX_STRIDE;

    reg                    active_buf_sel;
    wire [ADDR_W-1:0]      selected_complex_base;
    wire [ADDR_W-1:0]      base_plus_rx;
    wire [ADDR_W-1:0]      wr_addr_w;

    assign selected_complex_base = active_buf_sel ? COMPLEX_BASE1 : COMPLEX_BASE0;
    assign base_plus_rx          = selected_complex_base + RX_OFFSET;

    reg [2:0]      state;
    reg [K_W-1:0]  k;
    reg [BT_W-1:0] beat_cnt;

    assign wr_addr_w   = base_plus_rx
                       + ( {{(ADDR_W-K_W){1'b0}}, k} * BURST_STRIDE );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            k             <= {K_W{1'b0}};
            beat_cnt      <= {BT_W{1'b0}};
            busy          <= 1'b0;
            frame_done    <= 1'b0;
            s_axis_tready <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_awaddr  <= {ADDR_W{1'b0}};
            m_axi_awlen   <= 8'd0;
            m_axi_wvalid  <= 1'b0;
            m_axi_wlast   <= 1'b0;
            m_axi_wdata   <= {DATA_W{1'b0}};
            active_buf_sel <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    k             <= {K_W{1'b0}};
                    beat_cnt      <= {BT_W{1'b0}};
                    busy          <= 1'b0;
                    s_axis_tready <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_wlast   <= 1'b0;
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
                    m_axi_wlast   <= 1'b0;
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
                        m_axi_wlast  <= (beat_cnt == BEAT_LAST);
                        beat_cnt     <= beat_cnt + 1'b1;
                        if (beat_cnt == BEAT_LAST) begin
                            s_axis_tready <= 1'b0;
                            state         <= S_WAIT_B;
                        end
                    end else if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                    end
                end

                S_WAIT_B: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                    end
                    if (m_axi_bvalid)
                        state <= S_NEXT_BIN;
                end

                S_NEXT_BIN: begin
                    if (k < K_MAX) begin
                        k     <= k + 1'b1;
                        state <= S_ISSUE_AW;
                    end else begin
                        k          <= {K_W{1'b0}};
                        frame_done <= 1'b1;
                        busy       <= 1'b0;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
