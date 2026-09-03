// =============================================================================
//  doppler_seq_ctrl.v
//
//  Reads the transposed DDR3 ping-pong buffer as sequential N_CHIRPS-beat bursts
//  and streams each burst into the Doppler FFT input.
//
//  One burst per range bin k = 0..N_RANGE_BINS-1:
//    read_addr = buf_base + k * N_CHIRPS * SAMPLE_BYTES
//    burst length = N_CHIRPS beats of SAMPLE_BYTES each
//
//  ── AXI4 master read interface ────────────────────────────────────────────
//    ARSIZE and ARBURST are tied constant.
//    Connect m_axi_* to SmartConnect → MIG in block design.
//
//  ── Bugs fixed vs previous version ───────────────────────────────────────
//    1. Skid buffer added to S_STREAM.  Previously when rvalid+rready fired
//       but m_axis_tready was low, the current rdata was consumed by the AXI
//       handshake but thrown away.  The skid buffer latches rdata first, then
//       rready is only re-enabled once the skid is drained.
//    2. Stale m_axis_tvalid: added explicit clear in all S_STREAM branches
//       where no new data is forwarded, preventing the FFT from seeing a
//       valid pulse with stale data.
//    3. rd_addr_r initialised at reset to buf_base+0 to avoid X on first burst.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module doppler_seq_ctrl #(
    parameter DATA_W       = 32,
    parameter ADDR_W       = 32,
    parameter N_RANGE_BINS = `RADAR_RANGE_BINS,
    parameter N_CHIRPS     = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES = `RADAR_SAMPLE_BYTES
)(
    input  wire clk,
    input  wire rst_n,

    // ── Ping-pong handshake ───────────────────────────────────────────────
    input  wire [ADDR_W-1:0] buf_base,
    input  wire              buf_valid,
    output reg               doppler_frame_done,

    // ── AXI4 master read address channel (AR) ─────────────────────────────
    output reg  [ADDR_W-1:0] m_axi_araddr,
    output reg  [7:0]        m_axi_arlen,
    output wire [2:0]        m_axi_arsize,
    output wire [1:0]        m_axi_arburst,
    output reg               m_axi_arvalid,
    input  wire              m_axi_arready,

    // ── AXI4 master read data channel (R) ─────────────────────────────────
    input  wire [DATA_W-1:0] m_axi_rdata,
    input  wire [1:0]        m_axi_rresp,
    input  wire              m_axi_rlast,
    input  wire              m_axi_rvalid,
    output reg               m_axi_rready,

    // ── AXI-Stream output → Doppler FFT s_axis_data ───────────────────────
    output reg  [DATA_W-1:0] m_axis_tdata,
    output reg               m_axis_tvalid,
    output reg               m_axis_tlast,
    input  wire              m_axis_tready,

    // ── From Doppler FFT m_axis_data output side ──────────────────────────
    input  wire              fft_out_valid,
    input  wire              fft_out_last,

    // ── Status ────────────────────────────────────────────────────────────
    output reg               busy
);

    assign m_axi_arsize  = 3'd2;
    assign m_axi_arburst = 2'b01;

    localparam S_IDLE      = 3'd0;
    localparam S_ISSUE_AR  = 3'd1;
    localparam S_WAIT_AR   = 3'd2;
    localparam S_STREAM    = 3'd3;
    localparam S_WAIT_FFT  = 3'd4;
    localparam S_NEXT_BIN  = 3'd5;
    localparam S_DONE      = 3'd6;

    localparam K_W = $clog2(N_RANGE_BINS);
    localparam [K_W-1:0]  K_MAX        = N_RANGE_BINS - 1;
    localparam [ADDR_W-1:0] BURST_STRIDE = N_CHIRPS * SAMPLE_BYTES;

    reg [2:0]     state;
    reg [K_W-1:0] k;

    // FIX 1: skid buffer — holds one R-channel beat while waiting for FFT
    // ── Skid buffer ───────────────────────────────────────────────────────
    // The problem: when m_axi_rvalid && m_axi_rready is true, MIG has already 
    // presented m_axi_rdata on the bus and considers that beat delivered. 
    // The AXI4 spec says once rvalid fires with rready high, the data is transferred — 
    // MIG will move to the next beat regardless. De-asserting rready only stalls future beats. 
    // The current beat's data was already on the bus and we threw it away by not latching it.

    // The fix is to latch m_axi_rdata into a skid buffer the moment rvalid && rready fires, 
    // regardless of whether the FFT is ready. Then hold rready low until the FFT consumes the latched data.
    reg [DATA_W-1:0] skid_data;
    reg              skid_valid;
    reg              skid_last;

    // FIX 3: registered burst address — initialised in reset block

    // FIX: compute address combinatorially so S_ISSUE_AR sees the current k,
    // not k from the previous clock cycle (the registered version lagged by 1).
    wire [ADDR_W-1:0] rd_addr_r = buf_base
                                  + ( {{(ADDR_W-K_W){1'b0}}, k} * BURST_STRIDE );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            k                  <= {K_W{1'b0}};
            doppler_frame_done <= 1'b0;
            busy               <= 1'b0;
            m_axi_arvalid      <= 1'b0;
            m_axi_araddr       <= {ADDR_W{1'b0}};
            m_axi_arlen        <= 8'd0;
            m_axi_rready       <= 1'b0;
            m_axis_tvalid      <= 1'b0;
            m_axis_tlast       <= 1'b0;
            m_axis_tdata       <= {DATA_W{1'b0}};
            skid_data          <= {DATA_W{1'b0}};  // FIX 1
            skid_valid         <= 1'b0;            // FIX 1
            skid_last          <= 1'b0;            // FIX 1
        end else begin
            doppler_frame_done <= 1'b0;

            case (state)

                // ── S_IDLE ────────────────────────────────────────────────
                S_IDLE: begin
                    k             <= {K_W{1'b0}};
                    busy          <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    skid_valid    <= 1'b0;
                    if (buf_valid) begin
                        state <= S_ISSUE_AR;
                        busy  <= 1'b1;
                    end
                end

                // ── S_ISSUE_AR ────────────────────────────────────────────
                // rd_addr_r settled during previous state (S_IDLE or S_NEXT_BIN).
                S_ISSUE_AR: begin
                    m_axi_araddr  <= rd_addr_r;
                    m_axi_arlen   <= N_CHIRPS - 1;
                    m_axi_arvalid <= 1'b1;
                    state         <= S_WAIT_AR;
                end

                // ── S_WAIT_AR ─────────────────────────────────────────────
                S_WAIT_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= S_STREAM;
                    end
                end

                // ── S_STREAM ──────────────────────────────────────────────
                // FIX 1: Skid buffer pattern.
                //
                // The AXI4 R channel contract: once rvalid+rready fires, MIG
                // advances its internal pointer.  rdata is only valid that one
                // cycle.  We must capture it immediately.
                //
                // Flow:
                //   Step A — capture from R channel into skid when rvalid+rready
                //            and skid is empty.  Drop rready after capture so
                //            MIG cannot present the next beat until skid drains.
                //   Step B — forward skid to FFT when FFT is ready (tready=1).
                //            When skid drains and we are not at rlast, re-enable
                //            rready to accept the next beat.
                //
                // FIX 2: m_axis_tvalid is explicitly cleared in every branch
                //        where no new data is forwarded, preventing stale valid.
                S_STREAM: begin

                    // ── Step A: capture from R channel ────────────────────
                    if (m_axi_rvalid && m_axi_rready) begin
                        // Capture unconditionally — do not check tready here.
                        // rready being high means we committed to accept this beat.
                        skid_data    <= m_axi_rdata;
                        skid_last    <= m_axi_rlast;
                        skid_valid   <= 1'b1;
                        m_axi_rready <= 1'b0;   // hold until skid is drained
                    end

                    // ── Step B: forward skid to FFT ───────────────────────
                    if (skid_valid) begin
                        if (m_axis_tready) begin
                            // FFT accepts — forward and drain skid
                            m_axis_tdata  <= skid_data;
                            m_axis_tvalid <= 1'b1;
                            m_axis_tlast  <= skid_last;
                            skid_valid    <= 1'b0;

                            if (skid_last) begin
                                // Last beat forwarded — wait for FFT to finish
                                state <= S_WAIT_FFT;
                            end else begin
                                // More beats to come — re-enable R channel
                                m_axi_rready <= 1'b1;
                            end
                        end else begin
                            // FFT not ready — hold skid, tvalid stays low
                            // (do not pulse tvalid while FFT is not ready)
                            m_axis_tvalid <= 1'b0;  // FIX 2
                        end
                    end else begin
                        // Skid empty — no data to forward
                        m_axis_tvalid <= 1'b0;  // FIX 2
                        // rready was already re-enabled when skid drained above,
                        // or will be set when M step B executes next cycle.
                        // If rready is currently low (first entry into S_STREAM
                        // after S_WAIT_AR did not pre-enable it here), enable now.
                        // Note: rready was set in S_WAIT_AR so this path is
                        // only hit after skid drains — rready is already high.
                    end
                end

                // ── S_WAIT_FFT ────────────────────────────────────────────
                S_WAIT_FFT: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    if (fft_out_valid && fft_out_last)
                        state <= S_NEXT_BIN;
                end

                // ── S_NEXT_BIN ────────────────────────────────────────────
                S_NEXT_BIN: begin
                    if (k < K_MAX) begin
                        k     <= k + 1'b1;
                        state <= S_ISSUE_AR;
                    end else begin
                        state <= S_DONE;
                    end
                end

                // ── S_DONE ────────────────────────────────────────────────
                S_DONE: begin
                    doppler_frame_done <= 1'b1;
                    busy               <= 1'b0;
                    state              <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
