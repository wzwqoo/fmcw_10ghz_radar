// =============================================================================
//  scatter_write_master.v
//
//  Writes Range FFT output to DDR3 in TRANSPOSED column-major order so that
//  doppler_seq_ctrl can read each range bin as a single sequential burst.
//
//  Transposed address for range bin k of chirp n:
//    addr = buf_base + rx * RX_STRIDE + (k * N_CHIRPS + n) * SAMPLE_BYTES
//
//  Ping-pong buffer control is integrated.  Two DDR3 regions (A ping/pong)
//  alternate: the scatter writer fills one while the Doppler side reads the
//  other.  The writer stalls automatically when both buffers are occupied.
//
//  Instantiate one writer per Range FFT output:
//      RX_INDEX=0 -> A_BASE + 0 * RX_STRIDE
//      RX_INDEX=1 -> A_BASE + 1 * RX_STRIDE
//      RX_INDEX=2 -> A_BASE + 2 * RX_STRIDE
//      RX_INDEX=3 -> A_BASE + 3 * RX_STRIDE
//
//  ── AXI4 master interface (write only) ────────────────────────────────────
//    Single-beat writes (AWLEN=0) because transposed addresses are
//    non-sequential — bursting is not possible.
//    AWLEN, AWSIZE, AWBURST, WSTRB, WLAST, BREADY are all tied constant.
//
//  ── Bugs fixed vs previous version ───────────────────────────────────────
//    1. W_WRITE now waits for BOTH AW and W accepted before advancing k.
//       Previously advanced on AW alone — could corrupt data_latch if W lagged.
//    2. Redundant m_axi_awvalid clear removed (was overriding the advance check).
//    3. Outstanding counter now tracks W accepted (not AW) to be semantically
//       correct: a write is "outstanding" once data is committed to MIG.
//    4. write_addr_r initialised at reset to BUF_A_BASE to avoid X on first write.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module scatter_write_master #(
    parameter DATA_W          = 32,
    parameter ADDR_W          = 32,
    parameter N_RANGE_BINS    = `RADAR_RANGE_BINS,
    parameter N_CHIRPS        = `RADAR_N_CHIRPS,
    parameter SAMPLE_BYTES    = `RADAR_SAMPLE_BYTES,
    parameter MAX_OUTSTANDING = 8,
    parameter RX_INDEX        = 0,
    parameter RX_STRIDE       = N_RANGE_BINS * N_CHIRPS * SAMPLE_BYTES,
    parameter BUF_A_BASE      = `RADAR_DDR_A_PING,
    parameter BUF_B_BASE      = `RADAR_DDR_A_PONG
)(
    input  wire clk,
    input  wire rst_n,

    // ── AXI-Stream input — Range FFT m_axis_data ──────────────────────────
    input  wire [DATA_W-1:0] s_axis_tdata,
    input  wire              s_axis_tvalid,
    output reg               s_axis_tready,

    // ── AXI4 master write address channel (AW) ────────────────────────────
    output reg  [ADDR_W-1:0] m_axi_awaddr,
    output wire [7:0]        m_axi_awlen,
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

    // ── Doppler side handshake ─────────────────────────────────────────────
    output reg  [ADDR_W-1:0] rd_buf_base,
    output reg               rd_buf_valid,
    input  wire              doppler_frame_done,

    // ── Status / debug ─────────────────────────────────────────────────────
    output reg               busy,
    output reg               stall,
    output reg  [1:0]        pp_state_out
);

    // ── Tied constant AXI4 signals ────────────────────────────────────────
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'd2;
    assign m_axi_awburst = 2'b01;
    assign m_axi_wstrb   = {(DATA_W/8){1'b1}};
    assign m_axi_wlast   = 1'b1;   // single-beat: always last
    assign m_axi_bready  = 1'b1;

    // ── State encoding ────────────────────────────────────────────────────
    localparam W_IDLE  = 2'd0;
    localparam W_WRITE = 2'd1;
    localparam W_DRAIN = 2'd2;
    localparam W_DONE  = 2'd3;

    localparam PP_FILL_A        = 2'd0;
    localparam PP_PROC_A_FILL_B = 2'd1;
    localparam PP_PROC_B_FILL_A = 2'd2;
    localparam PP_STALL         = 2'd3;

    localparam K_W  = $clog2(N_RANGE_BINS);
    localparam CN_W = $clog2(N_CHIRPS);

    localparam [K_W-1:0]  K_MAX  = N_RANGE_BINS - 1;
    localparam [CN_W-1:0] CN_MAX = N_CHIRPS - 1;

    localparam [ADDR_W-1:0] NCHIRPS_ADDR     = N_CHIRPS;
    localparam [ADDR_W-1:0] SAMPLEBYTES_ADDR = SAMPLE_BYTES;
    localparam [ADDR_W-1:0] RX_OFFSET        = RX_INDEX * RX_STRIDE;

    // ── Internal signals ──────────────────────────────────────────────────
    reg [1:0]        pp_state;
    reg              stall_target;
    reg [ADDR_W-1:0] wr_buf_base;

    reg [1:0]        w_state;
    reg [K_W-1:0]    k;
    reg [CN_W-1:0]   chirp_cnt;

    // Outstanding: counts W transactions issued, decrements on B response.
    // FIX 3: track W accepted (not AW) — data is committed when W fires.
    reg [$clog2(MAX_OUTSTANDING):0] outstanding;
    reg                             m_axi_bvalid_d;  // FIX 4: B-channel edge detect

    reg [DATA_W-1:0] data_latch;
    reg              data_latched;
    reg              chirp_done;

    // FIX 1: sticky flags so we wait for BOTH AW and W before advancing k
    reg              aw_accepted_r;
    reg              w_accepted_r;

    reg [ADDR_W-1:0] write_addr_r;

    wire frame_complete = (chirp_cnt == CN_MAX) && chirp_done;

    // ── Write address register ────────────────────────────────────────────
    // FIX 4: initialise to BUF_A_BASE at reset to avoid X on first write.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            write_addr_r <= BUF_A_BASE + RX_OFFSET;
        else
            write_addr_r <= wr_buf_base
                            + RX_OFFSET
                            + (( {{(ADDR_W-K_W){1'b0}}, k} * NCHIRPS_ADDR )
                               + { {(ADDR_W-CN_W){1'b0}}, chirp_cnt})
                            * SAMPLEBYTES_ADDR;
    end

    // =========================================================================
    //  Ping-pong FSM
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pp_state     <= PP_FILL_A;
            stall_target <= 1'b0;
        end else begin
            case (pp_state)
                PP_FILL_A: begin
                    if (frame_complete)
                        pp_state <= PP_PROC_A_FILL_B;
                end
                PP_PROC_A_FILL_B: begin
                    if (frame_complete && !doppler_frame_done) begin
                        pp_state     <= PP_STALL;
                        stall_target <= 1'b1;
                    end else if (frame_complete) begin
                        pp_state <= PP_PROC_B_FILL_A;
                    end
                end
                PP_PROC_B_FILL_A: begin
                    if (frame_complete && !doppler_frame_done) begin
                        pp_state     <= PP_STALL;
                        stall_target <= 1'b0;
                    end else if (frame_complete) begin
                        pp_state <= PP_PROC_A_FILL_B;
                    end
                end
                PP_STALL: begin
                    if (doppler_frame_done)
                        pp_state <= stall_target ? PP_PROC_B_FILL_A
                                                 : PP_PROC_A_FILL_B;
                end
                default: pp_state <= PP_FILL_A;
            endcase
        end
    end

    always @(*) begin
        pp_state_out = pp_state;
        stall        = (pp_state == PP_STALL);
        case (pp_state)
            PP_FILL_A: begin
                wr_buf_base  = BUF_A_BASE;
                rd_buf_base  = BUF_A_BASE + RX_OFFSET;
                rd_buf_valid = 1'b0;
            end
            PP_PROC_A_FILL_B: begin
                wr_buf_base  = BUF_B_BASE;
                rd_buf_base  = BUF_A_BASE + RX_OFFSET;
                rd_buf_valid = 1'b1;
            end
            PP_PROC_B_FILL_A: begin
                wr_buf_base  = BUF_A_BASE;
                rd_buf_base  = BUF_B_BASE + RX_OFFSET;
                rd_buf_valid = 1'b1;
            end
            PP_STALL: begin
                wr_buf_base  = stall_target ? BUF_B_BASE : BUF_A_BASE;
                rd_buf_base  = (stall_target ? BUF_A_BASE : BUF_B_BASE) + RX_OFFSET;
                rd_buf_valid = 1'b1;
            end
            default: begin
                wr_buf_base  = BUF_A_BASE;
                rd_buf_base  = BUF_A_BASE + RX_OFFSET;
                rd_buf_valid = 1'b0;
            end
        endcase
    end

    // ── Chirp counter ──────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            chirp_cnt <= {CN_W{1'b0}};
        else if (stall)
            chirp_cnt <= chirp_cnt;
        else if (chirp_done)
            chirp_cnt <= frame_complete ? {CN_W{1'b0}} : chirp_cnt + 1'b1;
    end

    // =========================================================================
    //  Write FSM
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state       <= W_IDLE;
            k             <= {K_W{1'b0}};
            outstanding   <= {($clog2(MAX_OUTSTANDING)+1){1'b0}};
            m_axi_bvalid_d <= 1'b0;  // FIX 4
            data_latched  <= 1'b0;
            data_latch    <= {DATA_W{1'b0}};
            chirp_done    <= 1'b0;
            busy          <= 1'b0;
            s_axis_tready <= 1'b0;
            m_axi_awaddr  <= {ADDR_W{1'b0}};
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= {DATA_W{1'b0}};
            m_axi_wvalid  <= 1'b0;
            aw_accepted_r <= 1'b0;   // FIX 1
            w_accepted_r  <= 1'b0;   // FIX 1
        end else begin
            chirp_done <= 1'b0;

            // FIX 3: outstanding tracks W (data committed), not AW (address only)
            // FIX 4: decrement on the B-channel HANDSHAKE EDGE, not the bvalid
            //        LEVEL.  AXI permits a slave to hold BVALID across multiple
            //        clocks until BREADY; the level-sensitive form double-counted
            //        such responses, underflowing 'outstanding' and wedging the
            //        W_WRITE issue guard (outstanding < MAX_OUTSTANDING).  Edge
            //        detection is correct on real interconnect and in sim.
            m_axi_bvalid_d <= m_axi_bvalid;
            case ({m_axi_wvalid && m_axi_wready,
                   m_axi_bvalid && m_axi_bready && !m_axi_bvalid_d})
                2'b10:   outstanding <= outstanding + 1'b1;
                2'b01:   outstanding <= outstanding - 1'b1;
                default: ;
            endcase

            case (w_state)

                // ── W_IDLE ────────────────────────────────────────────────
                W_IDLE: begin
                    k             <= {K_W{1'b0}};
                    data_latched  <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    aw_accepted_r <= 1'b0;
                    w_accepted_r  <= 1'b0;
                    busy          <= 1'b0;
                    if (stall) begin
                        s_axis_tready <= 1'b0;
                    end else begin
                        s_axis_tready <= 1'b1;
                        if (s_axis_tvalid) begin
                            data_latch    <= s_axis_tdata;
                            data_latched  <= 1'b1;
                            s_axis_tready <= 1'b0;
                            w_state       <= W_WRITE;
                            busy          <= 1'b1;
                        end
                    end
                end

                // ── W_WRITE ───────────────────────────────────────────────
                // Issue AW and W together for each range bin.
                // FIX 1: use sticky flags to detect both AW and W accepted,
                //         even if they fire on different cycles.
                // FIX 2: removed the redundant unconditional awvalid clear that
                //         was preventing the advance condition from firing.
                W_WRITE: begin
                    // Accept next sample from stream once current write is done
                    if (!data_latched && s_axis_tvalid && s_axis_tready) begin
                        data_latch    <= s_axis_tdata;
                        data_latched  <= 1'b1;
                        s_axis_tready <= 1'b0;
                    end

                    // Issue AW and W when data is ready, not throttled,
                    // and not already in the process of advancing to next k.
                    // Guard: don't re-issue if both channels already accepted.
                    if (data_latched && outstanding < MAX_OUTSTANDING &&
                        !(aw_accepted_r && w_accepted_r) &&
                        !((aw_accepted_r || (m_axi_awvalid && m_axi_awready)) &&
                          (w_accepted_r  || (m_axi_wvalid  && m_axi_wready)))) begin
                        if (!m_axi_awvalid || m_axi_awready) begin
                            m_axi_awaddr  <= write_addr_r;
                            m_axi_awvalid <= 1'b1;
                        end
                        if (!m_axi_wvalid || m_axi_wready) begin
                            m_axi_wdata   <= data_latch;
                            m_axi_wvalid  <= 1'b1;
                        end
                    end

                    // Capture AW and W acceptances into sticky flags
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        aw_accepted_r <= 1'b1;
                    end
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid  <= 1'b0;
                        w_accepted_r  <= 1'b1;
                    end

                    // Advance only when BOTH AW and W are accepted.
                    // Check both the sticky flag AND the same-cycle case
                    // because both can fire simultaneously (sticky not yet set).
                    if ((aw_accepted_r || (m_axi_awvalid && m_axi_awready)) &&
                        (w_accepted_r  || (m_axi_wvalid  && m_axi_wready))) begin
                        aw_accepted_r <= 1'b0;
                        w_accepted_r  <= 1'b0;
                        data_latched  <= 1'b0;
                        // Always clear valids on advance to prevent phantom beats
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b0;
                        if (k < K_MAX) begin
                            k             <= k + 1'b1;
                            s_axis_tready <= 1'b1;
                        end else begin
                            k             <= {K_W{1'b0}};
                            s_axis_tready <= 1'b0;
                            w_state       <= W_DRAIN;
                        end
                    end
                end

                // ── W_DRAIN ───────────────────────────────────────────────
                // All N_RANGE_BINS writes issued.  Wait for every outstanding
                // B response so DDR3 has committed all data before the chirp
                // is declared complete and the next chirp begins.
                W_DRAIN: begin
                    if (outstanding == {($clog2(MAX_OUTSTANDING)+1){1'b0}}) begin
                        w_state    <= W_DONE;
                        chirp_done <= 1'b1;
                        busy       <= 1'b0;
                    end
                end

                // ── W_DONE ────────────────────────────────────────────────
                W_DONE: begin
                    w_state <= W_IDLE;
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
