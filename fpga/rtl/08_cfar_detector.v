`timescale 1ns/1ps
`default_nettype none
`include "radar_params.vh"

// ============================================================================
// cfar_streaming_axi_verilog.v
//
// Plain Verilog-2001 streaming CA-CFAR with AXI4 master DDR access.
//
// Purpose:
//   Replace the BRAM-heavy full-frame rd_mem/noise_mem CFAR structure.
//
// Keeps only:
//   - rd_row_buf[256]              : one incoming RD row
//   - noise_linebuf[37][256]       : 37-row sliding Doppler-noise window
//   - cut_linebuf[37][256]         : 37-row delayed CUT values
//   - range_sum[256]               : running range-window sums per Doppler bin
//   - det_row_buf[256]             : one output detection row
//
// Does NOT allocate:
//   - rd_mem[128][256]
//   - noise_mem[128][256]
//   - det_mem[128][256]
//
// RD map layout in DDR:
//   cell_index = range_bin * 256 + doppler_bin
//   byte_addr  = RD_MAP_BASE[buf] + cell_index * 4
//
// Detection map layout in DDR:
//   packed one-bit-per-cell bitmap, not one word per cell
//   linear_cell = range_bin * 256 + doppler_bin
//   word_index  = linear_cell >> 5
//   bit_index   = linear_cell[4:0]
//   byte_addr   = DET_MAP_BASE[buf] + word_index * 4
//   value bit   = 1 if CFAR accepted the cell, else 0
//
// B/C/D ping-pong contract:
//   frame_buf_sel is latched when cfar_start is accepted.
//     frame_buf_sel = 0 -> read C0, write D0
//     frame_buf_sel = 1 -> read C1, write D1
//   The top-level must provide the same buffer index used by the complex-map
//   cache and summed-power collector for this radar frame.
//
// Edge handling:
//   Out-of-map rows/bins are treated as zero.
//   N_TRAIN_TOTAL remains fixed at 688, matching the HLS threshold convention.
//
// Timing model:
//   - Read one actual RD row for feed_row 0..127.
//   - Feed virtual zero rows for feed_row 128..145.
//   - Detection for output row r becomes valid when feed_row = r + 18.
//   - Therefore first output row is generated after feed_row 18.
//   - Last output row, row 127, is generated after virtual feed_row 145.
//
// Notes:
//   - This is memory-optimized, not maximum-throughput.
//   - It writes each output row before reading the next feed row.
//   - Later, this can be pipelined with ping-pong output rows.
// ============================================================================

module cfar_streaming_axi_verilog
#(
    parameter AXI_ADDR_W    = 32,
    parameter AXI_DATA_W    = 32,

    parameter RD_MAP_BASE0  = `RADAR_DDR_C0_BASE,
    parameter RD_MAP_BASE1  = `RADAR_DDR_C1_BASE,
    parameter DET_MAP_BASE0 = `RADAR_DDR_D0_BASE,
    parameter DET_MAP_BASE1 = `RADAR_DDR_D1_BASE
)
(
    input  wire                     clk,
    input  wire                     rst_n,

    // Selects C0/D0 or C1/D1 for the complete CFAR frame.  This signal is
    // sampled on cfar_start and held internally until cfar_done.
    input  wire                     frame_buf_sel,

    // ------------------------------------------------------------------------
    // CFAR control handshake.
    // Connect:
    //   rd_map_collector_summed.cfar_start -> cfar_start
    //   cfar_done -> rd_map_collector_summed.cfar_done
    // ------------------------------------------------------------------------
    input  wire                     cfar_start,
    output reg                      cfar_done,

    // Q8.8 threshold factor.
    input  wire [15:0]              alpha_q8,

    // Status.
    output reg                      busy,
    output reg                      axi_error,

    // ------------------------------------------------------------------------
    // AXI4 master read address channel.
    // ------------------------------------------------------------------------
    output reg  [AXI_ADDR_W-1:0]    m_axi_araddr,
    output reg  [7:0]               m_axi_arlen,
    output wire [2:0]               m_axi_arsize,
    output wire [1:0]               m_axi_arburst,
    output wire [3:0]               m_axi_arcache,
    output wire [2:0]               m_axi_arprot,
    output wire [3:0]               m_axi_arqos,
    output reg                      m_axi_arvalid,
    input  wire                     m_axi_arready,

    // ------------------------------------------------------------------------
    // AXI4 master read data channel.
    // ------------------------------------------------------------------------
    input  wire [AXI_DATA_W-1:0]    m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rlast,
    input  wire                     m_axi_rvalid,
    output reg                      m_axi_rready,

    // ------------------------------------------------------------------------
    // AXI4 master write address channel.
    // ------------------------------------------------------------------------
    output reg  [AXI_ADDR_W-1:0]    m_axi_awaddr,
    output reg  [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire [3:0]               m_axi_awcache,
    output wire [2:0]               m_axi_awprot,
    output wire [3:0]               m_axi_awqos,
    output reg                      m_axi_awvalid,
    input  wire                     m_axi_awready,

    // ------------------------------------------------------------------------
    // AXI4 master write data channel.
    // ------------------------------------------------------------------------
    output reg  [AXI_DATA_W-1:0]    m_axi_wdata,
    output wire [AXI_DATA_W/8-1:0]  m_axi_wstrb,
    output reg                      m_axi_wlast,
    output reg                      m_axi_wvalid,
    input  wire                     m_axi_wready,

    // ------------------------------------------------------------------------
    // AXI4 master write response channel.
    // ------------------------------------------------------------------------
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output reg                      m_axi_bready
);

    // ========================================================================
    // AXI constants.
    // ========================================================================
    assign m_axi_arsize  = 3'd2;          // 4 bytes per beat.
    assign m_axi_awsize  = 3'd2;          // 4 bytes per beat.
    assign m_axi_arburst = 2'b01;         // INCR burst.
    assign m_axi_awburst = 2'b01;         // INCR burst.
    assign m_axi_arcache = 4'b0011;       // Normal non-cacheable bufferable.
    assign m_axi_awcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_awqos   = 4'b0000;
    assign m_axi_wstrb   = 4'b1111;

    // ========================================================================
    // Fixed radar/CFAR constants.
    // ========================================================================
    localparam RANGE_BINS       = `RADAR_RANGE_BINS;
    localparam DOPPLER_BINS     = `RADAR_DOPPLER_BINS;
    localparam RD_ROW_BYTES     = DOPPLER_BINS * 4;       // 256 uint32 cells = 1024 bytes
    localparam DET_ROW_WORDS    = DOPPLER_BINS / 32;      // 256 one-bit cells = 8 words
    localparam DET_ROW_BYTES    = DET_ROW_WORDS * 4;      // 8 words = 32 bytes
    localparam [7:0] DET_WR_LAST = DET_ROW_WORDS - 1;

    localparam FEED_LAST        = 8'd145;  // 127 + 18 flush rows.
    localparam ACTUAL_LAST_ROW  = 8'd127;

    localparam N_GUARD_R        = 2;
    localparam N_TRAIN_R        = 16;
    localparam N_GUARD_D        = 1;
    localparam N_TRAIN_D        = 8;

    localparam HALF_R           = 18;
    localparam HALF_D           = 9;

    localparam N_TRAIN_TOTAL    = 688;
    localparam ALPHA_FRAC       = 8;

    localparam LINEBUF_ROWS     = 37;

    // ========================================================================
    // FSM state encoding.
    // ========================================================================
    localparam S_IDLE           = 5'd0;
    localparam S_CLEAR_SUM      = 5'd1;

    localparam S_READ_AR        = 5'd2;
    localparam S_READ_R         = 5'd3;
    localparam S_ZERO_ROW       = 5'd4;

    localparam S_P1_INIT        = 5'd5;
    localparam S_P1_RUN         = 5'd6;

    localparam S_DET_SETUP      = 5'd7;
    localparam S_DET_RUN        = 5'd8;

    localparam S_WRITE_AW       = 5'd9;
    localparam S_WRITE_PREP     = 5'd10;
    localparam S_WRITE_W        = 5'd11;
    localparam S_WRITE_B        = 5'd12;

    localparam S_NEXT_FEED      = 5'd13;
    localparam S_DONE           = 5'd14;

    reg [4:0] state;

    // ========================================================================
    // Small memories only.
    // ========================================================================

    // One RD row read from DDR C.
    (* ram_style = "block" *)
    reg [31:0] rd_row_buf [0:255];

    // 37-row circular line buffer of Doppler-noise sums.
    // Width is 40 bits: enough for 16 summed 32-bit cells with headroom.
    (* ram_style = "block" *)
    reg [39:0] noise_linebuf [0:36][0:255];

    // 37-row circular line buffer of delayed CUT cells.
    (* ram_style = "block" *)
    reg [31:0] cut_linebuf [0:36][0:255];

    // Running range-window sum per Doppler column.
    // Final range sum can exceed 40 bits in worst case, so use 44 bits.
    reg [43:0] range_sum [0:255];

    // One output detection row.
    reg det_row_buf [0:255];

    // The selected buffer is latched at cfar_start so the top-level cannot
    // accidentally switch from C0/D0 to C1/D1 in the middle of a CFAR frame.
    reg active_buf_sel;
    wire [31:0] selected_rd_base;
    wire [31:0] selected_det_base;

    assign selected_rd_base  = active_buf_sel ? RD_MAP_BASE1  : RD_MAP_BASE0;
    assign selected_det_base = active_buf_sel ? DET_MAP_BASE1 : DET_MAP_BASE0;


    // ========================================================================
    // Main counters.
    // ========================================================================
    reg [7:0] feed_row;       // 0..145. 0..127 real rows, 128..145 zero rows.
    reg [5:0] feed_slot;      // feed_row mod 37.

    reg [7:0] rd_beat;        // AXI read beat inside one row.
    reg [7:0] wr_beat;        // AXI write beat inside one packed detection row.


    // Pack 32 one-bit detection decisions into one AXI write word.
    // Bit 0 corresponds to the lowest Doppler bin in the group, so software can
    // scan word_index=linear_cell/32 and bit_index=linear_cell%32 directly.
    reg [31:0] det_packed_word;
    reg [7:0]  det_pack_base;
    integer det_pack_i;

    always @(*) begin
        det_pack_base   = {wr_beat[2:0], 5'd0};
        det_packed_word = 32'd0;
        for (det_pack_i = 0; det_pack_i < 32; det_pack_i = det_pack_i + 1) begin
            det_packed_word[det_pack_i] = det_row_buf[det_pack_base + det_pack_i];
        end
    end

    reg [7:0] clear_idx;      // clear range_sum[0..255].

    reg [7:0] zero_idx;       // zero-fill rd_row_buf for virtual rows.

    reg [3:0] p1_init_i;      // initializes Doppler right sum, 0..7.
    reg [7:0] p1_d;           // Doppler index for Pass 1.

    reg [7:0] det_d;          // Doppler index for detection row.
    reg [6:0] out_row;        // output row = feed_row - 18.

    reg [15:0] alpha_q8_reg;

    // ========================================================================
    // Doppler sliding sums for current RD row.
    // ========================================================================
    reg [39:0] p1_left_sum;
    reg [39:0] p1_right_sum;

    wire [39:0] p1_noise_val;
    assign p1_noise_val = p1_left_sum + p1_right_sum;

    // ========================================================================
    // Circular slot helpers.
    // All values are modulo 37.
    //
    // For current feed_row f and output center c = f - 18:
    //
    //   current new row:       f       -> feed_slot
    //   add_left row:          c - 3   -> f - 21
    //   remove_left row:       c - 19  -> f - 37 -> feed_slot old value
    //   remove_right row:      c + 2   -> f - 16
    //   center CUT row:        c       -> f - 18
    // ========================================================================

    wire [5:0] center_slot;
    wire [5:0] add_left_slot;
    wire [5:0] remove_right_slot;

    assign center_slot =
        (feed_slot >= 6'd18) ? (feed_slot - 6'd18) : (feed_slot + 6'd19);

    assign add_left_slot =
        (feed_slot >= 6'd21) ? (feed_slot - 6'd21) : (feed_slot + 6'd16);

    assign remove_right_slot =
        (feed_slot >= 6'd16) ? (feed_slot - 6'd16) : (feed_slot + 6'd21);

    // ========================================================================
    // Range-sum update validity.
    //
    // During feed_row 3..18:
    //   build initial sum for output row 0:
    //      rows 3..18
    //
    // During feed_row > 18:
    //   slide from previous output row to current output row.
    // ========================================================================

    wire range_init_phase;
    wire range_slide_phase;

    assign range_init_phase  = (feed_row >= 8'd3)  && (feed_row <= 8'd18);
    assign range_slide_phase = (feed_row >  8'd18);

    wire valid_add_left;
    wire valid_remove_left;
    wire valid_remove_right;

    assign valid_add_left     = (feed_row >= 8'd21) && (feed_row <= 8'd148);
    assign valid_remove_left  = (feed_row >= 8'd37) && (feed_row <= 8'd164);
    assign valid_remove_right = (feed_row >= 8'd19) && (feed_row <= 8'd143);

    // ========================================================================
    // Doppler sliding helper values.
    //
    // Update from Doppler d to Doppler d+1:
    //   left_next  = left  + row[d-1]  - row[d-9]
    //   right_next = right + row[d+10] - row[d+2]
    // ========================================================================

    reg [39:0] p1_left_add;
    reg [39:0] p1_left_remove;
    reg [39:0] p1_right_add;
    reg [39:0] p1_right_remove;

    always @(*) begin
        p1_left_add     = 40'd0;
        p1_left_remove  = 40'd0;
        p1_right_add    = 40'd0;
        p1_right_remove = 40'd0;

        if (p1_d >= 8'd1) begin
            p1_left_add = {8'd0, rd_row_buf[p1_d - 8'd1]};
        end

        if (p1_d >= 8'd9) begin
            p1_left_remove = {8'd0, rd_row_buf[p1_d - 8'd9]};
        end

        if (p1_d <= 8'd253) begin
            p1_right_remove = {8'd0, rd_row_buf[p1_d + 8'd2]};
        end

        if (p1_d <= 8'd245) begin
            p1_right_add = {8'd0, rd_row_buf[p1_d + 8'd10]};
        end
    end

    // ========================================================================
    // Range-sum update terms.
    //
    // These read OLD line-buffer values before the current feed_slot write.
    // With nonblocking assignments, RHS memory reads use old values.
    // ========================================================================

    reg [43:0] range_add_current;
    reg [43:0] range_add_left;
    reg [43:0] range_remove_left;
    reg [43:0] range_remove_right;
    reg [43:0] range_next;

    always @(*) begin
        range_add_current  = {4'd0, p1_noise_val};
        range_add_left     = 44'd0;
        range_remove_left  = 44'd0;
        range_remove_right = 44'd0;

        if (valid_add_left) begin
            range_add_left = {4'd0, noise_linebuf[add_left_slot][p1_d]};
        end

        if (valid_remove_left) begin
            range_remove_left = {4'd0, noise_linebuf[feed_slot][p1_d]};
        end

        if (valid_remove_right) begin
            range_remove_right = {4'd0, noise_linebuf[remove_right_slot][p1_d]};
        end

        if (range_init_phase) begin
            range_next = range_sum[p1_d] + range_add_current;
        end else if (range_slide_phase) begin
            range_next = range_sum[p1_d]
                       + range_add_current
                       + range_add_left
                       - range_remove_left
                       - range_remove_right;
        end else begin
            range_next = range_sum[p1_d];
        end
    end

    // ========================================================================
    // Detection arithmetic.
    //
    // threshold  = (alpha_q8 * range_sum[d]) >> 8
    // cut_scaled = CUT * 688
    // det        = cut_scaled > threshold
    // ========================================================================

    reg [59:0] threshold_product;
    reg [63:0] threshold_shifted;
    reg [63:0] cut_product;
    reg        det_next;

    always @(*) begin
        threshold_product = {44'd0, alpha_q8_reg} * {16'd0, range_sum[det_d]};
        threshold_shifted = {4'd0, threshold_product} >> ALPHA_FRAC;
        cut_product       = {32'd0, cut_linebuf[center_slot][det_d]} * 64'd688;

        if (cut_product > threshold_shifted) begin
            det_next = 1'b1;
        end else begin
            det_next = 1'b0;
        end
    end

    // ========================================================================
    // Address generation.
    //
    // C row: 256 power cells * 4 bytes = 1024 bytes, so row offset = row << 10.
    // D row: 256 detection bits = 32 bytes, so row offset = row << 5.
    // ========================================================================

    wire [31:0] feed_row_byte_offset;
    wire [31:0] out_row_byte_offset;

    assign feed_row_byte_offset = {14'd0, feed_row, 10'd0};
    assign out_row_byte_offset  = {20'd0, out_row, 5'd0};

    // ========================================================================
    // Main FSM.
    // ========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;

            cfar_done      <= 1'b0;
            busy           <= 1'b0;
            axi_error      <= 1'b0;

            feed_row       <= 8'd0;
            feed_slot      <= 6'd0;

            rd_beat        <= 8'd0;
            wr_beat        <= 8'd0;
            clear_idx      <= 8'd0;
            zero_idx       <= 8'd0;

            p1_init_i      <= 4'd0;
            p1_d           <= 8'd0;

            det_d          <= 8'd0;
            out_row        <= 7'd0;

            alpha_q8_reg   <= 16'd0;

            p1_left_sum    <= 40'd0;
            p1_right_sum   <= 40'd0;

            m_axi_araddr   <= {AXI_ADDR_W{1'b0}};
            m_axi_arlen    <= 8'd0;
            m_axi_arvalid  <= 1'b0;
            m_axi_rready   <= 1'b0;

            m_axi_awaddr   <= {AXI_ADDR_W{1'b0}};
            m_axi_awlen    <= 8'd0;
            m_axi_awvalid  <= 1'b0;
            m_axi_wdata    <= {AXI_DATA_W{1'b0}};
            m_axi_wvalid   <= 1'b0;
            m_axi_wlast    <= 1'b0;
            m_axi_bready   <= 1'b0;
            active_buf_sel <= 1'b0;
        end else begin
            cfar_done <= 1'b0;

            case (state)

                // ------------------------------------------------------------
                // Wait for collector to finish DDR C write.
                // ------------------------------------------------------------
                S_IDLE: begin
                    busy          <= 1'b0;

                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_wlast   <= 1'b0;
                    m_axi_bready  <= 1'b0;

                    if (cfar_start) begin
                        active_buf_sel <= frame_buf_sel;
                        busy           <= 1'b1;
                        axi_error      <= 1'b0;
                        alpha_q8_reg   <= alpha_q8;

                        feed_row     <= 8'd0;
                        feed_slot    <= 6'd0;
                        clear_idx    <= 8'd0;

                        state        <= S_CLEAR_SUM;
                    end
                end

                // ------------------------------------------------------------
                // Clear running range sums for all Doppler bins.
                // This avoids stale data from previous frame.
                // ------------------------------------------------------------
                S_CLEAR_SUM: begin
                    range_sum[clear_idx] <= 44'd0;

                    if (clear_idx == 8'd255) begin
                        clear_idx <= 8'd0;
                        state     <= S_READ_AR;
                    end else begin
                        clear_idx <= clear_idx + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Issue one 256-beat AXI read burst for current actual row.
                // Only rows 0..127 are real DDR reads.
                // Rows 128..145 are virtual zero rows for bottom-edge flush.
                // ------------------------------------------------------------
                S_READ_AR: begin
                    if (feed_row <= ACTUAL_LAST_ROW) begin
                        m_axi_araddr  <= selected_rd_base + feed_row_byte_offset;
                        m_axi_arlen   <= 8'd255;
                        m_axi_arvalid <= 1'b1;
                        rd_beat       <= 8'd0;

                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 1'b0;
                            m_axi_rready  <= 1'b1;
                            state         <= S_READ_R;
                        end
                    end else begin
                        zero_idx <= 8'd0;
                        state    <= S_ZERO_ROW;
                    end
                end

                // ------------------------------------------------------------
                // Receive one DDR row into rd_row_buf.
                // ------------------------------------------------------------
                S_READ_R: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        rd_row_buf[rd_beat] <= m_axi_rdata[31:0];

                        if (m_axi_rresp != 2'b00) begin
                            axi_error <= 1'b1;
                        end

                        if (rd_beat == 8'd255) begin
                            if (!m_axi_rlast) begin
                                axi_error <= 1'b1;
                            end

                            m_axi_rready <= 1'b0;
                            p1_init_i    <= 4'd0;
                            p1_left_sum  <= 40'd0;
                            p1_right_sum <= 40'd0;
                            state        <= S_P1_INIT;
                        end else begin
                            if (m_axi_rlast) begin
                                axi_error <= 1'b1;
                            end

                            rd_beat <= rd_beat + 1'b1;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Fill rd_row_buf with zeros for virtual bottom-edge rows.
                // ------------------------------------------------------------
                S_ZERO_ROW: begin
                    rd_row_buf[zero_idx] <= 32'd0;

                    if (zero_idx == 8'd255) begin
                        zero_idx     <= 8'd0;
                        p1_init_i    <= 4'd0;
                        p1_left_sum  <= 40'd0;
                        p1_right_sum <= 40'd0;
                        state        <= S_P1_INIT;
                    end else begin
                        zero_idx <= zero_idx + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Initialize Doppler right_sum for d=0:
                // training bins 2..9.
                // ------------------------------------------------------------
                S_P1_INIT: begin
                    p1_right_sum <= p1_right_sum
                                  + {8'd0, rd_row_buf[p1_init_i + 4'd2]};

                    if (p1_init_i == 4'd7) begin
                        p1_d  <= 8'd0;
                        state <= S_P1_RUN;
                    end else begin
                        p1_init_i <= p1_init_i + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Compute Doppler sliding sum for one row.
                //
                // Each cycle:
                //   1. produce p1_noise_val for current Doppler bin
                //   2. write current noise row into noise_linebuf
                //   3. write delayed CUT row into cut_linebuf
                //   4. update range_sum[d] if row participates in current range window
                //   5. slide Doppler sums
                // ------------------------------------------------------------
                S_P1_RUN: begin
                    noise_linebuf[feed_slot][p1_d] <= p1_noise_val;
                    cut_linebuf[feed_slot][p1_d]   <= rd_row_buf[p1_d];

                    if (range_init_phase || range_slide_phase) begin
                        range_sum[p1_d] <= range_next;
                    end

                    if (p1_d == 8'd255) begin
                        if (feed_row >= 8'd18) begin
                            out_row <= feed_row - 8'd18;
                            det_d   <= 8'd0;
                            state   <= S_DET_SETUP;
                        end else begin
                            state <= S_NEXT_FEED;
                        end
                    end else begin
                        p1_left_sum  <= p1_left_sum
                                      + p1_left_add
                                      - p1_left_remove;

                        p1_right_sum <= p1_right_sum
                                      + p1_right_add
                                      - p1_right_remove;

                        p1_d <= p1_d + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Detection row setup.
                // ------------------------------------------------------------
                S_DET_SETUP: begin
                    det_d <= 8'd0;
                    state <= S_DET_RUN;
                end

                // ------------------------------------------------------------
                // Generate one detection row into det_row_buf.
                // ------------------------------------------------------------
                S_DET_RUN: begin
                    det_row_buf[det_d] <= det_next;

                    if (det_d == 8'd255) begin
                        wr_beat <= 8'd0;
                        state   <= S_WRITE_AW;
                    end else begin
                        det_d <= det_d + 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // Issue one 8-beat AXI write burst for one packed detection row.
                // Each beat packs 32 Doppler detections, so one 256-bin row is
                // only 32 bytes instead of the old incorrect 1024 bytes.
                // ------------------------------------------------------------
                S_WRITE_AW: begin
                    m_axi_awaddr  <= selected_det_base + out_row_byte_offset;
                    m_axi_awlen   <= DET_WR_LAST;
                    m_axi_awvalid <= 1'b1;
                    m_axi_bready  <= 1'b1;
                    m_axi_wlast   <= 1'b0;
                    wr_beat       <= 8'd0;

                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        state         <= S_WRITE_PREP;
                    end
                end

                // ------------------------------------------------------------
                // Prepare one write beat.
                // ------------------------------------------------------------
                S_WRITE_PREP: begin
                    m_axi_wdata  <= det_packed_word;
                    m_axi_wvalid <= 1'b1;
                    m_axi_wlast  <= (wr_beat == DET_WR_LAST);
                    state        <= S_WRITE_W;
                end

                // ------------------------------------------------------------
                // Hold WVALID until accepted.
                // ------------------------------------------------------------
                S_WRITE_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;

                        if (wr_beat == DET_WR_LAST) begin
                            state <= S_WRITE_B;
                        end else begin
                            wr_beat <= wr_beat + 1'b1;
                            state   <= S_WRITE_PREP;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Wait for row write response.
                // ------------------------------------------------------------
                S_WRITE_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        if (m_axi_bresp != 2'b00) begin
                            axi_error <= 1'b1;
                        end

                        state <= S_NEXT_FEED;
                    end
                end

                // ------------------------------------------------------------
                // Advance feed row and circular slot.
                // ------------------------------------------------------------
                S_NEXT_FEED: begin
                    if (feed_row == FEED_LAST) begin
                        state <= S_DONE;
                    end else begin
                        feed_row <= feed_row + 1'b1;

                        if (feed_slot == 6'd36) begin
                            feed_slot <= 6'd0;
                        end else begin
                            feed_slot <= feed_slot + 1'b1;
                        end

                        state <= S_READ_AR;
                    end
                end

                // ------------------------------------------------------------
                // Pulse cfar_done back to rd_map_collector_summed.
                // ------------------------------------------------------------
                S_DONE: begin
                    cfar_done <= 1'b1;
                    busy      <= 1'b0;
                    state     <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                    busy  <= 1'b0;
                end

            endcase
        end
    end

endmodule

`default_nettype wire
