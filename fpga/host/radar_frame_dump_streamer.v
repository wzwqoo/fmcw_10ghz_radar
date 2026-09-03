// =============================================================================
//  radar_frame_dump_streamer.v   *** 3-RX VARIANT — DDR -> host one-shot dump ***
//
//  Streams ONE radar frame's post-Doppler product regions (B, C, D) out as a
//  single 32-bit AXI4-Stream for the host to run post-CFAR / trajectory / spin
//  processing.  Region A (range-FFT scratch) is intentionally NOT dumped: B is
//  the Doppler transform of A, so A is redundant, and A has an independent
//  ping/pong that would need freezing to stay coherent.  B/C/D share one
//  frame_buf_sel, so the dump is coherent with a single select bit.
//
//  Region order and sizes (3-RX, from radar_params.vh):
//      region_id 1 = B : per-RX complex RD map RADAR_RX_RDMAP_CACHE_BYTES (384 KiB)
//      region_id 2 = C : summed |X|^2 RD map   RADAR_RD_MAP_BYTES         (128 KiB)
//      region_id 3 = D : packed CFAR det map   RADAR_DET_MAP_WORDS*4      (  4 KiB)
//  Total ~516 KiB / frame.  All region word-counts are exact multiples of
//  MAX_BEATS, so every AXI burst is a full MAX_BEATS-beat INCR burst.  Region id
//  keeps its B=1/C=2/D=3 numbering (A would have been 0) so headers are stable.
//
//  AXI read master idiom (AR/R + skid buffer, tied ARSIZE/ARBURST) mirrors
//  doppler_seq_ctrl.v so it drops onto the same SmartConnect -> MIG path.
//
//  Downstream: feed m_axis_* into radar_udp_packetizer.v.  The
//  per-region header the host needs (frame_id, region_id, region_words) is
//  exposed as stable sideband while that region streams; tuser marks the first
//  beat of a region and tlast marks its last beat, so the packetizer can emit
//  one UDP "region header + payload" sequence per region.
// =============================================================================

`default_nettype none
`include "radar_params.vh"

module radar_frame_dump_streamer #(
    parameter DATA_W    = 32,
    parameter ADDR_W    = 32,
    parameter MAX_BEATS = 256          // beats per AXI burst (ARLEN = MAX_BEATS-1)
)(
    input  wire clk,
    input  wire rst_n,

    // ── Control ───────────────────────────────────────────────────────────
    input  wire              start,         // 1-cycle pulse: dump the selected frame
    input  wire              frame_buf_sel, // B/C/D set: 0 -> B0/C0/D0, 1 -> B1/C1/D1
    input  wire [31:0]       frame_id,      // tag echoed to host
    output reg               busy,
    output reg               done,          // 1-cycle pulse after region D's last beat

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

    // ── AXI-Stream payload out (to UDP packetizer) ────────────────────────
    output reg  [DATA_W-1:0] m_axis_tdata,
    output reg               m_axis_tvalid,
    output reg               m_axis_tlast,   // last beat of the current region
    output reg               m_axis_tuser,   // first beat of the current region
    input  wire              m_axis_tready,

    // ── Sideband identifying the region/frame currently streaming ─────────
    output reg  [1:0]        region_id,      // 1=B 2=C 3=D
    output reg  [31:0]       frame_id_out,
    output reg  [31:0]       region_words    // total 32-bit words in current region
);

    assign m_axi_arsize  = 3'd2;     // 4 bytes per beat (DATA_W = 32)
    assign m_axi_arburst = 2'b01;    // INCR

    // ── Region word counts (32-bit words) ─────────────────────────────────
    localparam [31:0] LEN_B = (`RADAR_RX_RDMAP_CACHE_BYTES) >> 2;
    localparam [31:0] LEN_C = (`RADAR_RD_MAP_BYTES)         >> 2;
    localparam [31:0] LEN_D = (`RADAR_DET_MAP_WORDS);

    localparam [1:0] REG_FIRST = 2'd1;   // B
    localparam [1:0] REG_LAST  = 2'd3;   // D

    localparam [ADDR_W-1:0] BURST_BYTES = MAX_BEATS * (DATA_W/8);

    // ── Per-region base (from latched frame_buf_sel) and length; reg 1/2/3 ─
    function [ADDR_W-1:0] region_base(input [1:0] r, input sel);
        case (r)
            2'd1: region_base = sel ? `RADAR_DDR_B1_BASE : `RADAR_DDR_B0_BASE;
            2'd2: region_base = sel ? `RADAR_DDR_C1_BASE : `RADAR_DDR_C0_BASE;
            default: region_base = sel ? `RADAR_DDR_D1_BASE : `RADAR_DDR_D0_BASE;
        endcase
    endfunction

    function [31:0] region_len(input [1:0] r);
        case (r)
            2'd1: region_len = LEN_B;
            2'd2: region_len = LEN_C;
            default: region_len = LEN_D;
        endcase
    endfunction

    // ── State ─────────────────────────────────────────────────────────────
    localparam S_IDLE     = 3'd0;
    localparam S_SETUP    = 3'd1;
    localparam S_ISSUE_AR = 3'd2;
    localparam S_WAIT_AR  = 3'd3;
    localparam S_STREAM   = 3'd4;
    localparam S_NEXT_REG = 3'd5;
    localparam S_DONE     = 3'd6;

    reg [2:0]        state;
    reg              sel_r;          // latched B/C/D set select
    reg [1:0]        reg_idx;
    reg [ADDR_W-1:0] cur_base;       // base of the current burst within the region
    reg [31:0]       words_left;     // words remaining in the current region
    reg              region_first;   // pending tuser for first beat of region

    // Output-register skid: the m_axis_* output registers ARE the skid stage.
    // A captured R-beat lands directly on m_axis_tdata/tvalid and is held there
    // (tvalid stays asserted, tdata stable) until downstream tready accepts it —
    // protocol-correct under backpressure from a UDP FIFO.  m_axi_rready is only
    // raised while the skid is empty, so DDR backpressure propagates naturally.
    reg              skid_burst_last; // m_axi_rlast of the beat held in m_axis_tdata

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            busy          <= 1'b0;
            done          <= 1'b0;
            sel_r         <= 1'b0;
            reg_idx       <= REG_FIRST;
            cur_base      <= {ADDR_W{1'b0}};
            words_left    <= 32'd0;
            region_first  <= 1'b0;
            m_axi_araddr  <= {ADDR_W{1'b0}};
            m_axi_arlen   <= 8'd0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axis_tdata  <= {DATA_W{1'b0}};
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tuser  <= 1'b0;
            region_id     <= 2'd0;
            frame_id_out  <= 32'd0;
            region_words  <= 32'd0;
            skid_burst_last <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)

                // ── S_IDLE — wait for a dump request ──────────────────────
                S_IDLE: begin
                    busy          <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tuser  <= 1'b0;
                    if (start) begin
                        sel_r        <= frame_buf_sel;
                        frame_id_out <= frame_id;
                        reg_idx      <= REG_FIRST;   // start at B
                        busy         <= 1'b1;
                        state        <= S_SETUP;
                    end
                end

                // ── S_SETUP — latch base/len for the current region (B/C/D) ─
                S_SETUP: begin
                    cur_base      <= region_base(reg_idx, sel_r);
                    words_left    <= region_len(reg_idx);
                    region_words  <= region_len(reg_idx);
                    region_id     <= reg_idx;
                    region_first  <= 1'b1;     // next forwarded beat carries tuser
                    state         <= S_ISSUE_AR;
                end

                // ── S_ISSUE_AR — present one MAX_BEATS INCR burst ─────────
                S_ISSUE_AR: begin
                    m_axi_araddr  <= cur_base;
                    m_axi_arlen   <= MAX_BEATS - 1;
                    m_axi_arvalid <= 1'b1;
                    state         <= S_WAIT_AR;
                end

                S_WAIT_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= S_STREAM;
                    end
                end

                // ── S_STREAM — capture R into the output skid, drain to AXIS ──
                //   m_axis_tvalid IS the skid-occupied flag: it stays asserted
                //   with stable tdata until tready accepts the beat.  rready is
                //   only high while the skid is empty (m_axis_tvalid == 0).
                S_STREAM: begin
                    // Step A: capture an R-channel beat into the (empty) skid.
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axis_tdata    <= m_axi_rdata;
                        m_axis_tvalid   <= 1'b1;
                        m_axis_tuser    <= region_first;
                        m_axis_tlast    <= (words_left == 32'd1); // last word of region
                        skid_burst_last <= m_axi_rlast;           // end of this burst
                        region_first    <= 1'b0;
                        words_left      <= words_left - 32'd1;
                        m_axi_rready    <= 1'b0;                   // skid now full
                    end

                    // Step B: drain the held beat when downstream accepts it.
                    if (m_axis_tvalid && m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;                     // skid emptied
                        if (m_axis_tlast) begin
                            // Whole region delivered.
                            state <= S_NEXT_REG;
                        end else if (skid_burst_last) begin
                            // Burst finished but region continues: next burst.
                            cur_base <= cur_base + BURST_BYTES;
                            state    <= S_ISSUE_AR;
                        end else begin
                            // Mid-burst: accept the next beat.
                            m_axi_rready <= 1'b1;
                        end
                    end
                end

                // ── S_NEXT_REG — advance A->B->C->D, or finish ────────────
                S_NEXT_REG: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tuser  <= 1'b0;
                    if (reg_idx == REG_LAST) begin
                        state <= S_DONE;
                    end else begin
                        reg_idx <= reg_idx + 2'd1;
                        state   <= S_SETUP;
                    end
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
