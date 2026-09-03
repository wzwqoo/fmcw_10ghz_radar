`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// radar_udp_packetizer.v
//
// Packs the dumped frame stream into fixed-size UDP payloads.  Downstream idiom:
//
//   radar_frame_dump_streamer.m_axis
//      -> radar_udp_packetizer
//      -> fifo_generator_0   (Vivado IP, 32-bit, common clock)
//      -> udp / udp_tx       (Ethernet MAC/UDP core in the block design)
//
// Where the camera version packs a Mono8 row into one fixed UDP packet, this
// version chops the radar A/B/C/D word stream into fixed PAYLOAD_WORDS-word UDP
// packets, each carrying a 16-byte header so the host can reassemble per region.
//
// Because every radar region word-count is an exact multiple of PAYLOAD_WORDS
// (region bytes are multiples of the streamer's 256-beat burst), EVERY packet is
// full PAYLOAD_WORDS — there is no ragged final packet, so no padding logic is
// needed (unlike the camera early/late-TLAST handling).
//
// 16-byte packet header — matches host recv_radar_frame_udp.py struct ">IIBBHI":
//   offset size field        notes
//   0x00   4    magic        0x52414446 ('RADF'), big-endian on the wire
//   0x04   4    frame_id     big-endian
//   0x08   1    region_id    0=A 1=B 2=C 3=D
//   0x09   1    flags        bit0=region_first packet, bit1=region_last packet
//   0x0A   2    reserved     0
//   0x0C   4    byte_offset  byte offset of this payload within the region (BE)
// Payload: PAYLOAD_WORDS raw uint32 words, byte-swapped so the host reads them
//   as little-endian uint32 (np.dtype('<u4')).
//
// udp_tx sends tx_data[31:24] first, so a word written to the FIFO as
//   {b0,b1,b2,b3} leaves the wire as b0,b1,b2,b3.  Header fields are therefore
//   stored MSB-first (big-endian) and payload words are byte-reversed.
// -----------------------------------------------------------------------------

`default_nettype none

module radar_udp_packetizer #(
    parameter integer PAYLOAD_WORDS = 256,                 // uint32 words / packet
    parameter [15:0]  PACKET_BYTES  = 16'd1040             // 16 hdr + 256*4
)(
    input  wire        clk,
    input  wire        rst_n,

    // ── AXI4-Stream payload from radar_frame_dump_streamer.m_axis ─────────
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,   // first beat of a region
    input  wire        s_axis_tlast,   // last  beat of a region

    // ── Region/frame sideband (stable while a region streams) ─────────────
    input  wire [1:0]  s_region_id,
    input  wire [31:0] s_frame_id,
    input  wire [31:0] s_region_words, // total uint32 words in current region

    // ── Vivado FIFO Generator IP (32-bit, common clock) ───────────────────
    output reg  [31:0] fifo_din,
    output reg         fifo_wr_en,
    input  wire        fifo_full,
    input  wire        fifo_prog_full,
    input  wire [31:0] fifo_dout,
    output wire        fifo_rd_en,
    input  wire        fifo_empty,

    // ── Existing udp/udp_tx payload interface ─────────────────────────────
    output reg         tx_start_en,
    output wire [31:0] tx_data,
    output wire [15:0] tx_byte_num,
    input  wire        tx_done,
    input  wire        tx_req,

    // ── Debug ─────────────────────────────────────────────────────────────
    output reg  [15:0] queued_packets_dbg,
    output reg         tx_busy_dbg,
    output reg         underrun_dbg,
    output reg         frame_done_dbg   // 1-cycle pulse after a region-D last packet
);

    localparam [31:0] MAGIC = 32'h5241_4446;  // 'RADF'

    localparam [1:0] S_CAP    = 2'd0;  // grab a beat (and region info on tuser)
    localparam [1:0] S_HEADER = 2'd1;  // write the 4 header words
    localparam [1:0] S_PAYLOAD= 2'd2;  // forward PAYLOAD_WORDS payload words

    reg [1:0]  state;
    reg [1:0]  hdr_idx;

    // Latched per-region context
    reg [1:0]  region_id_l;
    reg [31:0] frame_id_l;
    reg [31:0] region_words_l;
    reg [31:0] word_off;        // word offset (within region) of the CURRENT packet start
    reg [31:0] pkt_word;        // payload words written in the current packet

    // One-beat hold (a captured beat waiting to be emitted after its header)
    reg        held_valid;
    reg [31:0] held_data;
    reg        held_last;

    reg        packet_complete_pulse;

    wire fifo_space = !fifo_full && !fifo_prog_full;

    // Flags computed from the packet-start offset.
    wire region_first = (word_off == 32'd0);
    wire region_last  = ((word_off + PAYLOAD_WORDS) >= region_words_l);

    // Header word for udp_tx byte order (MSB first on the wire).
    function [31:0] hdr_word(input [1:0] idx);
        case (idx)
            2'd0: hdr_word = MAGIC;
            2'd1: hdr_word = frame_id_l;
            2'd2: hdr_word = { {6'b0, region_id_l},                 // byte 8  region_id
                               {6'b0, region_last, region_first},   // byte 9  flags
                               16'h0000 };                          // bytes 10-11 reserved
            default: hdr_word = word_off << 2;                      // byte_offset (BE)
        endcase
    endfunction

    // Byte-reverse payload so the host reads little-endian uint32.
    function [31:0] swap32(input [31:0] w);
        swap32 = { w[7:0], w[15:8], w[23:16], w[31:24] };
    endfunction

    // S_CAP grabs the first beat of a packet (held while its header is written);
    // S_PAYLOAD then streams the remaining words straight through (gated by FIFO
    // space) once the held word has been drained.
    assign s_axis_tready =
        (state == S_CAP)     ? !held_valid :
        (state == S_PAYLOAD) ? (!held_valid && fifo_space) :
                               1'b0;

    wire        axis_fire = s_axis_tvalid && s_axis_tready;
    wire        pay_have  = held_valid || axis_fire;
    wire [31:0] pay_data  = held_valid ? held_data : s_axis_tdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                 <= S_CAP;
            hdr_idx               <= 2'd0;
            region_id_l           <= 2'd0;
            frame_id_l            <= 32'd0;
            region_words_l        <= 32'd0;
            word_off              <= 32'd0;
            pkt_word              <= 32'd0;
            held_valid            <= 1'b0;
            held_data             <= 32'd0;
            held_last             <= 1'b0;
            fifo_din              <= 32'd0;
            fifo_wr_en            <= 1'b0;
            packet_complete_pulse <= 1'b0;
            frame_done_dbg        <= 1'b0;
        end else begin
            fifo_wr_en            <= 1'b0;
            packet_complete_pulse <= 1'b0;
            frame_done_dbg        <= 1'b0;

            case (state)

                // Grab one beat; if it starts a region, (re)latch region context
                // and reset the in-region offset.  The beat is held until its
                // header has been written.
                S_CAP: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        held_data  <= s_axis_tdata;
                        held_last  <= s_axis_tlast;
                        held_valid <= 1'b1;
                        if (s_axis_tuser) begin
                            region_id_l    <= s_region_id;
                            frame_id_l     <= s_frame_id;
                            region_words_l <= s_region_words;
                            word_off       <= 32'd0;
                        end
                        hdr_idx <= 2'd0;
                        state   <= S_HEADER;
                    end
                end

                // Write the 4 header words (16 bytes) ahead of the payload.
                S_HEADER: begin
                    if (fifo_space) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din   <= hdr_word(hdr_idx);
                        if (hdr_idx == 2'd3) begin
                            pkt_word <= 32'd0;
                            state    <= S_PAYLOAD;
                        end else begin
                            hdr_idx <= hdr_idx + 2'd1;
                        end
                    end
                end

                // Forward PAYLOAD_WORDS payload words.  The first word is the
                // held beat (no stream ack); words 2..N come straight from the
                // stream with s_axis_tready asserted.
                S_PAYLOAD: begin
                    if (fifo_space && pay_have) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din   <= swap32(pay_data);
                        held_valid <= 1'b0;
                        word_off   <= word_off + 32'd1;

                        if (pkt_word == PAYLOAD_WORDS - 1) begin
                            // Packet finished.
                            packet_complete_pulse <= 1'b1;
                            if (region_last && (region_id_l == 2'd3))
                                frame_done_dbg <= 1'b1;
                            state <= S_CAP;     // next packet: grab next beat/region
                        end else begin
                            pkt_word <= pkt_word + 32'd1;
                        end
                    end
                end

                default: state <= S_CAP;
            endcase
        end
    end

    // ── UDP TX controller (identical idiom to camera_udp_packetizer) ──────
    wire can_start_tx = (!tx_busy_dbg) && (queued_packets_dbg != 16'd0) && (!fifo_empty);

    assign fifo_rd_en  = tx_req && !fifo_empty;
    assign tx_data     = fifo_dout;
    assign tx_byte_num = PACKET_BYTES;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            queued_packets_dbg <= 16'd0;
            tx_start_en        <= 1'b0;
            tx_busy_dbg        <= 1'b0;
            underrun_dbg       <= 1'b0;
        end else begin
            tx_start_en  <= 1'b0;
            underrun_dbg <= 1'b0;

            case ({packet_complete_pulse, can_start_tx})
                2'b10: queued_packets_dbg <= queued_packets_dbg + 16'd1;
                2'b01: queued_packets_dbg <= queued_packets_dbg - 16'd1;
                default: queued_packets_dbg <= queued_packets_dbg;
            endcase

            if (can_start_tx) begin
                tx_start_en <= 1'b1;
                tx_busy_dbg <= 1'b1;
            end else if (tx_done) begin
                tx_busy_dbg <= 1'b0;
            end

            if (tx_req && fifo_empty)
                underrun_dbg <= 1'b1;
        end
    end

endmodule

`default_nettype wire
