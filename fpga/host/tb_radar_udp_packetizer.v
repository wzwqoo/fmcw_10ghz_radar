// =============================================================================
//  tb_radar_udp_packetizer.v
//  Drives a synthetic A..D region stream into radar_udp_packetizer and checks
//  the FIFO word sequence: header magic/frame/region/flags/offset + byte-swapped
//  payload, packet segmentation, and the region-D frame_done pulse.
//  Uses PAYLOAD_WORDS=4 so packets are tiny.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_radar_udp_packetizer;
    localparam PW = 4;                 // payload words/packet
    localparam [31:0] MAGIC = 32'h52414446;
    localparam [31:0] FRAME = 32'h00001234;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // DUT I/O
    reg  [31:0] s_tdata; reg s_tvalid, s_tuser, s_tlast;
    wire        s_tready;
    reg  [1:0]  s_region; reg [31:0] s_frame, s_rwords;
    wire [31:0] fifo_din; wire fifo_wr_en;
    wire        frame_done;

    radar_udp_packetizer #(.PAYLOAD_WORDS(PW), .PACKET_BYTES(16 + PW*4)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .s_axis_tuser(s_tuser), .s_axis_tlast(s_tlast),
        .s_region_id(s_region), .s_frame_id(s_frame), .s_region_words(s_rwords),
        .fifo_din(fifo_din), .fifo_wr_en(fifo_wr_en),
        .fifo_full(1'b0), .fifo_prog_full(1'b0),
        .fifo_dout(32'd0), .fifo_rd_en(), .fifo_empty(1'b1),
        .tx_start_en(), .tx_data(), .tx_byte_num(), .tx_done(1'b0), .tx_req(1'b0),
        .queued_packets_dbg(), .tx_busy_dbg(), .underrun_dbg(),
        .frame_done_dbg(frame_done)
    );

    // Capture FIFO writes
    reg [31:0] cap [0:255];
    integer ncap = 0;
    always @(posedge clk) if (rst_n && fifo_wr_en) begin cap[ncap] = fifo_din; ncap = ncap + 1; end

    integer frame_done_count = 0;
    always @(posedge clk) if (rst_n && frame_done) frame_done_count = frame_done_count + 1;

    function [31:0] swap32(input [31:0] w); swap32 = {w[7:0],w[15:8],w[23:16],w[31:24]}; endfunction

    // Drive one region of `words` beats (data = base+i), tuser on first, tlast on last.
    task drive_region(input [1:0] id, input [31:0] words, input [31:0] base);
        integer i;
        begin
            for (i = 0; i < words; i = i + 1) begin
                @(negedge clk);
                s_tvalid <= 1'b1;
                s_tdata  <= base + i;
                s_tuser  <= (i == 0);
                s_tlast  <= (i == words-1);
                s_region <= id; s_frame <= FRAME; s_rwords <= words;
                // wait for the cycle where tready is high at the rising edge
                @(posedge clk);
                while (!s_tready) @(posedge clk);
            end
            @(negedge clk); s_tvalid <= 1'b0; s_tuser <= 1'b0; s_tlast <= 1'b0;
        end
    endtask

    integer errors = 0;
    task chk(input [31:0] got, input [31:0] exp, input [127:0] tag);
        begin
            if (got !== exp) begin
                $display("ERROR %0s: got %h exp %h", tag, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    integer p, base_i;
    initial begin
        s_tvalid=0; s_tdata=0; s_tuser=0; s_tlast=0; s_region=0; s_frame=0; s_rwords=0;
        repeat (3) @(posedge clk);
        rst_n = 1;

        // Region A (id0): 8 words -> 2 packets;  Region D (id3): 4 words -> 1 packet.
        drive_region(2'd0, 32'd8, 32'h0A00);
        drive_region(2'd3, 32'd4, 32'h0D00);
        repeat (8) @(posedge clk);

        // Expected: 3 packets, each = 4 hdr words + 4 payload words = 8 words.
        // ── packet 0: region 0, offset 0, flags=first(0x01) ──
        chk(cap[0], MAGIC,          "p0.magic");
        chk(cap[1], FRAME,          "p0.frame");
        chk(cap[2], 32'h00_01_0000, "p0.regflags");   // region0, flags first
        chk(cap[3], 32'd0,          "p0.offset");
        for (base_i=0; base_i<PW; base_i=base_i+1)
            chk(cap[4+base_i], swap32(32'h0A00+base_i), "p0.payload");

        // ── packet 1: region 0, offset 16 bytes, flags=last(0x02) ──
        chk(cap[8],  MAGIC,          "p1.magic");
        chk(cap[10], 32'h00_02_0000, "p1.regflags");   // region0, flags last
        chk(cap[11], 32'd16,         "p1.offset");
        for (base_i=0; base_i<PW; base_i=base_i+1)
            chk(cap[12+base_i], swap32(32'h0A04+base_i), "p1.payload");

        // ── packet 2: region 3, offset 0, flags=first|last(0x03) ──
        chk(cap[16], MAGIC,          "p2.magic");
        chk(cap[18], 32'h03_03_0000, "p2.regflags");   // region3, flags first+last
        chk(cap[19], 32'd0,          "p2.offset");
        for (base_i=0; base_i<PW; base_i=base_i+1)
            chk(cap[20+base_i], swap32(32'h0D00+base_i), "p2.payload");

        if (ncap != 24)            begin $display("ERROR: ncap=%0d exp 24", ncap); errors=errors+1; end
        if (frame_done_count != 1) begin $display("ERROR: frame_done=%0d exp 1", frame_done_count); errors=errors+1; end

        $display("------------------------------------------------------------");
        $display("fifo words = %0d, frame_done pulses = %0d, errors = %0d", ncap, frame_done_count, errors);
        if (errors==0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end
endmodule

`default_nettype wire
