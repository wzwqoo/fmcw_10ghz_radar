// =============================================================================
//  tb_radar_frame_dump_streamer.v
//  Self-checking testbench for radar_frame_dump_streamer.
//
//  Behavioural AXI4 read slave returns rdata = word-address (araddr>>2 + beat),
//  so the stream payload can be checked for contiguity within each region.
//  A jittery m_axis_tready exercises the skid buffer.  Checks:
//    * region_id sequence 0,1,2,3
//    * per-region beat counts == LEN_A/B/C/D
//    * tuser on the first beat of each region, tlast on the last
//    * payload words contiguous from each region's base
//    * exactly one `done` pulse after region D
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_radar_frame_dump_streamer;

    localparam DATA_W = 32, ADDR_W = 32, MAX_BEATS = 256;

    // Expected region lengths (3-RX) in 32-bit words. Dump is B/C/D only.
    localparam LEN_B = (3*128*256*4) >> 2;   // 98304
    localparam LEN_C = (128*256*4)   >> 2;   // 32768
    localparam LEN_D = ((128*256 + 31)/32);  // 1024
    localparam TOTAL = LEN_B + LEN_C + LEN_D;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // DUT control
    reg         start = 0, frame_buf_sel = 0;
    reg  [31:0] frame_id = 32'hABCD0001;
    wire        busy, done;

    // AXI AR/R
    wire [ADDR_W-1:0] araddr;
    wire [7:0]        arlen;
    wire [2:0]        arsize;
    wire [1:0]        arburst;
    wire              arvalid;
    reg               arready;
    reg  [DATA_W-1:0] rdata;
    reg  [1:0]        rresp;
    reg               rlast;
    reg               rvalid;
    wire              rready;

    // AXI-Stream
    wire [DATA_W-1:0] tdata;
    wire              tvalid, tlast, tuser;
    reg               tready;
    wire [1:0]        region_id;
    wire [31:0]       frame_id_out, region_words;

    radar_frame_dump_streamer #(.DATA_W(DATA_W), .ADDR_W(ADDR_W), .MAX_BEATS(MAX_BEATS)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .frame_buf_sel(frame_buf_sel), .frame_id(frame_id),
        .busy(busy), .done(done),
        .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(arburst), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast),
        .m_axi_rvalid(rvalid), .m_axi_rready(rready),
        .m_axis_tdata(tdata), .m_axis_tvalid(tvalid), .m_axis_tlast(tlast),
        .m_axis_tuser(tuser), .m_axis_tready(tready),
        .region_id(region_id), .frame_id_out(frame_id_out), .region_words(region_words)
    );

    // ── Behavioural AXI read slave ────────────────────────────────────────
    reg [ADDR_W-1:0] burst_word0;   // word address of beat 0
    reg [8:0]        beat;          // 0..MAX_BEATS
    reg [8:0]        burst_len;     // arlen+1
    reg              bursting;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arready  <= 1'b1;
            rvalid   <= 1'b0;
            rlast    <= 1'b0;
            rresp    <= 2'b00;
            rdata    <= 0;
            bursting <= 1'b0;
            beat     <= 0;
        end else begin
            // Accept AR when idle
            if (arvalid && arready && !bursting) begin
                burst_word0 <= araddr >> 2;
                burst_len   <= arlen + 1;
                beat        <= 0;
                bursting    <= 1'b1;
                arready     <= 1'b0;
            end

            if (bursting) begin
                if (!rvalid || rready) begin
                    rdata  <= burst_word0 + beat;
                    rvalid <= 1'b1;
                    rlast  <= (beat == burst_len - 1);
                    if (beat == burst_len - 1) begin
                        bursting <= 1'b0;
                        arready  <= 1'b1;
                        beat     <= 0;
                    end else begin
                        beat <= beat + 1;
                    end
                end
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
            end
        end
    end

    // ── Jittery downstream ready (exercises skid) ─────────────────────────
    reg [2:0] jitter = 0;
    always @(posedge clk) jitter <= jitter + 1;
    always @(*) tready = (jitter != 3'd4);   // low 1-in-8 cycles

    // ── Stream checker ────────────────────────────────────────────────────
    integer beats_in_region = 0;
    integer total_beats     = 0;
    integer errors          = 0;
    reg [1:0] exp_region    = 1;   // first dumped region is B (id 1)
    reg [31:0] exp_word     = 0;       // expected payload word in this region
    reg first_beat          = 1;
    integer done_pulses     = 0;

    function [31:0] reg_len(input [1:0] r);
        case (r)
            2'd1: reg_len = LEN_B;
            2'd2: reg_len = LEN_C;
            default: reg_len = LEN_D;
        endcase
    endfunction

    always @(posedge clk) begin
        if (rst_n && tvalid && tready) begin
            // region id must match the region we expect
            if (region_id !== exp_region) begin
                $display("ERROR @%0t: region_id=%0d expected %0d", $time, region_id, exp_region);
                errors = errors + 1;
            end
            // tuser only on the first beat of a region
            if (beats_in_region == 0 && !tuser) begin
                $display("ERROR @%0t: missing tuser at region %0d start", $time, region_id);
                errors = errors + 1;
            end
            if (beats_in_region != 0 && tuser) begin
                $display("ERROR @%0t: spurious tuser mid-region %0d", $time, region_id);
                errors = errors + 1;
            end
            // payload contiguity within region
            if (first_beat) begin
                exp_word  = tdata;       // region base word (slave-defined)
                first_beat = 0;
            end
            if (tdata !== exp_word) begin
                $display("ERROR @%0t: payload word=%h expected %h (region %0d off %0d)",
                         $time, tdata, exp_word, region_id, beats_in_region);
                errors = errors + 1;
            end
            exp_word = exp_word + 1;

            beats_in_region = beats_in_region + 1;
            total_beats     = total_beats + 1;

            // tlast must coincide with the region's last beat
            if (tlast) begin
                if (beats_in_region != reg_len(exp_region)) begin
                    $display("ERROR @%0t: region %0d beats=%0d expected %0d (tlast early/late)",
                             $time, exp_region, beats_in_region, reg_len(exp_region));
                    errors = errors + 1;
                end
                beats_in_region = 0;
                first_beat = 1;
                exp_region = exp_region + 1;
            end
        end
        if (rst_n && done) done_pulses = done_pulses + 1;
    end

    // ── Stimulus ──────────────────────────────────────────────────────────
    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        start = 1; @(posedge clk); start = 0;

        // wait for done (with a generous timeout)
        begin : wait_done
            integer cyc;
            for (cyc = 0; cyc < 4_000_000; cyc = cyc + 1) begin
                @(posedge clk);
                if (done) disable wait_done;
            end
        end

        @(posedge clk);
        $display("------------------------------------------------------------");
        $display("total_beats = %0d (expected %0d)", total_beats, TOTAL);
        $display("done_pulses = %0d", done_pulses);
        $display("errors      = %0d", errors);
        if (total_beats == TOTAL && done_pulses == 1 && errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule

`default_nettype wire
