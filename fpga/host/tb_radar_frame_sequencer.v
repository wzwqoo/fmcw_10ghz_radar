// tb_radar_frame_sequencer.v — checks bundle_ready pulse, frame_set report,
// frame_buf_sel toggle, and halt-holds-the-toggle.
`timescale 1ns/1ps
`default_nettype none

module tb_radar_frame_sequencer;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  cache_ready, frame_complete, cfar_sel, halt;
    wire frame_buf_sel, bundle_ready, frame_set;

    radar_frame_sequencer dut (
        .clk(clk), .rst_n(rst_n),
        .cache_ready(cache_ready), .frame_complete(frame_complete),
        .cfar_frame_buf_sel(cfar_sel), .halt(halt),
        .frame_buf_sel(frame_buf_sel), .bundle_ready(bundle_ready), .frame_set(frame_set)
    );

    integer errors = 0, bundles = 0;
    always @(posedge clk) if (rst_n && bundle_ready) bundles = bundles + 1;
    task chk(input c, input [127:0] m); if(!c) begin $display("FAIL: %0s",m); errors=errors+1; end endtask

    // One frame: cache_ready pulse, then frame_complete pulse with cfar_sel = set.
    task run_frame(input sel);
        begin
            cfar_sel = sel;
            @(negedge clk); cache_ready = 1; @(negedge clk); cache_ready = 0;
            repeat(2) @(negedge clk);
            @(negedge clk); frame_complete = 1; @(negedge clk); frame_complete = 0;
            @(negedge clk);
        end
    endtask

    initial begin
        cache_ready=0; frame_complete=0; cfar_sel=0; halt=0;
        repeat(3) @(negedge clk); rst_n=1; @(negedge clk);

        chk(frame_buf_sel==0, "start set 0");

        // Frame in set 0 -> bundle_ready, frame_set=0, toggle to 1.
        run_frame(1'b0);
        chk(frame_set==0, "frame_set reports 0");
        chk(frame_buf_sel==1, "toggled to 1");

        // Frame in set 1 -> frame_set=1, toggle back to 0.
        run_frame(1'b1);
        chk(frame_set==1, "frame_set reports 1");
        chk(frame_buf_sel==0, "toggled to 0");

        // Halt asserted: bundle still reported, but set must NOT toggle.
        halt = 1;
        run_frame(1'b0);
        chk(frame_set==0, "halt: frame_set still reported");
        chk(frame_buf_sel==0, "halt: set held (no toggle)");
        halt = 0;

        chk(bundles==3, "three bundle_ready pulses");

        $display("------------------------------------------------------------");
        $display("bundles=%0d errors=%0d", bundles, errors);
        if (errors==0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
endmodule

`default_nettype wire
