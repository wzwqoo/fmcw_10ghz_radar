// =============================================================================
//  tb_radar_dump_regs.v
//  Exercises the slimmed radar_dump_axi_regs (software does peak/SNR; this block
//  is just frame-ready status + dump control + halt):
//    * frame_complete -> STATUS.new_frame + cur_set
//    * CTRL.start_dump | set_halt -> dump_start with DUMP_SEL/FID, halt asserted
//    * dump_done -> STATUS.done_sticky;  CTRL clears flags
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_radar_dump_regs;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg         frame_complete, frame_set;
    reg  [6:0]  awaddr; reg awvalid; wire awready;
    reg  [31:0] wdata;  reg wvalid;  wire wready;
    wire [1:0]  bresp;  wire bvalid; reg bready;
    reg  [6:0]  araddr; reg arvalid; wire arready;
    wire [31:0] rdata;  wire [1:0] rresp; wire rvalid; reg rready;

    wire        dump_start, dump_fsel, halt; wire [31:0] dump_fid;
    reg         dump_busy, dump_done;

    radar_dump_axi_regs dut (
        .s_axi_aclk(clk), .s_axi_aresetn(rst_n),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .frame_complete(frame_complete), .frame_set(frame_set),
        .dump_start(dump_start), .dump_frame_buf_sel(dump_fsel),
        .dump_frame_id(dump_fid),
        .dump_busy(dump_busy), .dump_done(dump_done), .halt(halt)
    );

    integer errors = 0;
    task chk(input [31:0] got, exp, input [127:0 ] tag);
        if (got !== exp) begin $display("ERROR %0s got %h exp %h", tag, got, exp); errors=errors+1; end
    endtask

    integer dump_fires = 0; reg last_fsel; reg [31:0] last_fid;
    always @(posedge clk) if (rst_n && dump_start) begin
        dump_fires = dump_fires + 1; last_fsel<=dump_fsel; last_fid<=dump_fid;
    end

    task axi_write(input [6:0] a, input [31:0] d);
        begin
            @(posedge clk); #1; awaddr=a; awvalid=1; wdata=d; wvalid=1; bready=1;
            @(posedge clk); #1; while(!(awready&&wready)) begin @(posedge clk); #1; end
            awvalid=0; wvalid=0;
            while(!bvalid) begin @(posedge clk); #1; end
            @(posedge clk); #1; bready=0;
        end
    endtask
    task axi_read(input [6:0] a, output [31:0] d);
        begin
            @(posedge clk); #1; araddr=a; arvalid=1; rready=1;
            @(posedge clk); #1; while(!arready) begin @(posedge clk); #1; end
            arvalid=0;
            while(!rvalid) begin @(posedge clk); #1; end
            d = rdata;
            @(posedge clk); #1; rready=0;
        end
    endtask

    reg [31:0] rd;
    initial begin
        frame_complete=0; frame_set=0;
        awvalid=0; wvalid=0; bready=0; arvalid=0; rready=0; awaddr=0; araddr=0; wdata=0;
        dump_busy=0; dump_done=0;
        repeat(3) @(posedge clk); rst_n=1; repeat(2) @(posedge clk);

        // A frame completes in set 1.
        @(negedge clk); frame_complete<=1; frame_set<=1;
        @(posedge clk); @(negedge clk); frame_complete<=0;
        repeat(2) @(posedge clk);

        axi_read(7'h0C, rd);  // STATUS
        chk(rd & 32'h104, 32'h104, "STATUS new_frame + cur_set"); // [2]new_frame|[8]cur_set

        // Program DUMP_SEL = frame_buf_sel=1, FID = 0x20.
        axi_write(7'h04, 32'h1);
        axi_write(7'h08, 32'h20);
        // CTRL: start_dump(0) | set_halt(3) = 0x9
        axi_write(7'h00, 32'h9);
        repeat(3) @(posedge clk);
        chk(dump_fires, 32'd1, "dump fired");
        chk({31'd0,last_fsel}, 32'd1, "dump frame_buf_sel=1");
        chk(last_fid, 32'h20, "dump frame_id");
        chk({31'd0,halt}, 32'd1, "halt asserted");

        // Complete dump -> done_sticky
        @(negedge clk); dump_done<=1; @(posedge clk); @(negedge clk); dump_done<=0;
        repeat(2) @(posedge clk);
        axi_read(7'h0C, rd); chk(rd & 32'h2, 32'h2, "done_sticky set");

        // Clear: newframe(1)|done(2)|halt(4)=0x16
        axi_write(7'h00, 32'h16);
        repeat(2) @(posedge clk);
        axi_read(7'h0C, rd);
        chk(rd & 32'h0E, 32'h0, "new_frame/done/halt cleared");
        chk({31'd0,halt}, 32'd0, "halt deasserted");

        $display("------------------------------------------------------------");
        $display("dump_fires=%0d errors=%0d", dump_fires, errors);
        if (errors==0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end
endmodule

`default_nettype wire
