// =============================================================================
//  tb_noncoh_integrator3.v
//  Self-checking unit test for the 3-RX noncoh_integrator (the adder-tree edit).
//  Verifies out = sat32( |X0|^2 + |X1|^2 + |X2|^2 ) and tlast passthrough.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_noncoh_integrator3;
    localparam IQ_W = 16, OUT_W = 32;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [2*IQ_W-1:0] s0,s1,s2;
    reg               sv, sl;
    wire              s0r,s1r,s2r;
    wire [OUT_W-1:0]  m_tdata;
    wire              m_tvalid, m_tlast;
    reg               m_tready;

    noncoh_integrator #(.IQ_W(IQ_W), .OUT_W(OUT_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .s0_tdata(s0), .s0_tvalid(sv), .s0_tlast(sl), .s0_tready(s0r),
        .s1_tdata(s1), .s1_tvalid(sv), .s1_tlast(sl), .s1_tready(s1r),
        .s2_tdata(s2), .s2_tvalid(sv), .s2_tlast(sl), .s2_tready(s2r),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid),
        .m_axis_tlast(m_tlast), .m_axis_tready(m_tready)
    );

    // Reference model queue
    integer NTEST = 8;
    reg [OUT_W-1:0] exp_q [0:255];
    reg             exp_l [0:255];
    integer wr = 0, rd = 0, errors = 0;

    function [OUT_W-1:0] mag2_sum(input signed [IQ_W-1:0] i0,q0,i1,q1,i2,q2);
        reg [63:0] s;
        begin
            s = i0*i0 + q0*q0 + i1*i1 + q1*q1 + i2*i2 + q2*q2;
            if (s > {OUT_W{1'b1}}) mag2_sum = {OUT_W{1'b1}};
            else                   mag2_sum = s[OUT_W-1:0];
        end
    endfunction

    // One clean one-cycle valid pulse per cell (pipeline has no backpressure,
    // so a gap between cells just produces a gap in output valid).
    task drive(input signed [IQ_W-1:0] i0,q0,i1,q1,i2,q2, input last);
        begin
            // Drive on negedge so inputs are stable before the sampling posedge
            // (avoids a TB/DUT race at the valid de-assert edge).
            @(negedge clk);
            s0 = {i0,q0}; s1 = {i1,q1}; s2 = {i2,q2};
            sv = 1'b1; sl = last;
            exp_q[wr] = mag2_sum(i0,q0,i1,q1,i2,q2);
            exp_l[wr] = last;
            wr = wr + 1;
            @(negedge clk);
            sv = 1'b0; sl = 1'b0;
        end
    endtask

    // Output checker
    always @(posedge clk) begin
        if (rst_n && m_tvalid && m_tready) begin
            if (m_tdata !== exp_q[rd]) begin
                $display("ERROR cell %0d: got %h exp %h", rd, m_tdata, exp_q[rd]);
                errors = errors + 1;
            end
            if (m_tlast !== exp_l[rd]) begin
                $display("ERROR cell %0d: tlast got %b exp %b", rd, m_tlast, exp_l[rd]);
                errors = errors + 1;
            end
            rd = rd + 1;
        end
    end

    initial begin
        s0=0; s1=0; s2=0; sv=0; sl=0; m_tready=1;
        repeat (3) @(posedge clk);
        rst_n = 1;

        drive( 100,  0,   200, 0,   300, 0,   0);     // 100^2+200^2+300^2 = 140000
        drive( 1,    2,   3,   4,   5,   6,   0);     // 1+4+9+16+25+36 = 91
        drive(-32768,0,  -32768,0, -32768,0,  0);     // 3 * 2^30 = 3221225472 (fits 32b)
        drive(32767,32767, 32767,32767, 32767,32767, 0); // saturates -> 0xFFFFFFFF
        drive(-5, 7,  11, -13,  -17, 19,  0);
        drive( 0, 0,   0, 0,    0, 0,    1);          // tlast cell

        repeat (10) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("cells checked = %0d, errors = %0d", rd, errors);
        if (rd == wr && errors == 0) $display("RESULT: PASS");
        else                          $display("RESULT: FAIL (rd=%0d wr=%0d)", rd, wr);
        $display("------------------------------------------------------------");
        $finish;
    end
endmodule

`default_nettype wire
