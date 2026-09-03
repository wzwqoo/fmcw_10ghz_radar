// =============================================================================
//  tb_tdm_frontend3.v
//  Checks the 3-channel TDM front end:
//    * tdm_mux_3to1   — mod-3 counter, ch_id sequences 0,1,2 and data routes
//    * iq_demux_1to3  — routes a tagged TDM I/Q stream back to ch0/1/2
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_tdm_frontend3;
    localparam DATA_W = 16;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ── MUX under test ────────────────────────────────────────────────────
    reg  [DATA_W-1:0] ch0=16'hAAAA, ch1=16'hBBBB, ch2=16'hCCCC;
    wire [DATA_W-1:0] tdm_data;
    wire [1:0]        tdm_chid;
    wire              tdm_valid;

    tdm_mux_3to1 #(.DATA_W(DATA_W)) mux (
        .clk(clk), .rst_n(rst_n),
        .ch0_data(ch0), .ch1_data(ch1), .ch2_data(ch2),
        .ch0_vld(1'b1), .ch1_vld(1'b1), .ch2_vld(1'b1),
        .tdm_data(tdm_data), .tdm_ch_id(tdm_chid), .tdm_valid(tdm_valid)
    );

    integer mux_err = 0, mux_seen = 0;
    reg [1:0] prev_id; reg have_prev = 0;
    always @(posedge clk) begin
        if (rst_n && tdm_valid) begin
            // data must match the routed channel for the tag
            case (tdm_chid)
                2'd0: if (tdm_data !== 16'hAAAA) mux_err = mux_err + 1;
                2'd1: if (tdm_data !== 16'hBBBB) mux_err = mux_err + 1;
                2'd2: if (tdm_data !== 16'hCCCC) mux_err = mux_err + 1;
                default: mux_err = mux_err + 1;   // tag 3 must never appear
            endcase
            // tag must advance 0->1->2->0 (mod-3)
            if (have_prev) begin
                if (((prev_id == 2'd2) && (tdm_chid != 2'd0)) ||
                    ((prev_id != 2'd2) && (tdm_chid != prev_id + 2'd1)))
                    mux_err = mux_err + 1;
            end
            prev_id <= tdm_chid; have_prev <= 1;
            mux_seen = mux_seen + 1;
        end
    end

    // ── DEMUX under test ──────────────────────────────────────────────────
    reg  [DATA_W-1:0] i_d, q_d;
    reg  [1:0]        i_id;
    reg               i_v, q_v;
    wire [31:0] d0,d1,d2; wire v0,v1,v2;

    iq_demux_1to3 #(.IQ_W(DATA_W), .DATA_W(32)) dem (
        .clk(clk), .rst_n(rst_n),
        .i_tdm_data(i_d), .q_tdm_data(q_d), .i_tdm_ch_id(i_id),
        .i_tdm_valid(i_v), .q_tdm_valid(q_v),
        .ch0_tdata(d0), .ch0_tvalid(v0),
        .ch1_tdata(d1), .ch1_tvalid(v1),
        .ch2_tdata(d2), .ch2_tvalid(v2)
    );

    integer dem_err = 0;
    // Expected packed word for a given ch
    function [31:0] pack(input [15:0] i, input [15:0] q); pack = {q, i}; endfunction

    initial begin
        i_d=0; q_d=0; i_id=0; i_v=0; q_v=0;
        repeat (3) @(posedge clk);
        rst_n = 1;

        // Let the mux run a few full rounds.
        repeat (12) @(posedge clk);

        // ── Drive demux with tagged beats 0,1,2 ──────────────────────────
        drive_demux(2'd0, 16'h1111, 16'h2222);
        drive_demux(2'd1, 16'h3333, 16'h4444);
        drive_demux(2'd2, 16'h5555, 16'h6666);
        @(negedge clk); i_v=0; q_v=0;
        repeat (3) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("MUX: beats=%0d errors=%0d", mux_seen, mux_err);
        $display("DEMUX: errors=%0d", dem_err);
        if (mux_err==0 && dem_err==0 && mux_seen>0) $display("RESULT: PASS");
        else                                        $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

    task drive_demux(input [1:0] id, input [15:0] iv, input [15:0] qv);
        begin
            @(negedge clk);
            i_d=iv; q_d=qv; i_id=id; i_v=1; q_v=1;
            @(posedge clk);           // demux registers on this edge
            #1;
            case (id)
                2'd0: if (!(v0 && d0===pack(iv,qv))) dem_err=dem_err+1;
                2'd1: if (!(v1 && d1===pack(iv,qv))) dem_err=dem_err+1;
                2'd2: if (!(v2 && d2===pack(iv,qv))) dem_err=dem_err+1;
            endcase
        end
    endtask

endmodule

`default_nettype wire
