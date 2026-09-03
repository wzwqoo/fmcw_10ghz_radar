// =============================================================================
//  tb_e2e_3rx.v  —  self-contained 3-RX DDR round-trip end-to-end sim.
//
//  Exercises the real radar_3rx RTL that the 4->3 conversion touched, through a
//  full DDR round trip, with behavioral stand-ins for the Xilinx FFT IP so it
//  runs in plain iverilog (no Vivado / no IP), in seconds:
//
//    3x range-FFT stream (synthetic)
//        -> scatter_write_3rx      (3 AXI write masters, transposed store)
//        -> behavioral DDR (region A ping/pong)
//        -> doppler_seq_3rx        (3 AXI read masters, per-range-bin bursts)
//        -> identity "doppler FFT" (1-deep skid per lane, preserves tlast)
//        -> noncoh_integrator      (sum |X|^2 across the 3 RX)
//
//  Speed technique (à la test 05): tiny frame dims (N_RANGE=4, N_CHIRPS=8) via
//  parameter override, so a full frame is 32 cells and the sim finishes fast.
//
//  Check: the summed-power output equals sum_rx(I^2+Q^2) in doppler (k,n) order,
//  proving the 3-lane scatter/transpose/doppler/integrate wiring is correct.
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

// 1-deep skid = identity "doppler FFT": registers a beat, holds it (with tlast)
// under backpressure so the 3 lanes rendezvous at the integrator.
module axis_skid #(parameter W = 33) (
    input  wire        clk, rst_n,
    input  wire [W-1:0] s_tdata, input wire s_tvalid, output wire s_tready, input wire s_tlast,
    output wire [W-1:0] m_tdata, output wire m_tvalid, input wire m_tready, output wire m_tlast
);
    reg full; reg [W-1:0] d; reg l;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin full <= 1'b0; d <= {W{1'b0}}; l <= 1'b0; end
        else if (!full && s_tvalid) begin d <= s_tdata; l <= s_tlast; full <= 1'b1; end
        else if (full && m_tready)   full <= 1'b0;
    end
    assign s_tready = !full;
    assign m_tvalid = full;
    assign m_tdata  = d;
    assign m_tlast  = l;
endmodule


module tb_e2e_3rx;
    localparam integer NR = 4, NC = 8;         // small frame for speed
    localparam integer DW = 32, IQW = 16, SB = 4;
    localparam integer RX_STRIDE = NR*NC*SB;   // 128 bytes / RX
    localparam [31:0]  A_BASE = 32'd0;
    localparam [31:0]  B_BASE = 32'd512;       // > 3*RX_STRIDE, distinct region
    localparam integer DDR_WORDS = 1024;
    localparam integer NCELL = NR*NC;          // 32

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg [31:0] ddr [0:DDR_WORDS-1];

    // ── Stimulus values: V[rx][n][k], I in high half, Q=0 ──────────────────
    function [15:0] ival(input integer rx, input integer n, input integer k);
        ival = rx*100 + n*10 + k + 1;
    endfunction

    // ── DUT-facing AXI arrays (scalar module ports wired to [g]) ───────────
    // scatter write masters
    wire [31:0] sc_awaddr [0:2]; wire [7:0] sc_awlen [0:2]; wire [2:0] sc_awsize [0:2];
    wire [1:0]  sc_awburst[0:2]; wire sc_awvalid[0:2]; reg sc_awready[0:2];
    wire [31:0] sc_wdata [0:2]; wire [3:0] sc_wstrb[0:2]; wire sc_wlast[0:2]; wire sc_wvalid[0:2]; reg sc_wready[0:2];
    reg  [1:0]  sc_bresp [0:2]; reg sc_bvalid[0:2]; wire sc_bready[0:2];
    // doppler read masters
    wire [31:0] ds_araddr[0:2]; wire [7:0] ds_arlen[0:2]; wire [2:0] ds_arsize[0:2];
    wire [1:0]  ds_arburst[0:2]; wire ds_arvalid[0:2]; reg ds_arready[0:2];
    reg  [31:0] ds_rdata [0:2]; reg [1:0] ds_rresp[0:2]; reg ds_rlast[0:2]; reg ds_rvalid[0:2]; wire ds_rready[0:2];

    // scatter <-> doppler handshake
    wire [31:0] rd0b, rd1b, rd2b;
    wire [2:0]  rd_buf_valid, dfd, sc_busy, sc_stall;

    // scatter inputs
    reg  [31:0] sin_tdata [0:2]; reg sin_tvalid[0:2]; wire sin_tready[0:2];

    // doppler stream out -> skid -> noncoh
    wire [31:0] ds_tdata[0:2]; wire ds_tvalid[0:2]; wire ds_tlast[0:2]; wire ds_tready[0:2];
    wire [31:0] fx_tdata[0:2]; wire fx_tvalid[0:2]; wire fx_tlast[0:2]; wire fx_tready[0:2];

    // ── scatter_write_3rx ──────────────────────────────────────────────────
    scatter_write_3rx #(.DATA_W(DW), .ADDR_W(32), .N_RANGE_BINS(NR), .N_CHIRPS(NC),
                        .SAMPLE_BYTES(SB), .BUF_A_BASE(A_BASE), .BUF_B_BASE(B_BASE))
    u_scatter (
        .clk(clk), .rst_n(rst_n),
        .s0_tdata(sin_tdata[0]), .s0_tvalid(sin_tvalid[0]), .s0_tready(sin_tready[0]),
        .s1_tdata(sin_tdata[1]), .s1_tvalid(sin_tvalid[1]), .s1_tready(sin_tready[1]),
        .s2_tdata(sin_tdata[2]), .s2_tvalid(sin_tvalid[2]), .s2_tready(sin_tready[2]),
        .m0_awaddr(sc_awaddr[0]), .m0_awlen(sc_awlen[0]), .m0_awsize(sc_awsize[0]), .m0_awburst(sc_awburst[0]),
        .m0_awvalid(sc_awvalid[0]), .m0_awready(sc_awready[0]), .m0_wdata(sc_wdata[0]), .m0_wstrb(sc_wstrb[0]),
        .m0_wlast(sc_wlast[0]), .m0_wvalid(sc_wvalid[0]), .m0_wready(sc_wready[0]),
        .m0_bresp(sc_bresp[0]), .m0_bvalid(sc_bvalid[0]), .m0_bready(sc_bready[0]),
        .m1_awaddr(sc_awaddr[1]), .m1_awlen(sc_awlen[1]), .m1_awsize(sc_awsize[1]), .m1_awburst(sc_awburst[1]),
        .m1_awvalid(sc_awvalid[1]), .m1_awready(sc_awready[1]), .m1_wdata(sc_wdata[1]), .m1_wstrb(sc_wstrb[1]),
        .m1_wlast(sc_wlast[1]), .m1_wvalid(sc_wvalid[1]), .m1_wready(sc_wready[1]),
        .m1_bresp(sc_bresp[1]), .m1_bvalid(sc_bvalid[1]), .m1_bready(sc_bready[1]),
        .m2_awaddr(sc_awaddr[2]), .m2_awlen(sc_awlen[2]), .m2_awsize(sc_awsize[2]), .m2_awburst(sc_awburst[2]),
        .m2_awvalid(sc_awvalid[2]), .m2_awready(sc_awready[2]), .m2_wdata(sc_wdata[2]), .m2_wstrb(sc_wstrb[2]),
        .m2_wlast(sc_wlast[2]), .m2_wvalid(sc_wvalid[2]), .m2_wready(sc_wready[2]),
        .m2_bresp(sc_bresp[2]), .m2_bvalid(sc_bvalid[2]), .m2_bready(sc_bready[2]),
        .rd0_buf_base(rd0b), .rd1_buf_base(rd1b), .rd2_buf_base(rd2b),
        .rd_buf_valid(rd_buf_valid), .doppler_frame_done(dfd),
        .busy(sc_busy), .stall(sc_stall)
    );

    // ── doppler_seq_3rx ─────────────────────────────────────────────────────
    doppler_seq_3rx #(.DATA_W(DW), .ADDR_W(32), .N_RANGE_BINS(NR), .N_CHIRPS(NC), .SAMPLE_BYTES(SB))
    u_dseq (
        .clk(clk), .rst_n(rst_n),
        .rd0_buf_base(rd0b), .rd1_buf_base(rd1b), .rd2_buf_base(rd2b),
        .rd_buf_valid(rd_buf_valid), .doppler_frame_done(dfd),
        .m0_araddr(ds_araddr[0]), .m0_arlen(ds_arlen[0]), .m0_arsize(ds_arsize[0]), .m0_arburst(ds_arburst[0]),
        .m0_arvalid(ds_arvalid[0]), .m0_arready(ds_arready[0]), .m0_rdata(ds_rdata[0]), .m0_rresp(ds_rresp[0]),
        .m0_rlast(ds_rlast[0]), .m0_rvalid(ds_rvalid[0]), .m0_rready(ds_rready[0]),
        .m1_araddr(ds_araddr[1]), .m1_arlen(ds_arlen[1]), .m1_arsize(ds_arsize[1]), .m1_arburst(ds_arburst[1]),
        .m1_arvalid(ds_arvalid[1]), .m1_arready(ds_arready[1]), .m1_rdata(ds_rdata[1]), .m1_rresp(ds_rresp[1]),
        .m1_rlast(ds_rlast[1]), .m1_rvalid(ds_rvalid[1]), .m1_rready(ds_rready[1]),
        .m2_araddr(ds_araddr[2]), .m2_arlen(ds_arlen[2]), .m2_arsize(ds_arsize[2]), .m2_arburst(ds_arburst[2]),
        .m2_arvalid(ds_arvalid[2]), .m2_arready(ds_arready[2]), .m2_rdata(ds_rdata[2]), .m2_rresp(ds_rresp[2]),
        .m2_rlast(ds_rlast[2]), .m2_rvalid(ds_rvalid[2]), .m2_rready(ds_rready[2]),
        .m0_axis_tdata(ds_tdata[0]), .m0_axis_tvalid(ds_tvalid[0]), .m0_axis_tlast(ds_tlast[0]), .m0_axis_tready(ds_tready[0]),
        .m1_axis_tdata(ds_tdata[1]), .m1_axis_tvalid(ds_tvalid[1]), .m1_axis_tlast(ds_tlast[1]), .m1_axis_tready(ds_tready[1]),
        .m2_axis_tdata(ds_tdata[2]), .m2_axis_tvalid(ds_tvalid[2]), .m2_axis_tlast(ds_tlast[2]), .m2_axis_tready(ds_tready[2]),
        .fft0_out_valid(fx_tvalid[0]), .fft0_out_last(fx_tlast[0]),
        .fft1_out_valid(fx_tvalid[1]), .fft1_out_last(fx_tlast[1]),
        .fft2_out_valid(fx_tvalid[2]), .fft2_out_last(fx_tlast[2]),
        .busy()
    );

    // ── identity "doppler FFT" per lane (skid preserves data + tlast) ──────
    genvar g;
    generate for (g = 0; g < 3; g = g + 1) begin : g_fft
        axis_skid #(.W(DW)) u_skid (
            .clk(clk), .rst_n(rst_n),
            .s_tdata(ds_tdata[g]), .s_tvalid(ds_tvalid[g]), .s_tready(ds_tready[g]), .s_tlast(ds_tlast[g]),
            .m_tdata(fx_tdata[g]), .m_tvalid(fx_tvalid[g]), .m_tready(fx_tready[g]), .m_tlast(fx_tlast[g])
        );
    end endgenerate

    // ── noncoh_integrator (3 lanes) ─────────────────────────────────────────
    wire [31:0] nc_tdata; wire nc_tvalid, nc_tlast;
    noncoh_integrator #(.IQ_W(IQW), .OUT_W(32)) u_noncoh (
        .clk(clk), .rst_n(rst_n),
        .s0_tdata(fx_tdata[0]), .s0_tvalid(fx_tvalid[0]), .s0_tlast(fx_tlast[0]), .s0_tready(fx_tready[0]),
        .s1_tdata(fx_tdata[1]), .s1_tvalid(fx_tvalid[1]), .s1_tlast(fx_tlast[1]), .s1_tready(fx_tready[1]),
        .s2_tdata(fx_tdata[2]), .s2_tvalid(fx_tvalid[2]), .s2_tlast(fx_tlast[2]), .s2_tready(fx_tready[2]),
        .m_axis_tdata(nc_tdata), .m_axis_tvalid(nc_tvalid), .m_axis_tlast(nc_tlast), .m_axis_tready(1'b1)
    );

    // ── Behavioral DDR: 3 write slaves + 3 read slaves over shared ddr[] ────
    genvar w;
    generate for (w = 0; w < 3; w = w + 1) begin : g_wr
        reg [31:0] a; reg [8:0] beats; reg active;
        initial begin sc_awready[w]=1'b1; sc_wready[w]=1'b1; sc_bresp[w]=2'b00; sc_bvalid[w]=1'b0;
                      a=0; beats=0; active=0; end
        always @(negedge clk) begin
            if (!rst_n) begin sc_awready[w]<=1'b1; sc_wready[w]<=1'b1; sc_bvalid[w]<=1'b0; active<=1'b0; beats<=0; end
            else begin
                sc_awready[w] <= !active;
                sc_wready[w]  <= 1'b1;
                if (sc_bvalid[w] && sc_bready[w]) sc_bvalid[w] <= 1'b0;
                if (!active && sc_awvalid[w] && sc_awready[w]) begin
                    a <= sc_awaddr[w]; beats <= {1'b0, sc_awlen[w]} + 9'd1; active <= 1'b1;
                end
                if (sc_wvalid[w] && sc_wready[w] && (active || (sc_awvalid[w]&&sc_awready[w]))) begin
                    ddr[(active ? a : sc_awaddr[w]) >> 2] <= sc_wdata[w];
                    if ((active ? beats : 9'd1) == 9'd1) begin
                        active <= 1'b0; beats <= 9'd0; sc_bvalid[w] <= 1'b1; sc_bresp[w] <= 2'b00;
                    end else begin
                        beats <= beats - 9'd1; a <= a + (32'd1 << sc_awsize[w]);
                    end
                end
            end
        end
    end endgenerate

    generate for (w = 0; w < 3; w = w + 1) begin : g_rd
        reg [31:0] a; reg [8:0] beats; reg active; reg accepted;
        always @(posedge clk or negedge rst_n)
            if (!rst_n) accepted <= 1'b0; else accepted <= ds_rvalid[w] && ds_rready[w];
        initial begin ds_arready[w]=1'b1; ds_rdata[w]=0; ds_rresp[w]=0; ds_rlast[w]=0; ds_rvalid[w]=0;
                      a=0; beats=0; active=0; accepted=0; end
        always @(negedge clk) begin
            if (!rst_n) begin ds_arready[w]<=1'b1; ds_rvalid[w]<=1'b0; ds_rlast[w]<=1'b0; active<=1'b0; beats<=0; end
            else begin
                ds_arready[w] <= (!active && !(ds_rvalid[w] && !accepted));
                if (ds_rvalid[w] && !accepted) begin
                    ds_rvalid[w] <= ds_rvalid[w];  // hold until accepted
                end else if (active) begin
                    ds_rdata[w] <= ddr[a >> 2]; ds_rresp[w] <= 2'b00;
                    ds_rlast[w] <= (beats == 9'd1); ds_rvalid[w] <= 1'b1;
                    if (beats == 9'd1) begin active <= 1'b0; beats <= 9'd0; end
                    else begin beats <= beats - 9'd1; a <= a + (32'd1 << ds_arsize[w]); end
                end else if (ds_arvalid[w] && ds_arready[w]) begin
                    ds_rvalid[w] <= 1'b0; ds_rlast[w] <= 1'b0; active <= 1'b1;
                    beats <= {1'b0, ds_arlen[w]} + 9'd1; a <= ds_araddr[w];
                end else begin
                    ds_rvalid[w] <= 1'b0; ds_rlast[w] <= 1'b0;
                end
            end
        end
    end endgenerate

    // ── Stimulus: feed one frame to each RX in (chirp n outer, bin k inner) ─
    // AUTOMATIC so the 3 concurrent fork branches each get private loop vars.
    integer rx, n, k;
    task automatic feed_rx(input integer rxid);
        integer nn, kk;
        begin
            for (nn = 0; nn < NC; nn = nn + 1)
                for (kk = 0; kk < NR; kk = kk + 1) begin
                    @(negedge clk);
                    sin_tdata[rxid]  <= {ival(rxid, nn, kk), 16'd0};  // {I, Q}
                    sin_tvalid[rxid] <= 1'b1;
                    @(posedge clk);
                    while (!sin_tready[rxid]) @(posedge clk);
                end
            @(negedge clk); sin_tvalid[rxid] <= 1'b0;
        end
    endtask

    // ── Expected noncoh output (doppler order: k outer, n inner) ───────────
    reg [31:0] exp_q [0:NCELL-1];
    integer ei, rr;
    initial begin
        ei = 0;
        for (k = 0; k < NR; k = k + 1)
            for (n = 0; n < NC; n = n + 1) begin
                exp_q[ei] = 0;
                for (rr = 0; rr < 3; rr = rr + 1)
                    exp_q[ei] = exp_q[ei] + ival(rr,n,k) * ival(rr,n,k);
                ei = ei + 1;
            end
    end

    // ── Output checker ─────────────────────────────────────────────────────
    integer got = 0, errors = 0;
    always @(posedge clk) begin
        if (rst_n && nc_tvalid) begin
            if (got < NCELL) begin
                if (nc_tdata !== exp_q[got]) begin
                    $display("ERROR cell %0d: got %0d exp %0d", got, nc_tdata, exp_q[got]);
                    errors = errors + 1;
                end
            end
            got = got + 1;
        end
    end

    // ── Drive ───────────────────────────────────────────────────────────────
    integer i;
    initial begin
        for (i = 0; i < DDR_WORDS; i = i + 1) ddr[i] = 32'd0;
        for (i = 0; i < 3; i = i + 1) begin sin_tdata[i] = 0; sin_tvalid[i] = 0; end
        repeat (6) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);

        // Feed all 3 RX concurrently.
        fork feed_rx(0); feed_rx(1); feed_rx(2); join

        // Wait for the frame to flow through (bounded timeout).
        begin : wait_done
            for (i = 0; i < 200000; i = i + 1) begin
                @(posedge clk);
                if (got >= NCELL) disable wait_done;
            end
        end

        repeat (20) @(posedge clk);
        $display("------------------------------------------------------------");
        $display("noncoh cells captured = %0d (expected %0d), errors = %0d", got, NCELL, errors);
        if (got == NCELL && errors == 0) $display("RESULT: PASS");
        else                              $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin  // global watchdog
        #50_000_000;
        $display("RESULT: FAIL (timeout)"); $finish;
    end
endmodule

`default_nettype wire
