`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// tb_radar_pipeline_e2e.v
//
// Single-file end-to-end testbench for the radar 01..08 B/C/D ping-pong path.
//
// Merges the former 01_tb_full_pipeline_top.v (DUT wiring) and
// 02_tb_full_pipeline_xsim_e2e_top.v (stimulus/checking) into one module.
//
// Benefits vs the two-file split:
//   - No hierarchical dut.signal references => no XSim initial-block
//     scheduling races (the bvalid race that required the bvalid_pending fix)
//   - No intermediate port boundary => no FIFOs invented to paper over it
//   - DUT wiring matches radar_test.v block design exactly: direct connections,
//     no axis_fifo_simple anywhere in the signal path
//   - One file to maintain instead of two
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

`define AXI_WR_DECL(P) \
    wire [31:0] P``_awaddr; \
    wire [7:0]  P``_awlen; \
    wire [2:0]  P``_awsize; \
    wire [1:0]  P``_awburst; \
    wire        P``_awvalid; \
    reg         P``_awready; \
    wire [31:0] P``_wdata; \
    wire [3:0]  P``_wstrb; \
    wire        P``_wlast; \
    wire        P``_wvalid; \
    reg         P``_wready; \
    reg  [1:0]  P``_bresp; \
    reg         P``_bvalid; \
    wire        P``_bready

`define AXI_RD_DECL(P) \
    wire [31:0] P``_araddr; \
    wire [7:0]  P``_arlen; \
    wire [2:0]  P``_arsize; \
    wire [1:0]  P``_arburst; \
    wire        P``_arvalid; \
    reg         P``_arready; \
    reg  [31:0] P``_rdata; \
    reg  [1:0]  P``_rresp; \
    reg         P``_rlast; \
    reg         P``_rvalid; \
    wire        P``_rready

`define AXI_WR_PORTS(PORT, BUS) \
    .PORT``_awaddr(BUS``_awaddr), \
    .PORT``_awlen(BUS``_awlen), \
    .PORT``_awsize(BUS``_awsize), \
    .PORT``_awburst(BUS``_awburst), \
    .PORT``_awvalid(BUS``_awvalid), \
    .PORT``_awready(BUS``_awready), \
    .PORT``_wdata(BUS``_wdata), \
    .PORT``_wstrb(BUS``_wstrb), \
    .PORT``_wlast(BUS``_wlast), \
    .PORT``_wvalid(BUS``_wvalid), \
    .PORT``_wready(BUS``_wready), \
    .PORT``_bresp(BUS``_bresp), \
    .PORT``_bvalid(BUS``_bvalid), \
    .PORT``_bready(BUS``_bready)

`define AXI_RD_PORTS(PORT, BUS) \
    .PORT``_araddr(BUS``_araddr), \
    .PORT``_arlen(BUS``_arlen), \
    .PORT``_arsize(BUS``_arsize), \
    .PORT``_arburst(BUS``_arburst), \
    .PORT``_arvalid(BUS``_arvalid), \
    .PORT``_arready(BUS``_arready), \
    .PORT``_rdata(BUS``_rdata), \
    .PORT``_rresp(BUS``_rresp), \
    .PORT``_rlast(BUS``_rlast), \
    .PORT``_rvalid(BUS``_rvalid), \
    .PORT``_rready(BUS``_rready)

`define AXI_WRITE_SLAVE(P) \
    reg [31:0] P``_wr_addr; \
    reg [8:0]  P``_wr_beats_left; \
    reg [2:0]  P``_wr_size; \
    reg [1:0]  P``_wr_burst; \
    reg        P``_wr_active; \
    // bvalid_pending: bvalid was SET this negedge; do not clear until next negedge so \
    // the DUT's posedge always-block has a guaranteed window to sample it. \
    // Without this guard, XSim's initial-block negedge scheduling can set and clear \
    // bvalid in consecutive negedges straddling a posedge, making the DUT miss it. \
    reg        P``_bvalid_pending; \
    initial begin : P``_write_slave \
        P``_wr_addr = 32'd0; \
        P``_wr_beats_left = 9'd0; \
        P``_wr_size = 3'd2; \
        P``_wr_burst = 2'd1; \
        P``_wr_active = 1'b0; \
        P``_bvalid_pending = 1'b0; \
        P``_awready = 1'b1; \
        P``_wready = 1'b1; \
        P``_bresp = 2'b00; \
        P``_bvalid = 1'b0; \
        forever begin \
            @(negedge clk); \
            if (!rst_n) begin \
                P``_wr_addr = 32'd0; \
                P``_wr_beats_left = 9'd0; \
                P``_wr_size = 3'd2; \
                P``_wr_burst = 2'd1; \
                P``_wr_active = 1'b0; \
                P``_bvalid_pending = 1'b0; \
                P``_awready = 1'b1; \
                P``_wready = 1'b1; \
                P``_bresp = 2'b00; \
                P``_bvalid = 1'b0; \
            end else begin \
                P``_awready = !P``_wr_active; \
                P``_wready = 1'b1; \
                // Only clear bvalid if it was already asserted on the PREVIOUS negedge \
                // (P_bvalid_pending=0 now) so the DUT posedge between the two negedges \
                // has a full half-cycle window to sample bvalid=1. \
                if (P``_bvalid && P``_bready && !P``_bvalid_pending) begin \
                    P``_bvalid = 1'b0; \
                end \
                P``_bvalid_pending = 1'b0; \
                if (!P``_wr_active && P``_awvalid && P``_awready) begin \
                    P``_wr_addr = P``_awaddr; \
                    P``_wr_beats_left = {1'b0, P``_awlen} + 9'd1; \
                    P``_wr_size = P``_awsize; \
                    P``_wr_burst = P``_awburst; \
                    P``_wr_active = 1'b1; \
                end \
                if (P``_wvalid && P``_wready) begin \
                    if (!P``_wr_active) begin \
                        $display("ERROR: %s W beat before AW at %0t", `"P`", $time); \
                        $fatal(1); \
                    end \
                    ddr_write32(P``_wr_addr, P``_wdata, P``_wstrb); \
                    note_write(P``_wr_addr, P``_wdata); \
                    if (P``_wr_beats_left == 9'd1) begin \
                        if (!P``_wlast) begin \
                            $display("ERROR: %s missing WLAST at final beat, addr=0x%08x", `"P`", P``_wr_addr); \
                            $fatal(1); \
                        end \
                        P``_wr_active = 1'b0; \
                        P``_wr_beats_left = 9'd0; \
                        P``_bresp = 2'b00; \
                        P``_bvalid = 1'b1; \
                        P``_bvalid_pending = 1'b1; \
                    end else begin \
                        if (P``_wlast) begin \
                            $display("ERROR: %s early WLAST, addr=0x%08x beats_left=%0d", `"P`", P``_wr_addr, P``_wr_beats_left); \
                            $fatal(1); \
                        end \
                        P``_wr_beats_left = P``_wr_beats_left - 9'd1; \
                        if (P``_wr_burst == 2'd1) begin \
                            P``_wr_addr = P``_wr_addr + (32'd1 << P``_wr_size); \
                        end \
                    end \
                end \
            end \
        end \
    end

`define AXI_READ_SLAVE(P) \
    reg [31:0] P``_rd_addr; \
    reg [8:0]  P``_rd_beats_left; \
    reg [2:0]  P``_rd_size; \
    reg [1:0]  P``_rd_burst; \
    reg        P``_rd_active; \
    reg        P``_rd_accepted; \
    always @(posedge clk or negedge rst_n) begin : P``_read_accept_sample \
        if (!rst_n) begin \
            P``_rd_accepted <= 1'b0; \
        end else begin \
            P``_rd_accepted <= P``_rvalid && P``_rready; \
        end \
    end \
    initial begin : P``_read_slave \
        P``_rd_addr = 32'd0; \
        P``_rd_beats_left = 9'd0; \
        P``_rd_size = 3'd2; \
        P``_rd_burst = 2'd1; \
        P``_rd_active = 1'b0; \
        P``_arready = 1'b1; \
        P``_rdata = 32'd0; \
        P``_rresp = 2'b00; \
        P``_rlast = 1'b0; \
        P``_rvalid = 1'b0; \
        forever begin \
            @(negedge clk); \
            if (!rst_n) begin \
                P``_rd_addr = 32'd0; \
                P``_rd_beats_left = 9'd0; \
                P``_rd_size = 3'd2; \
                P``_rd_burst = 2'd1; \
                P``_rd_active = 1'b0; \
                P``_arready = 1'b1; \
                P``_rdata = 32'd0; \
                P``_rresp = 2'b00; \
                P``_rlast = 1'b0; \
                P``_rvalid = 1'b0; \
            end else begin \
                P``_arready = (!P``_rd_active && !(P``_rvalid && !P``_rd_accepted)); \
                if (P``_rvalid && !P``_rd_accepted) begin \
                    P``_rvalid = P``_rvalid; \
                end else begin \
                    if (P``_rd_active) begin \
                        P``_rdata = ddr_read32(P``_rd_addr); \
                        P``_rresp = 2'b00; \
                        P``_rlast = (P``_rd_beats_left == 9'd1); \
                        P``_rvalid = 1'b1; \
                        note_read(P``_rd_addr); \
                        if (P``_rd_beats_left == 9'd1) begin \
                            P``_rd_active = 1'b0; \
                            P``_rd_beats_left = 9'd0; \
                        end else begin \
                            P``_rd_beats_left = P``_rd_beats_left - 9'd1; \
                            if (P``_rd_burst == 2'd1) begin \
                                P``_rd_addr = P``_rd_addr + (32'd1 << P``_rd_size); \
                            end \
                        end \
                    end else if (P``_arvalid && P``_arready) begin \
                        P``_rvalid = 1'b0; \
                        P``_rlast = 1'b0; \
                        P``_rd_active = 1'b1; \
                        P``_rd_beats_left = {1'b0, P``_arlen} + 9'd1; \
                        P``_rd_size = P``_arsize; \
                        P``_rd_burst = P``_arburst; \
                        P``_rd_addr = P``_araddr; \
                    end else begin \
                        P``_rvalid = 1'b0; \
                        P``_rlast = 1'b0; \
                    end \
                end \
            end \
        end \
    end

    // Write buses: scatter, complex cache, RD-map collector, CFAR output.

module tb_e2e_3rx_xsim;

    // ------------------------------------------------------------------------
    // Timing and memory constants aligned with tb_radar_01_08.py
    // ------------------------------------------------------------------------
    localparam integer DEFAULT_CLK_HALF_NS = 4;
    // ADAR7251 byte clock is 9.6 MHz.  Use 52 ns half-period by default
    // because Verilog time is integer ns in this testbench, giving a close
    // 9.615 MHz.  Test 05 overrides this with a smaller +SCLK_ADC_HALF_NS to
    // run the same raw-ADC-to-CFAR path faster in simulation.
    localparam integer DEFAULT_SCLK_ADC_HALF_NS = 52;
    localparam integer RESET_CLK_CYCLES  = 32;
    localparam integer POST_RESET_CYCLES = 32;

    localparam [31:0] DDR_BASE = 32'h0A00_0000;
    localparam integer DDR_SIZE = 32'h0030_0000;       // Covers through D1.

    localparam [31:0] A_BASE = 32'h0A00_0000;
    localparam [31:0] B0_BASE = 32'h0A10_0000;
    localparam [31:0] C0_BASE = 32'h0A18_0000;
    localparam [31:0] D0_BASE = 32'h0A1A_0000;
    localparam [31:0] B1_BASE = 32'h0A20_0000;
    localparam [31:0] C1_BASE = 32'h0A28_0000;
    localparam [31:0] D1_BASE = 32'h0A2A_0000;

    localparam integer A_SIZE = 32'h0010_0000;
    localparam integer B_SIZE = 32'h0008_0000;
    localparam integer C_SIZE = 32'h0002_0000;
    localparam integer D_SIZE = 32'h0000_1000;
    localparam integer A_WORDS = A_SIZE / 4;
    localparam integer B_WORDS = B_SIZE / 4;
    localparam integer C_WORDS = C_SIZE / 4;
    localparam integer D_WORDS = D_SIZE / 4;

    localparam integer SAMPLE_SETS_PER_FRAME  = 128 * 256;
    localparam integer PPI_BYTES_PER_SAMPLE_SET = 8;
    localparam integer PPI_STREAM_BYTES = SAMPLE_SETS_PER_FRAME * PPI_BYTES_PER_SAMPLE_SET;

    // 120 ms at 8 ns system-clock period, matching the cocotb timeout.
    localparam integer TIMEOUT_CLK_CYCLES = 15_000_000;

    // ------------------------------------------------------------------------
    // Top-level stimulus/control signals
    // ------------------------------------------------------------------------
    reg        clk;
    reg        rst_n;
    reg        sclk_adc;
    reg        data_ready;
    reg [7:0]  dout;
    reg        frame_buf_sel;

    wire       conv_start;
    wire       ppi_valid;
    wire [2:0] rd_buf_valid;
    wire [2:0] doppler_frame_done;
    wire       rdmap_frame_complete;
    wire       cfar_start;
    wire       cfar_frame_buf_sel;
    wire       cfar_done;
    wire       cfar_busy;
    wire       cfar_axi_error;

    // ------------------------------------------------------------------------
    // DDR and PPI stimulus memories
    // ------------------------------------------------------------------------
    reg [31:0] ddr_A  [0:A_WORDS-1];
    reg [31:0] ddr_B0 [0:B_WORDS-1];
    reg [31:0] ddr_C0 [0:C_WORDS-1];
    reg [31:0] ddr_D0 [0:D_WORDS-1];
    reg [31:0] ddr_B1 [0:B_WORDS-1];
    reg [31:0] ddr_C1 [0:C_WORDS-1];
    reg [31:0] ddr_D1 [0:D_WORDS-1];
    reg [7:0] ppi_mem [0:PPI_STREAM_BYTES-1];

    string stim_hex_path;
    string ddr_dump_prefix;
    integer dump_ddr_enable;
    integer stim_limit_bytes;
    integer stim_drive_bytes;
    integer smoke_mode;
    integer smoke_drain_cycles;
    integer timeout_cycles;
    integer ppi_progress_bytes;
    integer status_interval_cycles;
    integer status_cycle_count;

    integer init_i;
    integer ppi_i;
    integer timeout_i;
    integer frame_buf_sel_int;
    integer clk_half_ns;
    integer sclk_adc_half_ns;

    reg [2:0] rd_buf_valid_d;
    reg [2:0] doppler_frame_done_d;
    reg       cfar_start_d;
    reg       cfar_done_d;
    reg       rdmap_frame_complete_d;

    // ------------------------------------------------------------------------
    // Region statistics, equivalent to AxiStats in the cocotb test.
    // ------------------------------------------------------------------------
    integer wr_A;
    integer wr_B0;
    integer wr_C0;
    integer wr_D0;
    integer wr_B1;
    integer wr_C1;
    integer wr_D1;
    integer wr_other;

    integer rd_A;
    integer rd_B0;
    integer rd_C0;
    integer rd_D0;
    integer rd_B1;
    integer rd_C1;
    integer rd_D1;
    integer rd_other;

    integer nonzero_A;
    integer nonzero_B0;
    integer nonzero_C0;
    integer nonzero_D0;
    integer nonzero_B1;
    integer nonzero_C1;
    integer nonzero_D1;
    integer nonzero_other;

    integer adar_valid_count;
    integer tdm_valid_count;
    integer bpf_valid_count;
    integer hilbert_valid_count;
    integer hilbert_accept_count;
    integer aligned_iq_count;
    integer range0_valid_count;
    integer range1_valid_count;
    integer range2_valid_count;
    integer rfft0_valid_count;
    integer rfft1_valid_count;
    integer rfft2_valid_count;
    integer rfft3_valid_count;
    integer dseq0_accept_count;
    integer dseq1_accept_count;
    integer dseq2_accept_count;
    integer dseq0_last_count;
    integer dseq1_last_count;
    integer dseq2_last_count;
    integer dseq0_fft_accept_count;
    integer dseq1_fft_accept_count;
    integer dseq2_fft_accept_count;
    integer dfft0_accept_count;
    integer dfft1_accept_count;
    integer dfft2_accept_count;
    integer dfft0_last_count;
    integer dfft1_last_count;
    integer dfft2_last_count;
    integer dfft0_fan_accept_count;
    integer dfft1_fan_accept_count;
    integer dfft2_fan_accept_count;
    integer noncoh_accept_count;



    // ========================================================================
    // DUT wiring: mirrors radar_test.v block design exactly.
    // No FIFOs. Direct connections throughout.
    // ========================================================================
localparam [15:0] RANGE_FFT_CONFIG   = 16'h2AAB;
    localparam [23:0] DOPPLER_FFT_CONFIG = 24'h00AAAB;
    localparam [15:0] CFAR_ALPHA_Q8      = 16'h0400;
    localparam integer RANGE_FFT_SAMPLES = 128; // = RADAR_RANGE_BINS (radar_params.vh)

    wire rst = ~rst_n;

    wire [15:0] adar_ch0;
    wire [15:0] adar_ch1;
    wire [15:0] adar_ch2;
    wire        adar_valid;

    wire [15:0] ppi_ch0_data;
    wire [15:0] ppi_ch1_data;
    wire [15:0] ppi_ch2_data;
    wire        ppi_ch0_vld;
    wire        ppi_ch1_vld;
    wire        ppi_ch2_vld;

    wire [15:0] tdm_data;
    wire [1:0]  tdm_ch_id;
    wire        tdm_valid;

    wire [15:0] bpf_tdata;
    wire [1:0]  bpf_tuser;
    wire        bpf_tvalid;
    wire [31:0] hilbert_tdata;
    wire        hilbert_s_tready;
    wire        hilbert_tvalid;
    wire        hilbert_accept;
    wire [15:0] hilbert_low16 = hilbert_tdata[15:0];
    assign hilbert_accept = bpf_tvalid & hilbert_s_tready;

    wire [1:0]  iq_i_ch_id;
    wire [15:0] iq_i_data;
    wire        iq_i_valid;
    wire [15:0] iq_q_data;
    wire        iq_q_valid;

    wire [31:0] range0_tdata;
    wire [31:0] range1_tdata;
    wire [31:0] range2_tdata;
    wire        range0_tvalid;
    wire        range1_tvalid;
    wire        range2_tvalid;
    wire        range0_tlast;
    wire        range1_tlast;
    wire        range2_tlast;
    wire [31:0] range0_fft_tdata;
    wire [31:0] range1_fft_tdata;
    wire [31:0] range2_fft_tdata;
    wire        range0_fft_tvalid;
    wire        range1_fft_tvalid;
    wire        range2_fft_tvalid;
    wire        range0_fft_tlast;
    wire        range1_fft_tlast;
    wire        range2_fft_tlast;
    // FIX 5: real FFT s_axis_data_tready, captured so input beats are never
    // dropped when the XFFT backpressures (sim runs far faster than the FFT
    // computes; on silicon tready stays high so this is harmless).
    wire        range0_fft_tready;
    wire        range1_fft_tready;
    wire        range2_fft_tready;
    wire [31:0] rfft0_tdata;
    wire [31:0] rfft1_tdata;
    wire [31:0] rfft2_tdata;
    wire [31:0] rfft3_tdata;
    wire        rfft0_tvalid;
    wire        rfft1_tvalid;
    wire        rfft2_tvalid;
    wire        rfft3_tvalid;
    wire        rfft0_tready;
    wire        rfft1_tready;
    wire        rfft2_tready;
    wire        rfft3_tready;

    wire [31:0] rd0_buf_base;
    wire [31:0] rd1_buf_base;
    wire [31:0] rd2_buf_base;

    wire [31:0] dseq0_tdata;
    wire [31:0] dseq1_tdata;
    wire [31:0] dseq2_tdata;
    wire        dseq0_tvalid;
    wire        dseq1_tvalid;
    wire        dseq2_tvalid;
    wire        dseq0_tlast;
    wire        dseq1_tlast;
    wire        dseq2_tlast;
    wire        dseq0_tready;
    wire        dseq1_tready;
    wire        dseq2_tready;
    wire [31:0] dseq0_fft_tdata;
    wire [31:0] dseq1_fft_tdata;
    wire [31:0] dseq2_fft_tdata;
    wire        dseq0_fft_tvalid;
    wire        dseq1_fft_tvalid;
    wire        dseq2_fft_tvalid;
    wire        dseq0_fft_tlast;
    wire        dseq1_fft_tlast;
    wire        dseq2_fft_tlast;
    wire        dseq0_fft_tready;
    wire        dseq1_fft_tready;
    wire        dseq2_fft_tready;

    wire [31:0] dfft0_tdata;
    wire [31:0] dfft1_tdata;
    wire [31:0] dfft2_tdata;
    wire        dfft0_tvalid;
    wire        dfft1_tvalid;
    wire        dfft2_tvalid;
    wire        dfft0_tlast;
    wire        dfft1_tlast;
    wire        dfft2_tlast;
    wire        cache0_tready;
    wire        cache1_tready;
    wire        cache2_tready;
    wire        cache3_tready;
    wire        noncoh0_tready;
    wire        noncoh1_tready;
    wire        noncoh2_tready;
    wire        noncoh3_tready;
    wire        dfft0_tready;
    wire        dfft1_tready;
    wire        dfft2_tready;
    wire        dfft0_event_tlast_unexpected;
    wire        dfft1_event_tlast_unexpected;
    wire        dfft2_event_tlast_unexpected;
    wire        dfft0_event_tlast_missing;
    wire        dfft1_event_tlast_missing;
    wire        dfft2_event_tlast_missing;
    wire        dfft0_event_data_in_halt;
    wire        dfft1_event_data_in_halt;
    wire        dfft2_event_data_in_halt;
    wire        dfft0_event_data_out_halt;
    wire        dfft1_event_data_out_halt;
    wire        dfft2_event_data_out_halt;

    wire [31:0] dfft0_fan_tdata;
    wire [31:0] dfft1_fan_tdata;
    wire [31:0] dfft2_fan_tdata;
    wire        dfft0_fan_tvalid;
    wire        dfft1_fan_tvalid;
    wire        dfft2_fan_tvalid;
    wire        dfft0_fan_tlast;
    wire        dfft1_fan_tlast;
    wire        dfft2_fan_tlast;
    // Fan-out tready: each FIFO pops only when BOTH consumers for that channel
    // are simultaneously ready — the cplx_cell_writer (cacheN_tready) AND the
    // noncoh_integrator (noncohN_tready = fire = all_valid & !stall).
    //
    // Bug fix: cplx_cell_writer counts a beat on (s_axis_tvalid & s_axis_tready)
    // but the FIFO pops on (dfftN_fan_tvalid & dfftN_fan_tready).  These are the
    // same only when noncohN_tready=1.  When noncoh is not ready the FIFO stalls
    // but the writer's beat_cnt still advances, causing WLAST one beat early.
    // Fix: gate s_tvalid presented to u_cache with noncohN_tready so the writer
    // only sees a valid beat on cycles where the FIFO actually pops.
    // Each channel is independent — no cross-channel coupling needed here.
    wire        dfft0_fan_tready = cache0_tready & noncoh0_tready;
    wire        dfft1_fan_tready = cache1_tready & noncoh1_tready;
    wire        dfft2_fan_tready = cache2_tready & noncoh2_tready;

    wire [31:0] noncoh_tdata;
    wire        noncoh_tvalid;
    wire        noncoh_tlast;
    wire        noncoh_tready;

    wire        cache_ready;

    `AXI_WR_DECL(sc0);
    `AXI_WR_DECL(sc1);
    `AXI_WR_DECL(sc2);
    `AXI_RD_DECL(ds0);
    `AXI_RD_DECL(ds1);
    `AXI_RD_DECL(ds2);
    `AXI_WR_DECL(cc0);
    `AXI_WR_DECL(cc1);
    `AXI_WR_DECL(cc2);
    `AXI_WR_DECL(rd);
    `AXI_RD_DECL(cfar);
    `AXI_WR_DECL(cfar);

    wire [2:0] scatter_busy;
    wire [2:0] scatter_stall;
    wire [2:0] dseq_busy;
    wire       cache_busy;
    wire       rdmap_busy;

    assign ppi_valid = adar_valid;

    reg [$clog2(RANGE_FFT_SAMPLES)-1:0] range0_sample_idx;
    reg [$clog2(RANGE_FFT_SAMPLES)-1:0] range1_sample_idx;
    reg [$clog2(RANGE_FFT_SAMPLES)-1:0] range2_sample_idx;

    assign range0_tlast = range0_tvalid && (range0_sample_idx == RANGE_FFT_SAMPLES - 1);
    assign range1_tlast = range1_tvalid && (range1_sample_idx == RANGE_FFT_SAMPLES - 1);
    assign range2_tlast = range2_tvalid && (range2_sample_idx == RANGE_FFT_SAMPLES - 1);


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            range0_sample_idx <= 8'd0;
            range1_sample_idx <= 8'd0;
            range2_sample_idx <= 8'd0;
        end else begin
            if (range0_tvalid) range0_sample_idx <= range0_tlast ? 8'd0 : range0_sample_idx + 8'd1;
            if (range1_tvalid) range1_sample_idx <= range1_tlast ? 8'd0 : range1_sample_idx + 8'd1;
            if (range2_tvalid) range2_sample_idx <= range2_tlast ? 8'd0 : range2_sample_idx + 8'd1;
        end
    end

    adar7251_ppi_rx u_adar (
        .sclk_adc(sclk_adc),
        .data_ready(data_ready),
        .dout(dout),
        .conv_start(conv_start),
        .ch0_out(adar_ch0),
        .ch1_out(adar_ch1),
        .ch2_out(adar_ch2),
        .valid_out(adar_valid),
        .clk_sys(clk),
        .rst(rst)
    );

    ppi_to_tdm_bridge u_ppi_bridge (
        .ch0_data(ppi_ch0_data),
        .ch0_in_data(adar_ch0),
        .ch0_vld(ppi_ch0_vld),
        .ch1_data(ppi_ch1_data),
        .ch1_in_data(adar_ch1),
        .ch1_vld(ppi_ch1_vld),
        .ch2_data(ppi_ch2_data),
        .ch2_in_data(adar_ch2),
        .ch2_vld(ppi_ch2_vld),
        .clk_tdm(clk),
        .in_valid(adar_valid),
        .rst_n_adc(rst_n),
        .rst_n_tdm(rst_n),
        .sclk_adc(sclk_adc)
    );

    tdm_mux_3to1 u_tdm_mux (
        .ch0_data(ppi_ch0_data),
        .ch0_vld(ppi_ch0_vld),
        .ch1_data(ppi_ch1_data),
        .ch1_vld(ppi_ch1_vld),
        .ch2_data(ppi_ch2_data),
        .ch2_vld(ppi_ch2_vld),
        .clk(clk),
        .rst_n(rst_n),
        .tdm_ch_id(tdm_ch_id),
        .tdm_data(tdm_data),
        .tdm_valid(tdm_valid)
    );

`ifdef TURBO_FAST_FRONTEND
    assign bpf_tdata = tdm_data;
    assign bpf_tuser = tdm_ch_id;
    assign bpf_tvalid = tdm_valid;
    assign hilbert_s_tready = 1'b1;
    assign hilbert_tdata = 32'd0;
    assign hilbert_tvalid = tdm_valid;
    assign iq_i_ch_id = tdm_ch_id;
    assign iq_i_data = tdm_data;
    assign iq_i_valid = tdm_valid;
    assign iq_q_data = 16'd0;
    assign iq_q_valid = tdm_valid;
`else
    radar_test_fir_compiler_0_0 u_fir_bpf (
        .aclk(clk),
        .s_axis_data_tvalid(tdm_valid),
        .s_axis_data_tready(),
        .s_axis_data_tuser(tdm_ch_id),
        .s_axis_data_tdata(tdm_data),
        .m_axis_data_tvalid(bpf_tvalid),
        .m_axis_data_tuser(bpf_tuser),
        .m_axis_data_tdata(bpf_tdata),
        .event_s_data_chanid_incorrect()
    );

    radar_test_fir_compiler_0_1 u_fir_hilbert (
        .aclk(clk),
        .s_axis_data_tvalid(bpf_tvalid),
        .s_axis_data_tready(hilbert_s_tready),
        .s_axis_data_tuser(bpf_tuser),
        .s_axis_data_tdata(bpf_tdata),
        .m_axis_data_tvalid(hilbert_tvalid),
        .m_axis_data_tdata(hilbert_tdata),
        .event_s_data_chanid_incorrect()
    );

    iq_delay_align #(
        .DATA_W(16),
        .HILBERT_LATENCY(36)
    ) u_iq_align (
        .bpf_tdm_ch_id(bpf_tuser),
        .bpf_tdm_data(bpf_tdata),
        .bpf_tdm_valid(hilbert_accept),
        .clk(clk),
        .hilbert_data(hilbert_low16),
        .hilbert_valid(hilbert_tvalid),
        .i_tdm_ch_id(iq_i_ch_id),
        .i_tdm_data(iq_i_data),
        .i_tdm_valid(iq_i_valid),
        .q_tdm_data(iq_q_data),
        .q_tdm_valid(iq_q_valid),
        .rst_n(rst_n)
    );
`endif

    iq_demux_1to3 u_iq_demux (
        .ch0_tdata(range0_tdata),
        .ch0_tvalid(range0_tvalid),
        .ch1_tdata(range1_tdata),
        .ch1_tvalid(range1_tvalid),
        .ch2_tdata(range2_tdata),
        .ch2_tvalid(range2_tvalid),
        .clk(clk),
        .i_tdm_ch_id(iq_i_ch_id),
        .i_tdm_data(iq_i_data),
        .i_tdm_valid(iq_i_valid),
        .q_tdm_data(iq_q_data),
        .q_tdm_valid(iq_q_valid),
        .rst_n(rst_n)
    );

    // Range FFT INPUT FIFOs (simulation only).
    // FIX 5: The XFFT IP backpressures via s_axis_data_tready while it computes
    // a transform.  The previous "free-run, ignore tready" wiring silently
    // dropped input beats whenever the turbo-clocked front end outran the FFT,
    // desynchronising tlast framing and channel-halting the FFT (which stalled
    // the whole chain at ~one frame's worth of samples).  These FIFOs capture
    // the real tready and carry {tlast,tdata} so no beat is ever lost.  On
    // silicon the IQ rate is slow relative to the FFT, tready stays high, the
    // FIFO never fills, and the behaviour is identical to a direct connection.
    wire        range0_fin_tvalid; wire        range0_fin_tready;
    wire [32:0] range0_fin_tdata;
    wire        range1_fin_tvalid; wire        range1_fin_tready;
    wire [32:0] range1_fin_tdata;
    wire        range2_fin_tvalid; wire        range2_fin_tready;
    wire [32:0] range2_fin_tdata;

    axis_fifo_simple #(.DATA_W(33), .DEPTH(4096)) u_range0_in_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({range0_tlast, range0_tdata}), .s_tvalid(range0_tvalid), .s_tready(),
        .m_tdata(range0_fin_tdata), .m_tvalid(range0_fin_tvalid), .m_tready(range0_fin_tready)
    );
    axis_fifo_simple #(.DATA_W(33), .DEPTH(4096)) u_range1_in_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({range1_tlast, range1_tdata}), .s_tvalid(range1_tvalid), .s_tready(),
        .m_tdata(range1_fin_tdata), .m_tvalid(range1_fin_tvalid), .m_tready(range1_fin_tready)
    );
    axis_fifo_simple #(.DATA_W(33), .DEPTH(4096)) u_range2_in_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({range2_tlast, range2_tdata}), .s_tvalid(range2_tvalid), .s_tready(),
        .m_tdata(range2_fin_tdata), .m_tvalid(range2_fin_tvalid), .m_tready(range2_fin_tready)
    );

    assign range0_fft_tdata  = range0_fin_tdata[31:0];
    assign range0_fft_tlast  = range0_fin_tdata[32];
    assign range0_fft_tvalid = range0_fin_tvalid;
    assign range0_fin_tready = range0_fft_tready;
    assign range1_fft_tdata  = range1_fin_tdata[31:0];
    assign range1_fft_tlast  = range1_fin_tdata[32];
    assign range1_fft_tvalid = range1_fin_tvalid;
    assign range1_fin_tready = range1_fft_tready;
    assign range2_fft_tdata  = range2_fin_tdata[31:0];
    assign range2_fft_tlast  = range2_fin_tdata[32];
    assign range2_fft_tvalid = range2_fin_tvalid;
    assign range2_fin_tready = range2_fft_tready;

    // Range FFT OUTPUT FIFOs: absorb burst FFT output while scatter issues
    // single-beat AXI writes (one DDR round-trip per sample).  In real hardware
    // DDR latency is comparable to FFT output rate so no FIFO is needed; in
    // simulation the FFT produces output much faster than the AXI slave responds,
    // so the FFT output FIFO halts without a buffer here.

    radar_test_xfft_0_1 u_range_fft0 (
        .aclk(clk),
        .s_axis_config_tdata(RANGE_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(range0_fft_tdata),
        .s_axis_data_tvalid(range0_fft_tvalid),
        .s_axis_data_tready(range0_fft_tready),
        .s_axis_data_tlast(range0_fft_tlast),
        .m_axis_data_tdata(rfft0_tdata),
        .m_axis_data_tvalid(rfft0_tvalid),
        .m_axis_data_tready(rfft0_tready),
        .m_axis_data_tlast(),
        .event_frame_started(),
        .event_tlast_unexpected(),
        .event_tlast_missing(),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(),
        .event_data_out_channel_halt()
    );

    radar_test_xfft_1_0 u_range_fft1 (
        .aclk(clk),
        .s_axis_config_tdata(RANGE_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(range1_fft_tdata),
        .s_axis_data_tvalid(range1_fft_tvalid),
        .s_axis_data_tready(range1_fft_tready),
        .s_axis_data_tlast(range1_fft_tlast),
        .m_axis_data_tdata(rfft1_tdata),
        .m_axis_data_tvalid(rfft1_tvalid),
        .m_axis_data_tready(rfft1_tready),
        .m_axis_data_tlast(),
        .event_frame_started(),
        .event_tlast_unexpected(),
        .event_tlast_missing(),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(),
        .event_data_out_channel_halt()
    );

    radar_test_xfft_2_1 u_range_fft2 (
        .aclk(clk),
        .s_axis_config_tdata(RANGE_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(range2_fft_tdata),
        .s_axis_data_tvalid(range2_fft_tvalid),
        .s_axis_data_tready(range2_fft_tready),
        .s_axis_data_tlast(range2_fft_tlast),
        .m_axis_data_tdata(rfft2_tdata),
        .m_axis_data_tvalid(rfft2_tvalid),
        .m_axis_data_tready(rfft2_tready),
        .m_axis_data_tlast(),
        .event_frame_started(),
        .event_tlast_unexpected(),
        .event_tlast_missing(),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(),
        .event_data_out_channel_halt()
    );


    // FIX 6: Range FFT output goes DIRECTLY to scatter, matching the known-good
    // DUT (01_tb_full_pipeline_top_old.v).  An intermediate FFT-output FIFO was
    // added in this flat TB and is the regression: it dropped the FFT's tlast
    // framing (DATA_W=32) and changed the FFT->scatter cadence so the chain
    // wedged with dseq=0.  Scatter natively backpressures the FFT via rfftN_tready
    // (single-beat-per-AXI-write); the Xilinx XFFT stalls cleanly on that, which
    // is correct in both simulation and on silicon.  No output FIFO is needed.

    scatter_write_3rx u_scatter (
        .clk(clk),
        .rst_n(rst_n),
        .s0_tdata(rfft0_tdata),
        .s0_tvalid(rfft0_tvalid),
        .s0_tready(rfft0_tready),
        .s1_tdata(rfft1_tdata),
        .s1_tvalid(rfft1_tvalid),
        .s1_tready(rfft1_tready),
        .s2_tdata(rfft2_tdata),
        .s2_tvalid(rfft2_tvalid),
        .s2_tready(rfft2_tready),
        `AXI_WR_PORTS(m0, sc0),
        `AXI_WR_PORTS(m1, sc1),
        `AXI_WR_PORTS(m2, sc2),
        .rd0_buf_base(rd0_buf_base),
        .rd1_buf_base(rd1_buf_base),
        .rd2_buf_base(rd2_buf_base),
        .rd_buf_valid(rd_buf_valid),
        .doppler_frame_done(doppler_frame_done),
        .busy(scatter_busy),
        .stall(scatter_stall)
    );

    doppler_seq_3rx u_dseq (
        .clk(clk),
        .rst_n(rst_n),
        .rd0_buf_base(rd0_buf_base),
        .rd1_buf_base(rd1_buf_base),
        .rd2_buf_base(rd2_buf_base),
        .rd_buf_valid(rd_buf_valid),
        .doppler_frame_done(doppler_frame_done),
        `AXI_RD_PORTS(m0, ds0),
        `AXI_RD_PORTS(m1, ds1),
        `AXI_RD_PORTS(m2, ds2),
        .m0_axis_tdata(dseq0_tdata),
        .m0_axis_tvalid(dseq0_tvalid),
        .m0_axis_tlast(dseq0_tlast),
        .m0_axis_tready(dseq0_tready),
        .m1_axis_tdata(dseq1_tdata),
        .m1_axis_tvalid(dseq1_tvalid),
        .m1_axis_tlast(dseq1_tlast),
        .m1_axis_tready(dseq1_tready),
        .m2_axis_tdata(dseq2_tdata),
        .m2_axis_tvalid(dseq2_tvalid),
        .m2_axis_tlast(dseq2_tlast),
        .m2_axis_tready(dseq2_tready),
        .fft0_out_valid(dfft0_tvalid),
        .fft0_out_last(dfft0_tlast),
        .fft1_out_valid(dfft1_tvalid),
        .fft1_out_last(dfft1_tlast),
        .fft2_out_valid(dfft2_tvalid),
        .fft2_out_last(dfft2_tlast),
        .busy(dseq_busy)
    );

    // Doppler sequencer → Doppler FFT FIFOs (simulation only).
    // Same rationale as range FFT FIFOs: the AXI read slave delivers data in
    // bursts faster than the Doppler FFT can consume in turbo simulation mode.
    axis_fifo_simple #(.DATA_W(33), .DEPTH(512)) u_dseq0_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({dseq0_tlast, dseq0_tdata}), .s_tvalid(dseq0_tvalid), .s_tready(dseq0_tready),
        .m_tdata({dseq0_fft_tlast, dseq0_fft_tdata}), .m_tvalid(dseq0_fft_tvalid), .m_tready(dseq0_fft_tready)
    );
    axis_fifo_simple #(.DATA_W(33), .DEPTH(512)) u_dseq1_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({dseq1_tlast, dseq1_tdata}), .s_tvalid(dseq1_tvalid), .s_tready(dseq1_tready),
        .m_tdata({dseq1_fft_tlast, dseq1_fft_tdata}), .m_tvalid(dseq1_fft_tvalid), .m_tready(dseq1_fft_tready)
    );
    axis_fifo_simple #(.DATA_W(33), .DEPTH(512)) u_dseq2_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_tdata({dseq2_tlast, dseq2_tdata}), .s_tvalid(dseq2_tvalid), .s_tready(dseq2_tready),
        .m_tdata({dseq2_fft_tlast, dseq2_fft_tdata}), .m_tvalid(dseq2_fft_tvalid), .m_tready(dseq2_fft_tready)
    );

    radar_test_xfft_1_1 u_doppler_fft0 (
        .aclk(clk),
        .s_axis_config_tdata(DOPPLER_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(dseq0_fft_tdata),
        .s_axis_data_tvalid(dseq0_fft_tvalid),
        .s_axis_data_tready(dseq0_fft_tready),
        .s_axis_data_tlast(dseq0_fft_tlast),
        .m_axis_data_tdata(dfft0_tdata),
        .m_axis_data_tvalid(dfft0_tvalid),
        .m_axis_data_tready(dfft0_tready),
        .m_axis_data_tlast(dfft0_tlast),
        .event_frame_started(),
        .event_tlast_unexpected(dfft0_event_tlast_unexpected),
        .event_tlast_missing(dfft0_event_tlast_missing),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(dfft0_event_data_in_halt),
        .event_data_out_channel_halt(dfft0_event_data_out_halt)
    );

    radar_test_xfft_5_0 u_doppler_fft1 (
        .aclk(clk),
        .s_axis_config_tdata(DOPPLER_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(dseq1_fft_tdata),
        .s_axis_data_tvalid(dseq1_fft_tvalid),
        .s_axis_data_tready(dseq1_fft_tready),
        .s_axis_data_tlast(dseq1_fft_tlast),
        .m_axis_data_tdata(dfft1_tdata),
        .m_axis_data_tvalid(dfft1_tvalid),
        .m_axis_data_tready(dfft1_tready),
        .m_axis_data_tlast(dfft1_tlast),
        .event_frame_started(),
        .event_tlast_unexpected(dfft1_event_tlast_unexpected),
        .event_tlast_missing(dfft1_event_tlast_missing),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(dfft1_event_data_in_halt),
        .event_data_out_channel_halt(dfft1_event_data_out_halt)
    );

    radar_test_xfft_6_0 u_doppler_fft2 (
        .aclk(clk),
        .s_axis_config_tdata(DOPPLER_FFT_CONFIG),
        .s_axis_config_tvalid(1'b1),
        .s_axis_config_tready(),
        .s_axis_data_tdata(dseq2_fft_tdata),
        .s_axis_data_tvalid(dseq2_fft_tvalid),
        .s_axis_data_tready(dseq2_fft_tready),
        .s_axis_data_tlast(dseq2_fft_tlast),
        .m_axis_data_tdata(dfft2_tdata),
        .m_axis_data_tvalid(dfft2_tvalid),
        .m_axis_data_tready(dfft2_tready),
        .m_axis_data_tlast(dfft2_tlast),
        .event_frame_started(),
        .event_tlast_unexpected(dfft2_event_tlast_unexpected),
        .event_tlast_missing(dfft2_event_tlast_missing),
        .event_status_channel_halt(),
        .event_data_in_channel_halt(dfft2_event_data_in_halt),
        .event_data_out_channel_halt(dfft2_event_data_out_halt)
    );


    // Direct connection matching live block design (radar_test.v util_vector_logic_4..7):
    // Doppler FFT outputs connect straight to the AND-gate tready fan-out.
    // No FIFOs exist in the live system between dfft and cache/noncoh.
    assign dfft0_fan_tdata  = dfft0_tdata;
    assign dfft0_fan_tlast  = dfft0_tlast;
    assign dfft0_fan_tvalid = dfft0_tvalid;
    assign dfft0_tready     = dfft0_fan_tready;

    assign dfft1_fan_tdata  = dfft1_tdata;
    assign dfft1_fan_tlast  = dfft1_tlast;
    assign dfft1_fan_tvalid = dfft1_tvalid;
    assign dfft1_tready     = dfft1_fan_tready;

    assign dfft2_fan_tdata  = dfft2_tdata;
    assign dfft2_fan_tlast  = dfft2_tlast;
    assign dfft2_fan_tvalid = dfft2_tvalid;
    assign dfft2_tready     = dfft2_fan_tready;


    cplx_cell_cache_top u_cache (
        .clk(clk),
        .rst_n(rst_n),
        .frame_buf_sel(frame_buf_sel),
        .s0_tdata(dfft0_fan_tdata),
        // Gate tvalid with noncohN_tready: writer beat_cnt must only advance
        // on cycles where the shared FIFO actually pops (both consumers ready).
        .s0_tvalid(dfft0_fan_tvalid & noncoh0_tready),
        .s0_tlast(dfft0_fan_tlast),
        .s0_tready(cache0_tready),
        .s1_tdata(dfft1_fan_tdata),
        .s1_tvalid(dfft1_fan_tvalid & noncoh1_tready),
        .s1_tlast(dfft1_fan_tlast),
        .s1_tready(cache1_tready),
        .s2_tdata(dfft2_fan_tdata),
        .s2_tvalid(dfft2_fan_tvalid & noncoh2_tready),
        .s2_tlast(dfft2_fan_tlast),
        .s2_tready(cache2_tready),
        `AXI_WR_PORTS(m0, cc0),
        `AXI_WR_PORTS(m1, cc1),
        `AXI_WR_PORTS(m2, cc2),
        .busy(cache_busy),
        .cache_ready(cache_ready)
    );

    noncoh_integrator u_noncoh (
        .clk(clk),
        .rst_n(rst_n),
        .s0_tdata(dfft0_fan_tdata),
        .s0_tvalid(dfft0_fan_tvalid),
        .s0_tlast(dfft0_fan_tlast),
        .s0_tready(noncoh0_tready),
        .s1_tdata(dfft1_fan_tdata),
        .s1_tvalid(dfft1_fan_tvalid),
        .s1_tlast(dfft1_fan_tlast),
        .s1_tready(noncoh1_tready),
        .s2_tdata(dfft2_fan_tdata),
        .s2_tvalid(dfft2_fan_tvalid),
        .s2_tlast(dfft2_fan_tlast),
        .s2_tready(noncoh2_tready),
        .m_axis_tdata(noncoh_tdata),
        .m_axis_tvalid(noncoh_tvalid),
        .m_axis_tlast(noncoh_tlast),
        .m_axis_tready(noncoh_tready)
    );

    rd_map_collector_summed u_rdmap (
        .clk(clk),
        .rst_n(rst_n),
        .frame_buf_sel(frame_buf_sel),
        .s_axis_tdata(noncoh_tdata),
        .s_axis_tvalid(noncoh_tvalid),
        .s_axis_tlast(noncoh_tlast),
        .s_axis_tready(noncoh_tready),
        `AXI_WR_PORTS(m_axi, rd),
        .cfar_start(cfar_start),
        .cfar_frame_buf_sel(cfar_frame_buf_sel),
        .cfar_done(cfar_done),
        .busy(rdmap_busy),
        .frame_complete(rdmap_frame_complete)
    );

    cfar_streaming_axi_verilog u_cfar (
        .clk(clk),
        .rst_n(rst_n),
        .frame_buf_sel(cfar_frame_buf_sel),
        .cfar_start(cfar_start),
        .cfar_done(cfar_done),
        .alpha_q8(CFAR_ALPHA_Q8),
        .busy(cfar_busy),
        .axi_error(cfar_axi_error),
        .m_axi_araddr(cfar_araddr),
        .m_axi_arlen(cfar_arlen),
        .m_axi_arsize(cfar_arsize),
        .m_axi_arburst(cfar_arburst),
        .m_axi_arcache(),
        .m_axi_arprot(),
        .m_axi_arqos(),
        .m_axi_arvalid(cfar_arvalid),
        .m_axi_arready(cfar_arready),
        .m_axi_rdata(cfar_rdata),
        .m_axi_rresp(cfar_rresp),
        .m_axi_rlast(cfar_rlast),
        .m_axi_rvalid(cfar_rvalid),
        .m_axi_rready(cfar_rready),
        .m_axi_awaddr(cfar_awaddr),
        .m_axi_awlen(cfar_awlen),
        .m_axi_awsize(cfar_awsize),
        .m_axi_awburst(cfar_awburst),
        .m_axi_awcache(),
        .m_axi_awprot(),
        .m_axi_awqos(),
        .m_axi_awvalid(cfar_awvalid),
        .m_axi_awready(cfar_awready),
        .m_axi_wdata(cfar_wdata),
        .m_axi_wstrb(cfar_wstrb),
        .m_axi_wlast(cfar_wlast),
        .m_axi_wvalid(cfar_wvalid),
        .m_axi_wready(cfar_wready),
        .m_axi_bresp(cfar_bresp),
        .m_axi_bvalid(cfar_bvalid),
        .m_axi_bready(cfar_bready)
    );


    // ------------------------------------------------------------------------
    // Clock generation
    // ------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        if (!$value$plusargs("CLK_HALF_NS=%d", clk_half_ns)) begin
            clk_half_ns = DEFAULT_CLK_HALF_NS;
        end
        if (clk_half_ns <= 0) begin
            $display("ERROR: CLK_HALF_NS must be > 0, got %0d", clk_half_ns);
            $fatal(1);
        end
        $display("INFO: CLK_HALF_NS=%0d", clk_half_ns);
        forever #(clk_half_ns) clk = ~clk;
    end

    initial begin
        sclk_adc = 1'b0;
        if (!$value$plusargs("SCLK_ADC_HALF_NS=%d", sclk_adc_half_ns)) begin
            sclk_adc_half_ns = DEFAULT_SCLK_ADC_HALF_NS;
        end
        if (sclk_adc_half_ns <= 0) begin
            $display("ERROR: SCLK_ADC_HALF_NS must be > 0, got %0d", sclk_adc_half_ns);
            $fatal(1);
        end
        $display("INFO: SCLK_ADC_HALF_NS=%0d", sclk_adc_half_ns);
        forever #(sclk_adc_half_ns) sclk_adc = ~sclk_adc;
    end

    // ------------------------------------------------------------------------
    // Address helpers
    // ------------------------------------------------------------------------
    function integer ddr_offset;
        input [31:0] addr;
        begin
            if (addr < DDR_BASE || addr > (DDR_BASE + DDR_SIZE - 4)) begin
                $display("ERROR: DDR address out of range: 0x%08x at time %0t", addr, $time);
                $fatal(1);
            end
            ddr_offset = addr - DDR_BASE;
        end
    endfunction

    function integer region_id;
        input [31:0] addr;
        begin
            if (addr >= A_BASE && addr < A_BASE + A_SIZE) begin
                region_id = 0;
            end else if (addr >= B0_BASE && addr < B0_BASE + B_SIZE) begin
                region_id = 1;
            end else if (addr >= C0_BASE && addr < C0_BASE + C_SIZE) begin
                region_id = 2;
            end else if (addr >= D0_BASE && addr < D0_BASE + D_SIZE) begin
                region_id = 3;
            end else if (addr >= B1_BASE && addr < B1_BASE + B_SIZE) begin
                region_id = 4;
            end else if (addr >= C1_BASE && addr < C1_BASE + C_SIZE) begin
                region_id = 5;
            end else if (addr >= D1_BASE && addr < D1_BASE + D_SIZE) begin
                region_id = 6;
            end else begin
                region_id = 7;
            end
        end
    endfunction

    function integer region_word_index;
        input [31:0] addr;
        input [31:0] base_addr;
        begin
            if (addr[1:0] != 2'b00) begin
                $display("ERROR: unaligned DDR word address: 0x%08x at time %0t", addr, $time);
                $fatal(1);
            end
            region_word_index = (addr - base_addr) >> 2;
        end
    endfunction

    function [31:0] ddr_read32;
        input [31:0] addr;
        integer r;
        begin
            r = region_id(addr);
            case (r)
                0: ddr_read32 = ddr_A[region_word_index(addr, A_BASE)];
                1: ddr_read32 = ddr_B0[region_word_index(addr, B0_BASE)];
                2: ddr_read32 = ddr_C0[region_word_index(addr, C0_BASE)];
                3: ddr_read32 = ddr_D0[region_word_index(addr, D0_BASE)];
                4: ddr_read32 = ddr_B1[region_word_index(addr, B1_BASE)];
                5: ddr_read32 = ddr_C1[region_word_index(addr, C1_BASE)];
                6: ddr_read32 = ddr_D1[region_word_index(addr, D1_BASE)];
                default: begin
                    ddr_offset(addr);
                    ddr_read32 = 32'd0;
                end
            endcase
        end
    endfunction

    task ddr_write32;
        input [31:0] addr;
        input [31:0] data;
        input [3:0]  strb;
        integer r;
        integer word_idx;
        reg [31:0] word_value;
        begin
            r = region_id(addr);
            word_value = ddr_read32(addr);
            if (strb[0]) word_value[7:0] = data[7:0];
            if (strb[1]) word_value[15:8] = data[15:8];
            if (strb[2]) word_value[23:16] = data[23:16];
            if (strb[3]) word_value[31:24] = data[31:24];
            case (r)
                0: begin word_idx = region_word_index(addr, A_BASE);  ddr_A[word_idx]  = word_value; end
                1: begin word_idx = region_word_index(addr, B0_BASE); ddr_B0[word_idx] = word_value; end
                2: begin word_idx = region_word_index(addr, C0_BASE); ddr_C0[word_idx] = word_value; end
                3: begin word_idx = region_word_index(addr, D0_BASE); ddr_D0[word_idx] = word_value; end
                4: begin word_idx = region_word_index(addr, B1_BASE); ddr_B1[word_idx] = word_value; end
                5: begin word_idx = region_word_index(addr, C1_BASE); ddr_C1[word_idx] = word_value; end
                6: begin word_idx = region_word_index(addr, D1_BASE); ddr_D1[word_idx] = word_value; end
                default: begin
                    ddr_offset(addr);
                end
            endcase
        end
    endtask

    task note_write;
        input [31:0] addr;
        input [31:0] data;
        integer r;
        begin
            r = region_id(addr);
            case (r)
                0: begin wr_A = wr_A + 1; if (data != 32'd0) nonzero_A = nonzero_A + 1; end
                1: begin wr_B0 = wr_B0 + 1; if (data != 32'd0) nonzero_B0 = nonzero_B0 + 1; end
                2: begin wr_C0 = wr_C0 + 1; if (data != 32'd0) nonzero_C0 = nonzero_C0 + 1; end
                3: begin wr_D0 = wr_D0 + 1; if (data != 32'd0) nonzero_D0 = nonzero_D0 + 1; end
                4: begin wr_B1 = wr_B1 + 1; if (data != 32'd0) nonzero_B1 = nonzero_B1 + 1; end
                5: begin wr_C1 = wr_C1 + 1; if (data != 32'd0) nonzero_C1 = nonzero_C1 + 1; end
                6: begin wr_D1 = wr_D1 + 1; if (data != 32'd0) nonzero_D1 = nonzero_D1 + 1; end
                default: begin wr_other = wr_other + 1; if (data != 32'd0) nonzero_other = nonzero_other + 1; end
            endcase
        end
    endtask

    task note_read;
        input [31:0] addr;
        integer r;
        begin
            r = region_id(addr);
            case (r)
                0: rd_A = rd_A + 1;
                1: rd_B0 = rd_B0 + 1;
                2: rd_C0 = rd_C0 + 1;
                3: rd_D0 = rd_D0 + 1;
                4: rd_B1 = rd_B1 + 1;
                5: rd_C1 = rd_C1 + 1;
                6: rd_D1 = rd_D1 + 1;
                default: rd_other = rd_other + 1;
            endcase
        end
    endtask

    // ------------------------------------------------------------------------
    // DDR dump helpers.
    //
    // Each dump file contains one 32-bit hexadecimal word per line.  The word
    // is reconstructed with ddr_read32(), so byte lane 0 is the least
    // significant byte, exactly like the AXI little-endian read model.
    // ------------------------------------------------------------------------
    task dump_region_words_hex;
        input string path;
        input [31:0] base_addr;
        input integer size_bytes;
        integer fh;
        integer byte_off;
        integer word_count;
        integer nz_count;
        reg [31:0] word_value;
        begin
            fh = $fopen(path, "w");
            if (fh == 0) begin
                $display("ERROR: could not open DDR dump file: %0s", path);
                $fatal(1);
            end

            word_count = 0;
            nz_count = 0;
            for (byte_off = 0; byte_off < size_bytes; byte_off = byte_off + 4) begin
                word_value = ddr_read32(base_addr + byte_off[31:0]);
                $fdisplay(fh, "%08x", word_value);
                word_count = word_count + 1;
                if (word_value != 32'd0) begin
                    nz_count = nz_count + 1;
                end
            end

            $fclose(fh);
            $display("INFO: dumped %0d words to %0s, nonzero_words=%0d", word_count, path, nz_count);
        end
    endtask

    task dump_summary_txt;
        input string path;
        integer fh;
        begin
            fh = $fopen(path, "w");
            if (fh == 0) begin
                $display("ERROR: could not open DDR summary file: %0s", path);
                $fatal(1);
            end

            $fdisplay(fh, "radar_bcd_pingpong_xsim_ddr_summary");
            $fdisplay(fh, "A_BASE=0x%08x A_SIZE_BYTES=%0d A_WRITES=%0d A_READS=%0d A_NONZERO_WRITES=%0d", A_BASE, A_SIZE, wr_A, rd_A, nonzero_A);
            $fdisplay(fh, "B0_BASE=0x%08x B_SIZE_BYTES=%0d B0_WRITES=%0d B0_READS=%0d B0_NONZERO_WRITES=%0d", B0_BASE, B_SIZE, wr_B0, rd_B0, nonzero_B0);
            $fdisplay(fh, "C0_BASE=0x%08x C_SIZE_BYTES=%0d C0_WRITES=%0d C0_READS=%0d C0_NONZERO_WRITES=%0d", C0_BASE, C_SIZE, wr_C0, rd_C0, nonzero_C0);
            $fdisplay(fh, "D0_BASE=0x%08x D_SIZE_BYTES=%0d D0_WRITES=%0d D0_READS=%0d D0_NONZERO_WRITES=%0d", D0_BASE, D_SIZE, wr_D0, rd_D0, nonzero_D0);
            $fdisplay(fh, "B1_BASE=0x%08x B_SIZE_BYTES=%0d B1_WRITES=%0d B1_READS=%0d B1_NONZERO_WRITES=%0d", B1_BASE, B_SIZE, wr_B1, rd_B1, nonzero_B1);
            $fdisplay(fh, "C1_BASE=0x%08x C_SIZE_BYTES=%0d C1_WRITES=%0d C1_READS=%0d C1_NONZERO_WRITES=%0d", C1_BASE, C_SIZE, wr_C1, rd_C1, nonzero_C1);
            $fdisplay(fh, "D1_BASE=0x%08x D_SIZE_BYTES=%0d D1_WRITES=%0d D1_READS=%0d D1_NONZERO_WRITES=%0d", D1_BASE, D_SIZE, wr_D1, rd_D1, nonzero_D1);
            $fdisplay(fh, "OTHER_WRITES=%0d OTHER_READS=%0d OTHER_NONZERO_WRITES=%0d", wr_other, rd_other, nonzero_other);
            $fclose(fh);
            $display("INFO: wrote DDR dump summary: %0s", path);
        end
    endtask

    task dump_all_ddr_regions;
        begin
            if (dump_ddr_enable == 0) begin
                $display("INFO: DDR dump disabled by +DUMP_DDR=0");
            end else begin
                dump_region_words_hex({ddr_dump_prefix, "_A_scatter_words.hex"}, A_BASE, A_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_B0_cache_words.hex"},  B0_BASE, B_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_C0_rdmap_words.hex"},  C0_BASE, C_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_D0_cfar_words.hex"},   D0_BASE, D_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_B1_cache_words.hex"},  B1_BASE, B_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_C1_rdmap_words.hex"},  C1_BASE, C_SIZE);
                dump_region_words_hex({ddr_dump_prefix, "_D1_cfar_words.hex"},   D1_BASE, D_SIZE);
                dump_summary_txt({ddr_dump_prefix, "_summary.txt"});
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // AXI slave model macros.
    //
    // These use negedge clk to avoid racing the DUT, which samples AXI signals
    // on posedge clk.  The model supports one outstanding burst per AXI port.
    // That is enough for the simple master blocks used in this radar pipeline.
    // ------------------------------------------------------------------------
`define AXI_WRITE_SLAVE(P) \
    reg [31:0] P``_wr_addr; \
    reg [8:0]  P``_wr_beats_left; \
    reg [2:0]  P``_wr_size; \
    reg [1:0]  P``_wr_burst; \
    reg        P``_wr_active; \
    // bvalid_pending: bvalid was SET this negedge; do not clear until next negedge so \
    // the DUT's posedge always-block has a guaranteed window to sample it. \
    // Without this guard, XSim's initial-block negedge scheduling can set and clear \
    // bvalid in consecutive negedges straddling a posedge, making the DUT miss it. \
    reg        P``_bvalid_pending; \
    initial begin : P``_write_slave \
        P``_wr_addr = 32'd0; \
        P``_wr_beats_left = 9'd0; \
        P``_wr_size = 3'd2; \
        P``_wr_burst = 2'd1; \
        P``_wr_active = 1'b0; \
        P``_bvalid_pending = 1'b0; \
        P``_awready = 1'b1; \
        P``_wready = 1'b1; \
        P``_bresp = 2'b00; \
        P``_bvalid = 1'b0; \
        forever begin \
            @(negedge clk); \
            if (!rst_n) begin \
                P``_wr_addr = 32'd0; \
                P``_wr_beats_left = 9'd0; \
                P``_wr_size = 3'd2; \
                P``_wr_burst = 2'd1; \
                P``_wr_active = 1'b0; \
                P``_bvalid_pending = 1'b0; \
                P``_awready = 1'b1; \
                P``_wready = 1'b1; \
                P``_bresp = 2'b00; \
                P``_bvalid = 1'b0; \
            end else begin \
                P``_awready = !P``_wr_active; \
                P``_wready = 1'b1; \
                // Only clear bvalid if it was already asserted on the PREVIOUS negedge \
                // (P_bvalid_pending=0 now) so the DUT posedge between the two negedges \
                // has a full half-cycle window to sample bvalid=1. \
                if (P``_bvalid && P``_bready && !P``_bvalid_pending) begin \
                    P``_bvalid = 1'b0; \
                end \
                P``_bvalid_pending = 1'b0; \
                if (!P``_wr_active && P``_awvalid && P``_awready) begin \
                    P``_wr_addr = P``_awaddr; \
                    P``_wr_beats_left = {1'b0, P``_awlen} + 9'd1; \
                    P``_wr_size = P``_awsize; \
                    P``_wr_burst = P``_awburst; \
                    P``_wr_active = 1'b1; \
                end \
                if (P``_wvalid && P``_wready) begin \
                    if (!P``_wr_active) begin \
                        $display("ERROR: %s W beat before AW at %0t", `"P`", $time); \
                        $fatal(1); \
                    end \
                    ddr_write32(P``_wr_addr, P``_wdata, P``_wstrb); \
                    note_write(P``_wr_addr, P``_wdata); \
                    if (P``_wr_beats_left == 9'd1) begin \
                        if (!P``_wlast) begin \
                            $display("ERROR: %s missing WLAST at final beat, addr=0x%08x", `"P`", P``_wr_addr); \
                            $fatal(1); \
                        end \
                        P``_wr_active = 1'b0; \
                        P``_wr_beats_left = 9'd0; \
                        P``_bresp = 2'b00; \
                        P``_bvalid = 1'b1; \
                        P``_bvalid_pending = 1'b1; \
                    end else begin \
                        if (P``_wlast) begin \
                            $display("ERROR: %s early WLAST, addr=0x%08x beats_left=%0d", `"P`", P``_wr_addr, P``_wr_beats_left); \
                            $fatal(1); \
                        end \
                        P``_wr_beats_left = P``_wr_beats_left - 9'd1; \
                        if (P``_wr_burst == 2'd1) begin \
                            P``_wr_addr = P``_wr_addr + (32'd1 << P``_wr_size); \
                        end \
                    end \
                end \
            end \
        end \
    end

`define AXI_READ_SLAVE(P) \
    reg [31:0] P``_rd_addr; \
    reg [8:0]  P``_rd_beats_left; \
    reg [2:0]  P``_rd_size; \
    reg [1:0]  P``_rd_burst; \
    reg        P``_rd_active; \
    reg        P``_rd_accepted; \
    always @(posedge clk or negedge rst_n) begin : P``_read_accept_sample \
        if (!rst_n) begin \
            P``_rd_accepted <= 1'b0; \
        end else begin \
            P``_rd_accepted <= P``_rvalid && P``_rready; \
        end \
    end \
    initial begin : P``_read_slave \
        P``_rd_addr = 32'd0; \
        P``_rd_beats_left = 9'd0; \
        P``_rd_size = 3'd2; \
        P``_rd_burst = 2'd1; \
        P``_rd_active = 1'b0; \
        P``_arready = 1'b1; \
        P``_rdata = 32'd0; \
        P``_rresp = 2'b00; \
        P``_rlast = 1'b0; \
        P``_rvalid = 1'b0; \
        forever begin \
            @(negedge clk); \
            if (!rst_n) begin \
                P``_rd_addr = 32'd0; \
                P``_rd_beats_left = 9'd0; \
                P``_rd_size = 3'd2; \
                P``_rd_burst = 2'd1; \
                P``_rd_active = 1'b0; \
                P``_arready = 1'b1; \
                P``_rdata = 32'd0; \
                P``_rresp = 2'b00; \
                P``_rlast = 1'b0; \
                P``_rvalid = 1'b0; \
            end else begin \
                P``_arready = (!P``_rd_active && !(P``_rvalid && !P``_rd_accepted)); \
                if (P``_rvalid && !P``_rd_accepted) begin \
                    P``_rvalid = P``_rvalid; \
                end else begin \
                    if (P``_rd_active) begin \
                        P``_rdata = ddr_read32(P``_rd_addr); \
                        P``_rresp = 2'b00; \
                        P``_rlast = (P``_rd_beats_left == 9'd1); \
                        P``_rvalid = 1'b1; \
                        note_read(P``_rd_addr); \
                        if (P``_rd_beats_left == 9'd1) begin \
                            P``_rd_active = 1'b0; \
                            P``_rd_beats_left = 9'd0; \
                        end else begin \
                            P``_rd_beats_left = P``_rd_beats_left - 9'd1; \
                            if (P``_rd_burst == 2'd1) begin \
                                P``_rd_addr = P``_rd_addr + (32'd1 << P``_rd_size); \
                            end \
                        end \
                    end else if (P``_arvalid && P``_arready) begin \
                        P``_rvalid = 1'b0; \
                        P``_rlast = 1'b0; \
                        P``_rd_active = 1'b1; \
                        P``_rd_beats_left = {1'b0, P``_arlen} + 9'd1; \
                        P``_rd_size = P``_arsize; \
                        P``_rd_burst = P``_arburst; \
                        P``_rd_addr = P``_araddr; \
                    end else begin \
                        P``_rvalid = 1'b0; \
                        P``_rlast = 1'b0; \
                    end \
                end \
            end \
        end \
    end

    // Write buses: scatter, complex cache, RD-map collector, CFAR output.
    `AXI_WRITE_SLAVE(sc0)
    `AXI_WRITE_SLAVE(sc1)
    `AXI_WRITE_SLAVE(sc2)
    `AXI_WRITE_SLAVE(cc0)
    `AXI_WRITE_SLAVE(cc1)
    `AXI_WRITE_SLAVE(cc2)
    `AXI_WRITE_SLAVE(rd)
    `AXI_WRITE_SLAVE(cfar)

    // Read buses: doppler sequencers and CFAR input.
    `AXI_READ_SLAVE(ds0)
    `AXI_READ_SLAVE(ds1)
    `AXI_READ_SLAVE(ds2)
    `AXI_READ_SLAVE(cfar)

    always @(posedge clk) begin
        if (rst_n && cfar_start && (cfar_frame_buf_sel !== frame_buf_sel)) begin
            $display("ERROR: CFAR buffer select mismatch at start: expected=%0d got=%0d time=%0t",
                     frame_buf_sel, cfar_frame_buf_sel, $time);
            $fatal(1);
        end
    end

    // ------------------------------------------------------------------------
    // Reset, stimulus loading, PPI driver, watchdog, and checks.
    // ------------------------------------------------------------------------
    initial begin : init_memories_and_stats
        for (init_i = 0; init_i < A_WORDS; init_i = init_i + 1) begin
            ddr_A[init_i] = 32'd0;
        end
        for (init_i = 0; init_i < B_WORDS; init_i = init_i + 1) begin
            ddr_B0[init_i] = 32'd0;
            ddr_B1[init_i] = 32'd0;
        end
        for (init_i = 0; init_i < C_WORDS; init_i = init_i + 1) begin
            ddr_C0[init_i] = 32'd0;
            ddr_C1[init_i] = 32'd0;
        end
        for (init_i = 0; init_i < D_WORDS; init_i = init_i + 1) begin
            ddr_D0[init_i] = 32'd0;
            ddr_D1[init_i] = 32'd0;
        end

        wr_A = 0; wr_B0 = 0; wr_C0 = 0; wr_D0 = 0; wr_B1 = 0; wr_C1 = 0; wr_D1 = 0; wr_other = 0;
        rd_A = 0; rd_B0 = 0; rd_C0 = 0; rd_D0 = 0; rd_B1 = 0; rd_C1 = 0; rd_D1 = 0; rd_other = 0;
        nonzero_A = 0; nonzero_B0 = 0; nonzero_C0 = 0; nonzero_D0 = 0; nonzero_B1 = 0; nonzero_C1 = 0; nonzero_D1 = 0; nonzero_other = 0;
        adar_valid_count = 0;
        tdm_valid_count = 0;
        bpf_valid_count = 0;
        hilbert_valid_count = 0;
        hilbert_accept_count = 0;
        aligned_iq_count = 0;
        range0_valid_count = 0;
        range1_valid_count = 0;
        range2_valid_count = 0;
        rfft0_valid_count = 0;
        rfft1_valid_count = 0;
        rfft2_valid_count = 0;
        rfft3_valid_count = 0;
        dseq0_accept_count = 0;
        dseq1_accept_count = 0;
        dseq2_accept_count = 0;
        dseq0_last_count = 0;
        dseq1_last_count = 0;
        dseq2_last_count = 0;
        dseq0_fft_accept_count = 0;
        dseq1_fft_accept_count = 0;
        dseq2_fft_accept_count = 0;
        dfft0_accept_count = 0;
        dfft1_accept_count = 0;
        dfft2_accept_count = 0;
        dfft0_last_count = 0;
        dfft1_last_count = 0;
        dfft2_last_count = 0;
        dfft0_fan_accept_count = 0;
        dfft1_fan_accept_count = 0;
        dfft2_fan_accept_count = 0;
        noncoh_accept_count = 0;

        if (!$value$plusargs("STIM_HEX=%s", stim_hex_path)) begin
            stim_hex_path = "ppi_stream.hex";
        end
        // ── B/C/D buffer-set select (the frame_buf_sel the top-level would
        //    normally toggle per frame).  This testbench runs ONE frame, so it
        //    drives frame_buf_sel as a STATIC constant (default 1) for the whole
        //    run: cplx_cell_cache_top and rd_map_collector_summed both take it,
        //    and the collector forwards its latched value as cfar_frame_buf_sel
        //    to CFAR — so B, C and D all land in the same set.  With a constant 1
        //    everything writes set 1 (B1/C1/D1) and set 0 stays empty, which is
        //    exactly what the DDR dump confirms.  A real 40-FPS build must
        //    generate a TOGGLING frame_buf_sel (held stable within a frame,
        //    flipped after frame_complete) so producer/consumer double-buffer.
        if (!$value$plusargs("FRAME_BUF_SEL=%d", frame_buf_sel_int)) begin
            frame_buf_sel_int = 1;
        end
        if (frame_buf_sel_int == 0) begin
            frame_buf_sel = 1'b0;
        end else if (frame_buf_sel_int == 1) begin
            frame_buf_sel = 1'b1;
        end else begin
            $display("ERROR: FRAME_BUF_SEL must be 0 or 1, got %0d", frame_buf_sel_int);
            $fatal(1);
        end
        if (!$value$plusargs("DDR_DUMP_PREFIX=%s", ddr_dump_prefix)) begin
            ddr_dump_prefix = "ddr";
        end
        if (!$value$plusargs("DUMP_DDR=%d", dump_ddr_enable)) begin
            dump_ddr_enable = 1;
        end
        if (!$value$plusargs("SMOKE_MODE=%d", smoke_mode)) begin
            smoke_mode = 0;
        end
        if (!$value$plusargs("STIM_LIMIT_BYTES=%d", stim_limit_bytes)) begin
            stim_limit_bytes = 0;
        end
        if (!$value$plusargs("SMOKE_DRAIN_CYCLES=%d", smoke_drain_cycles)) begin
            smoke_drain_cycles = 512;
        end
        if (!$value$plusargs("TIMEOUT_CLK_CYCLES=%d", timeout_cycles)) begin
            timeout_cycles = TIMEOUT_CLK_CYCLES;
        end
        if (!$value$plusargs("PPI_PROGRESS_BYTES=%d", ppi_progress_bytes)) begin
            ppi_progress_bytes = 4096;
        end
        if (!$value$plusargs("STATUS_INTERVAL_CYCLES=%d", status_interval_cycles)) begin
            status_interval_cycles = 0;
        end
        if (timeout_cycles <= 0) begin
            timeout_cycles = TIMEOUT_CLK_CYCLES;
        end
        if (smoke_drain_cycles < 0) begin
            $display("ERROR: SMOKE_DRAIN_CYCLES must be >= 0, got %0d", smoke_drain_cycles);
            $fatal(1);
        end
        if (ppi_progress_bytes < PPI_BYTES_PER_SAMPLE_SET) begin
            ppi_progress_bytes = 0;
        end
        if (status_interval_cycles < 0) begin
            status_interval_cycles = 0;
        end
        if ((stim_limit_bytes <= 0) || (stim_limit_bytes > PPI_STREAM_BYTES)) begin
            stim_drive_bytes = PPI_STREAM_BYTES;
        end else begin
            stim_drive_bytes = stim_limit_bytes;
        end
        stim_drive_bytes = (stim_drive_bytes / PPI_BYTES_PER_SAMPLE_SET) * PPI_BYTES_PER_SAMPLE_SET;
        if (stim_drive_bytes < PPI_BYTES_PER_SAMPLE_SET) begin
            $display("ERROR: STIM_LIMIT_BYTES leaves no complete ADAR7251 sample set: %0d", stim_limit_bytes);
            $fatal(1);
        end
        if ((smoke_mode == 0) && (stim_drive_bytes != PPI_STREAM_BYTES)) begin
            $display("ERROR: STIM_LIMIT_BYTES=%0d is a reduced-data run and requires SMOKE_MODE=1", stim_drive_bytes);
            $fatal(1);
        end

        $display("INFO: reading PPI stimulus hex: %0s", stim_hex_path);
        $display("INFO: DDR dump prefix: %0s", ddr_dump_prefix);
        $display("INFO: DUMP_DDR=%0d", dump_ddr_enable);
        $display("INFO: FRAME_BUF_SEL=%0d", frame_buf_sel_int);
        $display("INFO: SMOKE_MODE=%0d", smoke_mode);
        $display("INFO: STIM_DRIVE_BYTES=%0d/%0d", stim_drive_bytes, PPI_STREAM_BYTES);
        $display("INFO: SMOKE_DRAIN_CYCLES=%0d", smoke_drain_cycles);
        $display("INFO: TIMEOUT_CLK_CYCLES=%0d", timeout_cycles);
        $display("INFO: PPI_PROGRESS_BYTES=%0d", ppi_progress_bytes);
        $display("INFO: STATUS_INTERVAL_CYCLES=%0d", status_interval_cycles);
        $readmemh(stim_hex_path, ppi_mem);
        if ($isunknown(ppi_mem[0]) || $isunknown(ppi_mem[stim_drive_bytes-1])) begin
            $display("ERROR: PPI stimulus appears short/unreadable. First=0x%02x LastDriven=0x%02x",
                     ppi_mem[0], ppi_mem[stim_drive_bytes-1]);
            $fatal(1);
        end
    end

    initial begin : reset_driver
        rst_n = 1'b0;
        data_ready = 1'b0;
        dout = 8'd0;

        repeat (RESET_CLK_CYCLES) @(posedge clk);
        rst_n = 1'b1;
        repeat (POST_RESET_CYCLES) @(posedge clk);

        $display("INFO: reset released at %0t", $time);
    end

    // ========================================================================
    //  ppi_driver — THE STIMULUS SOURCE (emulates the ADAR7251 ADC chip)
    // ------------------------------------------------------------------------
    //  This is the block that "plays" the recorded radar frame into the DUT.
    //  It replaces the physical ADAR7251 by bit-banging its PPI byte-mode bus
    //  (dout[7:0], data_ready) on the sclk_adc clock, exactly as the chip would.
    //
    //  What is in ppi_mem:
    //    gen_ppi_hex_3rx.py wrote one full frame of PPI bytes into ppi_stream.hex,
    //    which init_memories_and_stats $readmemh'd into ppi_mem[].  The stream is
    //    a flat sequence of 8-byte "sample sets".  ONE sample set = one ADC sample
    //    from every channel:
    //        byte 0,1 = CH1 high,low     byte 2,3 = CH2 high,low
    //        byte 4,5 = CH3 high,low     byte 6,7 = CH4 high,low  (CH4 unused @3RX)
    //    High byte first (PAR_ENDIAN=0), 16-bit signed per channel.
    //    A full frame = RANGE_BINS * DOPPLER_BINS = 128 * 256 = 32768 sample sets
    //    per channel (so stim_drive_bytes = 32768 * 8 for one frame).
    //
    //  Timing contract with adar7251_ppi_rx (the DUT front end):
    //    * The RX FSM starts a capture on the RISING edge of data_ready, then
    //      latches dout on the next 8 sclk_adc RISING edges (bytes 0..7), and
    //      pulses valid_out after byte 7.
    //    * We therefore change dout/data_ready on the FALLING edge of sclk_adc so
    //      each byte is stable well before the rising edge the DUT samples on
    //      (setup/hold safe; also dodges any NBA-vs-blocking races).
    //    * data_ready is PULSED per sample set (high for the 8-byte window, then
    //      low), NOT held high for the whole frame.  If it were held high the RX
    //      FSM would only ever start ONE capture and the rest of the frame would
    //      be dropped — a subtle but fatal bug this pulsing avoids.
    //
    //  Loop: walk ppi_mem 8 bytes (one sample set) at a time.  Each iteration
    //  drives data_ready high, streams the 8 data bytes, then drops data_ready —
    //  producing exactly one valid_out from the RX per iteration, i.e. one
    //  {ch0,ch1,ch2} sample into the TDM front end.
    // ========================================================================
    initial begin : ppi_driver
        wait (rst_n == 1'b1);
        repeat (POST_RESET_CYCLES) @(posedge clk);

        $display("INFO: driving %0d/%0d PPI bytes", stim_drive_bytes, PPI_STREAM_BYTES);

        for (ppi_i = 0; ppi_i < stim_drive_bytes; ppi_i = ppi_i + PPI_BYTES_PER_SAMPLE_SET) begin
            // ── Start-of-sample-set: raise DATA_READY (dout still idle) ──────
            // The rising edge here is what the RX FSM edge-detects to begin the
            // 8-byte capture; the next 8 rising edges carry the data bytes.
            @(negedge sclk_adc);
            data_ready = 1'b1;
            dout = 8'd0;
            @(posedge sclk_adc);            // RX sees data_ready rising here

            // ── Bytes 0..7: change on negedge, RX samples on the next posedge ─
            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 0];      // CH1 high byte
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 1];      // CH1 low byte  -> ch0_out complete
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 2];      // CH2 high
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 3];      // CH2 low       -> ch1_out complete
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 4];      // CH3 high
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 5];      // CH3 low       -> ch2_out complete
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 6];      // CH4 high (clocked for timing, discarded)
            @(posedge sclk_adc);

            @(negedge sclk_adc);
            dout = ppi_mem[ppi_i + 7];      // CH4 low  -> RX pulses valid_out after this
            @(posedge sclk_adc);

            // ── End-of-sample-set: drop DATA_READY so the next iteration's ───
            // rising edge is a clean new-capture trigger.
            @(negedge sclk_adc);
            data_ready = 1'b0;
            dout = 8'd0;
            @(posedge sclk_adc);

            if ((ppi_progress_bytes != 0) &&
                ((((ppi_i + PPI_BYTES_PER_SAMPLE_SET) % ppi_progress_bytes) == 0) ||
                 ((ppi_i + PPI_BYTES_PER_SAMPLE_SET) >= stim_drive_bytes))) begin
                $display("INFO: drove %0d/%0d PPI bytes at %0t",
                         ppi_i + PPI_BYTES_PER_SAMPLE_SET, stim_drive_bytes, $time);
            end
        end

        $display("INFO: finished driving PPI stream at %0t", $time);

        if (smoke_mode != 0) begin
            $display("INFO: smoke drain for %0d clk cycles", smoke_drain_cycles);
            repeat (smoke_drain_cycles) @(posedge clk);
            if (cfar_axi_error === 1'b1) begin
                $display("ERROR: cfar_axi_error asserted during reduced-data smoke");
                $fatal(1);
            end
            $display("INFO: smoke DDR writes A=%0d B0=%0d C0=%0d D0=%0d B1=%0d C1=%0d D1=%0d other=%0d",
                     wr_A, wr_B0, wr_C0, wr_D0, wr_B1, wr_C1, wr_D1, wr_other);
            $display("INFO: smoke DDR reads  A=%0d B0=%0d C0=%0d D0=%0d B1=%0d C1=%0d D1=%0d other=%0d",
                     rd_A, rd_B0, rd_C0, rd_D0, rd_B1, rd_C1, rd_D1, rd_other);
            $display("PASS: XSim reduced-data radar smoke completed");
            $finish;
        end
    end

    always @(posedge clk or negedge rst_n) begin : progress_monitor
        if (!rst_n) begin
            rd_buf_valid_d <= 4'd0;
            doppler_frame_done_d <= 4'd0;
            cfar_start_d <= 1'b0;
            cfar_done_d <= 1'b0;
            rdmap_frame_complete_d <= 1'b0;
            status_cycle_count <= 0;
        end else begin
            rd_buf_valid_d <= rd_buf_valid;
            doppler_frame_done_d <= doppler_frame_done;
            cfar_start_d <= cfar_start;
            cfar_done_d <= cfar_done;
            rdmap_frame_complete_d <= rdmap_frame_complete;

            if ((rd_buf_valid & ~rd_buf_valid_d) != 4'd0) begin
                $display("INFO: rd_buf_valid rise bits=0x%0x at %0t", rd_buf_valid & ~rd_buf_valid_d, $time);
            end
            if ((doppler_frame_done & ~doppler_frame_done_d) != 4'd0) begin
                $display("INFO: doppler_frame_done rise bits=0x%0x at %0t", doppler_frame_done & ~doppler_frame_done_d, $time);
            end
            if (cfar_start && !cfar_start_d) begin
                $display("INFO: cfar_start rise buf_sel=%0d at %0t", cfar_frame_buf_sel, $time);
            end
            if (cfar_done && !cfar_done_d) begin
                $display("INFO: cfar_done rise at %0t", $time);
            end
            if (rdmap_frame_complete && !rdmap_frame_complete_d) begin
                $display("INFO: rdmap_frame_complete rise at %0t", $time);
            end

            if (status_interval_cycles != 0) begin
                if (status_cycle_count >= status_interval_cycles) begin
                    status_cycle_count <= 0;
                    $display("INFO: status ppi_i=%0d adar=%0d tdm=%0d rfft=%0d/%0d/%0d noncoh=%0d wr_A=%0d rd_A=%0d wr_C0=%0d wr_C1=%0d wr_D0=%0d wr_D1=%0d cfar_busy=%0b time=%0t",
                             ppi_i, adar_valid_count, tdm_valid_count,
                             rfft0_valid_count, rfft1_valid_count, rfft2_valid_count,
                             noncoh_accept_count, wr_A, rd_A, wr_C0, wr_C1, wr_D0, wr_D1,
                             cfar_busy, $time);
                end else begin
                    status_cycle_count <= status_cycle_count + 1;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin : frontend_counter_monitor
        if (!rst_n) begin
            adar_valid_count <= 0;
            tdm_valid_count <= 0;
            bpf_valid_count <= 0;
            hilbert_valid_count <= 0;
            hilbert_accept_count <= 0;
            aligned_iq_count <= 0;
            range0_valid_count <= 0;
            range1_valid_count <= 0;
            range2_valid_count <= 0;
            rfft0_valid_count <= 0;
            rfft1_valid_count <= 0;
            rfft2_valid_count <= 0;
            rfft3_valid_count <= 0;
            dseq0_accept_count <= 0;
            dseq1_accept_count <= 0;
            dseq2_accept_count <= 0;
            dseq0_last_count <= 0;
            dseq1_last_count <= 0;
            dseq2_last_count <= 0;
            dseq0_fft_accept_count <= 0;
            dseq1_fft_accept_count <= 0;
            dseq2_fft_accept_count <= 0;
            dfft0_accept_count <= 0;
            dfft1_accept_count <= 0;
            dfft2_accept_count <= 0;
            dfft0_last_count <= 0;
            dfft1_last_count <= 0;
            dfft2_last_count <= 0;
            dfft0_fan_accept_count <= 0;
            dfft1_fan_accept_count <= 0;
            dfft2_fan_accept_count <= 0;
            noncoh_accept_count <= 0;
        end else begin
            if (adar_valid) adar_valid_count <= adar_valid_count + 1;
            if (tdm_valid) tdm_valid_count <= tdm_valid_count + 1;
            if (bpf_tvalid) bpf_valid_count <= bpf_valid_count + 1;
            if (hilbert_tvalid) hilbert_valid_count <= hilbert_valid_count + 1;
            if (hilbert_accept) hilbert_accept_count <= hilbert_accept_count + 1;
            if (iq_i_valid && iq_q_valid) aligned_iq_count <= aligned_iq_count + 1;
            if (range0_tvalid) range0_valid_count <= range0_valid_count + 1;
            if (range1_tvalid) range1_valid_count <= range1_valid_count + 1;
            if (range2_tvalid) range2_valid_count <= range2_valid_count + 1;
            if (rfft0_tvalid && rfft0_tready) rfft0_valid_count <= rfft0_valid_count + 1;
            if (rfft1_tvalid && rfft1_tready) rfft1_valid_count <= rfft1_valid_count + 1;
            if (rfft2_tvalid && rfft2_tready) rfft2_valid_count <= rfft2_valid_count + 1;
            if (rfft3_tvalid && rfft3_tready) rfft3_valid_count <= rfft3_valid_count + 1;
            if (dseq0_tvalid && dseq0_tready) dseq0_accept_count <= dseq0_accept_count + 1;
            if (dseq1_tvalid && dseq1_tready) dseq1_accept_count <= dseq1_accept_count + 1;
            if (dseq2_tvalid && dseq2_tready) dseq2_accept_count <= dseq2_accept_count + 1;
            if (dseq0_tvalid && dseq0_tready && dseq0_tlast) dseq0_last_count <= dseq0_last_count + 1;
            if (dseq1_tvalid && dseq1_tready && dseq1_tlast) dseq1_last_count <= dseq1_last_count + 1;
            if (dseq2_tvalid && dseq2_tready && dseq2_tlast) dseq2_last_count <= dseq2_last_count + 1;
            if (dseq0_fft_tvalid && dseq0_fft_tready) dseq0_fft_accept_count <= dseq0_fft_accept_count + 1;
            if (dseq1_fft_tvalid && dseq1_fft_tready) dseq1_fft_accept_count <= dseq1_fft_accept_count + 1;
            if (dseq2_fft_tvalid && dseq2_fft_tready) dseq2_fft_accept_count <= dseq2_fft_accept_count + 1;
            if (dfft0_tvalid && dfft0_tready) dfft0_accept_count <= dfft0_accept_count + 1;
            if (dfft1_tvalid && dfft1_tready) dfft1_accept_count <= dfft1_accept_count + 1;
            if (dfft2_tvalid && dfft2_tready) dfft2_accept_count <= dfft2_accept_count + 1;
            if (dfft0_tvalid && dfft0_tready && dfft0_tlast) dfft0_last_count <= dfft0_last_count + 1;
            if (dfft1_tvalid && dfft1_tready && dfft1_tlast) dfft1_last_count <= dfft1_last_count + 1;
            if (dfft2_tvalid && dfft2_tready && dfft2_tlast) dfft2_last_count <= dfft2_last_count + 1;
            if (dfft0_fan_tvalid && dfft0_fan_tready) dfft0_fan_accept_count <= dfft0_fan_accept_count + 1;
            if (dfft1_fan_tvalid && dfft1_fan_tready) dfft1_fan_accept_count <= dfft1_fan_accept_count + 1;
            if (dfft2_fan_tvalid && dfft2_fan_tready) dfft2_fan_accept_count <= dfft2_fan_accept_count + 1;
            if (noncoh_tvalid && noncoh_tready) noncoh_accept_count <= noncoh_accept_count + 1;
        end
    end

    initial begin : watchdog
        wait (rst_n == 1'b1);
        if (smoke_mode != 0) begin
            disable watchdog;
        end
        for (timeout_i = 0; timeout_i < timeout_cycles; timeout_i = timeout_i + 1) begin
            @(posedge clk);
            if (rdmap_frame_complete === 1'b1) begin
                disable watchdog;
            end
        end
        $display("ERROR: timeout waiting for rdmap_frame_complete after %0d clk cycles", timeout_cycles);
        $fatal(1);
    end

    initial begin : completion_checks
        wait (rst_n == 1'b1);
        if (smoke_mode != 0) begin
            disable completion_checks;
        end
        wait (rdmap_frame_complete === 1'b1);
        $display("INFO: rdmap_frame_complete observed at %0t", $time);

        // Let any last-cycle AXI responses settle before checking counters.
        repeat (32) @(posedge clk);

        if (cfar_axi_error !== 1'b0) begin
            $display("ERROR: cfar_axi_error asserted");
            $fatal(1);
        end
        if (wr_A <= 0) begin $display("ERROR: scatter stage did not write DDR A"); $fatal(1); end
        if (rd_A <= 0) begin $display("ERROR: doppler sequencer did not read DDR A"); $fatal(1); end
        if (nonzero_A <= 0) begin $display("ERROR: DDR A writes were all zero"); $fatal(1); end

        if (frame_buf_sel == 1'b0) begin
            if (wr_B0 <= 0) begin $display("ERROR: complex cache did not write B0"); $fatal(1); end
            if (wr_C0 <= 0) begin $display("ERROR: RD-map collector did not write C0"); $fatal(1); end
            if (wr_D0 <= 0) begin $display("ERROR: CFAR did not write D0"); $fatal(1); end
            if (rd_C0 <= 0) begin $display("ERROR: CFAR did not read C0"); $fatal(1); end
            if (nonzero_B0 <= 0) begin $display("ERROR: B0 writes were all zero"); $fatal(1); end
            if (nonzero_C0 <= 0) begin $display("ERROR: C0 writes were all zero"); $fatal(1); end
            if (wr_B1 != 0 || wr_C1 != 0 || wr_D1 != 0 || rd_C1 != 0) begin
                $display("ERROR: frame_buf_sel=0 touched B1/C1/D1: wr_B1=%0d wr_C1=%0d wr_D1=%0d rd_C1=%0d",
                         wr_B1, wr_C1, wr_D1, rd_C1);
                $fatal(1);
            end
        end else begin
            if (wr_B1 <= 0) begin $display("ERROR: complex cache did not write B1"); $fatal(1); end
            if (wr_C1 <= 0) begin $display("ERROR: RD-map collector did not write C1"); $fatal(1); end
            if (wr_D1 <= 0) begin $display("ERROR: CFAR did not write D1"); $fatal(1); end
            if (rd_C1 <= 0) begin $display("ERROR: CFAR did not read C1"); $fatal(1); end
            if (nonzero_B1 <= 0) begin $display("ERROR: B1 writes were all zero"); $fatal(1); end
            if (nonzero_C1 <= 0) begin $display("ERROR: C1 writes were all zero"); $fatal(1); end
            if (wr_B0 != 0 || wr_C0 != 0 || wr_D0 != 0 || rd_C0 != 0) begin
                $display("ERROR: frame_buf_sel=1 touched B0/C0/D0: wr_B0=%0d wr_C0=%0d wr_D0=%0d rd_C0=%0d",
                         wr_B0, wr_C0, wr_D0, rd_C0);
                $fatal(1);
            end
        end

        $display("INFO: DDR writes A=%0d B0=%0d C0=%0d D0=%0d B1=%0d C1=%0d D1=%0d other=%0d",
                 wr_A, wr_B0, wr_C0, wr_D0, wr_B1, wr_C1, wr_D1, wr_other);
        $display("INFO: DDR reads  A=%0d B0=%0d C0=%0d D0=%0d B1=%0d C1=%0d D1=%0d other=%0d",
                 rd_A, rd_B0, rd_C0, rd_D0, rd_B1, rd_C1, rd_D1, rd_other);
        $display("INFO: nonzero writes A=%0d B0=%0d C0=%0d D0=%0d B1=%0d C1=%0d D1=%0d other=%0d",
                 nonzero_A, nonzero_B0, nonzero_C0, nonzero_D0, nonzero_B1, nonzero_C1, nonzero_D1, nonzero_other);

        dump_all_ddr_regions;

        $display("PASS: XSim end-to-end radar B/C/D ping-pong pipeline completed without board or cocotb");
        $finish;
    end

endmodule

// ============================================================================
// axis_fifo_simple — simulation-only FIFO used to buffer FFT outputs
// ============================================================================
module axis_fifo_simple #(
    parameter DATA_W = 33,
    parameter DEPTH  = 4096
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DATA_W-1:0] s_tdata,
    input  wire              s_tvalid,
    output wire              s_tready,
    output wire [DATA_W-1:0] m_tdata,
    output wire              m_tvalid,
    input  wire              m_tready
);
    localparam PTR_W = $clog2(DEPTH);
    localparam [PTR_W:0] DEPTH_COUNT = DEPTH;

    reg [DATA_W-1:0] mem [0:DEPTH-1];
    reg [PTR_W-1:0]  wr_ptr;
    reg [PTR_W-1:0]  rd_ptr;
    reg [PTR_W:0]    count;

    wire read_fire  = (count != {(PTR_W+1){1'b0}}) && m_tready;
    wire write_fire = s_tvalid && s_tready;

    assign m_tvalid = (count != {(PTR_W+1){1'b0}});
    assign s_tready = (count < DEPTH_COUNT) || read_fire;
    assign m_tdata  = mem[rd_ptr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {PTR_W{1'b0}};
            rd_ptr <= {PTR_W{1'b0}};
            count  <= {(PTR_W+1){1'b0}};
        end else begin
            if (s_tvalid && !s_tready) begin
                $display("ERROR: axis_fifo_simple overflow at %0t", $time);
                $fatal(1);
            end
            if (write_fire) begin
                mem[wr_ptr] <= s_tdata;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (read_fire) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
            case ({write_fire, read_fire})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: ;
            endcase
        end
    end
endmodule

`default_nettype wire
