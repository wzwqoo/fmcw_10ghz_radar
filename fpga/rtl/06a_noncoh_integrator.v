// =============================================================================
//  noncoh_integrator.v   *** 3-RX VARIANT ***
//
//  Non-coherent integration of 3 parallel complex Doppler-FFT streams.
//
//  For each cell (range_bin, doppler_bin) the module computes:
//      out = | sum_{rx=0..2} ( I_rx^2 + Q_rx^2 ) | saturated to OUT_W bits
//
//  Why
//  ---
//  CFAR operates on power (|X|^2).  Summing |X|^2 across 4 RX antennas before
//  CFAR gives ~6 dB SNR improvement vs single-channel CFAR.  Phase information
//  is discarded here — angle-of-arrival uses cplx_cell_cache, a parallel tap
//  off the same 4 streams.
//
//  Interface
//  ---------
//  Four AXI4-Stream inputs (sN_*), one per RX antenna.  tdata format is
//  signed I in the upper IQ_W bits and signed Q in the lower IQ_W bits
//  (matches Xilinx FFT IP "complex" output layout).
//
//  All 4 streams must produce data on the same cycle (cycle-aligned).  This is
//  the natural condition when the 4 Doppler FFTs are 4 instances of the same
//  Vivado IP fed by a cycle-aligned source.  If your topology can produce
//  misaligned streams, place an AXIS-Broadcaster + per-channel skid FIFO
//  upstream of this module.
//
//  Output is a single AXI4-Stream of OUT_W-bit unsigned summed magnitude.
//  tlast is taken from input channel 0 (all 4 streams' tlast must match).
//
//  Pipeline
//  --------
//      Stage 0 : capture I,Q for all 4 channels
//      Stage 1 : compute I*I and Q*Q for all 4 (8 DSP48s)
//      Stage 2 : per-channel sum I^2 + Q^2 (4 adds), pairwise sum (2 adds)
//      Stage 3 : final sum and saturation
//  Latency : 4 cycles.  Throughput : 1 cell/cycle when downstream is ready.
//
//  Saturation
//  ----------
//  Internal accumulator is wide enough to hold 3 * 2 * (2^(IQ_W-1))^2
//  without overflow.  Output is clipped (not wrapped) to OUT_W bits.
//  Default OUT_W=32 matches a widened cell_t in the CFAR IP (use ap_uint<32>).
//  If you change cell_t in cfar_detector.h, change OUT_W to match.
//
//  Note on signedness
//  ------------------
//  I*I and Q*Q on signed inputs always yield non-negative results, so the
//  intermediate products are treated as unsigned of width 2*IQ_W bits.
//  Verilog "signed * signed" returns a signed result of width 2*IQ_W — the
//  sign bit is always 0 for squares, but be careful when widening.
// =============================================================================

`default_nettype none

module noncoh_integrator #(
    parameter IQ_W   = 16,                   // bits per I or Q component
    parameter OUT_W  = 32                    // output sum width (saturated)
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // ── 3 complex AXI-Stream inputs ───────────────────────────────────────
    //   tdata layout per channel: { I[IQ_W-1:0] , Q[IQ_W-1:0] }  (signed)
    input  wire [2*IQ_W-1:0]          s0_tdata,
    input  wire                       s0_tvalid,
    input  wire                       s0_tlast,
    output wire                       s0_tready,

    input  wire [2*IQ_W-1:0]          s1_tdata,
    input  wire                       s1_tvalid,
    input  wire                       s1_tlast,
    output wire                       s1_tready,

    input  wire [2*IQ_W-1:0]          s2_tdata,
    input  wire                       s2_tvalid,
    input  wire                       s2_tlast,
    output wire                       s2_tready,

    // ── Summed |X|^2 AXI-Stream output ────────────────────────────────────
    output reg  [OUT_W-1:0]           m_axis_tdata,
    output reg                        m_axis_tvalid,
    output reg                        m_axis_tlast,
    input  wire                       m_axis_tready
);

    // ── Internal widths ───────────────────────────────────────────────────
    localparam PROD_W = 2*IQ_W;              // I*I, Q*Q width
    localparam MAG_W  = PROD_W + 1;          // I^2 + Q^2 width
    localparam PAIR_W = MAG_W + 1;           // pairwise sum width
    localparam SUM_W  = PAIR_W + 1;          // 4-channel sum width

    // The output stage saturates a SUM_W-wide value down to OUT_W bits.
    localparam [SUM_W-1:0] SAT_MAX = { {(SUM_W-OUT_W){1'b0}}, {OUT_W{1'b1}} };

    // ── Synchronous gather: all 3 inputs must be valid simultaneously ─────
    wire all_valid = s0_tvalid & s1_tvalid & s2_tvalid;

    // Pipeline space tracking: a "bubble" travels through stages and is
    // accepted at the output only when m_axis_tready is high.  Because the
    // pipeline is 4 deep, we need a 4-bit shift register of "in-flight" flags
    // and accept new inputs only when there is space (output drains as fast
    // as it fills, except when downstream stalls).
    //
    // Stall behaviour: when m_axis_tready is low AND m_axis_tvalid is high,
    // the entire pipeline freezes.  We do NOT spill mid-pipeline registers
    // into a FIFO — instead, all valids latch in place.

    wire stall  = m_axis_tvalid & ~m_axis_tready;
    wire fire   = all_valid & ~stall;

    assign s0_tready = fire;
    assign s1_tready = fire;
    assign s2_tready = fire;

    // ── Stage 0 : capture ─────────────────────────────────────────────────
    reg                        s0v_q, s0l_q;
    reg signed [IQ_W-1:0]      i0_q, q0_q, i1_q, q1_q;
    reg signed [IQ_W-1:0]      i2_q, q2_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0v_q <= 1'b0;
            s0l_q <= 1'b0;
        end else if (!stall) begin
            s0v_q <= fire;
            s0l_q <= s0_tlast;     // tlast from channel 0 represents all 3
            i0_q  <= s0_tdata[2*IQ_W-1 -: IQ_W];
            q0_q  <= s0_tdata[IQ_W-1   -: IQ_W];
            i1_q  <= s1_tdata[2*IQ_W-1 -: IQ_W];
            q1_q  <= s1_tdata[IQ_W-1   -: IQ_W];
            i2_q  <= s2_tdata[2*IQ_W-1 -: IQ_W];
            q2_q  <= s2_tdata[IQ_W-1   -: IQ_W];
        end
    end

    // ── Stage 1 : squares  (6 DSP48 multiplies) ───────────────────────────
    reg                        s1v_q, s1l_q;
    reg [PROD_W-1:0]           ii0_q, qq0_q, ii1_q, qq1_q;
    reg [PROD_W-1:0]           ii2_q, qq2_q;

    // signed * signed → 2*IQ_W bits, top bit always 0 for squares
    wire signed [PROD_W-1:0] ii0_w = i0_q * i0_q;
    wire signed [PROD_W-1:0] qq0_w = q0_q * q0_q;
    wire signed [PROD_W-1:0] ii1_w = i1_q * i1_q;
    wire signed [PROD_W-1:0] qq1_w = q1_q * q1_q;
    wire signed [PROD_W-1:0] ii2_w = i2_q * i2_q;
    wire signed [PROD_W-1:0] qq2_w = q2_q * q2_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1v_q <= 1'b0;
            s1l_q <= 1'b0;
        end else if (!stall) begin
            s1v_q <= s0v_q;
            s1l_q <= s0l_q;
            ii0_q <= ii0_w[PROD_W-1:0];
            qq0_q <= qq0_w[PROD_W-1:0];
            ii1_q <= ii1_w[PROD_W-1:0];
            qq1_q <= qq1_w[PROD_W-1:0];
            ii2_q <= ii2_w[PROD_W-1:0];
            qq2_q <= qq2_w[PROD_W-1:0];
        end
    end

    // ── Stage 2 : per-channel magnitude² and pairwise sum ─────────────────
    //   pair_lo = |X0|^2 + |X1|^2 ;  pair_hi = |X2|^2  (third channel passes
    //   straight through to the final adder).
    reg                        s2v_q, s2l_q;
    reg [PAIR_W-1:0]           pair_lo_q, pair_hi_q;

    wire [MAG_W-1:0] mag0_w = ii0_q + qq0_q;
    wire [MAG_W-1:0] mag1_w = ii1_q + qq1_q;
    wire [MAG_W-1:0] mag2_w = ii2_q + qq2_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2v_q <= 1'b0;
            s2l_q <= 1'b0;
        end else if (!stall) begin
            s2v_q     <= s1v_q;
            s2l_q     <= s1l_q;
            pair_lo_q <= mag0_w + mag1_w;
            pair_hi_q <= {{(PAIR_W-MAG_W){1'b0}}, mag2_w};
        end
    end

    // ── Stage 3 : final sum and saturation ────────────────────────────────
    wire [SUM_W-1:0] sum_w = pair_lo_q + pair_hi_q;
    wire overflow_w = |sum_w[SUM_W-1:OUT_W];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tdata  <= {OUT_W{1'b0}};
        end else if (!stall) begin
            m_axis_tvalid <= s2v_q;
            m_axis_tlast  <= s2l_q;
            m_axis_tdata  <= overflow_w ? {OUT_W{1'b1}}
                                        : sum_w[OUT_W-1:0];
        end
        // else: hold values (stall)
    end

endmodule

`default_nettype wire
