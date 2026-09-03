// =============================================================================
//  03_iq_delay_align.v
//  Aligns the I (in-phase) channel with Q (Hilbert transform output).
//
//  Signal path context
//  -------------------
//  ADC → TDM-mux → BPF FIR (4-ch TDM) → [this branch point]
//                                        ├─ Hilbert FIR (4-ch TDM) → Q (m_axis)
//                                        └─ THIS MODULE (shift-reg delay) → I
//
//  Why the BPF latency does NOT matter here
//  ----------------------------------------
//  Both I and Q paths see the BPF first, so the BPF group-delay cancels
//  perfectly.  This module only needs to compensate for the ADDITIONAL
//  latency introduced by the Hilbert FIR, measured from the BPF output.
//
//  Input port:  bpf_tdm_data/valid  (BPF FIR m_axis output — NOT raw ADC)
//  Input port:  hilbert_data/valid  (Hilbert FIR m_axis output)
//
//  HILBERT_LATENCY selection
//  -------------------------
//  Use the value measured/verified for the exact generated FIR instance and
//  wrapper handshake. FIR Compiler GUI latency and generated C_LATENCY can use
//  different counting conventions, so wrappers should override this parameter
//  explicitly instead of relying on the legacy default below.
//
//
//  Parameters
//  ----------
//  DATA_W          : sample width in bits (match ADC / FIR data width)
//  HILBERT_LATENCY : Hilbert FIR latency in TDM clock cycles.
//                    Legacy default is kept for compatibility; wrappers should
//                    pass the verified generated-core alignment value.
// =============================================================================

`default_nettype none

module iq_delay_align #(
    parameter DATA_W          = 16,
    parameter HILBERT_LATENCY = 35     // legacy default; override in wrappers
)(
    input  wire             clk,        // TDM clock (48 MHz = 4 × fs)
    input  wire             rst_n,      // active-low, synchronous

    // ── BPF FIR m_axis output (4-ch TDM interleaved) ─────────────────────
    // THIS IS THE BPF OUTPUT — NOT the raw ADC / TDM-mux output.
    // The channel tag that accompanied the TDM mux must propagate alongside
    // so the downstream demux knows which channel each beat belongs to.
    input  wire [DATA_W-1:0] bpf_tdm_data,
    input  wire              bpf_tdm_valid,
    input  wire [1:0]        bpf_tdm_ch_id,  // channel tag from TDM mux (delayed by BPF)

    // ── Hilbert FIR m_axis output ─────────────────────────────────────────
    input  wire [DATA_W-1:0] hilbert_data,
    input  wire              hilbert_valid,

    // ── Time-aligned I/Q outputs ──────────────────────────────────────────
    // Both valid on the same TDM clock cycle.
    // ch_id output is the delayed version of bpf_tdm_ch_id.
    output wire [DATA_W-1:0] i_tdm_data,
    output wire              i_tdm_valid,
    output wire [1:0]        i_tdm_ch_id,    // channel tag aligned with i_tdm_data
    output wire [DATA_W-1:0] q_tdm_data,
    output wire              q_tdm_valid
);

    // ── Shift registers: delay BPF output by HILBERT_LATENCY TDM clocks ──
    // Includes channel tag so downstream demux still knows which channel.
    (* shreg_extract = "yes" *)
    reg [DATA_W-1:0] i_data_sr [0:HILBERT_LATENCY-1];
    (* shreg_extract = "yes" *)
    reg              i_vld_sr  [0:HILBERT_LATENCY-1];
    (* shreg_extract = "yes" *)
    reg [1:0]        i_chid_sr [0:HILBERT_LATENCY-1];

    integer j;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (j = 0; j < HILBERT_LATENCY; j = j + 1) begin
                i_data_sr[j]  <= {DATA_W{1'b0}};
                i_vld_sr[j]   <= 1'b0;
                i_chid_sr[j]  <= 2'b00;
            end
        end else begin
            i_data_sr[0]  <= bpf_tdm_data;
            i_vld_sr[0]   <= bpf_tdm_valid;
            i_chid_sr[0]  <= bpf_tdm_ch_id;
            for (j = 1; j < HILBERT_LATENCY; j = j + 1) begin
                i_data_sr[j]  <= i_data_sr[j-1];
                i_vld_sr[j]   <= i_vld_sr[j-1];
                i_chid_sr[j]  <= i_chid_sr[j-1];
            end
        end
    end

    assign i_tdm_data  = i_data_sr[HILBERT_LATENCY-1];
    assign i_tdm_valid = i_vld_sr[HILBERT_LATENCY-1];
    assign i_tdm_ch_id = i_chid_sr[HILBERT_LATENCY-1];

    // ── Q = Hilbert FIR output (passes straight through) ─────────────────
    assign q_tdm_data  = hilbert_data;
    assign q_tdm_valid = hilbert_valid;

    // ── Simulation-only alignment check ──────────────────────────────────
    // synthesis translate_off
    integer align_warning_count;
    always @(posedge clk) begin
        if (!rst_n) begin
            align_warning_count <= 0;
        end else if (!$isunknown(i_tdm_valid) &&
                     !$isunknown(q_tdm_valid) &&
                     (i_tdm_valid !== q_tdm_valid)) begin
            if (align_warning_count < 8) begin
                $display("[iq_delay_align] WARNING t=%0t: I valid=%b Q valid=%b - HILBERT_LATENCY wrong?",
                         $time, i_tdm_valid, q_tdm_valid);
            end
            align_warning_count <= align_warning_count + 1;
        end
    end
    // synthesis translate_on

endmodule

`default_nettype wire
