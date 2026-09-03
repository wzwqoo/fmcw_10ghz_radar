`timescale 1ns/1ps
// ============================================================================
// radar_frame_sequencer.v
//
// The small top-level glue that (a) tells everyone which B/C/D set the current
// frame writes, (b) detects when a whole B/C/D bundle is complete, and (c)
// double-buffers by toggling the set between frames.  It is the piece that was
// "missing" between the writers/CFAR and radar_dump_axi_regs.
//
// Wiring:
//   frame_buf_sel  -> cplx_cell_cache_top.frame_buf_sel  (B select)
//                  -> rd_map_collector_summed.frame_buf_sel (C select; the
//                     collector forwards its latched value to CFAR as
//                     cfar_frame_buf_sel, so D follows automatically)
//   cache_ready    <- cplx_cell_cache_top.cache_ready   (all 3 B writers done)
//   frame_complete <- rd_map_collector_summed.frame_complete (C written + CFAR done)
//   cfar_frame_buf_sel <- rd_map_collector_summed.cfar_frame_buf_sel (completed set)
//   halt           <- radar_dump_axi_regs.halt          (freeze during a dump)
//   bundle_ready   -> radar_dump_axi_regs.frame_complete (a full B/C/D set is ready)
//   frame_set      -> radar_dump_axi_regs.frame_set      (which set just completed)
//
// Cadence: frame_complete (post-CFAR) is the LATER of the two completion pulses
// (B/cache finishes before CFAR), so it marks the whole bundle done; cache_ready
// is tracked sticky only as a defensive cross-check.  On bundle completion the
// set is toggled for the next frame UNLESS halt is asserted (a dump is freezing
// the pipeline), so the dumped set is not reused underneath the transfer.
//
// `halt` must ALSO gate wherever a new acquisition/frame starts (e.g. the ADC
// conv_start or the range-FFT/scatter frame-start) — this block only stops the
// set from advancing; it does not itself stop data flow.
// ============================================================================
`default_nettype none

module radar_frame_sequencer (
    input  wire clk,
    input  wire rst_n,

    // From the pipeline
    input  wire cache_ready,        // pulse: all B writers done
    input  wire frame_complete,     // pulse: C written + CFAR (D) done
    input  wire cfar_frame_buf_sel, // latched set of the completed frame

    // From the dump register block
    input  wire halt,               // hold the set while a dump is in flight

    // To the pipeline
    output reg  frame_buf_sel,      // B/C/D set select for the CURRENT frame

    // To the dump register block
    output reg  bundle_ready,       // pulse: a full B/C/D set is complete
    output reg  frame_set           // which set that completed bundle is in
);
    reg cache_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_buf_sel <= 1'b0;
            bundle_ready  <= 1'b0;
            frame_set     <= 1'b0;
            cache_seen    <= 1'b0;
        end else begin
            bundle_ready <= 1'b0;             // default: pulse low

            if (cache_ready) cache_seen <= 1'b1;

            if (frame_complete) begin
                // Whole bundle done: B (cache_seen / this cycle) + C + D.
                bundle_ready <= 1'b1;
                frame_set    <= cfar_frame_buf_sel;  // report the completed set
                cache_seen   <= 1'b0;

                // Advance to the other buffer for the next frame, unless a dump
                // is halting the pipeline (keep the dumped set intact).
                if (!halt)
                    frame_buf_sel <= ~frame_buf_sel;
            end
        end
    end

endmodule

`default_nettype wire
