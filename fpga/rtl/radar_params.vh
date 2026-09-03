// =============================================================================
//  radar_params.vh — single source of truth for radar pipeline dimensions
//  Every RTL stage and testbench includes this file; it is the only place
//  the frame geometry and the DDR address map are defined.
//
//  RADAR_RX_COUNT is 3, so the per-RX replicated buffers (range-FFT A
//  region and per-RX complex B region) are 3 * 128 KiB = 384 KiB instead of
//  512 KiB.  The 1 MiB B/C/D set stride is unchanged, so all base addresses
//  below stay valid with extra headroom.
//
//  MEMORY LAYOUT SUMMARY
//  ---------------------
//  The radar pipeline uses two different buffering ideas:
//
//    A region ping/pong:
//      protects the transposed 3-RX range-FFT scratch buffer used before
//      Doppler processing.
//
//    B/C/D frame-set ping/pong:
//      protects the complete post-Doppler products consumed by MicroBlaze:
//        B = per-RX complex range-Doppler maps
//        C = summed noncoherent power map, also the CFAR input image
//        D = packed CFAR detection bitmap
//      B/C/D must be switched together.  Software must never read B from one
//      frame, C from another frame, and D from a third frame.
//
//  One B/C/D frame set is:
//      B: 3 RX * 128 range bins * 256 Doppler bins * 4 bytes = 384 KiB
//      C:        128 range bins * 256 Doppler bins * 4 bytes = 128 KiB
//      D:        128 range bins * 256 Doppler bins * 1 bit  =   4 KiB
//
//  This file spaces B/C/D sets on a 1 MiB stride.  The extra gap is deliberate:
//  it keeps each large writer base aligned, avoids accidental overlap while
//  bringing up Vivado address maps, and makes frame-set selection simple.
//
//  Address map used here:
//      0x0A00_0000..0x0A05_FFFF  A0 range-FFT ping, 3 RX, 384 KiB
//      0x0A08_0000..0x0A0D_FFFF  A1 range-FFT pong, 3 RX, 384 KiB
//
//      0x0A10_0000..0x0A15_FFFF  B0 complex RD map, 3 RX, 384 KiB
//      0x0A18_0000..0x0A19_FFFF  C0 summed RD power map, 128 KiB
//      0x0A1A_0000..0x0A1A_0FFF  D0 packed det_map, 4 KiB
//      0x0A1A_1000..0x0A1F_FFFF  reserved/alignment gap
//
//      0x0A20_0000..0x0A25_FFFF  B1 complex RD map, 3 RX, 384 KiB
//      0x0A28_0000..0x0A29_FFFF  C1 summed RD power map, 128 KiB
//      0x0A2A_0000..0x0A2A_0FFF  D1 packed det_map, 4 KiB
//      0x0A2A_1000..0x0A2A_FFFF  reserved/alignment gap
//
//      0x0A2B_0000..             E compact MicroBlaze result/peak rings
//
//  Top-level ownership rule:
//      PL writes exactly one B/C/D set for a frame.
//      PL publishes the set only after B, C, and D are complete.
//      MicroBlaze invalidates cache and reads that same set.
//      MicroBlaze releases the set before PL reuses it.
// =============================================================================

`ifndef RADAR_PARAMS_VH
`define RADAR_PARAMS_VH

// ── Range × Doppler grid ─────────────────────────────────────────────────────
// Range bins  = samples per chirp / 2 (after range FFT)
// Doppler bins= chirps per frame
`define RADAR_RANGE_BINS   128
`define RADAR_DOPPLER_BINS 256
`define RADAR_N_CELLS      (`RADAR_RANGE_BINS * `RADAR_DOPPLER_BINS)

// Alias for legacy code that uses "N_CHIRPS" to mean Doppler bins
`define RADAR_N_CHIRPS     `RADAR_DOPPLER_BINS

// ── DDR3 layout — bytes per cell ────────────────────────────────────────────
`define RADAR_SAMPLE_BYTES 4
`define RADAR_RD_MAP_BYTES (`RADAR_N_CELLS * `RADAR_SAMPLE_BYTES)
`define RADAR_DET_MAP_WORDS ((`RADAR_N_CELLS + 31) / 32)
`define RADAR_DET_MAP_BYTES (`RADAR_DET_MAP_WORDS * 4)
`define RADAR_RX_COUNT 3
`define RADAR_RANGE_FFT_BUF_BYTES (`RADAR_RX_COUNT * `RADAR_RD_MAP_BYTES)
`define RADAR_RX_RDMAP_CACHE_BYTES (`RADAR_RX_COUNT * `RADAR_RD_MAP_BYTES)
`define RADAR_BCD_SET_STRIDE_BYTES 32'h0010_0000

// ── DDR3 base addresses ──────────────────────────────────────────────────────
// A: upstream range-FFT scratch ping/pong.  These are not the post-CFAR
//    software-consumed maps.  They protect the earlier Doppler-input staging.
`define RADAR_DDR_A_PING   32'h0A00_0000   // 384 KiB — range-FFT ping, 3 RX
`define RADAR_DDR_A_PONG   32'h0A08_0000   // 384 KiB — range-FFT pong, 3 RX

// B/C/D set 0: complete post-Doppler frame bundle for MicroBlaze consumption.
`define RADAR_DDR_B0_BASE  32'h0A10_0000   // 384 KiB — per-RX complex RD maps
`define RADAR_DDR_C0_BASE  32'h0A18_0000   // 128 KiB — Σ|X|² RD map, CFAR input
`define RADAR_DDR_D0_BASE  32'h0A1A_0000   //   4 KiB — packed det_map, CFAR output

// B/C/D set 1: second complete frame bundle for 40 FPS ping-pong operation.
`define RADAR_DDR_B1_BASE  32'h0A20_0000   // 384 KiB — per-RX complex RD maps
`define RADAR_DDR_C1_BASE  32'h0A28_0000   // 128 KiB — Σ|X|² RD map, CFAR input
`define RADAR_DDR_D1_BASE  32'h0A2A_0000   //   4 KiB — packed det_map, CFAR output

// Legacy aliases keep older module defaults compiling.  New code should prefer
// B0/B1, C0/C1, D0/D1 and drive frame_buf_sel explicitly.
`define RADAR_DDR_B_BASE   `RADAR_DDR_B0_BASE
`define RADAR_DDR_C_BASE   `RADAR_DDR_C0_BASE
`define RADAR_DDR_D_BASE   `RADAR_DDR_D0_BASE

// E: compact output region.  It is intentionally after both B/C/D sets, because
// the original 0x0A1B_0000 result base would sit inside the new set-0-to-set-1
// alignment gap and would be easy to collide with during address-map changes.
`define RADAR_DDR_E_BASE   32'h0A2B_0000
`define RADAR_RESULT_RECORD_BYTES 64
`define RADAR_RESULT_RING_SLOTS 1024
`define RADAR_RESULT_RING_BYTES (`RADAR_RESULT_RECORD_BYTES * `RADAR_RESULT_RING_SLOTS)
`define RADAR_DDR_E_PEAK_BASE (`RADAR_DDR_E_BASE + `RADAR_RESULT_RING_BYTES)
`define RADAR_PEAK_LIST_MAX 32
`define RADAR_PEAK_RECORD_WORDS 8
`define RADAR_PEAK_FRAME_WORDS 512
`define RADAR_PEAK_RING_SLOTS 64
`define RADAR_PEAK_RING_BYTES (`RADAR_PEAK_FRAME_WORDS * 4 * `RADAR_PEAK_RING_SLOTS)
`define RADAR_DDR_E_BYTES (`RADAR_RESULT_RING_BYTES + `RADAR_PEAK_RING_BYTES)

// ── Cell layout convention ───────────────────────────────────────────────────
// linear_cell = range_bin * DOPPLER_BINS + doppler_bin
// This matches det_map[range_bin][doppler_bin] in row-major C.
// Packed det_map convention:
//     word_index = linear_cell >> 5
//     bit_index  = linear_cell[4:0]
//     bit value  = 1 if CFAR accepted that cell, else 0

`endif // RADAR_PARAMS_VH
