#ifndef FUSION_CONTROL_H
#define FUSION_CONTROL_H
/*
 * fusion_control.h  —  radar_3rx SIMPLE software trigger.
 *
 * Scope for this study: NO camera / computer-vision fusion, and NO hardware
 * peak tracker.  Everything is software:
 *
 *   1. Hear a ball-club impact (AUDIO_LABEL_CLUB) -> ARM a short window.
 *   2. For each completed radar frame, scan the region-C summed |X|^2 map that
 *      the pipeline already wrote to DDR.  From it compute, in software:
 *          peak       = max cell power
 *          mean       = average cell power (noise-floor estimate)
 *          snr        = peak / mean
 *          peak_vel   = velocity of the peak's Doppler bin
 *   3. The FIRST frame (while armed) with |peak_vel| > 50 mph AND snr over the
 *      threshold (~30 dB) requests that frame's A/B/C/D dump to the host AND
 *      asserts `halt` (early stop) so the 2-deep ping-pong does not overwrite
 *      the chosen frame while it streams.
 *
 * Because there are only two ping-pong buffer sets, "remember the best frame
 * across the trajectory" is impossible (it gets overwritten); first-over-SNR +
 * early-stop is the correct, simplest proof-of-concept.
 *
 * Host-testable: compile with -DTRIG_TESTBENCH.
 */

#include "audio_dsp.h"
#include <stdint.h>

/* ── Radar map geometry (matches radar_params.vh) ──────────────────────── */
#define RADAR_RANGE_BINS    128u
#define RADAR_DOPPLER_BINS  256u
#define RADAR_N_CELLS       (RADAR_RANGE_BINS * RADAR_DOPPLER_BINS)
#define RADAR_DOPPLER_CTR   (RADAR_DOPPLER_BINS / 2u)   /* zero-velocity bin */

/* ── Tunables (placeholders for the PoC) ───────────────────────────────── */
enum {
    /* 50 mph = 2235 cm/s radial-speed gate. */
    TRIG_SPEED_CMS        = 2235,

    /* cm/s per Doppler bin.  Placeholder — set from the real chirp/PRF later. */
    TRIG_VEL_RES_CMS      = 55,

    /* SNR gate as a linear POWER ratio.  ~30 dB = 10^3 = 1000.  peak >= snr*mean */
    TRIG_SNR_RATIO        = 1000,

    /* Arm window after impact, and cooldown after a dump (ms). */
    TRIG_ARM_WINDOW_MS    = 400,
    TRIG_COOLDOWN_MS      = 1500
};

/* ── FSM state ─────────────────────────────────────────────────────────── */
typedef enum {
    TRIG_IDLE     = 0,   /* waiting for a ball-impact sound                  */
    TRIG_ARMED    = 1,   /* impact heard; scanning frames                    */
    TRIG_COOLDOWN = 2    /* dump requested; halted, ignore further frames    */
} trig_state_t;

/* Which DDR buffer set a completed frame lives in (software reads STATUS regs).
 * Only B/C/D are dumped to the host; region A (range-FFT scratch) is redundant
 * with B and is not transferred, so there is no A buffer select. */
typedef struct {
    uint8_t  frame_buf_sel;  /* B/C/D set 0/1 */
    uint32_t frame_id;       /* tag echoed to host */
} radar_frame_meta_t;

/* What software writes into radar_dump_axi_regs when a frame qualifies. */
typedef struct {
    uint8_t  dump_request;   /* 1 => write DUMP_SEL/FID then CTRL.start_dump|set_halt */
    uint8_t  halt;           /* 1 => freeze the radar pipeline (early stop)           */
    uint8_t  frame_buf_sel;
    uint32_t frame_id;
    /* Diagnostics from the scan (handy for logging / threshold tuning). */
    uint32_t peak;
    uint32_t mean;
    int32_t  peak_vel_cms;
    trig_state_t state;
} trig_output_t;

/* ── Public API ────────────────────────────────────────────────────────── */
void          trig_reset(void);
trig_state_t  trig_state(void);

/* Audio classifier output; CLUB impact arms the trigger. */
trig_output_t trig_submit_audio(audio_label_t label, uint32_t now_ms);

/* Scan one completed frame's region-C map (RADAR_N_CELLS uint32 cells) and
 * decide.  `cmap` points at C0 or C1 in DDR per meta->frame_buf_sel. */
trig_output_t trig_submit_frame(const uint32_t *cmap,
                                const radar_frame_meta_t *meta,
                                uint32_t now_ms);

trig_output_t trig_last_output(void);

#endif /* FUSION_CONTROL_H */
