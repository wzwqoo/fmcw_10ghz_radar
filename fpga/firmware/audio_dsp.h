#ifndef AUDIO_DSP_H
#define AUDIO_DSP_H
/*
 * audio_dsp.h  –  INMP441 / AXI-I2S impact detector, pure-DSP public API.
 *
 * Responsibilities of this module:
 *   - HPF → STE onset detection → feature extraction → LDA classify
 *   - Returns audio_label_t per sample, caller decides what to do with it
 *
 * This header is intentionally free of Xilinx BSP types so the DSP core
 * compiles on the host testbench without any board dependencies.
 *
 * Hardware layer (AXI DMA + I2S) is compiled only with -DMICROBLAZE_AUDIO_MAIN.
 * Fusion policy is NOT touched here; fusion_control_submit_audio() is called
 * by the hardware layer in audio_dsp.c, not by this DSP core.
 *
 * Testbench: -DTESTBENCH exposes the introspection hooks below
 * (audio_dsp_feat_win_size, audio_dsp_hpf_gain) so a host driver can check the
 * filter and framing constants without any board dependencies.
 */

#include <stdint.h>

/* ── Labels ───────────────────────────────────────────────────────────── */
typedef enum {
    AUDIO_LABEL_NONE  = -1,   /* no event this sample — do not act         */
    AUDIO_LABEL_OTHER =  0,   /* impact detected, not club or wall         */
    AUDIO_LABEL_CLUB  =  1,   /* club-ball impact                          */
    AUDIO_LABEL_WALL  =  2    /* ball-wall impact                          */
} audio_label_t;

/* ── Features (visible to caller for logging / retraining) ───────────── */
typedef struct {
    float peak_freq;          /* Hz  — dominant freq via ZCR              */
    float spectral_centroid;  /* Hz  — ZCR-weighted centroid approx       */
    float zcr;                /* 0..1 zero-crossing rate                  */
    float decay_tau;          /* s   — time from peak to 10% of peak      */
    float crest_factor;       /* peak / RMS                               */
} audio_features_t;

/* ── Core DSP API (always compiled) ──────────────────────────────────── */
void                    audio_dsp_reset(void);
audio_label_t           audio_dsp_push(float sample);  /* call per I2S sample */
const audio_features_t *audio_dsp_last_features(void); /* valid after onset    */

/* ── Hardware entry points (compiled only with -DMICROBLAZE_AUDIO_MAIN) ─
 *
 * audio_hw_init() — configures AXI DMA (simple S2MM mode).
 * audio_task()    — FreeRTOS task body; double-buffers DMA blocks,
 *                   calls audio_dsp_push(), forwards labels to
 *                   fusion_control_submit_audio(). Never returns.
 * audio_main()    — one-shot entry: inits HW, resets DSP, then either
 *                   spawns audio_task as a FreeRTOS task (FUSION_USE_FREERTOS)
 *                   or calls audio_task directly (bare-metal).
 */
#ifdef MICROBLAZE_AUDIO_MAIN
int  audio_hw_init(void);
void audio_task(void *arg);
int  audio_main(void);
#endif

/* ── Testbench hooks (compiled only with -DTESTBENCH) ────────────────── */
#ifdef TESTBENCH
#define EXPECTED_FEAT_WIN         480
#define EXPECTED_ONSET_HOLD_HOPS   40

int   audio_dsp_feat_win_size(void);
float audio_dsp_hpf_gain(float freq_hz);  /* power gain at freq without side-effects */
#endif

#endif /* AUDIO_DSP_H */