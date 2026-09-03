/*
 * audio_dsp.c  –  INMP441 / AXI-I2S impact detector.
 *
 * Structure:
 *   ┌─────────────────────────────────────────────────────┐
 *   │  Section 1: DSP core                                │
 *   │    hpf_process()  →  check_onset()  →              │
 *   │    extract_features()  →  classify()               │
 *   │    Public: audio_dsp_reset(), audio_dsp_push()      │
 *   ├─────────────────────────────────────────────────────┤
 *   │  Section 2: MicroBlaze hardware layer               │  ← #ifdef MICROBLAZE_AUDIO_MAIN
 *   │    Block design:                                    │
 *   │      INMP441 → AXI I2S Receiver                    │
 *   │        → AXI DMA (simple S2MM, no SG)              │
 *   │          → audio_dma_buf_a / _b  (on-chip / DDR)   │
 *   │    audio_hw_init()   — DMA init only                │
 *   │    audio_task()      — FreeRTOS task, never returns │
 *   │    audio_main()      — init + spawn/call task       │
 *   ├─────────────────────────────────────────────────────┤
 *   │  Section 3: Testbench hooks                         │  ← #ifdef TESTBENCH
 *   └─────────────────────────────────────────────────────┘
 *
 * Ownership note:
 *   This file calls fusion_control_submit_audio() to forward detected labels
 *   to the fusion FSM. It does NOT call fusion_control_reset(). Resetting
 *   the fusion FSM is exclusively fusion_control.c's responsibility.
 *
 * Bug fixes vs v1 (preserved here for traceability):
 *   BUG-1  HPF_ALPHA 0.90 → 0.5601  (correct for fc=2 kHz @ 16 kHz)
 *   BUG-2  onset_hold = ONSET_HOLD_HOPS*4 → ONSET_HOLD_HOPS (200 ms lockout)
 *   BUG-3  FEAT_WIN 240 → WINDOW_SAMPLES (480 = 30 ms)
 *   BUG-4  score < 0 → CLUB  →  score > 0 → CLUB
 */

#include "audio_dsp.h"
#include <math.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════
 * Section 1 — DSP core (always compiled)
 * ══════════════════════════════════════════════════════════════════════ */

/* ── Configuration ───────────────────────────────────────────────────── */
#define FS              16000u   /* sample rate, Hz              */
#define HOP_SAMPLES     80u      /* onset hop size = 5 ms        */
#define WINDOW_SAMPLES  480u     /* feature window   = 30 ms     */

/* HPF: 1st-order high-pass, fc = 2 kHz @ 16 kHz.
 *   alpha = tau / (tau + Ts)
 *   tau   = 1 / (2π × 2000) = 7.958e-5 s
 *   Ts    = 1 / 16000        = 6.250e-5 s
 *   alpha = 0.5601                            (BUG-1 FIXED) */
#define HPF_ALPHA       0.5601f

/* Onset lockout: 200 ms = 200 ms / 5 ms per hop = 40 hops. (BUG-2 FIXED) */
#define ONSET_HOLD_HOPS 40u

/* Feature window covers the full 30 ms circular buffer. (BUG-3 FIXED) */
#define FEAT_WIN        WINDOW_SAMPLES

/* Onset threshold: current STE must be 8× the previous hop's STE. */
#define ONSET_RATIO     8.0f

/* ── HPF state ───────────────────────────────────────────────────────── */
static float hpf_x_prev;
static float hpf_y_prev;

/* y[n] = alpha * (y[n-1] + x[n] - x[n-1]) */
static float hpf_process(float x)
{
    float y    = HPF_ALPHA * (hpf_y_prev + x - hpf_x_prev);
    hpf_x_prev = x;
    hpf_y_prev = y;
    return y;
}

/* ── STE onset detector ──────────────────────────────────────────────── */
static float prev_ste;
static int   onset_hold;

/* Returns 1 on onset, 0 otherwise. Enforces 200 ms lockout after onset. */
static int check_onset(float cur_ste)
{
    if (onset_hold > 0) {
        --onset_hold;
        return 0;
    }
    if (cur_ste > ONSET_RATIO * prev_ste) {
        onset_hold = (int)ONSET_HOLD_HOPS;   /* BUG-2 FIXED */
        return 1;
    }
    return 0;
}

/* ── Circular feature buffer ─────────────────────────────────────────── */
static float feat_buf[WINDOW_SAMPLES];
static int   feat_head;

/* ── Feature extraction ──────────────────────────────────────────────── */
static audio_features_t last_feats;

static void extract_features(void)
{
    float peak = 0.0f, rms_acc = 0.0f;
    int   peak_idx = 0, zc_total = 0;

    /* Pass 1: peak, RMS, global ZCR. */
    for (int i = 0; i < (int)FEAT_WIN; ++i) {
        int   idx = (feat_head - (int)FEAT_WIN + i + (int)WINDOW_SAMPLES)
                    % (int)WINDOW_SAMPLES;
        float s   = feat_buf[idx];
        rms_acc  += s * s;
        if (fabsf(s) > peak) { peak = fabsf(s); peak_idx = i; }
        if (i > 0) {
            int prev = (feat_head - (int)FEAT_WIN + i - 1 + (int)WINDOW_SAMPLES)
                       % (int)WINDOW_SAMPLES;
            if (feat_buf[prev] * s < 0.0f) ++zc_total;
        }
    }

    const float rms        = sqrtf(rms_acc / (float)FEAT_WIN);
    last_feats.crest_factor = (rms > 1e-9f) ? peak / rms : 0.0f;
    last_feats.zcr          = (float)zc_total / (float)FEAT_WIN;

    /* Decay tau: time from peak to 10% of peak. */
    const float thr         = peak * 0.1f;
    last_feats.decay_tau    = (float)FEAT_WIN / (float)FS;
    for (int i = peak_idx + 1; i < (int)FEAT_WIN; ++i) {
        int idx = (feat_head - (int)FEAT_WIN + i + (int)WINDOW_SAMPLES)
                  % (int)WINDOW_SAMPLES;
        if (fabsf(feat_buf[idx]) < thr) {
            last_feats.decay_tau = (float)(i - peak_idx) / (float)FS;
            break;
        }
    }

    /* Peak frequency via ZCR over 5 ms window around peak (80 samples). */
    int pzc = 0;
    for (int i = peak_idx; i < peak_idx + 79 && i < (int)FEAT_WIN - 1; ++i) {
        int ia = (feat_head - (int)FEAT_WIN + i     + (int)WINDOW_SAMPLES) % (int)WINDOW_SAMPLES;
        int ib = (feat_head - (int)FEAT_WIN + i + 1 + (int)WINDOW_SAMPLES) % (int)WINDOW_SAMPLES;
        if (feat_buf[ia] * feat_buf[ib] < 0.0f) ++pzc;
    }
    last_feats.peak_freq = (float)pzc * (float)FS / (2.0f * 80.0f);

    /* Spectral centroid: ZCR-weighted frequency centroid. */
    float num = 0.0f, den = 0.0f;
    for (int i = 1; i < (int)FEAT_WIN; ++i) {
        int   ia = (feat_head - (int)FEAT_WIN + i - 1 + (int)WINDOW_SAMPLES) % (int)WINDOW_SAMPLES;
        int   ib = (feat_head - (int)FEAT_WIN + i     + (int)WINDOW_SAMPLES) % (int)WINDOW_SAMPLES;
        float m  = fabsf(feat_buf[ib]);
        if (feat_buf[ia] * feat_buf[ib] < 0.0f) num += m * (float)i;
        den += m;
    }
    last_feats.spectral_centroid = (den > 1e-9f)
        ? (num / den) * ((float)FS / 2.0f) / (float)FEAT_WIN
        : 0.0f;
}

/* ── Impact classifier ───────────────────────────────────────────────── */
/*
 * Replace W[] and BIAS with values from your trained LDA model.
 * Feature order: [peak_freq, spectral_centroid, zcr, decay_tau, crest_factor]
 * club_score > 0 → CLUB. Low-frequency, slower-decay impacts are WALL.
 */
static const float CLUB_W[5] = { 0.00100f, -0.00030f, 20.0f, -20.0f, 0.30f };
static const float CLUB_BIAS = -3.0f;

#define WALL_MAX_PEAK_FREQ_HZ      2400.0f
#define WALL_MAX_CENTROID_HZ       1800.0f
#define WALL_MIN_DECAY_TAU_S       0.010f

static audio_label_t classify(void)
{
    const audio_features_t *f = &last_feats;
    const float club_score =
          CLUB_W[0] * f->peak_freq
        + CLUB_W[1] * f->spectral_centroid
        + CLUB_W[2] * f->zcr
        + CLUB_W[3] * f->decay_tau
        + CLUB_W[4] * f->crest_factor
        + CLUB_BIAS;

    if ((f->peak_freq <= WALL_MAX_PEAK_FREQ_HZ ||
         f->spectral_centroid <= WALL_MAX_CENTROID_HZ) &&
        f->decay_tau >= WALL_MIN_DECAY_TAU_S)
        return AUDIO_LABEL_WALL;

    if (club_score > 0.0f)
        return AUDIO_LABEL_CLUB;

    return AUDIO_LABEL_OTHER;
}

/* ── Hop accumulator ─────────────────────────────────────────────────── */
static float hop_acc;
static int   hop_n;

/* ── Public DSP API ──────────────────────────────────────────────────── */

void audio_dsp_reset(void)
{
    hpf_x_prev = 0.0f;
    hpf_y_prev = 0.0f;
    prev_ste   = 1e-12f;
    onset_hold = 0;
    feat_head  = 0;
    hop_acc    = 0.0f;
    hop_n      = 0;
    memset(feat_buf,    0, sizeof(feat_buf));
    memset(&last_feats, 0, sizeof(last_feats));
}

/*
 * Call once per I2S sample (float, normalised -1..1).
 * Returns AUDIO_LABEL_NONE most of the time.
 * Returns a non-NONE label at most once per ONSET_HOLD_HOPS × HOP_SAMPLES
 * samples (200 ms lockout).
 */
audio_label_t audio_dsp_push(float sample)
{
    const float s = hpf_process(sample);
    feat_buf[feat_head % (int)WINDOW_SAMPLES] = s;
    ++feat_head;

    hop_acc += s * s;
    ++hop_n;

    if (hop_n < (int)HOP_SAMPLES)
        return AUDIO_LABEL_NONE;

    const float cur_ste = hop_acc / (float)HOP_SAMPLES;
    hop_acc = 0.0f;
    hop_n   = 0;

    audio_label_t result = AUDIO_LABEL_NONE;
    if (check_onset(cur_ste) && feat_head >= (int)WINDOW_SAMPLES) {
        extract_features();
        result = classify();
    }

    prev_ste = cur_ste;
    return result;
}

const audio_features_t *audio_dsp_last_features(void)
{
    return &last_feats;
}

/* ══════════════════════════════════════════════════════════════════════
 * Section 2 — MicroBlaze hardware layer
 *             Compiled only with -DMICROBLAZE_AUDIO_MAIN
 * ══════════════════════════════════════════════════════════════════════
 *
 * Block design connection:
 *
 *   INMP441 mic (PDM)
 *     └─► AXI I2S Receiver IP  (Xilinx or custom)
 *           AXI4-Stream TX ──► AXI DMA (simple mode, S2MM channel only)
 *                                  │
 *                         S2MM writes 32-bit I2S words into
 *                         audio_dma_buf_a / audio_dma_buf_b
 *                         (256 words = 1 KB each, DDR or TCM)
 *                                  │
 *                         audio_task() reads finished buffer,
 *                         calls audio_dsp_push() per word,
 *                         forwards non-NONE labels to
 *                         fusion_control_submit_audio()
 *
 * I2S word format (Xilinx AXI I2S core default):
 *   Bits [31:8]  signed 24-bit sample, left-justified
 *   Bits [7:0]   zero-padded
 *   Override with -DAUDIO_I2S_SAMPLE_SHIFT=0 for right-justified.
 *
 * Double-buffer strategy:
 *   While processing block A, DMA is already filling block B.
 *   This hides DMA latency behind processing time.
 *   At 16 kHz × 256 samples = 16 ms per block — well above MicroBlaze
 *   processing time for 256 audio_dsp_push() calls.
 */
/* ══════════════════════════════════════════════════════════════════════ */

#ifdef MICROBLAZE_AUDIO_MAIN

#include "fusion_control.h"    /* fusion_control_submit_audio() only    */
#include "xaxidma.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"

#ifdef FUSION_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* ── DMA device ──────────────────────────────────────────────────────── */
#ifndef AUDIO_DMA_DEVICE_ID
# ifdef XPAR_AXIDMA_0_DEVICE_ID
#  define AUDIO_DMA_DEVICE_ID XPAR_AXIDMA_0_DEVICE_ID
# else
#  error "Set AUDIO_DMA_DEVICE_ID to your AXI DMA instance ID from xparameters.h"
# endif
#endif

/* Block size: 256 words = 256 I2S samples = 16 ms at 16 kHz. */
#ifndef AUDIO_DMA_WORDS_PER_BLOCK
# define AUDIO_DMA_WORDS_PER_BLOCK 256u
#endif

/* Poll timeout guard — prevents infinite spin if I2S clock is dead. */
#ifndef AUDIO_DMA_TIMEOUT_POLLS
# define AUDIO_DMA_TIMEOUT_POLLS 10000000u
#endif

/* I2S word → float conversion parameters.
 * Default: 24-bit sample left-justified in bits [31:8]. */
#ifndef AUDIO_I2S_SAMPLE_SHIFT
# define AUDIO_I2S_SAMPLE_SHIFT  8
#endif
#ifndef AUDIO_I2S_SAMPLE_SCALE
# define AUDIO_I2S_SAMPLE_SCALE  8388608.0f   /* 2^23 */
#endif

/* ── Module-private state ────────────────────────────────────────────── */
static XAxiDma   audio_dma;
static uint64_t  audio_sample_count;

/* Two DMA target buffers — one being filled by DMA while the other is
 * processed by audio_dsp_push(). Alignment required by AXI DMA IP. */
static u32 audio_dma_buf_a[AUDIO_DMA_WORDS_PER_BLOCK]
    __attribute__((aligned(64)));
static u32 audio_dma_buf_b[AUDIO_DMA_WORDS_PER_BLOCK]
    __attribute__((aligned(64)));

/* ── DMA helpers ─────────────────────────────────────────────────────── */

/*
 * audio_hw_init — configure AXI DMA for simple S2MM (device-to-memory).
 *
 * Requirements:
 *   - DMA must be configured in simple (non-SG) mode in Vivado block design.
 *   - Only S2MM channel is used — no MM2S needed.
 *   - Call once before audio_task().
 *   - Does NOT touch fusion_control state.
 */
int audio_hw_init(void)
{
    XAxiDma_Config *cfg = XAxiDma_LookupConfig((u32)AUDIO_DMA_DEVICE_ID);
    if (cfg == NULL) {
        xil_printf("audio: AXI DMA config not found (device id %d)\r\n",
                   AUDIO_DMA_DEVICE_ID);
        return XST_FAILURE;
    }

    int status = XAxiDma_CfgInitialize(&audio_dma, cfg);
    if (status != XST_SUCCESS) {
        xil_printf("audio: AXI DMA init failed %d\r\n", status);
        return status;
    }

    if (XAxiDma_HasSg(&audio_dma)) {
        xil_printf("audio: DMA is in SG mode — must be simple mode\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*
 * Kick DMA to fill buf with AUDIO_DMA_WORDS_PER_BLOCK words from I2S.
 * Returns immediately — does not wait for completion.
 * Call audio_dma_wait() after to block until the transfer is done.
 */
static int audio_dma_start(u32 *buf)
{
    const u32 bytes = (u32)AUDIO_DMA_WORDS_PER_BLOCK * sizeof(u32);
    Xil_DCacheFlushRange((UINTPTR)buf, bytes);
    return XAxiDma_SimpleTransfer(&audio_dma,
                                  (UINTPTR)buf,
                                  bytes,
                                  XAXIDMA_DEVICE_TO_DMA);
}

/*
 * Block until the active S2MM transfer completes.
 * Returns XST_SUCCESS or XST_FAILURE on timeout.
 */
static int audio_dma_wait(u32 *buf)
{
    const u32 bytes = (u32)AUDIO_DMA_WORDS_PER_BLOCK * sizeof(u32);
    u32 polls = 0u;

    while (XAxiDma_Busy(&audio_dma, XAXIDMA_DEVICE_TO_DMA)) {
        if (++polls >= (u32)AUDIO_DMA_TIMEOUT_POLLS) {
            xil_printf("audio: DMA timeout — I2S clock present?\r\n");
            return XST_FAILURE;
        }
    }

    Xil_DCacheInvalidateRange((UINTPTR)buf, bytes);
    return XST_SUCCESS;
}

/* ── Sample processing ───────────────────────────────────────────────── */

static float i2s_word_to_float(u32 word)
{
    const int32_t raw = (int32_t)word >> AUDIO_I2S_SAMPLE_SHIFT;
    return (float)raw / AUDIO_I2S_SAMPLE_SCALE;
}

static uint32_t audio_now_ms(void)
{
    return (uint32_t)((audio_sample_count * 1000ull) / (uint64_t)FS);
}

/*
 * Process one completed DMA block.
 * Calls audio_dsp_push() per sample; forwards non-NONE labels to
 * trig_submit_audio().  A ball-club impact only ARMS the radar-dump trigger;
 * the actual dump is decided later by trig_submit_frame() when a qualifying
 * radar frame arrives, so there is nothing to write to registers here.
 */
static void audio_process_block(const u32 *buf)
{
    for (u32 i = 0u; i < (u32)AUDIO_DMA_WORDS_PER_BLOCK; ++i) {
        const float         sample = i2s_word_to_float(buf[i]);
        const audio_label_t label  = audio_dsp_push(sample);
        ++audio_sample_count;

        if (label != AUDIO_LABEL_NONE) {
            (void)trig_submit_audio(label, audio_now_ms());
        }
    }
}

/* ── FreeRTOS / bare-metal task ──────────────────────────────────────── */

/*
 * audio_task — never returns.
 *
 * Double-buffer loop:
 *   1. Start DMA into buf_fill.
 *   2. Process buf_proc (previous block) while DMA runs.
 *   3. Wait for DMA to complete.
 *   4. Swap pointers, go to 1.
 *
 * First iteration: buf_proc is NULL so processing is skipped.
 */
void audio_task(void *arg)
{
    (void)arg;

    u32 *buf_fill = audio_dma_buf_a;
    u32 *buf_proc = NULL;

    /* Prime the pump: start first DMA transfer. */
    if (audio_dma_start(buf_fill) != XST_SUCCESS) {
        xil_printf("audio: first DMA start failed\r\n");
        return;
    }

    for (;;) {
        /* Wait for DMA filling buf_fill to complete. */
        if (audio_dma_wait(buf_fill) != XST_SUCCESS) {
            /* Timeout — try to recover by restarting the same buffer. */
            audio_dma_start(buf_fill);
            continue;
        }

        /* Swap: what was being filled is now ready to process. */
        u32 *ready = buf_fill;
        buf_fill   = (buf_fill == audio_dma_buf_a)
                     ? audio_dma_buf_b
                     : audio_dma_buf_a;

        /* Kick next DMA transfer immediately before processing. */
        if (audio_dma_start(buf_fill) != XST_SUCCESS) {
            xil_printf("audio: DMA start failed\r\n");
            continue;
        }

        /* Process the completed block while DMA fills the other buffer. */
        audio_process_block(ready);

#ifdef FUSION_USE_FREERTOS
        /* Yield at end of each block (16 ms) so fusion_task can run. */
        taskYIELD();
#endif
    }
}

/* ── Entry point ─────────────────────────────────────────────────────── */

/*
 * audio_main — call once from application main().
 *
 * Initialises AXI DMA, resets the DSP core, then:
 *   - With FUSION_USE_FREERTOS: creates audio_task as a FreeRTOS task.
 *   - Without: calls audio_task() directly (blocks forever).
 *
 * Does NOT call fusion_control_reset(). The fusion FSM is owned by
 * fusion_control.c — reset it there before calling audio_main().
 */
int audio_main(void)
{
    int status = audio_hw_init();
    if (status != XST_SUCCESS)
        return status;

    audio_dsp_reset();
    audio_sample_count = 0ull;
    xil_printf("audio: starting (block=%u words, %u ms)\r\n",
               AUDIO_DMA_WORDS_PER_BLOCK,
               (AUDIO_DMA_WORDS_PER_BLOCK * 1000u) / FS);

#ifdef FUSION_USE_FREERTOS
    {
        const BaseType_t rc = xTaskCreate(audio_task, "audio",
                                          configMINIMAL_STACK_SIZE + 256u,
                                          NULL, tskIDLE_PRIORITY + 3u, NULL);
        if (rc != pdPASS) {
            xil_printf("audio: xTaskCreate failed\r\n");
            return XST_FAILURE;
        }
        return XST_SUCCESS;   /* FreeRTOS scheduler starts elsewhere */
    }
#else
    audio_task(NULL);         /* bare-metal: never returns */
    return XST_SUCCESS;
#endif
}

#endif /* MICROBLAZE_AUDIO_MAIN */

/* ══════════════════════════════════════════════════════════════════════
 * Section 3 — Testbench hooks
 *             Compiled only with -DTESTBENCH
 * ══════════════════════════════════════════════════════════════════════ */

#ifdef TESTBENCH

int audio_dsp_feat_win_size(void) { return (int)FEAT_WIN; }

/*
 * Measure HPF power gain at freq_hz.
 * Runs a private simulation — does not disturb main DSP state.
 */
float audio_dsp_hpf_gain(float freq_hz)
{
    float y = 0.0f, xp = 0.0f;
    float sum_in = 0.0f, sum_out = 0.0f;
    const int   N  = (int)FS * 2;
    const float dp = 2.0f * 3.14159265f * freq_hz / (float)FS;

    for (int i = 0; i < N; ++i) {
        const float x  = sinf(dp * (float)i);
        const float yy = HPF_ALPHA * (y + x - xp);
        xp = x; y = yy;
        if (i > N / 2) { sum_in += x * x; sum_out += yy * yy; }
    }
    return (sum_in > 0.0f) ? sqrtf(sum_out / sum_in) : 0.0f;
}

#endif /* TESTBENCH */
