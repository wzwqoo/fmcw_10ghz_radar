/*
 * fusion_control.c  —  radar_3rx SIMPLE software trigger.
 *
 * See fusion_control.h.  All peak / mean / SNR / peak-velocity work is done
 * here in software by scanning the region-C summed |X|^2 map already in DDR;
 * there is no hardware peak tracker.  The first frame (while armed) that clears
 * the speed AND SNR gates requests a dump and asserts halt (early stop).
 */

#include "fusion_control.h"

/* ── Module state ───────────────────────────────────────────────────────── */
static trig_state_t  g_state  = TRIG_IDLE;
static uint32_t      g_arm_ms = 0;
static trig_output_t g_last;

static int32_t  trig_abs32(int32_t x) { return x < 0 ? -x : x; }
static uint32_t elapsed_ms(uint32_t now, uint32_t since) { return (uint32_t)(now - since); }

static trig_output_t blank_output(void)
{
    trig_output_t o;
    o.dump_request  = 0u;
    o.halt          = (g_state == TRIG_COOLDOWN) ? 1u : 0u;
    o.frame_buf_sel = 0u;
    o.frame_id      = 0u;
    o.peak          = 0u;
    o.mean          = 0u;
    o.peak_vel_cms  = 0;
    o.state         = g_state;
    return o;
}

void trig_reset(void)
{
    g_state  = TRIG_IDLE;
    g_arm_ms = 0u;
    g_last   = blank_output();
}

trig_state_t  trig_state(void)       { return g_state; }
trig_output_t trig_last_output(void) { return g_last;  }

/* Timed transitions: cooldown end, arm-window expiry. */
static void tick(uint32_t now_ms)
{
    if (g_state == TRIG_COOLDOWN &&
        elapsed_ms(now_ms, g_arm_ms) >= (uint32_t)TRIG_COOLDOWN_MS) {
        g_state = TRIG_IDLE;
    } else if (g_state == TRIG_ARMED &&
               elapsed_ms(now_ms, g_arm_ms) >= (uint32_t)TRIG_ARM_WINDOW_MS) {
        g_state = TRIG_IDLE;
    }
}

trig_output_t trig_submit_audio(audio_label_t label, uint32_t now_ms)
{
    tick(now_ms);
    if (label == AUDIO_LABEL_CLUB && g_state != TRIG_COOLDOWN) {
        g_state  = TRIG_ARMED;
        g_arm_ms = now_ms;
    }
    g_last = blank_output();
    return g_last;
}

/* Scan the C map: peak value, its Doppler bin, and the running sum (for mean). */
static void scan_cmap(const uint32_t *cmap,
                      uint32_t *out_peak, uint32_t *out_peak_dop, uint32_t *out_mean)
{
    uint32_t peak = 0u, peak_dop = 0u;
    uint64_t sum  = 0u;
    uint32_t r, d, idx = 0u;

    for (r = 0u; r < RADAR_RANGE_BINS; ++r) {
        for (d = 0u; d < RADAR_DOPPLER_BINS; ++d, ++idx) {
            const uint32_t v = cmap[idx];
            sum += v;
            if (v > peak) { peak = v; peak_dop = d; }
        }
    }
    *out_peak     = peak;
    *out_peak_dop = peak_dop;
    *out_mean     = (uint32_t)(sum / (uint64_t)RADAR_N_CELLS);
}

trig_output_t trig_submit_frame(const uint32_t *cmap,
                                const radar_frame_meta_t *meta,
                                uint32_t now_ms)
{
    trig_output_t o;
    uint32_t peak, peak_dop, mean;
    int32_t  vel_cms;
    int      snr_pass, speed_pass;

    tick(now_ms);
    o = blank_output();
    if (cmap == 0 || meta == 0) { g_last = o; return o; }

    scan_cmap(cmap, &peak, &peak_dop, &mean);

    /* Velocity of the peak's Doppler bin (signed about the zero-velocity bin). */
    vel_cms = ((int32_t)peak_dop - (int32_t)RADAR_DOPPLER_CTR) * (int32_t)TRIG_VEL_RES_CMS;

    /* SNR gate as an integer power ratio: peak >= TRIG_SNR_RATIO * mean.
     * (mean==0 means an empty map; require a real peak then.) */
    snr_pass   = (mean == 0u) ? (peak > 0u)
                              : ((uint64_t)peak >= (uint64_t)TRIG_SNR_RATIO * (uint64_t)mean);
    speed_pass = trig_abs32(vel_cms) > (int32_t)TRIG_SPEED_CMS;

    o.peak         = peak;
    o.mean         = mean;
    o.peak_vel_cms = vel_cms;

    if (g_state == TRIG_ARMED && speed_pass && snr_pass) {
        o.dump_request  = 1u;
        o.halt          = 1u;                 /* early stop: freeze the pipeline */
        o.frame_buf_sel = meta->frame_buf_sel;
        o.frame_id      = meta->frame_id;
        g_state  = TRIG_COOLDOWN;
        g_arm_ms = now_ms;
    }

    o.state = g_state;
    g_last  = o;
    return o;
}

/* ── Host testbench ─────────────────────────────────────────────────────── */
#ifdef TRIG_TESTBENCH
#include <stdio.h>
#include <stdlib.h>

static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); g_fail = 1; } } while (0)

static uint32_t g_map[RADAR_N_CELLS];

/* Fill the map with a flat noise floor, then plant one strong cell. */
static void make_map(uint32_t noise, uint32_t peak, uint32_t range_bin, uint32_t dop_bin)
{
    uint32_t i;
    for (i = 0u; i < RADAR_N_CELLS; ++i) g_map[i] = noise;
    g_map[range_bin * RADAR_DOPPLER_BINS + dop_bin] = peak;
}

int main(void)
{
    trig_output_t o;
    radar_frame_meta_t meta = { 1u, 0xBEEF };

    /* Doppler bin 200 -> (200-128)*55 = 3960 cm/s ~= 88 mph (fast). */
    /* bin 140 -> (140-128)*55 = 660 cm/s ~= 15 mph (slow).          */

    /* 1) No impact: strong+fast frame, but not armed -> no dump. */
    trig_reset();
    make_map(10u, 5000000u, 64u, 200u);          /* SNR huge, fast */
    o = trig_submit_frame(g_map, &meta, 100u);
    CHECK(o.dump_request == 0 && o.state == TRIG_IDLE, "no impact => no dump");

    /* 2) Impact then strong+fast frame -> dump that frame + halt. */
    trig_reset();
    (void)trig_submit_audio(AUDIO_LABEL_CLUB, 1000u);
    CHECK(trig_state() == TRIG_ARMED, "club impact arms");
    make_map(10u, 5000000u, 64u, 200u);
    o = trig_submit_frame(g_map, &meta, 1100u);
    CHECK(o.dump_request == 1 && o.halt == 1, "fast+strong armed => dump+halt");
    CHECK(o.frame_buf_sel == 1 && o.frame_id == 0xBEEF, "dump carries meta");
    CHECK(o.peak_vel_cms == (200 - 128) * 55, "reported peak velocity");
    CHECK(o.state == TRIG_COOLDOWN, "cooldown after dump");

    /* 3) Second qualifying frame during cooldown -> ignored. */
    make_map(10u, 5000000u, 64u, 200u);
    o = trig_submit_frame(g_map, &meta, 1150u);
    CHECK(o.dump_request == 0, "cooldown blocks second dump");

    /* 4) Armed, strong, but SLOW peak (bin 140) -> no dump. */
    trig_reset();
    (void)trig_submit_audio(AUDIO_LABEL_CLUB, 2000u);
    make_map(10u, 5000000u, 64u, 140u);
    o = trig_submit_frame(g_map, &meta, 2050u);
    CHECK(o.dump_request == 0, "slow peak => no dump");

    /* 5) Armed, fast, but LOW SNR (peak ~ noise*100, ratio<1000) -> no dump. */
    trig_reset();
    (void)trig_submit_audio(AUDIO_LABEL_CLUB, 3000u);
    make_map(1000u, 100000u, 64u, 200u);         /* peak/mean ~ 100 < 1000 */
    o = trig_submit_frame(g_map, &meta, 3050u);
    CHECK(o.dump_request == 0, "low SNR => no dump");

    /* 6) Window expiry: fast+strong but too late -> no dump. */
    trig_reset();
    (void)trig_submit_audio(AUDIO_LABEL_CLUB, 4000u);
    make_map(10u, 5000000u, 64u, 200u);
    o = trig_submit_frame(g_map, &meta, 4000u + TRIG_ARM_WINDOW_MS + 1u);
    CHECK(o.dump_request == 0 && o.state == TRIG_IDLE, "late frame => window expired");

    printf(g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return g_fail;
}
#endif /* TRIG_TESTBENCH */
