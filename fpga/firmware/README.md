# `fpga/firmware/` — MicroBlaze C

Two modules. `audio_dsp` decides *that* a ball was struck; `fusion_control`
decides *which* radar frame to keep and ship. Both are written so the logic
compiles and runs on a host PC with no Xilinx BSP in sight — the board-specific
layer is behind `#ifdef`.

| File | Role |
|---|---|
| `audio_dsp.c` / `.h` | INMP441 impact detector |
| `fusion_control.c` / `.h` | Audio + speed trigger FSM and frame selection |

## `audio_dsp`

INMP441 → AXI I2S receiver → AXI DMA (simple S2MM, no scatter-gather) → a pair
of ping-pong buffers. From there, per hop:

```
hpf_process() → check_onset() → extract_features() → classify()
```

Features are peak frequency and spectral centroid (both approximated from
zero-crossing rate), ZCR itself, decay time constant, and crest factor; an LDA
classifier turns them into `CLUB` / `WALL` / `OTHER` / `NONE`. Labels are
forwarded to `fusion_control_submit_audio()`. This module never resets the
trigger FSM — that is `fusion_control`'s alone.

Three compilation tiers:

| Define | Compiles |
|---|---|
| *(none)* | The DSP core only — portable C, no board dependencies |
| `MICROBLAZE_AUDIO_MAIN` | The hardware layer: DMA init, the FreeRTOS task, `audio_main()` |
| `TESTBENCH` | Introspection hooks (`audio_dsp_feat_win_size`, `audio_dsp_hpf_gain`) for host-side checks of the filter and framing constants |

## `fusion_control` — work in progress

The trigger FSM: `IDLE → ARMED → COOLDOWN`.

1. A `CLUB` label from `audio_dsp` arms a short window.
2. For each completed radar frame, scan the region-C summed |X|² map already in
   DDR for peak, mean, SNR (`peak/mean`) and the peak's Doppler velocity. All in
   software — there is no hardware peak tracker.
3. The **first** armed frame that clears both the speed gate and the SNR gate
   requests that frame's dump and asserts `halt`, so the 2-deep ping-pong cannot
   overwrite it mid-stream.

With only two buffer sets, keeping the best frame across a whole trajectory is
not possible — it gets overwritten. First-over-threshold with an early stop is
the policy that fits the memory budget.

The FSM logic and its host tests are done:

```bash
gcc -O2 -DTRIG_TESTBENCH fusion_control.c -lm -o trig_tb && ./trig_tb
# RESULT: PASS
```

**What is still outstanding:**

- The gates in `fusion_control.h` are placeholders. `TRIG_VEL_RES_CMS`
  (cm/s per Doppler bin) is stubbed at 55 and must be derived from the real
  chirp rate and PRF; `TRIG_SPEED_CMS` and the SNR ratio are first guesses.
- The region-C scan is not yet wired to `radar_dump_axi_regs.v`, the AXI-Lite
  block that actually pulses `start` on the frame dump streamer.
- No camera / computer-vision fusion, despite the module's name — the current
  scope is audio plus radar speed only.
