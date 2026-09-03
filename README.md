# 10 GHz FMCW Radar Sensing Platform

A single-board **1TX / 3RX, 10 GHz FMCW radar**, its **Artix-7 range–Doppler–CFAR
backend**, and the **carrier board** that mounts the radar alongside a
global-shutter camera and an impact microphone.

The target application is a golf launch monitor: recover a ball's speed, launch
direction, spin rate and spin axis from one radar and one camera. That
application drives every architectural choice here — three non-collinear
receivers (the minimum that gives a radar-native spin axis), per-receiver
*complex* range–Doppler maps published to DDR3 (the phase the spin-axis
interferometer consumes), and a once-per-shot Ethernet offload so the heavy
post-processing runs on a host PC instead of in fabric.

```
┌─ rf_frontend/ — 10 GHz FMCW front-end ────────────────────────────────────┐
│                                                                           │
│  ADF4159 ramp PLL ──► HMC1163 10 GHz VCO ──► 4 dB branchline coupler      │
│                                                │             │            │
│                                     TX patch column    1→3 Wilkinson tree │
│                                                              │ LO         │
│  RX patch column ×3 ──► PMA3-14LN+ ──► LTC5548 ◄─────────────┘            │
│                                           │                               │
│                           ADA4940-2 IF ───┴──► ADAR7251  4 ch, 16-bit,    │
│                                                          simultaneous     │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
                                        │  40-pin FPC
                                        ▼
┌─ carrier_board/ — camera + radar carrier ─────────────────────────────────┐
│                                                                           │
│  radar 40-pin FPC ────────────────────────┐                               │
│  OV9281 ──MIPI──► TC358748 ──parallel─────┼──► 140-pin B2B ──► FPGA module│
│  INMP441 ──────────I2S────────────────────┘                               │
│                                                                           │
│  RTL8211F 1 GbE ◄─────────────────────────────────────────── UDP offload  │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
                                        │  parallel + I2S over the B2B connector
                                        ▼
┌─ fpga/ — Artix-7 backend ─────────────────────────────────────────────────┐
│                                                                           │
│  PPI bytes ──► TDM ──► BPF + Hilbert FIR ──► range FFT ×3 ──► DDR A       │
│                                                                │          │
│                        Doppler FFT ×3 ◄────────────────────────┘          │
│                              ├──► DDR B   per-RX complex RD maps          │
│                              └──► Σ|X|² ──► DDR C ──► CFAR ──► DDR D      │
│                                                                           │
│  audio + speed trigger ──► one-shot B/C/D dump ──► UDP packetizer         │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼  host PC — trajectory, angle of arrival, spin
```

## Repository structure

| Path | Subsystem |
|---|---|
| [`rf_frontend/`](rf_frontend/) | The 10 GHz 1TX/3RX FMCW front-end: KiCad schematics and PCB on Isola 370HR, plus the ADS/Momentum EM results for every distributed structure |
| [`fpga/`](fpga/) | Artix-7 gateware: the raw-ADC → range–Doppler → CFAR pipeline, the DDR-to-host UDP offload path, MicroBlaze firmware, and the simulation stack |
| [`carrier_board/`](carrier_board/) | The carrier PCB that ties camera, radar, FPGA module, Ethernet and audio together, plus the CS-mount lens mechanics |

Each subsystem has its own README with the detail; the tables below are the map.

### `rf_frontend/` — 10 GHz FMCW front-end

A closed-loop fractional-N ramp PLL (ADF4159) drives a 10 GHz VCO (HMC1163)
across a 192 MHz sweep in 108 µs. The VCO fundamental is tapped through a 4 dB
branchline coupler and a 1→3 Wilkinson tree to supply a coherent LO to three
LNA-first receive chains (PMA3-14LN+ → LTC5548 → ADA4940-2), which land on an
ADAR7251's simultaneously-sampled channels. The antennas are series-fed patch
columns arranged in a staggered "^", stepped λ/2 between adjacent phase centres
in both array axes, so per-Doppler-bin angles never wrap.

→ [`rf_frontend/README.md`](rf_frontend/README.md)

### `fpga/` — Artix-7 digital backend

`01_adar7251_ppi_rx` → TDM mux → band-pass + Hilbert FIR → per-RX range FFT →
transposed DDR scatter (region A) → Doppler sequencer → per-RX Doppler FFT →
non-coherent integration (region C) + per-RX complex cache (region B) →
streaming cell-averaging CFAR (region D). A 128 × 256 grid, double-buffered so
software never reads a torn frame. An audio-plus-speed trigger picks the first
frame after impact that clears both gates and dumps its B/C/D regions to the
host over UDP.

→ [`fpga/README.md`](fpga/README.md)

### `carrier_board/` — camera + radar carrier

Hosts the camera FPC, the radar's 40-pin FPC, a 140-pin B2B connector to the
FPGA module, I2S audio headers, gigabit Ethernet and the power tree. Two
revisions live here: the current OV9281 design (schematics complete, layout not
yet drawn) and the earlier IMX477 design, which is where the completed layout
is.

→ [`carrier_board/README.md`](carrier_board/README.md)

## Status

| Item | State |
|---|---|
| `rf_frontend/` schematics + PCB layout | Complete; every distributed structure verified in ADS/Momentum |
| `fpga/` RTL pipeline (`01`…`08`) | Complete; unit tests and a raw-ADC-to-CFAR xsim run against the real Xilinx FFT/FIR IP |
| `fpga/` host offload (streamer, packetizer, receiver) | Complete and unit-tested |
| `fpga/firmware/audio_dsp.c` | Complete |
| `fpga/firmware/fusion_control.c` | **Work in progress** — the audio + speed trigger FSM and its frame-selection policy are still being written |
| `carrier_board/ov9281/` | **Work in progress** — schematics done, PCB layout not yet started |
| `carrier_board/imx477/` | Obsolete and not optimized — rolling-shutter sensor, hand-unfriendly USB 3.0 BGA. Kept as the revision with a finished layout |

## Toolchain

| Tool | Used for |
|---|---|
| KiCad ≥ 8 | `rf_frontend/kicad/`, `carrier_board/ov9281/`, `carrier_board/imx477/` |
| Keysight ADS + Momentum | EM verification of the antenna, coupler, splitter and LO tree |
| Vivado 2025.1 | Block design, IP generation, synthesis, the xsim end-to-end run |
| Icarus Verilog | The RTL unit tests — they need no vendor IP (`fpga/sim/run_unit_tests.sh`) |
| Python 3 (numpy, matplotlib) | Stimulus generation, CFAR golden checks, the UDP frame receiver |
| FreeCAD | `carrier_board/mechanical/CS-mount.FCStd` |

Large generated trees are deliberately out of the repository: the Vivado
project's `.runs`/`.gen`/`.cache` output, xsim build directories, and the ADS
workspace (its results are captured as images in `rf_frontend/ads/`).
