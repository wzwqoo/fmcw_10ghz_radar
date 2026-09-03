# `fpga/sim/` — simulation

Two tiers: unit tests that need nothing but Icarus Verilog, and a full
raw-ADC-to-CFAR run that instantiates the real Xilinx FFT and FIR IP.

## Unit tests

```bash
bash run_unit_tests.sh          # each testbench prints RESULT: PASS
```

| Test | Verifies |
|---|---|
| `tb_noncoh_integrator3.v` | 3-input Σ\|X\|² with saturation, and `tlast` propagation through the adder tree |
| `tb_tdm_frontend3.v` | `tdm_mux_3to1` mod-3 tag sequencing and `iq_demux_1to3` routing |
| `../host/tb_radar_frame_dump_streamer.v` | B→C→D DDR dump: region order, `region_id`, `tuser`/`tlast`, and correctness under backpressure |
| `../host/tb_radar_udp_packetizer.v` | Packet chopping, header fields, byte order, `region_first`/`region_last` |
| `../host/tb_radar_dump_regs.v` | AXI-Lite control register block |
| `../host/tb_radar_frame_sequencer.v` | `bundle_ready` pulse, frame-set report, `frame_buf_sel` toggle, and that `halt` holds the toggle |

The script builds into `$TMPDIR/radar_3rx_tests` and needs no vendor IP.

## End-to-end (`run_e2e_3rx.sh`)

One shot of raw ADAR7251 PPI bytes all the way to CFAR detections, through the
real vendor FFT/FIR IP — which is why it needs Vivado's `xsim` and a generated
block design rather than Icarus:

```
raw PPI → 3ch TDM → 3 range FFTs → scatter_write_3rx → DDR A
        → doppler_seq_3rx → 3 Doppler FFTs → {cplx_cell_cache, noncoh ×3} → C
        → CFAR → D
```

Each stage is checked for having written and read its DDR region.

```bash
export VIVADO_SETTINGS=/path/to/Vivado/settings64.sh
export BD_GEN=/path/to/<project>.gen/sources_1/bd/<bd_name>
bash run_e2e_3rx.sh
bash run_e2e_3rx.sh clean       # remove the xsim build directory
```

`BD_GEN` must point at the block design's generated IP tree — the script pulls
the FIR `.mif` files and the six FFT instances (3 range + 3 Doppler) from it,
and compiles the shared Xilinx IP support VHDL into a local `xsim.dir`.

Runtime is roughly 30 minutes with the real FFT IP. It defaults to turbo mode
(fast ADC byte clock, short clock periods); every knob is an environment
variable at the top of the script — `SMOKE_MODE=1` and `STIM_LIMIT_BYTES` give
a much shorter smoke run.

## Supporting scripts

| File | Purpose |
|---|---|
| `gen_ppi_hex_3rx.py` | Generates one frame of ADAR7251 PPI stimulus: a synthetic FMCW point target at a chosen range and velocity, with per-channel phases matching the array geometry in [`../../rf_frontend`](../../rf_frontend). Writes `ppi_stream.hex` (for `$readmemh`), a `.bin`, and a JSON metadata sidecar. No arguments — edit the constants at the top |
| `check_cfar_output.py` | Golden check on the CFAR detection map dumped from region D |
| `tb_e2e_3rx.v` | Behavioral end-to-end testbench |
| `tb_e2e_3rx_xsim.v` | The xsim variant, with the 3-lane behavioral DDR model that `run_e2e_3rx.sh` elaborates |

The summed region-C map and the region-D CFAR output are receiver-independent,
so the golden checks apply unchanged regardless of how the per-RX maps are
combined upstream.
