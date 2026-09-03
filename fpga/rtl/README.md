# `fpga/rtl/` — the pipeline

Files are numbered in dataflow order. Everything runs on one fast clock domain
except the ADC byte-capture front end, and every module that crosses an AXI
`RVALID`/`TVALID` seam carries a skid buffer so it stays correct under
backpressure.

| File | Module | Does |
|---|---|---|
| `01_adar7251_ppi_rx.v` | `adar7251_ppi_rx` | Byte capture from a single ADAR7251 in PPI byte-wide mode. The part always streams all four physical channels (8 bytes per sample set), so this receives all 8 bytes — preserving SCLK / DATA_READY timing and the byte-7 valid pulse — and exposes the three that are wired to antennas |
| `01b_ppi_to_tdm_bridge.v` | `ppi_to_tdm_bridge` | Adapts three simultaneously-valid channels (one `sclk_adc` cycle every ~833 ns) into the staggered one-per-cycle presentation the TDM mux expects |
| `02_tdm_mux_3to1.v` | `tdm_mux_3to1` | Three slow channels into one tagged TDM stream at 3× the input rate |
| `03_iq_delay_align.v` | `iq_delay_align` | Delays I to match the Hilbert FIR's group delay on Q, so the pair recombines in phase |
| `04_iq_demux_1to3.v` | `iq_demux_1to3` | Splits the TDM I/Q stream back into three per-RX lanes |
| `05a_scatter_write_master.v` | `scatter_write_master` | Writes one receiver's range-FFT output to DDR3 **transposed** (column-major), at `buf_base + rx·RX_STRIDE + (k·N_CHIRPS + n)·SAMPLE_BYTES`, with the A-region ping-pong integrated |
| `05a_scatter_write_3rx.v` | `scatter_write_3rx` | Three-lane wrapper — one writer per range FFT |
| `05b_doppler_seq_ctrl.v` | `doppler_seq_ctrl` | Reads region A back as one contiguous `N_CHIRPS`-beat burst per range bin and streams it into a Doppler FFT. The transpose done on the write side is what makes this a single burst |
| `05b_doppler_seq_3rx.v` | `doppler_seq_3rx` | Three-lane wrapper, one per Doppler FFT IP |
| `06a_noncoh_integrator.v` | `noncoh_integrator` | Per cell, `Σ_rx (I² + Q²)` across the three receivers, saturated to `OUT_W` bits |
| `06b_cplx_cell_writer.v` | `cplx_cell_writer` | Writes one receiver's complex Doppler-FFT output to DDR3 region B — the per-Doppler-bin phase the host's AoA and spin-axis processing reads |
| `06b_cplx_cell_cache_top.v` | `cplx_cell_cache_top` | Three writers, three separate AXI4 master ports; the SmartConnect arbitrates on the MIG side |
| `07a_rd_map_collector_summed.v` | `rd_map_collector_summed` | Writes the summed power map to region C and hands it to CFAR |
| `08_cfar_detector.v` | `cfar_streaming_axi_verilog` | Streaming 2-D cell-averaging CFAR: a 37-row sliding window held in BRAM, one cell per cycle, packed detections to region D |
| `radar_params.vh` | — | Frame geometry and the complete DDR address map |

## `radar_params.vh`

Every stage and every testbench includes it, and nothing else defines these
numbers:

```
RADAR_RANGE_BINS    128
RADAR_DOPPLER_BINS  256
RADAR_RX_COUNT      3
```

Per-frame DDR products, and the ping-pong discipline:

| Region | Contents | Size |
|---|---|---|
| A | Range-FFT scratch, transposed, 3 RX | 384 KiB — ping-pongs independently |
| B | Per-RX complex range–Doppler maps | 384 KiB |
| C | Summed \|X\|² map, and the CFAR input image | 128 KiB |
| D | Packed CFAR detection bitmap | 4 KiB |

B, C and D switch together on one `frame_buf_sel` bit and are spaced on a 1 MiB
stride. Software must never read B from one frame and D from another — that is
the reason for the single select bit rather than three.
