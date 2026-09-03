# `fpga/utilization/` — resource reports

Out-of-context (OOC) synthesis of every block-design IP plus the pipeline RTL,
targeting `xc7a100tfgg484-2` under Vivado 2025.1. One `.rpt` per module, plus
`summary.txt`, which is the LUT / FF / DSP / BRAM column extracted from each.

## What it says

The radar datapath itself is cheap. Summing every hand-written module in
`summary.txt` except CFAR (see below) gives **942 LUT, 2102 FF, 6 DSP** — the
whole `01`…`07` chain plus the three host-offload blocks. Device usage is
dominated by infrastructure, not by radar:

| Block | LUT |
|---|---|
| AXI SmartConnect | ~24.5k |
| MIG (DDR3 controller) | ~5.2k |
| MicroBlaze | ~1.2k |
| Each FFT instance (6 of them) | 628–660 |

The design fits `xc7a100t` at roughly 58% LUT. It does **not** fit `xc7a35t`
without cutting the interconnect down — the SmartConnect alone is most of a
35T.

## Reading `summary.txt`

Two rows are misleading and should not be taken at face value:

- **`cfar_streaming_axi_verilog` (432k LUT / 702k FF / 0 BRAM).** Synthesized
  out of context, CFAR's 37-row sliding window has nothing telling it to use
  block RAM, so it infers as registers and F7/F8 muxes. In context it maps to
  BRAM. The OOC number is an artifact of how the report was generated, not the
  module's real cost.
- **`TOTAL (sum of OOC)`.** It carries that artifact, so it is far above the
  real device usage.

Every other per-module row is representative.

Each `.rpt` is `report_utilization` output on a separately-synthesized module;
regenerate them from the Vivado project, which is not tracked here.
