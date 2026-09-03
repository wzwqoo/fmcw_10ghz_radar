#!/usr/bin/env bash
# Run the iverilog-runnable 3-RX unit tests (no Xilinx IP / Vivado needed).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
RTL="$HERE/../rtl"
HOST="$HERE/../host"
OUT="${TMPDIR:-/tmp}/radar_3rx_tests"
mkdir -p "$OUT"

run() {
  local name="$1"; shift
  echo "=== $name ==="
  iverilog -g2012 -I "$RTL" -o "$OUT/$name" "$@"
  vvp "$OUT/$name" | tail -5
  echo
}

run noncoh_integrator3 "$HERE/tb_noncoh_integrator3.v" "$RTL/06a_noncoh_integrator.v"
run tdm_frontend3      "$HERE/tb_tdm_frontend3.v" "$RTL/02_tdm_mux_3to1.v" "$RTL/04_iq_demux_1to3.v"
run frame_dump_stream  "$HOST/tb_radar_frame_dump_streamer.v" "$HOST/radar_frame_dump_streamer.v"
run udp_packetizer     "$HOST/tb_radar_udp_packetizer.v" "$HOST/radar_udp_packetizer.v"
run dump_regs          "$HOST/tb_radar_dump_regs.v" "$HOST/radar_dump_axi_regs.v"
run frame_sequencer    "$HOST/tb_radar_frame_sequencer.v" "$HOST/radar_frame_sequencer.v"

echo "All unit tests done. Each should print RESULT: PASS above."
