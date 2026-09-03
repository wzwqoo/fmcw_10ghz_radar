#!/usr/bin/env bash
set -euo pipefail
# ============================================================================
# run_e2e_3rx.sh  —  ONE-shot raw-ADC -> CFAR xsim e2e for radar_3rx.
#
# Self-contained.  Reuses ONLY the
# project's generated Xilinx FFT/FIR IP simulation sources (unavoidable — you
# cannot simulate a real xfft without them).  Compiles the radar_3rx RTL, a
# 3-lane behavioral-DDR testbench, elaborates and runs xsim in TURBO mode
# (fast ADC byte clock, à la test 05).  Real xfft IP => expect ~30 min.
#
#   raw ADAR7251 PPI -> 3ch TDM -> 3 range FFTs -> scatter_write_3rx -> DDR A
#     -> doppler_seq_3rx -> 3 doppler FFTs -> {cache_top, noncoh(3)} -> C
#     -> CFAR -> D ; checks each stage wrote/read its DDR region.
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RTL="$(cd "$SCRIPT_DIR/../rtl" && pwd)"

# Both of these must point at a local install / project; override in the env.
#   VIVADO_SETTINGS  Vivado settings64.sh
#   BD_GEN           the block design's generated IP tree, i.e.
#                    <proj>.gen/sources_1/bd/<bd_name>
VIVADO_SETTINGS="${VIVADO_SETTINGS:-/opt/Xilinx/2025.1/Vivado/settings64.sh}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
BD_GEN="${BD_GEN:-}"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/xsim_e2e_3rx}"
SNAPSHOT="${SNAPSHOT:-tb_e2e_3rx_xsim_elab}"
GEN="${GEN:-$SCRIPT_DIR/gen_ppi_hex_3rx.py}"
TB="$SCRIPT_DIR/tb_e2e_3rx_xsim.v"

# Turbo defaults (fast front end + short clocks), matching test 05.
TURBO_FAST_FRONTEND="${TURBO_FAST_FRONTEND:-1}"
CLK_HALF_NS="${CLK_HALF_NS:-1}"
SCLK_ADC_HALF_NS="${SCLK_ADC_HALF_NS:-1}"
FRAME_BUF_SEL="${FRAME_BUF_SEL:-1}"
DUMP_DDR="${DUMP_DDR:-1}"
SMOKE_MODE="${SMOKE_MODE:-0}"
STIM_LIMIT_BYTES="${STIM_LIMIT_BYTES:-0}"
SMOKE_DRAIN_CYCLES="${SMOKE_DRAIN_CYCLES:-512}"
PPI_PROGRESS_BYTES="${PPI_PROGRESS_BYTES:-8192}"
STATUS_INTERVAL_CYCLES="${STATUS_INTERVAL_CYCLES:-25000}"
TIMEOUT_CLK_CYCLES="${TIMEOUT_CLK_CYCLES:-15000000}"

[[ "${1:-}" == "clean" ]] && { rm -rf "$BUILD_DIR"; exit 0; }
[[ -f "$VIVADO_SETTINGS" ]] || { echo "ERROR: no Vivado settings: $VIVADO_SETTINGS (set VIVADO_SETTINGS)" >&2; exit 1; }
[[ -n "$BD_GEN" && -d "$BD_GEN" ]] || { echo "ERROR: set BD_GEN to <proj>.gen/sources_1/bd/<bd_name> (got: '${BD_GEN}')" >&2; exit 1; }
[[ -f "$TB" ]] || { echo "ERROR: testbench not found: $TB" >&2; exit 1; }

set +u; source "$VIVADO_SETTINGS"; set -u; unset LIBRARY_PATH || true

rm -rf "$BUILD_DIR"; mkdir -p "$BUILD_DIR"; cd "$BUILD_DIR"

# ── xsim library map ────────────────────────────────────────────────────────
XSIM_INI="$BUILD_DIR/xsim.local.ini"
LOCAL_LIBS=( xil_defaultlib xbip_utils_v3_0_14 axi_utils_v2_0_10 xbip_pipe_v3_0_10
  fir_compiler_v7_2_24 c_reg_fd_v12_0_10 xbip_dsp48_wrapper_v3_0_7 c_addsub_v12_0_20
  c_shift_ram_v12_0_19 mult_gen_v12_0_23 floating_point_v7_1_20 cmpy_v6_0_26 xfft_v9_1_14 )
: > "$XSIM_INI"
for lib in "${LOCAL_LIBS[@]}"; do
  mkdir -p "xsim.dir/$lib"
  printf '%s=xsim.dir/%s\n' "$lib" "$lib" >> "$XSIM_INI"
done

cp "$BD_GEN/ip/radar_test_fir_compiler_0_0/radar_test_fir_compiler_0_0.mif" . 2>/dev/null || true
cp "$BD_GEN/ip/radar_test_fir_compiler_0_1/radar_test_fir_compiler_0_1.mif" . 2>/dev/null || true

# ── Xilinx IP support VHDL (shared cores) ───────────────────────────────────
xvhdl --initfile "$XSIM_INI" --work xbip_utils_v3_0_14      "$BD_GEN/ipshared/b27f/hdl/xbip_utils_v3_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work axi_utils_v2_0_10       "$BD_GEN/ipshared/7e77/hdl/axi_utils_v2_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work xbip_pipe_v3_0_10       "$BD_GEN/ipshared/d531/hdl/xbip_pipe_v3_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work fir_compiler_v7_2_24    "$BD_GEN/ipshared/201d/hdl/fir_compiler_v7_2_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work xil_defaultlib \
  "$BD_GEN/ip/radar_test_fir_compiler_0_0/sim/radar_test_fir_compiler_0_0.vhd" \
  "$BD_GEN/ip/radar_test_fir_compiler_0_1/sim/radar_test_fir_compiler_0_1.vhd"
xvhdl --initfile "$XSIM_INI" --work c_reg_fd_v12_0_10       "$BD_GEN/ipshared/47fd/hdl/c_reg_fd_v12_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work xbip_dsp48_wrapper_v3_0_7 "$BD_GEN/ipshared/9bc6/hdl/xbip_dsp48_wrapper_v3_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work c_addsub_v12_0_20       "$BD_GEN/ipshared/c2a4/hdl/c_addsub_v12_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work c_shift_ram_v12_0_19    "$BD_GEN/ipshared/99ff/hdl/c_shift_ram_v12_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work mult_gen_v12_0_23       "$BD_GEN/ipshared/dad4/hdl/mult_gen_v12_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work floating_point_v7_1_20  "$BD_GEN/ipshared/53dc/hdl/floating_point_v7_1_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work cmpy_v6_0_26            "$BD_GEN/ipshared/6759/hdl/cmpy_v6_0_vh_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --2008 --work xfft_v9_1_14     "$BD_GEN/ipshared/7b99/hdl/xfft_v9_1_vh08_rfs.vhd"
xvhdl --initfile "$XSIM_INI" --work xfft_v9_1_14            "$BD_GEN/ipshared/7b99/hdl/xfft_v9_1_vh_rfs.vhd"

# Only the 6 FFT instances the 3-lane TB uses (3 range + 3 doppler).
xvhdl --initfile "$XSIM_INI" --work xil_defaultlib \
  "$BD_GEN/ip/radar_test_xfft_0_1/sim/radar_test_xfft_0_1.vhd" \
  "$BD_GEN/ip/radar_test_xfft_1_0/sim/radar_test_xfft_1_0.vhd" \
  "$BD_GEN/ip/radar_test_xfft_2_1/sim/radar_test_xfft_2_1.vhd" \
  "$BD_GEN/ip/radar_test_xfft_1_1/sim/radar_test_xfft_1_1.vhd" \
  "$BD_GEN/ip/radar_test_xfft_5_0/sim/radar_test_xfft_5_0.vhd" \
  "$BD_GEN/ip/radar_test_xfft_6_0/sim/radar_test_xfft_6_0.vhd"

# ── radar_3rx RTL (3-lane) ──────────────────────────────────────────────────
xvlog --initfile "$XSIM_INI" --work xil_defaultlib --relax -sv -i "$RTL" \
  "$RTL/01_adar7251_ppi_rx.v" \
  "$RTL/01b_ppi_to_tdm_bridge.v" \
  "$RTL/02_tdm_mux_3to1.v" \
  "$RTL/03_iq_delay_align.v" \
  "$RTL/04_iq_demux_1to3.v" \
  "$RTL/05a_scatter_write_master.v" \
  "$RTL/05a_scatter_write_3rx.v" \
  "$RTL/05b_doppler_seq_ctrl.v" \
  "$RTL/05b_doppler_seq_3rx.v" \
  "$RTL/06a_noncoh_integrator.v" \
  "$RTL/06b_cplx_cell_writer.v" \
  "$RTL/06b_cplx_cell_cache_top.v" \
  "$RTL/07a_rd_map_collector_summed.v" \
  "$RTL/08_cfar_detector.v"

# ── One frame of ADAR7251 PPI stimulus (writes ppi_stream.hex in CWD) ───────
"$PYTHON_BIN" "$GEN"
STIM_HEX="$BUILD_DIR/ppi_stream.hex"
[[ -s "$STIM_HEX" ]] || { echo "ERROR: stimulus not generated: $STIM_HEX" >&2; exit 1; }

# ── Testbench (forward TURBO define; it selects the fast front-end path) ─────
XVLOG_TB=()
[[ "$TURBO_FAST_FRONTEND" == "1" ]] && XVLOG_TB+=("-d" "TURBO_FAST_FRONTEND")
xvlog --initfile "$XSIM_INI" --work xil_defaultlib --relax -sv "${XVLOG_TB[@]}" -i "$RTL" "$TB"

# ── Elaborate ───────────────────────────────────────────────────────────────
mapfile -t LIBS < <(awk -F= '/^[A-Za-z0-9_]+[[:space:]]*=/{gsub(/[[:space:]]/,"",$1);print $1}' "$XSIM_INI" | sort -u)
XELAB_LIBS=(); for lib in "${LIBS[@]}"; do XELAB_LIBS+=("-L" "$lib"); done
xelab --initfile "$XSIM_INI" --relax "${XELAB_LIBS[@]}" \
  xil_defaultlib.tb_e2e_3rx_xsim -s "$SNAPSHOT"

cat > run_all.tcl <<'TCL'
run all
quit
TCL

# ── Run ─────────────────────────────────────────────────────────────────────
echo "INFO: running xsim (turbo=$TURBO_FAST_FRONTEND, ~30 min with real FFT IP)"
xsim "$SNAPSHOT" -tclbatch run_all.tcl \
  -testplusarg "STIM_HEX=$STIM_HEX" \
  -testplusarg "DUMP_DDR=$DUMP_DDR" \
  -testplusarg "DDR_DUMP_PREFIX=$BUILD_DIR/ddr" \
  -testplusarg "FRAME_BUF_SEL=$FRAME_BUF_SEL" \
  -testplusarg "CLK_HALF_NS=$CLK_HALF_NS" \
  -testplusarg "SCLK_ADC_HALF_NS=$SCLK_ADC_HALF_NS" \
  -testplusarg "SMOKE_MODE=$SMOKE_MODE" \
  -testplusarg "STIM_LIMIT_BYTES=$STIM_LIMIT_BYTES" \
  -testplusarg "SMOKE_DRAIN_CYCLES=$SMOKE_DRAIN_CYCLES" \
  -testplusarg "PPI_PROGRESS_BYTES=$PPI_PROGRESS_BYTES" \
  -testplusarg "STATUS_INTERVAL_CYCLES=$STATUS_INTERVAL_CYCLES" \
  -testplusarg "TIMEOUT_CLK_CYCLES=$TIMEOUT_CLK_CYCLES"
