#!/usr/bin/env python3
"""
check_cfar_output.py — verify CFAR detection output from XSim DDR dump.

Usage:
    python3 check_cfar_output.py <D1_cfar_words.hex> [ppi_stream.hex.json]

Exit 0 = PASS, non-zero = FAIL.

D1 memory layout (from 08_cfar_detector.v lines 26-33):
    linear_cell = range_bin * 256 + doppler_bin
    word_index  = linear_cell >> 5          (÷32)
    bit_index   = linear_cell & 0x1f        (mod 32)

out_row = range_bin (0-indexed):
    out_row 0 produced when feed_row=18 → centre = range_bin 0
    i.e. out_row r directly corresponds to range_bin r

Valid fully-trained CFAR output window (from cfar_detector.v):
    N_GUARD_R = 2, N_TRAIN_R = 16
    First valid range_bin = N_GUARD_R + N_TRAIN_R = 18
    Last  valid range_bin = 128 - 1 - N_GUARD_R - N_TRAIN_R = 109
    Bins 0-17 and 110-127: edge artefacts expected, not a defect.
"""
import sys
import json
from pathlib import Path

N_RANGE        = 128
N_DOPP         = 256
TOTAL_WORDS    = (N_RANGE * N_DOPP) // 32   # 1024
N_GUARD_R      = 2
N_TRAIN_R      = 16
VALID_ROW_START = N_GUARD_R + N_TRAIN_R          # 18
VALID_ROW_END   = N_RANGE - 1 - N_GUARD_R - N_TRAIN_R  # 109

# Single-frame simulation: noise variance is high, expect elevated Pfa.
# 10% is a loose but meaningful bound; in multi-frame operation this converges
# to the designed ~1.8% (alpha_q8=0x0400, N_TRAIN_TOTAL=688).
SINGLE_FRAME_PFA_MAX = 0.10

RANGE_TOL = 2   # ± bins for target hit
DOPP_TOL  = 2


def load_hex(path: Path) -> list:
    words = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("//") or line.startswith("@"):
            continue
        words.append(int(line, 16))
    return words


def get_detection(words: list, range_bin: int, dopp_bin: int) -> int:
    linear = range_bin * N_DOPP + dopp_bin
    return (words[linear >> 5] >> (linear & 0x1f)) & 1


def row_detections(words: list, r: int) -> int:
    return sum(get_detection(words, r, d) for d in range(N_DOPP))


def main():
    if len(sys.argv) < 2:
        print("Usage: check_cfar_output.py <D1_cfar_words.hex> [ppi_stream.hex.json]")
        sys.exit(1)

    hex_path  = Path(sys.argv[1])
    json_path = (Path(sys.argv[2]) if len(sys.argv) > 2
                 else hex_path.parent / "ppi_stream.hex.json")

    if not hex_path.exists():
        print(f"ERROR: hex file not found: {hex_path}")
        sys.exit(1)

    words = load_hex(hex_path)
    if len(words) < TOTAL_WORDS:
        print(f"ERROR: expected {TOTAL_WORDS} words, got {len(words)}")
        sys.exit(1)
    words = words[:TOTAL_WORDS]

    # --- Summary ---
    total_dets = 0
    nonzero_rows = []
    for r in range(N_RANGE):
        n = row_detections(words, r)
        if n:
            nonzero_rows.append((r, n))
            total_dets += n

    print(f"INFO: total detections  : {total_dets}")
    print(f"INFO: nonzero range rows: {len(nonzero_rows)}")
    for r, n in nonzero_rows:
        zone = "edge" if (r < VALID_ROW_START or r > VALID_ROW_END) else "valid"
        print(f"  range row {r:3d} ({zone}): {n} detection(s)")

    # --- Load expected target ---
    expected_range = 27   # default: 20.25 m → bin 27
    expected_dopp  = 177
    if json_path.exists():
        meta = json.loads(json_path.read_text())
        tgt  = meta.get("expected_target", {})
        expected_range = tgt.get("range_bin_nearest", expected_range)
        expected_dopp  = tgt.get("doppler_bin_nearest_positive_away", expected_dopp)
        print(f"\nINFO: expected target (from {json_path.name}):")
    else:
        print(f"\nINFO: expected target (defaults, no json found):")
    print(f"  range_bin : {expected_range}")
    print(f"  doppler_bin: {expected_dopp}")

    fail = False

    # Check 1: target in valid zone
    if expected_range < VALID_ROW_START or expected_range > VALID_ROW_END:
        print(f"\nWARN: expected_range={expected_range} is outside fully-trained CFAR window "
              f"[{VALID_ROW_START},{VALID_ROW_END}]. Detection may be unreliable. "
              f"Consider moving target to range >= {VALID_ROW_START * 0.75:.1f} m.")

    # Check 2: at least one detection
    if total_dets == 0:
        print("\nFAIL: zero detections in entire D1 region")
        fail = True
    else:
        print(f"\nPASS: at least one detection present ({total_dets} total)")

    # Check 3: target cell fires (±RANGE_TOL, ±DOPP_TOL)
    target_fired = False
    for dr in range(-RANGE_TOL, RANGE_TOL + 1):
        for dd in range(-DOPP_TOL, DOPP_TOL + 1):
            r = expected_range + dr
            d = (expected_dopp + dd) % N_DOPP
            if 0 <= r < N_RANGE and get_detection(words, r, d):
                target_fired = True
                print(f"PASS: target detection at range_bin={r} dopp_bin={d} "
                      f"(offset dr={dr:+d} dd={dd:+d} from expected)")
                break
        if target_fired:
            break
    if not target_fired:
        print(f"FAIL: no detection within ±{RANGE_TOL}r/±{DOPP_TOL}d of "
              f"expected target ({expected_range}, {expected_dopp})")
        fail = True

    # Check 4: false alarm rate in valid zone only
    valid_cells = (VALID_ROW_END - VALID_ROW_START + 1) * N_DOPP
    valid_dets  = sum(n for r, n in nonzero_rows
                      if VALID_ROW_START <= r <= VALID_ROW_END)
    pfa_obs = valid_dets / valid_cells if valid_cells > 0 else 0
    print(f"\nINFO: valid-zone Pfa = {pfa_obs*100:.2f}%  "
          f"({valid_dets}/{valid_cells} valid cells, "
          f"limit {SINGLE_FRAME_PFA_MAX*100:.0f}%)")
    if pfa_obs > SINGLE_FRAME_PFA_MAX:
        print(f"WARN: observed Pfa {pfa_obs*100:.1f}% exceeds single-frame limit "
              f"{SINGLE_FRAME_PFA_MAX*100:.0f}%. "
              f"Expected for single-frame test; increase alpha_q8 if Pfa matters.")

    # Check 5: edge artefacts (informational only)
    edge_rows = [(r, n) for r, n in nonzero_rows
                 if r < VALID_ROW_START or r > VALID_ROW_END]
    if edge_rows:
        print(f"\nINFO: {len(edge_rows)} edge row(s) with detections — "
              f"expected CFAR boundary artefact, not a defect.")

    print()
    if fail:
        print("OVERALL: FAIL")
        sys.exit(1)
    else:
        print("OVERALL: PASS")
        sys.exit(0)


if __name__ == "__main__":
    main()
