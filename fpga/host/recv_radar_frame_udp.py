#!/usr/bin/env python3
# =============================================================================
#  recv_radar_frame_udp.py
#  Host-side receiver for the one-shot radar B/C/D frame dump produced by
#  radar_frame_dump_streamer.v -> radar UDP packetizer -> udp_tx.
#
#  Each UDP packet starts with a 16-byte header (big-endian):
#     0  : u32  magic       0x52414446  ('RADF')
#     4  : u32  frame_id
#     8  : u8   region_id    0=A 1=B 2=C 3=D
#     9  : u8   flags        bit0 = region_first packet, bit1 = region_last packet
#     10 : u16  reserved
#     12 : u32  byte_offset  byte offset of this payload within the region
#  followed by up to PAYLOAD_BYTES of raw little-endian uint32 region data.
#
#  Region sizes (3-RX, must match radar_params.vh):
#     A range-FFT buffer       384 KiB
#     B per-RX complex RD map  384 KiB
#     C summed |X|^2 RD map     96 KiB   (host runs post-CFAR on this + B)
#     D packed CFAR det map      4 KiB
#
#  Output: <outdir>/frame_<id>_<A|B|C|D>.bin  plus a printed completeness report.
#  With --show, the C region is rendered as a 128x256 range-Doppler magnitude map.
# =============================================================================
import argparse
import socket
import struct
import sys

MAGIC = 0x52414446
HDR = struct.Struct(">IIBBHI")  # magic, frame_id, region_id, flags, rsvd, byte_offset
HDR_LEN = HDR.size

# Only B/C/D are dumped (region A/id 0 is redundant with B and is not sent).
REGION_NAME = {1: "B", 2: "C", 3: "D"}
REGION_IDS = (1, 2, 3)
# Expected region sizes in bytes (3-RX). Used only for the completeness report.
REGION_BYTES = {
    1: 3 * 128 * 256 * 4,   # B (3 RX complex maps)
    2: 1 * 128 * 256 * 4,   # C (summed power map)
    3: ((128 * 256 + 31) // 32) * 4,  # D (packed detections)
}

RANGE_BINS = 128
DOPPLER_BINS = 256


def main():
    ap = argparse.ArgumentParser(description="Receive a radar A/B/C/D DDR frame over UDP")
    ap.add_argument("--port", type=int, default=6000, help="UDP listen port")
    ap.add_argument("--bind", default="0.0.0.0", help="bind address")
    ap.add_argument("--outdir", default=".", help="output directory for region .bin files")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="seconds of silence after first packet before giving up")
    ap.add_argument("--show", action="store_true", help="render the C magnitude map (needs numpy/matplotlib)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 << 20)
    sock.bind((args.bind, args.port))
    print(f"listening on {args.bind}:{args.port} ...", flush=True)

    # regions[(frame_id, region_id)] = bytearray reassembled by byte_offset
    regions = {}
    region_last_seen = set()
    cur_frame = None

    sock.settimeout(None)
    while True:
        try:
            pkt, _ = sock.recvfrom(65535)
        except socket.timeout:
            break
        if len(pkt) < HDR_LEN:
            continue
        magic, frame_id, region_id, flags, _rsvd, byte_off = HDR.unpack_from(pkt, 0)
        if magic != MAGIC:
            continue
        payload = pkt[HDR_LEN:]

        if cur_frame is None:
            cur_frame = frame_id
            sock.settimeout(args.timeout)  # arm the silence timeout after first packet
        key = (frame_id, region_id)
        buf = regions.get(key)
        if buf is None:
            buf = bytearray(REGION_BYTES.get(region_id, 0))
            regions[key] = buf
        end = byte_off + len(payload)
        if end > len(buf):
            buf.extend(b"\x00" * (end - len(buf)))
        buf[byte_off:end] = payload
        if flags & 0x2:
            region_last_seen.add(key)

    # ── Report + write ────────────────────────────────────────────────────
    if cur_frame is None:
        print("no radar frame received.", file=sys.stderr)
        return 1

    print(f"\nframe_id = {cur_frame} (0x{cur_frame:08x})")
    ok = True
    for region_id in REGION_IDS:
        key = (cur_frame, region_id)
        buf = regions.get(key)
        name = REGION_NAME[region_id]
        if buf is None:
            print(f"  region {name}: MISSING")
            ok = False
            continue
        got = len(buf)
        want = REGION_BYTES[region_id]
        complete = (key in region_last_seen) and (got >= want)
        ok = ok and complete
        path = f"{args.outdir}/frame_{cur_frame}_{name}.bin"
        with open(path, "wb") as f:
            f.write(buf[:want] if want else buf)
        print(f"  region {name}: {got}/{want} bytes "
              f"{'OK' if complete else 'INCOMPLETE'} -> {path}")

    if args.show:
        show_c_map(regions.get((cur_frame, 2)))

    print("RESULT:", "COMPLETE" if ok else "INCOMPLETE")
    return 0 if ok else 2


def show_c_map(cbuf):
    if cbuf is None:
        print("no C region to show", file=sys.stderr)
        return
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("numpy/matplotlib not available; skipping --show", file=sys.stderr)
        return
    arr = np.frombuffer(bytes(cbuf), dtype="<u4", count=RANGE_BINS * DOPPLER_BINS)
    img = arr.reshape(RANGE_BINS, DOPPLER_BINS).astype(np.float64)
    img = 10.0 * np.log10(img + 1.0)  # dB-ish for display
    plt.imshow(img, aspect="auto", origin="lower")
    plt.xlabel("Doppler bin")
    plt.ylabel("Range bin")
    plt.title("C: summed |X|^2 range-Doppler map")
    plt.colorbar(label="10*log10(power)")
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
