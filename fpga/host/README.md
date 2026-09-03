# radar_3rx host readout — one-shot B/C/D frame dump over UDP

Ships the **highest-power radar frame**'s post-Doppler DDR3 product regions
(B/C/D) to a host PC so the host can run post-CFAR / trajectory / spin
processing. One-shot per shot, bounded (~516 KiB for 3 RX), triggered by
software (the audio+speed fusion FSM decides *which* frame is the highest-power
one and pulses `start`).

Region **A** (range-FFT scratch) is intentionally **not** dumped: B is the
Doppler transform of A (so A is redundant for all target metrics), and A has an
independent ping/pong that would need freezing to stay coherent. Dumping only
B/C/D — which share one `frame_buf_sel` — makes the transfer coherent with a
single select bit.

## Datapath

```
DDR3 (MIG) ─AXI4─ AXI SmartConnect ─AXI4─ radar_frame_dump_streamer ─AXIS(32b)─ radar_udp_packetizer ─32b─ fifo_generator ─ udp_tx ─ PHY ─► host
                                          (.m_axis + sideband)         (.s_axis + sideband)         (Vivado IP)   (UDP core)
```

Two verified blocks in this folder, feeding the block design's FIFO + UDP core:

- **`radar_frame_dump_streamer.v`** — AXI4 read master that walks B→C→D of the
  selected frame buffer and emits the frame as one 32-bit AXI-Stream:
  - `start` (1-cycle) + `frame_buf_sel` (0=B0/C0/D0, 1=B1/C1/D1) +
    `frame_id` → `busy`, then `done` (1-cycle) after region D's last beat.
  - Sideband while a region streams: `region_id` (0=A,1=B,2=C,3=D), `frame_id_out`,
    `region_words`; `tuser` marks a region's first beat, `tlast` its last beat.
  - Output skid holds `tvalid`/`tdata` until `tready` — correct under UDP FIFO
    backpressure (verified by `tb_radar_frame_dump_streamer.v`).

- **`radar_udp_packetizer.v`** — drives the `fifo_generator` + `udp_tx`
  `tx_start_en/tx_data/tx_byte_num/tx_req/tx_done` idiom. Chops the stream into
  fixed `PAYLOAD_WORDS`-word UDP packets (default 256 = 1024-byte payload, +16-byte
  header = `PACKET_BYTES`=1040), filling the header from the streamer sideband and
  driving `region_first`/`region_last` flags from offsets. Header words are
  big-endian on the wire; payload words are byte-swapped to little-endian to match
  the host `recv_radar_frame_udp.py` (`>IIBBHI` header, `<u4` payload). Verified by
  `tb_radar_udp_packetizer.v`.

  Every region word-count is an exact multiple of `PAYLOAD_WORDS`, so all packets
  are full — there is no padding or ragged-tail logic.

## UDP packet header (16 bytes, big-endian)

| offset | size | field        | notes                                   |
|-------:|-----:|--------------|-----------------------------------------|
| 0      | u32  | magic        | `0x52414446` = `'RADF'`                  |
| 4      | u32  | frame_id     | echo of `frame_id_out`                   |
| 8      | u8   | region_id    | 0=A, 1=B, 2=C, 3=D                        |
| 9      | u8   | flags        | bit0 = region_first, bit1 = region_last  |
| 10     | u16  | reserved     | 0                                        |
| 12     | u32  | byte_offset  | byte offset of this payload in region    |

Payload after the header is raw little-endian `uint32` region data.

## Region sizes (3 RX, from radar_params.vh)

| region | id | contents                      | bytes   |
|--------|----|-------------------------------|---------|
| B      | 1  | per-RX complex RD map (3 RX)  | 384 KiB |
| C      | 2  | summed \|X\|² RD map           | 128 KiB |
| D      | 3  | packed CFAR det map           |   4 KiB |

Total ~516 KiB. Region A (id 0) is not dumped. C is one summed map
(RX-independent): 128×256×4 = 128 KiB, the range-Doppler magnitude image.

## How to connect (block design wiring)

Five connection groups. The streamer is an **AXI4 read master** and an **AXIS
master**; the packetizer is the AXIS slave + the existing FIFO/UDP TX path.

1. **Clock/reset** — drive `clk`/`rst_n` of both blocks from the same domain as
   the MIG user clock (or cross with a CDC FIFO if your UDP core runs on a
   different clock). `rst_n` active-low.

2. **Streamer AXI4 read → SmartConnect → MIG.** Wire `m_axi_ar*`/`m_axi_r*` to a
   slave port of the AXI SmartConnect that already feeds the MIG (the same one
   `doppler_seq_*` / `cfar` read through). Tie `m_axi_arsize`/`arburst` are driven
   by the block. No write channel needed (read-only).

3. **Streamer AXIS + sideband → packetizer.**
   `m_axis_tdata/tvalid/tready/tlast/tuser` → `s_axis_*`, and the sideband
   `region_id → s_region_id`, `frame_id_out → s_frame_id`, `region_words →
   s_region_words`.

4. **Packetizer → FIFO Generator → udp_tx.** Connect
   `fifo_din/fifo_wr_en/fifo_full/fifo_prog_full` and `fifo_dout/fifo_rd_en/
   fifo_empty` to a Vivado **FIFO Generator** (32-bit, common clock, depth ≥ a few
   packets = ≥ ~1024 words), and `tx_start_en/tx_data/tx_byte_num/tx_req/tx_done`
   to the `udp`/`udp_tx` core.

5. **Control (the trigger).** Pulse `start` for one cycle with `frame_buf_sel` and
   `frame_id` valid. Source options:
   - **Software (recommended for "highest-power frame"):** a MicroBlaze AXI-Lite
     register block — `radar_dump_axi_regs.v` in this folder — whose write sets
     `frame_buf_sel`/`frame_id` and pulses `start`. The audio+speed trigger FSM
     ([`../firmware/fusion_control.c`](../firmware/fusion_control.c), still a
     work in progress) decides which buffered frame is the highest-power one and
     issues the dump.
   - **Hardware:** gate `start` off `cplx_cell_cache_top.cache_ready` (frame
     complete) when a peak-power threshold is met. `busy` blocks re-triggering;
     wait for `done` before the next `start`.

```verilog
radar_frame_dump_streamer u_dump (
    .clk(clk), .rst_n(rst_n),
    .start(dump_start), .frame_buf_sel(dump_sel),
    .frame_id(dump_frame_id), .busy(dump_busy), .done(dump_done),
    // AXI4 read -> SmartConnect slave port -> MIG
    .m_axi_araddr(ar_addr), .m_axi_arlen(ar_len), .m_axi_arsize(ar_size),
    .m_axi_arburst(ar_burst), .m_axi_arvalid(ar_valid), .m_axi_arready(ar_ready),
    .m_axi_rdata(r_data), .m_axi_rresp(r_resp), .m_axi_rlast(r_last),
    .m_axi_rvalid(r_valid), .m_axi_rready(r_ready),
    // AXIS payload + sideband -> packetizer
    .m_axis_tdata(ax_tdata), .m_axis_tvalid(ax_tvalid), .m_axis_tlast(ax_tlast),
    .m_axis_tuser(ax_tuser), .m_axis_tready(ax_tready),
    .region_id(ax_region), .frame_id_out(ax_frame), .region_words(ax_words)
);

radar_udp_packetizer #(.PAYLOAD_WORDS(256), .PACKET_BYTES(16'd1040)) u_pkt (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tdata(ax_tdata), .s_axis_tvalid(ax_tvalid), .s_axis_tready(ax_tready),
    .s_axis_tuser(ax_tuser), .s_axis_tlast(ax_tlast),
    .s_region_id(ax_region), .s_frame_id(ax_frame), .s_region_words(ax_words),
    .fifo_din(fifo_din), .fifo_wr_en(fifo_wr_en), .fifo_full(fifo_full),
    .fifo_prog_full(fifo_prog_full), .fifo_dout(fifo_dout),
    .fifo_rd_en(fifo_rd_en), .fifo_empty(fifo_empty),
    .tx_start_en(tx_start_en), .tx_data(tx_data), .tx_byte_num(tx_byte_num),
    .tx_done(tx_done), .tx_req(tx_req),
    .queued_packets_dbg(), .tx_busy_dbg(), .underrun_dbg(), .frame_done_dbg()
);
```

> Ordering rule: only `start` a dump on a frame buffer software has *finished*
> reading-back is not required, but PL must not be overwriting that same
> `frame_buf_sel` B/C/D set while it streams. Trigger on a completed, idle buffer.

## Host receiver

```
python3 recv_radar_frame_udp.py --port 6000 --outdir ./capture --show
```

Reassembles each region by `byte_offset`, writes `frame_<id>_B/C/D.bin`, prints
a completeness report, and with `--show` renders the C range-Doppler map.

## Simulation

```
iverilog -g2012 -I ../rtl -o /tmp/dump_tb \
    tb_radar_frame_dump_streamer.v radar_frame_dump_streamer.v
vvp /tmp/dump_tb        # expect: total_beats = 230400, errors = 0, RESULT: PASS
```
