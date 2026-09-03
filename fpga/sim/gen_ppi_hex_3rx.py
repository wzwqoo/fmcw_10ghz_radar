#!/usr/bin/env python3
"""
gen_ppi_hex_3rx.py

Self-contained ADAR7251 PPI stimulus generator for the xsim end-to-end run of
the 01..08 pipeline (see run_e2e_3rx.sh).

There is intentionally no argparse and no importlib.  Edit the constants below
when dimensions or output paths need to change.  Default output is written to
this script's current working directory, which the XSim run script sets to the
simulation build directory.

Default output:
    ppi_stream.hex       one 8-bit hex byte per line, for Verilog $readmemh
    ppi_stream.bin       same byte stream in raw binary form
    ppi_stream.hex.json  metadata sidecar for sanity checking
    ppi_preview.csv      short first-sample preview

Default dimensions:
    128 samples/chirp * 256 chirps/frame * 1 frame * 8 bytes/sample_set
    = 262144 bytes

Default waveform:
    One deterministic FMCW sawtooth point target for the ADAR7251 PPI path:
    10 GHz carrier, 200 MHz sweep, 1.2 MSPS real ADC, 3 m initial range,
    60 mph radial velocity away, 15 degree azimuth right, 15 degree elevation.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Final, List, Sequence

import numpy as np


# =============================================================================
# Hardcoded user parameters
# =============================================================================

# Output filenames are deliberately fixed so the Verilog/SystemVerilog XSim
# testbench and shell script do not need argument plumbing.
OUTPUT_HEX: Final[Path] = Path("ppi_stream.hex")
OUTPUT_BIN: Final[Path] = Path("ppi_stream.bin")
OUTPUT_METADATA: Final[Path] = Path("ppi_stream.hex.json")
OUTPUT_PREVIEW_CSV: Final[Path] = Path("ppi_preview.csv")

# Radar pipeline dimensions.  Keep FRAMES=1 for the current 01..08 pipeline
# smoke/end-to-end test because the original cocotb test also drove one frame.
SAMPLES_PER_CHIRP: Final[int] = 128
CHIRPS_PER_FRAME: Final[int] = 256
FRAMES: Final[int] = 1

# ADAR7251 / FMCW waveform configuration.  The ADAR7251 presents one real
# signed 16-bit sample per RX channel; the downstream FIR/Hilbert path builds
# the analytic I/Q stream in gateware.
SPEED_OF_LIGHT_MPS: Final[float] = 299_792_458.0
FS_ADC_HZ: Final[float] = 1_200_000.0
SCLK_ADC_HZ: Final[float] = 9_600_000.0
CLK_SYS_HZ: Final[float] = 100_000_000.0
RF_CENTER_HZ: Final[float] = 10_000_000_000.0
FMCW_BANDWIDTH_HZ: Final[float] = 200_000_000.0
TARGET_INITIAL_RANGE_M: Final[float] = 20.25  # bin 27 — inside valid CFAR window (N_GUARD_R+N_TRAIN_R=18 min)
TARGET_VELOCITY_MPS: Final[float] = 60.0 * 0.44704
TARGET_AZ_DEG: Final[float] = 15.0
TARGET_EL_DEG: Final[float] = 15.0
TARGET_AMPLITUDE_LSB: Final[int] = 12_000
TARGET_PHASE_RAD: Final[float] = 0.31
DC_OFFSET_LSB: Final[int] = 0
NOISE_STD_LSB: Final[float] = 80.0
CHANNEL_EXTRA_NOISE_STD_LSB: Final[float] = 35.0
FULL_SCALE_LSB: Final[int] = 32760
SEED: Final[int] = 7251
PREVIEW_ROWS: Final[int] = 32

# Keep this false for clean range-bin energy in the regression stimulus.
# When true, 60 mph moves the target by about one 0.75 m range bin during a
# 27.3 ms coherent frame, which is physical but smears a single-bin test target.
ENABLE_RANGE_MIGRATION_WITHIN_FRAME: Final[bool] = False

# RX phase-centre positions in half-wavelengths, as laid out on the V2 board
# (see ../../rf_frontend, antenna elements AE1/AE2/AE3).  x = along a patch
# column, y = across the columns (the stacking axis).  The three columns form
# a staggered "^": adjacent steps are lambda/2 in BOTH axes, so the outer pair (CH1,CH3)
# ends up a full lambda apart across the stack.
#
#            x (along column)   y (across columns)
#   CH1  AE1        0                  0
#   CH2  AE2        1  (lambda/2)      1  (lambda/2)      <- apex
#   CH3  AE3        0                  2  (lambda)
#   CH4  --         unpopulated: the ADAR7251's AIN4P/AIN4N are not connected on
#                   this board, so CH4 repeats CH1 to keep the four-channel PPI
#                   plumbing exercised without inventing a fourth aperture.
#
# Channel order is the PPI byte order consumed by 01_adar7251_ppi_rx.v.
RX_X_HALFWAVE: Final[tuple[float, ...]] = (0.0, 1.0, 0.0, 0.0)
RX_Y_HALFWAVE: Final[tuple[float, ...]] = (0.0, 1.0, 2.0, 0.0)

# PPI format constants.  Do not change unless adar7251_ppi_rx.v byte packing
# changes.
CHANNELS: Final[int] = 4
BYTES_PER_CHANNEL_SAMPLE: Final[int] = 2
BYTES_PER_SAMPLE_SET: Final[int] = CHANNELS * BYTES_PER_CHANNEL_SAMPLE
BITS_PER_CHANNEL_SAMPLE: Final[int] = 16
PPI_BUS_WIDTH_BITS: Final[int] = 8
INT16_MIN: Final[int] = -32768
INT16_MAX: Final[int] = 32767
BYTE_MASK: Final[int] = 0xFF


# =============================================================================
# Data containers
# =============================================================================


@dataclass(frozen=True)
class GeneratorConfig:
    """Fixed configuration snapshot written into the metadata JSON file."""

    samples_per_chirp: int = SAMPLES_PER_CHIRP
    chirps_per_frame: int = CHIRPS_PER_FRAME
    frames: int = FRAMES
    fs_adc_hz: float = FS_ADC_HZ
    sclk_adc_hz: float = SCLK_ADC_HZ
    clk_sys_hz: float = CLK_SYS_HZ
    speed_of_light_mps: float = SPEED_OF_LIGHT_MPS
    rf_center_hz: float = RF_CENTER_HZ
    fmcw_bandwidth_hz: float = FMCW_BANDWIDTH_HZ
    target_initial_range_m: float = TARGET_INITIAL_RANGE_M
    target_velocity_mps: float = TARGET_VELOCITY_MPS
    target_az_deg: float = TARGET_AZ_DEG
    target_el_deg: float = TARGET_EL_DEG
    target_amplitude_lsb: int = TARGET_AMPLITUDE_LSB
    target_phase_rad: float = TARGET_PHASE_RAD
    enable_range_migration_within_frame: bool = ENABLE_RANGE_MIGRATION_WITHIN_FRAME
    dc_offset_lsb: int = DC_OFFSET_LSB
    noise_std_lsb: float = NOISE_STD_LSB
    channel_extra_noise_std_lsb: float = CHANNEL_EXTRA_NOISE_STD_LSB
    full_scale_lsb: int = FULL_SCALE_LSB
    seed: int = SEED
    preview_rows: int = PREVIEW_ROWS


@dataclass(frozen=True)
class DerivedConfig:
    """Derived sizes used by both generation and HDL alignment checks."""

    total_sample_sets: int
    total_stream_bytes: int
    chirp_period_s: float
    frame_period_s: float
    stream_duration_s: float
    chirp_slope_hz_per_s: float
    wavelength_m: float
    rx_spacing_m: float
    range_resolution_m: float
    velocity_resolution_mps: float
    max_unambiguous_velocity_mps: float
    real_adc_max_range_m: float
    range_fft_bin_hz: float
    doppler_bin_hz: float
    target_range_beat_hz: float
    target_doppler_hz: float
    target_range_bin: float
    target_doppler_bin: float
    target_frame_range_step_m: float
    bytes_per_sample_set: int
    channels: int
    bits_per_channel_sample: int
    ppi_bus_width_bits: int


# =============================================================================
# Validation helpers
# =============================================================================


def validate_config(cfg: GeneratorConfig) -> None:
    """Validate constants before generating files so mistakes fail early."""

    assert cfg.samples_per_chirp > 0, "samples_per_chirp must be positive"
    assert cfg.chirps_per_frame > 0, "chirps_per_frame must be positive"
    assert cfg.frames > 0, "frames must be positive"
    assert cfg.fs_adc_hz > 0.0, "fs_adc_hz must be positive"
    assert cfg.sclk_adc_hz > 0.0, "sclk_adc_hz must be positive"
    assert cfg.clk_sys_hz > 0.0, "clk_sys_hz must be positive"
    assert cfg.speed_of_light_mps > 0.0, "speed_of_light_mps must be positive"
    assert cfg.rf_center_hz > 0.0, "rf_center_hz must be positive"
    assert cfg.fmcw_bandwidth_hz > 0.0, "fmcw_bandwidth_hz must be positive"
    assert cfg.target_initial_range_m >= 0.0, "target range must be non-negative"
    assert -90.0 <= cfg.target_az_deg <= 90.0, "target azimuth should be in [-90, 90]"
    assert -90.0 <= cfg.target_el_deg <= 90.0, "target elevation should be in [-90, 90]"
    assert 0 <= cfg.target_amplitude_lsb <= INT16_MAX, "target amplitude must fit int16"
    assert INT16_MIN <= cfg.dc_offset_lsb <= INT16_MAX, "dc_offset_lsb must fit int16"
    assert cfg.noise_std_lsb >= 0.0, "noise_std_lsb must be non-negative"
    assert cfg.channel_extra_noise_std_lsb >= 0.0, "extra noise must be non-negative"
    assert 1 <= cfg.full_scale_lsb <= INT16_MAX, "full_scale_lsb must be 1..32767"
    assert cfg.preview_rows >= 0, "preview_rows must be non-negative"

    expected_sclk = cfg.fs_adc_hz * float(BYTES_PER_SAMPLE_SET)
    relative_error = abs(cfg.sclk_adc_hz - expected_sclk) / expected_sclk
    assert relative_error < 0.02, "sclk_adc_hz should be close to fs_adc_hz * 8"



def validate_derived_config(cfg: GeneratorConfig, derived: DerivedConfig) -> None:
    """Validate derived FMCW quantities against the ADAR7251 real-ADC limits."""

    assert derived.chirp_slope_hz_per_s > 0.0, "chirp slope must be positive"
    assert derived.wavelength_m > 0.0, "wavelength must be positive"
    assert derived.range_resolution_m > 0.0, "range resolution must be positive"
    assert derived.velocity_resolution_mps > 0.0, "velocity resolution must be positive"
    assert derived.target_range_beat_hz >= 0.0, "range beat must be non-negative"
    assert derived.real_adc_max_range_m > cfg.target_initial_range_m, "target starts beyond real ADC range"

    if cfg.enable_range_migration_within_frame:
        final_range_time_s = derived.stream_duration_s
    else:
        final_range_time_s = float(cfg.frames - 1) * derived.frame_period_s
    final_frame_range_m = cfg.target_initial_range_m + cfg.target_velocity_mps * final_range_time_s
    assert final_frame_range_m >= 0.0, "target range became negative"
    assert final_frame_range_m < derived.real_adc_max_range_m, "target range exceeds real ADC range"

    nyquist_hz = cfg.fs_adc_hz / 2.0
    max_frame_beat_hz = 2.0 * derived.chirp_slope_hz_per_s * final_frame_range_m / cfg.speed_of_light_mps
    assert max_frame_beat_hz < nyquist_hz, "target range beat exceeds ADAR7251 real-ADC Nyquist"
    assert abs(cfg.target_velocity_mps) < derived.max_unambiguous_velocity_mps, "target velocity aliases"
    assert 0.0 <= derived.target_doppler_bin < float(cfg.chirps_per_frame), "target Doppler bin is out of grid"



def derive_config(cfg: GeneratorConfig) -> DerivedConfig:
    """Compute fixed dimensions from the hardcoded top-level config."""

    validate_config(cfg)
    total_sample_sets = cfg.samples_per_chirp * cfg.chirps_per_frame * cfg.frames
    total_stream_bytes = total_sample_sets * BYTES_PER_SAMPLE_SET
    chirp_period_s = cfg.samples_per_chirp / cfg.fs_adc_hz
    frame_period_s = cfg.chirps_per_frame * chirp_period_s
    stream_duration_s = cfg.frames * frame_period_s
    chirp_slope_hz_per_s = cfg.fmcw_bandwidth_hz / chirp_period_s
    wavelength_m = cfg.speed_of_light_mps / cfg.rf_center_hz
    rx_spacing_m = wavelength_m / 2.0   # adjacent step; outer pair spans 2x this
    range_resolution_m = cfg.speed_of_light_mps / (2.0 * cfg.fmcw_bandwidth_hz)
    velocity_resolution_mps = wavelength_m / (2.0 * cfg.chirps_per_frame * chirp_period_s)
    max_unambiguous_velocity_mps = wavelength_m / (4.0 * chirp_period_s)
    real_adc_max_range_m = (cfg.fs_adc_hz / 2.0) * cfg.speed_of_light_mps / (2.0 * chirp_slope_hz_per_s)
    range_fft_bin_hz = cfg.fs_adc_hz / cfg.samples_per_chirp
    doppler_bin_hz = (1.0 / chirp_period_s) / cfg.chirps_per_frame
    target_range_beat_hz = 2.0 * chirp_slope_hz_per_s * cfg.target_initial_range_m / cfg.speed_of_light_mps
    target_doppler_hz = 2.0 * cfg.target_velocity_mps / wavelength_m
    target_range_bin = target_range_beat_hz / range_fft_bin_hz
    target_doppler_bin = (cfg.chirps_per_frame / 2.0) + (target_doppler_hz / doppler_bin_hz)
    target_frame_range_step_m = cfg.target_velocity_mps * frame_period_s

    derived = DerivedConfig(
        total_sample_sets=total_sample_sets,
        total_stream_bytes=total_stream_bytes,
        chirp_period_s=chirp_period_s,
        frame_period_s=frame_period_s,
        stream_duration_s=stream_duration_s,
        chirp_slope_hz_per_s=chirp_slope_hz_per_s,
        wavelength_m=wavelength_m,
        rx_spacing_m=rx_spacing_m,
        range_resolution_m=range_resolution_m,
        velocity_resolution_mps=velocity_resolution_mps,
        max_unambiguous_velocity_mps=max_unambiguous_velocity_mps,
        real_adc_max_range_m=real_adc_max_range_m,
        range_fft_bin_hz=range_fft_bin_hz,
        doppler_bin_hz=doppler_bin_hz,
        target_range_beat_hz=target_range_beat_hz,
        target_doppler_hz=target_doppler_hz,
        target_range_bin=target_range_bin,
        target_doppler_bin=target_doppler_bin,
        target_frame_range_step_m=target_frame_range_step_m,
        bytes_per_sample_set=BYTES_PER_SAMPLE_SET,
        channels=CHANNELS,
        bits_per_channel_sample=BITS_PER_CHANNEL_SAMPLE,
        ppi_bus_width_bits=PPI_BUS_WIDTH_BITS,
    )
    validate_derived_config(cfg, derived)
    return derived



def ensure_int16(value: int) -> int:
    """Clamp integer-like values into signed int16 range."""

    assert isinstance(value, (int, np.integer)), "value must be integer-like"
    if value > INT16_MAX:
        return INT16_MAX
    if value < INT16_MIN:
        return INT16_MIN
    return int(value)


# =============================================================================
# FMCW point-target waveform generation
# =============================================================================


def rx_phase_offsets_rad(cfg: GeneratorConfig, derived: DerivedConfig) -> np.ndarray:
    """Return CH1..CH4 phase offsets for the staggered "^" RX layout.

    Azimuth steers the along-column axis (x) and elevation the across-column
    axis (y), which is how the board sits with its long edge horizontal.
    """

    validate_config(cfg)
    assert derived.channels == CHANNELS, "this generator assumes four RX channels"
    assert len(RX_X_HALFWAVE) == CHANNELS, "RX_X_HALFWAVE must list four channels"
    assert len(RX_Y_HALFWAVE) == CHANNELS, "RX_Y_HALFWAVE must list four channels"

    az_rad = math.radians(cfg.target_az_deg)
    el_rad = math.radians(cfg.target_el_deg)
    direction_x = math.cos(el_rad) * math.sin(az_rad)
    direction_y = math.sin(el_rad)
    phase_per_halfwave = math.pi

    phases = []
    for x_halfwave, y_halfwave in zip(RX_X_HALFWAVE, RX_Y_HALFWAVE):
        phase = phase_per_halfwave * ((x_halfwave * direction_x) + (y_halfwave * direction_y))
        phases.append(phase)

    return np.array(phases, dtype=np.float64)



def generate_channel_samples(cfg: GeneratorConfig, derived: DerivedConfig) -> np.ndarray:
    """Generate signed int16 ADAR7251 samples with shape [sample_set, channel]."""

    validate_config(cfg)
    assert derived.channels == CHANNELS, "this generator assumes four channels"

    rng = np.random.default_rng(cfg.seed)
    rx_phase = rx_phase_offsets_rad(cfg, derived)
    fast_sample_index = np.arange(cfg.samples_per_chirp, dtype=np.float64)
    samples_float = np.zeros((derived.total_sample_sets, CHANNELS), dtype=np.float64)

    sample_row = 0
    for frame_idx in range(cfg.frames):
        frame_start_s = float(frame_idx) * derived.frame_period_s
        frame_range_m = cfg.target_initial_range_m + cfg.target_velocity_mps * frame_start_s

        for chirp_idx in range(cfg.chirps_per_frame):
            chirp_start_s = frame_start_s + (float(chirp_idx) * derived.chirp_period_s)
            if cfg.enable_range_migration_within_frame:
                range_m = cfg.target_initial_range_m + cfg.target_velocity_mps * chirp_start_s
            else:
                range_m = frame_range_m

            beat_hz = 2.0 * derived.chirp_slope_hz_per_s * range_m / cfg.speed_of_light_mps
            range_phase = 2.0 * math.pi * beat_hz * fast_sample_index / cfg.fs_adc_hz
            doppler_phase = 2.0 * math.pi * derived.target_doppler_hz * chirp_start_s
            total_phase = range_phase[:, None] + doppler_phase + rx_phase[None, :] + cfg.target_phase_rad

            chirp_samples = float(cfg.target_amplitude_lsb) * np.cos(total_phase)
            row_stop = sample_row + cfg.samples_per_chirp
            samples_float[sample_row:row_stop, :] = chirp_samples
            sample_row = row_stop

    assert sample_row == derived.total_sample_sets, "generated sample row count mismatch"

    common_noise = rng.normal(
        loc=0.0,
        scale=cfg.noise_std_lsb,
        size=(derived.total_sample_sets, 1),
    )
    channel_noise = rng.normal(
        loc=0.0,
        scale=cfg.channel_extra_noise_std_lsb,
        size=(derived.total_sample_sets, CHANNELS),
    )

    samples_float = samples_float + common_noise + channel_noise + float(cfg.dc_offset_lsb)
    samples_rounded = np.rint(samples_float)
    samples_clipped = np.clip(samples_rounded, INT16_MIN, INT16_MAX).astype(np.int16)

    assert samples_clipped.shape == (derived.total_sample_sets, CHANNELS), "sample matrix shape mismatch"
    assert samples_clipped.dtype == np.int16, "sample matrix must be int16"
    return samples_clipped


# =============================================================================
# PPI byte packing
# =============================================================================


def sample_to_ppi_bytes(sample: Sequence[int]) -> List[int]:
    """Pack one four-channel signed sample set into eight high-first bytes."""

    assert len(sample) == CHANNELS, "one sample set must contain four channels"
    packed: List[int] = []

    for value in sample:
        signed_value = ensure_int16(int(value))
        unsigned_value = signed_value & 0xFFFF
        packed.append((unsigned_value >> 8) & BYTE_MASK)
        packed.append(unsigned_value & BYTE_MASK)

    assert len(packed) == BYTES_PER_SAMPLE_SET, "packed sample must have eight bytes"
    assert all(0 <= byte <= 255 for byte in packed), "packed values must be bytes"
    return packed



def pack_all_samples_to_stream(samples_int16: np.ndarray) -> np.ndarray:
    """Pack [N,4] int16 samples into [N*8] uint8 PPI stream."""

    assert isinstance(samples_int16, np.ndarray), "samples_int16 must be a numpy array"
    assert samples_int16.ndim == 2, "samples_int16 must be a 2D matrix"
    assert samples_int16.shape[1] == CHANNELS, "samples_int16 must have four channels"
    assert samples_int16.dtype == np.int16, "samples_int16 must use int16 dtype"

    # Big-endian int16 conversion gives CHx_HI then CHx_LO.  Row-major flattening
    # preserves the required PPI byte order per sample set.
    big_endian = samples_int16.astype(">i2", copy=False)
    stream_u8 = np.frombuffer(big_endian.tobytes(order="C"), dtype=np.uint8).copy()

    expected_len = samples_int16.shape[0] * BYTES_PER_SAMPLE_SET
    assert stream_u8.shape == (expected_len,), "stream byte length mismatch"
    return stream_u8



def unpack_stream_for_self_check(stream_u8: np.ndarray) -> np.ndarray:
    """Unpack generated PPI bytes back to int16 samples for self-checking."""

    assert isinstance(stream_u8, np.ndarray), "stream_u8 must be a numpy array"
    assert stream_u8.ndim == 1, "stream_u8 must be a flat vector"
    assert stream_u8.dtype == np.uint8, "stream_u8 must use uint8 dtype"
    assert (stream_u8.size % BYTES_PER_SAMPLE_SET) == 0, "stream length must divide by eight"

    n_samples = stream_u8.size // BYTES_PER_SAMPLE_SET
    rows = stream_u8.reshape(n_samples, BYTES_PER_SAMPLE_SET)
    reconstructed = np.zeros((n_samples, CHANNELS), dtype=np.int16)

    for ch in range(CHANNELS):
        hi = rows[:, 2 * ch].astype(np.uint16)
        lo = rows[:, 2 * ch + 1].astype(np.uint16)
        unsigned = (hi << 8) | lo
        reconstructed[:, ch] = unsigned.view(np.int16)

    assert reconstructed.shape == (n_samples, CHANNELS), "unpacked shape mismatch"
    return reconstructed



def run_self_check(samples_int16: np.ndarray, stream_u8: np.ndarray) -> None:
    """Verify generated bytes unpack exactly back to the generated samples."""

    assert samples_int16.ndim == 2 and samples_int16.shape[1] == CHANNELS, "samples must be [N,4]"
    assert stream_u8.ndim == 1 and stream_u8.dtype == np.uint8, "stream must be flat uint8"

    unpacked = unpack_stream_for_self_check(stream_u8)
    if not np.array_equal(samples_int16, unpacked):
        mismatch = np.argwhere(samples_int16 != unpacked)
        first = mismatch[0]
        sample_idx = int(first[0])
        channel_idx = int(first[1])
        raise RuntimeError(
            "PPI pack/unpack self-check failed at "
            f"sample={sample_idx}, channel={channel_idx}: "
            f"generated={int(samples_int16[sample_idx, channel_idx])}, "
            f"unpacked={int(unpacked[sample_idx, channel_idx])}"
        )


# =============================================================================
# File writers
# =============================================================================


def write_hex_stream(path: Path, stream_u8: np.ndarray) -> None:
    """Write one 8-bit hex byte per line for Verilog $readmemh."""

    assert path.suffix == ".hex", "hex path should end in .hex"
    assert stream_u8.dtype == np.uint8, "stream must be uint8"
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="ascii") as handle:
        for value in stream_u8:
            handle.write(f"{int(value):02X}\n")

    assert path.exists(), "hex file was not written"
    assert path.stat().st_size > 0, "hex file is empty"



def write_binary_stream(path: Path, stream_u8: np.ndarray) -> None:
    """Write raw PPI bytes for optional offline inspection."""

    assert path.suffix == ".bin", "binary path should end in .bin"
    assert stream_u8.dtype == np.uint8, "stream must be uint8"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(stream_u8.tobytes())
    assert path.stat().st_size == stream_u8.size, "binary write size mismatch"



def write_preview(path: Path, samples_int16: np.ndarray, stream_u8: np.ndarray, rows: int) -> None:
    """Write a compact preview for human sanity checking."""

    assert rows >= 0, "preview row count must be non-negative"
    assert samples_int16.shape[1] == CHANNELS, "samples must have four channels"
    preview_count = min(rows, samples_int16.shape[0])
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["sample_index", "ch0", "ch1", "ch2", "ch3", "ppi_bytes"])
        for idx in range(preview_count):
            byte_start = idx * BYTES_PER_SAMPLE_SET
            byte_stop = byte_start + BYTES_PER_SAMPLE_SET
            byte_text = " ".join(f"{int(byte):02X}" for byte in stream_u8[byte_start:byte_stop])
            row = samples_int16[idx]
            writer.writerow([idx, int(row[0]), int(row[1]), int(row[2]), int(row[3]), byte_text])

    assert path.exists(), "preview file was not written"



def write_metadata(path: Path, cfg: GeneratorConfig, derived: DerivedConfig, stream_u8: np.ndarray) -> None:
    """Write metadata that makes HDL/testbench alignment explicit."""

    assert path.suffix == ".json", "metadata path should end in .json"
    assert stream_u8.size == derived.total_stream_bytes, "metadata byte count mismatch"
    path.parent.mkdir(parents=True, exist_ok=True)

    rx_phase_rad = rx_phase_offsets_rad(cfg, derived)
    rx_phase_deg = [math.degrees(float(value)) for value in rx_phase_rad]
    rx_layout = [
        {
            "channel": f"CH{idx + 1}",
            "x_halfwave": RX_X_HALFWAVE[idx],
            "y_halfwave": RX_Y_HALFWAVE[idx],
            "phase_deg": rx_phase_deg[idx],
        }
        for idx in range(CHANNELS)
    ]

    metadata = {
        "purpose": "xsim radar 01..08 ADAR7251 FMCW point-target stimulus",
        "generator": "gen_ppi_hex_3rx.py",
        "outputs": {
            "hex": str(OUTPUT_HEX),
            "bin": str(OUTPUT_BIN),
            "preview_csv": str(OUTPUT_PREVIEW_CSV),
        },
        "model": {
            "adc": "ADAR7251 real signed 16-bit samples, four channels, 8-bit PPI byte stream",
            "fmcw_chirp": "sawtooth up-ramp",
            "range_doppler_model": (
                "range beat is generated inside each chirp; Doppler is generated as "
                "slow-time phase from chirp to chirp"
            ),
            "rx_layout": (
                "staggered '^' of three series-fed patch columns; adjacent phase "
                "centres stepped lambda/2 in both axes (x=along column, "
                "y=across columns), outer pair a full lambda apart; CH4 "
                "unpopulated and mirrored onto CH1"
            ),
            "range_migration_note": (
                "disabled by default to keep the regression target concentrated in one range bin; "
                "set enable_range_migration_within_frame=true to move during the coherent frame"
            ),
        },
        "config": asdict(cfg),
        "derived": asdict(derived),
        "expected_target": {
            "range_bin_float": derived.target_range_bin,
            "range_bin_nearest": int(round(derived.target_range_bin)),
            "doppler_bin_float_positive_away": derived.target_doppler_bin,
            "doppler_bin_nearest_positive_away": int(round(derived.target_doppler_bin)),
            "range_beat_hz": derived.target_range_beat_hz,
            "doppler_hz": derived.target_doppler_hz,
            "velocity_mph": cfg.target_velocity_mps / 0.44704,
            "frame_to_frame_range_step_m": derived.target_frame_range_step_m,
            "rx_phase_offsets_deg_ch1_to_ch4": rx_phase_deg,
        },
        "receiver_layout": rx_layout,
        "byte_order_per_sample_set": [
            "CH1_HI", "CH1_LO",
            "CH2_HI", "CH2_LO",
            "CH3_HI", "CH3_LO",
            "CH4_HI", "CH4_LO",
        ],
        "receiver_mapping": {
            "CH1": "adar7251_ppi_rx.ch0_out",
            "CH2": "adar7251_ppi_rx.ch1_out",
            "CH3": "adar7251_ppi_rx.ch2_out",
            "CH4": "adar7251_ppi_rx.ch3_out",
        },
        "xsim_drive_note": (
            "Verilog testbench should read ppi_stream.hex with $readmemh and drive "
            "one byte on dout[7:0] per sclk_adc edge while data_ready is high."
        ),
    }

    path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    assert path.exists(), "metadata file was not written"


# =============================================================================
# Top-level generation
# =============================================================================


def generate() -> None:
    """Generate all files required by the XSim-only end-to-end test."""

    cfg = GeneratorConfig()
    derived = derive_config(cfg)
    samples_int16 = generate_channel_samples(cfg, derived)
    stream_u8 = pack_all_samples_to_stream(samples_int16)

    assert stream_u8.size == derived.total_stream_bytes, "generated stream size mismatch"
    run_self_check(samples_int16, stream_u8)

    write_hex_stream(OUTPUT_HEX, stream_u8)
    write_binary_stream(OUTPUT_BIN, stream_u8)
    write_metadata(OUTPUT_METADATA, cfg, derived, stream_u8)
    write_preview(OUTPUT_PREVIEW_CSV, samples_int16, stream_u8, cfg.preview_rows)

    print("INFO: ADAR7251 FMCW point-target PPI generator complete")
    print(f"INFO: hex file          : {OUTPUT_HEX.resolve()}")
    print(f"INFO: binary file       : {OUTPUT_BIN.resolve()}")
    print(f"INFO: metadata file     : {OUTPUT_METADATA.resolve()}")
    print(f"INFO: preview file      : {OUTPUT_PREVIEW_CSV.resolve()}")
    print(f"INFO: samples/chirp     : {cfg.samples_per_chirp}")
    print(f"INFO: chirps/frame      : {cfg.chirps_per_frame}")
    print(f"INFO: frames            : {cfg.frames}")
    print(f"INFO: sample sets       : {derived.total_sample_sets}")
    print(f"INFO: stream bytes      : {derived.total_stream_bytes}")
    print(f"INFO: carrier Hz        : {cfg.rf_center_hz:.3f}")
    print(f"INFO: bandwidth Hz      : {cfg.fmcw_bandwidth_hz:.3f}")
    print(f"INFO: chirp time us     : {derived.chirp_period_s * 1e6:.6f}")
    print(f"INFO: target range m    : {cfg.target_initial_range_m:.6f}")
    print(f"INFO: target velocity   : {cfg.target_velocity_mps:.6f} m/s")
    print(f"INFO: expected range bin: {derived.target_range_bin:.3f}")
    print(f"INFO: expected dopp bin : {derived.target_doppler_bin:.3f}")
    print("INFO: self-check        : PASS")


if __name__ == "__main__":
    generate()
