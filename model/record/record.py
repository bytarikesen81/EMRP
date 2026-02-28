#!/usr/bin/env python3
"""
MPU6050 raw int16 -> window dataset generator (CSV).

Final fixed parameters (as requested):
  A  = 96    (window length)
  S  = 48    (stride)
  Fs = 100   (target sampling rate in Hz)
  D  = 150   (recording duration per class in seconds)

Outputs (one-time run):
  Move.csv
  Shake.csv
  Rest.csv

Each row is one window:
  label,window_index,x0_ax,x0_ay,...,x95_gz

Where each x*_axis is a signed int16 raw value from MPU6050:
  ax, ay, az, gx, gy, gz

All prints/comments are in English.
"""

import csv
import os
import random
import time
from typing import List, Tuple

from smbus2 import SMBus

# -------------------------
# Fixed parameters (FINAL)
# -------------------------
A = 96            # window length
S = 48            # stride
FS = 100.0        # target sampling rate (Hz)
D = 150.0         # seconds per class recording
K = 200           # windows per class (kept at your earlier plan)

SEED = 42

# -------------------------
# MPU6050 registers/address
# -------------------------
MPU_ADDR = 0x68        # AD0=GND -> 0x68 (most common)
REG_WHO_AM_I = 0x75
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B

AXES = ["ax", "ay", "az", "gx", "gy", "gz"]


def to_int16(h: int, l: int) -> int:
    v = (h << 8) | l
    return v - 65536 if (v & 0x8000) else v


class MPU6050:
    def __init__(self, bus_id: int = 1, addr_7bit: int = MPU_ADDR):
        self.bus = SMBus(bus_id)
        self.addr = addr_7bit

    def close(self) -> None:
        self.bus.close()

    def init(self) -> None:
        who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if who != 0x68:
            raise RuntimeError(
                f"Unexpected WHO_AM_I: 0x{who:02X}. Check wiring/address (0x68 vs 0x69)."
            )
        # Wake up (clear sleep bit)
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def read_raw6(self) -> Tuple[int, int, int, int, int, int]:
        # Burst read 14 bytes: accel(6) + temp(2) + gyro(6). Temperature is ignored.
        b = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 14)
        ax = to_int16(b[0], b[1])
        ay = to_int16(b[2], b[3])
        az = to_int16(b[4], b[5])
        gx = to_int16(b[8], b[9])
        gy = to_int16(b[10], b[11])
        gz = to_int16(b[12], b[13])
        return ax, ay, az, gx, gy, gz


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def build_header() -> List[str]:
    hdr = ["label", "window_index"]
    for t in range(A):
        for a in AXES:
            hdr.append(f"x{t}_{a}")
    return hdr


def measure_effective_hz(mpu: MPU6050, seconds: float = 2.0) -> float:
    """
    Measure effective read speed by reading as fast as possible for a short time.
    This is only a sanity check.
    """
    count = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        _ = mpu.read_raw6()
        count += 1
    dt = time.monotonic() - t0
    return (count / dt) if dt > 0 else 0.0


def record_samples(mpu: MPU6050) -> List[Tuple[int, int, int, int, int, int]]:
    """
    Record samples for D seconds at approximately FS Hz.
    """
    dt = 1.0 / FS
    t_end = time.monotonic() + D
    out: List[Tuple[int, int, int, int, int, int]] = []
    missed = 0
    
    zero6 = 0

    while time.monotonic() < t_end:
        t_read_start = time.monotonic()
        try:
            s = out.append(mpu.read_raw6())
            if s == (0,0,0,0,0,0):
                zero6 += 1
        except Exception:
            missed += 1

        elapsed = time.monotonic() - t_read_start
        sleep_s = dt - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    if missed:
        print(f"[WARN] {missed} read errors occurred during recording.")
        
    if len(out) == 0:
        raise RuntimeError("No samples recorded. Check I2C/wiring/power.")
    
    zero_ratio = zero6/len(out)
    if zero_ratio > 0.07:
        raise RuntimeError(f"Too many all-zero samples ({zero_ratio*100:.1f}%)."
                            "This indicates an I2C/power glitch. Not writing this recording.")
    return out


def make_windows(samples: List[Tuple[int, int, int, int, int, int]]) -> List[List[int]]:
    """
    Convert samples into flattened windows of length A*6 using stride S.
    """
    if len(samples) < A:
        return []

    windows: List[List[int]] = []
    last_start = len(samples) - A
    for start in range(0, last_start + 1, S):
        flat: List[int] = []
        for i in range(A):
            flat.extend(samples[start + i])
        windows.append(flat)
    return windows


def pick_exact_k(windows: List[List[int]], label: str) -> List[List[int]]:
    """
    Randomly pick exactly K windows from the collected list.
    """
    if len(windows) < K:
        raise RuntimeError(
            f"Not enough windows for {label}: have {len(windows)}, need {K}. "
            f"Increase D or reduce S."
        )
    rnd = random.Random(SEED)
    idxs = list(range(len(windows)))
    rnd.shuffle(idxs)
    return [windows[i] for i in idxs[:K]]


def write_csv(path: str, label: str, windows: List[List[int]]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(build_header())
        for wi, flat in enumerate(windows):
            w.writerow([label, wi] + flat)


def main() -> None:
    outdir = "imu_raw_ds_final"
    ensure_dir(outdir)

    print("[INFO] Final parameters:")
    print(f"[INFO] A={A}, S={S}, Fs={FS} Hz, D={D} s, K={K} windows/class")
    print("[INFO] Output files: Move.csv, Shake.csv, Rest.csv\n")

    mpu = MPU6050(bus_id=1, addr_7bit=MPU_ADDR)
    try:
        print("[INFO] Initializing MPU6050...")
        mpu.init()
        print("[OK] MPU6050 initialized.")

        eff = measure_effective_hz(mpu, seconds=2.0)
        print(f"[INFO] Effective read speed (no pacing): ~{eff:.1f} samples/s\n")

        classes = [
            ("Move", "Move.csv", "Move the device back and forth (clear movement)."),
            ("Shake", "Shake.csv", "Gently shake/rotate around x/y/z axes (clear shake)."),
            ("Rest", "Rest.csv", "Leave the device free (no intentional movement)."),
        ]

        for label, fname, instruction in classes:
            input(f"[READY] Press ENTER to start '{label}'. Instruction: {instruction} ")
            print(f"[REC] Recording '{label}' for {D:.1f} seconds at ~{FS:.1f} Hz...")
            #Preview 5 samples before recording
            print("[INFO] previewing the first 5 raw samples before recording....")
            for _ in range(5):
                print("{SAMPLE}", mpu.read_raw6())
                time.sleep(0.02)
            ##################################
            samples = record_samples(mpu)
            print(f"[INFO] Collected samples: {len(samples)}")

            windows = make_windows(samples)
            print(f"[INFO] Extracted windows (before selection): {len(windows)}")

            selected = pick_exact_k(windows, label=label)
            out_path = os.path.join(outdir, fname)
            write_csv(out_path, label=label, windows=selected)
            print(f"[OK] Wrote {K} windows to: {out_path}\n")

        print("[DONE] Dataset generation completed.")
        print(f"[DONE] Output directory: {outdir}")

    finally:
        mpu.close()


if __name__ == "__main__":
    main()
