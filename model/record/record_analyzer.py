#!/usr/bin/env python3
"""
Dataset quality analyzer for IMU windows stored in CSV format.

Expected CSV columns:
- label, window_index (optional but commonly present)
- x0_ax, x0_ay, x0_az, x0_gx, x0_gy, x0_gz
- ...
- x(A-1)_ax ... x(A-1)_gz

This script prints:
- Saturation percentage (near int16 limits)
- Per-axis min/max/mean/std/mean_abs
- Mean magnitude of accel and gyro vectors
- Class-to-class ratios based on gyro magnitude mean
"""

import argparse
import numpy as np
import pandas as pd

AXES = ["ax", "ay", "az", "gx", "gy", "gz"]

def load_windows(csv_path: str, A: int = 96) -> np.ndarray:
    df = pd.read_csv(csv_path)

    # Validate columns quickly
    missing = []
    for t in range(A):
        for a in AXES:
            c = f"x{t}_{a}"
            if c not in df.columns:
                missing.append(c)
    if missing:
        raise ValueError(f"Missing columns in {csv_path}: {missing[:10]} ... (total {len(missing)})")

    arr = np.zeros((df.shape[0], A, 6), dtype=np.int16)
    for t in range(A):
        for j, a in enumerate(AXES):
            col = f"x{t}_{a}"
            arr[:, t, j] = df[col].astype(np.int16).values
    return arr

def compute_stats(arr_i16: np.ndarray, sat_threshold: int = 32760) -> dict:
    x = arr_i16.astype(np.int32).reshape(-1, 6)  # (num_windows*A, 6)

    stats = {}
    stats["total_samples"] = x.shape[0]
    stats["total_values"] = x.shape[0] * x.shape[1]

    stats["min"] = x.min(axis=0)
    stats["max"] = x.max(axis=0)
    stats["mean"] = x.mean(axis=0)
    stats["std"] = x.std(axis=0)
    stats["mean_abs"] = np.abs(x).mean(axis=0)

    acc = x[:, 0:3].astype(np.float64)
    gyr = x[:, 3:6].astype(np.float64)
    stats["acc_mag_mean"] = np.linalg.norm(acc, axis=1).mean()
    stats["gyr_mag_mean"] = np.linalg.norm(gyr, axis=1).mean()

    # Saturation detection
    sat_mask = (np.abs(x) >= sat_threshold) | (x == -32768) | (x == 32767)
    stats["sat_count"] = int(sat_mask.sum())
    stats["sat_pct"] = float(sat_mask.mean() * 100.0)

    sat_axis = sat_mask.sum(axis=0)
    stats["sat_count_axis"] = sat_axis.astype(int)
    stats["sat_pct_axis"] = (sat_axis / x.shape[0] * 100.0)

    return stats

def print_report(name: str, st: dict):
    print(f"\n=== {name} ===")
    print(f"Total samples: {st['total_samples']}  |  Total values: {st['total_values']}")
    print(f"Saturation: {st['sat_pct']:.3f}%  ({st['sat_count']}/{st['total_values']})")

    print("\nPer-axis stats (min / max / mean / std / mean_abs / sat%):")
    for i, a in enumerate(AXES):
        print(
            f"  {a}: {int(st['min'][i])} / {int(st['max'][i])} / "
            f"{st['mean'][i]:.1f} / {st['std'][i]:.1f} / {st['mean_abs'][i]:.1f} / "
            f"{st['sat_pct_axis'][i]:.3f}%"
        )

    print("\nVector magnitude means:")
    print(f"  accel |a| mean: {st['acc_mag_mean']:.1f}")
    print(f"  gyro  |g| mean: {st['gyr_mag_mean']:.1f}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--move", required=True, help="Path to Move.csv")
    ap.add_argument("--shake", required=True, help="Path to Shake.csv (use final one, e.g., Shake_New.csv)")
    ap.add_argument("--rest", required=True, help="Path to Rest.csv")
    ap.add_argument("--A", type=int, default=96, help="Window length (default: 96)")
    ap.add_argument("--sat_threshold", type=int, default=32760, help="Saturation threshold near int16 limits")
    args = ap.parse_args()

    datasets = {
        "Move": load_windows(args.move, args.A),
        "Shake": load_windows(args.shake, args.A),
        "Rest": load_windows(args.rest, args.A),
    }

    stats = {}
    for name, arr in datasets.items():
        stats[name] = compute_stats(arr, args.sat_threshold)
        print_report(name, stats[name])

    print("\n=== Class-to-class ratios (gyro magnitude mean) ===")
    def ratio(a, b): 
        return stats[a]["gyr_mag_mean"] / (stats[b]["gyr_mag_mean"] + 1e-9)

    print(f"Shake / Rest: {ratio('Shake','Rest'):.2f}x")
    print(f"Move  / Rest: {ratio('Move','Rest'):.2f}x")
    print(f"Shake / Move: {ratio('Shake','Move'):.2f}x")

if _name_ == "_main_":
    main()