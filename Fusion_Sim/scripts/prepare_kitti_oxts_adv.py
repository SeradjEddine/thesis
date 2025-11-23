#!/usr/bin/env python3
"""
prepare_kitti_oxts.py

Usage:
    python3 prepare_kitti_oxts.py --input_dir <path-to-sequence-or-oxts-subdir> --output_dir <outdir> [--total N] [--n_corrupt M] [--error_rate R] [--seed S]
"""

import os
import glob
import pandas as pd
import numpy as np
import random
import argparse
from typing import Tuple

# ----------------------------------------------------------------------
#  🧩 1. CATEGORY DEFINITIONS & NOISE MAGNITUDE SCALE FACTORS
# ----------------------------------------------------------------------
#
#  You can tune these values freely. They multiply the *std-based magnitude*.
#
#  Recommended initial factors (realistic rough modeling):
#   - GPS:         0.2–0.4   (GPS in meters; keep noise small)
#   - Orientation: 0.5–1.0   (Yaw drift behaves differently but this is OK)
#   - Velocity:    1.0       (KITTI velocity variance already moderate)
#   - Accel:       1.0–1.5   (IMU accel noisy)
#   - Gyro:        1.0–1.5   (gyro even noisier)
#
# ----------------------------------------------------------------------

NOISE_SCALE = {
    "gps": 0.250,
    "orientation": 1.0,
    "velocity": 1.0,
    "accel": 1.2,
    "gyro": 1.2,
}

GPS_COLS = ["lat", "lon", "alt"]
ORIENT_COLS = ["roll", "pitch", "yaw"]
VELOCITY_COLS = ["vn", "ve", "vf", "vl", "vu"]
ACCEL_COLS = ["ax", "ay", "az", "af", "al", "au"]
GYRO_COLS = ["wx", "wy", "wz", "wf", "wl", "wu"]

def classify_column(col: str) -> str:
    """Return category name for noise scaling."""
    if col in GPS_COLS:
        return "gps"
    if col in ORIENT_COLS:
        return "orientation"
    if col in VELOCITY_COLS:
        return "velocity"
    if col in ACCEL_COLS:
        return "accel"
    if col in GYRO_COLS:
        return "gyro"
    return None  # default category = no scaling override


# ----------------------------------------------------------------------
#  ORIGINAL SCRIPT BELOW (UNCHANGED EXCEPT INJECTOR)
# ----------------------------------------------------------------------

def load_oxts_file(file_path: str) -> dict:
    """Parse a single OXTS line into a dict of useful values."""
    with open(file_path, 'r') as f:
        line = f.readline().strip()
    values = [float(x) for x in line.split()]
    return {
        "lat": values[0], "lon": values[1], "alt": values[2],
        "roll": values[3], "pitch": values[4], "yaw": values[5],
        "vn": values[6], "ve": values[7], "vf": values[8],
        "vl": values[9], "vu": values[10],
        "ax": values[11], "ay": values[12], "az": values[13],
        "af": values[14], "al": values[15], "au": values[16],
        "wx": values[17], "wy": values[18], "wz": values[19],
        "wf": values[20], "wl": values[21], "wu": values[22],
    }

def parse_timestamp_ns(ts: str) -> int:
    base, frac = ts.split(".")
    frac = (frac + "000000000")[:9]
    ns = int(frac)
    base_epoch = pd.Timestamp(base).to_datetime64().astype("datetime64[s]").astype(np.int64)
    return base_epoch * 1_000_000_000 + ns

def find_oxts_paths(input_dir: str) -> Tuple[str, str]:
    t1 = os.path.join(input_dir, "timestamps.txt")
    d1 = os.path.join(input_dir, "data")
    if os.path.isfile(t1) and os.path.isdir(d1):
        return t1, d1

    t2 = os.path.join(input_dir, "oxts", "timestamps.txt")
    d2 = os.path.join(input_dir, "oxts", "data")
    if os.path.isfile(t2) and os.path.isdir(d2):
        return t2, d2

    candidates = [p for p in glob.glob(os.path.join(input_dir, "*")) if os.path.isdir(p)]
    for c in candidates:
        t3 = os.path.join(c, "timestamps.txt")
        d3 = os.path.join(c, "data")
        if os.path.isfile(t3) and os.path.isdir(d3):
            return t3, d3

    raise FileNotFoundError("Could not find timestamps.txt and data/.")

def prepare_kitti_sequence(input_dir: str) -> pd.DataFrame:
    timestamps_file, oxts_data_dir = find_oxts_paths(input_dir)
    with open(timestamps_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]

    files = sorted(glob.glob(os.path.join(oxts_data_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError("Mismatch files vs timestamps")

    rows = []
    for ts, fpath in zip(timestamps, files):
        data = load_oxts_file(fpath)
        data["timestamp"] = ts
        rows.append(data)

    df = pd.DataFrame(rows)
    ns_times = np.array([parse_timestamp_ns(ts) for ts in df["timestamp"]], dtype=np.int64)
    rel_ns = ns_times - ns_times[0]
    df["rel_time"] = [f"{ns/1e9:.9f}" for ns in rel_ns]
    return df


# ----------------------------------------------------------------------
#  🛠️ 2. NEW INJECTOR WITH PER-CATEGORY NOISE CONTROL
# ----------------------------------------------------------------------
def inject_noise(df: pd.DataFrame, error_rate: float = 0.03, magnitude: float = 3.0, seed: int = None) -> pd.DataFrame:

    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    noisy_df = df.copy(deep=True)

    excluded_names = {"timestamp", "rel_time"}
    numeric_cols = [c for c in noisy_df.select_dtypes(include=[np.number]).columns if c not in excluded_names]

    n_rows = len(noisy_df)
    if len(numeric_cols) == 0 or n_rows == 0:
        return noisy_df

    total_cells = n_rows * len(numeric_cols)
    n_errors = max(1, int(total_cells * error_rate))

    base_per_col = n_errors // len(numeric_cols)
    remainder = n_errors % len(numeric_cols)

    cols_shuffled = numeric_cols.copy()
    random.shuffle(cols_shuffled)

    for idx, col in enumerate(cols_shuffled):
        n_for_col = base_per_col + (1 if idx < remainder else 0)
        if n_for_col <= 0:
            continue

        std = noisy_df[col].std()
        if not std or np.isnan(std):
            std = 1.0

        # --- CATEGORY-BASED NOISE SCALING ---
        category = classify_column(col)
        scale = NOISE_SCALE.get(category, 1.0)  # default 1.0 if unclassified
        effective_mag = magnitude * scale

        row_idxs = np.random.choice(noisy_df.index, size=min(n_for_col, n_rows), replace=False)

        for r in row_idxs:
            val = noisy_df.at[r, col]

            perturb = np.random.normal(0, effective_mag * std)

            # optional extreme spikes
            if random.random() < 0.2:
                perturb *= random.choice([-5, 5])

            noisy_df.at[r, col] = val + perturb

    return noisy_df


# ----------------------------------------------------------------------
#  MAIN (unchanged)
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Prepare OXTS CSV files with optional corruption")
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--total", type=int, default=5)
    parser.add_argument("--n_corrupt", type=int, default=0)
    parser.add_argument("--error_rate", type=float, default=0.03)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--magnitude", type=float, default=3.0)
    args = parser.parse_args()

    if args.n_corrupt > args.total:
        raise ValueError("n_corrupt > total")

    os.makedirs(args.output_dir, exist_ok=True)

    base_df = prepare_kitti_sequence(args.input_dir)

    random.seed(args.seed)
    corrupted_indices = set(random.sample(range(args.total), args.n_corrupt))

    for i in range(args.total):
        if i in corrupted_indices:
            df_out = inject_noise(base_df,
                                  error_rate=args.error_rate,
                                  magnitude=args.magnitude,
                                  seed=(args.seed + i))
            status = "corrupted"
        else:
            df_out = base_df
            status = "clean"

        out_path = os.path.join(args.output_dir, f"oxts{i}.csv")
        df_out.to_csv(out_path, index=False)
        print(f"Saved {len(df_out)} entries to {out_path} ({status})")


if __name__ == "__main__":
    main()
