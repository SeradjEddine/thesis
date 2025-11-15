#!/usr/bin/env python3
"""
prepare_kitti_oxts.py

Usage:
    python3 prepare_kitti_oxts.py \
        --input_dir <path-to-sequence-or-oxts-subdir> \
        --output_dir <outdir> \
        [--total N] [--n_corrupt M] [--error_rate R] [--seed S] [--run_id ID]
"""

import os
import glob
import pandas as pd
import numpy as np
import random
import argparse
import csv
from datetime import datetime
from typing import Tuple


# ----------------------------
# Utility functions
# ----------------------------

def load_oxts_file(file_path: str) -> dict:
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

    raise FileNotFoundError(f"Could not find timestamps.txt and data/ under {input_dir}")

def prepare_kitti_sequence(input_dir: str) -> pd.DataFrame:
    timestamps_file, oxts_data_dir = find_oxts_paths(input_dir)
    with open(timestamps_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]
    files = sorted(glob.glob(os.path.join(oxts_data_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError(f"OXTS files ({len(files)}) and timestamps ({len(timestamps)}) mismatch")
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


def inject_noise(df: pd.DataFrame, error_rate: float = 0.03, seed: int = None) -> pd.DataFrame:
    """Apply random noise to numeric columns."""
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)
    noisy_df = df.copy(deep=True)
    excluded = {"timestamp", "rel_time"}
    numeric_cols = [c for c in noisy_df.select_dtypes(include=[np.number]).columns if c not in excluded]
    if not numeric_cols:
        numeric_cols = [c for c in noisy_df.columns if c not in excluded]
    n_rows, n_cols = len(noisy_df), len(numeric_cols)
    if n_rows == 0 or n_cols == 0:
        return noisy_df
    total_cells = n_rows * n_cols
    n_errors = max(1, int(total_cells * error_rate))
    row_idxs = np.random.choice(noisy_df.index, size=n_errors, replace=True)
    col_idxs = np.random.choice(len(numeric_cols), size=n_errors, replace=True)
    for r, ci in zip(row_idxs, col_idxs):
        col = numeric_cols[ci]
        val = noisy_df.at[r, col]
        perturb = np.random.normal(0, 0.03 * (noisy_df[col].std() or 1.0))
        noisy_df.at[r, col] = val + perturb
    return noisy_df


# ----------------------------
# Local logging
# ----------------------------

def log_local_metadata(output_dir: str, run_id: str, n_total: int, n_corrupt: int, error_rate: float, seed: int):
    """Append a one-line metadata record to the set's local log."""
    log_path = os.path.join(output_dir, "set_log.csv")
    file_exists = os.path.isfile(log_path)
    with open(log_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "run_id", "n_total", "n_corrupt", "error_rate", "seed"])
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        writer.writerow([ts, run_id, n_total, n_corrupt, error_rate, seed])


# ----------------------------
# Main entry
# ----------------------------

def main():
    parser = argparse.ArgumentParser(description="Prepare OXTS CSV files with optional corruption")
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--total", type=int, default=5)
    parser.add_argument("--n_corrupt", type=int, default=0)
    parser.add_argument("--error_rate", type=float, default=0.03)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--run_id", type=str, default="0001", help="Unique identifier for this generation run")
    args = parser.parse_args()

    if args.n_corrupt > args.total:
        raise ValueError("Number of corrupted files cannot exceed total number of files.")

    os.makedirs(args.output_dir, exist_ok=True)
    base_df = prepare_kitti_sequence(args.input_dir)

    random.seed(args.seed)
    corrupted_indices = set(random.sample(range(args.total), args.n_corrupt))

    for i in range(args.total):
        if i in corrupted_indices:
            df_out = inject_noise(base_df, error_rate=args.error_rate, seed=(args.seed + i))
            status = "corrupted"
        else:
            df_out = base_df
            status = "clean"
        out_path = os.path.join(args.output_dir, f"oxts{i}.csv")
        df_out.to_csv(out_path, index=False)
        print(f"Saved {len(df_out)} entries to {out_path} ({status})")

    # --- Log locally (one line per run) ---
    log_local_metadata(args.output_dir, args.run_id, args.total, args.n_corrupt, args.error_rate, args.seed)
    print(f"Logged metadata to {os.path.join(args.output_dir, 'set_log.csv')}")


if __name__ == "__main__":
    main()

