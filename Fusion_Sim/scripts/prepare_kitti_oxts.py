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
    """
    Parse KITTI timestamp string (YYYY-MM-DD HH:MM:SS.sssssssss)
    into nanoseconds since epoch (int64).
    """
    base, frac = ts.split(".")
    frac = (frac + "000000000")[:9]  # pad/truncate to 9 digits
    ns = int(frac)

    # Parse base to seconds since epoch
    base_epoch = pd.Timestamp(base).to_datetime64().astype("datetime64[s]").astype(np.int64)

    return base_epoch * 1_000_000_000 + ns

def find_oxts_paths(input_dir: str) -> Tuple[str, str]:
    """
    Given an input path, determine the correct paths for timestamps.txt and data/.
    Accept either:
      - input_dir contains 'timestamps.txt' and 'data/' (preferred), OR
      - input_dir/oxts contains them.
    Returns (timestamps_file, data_dir)
    Raises FileNotFoundError with helpful message if neither is valid.
    """
    # Candidate 1: input_dir itself
    t1 = os.path.join(input_dir, "timestamps.txt")
    d1 = os.path.join(input_dir, "data")

    if os.path.isfile(t1) and os.path.isdir(d1):
        return t1, d1

    # Candidate 2: input_dir/oxts
    t2 = os.path.join(input_dir, "oxts", "timestamps.txt")
    d2 = os.path.join(input_dir, "oxts", "data")
    if os.path.isfile(t2) and os.path.isdir(d2):
        return t2, d2

    # Candidate 3: if user passed parent and there is exactly one oxts* folder inside, try it
    # (useful if input_dir points to category dir by mistake)
    candidates = [p for p in glob.glob(os.path.join(input_dir, "*")) if os.path.isdir(p)]
    for c in candidates:
        t3 = os.path.join(c, "timestamps.txt")
        d3 = os.path.join(c, "data")
        if os.path.isfile(t3) and os.path.isdir(d3):
            return t3, d3

    tried = [
        f"1) {t1} + {d1}",
        f"2) {t2} + {d2}",
        f"3) searched immediate subfolders of {input_dir}"
    ]
    raise FileNotFoundError(
        f"Could not find timestamps.txt and data/ in the provided input_dir. "
        f"Tried: {', '.join(tried)}. Please pass the sequence folder (which contains timestamps.txt and data/) "
        f"or the oxts subfolder."
    )

def prepare_kitti_sequence(input_dir: str) -> pd.DataFrame:
    """
    Load OXTS sequence into a DataFrame.
    input_dir may be either the sequence folder (containing timestamps.txt and data/)
    or an oxts/ subfolder.
    """
    timestamps_file, oxts_data_dir = find_oxts_paths(input_dir)

    with open(timestamps_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]

    files = sorted(glob.glob(os.path.join(oxts_data_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError(
            f"Number of OXTS files ({len(files)}) and timestamps ({len(timestamps)}) differ! "
            f"timestamps_file={timestamps_file}, oxts_data_dir={oxts_data_dir}"
        )

    rows = []
    for ts, fpath in zip(timestamps, files):
        data = load_oxts_file(fpath)
        data["timestamp"] = ts
        rows.append(data)

    df = pd.DataFrame(rows)

    # Compute relative time as before
    ns_times = np.array([parse_timestamp_ns(ts) for ts in df["timestamp"]], dtype=np.int64)
    rel_ns = ns_times - ns_times[0]  # relative nanoseconds (int64)
    df["rel_time"] = [f"{ns/1e9:.9f}" for ns in rel_ns]

    return df

def inject_noise(df: pd.DataFrame, error_rate: float = 0.03, magnitude: float = 3.0, seed: int = None) -> pd.DataFrame:
    """
    Randomly inject outlier values into up to error_rate fraction of numeric cells,
    spreading them evenly across numeric columns but excluding timestamp-like columns.
    """
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    noisy_df = df.copy(deep=True)

    # Build list of columns eligible for corruption:
    # numeric columns AND not timestamp / rel_time
    excluded_names = {"timestamp", "rel_time"}
    numeric_cols = [c for c in noisy_df.select_dtypes(include=[np.number]).columns if c not in excluded_names]

    if len(numeric_cols) == 0:
        # Fallback: if no numeric columns detected (unlikely), try all except excluded names
        numeric_cols = [c for c in noisy_df.columns if c not in excluded_names]

    n_rows = len(noisy_df)
    n_cols = len(numeric_cols)
    if n_cols == 0 or n_rows == 0:
        return noisy_df

    total_cells = n_rows * n_cols
    n_errors = max(1, int(total_cells * error_rate))

    # Distribute errors evenly per column
    base_per_col = n_errors // n_cols
    remainder = n_errors % n_cols

    # Shuffle columns to randomize where the extra remainder errors go
    cols_shuffled = numeric_cols.copy()
    random.shuffle(cols_shuffled)

    for idx, col in enumerate(cols_shuffled):
        # give one extra if within remainder
        n_for_col = base_per_col + (1 if idx < remainder else 0)
        if n_for_col <= 0:
            continue

        std = noisy_df[col].std()
        if not std or np.isnan(std):
            std = 1.0

        # pick unique row indices for this column
        n_for_col = min(n_for_col, n_rows)
        row_idxs = np.random.choice(noisy_df.index, size=n_for_col, replace=False)

        for r in row_idxs:
            val = noisy_df.at[r, col]
            perturb = np.random.normal(0, magnitude * std)
            # occasional extreme spike
            if random.random() < 0.2:
                perturb *= random.choice([-5, 5])
            noisy_df.at[r, col] = val + perturb

    return noisy_df

def main():
    parser = argparse.ArgumentParser(description="Prepare OXTS CSV files with optional corruption")
    parser.add_argument("--input_dir", required=True,
                        help="Path to sequence folder (or oxts subfolder). Must contain timestamps.txt and data/")
    parser.add_argument("--output_dir", required=True, help="Directory to write CSV files to (will be created)")
    parser.add_argument("--total", type=int, default=5, help="Total number of CSV files to produce (default=5)")
    parser.add_argument("--n_corrupt", type=int, default=0, help="Number of corrupted CSV files (default=0)")
    parser.add_argument("--error_rate", type=float, default=0.03, help="Fraction of numeric values to corrupt (default=0.03)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility (default=42)")
    parser.add_argument("--magnitude", type=float, default=3.0, help="Noise magnitude in units of column std dev (default=3.0)")
    args = parser.parse_args()

    if args.n_corrupt > args.total:
        raise ValueError("Number of corrupted files cannot exceed total number of files.")

    os.makedirs(args.output_dir, exist_ok=True)

    # Load base dataframe
    base_df = prepare_kitti_sequence(args.input_dir)

    # Decide which indices will be corrupted (deterministic w.r.t seed)
    random.seed(args.seed)
    corrupted_indices = set(random.sample(range(args.total), args.n_corrupt))

    for i in range(args.total):
        if i in corrupted_indices:
            df_out = inject_noise(base_df, error_rate=args.error_rate, magnitude=args.magnitude, seed=(args.seed + i))
            status = "corrupted"
        else:
            df_out = base_df
            status = "clean"
        out_path = os.path.join(args.output_dir, f"oxts{i}.csv")
        df_out.to_csv(out_path, index=False)
        print(f"Saved {len(df_out)} entries to {out_path} ({status})")

if __name__ == "__main__":
    main()

