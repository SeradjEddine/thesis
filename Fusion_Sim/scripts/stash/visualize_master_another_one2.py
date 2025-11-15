#!/usr/bin/env python3
"""
Advanced KITTI OXTS preparation script with structured error-injection config.

Usage:
    python3 prepare_kitti_oxts_adv.py \
        --input_dir <path> \
        --output_dir <path> \
        --total N \
        --n_corrupt M \
        --config error_config.yaml \
        [--seed S]
"""

import os
import yaml
import glob
import argparse
import numpy as np
import pandas as pd
import random
from typing import Dict, Any, Tuple

# --------------------------------------------------------
#  OXTS LOADING HELPERS
# --------------------------------------------------------

def load_oxts_file(file_path: str) -> dict:
    with open(file_path, 'r') as f:
        vals = [float(x) for x in f.readline().split()]
    return {
        "lat": vals[0], "lon": vals[1], "alt": vals[2],
        "roll": vals[3], "pitch": vals[4], "yaw": vals[5],
        "vn": vals[6], "ve": vals[7], "vf": vals[8],
        "vl": vals[9], "vu": vals[10],
        "ax": vals[11], "ay": vals[12], "az": vals[13],
        "af": vals[14], "al": vals[15], "au": vals[16],
        "wx": vals[17], "wy": vals[18], "wz": vals[19],
        "wf": vals[20], "wl": vals[21], "wu": vals[22],
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

    # Scan subfolders
    for c in glob.glob(os.path.join(input_dir, "*")):
        t3 = os.path.join(c, "timestamps.txt")
        d3 = os.path.join(c, "data")
        if os.path.isfile(t3) and os.path.isdir(d3):
            return t3, d3

    raise FileNotFoundError("Could not locate OXTS structure")

def prepare_kitti_sequence(input_dir: str) -> pd.DataFrame:
    timestamps_file, oxts_dir = find_oxts_paths(input_dir)

    timestamps = [line.strip() for line in open(timestamps_file)]
    files = sorted(glob.glob(os.path.join(oxts_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError("Mismatch between timestamps and data/*.txt files")

    rows = []
    for ts, fp in zip(timestamps, files):
        d = load_oxts_file(fp)
        d["timestamp"] = ts
        rows.append(d)

    df = pd.DataFrame(rows)

    # Relative time
    ns = np.array([parse_timestamp_ns(t) for t in df["timestamp"].values], dtype=np.int64)
    rel = (ns - ns[0]) / 1e9
    df["rel_time"] = rel

    return df

# --------------------------------------------------------
#  ERROR INJECTION ENGINE
# --------------------------------------------------------

def apply_bias_drift(series, cfg, dt):
    bias = np.random.normal(0, cfg["bias_std"])
    drift = cfg["drift_per_sec"] * dt
    return series + bias + drift

def apply_jump(series, cfg):
    jump = (np.random.randn() * cfg["magnitude"])
    return series + jump

def apply_dropout(series, cfg):
    r = random.random()
    if r < cfg["blackout_prob"]:
        return np.nan
    elif r < cfg["blackout_prob"] + cfg["freeze_prob"]:
        return series  # freeze
    else:
        return series + np.random.normal(0, cfg["noise_std"])

def apply_saturation(series, cfg):
    lim = cfg["saturation_level"]
    return np.clip(series, -lim, lim)

def apply_bias_walk(series, state, cfg):
    # random-walk update
    state["offset"] += np.random.normal(0, cfg["walk_std"])
    return series + state["offset"]

def apply_spike(series, cfg):
    return series * cfg["multiplier"]

def apply_catastrophic(series, active, cfg):
    if active["on"]:
        return series * cfg["multiplier"]
    return series

# --------------------------------------------------------
#  MAIN DRIVER FOR PER-FIELD INJECTION
# --------------------------------------------------------

def inject_errors(df: pd.DataFrame, cfg: Dict[str, Any], seed: int) -> pd.DataFrame:
    random.seed(seed)
    np.random.seed(seed)

    out = df.copy()

    # Precompute dt between samples
    dt = np.mean(np.diff(out["rel_time"].values))

    # Track random-walk IMU bias
    imu_bias_state = {"offset": 0.0}

    # Catastrophic event controller
    catastrophic_state = {"on": False, "remaining": 0}

    for idx in range(len(out)):
        row = out.iloc[idx]

        # trigger catastrophic events
        cat_cfg = cfg["catastrophic"]
        if cat_cfg["enable"] and random.random() < cat_cfg["probability"]:
            catastrophic_state["on"] = True
            catastrophic_state["remaining"] = int(cat_cfg["duration_sec"] / dt)

        if catastrophic_state["on"]:
            catastrophic_state["remaining"] -= 1
            if catastrophic_state["remaining"] <= 0:
                catastrophic_state["on"] = False

        # GPS Models -----------------------------------------
        gps_fields = ["lat", "lon", "alt"]
        for f in gps_fields:
            val = row[f]

            # cat
            val = apply_catastrophic(val, catastrophic_state, cat_cfg)

            # bias drift
            c = cfg["gps"]["bias_drift"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_bias_drift(val, c, row["rel_time"])

            # jumps
            c = cfg["gps"]["jumps"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_jump(val, c)

            # dropout
            c = cfg["gps"]["dropout"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_dropout(val, c)

            out.at[idx, f] = val

        # IMU Models ----------------------------------------
        imu_fields = ["ax", "ay", "az", "wx", "wy", "wz"]
        for f in imu_fields:
            val = row[f]

            # catastrophic
            val = apply_catastrophic(val, catastrophic_state, cat_cfg)

            # saturation
            c = cfg["imu"]["saturation"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_saturation(val, c)

            # bias walk
            c = cfg["imu"]["bias_walk"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_bias_walk(val, imu_bias_state, c)

            # spikes
            c = cfg["imu"]["spikes"]
            if c["enable"] and random.random() < c["probability"]:
                val = apply_spike(val, c)

            out.at[idx, f] = val

    return out

# --------------------------------------------------------
#  MAIN SCRIPT
# --------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--total", type=int, default=5)
    parser.add_argument("--n_corrupt", type=int, default=0)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int)

    args = parser.parse_args()

    # Load config
    cfg = yaml.safe_load(open(args.config))

    if args.seed is not None:
        cfg["defaults"]["seed"] = args.seed

    os.makedirs(args.output_dir, exist_ok=True)

    base_df = prepare_kitti_sequence(args.input_dir)

    # Select corrupt indices
    random.seed(cfg["defaults"]["seed"])
    corrupt_ids = set(random.sample(range(args.total), args.n_corrupt))

    # Main generation
    for i in range(args.total):
        if i in corrupt_ids:
            df_mod = inject_errors(base_df, cfg, seed=cfg["defaults"]["seed"] + i)
            status = "CORRUPT"
        else:
            df_mod = base_df
            status = "CLEAN"

        out_path = os.path.join(args.output_dir, f"oxts{i}.csv")
        df_mod.to_csv(out_path, index=False)
        print(f"[SAVED] {out_path}  ({status})")


if __name__ == "__main__":
    main()

