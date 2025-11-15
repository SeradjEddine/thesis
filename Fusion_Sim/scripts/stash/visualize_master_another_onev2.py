#!/usr/bin/env python3
"""
prepare_kitti_oxts.py

Prepare KITTI OXTS sequence to CSVs and produce synthetic corrupted copies
with configurable fault models.

Usage:
    python3 prepare_kitti_oxts.py --input_dir <seq> --output_dir <out> \
        --total N --n_corrupt M --fault_config fault_model.yaml [--seed S]

Output:
    - Writes CSV files named oxts_<i>_<clean|corrupted>_seed<seed>.csv
    - Writes metadata JSON named oxts_<i>_meta.json next to each CSV
"""

import os
import glob
import json
import argparse
import random
from typing import Tuple, Dict, Any

import numpy as np
import pandas as pd

# Optional dependency: PyYAML for config. If not available, instruct user.
try:
    import yaml
except Exception:
    yaml = None

# ------------------------
# Utilities for timestamps
# ------------------------
def parse_kitti_timestamps(ts_str: str) -> int:
    """
    Parse KITTI timestamp string 'YYYY-MM-DD HH:MM:SS.sssssssss' to ns since epoch (int).
    """
    # Use pandas Timestamp for robust parsing
    ts = pd.Timestamp(ts_str)
    return int(ts.value)  # pandas Timestamp.value is ns since epoch

def find_oxts_paths(input_dir: str) -> Tuple[str, str]:
    """
    Locate timestamps.txt and data/ folder given input_dir. See docstring in earlier conversation.
    """
    t1 = os.path.join(input_dir, "timestamps.txt")
    d1 = os.path.join(input_dir, "data")
    if os.path.isfile(t1) and os.path.isdir(d1):
        return t1, d1

    t2 = os.path.join(input_dir, "oxts", "timestamps.txt")
    d2 = os.path.join(input_dir, "oxts", "data")
    if os.path.isfile(t2) and os.path.isdir(d2):
        return t2, d2

    # search one level down
    for p in glob.glob(os.path.join(input_dir, "*")):
        if os.path.isdir(p):
            t3 = os.path.join(p, "timestamps.txt")
            d3 = os.path.join(p, "data")
            if os.path.isfile(t3) and os.path.isdir(d3):
                return t3, d3

    raise FileNotFoundError(f"Could not find timestamps.txt and data/ in {input_dir}")

# ------------------------
# OXTS loading
# ------------------------
def load_oxts_sequence(input_dir: str) -> pd.DataFrame:
    timestamps_file, oxts_data_dir = find_oxts_paths(input_dir)
    with open(timestamps_file, "r") as f:
        timestamps = [line.strip() for line in f if line.strip()]

    files = sorted(glob.glob(os.path.join(oxts_data_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError(f"Number of OXTS files ({len(files)}) != timestamps ({len(timestamps)})")

    rows = []
    for ts, fp in zip(timestamps, files):
        with open(fp, "r") as f:
            line = f.readline().strip()
        values = [float(x) for x in line.split()]
        # Map values (compatible with KITTI OXTS)
        row = {
            "timestamp": ts,
            "lat": values[0], "lon": values[1], "alt": values[2],
            "roll": values[3], "pitch": values[4], "yaw": values[5],
            "vn": values[6], "ve": values[7], "vf": values[8],
            "vl": values[9], "vu": values[10],
            "ax": values[11], "ay": values[12], "az": values[13],
            "af": values[14], "al": values[15], "au": values[16],
            "wx": values[17], "wy": values[18], "wz": values[19],
            "wf": values[20], "wl": values[21], "wu": values[22],
        }
        rows.append(row)

    df = pd.DataFrame(rows)
    # compute rel_time in seconds with nanosecond precision as string
    ns_times = np.array([parse_kitti_timestamps(ts) for ts in df["timestamp"]], dtype=np.int64)
    rel_ns = ns_times - ns_times[0]
    df["rel_time"] = [f"{ns/1e9:.9f}" for ns in rel_ns]
    return df

# ------------------------
# Fault profile generation
# ------------------------
def load_fault_config(path: str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to load YAML config. Install with `pip install pyyaml`.")
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg

def scale_config(cfg: Dict[str, Any], severity: float) -> Dict[str, Any]:
    """
    Apply 'severity' scaling to numeric magnitude fields in the config.
    This function walks common keys and multiplies thresholds by severity where meaningful.
    """
    out = json.loads(json.dumps(cfg))  # deep copy via JSON
    # Simple conventions: keys containing 'std'|'bias'|'scale'|'magnitude'|'prob' are adjusted sensibly
    def rec_scale(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if isinstance(v, (int, float)):
                    lowk = k.lower()
                    if any(x in lowk for x in ("std", "bias", "magnitude", "scale", "gain", "rate")):
                        node[k] = float(v) * float(severity)
                    elif "prob" in lowk or "probability" in lowk:
                        # probabilities scale differently: use logistic-ish compression to keep <=1
                        node[k] = float(v) * float(severity)
                        if node[k] > 1.0:
                            node[k] = 1.0
                    else:
                        # keep other numeric parameters unchanged
                        node[k] = v
                else:
                    rec_scale(v)
        elif isinstance(node, list):
            for elt in node:
                rec_scale(elt)
    rec_scale(out)
    return out

# ------------------------
# Helper: geodetic meters <-> degrees
# ------------------------
def meters_to_lat_deg(meters: float) -> float:
    # approximate: 1 deg latitude ~ 111319.5 m
    return meters / 111319.5

def meters_to_lon_deg(meters: float, lat_deg: float) -> float:
    # 1 deg longitude ~ 111319.5 * cos(lat)
    return meters / (111319.5 * max(1e-6, abs(np.cos(np.deg2rad(lat_deg)))))

# ------------------------
# Fault application functions
# ------------------------
def apply_gps_faults(df: pd.DataFrame, profile: Dict[str, Any], rng: np.random.RandomState) -> Dict[str, Any]:
    """
    Applies GPS faults: position_bias (meters), noise scaling, spike_prob, dropout_prob.
    Returns mutated copy and a metadata dict describing applied parameters.
    """
    out = df.copy(deep=True)
    n = len(out)
    meta = {}

    # baseline lat for longitude scaling
    mean_lat = out["lat"].mean() if "lat" in out.columns else 0.0

    # Constant position bias (meters) -> convert to degrees
    pos_bias_m = profile.get("position_bias_m", 0.0)
    # allow vector or scalar
    if isinstance(pos_bias_m, (list, tuple)) and len(pos_bias_m) >= 2:
        bias_e_m = float(pos_bias_m[0])
        bias_n_m = float(pos_bias_m[1])
        bias_u_m = float(pos_bias_m[2]) if len(pos_bias_m) > 2 else 0.0
    else:
        bias_e_m = float(pos_bias_m)
        bias_n_m = 0.0
        bias_u_m = 0.0

    lat_offset = meters_to_lat_deg(bias_n_m)
    lon_offset = meters_to_lon_deg(bias_e_m, mean_lat)
    out["lat"] = out["lat"] + lat_offset
    out["lon"] = out["lon"] + lon_offset
    out["alt"] = out["alt"] + bias_u_m
    meta["position_bias_m"] = [bias_e_m, bias_n_m, bias_u_m]

    # noise scaling for position and velocity columns
    noise_scale = profile.get("noise_std_scale", 1.0)
    gps_cols = []
    for c in ("lat", "lon", "alt", "vn", "ve", "vu"):
        if c in out.columns:
            gps_cols.append(c)

    for c in gps_cols:
        std = out[c].std()
        if not std or np.isnan(std):
            std = 1.0
        noise = rng.normal(loc=0.0, scale=noise_scale * std, size=n)
        out[c] = out[c] + noise
    meta["noise_std_scale"] = noise_scale

    # spikes: occasional large outliers
    spike_prob = float(profile.get("spike_probability", 0.0))
    spike_mag = float(profile.get("spike_magnitude_std", 8.0))
    meta["spike_probability"] = spike_prob
    meta["spike_magnitude_std"] = spike_mag
    if spike_prob > 0:
        n_spikes = rng.binomial(n, spike_prob)
        if n_spikes > 0:
            idxs = rng.choice(n, size=n_spikes, replace=False)
            for idx in idxs:
                c = rng.choice(gps_cols)
                s = out[c].std() if out[c].std() and not np.isnan(out[c].std()) else 1.0
                out.at[idx, c] = out.at[idx, c] + rng.choice([-1, 1]) * spike_mag * s

    # dropouts: set sequences of samples to NaN (simulate loss)
    dropout_prob = float(profile.get("dropout_probability", 0.0))
    meta["dropout_probability"] = dropout_prob
    if dropout_prob > 0:
        # expected number of dropout starts
        n_starts = rng.binomial(n, dropout_prob)
        for _ in range(n_starts):
            start = int(rng.randint(0, max(1, n-1)))
            dur = int(rng.randint(1, min(8, n//10 + 1)))
            end = min(n, start + dur)
            for c in gps_cols:
                out.loc[start:end-1, c] = np.nan

    return out, meta

def apply_imu_faults(df: pd.DataFrame, profile: Dict[str, Any], rng: np.random.RandomState) -> Dict[str, Any]:
    """
    Applies IMU faults: bias drift (random walk), scale errors, spike events, dropouts, additive noise.
    Works on ax, ay, az, wx, wy, wz, and returns mutated copy + metadata.
    """
    out = df.copy(deep=True)
    n = len(out)
    meta = {}

    accel_cols = [c for c in ("ax", "ay", "az") if c in out.columns]
    gyro_cols = [c for c in ("wx", "wy", "wz") if c in out.columns]

    # Bias drift: model as random walk with zero-mean increments std = drift_std
    accel_drift_std = float(profile.get("accel_bias_drift_std", 0.0))  # units m/s^2 per sample
    gyro_drift_std = float(profile.get("gyro_bias_drift_std", 0.0))    # units rad/s per sample
    meta["accel_bias_drift_std"] = accel_drift_std
    meta["gyro_bias_drift_std"] = gyro_drift_std

    # generate drift time series and add to columns
    if accel_drift_std > 0 and accel_cols:
        increments = rng.normal(loc=0.0, scale=accel_drift_std, size=(n, len(accel_cols)))
        bias_walk = np.cumsum(increments, axis=0)
        for j, c in enumerate(accel_cols):
            out[c] = out[c] + bias_walk[:, j]

    if gyro_drift_std > 0 and gyro_cols:
        increments = rng.normal(loc=0.0, scale=gyro_drift_std, size=(n, len(gyro_cols)))
        bias_walk = np.cumsum(increments, axis=0)
        for j, c in enumerate(gyro_cols):
            out[c] = out[c] + bias_walk[:, j]

    # scale errors (multiplicative)
    accel_scale = float(profile.get("accel_scale_error", 0.0))
    gyro_scale = float(profile.get("gyro_scale_error", 0.0))
    meta["accel_scale_error"] = accel_scale
    meta["gyro_scale_error"] = gyro_scale
    if accel_scale != 0 and accel_cols:
        for c in accel_cols:
            # multiply by (1 + s + small random per-sample noise)
            out[c] = out[c] * (1.0 + accel_scale + rng.normal(0, accel_scale * 0.1, size=n))
    if gyro_scale != 0 and gyro_cols:
        for c in gyro_cols:
            out[c] = out[c] * (1.0 + gyro_scale + rng.normal(0, gyro_scale * 0.1, size=n))

    # additive noise scaling
    accel_noise_scale = float(profile.get("accel_noise_scale", 0.0))
    gyro_noise_scale = float(profile.get("gyro_noise_scale", 0.0))
    meta["accel_noise_scale"] = accel_noise_scale
    meta["gyro_noise_scale"] = gyro_noise_scale
    for c in accel_cols:
        base = out[c].std() if out[c].std() and not np.isnan(out[c].std()) else 1.0
        out[c] = out[c] + rng.normal(0.0, accel_noise_scale * base, size=n)
    for c in gyro_cols:
        base = out[c].std() if out[c].std() and not np.isnan(out[c].std()) else 1.0
        out[c] = out[c] + rng.normal(0.0, gyro_noise_scale * base, size=n)

    # spikes
    spike_prob = float(profile.get("spike_probability", 0.0))
    spike_mag = float(profile.get("spike_magnitude_std", 8.0))
    meta["spike_probability"] = spike_prob
    meta["spike_magnitude_std"] = spike_mag
    imu_cols = accel_cols + gyro_cols
    if spike_prob > 0 and imu_cols:
        n_spikes = rng.binomial(n, spike_prob)
        if n_spikes > 0:
            idxs = rng.choice(n, size=n_spikes, replace=False)
            for idx in idxs:
                c = rng.choice(imu_cols)
                s = out[c].std() if out[c].std() and not np.isnan(out[c].std()) else 1.0
                out.at[idx, c] = out.at[idx, c] + rng.choice([-1, 1]) * spike_mag * s

    # dropouts: set to NaN or repeat last value for short bursts
    dropout_prob = float(profile.get("dropout_probability", 0.0))
    meta["dropout_probability"] = dropout_prob
    if dropout_prob > 0 and imu_cols:
        n_starts = rng.binomial(n, dropout_prob)
        for _ in range(n_starts):
            start = int(rng.randint(0, max(1, n-1)))
            dur = int(rng.randint(1, min(6, n//10 + 1)))
            end = min(n, start + dur)
            for c in imu_cols:
                # choose whether to NaN or hold-last
                if rng.random() < 0.5:
                    out.loc[start:end-1, c] = np.nan
                else:
                    # hold-last: repeat previous value where possible
                    if start == 0:
                        out.loc[start:end-1, c] = 0.0
                    else:
                        out.loc[start:end-1, c] = out.loc[start-1, c]

    return out, meta

# ------------------------
# Top-level injector
# ------------------------
def inject_faults_for_copy(df_base: pd.DataFrame, base_cfg: Dict[str, Any],
                           copy_seed: int, severity_scale: float, corrupt_gps: bool, corrupt_imu: bool) -> Tuple[pd.DataFrame, Dict[str, Any]]:
    """
    Generate a corrupted version of the base dataframe using configuration.
    Returns mutated df and metadata dict of applied parameters.
    """
    rng = np.random.RandomState(copy_seed)
    # scale the config by severity scale (makes global sweeps easy)
    cfg = scale_config(base_cfg, severity_scale) if severity_scale != 1.0 else base_cfg

    df_mut = df_base.copy(deep=True)
    meta = {"seed": int(copy_seed), "severity_scale": severity_scale}

    if corrupt_gps and "gps" in cfg and cfg["gps"].get("enabled", True):
        df_mut, gps_meta = apply_gps_faults(df_mut, cfg["gps"], rng)
        meta["gps"] = gps_meta
    else:
        meta["gps"] = {"applied": False}

    if corrupt_imu and "imu" in cfg and cfg["imu"].get("enabled", True):
        df_mut, imu_meta = apply_imu_faults(df_mut, cfg["imu"], rng)
        meta["imu"] = imu_meta
    else:
        meta["imu"] = {"applied": False}

    # Recompute rel_time column if any rows removed or NaNs inserted (keep original timestamps)
    # Keep rel_time as seconds with nanosecond precision string
    # (we keep original timestamp column intact)
    # Convert timestamps to ns to recompute relative times
    try:
        ns_times = np.array([parse_kitti_timestamps(ts) for ts in df_mut["timestamp"]], dtype=np.int64)
        rel_ns = ns_times - ns_times[0]
        df_mut["rel_time"] = [f"{ns/1e9:.9f}" for ns in rel_ns]
    except Exception:
        # fallback: keep original rel_time if something goes wrong
        pass

    return df_mut, meta

# ------------------------
# Main CLI
# ------------------------
def main():
    parser = argparse.ArgumentParser(description="Prepare KITTI OXTS CSVs with configurable fault injection.")
    parser.add_argument("--input_dir", required=True, help="KITTI sequence folder (contains timestamps.txt and data/).")
    parser.add_argument("--output_dir", required=True, help="Directory where generated CSVs will be written.")
    parser.add_argument("--total", type=int, default=5, help="Total number of output CSVs (default 5).")
    parser.add_argument("--n_corrupt", type=int, default=0, help="Number of corrupted CSVs among total (default 0).")
    parser.add_argument("--fault_config", type=str, required=True, help="Path to YAML fault model config file.")
    parser.add_argument("--severity", type=float, default=1.0, help="Global severity multiplier (default 1.0).")
    parser.add_argument("--seed", type=int, default=None, help="Optional master seed. If omitted, random seeds used per copy.")
    parser.add_argument("--gps_only", action="store_true", help="Only apply GPS faults (if corrupted).")
    parser.add_argument("--imu_only", action="store_true", help="Only apply IMU faults (if corrupted).")
    args = parser.parse_args()

    if args.n_corrupt > args.total:
        parser.error("--n_corrupt cannot exceed --total")

    if yaml is None:
        parser.error("PyYAML is required. Install it with: pip install pyyaml")

    # load base sequence
    base_df = load_oxts_sequence(args.input_dir)

    # load config
    base_cfg = load_fault_config(args.fault_config)

    # ensure output dir
    os.makedirs(args.output_dir, exist_ok=True)

    # deterministic selection of corrupted indices if seed provided, else random
    master_rng = np.random.RandomState(args.seed) if args.seed is not None else np.random.RandomState(random.randint(0, 2**31-1))
    all_indices = list(range(args.total))
    corrupted_indices = set(master_rng.choice(all_indices, size=args.n_corrupt, replace=False).tolist()) if args.n_corrupt > 0 else set()

    print(f"[INFO] Generating {args.total} copies; corrupting indices: {sorted(list(corrupted_indices))}")

    for i in range(args.total):
        is_corrupt = i in corrupted_indices
        # choose per-copy seed
        if args.seed is not None:
            copy_seed = int(args.seed + i + 1)
        else:
            copy_seed = int(master_rng.randint(0, 2**31-1))

        corrupt_gps = is_corrupt and (not args.imu_only)
        corrupt_imu = is_corrupt and (not args.gps_only)

        if not is_corrupt:
            # write pristine copy (just base_df)
            out_df = base_df.copy(deep=True)
            # still add a metadata file
            meta = {
                "seed": int(copy_seed),
                "status": "clean",
                "source": os.path.abspath(args.input_dir)
            }
            out_name = f"oxts_{i:03d}_clean_seed{copy_seed}.csv"
            out_csv = os.path.join(args.output_dir, out_name)
            out_df.to_csv(out_csv, index=False)
            meta_path = out_csv.replace(".csv", "_meta.json")
            with open(meta_path, "w") as mf:
                json.dump(meta, mf, indent=2)
            print(f"[SAVED] {out_csv} (clean)")
        else:
            out_df, meta = inject_faults_for_copy(
                base_df,
                base_cfg,
                copy_seed=copy_seed,
                severity_scale=args.severity,
                corrupt_gps=corrupt_gps,
                corrupt_imu=corrupt_imu
            )
            meta["status"] = "corrupted"
            meta["source"] = os.path.abspath(args.input_dir)
            out_name = f"oxts_{i:03d}_corrupted_seed{copy_seed}.csv"
            out_csv = os.path.join(args.output_dir, out_name)
            # write CSV (NaNs preserved). maintain column order
            out_df.to_csv(out_csv, index=False)
            meta_path = out_csv.replace(".csv", "_meta.json")
            with open(meta_path, "w") as mf:
                json.dump(meta, mf, indent=2)
            print(f"[SAVED] {out_csv} (corrupted) - meta -> {meta_path}")

    print("[DONE] Generation complete.")

if __name__ == "__main__":
    main()

