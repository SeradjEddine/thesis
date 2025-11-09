#!/usr/bin/env python3
"""
analyze_sim_results.py
Analyze simulation outputs under Data/sim_results_csv/

Usage:
    python3 analyze_sim_results.py [--run_id <id>] [--n_corrupt N] [--error_rate R]
"""

import os
import argparse
import pandas as pd
import numpy as np
from datetime import datetime

def compute_rmse(a, b):
    """Root mean squared error between two vectors of equal length."""
    return np.sqrt(np.mean((a - b)**2))

def analyze_run(run_dir, run_id, n_corrupt, error_rate):
    """Compute summary metrics for a single run directory."""
    try:
        imu_path = os.path.join(run_dir, "imu_prop.csv")
        gps_path = os.path.join(run_dir, "gps_updates.csv")
        fuzzy_path = os.path.join(run_dir, "fuzzy.csv")

        if not (os.path.isfile(imu_path) and os.path.isfile(gps_path)):
            return None  # incomplete run

        imu = pd.read_csv(imu_path)
        gps = pd.read_csv(gps_path)
        fuzzy = pd.read_csv(fuzzy_path) if os.path.isfile(fuzzy_path) else None

        metrics = {
            "run_id": run_id,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "n_corrupt": n_corrupt,
            "error_rate": error_rate
        }

        # --- Mahalanobis distances ---
        if "mahalanobis_pos" in gps.columns:
            metrics["mean_maha_pos"] = gps["mahalanobis_pos"].mean()
            metrics["max_maha_pos"] = gps["mahalanobis_pos"].max()
        if "mahalanobis_vel" in gps.columns:
            metrics["mean_maha_vel"] = gps["mahalanobis_vel"].mean()
            metrics["max_maha_vel"] = gps["mahalanobis_vel"].max()

        # --- Rejection ratio ---
        if "accepted_pos" in gps.columns:
            metrics["reject_ratio"] = 1 - gps["accepted_pos"].mean()

        # --- RMSE position ---
        if all(c in imu.columns for c in ["px", "py"]) and all(c in gps.columns for c in ["zx", "zy"]):
            ekf_xy = imu[["px", "py"]].to_numpy()
            gps_xy = gps[["zx", "zy"]].to_numpy()
            min_len = min(len(ekf_xy), len(gps_xy))
            metrics["rmse_pos"] = compute_rmse(ekf_xy[:min_len], gps_xy[:min_len])

        # --- RMSE velocity ---
        if all(c in imu.columns for c in ["vx", "vy"]) and all(c in gps.columns for c in ["vx", "vy"]):
            ekf_v = imu[["vx", "vy"]].to_numpy()
            gps_v = gps[["vx", "vy"]].to_numpy()
            min_len = min(len(ekf_v), len(gps_v))
            metrics["rmse_vel"] = compute_rmse(ekf_v[:min_len], gps_v[:min_len])

        # --- Fuzzy metrics ---
        if fuzzy is not None:
            for key in ["scale_R_gps", "scale_Q", "scale_gate"]:
                if key in fuzzy.columns:
                    metrics[f"mean_{key}"] = fuzzy[key].mean()

        return metrics

    except Exception as e:
        print(f"[WARN] Could not analyze {run_dir}: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description="Analyze local simulation results")
    parser.add_argument("--run_id", default="0001", help="Experiment run identifier")
    parser.add_argument("--n_corrupt", type=int, default=0, help="Number of corrupted files")
    parser.add_argument("--error_rate", type=float, default=0.0, help="Error rate applied to corrupted files")
    args = parser.parse_args()

    run_id = args.run_id
    n_corrupt = args.n_corrupt
    error_rate = args.error_rate

    base_dir = "../Data/sim_results_csv"

    # Collect all directories containing simulation results
    run_dirs = []
    for root, dirs, files in os.walk(base_dir):
        for d in dirs:
            folder_path = os.path.join(root, d)
            if any(fname.endswith(".csv") for fname in os.listdir(folder_path)):
                run_dirs.append(folder_path)

    if not run_dirs:
        print("No result directories found.")
        return

    for run_dir in sorted(run_dirs):
        metrics = analyze_run(run_dir, run_id, n_corrupt, error_rate)
        if metrics is None:
            continue

        log_path = os.path.join(run_dir, "analysis_log.csv")
        df_entry = pd.DataFrame([metrics])

        # Append or create the local log
        if os.path.exists(log_path):
            df_entry.to_csv(log_path, mode="a", header=False, index=False)
        else:
            df_entry.to_csv(log_path, mode="w", header=True, index=False)

        print(f"Logged analysis for {os.path.relpath(run_dir, base_dir)} → {log_path}")

    print(f"✅ Analysis completed for all runs with run_id={run_id}")


if __name__ == "__main__":
    main()

