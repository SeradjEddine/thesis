#!/usr/bin/env python3
import os
import sys
import pandas as pd
import numpy as np
from datetime import datetime

# --- CLI Args ---
if len(sys.argv) < 8:
    print("Usage: python eval_metrics.py <input_dir> <output_dir> <n_file> <n_corrupt> <error_ratio> <seed> <dataset_label>")
    sys.exit(1)

input_dir, output_dir = sys.argv[1:3]
n_file, n_corrupt, error_ratio, seed, dataset_label = int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5]), int(sys.argv[6]), sys.argv[7]
os.makedirs(output_dir, exist_ok=True)

dataset_name = os.path.basename(os.path.normpath(input_dir))
print(f"\n[INFO] Processing dataset: {dataset_label}/{dataset_name}")

gps_path = os.path.join(input_dir, "gps_updates.csv")
imu_path = os.path.join(input_dir, "imu_prop.csv")

if not (os.path.exists(gps_path) and os.path.exists(imu_path)):
    print(f"[ERROR] Missing gps/imu CSVs in {input_dir}")
    sys.exit(1)

gps_df = pd.read_csv(gps_path)
imu_df = pd.read_csv(imu_path)

if gps_df.empty or imu_df.empty:
    print(f"[WARN] Empty dataset: {input_dir}")
    sys.exit(0)

# --- Acceptance ---
pos_accept_rate = gps_df["accepted_pos"].mean()
vel_accept_rate = gps_df["accepted_vel"].mean()

# Filter accepted updates
pos_mask = gps_df["accepted_pos"] == 1
vel_mask = gps_df["accepted_vel"] == 1

# --- Mahalanobis (only accepted) ---
pos_mahal_mean = gps_df.loc[pos_mask, "mahalanobis_pos"].mean()
pos_mahal_std  = gps_df.loc[pos_mask, "mahalanobis_pos"].std()
vel_mahal_mean = gps_df.loc[vel_mask, "mahalanobis_vel"].mean()
vel_mahal_std  = gps_df.loc[vel_mask, "mahalanobis_vel"].std()

# --- Innovations (only accepted) ---
innov_pos_mag = np.sqrt(gps_df["innov_x"]**2 + gps_df["innov_y"]**2 + gps_df["innov_z"]**2)
innov_vel_mag = np.sqrt(gps_df["innov_vx"]**2 + gps_df["innov_vy"]**2 + gps_df["innov_vz"]**2)

mean_innov_pos = innov_pos_mag[pos_mask].mean()
std_innov_pos  = innov_pos_mag[pos_mask].std()
mean_innov_vel = innov_vel_mag[vel_mask].mean()
std_innov_vel  = innov_vel_mag[vel_mask].std()

# --- Covariance stats ---
mean_traceP = imu_df["traceP"].mean()
std_traceP  = imu_df["traceP"].std()

# --- Mahalanobis under threshold (accepted only) ---
pos_mahal_under_thresh = np.mean(gps_df.loc[pos_mask, "mahalanobis_pos"] < 16.27)
vel_mahal_under_thresh = np.mean(gps_df.loc[vel_mask, "mahalanobis_vel"] < 16.27)

# --- Compose Result Row ---
row = {
    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    "dataset_group": dataset_label,
    "dataset": dataset_name,
    "n_file": n_file,
    "n_corrupt": n_corrupt,
    "error_ratio": error_ratio,
    "seed": seed,
    "pos_accept_rate": pos_accept_rate,
    "vel_accept_rate": vel_accept_rate,
    "pos_mahal_mean": pos_mahal_mean,
    "pos_mahal_std": pos_mahal_std,
    "vel_mahal_mean": vel_mahal_mean,
    "vel_mahal_std": vel_mahal_std,
    "pos_mahal_under_thresh": pos_mahal_under_thresh,
    "vel_mahal_under_thresh": vel_mahal_under_thresh,
    "mean_innov_pos": mean_innov_pos,
    "std_innov_pos": std_innov_pos,
    "mean_innov_vel": mean_innov_vel,
    "std_innov_vel": std_innov_vel,
    "mean_traceP": mean_traceP,
    "std_traceP": std_traceP,
}

# --- Append to output CSV ---
output_csv = os.path.join(output_dir, "results_metrics.csv")
df_row = pd.DataFrame([row])
if not os.path.exists(output_csv):
    df_row.to_csv(output_csv, index=False)
    print(f"[NEW] Created {output_csv}")
else:
    df_row.to_csv(output_csv, mode='a', index=False, header=False)
    print(f"[APPEND] Added results to {output_csv}")

print(f"[DONE] Metrics computed for {dataset_label}/{dataset_name}")

