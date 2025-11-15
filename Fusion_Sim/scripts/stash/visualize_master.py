#!/usr/bin/env python3
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# --- CLI ---
if len(sys.argv) < 3:
    print("Usage: python plot_batch_results.py <input_root> <output_dir>")
    print("Example: python plot_batch_results.py ../Data/sim_results_csv ../Data/sim_results_plots")
    sys.exit(1)

input_root = sys.argv[1]       # e.g., ../Data/sim_results_csv
output_dir = sys.argv[2]       # e.g., ../Data/sim_results_plots
os.makedirs(output_dir, exist_ok=True)

# --- Collect results ---
records = []
for group in ["city", "residential", "road"]:
    group_path = os.path.join(input_root, group)
    if not os.path.isdir(group_path):
        print(f"[WARN] Missing group folder: {group_path}")
        continue
    for dataset in sorted(os.listdir(group_path)):
        results_file = os.path.join(group_path, dataset, "results_metrics.csv")
        if os.path.isfile(results_file):
            df = pd.read_csv(results_file)
            df["dataset_group"] = group
            df["dataset"] = dataset
            records.append(df)

if not records:
    print("[ERROR] No results_metrics.csv files found!")
    sys.exit(1)

data = pd.concat(records, ignore_index=True)
print(f"[INFO] Loaded {len(data)} records from {len(records)} datasets.")

# --- Helpers for BW plotting ---
MARKERS = ['o', 's', 'D', '^', 'v', '>', '<', 'x', '+']
LINESTYLES = ['-', '--', '-.', ':']
def bw_style(idx):
    return MARKERS[idx % len(MARKERS)], LINESTYLES[idx % len(LINESTYLES)]

def save_fig(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, name), dpi=300, bbox_inches='tight')
    plt.close(fig)

# --- 1. Accept rate vs error ratio ---
fig, ax = plt.subplots(figsize=(8,5))
for i, group in enumerate(sorted(data["dataset_group"].unique())):
    gdf = data[data["dataset_group"] == group]
    gdf = gdf.groupby("error_ratio").mean(numeric_only=True).reset_index()
    marker, style = bw_style(i)
    ax.plot(gdf["error_ratio"], gdf["pos_accept_rate"], style, marker=marker,
            markersize=5, label=f"{group} (pos)")
    ax.plot(gdf["error_ratio"], gdf["vel_accept_rate"], style, marker=marker,
            markersize=5, fillstyle='none', label=f"{group} (vel)")
ax.set_xlabel("Error ratio")
ax.set_ylabel("Acceptance rate")
ax.set_title("GPS Update Acceptance vs Error Level")
ax.grid(True, linestyle=':')
ax.legend()
save_fig(fig, "accept_rate_vs_error.png")

# --- 2. Mahalanobis mean vs std (error ratio) ---
for metric, title in [("pos_mahal", "Position Mahalanobis"), ("vel_mahal", "Velocity Mahalanobis")]:
    mean_col = f"{metric}_mean"
    std_col = f"{metric}_std"
    fig, ax = plt.subplots(figsize=(8,5))
    for i, group in enumerate(sorted(data["dataset_group"].unique())):
        gdf = data[data["dataset_group"] == group]
        gdf = gdf.groupby("error_ratio").mean(numeric_only=True).reset_index()
        marker, style = bw_style(i)
        ax.errorbar(gdf["error_ratio"], gdf[mean_col], yerr=gdf[std_col],
                    fmt=marker+style, capsize=3, label=group)
    ax.set_xlabel("Error ratio")
    ax.set_ylabel("Mahalanobis distance")
    ax.set_title(f"{title} Mean ± Std vs Error Ratio")
    ax.grid(True, linestyle=':')
    ax.legend()
    save_fig(fig, f"{metric}_mahal_vs_error.png")

# --- 3. Innovation magnitudes (mean & std) ---
for metric, label in [("mean_innov_pos", "Position innovation"), ("mean_innov_vel", "Velocity innovation")]:
    fig, ax = plt.subplots(figsize=(8,5))
    for i, group in enumerate(sorted(data["dataset_group"].unique())):
        gdf = data[data["dataset_group"] == group]
        gdf = gdf.groupby("error_ratio").mean(numeric_only=True).reset_index()
        marker, style = bw_style(i)
        ax.plot(gdf["error_ratio"], gdf[metric], style, marker=marker,
                label=f"{group}")
    ax.set_xlabel("Error ratio")
    ax.set_ylabel("Mean innovation (m or m/s)")
    ax.set_title(f"{label} vs Error Ratio")
    ax.grid(True, linestyle=':')
    ax.legend()
    save_fig(fig, f"{metric}_vs_error.png")

# --- 4. Trace(P) as indicator of uncertainty ---
fig, ax = plt.subplots(figsize=(8,5))
for i, group in enumerate(sorted(data["dataset_group"].unique())):
    gdf = data[data["dataset_group"] == group]
    gdf = gdf.groupby("error_ratio").mean(numeric_only=True).reset_index()
    marker, style = bw_style(i)
    ax.plot(gdf["error_ratio"], gdf["mean_traceP"], style, marker=marker,
            label=group)
ax.set_xlabel("Error ratio")
ax.set_ylabel("Mean trace(P)")
ax.set_title("State Covariance Trace vs Error Ratio")
ax.grid(True, linestyle=':')
ax.legend()
save_fig(fig, "traceP_vs_error.png")

# --- 5. Bar chart: acceptance rate by environment (for fixed error ratio=0.5) ---
subset = data[np.isclose(data["error_ratio"], 0.5, atol=1e-6)]
fig, ax = plt.subplots(figsize=(8,5))
groups = subset["dataset_group"].unique()
pos_accept = [subset[subset["dataset_group"]==g]["pos_accept_rate"].mean() for g in groups]
vel_accept = [subset[subset["dataset_group"]==g]["vel_accept_rate"].mean() for g in groups]
x = np.arange(len(groups))
ax.bar(x - 0.15, pos_accept, width=0.3, hatch='//', color='lightgray', label="pos")
ax.bar(x + 0.15, vel_accept, width=0.3, hatch='\\\\', color='white', edgecolor='black', label="vel")
ax.set_xticks(x)
ax.set_xticklabels(groups)
ax.set_ylabel("Acceptance rate")
ax.set_title("Acceptance Rate by Environment (error_ratio=0.5)")
ax.legend()
ax.grid(True, axis='y', linestyle=':')
save_fig(fig, "accept_rate_bar_env.png")

print(f"[DONE] Saved plots in {output_dir}")

