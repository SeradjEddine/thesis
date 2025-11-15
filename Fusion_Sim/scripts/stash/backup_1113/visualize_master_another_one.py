#!/usr/bin/env python3
"""
Plot performance metrics vs error ratio and corrupted sensor percentage
for a single dataset log file (results_metrics.csv).
"""

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.size": 11,
    "axes.grid": True,
    "grid.linestyle": ":",
    "lines.markersize": 5,
    "lines.linewidth": 1.2,
    "legend.frameon": True
})

# ------------- CLI -------------
if len(sys.argv) < 2:
    print("Usage: python plot_dataset_metrics.py <dataset_folder>")
    print("Example: python plot_dataset_metrics.py ../Data/sim_results_csv/city/oxts232")
    sys.exit(1)

dataset_dir = sys.argv[1]
if not os.path.isdir(dataset_dir):
    print(f"[ERROR] Directory not found: {dataset_dir}")
    sys.exit(1)

csv_path = os.path.join(dataset_dir, "results_metrics.csv")
if not os.path.isfile(csv_path):
    print(f"[ERROR] Missing results_metrics.csv in {dataset_dir}")
    sys.exit(1)

# ------------- Load and Prepare -------------
df = pd.read_csv(csv_path)
if df.empty:
    print(f"[WARN] Empty results file: {csv_path}")
    sys.exit(0)

# Derived field: corrupted sensor percentage
df["corrupt_pct"] = (df["n_corrupt"] / df["n_file"]) * 100.0

# Ensure numeric columns
df["error_ratio"] = df["error_ratio"].astype(float)
df["corrupt_pct"] = df["corrupt_pct"].astype(float)

# Helper for black & white style
MARKERS = ['o', 's', '^', 'D', 'x', '+']
LINESTYLES = ['-', '--', '-.', ':']
def bw_style(idx):
    return MARKERS[idx % len(MARKERS)], LINESTYLES[idx % len(LINESTYLES)]

def save_fig(fig, name):
    out_path = os.path.join(dataset_dir, name)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"[SAVED] {out_path}")

# ------------- Plot 1: Acceptance Rates vs Error Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("Acceptance Rate vs Error Ratio")

marker, style = bw_style(0)
ax.plot(df["error_ratio"], df["pos_accept_rate"], style, marker=marker,
        label="Position Acceptance", color="black")

marker, style = bw_style(1)
ax.plot(df["error_ratio"], df["vel_accept_rate"], style, marker=marker,
        fillstyle='none', label="Velocity Acceptance", color="gray")

ax.set_xlabel("Error Ratio")
ax.set_ylabel("Acceptance Rate")
ax.legend()
save_fig(fig, "accept_rate_vs_error_ratio.png")

# ------------- Plot 2: Mahalanobis Means vs Error Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("Mahalanobis Distance (Mean) vs Error Ratio")

marker, style = bw_style(0)
ax.plot(df["error_ratio"], df["pos_mahal_mean"], style, marker=marker,
        label="Position", color="black")

marker, style = bw_style(1)
ax.plot(df["error_ratio"], df["vel_mahal_mean"], style, marker=marker,
        fillstyle='none', label="Velocity", color="gray")

ax.set_xlabel("Error Ratio")
ax.set_ylabel("Mean Mahalanobis Distance")
ax.legend()
save_fig(fig, "mahalanobis_vs_error_ratio.png")

# ------------- Plot 3: Innovation Magnitudes vs Error Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("Innovation Magnitude vs Error Ratio")

marker, style = bw_style(0)
ax.plot(df["error_ratio"], df["mean_innov_pos"], style, marker=marker,
        label="Position Innovation", color="black")

marker, style = bw_style(1)
ax.plot(df["error_ratio"], df["mean_innov_vel"], style, marker=marker,
        fillstyle='none', label="Velocity Innovation", color="gray")

ax.set_xlabel("Error Ratio")
ax.set_ylabel("Mean Innovation (m or m/s)")
ax.legend()
save_fig(fig, "innovation_vs_error_ratio.png")

# ------------- Plot 4: Covariance Trace vs Error Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("State Covariance (trace P) vs Error Ratio")

marker, style = bw_style(0)
ax.plot(df["error_ratio"], df["mean_traceP"], style, marker=marker, color="black")
ax.fill_between(df["error_ratio"], 
                df["mean_traceP"] - df["std_traceP"], 
                df["mean_traceP"] + df["std_traceP"],
                color="lightgray", alpha=0.3, label="±1σ")
ax.set_xlabel("Error Ratio")
ax.set_ylabel("Mean trace(P)")
ax.legend()
save_fig(fig, "traceP_vs_error_ratio.png")

# ------------- Plot 5: Performance vs Corrupted Sensor Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("Performance vs Corrupted Sensor Percentage")

marker, style = bw_style(0)
ax.plot(df["corrupt_pct"], df["pos_accept_rate"], style, marker=marker,
        label="Pos Acceptance", color="black")

marker, style = bw_style(1)
ax.plot(df["corrupt_pct"], df["vel_accept_rate"], style, marker=marker,
        fillstyle='none', label="Vel Acceptance", color="gray")

ax.set_xlabel("Corrupted Sensor Percentage (%)")
ax.set_ylabel("Acceptance Rate")
ax.legend()
save_fig(fig, "accept_rate_vs_corrupt_pct.png")

# ------------- Plot 6: Mahalanobis Mean vs Corrupted Sensor Ratio -------------
fig, ax = plt.subplots(figsize=(7,5))
ax.set_title("Mahalanobis Mean vs Corrupted Sensor Percentage")

marker, style = bw_style(0)
ax.plot(df["corrupt_pct"], df["pos_mahal_mean"], style, marker=marker,
        label="Position", color="black")

marker, style = bw_style(1)
ax.plot(df["corrupt_pct"], df["vel_mahal_mean"], style, marker=marker,
        fillstyle='none', label="Velocity", color="gray")

ax.set_xlabel("Corrupted Sensor Percentage (%)")
ax.set_ylabel("Mean Mahalanobis Distance")
ax.legend()
save_fig(fig, "mahalanobis_vs_corrupt_pct.png")

print(f"\n✅ All plots saved in {dataset_dir}")

