#!/usr/bin/env python3
"""
Aggregated EKF performance visualization across all datasets.
Analyzes how filter metrics vary with error ratio and corruption percentage.
"""

import os
import sys
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.size": 11,
    "axes.grid": True,
    "grid.linestyle": ":",
    "lines.markersize": 5,
    "lines.linewidth": 1.5,
    "legend.frameon": True
})

# ---------- CLI ----------
if len(sys.argv) < 3:
    print("Usage: python analyze_all_results.py <input_root_dir> <output_summary_dir>")
    sys.exit(1)

root_dir = sys.argv[1]
output_dir = sys.argv[2]
os.makedirs(output_dir, exist_ok=True)

# ---------- Load all result CSVs ----------
csv_files = glob.glob(os.path.join(root_dir, "*", "oxts*", "results_metrics.csv"))
if not csv_files:
    print("[ERROR] No results_metrics.csv found")
    sys.exit(1)

dfs = []
for f in csv_files:
    try:
        df = pd.read_csv(f)
        if df.empty:
            continue
        df["source_file"] = os.path.basename(f)
        df["dataset_group"] = f.split("/")[-3]
        df["dataset"] = f.split("/")[-2]
        df["corrupt_pct"] = (df["n_corrupt"] / df["n_file"]) * 100.0
        dfs.append(df)
    except Exception as e:
        print(f"[WARN] Skipping {f}: {e}")

if not dfs:
    print("[ERROR] No valid data loaded.")
    sys.exit(1)

data = pd.concat(dfs, ignore_index=True)
print(f"[INFO] Loaded {len(data)} runs from {len(csv_files)} datasets")

# ---------- Aggregation helpers ----------
def agg_summary(df, group_vars):
    return df.groupby(group_vars).agg({
        "pos_accept_rate": ["mean", "std"],
        "vel_accept_rate": ["mean", "std"],
        "pos_mahal_mean": ["mean", "std"],
        "vel_mahal_mean": ["mean", "std"],
        "mean_traceP": ["mean", "std"]
    }).reset_index()

# Aggregated by error ratio and by corrupted sensor %
summary_err = agg_summary(data, ["dataset_group", "error_ratio"])
summary_corrupt = agg_summary(data, ["dataset_group", "corrupt_pct"])

# ---------- Plotting ----------
def save_fig(fig, name):
    fig.tight_layout()
    path = os.path.join(output_dir, name)
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"[SAVED] {path}")

MARKERS = ['o', 's', '^', 'D', 'x', '+']
LINESTYLES = ['-', '--', '-.', ':']
def bw_style(idx): 
    return MARKERS[idx % len(MARKERS)], LINESTYLES[idx % len(LINESTYLES)]

# === Plot 1: Acceptance vs Error Ratio ===
fig, ax = plt.subplots(figsize=(7,5))
for i, g in enumerate(sorted(summary_err["dataset_group"].unique())):
    subset = summary_err[summary_err["dataset_group"] == g]
    m, s = bw_style(i)
    ax.errorbar(subset["error_ratio"], subset["pos_accept_rate"]["mean"], 
                yerr=subset["pos_accept_rate"]["std"],
                fmt=m+LINESTYLES[i%len(LINESTYLES)], color="black",
                label=f"{g} (pos)")
ax.set_xlabel("Error Ratio")
ax.set_ylabel("Position Acceptance Rate")
ax.set_title("Acceptance vs Error Ratio (mean ± std)")
ax.legend()
save_fig(fig, "accept_vs_error_ratio.png")

# === Plot 2: Mahalanobis mean vs Error Ratio ===
fig, ax = plt.subplots(figsize=(7,5))
for i, g in enumerate(sorted(summary_err["dataset_group"].unique())):
    subset = summary_err[summary_err["dataset_group"] == g]
    m, s = bw_style(i)
    ax.plot(subset["error_ratio"], subset["pos_mahal_mean"]["mean"], 
            s, marker=m, color="black", label=f"{g}")
ax.set_xlabel("Error Ratio")
ax.set_ylabel("Mean Mahalanobis (Position)")
ax.set_title("Mahalanobis vs Error Ratio")
ax.legend()
save_fig(fig, "mahal_vs_error_ratio.png")

# === Plot 3: Trace(P) vs Error Ratio ===
fig, ax = plt.subplots(figsize=(7,5))
for i, g in enumerate(sorted(summary_err["dataset_group"].unique())):
    subset = summary_err[summary_err["dataset_group"] == g]
    m, s = bw_style(i)
    ax.plot(subset["error_ratio"], subset["mean_traceP"]["mean"],
            s, marker=m, color="black", label=f"{g}")
ax.set_xlabel("Error Ratio")
ax.set_ylabel("trace(P) (mean)")
ax.set_title("Covariance trace vs Error Ratio")
ax.legend()
save_fig(fig, "traceP_vs_error_ratio.png")

# === Plot 4: Acceptance vs Corrupted Sensor Percentage ===
fig, ax = plt.subplots(figsize=(7,5))
for i, g in enumerate(sorted(summary_corrupt["dataset_group"].unique())):
    subset = summary_corrupt[summary_corrupt["dataset_group"] == g]
    m, s = bw_style(i)
    ax.plot(subset["corrupt_pct"], subset["pos_accept_rate"]["mean"],
            s, marker=m, color="black", label=f"{g}")
ax.set_xlabel("Corrupted Sensor Percentage (%)")
ax.set_ylabel("Position Acceptance Rate")
ax.set_title("Acceptance vs Corrupted Sensor Percentage")
ax.legend()
save_fig(fig, "accept_vs_corrupt_pct.png")

# === Plot 5: Mahalanobis vs Corrupted Sensor Percentage ===
fig, ax = plt.subplots(figsize=(7,5))
for i, g in enumerate(sorted(summary_corrupt["dataset_group"].unique())):
    subset = summary_corrupt[summary_corrupt["dataset_group"] == g]
    m, s = bw_style(i)
    ax.plot(subset["corrupt_pct"], subset["pos_mahal_mean"]["mean"],
            s, marker=m, color="black", label=f"{g}")
ax.set_xlabel("Corrupted Sensor Percentage (%)")
ax.set_ylabel("Mean Mahalanobis (Position)")
ax.set_title("Mahalanobis vs Corrupted Sensors (%)")
ax.legend()
save_fig(fig, "mahal_vs_corrupt_pct.png")

print(f"\n✅ Summary plots saved in {output_dir}")

