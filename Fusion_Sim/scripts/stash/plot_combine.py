#!/usr/bin/env python3
"""
Visualize aggregated results from combined_n_corrupt_*.csv files.

Input:
    ../Data/combined_results/combined_n_corrupt_X.csv

Output:
    ../Data/plots_combined/n_corrupt_X/*.png

This script creates separate figures for each corruption level.
"""

import os
import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob

plt.rcParams.update({
    "font.size": 11,
    "axes.grid": True,
    "grid.linestyle": ":",
    "lines.markersize": 5,
    "lines.linewidth": 1.2,
    "legend.frameon": True
})

# ----------------------------------------------------------
# Paths (mirroring structure from your combiner)
# ----------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
COMBINED_DIR = SCRIPT_DIR.parent / "Data" / "combined_results"
OUT_DIR = SCRIPT_DIR.parent / "Data" / "plots_combined"
OUT_DIR.mkdir(exist_ok=True, parents=True)

# ----------------------------------------------------------
# Helper: B&W marker/linestyle cycling
# ----------------------------------------------------------
MARKERS = ['o', 's', '^', 'D', 'x', '+']
LINESTYLES = ['-', '--', '-.', ':']

def bw_style(idx):
    return MARKERS[idx % len(MARKERS)], LINESTYLES[idx % len(LINESTYLES)]


def save_fig(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[SAVED] {path}")


# ----------------------------------------------------------
# Main plotting function for one corruption level
# ----------------------------------------------------------
def plot_for_corruption_level(df, nc, out_folder):
    """
    df = combined dataframe for n_corrupt = nc
    """
    # Derived fields
    if "corrupt_pct" not in df.columns:
        df["corrupt_pct"] = (df["n_corrupt"] / df["n_file"]) * 100.0

    # Sort by error ratio to avoid zig-zag lines
    df = df.sort_values("error_ratio")

    # ------------------ Plot 1: Acceptance vs Error Ratio ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"Acceptance Rate vs Error Ratio (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["error_ratio"], df["pos_accept_rate"], ls, marker=m,
            color="black", label="Position Acceptance")

    m, ls = bw_style(1)
    ax.plot(df["error_ratio"], df["vel_accept_rate"], ls, marker=m,
            fillstyle="none", color="gray", label="Velocity Acceptance")

    ax.set_xlabel("Error Ratio")
    ax.set_ylabel("Acceptance Rate")
    ax.legend()
    save_fig(fig, out_folder / "accept_rate_vs_error_ratio.png")

    # ------------------ Plot 2: Mahalanobis Mean vs Error Ratio ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"Mahalanobis Distance Mean vs Error Ratio (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["error_ratio"], df["pos_mahal_mean"], ls, marker=m,
            color="black", label="Position")

    m, ls = bw_style(1)
    ax.plot(df["error_ratio"], df["vel_mahal_mean"], ls, marker=m,
            fillstyle="none", color="gray", label="Velocity")

    ax.set_xlabel("Error Ratio")
    ax.set_ylabel("Mean Mahalanobis Distance")
    ax.legend()
    save_fig(fig, out_folder / "mahalanobis_vs_error_ratio.png")

    # ------------------ Plot 3: Innovation Magnitude ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"Innovation Magnitude vs Error Ratio (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["error_ratio"], df["mean_innov_pos"], ls, marker=m,
            color="black", label="Position Innov")

    m, ls = bw_style(1)
    ax.plot(df["error_ratio"], df["mean_innov_vel"], ls, marker=m,
            fillstyle="none", color="gray", label="Velocity Innov")

    ax.set_xlabel("Error Ratio")
    ax.set_ylabel("Mean Innovation (m or m/s)")
    ax.legend()
    save_fig(fig, out_folder / "innovation_vs_error_ratio.png")

    # ------------------ Plot 4: Covariance Trace ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"trace(P) vs Error Ratio (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["error_ratio"], df["mean_tracep"], ls, marker=m, color="black")

    if "std_traceP" in df.columns:
        ax.fill_between(df["error_ratio"],
                        df["mean_tracep"] - df["std_tracep"],
                        df["mean_tracep"] + df["std_tracep"],
                        color="lightgray", alpha=0.3, label="±1σ")

    ax.set_xlabel("Error Ratio")
    ax.set_ylabel("Mean trace(P)")
    ax.legend()
    save_fig(fig, out_folder / "traceP_vs_error_ratio.png")

    # ------------------ Plot 5: Acceptance vs corrupt percentage ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"Acceptance Rate vs Corrupted Sensor % (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["corrupt_pct"], df["pos_accept_rate"], ls, marker=m,
            color="black", label="Pos Acceptance")

    m, ls = bw_style(1)
    ax.plot(df["corrupt_pct"], df["vel_accept_rate"], ls, marker=m,
            fillstyle="none", color="gray", label="Vel Acceptance")

    ax.set_xlabel("Corrupted Sensors (%)")
    ax.set_ylabel("Acceptance Rate")
    ax.legend()
    save_fig(fig, out_folder / "accept_rate_vs_corrupt_pct.png")

    # ------------------ Plot 6: Mahalanobis vs corrupt percentage ------------------
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"Mahalanobis Mean vs Corrupted Sensor % (n_corrupt={nc})")

    m, ls = bw_style(0)
    ax.plot(df["corrupt_pct"], df["pos_mahal_mean"], ls, marker=m,
            color="black", label="Position")

    m, ls = bw_style(1)
    ax.plot(df["corrupt_pct"], df["vel_mahal_mean"], ls, marker=m,
            fillstyle="none", color="gray", label="Velocity")

    ax.set_xlabel("Corrupted Sensors (%)")
    ax.set_ylabel("Mean Mahalanobis Distance")
    ax.legend()
    save_fig(fig, out_folder / "mahalanobis_vs_corrupt_pct.png")


# ----------------------------------------------------------
# Main
# ----------------------------------------------------------
def main():
    files = sorted(COMBINED_DIR.glob("combined_n_corrupt_*.csv"))
    if not files:
        print(f"[ERROR] No combined CSVs found in {COMBINED_DIR}")
        sys.exit(1)

    print(f"[INFO] Found {len(files)} combined files.")

    for csv_path in files:
        try:
            nc = int(csv_path.stem.split("_")[-1])
        except ValueError:
            print(f"[WARN] Skipping malformed filename: {csv_path}")
            continue

        df = pd.read_csv(csv_path)
        if df.empty:
            print(f"[WARN] Empty CSV: {csv_path}")
            continue

        out_folder = OUT_DIR / f"n_corrupt_{nc}"
        plot_for_corruption_level(df, nc, out_folder)

    print("\n✅ All combined plots generated.")


if __name__ == "__main__":
    main()

