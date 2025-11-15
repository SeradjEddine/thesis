#!/usr/bin/env python3
"""
Plot matrices per dataset set, grouped by n_corrupt.

- Iterates through all results_metrics.csv files under ../Data/sim_results_csv/
- Produces separate plots for each metric, one curve per n_corrupt
- Color added, markers preserved
"""

import argparse
from pathlib import Path
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

MARKERS = ['o', 's', '^', 'D', 'x', '+']
LINESTYLES = ['-', '--', '-.', ':']
COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']

def style(idx):
    return MARKERS[idx % len(MARKERS)], LINESTYLES[idx % len(LINESTYLES)], COLORS[idx % len(COLORS)]

def save_fig(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[SAVED] {path}")

def plot_metric(df, metric, xlabel, ylabel, title_prefix, out_folder):
    """
    Plot a single metric grouped by n_corrupt.
    """
    if "n_corrupt" not in df.columns:
        df["n_corrupt"] = 0  # default if missing

    out_folder.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7,5))
    ax.set_title(f"{title_prefix}")

    for idx, (nc, group) in enumerate(sorted(df.groupby("n_corrupt"))):
        group = group.sort_values("error_ratio")
        m, ls, c = style(idx)
        ax.plot(group["error_ratio"], group[metric], linestyle=ls, marker=m, color=c, label=f"n_corrupt={nc}")

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()
    save_fig(fig, out_folder / f"{metric}.png")

def main():
    parser = argparse.ArgumentParser(description="Plot per-set matrices grouped by n_corrupt")
    parser.add_argument("--file", type=str, help="Single CSV to plot")
    parser.add_argument("--dir", type=str, help="Directory to scan for results_metrics.csv")
    args = parser.parse_args()

    SCRIPT_DIR = Path(__file__).resolve().parent
    DEFAULT_DATA_DIR = SCRIPT_DIR.parent / "Data" / "sim_results_csv"

    # Determine CSV files to process
    if args.file:
        csv_paths = [Path(args.file)]
    elif args.dir:
        csv_paths = list(Path(args.dir).rglob("results_metrics.csv"))
    else:
        csv_paths = list(DEFAULT_DATA_DIR.rglob("results_metrics.csv"))

    if not csv_paths:
        print(f"[ERROR] No CSV files found in {DEFAULT_DATA_DIR}")
        return

    print(f"[INFO] Found {len(csv_paths)} CSV files.")

    for csv_path in csv_paths:
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f"[WARN] Failed to read {csv_path}: {e}")
            continue
        if df.empty:
            print(f"[WARN] Empty CSV: {csv_path}")
            continue

        # Determine output folder        
        rel_path = csv_path.relative_to(DEFAULT_DATA_DIR)  
        out_folder = SCRIPT_DIR.parent / "Data" / "plots_per_set" / rel_path.parent

        # Derived fields
        if "corrupt_pct" not in df.columns and "n_file" in df.columns:
            df["corrupt_pct"] = (df["n_corrupt"] / df["n_file"]) * 100.0

        # Plot each metric separately
        metrics_to_plot = [
            ("pos_accept_rate", "Error Ratio", "Acceptance Rate", "Position Acceptance Rate"),
            ("vel_accept_rate", "Error Ratio", "Acceptance Rate", "Velocity Acceptance Rate"),
            ("pos_mahal_mean", "Error Ratio", "Mean Mahalanobis Distance", "Position Mahalanobis Mean"),
            ("vel_mahal_mean", "Error Ratio", "Mean Mahalanobis Distance", "Velocity Mahalanobis Mean"),
            ("mean_innov_pos", "Error Ratio", "Mean Innovation (m or m/s)", "Position Innovation"),
            ("mean_innov_vel", "Error Ratio", "Mean Innovation (m or m/s)", "Velocity Innovation"),
            ("mean_traceP", "Error Ratio", "Mean trace(P)", "Covariance Trace")
        ]

        for metric, xlabel, ylabel, title in metrics_to_plot:
            if metric in df.columns:
                plot_metric(df, metric, xlabel, ylabel, title, out_folder)
            else:
                print(f"[WARN] Metric '{metric}' not found in {csv_path}")

    print("\n✅ All plots generated.")

if __name__ == "__main__":
    main()

