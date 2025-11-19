#!/usr/bin/env python3
"""
Clean scatter-plot generator for results_metrics.csv files with explicit per-metric winsorization.
Implements:
- Scatter only (no connecting lines)
- Removes n_corrupt plot (redundant)
- Winsorization per metric (instead of deleting outliers)
- Control dataset (n_corrupt = 0) plotted last for visibility
- Folder structure mirroring input hierarchy
- Removes row where n_corrupt=2 and error_rate=0
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
    "lines.markersize": 6,
    "legend.frameon": True,
})

MARKERS = ['o', 's', '^', 'D', 'x', '+']
COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']

def style(idx):
    return MARKERS[idx % len(MARKERS)], COLORS[idx % len(COLORS)]

def save_fig(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[SAVED] {path}")

def winsorize_series(series, lower=0.05, upper=0.95):
    """Clamp values to given percentile range per series."""
    lower_val = series.quantile(lower)
    upper_val = series.quantile(upper)
    return series.clip(lower=lower_val, upper=upper_val)

def plot_metric(df, metric, xlabel, ylabel, title, out_folder):
    """Scatter-only grouped plot, control group plotted last."""
    if metric not in df.columns:
        return

    df = df.copy().sort_values("error_ratio")

    # Apply explicit winsorization per metric
    df[metric] = winsorize_series(df[metric], 0.05, 0.95)

    # Extract groups, but plot control last
    groups = sorted(df.groupby("n_corrupt"), key=lambda x: x[0] != 0)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title(title)

    for idx, (nc, group) in enumerate(groups):
        m, c = style(idx)
        ax.scatter(group["error_ratio"], group[metric], marker=m, color=c, s=10,
                   label=f"n_corrupt={nc}")

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()
    save_fig(fig, out_folder / f"{metric}.png")

def plot_traceP_dB(df, out_folder):
    """Trace(P) in dB with explicit winsorization."""
    if "mean_traceP" not in df.columns:
        return

    df = df.copy().sort_values("error_ratio")

    # Apply explicit winsorization for traceP metrics
    df["mean_traceP"] = winsorize_series(df["mean_traceP"], 0.05, 0.95)
    df["std_traceP"] = winsorize_series(df["std_traceP"], 0.05, 0.95)

    groups = sorted(df.groupby("n_corrupt"), key=lambda x: x[0] != 0)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Covariance Trace (mean_traceP) in dB")

    for idx, (nc, group) in enumerate(groups):
        y = group["mean_traceP"].clip(lower=1e-12)
        y_db = 10 * np.log10(y)
        m, c = style(idx)
        ax.scatter(group["error_ratio"], y_db, marker=m, color=c, s=16,
                   label=f"n_corrupt={nc}")

    ax.set_xlabel("Error Ratio")
    ax.set_ylabel("mean_traceP (dB)")
    ax.legend()
    save_fig(fig, out_folder / "mean_traceP_dB.png")

def main():
    parser = argparse.ArgumentParser(description="Plot per-set metrics (scatter only)")
    parser.add_argument("--file", type=str, help="Single CSV to plot")
    parser.add_argument("--dir", type=str, help="Directory to scan for results_metrics.csv")
    args = parser.parse_args()

    SCRIPT_DIR = Path(__file__).resolve().parent
    DEFAULT_DATA_DIR = SCRIPT_DIR.parent / "Data" / "sim_results_csv"

    # Determine files
    if args.file:
        csv_paths = [Path(args.file)]
    elif args.dir:
        csv_paths = list(Path(args.dir).rglob("results_metrics.csv"))
    else:
        csv_paths = list(DEFAULT_DATA_DIR.rglob("results_metrics.csv"))

    if not csv_paths:
        print("[ERROR] No CSV files found.")
        return

    for csv_path in csv_paths:
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f"[WARN] Failed to read {csv_path}: {e}")
            continue

        if df.empty:
            continue

        # Remove row where n_corrupt=2 and error_rate=0
        df = df[~((df["n_corrupt"] == 2) & (df["error_ratio"] == 0))]

        # Output folder structure mirroring input
        rel_path = csv_path.relative_to(DEFAULT_DATA_DIR)
        out_folder = SCRIPT_DIR.parent / "Data" / "plots_per_set" / rel_path.parent

        # Metrics to plot (no n_corrupt!)
        metrics = [
            ("pos_accept_rate", "Error Ratio", "Acceptance", "Pos Acceptance Rate"),
            ("vel_accept_rate", "Error Ratio", "Acceptance", "Vel Acceptance Rate"),
            ("pos_mahal_mean", "Error Ratio", "Mahalanobis", "Pos Mahalanobis Mean"),
            ("vel_mahal_mean", "Error Ratio", "Mahalanobis", "Vel Mahalanobis Mean"),
            ("pos_mahal_std", "Error Ratio", "Mahalanobis Std", "Pos Mahalanobis Std"),
            ("vel_mahal_std", "Error Ratio", "Mahalanobis Std", "Vel Mahalanobis Std"),
            ("mean_innov_pos", "Error Ratio", "Innovation (m)", "Mean Pos Innovation"),
            ("std_innov_pos", "Error Ratio", "Innovation (m)", "Std Pos Innovation"),
            ("mean_innov_vel", "Error Ratio", "Innovation (m/s)", "Mean Vel Innovation"),
            ("std_innov_vel", "Error Ratio", "Innovation (m/s)", "Std Vel Innovation"),
            ("mean_traceP", "Error Ratio", "Trace(P)", "Covariance Trace Mean"),
            ("std_traceP", "Error Ratio", "Trace(P) Std", "Covariance Trace Std"),
        ]

        for metric, xlabel, ylabel, title in metrics:
            if metric in df.columns:
                plot_metric(df, metric, xlabel, ylabel, title, out_folder)

        # Special dB plot for traceP
        plot_traceP_dB(df, out_folder)

    print("\n✅ All plots generated.")

if __name__ == "__main__":
    main()

