#!/usr/bin/env python3
"""
visualize_master.py
Visualize metrics from simulation experiments.

Usage:
    python3 visualize_master.py
"""

import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# ----------------------------
# Configuration
# ----------------------------
DATA_DIR = "../Data"
SIM_RESULTS_DIR = os.path.join(DATA_DIR, "sim_results_csv")
PLOT_DIR = os.path.join(DATA_DIR, "sim_results_plot")
os.makedirs(PLOT_DIR, exist_ok=True)

# Metrics to visualize
METRICS = [
    "mean_maha_pos", "max_maha_pos",
    "mean_maha_vel", "max_maha_vel",
    "reject_ratio", "rmse_pos", "rmse_vel",
    "mean_scale_R_gps", "mean_scale_Q", "mean_scale_gate"
]

# ----------------------------
# Helper functions
# ----------------------------
def load_all_analysis_logs():
    """Recursively load all analysis_log.csv files under SIM_RESULTS_DIR."""
    all_entries = []
    for root, dirs, files in os.walk(SIM_RESULTS_DIR):
        for fname in files:
            if fname == "analysis_log.csv":
                fpath = os.path.join(root, fname)
                try:
                    df = pd.read_csv(fpath)
                    all_entries.append(df)
                except Exception as e:
                    print(f"[WARN] Could not read {fpath}: {e}")
    if not all_entries:
        raise FileNotFoundError("No analysis_log.csv files found under sim_results_csv/")
    df = pd.concat(all_entries, ignore_index=True)

    # Ensure numeric types for relevant parameters
    for col in ["error_rate", "n_corrupt"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def make_plot(df, metric, x="error_rate", hue="n_corrupt"):
    """Make and save a line plot for a metric."""
    if metric not in df.columns:
        print(f"[WARN] Metric '{metric}' not found in data, skipping.")
        return
    if x not in df.columns or hue not in df.columns:
        print(f"[WARN] Missing columns for plot ({x}, {hue}), skipping {metric}.")
        return

    plt.figure(figsize=(8, 5))
    sns.lineplot(data=df, x=x, y=metric, hue=hue, marker="o")
    plt.title(f"{metric} vs {x}")
    plt.xlabel(x)
    plt.ylabel(metric)
    plt.grid(True)
    plt.tight_layout()
    plot_file = os.path.join(PLOT_DIR, f"{metric}_vs_{x}.png")
    plt.savefig(plot_file)
    plt.close()
    print(f"Saved plot: {plot_file}")

# ----------------------------
# Main
# ----------------------------
def main():
    df = load_all_analysis_logs()
    print(f"Loaded {len(df)} entries from all analysis logs.")

    for metric in METRICS:
        make_plot(df, metric, x="error_rate", hue="n_corrupt")
        make_plot(df, metric, x="n_corrupt", hue="error_rate")

    print("✅ All plots generated in:", PLOT_DIR)


if __name__ == "__main__":
    main()

