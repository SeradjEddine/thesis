#!/usr/bin/env python3
"""
visualize_experiment_results.py

Load master log and analysis log, merge them, and plot metrics as a function
of error_rate and number of corrupted files.
"""

import pandas as pd
import matplotlib.pyplot as plt
import os
import seaborn as sns

# ----------------------
# Paths
# ----------------------
DATA_DIR = "../Data"
MASTER_LOG = os.path.join(DATA_DIR, "experiment_log_master.csv")
ANALYSIS_LOG = os.path.join(DATA_DIR, "experiment_analysis.csv")
PLOT_DIR = os.path.join(DATA_DIR, "sim_results_plot")
os.makedirs(PLOT_DIR, exist_ok=True)

# ----------------------
# Load logs
# ----------------------
master_df = pd.read_csv(MASTER_LOG)
analysis_df = pd.read_csv(ANALYSIS_LOG)

# Merge on run_id
df = pd.merge(master_df, analysis_df, on="run_id")

print(f"Loaded {len(df)} combined experiment entries.")

# ----------------------
# Plotting metrics vs error_rate
# ----------------------
metrics = [col for col in df.columns if col not in ["run_id","n_total","n_corrupt","error_rate","seed","timestamp"]]

for metric in metrics:
    plt.figure(figsize=(10,6))
    sns.lineplot(data=df, x="error_rate", y=metric, hue="n_corrupt", marker="o")
    plt.title(f"{metric} vs Error Rate (separate curves for n_corrupt)")
    plt.xlabel("Error rate")
    plt.ylabel(metric)
    plt.grid(True)
    out_file = os.path.join(PLOT_DIR, f"{metric}_vs_error_rate.png")
    plt.savefig(out_file)
    plt.close()
    print(f"Saved {out_file}")

# ----------------------
# Optional: heatmaps
# ----------------------
for metric in metrics:
    pivot = df.pivot_table(index="n_corrupt", columns="error_rate", values=metric)
    plt.figure(figsize=(10,6))
    sns.heatmap(pivot, annot=True, fmt=".3f", cmap="viridis")
    plt.title(f"{metric} heatmap (n_corrupt vs error_rate)")
    out_file = os.path.join(PLOT_DIR, f"{metric}_heatmap.png")
    plt.savefig(out_file)
    plt.close()
    print(f"Saved {out_file}")

print(f"All plots saved to {PLOT_DIR}")

