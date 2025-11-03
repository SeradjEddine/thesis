#!/bin/bash
# Master pipeline: process raw OXTS CSVs, run simulations, generate plots
# Usage: ./run_full_pipeline.sh [n_total_csvs] [n_corrupt] [error_rate] [sim_jobs] [plot_jobs]

set -e

# Default parameters
N_TOTAL=5
N_CORRUPT=0
ERROR_RATE=0.03
SIM_JOBS=4
PLOT_JOBS=4

# Input/output folders
RAW_DIR="../Data/sets_raw"
CSV_DIR="../Data/sets_csv"
SIM_RESULTS_DIR="../Data/sim_results_csv"
PLOTS_DIR="../Data/sim_results_plot"

# Override defaults with CLI arguments
[ $# -ge 1 ] && N_TOTAL="$1"
[ $# -ge 2 ] && N_CORRUPT="$2"
[ $# -ge 3 ] && ERROR_RATE="$3"
[ $# -ge 4 ] && SIM_JOBS="$4"
[ $# -ge 5 ] && PLOT_JOBS="$5"

echo "=== Full Pipeline ==="
echo "Raw data dir: $RAW_DIR"
echo "Processed CSVs: $CSV_DIR (total=$N_TOTAL, corrupt=$N_CORRUPT, error_rate=$ERROR_RATE)"
echo "Simulation results: $SIM_RESULTS_DIR (max jobs=$SIM_JOBS)"
echo "Plots output: $PLOTS_DIR (max jobs=$PLOT_JOBS)"

# --- Step 1: Process all sets ---
echo ">>> Step 1: Generating CSVs ..."
./process_all_sets.sh "$N_TOTAL" "$N_CORRUPT" "$ERROR_RATE"

# --- Step 2: Run Fusion simulations ---
echo ">>> Step 2: Running Fusion Simulations ..."
./run_fusion_sims.sh "$CSV_DIR" "$SIM_RESULTS_DIR" "$SIM_JOBS"

# --- Step 3: Generate plots ---
echo ">>> Step 3: Generating plots ..."
./run_all_plots.sh "$SIM_RESULTS_DIR" "$PLOTS_DIR" "$PLOT_JOBS"

echo "=== Full pipeline completed successfully ==="

