#!/bin/bash
# Full pipeline: process CSVs → run simulations → generate plots
# Usage: ./run_full_pipeline.sh [raw_csv_dir] [sim_results_dir] [plots_dir] [sim_jobs] [plot_jobs]

set -e

# Default folders
RAW_CSV_DIR="../Data/sets_csv"          # already processed CSVs (or produced with previous scripts)
SIM_RESULTS_DIR="../Data/sim_results_csv"
PLOTS_DIR="../Data/sim_results_plot"

# Max parallel jobs
SIM_JOBS=4
PLOT_JOBS=4

# Override defaults with CLI arguments
[ $# -ge 1 ] && RAW_CSV_DIR="$1"
[ $# -ge 2 ] && SIM_RESULTS_DIR="$2"
[ $# -ge 3 ] && PLOTS_DIR="$3"
[ $# -ge 4 ] && SIM_JOBS="$4"
[ $# -ge 5 ] && PLOT_JOBS="$5"

echo "=== Full Pipeline ==="
echo "Input CSVs: $RAW_CSV_DIR"
echo "Simulation output: $SIM_RESULTS_DIR"
echo "Plots output: $PLOTS_DIR"
echo "Max simulation jobs: $SIM_JOBS"
echo "Max plotting jobs: $PLOT_JOBS"

# --- Helper to limit concurrent jobs ---
function wait_for_jobs() {
    local max_jobs=$1
    while true; do
        current_jobs=$(jobs -rp | wc -l)
        if [ "$current_jobs" -lt "$max_jobs" ]; then
            break
        fi
        sleep 1
    done
}

# --- Step 1: Run simulations ---
echo ">>> Running Fusion Simulations ..."
for category_dir in "$RAW_CSV_DIR"/*; do
    [ -d "$category_dir" ] || continue
    category=$(basename "$category_dir")

    for seq_dir in "$category_dir"/*; do
        [ -d "$seq_dir" ] || continue
        sequence=$(basename "$seq_dir")

        echo "Simulating $category/$sequence ..."

        variation_count=$(ls "$seq_dir"/oxts*.csv 2>/dev/null | wc -l)
        if [ "$variation_count" -eq 0 ]; then
            echo "  No CSV files found, skipping."
            continue
        fi

        output_path="$SIM_RESULTS_DIR/$category/$sequence"
        mkdir -p "$output_path"

        ../build/fusion_sim "$variation_count" "$seq_dir/oxts" "$output_path" &
        wait_for_jobs $SIM_JOBS
    done
done

wait
echo ">>> All simulations completed."

# --- Step 2: Generate plots ---
echo ">>> Generating plots ..."
for category_dir in "$SIM_RESULTS_DIR"/*; do
    [ -d "$category_dir" ] || continue
    category=$(basename "$category_dir")

    for seq_dir in "$category_dir"/*; do
        [ -d "$seq_dir" ] || continue
        sequence=$(basename "$seq_dir")

        echo "Plotting $category/$sequence ..."
        output_path="$PLOTS_DIR/$category/$sequence"
        mkdir -p "$output_path"

        python3 plot_results.py "$seq_dir" "$output_path" &
        wait_for_jobs $PLOT_JOBS
    done
done

wait
echo "=== Full pipeline completed! ==="

