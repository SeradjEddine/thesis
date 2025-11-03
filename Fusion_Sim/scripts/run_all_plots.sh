#!/bin/bash
# Run plotting script for all simulation results
# Usage: ./run_all_plots.sh [sim_results_dir] [plots_dir] [max_jobs]

set -e

SIM_RESULTS_DIR="../Data/sim_results_csv"
PLOTS_DIR="../Data/sim_results_plot"
MAX_JOBS=10

[ $# -ge 1 ] && SIM_RESULTS_DIR="$1"
[ $# -ge 2 ] && PLOTS_DIR="$2"
[ $# -ge 3 ] && MAX_JOBS="$3"

echo "Processing simulation results from $SIM_RESULTS_DIR"
echo "Plots will be saved in $PLOTS_DIR"
echo "Max parallel jobs: $MAX_JOBS"

function wait_for_jobs() {
    local current_jobs
    while true; do
        current_jobs=$(jobs -rp | wc -l)
        if [ "$current_jobs" -lt "$MAX_JOBS" ]; then
            break
        fi
        sleep 1
    done
}

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

        wait_for_jobs
    done
done

wait
echo "All plotting completed."

