#!/bin/bash
# Run plotting script for all simulation results
# Usage: ./run_all_plots.sh [sim_results_dir] [plots_dir] [max_jobs] [n_total] [n_corrupt] [error_rate] [seed]

set -e

SIM_RESULTS_DIR="../Data/sim_results_csv"
PLOTS_DIR="../Data/sim_results_plot"
MAX_JOBS=10
N_TOTAL=0
N_CORRUPT=0
ERROR_RATE=0.00
SEED=0

[ $# -ge 1 ] && SIM_RESULTS_DIR="$1"
[ $# -ge 2 ] && PLOTS_DIR="$2"
[ $# -ge 3 ] && MAX_JOBS="$3"
[ $# -ge 4 ] && N_TOTAL="$4"
[ $# -ge 5 ] && N_CORRUPT="$5"
[ $# -ge 6 ] && ERROR_RATE="$6"
[ $# -ge 7 ] && SEED="$7"

META_STR="${N_TOTAL}-${N_CORRUPT}-${ERROR_RATE}-${SEED}"

echo "Processing simulation results from $SIM_RESULTS_DIR"
echo "Plots will be saved in $PLOTS_DIR"
echo "Max parallel jobs: $MAX_JOBS"
echo "Metadata string: $META_STR"

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

        python3 plot_results.py "$seq_dir" "$output_path" "$META_STR" &
        wait_for_jobs
    done
done

wait
echo "All plotting completed."

