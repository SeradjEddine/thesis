#!/bin/bash
# Run Fusion Sim for all sequences concurrently
# Usage: ./run_fusion_sims.sh [base_input_dir] [base_output_dir] [max_jobs]

set -e

# Default paths
BASE_INPUT_DIR="../Data/sets_csv"
BASE_OUTPUT_DIR="../Data/sim_results_csv"
MAX_JOBS=4  # default number of parallel simulations

# Override defaults if args provided
[ $# -ge 1 ] && BASE_INPUT_DIR="$1"
[ $# -ge 2 ] && BASE_OUTPUT_DIR="$2"
[ $# -ge 3 ] && MAX_JOBS="$3"

echo "Processing all sets from $BASE_INPUT_DIR"
echo "Output will be stored under $BASE_OUTPUT_DIR"
echo "Max parallel jobs: $MAX_JOBS"

# Function to limit the number of concurrent jobs
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

# Loop over categories: city, residential, road
for category_dir in "$BASE_INPUT_DIR"/*; do
    [ -d "$category_dir" ] || continue
    category=$(basename "$category_dir")

    for seq_dir in "$category_dir"/*; do
        [ -d "$seq_dir" ] || continue
        sequence=$(basename "$seq_dir")

        echo "Processing $category/$sequence ..."

        # Count variations
        variation_count=$(ls "$seq_dir"/oxts*.csv 2>/dev/null | wc -l)
        if [ "$variation_count" -eq 0 ]; then
            echo "  No CSV files found in $seq_dir, skipping."
            continue
        fi

        # Build output directory
        output_path="$BASE_OUTPUT_DIR/$category/$sequence"
        mkdir -p "$output_path"

        # Run simulator in background
        time ../build/fusion_sim "$variation_count" "$seq_dir/oxts" "$output_path" &

        # Limit number of concurrent jobs
        wait_for_jobs
    done
done

# Wait for all background jobs to finish
wait
echo "All simulations completed."

