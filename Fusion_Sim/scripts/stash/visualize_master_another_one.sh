#!/bin/bash
# File: scripts/batch_plot_results.sh
ROOT_DIR=${1:-../Data/sim_results_csv}

for group in city residential road; do
    for dataset in "$ROOT_DIR/$group"/*; do
        if [ -d "$dataset" ]; then
            echo "[RUN] $dataset"
            python3 "$(dirname "$0")/visualize_master_another_one.py" "$dataset"
        fi
    done
done

