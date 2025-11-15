#!/bin/bash
set -e

# --- Default paths ---
DEFAULT_INPUT="../data/sim_results_csv"
DEFAULT_OUTPUT="../data/sim_results_csv"

# --- Read CLI args ---
INPUT_DIR=${1:-$DEFAULT_INPUT}
OUTPUT_DIR=${2:-$DEFAULT_OUTPUT}
N_FILE=${3:-100}
N_CORRUPT=${4:-5}
ERROR_RATIO=${5:-0.05}
SEED=${6:-42}

echo "======================================="
echo " Batch EKF Metrics Evaluation"
echo " Input Dir:    $INPUT_DIR"
echo " Output Dir:   $OUTPUT_DIR"
echo " Metadata:     n_file=$N_FILE  n_corrupt=$N_CORRUPT  ratio=$ERROR_RATIO  seed=$SEED"
echo "======================================="

# --- Run through groups (city / road / residential) ---
for GROUP in "city" "road" "residential"; do
    GROUP_PATH="$INPUT_DIR/$GROUP"
    if [ ! -d "$GROUP_PATH" ]; then
        echo "[WARN] Missing group directory: $GROUP_PATH"
        continue
    fi

    echo -e "\n▶ Processing group: $GROUP"

    # Loop through each OXTS subfolder
    for DATASET_PATH in "$GROUP_PATH"/*; do
        if [ -d "$DATASET_PATH" ]; then
            echo "   - Running eval_metrics.py on $(basename "$DATASET_PATH")"
            python3 "$(dirname "$0")/eval_metrics.py" \
                "$DATASET_PATH" \
                "$DATASET_PATH" \
                "$N_FILE" "$N_CORRUPT" "$ERROR_RATIO" "$SEED" "$GROUP"
        fi
    done
done

echo -e "\n✅ Batch analysis completed for all groups."

