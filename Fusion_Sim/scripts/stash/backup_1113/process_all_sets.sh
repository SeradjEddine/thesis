#!/bin/bash
# Shell script to process all OXTS sets under sets_raw/* and save to sets_csv/*

# Resolve the script directory to make relative paths robust
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

RAW_DIR="$ROOT_DIR/Data/sets_raw"
CSV_DIR="$ROOT_DIR/Data/sets_csv"
PY_SCRIPT="$SCRIPT_DIR/prepare_kitti_oxts.py"

# Default parameters (can be overridden via command line)
TOTAL_FILES=${1:-5}
N_CORRUPT=${2:-0}
ERROR_RATE=${3:-0.00s}
SEED=${4:-42}

echo "Processing all sets from $RAW_DIR"
echo "Output will be stored under $CSV_DIR"
echo "Parameters: total=$TOTAL_FILES, corrupt=$N_CORRUPT, error_rate=$ERROR_RATE, seed=$SEED"
echo ""

for category in city residential road; do
    CATEGORY_RAW="$RAW_DIR/$category"
    CATEGORY_OUT="$CSV_DIR/$category"
    mkdir -p "$CATEGORY_OUT"

    for seq_dir in "$CATEGORY_RAW"/*; do
        if [ -d "$seq_dir" ]; then
            seq_name=$(basename "$seq_dir")
            OUTPUT_PATH="$CATEGORY_OUT/$seq_name"
            mkdir -p "$OUTPUT_PATH"

            echo "Processing $category/$seq_name ..."
            python3 "$PY_SCRIPT" \
                --input_dir "$seq_dir/oxts" \
                --output_dir "$OUTPUT_PATH" \
                --total "$TOTAL_FILES" \
                --n_corrupt "$N_CORRUPT" \
                --error_rate "$ERROR_RATE" \
                --seed "$SEED"
            echo "Finished $category/$seq_name"
            echo ""
        fi
    done
done

echo "✅ All sets processed!"

