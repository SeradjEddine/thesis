#!/bin/bash
# Master script to process all OXTS sets and generate corrupted CSVs.
# Logs one global entry per full dataset generation run.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

RAW_DIR="$ROOT_DIR/Data/sets_raw"
CSV_DIR="$ROOT_DIR/Data/sets_csv"
PY_SCRIPT="$SCRIPT_DIR/prepare_kitti_oxts.py"
GLOBAL_LOG="$ROOT_DIR/Data/experiment_log.csv"

# Default parameters (can be overridden)
TOTAL_FILES=${1:-5}
N_CORRUPT=${2:-0}
ERROR_RATE=${3:-0.00}
RUN_ID=${4:-0001}

echo "Processing all sets from: $RAW_DIR"
echo "Output stored in: $CSV_DIR"
echo "Global log file: $GLOBAL_LOG"
echo "Parameters: total=$TOTAL_FILES, corrupt=$N_CORRUPT, error_rate=$ERROR_RATE, run_id=$RUN_ID"
echo ""

# Ensure directories exist
mkdir -p "$CSV_DIR"
mkdir -p "$(dirname "$GLOBAL_LOG")"

# Initialize global log if missing
if [ ! -f "$GLOBAL_LOG" ]; then
    echo "timestamp,run_id,n_total,n_corrupt,error_rate,seed" > "$GLOBAL_LOG"
    echo "Created new global log file: $GLOBAL_LOG"
fi

# Default seed for reproducibility
SEED=42

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
                --seed "$SEED" \
                --run_id "$RUN_ID"

            echo "Finished $category/$seq_name"
            echo ""
        fi
    done
done

# Log one global line for the whole run
timestamp=$(date '+%Y-%m-%d %H:%M:%S')
echo "$timestamp,$RUN_ID,$TOTAL_FILES,$N_CORRUPT,$ERROR_RATE,$SEED" >> "$GLOBAL_LOG"

echo "✅ Logged run $RUN_ID to $GLOBAL_LOG"
echo "✅ All sets processed successfully."

