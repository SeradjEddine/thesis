#!/bin/bash
# Wrapper to process all sets under Data/sets_raw/* and save to Data/sets_csv/*
# Automatically loads error_injection_config.yaml from this script's directory
# Now includes parallel processing.

# Resolve paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

RAW_DIR="$ROOT_DIR/Data/sets_raw"
CSV_DIR="$ROOT_DIR/Data/sets_csv"
PY_SCRIPT="$SCRIPT_DIR/prepare_kitti_oxts_adv.py"
CONFIG_PATH="$SCRIPT_DIR/error_injection_config.yaml"

# -------------------------------
# CLI parameters
# -------------------------------
TOTAL_FILES=${1:-5}
N_CORRUPT=${2:-0}
ERROR_RATE=${3:-0}
SEED=${4:-42}

# Parallelism level (edit as needed)
MAX_JOBS=8

if [ ! -f "$CONFIG_PATH" ]; then
    echo "[ERROR] Default config file not found at:"
    echo "  $CONFIG_PATH"
    exit 1
fi

echo "Processing all sets from: $RAW_DIR"
echo "Output directory: $CSV_DIR"
echo "Parameters:"
echo "  total files     = $TOTAL_FILES"
echo "  corrupted files = $N_CORRUPT"
echo "  using config    = $CONFIG_PATH"
echo "  seed            = $SEED"
echo "  max parallel    = $MAX_JOBS"
echo ""

# Tracks running jobs
job_count=0

run_job() {
    category=$1
    seq_dir=$2
    seq_name=$3
    output_path=$4

    echo "[JOB] Start: $category/$seq_name"

    python3 "$PY_SCRIPT" \
        --input_dir "$seq_dir/oxts" \
        --output_dir "$output_path" \
        --total "$TOTAL_FILES" \
        --n_corrupt "$N_CORRUPT" \
        --error_rate "$ERROR_RATE" \
        --seed "$SEED"

    echo "[JOB] Finished: $category/$seq_name"
}

# ------------------------------------
# Process city, residential, road
# ------------------------------------
for category in city residential road; do
    CATEGORY_RAW="$RAW_DIR/$category"
    CATEGORY_OUT="$CSV_DIR/$category"
    mkdir -p "$CATEGORY_OUT"

    for seq_dir in "$CATEGORY_RAW"/*; do
        if [ -d "$seq_dir" ]; then
            seq_name=$(basename "$seq_dir")
            OUTPUT_PATH="$CATEGORY_OUT/$seq_name"
            mkdir -p "$OUTPUT_PATH"

            # Launch background job
            run_job "$category" "$seq_dir" "$seq_name" "$OUTPUT_PATH" &

            # Throttle to MAX_JOBS
            job_count=$((job_count + 1))
            if [ "$job_count" -ge "$MAX_JOBS" ]; then
                wait -n  # wait for at least ONE job to finish
                job_count=$((job_count - 1))
            fi
        fi
    done
done

# Wait for any last remaining jobs
wait

echo "✅ All sets processed!"

