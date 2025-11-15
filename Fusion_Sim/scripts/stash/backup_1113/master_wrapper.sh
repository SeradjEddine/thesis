#!/bin/bash
# Master wrapper: runs full pipeline across increasing corruption/error levels.
# Usage:
#   ./master_wrapper.sh <max_num_sensors> <max_num_corrupt> <max_error_rate> <rate_increment> [seed] [--vis]
#
# Example:
#   ./master_wrapper.sh 10 3 0.08 0.02 123 --vis

set -e

# --- Validate and parse required args ---
if [ $# -lt 4 ]; then
    echo "Usage: $0 <max_num_sensors> <max_num_corrupt> <max_error_rate> <rate_increment> [seed] [--vis]"
    exit 1
fi

MAX_SENSORS=$1
MAX_CORRUPT=$2
MAX_ERROR=$3
DELTA_ERROR=$4
USER_SEED=""
VIS_FLAG="--no-vis"

# --- Optional seed and visualizer flag ---
if [ $# -ge 5 ]; then
    if [[ "$5" == "--vis" ]]; then
        VIS_FLAG="--vis"
    else
        USER_SEED="$5"
    fi
fi

if [ $# -ge 6 ]; then
    if [[ "$6" == "--vis" ]]; then
        VIS_FLAG="--vis"
    fi
fi

# --- Default concurrency ---
SIM_JOBS=10
PLOT_JOBS=10

echo "=========================================="
echo " Master Wrapper: Sweeping Corruption/Error"
echo " Total sensors:     $MAX_SENSORS"
echo " Max corrupt:       $MAX_CORRUPT"
echo " Max error rate:    $MAX_ERROR"
echo " Error increment:   $DELTA_ERROR"
if [ -n "$USER_SEED" ]; then
    echo " Using fixed seed:  $USER_SEED"
else
    echo " Randomizing seed per iteration"
fi
echo " Visualizer mode:   $VIS_FLAG"
echo "=========================================="

# --- Sweep across corruption & error rates ---
for ((CORRUPT=1; CORRUPT<=MAX_CORRUPT; CORRUPT++)); do
    ERROR=0.0
    while (( $(echo "$ERROR <= $MAX_ERROR" | bc -l) )); do
        # Choose seed
        if [ -n "$USER_SEED" ]; then
            SEED=$USER_SEED
        else
            SEED=$RANDOM
        fi

        echo ""
        echo ">>> Running pipeline: total=$MAX_SENSORS, corrupt=$CORRUPT, error_rate=$ERROR, seed=$SEED"
        ./master.sh "$MAX_SENSORS" "$CORRUPT" "$ERROR" "$SIM_JOBS" "$PLOT_JOBS" "$SEED" "$VIS_FLAG"

        ERROR=$(echo "$ERROR + $DELTA_ERROR" | bc -l)
    done
done

echo ""
echo "✅ All pipeline sweeps completed."

