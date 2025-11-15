#!/bin/bash
# Sim Master: runs full pipeline across increasing corruption/error levels.
# Usage:
#   ./sim_master.sh <max_num_sensors> <max_num_corrupt> <max_error_rate> <rate_increment> [seed] [--vis]

set -e

# --- Parse required args ---
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

# --- Optional seed / visualizer flag ---
if [ $# -ge 5 ]; then
    if [[ "$5" == "--vis" ]]; then
        VIS_FLAG="--vis"
    else
        USER_SEED="$5"
    fi
fi
if [ $# -ge 6 ] && [[ "$6" == "--vis" ]]; then
    VIS_FLAG="--vis"
fi

# --- Defaults ---
SIM_JOBS=10
PLOT_JOBS=10

echo "=========================================="
echo " Simulation Master Sweep"
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

# --- Single clean edge-case: no corruption, no error ---
if (( MAX_CORRUPT == 0 )) && (( $(echo "$MAX_ERROR == 0" | bc -l) )); then
    SEED=${USER_SEED:-$RANDOM}
    echo ">>> Single baseline run: total=$MAX_SENSORS, corrupt=0, error_rate=0, seed=$SEED"
    ./sim_full_pipeline.sh "$MAX_SENSORS" 0 0 "$SIM_JOBS" "$PLOT_JOBS" "$SEED" "$VIS_FLAG"
    echo "✅ Completed baseline run."
    exit 0
fi

# --- Main sweep ---
for ((CORRUPT=0; CORRUPT<=MAX_CORRUPT; CORRUPT++)); do
    if (( CORRUPT == 0 )); then
        # For zero corruption → run once with error = 0
        SEED=${USER_SEED:-$RANDOM}
        echo ""
        echo ">>> Baseline (no corruption): total=$MAX_SENSORS, corrupt=0, error_rate=0, seed=$SEED"
        ./sim_full_pipeline.sh "$MAX_SENSORS" 0 0 "$SIM_JOBS" "$PLOT_JOBS" "$SEED" "$VIS_FLAG"
        continue
    fi

    # For corrupted cases → sweep error levels
    if (( $(echo "$DELTA_ERROR <= 0" | bc -l) )); then
        echo "⚠️  Invalid DELTA_ERROR ($DELTA_ERROR) for corruption sweep; skipping error iteration."
        break
    fi

    ERROR=0.0
    while (( $(echo "$ERROR <= $MAX_ERROR" | bc -l) )); do
        SEED=${USER_SEED:-$RANDOM}
        echo ""
        echo ">>> Running: total=$MAX_SENSORS, corrupt=$CORRUPT, error_rate=$ERROR, seed=$SEED"
        ./sim_full_pipeline.sh "$MAX_SENSORS" "$CORRUPT" "$ERROR" "$SIM_JOBS" "$PLOT_JOBS" "$SEED" "$VIS_FLAG"
        ERROR=$(echo "$ERROR + $DELTA_ERROR" | bc -l)
    done
done

echo ""
echo "✅ All pipeline sweeps completed."

