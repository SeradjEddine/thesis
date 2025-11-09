#!/usr/bin/env bash
# Master automation script for full simulation experiments
# Handles sequential run_ids (0001, 0002, 0003, ...)
# Logs one line per database-wide run into experiment_log_master.csv

set -e
set -o pipefail

# ----------------------
# Defaults (override via CLI)
# ----------------------
N_TOTAL=${1:-5}
MAX_CORRUPT=${2:-2}
SEED_BASE=${3:-42}
SIM_JOBS=${4:-10}
PLOT_JOBS=${5:-10}

# Scripts
PROCESS_SCRIPT="./process_all_sets.sh"
FUSION_SCRIPT="./run_fusion_sims.sh"
ANALYZE_SCRIPT="./analyze_sim_results.py"
PLOT_SCRIPT="./run_all_plots.sh"

# Data directories
DATA_DIR="../Data"
CSV_DIR="$DATA_DIR/sets_csv"
SIM_RESULTS_DIR="$DATA_DIR/sim_results_csv"
PLOTS_DIR="$DATA_DIR/sim_results_plot"

# Global log
MASTER_LOG="$DATA_DIR/experiment_log_master.csv"
mkdir -p "$DATA_DIR"
if [ ! -f "$MASTER_LOG" ]; then
    echo "run_id,n_total,n_corrupt,error_rate,seed,timestamp" > "$MASTER_LOG"
fi

# ----------------------
# Helper functions
# ----------------------
format_er() { printf "%.3f" "$1"; }
format_id() { printf "%04d" "$1"; }

RUN_COUNTER=1  # sequential run counter

# --------------------------
# 0) CONTROL run
# --------------------------
echo ""
echo "--------------------------------------------------------------"
echo "CONTROL RUN: n_corrupt=0, error_rate=0.0"
echo "--------------------------------------------------------------"

RUN_ID=$(format_id "$RUN_COUNTER")
RUN_SEED=$((SEED_BASE + RUN_COUNTER))
TIMESTAMP=$(date +%Y-%m-%dT%H:%M:%S)

echo "$RUN_ID,$N_TOTAL,0,0.0,$RUN_SEED,$TIMESTAMP" >> "$MASTER_LOG"

bash "$PROCESS_SCRIPT" "$N_TOTAL" 0 0.0 "$RUN_ID"
bash "$FUSION_SCRIPT" "$CSV_DIR" "$SIM_RESULTS_DIR" "$SIM_JOBS"
python3 "$ANALYZE_SCRIPT" --run_id "$RUN_ID" --n_corrupt 0 --error_rate 0.0

RUN_COUNTER=$((RUN_COUNTER + 1))
echo "✅ Completed CONTROL run ($RUN_ID)."

# --------------------------
# 1..) Corruption / error rate sweeps
# --------------------------
ER_START=0.005
ER_STEP=0.005
ER_END=0.05

n_corrupt=1
while [ "$n_corrupt" -le "$MAX_CORRUPT" ]; do
    er=$ER_START
    while (( $(echo "$er <= $ER_END" | bc -l) )); do
        ER_FMT=$(format_er "$er")
        RUN_ID=$(format_id "$RUN_COUNTER")
        RUN_SEED=$((SEED_BASE + RUN_COUNTER))
        TIMESTAMP=$(date +%Y-%m-%dT%H:%M:%S)

        echo ""
        echo "--------------------------------------------------------------"
        echo "RUN $RUN_ID: n_corrupt=$n_corrupt, error_rate=$ER_FMT"
        echo "--------------------------------------------------------------"

        # Log once per experiment
        echo "$RUN_ID,$N_TOTAL,$n_corrupt,$ER_FMT,$RUN_SEED,$TIMESTAMP" >> "$MASTER_LOG"

        # 1) prepare corrupted datasets
        bash "$PROCESS_SCRIPT" "$N_TOTAL" "$n_corrupt" "$ER_FMT" "$RUN_ID"

        # 2) run fusion simulations
        bash "$FUSION_SCRIPT" "$CSV_DIR" "$SIM_RESULTS_DIR" "$SIM_JOBS"

        # 3) analyze and log locally (pass extra parameters)
        python3 "$ANALYZE_SCRIPT" --run_id "$RUN_ID" --n_corrupt "$n_corrupt" --error_rate "$ER_FMT"

        echo "✅ Finished RUN $RUN_ID."
        RUN_COUNTER=$((RUN_COUNTER + 1))

        er=$(printf "%.3f" "$(echo "$er + $ER_STEP" | bc -l)")
    done
    n_corrupt=$((n_corrupt + 1))
done

# --------------------------
# Final plotting
# --------------------------
echo ""
echo "Generating plots for all results..."
bash "$PLOT_SCRIPT" "$SIM_RESULTS_DIR" "$PLOTS_DIR" "$PLOT_JOBS"

echo ""
echo "=============================================================="
echo "All experiment runs finished."
echo "Master run log: $MASTER_LOG"
echo "Plots saved under: $PLOTS_DIR"
echo "=============================================================="

