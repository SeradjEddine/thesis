#!/usr/bin/env bash
# Master automation script for full simulation experiments
# - First: control run (0 corrupted, 0.0 error)
# - Then: for n_corrupt = 1..MAX_CORRUPT, error_rate = 0.005..0.05 (step 0.005)
# Runs: data corruption -> fusion simulations -> analysis
# Usage: ./run_full_experiment.sh [N_TOTAL] [MAX_CORRUPT] [SEED_BASE] [SIM_JOBS] [PLOT_JOBS]

set -e
set -o pipefail

# ----------------------
# Defaults (override via CLI)
# ----------------------
N_TOTAL=${1:-5}        # total CSV variations per set
MAX_CORRUPT=${2:-2}    # max number of corrupted files to test (will test 1..MAX_CORRUPT)
SEED_BASE=${3:-42}     # base seed
SIM_JOBS=${4:-10}       # passed to run_fusion_sims.sh (max parallel jobs)
PLOT_JOBS=${5:-10}      # passed to run_all_plots.sh (max parallel jobs)

# Scripts (assumed in the same scripts/ folder)
PROCESS_SCRIPT="./process_all_sets.sh"
FUSION_SCRIPT="./run_fusion_sims.sh"
ANALYZE_SCRIPT="./analyze_sim_results.py"
PLOT_SCRIPT="./run_all_plots.sh"

# Data folders
DATA_DIR="../Data"
CSV_DIR="$DATA_DIR/sets_csv"
SIM_RESULTS_DIR="$DATA_DIR/sim_results_csv"
PLOTS_DIR="$DATA_DIR/sim_results_plot"

# Logging
MASTER_LOG="$DATA_DIR/experiment_log_master.csv"
mkdir -p "$DATA_DIR"
echo "run_id,n_total,n_corrupt,error_rate,seed,timestamp" > "$MASTER_LOG"

echo "=============================================================="
echo " Full experiment sweep"
echo " Total variations: $N_TOTAL"
echo " Max corrupted files to test: $MAX_CORRUPT"
echo " Seed base: $SEED_BASE"
echo " Sim jobs: $SIM_JOBS, Plot jobs: $PLOT_JOBS"
echo "=============================================================="

# small helper to format error_rate nicely
format_er() {
    printf "%.3f" "$1"
}

RUN_COUNTER=0

# --------------------------
# 0) CONTROL run: 0 corrupted, 0.0 error
# --------------------------
echo ""
echo "--------------------------------------------------------------"
echo "CONTROL RUN: n_corrupt=0, error_rate=0.0"
echo "--------------------------------------------------------------"

RUN_ID="control_corr0_err000_$(date +%Y%m%d_%H%M%S)"
RUN_SEED=$((SEED_BASE + RUN_COUNTER))
echo "$RUN_ID,$N_TOTAL,0,0.0,$RUN_SEED,$(date +%Y-%m-%dT%H:%M:%S)" >> "$MASTER_LOG"

# Step 1: generate CSVs
bash "$PROCESS_SCRIPT" "$N_TOTAL" 0 0.0

# Step 2: run fusion sims (pass CSV_DIR and SIM_RESULTS_DIR and SIM_JOBS)
bash "$FUSION_SCRIPT" "$CSV_DIR" "$SIM_RESULTS_DIR" "$SIM_JOBS"

# Step 3: analyze
python3 "$ANALYZE_SCRIPT"

RUN_COUNTER=$((RUN_COUNTER + 1))
echo "Completed CONTROL run."

# --------------------------
# 1..) Sweeps: n_corrupt = 1 .. MAX_CORRUPT, error_rate = 0.005 .. 0.05 step 0.005
# --------------------------
ER_START=0.005
ER_STEP=0.01
ER_END=0.05

n_corrupt=1
while [ "$n_corrupt" -le "$MAX_CORRUPT" ]; do
    er=$(printf "%.3f" "$ER_START")
    while (( $(echo "$er <= $ER_END" | bc -l) )); do
        ER_FMT=$(format_er "$er")
        echo ""
        echo "--------------------------------------------------------------"
        echo "Run: n_corrupt=$n_corrupt, error_rate=$ER_FMT"
        echo "--------------------------------------------------------------"

        RUN_ID="corr${n_corrupt}_err$(echo $ER_FMT | sed 's/\.//')_$(date +%Y%m%d_%H%M%S)"
        RUN_SEED=$((SEED_BASE + RUN_COUNTER))
        echo "$RUN_ID,$N_TOTAL,$n_corrupt,$ER_FMT,$RUN_SEED,$(date +%Y-%m-%dT%H:%M:%S)" >> "$MASTER_LOG"

        # 1) generate CSVs
        bash "$PROCESS_SCRIPT" "$N_TOTAL" "$n_corrupt" "$ER_FMT"

        # 2) run fusion sims
        bash "$FUSION_SCRIPT" "$CSV_DIR" "$SIM_RESULTS_DIR" "$SIM_JOBS"

        # 3) analyze
        python3 "$ANALYZE_SCRIPT"

        RUN_COUNTER=$((RUN_COUNTER + 1))

        # increment error rate
        er=$(printf "%.3f" "$(echo "$er + $ER_STEP" | bc -l)")
    done
    n_corrupt=$((n_corrupt + 1))
done

# (Optional) generate plots for all results (you may already run analyze to produce summary CSV)
# If you want plotting per-run too:
echo ""
echo "Generating plots for all simulation results..."
bash "$PLOT_SCRIPT" "$SIM_RESULTS_DIR" "$PLOTS_DIR" "$PLOT_JOBS"

echo ""
echo "=============================================================="
echo "All experiment runs finished."
echo "Master run log: $MASTER_LOG"
echo "Analysis summary (overwritten by analyze script): $DATA_DIR/experiment_analysis.csv"
echo "Plots saved under: $PLOTS_DIR"
echo "=============================================================="

