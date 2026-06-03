#!/usr/bin/env bash
# Assignment 2-2 parameter sweep: Odom/ResetCountdown.
#
# Sweeps Odom/ResetCountdown over [0, 1, 5, 10, 30] on pioneer_slam2
# (the bag with the largest baseline error, so the strongest signal).
# All other VO params held fixed at the current tuned values.
#
# Outputs:
#   eval_results/sweep_resetcountdown/<value>/{eval.bag, ape.txt, traj.zip, rtabmap.log}
#   eval_results/sweep_resetcountdown/summary.txt   (table of APE per value)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG="${BAG:-$REPO_ROOT/src/data/rgbd_dataset_freiburg2_pioneer_slam2.bag}"
RATE="${RATE:-1.0}"
VALUES=(${VALUES:-0 1 5 10 30})

# OUT_ROOT can be overridden so v2 sweep (rate=0.3 + extended bag set) doesn't
# overwrite the original rate=1.0 slam2 sweep results.
OUT_ROOT="${OUT_ROOT:-$REPO_ROOT/eval_results/sweep_resetcountdown}"
mkdir -p "$OUT_ROOT"

if [[ ! -f "$BAG" ]]; then
  echo "Bag not found: $BAG" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/devel/setup.bash"

for V in "${VALUES[@]}"; do
  OUT_DIR="$OUT_ROOT/$V"
  mkdir -p "$OUT_DIR"
  EVAL_BAG="$OUT_DIR/eval.bag"
  LOG="$OUT_DIR/rtabmap.log"

  ARGS="--Vis/MinInliers 12 --Vis/MaxFeatures 1500 --GFTT/MinDistance 5 --OdomF2M/MaxSize 3000 --Vis/CorGuessWinSize 80 --Odom/ResetCountdown ${V} --Kp/MaxFeatures -1 --delete_db_on_start"

  echo "=== [Odom/ResetCountdown=$V] starting (rate=$RATE) ===" | tee -a "$LOG"

  rm -f "$HOME/.ros/rtabmap.db"

  roslaunch my_rtabmap eval.launch \
    bag:="$BAG" \
    out_bag:="$EVAL_BAG" \
    rate:="$RATE" \
    rtabmap_viz:=false \
    args:="$ARGS" \
    >>"$LOG" 2>&1 || echo "=== [V=$V] roslaunch returned non-zero (often normal at bag end) ==="

  if [[ ! -s "$EVAL_BAG" ]]; then
    echo "[V=$V] eval.bag empty — skipping evo" | tee -a "$LOG"
    continue
  fi

  ( cd "$OUT_DIR" && \
    evo_ape bag eval.bag /pose /rtabmap/odom -va --save_results traj.zip \
      > ape.txt 2>&1 ) || true

  echo "--- V=$V APE ---"
  grep -E "^\s*(rmse|mean|median|max|min|std)" "$OUT_DIR/ape.txt" || true
done

# Summary table
SUMMARY="$OUT_ROOT/summary.txt"
{
  echo "Odom/ResetCountdown sweep on $(basename "$BAG") @ rate=$RATE"
  echo "-------------------------------------------------------"
  printf "%-8s %-10s %-10s %-10s %-10s\n" "value" "max" "mean" "median" "rmse"
  for V in "${VALUES[@]}"; do
    APE="$OUT_ROOT/$V/ape.txt"
    [[ -f "$APE" ]] || { printf "%-8s %s\n" "$V" "(missing)"; continue; }
    max=$(awk '$1=="max"     {print $2}' "$APE")
    mean=$(awk '$1=="mean"   {print $2}' "$APE")
    med=$(awk '$1=="median"  {print $2}' "$APE")
    rmse=$(awk '$1=="rmse"   {print $2}' "$APE")
    printf "%-8s %-10s %-10s %-10s %-10s\n" "$V" "$max" "$mean" "$med" "$rmse"
  done
} | tee "$SUMMARY"
