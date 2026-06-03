#!/usr/bin/env bash
# Run RTAB-Map eval on a TUM bag with OdomF2M/BundleAdjustment=1 added
# to the Option-C baseline args. Output goes under eval_results/BA_<bag>.
#
# Usage: scripts/run_eval_BA.sh <bag_path> [rate]

set -eo pipefail

BAG="${1:?usage: scripts/run_eval_BA.sh <bag_path> [rate]}"
RATE="${2:-0.3}"

[[ -f "$BAG" ]] || { echo "Bag not found: $BAG" >&2; exit 1; }

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Strip rgbd_dataset_freiburg2_ prefix and .bag suffix → "pioneer_360" etc.
RAW_NAME="$(basename "$BAG" .bag)"
SHORT_NAME="${RAW_NAME#rgbd_dataset_freiburg2_}"
NAME="BA_${SHORT_NAME}"   # display label only
OUT_DIR="$REPO_ROOT/eval_results/tum_bag/BA_ablation/$SHORT_NAME"
mkdir -p "$OUT_DIR"

EVAL_BAG="$OUT_DIR/eval.bag"
LOG="$OUT_DIR/rtabmap.log"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/devel/setup.bash"

rm -f "$HOME/.ros/rtabmap.db"

# Option-C baseline args + BundleAdjustment
ARGS="--Vis/MinInliers 12 --Vis/MaxFeatures 1500 --GFTT/MinDistance 5 --OdomF2M/MaxSize 3000 --Vis/CorGuessWinSize 80 --OdomF2M/BundleAdjustment 1 --Kp/MaxFeatures -1 --delete_db_on_start"

echo "=== [$NAME] starting eval at rate $RATE ===" | tee -a "$LOG"
echo "args: $ARGS" | tee -a "$LOG"

roslaunch my_rtabmap eval.launch \
  bag:="$BAG" \
  out_bag:="$EVAL_BAG" \
  rate:="$RATE" \
  rtabmap_viz:=false \
  args:="$ARGS" \
  >>"$LOG" 2>&1 || echo "=== [$NAME] roslaunch returned non-zero (often normal at bag end) ==="

[[ -s "$EVAL_BAG" ]] || { echo "[$NAME] eval.bag empty" | tee -a "$LOG"; exit 2; }

cd "$OUT_DIR"
evo_ape bag eval.bag /pose /rtabmap/odom -va --save_results traj.zip \
  > ape.txt 2>&1 || true

echo "--- $NAME APE ---"
sed -n '/APE w.r.t./,$p' ape.txt | head -22
echo "=== [$NAME] done ==="
