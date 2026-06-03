#!/usr/bin/env bash
# Run RTAB-Map eval on a single TUM bag and produce APE metrics.
#
# Usage: scripts/run_eval_one.sh <bag_path> [rate]
#
# Outputs into eval_results/<bag_basename>/:
#   - eval.bag        : recorded /pose + /rtabmap/odom
#   - rtabmap.log     : roslaunch stdout/stderr
#   - ape.txt         : evo_ape summary
#   - traj.zip        : evo trajectory results (for evo_traj plot later)

set -euo pipefail

BAG="${1:?usage: run_eval_one.sh <bag_path> [rate]}"
RATE="${2:-0.5}"

if [[ ! -f "$BAG" ]]; then
  echo "Bag not found: $BAG" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RAW_NAME="$(basename "$BAG" .bag)"
NAME="${RAW_NAME#rgbd_dataset_freiburg2_}"   # pioneer_360 etc.
OUT_DIR="$REPO_ROOT/eval_results/tum_bag/option_C/$NAME"
mkdir -p "$OUT_DIR"

EVAL_BAG="$OUT_DIR/eval.bag"
LOG="$OUT_DIR/rtabmap.log"

echo "=== [$NAME] starting eval at rate $RATE ===" | tee -a "$LOG"

# Source ROS + workspace
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/devel/setup.bash"

# Clean stale rtabmap.db so VO/SLAM starts fresh
rm -f "$HOME/.ros/rtabmap.db"

# eval.launch waits for the bag to finish (bag_player required=true),
# then shuts down everything. headless: rtabmap_viz=false.
roslaunch my_rtabmap eval.launch \
  bag:="$BAG" \
  out_bag:="$EVAL_BAG" \
  rate:="$RATE" \
  rtabmap_viz:=false \
  >>"$LOG" 2>&1 || {
    echo "=== [$NAME] roslaunch returned non-zero (often normal at bag end) ==="
}

if [[ ! -s "$EVAL_BAG" ]]; then
  echo "[$NAME] eval.bag is empty/missing — abort" | tee -a "$LOG"
  exit 2
fi

echo "=== [$NAME] running evo_ape ===" | tee -a "$LOG"
cd "$OUT_DIR"
evo_ape bag eval.bag /pose /rtabmap/odom -va --save_results traj.zip \
  > ape.txt 2>&1 || true

echo "--- $NAME APE ---"
sed -n '/APE w.r.t./,$p' ape.txt | head -25
echo "=== [$NAME] done ==="
