#!/usr/bin/env bash
# One-off eval using the STOCK rtabmap_launch/rtabmap.launch (no custom tuning).
# Loop closure stays at its default (ENABLED), so this measures what RTAB-Map
# does out-of-the-box on TUM-RGBD when you only fix the bag-specific overrides.
#
# Usage: scripts/run_eval_stock.sh <bag> [rate]
#
# Outputs eval_results/stock/<bagname>/{eval.bag, ape_odom.txt, ape_mappath.txt,
# rtabmap.log}. Records /pose, /rtabmap/odom, AND /rtabmap/mapPath so we can
# compare raw VO vs loop-closure-optimized path.

set -euo pipefail

BAG="${1:?usage: run_eval_stock.sh <bag> [rate]}"
RATE="${2:-0.3}"

[[ -f "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 1; }

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME="$(basename "$BAG" .bag)"
OUT_DIR="$REPO_ROOT/eval_results/stock/$NAME"
mkdir -p "$OUT_DIR"
EVAL_BAG="$OUT_DIR/eval.bag"
LOG="$OUT_DIR/rtabmap.log"

source /opt/ros/noetic/setup.bash
source "$REPO_ROOT/devel/setup.bash"

# Clean rtabmap.db
rm -f "$HOME/.ros/rtabmap.db"

echo "=== [$NAME] STOCK launch + loop closure ON, rate=$RATE ===" | tee -a "$LOG"

# 1) start roscore (background)
roscore >>"$LOG" 2>&1 &
ROSCORE_PID=$!
sleep 2

# 2) sim time on
rosparam set /use_sim_time true

# 3) stock rtabmap.launch with TUM-bag overrides only (no VO tuning, no Kp)
roslaunch rtabmap_launch rtabmap.launch \
    rgb_topic:=/camera/rgb/image_color \
    depth_topic:=/camera/depth/image \
    camera_info_topic:=/camera/rgb/camera_info \
    frame_id:=openni_rgb_optical_frame \
    approx_sync_max_interval:=0.04 \
    args:="--delete_db_on_start" \
    rtabmap_viz:=false \
    rviz:=false \
    >>"$LOG" 2>&1 &
LAUNCH_PID=$!

# wait for rtabmap to come up
sleep 5

# 4) recorder (record both odom and mapPath)
rosbag record -O "$EVAL_BAG" /pose /rtabmap/odom /rtabmap/mapPath \
    >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

# 5) play bag, blocking until done
rosbag play --clock -r "$RATE" "$BAG" \
    /tf:=/tf_unused /tf_static:=/tf_static_unused \
    >>"$LOG" 2>&1

# 6) drain mapPath last update (loop closure may still be processing)
sleep 5

# 7) shutdown
kill -INT $REC_PID 2>/dev/null || true
sleep 1
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 2
kill -INT $ROSCORE_PID 2>/dev/null || true
sleep 1
pkill -f rtabmap 2>/dev/null || true
pkill -f rosmaster 2>/dev/null || true

[[ -s "$EVAL_BAG" ]] || { echo "[$NAME] bag empty"; exit 2; }

# 8) evo on both topics
( cd "$OUT_DIR" && evo_ape bag eval.bag /pose /rtabmap/odom -va --t_max_diff 0.05 \
    > ape_odom.txt 2>&1 ) || true
( cd "$OUT_DIR" && evo_ape bag eval.bag /pose /rtabmap/mapPath -va --t_max_diff 0.05 \
    > ape_mappath.txt 2>&1 ) || true

echo "--- $NAME APE on /rtabmap/odom (VO raw) ---"
grep -E "^\s*(rmse|mean|median|max|min|std)" "$OUT_DIR/ape_odom.txt" || true
echo "--- $NAME APE on /rtabmap/mapPath (loop-closure-optimized) ---"
grep -E "^\s*(rmse|mean|median|max|min|std)" "$OUT_DIR/ape_mappath.txt" || true
echo "=== [$NAME] done ==="
