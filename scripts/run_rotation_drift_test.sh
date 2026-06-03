#!/usr/bin/env bash
# Drive straight then force in-place rotation(s), record /odom,
# /rtabmap/odom, /ground_truth/odom, and measure how much translation
# drift the rotation added on top of the pre-rotation baseline.
#
# Usage: scripts/run_rotation_drift_test.sh [run_name] [rotations]
#   run_name  : output subdir under eval_results/sim/
#   rotations : approximate number of full turns (default 1.04)
#
# Sequence:
#   phase 1: forward 0.3 m/s × 4s   = 1.2 m straight
#   phase 2: stop      0.0      × 2s
#   phase 3: in-place yaw 0.5 rad/s × (rotations × 12.566s)
#   phase 4: stop      0.0      × 2s
#
# Output: eval_results/sim/<run_name>/

set -eo pipefail

NAME="${1:-rot_$(date +%Y%m%d_%H%M%S)}"
ROTATIONS="${2:-1.04}"

# Phase 3 duration: rotations × 2π / 0.5 rad/s + 1s buffer
PHASE3_DUR=$(python3 -c "print(int(round($ROTATIONS * 6.2832 / 0.5)) + 1)")
echo "Will rotate $ROTATIONS turns over ${PHASE3_DUR}s"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$REPO_ROOT/eval_results/sim/rotation_drift/$NAME"
mkdir -p "$OUT_DIR"

EVAL_BAG="$OUT_DIR/eval.bag"
LOG="$OUT_DIR/rtabmap.log"
PHASES_LOG="$OUT_DIR/phases.txt"

echo "=== [$NAME] starting ===" | tee -a "$LOG"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/devel/setup.bash"
export DISPLAY="${DISPLAY:-:0}"

rm -f "$HOME/.ros/rtabmap.db"

# Launch sim+RTAB stack. We will kill dwa_planner_node before driving so
# our manual cmd_vel publisher owns /cmd_vel exclusively.
roslaunch dwa_planner dwa_rtabmap.launch rviz:=false gui:=false \
  >>"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo "=== [$NAME] cleanup ==="
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Start recording as early as possible — capture each topic's origin as it
# comes online (RTAB warmup, gt_odom_publisher first model_states msg, etc.).
# Wait briefly for rosmaster to be reachable, then record.
echo "=== [$NAME] waiting for rosmaster (max 10s) ==="
for i in $(seq 1 20); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 0.5
done

echo "=== [$NAME] start recording (early — captures spawn/init) ==="
rosbag record -O "$EVAL_BAG" \
  /odom /rtabmap/odom /ground_truth/odom /rtabmap/mapPath /tf /tf_static /clock \
  >>"$LOG" 2>&1 &
REC_PID=$!

echo "=== [$NAME] settling 12s ==="
sleep 12

# Make sure the topics are alive before we kill the planner.
if ! rostopic list 2>/dev/null | grep -q '^/rtabmap/odom$'; then
  echo "[$NAME] /rtabmap/odom missing — abort" | tee -a "$LOG"
  exit 2
fi

# Kill dwa_planner_node so it stops publishing /cmd_vel.
echo "=== [$NAME] killing dwa_planner_node ==="
rosnode kill /dwa_planner_node >>"$LOG" 2>&1 || true
sleep 1

# Stop any lingering velocity for a clean baseline.
rostopic pub --once /cmd_vel geometry_msgs/Twist '{}' >>"$LOG" 2>&1 || true
sleep 1

# Lock wall-clock <-> sim-time offset. With use_sim_time, rosbag stamps in
# sim time but `date +%s.%N` is wall. Without this offset every phase
# boundary in phases.txt would be unusable for analysis.
SIM_NOW=$(timeout 3 rostopic echo --noarr -n 1 /clock 2>/dev/null | \
          python3 -c "
import sys, re
c = sys.stdin.read()
m = re.search(r'secs:\s*(\d+)\s*\n\s*nsecs:\s*(\d+)', c)
print((int(m.group(1)) + int(m.group(2))/1e9) if m else 0.0)
")
WALL_NOW=$(date +%s.%N)
OFFSET=$(python3 -c "print($SIM_NOW - $WALL_NOW)")
echo "[$NAME] wall->sim offset = $OFFSET" | tee -a "$LOG"

stamp_sim() {
  python3 -c "print($(date +%s.%N) + $OFFSET)"
}

# Phase 1: forward
echo "phase=1 t_start=$(stamp_sim) action=forward(0.3,0,0)" | tee -a "$PHASES_LOG"
timeout 4 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  >>"$LOG" 2>&1 || true

# Phase 2: stop (settles, captures pre-rotation pose)
echo "phase=2 t_start=$(stamp_sim) action=stop" | tee -a "$PHASES_LOG"
timeout 2 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{}' >>"$LOG" 2>&1 || true

# Phase 3: in-place yaw rotation (variable duration based on requested turns)
echo "phase=3 t_start=$(stamp_sim) action=rotate(0,0,0.5) rotations=$ROTATIONS" | tee -a "$PHASES_LOG"
timeout "$PHASE3_DUR" rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' \
  >>"$LOG" 2>&1 || true

# Phase 4: stop (captures post-rotation pose)
echo "phase=4 t_start=$(stamp_sim) action=stop" | tee -a "$PHASES_LOG"
timeout 2 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{}' >>"$LOG" 2>&1 || true

echo "=== [$NAME] stop recording ==="
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

cleanup
trap - EXIT

if [[ ! -s "$EVAL_BAG" ]]; then
  echo "[$NAME] eval.bag empty — abort" | tee -a "$LOG"
  exit 3
fi

echo "=== [$NAME] evo_ape: /rtabmap/odom vs /ground_truth/odom ==="
cd "$OUT_DIR"
evo_ape bag eval.bag /ground_truth/odom /rtabmap/odom -va \
  --save_results traj_rtab_vs_gt.zip > ape_rtab_vs_gt.txt 2>&1 || true
sed -n '/APE w.r.t./,$p' ape_rtab_vs_gt.txt | head -20

echo "=== [$NAME] evo_ape: /odom vs /ground_truth/odom (sanity) ==="
evo_ape bag eval.bag /ground_truth/odom /odom -va \
  --save_results traj_odom_vs_gt.zip > ape_odom_vs_gt.txt 2>&1 || true
sed -n '/APE w.r.t./,$p' ape_odom_vs_gt.txt | head -20

echo "=== [$NAME] trajectory plot ==="
evo_traj bag eval.bag /odom /rtabmap/odom --ref /ground_truth/odom \
  --plot_mode xy --save_plot traj_xy.png >>"$LOG" 2>&1 || true

echo "=== [$NAME] phase log ==="
cat "$PHASES_LOG"

ls -la "$OUT_DIR"
echo "=== [$NAME] done ==="
