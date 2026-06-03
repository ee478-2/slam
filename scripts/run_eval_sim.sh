#!/usr/bin/env bash
# HW3-3 simulation evaluation: record /odom (sim GT-like) and /rtabmap/odom
# while the robot drives the graph_planner waypoints, then run evo_ape +
# evo_traj. Mirrors the HW3-1/2 eval flow (run_eval_one.sh) for the sim case.
#
# Usage: scripts/run_eval_sim.sh [duration_sec] [run_name]
#   duration_sec: recording window in seconds (default 90)
#   run_name:     subdir under eval_results/sim/ (default: timestamp)
#
# Outputs eval_results/sim/<run_name>/:
#   - eval.bag        : /odom + /rtabmap/odom + tf
#   - rtabmap.log     : roslaunch stdout/stderr
#   - ape.txt         : evo_ape summary
#   - traj.zip        : evo trajectory metrics
#   - traj_xy.png     : evo_traj XY trajectory plot
#
# Assumes nothing else is running on this ROS_MASTER. Will launch
# dwa_rtabmap.launch in background and shut it down on completion.

set -eo pipefail
# nounset is off because catkin/gazebo env-hooks expand unset vars.

DURATION="${1:-90}"
NAME="${2:-$(date +%Y%m%d_%H%M%S)}"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$REPO_ROOT/eval_results/sim/corridor/$NAME"
mkdir -p "$OUT_DIR"

EVAL_BAG="$OUT_DIR/eval.bag"
LOG="$OUT_DIR/rtabmap.log"

echo "=== [$NAME] eval window=${DURATION}s, outdir=$OUT_DIR ===" | tee -a "$LOG"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/devel/setup.bash"
export DISPLAY="${DISPLAY:-:0}"

# Fresh RTAB DB; rtabmap node also passes --delete_db_on_start anyway.
rm -f "$HOME/.ros/rtabmap.db"

# Launch sim+nav+RTAB stack in background; rviz disabled (headless eval).
roslaunch dwa_planner dwa_rtabmap.launch rviz:=false gui:=false \
  >>"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo "=== [$NAME] cleanup: SIGINT to roslaunch (pid $LAUNCH_PID) ==="
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  # give roscore time to finish, then ensure dead
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Wait for sim to reach steady state — robot spawn settles, RTAB-Map
# initializes, graph_planner starts publishing path. ~10s is enough on this
# laptop; bump if eval shows zero-length trajectory.
echo "=== [$NAME] settling for 12s ==="
sleep 12

if ! rostopic list 2>/dev/null | grep -q '^/rtabmap/odom$'; then
  echo "[$NAME] /rtabmap/odom not present after settle — aborting" | tee -a "$LOG"
  exit 2
fi

echo "=== [$NAME] recording ${DURATION}s ==="
rosbag record --duration="${DURATION}" -O "$EVAL_BAG" \
  /odom /rtabmap/odom /ground_truth/odom /rtabmap/mapPath /tf /tf_static \
  >>"$LOG" 2>&1

if [[ ! -s "$EVAL_BAG" ]]; then
  echo "[$NAME] eval.bag empty — aborting" | tee -a "$LOG"
  exit 3
fi

# Sim no longer needed; kill before evo so nothing competes for CPU.
cleanup
trap - EXIT

cd "$OUT_DIR"

echo "=== [$NAME] evo_ape: /odom vs /ground_truth/odom (dead-reckoning err) ==="
evo_ape bag eval.bag /ground_truth/odom /odom -va \
  --save_results traj_odom_vs_gt.zip > ape_odom_vs_gt.txt 2>&1 || true
sed -n '/APE w.r.t./,$p' ape_odom_vs_gt.txt | head -20

echo "=== [$NAME] evo_ape: /rtabmap/odom vs /ground_truth/odom (RTAB err) ==="
evo_ape bag eval.bag /ground_truth/odom /rtabmap/odom -va \
  --save_results traj_rtab_vs_gt.zip > ape_rtab_vs_gt.txt 2>&1 || true
sed -n '/APE w.r.t./,$p' ape_rtab_vs_gt.txt | head -20

echo "=== [$NAME] evo_ape: /rtabmap/odom vs /odom (HW2-style comparison) ==="
evo_ape bag eval.bag /odom /rtabmap/odom -va \
  --save_results traj.zip > ape.txt 2>&1 || true

echo
echo "=== [$NAME] evo_traj 3-way XY plot → traj_xy*.png ==="
evo_traj bag eval.bag /odom /rtabmap/odom --ref /ground_truth/odom \
  --plot_mode xy --save_plot traj_xy.png >>"$LOG" 2>&1 || true

ls -la "$OUT_DIR"
echo "=== [$NAME] done ==="
