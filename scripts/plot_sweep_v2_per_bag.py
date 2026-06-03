#!/usr/bin/env python3
"""
Per-bag XY trajectory overlay for HW3-2.2 sweep_v2.

For each TUM bag, loads the recorded /pose (GT) and /rtabmap/odom (estimate)
from each V's eval.bag, time-syncs them via evo.core.sync.associate_trajectories,
and aligns the estimate onto GT with scaled Umeyama (sim(3)) using
evo.core.trajectory.PoseTrajectory3D.align(correct_scale=True).
Then overlays GT + 5 V curves on one figure per bag.

Outputs: eval_results/tum_bag/sweep_v2/<label>_xy_overlay.png
"""
import copy
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag1 import Reader as Rosbag1Reader

from evo.core import sync
from evo.tools import file_interface

REPO = Path(__file__).resolve().parents[1]
SWEEP_DIR = REPO / "eval_results/tum_bag/sweep_v2"

VALUES = [0, 1, 5, 10, 30]
V_COLORS = {0: "#888888", 1: "#d33333", 5: "#1f7a1f",
            10: "#1f70a8", 30: "#a020a0"}

BAGS = [
    ("pioneer_360",   "rgbd_dataset_freiburg2_pioneer_360"),
    ("pioneer_slam",  "rgbd_dataset_freiburg2_pioneer_slam"),
    ("pioneer_slam2", "rgbd_dataset_freiburg2_pioneer_slam2"),
    ("pioneer_slam3", "rgbd_dataset_freiburg2_pioneer_slam3"),
]

GT_TOPIC = "/pose"
EST_TOPIC = "/rtabmap/odom"
MAX_DIFF = 0.05  # seconds


def load_pair(bag_path):
    """Return (traj_ref, traj_est) PoseTrajectory3D objects from a bag."""
    if not bag_path.exists():
        return None, None
    with Rosbag1Reader(bag_path) as reader:
        topics = {c.topic for c in reader.connections}
        if GT_TOPIC not in topics or EST_TOPIC not in topics:
            return None, None
        traj_ref = file_interface.read_bag_trajectory(reader, GT_TOPIC)
        traj_est = file_interface.read_bag_trajectory(reader, EST_TOPIC)
    return traj_ref, traj_est


for label, dirname in BAGS:
    fig, ax = plt.subplots(figsize=(7, 7))
    gt_plotted = False

    for V in VALUES:
        bag = SWEEP_DIR / dirname / str(V) / "eval.bag"
        traj_ref, traj_est = load_pair(bag)
        if traj_ref is None or traj_est is None:
            print(f"  [{label} V={V}] skip - missing topics or bag")
            continue

        # Plot GT once (it's identical /pose across all V runs of the same bag).
        if not gt_plotted:
            gt_xyz = traj_ref.positions_xyz
            ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "k-", linewidth=2.5,
                    label="GT /pose", alpha=0.95, zorder=10)
            gt_plotted = True

        # Sync by timestamp, then sim(3)-align the estimate onto GT.
        ref_sync, est_sync = sync.associate_trajectories(
            traj_ref, traj_est, max_diff=MAX_DIFF)
        if ref_sync.num_poses < 3:
            print(f"  [{label} V={V}] skip - only {ref_sync.num_poses} matches")
            continue

        est_aligned = copy.deepcopy(est_sync)
        R, t, s = est_aligned.align(ref_sync, correct_scale=True)
        xy = est_aligned.positions_xyz[:, :2]

        ax.plot(xy[:, 0], xy[:, 1], "-",
                color=V_COLORS[V], linewidth=1.4,
                label=f"V={V} (s={s:.3f})", alpha=0.85)
        print(f"  [{label} V={V}] matched={ref_sync.num_poses}  scale={s:.4f}")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{label} - RTAB odometry (5 V values) sim(3)-aligned to GT")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    out_path = SWEEP_DIR / f"{label}_xy_overlay.png"
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"saved: {out_path}")
