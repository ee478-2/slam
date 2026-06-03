#!/usr/bin/env python3
"""
Read eval.bag + phases.txt from a rotation-drift run; report pose deltas
per phase boundary so we can isolate "drift caused by in-place rotation"
from "drift caused by translation."

For each topic in {/odom, /rtabmap/odom, /ground_truth/odom}, find the
pose closest in time to:
  - end of phase 1 (forward): t1_end
  - end of phase 2 (stop):    t2_end  (= start of rotation, robot at rest)
  - end of phase 3 (rotate):  t3_end  (= rotation finished)
  - end of phase 4 (stop):    t4_end  (settled after rotation)

Drift induced by rotation alone = pose_change(t3_end) - pose_change(t2_end)
for /rtabmap/odom, minus same for /ground_truth/odom (which should be ~0).
"""
import sys
import math
import rosbag
from collections import defaultdict


def yaw_from_quat(q):
    # q = (x, y, z, w)
    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_phase_drift.py <run_dir>")
        sys.exit(1)
    run_dir = sys.argv[1]
    bag_path = f"{run_dir}/eval.bag"
    phases_path = f"{run_dir}/phases.txt"

    # Parse phases.txt
    phase_starts = {}
    with open(phases_path) as f:
        for line in f:
            parts = dict(p.split('=', 1) for p in line.strip().split() if '=' in p)
            phase = int(parts['phase'])
            phase_starts[phase] = float(parts['t_start'])
    # Phase end ≈ next phase start. For last phase, use phase_start + 2s.
    phase_ends = {}
    sorted_ph = sorted(phase_starts)
    for i, p in enumerate(sorted_ph):
        if i + 1 < len(sorted_ph):
            phase_ends[p] = phase_starts[sorted_ph[i + 1]]
        else:
            phase_ends[p] = phase_starts[p] + 2.0

    print(f"Phases parsed:")
    for p in sorted_ph:
        print(f"  phase {p}: t={phase_starts[p]:.2f} -> {phase_ends[p]:.2f}  "
              f"(dur={phase_ends[p]-phase_starts[p]:.2f}s)")

    # Read poses by topic
    topics = ['/odom', '/rtabmap/odom', '/ground_truth/odom']
    poses = defaultdict(list)  # topic -> list of (t_sec, x, y, yaw)
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            yaw = yaw_from_quat([o.x, o.y, o.z, o.w])
            poses[topic].append((ts, p.x, p.y, yaw))

    # For each phase boundary timestamp, find closest pose per topic
    boundary_names = {
        ('end', 1): "phase1_end_forward",
        ('end', 2): "phase2_end_pre_rotation",
        ('end', 3): "phase3_end_post_rotation",
        ('end', 4): "phase4_end_settled",
    }

    def closest_pose(samples, t):
        if not samples:
            return None
        best = min(samples, key=lambda s: abs(s[0] - t))
        return best

    print()
    print(f"{'boundary':30s} {'topic':25s}  {'x':>8s}  {'y':>8s}  {'yaw_deg':>9s}")
    snapshots = {}
    for (kind, phase), name in boundary_names.items():
        t_target = phase_ends[phase]
        for topic in topics:
            sample = closest_pose(poses[topic], t_target)
            if sample is None:
                continue
            ts, x, y, yaw = sample
            snapshots[(name, topic)] = (x, y, yaw)
            print(f"{name:30s} {topic:25s}  {x:8.4f}  {y:8.4f}  {math.degrees(yaw):9.2f}")
        print()

    # Compute translation change per topic for each phase, plus drift relative to GT
    print()
    print("=== translation change per phase (Δx, Δy, Δyaw) ===")
    print(f"{'phase':10s} {'topic':25s}  {'dx':>8s}  {'dy':>8s}  {'d_yaw_deg':>10s}  {'|d_xy|':>8s}")
    for phase in sorted(boundary_names.keys()):
        kind, p = phase
        if p == 1:
            continue  # skip phase 1 itself, just need transitions
        prev_name = boundary_names[('end', p - 1)]
        cur_name = boundary_names[('end', p)]
        for topic in topics:
            if (prev_name, topic) not in snapshots or (cur_name, topic) not in snapshots:
                continue
            x0, y0, yaw0 = snapshots[(prev_name, topic)]
            x1, y1, yaw1 = snapshots[(cur_name, topic)]
            dx, dy, dyaw = x1 - x0, y1 - y0, yaw1 - yaw0
            # Wrap dyaw to [-pi, pi]
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            mag = math.hypot(dx, dy)
            label = f"phase{p}"
            print(f"{label:10s} {topic:25s}  {dx:8.4f}  {dy:8.4f}  {math.degrees(dyaw):10.2f}  {mag:8.4f}")
        print()

    # Drift induced by rotation (phase 3) = |Δxy of rtabmap| - |Δxy of GT|
    # GT should be ~0 during pure rotation; rtabmap may have spurious motion.
    p3_prev = "phase2_end_pre_rotation"
    p3_post = "phase3_end_post_rotation"
    if all((n, t) in snapshots for n in (p3_prev, p3_post)
           for t in ('/rtabmap/odom', '/ground_truth/odom')):
        x0r, y0r, _ = snapshots[(p3_prev, '/rtabmap/odom')]
        x1r, y1r, _ = snapshots[(p3_post, '/rtabmap/odom')]
        dr = math.hypot(x1r - x0r, y1r - y0r)
        x0g, y0g, _ = snapshots[(p3_prev, '/ground_truth/odom')]
        x1g, y1g, _ = snapshots[(p3_post, '/ground_truth/odom')]
        dg = math.hypot(x1g - x0g, y1g - y0g)
        print()
        print(f"=== ROTATION-INDUCED FALSE TRANSLATION ===")
        print(f"  /rtabmap/odom        |Δxy| during rotation: {dr:.4f} m")
        print(f"  /ground_truth/odom   |Δxy| during rotation: {dg:.4f} m  (should be ~0)")
        print(f"  attributed to rotation:                     {abs(dr - dg):.4f} m")


if __name__ == '__main__':
    main()
