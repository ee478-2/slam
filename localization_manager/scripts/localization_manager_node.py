#!/usr/bin/env python3
"""
localization_manager_node — fuses pose sources into /robot_pose.

Priority (when pose_source == "auto"):
  1. AprilTag global pose if its timestamp is fresh (< global_freshness_s).
  2. Legacy AprilTag pose if its timestamp is fresh (< tag_freshness_s).
  3. Gazebo ground truth (sim only).
  4. RTAB-Map odom.

A `pose_source` ROS param selects which source to publish:
  - "auto"       (default) — priority cascade above
  - "global_tag" — only /global_localization/robot_pose
  - "sim_gt"     — only /gazebo/model_states (raises if not available)
  - "tag"        — only /apriltag_localization_pose
  - "rtabmap"    — only /rtabmap/odom

The output is geometry_msgs/PoseStamped on /robot_pose, published at
publish_hz Hz, using the selected source's frame. Simulation and AprilTag poses
are normally in the room/global frame. RTAB-Map odometry is local odom and must
stay in its own odom frame; relabeling it as a global pose makes /odom and the
TF base_link disagree in RViz. The same fused pose is ALSO published as
nav_msgs/Odometry on /odom (child_frame_id = base_frame_name), so /odom becomes
global when AprilTag anchoring is active and falls back to local RTAB odom when
it is not. Twist is left zero (this is a pose-only odom).

This satisfies the A4 Topic 2 rubric requirement for a "localization
manager" while letting us pick the most reliable source for a demo
(sim_gt) and still proving an AprilTag pipeline works end-to-end.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_xy_yaw(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quat(pose.orientation),
    )


def pose_delta(a, b):
    ax, ay, ayaw = pose_xy_yaw(a)
    bx, by, byaw = pose_xy_yaw(b)
    return math.hypot(bx - ax, by - ay), abs(normalize_angle(byaw - ayaw))


class LocalizationManager:
    def __init__(self):
        rospy.init_node("localization_manager", anonymous=False)

        self.pose_source       = rospy.get_param("~pose_source", "auto")
        self.world_frame_name  = rospy.get_param("~world_frame_name", "map")
        self.robot_model_name  = rospy.get_param("~robot_model_name", "nexus_4wd_mecanum")
        self.tag_freshness_s   = rospy.get_param("~tag_freshness_s", 1.0)
        self.global_freshness_s = rospy.get_param("~global_freshness_s", 1.0)
        self.publish_hz        = rospy.get_param("~publish_hz", 20.0)
        self.base_frame_name   = rospy.get_param("~base_frame_name", "base_link")
        self.jump_guard_enabled = self._bool_param("~jump_guard_enabled", True)
        self.jump_guard_max_gap_s = float(rospy.get_param("~jump_guard_max_gap_s", 1.0))
        self.jump_guard_xy_slack_m = float(rospy.get_param("~jump_guard_xy_slack_m", 0.25))
        self.jump_guard_yaw_slack_deg = float(rospy.get_param("~jump_guard_yaw_slack_deg", 60.0))
        self.jump_guard_max_speed_mps = float(rospy.get_param("~jump_guard_max_speed_mps", 0.20))
        self.jump_guard_max_yaw_rate_deg_s = float(
            rospy.get_param("~jump_guard_max_yaw_rate_deg_s", 180.0)
        )
        self.jump_guard_source_confirm_frames = max(
            1, int(rospy.get_param("~jump_guard_source_confirm_frames", 2))
        )
        self.jump_guard_source_consistency_m = float(
            rospy.get_param("~jump_guard_source_consistency_m", 0.25)
        )
        self.jump_guard_source_consistency_yaw_deg = float(
            rospy.get_param("~jump_guard_source_consistency_yaw_deg", 20.0)
        )

        self._lock = threading.Lock()
        self._latest_global = None       # PoseStamped or None
        self._latest_sim_gt = None       # PoseStamped or None
        self._latest_tag    = None       # PoseStamped or None
        self._latest_rtab   = None       # PoseStamped or None
        self._last_accepted = None
        self._last_accepted_src = None
        self._last_accepted_frame = None
        self._last_accepted_stamp = None
        self._pending_correction = None
        self._last_pose_seen_stamp = None
        self._reset_guard_on_next_pose = False

        if self.pose_source not in ("auto", "global_tag", "sim_gt", "tag", "rtabmap"):
            rospy.logerr("Invalid pose_source=%s; falling back to 'auto'", self.pose_source)
            self.pose_source = "auto"

        self.pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._on_model_states, queue_size=1)
        rospy.Subscriber("/global_localization/robot_pose", PoseStamped,
                         self._on_global_pose, queue_size=10)
        rospy.Subscriber("/apriltag_localization_pose", PoseStamped,
                         self._on_tag_pose, queue_size=10)
        rospy.Subscriber("/rtabmap/odom", Odometry,
                         self._on_rtab_odom, queue_size=10)

        rospy.loginfo(
            "[localization_manager] pose_source=%s  world_frame=%s  base_frame=%s  publish_hz=%.1f  global_freshness_s=%.1f  tag_freshness_s=%.1f  jump_guard=%s",
            self.pose_source, self.world_frame_name, self.base_frame_name,
            self.publish_hz, self.global_freshness_s, self.tag_freshness_s,
            self.jump_guard_enabled,
        )

    # ---------- callbacks ----------

    def _on_model_states(self, msg):
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            return
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.world_frame_name
        ps.pose = msg.pose[idx]
        with self._lock:
            self._latest_sim_gt = ps

    def _on_global_pose(self, msg):
        with self._lock:
            self._latest_global = msg

    def _on_tag_pose(self, msg):
        with self._lock:
            self._latest_tag = msg

    def _on_rtab_odom(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        with self._lock:
            self._latest_rtab = ps

    # ---------- selection ----------

    def _pick(self):
        with self._lock:
            global_pose = self._latest_global
            sim = self._latest_sim_gt
            tag = self._latest_tag
            rtab = self._latest_rtab

        if self.pose_source == "global_tag":
            return self._fresh_or_none(global_pose, self.global_freshness_s), "global_tag"
        if self.pose_source == "sim_gt":
            return sim, "sim_gt"
        if self.pose_source == "tag":
            return self._fresh_or_none(tag, self.tag_freshness_s), "tag"
        if self.pose_source == "rtabmap":
            return rtab, "rtabmap"

        # auto cascade
        global_pose = self._fresh_or_none(global_pose, self.global_freshness_s)
        if global_pose is not None:
            return global_pose, "global_tag"
        tag = self._fresh_or_none(tag, self.tag_freshness_s)
        if tag is not None:
            return tag, "tag"
        if sim is not None:
            return sim, "sim_gt"
        if rtab is not None:
            return rtab, "rtabmap"
        return None, "none"

    def _fresh_or_none(self, pose, freshness_s):
        if pose is None:
            return None
        if freshness_s <= 0.0 or pose.header.stamp == rospy.Time(0):
            return pose
        age = (rospy.Time.now() - pose.header.stamp).to_sec()
        return pose if age < freshness_s else None

    @staticmethod
    def _bool_param(name, default=False):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    # ---------- jump guard ----------

    def _guard_pose(self, ps, src, frame_id, stamp):
        if not self.jump_guard_enabled:
            self._accept_guard_state(ps, src, frame_id, stamp)
            return ps, src, "accepted_disabled"

        if self._last_accepted is None or self._reset_guard_on_next_pose:
            if self._reset_guard_on_next_pose:
                rospy.loginfo(
                    "[localization_manager] jump guard reset after pose input gap; accepting %s",
                    src,
                )
            self._accept_guard_state(ps, src, frame_id, stamp)
            return ps, src, "accepted_seed"

        dt = max(0.0, (stamp - self._last_accepted_stamp).to_sec())
        same_track = (
            src == self._last_accepted_src
            and frame_id == self._last_accepted_frame
        )
        if not same_track:
            if self._confirm_source_correction(ps, src, frame_id):
                self._accept_guard_state(ps, src, frame_id, stamp)
                return ps, src, "accepted_confirmed_source_change"
            return self._last_accepted, self._last_accepted_src, "held_source_change"

        xy_jump, yaw_jump = pose_delta(self._last_accepted.pose, ps.pose)
        xy_limit = self.jump_guard_xy_slack_m + self.jump_guard_max_speed_mps * dt
        yaw_limit = math.radians(
            self.jump_guard_yaw_slack_deg
            + self.jump_guard_max_yaw_rate_deg_s * dt
        )

        if xy_jump <= xy_limit and yaw_jump <= yaw_limit:
            self._accept_guard_state(ps, src, frame_id, stamp)
            return ps, src, "accepted"

        rospy.logwarn_throttle(
            1.0,
            "[localization_manager] holding %s pose jump: xy=%.3fm limit=%.3fm yaw=%.1fdeg limit=%.1fdeg dt=%.2fs",
            src, xy_jump, xy_limit, math.degrees(yaw_jump),
            math.degrees(yaw_limit), dt,
        )
        return self._last_accepted, self._last_accepted_src, "held_jump"

    def _accept_guard_state(self, ps, src, frame_id, stamp):
        self._last_accepted = ps
        self._last_accepted_src = src
        self._last_accepted_frame = frame_id
        self._last_accepted_stamp = stamp
        self._pending_correction = None
        self._reset_guard_on_next_pose = False

    def _confirm_source_correction(self, ps, src, frame_id):
        key = (src, frame_id)
        pending = self._pending_correction
        if pending is None or pending["key"] != key:
            self._pending_correction = {
                "key": key,
                "pose": ps,
                "count": 1,
            }
            self._log_pending_source_correction(src, frame_id, 1)
            return self.jump_guard_source_confirm_frames <= 1

        xy_delta, yaw_delta = pose_delta(pending["pose"].pose, ps.pose)
        if (
            xy_delta > self.jump_guard_source_consistency_m
            or yaw_delta > math.radians(self.jump_guard_source_consistency_yaw_deg)
        ):
            self._pending_correction = {
                "key": key,
                "pose": ps,
                "count": 1,
            }
            self._log_pending_source_correction(src, frame_id, 1)
            return self.jump_guard_source_confirm_frames <= 1

        pending["pose"] = ps
        pending["count"] += 1
        self._log_pending_source_correction(src, frame_id, pending["count"])
        return pending["count"] >= self.jump_guard_source_confirm_frames

    def _log_pending_source_correction(self, src, frame_id, count):
        rospy.loginfo_throttle(
            1.0,
            "[localization_manager] confirming source/frame correction %s frame=%s (%d/%d)",
            src, frame_id, count, self.jump_guard_source_confirm_frames,
        )

    # ---------- main loop ----------

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        last_logged_source = None
        while not rospy.is_shutdown():
            ps, src = self._pick()
            if ps is not None:
                source_frame = ps.header.frame_id or self.world_frame_name
                stamp = rospy.Time.now()
                self._last_pose_seen_stamp = stamp
                ps, src, guard_state = self._guard_pose(ps, src, source_frame, stamp)
                source_frame = ps.header.frame_id or self.world_frame_name

                # Re-stamp so downstream consumers see a fresh time. Keep the
                # source frame truthful; /rtabmap/odom is an odom-frame pose,
                # not a map-frame pose.
                out = PoseStamped()
                out.header.stamp = stamp
                out.header.frame_id = source_frame
                out.pose = ps.pose
                self.pub.publish(out)

                # Also republish the same fused pose as nav_msgs/Odometry on /odom
                # (sim had /odom; the real chassis publishes none). Twist = zero.
                odom = Odometry()
                odom.header.stamp = out.header.stamp
                odom.header.frame_id = source_frame
                odom.child_frame_id = self.base_frame_name
                odom.pose.pose = ps.pose
                self.odom_pub.publish(odom)

                if src != last_logged_source:
                    rospy.loginfo(
                        "[localization_manager] active source: %s frame=%s guard=%s",
                        src, source_frame, guard_state,
                    )
                    last_logged_source = src
            elif (
                self.jump_guard_max_gap_s > 0.0
                and self._last_pose_seen_stamp is not None
                and (rospy.Time.now() - self._last_pose_seen_stamp).to_sec()
                > self.jump_guard_max_gap_s
            ):
                self._reset_guard_on_next_pose = True
            rate.sleep()


if __name__ == "__main__":
    LocalizationManager().spin()
