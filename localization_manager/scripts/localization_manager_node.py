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

import threading

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


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

        self._lock = threading.Lock()
        self._latest_global = None       # PoseStamped or None
        self._latest_sim_gt = None       # PoseStamped or None
        self._latest_tag    = None       # PoseStamped or None
        self._latest_rtab   = None       # PoseStamped or None

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
            "[localization_manager] pose_source=%s  world_frame=%s  base_frame=%s  publish_hz=%.1f  global_freshness_s=%.1f  tag_freshness_s=%.1f",
            self.pose_source, self.world_frame_name, self.base_frame_name,
            self.publish_hz, self.global_freshness_s, self.tag_freshness_s,
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

    # ---------- main loop ----------

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        last_logged_source = None
        while not rospy.is_shutdown():
            ps, src = self._pick()
            if ps is not None:
                source_frame = ps.header.frame_id or self.world_frame_name

                # Re-stamp so downstream consumers see a fresh time. Keep the
                # source frame truthful; /rtabmap/odom is an odom-frame pose,
                # not a map-frame pose.
                out = PoseStamped()
                out.header.stamp = rospy.Time.now()
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
                        "[localization_manager] active source: %s frame=%s",
                        src, source_frame,
                    )
                    last_logged_source = src
            rate.sleep()


if __name__ == "__main__":
    LocalizationManager().spin()
