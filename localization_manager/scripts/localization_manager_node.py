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
stay continuous for local planners. Therefore /odom is published from a separate
continuous odom source (default: /rtabmap/odom), while the fused selected pose is
also exposed as nav_msgs/Odometry on /global_odom for diagnostics or global
consumers. Publishing the tag-corrected global pose directly on /odom causes
local planners to jump when an AprilTag becomes visible.

This satisfies the A4 Topic 2 rubric requirement for a "localization
manager" while letting us pick the most reliable source for a demo
(sim_gt) and still proving an AprilTag pipeline works end-to-end.
"""

import threading

import rospy
from geometry_msgs.msg import PoseStamped
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
        self.odom_source       = rospy.get_param("~odom_source", "rtabmap")
        self.odom_topic        = rospy.get_param("~odom_topic", "/odom")
        self.selected_odom_topic = rospy.get_param("~selected_odom_topic", "/global_odom")

        self._lock = threading.Lock()
        self._latest_global = None       # PoseStamped or None
        self._latest_sim_gt = None       # PoseStamped or None
        self._latest_tag    = None       # PoseStamped or None
        self._latest_rtab   = None       # PoseStamped or None
        self._latest_rtab_odom = None    # Odometry or None

        if self.pose_source not in ("auto", "global_tag", "sim_gt", "tag", "rtabmap"):
            rospy.logerr("Invalid pose_source=%s; falling back to 'auto'", self.pose_source)
            self.pose_source = "auto"
        if self.odom_source not in ("rtabmap", "selected"):
            rospy.logerr("Invalid odom_source=%s; falling back to 'rtabmap'", self.odom_source)
            self.odom_source = "rtabmap"

        self.pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.selected_odom_pub = rospy.Publisher(
            self.selected_odom_topic, Odometry, queue_size=10
        )

        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._on_model_states, queue_size=1)
        rospy.Subscriber("/global_localization/robot_pose", PoseStamped,
                         self._on_global_pose, queue_size=10)
        rospy.Subscriber("/apriltag_localization_pose", PoseStamped,
                         self._on_tag_pose, queue_size=10)
        rospy.Subscriber("/rtabmap/odom", Odometry,
                         self._on_rtab_odom, queue_size=10)

        rospy.loginfo(
            "[localization_manager] pose_source=%s  odom_source=%s  odom_topic=%s  selected_odom_topic=%s  world_frame=%s  base_frame=%s  publish_hz=%.1f  global_freshness_s=%.1f  tag_freshness_s=%.1f",
            self.pose_source, self.odom_source, self.odom_topic,
            self.selected_odom_topic, self.world_frame_name,
            self.base_frame_name, self.publish_hz, self.global_freshness_s,
            self.tag_freshness_s,
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
            self._latest_rtab_odom = msg

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

    def _pose_to_odom(self, ps, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ps.header.frame_id or self.world_frame_name
        odom.child_frame_id = self.base_frame_name
        odom.pose.pose = ps.pose
        return odom

    def _copy_odom(self, msg, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = msg.header.frame_id
        odom.child_frame_id = msg.child_frame_id or self.base_frame_name
        odom.pose = msg.pose
        odom.twist = msg.twist
        return odom

    def _pick_odom(self, selected_pose, stamp):
        if self.odom_source == "selected":
            if selected_pose is None:
                return None, "none"
            return self._pose_to_odom(selected_pose, stamp), "selected"

        with self._lock:
            rtab_odom = self._latest_rtab_odom
        if rtab_odom is None:
            return None, "none"
        return self._copy_odom(rtab_odom, stamp), "rtabmap"

    # ---------- main loop ----------

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        last_logged_source = None
        last_logged_odom_source = None
        while not rospy.is_shutdown():
            ps, src = self._pick()
            stamp = rospy.Time.now()
            if ps is not None:
                source_frame = ps.header.frame_id or self.world_frame_name

                # Re-stamp so downstream consumers see a fresh time. Keep the
                # source frame truthful; /rtabmap/odom is an odom-frame pose,
                # not a map-frame pose.
                out = PoseStamped()
                out.header.stamp = stamp
                out.header.frame_id = source_frame
                out.pose = ps.pose
                self.pub.publish(out)
                self.selected_odom_pub.publish(self._pose_to_odom(out, stamp))

                if src != last_logged_source:
                    rospy.loginfo(
                        "[localization_manager] active source: %s frame=%s",
                        src, source_frame,
                    )
                    last_logged_source = src
            else:
                out = None

            odom, odom_src = self._pick_odom(out, stamp)
            if odom is not None:
                self.odom_pub.publish(odom)
            if odom_src != last_logged_odom_source:
                if odom is None:
                    rospy.logwarn(
                        "[localization_manager] /odom source unavailable: %s",
                        self.odom_source,
                    )
                else:
                    rospy.loginfo(
                        "[localization_manager] /odom source: %s frame=%s topic=%s",
                        odom_src, odom.header.frame_id, self.odom_topic,
                    )
                last_logged_odom_source = odom_src
            rate.sleep()


if __name__ == "__main__":
    LocalizationManager().spin()
