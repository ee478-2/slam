#!/usr/bin/env python3
"""
Republish RTAB-Map and wheel odometry into one RViz comparison frame.

RTAB-Map odom and /wheel/odom intentionally use different frame ids. For a
visual drift check, publish both trajectories in a synthetic common frame,
assuming the two odometry origins were started from the same robot pose.
"""

import threading

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomCompareViz:
    def __init__(self):
        rospy.init_node("odom_compare_viz", anonymous=False)

        self.common_frame = rospy.get_param("~common_frame", "odom_compare")
        self.rtab_odom_topic = rospy.get_param("~rtab_odom_topic", "/rtabmap/odom")
        self.wheel_odom_topic = rospy.get_param("~wheel_odom_topic", "/wheel/odom")
        self.rtab_path_topic = rospy.get_param("~rtab_path_topic", "/rtabmap/mapPath")
        self.max_poses = int(rospy.get_param("~max_poses", 5000))

        self._lock = threading.Lock()
        self._rtab_path_from_topic = False
        self._rtab_path = Path()
        self._wheel_path = Path()
        self._rtab_path.header.frame_id = self.common_frame
        self._wheel_path.header.frame_id = self.common_frame

        self.rtab_odom_pub = rospy.Publisher(
            "/odom_compare/rtab_odom", Odometry, queue_size=20)
        self.wheel_odom_pub = rospy.Publisher(
            "/odom_compare/wheel_odom", Odometry, queue_size=20)
        self.rtab_path_pub = rospy.Publisher(
            "/odom_compare/rtab_path", Path, queue_size=5, latch=True)
        self.wheel_path_pub = rospy.Publisher(
            "/odom_compare/wheel_path", Path, queue_size=5, latch=True)

        rospy.Subscriber(self.rtab_odom_topic, Odometry, self._rtab_cb, queue_size=50)
        rospy.Subscriber(self.wheel_odom_topic, Odometry, self._wheel_cb, queue_size=50)
        if self.rtab_path_topic:
            rospy.Subscriber(self.rtab_path_topic, Path, self._rtab_path_cb, queue_size=5)

        rospy.loginfo(
            "[odom_compare_viz] rtab_odom=%s rtab_path=%s wheel=%s common_frame=%s max_poses=%d",
            self.rtab_odom_topic, self.rtab_path_topic or "(odom fallback)",
            self.wheel_odom_topic, self.common_frame, self.max_poses,
        )

    def _pose_stamped(self, msg):
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        ps.header.frame_id = self.common_frame
        ps.pose = msg.pose.pose
        return ps

    def _odom(self, msg, child_frame_id):
        out = Odometry()
        out.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        out.header.frame_id = self.common_frame
        out.child_frame_id = child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        return out

    def _append_path(self, path, ps):
        path.header.stamp = ps.header.stamp
        path.poses.append(ps)
        if self.max_poses > 0 and len(path.poses) > self.max_poses:
            del path.poses[:len(path.poses) - self.max_poses]

    def _rtab_cb(self, msg):
        ps = self._pose_stamped(msg)
        with self._lock:
            if not self._rtab_path_from_topic:
                self._append_path(self._rtab_path, ps)
                self.rtab_path_pub.publish(self._rtab_path)
        self.rtab_odom_pub.publish(self._odom(msg, "rtab_base"))

    def _rtab_path_cb(self, msg):
        path = Path()
        path.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        path.header.frame_id = self.common_frame
        poses = msg.poses[-self.max_poses:] if self.max_poses > 0 else msg.poses
        for pose in poses:
            ps = PoseStamped()
            ps.header.stamp = pose.header.stamp if pose.header.stamp else path.header.stamp
            ps.header.frame_id = self.common_frame
            ps.pose = pose.pose
            path.poses.append(ps)
        with self._lock:
            self._rtab_path_from_topic = True
            self._rtab_path = path
            self.rtab_path_pub.publish(self._rtab_path)

    def _wheel_cb(self, msg):
        ps = self._pose_stamped(msg)
        with self._lock:
            self._append_path(self._wheel_path, ps)
            self.wheel_path_pub.publish(self._wheel_path)
        self.wheel_odom_pub.publish(self._odom(msg, "wheel_base"))

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    OdomCompareViz().spin()
