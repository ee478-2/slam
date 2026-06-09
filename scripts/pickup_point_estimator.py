#!/usr/bin/env python3
"""
Publish the currently visible pickup target pose relative to the robot.

The YOLO pose model labels pickup as class 0. yolo_pose_tag_detector maps class
0 to AprilTag-style detection id 1000 by default. This node filters that
detection and transforms it into base_link for visual-servo consumers.
"""

import json
import math
import threading

import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def detection_has_id(detection, tag_id):
    return any(int(value) == int(tag_id) for value in detection.id)


class PickupPointEstimator:
    def __init__(self):
        rospy.init_node("pickup_point_estimator", anonymous=False)

        self.detections_topic = rospy.get_param("~detections_topic", "/tag_detections")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/pickup_point/relative_pose"
        )
        self.status_topic = rospy.get_param("~status_topic", "/pickup_point/status")
        self.pickup_tag_id = int(rospy.get_param("~pickup_tag_id", 1000))
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.max_age_s = float(rospy.get_param("~max_age_s", 0.5))
        self.tf_timeout_s = float(rospy.get_param("~tf_timeout_s", 0.1))
        self.status_hz = float(rospy.get_param("~status_hz", 5.0))
        self.allow_latest_tf = self._bool_param("~allow_latest_tf", True)
        self.restamp_output = self._bool_param("~restamp_output", True)

        self._lock = threading.Lock()
        self._last_pose = None
        self._last_source_frame = ""
        self._last_detection_stamp = None
        self._last_seen = None
        self._last_error = ""

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=5)
        self.status_pub = rospy.Publisher(
            self.status_topic,
            String,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            self.detections_topic,
            AprilTagDetectionArray,
            self.on_detections,
            queue_size=5,
        )
        rospy.Timer(
            rospy.Duration.from_sec(1.0 / max(0.1, self.status_hz)),
            self.on_status_timer,
        )

        rospy.loginfo(
            "[pickup_point_estimator] detections=%s pickup_tag_id=%d target_frame=%s pose=%s status=%s max_age=%.2fs",
            self.detections_topic,
            self.pickup_tag_id,
            self.target_frame,
            self.pose_topic,
            self.status_topic,
            self.max_age_s,
        )

    @staticmethod
    def _bool_param(name, default=False):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def on_detections(self, msg):
        candidates = []
        for detection in msg.detections:
            if not detection_has_id(detection, self.pickup_tag_id):
                continue
            pose = self.detection_pose_stamped(detection, msg)
            transformed = self.transform_pose(pose)
            if transformed is not None:
                candidates.append(transformed)

        if not candidates:
            return

        selected = min(candidates, key=self.pose_distance)
        now = rospy.Time.now()
        if self.restamp_output:
            selected.header.stamp = now
        with self._lock:
            self._last_pose = selected
            self._last_source_frame = selected.header.frame_id
            self._last_detection_stamp = msg.header.stamp
            self._last_seen = now
            self._last_error = ""
        self.pose_pub.publish(selected)
        self.publish_status()

    def detection_pose_stamped(self, detection, msg):
        pose = PoseStamped()
        pose.header = detection.pose.header
        if not pose.header.frame_id:
            pose.header.frame_id = msg.header.frame_id
        if pose.header.stamp == rospy.Time(0):
            pose.header.stamp = msg.header.stamp
        pose.pose = detection.pose.pose.pose
        return pose

    def transform_pose(self, pose):
        try:
            return self.tf_buffer.transform(
                pose,
                self.target_frame,
                rospy.Duration.from_sec(self.tf_timeout_s),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            if not self.allow_latest_tf:
                self.set_error("tf_failed: %s" % exc)
                return None
            latest_pose = PoseStamped()
            latest_pose.header = pose.header
            latest_pose.header.stamp = rospy.Time(0)
            latest_pose.pose = pose.pose
            try:
                return self.tf_buffer.transform(
                    latest_pose,
                    self.target_frame,
                    rospy.Duration.from_sec(self.tf_timeout_s),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as latest_exc:
                self.set_error("tf_failed: %s" % latest_exc)
                return None

    @staticmethod
    def pose_distance(pose):
        p = pose.pose.position
        return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

    def set_error(self, message):
        with self._lock:
            self._last_error = message
        rospy.logwarn_throttle(1.0, "[pickup_point_estimator] %s", message)
        self.publish_status()

    def on_status_timer(self, _event):
        self.publish_status()

    def publish_status(self):
        now = rospy.Time.now()
        with self._lock:
            pose = self._last_pose
            last_seen = self._last_seen
            detection_stamp = self._last_detection_stamp
            error = self._last_error

        if pose is None or last_seen is None:
            payload = {
                "visible": False,
                "tag_id": self.pickup_tag_id,
                "target_frame": self.target_frame,
                "error": error,
            }
        else:
            age_s = max(0.0, (now - last_seen).to_sec())
            visible = age_s <= self.max_age_s
            p = pose.pose.position
            payload = {
                "visible": visible,
                "tag_id": self.pickup_tag_id,
                "target_frame": pose.header.frame_id,
                "x": round(p.x, 4),
                "y": round(p.y, 4),
                "z": round(p.z, 4),
                "yaw": round(yaw_from_quat(pose.pose.orientation), 4),
                "distance": round(self.pose_distance(pose), 4),
                "age_s": round(age_s, 3),
                "error": error,
            }
            if detection_stamp is not None and detection_stamp != rospy.Time(0):
                payload["detection_stamp"] = {
                    "secs": int(detection_stamp.secs),
                    "nsecs": int(detection_stamp.nsecs),
                }

        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    PickupPointEstimator().spin()
