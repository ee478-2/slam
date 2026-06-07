#!/usr/bin/env python3
"""
Collect RGB frames for storefront YOLO training while the robot is driven by teleop.

Run the camera/SLAM stack, start this node, then drive with teleop in another
terminal. The node saves JPEG frames plus a metadata CSV containing image stamp
and the latest odometry pose when available.
"""

import csv
import math
import os
import sys
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Image


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class StorefrontDatasetCollector:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.output_root = os.path.expanduser(
            rospy.get_param(
                "~output_root",
                os.path.join(os.getcwd(), "data", "storefront_yolo", "raw"),
            )
        )
        self.session = rospy.get_param(
            "~session", time.strftime("session_%Y%m%d_%H%M%S")
        )
        self.capture_hz = float(rospy.get_param("~capture_hz", 2.0))
        self.min_translation_m = float(rospy.get_param("~min_translation_m", 0.08))
        self.min_yaw_rad = math.radians(float(rospy.get_param("~min_yaw_deg", 6.0)))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 92))
        self.max_images = int(rospy.get_param("~max_images", 0))
        self.startup_delay_sec = float(rospy.get_param("~startup_delay_sec", 0.0))
        class_param = rospy.get_param("~class_names", "storefront")
        if isinstance(class_param, (list, tuple)):
            class_iter = class_param
        else:
            class_iter = str(class_param).split(",")
        self.class_names = [str(item).strip() for item in class_iter if str(item).strip()]

        self.bridge = CvBridge()
        self.latest_odom = None
        self.last_saved_pose = None
        self.last_save_time = 0.0
        self.saved_count = 0
        self.start_time = rospy.get_time()

        self.session_dir = os.path.join(self.output_root, self.session)
        self.image_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.image_dir, exist_ok=True)
        with open(os.path.join(self.session_dir, "classes.txt"), "w") as class_file:
            class_file.write("\n".join(self.class_names) + "\n")

        self.metadata_path = os.path.join(self.session_dir, "metadata.csv")
        self.metadata_file = open(self.metadata_path, "w", newline="")
        self.metadata = csv.writer(self.metadata_file)
        self.metadata.writerow(
            [
                "image",
                "image_stamp",
                "received_time",
                "width",
                "height",
                "odom_stamp",
                "odom_frame",
                "odom_child_frame",
                "odom_x",
                "odom_y",
                "odom_z",
                "odom_yaw_rad",
            ]
        )

        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=10)
        rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)
        rospy.on_shutdown(self.close)

        rospy.loginfo("[storefront_collector] image_topic=%s", self.image_topic)
        rospy.loginfo("[storefront_collector] odom_topic=%s", self.odom_topic)
        rospy.loginfo("[storefront_collector] output=%s", self.session_dir)
        rospy.loginfo(
            "[storefront_collector] capture_hz=%.2f min_translation=%.3fm min_yaw=%.1fdeg",
            self.capture_hz,
            self.min_translation_m,
            math.degrees(self.min_yaw_rad),
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_odom = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "yaw": yaw,
        }

    def on_image(self, msg):
        now = rospy.get_time()
        if now - self.start_time < self.startup_delay_sec:
            return
        if self.max_images > 0 and self.saved_count >= self.max_images:
            rospy.signal_shutdown("max_images reached")
            return
        if not self.should_save(now):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "[storefront_collector] cv_bridge: %s", exc)
            return

        next_count = self.saved_count + 1
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = now
        name = "frame_%06d_%.6f.jpg" % (next_count, stamp)
        path = os.path.join(self.image_dir, name)
        ok = cv2.imwrite(
            path,
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.jpeg_quality))],
        )
        if not ok:
            rospy.logwarn("[storefront_collector] failed to write %s", path)
            return

        self.saved_count = next_count
        odom = self.latest_odom or {}
        self.metadata.writerow(
            [
                os.path.join("images", name),
                "%.9f" % stamp,
                "%.9f" % now,
                int(msg.width),
                int(msg.height),
                self.fmt(odom.get("stamp")),
                odom.get("frame_id", ""),
                odom.get("child_frame_id", ""),
                self.fmt(odom.get("x")),
                self.fmt(odom.get("y")),
                self.fmt(odom.get("z")),
                self.fmt(odom.get("yaw")),
            ]
        )
        self.metadata_file.flush()

        self.last_save_time = now
        self.last_saved_pose = self.pose_tuple(odom)
        rospy.loginfo_throttle(
            5.0,
            "[storefront_collector] saved %d frames in %s",
            self.saved_count,
            self.session_dir,
        )

    def should_save(self, now):
        if self.capture_hz > 0.0:
            min_period = 1.0 / self.capture_hz
            if self.last_save_time > 0.0 and now - self.last_save_time < min_period:
                return False

        current_pose = self.pose_tuple(self.latest_odom or {})
        if self.last_saved_pose is None or current_pose is None:
            return True

        dx = current_pose[0] - self.last_saved_pose[0]
        dy = current_pose[1] - self.last_saved_pose[1]
        dist = math.hypot(dx, dy)
        dyaw = abs(angle_delta(current_pose[2], self.last_saved_pose[2]))
        return dist >= self.min_translation_m or dyaw >= self.min_yaw_rad

    @staticmethod
    def pose_tuple(odom):
        if not odom:
            return None
        if odom.get("x") is None or odom.get("y") is None or odom.get("yaw") is None:
            return None
        return (float(odom["x"]), float(odom["y"]), float(odom["yaw"]))

    @staticmethod
    def fmt(value):
        return "" if value is None else "%.9f" % float(value)

    def close(self):
        try:
            self.metadata_file.close()
        except Exception:
            pass
        if self.saved_count:
            rospy.loginfo(
                "[storefront_collector] wrote %d images and %s",
                self.saved_count,
                self.metadata_path,
            )


def main():
    rospy.init_node("storefront_dataset_collector")
    try:
        StorefrontDatasetCollector()
    except OSError as exc:
        rospy.logerr("[storefront_collector] cannot initialize output: %s", exc)
        sys.exit(1)
    rospy.spin()


if __name__ == "__main__":
    main()
