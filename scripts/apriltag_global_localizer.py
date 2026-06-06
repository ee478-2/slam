#!/usr/bin/env python3
"""
Anchor RTAB-Map's local map frame to the room/global map with AprilTags.

apriltag_ros publishes detected bundle frames such as SIGNBOARD05 in the live
TF tree. RTAB-Map also publishes its local SLAM frame, normally "map". This node
uses known signboard poses from config/global_map.yaml and the observed
rtabmap_frame -> SIGNBOARDxx transform to publish:

  global_map -> map

That keeps RTAB-Map untouched while allowing RViz and downstream consumers to
transform local RTAB data into the fixed room/global coordinate frame.
"""

import json
import math
import os

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String

try:
    import rospkg
except ImportError:
    rospkg = None


def default_global_map_yaml():
    if rospkg is not None:
        try:
            return os.path.join(rospkg.RosPack().get_path("slam"), "config", "global_map.yaml")
        except Exception:
            pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "global_map.yaml"))


def normalize_quat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_multiply(a, b):
    return normalize_quat(quat_multiply_raw(a, b))


def quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inverse(q):
    x, y, z, w = normalize_quat(q)
    return (-x, -y, -z, w)


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quat((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def yaw_from_quat(q):
    x, y, z, w = normalize_quat(q)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def rotate_vec(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_multiply_raw(quat_multiply_raw(normalize_quat(q), qv), quat_inverse(q))
    return (r[0], r[1], r[2])


def legacy_tag_orientation_correction():
    # Matches AprilTagLocalization/src/apriltag_localization.cpp:
    #   tf2::Quaternion(0, 90deg, 0) * tf2::Quaternion(-90deg, 0, 0)
    pitch_90 = quat_from_rpy(0.0, math.radians(90.0), 0.0)
    yaw_m90 = quat_from_rpy(0.0, 0.0, math.radians(-90.0))
    return quat_multiply(pitch_90, yaw_m90)


class Transform:
    __slots__ = ("t", "q")

    def __init__(self, translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
        self.t = tuple(float(v) for v in translation)
        self.q = normalize_quat(rotation)


def compose(a, b):
    rb = rotate_vec(a.q, b.t)
    return Transform(
        (a.t[0] + rb[0], a.t[1] + rb[1], a.t[2] + rb[2]),
        quat_multiply(a.q, b.q),
    )


def inverse_tf(tf):
    qi = quat_inverse(tf.q)
    ti = rotate_vec(qi, (-tf.t[0], -tf.t[1], -tf.t[2]))
    return Transform(ti, qi)


def planarize_transform(tf, z=0.0):
    return Transform((tf.t[0], tf.t[1], z), yaw_quat(yaw_from_quat(tf.q)))


def transform_from_msg(msg):
    tr = msg.transform.translation
    rot = msg.transform.rotation
    return Transform((tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w))


def transform_to_msg(tf, stamp, parent_frame, child_frame):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = tf.t[0]
    msg.transform.translation.y = tf.t[1]
    msg.transform.translation.z = tf.t[2]
    msg.transform.rotation.x = tf.q[0]
    msg.transform.rotation.y = tf.q[1]
    msg.transform.rotation.z = tf.q[2]
    msg.transform.rotation.w = tf.q[3]
    return msg


def pose_from_transform(tf, stamp, frame_id):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = tf.t[0]
    msg.pose.position.y = tf.t[1]
    msg.pose.position.z = tf.t[2]
    msg.pose.orientation.x = tf.q[0]
    msg.pose.orientation.y = tf.q[1]
    msg.pose.orientation.z = tf.q[2]
    msg.pose.orientation.w = tf.q[3]
    return msg


def get_bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class AprilTagGlobalLocalizer:
    def __init__(self):
        rospy.init_node("apriltag_global_localizer", anonymous=False)

        self.global_map_yaml = os.path.expanduser(
            rospy.get_param("~global_map_yaml", default_global_map_yaml())
        )
        self.global_frame = rospy.get_param("~global_frame", "global_map")
        self.rtabmap_frame = rospy.get_param("~rtabmap_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.lookup_timeout_s = float(rospy.get_param("~lookup_timeout_s", 0.02))
        self.max_tag_age_s = float(rospy.get_param("~max_tag_age_s", 0.5))
        self.max_tag_distance_m = float(rospy.get_param("~max_tag_distance_m", 2.0))
        self.hold_last_transform = get_bool_param("~hold_last_transform", True)
        self.constrain_to_planar = get_bool_param("~constrain_to_planar", True)
        self.apply_legacy_orientation_correction = get_bool_param(
            "~apply_legacy_orientation_correction", True
        )

        if self.global_frame == self.rtabmap_frame:
            raise rospy.ROSException("global_frame and rtabmap_frame must be different")

        self.known_tags = self.load_known_signboards(self.global_map_yaml)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.transform_pub = rospy.Publisher(
            "/global_localization/transform", TransformStamped, queue_size=5
        )
        self.robot_pose_pub = rospy.Publisher(
            "/global_localization/robot_pose", PoseStamped, queue_size=5
        )
        self.selected_tag_pub = rospy.Publisher(
            "/global_localization/selected_tag", String, queue_size=5
        )

        self.last_global_from_rtab = None
        self.last_selected = None

        rospy.loginfo(
            "[apriltag_global_localizer] loaded %d signboard poses from %s; publishing %s -> %s planar=%s",
            len(self.known_tags), self.global_map_yaml, self.global_frame,
            self.rtabmap_frame, self.constrain_to_planar,
        )

    def load_known_signboards(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        correction = (
            legacy_tag_orientation_correction()
            if self.apply_legacy_orientation_correction
            else (0.0, 0.0, 0.0, 1.0)
        )
        known = {}
        for name, item in (data.get("signboards") or {}).items():
            pose = item.get("pose") or {}
            try:
                x = float(pose["x"])
                y = float(pose["y"])
                z = float(pose.get("z", 0.0))
                yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
            except (KeyError, TypeError, ValueError):
                rospy.logwarn("[apriltag_global_localizer] skip invalid signboard pose: %s", name)
                continue
            q = quat_multiply(yaw_quat(yaw), correction)
            known[str(name)] = Transform((x, y, z), q)

        if not known:
            raise rospy.ROSException("No signboard poses loaded from %s" % path)
        return known

    def lookup_transform(self, target_frame, source_frame, timeout_s=None):
        timeout = rospy.Duration(self.lookup_timeout_s if timeout_s is None else timeout_s)
        msg = self.tf_buffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0), timeout
        )
        return msg, transform_from_msg(msg)

    def transform_age_ok(self, msg, now):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0) or self.max_tag_age_s <= 0.0:
            return True
        return (now - stamp).to_sec() <= self.max_tag_age_s

    def distance_to_tag(self, tag_frame):
        try:
            _msg, base_from_tag = self.lookup_transform(
                self.base_frame, tag_frame, timeout_s=0.0
            )
            return math.sqrt(sum(v * v for v in base_from_tag.t))
        except Exception:
            return float("inf")

    def choose_visible_tag(self, now):
        best = None
        for tag_name, global_from_tag in self.known_tags.items():
            try:
                obs_msg, rtab_from_tag = self.lookup_transform(self.rtabmap_frame, tag_name)
            except Exception:
                continue
            if not self.transform_age_ok(obs_msg, now):
                continue
            dist = self.distance_to_tag(tag_name)
            if not math.isfinite(dist):
                dist = math.sqrt(sum(v * v for v in rtab_from_tag.t))
            if dist > self.max_tag_distance_m:
                continue
            candidate = (dist, tag_name, global_from_tag, rtab_from_tag)
            if best is None or candidate[0] < best[0]:
                best = candidate
        return best

    def publish_robot_pose(self, global_from_rtab, stamp):
        try:
            _msg, rtab_from_base = self.lookup_transform(
                self.rtabmap_frame, self.base_frame, timeout_s=0.0
            )
        except Exception:
            return
        global_from_base = compose(global_from_rtab, rtab_from_base)
        self.robot_pose_pub.publish(pose_from_transform(global_from_base, stamp, self.global_frame))

    def publish_anchor(self, global_from_rtab, stamp):
        tf_msg = transform_to_msg(
            global_from_rtab, stamp, self.global_frame, self.rtabmap_frame
        )
        self.tf_broadcaster.sendTransform(tf_msg)
        self.transform_pub.publish(tf_msg)
        self.publish_robot_pose(global_from_rtab, stamp)

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            selected = self.choose_visible_tag(now)
            if selected is not None:
                dist, tag_name, global_from_tag, rtab_from_tag = selected
                global_from_rtab = compose(global_from_tag, inverse_tf(rtab_from_tag))
                if self.constrain_to_planar:
                    global_from_rtab = planarize_transform(global_from_rtab)
                self.last_global_from_rtab = global_from_rtab
                self.last_selected = tag_name
                self.publish_anchor(global_from_rtab, now)
                self.selected_tag_pub.publish(String(json.dumps({
                    "tag": tag_name,
                    "distance_m": round(dist, 4),
                    "global_frame": self.global_frame,
                    "rtabmap_frame": self.rtabmap_frame,
                    "planar": self.constrain_to_planar,
                }, sort_keys=True)))
                rospy.loginfo_throttle(
                    3.0,
                    "[apriltag_global_localizer] anchored %s -> %s using %s at %.2fm",
                    self.global_frame, self.rtabmap_frame, tag_name, dist,
                )
            elif self.hold_last_transform and self.last_global_from_rtab is not None:
                self.publish_anchor(self.last_global_from_rtab, now)
                rospy.logwarn_throttle(
                    10.0,
                    "[apriltag_global_localizer] no fresh tag; holding last anchor from %s",
                    self.last_selected or "unknown",
                )
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "[apriltag_global_localizer] waiting for visible signboard TFs in %s",
                    self.rtabmap_frame,
                )
            rate.sleep()


if __name__ == "__main__":
    AprilTagGlobalLocalizer().spin()
