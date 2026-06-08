#!/usr/bin/env python3
"""
Anchor RTAB-Map's local map frame to the room/global map with AprilTags.

apriltag_ros publishes detected bundle IDs and poses in the camera frame.
RTAB-Map also publishes its local SLAM frame, normally "map". This node uses
known signboard poses from config/global_map.yaml and the observed detection
pose in the RTAB frame to publish:

  global_map -> map

That keeps RTAB-Map untouched while allowing RViz and downstream consumers to
transform local RTAB data into the fixed room/global coordinate frame.
"""

import json
import math
import os
import threading
from collections import deque

import rospy
import tf2_ros
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
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


def median(values):
    ordered = sorted(float(v) for v in values)
    n = len(ordered)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def circular_mean(angles):
    if not angles:
        return 0.0
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    if abs(s) <= 1e-12 and abs(c) <= 1e-12:
        return angles[-1]
    return math.atan2(s, c)


def median_planar_transform(transforms):
    if not transforms:
        return Transform()
    return Transform(
        (
            median(tf.t[0] for tf in transforms),
            median(tf.t[1] for tf in transforms),
            median(tf.t[2] for tf in transforms),
        ),
        yaw_quat(circular_mean([yaw_from_quat(tf.q) for tf in transforms])),
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


def planar_anchor_transform(global_from_tag, rtab_from_tag, z=0.0):
    # Solve the 2D alignment directly; projecting a 6-DoF tag solve can move
    # the tag's x/y point when roll, pitch, or camera height are present.
    yaw = yaw_from_quat(global_from_tag.q) - yaw_from_quat(rtab_from_tag.q)
    c = math.cos(yaw)
    s = math.sin(yaw)
    rx, ry = rtab_from_tag.t[0], rtab_from_tag.t[1]
    tx = global_from_tag.t[0] - (c * rx - s * ry)
    ty = global_from_tag.t[1] - (s * rx + c * ry)
    return Transform((tx, ty, z), yaw_quat(yaw))


def anchor_transform(global_from_tag, rtab_from_tag, planar=True):
    if planar:
        return planar_anchor_transform(global_from_tag, rtab_from_tag)
    return compose(global_from_tag, inverse_tf(rtab_from_tag))


def planar_point_error(global_from_rtab, global_from_tag, rtab_from_tag):
    yaw = yaw_from_quat(global_from_rtab.q)
    c = math.cos(yaw)
    s = math.sin(yaw)
    rx, ry = rtab_from_tag.t[0], rtab_from_tag.t[1]
    px = global_from_rtab.t[0] + (c * rx - s * ry)
    py = global_from_rtab.t[1] + (s * rx + c * ry)
    dx = px - global_from_tag.t[0]
    dy = py - global_from_tag.t[1]
    return math.sqrt(dx * dx + dy * dy)


def transform_from_msg(msg):
    tr = msg.transform.translation
    rot = msg.transform.rotation
    return Transform((tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w))


def transform_from_pose(pose):
    p = pose.position
    q = pose.orientation
    return Transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


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


def stamp_or_now(stamp):
    if stamp == rospy.Time(0):
        return rospy.Time.now()
    return stamp


def stamp_gap_s(newer, older):
    if newer == rospy.Time(0) or older == rospy.Time(0):
        return 0.0
    return max(0.0, (newer - older).to_sec())


def clamp01(value):
    return max(0.0, min(1.0, float(value)))


class AprilTagGlobalLocalizer:
    def __init__(self):
        rospy.init_node("apriltag_global_localizer", anonymous=False)

        self.global_map_yaml = os.path.expanduser(
            rospy.get_param("~global_map_yaml", default_global_map_yaml())
        )
        self.global_frame = rospy.get_param("~global_frame", "global_map")
        self.rtabmap_frame = rospy.get_param("~rtabmap_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tag_detections_topic = rospy.get_param(
            "~tag_detections_topic", "/tag_detections"
        )
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.lookup_timeout_s = float(rospy.get_param("~lookup_timeout_s", 0.02))
        self.max_tag_age_s = float(rospy.get_param("~max_tag_age_s", 0.5))
        self.max_tag_distance_m = float(rospy.get_param("~max_tag_distance_m", 2.0))
        self.min_stable_frames = max(
            1, int(rospy.get_param("~min_stable_frames", 3))
        )
        self.smoothing_window_size = max(
            1, int(rospy.get_param("~smoothing_window_size", 5))
        )
        self.stable_max_frame_gap_s = max(
            0.0, float(rospy.get_param("~stable_max_frame_gap_s", 0.35))
        )
        self.axis_filter_enabled = get_bool_param("~axis_filter_enabled", True)
        self.front_axis_weight = clamp01(
            rospy.get_param("~front_axis_weight", 1.0)
        )
        self.side_axis_weight = clamp01(
            rospy.get_param("~side_axis_weight", 0.25)
        )
        self.hold_last_transform = get_bool_param("~hold_last_transform", True)
        self.constrain_to_planar = get_bool_param("~constrain_to_planar", True)
        self.apply_legacy_orientation_correction = get_bool_param(
            "~apply_legacy_orientation_correction", True
        )

        if self.global_frame == self.rtabmap_frame:
            raise rospy.ROSException("global_frame and rtabmap_frame must be different")

        (
            self.known_tags,
            self.tag_to_signboard,
            self.signboard_yaws,
        ) = self.load_known_signboards(self.global_map_yaml)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._lock = threading.Lock()
        self._latest_detections = []
        self._detection_stability = {}
        self._anchor_windows = {}

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
        self._detection_warming_up = False

        rospy.Subscriber(
            self.tag_detections_topic, AprilTagDetectionArray, self.on_tag_detections,
            queue_size=10,
        )

        rospy.loginfo(
            "[apriltag_global_localizer] loaded %d signboards / %d tag ids from %s; detections=%s; publishing %s -> %s planar=%s stable_frames=%d smoothing_window=%d axis_filter=%s front=%.2f side=%.2f",
            len(self.known_tags), len(self.tag_to_signboard), self.global_map_yaml,
            self.tag_detections_topic, self.global_frame, self.rtabmap_frame,
            self.constrain_to_planar, self.min_stable_frames,
            self.smoothing_window_size, self.axis_filter_enabled,
            self.front_axis_weight, self.side_axis_weight,
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
        tag_to_signboard = {}
        signboard_yaws = {}
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
            signboard_yaws[str(name)] = yaw
            for tag in item.get("tags", []):
                try:
                    tag_to_signboard[int(tag["id"])] = str(name)
                except (KeyError, TypeError, ValueError):
                    rospy.logwarn(
                        "[apriltag_global_localizer] skip invalid tag id under %s",
                        name,
                    )

        if not known:
            raise rospy.ROSException("No signboard poses loaded from %s" % path)
        if not tag_to_signboard:
            raise rospy.ROSException("No signboard tag ids loaded from %s" % path)
        return known, tag_to_signboard, signboard_yaws

    def detection_signboard_id(self, tag_ids):
        matches = {
            self.tag_to_signboard[tag_id]
            for tag_id in tag_ids
            if tag_id in self.tag_to_signboard
        }
        if len(matches) != 1:
            return None
        return next(iter(matches))

    def on_tag_detections(self, msg):
        frame_stamp = stamp_or_now(msg.header.stamp)
        detections_by_signboard = {}
        for det in msg.detections:
            tag_ids = [int(tag_id) for tag_id in det.id]
            signboard_id = self.detection_signboard_id(tag_ids)
            if signboard_id is None:
                rospy.logwarn_throttle(
                    5.0,
                    "[apriltag_global_localizer] skip detection ids=%s; no unique signboard match",
                    tag_ids,
                )
                continue

            frame_id = det.pose.header.frame_id or msg.header.frame_id
            if not frame_id:
                rospy.logwarn_throttle(
                    5.0,
                    "[apriltag_global_localizer] skip %s ids=%s; detection frame is empty",
                    signboard_id, tag_ids,
                )
                continue
            stamp = det.pose.header.stamp
            if stamp == rospy.Time(0):
                stamp = frame_stamp
            obs = {
                "stamp": stamp,
                "frame_id": frame_id,
                "signboard_id": signboard_id,
                "tag_ids": tag_ids,
                "camera_from_detection": transform_from_pose(det.pose.pose.pose),
            }
            previous = detections_by_signboard.get(signboard_id)
            if previous is None:
                detections_by_signboard[signboard_id] = obs
                continue
            prev_dist = math.sqrt(sum(v * v for v in previous["camera_from_detection"].t))
            obs_dist = math.sqrt(sum(v * v for v in obs["camera_from_detection"].t))
            if obs_dist < prev_dist:
                detections_by_signboard[signboard_id] = obs

        detections = list(detections_by_signboard.values())
        seen_signboards = set(detections_by_signboard)

        with self._lock:
            self._latest_detections = detections
            for signboard_id in seen_signboards:
                obs = detections_by_signboard[signboard_id]
                stamp = stamp_or_now(obs["stamp"])
                track = self._detection_stability.get(signboard_id)
                if track is None:
                    count = 1
                elif (
                    self.stable_max_frame_gap_s <= 0.0
                    or stamp_gap_s(stamp, track["stamp"]) <= self.stable_max_frame_gap_s
                ):
                    count = track["count"] + 1
                else:
                    count = 1
                self._detection_stability[signboard_id] = {
                    "count": count,
                    "stamp": stamp,
                    "tag_ids": obs["tag_ids"],
                }
                if count == 1:
                    self._anchor_windows.pop(signboard_id, None)

            for signboard_id, track in list(self._detection_stability.items()):
                if signboard_id in seen_signboards:
                    continue
                if (
                    self.stable_max_frame_gap_s > 0.0
                    and stamp_gap_s(frame_stamp, track["stamp"]) > self.stable_max_frame_gap_s
                ):
                    track["count"] = 0
                    self._anchor_windows.pop(signboard_id, None)

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

    def update_anchor_window(self, signboard_id, stamp, rtab_from_detection):
        if self.smoothing_window_size <= 1 or not self.constrain_to_planar:
            return rtab_from_detection, 1

        sample = {
            "stamp": stamp_or_now(stamp),
            "rtab_from_detection": rtab_from_detection,
        }
        with self._lock:
            window = self._anchor_windows.get(signboard_id)
            if window is None:
                window = deque(maxlen=self.smoothing_window_size)
                self._anchor_windows[signboard_id] = window

            if window and window[-1]["stamp"] == sample["stamp"]:
                window[-1] = sample
            else:
                window.append(sample)

            samples = [item["rtab_from_detection"] for item in window]

        return median_planar_transform(samples), len(samples)

    def apply_anchor_axis_filter(self, signboard_id, raw_global_from_rtab):
        diagnostics = {
            "axis_filter_applied": False,
            "axis_filter_front_delta_m": None,
            "axis_filter_side_delta_m": None,
        }
        if (
            not self.axis_filter_enabled
            or not self.constrain_to_planar
            or self.last_global_from_rtab is None
        ):
            return raw_global_from_rtab, diagnostics

        yaw = self.signboard_yaws.get(signboard_id)
        if yaw is None:
            return raw_global_from_rtab, diagnostics

        previous = self.last_global_from_rtab
        dx = raw_global_from_rtab.t[0] - previous.t[0]
        dy = raw_global_from_rtab.t[1] - previous.t[1]
        front_x = math.cos(yaw)
        front_y = math.sin(yaw)
        side_x = -front_y
        side_y = front_x

        front_delta = dx * front_x + dy * front_y
        side_delta = dx * side_x + dy * side_y
        filtered_front = self.front_axis_weight * front_delta
        filtered_side = self.side_axis_weight * side_delta
        tx = previous.t[0] + filtered_front * front_x + filtered_side * side_x
        ty = previous.t[1] + filtered_front * front_y + filtered_side * side_y

        diagnostics.update({
            "axis_filter_applied": True,
            "axis_filter_front_delta_m": front_delta,
            "axis_filter_side_delta_m": side_delta,
        })
        return Transform((tx, ty, raw_global_from_rtab.t[2]), raw_global_from_rtab.q), diagnostics

    def choose_detection_anchor(self, now):
        with self._lock:
            detections = list(self._latest_detections)
            stability = {
                key: value.copy()
                for key, value in self._detection_stability.items()
            }

        best = None
        best_unstable = None
        for obs in detections:
            if self.max_tag_age_s > 0.0 and obs["stamp"] != rospy.Time(0):
                if (now - obs["stamp"]).to_sec() > self.max_tag_age_s:
                    continue

            stable_state = stability.get(obs["signboard_id"], {})
            stable_count = int(stable_state.get("count", 0))

            try:
                _msg, rtab_from_camera = self.lookup_transform(
                    self.rtabmap_frame, obs["frame_id"]
                )
            except Exception as e:
                rospy.logwarn_throttle(
                    5.0,
                    "[apriltag_global_localizer] cannot transform %s -> %s for %s: %s",
                    self.rtabmap_frame, obs["frame_id"], obs["signboard_id"], e,
                )
                continue

            rtab_from_detection = compose(
                rtab_from_camera, obs["camera_from_detection"]
            )
            dist = self.distance_to_tag(obs["signboard_id"])
            if not math.isfinite(dist):
                dist = math.sqrt(sum(v * v for v in obs["camera_from_detection"].t))
            if dist > self.max_tag_distance_m:
                continue

            smoothed_rtab_from_detection, smoothing_samples = self.update_anchor_window(
                obs["signboard_id"], obs["stamp"], rtab_from_detection
            )

            if stable_count < self.min_stable_frames:
                unstable = (
                    stable_count,
                    obs["signboard_id"],
                    obs["tag_ids"],
                )
                if best_unstable is None or stable_count > best_unstable[0]:
                    best_unstable = unstable
                continue

            global_from_detection = self.known_tags[obs["signboard_id"]]
            candidate = (
                dist,
                obs["signboard_id"],
                global_from_detection,
                smoothed_rtab_from_detection,
                "detection",
                obs["tag_ids"],
                stable_count,
                smoothing_samples,
            )
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None and best_unstable is not None:
            stable_count, signboard_id, tag_ids = best_unstable
            rospy.loginfo_throttle(
                2.0,
                "[apriltag_global_localizer] waiting for stable %s ids=%s (%d/%d frames)",
                signboard_id, tag_ids, stable_count, self.min_stable_frames,
            )
        return best, best_unstable is not None

    def choose_tf_anchor(self, now):
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
            candidate = (dist, tag_name, global_from_tag, rtab_from_tag, "tf", [], None, None)
            if best is None or candidate[0] < best[0]:
                best = candidate
        return best

    def choose_visible_anchor(self, now):
        detection_anchor, detection_is_warming_up = self.choose_detection_anchor(now)
        self._detection_warming_up = detection_is_warming_up
        if detection_anchor is not None:
            return detection_anchor
        if detection_is_warming_up:
            return None
        return self.choose_tf_anchor(now)

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
            selected = self.choose_visible_anchor(now)
            if selected is not None:
                (
                    dist,
                    tag_name,
                    global_from_tag,
                    rtab_from_tag,
                    method,
                    tag_ids,
                    stable_count,
                    smoothing_samples,
                ) = selected
                raw_global_from_rtab = anchor_transform(
                    global_from_tag, rtab_from_tag, self.constrain_to_planar
                )
                global_from_rtab, axis_filter = self.apply_anchor_axis_filter(
                    tag_name, raw_global_from_rtab
                )
                anchor_error_m = planar_point_error(
                    global_from_rtab, global_from_tag, rtab_from_tag
                )
                raw_anchor_error_m = planar_point_error(
                    raw_global_from_rtab, global_from_tag, rtab_from_tag
                )
                self.last_global_from_rtab = global_from_rtab
                self.last_selected = tag_name
                self.publish_anchor(global_from_rtab, now)
                self.selected_tag_pub.publish(String(json.dumps({
                    "anchor_error_m": round(anchor_error_m, 4),
                    "tag": tag_name,
                    "distance_m": round(dist, 4),
                    "global_frame": self.global_frame,
                    "method": method,
                    "rtabmap_frame": self.rtabmap_frame,
                    "planar": self.constrain_to_planar,
                    "tag_ids": tag_ids,
                    "stable_frames": stable_count,
                    "min_stable_frames": self.min_stable_frames,
                    "smoothing_window_samples": smoothing_samples,
                    "smoothing_window_size": self.smoothing_window_size,
                    "axis_filter_applied": axis_filter["axis_filter_applied"],
                    "front_axis_weight": self.front_axis_weight,
                    "side_axis_weight": self.side_axis_weight,
                    "axis_filter_front_delta_m": (
                        None if axis_filter["axis_filter_front_delta_m"] is None
                        else round(axis_filter["axis_filter_front_delta_m"], 4)
                    ),
                    "axis_filter_side_delta_m": (
                        None if axis_filter["axis_filter_side_delta_m"] is None
                        else round(axis_filter["axis_filter_side_delta_m"], 4)
                    ),
                    "raw_anchor_error_m": round(raw_anchor_error_m, 4),
                }, sort_keys=True)))
                rospy.loginfo_throttle(
                    3.0,
                    "[apriltag_global_localizer] anchored %s -> %s using %s match=%s ids=%s stable=%s/%d smooth=%s/%d front_w=%.2f side_w=%.2f at %.2fm planar_error=%.3fm raw_error=%.3fm",
                    self.global_frame, self.rtabmap_frame, method, tag_name,
                    tag_ids,
                    stable_count if stable_count is not None else "tf",
                    self.min_stable_frames,
                    smoothing_samples if smoothing_samples is not None else "tf",
                    self.smoothing_window_size, self.front_axis_weight,
                    self.side_axis_weight, dist, anchor_error_m,
                    raw_anchor_error_m,
                )
            elif self.hold_last_transform and self.last_global_from_rtab is not None:
                self.publish_anchor(self.last_global_from_rtab, now)
                rospy.logwarn_throttle(
                    10.0,
                    "[apriltag_global_localizer] no fresh tag; holding last anchor from %s",
                    self.last_selected or "unknown",
                )
            elif self._detection_warming_up:
                rospy.loginfo_throttle(
                    2.0,
                    "[apriltag_global_localizer] visible tag is not stable enough to anchor yet",
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
