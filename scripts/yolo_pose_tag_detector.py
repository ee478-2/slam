#!/usr/bin/env python3
"""
Publish YOLO-pose square-tag detections as apriltag_ros landmarks for RTAB-Map.

The detector expects a YOLO pose model whose keypoints are the four corners of a
single square target. When all four corners are visible, it solves a normal PnP
pose, but the translation is biased toward the horizontal pixel width because
the vertical extent may be occlusion-limited in the training labels. When only a
horizontal edge is reliable, it falls back to a width-based translation estimate
so RTAB can still receive a soft landmark observation.
"""

import json
import math
import os
import threading

import cv2
import numpy as np
import rospy
import yaml
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

try:
    import rospkg
except ImportError:
    rospkg = None


def default_model_path():
    if rospkg is not None:
        try:
            return os.path.join(rospkg.RosPack().get_path("slam"), "pose_best.engine")
        except Exception:
            pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pose_best.engine"))


def resolve_model_path(path):
    path = os.path.expanduser(path or default_model_path())
    if os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    if ext == ".engine":
        for fallback_ext in (".onnx", ".pt"):
            fallback = root + fallback_ext
            if not os.path.exists(fallback):
                continue
            rospy.logwarn(
                "[yolo_pose_tag_detector] optimized engine missing (%s); falling back to %s",
                path, fallback,
            )
            return fallback
    return path


def default_global_map_yaml():
    if rospkg is not None:
        try:
            return os.path.join(rospkg.RosPack().get_path("slam"), "config", "global_map.yaml")
        except Exception:
            pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "global_map.yaml"))


def get_bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_int_list(value, expected_len=None):
    items = [int(v.strip()) for v in str(value).split(",") if v.strip()]
    if expected_len is not None and len(items) != expected_len:
        raise ValueError("%s must contain %d integers" % (value, expected_len))
    return items


def parse_horizontal_pairs(value):
    pairs = []
    for item in str(value).split(";"):
        item = item.strip()
        if not item:
            continue
        pair = parse_int_list(item, expected_len=2)
        pairs.append((pair[0], pair[1]))
    if not pairs:
        raise ValueError("horizontal_pairs must contain at least one i,j pair")
    return pairs


def parse_class_id_map(value):
    mapping = {}
    text = str(value).strip()
    if not text:
        return mapping
    for item in text.split(","):
        key, tag_id = item.split(":", 1)
        mapping[int(key.strip())] = int(tag_id.strip())
    return mapping


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    q = q / n
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def quat_from_rotmat(rot):
    m = np.asarray(rot, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return normalize_quat((qx, qy, qz, qw))


def slerp_quat(q0, q1, alpha):
    q0 = np.asarray(normalize_quat(q0), dtype=float)
    q1 = np.asarray(normalize_quat(q1), dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return normalize_quat(q0 + alpha * (q1 - q0))
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * alpha
    sin_theta = math.sin(theta)
    sin_theta0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return normalize_quat(s0 * q0 + s1 * q1)


def make_covariance(linear_vars, angular_var):
    if isinstance(linear_vars, (list, tuple, np.ndarray)):
        if len(linear_vars) != 3:
            raise ValueError("linear_vars must be scalar or length 3")
        x_var, y_var, z_var = [float(v) for v in linear_vars]
    else:
        x_var = y_var = z_var = float(linear_vars)
    cov = [0.0] * 36
    cov[0] = x_var
    cov[7] = y_var
    cov[14] = z_var
    for idx in (21, 28, 35):
        cov[idx] = float(angular_var)
    return cov


class YoloPoseTagDetector:
    def __init__(self):
        rospy.init_node("yolo_pose_tag_detector", anonymous=False)

        self.model_path = resolve_model_path(
            rospy.get_param("~model_path", default_model_path())
        )
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.output_topic = rospy.get_param("~output_topic", "/tag_detections")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/yolo_pose_tag_detector/status"
        )
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/yolo_pose_tag_detector/debug_image"
        )
        self.publish_debug_image_enabled = get_bool_param("~publish_debug_image", True)
        self.global_map_yaml = os.path.expanduser(
            rospy.get_param("~global_map_yaml", default_global_map_yaml())
        )
        self.publish_global_store_status = get_bool_param(
            "~publish_global_store_status", True
        )
        self.tag_size_m = float(rospy.get_param("~tag_size_m", 0.15))
        self.base_tag_id = int(rospy.get_param("~base_tag_id", 1000))
        self.class_id_to_tag_id = parse_class_id_map(
            rospy.get_param("~class_id_to_tag_id", "")
        )
        self.keypoint_order = parse_int_list(
            rospy.get_param("~keypoint_order", "0,1,2,3"), expected_len=4
        )
        self.horizontal_pairs = parse_horizontal_pairs(
            rospy.get_param("~horizontal_pairs", "0,1;3,2")
        )
        self.min_box_conf = float(rospy.get_param("~min_box_conf", 0.50))
        self.min_keypoint_conf = float(rospy.get_param("~min_keypoint_conf", 0.45))
        self.inference_hz = float(rospy.get_param("~inference_hz", 5.0))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "")
        self.min_stable_frames = max(
            1, int(rospy.get_param("~min_stable_frames", 5))
        )
        self.ema_alpha = max(
            0.0, min(1.0, float(rospy.get_param("~ema_alpha", 0.35)))
        )
        self.allow_horizontal_fallback = get_bool_param(
            "~allow_horizontal_fallback", True
        )
        self.pnp_horizontal_translation_weight = max(
            0.0,
            min(
                1.0,
                float(rospy.get_param("~pnp_horizontal_translation_weight", 0.80)),
            ),
        )
        self.pnp_linear_variance = (
            float(rospy.get_param("~horizontal_linear_variance", 0.20)),
            float(rospy.get_param("~vertical_linear_variance", 1.50)),
            float(rospy.get_param("~depth_linear_variance", 0.50)),
        )
        self.fallback_linear_variance = (
            float(rospy.get_param("~fallback_horizontal_linear_variance", 0.40)),
            float(rospy.get_param("~fallback_vertical_linear_variance", 2.50)),
            float(rospy.get_param("~fallback_depth_linear_variance", 0.80)),
        )
        self.angular_variance = float(rospy.get_param("~angular_variance", 9999.0))

        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_lock = threading.Lock()
        self.last_inference_time = rospy.Time(0)
        self.processed_frame = 0
        self.track_state = {}

        self.object_points = self._make_square_object_points()
        self.global_store_by_class_id = self.load_global_store_order()
        self.det_pub = rospy.Publisher(
            self.output_topic, AprilTagDetectionArray, queue_size=5
        )
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=5)
        self.debug_image_pub = rospy.Publisher(
            self.debug_image_topic, Image, queue_size=2
        )

        from ultralytics import YOLO
        self.model = YOLO(self.model_path, task="pose")
        self.class_names = getattr(self.model.model, "names", {}) or {}

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.on_camera_info, queue_size=1
        )
        rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)

        rospy.loginfo(
            "[yolo_pose_tag_detector] model=%s output=%s debug_image=%s tag_size=%.3fm stable=%d ema=%.2f fallback=%s pnp_horizontal_weight=%.2f",
            self.model_path, self.output_topic, self.debug_image_topic, self.tag_size_m,
            self.min_stable_frames, self.ema_alpha,
            self.allow_horizontal_fallback, self.pnp_horizontal_translation_weight,
        )

    def _make_square_object_points(self):
        half = self.tag_size_m * 0.5
        canonical = np.array([
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ], dtype=np.float32)
        return canonical

    def load_global_store_order(self):
        if not self.publish_global_store_status:
            return {}
        try:
            with open(self.global_map_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn(
                "[yolo_pose_tag_detector] cannot load global map %s: %s",
                self.global_map_yaml, exc,
            )
            return {}

        mapping = {}
        for offset, store in enumerate(data.get("stores") or [], start=1):
            try:
                mapping[offset] = {
                    "id": str(store["id"]),
                    "category": str(store.get("category", "")),
                    "x": float(store["x"]),
                    "y": float(store["y"]),
                }
            except (KeyError, TypeError, ValueError):
                rospy.logwarn(
                    "[yolo_pose_tag_detector] skip invalid global store entry: %s",
                    store,
                )
        rospy.loginfo(
            "[yolo_pose_tag_detector] loaded %d global store mappings from %s",
            len(mapping), self.global_map_yaml,
        )
        return mapping

    def on_camera_info(self, msg):
        with self.camera_lock:
            self.camera_info = msg

    def should_run_inference(self, stamp):
        if self.inference_hz <= 0.0:
            return True
        if self.last_inference_time == rospy.Time(0):
            return True
        return (stamp - self.last_inference_time).to_sec() >= 1.0 / self.inference_hz

    def on_image(self, msg):
        now = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if not self.should_run_inference(now):
            return
        self.last_inference_time = now

        with self.camera_lock:
            camera_info = self.camera_info
        if camera_info is None:
            rospy.logwarn_throttle(
                5.0, "[yolo_pose_tag_detector] waiting for %s", self.camera_info_topic
            )
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "[yolo_pose_tag_detector] image conversion failed: %s", exc
            )
            return

        self.processed_frame += 1
        candidates, overlays = self.detect_candidates(image, msg, camera_info)
        detections, status = self.update_tracks(candidates, msg, camera_info)
        status["model_detections"] = len(overlays)
        self.det_pub.publish(AprilTagDetectionArray(
            header=msg.header,
            detections=detections,
        ))
        self.debug_pub.publish(String(json.dumps(status, sort_keys=True)))
        self.publish_debug_overlay(image, overlays, msg.header)

    def detect_candidates(self, image, image_msg, camera_info):
        kwargs = {
            "verbose": False,
            "conf": self.min_box_conf,
            "imgsz": self.imgsz,
        }
        if self.device:
            kwargs["device"] = self.device
        results = self.model(image, **kwargs)
        if not results:
            return [], []
        result = results[0]
        if result.keypoints is None or result.boxes is None:
            return [], []
        result_names = getattr(result, "names", None) or {}
        if result_names and not self.class_names:
            self.class_names = result_names

        keypoints_xy = result.keypoints.xy.cpu().numpy()
        keypoints_conf = (
            result.keypoints.conf.cpu().numpy()
            if result.keypoints.conf is not None
            else np.ones(keypoints_xy.shape[:2], dtype=float)
        )
        classes = (
            result.boxes.cls.cpu().numpy().astype(int)
            if result.boxes.cls is not None
            else np.zeros((len(keypoints_xy),), dtype=int)
        )
        box_conf = (
            result.boxes.conf.cpu().numpy()
            if result.boxes.conf is not None
            else np.ones((len(keypoints_xy),), dtype=float)
        )

        camera_matrix = np.array(camera_info.K, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(camera_info.D, dtype=np.float64).reshape((-1, 1))
        candidates_by_tag = {}
        overlays = []
        for idx, points in enumerate(keypoints_xy):
            cls_id = int(classes[idx])
            tag_id = self.class_id_to_tag_id.get(cls_id, self.base_tag_id + cls_id)
            score = float(box_conf[idx])
            class_names = result_names or self.class_names
            class_name = str(class_names.get(cls_id, cls_id))
            conf = keypoints_conf[idx]
            overlay = {
                "tag_id": int(tag_id),
                "class_id": int(cls_id),
                "class_name": class_name,
                "score": score,
                "keypoints_xy": np.asarray(points, dtype=float).tolist(),
                "keypoints_conf": np.asarray(conf, dtype=float).tolist(),
                "accepted": False,
                "reason": "low_box_conf",
            }
            if score < self.min_box_conf:
                overlays.append(overlay)
                continue

            candidate = self.solve_candidate(
                tag_id, cls_id, score, points, conf, camera_matrix, dist_coeffs
            )
            if candidate is None:
                overlay["reason"] = "no_pose"
                overlays.append(overlay)
                continue
            candidate["class_name"] = class_name
            candidate["global_store"] = self.global_store_by_class_id.get(cls_id)
            candidate["keypoints_xy"] = overlay["keypoints_xy"]
            candidate["keypoints_conf"] = overlay["keypoints_conf"]
            candidate["box_score"] = score
            overlay["accepted"] = True
            overlay["reason"] = "accepted"
            overlay["method"] = candidate["method"]
            overlay["horizontal_pairs"] = candidate.get("horizontal_pairs", 0)
            overlays.append(overlay)
            previous = candidates_by_tag.get(tag_id)
            if previous is None or candidate["score"] > previous["score"]:
                candidates_by_tag[tag_id] = candidate

        return list(candidates_by_tag.values()), overlays

    def publish_debug_overlay(self, image, overlays, header):
        if not self.publish_debug_image_enabled:
            return
        if self.debug_image_pub.get_num_connections() == 0:
            return

        debug = image.copy()
        if overlays:
            for item in overlays:
                self.draw_overlay_item(debug, item)
        else:
            self.draw_label(debug, "no yolo pose detection", (10, 24), (160, 160, 160))

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            msg.header = header
            self.debug_image_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "[yolo_pose_tag_detector] debug image publish failed: %s", exc
            )

    def draw_overlay_item(self, image, item):
        tag_id = int(item["tag_id"])
        state = self.track_state.get(tag_id, {})
        count = int(state.get("count", 0))
        stable = count >= self.min_stable_frames
        accepted = bool(item.get("accepted"))
        color = (40, 220, 80) if stable else (0, 190, 255)
        if not accepted:
            color = (60, 60, 255)

        points = item.get("keypoints_xy") or []
        confs = item.get("keypoints_conf") or []
        valid_points = []
        for index, point in enumerate(points):
            if len(point) < 2:
                continue
            px = float(point[0])
            py = float(point[1])
            if not np.isfinite([px, py]).all():
                continue
            x = int(round(px))
            y = int(round(py))
            conf = float(confs[index]) if index < len(confs) else 1.0
            valid = conf >= self.min_keypoint_conf
            if valid:
                valid_points.append((index, x, y))
                cv2.circle(image, (x, y), 4, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(image, (x, y), 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
                cv2.putText(
                    image, str(index), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, color, 1, cv2.LINE_AA,
                )
            elif 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.drawMarker(
                    image, (x, y), (60, 60, 255), markerType=cv2.MARKER_TILTED_CROSS,
                    markerSize=10, thickness=1, line_type=cv2.LINE_AA,
                )

        valid_by_index = {index: (x, y) for index, x, y in valid_points}
        for left_idx, right_idx in self.horizontal_pairs:
            if left_idx in valid_by_index and right_idx in valid_by_index:
                cv2.line(
                    image, valid_by_index[left_idx], valid_by_index[right_idx],
                    color, 2, lineType=cv2.LINE_AA,
                )

        ordered = [
            valid_by_index[idx] for idx in self.keypoint_order
            if idx in valid_by_index
        ]
        if len(ordered) == len(self.keypoint_order):
            cv2.polylines(
                image, [np.asarray(ordered, dtype=np.int32)], True,
                color, 1, lineType=cv2.LINE_AA,
            )

        label = self.overlay_label(item, count, stable)
        if valid_points:
            x0 = min(x for _index, x, _y in valid_points)
            y0 = min(y for _index, _x, y in valid_points) - 10
            anchor = (max(6, x0), max(18, y0))
        else:
            anchor = (10, 24)
        self.draw_label(image, label, anchor, color)

    def overlay_label(self, item, count, stable):
        class_name = item.get("class_name", item.get("class_id", "?"))
        method = str(item.get("method") or item.get("reason") or "?")
        method = {
            "pnp4_horizontal_weighted": "pnp4+h",
            "horizontal_width": "width",
            "low_box_conf": "low_box",
        }.get(method, method)
        state = "pub" if stable else "wait"
        return "%s id=%s %d/%d %s %.2f %s" % (
            class_name,
            item.get("tag_id", "?"),
            count,
            self.min_stable_frames,
            state,
            float(item.get("score", 0.0)),
            method,
        )

    @staticmethod
    def draw_label(image, text, anchor, color):
        x, y = anchor
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.45
        thickness = 1
        (w, h), baseline = cv2.getTextSize(text, font, scale, thickness)
        x = min(max(0, int(x)), max(0, image.shape[1] - w - 8))
        y = min(max(h + 4, int(y)), max(h + 4, image.shape[0] - 4))
        cv2.rectangle(
            image,
            (x - 3, y - h - 4),
            (x + w + 3, y + baseline + 3),
            (0, 0, 0),
            -1,
        )
        cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    def solve_candidate(self, tag_id, cls_id, box_score, points, conf, camera_matrix, dist_coeffs):
        valid = np.asarray(conf) >= self.min_keypoint_conf
        ordered_valid = all(
            idx < len(points) and idx < len(valid) and valid[idx]
            for idx in self.keypoint_order
        )
        horizontal = self.estimate_horizontal_pose(points, conf, camera_matrix)
        if ordered_valid:
            image_points = np.array(
                [points[idx] for idx in self.keypoint_order], dtype=np.float32
            )
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if ok:
                rot, _jac = cv2.Rodrigues(rvec)
                quat = quat_from_rotmat(rot)
                translation = tuple(float(v) for v in tvec.reshape(3))
                method = "pnp4"
                if horizontal is not None and self.pnp_horizontal_translation_weight > 0.0:
                    weight = self.pnp_horizontal_translation_weight
                    h_t = horizontal["t"]
                    translation = (
                        (1.0 - weight) * translation[0] + weight * h_t[0],
                        translation[1],
                        (1.0 - weight) * translation[2] + weight * h_t[2],
                    )
                    method = "pnp4_horizontal_weighted"
                score = box_score * float(np.mean([conf[idx] for idx in self.keypoint_order]))
                return {
                    "tag_id": int(tag_id),
                    "class_id": int(cls_id),
                    "score": score,
                    "t": translation,
                    "q": quat,
                    "method": method,
                    "linear_variance": self.pnp_linear_variance,
                    "horizontal_pairs": horizontal["pairs"] if horizontal else 0,
                }

        if not self.allow_horizontal_fallback:
            return None
        return self.solve_horizontal_fallback(tag_id, cls_id, box_score, horizontal)

    def estimate_horizontal_pose(self, points, conf, camera_matrix):
        estimates = []
        for left_idx, right_idx in self.horizontal_pairs:
            if left_idx >= len(points) or right_idx >= len(points):
                continue
            if conf[left_idx] < self.min_keypoint_conf or conf[right_idx] < self.min_keypoint_conf:
                continue
            p0 = np.asarray(points[left_idx], dtype=float)
            p1 = np.asarray(points[right_idx], dtype=float)
            pixel_width = float(np.linalg.norm(p1 - p0))
            if pixel_width <= 1.0:
                continue
            estimates.append({
                "weight": float(0.5 * (conf[left_idx] + conf[right_idx])),
                "center": 0.5 * (p0 + p1),
                "pixel_width": pixel_width,
            })
        if not estimates:
            return None

        total = sum(item["weight"] for item in estimates)
        if total <= 1e-9:
            return None
        center = sum(item["weight"] * item["center"] for item in estimates) / total
        pixel_width = sum(item["weight"] * item["pixel_width"] for item in estimates) / total

        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])
        depth = fx * self.tag_size_m / pixel_width
        u, v = center
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        return {
            "score": max(item["weight"] for item in estimates),
            "t": (float(x), float(y), float(depth)),
            "pixel_width": float(pixel_width),
            "pairs": len(estimates),
        }

    def solve_horizontal_fallback(self, tag_id, cls_id, box_score, horizontal):
        if horizontal is None:
            return None
        return {
            "tag_id": int(tag_id),
            "class_id": int(cls_id),
            "score": box_score * horizontal["score"],
            "t": horizontal["t"],
            "q": (0.0, 0.0, 0.0, 1.0),
            "method": "horizontal_width",
            "linear_variance": self.fallback_linear_variance,
            "horizontal_pairs": horizontal["pairs"],
        }

    def update_tracks(self, candidates, image_msg, camera_info):
        detections = []
        status = {
            "processed_frame": self.processed_frame,
            "raw_candidates": len(candidates),
            "published": 0,
            "tracks": [],
        }
        seen = set()
        for candidate in candidates:
            tag_id = candidate["tag_id"]
            seen.add(tag_id)
            state = self.track_state.get(tag_id)
            if state is None or state["last_frame"] != self.processed_frame - 1:
                state = {
                    "count": 1,
                    "ema_t": candidate["t"],
                    "ema_q": candidate["q"],
                    "last_frame": self.processed_frame,
                }
            else:
                state["count"] += 1
                state["last_frame"] = self.processed_frame
                state["ema_t"] = tuple(
                    (1.0 - self.ema_alpha) * state["ema_t"][i]
                    + self.ema_alpha * candidate["t"][i]
                    for i in range(3)
                )
                state["ema_q"] = slerp_quat(
                    state["ema_q"], candidate["q"], self.ema_alpha
                )
            self.track_state[tag_id] = state

            track_status = {
                "tag_id": tag_id,
                "class_id": candidate["class_id"],
                "count": state["count"],
                "method": candidate["method"],
                "horizontal_pairs": candidate.get("horizontal_pairs", 0),
                "score": round(candidate["score"], 4),
            }
            if "class_name" in candidate:
                track_status["class_name"] = candidate["class_name"]
            if candidate.get("global_store"):
                store = candidate["global_store"]
                track_status["global_map_id"] = store["id"]
                track_status["global_category"] = store["category"]
                track_status["global_xy"] = [
                    round(store["x"], 4),
                    round(store["y"], 4),
                ]
            status["tracks"].append(track_status)
            if state["count"] < self.min_stable_frames:
                continue

            detections.append(self.make_detection(
                tag_id, state["ema_t"], state["ema_q"], candidate,
                image_msg, camera_info,
            ))

        for tag_id in list(self.track_state):
            if tag_id not in seen and self.track_state[tag_id]["last_frame"] < self.processed_frame:
                self.track_state.pop(tag_id, None)

        status["published"] = len(detections)
        return detections, status

    def make_detection(self, tag_id, translation, quat, candidate, image_msg, camera_info):
        det = AprilTagDetection()
        det.id = [int(tag_id)]
        det.size = [float(self.tag_size_m)]
        det.pose.header.stamp = image_msg.header.stamp
        det.pose.header.frame_id = camera_info.header.frame_id or image_msg.header.frame_id
        det.pose.pose.pose.position.x = translation[0]
        det.pose.pose.pose.position.y = translation[1]
        det.pose.pose.pose.position.z = translation[2]
        det.pose.pose.pose.orientation.x = quat[0]
        det.pose.pose.pose.orientation.y = quat[1]
        det.pose.pose.pose.orientation.z = quat[2]
        det.pose.pose.pose.orientation.w = quat[3]
        det.pose.pose.covariance = make_covariance(
            candidate["linear_variance"], self.angular_variance
        )
        return det


if __name__ == "__main__":
    YoloPoseTagDetector()
    rospy.spin()
