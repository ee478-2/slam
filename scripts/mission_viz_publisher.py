#!/usr/bin/env python3
"""
Publish mission-level map and status markers for RViz.

Static geometry comes from config/global_map.yaml. Live status is optional and
comes from the agent/manipulation topics if they are running:
  /robot_pose, /shopping_list, /visited_stores, /grabbed_items, /inventory,
  /signboards/detections, /agent/target_pose, /agent/target_json.
"""

import json
import math
import os

import rospy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    import rospkg
except ImportError:
    rospkg = None


STORE_INVENTORY = {
    "cafe": ["cup", "drink"],
    "burger": ["hamburger", "drink"],
    "pharmacy": ["medicine"],
    "convenience_store": ["mixed"],
}


def yaw_to_quaternion(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def parse_jsonish(value, default=None):
    if value is None:
        return default
    if not isinstance(value, str):
        return value
    text = value.strip()
    if not text:
        return default
    try:
        return json.loads(text)
    except Exception:
        pass
    try:
        parsed = yaml.safe_load(text)
        return default if parsed is None else parsed
    except Exception:
        return default


def parse_list(value, keys=None):
    keys = keys or []
    parsed = parse_jsonish(value, default=value)
    if isinstance(parsed, dict):
        for key in keys:
            if key in parsed:
                parsed = parsed[key]
                break
    if isinstance(parsed, str):
        parsed = parsed.replace(",", " ").split()
    if not isinstance(parsed, (list, tuple, set)):
        return []
    return [str(item).strip() for item in parsed if str(item).strip()]


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def pose_yaw_radians(pose):
    if "yaw_deg" in pose:
        return math.radians(float(pose.get("yaw_deg", 0.0)))
    return float(pose.get("yaw", 0.0))


def default_global_map_yaml():
    if rospkg is not None:
        try:
            return os.path.join(rospkg.RosPack().get_path("slam"), "config", "global_map.yaml")
        except Exception:
            pass
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "global_map.yaml")
    )


class MissionVizPublisher:
    def __init__(self):
        rospy.init_node("mission_viz_publisher", anonymous=False)

        self.global_map_yaml = os.path.expanduser(
            rospy.get_param("~global_map_yaml", default_global_map_yaml())
        )
        self.marker_topic = rospy.get_param("~marker_topic", "/mission/markers")
        self.marker_hz = float(rospy.get_param("~marker_hz", 1.0))
        self.show_signboard_details = bool(
            rospy.get_param("~show_signboard_details", True)
        )
        self.show_visit_targets = bool(rospy.get_param("~show_visit_targets", True))
        self.status_image_topic = rospy.get_param(
            "~status_image_topic", "/mission/status_image"
        )
        self.status_image_width = int(rospy.get_param("~status_image_width", 720))
        self.status_image_height = int(rospy.get_param("~status_image_height", 300))

        self.map_data = self.load_global_map(self.global_map_yaml)
        self.frame_id = rospy.get_param(
            "~frame_id",
            self.map_data.get("frame_id", "map"),
        )
        self.status_anchor = self.compute_status_anchor()
        self.wall_top_z = self.compute_wall_top_z()

        self.robot_pose = None
        self.target_pose = None
        self.target_decision = {}
        self.robot_state = "INIT"
        self.initial_pose_received = None
        self.current_target_type = ""
        self.current_target_id = ""
        self.shopping_list = []
        self.visited_store_ids = set()
        self.grabbed_items = []
        self.inventory_value = None
        self.visible_observations = []

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.status_image_pub = None
        if cv2 is not None and np is not None:
            self.status_image_pub = rospy.Publisher(
                self.status_image_topic,
                Image,
                queue_size=1,
                latch=True,
            )
        else:
            rospy.logwarn(
                "[mission_viz] cv2/numpy unavailable; %s will not be published",
                self.status_image_topic,
            )

        rospy.Subscriber("/robot_pose", PoseStamped, self.on_robot_pose, queue_size=5)
        rospy.Subscriber("/robot_master/state", String, self.on_robot_state, queue_size=5)
        rospy.Subscriber(
            "/robot_master/state_detail",
            String,
            self.on_robot_state_detail,
            queue_size=5,
        )
        rospy.Subscriber("/shopping_list", String, self.on_shopping_list, queue_size=5)
        rospy.Subscriber("/visited_stores", String, self.on_visited_stores, queue_size=5)
        rospy.Subscriber("/grabbed_items", String, self.on_grabbed_items, queue_size=5)
        rospy.Subscriber("/inventory", String, self.on_inventory, queue_size=5)
        rospy.Subscriber(
            "/signboards/detections",
            String,
            self.on_signboard_detections,
            queue_size=10,
        )
        rospy.Subscriber("/agent/target_pose", PoseStamped, self.on_target_pose, queue_size=5)
        rospy.Subscriber("/agent/target_json", String, self.on_target_json, queue_size=5)

        period = 1.0 / max(0.1, self.marker_hz)
        rospy.Timer(rospy.Duration.from_sec(period), self.on_timer)

        rospy.loginfo(
            "[mission_viz] publishing %d walls, %d stores, %d signboards on %s; status_image=%s",
            len(self.map_data["walls"]),
            len(self.map_data["stores"]),
            len(self.map_data["signboards"]),
            self.marker_topic,
            self.status_image_topic if self.status_image_pub else "disabled",
        )

    def load_global_map(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        stores = []
        for store in data.get("stores", []):
            x = float(store["x"])
            y = float(store["y"])
            visit_offset = store.get("visit_offset", {}) or {}
            stores.append(
                {
                    "id": str(store["id"]),
                    "category": str(store.get("category", "unknown")),
                    "x": x,
                    "y": y,
                    "visit_x": x + float(visit_offset.get("x", 0.0)),
                    "visit_y": y + float(visit_offset.get("y", 0.0)),
                    "visit_yaw": float(visit_offset.get("yaw", 0.0)),
                    "visited": bool(store.get("visited", False)),
                }
            )

        walls = []
        for wall in data.get("walls", []):
            try:
                pose = wall.get("pose", {}) or {}
                size = wall.get("size", {}) or {}
                height = float(size.get("z", 0.18))
                walls.append(
                    {
                        "id": str(wall["id"]),
                        "x": float(pose["x"]),
                        "y": float(pose["y"]),
                        "z": float(pose.get("z", 0.5 * height)),
                        "yaw": pose_yaw_radians(pose),
                        "size_x": float(size["x"]),
                        "size_y": float(size["y"]),
                        "size_z": height,
                    }
                )
            except (KeyError, TypeError, ValueError) as exc:
                rospy.logwarn("[mission_viz] skipping invalid wall: %s", exc)

        signboards = []
        for signboard_id, signboard in (data.get("signboards", {}) or {}).items():
            pose = signboard.get("pose", {}) or {}
            tags = []
            for tag in signboard.get("tags", []):
                semantic = tag.get("semantic", {}) or {}
                tags.append(
                    {
                        "id": int(tag["id"]),
                        "slot": str(tag.get("slot", "")),
                        "arrow": str(semantic.get("arrow", "")),
                        "icon": str(semantic.get("icon", "")),
                    }
                )
            signboards.append(
                {
                    "id": str(signboard_id),
                    "model": str(signboard.get("model", signboard_id)),
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "z": float(pose.get("z", 0.36)),
                    "yaw_deg": float(pose.get("yaw_deg", 0.0)),
                    "tags": tags,
                }
            )

        return {
            "frame_id": data.get("metadata", {}).get("frame_id", "map"),
            "walls": walls,
            "stores": stores,
            "signboards": signboards,
        }

    def compute_status_anchor(self):
        static_items = (
            self.map_data["walls"]
            + self.map_data["stores"]
            + self.map_data["signboards"]
        )
        xs = [item["x"] for item in static_items]
        ys = [item["y"] for item in static_items]
        if not xs or not ys:
            return (-2.8, 2.4, 1.1)
        return (min(xs) - 0.35, max(ys) + 0.35, 1.1)

    def compute_wall_top_z(self):
        if not self.map_data["walls"]:
            return 0.18
        return max(wall["z"] + 0.5 * wall["size_z"] for wall in self.map_data["walls"])

    def on_robot_pose(self, msg):
        self.robot_pose = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "yaw": quat_to_yaw(msg.pose.orientation),
        }

    def on_shopping_list(self, msg):
        self.shopping_list = parse_list(msg.data, keys=["shopping_list", "items"])

    def on_visited_stores(self, msg):
        self.visited_store_ids = set(
            parse_list(msg.data, keys=["visited_store_ids", "visited_stores", "stores"])
        )

    def on_grabbed_items(self, msg):
        self.grabbed_items = parse_list(msg.data, keys=["grabbed_items", "items"])

    def on_inventory(self, msg):
        self.inventory_value = parse_jsonish(msg.data, default=msg.data)
        grabbed = parse_list(
            self.inventory_value,
            keys=["grabbed_items", "inventory", "items"],
        )
        if grabbed:
            self.grabbed_items = grabbed

    def on_signboard_detections(self, msg):
        parsed = parse_jsonish(msg.data, default={})
        if isinstance(parsed, dict):
            self.visible_observations = [
                obs for obs in parsed.get("observations", []) if isinstance(obs, dict)
            ]
        elif isinstance(parsed, list):
            self.visible_observations = [obs for obs in parsed if isinstance(obs, dict)]

    def on_target_pose(self, msg):
        self.target_pose = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "yaw": quat_to_yaw(msg.pose.orientation),
        }

    def on_robot_state(self, msg):
        state = str(msg.data).strip()
        if state:
            self.robot_state = state

    def on_robot_state_detail(self, msg):
        parsed = parse_jsonish(msg.data, default={})
        if not isinstance(parsed, dict):
            return
        state = str(parsed.get("state", "")).strip()
        if state:
            self.robot_state = state
        if "initial_pose_received" in parsed:
            self.initial_pose_received = bool(parsed.get("initial_pose_received"))
        target_type = parsed.get("target_type")
        target_id = parsed.get("target_id")
        self.current_target_type = "" if target_type in (None, "") else str(target_type)
        self.current_target_id = "" if target_id in (None, "") else str(target_id)

    def on_target_json(self, msg):
        parsed = parse_jsonish(msg.data, default={})
        self.target_decision = parsed if isinstance(parsed, dict) else {}
        if self.target_decision and not self.target_decision.get("success", True):
            self.target_pose = None
        target = self.target_decision.get("target") or {}
        if isinstance(target, dict):
            self.current_target_type = str(target.get("type", self.current_target_type))
            self.current_target_id = str(target.get("id", self.current_target_id))

    def on_timer(self, _event):
        self.publish_markers()

    def publish_markers(self):
        status_lines = self.status_lines()
        markers = []
        markers.extend(self.wall_markers())
        markers.extend(self.store_markers())
        markers.extend(self.signboard_markers())
        markers.extend(self.visible_signboard_markers())
        markers.extend(self.robot_markers())
        markers.extend(self.target_markers())
        markers.extend(self.status_markers(status_lines))
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.publish_status_image(status_lines)

    def wall_markers(self):
        markers = []
        for idx, wall in enumerate(self.map_data["walls"]):
            markers.append(self.delete_marker("wall_labels", 9100 + idx))
            marker = self.base_marker("walls", 9000 + idx, Marker.CUBE)
            marker.pose.position.x = wall["x"]
            marker.pose.position.y = wall["y"]
            marker.pose.position.z = wall["z"]
            qz, qw = yaw_to_quaternion(wall["yaw"])
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = wall["size_x"]
            marker.scale.y = wall["size_y"]
            marker.scale.z = wall["size_z"]
            self.set_color(marker, (1.0, 1.0, 1.0, 0.86))
            markers.append(marker)
        return markers

    def store_markers(self):
        markers = []
        for idx, store in enumerate(self.map_data["stores"]):
            visited = store["visited"] or store["id"] in self.visited_store_ids
            color = self.store_color(store["category"])
            if visited:
                color = (0.42, 0.42, 0.42, 0.9)

            marker = self.base_marker("storefronts", idx, Marker.CUBE)
            marker.pose.position.x = store["x"]
            marker.pose.position.y = store["y"]
            marker.pose.position.z = self.wall_top_z + 0.11
            marker.scale.x = 0.30
            marker.scale.y = 0.30
            marker.scale.z = 0.18
            self.set_color(marker, color)
            markers.append(marker)

            if self.show_visit_targets:
                markers.append(
                    self.arrow_marker(
                        "store_visit_targets",
                        500 + idx,
                        store["visit_x"],
                        store["visit_y"],
                        store["visit_yaw"],
                        self.wall_top_z + 0.14,
                        (1.0, 1.0, 1.0, 0.85),
                        scale=(0.26, 0.06, 0.06),
                    )
                )

            label = "{}\n{}".format(
                store["id"],
                store["category"],
            )
            inventory = STORE_INVENTORY.get(store["category"], [])
            if inventory:
                label += "\ninv: {}".format("/".join(inventory))
            if visited:
                label += "\nvisited"
            markers.append(
                self.text_marker(
                    "store_labels",
                    1000 + idx,
                    store["x"],
                    store["y"],
                    self.wall_top_z + 0.45,
                    label,
                    0.105,
                    (1.0, 1.0, 1.0, 1.0),
                )
            )
        return markers

    def signboard_markers(self):
        markers = []
        for idx, signboard in enumerate(self.map_data["signboards"]):
            markers.append(self.delete_marker("signboard_labels", 3000 + idx))
            marker = self.base_marker("signboards", 2000 + idx, Marker.CUBE)
            marker.pose.position.x = signboard["x"]
            marker.pose.position.y = signboard["y"]
            marker.pose.position.z = signboard["z"]
            qz, qw = yaw_to_quaternion(math.radians(signboard["yaw_deg"]))
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.055
            marker.scale.y = 0.34
            marker.scale.z = 0.18
            self.set_color(marker, (1.0, 0.86, 0.16, 0.96))
            markers.append(marker)
        return markers

    def visible_signboard_markers(self):
        markers = []
        for idx in range(40):
            markers.append(self.delete_marker("visible_signboards", 4000 + idx))
            markers.append(self.delete_marker("visible_signboard_labels", 4500 + idx))

        for idx, obs in enumerate(self.visible_observations[:40]):
            xy = obs.get("signboard_xy")
            if not isinstance(xy, (list, tuple)) or len(xy) < 2:
                continue
            x, y = float(xy[0]), float(xy[1])
            marker = self.base_marker("visible_signboards", 4000 + idx, Marker.SPHERE)
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.70
            marker.scale.x = 0.26
            marker.scale.y = 0.26
            marker.scale.z = 0.10
            self.set_color(marker, (0.0, 0.95, 1.0, 0.86))
            markers.append(marker)

        return markers

    def robot_markers(self):
        if not self.robot_pose:
            return [
                self.delete_marker("robot_pose", 6000),
                self.delete_marker("robot_pose_label", 6001),
            ]
        return [
            self.arrow_marker(
                "robot_pose",
                6000,
                self.robot_pose["x"],
                self.robot_pose["y"],
                self.robot_pose["yaw"],
                0.16,
                (0.2, 0.55, 1.0, 1.0),
                scale=(0.42, 0.10, 0.10),
            ),
            self.text_marker(
                "robot_pose_label",
                6001,
                self.robot_pose["x"],
                self.robot_pose["y"],
                0.70,
                "robot",
                0.11,
                (0.45, 0.75, 1.0, 1.0),
            ),
        ]

    def target_markers(self):
        if not self.target_pose:
            return [
                self.delete_marker("agent_target", 7000),
                self.delete_marker("agent_target_label", 7001),
                self.delete_marker("robot_to_target", 7002),
            ]

        markers = [
            self.arrow_marker(
                "agent_target",
                7000,
                self.target_pose["x"],
                self.target_pose["y"],
                self.target_pose["yaw"],
                0.19,
                (1.0, 0.16, 0.16, 1.0),
                scale=(0.46, 0.12, 0.12),
            )
        ]

        target = self.target_decision.get("target") or {}
        target_type = target.get("type", "target")
        target_id = target.get("id", "")
        reason = str(self.target_decision.get("reason", ""))
        label = "target\n{} {}".format(target_type, target_id).strip()
        if reason:
            label += "\n{}".format(reason[:90])
        markers.append(
            self.text_marker(
                "agent_target_label",
                7001,
                self.target_pose["x"],
                self.target_pose["y"],
                0.86,
                label,
                0.11,
                (1.0, 0.4, 0.4, 1.0),
            )
        )

        if self.robot_pose:
            line = self.base_marker("robot_to_target", 7002, Marker.LINE_STRIP)
            line.scale.x = 0.035
            line.points = [
                Point(self.robot_pose["x"], self.robot_pose["y"], 0.09),
                Point(self.target_pose["x"], self.target_pose["y"], 0.09),
            ]
            self.set_color(line, (1.0, 0.22, 0.22, 0.9))
            markers.append(line)
        return markers

    def status_lines(self):
        status = ["MISSION STATUS"]
        status.append("state: {}".format(self.robot_state or "unknown"))
        if self.initial_pose_received is not None:
            status.append(
                "initial pose: {}".format(
                    "received" if self.initial_pose_received else "waiting"
                )
            )
        status.append("destination: {}".format(self.destination_text()))
        status.append(
            "needed: {}".format(", ".join(self.shopping_list) if self.shopping_list else "waiting")
        )
        status.append(
            "grabbed: {}".format(", ".join(self.grabbed_items) if self.grabbed_items else "none")
        )
        status.append(
            "visited: {}".format(
                ", ".join(sorted(self.visited_store_ids)) if self.visited_store_ids else "none"
            )
        )
        if self.inventory_value not in (None, "", []):
            status.append("inventory: {}".format(self.short_inventory_text()))
        if self.visible_observations:
            status.append("visible tags: {}".format(len(self.visible_observations)))
        return status

    def status_markers(self, status_lines):
        x, y, z = self.status_anchor
        return [
            self.text_marker(
                "mission_status",
                8000,
                x,
                y,
                z,
                "\n".join(status_lines),
                0.145,
                (0.92, 0.95, 1.0, 1.0),
            )
        ]

    def publish_status_image(self, status_lines):
        if self.status_image_pub is None:
            return
        width = max(360, int(self.status_image_width))
        height = max(190, int(self.status_image_height))
        image = np.full((height, width, 3), (24, 28, 34), dtype=np.uint8)
        cv2.rectangle(image, (0, 0), (width - 1, 48), (44, 55, 68), -1)
        cv2.rectangle(image, (0, 0), (width - 1, height - 1), (94, 112, 132), 2)

        title = status_lines[0] if status_lines else "MISSION STATUS"
        cv2.putText(
            image,
            title,
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.78,
            (245, 248, 255),
            2,
            cv2.LINE_AA,
        )

        max_chars = max(30, int((width - 36) / 9.6))
        y = 76
        for line in status_lines[1:]:
            if y > height - 18:
                break
            text = line if len(line) <= max_chars else line[: max_chars - 3] + "..."
            cv2.putText(
                image,
                text,
                (18, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.54,
                (226, 234, 244),
                1,
                cv2.LINE_AA,
            )
            y += 28

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = image.tobytes()
        self.status_image_pub.publish(msg)

    def destination_text(self):
        parts = [part for part in (self.current_target_type, self.current_target_id) if part]
        if parts:
            return " ".join(parts)
        if self.target_pose:
            return "target ({:.2f}, {:.2f})".format(
                self.target_pose["x"],
                self.target_pose["y"],
            )
        return "none"

    def short_inventory_text(self):
        if isinstance(self.inventory_value, dict):
            return json.dumps(self.inventory_value, sort_keys=True)[:90]
        if isinstance(self.inventory_value, (list, tuple)):
            return ", ".join(str(item) for item in self.inventory_value)[:90]
        return str(self.inventory_value)[:90]

    def arrow_marker(self, ns, marker_id, x, y, yaw, z, color, scale):
        marker = self.base_marker(ns, marker_id, Marker.ARROW)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        qz, qw = yaw_to_quaternion(float(yaw))
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        self.set_color(marker, color)
        return marker

    def text_marker(self, ns, marker_id, x, y, z, text, scale, color):
        marker = self.base_marker(ns, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.scale.z = float(scale)
        marker.text = str(text)
        self.set_color(marker, color)
        return marker

    def base_marker(self, ns, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0)
        return marker

    def delete_marker(self, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.action = Marker.DELETE
        return marker

    def set_color(self, marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def store_color(self, category):
        if category == "cafe":
            return (0.12, 0.52, 1.0, 0.95)
        if category == "burger":
            return (1.0, 0.36, 0.08, 0.95)
        if category == "pharmacy":
            return (0.16, 0.80, 0.30, 0.95)
        if category == "convenience_store":
            return (0.76, 0.34, 1.0, 0.95)
        return (0.75, 0.75, 0.75, 0.95)


if __name__ == "__main__":
    MissionVizPublisher()
    rospy.spin()
