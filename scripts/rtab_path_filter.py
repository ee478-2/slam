#!/usr/bin/env python3
"""
Publish an RViz-safe RTAB-Map trajectory marker.

RTAB-Map's /rtabmap/mapPath can include landmark graph vertices. RViz's Path
display connects every vertex in order, so landmark detours and relocalization
jumps show up as long straight artifact lines. This node renders the same path
as independent line segments and drops suspicious vertices/edges.
"""

import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def finite_point(point):
    return (
        math.isfinite(point.x)
        and math.isfinite(point.y)
        and math.isfinite(point.z)
    )


class RtabPathFilter:
    def __init__(self):
        rospy.init_node("rtab_path_filter", anonymous=False)

        self.path_topic = rospy.get_param("~path_topic", "/rtabmap/mapPath")
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/rtabmap/filtered_path_marker"
        )
        self.marker_ns = rospy.get_param("~marker_ns", "rtab_filtered_path")
        self.max_step_m = float(rospy.get_param("~max_step_m", 0.80))
        self.max_abs_z = float(rospy.get_param("~max_abs_z", 0.12))
        self.max_segments = int(rospy.get_param("~max_segments", 5000))
        self.line_width = float(rospy.get_param("~line_width", 0.035))
        self.z_offset = float(rospy.get_param("~z_offset", 0.025))
        self.color = (
            float(rospy.get_param("~color_r", 0.10)),
            float(rospy.get_param("~color_g", 1.00)),
            float(rospy.get_param("~color_b", 0.35)),
            float(rospy.get_param("~color_a", 1.00)),
        )

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            Marker,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=1)

        rospy.loginfo(
            "[rtab_path_filter] %s -> %s max_step=%.2fm max_abs_z=%.2fm",
            self.path_topic,
            self.marker_topic,
            self.max_step_m,
            self.max_abs_z,
        )

    def make_marker(self, header):
        marker = Marker()
        marker.header.stamp = header.stamp if header.stamp else rospy.Time.now()
        marker.header.frame_id = header.frame_id or "map"
        marker.ns = self.marker_ns
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]
        return marker

    def pose_point(self, pose_stamped):
        point = pose_stamped.pose.position
        if not finite_point(point):
            return None
        if self.max_abs_z >= 0.0 and abs(point.z) > self.max_abs_z:
            return None
        out = Point()
        out.x = point.x
        out.y = point.y
        out.z = point.z + self.z_offset
        return out

    def edge_ok(self, a, b):
        if self.max_step_m <= 0.0:
            return True
        dx = b.x - a.x
        dy = b.y - a.y
        dz = b.z - a.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.max_step_m

    def on_path(self, msg):
        marker = self.make_marker(msg.header)
        points = []
        last = None
        dropped_vertices = 0
        dropped_edges = 0

        for pose_stamped in msg.poses:
            point = self.pose_point(pose_stamped)
            if point is None:
                dropped_vertices += 1
                continue
            if last is not None:
                if self.edge_ok(last, point):
                    points.extend((last, point))
                else:
                    dropped_edges += 1
            last = point

        if self.max_segments > 0:
            points = points[-2 * self.max_segments :]

        if not points:
            marker.action = Marker.DELETE
        marker.points = points
        self.marker_pub.publish(marker)

        rospy.loginfo_throttle(
            10.0,
            "[rtab_path_filter] segments=%d dropped_vertices=%d dropped_edges=%d",
            len(points) // 2,
            dropped_vertices,
            dropped_edges,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    RtabPathFilter().spin()
