#!/usr/bin/env python3
"""
Publish a MarkerArray representing the robot's TRUE pose (subscribed from
/odom — which in this Gazebo sim tracks ground truth to ~1mm). RViz can
visualize this with a MarkerArray display so you see "robot at GT" alongside
the RobotModel rendered at RTAB's estimated pose.

Topics:
  in:  /odom         (nav_msgs/Odometry — set ~input_topic to override)
  out: /true_robot/markers  (visualization_msgs/MarkerArray)

Markers:
  id=0  CUBE  semi-transparent green body (0.44 x 0.36 x 0.15 m, footprint+height)
  id=1  ARROW bright yellow heading indicator (0.5 m forward)
"""
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def make_markers(odom_msg):
    arr = MarkerArray()

    body = Marker()
    body.header = odom_msg.header
    body.ns = "true_robot"
    body.id = 0
    body.type = Marker.CUBE
    body.action = Marker.ADD
    body.pose = odom_msg.pose.pose
    # Footprint ≈ 0.44 x 0.36 (slightly larger than DWA footprint), height 0.15.
    body.scale.x = 0.44
    body.scale.y = 0.36
    body.scale.z = 0.15
    body.color.r = 0.0
    body.color.g = 1.0
    body.color.b = 0.0
    body.color.a = 0.4  # translucent so RobotModel underneath stays visible
    body.lifetime = rospy.Duration(0)  # persist until next update
    arr.markers.append(body)

    arrow = Marker()
    arrow.header = odom_msg.header
    arrow.ns = "true_robot"
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose = odom_msg.pose.pose
    arrow.scale.x = 0.5    # arrow length
    arrow.scale.y = 0.06   # shaft thickness
    arrow.scale.z = 0.06   # head thickness
    arrow.color.r = 1.0
    arrow.color.g = 1.0
    arrow.color.b = 0.0
    arrow.color.a = 1.0
    arrow.lifetime = rospy.Duration(0)
    arr.markers.append(arrow)

    return arr


def main():
    rospy.init_node("true_pose_marker", anonymous=False)
    in_topic = rospy.get_param("~input_topic", "/odom")
    out_topic = rospy.get_param("~output_topic", "/true_robot/markers")
    pub = rospy.Publisher(out_topic, MarkerArray, queue_size=1)

    def cb(m):
        pub.publish(make_markers(m))

    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)
    rospy.loginfo("true_pose_marker: %s -> %s", in_topic, out_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
