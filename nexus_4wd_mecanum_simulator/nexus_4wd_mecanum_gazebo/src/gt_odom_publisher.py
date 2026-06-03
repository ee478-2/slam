#!/usr/bin/env python3
"""
Publishes true ground-truth odometry from Gazebo's /gazebo/model_states.

The default /odom from nexus_ros_force_based_move integrates body velocities
(dead-reckoning), so it accumulates error during fast rotation or acceleration
— it is NOT ground truth. This node reads the model's actual world pose from
Gazebo, normalizes it to the spawn position so it shares an origin with /odom
and /rtabmap/odom, and republishes as /ground_truth/odom.

Usage:
    rosrun nexus_4wd_mecanum_gazebo gt_odom_publisher.py \\
        _model_name:=nexus_4wd_mecanum \\
        _output_topic:=/ground_truth/odom \\
        _frame_id:=odom \\
        _child_frame_id:=base_footprint
"""
import math
import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


def quat_inv(q):
    """Inverse of a unit quaternion = conjugate. Negates (x,y,z), keeps w."""
    return [-q[0], -q[1], -q[2], q[3]]


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def main():
    rospy.init_node('gt_odom_publisher', anonymous=False)

    model_name    = rospy.get_param('~model_name', 'nexus_4wd_mecanum')
    output_topic  = rospy.get_param('~output_topic', '/ground_truth/odom')
    frame_id      = rospy.get_param('~frame_id', 'odom')
    child_frame   = rospy.get_param('~child_frame_id', 'base_footprint')

    pub = rospy.Publisher(output_topic, Odometry, queue_size=10)

    # First-pose offset: GT is published relative to where the robot first
    # appeared, so /ground_truth/odom shares the (0,0,0) origin with /odom and
    # /rtabmap/odom. Without this, GT would be in world frame and trail the
    # other two by the spawn translation.
    spawn_pos = None
    spawn_quat_inv = None

    def cb(msg):
        nonlocal spawn_pos, spawn_quat_inv
        try:
            i = msg.name.index(model_name)
        except ValueError:
            return
        p = msg.pose[i]
        if spawn_pos is None:
            spawn_pos = (p.position.x, p.position.y, p.position.z)
            q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            spawn_quat_inv = quat_inv(q)
            rospy.loginfo("gt_odom: anchored origin at spawn (x=%.3f y=%.3f z=%.3f)",
                          *spawn_pos)

        # Translation in spawn frame: rotate world delta by inverse spawn yaw.
        # Vector rotation by quat: v' = q * v_quat * q^-1, where v_quat=(x,y,z,0).
        # Here q = spawn_quat_inv (taking world → spawn frame).
        dx = p.position.x - spawn_pos[0]
        dy = p.position.y - spawn_pos[1]
        dz = p.position.z - spawn_pos[2]
        v = [dx, dy, dz, 0.0]
        v_rot = quat_mul(quat_mul(spawn_quat_inv, v), quat_inv(spawn_quat_inv))

        # Orientation in spawn frame: q_world * spawn_quat_inv (post-multiply).
        q_world = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        q_rel = quat_mul(spawn_quat_inv, q_world)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = v_rot[0]
        odom.pose.pose.position.y = v_rot[1]
        odom.pose.pose.position.z = v_rot[2]
        odom.pose.pose.orientation.x = q_rel[0]
        odom.pose.pose.orientation.y = q_rel[1]
        odom.pose.pose.orientation.z = q_rel[2]
        odom.pose.pose.orientation.w = q_rel[3]
        # twist available in msg.twist[i] but not needed for visual comparison
        if i < len(msg.twist):
            odom.twist.twist = msg.twist[i]
        pub.publish(odom)

    rospy.Subscriber('/gazebo/model_states', ModelStates, cb, queue_size=10)
    rospy.loginfo("gt_odom_publisher: subscribing /gazebo/model_states for "
                  "model='%s', publishing to %s", model_name, output_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
