#!/usr/bin/env python3
"""
wheel_odom_node.py -- open-loop odometry from ArmPi chassis commands.

The stock Hiwonder chassis node accepts chassis_control/SetVelocity commands but
does not publish encoder odometry. This node subscribes to those commands,
converts the vendor linear speed scale into m/s with a tunable factor, and
integrates a planar odometry estimate on /wheel/odom.

This is command-integrated odometry, not encoder feedback. It is useful as a
motion prior/debug signal and should be calibrated against measured travel.
"""

import math
import struct
import threading

import genpy
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry


class SetVelocity(genpy.Message):
    _md5sum = "c6ffb1426a0612ef45289abb145aeb72"
    _type = "chassis_control/SetVelocity"
    _has_header = False
    _full_text = "float64 velocity\nfloat64 direction\nfloat64 angular"
    __slots__ = ["velocity", "direction", "angular"]
    _slot_types = ["float64", "float64", "float64"]
    _struct_3d = struct.Struct("<3d")

    def __init__(self, velocity=0.0, direction=90.0, angular=0.0):
        self.velocity = velocity
        self.direction = direction
        self.angular = angular

    def _get_types(self):
        return self._slot_types

    def serialize(self, buff):
        try:
            buff.write(self._struct_3d.pack(self.velocity, self.direction, self.angular))
        except struct.error as e:
            self._check_types(e)

    def deserialize(self, data):
        try:
            self.velocity, self.direction, self.angular = self._struct_3d.unpack(data[:24])
            return self
        except struct.error as e:
            raise genpy.DeserializationError(e)


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))


def planar_covariance(xy_var, yaw_var):
    cov = [0.0] * 36
    cov[0] = xy_var
    cov[7] = xy_var
    cov[14] = 1e6
    cov[21] = 1e6
    cov[28] = 1e6
    cov[35] = yaw_var
    return cov


def get_bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class WheelOdom:
    def __init__(self):
        rospy.init_node("wheel_odom", anonymous=False)

        self.cmd_topic = rospy.get_param("~cmd_topic", "/chassis_control/set_velocity")
        self.odom_topic = rospy.get_param("~odom_topic", "/wheel/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "wheel_odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_hz = float(rospy.get_param("~publish_hz", 30.0))
        self.cmd_timeout_s = float(rospy.get_param("~cmd_timeout_s", 0.5))
        self.linear_scale = float(rospy.get_param("~linear_scale", 0.01))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))
        self.publish_tf = get_bool_param("~publish_tf", False)
        self.initial_x = float(rospy.get_param("~initial_x", 0.0))
        self.initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.initial_yaw = float(rospy.get_param("~initial_yaw", 0.0))

        self.pose_covariance = planar_covariance(
            float(rospy.get_param("~pose_xy_variance", 0.25)),
            float(rospy.get_param("~pose_yaw_variance", 0.25)),
        )
        self.twist_covariance = planar_covariance(
            float(rospy.get_param("~twist_xy_variance", 0.5)),
            float(rospy.get_param("~twist_yaw_variance", 0.5)),
        )

        self._lock = threading.Lock()
        self._last_cmd = SetVelocity()
        self._last_cmd_time = None
        self._x = self.initial_x
        self._y = self.initial_y
        self._yaw = self.initial_yaw
        self._last_update = None

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        rospy.Subscriber(self.cmd_topic, SetVelocity, self._cmd_cb, queue_size=20)

        rospy.loginfo(
            "[wheel_odom] %s -> %s frame=%s child=%s linear_scale=%.5f angular_scale=%.3f publish_tf=%s",
            self.cmd_topic, self.odom_topic, self.odom_frame, self.base_frame,
            self.linear_scale, self.angular_scale, self.publish_tf,
        )

    def _cmd_cb(self, msg):
        with self._lock:
            self._last_cmd = msg
            self._last_cmd_time = rospy.Time.now()

    def _current_body_twist(self, now):
        with self._lock:
            cmd = self._last_cmd
            cmd_time = self._last_cmd_time

        if cmd_time is None or (now - cmd_time).to_sec() > self.cmd_timeout_s:
            return 0.0, 0.0, 0.0

        speed = cmd.velocity * self.linear_scale
        direction = math.radians(cmd.direction)

        # Hiwonder directions: 90=fwd, 270=back, 180=left, 0=right.
        vx = speed * math.sin(direction)
        vy = -speed * math.cos(direction)
        wz = cmd.angular * self.angular_scale
        return vx, vy, wz

    def _integrate(self, now, vx, vy, wz):
        if self._last_update is None:
            self._last_update = now
            return

        dt = (now - self._last_update).to_sec()
        self._last_update = now
        if dt <= 0.0 or dt > 1.0:
            return

        cy = math.cos(self._yaw)
        sy = math.sin(self._yaw)
        self._x += (cy * vx - sy * vy) * dt
        self._y += (sy * vx + cy * vy) * dt
        self._yaw = math.atan2(math.sin(self._yaw + wz * dt),
                               math.cos(self._yaw + wz * dt))

    def _publish(self, stamp, vx, vy, wz):
        quat = yaw_to_quaternion(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = self.twist_covariance
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf)

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            vx, vy, wz = self._current_body_twist(now)
            self._integrate(now, vx, vy, wz)
            self._publish(now, vx, vy, wz)
            rate.sleep()


if __name__ == "__main__":
    WheelOdom().spin()
