#!/usr/bin/env python3
"""
go_to_goal.py — drive the ArmPi mecanum base to a goal (x, y[, yaw]) using
rtabmap pose feedback. Holonomic: it drives STRAIGHT toward the goal (mecanum
strafe+drive) instead of rotate-then-go.

Usage:
    rosrun slam go_to_goal.py <x> <y> [yaw_deg]
    # rosrun slam go_to_goal.py 1.0 0.0          # 1 m ahead of rtabmap origin
    # rosrun slam go_to_goal.py 1.0 0.5 90       # + face +90 deg at the end

Frame: the goal is in rtabmap's MAP frame (origin = where rtabmap started this
run). This is step 1 of "go to store" — store NAMES in the room/global frame
need AprilTag global localization wired (step 2), not done here.

Requires the perception stack up (rtabmap publishing TF map->camera_link):
    slam up        # camera + rtabmap (+ apriltag)

How it avoids the stock chassis ramp/thread bug: it holds a near-constant
velocity magnitude and only varies DIRECTION (which the Pi node does NOT ramp)
and uses angular=0 while translating — so it barely triggers the ramp/race.

SAFETY: drives autonomously. Clear the space, keep a hand on Ctrl-C (aborts and
stops). A max-runtime timeout also stops it.
"""

import math
import signal
import struct
import sys

import genpy
import rospy
import tf2_ros


def _raise_kbi():
    raise KeyboardInterrupt


class SetVelocity(genpy.Message):
    _md5sum = "c6ffb1426a0612ef45289abb145aeb72"
    _type = "chassis_control/SetVelocity"
    _has_header = False
    _full_text = "float64 velocity\nfloat64 direction\nfloat64 angular"
    __slots__ = ["velocity", "direction", "angular"]
    _slot_types = ["float64", "float64", "float64"]
    _struct_3d = struct.Struct("<3d")

    def __init__(self, velocity=0.0, direction=0.0, angular=0.0):
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


# --- tunables ---
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"     # rtabmap tracks base_link (base_link->camera_link static TF)
HEADING_OFFSET_DEG = 0.0      # camera_link +x vs chassis forward; set if camera is yawed

RATE_HZ = 10.0
V_MAX = 60.0                  # Hiwonder velocity scale (drive_straight uses 40)
V_MIN = 20.0                  # below this the motors won't overcome deadband
KP_LIN = 90.0                 # velocity = KP_LIN * distance(m), clamped
POS_TOL = 0.12               # m, "arrived" radius
YAW_TOL = math.radians(8)     # rad, final-heading tolerance (only if yaw given)
YAW_KP = 0.8                  # angular gain for the optional final rotate
YAW_RATE_MAX = 0.35           # rad/s-ish cap for the final rotate
TIMEOUT_S = 60.0             # hard stop after this long


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    # z-yaw from a quaternion (x,y,z,w)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 3:
        print("usage: go_to_goal.py <x> <y> [yaw_deg]")
        sys.exit(1)
    try:
        gx, gy = float(sys.argv[1]), float(sys.argv[2])
    except ValueError:
        print("error: x and y must be numbers")
        sys.exit(1)
    goal_yaw = None
    if len(sys.argv) > 3:
        try:
            goal_yaw = math.radians(float(sys.argv[3]))
        except ValueError:
            print("error: yaw_deg must be a number")
            sys.exit(1)

    rospy.init_node("go_to_goal", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/chassis_control/set_velocity", SetVelocity,
                          queue_size=1, latch=False)
    signal.signal(signal.SIGTERM, lambda *_: _raise_kbi())   # kill/timeout -> stop via finally
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)
    rospy.sleep(0.6)   # let TF + the publisher connection come up

    rate = rospy.Rate(RATE_HZ)
    start = rospy.get_time()
    miss = 0
    phase = "drive"   # "drive" -> optional "rotate" -> done
    print("goal=(%.2f, %.2f)%s frame=%s  Ctrl-C aborts (stops)"
          % (gx, gy, "" if goal_yaw is None else " yaw=%.0fdeg" % math.degrees(goal_yaw), MAP_FRAME))

    def stop():
        for _ in range(5):
            pub.publish(SetVelocity())
            rospy.sleep(0.02)

    try:
        while not rospy.is_shutdown():
            if rospy.get_time() - start > TIMEOUT_S:
                print("\ntimeout -> stopping")
                break
            try:
                tf = tf_buf.lookup_transform(MAP_FRAME, ROBOT_FRAME,
                                             rospy.Time(0), rospy.Duration(0.2))
            except Exception as e:
                miss += 1
                if miss >= 15:
                    print("\nno TF %s->%s for 3s -> stopping (is rtabmap up?)" % (MAP_FRAME, ROBOT_FRAME))
                    break
                pub.publish(SetVelocity())   # hold stopped while pose is unknown
                rate.sleep()
                continue
            miss = 0

            t = tf.transform.translation
            rx, ry = t.x, t.y
            ryaw = yaw_from_quat(tf.transform.rotation)
            dx, dy = gx - rx, gy - ry
            dist = math.hypot(dx, dy)

            if phase == "drive":
                if dist <= POS_TOL:
                    phase = "rotate" if goal_yaw is not None else "done"
                    pub.publish(SetVelocity())
                    print("\narrived at (%.2f, %.2f), dist=%.3f" % (rx, ry, dist))
                    continue
                # bearing to goal relative to robot heading (CCW +)
                bearing = norm_angle(math.atan2(dy, dx) - ryaw)
                direction = (90.0 + math.degrees(bearing) + HEADING_OFFSET_DEG) % 360.0
                vel = max(V_MIN, min(V_MAX, KP_LIN * dist))
                pub.publish(SetVelocity(velocity=vel, direction=direction, angular=0.0))
                sys.stdout.write("\r drive: dist=%.2fm  bearing=%+.0fdeg  v=%.0f      " %
                                 (dist, math.degrees(bearing), vel))
                sys.stdout.flush()

            elif phase == "rotate":
                yaw_err = norm_angle(goal_yaw - ryaw)
                if abs(yaw_err) <= YAW_TOL:
                    pub.publish(SetVelocity())
                    print("\nfinal yaw reached (err=%.1fdeg)" % math.degrees(yaw_err))
                    break
                w = max(-YAW_RATE_MAX, min(YAW_RATE_MAX, YAW_KP * yaw_err))
                pub.publish(SetVelocity(velocity=0.0, direction=90.0, angular=w))
                sys.stdout.write("\r rotate: yaw_err=%+.0fdeg  w=%+.2f      " %
                                 (math.degrees(yaw_err), w))
                sys.stdout.flush()
            else:
                break

            rate.sleep()
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        stop()
        print("stopped after %.1fs" % (rospy.get_time() - start))


if __name__ == "__main__":
    main()
