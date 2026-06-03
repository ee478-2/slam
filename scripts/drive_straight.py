#!/usr/bin/env python3
"""
drive_straight.py — drive the ArmPi mecanum base straight FORWARD for T seconds.

Usage:
    rosrun slam drive_straight.py <seconds> [velocity]
    # rosrun slam drive_straight.py 3        # 3 s at default speed
    # rosrun slam drive_straight.py 3 80     # 3 s at velocity 80 (faster)

Publishes chassis_control/SetVelocity (direction=90 = forward, angular=0) at
10 Hz for the duration, then a 5x zero STOP burst. The SetVelocity message is
defined inline (same trick as teleop_keyboard.py) so this needs NEITHER the
chassis_control package NOR teleop_twist_keyboard on this machine.

Notes:
  * Put the robot on a clear, straight path first.
  * Ctrl-C aborts and still stops the base.
  * Constant-velocity drive only changes speed twice (start 0->V, end V->0), so
    it barely exercises the stock chassis node's ramp/thread bug -- it is safe to
    use a higher `velocity` here than in interactive teleop. Wheel speed is a
    signed byte; overflow is only near velocity~490, so 0..~150 is plenty of
    headroom.

Run with the robot's roscore as master:
    export ROS_MASTER_URI=http://192.168.0.200:11311
    export ROS_IP=<this machine's IP>
"""

import signal
import struct
import sys

import genpy
import rospy


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


FORWARD = 90.0      # direction in degrees: 90 = forward
RATE_HZ = 10.0
DEFAULT_VELOCITY = 40.0


def main():
    if len(sys.argv) < 2:
        print("usage: drive_straight.py <seconds> [velocity]")
        sys.exit(1)
    try:
        duration = float(sys.argv[1])
    except ValueError:
        print("error: <seconds> must be a number")
        sys.exit(1)
    if duration <= 0:
        print("error: <seconds> must be > 0")
        sys.exit(1)
    velocity = DEFAULT_VELOCITY
    if len(sys.argv) > 2:
        try:
            velocity = float(sys.argv[2])
        except ValueError:
            print("error: [velocity] must be a number")
            sys.exit(1)

    rospy.init_node("drive_straight", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/chassis_control/set_velocity", SetVelocity,
                          queue_size=1, latch=False)
    signal.signal(signal.SIGTERM, lambda *_: _raise_kbi())   # kill/timeout -> stop via finally
    rospy.sleep(0.5)   # let the publisher connect before the first command

    go = SetVelocity(velocity=velocity, direction=FORWARD, angular=0.0)
    rate = rospy.Rate(RATE_HZ)
    print("driving FORWARD: velocity=%.1f for %.1fs  (Ctrl-C aborts, still stops)"
          % (velocity, duration))
    start = rospy.get_time()
    try:
        while not rospy.is_shutdown() and (rospy.get_time() - start) < duration:
            pub.publish(go)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        # always leave the base stopped (5x zero, 20 ms apart)
        try:
            for _ in range(5):
                pub.publish(SetVelocity())
                rospy.sleep(0.02)
        except Exception:
            pass
        print("\nstopped after %.2fs" % (rospy.get_time() - start))


if __name__ == "__main__":
    main()
