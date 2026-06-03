#!/usr/bin/env python3
"""
teleop_keyboard.py — keyboard teleop for the ArmPi Pro mecanum base.

Drives /chassis_control/set_velocity (chassis_control/SetVelocity) directly,
so it needs NEITHER the chassis_control message package NOR
teleop_twist_keyboard on this machine. The SetVelocity message is defined
inline (same trick as manipulation_control/grasp_controller.py).

The base is a mecanum/omni drive. SetVelocity fields:
  velocity   : linear speed magnitude (Hiwonder scale; ~0..30 sane)
  direction  : heading in DEGREES — 90 = forward, 270 = back,
               180 = strafe-left, 0 = strafe-right
  angular    : yaw rate (rad/s-ish), + = turn left, - = turn right

SAFETY:
  * Hold a key to move; release and it AUTO-STOPS within ~0.4 s (watchdog).
    Key auto-repeat keeps it moving while held.
  * SPACE or 's' = immediate stop. 'q' / Ctrl-C = stop and quit.
  * A STOP is published on exit no matter how it exits.

Run it in a REAL terminal (needs a live keyboard) with the robot's roscore:
  export ROS_MASTER_URI=http://192.168.0.200:11311
  export ROS_IP=<this machine's IP>
  rosrun slam teleop_keyboard.py        # or: python3 teleop_keyboard.py
"""

import struct
import sys
import select
import termios
import tty

import genpy
import rospy


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


# key -> (direction_deg, uses_angular_sign). velocity/angular magnitudes are
# the tunable LIN/ANG below.
MOVE_KEYS = {
    "w": ("lin", 90.0, 0.0),    # forward
    "s": ("lin", 270.0, 0.0),   # backward
    "a": ("lin", 180.0, 0.0),   # strafe left
    "d": ("lin", 0.0, 0.0),     # strafe right
    "j": ("ang", 0.0, +1.0),    # rotate left
    "l": ("ang", 0.0, -1.0),    # rotate right
}

HELP = """
========== ArmPi keyboard teleop ==========
  w/s : forward / back        a/d : strafe left / right
  j/l : rotate left / right
  SPACE or x : STOP           r : re-arm (if some wheels stop)
  z / c : slower / faster (linear)
  q or Ctrl-C : quit (auto-stops)
  (hold a key to keep moving; release = auto-stop ~0.4s)
  linear=%.1f  angular=%.2f
===========================================
"""

WATCHDOG = 0.4   # s without a movement key -> auto stop
RATE_HZ = 20.0
LIN_MAX = 75.0    # max linear speed (2.5x the old 30 "sane" ceiling)
SPEED_STEP = 5.0  # z/c linear decrement / increment


def get_keys(timeout):
    """Drain ALL pending keystrokes this tick and return them in order.

    Reading a single byte per loop (the old behaviour) let key auto-repeat
    pile up in stdin faster than we drained it, so the robot kept executing a
    backlog of stale keys long after release -- the "laggy / queued" feel.
    Draining the whole buffer every tick keeps input real-time; the caller
    acts on only the most recent movement key.
    """
    keys = []
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    while r:
        keys.append(sys.stdin.read(1))
        r, _, _ = select.select([sys.stdin], [], [], 0)
    return keys


def main():
    rospy.init_node("teleop_keyboard", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/chassis_control/set_velocity", SetVelocity,
                          queue_size=1, latch=False)

    lin = 15.0    # conservative starting linear speed
    ang = 0.3     # starting yaw rate

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    sys.stdout.write(HELP % (lin, ang))
    sys.stdout.flush()

    last_move = 0.0
    cur = SetVelocity()
    rate = rospy.Rate(RATE_HZ)
    try:
        while not rospy.is_shutdown():
            keys = get_keys(1.0 / RATE_HZ)
            now = rospy.get_time()

            quit_loop = False
            rearm = False
            move_key = None     # act on only the most recent movement intent
            for k in keys:
                if k in ("q", "\x03"):       # q or Ctrl-C
                    quit_loop = True
                elif k in (" ", "x"):        # explicit stop
                    cur = SetVelocity()
                    move_key = None
                elif k == "r":               # re-arm latched/stalled wheels
                    rearm = True
                    cur = SetVelocity()
                    move_key = None
                elif k == "z":
                    lin = max(0.0, lin - SPEED_STEP)
                    sys.stdout.write("\r linear=%.1f        " % lin); sys.stdout.flush()
                elif k == "c":
                    lin = min(LIN_MAX, lin + SPEED_STEP)
                    sys.stdout.write("\r linear=%.1f        " % lin); sys.stdout.flush()
                elif k in MOVE_KEYS:
                    move_key = k

            if quit_loop:
                break

            if rearm:
                # Some wheels can latch off after sustained driving (per-motor
                # over-current / stall protection on the Pi's motor board). A
                # SINGLE zero may not clear it -- a sustained zero burst does,
                # which is exactly what quitting + restarting did. Hold zero
                # ~0.25 s here, then normal key control resumes -- no quit.
                for _ in range(12):
                    pub.publish(SetVelocity())
                    rospy.sleep(0.02)
                last_move = 0.0
                sys.stdout.write("\r[re-armed] hold a key to drive          ")
                sys.stdout.flush()
                continue

            if move_key is not None:
                kind, direction, asign = MOVE_KEYS[move_key]
                if kind == "lin":
                    cur = SetVelocity(velocity=lin, direction=direction, angular=0.0)
                else:
                    cur = SetVelocity(velocity=0.0, direction=90.0, angular=ang * asign)
                last_move = now

            # watchdog: no movement key recently -> stop
            if cur.velocity != 0.0 or cur.angular != 0.0:
                if now - last_move > WATCHDOG:
                    cur = SetVelocity()

            pub.publish(cur)
            rate.sleep()
    finally:
        # always leave the robot stopped + restore terminal
        try:
            for _ in range(5):
                pub.publish(SetVelocity())
                rospy.sleep(0.02)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        sys.stdout.write("\n[teleop] stopped.\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
