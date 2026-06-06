#!/usr/bin/env python3
"""
go_to_goal_avoid.py — go to (x, y[, yaw]) in rtabmap's map frame while avoiding
walls, using the FRONT depth camera (no sonar on this robot).

It is go_to_goal.py + a reactive potential field:
  * attractive vector  -> toward the goal (bearing in the robot frame)
  * repulsive vector   -> away from near obstacles seen in the depth image,
                          summed over a few forward sectors (the idea borrowed
                          from the DWA node's computeEscapeDir escape vector)
The sum gives a desired heading. To avoid sideways blind spots (the camera only
sees the front ~87 deg FOV), the base does NOT strafe: it either rotates in
place (yaw only) to face that heading, or drives straight forward (forward only)
-- so the camera always faces the travel direction. Near a wall it slows; too
close it turns away / SAFE-STOPS.

Usage:
    rosrun slam go_to_goal_avoid.py <x> <y> [yaw_deg]

Needs the perception stack up (rtabmap TF map->camera_link + depth stream):
    slam up

SAFETY: autonomous. Clear the space, hand on Ctrl-C. Sideways/rear are BLIND
(forward camera only) -- don't expect it to dodge a wall it can't see.
"""

import math
import signal
import struct
import sys

import numpy as np
import genpy
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo


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


# --- frames / pose ---
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"      # rtabmap tracks base_link (base_link->camera_link static TF)
HEADING_OFFSET_DEG = 0.0       # base_link +x vs chassis forward (mount yaw=0 -> 0)

# --- depth obstacle sensing ---
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"   # 16UC1, mm
CINFO_TOPIC = "/camera/color/camera_info"
N_SECTORS = 7                  # split the FOV into this many forward sectors
BAND_ABOVE = 20                # depth rows used: [cy-BAND_ABOVE, cy+BAND_BELOW]
BAND_BELOW = 10                # (tight band near the optical center -> walls, little floor)
INFLUENCE = 1.00               # m, start being pushed by obstacles within this
HARD_STOP = 0.35               # m, nearest-front distance that forces a full stop
SLOW_RANGE = 0.70              # m, below this clearance scales the speed down
MIN_VALID = 0.15               # m, ignore depth nearer than this (noise)
DEPTH_STALE_S = 1.0

# --- control / gains ---
RATE_HZ = 10.0
V_MAX = 60.0
V_MIN = 20.0
KP_LIN = 90.0
W_ATT = 1.0                    # attractive weight (unit goal vector)
REP_GAIN = 1.6                 # per-sector repulsive gain (tune on hardware)
POS_TOL = 0.12
YAW_TOL = math.radians(8)
YAW_KP = 0.8
YAW_RATE_MAX = 0.35
YAW_MIN = 0.12                  # min yaw magnitude to overcome rotation deadband
ALIGN_ENTER = math.radians(12)  # heading err within this -> start driving forward
ALIGN_EXIT = math.radians(30)   # heading err beyond this -> stop & rotate (hysteresis)
TIMEOUT_S = 90.0

_depth = {"img": None, "stamp": None}
_intr = {"fx": None, "cx": None, "cy": None}


def depth_cb(msg):
    if msg.encoding not in ("16UC1", "mono16"):
        return
    arr = np.frombuffer(msg.data, dtype=np.uint16)
    cols = msg.step // 2 if msg.step else msg.width
    arr = arr.reshape(msg.height, cols)[:, :msg.width]
    _depth["img"] = arr
    _depth["stamp"] = rospy.get_time()


def cinfo_cb(msg):
    _intr["fx"] = msg.K[0]
    _intr["cx"] = msg.K[2]
    _intr["cy"] = msg.K[5]


def _raise_kbi():
    raise KeyboardInterrupt


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def obstacle_field():
    """Return (rep_fwd, rep_left, min_front_dist) in the robot frame from the
    latest depth frame, or (0,0,inf) if no usable depth."""
    img = _depth["img"]
    fx, cx, cy = _intr["fx"], _intr["cx"], _intr["cy"]
    if img is None or fx is None:
        return 0.0, 0.0, float("inf")
    if _depth["stamp"] is None or (rospy.get_time() - _depth["stamp"]) > DEPTH_STALE_S:
        return 0.0, 0.0, None      # None => stale/unknown -> caller stops

    h, w = img.shape
    v0 = max(0, int(cy) - BAND_ABOVE)
    v1 = min(h, int(cy) + BAND_BELOW)
    band = img[v0:v1, :].astype(np.float32) / 1000.0   # meters; 0 = invalid
    band[band < MIN_VALID] = np.nan                     # drop invalid/too-near

    # nearest valid range per column, then per sector
    with np.errstate(all="ignore"):
        col_range = np.nanmin(band, axis=0)             # (w,)
    rep_fwd = rep_left = 0.0
    min_front = float("inf")
    edges = np.linspace(0, w, N_SECTORS + 1).astype(int)
    for s in range(N_SECTORS):
        seg = col_range[edges[s]:edges[s + 1]]
        if np.all(np.isnan(seg)):
            continue
        rng = float(np.nanmin(seg))
        uc = 0.5 * (edges[s] + edges[s + 1])            # sector center column
        # robot-frame direction of this sector (fwd, left)
        x_right = (uc - cx) / fx * rng
        fwd, left = rng, -x_right
        d = math.hypot(fwd, left)
        if d < 1e-3:
            continue
        if abs(uc - cx) < 0.30 * w:                     # central sectors gate the stop
            min_front = min(min_front, d)
        if d < INFLUENCE:
            wgt = REP_GAIN * (INFLUENCE - d) / INFLUENCE
            rep_fwd += -fwd / d * wgt                    # push opposite the obstacle
            rep_left += -left / d * wgt
    return rep_fwd, rep_left, min_front


def main():
    if len(sys.argv) < 3:
        print("usage: go_to_goal_avoid.py <x> <y> [yaw_deg]")
        sys.exit(1)
    try:
        gx, gy = float(sys.argv[1]), float(sys.argv[2])
    except ValueError:
        print("error: x and y must be numbers")
        sys.exit(1)
    goal_yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else None

    rospy.init_node("go_to_goal_avoid", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/chassis_control/set_velocity", SetVelocity,
                          queue_size=1, latch=False)
    # SIGTERM (kill / `timeout`) -> raise KeyboardInterrupt so `finally` still
    # publishes the STOP burst. Without this a kill leaves the base rolling.
    signal.signal(signal.SIGTERM, lambda *_: _raise_kbi())
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_cb, queue_size=1)
    rospy.Subscriber(CINFO_TOPIC, CameraInfo, cinfo_cb, queue_size=1)
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)
    rospy.sleep(0.8)

    rate = rospy.Rate(RATE_HZ)
    start = rospy.get_time()
    miss = 0
    phase = "drive"
    aligned = False
    print("goal=(%.2f, %.2f)%s  avoid: front depth, influence=%.2fm stop=%.2fm  Ctrl-C aborts"
          % (gx, gy, "" if goal_yaw is None else " yaw=%.0f" % math.degrees(goal_yaw),
             INFLUENCE, HARD_STOP))

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
            except Exception:
                miss += 1
                if miss >= 15:
                    print("\nno TF %s->%s -> stopping (is rtabmap up?)" % (MAP_FRAME, ROBOT_FRAME))
                    break
                pub.publish(SetVelocity())
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

                rep_fwd, rep_left, min_front = obstacle_field()
                if min_front is None:
                    pub.publish(SetVelocity())
                    sys.stdout.write("\r depth stale -> hold        "); sys.stdout.flush()
                    rate.sleep(); continue

		# [ADD THIS BLOCK] 
                # Symmetry breaking: If an obstacle is dead ahead, force a turn.
                if rep_fwd < -0.3 and abs(rep_left) < 0.15:
                    rep_left = 0.5  # Bias to always turn left when stuck head-on                
		# desired heading = goal-attractive + obstacle-repulsive, in the
                # chassis frame (camera yaw corrected by HEADING_OFFSET_DEG).
                chassis_yaw = ryaw - math.radians(HEADING_OFFSET_DEG)
                bearing = norm_angle(math.atan2(dy, dx) - chassis_yaw)
                tot_fwd = W_ATT * math.cos(bearing) + rep_fwd
                tot_left = W_ATT * math.sin(bearing) + rep_left
                if math.hypot(tot_fwd, tot_left) < 1e-3:
                    tot_fwd, tot_left = math.cos(bearing), math.sin(bearing)
                head_err = math.atan2(tot_left, tot_fwd)     # CCW from forward

                # NO strafing: rotate in place (yaw only) to face head_err, then
                # drive straight forward (forward only). Camera always faces the
                # travel direction -> no sideways blind spot. Hysteresis between
                # the two states avoids chattering.
                if aligned and abs(head_err) > ALIGN_EXIT:
                    aligned = False
                elif (not aligned) and abs(head_err) <= ALIGN_ENTER:
                    aligned = True
                if aligned and min_front < HARD_STOP:
                    aligned = False                          # wall ahead -> turn away

                if not aligned:
                    wv = max(-YAW_RATE_MAX, min(YAW_RATE_MAX, YAW_KP * head_err))
                    if 0.0 < abs(wv) < YAW_MIN:
                        wv = math.copysign(YAW_MIN, wv)
                    pub.publish(SetVelocity(velocity=0.0, direction=90.0, angular=wv))
                    sys.stdout.write("\r turn:  head_err=%+.0fdeg  w=%+.2f  front=%.2f   "
                                     % (math.degrees(head_err), wv, min_front)); sys.stdout.flush()
                else:
                    clearance = 1.0
                    if min_front < SLOW_RANGE:
                        clearance = max(0.0, (min_front - HARD_STOP) / (SLOW_RANGE - HARD_STOP))
                    vel = max(V_MIN, min(V_MAX, KP_LIN * dist)) * clearance
                    pub.publish(SetVelocity(velocity=vel, direction=90.0, angular=0.0))
                    sys.stdout.write("\r fwd:   dist=%.2f  front=%.2f  v=%.0f  head_err=%+.0fdeg   "
                                     % (dist, min_front, vel, math.degrees(head_err))); sys.stdout.flush()

            elif phase == "rotate":
                yaw_err = norm_angle(goal_yaw - ryaw)
                if abs(yaw_err) <= YAW_TOL:
                    pub.publish(SetVelocity())
                    print("\nfinal yaw reached (err=%.1fdeg)" % math.degrees(yaw_err))
                    break
                wv = max(-YAW_RATE_MAX, min(YAW_RATE_MAX, YAW_KP * yaw_err))
                pub.publish(SetVelocity(velocity=0.0, direction=90.0, angular=wv))
                sys.stdout.write("\r rotate: yaw_err=%+.0fdeg     " % math.degrees(yaw_err))
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
