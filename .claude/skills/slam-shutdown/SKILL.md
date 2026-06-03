---
name: slam-shutdown
description: Cleanly tear down the real-robot perception/SLAM stack (rtabmap, rtabmap_viz, RealSense camera) using SIGINT only, so the D435 releases the USB bus instead of wedging. Use when the user says "rtab 꺼", "카메라 꺼", "stack 내려", "shut it down", or before walking away from a session.
---

# slam-shutdown — clean SIGINT teardown

Stops the perception stack the safe way. **Why this skill exists:** killing the
realsense nodelet with SIGKILL / `rosnode kill` mid-stream wedges the D435 at the USB
level (`lsusb` still lists it, but `rs-enumerate-devices` reports "No device detected";
a `USBDEVFS_RESET` ioctl does NOT recover it — only a physical replug does). SIGINT to
the actual roslaunch processes releases the device cleanly.

## Hard rules

- **SIGINT (`kill -INT`) only.** Never `kill -9`, never `rosnode kill` on the camera.
- Stop **consumers before the camera**: rtabmap_viz → rtabmap → camera last.
- `setsid` detaches the launches, so `/tmp/*.pid` may hold the stale parent PID. Prefer
  `pkill -INT -f '<cmdline>'`, or resolve the live PID with `pgrep -f`.

## Teardown

```bash
# 1) viewer (if running)
pkill -INT -f 'rtabmap_viz/rtabmap_viz' ; echo "viz INT sent"

# 2) rtabmap SLAM stack
pkill -INT -f 'roslaunch slam rtabmap_realsense' ; echo "rtabmap INT sent"
sleep 6

# 3) camera LAST — SIGINT, then give it time to release the USB device
pkill -INT -f 'roslaunch realsense2_camera' ; echo "camera INT sent"
sleep 6
```

To stop only part of the stack (e.g. user said just "rtab 꺼"), run steps 1–2 and
**leave the camera up** — streaming alone is harmless and avoids a re-enumerate later.

## Verify down + device healthy

```bash
ps -ef | grep -E 'rtabmap|rgbd_odometry|realsense' | grep -v grep || echo "(all down)"
lsusb | grep -i 8086 && echo "D435 still enumerated (good)" || echo "D435 missing"
# deeper check that it didn't wedge (driver-level):
rs-enumerate-devices 2>/dev/null | grep -i 'Name\|Serial' | head || echo "rs-enumerate not on PATH"
```

If `rs-enumerate-devices` says "No device detected" while `lsusb` still shows 8086:0b07,
the D435 is wedged — tell the user it needs a **physical USB replug**; no software reset
recovers it. (A clean SIGINT teardown should prevent this.)

A pgrep/pkill that matches nothing returns exit code 1 — that's just "already down",
not a failure. Confirm with the `ps` check above rather than trusting the exit code.
