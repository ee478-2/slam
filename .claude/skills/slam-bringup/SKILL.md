---
name: slam-bringup
description: Bring up the ArmPi + Jetson perception/SLAM stack on the real robot — preflight checks (Pi roscore, D435 USB), RealSense camera over the USB2 link, rtabmap RGB-D SLAM, and teleop. Use when the user wants to start the robot / camera / rtabmap / "로봇 띄워", "perception 올려", or resume a real-robot SLAM test.
---

# slam-bringup — real-robot perception/SLAM bring-up

Brings the on-Jetson perception stack up end to end. The robot is a **two-computer
system**: Raspberry Pi (`192.168.0.200`, roscore + chassis + arm) and the Jetson
Orin (this machine, perception + D435). Run everything here with the Pi as ROS
master.

**Hard rules (learned the hard way — do not skip):**
- **SIGINT only** to stop the camera/nodelets. SIGKILL / `rosnode kill` on the
  realsense nodelet wedges the D435 at the USB level — only a physical replug
  recovers it. Teardown is the `slam-shutdown` skill.
- The D435 is on a **USB 2.0** link (charge-only/USB2 cable). RGB-D SLAM works
  only with the cut-down stream set below (640x480 @ 15fps, IR+IMU off). The
  default `rs_camera.launch` (848x480, all streams) saturates USB2 and floods
  `libusb: Resource temporarily unavailable`.
- Launches are started with `setsid`, so `$!` is the **detached parent**, not the
  live roslaunch. Always resolve the real PID with `pgrep -f`.

## 0. Environment

```bash
cd ~/catkin_ws && source devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=$(ip -4 -o addr show eth0 | awk '{print $4}' | cut -d/ -f1)   # this machine on the .0.x net
echo "ROS_IP=$ROS_IP  MASTER=$ROS_MASTER_URI"
```

## 1. Preflight (read-only — confirm before launching anything)

```bash
ping -c1 -W1 192.168.0.200 && echo "Pi reachable" || echo "Pi DOWN — power/ethernet/IP"
ip neigh show 192.168.0.200          # FAILED = Pi not on L2 (off / wrong segment)
lsusb | grep -i 8086                 # D435 = 8086:0b07. Bus 001 => USB2 (expected)
lsusb -t | grep -iB1 8086            # want 5000M if a real USB3 cable is ever used
```

Confirm the chassis is listening (teleop target) — and note the conflict:
```bash
rostopic info /chassis_control/set_velocity
```
`/chassis_control` must be a **Subscriber**. NOTE: stock `/visual_patrol` and
`/apriltag_detect` are also **publishers** on this topic — if either goes active it
fights teleop. If the robot moves on its own, that's them.

Stop here and tell the user if the Pi is unreachable or the D435 is absent — those
are physical fixes, not software.

## 2. RealSense camera — USB2 low-bandwidth recipe

```bash
setsid roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true align_depth:=true enable_color:=true \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  color_width:=640 color_height:=480 color_fps:=15 \
  depth_width:=640 depth_height:=480 depth_fps:=15 \
  > /tmp/rs_camera.log 2>&1 &
sleep 10
pgrep -f 'roslaunch realsense2_camera' | head -1 > /tmp/rs_camera.pid   # REAL pid
```

`realsense2_camera` is **outside `slam/` edit scope** — execute-only, and only with
the user's go-ahead for this run. Verify:
- `grep -iE 'libusb|Resource temporarily' /tmp/rs_camera.log` → **must be empty**.
  (`Device ... is connected using a 2.1 port` WARN is expected and fine.)
- Measure rates with a short rospy subscriber (`rostopic hz` never prints an average
  against this cross-machine master): `/camera/color/image_raw` and
  `/camera/aligned_depth_to_color/image_raw` should each be **~15 Hz**.

## 3. rtabmap RGB-D SLAM (headless)

```bash
setsid roslaunch slam rtabmap_realsense.launch rviz:=false rtabmap_viz:=false \
  > /tmp/rtabmap.log 2>&1 &
sleep 12
pgrep -f 'roslaunch slam rtabmap_realsense' | head -1 > /tmp/rtabmap.pid
```

The launch defaults `--Odom/ResetCountdown 1` (fail-fast VO recovery). Without it,
one broken frame leaves odom LOST forever and the map freezes at 1 node. Verify:
- `rosparam get /rtabmap/rgbd_odometry/Odom/ResetCountdown` → `1`
- log shows `Odom: quality=NNN` with NNN ~200-390 (not 0)
- no `Did not receive data since 5 seconds` from the rtabmap node

`--delete_db_on_start` is the default, so each bring-up starts a fresh map. To keep a
map across restarts, drop that arg (`rtabmap_args:=""`) — and never restart blindly.

## 4. teleop (user runs it — needs a live keyboard)

Tell the user to run, in a real terminal:
```bash
rosrun slam teleop_keyboard.py
```
`w/s` fwd/back, `a/d` strafe, `j/l` rotate, `z/c` slower/faster, SPACE/x stop, `q` quit.
Drive **slowly**, especially in-place rotation — fast yaw at 15fps breaks VO (ResetCountdown
recovers it, but you lose the segment).

## 5. Optional viewer on the GUI session

If a desktop session is open (check `who | grep ':[0-9]'`, e.g. DISPLAY `:1`), attach a
viewer to the already-running rtabmap (do NOT relaunch the stack with rviz:=true — that
restarts and wipes the map):
```bash
DISPLAY=:1 ROS_NAMESPACE=rtabmap setsid rosrun rtabmap_viz rtabmap_viz \
  _frame_id:=camera_link > /tmp/rtabmap_viz.log 2>&1 &
```

After bring-up, offer `slam-mapmon` to watch the map grow and `slam-shutdown` to tear
down cleanly. Update `docs/PROGRESS.md` if this run produces a result worth logging.
