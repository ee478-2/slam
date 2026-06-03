# ArmPi Pro — Robot Command Reference

Demo-practice cheat sheet for the real robot. The system is **two
computers** (EE478 Week6 hardware config):

| Machine | Role | IP | OS |
|---|---|---|---|
| ArmPi Pro Raspberry Pi 4 | `roscore`, chassis, arm, mono cam | `192.168.0.200` | Ubuntu 18.04 / ROS Melodic |
| Jetson Orin / laptop (client) | RealSense, perception, RViz | `192.168.0.101` | — / ROS Noetic |

Robot login: user `ubuntu`, password `hiwonder`. On the Pi the workspace
is `/home/ubuntu/armpi_pro/` (the `armpi/` copy on the laptop is the
same tree, for reading source only — its binaries are aarch64).

---

## 0. Connect to the robot

**SSH into the Pi** (run robot-side commands here):
```bash
ssh ubuntu@192.168.0.200          # password: hiwonder
```

**ROS multi-machine** (so the laptop sees the robot's topics) — on the
**laptop**, every terminal:
```bash
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=192.168.0.101
source /opt/ros/noetic/setup.bash
rostopic list                     # should list the robot's topics
```

---

## 1. Robot stack control  (run on the Pi)

The ROS stack **autostarts on boot** via systemd. Manage it:
```bash
sudo systemctl status  start_node.service     # is it running?
sudo systemctl restart start_node.service     # restart roscore + all nodes
sudo systemctl stop    start_node.service     # stop everything
```
Manual start (if not using systemd):
```bash
bash ~/armpi_pro/src/armpi_pro_bringup/scripts/start_node.sh
```
This brings up: `lab_config`, `web_video_server`, `rosbridge`, `usb_cam`,
sensor board, `hiwonder_servo_controllers` (arm), `chassis_control`,
`visual_processing`, `face_detect`, `visual_patrol`, `color_tracking`,
`apriltag_detect`.

Check what's up:
```bash
rosnode list
rostopic list
```

---

## 2. Camera  (mono USB "visual servoing" cam, on the Pi)

Setup: comes up with `start_camera.launch` (part of bringup). Topic:
`/usb_cam/image_raw`, 640×480.

```bash
# View in a browser (web_video_server, no ROS needed):
#   http://192.168.0.200:8080

# Or from the laptop with ROS networking set:
rqt_image_view /usb_cam/image_raw
rostopic hz /usb_cam/image_raw          # expect ~30 Hz
```

---

## 3. Chassis — mecanum drive  (on the Pi, or laptop with chassis_control built)

Setup: `chassis_control` node is in the bringup. Message
`chassis_control/SetVelocity { velocity, direction, angular }` —
`direction` is in **degrees** (90 = forward), base is holonomic.

⚠️ To publish this from the **laptop**, the laptop workspace must have the
`chassis_control` package built (custom msg). Easiest: run these on the
Pi via SSH.

```bash
# Forward (direction 90):
rostopic pub /chassis_control/set_velocity chassis_control/SetVelocity \
  "{velocity: 10.0, direction: 90.0, angular: 0.0}"
# Backward = direction 270 ; strafe = 0 / 180
# Rotate in place:
rostopic pub /chassis_control/set_velocity chassis_control/SetVelocity \
  "{velocity: 0.0, direction: 90.0, angular: 0.15}"
# STOP (always end with this):
rostopic pub -1 /chassis_control/set_velocity chassis_control/SetVelocity \
  "{velocity: 0.0, direction: 90.0, angular: 0.0}"
```
There is also `/chassis_control/set_translation` (`{velocity_x, velocity_y}`).

---

## 4. Arm + gripper  (on the Pi)

Setup: `hiwonder_servo_controllers` is in the bringup. Per-joint topics
are `std_msgs/Float64` (radians). Discover them:
```bash
rostopic list | grep controller
```

```bash
# Arm joints (joint1 = base rotation ... joint5 = wrist):
rostopic pub -1 /joint1_controller/command std_msgs/Float64 "{data: 0.5}"
rostopic pub -1 /joint2_controller/command std_msgs/Float64 "{data: 0.5}"

# Gripper (r_joint): positive = close, negative = open
rostopic pub -1 /r_joint_controller/command std_msgs/Float64 "{data: 0.5}"
rostopic pub -1 /r_joint_controller/command std_msgs/Float64 "{data: -1.0}"
```
Gripper via action interface (see `armpi/2.py`):
`/gripper_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory).

All-in-one smoke test (chassis + arm + gripper): `bash ~/armpi/run.bash`.

---

## 5. AprilTag  (on the Pi)

Setup: `apriltag_detect` node runs in the bringup. It is a *behavior*
node (tag-following), not a raw detector. Hold a tag in front of the
camera and watch the node react / the arm move.

For a clean raw-detection stream (`/tag_detections`) for signboard
recognition, run **apriltag_ros on a client** — see §7.

---

## 6. Inspect / debug

```bash
rosnode list ; rostopic list
rostopic echo /usb_cam/image_raw --noarr
rostopic hz   /chassis_control/set_velocity
rosnode info /chassis_control
roswtf                                  # general health check
```

---

## 7. Client-side launches  (laptop or Jetson — NOT the Pi)

These run on the client because the Pi is too weak. The client shares
the Pi's `roscore` (set `ROS_MASTER_URI` per §0).

**RTAB-Map** — needs the RealSense D435 (plugged into the Jetson, or the
laptop directly):
```bash
source ~/llm-skill/devel/setup.bash
roslaunch my_rtabmap rtabmap_realsense.launch
# RViz shows the map; carry the camera to build it + loop-close.
```

**Signboard recognition on the real USB cam**:
```bash
source ~/llm-skill/devel/setup.bash
roslaunch signboard_recognition signboard_recognition_real.launch
rqt_image_view /signboards/detections_image
rostopic echo  /signboards/detections
```

---

## Quick demo-practice sequence

```
1. ssh ubuntu@192.168.0.200 ; check  systemctl status start_node.service
2. browser http://192.168.0.200:8080            -> camera works
3. chassis forward / back / rotate / STOP        (§3)
4. arm joint1/joint2 + gripper open/close        (§4)
5. hold an AprilTag in front of the camera       (§5)
6. (client) roslaunch my_rtabmap rtabmap_realsense.launch
7. (client) roslaunch signboard_recognition signboard_recognition_real.launch
```
