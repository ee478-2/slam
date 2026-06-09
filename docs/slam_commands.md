# SLAM / Perception Command Cheat Sheet

Copy-paste reference for the **client-side** perception/SLAM stack we run on the
**Jetson Orin** (D435 + perception), against the Pi's roscore. The agent-facing
equivalents are the Claude skills `slam-bringup` / `slam-mapmon` / `slam-shutdown`
— this file is the human version.

| Machine | Role | IP |
|---|---|---|
| Raspberry Pi 4 | `roscore`, chassis, arm | `192.168.0.200` |
| Jetson Orin | D435, rtabmap, apriltag, YOLO, RViz | `192.168.0.101` |

**Hard rules**
- **Stop the RealSense camera with SIGINT (Ctrl-C / `kill -INT`) only.** SIGKILL /
  `rosnode kill` on the realsense nodelet wedges the D435 at the USB level — only a
  physical replug recovers it. (Other nodes — rtabmap/apriltag/yolo/rviz — are safe
  to SIGTERM/KILL.)
- The D435 is on a **USB 2.0** link → use the cut-down camera recipe below
  (640x480@15fps, IR+IMU off). Default `rs_camera.launch` saturates USB2.
- Launches below use `setsid ... &`; `$!` is the detached parent, so find the real
  PID with `ps -eo pid,comm` / `pgrep` (NB: `pgrep -f 'foo'` also matches your own
  command line — prefer `ps -eo pid,comm | awk '$2=="rviz"'`).

---

## Shortcuts (the fast path)

Source once, then one-word commands:
```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh   # add to ~/.bashrc to make permanent
slam up            # env + arm home + camera + wheel + rtabmap + apriltag + global-loc
slam wheel         # command-integrated /wheel/odom only
slam global-loc    # AprilTag anchor: global_map -> RTAB map
slam yolo-tags     # YOLO pose square tags as RTAB-compatible landmarks
slam rtab-map      # build RTAB feature DB for later localization
slam rtab-loc      # localize against saved RTAB feature DB
slam teleop        # drive
slam collect-cam   # camera-only storefront YOLO frame collection
slam collect       # save storefront YOLO frames while teleop drives
slam rviz          # viewer on :1   (slam rviz yolo for the YOLO view)
slam odom-viz      # RTAB odom path in green, wheel odom path in red
slam check         # quick status:  nodes / landmarks / tags seen
slam mon           # 15s map+VO monitor
slam down          # SIGINT teardown (camera stopped last)
slam help          # full list
```
Individual steps: `slam env | arm-home | cam | wheel | rtab | tags | yolo-tags | global-loc | loc | yolo | collect`.
Override the Pi IP / GUI display with `SLAM_PI=...`, `SLAM_DISPLAY=:0` before
sourcing.

The rest of this file is the long form of exactly what those shortcuts run.

---

## 0. Environment (every terminal)

```bash
cd ~/catkin_ws && source devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=$(ip -4 -o addr show eth0 | awk '{print $4}' | cut -d/ -f1)
```

Preflight (read-only):
```bash
ping -c1 -W1 192.168.0.200 && echo Pi-up || echo Pi-down   # roscore reachable?
ip neigh show 192.168.0.200                                # FAILED = Pi off L2
lsusb | grep -i 8086                                       # D435 = 8086:0b07
rostopic info /chassis_control/set_velocity                # chassis subscribed?
```

---

## 0b. Arm home pose

`slam up` starts by opening the gripper and moving the arm to the default home
pose through the Pi's existing controller topics:
```bash
setsid roslaunch slam arm_home.launch > /tmp/arm_home.log 2>&1 &
```

Defaults are radians, matching `joint1` through `joint5`:
`0 0.8 -3. -0.5 0`; gripper open is `-1.20` on
`/r_joint_controller/command`. Override for one run:
```bash
SLAM_ARM_HOME_POSE="0 0.8 -3. -0.5 0" \
SLAM_GRIPPER_HOME_POSITION="-1.20" slam up
```

---

## 1. RealSense camera — USB2 recipe

```bash
setsid roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true align_depth:=true enable_color:=true \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  color_width:=640 color_height:=480 color_fps:=15 \
  depth_width:=640 depth_height:=480 depth_fps:=15 \
  > /tmp/rs_camera.log 2>&1 &
```
Verify (must be NO `libusb: Resource temporarily unavailable`; the `2.1 port` WARN is fine):
```bash
grep -iE 'libusb|Resource temporarily' /tmp/rs_camera.log    # expect empty
# rate (rostopic hz won't average cross-master; use a subscriber):
python3 - <<'PY'
import rospy,time; from sensor_msgs.msg import Image
rospy.init_node('r',anonymous=True,disable_signals=True); c={'a':0,'b':0}
rospy.Subscriber('/camera/color/image_raw',Image,lambda _:c.__setitem__('a',c['a']+1))
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,lambda _:c.__setitem__('b',c['b']+1))
time.sleep(6); print(f"color~{c['a']/6:.1f}Hz depth~{c['b']/6:.1f}Hz")
PY
```

---

## 2. rtabmap RGB-D SLAM

```bash
setsid roslaunch slam rtabmap_realsense.launch rviz:=false rtabmap_viz:=false \
  > /tmp/rtabmap.log 2>&1 &
```
- Defaults: `--delete_db_on_start --Reg/Force3DoF true` (fresh map each run
  with flat-ground x/y/yaw registration) and `--Odom/ResetCountdown 1`
  (fail-fast VO recovery; without it a broken frame leaves odom LOST forever).
- To **keep** a map across restarts while preserving the flat-ground prior, add
  `rtabmap_args:="--Reg/Force3DoF true"`.
- Verify:
```bash
rosparam get /rtabmap/rgbd_odometry/Odom/ResetCountdown   # 1
rosparam get /rtabmap/rgbd_odometry/Reg/Force3DoF         # true
grep 'Odom: quality' /tmp/rtabmap.log | tail -1           # quality ~200-480, not 0
```

### 2b. Wheel odometry topic

```bash
setsid roslaunch slam wheel_odom.launch > /tmp/wheel_odom.log 2>&1 &
rostopic echo -n1 /wheel/odom
```

- Publishes `nav_msgs/Odometry` on `/wheel/odom`, frame `wheel_odom`,
  child `base_link`, by integrating `/chassis_control/set_velocity`.
- This is **open-loop command odometry**, not encoder feedback. The stock Pi
  chassis node exposes commands but no wheel tick topic.
- `linear_scale` converts the Hiwonder velocity field to m/s; default `0.01`
  means `velocity=40` integrates as `0.40 m/s`. Calibrate it with a measured
  straight drive before trusting distances.
- TF is off by default (`publish_tf:=false`) to avoid conflicting with RTAB-Map's
  live `odom->base_link` TF. For isolated tests only:
  `roslaunch slam wheel_odom.launch publish_tf:=true`.
- `rtabmap_realsense.launch` now passes through `odom_guess_frame_id`,
  `odom_guess_min_translation`, and `odom_guess_min_rotation` for future motion
  prior experiments; defaults leave RTAB-Map behavior unchanged.

### 2c. RTAB feature-DB workflow without YOLO labels

If labeling storefronts is too expensive, use RTAB-Map itself as the feature
matcher. The saved artifact is the RTAB database, not a rosbag. A rosbag is only
needed if you want to replay raw camera data later.

Build a reference visual map:
```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam env
slam cam
SLAM_RTAB_DB=$HOME/.ros/storefront_ref.db SLAM_RTAB_RESET=true slam rtab-map

# another terminal:
slam teleop
```

Stop with `slam down`; the DB remains at `$HOME/.ros/storefront_ref.db`.

Multiple mapping passes are OK, and usually better than trying to capture one
perfect view. For the first pass, use `SLAM_RTAB_RESET=true` to start a clean
DB. To add more coverage to the same DB later, keep the same camera mount and
run mapping again with `SLAM_RTAB_RESET=false`:

```bash
SLAM_RTAB_DB=$HOME/.ros/storefront_ref.db SLAM_RTAB_RESET=false slam rtab-map
```

Drive slowly, keep adjacent passes visually overlapping, and start additional
passes from a place the existing DB can recognize. Prefer stable structure:
walls, storefront frames, fixed signs, corners, and AprilTags. Avoid spending
mapping time on things likely to move or change, such as people, carts,
temporary displays, open doors, and seasonal merchandise. If the camera mount
or signboard/store layout changes significantly, build a new reference DB.

Later, localize against that DB without adding new map nodes:
```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam env
slam cam
SLAM_RTAB_DB=$HOME/.ros/storefront_ref.db slam rtab-loc
```

This gives feature-based place recognition / localization in RTAB's `map` frame.
It does not identify a semantic store category and it does not by itself align
RTAB's local map to `global_map`; use AprilTags, a known start pose, or manual
map alignment for that global anchor.

### 2d. YOLO pose square-tag landmarks for RTAB

The YOLO pose model at `$(find slam)/pose_best.engine` can publish square-tag
detections in the same `apriltag_ros/AprilTagDetectionArray` format RTAB already
subscribes to. The square is treated as exactly `0.15 m x 0.15 m`:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam env
slam cam
slam rtab
slam yolo-tags
```

Visualize the detector overlay:
```bash
rqt_image_view /yolo_pose_tag_detector/debug_image
```

Defaults:
- Output topic is `/tag_detections`, so RTAB sees these as normal landmarks.
- Debug overlay topic is `/yolo_pose_tag_detector/debug_image`; it draws
  keypoints, horizontal width edges, tag id, score, and the `3/3` publication
  gate state.
- Default model path is `pose_best.engine` through Ultralytics TensorRT. If the
  engine is missing, the detector falls back to `pose_best.onnx` and then
  `pose_best.pt` when matching files exist.
- Tag IDs default to `1000 + yolo_class_id`, avoiding collisions with physical
  AprilTag IDs `1..28`.
- `store1..store8` are softly mapped to the `stores:` entries in
  `config/global_map.yaml` in the same order. This appears only in
  `/yolo_pose_tag_detector/status` as `global_map_id`, `global_category`, and
  `global_xy`; it does not move `global_map -> map`.
- The same physical square must keep the same class/tag ID over time. If the
  model has one class but multiple physical square tags, RTAB will collapse them
  into one landmark; train/use per-tag classes or set
  `SLAM_YOLO_POSE_CLASS_ID_TO_TAG_ID="0:1000,1:1001"` before `slam yolo-tags`.
- A detection must survive `SLAM_YOLO_POSE_MIN_STABLE_FRAMES=3` consecutive
  processed frames before publishing.
- Published poses are EMA-smoothed with `SLAM_YOLO_POSE_EMA_ALPHA=0.35`.

Vertical keypoint extent is intentionally weak. Even if all four keypoints solve
PnP, the node biases x/z translation toward the horizontal pixel width
(`pnp_horizontal_translation_weight=0.80`) and publishes larger covariance on the
camera optical vertical axis than on the horizontal axis. This matches the
labeling rule: label the visible vertical part when occluded, but do not let
RTAB treat that vertical extent as a precise metric. If only one or both
horizontal edges are visible, the node falls back to width-only translation with
even larger vertical covariance.

RTAB-Map's AprilTag subscriber can also apply its global
`tag_linear_variance` parameter depending on version/configuration. The pose
itself is still horizontal-weighted, so the vertical-label caveat is not relying
only on covariance propagation.

Global-frame anchoring remains AprilTag-first. `slam global-loc` deliberately
does not use YOLO store IDs `1001..1008` to publish `global_map -> map` by
default; those detections are soft RTAB landmarks plus debug metadata only.

Useful overrides:
```bash
SLAM_YOLO_POSE_HZ=3.0 \
SLAM_YOLO_POSE_MIN_STABLE_FRAMES=3 \
SLAM_YOLO_POSE_EMA_ALPHA=0.35 \
slam yolo-tags
```

`training/storefront_yolo/export_yolo.py` generates the TensorRT engine. Build
or refresh the engine on the Jetson deployment GPU:
```bash
cd ~/catkin_ws/src/slam
python3 training/storefront_yolo/export_yolo.py \
  --weights pose_best.pt \
  --format engine \
  --half \
  --imgsz 640 \
  --device 0
```

To force the portable ONNX path instead:
```bash
SLAM_YOLO_POSE_MODEL=$HOME/catkin_ws/src/slam/pose_best.onnx slam yolo-tags
```

To offload storefront YOLO pose inference to a laptop, run camera/RTAB on the
Jetson and run the detector from a ROS-configured laptop shell. This keeps the
TensorRT/Ultralytics memory footprint off the Jetson while still publishing the
same `/tag_detections` topic for RTAB-Map:
```bash
# Jetson:
./net_init.sh          # or the route/NAT setup from §6b
slam cam
slam rtab

# Laptop:
sudo ip route add 192.168.0.0/24 via <jetson_hotspot_ip>
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam yolo-tags-remote
```

`slam yolo-tags-remote` sets `ROS_MASTER_URI=http://192.168.0.200:11311`, infers
`ROS_IP` from `ip route get 192.168.0.200`, uses `pose_best.onnx` by default,
subscribes `/camera/color/image_raw` + `/camera/color/camera_info`, and publishes
`/tag_detections`. Override with `SLAM_REMOTE_YOLO_POSE_MODEL`,
`SLAM_REMOTE_YOLO_POSE_HZ`, `SLAM_REMOTE_YOLO_POSE_IMAGE_TOPIC`,
`SLAM_REMOTE_YOLO_POSE_CAMERA_INFO_TOPIC`, or `SLAM_REMOTE_ROS_IP` if needed.

Stop it with `slam down`, or only that node with:
```bash
pkill -INT -f 'roslaunch slam yolo_pose_tag_detector'
pkill -INT -f 'yolo_pose_tag_detector.py'
```

---

## 3. AprilTag detection + global RTAB anchoring

```bash
setsid roslaunch slam apriltag_realsense.launch > /tmp/apriltag.log 2>&1 &
```
- Publishes `/tag_detections` (+ `/tag_detections_image`). `rtabmap_realsense.launch`
  already subscribes `/tag_detections` and adds tags as **graph landmarks**
  (landmark graph id = `-(tag_id)`; poses on `/rtabmap/landmarks`, frame `map`).
  Landmark translation variance defaults to `tag_linear_variance:=0.005`;
  rotation remains ignored with RTAB-Map's `tag_angular_variance:=9999`.
- Check detections + landmark registration:
```bash
python3 - <<'PY'
import rospy,time
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
rospy.init_node('t',anonymous=True,disable_signals=True); s={'ids':{},'lm':0}
def td(m):
    for d in m.detections:
        k=d.id[0] if len(d.id)==1 else tuple(d.id); s['ids'][str(k)]=s['ids'].get(str(k),0)+1
rospy.Subscriber('/tag_detections',AprilTagDetectionArray,td)
rospy.Subscriber('/rtabmap/landmarks',PoseArray,lambda m:s.__setitem__('lm',len(m.poses)))
time.sleep(6); print('tags seen:',s['ids'] or 'none','| landmarks:',s['lm'])
PY
```

Global anchoring is a separate node. It does not rewrite RTAB-Map's local
`map`; it publishes a parent transform from the fixed room frame to RTAB's map:

```bash
setsid roslaunch slam apriltag_global_localization.launch > /tmp/global_loc.log 2>&1 &
rostopic echo -n1 /global_localization/selected_tag
rosrun tf tf_echo global_map map
```

When a known signboard bundle is visible, the node matches the detected
`/tag_detections.id` values back to the owning `SIGNBOARDxx` in
`config/global_map.yaml`, solves the planar `global_map -> map` transform that
places the observed RTAB tag point exactly on that signboard's global x/y point,
and publishes `/global_localization/robot_pose`. The localization manager
consumes that pose first, so `/odom` and `/robot_pose` are in `global_map` while
the anchor is fresh; if no tag anchor is available, they fall back to RTAB's
local odometry frame. `/global_localization/selected_tag` reports the match
method, tag IDs, `stable_frames`, `min_stable_frames`,
`smoothing_window_samples`, `smoothing_window_size`, and `anchor_error_m`.

Direct `/tag_detections` anchors are stabilized by default: a signboard must be
seen for `min_stable_frames:=3` consecutive detection frames before it can move
`global_map -> map`. While the count is warming up, the node holds the previous
anchor instead of falling through to a one-frame TF anchor. After the count is
stable, direct detection anchors use a short smoothing window: x/y/z are median
filtered and yaw uses a circular mean over the latest
`smoothing_window_size:=5` samples. This intentionally does not add hard jump
rejection; if a large correction is real and stays consistent for the window, it
is allowed to update the anchor.

Planar anchor yaw ignores small AprilTag in-plane paper rotation by default.
The localizer compares the tag-frame Euler-yaw anchor against the tag plane
normal's horizontal heading and picks the normal-heading candidate closest to
the Euler-yaw prior, limited by `max_tag_inplane_yaw_correction_deg:=60.0`.
This corrects visibly twisted tags without allowing a 180-degree normal flip to
turn the map around. `/global_localization/selected_tag` reports
`heading_source` and `inplane_yaw_correction_deg` for live checks.

YOLO store detections are not global anchors in the default launch, even though
they share `/tag_detections`; only IDs configured as AprilTag signboard tags in
`global_map.yaml` can move `global_map -> map`.

Tune the shortcut with:

```bash
SLAM_APRILTAG_MIN_STABLE_FRAMES=5 SLAM_APRILTAG_SMOOTHING_WINDOW=7 slam global-loc
SLAM_APRILTAG_MAX_INPLANE_CORRECTION_DEG=35 slam global-loc
```

---

## 4. YOLO object detection (manipulation_control — outside slam scope, run with permission)

```bash
setsid rosrun manipulation_control object_detection.py \
  _base_frame:=camera_link _visualize:=true _inference_hz:=8.0 \
  > /tmp/yolo_det.log 2>&1 &
```
- Classes: `cup`, `drink`, `hamburger`, `medicine`. Publishes `/detected_objects`,
  `/detected_object_positions_base`, `/detected_objects/debug_image`.
- `_base_frame:=camera_link` avoids needing a robot/`base_link` TF.
```bash
rostopic echo -n1 /detected_objects                # names + confidence + pose
```

## 4b. Storefront YOLO data collection

This collects RGB images for training a storefront/signboard detector while the
robot is driven manually. For battery-limited collection, use the camera-only
path; it does **not** start RTAB-Map, AprilTag detection, wheel odometry, or the
localization manager:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam collect-cam

# another real keyboard terminal:
slam teleop
```

Equivalent manual steps:
```bash
slam env
slam cam
slam collect
```

Default output:
```text
~/catkin_ws/src/slam/data/storefront_yolo/raw/<session>/
  images/
  classes.txt
  metadata.csv
```

Useful overrides:
```bash
SLAM_STORE_YOLO_HZ=1.0 slam collect
SLAM_STORE_YOLO_SESSION=aisle_slow_01 slam collect
SLAM_STORE_YOLO_RAW=/media/usb/storefront_raw slam collect
```

The collector subscribes to `/camera/color/image_raw`. If `/odom` exists, it
also stores the latest pose in `metadata.csv`; without `/odom`, pose columns are
left blank and collection still works. It samples by time plus optional movement
gating, so slow teleop produces less duplicate data when odom is available.
Label the images in CVAT with class `storefront`, export as YOLO, then use
`training/storefront_yolo/prepare_cvat_yolo.py` and
`training/storefront_yolo/train_yolo.py` on the GPU server.

---

## 5. teleop (real keyboard required)

```bash
rosrun slam teleop_keyboard.py
```
`w/s` fwd/back · `a/d` strafe · `j/l` rotate · `z/c` slower/faster · SPACE/x stop ·
`r` re-arm · `q` quit. Hold to move (auto-stops ~0.4 s after release). Drive **slowly**,
esp. in-place rotation (15 fps VO breaks on fast yaw). Max linear 75; start ~15.
**If some wheels stop mid-drive** (motor-board over-current/stall latch — more likely at
high speed), press **`r`** to re-arm without quitting (sustained zero burst → resume).

---

## 6. Viewers on the GUI desktop (DISPLAY :1)

Attach a viewer to the **already-running** stack (do NOT relaunch with `rviz:=true` —
that restarts rtabmap and wipes the map):
```bash
# AprilTag + rtabmap landmarks + cloud + path:
DISPLAY=:1 setsid rviz -d ~/catkin_ws/src/slam/rviz/apriltag_rtabmap.rviz > /tmp/rviz.log 2>&1 &
# YOLO debug image:
DISPLAY=:1 setsid rviz -d ~/catkin_ws/src/slam/rviz/yolo.rviz > /tmp/rviz.log 2>&1 &
# Total mission view: global stores/signboards + status + rtabmap + tags + grasp:
slam mission
# RTAB visual odom vs command wheel odom, no AprilTag displays:
slam odom-viz
# rtabmap's own viewer (shows landmarks in the graph):
DISPLAY=:1 ROS_NAMESPACE=rtabmap setsid rosrun rtabmap_viz rtabmap_viz _frame_id:=camera_link &
```
Confirm rviz is up without self-matching your command:
```bash
ps -eo pid,comm | awk '$2=="rviz"{print "rviz pid="$1}'
```

The odom comparison view runs `launch/rtab_wheel_viz.launch`, which republishes
RTAB-Map's `/rtabmap/mapPath` plus `/rtabmap/odom` to `/odom_compare/rtab_*`
and `/wheel/odom` to `/odom_compare/wheel_*` in a synthetic `odom_compare`
frame. It assumes the RTAB and wheel odom origins are comparable, so start it
with the stack for the cleanest drift check; it is not a TF fusion result.
RTAB-Map is drawn green; wheel odom is drawn red.

---

## 6b. RViz on YOUR OWN PC (ROS Noetic) — remote, native

Best remote option: run RViz locally on your machine, subscribing over the
network. Renders on your GPU; **no custom package needed** — our config uses only
standard msgs (PointCloud2 / PoseArray / Image / Path / TF).

Why it works: ROS1 master (Pi) only resolves names; topic data flows
**peer-to-peer from the publisher**. rtabmap/camera/apriltag publish on the
**Jetson (192.168.0.101)** — so your PC must reach **both** the Pi (master) and
the Jetson (publishers).

On **your PC**, every terminal:
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=$(hostname -I | awk '{print $1}')      # your PC's 192.168.0.x IP
ping -c1 192.168.0.200 && ping -c1 192.168.0.101      # both must reply
rostopic list | grep -E 'rtabmap|tag_detections'      # should list them
```
Clock sync (TF breaks if the PC and Jetson clocks differ by >~0.1s):
```bash
date ; ssh ee478_team2@192.168.0.101 date             # compare
sudo ntpdate 192.168.0.200      # or chrony both to one source, if they drift
```
Get the config + launch:
```bash
scp ee478_team2@192.168.0.101:~/catkin_ws/src/slam/rviz/apriltag_rtabmap.rviz /tmp/
rviz -d /tmp/apriltag_rtabmap.rviz                    # Fixed Frame = map
```
For the **total mission view**, first run the marker publisher on the Jetson,
then run RViz on your PC:
```bash
# on Jetson
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam mission-pub

# on your PC
scp ee478_team2@192.168.0.101:~/catkin_ws/src/slam/rviz/mission.rviz /tmp/
rviz -d /tmp/mission.rviz
```
Notes:
- `/rtabmap/cloud_map` is the full growing cloud — heavy over **WiFi**. If laggy,
  uncheck the PointCloud2 display; landmarks + path + tag image stay light.
- The mission map markers are published on `/mission/markers` from
  `config/global_map.yaml` in frame `global_map`. RTAB path/cloud topics are
  transformed through `global_map -> map` once `slam global-loc` sees a known
  signboard tag. The status panel listens for `/shopping_list`, `/grabbed_items`,
  `/inventory`, and `/visited_stores` if those are available.
- Optional `sudo apt install ros-noetic-rtabmap-ros` only if you want rtabmap's
  own RViz plugins / `mapData` display — not needed for `apriltag_rtabmap.rviz`.
- Nothing runs on the Jetson `:1` for this; it's purely your machine.

### When your PC is on a DIFFERENT network (e.g. a phone hotspot)

The ROS master is the Pi on the **wired robot LAN** (`192.168.0.200`). If your PC
is on a hotspot it can't reach that LAN — but the Jetson is **dual-homed**
(`eth0` = robot LAN `192.168.0.101`, `wlan0` = hotspot e.g. `10.40.187.185`), so
make the **Jetson a gateway** and route your PC's robot-LAN traffic through it.

```bash
# --- on the JETSON (ip_forward is usually already 1) ---
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null \
  || sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
# *** the gotcha *** docker sets the FORWARD policy to DROP, which silently eats
# forwarded packets even with MASQUERADE + ip_forward. Open hotspot<->robot-LAN:
sudo iptables -I FORWARD 1 -i wlan0 -o eth0 -j ACCEPT
sudo iptables -I FORWARD 1 -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
#   (blunt alternative: sudo iptables -P FORWARD ACCEPT)

# --- on YOUR PC ---
sudo ip route add 192.168.0.0/24 via 10.40.187.185   # Jetson's hotspot IP
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
```
Debug ladder from the PC (find where it stops):
```bash
ping 10.40.187.185     # 1 Jetson hotspot   (DHCP can reassign this — recheck on Jetson: ip -br addr)
ip route get 192.168.0.200   # 2 must say 'via 10.40.187.185'
ping 192.168.0.101     # 3 Jetson eth0 (route only, no NAT)
ping 192.168.0.200     # 4 Pi (FORWARD+MASQUERADE) -- 3 OK but 4 fails == the docker FORWARD DROP
```
Notes: iptables/route changes are **not persistent** (gone on reboot — re-run, or
persist with `iptables-save`/a netplan route). The Pi is never touched (MASQUERADE
hides the PC behind the Jetson). Pi→PC XMLRPC callbacks can't route back, so
mid-session publisher changes may not propagate — fine for viewing live topics.
If this is too much, just **`ssh -X ee478_team2@10.40.187.185` and run `rviz` on the
Jetson** (X forwards to your PC) — needs no routing/NAT, only PC↔Jetson hotspot
reachability. (Run `rviz -d ...` directly, not the `slam rviz` shortcut, which
forces the Jetson-local `:1` display.)

---

## 7. Monitoring one-liners

Map growth / loop closures / VO health while driving:
```bash
python3 - <<'PY'
import rospy,time,math
from rtabmap_msgs.msg import MapData,Info
from nav_msgs.msg import Odometry
rospy.init_node('m',anonymous=True,disable_signals=True)
s={'n':0,'lc':0,'lost':0,'good':0,'p':None,'d':0.0}
def md(m): s['n']=len(m.graph.poses)            # TRUE node count (NOT len(m.nodes))
def inf(m):
    if getattr(m,'loopClosureId',0)>0: s['lc']+=1
def od(m):
    s['lost' if m.pose.covariance[0]>=9998 else 'good']+=1   # covariance on m.pose
    p=m.pose.pose.position
    if s['p']: s['d']+=math.dist((p.x,p.y,p.z),s['p'])
    s['p']=(p.x,p.y,p.z)
rospy.Subscriber('/rtabmap/mapData',MapData,md)
rospy.Subscriber('/rtabmap/info',Info,inf)
rospy.Subscriber('/rtabmap/odom',Odometry,od)
time.sleep(15); print(f"nodes={s['n']} loopclose={s['lc']} VO good/lost={s['good']}/{s['lost']} travel~{s['d']:.2f}m")
PY
```

---

## 8. Clean shutdown (SIGINT-only for the camera)

```bash
# consumers first (safe to TERM/KILL), camera LAST (SIGINT only):
pkill -INT -f 'rtabmap_viz/rtabmap_viz'
pkill -INT -f 'roslaunch slam apriltag_realsense'
pkill -INT -f 'roslaunch slam rtabmap_realsense'
pkill -f 'object_detection.py'            # yolo: TERM ok
sleep 5
pkill -INT -f 'roslaunch realsense2_camera'   # camera: SIGINT, then wait
# WAIT for full USB release before any restart — a quick relaunch races
# device-busy and the camera nodelet dies (topics advertised but 0 Hz):
while pgrep -f 'rs_camera.launch' >/dev/null; do sleep 1; done
# verify down + device healthy:
ps -eo pid,comm | grep -E 'rtabmap|rgbd_odom|nodelet|rviz' | grep -v grep || echo "(all down)"
lsusb | grep -i 8086 && echo "D435 ok"
```
If `rs-enumerate-devices` reports "No device detected" while `lsusb` still shows
8086:0b07 → the D435 is **wedged**, needs a physical USB replug.

---

## Gotchas (cost us time)

- **Node count** = `len(MapData.graph.poses)`, not `len(MapData.nodes)` (the latter is
  incremental and reads ~1).
- **VO-lost flag** = `odom.pose.covariance[0] >= 9998` (covariance is on
  `PoseWithCovariance`, i.e. `pose.covariance`, NOT `pose.pose.covariance`).
- **`/rtabmap/mapPath` includes landmark vertices** (z≈tag height) among robot poses
  (z≈0), so the RViz Path spikes up to landmarks — artifact, not a bug.
- **`pgrep -f`/`pkill -f` self-match** your own command line → false "UP" reads. Use
  `ps -eo pid,comm` (comm field has no args).
- `rostopic hz` never prints an average against the cross-machine master — use a rospy
  subscriber.
