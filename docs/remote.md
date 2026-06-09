# Remote Operation

This file covers laptop-side ROS use for the robot stack:

- remote RViz / `rostopic` inspection from a laptop
- laptop-side YOLO pose tag inference with `slam yolo-tags-remote`
- process checks, shutdown, and common `no new messages` failures

Network roles:

| Machine | Role | Default IP |
|---|---|---|
| Raspberry Pi 4 | ROS master, chassis, arm | `192.168.0.200` |
| Jetson Orin | D435 camera, RTAB-Map, gateway | `192.168.0.101` on robot LAN |
| Laptop | remote viewer or remote YOLO inference | route-dependent |

ROS1 topic data is peer-to-peer. The ROS master only resolves names, so the
laptop must be reachable by the machines that publish or subscribe to topics.
Do not guess `ROS_IP` from the first `hostname -I` address unless it matches the
route to the robot LAN.

---

## 1. Jetson Setup

Run this on the Jetson when the laptop is not directly on the robot LAN and must
reach `192.168.0.0/24` through the Jetson hotspot interface:

```bash
cd ~/catkin_ws/src/slam
./net_init.sh
```

`net_init.sh` enables IP forwarding and adds NAT/FORWARD iptables rules between
the Jetson hotspot interface and the robot LAN. It is not persistent across
reboots, so rerun it after reboot or network reset.

For laptop YOLO offload, run camera/RTAB on the Jetson and do not run Jetson
YOLO at the same time:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam env
slam cam
slam rtab
```

Optional Jetson-side nodes still make sense depending on the experiment:

```bash
slam tags        # physical AprilTag landmarks
slam global-loc  # global_map -> map from known signboard tags
```

Do not run both of these at the same time unless you intentionally want duplicate
YOLO landmark publishers:

```bash
slam yolo-tags          # Jetson-local YOLO, TensorRT engine default
slam yolo-tags-remote   # laptop-side YOLO, PyTorch .pt default
```

Camera timing does not have to be exact. `slam yolo-tags-remote` can be started
before the camera exists; it subscribes and waits. If the camera restarts with
the same ROS master and same topic names, the remote detector should reconnect.
If the ROS master, route, or `ROS_IP` changes, restart the remote detector.

---

## 2. Laptop Workspace

Copying only `scripts/slam_aliases.sh` is not enough for YOLO offload. The laptop
needs a ROS workspace containing the `slam` package, launch files, detector
script, message dependencies, Python dependencies, and the model file.

Minimum expected laptop layout:

```text
~/catkin_ws/
  devel/setup.bash
  src/slam/
    scripts/slam_aliases.sh
    scripts/yolo_pose_tag_detector.py
    launch/yolo_pose_tag_detector.launch
    pose_best.pt      # default remote model, ignored by git
    pose_best.onnx    # optional fallback/override, ignored by git
```

Build/source the workspace once after copying the package:

```bash
cd ~/catkin_ws
catkin_make --pkg slam
source devel/setup.bash
```

The remote detector imports at least:

- ROS Noetic Python packages, including `cv_bridge` and `apriltag_ros` messages
- `ultralytics`
- `torch` for `.pt` models
- `opencv-python` / system OpenCV Python bindings
- `numpy`
- `PyYAML`

By default `slam yolo-tags-remote` uses:

- model: `$SLAM_WS/src/slam/pose_best.pt`
- fallback model: `$SLAM_WS/src/slam/pose_best.onnx` if `.pt` is missing
- `imgsz`: `640` for `.pt`, `512` for `.onnx`
- image topic: `/camera/color/image_raw`
- camera info topic: `/camera/color/camera_info`
- output topic: `/tag_detections`
- status topic: `/yolo_pose_tag_detector/status`
- debug image topic: `/yolo_pose_tag_detector/debug_image`
- log: `/tmp/yolo_pose_tags_remote.log`

Useful overrides:

```bash
SLAM_REMOTE_YOLO_POSE_MODEL=~/catkin_ws/src/slam/pose_best.pt \
SLAM_REMOTE_YOLO_POSE_IMGSZ=640 \
SLAM_REMOTE_YOLO_POSE_HZ=5.0 \
SLAM_REMOTE_ROS_IP=<laptop_ip> \
slam yolo-tags-remote
```

Other supported overrides:

```bash
SLAM_REMOTE_YOLO_POSE_IMAGE_TOPIC=/camera/color/image_raw
SLAM_REMOTE_YOLO_POSE_CAMERA_INFO_TOPIC=/camera/color/camera_info
SLAM_REMOTE_YOLO_POSE_OUTPUT=/tag_detections
SLAM_REMOTE_YOLO_POSE_DEBUG_IMAGE=/yolo_pose_tag_detector/debug_image
SLAM_REMOTE_YOLO_POSE_PUBLISH_DEBUG_IMAGE=false
SLAM_REMOTE_YOLO_POSE_MIN_BOX_CONF=0.50
SLAM_REMOTE_YOLO_POSE_MIN_KEYPOINT_CONF=0.45
SLAM_REMOTE_YOLO_POSE_MIN_STABLE_FRAMES=5
SLAM_REMOTE_YOLO_POSE_EMA_ALPHA=0.35
SLAM_REMOTE_YOLO_POSE_CLASS_ID_TO_TAG_ID='1:1001,2:1002'
```

---

## 3. Laptop Network Setup

If the laptop is on the Jetson hotspot or another network that is not the robot
LAN, route robot-LAN traffic through the Jetson hotspot IP:

```bash
sudo ip route replace 192.168.0.0/24 via <jetson_hotspot_ip>
```

Then confirm the route to the Pi ROS master:

```bash
ip route get 192.168.0.200
```

Use the `src` address from that output as the laptop `ROS_IP` if an override is
needed. Example:

```text
192.168.0.200 via 10.40.187.185 dev wlan0 src 10.40.187.42
```

Here the laptop ROS IP is `10.40.187.42`, not `192.168.0.200` and not
necessarily the first address from `hostname -I`.

Manual ROS environment for generic remote tools:

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=$(ip route get 192.168.0.200 | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')
unset ROS_HOSTNAME
```

For `slam yolo-tags-remote`, the alias does this automatically:

- sources `$SLAM_WS/devel/setup.bash`
- sets `ROS_MASTER_URI=http://$SLAM_PI:11311`
- infers `ROS_IP` from `ip route get "$SLAM_PI"`
- unsets stale `ROS_HOSTNAME`

Run it from the laptop:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam yolo-tags-remote
```

If automatic `ROS_IP` inference is wrong:

```bash
SLAM_REMOTE_ROS_IP=<laptop_ip_from_route_src> slam yolo-tags-remote
```

Preflight from the laptop:

```bash
ping -c1 <jetson_hotspot_ip>
ip route get 192.168.0.200
ping -c1 192.168.0.200
ping -c1 192.168.0.101
rostopic list | head
```

If the laptop has a firewall, it must allow inbound ROS TCP connections from the
Jetson/Pi side. ROS1 uses XMLRPC plus dynamically allocated TCPROS ports, so a
strict firewall can make topics appear in `rostopic list` while message delivery
still fails.

---

## 4. Remote RViz / Read-Only Tools

For RViz only, the laptop does not need the full `slam` package if the RViz
config uses only standard message displays. Copy a config if needed:

```bash
scp ee478_team2@<jetson_hotspot_ip>:~/catkin_ws/src/slam/rviz/apriltag_rtabmap.rviz /tmp/
rviz -d /tmp/apriltag_rtabmap.rviz
```

For the total mission view, start marker publishing on the Jetson:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam mission-pub
```

Then run RViz on the laptop:

```bash
scp ee478_team2@<jetson_hotspot_ip>:~/catkin_ws/src/slam/rviz/mission.rviz /tmp/
rviz -d /tmp/mission.rviz
```

Useful read-only checks from the laptop:

```bash
rostopic list
rostopic hz /camera/color/image_raw
rostopic hz /camera/color/camera_info
rostopic echo -n1 /rtabmap/info
tf_echo map base_link
```

Clock skew can break TF. Compare laptop and Jetson time if RViz TF looks stale:

```bash
date
ssh ee478_team2@<jetson_hotspot_ip> date
```

---

## 5. Remote YOLO Bring-Up

Recommended order:

```bash
# Jetson
cd ~/catkin_ws/src/slam
./net_init.sh
source scripts/slam_aliases.sh
slam env
slam cam
slam rtab
```

```bash
# Laptop
sudo ip route replace 192.168.0.0/24 via <jetson_hotspot_ip>
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam yolo-tags-remote
```

Expected startup printout from the laptop includes:

```text
remote yolo pose square tags -> /tmp/yolo_pose_tags_remote.log
  MASTER=http://192.168.0.200:11311  IP=<laptop_ip>
  route to Pi: 192.168.0.200 via <jetson_hotspot_ip> ...
  model: .../pose_best.pt
  image: /camera/color/image_raw
  camera info: /camera/color/camera_info
  imgsz: 640
  output: /tag_detections (RTAB consumes /tag_detections)
  debug image: /yolo_pose_tag_detector/debug_image
```

Check process state on the laptop:

```bash
ps -eo pid,ppid,comm,etime,rss,args | \
  grep -E '[r]oslaunch slam yolo_pose_tag_detector|[y]olo_pose_tag_detector.py'
```

Check ROS state:

```bash
rosnode list | grep yolo
rosnode info /yolo_pose_tag_detector
rostopic hz /yolo_pose_tag_detector/status
rostopic hz /tag_detections
rostopic info /tag_detections
```

Check logs:

```bash
tail -n 80 /tmp/yolo_pose_tags_remote.log
tail -f /tmp/yolo_pose_tags_remote.log
```

The detector publishes an empty `AprilTagDetectionArray` even when no store tag
is detected. Therefore, if `/tag_detections` has `no new messages`, the detector
is usually not processing frames, not connected, or blocked before publishing.

---

## 6. `no new messages` Debug Ladder

Start on the laptop where `slam yolo-tags-remote` is running.

1. Verify the node exists:

```bash
rosnode list | grep yolo
rosnode info /yolo_pose_tag_detector
```

If there is no node, check `/tmp/yolo_pose_tags_remote.log` for model or Python
dependency failures.

2. Verify both camera topics exist and publish:

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/color/camera_info
```

`image_raw` alone is not enough. The detector waits for
`/camera/color/camera_info` before processing frames.

3. Verify detector status:

```bash
rostopic hz /yolo_pose_tag_detector/status
rostopic echo -n1 /yolo_pose_tag_detector/status
```

If status is missing, read the log:

```bash
tail -n 120 /tmp/yolo_pose_tags_remote.log
```

Common log meanings:

| Symptom | Meaning | Fix |
|---|---|---|
| `waiting for /camera/color/camera_info` | camera info missing | start/restart camera, or override camera info topic |
| model file missing | laptop does not have `pose_best.pt`/`.onnx` | copy model or set `SLAM_REMOTE_YOLO_POSE_MODEL` |
| `ImportError` / module missing | laptop Python env incomplete | install missing ROS/Python dependency |
| no log after startup | process died before roslaunch wrote useful output | inspect `ps`, rerun in a fresh shell |

4. Verify route and ROS IP:

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
ip route get 192.168.0.200
```

If `ROS_IP` does not match the `src` field from `ip route get 192.168.0.200`,
restart with:

```bash
SLAM_REMOTE_ROS_IP=<route_src_ip> slam yolo-tags-remote
```

5. Check from the Jetson too if RTAB does not receive landmarks:

```bash
rostopic info /tag_detections
rostopic hz /tag_detections
```

If the laptop can echo status but Jetson cannot receive `/tag_detections`, the
laptop is probably advertising an unreachable `ROS_IP` or blocking inbound ROS
TCP connections with a firewall.

---

## 7. Shutdown

Stop remote YOLO on the laptop that launched it:

```bash
pkill -INT -f 'roslaunch slam yolo_pose_tag_detector'
pkill -INT -f 'yolo_pose_tag_detector.py'
```

`slam down` on the laptop also kills the local remote YOLO process, but it is
broader and may also stop local RViz or other local slam helper processes.
Prefer the two `pkill -INT` lines above when only remote YOLO should stop.

Stop the Jetson perception stack from the Jetson:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam down
```

The RealSense camera must be stopped with SIGINT and stopped last. Do not use
SIGKILL or `rosnode kill` on the RealSense nodelet.

---

## 8. Quick Decision Table

| Question | Answer |
|---|---|
| Do I need to time remote YOLO with camera startup? | No. It can wait for camera topics. |
| Can I copy only `slam_aliases.sh` to the laptop? | No. Remote YOLO needs the `slam` package, launch file, detector script, model, and dependencies. |
| Should laptop remote use ONNX? | Not required. Default is `pose_best.pt`; ONNX is fallback/override. |
| Which IP goes in `ROS_IP`? | The laptop IP reachable from the robot network; use the `src` from `ip route get 192.168.0.200`. |
| Should Jetson run `slam yolo-tags` while laptop runs remote YOLO? | No, not unless intentionally testing duplicate publishers. |
| What topic does RTAB consume? | `/tag_detections`. Remote YOLO publishes the same topic. |
| What if `/tag_detections` says `no new messages`? | Check node, log, `camera_info`, route `src` IP, and firewall. |
