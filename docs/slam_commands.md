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
slam up            # env + camera + rtabmap + apriltag
slam teleop        # drive
slam rviz          # viewer on :1   (slam rviz yolo for the YOLO view)
slam check         # quick status:  nodes / landmarks / tags seen
slam mon           # 15s map+VO monitor
slam down          # SIGINT teardown (camera stopped last)
slam help          # full list
```
Individual steps: `slam env | cam | rtab | tags | yolo`. Override the Pi IP / GUI
display with `SLAM_PI=...`, `SLAM_DISPLAY=:0` before sourcing.

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
- Defaults: `--delete_db_on_start` (fresh map each run) and `--Odom/ResetCountdown 1`
  (fail-fast VO recovery — without it a broken frame leaves odom LOST forever).
- To **keep** a map across restarts: add `rtabmap_args:=""`.
- Verify:
```bash
rosparam get /rtabmap/rgbd_odometry/Odom/ResetCountdown   # 1
grep 'Odom: quality' /tmp/rtabmap.log | tail -1           # quality ~200-480, not 0
```

---

## 3. AprilTag detection + rtabmap landmarks

```bash
setsid roslaunch slam apriltag_realsense.launch > /tmp/apriltag.log 2>&1 &
```
- Publishes `/tag_detections` (+ `/tag_detections_image`). `rtabmap_realsense.launch`
  already subscribes `/tag_detections` and adds tags as **graph landmarks**
  (landmark graph id = `-(tag_id)`; poses on `/rtabmap/landmarks`, frame `map`).
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
> NOTE: this does NOT localize against `config/global_map.yaml` — rtabmap works in
> its own map frame. Global-frame absolute localization is not wired yet.

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

---

## 5. teleop (real keyboard required)

```bash
rosrun slam teleop_keyboard.py
```
`w/s` fwd/back · `a/d` strafe · `j/l` rotate · `z/c` slower/faster · SPACE/x stop · `q` quit.
Hold to move (auto-stops ~0.4 s after release). Drive **slowly**, esp. in-place rotation
(15 fps VO breaks on fast yaw). Max linear 75; start ~15.

---

## 6. Viewers on the GUI desktop (DISPLAY :1)

Attach a viewer to the **already-running** stack (do NOT relaunch with `rviz:=true` —
that restarts rtabmap and wipes the map):
```bash
# AprilTag + rtabmap landmarks + cloud + path:
DISPLAY=:1 setsid rviz -d ~/catkin_ws/src/slam/rviz/apriltag_rtabmap.rviz > /tmp/rviz.log 2>&1 &
# YOLO debug image:
DISPLAY=:1 setsid rviz -d ~/catkin_ws/src/slam/rviz/yolo.rviz > /tmp/rviz.log 2>&1 &
# rtabmap's own viewer (shows landmarks in the graph):
DISPLAY=:1 ROS_NAMESPACE=rtabmap setsid rosrun rtabmap_viz rtabmap_viz _frame_id:=camera_link &
```
Confirm rviz is up without self-matching your command:
```bash
ps -eo pid,comm | awk '$2=="rviz"{print "rviz pid="$1}'
```

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
sleep 6
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
