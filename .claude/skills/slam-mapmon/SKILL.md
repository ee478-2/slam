---
name: slam-mapmon
description: Monitor a running rtabmap RGB-D SLAM session on the real robot — map node growth, loop closures, visual-odometry good/lost frames, and travelled distance. Use while driving to build a map, to check "is the map growing / did a loop close / is VO surviving", or "맵 잘 쌓이나 봐줘".
---

# slam-mapmon — watch a live rtabmap session

Samples the running rtabmap (from `slam-bringup`) for a fixed window and reports map
growth, loop closures, and VO health. Assumes `/rtabmap/*` topics are up and
`ROS_MASTER_URI` / `ROS_IP` are set (source `~/catkin_ws/devel/setup.bash` first).

## Gotchas this skill exists to encode

- **Node count = `len(MapData.graph.poses)`, NOT `len(MapData.nodes)`.** `MapData.nodes`
  is published incrementally (only newly-added node data per message), so a naive
  `len(msg.nodes)` reads ~1 even with dozens of nodes. The rtabmap node log line
  `... (local map=NN, WM=NN)` is the same truth.
- **Odometry lost flag = `msg.pose.covariance[0] >= 9998`** on `/rtabmap/odom`
  (covariance lives on `PoseWithCovariance`, i.e. `msg.pose.covariance`, **not**
  `msg.pose.pose.covariance` — that AttributeErrors every frame and floods the log).
- `rostopic hz` never averages against the cross-machine master — always measure with
  a rospy subscriber like below.

## Monitor (default 60 s; adjust the `< 60` and `timeout`)

```bash
cd ~/catkin_ws && source devel/setup.bash
timeout 63 python3 - <<'PY'
import rospy, time, math
from rtabmap_msgs.msg import MapData, Info
from nav_msgs.msg import Odometry
rospy.init_node("slam_mapmon", anonymous=True, disable_signals=True)
s = {"nodes": 0, "lc": 0, "lost": 0, "good": 0, "p": None, "travel": 0.0}
def md(m):  s["nodes"] = len(m.graph.poses)                 # TRUE node count
def inf(m):
    if getattr(m, "loopClosureId", 0) > 0: s["lc"] += 1
def od(m):
    if m.pose.covariance[0] >= 9998: s["lost"] += 1         # VO lost
    else: s["good"] += 1
    p = m.pose.pose.position
    if s["p"]: s["travel"] += math.dist((p.x, p.y, p.z), s["p"])
    s["p"] = (p.x, p.y, p.z)
rospy.Subscriber("/rtabmap/mapData", MapData, md)
rospy.Subscriber("/rtabmap/info", Info, inf)
rospy.Subscriber("/rtabmap/odom", Odometry, od)
t0 = time.time()
while time.time() - t0 < 60:
    time.sleep(10)
    el = int(time.time() - t0)
    pos = "-" if not s["p"] else f"({s['p'][0]:.2f},{s['p'][1]:.2f})"
    print(f"t+{el:2d}s nodes={s['nodes']:3d} loopclose={s['lc']} "
          f"VO good/lost={s['good']}/{s['lost']} travel~{s['travel']:.2f}m pos={pos}",
          flush=True)
print(f"FINAL nodes={s['nodes']} loopclose={s['lc']} VOlost={s['lost']} travel~{s['travel']:.2f}m",
      flush=True)
PY
grep -i 'loop closure' /tmp/rtabmap.log | tail -3 || echo "(no loop closures in log)"
```

## One-shot snapshot (no window — quick "where are we")

```bash
cd ~/catkin_ws && source devel/setup.bash
timeout 8 python3 - <<'PY'
import rospy, time
from rtabmap_msgs.msg import MapData
from sensor_msgs.msg import PointCloud2
rospy.init_node("snap", anonymous=True, disable_signals=True)
s = {"poses": -1, "pts": -1}
rospy.Subscriber("/rtabmap/mapData", MapData, lambda m: s.__setitem__("poses", len(m.graph.poses)))
rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, lambda m: s.__setitem__("pts", m.width*m.height))
time.sleep(6)
print("map nodes:", s["poses"], " cloud_map points:", s["pts"])
PY
```

## Reading it

- `nodes` rising while driving = mapping works. Flat at 1 while `travel` rises = odom
  is feeding but the mapping node isn't adding nodes → check for a wedged/LOST odom
  (see `slam-bringup` step 3) or restart cleanly via `slam-shutdown` + `slam-bringup`.
- `loopclose > 0` = a previously-mapped area was re-recognised. To trigger one, drive a
  loop back to an earlier viewpoint; forward-only exploration won't produce them.
- `VO lost` should stay 0 on slow driving; a few isolated losses that recover are fine
  (ResetCountdown=1). Sustained losses = driving too fast / textureless view.
