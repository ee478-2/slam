# Progress — index

Single source of truth for "where are we." **Detail lives in `docs/progress/<month>.md`;
this file stays an index + current open items.** Newest first.

> Convention going forward: add a new dated entry to the **current month's**
> `docs/progress/YYYY-MM.md` (newest at top), then add one index row here. Keep this
> file lean — links and one-line hooks, no long detail.

## Active / Open (real robot, `slam` package)

- **`go_to_goal_avoid` tuning** — in-place yaw "wari-gari" oscillation on hardware; an
  oscillation fix was tried then reverted. Still being tuned.
- **"go to store_N" — step 2** = AprilTag *global* localization: rtabmap's map frame ≠
  `config/global_map.yaml` room frame, so absolute/global localization is not wired yet.
- **Loop closure via tag re-observation** — not yet captured (drive back to a seen tag).
- **Wheel odometry calibration / true encoders** — `/wheel/odom` now exists as
  command-integrated odom; chassis still publishes no encoder/tick feedback.
- **Chassis wheel-dropout** — root-caused to the stock Pi `chassis_control_node.py`
  (th-race + `slow_velocity` ramp); real fix is Pi-side, deliberately NOT applied (user's call).
- **USB3 cable** — D435 still on USB2; 640x480@15, IR/IMU-off is the validated fallback.
- Optional: prune the now-unused gazebo build-deps (`find_package(gazebo)`).

## Log index

### 2026-06 — real-robot `slam` → [`docs/progress/2026-06.md`](progress/2026-06.md)
- **2026-06-06** `/wheel/odom` command odometry — `db5668a` · added a
  command-integrated wheel odom topic from `/chassis_control/set_velocity`;
  true encoder feedback remains Pi-side/unexposed.
- **2026-06-06** mission RViz label cleanup — `4b396af` · removed raw xy
  coordinate text from store/signboard/robot marker labels while keeping marker positions.
- **2026-06-06** remove temporary signboard HUD wiring — `8800590` · backed
  `llm_agent` signboard recognition back out of `slam up`; launch it explicitly when needed.
- **2026-06-06** temporary signboard HUD wiring — `b4aa6d8` · `slam up`
  now starts `llm_agent` signboard recognition so mission RViz gets `/signboards/detections_image`.
- **2026-06-06** mission RViz relaunch fix — `17aa5b5` · `slam mission`
  now runs `slam env`; new `slam mission-pub` starts `/mission/markers` only for laptop RViz.
- **2026-06-06** mission RViz total view — `4b82198` · `/mission/markers`
  publishes global stores/signboards/status from `global_map.yaml`; `rviz/mission.rviz`
  overlays RTAB-Map trajectory, AprilTag images/landmarks, and grasp/inventory context.
- **2026-06-04** remove local_costmap_generator component — `6037cf8` · deleted the
  dormant heightmap node/nodelet (never in `slam up`); obstacle cloud + local costmap
  come from sibling `local_planner`, so no functional change. Pruned `pluginlib` dep.
- **2026-06-03** RViz config for local_planner goto+avoidance — `381bf59` ·
  `rviz/local_planner_goto.rviz`: robot pose/goal/path/obstacle-cloud/local-costmap;
  topics read from source, fixed frame `odom`, camera follows `base_link`.
- **2026-06-03** /odom: launch localization_manager with the stack — `95721c5` ·
  `slam loc` wired into `slam up`; `/odom` now publishes (the node just wasn't being run).
- **2026-06-03** real-robot nav scripts + chassis-bug root cause + sim/eval cleanup —
  `9107522`/`73c722e`/`247aa13`/`24bd833` · drive_straight, go_to_goal(_avoid), Pi chassis
  th-race found, Gazebo + eval trees removed.
- **2026-06-03** slam shortcuts + remote RViz + down→up camera race fix — `1b41f6c` ·
  `slam_aliases.sh`, remote RViz via dual-homed Jetson, device-busy race fix.
- **2026-06-03** YOLO perception + AprilTag-as-rtabmap-landmark — `fcc25b2`/`2a333af` ·
  YOLO live on Orin; tags 7/11 registered as rtabmap landmarks; global_map not wired.
- **2026-06-03** Claude skills for the perception lifecycle — `64fb713` ·
  slam-bringup/mapmon/shutdown skills; teleop input-lag fix + speed cap 30→75.
- **2026-06-03** rtabmap real map-building VERIFIED + VO fail-fast · 50 nodes / 67k pts;
  the `Odom/ResetCountdown=1` fix that stops VO wedging on the USB2 15 fps stream.
- **2026-06-03** Jetson perception test PASSED (RGB-D + rtabmap over USB2) · the USB2
  640x480@15 IR/IMU-off recipe; SIGINT-only camera shutdown (USB wedge).
- **2026-06-03** Workspace restructure → `slam` package + Claude-asset migration ·
  llm-skill split into slam / llm_agent / manipulation_control; scope + remote-git policies.
- **2026-06-03** A4 Jetson bring-up BLOCKED on D435 USB2 cable [reconstructed] ·
  SuperSpeed link never came up = USB2-only/charge-only cable; physical swap needed.

### 2026-05 — A4 agentic stack + HW3-3 sim → [`docs/progress/2026-05.md`](progress/2026-05.md)
- **2026-05-20** A4 parallel-work git split: perception vs agent [reconstructed] — `af534b7`.
- **2026-05-20** A4 real-robot enablement + teammate-stack discovery — `876322f`/`c6f624d`/
  `4518bcd`/`5464191`/`c30a9ca`/`efedb93` · two-computer system, D435 on Jetson, AprilTag-primary plan.
- **2026-05-20** A4 Topic 2 — full LLM-agentic stack landed — `e2250b2`/`f1aad97` ·
  localization_manager, apriltag_pipeline, signboard_recognition, agent_interface, missions.
- **2026-05-03** HW3-3 DWA tuning round — `f4894ac`/`aee9afa`/`5391dca`/`26c435f`.
- **2026-05-02** HW3-3 sim drift hunt: D435 calib + BA + viz — `f9a5ece`/`d34be11`/`d80bd8f`/
  `8ac85a9`/`8cf1063` · the RGB↔depth FOV-match drift win (1.54 m → 0.17 m).
- **2026-05-02** HW3-3 sim stack: RTAB + strafe-yaw + true GT — `fb23d5a`.
- **2026-05-02** HW3-3 setup: workspace consolidation + textured walls — `4e03433`.
- **2026-05-01** extended sweep 3 bags × 5 values @ rate=1.0 · run-to-run variance on slam2.
- **2026-05-01** Option C — drop ResetCountdown=1 from submitted launch — `1bdb523`.
- **2026-05-01** failed tuning round + baseline lock-in — `9ab5598` · Force3DoF-on-optical-frame trap.
- **2026-05-01** ResetCountdown sweep {0,1,5,10,30} · V=1 fail-fast wins by a large margin.
- **2026-05-01** +3 TUM bags, generalized `eval.launch` for headless batch.

### 2026-04 — Assignment 2 / TUM RTAB-Map → [`docs/progress/2026-04.md`](progress/2026-04.md)
- **2026-04-27** harden VO params for TUM pioneer_360 — `362309a` · MinInliers 12,
  MaxFeatures 1500, ResetCountdown 1; frame_id→optical + bag-TF strip. (+ historical HW2 `Open`)
