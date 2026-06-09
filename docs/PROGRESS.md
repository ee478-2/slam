# Progress — index

Single source of truth for "where are we." **Detail lives in `docs/progress/<month>.md`;
this file stays an index + current open items.** Newest first.

> Convention going forward: add a new dated entry to the **current month's**
> `docs/progress/YYYY-MM.md` (newest at top), then add one index row here. Keep this
> file lean — links and one-line hooks, no long detail.

## Active / Open (real robot, `slam` package)

- **`go_to_goal_avoid` tuning** — in-place yaw "wari-gari" oscillation on hardware; an
  oscillation fix was tried then reverted. Still being tuned.
- **AprilTag global localization verification** — `a6783dd` wires
  `global_map -> map` and global `/odom`; `4558bf3` ignores small in-plane
  AprilTag paper twist when solving anchor yaw; `f5cb7b7` holds established yaw
  for single-landmark corrections; `94dd9c9` requires an initial/previous yaw
  reference before trusting a single tag; `23ff4a3` adds final fused-pose jump
  guarding before `/robot_pose` and `/odom`. Needs live robot validation
  against real tag observations.
- **YOLO pose RTAB landmark validation** — `6208574` publishes 15 cm square-tag
  YOLO keypoints as RTAB-compatible landmarks; `ffd2087` adds soft
  `store1..store8` global-map status mapping; `f8d15ca` validates ONNX runtime
  compatibility; `ef42316` made ONNX the default; `7c3e1a6` makes
  `pose_best.engine` the TensorRT default; `602f388` raises confidence gates,
  requires 5 stable frames, and softens YOLO covariance. Needs live validation
  of TensorRT runtime, class-to-tag ID mapping, and RTAB landmark registration.
- **Loop closure via tag re-observation** — not yet captured (drive back to a seen tag).
- **Wheel odometry calibration / true encoders** — `/wheel/odom` now exists as
  command-integrated odom; chassis still publishes no encoder/tick feedback.
- **Chassis wheel-dropout** — root-caused to the stock Pi `chassis_control_node.py`
  (th-race + `slow_velocity` ramp); real fix is Pi-side, deliberately NOT applied (user's call).
- **USB3 cable** — D435 still on USB2; 640x480@15, IR/IMU-off is the validated fallback.
- Optional: prune the now-unused gazebo build-deps (`find_package(gazebo)`).

## Log index

### 2026-06 — real-robot `slam` → [`docs/progress/2026-06.md`](progress/2026-06.md)
- **2026-06-10** Localization jump guard — `23ff4a3` · holds same-source fused
  pose jumps and requires consecutive consistent source/frame corrections.
- **2026-06-10** YOLO landmark trust reduction — `602f388` · raises YOLO
  confidence/keypoint thresholds, requires 5 stable frames, and increases
  published covariance while noting the RTAB global-variance caveat.
- **2026-06-09** AprilTag yaw reference requirement — `94dd9c9` · waits for
  `/initialpose` or a previous anchor before one visible tag can set global yaw.
- **2026-06-09** AprilTag single-landmark yaw hold — `f5cb7b7` · keeps
  established `global_map -> map` yaw when one visible tag corrects x/y, so
  tilted/twisted tag pose yaw cannot keep rotating the map.
- **2026-06-09** Remote operation guide — `833d835` · adds
  `docs/remote.md` for laptop ROS networking, remote YOLO, process checks, and
  troubleshooting.
- **2026-06-09** YOLO remote PyTorch default — `a512ef2` · defaults laptop
  offload to `pose_best.pt`, with ONNX fallback and conditional remote `imgsz`.
- **2026-06-09** YOLO remote pose input size — `2a46034` · adds
  `SLAM_REMOTE_YOLO_POSE_IMGSZ` so ONNX fallback/override can match fixed input
  size.
- **2026-06-09** YOLO remote pose launcher — `9d7c319` · adds
  `slam yolo-tags-remote` for laptop-side inference against the Jetson ROS
  master.
- **2026-06-09** YOLO pose live topic validation — runtime check confirms
  camera input, detector status, debug image, and two published YOLO store tags.
- **2026-06-09** YOLO TensorRT engine default — `7c3e1a6` · switches detector,
  launch, `slam yolo-tags`, and docs to use `pose_best.engine` by default, with
  ONNX/PT fallback if the engine is missing.
- **2026-06-09** AprilTag in-plane heading twist — `4558bf3` · uses tag plane
  normal heading, bounded by the Euler-yaw prior, to stop small rotated tags
  from rotating `global_map -> map`.
- **2026-06-09** YOLO ONNX default weights — `ef42316` · switches detector,
  launch, `slam yolo-tags`, and docs to use `pose_best.onnx` by default.
- **2026-06-09** YOLO ONNX pose runtime validation — `f8d15ca` · validates
  `pose_best.onnx` format/runtime compatibility and forces exported models to
  load as pose in the detector.
- **2026-06-09** Initial-pose anchor launch args — `5d695f9` · restores
  `initial_pose_topic` / `enable_initial_pose_anchor` compatibility for
  `robot_master/mission.launch` without reintroducing the AprilTag normal-heading fix.
- **2026-06-09** Revert AprilTag normal heading fix — `5695c12` · restores the
  pre-`a327ee2` global-anchor yaw path after the map still appeared inverted.
- **2026-06-09** AprilTag observer-facing normal heading — `a327ee2` · derives
  anchor yaw from the observed tag plane normal facing the camera/base, avoiding
  180° front/back convention flips and reporting normal tilt diagnostics.
- **2026-06-09** YOLO pose detection visualization — `99c209a` · adds
  `/yolo_pose_tag_detector/debug_image` overlay for keypoints, horizontal edges,
  confidence, method, and the 3-frame publication gate.
- **2026-06-09** YOLO soft global-map store mapping — `ffd2087` · maps
  `store1..store8` to `global_map.yaml` store order in detector status only;
  YOLO global anchoring remains disabled by default.
- **2026-06-09** YOLO pose RTAB landmarks — `6208574` · publishes 15 cm
  square-tag YOLO keypoints as RTAB-compatible landmarks with 3-frame gating,
  EMA smoothing, and horizontal-weighted pose for occluded vertical labels.
- **2026-06-08** Revert AprilTag front/side weighting — `0c822dc` · removes
  side-axis damping after it degraded localization; k-frame + smoothing remain.
- **2026-06-08** AprilTag front/side anchor weighting — `a87230e` · trusts
  signboard front-axis corrections while damping side-axis corrections.
- **2026-06-08** RTAB DB capture guidance — `cfce332` · documents
  multi-pass reference DB capture and when to rebuild the DB.
- **2026-06-08** RTAB AprilTag landmark variance — `d0fd7a8` · raises
  tag translation variance to `0.005` while keeping rotation ignored.
- **2026-06-08** AprilTag anchor smoothing — `e1b717d` · adds
  short-window median/circular-mean smoothing without hard jump rejection.
- **2026-06-08** AprilTag anchor stabilization — `cc64563` · requires
  consecutive stable signboard detections before updating `global_map -> map`.
- **2026-06-08** RTAB feature-DB workflow — `6e0c816` · adds
  saved-DB mapping/localization shortcuts as a no-label alternative to YOLO landmarks.
- **2026-06-07** camera-only storefront collection — `93c37b6` · adds
  `slam collect-cam` so YOLO data capture can run without RTAB/AprilTags/localization.
- **2026-06-07** storefront YOLO data pipeline — `4e98232` · adds
  teleop image collection plus CVAT-to-Ultralytics prep and YOLO11 training/export code.
- **2026-06-06** flat-ground RTAB prior — `94976ba` · adds
  `Reg/Force3DoF=true` in `base_link` coordinates to suppress roll/pitch/z drift.
- **2026-06-06** revert planner-safe `/odom` split — `f55cb22` · restores
  selected fused pose publishing on `/odom` per user request.
- **2026-06-06** planar AprilTag anchor solve — `1fb2d38` · computes
  `global_map -> map` directly in x/y/yaw so the detected tag point lands on the configured signboard.
- **2026-06-06** detected-tag ID global anchor matching — `70c5fa5` · anchors
  from `/tag_detections.id -> SIGNBOARDxx` instead of relying on signboard TF names.
- **2026-06-06** planar AprilTag global anchor — `77f69de` · constrained
  `global_map -> map` to x/y/yaw so RTAB does not tilt/flip in `mission.rviz`.
- **2026-06-06** AprilTag global RTAB anchor + `/odom` — `a6783dd` · added
  `global_map -> map` from known signboard tags and made `/odom` global when anchored.
- **2026-06-06** localization `/odom` frame fix — `e801a98` · preserved
  RTAB-Map's source odom frame instead of relabeling local odom as `map`.
- **2026-06-06** arm home default pose update — `018157c` · changed the
  `slam arm-home` / `slam up` default to `0 0.8 -3. -0.5 0`.
- **2026-06-06** arm home on `slam up` — `d41a9b6` · added a
  `slam`-side one-shot arm-home publisher and wired it into the startup shortcut.
- **2026-06-06** RTAB-vs-wheel odom RViz — `de38748` · added a
  no-AprilTag comparison view: RTAB path/pose in green, `/wheel/odom` in red.
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
