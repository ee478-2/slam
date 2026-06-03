# Progress

## 2026-06-03 (YOLO perception + AprilTag-as-rtabmap-landmark, real robot)

Same on-Jetson session, after the map-building pass. Tested the two remaining
perception branches live, with viewers on the local `:1` desktop.

### YOLO object detection — pipeline verified (`fcc25b2`)
- Ran `manipulation_control/scripts/object_detection.py` on the live camera
  (it's outside `slam/` scope — executed with permission). Model `best.pt`
  classes = **`cup`, `drink`, `hamburger`, `medicine`** (the 4 grasp objects).
  torch CUDA available → runs on the Orin GPU; `/detected_objects/debug_image`
  published at ~6.7 Hz (inference_hz=8). No object was in view to score a
  detection, but the RGB-D → 3D → publish path is up.
- Ran with `_base_frame:=camera_link` so the base-frame transform uses the
  camera TF only (no robot/`base_link` TF exists with rtabmap off). Added
  `rviz/yolo.rviz` (Image on `/detected_objects/debug_image`).

### AprilTag detection + rtabmap landmarks — VERIFIED (`2a333af`)
- The sim `apriltag_pipeline.launch` is wired for the Gazebo camera and the
  sim world config, and its localization node uses `tag_config_name=
  2025/re540_simulation`. `localization_manager` is **disabled from the build**
  (`*.legacy`). So wrote `localization_manager/launch/apriltag_realsense.launch`
  — `apriltag_ros` continuous_detection on the real D435 (tag36h11, our bundle
  `tags.yaml`), correct real-camera remaps, no sim localization node.
- `rtabmap_realsense.launch` already subscribes `/tag_detections`. With both up
  and a short drive: **tags id 7 and 11 detected → rtabmap registered them as
  landmarks** (graph ids `-7`, `-11`; rtabmap stores a landmark as `-(tag_id)`).
  `/rtabmap/landmarks` (PoseArray) held the 2 landmark poses and **persisted
  them in the map frame after the tags left view**. Added
  `rviz/apriltag_rtabmap.rviz` (tag image + cloud_map + landmarks + path).

### Answered: are we localizing against `config/global_map.yaml`? — NO
- `global_map.yaml` is our semantic/global deliverable (stores, signboards,
  coordinate-source pointers). Nothing in the live stack reads it: rtabmap
  localizes in its **own `.db` map frame**; apriltag_ros just detects;
  the `apriltag_localization` node is sim-configured and its package is
  disabled. Global-frame absolute localization (real tag world poses matching
  `global_map.yaml`) is **not wired** — that's the open integration task.

### Non-obvious findings
- **`/rtabmap/mapPath` includes landmark nodes as path vertices**, not just
  robot poses. Confirmed: landmark coords (z≈0.2) appear as mapPath vertices
  among the ground-level robot poses (z≈0). So the RViz Path display draws
  lines spiking up to each landmark — looks like "lines connecting landmarks"
  but it's one Path with landmark detours (graph-id ordering artifact, not a
  bug, not an observation link). Robot's real trajectory = the z≈0 portion.
- **`pgrep -f` / `pkill -f` self-match their own command line** (the pattern
  string is literally in the checker's argv), giving false "process UP" reads
  and ineffective kills. Use `ps -eo pid,comm | awk '$2=="rviz"'` (comm field,
  no args) or a bracketed pattern on `ps` output. Cost real time this session.
- RealSense **camera nodelet is SIGINT-only** (USB wedge); YOLO/rviz/rtabmap
  nodes are safe to SIGTERM/KILL — only the camera has the wedge hazard.

### Open
- Remote viewing: RViz over SSH options scoped (X11-fwd / RViz-on-client via
  ROS master / VNC tunnel to `:1` / Foxglove+rosbridge) — not set up yet.
- Loop closure via tag re-observation still not captured (drive back to a seen
  tag). Wheel `/odom` still unwired (pure visual VO). global_map absolute
  localization still unwired (above).

## 2026-06-03 (Claude skills for the perception stack lifecycle)

Captured the repeated real-robot bring-up/teardown — and its hard-won gotchas —
as three project Claude skills under `.claude/skills/` so the workflow is
reproducible across sessions:
- `slam-bringup` — preflight (Pi roscore/ARP, D435 USB) → camera (USB2 recipe) →
  rtabmap (ResetCountdown=1) → teleop → optional viz on `:1`.
- `slam-mapmon` — live map-node / loop-closure / VO good-lost / travel monitor.
  Encodes the two field gotchas: count nodes via `graph.poses` (not `nodes`),
  and read VO-lost from `msg.pose.covariance[0]` (not `pose.pose`).
- `slam-shutdown` — **SIGINT-only** teardown (consumers before camera) to avoid
  the D435 USB wedge.
- Also fixed teleop input lag (`64fb713`): drain the stdin buffer each tick
  instead of one byte per 50 ms (key auto-repeat was queuing), and raised the
  linear-speed cap 30 → 75 (2.5x).

## 2026-06-03 (rtabmap real map-building VERIFIED + VO fail-fast fix)

Resumed the on-Jetson real test: re-confirmed teleop, then ran rtabmap RGB-D
SLAM live and closed the previously-open "map building" item. Drove the base
with `scripts/teleop_keyboard.py` while rtabmap built the map over the USB2
camera link.

### Result — map building PASSED
- Drove ~2.9 m: graph grew to **50 map nodes**, `/rtabmap/cloud_map` =
  **67,803 points**. VO **0 lost frames / 425** during the slow drive.
- Camera held the validated USB2 recipe (color+depth 15.0 Hz, 0 libusb warnings,
  640x480, IR+IMU off).

### The fix that made it work — `Odom/ResetCountdown=1`
- First drive attempt **wedged**: VO went `quality=0`, `Not enough inliers
  0/20 ... between -1 and N`, pose frozen at origin, map stuck at 1 node, and
  the rtabmap node logged `Did not receive data since 5 seconds`. Camera was
  fine the whole time (still 15 Hz, 0 libusb) — the *odometry node* was lost.
- Root cause: `rtabmap_realsense.launch` never set `Odom/ResetCountdown`, so it
  used the rtabmap default **0 = never reset**. On the 15 fps USB2 stream a fast
  move / in-place rotation breaks VO; with reset disabled it stays LOST forever
  and the map stops growing. Same **V=1 fail-fast** lesson as the HW2/HW3 TUM
  tuning.
- Added `odom_args` pass-through (default `--Odom/ResetCountdown 1`) to
  `rtabmap_realsense.launch` → `rtabmap.launch`. Re-drove slowly: 0 lost frames,
  map grew cleanly.

### Non-obvious gotcha (cost time)
- `MapData.nodes` is published **incrementally** (only newly-added node data per
  message), so a naive `len(msg.nodes)` reads ~1 even while the map has dozens
  of nodes. Use **`len(msg.graph.poses)`** for the true node count (or the
  `WM=NN` figure in the rtabmap node log).
- The background launches were started with `setsid`, so the shell's `$!` /
  job-control PID is the detached parent, not the live roslaunch. Find the real
  PID with `pgrep -f 'roslaunch ...'`. **Stop with SIGINT only** (camera USB2
  wedge hazard) — camera roslaunch and rtabmap roslaunch PIDs in
  `/tmp/rs_camera.pid`, `/tmp/rtabmap.pid`.

### Still open
- **Loop closure** unverified — need to re-visit an already-mapped area to
  trigger one (none seen yet; only forward exploration so far).
- **AprilTag** detection still needs a physical tag in view.
- **Wheel odometry** not wired: ArmPi chassis publishes no `/odom`; rtabmap runs
  pure visual VO. Optional next step — publish wheel `/odom` and feed
  `rgbd_odometry` as a motion guess (`guess_frame_id`) for better survival
  through fast motion. Chassis node is outside `slam/` scope (reference/execute).

## 2026-06-03 (Jetson perception test PASSED — full RGB-D + rtabmap over USB2)

Resumed the on-Jetson (Orin Nano) real-robot perception test after the earlier
USB-cable blocker. The D435 is still on a USB 2.0 link (no USB3 cable
available), but we found a working low-bandwidth recipe and verified the entire
perception stack end to end against the Pi roscore (`192.168.0.200`).

### Results (all green)
- **Color stream:** 640x480 rgb8 @ **15.1 Hz**, 0 drops, 0 libusb warnings.
- **RGB-D (color + depth + aligned_depth):** both @ **15 Hz**, 0 libusb
  warnings — USB2 carries RGB-D fine at 640x480/15fps.
- **signboard + apriltag pipeline** (`llm_agent signboard_recognition_real`):
  `/tag_detections_image` processed at **15.1 Hz**, keeping up with the camera.
  No tag detections (none in view — needs a physical AprilTag).
- **rtabmap RGB-D SLAM** (`slam rtabmap_realsense.launch`, headless): VO
  `/rtabmap/odom` @ **14.3 Hz**, quality ~350-380 inliers, 0 lost frames,
  std dev ~0.0004 m, update 0.05 s / delay 0.12 s — real-time on the Jetson.

### Non-obvious findings (the reason this works)
- **USB2 CAN run RGB-D SLAM** — but only if you cut the stream set:
  `enable_depth:=true align_depth:=true enable_color:=true`,
  `color/depth 640x480 @ 15fps`, **IR + IMU off**. Bandwidth math: color
  ~13.8 MB/s + depth ~9.2 MB/s ≈ 23 MB/s < USB2's ~35 MB/s usable.
  `align_depth` is host-side CPU, costs **no** USB bandwidth. The default
  `rs_camera.launch` (848x480, all streams incl. IR) saturates USB2 → floods
  `libusb: Resource temporarily unavailable` and stalls.
- **Shut the camera down with SIGINT (Ctrl-C), never SIGKILL / `rosnode kill`.**
  Killing the realsense nodelet mid-stream wedges the D435 at the USB level
  (lsusb still shows it, but `rs-enumerate-devices` reports "No device
  detected"). A `USBDEVFS_RESET` ioctl (no sudo, device node is `crw-rw-rw-`)
  did **not** recover it — only a physical replug did. SIGINT to the actual
  roslaunch/nodelet PIDs releases the device cleanly.
- `rostopic hz` never prints an average against this cross-machine master;
  measure rates with a short rospy subscriber instead.

### Open
- Tag detection + map building are the only unverified steps, both physical:
  point the D435 at an AprilTag (id → store label via `/signboards/detections`)
  and move the camera slowly to accumulate `/rtabmap/cloud_map` + loop closures.
- RViz/rtabmap_viz not run (headless shell, no DISPLAY) — open a viewer on a
  machine with a display to watch the map live.
- USB3 cable still wanted for full 848x480 / higher-rate operation; 640x480 @
  15fps is the validated USB2 fallback for now.

## 2026-06-03 (Workspace restructure → `slam` package + Claude-asset migration)

The monolithic `llm-skill` package was split by the team into per-domain ROS
packages under `catkin_ws/src/`. Our perception/SLAM/nav work now lives in
`slam/`; the agent stack moved to `llm_agent/`; the arm stack is
`manipulation_control/`. The old `llm-skill/` directory was **deleted** from
`catkin_ws/src/`, taking the Claude operating assets (CLAUDE.md, docs/) with
it. This entry records recovering and re-homing them into `slam/`.

### New layout (`catkin_ws/src/`)
- `slam/` — AprilTagLocalization, localization_manager, my_rtabmap,
  graph_planner, local_costmap_generator, nexus_4wd_mecanum_simulator,
  re540_final_map. **Our edit scope.** Now its own git repo (`git init`).
- `llm_agent/` — agent_interface, gpt_llm_client, **signboard_recognition**
  (moved here from our old tree), missions, srv. Reference-only for us.
- `manipulation_control/` — arm/grasping. Own git repo (`main`, ee478-2). RO.
- `realsense-ros/` — Intel driver source (`ros1-legacy`). RO.

### Git / build state at migration time
- `slam` and `llm_agent` were **not under version control**; `catkin_ws/.git`
  was an empty/broken init (no HEAD). Only `manipulation_control` and
  `realsense-ros` were real repos.
- Whole workspace **rebuilt successfully** (`devel/setup.bash`, 15:02) across
  the new layout — slam, llm_agent, signboard_recognition, etc. all compile.

### Migration actions
- `git init` in `slam/`; added `.gitignore` (build/devel, pyc, *.bag/*.db;
  map deliverables kept).
- Recovered CLAUDE.md + docs/ from the surviving (broken-worktree) copy at
  `agent_ws/src/llm-skill` and copied them into `slam/`.
- Added to CLAUDE.md: **Workspace Scope Policy** (edits confined to `slam/`;
  outside is read/execute-with-permission but never modify) and **Git Remote
  Policy** (agent must never run push/pull/fetch/clone/remote — user manages
  remotes manually).

### Stale-data notes
- The recovered docs predate this restructure: `agent_contract.md` and
  `implementation.md` still describe the old `llm-skill` paths/package names.
  Flagged with a banner at the top of each; treat paths there as historical.
- Two PROGRESS entries below were reconstructed because the recovered copy was
  older than the deleted working tree: the 2026-05-20 "parallel-work git
  split" entry, and the 2026-06-03 "D435 USB2 cable blocker" entry (restored
  from agent memory — the original PROGRESS edit was lost with `llm-skill`).

### Open
- `signboard_recognition` lives in `llm_agent/` now — our perception tests
  touch a package outside our edit scope (reference/execute only).
- `slam` has no git remote yet (user will add manually). Migrate the Claude
  memory dir to the new project slug `…-catkin-ws-src-slam` so it loads when
  Claude runs from `slam/`.

## 2026-06-03 (A4 — Jetson perception bring-up test: BLOCKED on D435 USB2 cable) [reconstructed from memory]

First on-Jetson (Orin Nano, `192.168.0.101`) attempt to run the real-robot
perception stack. Read-only test; goal was to verify the RealSense path end to
end before signboard/apriltag/rtabmap. **Note:** this entry was reconstructed
from agent memory — the original was lost when `llm-skill/` was deleted.

### What works
- Pi roscore (`192.168.0.200`) reachable, stock Hiwonder nodes up. Workspace
  built. `realsense2_camera`, `rtabmap_launch`, `apriltag_ros` all installed.
- D435 seen at driver level (`rs-enumerate-devices`: serial 233622078598, FW
  5.17.0.10). `rs_camera.launch align_depth:=true` brings up the camera node +
  all color/aligned-depth topics; `camera_info` and a single `image_raw` frame
  echo OK.

### Blocker — D435 enumerates at USB 2.0 (480M)
- Color forced to **640x480** (ignored 848x480), continuous libusb
  `Resource temporarily unavailable` warnings, `rostopic hz` on
  `/camera/color/image_raw` never prints an average → frames stall. USB2
  bandwidth can't carry color+depth+aligned_depth.
- **Root cause is the cable, not port/hub/software.** `lsusb -t`: Bus 02
  (USB3 root) has a Realtek 4-port hub at 10000M with nothing downstream →
  Orin USB3.2 port + hub fine. Bus 01 (USB2) → same hub's USB2 face (480M) →
  D435 at 480M. Zero SuperSpeed links to the D435 → SuperSpeed link never came
  up = classic USB2-only / charge-only cable.

### Next
- Physical fix only. Swap to a genuine USB3 cable (SuperSpeed USB-C↔USB-A),
  ideally plug D435 directly into the Orin USB 3.2 port (bypass hub). Verify
  `lsusb -t` shows D435 on Bus 02 at 5000M+. Then resume rtabmap_realsense →
  848x480 → signboard / apriltag / rtabmap tests.

## 2026-05-20 (A4 — parallel-work git split: perception vs agent) [reconstructed]

To let a teammate edit the agent stack while perception is tested in parallel
without git tangles, work was split onto separate branches in the old
`llm-skill` repo. Perception (`signboard_recognition`, `AprilTagLocalization`,
`my_rtabmap`) and agent (`agent_interface`, `gpt_llm_client`) touched disjoint
dirs, so merges back were conflict-free. (This branch split is now superseded
by the 2026-06-03 package-level restructure above.)

- `af534b7` feat(perception): real-robot perception switched to the D435
  RealSense color cam (`/camera/color/image_raw`) instead of the mono USB cam
  + `camera_info_publisher`. Config/icon paths made package-relative.
  `rtabmap_realsense` gained `launch_realsense:=true`.

## 2026-05-20 (A4 — real-robot enablement + teammate-stack discovery)

Work after the sim stack landed: demo-prep polish, real-robot
(ArmPi Pro) support, and integrating context from two codebases the
user added to the repo.

### Commits

- `876322f` docs: `implementation.md` — full software-stack reference
  for the report (architecture, per-component, design decisions).
- `c6f624d` feat: manual-verification stack — `verify_teleop.launch`
  + `a4_verify.rviz` + `teleop.sh`. teleop_twist_keyboard drives the
  sim robot (`/cmd_vel` confirmed moving the base).
- `4518bcd` feat: unified perception overlay — `signboard_recognition`
  HUD now draws on `/tag_detections_image`, so one stream shows raw
  image + apriltag boxes + textual labels. Node is camera-agnostic.
- `5464191` fix: `drive_to_store` teleport now has an approach margin
  + corridor clamp (`approach_margin_m=0.6`, `corridor_outer_y=1.4`)
  so the robot lands in the corridor, not inside the store wall.
  Single-call accuracy 0.05 m. Note: rapid back-to-back teleports
  race Gazebo's physics step — fine for one-at-a-time agent use.
- `c30a9ca` feat: `signboard_recognition_real.launch` +
  `camera_info_publisher.py` for the real ArmPi USB cam. Renamed the
  sim launch to `signboard_recognition_sim.launch` — **sim/real split
  is by launch file; the node is shared + parametrized.**
- `efedb93` / `eaac2dc` feat+fix: `rtabmap_realsense.launch` — RTAB-Map
  on a live RealSense D435. 848x480 16:9 (D435 native; 4:3 modes crop
  HFOV). `align_depth:=true` resolves the HW3 RGB-vs-depth FOV
  mismatch (`d34be11`) — the real driver's alignment node does what
  the sim plugin didn't.
- `430b8fc` / `e1864e7` docs: `docs/robot_commands.md` +
  `real_cmd.md` — real-robot command reference + run sheet.

### Non-obvious discoveries

- **Teammate's LLM stack** lives in `LLM/` (separate repo,
  github.com/ee478-2/LLM): `gpt_llm_client` (`/llm_query` service) +
  `llm_agent_planner` (planner + agent_interface + mocks). Built
  standalone at `~/llm_ws`; `demo_mock.launch` **verified** — full
  mission completes (explore -> visit all 4 categories -> RETURN_PICKUP).
  It uses different topics than our stack — integration needs 3
  bridges (see `docs/agent_contract.md` + the integration guide).
  Conflict: duplicate `gpt_llm_client` package name — keep theirs,
  drop the vendored MinSungjae one.
- **`armpi/`** is the real robot's ROS workspace (ArmPi Pro, Raspberry
  Pi 4, Ubuntu 18.04, **ROS Melodic**) copied off the SD card. Stock
  Hiwonder packages: chassis, arm, mono USB cam, apriltag.
- **The robot is a two-computer system** (EE478 Week6 hardware doc):
  Raspberry Pi (`roscore`, chassis, arm, mono cam — IP 192.168.0.200)
  + **Jetson Orin** (perception, RViz, **RealSense D435**). Corrected
  an earlier wrong assumption that the robot had only a mono camera —
  the D435 exists, it's just on the Jetson. Laptop = 192.168.0.101.
- Real USB-cam calibration (from `armpi/.../calibration_param.npz`):
  fx=519.84 fy=519.19 cx=336.09 cy=230.58, 640x480, 4-coef fisheye.
- ArmPi chassis: encoder motors (I2C board `0x34`) but the stock node
  publishes **no wheel odometry** — only open-loop velocity. Drive
  topic is `chassis_control/SetVelocity` (custom msg), not `/cmd_vel`.

### Decisions

- Topic 2 final demo target: **the real ArmPi Pro** (user's call), not
  sim-only. Sim stays the development/fallback platform.
- Localization plan: **AprilTag-primary** (absolute, drift-free — the
  map is tag-saturated by design); odometry/VO only bridges the gaps
  between tag sightings; each tag fix resets drift. SLAM is optional.

### Open

- **Phase 1 (next):** integrate Topic 2 end-to-end in sim — build
  `perception_bridge` + `navigation_executor`, resolve the
  `gpt_llm_client` clash, one unified mission launch. Converts the
  primitives into the graded "working agent stack."
- Phase 2: port to the real robot — `chassis_adapter`
  (goal -> `/chassis_control/set_velocity`), real-robot AprilTag
  localization, multi-machine bringup.
- Phase 3: **Topic 1 (grasp) not started** — YOLO on the 4 objects,
  RGB-D position estimation, adapt `intelligent_grasp`. ~60 pts;
  needs an owner assigned now.
- Known bug: `/apriltag_localization_pose` outputs the tag's world
  pose rather than the camera's — `pose_source=sim_gt` works around it.

## 2026-05-20 (A4 Topic 2 — full LLM-agentic stack landed)

Plan: `/home/d/.claude/plans/i-need-to-do-luminous-token.md`. Ten steps,
all 10 functional. Stack overview:

- `e2250b2` chore: vendor re540_final_map, gpt_llm_client,
  apriltag_localization. `catkin_make` clean on ROS Noetic. Gitignored
  the API-key file, catkin symlink, and the >10 MB `plate.jpg` texture.
- `3b08c92` docs(progress): kickoff entry.
- `f1aad97` feat: re540 world bringup launch + env-hook + ground-truth
  doc. Headless smoke-test: `/camera/color/image_raw @ 30 Hz`;
  `/gazebo/model_states` lists 16 signboards + 8 stores.
- `<after this>` feat: global_map.yaml signboards (28 tag-id entries) + stores.yaml
  (8 unknown-category entries). Mapping derived by cross-referencing the
  16 signboard PNGs with `tags.yaml` bundle slot x-offsets.
- `<after this>` feat: `localization_manager` package — fuses
  `/gazebo/model_states`, `/apriltag_localization_pose`, `/rtabmap/odom`
  into `/robot_pose`. `pose_source` launch arg toggles source.
- `<after this>` feat: `apriltag_pipeline.launch` — apriltag_ros
  continuous_detection on `/camera/color/image_raw` with our
  28-bundle tags.yaml + apriltag_localization configured for
  `re540_simulation`. E2E: robot at (0, -0.5, yaw=90°) detected bundle
  [15, 16] = SIGNBOARD09 at 30 Hz.
- `<after this>` feat: `signboard_recognition` — emits JSON observation
  events on `/signboards/detections` (tag_id, parent, slot, arrow, icon,
  signboard_xy, tag_pose_camera) + RViz HUD overlay on
  `/signboards/detections_image`.
- `<after this>` feat: `agent_interface` — `DriveToStore.srv`,
  `ObserveSignboards.srv`, locked JSON contract on
  `/gpt_llm_client/response`, fallback_prompt.py calling gpt-4o-mini
  with `--mock` for offline demos. Direct-service E2E ✓; JSON path needs
  minor quote-escape care in `rostopic pub` for bash.
- `<after this>` feat: `missions/find_cafe.launch` + `run_demo.sh`
  bring up the full stack; `missions/README.md` documents the three
  drive paths (service / topic JSON / fallback pipe).

Known issues (deferred):
- `/apriltag_localization_pose` publishes what looks like the tag's
  world position rather than the camera/robot pose. Diagnose later; for
  now `pose_source=sim_gt` covers the rubric's "localization manager"
  requirement and the AprilTag detection branch is verified upstream.
- DWA tuning for this world is untested; `teleport_fallback:=true` keeps
  the demo deterministic while the agent_interface still publishes a
  proper `nav_msgs/Path` for the future DWA pass.

Open (teammate work for the final deliverable):
- Teammate-owned: system prompt + few-shot examples inside the vendored
  `gpt_llm_client` package. Contract is locked in
  `docs/agent_contract.md`; `fallback_prompt.py` is the canonical
  reference implementation.
- Teammate or user: 5–10 evaluation queries for the report's
  query-success table ("find me a cafe", "which pharmacy is closest?",
  etc.). Scoring uses the ground-truth labels in
  `docs/world_ground_truth.md`.

## 2026-05-03 (HW3-3 DWA tuning round)

- `f4894ac` feat(eval): A3-3 sim Odom/ResetCountdown sweep V x {0,1,5,10,30}
  with 3-way (`/odom`, `/rtabmap/odom`, `/ground_truth/odom`) APE per V.
  V=30 monotonic best (mean 0.0165m). Includes sim_vs_tum_sweep.png panel
  comparison establishing the environment-cleanliness explanation for the
  sequence-dependent → monotonic shift between TUM and sim.
- `aee9afa` feat(eval): visICP / KLT / W120 ablations on TUM bags. Five
  paradigms now characterize slam2 (baseline + BA + visICP + KLT + W120),
  with mean APE clustering at 1.48-1.51m supporting Visual Degeneracy.
- `5391dca` feat(eval): pioneer_slam sweep added to sweep_v2 + 4-bag
  summary plot. V=1 wins on slam (mean 0.212m), confirming slam-class
  convergence on V=1 versus 360's V=5 — sequence-dependent optimum.
- `26c435f` tune(dwa): retune planner for sim corridor navigation.
  heading_offset_rad 0.4 → 0.2 (less yaw lag, smoother strafe), sim_time
  3.2 → 3.0 (tighter rollout score discrimination), ema_alpha 0.1 → 1.0
  in dynamic costmap layer (no smoothing), front sonar subscription
  commented out (rely on costmap obstacle layer only), odom_reset_countdown
  launch arg added for HW3-3.3 RTAB sweep override. Re-source devel and
  rebuild dwa_planner before next sim run.

## 2026-05-02 (HW3-3 sim drift hunt: D435 calibration + BA + viz)

- `f9a5ece` fix(gt_odom): quat_inv was negating w instead of (x,y,z).
  Until this fix, /ground_truth/odom appeared 180°-mirrored from the
  spawn-relative frame in evo_traj plots. Conjugate convention now
  correct.
- `d34be11` fix(sim): the BIG drift win. D435 RGB color sensor was
  69.4° HFOV at 960x540 while depth sensor was 85.2° HFOV at 640x360.
  Mismatch caused depth lookup at RGB feature pixels to return wrong
  3D positions, especially toward image edges → systematic ~22%
  underestimate of lateral motion in /rtabmap/odom (RTAB y / GT y ≈
  0.78 in single-point check). Matched RGB to depth (85.2° / 640x360)
  drops 1.54m snapshot error to 0.17m at the same world location —
  86% reduction. Wider FOV also helps side-wall feature retention
  through corridor turns. Same commit lowered depth Gaussian noise
  stddev 0.100 → 0.02 (real D435 is ~4-8cm at 4m).
- `d80bd8f` feat(viz): RobotModel only ever shows RTAB's belief
  (because TF is owned by rgbd_odometry). Added true_pose_marker.py
  publishing a translucent green CUBE + yellow heading ARROW at /odom
  on /true_robot/markers; RViz TrueRobotMarker display + topic switch
  GroundTruthOdom → /odom (HW2 consistency). Now you can visually
  see drift live = gap between the RobotModel and the green box.
- `8ac85a9` feat(rtabmap): enabled OdomF2M/BundleAdjustment based on
  rotation-drift experiment finding (rot{1,2,3}_turn). First in-place
  rotation injects ~4.6cm false translation because PnP misreads
  parallax differential as translation; BA refines each frame against
  the F2M local map and brings that to ~3.4cm in one isolated test
  (24% reduction). Run-to-run variance is high so real-world impact
  on 60s eval may be smaller, but the change is harmless.
- `8cf1063` feat(scripts): consolidated HW3-3 evaluation tooling —
  run_eval_sim.sh (60s 3-way APE), run_rotation_drift_test.sh
  (forward-then-rotate sequence with sim-time phase logging),
  analyze_phase_drift.py (per-phase pose delta analyzer).
- Open: (a) confirmed empirically that disabling in-place rotation
  in DWA makes drift WORSE (0.58 → 0.82 mean APE on 60s) because
  strafe-only paths take wider arcs with more continuous yaw — kept
  default behind_angle_rad=1.2. (b) tried switching dwa_planner →
  dwa_static; dwa_static stalls at close (~0.35m) waypoints because
  it lacks dwa_planner's goal-extension hack — switched back. dwa_
  static config (sim_time, costmap window) tweaks were reverted.
  (c) Loop closure still disabled (Kp/MaxFeatures=-1); HW3-1/2
  feedback memory says Vis/MinInliers=12 is too strict for LC, but
  textured sim env may behave differently — not retested yet.

## 2026-05-02 (HW3-3 sim stack: RTAB + strafe-yaw + true GT)

- `fb23d5a` HW3-3 sim launch (`dwa_rtabmap.launch`) finalized:
  - RTAB Force3DoF on rgbd_odometry+rtabmap (valid here because
    frame_id=base_footprint is z=up — would forbid forward motion in
    HW3-1/2 optical frame, see feedback_rtabmap_tuning.md).
  - RGBD/CreateOccupancyGrid + Grid params on so /cloud_map and
    /grid_map publish for HW3-2-style visualization.
  - directional_inflation_layer disabled in costmap_params.yaml
    (static env, no moving obstacles — cone projection would thrash
    on RTAB-odom drift).
- `fb23d5a` mecanum strafe-yaw offset added to dwa_planner_node.cpp:
  new params `heading_offset_rad` / `heading_offset_kp`; rotation
  phase target = bearing - offset, post-DWA `cmd.angular.z` overridden
  by P-controller. Body holds the offset while mecanum strafes via
  (vx,vy) toward goal → D435 keeps a side wall in FOV through corridor
  turns. Default 0.0 = backward compatible.
- `fb23d5a` discovered `/odom` from `nexus_ros_force_based_move` is
  body-velocity-integrated dead-reckoning, not ground truth (line
  337-340: `RelativeLinearVel/AngularVel` integrated into
  `odom_transform_`). New `gt_odom_publisher.py` reads
  `/gazebo/model_states`, anchors origin at spawn pose, republishes
  as `/ground_truth/odom`. RViz `GroundTruthOdom` topic switched.
  evo evaluation will need to compare against this, not /odom.
- New `dwa_rtabmap.rviz`: 14 displays organized by category. Cloud
  map subscribes to `/cloud_map` (root namespace) — rtabmap node
  publishes cloud_* topics on public NodeHandle while grid/path use
  private (`/rtabmap/grid_map`); without `<group ns="rtabmap">`
  wrapper the cloud_* topics drop to root.
- Open: (a) verify whether HW2 `/odom` was actually GT vs same
  dead-reckoning behavior — user expects GT but plugin shows
  integrate-and-publish. (b) heading_offset sign — currently +0.4
  intended for right-wall view but user reports sign feels wrong
  in sim. (c) Loop closure re-enable and ghost-frame cleanup are
  next debt items.

## 2026-05-02 (HW3-3 setup: workspace consolidation + textured walls)

- `4e03433` consolidated HW2 packages (dwa_planner, graph_planner,
  local_costmap_generator, nexus_4wd_mecanum_simulator, etc.) into the
  HW3 src/ tree. Side-by-side ee478-hw2/ workspace was causing
  CMAKE_PREFIX_PATH / devel-overlay confusion; one workspace fixes it.
  All 9 packages build clean.
- Same commit: triplanar GLSL shader + OGRE material `RaceTrack/Brick`
  added under `nexus_4wd_mecanum_gazebo/media/`. Lets us texture the
  race_track STL despite no UV coordinates by sampling along world-space
  YZ/XZ/XY planes and blending by world-normal. unit_box obstacles also
  swapped from solid Gazebo/Grey to varied textured materials
  (Bricks/Wood/WoodFloor/PaintedWall/CeilingTiled). Without textured
  walls RTAB-Map ORB extraction would collapse on long wall-only views
  during HW3-3 sim nav.
- Plumbing: `description.launch` gained `publish_gt_tf` arg so HW3-3
  launch can disable `control_plugin.py` and let RTAB-Map own
  odom→base_footprint TF. Both `dwa_planner.launch` and the new
  `dwa_rtabmap.launch` prepend `GAZEBO_RESOURCE_PATH` to make our
  `media/` discoverable by OGRE.
- New launch `dwa_planner/launch/dwa_rtabmap.launch` scaffolds the
  HW3-3-1 deliverable: Gazebo + nexus (no GT TF) + rgbd_odometry +
  rtabmap_slam + heightmap + graph_planner + dwa + RViz. Untested
  end-to-end yet — needs user-side smoke test.
- Open: visual verification that brick texture actually projects (could
  fail silently if OGRE can't compile the GLSL on this driver); tune
  `scale` in `race_track.material` if bricks look wrong size.

Newest first. One bullet per commit (with short SHA), plus notes for
experiments / decisions / blockers that don't have a commit.

## 2026-05-01 (night: extended sweep — 3 bags × 5 values @ rate=1.0)

- A second-agent review flagged that the original ResetCountdown sweep
  was on slam2 only. To strengthen the Assignment 2-2 deliverable,
  re-ran the sweep on **pioneer_360 + pioneer_slam2 + pioneer_slam3**
  at rate=1.0 (matching the original sweep config; new outputs in
  `eval_results/sweep_v2/`). pioneer_slam dropped because it's the
  longest bag (155 s × 5 = 13 min) and adds little signal not already
  covered by slam3.

- **Run-to-run variance discovery on slam2**: re-running V=1 on slam2
  with identical config gave max APE 5.44 (new) vs 2.31 (original).
  Same for other V values — entire sweep ordering changed:

  | V | original max | new max | original mean | new mean |
  |---:|---:|---:|---:|---:|
  | 0 | 3.72 | 3.97 | 1.48 | 1.28 |
  | 1 | **2.31** | **5.44** | **0.51** | **1.52** |
  | 5 | 5.41 | 5.20 | 1.79 | 1.27 |
  | 10 | 5.42 | 5.26 | 1.61 | 1.35 |
  | 30 | 5.26 | 5.19 | 1.89 | 1.42 |

  pioneer_slam2 sits at the VO failure threshold; ORB+RANSAC
  stochasticity dominates the parameter signal. Single-run results
  on this bag are not reliable. The trajectory plots show this
  directly — no V value reproduces the GT loop shape on slam2,
  unlike on the other bags.

- **Final 3-bag sweep result @ rate=1.0** (`sweep_v2/`):

  | V | 360 max | 360 mean | slam3 max | slam3 mean | slam2 max | slam2 mean |
  |---:|---:|---:|---:|---:|---:|---:|
  | 0 | 0.577 | 0.328 | 0.659 | 0.201 | 3.97 | 1.28 |
  | **1** | 0.548 | 0.175 | **0.087** | **0.053** | 5.44 | 1.52 |
  | **5** | **0.278** | **0.148** | 1.083 | 0.317 | 5.20 | 1.27 |
  | 10 | 0.510 | 0.169 | 0.504 | 0.229 | 5.26 | 1.35 |
  | 30 | 0.463 | 0.174 | 0.604 | 0.196 | 5.19 | 1.42 |

  **Optimum is sequence-dependent**:
  - pioneer_slam3 (loop trajectory, occasional VO failures): V=1 wins
    by a huge margin — max APE 0.087 m, ~0.3% drift, comparable to
    Labbé & Michaud 2019's RTAB-Map TUM benchmark (ATE-RMSE
    0.004–0.14 m). Fail-fast reset makes brief VO failures recover
    immediately instead of accumulating drift.
  - pioneer_360 (clean rotation): V=5 wins. The sequence has very
    few real VO failures, so mild reset tolerance avoids spurious
    resets that would drop hard-won feature continuity.
  - pioneer_slam2 (broken — no V reproduces GT shape): single-run
    results dominated by noise; "winner" flips between runs.

- **Plot**: `eval_results/sweep_v2/sweep_v2_3bag.png` — 3 bags × 2
  panels (xy overlay + APE timeseries) × 5 V curves.

- **Report narrative for Assignment 2-2**: parameter optimum is
  sequence-dependent. fail-fast (V=1) excels on sequences with
  occasional motion blur / textureless frames where each failure
  is brief — slam3 is the cleanest illustration. mild tolerance
  (V=5) excels on stable sequences where unnecessary resets cost
  more than they prevent — pioneer_360. extreme cases (slam2)
  expose the limit of single-run measurement when the sequence
  itself is at the failure boundary.

- The submitted launch (`9ab5598`-style baseline + dropped
  ResetCountdown via `1bdb523`) is left unchanged — sweep finding
  doesn't dictate a single "best" V, which is itself the point.

## 2026-05-01 (night: Option C — drop ResetCountdown=1 from submitted launch)

- After locking in the V=1 baseline (commit `d2a0473`), an exploratory
  run of the **stock** rtabmap_launch (no VO tuning, LC default-on) on
  pioneer_slam2 produced surprisingly better metrics:
    tuned (V=1)  : max 5.11, mean 1.55, rmse 1.87, median 1.20
    stock (V=0)  : max 3.68, mean 1.49, rmse 1.64, median 1.51
  Tuned won only on median (= more accurate between resets); stock's
  default `Odom/ResetCountdown=0` (never reset) eliminated the visible
  4 m straight-line spikes that were dominating tuned's max/mean/rmse.

- A third agent review (read on this date) attributed the per-bag
  trade-off to `Odom/ResetCountdown=1` specifically — the spike
  generator — while crediting the rest of the VO tuning
  (`Vis/MinInliers 12`, `Vis/MaxFeatures 1500`, `GFTT/MinDistance 5`,
  `OdomF2M/MaxSize 3000`, `Vis/CorGuessWinSize 80`) with the
  pioneer_360 spike fix. Recommendation: drop ResetCountdown only,
  keep the rest. The Assignment 2-2 sweep finding (V=1 best within
  the sweep config) remains valid as a documented sweep result; only
  the submitted launch reverts to default V=0.

- `1bdb523` — drop `--Odom/ResetCountdown 1` from submitted launch.
  Re-evaluated all 4 bags at rate=0.3. Plot: `final_optionC_4bag.png`.

  | Bag | max APE | mean | median | rmse |
  |---|---:|---:|---:|---:|
  | pioneer_360 | 0.637 | 0.146 | 0.106 | 0.193 |
  | pioneer_slam | 4.500 | 0.448 | 0.414 | 0.542 |
  | pioneer_slam2 | 3.745 | 1.497 | 1.432 | 1.648 |
  | pioneer_slam3 | 5.188 | 0.489 | 0.303 | 0.743 |

- **Trade-off vs V=1 baseline (d2a0473)**:
  - pioneer_360 regressed (max 0.29 → 0.64, mean 0.075 → 0.146).
    Still within the published Labbé & Michaud 2019 RTAB-Map TUM
    range (0.004 – 0.14 m); drift ~3% on a 22 m loop.
  - slam2 max 5.28 → 3.75 (-29%), mean 2.02 → 1.50 (-26%), median
    2.13 → 1.43 (-33%).
  - slam3 max similar but mean 1.21 → 0.49 (-60%), rmse 1.45 → 0.74
    (-49%), median 0.90 → 0.30 (-66%).
  - slam mean 0.57 → 0.45 (-21%), median 0.51 → 0.41 (-19%).
  - **Aggregate mean across 4 bags: 0.97 m → 0.65 m (-33%).**

- The TF flickering issue (`/openni_camera` parent alternating between
  `/laser` and `/kinect`) was raised on KLMS by another student
  (Inga Bhatt, 2026-04-29) and acknowledged by the TA (박성준,
  2026-04-30). TA officially permits a student-side workaround in
  the report. Our `/tf:=/tf_unused /tf_static:=/tf_static_unused`
  bag-side strip is one valid form of this workaround. Worth a
  paragraph in the report's "issues encountered" section.

## 2026-05-01 (evening: failed tuning round + baseline lock-in)

- Tried three more tuning ideas to suppress slam* spike outliers
  (1-2 frame jumps of ~4 m at fast-yaw + textureless wall regions).
  **All three regressed pioneer_360.** Kept commits in history as
  documented negative results:

  | commit | change | pioneer_360 max | slam.bag max |
  |---|---|---:|---:|
  | 362309a (baseline) | tuned VO + ResetCountdown=1 | 0.45 | 2.18 |
  | 8d102cc | + Vis/MaxDepth=4.0 | — | **6.15** (worse) |
  | 6b8a9ad | + Reg/Force3DoF + Odom/GuessMotion | **2.35** | — |
  | b16e130 | – GuessMotion (Force3DoF only) | **2.51** | — |
  | `9ab5598` | restore baseline args | (see below) | (see below) |

- **Why each failed**:
  - `Vis/MaxDepth=4.0` — hypothesis "Kinect v1 noise beyond 4 m
    drives spikes". Wrong target: spikes are 1-2 frame jumps (single
    bad PnP), not gradual depth drift. Capping depth left textureless
    corridors with too few features → degenerate inlier geometry.
  - `Odom/GuessMotion=true` — pioneer_360 is a 360° in-place
    rotation; constant-velocity prior assumes smooth motion and is
    wildly wrong when yaw direction changes faster than the prior.
    Log shows PnP guesses with `y=-566 mm` (50× typical inter-frame).
  - `Reg/Force3DoF=true` — the subtle one. This zeroes out z, roll,
    pitch in `frame_id`'s coordinates. Our `frame_id` is
    `openni_rgb_optical_frame` (camera convention z=forward). So
    Force3DoF was forbidding **forward translation along the optical
    axis** — exactly the motion the robot was doing. Result: 34
    registration failures, 7 odom resets, max APE 2.51 m on a
    sequence that worked at 0.45 m baseline. A previous tuning
    suggestion missed this; would only be valid if `frame_id=base_link`,
    which would require publishing our own static TF and is out of
    scope this close to the deadline.

- Decision after second independent agent review: **stop tuning,
  ship.** The ResetCountdown sweep is the Assignment 2-2 deliverable
  and the slam* residual error is the empirical demonstration of why
  loop closure exists, which the assignment explicitly disables to
  teach this lesson.

- `9ab5598` — restore baseline args. Re-ran all 4 bags at rate 0.3
  for the final lock-in table. Plot:
  `eval_results/final_4bag_summary.png` (xy trajectory + APE
  timeseries per bag).

  | Bag | max APE (m) | mean | median | rmse |
  |---|---:|---:|---:|---:|
  | pioneer_360  | **0.285** | **0.075** | 0.065 | **0.093** |
  | pioneer_slam  | 3.860 | 0.569 | 0.508 | 0.647 |
  | pioneer_slam2 | 5.275 | 2.016 | 2.134 | 2.244 |
  | pioneer_slam3 | 5.127 | 1.209 | 0.895 | 1.453 |

- pioneer_360 numbers are the best we've ever measured on this bag
  (max 0.45 → 0.29 m, rmse 0.14 → 0.09 m). The improvement vs the
  earlier rate=0.5 baseline (837a42d) is from the lower playback
  rate giving rtabmap more CPU headroom per frame, not from any
  param change.
- slam* `max` numbers vary run-to-run by 2-3 m because the bags sit
  near the VO failure threshold (ORB + RANSAC are stochastic). Mean
  and rmse are more stable and tell the consistent story.

## 2026-05-01 (later)

- Assignment 2-2 parameter sweep: `Odom/ResetCountdown` ∈ {0, 1, 5, 10, 30}
  on pioneer_slam2 (rate=1.0, all other VO params held fixed). Script:
  `scripts/sweep_resetcountdown.sh`. Results under
  `eval_results/sweep_resetcountdown/<v>/`, comparison plot
  `compare_xy.png`.

  | ResetCountdown | max APE (m) | mean | median | rmse |
  |---:|---:|---:|---:|---:|
  | 0 (disable) | 3.717 | 1.478 | 1.377 | 1.650 |
  | **1**       | **2.314** | **0.513** | **0.443** | **0.627** |
  | 5           | 5.412 | 1.794 | 1.581 | 1.961 |
  | 10          | 5.416 | 1.611 | 1.129 | 1.858 |
  | 30          | 5.263 | 1.893 | 1.335 | 2.241 |

- **V=1 is best** — counter to the naive "more tolerance = more
  robust" intuition. Story: this bag has many isolated VO failures
  (motion blur on fast turns, textureless walls). With V=0 the
  odometer never resets and drifts unbounded after each failure.
  With V≥5, odometry continues for several frames waiting for
  re-tracking; during those frames it accumulates uncorrected drift,
  and when the reset finally fires the jump is larger. V=1 fails
  fast: each failure → immediate reset → smallest possible jump.
- Side observation: V=1 here is 2.31 m max vs the earlier slam2
  baseline of 5.46 m (same params, rate=0.5). VO is stochastic
  (ORB + RANSAC) and slam2 sits near the failure threshold, so
  single-run numbers have non-trivial variance. Sweep trend is
  still robust because V=1 is best by a large margin (~2× the gap
  to the next-best value).

## 2026-05-01

- Added 3 more TUM bags (`pioneer_slam`, `pioneer_slam2`, `pioneer_slam3`)
  to `src/data/` and re-ran RTAB-Map on all 4 with the tuned launch.
- Generalized `eval.launch`: `rtabmap_viz` is now a pass-through arg so
  batch eval can run headless. Added `scripts/run_eval_one.sh` and
  `scripts/run_eval_all.sh` for one-shot per-bag eval (records
  `/pose` + `/rtabmap/odom`, then `evo_ape -va` with SE(3) Umeyama
  alignment). Per-bag artifacts land in `eval_results/<bag>/`.
- Results (rate=0.5, current tuned VO params, no loop closure):

  | Bag | max APE (m) | mean | median | rmse |
  |---|---:|---:|---:|---:|
  | pioneer_360  | **0.451** | 0.100 | 0.063 | 0.138 |
  | pioneer_slam  | 2.178 | 0.730 | 0.655 | 0.786 |
  | pioneer_slam2 | 5.459 | 2.155 | 1.843 | 2.366 |
  | pioneer_slam3 | 3.613 | 1.364 | 1.105 | 1.608 |

- pioneer_360 dropped from prior max **3.12 m → 0.45 m** with the
  tuned params — confirms the spike fix and clears the open task.
- pioneer_slam* sequences are much harder (longer, faster turns,
  more textureless walls) and the 360-tuned params do not
  generalize. slam2 is the worst (max 5.46 m). Expected — this is
  the kind of variation Assignment 2-2 wants us to study via a
  param sweep, so these numbers become the per-bag baseline for
  that experiment.

## 2026-04-27

- `362309a` — perf(rtabmap): harden VO params for TUM pioneer_360.
  Lowered `Vis/MinInliers` to 12, raised `Vis/MaxFeatures` to 1500,
  `GFTT/MinDistance` 5, `OdomF2M/MaxSize` 3000, `Vis/CorGuessWinSize` 80,
  `Odom/ResetCountdown` 1. Goal: kill the single ~3 m APE spike
  observed in earlier runs while keeping Kp/MaxFeatures -1 (loop
  closure disabled per Assignment 1).
- Decision: switched `frame_id` from `kinect` → `openni_rgb_optical_frame`
  and disabled bag's TF via `/tf:=/tf_unused` remap. Reason: bag's
  `/world -> /kinect` and `/odom -> /base_link -> .../openni_camera`
  trees both claim parents on every frame, causing TF-tree split and
  0-inlier registration. Using the optical frame as `frame_id` and
  stripping bag TF makes rtabmap's `odom -> openni_rgb_optical_frame`
  the only TF. GT remains accessible via `/pose` topic for evo.
- Result so far: `evo_ape -va` mean 0.088 m, median 0.072 m, rmse
  0.157 m, **max 3.12 m**. Tuning above is the attempt to reduce max.

## Open

- Generate submission screenshots: `rtabmap_viz` map view + rviz
  view with rtabmap odom + GT odom (per-bag, at least pioneer_360).
- Write Assignment 2 PDF section: pick figures (per-bag APE table,
  ResetCountdown sweep table + `compare_xy.png`) and explain the
  V=1 finding.
- Optional: rerun V=1 a few times to bound run-to-run variance,
  since slam2 sits near the VO failure threshold and single-run
  numbers can shift by 2-3 m.
- Move on to Assignment 3 (Gazebo sim, RTAB-Map odom for nav).
  The ResetCountdown=1 finding should carry over.
- Decide what to do with `src/my_rtabmap/launch/rtabmap copy.launch`
  (manual backup, untracked). Likely delete.
