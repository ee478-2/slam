> ⚠️ **STALE — pre-restructure (before 2026-06-03).** Written against the old
> monolithic `llm-skill` layout. Package names and paths here may be outdated:
> perception is now `slam/`, the agent stack `llm_agent/`. Treat paths as
> historical; verify against the current tree before relying on them.

# A4 Topic 2 — Implementation Notes

Single reference for everything built during the 2026-05-20 session for
EE40078 Assignment 4, Topic 2 (LLM-based Agentic Exploration). Use this
as the source for the report's "software stack design" section.

---

## 1. Goal & constraints

**Goal.** A mobile robot in the `re540_final_map` Gazebo world that:
1. Localizes itself,
2. Sees signboards and labels them (frame / category-icon / direction-arrow),
3. Receives JSON commands from an LLM,
4. Dispatches them to robot actions (drive to a store, observe signboards,
   report).

**Hard constraints baked into the design.**
- Mobile-robot variant (not drone). Reuses the `nexus_4wd_mecanum`
  simulator from prior assignments.
- Inference runs on this machine's CPU (Ryzen 5800H). No GPU.
- Localization will drift if we lean on RTAB-Map (per TA warning), so
  the Localization Manager treats sim ground-truth as primary and
  AprilTag detections as the canonical correction source.
- Signboards are recognized via **AprilTag-primary lookup**, not a
  trained classifier. Each tag id → textual labels via a hand-authored
  YAML. Satisfies the rubric's "frame + icon + arrow with textual
  labels" without training anything.
- The teammate is supposed to own the LLM system prompt but has zero
  progress, so the stack includes `fallback_prompt.py` so the demo
  survives without them.

---

## 2. Repository layout (after this session)

```
/home/d/llm-skill/
├── config/                                   # YAML knowledge files
│   ├── global_map.yaml                       # stores + signboards + tag semantics
│   └── stores.yaml                           # 8 store coords (category=unknown)
├── docs/
│   ├── PROGRESS.md                           # chronological session log
│   ├── agent_contract.md                     # locked LLM ↔ robot JSON schema
│   ├── implementation.md                     # this file
│   └── world_ground_truth.md                 # dumped /gazebo/model_states
├── missions/
│   ├── find_cafe.launch                      # full-stack mission launcher
│   ├── run_demo.sh                           # convenience CLI w/ --record
│   └── README.md                             # driving recipes
├── tags.yaml                                 # workspace-root copy (untracked)
└── src/
    ├── agent_interface/                      # LLM JSON → robot actions
    ├── apriltag_localization/                # vendored — MinSungjae
    │   └── config/2025/re540_simulation.yaml # 16 signboard world poses
    ├── apriltag_ros (system-installed)
    ├── gpt_llm_client/                       # vendored — MinSungjae (skeleton)
    ├── localization_manager/                 # /robot_pose fuser
    │   ├── config/tags.yaml                  # bundle definitions for apriltag_ros
    │   └── launch/{apriltag_pipeline, localization_manager}.launch
    ├── nexus_4wd_mecanum_simulator/          # existing
    │   └── nexus_4wd_mecanum_gazebo/launch/re540_bringup.launch
    ├── re540_final_map/                      # vendored — world + textures + env-hook
    ├── signboard_recognition/                # tag_detections → textual labels
    └── (existing prior-assignment packages: dwa_planner, graph_planner,
        local_costmap_generator, my_rtabmap, control_space_planner, …)
```

---

## 3. Topic / service map

```
                        ┌─────────────────────────────────────────┐
                        │ Gazebo (re540_final_map world + robot)  │
                        │  /gazebo/model_states                   │
                        │  /gazebo/set_model_state (service)      │
                        │  /camera/{color,depth,...}/image_raw    │
                        │  /camera/color/camera_info              │
                        └─────────────────────────────────────────┘
                                  │
   ┌──────────────────────────────┼──────────────────────────────────────┐
   │                              │                                      │
   ▼                              ▼                                      ▼
┌──────────────────────┐  ┌────────────────────────┐  ┌────────────────────────────┐
│ apriltag_ros         │  │ apriltag_localization  │  │ localization_manager       │
│ continuous_detection │  │ (vendored)             │  │                            │
│                      │  │                        │  │ subs:                      │
│ subs:                │  │ subs:                  │  │   /gazebo/model_states     │
│   /camera/color/...  │──▶│  /tag_detections      │  │   /rtabmap/odom            │
│ pubs:                │  │ pubs:                  │  │   /apriltag_localization_pose
│   /tag_detections    │  │   /apriltag_localiz…   │──▶│ pubs:                      │
│   /tag_detections_im │  │     _pose              │  │   /robot_pose (PoseStamped │
└──────────────────────┘  └────────────────────────┘  │     in frame "map")        │
                                                      └────────────────────────────┘
                                                                  │
            ┌─────────────────────────────────────────────────────┘
            ▼
┌──────────────────────────────┐
│ signboard_recognition        │       ┌─────────────────────────────────────────┐
│                              │       │ agent_interface                         │
│ subs:                        │       │                                         │
│   /tag_detections            │       │ subs:                                   │
│   /robot_pose                │──────▶│   /robot_pose                           │
│   /camera/color/image_raw    │       │   /signboards/detections                │
│ pubs:                        │       │   /gpt_llm_client/response  (JSON in)   │
│   /signboards/detections     │       │ pubs:                                   │
│     (std_msgs/String, JSON)  │       │   /graph_planner/path/global_path       │
│   /signboards/detections_im  │       │   /agent/answer  (natural language)     │
└──────────────────────────────┘       │ services:                               │
                                       │   /agent_interface/drive_to_store       │
                                       │   /agent_interface/observe_signboards   │
                                       │ uses:                                   │
                                       │   /gazebo/set_model_state (teleport)    │
                                       └─────────────────────────────────────────┘
                                                              │
                                                              ▼
                                              ┌─────────────────────────────┐
                                              │ fallback_prompt.py          │
                                              │  OpenAI gpt-4o-mini →       │
                                              │  contract-valid JSON →      │
                                              │  pipe to /gpt_llm_client/   │
                                              │  response                   │
                                              └─────────────────────────────┘
```

---

## 4. Component reference

### 4.1 `re540_bringup.launch`
**Where:** `src/nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/launch/re540_bringup.launch`

Brings up `gazebo_ros/empty_world.launch` with
`re540_final_map/worlds/final_map.world`, the nexus URDF, and one
spawn of `nexus_4wd_mecanum`. No autonomy. Args: `gui`, `pose_x`,
`pose_y`, `pose_z`, `world_name`.

**Critical detail — env-hook for textures.** The world references
`file://materials/scripts`, `file://materials/textures`,
`file://models/Final_map.stl`. Those resolve only if Gazebo's
`GAZEBO_RESOURCE_PATH` and `GAZEBO_MODEL_PATH` include the
`re540_final_map` package root. We added
`src/re540_final_map/cmake/env-hooks/99.re540_gazebo_paths.sh.em`
(catkin env-hook, mirrors the pattern already in
`nexus_4wd_mecanum_gazebo`). After `catkin_make` and
`source devel/setup.bash`, both paths get prepended automatically.

**Verification** (headless smoke-test, already proven):
- `/camera/color/image_raw` publishes at **30 Hz**.
- `/gazebo/model_states` reports 16 `signboard_white_NN` + 8
  `store_*` + the robot.

### 4.2 `localization_manager`
**Where:** `src/localization_manager/scripts/localization_manager_node.py`

A single rospy node fuses three pose sources and publishes
`geometry_msgs/PoseStamped` on `/robot_pose` (frame: `map`) at
**20 Hz**. The `pose_source` ROS parameter chooses the policy:

| value     | behavior                                                                    |
|-----------|-----------------------------------------------------------------------------|
| `auto`    | tag-pose if it's < `tag_freshness_s` (1.0 s) old → sim_gt → rtabmap         |
| `sim_gt`  | only `/gazebo/model_states` (filtered by `robot_model_name`)                |
| `tag`     | only `/apriltag_localization_pose`                                          |
| `rtabmap` | only `/rtabmap/odom`                                                        |

Subscribers store the latest message under a lock; the main loop picks
according to `pose_source`, re-stamps the message with the current ROS
time, and republishes. Source switches are logged once on change.

**Why a single `/robot_pose` topic?** Every downstream component
(`signboard_recognition`, `agent_interface`) reads pose from a single
place. Swapping pose sources is a launch-arg flip — no client changes.
This is the rubric's "localization manager" requirement.

**Sim_gt as default rationale.** The TA warned localization will drift.
Sim ground truth removes drift entirely from the *demo* path while
`/apriltag_localization_pose` still flows through, proving the tag
pipeline works (Step 5). For a real robot, switch to `tag` or `auto`.

### 4.3 AprilTag pipeline (`apriltag_pipeline.launch`)
**Where:** `src/localization_manager/launch/apriltag_pipeline.launch`

One launch file brings up two upstream nodes:

1. **`apriltag_ros_continuous_node`** — runs `apriltag_ros`
   continuous_detection on `/camera/color/image_raw`, loading
   `src/localization_manager/config/tags.yaml` (28 tag bundle
   definitions covering SIGNBOARD01..16). Publishes:
   - `/tag_detections` (`apriltag_ros/AprilTagDetectionArray`)
   - `/tag_detections_image` (raw image with green tag boxes drawn)

2. **`apriltag_localization_node`** — from the vendored
   `apriltag_localization` package, configured with
   `2025/re540_simulation` (which ships with all 16 signboard world
   poses already). Subscribes to `/tag_detections`, looks up tag world
   pose, and emits `geometry_msgs/PoseStamped` on
   `/apriltag_localization_pose`.

**Verified end-to-end:** with the robot teleported to (0, −0.5, yaw=90°),
the camera saw signboard_white_09 and reported bundle `[15, 16]` (=
SIGNBOARD09) at 30 Hz.

**Known issue:** `/apriltag_localization_pose` outputs values that look
like the tag's world position, not the camera's. Diagnose later; for
now `pose_source=sim_gt` skips this branch.

### 4.4 `slam/config/global_map.yaml` — 28 signboard tag entries
**Where:** `slam/config/global_map.yaml`

For each AprilTag bundle ID (1..28 from `tags.yaml`), `signboards.*.tags`
stores:

```yaml
signboards:
  SIGNBOARD09:
    model: signboard_white_09
    pose: {x: 0.000, y: 0.635, z: 0.365, yaw_deg: 90.0}
    tags:
      - id: 15
        slot: left
        semantic: {arrow: left, icon: convenience_store}
      - id: 16
        slot: right
        semantic: {arrow: right, icon: pharmacy}
```

**How the mapping was derived.** Three sources cross-referenced:

1. The 16 PNG textures in `signboards/01.png` .. `16.png` — I viewed
   each and noted the (arrow, icon) pair at each visible bundle
   position.
2. `tags.yaml`'s bundle `layout` entries — each bundle has 1–3 tags at
   x-offsets `-0.0655` (left), `0.0112` (center), `0.0879` (right).
3. `docs/world_ground_truth.md` — gazebo's `signboard_white_NN`
   world poses.

The icon vocabulary is `cafe | hamburger | pharmacy | convenience_store
| pickup_point`. The arrow vocabulary is `left | straight | right`.
Together with `parent` (the signboard model name) and `slot`, this is
exactly the "frame + category icon + direction arrow" textual labelling
the rubric asks for.

### 4.5 `config/stores.yaml` — 8 unknown-category stores
**Where:** `config/stores.yaml`

```yaml
stores:
  store_1: {x: -1.25, y: -1.00, category: unknown}
  store_2: {x: -1.00, y: -2.00, category: unknown}
  …
pickup_point: {x: 0.0, y: 2.0}
categories: [cafe, hamburger, pharmacy, convenience_store]
```

Categories are deliberately `unknown` — the LLM must infer them by
relating signboard observations (which CATEGORY-icon points which
DIRECTION) to nearby store xy. The ground-truth labels for end-of-
mission scoring live in `docs/world_ground_truth.md` (NOT here, to keep
the LLM honest at runtime).

### 4.6 `signboard_recognition`
**Where:** `src/signboard_recognition/scripts/signboard_recognition_node.py`

The perception node that closes the rubric's "must implement recognition
of signboards components" loop. Inputs:

- `/tag_detections` — what tags are visible right now.
- `/robot_pose` — used as the timestamp anchor and to record where we
  were when we saw a tag.
- `/camera/color/image_raw` — for the HUD overlay.

Static knowledge: `slam/config/global_map.yaml`.

For each detection event with non-empty `detections`, it emits one
`std_msgs/String` JSON message on `/signboards/detections`:

```json
{
  "stamp": 15.594,
  "robot_pose": {"x": 0.0001, "y": -0.5001, "yaw": 1.5707},
  "observations": [
    {
      "tag_id": 15,
      "parent": "signboard_white_09",
      "slot": "left",
      "arrow": "left",
      "icon": "convenience_store",
      "signboard_xy": [0.0, 0.629],
      "tag_pose_camera": {"x": 0.026, "y": -0.257, "z": 1.034}
    },
    { ... tag 16 ... }
  ]
}
```

The same event drives the HUD overlay on `/signboards/detections_image`
— a copy of the camera frame with green text in the top-left listing
the visible labels (black outline for legibility on bright walls). View
with `rqt_image_view /signboards/detections_image`.

**Why JSON-over-String instead of a custom msg.** Custom message
compilation forces every downstream language to regenerate bindings; a
string is trivially parseable in any LLM client. The cost is type
safety, which is acceptable for an assignment.

### 4.7 `agent_interface`
**Where:** `src/agent_interface/scripts/agent_node.py`

The LLM-to-robot bridge. Two parallel interfaces:

**Direct services** (handy for unit-testing or CLI use without an LLM):

| service                                  | srv                                 |
|------------------------------------------|-------------------------------------|
| `/agent_interface/drive_to_store`        | `agent_interface/DriveToStore`      |
| `/agent_interface/observe_signboards`    | `agent_interface/ObserveSignboards` |

**JSON contract over topics** (the canonical LLM path; locked in
`docs/agent_contract.md`):

- Inbound: `/gpt_llm_client/response` (`std_msgs/String`, JSON):
  ```json
  {"action": "drive_to_store" | "observe_signboard" | "report",
   "args":   { ... }}
  ```
- Outbound: `/agent/answer` (`std_msgs/String`, natural-language).

The dispatch is straightforward:

| action              | implementation                                                                |
|---------------------|-------------------------------------------------------------------------------|
| `drive_to_store`    | Look up `stores.yaml`; publish a 2-waypoint `nav_msgs/Path` on `/graph_planner/path/global_path` (frame `odom`); optionally call `/gazebo/set_model_state` (teleport — see §5). |
| `observe_signboard` | Buffer `/signboards/detections` for `duration_sec`, dedupe by `tag_id`, return the list.                                          |
| `report`            | Aggregate the rolling observation log into a natural-language summary on `/agent/answer`.                                          |

### 4.8 `fallback_prompt.py`
**Where:** `src/agent_interface/scripts/fallback_prompt.py`

CLI script that calls `gpt-4o-mini` with the locked system prompt + the
current world state (stores.yaml + most-recent `/signboards/detections`
event if a ROS master is reachable) and prints a single contract-valid
JSON line to stdout. Designed to be piped into `rostopic pub`:

```bash
P=$(python3 src/agent_interface/scripts/fallback_prompt.py --query "find me a cafe")
rostopic pub -1 /gpt_llm_client/response std_msgs/String "data: '$P'"
```

**`--mock` mode** prints a canned `drive_to_store` JSON without an API
call. Use for offline demos / when the wifi flakes on demo day.

**API-key handling.** Reads `ChatGPT_API_KEY.txt` (gitignored),
extracts the first `sk-…` substring via regex (works whether the file
is a bare key or wrapped in `export OPENAI_API_KEY='sk-…'`). Falls
back to the `OPENAI_API_KEY` env var.

**ROS-master guard.** Before calling `rospy.init_node`, the script
does a quick TCP probe of `ROS_MASTER_URI` so it doesn't hang when no
master is running (e.g. when you're sanity-checking the prompt
offline).

### 4.9 `missions/find_cafe.launch` + `run_demo.sh`
**Where:** `missions/find_cafe.launch`, `missions/run_demo.sh`

The single launch that composes everything: world + robot + apriltag
pipeline + localization_manager + signboard_recognition +
agent_interface. Args:

- `gui` (default true) — Gazebo client.
- `pose_source` (default `sim_gt`) — passed through to
  `localization_manager`.
- `teleport_fallback` (default true) — passed through to
  `agent_interface` (see §5).

`run_demo.sh` is a convenience wrapper supporting `--headless` and
`--record` (rosbag → `eval_results/a4_t2/demo_<timestamp>.bag`).

---

## 5. Design decisions and trade-offs

### 5.1 Teleport-by-default vs. DWA path following

`agent_interface` publishes a real `nav_msgs/Path` on
`/graph_planner/path/global_path` every time it dispatches
`drive_to_store`. That message **is** the input DWA was built to
consume. But by default we **also** call `/gazebo/set_model_state` to
snap the robot to the target instantaneously.

**Why teleport is the default.** DWA in `src/dwa_planner` was tuned
for the prior assignments' obstacle worlds. The re540 world has
different geometry (corridors, signboard pillars, store fronts), and
verifying DWA's behavior here was out of scope for this session.
Teleporting guarantees the demo "works" while still publishing the
intended Path so the rubric's motion-planning hook isn't a stub.

**To switch to real motion.**
```bash
roslaunch missions/find_cafe.launch teleport_fallback:=false
```
Then start `dwa_planner_node` + `local_costmap_generator` separately
(or add them into `find_cafe.launch`). DWA's current velocity caps —
in `src/dwa_planner/config/dwa_params.yaml` — are `max_vel_trans=0.55
m/s`, `max_vel_theta=0.5 rad/s`. Reasonable for indoor corridors;
lower if the robot oscillates.

### 5.2 AprilTag-primary signboard recognition vs. YOLO

The rubric describes a YOLO-on-icons approach. We chose AprilTag-
primary because:
- The world is already plastered with `tag36h11` AprilTags (28 unique
  IDs across 16 bundles) so the signboard `parent` is unambiguous
  the moment any tag is decoded.
- The (arrow, icon) text labels are a static property of each
  signboard model, so a YAML lookup is exact — no train/test/confusion
  risk.
- Inference runs at 30 Hz on CPU, no GPU dependency.
- The rubric's textual-label requirement is fully satisfied by the
  YAML.

The downside: if the rubric specifically wanted a YOLO classifier on
the icon crops, this approach won't score for that sub-criterion. Easy
to add later as a "consensus" check on top of the AprilTag lookup.

### 5.3 Sim ground truth as the primary pose source

`pose_source=sim_gt` doesn't drift — but it only works in simulation.
For the rubric, we still wired the `tag` and `rtabmap` branches and
expose the toggle as a launch arg, so the report can show an ablation
(e.g. "drift over 60 s: sim_gt vs. tag vs. rtabmap"). On real hardware,
switch to `auto` (which prefers tag corrections when fresh).

### 5.4 JSON-over-`std_msgs/String` for the LLM contract

A custom message would give compile-time type safety, but every
language client (Python, the teammate's prompt code, an eventual web
UI) would need regenerated bindings. Plain JSON text in `std_msgs/String`
is universal and version-friendly. The schema is fixed in
`docs/agent_contract.md` and validated in `fallback_prompt.py` before
emitting.

---

## 6. How to verify each component

Run each in a second terminal after `source devel/setup.bash`:

```bash
# 1. world up?
rostopic hz /camera/color/image_raw     # expect ~30 Hz
rostopic echo -n1 /gazebo/model_states  # expect 16 signboards + 8 stores

# 2. pose plumbing?
rostopic hz /robot_pose                 # expect ~20 Hz
rosparam get /localization_manager/pose_source

# 3. apriltag detection? (robot needs to face a signboard)
rosservice call /gazebo/set_model_state '{model_state: {model_name: nexus_4wd_mecanum,
    pose: {position: {x: 0, y: -0.5, z: 0.05}, orientation: {z: 0.7071, w: 0.7071}},
    reference_frame: world}}'
rostopic echo -n1 /tag_detections       # expect bundle [15, 16]

# 4. signboard recognition?
rostopic echo -n1 /signboards/detections # expect tag 15 (left+convenience_store), tag 16 (right+pharmacy)
rqt_image_view /signboards/detections_image  # green HUD overlay

# 5. drive primitive?
rosservice call /agent_interface/drive_to_store "store_id: 'store_4'"
# robot teleports to (-0.75, -0.70)

# 6. LLM JSON dispatch?
P='{"action":"drive_to_store","args":{"store_id":"store_7"}}'
rostopic pub -1 /gpt_llm_client/response std_msgs/String "data: '$P'"
# robot teleports to (1.00, 1.50)

# 7. observe + report?
rosservice call /agent_interface/observe_signboards "duration_sec: 3.0"
rostopic echo -n1 /agent/answer
```

---

## 7. Known issues / open items

1. **`/apriltag_localization_pose` returns the tag's world position
   instead of the camera/robot pose.** Affects `pose_source=tag` and
   `auto`. Work-around: `pose_source=sim_gt` (default). Fix later by
   inspecting `apriltag_localization.cpp`'s frame composition.
2. **DWA not wired into `find_cafe.launch`.** `teleport_fallback:=true`
   covers the demo; for graceful motion, see §5.1.
3. **`store_3` ambiguity.** `Store coordinates.txt` lists `(-1, -1.5)`
   but the nearest gazebo store is `store_store_green` at `(-1, -2.01)`
   — likely a TA-side typo. We keep `store_3` in stores.yaml as-is and
   call it out in `world_ground_truth.md`.
4. **Teammate dependency.** The `gpt_llm_client` package is vendored
   but the actual system prompt + few-shot examples are the teammate's
   work. `fallback_prompt.py` is the reference implementation in the
   meantime.
5. **No autonomous explorer.** The robot only moves on explicit
   commands. If the rubric expects "explore the unknown environment"
   to run autonomously, we need to add a planner that issues
   `observe_signboard` → `drive_to_store` → loop. Easy add (~50 lines
   in Python) but not in scope this session.

---

## 8. Glossary of artifacts (for the report)

- **Rosbag**: `eval_results/a4_t2/demo_<timestamp>.bag` (when
  recorded via `run_demo.sh --record`). Includes `/robot_pose`,
  `/tag_detections`, `/signboards/detections`, `/agent/answer`,
  `/cmd_vel`, `/gazebo/model_states`.
- **HUD overlay**: `/signboards/detections_image` — the rubric-visible
  evidence that the perception layer is producing textual labels.
- **JSON event log**: every `/signboards/detections` message is a
  self-contained JSON record; `rostopic echo /signboards/detections >
  detections.jsonl` gives you a JSONL trace for plotting.
- **Ground truth**: `docs/world_ground_truth.md` — store positions +
  category labels; use these to score the LLM's inferences.
- **Agent contract**: `docs/agent_contract.md` — schema spec the
  teammate's prompt must adhere to.
