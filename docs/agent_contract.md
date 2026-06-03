> ⚠️ **STALE — pre-restructure (before 2026-06-03).** Written against the old
> monolithic `llm-skill` layout. Package names and paths here may be outdated:
> the agent stack is now `llm_agent/` and perception is `slam/`. Treat paths as
> historical; verify against the current tree before relying on them.

# Agent JSON Contract — A4 Topic 2

This file is the **single source of truth** for the LLM-to-robot
interface. The `agent_interface` node enforces it. If you change the
schema here you must also update `agent_node.py` and the teammate's
prompt template.

## Inbound: `/gpt_llm_client/response` (`std_msgs/String`, JSON body)

```json
{
  "action": "drive_to_store" | "observe_signboard" | "report",
  "args":   { ... }
}
```

### `drive_to_store`
```json
{"action": "drive_to_store", "args": {"store_id": "store_3"}}
```
- `store_id` must be one of the keys in `config/stores.yaml`
  (`store_1` .. `store_8`).
- The agent publishes `nav_msgs/Path` on
  `/graph_planner/path/global_path` (frame `odom`) with two waypoints:
  current `/robot_pose` and the store xy. `dwa_planner` is expected to
  follow it.
- For deterministic demos the agent supports an off-by-default
  Gazebo teleport. Enable via launch arg `teleport_fallback:=true` or
  per-call `args.teleport: true`.

### `observe_signboard`
```json
{"action": "observe_signboard", "args": {"duration_sec": 3.0}}
```
- Buffers `/signboards/detections` for the given window, deduplicates by
  `tag_id`, returns the unique observations on `/agent/answer`.

### `report`
```json
{"action": "report", "args": {"query": "find me a cafe"}}
```
- Summarizes the rolling observation log + restates the user query as a
  natural-language string on `/agent/answer`.

## Outbound: `/agent/answer` (`std_msgs/String`)

Free-form natural-language reply. The teammate's chat layer can echo
this back to the user.

## Direct services (no JSON, for unit-test + CLI use)

| Service                                  | srv                                 |
|------------------------------------------|-------------------------------------|
| `/agent_interface/drive_to_store`        | `agent_interface/DriveToStore`      |
| `/agent_interface/observe_signboards`    | `agent_interface/ObserveSignboards` |

```
DriveToStore.srv
  string store_id
  ---
  bool   success
  string message

ObserveSignboards.srv
  float64 duration_sec
  ---
  bool   success
  string detections_json   # JSON-encoded list of unique observations
```

## What the teammate's prompt should produce

The teammate owns the system prompt + few-shot examples + parser inside
`gpt_llm_client`. The output of that pipeline must be a **single JSON
object** matching the schema above. Reference `fallback_prompt.py`'s
`SYSTEM_PROMPT` for the canonical wording.

`fallback_prompt.py --mock` emits a contract-valid `drive_to_store`
JSON without an API call — use it to demo the full robot loop offline.
