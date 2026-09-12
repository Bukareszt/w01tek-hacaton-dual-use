# Experiment: RAI (RobotecAI) on Wojtek, v2

> **Status: EXPERIMENTAL. Not production.**
> Movement tools are supported only in simulation. Against the physical
> robot, over its WiFi AP, the agent has run read-only (camera, position,
> reporting; af34698), and `walk` has been pointed at it once, with
> `text_commander` on the robot, to measure the subscriber discovery delay
> (4307178) -- that is not a supported configuration. The only supported
> real-robot configuration today is `WOJTEK_RAI_READONLY=1` with
> `WOJTEK_RAI_ODOMETRY=0`. Nothing here is deployed by `ros/deploy.sh`, and
> no package here is a dependency of `wojtek_bringup`. Interfaces are
> unstable by definition.

A typed instruction ("stand up, walk forward for three seconds, tell me what
you see") drives Wojtek -- the MuJoCo simulation, or the physical robot with
the movement tools stripped -- through
[RAI](https://github.com/RobotecAI/rai): a LangGraph ReAct agent with a small
set of robot tools, talking to the existing ROS 2 graph. Plan and
rationale: [docs/plans/rai-on-wojtek.md](../../docs/plans/rai-on-wojtek.md).

## How it is wired

```
 laptop                                   inference box (remote GPU, aarch64)
 ┌──────────────────────────────────┐     ┌──────────────────────────┐
 │ wojtek_robot  (ros/sim.sh)       │     │ ~/ollama/bin/ollama serve │
 │   MuJoCo plant, policy_node,     │     │ qwen3-vl:30b-a3b-instruct  │
 │   text_commander, camera         │     └────────────▲─────────────┘
 │        ▲ /wojtek/nav_command     │                  │ ssh -L 11435:11434
 │        │ /camera/.../image_raw   │                  │ (run.sh tunnel)
 │ wojtek_rai  (this experiment)    │                  │
 │   ReActAgent + tools ── ollama ──┼──────────────────┘
 │   streamlit :8501 / run.sh chat  │
 └──────────────────────────────────┘
   both containers: host network, ROS_DOMAIN_ID=42, CycloneDDS
```

- The agent never writes `/cmd_vel`. Its `walk` tool publishes
  `forward|left|right` to `/wojtek/nav_command` every 0.5 s for at most 5 s,
  then `stop`; the simulation's own `text_commander` turns that into
  `/cmd_vel` with a 2 s dead-man. `/wojtek/arm`, `enable`, `zero`, `reset`,
  `joint_targets` and `/cmd_vel` are on RAI's `forbidden` list
  (`wojtek_rai/limits.py`).
- Tools, in the default sim build: `walk`, `turn` (closed-loop on the
  odometry yaw), `stop`, `stand_up`, `lie_down`, `get_robot_position` (TF
  `odom -> base_link`), `get_camera_image`, `wait_for_seconds`. With the nav
  container: `navigate_to_pose`, `go_to_place`, `cancel_navigation`,
  `get_map_pose`, `get_map_image`. With the perception container:
  `find_objects`, `go_to_object`. Every Nav2 goal -- `go_to_object`
  included -- is checked against the workspace box, bounded by
  `limits.NAV_GOAL_TIMEOUT_S`, and cancellable; `stop` requests a cancel of
  the goal in flight before it publishes, because Nav2 outruns a text stop
  otherwise. A goal blocks the agent's turn until it ends, so a `stop` typed
  into the same chat runs only after arrival or the timeout; a goal in
  flight is reached only from a second tab of the same panel (see Known
  gaps).
- `WOJTEK_RAI_READONLY=1` leaves only `get_robot_position`,
  `get_camera_image` and `wait_for_seconds`: every tool that can move the
  robot goes, and `find_objects` with them (the perception tools are built
  together with the movers). `WOJTEK_RAI_ODOMETRY=0` drops `turn` and every
  Nav2 tool (the physical robot's `odom -> base_link` is a static identity,
  so Nav2 cannot track it).
- The streamlit sidebar carries an operator panel -- stand up, lie down, arm,
  disarm, policy on/off -- and the camera feed. It calls the services itself,
  outside the agent, and **it is not an e-stop**: it needs the WiFi link, the
  container and the browser. The pad and the robot's own disarm are.
- The robot's identity is a hand-written `wojtek_rai/embodiment.json`
  (RAI `EmbodimentInfo`); the `build-whoami` doc pipeline is not used yet.
- LLM vendor is `config.toml`: Ollama over the tunnel by default; `[vendor]`
  can be switched to `openai` (needs `OPENAI_API_KEY` in the root `.env`).

## Run it

```bash
# 0. once per machine
./experiments/wojtek_rai_v2/run.sh build           # wojtek_rai image (ros:jazzy-ros-base + rai-core)
./experiments/wojtek_rai_v2/run.sh inference       # Ollama on the inference box (needs WOJTEK_RAI_INFERENCE_SSH in .env)
./experiments/wojtek_rai_v2/run.sh pull qwen3-vl:30b-a3b-instruct   # NOT the bare qwen3-vl:30b: that is the thinking build

# 1. every session, three terminals
./ros/sim.sh --foxglove boot_pose:=folded          # MuJoCo sim; then zero -> stand_up -> arm from the console/Foxglove
./experiments/wojtek_rai_v2/run.sh tunnel          # localhost:11435 -> Ollama
./experiments/wojtek_rai_v2/run.sh agent           # http://localhost:8501

# or, scripted
./experiments/wojtek_rai_v2/run.sh topics          # smoke: RAI sees the sim graph
./experiments/wojtek_rai_v2/run.sh chat "report your position, walk forward for 3 seconds, report again"

# the physical robot, over its WiFi AP: look and report, nothing else
WOJTEK_RAI_READONLY=1 WOJTEK_RAI_ODOMETRY=0 WOJTEK_RAI_COLOR_TRANSPORT=compressed \
  ./experiments/wojtek_rai_v2/run.sh agent
```

Arming stays a human action, in the simulation too: the agent has no arm
tool. In the sim the standing pose sags a little under the soft PD servo, so
if `/wojtek/arm` refuses with a joint displacement just over the 0.15 rad
limit, loosen it for the session:
`ros2 param set /wojtek_real_io max_arm_jump_rad 0.3`.

## Navigation (Nav2) — `run.sh nav`

A third container, `wojtek_nav`, runs Nav2 + slam_toolbox fed by the depth
camera, and the agent gets RAI's Nav2 tools on top (`navigate_to_pose`,
`go_to_place`, `get_map_pose`, `get_map_image`). Plan and rationale:
[docs/plans/rai-nav2-on-wojtek.md](../../docs/plans/rai-nav2-on-wojtek.md).

```
depth (best-effort) ─► depth_relay ─► depthimage_to_laserscan ─► /scan ─► slam_toolbox ─► /map, map→odom
TF odom→base_link + /odom_vel (sim) ─► odom_relay ─► /odom
Nav2: planner (NavFn) · controller (Regulated Pure Pursuit) · behaviors · bt_navigator · velocity_smoother
      ─► /cmd_vel_nav ─► cmd_vel_watchdog (0.5 s dead-man) ─► /cmd_vel ─► policy_node
```

```bash
./experiments/wojtek_rai_v2/run.sh nav build      # once
./experiments/wojtek_rai_v2/run.sh nav launch     # with the sim running and the robot armed
./experiments/wojtek_rai_v2/run.sh chat "go to the hydrant and tell me what you see"
```

Named places for the sim scene live in `wojtek_rai/nav/config/places_sim.yaml`
(prop positions from `scene_sim.xml`, pulled back 0.6 m). Lessons that shaped
the parameters (`wojtek_rai/nav/config/nav2.yaml`):

- the camera publishes best-effort, `depthimage_to_laserscan` wants reliable:
  hence `depth_relay`;
- slam_toolbox is a lifecycle node in Jazzy: it needs its own lifecycle manager;
- the SLAM map starts tiny, so the global costmap is a 12 m rolling window
  (otherwise ComputePathToPose fails with 203, start outside map);
- the gait has a dead band (below ~0.2 rad/s the robot hardly turns), and
  RPP clamps its command to measured speed ± accel·dt: with a normal accel
  limit the command never leaves the dead band. Accel limits are therefore
  effectively off, `min_rotational_vel` 0.4.

## Object-grounded goals — `run.sh perception`

A fourth container, `wojtek_perception` (laptop GPU via the nvidia runtime),
runs RAI's GroundingDINO detection service on `/detection`
(`rai_interfaces/srv/RAIGroundingDino`). Detection only: SAM2-large next to
the simulator's renderer overflows a 4 GB GPU. Weights (~700 MB) download on
first start into the `wojtek_rai_weights` volume.

Agent tools (`wojtek_rai/perception_tools.py`):

- `find_objects(["ball", "fire hydrant"])` → per object: score, distance,
  bearing, position in `map`. Colour and depth have different intrinsics, so
  the colour box goes through the viewing ray (colour K) into the depth image
  (depth K); the depth is the median of the box centre; the point is placed
  in `base_link` and transformed to `map`. Measured error in the sim: 8 cm.
- `go_to_object("fire hydrant")` → the same detection, then a Nav2 goal 0.6 m
  in front of the object, facing it. Not visible → the agent is told to turn
  and retry (rule in the embodiment).

```bash
./experiments/wojtek_rai_v2/run.sh perception build   # once (~8 GB image: torch cu121)
./experiments/wojtek_rai_v2/run.sh perception up
./experiments/wojtek_rai_v2/run.sh chat "find the ball and walk to it"
```

## Layout

| path | purpose |
|---|---|
| `run.sh` | `build \| up \| down \| shell \| topics \| agent \| chat \| tunnel \| inference \| pull \| test \| nav ...` |
| `docker/` | the `wojtek_rai` image and compose service (host net, same DDS settings as `wojtek_robot`) |
| `ros/rai_interfaces.repos` | SHA pin of `rai_interfaces`, colcon-built in the image |
| `config.toml` | RAI vendor/model names; no credentials |
| `wojtek_rai/limits.py` | the safety envelope: topic/service allowlists, walk bounds |
| `wojtek_rai/tools.py` | the tools above, on RAI's `BaseROS2Tool` |
| `wojtek_rai/agent.py`, `app.py`, `chat.py`, `stream.py` | agent assembly, streamlit UI, terminal one-shot, turn streaming |
| `wojtek_rai/nav_tools.py` | RAI Nav2 tools + `go_to_place` |
| `wojtek_rai/perception_tools.py`, `perception_services.py` | `find_objects` / `go_to_object` on RAI's `/detection`; detection-only service launcher |
| `wojtek_rai/camera_feed.py`, `arm_switch.py` | live colour/depth preview and the operator panel's service calls |
| `wojtek_rai/nav/` | nav container glue: launch, Nav2/SLAM params, depth relay, odom relay, cmd_vel watchdog |
| `wojtek_rai/embodiment.json` | who the robot is, for the system prompt |
| `tests/` | model-free tests: limits, tools against a fake connector, hygiene |

## Isolation rules

1. Nothing outside this directory imports anything inside it.
2. `ros/sim.sh`, `ros/dev.sh`, `ros/docker/compose.yaml` are untouched; this
   experiment runs its own compose project (`wojtek_rai`) next to them.
3. Its ROS packages (`rai_interfaces`) are built inside its own image, never
   under `ros/src/`, so `ros/deploy.sh` cannot ship them.
4. Secrets and private host identities enter only through the gitignored
   root `.env` (`OPENAI_API_KEY`, `WOJTEK_RAI_INFERENCE_SSH`, ...), forwarded
   by name.

## Known gaps

- `qwen3-vl:30b-a3b-instruct` is the model in use; tool-call reliability on long
  instructions is unmeasured. The bare `qwen3-vl:30b` tag is the thinking build
  and Ollama 0.34 ignores `think=false` for it (8-70 s of hidden thinking per
  step, measured with `run.sh bench`).
- Navigation is sim-validated only: `map = odom` (static identity, scan
  matching off); the physical robot has no odometry source for Nav2 (plan
  phase N4). The nav stack's `target:=real` path -- slam_toolbox owning
  `map->odom`, the odometry preflight, the watchdog's stop burst on
  reconnect -- has never been run against the robot, and cannot be yet: the
  preflight module `wojtek_rai/nav/preflight.py` is not in the tree, so
  `run.sh nav launch target:=real` shuts itself down at the preflight gate
  (fail-safe) before slam_toolbox, the watchdog or Nav2 start.
- `stop` reaches a Nav2 goal in flight only from a second browser tab of the
  same streamlit panel: a goal blocks the agent's turn for up to
  `NAV_GOAL_TIMEOUT_S` (120 s), streamlit cannot interrupt a running turn
  until the tool returns, `chat.py` is one-shot, and the goal handle is
  per-process (`run.sh chat "stop"` from another process reaches nothing).
  Not verified against a goal in flight. The fix is an operator stop button
  on the panel calling `cancel_active_nav_goal`, or a non-blocking navigate.
- The operator panel is not an e-stop and there is no twist_mux yet: Nav2 and
  the pad both write `/cmd_vel`, and with `target:=real` the watchdog adds
  1 s of zero Twists after every heartbeat gap, interleaved with whatever the
  pad sends (a stop-biased stutter). Do not drive with the pad while the nav
  stack runs.
- Detection thresholds (0.35/0.45) miss small or distant objects (the sim
  person at 5 m); no object memory: an object out of view must be searched
  for by turning.
- `get_map_image` shows only what the depth camera has swept; the sim scene
  has no walls, so the map is mostly "unknown" with a few props.
- Deck cockpit / `/from_human` HRI wiring, `build-whoami`, and the SCAN
  planner as a navigation tool are listed as later steps in the plan.
