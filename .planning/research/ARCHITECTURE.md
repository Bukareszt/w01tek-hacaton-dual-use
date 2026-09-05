# Architecture Research

**Domain:** Bolting a RAI (RobotecAI) embodied-AI agent onto an existing ROS 2 Jazzy quadruped stack (Wojtek), sim-first, as an isolated `experiments/` overlay
**Researched:** 2026-09-05
**Confidence:** MEDIUM (RAI's own docs are thin on node-level diagrams; corroborated across robotecai.github.io/rai, github.com/RobotecAI/rai, the RAI arXiv paper, and DeepWiki. Wojtek-side architecture is HIGH — read directly from this repo's own docs.)

## Standard Architecture

### RAI's own component model (from upstream)

RAI is not a single ROS 2 node — it is a **Python agent process** that uses ROS 2 as one of several possible transports. Three abstractions, confirmed consistently across the RAI paper (arXiv:2505.07532), the docs site, and DeepWiki:

- **Agent** — the reasoning loop (`run()`/`stop()`), typically `ReActAgent`: a LangChain ReAct loop (LLM ↔ tool-calling) wrapping a set of Tools and holding references to Connectors. **The Agent is not itself an rclpy node.**
- **Connector** — the actual sensor/actuator bridge: `ROS2Connector` owns the rclpy node, publishers, subscribers, service/action clients. `ROS2HRIConnector` is the human-input/output specialization (multimodal: text, image, audio) that maps to `rai_interfaces/msg/HRIMessage` on `/from_human` and `/to_human`-style topics.
- **Tool** — a LangChain-compatible callable that a Connector executes on demand: `GetROS2TopicsNamesAndTypesTool`, `GetROS2ImageTool`, `PublishROS2MessageTool`, `CallROS2Service`, `GetROS2LidarDataTool`, a Nav2 `NavigateToPose`-style action tool, `GetDistanceToObjects`. All inherit `BaseROS2Tool`, constructed with a `ROS2Connector` **plus explicit readable/writable/forbidden topic-service-action allow-lists** — this is RAI's built-in scoping mechanism, and it is the first place Wojtek-specific safety boundaries belong (e.g. forbid `wojtek_bringup` state-machine services, allow only `cmd_vel` + camera topics + the nav action).

`rai_whoami` is **not a running node** — it's a build-time step: `build_whoami.py <dir> --build-vector-db` ingests a directory of docs/images/URDF, and `EmbodimentInfo.from_directory(dir).to_langchain()` turns that into the agent's system prompt at process start. This maps cleanly onto Wojtek: point it at `ros/src/wojtek_description/`, `CLAUDE.md`, and `training/docs/configuration.md` excerpts.

```
┌──────────────────────────────────────────────────────────────────────┐
│                    RAI Agent Process (one Python process)            │
│                                                                        │
│  EmbodimentInfo (whoami)  →  system prompt                           │
│         │                                                             │
│         ▼                                                             │
│  ReActAgent (LangChain ReAct loop, LLM/VLM backend via config.toml)  │
│         │ selects & invokes                                          │
│         ▼                                                             │
│  Tools: GetROS2Image / PublishROS2Message / CallROS2Service /        │
│         NavigateToPose action tool / GetROS2TopicsNamesAndTypes      │
│         │ (each tool scoped: readable/writable/forbidden lists)      │
│         ▼                                                             │
│  ROS2Connector (owns the rclpy node: pubs, subs, service+action      │
│                 clients) ─────────────┐                              │
│  ROS2HRIConnector (text/image/audio   │  target_connectors: fan out  │
│                 I/O, HRIMessage)      │  agent replies to multiple   │
│                                        │  sinks (chat UI + log)      │
└────────────────────────────────────────┴──────────────────────────────┘
                     │ ROS 2 DDS (CycloneDDS)
                     ▼
┌──────────────────────────────────────────────────────────────────────┐
│                 Existing Wojtek ROS 2 graph (unchanged)               │
│  /cmd_vel ──► wojtek_policy (50 Hz RL locomotion) ──► MuJoCo sim /    │
│                                                         real actuators │
│  camera/point-cloud topics ◄── wojtek_perception_bringup              │
│  (optional, this milestone) SCAN-Planner / Nav2 local planner        │
└──────────────────────────────────────────────────────────────────────┘
```

Source: [RAI docs — Tutorials/Overview](https://robotecai.github.io/rai/tutorials/overview/), [Tools](https://robotecai.github.io/rai/tutorials/tools/), [Complete walkthrough](https://robotecai.github.io/rai/tutorials/walkthrough/), [github.com/RobotecAI/rai](https://github.com/RobotecAI/rai), [RAI: Flexible Agent Framework for Embodied AI (arXiv:2505.07532)](https://arxiv.org/abs/2505.07532), [DeepWiki: Agent System](https://deepwiki.com/RobotecAI/rai/3-agent-system).

### Component Responsibilities

| Component | Responsibility | Typical Implementation |
|-----------|----------------|------------------------|
| RAI Agent (ReAct loop) | LLM/VLM reasoning, tool selection, conversation state | `rai.agents.ReActAgent`, LangChain, backend from `config.toml` |
| `rai_whoami` / `EmbodimentInfo` | Static identity + capability context injected as system prompt | Build step over a docs/URDF directory, loaded once at process start |
| `ROS2Connector` | Owns the actual rclpy node: publishers, subscribers, service/action clients | One per agent process (or shared) |
| `ROS2HRIConnector` | Text/image/audio human I/O over `rai_interfaces/msg/HRIMessage` | `/from_human`, `/to_human` topics, or a Streamlit chat UI on top |
| `BaseROS2Tool` subclasses | Scoped, single-purpose ROS 2 actions the agent can call | `GetROS2ImageTool`, `PublishROS2MessageTool`, `CallROS2Service`, Nav2 action tool |
| `cmd_vel` producer (RAI-side) | Converts agent intent ("walk forward") into `geometry_msgs/Twist` on `cmd_vel` | `PublishROS2MessageTool` targeting `cmd_vel`, or a small custom tool wrapping it |
| Velocity arbiter/limiter (new, Wojtek-side) | Bounds and arbitrates whichever `cmd_vel` source is live (agent vs teleop vs nav) | `twist_mux` + a velocity/acceleration clamp, sitting *between* the agent and `wojtek_policy` |
| Local planner (existing, reused) | Turns a goal into safe, collision-avoiding `cmd_vel` | SCAN-Planner (sim-only today) or Nav2 stack, feeding the same arbiter |
| `wojtek_policy` (existing, untouched) | 50 Hz RL locomotion inference | Already deployed; consumes `cmd_vel` exactly as it does for teleop today |

## Recommended Project Structure

Following the repo's own isolation rule (`experiments/<name>/` — see `experiments/autonomous_architecture_ros2_v1/README.md`), mirror that experiment's proven layout rather than inventing a new one:

```
experiments/rai_agent_v1/                    # new, self-contained
├── README.md                    # status, isolation-rule restatement, run instructions
├── run.sh                       # test | agent | sim  (own colcon build + PYTHONPATH, like the sibling experiment)
├── pyproject.toml               # RAI + this experiment's own deps (uv/pip, NOT training/pyproject.toml)
├── config.toml                  # RAI vendor config (LLM/VLM backend) — gitignored where it holds keys
├── ros/src/                     # colcon overlay workspace, OUTSIDE ros/src — deploy.sh can't reach it
│   ├── wojtek_rai_bringup/      # this experiment's launch package (RAI's repo has no rai_bringup pkg —
│   │                            #  every RAI demo ships its own; this is ours)
│   │   └── launch/
│   │       ├── agent.launch.py          # RAI agent node/process + ROS2Connector params
│   │       └── velocity_arbiter.launch.py  # twist_mux + clamp, params scoped to Wojtek's cmd_vel contract
│   └── wojtek_rai_tools/        # (optional) Wojtek-specific RAI Tool subclasses if stock tools are insufficient
│       └── wojtek_rai_tools/
│           ├── whoami/          # source directory rai_whoami builds from (URDF excerpt, docs, capability list)
│           └── nav_tool.py      # thin wrapper if SCAN-Planner needs a bespoke Tool (Nav2 action tool may suffice)
├── docs/
│   └── architecture.md          # this experiment's own settled decisions (pattern: autonomous_architecture_ros2_v1/docs/architecture.md)
└── tests/                       # model-free unit tests: tool scoping, whoami build, config loading
```

### Structure Rationale

- **`ros/src/` lives inside the experiment, not at repo-root `ros/src/`:** identical to `autonomous_architecture_ros2_v1`'s rule — `ros/deploy.sh` rsyncs only the top-level `ros/src/` and builds `--packages-up-to wojtek_bringup`, so a package that never appears there cannot reach the robot even by accident.
- **`run.sh` does its own `colcon build` and `PYTHONPATH`:** `ros/sim.sh`, `ros/docker/compose.yaml`, and `training/run.sh` stay untouched — same rule the existing experiment already validated.
- **No `rai_bringup` upstream to reuse:** RAI's own repo has no generic bringup/launch package; every RAI demo (manipulation, mobile robotics, agriculture) ships its own launch files. `wojtek_rai_bringup` here is *our* equivalent, not a fork of anything upstream.
- **`whoami/` is a source directory, not a package:** it's consumed once at agent startup by `EmbodimentInfo.from_directory()`, so it belongs next to config, not inside a ROS package.
- **Extension over edit, mirroring the sibling experiment's rule 4:** if SCAN-Planner needs a RAI-compatible Tool wrapper, write a new thin adapter Tool class rather than modifying either RAI's `BaseROS2Tool` or `training/wojtek_rl/scan/`.

## Architectural Patterns

### Pattern 1: Agent-as-external-ROS-participant (not a fork of the robot stack)

**What:** The RAI agent process is just another ROS 2 participant on the same DDS domain — it subscribes to Wojtek's existing topics and publishes to `cmd_vel` exactly the way `wojtek_teleop` or a gamepad already does. No node inside `ros/src/` needs to change.
**When to use:** Always, for this milestone — it is the only pattern compatible with the "no changes to production ROS topics or the policy contract" constraint in `PROJECT.md`.
**Trade-offs:** Clean isolation, but means the agent has zero built-in awareness of the robot's state machine (armed/disarmed, standing) unless a Tool is added to read it (e.g. a `CallROS2Service` against `/wojtek/zero` or a topic-echo tool against robot state) — worth adding early so the agent doesn't issue `cmd_vel` while disarmed.

### Pattern 2: Tool-scoped safety via allow-lists, backstopped by a ROS-level arbiter

**What:** RAI's `BaseROS2Tool` already takes readable/writable/forbidden lists per tool instance — this is the first safety layer (the agent literally cannot call a tool pointed at a forbidden topic/service). The second, independent layer is a ROS-level `cmd_vel` arbiter (`twist_mux`) sitting between the agent's `cmd_vel` publisher and `wojtek_policy`'s subscription, so a misbehaving agent can be capped or overridden without touching RAI internals.
**When to use:** From day one — do not rely on the LLM's own judgement as the only safety boundary.
**Trade-offs:** Two layers cost one extra hop (agent → arbiter → policy) and one extra launch file, but decouple "can the agent call this tool" from "how fast/hard can the robot actually move," which is the layer a runaway or hallucinating LLM output cannot bypass.

**Example (illustrative, not upstream RAI code):**
```yaml
# wojtek_rai_bringup/config/twist_mux.yaml
topics:
  agent:
    topic: /wojtek/agent/cmd_vel     # RAI's PublishROS2MessageTool writes here, NOT /cmd_vel directly
    timeout: 0.5
    priority: 10
  teleop:
    topic: /wojtek/teleop/cmd_vel
    timeout: 0.5
    priority: 100                    # human teleop always wins
locks:
  estop:
    topic: /wojtek/estop
    priority: 255
```
`twist_mux` then republishes the winning input to the real `/cmd_vel` that `wojtek_policy` already subscribes to — zero changes inside `wojtek_policy` or `wojtek_bringup`.

### Pattern 3: Local planner stays between "where" and "how", regardless of who asks

**What:** `training/docs/scan-planner.md` already establishes this for the VLM-nav research track: a VLM/agent decides *where* to go, SCAN-Planner (or Nav2) decides *how to get there without hitting anything*, and only the planner's output ever reaches `cmd_vel`/the policy. The same boundary applies verbatim when RAI is the "where" decider.
**When to use:** For any RAI navigation Tool ("go to the chair") — never let the agent's tool call publish a raw straight-line `cmd_vel` displacement.
**Trade-offs:** SCAN-Planner today is sim-only, no ROS node (`training/wojtek_rl/scan/`) — it must be wrapped as a ROS 2 node (or Nav2's own local planner/costmap stack substituted) before RAI can call it as a Tool. This is real, non-trivial work and should be sized as its own phase, not folded into "agent can drive `cmd_vel`."

## Data Flow

### Walking ("walk forward")

```
Human text ("walk forward")
    ↓ ROS2HRIConnector (/from_human, HRIMessage)
ReActAgent (LLM reasoning: text → intent)
    ↓ tool call: PublishROS2MessageTool(topic=/wojtek/agent/cmd_vel)
ROS2Connector publishes geometry_msgs/Twist
    ↓
twist_mux (arbitration: agent vs teleop vs lock)
    ↓ (optional) velocity/accel clamp
/cmd_vel (existing, unchanged topic)
    ↓
wojtek_policy (50 Hz RL inference, existing, unchanged)
    ↓
MuJoCo sim actuators (ros/sim.sh) — later: real actuators
```

### Goal navigation ("go to the chair")

```
Human text ("go to the chair")
    ↓
ReActAgent
    ↓ tool call: Nav2 NavigateToPose action tool  (or a custom goal tool)
ROS2Connector (action client) → Nav2 action server / SCAN-Planner ROS wrapper
    ↓ 10 Hz replan, obstacle-aware
local planner emits /cmd_vel (same topic, same arbiter path as above)
    ↓
twist_mux → wojtek_policy → sim/robot
```

### Look ("what do you see?")

```
Human text ("what do you see?")
    ↓
ReActAgent
    ↓ tool call: GetROS2ImageTool(topic=/wojtek/camera/image_raw or .../compressed)
ROS2Connector subscribes, pulls one frame
    ↓
frame → multimodal LangChain message → VLM backend (same config.toml vendor)
    ↓
text answer
    ↓ ROS2HRIConnector (/to_human, HRIMessage) + Streamlit/CLI chat UI
```

### QoS considerations

- **Camera topic:** the sibling experiment already resolved this — use `SensorDataQoS` (BestEffort, shallow depth) and a compressed image topic; a full-rate reliable image subscription is unnecessary for a one-shot "look" tool call and would only cost bandwidth on the eventual Jetson↔RPi AP link.
- **`cmd_vel` and the arbiter:** Reliable, small-depth queues — commands are low-rate, high-value, and must not silently drop.
- **HRI text topics (`/from_human`, `/to_human`):** Reliable — a dropped chat message is a worse failure mode than a dropped camera frame.
- **Nav2 action calls:** actions carry their own QoS/feedback semantics; no extra tuning needed beyond Nav2's defaults.

## Scaling Considerations

| Scale | Architecture Adjustments |
|-------|--------------------------|
| Sim, dev GPU box (this milestone) | Agent process, LLM/VLM backend (cloud or local), and ROS 2 graph all on one machine or over localhost — no DDS-over-network concerns yet |
| Sim, dev GPU box + real RPi later (stretch) | DDS crosses the robot's own access point; camera QoS/compression already resolved above matters here |
| Onboard Jetson (next milestone) | Agent process + LLM/VLM inference move onto the Jetson; RPi 4 keeps only the 50 Hz control loop. DDS between Jetson and RPi crosses the robot's own AP — CycloneDDS discovery and camera bandwidth become real constraints; the arbiter and topic contract designed now must not assume same-host latency |

### Scaling Priorities

1. **First bottleneck (this milestone):** none expected — everything is local. The real risk is *safety*, not scale: an ungated agent `cmd_vel` reaching `wojtek_policy` directly. Solved by Pattern 2 above, before any other work.
2. **Second bottleneck (Jetson milestone):** DDS discovery and image-topic bandwidth across the AP link. Mitigate by keeping the camera topic compressed/BestEffort (already the plan) and by testing DDS discovery across the AP early rather than assuming it "just works" once the agent moves off the dev box.

## Anti-Patterns

### Anti-Pattern 1: Agent tool publishes straight to `/cmd_vel`

**What people do:** Wire `PublishROS2MessageTool` directly at `/cmd_vel` because it's the fastest way to get the demo moving.
**Why it's wrong:** Removes any backstop between LLM output and the RL policy's real-time input; also makes it impossible to arbitrate against teleop or an e-stop later without touching the agent config.
**Do this instead:** Agent publishes to an agent-specific topic (`/wojtek/agent/cmd_vel`); `twist_mux` (or equivalent) republishes to the real `/cmd_vel`. One extra launch file, permanent safety margin.

### Anti-Pattern 2: Treating a VLM "forward 1.5" style command as ground truth motion

**What people do:** Let a navigation Tool convert a natural-language displacement or bearing directly into a straight-line `cmd_vel` sequence.
**Why it's wrong:** `training/docs/scan-planner.md` measured this exact failure mode across three different VLMs (FutureNav, Qwen3-VL, Claude) — 100% collision rate on non-trivial scenes. There is no reason to expect RAI's LLM backend to behave differently; it has no depth perception in the tool-call loop either.
**Do this instead:** Route any goal/displacement tool call through a local planner (SCAN-Planner wrapped as ROS, or Nav2's costmap-based planner) before it ever reaches `cmd_vel`.

### Anti-Pattern 3: Putting the RAI overlay's ROS packages under `ros/src/`

**What people do:** For convenience, add the new RAI launch/tool packages directly to `ros/src/` so `ros/sim.sh` "just launches everything."
**Why it's wrong:** Breaks the repo's core isolation rule — `ros/deploy.sh` rsyncs `ros/src/` wholesale and could ship agent code to the RPi by accident, and it makes `wojtek_bringup`'s own build implicitly depend on RAI being present.
**Do this instead:** Keep the overlay at `experiments/rai_agent_v1/ros/src/`, launched by the experiment's own `run.sh`, exactly as `autonomous_architecture_ros2_v1` already does for its voice/brain packages.

### Anti-Pattern 4: Skipping the state-machine check before allowing agent motion commands

**What people do:** Let the agent's `cmd_vel` reach the arbiter regardless of whether Wojtek is armed/standing.
**Why it's wrong:** `wojtek_bringup`'s control loop already has a DISARMED → ZERO → STAND_UP → ARMED state machine specifically to prevent torque commands in unsafe states; an agent that doesn't know about it will "walk forward" into a disarmed robot and either no-op unpredictably or, worse, catch a race during a state transition.
**Do this instead:** Add a small read-only Tool (topic echo or service call) that lets the agent query robot state before issuing motion, and/or gate the arbiter's agent input on an ARMED signal.

## Integration Points

### External Services

| Service | Integration Pattern | Notes |
|---------|---------------------|-------|
| LLM/VLM backend (cloud or local) | `config.toml` vendor selection (Ollama, OpenAI, AWS Bedrock, others), edited via `rai-config-init` or the Streamlit configurator | Concrete choice deferred per `PROJECT.md`; keep the Tool/Connector layer backend-agnostic so a later Jetson-local model swap is a config change, not a code change |
| RAI upstream package | `pip install rai-core` (end-user path) or `uv sync` + `vcs import ros_deps.repos` + `colcon build` (source/dev path, needed for `rai_interfaces`) | Pin a release tag; RAI 2.0 replaced Poetry with `uv` and introduced the current Connector API — do not target pre-2.0 docs/examples |

### Internal Boundaries

| Boundary | Communication | Notes |
|----------|---------------|-------|
| RAI agent process ↔ existing Wojtek ROS graph | ROS 2 topics/services/actions only (`cmd_vel`, camera, optionally a state-query service) | No shared code, no shared process — this is the boundary `PROJECT.md`'s Constraints section requires |
| RAI overlay (`experiments/rai_agent_v1/`) ↔ rest of repo | One-way read-only reuse of `training/wojtek_rl/scan/` (if SCAN-Planner is wrapped) and repo docs (for whoami); never the reverse | Mirrors the sibling experiment's already-audited rule: `training/`'s test suite must pass with the experiment directory deleted |
| Agent's `cmd_vel` ↔ `wojtek_policy`'s `cmd_vel` | `twist_mux` arbiter node (new, lives in the experiment's overlay, launched alongside the agent) | The only new node standing between untrusted agent output and the real-time control loop |
| Local planner (SCAN-Planner or Nav2) ↔ agent's goal tool | Nav2 action interface (`NavigateToPose`) if Nav2 is chosen; a bespoke topic/service if SCAN-Planner is wrapped instead | SCAN-Planner is sim-only/no-ROS today (`training/docs/scan-planner.md`) — wrapping it is real work, and Nav2 is the lower-risk fallback if that wrap slips |

## Suggested Build Order

Grounded in the dependency chain above — each step is runnable and demoable on its own before the next begins:

1. **Isolated scaffold + `whoami`.** Create `experiments/rai_agent_v1/` (README, run.sh, pyproject.toml, own colcon overlay dir), install RAI (`uv sync` / `pip install rai-core`), build the `whoami` embodiment directory from Wojtek's URDF + docs, and verify `EmbodimentInfo.from_directory()` produces a sane system prompt. No ROS graph interaction yet — this validates RAI installs cleanly inside the isolation rule and that the agent "knows it is Wojtek."
2. **Text chat, no ROS.** Bring up the RAI agent with a Streamlit or CLI chat interface and a stub/no-op tool, purely to validate the LLM/VLM backend wiring (`config.toml`) end to end before any robot risk is introduced.
3. **Velocity arbiter, before any agent `cmd_vel`.** Stand up `twist_mux` (or equivalent) on the agent-facing `/wojtek/agent/cmd_vel` → real `/cmd_vel`, tested first with a manual `ros2 topic pub`, so the safety backstop exists before the agent can reach it. This is the anti-pattern-1 fix landed proactively, not retrofitted.
4. **Walking.** Wire a `PublishROS2MessageTool` (or a thin custom Tool) from the agent to `/wojtek/agent/cmd_vel`, run against `ros/sim.sh`, and validate "walk forward" moves simulated Wojtek through the arbiter and the unmodified `wojtek_policy`.
5. **Look.** Add `GetROS2ImageTool` against the sim camera topic (with `SensorDataQoS` + compressed image, per the sibling experiment's resolved QoS choice) and validate "what do you see?" against the VLM backend. Independent of navigation — can be built in parallel with step 6 if resourcing allows.
6. **Goal navigation.** The highest-effort, highest-risk step: either (a) wrap SCAN-Planner as a ROS 2 node reusing `training/wojtek_rl/scan/` read-only, or (b) stand up a minimal Nav2 stack pointed at the sim's known map/costmap, then add the agent's goal Tool on top, routed through the same arbiter from step 3. Do this last because it has the largest unresolved unknown (SCAN-Planner has no ROS node today) and everything else is independently demoable without it.
7. **Robot-state awareness (cross-cutting, land alongside step 3 or 4).** A read-only Tool exposing Wojtek's armed/standing state, so the agent can be gated or at least informed — closes anti-pattern 4.

Steps 1–3 are pure plumbing/safety and should be one phase. Steps 4–5 are independent "prove the concept" phases and can run in parallel once 3 lands. Step 6 is large enough to be its own phase with a research spike on SCAN-Planner-as-ROS-node vs. Nav2-as-fallback before committing.

## Sources

- [RAI documentation — Overview](https://robotecai.github.io/rai/tutorials/overview/)
- [RAI documentation — Tools](https://robotecai.github.io/rai/tutorials/tools/)
- [RAI documentation — Complete walkthrough](https://robotecai.github.io/rai/tutorials/walkthrough/)
- [RAI documentation — Getting started / install](https://robotecai.github.io/rai/setup/install/)
- [GitHub — RobotecAI/rai](https://github.com/RobotecAI/rai) (README, `docs/demos/manipulation.md`, config.toml, issue/commit history)
- [GitHub — RobotecAI/rai-rosbot-xl-demo](https://github.com/RobotecAI/rai-rosbot-xl-demo)
- [RAI: Flexible Agent Framework for Embodied AI, arXiv:2505.07532](https://arxiv.org/abs/2505.07532)
- [DeepWiki — RobotecAI/rai, Agent System](https://deepwiki.com/RobotecAI/rai/3-agent-system)
- [Nav2 — Velocity Smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)
- [Nav2 — Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
- [robofoundry.medium.com — twist_mux writeup](https://robofoundry.medium.com/controlling-a-robot-with-multiple-inputs-using-twist-mux-4535b8ed9559)
- This repo: `.planning/PROJECT.md`, `.planning/codebase/ARCHITECTURE.md`, `.planning/codebase/STRUCTURE.md`, `experiments/autonomous_architecture_ros2_v1/README.md` + `docs/architecture.md`, `training/docs/scan-planner.md`

---
*Architecture research for: RAI agent integration onto Wojtek's ROS 2 stack*
*Researched: 2026-09-05*
