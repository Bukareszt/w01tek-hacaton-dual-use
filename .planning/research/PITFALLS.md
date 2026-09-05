# Pitfalls Research: RAI (RobotecAI) on Wojtek's ROS 2 Jazzy Stack

**Domain:** Embodied-AI agent framework (RAI: rai core + ROS 2 tools + rai
whoami) integrated as an `experiments/` layer on top of an existing ROS 2
Jazzy quadruped with a 50 Hz RL locomotion policy, a MuJoCo sim, a CycloneDDS
robot-AP network, a public repo, and a strict `experiments/` isolation rule.
**Researched:** 2026-09-05
**Confidence:** MEDIUM-HIGH (RAI's own docs/source and this repo's existing
config are HIGH; RAI-specific GitHub issue history is thin because the
project is young — treat anything marked MEDIUM as "plan defensively," not
"confirmed bug")

## Critical Pitfalls

### Pitfall 1: RAI install/version skew (ROS distro, Python ABI, unpinned LangChain/LangGraph)

**What goes wrong:**
RAI is installed into a Python environment that doesn't match the ROS 2
Jazzy `rclpy` build, or `pip`/`uv` resolves a LangChain/LangGraph combination
that breaks at runtime. RAI's own `rai_core` dependency line pins only
`langchain-core = "^0.3"` and leaves `langgraph`, `langgraph-prebuilt`,
`langchain`, `langchain-aws`, `langchain-openai`, `langchain-community`
completely unconstrained (`"*"`). This is not hypothetical: LangGraph shipped
exactly this failure mode upstream (`langgraph-prebuilt==1.0.2` broke callers
because `langgraph` didn't pin a compatible range — langchain-ai/langgraph
issue #6363), and `langchain-community` explicitly does not follow semver,
so a routine `pip install -U` inside the experiment's venv can silently pull
in an incompatible trio months after the experiment was pinned and working.

**Why it happens:**
RAI targets "Jazzy or Humble installed and sourced" with "the same version
of Python as ROS 2" (3.10 for Humble, 3.12 for Jazzy) — `rclpy` is a compiled
extension tied to one Python ABI, so a venv built against the wrong
interpreter imports but segfaults or raises confusing `ImportError`s only
when a ROS message type is touched. Separately, `rai_core`'s own
`pyproject.toml` uses `"*"` for the LangChain/LangGraph stack, so nothing in
RAI itself protects the experiment from a downstream break; the project has
to pin the resolved set itself.

**How to avoid:**
- Build the experiment's venv with the *exact* interpreter the Jazzy
  container ships (`ros:jazzy-ros-core` → Python 3.12), the same pattern
  `training/.venv` already uses for the sibling agent experiment (`run.sh
  test` warns when `EXP_PY` isn't pointed at that interpreter).
- After first successful install, run `uv lock` (or `pip freeze`) and commit
  the lockfile inside `experiments/<rai-experiment>/`; treat any LangChain/
  LangGraph bump as a deliberate, tested upgrade, never an incidental
  transitive resolution.
- Do not `pip install -U rai_core` in place without re-running the
  experiment's test suite; the `"*"` constraints mean a fresh resolve today
  can differ from the one that worked last month even with an unchanged
  `rai_core` version pin.
- Source ROS 2 Jazzy *before* creating/activating the venv (`setup_shell.sh`
  pattern from RAI's own repo), and add `$VENV/lib/python3.12/site-packages`
  to `$PYTHONPATH` only when invoking through `ros2 run`/`ros2 launch` — not
  when running RAI's own CLI directly, which does not need it.

**Warning signs:**
- `ImportError` or a segfault the moment `rclpy` touches a real ROS message
  type (not at process start).
- `uv sync`/`pip install` silently changing `langgraph`/`langchain-core`
  versions between two runs with no `pyproject.toml` edit.
- The experiment's own `run.sh test` passing yesterday and failing today
  with a stack trace inside `langgraph_prebuilt` or `langchain_core.tools`
  with no local code change.

**Phase to address:**
Phase 1 (RAI install & environment setup) — lock the venv and the dependency
set before any agent logic is written; re-verify with a smoke test any time
the lockfile changes.

---

### Pitfall 2: rclpy executor conflicts between RAI's node and the experiment's other ROS nodes

**What goes wrong:**
RAI's `ROS2Connector` (the piece `BaseROS2Tool` and the ROS 2 tools use)
creates and spins its own `rclpy` node/executor internally. If the
experiment also needs another ROS presence in the same process — a bridge
node, the demo app's own node, or anything adapted from the sibling
`autonomous_architecture_ros2_v1` experiment (which is deliberately
node-per-model) — a second `rclpy.init()`/spin in the same process either
raises ("rcl_init called while already initialized") or one spin call
blocks the thread the other needed, so callbacks silently stop firing
(subscriptions look "connected" in `ros2 topic info` but the callback never
runs).

**Why it happens:**
`rclpy`'s default context and executor model assumes one owner per process.
Frameworks that manage their own connector (RAI) and hand-rolled bridge code
both reach for the same global unless the integration explicitly shares one
context/executor or, more simply, keeps RAI and any other ROS presence in
separate OS processes talking over topics (the pattern the sibling
experiment already chose: "node-per-model", not one Python process wearing
many ROS hats).

**How to avoid:**
- Default to one ROS 2 node/executor per OS process for this integration,
  matching the sibling experiment's settled architecture. Let RAI own its
  node fully; if a bridge/adapter node is needed (e.g., translating a
  higher-level "walk forward" into `cmd_vel`), run it as its own process and
  a its own launch entry, not imported into RAI's Python process.
  This also keeps LLM-call latency from starving another node's spin loop
  in the same thread.
- If RAI must be embedded inside an existing multi-node launch file, spin it
  on a `MultiThreadedExecutor` with an explicit callback group per node and
  test that every subscription actually delivers a message under load
  (`ros2 topic hz` on each subscribed topic while the agent is talking), not
  just that `ros2 node info` shows a connection.

**Warning signs:**
- `ros2 topic info /cmd_vel` shows a subscriber but `ros2 topic hz /cmd_vel`
  shows no traffic while the agent is issuing commands.
- Intermittent hangs that disappear when RAI and the rest of the stack are
  split into separate `ros2 run` invocations.

**Phase to address:**
Phase 1 (RAI install & environment setup), re-verified at Phase 3 (velocity
walking) when the agent first needs to publish to a live topic alongside
other nodes.

---

### Pitfall 3: DDS/QoS mismatch on camera and other sensor topics (silent, no error)

**What goes wrong:**
Wojtek's own perception stack already publishes depth/camera topics on
`qos_profile_sensor_data` (BEST_EFFORT reliability) —
`ros/src/wojtek_perception_bringup/wojtek_perception_bringup/cloud_reduce_node.py`
subscribes to `depth/camera_info` and `depth/image` with exactly that
profile. If RAI's `GetROS2ImageTool` (or any RAI ROS 2 tool) subscribes with
the client-library default (RELIABLE), the subscription is created, shows
up as "connected" in introspection, and **never receives a single message**
— ROS 2 quietly refuses to deliver across an incompatible reliability/
durability policy pair rather than erroring. The agent's "what do you see?"
tool call then times out or returns a stale/empty frame with no exception to
debug from.

**Why it happens:**
QoS compatibility in DDS is negotiated per-topic at discovery time; a
RELIABLE subscriber and a BEST_EFFORT publisher simply never match, and
neither side logs a clear error by default (only `ros2 doctor`/`ros2 topic
info -v` or a `rclpy` QoS-event callback surfaces it). Generic tool
implementations that don't know a given robot's camera QoS convention
default to RELIABLE because that is `rclpy`'s library default for most
message types.

**How to avoid:**
- Before wiring the "look" tool, run `ros2 topic info -v <image topic>` on
  the sim's actual camera/perception topics and record their QoS (this repo
  already standardizes on `qos_profile_sensor_data` for camera/depth/point
  cloud — match it, don't guess).
- If RAI's ROS 2 tools accept a QoS override parameter, set it explicitly to
  `qos_profile_sensor_data` for every camera/image/point-cloud topic the
  agent reads; do not rely on the tool's default.
- Add a smoke test that publishes one image on the real topic/QoS and
  asserts the agent's image tool actually returns non-empty bytes — this
  catches the silent-mismatch class of bug that a "the topic exists" check
  cannot.

**Warning signs:**
- The "look" tool returns "no image available" or times out even though
  `ros2 topic list` shows the camera topic actively publishing.
- `ros2 topic info -v <topic>` shows one publisher and one subscriber with
  different QoS reliability/durability values.

**Phase to address:**
Phase 5 (image/VLM look tool) — verify against the sim's real QoS before
declaring the tool "working," not just against a hand-rolled test publisher
with default QoS (which would falsely pass).

---

### Pitfall 4: CycloneDDS domain/RMW mismatch and multi-host discovery over the robot AP

**What goes wrong:**
This repo already runs a two-machine DDS topology with real gotchas baked
into its own config: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
and a documented multicast-vs-unicast split (`cyclonedds.xml` for the
default/wifi-AP case, `cyclonedds_link.xml` with hardcoded peer IPs for the
cable case) because multicast discovery does not reliably cross the robot's
own access point. If the RAI experiment's process is launched without these
same environment variables — a bare `python3 rai_agent.py` outside the dev
container, or a fresh shell that never sourced the robot's env block — it
silently lands on domain 0 with the default RMW and **discovers nothing**:
no error, just an agent that "sees" zero topics.

**Why it happens:**
`ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` are process-environment settings,
not something `ros2 launch` or RAI's Python API can infer; every process
that should discover the robot's nodes must set them identically. RAI's own
docs are silent on DDS/QoS configuration entirely (confirmed by reading
`setup/install/`), so nothing in RAI itself will warn about a domain
mismatch — from RAI's point of view, an empty topic list is a valid, quiet
answer.

**How to avoid:**
- Run the RAI experiment inside the existing dev container / via
  `ros/sim.sh`'s environment, not a bespoke host shell, so it inherits
  `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and
  `CYCLONEDDS_URI` automatically.
  If the agent process must run standalone (e.g. as a separate Jetson
  process later), export all three explicitly at the top of its launcher,
  the same pattern `robot.py` already uses for the RPi SSH bridge.
- Add a startup self-check to the experiment's own launcher: call
  `GetROS2TopicsNamesAndTypesTool` (or `ros2 topic list`) immediately after
  init and fail loudly if the expected topics (e.g. `cmd_vel`, the camera
  topic) are absent, instead of letting the agent proceed and report "I see
  nothing" as if it were a scene-content answer.
- When the physical-robot milestone arrives, decide up front whether the
  Jetson uses the multicast profile (works over the AP today) or needs the
  unicast `cyclonedds_link.xml` fallback, and document the choice next to
  the existing RPi note rather than rediscovering the multicast-over-AP
  flakiness from scratch.

**Warning signs:**
- `GetROS2TopicsNamesAndTypesTool` (or `ros2 topic list` run from the same
  shell) returns an empty or drastically short list.
- The agent behaves identically whether or not the sim is even running.

**Phase to address:**
Phase 1 (RAI install & environment setup) for the env-var inheritance;
Phase 3/4 (walking/navigation) for the "fail loudly on missing topics"
self-check, since that's where a silent empty-topic-list first causes a
confusing behavioral symptom rather than an install-time error.

---

### Pitfall 5: Unbounded `cmd_vel` control — no clamp, no deadman, no contention handling

**What goes wrong:**
An LLM agent with write access to `cmd_vel` can (a) emit a velocity outside
any value the policy was trained/tuned for, (b) keep publishing after the
conversation moves on or the process should have stopped (no timeout means
the robot keeps walking on the last command), (c) fire multiple commands in
quick succession faster than the 50 Hz control loop can settle into, and
(d) fight with a human teleop/gamepad publisher on the same topic with no
arbitration, so whichever one published last "wins" nondeterministically.
RAI's own ROS 2 tool model has a `forbidden`/`writable` allowlist mechanism
at the tool level, but it enforces *which* topics a tool may touch, not
*what values or rate* it may publish — clamping and timeouts are the
integration's responsibility, not something RAI provides out of the box.

**Why it happens:**
`PublishROS2MessageTool`-style generic tools pass whatever the LLM decided
to publish straight to the topic; the LLM's own judgment about "safe"
velocity is not grounded in the policy's actual training envelope (e.g. the
domain-randomization velocity range from `training/docs/configuration.md`).
LLM tool calls also have no built-in liveness contract with ROS the way a
joystick's "stop publishing = robot understands zero" convention does — an
agent that stops responding (crashed, rate-limited, waiting on a slow model
call) leaves whatever it last commanded in effect indefinitely unless
something else times it out.

**How to avoid:**
- Do not let the raw generic `PublishROS2MessageTool` write `cmd_vel`
  directly. Wrap velocity commands in a dedicated tool/node that (1) clamps
  `vx`/`vy`/`wz` to the policy's trained/validated envelope, (2) republishes
  at a fixed rate with a hard timeout (e.g. re-zero `cmd_vel` if no new
  agent instruction arrives within N hundred ms — the same "stop = safe"
  convention teleop already relies on), and (3) is the *only* path from the
  agent to `cmd_vel`.
- Use RAI's own `forbidden`/`writable` topic list to make this explicit at
  the tool-registration layer: forbid the raw `cmd_vel` write path for
  generic tools and only expose the wrapped, clamped tool.
- Decide and document arbitration with teleop/gamepad before this milestone
  claims "done": either the agent and teleop are mutually exclusive modes
  (an explicit switch), or teleop always overrides the agent (matching the
  existing safety posture that a human joystick outranks any automated
  control).
- LLM round-trip latency is seconds, the policy runs at 50 Hz — treat every
  agent-issued command as a *sparse, high-level* setpoint the wrapper holds
  and re-publishes, never as something the agent is expected to stream at
  control-loop rate.

**Warning signs:**
- Robot keeps moving after the chat session is idle or the process is
  killed with `Ctrl+C` mid-response.
- `cmd_vel` values in a bag capture exceed the range ever seen from teleop
  or the training domain-randomization command range.
- Two publishers on `cmd_vel` observed via `ros2 topic info` during a
  session where both teleop and the agent were touched.

**Phase to address:**
Phase 3 (velocity walking) — this is the single highest-severity pitfall in
this integration because it is the first phase where the agent gets write
access to actuation; do not proceed to navigation (Phase 4) until the
clamp/timeout wrapper exists and is tested.

---

### Pitfall 6: Navigation hallucination — claiming arrival without odometry evidence, and straight-line commands into furniture

**What goes wrong:**
Two related failure modes, both already documented and measured in this
repo's own `training/docs/scan-planner.md`: (1) a VLM/agent issuing a
mid-level command like "forward 1.5" that gets executed as a literal
straight line walks the robot into furniture — measured at 100% collision
rate (6/6 and 8/8 episodes) in the "straight march" baseline before
SCAN-Planner existed; and (2) an agent that reports "I've arrived" or "goal
reached" based on its own belief about elapsed time/commands issued rather
than actual odometry/pose feedback, which is a generic LLM-agent failure
mode (confident narration decoupled from ground truth) that gets worse the
longer the agent goes without a tool call that returns real robot state.

**Why it happens:**
An LLM's job is language, not depth-channel obstacle checking; nothing
about the mid-level command "forward 1.5" implies "watch for obstacles while
executing it" unless something in the stack does that watching. Likewise,
nothing forces an agent to ground an "arrived" claim in a fresh pose/
odometry reading unless the tool contract requires it — a chat-style agent
will happily narrate success from its own turn history if no tool result
contradicts it.

**How to avoid:**
- Reuse the existing SCAN-Planner (or an equivalent collision-aware local
  planner) between the agent's mid-level navigation commands and the
  locomotion policy — this repo already measured that straight-line
  execution fails at 0% success in cluttered scenes and SCAN-Planner
  recovers to 100%/90% success (room/apartment). Do not let the agent's
  navigation tool publish raw straight-line `cmd_vel` sequences toward a
  goal; route through the planner exactly as `wojtek_rl/scan/executor.py`
  already does for the VLM navigators.
  Note this integration is sim-only and ground-truth-pose-fed today
  (`scan-planner.md`'s own "Known limits": "no ROS node, no odometry
  model" for SCAN-Planner) — porting it into the RAI ROS 2 path means
  deciding how it gets real odometry, not assuming the sim behavior
  transfers unchanged.
- Make "did I arrive" a tool call that returns actual pose/goal-distance
  from ROS (odometry or the planner's own progress state), never something
  the agent is allowed to assert from memory. The system prompt/whoami
  should state explicitly that arrival is only true when the tool result
  says so.
- Carry over the SCAN-Planner give-up semantics already tuned in this repo
  (blocked after "no movement for 1.6 s", not-progressing after "moved 8 s
  without getting nearer") — a naive single timeout aborts every legitimate
  detour, exactly the failure `scan-planner.md` documents fixing.

**Warning signs:**
- Agent says "arrived" while the robot's actual position (checked
  independently) is meters from the goal.
- Any code path that turns an agent's "go there" into more than one
  straight-line `cmd_vel` segment without a replanning/obstacle-check step
  in between.

**Phase to address:**
Phase 4 (navigation) — this phase should not be marked complete without
re-running (or at minimum re-reading and consciously accepting the limits
of) the room/apartment collision benchmarks already established for
SCAN-Planner, adapted to however odometry is sourced in the RAI/ROS path.

---

### Pitfall 7: Image/VLM pipeline pitfalls — stale frames, oversized base64 payloads, wrong encoding

**What goes wrong:**
A "what do you see?" tool call reads whatever frame happens to be latched/
buffered rather than a fresh one, so the agent describes a scene from
several frames (or seconds) ago — misleading right after the robot has just
moved or turned. Separately, sending a full-resolution image as base64 in
every tool-result message balloons token/context cost for cloud models and
adds real latency, and a wrong or unhandled encoding (raw `sensor_msgs/Image`
vs. `CompressedImage`, RGB vs. BGR, wrong `cv_bridge` encoding string) either
throws deep inside the tool or silently produces a color-shifted/garbled
image the VLM then "sees" incorrectly with no error surfaced to the
operator.

**Why it happens:**
ROS image topics are commonly published as raw `sensor_msgs/Image`; a
generic image tool that grabs "the latest message" without checking the
message's own timestamp against "now" cannot tell a genuinely fresh frame
from a stale QoS-cached one (especially relevant if a `TRANSIENT_LOCAL`
durability profile is in play anywhere). Multimodal LLM APIs also charge by
image size/resolution, so an integration that pipes the sim's full camera
resolution straight through pays needlessly in both cost and round-trip
time, and this repo's compute-reality note ("Agent compute is a dev/GPU box
now and an onboard Jetson later") makes local-model latency here a real
constraint, not a nice-to-have optimization.

**How to avoid:**
- Check the image message's `header.stamp` against the current time before
  handing it to the VLM; reject/re-fetch if it's older than one control
  cycle's worth of staleness for the current task (a few hundred ms for
  "what's in front of me" style queries).
- Downscale/compress before base64-encoding for the model call — the sim's
  RealSense-equivalent resolution is more than any current multimodal model
  needs for a scene description; pick a fixed downscale target once and
  reuse it everywhere the "look" tool is invoked.
- Verify the RGB/BGR and encoding string end to end with a known test image
  (a colored marker in the scene) before trusting VLM descriptions of color
  or object identity — this is the kind of bug that produces plausible but
  wrong answers rather than a crash.

**Warning signs:**
- VLM describes something that was true a second or two ago but isn't now
  (e.g. describes an obstacle the robot has already walked past).
- Multimodal API calls with unexpectedly high token counts / cost per "look"
  invocation.
- Colors in the VLM's description are consistently inverted/wrong versus
  what a human sees in the same frame (classic RGB/BGR swap).

**Phase to address:**
Phase 5 (image/VLM look tool).

---

### Pitfall 8: `rai whoami` built from thin or wrong embodiment docs — agent misidentifies its own capabilities

**What goes wrong:**
`rai_whoami` builds a system prompt / RAG vector database from whatever
directory of docs, images, and URDF it's pointed at
(`build_whoami.py <dir>/ --build-vector-db`). If that directory is thin
(just the raw URDF, no capability/constraint prose) or stale (describes a
prior gait, a different DOF count, or capabilities from the out-of-scope
list like tricks/jumping), the agent will answer confidently and wrongly
about what Wojtek can do — e.g. claiming it can jump or perform a D-pad
trick this milestone explicitly excludes, or misjudging its own velocity/
turning limits when planning a navigation command.

**Why it happens:**
RAG-backed embodiment is only as good as the corpus; a URDF alone encodes
geometry and joint limits, not behavioral facts like "walking commands go
through a 50 Hz RL policy with an action filter" or "this milestone has no
jump/trick/skill tools." Nothing in `rai_whoami` validates that the
generated system prompt matches the actual tool set the agent has been
given — a whoami doc claiming capabilities the tool registry doesn't expose
is a documentation bug, not a code error, so it never throws.

**How to avoid:**
- Write the whoami source docs deliberately, not just point `build_whoami.py`
  at the existing URDF/MuJoCo XML: include this milestone's actual scope
  (walk, navigate, look — no tricks, no voice) and Wojtek's real constraints
  (velocity envelope, RL policy control, no getup/jump this iteration) as
  first-class prose, mirroring what's already true in `CLAUDE.md`/
  `PROJECT.md`'s Out of Scope section.
  Do not hand-edit the generated MJX XML per the repo rule, but the whoami
  *docs* directory is exactly the human-authored prose layer that belongs
  in the experiment, separate from the generated model files.
- After building, spot-check the agent with capability-boundary questions
  ("can you jump?", "can you spin in place instantly?") and confirm answers
  match the actual tool registry, not just plausible robot-dog lore.
- Re-run `build_whoami.py --build-vector-db` whenever the tool set or the
  velocity envelope changes; treat a stale whoami DB the same as a stale
  generated XML — a drift risk the repo already has a documented pattern
  for (CONCERNS.md's "Generated XML drift risk").

**Warning signs:**
- Agent claims capabilities not in its tool registry (tricks, jumping,
  voice) or describes the wrong DOF count/gait.
- Agent's stated velocity/turning limits don't match the values actually
  clamped in the `cmd_vel` wrapper (Pitfall 5).

**Phase to address:**
Phase 2 (whoami / embodiment) — build and capability-boundary-test whoami
before Phase 3 gives the agent write access to `cmd_vel`, so the agent's
self-model and its actual permissions are verified in sync.

---

### Pitfall 9: Secrets leakage in RAI config for a public repository

**What goes wrong:**
RAI's own configuration convention (a `config.toml`-style file holding model
provider settings) is a natural place to also drop an API key "just to get
it working," and that file gets committed because it looks like ordinary
project config rather than a secret. This repo is explicitly public
(`github.com/machinekind/w01-tek`) with a standing rule against committing
any credential, and `CONCERNS.md` already flags this exact class of risk
generally ("Public repository with secret-leakage surface").

**Why it happens:**
Framework quickstarts usually show the fastest path to a working demo,
which is often "paste your key into the config file," and a developer
copying that quickstart under time pressure forgets the repo's own
`.gitignore`/`.env` convention doesn't automatically cover a new file
pattern RAI introduces.

**How to avoid:**
- Follow this repo's existing pattern exactly, not RAI's default quickstart:
  the model API key is an environment variable declared at the top of the
  experiment's launcher script and filled from the gitignored `.env` (see
  `.env.example`, and the `UBUNTU_PRO_TOKEN`/`WOJTEK_AP_PSK` handling in
  `ros/deploy.sh` as the pattern to copy).
- If RAI's config format requires a file (not just env vars) for model
  backend selection, generate that file at launch time from environment
  variables into a gitignored path, and commit only a `.example`/template
  version with placeholder values — mirroring `.env.example`.
- Add the new config file's real path (not just `.env`) to `.gitignore`
  explicitly before the first commit that introduces it, and grep the
  initial commit diff for anything that looks like a key pattern before
  pushing (the existing `test_job_scripts.py` FORBIDDEN_RE/SITE_VALUE_RE
  pattern is the model to extend, per `CONCERNS.md`'s own recommendation).

**Warning signs:**
- Any committed file under the RAI experiment directory containing a string
  matching a known API key prefix (`sk-`, `AKIA`, etc.) or a bare hostname/
  IP that isn't a placeholder.
- A config file present in `git status` that wasn't explicitly added to
  `.gitignore` when the experiment was scaffolded.

**Phase to address:**
Phase 1 (RAI install & environment setup) — decide the secrets-handling
pattern before the first working demo exists, since that's exactly the
moment a "just get it running" key paste happens.

---

### Pitfall 10: Experiments-isolation violations — RAI code creeping into `ros/` or being imported back

**What goes wrong:**
Two directions of violation are both plausible here: (1) the RAI experiment
imports something from `ros/src/` beyond read-only reuse of `wojtek_rl`
helpers (e.g. directly importing a `wojtek_policy` internal to short-circuit
ROS message passing), which the project's own constraints explicitly
forbid; or (2) something convenient built for the RAI experiment (a launch
file variant, a small utility) gets added to `ros/src/wojtek_bringup` or
`ros/sim.sh` "just this once" to make testing easier, which makes the
experiment reachable by `ros/deploy.sh` even though nothing intended that.
`CONCERNS.md` already documents this exact fragility class for the sibling
experiment: "if an experiment depends on and imports code from `ros/src/`,
a later refactor of `ros/src/` breaks the experiment silently," and the
inverse (experiment code leaking into `ros/`) is worse — it can reach the
physical robot.

**Why it happens:**
The friction of going through ROS topics/services (rather than a direct
Python import) is real, and "just import the function" is the path of
least resistance during iteration — especially under time pressure to get
the sim demo working. Nobody has to intend a violation; a single convenience
import silently changes the dependency graph.

**How to avoid:**
- Enforce the same rule the sibling experiment states explicitly: the
  dependency arrow points one way (experiment → `training/`'s public
  helpers only), never `ros/`'s internals, and `ros/sim.sh`/
  `docker/compose.yaml`/`ros/deploy.sh` stay untouched by this integration.
- Keep the RAI experiment's own ROS packages outside `ros/src/` entirely (as
  the sibling experiment already does), so `ros/deploy.sh`'s
  `rsync ros/src/` + `--packages-up-to wojtek_bringup` build cannot ship RAI
  code to the robot even by accident.
- Before merging any change, check: "does `ros/`'s own test suite (or
  `deploy.sh` dry-run) still pass with this experiment directory deleted?"
  — the sibling experiment's README states this as the concrete acceptance
  bar and it transfers directly.
- If the CI/pre-commit check `CONCERNS.md` recommends ("Python import guard
  or CI check" against `experiments/` imports into `ros/`/`training/`) does
  not exist yet, add a lightweight grep-based guard test for it as part of
  this milestone, rather than relying on discipline alone.

**Warning signs:**
- Any `from ros.` or `import wojtek_bringup`/`wojtek_policy` inside the RAI
  experiment's Python beyond documented `wojtek_rl` reuse.
- A new file under `ros/src/` whose only purpose is to support the RAI
  experiment (a "rai bridge" node, an agent-specific launch arg) rather than
  a generally-useful production capability.
- `ros/deploy.sh --dry-run` (once it exists — see `CONCERNS.md`'s own
  recommendation) showing any RAI-related file in its sync set.

**Phase to address:**
Phase 1 (RAI install & environment setup) — establish the import boundary
and, ideally, the guard test, before any other phase adds code that could
violate it.

---

### Pitfall 11: Dev-container bloat — RAI's optional extras pull in torch/CUDA/vision-model weights

**What goes wrong:**
RAI's own `pyproject.toml` groups its dependencies into extras, and the
`openset` group alone declares `torch ^2.3.1`, `torchvision ^0.18.1`, plus
Grounding DINO and SAM-2 model dependencies — a multi-GB addition dragged in
by a single extras flag, on top of whatever `torch`/CUDA stack (if any) the
image already carries for training-adjacent tooling. The project's explicit
scope for this milestone excludes `rai_perception`, `rai_nomad`, `rai_bench`,
`rai_sim`, and `rai_finetune` — but "core + ROS tools + whoami only" is easy
to violate accidentally if a later dependency (or a copy-pasted install
command from RAI's own docs/examples) pulls in an extras group nobody meant
to install.

**Why it happens:**
Extras are opt-in by design, but example commands in a framework's own
walkthrough docs often show the "kitchen sink" install for demo purposes,
and it's easy to copy that command rather than the minimal one. Because the
project's dev container is explicitly *not* a slimmed-down production image
(per the existing `ros/docker/Dockerfile` comment: "This is a dev/build
environment... ros-dev-tools stay installed"), there's less friction
stopping an oversized dependency from going unnoticed until a rebuild takes
noticeably longer or a disk-space error appears.

**How to avoid:**
- Install only `rai_core` (+ its ROS 2 tools) and `rai_whoami` for this
  milestone; do not install the `openset`, `simbench`, `nomad`, or `s2s`
  extras groups — they map directly to the packages already marked
  out-of-scope in `PROJECT.md`.
- Pin the install command in the experiment's own `README.md`/`run.sh` so
  "how to install RAI for this experiment" is unambiguous and doesn't
  silently drift toward a fuller install over time.
- If GPU/CUDA support is later needed for a local model backend (the
  project's "model backend configurable" requirement), make that an
  explicit, separately-justified addition — not a side effect of installing
  an unrelated RAI extras group.

**Warning signs:**
- `experiments/<rai>/` venv or container image size jumps by multiple GB
  after an "RAI install" step that was expected to be lightweight.
- `pip list`/`uv pip list` inside the experiment's venv shows `torch`,
  `torchvision`, or vision-model checkpoint downloads nobody intended.

**Phase to address:**
Phase 1 (RAI install & environment setup) — decide and document the minimal
install set before anyone runs a "just get RAI working" command from
upstream docs.

---

### Pitfall 12: Test strategy mistakes — no model-free unit tests, or tests that silently require a live LLM key

**What goes wrong:**
Two mirror-image failures: (1) the RAI experiment ships without any
model-free test coverage, so nothing catches a broken tool wrapper, a
misconfigured QoS override, or a `cmd_vel` clamp regression until a human
manually runs the sim; or (2) tests exist but quietly require a real LLM API
key/network call to pass, so CI (or another contributor without a key)
either skips them silently or fails for a reason unrelated to the code under
test. This repo already has a strong, explicit convention against both
failure modes: `training/`'s `tests/unit` vs `tests/integration` split is
enforced "by construction" (a guard test scans for env-instantiating imports
in `tests/unit`), and the sibling `autonomous_architecture_ros2_v1`
experiment ships 263 model-free tests with no GPU, no ROS runtime, and no
env instantiation.

**Why it happens:**
Testing an LLM-agent integration end-to-end feels like it *requires* a real
model call, so it's tempting to either skip unit-level coverage entirely
("it only makes sense to test this live") or write tests that call the real
API and gate them behind an environment variable that's easy to forget is
required — which then silently no-ops in CI or for a teammate who hasn't
set it.

**How to avoid:**
- Mirror the sibling experiment's pattern directly: model-free unit tests
  for everything that doesn't require an actual LLM response — the
  `cmd_vel` clamp/timeout wrapper (Pitfall 5), the QoS override wiring
  (Pitfall 3), the whoami capability-boundary content (Pitfall 8), the
  import-boundary guard (Pitfall 10) — using a fake/mock LLM response or a
  scripted tool-call sequence instead of a live model.
  Keep it in the same `run.sh test` shape the sibling experiment already
  uses (`./experiments/<name>/run.sh test`), so the root `CLAUDE.md`'s
  "Anything under `experiments/`" validation step keeps working unmodified.
- For the small number of things that genuinely need a live model (does the
  agent actually understand "go to the chair"), keep those explicitly
  separate (e.g. an `integration`/`live` marker) and make the requirement
  loud — fail with a clear "LLM_API_KEY not set, skipping live tests" rather
  than a quiet pass/skip that looks like coverage.
- Do not let CI (if/when this repo adds it for `experiments/`) depend on a
  real API key being present; live tests are a manual/pre-release gate, not
  part of the routine "does this change work" check.

**Warning signs:**
- `git grep` for `os.environ["...API_KEY"]` or similar directly inside a
  test file with no guard/skip logic around it.
- A test suite that reports "0 failed" but also "0 collected" when run
  without network access or an API key set.
- New tool/wrapper code (clamp, QoS override, whoami build) with no
  corresponding test at all.

**Phase to address:**
Every phase, enforced from Phase 1 onward — this is a process/convention
pitfall, not a single-phase technical one; the roadmap should require each
phase's tools/wrappers ship with model-free tests before merge, exactly as
the rest of this repository already does.

## Technical Debt Patterns

| Shortcut | Immediate Benefit | Long-term Cost | When Acceptable |
|----------|-------------------|-----------------|------------------|
| Let the generic `PublishROS2MessageTool` write `cmd_vel` directly instead of building a clamp/timeout wrapper | Faster first demo of "agent makes robot walk" | Unbounded/unsafe commands (Pitfall 5); rework once safety is taken seriously | Never beyond a throwaway local spike; must not survive into the phase's "done" state |
| Point `build_whoami.py` at the raw URDF/MJX XML with no authored prose | Whoami "works" immediately | Misidentified capabilities (Pitfall 8) surface later as confusing agent behavior, not a build error | Only for a first smoke test of the pipeline, never as the shipped embodiment |
| Install RAI with its full example/quickstart command (all extras) | One command works everywhere in docs | Multi-GB dev-container bloat (Pitfall 11), disk/build-time cost | Never for this milestone; acceptable only if a future milestone deliberately needs `openset`/`nomad` |
| Skip the QoS override on image topics and rely on defaults | Tool "just works" against a hand-rolled test publisher | Silent zero-message subscription against the real camera topics (Pitfall 3) | Never — the mismatch is invisible until tested against the real topic, so there's no safe "temporary" version |
| Unpin `rai_core`/LangChain/LangGraph versions ("whatever resolves") | No upfront pinning effort | Reproducibility breaks (Pitfall 1) on the next fresh install, possibly months later | Never past the first successful install — lock immediately after |

## Integration Gotchas

| Integration | Common Mistake | Correct Approach |
|-------------|----------------|-------------------|
| RAI ↔ ROS 2 Jazzy `rclpy` | Building the venv with the wrong Python (not 3.12) or without sourcing Jazzy first | Source ROS 2 Jazzy, then create the venv with the exact `python3.12` the container ships; verify with a real `rclpy` message import, not just `import rclpy` |
| RAI ↔ existing camera/perception topics | Trusting RAI's default (RELIABLE) QoS on image subscriptions | Explicitly set `qos_profile_sensor_data` (BEST_EFFORT) to match `wojtek_perception_bringup`'s existing publishers |
| RAI ↔ CycloneDDS multi-host network | Running the agent process in a shell that never sourced `ROS_DOMAIN_ID=42`/`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Run inside the existing dev container/`ros/sim.sh` environment, or export the same three env vars explicitly in any standalone launcher |
| RAI ↔ existing `cmd_vel` publishers (teleop/gamepad) | No arbitration between agent and teleop; last publisher silently wins | Decide and implement an explicit mode switch or teleop-always-overrides policy before Phase 3 is called done |
| RAI ↔ SCAN-Planner / mid-level navigation | Wiring the agent's navigation tool to publish straight-line `cmd_vel` toward a goal | Route every navigation command through the collision-aware local planner, as `wojtek_rl/scan/executor.py` already does for other VLM navigators |
| RAI ↔ model API keys | Committing a RAI `config.toml` with a real key because it "looked like project config" | Generate any required config file at launch from env vars sourced from the gitignored `.env`, matching the `ros/deploy.sh` secrets pattern |

## Performance Traps

| Trap | Symptoms | Prevention | When It Breaks |
|------|----------|------------|-----------------|
| Full-resolution image base64 on every "look" call | High latency and/or high per-call token cost from the multimodal API | Downscale/compress before encoding; fetch only on explicit "look" tool calls, not every turn | Immediately noticeable in cloud-model cost/latency; becomes a hard blocker once the agent moves to an onboard Jetson with a smaller local model |
| Treating agent-issued `cmd_vel` as a stream to publish every LLM token/response | Policy sees jittery, out-of-envelope command changes at a rate the 50 Hz control loop wasn't tuned for | Treat agent commands as sparse setpoints; a wrapper holds and re-publishes at a fixed safe rate | Shows up as gait instability or command starvation the moment LLM latency (seconds) meets the control loop (50 Hz) |
| Re-running `build_whoami.py --build-vector-db` (Faiss index build) on every process start | Slow agent startup, especially with a large embodiment doc corpus | Build once, cache the vector DB artifact, rebuild only when the source docs change | Noticeable once whoami docs grow past a handful of files; worse on Jetson-class compute later |
| Unbounded conversation history sent to the LLM every turn | Growing per-call cost/latency over a long session; eventually context-limit failures | Cap/summarize history, especially tool-call results (image bytes, topic dumps) that shouldn't linger in context | Breaks first on long "look → describe → navigate → look again" loops where image payloads accumulate |

## Security Mistakes

| Mistake | Risk | Prevention |
|---------|------|------------|
| API key pasted into RAI's own config file instead of routed through `.env` | Public-repo credential leak (this repo is public) | Follow the existing `.env`/`ros/deploy.sh` env-var pattern; generate any RAI config file at launch time from env vars, gitignore the real file |
| Exposing all ROS 2 topics/services as writable to the agent's tools by default | Agent (or a prompt-injected instruction from VLM-described scene text) could write to an unintended topic/service | Use RAI's `writable`/`forbidden` topic-list mechanism explicitly; default-deny, allowlist only `cmd_vel` (via the clamp wrapper) and the specific camera/perception topics needed |
| Treating scene descriptions from the VLM as trusted instructions | A description containing adversarial text (e.g. a sign in the scene) could be interpreted as a command if pipelines aren't careful about what counts as user instruction vs. observed data | Keep tool-result data (image descriptions, sensor readings) clearly separated from user instructions in the prompt structure; don't let observed-scene text re-enter as an actionable command without a human/instruction-level check |
| No confirmation-prompt equivalent for anything that could reach the physical robot later | Repo already flags a related gap generally (`CONCERNS.md`: "Deployment authorization gaps") | Keep this integration entirely sim-scoped this milestone (per `PROJECT.md`'s Out of Scope); when the Jetson/physical milestone arrives, require the same human-authorization posture `CLAUDE.md` already mandates for policy deployment |

## UX Pitfalls

| Pitfall | User Impact | Better Approach |
|---------|-------------|-------------------|
| Agent narrates "walking to the chair" / "arrived" with no grounding in real pose feedback | User trusts a false success/failure report (Pitfall 6) | Every status claim about position/arrival must come from a tool result, never from the agent's own turn-history narrative |
| Agent answers "what can you do?" from generic robot-dog knowledge instead of this build's actual tool set | User expects capabilities (jumping, voice) that don't exist this milestone | Whoami content and system prompt must state the actual, current tool registry (Pitfall 8), including explicit "not available yet" for tricks/voice |
| Silent, multi-second delay between a text instruction and the robot moving, with no acknowledgment | User assumes the command was dropped/ignored | Emit an immediate lightweight ack ("on it") separate from the actual `cmd_vel` publish, so LLM latency doesn't read as a hang |
| Chat interface allows arbitrarily long or off-topic instructions that get silently dropped by tool constraints | User doesn't know why "do a backflip" produced no reaction | Have the agent explicitly state when a request maps to no available tool (out-of-scope trick/voice) rather than silently no-op-ing |

## "Looks Done But Isn't" Checklist

- [ ] **"RAI installed and runnable"**: Often missing a locked dependency set — verify a *second*, from-scratch install (fresh venv, no cache) reproduces the same `langchain`/`langgraph` versions.
- [ ] **"Agent can walk the robot"**: Often missing the clamp/timeout wrapper — verify the robot stops if the agent process is killed mid-response, and that commanded velocities never exceed the trained envelope.
- [ ] **"Agent can look"**: Often missing a real-QoS check — verify against the sim's actual camera topic and QoS, not a hand-rolled test publisher with default QoS.
- [ ] **"Agent can navigate"**: Often missing collision-awareness — verify against a cluttered scene (not an empty room) and confirm it isn't silently doing straight-line `cmd_vel` toward the goal.
- [ ] **"whoami built"**: Often missing capability-boundary accuracy — verify the agent doesn't claim tricks/jumping/voice this milestone excludes.
- [ ] **"No secrets in the repo"**: Often missing coverage of a *new* file pattern RAI introduces (its own config format) — verify `.gitignore` covers it, not just `.env`.
- [ ] **"Isolation respected"**: Often missing a guard test — verify `ros/`'s own tests still pass with the experiment directory deleted, not just "no import errors observed."
- [ ] **"Tests pass"**: Often missing model-free coverage of the risky parts (clamp wrapper, QoS override, whoami content, import boundary) — verify test files actually exercise these, not just the LLM happy path with a live key.

## Recovery Strategies

| Pitfall | Recovery Cost | Recovery Steps |
|---------|----------------|-----------------|
| Unpinned LangChain/LangGraph broke on a fresh install | LOW | Roll back to the last known-good lockfile; re-pin explicit versions in the experiment's `pyproject.toml`/`uv.lock`; re-test before touching the pin again |
| Agent kept publishing `cmd_vel` after a crash (no timeout) | MEDIUM | Add the clamp/timeout wrapper immediately (should have existed pre-Phase-3); until then, keep a manual e-stop/teleop override always available during any agent session |
| QoS mismatch silently dropped all image messages | LOW | Add the explicit QoS override; re-run the "look" tool against the real topic to confirm delivery, not just absence of an exception |
| Whoami produced a confidently wrong capability claim in a demo | LOW | Rebuild the vector DB with corrected embodiment docs; add the specific wrong claim as a regression case in the capability-boundary test |
| Experiment code was found importing `ros/` internals | MEDIUM | Refactor the import to go through a ROS topic/service/action instead; add the import-boundary guard test so the same violation can't reoccur silently |
| Docker image ballooned from an unintended RAI extras install | LOW | Uninstall the extras group, rebuild, and pin the exact install command in the experiment's `README.md`/`run.sh` so it can't drift again |
| A "test suite" turned out to silently require a live API key and wasn't actually running in CI | MEDIUM | Split into model-free unit tests (mock/scripted LLM responses) and an explicitly-marked live/manual suite; fix the false sense of coverage before adding more tests on top of it |

## Pitfall-to-Phase Mapping

| Pitfall | Prevention Phase | Verification |
|---------|-------------------|----------------|
| 1. RAI install/version skew | Phase 1 (install & env setup) | Fresh from-scratch install reproduces the locked dependency set; `rclpy` message round-trip works, not just `import rclpy` |
| 2. rclpy executor conflicts | Phase 1, re-verified Phase 3 | `ros2 topic hz` shows real traffic on every subscribed topic while the agent is active, not just "connected" in `ros2 node info` |
| 3. Image/sensor QoS mismatch | Phase 5 (look tool), designed in Phase 1 | Look tool returns non-empty data against the sim's real camera topic and QoS, not a hand-rolled test publisher |
| 4. CycloneDDS domain/multi-host discovery | Phase 1 (env setup) | Agent process, launched exactly as intended for this phase, lists the expected topics on startup; fails loudly if it doesn't |
| 5. Unbounded `cmd_vel` control | Phase 3 (velocity walking) | Robot returns to zero command within the timeout window after the agent process is killed; commanded values stay within the trained envelope in a logged bag |
| 6. Navigation hallucination / straight-line collisions | Phase 4 (navigation) | Re-run (or consciously re-accept) the room/apartment collision benchmark logic already proven for SCAN-Planner, with real (not oracle) pose if that's how this phase sources it |
| 7. Image/VLM pipeline pitfalls | Phase 5 (look tool) | Frame timestamp freshness check passes; a known-color test object round-trips correctly through the VLM |
| 8. Whoami misidentification | Phase 2 (whoami/embodiment) | Capability-boundary Q&A test set (tricks/voice/jump = "not available") passes before Phase 3 begins |
| 9. Secrets leakage | Phase 1 (install & env setup) | `git diff`/grep of the first commits touching the experiment shows no key-shaped strings; new config file paths are gitignored |
| 10. Experiments-isolation violations | Phase 1 (install & env setup), enforced every phase | `ros/`'s own test suite passes with the experiment directory deleted; import-boundary guard test exists and passes |
| 11. Dev-container bloat from extras | Phase 1 (install & env setup) | Installed package list matches the documented minimal set (`rai_core` + ROS tools + `rai_whoami` only); no `torch`/`torchvision`/vision-checkpoint downloads present |
| 12. Test strategy mistakes | Every phase, gated from Phase 1 | Every new tool/wrapper ships a model-free test in the same commit; live-LLM tests are explicitly marked and never required for a phase's basic "done" bar |

## Sources

- [RAI GitHub repository](https://github.com/RobotecAI/rai) — HIGH (primary source, official framework repo)
- [RAI documentation — Getting started / install](https://robotecai.github.io/rai/setup/install/) — HIGH (official docs: Python/ROS version matrix, PYTHONPATH note, `rai_perception`/`rai_nomad` pip-availability gap)
- [RAI documentation — Tools](https://robotecai.github.io/rai/tutorials/tools/) — HIGH (official docs: `BaseROS2Tool` readable/writable/forbidden mechanism, example blocking `cmd_vel`)
- [RAI `rai_core` `pyproject.toml`](https://github.com/RobotecAI/rai/blob/development/src/rai_core/pyproject.toml) — HIGH (primary source: unpinned `langgraph`/`langchain`/`langchain-community` dependency lines)
- [RAI top-level `pyproject.toml`](https://github.com/RobotecAI/rai/blob/development/pyproject.toml) — HIGH (primary source: `openset` extras group pulling in torch/torchvision/Grounding DINO/SAM-2)
- [langchain-ai/langgraph issue #6363 — breaking change from unpinned `langgraph-prebuilt`](https://github.com/langchain-ai/langgraph/issues/6363) — HIGH (documented upstream incident matching Pitfall 1's exact failure mode)
- [LangChain release policy docs](https://docs.langchain.com/oss/python/release-policy) — MEDIUM (confirms `langchain-community` does not follow semver even post-1.0)
- ROS 2 QoS compatibility documentation and `ros2/ros2` issue #1434 — MEDIUM (general ROS 2 QoS-mismatch behavior, not RAI-specific, but directly explains Pitfall 3's silent-failure mechanism)
- This repository's own configuration as primary evidence (HIGH, first-party):
  `ros/src/wojtek_perception_bringup/wojtek_perception_bringup/cloud_reduce_node.py` (existing `qos_profile_sensor_data` usage),
  `ros/docker/compose.yaml` + `ros/docker/config/cyclonedds.xml` + `cyclonedds_link.xml` (domain ID 42, RMW cyclonedds, multicast/unicast split over the robot AP),
  `training/docs/scan-planner.md` (measured straight-line-into-furniture collision rates and SCAN-Planner's fix),
  `.planning/codebase/CONCERNS.md` (experiments-isolation fragility, public-repo secret-leakage risk, deployment-authorization gaps — general patterns this integration inherits),
  `experiments/autonomous_architecture_ros2_v1/README.md` (sibling experiment's settled isolation rules and model-free test-suite convention, directly reusable here).

---
*Pitfalls research for: RAI (RobotecAI) ROS 2 LLM-agent integration on Wojtek*
*Researched: 2026-09-05*
