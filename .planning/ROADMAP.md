# Roadmap: Wojtek RAI Integration

## Overview

The journey goes from an empty directory under `experiments/` to a typed instruction
that makes simulated Wojtek walk to a goal and describe what it sees. It starts by
standing up a pinned, isolated RAI environment that can join the running `ros/sim.sh`
ROS 2 graph without touching a single production package. Next it becomes a robot you
can talk to — a conversational agent that knows it is Wojtek and honestly states what
it cannot do — before it is allowed to move anything. Motion arrives only behind a
velocity arbiter that clamps, times out, and yields to teleop, so the LLM never gets a
direct line to `cmd_vel`. Then the agent gains senses: the live camera frame and its
own pose. Finally the highest-risk piece lands — goal navigation through SCAN-Planner,
with outcomes reported from planner evidence rather than model belief, and a separate
execution agent so the robot can be interrupted mid-walk.

Every phase is a vertical slice: at the end of each one there is something a human can
type and observe, not a layer waiting for the next layer.

## Phases

**Phase Numbering:**

- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [ ] **Phase 1: Isolated RAI Environment** - Pinned, self-contained RAI experiment that joins the running sim's ROS graph
- [ ] **Phase 2: Wojtek Talks** - Text chat with an agent that knows it is Wojtek and what it cannot do
- [ ] **Phase 3: Safe Walking** - Typed walking commands behind a velocity arbiter the agent cannot bypass
- [ ] **Phase 4: Seeing and Self-Report** - "What do you see?" and "Where are you?" answered from live sim data
- [ ] **Phase 5: Goal Navigation and Multi-Step Missions** - Collision-aware goal navigation, interruptible, chained with other tools

## Phase Details

### Phase 1: Isolated RAI Environment

**Goal**: A developer can bring up a pinned, self-contained RAI environment that discovers the running Wojtek simulation's ROS 2 topics, on either machine, without touching production packages
**Mode:** mvp
**Depends on**: Nothing (first phase)
**Requirements**: FOUND-01, FOUND-02, FOUND-03, FOUND-04, FOUND-06, FOUND-07
**Success Criteria** (what must be TRUE):

  1. From a clean checkout, one `run.sh` target installs the RAI stack at pinned versions (`rai-core==2.12.0`, `rai-whoami==0.0.5`, `rai_interfaces` at a pinned commit, LangChain/LangGraph locked in a committed lockfile) and succeeds on both the x86-64 laptop and the aarch64 remote GPU dev box
  2. With `ros/sim.sh` already running, one `run.sh` command starts the RAI process and it lists the simulation's live topics (`cmd_vel`, camera) — with zero edits to `ros/docker`, `ros/sim.sh`, or any package under `ros/src/`
  3. `run.sh test` passes with no LLM key, no ROS runtime and no GPU, and fails if anything outside the experiment directory imports it or if `ros/deploy.sh` could ship it
  4. Vendor and tracing keys are read only from the gitignored `.env`; the committed `config.toml` template contains placeholders, and a test rejects secret-looking values in tracked config

**Plans:** 2/5 plans executed

Plans:
**Wave 1**

- [x] 01-01-PLAN.md — Walking skeleton / tracer: pinned RAI install and live topic discovery end to end

**Wave 2** *(blocked on Wave 1 completion)*

- [x] 01-02-PLAN.md — `rai_interfaces` pinned to a commit SHA and built into the experiment's own colcon overlay

**Wave 3** *(blocked on Wave 2 completion)*

- [ ] 01-03-PLAN.md — Secrets hygiene: vendor config template, `.env`-only credentials, secret-shape guard
- [ ] 01-04-PLAN.md — Isolation, pinning, compose-override and model-free guard tests, plus the experiment README

**Wave 4** *(blocked on Wave 3 completion)*

- [ ] 01-05-PLAN.md — Two-machine verification record (x86-64 laptop and aarch64 remote GPU dev box)

### Phase 2: Wojtek Talks

**Goal**: A human can hold a text conversation with a single RAI agent that identifies as Wojtek, describes its real body and capabilities, and refuses what it cannot do
**Mode:** mvp
**Depends on**: Phase 1
**Requirements**: FOUND-05, EMB-01, EMB-02, HRI-01, HRI-02, HRI-03, HRI-04, AGT-01
**Success Criteria** (what must be TRUE):

  1. A human types into a chat UI started by `run.sh`, the message travels `/from_human` → one conversational (ReAct) agent → `/to_human`, and the reply plus any tool calls stream back in the UI
  2. Asked "who are you / what can you do", the agent answers as Wojtek with its actual joints, body and capability list, assembled by `rai whoami` from the URDF/MuJoCo model plus the capability document and rebuildable with one `run.sh` target
  3. Asked to grab an object, jump, or perform a trick, the agent declines and names what it can actually do (walk, navigate, look) instead of inventing a tool — verified by model-free prompt-assembly tests and a scripted evaluation prompt set
  4. An operator switches the model vendor and assigns the complex (reasoning) and simple (image description) model roles purely in config, restarts, and the same conversation works with no code change
  5. With tracing enabled in config, the operator can inspect every prompt, tool call and completion of a conversation; with tracing off (the default) nothing is emitted and no private host identity is committed

**Plans**: TBD
**UI hint**: yes

**Note**: This phase creates the single agent and its tool registry. Phases 3-5 register
the velocity, stop, image, state and navigate tools into this same agent; AGT-01's full
five-tool surface is complete once Phase 5 lands.

### Phase 3: Safe Walking

**Goal**: A typed walking instruction makes simulated Wojtek walk under the unchanged RL policy, with every agent-issued velocity passing through an arbiter it cannot bypass
**Mode:** mvp
**Depends on**: Phase 2
**Requirements**: MOT-01, MOT-02, MOT-03
**Success Criteria** (what must be TRUE):

  1. A velocity arbiter (`twist_mux` or equivalent) runs between the agent's own velocity topic and the policy's `cmd_vel`: it clamps to the trained command envelope, zeroes velocity when the agent's command goes stale, and gives teleop/gamepad priority over the agent
  2. A user types "walk forward slowly" or "turn left" and simulated Wojtek walks, with the command that reaches the policy inside the trained envelope
  3. An out-of-envelope or repeated agent command reaches the policy only in clamped form, and no agent code path publishes to `cmd_vel` directly — the arbiter is the only route
  4. A user types "stop" and the robot receives zero velocity immediately through a dedicated stop tool, without waiting on LLM reasoning latency

**Plans**: TBD

### Phase 4: Seeing and Self-Report

**Goal**: The agent answers what it sees and where it is from live simulation data rather than from memory
**Mode:** mvp
**Depends on**: Phase 2 (tools register into the Phase 2 agent; independent of Phase 3 motion)
**Requirements**: VIS-01, VIS-02
**Success Criteria** (what must be TRUE):

  1. A user asks "what do you see?" and gets a description of the actual current sim camera frame; moving an object in the scene changes the answer
  2. The image tool subscribes with sensor-data QoS and returns either a fresh frame or an explicit staleness error — it never silently returns nothing or a stale frame as current
  3. A user asks "where are you?" and the agent reports pose and current velocity from a robot-state tool, and that same tool exposes a stuck flag the agent can check during a task

**Plans**: TBD

### Phase 5: Goal Navigation and Multi-Step Missions

**Goal**: A described goal makes Wojtek walk there without hitting furniture, reporting real outcomes, while the agent stays responsive and can chain the trip with its other tools
**Mode:** mvp
**Depends on**: Phase 3 (velocity path and arbiter), Phase 4 (pose/state evidence)
**Requirements**: NAV-01, NAV-02, AGT-02, AGT-03
**Success Criteria** (what must be TRUE):

  1. A user types "go to the chair" or "go 2 m forward and 1 m left" and Wojtek reaches the goal in a cluttered sim scene without walking into furniture, via a `navigate_to` tool wrapping SCAN-Planner's existing executor as a ROS node with a sim pose source
  2. When a goal is unreachable, the agent tells the user "blocked" or "gave up" derived from the planner's own progress/stuck thresholds and pose evidence — never from the model asserting it arrived
  3. While a navigation run is in progress, the user can still chat with the agent and issue "stop", and the robot stops — long-running navigation runs in a separate StateBased execution agent
  4. "Go to the chair, then tell me what is there" executes as the correct tool sequence (navigate → image), and a scripted prompt set with expected tool traces passes against the sim

**Plans**: TBD

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Isolated RAI Environment | 2/5 | In Progress|  |
| 2. Wojtek Talks | 0/TBD | Not started | - |
| 3. Safe Walking | 0/TBD | Not started | - |
| 4. Seeing and Self-Report | 0/TBD | Not started | - |
| 5. Goal Navigation and Multi-Step Missions | 0/TBD | Not started | - |

## Notes

**Safety ordering (from research):** the velocity arbiter (MOT-02) is inside Phase 3
*before* any agent-published motion, never after. Nothing in this roadmap changes
`wojtek_policy`, the observation layout, or the schema-2 policy contract.

**Research flags carried into planning:**

- Phase 1: capture the exact LangChain/LangGraph versions resolved by `rai-core==2.12.0`
  and commit the lockfile; find `rai_interfaces`' tested commit in `rai_core`'s own
  `ros_deps.repos`

- Phase 3: extract the trained velocity envelope (max vx/vy/wz) from
  `training/docs/configuration.md` before writing the clamp

- Phase 5: **spike before planning** — SCAN-Planner ROS wrapping vs. Nav2, judged on
  effort, Jetson portability and quadruped tuning (this is the highest-risk item in
  the milestone)
