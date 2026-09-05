# Wojtek RAI Integration

## What This Is

Wojtek (w01-tek) is a 12-DOF quadruped whose walking is an RL policy trained in
MuJoCo MJX/Brax and deployed through a ROS 2 Jazzy stack on a Raspberry Pi 4.
This project imports the [RAI](https://github.com/RobotecAI/rai) embodied-AI
agent framework (RobotecAI) into the codebase as a new, self-contained
experiment, so that an LLM agent can command Wojtek at the high level
("walk forward", "go to the chair", "what do you see?") through the same ROS 2
topics the robot already exposes. The milestone target is RAI driving Wojtek in
the existing MuJoCo simulation; the physical robot (with an onboard Jetson for
the agent) is the follow-on.

## Core Value

A text instruction typed to a RAI agent makes simulated Wojtek walk, navigate to
a goal, and describe what its camera sees, using the production ROS 2 interfaces
unchanged.

## Requirements

### Validated

<!-- Inferred from existing code (.planning/codebase/). -->

- ✓ RL joystick locomotion policy trained with MJX/Brax PPO and exported as the
  schema-2 contract (`policy.npz` + `policy_meta.json`) — existing
- ✓ ROS 2 Jazzy stack runs the exported policy at 50 Hz on the RPi
  (`wojtek_policy`, `wojtek_bringup`, MD80 + IMU hardware interfaces) — existing
- ✓ Velocity commands reach the policy over `cmd_vel` (teleop, gamepad) — existing
- ✓ Full local MuJoCo simulation of the same ROS stack via `ros/sim.sh`
  (`wojtek_mujoco_hardware_interface`, `wojtek_pc` sim launch, RViz/Foxglove,
  web console) — existing
- ✓ Perception bringup (camera / point cloud) launch in `wojtek_perception_bringup` — existing
- ✓ Sim-only VLM navigation research in `training/wojtek_rl/` (FutureNav,
  Qwen3-VL, Claude) with the SCAN-Planner collision-aware local planner — existing
- ✓ Experimental voice/agent layer `experiments/autonomous_architecture_ros2_v1`
  (router + Bielik + Qwen-VL, ASR/TTS nodes, `wojtek_agent` tools); hear + talk
  verified, walk-and-look through ROS not built — existing
- ✓ Policy deployment by reference from Hugging Face (`ros/deploy.sh --policy`) — existing

### Active

- [ ] RAI (rai core + ROS 2 tools) installed and runnable inside this repo as a
  new self-contained experiment under `experiments/`, following the isolation
  rules (nothing outside imports it; not reachable by `ros/deploy.sh`)
- [ ] `rai whoami` embodiment built from Wojtek's URDF/MuJoCo model and docs, so
  the agent identifies as Wojtek and knows its body and capabilities
- [ ] RAI agent can command velocity walking: natural-language text →
  `cmd_vel` (vx, vy, wz) → RL joystick policy in the MuJoCo sim
- [ ] RAI agent can navigate to a described goal with obstacle avoidance in sim
  (reusing SCAN-Planner or RAI's navigation integration)
- [ ] RAI agent can look: grab the sim camera image as a tool and describe the
  scene / answer "what do you see?"
- [ ] Human interacts with the agent through RAI's text chat interface
- [ ] Agent works against the existing sim launch (`ros/sim.sh`) with no changes
  to production ROS topics or the policy contract
- [ ] Model backend (cloud vs local) is configurable — the concrete choice is
  decided in a separate task, not here

### Out of Scope

- Voice (ASR/TTS through RAI) — text chat only this milestone; the existing
  experiment already covers voice, and it stays untouched
- Replacing or porting the existing `autonomous_architecture_ros2_v1` agent
  experiment — RAI lives side by side; merging is a later decision
- `rai_perception`, `rai_nomad`, `rai_bench`, `rai_sim`, `rai_finetune` — not
  needed for the core value; revisit when goal navigation needs open-set
  detection or NoMaD
- Running RAI on the physical robot / Jetson bring-up — real-robot run is the
  stretch goal and the next milestone; sim first
- Skill / trick tools (getup, jump, D-pad show tricks) — walking, navigation and
  look are enough for v1
- Any change to the RL training pipeline, observation layout, or policy
  contract — RAI consumes the deployed policy through ROS, nothing more
- Committing LLM API keys, private hostnames, or cluster identity — public repo
  rule; secrets enter through `.env`

## Context

- **Repo layout and rules:** `CLAUDE.md` at the root. `experiments/` holds
  self-contained, non-production work with its own README, tests and `run.sh`;
  nothing in it may be a dependency of `wojtek_bringup` or reach the robot via
  `ros/deploy.sh` (which rsyncs `ros/src/` only). Apache-2.0 by default.
  "Wojtek" naming everywhere.
- **Codebase map:** `.planning/codebase/` (ARCHITECTURE, STACK, STRUCTURE,
  CONVENTIONS, INTEGRATIONS, TESTING, CONCERNS), refreshed 2026-09-05.
- **ROS 2:** Jazzy, Ubuntu base, CycloneDDS, dev container in `ros/docker/`.
  RAI supports Humble and Jazzy, Python 3.10/3.12, Apache-2.0.
- **Sim:** MuJoCo is the single engine (no Gazebo). `ros/sim.sh` runs the full
  stack in the dev container with RViz/Foxglove and a web console; the same
  launch vocabulary as the robot.
- **Prior agent work to learn from, not import:** the experiment's
  `docs/architecture.md` settled decisions (node-per-model, streaming
  sentence flush, walking policy decoupled from AI latency, mic/speaker local
  to compute). `training/docs/scan-planner.md` explains why a local planner
  must sit between any VLM and the policy: VLM-issued "forward 1.5" as a
  straight line walks into furniture.
- **Compute reality:** RPi 4 runs only the control loop and has no internet.
  Agent compute is a dev/GPU box now and an onboard Jetson later; ROS 2 DDS
  crosses the robot's own access point.
- **Known concerns** (CONCERNS.md): generated MJX XML drift, hard-coded obs size,
  action-filter train/deploy mismatch, `warp-lang==1.13.0` pin. None are
  touched by this project, but the agent must never alter policy inputs.

## Constraints

- **Isolation**: RAI integration lives under `experiments/<name>/`; no import
  from `ros/` or `training/` into it beyond read-only use of `wojtek_rl`
  helpers, and nothing outside imports it — repo rule, keeps the robot safe
- **Interfaces**: the agent talks to Wojtek only through existing ROS 2 topics
  (`cmd_vel`, camera/perception topics); no changes to `wojtek_policy` or the
  schema-2 contract — deployed policies must keep working
- **Tech stack**: ROS 2 Jazzy, Python, MuJoCo sim; RAI upstream unforked where
  possible (pin a release) — reduces maintenance
- **Security**: no secrets or private infrastructure identity in the tree;
  model API keys via `.env` — public repository
- **Safety**: real-robot launch and policy deployment stay human-authorized;
  this milestone does not touch hardware
- **Compute**: eventual target is an onboard Jetson; avoid designs that only
  work with a large cloud model (backend must be swappable)
- **License**: Apache-2.0 for new code; RAI itself is Apache-2.0

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Import RAI rather than extend the custom agent experiment | Battle-tested ROS 2 agent framework with ROS tools, whoami, HRI; avoids re-building agent plumbing | — Pending |
| RAI side by side with `autonomous_architecture_ros2_v1`, not replacing it | Existing experiment has verified voice work; compare before merging | — Pending |
| Scope = rai core + ROS tools + rai whoami only | Minimum for text-driven walk/nav/look; other RAI packages deferred | — Pending |
| Sim (MuJoCo via `ros/sim.sh`) is the milestone target, physical robot is stretch | Same ROS topics in sim and robot; validates the integration without hardware risk | — Pending |
| Text chat interface, no voice | Keeps v1 small; voice already exists elsewhere | — Pending |
| Agent compute is Jetson onboard in the long run | RPi 4 cannot run models and has no internet | — Pending |
| Model backend chosen in a separate task | User wants cloud-vs-local decided with its own evidence | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-09-05 after initialization*
