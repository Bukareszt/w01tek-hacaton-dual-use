# Codebase Structure

**Analysis Date:** 2026-09-05

## Directory Layout

```
w01-tek/
├── training/                      # MJX/Brax PPO training pipeline (Hydra-configured)
│   ├── run.sh                     # CLI entry point (dispatches to Python modules)
│   ├── pyproject.toml             # Python deps: jax, mujoco, brax, wandb, hydra
│   ├── wojtek_rl/                 # Core training package
│   │   ├── train.py               # Entry point: Hydra-decorated PPO trainer
│   │   ├── base.py                # Base env: MJX physics, model loading, utilities
│   │   ├── env.py                 # WojtekJoystick: velocity-tracking task (116 KB, 54-dim obs)
│   │   ├── env_getup.py           # WojtekGetup: fall recovery task
│   │   ├── env_jump.py            # WojtekJump: jumping task
│   │   ├── env_biped.py           # WojtekBiped: bipedal walk research variant
│   │   ├── export_policy.py       # Export Brax checkpoint → policy.npz + policy_meta.json
│   │   ├── deploy_contract.py     # Define schema-2 deployment contract
│   │   ├── np_policy.py           # Numpy mirror of exported network (validation)
│   │   ├── eval.py                # Evaluate policies (single rollout, metrics)
│   │   ├── battery.py             # Battery tests: fixed rollout suite (46 KB)
│   │   ├── courses.py             # Course benchmark: path-following score per scenario
│   │   ├── imu_grid.py            # IMU robustness grid: sweep gyro/accel/lag/gain
│   │   ├── distill.py             # DAgger knowledge distillation (student from teacher)
│   │   ├── build_model.py         # Generate wojtek_mjx.xml from source description
│   │   ├── build_terrain.py       # Build heightfield terrain (procedural/scanned)
│   │   ├── check_model_mjx.py     # Validate model XML, report potential issues
│   │   ├── check_terrain.py       # Validate terrain, measure contact budget
│   │   ├── build_room.py          # Build 3D room scene for navigation
│   │   ├── room_assets.py         # Room asset pipeline (trimesh, convex decomposition)
│   │   ├── room_app.py            # Interactive room navigation demo
│   │   ├── sysid/                 # System identification from rosbags
│   │   ├── scan/                  # SCAN-Planner local collision-aware planning
│   │   ├── futurenav_server/      # VLM navigation server bridge
│   │   ├── video/                 # Video generation (torque, depth overlay)
│   │   ├── terrain_env.py         # Terrain arena definition, height lookup
│   │   ├── paths.py               # Derived constants: leg order, joint indices
│   │   ├── height_scan.py         # Measure standing height per position
│   │   ├── ppo_rnd.py             # PPO + RND (random network distillation) variant
│   │   ├── policy_io.py           # Policy serialization utilities
│   │   ├── courses/               # Pre-defined courses (start pos, gates, lanes)
│   │   ├── conf/                  # Hydra configuration
│   │   │   ├── config.yaml        # Base: task, network, obs, ppo, run_name, seed
│   │   │   ├── experiment/        # Preset experiments
│   │   │   │   ├── locomotion.yaml       # Unified walk/trot/stand
│   │   │   │   ├── locomotion_v*.yaml   # Iterative improvements (v2-v8)
│   │   │   │   ├── getup.yaml           # Fall recovery
│   │   │   │   ├── jump.yaml            # Jumping
│   │   │   │   ├── stiff_*.yaml         # Stiffness ladder experiments
│   │   │   │   └── springy_*.yaml       # Compliant experiments
│   │   │   ├── task/              # Task YAML overrides (joystick, getup, jump)
│   │   │   ├── network/           # Network arch groups (default, small, large)
│   │   │   └── obs/               # Observation groups (full, no_imu)
│   ├── wojtek_eval/               # Navigation evaluation: occupancy, pathfinding, VLM bridge
│   ├── demo/                      # Interactive demos
│   │   ├── app.py                 # Web app with policy playground
│   │   └── static/                # UI assets
│   ├── assets/                    # 3D assets (room geometry, scenes)
│   ├── tests/                     # Unit + integration tests
│   │   ├── unit/                  # Model-free tests (reward funcs, etc.), ~3 s
│   │   ├── integration/           # Real MJX tests (env instantiation), ~6m cold
│   │   └── data/                  # Golden replay data for regression
│   ├── jobs/                      # Reusable parameterized training scripts (remote execution)
│   │   ├── train.sh               # Generic train job payload
│   │   ├── stiff_ladder.sh        # Gated PD-stiffness ladder
│   │   └── (other grid/suite jobs)
│   └── docs/                      # Configuration reference, training lessons
│       ├── configuration.md       # Hydra groups, all settings, backend terminology
│       ├── scan-planner.md        # SCAN-Planner collision detection docs
│       └── sysid.md               # System ID from rosbags

├── ros/                           # ROS 2 workspace (jazzy) — deployed to RPi via rsync
│   ├── deploy.sh                  # Deployment script: provision RPi, rsync src, build, restart service
│   ├── src/                       # ROS packages (only this dir syncs to RPi)
│   │   ├── wojtek_bringup/        # Main bring-up package, control loop, state machine (RPi-only)
│   │   │   ├── launch/
│   │   │   │   ├── real.launch.py          # Real robot launch
│   │   │   │   ├── sim.launch.py           # Simulation launch
│   │   │   │   ├── robot.launch.py         # Top-level orchestrator
│   │   │   │   └── (sensor, viz launchers)
│   │   │   ├── urdf/
│   │   │   │   ├── wojtek_real.urdf.xacro # Real robot description (kp, kd, max_torque)
│   │   │   │   └── wojtek_ros2_control.urdf.xacro  # ros2_control config
│   │   │   ├── config/
│   │   │   │   ├── real_controllers.yaml   # Controller config (MD80, IMU)
│   │   │   │   └── mag_calib.yaml         # Magnetometer calibration
│   │   │   ├── wojtek_bringup/
│   │   │   │   ├── robot.py               # One-command robot bring-up (from PC container)
│   │   │   │   ├── launch_common.py       # Shared launch utilities
│   │   │   │   ├── real_io_node.py        # Real I/O: state machine, action cmd publish
│   │   │   │   └── (initialization, poses)
│   │   │   └── package.xml
│   │   │
│   │   ├── wojtek_policy/        # Policy loading + numpy runtime (RPi-deployable)
│   │   │   ├── policy.py          # WojtekPolicy: load npz, run inference (no JAX)
│   │   │   ├── policy_node.py     # ROS node: 50 Hz control loop
│   │   │   ├── policy_source.py   # Resolve policy ref (HF org/name) → local store
│   │   │   ├── joint_map.py       # Joint order, actuator indices
│   │   │   ├── poses.py           # Home, zero, stand poses
│   │   │   ├── tricks.py          # Scripted tricks (gamepad D-pad)
│   │   │   ├── config/
│   │   │   │   └── joint_map.yaml # Joint order definition
│   │   │   ├── test/
│   │   │   │   ├── test_policy.py  # Unit tests for policy runtime
│   │   │   │   └── test_tricks.py
│   │   │   └── package.xml
│   │   │
│   │   ├── wojtek_description/   # URDF, meshes, MuJoCo model source
│   │   │   ├── mujoco/
│   │   │   │   ├── wojtek.xml     # Source model (hand-edited)
│   │   │   │   ├── wojtek_mjx.xml   # Generated for MJX training (from run.sh build)
│   │   │   │   └── scene_mjx.xml    # Generated training scenes
│   │   │   ├── urdf/
│   │   │   │   └── (URDF for simulation)
│   │   │   └── mesh/              # STL meshes
│   │   │
│   │   ├── wojtek_teleop/        # Gamepad teleoperation (teleop_twist_joy + custom)
│   │   ├── wojtek_pc/            # PC-only visualization (RViz config, dashboards) — NOT deployed
│   │   ├── wojtek_benchmark/     # Benchmark harness
│   │   ├── wojtek_perception_bringup/  # Camera, lidar bringup (optional)
│   │   │
│   │   └── Hardware interfaces (deployed to RPi):
│   │       ├── md80_hardware_interface/     # CAN communication to MD80 motors
│   │       ├── bmi160_serial_hardware_interface/   # Serial IMU (BMI160)
│   │       ├── bmx160_serial_hardware_interface/   # Serial IMU (BMX160)
│   │       ├── imu_i2c_hardware_interface/   # I2C IMU
│   │       └── magnetometer_broadcaster/    # Mag compass fusion
│   │
│   ├── deploy/                   # RPi deployment scripts
│   │   ├── rpi/
│   │   │   ├── install.sh         # RPi provisioning (RT kernel, ROS, services)
│   │   │   ├── cloud-init/        # Cloud-init configs
│   │   │   └── wojtek-robot.service  # Systemd service unit
│   │   ├── pc/                    # PC-side setup scripts
│   │   └── README.md
│   │
│   ├── policies/                 # Local policy store (gitignored, synced to RPi)
│   │   └── hvsr-robotics/         # Organization/keeper repos
│   │       ├── wojtek-stiff-locomotion/
│   │       │   ├── policy.npz
│   │       └── policy_meta.json
│   │
│   ├── foxglove/                 # Foxglove Studio extension (console panel)
│   ├── hw_tests/                 # Hardware-specific tests
│   ├── docker/                   # Docker compose for dev (dev.sh container)
│   └── README.md
│
├── experiments/                  # Isolated experimental work (NOT on robot, NOT in production)
│   └── autonomous_architecture_ros2_v1/    # Example: autonomous agent + VLM nav
│       ├── README.md                       # Status, stability warning
│       ├── run.sh                          # Experiment-local test runner
│       ├── ros/src/
│       │   ├── wojtek_agent/               # LLM-based agent
│       │   ├── wojtek_demo/                # VLM navigation demo
│       │   └── (other research packages)
│       └── (experiment's own tests, configs, docs)
│
├── skills/                       # Optional agent guides (opt-in via symlink)
│   ├── brax-locomotion-training/
│   │   ├── SKILL.md               # Overview, when to use
│   │   ├── references/
│   │   │   └── wojtek-training-lessons.md  # Evidence from past runs (reward tuning, gait)
│   │   └── scripts/               # Utility scripts
│   ├── mjx-robot-model-prep/
│   ├── futurenav-nav-demo/
│   └── (other optional guides)
│
├── docs/                         # Project-wide documentation
│   ├── plans/
│   │   └── mjwarp-phase0-report.md  # Backend buffer sizing, validation
│   ├── research/                 # Research notes, papers, references
│   └── perception/               # Vision/perception research
│
├── .claude/                      # Claude Code agent config (local)
│   └── (workspace symlinks, rules, credentials)
│
├── kotek/                        # Work in progress (untracked)
│
├── CLAUDE.md                     # Agent guide: repository boundaries, workflow rules
├── README.md                     # Project overview
├── NOTICE                        # GPL license notice (IMU firmware)
├── LICENSE                       # Apache-2.0 for new code
└── .env.example                  # Template for .env (HF_ORGANIZATION, UBUNTU_PRO_TOKEN, etc.)
```

## Directory Purposes

**`training/`**
- Purpose: Hydra-driven MJX/Brax PPO training, evaluation, and policy export
- Entry: `./training/run.sh train` (from repo root)
- Output: `training/runs/<run_name>/checkpoints/`, `training/runs/<run_name>/policy.npz`
- Generated: Deleted after training (gitignored): checkpoints, videos, WandB cache, JAX compile cache

**`ros/src/`**
- Purpose: ROS 2 packages for robot control and deployment
- Synced: Via `ros/deploy.sh` to RPi (full-recursive, excludes build/install)
- Not synced: `wojtek_pc/`, visualization-only packages (RViz is PC-bound)
- Build target: `colcon build --packages-up-to wojtek_bringup` on RPi

**`ros/deploy/`**
- Purpose: RPi provisioning and service configuration
- Deployed: By `deploy.sh` via rsync + SSH
- Contains: install.sh (idempotent), systemd unit, cloud-init, cyclon DDS config

**`experiments/`**
- Purpose: Research and experimental work, isolated from production
- Boundary: Nothing here may depend on or reach `wojtek_bringup` or the robot
- Layout: Each experiment self-contained with own README, run.sh, tests
- Unstable: Interfaces and structure are NOT precedent for main codebase

**`skills/`**
- Purpose: Optional agent guides (training lessons, model prep references)
- Opt-in: Users must explicitly symlink or ask Claude to read `SKILL.md`
- Not required: Core training/deployment does not depend on skills

**`docs/`**
- Purpose: Project documentation, planning artifacts, research notes
- Key files:
  - `docs/plans/mjwarp-phase0-report.md` — backend validation, buffer sizing
  - `skills/brax-locomotion-training/references/wojtek-training-lessons.md` — evidence from past runs

## Key File Locations

**Training Entry Points:**
- `training/run.sh` — CLI dispatcher: `train`, `eval`, `export`, `build`, `check`, `test`
- `training/wojtek_rl/train.py` — Hydra-decorated PPO trainer (main entry)
- `training/wojtek_rl/export_policy.py` — Convert checkpoint → deployment contract

**Training Configuration:**
- `training/wojtek_rl/conf/config.yaml` — Base config (task, network, obs, ppo, run_name)
- `training/wojtek_rl/conf/experiment/*.yaml` — Experiment presets (locomotion, getup, jump, stiff_ladder, etc.)
- Hydra merge order: base → `+experiment=<name>` → CLI overrides

**Task Environments:**
- `training/wojtek_rl/env.py` — WojtekJoystick (joystick velocity tracking)
- `training/wojtek_rl/env_getup.py` — WojtekGetup (fall recovery)
- `training/wojtek_rl/env_jump.py` — WojtekJump (commanded jump)
- `training/wojtek_rl/base.py` — WojtekEnv base class (MJX physics, model loading)

**Deployment Export:**
- `training/wojtek_rl/export_policy.py` — Main export script
- `training/wojtek_rl/deploy_contract.py` — Schema-2 contract definition (CONSUMED keys, resolved values)
- `training/wojtek_rl/np_policy.py` — Numpy runtime validator

**Deployment CLI:**
- `ros/deploy.sh` — Provision, sync, build, deploy to RPi
- `ros/src/wojtek_bringup/wojtek_bringup/robot.py` — One-command bringup (PC side)

**Robot Control Stack:**
- `ros/src/wojtek_bringup/wojtek_bringup/real_io_node.py` — State machine, I/O coordination
- `ros/src/wojtek_policy/wojtek_policy/policy_node.py` — 50 Hz policy loop (RPi)
- `ros/src/wojtek_policy/wojtek_policy/policy.py` — Numpy policy runtime (load, step, inference)

**Model & Description:**
- `ros/src/wojtek_description/mujoco/wojtek.xml` — Source MuJoCo model (hand-edited)
- `ros/src/wojtek_description/mujoco/wojtek_mjx.xml` — Generated training model (from `run.sh build`)
- `ros/src/wojtek_bringup/urdf/wojtek_real.urdf.xacro` — Real robot URDF (kp, kd, max_torque values)

**Validation & Testing:**
- `training/tests/unit/` — Fast tests (no env instantiation, ~3s)
- `training/tests/integration/` — Slow tests (real MJX, ~6m cold)
- `training/tests/data/` — Golden replay data

**Documentation:**
- `CLAUDE.md` — Agent guide (boundaries, workflow, validation rules)
- `training/docs/configuration.md` — Full Hydra reference (groups, all settings, backend terminology)
- `training/docs/scan-planner.md` — SCAN-Planner collision detection
- `training/docs/sysid.md` — System identification from rosbags

## Naming Conventions

**Files:**
- Python modules: `snake_case.py` (env.py, base.py, policy.py)
- Test files: `test_*.py` (test_policy.py, test_locomotion.py)
- Hydra YAML: `snake_case.yaml` (config.yaml, locomotion.yaml)
- Generated files: suffix with `_mjx` (wojtek_mjx.xml, scene_mjx.xml)
- Launch files: `*_ros2_control.urdf.xacro`, `*.launch.py` (real.launch.py, sim.launch.py)

**Directories:**
- Python packages: `snake_case/` (wojtek_rl/, wojtek_policy/, wojtek_bringup/)
- Config: `conf/`, `config/` depending on tool (Hydra uses `conf/`, ROS typically `config/`)
- Tests: `tests/`, with `unit/` and `integration/` split
- Generated outputs: `runs/`, `videos/`, `.jax_cache/` (all gitignored)

**ROS packages:**
- Format: `wojtek_<component>` (wojtek_bringup, wojtek_policy, wojtek_teleop)
- Hardware interfaces: `<hardware>_hardware_interface` (md80_hardware_interface, bmi160_serial_hardware_interface)

**Hydra experiment names:**
- Tasks: `locomotion`, `getup`, `jump`
- Variants: `locomotion_v1`, `locomotion_v2`, ... (version suffix)
- Stiffness sweeps: `stiff_ladder_kp50`, `stiff_ladder_kp80`, etc.
- Specialized: `springy_phase_a`, `springy_phase_b` (multi-phase experiments)

**Constants & symbols:**
- Leg/joint order defined once: `paths.LEGS`, `paths.ACTUATORS` (rear_left, rear_right, front_right, front_left)
- Observation size: `OBS_SIZE`, `PRIVILEGED_SIZE` (per task)
- Motor limits: `KNEE_SINGULARITY` (3.2 rad, safety boundary), `KNEE_ACTUATORS`, `ABDUCTION_ACTUATORS`

## Where to Add New Code

**New training task:**
1. Create `training/wojtek_rl/env_<taskname>.py` (inherit `WojtekEnv`)
2. Add `default_config()` with task-specific obs, reward, episode_length
3. Add YAML group: `training/wojtek_rl/conf/task/<taskname>.yaml`
4. Optionally add experiment preset: `training/wojtek_rl/conf/experiment/<taskname>.yaml`
5. Tests: `training/tests/integration/test_<taskname>.py`

**New evaluation metric or grid:**
1. Add module: `training/wojtek_rl/<metric>_grid.py` or extend `battery.py`
2. Add CLI dispatch in `training/run.sh` (if new subcommand needed)
3. Output: JSON/CSV reports to `training/runs/<run_name>/`

**New ROS package:**
1. Create under `ros/src/wojtek_<name>/`
2. Add `package.xml`, `setup.py`, standard ROS 2 structure
3. If hardware-specific: `ros/src/<hardware>_hardware_interface/`
4. Launch integration: add to `wojtek_bringup/launch/`

**New robot control behavior:**
1. Add to `real_io_node.py` (state machine) or create new ROS node
2. Publish control messages to `/wojtek/command` topic
3. Subscribe to sensor topics (IMU, encoders, gamepad)
4. Integrate into `robot.py` CLI if adding new command-line mode

**New hardware interface:**
1. Create `ros/src/<hardware>_hardware_interface/`
2. Implement `rclcpp_lifecycle::LifecycleNode`
3. Register with hardware_interface (MD80 uses MotionControllerInterface pattern)
4. Add to `real_controllers.yaml`, bring up in `real.launch.py`

**Experimental feature (does NOT go on robot yet):**
1. Create directory: `experiments/<feature_name>/`
2. Add self-contained `README.md` (status, stability)
3. Include own `run.sh`, `pyproject.toml` or `package.xml`, tests
4. Directory structure can differ from production (flexibility for research)
5. Promote to main codebase when ready (move code, update imports, add to main tests)

**Model or system ID changes:**
1. Edit `ros/src/wojtek_description/mujoco/wojtek.xml` (source)
2. Run `./training/run.sh build` to regenerate `wojtek_mjx.xml`, `scene_mjx.xml`
3. Commit both source and generated files
4. Run `./training/run.sh check` to validate
5. Retest policies: may need retraining if model significantly changed

**New Hydra configuration group:**
1. Create directory: `training/wojtek_rl/conf/<group>/`
2. Add YAML files (one per option)
3. Reference in config.yaml with defaults or CLI override
4. Document in `training/docs/configuration.md`

## Special Directories

**`training/runs/`**
- Purpose: Training outputs (checkpoints, videos, WandB cache)
- Generated: Yes (cleared between runs)
- Committed: No (gitignored)
- Cleanup: Manual deletion or `training/run.sh smoke` overwrites

**`ros/policies/`**
- Purpose: Local cache of downloaded policies
- Generated: Yes (by `deploy.sh` from HuggingFace)
- Committed: No (gitignored)
- Cleanup: Manual or auto (policies can be re-fetched from HF)

**`training/.jax_cache/`**
- Purpose: JAX persistent compilation cache
- Generated: Yes (first time integration tests run)
- Committed: No (gitignored)
- Benefit: Integration tests skip 6+ min recompilation on warm runs

**`experiments/`**
- Purpose: Research only, unstable, NOT part of production
- Generated: Yes (experiment outputs)
- Committed: Selectively (code/configs yes, checkpoints/videos no)
- Cleanup: Per-experiment (not deployed to RPi ever)

**`.claude/`**
- Purpose: Claude Code workspace configuration
- Committed: Local only (gitignored)
- Contains: Agent rules, skills, credentials cache

---

*Structure analysis: 2026-09-05*
