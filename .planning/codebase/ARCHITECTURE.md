<!-- refreshed: 2026-09-05 -->
# Architecture

**Analysis Date:** 2026-09-05

## System Overview

Wojtek is a quadruped robot locomotion system split into two coordinated subsystems:

1. **Training pipeline** (`training/`): MuJoCo MJX / Brax PPO training that produces policies
2. **Deployment pipeline** (`ros/`): ROS 2 stack that runs policies on the physical robot

The connection point is a schema-2 **deployment contract** (`policy_meta.json` + `policy.npz`) that encodes all environment configuration and the trained neural network in a form deployable without JAX/Brax.

```text
┌────────────────────────────────────────────────────────────────────┐
│                         Training Pipeline                           │
│                    (training/run.sh, Hydra config)                  │
├──────────────────┬──────────────────┬──────────────────┬───────────┤
│   Task Env       │  PPO Trainer     │  Evaluation      │  Export   │
│ (MJX Physics)    │  (Brax)          │  (Battery,       │ (policy.  │
│  - joystick      │                  │   Courses)       │  npz +    │
│  - getup         │ Backend:         │                  │  meta.    │
│  - jump          │ - MJWarp (GPU)   │                  │  json)    │
│                  │ - JAX (CPU)      │                  │           │
└────────┬─────────┴──────────┬───────┴──────────┬───────┴─────┬─────┘
         │                    │                  │             │
         └────────────────────┴──────────────────┴─────────────┘
                              │
            ┌─────────────────▼─────────────────┐
            │  Deployment Contract              │
            │  schema-2                         │
            │  - Resolved obs/action/anchor     │
            │  - Normalized MLP weights         │
            │  - No JAX/Brax knowledge needed   │
            └─────────────────┬─────────────────┘
                              │
┌─────────────────────────────▼─────────────────────────────────────┐
│                      Deployment Pipeline                           │
│              (ros/, deployed on RPi via rsync)                     │
├──────────────────────────────┬────────────────────┬───────────────┤
│  ROS Workspace               │  Policy Runtime    │  Robot Driver │
│                              │  (numpy, no JAX)   │               │
│  - wojtek_bringup            │                    │  - MD80 HW I/F│
│  - wojtek_policy             │  Load policy.npz + │  - IMU (BMI160│
│  - hardware interfaces       │  policy_meta.json  │    /BMX160)   │
│  - teleop control            │                    │  - Motor servo│
│  - perception stack          │  Step:             │               │
│                              │  obs -> action ->  │               │
└──────────────────────────────┴────────────┬───────┴───────────────┘
                                           │
                    ┌──────────────────────▼──────────────────────┐
                    │  Physical Robot (Wojtek Quadruped)          │
                    │  - 12 DOF (4 legs x 3 joints)               │
                    │  - PD servo (kp/kd tunable)                 │
                    │  - IMU, joint encoders                      │
                    │  - Real-time control loop (50 Hz)           │
                    └───────────────────────────────────────────────┘
```

## Component Responsibilities

| Component | Responsibility | File |
|-----------|----------------|------|
| Task Environment | Define task, reward, observations, domain randomization | `training/wojtek_rl/env.py`, `env_*.py` |
| Base Environment | Physics setup, model loading, common utilities | `training/wojtek_rl/base.py` |
| PPO Trainer | Brax PPO training loop, checkpointing, WandB logging | `training/wojtek_rl/train.py`, `ppo_rnd.py` |
| Hydra Config | Experiment presets, hyperparameter definitions | `training/wojtek_rl/conf/` |
| Export Pipeline | Convert Brax checkpoint → numpy artifact + deployment contract | `training/wojtek_rl/export_policy.py`, `deploy_contract.py` |
| Eval & Metrics | Policy evaluation, battery tests, course benchmarks | `training/wojtek_rl/eval.py`, `battery.py`, `courses.py` |
| Policy Runtime | Pure numpy inference for deployed policies | `ros/src/wojtek_policy/wojtek_policy/policy.py` |
| ROS Bringup | System bring-up, control loop, state machine | `ros/src/wojtek_bringup/wojtek_bringup/robot.py`, `real_io_node.py` |
| Hardware Interface | MD80 motor communication, IMU reading, encoder polling | `ros/src/md80_hardware_interface/`, `bmi160_serial_hardware_interface/` |
| Policy Node | Load policy, run inference at 50 Hz, publish actions | `ros/src/wojtek_policy/wojtek_policy/policy_node.py` |
| Deployment | Provision RPi, rsync workspace, resolve policies from HF | `ros/deploy.sh` |

## Pattern Overview

**Overall:** Multi-stage RL pipeline with explicit, reviewable configuration and reproducible deployment

**Key Characteristics:**
- **Hydra-driven training**: All experiment parameters (task, PPO, reward) are YAML-defined and resolvable before GPU time spent
- **Environment-as-contract**: Training envs export complete numerical contracts; deploy side never re-derives anything
- **JAX-free deployment**: Policies run on pure numpy at 50 Hz on the RPi; no deep-learning runtime needed on the robot
- **Multi-task support**: Three task types (joystick velocity tracking, getup recovery, jump) with shared base env class
- **Backend abstraction**: Unified MJX interface with MJWarp (GPU) and JAX (CPU) backends; backend selection is late-binding
- **Human-gated deployment**: Policy export, deployment, and robot arming all require explicit authorization

## Layers

**Configuration Layer (Hydra):**
- Purpose: Externalize all experiment-defining parameters; enable reproducibility and sweeps
- Location: `training/wojtek_rl/conf/`
- Contains: Task groups (joystick, getup, jump), PPO settings, experiment presets, observation/network selections
- Depends on: Nothing (pure data)
- Used by: `train.py` via Hydra decorators

**Physics Simulation Layer (MJX/Brax):**
- Purpose: Simulate four_bar_bot mechanics, apply forces, compute contact, sample randomization
- Location: `training/wojtek_rl/base.py` (base) + `env.py`, `env_getup.py`, `env_jump.py` (task-specific)
- Contains: MuJoCo model loading, terrain handling, physics stepping, observation assembly, reward computation
- Depends on: `mujoco.mjx` (backend: warp/jax), `brax` physics wrappers, `ml_collections` config
- Used by: PPO trainer (vectorized env)

**Training Layer (Brax/PPO):**
- Purpose: Run PPO algorithm over rollouts, checkpoint, manage training state
- Location: `training/wojtek_rl/train.py`, `ppo_rnd.py`
- Contains: PPO parameter setup, training loop, early-stopping logic, checkpoint/restore
- Depends on: Brax `ppo` trainer, task envs, WandB logging
- Used by: `run.sh train` entrypoint

**Evaluation Layer:**
- Purpose: Run policies through systematic test scenarios and compute metrics
- Location: `training/wojtek_rl/eval.py`, `battery.py`, `courses.py`, `imu_grid.py`
- Contains: Trajectory playback, metric aggregation, robustness grids (IMU noise, stiffness, lag)
- Depends on: Trained checkpoints, JAX (CPU mode), scenario/terrain definitions
- Used by: `run.sh eval`, `battery`, `courses`, `imu-grid`

**Export/Contract Layer:**
- Purpose: Convert trained checkpoint to deployable form; validate end-to-end equivalence
- Location: `training/wojtek_rl/export_policy.py`, `deploy_contract.py`, `np_policy.py`
- Contains: MLP weight extraction, contract schema (schema 2), numpy runtime validation
- Depends on: Training env (to extract resolved values), numpy
- Used by: `run.sh export`

**Deployment Layer (ROS):**
- Purpose: Load policies, manage robot state, execute control loop, handle hardware I/O
- Location: `ros/src/wojtek_policy/`, `wojtek_bringup/`
- Contains: Policy loading, 50 Hz control loop, state machine (disarmed → standing → locomotion), hardware interfaces
- Depends on: Deployed `policy.npz` + `policy_meta.json`, ROS 2 (jazzy), hardware drivers
- Used by: Real robot, simulator, benchmarking

**Infrastructure Layer:**
- Purpose: Launch management, provisioning, visualization, benchmarking
- Location: `ros/deploy.sh`, `ros/src/wojtek_bringup/wojtek_bringup/robot.py`, launch files
- Contains: SSH provisioning, rsync deployment, RViz/PlotJuggler setup, policy resolution from HuggingFace
- Depends on: SSH access, Python, colcon, ROS 2 tools
- Used by: Human operators, CI/CD

## Data Flow

### Primary Request Path (Training)

1. **Hydra config resolution** (`train.py`, line 140+)
   - Hydra loads base `conf/config.yaml`, merges `+experiment=<preset>`, then CLI overrides
   - Result: `DictConfig` with fully-resolved Hydra job

2. **Environment construction** (`train.py`, line 160+)
   - `task.env` (from resolved config) → task-specific env class (e.g., `WojtekJoystick`)
   - Base class `WojtekEnv.__init__()` loads `ros/src/wojtek_description/mujoco/wojtek.xml`
   - Terrain loaded if `task.env.terrain.enable=true`
   - Backend resolved: `auto` → `warp` (GPU+MJWarp) or `jax` (CPU fallback)

3. **PPO training** (`train.py`, line 180+)
   - Brax `ppo.train()` loop: rollout → GAE → PPO update steps → eval checkpoints
   - Every N steps: eval env runs, reward averaged, checkpoint saved to `training/runs/<run_name>/`
   - Early stop triggers if plateaued (reward stagnant for N evals)

4. **Checkpoint saved** (`training/runs/<run_name>/checkpoints/`)
   - Brax checkpoint contains: optimizer state, network params, training metadata
   - Run metadata (env_config, task, PPO hyperparams) saved to run's metadata file

### Policy Export Path

1. **Load run** (`export_policy.py`, line 100+)
   - Read run metadata, reconstruct env with exact config
   - Force JAX backend + 1 env (export is CPU-bound)

2. **Build deployment contract** (`deploy_contract.py`, line 100+)
   - Enumerate all env config keys: classify as CONSUMED (deploy-relevant) or TRAINING_ONLY
   - Compute resolved values: obs layout, action scale, motor target anchors, height table
   - Contract schema 2: JSON metadata with all numbers the policy needs

3. **Extract MLP weights** (`export_policy.py`, line 200+)
   - Brax checkpoint → numpy arrays (norm_mean/std + hidden_i_kernel/bias layers)
   - Save to `policy.npz` (compressed numpy archive)

4. **Validate end-to-end** (`export_policy.py`, line 250+)
   - Run numpy mirror of network inference against brax version: must match exactly
   - Load `WojtekPolicy` runtime (same code the robot runs) and step it
   - Compare against reference pipeline: must round-trip identically
   - Fail if contract and runtime disagree

### Deployment Path (Robot)

1. **Resolve policy** (`ros/deploy.sh`, line 126+)
   - Default policy pinned in `ros/src/wojtek_policy/policy_source.py`
   - If not cached locally, fetch from HuggingFace and verify
   - Store in `ros/policies/<org/name>/`

2. **Sync to RPi** (`ros/deploy.sh`, line 150+)
   - `rsync ros/src/ → rpi:wojtek_ws/src/` (excludes build, install, caches, PC-only packages)
   - RPi builds with `colcon build --packages-up-to wojtek_bringup`
   - No policy.npz/meta.json in git; copied separately from policy store

3. **Launch robot stack** (`robot.py`, line 90+)
   - SSH to RPi, start `wojtek-robot.service` (systemd unit with RT limits + taskset isolation)
   - Service runs `ros2 launch wojtek_bringup real.launch.py` on RPi
   - PC runs visualization (RViz, PlotJuggler) locally

4. **Control loop initialization** (`real_io_node.py`, line 50+)
   - Load `policy.npz` + `policy_meta.json` via `WojtekPolicy`
   - Subscribe to `/wojtek/command` (joy input), sync with IMU/encoder topics
   - State machine: DISARMED → ZERO (calibrate) → STAND_UP → ARMED (ready for locomotion)

5. **Policy step loop** (`real_io_node.py`, line 150+, `policy_node.py`)
   - Read IMU (gyro, gravity accel), joint encoders, command
   - Assemble observation vector (exact same layout as training env)
   - Call `policy.step(obs)` → returns 12-element motor target vector (+ 12 torque if tau_ff enabled)
   - Clamp to motor range, apply action filter if configured
   - Send target to MD80 servo over CAN, command executes at next servo step

**State Management:**
- Training state: PPO optimizer, network params, RNG seed → checkpointed to disk
- Deployed state: Policy network params (frozen), action filter state (low-pass), last observation → kept in memory on RPi

## Key Abstractions

**Environment Interface (MJXEnv subclass):**
- Purpose: Encapsulate task definition (reward, obs, reset, step)
- Examples: `WojtekJoystick` (velocity tracking), `WojtekGetup` (fall recovery), `WojtekJump` (jumping)
- Pattern: Inherit `WojtekEnv` base, override `reset()`, `step()`, `_compute_reward()`, `_get_observations()`

**Deployment Contract (schema 2):**
- Purpose: Complete specification of everything deploy side needs, in resolved numerical form
- Format: `policy_meta.json` (JSON) + `policy.npz` (numpy compressed archive)
- Fields: obs layout, action scale, motor target anchors, height table, network norm params, MLP layer weights
- Validation: If contract and training env disagree, export fails before any artifacts are written

**Task Configuration (Hydra groups):**
- Purpose: Define task-specific settings without code changes (reward scales, command ranges, gait timing)
- Pattern: Each task (joystick, getup, jump) has a Python `default_config()` + optional YAML overrides
- Merge order: base `config.yaml` → `experiment=<preset>` → CLI overrides (resolves deterministically)

**Control Loop (50 Hz):**
- Purpose: Real-time servo loop on the RPi
- Cycle: read sensors → normalize obs → policy.step() → clamp → send motor targets → wait 20ms → repeat
- Safety: DISARMED state prevents torque; explicit ARM command required
- Fallback: Raw service calls (`/wojtek/zero`, `/wojtek/stand_up`, `/wojtek/arm`) if policy fails

## Entry Points

**Training CLI (`training/run.sh`):**
- Location: `training/run.sh` → dispatches to Python subcommands
- Triggers:
  - `train`: Full PPO training with Hydra config
  - `smoke`: Bounded smoke test (100k steps, no WandB)
  - `eval`: Run policy on reference rollout
  - `export`: Convert checkpoint to policy.npz + meta.json
  - `build`, `check`, `build-terrain`: Model XML generation and validation
  - Test: `test` (unit, no env instantiation), `test-slow` (integration, real MJX)

**Deployment CLI (`ros/deploy.sh`):**
- Location: `ros/deploy.sh` (from repo root)
- Triggers:
  - Default: rsync src, build, restart service
  - `--provision`: Full RPi setup (install.sh + RT kernel)
  - `--enable-robot`: Boot autostart for robot service
  - `--policy <ref>`: Deploy alternate policy from HuggingFace

**Robot Bringup (`robot.py`):**
- Location: `ros2 run wojtek_bringup robot [args]`
- Triggers:
  - Default: launch stack on RPi, RViz on PC
  - `--sim`: MuJoCo sim locally (no RPi)
  - `--dry-run`: BENCH mode on RPi (no RT, no torque)
  - `--gamepad`: Enable gamepad teleop
  - `--foxglove`: Foxglove bridge instead of RViz

## Architectural Constraints

- **Single-threaded event loop on RPi**: Control loop runs on isolated cores (taskset), RT priority. No background ROS spinning.
- **Observation must match**: Actor obs layout must be reproducible on robot (IMU + encoders + history only, no privileged signals).
- **No mutual dependencies between experiments/ and ros/**: Experiments never become dependencies of `wojtek_bringup` or reach the robot via `deploy.sh`.
- **Generated files not hand-edited**: `wojtek_mjx.xml`, `scene_mjx.xml` are generated by `build_model.py`; do not commit edits to them.
- **Policies are config changes, not code changes**: Deploying a new policy is a parameter change to launch files, not a code commit.
- **Backend selection is late-binding**: `task.env.sim.backend=auto` picks GPU (warp) or CPU (jax) at env construction time, not compile time.

## Anti-Patterns

### Observation Layout Drift

**What happens:** Training env's obs layout differs from deployed policy's obs layout (e.g., gyro first vs gyro last)

**Why it's wrong:** Policy receives misaligned observations; first 12 actuators get garbage input; robot fails or thrashes

**Do this instead:** Define obs layout once in env, extract it to contract during export (`deploy_contract.py`), validate in deploy runtime (`policy.py`) against meta. If robot is running a policy trained with wrong layout, it will crash immediately on load (schema validation).

### Anchor/Scale Mismatch

**What happens:** Motor targets computed with different anchor vector or action scale between training and deployment

**Why it's wrong:** Policy trained to reach home pose + action*scale ends up hitting different target on robot; learned behaviors transfer poorly or break

**Do this instead:** Export extracts anchor from env's live state (`_home_ctrl`), multiplies by any per-joint scale, writes to contract. Runtime (`policy.py`) re-applies the exact same formula. Validation step runs both and compares; export fails if they diverge.

### Hardcoded Environment Assumptions

**What happens:** Code assumes specific joint order, leg naming, or kinematics instead of reading from model

**Why it's wrong:** Model changes break assumptions silently; policy trained on old robot crashes on new model

**Do this instead:** Derive everything from model at load time (`build_model.py` reads XML, extracts joint indices; `paths.py` defines leg order once). Use these derived values consistently. Tests check symmetry assumptions (KNEE_SINGULARITY, VIB_MIX reach all axes, etc.).

## Error Handling

**Strategy:** Fail explicitly and early. Export validates end-to-end before writing anything. Deployment contract schema is strict (unknown keys rejected).

**Patterns:**
- Training: Hydra `--cfg job --resolve` prints resolved config before running; human reviews before GPU time
- Export: Validate numpy vs brax equivalence, then validate full runtime with reference pipeline
- Deploy: Load policy, fail loudly if schema version mismatch or required metadata fields missing
- Runtime: If policy step returns NaN or out-of-range action, clamp and log (never silently corrupt motor targets)

## Cross-Cutting Concerns

**Logging:** Use `loguru` in training (JSON structured logs, per-step WandB), ROS 2 logging on robot (FATAL/ERROR/WARN/INFO levels)

**Validation:** 
- Training: `test` suite (fast, model-free) on every commit; `test-slow` (integration, real MJX) before deployment claims
- Deployment: Schema validation on policy load; symmetry checks on first step (gyro in expected range, joint angles within model bounds)

**Authentication/Security:**
- HuggingFace: Policy references resolved via `huggingface-hub` SDK; credentials in `~/.huggingface/token` or HF_TOKEN env var (never committed)
- RPi SSH: Non-interactive key-based auth; StrictHostKeyChecking=no in deploy.sh (safe because ansible-style deployment, not general-purpose)
- Robot arming: Explicit user authorization via console (`ARM` button or service call); no auto-arming on boot

---

*Architecture analysis: 2026-09-05*
