# External Integrations

**Analysis Date:** 2026-09-05

## APIs & External Services

**Experiment Tracking & Model Hosting:**
- WandB (Weights & Biases) — Training metrics, artifacts, checkpoints
  - SDK: `wandb>=0.23.0`
  - Auth: Environment variable (auto-detected from machine or token file)
  - Config: `training/wojtek_rl/conf/config.yaml` → `wandb.enable`, `wandb.project=fbb-locomotion`
  - Usage: `wojtek_rl/train.py` logs PPO rewards, policy checkpoints, videos to WandB runs
  - Offline mode: Supported via `wandb.mode=offline` (sync later)

- Hugging Face Hub — Policy distribution and keeper model repository
  - SDK: `huggingface-hub>=0.26`
  - Auth: `~/.huggingface/token` (standard HF credentials)
  - Usage: 
    - Policy export: `./training/run.sh export --run runs/[run_name]` → `policy.npz` + `policy_meta.json` uploaded to keeper repo
    - Policy loading: `ros/deploy.sh --policy <org/repo[@revision]>` fetches from HF and syncs to RPi
    - Policy source: `ros/src/wojtek_policy/wojtek_policy/policy_source.py` resolves refs
  - Keepers: Published to `https://huggingface.co/[HF_ORG]/wojtek-*-locomotion` and variants

**Vision & Language Models:**
- Anthropic Claude API (optional VLM navigation)
  - SDK: `anthropic>=0.40` (feature-gated `vlm` extra)
  - Auth: `ANTHROPIC_API_KEY` environment variable
  - Usage: `wojtek_rl/vlm_nav.py` async queries for mid-level navigation commands
  - Client: `AsyncAnthropic` (non-blocking inference)
  - Trigger: Room demo and navigation eval (`training/run.sh nav-eval`, `training/run.sh nav-episode`)

- MLX-VLM (on-device Qwen3-VL, Apple Silicon only)
  - SDK: `mlx-vlm>=0.3.3` (feature-gated `vlm-local` extra)
  - Auth: Weights auto-download from HuggingFace Hub on first use
  - Usage: `wojtek_rl/vlm_local.py` runs inference locally (macOS with Apple Neural Engine)
  - Deployment: Wojtek Brain experiment (`experiments/autonomous_architecture_ros2_v1/`)

## Data Storage

**Checkpoints & Models:**
- Local filesystem: `training/runs/[run_name]/` (training outputs, gitignored)
  - `latest.npz` — Latest policy checkpoint
  - `videos/` — Rendered evaluation clips
  - `run.json` — Metadata (reward, PPO timesteps, config)
- Hugging Face Hub: Policy keeper repositories
  - Single `policy.npz` + `policy_meta.json` (schema-2 deployment contract)
  - Readable metadata: `policy_source.py` or `hf_hub_download()`

**On-Robot Policy Storage:**
- Raspberry Pi: `ros/policies/` (synced via `ros/deploy.sh`)
  - gitignored, populated at deployment time
  - Single policy reference loaded via launch param: `policy:=<org/repo[@rev] | dir>`

**Bags & Recordings:**
- ROS Bags: `training/runs/[run_name]/rosbag2_*` (if recording enabled)
  - Full topic tree (joint state, IMU, policy inputs/outputs)
  - Reader: `rosbags.highlevel.AnyReader` (ROS-free, used in `wojtek_rl/sysid/bag.py`)
- Video: `training/runs/[run_name]/videos/` (evaluation MP4s)

**Configuration & State:**
- No database: All state is YAML (Hydra configs) or JSON (run metadata)
- Model source queries: Direct HF API calls via `huggingface-hub`

## File Storage

**On-device storage:**
- Local filesystem only (no S3/GCS)
- Cache locations:
  - `training/.jax_cache/` — JAX XLA compilation cache (persistent, speeds up repeated runs)
  - `training/.venv/` — Python virtual environment (gitignored)
  - `ros/policies/` — Downloaded policy artifacts (gitignored)

## Caching

**JAX Compilation:**
- Persistent disk cache: `training/.jax_cache/`
- Env var: `JAX_COMPILATION_CACHE_DIR` (default: `$PWD/.jax_cache`)
- Behavior: First run compiles ~6m (cold), repeat runs skip recompilation
- Integration test setting: `JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS=0` (full cache reuse)

**MuJoCo Model Cache:**
- None (models rebuilt on every `training/run.sh build`)

## Authentication & Identity

**WandB:**
- Method: Auto-detected from local machine login or `WANDB_API_KEY` env var
- Fallback: Offline mode `wandb.mode=offline` (no API needed)
- Project: `fbb-locomotion` (default, configurable via `wandb.project`)

**Hugging Face Hub:**
- Method: Token file at `~/.huggingface/token`
- Fallback: Unauthenticated access (if repo is public)
- Usage: Policy keeper repos read by `ros/deploy.sh`; write by `./training/run.sh export`

**Anthropic API:**
- Method: `ANTHROPIC_API_KEY` environment variable
- Conditional: Only needed if `vlm` extra installed and VLM navigation enabled
- Error handling: Graceful degradation if VLM is unavailable (navigation fallback in `wojtek_rl/vlm_nav.py`)

**Raspberry Pi SSH:**
- Method: Key-based SSH at fixed anchor `10.42.0.2` (env var: `RPI_HOST`)
- Auth: `~/.ssh/id_rsa` (assumed; no password auth in `deploy.sh`)

## Monitoring & Observability

**Training Metrics:**
- WandB charts: PPO reward, policy loss, actor/critic losses, learning rate
- Stored in WandB run artifacts
- Exported to JSON via WandB API (used by reporting scripts)

**Logs:**
- `loguru>=0.7.3` for structured logs in Python code
- File output: Optionally sent to `training/runs/[run_name]/logs.txt` (not implemented by default)
- Console: Real-time progress via `tqdm` during training and eval

**Error Tracking:**
- None (no Sentry, Datadog, etc.)
- Failures logged to stdout/stderr; WandB run marked failed if training crashes

## CI/CD & Deployment

**Hosting:**
- Raspberry Pi 4 (robot-side, 10.42.0.2)
- PC/Workstation (training & control)
- Hugging Face Hub (model distribution)
- WandB cloud (experiment tracking)

**CI Pipeline:**
- None (no GitHub Actions, GitLab CI, Jenkins)
- Manual validation: `./training/run.sh test` (unit tests, ~3s) and `./training/run.sh test-slow` (integration with real MJX, ~6m cold)
- Before-commit: `git diff --check` for documentation changes

**Deployment Process:**
- `ros/deploy.sh` — Main deployment script
  - Flags: `--provision` (fresh RPi install), `--enable-robot` (boot autostart), `--policy <ref>` (select policy)
  - Steps:
    1. Source `.env` for secrets (UBUNTU_PRO_TOKEN, RPI_HOST)
    2. Optional: Run `deploy/rpi/install.sh` (RT kernel, network, service setup)
    3. rsync `ros/src/` to RPi workspace
    4. SSH into RPi, run `colcon build --packages-up-to wojtek_bringup`
    5. Restart `wojtek-robot.service` (systemd service)
  - Policy switching: Resolves HF ref, syncs `policy.npz` to RPi `policies/`
- Docker: `ros/docker/Dockerfile` builds dev environment
  - Base: `ros:jazzy-ros-core`
  - Installs: Candle SDK (candle repo clone + git checkout), rosdep dependencies, MuJoCo pip packages
  - Multi-layer cache: package.xml → `apt-get install` → source copy → colcon build

**Deployment Secrets:**
- Stored in `.env` (gitignored, .env.example provided)
  - `UBUNTU_PRO_TOKEN` — Required for RT kernel provisioning
  - `RPI_HOST` — Optional (default: `rpi@10.42.0.2`)
- No secrets committed to repo (enforced by `training/tests/unit/test_job_scripts.py` and CLAUDE.md rules)

## Environment Configuration

**Required env vars:**
- None mandatory at runtime (all have sensible defaults)
- Optional: `WANDB_API_KEY`, `ANTHROPIC_API_KEY`, `RPI_HOST`, `UBUNTU_PRO_TOKEN`

**Feature flags (Hydra overrides):**
- `wandb.enable=true|false` — Log to WandB (default: true)
- `task.env.sim.backend=auto|warp|jax` — Physics backend (default: auto = warp on CUDA GPU, jax on CPU)
- `+experiment=locomotion|getup|jump|...` — Preset config group

**Secrets location:**
- `.env` (gitignored, root of repo)
- `~/.huggingface/token` (HuggingFace credentials)
- `~/.ssh/id_rsa` (SSH key for RPi)
- WandB token: Auto-detected from login or `WANDB_API_KEY` env var

## Webhooks & Callbacks

**Incoming:**
- None

**Outgoing:**
- WandB: Streaming metrics and artifacts from training process
- Hugging Face: Policy `policy.npz` + metadata uploaded on `./training/run.sh export`

**Recording:**
- ROS Bags: Full topics recorded to `training/runs/[run_name]/rosbag2_*/` on PC and robot (automatic via `robot.launch.py` and `sim.launch.py`)
  - Reader: Python code in `wojtek_rl/sysid/bag.py` (ROS-free, uses `rosbags`)

## Hardware Communication

**Robot Control Loop:**
- ROS 2 topic: `/joint_cmd` (std_msgs/Float64MultiArray) — Policy outputs to motors
- ROS 2 topic: `/joint_state` (sensor_msgs/JointState) — Motor feedback to policy
- Rate: 400 Hz (hard real-time with RT kernel on RPi)
- Hardware abstraction: `hardware_interface` (pluginlib-loaded drivers)
  - `md80_hardware_interface` — MD80 BLDC via Candle SDK
  - `imu_i2c_hardware_interface` — I2C IMU
  - `bmx160_serial_hardware_interface` / `bmi160_serial_hardware_interface` — Serial IMU (legacy)

**Camera (D435 Realsense):**
- ROS 2 topic: `/camera/*/image_rect_color` (sensor_msgs/Image)
- Used by: Benchmark camera node, vision-based navigation (experimental)

**Magnetometer:**
- ROS 2 topic: `/imu/mag` (sensor_msgs/MagneticField)
- Published by: `magnetometer_broadcaster` (controller-level plugin)

## Policy Deployment Pipeline

1. **Training**: `./training/run.sh train +experiment=locomotion` → checkpoint in `runs/wojtek_locomotion/latest.npz`
2. **Evaluation**: `./training/run.sh eval --run runs/wojtek_locomotion --x-vel 0.3`
3. **Export**: `./training/run.sh export --run runs/wojtek_locomotion` → `policy.npz` + `policy_meta.json`
4. **Publish** (manual): Upload to Hugging Face keeper repo `[ORG]/wojtek-locomotion`
5. **Deploy**: 
   - Option A (default pin): `./ros/deploy.sh` (uses policy_source pinned ref)
   - Option B (one-off): `./ros/deploy.sh --policy org/repo[@rev]` (fetches and syncs)
6. **Robot**: `wojtek_policy` ROS node loads policy and runs inference loop

---

*Integration audit: 2026-09-05*
