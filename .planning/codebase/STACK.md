# Technology Stack

**Analysis Date:** 2026-09-05

## Languages

**Primary:**
- Python 3.11-3.13 (dev: 3.12) - ML training, ROS nodes, CLI tools
- C++ (ROS 2 packages) - Hardware interfaces (MD80, IMU drivers), MuJoCo plugin
- Shell/Bash - Deployment and training orchestration

## Runtime

**Environment:**
- ROS 2 Jazzy (Ubuntu base)
- Python virtual environment in `training/.venv`
- JAX compilation cache: `training/.jax_cache`

**Package Manager:**
- pip (Python packages, including JAX CUDA 12 variant on Linux)
- rosdep (ROS package resolution)
- colcon (ROS 2 workspace build)

**Build System:**
- Hatchling (Python wheel builds) >= 1.26
- CMake (C++ ROS packages)
- Hydra CLI configuration framework (training)

## Frameworks

**Core ML:**
- JAX 0.9.2 (CUDA 12 on Linux, CPU on macOS)
  - Backend: MJWarp (Warp-lang 1.13.0, NVIDIA's Warp via MuJoCo MJX)
  - Fallback: JAX pure-JAX backend on CPU
- MuJoCo 3.2.0+ (physics engine)
- MuJoCo MJX 3.10.0 (XLA-compiled physics)
- Brax 0.12.1+ (PPO trainer, environment wrappers)
- Playground (Go1 reference PPO config)

**Training & Configuration:**
- Hydra 1.3.4+ (config management, experiment presets)
- WandB (experiment tracking, required at runtime with env var)
- ml-collections (config utilities)
- Loguru (structured logging)

**ROS 2:**
- rclpy (Python ROS client library)
- ros2_control (hardware abstraction)
- controller_manager (motor/joint controller daemon)
- ros2_controllers (controller implementations)
- robot_state_publisher (TF broadcasts)
- xacro (URDF macro processor)

**UI & Visualization:**
- PyQt5 (operator console GUI) — `wojtek_pc` only
- FastAPI + Uvicorn (web console, room demo)
- WebSockets (wsproto) (live browser UI)
- RViz2 (3D visualization)
- PlotJuggler (real-time plotting)
- Foxglove Bridge (native app websocket bridge for macOS)
- teleop_twist_keyboard (keyboard remote control)

**Testing:**
- pytest 8.0.0+ (unit and integration tests)
- JAX compilation cache for deterministic integration tests

**Build/Dev:**
- Ruff 0.14.1+ (linter/formatter)
- Docker (dev environment with ros-dev-tools, compilers)

## Key Dependencies

**Critical (Training):**
- `jax[cuda12]==0.9.2` — GPU acceleration (NVIDIA CUDA 12 on Linux)
- `mujoco-mjx==3.10.0` — Physics engine XLA compilation
- `warp-lang==1.13.0` — MJWarp backend (CUDA physics solver)
- `brax>=0.12.1` — PPO + environment framework
- `wandb>=0.23.0` — Experiment tracking and artifact storage
- `huggingface-hub>=0.26` — Policy distribution (model loading, upload)
- `hydra-core>=1.3.4` — Configuration and CLI

**Sensor & Vision:**
- pupil-apriltags (AprilTag marker detection for rig calibration)
- `python3-numpy` (ubiquitous)
- imageio + imageio-ffmpeg (video rendering)
- mediapy (visualization utilities)
- matplotlib (plotting)
- tqdm (progress bars)

**Optional (Feature-gated):**
- `fastapi` + `uvicorn[standard]` — Demo web server, room navigation app
- `anthropic>=0.40` — Claude API for VLM navigation (`vlm` extra)
- `mlx-vlm>=0.3.3` — On-device Qwen3-VL via MLX (Apple Silicon, `vlm-local` extra)
- `faster-whisper>=1.0` — Speech recognition for navigation (`eval` extra)
- `trimesh>=4.0` + `coacd>=1.0.0` + `pillow` — 3D geometry collision decomposition (`room` extra)
- `rosbags>=0.10` + `cmaes>=0.10` — System identification from ROS bags (`sysid` extra)

**Hardware & Drivers:**
- Candle (MD80 motor SDK, vendored git submodule `src/md80_hardware_interface/3rd_party/candle`)
- hardware_interface, pluginlib (ROS 2 hardware abstraction)
- rclcpp (C++ ROS client for hardware plugins)
- rclcpp_lifecycle (lifecycle component for motor initialization)

**ROS 2 Packages (standard):**
- sensor_msgs, geometry_msgs, std_msgs, std_srvs (message types)
- tf2_ros (transform broadcasting and listening)
- rosbag2_transport + rosbag2_storage_default_plugins (ROS bag recording)
- ros2bag (bag CLI tools)

## Configuration

**Environment Variables:**
- `MUJOCO_GL` — Renderer backend (egl/cgl/osmesa; defaults via OS)
- `JAX_PLATFORMS` — Compute platform (cpu/gpu; auto-selected by default)
- `JAX_COMPILATION_CACHE_DIR` — XLA compilation cache path (default: `training/.jax_cache`)
- `WANDB_PROJECT` — WandB project name (default: from `config.yaml` wandb.project)
- `UBUNTU_PRO_TOKEN` — Ubuntu Pro token for RT kernel (from `.env`)
- `RPI_HOST` — Target RPi SSH host (default: `rpi@10.42.0.2`)
- `REMOTE_WS` — RPi workspace path (default: `wojtek_ws`)
- `ROS_DISTRO` — ROS distro name (default: `jazzy`)
- `ANTHROPIC_API_KEY` — Claude API key for VLM navigation (if `vlm` extra enabled)

**Config Files:**
- `training/wojtek_rl/conf/config.yaml` — Hydra root config (WandB project, domain randomization, PPO overrides, distillation)
- `training/wojtek_rl/conf/task/` — Task presets (joystick, getup, jump, locomotion variants)
- `training/wojtek_rl/conf/experiment/` — Experiment presets (gaits, stiffness ladders)
- `training/wojtek_rl/conf/network/` — Actor-critic network architecture
- `training/wojtek_rl/conf/obs/` — Observation filter presets
- `ros/docker/Dockerfile` — ROS dev environment (ROS Jazzy, Python 3, MuJoCo, AprilTags)
- `.env.example` — Template for deployment secrets (UBUNTU_PRO_TOKEN, RPI_HOST)

**Build Settings:**
- Python build backend: Hatchling (`build-system.requires`)
- Wheel packages: `wojtek_rl`, `wojtek_eval` (from `training/`)
- CMake target: Release mode optimization (`-DCMAKE_BUILD_TYPE=Release`)
- Testing disabled in build (`-DBUILD_TESTING=OFF`)

## Platform Requirements

**Development:**
- Linux (x86-64) with NVIDIA CUDA 12-compatible GPU recommended for training
- macOS (Apple Silicon or Intel) for PC-side simulation and control (no Warp backend)
- Docker (for consistent ROS 2 environment)
- Git with submodule support (for Candle SDK fetch)

**Robot (Raspberry Pi 4):**
- Raspberry Pi 4 (4GB+ RAM recommended)
- Ubuntu 24.04 LTS with real-time kernel patch
- Fixed network anchor: 10.42.0.2 (Ethernet or AP mode)
- Python 3.11+
- No GPU; pure NumPy policy execution on ARM CPU

**Deployment Target:**
- Raspberry Pi 4 (robot-side): CPU-only, minimal GUI
- PC/Workstation (dev/control): NVIDIA GPU for training, display for RViz/browsers
- Hugging Face Hub (policy storage and distribution)
- WandB cloud (training artifact logging)

---

*Stack analysis: 2026-09-05*
