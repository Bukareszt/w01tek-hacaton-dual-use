# Walking Skeleton — Wojtek RAI Integration

**Phase:** 1
**Generated:** 2026-09-07

## Capability Proven End-to-End

A developer runs two commands from a clean checkout — `./experiments/wojtek_rai_v1/run.sh install`
and `./experiments/wojtek_rai_v1/run.sh agent-topics` — and a pinned RAI process running inside the
existing `wojtek_robot` container prints the live simulation's ROS 2 topic list, including `cmd_vel`
and a camera topic, with no file under `ros/docker/`, `ros/sim.sh`, `ros/dev.sh` or `ros/src/`
modified.

This is the non-web interpretation of the standard skeleton checklist agreed for this project:
experiment scaffold = "project scaffold", `run.sh install` (pinned venv + committed lockfile) =
"build/deploy", `run.sh agent-topics` (one real ROS 2 discovery round trip against the running sim)
= "one real read", `run.sh test` green = "test runner wired".

## Architectural Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Runtime host | The existing `wojtek_robot` dev container, extended by a compose override the experiment ships (D-01, D-02) | Inherits `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` from the base service; a sidecar image would mean re-plumbing DDS, mounts and the install path |
| Python environment | `experiments/wojtek_rai_v1/.venv`, created in-container with `uv venv --system-site-packages`, gitignored, rebuilt per architecture (D-03, D-06) | `--system-site-packages` plus a sourced `/opt/ros/jazzy/setup.bash` is the only way one interpreter reaches both `uv`-managed PyPI packages and apt-managed `rclpy`/`cv_bridge` |
| Dependency pinning | `pyproject.toml` holds direct pins (`rai-core==2.12.0`, `rai-whoami==0.0.5`); `uv.lock` is committed and is the only pin on RAI's six unconstrained LangChain/LangGraph transitives (D-07, D-08) | `rai-core@2.12.0` declares `langgraph-prebuilt`, `langchain-aws`, `langchain-openai`, `langchain-ollama`, `langchain-google-genai` and `langchain-community` with no version specifier at all |
| ROS message packages | `rai_interfaces` imported by `vcs` at a pinned commit into `experiments/wojtek_rai_v1/ros_ws/`, built with `colcon --base-paths ros_ws` (D-09) | `ros/deploy.sh` rsyncs `ros/src/` only; a package that never appears there cannot reach the robot even by accident |
| Secrets | Repo-root gitignored `.env`, extended `.env.example`, passed into the container by `run.sh` (D-11, D-12) | Mirrors the existing `UBUNTU_PRO_TOKEN` / `WOJTEK_AP_PSK` handling in `ros/deploy.sh`; RAI's `config.toml` schema has no key fields at all |
| Directory layout | `experiments/wojtek_rai_v1/` mirroring `experiments/autonomous_architecture_ros2_v1/` (D-10, D-13) | The sibling experiment's layout is the repo's proven isolation shape; `rm -rf` of the directory must leave nothing behind |
| Multi-arch strategy | One universal `uv.lock`; each machine builds its own `.venv` and image layer; no cross-compilation (D-16, D-17) | Every RAI dependency either is pure Python or ships `manylinux_*_aarch64` wheels |

## Stack Touched in Phase 1

- [ ] Experiment scaffold — `README.md`, `run.sh`, `pyproject.toml`, `.gitignore`, `wojtek_rai/` package
- [ ] Container extension — `docker/compose.override.yaml` bind-mounting the experiment into `wojtek_robot`
- [ ] Pinned install — `run.sh install` producing a committed `uv.lock`
- [ ] One real ROS 2 read — `run.sh agent-topics` calling `ROS2Connector.get_topics_names_and_types()` against the live sim
- [ ] Colcon overlay — `ros_ws/` with `rai_interfaces` at a pinned commit, outside `ros/src/`
- [ ] Test runner — `run.sh test`, model-free, green with no LLM key, no ROS runtime, no GPU
- [ ] Two-machine record — `docs/VERIFICATION.md` with literal outputs from the x86-64 laptop and the aarch64 remote GPU dev box

## Out of Scope (Deferred to Later Slices)

Explicit, so later phases do not re-litigate Phase 1's minimalism:

- Any agent behaviour: no ReAct agent, no tool registry, no LLM call, no prompt (Phase 2)
- `rai whoami` embodiment assembly, capability document, URDF ingestion (Phase 2)
- HRI topics (`/from_human`, `/to_human`), chat UI, Streamlit (Phase 2)
- Langfuse tracing actually enabled — Phase 1 commits the template with tracing off (Phase 2, FOUND-05)
- Any publisher, service client or action client — Phase 1's ROS surface is read-only discovery (Phase 3+)
- Velocity arbiter, `cmd_vel` writes, stop tool (Phase 3)
- Image tool, robot-state tool (Phase 4)
- `navigate_to`, SCAN-Planner wrapping, execution agent (Phase 5)
- A concrete model vendor choice — `config.toml` ships upstream defaults only (HRI-03 defers this)
- Sidecar RAI container, experiment-local `.env`, apt-installed `ros-jazzy-rai-interfaces` (rejected in CONTEXT.md `<deferred>`)
- Physical robot, Jetson, `ros/deploy.sh` — this phase touches no hardware

## Subsequent Slice Plan

Each later phase adds one vertical slice on top of this skeleton without altering its
architectural decisions:

- Phase 2: a human types into a chat UI and an agent that knows it is Wojtek replies
- Phase 3: a typed walking instruction moves simulated Wojtek through a velocity arbiter
- Phase 4: "what do you see?" and "where are you?" answered from live sim data
- Phase 5: "go to the chair" navigates without hitting furniture, interruptible mid-walk

## Skeleton Risk Recorded at Planning Time

The base compose stack has no extension point for additional compose files: `ros/sim.sh` and
`ros/dev.sh` each build their own `docker compose` argument array and run `up -d`, so a container
brought up with the experiment's override is recreated (losing the bind mount) the next time
`ros/sim.sh` runs, and vice versa. Plan `01-01` opens with a `checkpoint:decision` that puts the
two viable resolutions in front of the developer before any file is written. Whichever option is
chosen becomes part of this skeleton and every later phase's `run.sh` inherits it.
