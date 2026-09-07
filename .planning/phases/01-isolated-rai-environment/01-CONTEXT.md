# Phase 1: Isolated RAI Environment - Context

**Gathered:** 2026-09-07
**Status:** Ready for planning

<domain>
## Phase Boundary

A developer can bring up a pinned, self-contained RAI environment under
`experiments/wojtek_rai_v1/` that discovers the running Wojtek simulation's
ROS 2 topics, on both the x86-64 laptop and the aarch64 remote GPU dev box,
without editing `ros/docker`, `ros/sim.sh`, or any package under `ros/src/`.
Covers FOUND-01, FOUND-02, FOUND-03, FOUND-04, FOUND-06, FOUND-07: isolation,
pinned install + lockfile, one-command start against the sim, secret hygiene,
model-free test suite, multi-arch. No agent behaviour, no whoami, no chat, no
tools — those are Phase 2 onward.

</domain>

<decisions>
## Implementation Decisions

### Runtime placement
- **D-01:** The RAI process runs inside the existing `wojtek_robot` dev
  container (the one `ros/dev.sh` / `ros/sim.sh` start), not in a sidecar
  image. — **Reversibility:** costly — every later phase's `run.sh` targets,
  the compose override, and the multi-arch verification assume this
  container; moving to a sidecar means re-plumbing DDS, mounts, and the
  install path.
- **D-02:** The experiment reaches the container through a compose override
  file it ships itself (`experiments/wojtek_rai_v1/docker/compose.override.yaml`)
  that bind-mounts the experiment directory; `run.sh` invokes
  `docker compose -f ros/docker/compose.yaml -f <override> ...`. `ros/docker/`
  stays untouched.
- **D-03:** The RAI Python environment is `experiments/wojtek_rai_v1/.venv`
  on the bind mount, created in-container with the image's Python 3.12,
  gitignored, rebuilt per architecture (laptop and dev box each build their
  own). It survives container recreation.
- **D-04:** DDS settings are inherited from `wojtek_robot`
  (`ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `CYCLONEDDS_URI=file:///config/cyclonedds.xml`). The experiment carries no
  copy of the CycloneDDS config.
- **D-05:** `run.sh` offers both `agent` (assumes `ros/sim.sh` already
  running; starts only the RAI process) and `up` (starts the sim via
  `ros/sim.sh`, then the RAI process). Roadmap criterion 2 is verified with
  `agent`.

### Env + lock tooling
- **D-06:** `uv` manages the environment and lockfile; one universal
  `uv.lock` resolves both x86-64 and aarch64. `uv` is installed into the
  container by `run.sh` (pip or static binary), not baked into the image.
- **D-07:** The experiment is its own Python project with
  `experiments/wojtek_rai_v1/pyproject.toml` declaring direct deps
  (`rai-core==2.12.0`, `rai-whoami==0.0.5`, `pytest`, …); `uv.lock` sits
  beside it and is committed.
- **D-08:** Transitive pins (LangChain, LangGraph, langchain-*) live only in
  `uv.lock`; `pyproject.toml` keeps RAI's own ranges. Bumping is a deliberate
  re-lock, reviewed in a commit.
- **D-09:** `rai_interfaces` (ROS message package, not on PyPI) is sourced
  via a `vcs` `.repos` file at a pinned commit SHA
  (`experiments/wojtek_rai_v1/ros/rai_interfaces.repos`); `run.sh build`
  imports it and colcon-builds into the experiment's own `ros_ws/`. No
  submodule, no apt. — **Reversibility:** reversible.

### Layout + naming
- **D-10:** Directory is `experiments/wojtek_rai_v1/` (Wojtek naming rule,
  framework named, versioned like the sibling experiment).
- **D-11:** Vendor and tracing keys come from the repo-root gitignored `.env`
  (extend `.env.example` with placeholder names such as `OPENAI_API_KEY`,
  `LANGFUSE_*`); `run.sh` sources it and passes the variables into the
  container. No experiment-local `.env`.
- **D-12:** RAI's `config.toml` is committed inside the experiment with
  vendor/model names only; API keys are never written to it, RAI reads them
  from environment variables. A model-free test scans tracked config for
  secret-looking values (FOUND-04).
- **D-13:** Inner layout mirrors the sibling experiment: `README.md` (status
  line first), `run.sh`, `pyproject.toml`, `uv.lock`, `config.toml`,
  `docker/compose.override.yaml`, `wojtek_rai/` (Python package), `ros/`
  (`rai_interfaces.repos`, `src/` for own ROS packages in later phases),
  `tests/`, `docs/`.

### Remote dev box use
- **D-14:** Standing authorization for this project: GSD executors may run
  Phase 1 (and later phase) install, build, and test commands on the aarch64
  remote GPU dev box over the operator's existing SSH alias — syncing the
  repo, `docker compose`, `uv sync`, `run.sh test/build/agent`. Never the
  physical robot; remote training jobs still require an explicit ask. The
  box's hostname, login, and alias must never appear in committed files;
  refer to it as "the remote GPU dev box".
- **D-15:** The repo reaches the dev box by `git clone` and push/pull;
  verification there runs from a committed, pushed state, not an rsynced
  working tree.
- **D-16:** "Works on both machines" means the same three commands pass on
  the laptop and on the dev box: `run.sh install`, `run.sh test`, and the
  topic-discovery target (working name `run.sh agent-topics`, lists the
  sim's live topics). Outputs from both machines go into VERIFICATION.md.
- **D-17:** The dev box uses the same `wojtek_robot` compose via `ros/dev.sh`
  plus the experiment override; the image builds for arm64 from
  `ros:jazzy-ros-core`. Any arm64 build breakage inside `ros/docker` is
  recorded as a finding/blocker, not fixed inside this experiment.

### Claude's Discretion
- Exact `run.sh` target names beyond `install`, `test`, `build`, `agent`,
  `up`, and the topic-discovery target.
- How the isolation test detects imports of the experiment from outside and
  proves `ros/deploy.sh` cannot ship it (grep-based checks over `ros/`,
  `training/`, and the rsync path list are acceptable).
- How `uv` is installed into the container and where its cache lives.
- The exact `rai_interfaces` commit SHA (resolve against `rai-core==2.12.0`
  at execution time and record it in the `.repos` file and the lockfile
  commit message).
- README status wording, following the sibling's "EXPERIMENTAL. Not
  production, not on the robot" pattern.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Repo rules and isolation
- `CLAUDE.md` — `experiments/` boundary, Wojtek naming, no secrets or private
  infrastructure identity, Apache-2.0 default, human-authorized robot actions
- `experiments/autonomous_architecture_ros2_v1/README.md` §"The isolation
  rules this experiment follows" — the five isolation rules to mirror;
  `run.sh` and layout to copy
- `experiments/autonomous_architecture_ros2_v1/run.sh` — reference `run.sh`
  shape (self-contained PYTHONPATH, own colcon build outside `ros/src`)

### Research (stack, pitfalls, architecture)
- `.planning/research/STACK.md` §"Installation", §"Version Compatibility",
  §"What NOT to Use" — pinned versions, pip/uv recipe, `rai_interfaces`
  pinning, vendor config, no `openset` extras
- `.planning/research/PITFALLS.md` — install/version skew, unpinned
  LangChain/LangGraph, DDS domain discovery, container bloat, secrets in
  config, isolation violations, test-strategy mistakes
- `.planning/research/ARCHITECTURE.md` — RAI Agent/Connector/Tool model,
  suggested build order (scaffold first)
- `.planning/research/SUMMARY.md` — Phase 1 flags: lock LangChain/LangGraph
  on first install

### Existing infrastructure this phase plugs into
- `ros/docker/compose.yaml` — `wojtek_robot` service: host network,
  `ROS_DOMAIN_ID=42`, CycloneDDS URI, mounts, entrypoint; the override
  extends this file
- `ros/docker/Dockerfile` — `ros:jazzy-ros-core` base, Python 3.12,
  `python3-pip`, `ros-dev-tools`
- `ros/sim.sh` and `ros/dev.sh` — how the container and sim are started;
  `run.sh up` wraps `ros/sim.sh`
- `.env.example` and `.gitignore` — root secrets pattern to extend
- `.planning/codebase/STACK.md`, `.planning/codebase/STRUCTURE.md`,
  `.planning/codebase/TESTING.md` — existing conventions for Python
  projects, tests, and layout

### Requirements and roadmap
- `.planning/REQUIREMENTS.md` — FOUND-01..04, FOUND-06, FOUND-07 wording
- `.planning/ROADMAP.md` §"Phase 1" — the four success criteria
- `.planning/STATE.md` §"Blockers/Concerns" — Phase 1 LangChain lock flag

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `experiments/autonomous_architecture_ros2_v1/run.sh`: pattern for a
  self-contained entry point (`test`, `build`, demo targets), own
  `PYTHONPATH`, own `colcon build --base-paths ros/src`
- `ros/docker/compose.yaml`: already provides host networking, CycloneDDS
  config mount, `ROS_DOMAIN_ID=42`, SSH-agent and HF-cache mounts; the
  override only needs to add the experiment bind mount and env passthrough
- `ros/dev.sh`: starts the container and opens a shell; `run.sh` can wrap the
  same compose invocation with the override file added
- `training/tests/unit` guard test (no env/model instantiation): model for
  the experiment's own "model-free" guard
- `training/tests/unit/test_job_scripts.py`: existing secret/identity scan
  pattern to copy for the config.toml / `.env.example` test

### Established Patterns
- Experiments never edit shared build/test config; behaviour is added by
  extension and by files inside the experiment directory
- Secrets enter through root `.env` declared in `.env.example`, read at the
  top of scripts (`UBUNTU_PRO_TOKEN` handling in `ros/deploy.sh`)
- Python projects use `pyproject.toml` (Hatchling in `training/`); tests via
  pytest, `test`/`test-slow` hard split
- Generated/arch-specific artifacts (`.venv/`, caches) are gitignored at root

### Integration Points
- Container: `wojtek_robot` service, extended by
  `experiments/wojtek_rai_v1/docker/compose.override.yaml`
- Sim topics the discovery target must see: `cmd_vel`, camera/perception
  topics published by `ros/sim.sh` (`wojtek_pc` sim launch)
- Root `.env.example` gains placeholder entries for vendor and Langfuse keys
- Root `.gitignore` already covers `.venv/` and `.env`; experiment may add
  its own `.gitignore` for `ros_ws/build`, `install`, `log`

</code_context>

<specifics>
## Specific Ideas

- Verification on both machines is literal: paste the outputs of the same
  three commands from the laptop and from the remote GPU dev box into
  VERIFICATION.md.
- The remote GPU dev box is aarch64 (DGX Spark class), has Docker but no
  host ROS; it is reachable through an SSH alias in the operator's local
  `~/.ssh/config`. Never write its identity into the repo.
- Keep the container image unchanged; everything RAI-specific lives on the
  bind mount so `rm -rf experiments/wojtek_rai_v1` leaves nothing behind.

</specifics>

<deferred>
## Deferred Ideas

- Sidecar RAI container with its own image (rejected for now; revisit if the
  shared container becomes crowded or the Jetson deployment needs an image)
- Experiment-local `.env` layered over root (rejected; single secrets file)
- apt-installed `ros-jazzy-rai-interfaces` (unverified availability; `.repos`
  pin chosen)

</deferred>

---

*Phase: 01-isolated-rai-environment*
*Context gathered: 2026-09-07*
