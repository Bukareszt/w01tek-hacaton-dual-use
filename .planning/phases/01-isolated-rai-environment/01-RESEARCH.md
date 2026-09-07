# Phase 1: Isolated RAI Environment - Research

**Researched:** 2026-09-07
**Domain:** RAI (RobotecAI) embodied-AI framework install/isolation on a ROS 2 Jazzy dev container, multi-arch (x86-64 + aarch64)
**Confidence:** HIGH (all package/version/API claims in this document were re-verified this session directly against PyPI JSON, the `RobotecAI/rai` and `RobotecAI/rai_interfaces` GitHub repos at the pinned tag, `astral-sh/uv`'s own CLI source, and this repo's own `ros/docker/`, `ros/sim.sh`, `ros/dev.sh`, `ros/deploy.sh`, `.env.example`, and the sibling experiment)

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Runtime placement**
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

**Env + lock tooling**
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

**Layout + naming**
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

**Remote dev box use**
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

### Deferred Ideas (OUT OF SCOPE)
- Sidecar RAI container with its own image (rejected for now; revisit if the
  shared container becomes crowded or the Jetson deployment needs an image)
- Experiment-local `.env` layered over root (rejected; single secrets file)
- apt-installed `ros-jazzy-rai-interfaces` (unverified availability; `.repos`
  pin chosen)
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|-------------------|
| FOUND-01 | Self-contained `experiments/` directory, own `README.md`, `run.sh`, Python env, colcon overlay; nothing outside imports it, `ros/deploy.sh` cannot ship it, a test enforces the boundary | See Architecture Patterns (project structure, isolation contract) and Common Pitfalls #9 (isolation violations); test pattern in Code Examples mirrors `training/tests/unit/test_job_scripts.py` and the sibling's `rm -rf` acceptance bar |
| FOUND-02 | Pinned install: `rai-core==2.12.0`, `rai-whoami==0.0.5`, `rai_interfaces` at a pinned commit, LangChain/LangGraph locked in a committed lockfile | See Standard Stack (verified PyPI/GitHub versions, exact dependency ranges from `rai_core`'s own `pyproject.toml`) and Package Legitimacy Audit |
| FOUND-03 | One command starts the agent against `ros/sim.sh`; inherits DDS settings; discovers sim topics with zero edits to `ros/docker`/`ros/sim.sh`/`ros/src/` | See Architecture Patterns (compose override, DDS inheritance) and Common Pitfalls #4 (cv_bridge import chain), #6 (DDS domain inheritance), #7 (config.toml cwd-relative load) |
| FOUND-04 | Vendor/tracing keys only from gitignored `.env`; committed `config.toml` has no secrets; a test rejects secret-looking values in tracked config | See Code Examples (verified upstream `config.toml` template, exact env-var names RAI's vendor dispatch reads) and Common Pitfalls #8 (secrets in RAI config) |
| FOUND-06 | `run.sh test` needs no LLM key, no ROS runtime, no GPU | See Validation Architecture and Common Pitfalls #11 (test strategy) |
| FOUND-07 | Builds and runs on x86-64 laptop and aarch64 remote GPU dev box | See Standard Stack (aarch64 wheel verification for `rai-core`, `faiss-cpu`, `opencv-python-headless`, `uv`) and Environment Availability |
</phase_requirements>

## Summary

RAI's Python side is a lightweight, pure-Python install: `rai-core==2.12.0`
ships as a `py3-none-any` wheel (verified on PyPI) with pinned floors on
`langchain`/`langchain-core`/`langgraph` but genuinely unconstrained
(bare-name, no version specifier) `langgraph-prebuilt`, `langchain-aws`,
`langchain-openai`, `langchain-ollama`, `langchain-google-genai`, and
`langchain-community` — confirmed this session by reading `rai_core`'s own
`pyproject.toml` at the exact `2.12.0` tag, not a later/`development` branch
snapshot. `uv lock` immediately after first install is not optional; it is
the only thing pinning those six packages. All of RAI's remaining transitive
deps that could be architecture-sensitive (`faiss-cpu`, `opencv-python`,
`opencv-python-headless`) ship real `manylinux_aarch64` wheels on PyPI —
verified directly this session — so FOUND-07's multi-arch requirement has no
known blocker on the Python side. `uv` itself publishes a static
`aarch64-unknown-linux-gnu` binary.

The one install-time trap this session's research surfaced that prior
project-level research did not fully spell out: **`rai_core`'s ROS 2 tool
module imports `cv_bridge` at module scope**, and `cv_bridge` is not part of
`ros:jazzy-ros-core` and not installed anywhere in this repo's
`ros/docker/Dockerfile`. Even the plain topic-listing tool
(`GetROS2TopicsNamesAndTypesTool`) lives in the same file
(`rai/tools/ros2/generic/topics.py`) as the image tools that need
`cv_bridge`, so importing RAI's stock ROS 2 toolkit at all — which is the
natural way to satisfy FOUND-03's "lists the sim's live topics" criterion
through RAI's own machinery rather than a hand-rolled `rclpy` call — requires
installing `ros-jazzy-cv-bridge` (or equivalent) into the running container
at `run.sh install`/`build` time, not baked into the image (matching D-06's
pattern for `uv`). Separately, `rai_interfaces`' own `package.xml` declares
`vision_msgs`, `nav2_msgs`, `nav2_simple_commander`, `tf_transformations`,
and `portaudio19-dev` as dependencies (verified by reading the file at the
`0.3.0` tag) — none of which are in this repo's image either, so `run.sh
build` must run `rosdep install` against the imported `rai_interfaces`
source before `colcon build`, not assume the image already has what it
needs.

The RAI venv itself needs to reach two disjoint dependency sources at once:
`uv`-managed pure-Python packages (`rai-core`, `langchain*`, `streamlit`,
`faiss-cpu`, …) and apt/rosdep-managed ROS 2 Python bindings (`rclpy`,
`cv_bridge`, `rosidl_runtime_py`, `tf2_ros`) that live outside any venv's
normal `site-packages`. `uv venv --system-site-packages` (confirmed present
in `uv`'s own CLI source this session) is the mechanism, combined with
sourcing `/opt/ros/jazzy/setup.bash` (which sets `PYTHONPATH`, not a
`site-packages` install) before invoking the venv's Python — `run.sh` must do
both, in that order, for every venv-invoking target.

**Primary recommendation:** Pin `rai-core==2.12.0` + `rai-whoami==0.0.5` via
`uv` inside a `--system-site-packages` venv built with the container's
`python3.12`, run `uv lock` immediately and commit it, install
`ros-jazzy-cv-bridge` + `rosdep install` against the imported `rai_interfaces`
source as explicit `run.sh install`/`build` steps (not image changes), and
verify FOUND-03's topic-discovery target with RAI's own
`ROS2Connector.get_topics_names_and_types()` rather than a bare `rclpy` call,
so the phase actually proves the real import chain later phases depend on.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| RAI Python environment (rai-core, langchain stack) | Experiment / Application process | — | Runs as a plain Python process inside the shared dev container; not a ROS node itself, owns no ROS graph state |
| ROS 2 topic/service/action discovery | Experiment / Application process (via `ROS2Connector`) | ROS 2 / DDS transport | `ROS2Connector` is a thin `rclpy` wrapper; actual discovery happens over CycloneDDS, inherited unmodified from the existing container |
| Container lifecycle (build/start/exec) | Infra / Container tier (`ros/docker/`) | Experiment `docker/compose.override.yaml` | Base image and `wojtek_robot` service definition stay in `ros/docker/`; the experiment only *extends* via a second compose file, never edits the first |
| Dependency resolution & lockfile | Experiment / Build tooling (`uv`) | — | `uv` is installed at runtime into the container, not baked into the image; resolution is entirely the experiment's own concern |
| ROS message definitions (`rai_interfaces`) | Experiment's own colcon overlay (`ros_ws/`) | — | Built outside `ros/src/` specifically so `ros/deploy.sh`'s `rsync ros/src/` + `--packages-up-to wojtek_bringup` cannot reach it |
| Secrets (vendor/tracing API keys) | Repo-root `.env` (gitignored) | `run.sh` (env passthrough into container) | Matches the existing `UBUNTU_PRO_TOKEN`/`WOJTEK_AP_PSK` pattern in `ros/deploy.sh`; no experiment-local secret store |
| Multi-arch verification | Both machines independently (laptop x86-64, remote GPU dev box aarch64) | — | No cross-compilation; each machine builds its own `.venv` and image layer from the same pinned lockfile/Dockerfile |

## Standard Stack

### Core

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|---------------|
| `rai-core` (PyPI) | **2.12.0** — `[VERIFIED: PyPI JSON (pypi.org/pypi/rai-core/json) — full release list includes "2.12.0" and "2.12.1"; GitHub tags API confirms tag "2.12.0" exists on RobotecAI/rai]** | Core RAI framework: `ROS2Connector`, ReAct agent, ROS 2 tool toolkits, model-vendor dispatch | Locked decision (D-07); latest tag before this session's research date, first release on the langchain-1.x line |
| `rai-whoami` (PyPI) | **0.0.5** — `[VERIFIED: PyPI JSON (pypi.org/pypi/rai-whoami/json) — release list: 0.0.1..0.0.5, requires-python "<3.13,>=3.10"]` | Embodiment builder (identity/capabilities from URDF+docs, optional FAISS vector DB) | Locked decision (D-07); named explicitly in the milestone |
| `rai_interfaces` (git, not PyPI) | Pin a commit SHA at import time (D-09/D-14 discretion). Latest published tag as of this session: **0.3.0** at commit `2398f1f3e4c96d790365492294599439a38cdf9a` — `[VERIFIED: api.github.com/repos/RobotecAI/rai_interfaces/tags]`. Upstream `rai_core@2.12.0`'s own `ros_deps.repos` pins this to branch `main` (unpinned) — `[VERIFIED: raw.githubusercontent.com/RobotecAI/rai/2.12.0/ros_deps.repos]` | ROS 2 message/service definitions RAI's connectors and tools import (`HRIMessage`, `AudioMessage`, `RAIDetectionArray`) | Not on PyPI; small `colcon`-buildable package; the `main`-branch pin upstream gives no reproducibility guarantee, so this project must pin its own SHA (tag `0.3.0`'s commit is a reasonable, tested starting point since it is a cut release rather than a moving branch tip) |
| Python | **3.12** — `[ASSUMED: `ros:jazzy-ros-core` targets Ubuntu 24.04, whose default `python3` is 3.12; not independently re-verified inside a running container this session]` | Interpreter for the experiment's venv | Matches `rai-core`/`rai-whoami`'s `requires-python = ">=3.10,<3.13"` and the repo's existing dev convention (`training/.venv` also targets 3.12) |
| `uv` | Latest (**0.9.18** observed on this session's host) — `[VERIFIED: local `uv --version` on the research host; not the container's eventual version, which `run.sh` should pin or track "latest" per D-06's discretion]` | Environment/lockfile manager | Locked decision (D-06); publishes a static `aarch64-unknown-linux-gnu` binary — `[VERIFIED: api.github.com/repos/astral-sh/uv/releases/latest asset list]` |
| `colcon` | Whatever ships with ROS 2 Jazzy inside `wojtek_robot` (already present via `ros-dev-tools` in `ros/docker/Dockerfile`) | Builds `rai_interfaces` (and later, this experiment's own ROS packages) into `ros_ws/` | No new tool needed; already installed image-side |

### Supporting

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `langchain-core` | `>=1.2.10,<2.0.0` — `[VERIFIED: raw.githubusercontent.com/RobotecAI/rai/2.12.0/src/rai_core/pyproject.toml]` | LangChain primitives under RAI's agent loop | Always, transitive via `rai_core` |
| `langchain` | `>=1.0.0,<2.0.0` — `[VERIFIED: same source]` | Agent/tool-calling substrate | Always, transitive |
| `langgraph` | `>=1.1.5,<2.0.0` — `[VERIFIED: same source]` | Agent graph runtime | Always, transitive |
| `langgraph-prebuilt` | **unconstrained** (bare name, no version specifier) — `[VERIFIED: same source — line reads exactly `"langgraph-prebuilt",` with no comparator]` | Prebuilt LangGraph agent constructors RAI's `create_conversational_agent` uses | Always, transitive — **this is the exact package that broke callers upstream in `langchain-ai/langgraph#6363`**; lock the resolved version in `uv.lock` immediately |
| `langchain-aws`, `langchain-openai`, `langchain-ollama`, `langchain-google-genai`, `langchain-community` | **unconstrained** (bare names, no version specifiers) — `[VERIFIED: same source]` | Per-vendor chat/embedding backends + misc community integrations | Always installed transitively regardless of which vendor is actually configured; `langchain-community` explicitly does not follow semver upstream (per LangChain's own release-policy docs, `[CITED: docs.langchain.com/oss/python/release-policy]`) |
| `faiss-cpu` | Transitive dep of `rai_whoami`; PyPI latest observed **1.15.0**, ships `manylinux_2_27_aarch64`/`manylinux_2_28_aarch64` wheels — `[VERIFIED: pypi.org/pypi/faiss-cpu/json file list for the latest release]` | Vector store backing the whoami retrieval tool | Always needed for `rai_whoami`; CPU-only, no GPU dependency; confirmed aarch64-safe |
| `opencv-python` / `opencv-python-headless` | Transitive dep of `rai_core`; PyPI latest observed **5.0.0.93** (headless), ships `manylinux2014_aarch64`/`manylinux_2_28_aarch64` wheels — `[VERIFIED: pypi.org/pypi/opencv-python-headless/json file list]` | Image decode/encode inside `rai_core`'s ROS 2 image tools | Always installed transitively; confirmed aarch64-safe (no source build needed on the dev box) |
| `streamlit` | `>=1.44,<2.0.0` — `[VERIFIED: rai_core pyproject.toml @ 2.12.0]` | Text-chat HMI ("Hello RAI" pattern) | Not exercised this phase (no agent behavior yet — Phase 2+), but installed transitively since it is a hard `rai_core` dependency, not an extra |
| `tomli` / `tomli-w` | `>=2.0.1,<3.0.0` / `>=1.1.0,<2.0.0` — `[VERIFIED: rai_core pyproject.toml @ 2.12.0]` | Reads/writes `config.toml` | Transitive; this is exactly the library `load_config()` uses to parse the committed `config.toml` |
| `pytest` | `>=8.0.0` (repo convention — `[VERIFIED: training/pyproject.toml dev extra, read this session]`, `>=8.0.0`) | Model-free test runner | FOUND-06's test suite; add as a direct dependency in this experiment's own `pyproject.toml` (D-07) |

### Alternatives Considered

| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| `pip install rai-core rai-whoami` (pinned wheels) | Clone the full `RobotecAI/rai` monorepo + `uv sync` + build everything from source | Only needed if a later milestone adopts `rai_perception`/`rai_sim`/`rai_bench`/`rai_s2s`, none of which are on PyPI. Out of scope this phase; dragging in the monorepo now violates the "nothing extra" spirit of `experiments/` isolation |
| Pin `rai_interfaces` to the `0.3.0` tag's commit | Pin to `main`'s current HEAD commit | The tag is a deliberate cut, `main` is a moving target upstream itself pins (unhelpfully) to `main`. Either is a real commit SHA once pinned; the tag is the safer starting point but re-verify message field compatibility against `rai_core@2.12.0`'s expectations before relying on it (RAI's own repo does not guarantee this pairing was tested) |
| `uv venv --system-site-packages` + sourced ROS setup | A fully isolated venv with `rclpy`/`cv_bridge` reinstalled via `pip` inside it | `rclpy` is a compiled extension tied to the system ROS 2 install; reinstalling it via pip inside an isolated venv risks an ABI mismatch against the container's actual DDS libraries. Inheriting the system-installed ROS Python bindings is the standard, lower-risk approach |

**Installation (inside the `wojtek_robot` container, after `ros/dev.sh` or `ros/sim.sh`):**

```bash
# 1. uv, installed at runtime (D-06) — not baked into the image
curl -LsSf https://astral.sh/uv/install.sh | sh   # or a pinned static binary; cache location is Claude's discretion

# 2. venv with access to the ROS 2 Python bindings already on this interpreter
source /opt/ros/jazzy/setup.bash          # PYTHONPATH now carries rclpy, cv_bridge (once apt-installed), rosidl_runtime_py, tf2_ros
cd experiments/wojtek_rai_v1
uv venv --system-site-packages .venv --python python3.12
source .venv/bin/activate

# 3. Pinned RAI stack + this experiment's own deps, from pyproject.toml
uv sync                                    # first run: resolves and writes uv.lock — commit it immediately after

# 4. System packages rai_core's ROS 2 tools need that are NOT in ros/docker/Dockerfile today
#    (installed into the running container by run.sh install, never baked into the image — matches D-06's pattern for uv)
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  ros-jazzy-cv-bridge

# 5. rai_interfaces: vcs import + rosdep + colcon, into the experiment's own ros_ws/ (outside ros/src/)
mkdir -p ros_ws/src
vcs import ros_ws/src < ros/rai_interfaces.repos
rosdep install --from-paths ros_ws/src --ignore-src -r -y   # pulls vision_msgs, nav2_msgs,
                                                              # nav2_simple_commander, tf_transformations,
                                                              # portaudio19-dev — none present in the image today
colcon build --symlink-install --base-paths ros_ws
```

**Version verification performed this session:**
- `rai-core` 2.12.0: confirmed on PyPI JSON release list and as a GitHub tag on `RobotecAI/rai`; wheel is `rai_core-2.12.0-py3-none-any.whl` (pure Python, no arch-specific build) — `[VERIFIED]`
- `rai-whoami` 0.0.5: confirmed on PyPI JSON release list — `[VERIFIED]`
- `rai_interfaces`: no PyPI package exists (expected — it's a ROS message package); latest git tag `0.3.0` confirmed via GitHub tags API — `[VERIFIED]`
- `faiss-cpu`, `opencv-python-headless`: aarch64 wheel files confirmed present in the latest PyPI release's file list — `[VERIFIED]`
- `uv`: aarch64-unknown-linux-gnu static binary confirmed present in the latest GitHub release's asset list — `[VERIFIED]`

## Package Legitimacy Audit

Ecosystem: **pypi**. Ran via the `package-legitimacy check` seam this session. Every package returned a `SUS` verdict, but the *only* reason attached to almost all of them was `unknown-downloads` — the seam's download-stats source was unreachable in this session's network environment, which triggers on every package checked (including universally-known ecosystem staples), not a signal specific to any one package. `repoUrl` resolved correctly for most (confirming the official upstream repo), and `exists: true` for all. Treat `unknown-downloads`-only verdicts here as an environment limitation, not a per-package finding — but the protocol below still applies literally: flag as SUS, planner adds a `checkpoint:human-verify` before install.

| Package | Registry | Published (latest) | Downloads | Source Repo | Verdict | Disposition |
|---------|----------|---------------------|-----------|--------------|---------|-------------|
| `rai-core` | pypi | 2026-06-09 | unknown (seam) | none returned by seam — `[VERIFIED: github.com/RobotecAI/rai is the real upstream, confirmed by direct fetch this session]` | SUS (`unknown-downloads`, `no-repository`) | Flagged — planner adds checkpoint. Independently verified: official RobotecAI org repo, Apache-2.0, matches CONTEXT.md's locked pin |
| `rai-whoami` | pypi | 2025-05-29 | unknown (seam) | none returned by seam — same upstream as above | SUS (`unknown-downloads`, `no-repository`) | Flagged — planner adds checkpoint. Same independent verification as `rai-core` |
| `langchain` | pypi | 2026-09-03 | unknown (seam) | docs.langchain.com | SUS (`too-new`, `unknown-downloads`) | Flagged — planner adds checkpoint. "too-new" reflects the *latest release's* publish date, not the package's age; `langchain` is a foundational, multi-year ecosystem package |
| `langgraph` | pypi | 2026-08-11 | unknown (seam) | github.com/langchain-ai/langgraph | SUS (`too-new`, `unknown-downloads`) | Flagged — planner adds checkpoint. Same caveat as `langchain` |
| `langchain-aws` | pypi | 2026-09-02 | unknown (seam) | none returned | SUS (`too-new`, `unknown-downloads`, `no-repository`) | Flagged — planner adds checkpoint |
| `langchain-openai` | pypi | 2026-08-19 | unknown (seam) | docs.langchain.com | SUS (`too-new`, `unknown-downloads`) | Flagged — planner adds checkpoint |
| `langchain-ollama` | pypi | 2026-04-07 | unknown (seam) | docs.langchain.com | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |
| `langchain-google-genai` | pypi | 2026-09-01 | unknown (seam) | github.com/langchain-ai/langchain-google | SUS (`too-new`, `unknown-downloads`) | Flagged — planner adds checkpoint |
| `langchain-community` | pypi | 2026-05-22 | unknown (seam) | github.com/langchain-ai/langchain-community | SUS (`unknown-downloads`, …) | Flagged — planner adds checkpoint |
| `faiss-cpu` | pypi | 2026-08-03 | unknown (seam) | github.com/facebookresearch/faiss | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint. Meta/Facebook Research's own repo |
| `streamlit` | pypi | 2026-09-01 | unknown (seam) | streamlit.io | SUS (`too-new`, `unknown-downloads`) | Flagged — planner adds checkpoint |
| `opencv-python` / `opencv-python-headless` | pypi | 2026-07-02 | unknown (seam) | github.com/opencv/opencv-python | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |
| `transforms3d` | pypi | 2024-06-20 | unknown (seam) | github.com/matthew-brett/transforms3d | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |
| `pillow` | pypi | 2026-07-01 | unknown (seam) | github.com/python-pillow/Pillow | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |
| `tomli` / `tomli-w` | pypi | 2026-03-25 / 2025-01-15 | unknown (seam) | github.com/hukkin/tomli(-w) | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |
| `pydub` | pypi | 2021-03-10 | unknown (seam) | pydub.com | SUS (`unknown-downloads`) | Flagged — planner adds checkpoint |

**Packages removed due to `[SLOP]` verdict:** none — no package in this list returned `SLOP`.
**Packages flagged as suspicious `[SUS]`:** all packages listed above, per the seam's literal output. The planner should add a single `checkpoint:human-verify` before the first `uv sync`/install task rather than one per package, since the flagging reason is uniform (`unknown-downloads`, an environment limitation) rather than package-specific risk signals. `rai-core` and `rai-whoami` are the two genuinely worth a closer human look on their own merits (smaller, single-vendor project) — independently verified this session via direct GitHub/PyPI inspection, not the seam.

## Architecture Patterns

### System Architecture Diagram

```
┌─────────────────────────── Host (laptop x86-64 / dev box aarch64) ───────────────────────────┐
│                                                                                                 │
│   ros/dev.sh or ros/sim.sh          experiments/wojtek_rai_v1/run.sh {install|build|test|      │
│        │                                            agent|up}                                  │
│        │ docker compose -f ros/docker/compose.yaml           │                                 │
│        │             -f experiments/wojtek_rai_v1/docker/compose.override.yaml up               │
│        ▼                                                      ▼                                │
│   ┌─────────────────────────── wojtek_robot container (unchanged image) ─────────────────────┐ │
│   │                                                                                            │ │
│   │  Bind mounts: ros/src (existing) + experiments/wojtek_rai_v1 (new, via the override)       │ │
│   │                                                                                            │ │
│   │  ROS_DOMAIN_ID=42, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp,                                  │ │
│   │  CYCLONEDDS_URI=file:///config/cyclonedds.xml   (inherited, no experiment-side copy)       │ │
│   │                                                                                            │ │
│   │  ┌── existing ROS 2 graph (untouched) ──┐     ┌── RAI experiment process (new) ─────────┐ │ │
│   │  │  wojtek_policy (50 Hz control loop)   │     │  source /opt/ros/jazzy/setup.bash        │ │ │
│   │  │  wojtek_perception_bringup (camera)   │◄────┤  → PYTHONPATH carries rclpy, cv_bridge,  │ │ │
│   │  │  MuJoCo sim (ros/sim.sh)              │ DDS │    rosidl_runtime_py into the venv        │ │ │
│   │  │  publishes cmd_vel, camera/*          │     │  .venv (uv venv --system-site-packages)  │ │ │
│   │  └────────────────────────────────────────┘     │  rai_core.ROS2Connector                  │ │ │
│   │                                                  │    .get_topics_names_and_types()         │ │ │
│   │                                                  │  reads config.toml (cwd-relative load)   │ │ │
│   │                                                  └───────────────────────────────────────────┘ │ │
│   │                                                                                                │ │
│   │  Separate colcon overlay: experiments/wojtek_rai_v1/ros_ws/  (rai_interfaces, built here,       │ │
│   │  outside ros/src/ — ros/deploy.sh's rsync ros/src/ + --packages-up-to wojtek_bringup cannot     │ │
│   │  reach it)                                                                                      │ │
│   └────────────────────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                                 │
│   Secrets: repo-root .env (gitignored) → run.sh sources it → passed into container as env vars  │
│            OPENAI_API_KEY / AWS_* / GOOGLE_API_KEY / LANGFUSE_* (never written to config.toml)   │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### Recommended Project Structure

Matches D-13 exactly (mirrors `experiments/autonomous_architecture_ros2_v1`'s proven layout):

```
experiments/wojtek_rai_v1/
├── README.md                      # status line first: "EXPERIMENTAL. Not production, not on the robot."
├── run.sh                         # install | build | test | agent | up  (+ topic-discovery target)
├── pyproject.toml                 # direct deps: rai-core==2.12.0, rai-whoami==0.0.5, pytest, ...
├── uv.lock                        # committed; the only real pin on langgraph-prebuilt/langchain-*
├── config.toml                    # RAI vendor config — model names only, no keys
├── docker/
│   └── compose.override.yaml      # bind-mounts this directory into wojtek_robot; ros/docker/ untouched
├── ros/
│   ├── rai_interfaces.repos       # vcs import spec, pinned to a commit SHA (not `main`)
│   └── src/                       # this experiment's own ROS packages (later phases)
├── wojtek_rai/                    # this experiment's Python package
├── tests/                         # model-free unit tests (FOUND-06)
└── docs/
```

`.venv/`, `ros_ws/build`, `ros_ws/install`, `ros_ws/log` are gitignored inside the experiment (matching D-13's note that `.venv` is gitignored and rebuilt per architecture).

### Pattern 1: Compose override extends, never edits

**What:** `docker compose -f ros/docker/compose.yaml -f experiments/wojtek_rai_v1/docker/compose.override.yaml up -d`. The override adds a bind mount for the experiment directory and nothing else — no new service, no image change.
**When to use:** Every `run.sh` target that needs the container running.
**Example:**
```yaml
# experiments/wojtek_rai_v1/docker/compose.override.yaml
services:
  wojtek_robot:
    volumes:
      - ../:/ros2_ws/experiments/wojtek_rai_v1
```

### Pattern 2: `--system-site-packages` venv + sourced ROS setup, in that order

**What:** Create the venv with `uv venv --system-site-packages`, then always source `/opt/ros/jazzy/setup.bash` (and `ros_ws/install/setup.bash` once built) *before* invoking the venv's Python, in every `run.sh` target that touches ROS.
**When to use:** `run.sh agent`, `run.sh up`, and any test that imports `rclpy` or `rai.communication.ros2`.
**Why:** `--system-site-packages` gives the venv access to packages installed into the *global* interpreter's `site-packages`; ROS 2's apt-installed Python bindings are exposed via a `PYTHONPATH` addition from `setup.bash`, not a `site-packages` install — both mechanisms are needed together. Source: `[VERIFIED: crates/uv-cli/src/lib.rs in astral-sh/uv, read directly this session]` — verbatim: *"Give the virtual environment access to the system site packages directory... The `--system-site-packages` flag will provide the virtual environment with access to the system site packages directory at runtime, but will not affect the behavior of uv commands."*
**Example:**
```bash
# inside run.sh, before any venv-invoking command that touches ROS
source /opt/ros/jazzy/setup.bash
[ -f "$HERE/ros_ws/install/setup.bash" ] && source "$HERE/ros_ws/install/setup.bash"
exec "$HERE/.venv/bin/python" "$@"
```

### Pattern 3: `rai_interfaces` built outside `ros/src/`, exactly like the sibling experiment's ROS packages

**What:** `vcs import` into `experiments/wojtek_rai_v1/ros_ws/src/`, `colcon build --base-paths ros_ws` — never `ros/src/`.
**When to use:** `run.sh build`.
**Why:** identical rationale to the sibling experiment's rule 3 — `ros/deploy.sh` rsyncs `ros/src/` wholesale; a package that never appears there cannot reach the robot even by accident.

### Pattern 4: RAI's `config.toml` load is cwd-relative unless a path is passed

**What:** `rai.initialization.model_initialization.load_config(config_path=None)` does `open("config.toml", "rb")` — a bare relative path against the current working directory, not an absolute or `$HOME`-relative default.
**Source:** `[VERIFIED: raw.githubusercontent.com/RobotecAI/rai/2.12.0/src/rai_core/rai/initialization/model_initialization.py, read directly this session]`
**When to use:** Every place `run.sh` invokes RAI code, `cd` into `experiments/wojtek_rai_v1/` first (or pass `config_path` explicitly) so the committed `config.toml` is actually the one loaded — this is a straightforward but easy-to-miss gotcha since the container's shell default cwd is `/ros2_ws`, not the experiment directory.

### Anti-Patterns to Avoid

- **Baking `uv`, `cv_bridge`, or `rai_interfaces`' rosdep keys into `ros/docker/Dockerfile`:** Violates D-01/D-02/D-06's explicit "not baked into the image" requirement and the isolation rule that `ros/docker/` stays untouched. Install all of these as `run.sh install`/`build` steps that run against the *running* container instead.
- **Skipping `uv lock` after first install:** `rai_core`'s six unconstrained transitive deps (see Standard Stack) will silently drift on the next resolve; this is Pitfall 1 below, not hypothetical — it already happened upstream to LangGraph callers (`langchain-ai/langgraph#6363`).
- **Reading `config.toml` from a different working directory than the experiment root:** produces a confusing `FileNotFoundError` (or, worse, silently reads a wrong `config.toml` if one happens to exist at `/ros2_ws`) rather than an obvious "not configured" error.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|--------------|-----|
| ROS 2 topic/service/action discovery | A bespoke `rclpy` wrapper script | `rai.communication.ros2.ROS2Connector.get_topics_names_and_types()` | This is exactly the mechanism later phases' tools sit on top of; exercising it in Phase 1 (rather than a throwaway `ros2 topic list` shell-out) proves the real import chain (including the `cv_bridge` dependency) works before any agent logic is written |
| Vendor config file parsing | A custom TOML reader for `config.toml` | RAI's own `tomli`/`tomli-w`-based `load_config()` | Already handles the exact schema (`[vendor]`, `[openai]`, `[aws]`, `[ollama]`, `[google]`, `[tracing.langfuse]`, `[tracing.langsmith]`) that later phases' vendor-switch requirement (HRI-03) depends on |
| Secret-pattern scanning in tracked config | A brand-new regex framework | Extend the existing pattern from `training/tests/unit/test_job_scripts.py` (`FORBIDDEN_RE`/`SITE_VALUE_RE` style) with an API-key-shape regex (see Code Examples) | Repo already has a proven, reviewed pattern for exactly this class of test; reuse the shape, adapt the regex targets |
| Multi-arch dependency resolution | A separate `pyproject.toml`/lockfile per architecture | One `uv.lock`, resolved independently on each machine (D-06) | `uv.lock` records a universal resolution when the packages themselves are pure-Python or ship wheels for both platforms — verified true for every RAI dependency checked this session |

**Key insight:** Nearly everything this phase needs already exists either in RAI itself (config loading, ROS 2 connector, topic discovery) or in this repo's own conventions (secret-scanning test pattern, `experiments/` isolation contract, `.env`/`run.sh` secrets handling). The actual net-new work is glue: the compose override, the `run.sh` targets, and the two runtime-installed system-package gaps (`cv_bridge`, `rai_interfaces`' rosdep deps) this session's research surfaced.

## Common Pitfalls

### Pitfall 1: RAI's unconstrained transitive LangChain/LangGraph deps drift on re-resolve

**What goes wrong:** `langgraph-prebuilt`, `langchain-aws`, `langchain-openai`, `langchain-ollama`, `langchain-google-genai`, `langchain-community` have no version specifier at all in `rai_core@2.12.0`'s own `pyproject.toml` — confirmed by reading the file directly this session, not inferred from a changelog. A fresh `uv sync`/`pip install` weeks or months later can silently resolve a different, possibly incompatible set.
**Why it happens:** RAI's own dependency declaration is permissive by design; nothing in `rai_core` protects against this, and it already broke real callers upstream (`langchain-ai/langgraph#6363`, a documented incident, `[CITED: github.com/langchain-ai/langgraph/issues/6363]`).
**How to avoid:** Run `uv lock` immediately after the first successful `uv sync` and commit `uv.lock` in the same commit as `pyproject.toml` (D-07/D-08 already require this — this is the concrete mechanism). Never re-resolve without reviewing the diff.
**Warning signs:** `uv sync` reports package version changes with no corresponding `pyproject.toml` edit; `run.sh test` that passed last week fails today with a traceback inside `langgraph_prebuilt` or `langchain_core.tools`.
**Phase to address:** Phase 1 — this is exactly D-06/D-07/D-08's purpose; the pinning discipline must exist before any other phase adds agent logic on top.

### Pitfall 2: `cv_bridge` is required just to import RAI's stock topic-listing tool, and it is not in this repo's image

**What goes wrong:** `rai/tools/ros2/generic/topics.py` does `from cv_bridge import CvBridge` at module scope — confirmed by reading the file directly this session. `GetROS2TopicsNamesAndTypesTool` (the tool that would satisfy FOUND-03's "lists the sim's live topics" criterion through RAI's own machinery) is defined in that same file. `ros:jazzy-ros-core` + this repo's `ros/docker/Dockerfile` install list does not include `cv_bridge` or any `vision_opencv` package — confirmed by reading the Dockerfile directly this session (`ros-dev-tools`, `rmw-cyclonedds-cpp`, `plotjuggler-ros`, mesa/EGL libs only) and by grepping the existing `ros/src/*/package.xml` files for a `cv_bridge` dependency (none found).
**Why it happens:** `ros:jazzy-ros-core` is a genuinely minimal base; `cv_bridge` lives in the separate `vision_opencv` stack, not `ros-core`. This repo's own image never needed it before (its perception code uses raw `sensor_msgs/Image`/point-cloud processing directly, not `cv_bridge`).
**How to avoid:** `run.sh install` (or `build`) runs `apt-get install -y ros-jazzy-cv-bridge` against the *running* container, matching the D-06 pattern already established for `uv` — not an image change.
**Warning signs:** `ImportError: No module named 'cv_bridge'` the moment any RAI ROS 2 tool (including the plain topic-listing one) is imported, even though `ROS2Connector` itself imports fine.
**Phase to address:** Phase 1 — this blocks FOUND-03 if the topic-discovery target routes through RAI's own toolkit rather than a bare `rclpy` call.

### Pitfall 3: `rai_interfaces`' own `package.xml` needs ROS packages this image doesn't have

**What goes wrong:** `rai_interfaces`' `package.xml` at tag `0.3.0` declares `<depend>vision_msgs</depend>` and `<exec_depend>` entries for `nav2_msgs`, `nav2_simple_commander`, `tf_transformations`, `portaudio19-dev` — confirmed by fetching the file directly this session. None of these are installed in `ros/docker/Dockerfile` today (Nav2 is explicitly out of scope for this project per `.planning/REQUIREMENTS.md`'s Out of Scope table, so nothing in the existing stack pulled `nav2_msgs` in before).
**Why it happens:** `rai_interfaces` is a general-purpose message package serving all of RAI's demos (including Nav2-based ones), not scoped to what any one downstream project actually uses.
**How to avoid:** `run.sh build` must run `rosdep install --from-paths ros_ws/src --ignore-src -r -y` against the imported `rai_interfaces` source *before* `colcon build`, not assume the image's existing rosdep-resolved set covers it. This mirrors exactly what `ros/docker/Dockerfile` already does for Wojtek's own packages (`rosdep install --from-paths src -y -i`), just scoped to the experiment's own overlay.
**Warning signs:** `colcon build` inside `ros_ws/` fails with unresolved `find_package(nav2_msgs)`/`find_package(vision_msgs)` CMake errors.
**Phase to address:** Phase 1 — this is part of D-09's `run.sh build` target.

### Pitfall 4: DDS/RMW env vars only apply if the process actually inherits the container's shell environment

**What goes wrong:** If the RAI experiment process is launched in a way that bypasses the container's environment block (e.g. a bespoke `docker exec` that doesn't carry `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`), it silently lands on domain 0 with the default RMW and discovers nothing — no error.
**Why it happens:** These are process-environment settings, not something `rai_core`/`rclpy` infers. `ROS2Context.__enter__` calls `rclpy.init(domain_id=self.domain_id)` with `domain_id=None` by default — confirmed by reading `context.py` directly this session — which falls back to reading `ROS_DOMAIN_ID` from the process environment. If that variable isn't set in the shell the RAI process runs in, `rclpy` silently uses domain 0.
**How to avoid:** `run.sh agent`/`run.sh up` must invoke the RAI process via `docker exec` against the already-running `wojtek_robot` container (which already has these three env vars set at the container level per `ros/docker/compose.yaml`), not a separate `docker run`.
**Warning signs:** `ROS2Connector.get_topics_names_and_types()` returns an empty or drastically short list even though `ros2 topic list` in another shell into the same container shows topics.
**Phase to address:** Phase 1 — covered by D-01/D-02's "runs inside the existing `wojtek_robot` container" decision; the finding here is *why* that decision matters mechanically (env var inheritance, not just DDS config file sharing).

### Pitfall 5: `config.toml` load is cwd-relative, not experiment-root-relative

**What goes wrong:** `load_config(config_path=None)` opens `"config.toml"` relative to whatever the process's current working directory happens to be at call time — confirmed by reading `model_initialization.py` directly this session. The container's shell default cwd is `/ros2_ws`, not `experiments/wojtek_rai_v1/`.
**Why it happens:** RAI's own examples assume you're always running from the directory containing `config.toml`; nothing in the library enforces or even warns about this.
**How to avoid:** `run.sh` must `cd` into `experiments/wojtek_rai_v1/` (or the venv-invoking wrapper must) before calling any RAI code that reads config, or explicitly pass `config_path=` to `load_config()`/`get_llm_model()` calls in this experiment's own code.
**Warning signs:** `FileNotFoundError: [Errno 2] No such file or directory: 'config.toml'`, or — more dangerous — a silent read of an unrelated `config.toml` if one happens to exist elsewhere on the path.
**Phase to address:** Phase 1 — this is where `run.sh`'s invocation contract is established for every later phase.

### Pitfall 6: Secrets leakage into `config.toml` (repeated from project-level research, phase-specific mechanism)

**What goes wrong:** RAI's vendor dispatch reads API keys for its various backends via each LangChain integration's own env-var convention, not from `config.toml` itself — confirmed by reading `model_initialization.py` directly this session: `ChatOpenAI`/`ChatBedrock`/`ChatOllama`/`ChatGoogleGenerativeAI` each pull credentials through their own SDK's default env-var chain (`OPENAI_API_KEY`, AWS's standard credential chain, none for Ollama, `GOOGLE_API_KEY`), and Langfuse/LangSmith tracing separately read `LANGFUSE_PUBLIC_KEY`/`LANGFUSE_SECRET_KEY`/`LANGCHAIN_API_KEY` — all confirmed exact names by grepping the source directly. `config.toml` itself has no key fields at all in the upstream template (verified below), so there is no field to accidentally fill with a real key if the committed template is copied verbatim and the actual keys go through `.env` as D-11/D-12 require.
**Why it happens:** A quickstart copy-paste habit is the usual trap; not applicable here since the upstream template genuinely has no key fields.
**How to avoid:** Extend `.env.example` with the exact names above; write a test asserting the committed `config.toml` and `.env.example` contain no value matching a secret-shape regex (see Code Examples) — this is FOUND-04's literal requirement.
**Warning signs:** Any committed file under the experiment directory containing a string matching `sk-`, `AKIA`, a JWT-shaped string (`ey[A-Za-z0-9_-]{10,}\.`), or a bare non-empty value in `config.toml`'s vendor blocks that isn't a public model name.
**Phase to address:** Phase 1.

### Pitfall 7: Isolation violations — read Common Pitfalls in `.planning/research/PITFALLS.md` Pitfall 10 for the general shape

This project-level finding (imports leaking either direction between `experiments/` and `ros/`/`training/`) applies unchanged to this phase; it is not repeated in full here, but the concrete test pattern to enforce it (a grep-based guard test over `ros/`, `training/`, and the rsync path list) is in Code Examples.

### Pitfall 8: Test strategy — model-free coverage vs. a suite that quietly needs a live key

Also a repeated project-level finding (`.planning/research/PITFALLS.md` Pitfall 12), directly gating FOUND-06. The phase-specific instance: since no agent logic exists yet, Phase 1's tests are entirely infra-shaped (config loading, secret-scan, import-boundary, `pyproject.toml`/`uv.lock` consistency) — none of them need `rclpy`, an LLM key, or a GPU, so there is no live/manual split to design yet; that split becomes relevant starting Phase 2.

## Code Examples

### Verified upstream `config.toml` template (commit to the experiment as-is, no keys)

```toml
# Source: raw.githubusercontent.com/RobotecAI/rai/2.12.0/config.toml — fetched and verified this session
[vendor]
simple_model = "openai"
complex_model = "openai"
embeddings_model = "openai"

[aws]
simple_model = "anthropic.claude-3-haiku-20240307-v1:0"
complex_model = "anthropic.claude-3-5-sonnet-20240620-v1:0"
embeddings_model = "amazon.titan-embed-text-v1"
region_name = "us-east-1"

[openai]
simple_model = "gpt-4o-mini"
complex_model = "gpt-4o"
embeddings_model = "text-embedding-ada-002"
base_url = "https://api.openai.com/v1/"

[ollama]
simple_model = "llama3.2"
complex_model = "llama3.1:70b"
embeddings_model = "llama3.2"
base_url = "http://localhost:11434"

[google]
simple_model = "gemini-3-flash"
complex_model = "gemini-3-pro"
embeddings_model = "text-embedding-004"

[tracing]
project = "rai"

[tracing.langfuse]
use_langfuse = false
host = "http://localhost:3000"

[tracing.langsmith]
use_langsmith = false
host = "https://api.smith.langchain.com"
```

Note: the upstream template also has `[asr]`/`[tts]` sections (voice, out of scope this milestone) — safe to drop them from the committed file since `load_config()` only requires `[vendor]` (every other section falls back to a default per the dataclass defaults read directly in `model_initialization.py`).

### Env vars to add to root `.env.example` (D-11), exact names verified from RAI source

```bash
# Model vendor keys — read natively by each LangChain integration, never by RAI's config.toml.
# Only fill in the ones the chosen vendor (config.toml [vendor] block) actually needs.
OPENAI_API_KEY=
# AWS Bedrock vendor: standard AWS SDK chain (boto3) — AWS_ACCESS_KEY_ID / AWS_SECRET_ACCESS_KEY /
# AWS_SESSION_TOKEN, or an AWS profile. [ASSUMED: standard boto3 credential chain, not RAI-specific]
AWS_ACCESS_KEY_ID=
AWS_SECRET_ACCESS_KEY=
AWS_SESSION_TOKEN=
GOOGLE_API_KEY=
# Langfuse tracing (off by default; use_langfuse=false in config.toml) — verified exact names in
# rai/initialization/model_initialization.py this session
LANGFUSE_PUBLIC_KEY=
LANGFUSE_SECRET_KEY=
```

### Secret-scan test, modeled on `training/tests/unit/test_job_scripts.py`'s existing pattern

```python
# experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py
# Pattern source: training/tests/unit/test_job_scripts.py (FORBIDDEN_RE / SITE_VALUE_RE), read this
# session — adapted here for API-key shapes rather than scheduler/host identity.
import re
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]

# Common cloud API key / token shapes. Extend as new vendors are added (HRI-03).
SECRET_SHAPE_RE = re.compile(
    r"sk-[A-Za-z0-9]{16,}"          # OpenAI-style
    r"|AKIA[0-9A-Z]{16}"            # AWS access key id
    r"|AIza[0-9A-Za-z_-]{35}"       # Google API key
    r"|ey[A-Za-z0-9_-]{10,}\."      # JWT-shaped
    r"|(pk|sk)_(live|test)_[A-Za-z0-9]{16,}"
)


def tracked_config_files():
    return [HERE / "config.toml"]


def test_config_toml_has_no_secret_shaped_values():
    for path in tracked_config_files():
        text = path.read_text()
        hits = SECRET_SHAPE_RE.findall(text)
        assert not hits, f"{path.name} contains a secret-shaped value: {hits}"


def test_config_toml_vendor_blocks_have_no_populated_key_field():
    # Belt-and-suspenders: config.toml's schema (per RAI upstream) has no key/token
    # fields at all — assert none were added.
    text = (HERE / "config.toml").read_text()
    assert not re.search(r"(?i)\b(api_key|api_token|secret)\s*=\s*[\"'][^\"']+[\"']", text)
```

### Isolation guard test, adapted from the sibling experiment's stated rule

```python
# experiments/wojtek_rai_v1/tests/test_isolation_boundary.py
# Enforces FOUND-01: nothing outside this directory imports it, and ros/deploy.sh's
# rsync ros/src/ + --packages-up-to wojtek_bringup path cannot reach it.
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
EXPERIMENT_NAME = EXPERIMENT_DIR.name  # "wojtek_rai_v1"


def test_ros_src_and_training_do_not_import_the_experiment():
    hits = subprocess.run(
        ["grep", "-rl", "-E", rf"\b{EXPERIMENT_NAME}\b", "ros/src", "training"],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    assert hits.stdout.strip() == "", f"import boundary violated:\n{hits.stdout}"


def test_experiment_ros_packages_live_outside_ros_src():
    assert not (REPO_ROOT / "ros" / "src" / EXPERIMENT_NAME).exists()
    assert not list((REPO_ROOT / "ros" / "src").glob(f"*{EXPERIMENT_NAME}*"))
```

### `run.sh`'s ROS-aware Python invocation wrapper (Pattern 2 + Pattern 4 combined)

```bash
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
HERE="$PWD"

run_py() {
  # Combines Pattern 2 (system-site-packages venv needs sourced ROS setup) and
  # Pattern 4 (config.toml load is cwd-relative) in one place.
  source /opt/ros/jazzy/setup.bash
  [ -f "$HERE/ros_ws/install/setup.bash" ] && source "$HERE/ros_ws/install/setup.bash"
  cd "$HERE"
  exec "$HERE/.venv/bin/python" "$@"
}
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|-------------------|---------------|--------|
| RAI on the `langchain==0.x` line (tags `2.11.2 (langchain 0.x)` and earlier) | RAI on `langchain>=1.0.0,<2.0.0` starting at tag `2.12.0` | `2.12.0` release (this project's exact pin) | Pinning `rai-core==2.12.0` starts on the correct, current line from day one; no upgrade-path dead end |
| Poetry-based RAI monorepo builds | `uv`-based builds (`uv_build` backend in `rai_core`'s own `[build-system]`, confirmed this session) | RAI 2.0 | Consistent with this project's own choice of `uv` (D-06) — same tooling upstream and downstream |
| `pip install rai-core[all]`-style kitchen-sink installs shown in some walkthrough docs | Minimal, explicit `rai-core==2.12.0` + `rai-whoami==0.0.5` only | This project's own decision, not an upstream change | Avoids the `openset` extras group (`torch`/`torchvision`/Grounding DINO/SAM-2, multi-GB) entirely — confirmed absent from `rai_core`'s own base dependency list read this session |

**Deprecated/outdated:**
- Any RAI tag before `2.12.0` — do not follow examples referencing the pre-1.0 LangChain line.
- RAI's own default Dockerfile (`osrf/ros:jazzy-desktop-full`, `rmw_fastrtps_cpp`) — irrelevant here; this project's container is deliberately minimal (`ros:jazzy-ros-core`) with CycloneDDS, and RAI has no code-level RMW dependency (its ROS 2 tools are built on plain `rclpy`).

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|-----------------|
| A1 | The `wojtek_robot` container's `python3` is exactly 3.12 (not independently confirmed inside a running container this session; inferred from `ros:jazzy-ros-core`'s Ubuntu 24.04 base and this repo's stated dev convention) | Standard Stack | If actually a different 3.1x, `rai-core`/`rai-whoami`'s `requires-python = ">=3.10,<3.13"` still likely admits 3.10/3.11, but the venv-creation command (`uv venv --python python3.12`) would need adjusting to whatever's actually present |
| A2 | AWS Bedrock vendor credentials follow the standard boto3 chain (`AWS_ACCESS_KEY_ID`/`AWS_SECRET_ACCESS_KEY`/`AWS_SESSION_TOKEN` or a profile) — not independently verified against `ChatBedrock`'s exact credential-resolution code this session, only inferred from it being a standard `langchain_aws.ChatBedrock` instantiation | Code Examples (.env.example additions) | If `ChatBedrock` needs an additional/different env var, the `.env.example` template under-specifies it — low risk since AWS vendor isn't the concrete default choice this phase (HRI-03 defers vendor choice) |
| A3 | `uv`'s current version on the research host (0.9.18) is representative of what `run.sh` will install into the container — the exact version `run.sh` pins (or "latest") is Claude's own discretion per D-06, not fixed by this research | Standard Stack | Low risk; `uv`'s CLI surface used here (`venv --system-site-packages`, `sync`, `lock`) has been stable across recent releases |

**If this table is empty:** N/A — three low-risk assumptions logged above; none require user confirmation before planning proceeds (all are either easily verified at execution time inside the actual container, or don't affect this phase's locked decisions).

## Open Questions

1. **Exact `rai_interfaces` commit SHA to pin**
   - What we know: latest published tag is `0.3.0` at commit `2398f1f3e4c96d790365492294599439a38cdf9a` (verified this session); upstream `rai_core@2.12.0` itself pins to unhelpful `main`.
   - What's unclear: whether the `0.3.0` tag's message definitions (`HRIMessage`, `AudioMessage`, `RAIDetectionArray`) are actually the ones `rai_core@2.12.0`'s Python code expects field-for-field — no explicit compatibility statement found upstream.
   - Recommendation: pin `0.3.0`'s commit as the starting point (D-09/D-14's discretion already anticipates resolving this at execution time); if `run.sh build`/`test` surfaces a field mismatch, fall back to `main`'s HEAD at execution time and record the commit actually used in the `.repos` file and the lockfile commit message, as D-14 specifies.

2. **Does the topic-discovery success criterion need the full `ROS2TopicsToolkit` (with its `cv_bridge` import), or is a narrower `ROS2Connector.get_topics_names_and_types()` call sufficient?**
   - What we know: `ROS2Connector` itself (the class, not the LangChain tool wrapper) does not import `cv_bridge` — only `rai/tools/ros2/generic/topics.py`'s tool classes do.
   - What's unclear: whether FOUND-03's "lists the sim's live topics" criterion is meant to exercise RAI's actual `BaseTool` machinery (which later phases will use for real) or just prove ROS 2 discovery works through RAI's connector layer.
   - Recommendation: install `cv_bridge` anyway (Pitfall 2's fix is one `apt-get install`) and exercise the real toolkit import — it's needed by Phase 4's vision work regardless, so proving it now is strictly cheaper than deferring the dependency-install pain to a later phase.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|--------------|-----------|---------|----------|
| Docker | Container lifecycle (D-01/D-02) | ✓ (research host) | 29.0.4 | — |
| `uv` | Env/lockfile management (D-06) | ✓ (research host, for reference only — actual install target is inside the container per D-06) | 0.9.18 | Static binary download if `pip install uv` unavailable |
| `colcon` | `rai_interfaces` build (D-09) | ✗ (research host) — expected: only present *inside* the ROS-sourced container, confirmed already installed there via `ros-dev-tools` in `ros/docker/Dockerfile` | — | None needed; already present in the target environment (the container), just not on this research host |
| `ros-jazzy-cv-bridge` | RAI's stock ROS 2 tool import (Pitfall 2) | ✗ (not in `ros/docker/Dockerfile`, confirmed by reading it this session) | — | Install via `apt-get` inside the container at `run.sh install`/`build` time (not baked into the image) |
| aarch64 remote GPU dev box | FOUND-07 multi-arch verification | Not probed this session (D-14 authorizes GSD executors to use it; access is over the operator's local SSH alias, not something to verify from a research pass) | — | — |

**Missing dependencies with no fallback:** none — every gap identified (`ros-jazzy-cv-bridge`, `rai_interfaces`' rosdep deps) has a straightforward runtime-install fallback already documented in Common Pitfalls and Code Examples.

**Missing dependencies with fallback:** `ros-jazzy-cv-bridge` and `rai_interfaces`' rosdep-resolved packages (`vision_msgs`, `nav2_msgs`, `nav2_simple_commander`, `tf_transformations`, `portaudio19-dev`) — all installed at `run.sh install`/`build` time against the running container, per Pitfalls 2 and 3.

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest `>=8.0.0` (repo convention, `[VERIFIED: training/pyproject.toml]`) |
| Config file | none yet — Wave 0 gap: this experiment needs its own `pytest` invocation inside `run.sh test`, same shape as the sibling experiment's `run.sh test` (`"$PY" -m pytest tests -q`) |
| Quick run command | `./experiments/wojtek_rai_v1/run.sh test` |
| Full suite command | same — this phase has no slow/integration split yet (no ROS runtime, no LLM key needed for anything in Phase 1's scope) |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|---------------------|---------------|
| FOUND-01 | Nothing outside the experiment imports it; `ros/src`/`training` clean | unit (grep-based guard) | `pytest experiments/wojtek_rai_v1/tests/test_isolation_boundary.py -x` | ❌ Wave 0 |
| FOUND-02 | `pyproject.toml`/`uv.lock` declare the exact pinned versions | unit | `pytest experiments/wojtek_rai_v1/tests/test_pinned_versions.py -x` | ❌ Wave 0 |
| FOUND-03 | Compose override + `run.sh agent-topics` structurally correct (no live sim needed for the unit-test layer; the live discovery itself is a manual/VERIFICATION.md check per D-16) | unit (config presence/shape) + manual (live topic list, both machines) | `pytest experiments/wojtek_rai_v1/tests/test_compose_override.py -x` | ❌ Wave 0 |
| FOUND-04 | No secret-shaped values in tracked config | unit | `pytest experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py -x` | ❌ Wave 0 |
| FOUND-06 | `run.sh test` needs no LLM key/ROS runtime/GPU | unit (self-referential — the guard test from `training/tests/unit/test_job_scripts.py`'s pattern, adapted, could assert no test file imports `rclpy`/instantiates a live model) | `pytest experiments/wojtek_rai_v1/tests -q` (green with `LLM key=unset`, no ROS runtime, no GPU) | ❌ Wave 0 |
| FOUND-07 | Same three commands succeed on both machines | manual (VERIFICATION.md, per D-16 — not automatable from a single CI run without dev-box access) | `run.sh install && run.sh test && run.sh agent-topics` on each machine | N/A — manual per D-16 |

### Sampling Rate
- **Per task commit:** `./experiments/wojtek_rai_v1/run.sh test`
- **Per wave merge:** same (no slow suite this phase)
- **Phase gate:** Full suite green on the laptop, then re-run on the aarch64 dev box per D-16/D-17 before `/gsd-verify-work`

### Wave 0 Gaps
- [ ] `experiments/wojtek_rai_v1/tests/test_isolation_boundary.py` — covers FOUND-01
- [ ] `experiments/wojtek_rai_v1/tests/test_pinned_versions.py` — covers FOUND-02
- [ ] `experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py` — covers FOUND-04
- [ ] `experiments/wojtek_rai_v1/tests/test_compose_override.py` — covers FOUND-03's static-config half
- [ ] `experiments/wojtek_rai_v1/run.sh` itself, plus `pyproject.toml`/`uv.lock` — framework install: no test framework exists in this new directory yet; `pytest` must be added as this experiment's own direct dev dependency (D-07)

## Security Domain

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|-----------------|---------|---------------------|
| V2 Authentication | No | No user-facing authentication surface this phase (no agent, no chat interface yet) |
| V3 Session Management | No | Same — Phase 2+ concern |
| V4 Access Control | No | Same |
| V5 Input Validation | No | No external/user input parsed this phase — config files are developer-authored, not runtime input |
| V6 Cryptography | No | No cryptographic operations this phase |
| Secrets management (ASVS V6.4-equivalent / general secure-configuration practice) | **Yes** | Env-var-only secrets, sourced from gitignored `.env`, matching `ros/deploy.sh`'s existing `UBUNTU_PRO_TOKEN` pattern; enforced by the secret-scan test (FOUND-04) |
| Supply-chain / dependency integrity | **Yes** | Pinned exact versions + committed lockfile (`uv.lock`) for every transitive dependency, specifically to close the "unconstrained LangChain/LangGraph" gap identified in Common Pitfalls #1 |

### Known Threat Patterns for this stack

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|-------------------------|
| Secret committed to tracked config (`config.toml`, `.env.example`) in a public repo | Information Disclosure | Env-var-only secrets (D-11/D-12); automated secret-shape test (FOUND-04, Code Examples) |
| Supply-chain drift: an unpinned transitive dependency resolves to a compromised/broken version on a later install | Tampering | `uv.lock` committed and treated as the actual pin (Pitfall 1); `uv sync` from the lockfile, not a fresh resolve, for routine installs |
| Experiment code reachable by `ros/deploy.sh`, ending up on the physical robot unintentionally | Elevation of Privilege (of scope — experiment code gains robot-deployment reach it was never meant to have) | ROS packages built outside `ros/src/`; isolation guard test (FOUND-01, Code Examples) |

## Sources

### Primary (HIGH confidence)
- `https://pypi.org/pypi/rai-core/json`, `https://pypi.org/pypi/rai-whoami/json`, `https://pypi.org/pypi/faiss-cpu/json`, `https://pypi.org/pypi/opencv-python-headless/json` — fetched and parsed directly this session
- `https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/src/rai_core/pyproject.toml`, `.../src/rai_whoami/pyproject.toml`, `.../config.toml`, `.../ros_deps.repos`, `.../src/rai_core/rai/initialization/model_initialization.py`, `.../src/rai_core/rai/tools/ros2/generic/topics.py`, `.../src/rai_core/rai/communication/ros2/__init__.py`, `.../src/rai_core/rai/communication/ros2/context.py`, `.../src/rai_core/rai/communication/ros2/connectors/base.py` — fetched and read directly this session, at the exact pinned tag `2.12.0`
- `https://api.github.com/repos/RobotecAI/rai/tags`, `https://api.github.com/repos/RobotecAI/rai_interfaces/tags`, `https://api.github.com/repos/RobotecAI/rai_interfaces/branches` — fetched directly this session
- `https://raw.githubusercontent.com/RobotecAI/rai_interfaces/0.3.0/package.xml` — fetched directly this session
- `https://api.github.com/repos/astral-sh/uv/releases/latest`, `https://raw.githubusercontent.com/astral-sh/uv/main/crates/uv-cli/src/lib.rs`, `https://api.github.com/repos/astral-sh/uv/issues/1483` — fetched directly this session
- This repository, read directly this session: `ros/docker/compose.yaml`, `ros/docker/Dockerfile`, `ros/sim.sh`, `ros/dev.sh`, `ros/deploy.sh`, `.env.example` (via `git show HEAD:.env.example`), `.gitignore`, `training/pyproject.toml`, `training/tests/unit/test_job_scripts.py`, `experiments/autonomous_architecture_ros2_v1/README.md`, `experiments/autonomous_architecture_ros2_v1/run.sh`, `ros/src/wojtek_perception_bringup/package.xml` (grep, confirmed no `cv_bridge` dependency), `.planning/codebase/TESTING.md`, `.planning/codebase/STRUCTURE.md`, `.planning/config.json`
- `.planning/research/STACK.md`, `.planning/research/PITFALLS.md`, `.planning/research/ARCHITECTURE.md`, `.planning/research/FEATURES.md`, `.planning/research/SUMMARY.md` — project-level research, read in full this session; this document builds on it rather than repeating it, and corrects/refines two specifics (`langchain-core`'s actual verified range at the pinned tag; the `cv_bridge`/`rosdep` gaps not previously surfaced)

### Secondary (MEDIUM confidence)
- `[CITED: docs.langchain.com/oss/python/release-policy]` — `langchain-community`'s semver-exemption claim
- WebSearch results corroborating `uv venv --system-site-packages`'s existence and behavior (cross-checked against the primary source read directly, above)

### Tertiary (LOW confidence)
- A1 in the Assumptions Log (container's exact Python version) — not independently verified inside a running container this session

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — every version claim re-verified this session against PyPI JSON or the exact pinned GitHub tag, not training-data recall
- Architecture: HIGH — RAI's connector/tool/config-loading behavior read directly from source at the pinned tag; this repo's own container/compose/deploy mechanics read directly
- Pitfalls: HIGH for the three new findings this session surfaced (`cv_bridge`, `rai_interfaces` rosdep deps, `uv --system-site-packages` + cwd-relative config load) — each backed by a direct source read; MEDIUM-HIGH for the pitfalls carried over from `.planning/research/PITFALLS.md` (already well-sourced there)

**Research date:** 2026-09-07
**Valid until:** ~30 days for the pinned versions and this repo's own config (stable); re-verify `rai_interfaces`' commit pin and the exact aarch64 wheel availability if this phase's execution slips past that window, since upstream RAI releases on a roughly monthly cadence
