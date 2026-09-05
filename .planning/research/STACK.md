# Stack Research

**Domain:** RAI (RobotecAI) embodied-AI agent framework, integrated as a self-contained `experiments/` add-on to a ROS 2 Jazzy quadruped (Wojtek)
**Researched:** 2026-09-05
**Confidence:** HIGH (all package/version/API claims verified directly against pinned-tag source files, PyPI JSON metadata, and GitHub Releases/Tags API — not training-data recall)

## Recommended Stack

### Core Technologies

| Technology | Version | Purpose | Why Recommended |
|------------|---------|---------|-----------------|
| `rai_core` (PyPI: `rai-core`) | **2.12.0** pinned (GitHub tag exists; PyPI has gone on to 2.12.1 with no matching tag) | Core RAI framework: agents, ROS 2 connectors/tools, model-vendor dispatch, tracing | Latest tagged release, first release on the "langchain 1.x" line (`2.12.0 (langchain 1.x)`), current as of research date; matches project's "pin a release" constraint |
| `rai_whoami` (PyPI: `rai_whoami`) | **0.0.5** on PyPI (in-repo `pyproject.toml` at tag 2.12.0 says `0.1.0` — version-number drift, not a known functional gap; API checked, matches) | Builds the "whoami" embodiment package (identity, capabilities, FAISS vector DB over robot docs) that RAI agents load as system context | Exactly the tool named in the milestone ("rai whoami embodiment") |
| `rai_interfaces` (separate repo `RobotecAI/rai_interfaces`, ROS 2 package) | Pin a commit SHA, not `main` (see Version Compatibility) | ROS 2 message/service/action definitions RAI's ROS 2 connectors depend on (e.g. `rai_interfaces/msg/HRIMessage`) | Not a PyPI package — it is a `colcon`-buildable ROS package; needed as a small workspace overlay regardless of how `rai_core` itself is installed |
| Python | **3.12** | Interpreter for the RAI environment | `rai_core`/`rai_whoami` require `>=3.10,<3.13`; ROS 2 Jazzy's Ubuntu 24.04 base ships Python 3.12 by default, and Wojtek's own training stack already standardizes dev on 3.12 — no version drift inside the container |
| `langchain` | `>=1.0.0,<2.0.0` (pulled in transitively) | Agent/tool-calling substrate RAI is built on | Hard pin in `rai_core`'s own `pyproject.toml`; do not force an older `langchain==0.x` alongside it |
| `langchain-core` | `>=1.2.10,<2.0.0` | LangChain primitives | Transitive, exact floor verified in `rai_core` deps |
| `langgraph` | `>=1.1.5,<2.0.0` (+ `langgraph-prebuilt`) | Agent graph runtime under RAI's `create_conversational_agent` and friends | Transitive, exact floor verified in `rai_core` deps |
| `uv` | latest (`astral-sh/uv`) | Package/environment manager for building or developing against the RAI monorepo | RAI's own root `pyproject.toml` is a `uv` workspace (`[tool.uv.sources]` path-editable installs of `rai_core`, `rai_whoami`, etc.); RAI's own Dockerfile installs `uv` via `COPY --from=ghcr.io/astral-sh/uv:latest`. Not required if you only `pip install rai-core rai-whoami` from PyPI (see Installation) |
| `colcon` | whatever ships with ROS 2 Jazzy | Builds the `rai_interfaces` ROS package only | Wojtek's dev container already has `colcon`; no new build tool needed |

### Supporting Libraries

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `langchain-openai` | transitive dep of `rai_core` | OpenAI (and any OpenAI-compatible endpoint via `base_url`, e.g. **vLLM**, OpenRouter) chat + embeddings | Always installed with `rai_core`; this is also the path for a self-hosted vLLM backend, not a separate "vllm vendor" |
| `langchain-aws` | transitive dep of `rai_core` | `ChatBedrock` — the *only* way RAI reaches Anthropic Claude models natively | Needed only if you choose AWS Bedrock as a vendor |
| `langchain-ollama` | transitive dep of `rai_core` | Local Ollama chat + embeddings | Needed for the on-Jetson / fully-local backend path called out in `PROJECT.md` |
| `langchain-google-genai` | transitive dep of `rai_core` | Gemini chat + embeddings | Optional fourth vendor |
| `langchain-community` | transitive dep | Misc LangChain community integrations `rai_core` reaches into | Pulled automatically, no action needed |
| `faiss-cpu` | transitive dep of `rai_whoami` | Vector store backing the whoami document/image retrieval tool | Always needed for `rai whoami`; CPU-only build is correct — there is no GPU requirement here and none should be added |
| `streamlit` | `>=1.44,<2.0.0` (transitive dep of `rai_core`) | Web-based text chat HMI ("Hello RAI" pattern) | This is the text-chat interface named in the milestone — see `rai.frontend.run_streamlit_app` below |
| `opencv-python` / `opencv-python-headless` | transitive dep of `rai_core` | Image decode/encode inside ROS 2 image tools (`GetROS2ImageTool`) | Needed for the "look" requirement (camera → description) |
| `transforms3d` | transitive dep of `rai_core` | Quaternion/Euler math in ROS 2 tools | Pulled automatically |
| `pillow` | transitive dep, both packages | Image handling for whoami + multimodal messages | Pulled automatically |
| `tomli` / `tomli-w` | transitive dep of `rai_core` | Reads/writes `config.toml` (the vendor config file) | Pulled automatically |
| `langfuse` | `>=3.8.0,<4.0.0`, disabled by default (`use_langfuse = false`) | Optional LLM tracing | **Leave disabled.** Enabling it means shipping a Langfuse host/key — avoid given the repo's no-secrets, public-repo rule unless there's a real need |
| `nav2_msgs`, `tf_transformations` (ROS 2 apt packages) | ROS 2 Jazzy versions | Only needed if you use RAI's built-in `Nav2Toolkit` (`rai.tools.ros2.navigation.nav2`), which drives `nav2_msgs/action/NavigateToPose` | **Not needed for this milestone.** Wojtek has no Nav2 stack; navigation goes through the generic `PublishROS2MessageTool`/SCAN-Planner path instead (see Stack Patterns below), so skip this dependency entirely |

### Development Tools

| Tool | Purpose | Notes |
|------|---------|-------|
| `uv sync --locked` | Reproduce RAI's own dev environment if you ever need to run its example scripts/tests as reference | Only relevant if working *inside* a clone of `RobotecAI/rai`, not inside the Wojtek experiment itself |
| `rai-config-init` (console script from `rai_core`) | Interactively scaffolds a `config.toml` with vendor blocks | Run once after `pip install rai-core`; then hand-edit for the chosen vendor(s) |
| `rosdep` | Resolves `rai_interfaces`'/`rai_bringup`'s ROS package dependencies before `colcon build` | Same tool already used by Wojtek's `ros/` build |
| `vcs` (`vcstool`) | Imports `rai_interfaces` from `ros_deps.repos`-style YAML into a workspace `src/` | Small, one-shot use just for `rai_interfaces`; do not `vcs import` the full `demos.repos`/`ros_deps.repos` from the RAI monorepo, which also pulls sim/manipulation assets out of scope for this milestone |

## Installation

The scope is `rai_core` + ROS 2 tools + `rai_whoami` + text HMI only (per `PROJECT.md`, explicitly excluding `rai_perception`, `rai_nomad`, `rai_bench`, `rai_sim`, `rai_finetune`, and speech/`rai_s2s`). That scope is fully covered by **two PyPI packages** plus **one small ROS message package** — there is no need to clone or `colcon build` the ~9-package RAI monorepo.

```bash
# Inside the experiment's own venv (Python 3.12), isolated per repo isolation rules
python3.12 -m venv .venv
source .venv/bin/activate

# Core packages — pin exact versions for reproducibility
pip install "rai-core==2.12.0" "rai-whoami==0.0.5"

# Scaffold config.toml, then hand-edit vendor blocks (see Model Vendor Configuration)
rai-config-init

# ROS 2 message definitions RAI's connectors need (not on PyPI — see rationale above)
mkdir -p ros_ws/src
vcs import ros_ws/src <<'EOF'
repositories:
  rai_interfaces:
    type: git
    url: https://github.com/RobotecAI/rai_interfaces.git
    version: <pin a commit SHA here, verified against rai_core 2.12.0 — see Version Compatibility>
EOF
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros_ws/src --ignore-src -r -y
colcon build --symlink-install --base-paths ros_ws
source ros_ws/install/setup.bash
```

Do **not** also `pip install rai_bringup` or build it — it only contains launch files for RAI's own manipulation/O3DE demos. Wojtek already has its own launch surface (`ros/sim.sh`); write one small launch file in the experiment instead of reusing `rai_bringup`.

## Alternatives Considered

| Recommended | Alternative | When to Use Alternative |
|-------------|-------------|--------------------------|
| `pip install rai-core rai-whoami` (pinned) | Git submodule of the full `RobotecAI/rai` monorepo + `uv sync` + `colcon build` everything | Only if the project later adopts `rai_perception`, `rai_sim`, `rai_bench`, or `rai_s2s` — those are not on PyPI at all and require the source checkout. Out of scope this milestone per `PROJECT.md`; adopting the whole monorepo now would drag in O3DE sim bridges, benchmark suites, and voice code that violate `experiments/` isolation's "nothing extra" spirit and this repo's YAGNI convention |
| Small standalone `colcon` workspace for `rai_interfaces` only | `sudo apt install ros-jazzy-rai-interfaces` | Try apt first — RAI's own install docs mention this as optional. As of research date it could not be confirmed present on the ROS Jazzy buildfarm index (ambiguous signal); fall back to the `vcs`+`colcon` recipe above if `apt-cache search ros-jazzy-rai-interfaces` comes up empty |
| AWS Bedrock vendor for Claude models | Direct Anthropic API (`langchain-anthropic`) | RAI's `get_llm_model()` vendor dispatch (`src/rai_core/rai/initialization/model_initialization.py`) only implements `openai`, `aws`, `ollama`, `google` — there is no `anthropic` branch and no `langchain-anthropic` dependency anywhere in `rai_core`. If a direct Anthropic key (not Bedrock) is required, RAI's dispatch needs a small local patch/wrapper — this is a real gap, not a config oversight |
| Ollama vendor for local/on-Jetson inference | Self-hosted vLLM via the `[openai]` vendor block + custom `base_url` | vLLM is not a separate RAI vendor; it is reached by pointing the `openai` vendor's `base_url` at the vLLM OpenAI-compatible server (same pattern the community uses for OpenRouter, confirmed in `RobotecAI/rai#755`) |
| RAI's generic `PublishROS2MessageTool` for `cmd_vel` | RAI's built-in `Nav2Toolkit` (`rai.tools.ros2.navigation.nav2`) | `Nav2Toolkit` hard-depends on `nav2_msgs/action/NavigateToPose` and `tf_transformations` — i.e. a running Nav2 stack. Wojtek has no Nav2 stack and uses SCAN-Planner + direct `cmd_vel`; use it only if/when Wojtek adopts Nav2 |

## What NOT to Use

| Avoid | Why | Use Instead |
|-------|-----|--------------|
| `langchain==0.x` / pre-1.0 RAI releases (tag `2.11.2 (langchain 0.x)` or earlier) | RAI's `2.12.0` is the first release on the langchain-1.x line; mixing an older RAI tag with this project's other Python 1.x-era LangChain expectations, or pinning `rai_core<2.11` today, buys a dead-end upgrade path | Pin `rai-core==2.12.0` (langchain 1.x line) from the start |
| The full `RobotecAI/rai` monorepo as a git submodule for this milestone | Pulls `rai_sim` (O3DE), `rai_bench`, `rai_s2s` (voice — explicitly out of scope), `rai_finetune`, `rai_perception` — none of which this milestone needs, and all of which add build/dependency surface inside an `experiments/` directory that's supposed to stay self-contained and minimal | `pip install rai-core rai-whoami` (see Installation) |
| Assuming an `[anthropic]` vendor section in `config.toml` | Does not exist; `get_llm_model()` raises `ValueError: Unknown LLM vendor` for any vendor string other than `openai`, `aws`, `ollama`, `google` | Route Claude models through the `[aws]` vendor (Bedrock model IDs like `anthropic.claude-3-5-sonnet-...`), or extend RAI's dispatch locally |
| RAI's own `docker/Dockerfile` base image (`osrf/ros:jazzy-desktop-full`) or its `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` default, copied verbatim into this project | Wojtek's dev container is deliberately `ros:jazzy-ros-core` (minimal) with **CycloneDDS** (`rmw_cyclonedds_cpp`) everywhere — in `ros/docker/compose.yaml`, `ros/deploy/rpi/install.sh`, and `wojtek_bringup`. RAI has no code-level dependency on FastDDS; its Dockerfile just defaults to it for its own GUI/O3DE demos | Keep `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` inside the experiment's container layer so the RAI agent shares the same DDS domain/RMW as the rest of the running sim stack; do not introduce a second RMW implementation into the same ROS graph |
| Enabling Langfuse/LangSmith tracing (`use_langfuse`/`use_langsmith = true`) by default | Both need a cloud host + key; this is a public repo with a hard no-secrets rule, and tracing isn't part of the milestone's core value | Leave both `false` in `config.toml`; if tracing is wanted later, wire the key through `.env` like `ANTHROPIC_API_KEY`/`UBUNTU_PRO_TOKEN` already are |
| `rai_bench`, `rai_sim`, `rai_perception`, `rai_nomad`, `rai_finetune`, `rai_s2s` | All explicitly out of scope in `PROJECT.md`; none are on PyPI, all require the full monorepo/source build | Skip entirely for this milestone |

## Stack Patterns by Variant

**If the model backend is cloud (OpenAI or Claude-via-Bedrock):**
- Use the `[openai]` or `[aws]` vendor block in `config.toml`.
- `OPENAI_API_KEY` or `AWS_ACCESS_KEY_ID`/`AWS_SECRET_ACCESS_KEY`/`AWS_SESSION_TOKEN` come from `.env`, matching the `UBUNTU_PRO_TOKEN`/`WOJTEK_AP_PSK` pattern already used in `ros/deploy.sh`.
- Because agent compute today is a dev/GPU box with internet (per `PROJECT.md`), this path has no extra infrastructure need beyond the key.

**If the model backend is local (Ollama, on the eventual Jetson):**
- Use the `[ollama]` vendor block with `base_url` pointed at a locally-running `ollama serve`.
- Set the whoami build's `embeddings_model` to the `ollama` vendor too — RAI's own docs specifically recommend this to avoid per-build embedding costs against a cloud API when iterating on the whoami document set.
- No API key needed; matches the "RPi has no internet, Jetson may also be offline" constraint in `PROJECT.md`.

**If a self-hosted OpenAI-compatible server is preferred (vLLM):**
- Reuse the `[openai]` vendor block, just override `base_url` to the vLLM server's `/v1` endpoint. Do not look for a `[vllm]` section — none exists.

**If the text HMI is the Streamlit "Hello RAI" pattern:**
- Mirror RAI's own `examples/debugging_assistant.py`: `rai.frontend.run_streamlit_app` + `rai.agents.langchain.core.create_conversational_agent` + `rai.tools.ros2.ROS2CLIToolkit` (or a custom toolkit built from the generic tools) + `rai_whoami.EmbodimentInfo.from_file(...)` as the system prompt.
- This is the shortest path to the milestone's "human interacts through RAI's text chat interface" requirement — it is a ~40-line script, not a service to stand up separately.

**If a CLI-only text interface is preferred over Streamlit:**
- `create_conversational_agent(...)` returns a LangGraph-compiled agent; drive it directly from a plain Python REPL loop instead of `run_streamlit_app` — same agent construction, no `streamlit` process to manage. Trades away the web UI's image-rendering convenience for the "what do you see?" camera tool (Streamlit renders returned images inline; a bare CLI loop will only print text unless you add your own image display).

## Version Compatibility

| Package A | Compatible With | Notes |
|-----------|------------------|-------|
| `rai-core==2.12.0` | `python>=3.10,<3.13` | Matches ROS 2 Jazzy's Ubuntu 24.04 default Python 3.12; use 3.12 to match Wojtek's existing dev convention |
| `rai-core==2.12.0` | `langchain>=1.0,<2.0`, `langchain-core>=1.2.10,<2.0`, `langgraph>=1.1.5,<2.0` | Exact floors pulled from `rai_core`'s own `pyproject.toml`/PyPI metadata at this version — do not pin an older LangChain alongside it |
| `rai-whoami` | PyPI shows `0.0.5`; the monorepo's in-tree `pyproject.toml` at tag `2.12.0` shows `0.1.0` | Version-number mismatch between the published wheel and the source tree at the matching `rai_core` tag. The published `0.0.5` wheel was inspected directly and does export the API the current docs/examples use (`EmbodimentInfo`, `EmbodimentSource`, `QueryDatabaseTool`, `build_whoami` CLI) — treat as safe, but re-verify import behavior once installed rather than assuming parity with the 2.12.0-tagged source tree |
| `rai_interfaces` (message repo) | RAI's own `ros_deps.repos` at tag `2.12.0` pins it to branch `main`, not a commit/tag | Upstream does not give you a reproducible pin here. Latest published tag at research date is `0.3.0`; either use that tag or pin a specific commit SHA of `main`, and validate message field compatibility (e.g. `HRIMessage`) against what `rai_core` 2.12.0's connectors expect before relying on it |
| ROS 2 Jazzy + CycloneDDS | RAI's ROS 2 connectors (`rai.communication.ros2`) | No FastDDS-specific code found in `rai_core`; connectors are built on plain `rclpy`. Keep `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` as already set everywhere else in Wojtek's stack |
| ROS 2 distro support | Humble and Jazzy both documented | Do not target Kilted/Rolling or Foxy/Galactic-era distros — not part of RAI's documented support matrix |

## Sources

- https://github.com/RobotecAI/rai — repo root, license, package layout (Apache-2.0) — direct source inspection, HIGH confidence
- https://github.com/RobotecAI/rai/releases and `api.github.com/repos/RobotecAI/rai/releases` — release/tag history, dates verified via GitHub API — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/pyproject.toml, `.../src/rai_core/pyproject.toml`, `.../src/rai_whoami/pyproject.toml` — exact dependency pins at the recommended tag — HIGH confidence
- https://pypi.org/pypi/rai_core/json and https://pypi.org/pypi/rai_whoami/json — published version history, `requires_dist`, `requires_python` — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/config.toml — real default vendor config file (openai/aws/ollama/google, no anthropic) — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/src/rai_core/rai/initialization/model_initialization.py — vendor dispatch code, confirms no native Anthropic vendor — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/examples/debugging_assistant.py — canonical minimal text-HMI + whoami + ROS 2 tools pattern — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/src/rai_core/rai/tools/ros2/{simple,generic/topics,navigation/nav2}.py — ROS 2 tool surface (`PublishROS2MessageTool`, `GetROS2ImageConfiguredTool`, `Nav2Toolkit`) — HIGH confidence
- https://raw.githubusercontent.com/RobotecAI/rai/2.12.0/docker/Dockerfile, `.../ros_deps.repos` — upstream Docker/base-image and `rai_interfaces` pinning behavior — HIGH confidence
- https://robotecai.github.io/rai/setup/install/, `.../setup/setup_docker/`, `.../setup/vendors/`, `.../tutorials/create_robots_whoami/` — official docs, fetched and summarized — MEDIUM confidence (summarized by an auxiliary fetch tool; cross-checked against source where it mattered)
- https://github.com/RobotecAI/rai/discussions/755 — maintainer-confirmed OpenAI-compatible `base_url` pattern for OpenRouter/vLLM — MEDIUM confidence (community discussion, not docs, but backed by a merged PR referenced in-thread)
- https://github.com/RobotecAI/rai_interfaces — separate ROS interfaces repo, tags/license checked directly — HIGH confidence
- https://index.ros.org/p/rai_interfaces/ — ambiguous on Jazzy apt/binary availability — LOW confidence, flagged as a gap to verify at execution time
- `/home/bukareszt/Downloads/robodog/w01-tek/ros/docker/Dockerfile`, `ros/docker/compose.yaml`, `ros/deploy/rpi/install.sh` — existing Wojtek base image and CycloneDDS configuration, read directly for the compatibility note — HIGH confidence

---
*Stack research for: RAI integration into Wojtek's ROS 2 Jazzy quadruped stack*
*Researched: 2026-09-05*
