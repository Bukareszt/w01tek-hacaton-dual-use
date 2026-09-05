# Research Summary: RAI Integration onto Wojtek's ROS 2 Stack

**Project:** Wojtek RAI Integration (w01-tek, adding a RAI LLM agent framework as a text-driven control layer for the existing MuJoCo simulation and quadruped)
**Domain:** Embodied-AI agent framework (RAI/RobotecAI) integrated as an isolated `experiments/` overlay onto an existing ROS 2 Jazzy stack with RL locomotion policy
**Researched:** 2026-09-05
**Confidence:** MEDIUM-HIGH (framework APIs and version constraints verified directly against source; this repo's config is HIGH; RAI's own docs are thin on execution internals, flagging several pitfalls as design-time, not installation-time)

## Executive Summary

RAI (RobotecAI) is a mature, battle-tested Python agent framework built on LangChain for embodied-AI robots. This integration brings RAI into Wojtek's stack as an isolated, self-contained `experiments/` layer — a Python agent process (not a ROS node) that talks to the existing ROS 2 graph through standard topics and services, commanding motion via `cmd_vel`, reading camera frames for scene understanding, and using SCAN-Planner's collision-aware navigation logic to turn high-level goals into safe, observed motion. The target for this milestone is *simulation first* — proving that a text instruction ("walk forward", "go to the chair", "what do you see?") reaches all the way through the agent, tools, ROS topics, and down to simulated Wojtek's 50 Hz RL policy.

The recommended approach mirrors the existing `autonomous_architecture_ros2_v1` experiment's proven isolation and testing patterns: scaffold a lightweight Python venv (Python 3.12, matching Jazzy's default), pin all dependencies immediately (RAI's own `rai_core` leaves LangChain/LangGraph unpinned — a known version-drift trap that must be locked on first successful install), build the whoami embodiment from Wojtek's URDF plus carefully-authored prose docs about capabilities, and stand up the agent inside the existing `ros/sim.sh` development container environment so it inherits the correct ROS domain ID, RMW implementation (CycloneDDS), and multicast configuration.

The single highest-severity pitfall is **unbounded `cmd_vel` commands**. The LLM has no ground-truth knowledge of the RL policy's training envelope; this must be solved *before* Phase 3 (walking) with a ROS-level velocity clamp/timeout arbiter (`twist_mux`) sitting between the agent's `/wojtek/agent/cmd_vel` topic and the real `/cmd_vel`. Similarly, navigation must route all goal commands through SCAN-Planner (not straight-line `cmd_vel` sequences) — this repo already measured 100% collision failure on straight-line execution and proved SCAN-Planner fixes it to 90%+ success. Three other early traps: **dependency version skew** (unpinned LangChain/LangGraph), **DDS/QoS silent message drops** on camera topics, and **CycloneDDS domain-mismatch discovery failures**.

## Key Findings

### Recommended Stack

**Core technologies:**
- **`rai-core==2.12.0`** — RAI agent framework (ReAct loop, ROS 2 connectors, tool registry). First langchain-1.x release. **Critical:** This version leaves LangChain/LangGraph unconstrained; lock immediately after install.
- **`rai-whoami==0.0.5`** — Embodiment-builder tool (builds whoami from URDF + prose docs).
- **`rai_interfaces`** (separate ROS package) — ROS 2 message definitions. Build as colcon overlay; **pin a commit SHA**.
- **Python 3.12** — Matches ROS 2 Jazzy default and Wojtek's environment.
- **`langchain` 1.x stack, `streamlit`, `faiss-cpu`** — Transitive deps and HMI. Pin all versions immediately after install.

**What NOT to install:**
- Full RAI monorepo, extras groups (`openset`/`simbench`/`nomad`/`s2s`), `langchain==0.x`

### Expected Features

**Must have (P1 — table stakes):**
- Whoami embodiment (identity + capabilities from URDF + prose)
- Text HRI channel (`/from_human`/`/to_human`)
- Velocity tool (cmd_vel with clamp/timeout wrapper)
- Goal navigation tool (through SCAN-Planner, not straight-line)
- Image/VLM description tool
- ReAct conversational agent
- Swappable model backend

**Should have (P2):**
- Langfuse tracing
- Multi-agent split (Conversational + StateBased)

**Defer (v2+):**
- Voice, Nav2 full stack, `rai_perception`, `rai_nomad`, tricks/skills

### Architecture Approach

RAI is a Python agent process (not a ROS node) owning a ROS 2 connector, managing a LangChain ReAct loop, and delegating to Tools. EmbodimentInfo (whoami) is a build-time artifact loaded once at startup as system prompt.

**Major components:**
1. ReActAgent — Reasoning loop (LLM interprets text, selects tool, interprets results)
2. EmbodimentInfo — Static identity + capabilities from URDF + prose
3. ROS2Connector — Owns rclpy node, all ROS interaction
4. Tools — Scoped with readable/writable/forbidden allowlists (first safety layer)
5. Velocity clamp/timeout wrapper — ROS arbiter (`twist_mux`) between agent and `/cmd_vel` (second safety layer)
6. Local planner — SCAN-Planner or Nav2 (sits between "where" and "how")

### Critical Pitfalls (Top 5)

1. **Version skew** — Fresh `pip install -U` pulls incompatible LangChain/LangGraph. Prevention: Lock all four versions immediately; treat any bump as deliberate.

2. **Unbounded cmd_vel** — LLM publishes velocity outside policy envelope or keeps commanding after stopping. Prevention: Wrap tool with clamp/timeout; don't publish to `/cmd_vel` directly.

3. **QoS silent message drop** — BEST_EFFORT camera silently refuses to deliver to RELIABLE subscriber. Prevention: Explicitly set `qos_profile_sensor_data` on image tool.

4. **DDS domain mismatch** — Agent lands on domain 0 if not run in correct environment. Prevention: Run in dev container or export `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

5. **Navigation hallucination + collisions** — Agent claims arrival from memory; straight-line hits furniture. Prevention: Route through SCAN-Planner; make "arrived" a tool that returns real pose.

## Implications for Roadmap

### Phase 1: Environment Setup & RAI Installation
- Isolated venv, lock dependencies immediately, build whoami embodiment
- Model-free tests: embodiment build, capability-boundary Q&A, import guard
- Research flag: Exact LangChain/LangGraph versions from fresh install

### Phase 2: Text Chat Interface (No Robot)
- ReAct agent + Streamlit HMI, standalone (no ROS)
- Model-free tests: agent loop with mock tools, vendor dispatch
- Complexity: MEDIUM

### Phase 3: Velocity Arbiter + Walking
- Deploy `twist_mux` clamp/timeout between agent and `/cmd_vel` (Pitfall 5 mitigation)
- Wire velocity tool, validate "walk forward" in sim
- Model-free tests: clamp logic, timeout behavior
- Complexity: MEDIUM-HIGH
- Research flag: Max vx/vy/wz from training config

### Phase 4: Goal Navigation (SCAN-Planner or Nav2)
- HIGHEST RISK: SCAN-Planner is sim-only/ROS-less today; needs wrapping
- Navigate-to tool, validated in cluttered scenes (reusing scan-planner.md benchmarks)
- Complexity: HIGH
- Research flag: **CRITICAL SPIKE** — Decide SCAN-Planner wrapping vs Nav2 stack before Phase 4 planning

### Phase 5: Image Tool + VLM Description
- `GetROS2ImageTool` with explicit QoS override, frame freshness check, compression
- "What do you see?" validated with test objects
- Can run parallel with Phase 4
- Complexity: MEDIUM

### Phase Ordering
1. Phase 1 first (foundation for all others)
2. Phase 2 follows Phase 1 (validates agent loop without robot risk)
3. Phase 3 must come before walking (safety layer must exist first)
4. Phase 4 after Phase 3 proven (large, needs research spike on SCAN-Planner vs Nav2)
5. Phase 5 parallel with Phase 4 or after (independent tool)

### Research Flags

**Needs research:**
- Phase 1: Exact LangChain/LangGraph versions from `rai_core==2.12.0`
- Phase 3: Velocity envelope (max vx/vy/wz) from `training/docs/configuration.md`
- Phase 4: SCAN-Planner ROS wrapping vs Nav2 decision (schedule, Jetson portability, quadruped tuning)

**Standard patterns (skip research):**
- Phase 2: LangChain ReAct and Streamlit documented in RAI examples
- Phase 5: This repo's `training/wojtek_rl/` proved VLM-backed scene description works

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | Versions and APIs verified against RAI sources, PyPI, this repo's ROS 2 config. Unpinned LangChain/LangGraph clearly documented. |
| Features | MEDIUM | RAI docs thin on internals; corroborated across multiple sources. Safety-mechanism specifics (velocity limits, timeouts) are design-time decisions. |
| Architecture | MEDIUM-HIGH | RAI's component model (Agent ≠ node, ReAct, tool-scoping) verified. Wojtek-side (twist_mux, SCAN-Planner, isolation) HIGH. Integration (two-layer safety) is sound but not proven in this project yet. |
| Pitfalls | MEDIUM-HIGH | Anchored to real evidence: unpinned LangChain confirmed; rclpy/QoS/domain issues are known ROS 2 patterns; cmd_vel safety from general LLM architecture; navigation hallucination from this repo's VLM-nav research. |

**Overall:** MEDIUM-HIGH

### Gaps to Address

1. **SCAN-Planner ROS wrapping feasibility** — Spike engineering effort, Jetson portability during Phase 1 planning.
2. **Exact LangChain/LangGraph versions** — Install + capture in Phase 1; commit `uv.lock`.
3. **Velocity envelope bounds** — Extract from `training/docs/configuration.md` during Phase 3 planning.
4. **`rai_interfaces` pin point** — Check `rai_core` 2.12.0's own `ros_deps.repos` for tested commit.

## Sources

**PRIMARY (HIGH):**
- RAI GitHub (pyproject.toml, vendor dispatch, tools, examples): https://github.com/RobotecAI/rai
- PyPI JSON (versions, requires_dist): https://pypi.org/pypi/rai_core/json, https://pypi.org/pypi/rai_whoami/json
- This repo: ros/docker/, wojtek_perception_bringup (QoS), training/docs/scan-planner.md (collision rates), experiments/autonomous_architecture_ros2_v1/ (proven patterns)

**SECONDARY (MEDIUM):**
- RAI docs: https://robotecai.github.io/rai/
- RAI arXiv paper: "RAI: Flexible Agent Framework for Embodied AI" (https://arxiv.org/html/2505.07532v1)
- GitHub issues: langchain-ai/langgraph#6363 (unpinned breaking change); RobotecAI/rai#755 (OpenAI-compatible base_url)

---

*Research completed: 2026-09-05*
*Ready for roadmap: yes*
*Roadmapper should expect Phase 1 spike on LangChain versions + Phase 4 SCAN-Planner decision before final roadmap commit.*
