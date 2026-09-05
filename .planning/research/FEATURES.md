# Feature Research

**Domain:** RAI-based / ROS 2 LLM-agent robot control (text-driven quadruped agent in sim)
**Researched:** 2026-09-05
**Confidence:** MEDIUM

RAI's own documentation is thin on internals (many pages summarize rather than
enumerate), so this synthesizes the RAI docs site
(https://robotecai.github.io/rai/), the RAI GitHub repo and README
(https://github.com/RobotecAI/rai), the RAI arXiv paper "RAI: Flexible Agent
Framework for Embodied AI" (https://arxiv.org/html/2505.07532v1), the
rosbot_xl navigation demo (https://github.com/RobotecAI/rai-rosbot-xl-demo),
and the agentic-mobile-manipulator demo
(https://github.com/RobotecAI/agentic-mobile-manipulator). Confidence is
MEDIUM overall: framework-level claims (tool base classes, whoami, HRI topic
names, tracing config keys) are HIGH confidence (seen directly in docs/README
excerpts); safety-mechanism internals (velocity limits, timeouts, stuck
detection specifics) are LOW confidence — the public docs largely don't
enumerate them, so PITFALLS.md and this file flag them as build-yourself.

## Feature Landscape

### Table Stakes (Users Expect These)

Features a text-driven LLM-agent robot demo is assumed to have. Missing any
of these makes the "chat with Wojtek" milestone feel broken or unsafe, even
in sim.

| Feature | Why Expected | Complexity | Notes |
|---------|--------------|------------|-------|
| Text HRI channel (`/from_human`, `/to_human`, `rai_interfaces/msg/HRIMessages`) | Every RAI demo (voice or text) round-trips through this topic pair; it's the framework's one interaction primitive | LOW | RAI's voice tutorial documents this exact topic/type pair for speech; text-only skips ASR/TTS and lets a human type directly to `/from_human` and read `/to_human`, or a CLI wraps the same topics. Source: https://robotecai.github.io/rai/tutorials/voice_interface/ |
| ROS 2 tool set (`GetROS2ImageTool`, `GetROS2TopicsNamesAndTypesTool`, `PublishROS2MessageTool`, `GetROS2LidarDataTool`, `CallROS2Service`/`ReceiveROS2Message`) built on `BaseROS2Tool` | This is the entire mechanism by which the LLM touches the robot; without it the agent can only talk | MEDIUM | `BaseROS2Tool` supports `readable`/`writable`/`forbidden` topic-and-service allowlists — this is also the main safety knob (see Pitfalls). Sources: https://robotecai.github.io/rai/tutorials/tools/, https://arxiv.org/html/2505.07532v1 |
| `rai whoami` embodiment (identity + constitution + capabilities + docs → optimized system prompt, optional vector DB via FAISS) | Every RAI robot in the docs/demos has one; it's what makes the agent answer "who are you" / "what can you do" correctly and grounds tool selection | MEDIUM | Built from a directory of images, docs, and URDF: `python src/rai_whoami/rai_whoami/build_whoami.py <dir>/ --build-vector-db`. PROJECT.md already commits to this. Source: https://robotecai.github.io/rai/tutorials/walkthrough/ |
| Velocity-command tool → `cmd_vel` | This milestone's first capability ("walk forward"); it's the most direct and lowest-risk tool to expose | LOW | Thin wrapper: LLM extracts (vx, vy, wz) or a named gait command, publishes to the existing `cmd_vel` topic the RL policy already consumes. No new ROS interfaces needed — matches the "no changes to production topics" constraint. |
| `navigate_to`-style tool with obstacle avoidance | RAI's flagship rosbot_xl demo is exactly this: "natural language commands → nav2 goals," obstacle-aware | HIGH | Two implementation paths exist (see Differentiators/Dependencies): RAI's own Nav2 integration, or wrapping the repo's own SCAN-Planner behind a tool with the same interface contract. Either way this is a *tool*, not a new agent capability — the agent just calls it and reports success/failure. Source: https://github.com/RobotecAI/rai-rosbot-xl-demo |
| Image/vision tool (`GetROS2ImageTool`, optionally `Get360ImageTool`) + VLM description | RAI's manipulation and navigation demos both use camera tools to ground the LLM in what the robot currently sees | MEDIUM | Needs a VLM-capable backend (RAI's docs distinguish a "complex model" for reasoning and a "simple model" for image analysis — image description can run on the cheaper/simpler model). Source: https://robotecai.github.io/rai/tutorials/tools/ |
| ReAct tool-calling loop (single conversational agent) | This is RAI's baseline agent pattern ("Conversational... based on ReAct LLM architecture") and the minimum needed to route free text to the right tool | MEDIUM | Standard LangChain ReAct: think → call tool → observe → repeat until final answer. RAI's own README frames the whole framework around this loop. Source: https://arxiv.org/html/2505.07532v1 |
| Vendor-agnostic model backend config (cloud vs. local, "complex" vs. "simple" model roles) | PROJECT.md defers the concrete choice but requires swappability; RAI is explicitly built for this ("vendor agnostic agentic framework") | LOW | RAI's vendor setup already documents OpenAI, AWS Bedrock (Claude), and Ollama (local/self-hosted) side by side with a Configurator. Source: https://robotecai.github.io/rai/setup/vendors/ (fetched digest) |
| Task decomposition for compound instructions | "Go to the chair and tell me what's there" needs the agent to sequence navigate → look → describe, not treat it as one tool call | MEDIUM | Emergent from the ReAct loop + tool set above, not a separate feature — but it must be validated with multi-step prompts, since RAI's own paper flags "error handling and mission success detection were inconsistent" as an open issue. |

### Differentiators (Competitive Advantage / Research Value)

Not required for the milestone to "work," but where this integration proves
something interesting relative to the generic RAI demos, and where it can
reuse work this repo already has.

| Feature | Value Proposition | Complexity | Notes |
|---------|-------------------|------------|-------|
| SCAN-Planner behind the `navigate_to` tool instead of Nav2 | This repo already has a validated, quadruped-appropriate local planner (100% success / near-zero collisions on room+apartment suites per `training/docs/scan-planner.md`) that solves exactly the failure mode generic VLM/LLM navigation hits — straight-line commands walking into furniture. Wiring the agent's `navigate_to` tool to `ScanExecutor`'s existing submit/active/blocked contract is reuse, not new research. | MEDIUM (integration) | Nav2 is the RAI-standard path (wheeled rosbot_xl demo) and assumes a full costmap/AMCL stack; SCAN-Planner is a lighter, quadruped-tuned alternative that already exists sim-only in `training/wojtek_rl/scan/`. Bringing it into the ROS-facing experiment (currently sim-only, "no ROS node, no odometry model" per its own doc) is the actual differentiating engineering work this milestone can deliver. Depends on: a ROS-side odometry/pose source (ground truth in sim is fine per PROJECT.md scope). |
| Multi-agent split: Conversational agent (HRI + tool routing) separate from a state-based execution agent | RAI's own paper describes exactly this pattern for navigation deployments: "Conversational Agent... responsible for HRI" paired with "Robot Control Agent... focused solely on execution of the mission," reducing the chance that a long-running nav/walk task blocks or confuses the chat loop | MEDIUM–HIGH | RAI ships two pre-packaged agent types for this: **Conversational** (ReAct-based, for HRI/tool invocation) and **StateBased** (finite-state-machine agent mixing LLM reasoning with procedural steps, for long-running missions). Adopting StateBased for navigation avoids re-inventing "is the walk still happening" bookkeeping in the ReAct loop. Source: https://arxiv.org/html/2505.07532v1 |
| Observability via Langfuse or LangSmith tracing | RAI has first-class config for both (`[tracing.langfuse]` / `[tracing.langsmith]` toggles + host URL), turning every tool call, prompt, and completion into an inspectable trace — valuable for debugging a robot that occasionally "walks into a chair" or mis-parses "go to the kitchen" | LOW (config only) | Langfuse is open-source and self-hostable (fits the "no private infrastructure identity committed" and eventual on-Jetson/offline constraints better than a cloud-only LangSmith account). Recommend Langfuse as the default; LangSmith only if the team is already LangChain-invested. Source: RAI vendor/tracing setup docs (fetched digest); Langfuse project itself at https://github.com/langfuse/langfuse |
| Cross-comparison with `experiments/autonomous_architecture_ros2_v1` | That experiment already has a router + Bielik + Qwen-VL agent, ASR/TTS, and a `wojtek_agent` tool layer with its own goal FSM, spatial map, and trace module (`wojtek_agent/trace.py`) — running RAI side by side gives a real evaluation of "battle-tested framework" vs. "custom, Polish-voice-tuned agent" on the same robot | N/A (comparison, not a build) | PROJECT.md already scopes this as "side by side, not replacing." The differentiator is *doing the comparison*, not merging — that's explicitly out of scope this milestone. |
| Local/on-device model path validated in sim, ready for Jetson | The agentic-mobile-manipulator demo already demonstrates exactly the target end-state: local inference (llama.cpp/Vulkan, NPU-offloaded safety VLM) mirrored between sim and hardware for a mobile robot, explicitly to enable "seamless simulation-to-hardware transition" | HIGH (deferred) | Not required this milestone (model backend choice is a separate task per PROJECT.md), but worth noting as the credible precedent that supports the "backend must be swappable, Jetson eventually" constraint rather than treating it as speculative. Source: https://github.com/RobotecAI/agentic-mobile-manipulator |

### Anti-Features (Commonly Requested, Often Problematic — Deliberately NOT v1)

| Feature | Why Requested | Why Problematic | Alternative |
|---------|---------------|------------------|-------------|
| Voice (ASR/TTS) through RAI this milestone | RAI ships `rai_asr`/`rai_tts` and a full voice tutorial; "just turn it on" looks cheap since it's built-in | Doubles the moving parts under test (turn-taking via `/voice_commands` pause/play/stop, VAD, wake word) for a milestone whose value is proven by text alone; the existing `autonomous_architecture_ros2_v1` experiment already has verified voice — duplicating it here adds no new information | Text chat via `/from_human`/`/to_human` only, per PROJECT.md's explicit scope. Revisit voice once RAI's walk/nav/look loop is proven and a merge/comparison decision is made. |
| Nav2 full stack (costmap, AMCL localization, planner server) as the navigation backend | It's RAI's own documented/demoed path (rosbot_xl) and looks like "the standard way" | Nav2 assumes a wheeled differential/holonomic base with a mature costmap+localization pipeline; retrofitting that onto a quadruped whose actual locomotion is an RL policy over `(vx,vy,wz)` duplicates work this repo already solved with SCAN-Planner, and pulls in `rai_nomad`/full SLAM scope PROJECT.md explicitly excludes | Wrap SCAN-Planner's existing `MidLevelExecutor` (submit/active/blocked contract) behind a `navigate_to` RAI tool; treat Nav2 as a future option only if SCAN-Planner proves insufficient. |
| `rai_perception` open-set object detection (GroundingDINO/Grounded-SAM2) for "go to the chair" | RAI's manipulation/navigation demos use it, and it looks necessary to resolve "the chair" to a location | Adds a second vision model, a detection pipeline, and new failure modes (missed/false detections) before the core walk→navigate→look loop is even validated; PROJECT.md explicitly excludes `rai_perception` this milestone | The VLM image tool + LLM reasoning over the described scene can approximate "go toward the chair" for a demo; true object-grounded navigation is deferred, consistent with PROJECT.md's Out of Scope list. |
| `rai_nomad` (NoMaD visual navigation) | It's RAI's purpose-built vision-based navigation extension, so it seems like the "correct" upgrade path for goal navigation | Adds a learned visual-navigation policy layered on top of an already-nontrivial stack (RL locomotion + SCAN-Planner + LLM agent); no evidence it's needed for the target scenes (single room/apartment scale) that SCAN-Planner already handles at 100% success | Defer to when goal navigation needs true open-set, mapless, long-range routing — PROJECT.md already flags this as a later trigger. |
| Skill/trick tools (getup, jump, D-pad show tricks) as agent tools | They already exist as gamepad tricks and are the "fun" tools to expose to an LLM demo | Expands the safety surface (uncontrolled joint-level tricks triggered by an LLM's tool call) and the whoami/capability description work, for zero contribution to the milestone's actual value (walk, navigate, look) | Keep tricks gamepad-only. Revisit only after walk/nav/look and a confirmation-of-dangerous-actions pattern both exist. |
| A generic "confirm every action" HITL gate on all tool calls | Feels like the responsible default for a robot-controlling LLM | RAI's own docs and demos don't describe a built-in confirmation layer, and blanket confirmation kills the "type and watch it walk" experience that is this milestone's actual point, especially in a MuJoCo sim with no physical risk | Scope confirmation (if any) to a short allowlist of genuinely risky actions (none currently exist in sim scope — walk/navigate/look are all reversible/low-risk); rely on `BaseROS2Tool`'s `readable`/`writable`/`forbidden` allowlist as the real safety boundary instead of a chat-level "are you sure?" |
| Building a custom safety-stop / velocity-clamp / timeout layer from scratch, duplicating what a well-configured `BaseROS2Tool` + bounded `navigate_to` timeout already gives | RAI's public docs don't spell out velocity limits or timeouts as a distinct subsystem, which invites over-engineering one | The framework's actual safety primitive is topic/service allowlisting (`forbidden` list) plus whatever bounds the wrapped tool (e.g., SCAN-Planner already has "did not move for 1.6 s" / "moved 8 s without progress" give-up conditions per `training/docs/scan-planner.md`) — reinventing a parallel safety layer is redundant and untested | Reuse SCAN-Planner's existing stuck/give-up thresholds for `navigate_to`; keep the velocity-tool bounded to the RL policy's already-tuned command range; treat `BaseROS2Tool`'s allowlist as the ROS-level safety boundary. |

## Feature Dependencies

```
rai whoami (identity/constitution/capabilities)
    └──requires──> URDF + docs directory for Wojtek (exists: wojtek.xml, docs/)

Text HRI channel (/from_human, /to_human)
    └──enables──> Conversational (ReAct) agent loop
                       └──requires──> rai whoami (system prompt)
                       └──requires──> ROS 2 tool set registered with the agent

cmd_vel velocity tool
    └──requires──> BaseROS2Tool readable/writable config scoped to cmd_vel only
    (no dependency on navigate_to or image tool — can ship first, standalone)

navigate_to tool
    └──requires──> a local planner: SCAN-Planner (reused) OR Nav2 (new stack)
    └──requires──> odometry/pose source on the ROS side (ground truth in sim is acceptable)
    └──enhances──> task decomposition ("go to X and describe it" chains navigate_to + image tool)

Image/vision tool + VLM description
    └──requires──> "simple model" vision-capable backend (may differ from the "complex" reasoning model)
    └──independent of──> navigate_to (can ship before or in parallel)

Task decomposition (multi-step instructions)
    └──requires──> cmd_vel tool + navigate_to tool + image tool all registered
    └──requires──> ReAct loop (or StateBased agent for long-running nav)

Multi-agent split (Conversational + StateBased)
    └──enhances──> navigate_to reliability (isolates long-running nav from chat loop)
    └──conflicts with──> "ship the simplest possible v1" — treat as v1.x, not v1

Observability (Langfuse/LangSmith tracing)
    └──independent of all agent features──> pure config, add anytime, recommend early for debugging
```

### Dependency Notes

- **Conversational agent requires `rai whoami` and the ROS tool set:** the
  ReAct loop's system prompt comes from whoami, and it has nothing to act on
  without tools registered against `BaseROS2Tool`. Both must exist before
  end-to-end "type and it walks" is possible.
- **`navigate_to` requires a planner choice up front:** SCAN-Planner (reuse,
  sim-only today, needs a thin ROS-facing wrapper + a pose source) vs. Nav2
  (RAI's demoed path, but a new stack for a quadruped with no existing
  costmap/localization setup). This is the single highest-leverage decision
  in the roadmap — recommend SCAN-Planner given it's already validated
  against this exact failure mode ("VLM says forward 1.5, robot hits
  furniture") that generic Nav2+LLM integrations don't specifically solve.
- **Task decomposition is not a separate build item** — it emerges once
  cmd_vel, navigate_to, and image tools are all present and the ReAct loop
  can chain them. It only needs explicit *testing*, not new code.
- **Multi-agent split conflicts with minimal v1 scope:** it's a genuine
  RAI-documented pattern and solves a real problem (long nav blocking chat),
  but adds an agent-orchestration layer PROJECT.md's "minimum for
  walk/nav/look" scope doesn't require yet. Treat as the natural v1.x
  upgrade once a single ReAct agent's navigate_to calls prove too slow/blocking
  for the chat loop to feel responsive.
- **Observability has no dependencies and no blockers** — enabling
  Langfuse tracing early (before walk/nav/look are even debugged) costs only
  config and pays for itself the first time the agent picks the wrong tool.

## MVP Definition

### Launch With (v1)

Minimum to satisfy PROJECT.md's "Active" requirements.

- [ ] `rai whoami` built from Wojtek's URDF/MuJoCo model + docs — required for
      the agent to identify as Wojtek and ground tool selection
- [ ] Text HRI channel (`/from_human` / `/to_human`) as the sole interaction
      surface — no voice
- [ ] Velocity-command tool (`cmd_vel`) with `BaseROS2Tool` allowlisted to
      that one topic — lowest-risk, highest-value first capability
- [ ] `navigate_to` tool wrapping SCAN-Planner's existing
      `MidLevelExecutor` contract, with its existing stuck/give-up thresholds
      as the only safety bound — reuse, not new research
- [ ] Image tool (`GetROS2ImageTool` or equivalent against the sim camera
      topic) + VLM scene description for "what do you see?"
- [ ] Single Conversational (ReAct) agent tying the three tools together —
      task decomposition validated by testing multi-step prompts, not built
      separately
- [ ] Model backend swappable via RAI's existing vendor config (concrete
      choice deferred per PROJECT.md, but the plumbing must not hard-code one
      vendor)

### Add After Validation (v1.x)

- [ ] Langfuse tracing enabled — add as soon as walk/nav/look debugging gets
      painful; zero coupling to other features, so there's no reason to wait
      long
- [ ] Multi-agent split (Conversational + StateBased execution agent) — add
      once `navigate_to` calls are observed to block or lag the chat loop
- [ ] Confirmation-of-dangerous-action pattern — add only if/when a genuinely
      risky tool (e.g., real-robot deployment, high-velocity commands) is
      introduced; nothing in sim-scope v1 needs it

### Future Consideration (v2+)

- [ ] Voice (ASR/TTS) through RAI — defer until a merge/comparison decision
      is made against `autonomous_architecture_ros2_v1`'s existing voice work
- [ ] `rai_perception` open-set detection for object-grounded navigation
      ("go to the chair" resolved to a real detected chair, not VLM guess)
- [ ] `rai_nomad` visual navigation — only if goal navigation needs long-range,
      mapless routing beyond what SCAN-Planner's local window handles
- [ ] Nav2 as an alternative/fallback navigate_to backend — only if
      SCAN-Planner proves insufficient for a scenario it wasn't designed for
- [ ] Real-robot (Jetson) deployment of the RAI agent — explicit stretch goal,
      next milestone

## Feature Prioritization Matrix

| Feature | User Value | Implementation Cost | Priority |
|---------|------------|---------------------|----------|
| `rai whoami` embodiment | HIGH | MEDIUM | P1 |
| Text HRI channel | HIGH | LOW | P1 |
| `cmd_vel` velocity tool | HIGH | LOW | P1 |
| `navigate_to` (SCAN-Planner-backed) | HIGH | HIGH | P1 |
| Image tool + VLM description | HIGH | MEDIUM | P1 |
| Conversational ReAct agent loop | HIGH | MEDIUM | P1 |
| Swappable model backend config | MEDIUM | LOW | P1 |
| Langfuse/LangSmith tracing | MEDIUM | LOW | P2 |
| Multi-agent split (Conversational + StateBased) | MEDIUM | HIGH | P2 |
| Confirmation-of-dangerous-actions gate | LOW (sim, no physical risk) | LOW | P3 |
| Voice through RAI | LOW (duplicates existing experiment) | HIGH | P3 |
| `rai_perception` open-set detection | MEDIUM | HIGH | P3 |
| `rai_nomad` visual navigation | LOW (no current scenario needs it) | HIGH | P3 |
| Nav2 as navigate_to backend | LOW (SCAN-Planner already fits) | HIGH | P3 |

**Priority key:**
- P1: Must have for this milestone
- P2: Should have, add once v1 is proven
- P3: Explicitly out of scope this milestone (per PROJECT.md), revisit later

## Competitor / Reference-Implementation Feature Analysis

| Feature | RAI rosbot_xl demo (wheeled, Nav2) | RAI agentic-mobile-manipulator demo (arm + base, on-device) | Wojtek approach |
|---------|--------------------------------------|-------------------------------------------------------------|------------------|
| Locomotion command | Direct velocity/nav2 goals to a differential-drive base | Mobile base + arm, on-device VLM+LLM planning | `cmd_vel` → existing RL joystick policy (locomotion is already solved and decoupled; the agent never touches gait) |
| Obstacle-aware navigation | Full Nav2 stack (costmap, AMCL) | Not primary focus (warehouse, largely mapped) | SCAN-Planner local planner behind `navigate_to`, reusing this repo's validated collision-avoidance work instead of standing up Nav2 for a quadruped |
| Vision grounding | Camera tool + LLM description for HRI | Dedicated on-device VLM (LFM2-VL) for perception, plus a separate safety VLM on NPU | Single VLM-capable "simple model" role via `GetROS2ImageTool`, no dedicated safety VLM in v1 (sim-only, lower stakes) |
| Model deployment | Not the focus of that demo | Explicit on-device (llama.cpp/Vulkan/NPU) with cloud-offload option, sim=hardware-in-the-loop | v1 targets sim with a swappable backend; on-device/Jetson is explicitly the next milestone, mirroring this demo's precedent |
| Observability | Not documented in that demo | Not documented in that demo | Enable Langfuse from the start (framework-level feature both demos could use but don't document) — a genuine differentiator for this integration |

## Sources

- RAI GitHub repository (overview, package list, demos): https://github.com/RobotecAI/rai
- RAI documentation site: https://robotecai.github.io/rai/
- RAI tools tutorial (`BaseROS2Tool`, `GetROS2ImageTool`, etc.): https://robotecai.github.io/rai/tutorials/tools/
- RAI complete walkthrough (`rai whoami`, custom agent, HRI topics): https://robotecai.github.io/rai/tutorials/walkthrough/
- RAI voice interface tutorial (`/from_human`, `/to_human`, `/voice_commands`): https://robotecai.github.io/rai/tutorials/voice_interface/
- RAI vendor/tracing setup (model backends, Langfuse/LangSmith config): https://robotecai.github.io/rai/setup/vendors/
- RAI arXiv paper, "RAI: Flexible Agent Framework for Embodied AI" (multi-agent patterns, tool set, whoami internals): https://arxiv.org/html/2505.07532v1
- RAI rosbot_xl navigation demo (Nav2 integration precedent): https://github.com/RobotecAI/rai-rosbot-xl-demo
- RAI agentic-mobile-manipulator demo (on-device model precedent): https://github.com/RobotecAI/agentic-mobile-manipulator
- RAI Bench documentation (task success/scoring approach): https://robotecai.github.io/rai/simulation_and_benchmarking/rai_bench/
- This repo's SCAN-Planner reference (reuse candidate for `navigate_to`): `training/docs/scan-planner.md`
- This repo's existing custom agent experiment (comparison baseline): `experiments/autonomous_architecture_ros2_v1/README.md`
- This repo's project scope: `.planning/PROJECT.md`

---
*Feature research for: RAI-based ROS 2 LLM-agent quadruped control (text chat, walk/navigate/look)*
*Researched: 2026-09-05*
