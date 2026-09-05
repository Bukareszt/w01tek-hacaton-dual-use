# Requirements: Wojtek RAI Integration

**Defined:** 2026-09-05
**Core Value:** A text instruction typed to a RAI agent makes simulated Wojtek walk, navigate to a goal, and describe what its camera sees, using the production ROS 2 interfaces unchanged.

## v1 Requirements

Requirements for initial release. Each maps to roadmap phases.

### Foundation

- [ ] **FOUND-01**: Developer can run the RAI integration as one self-contained
  directory under `experiments/` with its own `README.md` (status line),
  `run.sh`, Python environment and colcon overlay; nothing outside the
  directory imports it, and `ros/deploy.sh` cannot ship it (isolation rules
  of the sibling experiment apply, and a test enforces the import boundary)
- [ ] **FOUND-02**: Developer can install a pinned RAI stack reproducibly:
  `rai-core==2.12.0`, `rai-whoami==0.0.5`, `rai_interfaces` at a pinned
  commit, with the resolved LangChain/LangGraph versions locked in a
  committed lockfile (RAI leaves them unpinned upstream)
- [ ] **FOUND-03**: Developer can start the agent against the existing
  simulation (`ros/sim.sh`) with one command from the experiment's `run.sh`;
  the agent process inherits the stack's DDS settings (`ROS_DOMAIN_ID`,
  CycloneDDS) and discovers the sim's topics without changes to `ros/docker`,
  `ros/sim.sh`, or any production package
- [ ] **FOUND-04**: Model vendor keys and tracing keys enter only through the
  gitignored `.env`; the committed RAI `config.toml` template contains no
  secret, and a unit test rejects secret-looking values in tracked config
- [ ] **FOUND-05**: Developer can toggle Langfuse tracing in config so every
  prompt, tool call and completion is inspectable; off by default, no
  private host identity committed
- [ ] **FOUND-06**: Developer can run a model-free unit test suite via
  `run.sh test` that needs no LLM key, no ROS runtime and no GPU, covering
  tool argument parsing, velocity clamping, config loading and whoami
  assembly
- [ ] **FOUND-07**: The experiment's container/environment builds and runs on
  both the x86-64 laptop and the aarch64 remote GPU dev box (multi-arch
  image or arch-agnostic install), so sim + agent can run on either

### Embodiment

- [ ] **EMB-01**: Agent identifies as Wojtek: `rai whoami` embodiment built
  from the robot's URDF/MuJoCo model plus a capability document, producing
  identity, constitution and capabilities in the system prompt, rebuildable
  by one `run.sh` target
- [ ] **EMB-02**: Agent correctly explains what it cannot do: asked to grab
  an object, jump, or perform a trick, it declines with its real capability
  list (walk, navigate, look) rather than hallucinating a tool; covered by
  model-free prompt-assembly tests and a scripted evaluation prompt set

### Interaction

- [ ] **HRI-01**: Human can send text to the agent on `/from_human` and read
  replies on `/to_human` using `rai_interfaces` HRI messages, the same
  channel RAI's voice and text demos use
- [ ] **HRI-02**: Human can chat with the agent through a UI started by
  `run.sh` (RAI's Streamlit pattern or a CLI wrapper on the HRI topics),
  seeing tool calls and replies as they stream
- [ ] **HRI-03**: Operator can switch the model vendor (OpenAI-compatible,
  AWS Bedrock, Ollama, Google) purely in config; no vendor is hard-coded in
  the agent code, and the concrete default is chosen in a separate task
- [ ] **HRI-04**: Operator can assign separate "complex" (reasoning) and
  "simple" (image description) model roles in config, per RAI's model-role
  convention

### Motion

- [ ] **MOT-01**: User can type a walking instruction ("walk forward slowly",
  "turn left") and the agent publishes a bounded (vx, vy, wz) on the agent's
  own velocity topic through an allowlisted RAI tool; simulated Wojtek walks
  under the RL policy
- [ ] **MOT-02**: A velocity arbiter (`twist_mux` or equivalent) sits between
  the agent's velocity topic and the policy's `cmd_vel`: it clamps to the
  trained command envelope, zeroes velocity when the agent's command is
  stale (timeout/deadman), and gives teleop/gamepad priority over the agent;
  the agent can never bypass it
- [ ] **MOT-03**: User can type "stop" and the robot receives zero velocity
  immediately through a dedicated stop tool, independent of LLM reasoning
  latency

### Navigation

- [ ] **NAV-01**: User can type a goal ("go to the chair", "go 2 m forward and
  1 m left") and the agent calls a `navigate_to` tool that wraps
  SCAN-Planner's existing executor as a ROS node with a sim pose source, so
  the robot reaches the goal without walking into furniture
- [ ] **NAV-02**: The `navigate_to` tool reports success, blocked, or gave-up
  from the planner's own progress/stuck thresholds and pose evidence, never
  from the LLM's belief; the agent relays that outcome to the user

### Vision

- [ ] **VIS-01**: User can ask "what do you see?" and the agent grabs the
  current sim camera frame through an image tool subscribed with sensor-data
  QoS, sends it to the VLM role, and answers with a scene description
- [ ] **VIS-02**: Agent can query a robot-state tool returning pose, current
  velocity, and a stuck flag, so it can answer "where are you" and check
  progress during tasks

### Agent

- [ ] **AGT-01**: A single RAI conversational (ReAct) agent exposes the
  velocity, stop, navigate, image and state tools and routes free text to
  the right tool
- [ ] **AGT-02**: Multi-step instructions ("go to the chair, then tell me what
  is there") execute as the correct tool sequence; a scripted prompt set
  with expected tool traces validates this in sim
- [ ] **AGT-03**: Long-running navigation runs in a separate RAI StateBased
  execution agent so the conversational agent stays responsive (can answer
  or stop) while the robot is walking

## v2 Requirements

Deferred to future release. Tracked but not in current roadmap.

### Embodiment

- **EMB-03**: whoami vector database (FAISS) over Wojtek documentation for
  retrieval-augmented answers

### Navigation

- **NAV-03**: Nav2 as alternative `navigate_to` backend if SCAN-Planner proves
  insufficient
- **NAV-04**: Object-grounded goals via `rai_perception` open-set detection
  ("the chair" resolved to a detected chair)

### Deployment

- **DEP-01**: RAI agent runs on the onboard Jetson, DDS over the robot's
  access point to the RPi control loop
- **DEP-02**: Local/on-device model backend validated for Jetson

### Interaction

- **HRI-05**: Voice through `rai_asr`/`rai_tts` after a comparison/merge
  decision against the existing voice experiment

## Out of Scope

Explicitly excluded. Documented to prevent scope creep.

| Feature | Reason |
|---------|--------|
| Replacing or merging `autonomous_architecture_ros2_v1` | Side-by-side comparison first; merge is a later decision |
| Skill/trick tools (getup, jump, D-pad tricks) via chat | Expands safety surface for zero milestone value; tricks stay gamepad-only |
| Blanket "confirm every action" gate | Kills the type-and-watch experience; allowlists and the arbiter are the safety boundary in sim |
| `rai_nomad`, `rai_bench`, `rai_sim`, `rai_finetune`, `openset` extras | Not needed for walk/nav/look; `openset` pulls torch/SAM into the container |
| Nav2 full stack in v1 | Wheeled-base assumptions duplicate SCAN-Planner, which is already validated for this quadruped |
| Any change to RL training, observation layout, policy contract, or `wojtek_policy` | Agent consumes the deployed policy through ROS only |
| Physical robot / Jetson bring-up | Sim-first milestone; real robot is next milestone (DEP-01) |
| Concrete model vendor choice | Decided in a separate task; v1 only guarantees swappability (HRI-03) |
| Private infrastructure identity in the tree | Public repo; remote dev box is referenced only via the operator's local SSH alias |

## Traceability

Which phases cover which requirements. Updated during roadmap creation.

| Requirement | Phase | Status |
|-------------|-------|--------|
| FOUND-01 | — | Pending |
| FOUND-02 | — | Pending |
| FOUND-03 | — | Pending |
| FOUND-04 | — | Pending |
| FOUND-05 | — | Pending |
| FOUND-06 | — | Pending |
| FOUND-07 | — | Pending |
| EMB-01 | — | Pending |
| EMB-02 | — | Pending |
| HRI-01 | — | Pending |
| HRI-02 | — | Pending |
| HRI-03 | — | Pending |
| HRI-04 | — | Pending |
| MOT-01 | — | Pending |
| MOT-02 | — | Pending |
| MOT-03 | — | Pending |
| NAV-01 | — | Pending |
| NAV-02 | — | Pending |
| VIS-01 | — | Pending |
| VIS-02 | — | Pending |
| AGT-01 | — | Pending |
| AGT-02 | — | Pending |
| AGT-03 | — | Pending |

**Coverage:**
- v1 requirements: 23 total
- Mapped to phases: 0
- Unmapped: 23 ⚠️

---
*Requirements defined: 2026-09-05*
*Last updated: 2026-09-05 after initial definition*
