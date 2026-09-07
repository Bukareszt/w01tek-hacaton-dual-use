---
phase: 01
plan: 01
subsystem: experiments/wojtek_rai_v1
tags: [rai, isolation, uv, docker-compose, ros2-topics, tracer]
requires: []
provides:
  - "experiments/wojtek_rai_v1/run.sh install|test|container|agent-topics|up (build/agent stubs for later plans)"
  - "pinned rai-core==2.12.0 + rai-whoami==0.0.5 install, committed uv.lock"
  - "wojtek_rai.topics.list_topics/format_topics/main"
  - "docker/compose.override.yaml bind-mount pattern for this experiment"
affects:
  - ".planning/phases/01-isolated-rai-environment/01-02..05 (inherit run.sh contract, container decision)"
tech-stack:
  added: ["rai-core==2.12.0", "rai-whoami==0.0.5", "uv (runtime-fetched, not baked into image)"]
  patterns:
    - "docker exec -i wojtek_robot bash -s <<HEREDOC for every container-side script (no host-side quoting of container paths)"
    - "compose override extends wojtek_robot with one bind-mounted volume, no environment: block (DDS vars inherited)"
key-files:
  created:
    - experiments/wojtek_rai_v1/run.sh
    - experiments/wojtek_rai_v1/pyproject.toml
    - experiments/wojtek_rai_v1/uv.lock
    - experiments/wojtek_rai_v1/.gitignore
    - experiments/wojtek_rai_v1/docker/compose.override.yaml
    - experiments/wojtek_rai_v1/wojtek_rai/__init__.py
    - experiments/wojtek_rai_v1/wojtek_rai/topics.py
    - experiments/wojtek_rai_v1/tests/conftest.py
    - experiments/wojtek_rai_v1/tests/test_topics_module.py
  modified: []
key-decisions:
  - "Task 1 (checkpoint:decision): option-a -- run.sh owns the wojtek_robot container lifecycle (replicates ros/sim.sh's/ros/dev.sh's own platform-detection branch), rather than adding an extra-compose-file hook to ros/sim.sh/ros/dev.sh. Zero edits under ros/ was the deciding criterion; the accepted cost is that run.sh's container/up targets duplicate that platform-detection logic."
  - "Task 2 (checkpoint:human-verify): approved -- rai-core 2.12.0 and rai-whoami 0.0.5 independently checked against PyPI/GitHub (RobotecAI org, Apache-2.0) before the first uv sync; the LangChain/LangGraph/faiss-cpu/opencv/streamlit/tomli transitive set accepted."
requirements-completed: []
duration: "~2.5 hours"
completed: "2026-09-07"
status: complete
coverage:
  - deliverable: "run.sh install produces a pinned venv with a committed uv.lock"
    verification:
      - kind: command
        ref: "./experiments/wojtek_rai_v1/run.sh install (run twice)"
        status: pass
    human_judgment: false
  - deliverable: "run.sh test is green with no LLM key, no ROS daemon, no GPU"
    verification:
      - kind: test
        ref: "experiments/wojtek_rai_v1/tests/test_topics_module.py (5 tests)"
        status: pass
    human_judgment: false
  - deliverable: "run.sh agent-topics lists the live simulation's topics (cmd_vel + camera) through RAI's own ROS2Connector"
    verification:
      - kind: command
        ref: "./experiments/wojtek_rai_v1/run.sh agent-topics against a live ros2 run wojtek_bringup robot --sim"
        status: pass
    human_judgment: false
  - deliverable: "Nothing under ros/docker/, ros/sim.sh, ros/dev.sh or ros/src/ is modified"
    verification:
      - kind: command
        ref: "git diff --stat HEAD -- ros/ (empty)"
        status: pass
    human_judgment: false
actuals:
  tokens: 26000
  tasks: 3
  commits: 1
---

# Phase 01 Plan 01: Isolated RAI Environment -- Walking Skeleton Summary

Scaffolded `experiments/wojtek_rai_v1/` end to end: a pinned `rai-core==2.12.0` +
`rai-whoami==0.0.5` install inside the existing `wojtek_robot` container (via a
compose override, never editing `ros/docker/`), and a `run.sh agent-topics`
target that lists the running MuJoCo simulation's live ROS 2 topics
(`cmd_vel`, the D435 camera topics) through RAI's own
`ROS2Connector.get_topics_names_and_types()` -- not a hand-rolled `rclpy` call.

## Task 1 -- Container lifecycle decision (checkpoint:decision)

**Decision: option-a.** `run.sh` owns bringing up `wojtek_robot` itself, replicating
the exact platform-detection branch (`uname -s` / `docker info` nvidia runtime) that
`ros/sim.sh` and `ros/dev.sh` already use, plus this experiment's compose override.

**Rationale (recorded by the human before this agent was spawned):** zero edits
under `ros/` was the deciding criterion -- it satisfies D-02, criterion 2's "no
changes to ros/docker, ros/sim.sh or ros/src", and the CLAUDE.md experiments
isolation rule to the letter. The accepted cost is duplicated platform-detection
logic between `run.sh` and `ros/sim.sh`/`ros/dev.sh`, which can drift if those
scripts change their own branch later.

**Verified:** `ros/sim.sh`, `ros/dev.sh` and every file under `ros/docker/` are
byte-identical to `HEAD` (`git diff --stat HEAD -- ros/` is empty).

## Task 2 -- Package legitimacy gate (checkpoint:human-verify, blocking-human)

**Approved** (recorded by the human before this agent was spawned) after checking:

1. `rai-core` 2.12.0 on PyPI -- project links to `github.com/RobotecAI/rai`,
   Apache-2.0, version exists.
2. `rai-whoami` 0.0.5 on PyPI -- same publisher, version exists.
3. `github.com/RobotecAI/rai` -- confirmed the real RobotecAI organisation, tag
   `2.12.0` exists.

The LangChain/LangGraph, faiss-cpu, opencv, streamlit and tomli transitive set
`rai-core==2.12.0` pulls in was accepted. No `uv sync`/`pip install` ran before
this approval.

## Task 3 -- End-to-end tracer: "list the simulation's live topics"

Built the full walking skeleton in one path, verified against a live simulation:

- `experiments/wojtek_rai_v1/run.sh` -- `install | build | test | container | agent |
  agent-topics | up`. `build` and `agent` are stubs (plan 01-02 and Phase 2
  respectively); the other five are real.
- `pyproject.toml` -- `rai-core==2.12.0`, `rai-whoami==0.0.5` pinned exact; a
  `[dependency-groups] dev` group carrying `pytest>=8.0.0` and `pyyaml>=6`; no
  version constraint anywhere on `langchain*`/`langgraph*` (D-08).
- `uv.lock` -- committed, 528KB, resolves 141 packages including every
  `langchain*`/`langgraph*` transitive.
- `docker/compose.override.yaml` -- one bind-mount (`../../experiments/wojtek_rai_v1`
  -> `/ros2_ws/experiments/wojtek_rai_v1`) added to the `wojtek_robot` service, no
  `environment:` block (DDS vars inherited unmodified, D-04).
- `wojtek_rai/topics.py` -- `list_topics`, `format_topics`, `main`; the
  `rai.communication.ros2` import is function-local, confirmed importable with
  no ROS 2 on `PYTHONPATH`.
- `tests/` -- 5 model-free tests, all passing with no LLM key, no ROS daemon,
  no GPU.

### Container's actual python3 version

RESEARCH.md assumption A1 ("the container's python3 is exactly 3.12, not
independently confirmed") is **confirmed**: `python3 --version` inside
`wojtek_robot` (after sourcing `/opt/ros/jazzy/setup.bash`) reports **Python
3.12.3**.

### Resolved transitive versions (from `uv.lock`, this session's first `uv sync` + `uv lock`)

| Package | Resolved version |
|---|---|
| `langchain` | 1.4.0 |
| `langchain-core` | 1.6.2 |
| `langgraph` | 1.2.11 |
| `langgraph-prebuilt` | 1.1.0 |
| `langchain-aws` | 1.7.5 |
| `langchain-openai` | 1.6.0 |
| `langchain-ollama` | 1.1.0 |
| `langchain-google-genai` | 4.4.0 |
| `langchain-community` | 0.4.2 |

### Live verification against the running simulation

Started the sim inside the `wojtek_robot` container
(`ros2 run wojtek_bringup robot --sim --no-viz policy:=hvsr-robotics/wojtek-quiet-locomotion`,
via the same session command `run.sh up` execs). `./experiments/wojtek_rai_v1/run.sh
agent-topics` printed (truncated to the criterion-relevant rows):

```
/camera/camera/color/camera_info  ['sensor_msgs/msg/CameraInfo']
/camera/camera/color/image_raw  ['sensor_msgs/msg/Image']
/camera/camera/depth/camera_info  ['sensor_msgs/msg/CameraInfo']
/camera/camera/depth/image_rect_raw  ['sensor_msgs/msg/Image']
/cmd_vel  ['geometry_msgs/msg/Twist']
... (25 more topics: controller_manager/*, /joint_states, /imu_sensor_broadcaster/imu,
    /tf, /tf_static, /rosout, /parameter_events, /sim/qpos, /sim/rtf,
    /wojtek/nav_command, /wojtek/joint_targets, /wojtek/joint_states_abs,
    /magnetometer_broadcaster/magnetic_field, /odom_vel, /robot_description,
    /dynamic_joint_states, /diagnostics)
```

`cmd_vel` and both camera streams (color + depth) are present, satisfying the
plan's backstop truth. `git status --short` showed no modification under
`ros/docker/`, `ros/sim.sh`, `ros/dev.sh` or `ros/src/` at the time of this check.

`./experiments/wojtek_rai_v1/run.sh install` run a second time exited 0 and left
`uv.lock` byte-identical (idempotency truth verified).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `run.sh test`'s host-native interpreter path silently ran the wrong Python**
- **Found during:** Task 3, first `run.sh test` run.
- **Issue:** The plan's literal wording (matching the sibling experiment's `EXP_PY`
  fallback chain) has `run.sh test` invoke `$HERE/.venv/bin/python` directly from
  the host. That venv was built *inside* the container with
  `uv venv --system-site-packages`, so its `bin/python` is a symlink to an
  absolute container-side path (`/usr/bin/python3`). On this development
  machine that same absolute path also exists on the host (Fedora 41 happens
  to ship `/usr/bin/python3.12` too), so the symlink silently resolved to an
  entirely unrelated **host** interpreter, which found `pytest` via the host
  user's own `~/.local` site-packages -- not this experiment's pinned install.
  The test suite passed for the wrong reason and would not reliably import
  `rai`/`rclpy` from the actual venv on a different host layout.
- **Fix:** `test`'s default path (no `EXP_PY` override) now runs inside the
  container via the same `docker exec -i wojtek_robot bash -s` mechanism as
  `install`/`agent-topics`, checking `.venv/bin/python` exists there first and
  printing "run ./run.sh install first" if not. `EXP_PY=/path/to/python` still
  works as an explicit host-interpreter escape hatch for a Docker-less CI
  environment.
- **Files modified:** `experiments/wojtek_rai_v1/run.sh`.
- **Verification:** `./run.sh test` now genuinely runs inside `.venv`
  (`.venv/lib/python3.12/site-packages/...` visible in tracebacks/output);
  confirmed idempotent and green.
- **Commit:** `5a4fcf0` (folded into Task 3's single commit; discovered and
  fixed before that commit was made).

**2. [Rule 1 - Bug] `--system-site-packages` venv exposes an incompatible ROS pytest plugin**
- **Found during:** Task 3, after fixing deviation 1 above.
- **Issue:** With `test` running inside the container, pytest's setuptools-entrypoint
  plugin autoload picked up the container's apt-installed `launch_testing` pytest
  plugin (built against the system ROS pytest), which is incompatible with the
  newer pinned `pytest==9.1.1` this venv's `uv.lock` resolved --
  `PluginValidationError: Plugin 'launch_testing' for hook 'pytest_pycollect_makemodule'`
  before a single test ran.
- **Fix:** `container_py()` now exports `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` for
  every container-side Python invocation. This suite needs no ROS pytest plugin
  and the setting has no effect on `rclpy`/`cv_bridge` availability (unrelated
  to pytest's own plugin discovery).
- **Files modified:** `experiments/wojtek_rai_v1/run.sh`.
- **Verification:** `./run.sh test` passes cleanly after the fix.
- **Commit:** `5a4fcf0`.

**3. [Rule 1 - Bug] Test assumed discovery always fails with no live sim -- wrong**
- **Found during:** Task 3, after fixing deviation 2, running the suite for real
  inside the container.
- **Issue:** `test_main_is_callable_with_optional_argv` asserted `main([])`
  returns non-zero, reasoning that ROS 2 discovery would fail with no sim
  running. In fact `ROS2Connector`'s own `TransformListener` subscribes to
  `/tf`/`/tf_static` on construction, so once a live ROS 2 domain exists (as it
  does inside the container, `ROS_DOMAIN_ID=42`), discovery always succeeds and
  returns at least the connector's own topics -- `main([])` returned 0, failing
  the test's over-specified assertion.
- **Fix:** Narrowed the assertion to exactly the plan's `<behavior>` contract:
  `main` is callable, accepts an optional `argv`, and always returns an `int`
  (never raises) -- not a specific exit code.
- **Files modified:** `experiments/wojtek_rai_v1/tests/test_topics_module.py`.
- **Verification:** Full suite green (5/5).
- **Commit:** `5a4fcf0`.

**4. [Rule 3 - Blocking issue, no file changes] Stale colcon overlay blocked the live-sim verification**
- **Found during:** Task 3's manual end-to-end check (starting the simulation to
  verify `agent-topics` against live topics).
- **Issue:** The `wojtek_robot` container's existing `install/` overlay still had
  a stale `wojtek_viz` package from before a rename to `wojtek_pc` (an
  environment leftover from prior sessions, unrelated to this plan), so
  `ros2 run wojtek_bringup robot --sim` failed with
  `Package 'wojtek_pc' not found`.
- **Fix:** Ran the exact prune-then-`colcon build` freshness pass `ros/sim.sh`
  itself performs automatically (prune `build`/`install` entries whose source
  package no longer exists, then `colcon build --packages-skip
  md80_hardware_interface`) directly against the running container, purely to
  get a real simulation up for this plan's own verification step. No file in
  the repository was changed by this fix -- it is a one-time runtime state
  correction on this development machine, the same maintenance `ros/sim.sh`
  performs on every invocation.
- **Files modified:** none.
- **Verification:** Simulation started cleanly afterward; `agent-topics`
  verification (above) succeeded.

**Total deviations:** 4 auto-fixed (3 Rule 1 bugs in this plan's own new code/tests,
1 Rule 3 blocking runtime-state issue with no file changes). **Impact:** all four
were found and fixed during this plan's own verification pass, before the single
Task 3 commit; none required a scope or architecture change. Deviations 1 and 2
are worth flagging to later phases: any container-side Python invocation this
experiment adds should keep going through `container_py()` (or an equivalent that
runs inside the container and sets `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`), not a
direct host-side `.venv/bin/python` invocation.

## Known Stubs

- `run.sh build` -- stub, prints a pointer to plan 01-02 and exits non-zero. Per
  plan: plan 01-02 implements `rai_interfaces` import via `vcs` + `colcon build`.
- `run.sh agent` -- stub, prints "not implemented until Phase 2" and exits
  non-zero. Per plan: reserved for Phase 2's ReAct agent entry point.

Both are explicitly in-scope stubs the plan itself specifies for this task; no
later plan in *this* phase resolves `agent` (that is Phase 2), but plan 01-02
resolves `build`.

## Self-Check: PASSED

- All 9 files in `key-files.created` exist on disk and are tracked by git
  (`git ls-files` confirms each).
- `git log --oneline --all --grep="01-01"` returns the Task 3 commit `5a4fcf0`.
- Every task-level `<acceptance_criteria>` from the plan re-run and passing (see
  Task 3 section and the "second install idempotent" note above).
- Plan-level `<verification>` items 1-4 all re-run and passing; item 5
  (`rm -rf experiments/wojtek_rai_v1` leaves the tree working) is deferred to
  plan 01-04's formal isolation test per the plan's own text.

## Next

Ready for `01-02-PLAN.md` (rai_interfaces via `vcs` + colcon build into
`ros_ws/`, resolving `run.sh build`'s stub).
