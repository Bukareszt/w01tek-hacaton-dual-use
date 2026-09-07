---
phase: 01-isolated-rai-environment
plan: 02
subsystem: experiments/wojtek_rai_v1
tags: [rai, ros2, vcstool, colcon, rosdep, rai_interfaces, tdd]

requires:
  - phase: 01-isolated-rai-environment (plan 01)
    provides: "run.sh install|test|container|agent-topics|up scaffold; the wojtek_robot container bind mount and .venv"
provides:
  - "experiments/wojtek_rai_v1/ros/rai_interfaces.repos pinning rai_interfaces to commit 2398f1f3e4c96d790365492294599439a38cdf9a (tag 0.3.0)"
  - "run.sh build: vcs import + rosdep install + colcon build into experiments/wojtek_rai_v1/ros_ws/, idempotent"
  - "tests/test_repos_pin.py, tests/test_build_target.py -- model-free guards over the pin and the run.sh build contract"
affects:
  - ".planning/phases/01-isolated-rai-environment/01-03..05 (rai_interfaces.msg now importable inside the experiment overlay)"

actuals:
  tokens: 3222
  tasks: 2
  commits: 3

tech-stack:
  added: ["vcstool (vcs import)", "rosdep (scoped to experiment overlay)", "colcon --base-paths/--build-base/--install-base/--log-base"]
  patterns:
    - "docker exec -i wojtek_robot bash -s <<'BUILD' for build, mirroring install's own heredoc shape (plan 01-01)"
    - "colcon --base-paths only controls package discovery; --build-base/--install-base/--log-base (the last as a top-level colcon flag, not a build-verb flag) must be set explicitly to relocate output away from cwd"
    - "idempotent vcs import: compare the checked-out HEAD SHA to the pinned SHA before re-cloning"

key-files:
  created:
    - experiments/wojtek_rai_v1/ros/rai_interfaces.repos
    - experiments/wojtek_rai_v1/tests/test_repos_pin.py
    - experiments/wojtek_rai_v1/tests/test_build_target.py
  modified:
    - experiments/wojtek_rai_v1/run.sh

key-decisions:
  - "Resolved rai_interfaces' tag 0.3.0 to commit 2398f1f3e4c96d790365492294599439a38cdf9a via a fresh git ls-remote at execution time (matched the value RESEARCH.md had recorded, but re-verified independently rather than copied)."
  - "Added tests/test_build_target.py beyond the plan's files_modified list (Rule 2 deviation) to give the tdd=\"true\" task a real RED/GREEN test, scoped to the deterministic, network-free part of run.sh build's contract -- the dynamic docker-dependent behaviors stay in the plan's own human-check, matching the existing suite's model-free design (FOUND-06)."

requirements-completed: [FOUND-01, FOUND-02]

coverage:
  - id: D1
    description: "rai_interfaces is pinned to a concrete 40-character commit SHA, never a branch or bare tag"
    requirement: "FOUND-02"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_repos_pin.py (4 tests)"
        status: pass
    human_judgment: false
  - id: D2
    description: "run.sh build imports rai_interfaces at the pinned SHA, resolves its rosdep keys, and colcon-builds it into the experiment's own ros_ws/ overlay, idempotently"
    requirement: "FOUND-01"
    verification:
      - kind: integration
        ref: "./experiments/wojtek_rai_v1/run.sh build (run twice against the live wojtek_robot container); docker exec ... python3 -c 'import rai_interfaces.msg' before (fails) and after sourcing ros_ws/install/setup.bash (prints ok)"
        status: pass
    human_judgment: false
  - id: D3
    description: "No rai_interfaces artifact reaches ros/src/, and ros/ is untouched by the build"
    requirement: "FOUND-01"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_repos_pin.py::test_no_rai_interfaces_or_experiment_source_under_ros_src"
        status: pass
      - kind: other
        ref: "git diff --stat HEAD -- ros/ (empty); ls ros/src | grep -c rai (0)"
        status: pass
    human_judgment: false

duration: "~35 min"
completed: "2026-09-07"
status: complete
---

# Phase 01 Plan 02: rai_interfaces via vcs + colcon Summary

Pinned `rai_interfaces` to commit `2398f1f3e4c96d790365492294599439a38cdf9a` (tag
`0.3.0`, resolved fresh via `git ls-remote`) and implemented `run.sh build`: a
`vcs import` + `rosdep install` + `colcon build` sequence that builds it into
`experiments/wojtek_rai_v1/ros_ws/` -- outside `ros/src/` -- and is safe to
re-run.

## Task 1 -- Pin `rai_interfaces` to a resolved commit SHA

Resolved `git ls-remote https://github.com/RobotecAI/rai_interfaces
refs/tags/0.3.0` at execution time: `2398f1f3e4c96d790365492294599439a38cdf9a`.
This matches the value RESEARCH.md had already recorded from an earlier
session, but was independently re-verified rather than copied, per the task's
own instruction.

Wrote `experiments/wojtek_rai_v1/ros/rai_interfaces.repos` (standard `vcstool`
YAML: `repositories.rai_interfaces.{type,url,version}`) and
`tests/test_repos_pin.py` (4 tests): the `version` field must be a
40-character lowercase hex SHA (never `main`/`master`/a bare tag), the `url`
must point at `RobotecAI/rai_interfaces`, and no path under `ros/src/` may
match `rai_interfaces` or `wojtek_rai_v1`. All four pass with no network
access.

## Task 2 -- `run.sh build` (TDD)

**RED** (`e51f770`): `tests/test_build_target.py` asserts the deterministic,
network-free part of `run.sh build`'s contract -- `--base-paths` present,
`rosdep install` present, no non-comment reference to `ros/src`, and a guard
message naming `colcon` when it is missing. 3 of 4 assertions failed against
the plan-01-01 stub (the fourth -- "never touches `ros/src`" -- trivially held
since the stub touched nothing).

**GREEN** (`9a088a2`): Replaced the stub with the real sequence, run inside
the `wojtek_robot` container via the same `docker exec -i ... bash -s`
heredoc shape `install` already uses:

1. Guard on `colcon` being on `PATH` after sourcing `/opt/ros/jazzy/setup.bash`.
2. `mkdir -p ros_ws/src`; `vcs import ros_ws/src < ros/rai_interfaces.repos`,
   skipped when `ros_ws/src/rai_interfaces` is already checked out at the
   pinned SHA (idempotent).
3. `rosdep update` only when the cache is absent, then
   `rosdep install --from-paths ros_ws/src --ignore-src -r -y`.
4. `colcon --log-base ros_ws/log build --symlink-install --base-paths ros_ws
   --build-base ros_ws/build --install-base ros_ws/install`.

Verified end to end against the live `wojtek_robot` container (all four
`<behavior>` items and all eight task `<acceptance_criteria>`):

- Before the build: `python3 -c "import rai_interfaces.msg"` inside the
  container raised `ModuleNotFoundError`.
- `./run.sh build` exited 0, created
  `experiments/wojtek_rai_v1/ros_ws/install/rai_interfaces/`.
- After sourcing `ros_ws/install/setup.bash`: `import rai_interfaces.msg`
  printed `ok`.
- A second `./run.sh build` exited 0 in ~1s (skipped the `vcs import`,
  reused the resolved rosdep set), left `rai_interfaces.repos` byte-identical
  (`md5sum` unchanged) and produced no `git status` output under `ros_ws/`.
- `git diff --stat HEAD -- ros/` stayed empty throughout; `ls ros/src | grep
  -c rai` returned 0.
- `./run.sh test` (13 tests: 5 from plan 01-01, 4 `test_repos_pin.py`, 4
  `test_build_target.py`) passed.

### System packages `rosdep install` pulled into the container

None of these were present in `ros/docker/Dockerfile`'s image before this
build (Nav2 is explicitly out of scope for this project, so nothing pulled
them in earlier):

| Package | Reason |
|---|---|
| `portaudio19-dev` (+ `libjack-jackd2-0`, `libjack-jackd2-dev`, `libportaudio2`, `libportaudiocpp0`) | `rai_interfaces` audio message support |
| `ros-jazzy-nav2-msgs` (+ `ros-jazzy-geographic-msgs`) | Nav2 message dependency |
| `ros-jazzy-nav2-simple-commander` | Nav2 dependency |
| `ros-jazzy-tf-transformations` (+ `python3-transforms3d`) | transform utility dependency |
| `ros-jazzy-vision-msgs` | detection/perception message dependency |

Installed into the running container by `run.sh build`, exactly as `run.sh
install` already does for `ros-jazzy-cv-bridge` -- never baked into
`ros/docker/Dockerfile`.

### Message-field compatibility with `rai-core==2.12.0`

The pinned commit (tag `0.3.0`) built and imported cleanly with no
compatibility errors; no fallback to `main`'s HEAD was needed. This only
proves the package builds and its Python modules import -- it does not yet
exercise `rai_core`'s actual use of specific message fields (`HRIMessage`,
`RAIDetectionArray`, etc.), which happens when a later phase's tools import
them. If a message-field mismatch surfaces then, re-verify this pin against
`main`'s current HEAD per the `.repos` file's own header comment.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `colcon --base-paths` does not relocate build/install/log output**
- **Found during:** Task 2, first `./run.sh build` run.
- **Issue:** The plan's literal wording (`colcon build --symlink-install
  --base-paths ros_ws`) only tells colcon where to *discover* packages
  (`ros_ws/src/`). It does not change where `build/`, `install/`, `log/` are
  written -- those default to the current working directory, which is the
  experiment root (`cd "$EXP_DIR"` runs before the colcon invocation). The
  first build silently created `build/`, `install/`, `log/` directly under
  `experiments/wojtek_rai_v1/`, not inside `ros_ws/` -- failing the task's own
  acceptance criterion (`ros_ws/install/rai_interfaces/` must exist) and the
  isolation-friendly layout RESEARCH.md's "Recommended Project Structure"
  calls for.
- **Fix:** Added explicit `--build-base ros_ws/build --install-base
  ros_ws/install` (build-verb flags) and `--log-base ros_ws/log` (a
  top-level `colcon` flag, so it must precede `build` on the command line,
  not follow it -- `colcon build --log-base ...` is rejected as an
  unrecognized argument).
- **Files modified:** `experiments/wojtek_rai_v1/run.sh`.
- **Verification:** Removed the misplaced `build/install/log` dirs, reran
  `./run.sh build`; output landed at `ros_ws/{build,install,log}`, import
  succeeded, second run stayed idempotent.
- **Committed in:** `9a088a2` (folded into Task 2's GREEN commit; discovered
  and fixed before that commit was made).

**2. [Rule 2 - Missing critical] Added `tests/test_build_target.py` for the `tdd="true"` task**
- **Found during:** Task 2, before starting implementation.
- **Issue:** The plan marks Task 2 `tdd="true"` and requires a RED test
  before implementation, but its `<files>` list only names `run.sh` and
  `.gitignore` -- no test file. `run.sh build`'s dynamic behaviors (actual
  clone, rosdep resolve, colcon build, import check) can only be exercised
  against the live container over the network, which would make every
  `./run.sh test` invocation network-dependent and minutes-long -- exactly
  the "test-strategy mistake" RESEARCH.md's Pitfalls section flags, and a
  regression against this suite's model-free design (FOUND-06).
- **Fix:** Added `tests/test_build_target.py`, scoped to the deterministic,
  host-readable subset of the `<behavior>` contract: `run.sh`'s text uses
  `--base-paths`, runs `rosdep install`, never references `ros/src` outside
  a comment, and names `colcon` in its missing-tool guard. Confirmed RED (3
  of 4 assertions failed against the stub) before implementing, confirmed
  GREEN after.
- **Files modified:** `experiments/wojtek_rai_v1/tests/test_build_target.py`
  (new).
- **Verification:** `./run.sh test -k test_build_target` -- 4/4 pass after
  the GREEN commit.
- **Committed in:** `e51f770` (RED), `9a088a2` (GREEN, no further changes to
  this file).

---

**Total deviations:** 2 auto-fixed (1 Rule 1 bug in this plan's own new
`run.sh` code, 1 Rule 2 missing-critical test coverage for the `tdd="true"`
task). **Impact:** the Rule 1 fix was necessary for the build to satisfy its
own acceptance criteria; the Rule 2 addition keeps the existing model-free
test-suite discipline intact rather than introducing a network-dependent
test. No scope or architecture change.

## Issues Encountered

None beyond the two deviations above -- both surfaced and were resolved
during this plan's own verification pass, before their respective commits.

## TDD Gate Compliance

Task 2's gate sequence is present and in order:
`git log --oneline --grep="^test(01-02)"` → `e51f770`;
`git log --oneline --grep="^feat(01-02)"` → `9a088a2` (Task 2 GREEN), `6bee2d5`
(Task 1, not a TDD task). No REFACTOR commit was needed -- the GREEN
implementation had no obvious cleanup left after the Rule 1 fix.

## User Setup Required

None -- no external service configuration required. `rosdep install` and the
`vcs import` both ran against public sources already reachable from this
environment; no new secrets or accounts.

## Next Phase Readiness

`rai_interfaces.msg` (and the rest of the package's message/service
definitions) is now importable inside the experiment's container overlay
once `ros_ws/install/setup.bash` is sourced -- ready for whichever later plan
in this phase (or Phase 2) needs `HRIMessage`/`RAIDetectionArray`/etc. No
blockers for `01-03-PLAN.md`.

## Self-Check: PASSED

- All 4 `key-files` entries exist on disk and are tracked by git (`git
  ls-files` confirms each of `ros/rai_interfaces.repos`,
  `tests/test_repos_pin.py`, `tests/test_build_target.py`, `run.sh`).
- `git log --oneline --all --grep="01-02"` returns all three commits
  (`6bee2d5`, `e51f770`, `9a088a2`).
- Every task-level `<acceptance_criteria>` from the plan re-run and passing
  (see Task 1 and Task 2 sections above).
- Plan-level `<verification>` items 1-4 all re-run and passing.

---
*Phase: 01-isolated-rai-environment*
*Completed: 2026-09-07*
