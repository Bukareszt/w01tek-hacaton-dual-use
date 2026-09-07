---
phase: 01-isolated-rai-environment
plan: 04
subsystem: experiments/wojtek_rai_v1
tags: [rai, isolation, tdd, guard-test, docker-compose, tomllib, ast, pytest]

requires:
  - phase: 01-isolated-rai-environment (plan 01)
    provides: "run.sh install/build/test/container/agent/agent-topics/up scaffold, wojtek_robot bind mount, pinned pyproject.toml/uv.lock"
  - phase: 01-isolated-rai-environment (plan 02)
    provides: "rai_interfaces.repos pin, run.sh build, tests/test_repos_pin.py"
provides:
  - "tests/test_isolation_boundary.py -- FOUND-01 import-boundary + deploy-reachability guard, proven fail-first for 3 properties"
  - "tests/test_pinned_versions.py -- FOUND-02 exact-pin + lockfile guard"
  - "tests/test_compose_override.py -- FOUND-03 static half: override extends the base service without redeclaring DDS/RMW vars"
  - "tests/test_model_free_guard.py -- FOUND-06 self-referential guard: no module-scope ROS/RAI/vendor import, suite non-vacuous"
  - "README.md -- status, scope, usage, layout, isolation rules (each naming its enforcing test), single external touch"
  - "docker/compose.override.yaml: read-write repo-root mount at /ros2_ws/repo_root, giving guard tests visibility into ros/ and training/"
  - "conftest.py's repo_root fixture, and test_repos_pin.py's ros/src check, now actually see ros/src inside the container instead of scanning a nonexistent path"
affects:
  - "Phase 2 onward -- the repo_root mount and its read/write-visibility pattern are now part of this experiment's container contract; any new guard test needing to see ros/ or training/ should reuse it"

actuals:
  tokens: 12430
  tasks: 3
  commits: 5

tech-stack:
  added: []
  patterns:
    - "REPO_ROOT resolution prefers /ros2_ws/repo_root when populated, falling back to ancestor-based Path(__file__).resolve().parents[N] for a host-side/EXP_PY path -- duplicated in conftest.py, test_isolation_boundary.py and test_compose_override.py rather than factored into one shared helper (acceptable, small, test-only duplication)"
    - "os.path.samefile(), not Path equality, when comparing a resolved bind-mount path against a path reached through a different mount of the same host directory -- two bind mounts of overlapping host paths are visible at different absolute paths inside the container even though they refer to the same inode"
    - "ast.parse(text).body (module-level statements only) to detect a module-scope import without executing the file being scanned"

key-files:
  created:
    - experiments/wojtek_rai_v1/tests/test_isolation_boundary.py
    - experiments/wojtek_rai_v1/tests/test_pinned_versions.py
    - experiments/wojtek_rai_v1/tests/test_compose_override.py
    - experiments/wojtek_rai_v1/tests/test_model_free_guard.py
    - experiments/wojtek_rai_v1/README.md
  modified:
    - experiments/wojtek_rai_v1/docker/compose.override.yaml
    - experiments/wojtek_rai_v1/tests/conftest.py

key-decisions:
  - "[Rule 2/3 deviation, Task 1] Added a third, read-write bind mount to compose.override.yaml (../..:/ros2_ws/repo_root) beyond the plan's anticipated scope. The wojtek_robot container only bind-mounts this experiment's own directory plus ros/src (D-02); ros/, training/ and ros/deploy.sh are otherwise invisible to run.sh test, the suite's only real execution path, so test_isolation_boundary.py could not run at all without it. Read-write (not read-only, unlike 01-03's .env.example precedent) because the guard's own fail-first boundary demonstration has to write a probe file one directory level outside the experiment -- a sibling under experiments/ -- and no other mount reaches that path."
  - "[Rule 1 deviation, follow-up] Fixed conftest.py's repo_root fixture to prefer the same repo_root mount. This retroactively gave test_repos_pin.py::test_no_rai_interfaces_or_experiment_source_under_ros_src (plan 01-02) real teeth: it had been scanning a nonexistent /ros2_ws/ros/src and passing vacuously (T-01-17) every run.sh test since plan 01-02 was committed. Demonstrated fail-first with a probe path, reverted."
  - "Task 2's tdd=\"true\" gate is satisfied by fail-first demonstration against a deliberately introduced violation (relaxed pin / added environment: block / added a module-scope ROS import), not a classic RED-commit-before-implementation split: all three guard tests passed immediately against already-conformant existing state, so there was no production code gap to drive a real RED. Documented explicitly in the Task 2 commit message rather than manufacturing an artificial RED commit."
  - "test_compose_override.py does not assert 'exactly one volume entry', as the plan's <action> text suggested when it was written. Two independently-justified deviations (01-03's .env.example mount, this plan's own repo_root mount) already make that literally false; the plan's own <behavior>/<must_haves> wording never actually required it, only that the experiment's own mount still exists and DDS/RMW vars are not redeclared -- which is what the test checks."

requirements-completed: [FOUND-01, FOUND-02, FOUND-03, FOUND-06]

coverage:
  - id: D1
    description: "Import-boundary and deploy-reachability guard: nothing outside experiments/wojtek_rai_v1/ references it, ros/src/ carries no rai/experiment-named path, and every ros/deploy.sh rsync source resolves inside ros/"
    requirement: "FOUND-01"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_isolation_boundary.py (7 tests)"
        status: pass
      - kind: other
        ref: "3 fail-first demonstrations: a reference to the experiment name in a new file under training/; a stray wojtek_rai_v1-named path under ros/src/; a widened ros/deploy.sh rsync source ${HERE}/../training/ -- each made the suite red, each reverted, git status clean after each"
        status: pass
    human_judgment: false
  - id: D2
    description: "rai-core/rai-whoami pinned with '==' in pyproject.toml; LangChain/LangGraph carry no direct pin (D-08); uv.lock parses, is non-empty, and resolves langgraph-prebuilt plus at least 4 of the other 5 unconstrained transitives"
    requirement: "FOUND-02"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_pinned_versions.py (9 tests)"
        status: pass
      - kind: other
        ref: "Relaxed rai-core to '>=2.12.0' -- run.sh test failed 1/46; reverted -- 46/46"
        status: pass
    human_judgment: false
  - id: D3
    description: "The compose override extends exactly the base compose.yaml's own service, declares no environment: mapping, and its experiment-directory volume source resolves (against the base compose file's directory) to the experiment directory"
    requirement: "FOUND-03"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_compose_override.py (6 tests)"
        status: pass
      - kind: other
        ref: "Added an environment: block to the override -- 1/6 failed; reverted -- 6/6"
        status: pass
    human_judgment: false
  - id: D4
    description: "No test_*.py file imports rclpy/rai*/cv_bridge/a model vendor SDK at module scope; the suite collects at least 9 files; no test is marked skip unconditionally"
    requirement: "FOUND-06"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_model_free_guard.py (13 tests)"
        status: pass
      - kind: other
        ref: "Added tests/test_zzz_violation_probe.py with a bare 'import rclpy' -- 1/13 failed; deleted the probe -- 13/13"
        status: pass
    human_judgment: false
  - id: D5
    description: "README states status, scope, usage (matching run.sh's real subcommand list), layout, isolation rules (each naming its enforcing test), and contains no private-infrastructure identifier"
    requirement: "FOUND-01"
    verification:
      - kind: other
        ref: "grep -c EXPERIMENTAL (1), grep -cE dotted-quad IP (0), grep -cE email/login form (0); every run.sh usage-line subcommand present in README; test_no_secrets_in_config.py's tracked_config_files() confirmed to include README.md"
        status: pass
    human_judgment: true
    rationale: "Whether the prose is actually clear and accurate to a developer reading it top to bottom (the plan's own human-check) is a judgment call automated scans cannot make, even though every mechanically-checkable property above passed."

duration: "64 min"
completed: "2026-09-07"
status: complete
---

# Phase 01 Plan 04: Isolation, Pinning, Compose-Shape and Model-Free Guards Summary

Four fail-first-proven guard tests (isolation boundary, version pinning, compose-override shape,
and the suite's own model-free contract) plus the experiment README, all running inside the
`wojtek_robot` container -- which required widening its bind mounts, since two of the four guards
need to see `ros/`, `training/` and `ros/deploy.sh`, none of which the container exposed before
this plan.

## Task 1 -- Isolation-boundary guard (TDD)

**RED** (`7c94b31`): wrote `tests/test_isolation_boundary.py` against the `wojtek_robot`
container's then-current mounts. Failed immediately, not on a logic bug but on an environment
constraint: `REPO_ROOT` (ancestor-resolved from the test file) landed on `/ros2_ws`, which has
neither `ros/` nor `training/` as a sibling inside the container -- only this experiment's own
directory and `ros/src` are bind-mounted (D-02).

**GREEN** (`7f8b487`): added a third mount to `docker/compose.override.yaml`,
`../..:/ros2_ws/repo_root`, read-write (unlike 01-03's read-only `.env.example` precedent) because
the test's own fail-first boundary demonstration needs to write a probe file one directory level
outside the experiment -- a sibling under `experiments/`, not reachable by any other existing
mount. `REPO_ROOT` now prefers this mount when populated. All 7 tests passed.

Demonstrated fail-first for all three properties the task named, each reverted (`git status
--porcelain` clean afterward):
1. A reference to `wojtek_rai_v1` in a new file under `training/` -- caught by the general
   production-tree scan.
2. A stray `_tmp_wojtek_rai_v1_probe` directory under `ros/src/` -- caught by the `ros/src`-specific
   check.
3. A widened `ros/deploy.sh` rsync source (`${HERE}/../training/`) -- this one caught a real bug in
   the test itself: the original check did `source.startswith("${HERE}")`, which a `../` escape
   still satisfies. Fixed to substitute `${HERE}` with `ros` and normalize with `os.path.normpath`
   before comparing, then re-demonstrated red, then reverted.

**Follow-up fix** (`c98de23`): while investigating whether the new mount also fixed other tests,
found that `test_repos_pin.py::test_no_rai_interfaces_or_experiment_source_under_ros_src` (plan
01-02) had been scanning a nonexistent `/ros2_ws/ros/src` and passing vacuously (exactly T-01-17,
this plan's own threat register entry) every `run.sh test` run since 01-02 was committed.
`conftest.py`'s `repo_root` fixture now prefers the repo_root mount too. Demonstrated fail-first
with a probe path, reverted.

## Task 2 -- Pinned-version and compose-override guards (TDD)

All three guard tests (`fd9ce4d`) passed immediately against already-conformant existing state --
`rai-core`/`rai-whoami` were already pinned exactly, `uv.lock` already resolved
`langgraph-prebuilt`, the compose override already had the right shape, and no test file imported
a forbidden module at module scope. There was no production-code gap to drive a classic
RED-before-GREEN split, so the `tdd="true"` gate is satisfied instead by fail-first demonstration
against a deliberately introduced violation, one per file:

- `test_pinned_versions.py` (9 tests): relaxed `rai-core` to `>=2.12.0` in `pyproject.toml` --
  `run.sh test` failed 1/46; reverted -- 46/46.
- `test_compose_override.py` (6 tests): added an `environment:` block to the override -- 1/6
  failed; reverted -- 6/6. Comparing the resolved volume source against the experiment directory
  needed `os.path.samefile()`, not `Path` equality: inside the container the experiment directory
  is visible at two different absolute paths through two different bind mounts of the same host
  directory (`/ros2_ws/experiments/wojtek_rai_v1` and `/ros2_ws/repo_root/experiments/wojtek_rai_v1`),
  and lexical path comparison would wrongly fail even on a correct source. Verified the two paths
  are genuinely `os.path.samefile()`-equal against the real container before relying on it.
- `test_model_free_guard.py` (13 tests): added `tests/test_zzz_violation_probe.py` with a bare
  `import rclpy` -- 1/13 failed; deleted the probe -- 13/13.

## Task 3 -- Experiment README

Wrote `README.md` following the sibling experiment's section order: status, scope (what Phase 1
built and, as importantly, what it deliberately has not -- no agent, no tools, no chat, no model
call), a usage section listing every subcommand `run.sh` itself prints with no argument (verified
literally against `./run.sh`'s output), a layout table matching the tree this phase actually
produced, an isolation-rules section naming the enforcing test for three of the five rules
(rules 4 and 5 have no test yet -- documented honestly, since Phase 1 has too little of its own
code for either risk to be real yet, rather than inventing a mapping), the single external touch
(`.env.example`), and a closing secrets section. `grep -c EXPERIMENTAL` returns 1; the dotted-quad
and email/login regexes both return 0. Confirmed directly (not just by inference) that
`test_no_secrets_in_config.py`'s own `tracked_config_files()` helper includes `README.md` in its
scan.

## Final state

`./experiments/wojtek_rai_v1/run.sh test` (and the identical invocation with
`OPENAI_API_KEY`/`ANTHROPIC_API_KEY`/`GOOGLE_API_KEY`/AWS keys unset) exits 0, collects and passes
all 65 tests (37 pre-existing + 28 new across the four guard files), reports 0 skipped. Every one
of the four new guard tests -- plus the retroactively-fixed pre-existing `ros/src` check -- has been
demonstrated red against a real, deliberately introduced violation and green again after revert,
with `git status --porcelain` empty after every single demonstration.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2/3 - Missing critical / Blocking] Widened compose.override.yaml with a third, read-write repo-root mount**
- **Found during:** Task 1, first real `run.sh test -k isolation_boundary` run.
- **Issue:** `test_isolation_boundary.py` needs to see `ros/`, `training/` and `ros/deploy.sh`,
  none of which the container's existing two mounts (D-02, plus 01-03's `.env.example`) exposed.
  Writing the boundary-pair fail-first probe one level outside the experiment also needs write
  access to `experiments/`, which no existing mount reached either.
- **Fix:** Added `../..:/ros2_ws/repo_root` (read-write) to `docker/compose.override.yaml`,
  documented inline with the same rationale style as 01-03's precedent.
- **Files modified:** `experiments/wojtek_rai_v1/docker/compose.override.yaml`.
- **Verification:** `docker exec wojtek_robot ls /ros2_ws/repo_root` shows the real tree; a write
  test through it round-tripped; all subsequent guard tests pass.
- **Commit:** `7f8b487`.

**2. [Rule 1 - Bug] `_rsync_source_resolves_inside_ros` used a bare string-prefix check, missing a `../` escape**
- **Found during:** Task 1's own fail-first demonstration of the third property (widened rsync
  source).
- **Issue:** `source.startswith("${HERE}")` is true for `${HERE}/../training/`, which normalizes to
  `training/` -- outside `ros/` -- defeating the exact check this test exists to be.
- **Fix:** Substitute `${HERE}` with the literal `ros`, `os.path.normpath` the result, and check it
  equals or starts with `ros` + separator.
- **Files modified:** `experiments/wojtek_rai_v1/tests/test_isolation_boundary.py`.
- **Verification:** Re-ran the same widened-source demonstration -- now correctly caught; reverted.
- **Commit:** `7f8b487`.

**3. [Rule 1 - Bug] `test_repos_pin.py`'s ros/src check had been passing vacuously since plan 01-02**
- **Found during:** Task 1 follow-up, investigating the new mount's effect on other tests.
- **Issue:** `conftest.py`'s `repo_root` fixture resolved to `/ros2_ws` inside the container, which
  has no `ros/` sibling -- `repo_root / "ros" / "src"` never existed, so the check's `rglob("*")`
  always returned empty and the test passed without ever scanning anything (T-01-17).
- **Fix:** `repo_root` fixture now prefers `/ros2_ws/repo_root` when populated, mirroring
  `test_isolation_boundary.py`'s identical resolution.
- **Files modified:** `experiments/wojtek_rai_v1/tests/conftest.py`.
- **Verification:** Fail-first demonstrated with a probe path under `ros/src/`; reverted.
- **Commit:** `c98de23`.

---

**Total deviations:** 3 auto-fixed (1 Rule 2/3 missing-critical/blocking mount, 2 Rule 1 bugs --
one in this plan's own new test, one in a prior plan's test that this plan's own mount made
detectable for the first time). **Impact:** all three were necessary for the plan's guard tests to
be genuinely proven, not just appear green; none changed scope or architecture. The repo-root mount
is a new, documented addition to this experiment's container contract that later guard tests can
reuse.

## Issues Encountered

None beyond the three deviations above, all surfaced and resolved during this plan's own
verification pass.

## TDD Gate Compliance

Task 1: `git log --oneline --grep="^test(01-04)"` -> `7c94b31` (RED); `git log --oneline
--grep="^feat(01-04)"` -> `7f8b487` (GREEN). Gate present and in order.

Task 2: no RED commit exists -- see "Key decisions" above and the Task 2 section: all three guard
tests passed immediately against already-conformant state, so there was no failing state to commit
before a fix. Fail-first was demonstrated at the violation level (a deliberately introduced defect
in the file *under test*, not in the guard test itself) instead, documented per-file in the
`fd9ce4d` commit message.

## User Setup Required

None -- no external service configuration required.

## Next Phase Readiness

FOUND-01, FOUND-02, FOUND-03 (static half) and FOUND-06 are now enforced by executable guards, all
proven fail-first, plus the README documenting the contract for a human reader. Phase 1's remaining
plan (01-05, if any) or Phase 2 can build on this container contract, including the new
`/ros2_ws/repo_root` mount if a future guard needs to see `ros/` or `training/`. No blockers.

## Self-Check: PASSED

- All 5 `key-files.created` entries and both `key-files.modified` entries exist on disk (`[ -f ]`
  confirmed for each) and are tracked by git (present in `git log --oneline -8` above HEAD via
  their respective commits).
- `git log --oneline --all --grep="01-04"` returns all 5 commits (`7c94b31`, `7f8b487`, `c98de23`,
  `fd9ce4d`, `f6b70ed`).
- Every task-level `<acceptance_criteria>` from the plan re-run and passing (see each Task section
  above, plus the explicit `grep -c`/`grep -cE` checks run directly against the final files).
- Plan-level `<verification>` items 1-4 all re-run and passing: `run.sh test` exits 0 with 0
  skipped and no vendor keys set; every guard demonstrated red-then-green with `git status
  --porcelain` clean afterward; the README's usage section matches `run.sh`'s real output; no
  address/login/alias anywhere in the README.

---
*Phase: 01-isolated-rai-environment*
*Completed: 2026-09-07*
