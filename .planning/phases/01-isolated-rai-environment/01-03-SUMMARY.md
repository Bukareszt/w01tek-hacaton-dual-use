---
phase: 01-isolated-rai-environment
plan: 03
subsystem: experiments/wojtek_rai_v1
tags: [rai, secrets, config, tdd, guard-test]

requires:
  - phase: 01-isolated-rai-environment (plan 01)
    provides: "run.sh install/test/container/agent-topics scaffold, wojtek_robot container + bind mount"
  - phase: 01-isolated-rai-environment (plan 02)
    provides: "run.sh build, rai_interfaces overlay in ros_ws/"
provides:
  - "experiments/wojtek_rai_v1/config.toml -- upstream RAI vendor/tracing template, no credential field"
  - "wojtek_rai.config.config_path/load_experiment_config/required_env_vars"
  - "tests/test_no_secrets_in_config.py -- secret-shape + private-infrastructure-identity guard, container-aware"
  - "run.sh: repo-root .env sourcing + CRED_ENV_ARGS credential passthrough into container_py()"
  - "root .env.example: 7 placeholder vendor/tracing credential names"
affects:
  - ".planning/phases/01-isolated-rai-environment/01-04-PLAN.md (inherits the same guard-test container-fallback pattern if it needs to scan tracked files from inside wojtek_robot)"

actuals:
  tokens: 5800
  tasks: 3
  commits: 4

tech-stack:
  added: []
  patterns:
    - "Guard tests that must run inside the wojtek_robot container detect an unreachable .git (only the experiment dir + ros/src are bind-mounted, D-02) and fall back to a .gitignore-aware directory walk instead of skipping"
    - "A single, narrowly-scoped read-only bind mount (.env.example only, already-public placeholder content) closes the gap between 'what FOUND-04 requires scanned' and 'what the container's filesystem view contains', without widening D-02's experiment-directory-only mount to the whole repo"
    - "docker exec -e NAME (no '=value') forwards a credential by name from the caller's own environment, never restating the value on a command line"

key-files:
  created:
    - experiments/wojtek_rai_v1/config.toml
    - experiments/wojtek_rai_v1/wojtek_rai/config.py
    - experiments/wojtek_rai_v1/tests/test_config_loading.py
    - experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py
  modified:
    - experiments/wojtek_rai_v1/run.sh
    - experiments/wojtek_rai_v1/docker/compose.override.yaml
    - .env.example

key-decisions:
  - "[Rule 2 deviation] Added a second, read-only bind mount of the repo-root .env.example (already-public, placeholder-only content) to docker/compose.override.yaml, at /ros2_ws/.env.example. Not in the plan's files_modified list. Necessary because the wojtek_robot container's filesystem view (D-02) contains only this experiment's own directory plus ros/src -- neither the repository's .git nor its root files are part of either mount -- so without this, tests/test_no_secrets_in_config.py could not scan .env.example at all under run.sh test, the suite's only real execution path, defeating FOUND-04's literal requirement."
  - "[Rule 3 deviation] tracked_config_files() detects whether .git is reachable by walking upward from the test file; when it is not (inside the container), it falls back to a plain directory walk filtered by this experiment's own .gitignore entries rather than failing or skipping. The fallback can only over-scan relative to git ls-files, never under-scan -- the safe direction for a security guard."
  - "PRIVATE_IDENTITY_RE's dotted-quad-IP alternative is exempted for uv.lock specifically: opencv-python-headless's version string '4.11.0.86' is shape-indistinguishable from an IP address, and a machine-generated lockfile is not where a human would hand-write a host identity. uv.lock stays covered by the secret-shape check."
  - "required_env_vars() returns only the five vendor-credential names (openai/google/aws x3); run.sh's CRED_ENV_ARGS additionally forwards the two Langfuse tracing names, since tracing keys are not vendor-specific and are declared alongside the vendor keys in .env.example."

requirements-completed: [FOUND-04, FOUND-06]

coverage:
  - id: D1
    description: "config.toml carries model names and disabled tracing only, no credential field, parses with tomllib"
    requirement: "FOUND-04"
    verification:
      - kind: unit
        ref: "python3 -c tomllib parse + top-level table assertions (task acceptance criteria)"
        status: pass
    human_judgment: false
  - id: D2
    description: "wojtek_rai.config is cwd-independent, raises FileNotFoundError naming the path it tried, and required_env_vars() maps each of the four vendors exactly, cross-checked against .env.example"
    requirement: "FOUND-06"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_config_loading.py (10 tests)"
        status: pass
    human_judgment: false
  - id: D3
    description: "A model-free guard rejects secret-shaped values and private-infrastructure identity in every tracked file under the experiment plus .env.example, and is proven fail-first"
    requirement: "FOUND-04"
    verification:
      - kind: unit
        ref: "experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py (7 tests)"
        status: pass
      - kind: other
        ref: "Introduced a synthetic sk-xxxx... value into config.toml -- run.sh test failed (1 failed, 29 passed); reverted -- run.sh test returned to 30/30, git diff against HEAD showed a clean revert"
        status: pass
    human_judgment: false
  - id: D4
    description: "run.sh sources the repo-root .env when present and forwards only the declared credential names into the container by name, printing no value; identical behavior with no .env present"
    requirement: "FOUND-04"
    verification:
      - kind: other
        ref: "./run.sh test green with no repo-root .env; wrote a scratch .env with a dummy OPENAI_API_KEY, ran agent-topics, grepped output for the dummy value (0 occurrences), removed the scratch .env, reran agent-topics"
        status: pass
    human_judgment: false
---

# Phase 01 Plan 03: Config, Secrets, and Env Passthrough Summary

Committed the upstream RAI vendor/tracing config template (no credential field
exists in its schema at all), a working-directory-independent `wojtek_rai.config`
module, a container-aware secret-shape and private-infrastructure-identity guard
test proven fail-first, and `run.sh` credential passthrough from the repo-root
`.env` -- all without ever writing a value, only names, into any tracked file.

## Task 1 -- Vendor config template and `.env.example`

Committed `experiments/wojtek_rai_v1/config.toml` verbatim from the upstream RAI
`2.12.0` template (research-verified), keeping `[vendor]`, `[aws]`, `[openai]`,
`[ollama]`, `[google]`, `[tracing]`, `[tracing.langfuse]`, `[tracing.langsmith]`
and dropping `[asr]`/`[tts]` (voice, out of scope; `load_config()` only requires
`[vendor]`). Both tracing switches ship `false`, the upstream default and this
project's FOUND-05 requirement. Appended seven placeholder credential names
(`OPENAI_API_KEY`, `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`,
`AWS_SESSION_TOKEN`, `GOOGLE_API_KEY`, `LANGFUSE_PUBLIC_KEY`,
`LANGFUSE_SECRET_KEY`) to the repo-root `.env.example`, all left empty, under a
new section naming the RAI experiment.

## Task 2 -- Config module and secret/identity guard (TDD)

**RED** (`608ff5f`): wrote `tests/test_config_loading.py` and
`tests/test_no_secrets_in_config.py` against a `wojtek_rai.config` module that
did not exist yet. `./run.sh test` failed at collection with
`ImportError: cannot import name 'config' from 'wojtek_rai'`.

**GREEN** (`e04961f`): implemented `wojtek_rai/config.py` --
`config_path()` (anchored on `Path(__file__).resolve().parent.parent`, not the
process cwd -- RAI's own loader is cwd-relative, and the container's default cwd
is `/ros2_ws`, not the experiment root), `load_experiment_config()` (`tomllib`,
no RAI import, no network), and `required_env_vars(vendor)` (exact tuple per
vendor, `ValueError` naming an unknown one).

Running the new tests against the real environment surfaced a genuine
environment constraint, not a code bug: the `wojtek_robot` container only has
this experiment's own directory plus `ros/src` bind-mounted (D-02) -- there is
no `.git` and no root `.env.example` anywhere inside that mount, so a naive
`git ls-files` (as drafted in RESEARCH.md) fails with exit 128 ("not a git
repository") the moment the suite actually runs through `run.sh test`, the
suite's only real execution path. Resolved with two scoped, documented
deviations (see below) rather than skipping the check or narrowing FOUND-04's
scope: `tracked_config_files()` now detects whether `.git` is reachable and
falls back to a `.gitignore`-aware directory walk when it isn't, and
`docker/compose.override.yaml` gained one additional read-only bind mount of
just `.env.example` (already public, placeholder-only content) so the fallback
path can still cover it.

A second false positive surfaced once real files were scanned: `uv.lock`
contains `opencv-python-headless`'s version string `"4.11.0.86"`, which is
shape-indistinguishable from a dotted-quad IP. Exempted `uv.lock` from the
private-identity check specifically (still covered by the secret-shape check),
documented inline with the reasoning.

Demonstrated fail-first per the plan's acceptance criteria: appended a
synthetic `sk-xxxxxxxxxxxxxxxxxxxx` value to `config.toml`, ran `./run.sh
test` (1 failed, 29 passed -- `test_no_secret_shaped_values_in_tracked_files`
caught it with the offending file and value named), reverted `config.toml` to
its committed content (`diff` against `git show HEAD:...` was empty), reran
`./run.sh test` (30/30 passed again).

## Task 3 -- `run.sh` env passthrough

Added `.env` sourcing (`set -a; . "$REPO_ROOT/.env"; set +a`, mirroring
`ros/deploy.sh`'s existing block byte-for-shape) above the subcommand dispatch,
and a `CRED_ENV_ARGS` array built from the seven vendor/tracing names, including
a name only when the calling process already has it set and non-empty.
`container_py()` -- the shared funnel for every container-side Python
invocation (`test`, `agent-topics`, and Phase 2's future `agent`) -- now passes
`${CRED_ENV_ARGS[@]+"${CRED_ENV_ARGS[@]}"}` into its `docker exec -i` call, using
the value-free `-e NAME` form so a credential's value never appears on a
command line, in a log line, or in any file. No experiment-local `.env` (D-11).

Verified both branches of the precondition: `./run.sh test` stayed green
(30/30) with no repo-root `.env` present (confirmed absent on this machine).
Wrote a scratch repo-root `.env` with `OPENAI_API_KEY=dummy-test-value-not-real`,
ran `./run.sh agent-topics`, and grepped its full output for the dummy value --
zero occurrences. Removed the scratch `.env` and reran `agent-topics`
successfully (printed the live ROS 2 graph's base topics). `./run.sh` with no
argument still prints the usage block and exits `1`.

## Secret scan coverage (as of this plan)

Inside the container (the suite's real execution path), `tracked_config_files()`
falls back to the directory-walk path and returns every file under
`experiments/wojtek_rai_v1/` except what `.gitignore` excludes
(`.venv/`, `.tools/`, `.uv-cache/`, `ros_ws/{build,install,log,src}/`,
`__pycache__/`) and this guard test's own source, plus the bind-mounted
`.env.example`. On the host (`git ls-files experiments/wojtek_rai_v1`, used if
`.git` is reachable, e.g. a future `EXP_PY` CI path), the tracked set was:

```
experiments/wojtek_rai_v1/.gitignore
experiments/wojtek_rai_v1/config.toml
experiments/wojtek_rai_v1/docker/compose.override.yaml
experiments/wojtek_rai_v1/pyproject.toml
experiments/wojtek_rai_v1/ros/rai_interfaces.repos
experiments/wojtek_rai_v1/run.sh
experiments/wojtek_rai_v1/tests/conftest.py
experiments/wojtek_rai_v1/tests/test_build_target.py
experiments/wojtek_rai_v1/tests/test_config_loading.py
experiments/wojtek_rai_v1/tests/test_repos_pin.py
experiments/wojtek_rai_v1/tests/test_topics_module.py
experiments/wojtek_rai_v1/uv.lock
experiments/wojtek_rai_v1/wojtek_rai/__init__.py
experiments/wojtek_rai_v1/wojtek_rai/config.py
experiments/wojtek_rai_v1/wojtek_rai/topics.py
```
(`test_no_secrets_in_config.py` self-excludes from this list; `.env.example`
is appended separately.) 15 files, all scanned clean by both regexes (with the
one documented `uv.lock` exemption for the private-identity check only).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical] Added a read-only `.env.example` bind mount to `docker/compose.override.yaml`**
- **Found during:** Task 2, first real `./run.sh test` run of the new guard test.
- **Issue:** FOUND-04 requires the guard test to scan `.env.example` alongside
  the experiment's tracked files, but the `wojtek_robot` container's only
  bind mounts (D-02) are the experiment directory and `ros/src` -- the
  repository root, including `.env.example`, is not part of either mount, and
  is therefore invisible to code running inside the container, which is
  `run.sh test`'s only real execution path.
- **Fix:** Added one additional, read-only volume line to
  `docker/compose.override.yaml`: `../../.env.example:/ros2_ws/.env.example:ro`.
  Scoped to exactly this one file, chosen because it is already public,
  committed, placeholder-only content by design -- mounting it carries no more
  risk than the file already carries by being tracked. Read-only so nothing
  inside the container can edit the host's copy.
- **Files modified:** `experiments/wojtek_rai_v1/docker/compose.override.yaml`
  (not in the plan's `files_modified` list).
- **Verification:** `docker exec wojtek_robot cat /ros2_ws/.env.example` (host
  side, via a Python read, never printed to the transcript) confirmed the
  mount; `./run.sh test` passed with the cross-check test
  (`test_every_required_env_var_name_is_declared_in_root_env_example`) actually
  reading real content.
- **Commit:** `e04961f`.

**2. [Rule 3 - Blocking issue] `tracked_config_files()` falls back to a directory walk when `.git` is unreachable**
- **Found during:** Task 2, same test run as above.
- **Issue:** `git ls-files` against `REPO_ROOT` (computed as
  `Path(__file__).resolve().parents[3]`, which resolves to `/ros2_ws` inside
  the container) failed with `CalledProcessError` / exit 128 -- "not a git
  repository" -- because no `.git` directory exists anywhere inside the
  container's bind-mounted view. Skipping the check was not an option (the
  plan explicitly requires "fail loudly rather than skip"), and neither is
  scanning a wrong or empty set silently.
- **Fix:** Added `_find_git_root()` (walks upward from the test file looking
  for `.git`) and `_walk_experiment_dir_excluding_generated()` (a plain
  `rglob` filtered by this experiment's own `.gitignore` entries).
  `tracked_config_files()` uses real `git ls-files` when `.git` is reachable
  (a possible future host-side/CI path) and the directory-walk fallback
  otherwise (the container path, `run.sh test`'s actual, only execution route
  today). The fallback can only over-scan relative to `git ls-files`, never
  under-scan.
- **Files modified:** `experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py`
  (within Task 2's own declared file, but beyond what the plan's drafted
  research code sketch anticipated).
- **Verification:** `./run.sh test` -- both guard tests pass inside the
  container via the fallback path; fail-first demonstration (above) confirms
  the fallback actually detects a real violation, not just an empty scan.
- **Commit:** `e04961f`.

**3. [Rule 1 - Bug] Regex false positives excluded: guard test's own source, and `uv.lock`'s version string**
- **Found during:** Task 2, running the guard against every real tracked file.
- **Issue:** `PRIVATE_IDENTITY_RE`, applied to the guard test's own source
  file, matched its own regex literal (`ssh|scp|rsync`) and its own
  runtime-built synthetic test fixtures -- a guard test inherently contains
  the patterns it detects, as code. Separately, `uv.lock` contains
  `opencv-python-headless`'s PyPI version string `"4.11.0.86"`, which the
  dotted-quad-IP alternative cannot distinguish from a real IP by shape alone.
- **Fix:** `tracked_config_files()` excludes `THIS_FILE` (the guard test's own
  resolved path) from its returned set;
  `test_no_private_infrastructure_identity_in_tracked_files` skips `uv.lock`
  specifically (still covered by the secret-shape check), with the reasoning
  documented inline at the skip.
- **Files modified:** `experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py`.
- **Verification:** `./run.sh test` -- 30/30 passing on the real, unmodified
  tree.
- **Commit:** `e04961f`.

**4. [Rule 3 - Blocking issue, no file changes] Recreating `wojtek_robot` for the compose-override change dropped the runtime-installed `ros-jazzy-cv-bridge`**
- **Found during:** Task 3, verifying `agent-topics` with a scratch `.env`.
- **Issue:** Deviation 1's compose-override edit required
  `docker compose ... up -d --remove-orphans` to pick up the new mount, which
  recreates the container -- and, like plan 01-01's own deviation 4, wipes any
  `apt-get install`ed package that was not baked into the image (this
  project's `ros-jazzy-cv-bridge`, installed at `run.sh install` time into the
  *previous* container instance). `agent-topics` then failed with
  `No module named 'cv_bridge'` -- unrelated to credentials or this task's own
  code, but it would have obscured the human-check's actual result.
- **Fix:** Reinstalled `ros-jazzy-cv-bridge` into the running container via
  the exact same `apt-get install -y --no-install-recommends` step
  `run.sh install` already performs. No file in the repository was changed --
  a one-time runtime state correction, the same class of fix plan 01-01's own
  SUMMARY documents.
- **Files modified:** none.
- **Verification:** `agent-topics` ran cleanly afterward (printed the live ROS
  2 graph's base topics, no `cv_bridge` error); the credential-leak and
  no-`.env` checks above were then verified against a working target.

**Total deviations:** 4 auto-fixed (2 Rule 2/3 fixes necessary to make FOUND-04's
guard test actually runnable and correct inside the container that is this
suite's real execution environment, 1 Rule 1 regex-false-positive fix, 1 Rule 3
runtime-state correction with no file changes). **Impact:** all four were
necessary for the plan's own acceptance criteria to be genuinely verified
(not merely appear green by accident) against the real `run.sh test`
execution path; none changed the plan's scope, architecture, or the D-01/D-02
container-isolation decisions -- the `.env.example` mount is a single,
narrowly-scoped, already-public file, read-only.

## Known Stubs

None introduced by this plan. `run.sh agent` remains the pre-existing stub
from plan 01-01 (Phase 2 work); untouched here.

## Self-Check: PASSED

- All 4 `key-files.created` entries exist on disk and are tracked by git
  (`git ls-files` confirms `config.toml`, `wojtek_rai/config.py`,
  `tests/test_config_loading.py`, `tests/test_no_secrets_in_config.py`).
- `git log --oneline --all --grep="01-03"` -- no commits use that scoped
  prefix in this plan (commit messages here follow the user's global
  `<type>: <description>` convention rather than `type(01-03): ...`); the
  four task-level commits are `5218d14`, `608ff5f`, `e04961f`, `eb1124d`, all
  present in `git log --oneline -6` above HEAD.
- Every task-level `<acceptance_criteria>` from the plan re-run and passing
  (see each Task section above).
- Plan-level `<verification>` items 1-4 all re-run and passing: `./run.sh
  test` exits 0 with `test_config_loading.py` and `test_no_secrets_in_config.py`
  collected, 0 skipped (30/30); the secret scan is fail-first (demonstrated
  and reverted); `run.sh` runs identically with and without a repo-root
  `.env` and printed no credential value; the diff introduced by this plan
  (`git diff 5218d14~1..HEAD -- experiments/wojtek_rai_v1`) contains no real
  dotted-quad address or `user@host` form (the two regex hits found while
  checking this were both false positives of the verification script itself
  -- an un-stripped diff `+` prefix, and this plan's own doc comment
  discussing the `uv.lock` version-string exemption -- not a real value).

## Next

Ready for `01-04-PLAN.md` (isolation boundary, pinned-versions and
compose-override guard tests, README). No blockers.
