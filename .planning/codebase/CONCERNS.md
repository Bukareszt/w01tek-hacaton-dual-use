# Codebase Concerns

**Analysis Date:** 2026-09-05

## Tech Debt

**Generated XML drift risk:**
- Issue: `wojtek_mjx.xml` and `scene_mjx.xml` are generated outputs but are committed to git. If the build step (`./training/run.sh build`) is skipped, stale XML can persist in the working tree and be deployed.
- Files: `ros/src/wojtek_description/mujoco/wojtek_mjx.xml`, `ros/src/wojtek_description/mujoco/scene_mjx.xml`
- Impact: Model or physics changes will not be reflected if build is omitted. Deployment will run with outdated kinematics or environment.
- Fix approach: Add a pre-commit hook or CI gate that regenerates XML and checks for uncommitted changes. Alternatively, move generated files to `.gitignore` and regenerate on every deployment.

**Observation layout schema evolution:**
- Issue: Actor observation size is hardcoded as `OBS_SIZE = 54` in `training/wojtek_rl/env.py`. Any change to the observation layout invalidates all trained checkpoints and deployed policies. Schema-2 metadata carries `obs_layout` but there is no runtime compatibility check in the training code when loading old checkpoints.
- Files: `training/wojtek_rl/env.py:28`, `ros/src/wojtek_policy/wojtek_policy/policy.py:216-220` (validates assembled obs size)
- Impact: Silent checkpoint incompatibility if observation components are added/removed without incrementing a version. Runtime policy load will fail with obs-size mismatch, but training may accept a stale checkpoint if the layout was not checked.
- Fix approach: Add observation layout versioning to policy metadata. At checkpoint restore time, compare the restored obs layout against the current env and fail with a clear message if they mismatch.

**Action filter training/deployment mismatch:**
- Issue: Policies trained with `task.env.action_filter > 0` require the equivalent filter applied in the deployment control loop. The filter is exponential moving average: `filt = af * filt + (1 - af) * raw_action`. If deployment omits or misconfigures this filter, policy behavior changes dramatically.
- Files: `training/wojtek_rl/env.py:159-169` (training config), `ros/src/wojtek_policy/wojtek_policy/policy.py:252-253` (runtime application), `ros/src/wojtek_policy/wojtek_policy/policy_node.py:80-240` (deployment node)
- Impact: Policy trained with filter=0.8 deployed with filter=0.0 will overshoot commands and destabilize gaits. Conversely, filter=0 trained policies deployed with filter>0 will be sluggish.
- Fix approach: Policy schema-2 metadata enforces `action_filter` at load time. Validate that deployment applies the same filter. Add a watchdog in policy_node.py that logs action filter value at startup.

## Known Bugs

**Policy schema version 1 incompatibility:**
- Symptoms: Older policies trained before schema-2 metadata format cannot load on current deployment stack. Error raised at `ros/src/wojtek_policy/wojtek_policy/policy.py:89-97` with instruction to re-export or migrate.
- Files: `ros/src/wojtek_policy/wojtek_policy/policy.py:89-97`, migration tool: `training/wojtek_rl/migrate_keeper_meta.py`
- Trigger: Attempting to deploy a published keeper policy whose meta has `schema_version: 1` (or missing schema_version).
- Workaround: Run `python3 training/wojtek_rl/migrate_keeper_meta.py --meta <old_meta.json>` to regenerate schema-2 metadata. Requires access to the training environment.

**MJWarp version constraint breakage:**
- Symptoms: On GPU with `warp-lang==1.14.0`, `mjx.put_model(impl="warp")` raises `AttributeError: type object 'int' has no attribute 'WARP'`. This is a version-skew issue: warp 1.14 removed `_src.jax_experimental` module that MJX 3.10.0 expects.
- Files: `pyproject.toml` (pins `warp-lang==1.13.0`)
- Trigger: Upgrading warp-lang beyond 1.13.0 or changing MuJoCo/MJX versions without testing.
- Workaround: Respect the pinned `warp-lang==1.13.0` in `pyproject.toml`. Do not upgrade without re-running the MJWarp feasibility test on a GPU box.

## Security Considerations

**Public repository with secret-leakage surface:**
- Risk: Repository is public (`github.com/machinekind/w01-tek`). CLAUDE.md explicitly states: "Never commit credentials or the identity of private infrastructure."
- Files: `.env.example` (documents required secrets), `ros/deploy.sh:44-50` (sources `.env`), `.gitignore` (ensures `.env` is never committed)
- Current mitigation: `.env` files are in `.gitignore`. Secrets are documented in `.env.example` as placeholders only. Secrets enter via environment variables filled from gitignored `.env` at runtime.
- Recommendations: 
  - Add a pre-commit hook that scans for hardcoded IP addresses, hostnames, and common secret patterns (API keys, tokens, AWS access keys).
  - Audit `CLAUDE.md`, job scripts under `training/jobs/`, and deployment scripts (`ros/deploy.sh`) for any accidental leaks of private infrastructure identifiers (partition names, account names, cluster sites).
  - `training/tests/unit/test_job_scripts.py` enforces this for job payloads (FORBIDDEN_RE, SITE_VALUE_RE, ABSOLUTE_PATH_RE). Extend similar checks to all shell scripts under `ros/`.

**Deployment authorization gaps:**
- Risk: CLAUDE.md states: "Remote training jobs and policy deployment stay human-authorized: do not start a remote run, deploy a policy, or launch/arm the physical robot without explicit user authorization."
- Files: `ros/deploy.sh:1-30` (user-facing deployment tool), `ros/src/wojtek_policy/wojtek_policy/policy_node.py:70-90` (policy load)
- Current mitigation: Deploy scripts require manual invocation; there is no autonomous trigger mechanism in the codebase.
- Recommendations: Add a confirmation prompt in `ros/deploy.sh` before syncing to the robot, especially for `--policy` changes. Log all deployments with timestamp and operator identity.

**GPL-licensed IMU firmware in Apache-2.0 repo:**
- Risk: Firmware directories under `ros/src/bmi160_serial_hardware_interface/` and `ros/src/bmx160_serial_hardware_interface/` are GPL-licensed and link GPL AHRS code. Shipping these with Apache-2.0 code may violate GPL copyleft.
- Files: `NOTICE` (licensing exception), `ros/src/bmi160_serial_hardware_interface/LICENSE.md`, `ros/src/bmx160_serial_hardware_interface/LICENSE.md`
- Current mitigation: NOTICE file explicitly carves out these directories from Apache-2.0 licensing. Individual LICENSE.md files in each firmware directory document their GPL status.
- Recommendations: 
  - Do not link GPL code into non-GPL user-facing binaries without careful license compliance review.
  - Keep firmware directories isolated: build/package them separately from `ros/src/wojtek_bringup` and other deployed code.
  - If contributing new code to IMU firmware, respect GPL licensing.

## Performance Bottlenecks

**MJX compilation overhead on first run:**
- Problem: `tests/integration` pay real MJX compile time (6m23s measured cold on a GPU). JAX compilation cache is used to skip recompilation on repeat runs.
- Files: `training/run.sh:43-44` (sets `JAX_COMPILATION_CACHE_DIR`), `training/tests/integration/` (all files)
- Cause: MuJoCo MJX models are JIT-compiled on first load; Warp backend adds additional GPU kernel compilation. Subsequent runs with the same model reuse cached artifacts.
- Improvement path: 
  - Ensure `JAX_COMPILATION_CACHE_DIR` is persisted across CI runs so cache is warm.
  - Pre-compile golden models during CI setup to warm the cache before tests run.
  - Consider splitting slow integration tests into a separate CI pipeline that runs less frequently.

**Warp buffer allocation scaling with batch size:**
- Problem: `njmax` (constraint-row budget per world) cannot scale linearly with batch size. Doing so allocates a dense Jacobian of `njmax * total_dofs`, which is 40 GiB at 1024 envs and crashes.
- Files: `docs/plans/mjwarp-phase0-report.md:72-73` (sizing rules), `training/wojtek_rl/base.py` (data_budget_kwargs)
- Cause: Warp memory layout requires dense Jacobian allocation. Naive scaling exhausts GPU memory.
- Improvement path: `naconmax = 32 * n_envs` and `njmax = 320` are the correct sizing rules (from Phase-0 report). Do not scale njmax beyond 320 even as batch grows; instead, reduce batch size or upgrade GPU memory.

## Fragile Areas

**Policy deployment at robot startup:**
- Files: `ros/src/wojtek_policy/wojtek_policy/policy_node.py:70-90`, `ros/src/wojtek_policy/wojtek_policy/policy_source.py` (policy resolution)
- Why fragile: Robot has no internet. Policy must exist locally in the gitignored policy store (`ros/policies/`). If store is empty or stale, robot cannot load the policy at startup and service fails silently (or with a network error that misleads operators).
- Safe modification: 
  - Always test policy resolution locally before deploying: `python3 -m wojtek_policy.policy_source --default`.
  - Add a startup check in policy_node.py that fails loudly if policy file is missing.
  - Document the policy store path and sync workflow in robot-bringup launch files.

**Test-unit/test-integration split enforcement:**
- Files: `training/tests/unit/` (all files), `training/tests/integration/` (all files), `training/run.sh:43-50` (test commands)
- Why fragile: `tests/unit` must never instantiate an environment or call `mjx.put_model` (enforced by construction, not code). If someone adds a test that loads the full env to `tests/unit`, the entire test suite becomes slow (6m23s instead of 3s). This is enforced by contract, not by a guard test.
- Safe modification: 
  - Add a guard test that scans `tests/unit` for any imports of `env.py`, `env_getup.py`, or `env_jump.py` and fails if found.
  - Document the split in comments at the top of `tests/unit/conftest.py`.

**Observation layout contract between training and deployment:**
- Files: `training/wojtek_rl/env.py:166-178` (obs spec), `ros/src/wojtek_policy/wojtek_policy/policy.py:136-149` (obs layout validation), `ros/src/wojtek_policy/wojtek_policy/policy_node.py:80-90` (joint validation only, not obs)
- Why fragile: Policy node validates `joint_names` at load but does not validate observation layout. If env.py changes observation component names/order and a policy is trained with the old layout, deployment will not catch the mismatch until policy._assemble_obs runs and fails with a size error.
- Safe modification: 
  - At policy export time, include a full observation schema in policy_meta.json (component names, types, ranges).
  - At policy_node startup, validate that runtime sensor observations match the policy's schema and fail with a clear message if they don't.

**Experiments/ interdependency risk:**
- Files: `experiments/autonomous_architecture_ros2_v1/` (entire subtree), `ros/deploy.sh:104` (rsyncs `ros/src/` only, not experiments)
- Why fragile: Code in `experiments/` is unstable by design, but if an experiment depends on and imports code from `ros/src/`, a later refactor of `ros/src/` breaks the experiment silently. The deploy script only touches `ros/src/`, so experiment code can drift out of sync.
- Safe modification: 
  - Enforce no imports of `experiments/` code into `ros/` or `training/` core at import time (Python import guard or CI check).
  - If code in an experiment becomes stable and production-ready, migrate it into `ros/src/` and remove the experiment copy.

## Scaling Limits

**Warp memory ceiling at large batch sizes:**
- Current capacity: RTX 4090 can train at 8192 envs with `naconmax=32*n_envs, njmax=320`.
- Limit: Scaling beyond 8192 envs on RTX 4090 risks OOM when allocating Jacobian matrices. Dense Jacobian scales as `njmax * (num_dofs) * (batch_size)`.
- Scaling path: 
  - Upgrade to a multi-GPU setup (2-GPU pmap is planned; see phase0-report.md:87).
  - Reduce `njmax` if the scenario allows (measure actual nefc_max per terrain with `./training/run.sh check-terrain`).
  - Use a larger GPU (H100 > RTX 4090).

**RPi policy store capacity:**
- Current capacity: RPi has local storage (microSD card, typically 32-128 GB).
- Limit: Policy store (`ros/policies/`) can hold N snapshots of policy.npz + policy_meta.json (~MB-scale each). After N policies, storage fills up.
- Scaling path: 
  - Regularly prune old policy versions from the store.
  - Use symbolic links to point to policies on an external USB drive (if RPi has USB).
  - Implement a cleanup script that removes policies older than X days.

## Dependencies at Risk

**MuJoCo/MJX version ecosystem:**
- Risk: MJX API surface is evolving. mujoco==3.10.0 + mujoco-mjx==3.10.0 + warp-lang==1.13.0 is the tested triple. Upgrading any one of these without testing the MJWarp path will break training.
- Impact: Training hangs, throws cryptic Warp/JAX errors, or silently produces incorrect physics.
- Migration plan: 
  - Before upgrading MuJoCo or warp-lang, run the MJWarp feasibility spike (see phase0-report.md).
  - Test on a GPU box with the new versions. Measure throughput and buffer overflow warnings.
  - Update `pyproject.toml` and `uv.lock` only after passing GPU validation.

**Brax PPO trainer fork (RND variant):**
- Risk: `training/wojtek_rl/ppo_rnd.py` is a vendored fork of brax 0.14.2's `ppo/train.py` with fenced RND insertions. If brax is upgraded, the fork must be re-merged or diverges further.
- Impact: Runs with `rnd.enable=true` will fail or behave incorrectly if brax is upgraded without re-applying the RND patches.
- Migration plan: 
  - Document the RND patches with line numbers and delta comments.
  - If brax is upgraded, re-apply patches manually or use a merge tool.
  - Consider upstreaming RND to brax (future improvement).

**WandB integration hardcoding:**
- Risk: `training/docs/configuration.md:128` shows `wandb.project = fbb-locomotion` as the default. If the WandB project is renamed or deleted, all runs default to the wrong project.
- Impact: Training runs are logged to the wrong project or fail if the project does not exist.
- Migration plan: 
  - Allow `wandb.project` to be overridden at the command line: `./training/run.sh train wandb.project=<new_name>`.
  - Alias the old project name to the new one in WandB (if possible).
  - Document the project migration in a breaking-change note.

## Missing Critical Features

**No observation layout versioning for forward/backward compatibility:**
- Problem: Training env and deployed policy must agree on observation layout. Currently, mismatch is detected only at runtime (in policy._assemble_obs) with a vague size error. There is no way to version the layout and support gradual migration.
- Blocks: Cannot safely change observation components without invalidating all prior checkpoints and deployed policies.
- Recommendation: Add `obs_layout_version` to policy_meta.json. At training time, serialize the full obs layout (names, types, sizes). At deployment, compare against deployed policy's layout and fail if mismatch detected.

**No checkpoint backward-compatibility layer:**
- Problem: Restoring a checkpoint from an older training run may fail if the env config has changed (e.g., different rewards, obs layout, or action scale). There is no automated migration or compatibility check.
- Blocks: Cannot refactor training configs without risking stale checkpoints.
- Recommendation: At checkpoint restore time, compare the checkpointed env config against the current env config and fail loudly if critical fields (obs_layout, action_scale) differ. Provide a migration guide for safe refactoring.

**No RPi policy sync dry-run mode:**
- Problem: `./ros/deploy.sh --policy <ref>` fetches and syncs policies without a preview. If a stale policy is accidentally deployed, the only recovery is another deploy.
- Blocks: Cannot safely test policy changes before committing them to the robot.
- Recommendation: Add `./ros/deploy.sh --policy <ref> --dry-run` to show what would be synced without actually syncing.

## Test Coverage Gaps

**Observation assembly validation:**
- What's not tested: The policy._assemble_obs logic is tested only at the Python level in `ros/src/wojtek_policy/test/test_policy.py`. Integration tests that verify the full sensor→observation→policy pipeline on the real robot are missing.
- Files: `ros/src/wojtek_policy/test/test_policy.py:63-150` (unit tests), missing: real-robot integration test
- Risk: Observation assembly bugs (wrong sensor ordering, unit mismatches) are not caught until deployment.
- Priority: **Medium** — unit tests are comprehensive, but end-to-end validation would catch sensor-pipeline issues earlier.

**Deployment authorization workflow:**
- What's not tested: There is no test that verifies `ros/deploy.sh` prompts for confirmation before syncing policy changes. A malicious or buggy script could silently deploy a dangerous policy.
- Files: `ros/deploy.sh:1-30` (no authorization prompt)
- Risk: Accidental or malicious policy deployment to the robot without operator awareness.
- Priority: **High** — add a confirmation prompt or gate in deploy.sh before any robot sync.

**MJWarp multi-device (pmap) training:**
- What's not tested: Phase-0 report marks 2-GPU pmap as "does not block G0" but it remains untested. Single-device Warp is proven; multi-device is not.
- Files: `docs/plans/mjwarp-phase0-report.md:87` (2-GPU pmap smoke waits for cluster node)
- Risk: If multi-device training is attempted without testing, it may hang, produce incorrect gradients, or fail silently.
- Priority: **Medium** — blocks scaling to 2+ GPUs; needed for very large batch sizes.

---

*Concerns audit: 2026-09-05*
