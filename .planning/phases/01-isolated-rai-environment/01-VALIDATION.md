---
phase: 01
slug: isolated-rai-environment
# status lifecycle: draft (seeded by plan-phase) → validated (set by validate-phase §6)
# audit-milestone §5.5 distinguishes NOT-VALIDATED (draft) from PARTIAL (validated + nyquist_compliant: false) (#2117)
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-09-07
---

# Phase 01 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest >=8.0.0 (repo convention, `training/pyproject.toml`) |
| **Config file** | none — Wave 0 installs (`experiments/wojtek_rai_v1/run.sh test` wraps `"$PY" -m pytest tests -q`, same shape as the sibling experiment) |
| **Quick run command** | `./experiments/wojtek_rai_v1/run.sh test` |
| **Full suite command** | `./experiments/wojtek_rai_v1/run.sh test` (no slow/integration split this phase) |
| **Estimated runtime** | ~5 seconds |

---

## Sampling Rate

- **After every task commit:** Run `./experiments/wojtek_rai_v1/run.sh test`
- **After every plan wave:** Run `./experiments/wojtek_rai_v1/run.sh test`
- **Before `/gsd-verify-work`:** Full suite green on the x86-64 laptop, then re-run on the aarch64 dev box (D-16/D-17)
- **Max feedback latency:** 30 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 01-TBD | TBD | 0 | FOUND-01 | — | Nothing outside `experiments/` imports the experiment; `ros/deploy.sh` cannot ship it | unit (grep guard) | `pytest experiments/wojtek_rai_v1/tests/test_isolation_boundary.py -x` | ❌ W0 | ⬜ pending |
| 01-TBD | TBD | 0 | FOUND-02 | — | N/A | unit | `pytest experiments/wojtek_rai_v1/tests/test_pinned_versions.py -x` | ❌ W0 | ⬜ pending |
| 01-TBD | TBD | 0 | FOUND-03 | — | N/A (static half); live topic list is manual | unit (config shape) | `pytest experiments/wojtek_rai_v1/tests/test_compose_override.py -x` | ❌ W0 | ⬜ pending |
| 01-TBD | TBD | 0 | FOUND-04 | T-01-secrets | No secret-shaped values in tracked config; keys only from gitignored `.env` | unit | `pytest experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py -x` | ❌ W0 | ⬜ pending |
| 01-TBD | TBD | 0 | FOUND-06 | — | Test suite needs no LLM key, ROS runtime, or GPU | unit (guard) | `pytest experiments/wojtek_rai_v1/tests -q` | ❌ W0 | ⬜ pending |
| 01-TBD | TBD | — | FOUND-07 | — | N/A | manual | `run.sh install && run.sh test && run.sh agent-topics` on each machine | N/A | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

*Task IDs are filled in by the planner; the planner must map every task to a row here or to a Wave 0 dependency.*

---

## Wave 0 Requirements

- [ ] `experiments/wojtek_rai_v1/tests/test_isolation_boundary.py` — stubs for FOUND-01
- [ ] `experiments/wojtek_rai_v1/tests/test_pinned_versions.py` — stubs for FOUND-02
- [ ] `experiments/wojtek_rai_v1/tests/test_compose_override.py` — stubs for FOUND-03 (static-config half)
- [ ] `experiments/wojtek_rai_v1/tests/test_no_secrets_in_config.py` — stubs for FOUND-04
- [ ] `experiments/wojtek_rai_v1/tests/conftest.py` — shared fixtures (repo root path, experiment dir)
- [ ] `experiments/wojtek_rai_v1/run.sh` + `pyproject.toml` + `uv.lock` — framework install: `pytest` as this experiment's own direct dev dependency (D-07); `run.sh test` target

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| RAI process lists live sim topics (`cmd_vel`, camera) | FOUND-03 | Needs a running `ros/sim.sh` and ROS runtime; unit tests are model-free/ROS-free by contract | Start `./ros/sim.sh`; run `./experiments/wojtek_rai_v1/run.sh agent-topics`; confirm `cmd_vel` and the camera topic appear in output |
| Install + test + topic discovery succeed on both machines | FOUND-07 | aarch64 dev box not reachable from a single CI run | On laptop and on the dev box: `run.sh install && run.sh test && run.sh agent-topics`; record results in VERIFICATION.md (D-16) |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 30s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
