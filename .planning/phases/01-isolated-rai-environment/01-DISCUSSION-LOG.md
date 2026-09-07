# Phase 1: Isolated RAI Environment - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-09-07
**Phase:** 1-Isolated RAI Environment
**Areas discussed:** Runtime placement, Env + lock tooling, Layout + naming, Remote dev box use

---

## Runtime placement

| Option | Description | Selected |
|--------|-------------|----------|
| Sidecar container | Own Dockerfile + compose in experiment dir, FROM ros:jazzy-ros-core, host network, cyclonedds.xml mounted | |
| Exec into wojtek_robot | Reuse existing sim container; RAI venv on bind-mounted experiment dir | ✓ |
| Host venv + host ROS | Only where ROS Jazzy is native; dev box has none | |
| You decide | Claude picks | |

**User's choice:** Exec into wojtek_robot
**Notes:** Base-image (`ros:jazzy-ros-core + pip`) and DDS (`mount ros/docker/config`) answers were given for the sidecar case and became moot; DDS is inherited from the existing container.

| Option | Description | Selected |
|--------|-------------|----------|
| Compose override file | Experiment ships compose.override.yaml adding the bind mount; ros/docker untouched | ✓ |
| Edit ros/docker/compose.yaml | Breaks roadmap criterion 2 and isolation rule | |
| Switch to sidecar container | Revert to own image | |

**User's choice:** Compose override file

| Option | Description | Selected |
|--------|-------------|----------|
| experiment/.venv on bind mount | Created in-container with python3.12, gitignored, per-arch | ✓ |
| System site-packages | pip --break-system-packages into container | |
| You decide | | |

**User's choice:** experiment/.venv on bind mount

| Option | Description | Selected |
|--------|-------------|----------|
| Assume sim running | run.sh agent starts RAI only | |
| run.sh up starts both | Convenience target calling ros/sim.sh then RAI | ✓ |
| Both targets | agent + up | |

**User's choice:** run.sh up starts both (an `agent` target that assumes the sim is running is kept for criterion 2)

---

## Env + lock tooling

| Option | Description | Selected |
|--------|-------------|----------|
| uv | Matches RAI upstream; universal uv.lock for x86-64 + aarch64 | ✓ |
| pip + pip-tools | Per-platform requirements locks | |
| pip freeze | No resolver, drift risk | |

**User's choice:** uv

| Option | Description | Selected |
|--------|-------------|----------|
| pyproject.toml in experiment | Own Python project, lockfile beside it | ✓ |
| requirements files only | No project metadata | |

**User's choice:** pyproject.toml in experiment

| Option | Description | Selected |
|--------|-------------|----------|
| Lockfile only | Transitive pins in uv.lock; pyproject keeps RAI ranges | ✓ |
| Also pin in pyproject | Duplicate explicit pins | |

**User's choice:** Lockfile only

| Option | Description | Selected |
|--------|-------------|----------|
| vcs .repos at pinned SHA | run.sh build imports + colcon builds into experiment ros_ws | ✓ |
| git submodule | Under experiment/ros/src | |
| apt if exists | Try apt, fall back to vcs | |

**User's choice:** vcs .repos at pinned SHA

---

## Layout + naming

| Option | Description | Selected |
|--------|-------------|----------|
| wojtek_rai_v1 | Wojtek naming, framework named, versioned | ✓ |
| rai_agent_v1 | Framework-first | |
| wojtek_rai | No version suffix | |

**User's choice:** wojtek_rai_v1

| Option | Description | Selected |
|--------|-------------|----------|
| Repo-root .env | Reuse existing root .env + .env.example pattern | ✓ |
| Experiment-local .env | Self-contained second secrets file | |
| Both, local overrides root | Layered | |

**User's choice:** Repo-root .env

| Option | Description | Selected |
|--------|-------------|----------|
| Commit config.toml, keys via env | Vendor/model names committed; keys from env vars; secret-scan test | ✓ |
| Commit config.toml.example, generate local | Real config.toml gitignored | |
| You decide | | |

**User's choice:** Commit config.toml, keys via env

| Option | Description | Selected |
|--------|-------------|----------|
| Mirror sibling | README, run.sh, pyproject, uv.lock, config.toml, docker/, wojtek_rai/, ros/, tests/, docs/ | ✓ |
| Flat minimal | run.sh, pyproject, package, tests | |
| You decide | | |

**User's choice:** Mirror sibling

---

## Remote dev box use

| Option | Description | Selected |
|--------|-------------|----------|
| Executors may run install/tests | Standing authorization: sync repo, docker compose, uv sync, run.sh on the dev box; never the robot | ✓ |
| Only after asking each time | Checkpoint before any remote command | |
| Only you, manually | Executor prints commands | |

**User's choice:** Executors may run install/tests

| Option | Description | Selected |
|--------|-------------|----------|
| git clone + push/pull | Dev box has its own checkout; verify from committed state | ✓ |
| rsync working tree | Fast iteration, uncommitted state | |
| You decide | | |

**User's choice:** git clone + push/pull

| Option | Description | Selected |
|--------|-------------|----------|
| Same 3 commands pass on both | install, test, topic discovery on laptop and dev box; outputs into VERIFICATION.md | ✓ |
| Laptop full, dev box install+test only | | |
| Dev box only | | |

**User's choice:** Same 3 commands pass on both

| Option | Description | Selected |
|--------|-------------|----------|
| Yes, same compose | ros/dev.sh + override on the dev box; arm64 image from ros:jazzy-ros-core | ✓ |
| Headless variant | Skip X11/RViz | |
| You decide | | |

**User's choice:** Yes, same compose

## Claude's Discretion

- Extra run.sh target names; isolation-test mechanics; uv install method and cache location; exact rai_interfaces SHA; README status wording.

## Deferred Ideas

- Sidecar RAI container with own image
- Experiment-local .env
- apt-installed rai_interfaces
