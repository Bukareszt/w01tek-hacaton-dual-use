#!/usr/bin/env bash
# Entry point for the RAI (RobotecAI) integration experiment. Self-contained
# on purpose: nothing outside this directory is configured to know about the
# experiment, so every path it needs is set up here. See README.md.
#
# Container lifecycle decision (Phase 1 plan 01-01, Task 1 checkpoint,
# option-a): this script owns bringing up `wojtek_robot` itself -- with the
# experiment's compose override always attached -- rather than relying on
# ros/sim.sh or ros/dev.sh to have started it with the override already in
# place. Neither of those scripts has an extension point for an extra
# compose file, and each recreates the container on its own `up -d`, so
# whichever script runs last would silently drop the other's compose stack.
# `container`, below, replicates the exact platform-detection branch
# ros/sim.sh and ros/dev.sh already use, so the result is identical to what
# they would produce, plus this experiment's bind mount. ros/sim.sh,
# ros/dev.sh and ros/docker/ are never edited to make this work.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$PWD"

# Path to this experiment as seen from *inside* the wojtek_robot container,
# via the bind mount docker/compose.override.yaml adds.
CONTAINER_EXP_DIR="/ros2_ws/experiments/wojtek_rai_v1"

# `test`'s interpreter escape hatch (matches the sibling experiment's
# EXP_PY): an explicit host interpreter that already has pytest, for a
# worktree/CI environment with no venv and no Docker at all.
#   EXP_PY=/path/to/python ./run.sh test

usage() {
  cat >&2 <<'USAGE'
usage: run.sh {install|build|test|container|agent|agent-topics|up} [args]
  install       pinned venv inside the wojtek_robot container (uv + uv.lock)
  build         colcon build of this experiment's own ROS packages (plan 01-02)
  test          model-free unit tests -- no ROS runtime, no LLM key, no GPU
  container     bring up wojtek_robot with this experiment's bind mount, idempotently
  agent         run the RAI agent process (not implemented until Phase 2)
  agent-topics  print the running simulation's live ROS 2 topic list
  up            start the sim (ros/sim.sh's session) with this experiment mounted
USAGE
}

# ---- container lifecycle ---------------------------------------------------

container() {
  # Replicates ros/sim.sh's / ros/dev.sh's own platform-detection branch
  # (uname -s / docker info nvidia runtime), so the stack this experiment
  # brings up is identical to what those scripts would produce, plus this
  # experiment's bind mount. Run from ros/docker/ so the base compose file's
  # own relative volume sources (../src, ./config/...) keep resolving,
  # exactly as ros/dev.sh does with its own `cd "$(dirname "$0")/docker"`.
  local ros_docker
  ros_docker="$(cd "$HERE/../../ros/docker" && pwd)"

  local compose=(docker compose)
  local base_added=false
  if [ "$(uname -s)" = "Darwin" ]; then
    compose+=(-f compose.yaml -f compose.mac.yaml)
    base_added=true
  elif docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
    compose+=(-f compose.yaml -f compose.gpu.yaml)
    base_added=true
  fi
  if ! $base_added; then
    compose+=(-f compose.yaml)
  fi
  compose+=(-f "$HERE/docker/compose.override.yaml")

  (cd "$ros_docker" && "${compose[@]}" up -d --remove-orphans)
}

# Sources ROS 2 plus this experiment's own colcon overlay (once built), cds
# into the experiment root (RAI's config.toml load is cwd-relative -- see
# RESEARCH.md Pitfall 5), then execs the venv's python inside the running
# container. One place, used by every container-side Python invocation
# (RESEARCH.md Pattern 2 + Pattern 4 combined), so `agent`/`agent-topics`
# never duplicate this sourcing order.
container_py() {
  docker exec -i wojtek_robot bash -s <<PYEOF
# -u (nounset) deliberately not set: /opt/ros/jazzy/setup.bash references
# unset variables internally (e.g. AMENT_TRACE_SETUP_FILES) -- matches
# ros/sim.sh's and ros/dev.sh's own "set -eo pipefail" for the same reason.
set -eo pipefail
EXP_DIR="$CONTAINER_EXP_DIR"
export UV_INSTALL_DIR="\$EXP_DIR/.tools"
export PATH="\$UV_INSTALL_DIR:\$PATH"
source /opt/ros/jazzy/setup.bash
[ -f "\$EXP_DIR/ros_ws/install/setup.bash" ] && source "\$EXP_DIR/ros_ws/install/setup.bash"
cd "\$EXP_DIR"
# [Rule 1 deviation] The venv is --system-site-packages (D-03/Pattern 2), so
# it also sees the container's apt-installed pytest plugins (e.g.
# ros-jazzy-launch-testing's launch_testing entry point). Those were built
# against the system pytest, not the newer pinned pytest this venv's uv.lock
# resolved, and pytest's setuptools-entrypoint plugin autoload then fails
# with a PluginValidationError before a single test runs. Disabling
# autoload is exactly targeted: it does not affect this suite (no ROS
# pytest plugin is needed for model-free tests) and does not affect
# rclpy/cv_bridge availability (that is PYTHONPATH, unrelated to pytest's
# own plugin discovery).
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec .venv/bin/python $*
PYEOF
}

case "${1:-}" in
  install)
    shift
    container
    # Runs entirely inside the running container via stdin (docker exec -i
    # ... bash -s), so no host-side quoting of container-side variables is
    # needed. UV_INSTALL_DIR/UV_CACHE_DIR keep uv itself and its cache under
    # this experiment's own bind mount (D-06) -- nothing baked into the
    # image, nothing outside the experiment directory.
    docker exec -i wojtek_robot bash -s <<'INSTALL'
# -u (nounset) deliberately not set -- see the comment in container_py()
# above; /opt/ros/jazzy/setup.bash is not nounset-safe.
set -eo pipefail
EXP_DIR=/ros2_ws/experiments/wojtek_rai_v1
cd "$EXP_DIR"

export UV_INSTALL_DIR="$EXP_DIR/.tools"
export UV_CACHE_DIR="$EXP_DIR/.uv-cache"

# 1. uv, fetched at runtime (D-06) -- never baked into the image. Guarded so
#    a re-run (e.g. after an interruption) never re-fetches needlessly.
if [ ! -x "$UV_INSTALL_DIR/uv" ]; then
  echo ">> fetching uv into $UV_INSTALL_DIR"
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="$UV_INSTALL_DIR:$PATH"

# 2. cv_bridge -- rai/tools/ros2/generic/topics.py imports it at module
#    scope (RESEARCH.md Pitfall 2); ros:jazzy-ros-core does not ship it and
#    ros/docker/Dockerfile is never touched to add it. The container's
#    default user is root, so call apt-get directly and only prefix sudo
#    when it is not.
if ! dpkg -s ros-jazzy-cv-bridge >/dev/null 2>&1; then
  echo ">> installing ros-jazzy-cv-bridge into the running container"
  SUDO=""
  [ "$(id -u)" -ne 0 ] && SUDO=sudo
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends ros-jazzy-cv-bridge
fi

# 3. venv: --system-site-packages so it can see rclpy/cv_bridge once ROS 2
#    is sourced (RESEARCH.md Pattern 2), built with the container's own
#    python3 -- never a hardcoded version (RESEARCH assumption A1).
source /opt/ros/jazzy/setup.bash
PYVER=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
case "$PYVER" in
  3.10|3.11|3.12) ;;
  *)
    echo "!! container python3 is $PYVER, outside >=3.10,<3.13 required by rai-core/rai-whoami" >&2
    exit 1
    ;;
esac
if [ ! -d .venv ]; then
  uv venv --system-site-packages .venv
fi

# 4. Pinned RAI stack + this experiment's own dev deps. A routine install
#    (uv.lock already present) never re-resolves (RESEARCH.md Pitfall 1) --
#    only the very first install writes the lockfile.
if [ -f uv.lock ]; then
  uv sync --frozen
else
  uv sync
  uv lock
fi
echo ">> install complete: $EXP_DIR/.venv"
INSTALL
    ;;

  build)
    echo "build: not implemented yet -- see plan 01-02 (rai_interfaces via vcs + colcon)" >&2
    exit 1
    ;;

  test)
    shift
    if [ -n "${EXP_PY:-}" ]; then
      PY="$EXP_PY"
      { [ -x "$PY" ] || command -v "$PY" >/dev/null 2>&1; } || {
        echo "!! EXP_PY=$PY is not executable" >&2
        exit 1
      }
      if ! "$PY" -c 'import pytest' >/dev/null 2>&1; then
        echo "!! $PY has no pytest -- run ./run.sh install first" >&2
        exit 1
      fi
      exec "$PY" -m pytest tests -q "$@"
    fi
    # No EXP_PY override: run inside the container, where the pinned venv
    # this experiment installed actually lives. Deliberately NOT
    # `$HERE/.venv/bin/python` run directly from the host: that path is a
    # symlink to an absolute system interpreter path (e.g. /usr/bin/python3)
    # recorded when the venv was created *inside* the container -- on a host
    # that happens to also have a binary at that same absolute path (as this
    # one does), the symlink silently resolves to a completely different,
    # unrelated interpreter with its own unrelated site-packages, so the
    # test run would prove nothing about this experiment's actual pinned
    # install. Running inside the container is the only way `test` verifies
    # what `install` produced.
    container
    if ! docker exec wojtek_robot test -x "$CONTAINER_EXP_DIR/.venv/bin/python"; then
      echo "!! no venv at $CONTAINER_EXP_DIR/.venv -- run ./run.sh install first" >&2
      exit 1
    fi
    container_py -m pytest tests -q "$@"
    ;;

  container)
    shift
    container
    ;;

  agent)
    echo "agent: not implemented until Phase 2 (ReAct agent + tool registry)" >&2
    exit 1
    ;;

  agent-topics)
    shift
    container
    container_py -m wojtek_rai.topics
    ;;

  up)
    shift
    # Task 1 decision, option-a: this experiment starts the sim itself
    # rather than delegating to ros/sim.sh, since neither ros/sim.sh nor
    # ros/dev.sh has an extension point for this experiment's compose
    # override. Same session command ros/sim.sh execs, run against the
    # container `container` just brought up (with the override attached).
    container
    exec docker exec -it wojtek_robot bash -c '
      source /opt/ros/jazzy/setup.bash
      source /ros2_ws/install/setup.bash
      exec ros2 run wojtek_bringup robot --sim --foxglove'
    ;;

  *)
    usage
    exit 1
    ;;
esac
