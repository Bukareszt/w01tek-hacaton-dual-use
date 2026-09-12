#!/usr/bin/env bash
# Entry point for the tower-camera follow experiment.  Self-contained on
# purpose: nothing outside this directory is configured to know it exists.
# See README.md.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$PWD"
PKG="$HERE/ros/src/wojtek_follow"
# The gimbal's message package lives in the targeting experiment.  An
# experiment may depend on another experiment; nothing in ros/src may.
TARGETING="$HERE/../wojtek_targeting/ros/src"

# The tests need numpy and pytest and nothing else.  There is no venv in a
# worktree, so allow an explicit interpreter:
#   EXP_PY=/path/to/python ./run.sh test
PY="${EXP_PY:-python3}"

case "${1:-}" in
  test)
    shift
    echo ">> wojtek_follow (rclpy-free core, plus the node over stubbed ROS)"
    PYTHONPATH="$PKG${PYTHONPATH:+:$PYTHONPATH}" exec "$PY" -m pytest "$PKG/test" -q "$@"
    ;;
  build)
    # colcon build for this experiment's ROS package only.  It lives outside
    # ros/src so that ros/deploy.sh, which rsyncs ros/src to the robot, can
    # never ship it; that also means ros/sim.sh does not build it, hence this
    # target.  Run it in an environment that has ROS 2 sourced.
    shift
    command -v colcon >/dev/null 2>&1 || {
      echo "colcon not found: source a ROS 2 setup.bash first" >&2
      exit 1
    }
    paths=("$PKG")
    if [ -d "$TARGETING" ]; then
      for pkg in "$TARGETING"/*/; do
        [ -f "$pkg/package.xml" ] && paths+=("${pkg%/}")
      done
    else
      echo "note: $TARGETING is not here, so wojtek_targeting_msgs will not" >&2
      echo "      resolve. Check out the targeting experiment beside this one." >&2
    fi
    exec colcon build --paths "${paths[@]}" --packages-up-to wojtek_follow "$@"
    ;;
  sim)
    # Into the simulation's container (./ros/sim.sh must be up): copy this
    # experiment in, build it there, and launch the node with the simulation
    # preset. The container mounts ros/src and nothing else of the
    # repository, which is the point of experiments/, so the copy is how the
    # package gets in. See README.md, "Running it".
    shift
    docker exec wojtek_robot rm -rf /ros2_ws/experiments/wojtek_follow_v1
    docker exec wojtek_robot mkdir -p /ros2_ws/experiments
    docker cp "$HERE" wojtek_robot:/ros2_ws/experiments/wojtek_follow_v1
    docker exec wojtek_robot bash -c '
      source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
      source /ros2_ws/install/setup.bash
      cd /ros2_ws/experiments/wojtek_follow_v1/ros
      colcon build --packages-select wojtek_follow --event-handlers status- desktop_notification-'
    exec docker exec -it wojtek_robot bash -c '
      source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
      source /ros2_ws/install/setup.bash
      source /ros2_ws/experiments/wojtek_follow_v1/ros/install/setup.bash
      share=$(ros2 pkg prefix --share wojtek_follow)
      exec ros2 launch wojtek_follow follow.launch.py params_file:=$share/config/follow_sim.yaml "$@"' _ "$@"
    ;;
  *)
    echo "usage: run.sh {test|build|sim} [args]"
    echo "  test   model-free unit tests (no ROS, no robot, no cameras)"
    echo "  build  colcon build of this experiment's ROS package"
    echo "  sim    build and launch inside the ./ros/sim.sh container, gimbal fixed"
    exit 1
    ;;
esac
