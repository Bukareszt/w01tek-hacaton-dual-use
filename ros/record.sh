#!/usr/bin/env bash
# Record RGB + RGBD from the LIVE robot, on demand, from the PC.
#
#   ./record.sh start [name]    turn depth on in the running camera driver and
#                               start a topic-selective rosbag on the RPi
#   ./record.sh status          is it recording, how big, how much disk is left
#   ./record.sh stop            stop the recorder, put the camera back the way
#                               it was (colour-only for the Deck)
#   ./record.sh pull [name]     rsync the bag to ./bags/<name>/ (gitignored)
#   ./record.sh list            bags on the robot
#
# Why a script and not bag:=true: the service's always-on `ros2 bag record -a`
# (400 Hz introspection to the SD card) starved the Pi and is off in
# deploy/rpi/wojtek-robot-local.conf. This records a short list of topics
# instead, pinned to the non-RT cores like every other helper on the robot,
# and never touches the control stack: the robot stays armed throughout.
#
# The camera the Deck runs (deck_camera:=true) is colour-only. `start` flips
# depth, depth->colour alignment and frame sync on in the running driver with
# `ros2 param set` (the driver restarts its sensors, the Deck view blips for a
# second) and lowers colour to 15 fps so colour and aligned depth pair 1:1.
# `stop` restores every value it changed from the copy it saved on `start`.
# The switches go on one at a time with a liveness check in between: the
# full perception config once segfaulted this driver on the Pi, and a dead
# driver blinds the Deck until the service restarts (which drops the robot).
# If `start` reports the driver died, disarm and lie down before that.
#
# What lands in the bag, and why these and not the raw streams:
#   /camera/camera/color/image_raw/compressed         RGB, JPEG, ~2 MB/s
#   /camera/camera/color/camera_info
#   /camera/camera/aligned_depth_to_color/image_raw   RGBD depth half, 16UC1
#   /camera/camera/aligned_depth_to_color/camera_info   640x480x15 = ~9 MB/s
#   /camera/camera/depth/camera_info, .../extrinsics/depth_to_color
#   /tf /tf_static /joint_states /wojtek/joint_states_abs /wojtek/joint_targets
#   /cmd_vel /joy /imu_sensor_broadcaster/imu
# The SD card writes ~28 MB/s. Raw colour alone at 640x480x30 is ~28 MB/s
# and the driver's combined RGBD message (~1.5 MB per frame) another ~23, so
# neither fits next to the control stack; the aligned-depth + colour pair
# shares one timestamp per frame (enable_sync) and IS the RGBD product.
# Add more with RECORD_TOPICS_EXTRA="/a /b" (e.g. the raw depth
# /camera/camera/depth/image_rect_raw, another ~9 MB/s).
#
# Bags are mcap with fast zstd chunks (depth compresses ~2-3x), split every
# 2 minutes so a session that ends badly still leaves readable files, and
# the recorder stops itself when the card has under 1.5 GB left. Output:
# ~/wojtek_bags/<name>/ on the RPi; `pull` brings it to ./bags/<name>/.
#
#   RPI_HOST=rpi@10.42.0.2 ./record.sh start    # override the anchor
#
# Nothing about a private machine is written here; see CLAUDE.md.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

# Local overrides (RPI_HOST, ...); never committed.
if [ -f "${HERE}/.env" ]; then
    set -a; . "${HERE}/.env"; set +a
fi

RPI_HOST="${RPI_HOST:-rpi@10.42.0.2}"      # static anchor; same over eth or the RPi's AP
REMOTE_BAGS="${REMOTE_BAGS:-wojtek_bags}"   # relative to the RPi user's home
CAMERA_NODE="${CAMERA_NODE:-/camera/camera}"
MIN_FREE_MB="${MIN_FREE_MB:-1500}"
RECORD_TOPICS_EXTRA="${RECORD_TOPICS_EXTRA:-}"
SSH_OPTS=(-o ConnectTimeout=6 -o BatchMode=yes)

TOPICS=(
    "${CAMERA_NODE}/color/image_raw/compressed"
    "${CAMERA_NODE}/color/camera_info"
    "${CAMERA_NODE}/aligned_depth_to_color/image_raw"
    "${CAMERA_NODE}/aligned_depth_to_color/camera_info"
    "${CAMERA_NODE}/depth/camera_info"
    "${CAMERA_NODE}/extrinsics/depth_to_color"
    /tf /tf_static
    /joint_states /wojtek/joint_states_abs /wojtek/joint_targets
    /cmd_vel /joy /imu_sensor_broadcaster/imu
)
# shellcheck disable=SC2206
TOPICS+=(${RECORD_TOPICS_EXTRA})

# Camera values `start` sets, as key=value. `stop` restores whatever `start`
# read first. Order matters: profiles while the streams are off, then the
# switches; `stop` walks the list backwards.
CAM_SET=(
    depth_module.depth_profile=640x480x15
    rgb_camera.color_profile=640x480x15
    enable_depth=true
    align_depth.enable=true
    enable_sync=true
)

usage() { sed -n '2,46p' "$0" | sed 's/^# \{0,1\}//'; }

# Everything on the robot runs through this. The service's environment
# (domain 42, CycloneDDS, the pinned interface profile) is not in a
# non-interactive ssh shell, so the remote script sets it, and the shell
# lands on the non-isolated cores by itself (isolcpus keeps 2,3 for the
# control loop). Arguments: <cmd> ... [-- key=value ... [-- topic ...]].
remote() {
    local q=""
    local a; for a in "$@"; do q+=" $(printf '%q' "$a")"; done
    ssh "${SSH_OPTS[@]}" "${RPI_HOST}" "bash -s --${q}" <<'EOF'
set -eo pipefail
# ROS's setup files read variables they never set; -u only after them.
source /opt/ros/jazzy/setup.bash
source "$HOME/wojtek_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds-rpi.xml
cmd="$1"; shift
bags="$HOME/wojtek_bags"

case "$cmd" in
start)
    name="$1"; node="$2"; min_free="$3"; shift 3
    [ "$1" = "--" ] && shift
    settings=(); while [ $# -gt 0 ] && [ "$1" != "--" ]; do settings+=("$1"); shift; done
    [ "${1:-}" = "--" ] && shift
    topics=("$@")
    dir="$bags/$name"
    mkdir -p "$bags"
    if [ -e "$dir" ]; then echo "!! $dir exists, pick another name" >&2; exit 2; fi
    if [ -f "$bags/.recording" ]; then
        echo "!! already recording $(cat "$bags/.recording"); stop it first" >&2; exit 2
    fi
    if ! ros2 node list 2>/dev/null | grep -qx "$node"; then
        echo "!! camera node $node is not running (deck_camera:=true in the service?)" >&2; exit 2
    fi
    # The driver is the Deck's eyes while the robot is armed, and the full
    # perception config once crashed it on this Pi (see the deck_camera
    # comment in wojtek_bringup/launch_common.py; which switch did it was
    # never pinned down). So: one switch at a time, and after each one the
    # driver process must still be there, else stop here and say so.
    cam_pid="$(pgrep -of realsense2_camera_node || true)"
    [ -n "$cam_pid" ] || { echo "!! no realsense2_camera_node process" >&2; exit 2; }
    # Remember what the driver runs with now; stop restores it.
    : > "$bags/$name.params"
    for kv in "${settings[@]}"; do
        key="${kv%%=*}"; val="${kv#*=}"
        cur="$(ros2 param get "$node" "$key" 2>/dev/null | sed -n 's/^.* value is: //p')"
        printf '%s %s\n' "$key" "$cur" >> "$bags/$name.params"
        echo ">> $key: $cur -> $val"
        ros2 param set "$node" "$key" "$val" >/dev/null
        sleep 2
        if ! kill -0 "$cam_pid" 2>/dev/null; then
            echo "!! the camera driver died after $key=$val. The Deck has no" >&2
            echo "!! camera until the service restarts (that drops the robot:" >&2
            echo "!! disarm and lie down first): sudo systemctl restart wojtek-robot" >&2
            exit 5
        fi
    done
    # Give the sensors a moment to come back and the new topics to be offered.
    want="$node/aligned_depth_to_color/image_raw"
    for _ in $(seq 1 30); do
        ros2 topic list 2>/dev/null | grep -qx "$want" && break
        sleep 0.5
    done
    if ! ros2 topic list 2>/dev/null | grep -qx "$want"; then
        echo "!! $want never appeared; check: journalctl -u wojtek-robot -e" >&2; exit 3
    fi
    # The recorder outlives this ssh session. SIGINT is how it is stopped
    # (mcap files get finalised); the guard next to it sends that when the
    # card runs low.
    setsid nohup taskset -c 0,1 ros2 bag record -o "$dir" -s mcap \
        --storage-preset-profile zstd_fast --max-bag-duration 120 \
        "${topics[@]}" > "$bags/$name.log" 2>&1 &
    pid=$!
    echo "$pid" > "$bags/$name.pid"
    echo "$name" > "$bags/.recording"
    setsid nohup bash -c '
        pid="$1"; bags="$2"; min_free="$3"
        while kill -0 "$pid" 2>/dev/null; do
            free_mb=$(df -Pm "$bags" | awk "NR==2{print \$4}")
            if [ "$free_mb" -lt "$min_free" ]; then
                echo ">> guard: ${free_mb} MB free < ${min_free} MB, stopping the recorder"
                kill -INT "$pid"; break
            fi
            sleep 5
        done
    ' guard "$pid" "$bags" "$min_free" >> "$bags/$name.log" 2>&1 &
    sleep 4
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "!! recorder died at start:" >&2; cat "$bags/$name.log" >&2; exit 4
    fi
    echo ">> recording $name -> $dir"
    grep -c "Subscribed to topic" "$bags/$name.log" | sed 's/^/>> topics subscribed: /'
    grep -v "Subscribed to topic" "$bags/$name.log" | tail -n 5
    ;;
status)
    if [ -f "$bags/.recording" ]; then
        name="$(cat "$bags/.recording")"
        if kill -0 "$(cat "$bags/$name.pid")" 2>/dev/null; then
            echo ">> recording $name"
        else
            echo ">> $name: recorder is NOT running (the disk guard stopped it, or it died; see the log)"
        fi
        du -sh "$bags/$name" 2>/dev/null || true
        ls -la "$bags/$name" 2>/dev/null | tail -n 4
        echo ">> last log lines:"; grep -v "Subscribed to topic" "$bags/$name.log" | tail -n 6
    else
        echo ">> not recording"
    fi
    echo ">> free on the card:"; df -h "$bags" | tail -n 1
    uptime
    ;;
stop)
    node="$1"
    if [ ! -f "$bags/.recording" ]; then echo ">> nothing is recording"; exit 0; fi
    name="$(cat "$bags/.recording")"; pid="$(cat "$bags/$name.pid")"
    if kill -0 "$pid" 2>/dev/null; then
        kill -INT "$pid"
        for _ in $(seq 1 40); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
        kill -0 "$pid" 2>/dev/null && { echo "!! recorder did not exit on SIGINT, killing" >&2; kill -KILL "$pid"; }
    fi
    rm -f "$bags/.recording"
    # Back to what the Deck was running with, switches first, profiles last.
    tac "$bags/$name.params" | while read -r key val; do
        [ -n "$val" ] || continue
        echo ">> $key -> $val"
        ros2 param set "$node" "$key" "$val" >/dev/null || echo "!! could not restore $key" >&2
    done
    echo ">> stopped $name"
    du -sh "$bags/$name"
    ros2 bag info "$bags/$name" 2>/dev/null | sed -n '1,40p' || true
    ;;
list)
    ls -la "$bags"
    ;;
*)
    echo "unknown remote command: $cmd" >&2; exit 1
    ;;
esac
EOF
}

cmd="${1:-}"; [ $# -gt 0 ] && shift
case "$cmd" in
start)
    name="${1:-rgbd_$(date +%Y%m%d_%H%M%S)}"
    remote start "$name" "$CAMERA_NODE" "$MIN_FREE_MB" -- "${CAM_SET[@]}" -- "${TOPICS[@]}"
    echo ">> stop with:  $0 stop"
    echo ">> pull with:  $0 pull $name"
    ;;
status) remote status ;;
stop)   remote stop "$CAMERA_NODE" ;;
list)   remote list ;;
pull)
    name="${1:-}"
    if [ -z "$name" ]; then
        name="$(ssh "${SSH_OPTS[@]}" "${RPI_HOST}" "ls -t ~/${REMOTE_BAGS} | grep -v '\.' | head -n 1")"
    fi
    [ -n "$name" ] || { echo "no bag to pull" >&2; exit 1; }
    mkdir -p "${HERE}/bags"
    rsync -a --info=progress2 "${RPI_HOST}:${REMOTE_BAGS}/${name}/" "${HERE}/bags/${name}/"
    echo ">> ${HERE}/bags/${name}"
    ls -la "${HERE}/bags/${name}"
    ;;
-h|--help|"") usage ;;
*) echo "unknown command: $cmd" >&2; usage >&2; exit 1 ;;
esac
