#!/usr/bin/env bash
# Record RGB + RGBD from the LIVE robot, on demand, from the PC.
#
#   ./record.sh start [name]    turn depth on in the camera driver and start a
#                               topic-selective rosbag on the RPi
#   ./record.sh status          is it recording, how big, how much disk is left
#   ./record.sh stop            stop the recorder, put the camera back the way
#                               it was (colour-only for the Deck)
#   ./record.sh pull [name]     rsync the bag to ./bags/<name>/ (gitignored)
#   ./record.sh list            bags on the robot
#   ./record.sh camera          (recovery) start a colour-only camera node for
#                               the Deck after the service's one died
#
# Why a script and not bag:=true: the service's always-on `ros2 bag record -a`
# (400 Hz introspection to the SD card) starved the Pi and is off in
# deploy/rpi/wojtek-robot-local.conf. This records a short list of topics
# instead, pinned to the non-RT cores like every other helper on the robot,
# and never touches the control stack: the robot stays armed throughout.
#
# The camera the Deck runs (deck_camera:=true) is colour-only. `start`
# reconfigures the running driver with `ros2 param set`: infra streams off,
# depth and colour to 640x480x15 so they pair 1:1, then depth, depth->colour
# alignment and frame sync on. One switch at a time, and after each one a
# frame has to actually arrive (the D435's depth module carries depth AND
# the two infra streams: with mismatched profiles it opens fine and then
# delivers nothing, and the next sensor restart segfaulted the driver --
# seen 2026-09-12 on the robot). The driver is the Deck's eyes while the
# robot is armed, and the launch does not respawn it, so if it dies:
# `./record.sh camera` starts a colour-only one by hand, no service restart.
# `start` does the same when it finds no camera node at all. `stop`
# restores every value it changed from the copy it saved on `start`.
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
# RECORD_ALIGN=0 skips alignment and sync and records the raw depth
# (/camera/camera/depth/image_rect_raw, same size) to align offline from
# the two camera_infos and the extrinsics -- the fallback if alignment
# turns out to be what kills the driver. Add topics with
# RECORD_TOPICS_EXTRA="/a /b".
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
RECORD_ALIGN="${RECORD_ALIGN:-1}"
RECORD_TOPICS_EXTRA="${RECORD_TOPICS_EXTRA:-}"
SSH_OPTS=(-o ConnectTimeout=6 -o BatchMode=yes)

TOPICS=(
    "${CAMERA_NODE}/color/image_raw/compressed"
    "${CAMERA_NODE}/color/camera_info"
    "${CAMERA_NODE}/depth/camera_info"
    "${CAMERA_NODE}/extrinsics/depth_to_color"
    /tf /tf_static
    /joint_states /wojtek/joint_states_abs /wojtek/joint_targets
    /cmd_vel /joy /imu_sensor_broadcaster/imu
)

# Camera values `start` sets, as key=value, in this order; `stop` restores
# whatever `start` read first, walking the list backwards. After an entry
# marked "key=value@topic" a message must arrive on that topic.
CAM_SET=(
    enable_infra1=false
    enable_infra2=false
    depth_module.depth_profile=640x480x15
    rgb_camera.color_profile=640x480x15
    "enable_depth=true@${CAMERA_NODE}/depth/image_rect_raw"
)
if [ "$RECORD_ALIGN" = "1" ]; then
    CAM_SET+=(
        "align_depth.enable=true@${CAMERA_NODE}/aligned_depth_to_color/image_raw"
        "enable_sync=true@${CAMERA_NODE}/color/image_raw/compressed"
    )
    TOPICS+=(
        "${CAMERA_NODE}/aligned_depth_to_color/image_raw"
        "${CAMERA_NODE}/aligned_depth_to_color/camera_info"
    )
else
    TOPICS+=("${CAMERA_NODE}/depth/image_rect_raw")
fi
# shellcheck disable=SC2206
TOPICS+=(${RECORD_TOPICS_EXTRA})

usage() { sed -n '2,58p' "$0" | sed 's/^# \{0,1\}//'; }

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
mkdir -p "$bags"

# The driver process, by its executable path -- a plain `pgrep -f` on the
# name also matches the shell running this script.
cam_pid() { pgrep -of '^[^ ]*/realsense2_camera_node ' || true; }

# One message on a topic within N seconds, or failure. `ros2 topic hz`
# does not survive a non-interactive shell; rclpy does.
wait_msg() {
    python3 - "$1" "$2" <<'PY'
import sys, rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image, CompressedImage
topic, secs = sys.argv[1], float(sys.argv[2])
msg = CompressedImage if topic.endswith("/compressed") else Image
rclpy.init()
node = rclpy.create_node("record_probe")
ok, _ = wait_for_message(msg, node, topic, qos_profile=qos_profile_sensor_data,
                         time_to_wait=secs)
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
}

# A colour-only camera node by hand: the same parameters the service gives
# its deck_camera (wojtek_bringup/launch_common.py), plus the infra streams
# off, on the non-RT cores, outliving this shell.
start_camera() {
    local node="$1" ns="${1%/*}"; ns="${ns#/}"; local name="${1##*/}"
    if [ -n "$(cam_pid)" ]; then echo ">> camera driver already running (pid $(cam_pid))"; return 0; fi
    echo ">> starting a colour-only camera node for the Deck"
    # A parameter file, not -p: the JPEG quality key starts with a dot
    # (image_transport's naming) and rcl's -p parser rejects that.
    cat > "$bags/camera_params.yaml" <<'YAML'
/**:
  ros__parameters:
    initial_reset: true
    enable_depth: false
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    rgb_camera.color_profile: "640x480x30"
    pointcloud.enable: false
    align_depth.enable: false
    enable_rgbd: false
    enable_sync: false
    .camera.color.image_raw.compressed.jpeg_quality: 80
YAML
    setsid nohup taskset -c 0,1 ros2 run realsense2_camera realsense2_camera_node \
        --ros-args -r "__ns:=/$ns" -r "__node:=$name" \
        --params-file "$bags/camera_params.yaml" \
        > "$bags/camera.log" 2>&1 < /dev/null &
    echo $! > "$bags/camera.pid"
    for _ in $(seq 1 40); do
        if ! kill -0 "$(cat "$bags/camera.pid")" 2>/dev/null; then
            echo "!! the camera node exited at start:" >&2; tail -n 20 "$bags/camera.log" >&2; exit 2
        fi
        ros2 node list 2>/dev/null | grep -qx "$node" && break
        sleep 1
    done
    if ! ros2 node list 2>/dev/null | grep -qx "$node"; then
        echo "!! $node did not come up; see ~/wojtek_bags/camera.log" >&2; tail -n 20 "$bags/camera.log" >&2; exit 2
    fi
    if ! wait_msg "$node/color/image_raw/compressed" 20; then
        echo "!! $node is up but sends no colour frames; see ~/wojtek_bags/camera.log" >&2; exit 2
    fi
    echo ">> $node up, colour flowing (pid $(cat "$bags/camera.pid"))"
}

case "$cmd" in
camera)
    start_camera "$1"
    ;;
start)
    name="$1"; node="$2"; min_free="$3"; shift 3
    [ "$1" = "--" ] && shift
    settings=(); while [ $# -gt 0 ] && [ "$1" != "--" ]; do settings+=("$1"); shift; done
    [ "${1:-}" = "--" ] && shift
    topics=("$@")
    dir="$bags/$name"
    if [ -e "$dir" ]; then echo "!! $dir exists, pick another name" >&2; exit 2; fi
    if [ -f "$bags/.recording" ]; then
        echo "!! already recording $(cat "$bags/.recording"); stop it first" >&2; exit 2
    fi
    if ! ros2 node list 2>/dev/null | grep -qx "$node"; then
        start_camera "$node"
    fi
    pid="$(cam_pid)"
    [ -n "$pid" ] || { echo "!! no realsense2_camera_node process" >&2; exit 2; }
    # Remember what the driver runs with now; stop restores it. After every
    # switch the driver must be alive AND, where the entry says so, a frame
    # must arrive: a driver that opens its sensors and delivers nothing is
    # the state that preceded the segfault.
    : > "$bags/$name.params"
    for kv in "${settings[@]}"; do
        check="${kv#*@}"; [ "$check" = "$kv" ] && check=""
        kv="${kv%%@*}"; key="${kv%%=*}"; val="${kv#*=}"
        cur="$(ros2 param get "$node" "$key" 2>/dev/null | sed -n 's/^.* value is: //p')"
        printf '%s %s\n' "$key" "$cur" >> "$bags/$name.params"
        echo ">> $key: $cur -> $val"
        ros2 param set "$node" "$key" "$val" >/dev/null
        sleep 2
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "!! the camera driver died after $key=$val (see journalctl -u wojtek-robot -e" >&2
            echo "!! or ~/wojtek_bags/camera.log). Get the Deck its eyes back with:" >&2
            echo "!!     ./record.sh camera" >&2
            exit 5
        fi
        if [ -n "$check" ]; then
            if wait_msg "$check" 15; then
                echo ">>   frames on $check"
            else
                echo "!! no frames on $check after $key=$val; the driver is up but" >&2
                echo "!! streaming nothing. Stopping here; ./record.sh stop restores it." >&2
                echo "$name" > "$bags/.recording"; : > "$bags/$name.pid"
                exit 6
            fi
        fi
    done
    # The recorder outlives this ssh session. SIGINT is how it is stopped
    # (mcap files get finalised); the guard next to it sends that when the
    # card runs low.
    setsid nohup taskset -c 0,1 ros2 bag record -o "$dir" -s mcap \
        --storage-preset-profile zstd_fast --max-bag-duration 120 \
        "${topics[@]}" > "$bags/$name.log" 2>&1 < /dev/null &
    rec=$!
    echo "$rec" > "$bags/$name.pid"
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
    ' guard "$rec" "$bags" "$min_free" >> "$bags/$name.log" 2>&1 < /dev/null &
    sleep 5
    if ! kill -0 "$rec" 2>/dev/null; then
        echo "!! recorder died at start:" >&2; cat "$bags/$name.log" >&2; exit 4
    fi
    echo ">> recording $name -> $dir"
    grep -c "Subscribed to topic" "$bags/$name.log" | sed 's/^/>> topics subscribed: /'
    grep -v "Subscribed to topic" "$bags/$name.log" | tail -n 5
    ;;
status)
    if [ -f "$bags/.recording" ]; then
        name="$(cat "$bags/.recording")"
        if [ -s "$bags/$name.pid" ] && kill -0 "$(cat "$bags/$name.pid")" 2>/dev/null; then
            echo ">> recording $name"
        else
            echo ">> $name: recorder is NOT running (start aborted, the disk guard stopped it, or it died; see the log)"
        fi
        du -sh "$bags/$name" 2>/dev/null || true
        ls -la "$bags/$name" 2>/dev/null | tail -n 4
        [ -f "$bags/$name.log" ] && { echo ">> last log lines:"; grep -v "Subscribed to topic" "$bags/$name.log" | tail -n 6; }
    else
        echo ">> not recording"
    fi
    echo ">> camera driver: ${cam_pid:+pid }$(cam_pid)"
    echo ">> free on the card:"; df -h "$bags" | tail -n 1
    uptime
    ;;
stop)
    node="$1"
    if [ ! -f "$bags/.recording" ]; then echo ">> nothing is recording"; exit 0; fi
    name="$(cat "$bags/.recording")"
    if [ -s "$bags/$name.pid" ]; then
        rec="$(cat "$bags/$name.pid")"
        if kill -0 "$rec" 2>/dev/null; then
            kill -INT "$rec"
            for _ in $(seq 1 40); do kill -0 "$rec" 2>/dev/null || break; sleep 0.5; done
            kill -0 "$rec" 2>/dev/null && { echo "!! recorder did not exit on SIGINT, killing" >&2; kill -KILL "$rec"; }
        fi
    fi
    rm -f "$bags/.recording"
    # Back to what the Deck was running with, switches first, profiles last.
    if [ -n "$(cam_pid)" ]; then
        tac "$bags/$name.params" | while read -r key val; do
            [ -n "$val" ] || continue
            echo ">> $key -> $val"
            ros2 param set "$node" "$key" "$val" >/dev/null || echo "!! could not restore $key" >&2
        done
    else
        echo "!! the camera driver is not running; nothing to restore. ./record.sh camera brings one up" >&2
    fi
    echo ">> stopped $name"
    [ -d "$bags/$name" ] && { du -sh "$bags/$name"; ros2 bag info "$bags/$name" 2>/dev/null | sed -n '1,40p' || true; }
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
camera) remote camera "$CAMERA_NODE" ;;
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
