#!/usr/bin/env bash
# Launcher for the SO-101 arm + wrist camera + VLM agent experiment.
#
#   ./run.sh rpi-camera      robot host: camera streamer (MJPEG on :8095, optional push to the viewer)
#   ./run.sh rpi-follower    robot host: arm controller (MOVES THE ARM; human-authorized)
#   ./run.sh dgx-tracker     inference host: YOLO11 + ByteTrack over the robot's camera stream (:8093)
#   ./run.sh dgx-commander   inference host: VLM agent, sequencer, UI (:8100)
#   ./run.sh viewer          laptop: relays for the raw (:8097) and annotated (:8099) video
#   ./run.sh probe           robot host: learn the joint direction signs once (moves the arm a few degrees)
#
# Hosts, devices and the model come from the repo-root .env (see .env.example):
#   WOJTEK_ARM_RPI_HOST, WOJTEK_ARM_DGX_HOST, WOJTEK_ARM_VIEWER_HOST, WOJTEK_ARM_CAMERA,
#   WOJTEK_ARM_SERIAL, WOJTEK_ARM_OLLAMA_URL, WOJTEK_ARM_VLM_MODEL, WOJTEK_ARM_SIGNS
# Extra arguments after the subcommand are passed to the underlying script.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
if [[ -f "$ROOT/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$ROOT/.env"
    set +a
fi

PY="${PYTHON:-python3}"
RPI="${WOJTEK_ARM_RPI_HOST:-}"
DGX="${WOJTEK_ARM_DGX_HOST:-}"
VIEWER="${WOJTEK_ARM_VIEWER_HOST:-}"
CAMERA="${WOJTEK_ARM_CAMERA:-/dev/video0}"
SERIAL="${WOJTEK_ARM_SERIAL:-/dev/ttyACM0}"
OLLAMA="${WOJTEK_ARM_OLLAMA_URL:-http://127.0.0.1:11434}"
MODEL="${WOJTEK_ARM_VLM_MODEL:-qwen3-vl:30b-a3b-instruct}"
read -r SIGN_PAN SIGN_TILT SIGN_LIFT <<<"${WOJTEK_ARM_SIGNS:-1 1 1}"

need() { [[ -n "${!1}" ]] || { echo "error: set $1 in $ROOT/.env" >&2; exit 2; }; }

cmd="${1:-}"; shift || true
case "$cmd" in
    rpi-camera)
        push=(); [[ -n "$VIEWER" ]] && push=(--push "$VIEWER:8096")
        exec "$PY" "$HERE/camera_stream.py" --dev "$CAMERA" --size 640x480 --fps 30 \
            --port 8095 --bind 0.0.0.0 "${push[@]}" "$@"
        ;;
    rpi-follower)
        need DGX
        exec "$PY" "$HERE/arm_follower.py" --execute --port "$SERIAL" \
            --tracker "http://$DGX:8093" --aim-url "http://$DGX:8100/aim" \
            --sign-pan "$SIGN_PAN" --sign-tilt "$SIGN_TILT" --sign-lift "$SIGN_LIFT" \
            --status-bind 0.0.0.0 "$@"
        ;;
    probe)
        need DGX
        exec "$PY" "$HERE/arm_follower.py" --execute --probe --port "$SERIAL" \
            --tracker "http://$DGX:8093" --status-bind 0.0.0.0 "$@"
        ;;
    dgx-tracker)
        need RPI
        exec "$PY" "$HERE/tracker.py" --source "http://$RPI:8095/stream" --model yolo11s.pt --device 0 \
            --tracker "$HERE/bytetrack_sticky.yaml" --port 8093 --bind 0.0.0.0 "$@"
        ;;
    dgx-commander)
        need RPI
        push=(); [[ -n "$VIEWER" ]] && push=(--push "$VIEWER:8098")
        exec "$PY" "$HERE/commander.py" --tracker http://127.0.0.1:8093 --follower "http://$RPI:8094" \
            --model "$MODEL" --ollama-url "$OLLAMA" --port 8100 --bind 0.0.0.0 "${push[@]}" "$@"
        ;;
    viewer)
        "$PY" "$HERE/stream_relay.py" --listen 0.0.0.0:8096 --http 127.0.0.1:8097 &
        exec "$PY" "$HERE/stream_relay.py" --listen 0.0.0.0:8098 --http 127.0.0.1:8099
        ;;
    *)
        sed -n '2,15p' "$0"
        exit 1
        ;;
esac
