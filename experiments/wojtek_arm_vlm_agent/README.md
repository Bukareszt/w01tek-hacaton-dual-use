# Experiment: SO-101 arm with a wrist camera, driven by a VLM agent

> **Status: EXPERIMENTAL. Not production, not on the robot.**
> Nothing here is deployed by `ros/deploy.sh` and nothing in `ros/` depends on
> it. Interfaces change without notice. Running the arm controller moves a
> physical arm and is a human-authorized action.

A robot arm (LeRobot SO-101, Feetech servos) carries a USB camera on its
wrist. A YOLO tracker and a vision-language model run on a GPU box; the arm
follows people and objects, and an operator talks to it in natural language:
*"target the man in the white shirt"*, *"find the banner with the QR code"*,
*"look left 40"*, *"cycle through everyone wearing glasses"*, *"stop"*.

Design decisions that shaped it, in order of appearance during the hackathon:

- **No memory, by design.** Earlier iterations kept bearing memories, a sightings
  log and CLIP re-identification; all of it was removed. The agent sees only
  through its tools and looks again when in doubt. This holds up when the arm
  is mounted on a walking base, where any stored bearing goes stale the moment
  the robot steps.
- **No sweeping.** Search is a look-and-turn loop: at each step the VLM grounds
  the description in the current picture; if it is not there it chooses a
  direction from what it sees, the code keeps it honest (coverage, joint
  limits), and it looks again.
- **The harness enforces the contract, not the prompt.** A 30B VLM will happily
  say "I will track it" and stop, or call `look` ten times. Observation
  commands get exactly one looking tool and then a description; action tools
  are terminal and the code writes the answer; repeated identical calls end the
  turn.

## Split between the robot host and the inference host

```
robot host (Raspberry Pi, USB camera + SO-101 serial)          inference host (GPU)
┌──────────────────────────────┐                                ┌──────────────────────────────┐
│ camera_stream.py   :8095 ────┼──── MJPEG (native MJPG) ──────▶│ tracker.py         :8093     │
│   native MJPG passthrough,   │                                │   YOLO11s + ByteTrack,       │
│   newest-frame-only          │                                │   80 COCO classes, ~5 ms/f   │
│                              │                                │        │ /frame + /tracks    │
│ arm_follower.py    :8094 ◀───┼──── GET /  (joint state) ──────┼── commander.py     :8100     │
│   latency-compensated        │──── GET /tracks (50 Hz) ──────▶│   VLM agent (Ollama, tools), │
│   absolute-goal servo        │──── GET /aim   (id | pan/tilt)▶│   sequencer, overlay, UI     │
└──────────────────────────────┘                                └───────────────┬──────────────┘
                                          annotated MJPEG (push)                │
                                   ┌────────────────────────────────────────────┘
                                   ▼
                       laptop: stream_relay.py  :8099 (annotated)  :8097 (raw)   UI: http://<inference host>:8100/
```

- The camera streamer hands out the camera's own JPEG frames without
  re-encoding and always serves the newest one; the tracker parses the stream
  by `Content-Length`, so no frame waits behind another.
- The follower reads tracks and the "what to aim at" decision over HTTP. Its
  control law is an absolute joint goal per camera frame: the joint angle that
  centres the target, computed from the arm position *at the frame's capture
  time* (a position history compensates pipeline latency), filtered lightly,
  with a 0.5° hysteresis so the servo is silent at rest. No velocity
  integration, so no windup and no hunting.
- The commander is the only component with a network dependency on the model:
  Ollama on the inference host with a vision model that supports tool calling
  (`qwen3-vl:30b-a3b-instruct` is what the hackathon used; about 0.5 s per tool
  step, 1 to 2 s with a picture). A Claude backend exists for the standing-rule
  classifier (`--model claude-opus-5`, needs `ANTHROPIC_API_KEY`).

Everything talks plain HTTP, so any of the three roles can move to another
machine by changing the host variables. Video from the inference host to the
laptop is *pushed* (the robot and the laptop connect out), which is convenient
when the GPU box sits behind a firewall that blocks inbound connections. The
firewall does have to accept the robot host on the tracker and commander ports
(`8093`, `8100`), or the follower cannot get tracks and aim decisions.

## Setup

Robot host (Raspberry Pi 4/5, 64-bit OS). The follower does **not** need LeRobot or
torch at runtime: `feetech_bus.py` talks to the servos through `feetech-servo-sdk`
(pyserial only) and reads LeRobot's calibration JSON, with LeRobot's exact unit
conventions (verified against `FeetechMotorsBus` on the same calibration: identical
degrees, identical sign-magnitude register encoding).

```bash
python3 -m venv ~/wojtek-arm && . ~/wojtek-arm/bin/activate
pip install -r experiments/wojtek_arm_vlm_agent/requirements-rpi.txt   # seconds, no torch
# user in the `video` and `dialout` groups
```

Calibrate the arm once with LeRobot on any machine
(`lerobot-calibrate --robot.type=so101_follower --robot.id=so101`) and copy the
resulting `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json`
to the same path on the robot host, or point `WOJTEK_ARM_CALIBRATION` at it. The
follower refuses to start if the servos' limit and homing registers do not match
the file, exactly like LeRobot.

Inference host (CUDA GPU, Ollama with the vision model pulled):

```bash
python3 -m venv ~/wojtek-arm && . ~/wojtek-arm/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130   # match your CUDA
pip install -r experiments/wojtek_arm_vlm_agent/requirements-dgx.txt
ollama pull qwen3-vl:30b-a3b-instruct
```

Repo-root `.env` (copy the block from `.env.example`): `WOJTEK_ARM_RPI_HOST`,
`WOJTEK_ARM_DGX_HOST`, optional `WOJTEK_ARM_VIEWER_HOST`, `WOJTEK_ARM_CAMERA`,
`WOJTEK_ARM_SERIAL`, `WOJTEK_ARM_OLLAMA_URL`, `WOJTEK_ARM_VLM_MODEL`,
`WOJTEK_ARM_SIGNS`.

## Run order

```bash
# robot host
./experiments/wojtek_arm_vlm_agent/run.sh rpi-camera
# inference host
./experiments/wojtek_arm_vlm_agent/run.sh dgx-tracker
./experiments/wojtek_arm_vlm_agent/run.sh dgx-commander
# robot host — moves the arm; get a human's go-ahead first
./experiments/wojtek_arm_vlm_agent/run.sh rpi-follower
# laptop (optional): video relays, then open http://127.0.0.1:8099/ for video
./experiments/wojtek_arm_vlm_agent/run.sh viewer
```

The UI is served by the commander on port 8100 of the inference host. Its
video element defaults to the laptop relay; pass `?video=http://HOST:8095/stream`
in the URL to watch the raw camera instead.

### First run: joint direction signs

The follower needs to know which way each joint moves the picture. Run
`./run.sh probe` once with a person standing still in view; it nudges pan and
tilt by 8° each and prints `--sign-pan N --sign-tilt N`. Put them in
`WOJTEK_ARM_SIGNS` (third value: shoulder lift, which takes over vertical
tracking when the wrist reaches its range; it had the same sign as the wrist on
our arm). On our SO-101 all three were `+1`: positive wrist flex and positive
shoulder lift both pitch the camera down, positive pan turns it right. The
probe is unreliable when a person fills the frame; verify with two snapshots if
in doubt.

## The agent's tools

| tool | arguments | effect |
|---|---|---|
| `look` | `classes` | fresh picture with numbered boxes plus detections (id, class, position, size) |
| `look_left` `look_right` `look_up` `look_down` | `degrees` | relative turn, then `look` |
| `turn` | `pan_deg`, `tilt_deg` | relative turn (clamped ±60/±40 per call), then `look` |
| `inspect` | `id` | large crop of one detection |
| `track` | `id` | follow a detection until told otherwise (terminal) |
| `aim_at` | `description` | ground anything visible in the current picture, centre it, hold; tracks it if it coincides with a detection (terminal) |
| `find` | `description`, `classes` | look-and-turn search until the thing is in view, then `aim_at` behaviour (terminal) |
| `cycle` | `description`, `mode`, `classes` | visit every visible matching detection once or in a loop (terminal) |
| `stop` `home` `wait` | | hold, return to the saved posture, wait then `look` |

`find` details: a plain class name ("a bottle") short-circuits to the largest
such detection; otherwise each step asks the model for a bounding box of the
description (the model answers on a 0–1000 grid, normalised here), turns
toward a candidate cut off at a frame edge before judging it, and, if nothing
matches, asks for one move (left/right/up/down, 10–60°) while the code tracks
covered pan ranges per tilt band, clamps to the joint limits and steers to the
nearest unexplored range when the model repeats itself. Measured: a person in
white found in 9 s over two steps; a banner found and centred in 11 s.

## HTTP API (commander, port 8100)

`GET /state` full JSON state; `GET /aim` what the follower should do;
`POST /agent {"text"}` run a command, `GET /agent` the transcript;
`POST /instruction {"text","classes"}` standing rule; `POST /control {"mode":
"off"|"once"|"loop", "next", "dwell_s"}`; `POST /aim {"id"}` or `{"pan","tilt"}`
or `{"dpan","dtilt"}` or `{"posture": true|"save"}`; `GET /crop/<id>`,
`GET /snap`, `GET /stream`.

## Safety and limits

- The follower has **no excursion or rate limits** beyond the calibrated joint
  range (removed on request during the hackathon). It refuses to start only if
  another process holds the serial port, and it turns torque off on exit. Stop
  it with SIGTERM, never SIGKILL, or torque stays on.
- Stale video (older than 1 s) clears the target and holds the arm.
- The camera used here delivers 20 fps in every mode despite advertising 30.
- Re-identification is deliberately absent: a person who leaves and returns
  is a new detection. Track continuity across tracker id churn is handled by
  box overlap within 2.5 s (`bytetrack_sticky.yaml` also keeps lost tracks
  alive longer).
- Observation versus action is decided by keywords in the command; unusual
  phrasing can misclassify. Plain `stop` and `home` bypass the model.
- On a walking base the vision loop keeps working (it is closed on the image),
  but nothing here knows the base moved; there is no odometry integration.

## Layout

```
camera_stream.py       robot host: MJPG passthrough streamer + optional push
arm_follower.py        robot host: SO-101 controller (absolute-goal servo, goto mode, status :8094)
feetech_bus.py         torch-free Feetech STS3215 bus with LeRobot calibration/units
tracker.py             inference host: YOLO11 + ByteTrack, /frame + /tracks (:8093)
commander.py           inference host: VLM agent, sequencer, overlay, UI (:8100)
stream_relay.py        laptop: receives pushed frames, serves MJPEG locally
bytetrack_sticky.yaml  tracker config with a longer lost-track buffer
run.sh                 launcher reading the repo-root .env
requirements-*.txt     per-host dependencies
```

License: Apache-2.0, as the rest of the repository.
