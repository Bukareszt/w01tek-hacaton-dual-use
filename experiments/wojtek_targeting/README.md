# wojtek_targeting

Baltic hackathon subsystem: camera -> YOLO -> gimbal/laser lock-on. Bolted
onto Wojtek for the weekend, deliberately isolated from the 400 Hz control
loop and from `wojtek_perception_bringup` (that package's D435 is for the
walking planner's terrain sensing; this one is for aiming).

Read this file before touching anything in here. It is the brief, not a
status report -- most of the boxes below are still unchecked.

## Scope: what this package owns and what it doesn't

- **Owns**: bringing up the targeting camera, and the `gimbal_bridge` node
  that turns a detection into a serial command for the microcontroller.
- **Does not own**: YOLO / the detection model itself. That is a separate
  effort (different person, possibly a different node or even a different
  machine) publishing onto the topic contract below. Do not add inference
  code to this package.
- **Does not touch**: `wojtek_bringup`, `ros2_control`, the RT service, or
  anything the real robot's walking stack depends on. If a change here
  requires editing those, stop and reconsider the approach instead.

## Architecture

```
[camera driver] --/targeting_camera/.../color/image_raw--> [YOLO, elsewhere]
                                                                   |
                                                     wojtek_targeting_msgs/LaserTarget
                                                                   v
                                                          [gimbal_bridge node]
                                                                   |
                                                      fixed-rate binary UART frame
                                                                   v
                                                     [MCU: 2x Dynamixel servo (pan/tilt)
                                                           + MOSFET-driven laser PWM]
```

Two machines, two languages, one contract in the middle:

- **RPi 5** (ROS 2 Jazzy, this workspace): camera driver + YOLO (elsewhere)
  + `gimbal_bridge`. `gimbal_bridge` is the only node this package ships
  that runs continuously; it is a thin, fast translator, not a planner.
- **MCU** (STM32 Nucleo-F411RE, or ESP32 if that's what's already on the
  bench -- see "Open decisions"): no ROS, no YOLO, just UART in, Dynamixel
  + PWM out. Kept dumb on purpose: nothing here should be blocked on the
  MCU side compiling or flashing cleanly under deadline.

## Topic contract (the thing to agree with the YOLO side *today*)

`wojtek_targeting_msgs/LaserTarget`, published by whatever does detection:

```
bool  detected
float32 target_x      # bbox center, normalized -1..1 (image width)
float32 target_y      # bbox center, normalized -1..1 (image height)
float32 distance_m    # from camera depth, 0 if unavailable
float32 confidence
```

`gimbal_bridge` is the only subscriber. It owns all angle math (pixel/depth
-> pan/tilt), the lock-on threshold, and the dead-man timeout -- none of
that belongs upstream in the detector. Whoever writes the detector only
needs to publish this message at whatever rate YOLO manages; `gimbal_bridge`
holds the last target and re-serves it to the MCU at its own fixed rate, so
a slow or jittery detector does not show up as a jittery gimbal.

## Serial frame (RPi -> MCU)

Fixed-size binary, not text -- cheaper to parse on the MCU, nothing to
desync:

```
[0xAA][pan_int16][tilt_int16][lock_flag][checksum]
```

- pan / tilt: int16, 0.01 deg units
- lock_flag: 0 = tracking, not locked · 1 = locked -> MCU starts the laser
  PWM blink pattern
- 115200 baud, written at a fixed 30-50 Hz regardless of YOLO's actual rate
- checksum: XOR (or sum) of the preceding bytes -- good enough for this link

## Camera

- Namespace/name **must not** be `camera`/`camera` -- that is
  `wojtek_perception_bringup`'s terrain camera, already live on
  `/camera/camera/...`. Use `camera_name:=targeting_camera
  camera_namespace:=targeting_camera` (or whatever the eventual driver
  calls its equivalent options).
- Config should be leaner than `wojtek_perception_bringup/config/d435.yaml`:
  no pointcloud, no RGBD, no depth post-processing stack. This camera only
  needs to feed YOLO (+ depth for range, if the sensor gives it cheaply).
- **Hardware chosen: plain USB webcam** -- a simple, low-cost approach that
  avoids both external SDKs and deep driver complexity. See
  [`ros/src/wojtek_targeting_camera/`](ros/src/wojtek_targeting_camera/) for
  the driver implementation. Original options were not pursued:
  - **Occipital Structure Core (STO2D-C)** would require the closed-source
    Structure SDK (developer.structure.io, XRPro LLC), which is not obtainable
    in this environment. No maintained ROS2 driver exists for this sensor.
  - **RealSense D435**: adds unnecessary complexity for YOLO's needs; USB
    webcam is sufficient and simpler to integrate on the RPi.

## Hardware / control loop notes (carried over from planning, don't relitigate)

- Servos: 2x Dynamixel (XM540-W270-R pan, XM430-W210-R tilt). TTL
  half-duplex serial bus (Protocol 2.0), not plain PWM servos -- needs
  DynamixelSDK or Dynamixel2Arduino on the MCU side, IDs/baud/operating
  mode set once via their config tool before any code touches them.
- MCU voltage: check the exact Dynamixel model's supply range against the
  robot's 40 V bus -- assume a buck converter is required, don't tap 40 V
  directly into a servo.
- Laser: MOSFET (logic-level, e.g. IRLZ44N) gated by MCU PWM, laser module
  on its own regulated rail off the 40 V bus, current-limited per the
  module's own driver circuitry -- don't drive a bare diode off the MOSFET.
  Keep it in the Class 2/3R visible-laser-pointer power range: this is a
  demo, not a beam weapon, and most venues won't allow anything hotter.
- IMU: cut. It only matters for compensating aim during walking, and the
  fallback plan is stop-then-lock-then-track, not aim-while-walking.
- CPU affinity on the RPi: match `wojtek_perception_bringup`'s pattern --
  keep the camera driver and `gimbal_bridge` off the `isolcpus` RT cores
  (`taskset -c 2,3` or whatever the robot bringup currently reserves for
  non-RT work). Verify with `ros2 param list` after launch, don't assume a
  parameter took effect.
- USB/CPU budget is not proven for "two cameras + YOLO on one RPi 5"
  simultaneously. `wojtek_perception_bringup`'s own numbers show a Python
  subscriber already dropping ~30% of frames on *one* camera's heavier
  streams. Test the full combination early, not as the last integration
  step.

## Status

| piece | state |
|---|---|
| topic contract (`LaserTarget.msg`) agreed with detection side | not yet -- do this first |
| camera hardware decision (RealSense vs Structure Core) | open, spike planned |
| `gimbal_bridge` node (angle math, dead-man, serial write) | not started |
| MCU: Dynamixel bring-up (single servo, single angle) | not started |
| MCU: both servos as pan/tilt pair | not started |
| MCU: MOSFET + laser PWM, bench-tested standalone | not started |
| MCU: UART frame parser | not started |
| end-to-end integration | not started |

## Open decisions (resolve, don't guess)

- STM32 vs ESP32 for the MCU: whichever is already on the bench wins --
  don't burn hours switching platforms for a marginal advantage.
- Whether depth/range is used at all for this hackathon slice, or whether
  bbox size / a fixed assumed demo distance is good enough to ship.
- Whether the camera driver stays a plain node (matching
  `wojtek_perception_bringup`'s "no component container" reasoning -- true
  here too, nothing else in this graph shares its process) or needs
  revisiting once the real hardware is in hand.

## Conventions carried over from the workspace root `CLAUDE.md`

- New code is Apache-2.0.
- Use "Wojtek" in prose, names, and paths -- no `fbb-*` in anything new.
- No credentials, hostnames, or private-infrastructure identifiers in this
  tree, even though this fork is currently private -- write it as if it
  will be merged upstream, because it might be.
- Arming/launching the *real* robot stays human-authorized; this package
  only ever talks to the MCU and the camera, never to `ros2_control` or the
  robot's arm/disarm services.
