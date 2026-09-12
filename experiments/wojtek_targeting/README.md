# wojtek_targeting

Baltic hackathon subsystem: camera -> YOLO -> gimbal/laser lock-on. Bolted
onto Wojtek for the weekend, deliberately isolated from the 400 Hz control
loop and from `wojtek_perception_bringup` (that package's D435 is for the
walking planner's terrain sensing; this one is for aiming).

Read this file before touching anything in here. It is the brief, not a
status report -- most of the boxes below are still unchecked.

## Scope: what this package owns and what it doesn't

- **Owns**: bringing up the targeting camera, and the `targeting_controller`
  node that turns a detection into bounded pan/tilt commands on the
  Dynamixel bus.
- **Does not own**: YOLO / the detection model itself. That is a separate
  effort (different person, possibly a different node or even a different
  machine) publishing onto the topic contract below. Do not add inference
  code to this package.
- **Does not touch**: `wojtek_bringup`, `ros2_control`, the RT service, or
  anything the real robot's walking stack depends on. If a change here
  requires editing those, stop and reconsider the approach instead.

## Architecture

```
[usb_camera_node] --/targeting_camera/.../image_raw--> [YOLO, elsewhere]
                                                               |
                                                 wojtek_targeting_msgs/LaserTarget
                                                               v
                                                 [targeting_controller node]
                                                               |
                                                DynamixelSDK, Protocol 2.0
                                                               v
                                                   [U2D2] -> XM540 pan (ID 1)
                                                          -> XM430 tilt (ID 2)
```

**There is no MCU.** The earlier plan put an STM32 between the Pi and the
servos, speaking a hand-rolled UART frame. That was dropped once it was
clear the Dynamixels are already smart serial devices with their own
position loop: an MCU in the middle would have been a second thing to flash
under deadline, translating one serial protocol into another one it does not
improve. The Pi drives the U2D2 directly.

One machine, three processes:

- **`wojtek_targeting_camera`** -- the USB webcam, `image_raw` + `camera_info`.
- **YOLO** -- somebody else's, possibly on another machine. Publishes
  `LaserTarget` and touches nothing else.
- **`wojtek_targeting`** -- `targeting_controller`, the only process that
  commands the servos. Owns the angle math, the limits, the lock threshold
  and the dead-man timeout.

The laser is **not** in this graph at all. It is armed by hardware outside
ROS, which is the point: no software fault should be able to energise it.

## Topic contract (the thing to agree with the YOLO side *today*)

`wojtek_targeting_msgs/LaserTarget`, published by whatever does detection:

```
std_msgs/Header header   # stamp is MANDATORY -- see below
bool  detected
string class_name
float32 target_x      # bbox center, normalized -1..1 (image width)
float32 target_y      # bbox center, normalized -1..1 (image height)
float32 distance_m    # 0 if unavailable; unused today
float32 confidence
```

`targeting_controller` is the only subscriber. It owns all angle math, the
lock-on threshold and the dead-man timeout -- none of that belongs upstream
in the detector. Whoever writes the detector only needs to publish this at
whatever rate YOLO manages; the controller holds the last target and
re-commands the servos at its own fixed 40 Hz, so a slow or jittery detector
does not show up as a jittery gimbal.

Two things the detector side must get right:

- **Stamp the header.** The controller drops any target older than 200 ms --
  that timeout is the dead-man that stops the gimbal if the detector dies
  mid-track. An unstamped message reads as infinitely old and is ignored
  entirely, which is the safe way for it to fail but looks like "the gimbal
  does nothing" from the outside.
- **Normalize against the frame you actually ran on.** -1..1 across the
  image, not pixels, so a detector working on a downscaled frame does not
  have to know the capture geometry.

`distance_m` is carried but unused: the camera rides on the gimbal, which
makes aiming a servoing problem that needs no range. See the note at the top
of `kinematics.py`.

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
  half-duplex serial bus (Protocol 2.0), not plain PWM servos. IDs, baud and
  operating mode are set once with Dynamixel Wizard before any code touches
  them -- and **check the baud rate**, because a wrong one is indistinguishable
  from a dead bus and eats an afternoon.
- Servo supply: check each model's range against the robot's 40 V bus --
  assume a buck converter is required, don't tap 40 V directly into a servo.
- Laser: on its own regulated rail, current-limited per the module's own
  driver circuitry, gated by hardware the software cannot reach. Keep it in
  the Class 2/3R visible-laser-pointer range: this is a demo, not a beam
  weapon, and most venues won't allow anything hotter.
- IMU: cut. It only matters for compensating aim during walking, and the
  fallback plan is stop-then-lock-then-track, not aim-while-walking.
- CPU affinity on the RPi: match `wojtek_perception_bringup`'s pattern --
  keep the camera driver and `targeting_controller` off the `isolcpus` RT
  cores (`taskset -c 2,3` or whatever the robot bringup currently reserves
  for non-RT work; both launches take a `cpus:=` argument). Verify with
  `ros2 param list` after launch, don't assume a parameter took effect.
- USB/CPU budget is not proven for "two cameras + YOLO on one RPi 5"
  simultaneously. `wojtek_perception_bringup`'s own numbers show a Python
  subscriber already dropping ~30% of frames on *one* camera's heavier
  streams. Test the full combination early, not as the last integration
  step.

## Status

Nothing below has run against hardware. "Written" means written, reviewed
and unit-tested with the hardware faked; the bench is where it gets found out.

| piece | state |
|---|---|
| camera hardware decision | settled -- USB webcam |
| `wojtek_targeting_camera` (usb_camera_node) | written, not run |
| topic contract (`LaserTarget.msg`) agreed with detection side | **message written, not yet agreed -- do this first** |
| `wojtek_targeting_msgs` (LaserTarget, TargetingStatus, SetGimbalAngles) | written |
| `dynamixel_driver.py` (U2D2, both servos, safe startup) | written, not run |
| `targeting_controller.py` (state machine, limits, dead-man) | written, not run |
| bench: one servo answering, correct baud and IDs | not started -- **next step** |
| bench: both servos, directions and centre set, limits verified | not started |
| camera calibration (`fx`/`fy`; tracking refuses without it) | not started |
| gimbal URDF / TF tree, verified in RViz | not started |
| closed-loop tracking against a real detector | not started |
| laser hardware, under the event's rules | not started |

## Open decisions (resolve, don't guess)

- **The topic contract with whoever writes the detector.** The message
  exists; the agreement does not. Settle the stamp and the normalization
  convention before either side writes more code against it.
- Whether depth/range is ever used, or whether the calibrated
  `pan_offset_deg`/`tilt_offset_deg` at a fixed demo distance is enough. The
  controller currently assumes the latter and ignores `distance_m`.
- Whether the camera driver stays a plain node (matching
  `wojtek_perception_bringup`'s "no component container" reasoning -- true
  here too, nothing else in this graph shares its process) or needs
  revisiting once the real hardware is in hand.

Settled, recorded so they are not reopened: no MCU (the Dynamixels already
have a position loop; see Architecture), and no TF/depth solve for aiming
(the camera rides on the gimbal, so it is a servoing problem).

## Conventions carried over from the workspace root `CLAUDE.md`

- New code is Apache-2.0.
- Use "Wojtek" in prose, names, and paths -- no `fbb-*` in anything new.
- No credentials, hostnames, or private-infrastructure identifiers in this
  tree, even though this fork is currently private -- write it as if it
  will be merged upstream, because it might be.
- Arming/launching the *real* robot stays human-authorized; this package
  only ever talks to the gimbal servos and the camera, never to
  `ros2_control` or the robot's arm/disarm services.
