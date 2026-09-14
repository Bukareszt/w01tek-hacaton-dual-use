# wojtek_targeting

Baltic hackathon subsystem: camera -> YOLO -> gimbal/laser lock-on. Bolted
onto Wojtek for the weekend, deliberately isolated from the 400 Hz control
loop and from `wojtek_perception_bringup` (that package's D435 is for the
walking planner's terrain sensing; this one is for aiming).

Read this file before touching anything in here. It is the brief, not a
status report -- most of the boxes below are still unchecked.

This slice of the experiment covers the targeting camera and the
Dynamixel angle/bus software (`wojtek_targeting`,
`wojtek_targeting_msgs`) that drives the pan/tilt gimbal directly from
the Pi -- no MCU. The detector (YOLO) and the laser hardware are still
not part of this package.

## Scope: what this package owns and what it doesn't

- **Owns**: bringing up the targeting camera and publishing its frames;
  bringing the pan/tilt Dynamixel pair up safely and commanding bounded
  angles (`wojtek_targeting`'s `dynamixel_driver.py` /
  `targeting_controller.py`).
- **Does not own**: YOLO / the detection model, or the laser. Those land
  in a separate package once that design is settled.
- **Does not touch**: `wojtek_bringup`, `ros2_control`, the RT service, or
  anything the real robot's walking stack depends on. If a change here
  requires editing those, stop and reconsider the approach instead.

## Gimbal / angle software

- [`ros/src/wojtek_targeting/`](ros/src/wojtek_targeting/) -- DynamixelSDK
  driver for the XM540 pan / XM430 tilt pair behind a U2D2, plus the
  controller that turns a detection (or a manual bench command) into
  bounded pan/tilt angles. See that package's own README for the bring-up
  order, the angle-conversion math (`ENCODER_RESOLUTION = 4096`,
  standard X-series 0.088 deg/tick), and the safety fixes carried over
  from a prior project's hardware bring-up (never let a failed read look
  like a zero, rewrite the RAM PID/profile registers on every connect,
  seed `GOAL_POSITION` with the present position before torque-on).
- [`ros/src/wojtek_targeting_msgs/`](ros/src/wojtek_targeting_msgs/) --
  `LaserTarget`, `TargetingStatus`, `SetGimbalAngles`.

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
  the driver implementation. The specific unit is a generic UVC webcam
  (vendor:product `10bb:2b08`, "Generic_USB_Camera", serial `EP.20CC54K01`),
  moved from the dev box to the RPi; the config's `device` param points at
  its `/dev/v4l/by-id/...` path rather than a `/dev/videoN` number for that
  reason. Original options were not pursued:
  - **Occipital Structure Core (STO2D-C)** would require the closed-source
    Structure SDK (developer.structure.io, XRPro LLC), which is not obtainable
    in this environment. No maintained ROS2 driver exists for this sensor.
  - **RealSense D435**: adds unnecessary complexity for YOLO's needs; USB
    webcam is sufficient and simpler to integrate on the RPi.

## Status

| piece | state |
|---|---|
| camera hardware decision | done -- generic UVC USB webcam, see "Camera" above |
| `wojtek_targeting_camera` (usb_camera_node) | **verified on the RPi 5** -- opened at 640x480 @ ~15 Hz over `/targeting_camera/targeting_camera/image_raw`, confirmed live in RViz over the robot's wlan0 link |
| camera calibration (`fx`/`fy`) | not started -- ships uncalibrated, node warns at startup |
| topic contract (`LaserTarget.msg`) agreed with detection side | not yet -- do this first |
| bus comms + angle math, pan (ID 1, XM540) | **bus-verified** -- answers at 1,000,000 baud (not the 57600 the config comments list as factory default); commanded raw-tick moves land within 1 tick, `deg_to_raw`/`raw_to_deg` checked against the actual reads |
| pan servo, physical rotation | **not confirmed -- see below** |
| tilt servo (ID 2, XM430) | **not answering** at any baud tried; likely wiring/power/ID, not yet root-caused |
| gimbal mechanical limits, direction, centre | not started -- blocked on the above |
| laser hardware, under the event's rules | not started |
| end-to-end integration | not started |

### Pan servo: encoder motion without confirmed physical rotation

Bench-testing pan (ID 1) turned up something that needs resolving before
anyone trusts this axis: commanded moves report success --
`PRESENT_POSITION` lands exactly on the commanded raw tick, current stays
near 0 throughout, no comms errors -- but the mount was not confirmed to
actually rotate, and a later attempt reported no visible or audible
motion at all (no motor sound, no vibration) despite the same clean
encoder telemetry. That combination -- correct encoder feedback, zero
holding current, no physical motion -- is consistent with an internal
gear-train fault (the position sensor sits on a different stage than the
one that failed) rather than a software or wiring problem, but this has
not been physically inspected. **Do not trust `PRESENT_POSITION` alone as
proof of motion on this unit until it is confirmed by eye.** Next step is
opening up or swapping the pan servo, not more software.

## Open decisions (resolve, don't guess)

- Whether depth/range is used at all for this hackathon slice, or whether
  bbox size / a fixed assumed demo distance is good enough to ship.
- Whether the camera driver stays a plain node (matching
  `wojtek_perception_bringup`'s "no component container" reasoning -- true
  here too, nothing else in this graph shares its process) or needs
  revisiting once the real hardware is in hand.
- Gimbal actuation approach: this slice has the Pi driving the Dynamixels
  directly (no MCU); whether that holds once the pan servo's physical
  rotation is resolved (see "Status" above) is still open.

## Conventions carried over from the workspace root `CLAUDE.md`

- New code is Apache-2.0.
- Use "Wojtek" in prose, names, and paths -- no `fbb-*` in anything new.
- No credentials, hostnames, or private-infrastructure identifiers in this
  tree, even though this fork is currently private -- write it as if it
  will be merged upstream, because it might be.
- Arming/launching the *real* robot stays human-authorized; this package
  only ever talks to the camera and the Dynamixel bus, never to
  `ros2_control` or the robot's arm/disarm services.
