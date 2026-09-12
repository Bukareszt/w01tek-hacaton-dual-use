# wojtek_structure_core

ROS 2 driver for the Occipital **Structure Core (ST02D-C)**, used as the
targeting camera of the `wojtek_targeting` experiment. Colour and depth with
their `camera_info`, and nothing else.

This is one of the two camera paths the [experiment brief](../../../README.md)
leaves open. It exists so that path can actually be tried; the brief's fallback
to a second RealSense is still the fallback, and the decision checkpoint in it
still applies — **if the sensor is not streaming within an hour or two, take
the RealSense path and stop spending the weekend on a camera.**

## The SDK still exists (checked 2026-09-12)

The concern that prompted this package was that the Structure SDK had gone
away with Occipital. It has not:

- The SDK is now published by **XRPro, LLC**, which holds the Structure
  licensing. The developer portal at `developer.structure.io` is live and the
  SDK is still behind the same free Developer Program registration.
- The **Cross-Platform** SDK (the Structure Core one — not the iOS Structure
  SDK, which is a different product) lists **Linux ARM64** among its supported
  platforms, which is what the RPi 5 needs, and its release notes call out
  support for the **ST02D (enclosed)** Structure Core model — this sensor.

What has *not* improved is the ROS story: there is still no maintained ROS 2
driver for this sensor. The community ones are ROS 1 (e.g.
`AutoModality/struct_core_ros`), and the SDK's own bundled ROS 2 wrapper is
beta with no evidence of being kept current against Jazzy. Hence this package:
small enough to read in one sitting, with no dependency beyond the SDK itself.

### Getting the SDK

Register for the Developer Program on the Structure developer portal and
download **Structure SDK (Cross-Platform)**. It is closed-source and
redistribution is not permitted, so it is **not vendored here** and must not be
committed — unpack it outside the repository (or somewhere gitignored) and
point the build at it.

The SDK archive also ships the udev rule the sensor needs; install it before
trying to open the device as a non-root user, or the driver will report
`USBDriverNotInstalled` and stop.

## Build

```bash
# from experiments/wojtek_targeting/ros
colcon build --packages-select wojtek_structure_core \
  --cmake-args -DSTRUCTURE_SDK_DIR=/path/to/StructureSDK-CrossPlatform
```

`STRUCTURE_SDK_DIR` may also come from the environment. The build fails loudly
with that message if the SDK is not found — it does not quietly skip itself,
because a camera package that builds without its camera library is worse than
one that does not build.

## Run

```bash
ros2 launch wojtek_structure_core targeting_camera.launch.py
```

Topics (default name/namespace, deliberately **not** `camera/camera` — that is
the terrain D435 and it is already live):

| topic | type | notes |
|---|---|---|
| `/targeting_camera/targeting_camera/color/image_raw` | `sensor_msgs/Image` | `rgb8`, 640x480 — what YOLO subscribes to |
| `/targeting_camera/targeting_camera/color/camera_info` | `sensor_msgs/CameraInfo` | `plumb_bob` from the SDK's intrinsics |
| `/targeting_camera/targeting_camera/depth/image_rect_raw` | `sensor_msgs/Image` | `16UC1` millimetres, 0 = no return |
| `/targeting_camera/targeting_camera/depth/camera_info` | `sensor_msgs/CameraInfo` | depth camera's own intrinsics |

Settings live in [`config/targeting_camera.yaml`](config/targeting_camera.yaml),
which explains why each one is what it is. The launch exposes the few that get
changed at the bench (`sensor_serial`, `enable_depth`, `depth_resolution`,
`cpus`).

## What it deliberately does not do

- **No registration between depth and colour.** The two cameras are physically
  apart, so a bbox centre in colour pixels is not the same point in the depth
  image. Correct it in the consumer using the two `camera_info` sets if the
  parallax turns out to matter at demo range; do not add a registration pass
  here.
- **No pointcloud, no RGBD, no depth post-processing.** Aiming does not need
  them and this sensor shares a USB bus and an RPi with the terrain camera.
- **No IMU.** Cut by the brief: the fallback is stop-then-lock-then-track, not
  aim-while-walking. The accelerometer and gyroscope are explicitly disabled in
  the capture settings rather than merely unpublished, so they cost no
  bandwidth.
- **No TF.** Nothing in the targeting graph transforms into the robot body.
- **Timestamps are node-clock arrival times**, not the SDK's frame timestamps,
  which sit on an unrelated epoch. Good enough for a fixed-rate consumer;
  not good enough if anything ever needs to fuse these frames with robot state.

## Not proven yet

Written against the SDK's published API, **not yet compiled or run against
hardware** — the SDK needs a registered download, and the sensor needs to be on
the bench. Expect the first build to be where the SDK version in hand disagrees
with this code, most likely in the capture-settings field names.

Two things from the brief that are still unmeasured and matter more than this
driver's code:

- Whether **two cameras plus YOLO on one RPi 5** fits in the USB and CPU
  budget. The perception package already measured a Python subscriber dropping
  ~30% of frames on *one* camera's heavier streams. Test the combination early.
- Whether the Structure Core talks to the RPi at all. Run the SDK's own sample
  app against the board before trusting anything here.
