# wojtek_targeting_camera

The targeting camera: a plain USB webcam on the RPi, published as `image_raw` +
`camera_info` for the detector to consume. One node, no depth, no
post-processing.

This is the camera the [experiment brief](../../../README.md) ended up with.
The two options it originally weighed are both closed: the Structure Core needs
a closed-source SDK that is not obtainable here (see
[`wojtek_structure_core`](../wojtek_structure_core/README.md)), and a second
RealSense was never necessary once depth stopped being a requirement for the
aiming path. A webcam is enough to feed YOLO, which is all the detector side
actually asked for.

## Run

```bash
ros2 launch wojtek_targeting_camera targeting_camera.launch.py
ros2 launch wojtek_targeting_camera targeting_camera.launch.py device:=/dev/video2
```

Topics (default name/namespace, deliberately **not** `camera/camera` — that is
the terrain D435 and it is already live):

| topic | type | notes |
|---|---|---|
| `/targeting_camera/targeting_camera/image_raw` | `sensor_msgs/Image` | `bgr8` — what YOLO subscribes to |
| `/targeting_camera/targeting_camera/camera_info` | `sensor_msgs/CameraInfo` | `plumb_bob`; **zeros until calibrated** |

Settings live in [`config/targeting_camera.yaml`](config/targeting_camera.yaml),
which explains why each one is what it is.

## Find the camera first

```bash
v4l2-ctl --list-devices          # which /dev/videoN is the webcam
v4l2-ctl -d /dev/video0 --list-formats-ext   # which geometries/rates it really offers
```

Two traps worth knowing before they cost you an hour:

- **`/dev/videoN` numbering is not stable across reboots.** With the D435 also
  on the bus, the numbers can swap. If that happens, point `device` at a
  `/dev/v4l/by-id/usb-<vendor>_<model>-video-index0` path instead, which is
  stable.
- **A webcam will silently give you 10 fps instead of 30** if it cannot fit the
  requested mode down the link. That is why the node asks for MJPG rather than
  the YUYV default, and why it logs the geometry and rate the camera *actually*
  accepted at startup. Read that log line — do not assume the request took.

## Calibration

`fx`/`fy` ship as zero, meaning uncalibrated, and the node warns about it at
startup. This matters more than it looks: `targeting_controller` turns a bbox offset
into a pan/tilt angle, and without a focal length that conversion has nothing to
stand on — the gimbal will move, but not to the right place.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/targeting_camera/targeting_camera/image_raw \
  camera:=/targeting_camera/targeting_camera
```

Paste the resulting `fx`, `fy`, `cx`, `cy` and distortion into the params file.

## Check it in RViz

```bash
ros2 topic hz /targeting_camera/targeting_camera/image_raw
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map targeting_camera_optical_frame
rviz2
```

Set **Fixed Frame** to `map`, then **Add → By topic →** pick `image_raw` →
**Image**. If the display stays blank while `topic hz` shows frames, set the
display's **Reliability Policy** to **Best Effort** — this node publishes on
sensor-data QoS, and that mismatch is the most common cause of an empty Image
panel.

## Design notes

- **`bgr8`, not `rgb8`.** OpenCV already holds BGR and a cv2-based detector
  wants BGR back, so nothing in this path pays for a colour conversion.
- **One capture buffer.** `read()` returns the newest frame rather than the
  oldest queued one. Aiming at where the target was four frames ago is the
  failure mode this package exists to avoid.
- **No TF.** Nothing in the targeting graph transforms into the robot body, and
  the terrain camera already owns the body-relative extrinsics.
- **Timestamps are capture-time on the node clock**, stamped when the frame is
  read, not when the sensor exposed it. A USB webcam gives no better.

## Not proven yet

Written and reviewed, but **not run against a camera or a ROS install** — there
is neither in the environment it was written in. The logic is unit-tested
([`test/test_usb_camera_node.py`](test/test_usb_camera_node.py)) with OpenCV and
rclpy stubbed out, which covers the message construction and the intrinsics
defaulting but proves nothing about V4L2 behaviour on real hardware. Expect the
first run on the RPi to be where a real webcam disagrees about formats.

The brief's open question about USB/CPU budget still stands: this camera shares
a bus and an RPi with the terrain D435, and "two cameras plus YOLO on one RPi 5"
has never been measured. Test the combination early.
