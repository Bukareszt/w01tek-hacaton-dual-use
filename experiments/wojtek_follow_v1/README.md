# Experiment: follow with the tower camera, v1

> **Status: EXPERIMENTAL. Not production, not on the robot.**
> Nothing here is deployed by `ros/deploy.sh`, and no package here is a
> dependency of `wojtek_bringup`. Treat every interface in this directory as
> unstable: names, topics and message fields change without a migration path.

The operator taps a target on the Deck. The tower camera locks onto it and
keeps it in the picture while the whole robot turns and walks toward it. The
robot holds about a metre and does not walk into the furniture.

The plan this implements is
[docs/plans/wojtek-follow-tower-plan.md](../../docs/plans/wojtek-follow-tower-plan.md).
The gimbal contract in that document is superseded by PR 6, which is what this
package is written against.

## Status

| part | built | tested | on the robot |
|---|---|---|---|
| `core/geometry.py`, the pinhole and the bearing | yes | model-free unit tests | no |
| `core/aim.py`, the 40 Hz relay with gyro feed-forward | yes | model-free unit tests | no |
| `core/rangefinder.py`, depth range and the box fallback | yes | synthetic depth images | no |
| `core/gate.py`, the three-band obstacle gate | yes | synthetic depth images | no |
| `core/controller.py`, the body loop | yes | every state and limit | no |
| `follow_node.py`, the ROS node | yes | callbacks over stubbed ROS | no |
| the whole chain against the real gimbal | yes | **no** | no |
| the whole chain in simulation | yes | **no** | no |

Nothing in this package has run on the robot, in simulation, or against the
real targeting controller. The tests are model-free and they use stubbed
messages. They say the arithmetic is self-consistent. They say nothing about
whether the servos turn the way this package assumes, which is the one thing
the bench check below has to settle first.

## The contract

Subscribed:

| topic | type | what it is |
|---|---|---|
| `/wojtek/track/target` | `std_msgs/String` | the Deck's lock as JSON, republished by the gateway |
| `/targeting/gimbal_state` | `sensor_msgs/JointState` | pan and tilt in radians, at 20 Hz |
| `/targeting/status` | `wojtek_targeting_msgs/TargetingStatus` | the gimbal's mode, at 40 Hz |
| `/targeting_camera/targeting_camera/camera_info` | `sensor_msgs/CameraInfo` | the tower camera's intrinsics |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | D435 depth, 16UC1 millimetres, 424x240 at 15 Hz |
| `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | the depth stream's intrinsics |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | body angular rates, x forward, y left, z up |

Published:

| topic | type | rate | what it is |
|---|---|---|---|
| `/targeting/target` | `wojtek_targeting_msgs/LaserTarget` | 40 Hz | where the target is in the tower picture now |
| `/wojtek/follow/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | one drive source for the gateway's DriveGate |
| `/wojtek/follow/status` | `std_msgs/String` | 10 Hz | JSON: state, bearing, range, bands, track age |

Called: `/targeting/enable_tracking` (`std_srvs/SetBool`), true when a lock
starts and false when it ends. The call is asynchronous, because blocking on
it would stop both loops until the gimbal answered.

The track JSON is the Deck's own message plus two fields the gateway adds:

```json
{"cx": 320, "cy": 240, "w": 60, "h": 60, "fw": 640, "fh": 480,
 "label": "bottle", "age": 0.0, "cam": "tower", "stamp": 1727000000.0}
```

`cam` says which camera the operator was looking at. `stamp` is the gateway's
receive time on the robot's clock, in seconds. The end of a lock is
`{"unlock": true, "stamp": ...}`.

Everything this node touches is in the `targeting` namespace or under
`/wojtek/follow/`. It never publishes `/cmd_vel`, never talks to
`ros2_control`, and arms nothing. The single `/cmd_vel` publisher in the Deck
setup is the gateway, and the dead-man that stops the robot lives there. While
the follow state is `IDLE` this node publishes no velocity at all, so a follow
node that has nothing to say is indistinguishable from one that is not running,
which is what the dead-man is for.

## Sign conventions, and the bench check that has to come first

Two frames meet in this package.

The optical frame is the tower team's. Yaw is positive when the target is to
the right of the optical axis. Pitch is positive when it is below the axis,
because image rows run downward.

The body frame is ROS on a z-up body. x is forward, y is left, z is up.
Azimuth is positive counterclockwise seen from above, so positive azimuth is to
the robot's left. Elevation is positive up. Positive `wz` is the body turning
left. Positive `wy` is the body pitching nose down.

The two are tied together by the gimbal angles, and this is the whole of it:

```
yaw   = pan_sign  * pan  - azimuth_target
pitch = tilt_sign * tilt - elevation_target
```

`pan_sign` is +1 when a positive pan angle turns the tower to the body's left.
`tilt_sign` is +1 when a positive tilt angle points it up. Both are parameters,
and both have a derivable default.

The targeting controller adds its optical error times its `pan_direction`
parameter to its pan command, and its Dynamixel driver multiplies that command
by `pan_direction` again on the way to the encoder (`deg_to_raw` is
`center + direction * counts`, `raw_to_deg` is `direction * (raw - center)`).
The two multiplications cancel. A target to the right of the picture therefore
always raises the raw encoder count, whatever `pan_direction` is, and
`pan_direction` only flips the sign of the angle the controller reports on
`/targeting/gimbal_state`. For the controller's own loop to converge at all,
raising the count must turn the camera to the right. A positive reported pan is
then camera right when `pan_direction` is +1, which in this package's
convention is `pan_sign = -1`. The same argument one axis over gives
`tilt_sign = -tilt_direction`. The controller's defaults are `pan_direction =
+1` and `tilt_direction = -1`, so the defaults here are `pan_sign = -1` and
`tilt_sign = +1`. Worth telling the tower team: their `direction` parameters
cannot change which way a servo answers an optical error, only the sign of the
reported angle.

What the bench check settles is whether the tower loop converges at all, that
is, whether raising the raw count really turns the camera the way the optical
error says. If it does, the signs above follow. If it does not, the tower loop
runs away on its own before this package gets a say, and the mount or the
`direction` parameters need attention on their side.

Every other sign in the package is derived from the two identities above,
including the two that matter most:

```
yaw_now   = yaw_0   + dpsi   + pan_sign  * (pan_now  - pan_0)
pitch_now = pitch_0 - dtheta + tilt_sign * (tilt_now - tilt_0)
```

Turning the body left and panning the tower left both turn the camera left, and
both push a fixed target toward the right of the picture. They have the same
sign. The plan document writes the pan term with a minus and contradicts itself
one section later; the plus is what the derivation and the bench check both
give. `test/test_aim.py` proves it three ways, and the middle one is the bench
check written down: body left ten degrees, tower right ten degrees, predicted
pixel unmoved.

**Verify this on the bench before the robot walks.** Lock onto something, turn
the body by hand, and watch the tower. It must counter-rotate and hold the
target in the picture. If it turns the other way, `pan_sign` is wrong, and a
wrong `pan_sign` turns the follow into a runaway: the body chases a bearing
that is running away from it. The same check one axis over settles `tilt_sign`,
by pitching the body and watching the tilt.

## How it works

**The aim relay, at 40 Hz.** The tower team's controller holds its pose
whenever the target it was given is older than 0.20 s, and it has no gyro term.
Fed the Deck's 10 Hz track directly, with a Deck round trip of 120 to 250 ms,
it would see a fresh target for one tick in four and chase a box that is
already a quarter second old. The relay keeps the last track, integrates the
body gyro and the gimbal's own motion since that track's picture was taken,
and publishes where the target is in the tower picture now. While the Deck's
own detection age stays under 0.7 s the target goes out at confidence 1.0.
Past that it still goes out at 40 Hz, at confidence 0.0, which the controller
reads as a target not worth moving for, so the gimbal holds instead of
searching.

**When the picture was taken.** The gateway stamps a track with its receive
time. The box in it was matched `age` seconds before the page sent it, on a
frame captured one round trip before that. The integrals start at
`stamp - age - track_latency_s`, not at the stamp. Starting at the stamp
charges everything the body did during the round trip to after the picture,
which at 40 deg/s is 5 to 10 degrees of bearing error and a gimbal that hunts
at the period of the round trip. `track_latency_s` is a parameter; measure it
with a clap test. A coasting page resends the same box with a growing `age`.
That message carries no new picture, so the relay keeps its anchor and the
integral keeps running from the picture it belongs to, instead of restarting
from a time the box does not belong to and throwing away the body rotation
since the last real match.

**The bearing.** The predicted optical angles and the current gimbal angles
give the target's azimuth and elevation in the body frame. Because the
prediction already carries the gimbal's motion since the track, a pan in
progress cancels out and the bearing is right in the middle of one. A
`gimbal_state` older than `gimbal_max_age_s` is not the gimbal's current
angle: the bearing then uses the pan the track was anchored on, and the node
warns. A targeting controller that faults or restarts mid-lock reports
`DISABLED` or `FAULT` on `/targeting/status`; once that report outlives the
node's own request by `tracking_reconcile_s`, the next track asks it to track
again, which on the controller's side also clears a latched fault and retries
the bus.

**The range.** When the bearing is inside the depth camera's view the ray is
projected into the depth image and the range is the 20th percentile of the
nonzero depths in a 24 px window, divided by the ray's own cosine because the
image stores depth and the controller wants distance. Otherwise the range comes
from the box width and a per-label width table, and it is marked `bbox`. A
`bbox` range steers the turn and never the walk. A depth frame older than
`depth_max_age_s` is withheld from the body loop altogether: a RealSense that
drops off the bus must not leave a frozen range driving the robot into the
person it was following, so the range falls back to `bbox` and the bands empty
until a fresh frame arrives.

**The obstacles.** Every fourth depth pixel is back-projected into the body
frame. Points below 0.06 m are the floor, points above 0.35 m are walked under,
points nearer than 0.3 m from the sensor are inside the D435's blind zone.
Heights are measured from the floor and ranges from the sensor, which is the
same range the rangefinder reports for the target. What is left is split into a
left band, a centre band and a right band by azimuth, and each band reports its
nearest point. The target's own cone is cut out first, because the target is an
obstacle by every other measure and without the cut the robot would refuse to
approach the thing it is following.

**The body loop, at 10 Hz.** `IDLE` until a lock. `WAITING` while a lock is
held but the tower camera has published no usable intrinsics, so there is no
bearing; nothing is published, the status says what is missing, and the loss
timers run so a lock the Deck stops feeding is still dropped. `ALIGN` turns in
place while the target is more than 30 degrees off the nose. `APPROACH` drives
the range error toward a metre. `HOLD` stands still inside the hysteresis band,
which doubles once inside it so the robot does not shuffle. `BLOCKED` scales
the forward speed down between 1.0 m and 0.5 m, kills it inside 0.5 m, and
steps toward whichever side band is emptier. `LOST` stops the body after a
second without a fresh detection and drops the lock after three.

## Running it

```bash
# The tests. No ROS, no robot, no cameras, about a tenth of a second.
./experiments/wojtek_follow_v1/run.sh test

# With an explicit interpreter, which is what a worktree needs.
EXP_PY=/path/to/python ./experiments/wojtek_follow_v1/run.sh test

# The build. Needs a sourced ROS 2 and the targeting experiment beside this
# one, for wojtek_targeting_msgs.
./experiments/wojtek_follow_v1/run.sh build
```

On the robot, with the gimbal and the cameras already up:

```bash
ros2 launch wojtek_follow follow.launch.py cpus:=0,1
```

Cores 2 and 3 belong to the walking loop and nothing in this package may land
there.

## Known gaps

- **Nothing has run on hardware.** Not against the real gimbal, not in
  simulation, not on the robot. The pan servo's rotation is unconfirmed by eye
  and the tilt servo did not answer at all when PR 6 was written. Until both
  move, this chain can only be exercised with the gimbal state held at zero,
  which is plain follow v1 with a fixed camera.
- **No floor-plane fit.** The floor is wherever `depth_camera_height_m` and
  `depth_camera_pitch_deg` say it is. Both are parameters and both are
  guesses until someone measures them. A pitch that is wrong by twenty degrees
  makes the gate see nothing at all, which `test_gate.py` demonstrates.
- **The tower to D435 offset is ignored.** The tower sits above and behind the
  depth camera, and the range projection pretends it does not. At a metre a
  10 cm vertical offset is under 6 degrees of elevation, which the 24 px window
  absorbs. Closer than that it does not.
- **No odometry and no pose.** Everything is reactive and nothing is
  remembered between frames. An obstacle that leaves the depth camera's view
  stops existing. The robot cannot step around anything it cannot currently see.
- **Front-camera tracks are refused.** A front-camera pixel carries no tower
  bearing without the front camera's intrinsics and its mount transform, and
  this node has neither. A track with `"cam": "front"` is dropped with a
  warning.
- **The Deck's frame is assumed to be a resize of the camera's.** Scaling the
  intrinsics is exact for a resize and wrong for a crop.
- **The gyro is integrated open loop.** There is no drift correction and no
  attitude estimate. Over the 0.1 s between Deck tracks that is fine; over a
  three-second coast it is not, which is one more reason the lock is dropped
  at three seconds.
- **The Deck round trip is a constant.** `track_latency_s` stands in for a
  capture time the page does not send. The page could include the frame's
  capture time relative to its own send time, and the gateway could convert
  it; until it does, the constant has to be measured and set by hand.
- **`distance_m` on the gimbal's target is only ever a measured range.** A
  box-width guess is sent as 0.0, which the message defines as unknown.

## Layout

```
ros/src/wojtek_follow/
  wojtek_follow/core/      the arithmetic, numpy only, no ROS anywhere in it
    geometry.py            pinhole, normalized target, body bearing
    aim.py                 the 40 Hz relay and its two little histories
    rangefinder.py         depth range, and the box-width fallback
    gate.py                the three-band obstacle gate
    controller.py          one pure function, one tick of the body loop
  wojtek_follow/follow_node.py   the thin rclpy node: callbacks and two timers
  config/follow.yaml       every parameter, with a line each about what it does
  launch/follow.launch.py  params_file, namespace, sign overrides, cpu affinity
  test/                    model-free tests; conftest.py stubs ROS
run.sh                     test | build
```
