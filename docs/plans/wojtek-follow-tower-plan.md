# Wojtek follow with the tower camera: lock on the tower, walk with the body

Status as of 2026-09-12. A plan, nothing built yet. It extends the follow
v1 plan (machinekind/w01-tek PR 26), which stands where this doc is silent.

## The goal

The tower camera sees an object. The operator taps it on the Deck. The
tower locks onto it and keeps it in the picture while the whole robot
turns and walks toward it. The robot holds about one metre and does not
walk into furniture. The lock survives the turn, the walk, and the target
leaving the front camera's view.

## The decision in short

The tower camera provides the bearing. The front camera provides the range
and the obstacles. The gimbal loop runs on the robot with gyro
feed-forward, so the lock holds through body motion without waiting for
the Deck. The body loop is the follow v1 reactive controller with one
change: the bearing comes from the pan angle plus the pixel offset, not
from the front camera's pixel. No map and no pose.

## Facts from the checkouts that shape the plan

- Two cameras. The RealSense D435 is fixed to the front of the body and
  publishes raw depth at 424×240 and 15 Hz plus colour. The tower camera
  is a generic UVC webcam at 640×480 and up to 30 Hz, on a pan axis with
  180° each way and a tilt axis. Its driver is PR 6 in this repo. It ships
  uncalibrated, publishes no transform, and the node that would aim the
  tower refuses to track until `fx` is real.
- The tower team's gimbal node is in PR 6: a Dynamixel driver and a
  `targeting_controller` that takes a normalized detection, aims the
  tower itself, and reports the angles it holds. On the bench the pan
  servo answers on the bus and lands on commanded positions, but its
  physical rotation is unconfirmed by eye, and the tilt servo does not
  answer yet.
- The lock-in lives on the Deck page (PR 2). A tap picks the YOLOX box,
  the page tracks it, and it streams `{"t":"track", cx, cy, w, h, fw, fh,
  label, age}` at 10 Hz and one `{"t":"unlock"}`. The page works on
  whatever picture the gateway's MJPEG stream carries.
- YOLOX-nano runs in the Deck page, not on the robot. The robot is a
  Raspberry Pi 3 B+ with 1 GB. Cores 2 and 3 belong to the control loop.
  Cores 0 and 1 carry the system, USB, the access point, the bridge and
  the gateway. Nobody has measured the headroom.
- The Deck to robot round trip is 120 to 250 ms. A loop closed through the
  Deck is a 10 Hz loop with a quarter second of delay.
- The robot publishes IMU angular velocity from the I2C IMU at the control
  rate. The walking policy takes `(vx, vy, wz)` on `/cmd_vel`. The
  gateway's `DriveGate` is the single `/cmd_vel` publisher in the Deck
  setup and holds the dead-man.
- There is no odometry and no pose estimator. The follow v1 controller
  needs neither.
- The laser is out of this plan. It belongs to the targeting experiment
  and follows the event's rules. This plan ends at a tower pointed at the
  target and a body standing one metre from it.

## Split between machines and packages

1. The Deck page designates and tracks, as in PR 2. It gains a camera
   selector. The tower picture is the default when the tower camera is
   up. The track message gains `"cam": "tower"` or `"front"`.
2. The gateway serves the tower camera as a second MJPEG stream at
   `/stream.mjpg?cam=tower`, on the same JPEG path as the front camera.
   It publishes the track as `std_msgs/String` on `/wojtek/track/target`,
   the page's JSON plus a `stamp` of the receive time, and `{"unlock":
   true}` once when the lock ends. A plain string, because the robot's
   package list is fixed and offline, and the repository already uses
   one for `/wojtek/nav_command`. The `DriveGate` gets the follow node as
   a second source, as in follow v1.
3. The gimbal node is the tower team's `targeting_controller` from PR 6.
   It takes a detection, not angles. It runs its own aim loop at 40 Hz
   and reports the angles it holds. It knows nothing about the IMU or
   the body.
4. The follow node is new and runs two loops. The aim relay turns the
   10 Hz Deck track and the IMU into a 40 Hz predicted target for the
   gimbal node. The body loop turns the bearing, the range, and the
   obstacles into `/cmd_vel` at 10 Hz. It subscribes to the track, the
   gimbal state and status, the tower `camera_info`, the D435 depth and
   its `camera_info`, and the IMU. It publishes the gimbal's target,
   `/wojtek/follow/cmd_vel`, and a status with the state, the bearing,
   the range, and the nearest obstacle. Its maths lives in modules
   without ROS, tested on synthetic inputs.

## The gimbal contract, as PR 6 wrote it

The gimbal node runs under the `targeting` namespace.

- In: `/targeting/target`, `wojtek_targeting_msgs/LaserTarget`. The box
  centre as `target_x` and `target_y`, normalized to −1..1 across the
  image, with `detected`, `class_name`, `confidence`, and a stamp. The
  node drops to searching and holds its pose when the stamp is older
  than 0.2 s, when `detected` is false, or when `confidence` is under
  0.65. Its aim loop takes 0.6 of the optical error per tick, rate
  limited to 45°/s pan and 35°/s tilt, clamped to ±80° pan and −35..45°
  tilt by default.
- In: `/targeting/camera_info`, remapped to the tower camera. Without a
  real focal length the node refuses to track.
- Out: `/targeting/gimbal_state`, `sensor_msgs/JointState` with
  `targeting_pan_joint` and `targeting_tilt_joint` in radians, at 20 Hz.
- Out: `/targeting/status`, `wojtek_targeting_msgs/TargetingStatus` with
  the mode, the requested and actual angles in degrees, and the
  detection age, at 40 Hz.
- Service: `/targeting/enable_tracking`, `std_srvs/SetBool`. The follow
  node calls it true when a lock starts and false when it ends.
- Optical convention in its kinematics: yaw positive means the target is
  right of the axis, pitch positive means below it. Which way positive
  pan and tilt turn the tower is a mounting question not yet confirmed
  on the bench. The follow node carries `pan_sign` and `tilt_sign`
  parameters, +1 meaning positive pan turns the tower left and positive
  tilt points it up, the ROS right-hand convention.

The pan range in PR 6 is ±80°, not the 180° each way the tower was
described with. One of the two is wrong and the bench settles it.

## What the aim relay computes

- Pixel error. The last Deck box goes through the tower intrinsics and
  gives an optical yaw and pitch at the track's time.
- Prediction. Between Deck tracks the relay integrates the body gyro. A
  body yaw to the left moves a fixed target right in the picture. A pan
  to the left moves it right as well, because both turn the camera. The
  predicted yaw is the yaw at the track time, plus the body yaw since,
  plus the pan change since. Pitch is the same with the body pitch and
  the tilt change. The prediction goes back to a normalized point and
  out as a `LaserTarget` at 40 Hz. The track is anchored at its capture
  time, the Deck stamp minus its age minus a measured round-trip
  latency, so the gyro is charged from when the picture was taken.
- Sign. The gimbal node reports the pan in the sense of its own
  `pan_direction` setting. The follow node's `pan_sign` must equal minus
  that setting, and the same for tilt. That is a derivation from PR 6's
  code, not a bench fact. The bench confirms `pan_direction` itself.
- Why. The gimbal node has no IMU term and a 0.2 s freshness rule. Fed
  only the 10 Hz Deck track with a quarter second of delay, it would
  drop to searching between frames and chase where the target was
  during a body turn. Fed the prediction at 40 Hz, it sees a fresh
  target that already accounts for the body's motion.
- Coast. While the track's `age` grows past 0.7 s the relay sends the
  prediction with zero confidence, which the gimbal node reads as hold.
  After 3 s the lock is dropped.

## What the follow node computes

- Bearing in the body frame. Azimuth is the pan angle plus the azimuth
  error from the pixel. Elevation is the tilt angle plus the elevation
  error. Both are read at the track's stamp, so a pan mid-motion still
  gives the right bearing.
- Range. When the azimuth is inside the D435's field of view, the bearing
  ray is projected into the depth image after the tower to D435 offset,
  and the range is the low quantile of a window around it, zeros dropped.
  That is the follow v1 method with a different ray. Outside that field
  of view the range comes from the box width and a per-label width table,
  a person at 0.45 m and a bottle at 0.07 m. That range steers the turn
  only and never the approach.
- Floor, own legs, obstacles. Unchanged from follow v1. The three-band
  gate runs on the D435 depth. The target's cone is cut out to the
  target's range.
- Controller. Yaw rate is a gain times the body azimuth with a 2° dead
  band and a 0.7 rad/s limit. Above 30° of azimuth the robot turns in
  place first. Forward speed is a gain times the range error from 1.0 m
  with hysteresis, and it is zero until a D435 range exists. Reverse is
  limited to 0.2 m/s. The obstacle gate zeroes or scales the forward
  speed and sidesteps as in follow v1.
- Target loss. The body holds heading for 1 s, then stops. After 3 s the
  lock is dropped and the page shows lost.

The difference from follow v1 is one line of geometry and one large
change in behaviour. The target can leave the front camera's view during
a turn, a sidestep, or a walk past an obstacle, and the lock holds.

## Calibration and geometry

- Tower intrinsics. One checkerboard session with `cameracalibrator`
  gives `fx`, `fy`, `cx`, `cy`. Until then the node refuses to track.
- Mount geometry. The tower's position on the body and the pan zero
  direction, measured with a tape. The pan zero must point where the body
  points. A 2° error in the zero becomes a 2° bearing error for ever.
- Tower to D435 offset. The tower sits above and behind the D435. The
  range projection needs that offset. At one metre a 10 cm vertical
  offset is under 6° in elevation, which the window absorbs.

## Where it lives

- The gimbal node lives where the tower team puts it, in the targeting
  experiment next to the camera driver from PR 6.
- `experiments/wojtek_follow_v1/`, the follow node with both loops, as in
  the follow v1 plan.
- Three things change in `ros/`. The page gets the camera selector and
  the `cam` field. The gateway gets the second stream, the track topic,
  and the drive mux. The `vision_msgs` dependency arrives. The gateway
  imports nothing from either experiment.
- After a working demo the follow node moves to `ros/src/wojtek_follow`.

## Order of work

1. The contract. One PR with the topic names, the message types, the
   frame names, and the width table. Half a day. This unblocks the gimbal
   side and the follow side at once.
2. Calibration and mount geometry. Half an hour with a checkerboard and a
   tape. Record the numbers in the camera config.
3. Gateway and page. The second stream with byte passthrough, the camera
   selector, the `cam` field, the track topic, the drive mux with unit
   tests. One day. Measure the USB bus with both cameras streaming and the
   headroom on cores 0 and 1 with the walking loop live.
4. Aim loop. The maths with tests on synthetic tracks and IMU rates, then
   the node publishing gimbal targets. Bench with the tower team's node:
   lock on a can, rotate the robot body by hand, the tower holds the can
   in the picture. Half a day of code, then the integration.
5. Body loop. The core with tests on synthetic depth and synthetic gimbal
   states, then the rest of the node. The first run is in simulation with
   the gimbal state fixed at zero, which is follow v1 exactly. Two to
   three days.
6. Robot on a leash. Bearing while standing, with the target at 0°, 90°
   and 150°. Range to a standing target. A box in the path. A person with
   the can walking a circle around the robot. One day.

## Risks

- Pi 3 headroom. Two cameras on one USB2 bus, two MJPEG streams, and two
  new nodes on cores 0 and 1. Byte passthrough for the tower stream is
  the mitigation. Measure before the leash test.
- Gimbal hardware. The pan servo's rotation is unconfirmed by eye and
  the tilt servo does not answer. Until both move, the follow chain can
  only be tested with the gimbal state held at zero, which is follow v1.
- Freshness. The gimbal node holds pose when a target is older than
  0.2 s. The relay must keep publishing at 40 Hz through every Deck gap,
  and its stamp must be the robot's clock, not the Deck's.
- Frame sign. A pan that turns right on positive angle flips the body
  loop into a runaway. The bench test in step 4 catches it: turn the body
  by hand and watch the tower counter-rotate.
- Gimbal slop. Backlash and servo lag put noise on the pan angle, which
  becomes noise on the bearing. The 2° dead band and the yaw limit cover
  a few degrees. More needs a stiffer mount.
- Range without depth. Behind the robot the range is a guess from the box
  width. The controller turns first and walks only on D435 range, so a
  bad guess costs time and not a collision.
- Detection range. COCO has no can. A 0.5 l can is seen inside about
  1.5 m. A person is seen across the room. Pick the demo target with
  that in mind.
- The follow v1 risks stand: legs swing wide, the D435 sees nothing under
  0.3 m and nothing to the sides, and a can in a hand means the robot
  follows the person.
