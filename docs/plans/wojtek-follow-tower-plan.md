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
- The tower team is building the gimbal node. It takes two angle targets,
  pan and tilt, and moves the servos. Whether it reports the angles back
  is open. The contract below assumes it does. Without feedback the
  follow node assumes the last target was reached after a fixed servo
  lag, which is worse but works.
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
2. The gateway serves the tower camera as a second MJPEG stream. The
   camera node publishes the webcam's own MJPG frames as
   `CompressedImage`, and the gateway forwards those bytes without
   decoding and re-encoding. That keeps the second stream cheap on cores
   0 and 1. The gateway publishes the track as `vision_msgs/Detection2D`
   on `/wojtek/track/target`. The `frame_id` names the camera. The
   `DriveGate` gets the follow node as a second source, as in follow v1.
3. The gimbal node belongs to the tower team. It takes pan and tilt
   targets on `/wojtek/gimbal/target`, moves the servos, and reports the
   angles on `/wojtek/gimbal/state`. It knows nothing about cameras,
   boxes, or the IMU.
4. The follow node is new and runs two loops. The aim loop turns the
   tracked box and the IMU into gimbal targets at 50 Hz. The body loop
   turns the bearing, the range, and the obstacles into `/cmd_vel` at
   10 Hz. It subscribes to the target, the gimbal state, the tower
   `camera_info`, the D435 depth and its `camera_info`, and the IMU. It
   publishes `/wojtek/gimbal/target`, `/wojtek/follow/cmd_vel`, and a
   status with the state, the bearing, the range, and the nearest
   obstacle. Its maths lives in a module without ROS, tested on synthetic
   inputs.

The gimbal side and the follow side meet at two topics. Either side can
be built and bench tested without the other.

## The gimbal contract, assumed until the tower team confirms it

- `/wojtek/gimbal/target`, `sensor_msgs/JointState`. `name` is
  `["tower_pan", "tower_tilt"]`, `position` is the two angles in radians,
  `velocity` and `effort` are empty. Sent at 50 Hz. The gimbal moves to
  the newest target at its own rate and ignores nothing.
- `/wojtek/gimbal/state`, the same message with the angles the servos
  actually hold, stamped, at 20 Hz or better.
- Frame. Pan zero points where the body points. Positive pan turns the
  tower left, counterclockwise seen from above. Tilt zero is level.
  Positive tilt points up. This is the right-hand rule on a z-up body,
  the ROS convention, so nobody has to negate anything.
- Range. Pan runs from −π to π. Tilt runs to the mount's limits, which
  the tower team states. A target past a limit is clamped by the gimbal
  and the clamped value shows in the state.
- If the state topic does not exist, the follow node estimates the angles
  as the target delayed by a fixed servo lag, a parameter measured on the
  bench.

## What the aim loop computes

- Pixel error. The box centre goes through the tower intrinsics and gives
  an azimuth and elevation error in the camera frame.
- Target. The pan target is the current pan plus a gain times the azimuth
  error, minus the body yaw from the IMU integrated since the last Deck
  track. The tilt target is the same with the elevation error and the
  body pitch. The IMU term cancels body motion at the IMU's rate. The
  Deck's 10 Hz track only corrects the slow residual, so a gain of 1.5
  with a 250 ms delay keeps margin, as in the follow v1 bearing loop.
- Limits. Pan targets stop 5° short of each end. When the target sits
  past an end, the tower holds the end and the body turn brings it back.
- Coast. While `age` grows past 0.7 s the target holds. After 3 s the
  target returns to centre.

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
- Gimbal feedback. Without the state topic the bearing rests on an
  assumed servo lag. A slow or loaded servo then puts a lag error on the
  bearing. Ask the tower team for the state topic in step 1.
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
