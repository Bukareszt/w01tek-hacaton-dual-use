# wojtek_targeting

The targeting gimbal: a DynamixelSDK driver for the XM540 pan / XM430 tilt
pair, and the controller that turns detections into bounded pan/tilt commands.

This package is the **only** thing that commands those two servos. The detector
publishes `LaserTarget` and nothing else; the manual-angle service is refused
while tracking is on. There is no laser code here and there should never be —
the laser is armed by hardware outside this graph, and ROS is not allowed to be
the only thing between a detection and an energised beam.

Nothing here touches the walking stack: not `ros2_control`, not the MD80
interface, not the 400 Hz loop, not arming.

```
USB webcam ──▶ detector ──▶ /targeting/target ──▶ targeting_controller ──▶ U2D2 ──▶ XM540 (ID 1)
(wojtek_targeting_camera)   (someone else's)      (this package)                     XM430 (ID 2)
```

## Run

```bash
ros2 launch wojtek_targeting_camera targeting_camera.launch.py   # the camera
ros2 launch wojtek_targeting targeting.launch.py                 # this
```

Tracking starts **disabled** and the servos hold the position they were already
in. Nothing moves until you say so:

```bash
ros2 service call /targeting/enable_tracking std_srvs/srv/SetBool "{data: true}"
```

| topic | type | |
|---|---|---|
| `/targeting/target` | `wojtek_targeting_msgs/LaserTarget` | in, from the detector |
| `/targeting/status` | `wojtek_targeting_msgs/TargetingStatus` | out, mode and why |
| `/targeting/gimbal_state` | `sensor_msgs/JointState` | out, rad / rad·s⁻¹ / amps |

| service | type | |
|---|---|---|
| `/targeting/enable_tracking` | `std_srvs/SetBool` | also clears a fault |
| `/targeting/home` | `std_srvs/Trigger` | refused while tracking |
| `/targeting/set_manual_angles` | `SetGimbalAngles` | bench only, refused while tracking |

## Bring it up in this order

Do not skip to the end. Each step is the one that catches a different mistake,
and the expensive mistakes all live in the first three.

**1. Find the adapter and give it a stable name.**

```bash
ls /dev/ttyUSB*
sudo cp udev/99-wojtek-gimbal.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/wojtek_gimbal
```

`/dev/ttyUSB0` is whatever enumerated first, which with cameras on the same Pi
is not reliably the U2D2.

**2. Confirm the servos answer, before any ROS.** Use Dynamixel Wizard, or
ROBOTIS' `ping` example. You are checking three things: both servos respond,
their IDs really are 1 and 2, and the bus baud rate is what you think. **A
wrong baud rate looks exactly like a dead bus** — same silence, same timeout —
and is the single most common reason a first bring-up goes nowhere. 57600 is
the ROBOTIS factory default; servos that came off another project are often at
1000000.

**3. Set the mechanical zero and the directions.** With the gimbal assembled
but tracking disabled:

```bash
ros2 topic echo /targeting/gimbal_state
ros2 service call /targeting/set_manual_angles wojtek_targeting_msgs/srv/SetGimbalAngles \
  "{pan_deg: 10.0, tilt_deg: 0.0}"
```

Pan should move **left-to-right positive** and tilt **up positive**. If either
goes the wrong way, flip `pan_direction`/`tilt_direction` in the config — do
not compensate for it anywhere else. Then point the gimbal where you want zero
to be, read the reported angle, and correct `pan_center_raw`/`tilt_center_raw`
by `angle × 4096 / 360`.

**4. Tighten the limits.** The shipped limits are guesses. Walk each axis out
to where the camera mount is about to hit something, and set the limits inside
that. They are enforced in the driver, in the controller, and in the servo's
own profile — but only against the numbers you give them.

**5. Calibrate the camera** (see
[`wojtek_targeting_camera`](../wojtek_targeting_camera/README.md)). Until `fx`
and `fy` are real the controller refuses to track at all, and says so on
`/targeting/status`. This is deliberate: a pixel offset carries no angle
without a focal length, so there is nothing to aim with.

**6. Trim the aim.** Put a target at the distance the demo is actually shot
from, track it, and adjust `pan_offset_deg`/`tilt_offset_deg` until the laser
lands where the crosshair is. This is also where camera-to-laser parallax gets
absorbed — see the note in `kinematics.py` about why it is not modelled.

## Modes

```
DISABLED ──enable──▶ SEARCHING ──fresh target──▶ TRACKING ──dwell──▶ LOCKED
   ▲                     ▲           │                │                 │
   │                     └──timeout──┘                └────timeout──────┘
   └──────── disable, or any bus failure (──▶ FAULT) ───────────────────┘
```

- **SEARCHING** holds the last aim rather than homing. A detector that blinks
  for three frames should not throw the aim away and start over.
- **FAULT** latches. A write to a servo failed, so the controller no longer
  knows what the gimbal is doing and stops commanding it — while leaving torque
  on, because a head-heavy assembly dropping limp is worse than one holding a
  stale pose. Toggle `enable_tracking` off and on to retry.
- The **target timeout** (200 ms) is the dead-man. If the detector dies
  mid-track the gimbal stops within a frame or two instead of continuing to aim
  at a box that is no longer there.

## What was carried over from the Bathymetry manipulator

That project ran this servo family on this kind of bus and paid for these in
hardware. They are in `dynamixel_driver.py` as bug fixes, not as caution:

- **A failed read must never look like a successful zero.** Returning 0 on a
  dropped packet put "encoder zero" into ~1.6% of position samples there — a
  jump of the whole mounting offset. On a gimbal that is a whip to the end
  stop, not a bad sample.
- **`PRESENT_CURRENT` is a signed int16.** Read unsigned, a small negative
  current comes back as ~65500.
- **The position PID registers are RAM.** A power cycle silently restores the
  factory 800/0/0, so they are rewritten on every connect.
- **Profile velocity 0 means "no profile", not "do not move"** — the servo
  slews to every goal at maximum speed. The driver refuses to start with it.
- **Torque must not come on until `GOAL_POSITION` holds the present position.**
  It is RAM that survives torque-off, so enabling torque first snaps the gimbal
  to wherever the last session left it.

Nothing was copied: no Bathymetry package, `ros2_control` plugin, URDF or
launch file is in this tree. The two additions on top of the reference are a
sync write for the goals (so pan and tilt start on the same packet instead of
tilt trailing by a transaction) and a single 10-byte block read for
current/velocity/position, which are contiguous at 126–135. At 57600 baud the
naive version spends more than a control period on state alone.

## Tests

```bash
cd experiments/wojtek_targeting/ros/src/wojtek_targeting
PYTHONPATH=.:test python -m pytest test/ -q
```

64 tests, no hardware and no ROS: the SDK, rclpy and the clock are all faked
(`test/conftest.py` says exactly what that leaves unproven). They cover the
angle conversions, the limit clamping, the read-failure handling, the safe
startup sequence and every state transition — which is most of what there is to
get wrong that is not the hardware's own behaviour.

## Not proven yet

**Written and reviewed, never run against a servo, a U2D2 or a ROS install** —
there is none of the three in the environment this was written in. Expect the
first bring-up on the bench to be where a real XM540 disagrees. The likeliest
places: the baud rate, the two `direction` signs, and whether the tilt axis
actually reaches the limits in the config.

Also still open, from the [experiment brief](../../README.md):

- The URDF (`targeting_gimbal.urdf.xacro`) does not exist yet. `gimbal_state`
  publishes joint names that nothing consumes until it does.
- `SweepGimbal.action` from the brief is not implemented. It is calibration
  tooling and nothing in the aiming path needs it.
- Two cameras plus YOLO on one RPi 5 has still never been measured. This node
  is cheap — one serial transaction per tick at 40 Hz — but it shares a Pi with
  everything that is not.
