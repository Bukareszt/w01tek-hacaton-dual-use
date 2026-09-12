"""The aim relay: a 10 Hz Deck track becomes a 40 Hz target for the gimbal.

The tower team's controller holds its pose whenever the target it was given is
older than 0.20 s, and it has no gyro term of its own.  Fed the Deck track
directly it would see a fresh target for one tick in four and chase a box that
is already a quarter second old while the body turns underneath it.  This relay
stands between the two.  It keeps the last track, integrates the body gyro and
the gimbal's own motion since that track arrived, and hands the controller a
target that says where the target is in the tower picture now.

Sign derivation, from the two identities in `geometry.py`.

The optical yaw is the camera's azimuth minus the target's azimuth, both in the
body frame:

    yaw = pan_sign * pan - azimuth_target

The body turning left by `dpsi` moves a world-fixed target clockwise in the
body frame, so `azimuth_target` falls by `dpsi` and the yaw rises by `dpsi`.
Panning the camera left by `dpan` raises `pan_sign * pan` by `dpan`, so the yaw
rises by `dpan` as well.  Both motions turn the camera left and both push a
fixed target to the right of the picture:

    yaw_now = yaw_0 + dpsi + pan_sign * (pan_now - pan_0)

This is the term that makes the bench check work.  Turn the body left and the
gimbal has to pan right to hold the target, so the two terms cancel and the
predicted pixel does not move.  That cancellation is the whole point of the
relay and it only happens with these signs.

Note for the plan's readers: `docs/plans/wojtek-follow-tower-plan.md` writes the
pan term with a minus, and its own body-bearing formula contradicts it.  The
plus above is the one that survives the derivation and the bench check, and
`test/test_aim.py` proves it three ways.

Pitch is the same argument one axis over.  The optical pitch is the camera's
elevation minus the target's elevation, and it counts downward:

    pitch = tilt_sign * tilt - elevation_target

A positive body `wy` is nose down in ROS, and a nose-down body lifts a fixed
target in the body frame, so `elevation_target` rises by `dtheta` and the pitch
falls by it.  Tilting the camera up by `dtilt` raises `tilt_sign * tilt` and so
raises the pitch:

    pitch_now = pitch_0 - dtheta + tilt_sign * (tilt_now - tilt_0)

Confidence is the relay's way of telling the controller to hold.  While the
Deck's own detection age stays under `coast_s` the target goes out at
confidence 1.0.  Past that the prediction still goes out at 40 Hz, at
confidence 0.0, which the tower controller reads as a target not worth moving
for.  The gimbal then holds its pose instead of dropping into a search.

When the track was taken.  The gateway stamps a track with its receive time,
and the box in it was matched `age` seconds before the page sent it, on a
picture that was captured a Deck round trip before that.  The integrals start
at the capture time:

    t0 = stamp - age - track_latency_s

Starting them at the receive time instead charges everything the body did
during the round trip to after the picture, which hands the gimbal a target
that is behind by the whole delay.  A coasting page resends the same box with a
growing `age`; such a message carries no new picture, so the anchor is kept and
the integral keeps running from the picture it belongs to.
"""

from collections import deque
from dataclasses import dataclass

from .geometry import (
    Bearing,
    inside_image,
    optical_to_pixel,
    pixel_to_normalized,
    pixel_to_optical,
    scale_intrinsics,
)


@dataclass(frozen=True)
class AimParams:
    """Everything the relay needs that is not a measurement."""

    # +1 means a positive pan angle turns the tower to the body's left, and a
    # positive tilt angle points it up.  The defaults follow from the tower
    # controller's own driver: pan_sign = -pan_direction and tilt_sign =
    # -tilt_direction of its parameters, which default to +1 and -1.  See the
    # README for the derivation.  Verify on the bench before trusting any of
    # this: turn the body by hand and the tower must counter-rotate.
    pan_sign: float = -1.0
    tilt_sign: float = 1.0
    # How stale the Deck's own detection may be before the target goes out at
    # zero confidence.  The Deck page coasts a lock for the same 0.7 s.
    coast_s: float = 0.7
    # The Deck round trip: from the tower camera's frame to the track that
    # describes it arriving at the gateway.  Subtracted from the track's stamp
    # to find the moment the picture was taken.  Measure it with a clap test.
    track_latency_s: float = 0.15
    # A gimbal sample older than this is not the gimbal's current angle.  The
    # gimbal publishes at 20 Hz, so this is ten missed messages.
    gimbal_max_age_s: float = 0.5
    # How long the gimbal and gyro samples are kept.  Long enough to cover a
    # Deck round trip several times over, short enough to stay small.
    history_s: float = 2.0


class GimbalHistory:
    """The last couple of seconds of `/targeting/gimbal_state`, in radians.

    The relay needs the gimbal angles as they were when the Deck's track was
    taken, not as they are now.  The gimbal publishes at 20 Hz, so the angle at
    an arbitrary instant is an interpolation between two samples.
    """

    def __init__(self, history_s=2.0):
        self._history_s = float(history_s)
        self._samples = deque()

    def add(self, t, pan, tilt):
        t = float(t)
        # A sample that arrives out of order would break the interpolation, so
        # it is dropped rather than sorted in.
        if self._samples and t < self._samples[-1][0]:
            return
        self._samples.append((t, float(pan), float(tilt)))
        self._trim(t)

    def _trim(self, now):
        while self._samples and now - self._samples[0][0] > self._history_s:
            self._samples.popleft()

    def latest(self):
        if not self._samples:
            return None
        return self._samples[-1]

    def at(self, t):
        """The angles at time `t`, or None when nothing has arrived yet.

        Before the first sample and after the last one the nearest sample is
        used.  Holding the end value is the honest answer for the gap between
        the last gimbal message and now, which is at most one 20 Hz period.
        """
        if not self._samples:
            return None
        t = float(t)
        if t <= self._samples[0][0]:
            return self._samples[0][1], self._samples[0][2]
        if t >= self._samples[-1][0]:
            return self._samples[-1][1], self._samples[-1][2]
        for i in range(len(self._samples) - 1):
            t0, pan0, tilt0 = self._samples[i]
            t1, pan1, tilt1 = self._samples[i + 1]
            if t0 <= t <= t1:
                span = t1 - t0
                if span <= 0.0:
                    return pan1, tilt1
                f = (t - t0) / span
                return pan0 + f * (pan1 - pan0), tilt0 + f * (tilt1 - tilt0)
        return self._samples[-1][1], self._samples[-1][2]


class GyroIntegral:
    """Body angular rates, integrated over an arbitrary window.

    Each sample is held until the next one arrives, which is a rectangle rule
    on a signal sampled far faster than the window being asked about.  The
    robot publishes its IMU at the control rate, so the windows here are tens
    of samples long.
    """

    def __init__(self, history_s=2.0):
        self._history_s = float(history_s)
        self._samples = deque()

    def add(self, t, wy, wz):
        t = float(t)
        if self._samples and t < self._samples[-1][0]:
            return
        self._samples.append((t, float(wy), float(wz)))
        while self._samples and t - self._samples[0][0] > self._history_s:
            self._samples.popleft()

    def integrate(self, t0, t1):
        """Returns (dtheta, dpsi): the body's pitch and yaw change over the window.

        `dtheta` is about the body y axis and is positive nose down.  `dpsi` is
        about the body z axis and is positive to the left.  A window that
        starts before the first retained sample is only covered from that
        sample on, which is the same behaviour as having been switched on late.
        """
        t0 = float(t0)
        t1 = float(t1)
        if t1 <= t0 or not self._samples:
            return 0.0, 0.0
        dtheta = 0.0
        dpsi = 0.0
        samples = list(self._samples)
        for i, (t, wy, wz) in enumerate(samples):
            end = samples[i + 1][0] if i + 1 < len(samples) else t1
            a = max(t, t0)
            b = min(end, t1)
            if b > a:
                dtheta += wy * (b - a)
                dpsi += wz * (b - a)
        return dtheta, dpsi


# The fields that say which picture a track describes.  Two tracks that agree
# on all of them are the same detection, resent.
_BOX_FIELDS = ("cx", "cy", "w", "h", "fw", "fh", "label")


def _same_box(a, b):
    return all(a.get(key) == b.get(key) for key in _BOX_FIELDS)


@dataclass(frozen=True)
class Anchor:
    """The track, resolved into angles, with the state it was taken against.

    `t_detect` is when the Deck matched the box, on the robot's clock.  `t0` is
    when the picture behind that match was taken, one round trip earlier, and
    it is where the gyro and gimbal integrals start.
    """

    t0: float
    t_detect: float
    yaw: float
    pitch: float
    pan: float
    tilt: float
    label: str
    age: float
    box_width_px: float
    box_height_px: float


class AimRelay:
    """Keeps the tower controller fed at 40 Hz from a 10 Hz Deck track."""

    def __init__(self, params=None):
        self.params = params or AimParams()
        self.gimbal = GimbalHistory(self.params.history_s)
        self.gyro = GyroIntegral(self.params.history_s)
        self._intrinsics = None
        self._track = None
        self._anchor = None
        self._range_m = None

    # --- inputs ------------------------------------------------------------

    def set_intrinsics(self, intrinsics):
        """The tower camera's own intrinsics, from its `camera_info`."""
        if intrinsics is not None and not intrinsics.valid:
            intrinsics = None
        if intrinsics != self._intrinsics:
            self._intrinsics = intrinsics
            # The anchor was computed with the old numbers, so it is recomputed
            # against the new ones rather than silently kept.
            self._anchor = None

    @property
    def intrinsics(self):
        return self._intrinsics

    def set_range(self, range_m):
        """The follow range, which rides along in the message as `distance_m`."""
        self._range_m = range_m

    def on_gimbal(self, t, pan, tilt):
        self.gimbal.add(t, pan, tilt)

    def on_gyro(self, t, wy, wz):
        self.gyro.add(t, wy, wz)

    def on_track(self, track):
        """A track from the Deck, through the gateway.

        `track` carries cx, cy, w, h, fw, fh, label, age and stamp.  The pixel
        is in the frame the Deck was shown, which is scaled onto the tower
        camera's own frame here.  The integrals restart from the moment this
        track's picture was taken, so nothing the body did before it is
        counted twice.

        A coasting page resends the box it last matched with a growing `age`.
        That is the same detection again, not a new one, so the anchor is kept
        and the integrals keep running from the picture they belong to.
        """
        previous = self._track
        self._track = dict(track)
        if (previous is not None and _same_box(previous, track)
                and float(track.get("age", 0.0)) > float(previous.get("age", 0.0))):
            return
        self._anchor = None

    def clear(self):
        """Drop the lock.  Nothing is published until a new track arrives."""
        self._track = None
        self._anchor = None
        self._range_m = None

    @property
    def has_track(self):
        return self._track is not None

    @property
    def track(self):
        return self._track

    # --- prediction --------------------------------------------------------

    def _anchor_now(self):
        if self._anchor is not None:
            return self._anchor
        if self._track is None or self._intrinsics is None:
            return None
        track = self._track
        frame = scale_intrinsics(
            self._intrinsics, int(track["fw"]), int(track["fh"]))
        yaw, pitch = pixel_to_optical(
            float(track["cx"]), float(track["cy"]), frame)
        age = float(track.get("age", 0.0))
        t_detect = float(track["stamp"]) - age
        t0 = t_detect - self.params.track_latency_s
        angles = self.gimbal.at(t0)
        # No gimbal state yet means the gimbal is assumed to be at zero.  That
        # is exactly the case the plan calls "follow v1 with the tower held
        # still", and it is the right assumption for a bench without servos.
        pan, tilt = angles if angles is not None else (0.0, 0.0)
        self._anchor = Anchor(
            t0=t0,
            t_detect=t_detect,
            yaw=yaw,
            pitch=pitch,
            pan=pan,
            tilt=tilt,
            label=str(track.get("label") or ""),
            age=age,
            box_width_px=float(track.get("w", 0.0)) * self._intrinsics.width / float(track["fw"]),
            box_height_px=float(track.get("h", 0.0)) * self._intrinsics.height / float(track["fh"]),
        )
        return self._anchor

    def track_age(self, now):
        """How old the Deck's detection is, counted at the robot's clock.

        The Deck's own `age` is how long the page had been coasting when it
        sent the message.  The time since the gateway stamped it is added on
        top, so a Deck that goes quiet keeps ageing.  The round trip is not
        counted here: this is the age the page and the coast window mean.
        """
        anchor = self._anchor_now()
        if anchor is None:
            if self._track is None:
                return float("inf")
            return float(self._track.get("age", 0.0)) + max(
                0.0, float(now) - float(self._track["stamp"]))
        return max(0.0, float(now) - anchor.t_detect)

    def gimbal_age(self, now):
        """Seconds since the last gimbal sample, or None before the first."""
        latest = self.gimbal.latest()
        if latest is None:
            return None
        return max(0.0, float(now) - latest[0])

    def gimbal_stale(self, now):
        """Whether the gimbal's angles are too old to be its current pose."""
        age = self.gimbal_age(now)
        return age is not None and age > self.params.gimbal_max_age_s

    def _gimbal_now(self, now, anchor):
        """The gimbal's current angles, or the anchor's when it has gone quiet.

        A targeting controller that faults or restarts stops publishing.  The
        last angle it sent is then a guess about the present, not a
        measurement, and the bearing must not keep treating it as one.
        """
        latest = self.gimbal.latest()
        if latest is None or self.gimbal_stale(now):
            return anchor.pan, anchor.tilt
        return latest[1], latest[2]

    def predict(self, now):
        """The optical angles the target should be at now, and the gimbal state.

        Returns (yaw, pitch, pan, tilt) or None when there is nothing to
        predict from.
        """
        anchor = self._anchor_now()
        if anchor is None:
            return None
        now = float(now)
        dtheta, dpsi = self.gyro.integrate(anchor.t0, now)
        pan, tilt = self._gimbal_now(now, anchor)
        yaw = anchor.yaw + dpsi + self.params.pan_sign * (pan - anchor.pan)
        pitch = anchor.pitch - dtheta + self.params.tilt_sign * (tilt - anchor.tilt)
        return yaw, pitch, pan, tilt

    def bearing(self, now):
        """Where the target is in the body frame, right now.

        The prediction already carries the gimbal's motion since the track, so
        combining it with the current gimbal angles gives a bearing that is
        correct in the middle of a pan.
        """
        predicted = self.predict(now)
        if predicted is None:
            return None
        yaw, pitch, pan, tilt = predicted
        return Bearing(
            azimuth=self.params.pan_sign * pan - yaw,
            elevation=self.params.tilt_sign * tilt - pitch,
        )

    def target(self, now):
        """The LaserTarget the tower controller is fed, as a plain dict.

        The node turns this into a message.  Keeping it a dict is what lets the
        whole relay be tested without ROS.
        """
        predicted = self.predict(now)
        if predicted is None:
            return None
        yaw, pitch, _pan, _tilt = predicted
        anchor = self._anchor
        intrinsics = self._intrinsics
        px, py = optical_to_pixel(yaw, pitch, intrinsics)
        target_x, target_y = pixel_to_normalized(
            px, py, intrinsics.width, intrinsics.height)
        age = self.track_age(now)
        on_sensor = inside_image(px, py, intrinsics)
        return {
            "detected": bool(on_sensor),
            "class_name": anchor.label,
            "target_x": float(target_x),
            "target_y": float(target_y),
            # 0.0 means unknown, which is what the message says a missing
            # range looks like.
            "distance_m": float(self._range_m) if self._range_m else 0.0,
            "confidence": 1.0 if age <= self.params.coast_s else 0.0,
            "stamp": float(now),
        }

    def box_width_px(self):
        """The tracked box's width in the tower camera's own pixels."""
        anchor = self._anchor_now()
        if anchor is None:
            return None
        return anchor.box_width_px
