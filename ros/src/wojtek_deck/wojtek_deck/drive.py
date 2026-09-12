"""The drive gate: pad frames in, /cmd_vel values out, dead-man in between.

Pure Python on purpose (no ROS, no asyncio) so the safety logic has a
model-free unit test. The gateway node feeds it wall-clock seconds and
publishes whatever `tick` returns.

The rules are the ones gamepad_teleop already lives by, because policy_node
latches the last /cmd_vel it received and never times it out:

  * Nothing is published until a source has spoken once. A resident gateway
    with no client must not spam zeros over another /cmd_vel source.
  * A frame older than `timeout_s` means the link is gone (tab closed, wifi
    drop, handheld asleep). The gate then publishes ZEROS -- the burst that
    overwrites the latched command -- for `silence_after_s`, and only then
    goes silent so another source can take over without being fought.
  * An explicit stop (the page said so, or the last client disconnected)
    is the same burst-then-silence, started right away.
  * The standing height is a held set-point: it survives stops, so the
    stance does not jump when the sticks are released.

Two sources reach the gate. The pad is the operator's sticks, normalized on
the page and scaled into the trained command box here. The follow source is
the follow node's /wojtek/follow/cmd_vel, already in metres per second and
radians per second, accepted only while the page holds a lock.

A moved stick always wins. One moved stick or one stop ends the follow
there and then, and the pad drives. A resting pad is different: a connected
pad streams all-zero frames twenty times a second, lock or no lock, so a
zero frame refreshes the pad's dead-man but does not claim the gate while a
lock is on. Follow frames drive while the lock is on, the follow node is
fresh, and the last pad frame was at rest or older than `timeout_s`. A
follow frame never touches the height.
"""

import math

IDLE = "idle"        # nothing to publish
LIVE = "live"        # fresh pad frames, sticks drive
DEADMAN = "deadman"  # frames stopped: zeroing burst
FOLLOW = "follow"    # the follow node drives, the pad is quiet


class DriveGate:
    def __init__(self, cmd_low, cmd_high, height_range, height_default,
                 timeout_s=0.5, silence_after_s=2.0):
        self.cmd_low = [float(v) for v in cmd_low]
        self.cmd_high = [float(v) for v in cmd_high]
        self.height_range = (float(height_range[0]), float(height_range[1]))
        self.height = float(height_default)
        self.timeout_s = float(timeout_s)
        self.silence_after_s = float(silence_after_s)
        self._cmd = (0.0, 0.0, 0.0)     # normalized [-1, 1] (vx, vy, yaw)
        self._stamp = None              # seconds, of the last pad frame
        self._follow = (0.0, 0.0, 0.0)  # m/s, m/s, rad/s
        self._follow_stamp = None       # seconds, of the last follow frame
        # Set when a lock starts on the page, cleared when it ends. While it
        # is false every follow frame is dropped at the door.
        self.follow_active = False
        self.state = IDLE

    # -- input ---------------------------------------------------------------
    def command(self, now, vx, vy, yaw, height=None):
        """A pad frame: normalized sticks, optional absolute height (m)."""
        clip = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
        self._cmd = (clip(vx), clip(vy), clip(yaw))
        # A moved stick is the operator taking the robot back. A resting pad
        # streams zeros every tick and must not end anything.
        if any(v != 0.0 for v in self._cmd):
            self.end_follow(now)
        if height is not None:
            lo, hi = self.height_range
            self.height = max(lo, min(hi, float(height)))
        self._stamp = float(now)

    def stop(self, now):
        """Explicit stop: start the zeroing burst now, keep the height."""
        self.end_follow(now)
        if self._stamp is None:
            return  # never drove: stay silent, there is nothing to undo
        self._cmd = (0.0, 0.0, 0.0)
        # Backdate the stamp so the very next tick sees a stale frame.
        self._stamp = min(self._stamp, float(now) - self.timeout_s)

    def start_follow(self):
        """A lock began on the page: follow frames may drive from now on."""
        self.follow_active = True

    def end_follow(self, now):
        """The lock ended: no more follow frames, and stop what they left.

        The last follow frame is backdated the way `stop` backdates a pad
        frame, so the next tick starts the zeroing burst instead of leaving
        policy_node latched on the speed the follow node last asked for.
        """
        self.follow_active = False
        if self._follow_stamp is not None:
            self._follow_stamp = min(self._follow_stamp,
                                     float(now) - self.timeout_s)

    def follow(self, now, vx, vy, yaw):
        """One /wojtek/follow/cmd_vel frame, in m/s and rad/s.

        Dropped unless a lock is on. The values are clipped to the same
        command box the sticks are scaled into, so a follow node with a bad
        gain cannot ask for more than the policy was trained on.
        """
        if not self.follow_active:
            return
        # Python's max/min pass NaN through as the box's upper limit, and a
        # Twist float64 can carry one. A frame with a NaN or an infinity is
        # dropped whole; the dead-man then zeroes as for a silent node.
        if not all(math.isfinite(float(v)) for v in (vx, vy, yaw)):
            return
        self._follow = tuple(
            max(self.cmd_low[i], min(self.cmd_high[i], float(v)))
            for i, v in enumerate((vx, vy, yaw)))
        self._follow_stamp = float(now)

    # -- output --------------------------------------------------------------
    def tick(self, now):
        """(vx, vy, yaw, height) to publish this tick, or None for silence.

        Updates `state` as a side effect; the node reports its changes.
        """
        now = float(now)
        pad_age = None if self._stamp is None else now - self._stamp
        follow_age = (None if self._follow_stamp is None
                      else now - self._follow_stamp)
        pad_fresh = pad_age is not None and pad_age < self.timeout_s
        follow_fresh = (self.follow_active and follow_age is not None
                        and follow_age < self.timeout_s)
        # A fresh pad frame owns the gate, with one exception: while a lock
        # is on, a resting pad (all zeros) is only the link's heartbeat, and
        # a fresh follow frame drives through it. A moved stick has already
        # ended the follow in `command`, so it never reaches the exception.
        pad_resting = not any(v != 0.0 for v in self._cmd)
        if pad_fresh and not (follow_fresh and pad_resting):
            self.state = LIVE
            return (*self._scaled(), self.height)
        if follow_fresh:
            self.state = FOLLOW
            return (*self._follow, self.height)
        # Neither source is fresh. Whichever spoke last owns the burst: zeros
        # for silence_after_s, which overwrites the latched command, then
        # silence, so another /cmd_vel source can take over unfought.
        ages = [a for a in (pad_age, follow_age) if a is not None]
        if not ages:
            self.state = IDLE
            return None
        age = min(ages)
        if age > self.timeout_s + self.silence_after_s:
            self.state = IDLE
            return None
        self.state = DEADMAN
        return (0.0, 0.0, 0.0, self.height)

    def _scaled(self):
        """The pad's normalized sticks in the trained (asymmetric) box:
        positive stick scales by high, negative by low."""
        return tuple(v * self.cmd_high[i] if v >= 0 else v * -self.cmd_low[i]
                     for i, v in enumerate(self._cmd))

    def step_height(self, delta_m):
        lo, hi = self.height_range
        self.height = max(lo, min(hi, self.height + float(delta_m)))
        return self.height
