"""DynamixelSDK driver for the targeting gimbal's two servos.

No ROS in here on purpose. The bus is the part of this subsystem most likely
to be wrong and least pleasant to debug through a node, so it stays a plain
importable module that can be driven from a REPL on the bench and tested
without a robot.

Both servos (XM540 pan, XM430 tilt) sit on one Protocol 2.0 TTL bus behind a
single U2D2, addressed by ID. `TargetingController` is the only thing that
should ever construct this -- two processes on one Dynamixel bus produce
garbage, which is why `connect()` takes an exclusive lock on the port.

Five things here were paid for in hardware on the Bathymetry manipulator,
which ran this same servo family on this same kind of bus. They are not
defensive programming, they are bug fixes:

1. A failed read must never be indistinguishable from a successful zero.
   Returning 0 on a dropped packet put a position of "encoder zero" into the
   state of ~1.6% of samples there -- a jump of the entire mounting offset.
   On a gimbal that is not a bad sample, it is a whip to the end stop. Every
   read here reports success separately and the caller keeps the last good
   value.
2. PRESENT_CURRENT is a signed int16. Read as unsigned, a small negative
   current comes back as ~65500 and every load reading is nonsense.
3. The position PID registers are RAM, so a power cycle silently restores the
   factory 800/0/0. They are rewritten on every connect rather than assumed
   to have survived.
4. Profile velocity 0 does not mean "do not move", it means "no profile":
   the servo slews to the goal at maximum speed. Leaving it at the default is
   how a gimbal shakes the robot it is bolted to, so a non-zero profile is
   required here rather than merely offered.
5. Torque must not be enabled until GOAL_POSITION holds the position the
   servo is already at. GOAL_POSITION is RAM that survives a torque-off, so
   enabling torque first makes the gimbal snap to wherever the last session
   left it.
"""

import fcntl
import os
from dataclasses import dataclass

import dynamixel_sdk as dxl

PROTOCOL_VERSION = 2.0

# Control table, XM430/XM540 (identical for everything used here).
ADDR_OPERATING_MODE = 11
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

MODE_POSITION = 3

# PRESENT_CURRENT(2) + PRESENT_VELOCITY(4) + PRESENT_POSITION(4) are
# contiguous at 126..135, so one 10-byte read gets all three. At 57600 baud a
# round trip costs roughly 5 ms; three separate reads per servo would spend
# 30 ms of a 25 ms control period on state alone.
STATE_BLOCK_ADDR = ADDR_PRESENT_CURRENT
STATE_BLOCK_LEN = 10

ENCODER_RESOLUTION = 4096

# Servo-native units for the XM series, from the ROBOTIS control table.
VELOCITY_UNIT_RPM = 0.229
CURRENT_UNIT_A = 0.00269


class DynamixelError(Exception):
    """A bus transaction failed, or the hardware is in no state to be used."""


@dataclass
class MotorConfig:
    """One servo on the bus.

    `center_raw` is the encoder count at this joint's zero. It defaults to the
    middle of the range, but a gimbal's mechanical zero essentially never
    lands there once it is assembled, so it is per-motor rather than a shared
    constant. `direction` is +1 or -1 and absorbs which way round the servo
    ended up being bolted on.
    """

    name: str
    motor_id: int
    center_raw: int = 2048
    direction: int = 1
    min_deg: float = -90.0
    max_deg: float = 90.0
    # Degrees/s and degrees/s^2 as the servo understands them, converted to
    # register units on write. Zero is rejected -- see the module docstring.
    profile_velocity_deg_s: float = 45.0
    profile_acceleration_deg_s2: float = 180.0
    p_gain: int = 800
    i_gain: int = 0
    d_gain: int = 0

    def deg_to_raw(self, deg: float) -> int:
        counts = deg * ENCODER_RESOLUTION / 360.0
        return int(round(self.center_raw + self.direction * counts))

    def raw_to_deg(self, raw: int) -> float:
        return self.direction * (raw - self.center_raw) * 360.0 / ENCODER_RESOLUTION

    def clamp(self, deg: float) -> float:
        return max(self.min_deg, min(self.max_deg, deg))


@dataclass
class MotorState:
    """Last known state of one servo.

    `stale` marks a sample the bus failed to refresh: the values are the last
    ones that did arrive, not fresh readings. Callers may keep using them --
    a slightly old angle is far better than a zero pretending to be one -- but
    must not treat a long run of stale samples as the gimbal holding still.
    """

    position_deg: float = 0.0
    velocity_rad_s: float = 0.0
    current_a: float = 0.0
    temperature_c: float = 0.0
    stale: bool = True


def _to_signed(value: int, bits: int) -> int:
    limit = 1 << bits
    return value - limit if value >= limit >> 1 else value


class DynamixelDriver:
    """Owns the U2D2 port and both gimbal servos.

    Lifecycle: `connect()`, `enable_torque()`, then `write_goals()` /
    `read_states()` per tick, and `shutdown()` at the end. `shutdown()` is safe
    to call from anywhere, including a half-finished `connect()`.
    """

    def __init__(self, port, baud_rate, motors, logger=None):
        self._port_name = port
        self._baud_rate = baud_rate
        self._motors = {m.motor_id: m for m in motors}
        self._log = logger
        self._port = None
        self._packet = None
        self._lock_fd = None
        self._torque_on = False
        self._states = {m.motor_id: MotorState() for m in motors}
        # Temperature changes on a thermal timescale, so it is polled one servo
        # per cycle instead of every servo every cycle. It is the only reading
        # that does not fit the contiguous block above.
        self._temperature_turn = 0

    @property
    def motors(self):
        return self._motors

    @property
    def last_states(self):
        """Last known state without touching the bus.

        For status publishing, which runs every control tick and must not cost
        a transaction to do it.
        """
        return dict(self._states)

    @property
    def torque_enabled(self) -> bool:
        return self._torque_on

    def _info(self, message):
        if self._log is not None:
            self._log.info(message)

    def _warn(self, message):
        if self._log is not None:
            self._log.warn(message)

    def connect(self):
        """Open the port, take the lock, and put both servos in a known state.

        Leaves torque OFF. Nothing moves until `enable_torque()`.
        """
        for motor in self._motors.values():
            if motor.profile_velocity_deg_s <= 0.0 or motor.profile_acceleration_deg_s2 <= 0.0:
                raise DynamixelError(
                    f"{motor.name}: profile velocity/acceleration must be > 0. Zero is not "
                    "'hold still' to a Dynamixel, it disables the motion profile and the "
                    "servo slews to every goal at maximum speed.")
            if motor.direction not in (1, -1):
                raise DynamixelError(f"{motor.name}: direction must be +1 or -1")
            if motor.min_deg >= motor.max_deg:
                raise DynamixelError(f"{motor.name}: min_deg must be below max_deg")

        if not os.path.exists(self._port_name):
            raise DynamixelError(
                f"{self._port_name} does not exist. Is the U2D2 plugged in? "
                "`ls /dev/ttyUSB*` and check the udev rule shipped with this package.")

        # Exclusive lock before the SDK touches the port. Two processes writing
        # one Dynamixel bus interleave their packets and both see corruption;
        # failing loudly here beats debugging that.
        try:
            self._lock_fd = open(self._port_name, "rb+", buffering=0)
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._release_lock()
            raise DynamixelError(
                f"{self._port_name} is already open in another process -- most likely a "
                "targeting_controller that did not exit. Check with "
                "`pgrep -af targeting_controller`.") from exc
        except OSError as exc:
            self._release_lock()
            raise DynamixelError(f"cannot open {self._port_name}: {exc}") from exc

        self._port = dxl.PortHandler(self._port_name)
        self._packet = dxl.PacketHandler(PROTOCOL_VERSION)
        if not self._port.openPort():
            raise DynamixelError(f"cannot open {self._port_name} through the Dynamixel SDK")
        if not self._port.setBaudRate(self._baud_rate):
            raise DynamixelError(
                f"cannot set {self._baud_rate} baud on {self._port_name}. The servos only "
                "answer at the rate they were configured for; check it with the Dynamixel "
                "Wizard rather than guessing.")

        for motor in self._motors.values():
            self._configure_motor(motor)

        self._info(
            f"{self._port_name} open at {self._baud_rate} baud; "
            f"servos {sorted(self._motors)} configured, torque off")

    def _configure_motor(self, motor):
        # OPERATING_MODE is EEPROM and refuses to change under torque, so torque
        # goes off first even though it should already be off.
        self._write1(motor, ADDR_TORQUE_ENABLE, 0)
        self._write1(motor, ADDR_OPERATING_MODE, MODE_POSITION)
        self._write4(motor, ADDR_PROFILE_VELOCITY, self._profile_velocity_raw(motor))
        self._write4(motor, ADDR_PROFILE_ACCELERATION, self._profile_acceleration_raw(motor))
        # RAM registers: a power cycle restores the factory 800/0/0, so these
        # are written every time rather than assumed to have survived.
        self._write2(motor, ADDR_POSITION_P_GAIN, motor.p_gain)
        self._write2(motor, ADDR_POSITION_I_GAIN, motor.i_gain)
        self._write2(motor, ADDR_POSITION_D_GAIN, motor.d_gain)

    @staticmethod
    def _profile_velocity_raw(motor) -> int:
        # Register unit is 0.229 rev/min.
        rpm = motor.profile_velocity_deg_s * 60.0 / 360.0
        return max(1, int(round(rpm / VELOCITY_UNIT_RPM)))

    @staticmethod
    def _profile_acceleration_raw(motor) -> int:
        # Register unit is 214.577 rev/min^2.
        rpm2 = motor.profile_acceleration_deg_s2 * 3600.0 / 360.0
        return max(1, int(round(rpm2 / 214.577)))

    def enable_torque(self):
        """Seed each servo's goal with where it already is, then enable torque.

        The read is retried and a failure aborts: GOAL_POSITION holding a stale
        value from the previous session, or a zero standing in for a dropped
        packet, both mean the gimbal lunges the instant torque comes on.
        """
        for motor in self._motors.values():
            raw = None
            for _ in range(5):
                value, ok = self._read4(motor, ADDR_PRESENT_POSITION)
                if ok:
                    raw = value
                    break
            if raw is None:
                raise DynamixelError(
                    f"{motor.name} (ID {motor.motor_id}) will not report its position, so "
                    "there is no safe goal to enable torque against. Refusing to energise "
                    "the gimbal.")

            present_deg = motor.raw_to_deg(raw)
            # Seed the goal with the *unclamped* present position. Clamping here
            # would command a move to the limit as the very first action after
            # torque-on, which is exactly the lunge this is meant to prevent.
            self._write4(motor, ADDR_GOAL_POSITION, raw)
            self._states[motor.motor_id] = MotorState(position_deg=present_deg, stale=False)
            self._write1(motor, ADDR_TORQUE_ENABLE, 1)

            if not motor.min_deg <= present_deg <= motor.max_deg:
                self._warn(
                    f"{motor.name} started at {present_deg:.1f} deg, outside its "
                    f"[{motor.min_deg:.1f}, {motor.max_deg:.1f}] limits. It is holding there; "
                    "the first commanded move will bring it back inside.")

        self._torque_on = True
        self._info("torque enabled; each servo holding the position it was already at")

    def disable_torque(self):
        if self._packet is None:
            return
        for motor in self._motors.values():
            self._write1(motor, ADDR_TORQUE_ENABLE, 0)
        self._torque_on = False

    def write_goals(self, goals_deg):
        """Command both servos in one bus transaction.

        A sync write is not just cheaper than two writes, it is the only way
        pan and tilt start moving on the same packet. Two sequential writes
        leave tilt a transaction behind pan, which on a diagonal track shows up
        as the aim point bowing away from the straight line to the target.

        Angles are clamped to each motor's limits here regardless of what the
        caller already checked. This is the last thing between a number and a
        servo, so it is where the limit has to be real.
        """
        if not self._torque_on:
            raise DynamixelError("write_goals() before enable_torque()")

        writer = dxl.GroupSyncWrite(self._port, self._packet, ADDR_GOAL_POSITION, 4)
        applied = {}
        for motor_id, deg in goals_deg.items():
            motor = self._motors[motor_id]
            clamped = motor.clamp(deg)
            raw = motor.deg_to_raw(clamped)
            if not writer.addParam(motor_id, list(raw.to_bytes(4, "little", signed=True))):
                writer.clearParam()
                raise DynamixelError(f"could not queue a goal for {motor.name}")
            applied[motor_id] = clamped

        result = writer.txPacket()
        writer.clearParam()
        if result != dxl.COMM_SUCCESS:
            raise DynamixelError(
                f"goal write failed: {self._packet.getTxRxResult(result)}")
        return applied

    def read_states(self):
        """Refresh and return every servo's state.

        One sync read covers current, velocity and position for both servos.
        Temperature is polled for one servo per call. A servo that does not
        answer keeps its previous values and is marked `stale`.
        """
        reader = dxl.GroupSyncRead(
            self._port, self._packet, STATE_BLOCK_ADDR, STATE_BLOCK_LEN)
        for motor_id in self._motors:
            reader.addParam(motor_id)

        result = reader.txRxPacket()
        bus_ok = result == dxl.COMM_SUCCESS

        for motor_id, motor in self._motors.items():
            state = self._states[motor_id]
            if not bus_ok or not reader.isAvailable(
                    motor_id, STATE_BLOCK_ADDR, STATE_BLOCK_LEN):
                state.stale = True
                continue

            current_raw = reader.getData(motor_id, ADDR_PRESENT_CURRENT, 2)
            velocity_raw = reader.getData(motor_id, ADDR_PRESENT_VELOCITY, 4)
            position_raw = reader.getData(motor_id, ADDR_PRESENT_POSITION, 4)

            # Both come out of the SDK unsigned; both are signed in the control
            # table. Skipping this turns a small negative current into ~65500
            # and a negative angle into a position near the far end of the
            # encoder.
            state.current_a = _to_signed(current_raw, 16) * CURRENT_UNIT_A
            state.velocity_rad_s = (
                _to_signed(velocity_raw, 32) * VELOCITY_UNIT_RPM * 2.0 * 3.141592653589793 / 60.0)
            state.position_deg = motor.raw_to_deg(_to_signed(position_raw, 32))
            state.stale = False

        reader.clearParam()
        self._poll_one_temperature()
        return dict(self._states)

    def _poll_one_temperature(self):
        motor_ids = sorted(self._motors)
        if not motor_ids:
            return
        motor_id = motor_ids[self._temperature_turn % len(motor_ids)]
        self._temperature_turn += 1
        value, ok = self._read1(self._motors[motor_id], ADDR_PRESENT_TEMPERATURE)
        if ok:
            self._states[motor_id].temperature_c = float(value)

    def shutdown(self):
        """Drop torque, close the port, release the lock. Safe to call twice."""
        try:
            self.disable_torque()
        except DynamixelError:
            # Already failing on the bus; the port still has to be released.
            pass
        if self._port is not None:
            self._port.closePort()
            self._port = None
        self._packet = None
        self._release_lock()

    def _release_lock(self):
        if self._lock_fd is None:
            return
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            self._lock_fd.close()
        finally:
            self._lock_fd = None

    # --- register access ---------------------------------------------------
    #
    # Writes raise, reads report success. A write that silently failed leaves
    # the servo doing something other than what was asked, which there is no
    # safe way to paper over; a read that failed has a sane fallback in the
    # caller's previous value.

    def _check_write(self, motor, result, error):
        if result != dxl.COMM_SUCCESS:
            raise DynamixelError(
                f"{motor.name} (ID {motor.motor_id}): "
                f"{self._packet.getTxRxResult(result)}")
        if error != 0:
            self._warn(
                f"{motor.name} (ID {motor.motor_id}) rejected a write: "
                f"{self._packet.getRxPacketError(error)}")

    def _write1(self, motor, addr, value):
        result, error = self._packet.write1ByteTxRx(
            self._port, motor.motor_id, addr, int(value))
        self._check_write(motor, result, error)

    def _write2(self, motor, addr, value):
        result, error = self._packet.write2ByteTxRx(
            self._port, motor.motor_id, addr, int(value))
        self._check_write(motor, result, error)

    def _write4(self, motor, addr, value):
        result, error = self._packet.write4ByteTxRx(
            self._port, motor.motor_id, addr, int(value) & 0xFFFFFFFF)
        self._check_write(motor, result, error)

    def _read1(self, motor, addr):
        value, result, _ = self._packet.read1ByteTxRx(self._port, motor.motor_id, addr)
        return value, result == dxl.COMM_SUCCESS

    def _read4(self, motor, addr):
        value, result, _ = self._packet.read4ByteTxRx(self._port, motor.motor_id, addr)
        return _to_signed(value, 32), result == dxl.COMM_SUCCESS
