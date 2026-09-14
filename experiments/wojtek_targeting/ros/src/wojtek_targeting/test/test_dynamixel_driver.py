"""Driver tests. See conftest.py for what is faked and what that leaves unproven."""

import pytest
from wojtek_targeting.dynamixel_driver import (
    ADDR_GOAL_POSITION,
    ADDR_POSITION_P_GAIN,
    ADDR_PROFILE_VELOCITY,
    ADDR_TORQUE_ENABLE,
    DynamixelDriver,
    DynamixelError,
    MotorConfig,
)


def _motors(**overrides):
    pan = MotorConfig(name="pan", motor_id=1, min_deg=-80.0, max_deg=80.0, **overrides)
    tilt = MotorConfig(name="tilt", motor_id=2, direction=-1, min_deg=-35.0, max_deg=45.0)
    return [pan, tilt]


def _driver(bus, motors=None):
    return DynamixelDriver(
        port=bus.port_path, baud_rate=57600, motors=motors or _motors())


# --- angle conversion -----------------------------------------------------

def test_angle_round_trips_through_raw():
    motor = MotorConfig(name="pan", motor_id=1)
    for deg in (-75.0, -10.0, 0.0, 12.5, 79.0):
        assert motor.raw_to_deg(motor.deg_to_raw(deg)) == pytest.approx(deg, abs=0.1)


def test_direction_flips_the_sense_of_an_angle():
    forward = MotorConfig(name="a", motor_id=1, direction=1)
    reversed_ = MotorConfig(name="b", motor_id=2, direction=-1)
    assert forward.deg_to_raw(30.0) > forward.center_raw
    assert reversed_.deg_to_raw(30.0) < reversed_.center_raw


def test_center_raw_moves_the_zero():
    offset = MotorConfig(name="a", motor_id=1, center_raw=1024)
    assert offset.deg_to_raw(0.0) == 1024
    assert offset.raw_to_deg(1024) == 0.0


# --- refusing to run in an unsafe configuration ---------------------------

def test_zero_profile_velocity_is_refused(bus):
    """Zero disables the motion profile, it does not mean 'slow'."""
    driver = _driver(bus, _motors(profile_velocity_deg_s=0.0))
    with pytest.raises(DynamixelError, match="profile velocity"):
        driver.connect()


def test_inverted_limits_are_refused(bus):
    driver = _driver(bus, [MotorConfig(name="pan", motor_id=1, min_deg=10.0, max_deg=-10.0)])
    with pytest.raises(DynamixelError, match="min_deg"):
        driver.connect()


def test_missing_port_is_refused():
    driver = DynamixelDriver(
        port="/dev/definitely-not-here", baud_rate=57600, motors=_motors())
    with pytest.raises(DynamixelError, match="does not exist"):
        driver.connect()


# --- connect and enable ---------------------------------------------------

def test_connect_leaves_torque_off(bus):
    driver = _driver(bus)
    driver.connect()

    assert not driver.torque_enabled
    for servo in bus.servos.values():
        assert servo.registers[ADDR_TORQUE_ENABLE] == 0
    driver.shutdown()


def test_connect_rewrites_the_ram_pid_registers(bus):
    """A power cycle restores the factory 800/0/0, so they are not assumed."""
    motors = _motors()
    motors[0].p_gain = 1200
    bus.servos[1].registers[ADDR_POSITION_P_GAIN] = 800

    driver = _driver(bus, motors)
    driver.connect()

    assert bus.servos[1].registers[ADDR_POSITION_P_GAIN] == 1200
    assert bus.servos[1].registers[ADDR_PROFILE_VELOCITY] > 0
    driver.shutdown()


def test_enable_torque_seeds_the_goal_with_the_present_position(bus):
    """Torque must not come on against a goal left over from last session."""
    bus.servos[1].registers[132] = 2500
    bus.servos[1].registers[ADDR_GOAL_POSITION] = 100   # stale, from a past run

    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()

    assert bus.servos[1].registers[ADDR_GOAL_POSITION] == 2500
    assert bus.servos[1].registers[ADDR_TORQUE_ENABLE] == 1
    driver.shutdown()


def test_enable_torque_refuses_when_position_cannot_be_read(bus):
    """A servo that will not say where it is does not get energised."""
    driver = _driver(bus)
    driver.connect()
    bus.servos[1].responding = False

    with pytest.raises(DynamixelError, match="will not report its position"):
        driver.enable_torque()
    assert not driver.torque_enabled


def test_enable_torque_seeds_outside_limits_without_commanding_a_move(bus):
    """Starting out of range must hold, not snap to the nearest limit."""
    bus.servos[1].registers[132] = 3300   # well past pan_max of 80 deg

    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()

    assert bus.servos[1].registers[ADDR_GOAL_POSITION] == 3300
    driver.shutdown()


# --- commanding -----------------------------------------------------------

def test_write_goals_clamps_to_limits(bus):
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()

    applied = driver.write_goals({1: 250.0, 2: -400.0})

    assert applied[1] == pytest.approx(80.0)
    assert applied[2] == pytest.approx(-35.0)
    driver.shutdown()


def test_write_goals_sends_both_servos_in_one_transaction(bus):
    """Sequential writes would leave tilt a packet behind pan."""
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.sync_writes.clear()

    driver.write_goals({1: 10.0, 2: -5.0})

    assert len(bus.sync_writes) == 1
    assert set(bus.sync_writes[0]) == {1, 2}
    driver.shutdown()


def test_write_goals_before_torque_is_refused(bus):
    driver = _driver(bus)
    driver.connect()
    with pytest.raises(DynamixelError, match="enable_torque"):
        driver.write_goals({1: 0.0})
    driver.shutdown()


def test_write_failure_raises_rather_than_passing_silently(bus):
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.servos[2].responding = False

    with pytest.raises(DynamixelError):
        driver.write_goals({1: 10.0, 2: 10.0})
    driver.shutdown()


# --- reading --------------------------------------------------------------

def test_read_states_converts_position_velocity_and_current(bus):
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.servos[1].registers[132] = 2048 + 1024   # +90 deg
    bus.servos[1].registers[128] = 10            # 10 * 0.229 rev/min

    states = driver.read_states()

    assert states[1].position_deg == pytest.approx(90.0, abs=0.2)
    assert states[1].velocity_rad_s == pytest.approx(0.2398, abs=1e-3)
    assert not states[1].stale
    driver.shutdown()


def test_negative_current_is_read_as_signed(bus):
    """Unsigned, a small negative current comes back as ~65500."""
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.servos[1].registers[126] = -50 & 0xFFFF

    states = driver.read_states()

    assert states[1].current_a == pytest.approx(-50 * 0.00269)
    driver.shutdown()


def test_negative_position_is_read_as_signed(bus):
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.servos[1].registers[132] = -512 & 0xFFFFFFFF

    states = driver.read_states()

    assert states[1].position_deg == pytest.approx((-512 - 2048) * 360.0 / 4096.0, abs=0.1)
    driver.shutdown()


def test_failed_read_keeps_the_last_value_and_marks_it_stale(bus):
    """The bug this guards: a dropped packet reading as a real zero."""
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()
    bus.servos[1].registers[132] = 2048 + 512
    good = driver.read_states()[1].position_deg
    assert good == pytest.approx(45.0, abs=0.2)

    bus.servos[1].responding = False
    states = driver.read_states()

    assert states[1].position_deg == pytest.approx(good)
    assert states[1].stale
    driver.shutdown()


def test_temperature_is_polled_round_robin(bus):
    """One servo per cycle -- it is the one reading outside the block read."""
    bus.servos[1].registers[146] = 41
    bus.servos[2].registers[146] = 52

    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()

    first = driver.read_states()
    assert first[1].temperature_c == 41.0
    assert first[2].temperature_c == 0.0

    second = driver.read_states()
    assert second[2].temperature_c == 52.0
    driver.shutdown()


# --- shutdown -------------------------------------------------------------

def test_shutdown_drops_torque_and_releases_the_port(bus):
    driver = _driver(bus)
    driver.connect()
    driver.enable_torque()

    driver.shutdown()

    assert bus.servos[1].registers[ADDR_TORQUE_ENABLE] == 0
    assert not bus.port_open
    driver.shutdown()   # must be safe twice


def test_second_driver_cannot_take_the_same_port(bus):
    """Two processes on one Dynamixel bus interleave packets and both lose."""
    first = _driver(bus)
    first.connect()

    second = _driver(bus)
    with pytest.raises(DynamixelError, match="already open"):
        second.connect()

    first.shutdown()
