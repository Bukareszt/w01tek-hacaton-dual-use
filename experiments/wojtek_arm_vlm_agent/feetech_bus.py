"""Minimal, torch-free Feetech STS3215 bus for the SO-101, compatible with LeRobot's calibration.

Depends only on `feetech-servo-sdk` (scservo_sdk) and pyserial. It reproduces the subset of
LeRobot's FeetechMotorsBus that the arm controller uses, with the same unit conventions:

- positions in DEGREES: ``deg = (raw - mid) * 360 / 4095`` with ``mid = (range_min + range_max) / 2``
  from the calibration file (LeRobot ``MotorNormMode.DEGREES``); the servo firmware already
  applies the homing offset that LeRobot wrote during calibration
- ``is_calibrated`` compares the servos' Homing_Offset / Min / Max_Position_Limit registers with
  the calibration file, like LeRobot does, so a mismatched arm is refused
- register names, addresses and sign-magnitude encodings are those of LeRobot's STS3215 table

Calibration file: LeRobot's ``~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json``
(``{motor: {id, drive_mode, homing_offset, range_min, range_max}}``).
"""

import json
import os
from types import SimpleNamespace

import scservo_sdk as scs

BAUDRATE = 1_000_000
PROTOCOL = 0
RESOLUTION = 4096
MODEL_NUMBER = 777           # STS3215

REGISTERS = {                 # name: (address, length)
    "Model_Number": (3, 2),
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Homing_Offset": (31, 2),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Maximum_Acceleration": (85, 1),
}
SIGN_BIT = {"Homing_Offset": 11, "Goal_Position": 15, "Present_Position": 15}
DEGREE_REGISTERS = {"Goal_Position", "Present_Position"}
SO101_MOTORS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}

DEFAULT_CALIBRATION = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json")


def _encode_sign(value, bit):
    return (abs(value) | (1 << bit)) if value < 0 else value


def _decode_sign(value, bit):
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


class FeetechBus:
    def __init__(self, port, calibration_path=None, motors=None):
        self.port = port
        self.motors = motors or dict(SO101_MOTORS)
        self.ids = {v: k for k, v in self.motors.items()}
        path = calibration_path or os.environ.get("WOJTEK_ARM_CALIBRATION", DEFAULT_CALIBRATION)
        if not os.path.exists(path):
            raise FileNotFoundError(f"calibration file not found: {path} (run LeRobot calibration first)")
        raw = json.load(open(path))
        self.calibration = {m: SimpleNamespace(**raw[m]) for m in self.motors}
        self.port_handler = None
        self.packet_handler = None

    # ---------- connection ----------
    def connect(self):
        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(PROTOCOL)
        if not self.port_handler.openPort():
            raise ConnectionError(f"cannot open {self.port}")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise ConnectionError(f"cannot set baudrate on {self.port}")
        missing = []
        for name, id_ in self.motors.items():
            model, comm, err = self.packet_handler.ping(self.port_handler, id_)
            if comm != scs.COMM_SUCCESS:
                missing.append(f"{name}(id {id_})")
        if missing:
            self.port_handler.closePort()
            raise ConnectionError(f"motors not answering on {self.port}: {', '.join(missing)}")

    def disconnect(self, disable_torque=False):
        if disable_torque:
            self.disable_torque()
        if self.port_handler is not None:
            self.port_handler.closePort()
            self.port_handler = None

    # ---------- raw register access ----------
    def read(self, reg, motor, normalize=True):
        addr, length = REGISTERS[reg]
        id_ = self.motors[motor]
        if length == 1:
            val, comm, err = self.packet_handler.read1ByteTxRx(self.port_handler, id_, addr)
        else:
            val, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, id_, addr)
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"read {reg} from {motor} failed: {self.packet_handler.getTxRxResult(comm)}")
        if reg in SIGN_BIT:
            val = _decode_sign(val, SIGN_BIT[reg])
        return self._to_deg(motor, val) if (normalize and reg in DEGREE_REGISTERS) else val

    def write(self, reg, motor, value, normalize=True):
        addr, length = REGISTERS[reg]
        id_ = self.motors[motor]
        if normalize and reg in DEGREE_REGISTERS:
            value = self._from_deg(motor, value)
        value = int(round(value))
        if reg in SIGN_BIT:
            value = _encode_sign(value, SIGN_BIT[reg])
        if length == 1:
            comm, err = self.packet_handler.write1ByteTxRx(self.port_handler, id_, addr, value)
        else:
            comm, err = self.packet_handler.write2ByteTxRx(self.port_handler, id_, addr, value)
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"write {reg} to {motor} failed: {self.packet_handler.getTxRxResult(comm)}")

    def sync_read(self, reg, motors=None, normalize=True):
        addr, length = REGISTERS[reg]
        names = list(motors or self.motors)
        group = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, length)
        for n in names:
            group.addParam(self.motors[n])
        comm = group.txRxPacket()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"sync read {reg} failed: {self.packet_handler.getTxRxResult(comm)}")
        out = {}
        for n in names:
            id_ = self.motors[n]
            if not group.isAvailable(id_, addr, length):
                raise ConnectionError(f"sync read {reg}: no data from {n}")
            val = group.getData(id_, addr, length)
            if reg in SIGN_BIT:
                val = _decode_sign(val, SIGN_BIT[reg])
            out[n] = self._to_deg(n, val) if (normalize and reg in DEGREE_REGISTERS) else val
        return out

    def sync_write(self, reg, values, normalize=True):
        addr, length = REGISTERS[reg]
        group = scs.GroupSyncWrite(self.port_handler, self.packet_handler, addr, length)
        for n, v in values.items():
            if normalize and reg in DEGREE_REGISTERS:
                v = self._from_deg(n, v)
            v = int(round(v))
            if reg in SIGN_BIT:
                v = _encode_sign(v, SIGN_BIT[reg])
            data = [v & 0xFF] if length == 1 else [v & 0xFF, (v >> 8) & 0xFF]
            group.addParam(self.motors[n], data)
        comm = group.txPacket()
        group.clearParam()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"sync write {reg} failed: {self.packet_handler.getTxRxResult(comm)}")

    # ---------- LeRobot-compatible helpers ----------
    @property
    def is_calibrated(self):
        for n in self.motors:
            cal = self.calibration[n]
            if (self.read("Min_Position_Limit", n) != cal.range_min or self.read("Max_Position_Limit", n) != cal.range_max
                    or self.read("Homing_Offset", n) != cal.homing_offset):
                return False
        return True

    def enable_torque(self, motors=None):
        for n in (motors or self.motors):
            self.write("Torque_Enable", n, 1)
            self.write("Lock", n, 1)

    def disable_torque(self, motors=None):
        for n in (motors or self.motors):
            self.write("Torque_Enable", n, 0)
            self.write("Lock", n, 0)

    def _to_deg(self, motor, raw):
        cal = self.calibration[motor]
        mid = (cal.range_min + cal.range_max) / 2
        return (raw - mid) * 360 / (RESOLUTION - 1)

    def _from_deg(self, motor, deg):
        cal = self.calibration[motor]
        mid = (cal.range_min + cal.range_max) / 2
        return deg * (RESOLUTION - 1) / 360 + mid
