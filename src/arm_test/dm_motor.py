"""Damiao DM4310 / DM4340 CAN protocol (MIT mode) — standalone implementation.

Byte-for-byte matched to how i2rt drives the real YAM Pro, per its
`i2rt/motor_drivers/dm_driver.py` and `utils.py`. Two details are easy to get
wrong and are called out below:

  * Feedback replies arrive on arbitration ID  = motor_id + 16 (0x10 + id),
    NOT the same id and NOT id+0.  (i2rt ReceiveMode.p16)
  * DM4340 VELOCITY_MAX = 10 (i2rt uses the 48 V constant), while Damiao stock
    24 V lists 8. We match i2rt so decoded velocities agree with the real arm.

This module is pure/offline — no CAN I/O — so it is unit-testable without hardware.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import Enum


# ---------------------------------------------------------------------------
# Per-motor scaling constants  (symmetric: MIN = -MAX unless noted)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MotorConstants:
    p_max: float          # rad
    v_max: float          # rad/s
    t_max: float          # N·m
    kp_max: float = 500.0
    kp_min: float = 0.0
    kd_max: float = 5.0
    kd_min: float = 0.0

    @property
    def p_min(self) -> float:
        return -self.p_max

    @property
    def v_min(self) -> float:
        return -self.v_max

    @property
    def t_min(self) -> float:
        return -self.t_max


class MotorType(str, Enum):
    DM4310 = "DM4310"
    DM4340 = "DM4340"

    @property
    def constants(self) -> MotorConstants:
        return _MOTOR_CONSTANTS[self]


_MOTOR_CONSTANTS: dict[MotorType, MotorConstants] = {
    # DM4310 matches Damiao stock exactly: P ±12.5, V ±30, T ±10
    MotorType.DM4310: MotorConstants(p_max=12.5, v_max=30.0, t_max=10.0),
    # DM4340: i2rt hard-codes V_MAX = 10 (48 V value), T ±28
    MotorType.DM4340: MotorConstants(p_max=12.5, v_max=10.0, t_max=28.0),
}


# ---------------------------------------------------------------------------
# Fixed-point <-> float  (denominator is 2^bits - 1, matching i2rt/Damiao)
# ---------------------------------------------------------------------------
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    x = min(max(x, x_min), x_max)  # clamp first (i2rt does)
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min


# ---------------------------------------------------------------------------
# CAN ID scheme (MIT mode)
# ---------------------------------------------------------------------------
CMD_ID_OFFSET_MIT = 0x000   # ControlMode.MIT
RECEIVE_ID_OFFSET = 16      # ReceiveMode.p16:  rx = motor_id + 16


def tx_arbitration_id(motor_id: int) -> int:
    """Arbitration ID to send an MIT command to a motor."""
    return CMD_ID_OFFSET_MIT + motor_id


def rx_arbitration_id(motor_id: int) -> int:
    """Arbitration ID a motor's feedback frame arrives on."""
    return motor_id + RECEIVE_ID_OFFSET


def motor_id_from_rx(arbitration_id: int) -> int:
    return arbitration_id - RECEIVE_ID_OFFSET


# ---------------------------------------------------------------------------
# Special command payloads  (7x 0xFF + terminator)
# ---------------------------------------------------------------------------
CMD_ENABLE = bytes([0xFF] * 7 + [0xFC])       # motor on
CMD_DISABLE = bytes([0xFF] * 7 + [0xFD])      # motor off
CMD_SAVE_ZERO = bytes([0xFF] * 7 + [0xFE])    # save current pos as zero
CMD_CLEAN_ERROR = bytes([0xFF] * 7 + [0xFB])  # clear fault state


# ---------------------------------------------------------------------------
# MIT control frame
# ---------------------------------------------------------------------------
def pack_mit_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    motor_type: MotorType,
) -> bytes:
    """Build the 8-byte MIT control payload for a DM motor."""
    c = motor_type.constants
    p = float_to_uint(position, c.p_min, c.p_max, 16)
    v = float_to_uint(velocity, c.v_min, c.v_max, 12)
    kp_i = float_to_uint(kp, c.kp_min, c.kp_max, 12)
    kd_i = float_to_uint(kd, c.kd_min, c.kd_max, 12)
    t = float_to_uint(torque, c.t_min, c.t_max, 12)

    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0xF) << 4) | (kp_i >> 8),
            kp_i & 0xFF,
            (kd_i >> 4) & 0xFF,
            ((kd_i & 0xF) << 4) | (t >> 8),
            t & 0xFF,
        ]
    )


def hold_command(motor_type: MotorType) -> bytes:
    """A zero-torque, zero-gain 'read-only' command (won't move the joint).

    Safe to send just to elicit a feedback frame without commanding motion.
    """
    return pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0, motor_type)


# ---------------------------------------------------------------------------
# Fault codes  (high nibble of feedback byte 0)
# ---------------------------------------------------------------------------
ERROR_CODES: dict[int, str] = {
    0x0: "disabled",
    0x1: "normal",
    0x8: "over voltage",
    0x9: "under voltage",
    0xA: "over current",
    0xB: "MOSFET over temperature",
    0xC: "rotor over temperature",
    0xD: "loss of communication",
    0xE: "overload",
}
NORMAL_CODE = 0x1


@dataclass
class Feedback:
    """Decoded 8-byte DM feedback frame."""

    error_code: int
    position: float      # rad
    velocity: float      # rad/s
    torque: float        # N·m
    temp_mos: int        # deg C
    temp_rotor: int      # deg C

    @property
    def error_text(self) -> str:
        return ERROR_CODES.get(self.error_code, f"unknown(0x{self.error_code:X})")

    @property
    def healthy(self) -> bool:
        return self.error_code == NORMAL_CODE

    @classmethod
    def decode(cls, data: bytes, motor_type: MotorType) -> "Feedback":
        if len(data) < 8:
            raise ValueError(f"feedback frame too short: {len(data)} bytes")
        c = motor_type.constants
        error_int = (data[0] & 0xF0) >> 4
        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        return cls(
            error_code=error_int,
            position=uint_to_float(p_int, c.p_min, c.p_max, 16),
            velocity=uint_to_float(v_int, c.v_min, c.v_max, 12),
            torque=uint_to_float(t_int, c.t_min, c.t_max, 12),
            temp_mos=data[6],
            temp_rotor=data[7],
        )


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0
