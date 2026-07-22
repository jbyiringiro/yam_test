"""Offline unit tests for the DM motor protocol — no hardware required.

Run:  pytest    (or:  python -m pytest tests/)
These lock in the exact byte packing / scaling that matches the real YAM arm.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from arm_test import dm_motor
from arm_test.dm_motor import (
    Feedback,
    MotorType,
    float_to_uint,
    uint_to_float,
    pack_mit_command,
)


# ---- scaling round-trips --------------------------------------------------
def test_float_uint_roundtrip_16bit():
    for v in (-12.5, -6.0, 0.0, 6.0, 12.5):
        u = float_to_uint(v, -12.5, 12.5, 16)
        back = uint_to_float(u, -12.5, 12.5, 16)
        assert abs(back - v) < 25.0 / 65535 + 1e-9


def test_float_to_uint_denominator_is_2pow_minus_1():
    # max value maps to full-scale (2^bits - 1), matching i2rt/Damiao
    assert float_to_uint(12.5, -12.5, 12.5, 16) == 65535
    assert float_to_uint(-12.5, -12.5, 12.5, 16) == 0
    assert float_to_uint(500.0, 0.0, 500.0, 12) == 4095


def test_float_to_uint_clamps():
    assert float_to_uint(999.0, -12.5, 12.5, 16) == 65535
    assert float_to_uint(-999.0, -12.5, 12.5, 16) == 0


# ---- motor constants ------------------------------------------------------
def test_dm4310_constants():
    c = MotorType.DM4310.constants
    assert (c.p_max, c.v_max, c.t_max) == (12.5, 30.0, 10.0)
    assert (c.kp_max, c.kd_max) == (500.0, 5.0)


def test_dm4340_uses_i2rt_velocity_10_not_stock_8():
    c = MotorType.DM4340.constants
    assert (c.p_max, c.v_max, c.t_max) == (12.5, 10.0, 28.0)


# ---- MIT frame packing ----------------------------------------------------
def test_mit_zero_command_layout():
    # pos/vel/torque=0 are MIDPOINTS of their symmetric ranges (not min);
    # kp/kd=0 are the true minimum (ranges are 0..max).
    data = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0, MotorType.DM4310)
    assert len(data) == 8
    assert data[0] == 0x7F and data[1] == 0xFF   # position midpoint -> 32767
    assert data[2] == 0x7F                        # velocity midpoint (2047 >> 4)
    assert data[4] == 0x00                        # kp low byte (kp=0, true min)
    assert data[5] == 0x00                        # kd byte (kd=0, true min)


def test_hold_command_is_zero_gain():
    data = dm_motor.hold_command(MotorType.DM4340)
    # kp and kd fields must be zero -> byte4 (kp low) == 0, byte5 (kd) == 0
    assert data[4] == 0x00
    assert data[5] == 0x00


# ---- CAN id scheme --------------------------------------------------------
def test_id_scheme():
    assert dm_motor.tx_arbitration_id(0x03) == 0x03
    assert dm_motor.rx_arbitration_id(0x03) == 0x13   # +16
    assert dm_motor.motor_id_from_rx(0x16) == 0x06


# ---- special commands -----------------------------------------------------
def test_special_command_payloads():
    assert dm_motor.CMD_ENABLE == bytes([0xFF] * 7 + [0xFC])
    assert dm_motor.CMD_DISABLE == bytes([0xFF] * 7 + [0xFD])
    assert dm_motor.CMD_SAVE_ZERO == bytes([0xFF] * 7 + [0xFE])
    assert dm_motor.CMD_CLEAN_ERROR == bytes([0xFF] * 7 + [0xFB])


# ---- feedback decode ------------------------------------------------------
def test_feedback_decode_normal_and_faults():
    # error nibble 0x1 = normal; position mid (0x7F 0xFF) -> ~0 rad
    data = bytes([0x11, 0x7F, 0xFF, 0x80, 0x00, 0x00, 30, 35])
    fb = Feedback.decode(data, MotorType.DM4310)
    assert fb.error_code == 0x1 and fb.healthy
    assert abs(fb.position) < 0.01
    assert fb.temp_mos == 30 and fb.temp_rotor == 35

    fault = bytes([0xB1, 0x7F, 0xFF, 0x80, 0x00, 0x00, 90, 95])
    fb2 = Feedback.decode(fault, MotorType.DM4310)
    assert fb2.error_code == 0xB
    assert "MOSFET" in fb2.error_text
    assert not fb2.healthy


def test_error_code_table():
    assert dm_motor.ERROR_CODES[0x8] == "over voltage"
    assert dm_motor.ERROR_CODES[0x9] == "under voltage"
    assert dm_motor.ERROR_CODES[0xA] == "over current"
    assert dm_motor.ERROR_CODES[0xD] == "loss of communication"
    assert dm_motor.ERROR_CODES[0xE] == "overload"


def test_pack_decode_position_roundtrip():
    # command a position, feed identical uint back through decode
    import struct
    pos_rad = 1.234
    data = pack_mit_command(pos_rad, 0, 0, 0, 0, MotorType.DM4310)
    p_int = (data[0] << 8) | data[1]
    fb_data = bytes([0x10 | 0x01, data[0], data[1], 0x80, 0x00, 0x00, 25, 25])
    fb = Feedback.decode(fb_data, MotorType.DM4310)
    assert abs(fb.position - pos_rad) < 25.0 / 65535 + 1e-6
