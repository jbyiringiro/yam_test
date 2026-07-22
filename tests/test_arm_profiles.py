"""Offline tests for leader/follower profiles + the trigger encoder protocol."""

import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from arm_test import encoder as enc
from arm_test.config import load_config


# ---- profiles -------------------------------------------------------------
def test_follower_has_gripper_no_trigger():
    cfg = load_config(arm="follower")
    names = [m.name for m in cfg.all_motors()]
    assert "gripper" in names
    assert len(names) == 7            # J1-J6 + gripper
    assert cfg.trigger is None
    assert cfg.arm == "follower"


def test_leader_has_trigger_no_gripper_motor():
    cfg = load_config(arm="leader")
    names = [m.name for m in cfg.all_motors()]
    assert "gripper" not in names
    assert len(names) == 6            # J1-J6 only; trigger is NOT a motor
    assert cfg.trigger is not None
    assert cfg.trigger.encoder_id == 0x50E
    assert cfg.arm == "leader"


def test_both_variants_available_for_detection():
    # regardless of active arm, both end-effector definitions load for auto-detect
    cfg = load_config(arm="leader")
    assert cfg.follower_gripper is not None and cfg.follower_gripper.motor_id == 0x07
    assert cfg.leader_trigger is not None and cfg.leader_trigger.encoder_id == 0x50E


def test_set_arm_switches_end_effector():
    cfg = load_config(arm="follower")
    cfg.set_arm("leader")
    assert cfg.gripper is None and cfg.trigger is not None
    cfg.set_arm("follower")
    assert cfg.gripper is not None and cfg.trigger is None


# ---- encoder protocol -----------------------------------------------------
def test_encoder_rx_is_id_plus_one():
    assert enc.encoder_rx_id(0x50E) == 0x50F


def test_encoder_poll_payload():
    assert enc.ENCODER_POLL_PAYLOAD == bytes([0xFF, 0x02])


def test_encoder_decode_position_and_buttons():
    # 4096 counts = 2*pi rad; 1024 -> pi/2
    data = struct.pack("!BhhB", 3, 1024, -512, 0b10)
    r = enc.decode_encoder(data, range_rad=0.7)
    assert r.device_id == 3
    assert abs(r.position_rad - (enc.TWO_PI / 4.0)) < 1e-6
    assert abs(r.velocity_rad - (-enc.TWO_PI / 8.0)) < 1e-6
    assert r.buttons == (False, True)   # bit0=0, bit1=1


def test_encoder_trigger_normalizes_and_gripper_cmd():
    # small position within range -> partial trigger
    raw = int(round(0.35 * 4096 / enc.TWO_PI))  # ~0.35 rad
    data = struct.pack("!BhhB", 1, raw, 0, 0)
    r = enc.decode_encoder(data, range_rad=0.7)
    assert 0.0 <= r.trigger <= 1.0
    assert abs(r.trigger - 0.5) < 0.02          # 0.35 / 0.7 = 0.5
    assert abs(r.gripper_cmd - (1.0 - r.trigger)) < 1e-9


def test_encoder_trigger_clamps_beyond_range():
    data = struct.pack("!BhhB", 1, 2000, 0, 0)  # way beyond range
    r = enc.decode_encoder(data, range_rad=0.7)
    assert r.trigger == 1.0
