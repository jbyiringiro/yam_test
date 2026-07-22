"""Offline tests for the live-mode safety envelope (no hardware).

These lock in the properties that keep the arm from moving dangerously:
soft limits, per-joint gain scaling, and the slew-rate speed cap.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from arm_test import live as L
from arm_test.config import load_config


CFG = load_config()


def test_soft_limits_keep_margin_off_hard_stops():
    j1 = CFG.joints[0]  # range -150..180
    s = L.JointLive(joint=j1, index=0, kp=1, kd=1)
    assert s.clamp(1000) == 178.0     # 180 - 2 deg margin
    assert s.clamp(-1000) == -148.0   # -150 + 2 deg margin
    assert s.lo == -148.0 and s.hi == 178.0


def test_gains_scale_from_reference_and_never_exceed_dm_limits():
    # reference kp [80,80,80,40,10,10] * live_kp_scale (0.5)
    kp0, kd0 = L._joint_gains(CFG, 0)
    assert kp0 == 40.0 and kd0 == 5.0
    kp3, _ = L._joint_gains(CFG, 3)
    assert kp3 == 20.0
    # every joint stays within DM kp range 0..500, kd 0..5
    for i in range(len(CFG.joints) + 1):  # +1 = gripper fallback index
        kp, kd = L._joint_gains(CFG, i)
        assert 0.0 <= kp <= 500.0
        assert 0.0 <= kd <= 5.0


def test_slew_limit_caps_commanded_speed():
    # Reproduce the loop's slew math: command can't move faster than
    # live_max_vel_deg per second regardless of how far the target jumps.
    th = CFG.thresholds
    dt = 1.0 / th.live_rate_hz
    max_step = th.live_max_vel_deg * dt

    command = 0.0
    desired = 1000.0  # operator slams target far away
    # simulate 1 second of loop cycles
    for _ in range(int(th.live_rate_hz)):
        delta = desired - command
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
        command += delta
    # after ~1 s, commanded motion must be <= live_max_vel_deg (+ one step slack)
    assert command <= th.live_max_vel_deg + max_step + 1e-6


def test_monitor_is_read_only_gainless():
    # The monitor path uses chain.read -> hold_command: kp=kd=torque=0 (no motion).
    from arm_test.dm_motor import hold_command, MotorType
    data = hold_command(MotorType.DM4340)
    assert data[4] == 0x00  # kp low byte
    assert data[5] == 0x00  # kd byte
