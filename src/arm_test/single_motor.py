"""Single-motor bench test — check ONE DM motor by CAN id and write a report.

Ideal for the swap test ("is this actuator or its wiring bad?"), checking a
spare before installing it, or isolating a stubborn joint like J6. Evaluates
against the DM-J4310-2EC datasheet limits (encoded in dm_motor.MotorConstants).

Read-only by default. `--enable` tests whether the motor actually energizes
(the "won't turn green" check); `--move` adds a gentle motion test.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from .config import ArmConfig, JointCfg
from .dm_motor import MotorType, rad_to_deg
from .diagnostics import motor_test
from .diagnostics.report import CheckResult, Report, Status
from .motor_chain import MotorChain


def _datasheet_row(motor_type: MotorType) -> CheckResult:
    c = motor_type.constants
    if c.rated_torque is None:
        return CheckResult(
            f"{motor_type.value} datasheet", Status.SKIP,
            "No datasheet ratings on hand for this motor type.", {})
    return CheckResult(
        f"{motor_type.value} datasheet", Status.PASS,
        f"rated {c.rated_torque} N·m / {c.rated_current} A · "
        f"peak {c.peak_torque} N·m / {c.peak_current} A · "
        f"{c.reduction_ratio:.0f}:1 · Kt {c.torque_constant} N·m/A",
        {
            "rated_torque_nm": c.rated_torque, "peak_torque_nm": c.peak_torque,
            "rated_current_a": c.rated_current, "peak_current_a": c.peak_current,
            "reduction_ratio": c.reduction_ratio, "torque_constant_nm_a": c.torque_constant,
        },
    )


def check_enable(chain: MotorChain, joint: JointCfg) -> CheckResult:
    """Test that the motor actually ENABLES (LED green / state normal).

    This is the J6-style check: red-solid = powered + talking but won't leave
    disable mode. Tries a clear+enable, then disables again (no motion).
    """
    fb = chain.enable_joint(joint)
    if fb is None:
        return CheckResult(f"{joint.name} enable", Status.FAIL,
                           "No reply to the enable command (unpowered / CAN drop).", {})
    if not fb.healthy:
        fb = chain.recover_joint(joint) or fb
    ok = fb.healthy
    try:
        chain.disable_joint(joint)
    except Exception:
        pass
    if ok:
        return CheckResult(f"{joint.name} enable", Status.PASS,
                           "Enabled OK (LED should be GREEN, state normal).",
                           {"state": fb.error_text})
    return CheckResult(
        f"{joint.name} enable", Status.FAIL,
        f"Would NOT enable — stuck '{fb.error_text}' (0x{fb.error_code:X}). "
        "Red-solid LED = disable mode: suspect this actuator's driver "
        "(swap-test it, or check with the Damiao assistant).",
        {"state": fb.error_text, "code": f"0x{fb.error_code:X}"})


def test_single_motor(
    chain: MotorChain,
    cfg: ArmConfig,
    motor_id: int,
    motor_type: MotorType,
    name: str = "motor",
    do_enable: bool = False,
    do_move: bool = False,
    move_deg: float = 5.0,
    range_deg: Optional[tuple] = None,
) -> Report:
    joint = JointCfg(name=name, motor_id=motor_id, motor_type=motor_type,
                     range_deg=range_deg or (-180.0, 180.0))
    report = Report(
        title=f"Single-motor test — {name} (0x{motor_id:02X}, {motor_type.value})",
        timestamp=datetime.now(timezone.utc).isoformat(timespec="seconds"),
    )
    report.add(_datasheet_row(motor_type))
    report.add(motor_test.check_joint_readonly(chain, cfg, joint))
    if do_enable:
        report.add(check_enable(chain, joint))
    if do_move:
        report.add(motor_test.check_joint_motion(chain, cfg, joint, move_deg=move_deg))
    return report
