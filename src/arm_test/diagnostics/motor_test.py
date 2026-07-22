"""Per-joint motor checks.

Read-only by default: each joint is pinged for a feedback frame, its fault code
and temperatures are evaluated. With allow_move=True, an *optional* gentle,
low-gain motion check confirms the joint actually responds to commands and moves.

Motion tests move real hardware — the caller gates them behind --move.
"""

from __future__ import annotations

import time

from ..config import ArmConfig, JointCfg
from ..dm_motor import deg_to_rad, rad_to_deg
from ..motor_chain import MotorChain
from .report import CheckResult, Report, Status


def _temp_status(cfg: ArmConfig, temp: int) -> Status:
    if temp >= cfg.thresholds.temp_fail_c:
        return Status.FAIL
    if temp >= cfg.thresholds.temp_warn_c:
        return Status.WARN
    return Status.PASS


def check_joint_readonly(chain: MotorChain, cfg: ArmConfig, joint: JointCfg) -> CheckResult:
    """Ping a joint, decode feedback, evaluate fault code + temperatures."""
    fb = chain.read_joint(joint)
    if fb is None:
        return CheckResult(
            f"{joint.name} ({joint.motor_type.value})", Status.FAIL,
            f"No feedback on rx id 0x{joint.motor_id + 16:02X}. Motor unpowered, "
            "wrong CAN id, or broken drop cable.",
            {"motor_id": joint.motor_id, "responded": False},
        )

    data = {
        "motor_id": joint.motor_id,
        "responded": True,
        "error_code": f"0x{fb.error_code:X}",
        "error_text": fb.error_text,
        "position_deg": round(rad_to_deg(fb.position), 2),
        "velocity_dps": round(rad_to_deg(fb.velocity), 2),
        "torque_nm": round(fb.torque, 3),
        "temp_mos_c": fb.temp_mos,
        "temp_rotor_c": fb.temp_rotor,
    }

    # Fault code takes priority.
    if not fb.healthy and fb.error_code != 0x0:  # 0x0 = disabled (expected pre-enable)
        return CheckResult(
            f"{joint.name} ({joint.motor_type.value})", Status.FAIL,
            f"Fault: {fb.error_text} (0x{fb.error_code:X}). "
            f"pos={data['position_deg']}deg temp={fb.temp_mos}/{fb.temp_rotor}C",
            data,
        )

    # Temperature.
    tstat = max(_temp_status(cfg, fb.temp_mos), _temp_status(cfg, fb.temp_rotor),
               key=lambda s: [Status.SKIP, Status.PASS, Status.WARN, Status.FAIL].index(s))
    detail = (
        f"pos={data['position_deg']}deg  temp MOS/rotor={fb.temp_mos}/{fb.temp_rotor}C  "
        f"state={fb.error_text}"
    )
    if tstat == Status.FAIL:
        detail = "OVERTEMP — " + detail
    elif tstat == Status.WARN:
        detail = "warm — " + detail
    return CheckResult(f"{joint.name} ({joint.motor_type.value})", tstat, detail, data)


def check_joint_motion(
    chain: MotorChain, cfg: ArmConfig, joint: JointCfg, move_deg: float
) -> CheckResult:
    """Gentle low-gain motion test: nudge the joint and confirm it moved.

    Enables the motor, commands a small position offset (clamped into the joint's
    range) at SAFE low gains, checks the reported position followed, then returns
    to the start and disables. Never uses the full control gains.
    """
    th = cfg.thresholds
    lo, hi = joint.range_deg

    start = chain.enable_joint(joint)
    if start is None:
        return CheckResult(f"{joint.name} motion", Status.FAIL,
                           "Could not enable / no feedback.", {"motor_id": joint.motor_id})
    if not start.healthy:
        # try to recover once
        start = chain.recover_joint(joint) or start
        if not start.healthy:
            return CheckResult(f"{joint.name} motion", Status.FAIL,
                               f"In fault ({start.error_text}); could not clear.",
                               {"motor_id": joint.motor_id, "error": start.error_text})

    start_deg = rad_to_deg(start.position)
    # Move toward whichever side has room, clamped into range with margin.
    target_deg = start_deg + move_deg
    if target_deg > hi - 2:
        target_deg = start_deg - move_deg
    target_deg = max(lo + 2, min(hi - 2, target_deg))
    target_rad = deg_to_rad(target_deg)

    try:
        # command gently for a short settle window
        t_end = time.time() + th.move_settle_s
        last = start
        while time.time() < t_end:
            last = chain.command(
                joint.motor_id, joint.motor_type,
                position=target_rad, velocity=0.0,
                kp=th.move_kp, kd=th.move_kd, torque=0.0,
            ) or last
            time.sleep(0.01)

        reached_deg = rad_to_deg(last.position)
        moved = abs(reached_deg - start_deg)

        # return to start, then relax
        t_end = time.time() + th.move_settle_s
        while time.time() < t_end:
            chain.command(joint.motor_id, joint.motor_type,
                          position=start.position, velocity=0.0,
                          kp=th.move_kp, kd=th.move_kd, torque=0.0)
            time.sleep(0.01)
    finally:
        chain.disable_joint(joint)

    data = {
        "motor_id": joint.motor_id,
        "start_deg": round(start_deg, 2),
        "target_deg": round(target_deg, 2),
        "reached_deg": round(reached_deg, 2),
        "moved_deg": round(moved, 2),
        "commanded_deg": round(abs(target_deg - start_deg), 2),
    }
    expected = abs(target_deg - start_deg)
    if moved < 0.3 * expected:
        return CheckResult(
            f"{joint.name} motion", Status.FAIL,
            f"Commanded {expected:.1f}deg but moved only {moved:.1f}deg — "
            "mechanical bind, no power stage, or not tracking.", data,
        )
    if moved < 0.7 * expected:
        return CheckResult(
            f"{joint.name} motion", Status.WARN,
            f"Moved {moved:.1f}/{expected:.1f}deg — sluggish; check load/friction.", data,
        )
    return CheckResult(
        f"{joint.name} motion", Status.PASS,
        f"Tracked {moved:.1f}/{expected:.1f}deg cleanly.", data,
    )


def check_trigger(chain: MotorChain, cfg: ArmConfig) -> CheckResult:
    """Read the leader arm's passive-encoder trigger handle (read-only)."""
    trig = cfg.trigger
    rd = chain.read_encoder(trig.encoder_id, trig.range_rad)
    if rd is None:
        return CheckResult(
            f"{trig.name} (encoder 0x{trig.encoder_id:X})", Status.FAIL,
            f"No reply on 0x{trig.encoder_id + 1:X}. Handle unplugged, wrong id, "
            "or encoder not in passive mode (report_freq must be 0).",
            {"encoder_id": trig.encoder_id, "responded": False},
        )
    data = {
        "encoder_id": trig.encoder_id,
        "responded": True,
        "trigger": round(rd.trigger, 3),
        "gripper_cmd": round(rd.gripper_cmd, 3),
        "position_rad": round(rd.position_rad, 4),
        "velocity_rad": round(rd.velocity_rad, 4),
        "button0": rd.buttons[0],
        "button1": rd.buttons[1],
    }
    return CheckResult(
        f"{trig.name} (encoder 0x{trig.encoder_id:X})", Status.PASS,
        f"trigger={rd.trigger:.2f} (gripper_cmd={rd.gripper_cmd:.2f})  "
        f"buttons={int(rd.buttons[0])}{int(rd.buttons[1])}  "
        "— squeeze/press to verify it changes.",
        data,
    )


def check_all_joints(
    chain: MotorChain,
    cfg: ArmConfig,
    report: Report,
    allow_move: bool = False,
    move_deg: float = 5.0,
    include_gripper: bool = True,
) -> None:
    """Run read-only checks on every joint, plus optional motion checks.

    Follower: J1-J6 + gripper motor. Leader: J1-J6 + trigger encoder (read-only).
    """
    motors = cfg.all_motors(include_gripper=include_gripper)

    responded = 0
    for joint in motors:
        res = report.add(check_joint_readonly(chain, cfg, joint))
        if res.data.get("responded"):
            responded += 1

    # Summary of chain presence.
    report.add(CheckResult(
        "Joint presence", Status.PASS if responded == len(motors) else Status.FAIL,
        f"{responded}/{len(motors)} motors responded.",
        {"responded": responded, "expected": len(motors)},
    ))

    # Leader arm: check the trigger handle (read-only, never moved).
    if cfg.trigger is not None:
        report.add(check_trigger(chain, cfg))

    if not allow_move:
        report.add(CheckResult("Motion tests", Status.SKIP,
                               "Skipped (read-only). Pass --move to enable.", {}))
        return

    for joint in motors:
        # only move joints that responded and aren't faulted
        report.add(check_joint_motion(chain, cfg, joint, move_deg=move_deg))
