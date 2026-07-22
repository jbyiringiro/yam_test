"""Load and model the YAM Pro config (config/yam_pro.yaml)."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional

import yaml

from .dm_motor import MotorType


_DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "config",
    "yam_pro.yaml",
)


@dataclass
class JointCfg:
    name: str
    motor_id: int
    motor_type: MotorType
    role: str = ""
    range_deg: tuple[float, float] = (-180.0, 180.0)


@dataclass
class TriggerCfg:
    """Leader arm's passive-encoder trigger handle (read-only input)."""
    name: str
    encoder_id: int
    range_rad: float = 0.7


@dataclass
class Thresholds:
    temp_warn_c: float = 55.0
    temp_fail_c: float = 75.0
    move_kp: float = 3.0
    move_kd: float = 1.0
    move_settle_s: float = 1.0
    # Live (jog / exercise) safety envelope
    live_kp_scale: float = 0.5     # fraction of reference kp used to hold position
    live_max_vel_deg: float = 20.0  # slew limit: max commanded deg/s (bounds speed)
    live_rate_hz: float = 100.0    # control-loop rate
    live_step_deg: float = 2.0     # jog step per key press
    live_exercise_amp_deg: float = 10.0   # exercise oscillation amplitude
    live_exercise_period_s: float = 4.0   # exercise oscillation period
    live_torque_limit_nm: float = 4.0     # freeze a joint's command above this torque
                                          # (prevents current runaway that trips the supply)


@dataclass
class ArmConfig:
    name: str
    interface: Optional[str]   # default CAN interface (e.g. gs_usb)
    channel: Optional[str]     # default channel (e.g. "0")
    bitrate: int
    extended_id: bool
    poll_timeout_s: float
    poll_retries: int
    joints: list[JointCfg] = field(default_factory=list)
    # End effector, resolved by arm role (see set_arm):
    arm: str = "follower"                       # "follower" | "leader"
    gripper: Optional[JointCfg] = None          # follower: DM gripper motor
    trigger: Optional[TriggerCfg] = None        # leader: passive-encoder trigger
    # Both variants available for auto-detection, regardless of active arm:
    follower_gripper: Optional[JointCfg] = None
    leader_trigger: Optional[TriggerCfg] = None
    thresholds: Thresholds = field(default_factory=Thresholds)
    reference_gains: dict = field(default_factory=dict)

    def set_arm(self, arm: str) -> "ArmConfig":
        """Select which end effector is active (follower gripper vs leader trigger)."""
        self.arm = arm
        if arm == "leader":
            self.gripper = None
            self.trigger = self.leader_trigger
        else:  # follower (default)
            self.gripper = self.follower_gripper
            self.trigger = None
        return self

    def all_motors(self, include_gripper: bool = True) -> list[JointCfg]:
        """DM motors present. The leader trigger is NOT a motor, so it's excluded."""
        motors = list(self.joints)
        if include_gripper and self.gripper is not None:
            motors.append(self.gripper)
        return motors


def _joint_from_dict(d: dict) -> JointCfg:
    lo, hi = d.get("range_deg", [-180, 180])
    return JointCfg(
        name=d["name"],
        motor_id=int(d["motor_id"]),
        motor_type=MotorType(d["motor_type"]),
        role=d.get("role", ""),
        range_deg=(float(lo), float(hi)),
    )


def load_config(path: Optional[str] = None, arm: str = "follower") -> ArmConfig:
    path = path or _DEFAULT_CONFIG
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    can = raw.get("can", {})
    th = raw.get("thresholds", {})

    # End-effector variants: follower has a DM gripper, leader has an encoder trigger.
    ees = raw.get("end_effectors", {})
    follower_gripper = None
    fg = ees.get("follower")
    if fg and fg.get("enabled", True):
        follower_gripper = _joint_from_dict(fg)
    leader_trigger = None
    lt = ees.get("leader")
    if lt and lt.get("enabled", True):
        leader_trigger = TriggerCfg(
            name=lt.get("name", "trigger"),
            encoder_id=int(lt["encoder_id"]),
            range_rad=float(lt.get("range_rad", 0.7)),
        )

    cfg = ArmConfig(
        name=raw.get("name", "YAM_PRO"),
        interface=can.get("interface"),
        channel=(str(can["channel"]) if can.get("channel") is not None else None),
        bitrate=int(can.get("bitrate", 1_000_000)),
        extended_id=bool(can.get("extended_id", False)),
        poll_timeout_s=float(can.get("poll_timeout_s", 0.01)),
        poll_retries=int(can.get("poll_retries", 15)),
        joints=[_joint_from_dict(j) for j in raw.get("joints", [])],
        follower_gripper=follower_gripper,
        leader_trigger=leader_trigger,
        thresholds=Thresholds(
            temp_warn_c=float(th.get("temp_warn_c", 55)),
            temp_fail_c=float(th.get("temp_fail_c", 75)),
            move_kp=float(th.get("move_kp", 3.0)),
            move_kd=float(th.get("move_kd", 1.0)),
            move_settle_s=float(th.get("move_settle_s", 1.0)),
            live_kp_scale=float(th.get("live_kp_scale", 0.5)),
            live_max_vel_deg=float(th.get("live_max_vel_deg", 20.0)),
            live_rate_hz=float(th.get("live_rate_hz", 100.0)),
            live_step_deg=float(th.get("live_step_deg", 2.0)),
            live_exercise_amp_deg=float(th.get("live_exercise_amp_deg", 10.0)),
            live_exercise_period_s=float(th.get("live_exercise_period_s", 4.0)),
            live_torque_limit_nm=float(th.get("live_torque_limit_nm", 4.0)),
        ),
        reference_gains=raw.get("reference_gains", {}),
    )
    cfg.set_arm(arm)
    return cfg
