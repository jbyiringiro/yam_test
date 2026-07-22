"""Auto-detect which arm (leader vs follower) is on the bus.

J1-J6 are identical on both arms, so we distinguish by the end effector:
  * follower -> a DM gripper motor answers at id 0x07 (feedback on 0x17)
  * leader   -> a passive encoder answers at id 0x50E (reply on 0x50F)

Whichever responds identifies the arm. Handy because each arm has its own bus
(i2rt uses can_leader_* / can_follower_*), so normally exactly one is present.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .config import ArmConfig
from .motor_chain import MotorChain

# Fast, bounded probes so detection is quick and never hangs.
_PROBE_TIMEOUT = 0.02
_PROBE_RETRIES = 5


@dataclass
class ArmDetection:
    arm: Optional[str]           # "follower" | "leader" | None (unknown)
    gripper_responded: bool
    trigger_responded: bool
    ambiguous: bool = False      # both answered (unexpected on a single-arm bus)

    @property
    def text(self) -> str:
        if self.ambiguous:
            return "ambiguous — both a gripper and a trigger responded"
        if self.arm:
            return f"{self.arm} arm detected"
        return "no end effector responded (arm off, wrong bus, or no gripper/trigger)"


def detect_arm(chain: MotorChain, cfg: ArmConfig) -> ArmDetection:
    gripper_ok = False
    trigger_ok = False

    if cfg.follower_gripper is not None:
        fb = chain.read(cfg.follower_gripper.motor_id, cfg.follower_gripper.motor_type,
                        _PROBE_TIMEOUT, _PROBE_RETRIES)
        gripper_ok = fb is not None

    if cfg.leader_trigger is not None:
        rd = chain.read_encoder(cfg.leader_trigger.encoder_id, cfg.leader_trigger.range_rad,
                                _PROBE_TIMEOUT, _PROBE_RETRIES)
        trigger_ok = rd is not None

    if gripper_ok and trigger_ok:
        return ArmDetection("follower", gripper_ok, trigger_ok, ambiguous=True)
    if gripper_ok:
        return ArmDetection("follower", gripper_ok, trigger_ok)
    if trigger_ok:
        return ArmDetection("leader", gripper_ok, trigger_ok)
    return ArmDetection(None, gripper_ok, trigger_ok)
