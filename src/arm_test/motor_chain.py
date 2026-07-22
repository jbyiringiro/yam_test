"""Multi-motor CAN manager for the YAM arm (the 'motor chain').

Thin, synchronous request/response layer over a python-can bus that speaks the
DM MIT protocol. It sends a command to a motor and waits for that motor's
feedback frame (arbitration id = motor_id + 16), retrying briefly — the same
poll model i2rt's DMChainCanInterface uses.

Kept deliberately simple: diagnostics send one command at a time and read the
reply. This is NOT a real-time control loop.
"""

from __future__ import annotations

import time
from typing import Optional

from . import dm_motor
from . import encoder as enc
from .config import ArmConfig, JointCfg
from .dm_motor import Feedback, MotorType
from .encoder import EncoderReading


class MotorChain:
    def __init__(self, bus, cfg: ArmConfig):
        self.bus = bus
        self.cfg = cfg

    # ---- low-level send/recv ---------------------------------------------
    def _send(self, arbitration_id: int, data: bytes) -> None:
        import can

        self.bus.send(
            can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=self.cfg.extended_id,
            )
        )

    def _send_and_get_feedback(
        self,
        motor_id: int,
        motor_type: MotorType,
        data: bytes,
        timeout: Optional[float] = None,
        retries: Optional[int] = None,
    ) -> Optional[Feedback]:
        """Send a command to `motor_id`, wait for its feedback frame.

        Returns the decoded Feedback, or None if the motor never replied.
        `timeout`/`retries` override the config defaults (the live loop passes
        small values so a missing motor never stalls the whole cycle).
        """
        want_rx = dm_motor.rx_arbitration_id(motor_id)
        deadline_retries = self.cfg.poll_retries if retries is None else retries
        recv_timeout = self.cfg.poll_timeout_s if timeout is None else timeout
        self._send(dm_motor.tx_arbitration_id(motor_id), data)

        for _ in range(deadline_retries):
            msg = self.bus.recv(timeout=recv_timeout)
            if msg is None:
                # resend once and keep waiting — motors reply to the last command
                self._send(dm_motor.tx_arbitration_id(motor_id), data)
                continue
            if getattr(msg, "is_error_frame", False):
                continue
            if msg.arbitration_id == want_rx and len(msg.data) >= 8:
                return Feedback.decode(bytes(msg.data), motor_type)
        return None

    # ---- motor commands ---------------------------------------------------
    def enable(self, motor_id: int, motor_type: MotorType) -> Optional[Feedback]:
        return self._send_and_get_feedback(motor_id, motor_type, dm_motor.CMD_ENABLE)

    def disable(self, motor_id: int, motor_type: MotorType) -> Optional[Feedback]:
        return self._send_and_get_feedback(motor_id, motor_type, dm_motor.CMD_DISABLE)

    def clean_error(self, motor_id: int, motor_type: MotorType) -> Optional[Feedback]:
        # clean_error goes to the raw arbitration id = motor_id (MIT offset 0)
        return self._send_and_get_feedback(motor_id, motor_type, dm_motor.CMD_CLEAN_ERROR)

    def save_zero(self, motor_id: int, motor_type: MotorType) -> Optional[Feedback]:
        return self._send_and_get_feedback(motor_id, motor_type, dm_motor.CMD_SAVE_ZERO)

    def read(
        self,
        motor_id: int,
        motor_type: MotorType,
        timeout: Optional[float] = None,
        retries: Optional[int] = None,
    ) -> Optional[Feedback]:
        """Elicit one feedback frame with a zero-gain, zero-torque hold (no motion)."""
        return self._send_and_get_feedback(
            motor_id, motor_type, dm_motor.hold_command(motor_type), timeout, retries
        )

    def command(
        self,
        motor_id: int,
        motor_type: MotorType,
        position: float,
        velocity: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
        timeout: Optional[float] = None,
        retries: Optional[int] = None,
    ) -> Optional[Feedback]:
        payload = dm_motor.pack_mit_command(position, velocity, kp, kd, torque, motor_type)
        return self._send_and_get_feedback(motor_id, motor_type, payload, timeout, retries)

    # ---- passive encoder (leader trigger handle) -------------------------
    def read_encoder(
        self,
        encoder_id: int = enc.PASSIVE_ENCODER_ID,
        range_rad: float = enc.ENCODER_DEFAULT_RANGE_RAD,
        timeout: Optional[float] = None,
        retries: Optional[int] = None,
    ) -> Optional[EncoderReading]:
        """Poll the leader's trigger encoder. Returns None if it never replied.

        Unlike DM motors, the encoder replies on encoder_id + 1 and uses its own
        frame format — so this does not go through _send_and_get_feedback.
        """
        import can

        want_rx = enc.encoder_rx_id(encoder_id)
        recv_timeout = self.cfg.poll_timeout_s if timeout is None else timeout
        tries = self.cfg.poll_retries if retries is None else retries

        self.bus.send(
            can.Message(arbitration_id=encoder_id, data=enc.ENCODER_POLL_PAYLOAD,
                        is_extended_id=self.cfg.extended_id)
        )
        for _ in range(tries):
            msg = self.bus.recv(timeout=recv_timeout)
            if msg is None:
                self.bus.send(
                    can.Message(arbitration_id=encoder_id, data=enc.ENCODER_POLL_PAYLOAD,
                                is_extended_id=self.cfg.extended_id)
                )
                continue
            if getattr(msg, "is_error_frame", False):
                continue
            if msg.arbitration_id == want_rx and len(msg.data) >= 6:
                return enc.decode_encoder(bytes(msg.data), range_rad)
        return None

    # ---- convenience over JointCfg ---------------------------------------
    def read_joint(self, joint: JointCfg) -> Optional[Feedback]:
        return self.read(joint.motor_id, joint.motor_type)

    def enable_joint(self, joint: JointCfg) -> Optional[Feedback]:
        return self.enable(joint.motor_id, joint.motor_type)

    def disable_joint(self, joint: JointCfg) -> Optional[Feedback]:
        return self.disable(joint.motor_id, joint.motor_type)

    def recover_joint(self, joint: JointCfg, tries: int = 3) -> Optional[Feedback]:
        """Try to clear a fault then re-enable (mirrors i2rt auto-recovery)."""
        fb = None
        for _ in range(tries):
            self.clean_error(joint.motor_id, joint.motor_type)
            time.sleep(0.01)
            fb = self.enable(joint.motor_id, joint.motor_type)
            if fb and fb.healthy:
                return fb
        return fb
