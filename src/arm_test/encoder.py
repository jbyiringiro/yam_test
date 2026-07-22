"""Passive encoder protocol — the leader arm's TRIGGER handle.

The YAM *leader* arm does NOT have a DM gripper motor at joint 7. Instead its
teaching handle exposes a passive CAN **encoder** (the trigger + two buttons),
read-only. This matches i2rt's `PassiveEncoderReader` in
`motor_drivers/dm_driver.py`.

Protocol (matched to i2rt):
  * Encoder CAN id: 0x50E (i2rt `get_encoder_chain([0x50E])`).
  * Poll: send data [0xFF, 0x02] to the encoder id.
  * Reply arrives on encoder_id + 1  (ReceiveMode.plus_one) -> 0x50F.
  * Reply payload, big-endian struct "!B h h B":
        uint8  device_id
        int16  position   (raw)   -> rad = raw * 2*pi / 4096
        int16  velocity   (raw)   -> rad/s = raw * 2*pi / 4096
        uint8  digital_inputs     -> button0 = bit0, button1 = bit1
  * Trigger value (what drives the follower gripper) is the position normalized
    to 0..1 over +/- range_rad:  norm = |pos| / range_rad ; gripper_cmd = 1 - norm.

Pure/offline — no CAN I/O — so it is unit-testable without hardware.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

TWO_PI = 6.283185307179586

# Defaults from i2rt.
PASSIVE_ENCODER_ID = 0x50E          # leader trigger handle
ENCODER_RX_OFFSET = 1               # ReceiveMode.plus_one -> reply on id + 1
ENCODER_POLL_PAYLOAD = bytes([0xFF, 0x02])
ENCODER_DEFAULT_RANGE_RAD = 0.7
_ENCODER_STRUCT = "!BhhB"           # uint8, int16, int16, uint8  (6 bytes)


def encoder_rx_id(encoder_id: int = PASSIVE_ENCODER_ID) -> int:
    return encoder_id + ENCODER_RX_OFFSET


@dataclass
class EncoderReading:
    device_id: int
    position_rad: float      # raw encoder angle, radians
    velocity_rad: float      # rad/s
    buttons: tuple[bool, bool]
    trigger: float           # normalized 0..1 (0 = released, 1 = fully squeezed)

    @property
    def gripper_cmd(self) -> float:
        """What this trigger would command a follower gripper (i2rt: 1 - norm)."""
        return 1.0 - self.trigger


def decode_encoder(data: bytes, range_rad: float = ENCODER_DEFAULT_RANGE_RAD) -> EncoderReading:
    """Decode a 6-byte passive-encoder reply frame."""
    if len(data) < 6:
        raise ValueError(f"encoder frame too short: {len(data)} bytes (need 6)")
    device_id, pos_raw, vel_raw, digital = struct.unpack(_ENCODER_STRUCT, bytes(data[:6]))
    position_rad = pos_raw * TWO_PI / 4096.0
    velocity_rad = vel_raw * TWO_PI / 4096.0
    button0 = bool(digital & 0x1)
    button1 = bool((digital >> 1) & 0x1)

    # normalize position to a 0..1 trigger value over +/- range_rad
    clamped = max(-range_rad, min(range_rad, position_rad))
    trigger = abs(clamped) / range_rad if range_rad else 0.0

    return EncoderReading(
        device_id=device_id,
        position_rad=position_rad,
        velocity_rad=velocity_rad,
        buttons=(button0, button1),
        trigger=trigger,
    )
