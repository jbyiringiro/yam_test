# Motors & CAN Protocol (DM4310 / DM4340)

Wire-level reference for the Damiao (DM) actuators in the YAM Pro, matched to how
i2rt actually drives them (`i2rt/motor_drivers/dm_driver.py`). This is what the
toolkit's [dm_motor.py](../src/arm_test/dm_motor.py) implements — read this to
understand or hand-verify any frame.

## The actuators

| | DM4310 | DM4340 |
|---|---|---|
| Used on | J4–J6 (elbow/wrist) + gripper | J1–J3 (shoulder) |
| Position range | ±12.5 rad | ±12.5 rad |
| Velocity range | ±30 rad/s | **±10 rad/s** |
| Torque range | ±10 N·m | ±28 N·m |
| Kp range | 0–500 | 0–500 |
| Kd range | 0–5 | 0–5 |

> **⚠ DM4340 velocity = ±10, not ±8.** i2rt hard-codes the 48 V velocity constant
> (10), while Damiao's stock 24 V table lists 8. Velocity encode/decode *must* use
> 10 or decoded speeds won't match the real arm. DM4310 matches stock exactly.

## CAN summary

- **Bitrate:** 1 Mbit/s. **Frames:** standard 11-bit. **Mode:** MIT.
- **Send** a command to a motor on arbitration ID = `motor_id` (MIT offset 0).
- **Receive** its feedback on arbitration ID = **`motor_id + 16`** (0x10 + id).
- Control loop polls each motor ~250 Hz. Factory firmware has a **400 ms safety
  timeout** — if a motor gets no command for 400 ms it faults into damping.

### ID map

| Joint | motor_id | TX id | RX id |
|---|---|---|---|
| J1 | 0x01 | 0x01 | 0x11 |
| J2 | 0x02 | 0x02 | 0x12 |
| J3 | 0x03 | 0x03 | 0x13 |
| J4 | 0x04 | 0x04 | 0x14 |
| J5 | 0x05 | 0x05 | 0x15 |
| J6 | 0x06 | 0x06 | 0x16 |
| Gripper | 0x07 | 0x07 | 0x17 |

## MIT control frame (8 bytes, TX)

Fields and bit widths: position 16, velocity 12, kp 12, kd 12, torque 12.

Each float is mapped to an unsigned int over its range with denominator
`2^bits − 1`:

```
u = round( (clamp(x, xmin, xmax) − xmin) · (2^bits − 1) / (xmax − xmin) )
```

Byte packing:

| Byte | Contents |
|---|---|
| 0 | position[15:8] |
| 1 | position[7:0] |
| 2 | velocity[11:4] |
| 3 | velocity[3:0] << 4 \| kp[11:8] |
| 4 | kp[7:0] |
| 5 | kd[11:4] |
| 6 | kd[3:0] << 4 \| torque[11:8] |
| 7 | torque[7:0] |

Because ranges are symmetric (±max), a commanded **0** for position/velocity/torque
sits at the numeric *midpoint* (e.g. position 0 → 32767 → bytes `7F FF`). Kp/Kd
ranges are 0–max, so 0 there is the true minimum (bytes 0).

A **read-only "hold"** (used by `yam-test motors`) sends position 0, velocity 0,
**kp 0, kd 0, torque 0** — zero gains means no motion; it just elicits a feedback frame.

## Feedback frame (8 bytes, RX)

Arrives on `motor_id + 16`.

| Byte | Contents |
|---|---|
| 0 | high nibble = **error/status code**; low nibble = motor id echo (i2rt ignores it, uses the arbitration id) |
| 1 | position[15:8] |
| 2 | position[7:0] |
| 3 | velocity[11:4] |
| 4 | high nibble = velocity[3:0]; low nibble = torque[11:8] |
| 5 | torque[7:0] |
| 6 | MOSFET temperature (°C, raw uint8) |
| 7 | rotor temperature (°C, raw uint8) |

Decode position/velocity/torque with the inverse of the encode scaling and the
**same motor-type constants**. Temperatures are plain integer °C.

## Special commands (8 bytes)

Seven `0xFF` bytes plus a terminator, sent to the motor's TX id:

| Command | Payload | Use |
|---|---|---|
| Enable (motor on) | `FF FF FF FF FF FF FF FC` | Energize before commanding motion |
| Disable (motor off) | `FF FF FF FF FF FF FF FD` | Relax / safe |
| Save zero position | `FF FF FF FF FF FF FF FE` | Set current pose as home (re-zeroing) |
| Clean error | `FF FF FF FF FF FF FF FB` | Clear a fault state |

**Recovery pattern** (what the toolkit's `recover_joint` does, mirroring i2rt):
send *clean error* (`FB`), then *enable* (`FC`), repeat up to 3× until the feedback
error code reads normal (`0x1`).

## Fault codes {#fault-codes}

High nibble of feedback byte 0:

| Code | Meaning | What it usually indicates |
|---|---|---|
| 0x0 | disabled | Motor simply not enabled yet (expected before `FC`) |
| **0x1** | **normal** | Healthy — this is the good state |
| 0x8 | over voltage | Supply too high / regen spike |
| 0x9 | under voltage | Supply sag, bad power connector, brown-out |
| 0xA | over current | Mechanical bind or short |
| 0xB | MOSFET over temp | Driver overheating — load/duty too high |
| 0xC | rotor over temp | Motor overheating — bind, overload, bad bearing |
| 0xD | loss of communication | Missed the 400 ms command deadline / bus dropout |
| 0xE | overload | Sustained torque beyond limit |

The toolkit treats any code other than `0x1` (or `0x0` before enable) as a fault,
reports it, and can attempt one recovery cycle.

## Gripper (LINEAR_4310)

A DM4310 at CAN id **0x07**. i2rt runs it with `kp 20.0`, `kd 0.5`, mapping a motor
stroke of ~6.57 rad to ~0.096 m of linear travel, with clog detection
(`clog_force_threshold 0.5`, `clog_speed_threshold 0.3`). It needs calibration
before its absolute limits are known. For diagnostics it's treated like any other
DM4310 joint (present / feedback / temperature / fault).

## Gearing & zero

No per-joint gear-ratio constant is stored anywhere in i2rt — reported joint values
are already at the output. i2rt applies only a software zero offset (radians) and a
direction (±1). If a joint reads offset after service, re-home it with **save zero**
(`FE`) and log the change.

## Sources

- i2rt driver: `github.com/i2rt-robotics/i2rt` → `motor_drivers/dm_driver.py`, `utils.py`, `config/yam_pro.yml`
- Damiao DM-J4310 manual; `cmjang/DM_Motor_Control/damiao.h` (stock constants)
