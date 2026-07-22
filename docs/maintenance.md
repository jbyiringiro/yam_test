# YAM Pro — Maintenance Guide

> Practical maintenance procedures for the i2rt YAM Pro 6-DOF arm.
> Motor-electrical and CAN-protocol specifics are in [motors.md](motors.md) and
> [wiring-and-can.md](wiring-and-can.md). This file is the *routine* + *fault*
> workflow.

## Safety first

- **De-energize before touching connectors.** CAN and motor power share the
  harness; hot-plugging a connector can brown-out the whole chain and corrupt a
  motor's zero position.
- The arm has **no brakes** on most joints — when power is removed it will fall
  under gravity. Support the arm or move it to a rest pose before powering down.
- Keep hands clear during any `motor-test` that commands motion. Start every
  session with the **passive** (listen-only) checks before commanding torque.

## Maintenance intervals

| Interval | Task |
|---|---|
| Every session (before use) | Passive CAN scan, check all 6 joints report feedback, check temperatures at idle |
| Weekly / heavy use | Inspect CAN connectors + strain relief, check for play/backlash per joint, log idle currents |
| Monthly | Full `yam-test full` sweep, record report JSON to build history, re-check joint zero offsets |
| On fault | Targeted per-joint test, error-frame watch, thermal check under load |

## Routine checklist (start of session)

1. Visually inspect the harness — no pinched/kinked cables, connectors seated.
2. Power on. Wait for motors to energize.
3. `yam-test scan` — confirm the adapter is found.
4. `yam-test can` — passive bus health (should see 6 feedback IDs streaming).
5. `yam-test motors --no-move` — every joint enables + reports position/temp.
6. Review temperatures: idle MOS/rotor temps should be near ambient.

## Common faults → where to look

| Symptom | Likely cause | Check |
|---|---|---|
| No CAN traffic at all | Bitrate mismatch, dead adapter, no termination, motors unpowered | `yam-test can`; verify 1 Mbit/s and 120 ohm termination |
| Only error frames | Bitrate mismatch, bad wiring | `wiring-and-can.md` → termination & bitrate |
| One joint missing from scan | That motor unpowered, wrong CAN ID, broken drop cable | Per-joint isolation, inspect that motor's connector |
| Joint reports but won't move | Not enabled, in fault state, mechanical bind | Read error code from feedback frame ([motors.md](motors.md)) |
| Joint overheats quickly | Mechanical bind, excessive load, bad bearing | Thermal check; back-drive by hand for smoothness |
| Position jumps / drifts | Lost zero, encoder fault, loose coupling | Re-zero; check coupling set screws |
| Intermittent dropouts | Flaky connector, marginal termination, EMI | `error_frame_watch`; wiggle-test connectors |

## Re-zeroing a joint

The DM motors store a zero (home) offset. If a joint's reported position is
offset after a service, move it to the mechanical reference and issue the
**set-zero** command (see [motors.md](motors.md) → special commands). Record the
new offset in the arm's maintenance log.

## Keeping records

Every `yam-test` run can dump a JSON report (`--out reports/<serial>-<date>.json`).
Keep these per arm serial — trending idle temperature, error-frame counts, and
per-joint current over time is the single best early-warning of a failing motor
or connector.

## Spare parts

| Part | Used on | Notes |
|---|---|---|
| DM4340 motor | Joints 1–3 (shoulder) | Higher-torque unit |
| DM4310 motor | Joints 4–6 (elbow/wrist) + gripper | Lower-torque unit |
| CAN drop cables / connectors | Whole harness | Keep spares; connectors are the #1 intermittent-fault source |
| 120 ohm termination resistor | Bus ends | Both ends must be terminated |

> Payload / reach / weight / build materials are listed as **TBD — see i2rt.com**
> on the spec sheet. Fill these in once i2rt publishes them or you measure them.
