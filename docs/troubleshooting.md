# Troubleshooting — Symptom → Fix

Fast reference. Start at the top (bus level) and work down (motor level): a bus
fault masquerades as many motor faults.

## 1. Adapter / connection

| Symptom | Likely cause | Fix |
|---|---|---|
| `yam-test scan` finds nothing | Adapter unplugged / driver missing | Plug in; install adapter driver ([wiring-and-can.md](wiring-and-can.md#adapters)) |
| `Could not open any CAN interface` | Wrong interface/channel | Pass `--interface`/`--channel` explicitly |
| Opens but immediately errors | Bitrate not 1 Mbit/s | Force `--bitrate 1000000` (default) and confirm adapter firmware |

## 2. Bus level (`yam-test can`)

| Symptom | Likely cause | Fix |
|---|---|---|
| No frames, no errors | Bitrate mismatch, no termination, motors unpowered | Verify 1 Mbit/s; measure ~60 Ω across CAN-H/L (power off); confirm motor power |
| Only error frames | Bitrate mismatch or swapped/shorted CAN-H/L | Fix bitrate; check wiring polarity |
| Errors come and go | Flaky connector, marginal termination, EMI | Wiggle test w/ `error_frame_watch`; reseat drops; check grounds |
| High bus load | A babbling node | Isolate nodes until load drops |

## 3. Motor level (`yam-test motors`)

| Symptom | Likely cause | Fix |
|---|---|---|
| One joint absent from scan | Motor unpowered / wrong CAN ID / broken drop | Test that motor alone; check its power+CAN drop |
| Joint reports but won't enable | Motor in fault state | Read the fault code from feedback ([motors.md](motors.md#fault-codes)) |
| Enables but won't move | Mechanical bind, gains too low, e-stop | Back-drive by hand; inspect coupling; check for bind |
| Overtemp fault fast | Bind, overload, bad bearing | Remove load; back-drive to feel for roughness |
| Over/under-voltage fault | Supply sag, bad power connector | Check supply voltage under load; reseat power |
| Position drift / jump | Lost zero, encoder fault, loose coupling | Re-zero; tighten coupling set screws |
| Overcurrent fault | Bind or short | Remove load; inspect for shorted phase |

## 4. Escalation

If bus-level checks are clean but a specific motor keeps faulting under no load,
the actuator itself is suspect — swap in a known-good spare of the same type
(DM4340 for J1–J3, DM4310 for J4–J6) and re-test to confirm before ordering.

## Reading fault codes

The DM feedback frame carries a per-motor error/status field. Its meaning
(overvoltage, undervoltage, overtemp, overcurrent, etc.) is decoded by the
toolkit and documented in [motors.md](motors.md#fault-codes).

## Always keep the report

`--out reports\<serial>-<date>.json` on every run. Comparing today's idle temps,
error counts, and currents against last month's is the fastest way to catch a
degrading motor or connector before it strands the arm.
