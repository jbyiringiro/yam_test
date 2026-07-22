# YAM Pro — Overview

The **i2rt YAM Pro** is a 6-DOF robot arm driven by Damiao (DM) quasi-direct-drive
actuators over a single 1 Mbit/s CAN bus. This page is the map; follow the links
for detail.

## Specifications (from the i2rt spec sheet)

| Parameter | Value |
|---|---|
| Price | $3,499 |
| Degrees of freedom | 6 |
| Communication | CAN bus (1 Mbit/s) |
| Shoulder motors | 3× DM4340 |
| Elbow / wrist motors | 3× DM4310 |
| Joint 1 range | −150° to +180° |
| Joint 2 range | 0° to +210° |
| Joint 3 range | 0° to +180° |
| Joint 4 range | −97° to +90° |
| Joint 5 range | −90° to +90° |
| Joint 6 range | −120° to +120° |
| Payload / Reach / Weight / Build | **TBD — see i2rt.com** |

## Joint map

| Joint | Motor | Role | Range |
|---|---|---|---|
| J1 | DM4340 | Base rotation | −150° … +180° |
| J2 | DM4340 | Shoulder | 0° … +210° |
| J3 | DM4340 | Shoulder/upper arm | 0° … +180° |
| J4 | DM4310 | Elbow/forearm | −97° … +90° |
| J5 | DM4310 | Wrist | −90° … +90° |
| J6 | DM4310 | Wrist/tool roll | −120° … +120° |
| J7 (follower) | DM4310 (LINEAR_4310) | Linear **gripper** @ 0x07 | — |
| J7 (leader) | passive encoder | **Trigger** handle @ 0x50E (read-only) | — |

The three DM4340 units carry the higher shoulder loads; the three DM4310 units
drive the lighter elbow/wrist joints. Because both are the same family of DM
actuator, the PD-control tuning carries over between YAM Standard and YAM Pro.

**Leader vs follower:** J1–J6 are identical on both arms. They differ only at
joint 7 — the follower has an actuated **gripper** (DM4310), the leader has a
read-only **trigger** encoder. See [leader-follower.md](leader-follower.md).

## Default control gains (reference)

From the i2rt YAM Pro control-gains sheet (identical to YAM Standard):

```yaml
kp:                 [80.0, 80.0, 80.0, 40.0, 10.0, 10.0]
kd:                 [5.0,  5.0,  5.0,  1.5,  1.5,  1.5]
gravity_comp_factor:[1.0,  1.1,  1.1,  1.2,  1.0,  1.0]
grav_comp_kd:       [0.1,  0.1,  0.1,  0.3,  0.05, 0.05]
coulomb_friction:   [0.3,  0.3,  0.3,  0.06, 0.06, 0.06]
```

These are control/tuning values, not diagnostic thresholds — the test tool holds
motors at a **safe low-gain** during motion checks rather than using these.

## How it's driven (software)

The official i2rt path:

```python
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

robot = get_yam_robot(
    channel="can0",
    arm_type=ArmType.YAM_PRO,
    gripper_type=GripperType.LINEAR_4310,
)
```

This toolkit deliberately **bypasses** that stack and speaks CAN directly, so it
still works when i2rt won't import, a motor is in a fault state, or the bus is
misbehaving. See [motors.md](motors.md) for the wire-level protocol.

## Where to go next

- Wire-level motor protocol & fault codes → [motors.md](motors.md)
- CAN harness, adapters, termination → [wiring-and-can.md](wiring-and-can.md)
- Maintenance routines & fault workflow → [maintenance.md](maintenance.md)
- Symptom → fix quick reference → [troubleshooting.md](troubleshooting.md)
