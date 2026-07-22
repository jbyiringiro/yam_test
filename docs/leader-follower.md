# Leader vs Follower Arms

Your YAM Pro arms come in two roles for teleoperation. **The 6 arm joints (J1–J6)
are identical** on both — same DM4340 ×3 + DM4310 ×3. They differ **only at the
end (joint 7):**

| | Follower | Leader |
|---|---|---|
| End device | **Gripper** (actuated output) | **Trigger** handle (read-only input) |
| Hardware | DM4310 motor | Passive CAN encoder + 2 buttons |
| CAN id | **0x07** (feedback on 0x17) | **0x50E** (reply on 0x50F) |
| Protocol | DM MIT (same as joints) | Passive-encoder frame (`!B h h B`) |
| i2rt config | `linear_4310` | `yam_teaching_handle` (`motor_type: ""`) |
| Role | Does the physical work | Operator squeezes it to command the follower's gripper |

The operator moves the **leader**; the **follower** mirrors it, and the leader's
**trigger** maps to the follower's **gripper** (`gripper_cmd = 1 − trigger`).

> **Key point for maintenance:** the leader has **no motor at joint 7**. Don't
> expect a DM feedback frame there — its trigger is a *different device on a
> different CAN id* speaking a *different protocol*. The toolkit handles this
> automatically per profile.

## Wiring

Each arm has **its own CAN bus / gs_usb adapter** (i2rt names them
`can_leader_r`, `can_follower_r`, `can_leader_l`, `can_follower_l` on a bimanual
rig). Diagnose one arm at a time on its own bus, or give each adapter a distinct
`--channel`.

## Using the toolkit per arm

The tool auto-detects which arm is on the bus by probing the end effector — a DM
gripper answering at 0x07 means **follower**; an encoder answering at 0x50E means
**leader**:

```powershell
yam-test arm                     # just tell me which arm is connected
yam-test motors                  # auto-detects, then runs the right checks
yam-test motors --arm leader     # force leader profile (skip auto-detect)
yam-test motors --arm follower   # force follower profile
yam-test full  --arm auto --out reports\leader-r.json
```

- **Follower** run: checks J1–J6 **plus the gripper motor** (feedback, temp, fault).
- **Leader** run: checks J1–J6 **plus the trigger** (encoder position, the 0–1
  trigger value, and the two handle buttons — all read-only). Squeeze the trigger
  and press the buttons during the check to confirm they change.

### Live monitoring shows the trigger

For a leader arm, `yam-test live` streams the trigger as a live bar plus button
state, alongside the joint feedback:

```powershell
yam-test live --arm leader        # monitor: J1-J6 + live trigger bar + buttons
```

## How detection works (and its limits)

Detection is by **end effector**, because J1–J6 are electrically identical:

| Probe | Responds → |
|---|---|
| DM gripper at 0x07 | **follower** |
| Encoder at 0x50E | **leader** |
| Neither | arm off, wrong bus, or a `no_gripper` arm — defaults to follower |
| Both (shouldn't happen on one bus) | reported as ambiguous |

If you ever run **both arms on one shared bus**, auto-detect can't split them by
role reliably — give each its own bus (recommended) or select with `--arm`.

## The trigger protocol (reference)

Matches i2rt's `PassiveEncoderReader`:

- Poll: send `[0xFF, 0x02]` to `0x50E`.
- Reply on `0x50F`, 6 bytes, big-endian `!B h h B`:
  device id (u8), position (i16), velocity (i16), digital inputs (u8).
- `position_rad = raw · 2π / 4096`; same for velocity.
- Buttons: `button0 = bit0`, `button1 = bit1` of the digital byte.
- Trigger value: clamp position to ±`range_rad` (default 0.7), then
  `trigger = |position| / range_rad` (0 = released … 1 = squeezed);
  `gripper_cmd = 1 − trigger`.
- The handle encoder must be in **passive mode** (`report_freq = 0`) so it only
  answers when polled. If it streams instead, it won't reply to the poll and the
  check reports "no reply".

See [config/yam_pro.yaml](../config/yam_pro.yaml) → `end_effectors` to change ids
or the trigger range.
