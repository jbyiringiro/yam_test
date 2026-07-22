# YAM Pro Arm — Test & CAN Diagnostics Toolkit

Standalone maintenance tooling for the **i2rt YAM Pro** 6-DOF robot arm.

It talks the **DM4310 / DM4340** motor protocol **directly over CAN**, so it can
diagnose faults *even when the full i2rt software stack won't come up*. Use it to:

- **Test the CAN bus itself** — bitrate/termination/wiring health, error frames, bus load.
- **Test each arm joint** — enable, read position/velocity/torque/temperature, decode fault codes, check range of motion.
- **Keep maintenance records** — every run can dump a JSON report per arm serial.

> This is a *diagnostic* tool for maintenance, not a control/teleop stack. It is
> intentionally independent of `i2rt.robots` so it stays useful when that's broken.

## The arm at a glance

| | |
|---|---|
| DOF | 6 |
| Communication | CAN bus @ **1 Mbit/s** |
| Shoulder motors (J1–J3) | 3× **DM4340** |
| Elbow/wrist motors (J4–J6) | 3× **DM4310** |
| Gripper | LINEAR_4310 |

Joint ranges, gains, and motor constants live in [config/yam_pro.yaml](config/yam_pro.yaml).
Deep-dive docs are in [docs/](docs/).

## Get started (for the team)

Clone, install, do the one-time adapter driver step, and run:

```powershell
# 1. clone
git clone https://github.com/jbyiringiro/yam_test.git
cd yam_test

# 2. install (needs Python 3.10+). If `python` isn't found, use `py` or your
#    full python path, e.g. "C:\ProgramData\Anaconda3\python.exe"
python -m pip install -r requirements.txt
python -m pip install -e .

# 3. one-time: bind the gs_usb / CANable adapter to WinUSB with Zadig
#    -> see docs/setup-windows.md

# 4. run (from the project folder)
.\yam.bat checkup           # guided end-to-end flow (recommended)
```

A virtual environment is optional (for isolation): `python -m venv .venv ;
.venv\Scripts\Activate.ps1` before the pip installs.

We use a **gs_usb** adapter (CANable / candleLight). `requirements.txt` installs
its backend; on Windows you also bind it to WinUSB **once** with Zadig — see
[docs/setup-windows.md](docs/setup-windows.md). After that the tool defaults to
`gs_usb` channel `0`, so no flags are needed.

### Running it in the terminal

Use the bundled launcher from the project folder — it finds Python for you and
works even if `python` isn't on your PATH:

```powershell
.\yam.bat checkup        # guided end-to-end flow (recommended)
.\yam.bat arm            # any command works the same way
```

`yam.bat` works in any shell (cmd or PowerShell). `.\yam.ps1 ...` also works in
PowerShell if script execution is enabled (`Set-ExecutionPolicy -Scope
CurrentUser RemoteSigned`). Long form: `python -m arm_test.cli checkup`.

## Quick start

```powershell
yam-test scan                       # confirm the gs_usb adapter is found
yam-test selftest                   # test the adapter ALONE (no arm needed)
yam-test arm                        # is a leader or follower arm connected?
yam-test motors --no-move           # each joint: feedback + temp, no motion
yam-test full --out reports\arm.json   # everything, saved to a report
```

> **Note:** DM motors are *request/response* — they only reply when polled, they
> don't broadcast. A purely passive listen (`yam-test can`) shows no traffic
> unless something is actively driving the bus; that's expected, not a fault.
> Use `arm`/`motors` to actively probe an arm, and `selftest` for the adapter alone.

Defaults (`gs_usb`, channel `0`, 1 Mbit/s) come from
[config/yam_pro.yaml](config/yam_pro.yaml). Override for a different adapter:

```powershell
yam-test can --interface gs_usb --channel 0
yam-test can --interface slcan  --channel COM5   # if using slcan firmware instead
```

## What each command does

| Command | Motion? | What it checks |
|---|---|---|
| `checkup` | guided | **One guided flow:** CAN link → detect arm → scan motors → activate (follower jog) or trigger+buttons (leader) |
| `scan` | no | Lists CAN adapters/candidates found on the machine |
| `selftest` | no | **Adapter loopback test — no arm/wiring/termination needed.** Proves the CANable's USB+driver+TX/RX all work |
| `can` | no | Passive traffic scan, error frames, bus load |
| `arm` | no | Detects whether a **leader** or **follower** arm is connected (active probe) |
| `gripper` | yes | **Follower only.** Open/close/operate the gripper, torque-limited so it stops at the stop or when it grips |
| `motors` | optional | Per-joint enable, feedback decode, temperature, fault codes (+ gripper or trigger) |
| `full` | optional | CAN health + all joints + summary report |
| `live` | mode-dependent | Live streaming: `monitor` (read-only), `jog` (keyboard control), `exercise` (auto oscillation) |

`--no-move` keeps everything **read-only / hold-at-zero**. Motion tests are opt-in.

## Leader vs follower arms

The 6 joints (J1–J6) are identical on both arms; they differ only at joint 7 —
the **follower** has an actuated **gripper** (DM4310 @ 0x07), the **leader** has a
read-only **trigger** encoder (@ 0x50E). The tool auto-detects which is connected,
or you can force it:

```powershell
yam-test arm                     # which arm is on this bus?
yam-test motors                  # auto-detects, checks the right end effector
yam-test motors --arm leader     # force the leader profile
yam-test live   --arm leader     # monitor J1-J6 + live trigger bar + buttons
```

Full detail in [docs/leader-follower.md](docs/leader-follower.md).

## Live streaming (`yam-test live`)

A continuous loop that streams every joint's position/velocity/torque/temperature/fault
in a live table. Three modes:

```powershell
yam-test live                       # monitor: read-only, streams feedback, NO motion
yam-test live --mode jog            # interactively move joints with the keyboard
yam-test live --mode exercise       # hands-off gentle oscillation (break-in / smoothness)
yam-test live --mode jog --joints J2,J3   # limit to specific joints
yam-test live --mode exercise --amp 8 --period 3   # tune the oscillation
```

## Gripper (`yam-test gripper`)

The gripper is on the **follower** arm (DM4310 @ 0x07). The leader has a trigger
instead, so `gripper` only works on a follower.

```powershell
.\yam.bat gripper --open      # open and stop
.\yam.bat gripper --close     # close and stop
.\yam.bat gripper             # interactive: [o]pen [c]lose [+/-] nudge [space] hold [q]uit
.\yam.bat gripper --invert    # if open/close come out reversed on your unit
```

It's **torque-limited** (default 1.2 N·m): it stops advancing when it hits the
physical stop or grips an object, rather than over-driving the linear mechanism.
The live view shows position, torque, and a **GRIP/STOP** flag when the limit
trips. In teleoperation the leader's *trigger* drives this same gripper
(`gripper_cmd = 1 − trigger`); `gripper` lets you operate/test it directly.

**Jog keys:** `↑/↓` (or `+`/`-`) move the active joint · `←/→` select joint ·
`,`/`.` change step size · `h` hold at current position · `SPACE` **E-STOP** ·
`e` re-enable · `q` quit.

**Session report / crash log.** Live now records what each joint was doing and
its **peak torque + temperature**. If the arm **faults** or **goes silent
mid-move** (e.g. a joint spikes current and the power supply trips), it
auto-saves a report to `reports/live-fault-<arm>-<mode>.json` capturing the
last-known state and which joint was active — so you can see *what happened*
even though a dead bus can't be read live. Add `--out reports\my.json` to always
save one:

```powershell
.\yam.bat live --mode jog --out reports\jog-session.json
```

The report shows, per joint: last position/velocity/**torque**/temperature/fault,
plus session **peak torque** and **peak temp** — the evidence for a bind or
overload on a specific joint.

**Safety envelope** (all configurable in [config/yam_pro.yaml](config/yam_pro.yaml)):
targets start at the measured position (no jump on enable) · a **slew-rate limiter**
caps commanded speed (default 20°/s) regardless of gains · a **torque limit**
(default 4 N·m) freezes a joint's command when it strains — so a lagging joint
can't build current until the power supply trips · targets are soft-clamped inside
each joint's range · hold stiffness is a fraction of the reference gains · **any
motor fault auto-triggers E-STOP** · motors are **always disabled on exit** (quit,
Ctrl+C, or error). Start with `monitor`, then `jog`.

## Repository layout

```
Arm_Test/
├── README.md
├── requirements.txt / pyproject.toml
├── config/yam_pro.yaml         # joints, motor types, gains, limits, CAN IDs
├── src/arm_test/
│   ├── can_backend.py          # adapter-agnostic CAN access + detection
│   ├── dm_motor.py             # DM4310/4340 protocol (frame pack/unpack, faults)
│   ├── encoder.py              # leader trigger-handle passive-encoder protocol
│   ├── motor_chain.py          # multi-motor manager over one bus
│   ├── detect.py               # leader/follower auto-detection
│   ├── live.py                 # live monitor / jog / exercise loop
│   ├── cli.py                  # `yam-test` entry point
│   └── diagnostics/
│       ├── can_health.py       # bus-level checks
│       ├── motor_test.py       # per-joint checks
│       └── report.py           # result model + JSON/table output
├── tests/                      # offline unit tests (protocol packing)
├── docs/                       # motors, wiring/CAN, maintenance, troubleshooting
└── reports/                    # saved diagnostic JSON (history per arm)
```

## Documentation

- [docs/setup-windows.md](docs/setup-windows.md) — gs_usb / CANable Windows setup (Zadig WinUSB)
- [docs/overview.md](docs/overview.md) — the arm, its parts, how it's addressed
- [docs/leader-follower.md](docs/leader-follower.md) — leader vs follower, trigger vs gripper
- [docs/motors.md](docs/motors.md) — DM4310/4340 protocol, feedback, fault codes
- [docs/wiring-and-can.md](docs/wiring-and-can.md) — harness, termination, adapters, bitrate
- [docs/maintenance.md](docs/maintenance.md) — routine + fault workflows
- [docs/troubleshooting.md](docs/troubleshooting.md) — symptom → fix table

## Safety

The arm has no joint brakes — **it falls under gravity when de-energized.** Support
it before power-down. Always run the **passive** checks before any command that
produces torque. Motion tests move real hardware — keep clear.
