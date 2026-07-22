"""`yam-test` command-line entry point.

Subcommands:
  scan    - list CAN adapters found on this machine (no bus opened)
  can     - passive CAN-bus health checks (safe, no motion, no transmit)
  motors  - per-joint checks; read-only by default, --move to command motion
  full    - can + motors, saved to a JSON report

Design goal: a maintenance tech can run `yam-test full` and get a clear
PASS/WARN/FAIL table without knowing any of the protocol internals.
"""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime, timezone

from . import __version__
from .can_backend import DEFAULT_BITRATE, detect_backends, open_bus
from .diagnostics import can_health
from .diagnostics.report import Report, Status


def _iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _resolve_iface(args) -> tuple:
    """Interface/channel from CLI flags, falling back to config defaults (gs_usb).

    Explicit --interface/--channel always win. Otherwise use the config's
    can.interface/can.channel so `yam-test can` "just works" for our gs_usb rig.
    Returns (interface, channel) — either may be None to trigger auto-detect.
    """
    iface, chan = args.interface, args.channel
    if iface is None:
        try:
            from .config import load_config
            cfg = load_config(getattr(args, "config", None))
            iface = iface or cfg.interface
            chan = chan or cfg.channel
        except Exception:
            pass
    return iface, chan


def _add_bus_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--interface", help="python-can interface (slcan, pcan, kvaser, gs_usb, ...)")
    p.add_argument("--channel", help="channel / COM port (e.g. COM5, PCAN_USBBUS1, 0)")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE, help="CAN bitrate (default 1 Mbit/s)")
    p.add_argument("--config", default=None, help="path to yam_pro.yaml (defaults to bundled config)")
    p.add_argument("--out", default=None, help="write JSON report to this path")
    p.add_argument("--serial", default=None, help="arm serial number, recorded in the report")


def _add_arm_arg(p: argparse.ArgumentParser) -> None:
    p.add_argument("--arm", choices=["auto", "follower", "leader"], default="auto",
                   help="which arm is connected: follower (gripper), leader (trigger), "
                        "or auto-detect (default)")


def _resolve_arm(chain, cfg, args) -> str:
    """Select follower/leader on cfg, auto-detecting from the bus when asked."""
    choice = getattr(args, "arm", "auto")
    if choice != "auto":
        cfg.set_arm(choice)
        print(f"Arm: {choice} (explicit)")
        return choice
    from .detect import detect_arm
    det = detect_arm(chain, cfg)
    if det.arm:
        cfg.set_arm(det.arm)
        print(f"Arm auto-detect: {det.text}")
        return det.arm
    cfg.set_arm("follower")
    print(f"Arm auto-detect: {det.text}. Defaulting to follower.")
    return "follower"


# ---------------------------------------------------------------------------
# scan
# ---------------------------------------------------------------------------
def cmd_scan(args) -> int:
    cands = detect_backends()
    if not cands:
        print("No CAN adapters detected.")
        print("Install your adapter's driver, or specify --interface/--channel.")
        return 1
    print("Detected CAN interface candidates:")
    for c in cands:
        print(f"  --interface {c.interface:8}  --channel {c.channel:12}  {c.note}")
    print("\nUse the one matching your adapter, e.g.:")
    print(f"  yam-test can --interface {cands[0].interface} --channel {cands[0].channel}")
    return 0


# ---------------------------------------------------------------------------
# can
# ---------------------------------------------------------------------------
def cmd_selftest(args) -> int:
    from .selftest import gs_usb_loopback_selftest
    from .diagnostics.report import Report

    report = Report(title="CAN adapter self-test (loopback)", timestamp=_iso_now())
    idx = 0
    if args.channel is not None:
        try:
            idx = int(args.channel)
        except ValueError:
            idx = 0
    report.add(gs_usb_loopback_selftest(index=idx, bitrate=args.bitrate))
    report.print()
    if args.out:
        report.to_json(args.out)
        print(f"Report written to {args.out}")
    return 0 if report.worst != Status.FAIL else 1


def cmd_can(args) -> int:
    report = Report(title="CAN bus health", arm_serial=args.serial, timestamp=_iso_now())
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        report.add(can_health.passive_traffic_scan(bus, duration=2.0))
        report.add(can_health.error_frame_watch(bus, duration=3.0))
        report.add(can_health.bus_load_estimate(bus, duration=1.0, bitrate=args.bitrate))
    finally:
        bus.shutdown()

    report.print()
    if args.out:
        report.to_json(args.out)
        print(f"Report written to {args.out}")
    return 0 if report.worst != Status.FAIL else 1


# ---------------------------------------------------------------------------
# motors  (implemented against motor_test once protocol module lands)
# ---------------------------------------------------------------------------
def cmd_motors(args) -> int:
    from .config import load_config
    from .motor_chain import MotorChain
    from .diagnostics import motor_test

    cfg = load_config(args.config)
    report = Report(title="YAM Pro joints", arm_serial=args.serial, timestamp=_iso_now())
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        chain = MotorChain(bus, cfg)
        _resolve_arm(chain, cfg, args)
        report.title = f"YAM Pro joints ({cfg.arm})"
        motor_test.check_all_joints(chain, cfg, report, allow_move=args.move,
                                     move_deg=args.move_deg)
    finally:
        bus.shutdown()

    report.print()
    if args.out:
        report.to_json(args.out)
        print(f"Report written to {args.out}")
    return 0 if report.worst != Status.FAIL else 1


# ---------------------------------------------------------------------------
# full
# ---------------------------------------------------------------------------
def cmd_full(args) -> int:
    from .config import load_config
    from .motor_chain import MotorChain
    from .diagnostics import motor_test

    cfg = load_config(args.config)
    report = Report(title="YAM Pro full diagnostics", arm_serial=args.serial, timestamp=_iso_now())
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        report.add(can_health.passive_traffic_scan(bus, duration=2.0))
        report.add(can_health.error_frame_watch(bus, duration=3.0))
        report.add(can_health.bus_load_estimate(bus, duration=1.0, bitrate=args.bitrate))
        chain = MotorChain(bus, cfg)
        _resolve_arm(chain, cfg, args)
        report.title = f"YAM Pro full diagnostics ({cfg.arm})"
        motor_test.check_all_joints(chain, cfg, report, allow_move=args.move,
                                    move_deg=args.move_deg)
    finally:
        bus.shutdown()

    report.print()
    if args.out:
        report.to_json(args.out)
        print(f"Report written to {args.out}")
    return 0 if report.worst != Status.FAIL else 1


# ---------------------------------------------------------------------------
# live  (monitor / jog / exercise)
# ---------------------------------------------------------------------------
def cmd_live(args) -> int:
    from .config import load_config
    from .motor_chain import MotorChain
    from . import live as live_mod

    cfg = load_config(args.config)
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        chain = MotorChain(bus, cfg)
        _resolve_arm(chain, cfg, args)
        joints_filter = [j.strip() for j in args.joints.split(",")] if args.joints else None
        live_mod.run_live(
            chain, cfg,
            mode=args.mode,
            include_gripper=args.include_gripper,
            joints_filter=joints_filter,
            amp_deg=args.amp,
            period_s=args.period,
            out=args.out,
            torque_limit=args.torque_limit,
        )
    finally:
        bus.shutdown()
    return 0


# ---------------------------------------------------------------------------
# gripper  (follower gripper control)
# ---------------------------------------------------------------------------
def cmd_gripper(args) -> int:
    from .config import load_config
    from .motor_chain import MotorChain
    from . import gripper as gripper_mod

    cfg = load_config(args.config)
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        chain = MotorChain(bus, cfg)
        # gripper only exists on the follower
        _resolve_arm(chain, cfg, args)
        if cfg.arm != "follower":
            print("The connected arm is a LEADER — it has a trigger, not a gripper.")
            print("Connect a follower arm to use the gripper.")
            return 1
        action = "open" if args.open else ("close" if args.close else "interactive")
        gripper_mod.run_gripper(chain, cfg, action=action, invert=args.invert)
    finally:
        bus.shutdown()
    return 0


# ---------------------------------------------------------------------------
# arm  (just detect which arm is connected)
# ---------------------------------------------------------------------------
def cmd_arm(args) -> int:
    from .config import load_config
    from .motor_chain import MotorChain
    from .detect import detect_arm

    cfg = load_config(args.config)
    iface, chan = _resolve_iface(args)
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    try:
        det = detect_arm(MotorChain(bus, cfg), cfg)
    finally:
        bus.shutdown()
    print(f"Detection: {det.text}")
    print(f"  gripper (0x{cfg.follower_gripper.motor_id:02X}) responded: {det.gripper_responded}"
          if cfg.follower_gripper else "  no follower gripper configured")
    print(f"  trigger (0x{cfg.leader_trigger.encoder_id:X}) responded: {det.trigger_responded}"
          if cfg.leader_trigger else "  no leader trigger configured")
    return 0 if det.arm else 1


# ---------------------------------------------------------------------------
# checkup  (guided end-to-end flow)
# ---------------------------------------------------------------------------
def _watch_buttons(chain, trig, seconds: float):
    """Poll the trigger encoder for `seconds`, return the set of (b0,b1) seen."""
    import time
    seen = set()
    end = time.time() + seconds
    while time.time() < end:
        rd = chain.read_encoder(trig.encoder_id, trig.range_rad, 0.01, 3)
        if rd is not None:
            seen.add(rd.buttons)
    return seen


def _guided_button_check(chain, cfg) -> None:
    """Step 5: check the leader's two buttons, one at a time."""
    import time
    trig = cfg.trigger
    print("\n--- Step 5: trigger + buttons (leader) ---")

    # live trigger reading
    print("Squeeze the trigger fully, then release (watching 4s)...")
    tmin, tmax = 1e9, -1e9
    end = time.time() + 4
    while time.time() < end:
        rd = chain.read_encoder(trig.encoder_id, trig.range_rad, 0.01, 3)
        if rd is not None:
            tmin = min(tmin, rd.trigger); tmax = max(tmax, rd.trigger)
    if tmax >= 0:
        print(f"  trigger range seen: {tmin:.2f} .. {tmax:.2f}  "
              f"({'OK — tracks squeeze' if (tmax - tmin) > 0.3 else 'little movement — squeeze harder / recheck'})")
    else:
        print("  no trigger reply — check the handle connection.")

    # buttons, one at a time
    input("\nRelease everything, then press Enter to start the button check...")
    results = {}
    for label in ("Button 1", "Button 2"):
        for c in (3, 2, 1):
            print(f"  Press and HOLD {label} in {c}...  ", end="\r")
            time.sleep(1)
        print(f"  Hold {label} now — watching 5s...        ")
        seen = _watch_buttons(chain, trig, 5.0)
        bits = sorted({i for (b0, b1) in seen for i, b in enumerate((b0, b1)) if b})
        if bits:
            print(f"  -> {label} DETECTED on bit(s) {bits}")
        else:
            print(f"  -> {label}: no change seen")
        results[label] = bits

    if not any(results.values()):
        print("\nNo button signals detected. This handle likely has no buttons wired "
              "(the encoder's digital byte stays 0).")
    else:
        print(f"\nButton mapping: {results}")


def cmd_checkup(args) -> int:
    """Guided end-to-end maintenance flow (steps 1-5)."""
    from .config import load_config
    from .motor_chain import MotorChain
    from .detect import detect_arm
    from .diagnostics import motor_test
    from . import live as live_mod

    cfg = load_config(args.config)
    iface, chan = _resolve_iface(args)

    # -- Step 1: CAN link --------------------------------------------------
    print("=== Step 1: CAN link ===")
    try:
        bus = open_bus(iface, chan, args.bitrate)
    except Exception as exc:
        print(exc)
        return 2
    print(f"CAN adapter OK ({iface or 'auto'} @ {args.bitrate} bps).")

    try:
        chain = MotorChain(bus, cfg)

        # -- Step 2: which arm --------------------------------------------
        print("\n=== Step 2: detect arm ===")
        det = detect_arm(chain, cfg)
        print(det.text)
        if not det.arm:
            print("No arm reachable over CAN. Check arm power and the CAN cable "
                  "(CAN-H/L/GND) to the adapter, then retry.")
            return 1
        cfg.set_arm(det.arm)

        # -- Step 3: scan motors ------------------------------------------
        print(f"\n=== Step 3: scan motors ({cfg.arm}) ===")
        report = Report(title=f"YAM Pro joints ({cfg.arm})", timestamp=_iso_now())
        motor_test.check_all_joints(chain, cfg, report, allow_move=False)
        report.print()
        n_expected = len(cfg.all_motors())
        n_ok = sum(1 for r in report.results if r.data.get("responded"))
        if n_ok < n_expected:
            print(f"\nOnly {n_ok}/{n_expected} motors responded. Fix the missing "
                  "joint(s) before activating. See docs/troubleshooting.md.")
            if input("Continue anyway? [y/N] ").strip().lower() != "y":
                return 1

        # -- Step 4/5: branch by arm --------------------------------------
        if cfg.arm == "leader":
            _guided_button_check(chain, cfg)
            print("\n=== Optional: live monitor (backdrive joints by hand) ===")
            if input("Open live monitor? [y/N] ").strip().lower() == "y":
                live_mod.run_live(chain, cfg, mode="monitor")
        else:
            print("\n=== Step 4: activate motors + live movement ===")
            print("!! The motors will be ENABLED and the arm WILL move.")
            print("!! The arm has no brakes — support it and keep clear.")
            if input("Activate and start live JOG control? [y/N] ").strip().lower() == "y":
                live_mod.run_live(chain, cfg, mode="jog")
            else:
                print("Skipped motion. (Run `yam-test live --mode jog` when ready.)")
    finally:
        bus.shutdown()

    print("\nCheckup complete.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="yam-test", description="YAM Pro arm & CAN diagnostics")
    p.add_argument("--version", action="version", version=f"yam-test {__version__}")
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("scan", help="list CAN adapters on this machine")
    s.set_defaults(func=cmd_scan)

    ck = sub.add_parser("checkup", help="guided end-to-end flow: CAN -> arm -> motors -> activate/buttons")
    _add_bus_args(ck)
    ck.set_defaults(func=cmd_checkup)

    st = sub.add_parser("selftest", help="adapter loopback self-test (no arm/wiring needed)")
    _add_bus_args(st)
    st.set_defaults(func=cmd_selftest)

    c = sub.add_parser("can", help="passive CAN-bus health (safe)")
    _add_bus_args(c)
    c.set_defaults(func=cmd_can)

    a = sub.add_parser("arm", help="detect whether a leader or follower arm is connected")
    _add_bus_args(a)
    a.set_defaults(func=cmd_arm)

    gr = sub.add_parser("gripper", help="open/close/operate the follower gripper (torque-limited)")
    _add_bus_args(gr)
    _add_arm_arg(gr)
    gr.add_argument("--open", action="store_true", help="open the gripper and exit")
    gr.add_argument("--close", action="store_true", help="close the gripper and exit")
    gr.add_argument("--invert", action="store_true", help="flip open/close direction if reversed")
    gr.set_defaults(func=cmd_gripper)

    m = sub.add_parser("motors", help="per-joint checks")
    _add_bus_args(m)
    _add_arm_arg(m)
    m.add_argument("--move", action="store_true", help="allow small commanded motion (default read-only)")
    m.add_argument("--no-move", dest="move", action="store_false", help="read-only (default)")
    m.add_argument("--move-deg", type=float, default=5.0, help="max motion for range test (deg)")
    m.set_defaults(func=cmd_motors, move=False)

    f = sub.add_parser("full", help="CAN + all joints, saved to a report")
    _add_bus_args(f)
    _add_arm_arg(f)
    f.add_argument("--move", action="store_true", help="allow small commanded motion")
    f.add_argument("--no-move", dest="move", action="store_false")
    f.add_argument("--move-deg", type=float, default=5.0)
    f.set_defaults(func=cmd_full, move=False)

    lv = sub.add_parser("live", help="live streaming: monitor / jog / exercise")
    _add_bus_args(lv)
    _add_arm_arg(lv)
    lv.add_argument("--mode", choices=["monitor", "jog", "exercise"], default="monitor",
                    help="monitor=read-only (safe), jog=keyboard control, exercise=auto oscillation")
    lv.add_argument("--joints", default=None,
                    help="comma-separated joint names to include (e.g. J2,J3). Default: all")
    lv.add_argument("--include-gripper", action="store_true", help="include the gripper")
    lv.add_argument("--amp", type=float, default=None, help="exercise amplitude (deg)")
    lv.add_argument("--period", type=float, default=None, help="exercise period (s)")
    lv.add_argument("--torque-limit", type=float, default=None,
                    help="max torque (N·m) before a joint's command freezes; raise so "
                         "shoulders can move against gravity (watch for supply trips)")
    lv.set_defaults(func=cmd_live)

    return p


def main(argv=None) -> int:
    # Windows consoles often default to cp1252; force UTF-8 so table borders and
    # dashes render instead of mojibake. Harmless if already UTF-8.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass
    # python-can logs a warning for every backend driver it can't find while
    # probing. That's expected noise on a machine with one adapter — hide it.
    logging.getLogger("can").setLevel(logging.ERROR)
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
