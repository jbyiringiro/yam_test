"""Gripper control for the FOLLOWER arm (DM4310 linear gripper @ 0x07).

Position-controlled and **torque-limited**: it stops advancing when it reaches
the physical stop or grips an object, instead of over-driving the linear
mechanism. Constants track i2rt's linear_4310 (motor_stroke 6.57 rad, kp 20,
kd 0.5) but use gentler gains + a torque cutoff for safe bench testing.

The leader arm has a trigger, not a gripper — this only applies to followers.
"""

from __future__ import annotations

import math
import time

from .config import ArmConfig
from .dm_motor import rad_to_deg
from .motor_chain import MotorChain

# --- gripper motion envelope (conservative; tune in testing) ---------------
STROKE_RAD = 6.57        # i2rt linear_4310 motor_stroke = full open<->close travel
KP = 8.0                 # softer than i2rt's 20 for gentle bench motion
KD = 0.5
TORQUE_LIMIT = 1.2       # N·m — stop advancing above this (stop hit / gripping)
MAX_VEL_RAD = 2.0        # rad/s slew limit
LOOP_HZ = 100.0
_LT, _LR = 0.005, 3      # fast poll for the loop


def _creep_to(chain: MotorChain, g, target_rad: float, sign: float, on_step=None):
    """Slew the gripper toward target_rad, stopping if torque exceeds the limit.

    Returns (feedback, status_text).
    """
    dt = 1.0 / LOOP_HZ
    max_step = MAX_VEL_RAD * dt
    fb = chain.read(g.motor_id, g.motor_type, _LT, _LR)
    if fb is None:
        return None, "no reply from gripper"
    cmd = fb.position
    end = time.time() + 6.0
    while time.time() < end:
        if fb is not None and abs(fb.torque) > TORQUE_LIMIT:
            return fb, f"stopped at torque limit ({fb.torque:+.2f} N·m) — stop or grip"
        delta = target_rad - cmd
        if abs(delta) <= max_step:
            cmd = target_rad
        else:
            cmd += math.copysign(max_step, delta)
        fb = chain.command(g.motor_id, g.motor_type, position=cmd,
                           kp=KP, kd=KD, timeout=_LT, retries=_LR)
        if fb is None:
            return None, "lost gripper reply mid-move"
        if on_step:
            on_step(fb, cmd)
        if abs(cmd - target_rad) < 1e-4:
            return fb, "reached commanded position"
        time.sleep(dt)
    return fb, "timeout"


def run_gripper(chain: MotorChain, cfg: ArmConfig, action: str = "interactive",
                invert: bool = False) -> None:
    """Operate the follower gripper.

    action: "open" | "close" | "interactive".
    invert: flip which direction counts as open (in case yours is reversed).
    """
    g = cfg.gripper
    if g is None:
        print("This arm has no gripper motor (leader arms have a trigger instead).")
        print("Connect a FOLLOWER arm to use the gripper.")
        return

    sign = -1.0 if invert else 1.0

    print("Enabling gripper motor...")
    fb = chain.enable(g.motor_id, g.motor_type)
    if fb is None:
        print(f"No reply from gripper (id 0x{g.motor_id:02X}). Is a follower connected/powered?")
        return
    if not fb.healthy and fb.error_code != 0:
        fb = chain.recover_joint(g) or fb
    start = fb.position
    print(f"Gripper enabled. start pos = {rad_to_deg(start):.1f}°, "
          f"torque limit = {TORQUE_LIMIT} N·m. (Ctrl+C or 'q' to stop.)")

    try:
        if action in ("open", "close"):
            direction = 1.0 if action == "open" else -1.0
            target = start + direction * sign * STROKE_RAD
            print(f"{action.capitalize()}ing...")
            fb, status = _creep_to(chain, g, target, sign)
            if fb is not None:
                print(f"  {status}. pos={rad_to_deg(fb.position):.1f}°  torque={fb.torque:+.2f} N·m")
            else:
                print(f"  {status}")
        else:
            _interactive(chain, cfg, g, sign)
    finally:
        chain.disable(g.motor_id, g.motor_type)
        print("Gripper disabled.")


def _interactive(chain: MotorChain, cfg: ArmConfig, g, sign: float) -> None:
    from .live import _read_key, _HAVE_KEYS

    if not _HAVE_KEYS:
        print("Interactive control needs Windows (msvcrt). Use --open / --close instead.")
        return

    print("\nKeys:  [o] open   [c] close   [+/-] nudge   [space] stop/hold   [q] quit")
    print("(If the terminal doesn't respond to keys, run in a standalone Windows "
          "Terminal, not VSCode's integrated terminal.)\n")

    fb = chain.read(g.motor_id, g.motor_type, _LT, _LR)
    target = fb.position if fb else 0.0
    nudge = math.radians(10)  # ~10° of motor travel per +/- press
    dt = 1.0 / LOOP_HZ
    max_step = MAX_VEL_RAD * dt

    try:
        from rich.live import Live
        from rich.console import Console
        console = Console()
        cmd = target
        with Live(console=console, refresh_per_second=20) as live:
            while True:
                key = _read_key()
                if key in ("q", "ESC"):
                    break
                elif key == "o":
                    target = fb.position + sign * STROKE_RAD
                elif key == "c":
                    target = fb.position - sign * STROKE_RAD
                elif key in ("+", "=", "UP"):
                    target = target + sign * nudge
                elif key in ("-", "_", "DOWN"):
                    target = target - sign * nudge
                elif key == " ":
                    target = fb.position if fb else target  # hold here

                # slew + torque cutoff
                if fb is not None and abs(fb.torque) > TORQUE_LIMIT:
                    target = fb.position  # freeze — stop / gripping
                    grip = "  [bold yellow]GRIP/STOP[/bold yellow]"
                else:
                    grip = ""
                delta = target - cmd
                if abs(delta) > max_step:
                    delta = math.copysign(max_step, delta)
                cmd += delta
                fb = chain.command(g.motor_id, g.motor_type, position=cmd,
                                   kp=KP, kd=KD, timeout=_LT, retries=_LR) or fb

                if fb is not None:
                    live.update(
                        f"gripper  pos=[bold]{rad_to_deg(fb.position):7.1f}°[/bold]  "
                        f"target={rad_to_deg(target):7.1f}°  "
                        f"torque={fb.torque:+.2f} N·m  "
                        f"temp={fb.temp_mos}/{fb.temp_rotor}°C{grip}\n"
                        "[dim][o]pen [c]lose [+/-] nudge [space] hold [q]uit[/dim]"
                    )
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
