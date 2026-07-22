"""Live streaming modes for the YAM Pro arm.

Three modes, sharing one continuous control loop (the loop is what keeps the DM
motors alive past their 400 ms command-timeout):

  monitor   read-only. Streams every joint's position/vel/torque/temp/fault.
            Zero gains, zero torque -> NO motion. Always safe.
  jog       interactive. Move joints with the keyboard while streaming feedback.
  exercise  hands-off. Gently oscillates joints (sine) for break-in / smoothness
            checks while streaming feedback.

Safety envelope for the moving modes (from config thresholds):
  * targets start at the *measured* position on enable -> no jump.
  * a slew-rate limiter caps commanded speed (live_max_vel_deg) independent of
    gains, so motion is gentle even if a gain is high.
  * targets are soft-clamped inside each joint's range (with margin).
  * hold stiffness = live_kp_scale x reference kp (a softer spring than control).
  * any motor fault -> auto E-STOP (all motors disabled).
  * motors are always disabled on exit (quit, Ctrl+C, or exception).

Keyboard is Windows-native (msvcrt); no extra dependency.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from .config import ArmConfig, JointCfg
from .dm_motor import Feedback, deg_to_rad, rad_to_deg
from .motor_chain import MotorChain

# Fast poll for the live loop so a missing motor never stalls a whole cycle.
_LOOP_TIMEOUT = 0.003
_LOOP_RETRIES = 2


# ---------------------------------------------------------------------------
# Keyboard (Windows)
# ---------------------------------------------------------------------------
try:
    import msvcrt  # Windows only

    _HAVE_KEYS = True
except ImportError:  # pragma: no cover
    _HAVE_KEYS = False


def _read_key() -> Optional[str]:
    """Non-blocking key read. Returns a normalized token or None.

    Tokens: single chars, or 'UP'/'DOWN'/'LEFT'/'RIGHT' for arrows, 'ESC'.
    """
    if not _HAVE_KEYS or not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):  # special key prefix -> arrow/function
        ch2 = msvcrt.getch()
        return {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}.get(ch2, "")
    if ch == b"\x1b":
        return "ESC"
    try:
        return ch.decode("ascii", "ignore")
    except Exception:
        return ""


# ---------------------------------------------------------------------------
# Per-joint live state
# ---------------------------------------------------------------------------
@dataclass
class JointLive:
    joint: JointCfg
    index: int
    kp: float
    kd: float
    desired_deg: float = 0.0     # where the operator/exercise wants it
    command_deg: float = 0.0     # slewed target actually sent
    center_deg: float = 0.0      # exercise oscillation center
    fb: Optional[Feedback] = None
    present: bool = False

    @property
    def lo(self) -> float:
        return self.joint.range_deg[0] + 2.0  # 2 deg margin off the hard stop

    @property
    def hi(self) -> float:
        return self.joint.range_deg[1] - 2.0

    def clamp(self, deg: float) -> float:
        return max(self.lo, min(self.hi, deg))


def _joint_gains(cfg: ArmConfig, index: int) -> tuple[float, float]:
    ref = cfg.reference_gains or {}
    kp_list = ref.get("kp") or []
    kd_list = ref.get("kd") or []
    scale = cfg.thresholds.live_kp_scale
    kp = (float(kp_list[index]) if index < len(kp_list) else 8.0) * scale
    kd = float(kd_list[index]) if index < len(kd_list) else 1.0
    return kp, kd


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _render(cfg: ArmConfig, states: list[JointLive], *, mode: str, active: int,
            step: float, estop: bool, moving: bool, note: str = "", trigger=None):
    from rich.table import Table
    from rich.panel import Panel
    from rich.console import Group

    warn, fail = cfg.thresholds.temp_warn_c, cfg.thresholds.temp_fail_c

    table = Table(expand=True)
    table.add_column("Joint", no_wrap=True)
    table.add_column("Type")
    if moving:
        table.add_column("Target°", justify="right")
    table.add_column("Pos°", justify="right")
    table.add_column("Vel°/s", justify="right")
    table.add_column("Torque", justify="right")
    table.add_column("MOS°C", justify="right")
    table.add_column("Rotor°C", justify="right")
    table.add_column("State")

    for i, s in enumerate(states):
        name = s.joint.name
        marker = "[reverse] > [/reverse]" if (moving and i == active) else "  "
        if not s.present or s.fb is None:
            row = [f"{marker}{name}", s.joint.motor_type.value]
            if moving:
                row.append("-")
            row += ["-", "-", "-", "-", "-", "[red]no reply[/red]"]
            table.add_row(*row)
            continue
        fb = s.fb
        tmax = max(fb.temp_mos, fb.temp_rotor)
        tcol = "red" if tmax >= fail else ("yellow" if tmax >= warn else "green")
        state = fb.error_text
        scol = "green" if fb.healthy else ("dim" if fb.error_code == 0 else "bold red")
        row = [f"{marker}{name}", s.joint.motor_type.value]
        if moving:
            row.append(f"{s.command_deg:7.1f}")
        row += [
            f"{rad_to_deg(fb.position):7.1f}",
            f"{rad_to_deg(fb.velocity):7.1f}",
            f"{fb.torque:6.2f}",
            f"[{tcol}]{fb.temp_mos}[/{tcol}]",
            f"[{tcol}]{fb.temp_rotor}[/{tcol}]",
            f"[{scol}]{state}[/{scol}]",
        ]
        table.add_row(*row)

    # header / controls
    if estop:
        status = "[bold red]E-STOP — motors disabled[/bold red]"
    elif not moving:
        status = "[green]MONITOR — read-only, no motion[/green]"
    else:
        status = f"[cyan]{mode.upper()}[/cyan]  active=[bold]{states[active].joint.name}[/bold]  step={step:.1f}°"

    if mode == "jog":
        keys = ("[b]↑/↓[/b] or [b]+/-[/b] move   [b]←/→[/b] select joint   "
                "[b],/.[/b] step   [b]h[/b] hold   [b]SPACE[/b] e-stop   "
                "[b]e[/b] re-enable   [b]q[/b] quit")
    elif mode == "exercise":
        keys = "[b]SPACE[/b] e-stop   [b]e[/b] re-enable   [b]q[/b] quit"
    else:
        keys = "[b]q[/b] quit"

    # Leader trigger handle (read-only), if present.
    if cfg.trigger is not None:
        if trigger is None:
            trig_line = f"[red]{cfg.trigger.name}: no reply (encoder 0x{cfg.trigger.encoder_id:X})[/red]"
        else:
            bar_n = int(round(trigger.trigger * 10))
            bar = "█" * bar_n + "·" * (10 - bar_n)
            btns = "".join("●" if b else "○" for b in trigger.buttons)
            trig_line = (f"{cfg.trigger.name}: [{bar}] {trigger.trigger:.2f}  "
                        f"(gripper_cmd={trigger.gripper_cmd:.2f})  buttons {btns}")
        status = f"{status}\n{trig_line}"

    head = f"{status}\n{keys}"
    if note:
        head += f"\n[yellow]{note}[/yellow]"
    title = f"YAM Pro {cfg.arm} — live {mode}"
    return Group(Panel(head, title=title), table)


# ---------------------------------------------------------------------------
# Main live loop
# ---------------------------------------------------------------------------
def run_live(
    chain: MotorChain,
    cfg: ArmConfig,
    mode: str = "monitor",
    include_gripper: bool = False,
    joints_filter: Optional[list[str]] = None,
    amp_deg: Optional[float] = None,
    period_s: Optional[float] = None,
) -> None:
    """Run a live mode until the operator quits.

    mode: "monitor" | "jog" | "exercise".
    """
    from rich.live import Live
    from rich.console import Console

    console = Console()
    moving = mode in ("jog", "exercise")
    th = cfg.thresholds
    dt = 1.0 / max(1.0, th.live_rate_hz)
    max_step_per_cycle = th.live_max_vel_deg * dt  # slew limit -> deg per cycle

    motors = cfg.all_motors(include_gripper=include_gripper)
    if joints_filter:
        want = {j.lower() for j in joints_filter}
        motors = [m for m in motors if m.name.lower() in want]
    if not motors:
        console.print("[red]No matching joints to run.[/red]")
        return

    states: list[JointLive] = []
    for i, j in enumerate(motors):
        kp, kd = _joint_gains(cfg, i)
        states.append(JointLive(joint=j, index=i, kp=kp, kd=kd))

    if moving and not _HAVE_KEYS:
        console.print("[red]Keyboard control needs Windows (msvcrt) — use monitor mode.[/red]")
        return

    active = 0
    step = th.live_step_deg
    estop = False
    amp = amp_deg if amp_deg is not None else th.live_exercise_amp_deg
    period = period_s if period_s is not None else th.live_exercise_period_s
    note = ""

    def enable_all() -> None:
        """Enable moving joints; seed targets at measured position."""
        for s in states:
            fb = chain.enable_joint(s.joint)
            if fb is None:
                s.present = False
                continue
            if not fb.healthy and fb.error_code != 0:
                fb = chain.recover_joint(s.joint) or fb
            s.present = True
            s.fb = fb
            pos = rad_to_deg(fb.position)
            s.desired_deg = s.clamp(pos)
            s.command_deg = s.clamp(pos)
            s.center_deg = s.clamp(pos)

    def disable_all() -> None:
        for s in states:
            try:
                chain.disable_joint(s.joint)
            except Exception:
                pass

    console.print(f"[bold]Starting live {mode}.[/bold] "
                  + ("Read-only — no motion." if not moving else
                     "Motors will be ENABLED and will move. Keep clear."))
    if moving:
        console.print("Enabling motors...")
        enable_all()

    t0 = time.perf_counter()
    try:
        with Live(console=console, refresh_per_second=max(4, min(30, int(th.live_rate_hz))),
                  screen=False) as live:
            while True:
                loop_start = time.perf_counter()

                # ---- input --------------------------------------------------
                key = _read_key()
                if key is not None:
                    if key in ("q", "ESC"):
                        break
                    if key == " ":
                        estop = True
                        disable_all()
                        note = "E-STOP pressed."
                    elif key == "e" and estop and moving:
                        estop = False
                        note = ""
                        enable_all()
                    elif moving and not estop:
                        if key in ("LEFT",):
                            active = (active - 1) % len(states)
                        elif key in ("RIGHT",):
                            active = (active + 1) % len(states)
                        elif key in ("UP", "+", "="):
                            s = states[active]
                            s.desired_deg = s.clamp(s.desired_deg + step)
                        elif key in ("DOWN", "-", "_"):
                            s = states[active]
                            s.desired_deg = s.clamp(s.desired_deg - step)
                        elif key == ".":
                            step = min(30.0, step + 1.0)
                        elif key == ",":
                            step = max(0.5, step - 1.0)
                        elif key == "h":  # hold: desired <- current measured
                            for s in states:
                                if s.fb is not None:
                                    s.desired_deg = s.clamp(rad_to_deg(s.fb.position))

                # ---- exercise trajectory -----------------------------------
                if mode == "exercise" and not estop:
                    t = time.perf_counter() - t0
                    phase = 2.0 * math.pi * t / max(0.1, period)
                    for s in states:
                        s.desired_deg = s.clamp(s.center_deg + amp * math.sin(phase))

                # ---- command / read each motor -----------------------------
                faulted = None
                for s in states:
                    if not s.present and moving:
                        continue
                    if not moving or estop:
                        fb = chain.read(s.joint.motor_id, s.joint.motor_type,
                                        _LOOP_TIMEOUT, _LOOP_RETRIES)
                    else:
                        # slew command_deg toward desired_deg (speed limit)
                        delta = s.desired_deg - s.command_deg
                        if abs(delta) > max_step_per_cycle:
                            delta = math.copysign(max_step_per_cycle, delta)
                        s.command_deg = s.clamp(s.command_deg + delta)
                        fb = chain.command(
                            s.joint.motor_id, s.joint.motor_type,
                            position=deg_to_rad(s.command_deg),
                            velocity=0.0, kp=s.kp, kd=s.kd, torque=0.0,
                            timeout=_LOOP_TIMEOUT, retries=_LOOP_RETRIES,
                        )
                    if fb is not None:
                        s.fb = fb
                        if moving and not estop and not fb.healthy and fb.error_code != 0:
                            faulted = (s.joint.name, fb.error_text)

                # ---- leader trigger handle (read-only) ---------------------
                trigger = None
                if cfg.trigger is not None:
                    trigger = chain.read_encoder(
                        cfg.trigger.encoder_id, cfg.trigger.range_rad,
                        _LOOP_TIMEOUT, _LOOP_RETRIES)

                # ---- fault -> auto e-stop ----------------------------------
                if faulted and not estop:
                    estop = True
                    disable_all()
                    note = f"FAULT on {faulted[0]}: {faulted[1]} — auto E-STOP. Press 'e' to recover."

                live.update(_render(cfg, states, mode=mode, active=active, step=step,
                                    estop=estop, moving=moving, note=note, trigger=trigger))

                # ---- pace the loop -----------------------------------------
                elapsed = time.perf_counter() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        if moving:
            console.print("Disabling motors...")
            disable_all()
        console.print("[bold]Live session ended.[/bold]")
