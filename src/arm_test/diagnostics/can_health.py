"""CAN-bus health checks (bus level, mostly protocol-independent).

These answer "is the CAN wiring/adapter/bus itself healthy?" *before* we blame
a motor. Passive checks only listen; the active ping lives in motor_test.py
because it needs the DM protocol.
"""

from __future__ import annotations

import time
from collections import Counter
from typing import Optional

from .report import CheckResult, Status


def passive_traffic_scan(bus, duration: float = 2.0) -> CheckResult:
    """Listen (no transmit) and summarise what IDs are on the bus.

    IMPORTANT: DM motors are request/response — they only reply when polled, they
    do NOT free-run. So silence here is NORMAL unless something is actively
    driving the bus (e.g. an i2rt controller on another node). To actively check
    an arm use `yam-test arm` / `yam-test motors`; to check the adapter alone use
    `yam-test selftest`. Error frames here, though, still mean a real bus fault.
    """
    ids: Counter = Counter()
    errors = 0
    end = time.time() + duration
    while time.time() < end:
        msg = bus.recv(timeout=0.2)
        if msg is None:
            continue
        if getattr(msg, "is_error_frame", False):
            errors += 1
            continue
        ids[msg.arbitration_id] += 1

    total = sum(ids.values())
    data = {
        "frames": total,
        "unique_ids": len(ids),
        "ids_seen": sorted(hex(i) for i in ids),
        "id_counts": {hex(k): v for k, v in ids.most_common()},
        "error_frames": errors,
        "duration_s": duration,
    }

    if total == 0 and errors == 0:
        return CheckResult(
            "CAN passive scan", Status.WARN,
            "No traffic in %.1fs. This is NORMAL if nothing is polling — DM motors "
            "only reply when asked. Use 'yam-test arm'/'motors' to actively probe "
            "the arm, or 'yam-test selftest' to check the adapter alone." % duration,
            data,
        )
    if errors and total == 0:
        return CheckResult(
            "CAN passive scan", Status.FAIL,
            f"Only error frames ({errors}). Almost always a BITRATE MISMATCH "
            "or bad wiring/termination.", data,
        )
    status = Status.WARN if errors else Status.PASS
    detail = f"{total} frames, {len(ids)} node(s): {', '.join(data['ids_seen'])}"
    if errors:
        detail += f"  (+{errors} error frames — investigate wiring/termination)"
    return CheckResult("CAN passive scan", status, detail, data)


def error_frame_watch(bus, duration: float = 3.0) -> CheckResult:
    """Count error frames + bus-error state over a window.

    Rising error frames while otherwise operating = intermittent wiring, a
    flaky connector, noise, or a node with the wrong bitrate joining/leaving.
    """
    errors = 0
    frames = 0
    end = time.time() + duration
    while time.time() < end:
        msg = bus.recv(timeout=0.2)
        if msg is None:
            continue
        frames += 1
        if getattr(msg, "is_error_frame", False):
            errors += 1

    ratio = (errors / frames) if frames else 0.0
    data = {"frames": frames, "error_frames": errors, "error_ratio": round(ratio, 4)}
    if errors == 0:
        return CheckResult("CAN error frames", Status.PASS, "No error frames.", data)
    if ratio > 0.05:
        return CheckResult(
            "CAN error frames", Status.FAIL,
            f"{errors}/{frames} error frames ({ratio:.1%}) — bus is unhealthy. "
            "Check connectors, cable length, termination.", data,
        )
    return CheckResult(
        "CAN error frames", Status.WARN,
        f"{errors} error frames seen ({ratio:.1%}). Intermittent — inspect wiring.", data,
    )


def bus_load_estimate(bus, duration: float = 1.0, bitrate: int = 1_000_000) -> CheckResult:
    """Rough bus-load %: assumes ~130 bits/standard-frame on average.

    Not exact (depends on stuffing/DLC) but good enough to flag a saturated bus.
    """
    frames = 0
    end = time.time() + duration
    while time.time() < end:
        if bus.recv(timeout=0.2) is not None:
            frames += 1
    bits = frames * 130
    load = bits / (bitrate * duration)
    data = {"frames": frames, "approx_load_pct": round(load * 100, 1)}
    if load > 0.8:
        return CheckResult("CAN bus load", Status.WARN,
                           f"~{load:.0%} load — high, may drop frames.", data)
    return CheckResult("CAN bus load", Status.PASS, f"~{load:.0%} bus load.", data)
