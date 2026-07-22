"""Adapter self-test — verify a gs_usb CANable with NO arm, wiring, or termination.

Uses the adapter's internal **loopback** mode: frames the adapter sends are
echoed straight back inside the chip, so a round-trip proves the USB link, the
driver bind (WinUSB), and the adapter's TX + RX paths all work. This is the
bench test to run before you ever connect an arm.

Loopback is a gs_usb feature, so this talks to the gs_usb package directly
(python-can doesn't expose the mode). Non-gs_usb adapters report SKIP.
"""

from __future__ import annotations

import time

from .can_backend import _ensure_libusb_on_path
from .diagnostics.report import CheckResult, Status

# Distinct id/data patterns to round-trip. Kept at 3: in internal loopback the
# gs_usb TX echo-context pool (~3) isn't recycled like it is on a real bus where
# each motor ACK frees it, so only the first ~3 frames per start() echo back.
# Three varied frames (different ids, lengths, and data) conclusively prove the
# adapter's USB link + driver + TX/RX paths.
_TEST_FRAMES = [
    (0x123, [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04]),  # 8 bytes
    (0x011, [0x11, 0x22, 0x33, 0x44]),                          # 4 bytes
    (0x555, [0xAA, 0xBB]),                                      # 2 bytes
]


def gs_usb_loopback_selftest(index: int = 0, bitrate: int = 1_000_000) -> CheckResult:
    """Round-trip frames through the adapter's internal loopback."""
    _ensure_libusb_on_path()
    try:
        from gs_usb.gs_usb import GsUsb
        from gs_usb.gs_usb_frame import GsUsbFrame
        from gs_usb.constants import GS_CAN_MODE_LOOP_BACK, GS_CAN_MODE_HW_TIMESTAMP
    except ImportError:
        return CheckResult("Adapter loopback self-test", Status.SKIP,
                           "gs_usb package not installed — loopback test is gs_usb-only.", {})

    def _safe_read(dev, frame, ms):
        # dev.read only guards USBError; a stale frame from a prior session with a
        # different HW-timestamp setting raises struct.error on unpack. Treat any
        # such failure as "no valid frame" so the drain/read loops don't crash.
        try:
            return dev.read(frame, ms)
        except Exception:
            return False

    devices = GsUsb.scan()
    if not devices:
        return CheckResult("Adapter loopback self-test", Status.FAIL,
                           "No gs_usb adapter found. Plug it in / check USB.", {"found": 0})
    if index >= len(devices):
        return CheckResult("Adapter loopback self-test", Status.FAIL,
                           f"gs_usb index {index} out of range ({len(devices)} found).",
                           {"found": len(devices)})

    dev = devices[index]
    try:
        # Reset any prior state, then start in loopback. Include HW_TIMESTAMP so
        # our framing matches what python-can leaves the device in (24-byte
        # frames) — otherwise stale frames misalign and unpack fails.
        try:
            dev.stop()
        except Exception:
            pass
        dev.set_bitrate(bitrate)
        dev.start(GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_HW_TIMESTAMP)
    except Exception as exc:
        hint = ""
        if "not found" in str(exc).lower() or "entity" in str(exc).lower():
            hint = " Adapter not bound to WinUSB — run Zadig (see docs/setup-windows.md)."
        return CheckResult("Adapter loopback self-test", Status.FAIL,
                           f"Could not start adapter: {type(exc).__name__}: {exc}.{hint}", {})

    try:
        # Drain anything left in the buffer from a prior session (up to ~0.6s).
        drained = 0
        rx = GsUsbFrame()
        t_end = time.time() + 0.6
        while time.time() < t_end:
            if _safe_read(dev, rx, 30):
                drained += 1
            else:
                break

        # Interleave: for each frame, clear the buffer, send it, then read its
        # echo back. Avoids both stale-frame confusion and echo-pool exhaustion.
        passed = 0
        results = []
        for cid, data in _TEST_FRAMES:
            drain = GsUsbFrame()
            while _safe_read(dev, drain, 5):
                pass
            dev.send(GsUsbFrame(can_id=cid, data=list(data)))
            got = False
            deadline = time.time() + 0.4
            while time.time() < deadline:
                rx = GsUsbFrame()
                if _safe_read(dev, rx, 50) and not rx.is_error_frame:
                    if rx.arbitration_id == cid and bytes(rx.data[:len(data)]) == bytes(data):
                        got = True
                        break
            passed += got
            results.append({"id": hex(cid), "bytes": len(data), "ok": got})
    finally:
        try:
            dev.stop()
        except Exception:
            pass

    data = {"found": len(devices), "drained": drained,
            "passed": passed, "total": len(_TEST_FRAMES), "frames": results}
    if passed == len(_TEST_FRAMES):
        return CheckResult(
            "Adapter loopback self-test", Status.PASS,
            f"{passed}/{len(_TEST_FRAMES)} frames round-tripped — adapter TX+RX OK "
            "(USB, driver, controller all good).", data)
    if passed == 0:
        return CheckResult(
            "Adapter loopback self-test", Status.FAIL,
            "Adapter started but echoed nothing back — RX path or firmware issue.", data)
    return CheckResult(
        "Adapter loopback self-test", Status.WARN,
        f"Only {passed}/{len(_TEST_FRAMES)} frames round-tripped — intermittent adapter.", data)
