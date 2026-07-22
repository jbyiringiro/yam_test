"""Adapter-agnostic CAN bus access for the YAM arm diagnostics.

The YAM Pro talks 1 Mbit/s CAN. On Windows the *physical adapter* varies
(CANable/slcan, PEAK PCAN, Kvaser, 8devices USB2CAN, ...). This module hides
that behind a single `open_bus()` factory plus a best-effort `detect_backends()`
so the tools work regardless of which adapter is plugged in.

python-can does the heavy lifting; we just pick/validate the interface.
"""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from typing import Optional


def _ensure_libusb_on_path() -> None:
    """Make the bundled libusb-1.0.dll discoverable for the gs_usb backend.

    pyusb (which gs_usb uses) needs a libusb backend DLL on the search path. The
    `libusb-package` wheel bundles one; we add its directory so gs_usb works on
    Windows without the user hand-editing PATH. Harmless if not installed.
    """
    try:
        import libusb_package

        dlldir = os.path.dirname(libusb_package.__file__)
        if dlldir not in os.environ.get("PATH", ""):
            os.environ["PATH"] = dlldir + os.pathsep + os.environ.get("PATH", "")
        add = getattr(os, "add_dll_directory", None)
        if add is not None:
            try:
                add(dlldir)
            except (OSError, FileNotFoundError):
                pass
    except Exception:
        pass


_ensure_libusb_on_path()

try:
    import can  # python-can
except ImportError as exc:  # pragma: no cover - import guard
    raise ImportError(
        "python-can is not installed. Run:  pip install -r requirements.txt"
    ) from exc


DEFAULT_BITRATE = 1_000_000  # YAM Pro CAN bus = 1 Mbit/s (per spec sheet)


@dataclass
class BackendSpec:
    """A concrete python-can interface + channel we can try to open."""

    interface: str          # python-can 'interface' / bustype name
    channel: str            # channel or COM port
    note: str = ""          # human-friendly description
    extra: Optional[dict] = None  # extra kwargs passed to can.Bus

    def as_kwargs(self, bitrate: int = DEFAULT_BITRATE) -> dict:
        kwargs = {"interface": self.interface, "channel": self.channel, "bitrate": bitrate}
        if self.extra:
            kwargs.update(self.extra)
        return kwargs

    def label(self) -> str:
        return f"{self.interface}:{self.channel}" + (f"  ({self.note})" if self.note else "")


def detect_backends() -> list[BackendSpec]:
    """Best-effort scan for likely CAN interfaces on this machine.

    We use python-can's `detect_available_configs()` where the backend
    supports it, then add heuristic candidates (CANable slcan on COM ports,
    gs_usb). Returned list is *candidates*, not guaranteed-open buses.
    """
    candidates: list[BackendSpec] = []

    # 0) gs_usb is the adapter in use here (CANable/candleLight family). List it
    #    first so auto-open tries it before anything else. gs_usb enumeration
    #    isn't always reported by detect_available_configs, so add an explicit
    #    channel-0 candidate as a reliable fallback.
    gs_usb_seen = False
    try:
        for cfg in can.detect_available_configs(interfaces=["gs_usb"]):
            gs_usb_seen = True
            candidates.append(
                BackendSpec(
                    interface="gs_usb",
                    channel=str(cfg.get("channel", "0")),
                    note="gs_usb (CANable/candleLight) — auto-detected",
                )
            )
    except Exception:
        pass
    if not gs_usb_seen:
        candidates.append(
            BackendSpec(interface="gs_usb", channel="0",
                        note="gs_usb (CANable/candleLight) — default guess")
        )

    # 1) python-can's own auto-detection for the other backends.
    for iface in ("pcan", "kvaser", "ixxat", "usb2can", "neovi", "vector"):
        try:
            for cfg in can.detect_available_configs(interfaces=[iface]):
                candidates.append(
                    BackendSpec(
                        interface=cfg.get("interface", iface),
                        channel=str(cfg.get("channel", "")),
                        note="auto-detected",
                    )
                )
        except Exception:
            # Backend not installed / no driver — skip quietly.
            continue

    # 2) CANable / USB-serial (slcan) — enumerate COM ports on Windows.
    try:
        from serial.tools import list_ports  # provided by pyserial (python-can dep)

        for port in list_ports.comports():
            desc = (port.description or "").lower()
            hwid = (port.hwid or "").lower()
            likely = any(k in desc + hwid for k in ("canable", "can", "slcan", "usb serial", "cdc"))
            candidates.append(
                BackendSpec(
                    interface="slcan",
                    channel=port.device,
                    note=("likely CAN adapter" if likely else port.description or "serial port"),
                )
            )
    except Exception:
        pass

    return candidates


def open_bus(
    interface: Optional[str] = None,
    channel: Optional[str] = None,
    bitrate: int = DEFAULT_BITRATE,
    receive_own_messages: bool = False,
    **extra,
) -> "can.BusABC":
    """Open a CAN bus.

    If `interface`/`channel` are given, open exactly that. Otherwise try the
    first auto-detected candidate. Raises a clear error listing what was found
    if nothing works, so a maintenance tech gets actionable guidance.
    """
    if interface and channel:
        return can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=receive_own_messages,
            **extra,
        )

    candidates = detect_backends()
    errors: list[str] = []
    for spec in candidates:
        try:
            return can.Bus(
                receive_own_messages=receive_own_messages,
                **spec.as_kwargs(bitrate),
            )
        except Exception as exc:  # try the next candidate
            errors.append(f"  - {spec.label()} -> {type(exc).__name__}: {exc}")

    detail = "\n".join(errors) if errors else "  (no CAN interfaces detected)"
    gs_usb_hint = ""
    if any("gs_usb" in e for e in errors) and any(
        k in e for e in errors for k in ("Entity not found", "USBError", "No backend", "NoBackend")
    ):
        gs_usb_hint = (
            "\n  * gs_usb adapter is present but not openable — bind it to the\n"
            "    WinUSB driver with Zadig (one-time). See docs/setup-windows.md.\n"
        )
    raise ConnectionError(
        "Could not open any CAN interface.\n"
        f"Tried:\n{detail}\n\n"
        "Fixes:\n"
        "  * Plug in the USB-CAN adapter and install its driver.\n"
        "  * Specify it explicitly, e.g.:  --interface slcan --channel COM5\n"
        "  * List candidates with:          yam-test scan\n"
        + gs_usb_hint
    )


def is_slcan_tool_present() -> bool:
    """Heuristic: is a serial terminal available for manual slcan poking?"""
    return shutil.which("python") is not None
