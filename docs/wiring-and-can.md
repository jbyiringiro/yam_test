# Wiring & CAN Bus

How the YAM Pro's CAN bus is wired, how to connect a diagnostic adapter, and how
to keep the bus healthy. This is the layer to check **before** blaming a motor.

## CAN basics for this arm

- **Bitrate: 1 Mbit/s.** Every node (all 6 motors + your adapter) must agree.
  A single node at the wrong bitrate floods the bus with error frames and can
  take the whole bus down.
- **Topology: linear bus, not a star.** Motors are daisy-chained CAN-H / CAN-L
  with short drops. Long stubs and star wiring cause reflections/errors at 1 Mbit/s.
- **Termination: 120 Ω at *both* ends of the bus.** Many DM-motor harnesses put
  one terminator at the far motor and expect the adapter end to terminate the
  other. Missing or doubled termination = error frames or no comms.
- **Shared harness.** CAN signal and motor power typically run in the same cable
  bundle. A marginal power connection can brown-out motors and *look* like a CAN
  fault. Check power and CAN together.

## Signal wiring

| Wire | Meaning |
|---|---|
| CAN-H | Differential high |
| CAN-L | Differential low |
| GND | Signal ground / reference — **must** be common between adapter and arm |
| V+ | Motor power (separate supply; do not power motors from the USB adapter) |

> Always tie the adapter's CAN ground to the arm's ground. "Floating" grounds are
> a classic cause of intermittent, weather-dependent CAN errors.

## Adapters {#adapters}

**We use a `gs_usb` adapter** (CANable / candleLight family). It's a raw-USB
device — **no COM port** — so on Windows it must be bound to the WinUSB driver
once with **Zadig**. The toolkit defaults to `gs_usb` channel `0`, so once set up,
`yam-test can` just works with no flags.

### gs_usb one-time Windows setup

1. `pip install -r requirements.txt` (installs `gs_usb` + `libusb-package`).
2. Plug in the adapter.
3. Run **[Zadig](https://zadig.akeo.ie/)** → *Options ▸ List All Devices* →
   select the CANable/candleLight device → install **WinUSB** as its driver.
   (If Windows already gave it a "usb serial"/COM port, that's the *slcan*
   firmware path instead — see the table below.)
4. `yam-test scan` — it should list a `gs_usb` candidate.
5. `yam-test can` — should stream ~6 feedback IDs with the arm powered.

> If Zadig replaced a working COM-port (slcan) driver and you'd rather use slcan,
> revert the driver in Device Manager and use `--interface slcan --channel COMx`.

### Other adapters (only if you switch hardware)

| Adapter | `--interface` | `--channel` example | Setup |
|---|---|---|---|
| **CANable/candleLight (gs_usb)** — ours | `gs_usb` | `0` | `requirements.txt` + Zadig WinUSB |
| CANable (slcan firmware) | `slcan` | `COM5` | COM port; no extra install |
| PEAK PCAN-USB | `pcan` | `PCAN_USBBUS1` | Install PEAK driver |
| Kvaser Leaf/USBcan | `kvaser` | `0` | Install Kvaser CANlib |
| 8devices USB2CAN | `usb2can` | serial/`0` | Install USB2CAN driver |
| IXXAT USB-to-CAN | `ixxat` | `0` | Install VCI driver |

Verify what's detected:

```powershell
yam-test scan          # prints detected interface + channel candidates
```

The default interface/channel live in [config/yam_pro.yaml](../config/yam_pro.yaml)
(`can.interface: gs_usb`, `can.channel: "0"`). Override per-run with
`--interface`/`--channel` when needed.

## Health checks (what "good" looks like)

Run `yam-test can` with the arm powered:

- **Passive scan** should show ~6 continuously-streaming feedback IDs (one per
  joint). That single result proves: adapter OK, bitrate correct, wiring/termination
  good enough, and at least those motors alive.
- **Error frames** should be ~zero. A steady error-frame stream almost always
  means **bitrate mismatch** or **termination/wiring**.
- **Bus load** at 1 Mbit/s with 6 motors streaming is low; a saturated bus points
  at a misbehaving/babbling node.

## Debugging a sick bus

1. **No frames at all** → bitrate wrong, adapter driver missing, no termination,
   or motors unpowered. Confirm 1 Mbit/s; measure ~60 Ω across CAN-H/CAN-L with
   power off (two 120 Ω in parallel).
2. **Only error frames** → bitrate mismatch is #1. Also check for a shorted or
   swapped CAN-H/CAN-L.
3. **Intermittent errors** → connectors. Do a wiggle test while running
   `error_frame_watch`; reseat every drop connector; check strain relief.
4. **One node missing** → that motor's power/CAN drop, or a wrong/duplicate CAN
   ID. Isolate by testing that motor alone.

## Termination quick test (power OFF)

Measure resistance across CAN-H and CAN-L:

| Reading | Meaning |
|---|---|
| ~60 Ω | Correct — two 120 Ω terminators in parallel |
| ~120 Ω | Only one terminator present — add the other |
| open / very high | No termination — bus will error at 1 Mbit/s |
| ~0 Ω / very low | Short between CAN-H and CAN-L |

> Motor CAN-ID assignment and the send/receive arbitration-ID scheme are in
> [motors.md](motors.md).
