# Damiao Debugging Assistant (parameters & firmware)

Some things can't be done over CAN with this toolkit — reading/writing a motor's
stored **parameters**, changing its **control mode**, or **re-flashing firmware**.
Those use Damiao's **UART debugging port** and their official **debugging
assistant** GUI. This page is the workflow; use it alongside `yam-test motor`.

> **Why not in `yam-test`?** Firmware flashing uses Damiao's bootloader over
> UART — reimplementing it risks **bricking** the motor. Parameter access needs
> the "DM Drive Control Protocol" register map. Use the official tool for these;
> use `yam-test` for CAN diagnostics.

## What you need (from the DM-J4310 manual)

- The motor's **debugging serial port**: a **GH1.25 3-pin** connector.
- A **USB-to-serial** module (or Damiao's USB-to-CAN debugging tool, which also
  carries the serial lines).
- **UART @ 921600 bps** — the assistant sets this automatically.
- The **Damiao debugging assistant** software + its manual:
  <http://www.dmbot.cn/forum.php?mod=viewthread&tid=364&extra=page%3D1>

## Connect one motor (e.g. J6) on the bench

1. Power the motor from a 24 V supply (XT30(2+2)-F power/CAN cable).
2. Plug the **GH1.25 3-pin** debug cable from the motor to the USB-to-serial module.
3. Plug the USB-to-serial module into the PC.
4. Open the **Damiao debugging assistant**, pick the serial port, open it.
5. Power on the motor — the serial log prints, and **"Control Mode"** shows the
   motor's current driving mode.

## What to check for a "won't enable" motor (J6, red-solid LED)

| Check | Why it matters |
|---|---|
| **Control mode** | Should be **MIT** for how `yam-test` and i2rt drive it. A wrong mode explains odd enable behavior. |
| **ESC_ID (CAN id)** | Confirm it's the expected id (J6 = 0x06). A duplicate/wrong id causes conflicts. |
| **Master ID (feedback id)** | i2rt/YAM expects feedback on id + 16 (0x16 for J6). |
| **Protection thresholds** | Undervoltage / overcurrent / overtemp limits — a too-tight limit can keep tripping it. |
| **Firmware version** | Compare against a known-good motor; re-flash only if Damiao advises. |

If the control mode, IDs, and limits all look right but the motor still won't
leave disable mode, that points to the **actuator's driver hardware** — do the
swap test below.

## Swap test (isolate actuator vs wiring) — no assistant needed

Use `yam-test` for this; it's the fastest way to localise a J6-type fault:

```powershell
# 1. test the suspect joint
yam-test motor --joint J6 --enable --out reports\j6-before.json

# 2. move J6's motor to a known-good position on the bus (or swap the actuator),
#    then test again at the SAME can id
yam-test motor --id 0x06 --type DM4310 --enable --out reports\j6-after.json
```

- Fault **follows the motor** → that actuator's driver is bad (RMA / replace, or
  try re-flashing firmware with the assistant).
- Fault **stays with the position/wiring** → it's the harness/connector to J6,
  not the motor.

## Re-flashing firmware (assistant only)

Follow the assistant's manual exactly. Only re-flash when Damiao support advises
it or a swap test proves the driver firmware is the issue. Do **not** power-cycle
or disconnect mid-flash. `yam-test` deliberately does not do this.

## Reading parameters over CAN (planned)

Damiao motors also expose a register read/write protocol on CAN id `0x7FF`. Once
the "DM Drive Control Protocol" register map is confirmed, `yam-test motor
--params` will read control mode / IDs / protection thresholds over CAN (read-only)
so you can check them without the serial cable. Not yet enabled — pending that doc.
