# Windows Setup (gs_usb / CANable)

One-time setup to make a **gs_usb** adapter (CANable / candleLight) work on
Windows. Your adapter is a **CANable 2** (`VID_1D50 PID_606F`, candleLight
firmware) — it needs the **WinUSB** driver bound with Zadig before software can
open it. This is normal: Windows ships no driver for candleLight.

## 1. Install the Python deps

```powershell
cd "path\to\Arm_Test"
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
pip install -e .
```

(The libusb DLL the backend needs is bundled via `libusb-package`; `yam-test`
adds it to the search path automatically — no manual PATH editing.)

## 2. Bind the adapter to WinUSB with Zadig

Symptom if you skip this: `yam-test scan` may find nothing, or opening fails with
**"Entity not found" / "No backend available"**. The device shows **Status: Error,
no driver** in Device Manager.

1. Download **Zadig** from https://zadig.akeo.ie/ (single .exe, no install).
2. Plug in the CANable. Run Zadig **as Administrator**.
3. **Options → List All Devices** (tick it).
4. In the dropdown, select **`canable2 gs_usb`** — importantly the one whose
   interface is **(Interface 0)** / `MI_00`. It's a composite device, so you may
   see two entries; pick the **gs_usb / Interface 0** one, not any CDC/serial one.
5. To the right of the green arrow, choose the target driver: **WinUSB**.
6. Click **Install Driver** (or *Replace Driver*). Wait for "successfully
   installed".
7. Unplug and replug the adapter.

> Reversible: if you ever need the old state back, Device Manager → the device →
> *Uninstall device* (tick "delete driver"), then replug.

## 3. Verify

```powershell
yam-test scan          # should list a gs_usb candidate
yam-test can           # with an arm powered: ~6 feedback IDs streaming
```

Quick raw check that the driver bind worked (no arm needed):

```powershell
python -c "from gs_usb.gs_usb import GsUsb; d=GsUsb.scan(); print('found', len(d)); dev=d[0]; dev.start(0); print('opened OK'); dev.stop()"
```

- `found 1` **and** `opened OK` → driver is bound correctly; you're done.
- `found 1` but an error on open → WinUSB not bound to **Interface 0**; redo step 2.
- `found 0` → adapter not enumerating; try another USB port/cable.

## Channel / multiple adapters

- Single adapter: `yam-test` defaults to `gs_usb` channel `0` — no flags needed.
- Two adapters (e.g. leader + follower on separate buses): they enumerate as
  index `0` and `1`. Select with `--interface gs_usb --channel 1`. Plug them in a
  consistent order, or label the USB ports, so leader/follower stay predictable.

## Alternative: slcan firmware (no Zadig)

Some CANables run **slcan** firmware instead, which appears as a **COM port** and
needs no Zadig:

```powershell
yam-test can --interface slcan --channel COM5
```

Yours is running **gs_usb/candleLight** firmware (PID 606F), so use the Zadig path
above. Only consider reflashing to slcan if you specifically prefer COM-port
access.

## Do I need Linux?

No — this Windows PC runs the **`yam-test` toolkit** fine over gs_usb. Only
i2rt's *own* control stack (`get_yam_robot`, teleop) requires Linux, because it
uses socketcan. See the note in [wiring-and-can.md](wiring-and-can.md).
