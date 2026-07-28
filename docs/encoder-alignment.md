# Aligning the Trigger Encoder Magnet

The leader handle's trigger is a **magnetic absolute encoder**: a **diametric
magnet** on the shaft, read by an encoder chip. Reading accuracy depends entirely
on how that magnet sits relative to the chip. A misaligned magnet gives an
offset/reversed zero (the "rest reads 0.98, squeeze opens" fault) and non-linear
or jittery values.

`yam-test trigger` is your alignment gauge — it shows the **raw position** and a
running **min/max span** so you can set the magnet by feedback instead of guesswork.

## The four things that must be right

| Aspect | Symptom if wrong | Fix |
|---|---|---|
| **Air gap** (magnet↔chip distance) | Jittery/noisy (too far) or stuck/saturated (too near) | Match the good handle's gap (typ. ~0.5–2 mm; check the spec) |
| **Concentricity** (magnet centered on chip) | Non-linear — value speeds up/slows through the sweep, dead spots | Re-center the magnet so its spin axis is over the chip center |
| **Parallel / flat** (no tilt) | Distorted, inconsistent readings | Magnet face parallel to the chip, spinning in-plane |
| **Rotational zero** (orientation vs rest) | Offset/reversed (rest ≠ 0), or a jump at ±π | Rotate the magnet so **rest reads ~0.000 rad** |

## Procedure (use `yam-test trigger` as the gauge)

**1. Baseline a GOOD handle first.** Run it on a working handle and note the
targets:
```powershell
.\yam.bat trigger
```
- **At rest:** `raw pos ≈ 0.000 rad`, `trigger ≈ 0.00`
- **Fully squeezed:** note the `trigger` (~1.00) and the `span` (e.g. pos `0.000 .. +0.70`)

That span + a ~0 rest is your target for the faulty handle.

**2. Set the mechanicals on the faulty handle** (power off): match the good
handle's **air gap**, make sure the magnet is **centered** under the chip and
**flat** (not tilted). Most drift comes from gap and centering.

**3. Set the rotational zero with live feedback.** Power on, run `.\yam.bat trigger`,
hold the trigger at its **released/rest** position, then loosen the magnet (or its
holder) and **rotate it until `raw pos` reads ~0.000 rad**. Lock it down.

**4. Verify the sweep is clean.** Press **`r`** to reset the span, then squeeze
fully in and out a few times and watch the `span` line:
- ✅ `pos` sweeps **smoothly and monotonically** from ~0 to the range (no jumps).
- ✅ `trigger` spans **~0.00 .. ~1.00**, matching the good handle.
- ✅ **No wrap:** the raw pos must not jump between +π and −π during the squeeze —
  if it does, the zero sits on the encoder's wrap boundary; rotate the magnet so
  the whole squeeze stays in one continuous region.
- ❌ Jumpy / non-linear / jittery → concentricity or air gap still off; redo step 2.

**5. Confirm at rest it's stable.** At rest, `vel ≈ 0` and `pos` shouldn't jitter.
Jitter = weak field (gap too big) or noise.

## Acceptance criteria (matches a good handle)

- Rest: `raw pos` within ~±0.02 rad of 0, `trigger` ≤ 0.05.
- Full squeeze: `trigger` ≥ 0.95.
- Smooth, monotonic sweep with no ±π wrap.

When those hold, the handle reads correctly with **no `invert` flag** — remove any
`invert: true` you'd set as a stopgap.

## If you can't hit ~0 at rest mechanically

If the magnet physically can't be rotated to land rest at 0 (fixed mounting), fall
back to the software options: `end_effectors.leader.invert` (for a fully reversed
handle) or the range tuning `range_rad`. But mechanical alignment is the accurate
fix — a well-centered magnet at the right gap is what gives linear, repeatable
readings.
