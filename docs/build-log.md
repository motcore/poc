# Motcore — Build Log (Prototype v0.1)

Running journal for a **specific physical build**: purchases, prints, calibrations,
tests and hands-on decisions. Major geometry changes live in git; this is the
day-to-day record that git doesn't capture. **Newest entries at the top.**

---

## Bill of materials (this prototype)

Status: ✅ have · 🛒 ordered · ⬜ to buy

| Item | Spec | Qty | Status | Notes |
|------|------|-----|--------|-------|
| Servo | positional ~180° (NOT 360° continuous) | 1+ | ✅ | verify it's the positional one |
| Flange hub Ø5 | flange Ø22, bolt circle Ø16, 4×M3 | 3 (test) / 6 (cube) | ✅ | 4 wheels + 2 disc |
| Bearing | MR105ZZ (Ø5×Ø10×4) | 4 (test) / 10 (cube) | ✅ | one type everywhere |
| Universal joint | Ø11×23, 5 mm bore | 1 / 4 | 🛒 | ordered |
| 3D printer | Creality Hi (without CFS) | 1 | ✅ | |
| Filament dryer | Creality Space Pi | 1 | ✅ | for PETG/Nylon |
| Filament | Hyper PETG 1.75 mm 1 kg, white | 1 | ✅ | functional parts |
| Filament | PLA | 1 | ⬜ | fast fit-check iteration |
| Shaft rod | 6061 aluminium Ø5, 305 mm | 2 | ✅ | soft — OK for test; steel for final |
| Shaft collar | Ø5 bore, Ø10, 5 mm, set screw | 2 | ✅ | comes with grub screw |
| O-ring | 30×25×2.5 (OD×ID×CS), NBR | pack | ✅ | NBR for test; silicone for final |
| Screw kit | M2/M3 socket-head + nuts + washers | 1 | ⬜ | |
| Threadlocker | Loctite 243 (blue) | 1 | ⬜ | on all set screws |
| Allen keys | hex key set | 1 | ⬜ | for socket screws |
| Drive motor | external, spins the central shaft | 1 | ⬜ | drill or DC motor for the test |
| Servo control | servo tester or Arduino + 5–6 V | 1 | ⬜ | firmware in software/ |

---

## Printer calibration

| Printer | Material | Bearing (Ø10) | Shaft (Ø5 slip) | M3 pilot | Date |
|---------|----------|---------------|-----------------|----------|------|
| Creality Hi | PLA | **10.3** press / 10.4 slip | **5.8** (disc bore 5.6) | **3.1** | 2026-07-07 |

Applied to `motcore_compliant_lever.py` (FDM block): pilots 3.1, disc bore 5.6,
bearing seats press +0.15 (Ø10.3) / slip +0.20 (Ø10.4). Shaft pass-through holes
were already Ø5.8 → left as-is. **Estimated (not directly measured, verify on first
assembly)**: wall-in-groove width 4.8 and peg/socket Ø7.0 (printed male-in-female
fits — forgiving; sand if tight, shim if loose).

Coupon: `cad/calibration.py` → `cad/stl/motcore_calibration.stl`. Once measured,
these offsets get set in `cad/motcore_compliant_lever.py`.

- **Bearing**: a hole modelled at 10.3 gives a good press for the Ø10 MR105 (holds
  when flipped, out with a firm tap); 10.2 was too brutal for brittle PLA. → press
  seat ≈ **10.3**, float ≈ 10.4 (to confirm on v2 coupon).
- **Shaft (v1)**: Ø5 rod only entered the biggest hole (5.5) and even that was zero
  clearance + squeak → **small holes run ~0.5 mm undersize**. Free slip is > 5.5.
- **Pilot (v1)**: M3 didn't self-tap in any (2.3–2.7 → printed ~1.8–2.2, too small).
- → **coupon v2** with shifted ranges (bearing 10.2–10.6, shaft 5.6–6.4, pilot
  2.8–3.2) to nail shaft slip + M3 pilot + bearing float. Reprint & measure.
- Note the disc centring bore (5.0 in the macro) would print ~4.5 → the rod won't
  fit; all shaft holes need bumping up in the FDM tolerance pass.

---

## Print log

| Date | Part | Material | Notes / result |
|------|------|----------|----------------|
| 2026-06-17 | calibration coupon | PLA | printed; measuring |

---

## Test log

| Date | What | Result |
|------|------|--------|
| — | — | — |

---

## Backlog — issues found on the first PLA build (2026-07-07)

| # | Issue | Type | Plan |
|---|-------|------|------|
| 1 | O-ring loose in the wheel groove (wobbles) | model | ✅ stretch 1%→3% + dovetail groove |
| 2 | O-ring slips (rotates) in its groove — needs grip | model | ✅ **dovetail groove** (revolved trapezoid, bottom 3.25 > opening 2.62) keys the O-ring; verify the revolve + ~16° undercut print in FreeCAD; glue for the current printed wheel |
| 3 | Flexure blade roots need progressive material (fillets) on both sides | model | ✅ tilt blades: 45° gussets both ends + both faces. Engagement-spring (servo lever) root fillets tried then removed — not needed |
| 3b | Head bearing easier to install from the wheel side | model | ✅ head bearing seat now opens toward the wheel (head_y0) instead of the wall side |
| 4 | Bracket supports clogged the holes | print | tree supports / support blockers on holes / reorient; drill out for now |
| 5 | Ø5 aluminium rod won't enter the bearing | hardware | file/sand to fit; consider **mild steel SMOOTH** rod (NOT threaded — a threaded rod rides on its thread tips in a bearing) |
| 6 | Bearing too loose in the frame-base boss | model | the tall boss prints looser than the flat coupon → tighten the plate-boss bearing fit (per-feature) |
| 7 | Wall internal overhangs (foot pocket / bracket seating faces) rough — printed without support | print/model | support there, or chamfer those overhangs < 45° so they print clean |

## Journal

### 2026-07-07
- Printed frame base + top: spotted a **step** where the wall rests — the column
  groove's inner wall didn't line up with the base/top backing rib, and it varied
  per face. Root cause: the groove was positioned asymmetrically for ±sx/±sy
  (0.1 mm on +, ~0.7 mm on −). Fixed: groove inner wall now sits exactly at
  cube_half (= rib plane) on all four faces, clearance moved to the outer side.
  Already-printed frame has the step — usable for the test; reprint later for a
  clean assembly.
- Printed the wall (PLA) — spotted that the M3 **clearance** holes were missed in
  the FDM pass: modelled Ø3.2 → printed ~2.7 → the screw taps the wall instead of
  passing through. Bumped `m3_screw_r` 1.6 → 1.9 (Ø3.8 → ~3.3 free). Already-printed
  wall: drill out to ~3.3 or use as-is. Fix applies to wall/foot/top-plate M3 holes.
- Coupon v2 measured: shaft free slip ≈ 5.8, M3 pilot 3.1, bearing press 10.3.
  Confirmed the printer runs small holes ~0.5 mm undersize.
- Applied the **FDM tolerance pass** to the macro (pilots 3.1, disc bore 5.6,
  bearing seats 10.3/10.4). Peg/socket + wall-groove clearances estimated — to
  verify on the first assembly print. Ready to export the real parts.

### 2026-06-17
- Printed the calibration coupon (PLA) and measured the **bearing row**: 10.3 is a
  good press seat (printer ~0.2–0.3 mm undersize). Shaft/pilot pending (rod + M3
  screws not arrived). Macro fits NOT changed yet — waiting for all three.
- Bought Creality Hi + Space Pi dryer + Hyper PETG 1 kg (white). Coupon printing.
- Decided: PLA for fit-check iteration, PETG for functional test, SLS PA12 for the
  final bracket if needed.
- Bought aluminium Ø5 rod (soft — set-screw/bearing durability lower than steel;
  fine for the first test). O-rings 30×25×2.5 NBR. Shaft collars Ø5 (set-screw).
- Tuned the wheel groove to the real O-ring (ID 25) in the macro.
- Still to source: M2/M3 screw kit, Loctite 243, Allen keys, a motor to spin the
  disc, and a way to command the servo.
