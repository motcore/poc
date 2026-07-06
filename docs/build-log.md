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
| Creality Hi | PLA | **10.3** press / 10.4 float | _pending_ | _pending_ | 2026-06-17 |

Coupon: `cad/calibration.py` → `cad/stl/motcore_calibration.stl`. Once measured,
these offsets get set in `cad/motcore_compliant_lever.py`.

- **Bearing**: a hole modelled at 10.3 gives a good press for the Ø10 MR105 (holds
  when flipped, out with a firm tap); 10.2 was too brutal for brittle PLA. → the
  printer runs **~0.2–0.3 mm undersize**, so all bearing seats (were 10.0–10.05 for
  SLS) must go up to ~10.3. Shaft + pilot pending (parts not arrived yet).

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

## Journal

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
