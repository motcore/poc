# Motcore v6 — bill of materials, one cube (4 axes)

State of the macro at 2026-09-22 (`cad/motcore_v6_apex_pivot.py`, 31 checks
green). Quantities are **per cube**. The central motor and the electronics are
not included. Parts marked *measure* have provisional dimensions in the macro:
check them on arrival and update the macro.

## Bought

| Qty | Part | Where it goes | Notes |
|---|---|---|---|
| 4 | Bearing 6805 (25×37×7) | output cone's neck | |
| 8 | Bearing MR105ZZ (5×10×4) | output shafts, 2 each | |
| 2 | Bearing 688ZZ (8×16×5) | motor shaft, floor and ceiling | |
| 1 | Ø8 D-shaft, 105 mm | motor shaft | flat 7.5 across — *measure* |
| 4 | Ø5 D-shaft, 35 mm | output shafts | flat 4.5 across — *measure* |
| 4 | Ø4 rod, 53 mm | guide rods | |
| 4 | T8 lead screw, lead 8 (4-start), 62 mm | actuation | stock lengths are 100 mm+: order cut, or cut |
| 1 | T8 anti-backlash kit, CESFONJER (4 sets) | the rectangular-flange nut only | flange holes likely M3 — *measure* |
| 4 | Servo MG90D | actuation | comes with its horns, horn screw (M2.5) and 2 tab screws |
| 4 | T8 lock collar (grub type, ~Ø14×6) | lead screw thrust stop | *measure* |
| 8 | PTFE washer 8×12×1 | lead screw thrust stop | brass + grease also works |
| 8 | Spring stack Ø11/Ø5, ~72 N/mm | series spring, 2 per axis | **part not chosen** (Belleville or die spring) |
| 8 | Dowel ISO 8734 4×24 | four-bar pins A | |
| 8 | Dowel ISO 8734 4×40 | four-bar pins B | |
| 4 | Dowel ISO 8734 3×14 | ear pins | |
| 32 | Dowel Ø2 | cardan cross pins, 8 per axis | lengths still to match to stock |
| ~32 | Shim DIN 988 4×8, 0.1–0.5 mm | four-bar thrust faces | an assortment |
| 8 | M3×10 socket head (DIN 912) | nut flanges | |
| 16 | M3×12 socket head (DIN 912) | frame posts, from outside the decks | |
| 16 | M3×10 countersunk (DIN 7991) | walls to ceiling and floor | |
| 10 | M3 grub screw, ~4 mm | 4 servo couplers + 6 dogs | |
| 24 | Disc magnet N52 10×2 | 4 per face | same pole on the 5 output faces, the other on the input |
| 1 | Rubber sheet, 2 mm | 6 bands: 2 motor cones, 4 output cones | material still open (§10) |

## Printed

| Qty | Part | Material |
|---|---|---|
| 2 | Motor cone (same part, flipped) | PLA |
| 2 | Motor shaft spacer (top 22 mm, bottom 7 mm) | PLA |
| 4 | Wall (servo cradle, guide rod foot and thrust plate built in) | PLA |
| 1 | Ceiling (4 screw-top posts, ribs) | PLA |
| 1 | Floor (ribs, input socket) | PLA |
| 4 | Output cone (shell) | PLA |
| 4 | Carriage | PLA |
| 8 | Link (each carries both arms) | PLA |
| 8 | Frame post (screwed to the decks) | PLA |
| 4 × 4 | Cardan: UJMid, UJRing, UJCross, UJFork | PLA |
| 4 | Spring cage | PLA |
| 4 | Spring carrier | PLA |
| 4 | Servo-to-screw coupler | PLA |
| 5 | Male dog (4 with a D5 bore, 1 with a D8) | PLA or PETG |
| 1 | Female dog, D8 | PLA or PETG — the cube's torque cap |
| 1 | Spider | TPU |

## Not included yet

- Central motor and its male dog adapter.
- AS5600 per output shaft (4), a TCA9548A (fixed I2C address 0x36), controller.
- Calibration coupon (`cad/calibration.py`), to print first.

## Measure on arrival

The D flats (both shafts), the lock collar, the nut's flange holes, the servo
horn (hub Ø7 × 4, arm 5 × 2 as measured by eye), and the rubber's stiffness,
which every torque figure depends on.
