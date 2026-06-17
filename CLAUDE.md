# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis friction drive actuator**.
One central motor (Z axis, vertical) drives multiple output axes (±X, ±Y) via
servo-controlled **O-ring friction clutches**. Each output axis has its own
clutch module; engaging a clutch connects that axis to the rotating motor disc.

All mechanical design files are in `cad/`. Firmware in `software/`.

---

## Physical layout

- **Motor shaft**: vertical (Z axis), at the centre of the cube.
- **Motor disc**: flat horizontal disc (like a plate) on the motor shaft.
  Both faces (top and bottom) are contact surfaces.
- **Output axes**: 4 horizontal shafts in a cross pattern (±X, ±Y), one per cube wall.
- **Each output shaft** pivots around a UJ point near its wall and tilts ±A to engage
  the top face (+) or bottom face (−) of the motor disc.
- **Cube size**: ~116 × 116 mm (derived from B and R, see equations below).

```
        top view              side view (one axis)

         [+Y]                   motor shaft (Z)
          │                          │
  [−X] ──●── [+X]        ┌──────────●──────────┐  ← top disc face
          │               │       ↗              │
         [−Y]             │  output shaft (tilt A)
                          │                      │
                          └──────────────────────┘  ← bottom disc face
```

---

## Clutch mechanism — O-ring design

Each output axis clutch works as follows:

- The **motor disc** is a smooth flat disc (PETG or metal) on the central Z shaft.
  It rotates continuously. No groove, no O-ring on the disc.
- The **output shaft** carries a **wheel** with an O-ring in a toroidal groove on its rim.
- The shaft **tilts ±A around a pivot near the wall** to press the O-ring against the
  top (+A) or bottom (−A) face of the motor disc → torque in either direction.
- **Engagement actuation**: a servo drives the tilt through a **compliant engagement
  blade** printed as one piece with the shaft bracket. The blade has a thin flexible
  (spring) section near the top and a rigid arm below ending in a slot; a pin on the
  short servo arm rides that slot. Servo position → blade deflection → normal force, so
  the spring decouples servo position from reaction forces while the shaft spins.
- **Over-centre latch**: the servo arm sweeps just past its 90° dead point to a hard
  stop at the slot end. There the spring presses the pin against the stop, so the clutch
  self-holds at full engagement **even with the servo unpowered** — the servo only draws
  current during transitions.
- **Torque control** (proportional regime, before the latch): more servo travel → more
  blade deflection → more normal force → more friction torque transmitted.

The bevel cone geometry is **superseded**. Differential slip is not a concern with
point/line contact.

### Two design branches

| Branch | File | Tilt pivot | Compliance | Status |
|--------|------|------------|------------|--------|
| **Compliant lever** | `cad/motcore_compliant_lever.py` | purchased metal UJ (Ø11×23, 5 mm bore) | in the engagement blade | **ACTIVE** |
| Solid | `cad/motcore_v1.py` | printed compliant neck in the shaft | in the shaft neck | parked |

The **active** design uses **metal output shafts** (two Ø5 mm segments split at the UJ);
the tilt compliance lives in the engagement blade, not the shaft. The parked solid branch
instead put a flexible neck in the printed shaft and used a separate pinned lever — kept
as a fallback.

---

## Clutch geometry — key parameters

| Symbol | Default | Description |
|--------|---------|-------------|
| **A**  | 1.5°    | Engagement angle — shaft tilt from horizontal (small, to keep the neutral gap from eating the short servo-arm stroke) |
| **B**  | 35 mm   | UJ pivot → output wheel centre (along shaft) |
| **R**  | 20 mm   | Motor disc radius (rim) |
| **dw** | 2.5 mm  | O-ring wire diameter |

Derived (active branch, A=1.5°):

| Symbol       | Formula                  | Value |
|--------------|--------------------------|-----------------|
| WT           | 3.2 × dw                 | 8.0 mm  — wheel & disc thickness |
| R_out        | wc_dist − WT/2 − gap     | 15.0 mm — **actual contact radius** (set by the square no-overlap condition between adjacent wheels, so < R) |
| Rw           | R_out − 0.4 × dw         | 14.0 mm — structural wheel radius |
| contactY     | B·cosA − R·sinA          | 34.5 mm — contact point Y from UJ |
| contactZ     | B·sinA + R_out·cosA      | 15.9 mm — contact point Z from UJ |
| motorY       | contactY + R             | 54.5 mm — motor shaft distance from UJ |
| cube_half    | motorY + wall_thick      | 58.5 mm — half cube side (cube ≈ 117 mm) |
| disc_height  | 2 × contactZ             | 31.8 mm — motor disc total height |
| transmission | R_out / R                | ≈0.75 (ω_out ≈ 1.34 × ω_motor) — no longer 1:1 |

---

## Coordinate system (2D visualiser, Y-Z cross-section)

- **UJ pivot** = origin (0, 0)
- **Y axis** = horizontal, pointing from UJ toward motor shaft
- **Z axis** = vertical, upward (= motor shaft direction)
- Output shaft at angle θ ∈ [−A, +A] from horizontal

Contact point at full engagement (matches `clutch_geometry_v3.html` and the macro):
```
contactY = B·cosA − R·sinA       (uses R — fixes the motor shaft position)
contactZ = B·sinA + R_out·cosA   (uses R_out — where the O-ring actually meets the
                                  disc face; top engagement, bottom = −contactZ)
```

Motor shaft at:
```
motorY = contactY + R
```

The output wheel contacts at R_out (< R) to avoid overlap between adjacent wheels, so
the contact Z uses R_out while the motor shaft placement still uses R.

## Coordinate system (3D FreeCAD macro)

- **Origin** = cube centre = motor shaft axis at mid-height
- **Z** = motor shaft, upward
- **X, Y** = output shaft directions
- UJ for each axis at distance `motorY` from origin along its axis direction

---

## Governing equations

### Normal force at contact

The servo pin pushes the engagement blade with force F; a moment balance about the
UJ tilt axis converts that into the contact normal force through the blade leverage:

```
N = F · (pin_arm / contact_arm)
```

where `pin_arm` is the pin height above the tilt axis and `contact_arm` the wheel
contact offset from it (≈1.9× with current geometry). F itself comes from the blade
spring stiffness × deflection (set by servo angle), **not** directly from servo torque —
the spring decouples the two. Servo torque is not the limiting factor (peak demand
≈ 23 N·mm, far below any micro-servo).

### Transmitted torque

```
T_out = μ · N · R_out
```

where R_out ≈ 15 mm is the **actual** contact radius and μ ≈ 0.6 (conservative) to
0.9 for silicone O-ring on PETG/metal. At the over-centre latch this gives
T_out ≈ 250 N·mm ≈ 2× the servo's continuous torque — i.e. the rig transmits far more
torque than the servo itself produces, which is the whole point of Motcore.

### Torque limiter

The friction clutch is also a **per-axis, software-adjustable torque limiter**: it slips
(protecting shafts and gears) above μ·N·R_out, and N is set by servo position, so each
axis can cap its slip torque anywhere from ~0 up to the latched maximum.

### O-ring groove dimensions

| Dimension    | Formula      | Value (dw=2.5mm) |
|--------------|--------------|-----------------|
| Groove depth | 0.6 × dw     | 1.5 mm          |
| Groove width | 1.2 × dw     | 3.0 mm          |
| O-ring protrusion beyond rim | 0.4 × dw | 1.0 mm |
| Wheel rim radius (Rw) | R_out − 0.4 × dw | 14.0 mm |

O-ring centre is 0.1 × dw (0.25 mm) below the wheel rim — protrudes 0.4 × dw outward.
Wheel face never contacts motor disc (clearance 0.15–0.2 × dw at 20–25% compression).

---

## Design invariants — do not violate these

1. **O-ring on output wheel only.** Motor disc is smooth — no groove, no O-ring.
2. **Disc must span both contact faces** — disc height ≥ 2 × contactZ so top and
   bottom engagements both work.
3. **Disc covers the contact point** — with defaults R=20 < contactY=34.5, so the disc
   visual radius is extended to disc_vr = R + WT/2; the geometric contact is at the rim
   (motorY − R = contactY). Satisfied by construction.
4. **O-ring material: silicone** — higher μ and better thermal tolerance than NBR.
5. **Disc material: PETG minimum** — PLA for prototyping only.
6. **Clearance in neutral** — at θ = 0 the O-ring must not graze either disc face.

---

## Mechanical design decisions (active — compliant lever)

- **Tilt pivot**: purchased **metal universal joint** (Ø11 × 23 mm, 5 mm bore each side),
  straddling the wall. Replaces the printed pin/neck of the parked branch.
- **Output shaft**: **metal**, Ø5 mm, in two segments split at the UJ. Not printed, no
  flexible neck — so shaft shear is never the limiting factor.
- **Output wheel**: printed, press-fit / fixed to the metal shaft (the shaft↔wheel
  joint detail is still TBD). Carries the O-ring groove.
- **Engagement blade**: printed in one piece with the wall + bracket. Thin flexible XZ
  spring (14 × 4.5 mm) sized so the **sustained latch stress stays ≈ 14 MPa**, keeping
  PETG stress relaxation (creep) negligible.
- **Servo**: **MUST be positional** (≈180°, controls position) — e.g. MG90S or a
  *positional* MG90D, metal gear preferred. ⚠️ **NOT a 360° continuous-rotation servo**
  (those control speed, not position, and break the proportional control + latch). Torque
  is not a selection constraint (peak demand ≈ 23 N·mm); choose for precision/durability.
- **Servo arm**: short (r = 4 mm) so the pin sweeps to an over-centre latch within a
  short slot at low sustained spring stress.
- **Module per wall**: each wall carries one complete clutch module, removable as a unit
  (modular foot, M3 screws with nut traps in the wall side-bars).
- **Transmission ratio**: ≈0.75 : 1 (contact at R_out ≈ 15 mm on the wheel vs R = 20 mm
  on the disc), so ω_out ≈ 1.34 × ω_motor — not 1:1.

---

## Files

| File | Purpose |
|------|---------|
| `cad/motcore_compliant_lever.py` | FreeCAD macro — **active** design (metal UJ + compliant engagement blade + over-centre latch) |
| `cad/motcore_v1.py` | FreeCAD macro — solid branch (printed compliant neck), parked fallback |
| `cad/clutch_geometry_v3.html` | Interactive 2D visualiser (Y-Z cross-section) |
| `cad/motcore_plates.py` | FreeCAD macro — bevel cone design (superseded) |
| `cad/motcore_animate.py` | FreeCAD macro — bevel cone animation (superseded) |
| `software/controller/` | Arduino firmware — master (touchscreen UI) |
| `software/driver/` | Arduino firmware — receiver (motor + servo control) |

Web: `motcore.github.io/clutch-geometry.html` — live visualiser.

---

## Claude coding conventions

- **Language**: all code (comments, variable names, docstrings) and all git commit messages must be in **English**.
- Respond to the user in whatever language they use; only the code and commits must be English.
