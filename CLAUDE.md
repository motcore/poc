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
- A **servo + spring** tilts the output shaft around the pivot by angle A:
  - **Neutral**: servo at centre → spring relaxed → O-ring clears both disc faces.
  - **Engage**: servo moves → spring compresses → O-ring presses disc face.
  - **Torque control**: more servo travel → more spring compression → more normal
    force → more friction torque transmitted. The spring decouples servo position
    from reaction forces while the shaft is spinning.
  When tilted +A: O-ring presses against **top** disc face → torque in one direction.
  When tilted −A: O-ring presses against **bottom** disc face → torque reversed.
- The **output shaft has two thin cylindrical necks in series** centred at the pivot
  point. Each neck bends only A/2, halving bending stress and improving fatigue life.
  The necks transmit output torque; they are sized for torsional strength, not as
  return springs (the servo+spring handles return to neutral).

The bevel cone geometry is **superseded**. Differential slip is not a concern with
point/line contact.

---

## Clutch geometry — key parameters

| Symbol | Default | Description |
|--------|---------|-------------|
| **A**  | 3°      | Engagement angle — shaft tilt from horizontal |
| **B**  | 35 mm   | UJ pivot → output wheel centre (along shaft) |
| **R**  | 20 mm   | Contact radius (O-ring outer edge = motor disc rim) |
| **dw** | 2.5 mm  | O-ring wire diameter |

Derived:

| Symbol       | Formula              | Value (defaults) |
|--------------|----------------------|-----------------|
| WT           | 3.2 × dw             | 8.0 mm  — wheel & disc thickness |
| Rw           | R − 0.4 × dw         | 19.0 mm — structural wheel radius |
| contactY     | B·cosA − R·sinA      | 33.9 mm — contact point Y from UJ |
| contactZ     | B·sinA + R·cosA      | 21.8 mm — contact point Z from UJ |
| motorY       | contactY + R         | 53.9 mm — motor shaft distance from UJ |
| cube_half    | motorY + wall_thick  | ~58 mm  — half cube side |
| disc_height  | 2 × contactZ         | 43.6 mm — motor disc total height |

---

## Coordinate system (2D visualiser, Y-Z cross-section)

- **UJ pivot** = origin (0, 0)
- **Y axis** = horizontal, pointing from UJ toward motor shaft
- **Z axis** = vertical, upward (= motor shaft direction)
- Output shaft at angle θ ∈ [−A, +A] from horizontal

Contact point at full engagement:
```
contactY = B·cosA − R·sinA
contactZ = B·sinA + R·cosA   (top engagement; bottom = −contactZ)
```

Motor shaft at:
```
motorY = contactY + R
```

## Coordinate system (3D FreeCAD macro)

- **Origin** = cube centre = motor shaft axis at mid-height
- **Z** = motor shaft, upward
- **X, Y** = output shaft directions
- UJ for each axis at distance `motorY` from origin along its axis direction

---

## Governing equations

### Normal force at contact

The servo compresses a spring (or bends a flexible neck) to tilt the shaft by A.
The effective normal force pressing the O-ring against the disc:

```
N = τ_servo / B
```

where τ_servo is the torque around the UJ pivot. The servo SG90 applies
full torque to reach the commanded position; spring stiffness controls N.

### Transmitted torque

```
T_out = μ · N · R
```

where R is the contact radius and μ ≈ 0.7–0.9 for silicone O-ring on PETG/metal.

### O-ring groove dimensions

| Dimension    | Formula      | Value (dw=2.5mm) |
|--------------|--------------|-----------------|
| Groove depth | 0.6 × dw     | 1.5 mm          |
| Groove width | 1.2 × dw     | 3.0 mm          |
| O-ring protrusion beyond rim | 0.4 × dw | 1.0 mm |
| Wheel rim radius (Rw) | R − 0.4 × dw | 19.0 mm |

O-ring centre is 0.1 × dw (0.25 mm) below the wheel rim — protrudes 0.4 × dw outward.
Wheel face never contacts motor disc (clearance 0.15–0.2 × dw at 20–25% compression).

---

## Design invariants — do not violate these

1. **O-ring on output wheel only.** Motor disc is smooth — no groove, no O-ring.
2. **Disc must span both contact faces** — disc height ≥ 2 × contactZ so top and
   bottom engagements both work.
3. **R ≥ contactY** — motor disc radius must cover the contact point Y position.
   With defaults: R=20 < contactY=33.9 → the disc visual radius is extended to
   disc_vr = R + WT/2 for aesthetics; the geometric contact is at the left rim
   (motorY − R = contactY). This invariant is satisfied by construction.
4. **O-ring material: silicone** — higher μ and better thermal tolerance than NBR.
5. **Disc material: PETG minimum** — PLA for prototyping only.
6. **Clearance in neutral** — at θ = 0 the O-ring must not graze either disc face.

---

## Mechanical design decisions (v1 prototype)

- **UJ pivot**: printed plastic + M3 pin. No bearing needed (oscillatory motion ±3°).
- **Output shaft**: printed, with a **flexible neck** (thin section near UJ) as the
  elastic return element — avoids a separate return spring for v1.
- **Output wheel**: same shaft print or press-fit. Carries the O-ring groove.
- **Servo**: SG90 or equivalent. Controls position, not torque. Spring between servo
  and shaft converts position → normal force.
- **Module per wall**: each wall carries one complete clutch module (shaft + wheel +
  servo + lever arm), designed to be removed and replaced as a unit (2× M3 screws).
- **Transmission ratio**: 1:1 (contact at radius R on both wheel and motor disc centre).

---

## Files

| File | Purpose |
|------|---------|
| `cad/motcore_v1.py` | FreeCAD macro — O-ring clutch geometry (current) |
| `cad/clutch_geometry_v3.html` | Interactive 2D visualiser (Y-Z cross-section) |
| `cad/motcore_plates.py` | FreeCAD macro — bevel cone design (superseded) |
| `cad/motcore_animate.py` | FreeCAD macro — bevel cone animation (superseded) |
| `software/controller/` | Arduino firmware — master (touchscreen UI) |
| `software/driver/` | Arduino firmware — receiver (motor + servo control) |

Web: `motcore.github.io/clutch-geometry.html` — live visualiser.
