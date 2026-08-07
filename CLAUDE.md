# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis actuator tree**. One central motor
(Z axis, vertical) drives multiple output axes through **vertical conical
friction clutches**. Each output axis has its own clutch module; engaging a
clutch connects that axis to the rotating motor cone.

Each output feeds the **next hub of the tree**, so every level is a
**reduction**, never 1:1 — torque has to be regenerated at each stage, and
speed is the resource there is plenty of.

All mechanical design files are in `cad/`. Firmware in `src/`.

---

## Physical layout

- **Motor cone**: on the central vertical shaft (Z), fixed height, rotating
  continuously.
- **Output cone**: mounted on a **carriage that translates vertically** on two
  **3 mm guide rods**. No universal joint, no shaft tilt, no pivot.
- **Output shaft**: horizontal, fixed, perpendicular to the motor axis.
  Apexes of the two cones are (nominally) common.
- **Contact**: interleaved **rubber O-rings** on both cones, meeting
  **flank to flank** — rubber on rubber, never touching the plastic.
- Actuation is a single degree of freedom: the vertical position of the carriage.

```
        side view (one axis, Y-Z cross-section)

           motor axis (Z)
                │
                │        ┌───┐  ← output cone on carriage
                │       ╱     ╲       (translates vertically ↕)
                │      │  ○ ○  │ ── output shaft (fixed)
              ╱ │ ╲     ╲     ╱       + internal corona
             ╱  │  ╲     └───┘
            ╱ ○ │ ○ ╲   ← O-rings interleaved, flank to flank
           ╱────┴────╲
             motor cone
```

---

## Clutch mechanism — v5, vertical cone

- The **motor cone** carries `n` O-rings seated on its generatrix; the
  **output cone** carries `n − 1`, offset by half a pitch so the two sets
  **interdigitate**.
- Torque passes **rubber against rubber** (μ ≈ 1.2–1.5) instead of rubber on
  plastic (μ ≈ 0.6–0.9).
- **Preload is radial, between flanks.** That decouples it from apex
  displacement — pushing harder does not need the cones to move axially into
  each other.
- **Gear stage**: a **pinion rigid to the output cone** lives **permanently
  inside an internal corona (ring gear) fixed to the output shaft**. The
  pinion never enters or leaves the ring — it is captive by construction.
- **Free = pinion concentric with the corona** (centre distance 0). The output
  shaft carries nothing at all → **real free rotation**, not a friction
  threshold.

### Four carriage positions, descending

| # | Position | State |
|---|----------|-------|
| 1 | **Free**     | pinion concentric, rings separated, output shaft drives nothing |
| 2 | **Mesh**     | correct centre distance `e` reached, rings **still separated**, relative velocity zero → teeth engage without shock |
| 3 | **Contact**  | rubber flanks touch, normal force zero |
| 4 | **Preload**  | flanks compressed, torque transmitted |

The preload travel is absorbed by the gear's own **root clearance**
(`0.25 · m`). **No slot, no floating ring, no compliant blade.**

---

## Key parameters

| Symbol | Default | Description |
|--------|---------|-------------|
| **α**  | 55°   | motor cone half-angle (from the vertical axis) |
| **m**  | 1.0   | gear module |
| **Zp** | 14    | pinion teeth (on the output cone / carriage) |
| **Zc** | 28    | corona teeth (on the output shaft) |
| **L**  | 18 mm | contact line length along the generatrix |
| **s₀** | 10 mm | apex → start of the contact line |
| **d**  | 2.5 mm| O-ring wire diameter |
| **n**  | 5     | rings on the motor cone (output carries n − 1) |
| **g**  | 0.20 mm | margin: full mesh → rubber contact |
| **δ**  | 0.25 mm | preload travel past contact |

Derived at defaults:

| Quantity | Value |
|----------|-------|
| friction ratio | 1.428 |
| gear ratio     | 0.500 |
| **total ω_out/ω_motor** | **0.714** (→ 1.4× torque) |
| centre distance `e` | 7.00 mm |
| free float (no tooth touch) | 5.00 mm |
| mesh travel | 2.00 mm |
| **total stroke** | **7.45 mm** |
| ring pitch `q` | 2.25 mm (< d ✓) |
| apex separation | 1.33 mm |
| micro-slip | ≈ 7.4 % |

---

## Governing equations

```
α_out       = 90 − α_motor                  (perpendicular axes, common apex)

ratio_fric  = sin(α_m) / sin(α_o)           independent of position along the
                                            contact line

ratio_gear  = Zp / Zc                       internal mesh, reducing

ratio_total = ratio_fric · ratio_gear

e           = m · (Zc − Zp) / 2             centre distance at mesh
free float  = e − 2m
mesh travel = 2m

q           = L / (2n − 2)                  ring pitch along the contact line
apex sep    = sqrt(d² − q²) / sin(α_m)

stroke      = e + g + δ
```

### Transmitted torque

Torque is set by the radial preload between rubber flanks, with
μ ≈ 1.2–1.5 (rubber on rubber). It is also a **per-axis torque limiter**: it
slips above the preload-set threshold, and the preload is set by carriage
position, so each axis can cap its slip torque in software.

---

## Coordinate system (2D visualiser, Y-Z cross-section)

- **Z** = motor shaft, vertical, upward. Motor axis at Y = 0.
- **Y** = horizontal, pointing from the motor axis toward the wall / output shaft.
- The cross-section is the vertical plane containing both the motor axis and
  the output shaft axis.
- Carriage height reference: **contact = 0**, downward negative.
  `z_free = g + e`, `z_mesh = g`, `z_contact = 0`, `z_preload = −δ`.
- The output shaft axis is fixed at `z = g + e` (so the pinion is concentric
  with the corona in position 1).

---

## Design invariants — do not violate these

1. **Reduction, not 1:1.** The output feeds the next hub of the tree; torque
   must be regenerated at every level. Speed is the surplus resource.
2. **Mesh BEFORE rubber contact, never after.** Teeth must engage at zero
   relative velocity.
3. **`g + δ ≤ 0.25 · m`** (root clearance) — otherwise the pinion jams into
   the corona.
4. **`q < d`**, or the flanks never touch at all.
5. **`Zc − Zp ≥ 8`** — internal-mesh interference.
6. **The AS5600 on every output shaft is structural, not optional.** Friction
   slip accumulates non-repeatably in series along a tree; position is only
   known by measuring it. The I2C address is fixed at **0x36**, so
   multiplexing is required (analog output, PWM, or a TCA9548A).
7. **Rubber on rubber only.** Ring flanks meet each other; they never contact
   the plastic cone surface.

---

## Open questions — do NOT present these as settled

- **Ring density vs micro-slip.** More rings interdigitate better but push the
  cones apart, separating the apexes. At n=5 slip is ≈ 7.4 %, at n=7 ≈ 13.6 %,
  at n=3 the flanks never reach each other. Narrow window — needs a sweep over
  `L` and `d`.
- **`g + δ` exceeds the root clearance at m=1.** Unresolved (invariant 3 is
  currently violated by the defaults; the visualiser flags it).
- **Re-meshing while the output shaft is still turning** (e.g. an arm falling)
  causes tooth clash. Firmware problem — requires a velocity check via the
  AS5600 before engaging.
- **How the rings are manufactured.** Catalogue O-rings fix the diameters, and
  the ring pitches then have to land on the generatrix.

---

## Discarded — do not re-propose

- **Tilting flat disc with a universal joint.** Superseded by the vertical cone.
- **Laboratory conical rubber stoppers** as friction elements.
- **Fixed-threshold friction clutches to obtain free rotation.** Lifting and
  falling demand the same torque, so no threshold separates them. Free
  rotation comes from the concentric-pinion geometry instead.

---

## Files

| File | Purpose |
|------|---------|
| `cad/clutch_geometry_v5.html` | Interactive 2D visualiser — **active**, v5 vertical cone clutch |
| `cad/clutch_geometry_v3.html` | Visualiser for the superseded tilting-disc design |
| `cad/clutch_geometry.html`    | Older visualiser (superseded) |
| `cad/motcore_compliant_lever.py` | FreeCAD macro — tilting-disc branch (superseded) |
| `cad/motcore_v1.py`           | FreeCAD macro — solid tilting branch (superseded) |
| `cad/motcore_plates.py`       | FreeCAD macro — bevel cone design (superseded) |
| `cad/motcore_animate.py`      | FreeCAD macro — bevel cone animation (superseded) |
| `cad/calibration.py`          | FreeCAD macro — FDM tolerance calibration coupon |
| `docs/build-log.md`           | Prototype build log — purchases, prints, calibrations, tests |
| `docs/clutch-geometry.md`     | Clutch geometry notes |
| `src/controller/`             | Arduino firmware — master (touchscreen UI) |
| `src/driver/`                 | Arduino firmware — receiver (motor + servo control) |

Web: `motcore.github.io/clutch-geometry.html` — live visualiser.

---

## Claude coding conventions

- **Language**: all code (comments, variable names, docstrings) and all git
  commit messages must be in **English**.
- Respond to the user in whatever language they use; only the code and commits
  must be English.
