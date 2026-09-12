# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis actuator tree**. One central motor
(Z axis, vertical) drives multiple output axes through **conical friction
clutches**. Each output axis has its own clutch module; engaging a clutch
connects that axis to the rotating motor cone.

Each output feeds the **next hub of the tree**, so every level is a
**reduction**, never 1:1 — torque has to be regenerated at each stage, and
speed is the resource there is plenty of.

The target application is **passive-dynamic walkers** (a rocking biped, a
bouncing quadruped), which changes what matters: a genuinely free "free" state,
back-drivability and low reflected inertia are features, not compromises. See
`docs/clutch-geometry-v6.md` §2 and the open questions.

All mechanical design files are in `cad/`. Firmware in `src/`.

---

## Physical layout

- **Two motor cones** on the central vertical shaft (Z), fixed in height,
  rotating continuously, flaring away from a common apex at the origin.
- **Output cone**: ONE per axis, sitting in the waist, on a carriage that
  **pivots ±2° about that common apex**. The carriage hangs on a **four-bar
  linkage** whose link axes converge on the apex, so the pivot is virtual —
  nothing physical sits at the apex, which is just as well because the motor
  shaft is there.
- **Output shaft**: horizontal, fixed, perpendicular to the motor axis at rest,
  **centred on the wall**.
- **Contact**: a **continuous rubber layer** on both cones, meeting along the
  shared generatrix — rubber on rubber, never plastic.
- Actuation is a single degree of freedom: the carriage's tilt angle.
- **Four axes**, one per cube wall.

```
        side view (one axis, Y-Z cross-section)

           motor axis (Z)
            ╲   │   ╱
             ╲  │  ╱       ← upper motor cone (apex down)
              ╲ │ ╱
               ╲│╱
                ◉ ← ONE common apex = the pivot (virtual)
               ╱│╲
              ╱ │ ╲        ← lower motor cone (apex up)
             ╱  │  ╲
            ╱   │   ╲

                 ┌────┐ ← output cone, pivots ±2° about the apex
                ╱      ╲      tilt UP   → output turns one way
               │   ○    │ ─── tilt DOWN → the other way
                ╲      ╱      middle    → free
                 └────┘       + pinion, idlers, corona (all meshed, always)
```

### Why two cones: one axis, both directions

Both motor cones turn with the motor, always the same way — but their flanks
face **opposite sides of the output axis**. Tilting the carriage one way brings
the output cone onto the lower motor cone; the other way onto the upper one, and
the output shaft then turns **the other way**. One motor, one axis, both
directions, without ever reversing the motor.

This is why **free is the middle** of the travel rather than one end.

---

## Clutch mechanism — v6 "Apex Pivot"

Full derivation, numbers and rejected alternatives: **`docs/clutch-geometry-v6.md`**.
Read that before proposing changes. Summary:

- **The apex is the hinge.** It never moves, in any position. Preload therefore
  acts exactly along the contact normal (100%, against 87% for v5's vertical
  push) and adds zero apex error.
- **Continuous rubber layer**, thickness `t`, replacing v5's interleaved
  O-rings. The **plastic** apexes are pulled back by `t / sin(half-angle)` so
  the **rubber** surfaces are what converge on the origin. Micro-slip ≈ 0
  (v5: 7.4%).
- **The gear stage never disengages.** Free rotation comes from the rubber
  separating, not from the gears letting go; the residual drag is the cone's
  reflected inertia, ~2% of a leg's, with no torque threshold.
- **Idler gears centre the output shaft.** Pinion (8t, on the carriage) →
  idlers (8t, fixed axes, **left and right in X, never up/down**) → corona
  (24t, output). Ratio 1/3, output on the wall centre.
- **Actuation**: servo → crank → telescopic link with two springs inside → pin
  on the carriage's bearing housing, ~46 mm from the apex. No slot, no external
  guide. The springs make the preload force-controlled rather than
  position-controlled.

### Three carriage positions — symmetric, both ways from the middle

The angle `φ` is measured from the mid position. There is **no mesh step**: the
gears are always engaged.

| # | Position | \|φ\| | State |
|---|----------|-------|-------|
| 1 | **Free**     | `0`        | rubber separated, output shaft drives nothing |
| 2 | **Contact**  | `φc`       | rubber surfaces touch, normal force zero |
| 3 | **Preload**  | `φc + δφ`  | rubber compressed, torque transmitted |

`φc = 90° − α − β` — the free gap angle **is** the cone geometry. That coupling
is the central fact of v6: you cannot open the travel without thinning the
output cone, which is why the concentric pinion does not fit (see the doc, §9).

---

## Key parameters

| Symbol | Default | Description |
|--------|---------|-------------|
| **α**  | 55°   | motor cone half-angle (from the vertical axis) |
| **β**  | 33°   | output cone half-angle (from its own axis) |
| **t**  | 2.0 mm | rubber layer thickness |
| **L**  | 18 mm | contact line length along the generatrix |
| **s₀** | 10 mm | apex → start of the contact line |
| **m**  | 1.0   | gear module |
| **Zp** | 8     | pinion teeth (on the carriage) |
| **Zi** | 8     | idler teeth |
| **Zc** | 24    | corona teeth (on the output shaft) |
| **R_push** | 46 mm | apex → servo push point (bearing housing) |
| **δφ** | ~0.6° | preload rotation past contact |

Derived at defaults:

| Quantity | Value |
|----------|-------|
| free gap angle `φc` | **2°** |
| friction ratio | 1.503 |
| gear ratio | 0.333 |
| **total ω_out/ω_motor** | **0.501** (→ ×2.0 torque) |
| plastic apex offset, motor cones | 2.44 mm each |
| plastic apex offset, output cone | 3.67 mm |
| micro-slip from the ring/layer geometry | ≈ 0 (v5: 7.4%) |
| residual apex drift from the four-bar | 0.22 mm at full preload ≈ 1% slip |
| lever (R_push / mean rubber distance) | ≈ ×2.4 |
| output torque per newton of actuator force | ≈ 0.098 Nm/N **(upper bound, unverified)** |

---

## Governing equations

```
φc          = 90 − α − β                   free gap angle = the cone geometry

apex offset = t / sin(half-angle)          plastic pulled back so the RUBBER
                                           surfaces share the apex

travel(p)   = distance(p, apex) · φ        every point moves by its own radius —
                                           this is what makes v6 unlike v5

ratio_fric  = sin α / sin β
ratio_gear  = Zp / Zc                      idlers do not change the ratio
ratio_total = ratio_fric · ratio_gear      must stay < 1

ΣN          = F · R_push / s̄               normal force from actuator force
T_out       ≈ μ · F · R_push · sin β · (Zc/Zp)
```

### Transmitted torque

Set by the preload between rubber layers, μ ≈ 1.2–1.5 (rubber on rubber). It is
also a **per-axis torque limiter**: it slips above the preload-set threshold, and
the preload is set by servo angle against the series spring, so each axis caps
its own slip torque. Slipping also makes the joint back-drivable, which the
passive-dynamics goal wants.

---

## Coordinate system (Y-Z cross-section)

- **Z** = motor shaft, vertical, upward. Motor axis at Y = 0.
- **Y** = horizontal, from the motor axis toward the wall / output shaft.
- **Origin = the common apex = the pivot.** Everything is measured from it.
- Carriage tilt `φ`: 0 is free, negative engages the lower motor cone, positive
  the upper one.
- The output shaft axis lies along Y through the origin (centred on the wall).

---

## Design invariants — do not violate these

1. **Reduction, not 1:1.** The output feeds the next hub of the tree; torque
   must be regenerated at every level. Speed is the surplus resource.
2. **All three cones share ONE apex, and it is the pivot.** What must converge
   there are the **rubber** surfaces, so the plastic apexes are offset by
   `t / sin(half-angle)`. Never dial this in by hand.
3. **Rubber on rubber only.** The plastic cone surfaces never touch.
4. **The gear stage never disengages.** Free comes from the rubber separating.
5. **The cone axis must pass through the apex**, which is why the output shaft
   is the thing that gets centred by idlers, not the pinion.
6. **Idlers on the X axis, never on Z.** The pinion moves vertically when the
   carriage tilts; idlers above/below would jam on one side and disengage on the
   other.
7. **`Zc − Zp ≥ 8`** — internal-mesh interference.
8. **Free is the middle of the travel, not an end.** The two motor cones make
   each axis bidirectional.
9. **The AS5600 on every output shaft is structural, not optional.** Friction
   slip accumulates non-repeatably in series along a tree; position is only
   known by measuring it. I2C address fixed at **0x36**, so multiplexing is
   required (analog output, PWM, or a TCA9548A).

---

## Open questions — do NOT present these as settled

- **Rubber stiffness. The number everything hangs on.** How much normal force
  does a 2 mm layer develop at ~0.2 mm of squeeze? Until it is measured on the
  bench, every torque figure here is an estimate. It decides the spring, the
  servo and whether the cones must grow.
- **Spring rate, and how the extra travel splits between spring and rubber.**
  The 10:1 currently assumed is invented.
- **Four-bar drift.** "< 0.1 mm over ±2°" is an estimate; the macro should
  compute and print it.
- **How the rubber layer is made.** A cone unrolls into a flat sector, so a cut
  sheet can be wrapped on — untested (adhesive, seam, uniformity).
- **Packaging**: servo placement, idler support bracket, and the frame pivots
  for the links (they sit at z ≈ ±44 mm, past where the motor cones end).
- **Central motor sizing.** Depends on the rubber measurement and on how many
  axes engage at once — demands **add**, they do not divide.

---

## Discarded — do not re-propose

- **Tilting flat disc with a universal joint** (v3/v4). Superseded.
- **Laboratory conical rubber stoppers** as friction elements.
- **Fixed-threshold friction clutches to obtain free rotation.** Lifting and
  falling demand the same torque, so no threshold separates them.
- **Interleaved O-rings** (v5). The interdigitation forces the cones apart and
  costs 7.4% micro-slip, uncorrectably. Replaced by the continuous layer.
- **Concentric pinion in v6** (free = pinion concentric). Needs a gap angle of
  10–12° whatever you do, because rotation moves the pinion ~3× faster than the
  rubber and lengthening the shaft gains and loses in the same proportion. A
  conical/beveloid pair makes the tilt a design angle and a working point does
  exist, but at ~0.4 mm of clearance margin and module 0.85 — rejected as too
  tight, not as impossible. Full reasoning in the doc, §9.
- **Cam (disc or drum) for a variable ratio.** Unnecessary: the series spring
  already gives fast-then-powerful, because the load changes, not the ratio.
- **Scotch yoke with slot and rail.** Replaced by the telescopic spring link.
- **Belt or pulley reduction at the joint.** Breaks stacking symmetry.
- **Flexure virtual pivot.** Deferred to v7, not rejected — PLA creeps under
  sustained load.

---

## Files

| File | Purpose |
|------|---------|
| `docs/clutch-geometry-v6.md` | **v6 design — the source of truth.** Geometry, actuation, rejected alternatives, open questions, next steps |
| `cad/motcore_v6_apex_pivot.py` | FreeCAD macro — **to be written**, from the v5 macro |
| `cad/motcore_v5_vertical_clutch.py` | FreeCAD macro — v5, superseded. Still the best reference for cone solids, FDM hole compensation, gear helpers and the self-check scaffolding |
| `cad/clutch_geometry_v6.html` | Interactive 2D visualiser for v6 — four-bar, spring link, gear front view. Self-contained (no deps), meant for motcore.github.io |
| `cad/clutch_geometry_v5.html` | 2D visualiser — superseded, two generations stale (single motor cone, one-sided ladder) |
| `cad/clutch_geometry_v3.html` | Visualiser for the superseded tilting-disc design |
| `cad/clutch_geometry.html`    | Older visualiser (superseded) |
| `cad/motcore_compliant_lever.py` | FreeCAD macro — tilting-disc branch (superseded) |
| `cad/motcore_v1.py`           | FreeCAD macro — solid tilting branch (superseded) |
| `cad/motcore_plates.py`       | FreeCAD macro — bevel cone design (superseded) |
| `cad/motcore_animate.py`      | FreeCAD macro — bevel cone animation (superseded) |
| `cad/calibration.py`          | FreeCAD macro — FDM tolerance calibration coupon |
| `docs/build-log.md`           | Prototype build log — purchases, prints, calibrations, tests |
| `docs/clutch-geometry.md`     | v5 clutch geometry notes (superseded by the v6 doc) |
| `docs/design-evolution.md`    | History of all generations |
| `src/controller/`             | Arduino firmware — master (touchscreen UI) |
| `src/driver/`                 | Arduino firmware — receiver (motor + servo control) |

Web: `motcore.github.io/clutch-geometry.html` — live visualiser.

---

## Claude coding conventions

- **Language**: all code (comments, variable names, docstrings) and all git
  commit messages must be in **English**.
- Respond to the user in whatever language they use; only the code and commits
  must be English.
