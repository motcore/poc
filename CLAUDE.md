# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis actuator tree**. One central motor
(Z axis, vertical) drives multiple output axes through **conical friction
clutches**. Each output axis has its own clutch module; engaging a clutch
connects that axis to the rotating motor cone.

Each output feeds the **next hub of the tree**, so every level must end up a
**reduction** — torque has to be regenerated at each stage, and speed is the
resource there is plenty of. The cube itself is now 1.5:1 overdrive (friction
only, 1:1 drive out), so the reduction lives **between hubs**.

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
  **centred on the wall**, driven from the tilting cone by a **folded double
  cardan inside the hollow cone**.
- **Contact**: a **continuous rubber layer** on both cones, meeting along the
  shared generatrix — rubber on rubber, never plastic.
- Actuation is a single degree of freedom: the carriage's tilt angle.
- **Four axes**, one per cube wall, **identical rotated copies** (none turned
  over).

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
                 └────┘       + folded cardan inside it → output shaft (1:1)
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
- **The cone is hollow and is the cardan's outer yoke.** It runs on one 6805
  on its neck, held by the carriage, which is a ring round that neck. Inside:
  ring cross (y 40, cone axis) → intermediate tube → solid cross (y 26, output
  axis) → fork → output shaft in 2× MR105ZZ at the wall. Both crosses are in
  front of the apex, so it is not constant-velocity: bends 6.6° / 4.2°,
  ~0.11° error. Intermediate length changes 0.25 mm → slotted pin holes. Doc §6.
- **Four-bar**: link rays at 45°, frame pivots r 55 on the wall, carriage pivots
  r 40.5, links 14.5 mm. Apex drift 0.187 mm at preload, 0.60% slip. Doc §5.
- **Actuation (lead screw, doc §12d and §15)**: MG90D standing on the floor in
  the axis' own corner → its double horn (cut to 11 mm) keyed in a printed
  coupler, free in Z → T8 lead screw (lead 8), gripped on its thread → brass
  rectangular-flange nut on the spring cage, which slides on a Ø4 guide rod →
  a floating carrier between two spring stacks (series spring) → Ø3×14 ear pin
  pressed into the carriage block's −X face. The screw's thrust stops at the
  guide rod's foot (coupler | PTFE | plate | PTFE | lock collar); the servo
  takes torque only. ±80°: ~59° closes the gap, the rest compresses the spring.
  34 N at the ear at 60% of stall.
- **Further out is not more leverage.** Servo torque × servo angle = carriage
  moment × tilt; the tilt is fixed, so the push point is chosen for packaging.

### Three carriage positions — symmetric, both ways from the middle

The angle `φ` is measured from the mid position.

| # | Position | \|φ\| | State |
|---|----------|-------|-------|
| 1 | **Free**     | `0`        | rubber separated, output shaft drives nothing |
| 2 | **Contact**  | `φc`       | rubber surfaces touch, normal force zero |
| 3 | **Preload**  | `φc + δφ`  | rubber compressed, torque transmitted |

`φc = 90° − α − β` — the free gap angle **is** the cone geometry. That coupling
is the central fact of v6: you cannot open the travel without thinning the
output cone. The four-bar's outward drift makes real contact arrive at 2.18°,
at the band's outer end; preload stop 2.375°.

---

## Key parameters

| Symbol | Default | Description |
|--------|---------|-------------|
| **α**  | 55°   | motor cone half-angle (from the vertical axis) |
| **β**  | 33°   | output cone half-angle (from its own axis) |
| **t**  | 2.0 mm | rubber layer thickness |
| **L**  | 23.4 mm | contact band along the generatrix (derived), s = 10…33.4 |
| **s₀** | 10 mm | apex → start of the contact line |
| **four-bar** | 45°, r 55 / 40.5, link 14.5 | link rays, frame / carriage pivot radii |
| **cardan** | crosses y 26 / 40 | solid cross (output axis) / ring cross (cone axis) |
| **servo sweep** | ±80° (~59 + 21) | to contact + spring compression |
| **lead screw** | T8, lead 8, η 0.5 | + PTFE thrust washer friction |
| **ear** | y 38 | the moment arm of the vertical push |
| **cube** | 106.1 mm, 4 mm faces | a CUBE (user); decks stiffened by 12 mm ribs |

Derived at defaults (macro):

| Quantity | Value |
|----------|-------|
| free gap angle `φc` | **2°** (real contact 2.18°) |
| cube ratio ω_out/ω_motor | **1.50** (sin α / sin β, 1:1 drive out) |
| plastic apex offset, motor cones | 2.44 mm each |
| plastic apex offset, output cone | 3.67 mm |
| residual apex drift from the four-bar | 0.187 mm at preload = 0.60% slip |
| cardan residual angle error | ~0.11° |
| normal force per N on the ear (y_ear / s̄, s̄ = 29.4) | **×1.46** — force, NOT torque |
| output torque per N on the ear | ≈ 0.027 Nm/N; ~0.92 Nm at 34 N **(upper bound, μ 1.3)** |
| cube input (printed female dog) | ~1.1 Nm at the motor shaft: the whole cube's torque cap |

---

## Governing equations

```
φc          = 90 − α − β                   free gap angle = the cone geometry

apex offset = t / sin(half-angle)          plastic pulled back so the RUBBER
                                           surfaces share the apex

travel(p)   = distance(p, apex) · φ        every point moves by its own radius —
                                           this is what makes v6 unlike v5

ratio_total = sin α / sin β               1.50 per cube; the tree's reduction
                                           lives between hubs

ΣN          = F · lever / s̄                normal force from actuator force
T_out       = μ · F · lever · sin β        lever = y of the ear: the ear link
                                           pulls vertically

                                           s̄ does NOT appear in T_out, and the
                                           cancellation is exact: the moment
                                           arm and the friction radius are both
                                           s, so the contact's position divides
                                           out. Moving the rubber outward buys
                                           radius and costs normal force
                                           equally. Doc §8.
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

1. **Reduction across the tree.** Torque must be regenerated at every level.
   Each cube is 1.5:1 overdrive now, so the chain between hubs must more than
   undo that.
2. **All three cones share ONE apex, and it is the pivot.** What must converge
   there are the **rubber** surfaces, so the plastic apexes are offset by
   `t / sin(half-angle)`. Never dial this in by hand.
3. **Rubber on rubber only.** The plastic cone surfaces never touch.
4. **Free comes from the rubber separating.** The drive out stays coupled.
5. **The cone axis must pass through the apex.** Nothing rigid and coaxial can
   join the cone to the output shaft: at any distance from the apex the axes
   are offset as well as angled. Whatever couples them takes both (today: the
   folded cardan).
6. **Nothing may cross the apex.** The motor shaft is there; that is why the
   pivot is virtual and why both cardan crosses sit in front of it.
7. **All four axes are identical rotated copies.** Each owns one corner of the
   pinwheel (+X side); nothing is turned over.
8. **Free is the middle of the travel, not an end.** The two motor cones make
   each axis bidirectional.
9. **The AS5600 on every output shaft is structural, not optional.** Friction
   slip accumulates non-repeatably in series along a tree; position is only
   known by measuring it. I2C address fixed at **0x36**, so multiplexing is
   required (analog output, PWM, or a TCA9548A).
10. **Park in free.** Rubber left compressed takes a permanent set at that one
    spot, and a flat on a friction drive is a once-per-revolution thump — the
    failure that killed idler-wheel turntables. The working squeeze is ~0.08 mm
    and butyl sets badly, so this is not a small effect. Free being the middle
    of the travel does **not** make it automatic: gravity acts within the tilt
    plane and pulls toward the lower cone, and an unpowered servo's gearbox
    holds wherever firmware left the carriage. Command free before power-down
    and between movements. Doc §4.

---

## Open questions — do NOT present these as settled

- **Rubber stiffness. The number everything hangs on.** How much normal force
  does a 2 mm layer develop at ~0.2 mm of squeeze? Until it is measured on the
  bench, every torque figure here is an estimate. It decides the spring, the
  servo and whether the cones must grow.
- **Spring rate, and how the extra travel splits between spring and rubber.**
  The two spring stacks (~72 N/mm, Ø11/Ø5) are sized to the servo, not the
  rubber; `spring_ratio` = 15 is invented. No catalogue part chosen yet.
- **Self-energizing clutch (doc §14) — the project's original idea.** Today the
  torque that passes is set by the servo's squeeze, so a stronger motor does
  NOT give more torque. A chevron-guided yaw of the carriage about the apex
  would make the torque tighten the clutch (G = μ·cos β/k, keep G < 0.8 at
  μ_max). Not designed yet.
- **The fourth state: a brake / lock.** Free, forward and back exist; holding
  a pose under load does not (the clutch would slip). Needed by the walker.
- **Cube-to-cube torque.** The printed female dog caps each cube at ~1.1 Nm at
  its input; a column of cubes on the motor axis needs ~5–6 Nm.
- **How the rubber layer is made.** A cone unrolls into a flat sector, so a cut
  sheet can be wrapped on — untested (adhesive, seam, uniformity).
- **Sizing**: the layout is settled and the macro checks every pair of parts at
  every stop, sweeps the links, the cardan and the actuation chain as
  distances, and checks each axis against all three neighbours — but nothing has
  been loaded, FEA'd or printed. Weakest-looking: the printed ring cross, the
  output shaft's reach to the fork, the printed input dog.
  Building the cardan (not print-in-place; pins pressed into the crosses, free in
  the parts round them; Ø1.9 / Ø2.1 drills) and printing the carriage (wall-side
  face on the bed, the ring's extension is the bearing's stop): doc §6.
- **The macro is slow because of its checks.** `RUN_CHECKS = None` runs them all
  headless (freecadcmd, ~1 min) and skips them in the FreeCAD window (geometry in
  seconds). Always run it headless after a geometry change.
- **Central motor sizing.** Depends on the rubber measurement and on how many
  axes engage at once — demands **add**, they do not divide.

---

## Discarded — do not re-propose

- **Tilting flat disc with a universal joint** (v3/v4). Superseded.
- **Laboratory conical rubber stoppers** as friction elements. Their taper is
  7-10° of half-angle where the design needs 33° and 55°, and `α+β` is the
  angle between the axes, so they would put the output shaft nearly parallel
  to the motor. More fundamentally, a solid rubber cone winds up ~105° at
  2.4 Nm carrying the torque through its own body, against ~6° for a 2 mm
  bonded layer: the rubber must be thin and backed by something rigid. Full
  numbers in `docs/design-evolution.md`.
- **Fixed-threshold friction clutches to obtain free rotation.** Lifting and
  falling demand the same torque, so no threshold separates them.
- **Interleaved O-rings** (v5). The interdigitation forces the cones apart and
  costs 7.4% micro-slip, uncorrectably. Replaced by the continuous layer.
- **Gear stage** (pinion → idlers → corona), **spline/crowned coaxial
  coupling** (axes are offset, not just angled), **straight double cardan** and
  **cardan + Oldham** (too long), **Oldham alone** (120.8 mm cube, disc nods
  ±0.46 mm). Replaced by the folded cardan inside the cone. Doc §9.
- **Telescopic spring link from a crank on the floor**, **servo standing in a
  corner** (horn and carriage move in different planes), **servo at mid-height in
  the side corner** (hits the neighbour's cone), **servo flipped shaft-inward**
  (chain on the four-bar links), **turning alternate axes over**. Doc §9.
- **Concentric pinion in v6** (free = pinion concentric). Needs a gap angle of
  10–12° whatever you do, because rotation moves the pinion ~3× faster than the
  rubber and lengthening the shaft gains and loses in the same proportion. A
  conical/beveloid pair makes the tilt a design angle and a working point does
  exist, but at ~0.4 mm of clearance margin and module 0.85 — rejected as too
  tight, not as impossible. Full reasoning in the doc, §9.
- **Cam (disc or drum) for a variable ratio.** Unnecessary: the series spring
  already gives fast-then-powerful, because the load changes, not the ratio.
- **Scotch yoke with slot and rail.** Replaced long ago.
- **Belt or pulley reduction at the joint.** Breaks stacking symmetry.
- **Flexure virtual pivot.** Deferred to v7, not rejected — PLA creeps under
  sustained load.

---

## Files

| File | Purpose |
|------|---------|
| `docs/clutch-geometry-v6.md` | **v6 design — the source of truth.** Geometry, actuation, rejected alternatives, open questions, next steps |
| `cad/motcore_v6_apex_pivot.py` | FreeCAD macro — **the v6 assembly**. Run it headless (`freecadcmd`) and read its report: 31 numeric checks, the four-bar's measured drift, the cardan's bends, the actuation chain, the cube's driver, and the printed/purchased lists |
| `cad/motcore_v6_flat_pattern.svg` | 1:1 cutting template for the rubber bands, regenerated by the macro on every run |
| `cad/motcore_v5_vertical_clutch.py` | FreeCAD macro — v5, superseded. Still the best reference for cone solids, FDM hole compensation, gear helpers and the self-check scaffolding |
| `cad/clutch_geometry_v6.html` | Interactive 2D visualiser for v6 — **STALE** (telescopic link, gear front view); update before publishing |
| `cad/clutch_geometry_v5.html` | 2D visualiser — superseded, two generations stale (single motor cone, one-sided ladder) |
| `cad/clutch_geometry_v3.html` | Visualiser for the superseded tilting-disc design |
| `cad/clutch_geometry.html`    | Older visualiser (superseded) |
| `cad/motcore_compliant_lever.py` | FreeCAD macro — tilting-disc branch (superseded) |
| `cad/motcore_v1.py`           | FreeCAD macro — solid tilting branch (superseded) |
| `cad/motcore_plates.py`       | FreeCAD macro — bevel cone design (superseded) |
| `cad/motcore_animate.py`      | FreeCAD macro — bevel cone animation (superseded) |
| `cad/calibration.py`          | FreeCAD macro — FDM tolerance calibration coupon |
| `docs/bom-v6.md`              | Bill of materials for one cube, bought and printed, and what to measure on arrival |
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
