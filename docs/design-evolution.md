# Motcore — Design Evolution

Five generations of the same question: **how does one motor drive many axes
independently, cheaply, and with real free rotation?**

Each generation was killed by a specific, identifiable failure — not by taste.
This document records what each one was, what broke it, and what survived into
the next. Nothing here is a proposal; the only live design is **v5**.

| Gen | Dates | Mechanism | Status |
|-----|-------|-----------|--------|
| **v1** | Mar–May 2026 | LEGO + lateral friction wheels | superseded |
| **v2** | May 2026 | Bevel cone friction drive, common apex | superseded |
| **v3** | Jun 2026 | Flat motor disc + O-ring wheel, printed compliant neck | superseded |
| **v4** | Jun–Jul 2026 | Metal UJ + engagement blade, over-centre latch | superseded |
| **v5** | Aug 2026 | Vertical conical clutch, interleaved O-rings, captive pinion | **active** |

The numbering follows **design generations**, not files. There is no
`clutch_geometry_v2.html` or `_v4.html` — v2 is `clutch_geometry.html`
("Bevel Clutch Geometry"), and v4 reused v3's visualiser because its geometry
was unchanged in the Y-Z cross-section.

---

## v1 — LEGO proof of concept

*Lateral friction wheels, ~Mar–May 2026*

One 28BYJ-48 stepper turning continuously; four SG90 servos pushing friction
wheels against the sides of a central shaft. Output direction depended on which
side of the shaft the wheel touched. Frame in LEGO Technic.

**What it proved:** selective friction engagement works as a transmission
method, and four axes can be driven independently from one motor. This is the
result the whole project rests on.

**Why it was superseded:** LEGO tolerances, no real parameter control, and no
way to reason about contact geometry. It was a demonstrator, never a design.

**Survived into later generations:** the two-Arduino architecture
(controller + driver), the touchscreen UI, and the basic BOM.

---

## v2 — Bevel cone, common apex

*`cad/motcore_plates.py`, `cad/motcore_animate.py`,
`cad/clutch_geometry.html`, May 2026*

A truncated cone on each output shaft, tangent along a **common generator line**
to a cone on the motor shaft. The output shaft tilted around a UJ pivot near the
wall to bring the cones into contact. Both cones shared an apex, so the
`α_clutch + α_motor = 90° − A` bevel condition held automatically for any choice
of A, B, C, D.

Defaults: A = 8°, B = 35 mm, C = D = 9 mm, L ≈ 42.4 mm, α ≈ 49° for both cones.

**What it fixed:** differential slip. Line contact along a common generatrix
means every point on the contact line has matched surface speed — a flat wheel
on a flat disc does not.

**Why it was superseded:** the full derivation lives in
[archive/clutch-geometry-bevel.md](archive/clutch-geometry-bevel.md). In short,
holding the common apex through a tilt made every dimension depend on the
engagement angle, and the plastic-on-plastic (later plastic-on-rubber) contact
had too little friction to be useful at achievable normal forces.

**Survived:** the common-apex idea itself, which is still the core of v5 — only
now the apexes are held by translating a carriage instead of tilting a shaft.

---

## v3 — Flat disc + O-ring wheel, compliant flexure

*`cad/motcore_v1.py`, `cad/clutch_geometry_v3.html`, Jun 2026*

The motor cone became a **flat horizontal disc** with two usable faces. Each
output shaft carried a wheel with a silicone O-ring in a toroidal groove, and
tilted ±A to press that O-ring against the top face (one direction) or the
bottom face (the other). The tilt came from flexure blades printed in one piece
with the wall, plus a compliant neck in the printed shaft.

Defaults: A = 3°, B = 35 mm, R = 20 mm, O-ring wire 2.5 mm, blade 1.5 × 6.0 mm,
neck Ø2.5 × 13 mm. Contact force ≈ 4.85 N, output torque ≈ 58 N·mm,
speed ratio ≈ 1.39× the motor.

**What it fixed:** friction. Rubber on plastic (μ ≈ 0.6–0.9) beats cone-on-cone
plastic by a wide margin, and one disc with two faces gave both directions from
a single motor without reversing it.

**Why it was superseded:** the printed compliant neck was the weak link — it had
to be soft enough to tilt and stiff enough to carry output torque, and those two
requirements pulled in opposite directions. Output torque was low
(58 N·mm), and the design multiplied speed (1.39×) when what a tree of actuators
needs is torque.

**Survived:** O-rings as the friction element, and the torque-limiter framing —
a slipping clutch protects the drivetrain, and slip threshold is settable in
software.

---

## v4 — Metal UJ + engagement blade, over-centre latch

*`cad/motcore_compliant_lever.py`, Jun–Jul 2026*

The most developed of the superseded branches, and the one that got printed.
The printed neck was replaced by a **purchased metal universal joint**
(Ø11 × 23 mm, 5 mm bore) straddling the wall, with metal Ø5 mm output shafts in
two segments. All compliance moved into a printed **engagement blade**: a thin
spring section driven by a pin on a short (4 mm) servo arm riding in a slot.

Its best trick was the **over-centre latch** — the servo arm swept just past its
90° dead point onto a hard stop, so the spring held the clutch engaged with the
**servo unpowered**. Current only flowed during transitions. Blade geometry was
sized to keep sustained latch stress ≈ 14 MPa so PETG creep stayed negligible.

Defaults: A = 1.5°, B = 35 mm, R = 20 mm, contact radius R_out = 15 mm,
ratio ≈ 0.75 (ω_out ≈ 1.34 × ω_motor), T_out ≈ 250 N·mm at the latch — roughly
2× the servo's own continuous torque.

Weeks of FDM work went into this branch: bearing seats, M3 clearance passes, a
dovetail O-ring groove to stop the ring slipping, a two-part printable motor
disc, a corner-post cage frame.

**Why it was superseded — this is the important one.** A friction clutch with a
fixed threshold cannot give **free rotation**. To let an arm fall under its own
weight, the clutch must transmit nothing; to lift that same arm, it must
transmit torque. Both demand the same magnitude of torque at the shaft, so
**no threshold separates them**. Loosening the clutch enough to let the arm fall
also loses the ability to lift it. Every fix within the tilting-clutch family
runs into this wall.

The second problem: at ratio 0.75 the design still multiplied speed. Feeding one
axis into the next hub of a tree means torque has to be *regenerated* at every
level, and v4 spent it instead.

---

## v5 — Vertical cone, interleaved rings, captive pinion

*`cad/clutch_geometry_v5.html`, Aug 2026 — **active***

Full description in [clutch-geometry.md](clutch-geometry.md) and the project's
`CLAUDE.md`. The three changes that answer v4's failures:

**1. Free rotation is geometric, not frictional.** A pinion rigid to the output
cone lives permanently *inside* an internal corona fixed to the output shaft.
Free = pinion **concentric** with the corona, centre distance 0 — the output
shaft carries nothing at all. No threshold, no residual drag, no compromise
between lifting and falling. The pinion is captive by construction and cannot
fall out.

**2. Rubber meets rubber.** Both cones carry O-rings — `n` on the motor cone,
`n − 1` on the output cone, offset half a pitch so the two sets
**interdigitate** and touch flank to flank. μ ≈ 1.2–1.5 instead of 0.6–0.9, and
the rings never touch the plastic. Preload is **radial between flanks**, which
decouples it from apex displacement: pushing harder no longer requires the cones
to move axially into each other.

**3. It reduces.** Friction ratio 1.428 × gear ratio 0.500 = **0.714**, i.e.
**1.4× torque**. Speed is the surplus resource in a tree; torque is not.

Actuation collapses to a single degree of freedom — the vertical position of a
carriage on two 3 mm guide rods — passing through four stops on the way down:

| # | Position | State |
|---|----------|-------|
| 1 | Free | pinion concentric, rings separated, shaft drives nothing |
| 2 | Mesh | centre distance `e` reached, rings still separated, relative velocity zero → teeth engage without shock |
| 3 | Contact | rubber flanks touch, normal force zero |
| 4 | Preload | flanks compressed, torque transmitted |

Total stroke 7.45 mm at defaults. Preload travel is absorbed by the gear's own
root clearance — no slot, no floating ring, no compliant blade.

**Still open, not settled:** ring density versus micro-slip (≈ 7.4 % at n = 5,
13.6 % at n = 7, flanks never touch at n = 3); `g + δ` exceeding root clearance
at m = 1; tooth clash if the carriage re-meshes while the output shaft is still
turning; and how the rings are actually manufactured. See the open questions in
`CLAUDE.md`.

---

## What carried through all five

- **One motor, many axes, selectively engaged.** Never in question since v1.
- **Common apex / matched surface speed.** Introduced in v2, still the core of v5.
- **O-rings as the friction element.** Introduced in v3.
- **Friction as a per-axis, software-settable torque limiter.** Introduced in v3.
- **Two-Arduino split**, controller and driver. Unchanged since v1.

## Dead ends — do not re-propose

- **Tilting a shaft (universal joint, printed neck, or flexure) to engage.**
  v2, v3 and v4 all did this. Superseded by carriage translation.
- **Laboratory conical rubber stoppers** as friction elements.
- **Fixed-threshold friction clutches to obtain free rotation.** Lifting and
  falling demand the same torque, so no threshold separates them.
- **Any 1:1 or speed-multiplying stage.** The tree needs reduction at every level.
