# Motcore v5 Clutch — Geometry & Equations

The interactive visualiser is at `cad/clutch_geometry_v5.html`. This document
derives the equations implemented there.

For the four superseded generations that led here, see
[design-evolution.md](design-evolution.md). The v2 bevel derivation is archived
at [archive/clutch-geometry-bevel.md](archive/clutch-geometry-bevel.md).

---

## 1. Design intent

A single vertical motor shaft must drive several output shafts, each of which
feeds the **next hub of an actuator tree**. Three requirements, in order of how
hard they were to satisfy:

1. **Real free rotation.** A disengaged axis must carry *nothing* — not a small
   residual friction, nothing. An arm has to be able to fall under its own
   weight. This is what killed every previous generation.
2. **Reduction at every level.** Torque is consumed going down the tree and has
   to be regenerated at each stage. Speed is the surplus resource.
3. **Torque limiting per axis**, settable in software.

v5 answers (1) with geometry rather than friction, (2) with an internal gear
stage, and (3) with the friction stage it already needed.

---

## 2. Architecture

- **Two motor cones** — on the central vertical shaft (Z), fixed, rotating
  continuously, facing **apex to apex** (an hourglass with a waist at the
  mid-plane). The lower has its apex up and flares downward; the upper is the
  same part fitted upside down.
- **Output cone** — one per axis, in that waist, on a **carriage that
  translates vertically** on two 3 mm guide rods. No tilt, no pivot, no
  universal joint. One degree of freedom.
- **Output shaft** — horizontal, fixed, perpendicular to the motor axis, at the
  mid-plane, with the apexes nominally common with whichever cone is engaged.
- **Friction contact** — interleaved rubber O-rings on both cones, meeting
  flank to flank.
- **Gear stage** — a pinion rigid to the output cone, permanently inside an
  internal corona (ring gear) fixed to the output shaft.
- **Four axes**, one per cube wall.

### 2.1 Two cones = one bidirectional axis

The second cone is not a second set of axes. Both motor cones turn with the
motor, always the same way, but their flanks face **opposite sides of the
output axis**. Move the carriage down and the output cone's lower flank meets
the lower motor cone; move it up and its upper flank meets the upper one — and
the output shaft turns **the other way**.

One motor, one axis, both directions, without ever reversing the motor. It is
the conical descendant of v3/v4's "one disc, two usable faces" (see
[design-evolution.md](design-evolution.md)), and it is why **free sits in the
middle of the travel** rather than at one end.

The internal mesh is indifferent to the sign: the pinion is captive inside the
corona and meshes at whatever azimuth it is offset to, up or down.

---

## 3. Coordinate system

The geometry is described in the Y-Z cross-section: the vertical plane
containing both the motor axis and the output shaft axis.

```
Z (up)
│         ┌───┐  ← output cone on the carriage
│        ╱     ╲    (translates vertically ↕)
│       │  ○ ○  │ ── output shaft (fixed) + internal corona
│     ╱ ╲╲     ╱╱
│    ╱ ○ ┴╲───╱  ← O-rings interleaved, flank to flank
│   ╱──────╲
│    motor cone
└──────────────────────────── Y (horizontal, toward the wall)
     motor axis at Y = 0
```

- **Z** = motor shaft, vertical, upward. Motor axis at Y = 0.
- **Y** = horizontal, from the motor axis toward the wall / output shaft.
- Carriage height reference: **free = 0**, the mid-plane between the two motor
  cones. Down is negative, up positive, and the stop ladder is symmetric.
- The output shaft axis is fixed at `z = 0`, so the pinion is concentric with
  the corona at rest, in position 1.

---

## 4. Free parameters

```
α   — motor cone half-angle from the vertical axis (default 55°)
m   — gear module (default 1.0)
Zp  — pinion teeth, on the output cone / carriage (default 14)
Zc  — corona teeth, on the output shaft (default 28)
L   — contact line length along the generatrix (default 18 mm)
s₀  — apex → start of the contact line (default 10 mm)
d   — O-ring wire diameter (default 2.5 mm)
n   — rings on the motor cone; the output cone carries n − 1 (default 5)
g   — margin from full mesh to rubber contact (default 0.20 mm)
δ   — preload travel past contact (default 0.25 mm)
```

Everything else is derived.

---

## 5. Cone angles

The two shaft axes are perpendicular and the apexes are common, so:

```
α_out = 90° − α_motor
```

At α = 55° the output cone half-angle is 35°.

---

## 6. Transmission ratio

The friction stage ratio follows from the two cone radii at any point on the
shared generatrix. Because both radii scale linearly with distance from the
common apex, the ratio is **independent of position along the contact line** —
which is exactly why the common apex matters:

```
ratio_fric = sin(α_m) / sin(α_o)
```

The gear stage is an internal mesh, pinion driving corona, so it reduces:

```
ratio_gear = Zp / Zc
```

```
ratio_total = ratio_fric · ratio_gear
```

At defaults: 1.428 × 0.500 = **0.714**, i.e. ω_out = 0.714 · ω_motor and
**1.40× torque**. `ratio_total > 1` means the design is multiplying speed and
losing torque — the visualiser flags it.

---

## 7. The gear stage and the vertical ladder

The pinion never leaves the corona; only the **centre distance** changes as the
carriage descends. At mesh:

```
e = m · (Zc − Zp) / 2
```

Tip radii are `r_tip,p = m·(Zp/2 + 1)` for the pinion and
`r_tip,c = m·(Zc/2 − 1)` for the corona, so the gap before the first tooth can
touch anything is:

```
free float  = r_tip,c − r_tip,p = e − 2m
mesh travel = 2m
```

That gives four stops. `z` is measured from the mid-plane, where the output
shaft and its corona sit, so **free is z = 0** and the same ladder runs both
ways — only the sign changes:

| # | Position | \|z\| of the output cone axis | State |
|---|----------|------------------------------|-------|
| 1 | **Free** | `0` | pinion concentric, rings separated, output shaft drives nothing |
| 2 | **Mesh** | `e` | centre distance reached, rings **still separated**, relative velocity zero → teeth engage without shock |
| 3 | **Contact** | `e + g` | rubber flanks touch, normal force zero |
| 4 | **Preload** | `e + g + δ` | flanks compressed, torque transmitted |

Negative `z` engages the lower motor cone, positive the upper one.

```
stroke = e + g + δ                (half stroke, free → preload one way)
travel = 2 · (e + g + δ)          (full, down to up)
```

At defaults: e = 7.20, free float 3.60, mesh travel 3.60, **stroke 7.65 mm**,
full travel **15.30 mm**.

### 7.1 Where the motor cones sit — derived, not chosen

Each motor cone's apex must be exactly where the output cone's apex lands at
**contact** on that side. That is what makes the two apexes common, which is
the entire basis of the matched-surface-speed contact of §6. So the apex
separation falls out of the gear stage and is not a free parameter:

```
motor cone apexes at  z = ± (e + g)
separation          = 2 · (e + g)
```

At defaults the apexes sit at z = ±7.40 mm, 14.80 mm apart.

The order of stops 2 and 3 is not negotiable. Teeth must be fully meshed
*before* the rubber starts turning the output cone, or they clash at speed.

---

## 8. Preload and root clearance

Past position 3 the carriage keeps travelling by δ to compress the rubber
flanks. The pinion therefore sits δ + g past the nominal centre distance, and
that overshoot has to fit in the gear's own **root clearance**:

```
c = 0.25 · m

g + δ ≤ c            [constraint]
```

This is why there is no slot, no floating ring and no compliant blade — the
gear already has the compliance the preload needs.

**Resolved (was open).** The constraint is really a floor on the module:
`m ≥ 4·(g + δ)`. It was violated at the original m = 1.0 (`0.45` against
`c = 0.25`); the module is now **1.8**, the smallest value that satisfies it,
giving `c = 0.45` exactly. It holds with **zero margin** — any tolerance that
has to be absorbed here means raising `m` (and with it the whole gear stage).

---

## 9. Ring interleaving

`n` rings sit on the motor generatrix and `n − 1` on the output generatrix,
offset by half a pitch. Counting both sets, N = 2n − 1 rings share the contact
line, so the **alternating** spacing is:

```
q = L / (2n − 2)
```

For two adjacent rings of wire diameter `d` to touch flank to flank, their
centres must be closer than `d` measured along the line:

```
q < d                [constraint]
```

If `q ≥ d` the flanks never reach each other and no torque passes at all.

When they do touch, the two generatrices are pushed apart by the normal offset:

```
h = √(d² − q²)
```

which separates the two apexes along the motor axis by:

```
apex separation = h / sin(α_m)
```

and that separation is what breaks the common-apex condition, producing
**micro-slip** along the contact line:

```
micro-slip ≈ apex separation / L
```

At defaults: q = 2.25 mm < d = 2.5 mm ✓, apex separation 1.33 mm,
micro-slip ≈ 7.4 %.

> **Open — narrow window.** More rings interdigitate better but push the cones
> further apart:
>
> | n | q | flanks touch? | micro-slip |
> |---|---|---------------|------------|
> | 3 | 4.50 mm | ✗ (q > d) | — |
> | 5 | 2.25 mm | ✓ | 7.4 % |
> | 7 | 1.50 mm | ✓ | 13.6 % |
>
> Needs a sweep over `L` and `d`. Catalogue O-rings fix the available `d`, and
> the resulting pitches then have to land on the generatrix.

---

## 10. Internal mesh interference

An internal pinion/ring pair with too small a tooth-count difference fouls on
assembly and during rotation:

```
Zc − Zp ≥ 8          [constraint]
```

At defaults 28 − 14 = 14 ✓.

---

## 11. Transmitted torque

Torque is set by the **radial preload between rubber flanks**, with μ ≈ 1.2–1.5
for rubber on rubber — against 0.6–0.9 for the rubber-on-plastic contact of the
earlier generations. Because preload is radial and not axial, it is decoupled
from apex displacement: pushing harder does not require the cones to move into
each other.

The friction stage is also a **per-axis torque limiter**. It slips above the
preload-set threshold, the preload is set by carriage position, and carriage
position is software — so each axis can cap its own slip torque.

---

## 12. Summary of constraints

| # | Constraint | At defaults |
|---|------------|-------------|
| 1 | `ratio_total < 1` — reduction, never 1:1 | 0.714 ✓ |
| 2 | Mesh before rubber contact (`\|z_mesh\| < \|z_contact\|`) | 0.20 mm ✓ |
| 3 | `g + δ ≤ 0.25 · m` — root clearance | 0.45 vs 0.45 ✓ **zero margin** |
| 4 | `q < d` — flanks can touch | 2.25 < 2.5 ✓ |
| 5 | `Zc − Zp ≥ 8` — internal mesh interference | 8 ✓ **zero margin** |
| 6 | AS5600 on every output shaft (structural, not optional) | — |
| 7 | Rubber on rubber only, never rubber on plastic | — |

The FreeCAD macro re-checks all of these on every run and prints the result,
along with solid-interference checks between adjacent axes and between the
output cone and the idle motor cone.

---

## 13. Default parameter values

| Parameter | Value | | Derived | Value |
|-----------|-------|-|---------|-------|
| α | 55° | | friction ratio | 1.428 |
| m | 1.8 mm | | gear ratio | 0.500 |
| Zp / Zc | 8 / 16 | | **total ratio** | **0.714** (1.4× torque) |
| L | 18 mm | | centre distance `e` | 7.20 mm |
| s₀ | 10 mm | | free float | 3.60 mm |
| d | 2.5 mm | | mesh travel | 3.60 mm |
| n | 5 | | **half stroke** | **7.65 mm** |
| g | 0.20 mm | | **full travel** (down..up) | **15.30 mm** |
| δ | 0.25 mm | | motor cone apexes | ± 7.40 mm (sep. 14.80) |
| | | | ring pitch `q` | 2.25 mm |
| | | | ring-induced apex offset | 1.33 mm |
| | | | micro-slip | ≈ 7.4 % |

**Why m = 1.8 and only 8/16 teeth.** The root-clearance constraint sets a floor
on the module regardless of tooth count: `m ≥ 4·(g + δ) = 1.8`. Going above that
floor only bloats the gear stage — at m = 2.0 with 14/28 teeth the corona came
out *larger than the motor cone itself*, which defeats the point of the vertical
cone. At the floor, fewer teeth is the remaining lever to shrink it (radius
scales with `m·Z`), down to the interference floor of `Zc − Zp = 8`. Both
constraints now sit at zero margin, so neither can absorb a tolerance: if real
parts need headroom, `m` has to move first.

---

## 14. Position feedback is structural

Friction slip accumulates non-repeatably in series along a tree. Commanded
position is therefore not knowable by dead reckoning — it has to be **measured**
at every output shaft. An **AS5600** magnetic encoder on each output shaft is
part of the mechanism, not an optional extra.

Its I2C address is fixed at **0x36**, so multiple encoders require multiplexing:
analog output, PWM, or a TCA9548A.

Firmware also needs the encoder *before* engaging: re-meshing while the output
shaft is still turning (an arm falling, say) clashes teeth. A velocity check via
the AS5600 has to gate the transition into position 2.
