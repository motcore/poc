# Clutch geometry — v6 "Apex Pivot"

Status: **active design**, settled 2026-09-12. Supersedes v5 (vertical carriage).
No FreeCAD macro yet — `cad/motcore_v6_apex_pivot.py` is the next thing to build,
starting from `cad/motcore_v5_vertical_clutch.py`.

---

## 1. The one-line change

In v5 the output cone **translated vertically** to engage. In v6 it **pivots
about the common apex**. Everything else follows from that.

The apex stops being a point the geometry passes through and becomes the hinge
the whole carriage swings on. It never moves, in any position.

---

## 2. What that buys

- **Preload acts exactly along the contact normal.** Rotating about the apex
  moves every point of the cone surface perpendicular to the shared generatrix,
  which is the contact normal. 100% of the push does useful work. In v5 the
  vertical push only delivered 87%, and the other 13% went sideways into the
  guide rods.
- **The apexes never separate.** In v5 the preload overtravel (δ) pushed the
  output cone 0.25 mm past the position where the apexes coincide, adding ~1.4%
  slip. Here the apex is the pivot, so preload adds zero apex error.
- **The motor cones can share one apex.** v5 had to hold them 14.8 mm apart
  (at ±(e+g)); in v6 their rubber surfaces converge on a single point, so the
  hourglass waist closes and the cube gets shorter.
- **The carriage is a lever.** The push point sits further from the apex than
  the rubber does, so the actuator gets mechanical advantage for free
  (≈ ×2.4 with the current numbers).

---

## 3. Cone geometry and the apex compensation

Three cones share one apex at the origin, on the motor axis:

- two motor cones, half-angle **α = 55°** from the vertical (Z) axis, flaring
  away from the waist;
- one output cone per wall, half-angle **β = 33°** from its own (Y) axis.

Contact happens when the angle between the axes equals α + β. At rest the axes
are perpendicular, so the **free gap angle** is

```
φc = 90° − α − β = 2°
```

Rotating the carriage by −φc closes the gap onto the lower motor cone, +φc onto
the upper one. Free is the middle, as in v5.

### The compensation that kills micro-slip

The rubber is a **continuous layer of thickness t = 2 mm** on each cone (see §4),
so the surface that actually touches sits t proud of the plastic. Offsetting a
cone's surface outward by t moves its apex backwards along its own axis by

```
apex offset = t / sin(half-angle)
```

So the **plastic** apexes are pulled back deliberately, and the **rubber**
surfaces are what converge on the origin:

| Cone | Offset along its own axis |
|---|---|
| each motor cone | 2 / sin 55° = **2.44 mm** |
| output cone | 2 / sin 33° = **3.67 mm** |

With that, the contact line passes through the point where the two axes cross,
which is the condition for matched surface speeds. **Micro-slip ≈ 0**, against
7.4% in v5.

This is the thing interleaved O-rings could not do: their offset came out of the
interdigitation geometry and was not freely correctable.

---

## 4. Continuous rubber layer (replaces the O-rings)

v5 used `n` O-rings on the motor cone and `n−1` on the output cone, interleaved
half a pitch so the flanks met rubber-on-rubber. v6 drops them for a **uniform
rubber layer** on both cones.

Why:

- **Contact area.** Two crossed rings touch at a point; two coated cones touch
  along the whole 18 mm generatrix. Far more grip for the same squeeze.
- **Zero micro-slip**, via the compensation above.
- **No pitch matching** between the two ring sets, and no `q < d` constraint.

How to make it: a cone is a developable surface, so it unrolls into a flat
circular sector. A rubber sheet cut to that sector can be wrapped on. **Not yet
tested** — see open questions.

Rubber on rubber stays the rule: the plastic surfaces never touch.

---

## 5. The virtual pivot (four-bar)

The apex sits on the motor shaft, so no physical bearing can go there. The
carriage hangs on a **four-bar linkage whose two link axes converge on the
apex**; the instant centre of the carriage is therefore the apex itself.

Current geometry (Y-Z plane, apex at origin):

| | |
|---|---|
| Link axis angles | ±50° from the Y axis, both through the origin |
| Frame pivots | radius 58 mm |
| Carriage pivots | radius 42 mm |
| Link length | 16 mm |
| Sets | two, one at each side in X, for out-of-plane stiffness |

**Measured in the v6 visualiser, 2026-09-12** (the earlier "< 0.1 mm" estimate was
wrong): at full preload (±2.49°) the point that started at the apex moves
**0.22 mm**, and about 0.14 mm at contact. That is a residual apex error worth
roughly 1% of slip at full preload — an order better than v5's 7.4%, but **not
zero**, so do not quote "≈ 0" for the total slip. It is tunable through the link
angles and lengths; the macro should compute it and print it as a check.

The **instant centre** itself wanders much further, ~10 mm at full stroke. That is
normal four-bar behaviour and is not a fault: what matters is how far the apex
*point* travels, which is the 0.22 mm above.

Links must not be at ±35°: that is exactly the shared generatrix direction, i.e.
the cone surfaces. Anything between 33° and 35° is inside the clearance wedge.

A flexure version (crossed or converging blades, monolithic, zero backlash) was
considered and **deferred to v7** — PLA creeps under sustained load, and the
project already abandoned a compliant blade in v4.

---

## 6. Gear stage — always meshed, output centred

**The pinion never disengages.** Free rotation comes from the rubber separating,
not from the gears letting go. With the gears permanently meshed the joint still
drags only the cone's inertia reflected through the ratio, about 2% of a typical
leg's inertia — no torque threshold, which was the real requirement.

That removes, in one stroke: the mesh-before-contact ladder, the tooth clash on
re-mesh (an open question since v5), the root clearance constraint `g+δ ≤ 0.25m`
and the module floor it imposed.

**Centring.** A meshed internal pair has its centres offset by `e`, so either the
pinion or the output shaft ends up off the wall centre. Idler gears solve it:

```
pinion (on the carriage, centred)  →  idler(s) on fixed axes  →  corona (output, centred)
```

| | Teeth | Module | Pitch radius |
|---|---|---|---|
| Pinion (carriage) | 8 | 1 | 4 mm |
| Idlers | 8 | 1 | 4 mm |
| Corona (output) | 24 | 1 | 12 mm |

Ratio 1/3, same as the offset single pair would have given, with the output shaft
on the wall centre. Two idlers, **placed left and right in X, never above and
below**: the pinion moves vertically when the carriage tilts, so idlers on the X
axis see the centre distance change only to second order (+0.35 mm worst case,
i.e. slightly more backlash), while idlers on the Z axis would have one jam and
the other disengage. One idler is enough functionally; two balance the radial
load on the pinion.

Consequences: the corona turns opposite to the pinion (irrelevant — the axis is
bidirectional, it is a sign in firmware), there is one more mesh (a little more
lost motion and loss), and the idlers need a support bracket entering from the
carriage side, because the corona's output face is closed by its plate.

The pinion tilts ±2° inside the mesh, which modest crowning handles.

---

## 7. Actuation

```
servo  →  crank  →  telescopic link with two springs inside  →  pin on the carriage's bearing housing
```

- **Push point: the bearing housing**, on the centre line, ~46 mm from the apex,
  between the cone base and the pinion. Centred means the push is purely
  tangential (vertical at the centre line) and both links share the load evenly.
  It is also the stiffest part of the carriage, which matters because the whole
  travel to contact is only ~1.6 mm.
- **No slot, no external guide.** The link is pinned at both ends; the only
  sliding left is the coaxial tube-in-tube inside the link, which is what the
  springs compress against. This replaced an earlier Scotch-yoke-plus-rail
  scheme.
- **Series springs are the whole trick.** Before contact the carriage follows the
  servo with almost no load. After contact the carriage stops and every further
  degree of servo goes into compressing the spring, so spring force — not
  position — sets the preload. That is the "fast then slow and powerful"
  behaviour, obtained without any cam.
- **Toggle.** Running the preload phase near the crank's dead centre multiplies
  force (the toggle-clamp effect) and lets the servo hold high force at low
  torque, which also fixes the micro-servo overheating worry.

Working numbers (first pass):

| | |
|---|---|
| Servo | MG90D (digital, ~0.22 Nm, ~0.08–0.1 s/60°) |
| Crank radius | ~6.3 mm |
| Sweep | ±85°, contact at ~15° |
| Travel to contact at the push point | ~1.6 mm |
| Spring travel | ~4.5 mm |
| Spring rate | ~20 N/mm **(placeholder)** |
| Spring/rubber split | 10:1 **(placeholder, needs the rubber measurement)** |

**Pick the servo on speed, not torque.** Torque is comfortable; the binding spec
is getting free→contact done inside a push-off window of 50–150 ms. Digital also
matters for its small deadband, because preload is set by small angle changes.

---

## 8. Torque estimate

Moment balance about the apex gives a closed form:

```
ΣN      = F · R_push / s̄                    normal force from the actuator force
T_out   ≈ μ · F · R_push · sin β · (Zc/Zp)
```

With μ = 1.3, R_push = 46 mm, β = 33°, Zc/Zp = 3: **≈ 0.098 Nm of output torque
per newton of actuator force** — so ~2.4 Nm at 25 N and ~8.8 Nm at 90 N. That is
roughly 3× v5, from the lever (×2.1) and the bigger gear ratio (×1.5).

**This is an upper bound.** It assumes the rubber actually develops that normal
force within the available squeeze, which is exactly the unmeasured number below.
Do not quote it as fact.

Slip is a feature: it is a per-axis torque limiter and makes the joint
back-drivable, which matters for the passive-dynamics goal.

---

## 9. Why this design and not the alternatives

Recorded so they are not re-litigated. Each cost real time.

**Concentric pinion (free = pinion concentric, as in v5).** Wanted because it
puts the output shaft on the wall centre. It does not fit a pivoting carriage.
Rotation moves each point by (distance from apex) × angle, so the pinion — about
3× further from the apex than the rubber — digs into the corona 3× faster than
the rubber squeezes. The clearance it can use is `0.25 · m`, so the module must
grow, which grows the mesh travel `e`, which needs more angle. The bound is

```
φc  >  2 · (Zc − Zp) · q / s̄            q = rubber squeeze, s̄ = mean rubber distance from apex
```

≈ 10–12° with any sensible numbers, and **independent of where the pinion sits** —
lengthening the shaft gains travel and loses clearance in exactly the same
proportion. Designing the pair as a conical/beveloid gear (the right family for
small shaft angles, and the user was right that it exists) makes the tilt a
design angle rather than a misalignment, and a working point does exist — α≈45°,
pinion ~20 mm from the apex, module ~0.85, corona radius ~9 mm — but the
clearance margin is ~0.4 mm and the module is at the edge of what FDM prints.
**Rejected as too tight**, not as impossible.

**Cam (disc or drum) to get a variable ratio.** Unnecessary. The series spring
already produces fast-then-powerful, because the load changes, not the ratio. A
drum cam would be more compact than a disc (stroke becomes axial, so the diameter
stops depending on it) if one were ever needed.

**Scotch yoke with slot and rail.** Replaced by the telescopic link — fewer
parts, no slot to jam, same toggle.

**Belt or pulley reduction at the joint.** Breaks the stacking symmetry: the
output must present the same interface as the input so cubes can chain.

**Flexure virtual pivot.** Deferred to v7 (PLA creep).

---

## 10. Open questions

1. **Rubber stiffness — the number everything hangs on.** How much normal force
   does a 2 mm rubber layer develop at ~0.2 mm of squeeze? Until this is measured
   the output torque is a guess. Bench test: press two rubber-faced coupons
   together with a micrometer screw and a kitchen scale, and record N/cm at 0.04,
   0.1 and 0.2 mm. Half an hour of work; it decides the spring, the servo and
   whether the cones need to grow.
2. **Spring rate and the spring/rubber split.** The 10:1 in §7 is invented.
3. **Four-bar drift.** The "< 0.1 mm over ±2°" is an estimate. The macro should
   compute the apex position at the engaged stops and print it as a check.
4. **Making the rubber layer.** Wrapping a flat sector onto the cone is
   geometrically right but untested: adhesive, seam, thickness uniformity.
5. **Packaging.** Servo placement, idler support bracket, link geometry and the
   frame pivots' brackets (they sit at z ≈ ±44 mm, well past where the motor
   cones end, so they need new structure).
6. **Central motor sizing.** Depends on (1) and on how many axes are engaged at
   once. Note the demands add, they do not divide: two engaged axes ask the motor
   for the sum of their slip torques.

---

## 11. Next steps

1. Build `cad/motcore_v6_apex_pivot.py` from the v5 macro. Reusable almost
   verbatim: cone solids, the FDM hole compensations, the gear helpers, the
   document/placement/interference-check scaffolding. New: single-apex layout
   with rubber-surface compensation, rubber layer instead of grooves, four-bar
   instead of guide rods, idlers, and the checks in §5 and §10.3.
2. Run the rubber measurement (open question 1) before committing to a spring or
   a servo.
3. **Done 2026-09-12:** `cad/clutch_geometry_v6.html`, a standalone interactive
   visualiser (four-bar with the instant centre traced, telescopic spring link,
   gear front view with the idlers). Self-contained, no dependencies, ready to
   publish on motcore.github.io. `cad/clutch_geometry_v5.html` can be retired.
