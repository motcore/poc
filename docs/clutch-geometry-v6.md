# Clutch geometry — v6 "Apex Pivot"

Status: **active design**, settled 2026-09-12, repacked 2026-09-15. Supersedes
v5 (vertical carriage). The macro is `cad/motcore_v6_apex_pivot.py`; it builds
the assembly and self-checks it (26 numeric checks), and every number quoted
below that says "measured" comes from its own output.

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
- **The carriage is a lever.** The push arrives further from the apex than the
  rubber does, so a newton of actuator force becomes **×1.53** of total normal
  force (macro, measuring s̄ at the squeeze's own centroid). Read that as force,
  not as torque: see §8 — where the contact sits cancels out of the output
  torque exactly.
- **The box got shorter.** Hollowing the output cone and putting the bearing
  housing inside it moved the whole gear stack behind the push point: 145 mm
  cube → **119 mm**, with the corona's running clearance now the thing that
  sets it.

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

### Park in free, always

Idler-wheel turntables — Garrard, Lenco — drove exactly like this, a thin rubber
tyre on a rigid hub transmitting by friction. Their classic failure was a **flat
spot**: left switched off with the idler still pressed against the motor spindle,
the rubber took a permanent set at that one point, and afterwards it thumped once
per revolution. Good decks lifted the idler when you switched off.

The same applies here, and the numbers are not comfortable: the working squeeze
is ~0.08 mm on a 2 mm layer, and butyl — the obvious prototype material, since a
bicycle inner tube is one — is among the worst rubbers for compression set. Thirty
percent of that squeeze taken as permanent set is 0.024 mm, a third of the
preload, at **one azimuth of each cone**. That is the turntable thump again.

v6 looks immune because free is the middle of the travel, with the rubber clear
on both sides — but the carriage does not return there by itself. Gravity acts
**within** the tilt plane on all four axes (each tilt plane contains Z), so it
pulls toward the lower motor cone rather than toward the middle, and with the
servo unpowered what actually holds the carriage is the servo's own gearbox,
which is stiff to back-drive. The carriage stays wherever firmware left it.

So it is a firmware rule, not a property of the mechanism: **command free before
cutting power, and do not rest in preload between movements.** Making it true
even through a crash or a flat battery would need a centring spring on the
carriage itself — the four-bar links are the natural place — and that is design,
not a line of code.

---

## 5. The virtual pivot (four-bar)

The apex sits on the motor shaft, so no physical bearing can go there. The
carriage hangs on a **four-bar linkage whose two link axes converge on the
apex**; the instant centre of the carriage is therefore the apex itself.

Current geometry (Y-Z plane, apex at origin):

| | |
|---|---|
| Link axis angles | ±40° from the Y axis, both through the origin |
| Frame pivots | radius 62 mm |
| Carriage pivots | radius 40.5 mm |
| Link length | 21.5 mm |
| Sets | two, one at each side in X, joined at the carriage pivot into one part |

The ray came down from 50° and the carriage pivot moved out from 42 mm when the
output cone became a shell: the carriage's arms now start *inside* the cone and
can only leave through its mouth at y = 30.8, so their pivot has to be beyond
that. Tilting the ray is what keeps the link long while doing it — and a long
link is what keeps the drift small.

**Measured by the macro** (`apex_drift`, on the linkage's own solved pose, not on
an ideal rotation): at full preload the point that started at the apex moves
**0.133 mm**, and 0.093 mm at contact — against 0.226 mm for the ±50°/16 mm
layout this replaces. Residual slip **0.44%** of the contact line, against v5's
7.4%. Not zero: do not quote "≈ 0".

The drift decomposes into a component **along** the shared generatrix (the part
that slips, +0.103 mm) and one **normal** to it (−0.085 mm), which is a preload
error worth 4% of the rubber's thickness — enough that contact arrives at the rim
first and spreads inward rather than landing along the whole line at once. The
macro prints both.

The **instant centre** itself wanders much further, ~10 mm at full stroke. That is
normal four-bar behaviour and is not a fault: what matters is how far the apex
*point* travels, which is the 0.133 mm above.

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
| Pinion (carriage) | 8 | 1.8 | 7.2 mm |
| Idlers | 8 | 1.8 | 7.2 mm |
| Corona (output) | 24 | 1.8 | 21.6 mm |

**The module was 1.0 here and 1.0 is not buildable.** At `Zp = 8` the pinion's
root radius is `m·(Zp/2 − 1.25)` = 2.75 mm, and the bore for the Ø5 output shaft
is 2.90 mm once the printer's undersize is compensated: the bore eats the hub and
the gear comes out as eight loose teeth. The floor for a 2 mm hub wall is

```
m  ≥  (bore radius + wall) / (Zp/2 − 1.25)  =  1.78
```

hence 1.8, which is also where v5's module floor landed for an unrelated reason.
The alternatives, if 1.8 is too big: more pinion teeth, a stepped Ø3 shaft end
(but Ø3 steel is at its torsional limit near 0.5 Nm), or a pinion integral with
the shaft — which then cannot be plastic, since it carries the full output
torque. Note the module does **not** touch the clutch geometry in v6: the gears
never disengage, so there is no mesh travel and no root-clearance constraint.
It is purely a packaging cost, unlike v5 where `m` was on the critical path.

Ratio 1/3, same as the offset single pair would have given, with the output shaft
on the wall centre. Two idlers, **placed left and right in X, never above and
below**: the pinion moves vertically when the carriage tilts, so idlers on the X
axis see the centre distance change only to second order (+0.20 mm worst case
at m = 1.8, measured in the macro,
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
servo  →  crank  →  telescopic link with two springs inside  →  pin on the carriage's horn
```

- **Push point: a horn on the carriage**, at y = 40 mm, z = −30 mm — not on the
  centre line. The centre line is where the output shaft is, and straight out
  from the housing is where the gear plane is, so the horn drops below the
  corona's 25.6 mm rim first and then runs forward. It carries a Ø5 steel pin
  pressed through it, not a printed boss: the load bends that pin across the
  layers, which is the one direction FDM has none.
- **R_push is not the lever.** The force arrives along the link, and the link
  comes up off the floor at a shallow angle, so the moment arm about the apex is
  the perpendicular distance from the apex to the link's *line of action* —
  **42.3–43.2 mm** over the stroke, measured by the macro (`push_lever`). Swept
  over R_push = 32…50 mm that distance barely moves: pushing from further out
  costs as much angle as it gains radius. So R_push is chosen for packaging,
  and the horn is as short as the corona's rim allows.
- Because the push point sits 30 mm below the axis, the link's large **Y**
  component earns a moment of its own about the apex and very nearly makes up
  for the Z component it gives up. That is why the servo could be pulled inboard
  along the floor at all.
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
| Crank radius | 6.3 mm |
| Servo-side travel | ±6 mm |
| Travel to contact at the push point | 1.40 mm |
| Spring travel | ~4.6 mm |
| Spring rate | ~20 N/mm **(placeholder)** |
| Spring/rubber split | 10:1 at R = 50 mm **(placeholder, needs the rubber measurement)** |

**The split is a ratio, not a constant.** A given spring and a given rubber look
like a *different* ratio from a different radius, because the same push-point
travel becomes a bigger tilt and the tilt is what squeezes the rubber — it goes
as 1/R². The macro derives `spring_split` from the invented 10:1 and the radius
it was invented at, so shortening the horn cannot quietly raise the preload.
(Before that was fixed, going from R = 50 to R = 40 took the preload squeeze
from 4.0 to 7.4 mm³ on a change that was only supposed to be about packaging.)

### Packaging

- The **servo lies on the deck**, shaft on X, case running *inboard* along that
  shaft axis. Outboard is the neighbouring axis' wall, 55.5 mm out, and the
  stack (link, crank, standoff, 29 mm of case) needs 60. Inboard, under the
  motor cone and above the deck, there is a pocket with nothing in it.
- The **crank sits inboard of the link plane**, with its pin reaching outboard
  through the link. Outboard, the crank's boss would have to cross the link
  plane at the hub — and near dead centre the link lies right on top of the hub.
- **Alternate axes are assembled turned over**: the same parts, rotated 180°
  about their own radius, so their servo, crank and horn lie against the other
  deck. It is a rotation, not a mirror, so it costs no new parts, and the shared
  motor cones are symmetric in Z, so a turned-over axis still meshes with them.
  Without it two neighbouring servos want the same corner of the same floor, and
  no arrangement inside this box avoids it — the case is 29 mm along the shaft
  and the crank has to sit beside the horn, so it reaches across the middle.
  The decks and walls therefore carry **both hands** of every bracket's screw
  pattern; a boss nothing screws into costs a gram.

**Pick the servo on speed, not torque.** Torque is comfortable; the binding spec
is getting free→contact done inside a push-off window of 50–150 ms. Digital also
matters for its small deadband, because preload is set by small angle changes.

---

## 8. Torque estimate

Moment balance about the apex gives a closed form:

```
ΣN      = F · lever / s̄                     normal force from the actuator force
T_out   ≈ μ · F · lever · sin β · (Zc/Zp)
```

`lever` is the perpendicular distance from the apex to the actuation link's line
of action (§7), **not** R_push — R_push would be the arm only if the push were
vertical.

With μ = 1.3, lever = 42.3 mm, β = 33°, Zc/Zp = 3: **≈ 0.090 Nm of output torque
per newton of actuator force** — so ~2.2 Nm at 25 N and ~8.1 Nm at 90 N.

**Where the contact sits does not appear in that, and the cancellation is
exact.** Moments about the apex give `F·lever = ∫ n(s)·s ds`, because a force
perpendicular to the generatrix at distance `s` from the pivot has moment arm
exactly `s`. The output torque is `μ·∫ n(s)·(s·sin β) ds`, the same integral —
so `T = μ·sin β·F·lever` whatever the pressure distribution. Moving the rubber
outward buys friction radius and costs normal force in the same proportion. It
is the same property of the apex pivot that matches the surface speeds:
everything scales with `s`.

So `lever / s̄` is **not** a torque multiplier, and reading it as one is a trap
this document previously set. What it gives is the total NORMAL force, which is
what the rubber has to develop inside the available squeeze — open question 1.
The macro measures `s̄` at the centroid of the actual squeeze, 27.7 mm, giving
**×1.53**; the band's midpoint, which is what an earlier ×2.4 came from,
overstates it by half because the contact is not where the band's middle is.

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
2. **Spring rate and the spring/rubber split.** The 10:1 in §7 is invented. It
   is now carried across changes of radius correctly (§7), but the number itself
   still has to be measured.
3. ~~**Four-bar drift.**~~ **Answered.** The macro solves the linkage and
   measures the apex's travel at every stop: 0.133 mm at preload, split into
   0.103 mm of slip along the generatrix and 0.085 mm of preload error normal to
   it. See §5.
4. **Making the rubber layer.** Wrapping a flat sector onto the cone is
   geometrically right but untested: adhesive, seam, thickness uniformity.
5. ~~**Packaging.**~~ **Answered, and checked.** Servo on the deck with its case
   inboard, idler on a D-shaped bracket screwed to the wall from inside, frame
   pivots on posts standing on the decks, alternate axes turned over (§7). The
   macro checks every pair of parts at every stop, and each axis against all
   three of its neighbours. What is left is not layout but sizing: none of the
   brackets has been loaded or FEA'd.
6. **Central motor sizing.** Depends on (1) and on how many axes are engaged at
   once. Note the demands add, they do not divide: two engaged axes ask the motor
   for the sum of their slip torques.

---

## 11. Next steps

1. **Done 2026-09-15:** `cad/motcore_v6_apex_pivot.py` — single-apex layout with
   rubber-surface compensation, the continuous rubber band (with its 1:1 cutting
   template, `cad/motcore_v6_flat_pattern.svg`), the four-bar with its drift
   measured, the idler stage, the fasteners as solids, and 26 numeric checks.
2. Run the rubber measurement (open question 1) before committing to a spring or
   a servo. It is the only thing still blocking a torque figure.
3. Print one axis. Everything above is geometry that has never been in a
   printer.
3. **Done 2026-09-12:** `cad/clutch_geometry_v6.html`, a standalone interactive
   visualiser (four-bar with the instant centre traced, telescopic spring link,
   gear front view with the idlers). Self-contained, no dependencies, ready to
   publish on motcore.github.io. `cad/clutch_geometry_v5.html` can be retired.
