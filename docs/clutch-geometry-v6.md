# Clutch geometry — v6 "Apex Pivot"

Status: **active design**, settled 2026-09-12, repacked 2026-09-15, drive and
actuation rebuilt 2026-09-17 (branch `v7-cardan-zigzag`). Supersedes v5
(vertical carriage). The macro is `cad/motcore_v6_apex_pivot.py`; it builds the
assembly and self-checks it (22 numeric checks, plus the rubber template's area), and every number quoted below
that says "measured" comes from its own output.

What changed on 2026-09-17, in one paragraph: the gear stage (pinion, idlers,
corona) and the Oldham that briefly replaced it are gone. The drive out of the
cone is a **folded double cardan living inside the hollow cone**, 1:1. The cone
runs on one thin-section bearing on its neck. The four-bar moved to a 45° ray
with shorter links. The actuation is a **servo under the ceiling → torsion
spring → horn → 2.1:1 lever → link → ear on the carriage**, and all four axes are
identical rotated copies. Cube **106.4 mm**.

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
  rubber does, so a newton on the ear becomes **×1.46** of total normal force
  (ear at y = 42.9 mm, squeeze centroid s̄ = 29.4 mm, both measured). Read that
  as force, not as torque: see §8 — where the contact sits cancels out of the
  output torque exactly.
- **The box.** 145 mm (first v6) → 119 (hollow cone, gears behind the push
  point) → 120.8 (Oldham) → **106.4 mm** now, set by the servo case under the
  ceiling.

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

The band runs s = 10.0…33.4 mm from the apex on the rubber surface (L = 23.4 mm,
derived: it fills the cone and stops 2 mm short of the rim for a bonding lip).

---

## 4. Continuous rubber layer (replaces the O-rings)

v5 used `n` O-rings on the motor cone and `n−1` on the output cone, interleaved
half a pitch so the flanks met rubber-on-rubber. v6 drops them for a **uniform
rubber layer** on both cones.

Why:

- **Contact area.** Two crossed rings touch at a point; two coated cones touch
  along the whole generatrix. Far more grip for the same squeeze.
- **Zero micro-slip**, via the compensation above.
- **No pitch matching** between the two ring sets, and no `q < d` constraint.

How to make it: a cone is a developable surface, so it unrolls into a flat
annular sector (295° for the motor cone, 196° for the output). The macro writes a
1:1 cutting template, `cad/motcore_v6_flat_pattern.svg`, with a spiral seam that
crosses the contact line at a single sweeping point. **Not yet tested** — see
open questions.

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
| Link axis angles | ±45° from the Y axis, both through the origin |
| Frame pivots | radius 55 mm, on posts screwed to the wall |
| Carriage pivots (B) | radius 40.5 mm |
| Link length | 14.5 mm |
| Sets | two, one at each side in X, joined by a knuckle **on B** into one part |

Why 45°/55: with the Oldham gone the frame post was what set the cube (54.5).
Sweeping the linkage, ray 45° with the frame pivot at r = 55 brings the post
under the next constraint for 0.61% slip; ray 40° at r = 52 gets the same cube at
0.65%. The carriage pivot cannot come in below r = 40.5 without landing on the
motor cone's rim. The knuckle sits on B because a 14.5 mm link is too short for
the frame post's lug and a mid-span knuckle to share; at B the 45° ray keeps it
1.9 mm off the motor cone (swept check).

**Measured by the macro** (`apex_drift`, on the linkage's own solved pose): at
full preload the point that started at the apex moves **0.187 mm** (0.132 at
contact), **+0.140 along** the shared generatrix (slip) and **−0.124 normal** to
it (preload error, 6% of the rubber thickness). Residual slip **0.60%** of the
contact line, against v5's 7.4%. Not zero: do not quote "≈ 0".

Because the drift is outward along +Y it OPENS the contact: contact really
happens at **2.18°**, not 2.00°, first at the band's outer end, and spreads
inward as the preload grows. The usable preload travel is +0.20° past real
contact.

The **instant centre** itself wanders much further. That is normal four-bar
behaviour and not a fault: what matters is how far the apex *point* travels.

Links must not be at ±35°: that is exactly the shared generatrix direction, i.e.
the cone surfaces. Anything between 33° and 35° is inside the clearance wedge.

A flexure version was considered and **deferred to v7** — PLA creeps under
sustained load, and the project already abandoned a compliant blade in v4.

The carriage itself is now a **ring round the cone's neck** (§6) holding the
bearing, with two arms at x = ±15 whose uprights are flush with the ring (y 33…42) and run to B at 45°.

---

## 6. Drive out of the cone — folded double cardan, 1:1

The cone tilts about the apex and the output shaft does not. At distance y from
the apex their axes are apart by y·sin φ **and** at an angle φ. Whatever couples
them must take both.

```
cone (outer yoke) → ring cross → intermediate tube → solid cross → fork → output shaft
```

- **The cone is the outer yoke.** It is a hollow shell, solid only to y = 18,
  with a Ø20 bore from where its cavity is that wide out through a **neck**. The
  neck carries **one 6805 (25×37×7)** on its outside, held by the carriage ring —
  there is no carriage shaft any more — and the ring cross's X pins through its
  wall, under the bearing, which keeps them in.
- **Ring cross at y = 40, on the cone axis.** Ø10.2 bore / Ø14.6 × 5 mm: the
  output shaft passes through it. Its wall was 1.35 mm, which gave its pins
  nowhere to sit (~30 MPa on the plastic at 1 Nm, 0.5 mm beside each hole); now
  2.2 mm (~18 MPa) and 1.5 mm beside each hole. The neck's bore is Ø21 round it,
  leaving 2.0 mm of neck wall for the X pins — the limit with a 6805.
- **Intermediate: a stepped tube** running back toward the apex, 14.0 mm
  between pin planes: Ø13.5/16.5 at its tail, where it sits in the cone's
  conical cavity and cannot grow, and Ø15.6/19.6 only round the ring. The step
  sits just behind the ring because the tube rocks ~4° inside the cone, which
  moves it sideways by tenths near the ring and by more further back. Windows in
  the head let the ring's X pins out to the neck.
- **Solid cross at y = 26, on the output axis**: a **prism** 4.6 × 4 × 10 mm,
  narrow in X, long in Z. One through pin in X to both arms of a fork keyed to
  the output shaft by a D; two short pins from its long ends to the tube, each
  3.5 mm deep in it. A cube could not take two pins crossing at its centre.
- **Output shaft** in two MR105ZZ: one in a boss on the wall's inner face, one in
  the wall seat. It reaches ~16 mm in from the inner bearing to the fork.

**Why folded, and why it is not constant-velocity.** Both crosses sit in front of
the apex (the motor shaft is behind it). For a double cardan to cancel its own
velocity error the two joints must bend equally, and with the output axis flat
and the ring above it that needs one cross behind the apex. So one joint always
bends φ more than the other. Measured at full preload: **solid cross 6.6°, ring
4.2°**, residual angle error ~ (a₁² − a₂²)/4 = **0.11°** (one Hooke joint at the
tilt would be 0.025°). What the fold buys is a flat intermediate in little
length: its angle is ≈ ring_y·sin φ / (ring_y − cross_y), so moving the solid
cross back toward the apex — into the cone — is what keeps the crosses nearly
straight and the ring bore small. The bore has to swallow the offset between the
axes *at the ring*, which only depends on how far out the ring is.

**Length change.** The ring moves with the cone, the solid cross does not, so the
intermediate's span changes by **0.25 mm** over the stroke. Its ring-end pin
holes are slotted 0.4 mm instead of making the tube telescopic.

Swept clearances (macro, 9 tilts): tightest running gap **0.51 mm**, tube head
against the cone's bore. The model draws one phase of the turn only, so two
checks are analytic: the ring rocking inside the tube's head (0.34 mm) and the
prism rocking inside the tube (0.89 mm).

### Building it

**Not print-in-place.** A printed-in-place joint needs 0.3–0.4 mm of clearance
per pin, which at these radii is ~3° of backlash at the output; supports inside
the closed cone could never be removed; and Ø2 PLA pins at ~70–125 N would
creep.

**Five printed parts and eight steel Ø2 dowel pins** (ISO 8734): cone, tube,
ring, prism, fork. Each pin is **pressed into the cross** (prism or ring) and
**turns free in the part round it**:

| Pin | Pressed into | Turns in | Kept in by |
|---|---|---|---|
| through, prism ↔ fork (X) | — (free in both) | prism and fork arms | its length: sliding to the tube, it is still in both arms |
| prism ↔ tube (Z) ×2 | prism, 3.5 mm | tube tail wall | press fit; the cone's bore 1.75 mm out |
| ring ↔ tube (Z) ×2 | ring, 2.2 mm | slot in the tube head | press fit |
| ring ↔ cone (X) ×2 | ring, 2.2 mm | neck wall | press fit; the 6805 over it |

Holes: this printer runs small holes ~0.5 mm under, so print them at Ø1.5–1.8
and drill — **Ø1.9 for a press, Ø2.1 to turn**. Try both on a coupon first. A
press that comes out loose takes a drop of cyanoacrylate (super glue — not a
Loctite threadlocker, which is anaerobic and does not cure on plastic), on the
pressed side only, before the joint is assembled.

Print the ring solid (100% infill), bore vertical, so its pin holes come out
horizontal and round. PETG takes the oscillation better than PLA.

**Assembly order**, every pin reachable: prism into fork (through pin) → tube
over the prism (its two pins, from outside the tube) → ring into the tube's head
(two pins, through the slots) → the whole thing into the cone through the neck
→ ring's X pins from outside through the neck wall and the tube's windows →
6805 over the neck, trapping them → output shaft in through the wall, the ring's
bore and into the fork's D.

### Printing the carriage

The bearing seat (Ø37, along Y) and the pivot and ear holes (along X) are
perpendicular, so one of them prints lying down. The seat is the one that must
come out round, so the carriage prints **with its wall-side face on the bed**,
seat vertical:

- the 45° jogs out to the pivots B rise off the ring unsupported, and the ear's
  bridge climbs at ~27° from vertical;
- the ring runs on toward the wall past the bearing, to the ear block's own far
  face (y 45.9), so ring and ear stand on the bed together — and that extra
  length **is the bearing's stop**: a solid flange (bore Ø32) from the bearing
  down to the bed, not a lip hanging in the air;
- the bearing goes into the carriage **from the cone side** against that flange;
  then carriage and bearing slide onto the cone's neck together. The neck's
  shoulder holds the inner race on the other side, so the 6805 is located both
  ways;
- the pivot and ear holes print lying down, slightly oval: drill them to size.
  The round bosses round B have a small curved overhang underneath — the first
  thing to look at when the part comes off.

**Plan B for the ring**, if the printed one ovalises: a standard 10 × 14 mm
sintered-bronze bushing (or steel spacer) cut to 5 mm, four radial Ø2 holes
drilled through a printed jig. Its 10.0 bore is 0.2 under the design's, so the
model would need adjusting. For the prism, the steel cross out of a bought mini
cardan, if its size fits the fork and the tube.

---

## 7. Actuation

```
servo → torsion spring → horn → lever 2.1:1 → link → ear on the carriage
```

### Sweep split and the spring

The servo spline turns **±80°** (margin to an MG90's end stops). The first **20°**
bring the rubber into contact; past that the carriage has nowhere to go and the
remaining **60°** wind a **torsion spring without preload** between spline and
horn. Spring torque — not position — sets the squeeze. The horn itself only ever
swings ~25°.

Why the spring sits at the servo and not in a telescopic link: a spring at the
servo only sees force if the servo sees torque, so the chain must have a
near-constant ratio. The old crank ran near dead centre (toggle), where the servo
sees almost no torque — a spring on its shaft would barely wind. Levers and
links far from dead centre keep the ratio steady; the check is that no joint
gets within 40° of dead centre.

Why a lever at all: 1.4 mm of ear travel in 20° of horn needs a 4 mm horn, and
0.2 mm of pin slop on that is 5% of the stroke. Through a 2.1:1 lever the horn is
9.0 mm and servo-side slop reaches the carriage divided.

**The lever rule that makes it "further out is more leverage" false.** Work in =
work out: servo torque × servo angle = carriage moment × tilt. The tilt is fixed
(~2.4°), so the multiplication is fixed by how much servo sweep the stroke uses,
not by where the push lands. Pushing further out gains arm and costs travel in
the same proportion. The push point is chosen for **packaging**.

| Measured / derived | |
|---|---|
| Horn radius (derived from the split and the lever) | 9.04 mm |
| Lever | 8.0 / 3.8 about (y 39.1, z 35.0), on a post from the ceiling |
| Links | link1 7.6 mm, link2 7.5 mm |
| Ear pin | (y 42.9, z 27.5), carriage frame |
| Horn swing | 20.3° to contact, 24.8° at full preload |
| Worst transmission angle | 41.7° |
| Spring (placeholder) | 2.40 N·mm/°, 0.144 Nm after 60° = 80% of MG90 stall |
| At full preload | 34 N on the ear, 1.44 Nm about the apex, 49 N normal |

`spring_ratio` (rubber vs spring stiffness, as servo-side angle past contact) is
**15, invented**; it only sets how far the carriage still creeps past contact
(φ_preload = 2.375°).

### Packaging

- **All four axes are identical, rotated about Z. Nothing is turned over.** Each
  servo sits in its own (+X) corner of the pinwheel. (Alternate axes used to be
  assembled upside down only because two servos lying on the same floor wanted
  the same corner.)
- **Servo under the ceiling**, shaft along +X, case x 0…29, y 5…28, z 36.5…48.7.
  Below it the four-bar's upper links rise to z ≈ y + 5.7; that sets its height,
  and its top sets the cube.
- **The chain stacks back inboard under the servo's face.** The case ends at
  y = 27.8 and the chain lives further out in Y, so its X band is free: horn
  x 32.5…34.5 outboard of the face (spring between), then link1 and the lever's
  post (different Y) at 30…32, lever 26.5…29.5, link2 24…26, ear 20.5…23.5. The
  ear therefore lands beside the carriage's own +X arm and its bridge is a stub,
  rooted on the arm's 45° jog and ending in a block round the pin.
- The **servo bracket and the lever post mount to the ceiling**, not the wall.
  Screws into the ceiling are not modelled.
- Swept clearances over 9 tilts, pinned neighbours excluded: tightest **0.88 mm**
  (link1 vs spring), lever vs servo case 0.94, link1 vs case 1.00.

**Pick the servo on speed, not torque.** The binding spec is getting
free→contact done inside a push-off window of 50–150 ms: 20° is ~35 ms on an
MG90-class servo, full preload ~135 ms. Digital also matters for its small
deadband, because preload is set by small angle changes.

---

## 8. Torque estimate

Moment balance about the apex gives a closed form. The cube is **1:1** now (no
gear stage), so:

```
ΣN      = F_ear · y_ear / s̄
T_out   ≈ μ · F_ear · y_ear · sin β
```

The ear link pulls vertically, so its arm about the apex is simply its Y.

With μ = 1.3, y_ear = 42.9 mm, β = 33°: **≈ 0.030 Nm of output torque per newton
on the ear**; at the spring's 34 N, **≈ 1.0 Nm**. The overall cube ratio is
ω_out/ω_motor = sin α / sin β = **1.50** — an overdrive. The reduction the tree
needs (invariant 1) now has to live **between hubs**, and it has to beat that
1.5.

**Where the contact sits does not appear in that, and the cancellation is
exact.** Moments about the apex give `F·lever = ∫ n(s)·s ds`, because a force
perpendicular to the generatrix at distance `s` from the pivot has moment arm
exactly `s`. The output torque is `μ·∫ n(s)·(s·sin β) ds`, the same integral —
so `T = μ·sin β·F·lever` whatever the pressure distribution. Moving the rubber
outward buys friction radius and costs normal force in the same proportion.

So `lever / s̄` is **not** a torque multiplier. What it gives is the total NORMAL
force, which is what the rubber has to develop inside the available squeeze —
open question 1.

**This is an upper bound.** It assumes the rubber actually develops that normal
force within the available squeeze, which is exactly the unmeasured number below.
Do not quote it as fact.

Slip is a feature: it is a per-axis torque limiter and makes the joint
back-drivable, which matters for the passive-dynamics goal.

---

## 9. Why this design and not the alternatives

Recorded so they are not re-litigated. Each cost real time.

### Drive out of the cone

**Gear stage (pinion → idlers → corona, 1:3).** Worked, but asymmetric and
heavy on the packaging; removed. **Concentric pinion** before it: needs a free
gap angle of 10–12° whatever you do, because rotation moves the pinion ~3× faster
than the rubber (bound φc > 2·(Zc − Zp)·q / s̄, independent of where the pinion
sits). A beveloid pair has a working point at ~0.4 mm margin and module 0.85 —
rejected as too tight, not as impossible.

**Spline / crowned coupling, coaxial.** Assumed the axes only see an angle
because they cross at the apex. False: at the teeth (y ≈ 48) they are 2 mm apart,
and a spline cannot take offset — its side teeth would slide round the circle
while the top ones stay put. Modelled, rubbed 68 mm³.

**Corona on the cone shaft, always meshed; pinion/corona decoupled in free.**
Neither works; see the gear-stage note above.

**Double cardan, straight.** Works; ~40 mm long, the cube grows.

**Cardan + Oldham** (one joint per misalignment). Works, modelled on
`v7-cardan-oldham`; too long.

**Oldham alone.** Modelled on `v7-oldham`, 120.8 mm cube. Takes offset by
construction and 2.4° with 0.08 mm of flank room, but the disc nods ±0.46 mm once
a revolution, and it is a parallel-offset coupling asked to take an angle.

**Folded cardan with the tube on the cone and the crosses just past the carriage.**
Between the carriage (y 38) and the wall there is not enough length: the
intermediate has to rise ~20°, its corner reached the wall and the ring bore
rubbed the shaft. Moving the solid cross *into* the cone is what made it fit.

### Actuation

**Telescopic spring link from a crank on the floor.** Replaced: its crank ran
near dead centre, and with the smaller cube the servo landed on the lower link.
**Scotch yoke with slot and rail** before that. **Cam (disc or drum)** for a
variable ratio: unnecessary, the series spring already gives fast-then-powerful.

**Servo standing up in a corner.** Its horn turns in a horizontal plane and the
carriage moves in a vertical one: the linkage would need ball joints.

**Servo lying at mid-height in the side corner.** Lands on the neighbouring
axis' cone and carriage ring; the free band between them is 9.6 mm against a
12.2 mm case.

**Servo flipped (bottom to the neighbour's wall, shaft inward).** Its chain
would sit on the four-bar's upper links, and the ear would end up near the apex
with a ~4–7 mm horn.

**Turning alternate axes over.** Only needed while servos lay on the floor.

**Belt or pulley reduction at the joint.** Breaks the stacking symmetry.

**Flexure virtual pivot.** Deferred to v7 (PLA creep).

---

## 10. Open questions

1. **Rubber stiffness — the number everything hangs on.** How much normal force
   does a 2 mm rubber layer develop at ~0.2 mm of squeeze? Until this is measured
   the output torque is a guess. It decides the spring, the servo and whether
   the cones need to grow. No bench time for now: design for a range.
2. **Spring rate and spring_ratio.** The torsion spring (2.4 N·mm/°) is sized to
   the servo, not to the rubber; spring_ratio = 15 is invented.
3. **Making the rubber layer.** The flat pattern exists; wrapping it onto the
   cone is untested: adhesive, seam, thickness uniformity.
4. **Sizing, not layout.** Every pair of parts is checked at every stop, the
   links, the cardan and the actuation chain are swept as distances, and each
   axis is checked against its three neighbours. Nothing has been loaded, FEA'd
   or printed. Weakest-looking: the printed ring (pins at ~18 MPa, oscillating
   all the time), the output shaft's ~16 mm reach to the fork, the lever post
   from the ceiling.
5. **Central motor sizing.** Depends on (1) and on how many axes are engaged at
   once. Demands add, they do not divide.
6. **Where the reduction lives.** Each cube is now 1.5:1 overdrive; the chain
   between hubs has to supply the tree's reduction.

---

## 11. Next steps

1. Print one axis. Everything above is geometry that has never been in a
   printer: start with the cone + cardan + bearing, which is the new risk.
2. Model the ceiling screws for the servo bracket and the lever post, and size
   the torsion spring as a real part (wire, coils, legs).
3. Run the rubber measurement when there is time.
4. `cad/clutch_geometry_v6.html` is **stale** (telescopic link, gear front
   view): update or retire it before publishing.

---

## 12. Ideas raised 2026-09-15 — status

1. **Material at the cone tip.** Done: solid to y = 18.
2. **Four-bar links near the motor cone.** Checked by a swept distance (1.90 mm
   at the knuckle), not just at three stops.
3. **The rubber band's edge is not square.** Open.
4. **Get rid of the corona.** Done — replaced by the folded cardan (§6).
5. **Design the servo link for real.** Superseded — the telescopic link is gone
   (§7).
