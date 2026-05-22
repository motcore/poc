# Motcore Bevel Clutch — Geometry & Equations

This document derives the governing equations of the bevel friction clutch used
in Motcore v0.3. The interactive visualizer is at
`cad/clutch_geometry.html` (also accessible via htmlpreview.github.io).

---

## 1. Design intent

A single Z-axis motor must drive output shafts along ±X and ±Y. Each output
shaft has a **bevel cone friction clutch**: a truncated cone mounted on the
output shaft that, when the shaft is tilted by a servo, makes surface contact
with the motor cone on the Z shaft and transmits torque by friction.

The design goal is **generator-line contact** (not point, not interference) so
that torque is distributed along a line and the clutch can disengage cleanly.

---

## 2. The four parameters

```
A  — engagement angle (degrees): how much the servo tilts the output shaft
B  — arm length (mm): distance along the output shaft from the UJ pivot
     to the large face of the clutch cone
C  — clutch cone large-face radius (mm)
D  — motor cone large-face radius (mm)
```

All other dimensions are derived from these four.

---

## 3. Geometry in the Y-Z plane

In the neutral (disengaged) position the output shaft lies along −Y. When the
servo engages, it tilts the shaft by angle A around the UJ pivot.

```
Z
│         apex (shared)
│         ╱╲
│        ╱  ╲  ← motor cone (opens downward, half-angle α)
│       ╱    ╲
│──────●──────── z_center = 50 mm
│       ╲    ╱
│        ╲  ╱  ← clutch cone (tilted A° from −Y axis, half-angle α)
│         ╲╱
│
│         UJ pivot at y = −42 mm
│
Y ──────────────────────
```

Key points (at engagement):
- **Shared apex**: on the Z axis, at z = z_center + arm · sin(A)
- **Contact generator**: the line along both cone surfaces from apex to the
  shared large-face edge

---

## 4. Bevel condition

For generator-line contact, the two cone surfaces must be tangent along a
common generator. This requires that both cones share exactly one apex, and
that the sum of their half-angles equals the angle between their axes:

```
α_e + α_m = angle_between_axes
```

At engagement the output shaft axis makes angle (90° − A) with the Z axis
(since A is measured from the XY plane). Therefore:

```
angle_between_axes = 90° − A

→  α_e + α_m = 90° − A          [Bevel condition]
```

**This condition is automatically satisfied** by the shared-apex construction.
Choosing any A, B, C, D and computing the cone half-angles from them
geometrically guarantees the bevel condition holds. You do not need to check it
separately.

---

## 5. Cone half-angles from the parameters

The clutch cone large face is at distance B from the UJ pivot along the output
shaft. Its large-face radius is C. Therefore:

```
α_e = arctan(C / B)
```

The motor cone large-face radius is D. For its apex to coincide with the clutch
cone apex at engagement, the motor cone height h_m satisfies:

```
h_m = D / tan(α_m)    →    α_m = arctan(D / h_m)
```

At engagement the apex sits at:

```
z_apex = z_center + arm_length · sin(A)
```

where `arm_length` is the UJ-pivot-to-apex distance in neutral position.

---

## 6. 1:1 ratio condition

Bevel gear ratio = sin(α_m) / sin(α_e).

For **1:1 ratio**: α_m = α_e = α, which means C = D (equal large-face radii).

Combined with the bevel condition:

```
2α = 90° − A

→  α = (90° − A) / 2            [Half-angle for 1:1 ratio]
```

Example: A = 40°  →  α = 25°.

---

## 7. Large-face radius from cone height

Given cone height h and half-angle α:

```
r = h · tan(α)
```

Motcore uses equal heights for both cones (h = 20 mm), so both radii are equal
(C = D) and are fully determined by A.

---

## 8. Motor cone apex position

In neutral, the clutch cone apex is at world position (0, 0, z_center). The UJ
pivot is at (0, uj_y, z_center) where uj_y = −(cube_size/2 − plate_thick).

When the servo tilts the shaft by angle A, the apex traces an arc of radius
`arm_length = |uj_y|` around the UJ pivot. Its Z coordinate becomes:

```
z_apex_motor = z_center + arm_length · sin(A)
```

The motor cone apex must be placed at this Z coordinate so the two apices
coincide at full engagement.

---

## 9. Self-locking condition

The clutch must be disengageable by the servo. With a conical surface, the
friction force has a component along the cone axis. Self-locking occurs when:

```
μ ≥ tan(α)    ← clutch cannot be disengaged
```

Safe operation requires:

```
μ < tan(α)    [Self-locking avoidance condition]
```

For A = 40°, α = 25°, tan(25°) ≈ 0.47. Typical rubber-on-plastic μ ≈ 0.3–0.4,
so the margin is comfortable.

Choosing a **smaller A** (smaller α) reduces this margin. The minimum practical
A depends on the friction material; for rubber μ ≈ 0.4, the limit is
α > arctan(0.4) ≈ 22°, i.e. A < 46°.

---

## 10. Neutral-state clearance

For the motor cone not to overlap the clutch cone in the disengaged position,
the motor cone large face (at z = z_apex_motor − h_m) must stay above z_center:

```
z_apex_motor − h_m > z_center

→  arm_length · sin(A) > h_m

→  A > arcsin(h_m / arm_length)    [Minimum engagement angle]
```

With arm_length = 42 mm, h_m = 20 mm: A_min = arcsin(20/42) ≈ 28.5°. In
practice a margin of ~10° is advisable, giving A ≥ 37–40°.

---

## 11. Summary table

| Equation | Formula |
|----------|---------|
| Bevel condition | α_e + α_m = 90° − A |
| 1:1 half-angle | α = (90° − A) / 2 |
| Cone radius | r = h · tan(α) |
| Apex Z at engagement | z_apex = z_center + arm · sin(A) |
| Self-locking limit | μ < tan(α) |
| Minimum A (clearance) | A > arcsin(h_m / arm) |

---

## 12. Reference: current values (A = 40°)

| Quantity | Value |
|----------|-------|
| A (engagement angle) | 40° |
| α (half-angle, both cones) | 25° |
| B (arm length, UJ → apex) | 42 mm |
| C = D (large-face radius) | ≈ 9.3 mm |
| h_m = h_e (cone height) | 20 mm |
| z_apex at engagement | ≈ 77 mm |
| tan(α) | 0.466 |
| Self-locking margin (μ < 0.47) | comfortable for rubber/plastic |
