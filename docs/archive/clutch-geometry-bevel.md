# Motcore Bevel Clutch — Geometry & Equations (v2, ARCHIVED)

> **⚠️ Superseded.** This document describes the **v2 bevel cone** generation
> (May 2026), two generations behind the active design. It is kept for the
> record only — do not build from it and do not re-propose the tilting-shaft
> family it belongs to.
>
> - Active geometry: [../clutch-geometry.md](../clutch-geometry.md) (v5)
> - Why it was superseded: [../design-evolution.md](../design-evolution.md)

The interactive visualizer for this generation is at `cad/clutch_geometry.html`.
This document derives the equations implemented there.

---

## 1. Design intent

A single Z-axis motor must drive output shafts along ±X and ±Y. Each output
shaft has a **bevel cone friction clutch**: a truncated cone mounted on the
output shaft that, when the shaft is tilted by a servo, makes surface contact
with the motor cone on the Z shaft and transmits torque by friction.

The design goal is **generator-line contact** — both cones share a common apex
and are tangent along a common generator line.

---

## 2. The four free parameters

```
A  — engagement angle (degrees): how much the servo tilts the output shaft
B  — arm length (mm): distance along the output shaft from UJ pivot to the
     large face of the clutch cone
C  — clutch cone large-face radius (mm)
D  — motor cone large-face radius (mm)
```

All other dimensions are derived from these four.

---

## 3. Coordinate system

The geometry is described in the Y-Z cross-section:

```
Z (up)
│                     ╔═══════╗
│             apex ●──╢ motor ╟── input shaft at y = L
│            ╱    └───╚═══════╝
│           ╱  ← contact edge (red)
│          ╱ ← clutch cone (tilted A° from horizontal)
│─────────●──────────────────── Y (horizontal)
│        UJ pivot (origin)
```

- **UJ pivot** = world origin (y=0, z=0)
- **Y** = horizontal, pointing toward the input shaft
- **Z** = vertical, pointing up (input shaft direction)
- **Input shaft** sits at a horizontal distance L from the UJ (derived)

---

## 4. Deriving L — the key constraint

When the output shaft is tilted by angle A, the clutch cone large face is at:

```
lfc_y = B·cos(A)
lfc_z = B·sin(A)
```

The perpendicular to the shaft axis (pointing toward the motor) is (−sin A, cos A).
The contact edge endpoint (where the clutch cone large face meets the motor cone) is:

```
contact_y = B·cos(A) − C·sin(A)
contact_z = B·sin(A) + C·cos(A)
```

For this point to lie on the motor cone large face, which has radius D centred
on the input shaft at y = L, we need:

```
contact_y = L − D
```

Therefore:

```
L = D + B·cos(A) − C·sin(A)          [Key derived dimension]
```

L is not a free parameter — it is fully determined by A, B, C, D.

---

## 5. Shared apex position

The output shaft axis, tilted by A from horizontal, passes through the UJ origin
with slope tan(A). It meets the input shaft (vertical line at y = L) at:

```
apex_y = L
apex_z = L · tan(A)                   [Shared apex]
```

Both cones have their apex at this point.

---

## 6. Motor cone geometry

The motor cone's large face is horizontal at height contact_z, centred on the
input shaft. Its height (from apex to large face) is:

```
h_motor = contact_z − apex_z
        = B·sin(A) + C·cos(A) − L·tan(A)
```

---

## 7. Cone half-angles

```
dist_apex_lfc = √[(lfc_y − apex_y)² + (lfc_z − apex_z)²]

α_clutch = arctan( C / dist_apex_lfc )
α_motor  = arctan( D / h_motor )
```

---

## 8. Bevel condition

For generator-line contact, the sum of the two half-angles must equal the angle
between the shaft axes:

```
α_clutch + α_motor = 90° − A          [Bevel condition]
```

**This is automatically satisfied** by the shared-apex construction — you do not
need to check or enforce it. Choosing any A, B, C, D and computing L as above
guarantees the bevel condition holds.

---

## 9. Self-locking condition

With a conical surface, friction has a component along the cone axis. Self-locking
occurs when the servo cannot disengage the clutch:

```
μ ≥ tan(α_clutch)    ← self-locked, cannot disengage

μ < tan(α_clutch)    ← safe, servo can disengage
```

At small A (e.g. 8°), α_clutch is large (~49°), so tan(α_clutch) ≈ 1.1 — well
above any practical friction coefficient. Self-locking is not a concern at small
engagement angles.

---

## 10. Summary table

| Quantity | Formula |
|----------|---------|
| L (UJ to input shaft) | D + B·cos(A) − C·sin(A) |
| Apex position | (L, L·tan(A)) in (y, z) |
| Contact point | (L−D, B·sin(A)+C·cos(A)) |
| Motor cone height | B·sin(A) + C·cos(A) − L·tan(A) |
| α_clutch | arctan(C / dist(apex, lfc)) |
| α_motor | arctan(D / h_motor) |
| Bevel condition | α_clutch + α_motor = 90° − A (auto) |
| Self-locking limit | μ < tan(α_clutch) |

---

## 11. Default parameter values

| Parameter | Value |
|-----------|-------|
| A | 8° |
| B | 35 mm |
| C | 9 mm |
| D | 9 mm |
| L (derived) | ≈ 42.4 mm |
| apex_z (derived) | ≈ 6.0 mm |
| h_motor (derived) | ≈ 7.8 mm |
| α_clutch ≈ α_motor | ≈ 49° |
