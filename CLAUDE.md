# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis friction drive actuator**.
One central stepper motor (Z axis) drives multiple output axes (±X, ±Y) via
servo-controlled **bevel cone friction clutches**. Each output axis has its own
clutch; engaging a clutch connects that axis to the rotating central shaft.

All mechanical design files are in `cad/`. Firmware in `software/`.

---

## Clutch geometry — the four parameters

The bevel clutch is fully described by four free parameters. Everything else is derived.

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Engagement angle | **A** | Degrees the output shaft tilts up from horizontal to engage |
| Arm length | **B** | Distance along the output shaft from UJ pivot to clutch cone large face |
| Clutch cone radius | **C** | Large-face radius of the clutch (output) cone |
| Motor cone radius | **D** | Large-face radius of the motor (input) cone |

Default values: A = 8°, B = 35 mm, C = 9 mm, D = 9 mm.

---

## Coordinate system (Y-Z cross-section)

- **UJ pivot** = world origin (0, 0)
- **Y axis** = horizontal, pointing toward the input shaft
- **Z axis** = vertical, pointing up (= input shaft direction)
- **Input shaft** lies along the Z axis at y = L (derived)

---

## Governing equations

### L — horizontal distance from UJ to input shaft axis (derived, not free)

```
L = D + B·cos(A) − C·sin(A)
```

This follows from the contact condition: the clutch cone large face edge must
touch the motor cone large face edge at the input shaft axis.

### Shared apex

The output shaft axis, when tilted by A, meets the input shaft (Z axis) at:

```
apex_y = L
apex_z = L · tan(A)
```

### Contact point (common generator endpoint)

```
contact_y = L − D
contact_z = B·sin(A) + C·cos(A)
```

### Motor cone height

```
h_motor = contact_z − apex_z
        = B·sin(A) + C·cos(A) − L·tan(A)
```

### Cone half-angles

```
α_clutch = arctan( C / dist(apex, lfc) )
α_motor  = arctan( D / h_motor )
```

where `lfc` = large face centre of clutch cone = (B·cos(A), B·sin(A)).

### Bevel condition (automatically satisfied)

```
α_clutch + α_motor = 90° − A
```

Guaranteed by the shared-apex construction. No need to enforce separately.

### Self-locking constraint

```
μ < tan(α_clutch)
```

If μ ≥ tan(α_clutch) the clutch self-locks and the servo cannot disengage it.

---

## Design invariants — do not violate these

1. **Both cones share the same apex.** This is the geometric foundation.
2. **Motor cone opens upward (+Z).** Large face above apex.
3. **Clutch cone opens away from UJ along the tilted shaft.** Large face at distance B.
4. **L must be positive** — if L ≤ 0 the geometry is invalid.
5. **h_motor must be positive** — apex must be below the contact point.
6. **µ < tan(α_clutch)** — required for the servo to disengage.

---

## Files

| File | Purpose |
|------|---------|
| `cad/clutch_geometry.html` | **Primary geometry reference** — interactive Y-Z visualizer |
| `cad/motcore_plates.py` | FreeCAD macro — 3D solid model |
| `cad/motcore_animate.py` | FreeCAD macro — clutch engagement animation |
| `docs/clutch-geometry.md` | Full derivation of the governing equations |
| `software/controller/` | Arduino firmware — master (touchscreen UI) |
| `software/driver/` | Arduino firmware — receiver (motor + servo control) |
