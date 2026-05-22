# Motcore — AI Context & Design Rules

## What this project is

Motcore is an open hardware **multi-axis friction drive actuator**.
One central stepper motor (Z axis) drives multiple output axes (±X, ±Y) via
servo-controlled **bevel cone friction clutches**. Each output axis has its own
clutch; engaging a clutch connects that axis to the rotating central shaft.

All mechanical design files are in `cad/`. Firmware in `software/`.

---

## Clutch geometry — the four parameters

The bevel clutch is fully described by four parameters. Everything else is derived.

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Engagement angle | **A** | Degrees the output shaft tilts to engage the clutch (servo travel) |
| Arm length | **B** | Distance along output shaft from UJ pivot to clutch cone large face |
| Clutch cone radius | **C** | Large-face radius of the clutch (output) cone |
| Motor cone radius | **D** | Large-face radius of the motor (input) cone |

**Contact condition** (C and D must match at the shared apex):
```
D = B · tan(α_m)      where α_m = arctan(D / h_m)
C = B · tan(α_e)      where α_e = arctan(C / h_e)
```

For **1:1 ratio**: C = D → both cones have equal half-angle α.

---

## Governing equations

### Cone half-angle (1:1 ratio)
```
α = (90° − A) / 2
```

### Bevel condition (automatically satisfied by shared-apex construction)
```
α_e + α_m = 90° − A
```
This is a geometric consequence of both cones sharing the same apex. You do not
need to enforce it explicitly — choosing A, B, C, D satisfies it automatically.

### Large-face radius from cone height h and half-angle α
```
r = h · tan(α)
```

### Motor cone apex position (world Z, above cube centre)
```
z_apex_motor = z_center + arm_length · sin(A)
```
where `arm_length = |uj_y|` = distance from UJ pivot to the output shaft apex
in the neutral (disengaged) position.

### Self-locking constraint (servo must be able to disengage)
```
μ < tan(α)
```
If friction coefficient μ ≥ tan(α) the clutch self-locks and the servo cannot
disengage it. For α ≈ 25° (A = 40°), tan(α) ≈ 0.47 — well above typical
rubber/plastic μ ≈ 0.3–0.4, so disengagement is reliable.

### Minimum engagement angle to avoid neutral-state cone overlap
```
A_min ≈ 37°    (empirical, for cube_size=100, plate_thick=8, motor_cone_h=20)
```
At A < 37° the motor cone large face descends below z_center and overlaps the
clutch cone volume in the neutral (disengaged) position.

---

## Key coordinate system

- **Origin**: centre of bottom plate (world 0, 0, 0)
- **Z axis**: input shaft (motor), pointing up
- **Cube centre**: (0, 0, 50) mm — the **shared apex** of both cones
- **UJ pivot**: (0, −42, 50) mm — output shaft tilts around this point
- Output axes are along ±X and ±Y; each has its own clutch at z = 50 mm

---

## Current parameter values (motcore_plates.py)

```python
cube_size         = 100.0  # mm
plate_thick       =   8.0  # mm
shaft_dia         =   8.0  # mm  (D-shaft for output, round for input)
cone_engage_angle =  40.0  # deg  ← parameter A
motor_cone_h      =  20.0  # mm
clutch_cone_h     =  20.0  # mm
# Derived:
alpha             =  25.0  # deg  = (90 - 40) / 2
motor_cone_r      ≈  9.3   # mm  = 20 · tan(25°)
uj_y              = -42.0  # mm
z_apex_motor      ≈ 77.0   # mm  = 50 + 42 · sin(40°)
```

---

## Files

| File | Purpose |
|------|---------|
| `cad/motcore_plates.py` | FreeCAD macro — builds plates, motor cone, clutch cone, shaft refs |
| `cad/motcore_animate.py` | FreeCAD macro — animates clutch engagement (run after plates macro) |
| `cad/clutch_geometry.html` | Interactive Y-Z cross-section visualizer (sliders for A, B, C, D) |
| `software/controller/` | Arduino firmware — master (touchscreen UI) |
| `software/driver/` | Arduino firmware — receiver (motor + servo control) |
| `docs/clutch-geometry.md` | Full derivation of the bevel clutch equations |

---

## Design invariants — do not violate these

1. **Both cones must share the same apex.** This is the geometric foundation.
   Moving either apex independently breaks the bevel condition and causes
   point contact or interference instead of generator-line contact.

2. **Motor cone opens downward (−Z).** Large face below apex, apex at top.
   The clutch cone opens away from the motor (along −Y in neutral).

3. **cone_engage_angle ≥ 37°** to avoid neutral-state overlap.

4. **µ < tan(α)** — verify after changing friction material or cone angle.

5. **Clutch cone apex = (0, 0, z_center) in neutral position.** The servo
   swings it to (0, ~0, z_apex_motor) at engagement — the two apices coincide
   on the Z axis only at full engagement angle A.
