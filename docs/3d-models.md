# 3D Models — Motcore v0.2

> **Status:** 🔄 In development. This page will be updated as models are completed.

---

## Overview

Version 0.2 replaces the LEGO Technic proof-of-concept with parametric 3D-printed components designed in **FreeCAD**. The goal is a reproducible, printable design using standard mechanical components (bearings, smooth rods, O-rings).

---

## Design Principles

- **Parametric** — all key dimensions are driven by a spreadsheet in FreeCAD, making it easy to adapt to different motor sizes or axis counts
- **Print-in-place friendly** — minimize post-processing and hardware inserts
- **Standard components** — use off-the-shelf bearings, rods, and O-rings where possible
- **Compliant mechanisms** — the friction engagement system uses flexure-based compliance instead of rigid joints

---

## Planned Components

| Component | Status | File |
|---|---|---|
| Central motor mount | 🔄 In progress | `cad/MotorMount.FCStd` |
| Friction wheel (TPU) | ⬜ Planned | — |
| Output axis carrier | ⬜ Planned | — |
| Servo bracket | ⬜ Planned | — |
| Main frame | ⬜ Planned | — |
| Engagement arm (compliant) | ⬜ Planned | — |

---

## Software Requirements

- [FreeCAD](https://www.freecad.org/) 0.21 or later
- No additional workbenches required for basic parts
- **Lattice2** workbench recommended for array operations

---

## Printing Recommendations

| Parameter | Recommendation |
|---|---|
| Layer height | 0.2mm |
| Infill | 40% (structural parts), 20% (brackets) |
| Material (rigid parts) | PLA or PETG |
| Material (friction wheels) | TPU 95A |
| Supports | Only where indicated |

---

## Contributing CAD Models

If you'd like to contribute FreeCAD models:

1. Use the existing parametric spreadsheet as the source of truth for dimensions
2. Follow the naming convention: `ComponentName_vX.X.FCStd`
3. Export a STEP file alongside the native FreeCAD file
4. Include a screenshot in `docs/images/`
5. Open an issue before starting major design work to avoid duplication

---

> 📷 Render images and assembly animations will be added here as the design matures.
