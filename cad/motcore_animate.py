"""
motcore_animate.py — Motcore v0.3 bevel-cone engagement animation
=================================================================
Tilts the clutch cone + output shaft around the shared apex (UJ pivot at
cube centre) to show how the clutch cone surface meets the motor cone surface.

Run AFTER motcore_plates.py has already built the FreeCAD document.

FreeCAD Macro menu → Macros… → select this file → Execute
"""

import FreeCAD    as App
import FreeCADGui as Gui
import Part
import math
import time

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS  (keep in sync with motcore_plates.py)
# ═══════════════════════════════════════════════════════════════════
cube_size         = 100.0
plate_thick       =   8.0
hs                = cube_size / 2    # = 50 mm
z_center          = cube_size / 2    # = 50 mm

cone_engage_angle = 5.0              # deg — shaft tilt to engage
alpha             = (90.0 - cone_engage_angle) / 2.0   # = 42.5°

# UJ pivot — close to the cube wall, same as original design
uj_y = -(hs - plate_thick)          # = −42 mm
PIVOT = App.Vector(0, uj_y, z_center)   # = (0, −42, 50)

# ═══════════════════════════════════════════════════════════════════
# ANIMATION SETTINGS
# ═══════════════════════════════════════════════════════════════════
FRAMES       = 80     # frames per full oscillation cycle
CYCLES       = 3      # oscillations
FRAME_DELAY  = 0.04   # seconds (≈ 25 fps)
PAUSE_AT_END = 0.30   # extra pause at full engagement

# Tilt slightly past the contact angle so contact is clearly visible
MAX_ANGLE    = cone_engage_angle * 1.15

# Parts created by motcore_plates.py that move as a rigid body
MOVING_PARTS = ["ClutchCone_Front", "OutputShaft_Pivot_REF"]

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

# Rotation that places local +Z along world −Y (output shaft direction)
rot_out_y  = App.Rotation(App.Vector(1, 0, 0), 90)

# Neutral placement: apex at cube centre, shaft along −Y
NEUTRAL_PL = App.Placement(App.Vector(0, 0, z_center), rot_out_y)


def tilt_placement(base_pl, angle_deg):
    """
    Return a new Placement produced by rotating `base_pl` by `angle_deg`
    around the global X-axis, centred on PIVOT (shared apex / UJ point).
    Positive angle → clutch cone tilts toward +Z (upper motor cone).

    Because PIVOT == base_pl.Base (the apex IS the pivot), the position
    stays fixed and only the orientation changes — the apex is always at
    the cube centre regardless of tilt angle.
    """
    rot     = App.Rotation(App.Vector(1, 0, 0), angle_deg)
    delta   = base_pl.Base - PIVOT          # = (0,0,0) for our geometry
    new_pos = PIVOT + rot.multVec(delta)    # = PIVOT  (apex stays put)
    new_rot = rot.multiply(base_pl.Rotation)
    return App.Placement(new_pos, new_rot)


def apply_tilt(doc, angle_deg):
    """Move all animated parts to the given tilt angle and refresh the view."""
    pl = tilt_placement(NEUTRAL_PL, angle_deg)
    for name in MOVING_PARTS:
        obj = doc.getObject(name)
        if obj:
            obj.Placement = pl
    doc.recompute()
    Gui.updateGui()


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

doc = App.ActiveDocument
if doc is None:
    raise RuntimeError(
        "No active FreeCAD document found.\n"
        "Run motcore_plates.py first, then run this animation macro."
    )

missing = [n for n in MOVING_PARTS if doc.getObject(n) is None]
if missing:
    raise RuntimeError(
        f"Parts not found in document: {missing}\n"
        "Run motcore_plates.py first."
    )

saved_placements = {n: doc.getObject(n).Placement for n in MOVING_PARTS}

print("=" * 58)
print("Motcore v0.3 — bevel-cone engagement animation")
print(f"  UJ pivot:          (0, {uj_y}, {z_center}) mm")
print(f"  Cone half-angle:   α = {alpha:.2f}°  (both cones)")
print(f"  Engagement tilt:   {cone_engage_angle}°")
print(f"  Max tilt shown:    {MAX_ANGLE:.2f}°")
print(f"  Cycles:            {CYCLES} × {FRAMES} frames @ {1/FRAME_DELAY:.0f} fps")
print("=" * 58)

try:
    for cycle in range(CYCLES):
        for frame in range(FRAMES):
            t     = frame / FRAMES
            angle = MAX_ANGLE * math.sin(2 * math.pi * t)
            apply_tilt(doc, angle)

            if abs(t - 0.25) < (1 / FRAMES) or abs(t - 0.75) < (1 / FRAMES):
                time.sleep(PAUSE_AT_END)
            else:
                time.sleep(FRAME_DELAY)

finally:
    for name, pl in saved_placements.items():
        obj = doc.getObject(name)
        if obj:
            obj.Placement = pl
    doc.recompute()
    Gui.updateGui()
    print("Animation finished — parts restored to neutral position.")
