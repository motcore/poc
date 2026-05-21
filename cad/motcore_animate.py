"""
motcore_animate.py — Motcore output-axis tilt animation
========================================================
Rocks the cone wheel + pivoting D-shaft back and forth around the UJ pivot
point so you can see how the clutch engages the friction-wheel rubber rings.

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
cube_size    = 100.0
plate_thick  =   8.0
hs           = cube_size / 2           # = 50 mm
z_center     = cube_size / 2           # = 50 mm  (vertical centre of the cube)

wheel_sep    =  30.0
wheel_thick  =   4.0
rubber_thick =   2.0
rubber_id    =  18.0
hub_body_dia =  16.0
shaft_dia    =   8.0

cone_r_large = rubber_id   / 2                           # = 9 mm
cone_r_small = hub_body_dia / 2                          # = 8 mm
uj_y         = -(hs - plate_thick)                       # = −42 mm
cone_tip_y   = -(rubber_id / 2)                          # = −9 mm
arm_length   = abs(cone_tip_y - uj_y)                    # = 33 mm
z_clearance  = (wheel_sep / 2
                - wheel_thick  / 2
                - rubber_thick
                - cone_r_large)                          # = 2 mm
cone_angle   = math.degrees(math.asin(z_clearance / arm_length))   # ≈ 3.5°

# Rotation used when the parts were placed (local +Z → world −Y)
rot_out_y    = App.Rotation(App.Vector(1, 0, 0), 90)

# ═══════════════════════════════════════════════════════════════════
# ANIMATION SETTINGS
# ═══════════════════════════════════════════════════════════════════
FRAMES       = 80     # frames per full oscillation cycle
CYCLES       = 3      # how many times to rock back and forth
FRAME_DELAY  = 0.04   # seconds between frames  (≈ 25 fps)
PAUSE_AT_END = 0.25   # extra pause at the engaged position (shows contact)

# Tilt slightly past the contact angle so it's clear the cone touches the ring
MAX_ANGLE    = cone_angle * 1.15

# Parts created by motcore_plates.py that move together as a rigid body
MOVING_PARTS = ["ConeWheel_Front", "OutputShaft_Pivot_REF"]

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

# Neutral placement shared by cone wheel and pivoting shaft
NEUTRAL_PL = App.Placement(App.Vector(0, cone_tip_y, z_center), rot_out_y)

# UJ pivot point in world coordinates
PIVOT = App.Vector(0, uj_y, z_center)


def tilt_placement(base_pl, angle_deg):
    """
    Return a new Placement produced by rotating `base_pl` by `angle_deg`
    around the global X-axis (YZ-plane tilt) centred on the UJ pivot point.
    Positive angle → cone tip moves toward +Z (upper friction wheel).
    """
    rot     = App.Rotation(App.Vector(1, 0, 0), angle_deg)
    delta   = base_pl.Base - PIVOT
    new_pos = PIVOT + rot.multVec(delta)
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

# Save original placements so we can restore them afterwards
saved_placements = {n: doc.getObject(n).Placement for n in MOVING_PARTS}

print("=" * 55)
print(f"Motcore tilt animation")
print(f"  UJ pivot    : y = {uj_y:.1f} mm")
print(f"  Arm length  : {arm_length:.1f} mm")
print(f"  Cone angle  : ±{cone_angle:.2f}°  (contact at ±{z_clearance:.1f} mm in Z)")
print(f"  Max tilt    : ±{MAX_ANGLE:.2f}°")
print(f"  Cycles      : {CYCLES}  ×  {FRAMES} frames @ {1/FRAME_DELAY:.0f} fps")
print("=" * 55)

try:
    for cycle in range(CYCLES):
        for frame in range(FRAMES):
            # Smooth sine oscillation: neutral → up → neutral → down → neutral
            t     = frame / FRAMES                        # 0 … 1
            angle = MAX_ANGLE * math.sin(2 * math.pi * t)
            apply_tilt(doc, angle)

            # Brief pause when the cone reaches the extreme (contact) position
            if abs(t - 0.25) < (1 / FRAMES) or abs(t - 0.75) < (1 / FRAMES):
                time.sleep(PAUSE_AT_END)
            else:
                time.sleep(FRAME_DELAY)

finally:
    # Always restore parts to neutral, even if the user presses Escape / an error occurs
    for name, pl in saved_placements.items():
        obj = doc.getObject(name)
        if obj:
            obj.Placement = pl
    doc.recompute()
    Gui.updateGui()
    print("Animation finished — parts restored to neutral position.")
