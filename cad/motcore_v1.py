# Motcore v1 — O-ring Friction Clutch
# FreeCAD Python Macro
#
# Flat motor disc on vertical Z-shaft.
# Four output shafts (±X, ±Y) are horizontal and tilt ±A around a UJ pivot
# near the cube wall to press their O-ring wheel against the top or bottom
# face of the motor disc.
#
# Coordinate system:
#   Origin  = cube centre (= motor shaft axis at mid-height)
#   Z       = motor shaft, upward
#   X, Y    = output shaft directions
#
# Run from FreeCAD: Macro → Macros → motcore_v1.py → Execute

import FreeCAD as App
import Part
import math

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS  ← edit here, then re-run
# ═══════════════════════════════════════════════════════════════════

A_deg         = 3.0   # deg  — engagement angle (shaft tilt from horizontal)
B             = 35.0  # mm   — UJ pivot → output wheel centre (along shaft)
R             = 20.0  # mm   — contact radius (O-ring outer edge = motor disc rim)
dw            = 2.5   # mm   — O-ring wire diameter

wall_thick    = 4.0   # mm   — cube wall thickness
shaft_dia     = 5.0   # mm   — output shaft diameter
motor_shaft_d = 8.0   # mm   — motor (Z) shaft diameter
cube_h        = 70.0  # mm   — cube internal height (top/bottom plates not modelled yet)

# ═══════════════════════════════════════════════════════════════════
# DERIVED  (do not edit)
# ═══════════════════════════════════════════════════════════════════

A   = math.radians(A_deg)
cA  = math.cos(A)
sA  = math.sin(A)

WT  = 3.2 * dw        # wheel / disc thickness (groove walls + groove width)
Rw  = R - 0.4 * dw    # structural wheel radius (O-ring outer edge sits at R)

# Visualiser geometry (Y = along output shaft toward motor, Z = up):
#   contact point at engagement: (contactY, contactZ) from UJ
#   motor shaft at motorY from UJ
contactY = B * cA - R * sA
contactZ = B * sA + R * cA
motorY   = contactY + R   # = distance from UJ to motor shaft axis

# 3D world: motor shaft at (0,0). UJ for each axis at ±motorY from centre.
uj_dist   = motorY                       # UJ distance from cube centre
cube_half = uj_dist + wall_thick         # half cube side (outer wall)
cube_size = 2 * cube_half

# Motor disc spans ±contactZ in Z (top face = +contactZ, bottom face = −contactZ)
disc_vr = R + WT / 2     # visual radius (slightly larger, geometry unchanged)
disc_bot = -contactZ      # bottom face Z
disc_h   = 2 * contactZ   # total disc height

# Output wheel centre: at B from UJ toward cube centre
wc_dist = uj_dist - B     # distance from cube centre to wheel centre


# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def v(x, y, z):
    return App.Vector(x, y, z)


def add(doc, name, shape, color=(0.8, 0.8, 0.8), transparency=0):
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if HAS_GUI:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency
    return obj


def cyl(r, h, base, direction=v(0, 0, 1)):
    return Part.makeCylinder(r, h, base, direction)


def av(axis, dist):
    """Point at dist along axis unit vector from origin."""
    return v(axis.x * dist, axis.y * dist, axis.z * dist)


def make_motor_disc():
    disc = cyl(disc_vr, disc_h, v(0, 0, disc_bot))
    bore = cyl(motor_shaft_d / 2 + 0.1, disc_h + 2, v(0, 0, disc_bot - 1))
    return disc.cut(bore)


def make_output_wheel(axis):
    """
    Disc perpendicular to axis, centred at wc_dist from origin.
    Structural radius Rw, thickness WT. O-ring groove on rim.
    In neutral position (shaft horizontal, no tilt).
    """
    wc   = av(axis, wc_dist)            # wheel centre
    base = wc - av(axis, WT / 2)        # disc start along axis

    disc  = cyl(Rw, WT, base, axis)
    bore  = cyl(shaft_dia / 2 + 0.1, WT + 2, base - axis, axis)

    # O-ring groove: width = 1.2·dw, depth = 0.6·dw, centred on wheel face
    gw    = dw * 1.2
    gd    = dw * 0.6
    gb    = wc - av(axis, gw / 2)       # groove start
    g_out = cyl(Rw + 0.1, gw, gb, axis)
    g_in  = cyl(Rw - gd,  gw + 2, gb - axis, axis)
    groove = g_out.cut(g_in)

    return disc.cut(bore).cut(groove)


def make_output_shaft(axis):
    """
    Shaft stub from UJ toward wheel centre and slightly beyond.
    Goes in -axis direction from UJ (toward cube centre).
    """
    uj   = av(axis, uj_dist)
    length = B + WT / 2 + 6
    neg_axis = v(-axis.x, -axis.y, -axis.z)
    return cyl(shaft_dia / 2, length, uj, neg_axis)


def make_uj_marker(axis):
    return Part.makeSphere(2.5, av(axis, uj_dist))


def make_cube_wall(axis):
    """Thin wall plate on one face of the cube."""
    n   = v(axis.x, axis.y, axis.z)   # outward normal
    cx  = cube_half if n.x > 0 else (-cube_half - wall_thick if n.x < 0 else -cube_half)
    cy  = cube_half if n.y > 0 else (-cube_half - wall_thick if n.y < 0 else -cube_half)

    if abs(axis.x) > 0.5:
        # Wall perpendicular to X
        x0 = cube_half if axis.x > 0 else -cube_half - wall_thick
        return Part.makeBox(wall_thick, cube_size, cube_h,
                            v(x0, -cube_half, -cube_h / 2))
    else:
        # Wall perpendicular to Y
        y0 = cube_half if axis.y > 0 else -cube_half - wall_thick
        return Part.makeBox(cube_size + 2 * wall_thick, wall_thick, cube_h,
                            v(-cube_half - wall_thick, y0, -cube_h / 2))


# ═══════════════════════════════════════════════════════════════════
# BUILD DOCUMENT
# ═══════════════════════════════════════════════════════════════════

doc_name = "Motcore_v1"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

# Motor shaft (reference cylinder)
add(doc, "MotorShaft_REF",
    cyl(motor_shaft_d / 2, cube_h * 1.2, v(0, 0, -cube_h * 0.6)),
    color=(0.6, 0.6, 0.6))

# Motor disc
add(doc, "MotorDisc",
    make_motor_disc(),
    color=(1.0, 0.60, 0.15))

# 4 output axes
AXES = [
    ("PosX", v( 1,  0, 0)),
    ("NegX", v(-1,  0, 0)),
    ("PosY", v( 0,  1, 0)),
    ("NegY", v( 0, -1, 0)),
]

for name, axis in AXES:
    add(doc, f"OutputWheel_{name}",
        make_output_wheel(axis),
        color=(0.20, 0.80, 0.60))
    add(doc, f"OutputShaft_{name}",
        make_output_shaft(axis),
        color=(0.45, 0.75, 0.65))
    add(doc, f"UJ_{name}",
        make_uj_marker(axis),
        color=(1.0, 0.90, 0.15))
    add(doc, f"Wall_{name}",
        make_cube_wall(axis),
        color=(0.45, 0.55, 0.75),
        transparency=70)

doc.recompute()

if HAS_GUI:
    Gui.ActiveDocument = Gui.getDocument(doc.Name)
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewIsometric()

print("=" * 60)
print("Motcore v1 — O-ring Friction Clutch")
print(f"  Engagement angle:   A  = {A_deg}°")
print(f"  Arm length:         B  = {B} mm")
print(f"  Contact radius:     R  = {R} mm")
print(f"  O-ring wire:        dw = {dw} mm")
print(f"  Wheel thickness:    WT = {WT:.1f} mm  (3.2 × dw)")
print(f"  Wheel radius:       Rw = {Rw:.1f} mm  (R − 0.4·dw)")
print(f"  contactZ:           {contactZ:.2f} mm  → disc height = {disc_h:.1f} mm")
print(f"  Motor disc vr:      {disc_vr:.1f} mm  (R + WT/2)")
print(f"  UJ distance:        {uj_dist:.1f} mm from centre")
print(f"  Cube size:          {cube_size:.0f} × {cube_size:.0f} mm")
print("=" * 60)
