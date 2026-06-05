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

neck_dia      = 2.5   # mm   — neck diameter (torsion strength + stiffness match)
neck_len      = 13.0  # mm   — neck length (matched to combined blade stiffness)
head_gap      = 2.0   # mm   — clearance between wheel outer face and bracket head
head_depth    = 8.0   # mm   — bracket head bearing length
lever_len     = 15.0  # mm   — fixed shaft stub beyond wall (servo lever arm side)

# ═══════════════════════════════════════════════════════════════════
# DERIVED  (do not edit)
# ═══════════════════════════════════════════════════════════════════

A   = math.radians(A_deg)
cA  = math.cos(A)
sA  = math.sin(A)

WT  = 3.2 * dw        # wheel / disc thickness (groove walls + groove width)

# ── Motor disc geometry (driven by R, the motor disc radius) ──────────────
contactY = B * cA - R * sA       # 1:1 contact Y from UJ (used to fix motorY)
motorY   = contactY + R           # motor shaft distance from UJ
uj_dist  = motorY
cube_half = uj_dist + wall_thick
cube_size = 2 * cube_half
disc_vr   = R + WT / 2            # motor disc visual radius

# ── Output wheel centre ───────────────────────────────────────────────────
wc_dist = uj_dist - B             # wheel centre distance from cube centre

# ── Output wheel radius — square no-overlap condition ─────────────────────
# Top view: each wheel is a rectangle (width WT, height 2·Rw).
# No-overlap: outer rim Rw ≤ inner face of adjacent wheel at wc_dist − WT/2.
# → R_out = wc_dist − WT/2  (exact square; subtract gap for clearance)
wheel_gap = 0.5                   # mm clearance at corners between adjacent wheels
R_out = wc_dist - WT / 2 - wheel_gap   # output wheel contact radius (< R, breaks 1:1)
Rw    = R_out - 0.4 * dw          # structural wheel radius

# ── Contact geometry uses R_out ───────────────────────────────────────────
# Motor disc faces are where the O-ring actually contacts them.
contactZ = B * sA + R_out * cA    # engagement height (was B·sinA + R·cosA in 1:1)
disc_bot  = -contactZ
disc_h    = 2 * contactZ

# ── Bracket head and shaft neck geometry ─────────────────────────────────
head_y0    = wc_dist + WT / 2 + head_gap   # head inner face (just past wheel)
head_y1    = head_y0 + head_depth          # head outer face
# Single compliant neck centred in the free span between head and wall
neck_start = head_y1 + (cube_half - head_y1 - neck_len) / 2
neck_end   = neck_start + neck_len


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
    bore_r = motor_shaft_d / 2 + 0.1
    # Upper disc: contact face at z = +contactZ (bottom of disc), body extends upward
    top_disc = cyl(disc_vr, WT, v(0, 0, contactZ))
    top_bore = cyl(bore_r, WT + 2, v(0, 0, contactZ - 1))
    # Lower disc: contact face at z = -contactZ (top of disc), body extends downward
    bot_disc = cyl(disc_vr, WT, v(0, 0, -contactZ - WT))
    bot_bore = cyl(bore_r, WT + 2, v(0, 0, -contactZ - WT - 1))
    return top_disc.cut(top_bore).fuse(bot_disc.cut(bot_bore))


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


def make_oring(axis):
    Ror = R_out - 0.5 * dw   # O-ring centre radius (0.1·dw inside rim, protrudes 0.4·dw)
    wc  = av(axis, wc_dist)
    return Part.makeTorus(Ror, dw / 2, wc, axis)


def make_output_shaft(axis):
    """
    Output shaft — uniform shaft_dia with a single compliant neck.

    The neck is centred in the free span between the bracket head and the wall.
    Its stiffness matches the two flexure blades (equal tilt contribution A/2 each).
    The shaft rotates in Y (output torque); the neck flexes in Z (tilt A/2).
    """
    neg = v(-axis.x, -axis.y, -axis.z)

    # Inner body: from just past wheel bore inward end to neck_start
    inner_len = neck_start - (wc_dist - WT / 2 - 2.0)
    inner_start = av(axis, neck_start)
    inner = cyl(shaft_dia / 2, inner_len, inner_start, neg)

    # Compliant neck (neck_dia, neck_len) — centred in free span
    neck = cyl(neck_dia / 2, neck_len, av(axis, neck_start), axis)

    # Outer body: from neck_end through wall to lever arm tip
    outer_len = cube_half + wall_thick + lever_len - neck_end
    outer = cyl(shaft_dia / 2, outer_len, av(axis, neck_end), axis)

    return inner.fuse(neck).fuse(outer)


def make_uj_marker(axis):
    return Part.makeSphere(2.5, av(axis, uj_dist))


def make_cube_wall(axis):
    """Wall with integrated compliant flexure bracket (one-piece print).
    Includes a through-bore for the output shaft lever arm.
    """
    shaft_hole_r = shaft_dia / 2 + 0.4   # clearance fit for rotating shaft

    if abs(axis.x) > 0.5:
        x0 = cube_half if axis.x > 0 else -cube_half - wall_thick
        wall = Part.makeBox(wall_thick, cube_size, cube_h,
                            v(x0, -cube_half, -cube_h / 2))
        # Shaft bore along X through the wall at (y=0, z=0)
        hole_base = v(x0 - 1, 0, 0)
        wall = wall.cut(cyl(shaft_hole_r, wall_thick + 2, hole_base, v(1, 0, 0)))
    else:
        y0 = cube_half if axis.y > 0 else -cube_half - wall_thick
        wall = Part.makeBox(cube_size, wall_thick, cube_h,
                            v(-cube_half, y0, -cube_h / 2))
        # Shaft bore along Y through the wall at (x=0, z=0)
        hole_base = v(0, y0 - 1, 0)
        wall = wall.cut(cyl(shaft_hole_r, wall_thick + 2, hole_base, v(0, 1, 0)))

    return wall.fuse(make_shaft_bracket(axis))


def make_corner_bevel(sx, sy):
    """45° triangular prism filling the gap at corner (sx, sy)."""
    x0 = sx * cube_half
    y0 = sy * cube_half
    z0 = -cube_h / 2
    p1 = v(x0,                  y0,                  z0)
    p2 = v(x0 + sx * wall_thick, y0,                  z0)
    p3 = v(x0,                  y0 + sy * wall_thick, z0)
    wire = Part.Wire([Part.makeLine(p1, p2),
                      Part.makeLine(p2, p3),
                      Part.makeLine(p3, p1)])
    return Part.Face(wire).extrude(v(0, 0, cube_h))


def make_shaft_bracket(axis):
    """
    Compliant flexure mount - output shaft support.

    Head  : block near the wheel with a bearing bore for the shaft.
    Blades: two thin flat strips running from head to wall, one on each
            pa side of the shaft (+-pa), lying at z = 0 (shaft height).
              - Thin in Z  -> bend easily -> allow +-A tilt in vertical plane
              - Wide in pa -> stiff in-plane -> constrain horizontal rotation
    One-piece print. No pin, no assembly.
    Blade spring stiffness provides the return-to-neutral force.

    Canonical frame: shaft = +Y, pa (pin axis) = +X.
    """
    blade_t     = 1.5              # blade thickness in Z — matched to neck stiffness
    blade_w     = 6.0              # blade width in pa (horizontal stiffness)
    blade_gap   = shaft_dia + 1.0  # gap between blades in pa (shaft clears through)
    head_z_half = shaft_dia / 2 + 3.5  # head half-height in Z

    bore_r = shaft_dia / 2 + 0.25  # bearing bore radius

    # head_y0, head_y1 from global DERIVED (shared with make_output_shaft)
    blade_len = cube_half - head_y1

    bw_half = blade_gap / 2 + blade_w  # half total bracket width in pa

    # Head block with bearing bore
    head = Part.makeBox(2 * bw_half, head_depth, 2 * head_z_half,
                        v(-bw_half, head_y0, -head_z_half))
    head = head.cut(cyl(bore_r, head_depth + 2,
                        v(0, head_y0 - 1, 0),  v(0, 1, 0)))

    # Right blade (+pa side): lies flat at z=0, thin in Z, wide in pa
    blade_r = Part.makeBox(blade_w, blade_len, blade_t,
                           v( blade_gap / 2,              head_y1, -blade_t / 2))
    # Left blade (-pa side)
    blade_l = Part.makeBox(blade_w, blade_len, blade_t,
                           v(-(blade_gap / 2 + blade_w),  head_y1, -blade_t / 2))

    shape = head.fuse(blade_r).fuse(blade_l)

    # Rotate canonical (+Y) to actual axis
    if   axis.x >  0.5: angle = -90.0
    elif axis.x < -0.5: angle =  90.0
    elif axis.y < -0.5: angle = 180.0
    else:               angle =   0.0
    if angle:
        shape = shape.rotate(v(0, 0, 0), v(0, 0, 1), angle)
    return shape

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
    add(doc, f"Oring_{name}",
        make_oring(axis),
        color=(0.90, 0.20, 0.50))
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

# 45° corner bevels — one per corner, same transparency as walls
for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
    label = f"Corner_{'P' if sx > 0 else 'N'}X_{'P' if sy > 0 else 'N'}Y"
    add(doc, label, make_corner_bevel(sx, sy),
        color=(0.45, 0.55, 0.75), transparency=70)

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
print(f"  Output wheel R_out: {R_out:.1f} mm  (wc_dist − WT/2 − {wheel_gap} mm gap)")
print(f"  Wheel radius:       Rw = {Rw:.1f} mm  (R_out − 0.4·dw)")
print(f"  Transmission ratio: {R_out/R:.3f}  (R_out / R,  ω_out ≈ {R/R_out:.3f} · ω_motor)")
print(f"  contactZ:           {contactZ:.2f} mm  → disc height = {disc_h:.1f} mm")
print(f"  Motor disc vr:      {disc_vr:.1f} mm  (R + WT/2)")
print(f"  UJ distance:        {uj_dist:.1f} mm from centre")
print(f"  Cube size:          {cube_size:.0f} × {cube_size:.0f} mm")
print("=" * 60)
