# Motcore v0.3 — Bevel Friction Drive (zero differential slip)
# FreeCAD Python Macro
#
# Both motor cone and clutch cone are bevel cones sharing the same apex.
# The shared apex is at the intersection of both rotation axes (cube centre).
# When the output shaft tilts by cone_engage_angle the two cone surfaces make
# contact along a generator → pure rolling, no differential slip.
#
# Geometry (bevel-gear principle):
#   α_motor + α_clutch = 90° − cone_engage_angle
#   For 1:1 ratio → α_motor = α_clutch = α = (90° − θ_engage) / 2
#
# Run from FreeCAD: Macro → Macros → motcore_plates.py → Execute

import FreeCAD as App
import Part
import math

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS  ← edit these, then re-run
# ═══════════════════════════════════════════════════════════════════

cube_size         = 100.0  # mm — cube side length
shaft_dia         =   8.0  # mm — input Z-shaft and output D-shaft diameter
bearing_od        =  22.0  # mm — 608 bearing outer diameter
bearing_id        =   8.0  # mm — 608 bearing inner diameter
bearing_w         =   7.0  # mm — 608 bearing width
plate_thick       =   8.0  # mm — top / bottom plate thickness
tol               =   0.2  # mm — print tolerance (added to holes / seats)

# Bevel friction drive ────────────────────────────────────────────
cone_engage_angle = 5.0    # deg — output-shaft tilt required to engage
#   (determines servo travel; both cone half-angles are derived from this)
motor_cone_h      = 20.0   # mm — motor cone height along Z from apex
clutch_cone_h     = 20.0   # mm — clutch cone height along output shaft from apex

m3_tap_dia        =  2.5   # mm — M3 set-screw tap drill

# ═══════════════════════════════════════════════════════════════════
# DERIVED VALUES  (do not edit)
# ═══════════════════════════════════════════════════════════════════

hs          = cube_size / 2          # = 50 mm
z_center    = cube_size / 2          # = 50 mm
z_top_plate = cube_size - plate_thick  # = 92 mm

# Bevel cone half-angle for both cones (1:1 ratio, 90°−θ between axes at engagement)
alpha     = (90.0 - cone_engage_angle) / 2.0   # = 42.5°
alpha_rad = math.radians(alpha)

motor_cone_r  = motor_cone_h  * math.tan(alpha_rad)   # large-face radius, motor cone  (≈ 18.3 mm)
clutch_cone_r = clutch_cone_h * math.tan(alpha_rad)   # large-face radius, clutch cone (≈ 18.3 mm)

# UJ pivot — same as original design: close to the cube wall
uj_y       = -(hs - plate_thick)   # = −42 mm  (8 mm inside the side wall)

# Clutch cone apex sits at the inner end of the arm, at y = 0 (Z axis).
# When the shaft tilts cone_engage_angle around the UJ, the apex traces an arc
# and arrives at approximately (0, 0, z_apex_motor) on the Z axis — exactly
# where the motor cone apex must sit for the two apices to coincide at engagement.
arm_length   = abs(0.0 - uj_y)                              # = 42 mm  (apex → UJ)
z_apex_motor = z_center + arm_length * math.sin(alpha_rad * 2 * cone_engage_angle / (90 - cone_engage_angle))
# Simpler exact formula: at tilt θ, apex z = z_center + arm * sin(θ)
z_apex_motor = z_center + arm_length * math.sin(math.radians(cone_engage_angle))
# ≈ 50 + 42 * sin(5°) ≈ 53.7 mm

# Output shaft segments
fixed_shaft_len = (hs - abs(uj_y)) + 15.0   # UJ → wall + 15 mm stub  (= 23 mm)

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def add_part(doc, name, shape, color=(0.8, 0.8, 0.8), placement=None):
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if HAS_GUI:
        obj.ViewObject.ShapeColor = color
    if placement:
        obj.Placement = placement
    return obj


def make_plate(z_base, bearing_from_top):
    """100×100×8 mm plate with 608 bearing pocket and shaft through-hole."""
    base = Part.makeBox(cube_size, cube_size, plate_thick,
                        App.Vector(-hs, -hs, z_base))
    pocket_z = z_base if bearing_from_top else z_base + plate_thick - bearing_w
    bearing_pocket = Part.makeCylinder(
        (bearing_od + tol) / 2, bearing_w + 0.01,
        App.Vector(0, 0, pocket_z))
    shaft_hole = Part.makeCylinder(
        (shaft_dia + tol) / 2, plate_thick + 2,
        App.Vector(0, 0, z_base - 1))
    return base.cut(bearing_pocket).cut(shaft_hole)


def make_d_bore(length):
    """
    D-shaped through-hole, extruded along local +Z for `length` mm.
    Round part: ⌀(shaft_dia + tol).  Flat: 1.5 mm removed from +Y side.
    """
    r_bore   = (shaft_dia + tol) / 2
    flat_dep = 1.5
    y_flat   = r_bore - flat_dep
    x_flat   = math.sqrt(r_bore**2 - y_flat**2)
    p_right  = App.Vector( x_flat,  y_flat, 0)
    p_bottom = App.Vector(0,       -r_bore,  0)
    p_left   = App.Vector(-x_flat,  y_flat, 0)
    arc_edge  = Part.Arc(p_right, p_bottom, p_left).toShape()
    line_edge = Part.LineSegment(p_left, p_right).toShape()
    wire = Part.Wire([arc_edge, line_edge])
    face = Part.Face(wire)
    bore = face.extrude(App.Vector(0, 0, length + 2))
    bore.translate(App.Vector(0, 0, -1))
    return bore


def make_d_shaft_seg(length):
    """
    Solid D-shaft segment extruded along local +Z for `length` mm.
    Matches the D-bore profile so the shaft slides cleanly into the clutch cone.
    """
    r        = shaft_dia / 2
    flat_dep = 1.5
    y_flat   = r - flat_dep
    x_flat   = math.sqrt(r**2 - y_flat**2)
    p_right  = App.Vector( x_flat,  y_flat, 0)
    p_bottom = App.Vector(0,       -r,       0)
    p_left   = App.Vector(-x_flat,  y_flat, 0)
    arc_edge  = Part.Arc(p_right, p_bottom, p_left).toShape()
    line_edge = Part.LineSegment(p_left, p_right).toShape()
    wire = Part.Wire([arc_edge, line_edge])
    face = Part.Face(wire)
    return face.extrude(App.Vector(0, 0, length))


def make_motor_cone():
    """
    Motor (input) bevel cone — mounts on the Z input shaft.

    Local frame (placed with no rotation):
      z = 0        : apex  (sharp end) — sits at z_center (cube centre)
      z = motor_cone_h : large face  — points away from cube centre (+Z direction)

    Half-angle α = (90° − cone_engage_angle) / 2 from the Z axis.
    Round bore for Z shaft + M3 radial set screw near large face.
    """
    outer = Part.makeCone(0, motor_cone_r, motor_cone_h)

    # Round bore for Z input shaft
    bore = Part.makeCylinder(
        (shaft_dia + tol) / 2, motor_cone_h + 2,
        App.Vector(0, 0, -1))
    outer = outer.cut(bore)

    # M3 set-screw tap hole — radial, 2 mm from large face
    setscrew = Part.makeCylinder(
        m3_tap_dia / 2, motor_cone_r * 2 + 2,
        App.Vector(-motor_cone_r - 1, 0, motor_cone_h - 2),
        App.Vector(1, 0, 0))
    outer = outer.cut(setscrew)

    return outer


def make_clutch_cone():
    """
    Clutch (output) bevel cone — mounts on the output D-shaft.

    Local frame (placed with rot_out_y so local +Z → world −Y):
      z = 0          : apex  — world position (0, 0, z_center) = UJ pivot = shared apex
      z = clutch_cone_h : large face — world y = −clutch_cone_h (toward cube wall)

    Half-angle α from the output-shaft axis (same as motor cone → 1:1 ratio).
    D-bore for output shaft + M3 radial set screw near large face.
    """
    outer = Part.makeCone(0, clutch_cone_r, clutch_cone_h)

    # D-bore for output shaft (extruded along local Z = world −Y with rot_out_y)
    outer = outer.cut(make_d_bore(clutch_cone_h))

    # M3 set-screw tap hole — radial, 2 mm from large face
    setscrew = Part.makeCylinder(
        m3_tap_dia / 2, clutch_cone_r * 2 + 2,
        App.Vector(-clutch_cone_r - 1, 0, clutch_cone_h - 2),
        App.Vector(1, 0, 0))
    outer = outer.cut(setscrew)

    return outer


# ═══════════════════════════════════════════════════════════════════
# BUILD DOCUMENT
# ═══════════════════════════════════════════════════════════════════

doc_name = "Motcore_Plates"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

# ── Top / bottom plates ─────────────────────────────────────────────
add_part(doc, "TopPlate",
         make_plate(z_base=z_top_plate, bearing_from_top=True),
         color=(0.55, 0.70, 1.00))
add_part(doc, "BottomPlate",
         make_plate(z_base=0.0, bearing_from_top=False),
         color=(0.55, 0.70, 1.00))

# ── Input shaft reference (Z axis) ──────────────────────────────────
add_part(doc, "InputShaft_REF",
         Part.makeCylinder(shaft_dia / 2, cube_size),
         color=(0.55, 0.55, 0.55))

# ── Motor cone (upper) ───────────────────────────────────────────────
# Apex at (0, 0, z_apex_motor) — shifted above z_center so that at engagement
# tilt the two apices coincide on the Z axis.
# Large face at (0, 0, z_apex_motor + motor_cone_h).
add_part(doc, "MotorCone_Upper",
         make_motor_cone(),
         color=(1.00, 0.60, 0.15),
         placement=App.Placement(App.Vector(0, 0, z_apex_motor), App.Rotation()))

# ── Output axis — front face (−Y direction) ─────────────────────────
# rot_out_y: local +Z → world −Y.
# The clutch cone apex is at world (0, 0, z_center); local z=0 maps there.
# The UJ pivot is at (0, uj_y, z_center) = (0, −42, 50) — same as original.
# The servo tilts the shaft around the UJ; the apex swings in an arc and
# arrives at the Z axis (y≈0, z≈z_apex_motor) at cone_engage_angle tilt.
rot_out_y = App.Rotation(App.Vector(1, 0, 0), 90)
apex_pl   = App.Placement(App.Vector(0, 0, z_center), rot_out_y)

# Clutch cone: apex at (0, 0, z_center), large face at y = −clutch_cone_h
add_part(doc, "ClutchCone_Front",
         make_clutch_cone(),
         color=(1.00, 0.75, 0.20),
         placement=apex_pl)

# Pivoting D-shaft: from clutch cone apex (y=0) to UJ pivot (y=uj_y=−42 mm)
# This segment tilts with the servo — amber colour to match original convention.
add_part(doc, "OutputShaft_Pivot_REF",
         make_d_shaft_seg(arm_length),
         color=(1.00, 0.80, 0.30),
         placement=apex_pl)

# Fixed segment: plain cylinder from UJ pivot outward through the side wall
add_part(doc, "OutputShaft_Fixed_REF",
         Part.makeCylinder(shaft_dia / 2, fixed_shaft_len),
         color=(0.70, 0.70, 0.70),
         placement=App.Placement(App.Vector(0, uj_y, z_center), rot_out_y))

# ── Spreadsheet ──────────────────────────────────────────────────────
sheet = doc.addObject("Spreadsheet::Sheet", "Parameters")
rows = [
    ("Parameter",           "Value",                          "Unit",  "Notes"),
    ("cube_size",           cube_size,                        "mm",    "Cube side"),
    ("shaft_dia",           shaft_dia,                        "mm",    "Input Z-shaft / output D-shaft"),
    ("bearing_od",          bearing_od,                       "mm",    "608 bearing OD"),
    ("bearing_w",           bearing_w,                        "mm",    "608 bearing width"),
    ("plate_thick",         plate_thick,                      "mm",    "Top/bottom plate thickness"),
    ("",                    "",                               "",      ""),
    ("cone_engage_angle",   cone_engage_angle,                "deg",   "Shaft tilt to engage  <- PARAMETRIC"),
    ("alpha",               round(alpha, 2),                  "deg",   "Bevel cone half-angle (both cones)"),
    ("motor_cone_h",        motor_cone_h,                     "mm",    "Motor cone height along Z"),
    ("motor_cone_r",        round(motor_cone_r, 2),           "mm",    "Motor cone large-face radius (auto)"),
    ("clutch_cone_h",       clutch_cone_h,                    "mm",    "Clutch cone height along output shaft"),
    ("clutch_cone_r",       round(clutch_cone_r, 2),          "mm",    "Clutch cone large-face radius (auto)"),
    ("",                    "",                               "",      ""),
    ("uj_y",                uj_y,                             "mm",    "UJ pivot world Y (near cube wall)"),
    ("arm_length",          arm_length,                       "mm",    "Apex → UJ distance"),
    ("z_apex_motor",        round(z_apex_motor, 2),           "mm",    "Motor cone apex Z (above z_center)"),
    ("fixed_shaft_len",     fixed_shaft_len,                  "mm",    "Fixed shaft stub length"),
    ("",                    "",                               "",      ""),
    ("ratio",               "1:1",                            "",      "sin(alpha)/sin(alpha) = 1"),
    ("diff_slip",           "zero",                           "",      "Shared apex → pure rolling contact"),
]
cols = ["A", "B", "C", "D"]
for i, row in enumerate(rows, start=1):
    for j, val in enumerate(row):
        sheet.set(f"{cols[j]}{i}", str(val))

doc.recompute()

if HAS_GUI:
    Gui.ActiveDocument = Gui.getDocument(doc.Name)
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewIsometric()

print("=" * 60)
print("Motcore v0.3 — Bevel friction drive")
print(f"  Cube:              {cube_size} × {cube_size} × {cube_size} mm")
print(f"  Engagement tilt:   {cone_engage_angle}°")
print(f"  Cone half-angle:   α = {alpha:.2f}°  (both cones)")
print(f"  Motor cone:        apex ⌀0 → base ⌀{motor_cone_r*2:.1f} mm  h = {motor_cone_h} mm")
print(f"  Clutch cone:       apex ⌀0 → base ⌀{clutch_cone_r*2:.1f} mm  h = {clutch_cone_h} mm")
print(f"  UJ pivot:          y = {uj_y:.1f} mm  (arm = {arm_length:.1f} mm)")
print(f"  Motor cone apex:   z = {z_apex_motor:.2f} mm  (apex at engagement on Z axis)")
print(f"  Ratio:             1:1  |  Differential slip: zero")
print("=" * 60)
