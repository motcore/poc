# Motcore v0.2 — Top/Bottom Plates + Friction Wheels + Output Axis (Cone Wheel)
# FreeCAD Python Macro
#
# Run from FreeCAD: Macro → Macros → motcore_plates.py → Execute
# To change wheel size or separation: edit the PARAMETERS section and re-run.

import FreeCAD as App
import Part
import math

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS  ← edit these values, then re-run the macro
# ═══════════════════════════════════════════════════════════════════

cube_size   = 100.0   # mm — cube side length
shaft_dia   =   8.0   # mm — input shaft (steel rod)
bearing_od  =  22.0   # mm — 608 bearing outer diameter
bearing_id  =   8.0   # mm — 608 bearing inner diameter
bearing_w   =   7.0   # mm — 608 bearing width
plate_thick =   8.0   # mm — top/bottom plate thickness

wheel_dia   =  40.0   # mm — friction wheel outer diameter   ← PARAMETRIC
wheel_sep   =  30.0   # mm — distance between wheel centers  ← PARAMETRIC
wheel_thick =  4.0   # mm — friction wheel height

# Rubber ring (arandela de goma) — sits on the inward face of each wheel
rubber_thick =   2.0  # mm — ring thickness
rubber_id    =  18.0  # mm — inner diameter (clears hub body)

# Standard flanged shaft hub (generic AliExpress/Amazon 8mm flanged hub)
# ← Update these when the physical hub arrives
hub_body_dia    = 16.0   # mm — hub cylindrical body outer diameter
hub_body_h      = 10.0   # mm — hub cylindrical body height
hub_flange_dia  = 32.0   # mm — flange outer diameter
hub_flange_h    =  3.0   # mm — flange thickness
hub_bolt_circle = 24.0   # mm — bolt circle diameter
hub_bolt_count  =  4     # number of bolts on flange
hub_bolt_dia    =  3.3   # mm — M3 clearance hole (shaft)
m3_hex_af       =  5.5   # mm — M3 hex head across flats
m3_head_h       =  2.0   # mm — M3 hex head height

tol         =   0.2   # mm — print tolerance (added to holes/seats)

# Conical clutch wheel (rueda cónica) — D-bore + M3 set screw on the output axis
m3_tap_dia  =  2.5   # mm — M3 tap drill diameter (for set-screw / prisionero)

# ═══════════════════════════════════════════════════════════════════
# DERIVED VALUES  (auto-calculated — do not edit)
# ═══════════════════════════════════════════════════════════════════

hs          = cube_size / 2
z_center    = cube_size / 2
z_top_plate = cube_size - plate_thick

rubber_od = wheel_dia   # always covers full wheel face

z_upper_wheel = z_center + wheel_sep / 2   # center of upper wheel
z_lower_wheel = z_center - wheel_sep / 2   # center of lower wheel

# Conical clutch wheel geometry
# Orientation: small face (tip, ⌀16 mm) points INWARD toward the friction wheels.
#              large face (base, ⌀18 mm) faces outward toward the UJ pivot.
# Contact:     the large-face RIM (r = cone_r_large = 9 mm) is what presses against
#              the rubber ring inner edge (also r = rubber_id/2 = 9 mm) when the shaft tilts.
# z_clearance: in neutral the large-face rim's top (z_center + cone_r_large) is 2 mm below
#              the rubber ring face — this is the gap the shaft tilt must close.
cone_r_large = rubber_id / 2                          # = 9 mm  (= rubber ring inner radius)
cone_r_small = hub_body_dia / 2                        # = 8 mm  (matches shaft diameter)
uj_y         = -(hs - plate_thick)                    # world Y of UJ pivot
cone_tip_y   = -(rubber_id / 2)                        # world Y of the SMALL FACE (tip) — inward end
                                                       # value = 9 mm chosen so rim aligns with rubber_id
arm_length   = abs(cone_tip_y - uj_y)                 # UJ → tip distance along neutral shaft axis (= 33 mm)
z_clearance  = wheel_sep / 2 - wheel_thick / 2 - rubber_thick - cone_r_large  # = 2 mm gap to rubber ring face
cone_angle   = math.degrees(math.asin(z_clearance / arm_length))   # ≈ 3.5° shaft tilt to engage
cone_h       = (cone_r_large - cone_r_small) / math.tan(math.radians(cone_angle))  # frustum height ≈ 16 mm

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
    base = Part.makeBox(
        cube_size, cube_size, plate_thick,
        App.Vector(-hs, -hs, z_base)
    )
    pocket_z = z_base if bearing_from_top else z_base + plate_thick - bearing_w
    bearing_pocket = Part.makeCylinder(
        (bearing_od + tol) / 2, bearing_w + 0.01,
        App.Vector(0, 0, pocket_z)
    )
    shaft_hole = Part.makeCylinder(
        (shaft_dia + tol) / 2, plate_thick + 2,
        App.Vector(0, 0, z_base - 1)
    )
    return base.cut(bearing_pocket).cut(shaft_hole)


def make_hex_pocket(af, height, cx, cy, z_base, angle_offset=0):
    """
    Hexagonal prism to subtract — creates a hex bolt head recess.
    af: across-flats dimension (+ tolerance added here).
    angle_offset: rotation in degrees. Use (bolt_angle_deg - 30) so that
    a flat face points toward the wheel center, keeping clear of the inner hole.
    """
    R = (af + tol) / math.sqrt(3)   # circumradius from across-flats
    pts = [App.Vector(cx + R * math.cos(math.radians(angle_offset + i * 60)),
                      cy + R * math.sin(math.radians(angle_offset + i * 60)),
                      z_base) for i in range(6)]
    pts.append(pts[0])
    face = Part.Face(Part.makePolygon(pts))
    return face.extrude(App.Vector(0, 0, height))


def make_friction_wheel():
    """
    Flat disc that mounts on the flanged shaft hub.
    Inward face (z=0) is where the rubber ring sits.
    Single through-hole for the hub flange (no step).
    """
    disc = Part.makeCylinder(wheel_dia / 2, wheel_thick)

    # Through-hole for the hub body (flange stays on the outward face)
    hub_hole = Part.makeCylinder(
        (hub_body_dia + tol) / 2, wheel_thick + 2,
        App.Vector(0, 0, -1)
    )

    # Hex head pockets (inward face, z=0) + shaft through-holes
    for i in range(hub_bolt_count):
        a_deg = i * 360 / hub_bolt_count
        a     = math.radians(a_deg)
        bx    = hub_bolt_circle / 2 * math.cos(a)
        by    = hub_bolt_circle / 2 * math.sin(a)
        # Hex pocket: rotate so a flat face points toward wheel center
        # (angle_offset = a_deg - 30 puts the inward flat perpendicular to radius)
        disc = disc.cut(make_hex_pocket(m3_hex_af, m3_head_h, bx, by, 0,
                                        angle_offset=a_deg - 30))
        # Clearance hole for bolt shaft through full wheel
        disc = disc.cut(Part.makeCylinder(
            hub_bolt_dia / 2, wheel_thick + 2,
            App.Vector(bx, by, -1)
        ))

    return disc.cut(hub_hole)


def make_rubber_ring_ref():
    """
    Visual reference of the rubber ring (arandela de goma — not printed).
    Built protruding from z=0 toward negative Z (inward face of wheel).
    """
    outer = Part.makeCylinder(
        rubber_od / 2, rubber_thick,
        App.Vector(0, 0, -rubber_thick)
    )
    inner = Part.makeCylinder(
        rubber_id / 2, rubber_thick + 2,
        App.Vector(0, 0, -rubber_thick - 1)
    )
    return outer.cut(inner)


def make_hub_ref():
    """
    Visual reference of the flanged shaft hub (aluminium — not printed).
    Flange at z=0, body extending upward.
    """
    flange = Part.makeCylinder(hub_flange_dia / 2, hub_flange_h)
    body   = Part.makeCylinder(
        hub_body_dia / 2, hub_body_h,
        App.Vector(0, 0, hub_flange_h)
    )
    shaft_hole = Part.makeCylinder(
        (shaft_dia + tol) / 2, hub_flange_h + hub_body_h + 2,
        App.Vector(0, 0, -1)
    )
    bolt_holes = []
    for i in range(hub_bolt_count):
        a = math.radians(i * 360 / hub_bolt_count)
        hole = Part.makeCylinder(
            1.5, hub_flange_h + 2,
            App.Vector(hub_bolt_circle / 2 * math.cos(a),
                       hub_bolt_circle / 2 * math.sin(a), -1)
        )
        bolt_holes.append(hole)
    result = flange.fuse(body).cut(shaft_hole)
    for hole in bolt_holes:
        result = result.cut(hole)
    return result


def make_d_shaft_seg(length):
    """
    D-shaft segment extruded along local +Z for `length` mm.
    Cross-section: circle of shaft_dia/2 with a 1.5 mm flat on the +Y side.
    The flat orientation matches the D-bore in make_cone_wheel() so the shaft
    slides cleanly into the cone wheel when both are placed with rot_out_y.
    """
    r        = shaft_dia / 2                        # = 4.0 mm
    flat_dep = 1.5                                  # same depth as the bore flat
    y_flat   = r - flat_dep                         # = 2.5 mm from axis
    x_flat   = math.sqrt(r**2 - y_flat**2)         # half-chord = 3.12 mm

    p_right  = App.Vector( x_flat, y_flat, 0)
    p_bottom = App.Vector(0,      -r,       0)
    p_left   = App.Vector(-x_flat, y_flat, 0)
    arc_edge  = Part.Arc(p_right, p_bottom, p_left).toShape()
    line_edge = Part.LineSegment(p_left, p_right).toShape()
    wire      = Part.Wire([arc_edge, line_edge])
    face      = Part.Face(wire)
    return face.extrude(App.Vector(0, 0, length))


def make_cone_wheel():
    """
    Conical clutch wheel — D-bore + M3 set screw (prisionero).
    Mounts directly on the 8 mm output shaft; no flanged hub needed.

    Local z=0      : small face / tip (⌀16 mm) — inward, points toward the friction wheels.
                     Placed at y = cone_tip_y (world Y).  When the shaft tilts ±cone_angle
                     around the UJ pivot the tip face swings ±z_clearance to press against
                     the rubber ring flat face (horizontal contact).
    Local z=cone_h : large face (⌀18 mm) — outward, toward the UJ pivot.
    Centre hole    : D-bore for 8 mm shaft (round hole + flat on one side).
    Set screw      : M3 tap hole, radial, 2 mm from the large face (z = cone_h − 2).
    """
    cone = Part.makeCone(cone_r_small, cone_r_large, cone_h)

    # D-bore shaft hole — built as a 2D profile (arc + flat line) extruded along Z.
    # A cylinder-minus-box approach cuts the cone wall on the sides (omega artefact);
    # an extruded profile avoids that entirely.
    r_bore   = (shaft_dia + tol) / 2          # = 4.1 mm
    d_flat_h = 1.5                             # mm removed from one side
    y_flat   = r_bore - d_flat_h              # flat chord position from axis = 2.6 mm
    x_flat   = math.sqrt(r_bore**2 - y_flat**2)  # half-width of chord = 3.17 mm

    # Arc through bottom (p_right → p_bottom → p_left) then straight line back
    p_right  = App.Vector( x_flat, y_flat, 0)
    p_bottom = App.Vector(0,      -r_bore,  0)
    p_left   = App.Vector(-x_flat, y_flat, 0)
    arc_edge  = Part.Arc(p_right, p_bottom, p_left).toShape()
    line_edge = Part.LineSegment(p_left, p_right).toShape()
    wire      = Part.Wire([arc_edge, line_edge])
    face      = Part.Face(wire)
    d_bore    = face.extrude(App.Vector(0, 0, cone_h + 2))
    d_bore.translate(App.Vector(0, 0, -1))
    cone = cone.cut(d_bore)

    # M3 set-screw tap hole — radial, 2 mm from large face (outward end)
    setscrew = Part.makeCylinder(
        m3_tap_dia / 2, cone_r_large * 2 + 2,
        App.Vector(-cone_r_large - 1, 0, cone_h - 2),
        App.Vector(1, 0, 0)
    )
    cone = cone.cut(setscrew)

    return cone


# ═══════════════════════════════════════════════════════════════════
# BUILD DOCUMENT
# ═══════════════════════════════════════════════════════════════════

doc_name = "Motcore_Plates"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

# ── Plates ──────────────────────────────────────────────────────────
add_part(doc, "TopPlate",
         make_plate(z_base=z_top_plate, bearing_from_top=True),
         color=(0.55, 0.70, 1.00))
add_part(doc, "BottomPlate",
         make_plate(z_base=0.0, bearing_from_top=False),
         color=(0.55, 0.70, 1.00))

# ── Input shaft reference ────────────────────────────────────────────
add_part(doc, "InputShaft_REF",
         Part.makeCylinder(shaft_dia / 2, cube_size),
         color=(0.60, 0.60, 0.60))

# ── Friction wheels + hubs + rubber rings ───────────────────────────
# Upper: flange recess & rubber ring on bottom face (z=0 local), hub body up.
# Lower: same wheel & hub flipped 180° around X — rubber ring & flange face up.
wheel  = make_friction_wheel()
hub    = make_hub_ref()
rubber = make_rubber_ring_ref()

rot_norm = App.Rotation()
rot_flip = App.Rotation(App.Vector(1, 0, 0), 180)

z_upper_base = z_upper_wheel - wheel_thick / 2   # inward face of upper wheel
z_lower_top  = z_lower_wheel + wheel_thick / 2   # inward face of lower wheel

# Wheel and rubber ring share the same placement (inward face = local z=0)
for name, z_ref, rot in [("Upper", z_upper_base, rot_norm),
                          ("Lower", z_lower_top,  rot_flip)]:
    pl = App.Placement(App.Vector(0, 0, z_ref), rot)
    add_part(doc, f"FrictionWheel_{name}", wheel,  color=(1.00, 0.60, 0.15), placement=pl)
    add_part(doc, f"RubberRing_{name}_REF", rubber, color=(0.15, 0.15, 0.15), placement=pl)

# Hub flange sits on the OUTWARD face (away from cube center).
# Upper: rot_flip so flange faces up, body passes down through wheel.
# Lower: rot_norm so flange faces down, body passes up through wheel.
# Placement Z = outward face ± hub_flange_h to position the flange flush.
add_part(doc, "FlangedHub_Upper_REF", hub, color=(0.75, 0.75, 0.75),
         placement=App.Placement(
             App.Vector(0, 0, z_upper_base + wheel_thick + hub_flange_h), rot_flip))
add_part(doc, "FlangedHub_Lower_REF", hub, color=(0.75, 0.75, 0.75),
         placement=App.Placement(
             App.Vector(0, 0, z_lower_top - wheel_thick - hub_flange_h), rot_norm))

# ── Output axis — front face (−Y direction) ─────────────────────────
# Rotation: local +Z  →  world −Y  (axis points outward through front face).
# With this rotation:
#   • cone small tip  (local z=0)      is at the INWARD end  (y = cone_tip_y = −9 mm)
#   • cone large face (local z=cone_h) is outward             (y ≈ cone_tip_y − cone_h ≈ −25 mm)
#   • shaft continues large face → UJ pivot                   (y = uj_y = −42 mm)
rot_out_y = App.Rotation(App.Vector(1, 0, 0), 90)

# Cone wheel: small tip at (0, cone_tip_y, z_center)
pl_cone = App.Placement(App.Vector(0, cone_tip_y, z_center), rot_out_y)
add_part(doc, "ConeWheel_Front", make_cone_wheel(),
         color=(1.00, 0.60, 0.15), placement=pl_cone)

# Output axis — two D-shaft segments that meet at the UJ pivot point (y = uj_y).
# Both use rot_out_y so the flat (local +Y side) faces the same direction on both
# segments — they represent one continuous D-shaft passing through the UJ.
#
# ── Pivoting segment: cone tip → UJ pivot  (tilts to engage)
add_part(doc, "OutputShaft_Pivot_REF",
         make_d_shaft_seg(arm_length),
         color=(1.00, 0.80, 0.30),   # amber — moves
         placement=App.Placement(App.Vector(0, cone_tip_y, z_center), rot_out_y))

# ── Fixed segment: UJ pivot → 15 mm outside cube  (stays still, round shaft)
# No D-flat here: this part runs through the side-wall bearing and is a plain cylinder.
fixed_shaft_len = hs + 15 + uj_y   # = 50 + 15 − 42 = 23 mm
add_part(doc, "OutputShaft_Fixed_REF",
         Part.makeCylinder(shaft_dia / 2, fixed_shaft_len),
         color=(0.70, 0.70, 0.70),   # grey — fixed
         placement=App.Placement(App.Vector(0, uj_y, z_center), rot_out_y))

# ── Spreadsheet ──────────────────────────────────────────────────────
sheet = doc.addObject("Spreadsheet::Sheet", "Parameters")
rows = [
    ("Parameter",      "Value",                    "Unit", "Notes"),
    ("cube_size",      cube_size,                  "mm",   "Cube side length"),
    ("shaft_dia",      shaft_dia,                  "mm",   "Input shaft (steel rod)"),
    ("bearing_od",     bearing_od,                 "mm",   "608 bearing OD"),
    ("bearing_w",      bearing_w,                  "mm",   "608 bearing width"),
    ("plate_thick",    plate_thick,                "mm",   "Top/bottom plate thickness"),
    ("wheel_dia",      wheel_dia,                  "mm",   "Friction wheel diameter  <- PARAMETRIC"),
    ("wheel_sep",      wheel_sep,                  "mm",   "Wheel center separation  <- PARAMETRIC"),
    ("wheel_thick",    wheel_thick,                "mm",   "Friction wheel height"),
    ("rubber_thick",   rubber_thick,               "mm",   "Rubber ring thickness"),
    ("rubber_id",      rubber_id,                  "mm",   "Rubber ring inner diameter"),
    ("rubber_od",      rubber_od,                  "mm",   "Rubber ring outer diameter"),
    ("",               "",                         "",     ""),
    ("z_upper_wheel",  z_upper_wheel,              "mm",   "Upper wheel center Z (auto)"),
    ("z_lower_wheel",  z_lower_wheel,              "mm",   "Lower wheel center Z (auto)"),
    ("",               "",                         "",     ""),
    ("m3_tap_dia",     m3_tap_dia,                  "mm",   "M3 set-screw tap drill diameter"),
    ("cone_r_large",   round(cone_r_large, 1),     "mm",   "Cone large face radius (=rubber_id/2)"),
    ("cone_r_small",   round(cone_r_small, 1),     "mm",   "Cone small face radius (=shaft_dia/2)"),
    ("cone_tip_y",     round(cone_tip_y, 1),      "mm",   "Cone tip (small face) world Y — inward end"),
    ("uj_y",           round(uj_y, 1),              "mm",   "UJ world Y position (auto)"),
    ("arm_length",     round(arm_length, 1),        "mm",   "UJ to cone tip distance (auto)"),
    ("z_clearance",    round(z_clearance, 1),       "mm",   "Z gap to rubber ring in neutral (auto)"),
    ("cone_angle",     round(cone_angle, 1),        "deg",  "Shaft tilt to engage rubber ring (auto)"),
    ("cone_h",         round(cone_h, 1),            "mm",   "Cone frustum height (auto)"),
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
print("Motcore Plates — model created successfully")
print(f"  Cube:         {cube_size} × {cube_size} × {cube_size} mm")
print(f"  Shaft:        ⌀{shaft_dia} mm  |  Bearing 608")
print(f"  Wheel dia:    ⌀{wheel_dia} mm  |  sep: {wheel_sep} mm")
print(f"  Rubber ring:  ⌀{rubber_id}–{rubber_od} mm  ×  {rubber_thick} mm thick")
print(f"  Cone angle:   {cone_angle:.1f}°  (clutch wheel cone)")
print(f"  Cone wheel:   tip ⌀{cone_r_small*2:.1f} → base ⌀{cone_r_large*2:.1f} mm  h={cone_h:.1f} mm  tilt={cone_angle:.1f}°")
print(f"  Pivot point:  y={uj_y:.1f} mm  |  tip y={cone_tip_y:.1f} mm  |  arm={arm_length:.1f} mm  gap={z_clearance:.1f} mm")
print("=" * 60)
