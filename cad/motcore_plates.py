# Motcore v0.2 — Top/Bottom Plates + Friction Wheel Hubs
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

wheel_dia   =  60.0   # mm — friction wheel outer diameter   ← PARAMETRIC
wheel_sep   =  30.0   # mm — distance between wheel centers  ← PARAMETRIC
wheel_thick =  4.0   # mm — friction wheel height

# Rubber ring (arandela de goma) — sits on the inward face of each wheel
rubber_thick =   3.0  # mm — ring thickness
rubber_id    =  18.0  # mm — inner diameter (clears hub body)
rubber_od    =  54.0  # mm — outer diameter (contact zone for clutch wheel)

# Standard flanged shaft hub (generic AliExpress/Amazon 8mm flanged hub)
# ← Update these when the physical hub arrives
hub_body_dia    = 16.0   # mm — hub cylindrical body outer diameter
hub_body_h      = 10.0   # mm — hub cylindrical body height
hub_flange_dia  = 32.0   # mm — flange outer diameter
hub_flange_h    =  3.0   # mm — flange thickness
hub_bolt_circle = 22.0   # mm — bolt circle diameter  ← measure when hub arrives
hub_bolt_count  =  4     # number of bolts on flange
hub_bolt_dia    =  3.3   # mm — M3 clearance hole in wheel (3.0 mm in hub)

tol         =   0.2   # mm — print tolerance (added to holes/seats)

# ═══════════════════════════════════════════════════════════════════
# DERIVED VALUES  (auto-calculated — do not edit)
# ═══════════════════════════════════════════════════════════════════

hs          = cube_size / 2
z_center    = cube_size / 2
z_top_plate = cube_size - plate_thick

z_upper_wheel = z_center + wheel_sep / 2   # center of upper wheel
z_lower_wheel = z_center - wheel_sep / 2   # center of lower wheel

# Cone angle of the clutch wheel:
#   UJ is at the outer wall (x = hs from shaft axis)
#   Contact point is at mid-radius of rubber ring
#   Vertical displacement = from output axis (z_center) to inward wheel face
r_contact = (rubber_od / 2 + rubber_id / 2) / 2       # mid-radius of rubber ring
L         = hs - r_contact                              # horiz. distance UJ → contact
dz        = wheel_sep / 2 - wheel_thick / 2             # vert. displacement to face
cone_angle = math.degrees(math.atan2(dz, L))            # degrees

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

    # M3 bolt holes on hub bolt circle
    bolt_holes = []
    for i in range(hub_bolt_count):
        a = math.radians(i * 360 / hub_bolt_count)
        hole = Part.makeCylinder(
            hub_bolt_dia / 2, wheel_thick + 2,
            App.Vector(hub_bolt_circle / 2 * math.cos(a),
                       hub_bolt_circle / 2 * math.sin(a), -1)
        )
        bolt_holes.append(hole)

    result = disc.cut(hub_hole)
    for hole in bolt_holes:
        result = result.cut(hole)
    return result


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
    ("r_contact",      round(r_contact, 1),        "mm",   "Contact radius on rubber ring (auto)"),
    ("L",              round(L, 1),                "mm",   "UJ to contact point distance (auto)"),
    ("dz",             round(dz, 1),               "mm",   "Vertical displacement to face (auto)"),
    ("cone_angle",     round(cone_angle, 1),       "deg",  "Clutch wheel cone angle (auto)"),
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
print("=" * 60)
