# Motcore — FDM/PLA tolerance calibration coupon
# FreeCAD Python Macro
#
# Prints a small coupon with stepped hole sizes so you can find your printer's
# real fit offsets, then feed 3 numbers back into motcore_compliant_lever.py.
#
# Run from FreeCAD: Macro → Macros → calibration.py → Execute
# It builds the coupon and exports cad/stl/motcore_calibration.stl.

import FreeCAD as App
import Part
import os

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ── What we calibrate (target sizes used in the design) ──────────────────────
# For each row, print the coupon, try the REAL part in each hole, and note the
# modelled size of the hole that fits best.
# v2 ranges — shifted up after v1 showed this printer runs small holes ~0.5 mm
# undersize (v1: bearing best at 10.3; shaft/pilot maxed out still too tight).
BEARING_STEPS = [10.2, 10.3, 10.4, 10.5, 10.6]       # MR105ZZ OD: press → slip (blind)
SHAFT_STEPS   = [5.6, 5.8, 6.0, 6.2, 6.4]            # Ø5 shaft free slip (through)
PILOT_STEPS   = [2.8, 2.9, 3.0, 3.1, 3.2]            # M3 self-tap pilot (blind)

# ── Coupon geometry ──────────────────────────────────────────────────────────
plate_t   = 6.0     # mm — coupon thickness
pitch     = 14.0    # mm — hole spacing along a row
margin    = 10.0    # mm — border
row_gap   = 16.0    # mm — spacing between rows
blind_d   = 5.0     # mm — blind hole depth (bearing / pilot)


def v(x, y, z):
    return App.Vector(x, y, z)


def cyl(r, h, base, d=v(0, 0, 1)):
    return Part.makeCylinder(r, h, base, d)


def build_coupon():
    rows = [("BEARING", BEARING_STEPS, blind_d),
            ("SHAFT",   SHAFT_STEPS,   plate_t + 2),   # through
            ("PILOT",   PILOT_STEPS,   blind_d)]

    n_max = max(len(s) for _, s in [(r[0], r[1]) for r in rows])
    width = 2 * margin + (n_max - 1) * pitch
    depth = 2 * margin + (len(rows) - 1) * row_gap

    plate = Part.makeBox(width, depth, plate_t, v(0, 0, 0))

    for ri, (name, steps, hole_depth) in enumerate(rows):
        y = margin + ri * row_gap
        for ci, dia in enumerate(steps):
            x = margin + ci * pitch
            # hole from the TOP face downward
            z0 = plate_t + 0.5
            plate = plate.cut(cyl(dia / 2, hole_depth + 0.5, v(x, y, z0), v(0, 0, -1)))
        # reference notch next to hole #1 (the smallest) → marks row start
        nx = margin - pitch / 2
        plate = plate.cut(Part.makeBox(3, 3, plate_t + 2, v(nx - 1.5, y - 1.5, -1)))

    return plate


doc_name = "Motcore_Calibration"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

coupon = build_coupon()
obj = doc.addObject("Part::Feature", "CalibrationCoupon")
obj.Shape = coupon
if HAS_GUI:
    obj.ViewObject.ShapeColor = (0.4, 0.7, 0.9)
doc.recompute()

if HAS_GUI:
    Gui.ActiveDocument = Gui.getDocument(doc.Name)
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewAxonometric()

# ── Export STL ───────────────────────────────────────────────────────────────
try:
    _here = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _here = App.getUserMacroDir(True)
OUT_DIR = os.path.join(_here, "stl")
if not os.path.isdir(OUT_DIR):
    os.makedirs(OUT_DIR)
_path = os.path.join(OUT_DIR, "motcore_calibration.stl")
try:
    import MeshPart
    MeshPart.meshFromShape(Shape=coupon, LinearDeflection=0.1,
                           AngularDeflection=0.5).write(_path)
except Exception:
    coupon.exportStl(_path)

# ── Legend ───────────────────────────────────────────────────────────────────
print("=" * 64)
print("Motcore — FDM/PLA calibration coupon")
print(f"  STL: {_path}")
print("  The notch marks hole #1 of each row (smallest). Sizes per row listed below.")
print("-" * 64)
print("  Row 1 (near notch side, blind)  BEARING — push a real MR105ZZ in:")
print(f"    {BEARING_STEPS}  → firm press, stays put, no crack = your value")
print("  Row 2 (through)                 SHAFT — push the Ø5 rod through:")
print(f"    {SHAFT_STEPS}  → slides smooth, zero wobble = your value")
print("  Row 3 (blind)                   PILOT — drive an M3 screw in:")
print(f"    {PILOT_STEPS}  → self-taps firm, no crack = your value")
print("-" * 64)
print("  Tell me the 3 winning sizes and I set the fits in the main macro.")
print("=" * 64)
