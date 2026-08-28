# Motcore v5 — Vertical Cone Clutch
# FreeCAD Python Macro
#
# Full cube: two motor cones (one flipped) x 4 output axes each = 8 axes.
# Reference/first-pass model. See docs/clutch-geometry.md
# and CLAUDE.md ("Clutch mechanism — v5") for the full derivation this macro
# implements; cad/clutch_geometry_v5.html is the interactive 2D visualiser
# this geometry is built from.
#
# Architecture (Y-Z cross-section, see docs/clutch-geometry.md §3):
#   - Motor cone: apex at global origin, axis = Z (vertical), fixed, flares
#     downward (material at z < 0). Rotates continuously with the motor shaft.
#   - Output cone: axis = Y (horizontal), apex at (0, 0, z_carriage). Rigid to
#     the carriage, which translates vertically on 2 guide rods — z_carriage
#     is the ONE degree of freedom of the whole mechanism.
#   - Pinion: rigid to the output cone/carriage, same axis, further along +Y.
#   - Corona: fixed to the output shaft, axis fixed at z = g + e (so the
#     pinion sits concentric with it at the "free" stop).
#   - Contact = 0 reference: the carriage height at which the output cone's
#     own axis crosses the motor's Z axis exactly at the motor's apex — i.e.
#     the two nominal apexes coincide and the shared generatrix is real.
#
# Gear stage (pinion + corona): real involute teeth via the freecad.gears
# addon (workbench "Gear", package freecad.gears — install via Addon Manager
# if GEARS_AVAILABLE prints False below). Falls back to plain tip-circle
# reference cylinders/tubes if the addon isn't installed.
#
# Run from FreeCAD: Macro → Macros → motcore_v5_vertical_clutch.py → Execute

import FreeCAD as App
import Part
import math

try:
    from freecad.gears.involutegear import InvoluteGear
    from freecad.gears.internalinvolutegear import InternalInvoluteGear
    from freecad.gears.basegear import ViewProviderGear
    GEARS_AVAILABLE = True
except ImportError:
    GEARS_AVAILABLE = False

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS  ← edit here, then re-run  (mirrors clutch_geometry_v5.html)
# ═══════════════════════════════════════════════════════════════════

alpha_deg     = 55.0   # deg — motor cone half-angle from the vertical (Z) axis
m_mod         = 1.8    # mm  — gear module. The root-clearance constraint
                        #       (g+delta <= 0.25*m) sets a FLOOR on m regardless
                        #       of tooth count: m >= 4*(g+delta) = 1.8. Going
                        #       any higher than that floor (e.g. the earlier
                        #       m=2.0) only bloats the gear stage for no
                        #       benefit — at m=2.0/Zp=14/Zc=28 the corona came
                        #       out bigger than the motor cone itself, which
                        #       defeats the point of the vertical-cone design
                        #       (maximise cone size in the cube). At the floor
                        #       (m=1.8) g+delta=0.45 == c=0.45 EXACTLY — zero
                        #       margin; nudge m up slightly (e.g. 1.85) if real
                        #       tolerances need headroom here.
Zp            = 8      # —   — pinion teeth (rigid to the output cone/carriage).
                        #       At the module floor, fewer teeth is the other
                        #       lever to shrink the gear stage further (radius
                        #       scales with m*Z). Zc-Zp=8 is the interference
                        #       floor (constraint 5) — zero margin there too.
Zc            = 16     # —   — corona teeth (fixed to the output shaft)
L_line        = 18.0   # mm  — contact line length along the shared generatrix
s0            = 10.0   # mm  — apex → start of the contact line
d_oring       = 2.5    # mm  — O-ring wire diameter
n_rings       = 5      # —   — rings on the motor cone (output cone carries n-1)
g_margin      = 0.20   # mm  — margin: full mesh → rubber contact
delta_preload = 0.25   # mm  — preload travel past contact

CARRIAGE_STOP = "contact"   # "free" | "mesh" | "contact" | "preload" — which of
                             # the four stops to render (DERIVED table below)
CARRIAGE_DIR  = -1          # -1 = carriage moved DOWN, toward the lower motor
                             # cone; +1 = UP, toward the upper one. Ignored at
                             # the "free" stop, which is the mid position.

# ── Cube layout ──────────────────────────────────────────────────────────────
# FOUR output axes, one per cube wall. Each axis has ONE output cone, sitting
# in the waist between TWO motor cones that share the central shaft: the lower
# one has its apex up and flares downward, the upper one is that same cone
# turned upside down, apexes facing each other (an hourglass).
#
# The two cones are what make the axis BIDIRECTIONAL. Both turn with the motor,
# always the same way, but their flanks face opposite sides of the output axis.
# Drive the carriage down and the output cone's lower flank meets the lower
# motor cone; drive it up and its upper flank meets the upper one — and the
# output shaft turns the other way. This is the conical descendant of v3/v4's
# "one disc, two usable faces": one motor, one axis, both directions, without
# ever reversing the motor.
#
# The apex separation is NOT a free parameter — see DERIVED (motor_cone_sep).
SHOW_WALLS = True      # build the 4 cube walls (reference, for the fit check)

wall_thick   = 4.0     # mm — cube wall thickness
wall_gap     = 6.0     # mm — clearance from the outermost rotating part
                        #      (corona rim) to the inside face of the wall
wall_shaft_clr = 2.0   # mm — radial clearance of the wall's output-shaft hole

# ── Reference-only extras — not in the visualiser, needed to turn the 2D
#    cross-section into solids. First pass; expect to retune once this is
#    open in FreeCAD. ──────────────────────────────────────────────────────
cone_apex_trim = 3.0   # mm — truncate each cone this far from its mathematical
                        #      apex (avoids a zero-thickness tip; leaves room
                        #      for the shaft bore to start)
cone_margin    = 6.0   # mm — extra generatrix length past s0+L, structural
                        #      material beyond the last ring

groove_depth       = d_oring * 0.75   # mm — O-ring groove depth (~75% of wire dia)
groove_bottom_frac = 1.30             # dovetail bottom width, × d_oring — keys
                                       #  the O-ring so it can't roll/slip
                                       #  (lesson from the v4 build, see
                                       #  docs/build-log.md backlog #2)
groove_open_frac   = 1.05             # dovetail opening width, × d_oring
groove_overcut     = 0.3   # mm — the open side extends slightly past the
                            #      nominal surface so the cut is a clean boolean

shaft_d       = 5.0    # mm — motor + output shaft diameter (reference)
guide_rod_d   = 3.0    # mm — carriage guide rod diameter (2 rods, per docs)
rod_span      = 24.0   # mm — X distance between the 2 guide rod centres
gear_face_w   = 5.0    # mm — pinion/corona face width. Trimmed from 8mm (was
                        #      ~4.4x module, generous) to ~2.8x module — thinner
                        #      teeth free up axial (Y) space too, same idea as
                        #      shrinking the radius. First-pass value; revisit
                        #      once real tooth root bending stress is checked
                        #      against the actual output torque.
gear_gap      = 18.0   # mm — clearance, output-cone base → pinion/corona face.
                        #      The carriage plate lives IN this gap (with margin
                        #      on both sides) — must be > carriage_plate_t, not
                        #      just a nominal air gap. (reference packaging —
                        #      first pass, not settled)
corona_rim_t  = 4.0    # mm — corona outer rim beyond the tooth-tip circle
corona_plate_t   = 4.0  # mm — plate closing the corona's back face (away from
                         #      the pinion), fused solid into the output shaft
                         #      — without it the corona is just a floating
                         #      ring, no way to mount it
carriage_plate_t = 10.0 # mm — carriage plate thickness. Bumped from 6mm —
                         #      this plate takes the reaction load every time
                         #      the carriage bottoms out at Preload, so more
                         #      cross-section here is cheap insurance. Still
                         #      fits the gear_gap (18mm) with 4mm clearance
                         #      each side.
carriage_margin   = 9.0 # mm — carriage plate half-size beyond the rod span.
                         #      Bumped from 6mm for more material around the
                         #      rod bores (less chance of the holes tearing
                         #      out / more length to bush them properly later).
carriage_bore_clr = 0.4 # mm — DIAMETRAL clearance of the plate's central
                         #      through-bore around the shaft. This bore is
                         #      NOT a running surface (the bearings below carry
                         #      the shaft) — it only has to let the shaft pass
                         #      between the two bearing seats.

# ── Carriage bearings — MR105ZZ, the project's single standard bearing ────────
# The output shaft rotates continuously inside the carriage while transmitting
# torque, so a plain bore would just rub. TWO bearings, one seated at each face
# of the plate: the cone hangs off one end of the shaft and the pinion off the
# other, so a single narrow bearing would leave both overhung with nothing to
# react their tilting moment. The 2mm of plate left between the seats is the
# shoulder that axially locates both.
brg_id        = 5.0    # mm — bearing bore (= shaft)      ┐ MR105ZZ
brg_od        = 10.0   # mm — bearing outer diameter      │ (5×10×4)
brg_w         = 4.0    # mm — bearing width               ┘
brg_fit_press = 0.15   # mm — radial add to the seat for a PRESS fit (→ Ø10.3).
                        #      FDM-calibrated on the Creality Hi, see
                        #      docs/build-log.md — that calibration predates v5
                        #      but is a printer property, still valid.
rail_margin   = 10.0   # mm — guide rod length past where the CARRIAGE PLATE's
                        #      own extent reaches, at each end of its travel
                        #      (not past the bare stop positions — the plate
                        #      is ±carriage_half around those, see DERIVED)

# ── FDM print calibration (Creality Hi / PLA) — see docs/build-log.md ─────────
# The PRINTED parts here are the two cones, the pinion, the CoronaShaft and the
# carriage plate. This printer runs small holes ~0.5 mm UNDERSIZE, so a bore
# modelled at nominal Ø5 prints ~4.5 and the Ø5 rod will not enter — the v4
# build hit exactly this (build-log backlog #5, and the note about the disc
# centring bore). Every hole that has to receive real hardware is therefore
# modelled oversize by these calibrated amounts, NOT at nominal.
# (For SLS/another printer, reset these toward nominal and re-calibrate.)
fdm_shaft_hole_d = 5.8  # mm — modelled Ø for a Ø5 rod to slip through / seat a
                         #      set-screwed hub. Measured on coupon v2.
fdm_rod_hole_d   = 3.8  # mm — modelled Ø for the Ø3 guide rods. NOT directly
                         #      measured (the coupon only covered Ø5/Ø10/M3) —
                         #      extrapolated from the same ~0.5 undersize, and
                         #      this one is a SLIDING fit that the whole
                         #      mechanism's single DOF depends on, so verify it
                         #      on a test print before committing.

# ═══════════════════════════════════════════════════════════════════
# DERIVED  (see docs/clutch-geometry.md for the equations)
# ═══════════════════════════════════════════════════════════════════

alpha    = math.radians(alpha_deg)
beta_out = math.radians(90.0 - alpha_deg)   # output cone half-angle from its own (Y) axis

ratio_fric  = math.sin(alpha) / math.sin(beta_out)
ratio_gear  = Zp / Zc
ratio_total = ratio_fric * ratio_gear

e_dist         = m_mod * (Zc - Zp) / 2.0
r_tip_p        = m_mod * (Zp / 2.0 + 1.0)
r_tip_c        = m_mod * (Zc / 2.0 - 1.0)          # tooth-tip radius (mesh geometry)
r_corona_pitch = m_mod * Zc / 2.0                  # pitch radius — freecad.gears sizes
                                                    # the corona's OUTER solid rim off
                                                    # this (outside_diameter = pitch +
                                                    # 2*thickness), NOT off r_tip_c —
                                                    # using r_tip_c for the plate/rim
                                                    # made it 2 mm undersize vs the ring
r_corona_outer = r_corona_pitch + corona_rim_t
free_float     = r_tip_c - r_tip_p
mesh_travel    = 2.0 * m_mod
root_clearance = 0.25 * m_mod
stroke         = e_dist + g_margin + delta_preload

carriage_half  = rod_span / 2.0 + carriage_margin   # plate's own half-extent in
                                                     # Z, shared by the plate
                                                     # halves and the guide-rod
                                                     # length below — the rod
                                                     # has to clear the PLATE's
                                                     # own size, not just the
                                                     # stop-to-stop travel

q_pitch    = L_line / (2 * n_rings - 2)
h_offset   = math.sqrt(max(d_oring ** 2 - q_pitch ** 2, 0.0))
apex_sep   = h_offset / math.sin(alpha)
micro_slip = apex_sep / L_line

# ── Symmetric stop ladder (bidirectional) ────────────────────────────────────
# The output shaft (and its corona) sit at z = 0, the mid-plane. FREE is the
# middle: the pinion is concentric with the corona, centre distance 0. From
# there the carriage can travel EITHER WAY, and the ladder is the same in both
# directions — only the sign changes:
#
#   |z| = 0           free      pinion concentric, shaft drives nothing
#   |z| = e           mesh      centre distance reached, rings still apart
#   |z| = e + g       contact   rubber flanks touch, normal force zero
#   |z| = e + g + d   preload   flanks compressed, torque transmitted
#
# Down engages the LOWER motor cone, up engages the UPPER one. Both cones turn
# with the same shaft, but the contact sits on opposite sides of the output
# axis, so the output shaft turns the opposite way — one motor, one axis, both
# directions. (The internal mesh does not care about the sign: the pinion is
# captive inside the corona and meshes at whatever azimuth it is offset to.)
Z_FREE     = 0.0
Z_MESH     = e_dist
Z_CONTACT  = e_dist + g_margin
Z_PRELOAD  = e_dist + g_margin + delta_preload
STOPS = {"free": Z_FREE, "mesh": Z_MESH, "contact": Z_CONTACT, "preload": Z_PRELOAD}
z_carriage = CARRIAGE_DIR * STOPS[CARRIAGE_STOP]
z_corona   = 0.0

# Each motor cone's apex sits exactly where the output cone's apex lands at
# CONTACT on that side — that is what makes the two apexes common, which is the
# whole basis of the matched-surface-speed contact. So the separation is not a
# free choice: it falls out of the gear stage.
motor_apex_z   = Z_CONTACT
motor_cone_sep = 2.0 * motor_apex_z
total_travel   = 2.0 * Z_PRELOAD

# Ring positions along the generatrix (arc length s from the apex).
motor_ring_s  = [s0 + 2 * q_pitch * k for k in range(n_rings)]
output_ring_s = [s0 + q_pitch + 2 * q_pitch * k for k in range(n_rings - 1)]

# Constraint checks (docs/clutch-geometry.md §12)
constraints = [
    ("ratio_total < 1  (reduction, never 1:1)",
     ratio_total, "< 1", ratio_total < 1.0),
    ("mesh before rubber contact  (|Z_MESH| < |Z_CONTACT|)",
     Z_CONTACT - Z_MESH, "> 0", Z_CONTACT > Z_MESH),
    ("g + delta <= 0.25*m  (root clearance)",
     g_margin + delta_preload, f"<= {root_clearance:.2f}",
     (g_margin + delta_preload) <= root_clearance),
    ("q < d  (ring flanks can touch)",
     q_pitch, f"< {d_oring:.2f}", q_pitch < d_oring),
    ("Zc - Zp >= 8  (internal mesh interference)",
     Zc - Zp, ">= 8", (Zc - Zp) >= 8),
]

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def v(x, y, z):
    return App.Vector(x, y, z)


def add(doc, name, shape, color=(0.8, 0.8, 0.8), transparency=0):
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if HAS_GUI and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency
    return obj


def cyl(r, h, base, direction=v(0, 0, 1)):
    return Part.makeCylinder(r, h, base, direction)




def cone_ring_groove(apex, axis_unit, radial_ref, beta, s_center):
    """Revolved dovetail groove cut into a cone's flank, centred at generatrix
    distance s_center from apex. axis_unit is the direction from the apex INTO
    the material (s increases along it); radial_ref is any unit vector
    perpendicular to axis_unit (defines which meridian plane the profile is
    built in — irrelevant to the result, since revolve sweeps 360°).

    gdir = generatrix direction (away from apex, tilted by beta from axis_unit)
    ndir = perpendicular to gdir in the same meridian plane, pointing OUT of
           the solid (away from the axis) — matches a normal pulley-groove
           cut, generalised from a cylinder (beta=0) to a cone.
    """
    cb, sb = math.cos(beta), math.sin(beta)
    gdir = v(axis_unit.x * cb + radial_ref.x * sb,
             axis_unit.y * cb + radial_ref.y * sb,
             axis_unit.z * cb + radial_ref.z * sb)
    ndir = v(radial_ref.x * cb - axis_unit.x * sb,
             radial_ref.y * cb - axis_unit.y * sb,
             radial_ref.z * cb - axis_unit.z * sb)

    centre = v(apex.x + s_center * gdir.x,
               apex.y + s_center * gdir.y,
               apex.z + s_center * gdir.z)

    gw_bot = d_oring * groove_bottom_frac
    gw_open = d_oring * groove_open_frac

    def pt(along_g, along_n):
        return v(centre.x + along_g * gdir.x + along_n * ndir.x,
                  centre.y + along_g * gdir.y + along_n * ndir.y,
                  centre.z + along_g * gdir.z + along_n * ndir.z)

    p1 = pt(-gw_bot / 2, -groove_depth)
    p2 = pt(gw_bot / 2, -groove_depth)
    p3 = pt(gw_open / 2, groove_overcut)
    p4 = pt(-gw_open / 2, groove_overcut)
    pts = [p1, p2, p3, p4]
    wire = Part.Wire([Part.makeLine(pts[i], pts[(i + 1) % 4]) for i in range(4)])
    return Part.Face(wire).revolve(apex, axis_unit, 360)


def make_motor_cone(sd):
    """One of the two motor cones on the central vertical shaft. sd=-1 is the
    LOWER cone (apex up at z=-motor_apex_z, flaring downward), sd=+1 the UPPER
    one (apex down at z=+motor_apex_z, flaring upward) — apexes facing each
    other across the waist where the output cones live. They are the same part
    printed twice, one fitted upside down.

    Truncated near the apex (cone_apex_trim) and extended past the ring zone
    (cone_margin) for a printable part."""
    apex = v(0, 0, sd * motor_apex_z)
    axis_unit = v(0, 0, sd)          # from the apex INTO the material, i.e.
                                      # away from the waist
    radial_ref = v(1, 0, 0)

    s_lo, s_hi = cone_apex_trim, s0 + L_line + cone_margin
    z_lo = apex.z + sd * s_lo * math.cos(alpha)      # near the apex, narrow
    z_hi = apex.z + sd * s_hi * math.cos(alpha)      # far from it, wide
    r_lo, r_hi = s_lo * math.sin(alpha), s_hi * math.sin(alpha)

    z_bot, z_top = min(z_lo, z_hi), max(z_lo, z_hi)
    r_bot, r_top = (r_hi, r_lo) if sd < 0 else (r_lo, r_hi)

    cone = Part.makeCone(r_bot, r_top, z_top - z_bot, v(0, 0, z_bot), v(0, 0, 1))
    cone = cone.cut(cyl(fdm_shaft_hole_d / 2, z_top - z_bot + 2,
                        v(0, 0, z_bot - 1)))

    for s in motor_ring_s:
        cone = cone.cut(cone_ring_groove(apex, axis_unit, radial_ref, alpha, s))
    return cone


def make_output_cone(z_c):
    """Cone on the output shaft, rotating with it. Apex at (0, 0, z_c), axis Y,
    flares toward +Y (away from the motor shaft, toward the pinion / wall side).
    Bored for the shaft: cone and pinion are SEPARATE parts clamped to a common
    shaft (set screws on a filed flat — the joint pattern the project already
    uses for its wheel/disc hubs). Fusing the two into one rigid piece was
    tried and reverted: it forced the carriage to split into halves around the
    shaft, which then needed an impractically long through-bolt to close.
    """
    apex = v(0, 0, z_c)
    axis_unit = v(0, 1, 0)
    radial_ref = v(0, 0, 1)

    s_lo, s_hi = cone_apex_trim, s0 + L_line + cone_margin
    y_lo, y_hi = s_lo * math.cos(beta_out), s_hi * math.cos(beta_out)
    r_lo, r_hi = s_lo * math.sin(beta_out), s_hi * math.sin(beta_out)
    height = y_hi - y_lo

    base_pt = v(apex.x, apex.y + y_hi, apex.z)
    cone = Part.makeCone(r_hi, r_lo, height, base_pt, v(0, -1, 0))
    cone = cone.cut(cyl(fdm_shaft_hole_d / 2, height + 2,
                        v(apex.x, apex.y + y_lo - 1, apex.z), v(0, 1, 0)))

    for s in output_ring_s:
        cone = cone.cut(cone_ring_groove(apex, axis_unit, radial_ref, beta_out, s))
    return cone


def cone_base_y(beta):
    """Y-reach of a cone's outer (base) rim from its apex — used to place the
    gear stage and carriage clear of the friction cone."""
    return (s0 + L_line + cone_margin) * math.cos(beta)


def make_pinion_ref(y_centre, z_c):
    """Reference cylinder at the pinion tip-circle diameter. Rigid to the
    output cone/carriage — same axis (Y, at height z_c). NO TEETH — placeholder
    for a real involute pinion."""
    return cyl(r_tip_p, gear_face_w, v(0, y_centre - gear_face_w / 2, z_c), v(0, 1, 0))


def _gear_placement(y_centre, z_c):
    """freecad.gears extrudes along local +Z from local z=0. Rotate that axis
    onto global +Y (Rotation(X, -90) maps local Z -> global Y), then place the
    extrusion's start face at (0, y_centre - gear_face_w/2, z_c) — matching the
    Y-centring convention used by the reference cylinders above."""
    base = v(0, y_centre - gear_face_w / 2, z_c)
    return App.Placement(base, App.Rotation(App.Vector(1, 0, 0), -90))


def _attach_gear_view(obj):
    """freecad.gears' own command code always attaches ViewProviderGear to the
    ViewObject BEFORE the geometry proxy (see commands.py BaseCommand.create,
    GUI branch) — skipping it left Pinion/Corona with no working ViewObject
    proxy, so FreeCAD silently failed to display them even though the Shape
    itself was valid."""
    if HAS_GUI and obj.ViewObject is not None:
        ViewProviderGear(obj.ViewObject)


def make_pinion_real(doc, y_centre, z_c):
    """Real involute pinion (freecad.gears), clamped to the output shaft."""
    obj = doc.addObject("Part::FeaturePython", "Pinion")
    _attach_gear_view(obj)
    InvoluteGear(obj)
    obj.num_teeth = Zp
    obj.module = f"{m_mod} mm"
    obj.height = f"{gear_face_w} mm"
    obj.axle_hole = True        # freecad.gears builds gears solid by default
    obj.axle_holesize = f"{fdm_shaft_hole_d} mm"   # FDM-compensated, not nominal
    obj.Placement = _gear_placement(y_centre, z_c)
    return obj


def make_corona_real(doc, y_centre):
    """Real internal involute gear (freecad.gears), fixed to the output shaft."""
    obj = doc.addObject("Part::FeaturePython", "Corona")
    _attach_gear_view(obj)
    InternalInvoluteGear(obj)
    obj.num_teeth = Zc
    obj.module = f"{m_mod} mm"
    obj.height = f"{gear_face_w} mm"
    obj.thickness = f"{corona_rim_t} mm"
    obj.Placement = _gear_placement(y_centre, z_corona)
    return obj


def make_corona_ref(y_centre):
    """Reference tube at the corona tip-circle diameter (internal gear — teeth
    point inward, so the tip circle is the INNER bound here). Fixed to the
    output shaft — axis Y, height fixed at z_corona. NO TEETH — placeholder."""
    outer = cyl(r_corona_outer, gear_face_w,
                v(0, y_centre - gear_face_w / 2, z_corona), v(0, 1, 0))
    inner = cyl(r_tip_c, gear_face_w + 2,
                v(0, y_centre - gear_face_w / 2 - 1, z_corona), v(0, 1, 0))
    return outer.cut(inner)


def make_corona_plate(y_centre):
    """Disc closing the corona's back face (the side away from the pinion).
    No bore here: the plate is fused solid into the output shaft (see BUILD —
    corona + plate + shaft are one piece, no separate joint), so there's
    nothing for a bore to give clearance to."""
    y0 = y_centre + gear_face_w / 2   # ring's back face
    return cyl(r_corona_outer, corona_plate_t, v(0, y0, z_corona), v(0, 1, 0))


def make_output_shaft_ref(y_start):
    """Fixed output shaft, axis Y at z_corona, continuing past the corona
    toward the next hub of the tree."""
    length = 40.0
    return cyl(shaft_d / 2, length, v(0, y_start, z_corona), v(0, 1, 0))


def carriage_bearing_y(y_centre, sd):
    """Y of the inner face of one carriage bearing. sd=-1 is the cone-side
    bearing, sd=+1 the pinion-side one; each is seated flush at its own face of
    the plate, so they sit brg_w apart from the faces inward."""
    face = y_centre + sd * carriage_plate_t / 2
    return face - sd * brg_w


def make_carriage_plate(y_centre, z_c):
    """Carriage plate joining the output shaft to the 2 guide rods. ONE piece:
    the shaft threads through its bore at assembly, and the cone and pinion are
    clamped onto the shaft afterwards, one at each end. Sits in the gear_gap
    (between the output cone's base and the pinion face), clear of both.
    Perpendicular to Y, centred on the current carriage height.

    Carries two MR105ZZ bearings, one seated at each face — see the parameter
    block. The central through-bore between the seats is a clearance passage,
    not a running surface. Real carriage design (actuator attachment, rod
    bushings) is still a later pass."""
    half = carriage_half
    plate = Part.makeBox(2 * half, carriage_plate_t, 2 * half,
                          v(-half, y_centre - carriage_plate_t / 2, z_c - half))
    plate = plate.cut(cyl(fdm_shaft_hole_d / 2 + carriage_bore_clr / 2,
                          carriage_plate_t + 2,
                          v(0, y_centre - carriage_plate_t / 2 - 1, z_c), v(0, 1, 0)))

    # Bearing seats, one per face, cut inward; the material left between them
    # is the shoulder that locates both bearings axially.
    for sd in (-1, 1):
        y_face = y_centre + sd * carriage_plate_t / 2
        y_start = min(y_face, y_face - sd * brg_w)
        plate = plate.cut(cyl(brg_od / 2 + brg_fit_press, brg_w,
                              v(0, y_start, z_c), v(0, 1, 0)))

    for sx in (-1, 1):
        # Rods run vertically (Z), same as make_guide_rod below — the hole has
        # to bore through the plate's full Z-extent, NOT its Y-thickness (that
        # was an earlier bug: cutting along Y only pierced the thickness,
        # leaving a hole perpendicular to the actual rod).
        plate = plate.cut(cyl(fdm_rod_hole_d / 2, 2 * half + 2,
                              v(sx * rod_span / 2, y_centre, z_c - half - 1),
                              v(0, 0, 1)))
    return plate


def make_carriage_bearing(y_centre, sd):
    """One MR105ZZ in the carriage (reference part — purchased)."""
    y0 = carriage_bearing_y(y_centre, sd)
    y_start = min(y0, y0 + sd * brg_w)
    outer = cyl(brg_od / 2, brg_w, v(0, y_start, z_carriage), v(0, 1, 0))
    inner = cyl(brg_id / 2 + 0.05, brg_w + 2, v(0, y_start - 1, z_carriage), v(0, 1, 0))
    return outer.cut(inner)


def make_guide_rod(sx, y_centre):
    """Fixed vertical guide rod, spanning the carriage's full travel PLUS the
    carriage plate's own extent at each end (not just the bare stop-to-stop
    range — the plate is +-carriage_half around wherever its centre sits),
    plus rail_margin for mounting into the frame."""
    z_top = Z_FREE + carriage_half + rail_margin
    z_bot = Z_PRELOAD - carriage_half - rail_margin
    return cyl(guide_rod_d / 2, z_top - z_bot, v(sx * rod_span / 2, y_centre, z_bot))

# ═══════════════════════════════════════════════════════════════════
# BUILD DOCUMENT
# ═══════════════════════════════════════════════════════════════════

doc_name = "Motcore_v5"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

_cone_base_y = cone_base_y(beta_out)
_carriage_y  = _cone_base_y + gear_gap / 2       # mid-gap: clear of both the
                                                  # cone base and the gear stage
_gear_y      = _cone_base_y + gear_gap + gear_face_w / 2

# Cube half-size: inside face of the wall, clear of the outermost rotating part
# (the corona's back plate).
cube_half = _gear_y + gear_face_w / 2 + corona_plate_t + wall_gap
cube_out  = cube_half + wall_thick

def _color(obj, color, transparency=0):
    if HAS_GUI and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency

# ── One output axis, built along +Y; every other axis is a rotated copy ───────
if GEARS_AVAILABLE:
    # Both gears are only needed as SHAPES here (they get rotated/mirrored into
    # 8 positions), so the parametric objects are deleted once their shape has
    # been taken — leaving them in the tree reads as duplicate parts, and
    # editing them there would not change the copies anyway. Tooth counts and
    # module are edited in PARAMETERS at the top of this macro.
    _pin_obj = make_pinion_real(doc, _gear_y, z_carriage)
    _cor_obj = make_corona_real(doc, _gear_y)
    doc.recompute()            # need both .Shape ready before copying them
    pinion_shape = _pin_obj.Shape.copy()
    corona_shape = _cor_obj.Shape.copy()
    doc.removeObject(_pin_obj.Name)
    doc.removeObject(_cor_obj.Name)
else:
    pinion_shape = make_pinion_ref(_gear_y, z_carriage)
    corona_shape = make_corona_ref(_gear_y)

# Corona + its mounting plate + the output-shaft stub, fused into ONE part —
# a separate shaft/hub joint here would just be an extra failure point on the
# torque path for no benefit (no bearing-precision running surface elsewhere
# on this shaft yet that would call for a hard metal insert).
coronashaft = corona_shape.fuse(make_corona_plate(_gear_y)).fuse(
    make_output_shaft_ref(_gear_y + gear_face_w / 2))

# Cone and pinion are separate parts on a shared shaft (set screws on a filed
# flat) — the shaft threads through the one-piece carriage plate first, then a
# part is clamped on at each end.
AXIS_PARTS = [
    ("OutputCone",   make_output_cone(z_carriage),            (0.20, 0.80, 0.60), 0),
    ("Pinion",       pinion_shape,                            (0.85, 0.65, 0.10), 0),
    ("CoronaShaft",  coronashaft,                             (0.55, 0.55, 0.85), 0),
    ("CarriageShaft", cyl(shaft_d / 2, gear_gap + gear_face_w + 4,
                          v(0, _cone_base_y - 2, z_carriage), v(0, 1, 0)),
                                                              (0.60, 0.60, 0.60), 0),
    ("CarriagePlate", make_carriage_plate(_carriage_y, z_carriage),
                                                              (0.70, 0.70, 0.70), 50),
    ("BearingCone",  make_carriage_bearing(_carriage_y, -1),   (0.30, 0.30, 0.32), 0),
    ("BearingPinion", make_carriage_bearing(_carriage_y, 1),   (0.30, 0.30, 0.32), 0),
    ("GuideRodPos",  make_guide_rod(1, _carriage_y),           (0.50, 0.50, 0.55), 0),
    ("GuideRodNeg",  make_guide_rod(-1, _carriage_y),          (0.50, 0.50, 0.55), 0),
]

# Four walls / four axes, 90° apart about Z.
AXES = [("PosY", 0.0), ("NegX", 90.0), ("NegY", 180.0), ("PosX", 270.0)]

Z_AXIS = v(0, 0, 1)
ORIGIN = v(0, 0, 0)

def place(shape, rot_deg):
    """A copy of `shape` rotated onto its wall.

    Placement-level rotation only. Do not "bake" the geometry with
    transformGeometry to allow a mirror instead — it distorts the
    freecad.gears corona, and Shape.mirror() composes wrongly with the
    Placement the gears carry.

    Each call starts from its own .copy(): Shape.rotate() mutates in place and
    returns the same object, so reusing one base shape across the loop
    corrupts it (a gotcha already hit in the v4 macro)."""
    s = shape.copy()
    if rot_deg:
        s.rotate(ORIGIN, Z_AXIS, rot_deg)
    return s

for _ax_name, _ax_rot in AXES:
    for _pname, _pshape, _pcolor, _ptrans in AXIS_PARTS:
        add(doc, f"{_pname}_{_ax_name}", place(_pshape, _ax_rot),
            color=_pcolor, transparency=_ptrans)

# ── Central column: the two motor cones + the shaft through them ─────────────
# Same part twice, the upper one fitted upside down.
for _sd, _tag in ((-1, "Lower"), (1, "Upper")):
    add(doc, f"MotorCone{_tag}", make_motor_cone(_sd), color=(1.0, 0.60, 0.15))

_ms_reach = motor_apex_z + (s0 + L_line + cone_margin) * math.cos(alpha) + 15
add(doc, "MotorShaft", cyl(shaft_d / 2, 2 * _ms_reach, v(0, 0, -_ms_reach)),
    color=(0.6, 0.6, 0.6))


def make_cube_wall(rot_deg):
    """One cube wall (reference), at +Y before rotation, with a single
    clearance hole for its one output shaft (which sits at the mid-plane)."""
    wall = Part.makeBox(2 * cube_out, wall_thick, 2 * cube_out,
                        v(-cube_out, cube_half, -cube_out))
    wall = wall.cut(cyl(shaft_d / 2 + wall_shaft_clr, wall_thick + 2,
                        v(0, cube_half - 1, z_corona), v(0, 1, 0)))
    wall.rotate(ORIGIN, Z_AXIS, rot_deg)
    return wall

if SHOW_WALLS:
    for _ax_name, _ax_rot in AXES:
        add(doc, f"Wall_{_ax_name}", make_cube_wall(_ax_rot),
            color=(0.45, 0.55, 0.75), transparency=70)

doc.recompute()

# ── Cube geometry checks ─────────────────────────────────────────────────────
# Two cones sharing an apex, axes 90° apart: they clear each other iff the sum
# of their half-angles is under 90°. This is the v5 equivalent of the
# four-wheels-around-the-disc non-overlap condition that capped v4's ratio.
_adj_gap_deg = 90.0 - 2 * (90.0 - alpha_deg)

# Real solid check on the FULL assembly (not just the cones): fuse one axis,
# rotate a copy by 90°, and see whether they share any volume.
_asm = AXIS_PARTS[0][1].copy()
for _, _s, _, _ in AXIS_PARTS[1:]:
    _asm = _asm.fuse(_s)
_asm_rot = _asm.copy()
_asm_rot.rotate(ORIGIN, Z_AXIS, 90.0)
_adj_overlap = _asm.common(_asm_rot).Volume

# The bidirectional check that matters: when the carriage engages one motor
# cone, the output cone must stay clear of the OTHER one — and at FREE it must
# be clear of both. Plastic-on-plastic overlap is measured here; the rubber
# rings sit proud of these surfaces and are what actually meet.
_mc_lo, _mc_up = make_motor_cone(-1), make_motor_cone(1)
_oc_now  = AXIS_PARTS[0][1]
_oc_free = make_output_cone(Z_FREE)
_ov_lo, _ov_up = _oc_now.common(_mc_lo).Volume, _oc_now.common(_mc_up).Volume
_ov_engaged, _ov_idle = ((_ov_lo, _ov_up) if CARRIAGE_DIR < 0 else (_ov_up, _ov_lo))
_engaged_name = "lower" if CARRIAGE_DIR < 0 else "upper"
_ov_free_max = max(_oc_free.common(_mc_lo).Volume, _oc_free.common(_mc_up).Volume)

if HAS_GUI:
    try:
        Gui.ActiveDocument = Gui.getDocument(doc.Name)
        Gui.SendMsgToActiveView("ViewFit")
        Gui.activeDocument().activeView().viewIsometric()
    except AttributeError:
        pass   # freecadcmd (no real GUI view) — fine, geometry already built

print("=" * 68)
print(f"Motcore v5 — Vertical Cone Clutch ({len(AXES)} bidirectional axes)")
print(f"  alpha (motor half-angle):  {alpha_deg:.1f} deg  ->  output half-angle {90-alpha_deg:.1f} deg")
print(f"  Friction ratio:  {ratio_fric:.3f}   Gear ratio: {ratio_gear:.3f}"
      f"   Total: {ratio_total:.3f}  (-> {1/ratio_total:.2f}x torque)")
print(f"  Module m = {m_mod:.2f}   e (centre dist) = {e_dist:.2f} mm"
      f"   free float = {free_float:.2f} mm   mesh travel = {mesh_travel:.2f} mm")
print(f"  Half stroke = e + g + delta = {stroke:.2f} mm"
      f"   full travel (down..up) = {total_travel:.2f} mm")
print(f"  Ring pitch q = {q_pitch:.3f} mm   apex separation = {apex_sep:.3f} mm"
      f"   micro-slip = {micro_slip*100:.1f}%")
print(f"  Stops, |z| of the carriage from the mid-plane (free = 0, mirrored"
      f" both ways):")
print(f"      free=0.00  mesh={Z_MESH:.2f}  contact={Z_CONTACT:.2f}"
      f"  preload={Z_PRELOAD:.2f}")
print(f"  Motor cone apexes at z = +-{motor_apex_z:.2f} (separation"
      f" {motor_cone_sep:.2f} mm) — DERIVED, = contact")
print(f"  Rendering: '{CARRIAGE_STOP}' {'down' if CARRIAGE_DIR < 0 else 'up'}"
      f"  ->  z_carriage = {z_carriage:.2f} mm   (corona fixed at z ="
      f" {z_corona:.2f})")
print("-" * 68)
for label, value, target, ok in constraints:
    mark = "OK " if ok else "FAIL"
    print(f"  [{mark}] {label}:  {value:.3f}  ({target})")
print("-" * 68)
_shoulder = carriage_plate_t - 2 * brg_w
print(f"  Carriage: 2x MR105ZZ ({brg_id:.0f}x{brg_od:.0f}x{brg_w:.0f}), one per face,"
      f" seat Ø{brg_od + 2 * brg_fit_press:.1f} press")
print(f"    [{'OK ' if _shoulder > 0 else 'FAIL'}] shoulder between seats:"
      f" {_shoulder:.2f} mm  (plate {carriage_plate_t:.1f} - 2 x {brg_w:.1f} bearing)")
print("-" * 68)
print(f"  CUBE: {len(AXES)} axes, one per wall, each bidirectional"
      f" (2 motor cones)")
print(f"    side = {2 * cube_out:.1f} mm (half {cube_half:.1f} inside"
      f" + {wall_thick:.1f} wall)")
print(f"    [{'OK ' if _adj_gap_deg > 0 else 'FAIL'}] adjacent cones, angular gap:"
      f" {_adj_gap_deg:.1f} deg  (90 - 2 x {90-alpha_deg:.0f} deg half-angle)")
print(f"    [{'OK ' if _adj_overlap < 1e-6 else 'FAIL'}] adjacent FULL assemblies"
      f" (90 deg apart) overlap: {_adj_overlap:.2f} mm3")
print(f"    [{'OK ' if _ov_idle < 1e-6 else 'FAIL'}] output cone clears the IDLE"
      f" motor cone while engaging the {_engaged_name}: {_ov_idle:.2f} mm3")
print(f"    [{'OK ' if _ov_free_max < 1e-6 else 'FAIL'}] at FREE the output cone"
      f" clears BOTH motor cones: {_ov_free_max:.2f} mm3")
print(f"    engaged-side plastic interference: {_ov_engaged:.2f} mm3"
      f"  (rubber sits proud of this; > 0 only expected at 'preload')")
print("-" * 68)
print("  PRINTED (PLA/PETG): MotorCone, OutputCone, Pinion, CoronaShaft, CarriagePlate")
print("  PURCHASED: 2x MR105ZZ, Ø5 rod, 2x Ø3 guide rod, O-rings, set screws")
print(f"  FDM holes (modelled oversize, this printer runs ~0.5 under):"
      f"  shaft Ø{fdm_shaft_hole_d:.1f}  rod Ø{fdm_rod_hole_d:.1f}"
      f"  bearing seat Ø{brg_od + 2 * brg_fit_press:.1f}")
print("-" * 68)
print(f"  Motor ring positions (s from apex): {[f'{s:.2f}' for s in motor_ring_s]}")
print(f"  Output ring positions (s from apex): {[f'{s:.2f}' for s in output_ring_s]}")
print("=" * 68)
if GEARS_AVAILABLE:
    print("Gear stage: real involute pinion/corona (freecad.gears), e = "
          f"{e_dist:.2f} mm centre distance at mesh.")
else:
    print("NOTE: freecad.gears not found -- pinion/corona are reference")
    print("cylinders (no teeth). Install the 'Gear' addon to get real teeth.")
print("Carriage/rail packaging (gear_gap, rod_span, carriage plate) is a")
print("first-pass layout, not derived from the visualiser -- expect to")
print("retune visually in FreeCAD.")
