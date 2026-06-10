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

A_deg         = 2.0   # deg  — engagement angle (shaft tilt from horizontal)
B             = 35.0  # mm   — UJ pivot → output wheel centre (along shaft)
R             = 20.0  # mm   — contact radius (O-ring outer edge = motor disc rim)
dw            = 2.5   # mm   — O-ring wire diameter

wall_thick    = 4.0   # mm   — cube wall thickness
shaft_dia     = 5.0   # mm   — output shaft diameter
motor_shaft_d = 8.0   # mm   — motor (Z) shaft diameter
# cube_h derived below from cube_size (the cube is cubic: height = side length)

neck_dia      = 2.5   # mm   — neck diameter (torsion strength + stiffness match)
neck_len      = 13.0  # mm   — neck length (matched to combined blade stiffness)
head_gap      = 2.0   # mm   — clearance between wheel outer face and bracket head
head_depth    = 8.0   # mm   — bracket head bearing length
lever_len     = 15.0  # mm   — fixed shaft stub beyond wall (servo lever arm side)

# ── Frame (top + bottom plates + lever pivot posts) ───────────────────────────
plate_t       = 4.0   # mm   — plate thickness (top and bottom)
post_w        = 10.0  # mm   — post width  in pa direction (along trunnion hole)
post_d        =  5.0  # mm   — post depth in axis direction (thin face)
post_side_gap = 1.5   # mm   — clearance between blade outer edge and post
pivot_z       =  0.0  # mm   — lever pivot socket Z (0 = shaft height = cube mid)
trun_r        =  2.1  # mm   — trunnion socket radius (Ø4 mm trunnion + 0.1 clearance)
head_trun_r   =  1.5  # mm   — head trunnion pin radius
head_trun_len =  5.0  # mm   — head trunnion pin length (sticks out from head side)

# ── Engagement lever blade (integral with bracket head, extends downward) ─────
eng_blade_l  = 30.0  # mm — spring section length in −Z (XZ arm, free to flex)
eng_blade_w  = 10.0  # mm — width in pa direction (centred at pa = 0)
eng_blade_t  =  1.5  # mm — XZ arm thickness in Y (spring direction — keep thin)
eng_yz_t     =  4.0  # mm — YZ arm thickness in X (contact arm — no need to flex)
eng_overlap  =  8.0  # mm — how far YZ arm overlaps upward into the XZ spring section
eng_tip_clr  =  3.0  # mm — clearance above bottom plate for YZ contact arm
eng_slot_w   =  2.5  # mm — ranura: ancho en pa (diámetro muñón + holgura)
eng_slot_h   =  6.0  # mm — ranura: longitud en Z (Ø pin 2.5 + subida Z a ±45° 2.3 + holgura 0.5)

# ── MG90D servo (Tower Pro — digital, metal gears, ~0.1° resolution) ─────────
sg90_l        = 22.8  # mm — body depth along shaft axis (pa direction)
sg90_h        = 25.4  # mm — body cross-section in Y direction (tall axis)
sg90_w        = 12.8  # mm — body cross-section in Z direction
sg90_sd       =  4.8  # mm — output shaft outer diameter (with spline, same as SG90)
sg90_sl       =  4.0  # mm — shaft protrusion beyond body face
sg90_tol      =  0.3  # mm — pocket clearance per side
sg90_wall     =  2.0  # mm — pocket wall thickness
sg90_shaft_off =  6.0  # mm — eje desplazado desde la cara delantera del cuerpo (dimensión sg90_h)
sg90_arm_r     =  8.0  # mm — radio del brazo del servo (pin en punta del brazo)

# ── Modular foot attachment (2× M3 screws + nut traps, no tools needed) ──────
foot_tol      = 0.2   # mm — fit clearance per side (pocket = foot + 2×foot_tol)
m3_nut_af     = 5.7   # mm — M3 hex nut AF with print clearance (5.5 mm + 0.2)
m3_nut_thick  = 2.6   # mm — nut trap depth (2.4 mm nut + 0.2 mm clearance)
m3_screw_r    = 1.6   # mm — M3 clearance hole radius (Ø3.2 mm)
foot_screw_pa = 5.0   # mm — screw centre offset in pa direction (±pa)

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
cube_h    = cube_size          # cube is cubic: internal height = side length
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
head_y1    = head_y0 + head_depth          # head outer face / blade root
blade_gap  = shaft_dia + 1.0               # gap between blades in pa (shaft clears)
blade_w_g  = 6.0                           # blade width in pa
bw_half    = blade_gap / 2 + blade_w_g     # bracket half-width in pa = 9 mm
# Single compliant neck centred in the free span between head and wall
neck_start = head_y1 + (cube_half - head_y1 - neck_len) / 2
neck_end   = neck_start + neck_len

# ── Bracket head half-height in Z (shared with make_lever) ───────────────
head_z_half = shaft_dia / 2 + 3.5   # = 6.0 mm with defaults

# ── Lever pivot post geometry (shared with make_frame and make_lever) ─────
post_pa_off  = bw_half + post_side_gap + post_w / 2  # post centre offset in pa
head_trun_y  = head_y0 + 2.0                          # head trunnion Y (2 mm from inner face)
head_pin_pa  = bw_half + head_trun_len / 2            # head pin midpoint in pa

# ── Modular foot half-width — auto-sized to clear nut traps with 1.5 mm wall ─
_nut_circ_r  = m3_nut_af / math.sqrt(3)              # hex circumradius
foot_half_w  = foot_screw_pa + _nut_circ_r + 1.5     # pa + nut + 1.5 mm wall


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

    # ── Pocket for modular bracket foot ───────────────────────────────────────
    # Built in canonical frame (+Y axis) then rotated to match actual axis.
    # Pocket is 2×foot_tol oversize per side for fit clearance.
    _pkt_w = 2 * foot_half_w + 2 * foot_tol   # width in pa
    _pkt_h = 2 * head_z_half + 2 * foot_tol   # height in Z
    _pocket = Part.makeBox(_pkt_w, wall_thick + 2, _pkt_h,
                           v(-_pkt_w / 2, cube_half - 1, -_pkt_h / 2))
    # M3 screw clearance holes through wall (in +Y canonical)
    for _spa in [foot_screw_pa, -foot_screw_pa]:
        _pocket = _pocket.fuse(cyl(m3_screw_r, wall_thick + 2,
                                   v(_spa, cube_half - 1, 0), v(0, 1, 0)))
    _angle = 0.0
    if   axis.x >  0.5: _angle = -90.0
    elif axis.x < -0.5: _angle =  90.0
    elif axis.y < -0.5: _angle = 180.0
    if _angle:
        _pocket = _pocket.rotate(v(0, 0, 0), v(0, 0, 1), _angle)
    wall = wall.cut(_pocket)

    return wall  # bracket is a separate part; add Bracket_{name} in main loop


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
    # head_z_half is global (defined in DERIVED)

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

    # Engagement lever blade — two independent sections:
    #
    # XZ arm (spring): from head bottom face down eng_blade_l mm.
    #   Thin in Y → flexible in Y → servo force in ±Y creates torque about pa (X).
    #   Front face flush with head outer wall (y = head_y1).
    #   Completely free — YZ arm does NOT overlap this section.
    #
    # YZ arm (servo contact): starts where XZ spring ends, continues to near floor.
    #   Thin in X, wide in Y → large flat face for servo arm to bear against.
    #   Centred at head_y1 (protrudes eng_blade_w/2 beyond head outer face).
    #   Servo: shaft along pa (X), arm sweeps in YZ plane, pushes face in ±Y.
    #
    z_spring_bot = -(head_z_half + eng_blade_l)     # bottom of XZ spring
    z_yz_top     = z_spring_bot + eng_overlap        # YZ arm starts this far into spring
    z_yz_bot     = -(cube_half - eng_tip_clr)        # YZ bottom, eng_tip_clr above base plate
    yz_len       = abs(z_yz_bot - z_yz_top)

    # XZ arm — full spring section
    eng_xz = Part.makeBox(
        eng_blade_w, eng_blade_t, eng_blade_l,
        v(-eng_blade_w / 2, head_y1 - eng_blade_t, z_spring_bot)
    )
    # YZ arm — starts eng_overlap mm above spring bottom, runs to near base plate
    # Centred on the XZ arm mid-plane: y_centre = head_y1 - eng_blade_t/2
    y_ctr = head_y1 - eng_blade_t / 2                 # Y centre of cross section
    yz_y0 = y_ctr - eng_blade_w / 2
    eng_yz = Part.makeBox(
        eng_yz_t, eng_blade_w, yz_len,
        v(-eng_yz_t / 2, yz_y0, z_yz_bot)
    )

    # ── Chamfer the transition corners in the overlap zone ────────────────────
    # XZ side: tapers from ±eng_blade_w/2 to ±eng_blade_t/2 in X at z_spring_bot.
    chamfer_h_xz = (eng_blade_w - eng_blade_t) / 2
    # YZ side: tapers from ±eng_blade_w/2 to ±eng_yz_t/2 in Y at z_yz_top.
    chamfer_h_yz = (eng_blade_w - eng_yz_t) / 2

    # Ranura en la lámina YZ: elongada en Z, estrecha en Y.
    # El brazo cuelga en −Z en neutro → el pin está en el FONDO de la ranura.
    # Al rotar ±α el pin sube en Z y empuja ±Y; la ranura absorbe el desplazamiento Z.
    pin_r        = eng_slot_w / 2                      # radio del muñón
    pin_z        = z_yz_bot + pin_r + 1.0              # centro pin en neutro (fondo útil ranura)
    slot_z0      = pin_z - pin_r                       # borde inferior del corte = 1 mm sobre base
    sg90_z_shaft = pin_z + sg90_arm_r                  # eje encima del centro del pin neutro
    # Ranura con esquina inferior redondeada:
    #   · Parte recta: desde z=pin_z (centro del semicilindro) hacia arriba
    #   · Parte curva: semicilindro a lo largo de X, radio=pin_r, centro en pin_z
    slot_depth   = eng_yz_t + 2                        # profundidad del corte en X
    slot_x0      = -eng_yz_t / 2 - 1                  # arranque del corte en X
    # Ranura en forma de estadio (oblong): dos semicilindros + rectángulo central.
    # Altura total = eng_slot_h, sin cambiar la posición del slot.
    slot_top_ctr = slot_z0 + eng_slot_h - pin_r      # centro semicilindro superior
    slot_rect = Part.makeBox(
        slot_depth, eng_slot_w, eng_slot_h - 2 * pin_r,
        v(slot_x0, y_ctr - pin_r, pin_z)
    )
    slot_round_bot = cyl(pin_r, slot_depth, v(slot_x0, y_ctr, pin_z),        v(1, 0, 0))
    slot_round_top = cyl(pin_r, slot_depth, v(slot_x0, y_ctr, slot_top_ctr), v(1, 0, 0))
    eng_yz = eng_yz.cut(slot_rect.fuse(slot_round_bot).fuse(slot_round_top))

    # XZ blade — chamfer bottom ±X corners
    for sx in [1, -1]:
        pts = [v(sx * eng_blade_w / 2, head_y1 - eng_blade_t, z_spring_bot),
               v(sx * eng_blade_t / 2, head_y1 - eng_blade_t, z_spring_bot),
               v(sx * eng_blade_w / 2, head_y1 - eng_blade_t, z_spring_bot + chamfer_h_xz)]
        wire = Part.Wire([Part.makeLine(pts[0], pts[1]),
                          Part.makeLine(pts[1], pts[2]),
                          Part.makeLine(pts[2], pts[0])])
        eng_xz = eng_xz.cut(Part.Face(wire).extrude(v(0, eng_blade_t, 0)))

    # YZ blade — chamfer top ±Y corners (tapers to eng_yz_t in Y)
    for sy in [1, -1]:
        pts = [v(-eng_yz_t / 2, y_ctr + sy * eng_blade_w / 2, z_yz_top),
               v(-eng_yz_t / 2, y_ctr + sy * eng_yz_t / 2,   z_yz_top),
               v(-eng_yz_t / 2, y_ctr + sy * eng_blade_w / 2, z_yz_top - chamfer_h_yz)]
        wire = Part.Wire([Part.makeLine(pts[0], pts[1]),
                          Part.makeLine(pts[1], pts[2]),
                          Part.makeLine(pts[2], pts[0])])
        eng_yz = eng_yz.cut(Part.Face(wire).extrude(v(eng_yz_t, 0, 0)))

    eng_blade = eng_xz.fuse(eng_yz)

    # ── Modular attachment foot ────────────────────────────────────────────────
    # Rectangular plug (2×foot_half_w × wall_thick × 2×head_z_half) that slides
    # into the pocket cut in the wall.  Canonical: protrudes in +Y from cube_half.
    # Two M3 screw clearance holes in Y + two hex nut traps open from −Y face.
    _nut_cr = m3_nut_af / math.sqrt(3)      # hex circumradius
    foot = Part.makeBox(
        2 * foot_half_w, wall_thick, 2 * head_z_half,
        v(-foot_half_w, cube_half, -head_z_half)
    )
    # Screw clearance holes Ø3.2 mm through full foot depth
    for _spa in [foot_screw_pa, -foot_screw_pa]:
        foot = foot.cut(cyl(m3_screw_r, wall_thick + 2,
                            v(_spa, cube_half - 1, 0), v(0, 1, 0)))
    # Hex nut traps: open from −Y face (y = cube_half), extrude in +Y by m3_nut_thick
    for _spa in [foot_screw_pa, -foot_screw_pa]:
        _hv = [v(_spa + _nut_cr * math.cos(i * math.pi / 3),
                 cube_half,
                 _nut_cr * math.sin(i * math.pi / 3)) for i in range(6)]
        _hv.append(_hv[0])
        _trap = Part.Face(Part.Wire(
            [Part.makeLine(_hv[j], _hv[j + 1]) for j in range(6)]
        )).extrude(v(0, m3_nut_thick, 0))
        foot = foot.cut(_trap)

    shape = head.fuse(blade_r).fuse(blade_l).fuse(eng_blade).fuse(foot)

    # Rotate canonical (+Y) to actual axis
    if   axis.x >  0.5: angle = -90.0
    elif axis.x < -0.5: angle =  90.0
    elif axis.y < -0.5: angle = 180.0
    else:               angle =   0.0
    if angle:
        shape = shape.rotate(v(0, 0, 0), v(0, 0, 1), angle)
    return shape

def make_frame():
    """
    Single-piece frame: top plate + bottom plate + lever pivot posts.

    Posts sit at the blade-root / head junction (y = head_y1 in the canonical
    frame), one on each ±pa side just outside the bracket width.  This location
    clears the output wheels (head_y1 > wheel outer face), the motor disc
    (radius ≈ 34 mm from Z-axis >> disc_vr = 24 mm), and the blades.

    The trunnion socket is a horizontal through-hole along pa at pivot_z.
    """
    # post_pa_off is global (defined in DERIVED)
    z_top  =  cube_h / 2
    z_bot  = -(cube_h / 2 + plate_t)
    post_h =  cube_h + plate_t           # z_bot → z_top

    # Top plate omitted for visualisation — set SHOW_TOP = True to restore
    SHOW_TOP = False

    # Bottom plate with motor shaft clearance hole
    bot = Part.makeBox(cube_size, cube_size, plate_t,
                       v(-cube_half, -cube_half, z_bot))
    bot = bot.cut(cyl(motor_shaft_d / 2 + 1.0, plate_t + 2,
                      v(0, 0, z_bot - 1)))

    frame = bot

    if SHOW_TOP:
        top = Part.makeBox(cube_size, cube_size, plate_t,
                           v(-cube_half, -cube_half, z_top))
        top = top.cut(cyl(motor_shaft_d / 2 + 1.0, plate_t + 2,
                          v(0, 0, z_top - 1)))
        frame = frame.fuse(top)

    # Posts removed: servo body pockets replace them (see make_servo_mount).
    return frame


def make_servo_mount(axis):
    """
    SG90 body pocket, part of the frame (fixed).
    Canonical: shaft=+Y, pa=+X, servo on canonical -pa side (pa = -post_pa_off).

    Geometry
    --------
    Pocket is open from the outer face (cube-corner side) so the servo slides
    in along the pa axis.  An inner wall (sg90_wall thick) stops the body and
    has a clearance hole for the output shaft + lever boss.

    pa extent : srv_pa - sg90_l  …  srv_pa + sg90_wall
                (open outer)         (inner wall outer face)
    Y extent  : head_y1 ± (sg90_h/2 + sg90_tol + sg90_wall)
    Z extent  : ±(sg90_w/2 + sg90_tol + sg90_wall)
    """
    srv_pa = -post_pa_off                        # canonical servo-side shaft pa

    pkt_x  = sg90_l   + sg90_tol                 # pocket depth  (body + tol)
    pkt_y  = sg90_h   + 2 * sg90_tol             # pocket width  in Y
    pkt_z  = sg90_w   + 2 * sg90_tol             # pocket height in Z

    # Boss outer radius (lever socket) — inner wall hole sized to match
    boss_r = sg90_sd / 2 + 2.0                   # = 4.4 mm

    # Mount block extents
    blk_x0 = srv_pa - pkt_x                      # outer (open) face
    blk_x1 = srv_pa + sg90_wall                   # inner wall outer face
    blk_y0 = head_y1 - pkt_y / 2 - sg90_wall
    blk_y1 = head_y1 + pkt_y / 2 + sg90_wall
    blk_z0 = -(pkt_z / 2 + sg90_wall)
    blk_z1 = +(pkt_z / 2 + sg90_wall)

    blk = Part.makeBox(blk_x1 - blk_x0,
                       blk_y1 - blk_y0,
                       blk_z1 - blk_z0,
                       v(blk_x0, blk_y0, blk_z0))

    # Pocket cut (open from outer face at blk_x0)
    pocket = Part.makeBox(pkt_x, pkt_y, pkt_z,
                          v(blk_x0, head_y1 - pkt_y / 2, -pkt_z / 2))
    blk = blk.cut(pocket)

    # Shaft + boss clearance hole through inner wall
    boss_hole = cyl(boss_r + sg90_tol,
                    sg90_wall + 2,
                    v(srv_pa, head_y1, 0), v(1, 0, 0))
    blk = blk.cut(boss_hole)

    # Rotate canonical (+Y) → actual axis
    if   axis.x >  0.5: angle = -90.0
    elif axis.x < -0.5: angle =  90.0
    elif axis.y < -0.5: angle = 180.0
    else:               angle =   0.0
    if angle:
        blk = blk.rotate(v(0, 0, 0), v(0, 0, 1), angle)
    return blk


def make_lever(axis):
    """
    Direct-drive lever for SG90 servo.
    Canonical: shaft=+Y, pa=+X, servo on canonical -pa side (pa = -post_pa_off).

    The SG90 output shaft plugs into the socket boss (blind bore).
    Two corridor legs capture both head trunnion pins.
    Bridge spans from servo socket to the far corridor at z = blade_z0.
    """
    slot_r       = head_trun_r + 0.2
    block_z_half = slot_r + 3.0
    block_y0     = head_y0
    post_inner   = post_pa_off - post_d / 2 - 0.4   # ≈ 12.6 mm
    rail_w       = post_inner - bw_half              # ≈ 3.6 mm

    arm_z    = 4.0
    blade_z0 = head_z_half + 1.0

    # ── Servo socket boss ─────────────────────────────────────────────────────
    # Canonical servo side: pa = -post_pa_off = -15.5 mm
    # Shaft protrudes from body face (at srv_pa) in +pa direction toward lever.
    # Boss: solid cylinder, open blind-bore socket for SG90 shaft.
    # Boss outer radius is sized so it passes through the mount inner wall hole.
    srv_pa     = -post_pa_off
    sock_r_out = sg90_sd / 2 + 2.0          # = 4.4 mm outer radius
    sock_r_in  = sg90_sd / 2 + 0.10         # = 2.5 mm bore (tight, set-screw recommended)
    sock_depth = sg90_sl + 3.0              # = 7 mm total boss depth

    boss = cyl(sock_r_out, sock_depth,
               v(srv_pa, head_y1, 0), v(1, 0, 0))
    # Blind bore: slightly shorter than boss so the back wall stays solid
    bore_sock = cyl(sock_r_in, sg90_sl + 1.0,
                    v(srv_pa, head_y1, 0), v(1, 0, 0))
    boss = boss.cut(bore_sock)

    # ── Bridge: from servo socket to far corridor ─────────────────────────────
    # spans pa = srv_pa … +post_inner  at z = blade_z0, arm_z thick in Z and Y
    bridge = Part.makeBox(
        post_pa_off + post_inner,            # = 28.1 mm  (srv_pa=-15.5 to +post_inner=+12.6)
        arm_z,
        arm_z,
        v(srv_pa, head_y1 - arm_z / 2, blade_z0)
    )
    shape = boss.fuse(bridge)

    # ── Two corridor legs (capture both head trunnion pins) ───────────────────
    for sign in [1, -1]:
        # sign=+1 → right corridor: pa = +9 … +12.6 mm
        # sign=−1 → left  corridor: pa = −12.6 … −9 mm  (servo side)
        pa_lo = sign * bw_half    if sign > 0 else sign * post_inner
        pa_hi = sign * post_inner if sign > 0 else sign * bw_half

        block = Part.makeBox(
            rail_w,
            head_y1 + post_d - block_y0,
            blade_z0 + arm_z + block_z_half,
            v(pa_lo, block_y0, -block_z_half)
        )

        # Bore for head trunnion pin
        bore_pin = cyl(slot_r, rail_w + 2,
                       v(pa_lo - 1, head_trun_y, 0), v(1, 0, 0))
        block = block.cut(bore_pin)

        # Slot opening toward −Y (assembly access)
        slot_box = Part.makeBox(
            rail_w, head_trun_y - block_y0, 2 * slot_r,
            v(pa_lo, block_y0, -slot_r)
        )
        block = block.cut(slot_box)

        shape = shape.fuse(block)

    # Rotate canonical (+Y) to actual axis
    if   axis.x >  0.5: angle = -90.0
    elif axis.x < -0.5: angle =  90.0
    elif axis.y < -0.5: angle = 180.0
    else:               angle =   0.0
    if angle:
        shape = shape.rotate(v(0, 0, 0), v(0, 0, 1), angle)
    return shape


def make_sg90_ref(axis):
    """
    Referencia visual del SG90 en posición de montaje.

    Geometría canónica (eje real = +Y → pa = +X):
    ─ Eje del servo: a lo largo de X (pa direction).
    ─ Eje en Z justo por encima del tramo YZ de la lámina compliant.
    ─ Brazo cuelga en −Z en neutro; al rotar ±α el muñón empuja ±Y.
    ─ Muñón pasa por la ranura (elongada en Z) de la lámina YZ.

    El cuerpo del servo se extiende en +X desde la cara por donde sale el eje (x=0).
    """
    # Posición consistente con el slot de make_shaft_bracket:
    # Posición consistente con make_shaft_bracket.
    # Mismo cálculo: pin_r + 1 mm de pared bajo el pin, eje a sg90_arm_r sobre el pin.
    blade_y_ctr  = head_y1 - eng_blade_t / 2        # ≈ 32.15 mm — centro ranura en Y
    z_yz_bot_r   = -(cube_half - eng_tip_clr)        # fondo lámina YZ ≈ −54.9 mm
    pin_r_ref    = eng_slot_w / 2                    # radio del muñón
    pin_z        = z_yz_bot_r + pin_r_ref + 1.0      # centro muñón en neutro
    z_shaft      = pin_z + sg90_arm_r                 # eje encima del centro del muñón
    y_shaft      = blade_y_ctr                        # mismo Y que la ranura
    # Brazo como bloque rectangular: arm_t en X, arm_w en Y, sg90_arm_r en Z.
    # Centro del bloque en x_arm; cara −X del bloque a 0.5 mm de la cara +X de la lámina.
    # Cuerpo: cara −X a x_arm + arm_t/2 (tangente al bloque, sin solapar).
    # Eje (stub): desde body_x0 hacia −X, cortado antes de llegar a la lámina.
    arm_t   = 2.0                                    # grosor del brazo en X
    arm_w   = 4.0                                    # anchura del brazo en Y
    # Brazo centrado a 0.5 mm de la cara +X de la lámina.
    arm_x0  = eng_yz_t / 2 + 0.5                    # cara −X del brazo (0.5 mm libre de la lámina YZ)
    body_x0 = arm_x0 + arm_t                        # cara −X del cuerpo

    # ── Cuerpo del servo ──────────────────────────────────────────────────────
    body = Part.makeBox(
        sg90_l, sg90_h, sg90_w,
        v(body_x0,
          y_shaft - sg90_shaft_off,
          z_shaft - sg90_w / 2)
    )

    # ── Eje: a ras de la cara −X del brazo, longitud = grosor del brazo ───────
    shaft_stub = cyl(sg90_sd / 2, arm_t,
                     v(arm_x0, y_shaft, z_shaft),
                     v(1, 0, 0))

    # ── Brazo rectangular: arm_t en X, arm_w en Y, cubre el muñón en Z ───────
    arm_block = Part.makeBox(
        arm_t, arm_w, sg90_arm_r + pin_r_ref,
        v(arm_x0, y_shaft - arm_w / 2, pin_z - pin_r_ref)
    )

    # ── Muñón: desde 2 mm más allá de la lámina (−X) hasta la cara −X del brazo
    pin_x0  = -(eng_yz_t / 2 + 2.0)
    pin_len = arm_x0 - pin_x0
    munon   = cyl(eng_slot_w / 2 - 0.15, pin_len,
                  v(pin_x0, y_shaft, pin_z),
                  v(1, 0, 0))

    shape = body.fuse(shaft_stub).fuse(arm_block).fuse(munon)

    # Rotar al eje real
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

# Single axis for clarity — all four are identical rotated 90°
AXES = [
    ("PosY", v( 0,  1, 0)),
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
        transparency=30)
    add(doc, f"Bracket_{name}",
        make_shaft_bracket(axis),
        color=(0.30, 0.65, 0.90))

add(doc, "Frame",
    make_frame(),
    color=(0.70, 0.55, 0.85),
    transparency=40)

add(doc, "SG90_REF",
    make_sg90_ref(v(0, 1, 0)),
    color=(0.20, 0.55, 1.00),
    transparency=40)

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
