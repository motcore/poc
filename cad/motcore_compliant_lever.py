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

A_deg         = 1.5   # deg  — engagement angle (shaft tilt from horizontal);
                      #        kept small so the neutral gap does not eat the
                      #        short servo-arm stroke (see SPRING SIZING)
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
lever_len     = 25.0  # mm   — fixed shaft stub beyond wall (increased for UJ clearance)

# ── Universal joint (purchased, metal, Ø11×23 mm, 5 mm bore) ─────────────────
uj_od         = 11.0  # mm — UJ outer diameter
uj_len        = 23.0  # mm — UJ total length (both hubs + cross)
uj_cross_w    =  8.0  # mm — central cross/yoke region (no bore); each hub bore is
                      #      (uj_len − uj_cross_w)/2 deep and receives a shaft end

# ── Flange shaft hub (purchased aluminium, Ø5 bore) — wheel ↔ shaft coupling ──
# Torque path is metal-on-metal (set screws on the shaft) → no PETG creep on the
# load path; the printed wheel only sees the bolt circle (hugely over-margined).
hub_bore_d    = 5.0   # mm — hub bore (matches the metal output shaft)
hub_od        = 10.0  # mm — hub body outer diameter
hub_body_h    = 8.0   # mm — hub body height (above flange)
hub_flange_od = 22.0  # mm — flange outer diameter
hub_flange_t  = 2.5   # mm — flange thickness
hub_bolt_cd   = 16.0  # mm — flange bolt-circle diameter
hub_bolt_d    = 3.0   # mm — flange bolt hole diameter (M3)
hub_bolt_n    = 4     # —    — number of flange bolts (evenly spaced, 90°)
hub_bolt_ph   = 45.0  # deg  — bolt-circle phase offset
hub_pilot_d   = 2.5   # mm — pilot hole Ø in the wheel; the M3 flange bolt self-taps
                      #      straight into the PETG (no insert — ~8 N/bolt load)
hub_pilot_h   = 6.0   # mm — pilot hole depth
# Hub mounts from the MOTOR side: flange on the wheel's motor-side face, neck
# passing through the wheel. This keeps the wall-side shaft clear for a bearing.

# ── Output shaft bearing (in the bracket head) ───────────────────────────────
brg_id        = 5.0   # mm — bearing bore (= shaft)            ┐ MR105ZZ
brg_od        = 10.0  # mm — bearing outer diameter            │ (5×10×4)
brg_w         = 4.0   # mm — bearing width                     ┘
brg_wall      = 2.0   # mm — min PETG wall around the bearing seat (sets head height)

# ── Motor disc: one rigid part (2 plates + spool) on a Ø8 flange hub ──────────
# The disc carries the sum of the engaged axes (~4× a single wheel). Concentricity
# is critical (disc wobble detunes all four clutch gaps at once), so a machined
# Ø8 metal hub grips the shaft; the spool's close-fit bore centres the disc along
# the shaft. Hub mounts BELOW the disc so its set screws stay accessible.
disc_spool_d   = 20.0  # mm — spool joining the two plates into one rigid disc
disc_bore_d    =  8.0  # mm — central bore, close slip fit on the Ø8 shaft (centring)
mhub_bore_d    =  8.0  # mm — motor hub bore (= motor shaft)        ┐ purchased Ø8
mhub_od        = 13.0  # mm — motor hub body OD                     │ flange hub —
mhub_body_h    = 10.0  # mm — motor hub body height (below disc)    │ match params
mhub_flange_od = 25.0  # mm — motor hub flange OD                   │ to the real
mhub_flange_t  =  3.0  # mm — motor hub flange thickness            ┘ part
mhub_bolt_cd   = 18.0  # mm — motor hub bolt-circle Ø
mhub_bolt_d    =  3.0  # mm — flange bolt hole Ø (M3)
mhub_bolt_n    =  4    # —    — number of flange bolts
mhub_pilot_d   =  2.5  # mm — self-tap pilot Ø in the disc (M3 self-taps into PETG)
mhub_pilot_h   =  7.0  # mm — pilot depth

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
eng_blade_l  = 36.0  # mm — spring section length in −Z (XZ arm, free to flex);
                     #      junction block spans z −42 … −34, fully above the
                     #      servo body top (z ≈ −43.2)
eng_blade_w  = 14.0  # mm — XZ spring width in pa direction (centred at pa = 0);
                     #      narrowed from 20 with t 4.0→4.5 (w·t³ preserved →
                     #      same latch force, sigma 12.8→14.4 MPa, creep-safe)
eng_yz_w     = 10.0  # mm — YZ arm width in Y direction (structural, not spring)
eng_blade_t  =  4.5  # mm — XZ arm thickness in Y (spring direction; sized so the
                     #      sustained latch stress stays ≤ ~15 MPa — see
                     #      ENGAGEMENT SPRING SIZING in DERIVED)
eng_yz_t     =  4.0  # mm — YZ arm thickness in X (contact arm — no need to flex)
eng_overlap  =  8.0  # mm — how far YZ arm overlaps upward into the XZ spring section
eng_blk_w    =  9.0  # mm — junction (overlap) block width in pa; narrowed so the
                     #      lowered block clears the servo body (front face at x ≈ 6.5)
eng_tip_clr  =  3.0  # mm — clearance above bottom plate for YZ contact arm
eng_slot_w   =  2.5  # mm — ranura: ancho en pa (diámetro muñón + holgura)
latch_deg    = 95.0  # deg — arm angle at the slot-top hard stop, just past the
                     #       90° dead point → passive over-centre latch
                     #       (eng_slot_h is now DERIVED from this)

# ── MG90D servo (Tower Pro — digital, metal gears, ~0.1° resolution) ─────────
sg90_l        = 28.5  # mm — body length along shaft axis (pa direction)          [C=28.5]
sg90_h        = 22.6  # mm — body cross-section in Y direction (oreja a oreja)    [B=22.6]
sg90_w        = 12.0  # mm — body cross-section in Z direction                     [D=12.0]
sg90_sd       =  4.8  # mm — output shaft outer diameter (with spline)
sg90_sl       =  4.0  # mm — shaft protrusion beyond body face              [A-C=32.5-28.5]
sg90_tol      =  0.3  # mm — mounting clearance per side
sg90_wall     =  2.0  # mm — bracket wall thickness (legacy, kept for reference)
sg90_shaft_off =  6.0  # mm — shaft centre from front body face (by0), estimated
sg90_arm_r     =  4.0  # mm — servo arm radius (pin at arm tip); short arm →
                       #      over-centre latch within a short slot and low
                       #      sustained spring stress (creep-safe)
sg90_arm_t     =  4.0  # mm — arm thickness in X (was 2.0)
sg90_ear_y     =  4.45 # mm — ear tab protrusion beyond body face (±Y)  [(E-B)/2=(31.5-22.6)/2]
sg90_ear_t     =  2.0  # mm — ear tab thickness in X (bolt direction)
sg90_ear_x     =  8.7  # mm — ear centre X from body_x0 (along shaft axis)  [C-F=28.5-19.8]
sg90_m2_r      =  1.1  # mm — M2 hole radius in ear (clearance for M2 bolt)

# ── Modular foot + side bars (4× M3, no tools needed) ────────────────────────
# Foot      : plate in wall pocket, shaft clearance hole + 4 screw holes.
# Side bars : 2 vertical bars on wall inner face, one at +pa and one at −pa.
#             Each bar spans Z = [−foot_screw_z−boss_pad … +foot_screw_z+boss_pad]
#             and carries 2 nut traps (at Z = ±foot_screw_z).
#             Central gap between bars = 2×(foot_screw_pa−boss_pad) ≈ 28.4 mm,
#             wide enough for the bracket head (25 mm) to pass during assembly.
m3_nut_af     = 5.7   # mm — M3 hex nut AF with print clearance (5.5 mm + 0.2)
m3_nut_thick  = 2.6   # mm — nut trap depth (2.4 mm nut + 0.2 mm clearance)
m3_screw_r    = 1.6   # mm — M3 clearance hole radius (Ø3.2 mm)
foot_screw_pa = 19.0  # mm — screw centre offset in pa direction (±pa)
                      #      wide enough so side bars clear the bracket head (±12.5 mm) during assembly
foot_screw_z  = 9.0   # mm — screw centre Z offset (±Z, clear of blades at Z≈0 ±0.75)
foot_fit      = 0.3   # mm — pocket fit clearance per side
flange_depth  = 5.0   # mm — flange Y depth (protrudes inward from wall inner face)

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
blade_gap  = uj_od + 2.0                   # gap between blades in pa — clears UJ body (Ø11 + 1 mm each side)
blade_w_g  = 6.0                           # blade width in pa
bw_half    = blade_gap / 2 + blade_w_g     # bracket half-width in pa = 12.5 mm
# Single compliant neck centred in the free span (kept for reference; shaft now uses UJ)
neck_start = head_y1 + (cube_half - head_y1 - neck_len) / 2
neck_end   = neck_start + neck_len

# ── Universal joint position ───────────────────────────────────────────────
# Outer face of UJ flush with wall INNER face → UJ entirely inside the cube.
# Bearing sits in the wall (wall_thick = 4 mm); shaft passes through bearing.
# Cross (actual pivot) at cube_half - uj_len/2 ≈ 46.8 mm.
# Ideal pivot (uj_dist) was at 54.3 mm → displacement Δ = 7.5 mm inward.
# Contact-Z error: Δ × sin(2°) = 7.5 × 0.035 = 0.26 mm — still negligible.
uj_outer_y = cube_half                       # outer face flush with wall inner face
uj_inner_y = uj_outer_y - uj_len            # inner face ≈ 35.3 mm (inside cube)
uj_center  = uj_outer_y - uj_len / 2        # actual pivot ≈ 46.8 mm from cube centre

# ── Bracket head half-height in Z (shared with make_lever) ───────────────
head_z_half = max(shaft_dia / 2 + 3.5, brg_od / 2 + brg_wall)  # sized for bearing seat

# ── Lever pivot post geometry (shared with make_frame and make_lever) ─────
post_pa_off  = bw_half + post_side_gap + post_w / 2  # post centre offset in pa
head_trun_y  = head_y0 + 2.0                          # head trunnion Y (2 mm from inner face)
head_pin_pa  = bw_half + head_trun_len / 2            # head pin midpoint in pa

# ── Slot height (derived): the pin rises r·(1−cos α) during the arm sweep;
#    the slot top is a hard stop at latch_deg, just past the 90° dead point →
#    passive over-centre latch: the spring presses the pin against the stop,
#    the clutch self-holds at max force and the servo is unloaded (works even
#    unpowered).  Stadium-slot vertical play = eng_slot_h − eng_slot_w.
eng_slot_h = eng_slot_w + sg90_arm_r * (1 - math.cos(math.radians(latch_deg)))

# ── ENGAGEMENT SPRING SIZING (XZ flexure + over-centre latch) ────────────────
# The servo pin (arm radius sg90_arm_r) pushes the YZ arm with force F (±Y);
# pin Y-travel x = r·sin α.  Moment balance about the UJ tilt axis:
#     N = F · (pin_arm / contact_arm)        T_slip = mu · N · R_out
# Max bending stress at the spring root (head bottom face):
#     sigma = 6 · F · a / (w · t²)           a = root → pin distance
# The latch stress is sustained → kept ≤ ~15 MPa so PETG stress relaxation
# stays negligible.  Servo demand peaks mid-sweep and is transient only.
E_PETG      = 2000.0   # MPa  — printed PETG flexural modulus
MU_ORING    = 0.6      # —    — silicone O-ring on PETG disc, conservative
SRV_T_CONT  = 125.0    # N·mm — MG90D continuous torque (~45 % of 6 V stall)

spr_pin_z    = -(cube_half - eng_tip_clr) + eng_slot_w / 2 + 1.0  # pin centre, neutral
spr_root_z   = -head_z_half                 # spring fixed end (head bottom face)
spr_a        = spr_root_z - spr_pin_z       # root → pin lever arm (≈ 47 mm)
spr_L        = eng_blade_l - eng_overlap    # free flexible length (overlap block is rigid)
spr_pin_arm  = -spr_pin_z                   # pin force arm about UJ tilt axis
spr_cont_arm = uj_center - wc_dist          # normal force arm about UJ tilt axis
spr_leverage = spr_pin_arm / spr_cont_arm

spr_I     = eng_blade_w * eng_blade_t ** 3 / 12
# Pin compliance: cantilever (free length spr_L) + rigid extension to the pin
_th_unit  = (spr_a * spr_L - spr_L ** 2 / 2) / (E_PETG * spr_I)
_dl_unit  = (spr_a * spr_L ** 2 / 2 - spr_L ** 3 / 6) / (E_PETG * spr_I)
spr_k     = 1.0 / (_dl_unit + _th_unit * (spr_a - spr_L))  # N/mm at the pin

spr_free      = (contactZ - R_out) / spr_cont_arm * spr_pin_arm  # pin free travel
spr_a_contact = math.degrees(math.asin(min(spr_free / sg90_arm_r, 1.0)))
spr_F_latch   = spr_k * (sg90_arm_r * math.sin(math.radians(latch_deg)) - spr_free)
spr_N_latch   = spr_F_latch * spr_leverage
spr_T_latch   = MU_ORING * spr_N_latch * R_out
spr_sig_latch = 6 * spr_F_latch * spr_a / (eng_blade_w * eng_blade_t ** 2)
# Peak transient servo torque on the way to the latch (proportional range)
spr_peak_dem  = max(
    spr_k * max(sg90_arm_r * math.sin(math.radians(_d)) - spr_free, 0.0)
          * sg90_arm_r * math.cos(math.radians(_d))
    for _d in range(0, 91))
# Latch force lost per 0.1 mm of extra neutral gap (O-ring wear, print tolerance)
spr_dF_wear   = spr_k * 0.1 * spr_pin_arm / spr_cont_arm

# ── Modular foot + side-bar dimensions ────────────────────────────────────────
# Screws at (±foot_screw_pa, ±foot_screw_z) — clear of all blades (Z≈0 ±0.75 mm).
# Side bars: inner edge at (foot_screw_pa − boss_pad) from centre, outer edge at
#            (foot_screw_pa + bar_outer_w) — extend toward the wall edge for grip.
#            Central gap = 2×(foot_screw_pa − boss_pad) ≈ 18.4 mm > 18 mm ✓
_nut_circ_r   = m3_nut_af / math.sqrt(3)             # hex circumradius ≈ 3.29 mm
boss_pad      = _nut_circ_r + 1.5                     # ≈ 4.8 mm — material around nut
bar_ext       = 5.0                                    # mm — extra reach in pa (outward) and Z (both ends)
foot_half_w   = foot_screw_pa + m3_screw_r + 1.5     # = 22.1 mm (wider to clear blades)
foot_z_half   = foot_screw_z  + m3_screw_r + 1.5     # = 12.1 mm


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


def perp_basis(axis):
    """Two unit vectors spanning the plane perpendicular to a horizontal axis:
    u = vertical (Z), w = the other horizontal direction (axis × Z)."""
    return v(0, 0, 1), v(axis.y, -axis.x, 0)


def bolt_circle_points(centre, axis, radius, n, phase_deg):
    """n points on a circle of given radius, centred at `centre`, lying in the
    plane perpendicular to `axis` (used for flange bolt patterns)."""
    u, w = perp_basis(axis)
    pts = []
    for k in range(n):
        a = math.radians(360.0 / n * k + phase_deg)
        c, s = math.cos(a), math.sin(a)
        pts.append(v(centre.x + radius * (c * w.x + s * u.x),
                     centre.y + radius * (c * w.y + s * u.y),
                     centre.z + radius * (c * w.z + s * u.z)))
    return pts


def make_motor_disc():
    """One rigid part: top plate + central spool + bottom plate.
    Contact faces at z = ±contactZ; the spool joins them so they stay parallel.
    Central bore is a close slip fit on the Ø8 shaft (centres the disc — it rotates
    with the shaft, no relative motion). Torque comes from the motor hub below via
    a bolt circle into self-tap pilots in the bottom plate.
    """
    top   = cyl(disc_vr, WT, v(0, 0, contactZ))                 # face at +contactZ
    bot   = cyl(disc_vr, WT, v(0, 0, -contactZ - WT))           # face at −contactZ
    spool = cyl(disc_spool_d / 2, 2 * contactZ, v(0, 0, -contactZ))
    disc  = top.fuse(spool).fuse(bot)

    # Central centring bore on the shaft, through the whole stack
    disc = disc.cut(cyl(disc_bore_d / 2 + 0.05, 2 * contactZ + 2 * WT + 2,
                        v(0, 0, -contactZ - WT - 1)))

    # Self-tap pilots on the bottom plate outer face for the hub flange bolts
    z_face = -contactZ - WT
    rbc = mhub_bolt_cd / 2
    for k in range(mhub_bolt_n):
        a = math.radians(360.0 / mhub_bolt_n * k + 45.0)
        disc = disc.cut(cyl(mhub_pilot_d / 2, mhub_pilot_h + 0.5,
                            v(rbc * math.cos(a), rbc * math.sin(a), z_face - 0.5),
                            v(0, 0, 1)))
    return disc


def make_motor_hub():
    """Purchased Ø8 flange hub for the motor disc (reference part).
    Mounts BELOW the disc: flange bolted up to the bottom plate, body + 2 set
    screws (at 90°) below it where they stay accessible and grip the Ø8 shaft.
    """
    z_top    = -contactZ - WT                       # flange top = disc bottom face
    z_fl     = z_top - mhub_flange_t                # flange bottom
    z_body   = z_fl - mhub_body_h                   # body bottom

    flange = cyl(mhub_flange_od / 2, mhub_flange_t, v(0, 0, z_fl))
    body   = cyl(mhub_od / 2, mhub_body_h, v(0, 0, z_body))
    hub    = flange.fuse(body)

    # Bore for the shaft
    hub = hub.cut(cyl(mhub_bore_d / 2 + 0.05, mhub_flange_t + mhub_body_h + 2,
                      v(0, 0, z_body - 1)))

    # Flange bolt clearance holes
    rbc = mhub_bolt_cd / 2
    for k in range(mhub_bolt_n):
        a = math.radians(360.0 / mhub_bolt_n * k + 45.0)
        hub = hub.cut(cyl(mhub_bolt_d / 2, mhub_flange_t + 2,
                          v(rbc * math.cos(a), rbc * math.sin(a), z_fl - 1),
                          v(0, 0, 1)))

    # Two radial M3 set-screw holes at 90° through the body
    z_mid = z_body + mhub_body_h / 2
    for d in (v(1, 0, 0), v(0, 1, 0)):
        hub = hub.cut(cyl(1.5, mhub_od / 2 + 2,
                          v(d.x * (mhub_od / 2 + 1), d.y * (mhub_od / 2 + 1), z_mid),
                          v(-d.x, -d.y, 0)))
    return hub


def make_output_wheel(axis):
    """
    Disc perpendicular to axis, centred at wc_dist from origin.
    Structural radius Rw, thickness WT. O-ring groove on rim.
    Motor-side face (−axis end) carries the flange-hub mounting: a flat seat +
    hub_bolt_n pilot holes on a hub_bolt_cd circle (M3 bolts self-tap into the
    PETG). The central bore is Ø(hub_od) so the hub neck passes through; torque
    comes through the bolts.
    Mounting the hub from the motor side keeps the wall-side shaft clear for the
    head bearing.
    In neutral position (shaft horizontal, no tilt).
    """
    wc   = av(axis, wc_dist)            # wheel centre
    base = wc - av(axis, WT / 2)        # disc start along axis

    disc  = cyl(Rw, WT, base, axis)
    # Through bore for the hub neck (clearance over Ø hub_od)
    bore  = cyl(hub_od / 2 + 0.1, WT + 2, base - axis, axis)

    # O-ring groove: width = 1.2·dw, depth = 0.6·dw, centred on wheel face
    gw    = dw * 1.2
    gd    = dw * 0.6
    gb    = wc - av(axis, gw / 2)       # groove start
    g_out = cyl(Rw + 0.1, gw, gb, axis)
    g_in  = cyl(Rw - gd,  gw + 2, gb - axis, axis)
    groove = g_out.cut(g_in)

    wheel = disc.cut(bore).cut(groove)

    # Self-tap pilot holes, drilled inward (+axis) from the MOTOR-side face
    motor_face = av(axis, wc_dist - WT / 2)
    for p in bolt_circle_points(motor_face, axis, hub_bolt_cd / 2,
                                hub_bolt_n, hub_bolt_ph):
        mouth = v(p.x - axis.x * 0.5, p.y - axis.y * 0.5, p.z - axis.z * 0.5)
        wheel = wheel.cut(cyl(hub_pilot_d / 2, hub_pilot_h + 0.5, mouth, axis))

    return wheel


def make_flange_hub(axis):
    """
    Purchased aluminium flange shaft hub (reference part), mounted from the MOTOR
    side. The flange seats on the wheel's motor-side face; the neck passes through
    the wheel toward the wall. Two M3 set screws at 90° grip the Ø5 shaft; the
    hub_bolt_n flange bolts drive the printed wheel.
    """
    neg = v(-axis.x, -axis.y, -axis.z)
    motor_face  = av(axis, wc_dist - WT / 2)                    # wheel motor-side face
    flange_base = av(axis, wc_dist - WT / 2 - hub_flange_t)     # flange outer face

    flange = cyl(hub_flange_od / 2, hub_flange_t, flange_base, axis)
    # Neck runs from the flange through the wheel (toward the wall)
    neck   = cyl(hub_od / 2, hub_flange_t + hub_body_h, flange_base, axis)
    hub    = flange.fuse(neck)

    # Central bore for the shaft
    hub = hub.cut(cyl(hub_bore_d / 2 + 0.05, hub_flange_t + hub_body_h + 2,
                      v(flange_base.x - axis.x, flange_base.y - axis.y,
                        flange_base.z - axis.z), axis))

    # Flange through-holes on the bolt circle (from the flange outer face inward)
    for p in bolt_circle_points(flange_base, axis, hub_bolt_cd / 2,
                                hub_bolt_n, hub_bolt_ph):
        start = v(p.x - axis.x, p.y - axis.y, p.z - axis.z)
        hub = hub.cut(cyl(hub_bolt_d / 2, hub_flange_t + 2, start, axis))

    # Two radial M3 set-screw holes at 90°, in the neck where it passes the shaft
    neck_mid = av(axis, wc_dist - WT / 2 + hub_body_h / 2)
    u, w = perp_basis(axis)
    for d in (w, u):
        start = v(neck_mid.x + d.x * (hub_od / 2 + 1),
                  neck_mid.y + d.y * (hub_od / 2 + 1),
                  neck_mid.z + d.z * (hub_od / 2 + 1))
        hub = hub.cut(cyl(1.5, hub_od / 2 + 2, start, v(-d.x, -d.y, -d.z)))
    return hub


def make_oring(axis):
    Ror = R_out - 0.5 * dw   # O-ring centre radius (0.1·dw inside rim, protrudes 0.4·dw)
    wc  = av(axis, wc_dist)
    return Part.makeTorus(Ror, dw / 2, wc, axis)


def make_output_shaft(axis):
    """
    Output shaft — two solid Ø5 mm segments, split at the UJ.

    Inner shaft : wheel end → UJ inner face (uj_inner_y ≈ 35.3 mm, inside cube).
    Outer shaft : UJ outer face (uj_outer_y ≈ 58.3 mm, wall inner face) → lever tip.
    No compliant neck — the UJ (make_uj_body) provides the tilt pivot.
    """
    neg = v(-axis.x, -axis.y, -axis.z)

    # Both segments plug INTO the UJ hub bores, stopping at the cross region —
    # faithful to the purchased UJ (5 mm bore each side, set-screwed).
    inner_into = uj_center - uj_cross_w / 2     # inner shaft end inside the UJ
    outer_into = uj_center + uj_cross_w / 2     # outer shaft end inside the UJ

    # Inner shaft: motor-side end (through wheel/hub + head bearing) → into the UJ
    inner_far = wc_dist - WT / 2 - 2.0
    inner = cyl(shaft_dia / 2, inner_into - inner_far, av(axis, inner_into), neg)

    # Outer shaft: from inside the UJ → through the foot bearing → lever tip
    outer_far = cube_half + wall_thick + lever_len
    outer = cyl(shaft_dia / 2, outer_far - outer_into, av(axis, outer_into), axis)

    return inner.fuse(outer)


def make_uj_body(axis):
    """Universal joint — reference Ø{uj_od} × {uj_len} mm, hubs bored Ø5 each side
    so the shaft ends plug in; the central uj_cross_w region (the cross/yoke) is
    left solid. Represents the purchased metal UJ.
    """
    neg  = v(-axis.x, -axis.y, -axis.z)
    body = cyl(uj_od / 2, uj_len, av(axis, uj_inner_y), axis)
    bore_r = shaft_dia / 2 + 0.2
    depth  = (uj_len - uj_cross_w) / 2
    body = body.cut(cyl(bore_r, depth + 0.01, av(axis, uj_inner_y - 0.01), axis))
    body = body.cut(cyl(bore_r, depth + 0.01, av(axis, uj_outer_y + 0.01), neg))
    return body


def make_cube_wall(axis):
    """Wall with integrated compliant flexure bracket (one-piece print).
    Includes a through-bore for the output shaft lever arm.
    """
    shaft_hole_r = uj_od / 2 + 0.5       # clearance bore for UJ body (Ø11 + 1 mm)

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

    # ── Pocket for foot (portrait cutout through wall) ────────────────────────
    # Built in canonical frame, rotated to actual axis.
    _pkt_w = 2 * foot_half_w + 2 * foot_fit    # 14.2 + 0.6 mm
    _pkt_h = 2 * foot_z_half + 2 * foot_fit    # 24.2 + 0.6 mm
    _pocket = Part.makeBox(_pkt_w, wall_thick + 2, _pkt_h,
                           v(-_pkt_w / 2, cube_half - 1, -_pkt_h / 2))
    _angle = 0.0
    if   axis.x >  0.5: _angle = -90.0
    elif axis.x < -0.5: _angle =  90.0
    elif axis.y < -0.5: _angle = 180.0
    if _angle:
        _pocket = _pocket.rotate(v(0, 0, 0), v(0, 0, 1), _angle)
    wall = wall.cut(_pocket)

    # ── Two vertical side bars ("solapas laterales") ───────────────────────────
    # One bar at +pa, one at −pa.  Each bar:
    #   inner edge : foot_screw_pa − boss_pad          ≈  9.2 mm from centre
    #   outer edge : foot_screw_pa + boss_pad + bar_ext ≈ 23.8 mm from centre
    #   Z half-height: foot_screw_z + boss_pad + bar_ext ≈ 18.8 mm
    #   Y depth    : flange_depth = 5 mm  (flush against wall inner face)
    # Central gap between bars = 2×(foot_screw_pa − boss_pad) ≈ 18.4 mm > 18 mm ✓
    # 2 nut traps per bar at Z = ±foot_screw_z; screws from outside → foot → nut.
    _nut_cr   = m3_nut_af / math.sqrt(3)
    _fl_y0    = cube_half - flange_depth                 # bar inner face (nut trap opening)
    _bar_z0   = foot_screw_z + boss_pad + bar_ext        # bar half-height in Z
    for _spa in [foot_screw_pa, -foot_screw_pa]:
        _sn      = 1 if _spa > 0 else -1
        _x_inner = _spa - _sn * boss_pad                 # toward centre      ≈ ±9.2 mm
        _x_outer = _spa + _sn * (boss_pad + bar_ext)     # toward wall edge   ≈ ±23.8 mm
        _bar = Part.makeBox(
            abs(_x_outer - _x_inner), flange_depth, 2 * _bar_z0,
            v(min(_x_inner, _x_outer), _fl_y0, -_bar_z0)
        )
        for _sz in [foot_screw_z, -foot_screw_z]:
            # Hex nut trap from inner face (y = _fl_y0), extruded in +Y
            _hv = [v(_spa + _nut_cr * math.cos(i * math.pi / 3),
                     _fl_y0,
                     _sz  + _nut_cr * math.sin(i * math.pi / 3)) for i in range(6)]
            _hv.append(_hv[0])
            _trap = Part.Face(Part.Wire(
                [Part.makeLine(_hv[j], _hv[j + 1]) for j in range(6)]
            )).extrude(v(0, m3_nut_thick, 0))
            _bar = _bar.cut(_trap)
            # Screw clearance hole through full bar depth (for shank beyond nut)
            _bar = _bar.cut(cyl(m3_screw_r, flange_depth + 2,
                                v(_spa, _fl_y0 - 1, _sz), v(0, 1, 0)))
        # Rotate canonical → actual axis and fuse to wall
        if _angle:
            _bar = _bar.rotate(v(0, 0, 0), v(0, 0, 1), _angle)
        wall = wall.fuse(_bar)

    return wall  # bracket is a separate part — shown as Bracket_{name}


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
    # blade_gap, blade_w_g, bw_half — use globals from DERIVED (clears UJ body Ø11 mm)
    # head_z_half is global (defined in DERIVED)

    # head_y0, head_y1 from global DERIVED (shared with make_output_shaft)
    blade_len = cube_half - head_y1

    # Head block with a ball-bearing seat.
    # Bearing slides in from the wall side (head_y1) and seats against a shoulder;
    # the shaft passes through the Ø(shaft+clr) front lip toward the wheel.
    head = Part.makeBox(2 * bw_half, head_depth, 2 * head_z_half,
                        v(-bw_half, head_y0, -head_z_half))
    # shaft clearance through the whole head
    head = head.cut(cyl(shaft_dia / 2 + 0.4, head_depth + 2,
                        v(0, head_y0 - 1, 0), v(0, 1, 0)))
    # bearing pocket from the wall-side face inward (press/slip fit)
    head = head.cut(cyl(brg_od / 2 + 0.05, brg_w + 0.3,
                        v(0, head_y1 - brg_w, 0), v(0, 1, 0)))

    # Right blade (+pa side): lies flat at z=0, thin in Z, wide in pa
    blade_r = Part.makeBox(blade_w_g, blade_len, blade_t,
                           v( blade_gap / 2,                head_y1, -blade_t / 2))
    # Left blade (-pa side)
    blade_l = Part.makeBox(blade_w_g, blade_len, blade_t,
                           v(-(blade_gap / 2 + blade_w_g),  head_y1, -blade_t / 2))

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

    # XZ arm — wide spring section spans head bottom → junction block top only.
    # Below z_yz_top just the narrow block + YZ arm continue, so the wide part
    # stays clear of the servo body (front face x ≈ 6.5, top z ≈ −43).
    eng_xz = Part.makeBox(
        eng_blade_w, eng_blade_t, eng_blade_l - eng_overlap,
        v(-eng_blade_w / 2, head_y1 - eng_blade_t, z_yz_top)
    )
    # YZ arm — starts eng_overlap mm above spring bottom, runs to near base plate
    # Centred on the XZ arm mid-plane: y_centre = head_y1 - eng_blade_t/2
    y_ctr  = head_y1 - eng_blade_t / 2                 # Y centre of cross section
    yz_y0  = y_ctr - eng_yz_w / 2
    r_cap  = eng_yz_w / 2                              # radio = media anchura en Y
    # pin_z se calcula justo debajo — cap_z coincide con el centro del fondo de la ranura
    # → punta exterior concéntrica con la semicircunferencia interior del slot
    pin_r  = eng_slot_w / 2
    pin_z  = z_yz_bot + pin_r + 1.0
    cap_z  = pin_z                                     # mismo centro que el fondo de la ranura

    # Box arranca en cap_z; el cilindro cuelga por debajo añadiendo material
    eng_yz = Part.makeBox(
        eng_yz_t, eng_yz_w, z_yz_top - cap_z,
        v(-eng_yz_t / 2, yz_y0, cap_z)
    )
    # Cilindro a lo largo de X: punta semicircular en YZ, concéntrica con el slot
    eng_yz = eng_yz.fuse(cyl(r_cap, eng_yz_t, v(-eng_yz_t / 2, y_ctr, cap_z), v(1, 0, 0)))

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
    slot_depth   = eng_blk_w + 3                       # cut spans the full block width
    slot_x0      = -(eng_blk_w / 2 + 1.5)              # clears the pin shank at −X too
    # Ranura en forma de estadio (oblong): dos semicilindros + rectángulo central.
    # Altura total = eng_slot_h, sin cambiar la posición del slot.
    slot_top_ctr = slot_z0 + eng_slot_h - pin_r      # centro semicilindro superior
    slot_rect = Part.makeBox(
        slot_depth, eng_slot_w, eng_slot_h - 2 * pin_r,
        v(slot_x0, y_ctr - pin_r, pin_z)
    )
    slot_round_bot = cyl(pin_r, slot_depth, v(slot_x0, y_ctr, pin_z),        v(1, 0, 0))
    slot_round_top = cyl(pin_r, slot_depth, v(slot_x0, y_ctr, slot_top_ctr), v(1, 0, 0))
    slot_cut = slot_rect.fuse(slot_round_bot).fuse(slot_round_top)

    # ── Overlap zone: solid block bridging the XZ spring and YZ arm ──────────
    # Narrow (eng_blk_w) so the lowered block clears the servo body in +X.
    overlap_block = Part.makeBox(
        eng_blk_w, eng_yz_w, eng_overlap,
        v(-eng_blk_w / 2, yz_y0, z_spring_bot)
    )

    # Fuse first, then cut the slot through everything: with the junction block
    # lowered the slot top reaches ~0.4 mm into the block bottom — the latch
    # stop must be the slot end, not the block face.
    eng_blade = eng_xz.fuse(eng_yz).fuse(overlap_block).cut(slot_cut)

    # ── Modular attachment foot (plate in wall pocket) ───────────────────────────
    # At y = [cube_half, cube_half + wall_thick] — slides into pocket in the wall.
    # 2×foot_half_w (34.2 mm) wide × 2×foot_z_half (24.2 mm) tall.
    # Shaft clearance hole at centre + 4× M3 screw clearance holes at (±14 mm, ±9 mm).
    # No nut traps here — those are in the four boss blocks on the wall inner face.
    foot = Part.makeBox(
        2 * foot_half_w, wall_thick, 2 * foot_z_half,
        v(-foot_half_w, cube_half, -foot_z_half)
    )
    # Output-shaft bearing seat ("pie de la pared"): MR105ZZ pressed into the
    # foot, flush with the wall faces. Foot thickness (wall_thick = 4 mm) = the
    # bearing width, so it sits fully inside the foot — no protruding boss.
    foot = foot.cut(cyl(brg_od / 2 + 0.05, wall_thick + 2,
                        v(0, cube_half - 1, 0), v(0, 1, 0)))
    # 4× M3 screw clearance holes at corner positions (±pa, ±Z)
    for _spa in [foot_screw_pa, -foot_screw_pa]:
        for _spz in [foot_screw_z, -foot_screw_z]:
            foot = foot.cut(cyl(m3_screw_r, wall_thick + 2,
                                v(_spa, cube_half - 1, _spz), v(0, 1, 0)))

    shape = head.fuse(blade_r).fuse(blade_l).fuse(eng_blade).fuse(foot)

    # Rotate canonical (+Y) to actual axis
    if   axis.x >  0.5: angle = -90.0
    elif axis.x < -0.5: angle =  90.0
    elif axis.y < -0.5: angle = 180.0
    else:               angle =   0.0
    if angle:
        shape = shape.rotate(v(0, 0, 0), v(0, 0, 1), angle)
    return shape

def make_frame(axes=None):
    """
    Single-piece frame: bottom plate + servo mount wings for each axis.

    axes : list of (name, axis_vector) — mismo formato que AXES en BUILD DOCUMENT.
           Si se pasa, los mounts de cada eje se fusionan aquí (una sola pieza).
    """
    z_bot  = -(cube_h / 2 + plate_t)

    # Top plate omitted for visualisation — set SHOW_TOP = True to restore
    SHOW_TOP = False

    # Bottom plate with motor shaft clearance hole
    bot = Part.makeBox(cube_size, cube_size, plate_t,
                       v(-cube_half, -cube_half, z_bot))
    bot = bot.cut(cyl(motor_shaft_d / 2 + 1.0, plate_t + 2,
                      v(0, 0, z_bot - 1)))

    frame = bot

    if SHOW_TOP:
        z_top = cube_h / 2
        top = Part.makeBox(cube_size, cube_size, plate_t,
                           v(-cube_half, -cube_half, z_top))
        top = top.cut(cyl(motor_shaft_d / 2 + 1.0, plate_t + 2,
                          v(0, 0, z_top - 1)))
        frame = frame.fuse(top)

    # Servo mount wings — misma pieza que la placa inferior
    if axes:
        for _, axis in axes:
            frame = frame.fuse(make_servo_mount(axis))

    return frame


def make_servo_mount(axis):
    """
    MG90D ear-mount bracket (structural piece, fixed to frame).

    Geometry (canonical frame: shaft=+Y, pa=+X)
    ─────────────────────────────────────────────
    Servo lies with shaft along X (pa direction), body at +X from blade.
    Ears are tabs extending ±Y beyond the body at X = body_x0 + sg90_ear_x,
    with M2 through-holes in X direction at Z = z_shaft.

    The bracket has two wings (front −Y, back +Y) that capture the ear tabs.
    Each wing: X wraps the ear (ear_xc ± sg90_ear_t/2 ± brk_wall),
               Y covers ear tab + brk_wall outward,
               Z = full servo body Z extent.
    A base plate (below servo body in Z) connects both wings for rigidity.
    M2 through-holes in X let the servo slide in from +X; bolts lock it in place.
    """
    # ── Reproduce ear positions from make_sg90_ref ────────────────────────────
    blade_y_ctr = head_y1 - eng_blade_t / 2
    z_yz_bot_r  = -(cube_half - eng_tip_clr)
    pin_r_ref   = eng_slot_w / 2
    pin_z       = z_yz_bot_r + pin_r_ref + 1.0
    z_shaft     = pin_z + sg90_arm_r
    y_shaft     = blade_y_ctr
    arm_t       = sg90_arm_t
    arm_x0      = eng_yz_t / 2 + 0.5
    body_x0     = arm_x0 + arm_t

    by0 = y_shaft - sg90_shaft_off      # body front face Y
    by1 = by0 + sg90_h                  # body back face Y
    bz0 = z_shaft - sg90_w / 2         # body bottom Z
    bz1 = bz0 + sg90_w                 # body top Z

    ear_xc  = body_x0 + sg90_ear_x     # ear centre X
    ear_x0e = ear_xc - sg90_ear_t / 2  # ear inner X face
    ear_x1e = ear_xc + sg90_ear_t / 2  # ear outer X face

    brk_wall = 2.5                           # bracket wall thickness (X and outer-Y)
    z_base   = -(cube_h / 2 + plate_t)      # fondo de la placa inferior del frame

    # ── X span: wraps the ear ─────────────────────────────────────────────────
    bx0    = ear_x0e - brk_wall              # −X face of bracket wing
    bx1    = ear_x1e + brk_wall              # +X face of bracket wing
    bx_len = bx1 - bx0

    # ── Alas verticales: desde el fondo de la placa hasta la cima del cuerpo ──
    # Se fusionan con la placa inferior de make_frame (geometría coincidente).
    wing_f = Part.makeBox(bx_len, sg90_ear_y + brk_wall, bz1 - z_base,
                          v(bx0, by0 - sg90_ear_y - brk_wall, z_base))

    # Ala trasera: llega hasta la cara interior de la pared (cube_half)
    # → columna de unión entre el suelo del frame y la pared
    wing_b = Part.makeBox(bx_len, cube_half - by1, bz1 - z_base,
                          v(bx0, by1, z_base))

    # ── Ranuras para orejas — abiertas por +X (el servo entra deslizando) ─────
    # Longitud del corte: desde cara interior de la oreja hasta cara +X del ala
    slot_x0  = ear_x0e - sg90_tol          # un poco antes del borde interno de la oreja
    slot_len = bx1 + 1 - slot_x0           # llega hasta +X y sale 1 mm (corte limpio)
    slot_z0  = bz0 - sg90_tol
    slot_z1  = bz1 + sg90_tol

    # Hueco oreja frontal (en wing_f, abierto en su cara +Y = by0)
    slot_f = Part.makeBox(slot_len, sg90_ear_y + sg90_tol, slot_z1 - slot_z0,
                          v(slot_x0, by0 - sg90_ear_y - sg90_tol, slot_z0))
    wing_f = wing_f.cut(slot_f)

    # Hueco oreja trasera (en wing_b, abierto en su cara −Y = by1)
    slot_b = Part.makeBox(slot_len, sg90_ear_y + sg90_tol, slot_z1 - slot_z0,
                          v(slot_x0, by1, slot_z0))
    wing_b = wing_b.cut(slot_b)

    blk = wing_f.fuse(wing_b)

    # ── M2 through-holes in X direction (at Z = z_shaft, Y = ear centre) ─────
    m2_f = cyl(sg90_m2_r + sg90_tol, bx_len + 2,
               v(bx0 - 1, by0 - sg90_ear_y / 2, z_shaft), v(1, 0, 0))
    m2_b = cyl(sg90_m2_r + sg90_tol, bx_len + 2,
               v(bx0 - 1, by1 + sg90_ear_y / 2, z_shaft), v(1, 0, 0))
    blk = blk.cut(m2_f).cut(m2_b)

    # ── Rotate canonical (+Y) → actual axis ──────────────────────────────────
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
    arm_t   = sg90_arm_t                               # grosor del brazo en X
    arm_w   = 4.0                                    # anchura del brazo en Y
    # Brazo centrado a 0.5 mm de la cara +X de la lámina.
    arm_x0  = eng_yz_t / 2 + 0.5                    # cara −X del brazo (0.5 mm libre de la lámina YZ)
    body_x0 = arm_x0 + arm_t                        # cara −X del cuerpo

    # Derived body faces
    by0 = y_shaft - sg90_shaft_off      # body front face Y
    by1 = by0 + sg90_h                  # body back face Y
    bz0 = z_shaft - sg90_w / 2         # body bottom Z

    # ── Cuerpo del servo ──────────────────────────────────────────────────────
    body = Part.makeBox(sg90_l, sg90_h, sg90_w, v(body_x0, by0, bz0))

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

    # ── Orejas de montaje: tabs en ±Y, agujero M2 en dirección X ─────────────
    # Centradas en X a sg90_ear_x desde la cara del shaft (body_x0)
    ear_xc  = body_x0 + sg90_ear_x          # centro X de la oreja
    ear_x0e = ear_xc - sg90_ear_t / 2       # cara −X de la oreja
    ear_x1e = ear_xc + sg90_ear_t / 2       # cara +X de la oreja
    ear_f   = Part.makeBox(sg90_ear_t, sg90_ear_y, sg90_w,
                           v(ear_x0e, by0 - sg90_ear_y, bz0))  # oreja frontal (−Y)
    ear_b   = Part.makeBox(sg90_ear_t, sg90_ear_y, sg90_w,
                           v(ear_x0e, by1,               bz0))  # oreja trasera (+Y)
    # Agujeros M2 en X a Z = z_shaft (centro del cuerpo en Z)
    m2_hole = cyl(sg90_m2_r, sg90_ear_t + 2, v(ear_x0e - 1, by0 - sg90_ear_y / 2, z_shaft), v(1, 0, 0))
    ear_f   = ear_f.cut(m2_hole)
    m2_hole = cyl(sg90_m2_r, sg90_ear_t + 2, v(ear_x0e - 1, by1 + sg90_ear_y / 2, z_shaft), v(1, 0, 0))
    ear_b   = ear_b.cut(m2_hole)

    shape = body.fuse(shaft_stub).fuse(arm_block).fuse(munon).fuse(ear_f).fuse(ear_b)

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

# Motor disc (one rigid part: two plates + spool) + its Ø8 flange hub
add(doc, "MotorDisc",
    make_motor_disc(),
    color=(1.0, 0.60, 0.15))
add(doc, "MotorHub",
    make_motor_hub(),
    color=(0.75, 0.75, 0.78))

# Single axis for clarity — all four are identical rotated 90°
AXES = [("PosY", v(0, 1, 0))]

for name, axis in AXES:
    add(doc, f"OutputWheel_{name}",
        make_output_wheel(axis),
        color=(0.20, 0.80, 0.60))
    add(doc, f"FlangeHub_{name}",
        make_flange_hub(axis),
        color=(0.75, 0.75, 0.78))
    # Ball bearing reference: ring seated at the head shoulder
    _brg_outer = cyl(brg_od / 2, brg_w, av(axis, head_y1 - brg_w), axis)
    _brg_inner = cyl(brg_id / 2 + 0.05, brg_w + 2, av(axis, head_y1 - brg_w - 1), axis)
    add(doc, f"HeadBearing_{name}",
        _brg_outer.cut(_brg_inner),
        color=(0.30, 0.30, 0.32))
    # Output-shaft bearing, flush inside the wall foot (58.5 → 62.5)
    _wb_o = cyl(brg_od / 2, brg_w, av(axis, cube_half + wall_thick - brg_w), axis)
    _wb_i = cyl(brg_id / 2 + 0.05, brg_w + 2, av(axis, cube_half - 1), axis)
    add(doc, f"WallBearing_{name}",
        _wb_o.cut(_wb_i),
        color=(0.30, 0.30, 0.32))
    add(doc, f"Oring_{name}",
        make_oring(axis),
        color=(0.90, 0.20, 0.50))
    add(doc, f"OutputShaft_{name}",
        make_output_shaft(axis),
        color=(0.45, 0.75, 0.65))
    add(doc, f"UJ_{name}",
        make_uj_body(axis),
        color=(0.85, 0.65, 0.10))
    add(doc, f"Wall_{name}",
        make_cube_wall(axis),
        color=(0.45, 0.55, 0.75),
        transparency=30)
    add(doc, f"Bracket_{name}",
        make_shaft_bracket(axis),
        color=(0.30, 0.65, 0.90))

add(doc, "Frame",
    make_frame(AXES),
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
print("-" * 60)
print(f"  Engagement spring:  {eng_blade_w:.0f} × {eng_blade_t:.1f} mm, free length {spr_L:.0f} mm,"
      f"  k = {spr_k:.1f} N/mm at pin")
print(f"  Servo arm:          r = {sg90_arm_r:.0f} mm, latch stop at {latch_deg:.0f}°"
      f"  → slot height {eng_slot_h:.1f} mm (derived)")
print(f"  Free travel:        {spr_free:.2f} mm → contact at {spr_a_contact:.0f}° of arm sweep")
print(f"  Latched (self-held): F = {spr_F_latch:.1f} N → N = {spr_N_latch:.1f} N"
      f" → T_slip = {spr_T_latch:.0f} N·mm = {spr_T_latch / SRV_T_CONT:.1f}× servo cont.")
print(f"  Latch root stress:  {spr_sig_latch:.1f} MPa sustained  (creep target ≤ ~15 MPa)")
print(f"  Servo demand:       {spr_peak_dem:.0f} N·mm peak, transient"
      f"  (MG90D continuous {SRV_T_CONT:.0f} N·mm)")
print(f"  Wear sensitivity:   −{spr_dF_wear:.1f} N per 0.1 mm gap growth"
      f"  ({100 * spr_dF_wear / spr_F_latch:.0f}% of latch force)")
print("-" * 60)
_flange_disc_clr = contactZ - hub_flange_od / 2
_free_shaft      = head_y1 - (wc_dist + WT / 2)
print(f"  Wheel↔shaft hub:    flange Ø{hub_flange_od:.0f} on MOTOR side, neck through wheel,")
print(f"                      {hub_bolt_n}× M{hub_bolt_d:.0f} on Ø{hub_bolt_cd:.0f} circle,"
      f" M3 self-tap into PETG, 2× M3 set screws on a shaft flat")
print(f"  Flange↔disc gap:    {_flange_disc_clr:.1f} mm"
      f"  ({'OK' if _flange_disc_clr > 0 else 'COLLISION'})")
print(f"  Output bearings:    2× Ø{brg_id:.0f}×{brg_od:.0f}×{brg_w:.0f} (MR105ZZ) per axis —"
      f" head (half-height {head_z_half:.1f} mm) + wall foot")
print(f"  Wall-side shaft:    free {wc_dist + WT / 2:.1f}→{head_y1:.1f} mm"
      f" for the bearing seat in the head")
print(f"  Motor disc:         one part (2 plates + Ø{disc_spool_d:.0f} spool),"
      f" Ø8 hub below, {mhub_bolt_n}× M{mhub_bolt_d:.0f} self-tap")
print(f"  Motor disc torque:  ≤ {4 * spr_T_latch:.0f} N·mm (4 axes engaged)"
      f"  → {4 * spr_T_latch / mhub_bolt_n / (mhub_bolt_cd / 2):.0f} N/bolt")
print("=" * 60)
