# Motcore v6 — Apex Pivot Cone Clutch
# FreeCAD Python Macro
#
# Built from cad/motcore_v5_vertical_clutch.py (cone solids, FDM hole
# compensation, gear helpers and the self-check scaffolding are lifted almost
# verbatim). The design it implements is docs/clutch-geometry-v6.md — read that
# first; cad/clutch_geometry_v6.html is the interactive 2D visualiser this
# geometry is taken from, and every number below that looks arbitrary comes
# from it.
#
# The one-line difference from v5: the output cone PIVOTS about the common
# apex instead of translating. Consequences that shape this file:
#
#   - ONE apex for all three cones, at the global origin, and what converges
#     there are the RUBBER surfaces. The plastic apexes are therefore pulled
#     back along their own axes by t/sin(half-angle) (invariant 2).
#   - A continuous rubber layer of thickness t replaces v5's interleaved
#     O-rings, so there are no grooves and no ring pitch.
#   - The pivot is VIRTUAL: a four-bar linkage whose two link axes converge on
#     the apex (nothing physical can sit there — the motor shaft does). The
#     linkage is solved numerically here, and the residual apex drift is
#     measured and printed, which answers open question §10.3 of the doc.
#   - The gear stage NEVER disengages. Free rotation comes from the rubber
#     separating. Pinion (carriage, centred) -> two idlers on fixed axes at
#     +-X -> internal corona on the output shaft, which is what puts the
#     output shaft on the wall centre.
#
# Coordinates (one axis, before the per-wall rotation about Z):
#   origin = the common apex = the pivot.  +Z = motor shaft, up.
#   +Y     = toward this axis' wall / output shaft.  X = across.
#   phi    = carriage tilt, 0 = free, + engages the UPPER motor cone.
#
# Run headless to read the checks:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" motcore_v6_apex_pivot.py
# or from FreeCAD: Macro -> Macros -> motcore_v6_apex_pivot.py -> Execute

import FreeCAD as App
import Part
import math
import os

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
# PARAMETERS  ← edit here, then re-run   (mirrors clutch_geometry_v6.html)
# ═══════════════════════════════════════════════════════════════════

# ── Cones and rubber ─────────────────────────────────────────────────────────
alpha_deg = 55.0   # deg — motor cone half-angle, from the vertical (Z) axis
beta_deg  = 33.0   # deg — output cone half-angle, from its own (Y) axis.
                    #       alpha + beta < 90: the difference IS the free gap
                    #       angle phi_c, so these two numbers set the travel.
t_rubber  = 2.0    # mm  — continuous rubber layer thickness (both cones)
cone_gen  = 34.0   # mm  — motor cone generatrix length, from its own plastic
                    #       apex. This is the size parameter now: the cone is
                    #       what has to fit in the cube, and the contact line is
                    #       whatever the cone gives once the lip comes off.
scarf_w   = 3.0    # mm  — width of the feathered (scarfed) seam edge. Both
                    #       edges are bevelled the opposite way, so they
                    #       overlap into a wedge WITHOUT adding thickness —
                    #       a lapped seam would double t right in the contact
                    #       and break invariant 2.
WRITE_FLAT_PATTERN = True   # write the 1:1 rubber cutting template beside this
                             # macro on every run, so it cannot fall out of step
                             # with the parameters above
rubber_lip = 2.0   # mm  — bare plastic left past the rubber's outer edge, to
                    #       bond and retain the wrapped sheet. That lip is the
                    #       ONLY reason for plastic outside the band: v5 needed
                    #       6 mm there to hold material past the last O-ring
                    #       groove, and the margin was inherited without its
                    #       reason. Since contact pins itself at the largest
                    #       radius the band offers, every mm of bare rim is
                    #       contact length thrown away. L_line is DERIVED now.
s0        = 10.0   # mm  — apex → start of the contact line, ON THE RUBBER
                    #       surface (which is the one whose apex is the origin)

# ── Gear stage — always meshed ───────────────────────────────────────────────
# m = 1.0 IS NOT BUILDABLE and the doc's table is wrong about it. At Zp = 8 the
# pinion root radius is m*(Zp/2 - 1.25) = 2.75 mm, and the FDM-compensated bore
# for the Ø5 output shaft is r = 2.90 mm: the bore eats the whole hub and the
# gear comes out as eight loose teeth. The floor for a 2 mm hub wall is
#       m >= (fdm_shaft_hole_d/2 + wall) / (Zp/2 - 1.25) = 1.78
# so the default here is 1.8 (which is also v5's module floor, for an unrelated
# reason). The "pinion hub wall" check at the bottom prints the margin; set
# m_mod = 1.0 and it fails loudly with the number. The alternatives, if 1.8 is
# too big: more pinion teeth, a stepped Ø3 shaft end (but Ø3 steel is at its
# torsional limit near 0.5 Nm), or a pinion printed integral with the shaft
# (which then cannot be plastic — it carries the full output torque).
# NOTE the module does NOT touch the clutch geometry in v6: the gears never
# disengage, so there is no mesh travel and no root-clearance constraint. It is
# purely a packaging cost, unlike v5 where m was on the critical path.
m_mod = 1.8        # mm  — gear module
Zp    = 8          # —   — pinion teeth (on the carriage, centred)
Zi    = 8          # —   — idler teeth (fixed axes, left and right in X)
Zc    = 24         # —   — corona teeth (internal, on the output shaft)
                    #       Zc = Zp + 2*Zi is NOT a free choice: it is what
                    #       makes the idler centre distance the same on both
                    #       meshes, i.e. what lets the corona stay concentric
                    #       with the pinion. Checked below.

# ── Four-bar virtual pivot (y, z), apex at the origin ────────────────────────
# Both link axes must pass through the origin — that is what makes the
# carriage's instant centre the apex. The residual drift of the apex POINT is
# measured further down; the instant centre itself wanders ~10 mm, which is
# normal and harmless.
fb_A = (37.3, 44.4)   # mm — frame pivot, upper link (radius 58 from the apex)
fb_B = (27.0, 32.2)   # mm — carriage pivot, upper link (radius 42)
                       #      The lower pair is the mirror in z.
link_x     = 9.0      # mm — X of the link plane (two sets, at +-link_x, for
                       #      out-of-plane stiffness)
link_t     = 5.0      # mm — link thickness (along X)
link_w     = 10.0     # mm — link width
link_knuckle = 5.0    # mm — radius of the knuckle joining the pair into one
                       #      part. It sits ON the carriage pivot, which is the
                       #      one place with room: the span between the pivots
                       #      is taken by the frame lug hanging down the centre,
                       #      and a tail out past the pivot reaches down into
                       #      the motor cone once the bar's own half-width is
                       #      counted.
pin_d      = 4.0      # mm — pivot pin diameter (all four-bar pins)

# ── Actuation: servo → crank → telescopic spring link → carriage trunnion ────
# First pass. Everything here is a placeholder until the rubber stiffness is
# measured (doc §10.1) — that number decides the spring, and the spring decides
# the servo.
R_push        = 46.0   # mm — apex → push point, on the carriage centre line
act_amp       = 6.0    # mm — servo-side travel of the push point, each way
spring_split  = 11.0   # —  — after contact, 1 part of further servo travel
                        #      goes into the carriage and (split-1) into the
                        #      spring. 10:1 is invented (doc §10.2).
crank_hub     = (46.0, -28.0)  # mm (y, z) — servo output shaft. It belongs
                        #      on the push point's own vertical: off it, the
                        #      distance hub→push point varies over the stroke
                        #      and the link stops reaching one end. Moved to
                        #      y = 40 to dodge the corona, it could no longer
                        #      reach the top of the stroke at all (34.5 mm
                        #      needed, 34.3 available) — the clearance came
                        #      from moving the actuation plane out in X
                        #      instead, see act_x.
crank_r       = 6.3    # mm — crank radius
act_link_len  = 28.0   # mm — telescopic link, free length
act_x         = 24.0   # mm — X of the actuation plane (link + trunnion).
                        #      Set by the clamp nuts, which stand 2.4 mm proud
                        #      of the block's outer face at x = 17: at 22 the
                        #      link sat 0.1 mm off one of them.
                        #      Moved out from 20 when the arms became L-shaped:
                        #      their uprights run straight down at y = 37..46,
                        #      which is exactly where the crank pin swings, and
                        #      the diagonal arms used to pass inboard of it.
                        #      OUTBOARD of the carriage side plate: inboard of
                        #      it the only gap is the 4 mm between the housing
                        #      (r = 8) and the plate, and the link is 5 thick.
                        #      The trunnion therefore spans housing → plate →
                        #      link, i.e. it is held at both ends rather than
                        #      cantilevered off the housing.
                        #      NOTE the push is single-sided here: a real
                        #      symmetric push would need a fork straddling the
                        #      bearing housing, or a second servo.
servo_body    = (12.2, 23.0, 29.0)   # mm (X, Y, Z) — MG90-class placeholder
servo_tab_t   = 2.5    # mm — thickness of the servo's own mounting tabs
servo_tab_out = 4.7    # mm — how far each tab reaches past the body (an MG90 is
                        #      32.2 long over the tabs against 22.8 of body)
servo_screw_d = 2.0    # mm — M2 through the tabs into the bracket
servo_plate_t = 3.0    # mm — bracket face plate thickness
servo_spline_h = 4.0   # mm — how far the output spline stands off the servo's
                        #      top face. The crank rides on the spline, which is
                        #      what lets it sweep OVER the two tab screws: its
                        #      own swept radius is 10.3 mm and the screws sit at
                        #      8.15, so with the crank flat on the case face
                        #      they collide.
servo_foot_z  = -39.0  # mm — Z of the bracket's two screws into the wall

# ── Render pose ──────────────────────────────────────────────────────────────
CARRIAGE_STOP = "contact"   # "free" | "contact" | "preload"
CARRIAGE_DIR  = -1          # -1 = tilt DOWN (engages the LOWER motor cone),
                             # +1 = UP. Ignored at "free", which is the middle.

AXES_SHOWN = 1     # 0..4 — output axes built (mechanism + wall). 1 keeps the
                    #        view readable; 4 is the full cube. The central
                    #        motor cones and shaft are always built.

# ── Carriage ─────────────────────────────────────────────────────────────────
hous_x        = 17.0   # mm — bearing block half-width (X). The side arms are
                        #      slices of this same block, so it reaches out to
                        #      side_x + side_t/2.
hous_z        = 12.0   # mm — bearing block half-height (Z). Taller than the
                        #      bearing seats need, to leave room beside them
                        #      for the clamp bolts.
hous_y0       = 33.5   # mm — block, cone-side face. The output cone's base rim
                        #      sweeps to y = 32.5 within |z| < hous_z at full
                        #      preload, so this is a 1 mm running clearance.
hous_y1       = 49.0   # mm — block, pinion-side face. Runs past the push
                        #      trunnion (y = R_push = 46, Ø5, so it ends at
                        #      48.5) instead of stopping level with it, which
                        #      left the pin emerging from the very corner. The
                        #      idler bracket's arm at y = 50 is the stop: at
                        #      full preload the block's top corner swings to
                        #      y = 49.3.
side_x        = 14.5   # mm — X of the carriage side plates (two, mirrored).
                        #      Must clear the output cone's base radius: the
                        #      arms are routed around it, see make_carriage.
side_t        = 5.0    # mm — side plate thickness
arm_w         = 9.0    # mm — carriage arm width
arm_root      = (44.0, 5.0)   # mm (y, |z|) — where each arm leaves the block
                               #      on its way to a four-bar pivot. Diagonal,
                               #      not an L: once the part is printed lying
                               #      on its side the two are equally printable,
                               #      and the diagonal is the lighter and
                               #      stiffer load path. NOT free, though: the
                               #      arm has to cross the output cone's base
                               #      plane outside the cone, which is 18.2 mm
                               #      in radius there against the plate at
                               #      |x| = 12, so it must cross y = 32 above
                               #      |z| = 13.9. From here it crosses at 15.7;
                               #      from (40, 4) it crossed at 9.4 and cut
                               #      into the cone.
bolt_y        = (36.0, 46.5)  # mm — clamp bolts, one pair beside each bearing
bolt_z        = 8.5    # mm — |Z| of the clamp bolts: clear of the Ø10.3 seat
                        #      (5.15) and of the block's own face (12.0)
bolt_d        = 3.0    # mm ┐
bolt_head_d   = 5.5    # mm │ M3 socket head and nut, as envelopes. Modelled as
bolt_head_h   = 3.0    # mm │ solids and not just as holes, because a hole
bolt_nut_d    = 6.4    # mm │ collides with nothing: it was the HEAD that ran
bolt_nut_h    = 2.4    # mm ┘ into the actuation link, and nothing could see it.
trunnion_d    = 5.0    # mm — push trunnion diameter (a length of the Ø5 rod
                        #      the project already buys, NOT a printed boss)
trunnion_x_in = 6.0    # mm — |X| the trunnion is pressed in to; clear of the
                        #      Ø10.3 bearing seat, which reaches 5.15

# ── Gear stage packaging ─────────────────────────────────────────────────────
gear_face_w   = 5.0    # mm — pinion / idler / corona face width
gear_y0       = 53.5   # mm — gear plane, front face (carriage side). Pushed
                        #      out from 51.5: the idler bracket's arm has to
                        #      cross in front of it, and at 51.5 that arm ran
                        #      into the push trunnion's Ø5 boss at y = 46.
gear_back_gap = 1.5    # mm — axial clearance, pinion back face → corona
                        #      plate, taken out of the PINION's face width.
                        #      Not cosmetic: the pinion tilts with the carriage,
                        #      so its rim sweeps ~0.4 mm in Y at full preload,
                        #      and flush against the plate it jammed. Taking it
                        #      out of the corona's Y position instead detached
                        #      the ring from its own back plate — the part came
                        #      out as two solids fused into one Shape.
corona_rim_t  = 4.0    # mm — corona rim beyond the pitch circle (freecad.gears
                        #      sizes the outer solid as pitch + 2*thickness)
corona_plate_t = 4.0   # mm — plate closing the corona's back face, fused into
                        #      the output shaft
idler_axle_d  = 4.0    # mm — idler stub axle
idler_arm_t   = 3.0    # mm — idler bracket arm thickness (along Y)
idler_arm_w   = 5.0    # mm — idler bracket arm width (along X)
idler_leg_t   = 4.0    # mm — idler bracket leg thickness (along Z)
idler_arm_y1  = 53.0   # mm — bracket arm, gear-side face (clear of gear_y0)

# ── Frame ────────────────────────────────────────────────────────────────────
fp_plate_t    = 6.0    # mm — frame-pivot bracket plate thickness
fp_plate_gap  = 6.5    # mm — plate's inner face above the pin axis. Must
                        #      clear the link's own eye (radius link_w/2), not
                        #      just the pin: at 2.0 the plate sat inside it.
fp_lug_x      = 6.0    # mm — frame-pivot lug half-width in X (the links sit
                        #      just outboard of it)
fp_bracket_x  = 14.0   # mm — bracket plate half-width in X
fp_bracket_y0 = 31.0   # mm — bracket plate, inboard end

foot_t        = 3.0    # mm — thickness of a bracket's foot against the wall
foot_screw_d  = 3.0    # mm — M3 through the wall into a bracket foot
foot_hole_d   = 3.8    # mm — modelled Ø for it (printer runs holes under)
foot_flange   = 9.0    # mm — how far a frame-bracket foot turns inboard to give
                        #      its screws something to pass through
wall_thick     = 4.0   # mm — cube wall thickness
wall_gap       = 6.0   # mm — clearance, outermost rotating part → wall
wall_shaft_clr = 2.0   # mm — radial clearance of the wall's output-shaft hole

# ── Reference-only extras (2D cross-section → solids) ────────────────────────
cone_apex_trim = 3.0   # mm — truncate a cone this far (generatrix) from its
                        #      own PLASTIC apex
cone_tip_wall  = 0.8   # mm — minimum wall at a cone's truncated tip where a
                        #      through bore exists (motor cones)
cone_bore_wall = 2.0   # mm — wall left at the far end of the output cone's
                        #      BLIND bore, which is what sets its depth
shaft_d        = 5.0   # mm — motor and output shaft diameter

# ── Carriage bearings — MR105ZZ, the project's single standard bearing ───────
brg_id        = 5.0    # mm ┐
brg_od        = 10.0   # mm │ MR105ZZ (5x10x4)
brg_w         = 4.0    # mm ┘
brg_fit_press = 0.15   # mm — radial add for a press fit (→ Ø10.3), FDM
                        #      calibrated on the Creality Hi, see build-log

# ── FDM print calibration (Creality Hi / PLA) — see docs/build-log.md ────────
# This printer runs small holes ~0.5 mm UNDERSIZE, so every hole that receives
# real hardware is modelled oversize by a measured amount, not at nominal.
fdm_shaft_hole_d = 5.8  # mm — modelled Ø for a Ø5 rod (measured, coupon v2)
fdm_pin_hole_d   = 4.6  # mm — modelled Ø for a Ø4 pin (extrapolated — verify)
fdm_bolt_hole_d  = 3.8  # mm — modelled Ø for an M3 clamp bolt to pass through
fdm_dowel_hole_d = 5.5  # mm — modelled Ø for the Ø5 trunnion dowel, INTERFERENCE
                         #      (5.8 is the measured slip fit, so this is 0.3
                         #      tighter). Extrapolated, not measured — verify on
                         #      a coupon before relying on it.

# ═══════════════════════════════════════════════════════════════════
# DERIVED
# ═══════════════════════════════════════════════════════════════════

alpha = math.radians(alpha_deg)
beta  = math.radians(beta_deg)

phi_c = math.radians(90.0 - alpha_deg - beta_deg)   # free gap angle (rad)

# Invariant 2: the RUBBER surfaces converge on the origin, so each plastic apex
# is pulled back along its own axis, in the direction the cone flares.
apex_off_motor  = t_rubber / math.sin(alpha)
apex_off_output = t_rubber / math.sin(beta)

ratio_fric  = math.sin(alpha) / math.sin(beta)
ratio_gear  = Zp / Zc                 # the idlers do not change the ratio
ratio_total = ratio_fric * ratio_gear

# Gear stage geometry
r_pitch_p = m_mod * Zp / 2.0
r_pitch_i = m_mod * Zi / 2.0
r_pitch_c = m_mod * Zc / 2.0
r_tip_p   = m_mod * (Zp / 2.0 + 1.0)
r_tip_i   = m_mod * (Zi / 2.0 + 1.0)
r_tip_c   = m_mod * (Zc / 2.0 - 1.0)          # internal: teeth point inward
r_root_p  = m_mod * (Zp / 2.0 - 1.25)
r_corona_outer = r_pitch_c + corona_rim_t
e_ext     = m_mod * (Zp + Zi) / 2.0           # pinion → idler centre distance
e_int     = m_mod * (Zc - Zi) / 2.0           # idler → corona centre distance
idler_x   = e_ext                              # idlers on the X axis (inv. 6)
idler_leg_z = r_corona_outer + 3.0             # bracket clears the corona rim
pinion_hub_wall = r_root_p - fdm_shaft_hole_d / 2.0

gear_y1   = gear_y0 + gear_face_w
gear_y_mid = gear_y0 + gear_face_w / 2.0

# Cone extents. s is measured along the generatrix from the cone's OWN plastic
# apex; the rubber band is specified from the COMMON apex instead, and the two
# are offset by t*cot(half-angle) — see docs §3.
def _s_plastic(s_rubber, half_angle):
    """Generatrix distance from the PLASTIC apex of the point that sits under
    the rubber-surface point at s_rubber from the common apex."""
    return s_rubber - t_rubber / math.tan(half_angle)

# The band is shared, so its outer end is set by whichever cone's rubber runs
# out first: the motor cone, whose apex offset is the smaller of the two. Each
# cone's PLASTIC then stops rubber_lip past the band — which makes the output
# cone shorter than it used to be, since its shallower angle reaches the same
# rubber radius with less plastic.
s_rub_lo = s0
s_rub_hi = cone_gen + t_rubber / math.tan(alpha) - rubber_lip
L_line = s_rub_hi - s_rub_lo                   # DERIVED, printed below
s_mot_hi = cone_gen                            # plastic, from the plastic apex
s_out_hi = s_rub_hi + rubber_lip - t_rubber / math.tan(beta)
s_motor_lo  = max(cone_apex_trim,
                  (fdm_shaft_hole_d / 2.0 + cone_tip_wall) / math.sin(alpha))
s_output_lo = cone_apex_trim                    # blind bore → tip stays solid


out_apex_y  = apex_off_output                   # output cone's plastic apex
out_base_y  = out_apex_y + s_out_hi * math.cos(beta)
out_base_r  = s_out_hi * math.sin(beta)
out_tip_y   = out_apex_y + s_output_lo * math.cos(beta)
# Blind bore: deep enough to clamp on the shaft, stopping where the cone wall
# has thinned to cone_bore_wall.
out_bore_end_y = out_apex_y + (
    (fdm_shaft_hole_d / 2.0 + cone_bore_wall) / math.tan(beta))
out_bore_depth = out_base_y - out_bore_end_y

mot_apex_z = apex_off_motor
mot_base_z = mot_apex_z + s_mot_hi * math.cos(alpha)
mot_base_r = s_mot_hi * math.sin(alpha)

# ── Four-bar ────────────────────────────────────────────────────────────────
# (y, z) pairs. Upper link A1-B1, lower link A2-B2, coupler B1-B2 = the
# carriage. Solved numerically: the closed form would hide exactly the drift
# this macro exists to measure.
A1 = (fb_A[0],  fb_A[1])
A2 = (fb_A[0], -fb_A[1])
B1_0 = (fb_B[0],  fb_B[1])
B2_0 = (fb_B[0], -fb_B[1])
link_len   = math.dist(A1, B1_0)
coupler_len = math.dist(B1_0, B2_0)
_a10  = math.atan2(B1_0[1] - A1[1], B1_0[0] - A1[0])
_ang0 = math.atan2(B2_0[1] - B1_0[1], B2_0[0] - B1_0[0])
r_frame_pivot    = math.hypot(*A1)
r_carriage_pivot = math.hypot(*B1_0)
# Invariant: both link-axis endpoints must lie on ONE ray from the apex.
link_axis_deg    = math.degrees(math.atan2(A1[1], A1[0]))
link_axis_err    = abs(link_axis_deg - math.degrees(math.atan2(B1_0[1], B1_0[0])))


def _circ_int(p0, r0, p1, r1, ref):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    xm, ym = p0[0] + a * dx / d, p0[1] + a * dy / d
    s1 = (xm + h * dy / d, ym - h * dx / d)
    s2 = (xm - h * dy / d, ym + h * dx / d)
    return s1 if math.dist(s1, ref) < math.dist(s2, ref) else s2


def fb_pose(theta):
    """Linkage pose for an input rotation theta of the upper link about A1.
    Returns (B1, B2, phi) with phi = the carriage's own rotation."""
    B1 = (A1[0] + link_len * math.cos(_a10 + theta),
          A1[1] + link_len * math.sin(_a10 + theta))
    B2 = _circ_int(A2, link_len, B1, coupler_len, B2_0)
    if B2 is None:
        return None
    return B1, B2, math.atan2(B2[1] - B1[1], B2[0] - B1[0]) - _ang0


def fb_solve(phi_target, step=2e-5, limit=0.4):
    """Pose at a given carriage tilt. Marches theta out from zero (the
    theta→phi relation is not monotone far from the rest pose, so a plain
    bisection over a wide bracket picks the wrong branch or a locked one) and
    bisects inside the interval that brackets the target."""
    if abs(phi_target) < 1e-12:
        return fb_pose(0.0)
    probe = fb_pose(1e-4)
    direction = 1.0 if math.copysign(1.0, probe[2]) == math.copysign(1.0, phi_target) else -1.0
    theta = 0.0
    while abs(theta) < limit:
        nxt = theta + direction * step
        p = fb_pose(nxt)
        if p is None:
            break
        if abs(p[2]) >= abs(phi_target):
            lo, hi = theta, nxt
            for _ in range(60):
                mid = (lo + hi) / 2.0
                q = fb_pose(mid)
                if q is None or abs(q[2]) < abs(phi_target):
                    lo = mid
                else:
                    hi = mid
            return fb_pose((lo + hi) / 2.0)
        theta = nxt
    return None


def carriage_transform(phi):
    """The rigid (y, z) map the four-bar applies to the carriage at tilt phi:
    rotate about the rest position of B1, then translate B1 to where the
    linkage actually puts it."""
    pose = fb_solve(phi)
    B1 = pose[0]
    c, s = math.cos(phi), math.sin(phi)

    def T(q):
        dy, dz = q[0] - B1_0[0], q[1] - B1_0[1]
        return (c * dy - s * dz + B1[0], s * dy + c * dz + B1[1])
    return T, pose


def apex_drift(phi):
    """How far the carriage point that starts AT the apex actually moves, and
    how that splits between the shared generatrix (→ slip) and its normal
    (→ preload). Answers doc open question §10.3."""
    T, _ = carriage_transform(phi)
    d = T((0.0, 0.0))
    total = math.hypot(*d)
    # Shared generatrix at contact, on the engaged side: 90-alpha from +Y.
    sd = 1.0 if phi >= 0 else -1.0
    ga = math.radians(90.0 - alpha_deg) * sd
    g = (math.cos(ga), math.sin(ga))
    along = d[0] * g[0] + d[1] * g[1]
    normal = -d[0] * g[1] + d[1] * g[0]
    return total, along, normal, d


# ── Stop ladder. No mesh step: the gears never disengage (invariant 4) ───────
contact_travel = R_push * phi_c                 # push-point travel, free→contact
phi_preload = (contact_travel
               + (act_amp - contact_travel) / spring_split) / R_push
STOPS = {"free": 0.0, "contact": phi_c, "preload": phi_preload}
phi = CARRIAGE_DIR * STOPS[CARRIAGE_STOP] if CARRIAGE_STOP != "free" else 0.0

drift_contact = apex_drift(phi_c)
drift_preload = apex_drift(phi_preload)
slip_fourbar  = abs(drift_preload[1]) / L_line   # same measure as v5's micro-slip

# ── Actuator state at the rendered pose ─────────────────────────────────────
_UNREACHABLE = []


def actuator(phi_r):
    """Servo-side push-point position and crank pin, from the series-spring
    model in the visualiser: before contact the carriage follows the servo,
    after it only 1/spring_split of further servo travel reaches the carriage
    and the rest compresses the spring."""
    zh = R_push * phi_r
    sgn = 1.0 if zh >= 0 else -1.0
    if abs(zh) <= contact_travel:
        zc = zh
    else:
        zc = sgn * (contact_travel + (abs(zh) - contact_travel) * spring_split)
    # Crank pin = the crank circle meeting the link circle about the push
    # point. Circle-circle rather than the visualiser's closed form, so that
    # the branch can be chosen: the two solutions are mirror images about the
    # hub→push-point line, and the crank sweeping on the MOTOR side (pin_y <
    # R_push) is what keeps the link out of the gear plane. On the wall side
    # the pin reaches y = 52.3 and the link, being a tube, put its shoulder
    # 0.2 mm inside the corona's front face. Same mechanism, mirrored.
    pin = _circ_int(crank_hub, crank_r, (R_push, zc), act_link_len,
                    (crank_hub[0] - crank_r, crank_hub[1]))
    if pin is None:      # crank + link cannot reach this stop at all
        if abs(zc) <= act_amp + 1e-9:    # only the design stroke must be
            _UNREACHABLE.append(zc)      # reachable; the contact bisection
                                          # deliberately explores well past it
        pin = (crank_hub[0] + crank_r, crank_hub[1])
    psi = math.atan2(pin[0] - crank_hub[0], pin[1] - crank_hub[1])
    return zc, psi, pin



# ── Cube ────────────────────────────────────────────────────────────────────
pinion_face_w = gear_face_w - gear_back_gap   # short face = the clearance
corona_plate_y = gear_y1                      # plate is CONTIGUOUS with the ring
corona_back_y = corona_plate_y + corona_plate_t
cube_half = corona_back_y + wall_gap
cube_out  = cube_half + wall_thick

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def v(x, y, z):
    return App.Vector(x, y, z)


X_AXIS, Y_AXIS, Z_AXIS = v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)
ORIGIN = v(0, 0, 0)


def add(doc, name, shape, color=(0.8, 0.8, 0.8), transparency=0):
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if HAS_GUI and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency
    return obj


def cyl(r, h, base, direction=Z_AXIS):
    return Part.makeCylinder(r, h, base, direction)


def cone_frustum(apex, axis, half_angle, s_lo, s_hi):
    """Solid cone between generatrix distances s_lo and s_hi from `apex`,
    flaring along `axis`. s is the true slant distance, so the same number
    means the same surface point on the plastic and on the rubber."""
    ca, sa = math.cos(half_angle), math.sin(half_angle)
    base = v(apex.x + s_lo * ca * axis.x,
             apex.y + s_lo * ca * axis.y,
             apex.z + s_lo * ca * axis.z)
    return Part.makeCone(s_lo * sa, s_hi * sa, (s_hi - s_lo) * ca, base, axis)


def prism_yz(points, x0, thick):
    """Prism from a (y, z) polygon, extruded along +X from x0."""
    pts = [v(x0, p[0], p[1]) for p in points]
    wire = Part.makePolygon(pts + [pts[0]])
    return Part.Face(wire).extrude(v(thick, 0, 0))


def disc_yz(centre, r, x0, thick):
    circ = Part.makeCircle(r, v(x0, centre[0], centre[1]), X_AXIS)
    return Part.Face(Part.Wire([circ])).extrude(v(thick, 0, 0))


def bar_yz(p, q, w, x0, thick):
    """Round-ended bar from (y, z) point p to q, width w."""
    dy, dz = q[0] - p[0], q[1] - p[1]
    ln = math.hypot(dy, dz)
    ny, nz = -dz / ln * w / 2.0, dy / ln * w / 2.0
    quad = [(p[0] + ny, p[1] + nz), (q[0] + ny, q[1] + nz),
            (q[0] - ny, q[1] - nz), (p[0] - ny, p[1] - nz)]
    return (prism_yz(quad, x0, thick)
            .fuse(disc_yz(p, w / 2.0, x0, thick))
            .fuse(disc_yz(q, w / 2.0, x0, thick)))


def box_yz(y0, y1, z0, z1, x0=0.0, thick=1.0):
    return Part.makeBox(thick, y1 - y0, z1 - z0, v(x0, y0, z0))


def pin_x(centre, d, x0, length):
    return cyl(d / 2.0, length, v(x0, centre[0], centre[1]), X_AXIS)


def pose_state(phi_t):
    """Everything that depends on the tilt, in one dict, so that any pose can
    be built and checked rather than only the rendered one. The first version
    of this macro baked the rendered pose into module globals, and the links
    and the actuation train were consequently never checked against anything —
    which is exactly where two real collisions were hiding."""
    T, pose = carriage_transform(phi_t)
    zc, psi, pin = actuator(phi_t)
    return {"phi": phi_t, "T": T, "B1": pose[0], "B2": pose[1],
            "pin": pin, "tab": T((R_push, 0.0)), "zc": zc, "psi": psi}


def make_screw(base, direction, d, length, head_d, head_h):
    """A screw as an envelope: shank from `base` along `direction`, head on the
    near side. Modelled as a solid, not as a hole — a hole collides with
    nothing, which is how a clamp nut ended up inside the actuation link."""
    head_base = v(base.x - direction.x * head_h,
                  base.y - direction.y * head_h,
                  base.z - direction.z * head_h)
    return cyl(d / 2.0, length, base, direction).fuse(
        cyl(head_d / 2.0, head_h, head_base, direction))


def wall_screws():
    """(x, z) of every screw that passes through this axis's wall. One list, so
    the wall and the brackets cannot disagree about where the holes are."""
    out = []
    for sd in (1, -1):
        for dx in (-(idler_arm_w / 2.0 + 3.5), idler_arm_w / 2.0 + 3.5):
            out.append((sd * idler_x + dx, idler_leg_z - idler_leg_t / 2.0))
    for zs in (1, -1):
        z_in = zs * (fb_A[1] + fp_plate_gap)
        for dx in (-12.0, 12.0):
            out.append((dx, z_in - zs * foot_flange / 2.0))
    for dx in (4.0, 11.0):
        out.append((servo_box()[0] + servo_tab_t + dx, servo_foot_z))
    return out


def place_carriage(shape, st):
    """Move a carriage part from its rest pose to where the four-bar actually
    puts it. Rotation about +X takes +Y toward +Z, which is the sign
    convention for phi (positive engages the upper motor cone)."""
    sh = shape.copy()
    if abs(st["phi"]) > 1e-12:
        sh.rotate(v(0, B1_0[0], B1_0[1]), X_AXIS, math.degrees(st["phi"]))
        sh.translate(v(0, st["B1"][0] - B1_0[0], st["B1"][1] - B1_0[1]))
    return sh


# ═══════════════════════════════════════════════════════════════════
# PART BUILDERS
# ═══════════════════════════════════════════════════════════════════

def make_motor_cone(sd):
    """One motor cone. sd = -1 is the LOWER one (flares down, apex up), +1 the
    UPPER one — the same printed part, fitted upside down. Its PLASTIC apex is
    pulled back to z = sd*apex_off_motor so that its RUBBER surface's apex is
    the origin (invariant 2)."""
    axis = v(0, 0, sd)
    apex = v(0, 0, sd * mot_apex_z)
    cone = cone_frustum(apex, axis, alpha, s_motor_lo, s_mot_hi)
    bore_h = (s_mot_hi - s_motor_lo) * math.cos(alpha) + 4
    cone = cone.cut(cyl(fdm_shaft_hole_d / 2.0, bore_h,
                        v(0, 0, apex.z + sd * (s_motor_lo * math.cos(alpha) - 2)),
                        axis))
    return cone


def make_motor_rubber(sd):
    """The continuous rubber layer on one motor cone: everything between the
    plastic surface and the surface t proud of it, over the contact band. Its
    outer surface is a cone whose apex IS the origin — that is the whole point
    of v6, and it is what the interference checks below measure."""
    axis = v(0, 0, sd)
    outer = cone_frustum(ORIGIN, axis, alpha, s_rub_lo, s_rub_hi)
    plastic = cone_frustum(v(0, 0, sd * mot_apex_z), axis, alpha,
                           0.0, s_mot_hi + 20.0)
    return outer.cut(plastic)


def make_output_cone():
    """Output cone, rest pose: axis +Y, plastic apex pulled back to
    y = apex_off_output, flaring toward the wall. Bored BLIND from the base —
    a through bore would leave a 0.9 mm wall where the rubber band starts,
    and the tip points at the motor shaft anyway, so nothing needs to come out
    that end."""
    cone = cone_frustum(v(0, out_apex_y, 0), Y_AXIS, beta,
                        s_output_lo, s_out_hi)
    cone = cone.cut(cyl(fdm_shaft_hole_d / 2.0, out_bore_depth + 1.0,
                        v(0, out_bore_end_y, 0), Y_AXIS))
    return cone


def make_output_rubber():
    outer = cone_frustum(ORIGIN, Y_AXIS, beta, s_rub_lo, s_rub_hi)
    plastic = cone_frustum(v(0, out_apex_y, 0), Y_AXIS, beta,
                           0.0, s_out_hi + 20.0)
    return outer.cut(plastic)


def make_carriage_shaft():
    """Ø5 steel output shaft on the carriage: cone clamped at one end, pinion
    at the other, two bearings between (set screws on a filed flat, the joint
    pattern the project already uses)."""
    y_end = gear_y0 + pinion_face_w      # flush with the pinion's back face
    return cyl(shaft_d / 2.0, y_end - out_bore_end_y,
               v(0, out_bore_end_y, 0), Y_AXIS)


def make_carriage_half(sd):
    """HALF the carriage: bearing block, arms and push trunnion, cut on the
    x = 0 plane. The two halves are identical and bolt together around the
    shaft.

    This is a printing decision, and it is the only one that changes anything.
    Everything in this part is built from prisms swept along X — boxes, bars and
    discs in the Y-Z plane — so the whole carriage is a 2.5D extrusion in X and
    prints without a single overhang when laid on its side. Everything except
    the bearing seats, whose axis is Y: printed that way they would be bridged
    horizontal holes, out of round, and they are the one press fit in the part.
    Splitting on x = 0 turns each seat into a half-round pocket opening in the
    print direction, and every pin and bolt hole into a vertical one.

    v5 considered splitting the carriage and rejected it, but for a different
    reason: there the bolt had to run the length of the carriage. Here they run
    across its 34 mm width — M3 x 40.

    Making both halves carry a trunnion is what makes them identical, and it is
    also the fork mounting the doc asks for, which would take the twist out of
    the single-sided push. (The idea that the push pin could double as a clamp
    bolt does not survive contact with the geometry: it sits at z = 0, on the
    bore's own axis.)"""
    x_out = sd * hous_x
    x0, x1 = sorted((0.0, x_out))
    body = Part.makeBox(x1 - x0, hous_y1 - hous_y0, 2 * hous_z,
                        v(x0, hous_y0, -hous_z))
    # Cut with the FULL cylinders: each half keeps its own half of them, and
    # the two halves then clamp the bearing between them.
    body = body.cut(cyl(shaft_d / 2.0 + 0.75, hous_y1 - hous_y0 + 2,
                        v(0, hous_y0 - 1, 0), Y_AXIS))
    for y_face, sgn in ((hous_y0, 1), (hous_y1, -1)):
        y_start = min(y_face, y_face + sgn * brg_w)
        body = body.cut(cyl(brg_od / 2.0 + brg_fit_press, brg_w,
                            v(0, y_start, 0), Y_AXIS))
    body = body.fuse(make_carriage_arms(sd))
    # Holes come out LAST. Cut into the bare block and then fused over, the
    # arm's root disc grows straight back across the y = 46.5 bolt pair — a hole
    # that closes again is invisible to every interference check in this file,
    # so there is one below that probes them.
    for by in bolt_y:
        for bz in (bolt_z, -bolt_z):
            body = body.cut(pin_x((by, bz), fdm_bolt_hole_d,
                                  x0 - 1.0, (x1 - x0) + 2.0))
    return body.cut(trunnion_bore(sd, x_out))


def make_carriage_arms(sd):
    """The two arms on one side, from the block out to the four-bar pivots.
    Routed around the output cone — see arm_root."""
    x0 = sd * side_x - side_t / 2.0
    part = None
    for zs in (1, -1):
        b = (fb_B[0], zs * fb_B[1])
        arm = bar_yz((arm_root[0], zs * arm_root[1]), b, arm_w, x0, side_t)
        arm = arm.fuse(disc_yz(b, arm_w / 2.0 + 0.5, x0, side_t))
        arm = arm.cut(pin_x(b, fdm_pin_hole_d, x0 - 1, side_t + 2))
        part = arm if part is None else part.fuse(arm)
    return part


def make_clamp_bolt(by, bz):
    """One M3 through both halves, head outboard on -X and nut on +X. An
    envelope, not a thread — enough for the interference check to do its job."""
    shank = cyl(bolt_d / 2.0, 2 * hous_x + bolt_nut_h + 2.0,
                v(-hous_x - 1.0, by, bz), X_AXIS)
    head = cyl(bolt_head_d / 2.0, bolt_head_h,
               v(-hous_x - bolt_head_h, by, bz), X_AXIS)
    nut = cyl(bolt_nut_d / 2.0, bolt_nut_h, v(hous_x, by, bz), X_AXIS)
    return shank.fuse(head).fuse(nut)


def make_trunnion(sd):
    """Push pin: a length of the Ø5 rod the project already buys, pressed into a
    blind hole in each half — deliberately not a printed boss. Laid on its cut
    face the half prints beautifully, but a boss here becomes a vertical pillar
    and the actuator load bends it ACROSS the layers, the one direction FDM has
    no strength in: Ø5 over an 8 mm overhang at the top of the force range is
    ~58 MPa of interlayer tension, against 30-40 MPa for PLA.

    Both halves carry one, which is what keeps them the same part and is also
    the fork mounting the doc wants. The link in this model still uses only the
    +X pin, so the twisting moment of F * act_x about the shaft axis stays until
    the link becomes a fork."""
    lo, hi = sorted((sd * trunnion_x_in, sd * (act_x + link_t / 2.0 + 0.5)))
    return cyl(trunnion_d / 2.0, hi - lo, v(lo, R_push, 0.0), X_AXIS)


def trunnion_bore(sd, x_out):
    """The blind hole the dowel presses into, from the half's outer face in."""
    lo, hi = sorted((sd * trunnion_x_in, x_out + sd * 1.0))
    return cyl(fdm_dowel_hole_d / 2.0, hi - lo, v(lo, R_push, 0.0), X_AXIS)


def make_carriage_bearing(y_face, sd):
    y_start = min(y_face, y_face + sd * brg_w)
    outer = cyl(brg_od / 2.0, brg_w, v(0, y_start, 0), Y_AXIS)
    return outer.cut(cyl(brg_id / 2.0 + 0.05, brg_w + 2,
                         v(0, y_start - 1, 0), Y_AXIS))


def make_link(A, B):
    """One four-bar link — ONE part carrying both arms, not two loose plates.

    The two arms at +-link_x are the same rigid body (they move identically, by
    construction), so joining them costs nothing, makes the pair self-squaring,
    and actually delivers the out-of-plane stiffness that having two of them was
    for. They meet at a knuckle on the carriage pivot — see link_knuckle for why
    that is the only place it fits."""
    body = None
    for xs in (1, -1):
        arm = bar_yz(A, B, link_w, xs * link_x - link_t / 2.0, link_t)
        body = arm if body is None else body.fuse(arm)
    # Overlaps each arm by 1 mm rather than meeting it on a coincident face,
    # which fuses far more reliably.
    web_x = link_x - link_t / 2.0 + 1.0
    body = body.fuse(disc_yz(B, link_knuckle, -web_x, 2 * web_x))

    reach = link_x + link_t / 2.0 + 2.0
    for q in (A, B):
        body = body.cut(pin_x(q, fdm_pin_hole_d, -reach, 2 * reach))
    return body


def make_frame_bracket(zs):
    """Frame-pivot bracket: a plate reaching in from the wall, with a lug
    hanging down to the pin axis. The frame pivots sit at |z| ~ 44, past where
    the motor cones end, so they need structure of their own (doc §10.5) — this
    is the first pass at it, and it is what keeps the servo's space clear
    (a full deck plate at this height would run into the servo body)."""
    z_pin = zs * fb_A[1]
    z_in = z_pin + zs * fp_plate_gap
    z_out = z_in + zs * fp_plate_t
    plate = box_yz(fp_bracket_y0, cube_half, min(z_in, z_out), max(z_in, z_out),
                   -fp_bracket_x, 2 * fp_bracket_x)
    lug_far = z_pin - zs * (link_w / 2.0 + 1.0)
    lug = box_yz(fb_A[0] - 6.0, fb_A[0] + 6.0,
                 min(lug_far, z_out), max(lug_far, z_out),
                 -fp_lug_x, 2 * fp_lug_x)
    lug = lug.fuse(disc_yz((fb_A[0], z_pin), link_w / 2.0 + 1.0,
                           -fp_lug_x, 2 * fp_lug_x))
    # Foot turning inboard off the plate's end, so its screws pass through
    # material instead of into the end of a horizontal plate.
    z_f0, z_f1 = sorted((z_in, z_in - zs * foot_flange))
    foot = Part.makeBox(2 * fp_bracket_x, foot_t, z_f1 - z_f0,
                        v(-fp_bracket_x, cube_half - foot_t, z_f0))
    part = plate.fuse(lug).fuse(foot)
    part = part.cut(pin_x((fb_A[0], z_pin), fdm_pin_hole_d,
                          -fp_lug_x - 1, 2 * fp_lug_x + 2))
    for sx, sz in wall_screws():
        if abs(sx) < 13.0 and sz * zs > 0:
            part = part.cut(cyl(foot_hole_d / 2.0, foot_t + 2,
                                v(sx, cube_half - foot_t - 1, sz), Y_AXIS))
    return part


def make_idler_bracket(sd):
    """Fixed support for one idler. It cannot come from the corona side (the
    corona's back plate closes that face and rotates) nor down the middle (the
    output shaft is there), so it reaches in from the wall, passes outside the
    corona's rim, and turns inward in the 3 mm of Y between the carriage and
    the gear plane.

    It approaches from +Z, not from +-X. Coming in along X laid the arm across
    z = 0 at exactly the radius the actuation link needs on its way down to the
    crank; the space above the corona is empty."""
    x0 = sd * idler_x - idler_arm_w / 2.0
    y0 = idler_arm_y1 - idler_arm_t
    arm = Part.makeBox(idler_arm_w, idler_arm_t, idler_leg_z, v(x0, y0, 0.0))
    leg = Part.makeBox(idler_arm_w, cube_half - y0, idler_leg_t,
                       v(x0, y0, idler_leg_z - idler_leg_t))
    axle = cyl(idler_axle_d / 2.0, gear_face_w + 3.0,
               v(sd * idler_x, y0, 0), Y_AXIS)
    # Foot against the wall's inner face: a flange wide enough in X to put a
    # screw either side of the leg, rather than driving into the leg's own end
    # grain, which on a printed part is the weakest joint available.
    foot = Part.makeBox(idler_arm_w + 14.0, foot_t, idler_leg_t,
                        v(sd * idler_x - (idler_arm_w + 14.0) / 2.0,
                          cube_half - foot_t, idler_leg_z - idler_leg_t))
    body = arm.fuse(leg).fuse(axle).fuse(foot)
    for sx, sz in wall_screws():
        if abs(sx - sd * idler_x) < 12.0:
            body = body.cut(cyl(foot_hole_d / 2.0, foot_t + 2,
                                v(sx, cube_half - foot_t - 1, sz), Y_AXIS))
    return body


def make_corona_plate():
    return cyl(r_corona_outer, corona_plate_t, v(0, corona_plate_y, 0), Y_AXIS)


def make_output_shaft_ref():
    return cyl(shaft_d / 2.0, 40.0, v(0, corona_back_y, 0), Y_AXIS)


def make_crank(st):
    """Servo crank, dog-legged out to the actuation plane."""
    x_web = act_x + link_t / 2.0 + 0.5
    hub = cyl(6.0, 3.5, v(x_web, crank_hub[0], crank_hub[1]), X_AXIS)
    web = bar_yz(crank_hub, st["pin"], 8.0, x_web, 3.5)
    # Ø3, matching the 3.2 hole in the link. It was written as cyl(3.0, ...),
    # which is a RADIUS — a Ø6 pin through a Ø3.2 hole, 101 mm3 of overlap.
    pin = cyl(1.5, link_t + 4.0, v(act_x - link_t / 2.0 - 1.0,
                                   st["pin"][0], st["pin"][1]), X_AXIS)
    return hub.fuse(web).fuse(pin)


def make_act_link(st):
    """Telescopic link, pinned at both ends, with the springs inside. Drawn at
    its ACTUAL length (pin → trunnion as the linkage puts them), so the
    difference from act_link_len is the real spring compression — which is the
    number the preload is set by, not any position."""
    x0 = act_x - link_t / 2.0
    pin_pt, tab_pt = st["pin"], st["tab"]
    mid = ((pin_pt[0] + tab_pt[0]) / 2.0, (pin_pt[1] + tab_pt[1]) / 2.0)
    body = bar_yz(pin_pt, mid, 9.0, x0, link_t).fuse(
        bar_yz(mid, tab_pt, 5.0, x0, link_t))
    body = body.cut(pin_x(pin_pt, 3.2, x0 - 1, link_t + 2))
    body = body.cut(pin_x(tab_pt, trunnion_d + 0.4, x0 - 1, link_t + 2))
    return body


def servo_box():
    """(x0, y0, z0, bx, by, bz) of the servo body. Its top face — the one the
    shaft comes out of — is at x0, against the crank."""
    bx, by, bz = servo_body
    x0 = act_x + link_t / 2.0 + 0.5 + 3.5 + servo_spline_h
    return (x0, crank_hub[0] - by / 4.0, crank_hub[1] - bz + 5.0, bx, by, bz)


def make_servo_body():
    """MG90-class placeholder: body plus the two mounting tabs, which are what
    it actually hangs from."""
    x0, y0, z0, bx, by, bz = servo_box()
    body = Part.makeBox(bx, by, bz, v(x0, y0, z0))
    tabs = Part.makeBox(servo_tab_t, by + 2 * servo_tab_out, bz,
                        v(x0, y0 - servo_tab_out, z0))
    body = body.fuse(tabs)
    for sy in servo_screw_ys():
        body = body.cut(cyl(servo_screw_d / 2.0 + 0.1, servo_tab_t + 2,
                            v(x0 - 1, sy, crank_hub[1]), X_AXIS))
    return body


def servo_screw_ys():
    """Y of the two tab screws: an MG90's holes are 27.8 apart."""
    _, y0, _, _, by, _ = servo_box()
    return (y0 - servo_tab_out / 2.0, y0 + by + servo_tab_out / 2.0)


def make_servo_bracket():
    """What the servo hangs from, which until now was nothing at all.

    A face plate across the servo's tabs with the body passing through it, and a
    foot onto the wall — the only structure within reach, 5.5 mm past the end of
    the body. The obvious alternative, standing it on the bottom frame bracket,
    does not work: that deck's top face is at z = -50.9 and the servo's own base
    reaches -52, so the body wants the space the deck is in."""
    x0, y0, z0, bx, by, bz = servo_box()
    px0 = x0 + servo_tab_t
    plate = Part.makeBox(servo_plate_t, 34.5, 37.0,
                         v(px0, 34.0, z0 - 4.0))
    plate = plate.cut(Part.makeBox(servo_plate_t + 2, by + 0.8, bz + 0.8,
                                   v(px0 - 1, y0 - 0.4, z0 - 0.4)))
    # Outboard of the face plate, not inboard: the servo's own tabs run the
    # full height of the body and reach to y = 68, so a foot on that side of
    # the plate passes straight through them.
    foot = Part.makeBox(15.0, foot_t, 12.0,
                        v(px0, cube_half - foot_t, servo_foot_z - 6.0))
    part = plate.fuse(foot)
    for sx, sz in wall_screws():
        if abs(sx) > 25.0:
            part = part.cut(cyl(foot_hole_d / 2.0, foot_t + 2,
                                v(sx, cube_half - foot_t - 1, sz), Y_AXIS))
    for sy in servo_screw_ys():
        part = part.cut(cyl(servo_screw_d / 2.0 + 0.4, servo_plate_t + 2,
                            v(px0 - 1, sy, crank_hub[1]), X_AXIS))
    return part


def make_wall():
    """One cube wall. It stops at the neighbour's inner face on one side and
    runs out to the full corner on the other, so the four of them interlock in a
    pinwheel: no overlap and no gap, and the same part four times. Spanning the
    full width both ways -- which is what it did while the wall sat outside the
    interference check -- put 2320 mm3 of two walls inside each other, one
    corner post per corner."""
    wall = Part.makeBox(cube_out + cube_half, wall_thick, 2 * cube_out,
                        v(-cube_out, cube_half, -cube_out))
    wall = wall.cut(cyl(shaft_d / 2.0 + wall_shaft_clr, wall_thick + 2,
                        v(0, cube_half - 1, 0), Y_AXIS))
    for sx, sz in wall_screws():
        wall = wall.cut(cyl(foot_hole_d / 2.0, wall_thick + 2,
                            v(sx, cube_half - 1, sz), Y_AXIS))
    return wall


# ── Gears ───────────────────────────────────────────────────────────────────
def _attach_gear_view(obj):
    """freecad.gears' own command code attaches ViewProviderGear BEFORE the
    geometry proxy; skipping it leaves the object with no working ViewObject
    proxy and FreeCAD silently draws nothing."""
    if HAS_GUI and obj.ViewObject is not None:
        ViewProviderGear(obj.ViewObject)


def _gear_shape(doc, kind, teeth, **kw):
    """Build a gear with identity placement, take its shape, drop the
    parametric object (leaving it in the tree would read as a duplicate part,
    and editing it there would not change the copies)."""
    obj = doc.addObject("Part::FeaturePython", "TmpGear")
    _attach_gear_view(obj)
    (InternalInvoluteGear if kind == "internal" else InvoluteGear)(obj)
    obj.num_teeth = teeth
    obj.module = f"{m_mod} mm"
    obj.height = f"{gear_face_w} mm"
    for k, val in kw.items():       # a caller may override height (the pinion)
        setattr(obj, k, val)
    doc.recompute()
    shape = obj.Shape.copy()
    doc.removeObject(obj.Name)
    return shape


def _to_gear_plane(shape, cx, cz):
    """Local +Z (the extrusion axis) onto global +Y, then to the gear plane."""
    s = shape.copy()
    s.rotate(ORIGIN, X_AXIS, -90.0)
    s.translate(v(cx, gear_y0, cz))
    return s


def tooth_phase(shape, cx, cz, r_probe, n_teeth):
    """Global polar angle (deg, from +X toward +Z) of a tooth centre, found by
    probing the solid rather than trusting the addon's internal convention.
    Returns None if the probe radius misses the teeth entirely."""
    pitch = 360.0 / n_teeth
    n = 240
    step = 2.0 * pitch / n
    y = gear_y_mid
    hits = []
    for i in range(n):
        a = math.radians(i * step)
        p = v(cx + r_probe * math.cos(a), y, cz + r_probe * math.sin(a))
        hits.append(shape.isInside(p, 1e-7, True))
    runs, start = [], None
    for i, h in enumerate(hits):
        if h and start is None:
            start = i
        elif not h and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, n - 1))
    for a, b in runs:
        if a > 0 and b < n - 1:
            return (a + b) / 2.0 * step
    return None


def phase_gear(shape, cx, cz, r_probe, n_teeth, target_deg):
    """Rotate a gear about its own axis so a tooth centre lands on
    target_deg. Rotating by +psi about +Y DECREASES the global polar angle."""
    found = tooth_phase(shape, cx, cz, r_probe, n_teeth)
    if found is None:
        return shape, None
    psi = found - target_deg
    shape.rotate(v(cx, gear_y_mid, cz), Y_AXIS, psi)
    return shape, found


# ═══════════════════════════════════════════════════════════════════
# FLAT PATTERN  (rubber cutting template, 1:1)
# ═══════════════════════════════════════════════════════════════════
# A cone is developable, so the rubber layer unrolls into a flat annular sector
# of arc 2*pi*sin(half-angle) — 295 deg for the motor cone, not 360. The missing
# wedge is what the seam closes, which is why a seam cannot be purely
# circumferential however much one would like it to be.
#
# The two mating edges only have to be the SAME curve offset by the sector arc,
# so they can be spirals rather than straight radii. A curve at a constant angle
# to the generatrix develops into a logarithmic spiral (the development is an
# isometry, so angles survive it), and a seam lying nearly circumferential
# crosses the contact line at a single point that SWEEPS along it as the cone
# turns, instead of the whole line landing on the seam once per revolution.


def seam_spiral(half_angle):
    """(sector arc, spiral constant, angle to the generatrix, seam length) for
    a seam that sweeps exactly one turn of the cone across the band."""
    arc = 2.0 * math.pi * math.sin(half_angle)
    k = arc / math.log(s_rub_hi / s_rub_lo)
    psi = math.atan(k)
    return arc, k, psi, (s_rub_hi - s_rub_lo) / math.cos(psi)


def _svg_pattern(cx, cy, half_angle, label, qty):
    """One cone's pattern: outline, seam bevel marks, and the circle where the
    contact patch actually starts."""
    arc, k, psi, seam_len = seam_spiral(half_angle)
    N = 180

    def pt(r, th):
        return (cx + r * math.cos(th), cy - r * math.sin(th))

    def edge(offset, outward=True):
        rng = range(N + 1) if outward else range(N, -1, -1)
        out = []
        for i in rng:
            r = s_rub_lo + (s_rub_hi - s_rub_lo) * i / N
            out.append(pt(r, k * math.log(r / s_rub_lo) + offset))
        return out

    pts = edge(0.0, True)
    th_end = k * math.log(s_rub_hi / s_rub_lo)
    pts += [pt(s_rub_hi, th_end + arc * i / N) for i in range(1, N + 1)]
    pts += edge(arc, False)
    pts += [pt(s_rub_lo, arc * (1.0 - i / N)) for i in range(1, N)]

    body = ['<polygon points="%s" fill="none" stroke="#000" stroke-width="0.3"/>'
            % " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)]

    # Scarf marks: the perpendicular distance between two spirals a small angle
    # apart is r*dtheta*cos(psi), so the angular offset grows as the spiral
    # shallows and as r shrinks.
    for off, sign in ((0.0, +1), (arc, -1)):
        line = []
        for i in range(N + 1):
            r = s_rub_lo + (s_rub_hi - s_rub_lo) * i / N
            dth = sign * scarf_w / (r * math.cos(psi))
            line.append(pt(r, k * math.log(r / s_rub_lo) + off + dth))
        body.append('<polyline points="%s" fill="none" stroke="#c00"'
                    ' stroke-width="0.2" stroke-dasharray="2,1.5"/>'
                    % " ".join(f"{x:.2f},{y:.2f}" for x, y in line))

    if _sq_lo is not None:
        body.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{_sq_lo:.2f}"'
                    ' fill="none" stroke="#08a" stroke-width="0.2"'
                    ' stroke-dasharray="1,2"/>')
    for i, txt in enumerate((f"{label} x{qty}",
                             f"t = {t_rubber:.1f} mm   seam"
                             f" {math.degrees(psi):.0f}° to the generatrix,"
                             f" {seam_len:.0f} mm")):
        body.append(f'<text x="{cx:.1f}" y="{cy + s_rub_hi + 6 + 4.5 * i:.1f}"'
                    f' font-family="sans-serif" font-size="3.2"'
                    f' text-anchor="middle">{txt}</text>')
    # Shoelace area against the analytic band area: the outline is built from
    # two spirals and two arcs, and a sign or offset slip in any of them shows
    # up here and nowhere else.
    area = abs(sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1]
                   for i in range(len(pts)))) / 2.0
    exact = arc / 2.0 * (s_rub_hi ** 2 - s_rub_lo ** 2)
    return body, area, exact


def write_flat_pattern(path):
    """1:1 SVG cutting template. Regenerated on every run from the same
    parameters as the solids, so it cannot drift out of step with them."""
    pad, gap = 12.0, 16.0
    w = 4 * s_rub_hi + 2 * pad + gap
    h = 2 * s_rub_hi + 2 * pad + 26
    cx1 = pad + s_rub_hi
    cx2 = cx1 + 2 * s_rub_hi + gap
    cy = pad + s_rub_hi
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.1fmm"'
           ' height="%.1fmm" viewBox="0 0 %.1f %.1f">' % (w, h, w, h)]
    err = 0.0
    for _cx, _ha, _lbl, _qty in ((cx1, alpha, "MOTOR CONE", 2),
                                 (cx2, beta, "OUTPUT CONE", 4)):
        body, area, exact = _svg_pattern(_cx, cy, _ha, _lbl, _qty)
        out += body
        err = max(err, abs(area - exact) / exact)
    # Scale bar: printers lie, and a pattern that prints at 97% is worse than
    # no pattern at all.
    y0 = h - 18.0
    out.append(f'<path d="M{pad:.1f},{y0:.1f} h50 m0,-2 v4 m-50,0 v-4"'
               ' stroke="#000" stroke-width="0.3" fill="none"/>')
    for i, txt in enumerate(("50 mm — measure it on the print before cutting.",
                             "Solid = cut.  Red dashed = scarf this much of the"
                             " edge, opposite faces on the two edges.",
                             "Blue = where the contact patch starts; inside it"
                             " the rubber never touches.")):
        out.append(f'<text x="{pad:.1f}" y="{y0 + 6.0 + 3.4 * i:.1f}"'
                   ' font-family="sans-serif" font-size="2.8">'
                   f'{txt}</text>')
    out.append('</svg>')
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out))
    return path, err


# ═══════════════════════════════════════════════════════════════════
# BUILD DOCUMENT
# ═══════════════════════════════════════════════════════════════════

doc_name = "Motcore_v6"
if App.listDocuments().get(doc_name):
    App.closeDocument(doc_name)
doc = App.newDocument(doc_name)

# ── Gear stage ──────────────────────────────────────────────────────────────
# Phasing, in one place because it is a chain: pinion tooth on the line of
# centres → idler gap facing back at it → corona gap rolled by Zi/Zc from the
# reference. Both idlers can share one phase only because 180 deg is a whole
# number of tooth pitches on the pinion and on the corona (Zp, Zc even).
gear_phase_ok = True
if GEARS_AVAILABLE:
    pinion_shape = _gear_shape(doc, "external", Zp, axle_hole=True,
                               axle_holesize=f"{fdm_shaft_hole_d} mm",
                               height=f"{pinion_face_w} mm")
    idler_shape = _gear_shape(doc, "external", Zi, axle_hole=True,
                              axle_holesize=f"{idler_axle_d + 0.6} mm")
    corona_shape = _gear_shape(doc, "internal", Zc,
                               thickness=f"{corona_rim_t} mm")
    pinion_shape = _to_gear_plane(pinion_shape, 0.0, 0.0)
    corona_shape = _to_gear_plane(corona_shape, 0.0, 0.0)
    idler_pos = _to_gear_plane(idler_shape, idler_x, 0.0)
    idler_neg = _to_gear_plane(idler_shape, -idler_x, 0.0)

    pinion_shape, _p = phase_gear(pinion_shape, 0.0, 0.0, r_pitch_p, Zp, 0.0)
    idler_pos, _i = phase_gear(idler_pos, idler_x, 0.0, r_pitch_i, Zi,
                               180.0 + 180.0 / Zi)
    idler_neg, _j = phase_gear(idler_neg, -idler_x, 0.0, r_pitch_i, Zi,
                               180.0 + 180.0 / Zi)
    corona_shape, _c = phase_gear(corona_shape, 0.0, 0.0, r_pitch_c, Zc, 0.0)
    gear_phase_ok = None not in (_p, _i, _j, _c)
else:
    pinion_shape = cyl(r_tip_p, pinion_face_w, v(0, gear_y0, 0), Y_AXIS)
    corona_shape = (cyl(r_corona_outer, gear_face_w, v(0, gear_y0, 0), Y_AXIS)
                    .cut(cyl(r_tip_c, gear_face_w + 2, v(0, gear_y0 - 1, 0), Y_AXIS)))
    idler_pos = cyl(r_tip_i, gear_face_w, v(idler_x, gear_y0, 0), Y_AXIS)
    idler_neg = cyl(r_tip_i, gear_face_w, v(-idler_x, gear_y0, 0), Y_AXIS)

# Corona + back plate + output shaft: one part, as in v5 — a hub joint here
# would only add a failure point on the torque path.
coronashaft = corona_shape.fuse(make_corona_plate()).fuse(make_output_shaft_ref())

# ── Carriage, built at rest then moved by the linkage ────────────────────────
CARRIAGE_REST = [
    ("OutputCone",    make_output_cone(),                     (0.20, 0.80, 0.60), 0),
    ("OutputRubber",  make_output_rubber(),                   (0.15, 0.15, 0.18), 0),
    ("CarriageHalfP", make_carriage_half(1),                  (0.70, 0.70, 0.72), 0),
    ("CarriageHalfN", make_carriage_half(-1),                 (0.72, 0.70, 0.70), 0),
    ("PushPinP",      make_trunnion(1),                       (0.60, 0.60, 0.60), 0),
    ("PushPinN",      make_trunnion(-1),                      (0.60, 0.60, 0.60), 0),
] + [
    (f"ClampBolt{i}", make_clamp_bolt(by, bz), (0.45, 0.45, 0.50), 0)
    for i, (by, bz) in enumerate((y, z) for y in bolt_y for z in (bolt_z, -bolt_z))
] + [
    ("CarriageShaft", make_carriage_shaft(),                  (0.60, 0.60, 0.60), 0),
    ("Pinion",        pinion_shape,                           (0.85, 0.65, 0.10), 0),
    ("BearingCone",   make_carriage_bearing(hous_y0, 1),      (0.30, 0.30, 0.32), 0),
    ("BearingPinion", make_carriage_bearing(hous_y1, -1),     (0.30, 0.30, 0.32), 0),
]

# ── Fixed to the frame ──────────────────────────────────────────────────────
FIXED_PARTS = [
    ("CoronaShaft",   coronashaft,                            (0.55, 0.55, 0.85), 0),
    ("IdlerPosX",     idler_pos,                              (0.30, 0.55, 0.85), 0),
    ("IdlerNegX",     idler_neg,                              (0.30, 0.55, 0.85), 0),
    ("IdlerBracketP", make_idler_bracket(1),                  (0.65, 0.65, 0.68), 0),
    ("IdlerBracketN", make_idler_bracket(-1),                 (0.65, 0.65, 0.68), 0),
    ("FrameBracketT", make_frame_bracket(1),                  (0.75, 0.75, 0.78), 0),
    ("FrameBracketB", make_frame_bracket(-1),                 (0.75, 0.75, 0.78), 0),
    ("ServoBody",     make_servo_body(),                      (0.20, 0.25, 0.30), 0),
    ("ServoBracket",  make_servo_bracket(),                   (0.75, 0.75, 0.78), 0),
    ("Wall",          make_wall(),                            (0.45, 0.55, 0.75), 70),
] + [
    (f"ServoScrew{i}",
     make_screw(v(servo_box()[0], sy, crank_hub[1]), v(1, 0, 0), servo_screw_d,
                servo_tab_t + servo_plate_t, 3.8, 2.0),
     (0.45, 0.45, 0.50), 0)
    for i, sy in enumerate(servo_screw_ys())
] + [
    (f"WallScrew{i}",
     make_screw(v(sx, cube_out, sz), v(0, -1, 0), foot_screw_d,
                wall_thick + foot_t, bolt_head_d, bolt_head_h),
     (0.45, 0.45, 0.50), 0)
    for i, (sx, sz) in enumerate(wall_screws())
]

_LINK_COL = (0.90, 0.45, 0.20)
_PIN_COL = (0.45, 0.45, 0.50)
_ACT_COL = (0.30, 0.55, 0.85)


def moving_parts(st):
    """Every part whose position depends on the tilt: the carriage, the four
    links with their pins, and the actuation train."""
    out = [(n, place_carriage(sh, st), c, t) for n, sh, c, t in CARRIAGE_REST]
    pin_reach = link_x + link_t / 2.0 + 1.0
    pin_reach_b = side_x + side_t / 2.0 + 1.0
    for zs, A, B in ((1, A1, st["B1"]), (-1, A2, st["B2"])):
        tag = "T" if zs > 0 else "B"
        out.append((f"Link{tag}", make_link(A, B), _LINK_COL, 0))
        out.append((f"PinA{tag}",
                    pin_x(A, pin_d, -pin_reach, 2 * pin_reach), _PIN_COL, 0))
        # One pin per link now, right through both carriage arms, both link
        # arms and the web, instead of a short stub on each side.
        out.append((f"PinB{tag}",
                    pin_x(B, pin_d, -pin_reach_b, 2 * pin_reach_b),
                    _PIN_COL, 0))
    out.append(("Crank", make_crank(st), _ACT_COL, 0))
    out.append(("ActLink", make_act_link(st), _ACT_COL, 0))
    return out


STATE = pose_state(phi)
AXIS_PARTS = moving_parts(STATE) + FIXED_PARTS
BY_NAME = {n: sh for n, sh, c, t in AXIS_PARTS}

act_len_now = math.dist(STATE["pin"], STATE["tab"])
spring_comp = act_link_len - act_len_now

AXES = [("PosY", 0.0), ("NegX", 90.0), ("NegY", 180.0), ("PosX", 270.0)]


def place(shape, rot_deg):
    """A copy of `shape` rotated onto its wall. Placement-level rotation only:
    never transformGeometry (it distorts the freecad.gears corona) and never
    mirror (it composes wrongly with the Placement the gears carry — use a
    180 deg rotation, which is also the physically right thing: the flipped
    part is the SAME printed part, not a chiral second one). Always from a
    fresh .copy(): Shape.rotate mutates in place."""
    s = shape.copy()
    if rot_deg:
        s.rotate(ORIGIN, Z_AXIS, rot_deg)
    return s


for _ax_name, _ax_rot in AXES[:AXES_SHOWN]:
    for _pname, _pshape, _pcolor, _ptrans in AXIS_PARTS:
        add(doc, f"{_pname}_{_ax_name}", place(_pshape, _ax_rot),
            color=_pcolor, transparency=_ptrans)

# ── Central column ──────────────────────────────────────────────────────────
for _sd, _tag in ((-1, "Lower"), (1, "Upper")):
    add(doc, f"MotorCone{_tag}", make_motor_cone(_sd), color=(1.0, 0.60, 0.15))
    add(doc, f"MotorRubber{_tag}", make_motor_rubber(_sd), color=(0.15, 0.15, 0.18))

_ms_reach = mot_base_z + 15
add(doc, "MotorShaft", cyl(shaft_d / 2.0, 2 * _ms_reach, v(0, 0, -_ms_reach)),
    color=(0.6, 0.6, 0.6))

doc.recompute()

# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════
# Booleans and distances, not "it did not raise". Geometry this macro gets
# wrong comes out as a part in the wrong place, never as an exception.

_mc_lo, _mc_up = make_motor_cone(-1), make_motor_cone(1)
_mr_lo, _mr_up = make_motor_rubber(-1), make_motor_rubber(1)
_oc_now = BY_NAME["OutputCone"]
_or_now = BY_NAME["OutputRubber"]

# Rubber, at the rendered pose: which side is engaged, and by how much.
_rub_lo = _or_now.common(_mr_lo).Volume
_rub_up = _or_now.common(_mr_up).Volume
_gap_lo = _or_now.distToShape(_mr_lo)[0]
_gap_up = _or_now.distToShape(_mr_up)[0]
_plastic_ov = _oc_now.common(_mc_lo).Volume + _oc_now.common(_mc_up).Volume

# At FREE the rubber must be clear of both cones (that IS the free state).
_or_free = make_output_rubber()
_free_gap = min(_or_free.distToShape(_mr_lo)[0], _or_free.distToShape(_mr_up)[0])


def _rubber_gap(phi_t):
    """Real distance between the rubber surfaces at tilt phi_t, on the engaged
    side — measured on the solids at the four-bar's actual pose, not on the
    ideal rotation. Also returns how far from the apex the closest point sits,
    which says whether the band meets at its inner or its outer end."""
    moved = place_carriage(make_output_rubber(), pose_state(phi_t))
    dist, pts, _ = moved.distToShape(_mr_up if phi_t > 0 else _mr_lo)
    s_at = math.hypot(pts[0][0].y, pts[0][0].z) if pts else float("nan")
    return dist, s_at


# Where does contact ACTUALLY happen? phi_c is the pure-rotation answer. The
# four-bar's drift is almost entirely along +Y — the output cone backs away from
# the apex — and backing a cone off along its own axis moves its surface away
# from the motor cone by drift*sin(beta). That OPENS the contact, and because
# the drift grows roughly as phi^2 while the tilt closes the gap linearly, the
# two nearly cancel: contact is late, and it starts at the OUTER end of the band
# (where the closing rate s*dphi is biggest) instead of along the whole line.
# Bisected on the solids rather than extrapolated — a two-sample linear fit gets
# this badly wrong, for exactly the quadratic reason above.
def _rubber_squeeze(phi_t):
    """Overlap solid between the two rubber layers at phi_t: its volume, and
    how much of the designed contact band it actually covers. Interference
    volume stands in for squeeze because the layers are modelled rigid."""
    moved = place_carriage(make_output_rubber(), pose_state(phi_t))
    lump = moved.common(_mr_up if phi_t > 0 else _mr_lo)
    if lump.Volume < 1e-9:
        return 0.0, None, None, float("nan")
    # Edges discretised, not just vertices: the overlap is a sliver bounded by
    # two smooth tangential surfaces, so its only VERTICES sit on the band's
    # trimmed rim and reading those alone says "a point at s = 28", which is
    # wrong — the patch runs a good way inboard from there.
    ss = []
    for e in lump.Edges:
        for pt in e.discretize(24):
            ss.append(math.hypot(pt.y, pt.z))
    # common() hands back a Compound, which has no CenterOfMass of its own:
    # weight the solids' centroids by their volumes.
    tot = sum(sol.Volume for sol in lump.Solids) or 1.0
    cy = sum(sol.CenterOfMass.y * sol.Volume for sol in lump.Solids) / tot
    cz = sum(sol.CenterOfMass.z * sol.Volume for sol in lump.Solids) / tot
    return lump.Volume, min(ss), max(ss), math.hypot(cy, cz)


_sq_vol, _sq_lo, _sq_hi, _sq_sbar = _rubber_squeeze(-phi_preload)
_lo_phi, _hi_phi = phi_c, 2.5 * phi_c
_hi_gap, _hi_s = _rubber_gap(-_hi_phi)
if _hi_gap > 1e-6:
    phi_contact_real, contact_s = None, float("nan")
else:
    for _ in range(14):
        _mid = (_lo_phi + _hi_phi) / 2.0
        if _rubber_gap(-_mid)[0] > 1e-6:
            _lo_phi = _mid
        else:
            _hi_phi = _mid
    phi_contact_real = (_lo_phi + _hi_phi) / 2.0
    contact_s = _rubber_gap(-phi_contact_real * 1.02)[1]

# EVERY pair of parts, at every stop. The earlier version of this check only
# compared carriage parts against fixed ones, which left two whole categories
# unchecked — carriage parts against each other (the output cone IS a carriage
# part, and the arms were cutting into it) and the actuation train against
# anything at all (its link was inside the corona). Bounding boxes first, so
# the ~200 pairs cost three booleans, not two hundred.
#
# Only the meshing gear pairs are exempt: at a tilt the pinion rolls to
# whatever phase the mesh needs, so a solid overlap there is an artefact of
# drawing it at its free-pose phase. The mesh itself is checked at free, where
# the phasing is exact.
_MESH_PAIRS = {("Pinion", "IdlerPosX"), ("Pinion", "IdlerNegX")}


def _bb_hit(a, b):
    A, B = a.BoundBox, b.BoundBox
    return (A.XMin < B.XMax and B.XMin < A.XMax
            and A.YMin < B.YMax and B.YMin < A.YMax
            and A.ZMin < B.ZMax and B.ZMin < A.ZMax)


_pair_ov, _pair_who = 0.0, "-"
_carr_mot_ov = 0.0
for _tag, _phi_t in (("free", 0.0), ("up", phi_preload), ("dn", -phi_preload)):
    _parts = moving_parts(pose_state(_phi_t)) + FIXED_PARTS
    for _i in range(len(_parts)):
        _na, _sa = _parts[_i][0], _parts[_i][1]
        for _j in range(_i + 1, len(_parts)):
            _nb, _sb = _parts[_j][0], _parts[_j][1]
            if (_na, _nb) in _MESH_PAIRS or (_nb, _na) in _MESH_PAIRS:
                continue
            if not _bb_hit(_sa, _sb):
                continue
            _ov = _sa.common(_sb).Volume
            if _ov > max(_pair_ov, 1e-6):
                _pair_ov, _pair_who = _ov, f"{_na} x {_nb} at {_tag}"
        if _bb_hit(_sa, _mc_up) or _bb_hit(_sa, _mc_lo):
            _carr_mot_ov = max(_carr_mot_ov,
                               _sa.common(_mc_up).Volume,
                               _sa.common(_mc_lo).Volume)

# Gear mesh, checked at FREE where the phasing is exact. Under tilt the pinion
# simply rolls to whatever phase the mesh needs, so an overlap measured there
# would be an artefact, not a collision — what tilt really costs is centre
# distance, reported separately below.
_pin_free = pinion_shape
_mesh_ext = _pin_free.common(idler_pos).Volume + _pin_free.common(idler_neg).Volume
_mesh_int = (corona_shape.common(idler_pos).Volume
             + corona_shape.common(idler_neg).Volume)
_pin_corona_gap = _pin_free.distToShape(corona_shape)[0]

# Pinion travel under tilt: the idlers are on X for a reason (invariant 6).
_pin_c_max = carriage_transform(phi_preload)[0]((gear_y_mid, 0.0))
_e_tilt = math.hypot(idler_x, _pin_c_max[1])
_e_growth = _e_tilt - e_ext
_e_if_z = abs(idler_x - _pin_c_max[1])     # what an idler ON Z would have seen

# Adjacent axes, full assemblies 90 deg apart.
_asm = AXIS_PARTS[0][1].copy()
for _n, _s, _c, _t in AXIS_PARTS[1:]:
    _asm = _asm.fuse(_s)
_asm_rot = _asm.copy()
_asm_rot.rotate(ORIGIN, Z_AXIS, 90.0)
_adj_overlap = _asm.common(_asm_rot).Volume

# Can a bolt actually pass? A hole that a later fuse closes back up leaves a
# perfectly valid solid of the right volume in the right place, so nothing else
# here would notice.
_halves = [BY_NAME["CarriageHalfP"], BY_NAME["CarriageHalfN"]]
_bolt_blocked = 0.0
for _by in bolt_y:
    for _bz in (bolt_z, -bolt_z):
        _probe = pin_x((_by, _bz), fdm_bolt_hole_d - 0.4,
                       -hous_x - 2.0, 2 * hous_x + 4.0)
        _probe = place_carriage(_probe, STATE)
        for _h in _halves:
            _bolt_blocked += _probe.common(_h).Volume

# The two halves have to be the SAME printed part, not a chiral pair: rotating
# one 180 deg about the shaft axis must land it exactly on the other. That holds
# only while the part stays symmetric in Z, which the trunnion on both faces and
# the arms above and below currently make true — and which a single asymmetric
# feature would quietly destroy.
_half_p = make_carriage_half(1)
_half_n = make_carriage_half(-1)
_half_rot = _half_p.copy()
_half_rot.rotate(ORIGIN, Y_AXIS, 180.0)
_half_diff = (_half_rot.cut(_half_n).Volume + _half_n.cut(_half_rot).Volume)

# Output cone tip vs the motor shaft it points at.
_tip_clr = out_tip_y - shaft_d / 2.0

# Can the crank and link actually reach every stop? Swept rather than checked at
# the three stops alone, because it is the ENDS of the stroke that fail first.
# A silent fallback here once hid a linkage that could not reach its own travel.
_reach = []
for _k in range(41):
    _z = -act_amp + 2.0 * act_amp * _k / 40.0
    _d = math.dist(crank_hub, (R_push, _z))
    _reach.append(min(act_link_len + crank_r - _d, _d - (act_link_len - crank_r)))
_reach_margin = min(_reach)

# Every shape in the tree is a valid solid. A boolean that half-failed leaves a
# shape that still draws and still has a volume, so this is not free.
_invalid = [o.Name for o in doc.Objects if not o.Shape.isValid()]

# And every part is ONE solid. Fusing shapes that do not touch is silent: the
# result is valid, has the right volume, draws correctly and is not a part.
# The carriage was exactly that — housing plus two side plates, fused, with
# nothing between them (the web that now joins them did not exist).
_loose = [n for n, sh, c, t in AXIS_PARTS if len(sh.Solids) != 1]
_loose += [n for n, sh in (("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up),
                           ("MotorRubberLower", _mr_lo), ("MotorRubberUpper", _mr_up))
           if len(sh.Solids) != 1]

checks = [
    ("ratio_total < 1  (reduction, never 1:1)",
     ratio_total, "< 1", ratio_total < 1.0),
    ("free gap angle phi_c = 90 - alpha - beta  (deg)",
     math.degrees(phi_c), "> 0", phi_c > 0),
    ("Zc = Zp + 2*Zi  (idlers can centre the output shaft)",
     Zc - Zp - 2 * Zi, "== 0", Zc == Zp + 2 * Zi),
    ("Zc - Zp >= 8  (internal mesh interference)",
     Zc - Zp, ">= 8", (Zc - Zp) >= 8),
    ("Zp, Zc even  (one idler phase serves both sides)",
     (Zp % 2) + (Zc % 2), "== 0", Zp % 2 == 0 and Zc % 2 == 0),
    ("link axes converge on the apex  (deg between the two rays)",
     link_axis_err, "< 0.2", link_axis_err < 0.2),
    ("link axis outside the cone wedge  (beta..90-alpha = "
     f"{beta_deg:.0f}..{90-alpha_deg:.0f} deg)",
     link_axis_deg, f"not in {beta_deg:.0f}..{90-alpha_deg:.0f}",
     not (beta_deg <= link_axis_deg <= 90 - alpha_deg)),
    ("pinion hub wall  (root radius - shaft bore radius, mm)",
     pinion_hub_wall, ">= 1.5", pinion_hub_wall >= 1.5),
    ("output cone blind-bore depth  (mm)",
     out_bore_depth, ">= 15", out_bore_depth >= 15.0),
    ("output cone tip clears the motor shaft  (mm)",
     _tip_clr, "> 1.0", _tip_clr > 1.0),
    ("clamp bolts have a clear path through both halves  (mm3)",
     _bolt_blocked, "== 0", _bolt_blocked < 1e-6),
    ("the two carriage halves are the SAME part, 180 deg apart  (mm3)",
     _half_diff, "== 0", _half_diff < 1e-3),
    ("bearing shoulder inside the housing  (mm)",
     hous_y1 - hous_y0 - 2 * brg_w, "> 0", hous_y1 - hous_y0 - 2 * brg_w > 0),
    ("crank + link reach the whole stroke  (mm to dead centre)",
     _reach_margin, "> 0.1", _reach_margin > 0.1 and not _UNREACHABLE),
    ("FREE: rubber clear of both motor cones  (mm)",
     _free_gap, "> 0.10", _free_gap > 0.10),
    ("plastic never touches plastic  (mm3)",
     _plastic_ov, "== 0", _plastic_ov < 1e-6),
    (f"every pair of parts, every stop [{_pair_who}]  (mm3)",
     _pair_ov, "== 0", _pair_ov < 1e-6),
    ("nothing on the axis touches the motor cones  (mm3)",
     _carr_mot_ov, "== 0", _carr_mot_ov < 1e-6),
    ("pinion concentric inside the corona, no mesh  (mm)",
     _pin_corona_gap, "> 1.0", _pin_corona_gap > 1.0),
    ("pinion/idler mesh at FREE, no jam  (mm3)",
     _mesh_ext, "== 0", _mesh_ext < 1e-6),
    ("idler/corona mesh at FREE, no jam  (mm3)",
     _mesh_int, "== 0", _mesh_int < 1e-6),
    (f"every built shape is a valid solid  {_invalid if _invalid else ''}",
     len(_invalid), "== 0", not _invalid),
    (f"every part is ONE connected solid  {_loose if _loose else ''}",
     len(_loose), "== 0", not _loose),
    ("adjacent axes 90 deg apart, full assemblies  (mm3)",
     _adj_overlap, "== 0", _adj_overlap < 1e-6),
]

if HAS_GUI:
    try:
        Gui.ActiveDocument = Gui.getDocument(doc.Name)
        Gui.SendMsgToActiveView("ViewFit")
        Gui.activeDocument().activeView().viewIsometric()
    except AttributeError:
        pass   # freecadcmd: no real view, geometry is already built

# ═══════════════════════════════════════════════════════════════════
# REPORT
# ═══════════════════════════════════════════════════════════════════
print("=" * 72)
print(f"Motcore v6 — Apex Pivot ({len(AXES)} bidirectional axes, {AXES_SHOWN} built)")
print(f"  alpha = {alpha_deg:.1f} deg (motor)   beta = {beta_deg:.1f} deg (output)"
      f"   rubber t = {t_rubber:.1f} mm")
print(f"  Friction ratio {ratio_fric:.3f} x gear ratio {ratio_gear:.3f}"
      f"  =  {ratio_total:.3f}  (-> x{1/ratio_total:.2f} torque)")
print(f"  Plastic apex offsets (invariant 2): motor {apex_off_motor:.3f} mm,"
      f" output {apex_off_output:.3f} mm")
print(f"  Contact band on the rubber surface: s = {s_rub_lo:.1f}..{s_rub_hi:.1f} mm"
      f" from the apex  (L = {L_line:.1f})")
print(f"  L is DERIVED: the band fills the cone, stopping {rubber_lip:.1f} mm"
      f" short of the rim so the wrapped")
print(f"  sheet has a lip to bond to. Cone generatrix {s_mot_hi:.1f} mm (motor),"
      f" {s_out_hi:.1f} mm (output — shorter,")
print(f"  its shallower angle reaches the same rubber radius with less plastic)")
print("-" * 72)
print("  FLAT PATTERN — a cone is developable, so the rubber is cut from sheet")
for _tag, _ha, _qty in (("motor ", alpha, 2), ("output", beta, 4)):
    _arc, _k, _psi, _len = seam_spiral(_ha)
    print(f"    {_tag} (x{_qty}): annular sector, r {s_rub_lo:.1f}..{s_rub_hi:.1f}"
          f" mm, {math.degrees(_arc):.0f} deg of arc"
          f"  ->  needs {2*s_rub_hi:.0f} x {2*s_rub_hi:.0f} mm of sheet")
    print(f"            spiral seam theta = {_k:.3f} * ln(s / {s_rub_lo:.0f}):"
          f" {math.degrees(_psi):.0f} deg from the generatrix, {_len:.0f} mm long")
print("    Both edges are the SAME spiral, offset by the sector arc. A seam at"
      " that angle crosses the")
print("    contact line at one point that sweeps along it as the cone turns,"
      " instead of the whole")
print("    line landing on the seam once per revolution. Shallower still (bigger"
      " k) keeps a crossing")
print("    permanently in contact, at the cost of seam length.")
if WRITE_FLAT_PATTERN:
    try:
        _here = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        _here = os.getcwd()
    try:
        _svg, _svg_err = write_flat_pattern(
            os.path.join(_here, "motcore_v6_flat_pattern.svg"))
        print(f"    -> written 1:1 to {_svg}")
        print(f"       [{'OK ' if _svg_err < 2e-3 else 'FAIL'}] pattern area vs"
              f" the analytic band area: {_svg_err*100:.3f}% off")
    except OSError as exc:
        print(f"    -> could NOT write the template: {exc}")
print("-" * 72)
print("  STOPS (tilt from the middle; free is the middle, both ways)")
print(f"    free    phi = 0.000 deg")
print(f"    contact phi = {math.degrees(phi_c):.3f} deg"
      f"   push point travel {contact_travel:.3f} mm")
print(f"    preload phi = {math.degrees(phi_preload):.3f} deg"
      f"   servo-side travel {act_amp:.2f} mm, split 1:{spring_split-1:.0f}")
print(f"  Rendering '{CARRIAGE_STOP}' "
      f"{'up' if CARRIAGE_DIR > 0 else 'down'}  ->  phi = {math.degrees(phi):+.3f} deg")
print("-" * 72)
print("  FOUR-BAR (answers doc open question 10.3)")
print(f"    links {link_len:.2f} mm at {link_axis_deg:.2f} deg from +Y,"
      f" frame pivots r = {r_frame_pivot:.2f}, carriage pivots r = {r_carriage_pivot:.2f}")
for _name, _d in (("contact", drift_contact), ("preload", drift_preload)):
    print(f"    apex drift at {_name:<8} {_d[0]:.4f} mm"
          f"   along generatrix {_d[1]:+.4f}  normal {_d[2]:+.4f}")
print(f"    -> residual slip {slip_fourbar*100:.2f}% of the contact line"
      f"   (v5, interleaved O-rings: 7.4%)")
print(f"    -> the normal component is a preload error of"
      f" {abs(drift_preload[2]):.3f} mm, {abs(drift_preload[2])/t_rubber*100:.0f}%"
      f" of the rubber thickness: NOT negligible against a ~0.2 mm squeeze")
print("-" * 72)
print("  CONTACT STATE at the rendered pose")
print(f"    rubber gap: lower cone {_gap_lo:.4f} mm, upper cone {_gap_up:.4f} mm")
print(f"    rubber squeeze volume: lower {_rub_lo:.2f} mm3, upper {_rub_up:.2f} mm3")
print(f"    free-state gap (both cones, phi = 0): {_free_gap:.4f} mm")
if phi_contact_real is None:
    print("    !! the rubber never meets within 2.5x phi_c: the four-bar's outward")
    print("       drift opens the contact as fast as the tilt closes it")
else:
    print(f"    contact really happens at phi ="
          f" {math.degrees(phi_contact_real):.3f} deg, not"
          f" {math.degrees(phi_c):.3f} — the drift is outward along +Y, so it")
    print(f"    OPENS the contact by ~drift*sin(beta); first touch is at s ="
          f" {contact_s:.1f} mm (band is {s_rub_lo:.0f}..{s_rub_hi:.0f}),"
          f" i.e. the {'outer' if contact_s > (s_rub_lo+s_rub_hi)/2 else 'inner'} end")
    print(f"    at the PRELOAD stop the squeeze is {_sq_vol:.2f} mm3"
          + (f", over s = {_sq_lo:.1f}..{_sq_hi:.1f} mm"
             f" = the outer {(_sq_hi-_sq_lo)/L_line*100:.0f}% of the band"
             if _sq_lo is not None else " — nothing touches"))
    if _sq_lo is not None:
        print(f"       (contact starts at the rim and spreads inward as the"
              f" preload grows, instead of arriving along the whole line at once)")
    print(f"    -> usable preload travel is"
          f" {math.degrees(phi_preload - phi_contact_real):+.3f} deg, not the"
          f" {math.degrees(phi_preload - phi_c):.3f} deg the pure-rotation"
          f" model gives")
print("-" * 72)
print("  GEAR STAGE (never disengages)")
print(f"    m = {m_mod:.2f}  Zp {Zp} -> Zi {Zi} (x2, on X) -> Zc {Zc}"
      f"   centre distances {e_ext:.2f} / {e_int:.2f} mm")
print(f"    corona pitch r {r_pitch_c:.2f}, outer r {r_corona_outer:.2f};"
      f" pinion tip r {r_tip_p:.2f}; hub wall {pinion_hub_wall:.2f} mm")
print(f"    faces: idler/corona {gear_face_w:.1f} mm, pinion"
      f" {pinion_face_w:.1f} mm (the difference is its tilt clearance against"
      f" the corona plate)")
print(f"    pinion centre rises {_pin_c_max[1]:.3f} mm at full preload"
      f"  ->  centre distance {e_ext:.2f} -> {_e_tilt:.2f} ({_e_growth:+.3f})")
print(f"    an idler on Z instead would have seen {_e_if_z:.2f} mm"
      f" ({_e_if_z - e_ext:+.2f}): jam on one side, disengage on the other"
      f"  <- invariant 6")
if GEARS_AVAILABLE and not gear_phase_ok:
    print("    WARNING: tooth-phase probe found no material — mesh phase is"
          " NOT set, the mesh volumes below are meaningless")
print("-" * 72)
print("  ACTUATION (placeholder until the rubber stiffness is measured)")
print(f"    push point at R = {R_push:.1f} mm; crank r {crank_r:.2f} at"
      f" (y {crank_hub[0]:.1f}, z {crank_hub[1]:.1f}), link {act_link_len:.1f} mm")
print(f"    servo side {STATE['zc']:+.3f} mm,"
      f" crank angle {math.degrees(STATE['psi']):.1f} deg,"
      f" link now {act_len_now:.3f} mm -> spring compressed {spring_comp:+.3f} mm")
print(f"    {_reach_margin:.2f} mm from dead centre at the ends of the stroke —"
      f" that IS the toggle (force")
print(f"    multiplied, travel nearly free), and it is also why the last"
      f" fraction of a mm of")
print(f"    push costs most of the servo's sweep")
def _crank_tilt(phi_t):
    """Degrees the crank arm sits BELOW horizontal at this stop (negative =
    above). The arm follows the engaged cone: down for the lower one, up for
    the upper — but only near the ends of the stroke, because contact happens
    in the first few degrees of the sweep."""
    return abs(math.degrees(actuator(phi_t)[1])) - 90.0


print(f"    crank arm vs horizontal: {_crank_tilt(-phi_preload):+.0f} deg at"
      f" preload DOWN (lower cone), {_crank_tilt(phi_preload):+.0f} deg at preload"
      f" UP, and only")
print(f"    {_crank_tilt(-phi_c):+.0f} deg at contact — positive is downward."
      f" It looks nearly horizontal at the 'contact' stop because")
print(f"    contact is reached in the first eighth of the sweep; the rest of"
      f" the sweep is preload")
# The "lever" is NOT a torque multiplier, and reading it as one is a trap.
# Moments about the apex give  F*R_push = INTEGRAL n(s)*s ds, and the output
# torque is  mu * INTEGRAL n(s)*(s*sin beta) ds  =  mu*sin(beta)*F*R_push:
# the contact's position cancels out exactly. That is the same property of the
# apex pivot that matches the surface speeds — every quantity scales with s.
# What R_push/s_bar really gives is the total NORMAL force, which is what has
# to be developed inside the available squeeze (open question 1), not torque.
_mu_ref = 1.3
print(f"    normal force per newton of actuator force: R_push / s_bar ="
      f" {R_push / _sq_sbar:.2f}"
      f"  (s_bar = {_sq_sbar:.1f} mm, the squeeze's own centroid, not the"
      f" band's midpoint)")
print(f"    output torque per newton: mu*sin(beta)*R_push*(Zc/Zp) ="
      f" {_mu_ref * math.sin(beta) * R_push / 1000.0 * (Zc / Zp):.4f} Nm/N at"
      f" mu = {_mu_ref}")
print(f"    — and that does NOT depend on where along the generatrix the"
      f" contact sits: more radius buys")
print(f"      more friction torque and costs exactly as much normal force."
      f" Only F and R_push move it.")
print("-" * 72)
for label, value, target, ok in checks:
    print(f"  [{'OK ' if ok else 'FAIL'}] {label}:  {value:.3f}  ({target})")
print("-" * 72)
_hbb = _half_p.BoundBox
print(f"  Carriage half: {_hbb.YLength:.0f} x {_hbb.ZLength:.0f} mm footprint,"
      f" {_hbb.XLength:.0f} mm tall, printed on its cut face — every hole"
      f" vertical, no supports")
print(f"  CUBE side {2 * cube_out:.1f} mm"
      f"  (half {cube_half:.1f} inside + {wall_thick:.1f} wall)")
print("  PRINTED: MotorCone x2 (same part, flipped), OutputCone,")
print("           CarriageHalf x2 (the same part twice, split on x = 0),")
print("           Pinion, Idler x2, CoronaShaft, IdlerBracket x2, FrameBracket x2,")
print("           Link x4, Crank, ActLink")
print("  PURCHASED, per axis: 2x MR105ZZ, Ø5 rod (shaft + 2 push pins),")
print("             Ø4 pin stock (4 pivot pins), 4x M3x40 + nuts to clamp the")
print(f"             carriage halves, {len(wall_screws())}x M3x8 through the wall"
      f" into the bracket feet,")
print("             2x M2x6 for the servo tabs, rubber sheet, springs, servo")
print(f"  FDM holes (this printer runs ~0.5 under): shaft Ø{fdm_shaft_hole_d:.1f}"
      f"  pin Ø{fdm_pin_hole_d:.1f}  bearing seat Ø{brg_od + 2*brg_fit_press:.1f}")
print("=" * 72)
if not GEARS_AVAILABLE:
    print("NOTE: freecad.gears not found — pinion/idlers/corona are reference")
    print("cylinders with no teeth. Install the 'Gear' addon for real involutes.")
print("Packaging (carriage arms, idler bracket, frame brackets, servo mount) is")
print("a first pass: the geometry above is derived, the brackets are not.")
