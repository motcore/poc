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
Zi    = 8          # —   — idler teeth (fixed axis, on X: invariant 6)
IDLER_SIDES = (-1,)  #    — which sides carry an idler. ONE, and that is not a
                     #       preference: pinion + two idlers + corona is a closed
                     #       loop, and the pinion MOVES — 2.42 mm at full preload,
                     #       which is the mechanism working, not an error. Each
                     #       idler then demands its own pinion phase, ~9 deg
                     #       apart, and no single phase satisfies both: swept in
                     #       the macro, the best compromise still leaves 4.1 mm3
                     #       of tooth interference. The train jams. One idler
                     #       leaves the loop open, and it still centres the
                     #       output shaft, which is the whole job (Zc = Zp + 2*Zi
                     #       makes both its centre distances equal). What is lost
                     #       is the radial-load balancing the second one was
                     #       for — the carriage bearings take it instead, ~111 N
                     #       at 2.4 Nm out, against MR105ZZ's ~800 N rating.
                     #       -X rather than +X for no deeper reason than that
                     #       +X is where the actuation lives — link, crank,
                     #       servo and its bracket. Either side satisfies
                     #       invariant 6, which only forbids Z, so giving each
                     #       its own side costs nothing and buys room.
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
# Both pivots sit on ONE ray from the apex, at 40 deg from +Y, at radius 62 and
# 40.5. The ray came down from 50 deg and the carriage pivot came out from 42
# because the cone is now a closed shell: the carriage's arms start inside it and
# can only leave through its mouth at y = 30.8, so the pivot has to be beyond
# that. Tilting the ray is what keeps the link long while doing it.
#
# It turned out to be worth more than the packaging. Measured on the linkage:
#     ray 50, link 16.0 (the old one)   drift 0.226 mm   0.74% slip
#     ray 50, link  9.8 (pivot moved)   drift 0.424      1.24%
#     ray 40, link 16.0                 drift 0.177      0.57%
#     ray 40, link 21.5 (this one)      drift 0.145      0.48%
# — a third less drift than the layout it replaces, which also means less of the
# preload travel lost to late contact.
fb_A = (47.49, 39.85)  # mm — frame pivot, upper link (radius 62 at 40 deg)
fb_B = (31.02, 26.03)  # mm — carriage pivot, upper link (radius 40.5)
                        #      The lower pair is the mirror in z.
link_x     = 9.0      # mm — X of the link plane (two sets, at +-link_x, for
                       #      out-of-plane stiffness). Just outboard of the
                       #      carriage's own arms.
link_t     = 5.0      # mm — link thickness (along X)
link_w     = 8.0      # mm — link width. Was 10, and the inner edge of the arms
                       #      then passed 0.64 mm from the motor cone's rim —
                       #      which only became visible once the knuckle moved
                       #      off B and stopped being the closest thing. The
                       #      link is a two-force member, so width buys it
                       #      almost nothing: 8 x 5 in compression is ample, and
                       #      the millimetre off each edge goes straight into
                       #      the clearance, 0.64 -> 1.63 mm.
link_knuckle = 5.0    # mm — radius of the knuckle joining the pair into one
                       #      part.
link_knuckle_d = 17.5 # mm — where that knuckle sits, measured along the link
                       #      from the FRAME pivot A. It used to sit on the
                       #      carriage pivot B (d = link_len), and that is the
                       #      one place it must not: the knuckle is the only
                       #      part of the link crossing x = 0, so it is the part
                       #      nearest the motor cone's rim, and there it cleared
                       #      it by 0.077 mm — under a printed wall's roughness,
                       #      i.e. touching. Sliding it 4 mm inboard costs
                       #      nothing and the two constraints then balance:
                       #      the frame post creeps up from A, the motor cone
                       #      down from B, and they cross here at 4.0 mm each.
                       #      Swept in the checks below, not assumed.
pin_d      = 4.0      # mm — pivot pin diameter (all four-bar pins)

# ── Actuation: servo → crank → telescopic spring link → carriage trunnion ────
# First pass. Everything here is a placeholder until the rubber stiffness is
# measured (doc §10.1) — that number decides the spring, and the spring decides
# the servo.
R_push        = 40.0   # mm — apex → push point, measured in Y. It is NOT the
                        #      lever any more, and that is the point of this
                        #      number being 40 and not 50.
                        #      R_push would be the moment arm if the push were
                        #      vertical. It is not: the servo lies on the floor
                        #      and the link comes up at a shallow angle, so the
                        #      arm is the perpendicular distance from the apex
                        #      to the LINK's line — measured, see push_lever().
                        #      Swept over R_push = 32..50 that distance barely
                        #      moves (39..45 mm): pushing from further out costs
                        #      as much angle as it gains radius. So R_push is
                        #      chosen for PACKAGING instead, and a shorter horn
                        #      is a stiffer horn, less mass on the part that
                        #      tilts, and — at anything under 48 — a cube set by
                        #      the corona rather than by the horn.
                        #      What DOES move the lever is the servo's own Y,
                        #      see crank_hub_y.
push_z        = -30.0  # mm — Z of the push point. Off the centre line because
                        #      the output shaft is on it, and below the corona's
                        #      rim so the horn never crosses the gear plane.
act_amp       = 6.0    # mm — servo-side travel of the push point, each way
spring_ratio  = 10.0   # —  — rubber stiffness / spring stiffness, seen at the
                        #      push point, with the push point at spring_ratio_R.
                        #      Invented (doc §10.2), to be measured.
spring_ratio_R = 50.0  # mm — where that ratio was invented. It is not a free
                        #      parameter: a given spring and a given rubber look
                        #      like a DIFFERENT ratio from a different radius,
                        #      because the same push-point travel becomes a
                        #      bigger tilt, and the tilt is what squeezes the
                        #      rubber. The ratio at the push point therefore
                        #      goes as 1/R^2, and spring_split below carries it
                        #      across. Without that, shortening the horn from 50
                        #      to 40 quietly raised the preload by half — the
                        #      squeeze went from 4.0 to 7.4 mm3 — on a design
                        #      change that was supposed to be about packaging.
crank_hub_y   = 24.0   # mm — Y of the servo output shaft. On the floor,
                        #      under the corona, with the case pointing inward:
                        #      the deck exists now, and it is the one surface
                        #      with room to spare. The link simply gets longer;
                        #      the push point stays where it was.
                        #      Y is what sets the LEVER, so it wants to be as
                        #      far out as it will go: the link's line of action
                        #      passes 39 mm from the apex with the shaft at 21
                        #      and 43.5 mm with it at 29, which is 11% of output
                        #      torque for nothing. (It used to be the far tab
                        #      against the wall that set this; the wall is no
                        #      longer the binding side.)
                        #      What stops it going further is the four-bar's
                        #      lower link, which runs from (31, -26) down to
                        #      (47, -40): past y = 36 it has dropped to the top
                        #      of the servo's own case.
                        #      Its Z is DERIVED — it follows the deck.
servo_lift    = 6.5    # mm — how far the bracket's foot lifts the case off the
                        #      deck. Not cosmetic: the crank's radius is 6.3 and
                        #      the shaft sits only 6.1 above a case lying flat,
                        #      so without the lift the crank circle cuts the
                        #      floor. Came down from 11 when the horn appeared
                        #      overhead: the case's top face, and the bracket
                        #      plate that reaches it, have to stay under the
                        #      horn's lower edge — which is at z = -35 at rest
                        #      and 1.5 mm lower at full preload down.
                        #      The crank's own swept circle is what stops it
                        #      going lower: 9.3 mm about a shaft 6.1 above the
                        #      case's underside, against a floor 55 mm down.
                        #      (Was (46, -28), directly under the push point.
                        #      on the push point's own vertical: off it, the
                        #      distance hub→push point varies over the stroke
                        #      and the link stops reaching one end. Moved to
                        #      y = 40 to dodge the corona, it could no longer
                        #      reach the top of the stroke at all (34.5 mm
                        #      needed, 34.3 available) — the clearance came
                        #      from moving the actuation plane out in X
                        #      instead, see act_x.)
crank_r       = 6.3    # mm — crank radius
# act_link_len is DERIVED: the link's free length is whatever spans hub to push
# point at rest, so moving the servo cannot silently put it out of reach.
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
servo_body    = (29.0, 23.0, 12.2)   # mm (X, Y, Z) — MG90-class placeholder.
                        #      The shaft comes out of the TOP face, so the case's
                        #      29 mm height runs along the shaft axis, X. The
                        #      other two turn about it, and here the case lies
                        #      FLAT ON THE FLOOR: 12.2 of thickness up Z, 23 of
                        #      length along Y, reaching inward from the shaft
                        #      toward the motor.
servo_tab_t   = 2.5    # mm — thickness of the servo's own mounting tabs
servo_tab_out = 4.7    # mm — how far each tab reaches past the body (an MG90 is
                        #      32.2 long over the tabs against 22.8 of body)
servo_screw_d = 2.0    # mm — M2 through the tabs into the bracket
servo_plate_t = 3.0    # mm — bracket face plate thickness
crank_web_t   = 3.5    # mm — crank web thickness (X)
crank_hub_r   = 5.0    # mm — crank boss radius. It passes through a clearance
                        #      hole in the bracket plate, and the servo's near
                        #      tab screw is 8.15 mm from the shaft, so this is
                        #      what leaves the plate ~1 mm of material between
                        #      the two.
crank_boss_h  = 4.0    # mm — how far the crank's web stands off the servo's top
                        #      face: the boss wraps the output spline, which is
                        #      what lets the crank sweep OVER the two tab screws
                        #      (its swept radius is 10.3 and they sit at 8.15,
                        #      so flat on the case they collide).
                        #      It can stay this short — a long boss is a long
                        #      lever on the servo's own shaft — only because the
                        #      crank sits INBOARD of the link plane, with the
                        #      pin reaching outboard through it. Outboard, the
                        #      boss would have to cross the link plane at the
                        #      hub, and near dead centre the link lies right on
                        #      top of the hub.
servo_wall_clr = 1.5   # mm — clearance from the servo's far tab to the wall

# ── Render pose ──────────────────────────────────────────────────────────────
CARRIAGE_STOP = "contact"   # "free" | "contact" | "preload"
CARRIAGE_DIR  = -1          # -1 = tilt DOWN (engages the LOWER motor cone),
                             # +1 = UP. Ignored at "free", which is the middle.

AXES_SHOWN = 1     # 0..4 — output axes built (mechanism + wall). 1 keeps the
                    #        view readable; 4 is the full cube. The central
                    #        motor cones and shaft are always built.

# ── Carriage ─────────────────────────────────────────────────────────────────
hous_r        = 7.5    # mm — bearing housing radius. It lives INSIDE the cone,
                        #      so the cavity sets it: 8.27 mm of radius at
                        #      y = 21, leaving 0.77 of running clearance. Over
                        #      the Ø10.3 bearing seat the wall is 2.35 mm.
hous_y0       = 21.0   # mm — housing, cone-side face. Was 33.5, outside the cone
                        #      altogether; those 12.5 mm are the whole prize.
hous_y1       = 38.0   # mm — housing, pinion-side face, past the cone's mouth at
                        #      30.8. ONE piece, not two halves: at this radius
                        #      there is no room for clamp bolts and no need for
                        #      them — the split existed only to keep the bearing
                        #      seats off their sides while printing, and a
                        #      housing this shape prints with its bore straight
                        #      up. Both bearings go in from the pinion end,
                        #      against a lip at the far one.
side_x        = 15.0   # mm — X of the carriage's two arms. OUTBOARD of the
                        #      four-bar links, because the knuckle that joins
                        #      each pair of links into one part runs across the
                        #      middle. A short web takes them out there from the
                        #      housing.
side_t        = 4.0    # mm — arm thickness
arm_w         = 7.0    # mm — carriage arm width. It has to pass through a gap
                        #      9 mm wide: the output cone's rim at y = 30.8 on
                        #      one side, the idler bracket's plate at 40.5 on
                        #      the other, and a round-ended bar reaches arm_w/2
                        #      past its own root at each end.
arm_root      = (36.0, 5.0)   # mm (y, |z|) — where each arm leaves the housing.
                        #      Y is set by the cone's own rim: the arms are
                        #      round-ended bars, so each one reaches arm_w/2
                        #      further in than its root, and the cone's mouth is
                        #      at y = 30.8 with a 17.6 mm rim — which is exactly
                        #      the radius the arms pass at.
horn_w        = 10.0   # mm — width of the horn that carries the push point
horn_t        = 6.0    # mm — its thickness (X). It is the whole actuation load
                        #      path, so it is the thickest plate on the part.
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
bolt_d        = 3.0    # mm ┐
bolt_head_d   = 5.5    # mm │ M3 socket head and nut, as envelopes. Modelled as
bolt_head_h   = 3.0    # mm │ solids and not just as holes, because a hole
bolt_nut_d    = 6.4    # mm │ collides with nothing: it was the HEAD that ran
bolt_nut_h    = 2.4    # mm ┘ into the actuation link, and nothing could see it.
trunnion_d    = 5.0    # mm — push trunnion diameter (a length of the Ø5 rod
                        #      the project already buys, NOT a printed boss)

# ── Gear stage packaging ─────────────────────────────────────────────────────
gear_face_w   = 5.0    # mm — pinion / idler / corona face width
gear_y0       = 44.5   # mm — gear plane, front face (carriage side). It looked
                        #      like it could come right up against the housing
                        #      once that moved inside the cone, and it cannot:
                        #      the carriage's arms sit at |x| = 15 and the idler
                        #      at 14.4, so the bracket's plate needs its own Y to
                        #      get past them. Costs no cube while the horn is
                        #      what sets it. Pushed
                        #      out from 51.5: the idler bracket's plate has to
                        #      cross in front of it, and at 51.5 that arm ran
                        #      into the push trunnion's Ø5 boss at y = 46.
                        #      Then out again from 53.5, because those 3 mm of
                        #      Y are ALL there is between the carriage and the
                        #      gears, and the plate has to fit in them with a
                        #      running clearance at one face and the idler's
                        #      axle rooted in the other. Each mm here costs 2 mm
                        #      of cube side.
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
idler_arm_t   = 3.5    # mm — idler bracket plate thickness (along Y), which is
                        #      also how deep the axle is rooted
idler_neck_w  = 8.0    # mm — width (Z) of the neck between the plate and its
                        #      seat. This one really does have to pass BETWEEN
                        #      the two screw heads.
idler_lobe_r  = 5.5    # mm — radius of the plate's rounded end around the axle.
                        #      Small on purpose: it has to stop short of the
                        #      bearing housing's 7.5 mm radius, because the
                        #      plate no longer crosses in FRONT of the housing —
                        #      it sits BESIDE it, which is what lets the gear
                        #      plane come right up against the carriage. What
                        #      roots the axle is the plate's thickness, not this.
fdm_axle_press_d = 4.3  # mm — modelled Ø for the Ø4 idler axle, INTERFERENCE.
                         #      Extrapolated from the Ø5 numbers, not measured.
shaft_flat_d  = 4.0    # mm — the Ø5 output shaft is filed to a flat, leaving
                        #      this across it, and the cone and the pinion carry
                        #      the matching D. Form, not friction: a set screw
                        #      does not fit anywhere on the pinion — its hub
                        #      wall is 2.05 mm, and beyond the gear's faces
                        #      there is 0.5 mm of Y to the idler plate on one
                        #      side and 1.5 mm to the corona plate on the other.
                        #      The joint carries 0.80 Nm at the doc's upper
                        #      bound, 320 N at the shaft's surface: on a ~4.3 mm
                        #      chord over the pinion's 3.5 mm face that is about
                        #      24 MPa of bearing, against ~50 for PLA.
                        #      FILING THAT FLAT IS THE ONE MANUAL STEP the macro
                        #      cannot check for you.
shaft_flat_clr = 0.25  # mm — how much the printed D is relieved off the flat
idler_anchor_x = 32.0  # mm — |X| where the bracket anchors to the wall: clear
                        #      of the corona's 25.6 mm rim, with room for a
                        #      screw's edge margin inboard of it
idler_pad_z   = 8.0    # mm — |Z| of its two screws. 16 apart, so the Ø11 bosses
                        #      do not touch, and far enough out that the heads —
                        #      which now stand proud INSIDE — clear the leg.


# ── Frame ────────────────────────────────────────────────────────────────────
fp_lug_x      = 6.0    # mm — frame-pivot lug half-width in X (the links sit
                        #      just outboard of it)
fp_post_x     = 19.0   # mm — half-width in X of the post's foot on the floor
fp_post_y     = 6.0    # mm — half-depth in Y of that foot. Its two screws are
                        #      spread in X, so Y only needs edge margin — and
                        #      this foot is one of the things the wall has to
                        #      clear.
fp_screw_x    = 14.5   # mm — |X| of its two screws into the floor. Out past the
                        #      LINKS, not merely past the post's own column: the
                        #      heads stand 3 mm proud of the foot and the link
                        #      arms pass over them. (The knuckle used to swing
                        #      0.9 mm into that too; it no longer comes near —
                        #      see link_knuckle_d — but the arms still do.) The
                        #      wider stance also suits the load, which arrives
                        #      along the link at 40 deg and tips the post.
deck_t        = 4.0    # mm — floor / ceiling plate thickness
deck_boss_h   = 5.0    # mm — how far their screw bosses stand proud, inward

foot_t        = 5.0    # mm — thickness of a bracket's foot. Was 3, which with
                        #      a Ø3.8 hole through it was not a joint, it was a
                        #      tab with a hole in it.
foot_edge     = 4.5    # mm — material from a screw's centre to any free edge of
                        #      a foot: 1.5x the M3's nominal diameter, so a
                        #      seat is 13 mm across rather than 9.
foot_screw_d  = 3.0    # mm — M3, driven from INSIDE the cube
foot_hole_d   = 3.8    # mm — clearance Ø in the foot (printer runs holes under)
foot_tap_d    = 2.6    # mm — blind Ø in the wall boss for an M3 self-tapper.
                        #      Not measured on this printer — verify on a coupon.
wall_boss_h   = 5.0    # mm — how far the wall's screw bosses stand proud of its
                        #      INNER face. Screws now go in from inside and stop
                        #      in these, so the outer face of the cube stays
                        #      clean — no heads on the outside of the machine.
wall_boss_d   = 11.0   # mm — boss Ø: foot_tap_d plus a real wall all round
foot_flange   = 9.0    # mm — how far a frame-bracket foot turns inboard to give
                        #      its screws something to pass through
wall_thick     = 4.0   # mm — cube wall thickness
wall_gap       = 2.0   # mm — RUNNING clearance, corona back plate → wall. It
                        #      was 6, which is a lot of air for a disc facing a
                        #      fixed plate. Note it no longer sets the cube on
                        #      its own — see cube_half.
wall_shaft_clr = 2.0   # mm — radial clearance of the wall's output-shaft hole

# ── Reference-only extras (2D cross-section → solids) ────────────────────────
cone_apex_trim = 3.0   # mm — truncate a cone this far (generatrix) from its
                        #      own PLASTIC apex
cone_tip_wall  = 0.8   # mm — minimum wall at a cone's truncated tip where a
                        #      through bore exists (motor cones)
cone_bore_wall = 2.0   # mm — wall left at the far end of the output cone's
                        #      BLIND bore, which is what sets its depth
cone_wall     = 2.5    # mm — wall of the output cone, which is a SHELL. This is
                        #      what the whole outboard layout turns on: the cone
                        #      was a solid lump of plastic sitting between the
                        #      apex and the gears, and hollowing it lets the
                        #      bearing housing live INSIDE it. The entire gear
                        #      stack then sits behind the push point instead of
                        #      in front of it. A conical shell gives up almost
                        #      nothing in torsion against a solid cone, and it
                        #      takes a lot of mass off the part that tilts.
shaft_d        = 5.0   # mm — motor and output shaft diameter

# ── Carriage bearings — MR105ZZ, the project's single standard bearing ───────
brg_id        = 5.0    # mm ┐
brg_od        = 10.0   # mm │ MR105ZZ (5x10x4)
brg_w         = 4.0    # mm ┘
brg_seat_lip  = 1.5    # mm — lip at the cone end of the seat bore. Both bearings
                        #      go in from the pinion end, and this is what the
                        #      far one stops against.
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
idler_arm_y1 = gear_y0 - 0.5                   # plate's gear-side face: derived,
                                                # so it cannot drift away from
                                                # the gear plane it has to clear
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
# After contact, 1 part of further servo travel goes into the carriage and
# (split - 1) into the spring.
spring_split = 1.0 + spring_ratio * (spring_ratio_R / R_push) ** 2
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
    pin = _circ_int(crank_hub, crank_r, (R_push, push_z + zc), act_link_len,
                    (crank_hub[0] - crank_r, crank_hub[1]))
    if pin is None:      # crank + link cannot reach this stop at all
        if abs(zc) <= act_amp + 1e-9:    # only the design stroke must be
            _UNREACHABLE.append(zc)      # reachable; the contact bisection
                                          # deliberately explores well past it
        pin = (crank_hub[0] + crank_r, crank_hub[1])
    psi = math.atan2(pin[0] - crank_hub[0], pin[1] - crank_hub[1])
    return zc, psi, pin



# ── Cube ────────────────────────────────────────────────────────────────────
# Short face: the clearance to the corona's back plate, which rotates. Both
# the pinion AND the idler need it — the idler's back face sat exactly on the
# plate, which is a rub, not a fit.
pinion_face_w = gear_face_w - gear_back_gap
corona_plate_y = gear_y1                      # plate is CONTIGUOUS with the ring
corona_back_y = corona_plate_y + corona_plate_t
# Y of the servo's far end, which is the other thing that has to fit inside the
# wall — and, at the moment, the thing that actually sets the cube.
servo_y_end = crank_hub_y + 0.25 * servo_body[1] + servo_tab_out
# The cube is as small as the FURTHEST thing that must fit, not as the corona
# alone. Printing which one binds is the point: at the defaults it is the servo,
# whose case runs outboard from its shaft, so shaving the corona's clearance
# buys nothing until the servo moves.
# The tabs span the same Y as the case, so they are not a separate entry — they
# were, with a different margin, which is how a tie came to be reported as a
# winner.
# Everything that has to fit inside the wall. The push point and the four-bar's
# frame post were missing from this list, and the wall promptly closed in on top
# of them — the horn ends up as the furthest thing out now that the gears are
# behind it, which is exactly the point of the layout.
cube_half_by = {
    "corona running clearance": corona_back_y + wall_gap,
    "servo's far mounting tab": servo_y_end + servo_wall_clr,
    "push point and its horn": R_push + horn_w / 2.0 + 2.0,
    "four-bar frame post": fb_A[0] + fp_post_y + 1.0,
}
cube_half_driver = max(cube_half_by, key=cube_half_by.get)
cube_half = cube_half_by[cube_half_driver]
# The servo lies on the deck, so its shaft height follows from the deck, not the
# other way round.
crank_hub = (crank_hub_y,
             -cube_half + servo_lift + servo_body[2] / 2.0)
act_link_len = math.dist(crank_hub, (R_push, push_z))  # free length = hub to
                                                       # push point at rest
# X of the crank's web, and of the servo's top face crank_boss_h behind it. The
# case runs INBOARD from that face, not outboard: outboard it is the NEIGHBOURING
# axis' wall that stops it, 57 mm out, and the stack (link, crank, standoff, 29 mm
# of case) needs 60. Inboard, under the motor cone and above the deck, there is a
# pocket 40 mm deep with nothing in it. That flip is what lets the cube close.
crank_web_x  = act_x - link_t / 2.0 - 0.5 - crank_web_t
servo_face_x = crank_web_x - crank_boss_h
# The bracket's deck screws go up through its own plate, which is on the shaft
# side of the tabs.
servo_anchor_x = servo_face_x + servo_plate_t / 2.0
cube_out  = cube_half + wall_thick
# Brackets seat on the bosses, not on the wall itself.
wall_face_y = cube_half - wall_boss_h

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
            "pin": pin, "tab": T((R_push, push_z)), "zc": zc, "psi": psi}


def make_screw(base, direction, d, length, head_d, head_h):
    """A screw as an envelope: shank from `base` along `direction`, head on the
    near side. Modelled as a solid, not as a hole — a hole collides with
    nothing, which is how a clamp nut ended up inside the actuation link."""
    head_base = v(base.x - direction.x * head_h,
                  base.y - direction.y * head_h,
                  base.z - direction.z * head_h)
    return cyl(d / 2.0, length, base, direction).fuse(
        cyl(head_d / 2.0, head_h, head_base, direction))


def _deck_seat_t(sx):
    """Material UNDER a deck screw's head. The frame posts present a plain
    foot; the servo bracket's pad is counterbored, because the servo's own case
    sits 2 mm above it and a head standing proud would be inside it."""
    if abs(sx - servo_anchor_x) < 1e-6:
        return servo_lift - deck_boss_h - (bolt_head_h + 0.5)
    return foot_t


def _screw_seat_y(sx):
    """Y of the face a wall screw's head bears on. The idler's D is thickened
    all the way back, so its two screws start at the front of the plate; every
    other bracket presents a 5 mm foot against the bosses."""
    if abs(abs(sx) - idler_anchor_x) < 1e-6:
        return idler_arm_y1 - idler_arm_t
    return wall_face_y - foot_t


def wall_screws():
    """(x, z) of every screw that passes through this axis's wall. One list, so
    the wall and the brackets cannot disagree about where the holes are."""
    out = []
    # Two, spread in Z: the idler's tooth force is mostly along Z, so the pair
    # takes it as a shear couple.
    for sd in IDLER_SIDES:
        for zs in (1, -1):
            out.append((sd * idler_anchor_x, zs * idler_pad_z))
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
    # The cavity: the same cone offset inward by the wall, open at the base.
    # Its apex sits cone_wall/sin(beta) further out than the plastic one.
    cone = cone.cut(cone_frustum(v(0, out_apex_y + cone_wall / math.sin(beta), 0),
                                 Y_AXIS, beta, 0.0, s_out_hi + 5.0))
    cone = cone.cut(d_bore(out_bore_end_y, out_bore_depth + 1.0))
    return cone


def make_output_rubber():
    outer = cone_frustum(ORIGIN, Y_AXIS, beta, s_rub_lo, s_rub_hi)
    plastic = cone_frustum(v(0, out_apex_y, 0), Y_AXIS, beta,
                           0.0, s_out_hi + 20.0)
    return outer.cut(plastic)


def d_bore(y0, length, extra=0.0):
    """The D: a round bore with the flat's slab put back. Cut from a part, it
    leaves a bore that cannot turn on the filed shaft."""
    bore = cyl(fdm_shaft_hole_d / 2.0 + extra, length, v(0, y0, 0), Y_AXIS)
    # PLUS the clearance, not minus: the D's flat has to sit clear of the
    # shaft's, not bite into it.
    z_flat = shaft_flat_d / 2.0 + shaft_flat_clr
    return bore.cut(Part.makeBox(fdm_shaft_hole_d + 2, length,
                                 fdm_shaft_hole_d,
                                 v(-(fdm_shaft_hole_d / 2.0 + 1), y0, z_flat)))


def make_carriage_shaft():
    """Ø5 steel output shaft on the carriage: cone clamped at one end, pinion
    at the other, two bearings between (set screws on a filed flat, the joint
    pattern the project already uses)."""
    y_end = gear_y0 + pinion_face_w      # flush with the pinion's back face
    shaft = cyl(shaft_d / 2.0, y_end - out_bore_end_y,
                v(0, out_bore_end_y, 0), Y_AXIS)
    # The filed flat, running the whole length so one pass of the file keys
    # both the cone and the pinion.
    return shaft.cut(Part.makeBox(shaft_d + 2, y_end - out_bore_end_y, shaft_d,
                                  v(-(shaft_d / 2.0 + 1), out_bore_end_y,
                                    shaft_flat_d / 2.0)))


def make_carriage():
    """The carriage, in one piece: a bearing housing living inside the hollow
    cone, two arms out to the four-bar, and a horn down and out to the push
    point.

    ONE piece again. It was split in two halves so its bearing seats would not
    print as bridged horizontal holes; inside the cone it is a slim cylinder,
    which prints with its bore straight up, so the split bought nothing and cost
    four bolts. Both bearings now go in from the pinion end and stop against a
    lip at the far one.

    The arms can only leave through the cone's mouth — a shell has no other way
    out — so they start at the housing's outboard end and climb steeply. The
    horn goes the other way: down clear of the corona's rim first, then forward,
    because between those two it would cross the gear plane."""
    body = cyl(hous_r, hous_y1 - hous_y0, v(0, hous_y0, 0), Y_AXIS)
    body = body.fuse(make_carriage_arms(1)).fuse(make_carriage_arms(-1))
    body = body.fuse(make_horn())

    # Every hole LAST. The arms' own pivot holes were cut before the horn was
    # fused on, and the horn — which runs down the same X band as the arms —
    # filled the lower pair straight back in; the web that ties the arms to the
    # housing did the same to the bearing seat. A hole that a later fuse closes
    # looks perfectly fine until something is checked against it.
    #
    # Straight seat bore from the pinion end, with a lip at the cone end for the
    # far bearing to seat against.
    body = body.cut(cyl(brg_od / 2.0 + brg_fit_press,
                        hous_y1 - hous_y0 - brg_seat_lip + 1.0,
                        v(0, hous_y0 + brg_seat_lip, 0), Y_AXIS))
    body = body.cut(cyl(shaft_d / 2.0 + 0.75, hous_y1 - hous_y0 + 2,
                        v(0, hous_y0 - 1, 0), Y_AXIS))
    for zs in (1, -1):
        body = body.cut(pin_x((fb_B[0], zs * fb_B[1]), fdm_pin_hole_d,
                              -(side_x + side_t / 2.0 + 2.0),
                              2 * (side_x + side_t / 2.0 + 2.0)))
    body = body.cut(pin_x((R_push, push_z), fdm_dowel_hole_d,
                          side_x - horn_t / 2.0 - 1.0, horn_t + 2.0))
    return body


def make_horn():
    """Down, then forward: the arm that carries the push point out past the
    gears. Straight across, it would cross the gear plane; below the corona's
    rim there is nothing in the way at all."""
    # Out at the arms' own X, not on the centre line: down the middle is where
    # the lower link's knuckle lives.
    x0 = side_x - horn_t / 2.0
    corner = (hous_y1 - 2.0, push_z)
    horn = bar_yz((hous_y1 - 2.0, -hous_r + 1.0), corner, horn_w, x0, horn_t)
    horn = horn.fuse(bar_yz(corner, (R_push, push_z), horn_w, x0, horn_t))
    return horn

def make_carriage_arms(sd):
    """The two arms on one side, from the housing out to the four-bar pivots."""
    x0 = sd * side_x - side_t / 2.0
    part = None
    for zs in (1, -1):
        b = (fb_B[0], zs * fb_B[1])
        # Web out from the housing to the arm's plane.
        wx0, wx1 = sorted((sd * 4.0, x0 + (side_t if sd > 0 else 0.0)))
        arm = Part.makeBox(wx1 - wx0, 4.0, arm_w,
                           v(wx0, arm_root[0] - 4.0,
                             zs * arm_root[1] - arm_w / 2.0))
        arm = arm.fuse(bar_yz((arm_root[0], zs * arm_root[1]), b, arm_w, x0,
                              side_t))
        arm = arm.fuse(disc_yz(b, arm_w / 2.0 + 0.5, x0, side_t))
        part = arm if part is None else part.fuse(arm)
    return part

def make_trunnion():
    """Push pin at the end of the horn: a length of the Ø5 rod the project
    already buys, pressed through it. Not a printed boss — the load bends it
    across the layers, which is the one direction FDM has none."""
    # Starts flush with the horn's inner face: 1 mm of it standing proud on the
    # inboard side is 1 mm inside the four-bar's lower link.
    lo = side_x - horn_t / 2.0
    return cyl(trunnion_d / 2.0, act_x + link_t / 2.0 + 0.5 - lo,
               v(lo, R_push, push_z), X_AXIS)

def make_carriage_bearing(y_face, sd):
    y_start = min(y_face, y_face + sd * brg_w)
    outer = cyl(brg_od / 2.0, brg_w, v(0, y_start, 0), Y_AXIS)
    return outer.cut(cyl(brg_id / 2.0 + 0.05, brg_w + 2,
                         v(0, y_start - 1, 0), Y_AXIS))


def make_link_knuckle(A, B):
    """The web that joins the link's two arms, as its own shape.

    Separate from make_link because the checks need it alone: the ARMS are
    allowed to run within half a millimetre of the frame post — that is the
    pivot's own sliding fit, and a hinge wants it tight. The knuckle has no
    such licence. It is the one piece of the link crossing x = 0, so it is the
    piece that meets the motor cone's rim, and it must simply clear."""
    # Overlaps each arm by 1 mm rather than meeting it on a coincident face,
    # which fuses far more reliably.
    web_x = link_x - link_t / 2.0 + 1.0
    # Its place is a distance FROM A, not a fraction of the span: A is fixed
    # and B swings, and a fraction would let the knuckle drift down the link
    # as the carriage tilts.
    f = link_knuckle_d / math.dist(A, B)
    K = (A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f)
    return disc_yz(K, link_knuckle, -web_x, 2 * web_x)


def make_link(A, B):
    """One four-bar link — ONE part carrying both arms, not two loose plates.

    The two arms at +-link_x are the same rigid body (they move identically, by
    construction), so joining them costs nothing, makes the pair self-squaring,
    and actually delivers the out-of-plane stiffness that having two of them was
    for. They meet at a knuckle part-way along the span — see link_knuckle_d for
    why it is there and not on either pivot."""
    body = None
    for xs in (1, -1):
        arm = bar_yz(A, B, link_w, xs * link_x - link_t / 2.0, link_t)
        body = arm if body is None else body.fuse(arm)
    body = body.fuse(make_link_knuckle(A, B))

    reach = link_x + link_t / 2.0 + 2.0
    for q in (A, B):
        body = body.cut(pin_x(q, fdm_pin_hole_d, -reach, 2 * reach))
    return body


def make_frame_bracket(zs):
    """The four-bar's frame pivot, now a post standing on the deck rather than a
    plate reaching in from the wall.

    The link reaction at this pivot runs along the link, 50 deg from Y, so it is
    mostly Z — straight down the post into the deck, in compression. Off the
    wall it was 35 mm of cantilever taking the same load as bending. It also
    gives the wall back, which is what lets the servo lie under the corona."""
    z_pin = zs * fb_A[1]
    z_deck = zs * (cube_half - deck_boss_h)
    lug_far = z_pin - zs * (link_w / 2.0 + 1.0)

    z_lo, z_hi = sorted((z_deck, z_deck - zs * foot_t))
    foot = Part.makeBox(2 * fp_post_x, 2 * fp_post_y, z_hi - z_lo,
                        v(-fp_post_x, fb_A[0] - fp_post_y, z_lo))
    z_lo, z_hi = sorted((z_deck, lug_far))
    post = Part.makeBox(2 * fp_lug_x, 2 * fp_post_y, z_hi - z_lo,
                        v(-fp_lug_x, fb_A[0] - fp_post_y, z_lo))
    part = foot.fuse(post)
    part = part.fuse(disc_yz((fb_A[0], z_pin), link_w / 2.0 + 1.0,
                             -fp_lug_x, 2 * fp_lug_x))
    part = part.cut(pin_x((fb_A[0], z_pin), fdm_pin_hole_d,
                          -fp_lug_x - 1, 2 * fp_lug_x + 2))
    for sx, sy in frame_deck_screws():
            part = part.cut(cyl(foot_hole_d / 2.0, foot_t + 2,
                                v(sx, sy, z_deck - zs * (foot_t + 1)),
                                v(0, 0, zs)))
    return part

def make_idler_bracket(sd):
    """Fixed support for the idler: a D — round end over the gear with the axle
    at its centre, flat end thickened back to the wall, two holes through it.

    Everything happens in the Y in FRONT of the gear plane. That is forced:
    behind it, at any radius under the corona's 25.6 mm rim, the space belongs
    to the corona's back plate, which rotates. But being in front also means the
    corona's radius constrains nothing here, so the plate can be as generous as
    it likes — hence the lobe, which roots the axle in a boss rather than
    leaving it hanging off the end of a strip.

    The screws pass straight through the thickened end, heads bearing on its
    front face, and tap into the wall bosses beyond. No step, no neck, no holes
    to dodge the heads: one flat part with two holes and a pin, which prints on
    its back with every hole vertical."""
    xa = sd * idler_anchor_x
    x_ax = sd * idler_x
    y0 = idler_arm_y1 - idler_arm_t
    half_w = idler_pad_z + foot_edge
    x_lo, x_hi = sorted((x_ax, xa + sd * (foot_edge + 0.5)))

    body = Part.makeBox(x_hi - x_lo, idler_arm_t, 2 * half_w,
                        v(x_lo, y0, -half_w))
    body = body.fuse(cyl(idler_lobe_r, idler_arm_t, v(x_ax, y0, 0), Y_AXIS))
    # The flat of the D: thickened all the way back to the wall bosses, so the
    # screws run through material instead of across a gap.
    body = body.fuse(Part.makeBox(2 * (foot_edge + 0.5), wall_face_y - y0,
                                  2 * half_w,
                                  v(xa - foot_edge - 0.5, y0, -half_w)))
    body = body.cut(cyl(fdm_axle_press_d / 2.0, idler_arm_t + 2,
                        v(x_ax, y0 - 1, 0), Y_AXIS))
    for sx, sz in wall_screws():
        if abs(abs(sx) - idler_anchor_x) < 1e-6:
            body = body.cut(cyl(foot_hole_d / 2.0, wall_face_y - y0 + 2,
                                v(sx, y0 - 1, sz), Y_AXIS))
    return body


def make_idler_pin(sd):
    """The idler's axle: Ø4 dowel from the same stock as the four-bar pins,
    pressed through the plate and cantilevered out to carry the gear.

    Printed as a boss it would be a vertical pillar taking the tooth force
    across the layers — ~111 N at the doc's upper-bound torque, which is over
    100 MPa in bending on Ø4. In steel it is comfortable: the load sits 4.0 mm
    from the middle of a 3.5 mm root, so the root takes about 1.7x the tooth
    force as a couple, ~27 MPa of bearing on the PLA around it against a
    compressive strength near 50. (An earlier note here said 2.8x and called it
    marginal. That was computed before the idler's face was narrowed to clear
    the corona plate, which halved the overhang.)"""
    y0 = idler_arm_y1 - idler_arm_t
    return cyl(idler_axle_d / 2.0, gear_y0 + pinion_face_w + 0.5 - y0,
               v(sd * idler_x, y0, 0), Y_AXIS)

def make_corona_plate():
    return cyl(r_corona_outer, corona_plate_t, v(0, corona_plate_y, 0), Y_AXIS)


def make_output_shaft_ref():
    return cyl(shaft_d / 2.0, 40.0, v(0, corona_back_y, 0), Y_AXIS)


def make_crank(st):
    """Servo crank, dog-legged out to the actuation plane."""
    x_web = crank_web_x
    # The boss runs back to the servo's face: it wraps the spline and stands the
    # web off it. Ø12, so it passes inside the tab screws at 8.15 mm radius.
    hub = cyl(crank_hub_r, x_web + crank_web_t - servo_face_x,
              v(servo_face_x, crank_hub[0], crank_hub[1]), X_AXIS)
    # 6 wide, not 8: the web's own half-width is part of the crank's swept
    # circle, and that circle is what holds the servo up off the floor.
    web = bar_yz(crank_hub, st["pin"], 6.0, x_web, crank_web_t)
    # Ø3, matching the 3.2 hole in the link. It was written as cyl(3.0, ...),
    # which is a RADIUS — a Ø6 pin through a Ø3.2 hole, 101 mm3 of overlap.
    # Outboard from inside the web, through the link and a little past it.
    pin = cyl(1.5, link_t + 4.0, v(crank_web_x + 1.0,
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
    shaft comes out of — is the HIGH-x face, at servo_face_x, and the case runs
    inboard from there. The shaft sits on the centre line of the case's
    thickness and a quarter of the way along its length, as it does on an MG90;
    the case therefore runs INWARD in Y from the crank too, under the corona."""
    bx, by, bz = servo_body
    return (servo_face_x - bx, crank_hub[0] - 0.75 * by,
            crank_hub[1] - bz / 2.0, bx, by, bz)


def make_servo_body():
    """MG90-class placeholder: body plus the two mounting tabs, which are what
    it actually hangs from."""
    x0, y0, z0, bx, by, bz = servo_box()
    body = Part.makeBox(bx, by, bz, v(x0, y0, z0))
    # The tabs reach past the case along its length and span its width, at the
    # top face — which is the outboard end of the case.
    tabs = Part.makeBox(servo_tab_t, by + 2 * servo_tab_out, bz,
                        v(servo_face_x - servo_tab_t, y0 - servo_tab_out, z0))
    body = body.fuse(tabs)
    for sy in servo_screw_ys():
        body = body.cut(cyl(servo_screw_d / 2.0 + 0.1, servo_tab_t + 2,
                            v(servo_face_x - servo_tab_t - 1, sy,
                              crank_hub[1]), X_AXIS))
    return body


def servo_screw_ys():
    """Y of the two tab screws: an MG90's holes are 27.8 apart, along the case's
    length, which runs along Y."""
    _, y0, _, _, by, _ = servo_box()
    return (y0 - servo_tab_out / 2.0, y0 + by + servo_tab_out / 2.0)


def make_servo_bracket():
    """What the servo hangs from: a plate across its two mounting tabs, on the
    SHAFT side of them, standing on the deck.

    Shaft side, not case side, because the case side is where the four-bar's
    lower link comes down: the plate reaches 6.8 mm past the case at each end,
    and at the far end that lands under the link. On the shaft side it sits in
    the X band between the links and the carriage's arms, and everything there
    is 10 mm higher up.

    The crank's boss passes through it, so it has a clearance hole — which is
    what limits the boss to Ø10: the servo's own near tab screw is only 8.15 mm
    from the shaft, and the plate has to keep some material between the two."""
    x0, y0, z0, bx, by, bz = servo_box()
    sy0, sy1 = servo_screw_ys()
    z_deck = -(cube_half - deck_boss_h)

    part = Part.makeBox(servo_plate_t, (sy1 + foot_edge) - (sy0 - foot_edge),
                        z0 + bz - z_deck,
                        v(servo_face_x, sy0 - foot_edge, z_deck))
    part = part.cut(cyl(crank_hub_r + 0.75, servo_plate_t + 2,
                        v(servo_face_x - 1, crank_hub[0], crank_hub[1]),
                        X_AXIS))
    for sx, sy in servo_deck_screws():
        part = part.fuse(Part.makeBox(servo_plate_t + 8.0, 2 * foot_edge,
                                      servo_lift - deck_boss_h,
                                      v(servo_face_x - 4.0, sy - foot_edge,
                                        z_deck)))
    for sx, sy in servo_deck_screws():
        part = part.cut(cyl(foot_hole_d / 2.0, servo_lift + 2,
                            v(sx, sy, z_deck - 1), Z_AXIS))
        # Counterbore: the case sits just above this pad.
        part = part.cut(cyl((bolt_head_d + 0.6) / 2.0, bolt_head_h + 0.5,
                            v(sx, sy,
                              z_deck + servo_lift - deck_boss_h
                              - (bolt_head_h + 0.5)), Z_AXIS))
    for sy in servo_screw_ys():
        part = part.cut(cyl(servo_screw_d / 2.0 + 0.4, servo_plate_t + 2,
                            v(servo_face_x - 1, sy, crank_hub[1]), X_AXIS))
    return part

def servo_deck_screws():
    """The subset of deck_screws() that belongs to the servo bracket."""
    return [q for q in deck_screws() if abs(q[0] - servo_anchor_x) < 1e-6]

def both_hands(pts):
    """A screw pattern and its X mirror, without duplicates.

    The decks and the walls are ONE part each, used by all four axes, and
    alternate axes are assembled turned over (see axis_shape) — which mirrors
    their brackets in X. So each deck and each wall carries both hands of the
    pattern. A boss nothing screws into costs a gram."""
    out = list(pts)
    for sx, sy in pts:
        if all(abs(sx + qx) > 1e-6 or abs(sy - qy) > 1e-6 for qx, qy in out):
            out.append((-sx, sy))
    return out


def frame_deck_screws():
    """The deck screws that belong to a four-bar frame post, not to the servo."""
    return [q for q in deck_screws() if abs(q[0] - servo_anchor_x) > 1e-6]


def deck_screws():
    """(x, y) of the screws into one deck, for ONE axis, before the axis is
    rotated into place. The deck carries the four-bar's frame pivot now — these
    are what hold the machine's own reaction — and the servo stands on it too.

    The servo's only exists on the bottom deck; putting it in both is harmless
    (a spare boss) and keeps the two decks the same part."""
    out = [(dx, fb_A[0]) for dx in (-fp_screw_x, fp_screw_x)]
    # The servo bracket's two deck screws. They go straight up through its own
    # plate, which is inboard of the tabs — the crank's swept circle is 10.3 mm
    # in radius and is nowhere near it in X any more.
    # Spread along the bracket, and kept OUT of the middle of the box: the
    # four axes' decks are one part, and screws this close to the centre line
    # land on each other once the pattern is rotated 90 deg. One pad is under
    # the case (it is what the case rests on), the other under the plate's far
    # leg.
    out += [(servo_anchor_x, crank_hub_y - 4.0),
            (servo_anchor_x, crank_hub_y + 8.0)]
    return out


def make_deck(zs):
    """Floor or ceiling. They did not exist: the cube was four walls and open
    top and bottom, which was fine while nothing needed them — but the four-bar
    brackets do. Its pivot takes the link reaction, which at 50 deg is mostly Z,
    and a post standing on a deck carries that in compression instead of as
    bending along 35 mm of plate cantilevered off a wall.

    Bosses inward, blind holes, same as the walls: nothing goes through, so the
    outside stays clean."""
    z_in = zs * cube_half
    z_out = zs * (cube_half + deck_t)
    deck = Part.makeBox(2 * cube_out, 2 * cube_out, deck_t,
                        v(-cube_out, -cube_out, min(z_in, z_out)))
    deck = deck.cut(cyl(shaft_d / 2.0 + wall_shaft_clr, deck_t + 2,
                        v(0, 0, min(z_in, z_out) - 1)))
    for _, rot in AXES:
        for sx, sy in both_hands(deck_screws()):
            pt = App.Vector(sx, sy, 0)
            pt = App.Rotation(App.Vector(0, 0, 1), rot).multVec(pt)
            base = v(pt.x, pt.y, z_in - zs * deck_boss_h)
            deck = deck.fuse(cyl(wall_boss_d / 2.0, deck_boss_h, base,
                                 v(0, 0, zs)))
            deck = deck.cut(cyl(foot_tap_d / 2.0, deck_boss_h + 2.0, base,
                                v(0, 0, zs)))
    return deck


def make_wall():
    """One cube wall. It stops at the neighbour's inner face on one side and
    runs out to the full corner on the other, so the four of them interlock in a
    pinwheel: no overlap and no gap, and the same part four times. Spanning the
    full width both ways -- which is what it did while the wall sat outside the
    interference check -- put 2320 mm3 of two walls inside each other, one
    corner post per corner."""
    # Stops at the decks and lets them cap it, rather than running the full
    # height and living inside them — the same mistake as the corners, 2163 mm3
    # of two parts in the same place.
    wall = Part.makeBox(cube_out + cube_half, wall_thick, 2 * cube_half,
                        v(-cube_out, cube_half, -cube_half))
    wall = wall.cut(cyl(shaft_d / 2.0 + wall_shaft_clr, wall_thick + 2,
                        v(0, cube_half - 1, 0), Y_AXIS))
    # Bosses on the INNER face, blind. Nothing passes through, so the outside
    # of the cube stays a clean surface.
    for sx, sz in both_hands(wall_screws()):
        wall = wall.fuse(cyl(wall_boss_d / 2.0, wall_boss_h,
                             v(sx, wall_face_y, sz), Y_AXIS))
        wall = wall.cut(cyl(foot_tap_d / 2.0, wall_boss_h + 2.0,
                            v(sx, wall_face_y, sz), Y_AXIS))
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
    pinion_shape = _gear_shape(doc, "external", Zp, axle_hole=False,
                               height=f"{pinion_face_w} mm")
    idler_shape = _gear_shape(doc, "external", Zi, axle_hole=True,
                              axle_holesize=f"{idler_axle_d + 0.6} mm",
                              height=f"{pinion_face_w} mm")
    corona_shape = _gear_shape(doc, "internal", Zc,
                               thickness=f"{corona_rim_t} mm")
    pinion_shape = _to_gear_plane(pinion_shape, 0.0, 0.0)
    corona_shape = _to_gear_plane(corona_shape, 0.0, 0.0)
    idlers = {sd: _to_gear_plane(idler_shape, sd * idler_x, 0.0)
              for sd in IDLER_SIDES}

    pinion_shape, _p = phase_gear(pinion_shape, 0.0, 0.0, r_pitch_p, Zp, 0.0)
    # AFTER the phasing: that rotates the pinion about its own axis, and a D cut
    # before it would have its flat swung off the shaft's. Physically this is
    # the right order too — the flat is filed to suit, and the teeth land where
    # the mesh needs them.
    pinion_shape = pinion_shape.cut(d_bore(gear_y0 - 1, pinion_face_w + 2))
    _ph = [_p]
    for sd in IDLER_SIDES:
        idlers[sd], _i = phase_gear(idlers[sd], sd * idler_x, 0.0, r_pitch_i, Zi,
                                    180.0 + 180.0 / Zi)
        _ph.append(_i)
    corona_shape, _c = phase_gear(corona_shape, 0.0, 0.0, r_pitch_c, Zc, 0.0)
    gear_phase_ok = None not in (_ph + [_c])
else:
    pinion_shape = cyl(r_tip_p, pinion_face_w, v(0, gear_y0, 0), Y_AXIS)
    pinion_shape = pinion_shape.cut(d_bore(gear_y0 - 1, pinion_face_w + 2))
    corona_shape = (cyl(r_corona_outer, gear_face_w, v(0, gear_y0, 0), Y_AXIS)
                    .cut(cyl(r_tip_c, gear_face_w + 2, v(0, gear_y0 - 1, 0), Y_AXIS)))
    idlers = {sd: cyl(r_tip_i, pinion_face_w, v(sd * idler_x, gear_y0, 0),
                      Y_AXIS)
              for sd in IDLER_SIDES}

# Corona + back plate + output shaft: one part, as in v5 — a hub joint here
# would only add a failure point on the torque path.
coronashaft = corona_shape.fuse(make_corona_plate()).fuse(make_output_shaft_ref())

# ── Carriage, built at rest then moved by the linkage ────────────────────────
CARRIAGE_REST = [
    ("OutputCone",    make_output_cone(),                     (0.20, 0.80, 0.60), 0),
    ("OutputRubber",  make_output_rubber(),                   (0.15, 0.15, 0.18), 0),
    ("Carriage",      make_carriage(),                        (0.70, 0.70, 0.72), 0),
    ("PushPin",       make_trunnion(),                        (0.60, 0.60, 0.60), 0),
] + [
    ("CarriageShaft", make_carriage_shaft(),                  (0.60, 0.60, 0.60), 0),
    ("Pinion",        pinion_shape,                           (0.85, 0.65, 0.10), 0),
    ("BearingCone",   make_carriage_bearing(hous_y0 + brg_seat_lip, 1),
                                                              (0.30, 0.30, 0.32), 0),
    ("BearingPinion", make_carriage_bearing(hous_y1, -1),     (0.30, 0.30, 0.32), 0),
]

# ── Fixed to the frame ──────────────────────────────────────────────────────
FIXED_PARTS = [
    ("CoronaShaft",   coronashaft,                            (0.55, 0.55, 0.85), 0),
] + [
    (f"Idler{'P' if sd > 0 else 'N'}", idlers[sd], (0.30, 0.55, 0.85), 0)
    for sd in IDLER_SIDES
] + [
    (f"IdlerBracket{'P' if sd > 0 else 'N'}", make_idler_bracket(sd),
     (0.65, 0.65, 0.68), 0)
    for sd in IDLER_SIDES
] + [
    (f"IdlerPin{'P' if sd > 0 else 'N'}", make_idler_pin(sd),
     (0.60, 0.60, 0.60), 0)
    for sd in IDLER_SIDES
] + [
    ("FramePostT",    make_frame_bracket(1),                  (0.75, 0.75, 0.78), 0),
    ("FramePostB",    make_frame_bracket(-1),                 (0.75, 0.75, 0.78), 0),
    ("ServoBody",     make_servo_body(),                      (0.20, 0.25, 0.30), 0),
    ("ServoBracket",  make_servo_bracket(),                   (0.75, 0.75, 0.78), 0),
    ("Wall",          make_wall(),                            (0.45, 0.55, 0.75), 70),
] + [
    (f"ServoScrew{i}",
     make_screw(v(servo_face_x - servo_tab_t, sy, crank_hub[1]),
                v(1, 0, 0), servo_screw_d, servo_tab_t + servo_plate_t,
                3.8, 2.0),
     (0.45, 0.45, 0.50), 0)
    for i, sy in enumerate(servo_screw_ys())
] + [
    (f"WallScrew{i}",
     make_screw(v(sx, _screw_seat_y(sx), sz), v(0, 1, 0), foot_screw_d,
                wall_face_y - _screw_seat_y(sx) + wall_boss_h + 1.5,
                bolt_head_d, bolt_head_h),
     (0.45, 0.45, 0.50), 0)
    for i, (sx, sz) in enumerate(wall_screws())
] + [
    (f"DeckScrew{'T' if zs > 0 else 'B'}{i}",
     make_screw(v(sx, sy, zs * (cube_half - deck_boss_h - _deck_seat_t(sx))),
                v(0, 0, zs), foot_screw_d,
                _deck_seat_t(sx) + deck_boss_h + 1.5,
                bolt_head_d, bolt_head_h),
     (0.45, 0.45, 0.50), 0)
    # The servo's two only exist on the deck the servo is on. Its BOSS is in
    # both decks — the two decks are one part — but a screw where there is no
    # bracket is not a spare, it is a part standing in the next axis's way:
    # alternate axes are turned over, so the neighbour's servo is against the
    # other deck and its bracket lands exactly there.
    for zs in (1, -1) for i, (sx, sy) in enumerate(deck_screws())
    if zs < 0 or abs(sx - servo_anchor_x) > 1e-6
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

for _zs, _tag in ((1, "Top"), (-1, "Bottom")):
    add(doc, f"Deck{_tag}", make_deck(_zs), color=(0.45, 0.55, 0.75),
        transparency=70)

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
_MESH_PAIRS = {("Pinion", "IdlerP"), ("Pinion", "IdlerN")}


def _exempt(na, nb):
    """Pairs that are MEANT to share volume.

    Meshing gears, drawn at their free-pose phase (the mesh itself is checked
    at free, and a phase is proven to exist at tilt). And a self-tapping screw
    in its pilot hole: the screw is Ø3 into a Ø2.6 pilot, and that difference
    is the thread it forms. Everything else that overlaps is a mistake."""
    if (na, nb) in _MESH_PAIRS or (nb, na) in _MESH_PAIRS:
        return True
    for host, screw in (("Wall", "WallScrew"), ("DeckTop", "DeckScrewT"),
                        ("DeckBottom", "DeckScrewB")):
        if {na, nb} == {host} | {n for n in (na, nb) if n.startswith(screw)}:
            return True
    return False


def _bb_hit(a, b):
    A, B = a.BoundBox, b.BoundBox
    return (A.XMin < B.XMax and B.XMin < A.XMax
            and A.YMin < B.YMax and B.YMin < A.YMax
            and A.ZMin < B.ZMax and B.ZMin < A.ZMax)


# Parts shared by every axis: they are built once, so they are not in
# AXIS_PARTS and the all-pairs loop never sees them. The decks landed here too —
# the same blind spot the wall was in.
_SHARED = [("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up),
           ("DeckTop", make_deck(1)), ("DeckBottom", make_deck(-1))]
_carr_mot_who = "-"
_pair_ov, _pair_who = 0.0, "-"
_carr_mot_ov = 0.0
for _tag, _phi_t in (("free", 0.0), ("up", phi_preload), ("dn", -phi_preload)):
    _parts = moving_parts(pose_state(_phi_t)) + FIXED_PARTS
    for _i in range(len(_parts)):
        _na, _sa = _parts[_i][0], _parts[_i][1]
        for _j in range(_i + 1, len(_parts)):
            _nb, _sb = _parts[_j][0], _parts[_j][1]
            if _exempt(_na, _nb):
                continue
            if not _bb_hit(_sa, _sb):
                continue
            _ov = _sa.common(_sb).Volume
            if _ov > max(_pair_ov, 1e-6):
                _pair_ov, _pair_who = _ov, f"{_na} x {_nb} at {_tag}"
        for _shn, _shs in _SHARED:
            if _bb_hit(_sa, _shs) and not _exempt(_na, _shn):
                _ov = _sa.common(_shs).Volume
                if _ov > _carr_mot_ov:
                    _carr_mot_ov, _carr_mot_who = _ov, f"{_na} x {_shn}"


# The link, swept, as a DISTANCE. Everything above is "do they overlap", at
# three stops. Both of those hid the same fault, and it took the two of them:
#
#   - the minimum is INTERIOR to the stroke. The four-bar's drift carries the
#     carriage outward while the tilt carries the link inward, the two nearly
#     cancel, and what is left is a shallow bowl with its floor at about half
#     travel. The three stops sample the rim of that bowl, never its floor.
#   - and at the floor the parts still did not overlap. They cleared by
#     0.077 mm, which "== 0 mm3" passes and a printer does not: it is finer
#     than the wall roughness on either face.
#
# So this one sweeps, and it reports millimetres. The link is the part worth
# the cost — it is the only thing that swings through the gap between the
# motor cone's rim and the frame post, and its knuckle crosses x = 0, where
# the cone is widest.
_LINK_CONES = [("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up)]
_LINK_POSTS = [(_n, _s) for _n, _s, _c, _t in FIXED_PARTS
               if _n.startswith("FramePost")]
_link_gap, _link_who = 1e9, "-"
for _k in range(-4, 5):
    _phi_k = phi_preload * _k / 4.0
    _st_k = pose_state(_phi_k)
    for _A_k, _Bk in ((A1, "B1"), (A2, "B2")):
        # Whole link against the cones; knuckle only against the posts, since
        # the arms are meant to run close there — see make_link_knuckle.
        for _sub, _against in ((make_link(_A_k, _st_k[_Bk]), _LINK_CONES),
                               (make_link_knuckle(_A_k, _st_k[_Bk]), _LINK_POSTS)):
            for _n, _s in _against:
                _d = _sub.distToShape(_s)[0]
                if _d < _link_gap:
                    _link_gap = _d
                    _link_who = f"{_n} at {math.degrees(_phi_k):+.1f} deg"


# Gear mesh, checked at FREE where the phasing is exact. Under tilt the pinion
# simply rolls to whatever phase the mesh needs, so an overlap measured there
# would be an artefact, not a collision — what tilt really costs is centre
# distance, reported separately below.
_pin_free = pinion_shape
_mesh_ext = sum(_pin_free.common(idlers[sd]).Volume for sd in IDLER_SIDES)
_mesh_int = sum(corona_shape.common(idlers[sd]).Volume for sd in IDLER_SIDES)
_pin_corona_gap = _pin_free.distToShape(corona_shape)[0]

# "They do not overlap" is ALSO true of two gears too far apart to touch, which
# is how a jammed train went unnoticed. Both meshes must actually touch.
_mesh_touch = max([_pin_free.distToShape(idlers[sd])[0] for sd in IDLER_SIDES]
                  + [corona_shape.distToShape(idlers[sd])[0]
                     for sd in IDLER_SIDES])

# And a phase has to EXIST that clears every idler at full tilt. With one idler
# there always is one; with two the loop is closed, the pinion's 2.4 mm of
# travel breaks it, and no rotation satisfies both.
_st_tilt = pose_state(-phi_preload)
_pc_tilt = _st_tilt["T"]((gear_y_mid, 0.0))
def _jam_at(angle):
    sh = place_carriage(pinion_shape, _st_tilt)
    sh.rotate(v(0, _pc_tilt[0], _pc_tilt[1]), Y_AXIS, angle)
    return max(sh.common(idlers[sd]).Volume for sd in IDLER_SIDES)


# Coarse over one tooth pitch, then fine around the best: on a 5 deg grid even a
# perfectly meshing single idler reads ~0.3 mm3, purely because no sample lands
# on the exact phase.
_best_a, _jam = min(((float(k), _jam_at(float(k))) for k in range(0, 45, 5)),
                    key=lambda t: t[1])
for _k in range(-9, 10):
    _a = _best_a + _k * 0.5
    _val = _jam_at(_a)
    if _val < _jam:
        _best_a, _jam = _a, _val

# Pinion travel under tilt: the idlers are on X for a reason (invariant 6).
_pin_c_max = carriage_transform(phi_preload)[0]((gear_y_mid, 0.0))
_e_tilt = math.hypot(idler_x, _pin_c_max[1])
_e_growth = _e_tilt - e_ext
_e_if_z = abs(idler_x - _pin_c_max[1])     # what an idler ON Z would have seen

# What the actuator actually pulls against. R_push is the moment arm of a
# VERTICAL push, and the link is not vertical: it comes up off the floor at
# 25 deg. The arm of any force is the perpendicular distance from the apex to
# its LINE of action, so measure that instead of assuming.
#
# It comes out at essentially R_push anyway, and not by luck: the push point
# sits 30 mm below the axis, so the link's large Y component earns its own
# moment about the apex — 27 mm of the 49 — and very nearly makes up for the
# small Z one. Pulling the servo inboard along the floor costs the lever almost
# nothing, which is why it could be pulled inboard at all.
def push_lever(phi_t):
    """Perpendicular distance, apex to the actuation link's line of action."""
    st = pose_state(phi_t)
    (py, pz), (qy, qz) = st["pin"], st["tab"]
    dy, dz = qy - py, qz - pz
    return abs(py * dz - pz * dy) / math.hypot(dy, dz)


_lever = [push_lever(_s) for _s in (0.0, -phi_c, -phi_preload,
                                    phi_c, phi_preload)]
# Two things matter about it: that it is not small (it IS the output torque,
# newton for newton) and that it does not move much over the stroke, or the
# preload force would depend on where in the sweep the servo happens to stop.


# Every other axis, not just the next one. The four of them share one box.
_mech = None
for _n, _s, _c, _t in AXIS_PARTS:
    if _n == "Wall":
        _wall_shape = _s
        continue
    _mech = _s.copy() if _mech is None else _mech.fuse(_s)


def axis_shape(k):
    """Axis k of the pinwheel, as one shape.

    ALTERNATE AXES ARE TURNED OVER — 180 deg about their own Y. That is a
    rotation, not a mirror, so it is the same printed parts assembled the other
    way up, and the motor cones it meshes with are symmetric in Z, so nothing
    about the drive changes. What it buys is the floor: the actuation (servo,
    crank, link, horn) all lives against one deck, and two neighbouring axes
    that both wanted the same corner of it now use opposite ends of the box.
    Without it the servo cases overlap by a cubic centimetre and there is no
    arrangement of this box that avoids it — the case is 29 mm along the shaft
    and the crank has to sit beside the horn, so it reaches across the middle.

    The WALL is not turned with the mechanism: the four walls interlock in a
    pinwheel and only fit one way up. The decks and walls therefore carry both
    hands of every bracket's screw pattern, see both_hands()."""
    mech = _mech.copy()
    if k % 2:
        mech.rotate(ORIGIN, Y_AXIS, 180.0)
    shape = mech.fuse(_wall_shape)
    shape.rotate(ORIGIN, Z_AXIS, 90.0 * k)
    return shape


_asm = axis_shape(0)
_adj_overlap = 0.0
_adj_worst = 0
for _k in (1, 2, 3):
    _v = _asm.common(axis_shape(_k)).Volume
    if _v > _adj_overlap:
        _adj_overlap, _adj_worst = _v, _k

# Output cone tip vs the motor shaft it points at.
_tip_clr = out_tip_y - shaft_d / 2.0

# Can the crank and link actually reach every stop? Swept rather than checked at
# the three stops alone, because it is the ENDS of the stroke that fail first.
# A silent fallback here once hid a linkage that could not reach its own travel.
_reach = []
for _k in range(41):
    _z = -act_amp + 2.0 * act_amp * _k / 40.0
    _d = math.dist(crank_hub, (R_push, push_z + _z))
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
    ("bearing shoulder inside the housing  (mm)",
     hous_y1 - hous_y0 - 2 * brg_w, "> 0", hous_y1 - hous_y0 - 2 * brg_w > 0),
    ("actuation lever about the apex, smallest of every stop  (mm)",
     min(_lever), "> 35", min(_lever) > 35.0),
    ("...and how much it MOVES over the stroke  (mm)",
     max(_lever) - min(_lever), "< 3", max(_lever) - min(_lever) < 3.0),
    ("crank + link reach the whole stroke  (mm to dead centre)",
     _reach_margin, "> 0.1", _reach_margin > 0.1 and not _UNREACHABLE),
    ("FREE: rubber clear of both motor cones  (mm)",
     _free_gap, "> 0.10", _free_gap > 0.10),
    ("plastic never touches plastic  (mm3)",
     _plastic_ov, "== 0", _plastic_ov < 1e-6),
    (f"every pair of parts, every stop [{_pair_who}]  (mm3)",
     _pair_ov, "== 0", _pair_ov < 1e-6),
    (f"nothing on the axis touches the shared parts [{_carr_mot_who}]  (mm3)",
     _carr_mot_ov, "== 0", _carr_mot_ov < 1e-6),
    # A printed clearance, not a mathematical one: 1 mm is about two perimeters
    # plus the pin's slop. Anything under that is touching once it is a part.
    (f"link clearance, SWEPT not stopped [{_link_who}]  (mm)",
     _link_gap, "> 1.0", _link_gap > 1.0),
    ("pinion concentric inside the corona, no mesh  (mm)",
     _pin_corona_gap, "> 1.0", _pin_corona_gap > 1.0),
    ("pinion/idler mesh at FREE, no jam  (mm3)",
     _mesh_ext, "== 0", _mesh_ext < 1e-6),
    ("every mesh actually TOUCHES, not merely misses  (mm)",
     _mesh_touch, "< 0.05", _mesh_touch < 0.05),
    # Tolerance, not zero: this is a swept minimum on a 0.5 deg grid, and the
    # residual there is ~0.003. The failure it exists to catch is three orders
    # bigger — two idlers leave 4.1 mm3 at the best compromise phase.
    ("a pinion phase exists that clears every idler at full tilt  (mm3)",
     _jam, "< 0.05", _jam < 0.05),
    ("idler/corona mesh at FREE, no jam  (mm3)",
     _mesh_int, "== 0", _mesh_int < 1e-6),
    (f"every built shape is a valid solid  {_invalid if _invalid else ''}",
     len(_invalid), "== 0", not _invalid),
    (f"every part is ONE connected solid  {_loose if _loose else ''}",
     len(_loose), "== 0", not _loose),
    (f"every other axis, alternate ones turned over [worst: {_adj_worst*90}"
     f" deg]  (mm3)",
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
      f"   servo-side travel {act_amp:.2f} mm,"
      f" split 1:{spring_split-1:.1f}")
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
print(f"    m = {m_mod:.2f}  Zp {Zp} -> Zi {Zi} (x{len(IDLER_SIDES)}, on X)"
      f" -> Zc {Zc}"
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
# Moments about the apex give  F*lever = INTEGRAL n(s)*s ds, and the output
# torque is  mu * INTEGRAL n(s)*(s*sin beta) ds  =  mu*sin(beta)*F*lever:
# the contact's position cancels out exactly. That is the same property of the
# apex pivot that matches the surface speeds — every quantity scales with s.
# What lever/s_bar really gives is the total NORMAL force, which is what has
# to be developed inside the available squeeze (open question 1), not torque.
#
# `lever`, not R_push. R_push would be the arm if the push were vertical, and
# this one is not: it arrives along the link.
_mu_ref = 1.3
print(f"    the LINK's lever about the apex is {min(_lever):.1f}..{max(_lever):.1f}"
      f" mm, against R_push = {R_push:.1f}. The link comes up off the floor at a"
      f" shallow angle, but the push point is {abs(push_z):.0f} mm below the"
      f" axis, so the link's Y component earns a moment of its own and very"
      f" nearly makes up for what its Z component gives up.")
print(f"    normal force per newton of actuator force: lever / s_bar ="
      f" {min(_lever) / _sq_sbar:.2f}"
      f"  (s_bar = {_sq_sbar:.1f} mm, the squeeze's own centroid, not the"
      f" band's midpoint)")
print(f"    output torque per newton: mu*sin(beta)*lever*(Zc/Zp) ="
      f" {_mu_ref * math.sin(beta) * min(_lever) / 1000.0 * (Zc / Zp):.4f} Nm/N"
      f" at mu = {_mu_ref}")
print(f"    — and that does NOT depend on where along the generatrix the"
      f" contact sits: more radius buys")
print(f"      more friction torque and costs exactly as much normal force."
      f" Only F and the lever move it.")
print("-" * 72)
for label, value, target, ok in checks:
    print(f"  [{'OK ' if ok else 'FAIL'}] {label}:  {value:.3f}  ({target})")
print("-" * 72)
_cbb = BY_NAME["Carriage"].BoundBox
print(f"  Carriage: ONE piece, {_cbb.YLength:.0f} x {_cbb.ZLength:.0f} x"
      f" {_cbb.XLength:.0f} mm, bearing housing inside the hollow cone")
_cube_tied = [k for k, val in cube_half_by.items() if val > cube_half - 0.05]
print(f"  Cube half-size ({cube_half:.1f}) = motor axis to the wall's inner"
      f" face, set by: {' AND '.join(_cube_tied)}")
for _k, _val in sorted(cube_half_by.items(), key=lambda kv: -kv[1]):
    print(f"      {_val:5.1f}  {_k}")
print(f"  CUBE side {2 * cube_out:.1f} mm"
      f"  (half {cube_half:.1f} inside + {wall_thick:.1f} wall)")
print("  PRINTED: MotorCone x2 (same part, flipped), OutputCone (a SHELL),")
print("           Carriage (one piece),")
print(f"           Pinion, Idler x{len(IDLER_SIDES)}, CoronaShaft,"
      f" IdlerBracket x{len(IDLER_SIDES)}, FramePost x2,")
print("           Link x2 (each carries both its arms), Crank, ActLink,"
      " ServoBracket")
print("  ASSEMBLY: alternate axes go in TURNED OVER \u2014 the same parts, rotated")
print("            180 deg about their own radius, so their servo, crank and horn")
print("            lie against the other deck. Two neighbours both wanting the same")
print("            corner of the same floor is the one thing this box cannot fit.")
print(f"  Cone and pinion are keyed to the shaft by a D on a filed flat"
      f" ({shaft_flat_d:.1f} mm across),")
print("  because a set screw fits nowhere: 2.05 mm of pinion hub wall, and"
      " 0.5/1.5 mm of Y")
print("  beyond its faces. Filing the flat is the one manual step here.")
print("  PURCHASED, per axis: 2x MR105ZZ, Ø5 rod (output shaft + the push pin),")
print("             Ø4 pin stock (4 pivot pins),")
print(f"             {len(wall_screws())}x M3x8 through the wall into the"
      f" idler bracket,\n             {len(frame_deck_screws()) * 2}x M3x10 up"
      f" through the decks into the frame posts,"
      f"\n             {len(servo_deck_screws())}x M3x10 into the servo bracket"
      f" (ONE deck only),")
print("             2x M2x6 for the servo tabs, rubber sheet, springs, servo")
print(f"  FDM holes (this printer runs ~0.5 under): shaft Ø{fdm_shaft_hole_d:.1f}"
      f"  pin Ø{fdm_pin_hole_d:.1f}  bearing seat Ø{brg_od + 2*brg_fit_press:.1f}")
print("=" * 72)
if not GEARS_AVAILABLE:
    print("NOTE: freecad.gears not found — pinion/idlers/corona are reference")
    print("cylinders with no teeth. Install the 'Gear' addon for real involutes.")
print("Packaging (carriage arms, idler bracket, frame brackets, servo mount) is")
print("a first pass: the geometry above is derived, the brackets are not.")
