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

# ── Drive out of the cone: a folded ("zig-zag") double cardan ────────────────
# The cone TILTS about the apex and the output shaft does not. At a distance y
# from the apex their axes are apart by y*sin(phi) AND at an angle of phi.
#
# The joint lives INSIDE the hollow cone. The cone itself is the outer yoke (a
# bell): its neck carries a RING cross on pins along X. The ring's other pins
# carry an intermediate TUBE, which runs back toward the apex to a SOLID cross
# on the end of the output shaft, which reaches in through the ring.
#
# Both crosses sit IN FRONT of the apex (the motor shaft is behind it), so the
# two joints can never bend equally: the solid one always bends phi more than
# the ring. The joint is therefore not constant-velocity; it behaves like one
# Hooke joint of phi, ~0.025 deg of error at 2.4 deg. What the fold buys is
# length: the further back the solid cross sits, the flatter the intermediate
# (angle ~ ring_y*sin(phi) / (ring_y - cross_y)) and the smaller the ring's
# bore, which only has to swallow the axes' offset AT the ring.
#
# With no carriage shaft left, the cone runs on ONE thin-section bearing on
# the OUTSIDE of its neck, held by the carriage.

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
#
# v7-cardan-zigzag: ray 45 deg, frame pivot r 55. With the Oldham gone the
# frame post was what set the cube (54.5); at 45 deg and r 55 it drops under
# the horn's 47 with the link at 14.5 mm. Swept on this linkage before
# choosing: ray 40 r 52 gives the same cube at 0.65% slip, this one 0.61%.
_fb_ray = math.radians(45.0)
fb_A = (55.0 * math.cos(_fb_ray), 55.0 * math.sin(_fb_ray))   # frame pivot
fb_B = (40.5 * math.cos(_fb_ray), 40.5 * math.sin(_fb_ray))   # carriage pivot
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
link_knuckle = 4.0    # mm — radius of the knuckle joining the pair into one
                       #      part: the arms' own end radius, so it reads as one
                       #      cylinder, flush with them.
                       #      part.
link_knuckle_d = 14.5 # mm — (v7-cardan-zigzag: ON B. The 14.5 link is too
                       #      short for the frame post's 6 mm lug and a knuckle
                       #      to share it; at B the ray-45 layout keeps it 2 mm
                       #      above the motor cone's rim.)
                       #      where that knuckle sits, measured along the link
                       #      from the FRAME pivot A. It used to sit on the
                       #      carriage pivot B (d = link_len), and that is the
                       #      one place it must not: the knuckle is the only
                       #      part of the link crossing x = 0, so it is the part
                       #      nearest the motor cone's rim, and there it cleared
                       #      it by 0.077 mm — under a printed wall's roughness,
                       #      i.e. touching. Sliding it inboard clears that, but
                       #      17.5 (4 mm off B, the first value tried) put the
                       #      knuckle's own 5 mm radius PAST B — visually fused
                       #      into the arm's own end boss there, even though the
                       #      two are one part and it costs nothing structurally.
                       #      15.0 clears B by 6.5 mm (radius 5, so genuinely
                       #      apart) and still keeps 2.2 mm off the frame post —
                       #      the tighter of the two constraints from here in.
                       #      Swept in the checks below, not assumed.
pin_d      = 4.0      # mm — pivot pin diameter (all four-bar pins)

# ── Locating the carriage in X ──────────────────────────────────────────────
# The four-bar only holds the carriage in the Y-Z plane. Along X it is held by
# nothing but the side faces of its joints: carriage arm against link at B,
# link against the frame lug at A, two joints in series, top AND bottom (with
# only one chain tight the carriage rocks about it). Whatever play is left
# there moves the cone's apex off the motor's by the same amount, and an apex
# offset across the contact plane slips exactly like the four-bar's own drift
# along it (slip velocity w x offset, same magnitude either way): delta / L.
# Every one of those faces is printed shim_gap short and filled with shim
# washers at assembly — the printer's +-0.2 cannot be trusted to hit a
# tenth-of-a-millimetre fit on its own.
shim_gap   = 0.3      # mm — printed gap at each thrust face, nominal
shim_od    = 8.0      # mm — DIN 988 4x8 shim washer (0.1 / 0.2 / 0.3 / 0.5)
shim_id    = 4.1      # mm — modelled a hair over the pin so they do not overlap
x_play_max = 0.08     # mm — carriage X play allowed after shimming: its slip,
                       #      x_play_max / L_line, must not beat the four-bar's

# ── Actuation: servo → torsion spring → crank → ONE link → ear ─────────────
# Everything lives in ONE vertical plane per part (Y-Z), stacked in X, in the
# band between the carriage's top and the ceiling, in this axis' own corner of
# the pinwheel (+X). All four axes are identical rotated copies: no part is
# turned over any more, so each servo has a corner of its own.
#
# The servo spline turns sv_theta_max each way. The first sv_theta_c of it
# brings the rubber into contact; past that the carriage has nowhere to go and
# the rest of the sweep winds a torsion spring between spline and horn, which
# is what sets the squeeze. The HORN therefore only ever swings ~sv_theta_c.
#
# 2026-09-20: the lever and its link are GONE (user: simplify the chain). A
# small crank straight on the spline pulls the ear through one link, so the
# chain is 2 pins instead of 5, and the lever, its ceiling post, link1, three
# pins and three clips all go with it. Nothing is lost doing it: a 9.04 crank
# through a 2.1:1 lever gave the ear an effective radius of 4.29 mm, and a
# crank of horn_r alone gives 4.35 — same force, same travel, same spring.
# The worst transmission angle improves from 41.7 to about 66 deg, and the
# stack loses two plates and two gaps, which is 6 mm off the carriage in X.
                        #      (sv_theta_c — the sweep to contact — is DERIVED
                        #       from the crank now; see below)
sv_theta_max  = 80.0   # deg — servo sweep, free → full preload (margin to the
                        #       MG90's own ~90 deg end stops)
sv_stall      = 0.18   # Nm  — MG90-class stall torque; the spring is sized to
sv_use        = 0.6    #       reach this fraction of it at full preload.
                        #       Was 0.8 (user, 2026-09-21): the datasheet's
                        #       stall is a best case at 4.8 V, cheap servos
                        #       come in under it, and the supply sags just
                        #       when the servo pulls hardest.
spring_ratio  = 15.0   # —   — rubber stiffness / spring stiffness, both seen as
                        #       servo-side angle past contact. Invented (doc
                        #       §10.2): it only sets how far past contact the
                        #       carriage still creeps, i.e. phi_preload.
servo_body    = (28.2, 22.4, 12.5)   # mm — MG90D, off its own datasheet
                        #      (user, 2026-09-20): the shaft direction first,
                        #      then the case's length and its thickness. The
                        #      CASE only: the datasheet's 32.2 runs to the top
                        #      of the spline, and the spline is the last 4.
sv_spline_h   = 4.0    # mm — the splined shaft standing off the case
sv_case_h     = 22.4   # mm — the MAIN case is only this tall (datasheet). The
                        #      rest of servo_body[0], up to 28.2, is the gear
                        #      TOWER round the shaft, and it is only
sv_tower_len  = 12.2   # mm — this long, at the shaft's own end (user,
                        #      2026-09-21). The other end of the case has no
                        #      body above sv_case_h — which is exactly where
                        #      the carriage block's corner comes.
sv_shaft_end  = 6.25   # mm — the shaft sits this far from the case's SIDE,
                        #      measured on the body alone, tabs not counted
                        #      (user, 2026-09-20). On a 22.4 case that is
                        #      4.95 off the centreline, not the 1.5 read off
                        #      the drawing before.
sv_shaft_off  = servo_body[1] / 2.0 - sv_shaft_end   # DERIVED
sv_tab_from_base = 18.0  # mm — from the case's base UP to the tab plate
                        #      (datasheet). Measured from the base now, not
                        #      from the shaft end, because the base is what
                        #      the bracket holds.
_servo_body_old = (29.0, 22.8, 12.2)   # mm — MG90-class: X along the shaft,
                        #      then the case's LENGTH and its thickness.
                        #      2026-09-20: the servo lies against the WALL, not
                        #      under the ceiling (user). So the length stands up
                        #      the wall in Z and the thickness is what it takes
                        #      out of the cube in Y; the shaft still points +X,
                        #      so the crank still sweeps a Y-Z plane and the
                        #      chain's kinematics are untouched.
sv_face_x     = -42.0  # mm — X of the case's shaft face. The shaft points
                        #      −X and the case runs +X from it, so its BASE
                        #      ends up beside the four-bar (user, 2026-09-20)
                        #      and the whole chain lives in the corner behind
                        #      the shaft, with 8.3 mm of clear X to itself —
                        #      instead of sharing 8.8 mm with the four-bar.
                        #      of it. NEGATIVE: the servo lives in this axis'
                        #      −X corner. It does not fit in the +X one — the
                        #      carriage's own ring leaves 23.8 mm there against
                        #      the case's 29, and the pinwheel makes the −X end
                        #      of the same wall 4 mm deeper. The chain and the
                        #      ear move to the carriage's −X arm with it.
sv_z0         = -47.0  # mm — case bottom, down by the floor. STANDING: its
                        #      22.8 length lies along Y, into the cube, and it
                        #      is only 12.2 tall, so the carriage's own ear
                        #      bridge passes clear OVER it.
sv_wall_clr   = 0.5    # mm — case back to the wall
servo_tab_t   = 2.5    # mm — thickness of the servo's own mounting tabs
sv_cable_h    = 6.0    # mm — the cable's slot in the cradle: this tall from
                        #      the floor (MG90D: the lead leaves the case's
                        #      SHAFT end, low — user, 2026-09-21)
servo_tab_out = 4.7    # mm — how far each tab reaches past the body
servo_screw_d = 2.0    # mm — M2 through the tabs into the bracket
horn_t        = 1.8    # mm ┐ X stack: spring and crank outboard of the
act_gap       = 0.4    # mm │ servo's face; the link and the ear stacked back
lk_t          = 1.8    # mm ┘ INBOARD under it — see x_link below
spring_len    = 2.0    # mm — torsion spring envelope along X. Short: the
                        #      whole chain lives in the 7.8 mm between the
                        #      shaft face and the neighbour's wall.
spring_r      = 5.5    # mm — its outer radius
sv_stand      = 0.0    # mm — how far the cradle lifts the servo off the
                        #      floor. None now (user, 2026-09-21): standing
                        #      on the floor itself puts its tab 2 mm further
                        #      from the carriage's lug, which the arms' new
                        #      bosses had brought to 0.65 mm.
screw_x       = -37.7  # mm ┐ the lead screw's axis: up the empty column
screw_y       = 41.5   # mm ┘ beside the carriage's arm, clear of its ring
screw_d       = 8.0    # mm — T8 lead screw (the 3D-printer standard part)
screw_lead    = 8.0    # mm per turn — T8 is 4-start, so a turn is 8 mm. This
                        #      is what makes a ±80 deg servo enough: 160 deg
                        #      of it is 3.6 mm at the nut, against the 1.9 the
                        #      ear needs.
screw_eff     = 0.5    # — thread efficiency at that lead (steel on brass).
                        #      It is high BECAUSE the lead is coarse; the same
                        #      coarseness is why it does not self-lock.
# The nut is the rectangular-flange one from a T8 anti-backlash kit for the
# Ender 3 (CESFONJER, user, 2026-09-21) — its spring and its second nut are
# not used. Its flange is 22 x 10.5, so it fits the 16.6 mm the cage has in
# Y without filing, and its two holes both hold it and stop it turning.
nut_d         = 10.0   # mm — the brass nut's body
nut_l         = 15.0   # mm — overall, flange included
nut_fl        = (22.0, 10.5, 4.0)   # mm — flange: along X, along Y, thick
nut_fl_hole   = 2.6    # mm — its two holes (likely tapped M3 — check it)
nut_fl_pitch  = 16.0   # mm — between them, along X
guide_dy      = -7.8   # mm — the rod stands this far from the screw in Y,
                        #      toward the motor: JUST enough for the nut's
                        #      whole flange to clear it (0.6 mm at its corner),
                        #      so the flange keeps both ears and both screws
                        #      (user, 2026-09-21: no cutting the nut). Moving
                        #      it out in X instead was not on — the servo's
                        #      tab is already 0.4 off the neighbour's wall, and
                        #      the rod's foot meets the carriage's ring.
guide_y       = screw_y + guide_dy   # DERIVED — the rod's own Y
push_t        = 4.0    # mm — the pusher arm from the nut out to the ear
push_w        = 14.0   # mm — the carrier's arm. It has to STRADDLE the
                        #      screw: the rod is on one side of it and the ear
                        #      on the other, so the arm carries a clearance
                        #      hole for the screw and needs material round it.
# The series spring lives at the END of the pusher, as a cartridge: the ear's
# pin rides in a slot with a spring stack above it and another below, so the
# nut can push the carriage BOTH ways through a spring. It has to be both
# ways — free is the middle of the travel — which is why it is two stacks and
# not one spring.
spr_od        = 11.0   # mm — outside Ø of each stack. They sit round the
                        #      GUIDE ROD, not round the screw: at the screw
                        #      the wall is 7 mm away and the brass nut itself
                        #      is 14 across, so nothing fits round it.
spr_id        = 5.0    # mm — bore, over the Ø4 rod
carrier_t     = 6.0    # mm — the floating carrier's arm. At 4 it was a
                        #      21 mm cantilever at 32 MPa, which printed PLA
                        #      does not have to give; at 6 it is 14.
carrier_bush  = 12.0   # mm — and its bush on the guide rod is this long. The
                        #      rod is what reacts the moment the arm makes
                        #      (57 N over 21 mm), and a 4 mm bush would have
                        #      seen 112 MPa of edge pressure doing it; 12 mm
                        #      brings that to 12.
pin_boss_t    = 8.0    # mm — it thickens to this round the ear pin's slot
pin_boss_x    = 5.0    # mm — over this much of its length: short, so it
                        #      stops before the screw the arm passes
cage_t        = 2.5    # mm — the driver's LOWER seat plate
cage_top_t    = 5.0    # mm — its UPPER one, thicker: the nut's flange screws
                        #      down into it, and over the chamber this plate is
                        #      all the thread they get. At 2.5 it was nothing.
spr_h         = 1.8    # mm — its height, seated. Short, because the cage
                        #      has to hold bush, springs and stroke between
                        #      the nut above and the ring below.
spr_clr       = 0.2    # mm — slack in the slot beyond the working stroke
guide_d       = 4.0    # mm — the anti-rotation guide: a Ø4 rod beside the
                        #      screw, through a bore in the nut's own pusher.
                        #      Without it the nut just turns with the screw;
                        #      with it, it also takes the moment the pusher
                        #      makes by reaching 15 mm out to the ear, which
                        #      otherwise all lands on the ear's pin.
guide_dx      = 8.5    # mm — the rod sits between the screw and the ear
                        #      (user, 2026-09-20), which is the right way
                        #      round: the carrier's arm then runs 11 mm from
                        #      its bush to the pin instead of 21, and never
                        #      has to cross the screw. The screw goes out into
                        #      the corner and takes the servo with it.
# The coupler between the servo and the screw (user, 2026-09-21). The servo's
# own DOUBLE-arm horn (the symmetric one of the three it ships with) sits in
# a pocket in the coupler's base, its arms cut down to horn_arm_len tip to
# tip: it drives the coupler by its shape and is FREE in Z in there, so the
# servo never takes the screw's thrust. The coupler is gripped on the screw's
# filed flat by a grub, and is the thrust stack's lower collar.
horn_hub_d    = 7.0    # mm — the horn's hub (measured, roughly)
horn_hub_h    = 4.0    # mm — its height
horn_arm_t    = 2.0    # mm — the arms' thickness, flush with the hub's top
horn_arm_w    = 5.0    # mm — their width by the hub
horn_arm_len  = 11.0   # mm — tip to tip, CUT to this: the coupler turns with
                        #      the servo, ±80 deg, so it sweeps a full disc,
                        #      and the wall is 7.6 mm off the screw's axis
horn_lift     = 0.5    # mm — the hub's underside above the case
horn_screw_d  = 2.5    # mm — the horn's screw (measured)
cpl_d         = 13.5   # mm — the coupler's diameter
cpl_floor     = 1.5    # mm — between the horn's pocket and the D bore
cpl_fit       = 0.2    # mm — the horn's pocket, over its outline
cpl_float     = 1.0    # mm — the pocket's roof above the horn: its Z play
cpl_head_h    = 2.0    # mm — the horn screw's head (stock screw)
cpl_top_z     = -6.4   # mm — the coupler's top face: the stack above it
                        #      ends 1 mm under the cage's lowest point (6.7)
screw_flat    = 7.0    # mm — across the D, on the screw's end (filed, -Y)
# The screw's THRUST stop (user, 2026-09-21): nothing may stand above the
# ceiling (a cube can be stacked there) and the wall is 7.6 mm off the screw,
# too close for any 8 mm-bore thrust bearing (Ø16). So a plate on the guide
# rod's foot, pinched between the coupler below and a lock collar above, a
# PTFE washer each side. Costs ~0.03 Nm of friction (~19% of the servo).
tw_od         = 12.0   # mm ┐ PTFE thrust washers, 8 x 12 x 1
tw_t          = 1.0    # mm ┘
tw_mu         = 0.12   # — PTFE on PLA, dry: friction on the loaded washer
tp_t          = 4.0    # mm — the thrust plate
col_d         = 14.0   # mm ┐ T8 lock collar (grub-screw type) — measure it
col_h         = 6.0    # mm ┘
guide_z0      = cpl_top_z + tw_t + 1.0   # DERIVED — the rod starts inside the
                        #      foot, 1 mm above its underside
screw_top_z   = 45.3   # mm — where the screw's top bearing sits, clear
                        #      above the nut's own travel
horn_r        = 4.3    # mm — crank radius, the ONE number that sets the
                        #      chain's ratio now. It buys the ear an effective
                        #      radius of ~4.35 mm (measured below), matching
                        #      the 4.29 the old crank-and-lever gave, so the
                        #      spring and the force at the ear are unchanged.
                        #      Bigger is a lighter servo sweep and less force;
                        #      smaller is the reverse, with the transmission
                        #      angle worsening at both ends (best near 7).
ear_sx        = -1     # — which of the carriage's two arms carries the ear:
                        #      the one the servo is on
ear_y         = 38.0   # mm — Y of the ear pin at rest. Its boss is a disc of
                        #      act_boss_r + 1.5, and at 40 that disc stood 2 mm
                        #      proud of the ring's outer face — which is the
                        #      face the carriage is PRINTED on. At 38 the whole
                        #      part sits flat on the bed. Was where the lever's
ear_z         = 19.0   # mm — on the carriage's arm, HIGH. Three things push
                        #      it there: below z 14.4 the pusher would have to
                        #      reach through the carriage's own ring to get to
                        #      the arm, below z -13 it would be inside the
                        #      servo, and the pin's boss on the carrier needs
                        #      its own height clear of the ring's rim, which
                        #      at that X reaches z 13.9.
                        #      Raising it costs NOTHING in travel: what the
                        #      screw has to push through is ear_y * phi, and
                        #      the height does not enter.
                        #      carriage's own ear and its checks do not move.
ear_w         = 5.0    # mm — ear bridge width
ear_h         = 6.0    # mm — ear bridge height
act_pin_d     = 3.0    # mm — actuation pins
act_boss_r    = 2.5    # mm — boss radius round each actuation pin

# ── Checks ───────────────────────────────────────────────────────────────────
RUN_CHECKS = None   # None = automatic: all the checks when run headless
                    # (freecadcmd), none when opened in the FreeCAD window, so
                    # the model shows in seconds. True / False to force it.
                    # The checks are what take the time: every pair of parts at
                    # three stops, three swept distance checks, the four axes
                    # against each other, and the rubber's real contact point.

# ── Render pose ──────────────────────────────────────────────────────────────
CARRIAGE_STOP = "contact"   # "free" | "contact" | "preload"
CARRIAGE_DIR  = -1          # -1 = tilt DOWN (engages the LOWER motor cone),
                             # +1 = UP. Ignored at "free", which is the middle.

AXES_SHOWN = 1     # 0..4 — output axes built (mechanism + wall). 1 keeps the
                    #        view readable; 4 is the full cube. The central
                    #        motor cones and shaft are always built.

# ── Carriage ─────────────────────────────────────────────────────────────────
# The carriage no longer lives inside the cone: the joint does. It is a RING
# round the cone's neck now, holding the one thin-section bearing the cone runs
# on (6805, 25x37x7), with the arms and the horn rooted on its outside.
nb_id         = 25.0   # mm ┐
nb_od         = 37.0   # mm │ 6805-2RS thin-section bearing on the cone's neck
nb_w          = 7.0    # mm ┘
nb_y0         = 34.0   # mm — bearing, cone-side face (cone frame). Clear of the
                        #      cone's rim at 30.8 by the neck's own shoulder.
hous_lip      = 1.0    # mm — seat left open past the bearing on the cone side
                        #      (the arms' uprights are flush with this face)
hous_flange_ri = 16.0  # mm — the flange the bearing stops against, on the WALL
                        #      side: its bore clears the neck and the inner race.
                        #      It runs solid from the bearing to the ring's
                        #      wall-side face, which is the face the carriage
                        #      prints on, so the stop grows up off the bed.
hous_ro       = 21.5   # mm — housing outer radius (2.3 mm over the seat)
carr_half     = 20.9   # mm — the carriage is a BLOCK now, not a ring (user,
                        #      2026-09-21): this is its half-width in X and Z,
                        #      the least that keeps 2.2 mm of wall over the
                        #      6805's seat on the flats. Its corners reach 29.6,
                        #      which the ring never did, and they fit only
                        #      because the guide rod moved in Y and could then
                        #      come in toward the screw in X.
hous_y0       = nb_y0 - hous_lip          # DERIVED
hous_y1       = nb_y0 + nb_w + 1.0        # DERIVED, 1 mm proud of the bearing
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
arm_up_w      = hous_y1 - hous_y0          # DERIVED — upright width
arm_root      = ((hous_y0 + hous_y1) / 2.0, 5.0)   # DERIVED (v7-cardan-zigzag):
                        #      the upright is centred on the housing ring and
                        #      exactly as wide (arm_up_w), flush with both faces.
                        #      Y is set by the cone's own rim: the arm's first
                        #      leg climbs straight up in Z at this Y, and it has
                        #      to already be past the cone's mouth (30.8, 17.6 mm
                        #      rim) before it turns — an L that turned too soon
                        #      would cut into the cone on its way up. Was a
                        #      single diagonal bar, which is lighter and stiffer
                        #      for the same reason a strut beats a right angle;
                        #      swapped for two straight legs so the shape reads
                        #      at a glance and each face prints flat. Checked
                        #      by the sweep below, not assumed.
bolt_d        = 3.0    # mm ┐
bolt_head_d   = 5.5    # mm │ M3 socket head and nut, as envelopes. Modelled as
bolt_head_h   = 3.0    # mm │ solids and not just as holes, because a hole
bolt_nut_d    = 6.4    # mm │ collides with nothing: it was the HEAD that ran
bolt_nut_h    = 2.4    # mm ┘ into the actuation link, and nothing could see it.

# ── Folded double cardan and output shaft support ─────────────────────────
uj_cross_y    = 26.0   # mm — solid cross centre, on the OUTPUT axis. As far
                        #      back as the cone's cavity lets the intermediate's
                        #      tail go.
uj_ring_y     = 40.0   # mm — ring cross centre, on the CONE axis (cone frame),
                        #      under the neck bearing
neck_ri       = 10.0   # mm — the cone's neck bore = the bell the joint turns in
neck_ro       = nb_id / 2.0   # DERIVED — the neck IS the bearing's seat
neck_shoulder = 1.0    # mm — radial step the bearing's inner race stops on
neck_y1       = 42.0   # mm — end of the neck (cone frame)
uj_ring_ri    = 5.1    # mm — ring bore: the output shaft passes through it,
                        #      offset by ring_y*sin(phi) and tilted inside it
uj_ring_ro    = 7.3    # mm — 2.4 mm of wall for its pins to sit in (was 1.35,
                        #      ~30 MPa on the plastic at 1 Nm; now ~17)
uj_ring_t     = 5.0    # mm — ring thickness along its axis: 1.5 mm of material
                        #      each side of a Ø2 hole (was 3, i.e. 0.5)
uj_mid_ri     = 6.75   # mm — intermediate tube
uj_mid_ro     = 8.25   # mm
uj_mid_end    = 1.5    # mm — tube material past the solid cross's pin plane
# The tube is STEPPED: narrow at its tail, where it sits in the cone's conical
# cavity and cannot grow, and wide only at its head, round the ring. The step
# sits just behind the ring because the tube rocks ~4 deg inside the cone, and
# near the ring that moves it sideways by tenths, further back by more.
uj_head_ri    = 7.8    # mm ┐ tube head: 2.0 mm of wall for the ring's Z pins
uj_head_ro    = 9.8    # mm ┘
uj_head_back  = 3.0    # mm — head starts this far behind the ring's pin plane
uj_head_end   = 3.0    # mm — ...and ends this far past it
neck_ri_head  = 10.5   # mm — the neck's bore round the tube head; 2.0 mm of
                        #      neck wall left for the ring's X pins, which the
                        #      6805 then covers
uj_pin_d      = 2.0    # mm — cross pins (steel, Ø2)
uj_cross_a    = 2.3    # mm — solid cross, half-size in X (between the fork's arms, 0.7 each side)
uj_cross_hy   = 2.0    # mm — ...half-size along the shaft (Y)
uj_cross_hz   = 5.0    # mm — ...half-size in Z: a PRISM, long toward the tube.
                        #      One pin goes right through its narrow X to both
                        #      fork arms (they carry the most load, ~125 N a side
                        #      at 1 Nm); the two short pins to the tube go into
                        #      its long ends, so each sits ~4 mm deep — deeper
                        #      than the 1.75 mm they could slide before the cone
                        #      stops them. Its far corner, at any phase of the
                        #      turn, must still clear the tube's bore.
uj_fork_ri    = 3.0    # mm ┐ output fork arms: |x| between these, straddling the
uj_fork_ro    = 5.0    # mm ┘ cross block and its X pins
uj_fork_back  = 3.5    # mm — fork base starts this far past the cross, wall side
uj_fork_base  = 3.0    # mm — base thickness
uj_slide      = 0.4    # mm — the intermediate's length changes over the stroke
                        #      (the ring moves with the cone, the cross does not):
                        #      its ring-end pin holes are slotted this much along
                        #      the tube instead of making the tube telescopic
run_clr       = 1.0    # mm — running gap between parts moving against each other
out_brg_wall  = 3.0    # mm — material wall around the output shaft's bearing
                        #      seat in the wall. out_brg_len (the seat's total
                        #      length) is derived below, by the carriage
                        #      bearing block — it needs brg_w and brg_seat_lip,
                        #      not known yet here.
shaft_flat_d  = 4.0    # mm — the Ø5 shafts are filed to a flat, leaving this
                        #      across it; printed parts carry the matching D and
                        #      the cardan's grub screws bear on it. Filing it is
                        #      the one manual step the macro cannot check.
shaft_flat_clr = 0.25  # mm — how much the printed D is relieved off the flat

# ── Frame ────────────────────────────────────────────────────────────────────
fp_lug_x      = link_x - link_t / 2.0 - shim_gap   # DERIVED — frame-pivot lug
                        #      half-width in X; the links sit just outboard of
                        #      it, a shim's width away (was 6.0, 0.5 of play)
fp_post_x     = 22.0   # mm — half-width in X of the block: fp_screw_x plus a
                        #      foot_edge margin, so the screws don't sit at the
                        #      block's own free edge
fp_post_y     = 6.0    # mm — half-height (Z) the block adds above and below
                        #      the screw spread and the pivot lug
fp_screw_off  = 6.0    # mm — each screw this far from the pin's axis, one in
                        #      front and one behind: symmetric, so the post is
                        #      clamped evenly about its own pivot.
fp_deck_ys    = (fb_A[0] - fp_screw_off, fb_A[0] + fp_screw_off)   # mm — Y of
                        #      the two screws that hold each
                        #      frame post: one IN FRONT of the pin (y 38.9), one
                        #      BEHIND it, both on the post's centreline. That is
                        #      as far apart as they can go (user, 2026-09-21):
                        #      spread in X they would leave the post, which is
                        #      only the lug's width because the link arms swing
                        #      either side of it. Each clears the pin's bore, so
                        #      it can go deep beside the pin.
fp_chamfer    = 3.5    # mm — legs of the 45 deg chamfer on the post's front
                        #      edge, over the link's knuckle
fp_post_back  = 0.0    # mm — the post runs right back to the wall's inner
                        #      face (user, 2026-09-21), and forward by the same
                        #      amount on the other side of the pin, so it is
                        #      symmetric about its own pivot
fp_deck_dx    = 0.0    # mm — |X| of those screws: centred
                        #      to the deck's own block. They run along Z, in
                        #      from OUTSIDE the cube, and stop short of the
                        #      pin (user, 2026-09-20).
fp_deck_len   = 12.0   # mm — and they are this long: enough to reach through
                        #      deck and block and bite the post, short enough
                        #      never to arrive at the pin's bore.
fp_screw_dz   = 4.0    # mm — its screws spread in Z now, not in X: the −X
                        #      half of this wall is the actuation's column,
                        #      and a symmetric foot ran straight into the
                        #      servo's cradle and the spring cage. 4 and not
                        #      more because the lower post's own boss, Ø11 on
                        #      the wall, would otherwise reach through the
                        #      floor.
fp_screw_x    = 16.0   # mm — X of its two wall screws. Was 17, and before
                        #      that 14.5 (out past
                        #      the LINKS' own arms, which reach 11.5) until the
                        #      pin itself turned out to reach further, to 12.5,
                        #      and the screw's own head — 5.5 wide — needs to
                        #      clear THAT with margin, not just the arms.
fw_screw_z    = 0.0    # mm — |Z| the wall screws sit off the pivot's own
                        #      height. Wanted to be 8, spreading them to brace
                        #      against tipping the way the old (removed) idler
                        #      bracket did on this same wall — but the bottom
                        #      post shares this stretch of wall with the
                        #      HORN (push_z = -30, R_push = 40), which sweeps
                        #      to within 2.1 mm of the pivot's own height under
                        #      full tilt and closes to under 1 mm by 4 mm of
                        #      spread. Bracing against tipping loses to a
                        #      swept part actually being there: two screws,
                        #      X-spread only, same count and pattern the deck
                        #      version used.
deck_t        = 12.0   # mm — floor / ceiling plate thickness. Was 4: the
                        #      motor shaft's axial load lands in the middle of
                        #      these plates, 36 N per engaged axis (144 with all
                        #      four on one cone), and 4 mm of PLA sank 0.1-0.7
                        #      mm under it — the rubber's whole squeeze. Grown
                        #      OUTWARD (the inside is full); printed with 4-5
                        #      solid layers each face and ~15-20% gyroid it IS a
                        #      box plate, ~10-15x stiffer (user, 2026-09-21).
deck_boss_h   = 5.0    # mm — how far their screw bosses stand proud, inward
deck_pocket_min = 1.0  # mm — relief in the CEILING's inner face over the crank's
                        #      path: at full stroke the crank passed 0.21 mm under
                        #      it (the chain's sweep never looked at the decks),
                        #      and H's E-clip stands 0.5 over the crank's boss.
                        #      The depth is DERIVED from how high the crank
                        #      actually reaches; this is only its floor.

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
bracket_weld  = 1.0    # mm — how far a bracket printed as part of its host
                        #      runs INTO it, so the fuse is a solid joint and
                        #      not two solids sharing a face
wall_thick     = 12.0  # mm — cube wall thickness: the same 12 as the decks,
                        #      because it has to be a CUBE (user, 2026-09-21),
                        #      and a box plate like them
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
tip_fill_y     = 18.0   # mm — the cone is SOLID from its tip to here, hollow
                        #      beyond. No shaft to grip any more; this only has
                        #      to stay clear of the joint's tail, which the
                        #      all-pairs check measures.
cone_wall     = 2.5    # mm — wall of the output cone, which is a SHELL. This is
                        #      what the whole outboard layout turns on: the cone
                        #      was a solid lump of plastic sitting between the
                        #      apex and the gears, and hollowing it lets the
                        #      bearing housing live INSIDE it. The entire gear
                        #      stack then sits behind the push point instead of
                        #      in front of it. A conical shell gives up almost
                        #      nothing in torsion against a solid cone, and it
                        #      takes a lot of mass off the part that tilts.
shaft_d        = 5.0   # mm — output shaft diameter
# The central column (user, 2026-09-21): "the motor cones must not move".
motor_shaft_d  = 8.0   # mm — ground Ø8 rod. Ø5 bent ~0.08 mm under one axis'
                        #      radial load between the decks, against a ~0.2
                        #      mm squeeze; Ø8 bends ~0.01
mbrg           = (8.0, 22.0, 7.0)   # mm — 608ZZ, one in each deck
mflange        = (22.0, 4.0)        # mm — rigid flange coupling for Ø8: flange
mflange_hub    = (12.0, 13.0)       #      Ø x thick, hub Ø x long — PROVISIONAL,
mflange_pcd    = 17.0               #      measure the one bought; 4x M3 on PCD
mcol_wash_t    = 1.0   # mm — washer 8x12 between each collar and its bearing's
                        #      inner race (a collar Ø14 would reach the shield)
mbrg_lip       = 1.0   # mm — the lip each 608's outer race pushes on, Ø18 bore
# Cube to cube (user, 2026-09-21): FIVE identical male outputs (top and the
# four walls) and one female input (bottom), so a cube stacks on any output.
# The joint is a jaw (spider) coupling — halves bought with a 5 or an 8 mm
# bore, the same outside — located by a square spigot (it centres and it
# carries the reaction torque, in 90 deg steps) and held by magnets at the
# corners. A hard knock pulls the cubes apart, which is wanted.
jc_d           = 20.0  # mm ┐ jaw coupling D20 L25: half = hub + claws,
jc_hub         = 7.5   # mm │ assembled length jc_len — PROVISIONAL, measure
jc_claw        = 9.0   # mm │
jc_len         = 25.0  # mm ┘
jc_clr         = 0.6   # mm — round each half, radially and at its base
sp_side        = 34.0  # mm ┐ the square spigot on each output face
sp_h           = 3.0   # mm ┘
sp_clr         = 0.2   # mm — the input face's socket over the spigot
mag_d, mag_t   = 10.0, 3.0   # mm — N52 disc magnets, four per face
mag_at         = 40.0  # mm — at (±40, ±40) from the face's centre

# ── Carriage bearings — MR105ZZ, the project's single standard bearing ───────
brg_id        = 5.0    # mm ┐
brg_od        = 10.0   # mm │ MR105ZZ (5x10x4)
brg_w         = 4.0    # mm ┘
brg_seat_lip  = 1.5    # mm — lip at the cone end of the seat bore. Both bearings
                        #      go in from the pinion end, and this is what the
                        #      far one stops against.
brg_fit_press = 0.15   # mm — radial add for a press fit (→ Ø10.3), FDM
                        #      calibrated on the Creality Hi, see build-log
out_brg_len   = brg_w + brg_seat_lip   # DERIVED — the output shaft's OUTER
                        # bearing seat, total length: the bearing's own width
                        # plus the shoulder that stops it, mostly inside the
                        # wall's own 4 mm.
                        # The shaft now reaches ~20 mm in from the wall to the
                        # solid cross, so ONE bearing is not enough: a second
                        # MR105ZZ sits in a boss on the wall's INNER face,
                        # pressed in from inside against the wall's own hole.

# ── FDM print calibration (Creality Hi / PLA) — see docs/build-log.md ────────
# This printer runs small holes ~0.5 mm UNDERSIZE, so every hole that receives
# real hardware is modelled oversize by a measured amount, not at nominal.
fdm_shaft_hole_d = 5.8  # mm — modelled Ø for a Ø5 rod (measured, coupon v2)
fdm_mshaft_hole_d = 8.8 # mm — same allowance on the Ø8 motor shaft: a slip fit,
                        #      so the cones slide on it until their flanges clamp
fdm_pin_hole_d   = 4.6  # mm — modelled Ø for a Ø4 pin (extrapolated — verify)
fdm_bolt_hole_d  = 3.8  # mm — modelled Ø for an M3 clamp bolt to pass through
fdm_dowel_hole_d = 5.5  # mm — modelled Ø for the Ø5 trunnion dowel, INTERFERENCE
                         #      (5.8 is the measured slip fit, so this is 0.3
                         #      tighter). Extrapolated, not measured — verify on
                         #      a coupon before relying on it.
fdm_pin_press_d  = 4.3  # mm — Ø4 pin PRESSED (0.3 under the slip fit, same
                         #      rule as the dowel). Extrapolated — verify.
fdm_act_hole_d   = act_pin_d + 0.6   # mm — Ø3 pin RUNNING. Was act_pin_d + 0.2,
                         #      which this printer makes ~2.7: the pin would not
                         #      have gone in. +0.6 is the Ø4's measured-ish margin.
fdm_act_press_d  = fdm_act_hole_d - 0.3   # mm — Ø3 pin PRESSED. Verify.

# ── Pin retention: nothing clamps a joint ───────────────────────────────────
# Every pin is PRESSED into one part and RUNS in the other, so it turns in one
# bore only. Four-bar: pressed into the MIDDLE part (lug at A, link at B); the
# part straddling it can't come off because it straddles, so no clip at all.
# Actuation chain: lap joints, two plates, nothing straddles — the running
# plate is held by an E-clip (DIN 6799 RS 2.3) on its free face, clip_gap off
# it, so the clip never squeezes the joint. Sides chosen by a sweep: each is
# the side with room for the clip all stroke long.
clip_od    = 6.0      # mm — DIN 6799 RS 2.3 (shaft 3-4, groove Ø2.3)
clip_t     = 0.6      # mm
clip_gap   = 0.1      # mm — axial free play left under the clip
clip_tail  = 0.3      # mm — pin past the clip (the groove's far shoulder)

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
ratio_gear  = 1.0                     # a double cardan is 1:1
ratio_total = ratio_fric * ratio_gear

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


# ── Actuation chain, rest geometry (Y, Z) ────────────────────────────────────
# Servo shaft S; crank tip H (crank level, pointing +Y at rest, so H moves in
# Z); ONE link from H to the ear E on the carriage. Two pins, and the ratio is
# the crank radius alone — see horn_r.
E0 = (ear_y, ear_z)
def act_chain(T):
    """Where the actuation is at this tilt.

    A lead screw, not a linkage: the nut travels straight up the screw and
    carries the ear with it. So there is nothing to solve — the ear is where
    the four-bar puts it, and the nut is at the ear's own height. The servo
    angle follows from the lead.

    The ear swings on an arc while the nut goes straight, but over the whole
    stroke that arc departs from the line by 0.04 mm, which the pin's own
    slot takes. That is the whole reason this works where a linkage needed a
    lever to change planes."""
    E = T(E0)
    return {"E": E, "nut_z": E[1],
            "servo_deg": (E[1] - E0[1]) / screw_lead * 360.0}


def horn_swing(phi_t):
    """How far the servo has turned, in degrees, at this tilt."""
    T, _ = carriage_transform(phi_t)
    return abs(act_chain(T)["servo_deg"])


def uj_geom(phi_t):
    """The folded cardan at tilt phi_t, from where the four-bar actually puts
    the ring (drift included): intermediate length, the intermediate's own
    angle from +Y, and the ring centre (y, z). The solid cross does not move."""
    T, _ = carriage_transform(phi_t)
    R = T((uj_ring_y, 0.0))
    dy, dz = R[0] - uj_cross_y, R[1]
    return math.hypot(dy, dz), math.atan2(dz, dy), R


_POSE_CACHE = {}   # pose_state's, declared here: the fixed point below has
                    # to clear it between passes, and it runs first.


# ── Cardan, cube and chain: a small fixed point ─────────────────────────────
# These three need one another. The servo lies against the WALL, so where its
# shaft sits depends on cube_half; cube_half is whatever reaches furthest,
# today the cardan's intermediate, whose reach depends on the stroke; and the
# stroke depends on the chain the servo drives. Two passes settle it to
# microns. The loop asserts that it HAS settled rather than trusting it would,
# and clears the pose cache each pass — a cached pose carries the chain's own
# geometry with it, and that is exactly what moves between passes.
phi_preload = phi_c * 1.2          # bootstrap, replaced on the first pass
for _pass in range(8):
    _POSE_CACHE.clear()
    _uj_sweep = [(phi_preload * k / 8.0, uj_geom(phi_preload * k / 8.0))
                 for k in range(-8, 9)]
    uj_L_rest    = uj_ring_y - uj_cross_y
    uj_L_range   = (min(g[0] for _, g in _uj_sweep), max(g[0] for _, g in _uj_sweep))
    uj_bend_cross = max(abs(g[1]) for _, g in _uj_sweep)             # rad
    uj_bend_ring  = max(abs(g[1] - p) for p, g in _uj_sweep)         # rad
    # Furthest the intermediate's front corner reaches toward the wall.
    uj_front_y = max(uj_cross_y + (g[0] + uj_head_end) * math.cos(g[1])
                     + uj_head_ro * abs(math.sin(g[1])) for _, g in _uj_sweep)

    # ── Cube ──────────────────────────────────────────────────────────────────────
    # The cube is as small as the FURTHEST thing that must fit, in Y (to the wall)
    # or in Z (to the ceiling) — it is a cube, so both are the same number.
    cube_half_by = {
        "cardan's intermediate + inner output bearing boss":
            uj_front_y + run_clr + brg_w,
        "four-bar frame post": fb_A[0] + fp_post_y + 1.0,
        "servo case standing on the wall (its own Z)": abs(sv_z0) + run_clr,
        "actuation ear boss (to the wall)":
            E0[0] + act_boss_r + run_clr,
    }
    cube_half_driver = max(cube_half_by, key=cube_half_by.get)
    cube_half = cube_half_by[cube_half_driver]
    cube_out  = cube_half + wall_thick
    # Brackets seat on the bosses, not on the wall itself.
    wall_face_y = cube_half - wall_boss_h

    # The sweep to contact is DERIVED now, not chosen: with the lever gone, the
    # crank radius and where the ear sits are what set it. (It used to be the
    # input, and the crank radius came out of it through the lever's ratio.)
    sv_theta_c = horn_swing(phi_c)
    if sv_theta_c is None:
        raise RuntimeError("the actuation chain cannot reach the contact stop")


    # ── Stop ladder. No mesh step: the gears never disengage (invariant 4) ───────
    # Before contact the carriage follows the servo at sv_theta_c per phi_c. After
    # it, of every further degree of servo sweep 1 part still reaches the carriage
    # (squeezing the rubber) and spring_ratio parts wind the spring.
    spring_split = 1.0 + spring_ratio
    _phi_new = phi_c + (math.radians(sv_theta_max - sv_theta_c)
                           * (phi_c / math.radians(sv_theta_c)) / spring_split)
    _settled = abs(_phi_new - phi_preload) < 1e-9
    phi_preload = _phi_new
    if _settled:
        break
else:
    raise RuntimeError("cardan, cube and chain did not settle")
_POSE_CACHE.clear()
STOPS = {"free": 0.0, "contact": phi_c, "preload": phi_preload}
phi = CARRIAGE_DIR * STOPS[CARRIAGE_STOP] if CARRIAGE_STOP != "free" else 0.0

# The spring's working stroke: what is left of the servo's turn once the free
# gap is closed, in millimetres at the nut. Everything about the spring — rate,
# size, which family of part it can be — follows from this one number.
spring_stroke = max(0.05, (sv_theta_max - sv_theta_c) / 360.0 * screw_lead)
def screw_force():
    """Force at the nut for the servo's working torque: the screw's own
    equation, plus the loaded thrust washer's friction at its mean radius."""
    r_m = (screw_d / 2.0 + tw_od / 2.0) / 2.0 / 1000.0
    per_newton = (screw_lead / 1000.0) / (2.0 * math.pi * screw_eff) + tw_mu * r_m
    return sv_use * sv_stall / per_newton


spring_rate = screw_force() / spring_stroke     # N/mm


drift_contact = apex_drift(phi_c)
drift_preload = apex_drift(phi_preload)
slip_fourbar  = abs(drift_preload[1]) / L_line   # same measure as v5's micro-slip

# X stack, from the servo's face outward: spring, then the LINK, then the
# crank outermost; the ear stacked back inboard of the link, under the
# spring's own band but far out in Y, where the case and the spring are not.
#
# The link has to be outboard of the spring, not inboard of the crank: with
# the crank perpendicular to the link (which is where the transmission is),
# the link's pin sits one crank radius from the spline, and the spring on
# that spline is wider than that — the JOINT itself lands inside the spring,
# so no shape of link gets round it. Stepping the band out costs the ear
# 2.5 mm in X; growing the crank instead would have cost force at the ear in
# the same proportion, which is the one thing the lever was there to buy.
# The shaft points −X, so every band is measured the other way: spring
# first, then the link, then the crank furthest into the corner, with the ear
# stacked back toward the case — in the spring's own band, but nowhere near it
# in Z, because the ear rides over the case while the spring is on the shaft.
# No X stack any more: the screw's nut pushes the ear straight, so the only
# band left is the carriage arm's own, which the ear is part of.
# The ear is bored into the block's own face, ear_pin_in deep.
ear_pin_in = 8.0
ear_pin_out = 5.0   # mm — and this far into the carrier's slot
x_ear = ((-carr_half, -carr_half + ear_pin_in) if ear_sx < 0
         else (carr_half - ear_pin_in, carr_half))


def _rot_about(p, c, a):
    ca, sa = math.cos(a), math.sin(a)
    dy, dz = p[0] - c[0], p[1] - c[1]
    return (c[0] + ca * dy - sa * dz, c[1] + sa * dy + ca * dz)


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
    key = round(phi_t, 12)
    st = _POSE_CACHE.get(key)
    if st is not None:
        return st
    T, pose = carriage_transform(phi_t)
    st = {"phi": phi_t, "T": T, "B1": pose[0], "B2": pose[1]}
    st.update(act_chain(T))
    # Cached: every sweep asks for the same handful of poses, and each one
    # costs a bisection through the four-bar. Read-only, like cached().
    _POSE_CACHE[key] = st
    return st


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
    return foot_t


def _screw_seat_y(sx):
    """Y of the face a wall screw's head bears on: a 5 mm foot against the
    bosses. The idler's thickened D was the one exception, and it is gone."""
    return wall_face_y - foot_t


def wall_screws():
    """(x, z) of every screw through this axis's wall: the four-bar's two
    frame pivots, X-spread only (see fw_screw_z for why not four).

    The list stays the single source for both the wall's bosses and the
    brackets' own holes, so the two cannot disagree."""
    return []


_SHAPE_CACHE = {}


def cached(fn, *args):
    """Build a shape once and hand the same one out afterwards.

    Every sweep rebuilds the same solids at every pose — the carriage nine
    times over, the frame brackets (whose swept-envelope cut is expensive)
    four times per run. Nothing may MUTATE what comes back: Shape.rotate and
    .translate work in place, so callers copy first (place, place_carriage,
    uj_place all do)."""
    key = (fn.__name__,) + args
    if key not in _SHAPE_CACHE:
        _SHAPE_CACHE[key] = fn(*args)
    return _SHAPE_CACHE[key]


def bb_gap(a, b):
    """Lower bound on the distance between two shapes, from their bounding
    boxes alone. Cheap, and a true lower bound, so a sweep looking for the
    SMALLEST distance can skip any pair whose boxes are already further apart
    than the best it has found — same answer, far fewer boolean solves."""
    A, B = a.BoundBox, b.BoundBox
    dx = max(0.0, A.XMin - B.XMax, B.XMin - A.XMax)
    dy = max(0.0, A.YMin - B.YMax, B.YMin - A.YMax)
    dz = max(0.0, A.ZMin - B.ZMax, B.ZMin - A.ZMax)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


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
    cone = cone.cut(cyl(fdm_mshaft_hole_d / 2.0, bore_h,
                        v(0, 0, apex.z + sd * (s_motor_lo * math.cos(alpha) - 2)),
                        axis))
    # the flange coupling's hub goes INTO the cone's base, so the space under
    # the base is left to the bearing; its four M3 self-tap into the base
    cone = cone.cut(cyl(mflange_hub[0] / 2.0 + 0.3, mflange_hub[1] + 1.5,
                        v(0, 0, sd * (mot_base_z + 1.0)), v(0, 0, -sd)))
    for k in range(4):
        a = math.radians(45.0 + 90.0 * k)
        cone = cone.cut(cyl(foot_tap_d / 2.0, 9.0,
                            v(mflange_pcd / 2.0 * math.cos(a),
                              mflange_pcd / 2.0 * math.sin(a),
                              sd * (mot_base_z + 1.0)), v(0, 0, -sd)))
    return cone


def _ring_z(ri, ro, z0, h):
    """A ring on the motor axis from z0 over h (h may be negative)."""
    za = min(z0, z0 + h)
    return cyl(ro, abs(h), v(0, 0, za)).cut(cyl(ri, abs(h) + 2, v(0, 0, za - 1)))


def make_motor_shaft():
    """The motor shaft: Ø8, from inside the input's female coupling half to
    inside the top output's male half."""
    z0 = _col_z(-1)["hub"][1]          # the female half's hub, outer end
    z1 = _col_z(1)["hub"][1]           # the male half's hub, outer end
    return cyl(motor_shaft_d / 2.0, z1 - z0, v(0, 0, z0))


def _jc_sink():
    """How deep each male half sits in its own face: as deep as the walls
    allow, 0.7 mm clear of the output shaft's outer bearing."""
    return wall_thick - (brg_seat_lip + brg_w + 0.7)


def _col_z(sd):
    """Z of the column's parts on one side (sd = +1 top, -1 bottom), as a
    dict, all measured from the cube's outer face inward. Top: the male half
    in the face, the 608 under it on a lip. Bottom: the female half and its
    spider in a socket, the 608 over it on a lip, standing in a boss."""
    zf = cube_half + deck_t
    if sd > 0:
        half0 = zf - _jc_sink()                  # male half's base
        hub = (half0, half0 + jc_hub)
        claws = (half0 + jc_hub, half0 + jc_hub + jc_claw)
        socket = half0 - jc_clr                  # counterbore's floor
    else:
        inner = zf - (jc_len - _jc_sink())       # female half's inner end
        hub = (inner, inner + jc_hub)
        claws = (inner + jc_hub, inner + jc_hub + jc_claw)
        socket = inner - jc_clr
    lip = (socket - mbrg_lip, socket)
    brg = (lip[0] - mbrg[2], lip[0])
    wash = (brg[0] - mcol_wash_t, brg[0])
    col = (wash[0] - col_h, wash[0])
    # every entry as (inner end, outer end), then mirrored for the bottom
    d = dict(face=zf, hub=hub, claws=claws, socket=socket, lip=lip,
             brg=brg, wash=wash, col=col)
    if sd < 0:
        d = {k: (tuple(-x for x in val) if isinstance(val, tuple) else -val)
             for k, val in d.items()}
    return d


def _zspan(a, b, r_in, r_out):
    z0, z1 = min(a, b), max(a, b)
    ring = cyl(r_out, z1 - z0, v(0, 0, z0))
    if r_in > 0:
        ring = ring.cut(cyl(r_in, z1 - z0 + 2, v(0, 0, z0 - 1)))
    return ring


def make_motor_bearing(sd):
    """608ZZ, on its lip, just inside the coupling's socket."""
    z = _col_z(sd)["brg"]
    return _zspan(z[0], z[1], mbrg[0] / 2.0, mbrg[1] / 2.0)


def make_motor_washer(sd):
    """8x12x1 washer on the bearing's inner race, on the cone side."""
    z = _col_z(sd)["wash"]
    return _zspan(z[0], z[1], motor_shaft_d / 2.0, 6.0)


def make_motor_collar(sd):
    """Lock collar on the shaft against that washer: the pair of them hold
    the shaft in Z, each deck taking the thrust toward itself."""
    z = _col_z(sd)["col"]
    return _zspan(z[0], z[1], motor_shaft_d / 2.0, col_d / 2.0)


def make_motor_spacer():
    """Bottom only, PRINTED: a sleeve from the lower cone's flange down to the
    washer on the bottom bearing. There is no room for a collar there — the
    flange's screw heads take it — so the flange itself, through this, is the
    down stop. Its length sets the cones' height: print it to length, or shim
    it, to put the rubber apex on the carriage's."""
    z0 = -(mot_base_z + mflange[1])
    z1 = _col_z(-1)["wash"][0]
    return _zspan(z0, z1, motor_shaft_d / 2.0 + 0.2, 5.8)


def make_motor_flange(sd):
    """Rigid flange coupling on the cone's base: flange against the base,
    hub INSIDE the cone, grubs on the shaft."""
    z0 = sd * mot_base_z
    body = _zspan(z0, z0 + sd * mflange[1], motor_shaft_d / 2.0, mflange[0] / 2.0)
    body = body.fuse(_zspan(z0, z0 - sd * mflange_hub[1],
                            motor_shaft_d / 2.0, mflange_hub[0] / 2.0))
    for k in range(4):
        a = math.radians(45.0 + 90.0 * k)
        body = body.cut(cyl(1.7, mflange[1] + 2.0,
                            v(mflange_pcd / 2.0 * math.cos(a),
                              mflange_pcd / 2.0 * math.sin(a),
                              z0 - sd * 1.0), v(0, 0, sd)))
    return body


def _jaw_half(hub, claws, bore_d, axis_pt, axis):
    """A jaw-coupling half as its envelope: the hub with its bore, and the
    claws (with the spider, on the female) as a solid drum."""
    def span(a, b):
        lo, hi = min(a, b), max(a, b)
        return lo, hi - lo
    lo, ln = span(*hub)
    body = cyl(jc_d / 2.0, ln, axis_pt(lo), axis)
    body = body.cut(cyl(bore_d / 2.0, ln + 2, axis_pt(lo - 1), axis))
    lo, ln = span(*claws)
    return body.fuse(cyl(jc_d / 2.0, ln, axis_pt(lo), axis))


def make_jaw_top():
    """The top output's male half, on the motor shaft."""
    z = _col_z(1)
    return _jaw_half(z["hub"], z["claws"], motor_shaft_d,
                     lambda t: v(0, 0, t), Z_AXIS)


def make_jaw_bottom():
    """The input's female half, with its spider, on the motor shaft."""
    z = _col_z(-1)
    return _jaw_half(z["hub"], z["claws"], motor_shaft_d,
                     lambda t: v(0, 0, t), Z_AXIS)


def make_jaw_wall():
    """A wall output's male half, on the output shaft (per axis)."""
    y0 = cube_half + wall_thick - _jc_sink()
    return _jaw_half((y0, y0 + jc_hub), (y0 + jc_hub, y0 + jc_hub + jc_claw),
                     shaft_d, lambda t: v(0, t, 0), Y_AXIS)


def _face_magnet_pts():
    return [(sx * mag_at, sz * mag_at) for sx in (1, -1) for sz in (1, -1)]


def make_wall_magnet(k):
    """One of a wall's four magnets, flush with its outer face."""
    x, z = _face_magnet_pts()[k]
    return cyl(mag_d / 2.0, mag_t, v(x, cube_half + wall_thick - mag_t, z), Y_AXIS)


def make_deck_magnet(zs, k):
    """One of a deck's four magnets, flush with its outer face."""
    x, y = _face_magnet_pts()[k]
    zf = zs * (cube_half + deck_t)
    return cyl(mag_d / 2.0, mag_t, v(x, y, zf - zs * mag_t if zs > 0 else zf))


def _deck_column_features(deck, zs):
    """The deck's middle: the 608 on its lip, the coupling's socket, the
    spigot (top) or its socket (bottom), and the four magnet pockets. All
    cut LAST."""
    z = _col_z(zs)
    zf = z["face"]
    seat_r = mbrg[1] / 2.0 + brg_fit_press
    b_in, b_out = z["brg"]
    # A boss inside the cube wherever the bearing stands proud of the deck.
    if abs(b_in) < cube_half:
        deck = deck.fuse(_zspan(b_in, zs * (cube_half + 1.0), 0.0, seat_r + 3.0))
    if zs > 0:
        deck = deck.fuse(Part.makeBox(sp_side, sp_side, sp_h,
                                      v(-sp_side / 2.0, -sp_side / 2.0, zf)))
    else:
        w = sp_side + 2 * sp_clr
        deck = deck.cut(Part.makeBox(w, w, sp_h + 0.2,
                                     v(-w / 2.0, -w / 2.0, zf - 0.01)))
    deck = deck.cut(_zspan(b_in - zs * 0.01, b_out, 0.0, seat_r))
    deck = deck.cut(_zspan(z["lip"][0], z["lip"][1], 0.0, 9.0))
    deck = deck.cut(_zspan(z["socket"], zf + zs * (sp_h + 1.0), 0.0,
                           jc_d / 2.0 + jc_clr))
    for x, y in _face_magnet_pts():
        deck = deck.cut(cyl(mag_d / 2.0 + 0.1, mag_t + 0.2 + 1.0,
                            v(x, y, zf - zs * (mag_t + 0.2) if zs > 0 else zf - 1.0)))
    return deck


def make_motor_flange_screw(sd, k):
    """One of the flange's M3 x 10, head on the flange, into the cone."""
    a = math.radians(45.0 + 90.0 * k)
    base = v(mflange_pcd / 2.0 * math.cos(a), mflange_pcd / 2.0 * math.sin(a),
             sd * (mot_base_z + mflange[1]))
    return make_screw(base, v(0, 0, -sd), 3.0, 10.0, 5.0, 2.0)


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
    y = apex_off_output, flaring toward the wall, and now ALSO the cardan's
    outer yoke: a neck past its rim carries the 6805 on the outside and the
    ring cross's pins on the inside.

    SOLID from the tip to tip_fill_y. Beyond it the cone is a shell whose
    cavity stops cone_wall short of the rim, so the rim is a closed plate
    joining shell to neck, and a Ø(2*neck_ri) bore runs from where the shell's
    cavity is that wide right out through the neck."""
    cone = cone_frustum(v(0, out_apex_y, 0), Y_AXIS, beta,
                        s_output_lo, s_out_hi)
    # Neck: a shoulder the bearing's inner race stops on, then the seat.
    cone = cone.fuse(cyl(neck_ro + neck_shoulder, nb_y0 - out_base_y + 1.0,
                         v(0, out_base_y - 1.0, 0), Y_AXIS))
    cone = cone.fuse(cyl(neck_ro, neck_y1 - nb_y0 + 0.5,
                         v(0, nb_y0 - 0.5, 0), Y_AXIS))
    cav_apex_y = out_apex_y + cone_wall / math.sin(beta)
    cavity = cone_frustum(v(0, cav_apex_y, 0), Y_AXIS, beta, 0.0, s_out_hi + 5.0)
    cavity = cavity.common(Part.makeBox(200.0, out_base_y - cone_wall - tip_fill_y,
                                        200.0, v(-100.0, tip_fill_y, -100.0)))
    bore_y0 = cav_apex_y + neck_ri / math.tan(beta)
    cavity = cavity.fuse(cyl(neck_ri, neck_y1 - bore_y0 + 1.0,
                             v(0, bore_y0, 0), Y_AXIS))
    # Wider round the tube's head.
    y_head = uj_ring_y - uj_head_back - 1.0
    cavity = cavity.fuse(cyl(neck_ri_head, neck_y1 - y_head + 1.0,
                             v(0, y_head, 0), Y_AXIS))
    cone = cone.cut(cavity)
    # The ring cross's X pins: through the neck wall, under the bearing, which
    # is what keeps them in.
    for xs in (1, -1):
        x0 = xs * (neck_ri_head - 1.0) if xs > 0 else -(neck_ro + 1.0)
        cone = cone.cut(pin_x((uj_ring_y, 0.0), uj_pin_d + 0.1, x0,
                              neck_ro - neck_ri_head + 2.0))
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


def make_uj_mid(L):
    """Cardan intermediate, built on +Y from the solid cross (s = 0 at
    uj_cross_y) at length L; uj_place() tilts it. A stepped tube: the solid
    cross's Z pins go into its narrow tail, the ring's Z pins into slotted holes
    in its wide head (the length changes by uj_slide over the stroke), and the
    ring's X pins pass out through two windows in the head to the cone's neck."""
    y0 = uj_cross_y - uj_mid_end
    yr = uj_cross_y + L
    yh = yr - uj_head_back
    tail = cyl(uj_mid_ro, yh - y0 + 0.5, v(0, y0, 0), Y_AXIS)
    head = cyl(uj_head_ro, uj_head_back + uj_head_end, v(0, yh, 0), Y_AXIS)
    tube = tail.fuse(head)
    tube = tube.cut(cyl(uj_mid_ri, yh - y0 + 2, v(0, y0 - 1, 0), Y_AXIS))
    tube = tube.cut(cyl(uj_head_ri, uj_head_back + uj_head_end + 1,
                        v(0, yh, 0), Y_AXIS))
    hole = uj_pin_d / 2.0 + 0.05
    span = 2 * uj_head_ro + 2
    tube = tube.cut(cyl(hole, span, v(0, uj_cross_y, -span / 2.0), Z_AXIS))
    for dy in (-uj_slide / 2.0, uj_slide / 2.0):
        tube = tube.cut(cyl(hole, span, v(0, yr + dy, -span / 2.0), Z_AXIS))
    tube = tube.cut(Part.makeBox(span, uj_slide, 2 * hole,
                                 v(-span / 2.0, yr - uj_slide / 2.0, -hole)))
    # Window: the X pin rocks about the Z pins by the ring's own bend, so it
    # walks along the tube by r*sin(bend) each way, plus the slide.
    wy = hole + uj_head_ro * math.sin(uj_bend_ring) + uj_slide / 2.0 + 0.3
    wz = hole + 0.3
    tube = tube.cut(Part.makeBox(span, 2 * wy, 2 * wz,
                                 v(-span / 2.0, yr - wy, -wz)))
    return tube


def make_uj_ring(L):
    """Ring cross at the head of the intermediate: the output shaft passes
    through its bore. Z pins into the intermediate, X pins out through its
    windows into the cone's neck. Pins modelled as part of it."""
    yr = uj_cross_y + L
    ring = cyl(uj_ring_ro, uj_ring_t, v(0, yr - uj_ring_t / 2.0, 0), Y_AXIS).cut(
        cyl(uj_ring_ri, uj_ring_t + 2, v(0, yr - uj_ring_t / 2.0 - 1, 0), Y_AXIS))
    r0 = uj_ring_ro - 0.5
    z_len = uj_head_ro - 0.2 - r0
    x_len = (neck_ri_head + neck_ro) / 2.0 - r0
    for s in (1, -1):
        ring = ring.fuse(cyl(uj_pin_d / 2.0, z_len,
                             v(0, yr, s * r0), v(0, 0, s)))
        ring = ring.fuse(cyl(uj_pin_d / 2.0, x_len,
                             v(s * r0, yr, 0), v(s, 0, 0)))
    return ring


def make_uj_cross(pins=True):
    """Solid cross on the output shaft's fork: a prism, narrow in X and long in
    Z. One through pin in X to the fork's arms; two short pins from its long
    ends out to the intermediate's tail, each in a blind hole that stops short
    of the through pin."""
    a, hy, hz = uj_cross_a, uj_cross_hy, uj_cross_hz
    body = Part.makeBox(2 * a, 2 * hy, 2 * hz, v(-a, uj_cross_y - hy, -hz))
    if not pins:
        return body
    r = uj_pin_d / 2.0
    x_len = uj_fork_ro - 0.2
    body = body.fuse(cyl(r, 2 * x_len, v(-x_len, uj_cross_y, 0), X_AXIS))
    z_in = r + 0.5                      # blind: stops clear of the through pin
    for sgn in (1, -1):
        body = body.fuse(cyl(r, uj_mid_ro - 0.2 - z_in,
                             v(0, uj_cross_y, sgn * z_in), v(0, 0, sgn)))
    return body


def make_uj_fork():
    """Output-shaft fork: two arms straddling the solid cross in X, a base
    keyed on the output shaft by the D."""
    a = uj_cross_a
    ya = uj_cross_y - (a + 1.0)
    yb = uj_cross_y + uj_fork_back
    h = a + 0.5
    fork = cyl(uj_fork_ro, uj_fork_base, v(0, yb, 0), Y_AXIS)
    for s in (1, -1):
        x0 = uj_fork_ri if s > 0 else -uj_fork_ro
        fork = fork.fuse(Part.makeBox(uj_fork_ro - uj_fork_ri, yb - ya + 0.5,
                                      2 * h, v(x0, ya, -h)))
    fork = fork.cut(pin_x((uj_cross_y, 0.0), uj_pin_d + 0.1,
                          -uj_fork_ro - 1, 2 * uj_fork_ro + 2))
    return fork.cut(d_bore(yb - 1.0, uj_fork_base + 2.0))


def uj_place(shape, st):
    """Tilt a cardan part (built on +Y from the solid cross) to the
    intermediate's angle at this pose."""
    sh = shape.copy()
    ang = uj_geom(st["phi"])[1]
    if abs(ang) > 1e-12:
        sh.rotate(v(0, uj_cross_y, 0), X_AXIS, math.degrees(ang))
    return sh


def make_output_shaft():
    """Ø5 output shaft: from the cardan's fork out through the wall's two
    bearings (one in an inner boss, one in the wall), a little past."""
    y0 = uj_cross_y + uj_fork_back
    y1 = cube_half + wall_thick - _jc_sink() + jc_hub   # through its male half
    shaft = cyl(shaft_d / 2.0, y1 - y0, v(0, y0, 0), Y_AXIS)
    return shaft.cut(Part.makeBox(shaft_d + 2, y1 - y0, shaft_d,
                                  v(-(shaft_d / 2.0 + 1), y0, shaft_flat_d / 2.0)))


def make_carriage():
    """The carriage, in one piece: a ring round the cone's neck holding the
    6805, two arms out to the four-bar, and the ear the actuation pulls on.
    Nothing of it is inside the cone any more — that is the joint's.

    Printed with its WALL-side face on the bed: the bearing seat comes out as a
    vertical round hole and the 45 deg jogs out to B rise from it unsupported.
    The ring runs on toward the wall past the bearing, to the ear block's own
    far face, so the two stand on the bed together — and that extra length IS
    the bearing's stop: a solid flange from the bearing down to the bed. The
    bearing goes in from the cone side, then carriage and bearing slide onto
    the neck together. The arms' uprights stay the bearing's width."""
    y_end = hous_y1
    # A BLOCK round the neck, not a ring: flat faces for the ear and for the
    # bed, and it only fits now that the actuation has moved off its corners.
    body = Part.makeBox(2 * carr_half, y_end - hous_y0, 2 * carr_half,
                        v(-carr_half, hous_y0, -carr_half))
    body = body.fuse(make_carriage_arms(1)).fuse(make_carriage_arms(-1))
    # Every hole LAST: a later fuse fills an earlier hole straight back in.
    seat_r = nb_od / 2.0 + brg_fit_press
    body = body.cut(cyl(hous_flange_ri, 60.0, v(0, 0.0, 0), Y_AXIS))
    body = body.cut(cyl(seat_r, nb_y0 + nb_w - (hous_y0 - 1.0),
                        v(0, hous_y0 - 1.0, 0), Y_AXIS))
    for zs in (1, -1):
        body = body.cut(pin_x((fb_B[0], zs * fb_B[1]), fdm_pin_hole_d,
                              -(carr_half + 1.0), 2 * (carr_half + 1.0)))
    # The ear is the block's own face: the pin is PRESSED into it here, and
    # runs in the carrier's slot.
    body = body.cut(pin_x(E0, fdm_act_press_d, x_ear[0] - 1.0,
                          x_ear[1] - x_ear[0] + 1.0))
    # One face per flat: the arms' faces coplanar with the block's leave seams.
    return body.removeSplitter()


def make_ear():
    """The ear the actuation pushes — a BOSS on the carriage's own arm.

    With a lead screw there is no linkage to reach the ear from somewhere
    else: the nut's pusher comes to the arm, so the ear is a thicker patch of
    arm round a pin hole, and the load goes straight into the arm."""
    x0 = ear_sx * side_x - side_t / 2.0
    return disc_yz((ear_y, ear_z), act_boss_r + 1.5, x0, side_t)


def make_carriage_arms(sd):
    """One upright of the carriage's 'H': a vertical bar at arm_root's Y, from
    the lower pivot's height to the upper one's, crossing the housing ring it
    roots on, with a short jog at each end out to its pivot."""
    x0 = sd * side_x - side_t / 2.0
    # The jog out to each pivot runs at 45 deg, not square: from B straight
    # down-and-out until it meets the upright.
    z_corner = fb_B[1] - (arm_root[0] - fb_B[0])
    # The jogs and the pin's boss run OUTWARD to the block's own face: the
    # pin had 4 mm of arm round it and stood a millimetre out of it, in air.
    # Flush with the block, the pin is held along its whole reach. The
    # upright goes out to the block's edge with them (user, 2026-09-21).
    x0_out = x0 if sd > 0 else -carr_half
    t_out = carr_half - (side_x - side_t / 2.0)
    # Square-ended, not round: a round end stood out over the block's top
    # beside the jog.
    part = box_yz(arm_root[0] - arm_up_w / 2.0, arm_root[0] + arm_up_w / 2.0,
                  -z_corner, z_corner, x0_out, t_out)
    for zs in (1, -1):
        b = (fb_B[0], zs * fb_B[1])
        # The jog's inner end runs 5 mm on INTO the block, so its round end
        # is buried there instead of standing out over the block's top.
        _k = 5.0 / math.sqrt(2.0)
        corner = (arm_root[0] + _k, zs * (z_corner - _k))
        part = part.fuse(bar_yz(corner, b, arm_w, x0_out, t_out))
        part = part.fuse(disc_yz(b, arm_w / 2.0 + 0.5, x0_out, t_out))
        # Thrust boss on the arm's INNER face, standing out to a shim's width
        # off the link's outer face (1.5 mm of X play before).
        x_in = link_x + link_t / 2.0 + shim_gap
        x_arm_in = side_x - side_t / 2.0
        part = part.fuse(disc_yz(b, arm_w / 2.0 + 0.5,
                                 sd * x_in if sd > 0 else -x_arm_in,
                                 x_arm_in - x_in))
    # Nothing past the wall-side face: it is the face on the bed.
    return part.cut(Part.makeBox(2 * carr_half + 20.0, 20.0, 2 * carr_half + 40.0,
                                 v(-carr_half - 10.0, hous_y1,
                                   -carr_half - 20.0)))

def make_carriage_bearing(y_face, sd):
    y_start = min(y_face, y_face + sd * brg_w)
    outer = cyl(brg_od / 2.0, brg_w, v(0, y_start, 0), Y_AXIS)
    return outer.cut(cyl(brg_id / 2.0 + 0.05, brg_w + 2,
                         v(0, y_start - 1, 0), Y_AXIS))


def make_neck_bearing():
    """The 6805 the cone runs on, as its envelope."""
    outer = cyl(nb_od / 2.0, nb_w, v(0, nb_y0, 0), Y_AXIS)
    return outer.cut(cyl(nb_id / 2.0 + 0.05, nb_w + 2, v(0, nb_y0 - 1, 0), Y_AXIS))


def make_link_knuckle(A, B):
    """The web that joins the link's two arms, as its own shape.

    Separate from make_link because the checks need it alone: the ARMS are
    allowed to run within half a millimetre of the frame post — that is the
    pivot's own sliding fit, and a hinge wants it tight. The knuckle has no
    such licence. It is the one piece of the link crossing x = 0, so it is the
    piece that meets the motor cone's rim, and it must simply clear."""
    # Right across both arms to their outer faces, so the knuckle and the arm
    # ends read as one cylinder instead of a smaller drum set between them.
    web_x = link_x + link_t / 2.0
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
    # Runs on the lug's pin at A; the pin at B is pressed in HERE (see clip_od).
    for q, d in ((A, fdm_pin_hole_d), (B, fdm_pin_press_d)):
        body = body.cut(pin_x(q, d, -reach, 2 * reach))
    return body


def _link_swept_envelope(A, Bkey):
    """UNION of the link's two arms across the whole tilt stroke, each grown
    by run_clr up front (built oversize, not offset after — makeOffsetShape
    chokes on a compound this shape). A is fixed, so this is what the link
    actually sweeps near its frame pivot — a disc centred on A undercounts
    it, because a point on the bar's STRAIGHT edge, even close to A, sits
    further than link_w/2 from A in a straight line (its own along-the-bar
    offset adds in quadrature). Real geometry, not a hand-fitted
    approximation — the same lesson as the link-vs-cone gap earlier: routing
    an edge around a diagonal member by eye undercounts it.

    Grown in X too: inward by shim_gap, so the shim reaches the lug (the ONE
    thrust face at A), outward by run_clr. At link_t exactly the foot's slot
    walls sat on both arm faces at zero clearance — rubbing, and a second,
    unshimmable X stop fighting the lug."""
    env = None
    w = link_w + 2.0 * run_clr
    t = link_t + shim_gap + run_clr
    for k in range(-4, 5):
        st = pose_state(phi_preload * k / 4.0)
        B = st[Bkey]
        for xs in (1, -1):
            x_in = link_x - link_t / 2.0 - shim_gap
            x0 = x_in if xs > 0 else -(x_in + t)
            arm = bar_yz(A, B, w, x0, t)
            env = arm if env is None else env.fuse(arm)
    return env


def frame_pad(zs):
    """The block the deck grows to meet a frame post, with the clearance
    holes and the sunk heads for the two screws that hold it.

    The user's idea, and a better one than a foot on the wall: the deck is
    right there — 4 mm above the upper post and below the lower one — the
    screws run along Z so they print true, they go in from OUTSIDE the cube
    where a screwdriver can reach them, and their heads sink flush into the
    deck's own face."""
    z_face = zs * (fb_A[1] + fp_post_y)
    z_deck = zs * cube_half
    y_back = cube_half - fp_post_back
    y_far = 2.0 * fb_A[0] - y_back
    # The post's own footprint, no wider: the extra width was only ever there
    # for screws spread in X, and they are spread in Y instead.
    pad = Part.makeBox(2 * fp_lug_x, y_back - y_far,
                       abs(z_deck - z_face) + bracket_weld,
                       v(-fp_lug_x, y_far,
                         min(z_face, z_deck + zs * bracket_weld)))
    return pad


def frame_pad_holes(zs):
    """What gets cut through the deck and its pad for those two screws: a
    clearance hole all the way, and a counterbore in the deck's OUTSIDE face
    so the head sits flush. Cut after the pads are fused — cut before, the
    deck plate fills them straight back in."""
    tool = None
    z_out = zs * (cube_half + deck_t)
    z_face = zs * (fb_A[1] + fp_post_y)
    for sy in fp_deck_ys:
        hole = cyl(foot_hole_d / 2.0, abs(z_out - z_face) + 2.0,
                   v(fp_deck_dx, sy, z_out + zs * 1.0), v(0, 0, -zs))
        head = cyl(foot_screw_d, deck_t - 2.0 + 1.0,
                   v(fp_deck_dx, sy, z_out + zs * 1.0), v(0, 0, -zs))
        t = hole.fuse(head)
        tool = t if tool is None else tool.fuse(t)
    return tool


def make_frame_screw(zs, sy):
    """One of those screws, drawn: its head sunk in the deck, its shank
    running down through deck and pad into the post, beside the pin."""
    z_out = zs * (cube_half + 2.0)      # head sunk to 2 mm off the inner face
    return make_screw(v(fp_deck_dx, sy, z_out), v(0, 0, -zs), foot_screw_d,
                      fp_deck_len, foot_screw_d * 1.8, 2.0)


def make_frame_bracket(zs):
    """The four-bar's frame pivot: a foot against the wall's boss face and a
    post reaching in to the pivot. PRINTED SEPARATELY and screwed on (user,
    2026-09-20).

    Its pin bore runs along X. Printed as part of the wall, with the wall flat
    on the bed, that bore comes out horizontal — the one direction where an
    FDM hole goes oval and rough. On its own the post can be laid with the
    bore vertical, and it comes out round. That is the same rule that sends
    the screw's and the rod's anchors to the ceiling, where THEIR bores are
    the vertical ones.

    Moved back off the deck, at the user's request, so every support for this
    axis lives on the one part that comes off the machine: pull the wall and
    the whole mechanism is loose. The link reaction at this pivot runs along
    the link, 50 deg from Y, so on the deck it was compression, straight down
    a post; here it is bending, across a short arm. That arm is short, though:
    wall_face_y - fb_A[0] is under 4 mm today, because the cube has grown
    since a wall mount was last tried (the old design put this reach at
    35 mm, when the corona's own clearance drove the cube, not the Oldham's).
    At today's span the reach does not need engineering to survive it. What
    it does need is to fit: the bottom post shares this wall with the horn's
    swept path (see fw_screw_z), which is the real reason this stayed at two
    screws, X-spread only, same as the deck version."""
    z_pin = zs * fb_A[1]
    y_back = cube_half - fp_post_back
    y_far = 2.0 * fb_A[0] - y_back          # as far in front as behind
    part = Part.makeBox(2 * fp_lug_x, y_back - y_far, 2 * fp_post_y,
                        v(-fp_lug_x, y_far, z_pin - fp_post_y))
    part = part.fuse(disc_yz((fb_A[0], z_pin), link_w / 2.0 + 1.0,
                             -fp_lug_x, 2 * fp_lug_x))
    part = part.cut(pin_x((fb_A[0], z_pin), fdm_pin_press_d,
                          -fp_lug_x - 1, 2 * fp_lug_x + 2))
    # The pin reaches past the lug on both sides (the same steel that passes
    # through both link arms), so clear its full reach, not just the lug's
    # width. With the foot gone nothing else stands in its insertion path.
    _pin_reach = link_x + link_t / 2.0 + 1.0
    part = part.cut(pin_x((fb_A[0], z_pin), pin_d + 2.0 * run_clr,
                          -_pin_reach, 2.0 * _pin_reach))
    # Where the post meets the wall it is as wide as the links themselves, so
    # cut the link's own swept, oversized envelope out of it — a disc centred
    # on A undercounts that sweep (see _link_swept_envelope).
    swept = cached(_link_swept_envelope,
                   A1 if zs > 0 else A2, "B1" if zs > 0 else "B2")
    part = part.cut(swept)
    # A 45 deg CHAMFER on the front edge that faces the link's knuckle: the
    # post now runs as far in front of its pin as behind it, and in front it
    # sits right over the knuckle. Straight, not the knuckle's own round
    # path — it prints clean, and a round cut only follows the knuckle
    # because that is the easy thing to program (user, 2026-09-21). Legs of
    # 3.1 clear the knuckle's swept envelope; fp_chamfer leaves margin.
    z_edge = z_pin - zs * fp_post_y
    tri = [v(-fp_lug_x - 1.0, y_far - 0.01, z_edge + zs * fp_chamfer),
           v(-fp_lug_x - 1.0, y_far - 0.01, z_edge - zs * 0.01),
           v(-fp_lug_x - 1.0, y_far + fp_chamfer, z_edge - zs * 0.01)]
    cham = Part.Face(Part.makePolygon(tri + [tri[0]])).extrude(
        v(2 * fp_lug_x + 2.0, 0, 0))
    part = part.cut(cham)
    # Tapped from the deck side, along Z: the deck's own block comes down
    # (or up) to this face and the screw goes in from OUTSIDE the cube.
    _z_face = z_pin + zs * fp_post_y
    # Deep enough for the screw's own tip (head sunk 2 mm into the deck's
    # outside face, then fp_deck_len of shank), plus a millimetre.
    _tip = cube_half + 2.0 - fp_deck_len
    _depth = abs(_z_face) - _tip + 1.0
    for sy in fp_deck_ys:
        part = part.cut(cyl(foot_tap_d / 2.0, _depth,
                            v(fp_deck_dx, sy, _z_face), v(0, 0, -zs)))
    return part

def servo_box():
    """(x0, y0, z0, bx, by, bz) of the servo case, standing on the FLOOR with
    its shaft pointing UP (+Z) along the screw. Its 32.2 is the shaft
    direction, so that is its height here; length in X, thickness in Y."""
    ht, ln, th = servo_body
    # The shaft is sv_shaft_end from the case's own end, and that end is the
    # one in the corner: the case then runs back toward the carriage, which
    # is where the room is.
    return (screw_x - sv_shaft_end, screw_y - th / 2.0,
            -cube_half + sv_stand, ln, th, ht)


def _tab_z():
    """Z band of the mounting tabs, sv_tab_from_base up from the case's base
    (datasheet). Upright, they are a shelf, and the bracket holds them."""
    z1 = servo_box()[2] + sv_tab_from_base
    return (z1 - servo_tab_t, z1)


def make_servo_body():
    """MG90-class placeholder: case plus its tab plate, upright on the floor
    under the screw, tabs out to each side in X."""
    x0, y0, z0, bx, by, bz = servo_box()
    # The main case, and the gear tower over the shaft end only.
    body = Part.makeBox(bx, by, sv_case_h, v(x0, y0, z0))
    body = body.fuse(Part.makeBox(sv_tower_len, by, bz - sv_case_h,
                                  v(x0, y0, z0 + sv_case_h)))
    tz0, tz1 = _tab_z()
    body = body.fuse(Part.makeBox(bx + 2 * servo_tab_out, by, tz1 - tz0,
                                  v(x0 - servo_tab_out, y0, tz0)))
    for sx in servo_screw_xs():
        body = body.cut(cyl(servo_screw_d / 2.0 + 0.1, tz1 - tz0 + 2,
                            v(sx, screw_y, tz0 - 1), Z_AXIS))
    # Spline stub, where the coupler to the screw sits: the last 4 mm of
    # the datasheet's 32.2.
    body = body.fuse(cyl(2.5, sv_spline_h, v(screw_x, screw_y, z0 + bz),
                         Z_AXIS))
    return body


def servo_screw_xs():
    x0, _, _, bx, _, _ = servo_box()
    return (x0 - servo_tab_out / 2.0, x0 + bx + servo_tab_out / 2.0)


def make_servo_bracket():
    """The servo's cradle, on the WALL (user, 2026-09-21): a back behind the
    case and a post under each mounting tab. PRINTED AS PART of the wall.

    It was on the floor so the tab screws would come out as vertical holes;
    on the wall they are horizontal, but they are Ø1.6 self-tapping pilots,
    and in exchange the cradle holds on by its whole back. The case itself
    still stands on the floor."""
    tz0, tz1 = _tab_z()
    x0, y0, z0, bx, by, bz = servo_box()
    part = None
    for sx, out in ((x0, -1.0), (x0 + bx, 1.0)):
        post = Part.makeBox(servo_tab_out + 1.0, by, tz0 - z0,
                            v(sx if out > 0 else sx - servo_tab_out - 1.0,
                              y0, z0))
        part = post if part is None else part.fuse(post)
    # A BACK behind the case, filling the strip between it and the wall
    # (user, 2026-09-21), from the floor up to the tabs: the case is held
    # on its wall side as well as on its tabs.
    part = part.fuse(Part.makeBox(bx + 2.0 * (servo_tab_out + 1.0),
                                  cube_half + bracket_weld - (y0 + by), tz1 - z0,
                                  v(x0 - servo_tab_out - 1.0, y0 + by, z0)))
    for sx in servo_screw_xs():
        part = part.cut(cyl(foot_tap_d / 2.0, 6.0, v(sx, screw_y, tz0 - 4.0),
                            Z_AXIS))
    # The cable's way out (user, 2026-09-21). It leaves the case at the
    # shaft end, which is the corner, so a POCKET in that post: blind toward
    # the neighbour's wall (1.5 mm left), open on the motor side, so the lead
    # turns up into the cube and drops in with the servo in place.
    _xw = -cube_half + run_clr + 1.5
    part = part.cut(Part.makeBox(x0 + 1.0 - _xw, screw_y + 2.0 - (y0 - 1.0),
                                 sv_cable_h, v(_xw, y0 - 1.0, z0)))
    # Trimmed to the cube: the cradle is wider than the case, and the case is
    # already out at the corner.
    return part.cut(Part.makeBox(40.0, 60.0, 60.0,
                                 v(-cube_half - 40.0 + run_clr, 0.0,
                                   -cube_half - 10.0)))

def _horn_z():
    """Z of the horn hub's underside and top."""
    _, _, z0s, _, _, bz = servo_box()
    z0 = z0s + bz + horn_lift
    return z0, z0 + horn_hub_h


def _cpl_z():
    """Z of the coupler's base, its floor's top face, and its top face."""
    h0, h1 = _horn_z()
    return h0, h1 + cpl_float + cpl_floor, cpl_top_z


def _thrust_z():
    """Z of the thrust plate's underside and top, and of the collar's base."""
    tp0 = cpl_top_z + tw_t
    return tp0, tp0 + tp_t, tp0 + tp_t + tw_t


def _horn_outline(grow, z0, h):
    """The horn's hub and its cut-down arms (along X), grown by `grow`."""
    hub = cyl(horn_hub_d / 2.0 + grow, h, v(screw_x, screw_y, z0), Z_AXIS)
    half = horn_arm_len / 2.0 + grow
    w = horn_arm_w + 2.0 * grow
    _h1 = _horn_z()[1]
    arms = Part.makeBox(2.0 * half, w, horn_arm_t + grow,
                        v(screw_x - half, screw_y - w / 2.0,
                          _h1 - horn_arm_t - grow))
    return hub.fuse(arms)


def make_servo_horn():
    """The servo's double-arm horn, as it ships, arms cut short: BOUGHT."""
    h0, h1 = _horn_z()
    body = _horn_outline(0.0, h0, h1 - h0)
    body = body.cut(cyl(2.6, h1 - h0 - 1.0, v(screw_x, screw_y, h0 - 0.1),
                        Z_AXIS))                        # its spline socket
    return body.cut(cyl(horn_screw_d / 2.0 + 0.1, h1 - h0 + 2.0,
                        v(screw_x, screw_y, h0 - 1.0), Z_AXIS))


def _screw_bore_r():
    """A running bore on the screw's crests, with the printer's allowance."""
    return (fdm_shaft_hole_d - 5.0 + screw_d) / 2.0


def make_coupler():
    """The coupler: PRINTED. The horn in a pocket in its base, free in Z;
    above it a floor, and a D bore gripped on the screw's flat by a grub.
    Its top face is the thrust stack's lower collar.

    Printed with its top on the bed: the D bore opens on the bed, the floor
    bridges over it, and the horn's pocket opens upward — nothing needs
    support, and the D comes out a true vertical hole."""
    c0, cf, c1 = _cpl_z()
    h0, h1 = _horn_z()
    g = cpl_fit
    top = h1 + cpl_float
    body = cyl(cpl_d / 2.0, c1 - c0, v(screw_x, screw_y, c0), Z_AXIS)
    pocket = cyl(horn_hub_d / 2.0 + g, top - c0 + 1.0,
                 v(screw_x, screw_y, c0 - 1.0), Z_AXIS)
    half = horn_arm_len / 2.0 + g
    w = horn_arm_w + 2.0 * g
    # The arms' slot runs right down to the base: the arms are at the TOP
    # of the hub, so a slot only at their height leaves the base closed under
    # them and the horn cannot go in.
    pocket = pocket.fuse(Part.makeBox(2.0 * half, w, top - c0 + 1.0,
                                      v(screw_x - half, screw_y - w / 2.0,
                                        c0 - 1.0)))
    body = body.cut(pocket)
    # the horn screw's head goes through the floor: the floor clamps nothing
    body = body.cut(cyl(2.6, cpl_floor + 2.0,
                        v(screw_x, screw_y, top - 1.0), Z_AXIS))
    r = _screw_bore_r()
    bore = cyl(r, c1 - cf + 1.0, v(screw_x, screw_y, cf), Z_AXIS)
    y_flat = screw_y - (screw_flat - screw_d / 2.0) - 0.2
    bore = bore.cut(Part.makeBox(2 * r + 2, y_flat - (screw_y - r - 1.0),
                                 c1 - cf + 2.0,
                                 v(screw_x - r - 1, screw_y - r - 1.0, cf - 0.5)))
    body = body.cut(bore)
    # the grub's tapped hole, square onto the flat
    return body.cut(cyl(foot_tap_d / 2.0, cpl_d / 2.0 + 1.0,
                        v(screw_x, screw_y - cpl_d / 2.0 - 0.5, (cf + c1) / 2.0),
                        Y_AXIS))


def make_coupler_grub():
    """M3 grub: through the coupler's side onto the screw's flat."""
    _, cf, c1 = _cpl_z()
    y0 = screw_y - cpl_d / 2.0 + 0.5
    y1 = screw_y - (screw_flat - screw_d / 2.0)
    return cyl(1.5, y1 - y0, v(screw_x, y0, (cf + c1) / 2.0), Y_AXIS)


def make_horn_screw():
    """The horn's own M2.5 (stock): holds the horn on the spline, and only
    that — its head sits in the coupler's floor without touching it."""
    _, h1 = _horn_z()
    return make_screw(v(screw_x, screw_y, h1), v(0, 0, -1), horn_screw_d,
                      horn_hub_h, 4.5, cpl_head_h)


def make_thrust_washer(lo):
    """PTFE 8 x 12 x 1: under the thrust plate (lo) or on it."""
    tp0, tp1, _ = _thrust_z()
    z0 = tp0 - tw_t if lo else tp1
    ring = cyl(tw_od / 2.0, tw_t, v(screw_x, screw_y, z0), Z_AXIS)
    return ring.cut(cyl(screw_d / 2.0, tw_t + 2, v(screw_x, screw_y, z0 - 1),
                        Z_AXIS))


def make_lock_collar():
    """T8 lock collar on the screw, above the thrust plate: the down stop."""
    _, _, zc = _thrust_z()
    ring = cyl(col_d / 2.0, col_h, v(screw_x, screw_y, zc), Z_AXIS)
    return ring.cut(cyl(screw_d / 2.0, col_h + 2, v(screw_x, screw_y, zc - 1),
                        Z_AXIS))


def make_screw_shaft():
    """The lead screw itself: from the coupler's D bore, through the thrust
    plate, past the nut's whole travel and up into its top bearing. Its
    bottom end has a flat filed on it, on the -Y side."""
    _, cf, c1 = _cpl_z()
    z0 = cf + 0.5
    z1 = cube_half - 0.5
    sh = cyl(screw_d / 2.0, z1 - z0, v(screw_x, screw_y, z0), Z_AXIS)
    y_flat = screw_y - (screw_flat - screw_d / 2.0)
    return sh.cut(Part.makeBox(screw_d + 2, y_flat - (screw_y - screw_d / 2.0 - 1.0),
                               c1 - z0 + 1.5,
                               v(screw_x - screw_d / 2.0 - 1,
                                 screw_y - screw_d / 2.0 - 1.0, z0 - 1.0)))


def make_screw_top():
    """The screw's top bearing AND the guide rod's top anchor, hanging from
    the CEILING — printed with it (user, 2026-09-20).

    Both bores run along Z, and the ceiling is printed flat, so on it they
    come out as true round holes. On the wall the same bores would have been
    horizontal, which is where FDM holes go oval.

    A screw pushes as hard as it pulls, and a servo's output bearing is not
    meant to take either, so the thrust is caught here and at the floor,
    never through the servo."""
    # 3 mm of wall round both bores. It was sized off the nut's body, and
    # left the screw's bore 0.6 mm from two of its faces.
    rs = fdm_shaft_hole_d / 2.0 + 1.5 + 3.0
    rg = guide_d / 2.0 + 3.0
    gx = screw_x + guide_dx
    x0 = min(screw_x - rs, gx - rg)
    x1 = max(screw_x + rs, gx + rg)
    y0 = min(screw_y - rs, guide_y - rg)
    y1 = min(max(screw_y + rs, guide_y + rg), cube_half - 0.6)   # off the wall
    post = Part.makeBox(x1 - x0, y1 - y0, cube_half + bracket_weld - screw_top_z,
                        v(x0, y0, screw_top_z))
    post = post.cut(cyl(fdm_shaft_hole_d / 2.0 + 1.5, 30.0,
                        v(screw_x, screw_y, screw_top_z - 1.0), Z_AXIS))
    return post.cut(cyl(fdm_pin_press_d / 2.0, 30.0,
                        v(screw_x + guide_dx, guide_y, screw_top_z - 1.0),
                        Z_AXIS))


def make_guide_foot():
    """The guide rod's bottom anchor: a short arm off the WALL, printed with
    it. It sits above the servo and below the nut's travel — the rod cannot
    be footed on the servo's own bracket, because at that height the case
    itself is in the way."""
    r = guide_d / 2.0 + 3.0
    # From r IN FRONT of the rod's axis, not from the axis itself: a box that
    # starts on the axis leaves the bore half open, and half a bore holds
    # half a rod.
    # Narrow on BOTH sides in X: the carriage passes on one, and on the other
    # the screw, which the rod now stands only 8.5 mm from.
    r_in = 3.5
    gx = screw_x + guide_dx
    x_lo, x_hi = gx - r_in, gx + r_in
    tp0, tp1, _ = _thrust_z()
    h = 10.0
    y_w = cube_half + bracket_weld
    body = Part.makeBox(x_hi - x_lo, y_w - (guide_y - r), h,
                        v(x_lo, guide_y - r, tp0))
    # ...and on round the screw as its THRUST plate (user, 2026-09-21)
    rb = _screw_bore_r()
    xs0 = screw_x - rb - 3.0
    ys0 = screw_y - rb - 3.0
    body = body.fuse(Part.makeBox(x_hi - xs0, y_w - ys0, tp_t,
                                  v(xs0, ys0, tp0)))
    # room over the plate for the washer and the collar, turning
    body = body.cut(cyl(col_d / 2.0 + 0.8, h, v(screw_x, screw_y, tp1), Z_AXIS))
    body = body.cut(cyl(rb, tp_t + 2.0, v(screw_x, screw_y, tp0 - 1.0), Z_AXIS))
    return body.cut(cyl(fdm_pin_press_d / 2.0, h + 2.0,
                        v(gx, guide_y, tp0 - 1.0), Z_AXIS))


def make_guide_rod():
    """The anti-rotation rod: Ø4, pressed into the wall bracket at the bottom
    and the screw's top post above, with the nut's pusher sliding on it."""
    z0 = _guide_z0()
    z1 = cube_half - 0.5          # right up into the top post's bore
    return cyl(guide_d / 2.0, z1 - z0,
               v(screw_x + guide_dx, guide_y, z0), Z_AXIS)


def _guide_z0():
    return guide_z0


def _nut_body_z(st):
    """Z of the nut's FLANGE underside, which is the cage's top face.

    The flange sits on the upper seat plate and the body hangs down into the
    cage beside the springs, where the chamber is empty: that keeps the whole
    assembly low enough to clear the screw's top bearing."""
    return st["nut_z"] + _cage_gap() / 2.0 + cage_top_t


def _cage_gap():
    """Clear height between the driver's two seat plates: the carrier, its
    two springs, and the stroke they have to give each way."""
    return carrier_bush + 2.0 * spr_h + 2.0 * (spring_stroke + spr_clr)


def make_brass_nut(st):
    """The nut itself: BOUGHT, brass. Flange on top of the cage, body hanging
    down into it, the screw's thread through both."""
    zf = _nut_body_z(st)
    fx, fy, ft = nut_fl
    body = cyl(nut_d / 2.0, nut_l - ft, v(screw_x, screw_y, zf - (nut_l - ft)),
               Z_AXIS)
    body = body.fuse(Part.makeBox(fx, fy, ft,
                                  v(screw_x - fx / 2.0, screw_y - fy / 2.0, zf)))
    body = body.cut(cyl(screw_d / 2.0 - 0.6, nut_l + 2,
                        v(screw_x, screw_y, zf - nut_l), Z_AXIS))
    for sx in (-nut_fl_pitch / 2.0, nut_fl_pitch / 2.0):
        body = body.cut(cyl(nut_fl_hole / 2.0, ft + 2,
                            v(screw_x + sx, screw_y, zf - 1), Z_AXIS))
    return body


def make_nut(st):
    """The CAGE, printed: it grips the brass nut and holds the carrier and
    its two springs.

    Built as ONE block with only what has to be empty cut out of it — not as
    collar, plates, web, spine and roof fused one by one, which left slivers
    and steps between them that did nothing (the user found three). What is
    cut out:
      * the CHAMBER between the two seat plates, where carrier and springs
        live. Open toward the wall, so they go in from there, and toward the
        ear, where the carrier's arm leaves. Its motor-side wall is the
        'roof', taking the wall as the floor, as the user does.
      * the brass nut's pocket above it, also open toward the wall;
      * the screw's bore, and the guide rod's.
    The nut pushes a plate, the plate pushes a spring, the spring pushes the
    carrier: that is the series compliance, both ways."""
    z = st["nut_z"]
    zn = _nut_body_z(st)
    g = _cage_gap()
    gx = screw_x + guide_dx
    roof_t = 2.5
    # Out far enough to take the flange's outer screw WHOLE — the cage used
    # to end on that screw's own axis (user, 2026-09-21) — and no closer to
    # the neighbour's wall than a running gap.
    x0 = min(screw_x - nut_d / 2.0 - 3.0, gx - spr_od / 2.0 - 2.0,
             max(screw_x - nut_fl_pitch / 2.0 - foot_tap_d / 2.0 - 1.5,
                 -cube_half + run_clr / 2.0 + 0.05))
    x1 = max(screw_x + nut_d / 2.0 + 3.0, gx + spr_od / 2.0 + 0.5)
    y_in = min(screw_y - nut_fl[1] / 2.0,
               guide_y - spr_od / 2.0, _carrier_y()[0]) - 0.5
    y0 = y_in - roof_t
    y1 = min(screw_y + nut_d / 2.0 + 2.0, cube_half - run_clr)
    z0 = z - g / 2.0 - cage_t
    z1 = zn                          # the cage's top IS the flange's seat
    body = Part.makeBox(x1 - x0, y1 - y0, z1 - z0, v(x0, y0, z0))
    # the chamber: from the spine at x0 to open at x1, from the roof to open
    # at the wall side, between the seat plates
    spine_t = 2.5
    body = body.cut(Part.makeBox(x1 - x0 - spine_t + 1.0, y1 - y_in + 1.0, g,
                                 v(x0 + spine_t, y_in, z - g / 2.0)))
    # the brass nut's body goes down through the top into the chamber, and
    # its flange screws down onto the top face: two pilots for M3
    body = body.cut(cyl(nut_d / 2.0 + 0.2, nut_l - nut_fl[2] + 0.5,
                        v(screw_x, screw_y, zn - (nut_l - nut_fl[2]) - 0.5),
                        Z_AXIS))
    for sx in (-nut_fl_pitch / 2.0, nut_fl_pitch / 2.0):
        body = body.cut(cyl(foot_tap_d / 2.0, 7.0,
                            v(screw_x + sx, screw_y, zn - 7.0), Z_AXIS))
    # Clear of the carriage's ring, which its lower corner comes near when
    # the carriage tilts down: a cut of the ring's own cylinder, grown by its
    # swing, takes exactly what is in the way and nothing else.
    _h = carr_half + 1.5
    body = body.cut(Part.makeBox(2 * _h, hous_y1 - hous_y0 + 2.0, 2 * _h,
                                 v(-_h, hous_y0 - 1.0, -_h)))
    # bores
    body = body.cut(cyl(screw_d / 2.0 + 0.2, z1 - z0 + 2.0,
                        v(screw_x, screw_y, z0 - 1.0), Z_AXIS))
    return body.cut(cyl(guide_d / 2.0 + 0.25, z1 - z0 + 2.0,
                        v(gx, guide_y, z0 - 1.0), Z_AXIS))

def _carrier_y():
    """Y band of the carrier's arm: wide enough to take its bush on the rod
    at one end and the ear's pin slot at the other."""
    lo = min(guide_y, ear_y) - 4.5
    hi = max(guide_y, ear_y) + 4.5
    return lo, hi


def make_carrier(st):
    """The floating carrier: it slides on the guide rod between the two
    springs, and its arm is what actually reaches the ear."""
    z = st["nut_z"]
    gx = screw_x + guide_dx
    # A BUSH on the rod, not a plate with a hole: the rod takes the moment
    # the arm makes, and it takes it on this length.
    _r = guide_d / 2.0 + 3.0
    body = cyl(_r, carrier_bush,
               v(gx, guide_y, z - carrier_bush / 2.0), Z_AXIS)
    # ONE block, the bush's own height, from the rod to the carriage's face:
    # rounded round the rod, flat where it faces the ear. The arm is short
    # now (7.6 mm), so a thin plate with a boss on its end bought nothing —
    # the springs seat on its whole faces and the pin has 3.5 mm of wall
    # above and below it.
    _x_end = _x_face_push()
    _y0, _y1 = _carrier_y()
    _y0 = max(_y0, guide_y - _r)
    body = body.fuse(Part.makeBox(_x_end - gx, _y1 - _y0, carrier_bush,
                                  v(gx, _y0, z - carrier_bush / 2.0)))
    body = body.cut(cyl(guide_d / 2.0 + 0.25, carrier_bush + 2,
                        v(gx, guide_y, z - carrier_bush / 2.0 - 1), Z_AXIS))
    # A SLOT, not a hole: the ear swings on its arc while the carrier goes
    # straight up, and the difference — 0.64 mm over the stroke — has to go
    # somewhere. It goes here.
    # Free in Y, TIGHT in Z. The ear's arc has to be swallowed, and that
    # arc is in Y; in Z the slot is the load path, and 0.6 mm of slop there
    # is more than the spring's whole working stroke — the actuation would
    # spend its travel taking up its own slack.
    _sl_y = fdm_act_hole_d + 1.6
    _sl_z = act_pin_d + 0.2
    # BLIND: open on the face toward the carriage, 0.7 mm past the pin's
    # end, so it keeps its wall off the rod's bore behind it.
    _xa = _x_end - ear_sx * 1.0
    _xb = _x_end + ear_sx * (ear_pin_out + 0.7)
    return body.cut(Part.makeBox(abs(_xb - _xa), _sl_y, _sl_z,
                                 v(min(_xa, _xb),
                                   ear_y - _sl_y / 2.0,
                                   st["E"][1] - _sl_z / 2.0)))


def make_ear_pin(st):
    """The one pin left in the whole actuation: it joins the carrier's arm to
    the ear, pressed into the carriage block's face and running in the
    carrier's slot."""
    x_in = x_ear[1] if ear_sx < 0 else x_ear[0]
    x_out = _x_face_push() + ear_sx * ear_pin_out
    return pin_x(st["E"], act_pin_d, min(x_in, x_out), abs(x_out - x_in))


def make_spring_stack(st, side):
    """One of the two stacks, as its envelope: seated on a plate of the
    driver's cage at one end and on the carrier at the other.

    Its rate and stroke are the screw's, not a catalogue's: spring_rate over
    spring_stroke. At ~140 N/mm over 0.4 mm that is die-spring or Belleville
    territory — a plain coil spring this short cannot do it."""
    z = st["nut_z"] + side * (carrier_bush / 2.0)
    z0 = z if side > 0 else z - spr_h
    return cyl(spr_od / 2.0, spr_h,
               v(screw_x + guide_dx, guide_y, z0), Z_AXIS).cut(
        cyl(spr_id / 2.0, spr_h + 2,
            v(screw_x + guide_dx, guide_y, z0 - 1), Z_AXIS))


def _plate_bar(pts, x_band, w, press=(), holes=None):
    """A flat bar through (y, z) points in an X band, holes at the indices in
    `holes` (default: all of them): press fit at the indices in `press`,
    running fit elsewhere. A point with no hole is a KNEE, not a joint."""
    body = None
    for p, q in zip(pts, pts[1:]):
        b = bar_yz(p, q, w, x_band[0], x_band[1] - x_band[0])
        body = b if body is None else body.fuse(b)
    for i, p in enumerate(pts):
        if holes is not None and i not in holes:
            continue
        d = fdm_act_press_d if i in press else fdm_act_hole_d
        body = body.cut(pin_x(p, d, x_band[0] - 1,
                              x_band[1] - x_band[0] + 2))
    return body


def _x_face_push():
    """X of the carrier's outboard face: a running gap off the carriage
    block's face, where the ear is."""
    return -(carr_half + act_gap) if ear_sx < 0 else (carr_half + act_gap)


def make_flange_screw(st, sx):
    """One of the two M3 screws that hold the nut's flange down on the cage:
    from above, through the flange, into the cage."""
    zf = _nut_body_z(st) + nut_fl[2]
    # M3 x 10: 4 through the flange, 6 into the cage.
    return make_screw(v(screw_x + sx, screw_y, zf), v(0, 0, -1), 3.0, 10.0,
                      5.5, 2.0)


def act_moving_parts(st):
    """The nut and its pusher. The screw turns but does not move, so it is a
    fixed part; the servo likewise."""
    return [("ScrewNut", make_brass_nut(st), (0.72, 0.55, 0.30), 0),
            ("NutScrew0", make_flange_screw(st, -nut_fl_pitch / 2.0),
             (0.35, 0.35, 0.38), 0),
            ("NutScrew1", make_flange_screw(st, nut_fl_pitch / 2.0),
             (0.35, 0.35, 0.38), 0),
            ("ActNut", make_nut(st), (0.30, 0.55, 0.85), 0),
            ("SpringCarrier", make_carrier(st), (0.30, 0.55, 0.85), 0),
            ("EarPin", make_ear_pin(st), (0.45, 0.45, 0.50), 0),
            ("SpringUp", make_spring_stack(st, 1), (0.85, 0.85, 0.20), 0),
            ("SpringDown", make_spring_stack(st, -1), (0.85, 0.85, 0.20), 0)]


def both_hands(pts):
    """The screw pattern, as given.

    It used to add the X mirror of every hole, because alternate axes were
    assembled turned over and their brackets came out mirrored. Nothing is
    turned over any more (invariant 7: identical rotated copies), so the
    mirrored half was holes nobody used — and worse, on this wall they landed
    in the actuation's own corner, where they ran into the servo's cradle and
    the spring cage."""
    return list(pts)

def deck_screws():
    """(x, y) of the screws into one deck, for ONE axis. EMPTY: the frame
    pivots and the servo both moved to the wall (see wall_screws), so the
    deck is a plain cap now, nothing is bolted to it. The list stays so the
    deck's own boss loop and anything that used to read this cannot drift."""
    return []


def make_deck(zs):
    """Floor or ceiling. Nothing is bolted to it any more — the frame pivots
    and the servo both moved to the wall (deck_screws() is empty) — so it is
    a plain structural cap now, not something the mechanism hangs off. Kept
    as its own part rather than folded away: it still closes the cube's top
    and bottom, and both_hands()' screw pattern lives on here for when
    something needs it again.

    Bosses inward, blind holes, same as the walls: nothing goes through, so
    the outside stays clean."""
    z_in = zs * cube_half
    z_out = zs * (cube_half + deck_t)
    deck = Part.makeBox(2 * cube_out, 2 * cube_out, deck_t,
                        v(-cube_out, -cube_out, min(z_in, z_out)))
    deck = deck.cut(cyl(motor_shaft_d / 2.0 + 1.0, deck_t + 2,
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
    for _, rot in AXES:
        # The blocks that reach the four-bar's posts, one per axis — and then
        # the holes through them, cut LAST.
        deck = deck.fuse(place(frame_pad(zs), rot))
    for _, rot in AXES:
        deck = deck.cut(place(frame_pad_holes(zs), rot))
    if zs > 0:
        # The screw's and the rod's top anchors: their bores run along Z, so
        # the ceiling is the part they come out true on.
        for _, rot in AXES:
            deck = deck.fuse(place(cached(make_screw_top), rot))
    return _deck_column_features(deck, zs)


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
    # The output shaft's one bearing: mostly INSIDE the wall's own 4 mm
    # thickness (it is a 4 mm wide MR105ZZ), with just enough boss standing
    # proud outside it to fit the shoulder. Pressed in from outside, stops
    # against that shoulder — same convention as the carriage's own pair.
    wall = wall.fuse(cyl(brg_od / 2.0 + out_brg_wall, out_brg_len,
                         v(0, cube_half, 0), Y_AXIS))
    wall = wall.cut(cyl(brg_od / 2.0 + brg_fit_press, out_brg_len - brg_seat_lip + 1.0,
                        v(0, cube_half + brg_seat_lip, 0), Y_AXIS))
    # The INNER bearing: a boss on the inside face, seat open inward, the
    # bearing pressed in until it stops against the wall's own hole.
    wall = wall.fuse(cyl(brg_od / 2.0 + out_brg_wall, brg_w + 0.5,
                         v(0, cube_half - brg_w, 0), Y_AXIS))
    wall = wall.cut(cyl(brg_od / 2.0 + brg_fit_press, brg_w + 1.0,
                        v(0, cube_half - brg_w - 1.0, 0), Y_AXIS))
    wall = wall.cut(cyl(shaft_d / 2.0 + wall_shaft_clr, out_brg_len + brg_w + 3,
                        v(0, cube_half - brg_w - 1, 0), Y_AXIS))
    # Bosses on the INNER face, blind. Nothing passes through, so the outside
    # of the cube stays a clean surface.
    for sx, sz in both_hands(wall_screws()):
        wall = wall.fuse(cyl(wall_boss_d / 2.0, wall_boss_h,
                             v(sx, wall_face_y, sz), Y_AXIS))
        wall = wall.cut(cyl(foot_tap_d / 2.0, wall_boss_h + 2.0,
                            v(sx, wall_face_y, sz), Y_AXIS))
    # And the guide rod's foot, which is the one anchor that has to come off
    # this wall: it sits above the servo, where neither floor nor ceiling can
    # reach it without passing through the case.
    wall = wall.fuse(cached(make_guide_foot))
    wall = wall.fuse(cached(make_servo_bracket))   # the servo's cradle
    # The output face (user, 2026-09-21): the square spigot, the male half's
    # socket through it, and four magnet pockets.
    yf = cube_half + wall_thick
    wall = wall.fuse(Part.makeBox(sp_side, sp_h, sp_side,
                                  v(-sp_side / 2.0, yf, -sp_side / 2.0)))
    y_sock = yf - _jc_sink() - jc_clr
    wall = wall.cut(cyl(jc_d / 2.0 + jc_clr, yf + sp_h + 1.0 - y_sock,
                        v(0, y_sock, 0), Y_AXIS))
    for x, z in _face_magnet_pts():
        wall = wall.cut(cyl(mag_d / 2.0 + 0.1, mag_t + 0.2 + 1.0,
                            v(x, yf - mag_t - 0.2, z), Y_AXIS))
    return wall


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

# ── Carriage, built at rest then moved by the linkage ────────────────────────
CARRIAGE_REST = [
    ("OutputCone",    make_output_cone(),                     (0.20, 0.80, 0.60), 0),
    ("OutputRubber",  make_output_rubber(),                   (0.15, 0.15, 0.18), 0),
    ("Carriage",      make_carriage(),                        (0.70, 0.70, 0.72), 0),
    ("NeckBearing",   make_neck_bearing(),                    (0.30, 0.30, 0.32), 0),
]

# ── Fixed to the frame ──────────────────────────────────────────────────────
FIXED_PARTS = [
    ("OutputShaft",   make_output_shaft(),                    (0.60, 0.60, 0.60), 0),
    ("JawMale",       make_jaw_wall(),                        (0.70, 0.30, 0.25), 0),
    ("WallMagnet0",   make_wall_magnet(0),                    (0.55, 0.55, 0.60), 0),
    ("WallMagnet1",   make_wall_magnet(1),                    (0.55, 0.55, 0.60), 0),
    ("WallMagnet2",   make_wall_magnet(2),                    (0.55, 0.55, 0.60), 0),
    ("WallMagnet3",   make_wall_magnet(3),                    (0.55, 0.55, 0.60), 0),
    ("UJFork",        make_uj_fork(),                         (0.85, 0.65, 0.10), 0),
    ("BearingOutput", make_carriage_bearing(cube_half + out_brg_len, -1),
                                                              (0.30, 0.30, 0.32), 0),
    ("BearingOutIn",  make_carriage_bearing(cube_half, -1),   (0.30, 0.30, 0.32), 0),
    ("FramePostT",    cached(make_frame_bracket, 1),           (0.75, 0.75, 0.78), 0),
    ("FramePostB",    cached(make_frame_bracket, -1),          (0.75, 0.75, 0.78), 0),
    ("ServoBody",     make_servo_body(),                      (0.20, 0.25, 0.30), 0),
    ("ServoBracket",  cached(make_servo_bracket),             (0.75, 0.75, 0.78), 0),
    ("ScrewShaft",    cached(make_screw_shaft),               (0.60, 0.60, 0.60), 0),
    ("ServoHorn",     cached(make_servo_horn),                (0.12, 0.12, 0.12), 0),
    ("Coupler",       cached(make_coupler),                   (0.30, 0.65, 0.60), 0),
    ("HornScrew",     cached(make_horn_screw),                (0.35, 0.35, 0.38), 0),
    ("CouplerGrub",   cached(make_coupler_grub),              (0.35, 0.35, 0.38), 0),
    ("ThrustWasherLo", cached(make_thrust_washer, True),      (0.92, 0.92, 0.90), 0),
    ("ThrustWasherHi", cached(make_thrust_washer, False),     (0.92, 0.92, 0.90), 0),
    ("LockCollar",    cached(make_lock_collar),               (0.35, 0.35, 0.38), 0),
    ("FrameScrewT0",  make_frame_screw(1, fp_deck_ys[0]),     (0.35, 0.35, 0.38), 0),
    ("FrameScrewT1",  make_frame_screw(1, fp_deck_ys[1]),     (0.35, 0.35, 0.38), 0),
    ("FrameScrewB0",  make_frame_screw(-1, fp_deck_ys[0]),    (0.35, 0.35, 0.38), 0),
    ("FrameScrewB1",  make_frame_screw(-1, fp_deck_ys[1]),    (0.35, 0.35, 0.38), 0),
    ("GuideRod",      cached(make_guide_rod),                 (0.45, 0.45, 0.50), 0),
    ("GuideFoot",     cached(make_guide_foot),                (0.75, 0.75, 0.78), 0),
    ("ScrewTop",      cached(make_screw_top),                 (0.75, 0.75, 0.78), 0),
    ("Wall",          make_wall(),                            (0.45, 0.55, 0.75), 70),
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
    for zs in (1, -1) for i, (sx, sy) in enumerate(deck_screws())
]

_LINK_COL = (0.90, 0.45, 0.20)
_PIN_COL = (0.45, 0.45, 0.50)
_ACT_COL = (0.30, 0.55, 0.85)


_MOVING_CACHE = {}


def moving_parts(st):
    """Every part whose position depends on the tilt: the carriage, the four
    links with their pins, and the actuation train. Cached per pose (the
    sweeps ask for the same ones over and over); nothing may mutate what
    comes back — see cached()."""
    key = round(st["phi"], 12)
    if key in _MOVING_CACHE:
        return _MOVING_CACHE[key]
    out = [(n, place_carriage(sh, st), c, t) for n, sh, c, t in CARRIAGE_REST]
    pin_reach = link_x + link_t / 2.0 + 1.0
    pin_reach_b = carr_half - 0.5
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
        # Shim washers, one at each thrust face: link outer face at B, link
        # inner face at A (against the lug), both sides.
        # Built at -x directly, not mirrored (see the FreeCAD gotchas).
        for jn, q, xa in (("B", B, link_x + link_t / 2.0),
                          ("A", A, link_x - link_t / 2.0 - shim_gap)):
            for xs in (1, -1):
                x0 = xa if xs > 0 else -(xa + shim_gap)
                w = cyl(shim_od / 2.0, shim_gap, v(x0, q[0], q[1]), X_AXIS)
                w = w.cut(cyl(shim_id / 2.0, shim_gap + 2, v(x0 - 1, q[0], q[1]), X_AXIS))
                out.append((f"Shim{tag}{jn}{'+' if xs > 0 else '-'}", w, _PIN_COL, 0))
    # The cardan's moving parts, at the length and angle this pose gives the
    # intermediate. Drawn in the phase where the bend lies in the tilt plane:
    # both joints hinge on their X pins, the Z pins ride along.
    L_now = uj_geom(st["phi"])[0]
    out.append(("UJMid", uj_place(make_uj_mid(L_now), st), (0.90, 0.35, 0.20), 0))
    out.append(("UJRing", uj_place(make_uj_ring(L_now), st), (0.50, 0.45, 0.85), 0))
    out.append(("UJCross", uj_place(make_uj_cross(), st), (0.50, 0.45, 0.85), 0))
    out += act_moving_parts(st)
    _MOVING_CACHE[key] = out
    return out


STATE = pose_state(phi)
AXIS_PARTS = moving_parts(STATE) + FIXED_PARTS
BY_NAME = {n: sh for n, sh, c, t in AXIS_PARTS}


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


# Parts that are PRINTED AS PART of another one. They stay in AXIS_PARTS,
# because the checks need to see them as themselves — that they merge, that
# they clear what they pass — but they are not DRAWN separately: their host
# already contains them, and drawing both shows every one of them twice.
WELDED_INTO = {"ServoBracket": "Wall", "ScrewTop": "DeckTop",
               "GuideFoot": "Wall"}

for _ax_name, _ax_rot in AXES[:AXES_SHOWN]:
    for _pname, _pshape, _pcolor, _ptrans in AXIS_PARTS:
        if _pname in WELDED_INTO:
            continue
        add(doc, f"{_pname}_{_ax_name}", place(_pshape, _ax_rot),
            color=_pcolor, transparency=_ptrans)

# ── Central column ──────────────────────────────────────────────────────────
_STEEL = (0.6, 0.6, 0.62)
COLUMN_PARTS = [("MotorShaft", make_motor_shaft(), _STEEL),
                ("JawTop", make_jaw_top(), (0.70, 0.30, 0.25)),
                ("JawBottom", make_jaw_bottom(), (0.70, 0.30, 0.25))]
COLUMN_PARTS += [(f"DeckMagnet{'T' if _zs > 0 else 'B'}{_k}",
                  make_deck_magnet(_zs, _k), (0.55, 0.55, 0.60))
                 for _zs in (1, -1) for _k in range(4)]
for _sd, _tag in ((-1, "Lower"), (1, "Upper")):
    COLUMN_PARTS += [
        (f"MotorBearing{_tag}", make_motor_bearing(_sd), (0.30, 0.30, 0.32)),
        (f"MotorWasher{_tag}", make_motor_washer(_sd), _STEEL),
        ((f"MotorCollar{_tag}", make_motor_collar(_sd), (0.35, 0.35, 0.38))
         if _sd > 0 else ("MotorSpacerLower", make_motor_spacer(), (0.30, 0.65, 0.60))),
        (f"MotorFlange{_tag}", make_motor_flange(_sd), _STEEL)]
    COLUMN_PARTS += [(f"MotorFlangeScrew{_tag}{_k}",
                      make_motor_flange_screw(_sd, _k), (0.35, 0.35, 0.38))
                     for _k in range(4)]
for _sd, _tag in ((-1, "Lower"), (1, "Upper")):
    add(doc, f"MotorCone{_tag}", make_motor_cone(_sd), color=(1.0, 0.60, 0.15))
    add(doc, f"MotorRubber{_tag}", make_motor_rubber(_sd), color=(0.15, 0.15, 0.18))

for _zs, _tag in ((1, "Top"), (-1, "Bottom")):
    add(doc, f"Deck{_tag}", cached(make_deck, _zs), color=(0.45, 0.55, 0.75),
        transparency=70)

for _n, _sh, _c in COLUMN_PARTS:
    add(doc, _n, _sh, color=_c)

doc.recompute()

if HAS_GUI:
    try:
        Gui.ActiveDocument = Gui.getDocument(doc.Name)
        Gui.SendMsgToActiveView("ViewFit")
        Gui.activeDocument().activeView().viewIsometric()
    except AttributeError:
        pass   # freecadcmd: no real view, geometry is already built

if RUN_CHECKS is None:
    RUN_CHECKS = not getattr(App, "GuiUp", 0)

if RUN_CHECKS:
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
    # Nothing is meant to share volume: the Oldham's tongues live inside the
    # disc's axial float, which the sweep checks like any other clearance.
    _MESH_PAIRS = set()


    _WELDED = [{"DeckTop", "ScrewTop"}, {"Wall", "ServoBracket"},
               {"Wall", "GuideFoot"}]


    def _exempt(na, nb):
        """Pairs that are MEANT to share volume.

        Meshing gears, drawn at their free-pose phase (the mesh itself is checked
        at free, and a phase is proven to exist at tilt). And a self-tapping screw
        in its pilot hole: the screw is Ø3 into a Ø2.6 pilot, and that difference
        is the thread it forms. Everything else that overlaps is a mistake."""
        if (na, nb) in _MESH_PAIRS or (nb, na) in _MESH_PAIRS:
            return True
        for host, screw in (("Wall", "WallScrew"), ("DeckTop", "DeckScrewT"),
                            ("DeckBottom", "DeckScrewB"),
                            ("FramePostT", "FrameScrewT"),
                            ("FramePostB", "FrameScrewB")):
            if {na, nb} == {host} | {n for n in (na, nb) if n.startswith(screw)}:
                return True
        # A bracket printed as part of its host IS its host: they are meant to
        # share the bracket_weld millimetres where they merge. That they really
        # merge (and merge into ONE solid) is its own check.
        if {na, nb} in _WELDED:
            return True
        # A nut on its own thread: the bore is the thread, so it shares metal
        # with the screw by definition.
        if {na, nb} == {"ScrewNut", "ScrewShaft"}:
            return True
        # the flange's screws: threaded through the flange, self-tapping into
        # the cage below it
        if {na, nb} & {"NutScrew0", "NutScrew1"} and \
                {na, nb} & {"ScrewNut", "ActNut"}:
            return True
        # the horn's screw threads into the spline, and the spline is inside
        # the horn's socket: both modelled solid
        if {na, nb} in ({"HornScrew", "ServoBody"}, {"ServoHorn", "ServoBody"},
                        {"CouplerGrub", "Coupler"}):
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
    _deck_top, _deck_bot = cached(make_deck, 1), cached(make_deck, -1)
    _SHARED = [("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up),
               ("DeckTop", _deck_top), ("DeckBottom", _deck_bot)] + \
              [(n, sh) for n, sh, c in COLUMN_PARTS]
    # The column among itself (nothing else looked at these pairs): overlaps
    # only where a screw goes into its part.
    _col = [("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up),
            ("DeckTop", _deck_top), ("DeckBottom", _deck_bot)] + \
           [(n, sh) for n, sh, c in COLUMN_PARTS]
    _col_ov, _col_who = 0.0, "-"
    for _i in range(len(_col)):
        for _j in range(_i + 1, len(_col)):
            _na, _nb = _col[_i][0], _col[_j][0]
            if "FlangeScrew" in _na or "FlangeScrew" in _nb:
                _other = _nb if "FlangeScrew" in _na else _na
                if _other.startswith(("MotorCone", "MotorFlange")):
                    continue
            if _bb_hit(_col[_i][1], _col[_j][1]):
                _ov = _col[_i][1].common(_col[_j][1]).Volume
                if _ov > max(_col_ov, 1e-6):
                    _col_ov, _col_who = _ov, f"{_na} x {_nb}"
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
                    if bb_gap(_sub, _s) >= _link_gap:
                        continue
                    _d = _sub.distToShape(_s)[0]
                    if _d < _link_gap:
                        _link_gap = _d
                        _link_who = f"{_n} at {math.degrees(_phi_k):+.1f} deg"


    # The cardan, swept as DISTANCES, same lesson as the link: its tightest
    # running gaps (intermediate inside the cone's bore, ring bore round the output
    # shaft, intermediate's head against the inner bearing boss) move with the
    # tilt, and "== 0 mm3" at three stops passes a part rubbing at half travel.
    _uj_gap, _uj_who = 1e9, "-"
    _fixed = {n: s for n, s, c, t in FIXED_PARTS}
    for _k in range(-4, 5):
        _st_k = pose_state(phi_preload * _k / 4.0)
        _L_k = uj_geom(_st_k["phi"])[0]
        _mid = uj_place(make_uj_mid(_L_k), _st_k)
        _ring = uj_place(make_uj_ring(_L_k), _st_k)
        # The prism alone: its pins sit in the fork's holes by design.
        _cross = uj_place(make_uj_cross(pins=False), _st_k)
        _cone = place_carriage(cached(make_output_cone), _st_k)
        for _a, _an, _b, _bn in ((_mid, "UJMid", _cone, "OutputCone"),
                                 (_mid, "UJMid", _fixed["Wall"], "Wall"),
                                 (_mid, "UJMid", _fixed["BearingOutIn"], "BearingOutIn"),
                                 (_mid, "UJMid", _fixed["UJFork"], "UJFork"),
                                 (_ring, "UJRing", _fixed["OutputShaft"], "OutputShaft"),
                                 (_cross, "UJCross", _fixed["UJFork"], "UJFork")):
            if bb_gap(_a, _b) >= _uj_gap:
                continue
            _d = _a.distToShape(_b)[0]
            if _d < _uj_gap:
                _uj_gap = _d
                _uj_who = f"{_an} x {_bn} at {math.degrees(_st_k['phi']):+.1f} deg"

    # The actuation chain over the whole stroke: it must assemble everywhere, the
    # horn must swing about what the sweep split was designed for, and no joint may
    # go near dead centre (a link nearly in line with its arm transmits nothing
    # useful and multiplies slop).
    def _trans(a, b, c):
        """Angle at b between b->a and b->c, folded to 0..90 (90 = best)."""
        u = (a[0] - b[0], a[1] - b[1])
        w = (c[0] - b[0], c[1] - b[1])
        cosang = (u[0] * w[0] + u[1] * w[1]) / (math.hypot(*u) * math.hypot(*w))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        return min(ang, 180.0 - ang)


    _act_fail = 0
    _trans_min = 90.0
    for _k in range(-8, 9):
        _st_k = pose_state(phi_preload * _k / 8.0)
        if _st_k["E"] is None:
            _act_fail += 1
            continue
        # Two joints, so two angles: at the crank (crank to link) and at
        # the ear (link to the ear's own velocity, which is perpendicular to
        # its radius from the apex).
        # A screw has no dead centre to worry about; what matters is that
        # the ear's arc stays close to the nut's straight line, because the
        # pin's slot has to take the difference.
        _trans_min = min(_trans_min, 90.0 - abs(math.degrees(_st_k["phi"])))
    _st_c = pose_state(phi_c)
    _st_p = pose_state(phi_preload)
    _horn_contact = abs(_st_c["servo_deg"])
    _horn_preload = abs(_st_p["servo_deg"])
    # What the nut has to travel, measured from the poses rather than read
    # off the geometry, and how far the ear wanders off the nut's straight
    # line while doing it — the pin's slot has to swallow that.
    _ear_travel = _st_p["E"][1] - E0[1]
    _ear_sideways = max(abs(pose_state(phi_preload * k / 8.0)["E"][0] - E0[0])
                        for k in range(-8, 9))
    # Force at the nut: a screw's own equation, with its efficiency.
    _F_screw = screw_force()
    _spring_stroke = spring_stroke

    # The chain, swept as DISTANCES over the stroke, same lesson as the links and
    # the cardan: its stack is 0.5 mm plate to plate and it runs past the servo's
    # case, the four-bar's post and the carriage, so "== 0 mm3" at three stops is
    # not enough. Pairs joined by a pin are left out — their 0.5 mm is the
    # designed gap between neighbouring plates, not a running clearance.
    _ACT_PINNED = {frozenset(p) for p in (("NutScrew0", "ScrewNut"),
                                          ("NutScrew1", "ScrewNut"),
                                          ("NutScrew0", "ActNut"),
                                          ("NutScrew1", "ActNut"),
                                          ("EarPin", "Carriage"),
                                          ("EarPin", "SpringCarrier"),
                                          ("ScrewNut", "ActNut"),
                                          ("ScrewNut", "ScrewShaft"),
                                          ("ScrewNut", "SpringUp"),
                                          ("ScrewNut", "SpringDown"),
                                          ("SpringCarrier", "Carriage"),
                                          ("SpringCarrier", "GuideRod"),
                                          ("SpringCarrier", "SpringUp"),
                                          ("SpringCarrier", "SpringDown"),
                                          ("ActNut", "SpringUp"),
                                          ("ActNut", "SpringDown"),
                                          ("ActNut", "SpringCarrier"),
                                          ("SpringUp", "GuideRod"),
                                          ("SpringDown", "GuideRod"),
                                          ("ActNut", "Carriage"),
                                          ("ActNut", "ScrewShaft"),
                                          ("ActNut", "GuideRod"),
                                          ("GuideRod", "ScrewTop"),
                                          ("GuideRod", "GuideFoot"),
                                          ("GuideRod", "Wall"),
                                          ("ScrewShaft", "ServoBody"),
                                          ("ScrewShaft", "ScrewTop"),
                                          ("ScrewShaft", "Wall"),
                                          ("ScrewShaft", "DeckBottom"))}
    _ACT_AGAINST = ("ServoBody", "ServoBracket", "ScrewShaft", "ScrewTop",
                    "Coupler", "ServoHorn", "LockCollar", "ThrustWasherHi",
                    "GuideRod", "GuideFoot",
                    "FramePostT", "FramePostB", "Wall", "DeckTop", "DeckBottom")
    _act_gap, _act_who = 1e9, "-"
    _act_tight = {}
    _fixed_act = {n: s for n, s, c, t in FIXED_PARTS if n in _ACT_AGAINST}
    # The decks are shared, not in FIXED_PARTS: that is how the horn got to
    # 0.21 mm under the ceiling unseen.
    _fixed_act.update({n: s for n, s in _SHARED if n in _ACT_AGAINST})
    for _k in range(-4, 5):
        _st_k = pose_state(phi_preload * _k / 4.0)
        _mov = {n: s for n, s, c, t in act_moving_parts(_st_k)
                if not n.startswith("Pin")}
        _mov["Carriage"] = place_carriage(cached(make_carriage), _st_k)
        _pairs = [(a, b) for a in _mov for b in _fixed_act]
        _names = list(_mov)
        _pairs += [(_names[i], _names[j]) for i in range(len(_names))
                   for j in range(i + 1, len(_names))]
        for _a, _b in _pairs:
            if frozenset((_a, _b)) in _ACT_PINNED:
                continue
            # The carriage against the wall and the posts is the four-bar's own
            # business, already swept above; here it only meets the chain.
            if _a == "Carriage" and _b in ("Wall", "FramePostT", "FramePostB"):
                continue
            _sa = _mov[_a]
            _sb = _mov[_b] if _b in _mov else _fixed_act[_b]
            if bb_gap(_sa, _sb) >= max(_act_gap, 1.5):
                continue
            _d = _sa.distToShape(_sb)[0]
            if _d < 1.5:
                _act_tight[(_a, _b)] = min(_d, _act_tight.get((_a, _b), 9.0))
            if _d < _act_gap:
                _act_gap = _d
                _act_who = f"{_a} x {_b} at {math.degrees(_st_k['phi']):+.1f} deg"


    # Every other axis, not just the next one. The four of them share one box.
    def axis_parts(k):
        """Axis k of the pinwheel, part by part.

        All four axes are the SAME, rotated about Z — nothing is turned over any
        more. Each axis' servo sits in its own (+X) corner of the pinwheel, so no
        two want the same one. (Turning alternate axes over was only ever there
        for two servos lying on the same floor.)

        Part by part rather than fused into one solid: fusing forty parts and
        then intersecting the results cost more than every other check put
        together, and the bounding boxes throw out all but a handful of the
        pairs anyway."""
        return [(n, place(sh, 90.0 * k)) for n, sh, c, t in AXIS_PARTS]


    _axis0 = axis_parts(0)
    _adj_overlap = 0.0
    _adj_worst = 0
    for _k in (1, 2, 3):
        _v = 0.0
        for _na, _sa in _axis0:
            for _nb, _sb in axis_parts(_k):
                if not _bb_hit(_sa, _sb):
                    continue
                _v += _sa.common(_sb).Volume
        if _v > _adj_overlap:
            _adj_overlap, _adj_worst = _v, _k

    # X location, MEASURED: nudge a part along X and see where it first hits
    # its neighbour in the chain. Just under shim_gap must clear, just over
    # must hit — both ways, top and bottom. That is the play the shims fill,
    # found in the real solids rather than read back off the parameters.
    def _near_joint(shape, pivot, r=12.0):
        """Just the part of a shape around one pivot, full width in X.

        The nudge test below is a boolean between solids, and on the whole
        carriage against the whole link that is the most expensive check in
        the macro. Cropping both to the joint first is the same test on
        shapes a tenth the size."""
        box = Part.makeBox(200.0, 2 * r, 2 * r,
                           v(-100.0, pivot[0] - r, pivot[1] - r))
        return shape.common(box)


    def _x_stop(mov, fixed_list, d):
        m = mov.copy()
        m.translate(v(d, 0, 0))
        return any(_bb_hit(m, f) and m.common(f).Volume > 1e-6 for f in fixed_list)
    _st0 = {n: sh for n, sh, c, t in moving_parts(pose_state(0.0)) + FIXED_PARTS}
    _x_bad = []
    for _mn, _fn, _piv in (("Carriage", ("LinkT",), B1_0), ("Carriage", ("LinkB",), B2_0),
                           ("LinkT", ("FramePostT",), A1), ("LinkB", ("FramePostB",), A2)):
        _mv = _near_joint(_st0[_mn], _piv)
        _fl = [_near_joint(_st0[n], _piv) for n in _fn]
        for _sg in (1, -1):
            if (_x_stop(_mv, _fl, _sg * (shim_gap - 0.02))
                    or not _x_stop(_mv, _fl, _sg * (shim_gap + 0.02))):
                _x_bad.append(f"{_mn}@{_fn[0]}{'+' if _sg > 0 else '-'}")
    _x_slip = x_play_max / L_line

    # Pins have to GO IN: the four-bar's frame pins slide in along X from
    # outside the foot. Their path, both ways, must be empty.
    # ONE clear way in is enough — the pin only goes in one way. With the
    # foot on the +X side only, that way is from −X.
    _ins_ov = 0.0
    for _zs, _A in ((1, A1), (-1, A2)):
        _fp = _st0["FramePostT" if _zs > 0 else "FramePostB"]
        _xo = link_x + link_t / 2.0
        _ins_ov += min(pin_x(_A, pin_d, _xo, 30.0).common(_fp).Volume,
                       pin_x(_A, pin_d, -(_xo + 30.0), 30.0).common(_fp).Volume)

    # The E-clips, SWEPT: each against everything but its own joint's plates.
    _CLIP_OWN = {}      # no clips: there are no pinned plates left
    _clip_gap_min, _clip_who = (1e9, "-") if _CLIP_OWN else (9.99, "no clips")
    for _k in range(-4, 5):
        _st_k = pose_state(phi_preload * _k / 4.0)
        _all = {n: sh for n, sh, c, t in moving_parts(_st_k) + FIXED_PARTS}
        _all.update({"DeckTop": _deck_top, "DeckBottom": _deck_bot})
        for _j, _own in _CLIP_OWN.items():
            _c = _all["PinClip" + _j]
            for _n, _sh in _all.items():
                if _n in _own or _n.startswith(("Pin", "Shim")):
                    continue
                if bb_gap(_c, _sh) >= _clip_gap_min:
                    continue
                _d = _c.distToShape(_sh)[0]
                if _d < _clip_gap_min:
                    _clip_gap_min = _d
                    _clip_who = f"clip {_j} x {_n} at {math.degrees(_st_k['phi']):+.1f} deg"

    # Output cone tip vs the motor shaft it points at.
    _tip_clr = out_tip_y - motor_shaft_d / 2.0

    # Every shape in the tree is a valid solid. A boolean that half-failed leaves a
    # shape that still draws and still has a volume, so this is not free.
    _invalid = [o.Name for o in doc.Objects if not o.Shape.isValid()]

    # And every part is ONE solid. Fusing shapes that do not touch is silent: the
    # result is valid, has the right volume, draws correctly and is not a part.
    # The carriage was exactly that — housing plus two side plates, fused, with
    # nothing between them (the web that now joins them did not exist).
    _loose = [n for n, sh, c, t in AXIS_PARTS if len(sh.Solids) != 1]
    _loose += [n for n, sh in (("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up),
                               ("MotorRubberLower", _mr_lo), ("MotorRubberUpper", _mr_up),
                               ("DeckTop", _deck_top), ("DeckBottom", _deck_bot))
               if len(sh.Solids) != 1]

    # Fixed parts against fixed parts, as DISTANCES.
    #
    # Everything else here asks "do they overlap". That passes a servo tab
    # 0.07 mm off the carriage arm, a guide rod held by half a bore, a
    # bracket rib grazing a case — three faults in one afternoon, every one
    # of them found by LOOKING at the model, none by the checks. Parts that
    # never move relative to each other still have to be printed, put in
    # past one another and live with the printer's own error, so they get a
    # real gap or they are a designed contact, and the designed ones are
    # listed.
    _TOUCH_OK = {frozenset(q) for q in (
        # welded: printed as one part with their host
        ("Wall", "GuideFoot"),
        # the frame posts' screws: through the deck, into the post
        ("DeckTop", "FrameScrewT0"), ("DeckTop", "FrameScrewT1"),
        ("DeckBottom", "FrameScrewB0"), ("DeckBottom", "FrameScrewB1"),
        ("FramePostT", "FrameScrewT0"), ("FramePostT", "FrameScrewT1"),
        ("FramePostB", "FrameScrewB0"), ("FramePostB", "FrameScrewB1"),
        ("DeckTop", "FramePostT"), ("DeckBottom", "FramePostB"),
        # a screw in its own boss
        ("Wall", "WallScrew0"), ("Wall", "WallScrew1"),
        ("Wall", "WallScrew2"), ("Wall", "WallScrew3"),
        # screwed on, so they sit on the wall's own bosses, and their
        # screws pass through both
        ("Wall", "FramePostT"), ("Wall", "FramePostB"),
        ("FramePostT", "WallScrew0"), ("FramePostT", "WallScrew1"),
        ("FramePostT", "WallScrew2"), ("FramePostT", "WallScrew3"),
        ("FramePostB", "WallScrew0"), ("FramePostB", "WallScrew1"),
        ("FramePostB", "WallScrew2"), ("FramePostB", "WallScrew3"),
        ("Wall", "ServoBracket"), ("DeckTop", "ScrewTop"),
        # the cradle's posts stand on the floor, as the case does
        ("DeckBottom", "ServoBracket"),
        # assembled: pressed, seated, screwed or bolted together
        ("ServoBody", "ServoBracket"), ("ServoBody", "ScrewShaft"),
        # the coupler stack: horn on the spline, keyed into the coupler, the
        # horn screw through both, the screw's D end running in the bore
        ("ServoHorn", "ServoBody"), ("ServoHorn", "Coupler"),
        ("HornScrew", "ServoHorn"), ("HornScrew", "Coupler"),
        ("HornScrew", "ServoBody"), ("Coupler", "ServoBody"),
        ("Coupler", "ScrewShaft"),
        ("CouplerGrub", "Coupler"), ("CouplerGrub", "ScrewShaft"),
        # the thrust stack: coupler | washer | plate | washer | collar
        ("ThrustWasherLo", "Coupler"), ("ThrustWasherLo", "GuideFoot"),
        ("ThrustWasherLo", "Wall"), ("ThrustWasherLo", "ScrewShaft"),
        ("ThrustWasherHi", "LockCollar"), ("ThrustWasherHi", "GuideFoot"),
        ("ThrustWasherHi", "Wall"), ("ThrustWasherHi", "ScrewShaft"),
        ("LockCollar", "ScrewShaft"), ("GuideFoot", "ScrewShaft"),
        # a bracket IS its host, so anything it holds touches that host
        ("ServoBody", "DeckBottom"), ("ServoBody", "Wall"), ("GuideRod", "Wall"),
        ("GuideRod", "DeckTop"), ("ScrewShaft", "DeckTop"),
        ("ScrewNut", "DeckTop"), ("ScrewShaft", "Wall"),
        ("ScrewShaft", "ScrewTop"), ("GuideRod", "GuideFoot"),
        ("GuideRod", "ScrewTop"), ("OutputShaft", "UJFork"),
        ("OutputShaft", "BearingOutput"), ("OutputShaft", "BearingOutIn"),
        ("Wall", "BearingOutput"), ("Wall", "BearingOutIn"),
        ("Wall", "OutputShaft"), ("DeckTop", "Wall"), ("DeckBottom", "Wall"),
        ("JawMale", "OutputShaft"),
        ("Wall", "WallMagnet0"), ("Wall", "WallMagnet1"),
        ("Wall", "WallMagnet2"), ("Wall", "WallMagnet3"),
    )}
    _fx = [(n, sh) for n, sh, c, t in FIXED_PARTS] + [
        ("DeckTop", _deck_top), ("DeckBottom", _deck_bot),
        ("MotorConeLower", _mc_lo), ("MotorConeUpper", _mc_up)]
    _fix_gap, _fix_who = 1e9, "-"
    for _i in range(len(_fx)):
        for _j in range(_i + 1, len(_fx)):
            _na, _sa = _fx[_i]
            _nb, _sb = _fx[_j]
            if frozenset((_na, _nb)) in _TOUCH_OK:
                continue
            if bb_gap(_sa, _sb) >= _fix_gap:
                continue
            _d = _sa.distToShape(_sb)[0]
            if _d < _fix_gap:
                _fix_gap, _fix_who = _d, f"{_na} x {_nb}"

    # Brackets printed as part of their host: each has to really MERGE with it
    # (share bracket_weld's worth of material), not just touch its face — two
    # solids meeting on a face fuse into something that draws fine and is not
    # a part. The host being ONE solid afterwards is the check above.
    _weld_min, _weld_who = 1e9, "-"
    _all_named = {n: sh for n, sh, c, t in AXIS_PARTS}
    _all_named.update({"DeckTop": _deck_top, "DeckBottom": _deck_bot})
    for _pair in _WELDED:
        _hn, _bn = sorted(_pair, key=lambda n: n.startswith(("Wall", "Deck")),
                          reverse=True)
        _vol = _all_named[_hn].common(_all_named[_bn]).Volume
        if _vol < _weld_min:
            _weld_min, _weld_who = _vol, f"{_bn} into {_hn}"

    checks = [
        # Informational, and NOT below 1: at 1:1 the cube is an overdrive, the
        # friction stage alone. The reduction invariant 1 demands lives between
        # hubs now, and it has to beat THIS number, not 0.5.
        ("cube ratio w_out/w_motor  (reduction moved between hubs)",
         ratio_total, "info", True),
        ("free gap angle phi_c = 90 - alpha - beta  (deg)",
         math.degrees(phi_c), "> 0", phi_c > 0),
        ("link axes converge on the apex  (deg between the two rays)",
         link_axis_err, "< 0.2", link_axis_err < 0.2),
        ("link axis outside the cone wedge  (beta..90-alpha = "
         f"{beta_deg:.0f}..{90-alpha_deg:.0f} deg)",
         link_axis_deg, f"not in {beta_deg:.0f}..{90-alpha_deg:.0f}",
         not (beta_deg <= link_axis_deg <= 90 - alpha_deg)),
        ("neck bearing clear of the cone's rim  (mm)",
         nb_y0 - out_base_y, "> 1.0", nb_y0 - out_base_y > 1.0),
        ("output cone tip clears the motor shaft  (mm)",
         _tip_clr, "> 1.0", _tip_clr > 1.0),
        (f"central column parts clear each other [{_col_who}]  (mm3)",
         _col_ov, "== 0", _col_ov < 1e-6),
        ("actuation chain assembles at every tilt of the stroke  (poses failed)",
         _act_fail, "== 0", _act_fail == 0),
        # The screw's own two questions: does the servo have the turn for
        # the travel, and is there force enough at the end of it.
        # With a screw most of the servo's turn goes into closing the free
        # gap, because the ear is 40 mm from the apex and the gap is 2 deg.
        # What is left is the spring's whole working stroke.
        ("spring stroke left after contact  (mm)",
         _spring_stroke, "> 0.30", _spring_stroke > 0.30),
        ("force at the ear from the screw  (N, the crank chain gave 34)",
         _F_screw, "> 30", _F_screw > 30.0),
        ("ear wanders off the nut's straight line  (mm, the pin's slot)",
         _ear_sideways, "< 1.20", _ear_sideways < 1.20),
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
        # Drawn in one phase only: the prism is aligned with the tube there. Half
        # a turn later it rocks inside it about the Z pins, and its far corner is
        # what comes closest to the tube's bore.
        # Same for the ring inside the tube's head: half a turn on, it rocks about
        # its Z pins by the ring joint's own bend.
        ("ring cross inside the tube's head, any phase  (mm)",
         uj_head_ri - (uj_ring_ro * math.cos(uj_bend_ring)
                       + uj_ring_t / 2.0 * math.sin(uj_bend_ring)),
         "> 0.3",
         uj_head_ri - (uj_ring_ro * math.cos(uj_bend_ring)
                       + uj_ring_t / 2.0 * math.sin(uj_bend_ring)) > 0.3),
        ("solid cross prism inside the intermediate, any phase  (mm)",
         uj_mid_ri - math.sqrt(uj_cross_a ** 2 + uj_cross_hy ** 2 + uj_cross_hz ** 2),
         "> 0.5",
         uj_mid_ri - math.sqrt(uj_cross_a ** 2 + uj_cross_hy ** 2 + uj_cross_hz ** 2) > 0.5),
        (f"cardan running clearance, SWEPT [{_uj_who}]  (mm)",
         _uj_gap, "> 0.5", _uj_gap > 0.5),
        (f"actuation chain running clearance, SWEPT [{_act_who}]  (mm)",
         _act_gap, "> 0.5", _act_gap > 0.5),
        ("cardan intermediate length change, within its slotted holes  (mm)",
         uj_L_range[1] - uj_L_range[0], f"< {uj_slide:.1f}",
         uj_L_range[1] - uj_L_range[0] < uj_slide),
        (f"carriage X play is the shims' to fill: stops at shim_gap"
         f" {shim_gap:.1f} each way  {_x_bad if _x_bad else ''}",
         len(_x_bad), "== 0", not _x_bad),
        ("four-bar frame pins can be pushed in from outside the foot  (mm3)",
         _ins_ov, "== 0", _ins_ov < 1e-6),
        (f"E-clip running clearance, SWEPT [{_clip_who}]  (mm)",
         _clip_gap_min, "> 0.5", _clip_gap_min > 0.5),
        (f"slip from {x_play_max:.2f} mm of X play left after shimming  (%)",
         _x_slip * 100, f"<= four-bar's {slip_fourbar*100:.2f}",
         _x_slip <= slip_fourbar),
        (f"fixed parts clear each other [{_fix_who}]  (mm)",
         _fix_gap, "> 0.5", _fix_gap > 0.5),
        (f"brackets really merge into their host [worst: {_weld_who}]  (mm3)",
         _weld_min, "> 1", _weld_min > 1.0),
        (f"every built shape is a valid solid  {_invalid if _invalid else ''}",
         len(_invalid), "== 0", not _invalid),
        (f"every part is ONE connected solid  {_loose if _loose else ''}",
         len(_loose), "== 0", not _loose),
        (f"every other axis, all identical and rotated [worst: {_adj_worst*90}"
         f" deg]  (mm3)",
         _adj_overlap, "== 0", _adj_overlap < 1e-6),
    ]

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
          f"   servo {sv_theta_c:.0f} deg")
    print(f"    preload phi = {math.degrees(phi_preload):.3f} deg"
          f"   servo {sv_theta_max:.0f} deg, split 1:{spring_split-1:.1f}")
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
    print("  DRIVE OUT: folded double cardan inside the cone, 1:1")
    print(f"    solid cross y {uj_cross_y:.1f} (output axis), ring y {uj_ring_y:.1f}"
          f" (cone axis), intermediate {uj_L_rest:.1f} mm, tube"
          f" \u00d8{2*uj_mid_ri:.1f}/{2*uj_mid_ro:.1f} (head \u00d8{2*uj_head_ri:.1f}/{2*uj_head_ro:.1f}),"
          f" ring \u00d8{2*uj_ring_ri:.1f}/{2*uj_ring_ro:.1f} x {uj_ring_t:.0f}")
    print(f"    bends at full preload: solid cross {math.degrees(uj_bend_cross):.2f} deg,"
          f" ring {math.degrees(uj_bend_ring):.2f} deg  (never equal: both crosses are"
          f" in front of the apex)")
    _hooke = (uj_bend_cross ** 2 - uj_bend_ring ** 2) / 4.0
    print(f"    residual angle error ~ (a1^2 - a2^2)/4 = {math.degrees(_hooke):.3f} deg"
          f"  (one Hooke joint at the tilt would be"
          f" {math.degrees(phi_preload**2/4):.3f})")
    print(f"    intermediate length {uj_L_range[0]:.3f}..{uj_L_range[1]:.3f} mm over the"
          f" stroke -> ring-end pin holes slotted {uj_slide:.1f} mm")
    print(f"    cone runs on one 6805 ({nb_id:.0f}x{nb_od:.0f}x{nb_w:.0f}) on its neck,"
          f" y {nb_y0:.1f}..{nb_y0+nb_w:.1f}, held by the carriage ring")
    print(f"    output shaft: 2x MR105ZZ, inner boss y {cube_half-brg_w:.1f}..{cube_half:.1f}"
          f" and wall seat; reaches {cube_half - brg_w - (uj_cross_y + uj_fork_back):.1f}"
          f" mm in from the inner bearing to the fork")
    print("-" * 72)
    print("  ACTUATION: servo -> coupler -> T8 lead screw -> nut -> ear"
          "   (no linkage at all)")
    print("  (spring and forces are placeholders until the rubber stiffness is measured)")
    print(f"    screw at (x {screw_x:.1f}, y {screw_y:.1f}), lead {screw_lead:.1f} mm/turn,"
          f" ear at (y {E0[0]:.1f}, z {E0[1]:.1f})")
    print(f"    nut travel {abs(_ear_travel):.2f} mm over the stroke;"
          f" the servo's own {2*sv_theta_max:.0f} deg would give"
          f" {2*sv_theta_max/360.0*screw_lead:.2f}")
    print(f"    force at the ear: screw + thrust washer friction = {_F_screw:.0f} N"
          f"  (the crank chain gave 34)")
    print(f"    servo turn: {_horn_contact:.1f} deg to contact, {_horn_preload:.1f} deg"
          f" at full preload, of {sv_theta_max:.0f} available — the rest"
          f" compresses the spring")
    _tau = sv_use * sv_stall
    _k_spring = _tau / math.radians(sv_theta_max - sv_theta_c)
    _k_lin = _F_screw / ((sv_theta_max - sv_theta_c) / 360.0 * screw_lead)
    _F_ear = _F_screw
    _M = _F_ear * E0[0] / 1000.0
    _mu_ref = 1.3
    print(f"    spring: COMPRESSION now, {_k_lin:.1f} N/mm, reaching {_F_screw:.0f} N"
          f" after {(sv_theta_max - sv_theta_c)/360.0*screw_lead:.2f} mm of squeeze"
          f"  (a catalogue part, unlike the torsion spring)")
    _sq_txt = (f" normal force {_M*1000/_sq_sbar:.0f} N (s_bar {_sq_sbar:.1f} mm)"
               if _sq_sbar == _sq_sbar else
               "  (no squeeze volume at this stop: the spring takes almost all"
               " of the travel past contact, so the carriage barely creeps)")
    print(f"    at full preload: {_F_ear:.0f} N on the ear, {_M:.2f} Nm about the apex,"
          + _sq_txt)
    print(f"    output torque ceiling: mu*sin(beta)*M = {_mu_ref*math.sin(beta)*_M:.2f} Nm"
          f" at mu = {_mu_ref} — IF the rubber takes that force within its squeeze")
    for (_a, _b), _d in sorted(_act_tight.items(), key=lambda kv: kv[1]):
        print(f"    chain clearance under 1.5 mm, worst over the stroke: {_a} x {_b}  {_d:.2f} mm")
    print("-" * 72)
    for label, value, target, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {label}:  {value:.3f}  ({target})")
    print("-" * 72)
    _cbb = BY_NAME["Carriage"].BoundBox
    print(f"  Carriage: ONE piece, {_cbb.YLength:.0f} x {_cbb.ZLength:.0f} x"
          f" {_cbb.XLength:.0f} mm, a block round the cone's neck")
    _cube_tied = [k for k, val in cube_half_by.items() if val > cube_half - 0.05]
    print(f"  Cube half-size ({cube_half:.1f}) = motor axis to the wall's inner"
          f" face, set by: {' AND '.join(_cube_tied)}")
    for _k, _val in sorted(cube_half_by.items(), key=lambda kv: -kv[1]):
        print(f"      {_val:5.1f}  {_k}")
    print(f"  CUBE side {2 * cube_out:.1f} mm"
          f"  (half {cube_half:.1f} inside + {wall_thick:.1f} wall),"
          f" HEIGHT {2 * (cube_half + deck_t):.1f} mm (decks {deck_t:.0f})")
    print("  PRINTED: MotorCone x2 (same part, flipped), the shaft's spacer, OutputCone (a SHELL),")
    print("           Carriage (one piece),")
    print("           UJMid, UJRing, UJCross, UJFork,")
    print("           Link x2 (each carries both its arms), the nut's pusher")
    print("           WITH THE WALL: the two frame posts, the servo's bracket,")
    print("           the screw's top bearing and the guide rod's foot.")
    print("  ASSEMBLY: all four axes identical, rotated about the motor; each servo in")
    print("            its own corner under the ceiling.")
    print(f"  The cardan's fork is keyed on the output shaft by a D on a filed flat"
          f" ({shaft_flat_d:.1f} mm across);")
    print("  Filing the flat is the one manual step here.")
    print("  PURCHASED, per axis: 1x 6805 (cone), 2x MR105ZZ (output shaft),")
    print("             Ø5 rod (output shaft),")
    print(f"             SHARED: Ø8 ground rod ~{2 * (cube_half + deck_t + 15):.0f} mm"
          f" (motor shaft), 2x 608ZZ, 1x Ø8 shaft collar, 2x washers 8x12x1,")
    print("             2x rigid flange couplings Ø8 (+ 8x M3x10), for the motor cones,")
    print("             PER CUBE: jaw couplings D20 — 1 half bore 8 + spider (input),")
    print("             1 half bore 8 (top output), 4 halves bore 5 (wall outputs);")
    print("             24 disc magnets N52 10x3 (4 per face, 6 faces).")
    print("             Ø4 pin stock (4 pivot pins), Ø2 pin stock (8 cross pins),")
    print(f"             T8 lead screw, lead {screw_lead:.0f} (about"
          f" {screw_top_z - (-cube_half + servo_body[0]):.0f} mm of it) + its nut,")
    print(f"             Ø{guide_d:.0f} rod for the guide, Ø3 pin stock (1 ear pin),")
    print(f"             8 shim washers 4x8 (0.1-0.5) for the four-bar's thrust faces,")
    print(f"             1 compression spring ~{_k_lin:.0f} N/mm,"
          f" 1 T8 lock collar, 2 PTFE washers 8x12x1, 1 M3 grub,")
    print("             2x M2x6 for the servo tabs, rubber sheet, servo.")
    print(f"  FDM holes (this printer runs ~0.5 under): shaft Ø{fdm_shaft_hole_d:.1f}"
          f"  pin Ø{fdm_pin_hole_d:.1f}  bearing seat Ø{brg_od + 2*brg_fit_press:.1f}")
    print("=" * 72)
    print("Packaging (carriage arms, frame brackets, servo mount, cardan) is")
    print("a first pass: the geometry above is derived, the brackets are not.")

else:
    print("=" * 72)
    print(f"Motcore v6 — Apex Pivot: geometry only, CHECKS SKIPPED (RUN_CHECKS)")
    print(f"  cube side {2 * cube_out:.1f} mm, set by {cube_half_driver}")
    print("  Run headless, or set RUN_CHECKS = True, before trusting a change.")
    print("=" * 72)
