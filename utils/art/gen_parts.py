#!/usr/bin/env python3
"""Generate res/data/parts.json from the part meshes + physical constants.
Size (radius/height/volume) comes from the mesh; behavior is derived from that
geometry. Mass is DRY structure only -- propellant rides `capacity` + effectiveMass.
Source of truth for the committed JSON; --check verifies no drift."""

import argparse
import json
import math
import os
import re

import trimesh

from gen_out import REPO_ROOT   # the one spelling (utils/art is two levels down)

# --- physical constants (SI) ------------------------------------------------
EXHAUST_VELOCITY = 4400.0        # m/s, H2/LOX vacuum (Isp = 4400/9.81 ~ 449 s)
PROP_DENSITY = 133.0             # kg/m^3, 50/50 LH2 + LOX mixture by mass
TANK_DRY_DENSITY = 13.3          # kg/m^3, structural wall mass per tank volume
ENGINE_THRUST_PER_M2 = 50000.0   # N, thrust at radius = 1 m (scales with r^2)
ENGINE_MASS_PER_N = 0.01         # kg per newton of thrust (~100 N/kg)
# TVC (#415): a wheel-less rocket steers on thrust alone. The six hydrogen
# engines (chemical + orbital) gimbal +/-5 deg at 90 deg/s; the nuclear one
# does not.
ENGINE_GIMBAL_MAX = 0.0873       # rad (+/-5 deg)
ENGINE_GIMBAL_RATE = 1.5708      # rad/s (90 deg/s)
# Nuclear thermal: H2 only (no oxidizer), higher Isp, lower thrust, heavier.
NUCLEAR_THRUST_PER_M2 = 30000.0  # N, thrust at radius = 1 m (scales with r^2)
NUCLEAR_EXHAUST_VELOCITY = 9000.0  # m/s (Isp = 9000/9.81 ~ 917 s), ~2x chemical
NUCLEAR_MASS_PER_N = 0.03         # kg per newton -- reactor + shielding, ~3x chemical
HYDRAZINE_DENSITY = 100.0        # kg/m^3, monopropellant hydrazine (mono)
# RCS rings are authored, not scaled: a part declares its per-bell rating and its
# bells. A size-scaled rating implies bells on faces the part's own mounts
# occupy (#1120 removed the round inline pods that came with exactly that
# fiction). Reach is rcsReachableAlong() in src/rcs.h, never the installed figure.

# Jet (air-breathing): T = T_fan + m_f*V_E + rho*A*v*(V_E - v) (src/drag.h jetThrust)
JET_FAN_PER_M2    = 90000.0      # N, static (fan) thrust at radius = 1 m
JET_EXH_VEL       = 550.0        # m/s, the real exhaust velocity (not a knob)
JET_INTAKE_PER_M2 = 0.35         # m^2, effective intake area at radius = 1 m
JET_FUEL_PER_M2   = 1.2          # kg/s, jet-fuel flow at radius = 1 m (~0.47 lb/lbf/h)
JET_MASS_PER_M2   = 700.0        # kg, engine mass at radius = 1 m (~129 N/kg)

JET_FUEL_DENSITY = 70.0          # kg/m^3, jet fuel (separate resource from H2/LOX)

MASS_DENSITY = {
    "capsule":        183.0,
    "fuselage":       25.0,
    "reaction_wheel": 128.0,
    "adapter":        15.0,
    # A cap is a closed shell -- fuselage's structure, with the far end buttoned
    # up. It was 192.0, denser than a crew capsule, which put a 1.125 m tip at
    # 3435 kg: more than the 2531 kg engine it sits on, and 13.7 t of solid iron
    # on a stack that carries four of them (#951). A cap that houses a chute bay
    # should say so the way `chute` does -- a declared mass, not a density.
    "nose_cap":       25.0,
    "cargo":          40.0,
    # A lifting body is thin structure -- cargo's shell density suits it.
    "liftbody":       40.0,
    "materials_pod":  60.0,
}

# A crew pod is more than the shell its mesh encloses: seat, harness, instrument panel,
# life support, hatch hardware and the drums inside it scale with the SEATS, not with the
# volume around them. #1171's volume basis priced the skiff's pod at 93.72 kg -- less
# than the 97.05 kg kerbal in it -- and the sample glider's balance point walked aft:
# static margin +0.55 -> +0.31 and its hull bill moved in FRONT of the COM, so under
# throttle the airframe diverges in pitch instead of weathervaning (#1203; hands off,
# even that hull holds 4 deg). Interim: one declared allowance on the pod the shipped
# airframes use, at the mass their aero bands were pinned against.
# READ THE NUMBER AS WHAT IT IS: 93.72 + 121.87 = 215.59 is exactly what
# capsule_r0.5h1.5 cost BEFORE #1171, and 121.87 = (pi r^2 h - V_mesh) x 183. So this
# restores a mass, it is not a measured bill for a seat and a panel -- it is the same
# bounding-cylinder interior #1171 took away, billed by name instead of by geometry.
# It also makes this the cheapest seat in the family (215.59/1 against capsule's 499.82/1
# and capsule_r2.25h4.5's 5693.28/6), which is the opposite of what an outfit allowance
# is for. The law -- per seat, acting at the seat, without double-counting the occupant,
# across all five pods -- is #1204.
POD_OUTFIT_KG = {
    "capsule_r0.5h1.5": 121.87,
}

CAPSULE_TORQUE_PER_M = 200.0
WHEEL_TORQUE_PER_M = 2000.0

# electrical (KSP-style EC): watts / watt-hours
# An electric actuator bills by the authority it can bring about, not by the housing
# it came in (#596): 0.5 W per N m of "torque" covers a wheel block (r1: 2000 N m ->
# 1000 W) and a capsule's built-in drums (r1: 200 N m -> 100 W) with one rule. The
# figure is per HOUSING, not per drum -- a #593 triad bills once (#593 left that
# convention alone; changing it would triple every shipped wheel's bill).
# What that figure MEANS, now that #648 bills by the delivered setpoint rather than by
# the switch being on: it is the housing's PEAK -- what the package costs with its
# busiest drum at full demand. So a triad steering one line at full throttle costs the
# figure above, one asking a quarter of its rating costs a quarter, and one idle costs
# nothing. Read it as per-drum instead and every shipped wheel's cost triples at full
# three-axis use, which is the choice #593 declined to make; the ambiguity was implicit
# there and is explicit here.
WHEEL_DRAW_WATTS_PER_NM = 0.5
RTG_WATTS_PER_M3       = 380.0    # W per m^3 (r1 -> ~300 W)
RTG_DENSITY            = 150.0    # kg/m^3, fuel + thermoelectrics + housing
BATTERY_ACTIVE_DENSITY = 1000.0   # kg/m^3, Li-ion pack bulk density
BATTERY_WATTS_PER_KG   = 200.0    # Wh/kg, modern space Li-ion (per kg of cells)
BATTERY_DRY_DENSITY    = 100.0    # kg/m^3, hull + BMS + wiring overhead
CAPSULE_LIFE_SUPPORT_W_PER_CREW = 100.0   # W per crew, constant (base capsule = 100 W)
CAPSULE_BATTERY_WH_PER_CREW     = 2000.0  # Wh per crew (~20 laptop batteries; base = 2 kWh)

WING_DENSITY       = 50.0     # kg/m^3, wing structure (skin + spars)
WING_CL            = 6.0      # lift-curve slope (per radian)
WING_STALL_ANGLE   = 0.35     # rad (~20 deg), where lift peaks + the flow stalls

RUDDER_DENSITY        = 50.0  # kg/m^3, control-surface structure (skin + spars)
RUDDER_CL             = 6.0   # deflection effectiveness (per radian)
RUDDER_MAX_DEFLECTION = 0.35  # rad (~20 deg), the travel limit
# Where a DEFLECTED surface stops biting. The loader reads the same "stall_angle"
# field for both jobs: incidence stall on a lifting part, deflection stall on a
# steering part (#494). A plain plate hinged into the flow peaks near 15 deg.
# It goes to nothing at 2x that, and max_deflection is below the zero, so the
# pedal never runs backwards on us.
CONTROL_STALL_ANGLE   = 0.26  # rad (~15 deg)
# Fraction of a control surface's footprint that actually becomes authority.
# Not a physical constant: it is the knob that keeps a fin's moment near the
# ship's own wheel budget (#399 asks for that ratio to come from the sim).
CONTROL_AREA_FRACTION = 0.5

# Per-type aerodynamic shape coefficients (src/drag.h partCd).
# drag = symmetric default; drag_forward/side/backward by how the part faces
# the flow (nose +Z / broadside / base -Z). Keyed by TYPE (shape family).

# The thin disc: reaction wheel, battery, rtg, mono tank, decoupler, dock port,
# materials pod, barometer/thermometer stick, the small rcs puck. Ten types, one
# shape -- a ~2.0 x 2.0 x 0.25 m puck -- so one dict, and the numbers were
# measured on reaction_wheel's own mesh (tmp/n1099/dump_verts.py, faceprobe2).
#
# #1208: drag_side used to be 0.2, which is what a THIN plate is worth edge-on.
# That figure is small only because a thin plate's shadow edge-on is a hairline
# (thickness x span), and the silhouette law already does that part for us --
# it bills cdSide over the part's ACTUAL projected area, and this puck's broadside
# shadow is 0.50 m2 of blunt wall. A blunt body at zero incidence pays roughly one
# dynamic pressure of base suction behind it, so its own shadow is worth cd ~0.9;
# face-on stays 1.1. Newtonian on the same mesh says 1.331 broadside, and #635
# gives that band its twin in DRAG_CD_SUP below -- the pair has to be decided
# together, because #947's crossflow reads the same anchor at every Mach.
DISC_CD = {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.9,
           "drag_backward": 1.1}

DRAG_CD = {
    "capsule":        {"drag": 1.0, "drag_forward": 0.35, "drag_side": 0.8,
                       "drag_backward": 1.3},
    # the one rcs part overrides this family by name -- a cube is not a disc
    "reaction_wheel": DISC_CD,
    "materials_pod":  DISC_CD,
    "barometer":      DISC_CD,
    # Same stick, same shape -- the thermometer reuses the barometer's mesh.
    "thermometer":    DISC_CD,
    "battery":        DISC_CD,
    "rtg":            DISC_CD,
    "mono_tank":      DISC_CD,
    "rcs":            DISC_CD,
    "decoupler":      DISC_CD,
    "docking_port":   DISC_CD,
    # blunt cylinder (tank / crate)
    "fuel_tank":      {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    "jet_tank":       {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    "cargo":          {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    # #1097: the flat lifting-body hull. Edge-on is the low-drag case (the chord
    # runs with the flow); broadside is a flat plate at rest, so cdSide IS the
    # plate's normal-force coefficient: 1.0 x the ship's drag_cd 1.2 = 1.20, inside
    # the measured 1.17-1.28. #1107 fitted that against body_lift as a PAIR -- the
    # two anchors push the same hull 90 deg apart, so what has to land right is the
    # pair's best glide, body_lift / (2 sqrt(cdSide x cdForward)) = 1.03, a flat
    # plate's ~1:1. tests/test_aerosweep.cpp sweeps the shipped def and pins it.
    # forward 0.1 / backward 0.5 is not a claim that a symmetric plate flies better
    # one way -- it is the catalog's bluffer-base convention (fuselage 0.10/0.20:
    # the lee face sits in the wake). But forward is HALF of the glide equation: that
    # closed form is body_lift / (2 sqrt(cdSide x cdForward)), so 0.65 was SOLVED
    # FROM 0.1. Streamline this def -- forward 0.05 buys the same plate a 41% better
    # glide -- and body_lift has to be re-solved with it, not left behind.
    "liftbody":       {"drag": 0.4, "drag_forward": 0.1, "drag_side": 1.0,
                       "drag_backward": 0.5},
    "engine":         {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "orbital_engine": {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "nuclear_engine": {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    # A shrouded duct: nose-first is a nacelle's 0.15. The BACK anchor stays blunt,
    # which is what a rocket ascent sees.
    "jet":            {"drag": 0.6, "drag_forward": 0.15, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "adapter":        {"drag": 0.2, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    # AEROPLANE FUSELAGE (gen_fuselage.py): one smooth body at Cd 0.10 nose-first,
    # replacing a capsule (0.35) + tank (1.0) + shroud (0.9) + wheel (1.1) stack
    # whose area-weighted Cd is 0.84.
    "fuselage":       {"drag": 0.6, "drag_forward": 0.10, "drag_side": 0.9,
                       "drag_backward": 0.20},
    # cone (apex = +Z): tip-first sleek, flat base blunt
    "nose_cap":       {"drag": 0.6, "drag_forward": 0.35, "drag_side": 0.9,
                       "drag_backward": 1.3},
    "kerbal":         {"drag": 0.5, "drag_forward": 0.5, "drag_side": 0.9,
                       "drag_backward": 0.9},
    # thin plate (wing / control surfaces): chord along the part's own +Z, so
    # the flow along its chord is the low-drag case and spanwise is blunt.
    # #455: these four families are LIFTING/STEERING surfaces, so they are out of
    # the hull silhouette union (drag.h::isAeroSurface) and pay their planform x
    # this cd instead -- a PLANFORM-referenced profile cd, not the bluff-body Cd
    # the rest of this table authors. A smooth section is 0.006-0.013 there; 0.03
    # is a faceted plate, and it is what the yardstick airframe's L/D is currently
    # paid in (#490's second half re-derives that number rather than this one).
    # A smooth section. The mesh is a flat plate (a stand-in); 0.010 is the
    # smooth-laminate value this note has quoted since #455. 0.03 was the faceted
    # plate, three times the bill: even smooth, the shipped skiff pays Profile= 0.61 m2
    # of CdA at cruise, which is where the rest of its L/D is hiding (#567's bed).
    "wing":           {"drag": 0.1, "drag_forward": 0.010, "drag_side": 1.0,
                       "drag_backward": 0.010},
    "rudder":         {"drag": 0.1, "drag_forward": 0.010, "drag_side": 1.0,
                       "drag_backward": 0.010},
    "elevator":       {"drag": 0.1, "drag_forward": 0.010, "drag_side": 1.0,
                       "drag_backward": 0.010},
    "aileron":        {"drag": 0.1, "drag_forward": 0.010, "drag_side": 1.0,
                       "drag_backward": 0.010},
    # parachute: stowed it is the nose-cap silhouette, but the deployed canopy
    # dominates the aero pass (the Cd.A figure lives in EXTRA_FIELDS.chute_area,
    # not here), so it gets its own Cd family rather than nose_cap's.
    "chute":          {"drag": 0.9, "drag_forward": 0.6, "drag_side": 0.9,
                       "drag_backward": 1.2},
    # landing gear: a small bluff body tucked under the hull (a wheel + strut).
    "gear":           {"drag": 0.7, "drag_forward": 0.8, "drag_side": 0.9,
                       "drag_backward": 0.9},
    # landing leg: the same bluff-body family (a pad + strut + plate).
    "leg":            {"drag": 0.7, "drag_forward": 0.8, "drag_side": 0.9,
                       "drag_backward": 0.9},
    # EVA ladder: a long thin box bolted to the hull -- bluff broadside,
    # near-edge-on along its own length.
    "ladder":         {"drag": 0.7, "drag_forward": 0.8, "drag_side": 1.0,
                       "drag_backward": 0.8},
}

# Per-part Cd overrides on top of the type's family: rcs_box is a flush cube
# with no slipstream face along its own axis, so it has no low-drag side.
DRAG_CD_BY_NAME = {
    "rcs_box":        {"drag": 0.8, "drag_side": 1.1},
    # The flat-topped capsule (#1170). The cone family's 0.35 forward is what a
    # pointed shoulder buys; this part has two flat discs and a straight wall, so
    # neither end is streamlined. A cylinder is mirror-symmetric about its own
    # midplane, so forward and backward MUST agree -- an asymmetric pair here would
    # be billing a force that reversing the flow cannot produce. Blunt bodies sit
    # near 1.0 against their own silhouette, which is what the area terms use.
    "capsule_cyl_r1h2": {"drag": 1.0, "drag_forward": 1.0, "drag_side": 0.9,
                         "drag_backward": 1.0},
}

# The Cd family those tables write. It is NOT a legal EXTRA_FIELDS key: that
# table is applied before the family, so a drag key there is silently clobbered.
DRAG_KEYS = ("drag", "drag_forward", "drag_side", "drag_backward")

# #635: the same three shape anchors for the SUPERSONIC band. src/drag.h joins the
# two anchor sets across kTransonicMach..kSupersonicMach with regimeMix, on the free-
# stream Mach, once per substep. A family absent here means its cd does NOT change
# across the band -- the C++ accessors answer the subsonic anchor when there is no
# supersonic one (src/shipdef.h dragForwardSup()), never 0. That is the honest claim
# for a small strut of gear or a leg; it is not the claim for a blunt body.
#
# The numbers come from a rule, then a hand per family:
#     sup = max(subsonic, min(newtonian_measured, 1.35))
# newtonian_measured being the Newtonian integral over the part's OWN shipped mesh
# (tmp/n1099/faceprobe2; fwd = the part's +Z end into the flow, broad = perpendicular,
# bwd = -Z), referenced to that mesh's own projected area -- which is what partCd bills
# on. Two corrections to it:
#   - the 1.35 cap. Newtonian gives Cp = 2 on every face normal to the flow and no lee
#     suction behind it, so a flat end reads exactly 2.000. A real flat disc measures
#     ~1.1-1.3, so the axial column wants capping.
#   - the broadside column (1.17-1.33 across every fat body) is the one to take at face
#     value: that is where the surfaces are blunt shoulders, which is exactly what
#     Newtonian is good at.
# What the table is FOR: fuselage nose-on is authored 0.10, a genuine subsonic figure,
# and Newtonian puts the same shape at 0.37-0.50. That gap is the wall a spaceplane has
# to push through; nothing else here moves a subsonic hull at all (regimeMix is 0 below
# Mach 0.75).
#   fwd  / side / bwd
DRAG_CD_SUP = {
    # the nose stays cheap (the shipped pod's pointed end is pointed), the base is blunt
    "capsule":        (0.45, 1.2, 1.5),
    # blunt cylinder, already ~1.0 both ends
    "fuel_tank":      (1.2, 1.3, 1.2),
    "jet_tank":       (1.2, 1.3, 1.2),
    "cargo":          (1.2, 1.3, 1.2),
    # engine bells, nozzles and the adapter cones: blunt both ends, no streamlined side
    "engine":         (1.15, 1.2, 1.15),
    "orbital_engine": (1.15, 1.2, 1.15),
    "nuclear_engine": (1.15, 1.2, 1.15),
    "adapter":        (1.15, 1.2, 1.15),
    # A nacelle is a shrouded duct, and jetThrust's ram term already prices the momentum
    # its intake captures -- so wave cd on the SAME momentum double-counts it. Raise the
    # nose least of any family here.
    "jet":            (0.3, 1.2, 1.1),
    # the disc family: blunt both faces, and since #1208 the broadside is blunt too (a
    # 0.25 m wall is a wall at any Mach), so the whole set sits at Newtonian's capped
    # broadside. rcs_box inherits this; being a cube, it is blunt either way.
    "reaction_wheel": (1.3, 1.3, 1.3),
    "materials_pod":  (1.3, 1.3, 1.3),
    "barometer":      (1.3, 1.3, 1.3),
    "thermometer":    (1.3, 1.3, 1.3),
    "battery":        (1.3, 1.3, 1.3),
    "rtg":            (1.3, 1.3, 1.3),
    "mono_tank":      (1.3, 1.3, 1.3),
    "rcs":            (1.3, 1.3, 1.3),
    "decoupler":      (1.3, 1.3, 1.3),
    "docking_port":   (1.3, 1.3, 1.3),
    # THE WALL. 0.10 nose-first is what a smooth body is worth under Mach 0.8; above the
    # band the same 0.5 x 6 m hull referenced to its own 0.78 m2 disc pays ~4x that.
    "fuselage":       (0.45, 1.2, 0.5),
    # a shallow cone (0.5 m on a 2 m base) is nearly a flat disc apex-first
    "nose_cap":       (0.6, 0.9, 1.5),
    # Deliberately modest: #1107 solved this hull's glide as a PAIR (body_lift 0.65 against
    # drag_side 1.0 / drag_forward 0.1 -> best glide 1.03), and body_lift has no regime law
    # yet (drag.h #1097), so raising only its cd across the band would bend a flight path
    # that nothing else in the pass is bending.
    "liftbody":       (0.25, 1.2, 0.7),
    "kerbal":         (1.0, 1.1, 1.0),
    # Left out on purpose -- chute (its own CdA term dominates), gear, leg, ladder (small
    # area), and every LIFTING/STEERING surface (wing, rudder, elevator, aileron), whose
    # regime cd is its thickness-to-chord through drag.h::waveCd. The loader refuses these
    # keys on a part with lift_area or control_area, so a typo here fails at startup.
}

# A part whose shape is not its type's (the flat-topped capsule has two flat discs,
# not a pointed shoulder).
DRAG_CD_SUP_BY_NAME = {
    "capsule_cyl_r1h2": (1.2, 1.3, 1.2),
}

DRAG_SUP_KEYS = ("drag_forward_sup", "drag_side_sup", "drag_backward_sup")

# --- the catalog: (name, type, mesh, texture). Add a part = add a line. ----
# fuel_link is virtual (mesh/texture = None). Radial sizes 1.0 / 1.5 / 2.25 m.
PARTS = [
    ("capsule",          "capsule",        "meshes/capsule.obj",                  "textures/capsule.png"),
    # The light pod a small aeroplane is built around. At r1 every family member
    # puts 3.1 m2 of disc in the flow and >1 t on the airframe, which caps the
    # whole game's L/D near 2 no matter how much wing you add (#490).
    ("capsule_r0.5h1.5", "capsule",        "meshes/capsule_r0.5h1.5.obj",         "textures/capsule.png"),
    ("capsule_r1.5h3",   "capsule",        "meshes/capsule_r1.5h3.obj",           "textures/capsule.png"),
    ("capsule_r2.25h4.5","capsule",        "meshes/capsule_r2.25h4.5.obj",        "textures/capsule.png"),
    # The flat-topped pod (#1170). Every other capsule is a truncated cone whose top
    # radius is a quarter of its base, so a 2 m docking port (docking_port_r1, radius
    # 1.0) bolted on top sits on a slope with 0.75 m of ring overhanging the
    # shoulder. Same radius and height as `capsule`, straight wall: the port's radius
    # IS the capsule's, so it lands flush and its bore stays on the stack axis.
    # Its own texture, not the cone's: capsule.png is painted face-by-face for the
    # cone's islands, and the tube's wall is a different set of patches entirely. It
    # starts as one flat fill at the cone's dominant colour (41,41,41) -- a canvas to
    # draw on rather than a picture borrowed from a different shape.
    ("capsule_cyl_r1h2", "capsule",        "meshes/capsule_cyl_r1h2.obj",         "textures/capsule_cyl.png"),
    ("fuselage_r0.4h4",  "fuselage",       "meshes/fuselage_r0.4h4.obj",          "textures/capsule.png"),
    ("fuselage_r0.5h3.5","fuselage",       "meshes/fuselage_r0.5h3.5.obj",        "textures/capsule.png"),
    ("fuselage_r0.5h6",  "fuselage",       "meshes/fuselage_r0.5h6.obj",          "textures/capsule.png"),
    ("fuselage_r1h6",    "fuselage",       "meshes/fuselage_r1h6.obj",            "textures/capsule.png"),
    ("reaction_wheel",   "reaction_wheel", "meshes/reaction_wheel_r1h0.25.obj",   "textures/reaction_wheel.png"),
    ("reaction_wheel_r1.5h0.375",  "reaction_wheel", "meshes/reaction_wheel_r1.5h0.375.obj",  "textures/reaction_wheel.png"),
    ("reaction_wheel_r2.25h0.5625","reaction_wheel", "meshes/reaction_wheel_r2.25h0.5625.obj","textures/reaction_wheel.png"),
    ("battery",          "battery",        "meshes/reaction_wheel_r1h0.25.obj",   "textures/reaction_wheel.png"),
    ("battery_r1.5h0.375","battery",       "meshes/reaction_wheel_r1.5h0.375.obj","textures/reaction_wheel.png"),
    ("battery_r2.25h0.5625","battery",     "meshes/reaction_wheel_r2.25h0.5625.obj","textures/reaction_wheel.png"),
    ("rtg",              "rtg",            "meshes/reaction_wheel_r1h0.25.obj",   "textures/reaction_wheel.png"),
    ("rtg_r1.5h0.375",   "rtg",            "meshes/reaction_wheel_r1.5h0.375.obj","textures/reaction_wheel.png"),
    ("rtg_r2.25h0.5625", "rtg",            "meshes/reaction_wheel_r2.25h0.5625.obj","textures/reaction_wheel.png"),
    ("engine",           "engine",         "meshes/engine.obj",                   "textures/engine.png"),
    ("engine_r1.5h3",    "engine",         "meshes/engine_r1.5h3.obj",            "textures/engine.png"),
    ("engine_r2.25h4.5", "engine",         "meshes/engine_r2.25h4.5.obj",         "textures/engine.png"),
    ("orbital_engine",        "orbital_engine", "meshes/orbital_engine.obj",              "textures/engine.png"),
    ("orbital_engine_r1.5h1.5","orbital_engine", "meshes/orbital_engine_r1.5h1.5.obj",     "textures/engine.png"),
    ("orbital_engine_r2.25h2.25","orbital_engine","meshes/orbital_engine_r2.25h2.25.obj", "textures/engine.png"),
    ("nuclear_engine",   "nuclear_engine", "meshes/nuclear_engine.obj",             "textures/engine.png"),
    ("jet",            "jet",            "meshes/engine.obj",                   "textures/jet_engine.png"),
    # A jet small enough for a light aeroplane: 90 kN on a 700 kg engine is a
    # 6x thrust-to-weight on anything that flies (#490).
    ("jet_r0.25h1",    "jet",            "meshes/engine_r0.25h1.obj",           "textures/jet_engine.png"),
    ("jet_tank_r0.4h2",  "jet_tank",     "meshes/tank_r0.4h2.obj",              "textures/fuel_tank.png"),
    ("jet_tank_r1h1",  "jet_tank",       "meshes/tank_r1h1.obj",                "textures/fuel_tank.png"),
    ("jet_tank_r1h3",  "jet_tank",       "meshes/tank_r1h3.obj",                "textures/fuel_tank.png"),
    ("jet_tank_r1h5",  "jet_tank",       "meshes/tank_r1h5.obj",                "textures/fuel_tank.png"),
    ("fuel_tank",        "fuel_tank",      "meshes/fuel_tank.obj",                "textures/fuel_tank.png"),
    ("tank_r1h1",        "fuel_tank",      "meshes/tank_r1h1.obj",                "textures/fuel_tank.png"),
    ("tank_r1h3",        "fuel_tank",      "meshes/tank_r1h3.obj",                "textures/fuel_tank.png"),
    ("tank_r1h5",        "fuel_tank",      "meshes/tank_r1h5.obj",                "textures/fuel_tank.png"),
    ("tank_r1.5h1",      "fuel_tank",      "meshes/tank_r1.5h1.obj",              "textures/fuel_tank.png"),
    ("tank_r1.5h2",      "fuel_tank",      "meshes/tank_r1.5h2.obj",              "textures/fuel_tank.png"),
    ("tank_r1.5h3",      "fuel_tank",      "meshes/tank_r1.5h3.obj",              "textures/fuel_tank.png"),
    ("tank_r1.5h5",      "fuel_tank",      "meshes/tank_r1.5h5.obj",              "textures/fuel_tank.png"),
    ("tank_r2.25h1",     "fuel_tank",      "meshes/tank_r2.25h1.obj",             "textures/fuel_tank.png"),
    ("tank_r2.25h3",     "fuel_tank",      "meshes/tank_r2.25h3.obj",             "textures/fuel_tank.png"),
    ("tank_r2.25h5",     "fuel_tank",      "meshes/tank_r2.25h5.obj",             "textures/fuel_tank.png"),
    ("mono_tank_r1",     "mono_tank",      "meshes/reaction_wheel_r1h0.25.obj",   "textures/reaction_wheel.png"),
    ("mono_tank_r1.5",   "mono_tank",      "meshes/reaction_wheel_r1.5h0.375.obj","textures/reaction_wheel.png"),
    ("mono_tank_r2.25",  "mono_tank",      "meshes/reaction_wheel_r2.25h0.5625.obj","textures/reaction_wheel.png"),
    # rcs_box is the only RCS hardware: a surface-mounted cube with authored
    # bells, so it cannot claim a face its mount occupies (#1120).
    ("rcs_box",          "rcs",            "meshes/rcs_box.obj",                  "textures/rcs_box.png"),
    ("adapter_r1to1.5",  "adapter",        "meshes/adapter_r1to1.5.obj",          "textures/adapter.png"),
    ("adapter_r1to2.25", "adapter",        "meshes/adapter_r1to2.25.obj",         "textures/adapter.png"),
    ("adapter_r1.5to1",  "adapter",        "meshes/adapter_r1.5to1.obj",          "textures/adapter.png"),
    ("adapter_r1.5to2.25","adapter",       "meshes/adapter_r1.5to2.25.obj",       "textures/adapter.png"),
    ("adapter_r2.25to1", "adapter",        "meshes/adapter_r2.25to1.obj",         "textures/adapter.png"),
    ("adapter_r2.25to1.5","adapter",       "meshes/adapter_r2.25to1.5.obj",       "textures/adapter.png"),
    ("decoupler_r1",     "decoupler",      "meshes/decoupler_r1.obj",             "textures/decoupler.png"),
    ("decoupler_r1.5",   "decoupler",      "meshes/decoupler_r1.5.obj",           "textures/decoupler.png"),
    ("decoupler_r2.25",  "decoupler",      "meshes/decoupler_r2.25.obj",          "textures/decoupler.png"),
    ("decoupler_radial", "decoupler",      "meshes/decoupler_radial.obj",         "textures/decoupler.png"),
    ("docking_port_r1",    "docking_port", "meshes/docking_port_r1.obj",          "textures/docking_port.png"),
    ("docking_port_r1.5",  "docking_port", "meshes/docking_port_r1.5.obj",        "textures/docking_port.png"),
    ("docking_port_r2.25", "docking_port", "meshes/docking_port_r2.25.obj",       "textures/docking_port.png"),
    ("nose_cap",         "nose_cap",       "meshes/nose_cap.obj",                 "textures/nose_cap.png"),
    ("nose_cap_r1.5h0.75","nose_cap",      "meshes/nose_cap_r1.5h0.75.obj",       "textures/nose_cap.png"),
    ("nose_cap_r2.25h1.125","nose_cap",    "meshes/nose_cap_r2.25h1.125.obj",     "textures/nose_cap.png"),
    ("kerbal",           "kerbal",         "meshes/kerbal.obj",                   "textures/kerbal.png"),
    ("cargo",            "cargo",          "meshes/fuel_tank.obj",                "textures/fuel_tank.png"),
    ("materials_pod",    "materials_pod",  "meshes/materials_pod.obj",           "textures/materials_pod.png"),
    ("barometer",        "barometer",      "meshes/barometer.obj",                "textures/barometer.png"),
    # #667: the other end of the air. Same stick, same art -- an instrument's
    # identity is its family + what it measures, not its mesh.
    ("thermometer",      "thermometer",    "meshes/barometer.obj",                "textures/barometer.png"),
    ("wing",             "wing",           "meshes/wing_r2h2.obj",                "textures/wing.png"),
    # High-aspect-ratio surfaces. inducedCd charges a part at ITS OWN span/area,
    # so the `wing` default (4 m2 of planform on a 4 m mesh span) pays that AR no
    # matter how a pair of them is arranged; these give an airframe a span worth
    # inducing.
    ("wing_r3h1.8",      "wing",           "meshes/wing_r3h1.8.obj",              "textures/wing.png"),
    # The span a 5-600 kg airframe wants. inducedCd charges (cl*A)^2 / (pi*e*span^2),
    # so span^2 is the only shape term and span is how lift gets bought cheap. Chord
    # buys lift too -- less AoA, a lower stall floor -- but pays profile drag and mass,
    # and moves the bill only by changing the speed and trim the hull settles at
    # (measured, #567: 12 m -> 16 m took the skiff from 0.488 to 0.333 m2 of Induced=
    # and its glide from L/D 7.5 to 8.6, while chord at a fixed span moved the bill UP,
    # 0.276 -> 0.333 at 16 m). The 1.5 m chord also stops the wing being a third of the
    # body again.
    ("wing_r4h1.5",      "wing",           "meshes/wing_r4h1.5.obj",              "textures/wing.png"),
    ("wing_r1h1",        "wing",           "meshes/wing_r1h1.obj",                "textures/wing.png"),
    ("wing_r0.7h1",      "wing",           "meshes/wing_r0.7h1.obj",              "textures/wing.png"),
    ("rudder",           "rudder",         "meshes/fin_r1h2.obj",                 "textures/rudder.png"),
    ("rudder_r0.6h0.8",  "rudder",         "meshes/fin_r0.6h0.8.obj",             "textures/rudder.png"),
    ("elevator",         "elevator",       "meshes/fin_r1h2.obj",                 "textures/elevator.png"),
    ("elevator_r0.8h0.7","elevator",       "meshes/fin_r0.8h0.7.obj",             "textures/elevator.png"),
    ("aileron",          "aileron",        "meshes/fin_r1h2.obj",                 "textures/aileron.png"),
    ("aileron_r0.8h0.7", "aileron",        "meshes/fin_r0.8h0.7.obj",            "textures/aileron.png"),
    # the parachute rides the nose-cap hull; the canopy is an overlay mesh
    ("chute",            "chute",          "meshes/nose_cap.obj",                 "textures/nose_cap.png"),
    # landing gear (utils/art/gen_gear.py): a wheel on a strut, surface-mounted under
    # the hull. The ground law (src/wheel.h) rolls it; one size for v1. gear_nose
    # is the same wheel with steering (the nose wheel); the mains stay fixed.
    ("gear",             "gear",           "meshes/gear.obj",                     "textures/gear.png"),
    ("gear_nose",        "gear",           "meshes/gear.obj",                     "textures/gear.png"),
    # rocket landing leg (utils/art/gen_leg.py): a footpad on a splayed strut with
    # a mount plate at the top, surface-mounted on a tank's side. The ground
    # law (src/wheel.h via the leg loop) stands it; one size for v1, sized for
    # the 2 m engine class (reports/landing-legs2026_11_21).
    ("leg",              "leg",            "meshes/leg.obj",                      "textures/gear.png"),
    # EVA ladder (utils/art/gen_ladder.py): a long box surface-mounted on the hull's
    # side; the kerbal climbs the segment its "ladder" field authors. Three
    # sizes (#1078): 2 m, 1 m and 0.5 m -- all one art family, the length is
    # the size. (0.25 m waits on a pass over the EVA transfer bands, #1093.)
    ("ladder",           "ladder",         "meshes/ladder.obj",                   "textures/ladder.png"),
    ("ladder_1m",        "ladder",         "meshes/ladder_1m.obj",                "textures/ladder.png"),
    ("ladder_0.5m",      "ladder",         "meshes/ladder_0.5m.obj",              "textures/ladder.png"),
    ("fuel_link",        "fuel_link",      None,                           None),
    # The first hull part to opt in to body lift (#1097): a 4 m flat plate,
    # flown by flight/1097-a. Every older family stays at body_lift 0 until the
    # drag-anchor re-fit (their drag_side already prices a crossflow share).
    ("liftbody_4m",      "liftbody",       "meshes/liftbody_4m.obj",              "textures/fuel_tank.png"),
]

# per-part extra fields that do NOT derive from the geometry (crew, declared
# masses, shrouds, flags). decoupler_r2.25/docking_port_r2.25 declare height
# because mesh_geom rounds 0.5625 to 0.562.
EXTRA_FIELDS = {
    "engine":                {"shroud": "meshes/engine_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "engine_r1.5h3":         {"shroud": "meshes/engine_r1.5h3_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "engine_r2.25h4.5":      {"shroud": "meshes/engine_r2.25h4.5_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "orbital_engine":        {"shroud": "meshes/orbital_engine_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "orbital_engine_r1.5h1.5":  {"shroud": "meshes/orbital_engine_r1.5h1.5_shroud.obj",
                                 "shroud_texture": "textures/engine_shroud.png"},
    "orbital_engine_r2.25h2.25":  {"shroud": "meshes/orbital_engine_r2.25h2.25_shroud.obj",
                                   "shroud_texture": "textures/engine_shroud.png"},
    "nuclear_engine":            {"shroud": "meshes/nuclear_engine_shroud.obj",
                                  "shroud_texture": "textures/engine_shroud.png"},
    # the jet reuses the engine mesh, so it gets the engine's shroud too
    "jet":                   {"shroud": "meshes/engine_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "jet_r0.25h1":           {"shroud": "meshes/engine_r0.25h1_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "capsule":           {"crew_capacity": 1, "experiment_storage": "container"},
    "capsule_r0.5h1.5":  {"crew_capacity": 1, "experiment_storage": "container"},
    "capsule_r1.5h3":    {"crew_capacity": 3, "experiment_storage": "container"},
    "capsule_r2.25h4.5": {"crew_capacity": 6, "experiment_storage": "container"},
    # 2 crew, not the cone's 1: the straight wall is what buys it. The cabin a
    # kerbal actually fits in is 6.24 m3 against the cone r1h2's 2.73, and the
    # family's own ladder (1m:1, 2m cone:1, 3m:3, 4.5m:6) puts that volume between
    # the 2 m cone and the 3 m one. Life support and the built-in battery ride this
    # number (CAPSULE_LIFE_SUPPORT_W_PER_CREW / _BATTERY_WH_PER_CREW), so the pod
    # that carries two also drains for two.
    "capsule_cyl_r1h2": {"crew_capacity": 2, "experiment_storage": "container"},
    "fuselage_r0.4h4":   {"crew_capacity": 1, "mass": 130.0},
    "fuselage_r0.5h3.5": {"crew_capacity": 2, "mass": 180.0},
    "fuselage_r0.5h6":   {"crew_capacity": 2, "mass": 220.0},
    "fuselage_r1h6":     {"crew_capacity": 4, "mass": 520.0,
                          "experiment_storage": "container"},
    # kerbal: DRY mass (full-EVA-gear minus the 10 kg RCS hydrazine, a separate
    # capacity). Suit inventory pocket; science courier (1 finding per family).
    "kerbal":            {"mass": 87.05, "capacity": {"hydrazine": 10.0},
                          "inventory_capacity": 3, "experiment_storage": "courier"},
    "cargo":             {"inventory_capacity": 10},
    # #1097: the signed body-lift anchor (drag.h::bodyLiftCdA), fitted AGAINST the
    # drag anchors by #1107 rather than on its own: over cdSide 1.0 / cdForward 0.1
    # the pair's best glide is body_lift / (2 sqrt(cdSide x cdForward)), so 0.65
    # buys 1.03 -- a flat plate's ~1:1 ceiling. The first pass at 0.9 bought 1.36,
    # which is a plate gliding better than a plate. tests/test_aerosweep.cpp pins it.
    "liftbody_4m":       {"body_lift": 0.65},
    "materials_pod":     {"experiment_family": "materials study",
                          "experiment_storage": "instrument"},
    # rcs_box: a flush 0.3 m cube of five fixed bells, surface-mounted. Mass and
    # the per-nozzle rating are DECLARED (like the decouplers'), and the bells
    # are authored -- the -X mount face has none, so it cannot fire into the
    # hull (src/rcs.h, #428). It is the catalog's ONLY RCS hardware: a part whose
    # rating scaled with its size implied bells on faces its mounts occupy
    # (#1120 deleted rcs_r1/.5/2.25, which did exactly that as inline stack parts).
    "rcs_box":           {"mass": 12.0, "rcs_thrust": 60,
                          "rcs_nozzle_dirs": [
                              [1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]},
    # barometer: tiny stick instrument, mass declared (mesh supplies shape only);
    # scores biome-specific science only on the surface (res/data/experiments.json).
    "barometer":         {"mass": 10, "experiment_family": "barometer",
                          "experiment_storage": "instrument"},
    # thermometer: the same stick measuring the other half of the air (#667).
    "thermometer":       {"mass": 10, "experiment_family": "thermometer",
                          "experiment_storage": "instrument"},
    "decoupler_r1":      {"mass": 50, "decoupler": True, "fuel_barrier": True},
    "decoupler_r1.5":    {"mass": 75, "decoupler": True, "fuel_barrier": True},
    "decoupler_r2.25":   {"mass": 110, "decoupler": True, "fuel_barrier": True,
                          "height": 0.5625},
    "decoupler_radial":  {"mass": 40, "decoupler": True, "fuel_barrier": True},
    "docking_port_r1":   {"mass": 50, "docking_port": True, "fuel_barrier": True},
    "docking_port_r1.5": {"mass": 75, "docking_port": True, "fuel_barrier": True},
    "docking_port_r2.25":{"mass": 110, "docking_port": True, "fuel_barrier": True,
                         "height": 0.5625},
    # chute: rides the nose-cap hull. Mass is DECLARED (like the decouplers and
    # the barometer) because a stowed canopy's mass is not hull volume;
    # chute_area is the deployed canopy's Cd.A (m^2, src/drag.h) and the canopy
    # art is an overlay, so neither derives from the mesh.
    "chute":             {"mass": 150.0, "chute_area": 25.0,
                          "chute_mesh": "meshes/chute_canopy.obj",
                          "chute_texture": "textures/chute.png"},
    # gear: mass + suspension declared (one size for v1). wheel_radius is derived
    # from the mesh in generate(). The surface node is the strut top (+Z), so the
    # wheel hangs below the belly it is bolted to; the rolling direction is the
    # part's +Y (src/wheel.h).
    #
    # SPRING-ONLY SUPPORT, spelled out (#1158, wheel.h "RIDE HEIGHT"): the
    # contact face rides (travel - compression) above the ray hit, so
    #     float = travel - W/(n*k)     and     landing stroke = travel - comp
    # i.e. the visible float above the drawn tyre and the stroke left for a
    # touchdown are ONE quantity. Raising the spring makes the float worse (less
    # compression = more unused strut); the only way to have both is a strut that
    # telescopes, so the drawn mesh slides with compression -- an issue, not a
    # tweak here (#1166). What 0.4 -> 0.15 buys: a skiff's tyres sit 0.041..0.077
    # m off the deck instead of 0.29..0.33 (0.092..0.126 at the half-step 0.2).
    # What it costs is the load ceiling, and the load is uneven: the main wheel
    # takes 42% of the 659 kg hull on a third of the wheels, compressing
    # 0.106..0.109 of its 0.15, so a ~38% heavier plane puts THAT wheel on its
    # bump stops -- not the 74% (n*k*travel = 1.15 t) an even share implies. On
    # the stops the hull is held by collision, the scrub/pump regime wheel.h
    # refuses to rest on. e2e 1158-b pins 0.02 m of stroke still free on every
    # wheel, which is what turns that ceiling red.
    "gear":              {"mass": 30.0, "wheel_travel": 0.15, "wheel_spring": 25000.0,
                          "wheel_damp": 3000.0, "wheel_steer": 0.0,
                          "nodes": [{"id": "srf", "pos": [0.0, 0.0, 0.65],
                                     "dir": [0.0, 0.0, 1.0], "surface": True}]},
    "fuel_link":         {"fuel_link": True},
}

# The nose wheel is the same gear with steering authority (deg); the mains are
# fixed. Same mesh, same suspension -- only wheel_steer differs.
EXTRA_FIELDS["gear_nose"] = dict(EXTRA_FIELDS["gear"], wheel_steer=20.0)

# leg: mass + suspension declared (one size for v1). leg_foot is the distance
# from the part origin (footpad centre) to the pad contact face along -Z --
# the wheel_radius analogue for the leg ground loop. The surface node is the
# plate's hull-side face; keep in sync with gen_leg.py's defaults (drop 2.6,
# splay 20 deg, plate-thick 0.08): node x = -(2.6*tan(20) + 0.04) = -0.986.
# travel is the float AND the landing stroke -- see the gear note above
# (#1158). 0.4 -> 0.15 puts the racer_legs footpads 0.024..0.038 m off the
# ground instead of 0.273..0.302 m, at a rest compression of 0.112..0.126 of the
# stroke. The ceiling that trades away: 4 legs at k=60000 hold 36 kN, which an
# even share says is 3.7 t -- but the measured share is uneven (0.112..0.126
# across the four), so the FIRST leg to reach its stops does so at ~3.5 t (the
# racer_legs hull is 2.93 t). A heavier lander needs more legs, not a longer
# strut. Hard landings turned out NOT to be the constraint: measured arrivals of
# 6.0 and 10.8 m/s stay upright at 0.20/0.15/0.13 alike (worst nose.z 0.967), and
# the hull tips over at ~15.7 m/s at every travel, so stroke length is not what
# saves a bad landing. What travel does set is whether the hull rocks on after a
# hop: at 0.2 the same 6 m/s arrival keeps |w| at 0.074 after t=5 s against
# 0.032 at 0.15 (e2e 1158-c). 1158-a/c pin the float and the reserve.
EXTRA_FIELDS["leg"] = {"mass": 40.0, "leg_foot": 0.05, "leg_travel": 0.15,
                       "leg_spring": 60000.0, "leg_damp": 8000.0,
                       "nodes": [{"id": "srf", "pos": [-0.986, 0.0, 2.6],
                                  "dir": [-1.0, 0.0, 0.0], "surface": True}]}

# ladder: mass declared (a climb rail is not hull volume), scaled with length.
# The surface node is the box's hull-side face ANCHORED AT ITS TOP END (#1078):
# matePose places the child's node point at the contact, so the ladder hangs
# down FROM the cursor and can clear an engine stacked under the parent. The
# dir stays -X -- the pose comes from the dir (a +Z dir would stand the ladder
# off a flank wall); clicking a flat bottom cap wants that +Z dir plus a VAB
# surface-node pick, deferred as #1094. Keep in sync with gen_ladder.py's
# defaults (thick 0.1, wide 0.4): node x = -thick/2. top/bottom stack nodes let
# ladders bolt end-to-end exactly; surface attach puts them anywhere.
def ladder_nodes(length):
    hz = length / 2.0
    return [{"id": "srf", "pos": [-0.05, 0.0, hz], "dir": [-1.0, 0.0, 0.0],
             "surface": True},
            {"id": "top", "pos": [0.0, 0.0, hz], "dir": [0.0, 0.0, 1.0]},
            {"id": "bottom", "pos": [0.0, 0.0, -hz], "dir": [0.0, 0.0, -1.0]}]


EXTRA_FIELDS["ladder"] = {"mass": 5.0, "nodes": ladder_nodes(2.0)}
EXTRA_FIELDS["ladder_1m"] = {"mass": 2.5, "nodes": ladder_nodes(1.0)}
EXTRA_FIELDS["ladder_0.5m"] = {"mass": 1.25, "nodes": ladder_nodes(0.5)}

# every overlay-art path in the table must name its own asset directory
for _n, _f in EXTRA_FIELDS.items():
    for _k, _pfx in (("shroud", "meshes/"), ("shroud_texture", "textures/"),
                     ("chute_mesh", "meshes/"), ("chute_texture", "textures/")):
        if _k in _f:
            assert _f[_k].startswith(_pfx), _f[_k]
    # ... and no part declares a Cd here, where DRAG_CD would overwrite it
    assert not (set(_f) & (set(DRAG_KEYS) | set(DRAG_SUP_KEYS))), _n


def mesh_geom(mesh_file):
    """(radius, height, volume, planform, extents, bounds, volume_is_shell): half
    largest x/y span, z span, enclosed volume (bounding-cylinder fallback only for a
    genuinely OPEN mesh, e.g. an engine shroud -- volume_is_shell says which happened,
    because an allowance added on top of a fallback volume bills the same interior
    twice), the area of the mesh's x/z footprint, the raw (x, y, z)
    spans, and the (min, max) corners. Off-centre meshes (the landing leg)
    need bounds: their PartDef radius is the SYMMETRISED half-span
    (meshHalfSpan), not half the full span."""
    t = trimesh.load_mesh(os.path.join(REPO_ROOT, "res", mesh_file), process=False)
    # Watertightness is a property of the GEOMETRY, not of how the file happens to
    # split vertices for texture corners. With process=False, a .obj that carries a
    # UV per corner -- every Blender-authored part in res/meshes, and anything with
    # per-face UV islands -- loads as a pile of disconnected faces that reports
    # itself OPEN, so the volume below fell back to the bounding cylinder: the 2 m
    # cone capsule was weighed at 1149.82 kg instead of the 499.82 kg its own mesh
    # encloses, and a cone's bounding cylinder is 3x the cone. Merging first means
    # only genuinely open meshes (an engine shroud) take the fallback.
    t.merge_vertices(merge_tex=True, merge_norm=True)
    ext = t.extents                       # (x, y, z) spans
    radius = max(ext[0], ext[1]) / 2.0
    height = ext[2]
    if t.is_watertight:
        volume = float(t.volume)
        volume_is_shell = True
        # A watertight mesh wound inside-out has a NEGATIVE volume, and volume
        # is what every mass below is minted from. A zero-volume watertight mesh is
        # the other half of the trap: merging the corners of a shell with no
        # thickness -- chute_canopy.obj is a canopy of flat quads -- closes it into
        # something trimesh calls watertight enclosing nothing. It is only ever
        # referenced as chute_mesh, so mesh_geom never sees it today; if it ever
        # becomes a part's own mesh, this fires instead of minting a massless part.
        assert volume > 0.0, mesh_file
    else:
        volume = math.pi * radius * radius * height
        volume_is_shell = False
    return (round(radius, 3), round(height, 3), volume,
            hull_area([(v[0], v[2]) for v in t.vertices]),
            (float(ext[0]), float(ext[1]), float(ext[2])),
            (t.bounds[0].tolist(), t.bounds[1].tolist()),
            volume_is_shell)


def hull_area(pts):
    """Area of the 2D convex hull of (x, y) points: monotone chain + shoelace.
    A plate's footprint (gen_wing.py) is its planform, so this is what a lifting
    surface actually pushes on -- convex hull, so a planform with a notch in it
    would over-declare."""
    p = sorted(set((float(x), float(y)) for x, y in pts))
    if len(p) < 3:
        return 0.0

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    hull = []
    for q in p:
        while len(hull) >= 2 and cross(hull[-2], hull[-1], q) <= 0.0:
            hull.pop()
        hull.append(q)
    k = len(hull) + 1
    for q in reversed(p):
        while len(hull) >= k and cross(hull[-2], hull[-1], q) <= 0.0:
            hull.pop()
        hull.append(q)
    a = 0.0
    for i in range(len(hull) - 1):
        x0, y0 = hull[i]
        x1, y1 = hull[i + 1]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def plate(name, mesh, extents):
    """Assert a plate part is authored in the frame a surface mount expects:
    thin along Y, so its span is X (outboard -- the synthesized surface node
    sits at -radius, shipdef.h) and its chord is Z (fore-aft -- the parent's
    stack axis, shipdef.cpp surfaceMatingRot). Get this wrong and the numbers
    below stop describing the part: radius is no longer half the span, the
    footprint is no longer the planform, and the plate mounts edge-up. That is
    how a transposed gen_wing.py hid for as long as the wing was square."""
    ex, ey, ez = extents
    assert ey < ex and ey < ez, ("%s (%s): extents %gx%gx%g are not a Y-thin plate"
                                 % (name, mesh, ex, ey, ez))


def clean(x):
    """round to a clean number (ints where whole, else 2 dp)."""
    r = round(float(x), 2)
    return int(r) if abs(r - round(r)) < 1e-9 else r


def ladder_spec(ptype, mesh):
    """The climbable segment (a -> b) + outward hang direction, in the part's
    LOCAL frame -- the "ladder" field src/shipdef.cpp parses and the EVA law
    climbs (reports/ladders2026_11_21). Read off the mesh so a reshaped part
    moves its ladder with it:
      capsule: the bottom 2/3 of the slope at azimuth 0 (+X); b is the hatch
               the kerbal spawns holding, out the slope's outward normal.
      fuselage: the parallel midbody at azimuth 0 (the ogive ends taper).
      ladder part: the outer (+X) face, opposite the surface node."""
    t = trimesh.load_mesh(os.path.join(REPO_ROOT, "res", mesh), process=False)
    vs = t.vertices
    if ptype == "ladder":
        hx = max(abs(v[0]) for v in vs)
        hz = max(abs(v[2]) for v in vs)
        a, b, out = [hx, 0.0, -hz], [hx, 0.0, hz], [1.0, 0.0, 0.0]
    elif ptype == "capsule":
        zmin = min(v[2] for v in vs)
        zmax = max(v[2] for v in vs)
        rbot = max(math.hypot(v[0], v[1]) for v in vs if abs(v[2] - zmin) < 1e-6)
        rtop = max(math.hypot(v[0], v[1]) for v in vs if abs(v[2] - zmax) < 1e-6)
        a = [rbot, 0.0, zmin]
        b = [rbot + (rtop - rbot) * 2.0 / 3.0, 0.0, zmin + (zmax - zmin) * 2.0 / 3.0]
        n = [zmax - zmin, 0.0, rbot - rtop]   # perpendicular to the slope, outward
        L = math.sqrt(sum(c * c for c in n))
        out = [c / L for c in n]
    else:   # fuselage: stations at (essentially) the full radius are the midbody
        rs = [math.hypot(v[0], v[1]) for v in vs]
        rmax = max(rs)
        mid = [v[2] for v, r in zip(vs, rs) if r >= 0.99 * rmax]
        assert mid, mesh
        a, b, out = [rmax, 0.0, min(mid)], [rmax, 0.0, max(mid)], [1.0, 0.0, 0.0]
    return {"a": [clean(c) for c in a], "b": [clean(c) for c in b],
            "out": [round(c, 4) for c in out]}


# Display-only labels for the UI (behavior still comes from the part fields).
DISPLAY_BASE = {
    "capsule":        "Capsule",
    "reaction_wheel": "Reaction Wheel",
    "battery":        "Battery",
    "rtg":            "RTG",
    "engine":         "Engine",
    "orbital_engine": "Orbital Engine",
    "nuclear_engine": "Nuclear Engine",
    "jet":            "Jet",
    "jet_tank":       "Jet Fuel Tank",
    "fuel_tank":      "Fuel Tank",
    "mono_tank":      "Mono Tank",
    "rcs":            "RCS",
    "adapter":        "Adapter",
    "decoupler":      "Decoupler",
    "docking_port":   "Docking Port",
    "nose_cap":       "Nose Cap",
    "kerbal":         "Staff",
    "cargo":          "Cargo Crate",
    "liftbody":       "Lifting Body",
    "materials_pod":  "Materials Pod",
    "barometer":      "Barometer",
    "thermometer":    "Thermometer",
    "wing":           "Wing",
    "rudder":         "Rudder",
    "elevator":       "Elevator",
    "aileron":        "Aileron",
    "chute":          "Parachute",
    "gear":           "Landing Gear",
    "leg":            "Landing Leg",
    "ladder":         "Ladder",
    "fuel_link":      "Fuel Link",
}

def display_name_for(name, ptype, radius, height):
    base = DISPLAY_BASE.get(ptype, name)
    # leg: its declared radius is the symmetrised half-span (off-centre mesh),
    # not the `radius` this function is handed, so a diameter suffix would lie.
    if ptype in ("kerbal", "fuel_link", "barometer", "thermometer", "leg"):
        return base
    # ladder: the length is the size that matters, not the diameter
    if ptype == "ladder":
        return "%s (%sm)" % (base, clean(height))
    if name == "decoupler_radial":
        return "Radial Decoupler"
    if name == "rcs_box":
        # the count comes from the authored bells, so the label cannot rot
        return "RCS box (%d-dir)" % len(EXTRA_FIELDS[name]["rcs_nozzle_dirs"])
    if ptype == "adapter":
        m = re.match(r"adapter_r(.*)to(.*)$", name)
        if m:
            return "%s %s to %s" % (base, clean(m.group(1)), clean(m.group(2)))
        return base
    # The straight-walled pods say so: at 2 m a cylinder and the shipped cone are
    # otherwise two palette rows reading "Capsule (2m)" with nothing to tell a
    # player which one takes a docking port flush (#1170). Prefix rule, so a 3 m
    # cylinder inherits the label instead of a per-name exception.
    if name.startswith("capsule_cyl"):
        return "%s (%sm, cylinder)" % (base, clean(radius * 2.0))
    # base name + DIAMETER (radius * 2); tanks also carry height
    d = clean(radius * 2.0)
    if ptype in ("fuel_tank", "jet_tank"):
        return "%s (%sm x %sm)" % (base, d, clean(height))
    return "%s (%sm)" % (base, d)


def generate(name, ptype, mesh, texture):
    if ptype == "fuel_link":
        e = {"name": name, "type": ptype, "display_name": "Fuel Link"}
        e.update(EXTRA_FIELDS[name])
        return e

    radius, height, volume, planform, extents, bounds, volume_is_shell = mesh_geom(mesh)
    e = {
        "name": name,
        "type": ptype,
        "display_name": display_name_for(name, ptype, radius, height),
        "mesh": mesh,
        "texture": texture,
    }
    assert mesh.startswith("meshes/"), mesh
    assert texture.startswith("textures/"), texture

    if ptype in ("engine", "orbital_engine", "nuclear_engine"):
        if ptype == "nuclear_engine":
            # nuclear thermal: H2 only (reactor heats hydrogen; no oxidizer)
            thrust = NUCLEAR_THRUST_PER_M2 * radius * radius
            ve = NUCLEAR_EXHAUST_VELOCITY
            mass_per_n = NUCLEAR_MASS_PER_N
            propellant = {"hydrogen": clean(thrust / ve)}
        else:
            thrust = ENGINE_THRUST_PER_M2 * radius * radius
            if ptype == "orbital_engine":
                # 1/3 thrust -> 1/3 mass and 1/3 propellant flow (same ve)
                thrust /= 3.0
            ve = EXHAUST_VELOCITY
            mass_per_n = ENGINE_MASS_PER_N
            # chemical: H2 + LOX, each half the total flow (thrust / ve)
            half = clean(thrust / (2.0 * ve))
            propellant = {"hydrogen": half, "lox": half}
        e["mass"] = clean(thrust * mass_per_n)
        e["radius"] = radius
        e["height"] = height
        e["propellant"] = propellant
        e["exhaust_velocity"] = ve
        if ptype in ("engine", "orbital_engine"):
            e["gimbal_max"] = ENGINE_GIMBAL_MAX
            e["gimbal_rate"] = ENGINE_GIMBAL_RATE
    elif ptype == "jet":
        # jet_fan_thrust is the static (fan) thrust; the ram term is added at
        # runtime. Burns JET FUEL only (air is the free oxidizer -- no LOX).
        fan = JET_FAN_PER_M2 * radius * radius
        intake = JET_INTAKE_PER_M2 * radius * radius
        e["mass"] = clean(JET_MASS_PER_M2 * radius * radius)
        e["radius"] = radius
        e["height"] = height
        e["propellant"] = {"jetfuel": clean(JET_FUEL_PER_M2 * radius * radius)}
        e["exhaust_velocity"] = JET_EXH_VEL
        e["jet"] = True
        e["jet_fan_thrust"] = clean(fan)
        e["jet_intake_area"] = clean(intake)
    elif ptype == "fuel_tank":
        capacity = volume * PROP_DENSITY
        dry = volume * TANK_DRY_DENSITY
        half = capacity / 2.0
        # mass = DRY structure; propellant rides `capacity` + effectiveMass
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrogen": clean(half), "lox": clean(half)}
    elif ptype == "mono_tank":
        capacity = volume * HYDRAZINE_DENSITY
        dry = volume * TANK_DRY_DENSITY
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrazine": clean(capacity)}
    elif ptype == "jet_tank":
        capacity = volume * JET_FUEL_DENSITY
        dry = volume * TANK_DRY_DENSITY
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"jetfuel": clean(capacity)}
    elif ptype == "fuselage":
        # The body is also the tank (a separate tank part is what this replaces).
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"jetfuel": clean(0.5 * volume * JET_FUEL_DENSITY)}
    elif ptype == "rcs":
        # A box of fixed nozzles scales with nothing: mass, the per-bell rating
        # and the bells themselves are DECLARED (EXTRA_FIELDS), the way the
        # decouplers' are. #1120: the size-scaled round pods that used to fall
        # out of this branch claimed bells on their stack faces.
        declared = EXTRA_FIELDS.get(name)
        if not declared or "mass" not in declared or "rcs_thrust" not in declared:
            raise SystemExit("%s: an rcs part declares its mass, rcs_thrust and "
                             "rcs_nozzle_dirs in EXTRA_FIELDS -- there is no "
                             "size-scaled RCS rating any more (#1120)" % name)
        e["mass"] = clean(declared["mass"])
        e["radius"] = radius
        e["height"] = height
        e["rcs_thrust"] = clean(declared["rcs_thrust"])
    elif ptype in ("decoupler", "docking_port", "barometer", "thermometer", "chute"):
        # mass is declared in EXTRA_FIELDS; radius/height follow the mesh
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    elif ptype == "gear":
        # mass + suspension declared (EXTRA_FIELDS); the wheel radius is the
        # mesh's half across-axle extent (the wheel's diameter lies along Y).
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
        e["wheel_radius"] = clean(extents[1] / 2.0)
    elif ptype == "leg":
        # mass + suspension declared (EXTRA_FIELDS). The leg is authored
        # off-centre (the strut leans -X), so its radius is the SYMMETRISED
        # half-span the catalog pins against (PartDef::meshHalfSpan), not
        # mesh_geom's half-full-span.
        lo, hi = bounds
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = clean(max(abs(lo[0]), abs(hi[0]), abs(lo[1]), abs(hi[1])))
        e["height"] = height
    elif ptype == "ladder":
        # mass declared (EXTRA_FIELDS); the climb segment is authored below
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    elif ptype == "battery":
        active = volume * BATTERY_ACTIVE_DENSITY
        dry = volume * BATTERY_DRY_DENSITY
        e["mass"] = clean(active + dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"ec": clean(active * BATTERY_WATTS_PER_KG)}
    elif ptype == "rtg":
        e["mass"] = clean(volume * RTG_DENSITY)
        e["radius"] = radius
        e["height"] = height
        e["power_gen"] = clean(volume * RTG_WATTS_PER_M3)
    elif ptype == "wing":
        plate(name, mesh, extents)
        e["mass"] = clean(volume * WING_DENSITY)
        e["radius"] = radius
        e["height"] = height
        # The mesh's footprint, since that is the plate the air pushes on. The
        # radius*height this replaces is (span/2)*chord, which is exactly right
        # for a tapered planform and half the area of a rectangle (#455).
        e["lift_area"] = clean(planform)
        assert e["lift_area"] > 0, (name, mesh)
        e["cl"] = WING_CL
        e["stall_angle"] = WING_STALL_ANGLE
    elif ptype in ("rudder", "elevator", "aileron"):
        # one steering axis each: rudder=yaw, elevator=pitch, aileron=roll
        plate(name, mesh, extents)
        e["mass"] = clean(volume * RUDDER_DENSITY)
        e["radius"] = radius
        e["height"] = height
        # A fin keeps the authority it had before its mesh got its own shape:
        # 0.5 is a compatibility factor, not a derived one, and #399 (a fin's
        # moment runs many times the ship's wheel budget, growing with v^2) is
        # where that ratio should actually come from.
        e["control_area"] = clean(planform * CONTROL_AREA_FRACTION)
        assert e["control_area"] > 0, (name, mesh)
        e["control_axis"] = {"rudder": "yaw", "elevator": "pitch",
                             "aileron": "roll"}[ptype]
        e["cl_control"] = RUDDER_CL
        e["max_deflection"] = RUDDER_MAX_DEFLECTION
        e["stall_angle"] = CONTROL_STALL_ANGLE
    elif ptype == "kerbal":
        # mass is the DRY body, declared in EXTRA_FIELDS (mesh supplies shape only)
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    else:  # capsule / reaction_wheel / adapter / nose_cap / cargo
        outfit = POD_OUTFIT_KG.get(name, 0.0)
        # The allowance rides ON TOP of the shell's own volume, so it needs the shell.
        # On mesh_geom's bounding-cylinder fallback it bills that interior twice, and
        # nothing else here would notice -- capsule_r0.5h1.5 would go 215.59 -> 337.46.
        assert outfit == 0.0 or volume_is_shell, (
            name, "outfit allowance on a fallback volume")
        e["mass"] = clean(volume * MASS_DENSITY[ptype] + outfit)
        e["radius"] = radius
        e["height"] = height
        if ptype == "capsule":
            e["torque"] = clean(CAPSULE_TORQUE_PER_M * radius)
            # A pod's wheel is a housing of three orthogonal drums (#593), not one
            # stacked disc: a single drum levers the stack axis and nothing else, so
            # a coasting capsule could roll and could not point itself.
            e["wheel_axes"] = 3
            # The pod's drums bill like any other wheel's (#596). Before this no
            # capsule had a power_draw, so commanding a pod-only hull added nothing to
            # its drain: the power block's Active row (Game Debug, F1) read 0.0 W, and
            # the pod's own battery fell at life-support rate no matter how hard the
            # stick worked -- the same authority through a wheel block ate the pool
            # that much faster.
            # The two lines below are separate and ride the CREW: life support is
            # always on, and the built-in battery is storage, not a draw.
            e["power_draw"] = clean(WHEEL_DRAW_WATTS_PER_NM * e["torque"])
            # constant life-support draw + small built-in battery, both ~ crew
            crew = EXTRA_FIELDS[name]["crew_capacity"]
            e["power_draw_constant"] = clean(CAPSULE_LIFE_SUPPORT_W_PER_CREW * crew)
            e["capacity"] = {"ec": clean(CAPSULE_BATTERY_WH_PER_CREW * crew)}
        elif ptype == "reaction_wheel":
            e["torque"] = clean(WHEEL_TORQUE_PER_M * radius)
            e["wheel_axes"] = 3   # the same triad housing, one drum per axis
            e["power_draw"] = clean(WHEEL_DRAW_WATTS_PER_NM * e["torque"])

    # The climbable segment: every crew part carries a hatch ladder (invisible,
    # pure data) and the ladder part carries its own. Presence of the field is
    # what makes a part climbable (PartDef::isLadder).
    if ptype in ("capsule", "fuselage", "ladder"):
        e["ladder"] = ladder_spec(ptype, mesh)

    e.update(EXTRA_FIELDS.get(name, {}))
    # #1081: a docking port's SURFACE node is its bottom FLANGE, not the
    # synthesized -X side node. attachSurface opposes the child NODE's dir to
    # the contact normal, so a flange node (dir -Z, along the bore) leaves the
    # bore pointing OUT of the hull it is bolted to; the side node left it
    # tangent. top/bottom stay unflagged: VAB stack snap skips surface nodes
    # (vab.cpp), so flagging the bottom node itself would cost the port its
    # axial stacking -- the flange is a third node, geometrically coincident
    # with bottom. The half-heights are exact binary fractions, so no clean().
    if ptype == "docking_port":
        hz = e["height"] / 2.0
        e["nodes"] = [
            {"id": "top", "pos": [0.0, 0.0, hz], "dir": [0.0, 0.0, 1.0]},
            {"id": "bottom", "pos": [0.0, 0.0, -hz], "dir": [0.0, 0.0, -1.0]},
            {"id": "srf", "pos": [0.0, 0.0, -hz], "dir": [0.0, 0.0, -1.0],
             "surface": True},
        ]
    d = dict(DRAG_CD.get(ptype) or {})
    d.update(DRAG_CD_BY_NAME.get(name, {}))
    if d:
        # A name override sits ON TOP of its type's family, so the family must
        # be complete (a lone override would silently lose a Cd axis).
        assert set(d) == set(DRAG_KEYS), name
        for _k in DRAG_KEYS:
            e[_k] = d[_k]
    sup = DRAG_CD_SUP_BY_NAME.get(name, DRAG_CD_SUP.get(ptype))
    if sup:
        # drag.h joins this set against the subsonic one across the band, so a
        # supersonic family whose subsonic anchors are missing joins against 0.
        assert all(k in d for k in DRAG_KEYS[1:]), name
        for _k, _v in zip(DRAG_SUP_KEYS, sup):
            e[_k] = _v
    return e


def summary_line(e):
    n = e["name"]
    if "fuel_link" in e:
        return "  %-24s virtual one-way fuel link" % n
    if "chute_area" in e:
        return "  %-24s Cd.A=%5s m^2  mass=%7s" % (n, e["chute_area"], e["mass"])
    if "lift_area" in e:
        return "  %-24s S=%5s  cl=%4s  A_stall=%s  mass=%7s" % (
            n, e["lift_area"], e["cl"], e["stall_angle"], e["mass"])
    if "control_area" in e:
        return "  %-24s S=%5s  cl_c=%4s  axis=%-6s delta_max=%s A_stall=%s  mass=%7s" % (
            n, e["control_area"], e.get("cl_control", 0),
            e.get("control_axis", "pitch"), e["max_deflection"],
            e["stall_angle"], e["mass"])
    if "propellant" in e:
        rate = sum(e["propellant"].values())
        t = rate * e["exhaust_velocity"]
        fuels = " ".join("%s@%.2f" % (k, v) for k, v in e["propellant"].items())
        return "  %-24s T=%8.1fkN  %s  mass=%7s" % (
            n, t / 1e3, fuels, e["mass"])
    if "capacity" in e and "hydrogen" in e["capacity"]:
        c = e["capacity"]["hydrogen"] + e["capacity"].get("lox", 0.0)
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "capacity" in e and "jetfuel" in e["capacity"]:
        c = e["capacity"]["jetfuel"]
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "capacity" in e and "hydrazine" in e["capacity"]:
        c = e["capacity"]["hydrazine"]
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "rcs_thrust" in e:
        return "  %-24s RCS=%7.1fkN  mass=%7s" % (n, e["rcs_thrust"] / 1e3, e["mass"])
    if "power_draw_constant" in e:
        tor = "  torque=%s" % e["torque"] if "torque" in e else ""
        # A pod now has BOTH electrical figures (#596): the constant life-support
        # draw and its drums' active draw. Hiding the second here is how a balance
        # rule the catalog owns stops being visible where it is set.
        dr = "  draw=%dW" % e["power_draw"] if "power_draw" in e else ""
        ec = "  EC=%dWh" % e["capacity"]["ec"] if "capacity" in e and "ec" in e["capacity"] else ""
        return "  %-24s mass=%7s%s%s  const=%dW%s" % (n, e["mass"], tor, dr, e["power_draw_constant"], ec)
    if "capacity" in e and "ec" in e["capacity"]:
        return "  %-24s EC=%6dWh  mass=%7s" % (n, e["capacity"]["ec"], e["mass"])
    if "power_gen" in e:
        return "  %-24s gen=%5dW  mass=%7s" % (n, e["power_gen"], e["mass"])
    tor = "  torque=%s" % e["torque"] if "torque" in e else ""
    draw = "  draw=%dW" % e["power_draw"] if "power_draw" in e else ""
    return "  %-24s mass=%7s%s%s" % (n, e["mass"], tor, draw)


def check(parts, path):
    """Verify the committed parts.json still matches this script."""
    if not os.path.exists(path):
        print(f"MISSING {path}")
        return False
    with open(path) as f:
        committed = json.load(f)
    out = []
    cb = {x["name"]: x for x in committed.get("parts", [])}
    gb = {x["name"]: x for x in parts}
    for name in cb:
        if name not in gb:
            out.append(f"part {name}: in committed file, missing from generated")
    for name in gb:
        if name not in cb:
            out.append(f"part {name}: in generated, missing from committed file")
    # Order too: the array order is this script's to choose and it is the
    # catalog's own order (the VAB palette lists it verbatim), so a
    # hand-reorder of the JSON is drift as well -- same pin as gen_systems.py.
    cn = [x["name"] for x in committed.get("parts", [])]
    gn = [x["name"] for x in parts]
    if cn != gn and sorted(cn) == sorted(gn):
        out.append("parts: same names in a different order")
    for name in cb:
        if name in gb and cb[name] != gb[name]:
            for k in sorted(set(cb[name]) | set(gb[name])):
                if cb[name].get(k) != gb[name].get(k):
                    out.append(f"{name}.{k}: committed={cb[name].get(k)!r} "
                               f"generated={gb[name].get(k)!r}")
    # Text too, not just parsed values: the layout (key order, 12 vs 12.0, one
    # nozzle bell per line) is this script's to choose as well, and a
    # parsed-only check would stay green while a regenerate churned the file.
    with open(path) as f:
        if f.read() != dump_parts(parts) + "\n":
            out.append("text: regenerating parts.json would change its layout")
    if out:
        print(f"DRIFT   {path}:")
        for line in out:
            print(f"    {line}")
        return False
    print(f"OK      {path}")
    return True


def dump_parts(parts):
    """The catalog as text. indent=2 would spread a nozzle's [x, y, z] over five
    lines, so each integer triple is put back on one -- the committed file's
    shape, so regenerating it is a no-op."""
    text = json.dumps({"parts": parts}, indent=2)
    return re.sub(r"\[\s+(-?\d+),\s+(-?\d+),\s+(-?\d+)\s+\]",
                  lambda m: "[%s, %s, %s]" % m.groups(), text)


ASSET_KEYS = ("mesh", "texture", "chute_mesh", "chute_texture",
              "shroud", "shroud_texture")


def check_assets(parts):
    """Every asset the catalog names has to exist in res/. A wrong string here is
    invisible until the part renders black in the VAB, and the art pins
    (check_mesh_pins.py) only weigh files that are already named correctly."""
    for p in parts:
        for k in ASSET_KEYS:
            v = p.get(k)
            if v is not None:
                assert os.path.exists(os.path.join(REPO_ROOT, "res", v)), \
                    "part %s names res/%s, which is not in the tree" % (p["name"], v)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "res", "data", "parts.json"),
                    help="output parts.json (default: res/data/parts.json)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the catalog table without writing")
    ap.add_argument("--check", action="store_true",
                    help="verify the committed parts.json still matches this "
                         "script, without writing anything")
    ap.add_argument("--strict", action="store_true",
                    help="with --check, exit non-zero on drift. Default is advisory "
                         "(WARN, exit 0): the numbers get hand-edited too, and this is a "
                         "two-person project, not a release pipeline.")
    a = ap.parse_args()

    parts = [generate(*p) for p in PARTS]
    check_assets(parts)

    # A renamed or dropped pod would otherwise shed its allowance in silence: the hull
    # loses 121.87 kg at its nose and the next person chases the flight model instead of
    # the catalogue (#1203 is what that chase looks like).
    unauthored = sorted(set(POD_OUTFIT_KG) - {e["name"] for e in parts})
    assert not unauthored, "POD_OUTFIT_KG names parts that do not exist: %s" % unauthored

    if a.check:
        ok = check(parts, a.out)
        if not ok and not a.strict:
            print("WARN: parts.json drifted from gen_parts.py -- advisory, not a build "
                  "failure. Reconcile by hand or run 'python3 utils/art/gen_parts.py' "
                  "(it overwrites the JSON). --strict reddens this.")
        raise SystemExit(0 if (ok or not a.strict) else 1)

    print("generated catalog: %d parts" % len(parts))
    print("(Isp = EXHAUST_VELOCITY/9.81 = %.0f s; propellant = %.0f kg/m^3)" % (
        EXHAUST_VELOCITY / 9.81, PROP_DENSITY))
    for e in parts:
        print(summary_line(e))

    if a.dry_run:
        print("\n[dry-run] not writing %s" % a.out)
        return

    with open(a.out, "w") as f:
        f.write(dump_parts(parts))
        f.write("\n")
    print("\nwrote %s" % a.out)


if __name__ == "__main__":
    main()
