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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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
RCS_THRUST_PER_M2 = 200.0        # N, RCS thrust at radius = 1 m (scales with r^2)

# Jet (air-breathing): T = T_fan + m_f*V_E + rho*A*v*(V_E - v) (src/drag.h jetThrust)
JET_FAN_PER_M2    = 90000.0      # N, static (fan) thrust at radius = 1 m
JET_EXH_VEL       = 550.0        # m/s, the real exhaust velocity (not a knob)
JET_INTAKE_PER_M2 = 0.35         # m^2, effective intake area at radius = 1 m
JET_FUEL_PER_M2   = 1.2          # kg/s, jet-fuel flow at radius = 1 m (~0.47 lb/lbf/h)
JET_MASS_PER_M2   = 700.0        # kg, engine mass at radius = 1 m (~129 N/kg)

JET_FUEL_DENSITY = 70.0          # kg/m^3, jet fuel (separate resource from H2/LOX)

MASS_DENSITY = {
    "capsule":        183.0,
    "reaction_wheel": 128.0,
    "adapter":        15.0,
    "nose_cap":       192.0,
    "rcs":            40.0,
    "cargo":          40.0,
    "materials_pod":  60.0,
}

CAPSULE_TORQUE_PER_M = 200.0
WHEEL_TORQUE_PER_M = 2000.0

# electrical (KSP-style EC): watts / watt-hours
WHEEL_DRAW_WATTS_PER_M = 1000.0   # W per m of radius (r1 -> 1000 W)
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
# Fraction of a control surface's footprint that actually becomes authority.
# Not a physical constant: it is the knob that keeps a fin's moment near the
# ship's own wheel budget (#399 asks for that ratio to come from the sim).
CONTROL_AREA_FRACTION = 0.5

# Per-type aerodynamic shape coefficients (src/drag.h partCd).
# drag = symmetric default; drag_forward/side/backward by how the part faces
# the flow (nose +Z / broadside / base -Z). Keyed by TYPE (shape family).
DRAG_CD = {
    "capsule":        {"drag": 1.0, "drag_forward": 0.35, "drag_side": 0.8,
                       "drag_backward": 1.3},
    # thin disc (wheel / battery / rtg / mono / rcs / decoupler / dock port)
    "reaction_wheel": {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "materials_pod":  {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "barometer":      {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "battery":        {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "rtg":            {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "mono_tank":      {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "rcs":            {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "decoupler":      {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    "docking_port":   {"drag": 0.6, "drag_forward": 1.1, "drag_side": 0.2,
                       "drag_backward": 1.1},
    # blunt cylinder (tank / crate)
    "fuel_tank":      {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    "jet_tank":       {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    "cargo":          {"drag": 0.7, "drag_forward": 1.0, "drag_side": 1.1,
                       "drag_backward": 1.0},
    "engine":         {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "orbital_engine": {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "nuclear_engine": {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "jet":            {"drag": 0.6, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    "adapter":        {"drag": 0.2, "drag_forward": 0.9, "drag_side": 1.0,
                       "drag_backward": 0.9},
    # cone (apex = +Z): tip-first sleek, flat base blunt
    "nose_cap":       {"drag": 0.6, "drag_forward": 0.35, "drag_side": 0.9,
                       "drag_backward": 1.3},
    "kerbal":         {"drag": 0.5, "drag_forward": 0.5, "drag_side": 0.9,
                       "drag_backward": 0.9},
    # thin plate (wing / control surfaces): chord along the part's own +Z, so
    # the flow along its chord is the low-drag case and spanwise is blunt
    "wing":           {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "rudder":         {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "elevator":       {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "aileron":        {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    # parachute: stowed it is the nose-cap silhouette, but the deployed canopy
    # dominates the aero pass (the Cd.A figure lives in EXTRA_FIELDS.chute_area,
    # not here), so it gets its own Cd family rather than nose_cap's.
    "chute":          {"drag": 0.9, "drag_forward": 0.6, "drag_side": 0.9,
                       "drag_backward": 1.2},
}

# Per-part Cd overrides on top of the type's family: rcs_box is a flush cube
# with no slipstream face along its own axis, so it has no low-drag side.
DRAG_CD_BY_NAME = {
    "rcs_box":        {"drag": 0.8, "drag_side": 1.1},
}

# The Cd family those tables write. It is NOT a legal EXTRA_FIELDS key: that
# table is applied before the family, so a drag key there is silently clobbered.
DRAG_KEYS = ("drag", "drag_forward", "drag_side", "drag_backward")

# --- the catalog: (name, type, mesh, texture). Add a part = add a line. ----
# fuel_link is virtual (mesh/texture = None). Radial sizes 1.0 / 1.5 / 2.25 m.
PARTS = [
    ("capsule",          "capsule",        "meshes/capsule.obj",                  "textures/capsule.png"),
    ("capsule_r1.5h3",   "capsule",        "meshes/capsule_r1.5h3.obj",           "textures/capsule.png"),
    ("capsule_r2.25h4.5","capsule",        "meshes/capsule_r2.25h4.5.obj",        "textures/capsule.png"),
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
    ("rcs_r1",           "rcs",            "meshes/reaction_wheel_r1h0.25.obj",   "textures/reaction_wheel.png"),
    ("rcs_r1.5",         "rcs",            "meshes/reaction_wheel_r1.5h0.375.obj","textures/reaction_wheel.png"),
    ("rcs_r2.25",        "rcs",            "meshes/reaction_wheel_r2.25h0.5625.obj","textures/reaction_wheel.png"),
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
    ("wing",             "wing",           "meshes/wing_r2h2.obj",                "textures/wing.png"),
    ("rudder",           "rudder",         "meshes/fin_r1h2.obj",                 "textures/rudder.png"),
    ("elevator",         "elevator",       "meshes/fin_r1h2.obj",                 "textures/elevator.png"),
    ("aileron",          "aileron",        "meshes/fin_r1h2.obj",                 "textures/aileron.png"),
    # the parachute rides the nose-cap hull; the canopy is an overlay mesh
    ("chute",            "chute",          "meshes/nose_cap.obj",                 "textures/nose_cap.png"),
    ("fuel_link",        "fuel_link",      None,                           None),
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
    "capsule":           {"crew_capacity": 1, "experiment_storage": "container"},
    "capsule_r1.5h3":    {"crew_capacity": 3, "experiment_storage": "container"},
    "capsule_r2.25h4.5": {"crew_capacity": 6, "experiment_storage": "container"},
    # kerbal: DRY mass (full-EVA-gear minus the 10 kg RCS hydrazine, a separate
    # capacity). Suit inventory pocket; science courier (1 finding per family).
    "kerbal":            {"mass": 87.05, "capacity": {"hydrazine": 10.0},
                          "inventory_capacity": 3, "experiment_storage": "courier"},
    "cargo":             {"inventory_capacity": 10},
    "materials_pod":     {"experiment_family": "materials study",
                          "experiment_storage": "instrument"},
    # rcs_box: a flush 0.3 m cube of five fixed bells, not a round pod. Mass and
    # the per-nozzle rating are DECLARED (like the decouplers'), and the bells
    # are authored -- the -X mount face has none, so it cannot fire into the
    # hull (src/rcs.h, #428).
    "rcs_box":           {"mass": 12.0, "rcs_thrust": 60,
                          "rcs_nozzle_dirs": [
                              [1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]},
    # barometer: tiny stick instrument, mass declared (mesh supplies shape only);
    # scores biome-specific science only on the surface (res/data/experiments.json).
    "barometer":         {"mass": 10, "experiment_family": "barometer",
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
    "fuel_link":         {"fuel_link": True},
}

# every overlay-art path in the table must name its own asset directory
for _n, _f in EXTRA_FIELDS.items():
    for _k, _pfx in (("shroud", "meshes/"), ("shroud_texture", "textures/"),
                     ("chute_mesh", "meshes/"), ("chute_texture", "textures/")):
        if _k in _f:
            assert _f[_k].startswith(_pfx), _f[_k]
    # ... and no part declares a Cd here, where DRAG_CD would overwrite it
    assert not (set(_f) & set(DRAG_KEYS)), _n


def mesh_geom(mesh_file):
    """(radius, height, volume, planform): half largest x/y span, z span,
    enclosed volume (or bounding-cylinder fallback if not watertight), and the
    area of the mesh's x/z footprint."""
    t = trimesh.load_mesh(os.path.join(REPO_ROOT, "res", mesh_file), process=False)
    ext = t.extents                       # (x, y, z) spans
    radius = max(ext[0], ext[1]) / 2.0
    height = ext[2]
    volume = float(t.volume) if t.is_watertight else math.pi * radius * radius * height
    return round(radius, 3), round(height, 3), volume, hull_area([(v[0], v[2]) for v in t.vertices])


def hull_area(pts):
    """Area of the 2D convex hull of (x, y) points: monotone chain + shoelace.
    A plate's footprint (gen_wing.py) is its planform, so this is what a lifting
    surface actually pushes on, whatever the planform's shape."""
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


def clean(x):
    """round to a clean number (ints where whole, else 2 dp)."""
    r = round(float(x), 2)
    return int(r) if abs(r - round(r)) < 1e-9 else r


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
    "materials_pod":  "Materials Pod",
    "barometer":      "Barometer",
    "wing":           "Wing",
    "rudder":         "Rudder",
    "elevator":       "Elevator",
    "aileron":        "Aileron",
    "chute":          "Parachute",
    "fuel_link":      "Fuel Link",
}

def display_name_for(name, ptype, radius, height):
    base = DISPLAY_BASE.get(ptype, name)
    if ptype in ("kerbal", "fuel_link", "barometer"):
        return base
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

    radius, height, volume, planform = mesh_geom(mesh)
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
    elif ptype == "rcs":
        # The round pods scale with their face area; rcs_box declares its mass
        # and rating (a fixed box of nozzles does not scale).
        declared = EXTRA_FIELDS.get(name, {})
        e["mass"] = clean(declared.get("mass", volume * MASS_DENSITY[ptype]))
        e["radius"] = radius
        e["height"] = height
        e["rcs_thrust"] = clean(declared.get("rcs_thrust",
                                             RCS_THRUST_PER_M2 * radius * radius))
    elif ptype in ("decoupler", "docking_port", "barometer", "chute"):
        # mass is declared in EXTRA_FIELDS; radius/height follow the mesh
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
        e["mass"] = clean(volume * WING_DENSITY)
        e["radius"] = radius
        e["height"] = height
        # The plate's real footprint, not radius*height: that product is a
        # triangle's area, so it only ever matched the old 2x2 delta by
        # coincidence and under-reads a rectangle by 2x (#455).
        e["lift_area"] = clean(planform)
        e["cl"] = WING_CL
        e["stall_angle"] = WING_STALL_ANGLE
    elif ptype in ("rudder", "elevator", "aileron"):
        # one steering axis each: rudder=yaw, elevator=pitch, aileron=roll
        e["mass"] = clean(volume * RUDDER_DENSITY)
        e["radius"] = radius
        e["height"] = height
        # A hinged plate does not earn its whole footprint: it works against the
        # flow it turns, and this authority is already 10-30x the ship's wheel
        # budget at cruise (#399), so credit it with a fraction and keep that
        # fraction in one place rather than in the footprint.
        e["control_area"] = clean(planform * CONTROL_AREA_FRACTION)
        e["control_axis"] = {"rudder": "yaw", "elevator": "pitch",
                             "aileron": "roll"}[ptype]
        e["cl_control"] = RUDDER_CL
        e["max_deflection"] = RUDDER_MAX_DEFLECTION
    elif ptype == "kerbal":
        # mass is the DRY body, declared in EXTRA_FIELDS (mesh supplies shape only)
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    else:  # capsule / reaction_wheel / adapter / nose_cap / cargo
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        if ptype == "capsule":
            e["torque"] = clean(CAPSULE_TORQUE_PER_M * radius)
            # constant life-support draw + small built-in battery, both ~ crew
            crew = EXTRA_FIELDS[name]["crew_capacity"]
            e["power_draw_constant"] = clean(CAPSULE_LIFE_SUPPORT_W_PER_CREW * crew)
            e["capacity"] = {"ec": clean(CAPSULE_BATTERY_WH_PER_CREW * crew)}
        elif ptype == "reaction_wheel":
            e["torque"] = clean(WHEEL_TORQUE_PER_M * radius)
            e["power_draw"] = clean(WHEEL_DRAW_WATTS_PER_M * radius)

    e.update(EXTRA_FIELDS.get(name, {}))
    d = dict(DRAG_CD.get(ptype) or {})
    d.update(DRAG_CD_BY_NAME.get(name, {}))
    if d:
        # A name override sits ON TOP of its type's family, so the family must
        # be complete (a lone override would silently lose a Cd axis).
        assert set(d) == set(DRAG_KEYS), name
        for _k in DRAG_KEYS:
            e[_k] = d[_k]
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
        return "  %-24s S=%5s  cl_c=%4s  axis=%-6s delta_max=%s  mass=%7s" % (
            n, e["control_area"], e.get("cl_control", 0),
            e.get("control_axis", "pitch"), e["max_deflection"], e["mass"])
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
        ec = "  EC=%dWh" % e["capacity"]["ec"] if "capacity" in e and "ec" in e["capacity"] else ""
        return "  %-24s mass=%7s%s  const=%dW%s" % (n, e["mass"], tor, e["power_draw_constant"], ec)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "res", "data", "parts.json"),
                    help="output parts.json (default: res/data/parts.json)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the catalog table without writing")
    ap.add_argument("--check", action="store_true",
                    help="verify the committed parts.json still matches this "
                         "script, without writing anything; exit non-zero on drift")
    a = ap.parse_args()

    parts = [generate(*p) for p in PARTS]

    if a.check:
        raise SystemExit(0 if check(parts, a.out) else 1)

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
