#!/usr/bin/env python3
"""Generate res/data/parts.json from the part meshes + physical constants.
Size (radius/height/volume) comes from the mesh; behavior is derived from that
geometry. Mass is DRY structure only -- propellant rides `capacity` + effectiveMass."""

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
# Nuclear thermal: H2 only (no oxidizer), higher Isp, lower thrust, heavier.
NUCLEAR_THRUST_PER_M2 = 30000.0  # N, thrust at radius = 1 m (scales with r^2)
NUCLEAR_EXHAUST_VELOCITY = 9000.0  # m/s (Isp = 9000/9.81 ~ 917 s), ~2x chemical
NUCLEAR_MASS_PER_N = 0.03         # kg per newton -- reactor + shielding, ~3x chemical
HYDRAZINE_DENSITY = 100.0        # kg/m^3, monopropellant hydrazine (mono)
RCS_THRUST_PER_M2 = 200.0        # N, RCS thrust at radius = 1 m (scales with r^2)

# Jet (air-breathing): T = T_fan + m_f*V_E + rho*A*v*(V_E - v) (src/drag.h jetThrust)
JET_FAN_PER_M2    = 32000.0      # N, static (fan) thrust at radius = 1 m
JET_EXH_VEL       = 550.0        # m/s, the real exhaust velocity (not a knob)
JET_INTAKE_PER_M2 = 0.9          # m^2, effective intake area at radius = 1 m
JET_FUEL_PER_M2   = 6.82         # kg/s, jet-fuel flow at radius = 1 m
JET_MASS_PER_M2   = 1200.0       # kg, engine mass at radius = 1 m (~120 kN class)

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
    # thin plate (wing / control surfaces): span +Z, thickness Y
    "wing":           {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "rudder":         {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "elevator":       {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
    "aileron":        {"drag": 0.1, "drag_forward": 0.2, "drag_side": 1.0,
                       "drag_backward": 0.2},
}

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
    ("wing",             "wing",           "meshes/wing.obj",                     "textures/wing.png"),
    ("rudder",           "rudder",         "meshes/wing.obj",                     "textures/rudder.png"),
    ("elevator",         "elevator",       "meshes/wing.obj",                     "textures/elevator.png"),
    ("aileron",          "aileron",        "meshes/wing.obj",                     "textures/aileron.png"),
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
    "decoupler_r1":      {"mass": 50, "decoupler": True, "fuel_barrier": True},
    "decoupler_r1.5":    {"mass": 75, "decoupler": True, "fuel_barrier": True},
    "decoupler_r2.25":   {"mass": 110, "decoupler": True, "fuel_barrier": True,
                          "height": 0.5625},
    "decoupler_radial":  {"mass": 40, "decoupler": True, "fuel_barrier": True},
    "docking_port_r1":   {"mass": 50, "docking_port": True, "fuel_barrier": True},
    "docking_port_r1.5": {"mass": 75, "docking_port": True, "fuel_barrier": True},
    "docking_port_r2.25":{"mass": 110, "docking_port": True, "fuel_barrier": True,
                         "height": 0.5625},
    "fuel_link":         {"fuel_link": True},
}

for _n, _f in EXTRA_FIELDS.items():
    if "shroud" in _f:
        assert _f["shroud"].startswith("meshes/"), _f["shroud"]
    if "shroud_texture" in _f:
        assert _f["shroud_texture"].startswith("textures/"), _f["shroud_texture"]


def mesh_geom(mesh_file):
    """(radius, height, volume): half largest x/y span, z span, enclosed volume
    (or bounding-cylinder fallback if not watertight)."""
    t = trimesh.load_mesh(os.path.join(REPO_ROOT, "res", mesh_file), process=False)
    ext = t.extents                       # (x, y, z) spans
    radius = max(ext[0], ext[1]) / 2.0
    height = ext[2]
    volume = float(t.volume) if t.is_watertight else math.pi * radius * radius * height
    return round(radius, 3), round(height, 3), volume


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
    "kerbal":         "Kerbal",
    "cargo":          "Cargo Crate",
    "materials_pod":  "Materials Pod",
    "wing":           "Wing",
    "rudder":         "Rudder",
    "elevator":       "Elevator",
    "aileron":        "Aileron",
    "fuel_link":      "Fuel Link",
}

def display_name_for(name, ptype, radius, height):
    base = DISPLAY_BASE.get(ptype, name)
    if ptype in ("kerbal", "fuel_link"):
        return base
    if name == "decoupler_radial":
        return "Radial Decoupler"
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

    radius, height, volume = mesh_geom(mesh)
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
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        e["rcs_thrust"] = clean(RCS_THRUST_PER_M2 * radius * radius)
    elif ptype in ("decoupler", "docking_port"):
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
        e["lift_area"] = clean(radius * height)
        e["cl"] = WING_CL
        e["stall_angle"] = WING_STALL_ANGLE
    elif ptype in ("rudder", "elevator", "aileron"):
        # one steering axis each: rudder=yaw, elevator=pitch, aileron=roll
        e["mass"] = clean(volume * RUDDER_DENSITY)
        e["radius"] = radius
        e["height"] = height
        e["control_area"] = clean(radius * height)
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
    d = DRAG_CD.get(ptype)
    if d is not None:
        e["drag"] = d["drag"]
        e["drag_forward"] = d["drag_forward"]
        e["drag_side"] = d["drag_side"]
        e["drag_backward"] = d["drag_backward"]
    return e


def summary_line(e):
    n = e["name"]
    if "fuel_link" in e:
        return "  %-24s virtual one-way fuel link" % n
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "res", "data", "parts.json"),
                    help="output parts.json (default: res/data/parts.json)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the catalog table without writing")
    a = ap.parse_args()

    parts = [generate(*p) for p in PARTS]

    print("generated catalog: %d parts" % len(parts))
    print("(Isp = EXHAUST_VELOCITY/9.81 = %.0f s; propellant = %.0f kg/m^3)" % (
        EXHAUST_VELOCITY / 9.81, PROP_DENSITY))
    for e in parts:
        print(summary_line(e))

    if a.dry_run:
        print("\n[dry-run] not writing %s" % a.out)
        return

    with open(a.out, "w") as f:
        json.dump({"parts": parts}, f, indent=2)
        f.write("\n")
    print("\nwrote %s" % a.out)


if __name__ == "__main__":
    main()
