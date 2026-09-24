#!/usr/bin/env python3
"""Generate res/data/parts.json from the part meshes + a few physical constants.

The meshes are the source of truth for a part's SIZE (radius/height from the
bounding box, enclosed volume for watertight meshes). The behavior values are
then derived from that geometry with a small set of physical constants, so the
catalog is reproducible and internally consistent instead of hand-tuned:

  fuel_tank       capacity = volume * PROP_DENSITY (50/50 LH2 + LOX by mass)
                  mass     = volume * TANK_DRY_DENSITY   (DRY structure only --
                  the propellant is NOT in the mass; it rides `capacity` and
                  Part::effectiveMass, shedding as the tank burns, so a spent
                  tank is left with just its structure)
  mono_tank       capacity = volume * HYDRAZINE_DENSITY (hydrazine mono, the
                  RCS fuel); mass = dry like the fuel tank. A tank, nothing else.
  rcs             rcs_thrust = RCS_THRUST_PER_M2 * radius^2 (translation
                  authority, burns hydrazine mono); mass = volume * 40
                  (mostly structure + small thrusters, lighter than the wheel)
  engine          thrust   = ENGINE_THRUST_PER_M2 * radius^2  (exit area)
                  mass     = thrust * ENGINE_MASS_PER_N
                  fuel_rate= thrust / (2 * EXHAUST_VELOCITY)   (both tanks)
  orbital_engine  like engine, but 1/3 thrust -- hence 1/3 mass and 1/3
                  fuel rate; half the height (from the mesh). A low-thrust
                  engine for orbital maneuvering.
  capsule / wheel / adapter / nose_cap
                  mass     = volume * MASS_DENSITY[<type>]
                  capsule / wheel also carry attitude torque ~ radius
                  wheel also carries power_draw = radius * WHEEL_DRAW_WATTS_PER_M
  capsule (power) power_draw_constant = crew * CAPSULE_LIFE_SUPPORT_W_PER_CREW
                  (a CONSTANT draw, life support -- on all the time, unlike a
                   wheel's power_draw which is only while active)
                  capacity[EC]        = crew * CAPSULE_BATTERY_WH_PER_CREW
                  (a small built-in battery; the reserve that keeps the crew
                   alive. Scales with crew.)
  battery         active   = volume * BATTERY_ACTIVE_DENSITY (the Li-ion cells)
                  dry      = volume * BATTERY_DRY_DENSITY (hull/BMS/wiring)
                  mass     = active + dry
                  capacity = active * BATTERY_WATTS_PER_KG (Wh of EC charge)
  rtg             mass      = volume * RTG_DENSITY (fuel + thermos + housing)
                  power_gen = volume * RTG_WATTS_PER_M3 (a constant source)
  decoupler       staging boundary: decoupler + fuel_barrier flags; the
                  mass is declared (EXTRA_FIELDS), radius/height follow
                  the mesh unless declared
  docking_port    like the decoupler (declared mass, geometry from mesh),
                  but carries the docking_port + fuel_barrier flags
  fuel_link       virtual one-way fuel connection: no mesh, no physics --
                  just the fuel_link flag (EXTRA_FIELDS)
  extras (EXTRA_FIELDS)
                  crew seats, the kerbal's RCS propellant, the decouplers'
                  declared mass + staging flags, the engine shrouds --
                  per-part values that don't derive from geometry, applied
                  on top

Radial sizes are 1.0 / 1.5 / 2.25 m (see PARTS).

Resolves res/ and parts.json relative to the repo root (the parent of
utils/, where this script lives), so it can be run from anywhere:

    python3 utils/gen_parts.py              # rewrite res/data/parts.json
    python3 utils/gen_parts.py --dry-run    # print the table, write nothing
    python3 utils/gen_parts.py --out X.json
"""

import argparse
import json
import math
import os
import re

import trimesh

# This script lives in utils/; res/ and parts.json are in the repo root
# (the parent of utils/), so resolve paths relative to the parent dir.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- physical constants (SI) ------------------------------------------------
EXHAUST_VELOCITY = 4400.0        # m/s, H2/LOX vacuum (Isp = 4400/9.81 ~ 449 s)
PROP_DENSITY = 133.0             # kg/m^3, 50/50 LH2 + LOX mixture by mass
TANK_DRY_DENSITY = 13.3          # kg/m^3, structural wall mass per tank volume
ENGINE_THRUST_PER_M2 = 50000.0   # N, thrust at radius = 1 m (scales with r^2)
ENGINE_MASS_PER_N = 0.01         # kg per newton of thrust (~100 N/kg)
# RCS (reaction control): the mono (hydrazine) tank + the thruster. Both are
# reaction-wheel sized (the flat disc meshes), 3 radial sizes. The tank is
# just a tank -- hydrazine capacity derived from the part volume like the
# LOX tank (its own geometry, not the kerbal's backpack). The thruster is
# mostly structure + a few small nozzles, so a low mass density (lighter
# than the other parts); its translation authority scales with the exit
# area (r^2).
HYDRAZINE_DENSITY = 100.0        # kg/m^3, monopropellant hydrazine (mono)
RCS_THRUST_PER_M2 = 200.0        # N, RCS thrust at radius = 1 m (scales with r^2)

# Jet engine (air-breathing): thrust from the momentum balance
#   T = T_fan + m_f*V_E + rho*A*v*(V_E - v)          (src/drag.h jetThrust)
# FAN    static (fan) thrust at sea level (the VTOL floor) -- scales with r^2
# V_E    the REAL exhaust velocity (m/s), used in the fuel + ram terms
# A      effective intake/capture area (m^2) -- scales with r^2 (the ram term)
# FUEL   the jet-fuel mass flow (kg/s), the burn rate -- scales with r^2
# The jet burns JET FUEL only (air is the free oxidizer -- no LOX, no delta-v).
JET_FAN_PER_M2    = 32000.0      # N, static (fan) thrust at radius = 1 m
JET_EXH_VEL       = 550.0        # m/s, the real exhaust velocity (not a knob)
JET_INTAKE_PER_M2 = 0.9          # m^2, effective intake area at radius = 1 m
JET_FUEL_PER_M2   = 6.82         # kg/s, jet-fuel flow at radius = 1 m
JET_MASS_PER_M2   = 1200.0       # kg, engine mass at radius = 1 m (~120 kN class)

# Jet-fuel tank: 100% jet fuel -- a SEPARATE resource from rocket H2/LOX
# (air is the free oxidizer, so no LOX, and it gives no delta-v). Derived
# from the part volume like the mono tank (the 50/50 fuel tank splits the
# volume hydrogen/LOX; the jet tank does not).
JET_FUEL_DENSITY = 70.0          # kg/m^3, jet fuel (kept at the old H2 value for balance)

# structural mass per unit enclosed volume (kg/m^3); tuned so the base (r1)
# part of each kind lands on a sensible mass, then scales with real volume
MASS_DENSITY = {
    "capsule":        183.0,     # crew module (structure + life support)
    "reaction_wheel": 128.0,     # flywheel
    "adapter":        15.0,      # thin coupler ring, mostly air
    "nose_cap":       192.0,     # thin fairing
    "rcs":            40.0,      # mostly structure + small thrusters (light)
}

# attitude authority (N m), scales with radius (leverage of the wheel/arm)
CAPSULE_TORQUE_PER_M = 200.0
WHEEL_TORQUE_PER_M = 2000.0

# electrical (KSP-style EC): power in watts (W), charge in watt-hours (Wh).
# Like the fuel tanks, the values are derived from the part VOLUME (the mesh
# is the source of truth for size), so the three sizes scale consistently:
#   reaction wheel  power_draw = radius * WHEEL_DRAW_WATTS_PER_M
#                   (draws while active; scales with radius like the torque)
#   rtg             power_gen  = volume * RTG_WATTS_PER_M3
#                   (a constant source; output scales with volume)
#   battery         capacity   = volume * BATTERY_ACTIVE_DENSITY * BATTERY_WATTS_PER_KG
#                   (the cells fill the volume at the pack's bulk density;
#                    the charge is that mass times the cell Wh/kg. Like the
#                    fuel tank, a share of the volume is hull/BMS/wiring.)
#   capsule         power_draw_constant = crew * CAPSULE_LIFE_SUPPORT_W_PER_CREW
#                   (a CONSTANT draw, life support, on all the time; scales
#                    with the crew it shelters)
#                   capacity[EC]        = crew * CAPSULE_BATTERY_WH_PER_CREW
#                   (a small built-in battery, the reserve that keeps the
#                    crew alive; scales with crew.)
WHEEL_DRAW_WATTS_PER_M = 1000.0   # W per m of radius (r1 -> 1000 W)
RTG_WATTS_PER_M3       = 380.0    # W per m^3 (r1 -> ~300 W)
RTG_DENSITY            = 150.0    # kg/m^3, fuel + thermoelectrics + housing
BATTERY_ACTIVE_DENSITY = 1000.0   # kg/m^3, Li-ion pack bulk density
BATTERY_WATTS_PER_KG   = 200.0    # Wh/kg, modern space Li-ion (per kg of cells)
BATTERY_DRY_DENSITY    = 100.0    # kg/m^3, hull + BMS + wiring overhead
CAPSULE_LIFE_SUPPORT_W_PER_CREW = 100.0   # W per crew, constant (base capsule = 100 W)
CAPSULE_BATTERY_WH_PER_CREW     = 2000.0  # Wh per crew (~20 laptop batteries; base = 2 kWh)

# wing (a lifting surface): mass from the structure; the aero fields are
# design values (not geometry-derived). lift_area is the planform (delta)
# area = radius * height (a 2 m x 2 m triangle = 2 m^2). cl is the
# lift-curve slope (per radian, ~ the thin-airfoil 2*pi); stall_angle is
# where the lift peaks and the flow separates (see src/drag.h liftCurve).
# The wing's DRAG comes from the ship's convex-hull silhouette (src/drag.h,
# reports/projected-drag) -- a thin plate is nearly edge-on to the flow, so
# it drags little -- and the global --drag-cd; there is no per-part drag knob.
WING_DENSITY       = 50.0     # kg/m^3, wing structure (skin + spars)
WING_CL            = 6.0      # lift-curve slope (per radian)
WING_STALL_ANGLE   = 0.35     # rad (~20 deg), where lift peaks + the flow stalls

# rudder (a control surface): a deflection-driven steering surface (an
# elevator / rudder). Reuses the wing mesh (a flat plate) + texture. The
# control_area is the planform area = radius * height; cl_control is the
# deflection effectiveness (per radian, the same thin-airfoil 2*pi as the
# wing's lift-curve slope); max_deflection is the travel limit (rad).
# Its drag, like the wing's, comes from the ship's hull silhouette + the
# global --drag-cd (no per-part drag knob).
RUDDER_DENSITY        = 50.0  # kg/m^3, control-surface structure (skin + spars)
RUDDER_CL             = 6.0   # deflection effectiveness (per radian)
RUDDER_MAX_DEFLECTION = 0.35  # rad (~20 deg), the travel limit

# --- the catalog: (name, type, mesh, texture). Add a part = add a line. ----
# fuel_link is virtual: mesh/texture are None and generate() skips the
# geometry step for it.
# Radial sizes are 1.0 / 1.5 / 2.25 m. Heights follow the per-type ratio
# (capsule & engine h=2r, wheel h=0.25r, nose cap h=r/2, adapter h=max(r)/2);
# tanks keep the independent fuel-height options.
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
    # jet engine (air-breathing): one size (r1), the rocket engine's mesh
    # with its own tinted texture (like the rudder reuses wing.obj).
    ("jet",            "jet",            "meshes/engine.obj",                   "textures/jet_engine.png"),
    # jet-fuel tanks: one radius, the fuel tank's heights.
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
    # mono (hydrazine) RCS fuel tank + RCS thruster, reaction-wheel sized
    # (the flat disc meshes), 3 radial sizes. The tank stores hydrazine; the
    # thruster provides translation authority (burns the tank's hydrazine).
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
    # docking ports: same geometry + declared mass as the decouplers, but
    # they mate ships together instead of staging (docking_port, not
    # decoupler). Mesh/texture are their own copies of the decoupler's.
    ("docking_port_r1",    "docking_port", "meshes/docking_port_r1.obj",          "textures/docking_port.png"),
    ("docking_port_r1.5",  "docking_port", "meshes/docking_port_r1.5.obj",        "textures/docking_port.png"),
    ("docking_port_r2.25", "docking_port", "meshes/docking_port_r2.25.obj",       "textures/docking_port.png"),
    ("nose_cap",         "nose_cap",       "meshes/nose_cap.obj",                 "textures/nose_cap.png"),
    ("nose_cap_r1.5h0.75","nose_cap",      "meshes/nose_cap_r1.5h0.75.obj",       "textures/nose_cap.png"),
    ("nose_cap_r2.25h1.125","nose_cap",    "meshes/nose_cap_r2.25h1.125.obj",     "textures/nose_cap.png"),
    ("kerbal",           "kerbal",         "meshes/kerbal.obj",                   "textures/kerbal.png"),
    # a wing: a lifting surface (delta wing, wing.obj by gen_wing.py). Adds
    # lift + a weathervane drag to a ship (see the WING_* constants).
    ("wing",             "wing",           "meshes/wing.obj",                     "textures/wing.png"),
    # a rudder: a control surface (deflection-driven steering authority).
    # Reuses the wing mesh (a flat plate) but gets its OWN tinted texture so
    # the surfaces are visually distinct (blue rudder / red elevator / green
    # aileron) from the gray wing and each other (see the RUDDER_* const).
    ("rudder",           "rudder",         "meshes/wing.obj",                     "textures/rudder.png"),
    ("elevator",         "elevator",       "meshes/wing.obj",                     "textures/elevator.png"),
    ("aileron",          "aileron",        "meshes/wing.obj",                     "textures/aileron.png"),
    ("fuel_link",        "fuel_link",      None,                           None),
]

# per-part extra fields that do NOT derive from the geometry: crew seats,
# the kerbal's RCS propellant, the decouplers'/docking ports' mass + flags,
# the engine shrouds, and the fuel_link's flag.
# Applied on top of the generated entry so the catalog stays fully
# reproducible (no hand-edits to parts.json). The kerbal mass is declared
# (not mesh-derived) to preserve the hand-set value; the decouplers'/docking
# ports' masses likewise. decoupler_r2.25's and docking_port_r2.25's height
# are declared because mesh_geom rounds the 0.5625 m mesh span to 0.562
# (round-3, as the wheel's entry).
EXTRA_FIELDS = {
    # Engine shrouds (src/shipdef.h PartDef.shroud): the open-cylinder
    # wrap drawn over the engine while a part is attached below it
    # (utils/gen_engine_shroud.py). One shared light-gray texture.
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
    # the jet reuses the engine mesh, so it gets the engine's shroud too
    "jet":                   {"shroud": "meshes/engine_shroud.obj",
                              "shroud_texture": "textures/engine_shroud.png"},
    "capsule":           {"crew_capacity": 1},
    "capsule_r1.5h3":    {"crew_capacity": 3},
    "capsule_r2.25h4.5": {"crew_capacity": 6},
    # kerbal: DRY mass (full-EVA-gear ~94 kg minus the 10 kg RCS hydrazine,
    # which is a separate capacity that rides effectiveMass, not the body).
    "kerbal":            {"mass": 87.05, "capacity": {"hydrazine": 10.0}},
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
    """(radius, height, volume) of a part mesh.

    radius = half the largest cross-section span (x/y); height = the z span
    (the stack axis); volume = enclosed volume for watertight meshes, else the
    bounding-cylinder volume as a fallback."""
    t = trimesh.load_mesh(os.path.join(REPO_ROOT, "res", mesh_file), process=False)
    ext = t.extents                       # (x, y, z) spans
    radius = max(ext[0], ext[1]) / 2.0
    height = ext[2]
    volume = float(t.volume) if t.is_watertight else math.pi * radius * radius * height
    # the meshes are authored on integer/half-metre dims; kill float noise
    return round(radius, 3), round(height, 3), volume


def clean(x):
    """round to a clean number (ints where whole, else 2 dp)."""
    r = round(float(x), 2)
    return int(r) if abs(r - round(r)) < 1e-9 else r


# Human-readable part names for the UI. The catalog `name` is a machine id
# ("capsule_r1.5h3"); the display name is the label a person reads ("Capsule
# (3m)"). Display-only: behavior still comes from the part fields. Derived
# from the type + size, with a few specials handled by name.
DISPLAY_BASE = {
    "capsule":        "Capsule",
    "reaction_wheel": "Reaction Wheel",
    "battery":        "Battery",
    "rtg":            "RTG",
    "engine":         "Engine",
    "orbital_engine": "Orbital Engine",
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
    "wing":           "Wing",
    "rudder":         "Rudder",
    "elevator":       "Elevator",
    "aileron":        "Aileron",
    "fuel_link":      "Fuel Link",
}

def display_name_for(name, ptype, radius, height):
    base = DISPLAY_BASE.get(ptype, name)
    if ptype in ("kerbal", "fuel_link"):
        return base                       # a character / a virtual link: no size
    if name == "decoupler_radial":
        return "Radial Decoupler"         # the odd-shaped radial separator
    if ptype == "adapter":
        m = re.match(r"adapter_r(.*)to(.*)$", name)   # "adapter_r1to1.5"
        if m:
            return "%s %s to %s" % (base, clean(m.group(1)), clean(m.group(2)))
        return base
    # the rest: base name + the part's DIAMETER (radius * 2, matching the UI's
    # "dia" readout). Fuel tanks also carry height -- they come in lengths.
    d = clean(radius * 2.0)
    if ptype in ("fuel_tank", "jet_tank"):
        return "%s (%sm x %sm)" % (base, d, clean(height))
    return "%s (%sm)" % (base, d)


def generate(name, ptype, mesh, texture):
    if ptype == "fuel_link":
        # a virtual one-way fuel connection (see PartDef.fuel_link): no
        # mesh, no physics -- just the fuel_link flag from EXTRA_FIELDS
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

    if ptype in ("engine", "orbital_engine"):
        thrust = ENGINE_THRUST_PER_M2 * radius * radius
        if ptype == "orbital_engine":
            # 1/3 thrust -> 1/3 mass and 1/3 fuel rate (same exhaust velocity)
            thrust /= 3.0
        e["mass"] = clean(thrust * ENGINE_MASS_PER_N)
        e["radius"] = radius
        e["height"] = height
        e["fuel_rate"] = clean(thrust / (2.0 * EXHAUST_VELOCITY))
        e["exhaust_velocity"] = EXHAUST_VELOCITY
    elif ptype == "jet":
        # Air-breathing (see the JET_* constants + src/drag.h jetThrust).
        # jet_fan_thrust is the static (fan) thrust at sea level; the ram
        # term (rho*A*v*(V_E - v)) is added at runtime from the local air.
        # The jet burns H2 only (air is the free oxidizer -- no LOX).
        fan = JET_FAN_PER_M2 * radius * radius
        intake = JET_INTAKE_PER_M2 * radius * radius
        e["mass"] = clean(JET_MASS_PER_M2 * radius * radius)
        e["radius"] = radius
        e["height"] = height
        e["fuel_rate"] = clean(JET_FUEL_PER_M2 * radius * radius)
        e["exhaust_velocity"] = JET_EXH_VEL
        e["jet"] = True
        e["jet_fan_thrust"] = clean(fan)
        e["jet_intake_area"] = clean(intake)
    elif ptype == "fuel_tank":
        capacity = volume * PROP_DENSITY
        dry = volume * TANK_DRY_DENSITY
        half = capacity / 2.0
        # mass = DRY structure only. The propellant is NOT baked into the mass:
        # it lives in `capacity` and rides Part::effectiveMass (added back at
        # runtime), shedding as the tank burns -- so a spent tank is its
        # structure. (Was `capacity + dry`: the wet mass; see the tank-mass
        # refactor, the body is now dry + explicit fuel.)
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrogen": clean(half), "lox": clean(half)}
    elif ptype == "mono_tank":
        # hydrazine (mono) RCS fuel tank: just a tank. Capacity from the part
        # volume (like the LOX tank, but 100% hydrazine instead of 50/50);
        # the structure is the same tank dry mass.
        capacity = volume * HYDRAZINE_DENSITY
        dry = volume * TANK_DRY_DENSITY
        # DRY structure only (fuel rides capacity + effectiveMass, not the body).
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrazine": clean(capacity)}
    elif ptype == "jet_tank":
        # jet-fuel tank: 100% jet fuel -- a SEPARATE resource from rocket
        # H2/LOX (air is the free oxidizer, so no LOX and no delta-v). Like
        # the mono tank's full-volume hydrazine; the 50/50 fuel tank splits
        # the volume hydrogen/LOX. The jet draws this.
        capacity = volume * JET_FUEL_DENSITY
        dry = volume * TANK_DRY_DENSITY
        # DRY structure only (fuel rides capacity + effectiveMass, not the body).
        e["mass"] = clean(dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"jetfuel": clean(capacity)}
    elif ptype == "rcs":
        # RCS thruster: translation authority scales with the exit area
        # (r^2); the part is mostly structure + small thrusters, so a low
        # mass density (lighter than the reaction wheel).
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        e["rcs_thrust"] = clean(RCS_THRUST_PER_M2 * radius * radius)
    elif ptype in ("decoupler", "docking_port"):
        # staging boundary (decoupler) / docking port: mass is declared in
        # EXTRA_FIELDS; radius/height follow the mesh unless overridden there.
        # The decoupler/docking_port + fuel_barrier flags land in the final
        # EXTRA_FIELDS update below, keeping the key order of the entries.
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    elif ptype == "battery":
        # EC storage (KSP-style): the cells fill the part volume at the
        # pack's bulk density, the charge is that mass times the cell
        # Wh/kg, and a share of the volume is hull/BMS/wiring (dry) --
        # the same capacity + dry structure as the fuel tank.
        active = volume * BATTERY_ACTIVE_DENSITY
        dry = volume * BATTERY_DRY_DENSITY
        e["mass"] = clean(active + dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"ec": clean(active * BATTERY_WATTS_PER_KG)}
    elif ptype == "rtg":
        # constant power source: output scales with volume (more fuel +
        # thermoelectrics); the mass is the fuel/thermos/housing.
        e["mass"] = clean(volume * RTG_DENSITY)
        e["radius"] = radius
        e["height"] = height
        e["power_gen"] = clean(volume * RTG_WATTS_PER_M3)
    elif ptype == "wing":
        # a lifting surface: the wing's flat plate (radius * height = the
        # triangular plate's bounding box) is its LIFT area. Lift curve slope
        # (cl) and stall angle are declared constants (not geometry-derived).
        # Its drag comes from the ship's hull silhouette (a thin plate is
        # nearly edge-on, so it drags little) + the global --drag-cd.
        e["mass"] = clean(volume * WING_DENSITY)
        e["radius"] = radius
        e["height"] = height
        e["lift_area"] = clean(radius * height)
        e["cl"] = WING_CL
        e["stall_angle"] = WING_STALL_ANGLE
    elif ptype in ("rudder", "elevator", "aileron"):
        # a control surface: the plate's area (radius * height) is its
        # CONTROL area. The deflection effectiveness (cl_control) and travel
        # limit (max_deflection) are declared constants (not geometry-derived).
        # Each type is ONE steering axis, like the real control surfaces: a
        # rudder yaws (A/D), an elevator pitches (W/S), an aileron rolls (Q/E).
        # Its drag, like the wing's, comes from the ship's hull silhouette +
        # the global --drag-cd (no per-part drag knob).
        e["mass"] = clean(volume * RUDDER_DENSITY)
        e["radius"] = radius
        e["height"] = height
        e["control_area"] = clean(radius * height)
        e["control_axis"] = {"rudder": "yaw", "elevator": "pitch",
                             "aileron": "roll"}[ptype]
        # cl_control = the deflection effectiveness (per radian). These are
        # control surfaces, not lifting surfaces (no lift_area), so cl (the
        # lift-curve slope) stays 0 -- cl_control is their own number (I4:
        # the two "cl"s are no longer overloaded).
        e["cl_control"] = RUDDER_CL
        e["max_deflection"] = RUDDER_MAX_DEFLECTION
    elif ptype == "kerbal":
        # A character, not hardware: the mass is the DRY body (~87 kg
        # full-EVA-gear) declared in EXTRA_FIELDS, like the decouplers' -- the
        # 10 kg RCS hydrazine is a separate capacity (rides effectiveMass, not
        # the body). The mesh only supplies the shape (visual + hull).
        e["mass"] = clean(EXTRA_FIELDS[name]["mass"])
        e["radius"] = radius
        e["height"] = height
    else:  # capsule / reaction_wheel / adapter / nose_cap
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        if ptype == "capsule":
            e["torque"] = clean(CAPSULE_TORQUE_PER_M * radius)
            # a crew module also has a CONSTANT life-support draw (on all the
            # time, unlike a wheel's active power_draw) and a small built-in
            # battery (EC capacity) as the reserve; both scale with the crew
            # it shelters (EXTRA_FIELDS).
            crew = EXTRA_FIELDS[name]["crew_capacity"]
            e["power_draw_constant"] = clean(CAPSULE_LIFE_SUPPORT_W_PER_CREW * crew)
            e["capacity"] = {"ec": clean(CAPSULE_BATTERY_WH_PER_CREW * crew)}
        elif ptype == "reaction_wheel":
            e["torque"] = clean(WHEEL_TORQUE_PER_M * radius)
            e["power_draw"] = clean(WHEEL_DRAW_WATTS_PER_M * radius)

    e.update(EXTRA_FIELDS.get(name, {}))
    return e


def summary_line(e):
    n = e["name"]
    if "fuel_link" in e:
        return "  %-24s virtual one-way fuel link" % n
    if "lift_area" in e:
        # a wing: its lift area + lift curve. (Its drag is the ship's hull
        # silhouette x the global --drag-cd, so no per-part drag to show.)
        return "  %-24s S=%5s  cl=%4s  A_stall=%s  mass=%7s" % (
            n, e["lift_area"], e["cl"], e["stall_angle"], e["mass"])
    if "control_area" in e:
        # a control surface: its control area + deflection effectiveness,
        # and the weathervane drag it also provides.
        return "  %-24s S=%5s  cl_c=%4s  axis=%-6s delta_max=%s  mass=%7s" % (
            n, e["control_area"], e.get("cl_control", 0),
            e.get("control_axis", "pitch"), e["max_deflection"], e["mass"])
    if "fuel_rate" in e:
        t = 2.0 * e["fuel_rate"] * e["exhaust_velocity"]
        return "  %-24s T=%8.1fkN  rate=%7.2f  mass=%7s" % (
            n, t / 1e3, e["fuel_rate"], e["mass"])
    if "capacity" in e and "hydrogen" in e["capacity"]:
        # a fuel tank (50/50 hydrogen + LOX); mass = dry structure, fuel = capacity
        c = e["capacity"]["hydrogen"] + e["capacity"].get("lox", 0.0)
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "capacity" in e and "jetfuel" in e["capacity"]:
        # a jet fuel tank (100% jet fuel, no LOX); mass = dry structure
        c = e["capacity"]["jetfuel"]
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "capacity" in e and "hydrazine" in e["capacity"]:
        # a mono tank / the kerbal's suit fuel; mass = dry structure
        c = e["capacity"]["hydrazine"]
        return "  %-24s cap=%8skg  dry=%7skg" % (n, c, e["mass"])
    if "rcs_thrust" in e:
        return "  %-24s RCS=%7.1fkN  mass=%7s" % (n, e["rcs_thrust"] / 1e3, e["mass"])
    if "power_draw_constant" in e:
        # a capsule: attitude torque + a constant life-support draw + a small
        # built-in battery (EC capacity) -- show all three, not just the EC.
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
