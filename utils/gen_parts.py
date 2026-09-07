#!/usr/bin/env python3
"""Generate res/parts.json from the part meshes + a few physical constants.

The meshes are the source of truth for a part's SIZE (radius/height from the
bounding box, enclosed volume for watertight meshes). The behavior values are
then derived from that geometry with a small set of physical constants, so the
catalog is reproducible and internally consistent instead of hand-tuned:

  fuel_tank       capacity = volume * PROP_DENSITY (50/50 LH2 + LOX by mass)
                  dry mass = volume * TANK_DRY_DENSITY
                  mass     = capacity + dry   (the body sheds propellant as it
                  burns, so a spent tank is left with just its structure)
  mono_tank       capacity = volume * HYDRAZINE_DENSITY (hydrazine mono, the
                  RCS fuel); dry/mass like the fuel tank. A tank, nothing else.
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
  fuel_link       virtual one-way fuel connection: no mesh, no physics --
                  just the fuel_link flag (EXTRA_FIELDS)
  extras (EXTRA_FIELDS)
                  crew seats, the kerbal's RCS propellant, the decouplers'
                  declared mass + staging flags -- per-part values that
                  don't derive from geometry, applied on top

Radial sizes are 1.0 / 1.5 / 2.25 m (see PARTS).

Resolves res/ and parts.json relative to the repo root (the parent of
utils/, where this script lives), so it can be run from anywhere:

    python3 utils/gen_parts.py              # rewrite res/parts.json
    python3 utils/gen_parts.py --dry-run    # print the table, write nothing
    python3 utils/gen_parts.py --out X.json
"""

import argparse
import json
import math
import os

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

# structural mass per unit enclosed volume (kg/m^3); tuned so the base (r1)
# part of each kind lands on a sensible mass, then scales with real volume
MASS_DENSITY = {
    "capsule":        183.0,     # crew module (structure + life support)
    "reaction_wheel": 128.0,     # flywheel
    "adapter":        15.0,      # thin coupler ring, mostly air
    "nose_cap":       192.0,     # thin fairing
    "kerbal":         160.0,     # one crew member (mesh by gen_kerbal.py)
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

# --- the catalog: (name, type, mesh, texture). Add a part = add a line. ----
# fuel_link is virtual: mesh/texture are None and generate() skips the
# geometry step for it.
# Radial sizes are 1.0 / 1.5 / 2.25 m. Heights follow the per-type ratio
# (capsule & engine h=2r, wheel h=0.25r, nose cap h=r/2, adapter h=max(r)/2);
# tanks keep the independent fuel-height options.
PARTS = [
    ("capsule",          "capsule",        "capsule.obj",                  "capsule.png"),
    ("capsule_r1.5h3",   "capsule",        "capsule_r1.5h3.obj",           "capsule.png"),
    ("capsule_r2.25h4.5","capsule",        "capsule_r2.25h4.5.obj",        "capsule.png"),
    ("reaction_wheel",   "reaction_wheel", "reaction_wheel_r1h0.25.obj",   "reaction_wheel.png"),
    ("reaction_wheel_r1.5h0.375",  "reaction_wheel", "reaction_wheel_r1.5h0.375.obj",  "reaction_wheel.png"),
    ("reaction_wheel_r2.25h0.5625","reaction_wheel", "reaction_wheel_r2.25h0.5625.obj","reaction_wheel.png"),
    ("battery",          "battery",        "reaction_wheel_r1h0.25.obj",   "reaction_wheel.png"),
    ("battery_r1.5h0.375","battery",       "reaction_wheel_r1.5h0.375.obj","reaction_wheel.png"),
    ("battery_r2.25h0.5625","battery",     "reaction_wheel_r2.25h0.5625.obj","reaction_wheel.png"),
    ("rtg",              "rtg",            "reaction_wheel_r1h0.25.obj",   "reaction_wheel.png"),
    ("rtg_r1.5h0.375",   "rtg",            "reaction_wheel_r1.5h0.375.obj","reaction_wheel.png"),
    ("rtg_r2.25h0.5625", "rtg",            "reaction_wheel_r2.25h0.5625.obj","reaction_wheel.png"),
    ("engine",           "engine",         "engine.obj",                   "engine.png"),
    ("engine_r1.5h3",    "engine",         "engine_r1.5h3.obj",            "engine.png"),
    ("engine_r2.25h4.5", "engine",         "engine_r2.25h4.5.obj",         "engine.png"),
    ("orbital_engine",        "orbital_engine", "orbital_engine.obj",              "engine.png"),
    ("orbital_engine_r1.5h1.5","orbital_engine", "orbital_engine_r1.5h1.5.obj",     "engine.png"),
    ("orbital_engine_r2.25h2.25","orbital_engine","orbital_engine_r2.25h2.25.obj", "engine.png"),
    ("fuel_tank",        "fuel_tank",      "fuel_tank.obj",                "fuel_tank.png"),
    ("tank_r1h1",        "fuel_tank",      "tank_r1h1.obj",                "fuel_tank.png"),
    ("tank_r1h3",        "fuel_tank",      "tank_r1h3.obj",                "fuel_tank.png"),
    ("tank_r1h5",        "fuel_tank",      "tank_r1h5.obj",                "fuel_tank.png"),
    ("tank_r1.5h1",      "fuel_tank",      "tank_r1.5h1.obj",              "fuel_tank.png"),
    ("tank_r1.5h2",      "fuel_tank",      "tank_r1.5h2.obj",              "fuel_tank.png"),
    ("tank_r1.5h3",      "fuel_tank",      "tank_r1.5h3.obj",              "fuel_tank.png"),
    ("tank_r1.5h5",      "fuel_tank",      "tank_r1.5h5.obj",              "fuel_tank.png"),
    ("tank_r2.25h1",     "fuel_tank",      "tank_r2.25h1.obj",             "fuel_tank.png"),
    ("tank_r2.25h3",     "fuel_tank",      "tank_r2.25h3.obj",             "fuel_tank.png"),
    ("tank_r2.25h5",     "fuel_tank",      "tank_r2.25h5.obj",             "fuel_tank.png"),
    # mono (hydrazine) RCS fuel tank + RCS thruster, reaction-wheel sized
    # (the flat disc meshes), 3 radial sizes. The tank stores hydrazine; the
    # thruster provides translation authority (burns the tank's hydrazine).
    ("mono_tank_r1",     "mono_tank",      "reaction_wheel_r1h0.25.obj",   "reaction_wheel.png"),
    ("mono_tank_r1.5",   "mono_tank",      "reaction_wheel_r1.5h0.375.obj","reaction_wheel.png"),
    ("mono_tank_r2.25",  "mono_tank",      "reaction_wheel_r2.25h0.5625.obj","reaction_wheel.png"),
    ("rcs_r1",           "rcs",            "reaction_wheel_r1h0.25.obj",   "reaction_wheel.png"),
    ("rcs_r1.5",         "rcs",            "reaction_wheel_r1.5h0.375.obj","reaction_wheel.png"),
    ("rcs_r2.25",        "rcs",            "reaction_wheel_r2.25h0.5625.obj","reaction_wheel.png"),
    ("adapter_r1to1.5",  "adapter",        "adapter_r1to1.5.obj",          "adapter.png"),
    ("adapter_r1to2.25", "adapter",        "adapter_r1to2.25.obj",         "adapter.png"),
    ("adapter_r1.5to1",  "adapter",        "adapter_r1.5to1.obj",          "adapter.png"),
    ("adapter_r1.5to2.25","adapter",       "adapter_r1.5to2.25.obj",       "adapter.png"),
    ("adapter_r2.25to1", "adapter",        "adapter_r2.25to1.obj",         "adapter.png"),
    ("adapter_r2.25to1.5","adapter",       "adapter_r2.25to1.5.obj",       "adapter.png"),
    ("decoupler_r1",     "decoupler",      "decoupler_r1.obj",             "decoupler.png"),
    ("decoupler_r1.5",   "decoupler",      "decoupler_r1.5.obj",           "decoupler.png"),
    ("decoupler_r2.25",  "decoupler",      "decoupler_r2.25.obj",          "decoupler.png"),
    ("decoupler_radial", "decoupler",      "decoupler_radial.obj",         "decoupler.png"),
    ("nose_cap",         "nose_cap",       "nose_cap.obj",                 "nose_cap.png"),
    ("nose_cap_r1.5h0.75","nose_cap",      "nose_cap_r1.5h0.75.obj",       "nose_cap.png"),
    ("nose_cap_r2.25h1.125","nose_cap",    "nose_cap_r2.25h1.125.obj",     "nose_cap.png"),
    ("kerbal",           "kerbal",         "kerbal.obj",                   "kerbal.png"),
    ("fuel_link",        "fuel_link",      None,                           None),
]

# per-part extra fields that do NOT derive from the geometry: crew seats,
# the kerbal's RCS propellant, the decouplers' mass + staging flags, and
# the fuel_link's flag.
# Applied on top of the generated entry so the catalog stays fully
# reproducible (no hand-edits to parts.json). The kerbal mass is declared
# (not mesh-derived) to preserve the hand-set value; the decouplers'
# masses likewise. decoupler_r2.25's height is declared because mesh_geom
# rounds the 0.5625 m mesh span to 0.562 (round-3, as the wheel's entry).
EXTRA_FIELDS = {
    "capsule":           {"crew_capacity": 1},
    "capsule_r1.5h3":    {"crew_capacity": 3},
    "capsule_r2.25h4.5": {"crew_capacity": 6},
    "kerbal":            {"mass": 97.05, "capacity": {"hydrazine": 10.0}},
    "decoupler_r1":      {"mass": 50, "decoupler": True, "fuel_barrier": True},
    "decoupler_r1.5":    {"mass": 75, "decoupler": True, "fuel_barrier": True},
    "decoupler_r2.25":   {"mass": 110, "decoupler": True, "fuel_barrier": True,
                          "height": 0.5625},
    "decoupler_radial":  {"mass": 40, "decoupler": True, "fuel_barrier": True},
    "fuel_link":         {"fuel_link": True},
}


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


def generate(name, ptype, mesh, texture):
    if ptype == "fuel_link":
        # a virtual one-way fuel connection (see PartDef.fuel_link): no
        # mesh, no physics -- just the fuel_link flag from EXTRA_FIELDS
        e = {"name": name, "type": ptype}
        e.update(EXTRA_FIELDS[name])
        return e

    radius, height, volume = mesh_geom(mesh)
    e = {
        "name": name,
        "type": ptype,
        "mesh": mesh,
        "texture": texture,
    }

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
    elif ptype == "fuel_tank":
        capacity = volume * PROP_DENSITY
        dry = volume * TANK_DRY_DENSITY
        half = capacity / 2.0
        e["mass"] = clean(capacity + dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrogen": clean(half), "lox": clean(half)}
    elif ptype == "mono_tank":
        # hydrazine (mono) RCS fuel tank: just a tank. Capacity from the part
        # volume (like the LOX tank, but 100% hydrazine instead of 50/50);
        # the structure is the same tank dry mass.
        capacity = volume * HYDRAZINE_DENSITY
        dry = volume * TANK_DRY_DENSITY
        e["mass"] = clean(capacity + dry)
        e["radius"] = radius
        e["height"] = height
        e["capacity"] = {"hydrazine": clean(capacity)}
    elif ptype == "rcs":
        # RCS thruster: translation authority scales with the exit area
        # (r^2); the part is mostly structure + small thrusters, so a low
        # mass density (lighter than the reaction wheel).
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        e["rcs_thrust"] = clean(RCS_THRUST_PER_M2 * radius * radius)
    elif ptype == "decoupler":
        # staging boundary: mass is declared in EXTRA_FIELDS; radius/height
        # follow the mesh unless overridden there. The decoupler/fuel_barrier
        # flags land in the final EXTRA_FIELDS update below, keeping the key
        # order of the hand-written entries.
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
    if "fuel_rate" in e:
        t = 2.0 * e["fuel_rate"] * e["exhaust_velocity"]
        return "  %-24s T=%8.1fkN  rate=%7.2f  mass=%7s" % (
            n, t / 1e3, e["fuel_rate"], e["mass"])
    if "capacity" in e and "hydrogen" in e["capacity"]:
        c = e["capacity"]["hydrogen"] + e["capacity"]["lox"]
        return "  %-24s cap=%8skg  mass=%7s (dry %s)" % (
            n, c, e["mass"], clean(c * TANK_DRY_DENSITY / PROP_DENSITY))
    if "capacity" in e and "hydrazine" in e["capacity"]:
        c = e["capacity"]["hydrazine"]
        return "  %-24s cap=%8skg  mass=%7s (dry %s)" % (
            n, c, e["mass"], clean(c * TANK_DRY_DENSITY / HYDRAZINE_DENSITY))
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
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "res", "parts.json"),
                    help="output parts.json (default: res/parts.json)")
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
