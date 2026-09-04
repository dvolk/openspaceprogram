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
  engine          thrust   = ENGINE_THRUST_PER_M2 * radius^2  (exit area)
                  mass     = thrust * ENGINE_MASS_PER_N
                  fuel_rate= thrust / (2 * EXHAUST_VELOCITY)   (both tanks)
  capsule / wheel / adapter / nose_cap
                  mass     = volume * MASS_DENSITY[<type>]
                  capsule / wheel also carry attitude torque ~ radius
  extras (EXTRA_FIELDS)
                  crew seats + the kerbal's RCS propellant -- per-part
                  values that don't derive from geometry, applied on top

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

# structural mass per unit enclosed volume (kg/m^3); tuned so the base (r1)
# part of each kind lands on a sensible mass, then scales with real volume
MASS_DENSITY = {
    "capsule":        183.0,     # crew module (structure + life support)
    "reaction_wheel": 128.0,     # flywheel
    "adapter":        15.0,      # thin coupler ring, mostly air
    "nose_cap":       192.0,     # thin fairing
    "kerbal":         160.0,     # one crew member (mesh by gen_kerbal.py)
}

# attitude authority (N m), scales with radius (leverage of the wheel/arm)
CAPSULE_TORQUE_PER_M = 200.0
WHEEL_TORQUE_PER_M = 2000.0

# --- the catalog: (name, type, mesh, texture). Add a part = add a line. ----
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
    ("engine",           "engine",         "engine.obj",                   "engine.png"),
    ("engine_r1.5h3",    "engine",         "engine_r1.5h3.obj",            "engine.png"),
    ("engine_r2.25h4.5", "engine",         "engine_r2.25h4.5.obj",         "engine.png"),
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
    ("adapter_r1to1.5",  "adapter",        "adapter_r1to1.5.obj",          "adapter.png"),
    ("adapter_r1to2.25", "adapter",        "adapter_r1to2.25.obj",         "adapter.png"),
    ("adapter_r1.5to1",  "adapter",        "adapter_r1.5to1.obj",          "adapter.png"),
    ("adapter_r1.5to2.25","adapter",       "adapter_r1.5to2.25.obj",       "adapter.png"),
    ("adapter_r2.25to1", "adapter",        "adapter_r2.25to1.obj",         "adapter.png"),
    ("adapter_r2.25to1.5","adapter",       "adapter_r2.25to1.5.obj",       "adapter.png"),
    ("nose_cap",         "nose_cap",       "nose_cap.obj",                 "nose_cap.png"),
    ("nose_cap_r1.5h0.75","nose_cap",      "nose_cap_r1.5h0.75.obj",       "nose_cap.png"),
    ("nose_cap_r2.25h1.125","nose_cap",    "nose_cap_r2.25h1.125.obj",     "nose_cap.png"),
    ("kerbal",           "kerbal",         "kerbal.obj",                   "kerbal.png"),
]

# per-part extra fields that do NOT derive from the geometry: crew seats and
# the kerbal's RCS propellant. Applied on top of the generated entry so the
# catalog stays fully reproducible (no hand-edits to parts.json). The kerbal
# mass is declared (not mesh-derived) to preserve the hand-set value.
EXTRA_FIELDS = {
    "capsule":           {"crew_capacity": 1},
    "capsule_r1.5h3":    {"crew_capacity": 3},
    "capsule_r2.25h4.5": {"crew_capacity": 6},
    "kerbal":            {"mass": 97.05, "capacity": {"hydrazine": 10.0}},
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
    radius, height, volume = mesh_geom(mesh)
    e = {
        "name": name,
        "type": ptype,
        "mesh": mesh,
        "texture": texture,
    }

    if ptype == "engine":
        thrust = ENGINE_THRUST_PER_M2 * radius * radius
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
    else:  # capsule / reaction_wheel / adapter / nose_cap
        e["mass"] = clean(volume * MASS_DENSITY[ptype])
        e["radius"] = radius
        e["height"] = height
        if ptype == "capsule":
            e["torque"] = clean(CAPSULE_TORQUE_PER_M * radius)
        elif ptype == "reaction_wheel":
            e["torque"] = clean(WHEEL_TORQUE_PER_M * radius)

    e.update(EXTRA_FIELDS.get(name, {}))
    return e


def summary_line(e):
    n = e["name"]
    if "fuel_rate" in e:
        t = 2.0 * e["fuel_rate"] * e["exhaust_velocity"]
        return "  %-24s T=%8.1fkN  rate=%7.2f  mass=%7s" % (
            n, t / 1e3, e["fuel_rate"], e["mass"])
    if "capacity" in e and "hydrogen" in e["capacity"]:
        c = e["capacity"]["hydrogen"] + e["capacity"]["lox"]
        return "  %-24s cap=%8skg  mass=%7s (dry %s)" % (
            n, c, e["mass"], clean(c * TANK_DRY_DENSITY / PROP_DENSITY))
    tor = "  torque=%s" % e["torque"] if "torque" in e else ""
    return "  %-24s mass=%7s%s" % (n, e["mass"], tor)


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
