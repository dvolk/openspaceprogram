#!/usr/bin/env python3
# Generate res/systems/{old,ksp}_system.json. Angular speeds = 2*pi / period.
# Source of truth for the committed JSONs; --check verifies no drift.
# SOIs are NOT emitted: the loader derives the near-body shells and the
# inertial spheres from physics -- "soi_law": "patched_conic" reproduces the
# KSP wiki SOI values the old table hardcoded (src/bodylimits.h).
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sci_dist import stamp_system, phase0

TWOPI = 2.0 * math.pi
G = 6.674e-11
STAR_SOI = 1e18   # m; the root frame's inertial soi: the universe bound

ROOT = os.path.dirname(HERE)

def spd(period):
    if not period:
        return 0.0
    # Rates are magnitudes; retrograde lives in tilt/inclination (issue #139).
    # Fail at generation, not at game load (load_system rejects negatives).
    assert period > 0.0, "period must be positive; encode retrograde via tilt/inclination"
    return TWOPI / period

def true_anomaly_from_mean(M, e):
    """True anomaly at mean anomaly M (Newton-solve Kepler; e->0 gives nu=M)."""
    M = M % TWOPI
    E = M if e < 0.8 else math.pi
    for _ in range(30):
        step = (E - e * math.sin(E) - M) / (1.0 - e * math.cos(E))
        E -= step
        if abs(step) < 1e-13:
            break
    cosE, sinE = math.cos(E), math.sin(E)
    cosnu = (cosE - e) / (1.0 - e * cosE)
    sinnu = (math.sqrt(1.0 - e * e) * sinE) / (1.0 - e * cosE)
    return math.atan2(sinnu, cosnu) % TWOPI

def pos_at_longitude(lon, r):
    """inertial.pos for a body at rail LONGITUDE lon [rad], radius r [m].

    The loader reads an authored pos back as atan2(-z, x) (system.cpp, #146),
    which is why the z component carries the minus sign.
    """
    return [int(round(r * math.cos(lon))), 0, int(round(-r * math.sin(lon)))]

def load_wiki_orbits(csv_path):
    """Per-body orbital elements from ksp_bodies.csv."""
    import csv
    out = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            def num(key):
                v = (row.get(key) or "").strip()
                if v in ("", "nan", "inf", "-inf"):
                    return None
                return float(v)
            out[row["name"]] = {
                "e": num("eccentricity"),
                "i": math.radians(num("inclination_deg") or 0.0),
                "omega": math.radians(num("arg_periapsis_deg") or 0.0),
                "raan": math.radians(num("long_asc_node_deg") or 0.0),
                "M": num("mean_anomaly_rad"),
                "period": num("orbital_period_s"),
            }
    return out

WIKI_ORBITS = load_wiki_orbits(os.path.join(HERE, "ksp_bodies.csv"))

# Eerbon: legacy home + moon. Values from the pre-refactor setup_frames();
# seed 0 keeps the original unseeded terrain. Deliberately authored WITHOUT
# spin_phase0/tilt_azimuth (#141): a frozen legacy fixture, not a showcase
# for the new fields -- its spin stays node-locked by design. Also without
# "belts", which is why tests/test_belts.cpp uses it as the base for the
# optional-field and malformed-belt pins.
eerbon = {
    "home": "Eerbon",
    "soi_law": "patched_conic",
    "epoch_year": 1,   # fictional systems start at Year 1 (loader default; explicit)
    "skybox": "res/skybox/v1",   # a directory of skybox_{px,nx,py,ny,pz,nz}.png
    "bodies": [
        {
            "name": "Sun",
            "type": "star",
            "radius": 261600000,
            "mass": 1.757e28,
            "g": 17.131,
            "seed": 0,
            "has_sea": False,
            "power_scaler": 1,
            "inertial": {"soi": STAR_SOI, "pos": [0, 0, 0], "orb_ang_speed": 0.0},
        },
        {
            "name": "Eerbon",
            "type": "planet",
            "orbits": "Sun",
            "radius": 600000,
            "mass": 5.2915793e22,
            "g": 9.81,
            "seed": 0,
            "has_sea": True,
            "power_scaler": 3,
            "surface": {
                "atmosphere": {"color": [0.30, 0.50, 1.00], "thickness": 15000,
                               "power": 4.0, "intensity": 0.7},
            },
            "inertial": {
                "pos": [0, 0, -13599840260],
                "orb_ang_speed": 0.00000068269186570822291594437651,
            },
            "rotating": {
                "rot_ang_speed": 0.00029157090303706880702966723086,
                "axial_tilt": math.radians(23.4),
            },
        },
        {
            "name": "Moon",
            "type": "moon",
            "orbits": "Eerbon",
            "radius": 200000,
            "mass": 9.7600236e20,
            "g": 1.628,
            "seed": 0,
            "has_sea": False,
            "power_scaler": 1,
            "inertial": {
                "pos": [-12000000, 0, 0],
                "orb_ang_speed": 0.00004520797578987211820731369629,
                "orb_incl": math.radians(5.1),
            },
            "rotating": {
                "rot_ang_speed": 0.00004520785218583258404235991675,
                "axial_tilt": math.radians(6.7),
            },
        },
    ],
}

# KSP bodies leave sma/ecc/inc/orb_s as None (ksp_bodies.csv is source of truth).
# tilt_deg: axial tilt (0 = omit). phase_deg: non-KSP start angle (Shay=60 = L4).
# No soi column: the loader derives inertial SOIs with the patched_conic law
# (reproduces the wiki values within rounding) and near-body shells from the
# atmosphere -- see src/bodylimits.h.
K = [
    # name,     type,    orbits,  sma_m,        ecc,    mass_kg,  g,      radius_m, inc_deg, orb_s,      rot_s,      tilt_deg, has_sea, seed, ps [, phase_deg]
    ("Kerbol", "star",   None,    None,         None,   1.757e28, 17.131, 261600000, None,    None,       432000,    7.25,     False, 0.1, 1),
    ("Moho",   "planet", "Kerbol", None,        None,   2.526e21, 2.698,  250000,   None,    None,       1210000,   0.03,     False, 1,   3),
    ("Eve",    "planet", "Kerbol", None,        None,   1.224e23, 16.677, 700000,   None,    None,       80500,     2.64,     False, 2,   3),
    ("Gilly",  "moon",   "Eve",    None,        None,   1.242e17, 0.049,  13000,    None,    None,       28255,     1.2,      False, 3,   1),
    ("Kerbin", "planet", "Kerbol", None,        None,   5.292e22, 9.81,   600000,   None,    None,       21549,     23.44,    True,  1,   3),
    ("Mun",    "moon",   "Kerbin", None,        None,   9.760e20, 1.628,  200000,   None,    None,       138984,    6.68,     False, 5,   1),
    ("Minmus", "moon",   "Kerbin", None,        None,   2.646e19, 0.491,  60000,    None,    None,       40400,     12.0,     False, 6,   1),
    # Shay: Kerbin's L4 trojan (60 deg ahead). Period must match Kerbin's exactly.
    ("Shay",   "planet", "Kerbol", 13599840260, 0.0,    4.0762e22, 8.9933, 550000,   0.0,     9203544.6,  21549.425, 5.0,      True,  0,   3,  60),
    ("Duna",   "planet", "Kerbol", None,        None,   4.515e21, 2.943,  320000,   None,    None,       65518,     25.19,    False, 7,   3),
    ("Ike",    "moon",   "Duna",   None,        None,   2.782e20, 1.099,  130000,   None,    None,       65518,     1.76,     False, 8,   1),
    ("Dres",   "planet", "Kerbol", None,        None,   3.219e20, 1.128,  138000,   None,    None,       34800,     4.0,      False, 9,   3),
    ("Jool",   "planet", "Kerbol", None,        None,   4.233e24, 7.848,  6000000,  None,    None,       36000,     3.13,     False, 10,  3),
    ("Laythe", "moon",   "Jool",   None,        None,   2.940e22, 7.848,  500000,   None,    None,       52981,     0.5,      True,  11,  3),
    ("Vall",   "moon",   "Jool",   None,        None,   3.109e21, 2.305,  300000,   None,    None,       105962,    2.0,      False, 12,  1),
    ("Tylo",   "moon",   "Jool",   None,        None,   4.233e22, 7.848,  600000,   None,    None,       211926,    0.2,      False, 13,  3),
    ("Bop",    "moon",   "Jool",   None,        None,   3.726e19, 0.589,  65000,    None,    None,       544507,    25.0,     False, 14,  1),
    ("Pol",    "moon",   "Jool",   None,        None,   1.081e19, 0.373,  44000,    None,    None,       901903,    10.0,     False, 15,  1),
    ("Eeloo",  "planet", "Kerbol", None,        None,   1.115e21, 1.687,  210000,   None,    None,       19460,     122.5,    False, 16,  3),
]

# Optional "surface" block: palette (elevation 0..1 -> color), sea_*/,
# amplitude, bands/* for gas giants.
SURFACES = {
    "Kerbol": {
        "palette": [[0.0, [1.00, 0.80, 0.35]], [1.0, [1.00, 1.00, 0.75]]],
    },
    "Moho": {
        "amplitude": 800,
        "palette": [[0.0, [0.35, 0.22, 0.20]],
                    [0.6, [0.50, 0.32, 0.30]],
                    [1.0, [0.65, 0.45, 0.42]]],
    },
    "Eve": {
        "amplitude": 5000,
        "persistence": 0.6,
        "frequency": 1.1,
        "palette": [[0.0, [0.45, 0.08, 0.20]],
                    [0.5, [0.65, 0.15, 0.30]],
                    [1.0, [0.80, 0.40, 0.45]]],
        "atmosphere": {"color": [0.80, 0.85, 0.35], "thickness": 25000,
                       "power": 4.0, "intensity": 0.75,
                       "sea_level_density": 1.7, "scale_height": 7000,
                       "height": 90000},
    },
    "Gilly": {
        "amplitude": 400,
        "palette": [[0.0, [0.20, 0.16, 0.32]],
                    [0.7, [0.40, 0.28, 0.45]],
                    [1.0, [0.60, 0.45, 0.50]]],
    },
    "Kerbin": {
        "amplitude": 5000,
        "persistence": 0.6,
        "frequency": 1.1,
        "sea_color": [0.20, 0.45, 0.65],
        "palette": [[0.0, [0.13, 0.45, 0.13]],
                    [0.45, [0.45, 0.55, 0.20]],
                    [0.8, [0.55, 0.45, 0.35]],
                    [1.0, [1.00, 1.00, 1.00]]],
        "atmosphere": {"color": [0.30, 0.50, 1.00], "thickness": 15000,
                       "power": 4.0, "intensity": 0.7,
                       "sea_level_density": 1.225, "scale_height": 5500,
                       "height": 70000},
        "clouds": {"height": 2500, "coverage": 0.6, "freq": 10,
                   "drift": 0.00001},
    },
    "Mun": {
        "amplitude": 2500,
        "persistence": 0.7,
        "frequency": 1.2,
        "palette": [[0.0, [0.35, 0.35, 0.36]],
                    [1.0, [0.65, 0.65, 0.67]]],
    },
    "Minmus": {
        "amplitude": 2000,
        "palette": [[0.0, [0.28, 0.48, 0.55]],
                    [1.0, [0.70, 0.90, 0.90]]],
    },
    "Shay": {
        "amplitude": 3000,
        "persistence": 0.55,
        "frequency": 1.1,
        "atmosphere": {"color": [0.40, 0.65, 0.70], "thickness": 22000,
                       "power": 4.0, "intensity": 0.7,
                       "sea_level_density": 1.225, "scale_height": 6000,
                       "height": 60000},
        "clouds": {"height": 3000, "coverage": 0.5, "freq": 10,
                   "drift": 0.00001},
    },
    "Duna": {
        "palette": [[0.0, [0.55, 0.22, 0.08]],
                    [0.6, [0.70, 0.35, 0.15]],
                    [1.0, [0.85, 0.60, 0.40]]],
        "atmosphere": {"color": [0.80, 0.48, 0.30], "thickness": 8000,
                       "power": 4.0, "intensity": 0.55,
                       "sea_level_density": 0.12, "scale_height": 4000,
                       "height": 50000},
    },
    "Ike": {
        "amplitude": 1000,
        "palette": [[0.0, [0.45, 0.35, 0.30]],
                    [1.0, [0.70, 0.55, 0.45]]],
    },
    "Dres": {
        "amplitude": 2000,
        "palette": [[0.0, [0.40, 0.15, 0.10]],
                    [1.0, [0.65, 0.35, 0.25]]],
    },
    "Jool": {
        "bands": True,
        "band_count": 9,
        "palette": [[0.0, [0.30, 0.42, 0.45]],
                    [1.0, [0.80, 0.87, 0.83]]],
        "atmosphere": {"color": [0.75, 0.85, 0.85], "thickness": 90000,
                       "power": 3.0, "intensity": 0.6,
                       "sea_level_density": 2.0, "scale_height": 20000,
                       "height": 200000},
    },
    "Laythe": {
        "amplitude": 5000,
        "octaves": 10,
        "persistence": 0.55,
        "frequency": 1.1,
        "sea_color": [0.00, 0.40, 0.45],
        "palette": [[0.0, [0.25, 0.55, 0.25]],
                    [0.7, [0.50, 0.60, 0.35]],
                    [1.0, [1.00, 1.00, 1.00]]],
        "atmosphere": {"color": [0.30, 0.55, 0.90], "thickness": 12000,
                       "power": 4.0, "intensity": 0.7,
                       "sea_level_density": 1.225, "scale_height": 5500,
                       "height": 50000},
        "clouds": {"height": 2000, "coverage": 0.55, "freq": 10,
                   "drift": 0.00001},
    },
    "Vall": {
        "amplitude": 2000,
        "palette": [[0.0, [0.60, 0.20, 0.25]],
                    [1.0, [0.85, 0.70, 0.70]]],
    },
    "Tylo": {
        "palette": [[0.0, [0.45, 0.35, 0.25]],
                    [1.0, [0.75, 0.65, 0.50]]],
    },
    "Bop": {
        "amplitude": 800,
        "palette": [[0.0, [0.30, 0.30, 0.50]],
                    [1.0, [0.60, 0.60, 0.75]]],
    },
    "Pol": {
        "amplitude": 600,
        "palette": [[0.0, [0.30, 0.45, 0.55]],
                    [1.0, [0.70, 0.80, 0.85]]],
    },
    "Eeloo": {
        "amplitude": 5000,
        "palette": [[0.0, [0.60, 0.30, 0.20]],
                    [1.0, [0.80, 0.50, 0.45]]],
    },
}


def ksp_body(name, typ, orbits, sma, ecc, mass, g, radius, inc_deg, orb_s, rot_s, tilt_deg, has_sea, seed, ps, phase_deg=0):
    b = {
        "name": name,
        "type": typ,
    }
    if orbits:
        b["orbits"] = orbits
    b["radius"] = radius
    b["mass"] = mass
    b["g"] = g
    b["seed"] = seed
    b["has_sea"] = has_sea
    b["power_scaler"] = ps
    if name in SURFACES:
        b["surface"] = SURFACES[name]

    if typ == "star":
        b["inertial"] = {"soi": STAR_SOI, "pos": [0, 0, 0], "orb_ang_speed": 0.0}
    else:
        wiki = WIKI_ORBITS.get(name)
        if wiki and wiki.get("period") and orbits in MASS:
            # KSP body: start at the real wiki epoch position (e, i, omega,
            # raan, M). load_system derives a from orb_ang_speed via Kepler III.
            e = wiki["e"] or 0.0
            i = wiki["i"] or 0.0
            omega = wiki["omega"] or 0.0
            raan = wiki["raan"] or 0.0
            M = wiki["M"] or 0.0
            w = spd(wiki["period"])
            nu = true_anomaly_from_mean(M, e)
            inertial = {
                "orb_ang_speed": w,
                "arg_peri": omega,
                "true_anomaly0": nu,
            }
            if i:
                inertial["orb_incl"] = i
            if raan:
                inertial["lon_asc_node"] = raan
            if e:
                inertial["ecc"] = e
            b["inertial"] = inertial
        else:
            # Non-wiki body (Shay): Kerbin's L4 trojan, phase_deg AHEAD of
            # Kerbin along its direction of motion (increasing longitude).
            kb = WIKI_ORBITS.get("Kerbin")
            if phase_deg and kb and kb.get("period"):
                kb_lon = (kb["raan"] or 0.0) + (kb["omega"] or 0.0) \
                    + true_anomaly_from_mean(kb["M"] or 0.0, kb["e"] or 0.0)
                pos = pos_at_longitude(kb_lon + math.radians(phase_deg), sma)
            elif phase_deg:
                pos = pos_at_longitude(math.radians(phase_deg), sma)
            elif typ == "planet":
                pos = [0, 0, -sma]              # longitude +90 deg
            else:
                pos = [-sma, 0, 0]              # longitude 180 deg
            inertial = {
                "pos": pos,
                "orb_ang_speed": spd(orb_s),
            }
            if inc_deg:
                inertial["orb_incl"] = math.radians(inc_deg)
            b["inertial"] = inertial
        rotating = {
            "rot_ang_speed": spd(rot_s),
            # #141: seeded-random epoch spin phase (see make_solar_system.py);
            # tilt_azimuth frees the obliquity node from the ascending node.
            "spin_phase0": phase0(name, "spin"),
        }
        # Axial tilt: lean the spin axis from the orbital normal (0 = omit).
        if tilt_deg:
            rotating["axial_tilt"] = math.radians(tilt_deg)
            rotating["tilt_azimuth"] = phase0(name, "tilt")
        b["rotating"] = rotating
    return b

# Parent masses for the eccentric-orbit fit in ksp_body (Kepler's third law).
MASS = {row[0]: row[5] for row in K}

ksp = {
    "home": "Kerbin",
    "soi_law": "patched_conic",
    "epoch_year": 1,   # fictional systems start at Year 1 (loader default; explicit)
    "skybox": "res/skybox/v1",   # a directory of skybox_{px,nx,py,ny,pz,nz}.png
    # Debris belts (root "belts", src/system.cpp): named annuli around Kerbol,
    # drawn on the Tracking map. Same entry shape as a body's surface.rings
    # band. Radii [m] from Kerbol, placed against THIS system's planets rather
    # than scaled from the real Solar System's AU numbers -- Kerbol's system is
    # far more compressed than ours, so a naive AU scaling lands the Kuiper
    # band 4.5x beyond Eeloo, where nothing is.
    #   asteroid belt: the Duna..Dres gap (Duna apoapsis 21.8 Gm, Dres
    #     periapsis 34.9 Gm), so it sits between the last rocky planet and the
    #     belt dwarf rather than swallowing Dres.
    #   Kuiper belt: starts just inside Eeloo's apoapsis (113.6 Gm) and reaches
    #     past it, so Eeloo's 66.7..113.6 Gm ellipse grazes the inner edge the
    #     way Neptune sits on our Kuiper edge with Pluto dipping in. A band
    #     starting well inside the outermost orbit (72 Gm, just past Jool)
    #     reads as that orbit's own neighbourhood, not the far edge of the
    #     system. tests/test_belts.cpp pins the graze.
    "belts": [
        {"name": "asteroid belt", "inner": 2.4e10, "outer": 3.3e10},
        {"name": "Kuiper belt", "inner": 1.08e11, "outer": 1.8e11},
    ],
    "bodies": [
        ksp_body(*row) for row in K
    ],
}
# Science fields at creation (sci_dist.annotate remains for the CLI).
stamp_system(eerbon)
stamp_system(ksp)

# science_mult exceptions: the committed JSON is generated output, so a
# body that should sit off the computed default is overridden HERE, not by
# editing the JSON afterwards. Shay: the co-orbital drift dv (sci_phase.py)
# computes 1.3, but the design intent is a Duna-class interplanetary
# destination (issue #128).
KSP_SCIENCE_MULT_OVERRIDES = {"Shay": 1.4}
for _name, _m in KSP_SCIENCE_MULT_OVERRIDES.items():
    _b = next(b for b in ksp["bodies"] if b["name"] == _name)
    _b["science_mult"] = _m

def render(obj):
    import json as _json
    return _json.dumps(obj, indent=2) + "\n"

def write(obj, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(render(obj))
    print("wrote", path)

def deep_diff(a, b, path, out):
    """Value-level diff (a = committed, b = generated); numeric compare ignores float spelling.
    science_mult / transfer_dv are INCLUDED: the committed JSON is generated
    output, and off-model values (e.g. Shay's science_mult) are overrides in
    this script -- so any JSON drift here is real drift and must fail."""
    if isinstance(a, dict) and isinstance(b, dict):
        for k in a:
            p = f"{path}.{k}" if path else k
            if k not in b:
                out.append(f"{p}: in committed file, missing from generated ({a[k]!r})")
            else:
                deep_diff(a[k], b[k], p, out)
        for k in b:
            p = f"{path}.{k}" if path else k
            if k not in a:
                out.append(f"{p}: in generated, missing from committed file ({b[k]!r})")
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append(f"{path}: list length committed={len(a)} generated={len(b)}")
        for i in range(min(len(a), len(b))):
            deep_diff(a[i], b[i], f"{path}[{i}]", out)
    else:
        if a != b:
            out.append(f"{path}: committed={a!r} generated={b!r}")

def check(obj, path):
    import json as _json
    if not os.path.exists(path):
        print(f"MISSING {path}")
        return False
    with open(path) as f:
        committed = _json.load(f)
    out = []
    if committed.get("home") != obj.get("home"):
        out.append(f"home: committed={committed.get('home')!r} generated={obj.get('home')!r}")
    if committed.get("skybox") != obj.get("skybox"):
        out.append(f"skybox: committed={committed.get('skybox')!r} generated={obj.get('skybox')!r}")
    if committed.get("belts") != obj.get("belts"):
        out.append(f"belts: committed={committed.get('belts')!r} generated={obj.get('belts')!r}")
    cb = {x["name"]: x for x in committed.get("bodies", [])}
    gb = {x["name"]: x for x in obj.get("bodies", [])}
    for name in cb:
        if name not in gb:
            out.append(f"body {name}: in committed file, missing from generated")
    for name in gb:
        if name not in cb:
            out.append(f"body {name}: in generated, missing from committed file")
    for name in cb:
        if name in gb:
            body_out = []
            deep_diff(cb[name], gb[name], "", body_out)
            for line in body_out:
                out.append(f"{name}: {line}")
    if out:
        print(f"DRIFT   {path}:")
        for line in out:
            print(f"    {line}")
        return False
    print(f"OK      {path}")
    return True

def main():
    ap = argparse.ArgumentParser(
        description="Generate res/systems/{old,ksp}_system.json, or --check "
                    "that the committed files match this script.")
    ap.add_argument("--check", action="store_true",
                    help="verify the committed JSONs still match, without "
                         "writing anything; exit non-zero on any drift")
    args = ap.parse_args()
    targets = [
        (eerbon, os.path.join(ROOT, "res", "systems", "old_system.json")),
        (ksp, os.path.join(ROOT, "res", "systems", "ksp_system.json")),
    ]
    if args.check:
        ok = True
        for obj, path in targets:
            ok = check(obj, path) and ok
        raise SystemExit(0 if ok else 1)
    for obj, path in targets:
        write(obj, path)

main()
