#!/usr/bin/env python3
# Generate system.json (Eerbon) and ksp_system.json (Kerbal) for the
# refactored load_system() JSON format. Angular speeds are derived from the
# CSV orbital / rotational periods: speed = 2*pi / period.
import math

TWOPI = 2.0 * math.pi

def spd(period):
    if not period:
        return 0.0
    return TWOPI / period

# ---------------------------------------------------------------------------
# Eerbon system (single home planet + one moon) - values taken verbatim from
# the pre-refactor setup_frames() / TerrainBody creation in src/main.cpp.
# Seeds are 0 = the legacy (unseeded) noise pattern, so Eerbon keeps exactly
# the terrain it always had.
# ---------------------------------------------------------------------------
eerbon = {
    "home": "Eerbon",
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
            "moves": False,
            "inertial": {"soi": 1e16, "pos": [0, 0, 0], "orb_ang_speed": 0.0},
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
            "moves": False,
            "inertial": {
                "soi": 84159286,
                "pos": [0, 0, -13599840260],
                "orb_ang_speed": 0.00000068269186570822291594437651,
                # coplanar with the Sun's reference plane
            },
            "rotating": {
                "soi": 700000,
                "rot_ang_speed": 0.00029157090303706880702966723086,
                # Earth-like obliquity: spin axis 23.4 deg off the orbit normal
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
            "moves": True,
            "inertial": {
                "soi": 2429559.1,
                "pos": [-12000000, 0, 0],
                "orb_ang_speed": 0.00004520797578987211820731369629,
                # real-Moon-style 5.1 deg tilt of its orbit about Eerbon's
                # orbital plane
                "orb_incl": math.radians(5.1),
            },
            "rotating": {
                "soi": 300000,
                "rot_ang_speed": 0.00004520785218583258404235991675,
                # the real Moon's obliquity to its orbit is ~6.7 deg
                "axial_tilt": math.radians(6.7),
            },
        },
    ],
}

# ---------------------------------------------------------------------------
# Kerbal system (ksp_system.csv). Fields:
#   name, type, orbits, sma_m, mass_kg, g, radius_m, inc_deg,
#   orb_period_s, rot_period_s, soi_m, has_sea, seed, power_scaler
# Position convention (matches the Eerbon data):
#   planets -> [0, 0, -sma]
#   moons   -> [-sma, 0, 0]
# inc_deg: orbital inclination from the CSV Inc. column (KSP wiki values),
#   emitted as inertial.orb_incl in radians; 0 = coplanar (field omitted).
# Rotating-frame SOI (near-body boundary) = radius + 100 km, the same rule the
# Eerbon data follows (Eerbon 600km+100km=700km, Moon 200km+100km=300km).
# ---------------------------------------------------------------------------
K = [
    # name,     type,    orbits,  sma_m,        mass_kg,  g,      radius_m, inc_deg, orb_s,      rot_s,      soi_m,        has_sea, seed, ps
    ("Kerbol", "star",   None,    0,            1.757e28, 17.131, 261600000, 0.0,     None,       432000,    1e16,         False, 0.1, 1),
    ("Moho",   "planet", "Kerbol", 5263138300,  2.526e21, 2.698,  250000,   7.0,     2215754.2,  1210000,   9646660,      False, 1,   3),
    ("Eve",    "planet", "Kerbol", 9832684540,  1.224e23, 16.677, 700000,   2.1,     5657995.1,  80500,     85109360,     False, 2,   3),
    ("Gilly",  "moon",   "Eve",    31500000,    1.242e17, 0.049,  13000,    12.0,    388587.4,   28255,     126120,       False, 3,   1),
    ("Kerbin", "planet", "Kerbol", 13599840260, 5.292e22, 9.81,   600000,   0.0,     9203544.6,  21549,     84159290,     True,  1,   3),
    ("Mun",    "moon",   "Kerbin", 12000000,    9.760e20, 1.628,  200000,   0.0,     138984.4,   138984,    2429560,      False, 5,   1),
    ("Minmus", "moon",   "Kerbin", 47000000,    2.646e19, 0.491,  60000,    6.0,     1077310.5,  40400,     2247430,      False, 6,   1),
    ("Duna",   "planet", "Kerbol", 20726155260, 4.515e21, 2.943,  320000,   0.06,    17315400.1, 65518,     47921950,     False, 7,   3),
    ("Ike",    "moon",   "Duna",   3200000,     2.782e20, 1.099,  130000,   0.2,     65517.9,    65518,     1049600,      False, 8,   1),
    ("Dres",   "planet", "Kerbol", 40839348200, 3.219e20, 1.128,  138000,   5.0,     47893063.1, 34800,     32832840,     False, 9,   3),
    ("Jool",   "planet", "Kerbol", 68773560320, 4.233e24, 7.848,  6000000,  1.304,   104661432.1,36000,     2455985190,   False, 10,  3),
    ("Laythe", "moon",   "Jool",   27184000,    2.940e22, 7.848,  500000,   0.0,     52980.9,    52981,     3723650,      True,  11,  3),
    ("Vall",   "moon",   "Jool",   43152000,    3.109e21, 2.305,  300000,   0.0,     105962.1,   105962,    2406400,      False, 12,  1),
    ("Tylo",   "moon",   "Jool",   68500000,    4.233e22, 7.848,  600000,   0.025,   211926.4,   211926,    10856520,     False, 13,  3),
    ("Bop",    "moon",   "Jool",   128500000,   3.726e19, 0.589,  65000,    15.0,    544507.4,   544507,    1221060,      False, 14,  1),
    ("Pol",    "moon",   "Jool",   179890000,   1.081e19, 0.373,  44000,    4.25,    901902.6,   901903,    1042140,      False, 15,  1),
    ("Eeloo",  "planet", "Kerbol", 90118820000, 1.115e21, 1.687,  210000,   6.15,    156992048.4,19460,     119082940,    False, 16,  3),
]

# ---------------------------------------------------------------------------
# Per-body surface appearance (the optional "surface" JSON block):
#   palette:  land-color stops [elevation 0..1, [r,g,b]], lerped by altitude
#   sea_color / sea_level: ocean tint + level (m above base radius)
#   amplitude: peak terrain noise height [m]
#   bands/band_count: gas giant — smooth sphere colored by latitude stripes
# Colors are hand-picked KSP-flavored approximations.
# ---------------------------------------------------------------------------
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
        "amplitude": 3000,
        "palette": [[0.0, [0.45, 0.08, 0.20]],
                    [0.5, [0.65, 0.15, 0.30]],
                    [1.0, [0.80, 0.40, 0.45]]],
    },
    "Gilly": {
        "amplitude": 400,
        "palette": [[0.0, [0.20, 0.16, 0.32]],
                    [0.7, [0.40, 0.28, 0.45]],
                    [1.0, [0.60, 0.45, 0.50]]],
    },
    "Kerbin": {
        "sea_color": [0.00, 0.35, 0.75],
        "palette": [[0.0, [0.13, 0.45, 0.13]],
                    [0.45, [0.45, 0.55, 0.20]],
                    [0.8, [0.55, 0.45, 0.35]],
                    [1.0, [1.00, 1.00, 1.00]]],
    },
    "Mun": {
        "amplitude": 1500,
        "palette": [[0.0, [0.35, 0.35, 0.36]],
                    [1.0, [0.65, 0.65, 0.67]]],
    },
    "Minmus": {
        "amplitude": 2000,
        "palette": [[0.0, [0.28, 0.48, 0.55]],
                    [1.0, [0.70, 0.90, 0.90]]],
    },
    "Duna": {
        "palette": [[0.0, [0.55, 0.22, 0.08]],
                    [0.6, [0.70, 0.35, 0.15]],
                    [1.0, [0.85, 0.60, 0.40]]],
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
    },
    "Laythe": {
        "amplitude": 3000,
        "sea_color": [0.00, 0.40, 0.45],
        "palette": [[0.0, [0.25, 0.55, 0.25]],
                    [0.7, [0.50, 0.60, 0.35]],
                    [1.0, [1.00, 1.00, 1.00]]],
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


def ksp_body(name, typ, orbits, sma, mass, g, radius, inc_deg, orb_s, rot_s, soi, has_sea, seed, ps):
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
    b["moves"] = False

    if typ == "star":
        b["inertial"] = {"soi": soi, "pos": [0, 0, 0], "orb_ang_speed": 0.0}
    else:
        if typ == "planet":
            pos = [0, 0, -sma]
        else:
            pos = [-sma, 0, 0]
        inertial = {
            "soi": soi,
            "pos": pos,
            "orb_ang_speed": spd(orb_s),
        }
        # Orbital inclination (KSP wiki Inc. values) — tilt of the orbital
        # plane about the parent's +X (line of nodes); 0 = coplanar (omitted).
        if inc_deg:
            inertial["orb_incl"] = math.radians(inc_deg)
        b["inertial"] = inertial
        # Every planet / moon spins; near-body SOI = radius + 100 km.
        b["rotating"] = {
            "soi": radius + 100000,
            "rot_ang_speed": spd(rot_s),
        }
    return b

ksp = {
    "home": "Kerbin",
    "bodies": [
        ksp_body(*row) for row in K
    ],
}

def emit(obj, path):
    import json as _json
    with open(path, "w") as f:
        _json.dump(obj, f, indent=2)
        f.write("\n")
    print("wrote", path)

emit(eerbon, "/home/ubuntu/openspaceprogram/system.json")
emit(ksp, "/home/ubuntu/openspaceprogram/ksp_system.json")
