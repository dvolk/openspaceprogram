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
            "seed": 0.1,
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
            "seed": 1,
            "has_sea": True,
            "power_scaler": 3,
            "moves": False,
            "inertial": {
                "soi": 84159286,
                "pos": [0, 0, -13599840260],
                "orb_ang_speed": 0.00000068269186570822291594437651,
            },
            "rotating": {
                "soi": 700000,
                "rot_ang_speed": 0.00029157090303706880702966723086,
            },
        },
        {
            "name": "Moon",
            "type": "moon",
            "orbits": "Eerbon",
            "radius": 200000,
            "mass": 9.7600236e20,
            "g": 1.628,
            "seed": 0.1,
            "has_sea": False,
            "power_scaler": 1,
            "moves": True,
            "inertial": {
                "soi": 2429559.1,
                "pos": [-12000000, 0, 0],
                "orb_ang_speed": 0.00004520797578987211820731369629,
            },
            "rotating": {
                "soi": 300000,
                "rot_ang_speed": 0.00004520785218583258404235991675,
            },
        },
    ],
}

# ---------------------------------------------------------------------------
# Kerbal system (ksp_system.csv). Fields:
#   name, type, orbits, sma_m, mass_kg, g, radius_m,
#   orb_period_s, rot_period_s, soi_m, has_sea, seed, power_scaler
# Position convention (matches the Eerbon data):
#   planets -> [0, 0, -sma]
#   moons   -> [-sma, 0, 0]
# Rotating-frame SOI (near-body boundary) = radius + 100 km, the same rule the
# Eerbon data follows (Eerbon 600km+100km=700km, Moon 200km+100km=300km).
# ---------------------------------------------------------------------------
K = [
    # name,     type,    orbits,  sma_m,        mass_kg,  g,      radius_m, orb_s,      rot_s,      soi_m,        has_sea, seed, ps
    ("Kerbol", "star",   None,    0,            1.757e28, 17.131, 261600000, None,       432000,    1e16,         False, 0.1, 1),
    ("Moho",   "planet", "Kerbol", 5263138300,  2.526e21, 2.698,  250000,   2215754.2,  1210000,   9646660,      False, 1,   3),
    ("Eve",    "planet", "Kerbol", 9832684540,  1.224e23, 16.677, 700000,   5657995.1,  80500,     85109360,     False, 2,   3),
    ("Gilly",  "moon",   "Eve",    31500000,    1.242e17, 0.049,  13000,    388587.4,   28255,     126120,       False, 3,   1),
    ("Kerbin", "planet", "Kerbol", 13599840260, 5.292e22, 9.81,   600000,   9203544.6,  21549,     84159290,     True,  1,   3),
    ("Mun",    "moon",   "Kerbin", 12000000,    9.760e20, 1.628,  200000,   138984.4,   138984,    2429560,      False, 5,   1),
    ("Minmus", "moon",   "Kerbin", 47000000,    2.646e19, 0.491,  60000,    1077310.5,  40400,     2247430,      False, 6,   1),
    ("Duna",   "planet", "Kerbol", 20726155260, 4.515e21, 2.943,  320000,   17315400.1, 65518,     47921950,     False, 7,   3),
    ("Ike",    "moon",   "Duna",   3200000,     2.782e20, 1.099,  130000,   65517.9,    65518,     1049600,      False, 8,   1),
    ("Dres",   "planet", "Kerbol", 40839348200, 3.219e20, 1.128,  138000,   47893063.1, 34800,     32832840,     False, 9,   3),
    ("Jool",   "planet", "Kerbol", 68773560320, 4.233e24, 7.848,  6000000,  104661432.1,36000,     2455985190,   False, 10,  3),
    ("Laythe", "moon",   "Jool",   27184000,    2.940e22, 7.848,  500000,   52980.9,    52981,     3723650,      True,  11,  3),
    ("Vall",   "moon",   "Jool",   43152000,    3.109e21, 2.305,  300000,   105962.1,   105962,    2406400,      False, 12,  1),
    ("Tylo",   "moon",   "Jool",   68500000,    4.233e22, 7.848,  600000,   211926.4,   211926,    10856520,     False, 13,  3),
    ("Bop",    "moon",   "Jool",   128500000,   3.726e19, 0.589,  65000,    544507.4,   544507,    1221060,      False, 14,  1),
    ("Pol",    "moon",   "Jool",   179890000,   1.081e19, 0.373,  44000,    901902.6,   901903,    1042140,      False, 15,  1),
    ("Eeloo",  "planet", "Kerbol", 90118820000, 1.115e21, 1.687,  210000,   156992048.4,19460,     119082940,    False, 16,  3),
]

def ksp_body(name, typ, orbits, sma, mass, g, radius, orb_s, rot_s, soi, has_sea, seed, ps):
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
    b["moves"] = False

    if typ == "star":
        b["inertial"] = {"soi": soi, "pos": [0, 0, 0], "orb_ang_speed": 0.0}
    else:
        if typ == "planet":
            pos = [0, 0, -sma]
        else:
            pos = [-sma, 0, 0]
        b["inertial"] = {
            "soi": soi,
            "pos": pos,
            "orb_ang_speed": spd(orb_s),
        }
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
