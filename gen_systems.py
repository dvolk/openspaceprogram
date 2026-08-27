#!/usr/bin/env python3
# Generate system.json (Eerbon) and ksp_system.json (Kerbal) for the
# refactored load_system() JSON format. Angular speeds are derived from the
# CSV orbital / rotational periods: speed = 2*pi / period.
import math

TWOPI = 2.0 * math.pi
G = 6.674e-11

def spd(period):
    if not period:
        return 0.0
    return TWOPI / period

def true_anomaly_from_mean(M, e):
    """True anomaly nu at mean anomaly M: Newton-solve Kepler's equation
    M = E - e sin E for the eccentric anomaly E, then convert. For e -> 0
    this is exactly nu = M."""
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

def load_wiki_orbits(csv_path):
    """Per-body orbital elements from ksp_wiki_bodies.csv (the individual
    KSP wiki pages): eccentricity, inclination, argument of periapsis (w),
    longitude of the ascending node (raan), mean anomaly at epoch (M) and
    the orbital period. Used to place each body at its real KSP starting
    position instead of the old shared-axis layout."""
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

WIKI_ORBITS = load_wiki_orbits(
    "/home/ubuntu/openspaceprogram/ksp_wiki_bodies.csv")

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
            # N2/O2 rim (see reports/atmosphere2026_08_25)
            "surface": {
                "atmosphere": {"color": [0.30, 0.50, 1.00], "thickness": 15000,
                               "power": 4.0, "intensity": 0.7},
            },
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
#   name, type, orbits, sma_m, ecc, mass_kg, g, radius_m, inc_deg,
#   orb_period_s, rot_period_s, soi_m, has_sea, seed, power_scaler
# Position convention (matches the Eerbon data):
#   planets -> [0, 0, -sma]
#   moons   -> [-sma, 0, 0]
# phase_deg (optional, last column): rotate that starting point about the
#   orbit normal; 0 = the defaults above. Eden = 60: Kerbin's L4 slot,
#   60 deg ahead on the same circle.
# inc_deg: orbital inclination from the CSV Inc. column (KSP wiki values),
#   emitted as inertial.orb_incl in radians; 0 = coplanar (field omitted).
# ecc: eccentricity from the CSV Ecc. column. Emitted together with
#   arg_peri / true_anomaly0 fitted so the body starts EXACTLY where it did
#   before (radius = |pos|, same direction): r(nu0) = |pos| gives
#   cos(nu0) = (a(1-e^2)/|pos| - 1)/e (outbound branch), periapsis at
#   arg_peri = pos_angle - nu0. The dynamically consistent a (Kepler's third
#   law with the parent's rounded mass) differs from the CSV sma by < 0.01%,
#   so the fit uses a, and the speed at the start is the old circular speed
#   up to the same < 0.01%; the velocity direction tilts (outbound).
#   Nothing jumps on load. 0 = circular (fields omitted).
# tilt_deg: axial tilt from the CSV Axial tilt column, emitted as
#   rotating.axial_tilt in radians; 0 = pole on the orbit normal (omitted).
#   Values follow the real-solar-system equivalents (Sun 7.25, Earth 23.44,
#   Pluto 122.5, ...); bodies without a real analog get plausible small values.
# Rotating-frame SOI (near-body boundary) = radius + 100 km, the same rule the
# Eerbon data follows (Eerbon 600km+100km=700km, Moon 200km+100km=300km).
# ---------------------------------------------------------------------------
K = [
    # name,     type,    orbits,  sma_m,        ecc,    mass_kg,  g,      radius_m, inc_deg, orb_s,      rot_s,      tilt_deg, soi_m,        has_sea, seed, ps [, phase_deg]
    ("Kerbol", "star",   None,    0,            0.0,    1.757e28, 17.131, 261600000, 0.0,     None,       432000,    7.25,     1e16,         False, 0.1, 1),
    ("Moho",   "planet", "Kerbol", 5263138300,  0.2,    2.526e21, 2.698,  250000,   7.0,     2215754.2,  1210000,   0.03,     9646660,      False, 1,   3),
    ("Eve",    "planet", "Kerbol", 9832684540,  0.01,   1.224e23, 16.677, 700000,   2.1,     5657995.1,  80500,     2.64,     85109360,     False, 2,   3),
    ("Gilly",  "moon",   "Eve",    31500000,    0.55,   1.242e17, 0.049,  13000,    12.0,    388587.4,   28255,     1.2,      126120,       False, 3,   1),
    ("Kerbin", "planet", "Kerbol", 13599840260, 0.0,    5.292e22, 9.81,   600000,   0.0,     9203544.6,  21549,     23.44,    84159290,     True,  1,   3),
    ("Mun",    "moon",   "Kerbin", 12000000,    0.0,    9.760e20, 1.628,  200000,   0.0,     138984.4,   138984,    6.68,     2429560,      False, 5,   1),
    ("Minmus", "moon",   "Kerbin", 47000000,    0.0,    2.646e19, 0.491,  60000,    6.0,     1077310.5,  40400,     12.0,     2247430,      False, 6,   1),
    # Eden: Kerbin's L4 trojan, 60 deg ahead on the same circle. Its orbital
    # period is Kerbin's exactly -- anything else and it drifts off L4.
    # Day = 21549.425 s: Eerbon's day, ported verbatim with the rest of it.
    ("Eden",   "planet", "Kerbol", 13599840260, 0.0,    8.4035e22, 11.446, 700000,   0.0,     9203544.6,  21549.425, 5.0,      98185838,     True,  0,   3,  60),
    ("Duna",   "planet", "Kerbol", 20726155260, 0.05,   4.515e21, 2.943,  320000,   0.06,    17315400.1, 65518,     25.19,    47921950,     False, 7,   3),
    ("Ike",    "moon",   "Duna",   3200000,     0.03,   2.782e20, 1.099,  130000,   0.2,     65517.9,    65518,     1.76,     1049600,      False, 8,   1),
    ("Dres",   "planet", "Kerbol", 40839348200, 0.15,   3.219e20, 1.128,  138000,   5.0,     47893063.1, 34800,     4.0,      32832840,     False, 9,   3),
    ("Jool",   "planet", "Kerbol", 68773560320, 0.05,   4.233e24, 7.848,  6000000,  1.304,   104661432.1,36000,     3.13,     2455985190,   False, 10,  3),
    ("Laythe", "moon",   "Jool",   27184000,    0.0,    2.940e22, 7.848,  500000,   0.0,     52980.9,    52981,     0.5,      3723650,      True,  11,  3),
    ("Vall",   "moon",   "Jool",   43152000,    0.0,    3.109e21, 2.305,  300000,   0.0,     105962.1,   105962,    2.0,      2406400,      False, 12,  1),
    ("Tylo",   "moon",   "Jool",   68500000,    0.0,    4.233e22, 7.848,  600000,   0.025,   211926.4,   211926,    0.2,      10856520,     False, 13,  3),
    ("Bop",    "moon",   "Jool",   128500000,   0.23,   3.726e19, 0.589,  65000,    15.0,    544507.4,   544507,    25.0,     1221060,      False, 14,  1),
    ("Pol",    "moon",   "Jool",   179890000,   0.17,   1.081e19, 0.373,  44000,    4.25,    901902.6,   901903,    10.0,     1042140,      False, 15,  1),
    ("Eeloo",  "planet", "Kerbol", 90118820000, 0.26,   1.115e21, 1.687,  210000,   6.15,    156992048.4,19460,     122.5,    119082940,    False, 16,  3),
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
        # thick toxic SO2 haze (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.80, 0.85, 0.35], "thickness": 25000,
                       "power": 4.0, "intensity": 0.75},
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
        # N2/O2 rim (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.30, 0.50, 1.00], "thickness": 15000,
                       "power": 4.0, "intensity": 0.7},
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
    "Eden": {
        # thicker green-tinted N2/O2 rim than Kerbin's
        # (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.40, 0.65, 0.70], "thickness": 22000,
                       "power": 4.0, "intensity": 0.7},
    },
    "Duna": {
        "palette": [[0.0, [0.55, 0.22, 0.08]],
                    [0.6, [0.70, 0.35, 0.15]],
                    [1.0, [0.85, 0.60, 0.40]]],
        # thin dusty CO2 haze (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.80, 0.48, 0.30], "thickness": 8000,
                       "power": 4.0, "intensity": 0.55},
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
        # gas giant: the "atmosphere" is the whole body, so a broad, soft
        # pale rim (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.75, 0.85, 0.85], "thickness": 90000,
                       "power": 3.0, "intensity": 0.6},
    },
    "Laythe": {
        "amplitude": 3000,
        "sea_color": [0.00, 0.40, 0.45],
        "palette": [[0.0, [0.25, 0.55, 0.25]],
                    [0.7, [0.50, 0.60, 0.35]],
                    [1.0, [1.00, 1.00, 1.00]]],
        # N2/O2 rim (see reports/atmosphere2026_08_25)
        "atmosphere": {"color": [0.30, 0.55, 0.90], "thickness": 12000,
                       "power": 4.0, "intensity": 0.7},
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


def ksp_body(name, typ, orbits, sma, ecc, mass, g, radius, inc_deg, orb_s, rot_s, tilt_deg, soi, has_sea, seed, ps, phase_deg=0):
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
        b["inertial"] = {"soi": soi, "pos": [0, 0, 0], "orb_ang_speed": 0.0}
    else:
        wiki = WIKI_ORBITS.get(name)
        if wiki and wiki.get("period") and orbits in MASS:
            # KSP body: start at its real epoch position, from the orbital
            # elements on its individual wiki page (eccentricity, inclination,
            # argument of periapsis w, longitude of the ascending node raan,
            # mean anomaly at 0s UT) instead of the old shared-axis layout.
            # load_system derives a from orb_ang_speed via Kepler's third law,
            # so the JSON only needs w + the angles. arg_peri and true_anomaly0
            # are emitted even for circular orbits (e=0): they set the starting
            # angle on the circle (w + M), which is what the wiki encodes.
            e = wiki["e"] or 0.0
            i = wiki["i"] or 0.0
            omega = wiki["omega"] or 0.0
            raan = wiki["raan"] or 0.0
            M = wiki["M"] or 0.0
            w = TWOPI / wiki["period"]
            nu = true_anomaly_from_mean(M, e)
            inertial = {
                "soi": soi,
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
            # Non-wiki body (Eden): a game-added body, not on the KSP wiki.
            # It is Kerbin's L4 trojan -- phase_deg ahead of Kerbin in the
            # orbit direction, on the same circle. Kerbin now sits at its
            # wiki epoch longitude, so anchor Eden to THAT: "ahead" is the
            # orbit direction (decreasing in-plane angle), i.e. Kerbin's
            # epoch longitude minus phase_deg. Any other non-wiki body keeps
            # the old shared-axis layout (planets -Z, moons -X).
            kb = WIKI_ORBITS.get("Kerbin")
            if phase_deg and kb and kb.get("period"):
                kb_lon = (kb["raan"] or 0.0) + (kb["omega"] or 0.0) \
                    + true_anomaly_from_mean(kb["M"] or 0.0, kb["e"] or 0.0)
                th = kb_lon - math.radians(phase_deg)
                pos = [sma * math.cos(th), 0.0, sma * math.sin(th)]
                pos = [int(round(p)) for p in pos]
            elif phase_deg:
                th = math.radians(phase_deg)
                if typ == "planet":
                    pos = [-sma * math.sin(th), 0.0, -sma * math.cos(th)]
                else:
                    pos = [-sma * math.cos(th), 0.0, -sma * math.sin(th)]
                pos = [int(round(p)) for p in pos]
            elif typ == "planet":
                pos = [0, 0, -sma]
            else:
                pos = [-sma, 0, 0]
            inertial = {
                "soi": soi,
                "pos": pos,
                "orb_ang_speed": spd(orb_s),
            }
            if inc_deg:
                inertial["orb_incl"] = math.radians(inc_deg)
            b["inertial"] = inertial
        # Every planet / moon spins; near-body SOI = radius + 100 km.
        rotating = {
            "soi": radius + 100000,
            "rot_ang_speed": spd(rot_s),
        }
        # Axial tilt (CSV "Axial tilt" column): lean the spin axis from the
        # orbital normal toward +X; 0 = pole on the orbit normal (omitted).
        # The star gets no "rotating" block (dummy frame), so its tilt value
        # is documentation only.
        if tilt_deg:
            rotating["axial_tilt"] = math.radians(tilt_deg)
        b["rotating"] = rotating
    return b

# Parent masses for the eccentric-orbit fit in ksp_body (Kepler's third law).
MASS = {row[0]: row[5] for row in K}

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
