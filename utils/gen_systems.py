#!/usr/bin/env python3
# Generate res/systems/{old,ksp}_system.json. Angular speeds = 2*pi / period.
# Source of truth for the committed JSONs; --check verifies no drift.
# One exception: the launch SITES are hand-authored fragments under
# res/data/ (res/data/{old,ksp}_launch_sites.json). This script names them via
# "world_buildings_files" and emits each system's inline polar pad, but never
# writes a site table -- pads used to be hand-edited inside generated output,
# where a regenerate silently dropped them and --check never looked (#226).
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
# #636: the named gas mixes live in ONE module that BOTH system generators import.
# Each generator only ever compares a file against its OWN output, so a mix kept here
# as well as in make_solar_system.py can drift while both --check passes stay green --
# which is how DRY_AIR and CO2_HEAVY ended up typed twice. Names are not swappable:
# the suffix says which system measured it.
from gas_mixes import DRY_AIR, CO2_HEAVY, H2_HE_CH4_KSP
# #690: same single-spelling rule as the gas mixes -- a layer table copied from a published
# standard is stated per GEOPOTENTIAL metre, and this is the only place that unit gets
# converted (a table that carries no marker is read as GEOMETRIC metres, which is what every
# altitude in a system file means). Both generators call the same function, because each
# --check only compares its own output and would not notice the two drifting.
from atmo_units import (resolve_layer_heights, anchor_sea_level_pressure,
                       check_anchor_conversion, check_body_gravity,
                        SIGMA_SB, semimajor_axis, star_temperature_from_flux,
                        blackbody_temperature, check_constants_match_src)

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


def load_wiki_thermal(csv_path):
    """The thermal columns ksp_bodies.csv carries: {name: (sma_m, T_lo_K, T_hi_K)}.
    For a PLANET that pair is a NIGHT/DAY extreme, not an average (Kerbin 186.95 -
    288.15 K); for the star it is a photosphere-to-corona range that no single surface
    reading satisfies (Kerbol 4014.95 - 9621.2 K). Both get printed against what this
    script authors, never quietly averaged down to one number (#687)."""
    import csv
    out = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            def num(key):
                v = (row.get(key) or "").strip()
                if v in ("", "nan", "inf", "-inf"):
                    return None
                return float(v)
            lo, hi = num("temperature_min_k"), num("temperature_max_k")
            if lo is None and hi is None:
                continue
            out[row["name"]] = (num("semi_major_axis_m"), lo, hi)
    return out

WIKI_THERMAL = load_wiki_thermal(os.path.join(HERE, "ksp_bodies.csv"))

def wiki_pair_note(name, T):
    """What utils/ksp_bodies.csv states as this body's night/day temperature pair, and a
    flag when the temperature the caller names sits outside it. The albedo pass flags an
    AIRLESS MEAN against the same pair; this flags the air a file AUTHORS, which is the
    number a player reads in a dossier and the one a temperature field has to answer to.
    #625 is why both are worth printing: Jool's air used to read 44.63 K under a 111 K
    mean -- 67 K colder than its own sunshine, which no greenhouse reading explains
    (#738) -- and its authored 151.43 K sits inside the pair's 120.01 - 225.07 K. Laythe
    stays flagged: 288.15 K of sea-level air over a 113 K airless mean asks 175 K of a
    0.6 atm sky, and the csv's own top row reads 277 K. A flag that still fires on a body
    whose every number was argued is the instrument asking the next question, not this
    pass failing the last one."""
    rng = WIKI_THERMAL.get(name)
    if not rng or rng[1] is None or rng[2] is None:
        return ""
    note = "  wiki night/day %7.2f - %7.2f K" % (rng[1], rng[2])
    if not (rng[1] <= T <= rng[2]):
        note += '  <-- the air this file authors is outside that pair (#738)'
    return note

# What the home world is built to receive. Kerbin's air authors 101.3 kPa of dry air,
# its sea is fresh water, its tilt is 23.44 deg and its datum 288.15 K: it is an
# Earth-shaped world, and Earth's own solar constant is the light that goes with that.
EARTH_SOLAR_FLUX = 1361.0   # W/m^2, Earth's Bulk-parameters row in utils/rss/html/

# WHAT THE AIR IS MADE OF (#552 A4b, #630), by MOLE. A body authors its mix and the
# loader resolves molar mass, gamma and the oxidizer share the jet gate reads from it
# (src/atmo.h:resolveMix), so "molar_mass"/"gamma" are no longer authorable keys.
# Species names must exist in the game's gas table (atmo.h:gasSpecies). The named mixes
# are imported, not typed here: a mix two generators could each invent has to exist
# once, since each one only ever validates its own output (#636).


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
    # Launch pads. The sites live in a hand-authored "world_buildings"
    # fragment that the loader merges (src/buildings.cpp); this script names
    # the file and never writes it, so regenerating the system JSON cannot
    # clobber a coordinate -- which is how the pads went stale in the first
    # place, being hand-edited inside generated output. Merged fragments come
    # BEFORE the inline array below, and the merged list's first
    # VAB_LAUNCH_PAD is the game's default site.
    "world_buildings_files": ["res/data/old_launch_sites.json"],
    # A second pad at the geographic north pole (flag POLAR_PAD). Inline
    # because it belongs to THIS system, not to the site table -- and inline
    # keeps it out of the default-site race, since the merged fragment's first
    # VAB_LAUNCH_PAD is the game's default.
    "world_buildings": [
        {"id": "polar_pad", "type": "space_port",
         "display_name": "Polar pad", "body": "Eerbon",
         "lat_deg": 90.0, "lon_deg": 0.0, "height_m": 5.0,
         "heading_deg": 0.0,
         "flags": ["VAB_LAUNCH_PAD", "POLAR_PAD"]},
    ],
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
# ocean ("mesh" | "flat", default "mesh"), amplitude, bands/* for gas giants.
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
                       # #625: this body's profile is authored in KSP_AIR below, which
                       # is where its temperature, its lapse and its ceiling come from.
                       # The pressure stays what the old rho0 x H column weighed (1.7
                       # kg/m3 on 7 km at g 16.677), deliberately: rung A is about the
                       # air's TEMPERATURE, and moving a datum and a temperature in one
                       # diff makes neither of them reviewable. The wiki's 5 atm row is
                       # in utils/ksp_bodies.csv and stays unadopted (#702).
                       "sea_level_pressure": 198456.3,
                       # Denser sea-level air than Kerbin's -- 3.56 kg/m3 at the datum,
                       # which is what 1.96 atm weighs on a 1.7 g world -- and not one
                       # gram of it will burn: a jet on Eve has plenty to swallow and
                       # nothing to swallow it with (#630).
                       "composition": CO2_HEAVY},
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
        # #687: the ground's temperature. ksp_bodies.csv states Kerbin's range as
        # 186.95 - 288.15 K -- a night/day PAIR, not an average -- and the upper figure
        # is already the number this body's air starts from (the 1976 table's sea-level
        # row, 288.15 K, three lines below). So one published number is used twice, and
        # the ground and the air's datum agree by authorship rather than by accident.
        # Every other KSP body's range is left UNAUTHORED: a day-side maximum is not a
        # mean, and inventing the difference is a design call this script does not get
        # to make silently. The readouts print a dash for those.
        "temperature": 288.15,
        # #686: Kerbin stops being a 188 K world. A scale height IS a surface temperature --
        # the trio lock (p0 = rho0 g H, T = g H M / R) makes the old `scale_height: 5500` read
        # as T0 187.96 K at 66.1 kPa with a 275 m/s sound speed, which is pinned at
        # tests/test_scenarios.cpp and is what every Mach number and barometer on the home world
        # has been paying. Warm and thin cannot both come from one exponential: isothermal at
        # 288.15 K wants H 8432 m, which would put x111 more air at an 85 km top than 5500 does
        # -- weighed in the body's own 1/r^2 field the way the loader weighs it (#683; the
        # uniform column says x84 at 70 km, which is the number this note used to quote). A
        # LAYER TABLE is the shape that pays both: it keeps the warm datum and spends most of
        # that back aloft, landing x23 at the top rather than x111 -- so Kerbin now wears the
        # same 1976 rows Earth does, and the generator converts them against a 600 km radius
        # instead of a 6371 km one. That conversion is not
        # cosmetic here: the 11 km row moves 19 m on Earth and 205.4 m here, and the 51 km row
        # 412 m vs 4,737.7 m (#690, utils/atmo_units.py).
        #
        # `height` 70000 -> 85000 is #682's half of the same diff, and it is a decision, not a
        # side effect: the top is where satellites start feeling drag, and shellEdge is
        # max(100 km, 1.2 top), so this moves the rotating frame, the LowOrbit/HighOrbit bands,
        # the flying-slow spawn bed (0.1 top: 7.0 -> 8.5 km) and the atlas disk together. Seven
        # levels need it: the 71 km row resolves to 80,529.3 m geometric here, so a 70 km body
        # refuses it (atmo.h: the last level must sit below `height`).
        "atmosphere": {"color": [0.30, 0.50, 1.00], "thickness": 15000,
                       "power": 4.0, "intensity": 0.7,
                       # US76's own sea-level row: 101,325 Pa at 288.15 K. Its
                       # 1.225 kg/m3 is that pair's rounded consequence, so the file
                       # states the pair and the loader resolves the density (#739).
                       "sea_level_pressure": 101325.0,
                       # Written out rather than left to the dry-air default: the
                       # atlas names the air, and "breathable" is a fact about this
                       # world that a player should be able to read off it.
                       "composition": DRY_AIR,
                       "height": 85000,
                       # #690: these are the standard's OWN numbers, per GEOPOTENTIAL metre.
                       # resolve_layer_heights() converts altitude and lapse rate against this
                       # body's radius and drops the marker, so the emitted file speaks the one
                       # unit every other altitude in it speaks.
                       "heights": "geopotential",
                       "layers": [
                           {'alt': 0,     'T': 288.15, 'lapse': -0.0065},
                           {'alt': 11000, 'T': 216.65, 'lapse': 0.0},
                           {'alt': 20000, 'T': 216.65, 'lapse': 0.001},
                           {'alt': 32000, 'T': 228.65, 'lapse': 0.0028},
                           {'alt': 47000, 'T': 270.65, 'lapse': 0.0},
                           {'alt': 51000, 'T': 270.65, 'lapse': -0.0028},
                           {'alt': 71000, 'T': 214.65, 'lapse': -0.002}]},
        # Deck height is metres above the SEA-LEVEL datum, like the layers table above
        # it and the air top (#719). It used to be an offset off the highest ground, so
        # Kerbin's 2500 meant "7.5 km up" only as long as its 5 km of relief held.
        "clouds": {"height": 7500, "coverage": 0.6, "freq": 10,
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
                       # The same US76 datum pair Kerbin wears, for the same reason
                       # (#700, #739).
                       "sea_level_pressure": 101325.0,
                       # #700 step 2, Shay's half. It has a sea (has_sea) and wore an
                       # air whose datum read 188.0 K: one exponential IS isothermal,
                       # so the trio lock (#686) had already turned the authored
                       # H 6000 into a surface temperature, and 188 K at 66.1 kPa is
                       # a world whose sea is a rock. Nothing in src/ has a ground
                       # temperature to freeze yet (#687), but the atlas prints that
                       # 188 K as the air's fact and every Mach number pays it.
                       # Same medicine Kerbin took in #686: wear the 1976 rows, which
                       # keep the warm datum and spend it back aloft, instead of one
                       # warm exponential. Warmed to a 288 K datum at Shay's own g an
                       # exponential wants H 9.2 km, which leaves 109x the air at a
                       # 95 km ceiling that the cold H 6000 left -- the table leaves a
                       # fifth of even that (1.8e-4 vs 3.6e-5 kg/m3).
                       # Shay is Kerbin's L4 trojan -- same orbit, same sunlight --
                       # so a 288.15 K datum is not a guess about its climate, it is
                       # the same distance from Kerbol. p0 = rho0 (R/M) T0 carries no
                       # g, so at rho0 1.225 Shay's datum lands on 101.3 kPa like
                       # Earth's and Kerbin's: three worlds, one sea-level pressure,
                       # differing only in how high that air reaches.
                       "composition": DRY_AIR,
                       # Forced first, then chosen. The 71 km row resolves to
                       # 81,524.0 m geometric on a 550 km world and atmo.h refuses a
                       # table whose last level sits at or above `height`, so the old
                       # 60000 cannot carry this profile. What it becomes is #682's
                       # call, answered the way Kerbin's 85 km was: 95 km is where a
                       # circular orbit feels 75 Pa of q -- Kerbin's edge is 97.8 Pa
                       # and Duna's is 110.5, all three inside the 10..1000 Pa band.
                       # It moves shellEdge off the 100 km floor to 114 km and the
                       # flying-slow bed 6.0 -> 9.5 km. It does NOT move the rendered
                       # sky rim, which is terrain max height + `thickness` above the
                       # datum and not derived from `height` at all (#56).
                       "height": 95000,
                       # Same authored rows as Kerbin and Earth (#690 converts bases
                       # and lapse against THIS radius, per geopotential metre).
                       "heights": "geopotential",
                       "layers": [
                           {'alt': 0,     'T': 288.15, 'lapse': -0.0065},
                           {'alt': 11000, 'T': 216.65, 'lapse': 0.0},
                           {'alt': 20000, 'T': 216.65, 'lapse': 0.001},
                           {'alt': 32000, 'T': 228.65, 'lapse': 0.0028},
                           {'alt': 47000, 'T': 270.65, 'lapse': 0.0},
                           {'alt': 51000, 'T': 270.65, 'lapse': -0.0028},
                           {'alt': 71000, 'T': 214.65, 'lapse': -0.002}]},
        "clouds": {"height": 6000, "coverage": 0.5, "freq": 10,   # datum m (#719)
                   "drift": 0.00001},
    },
    "Duna": {
        "palette": [[0.0, [0.55, 0.22, 0.08]],
                    [0.6, [0.70, 0.35, 0.15]],
                    [1.0, [0.85, 0.60, 0.40]]],
        "atmosphere": {"color": [0.80, 0.48, 0.30], "thickness": 8000,
                       "power": 4.0, "intensity": 0.55,
                       # The pressure the old H 13.6 km column weighed (0.015 kg/m3 at
                       # g 2.943), kept while #625 re-weights the temperature; the wiki
                       # states 6.755 kPa here, 11x denser, unadopted (#702).
                       "sea_level_pressure": 600.372,
                       # Duna wears Mars's air at Duna's gravity (#700 step 1, and
                       # the same argument as #698). What it used to author -- 0.12
                       # kg/m^3 on a 4 km scale height -- is not a world: the trio
                       # lock (#686) says H IS a surface temperature, so this body
                       # read T0 61.6 K, and CO2 at its own 1.41 kPa precipitates as
                       # dry ice below ~148 K. The air on that sheet is a rock.
                       # H 13.6 km at g 2.943 and this mix reads T0 209.4 K, which
                       # is Mars's mean; rho0 0.015 puts the datum at 600 Pa, again
                       # Mars's. It is thinner than Mars only because the column is
                       # shorter, not because the ground is bare.
                       # A world at 209 K cannot keep CO2 ice caps at its surface,
                       # so any polar-cap story here is a #687 question -- nothing
                       # in src/ has a ground temperature to freeze.
                       # Thin CO2 with a 0.13% oxygen trace: enough to measure, not
                       # enough to run a turbine on.
                       "composition": CO2_HEAVY},
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
                       # A giant's DATUM is the 1 bar level -- that is the level every
                       # giant row in the reference file is stated at (Jupiter, Saturn,
                       # Uranus, Neptune all source "at 1 bar" densities and
                       # temperatures, #697), and a body with no ground has no other
                       # non-arbitrary one. 100,000 Pa at 151.43 K of this mix resolves
                       # 0.188 kg/m3, against Jupiter's published 0.162 kg/m3 at its own
                       # 1 bar. What this used to author -- 2.0 kg/m3 on a 20 km scale
                       # height -- weighed 313.9 kPa and read 44.63 K, 67 K colder than
                       # this world's own sunlight, which is #738's absurdity.
                       "sea_level_pressure": 100000.0,
                       # H2 with helium and a methane crown. The most fuel-rich air in
                       # this system and 0% oxidizer: Jool is a fuel store, not a sky a
                       # system and 0% oxidizer: Jool is a fuel store, not a sky a
                       # jet can cross. KSP's Jool, not the solar system's Uranus:
                       # same three gases, different sky (#636).
                       "composition": H2_HE_CH4_KSP},
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
                       # The same US76 datum pair Kerbin wears, for the same reason
                       # (#700, #739).
                       "sea_level_pressure": 101325.0,
                       # #700 step 2, Laythe's half. This is the world whose whole
                       # pitch is a sea, and its air read 150.4 K / 52.9 kPa -- the
                       # number that made src/drag.h open the trio lock with it. An
                       # ocean under a 150 K sky is a contradiction the file itself
                       # was asserting, so it now wears the same 1976 rows Kerbin and
                       # Earth wear, converted against its own 500 km radius, and the
                       # datum reads 288.15 K at 101.3 kPa. Water is the point of
                       # this world, so the datum has to be above 273 K.
                       # What this does NOT buy: nothing here says why a moon of Jool
                       # is 288 K. The KSP catalogue carries no orbital radius for
                       # Laythe, so no authored number contradicts it, and #687 is
                       # where insolation and a ground temperature arrive and will
                       # have to answer for it -- a world this far out wants either a
                       # greenhouse or a colder, wetter air.
                       "composition": DRY_AIR,
                       # Forced, then chosen, exactly as on Shay: the 71 km row lands
                       # at 82,750.6 m geometric on a 500 km world, so the old 50 km
                       # top refuses this table. Above that floor the number is
                       # #682's rule, not a round figure: at 105 km a circular orbit
                       # meets 106.1 Pa, against Kerbin's 97.8 at its own edge. (90 km
                       # would also be in band at 430 Pa, but a top is where orbit
                       # stops being orbit, and 4.4x Kerbin's edge reads as a wall.)
                       # shellEdge goes from the 100 km floor to 126 km; the flying
                       # beds go 5.0 -> 10.5 km. Laythe's column reaches further than
                       # Kerbin's or Shay's because its g is lower and the same air is
                       # therefore puffier -- the datum is identical on all three.
                       # The band only speaks AT the ceiling, and this is the world
                       # where that is not the whole story: at 70 km a circular orbit
                       # already feels 2.36 kPa of q on Laythe, against Shay's 1.00 kPa
                       # and Kerbin's 0.53 -- a drag-heavy low sky long before the
                       # edge, which is what #553/#554 will have to price. And the
                       # 71 km row's own lapse runs on up to the ceiling, so the top
                       # sits in 183 K air (214.65 - 0.00141818 x 22.2 km).
                       "height": 105000,
                       "heights": "geopotential",
                       "layers": [
                           {'alt': 0,     'T': 288.15, 'lapse': -0.0065},
                           {'alt': 11000, 'T': 216.65, 'lapse': 0.0},
                           {'alt': 20000, 'T': 216.65, 'lapse': 0.001},
                           {'alt': 32000, 'T': 228.65, 'lapse': 0.0028},
                           {'alt': 47000, 'T': 270.65, 'lapse': 0.0},
                           {'alt': 51000, 'T': 270.65, 'lapse': -0.0028},
                           {'alt': 71000, 'T': 214.65, 'lapse': -0.002}]},
        "clouds": {"height": 7000, "coverage": 0.55, "freq": 10,  # datum m (#719)
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
        # #690: honour the unit marker if a surface table carries one, against the datum
        # radius this body's altitudes will be measured from.
        b["surface"] = resolve_layer_heights(SURFACES[name], radius)
        # #739: the datum anchor the FILE states is a pressure; the densities authored
        # in SURFACES below are sheet/wiki rows and are converted here.
        anchor_sea_level_pressure(b["surface"], radius, g)

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
    # Launch pads: same split as eerbon above. The Kerbol table spans three
    # bodies (Kerbin, Moho, Duna), so it is named for the system, while
    # res/data/earth_launch_sites.json is named for the one body it serves.
    "world_buildings_files": ["res/data/ksp_launch_sites.json"],
    # A second pad at the geographic north pole (flag POLAR_PAD). Inline
    # because it belongs to THIS system, not to the site table -- and inline
    # keeps it out of the default-site race, since the merged fragment's first
    # VAB_LAUNCH_PAD is the game's default.
    "world_buildings": [
        {"id": "polar_pad", "type": "space_port",
         "display_name": "Polar pad", "body": "Kerbin",
         "lat_deg": 90.0, "lon_deg": 0.0, "height_m": 5.0,
         "heading_deg": 0.0,
         "flags": ["VAB_LAUNCH_PAD", "POLAR_PAD"]},
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
    # Whole-document compare, bodies excepted: this script is the source of
    # truth for EVERY root key, so anything that differs is drift. It used to
    # whitelist home/skybox/belts -- and a whitelist is exactly why the pads
    # went unnoticed (#226): they were hand-edited inside generated output,
    # so a regenerate dropped them and nothing compared them. Bodies are
    # compared by name below instead, which reports a rename or a dropped
    # body as itself rather than as a cascade of positional noise.
    deep_diff({k: v for k, v in committed.items() if k != "bodies"},
              {k: v for k, v in obj.items() if k != "bodies"}, "", out)
    # A reference to a missing fragment throws at load, so catch the typo here
    # where the message names the key rather than as a failed system load.
    for name in obj.get("world_buildings_files", []):
        if not os.path.exists(os.path.join(ROOT, name)):
            out.append(f"world_buildings_files: '{name}' not found under {ROOT}")
    cb = {x["name"]: x for x in committed.get("bodies", [])}
    gb = {x["name"]: x for x in obj.get("bodies", [])}
    for name in cb:
        if name not in gb:
            out.append(f"body {name}: in committed file, missing from generated")
    for name in gb:
        if name not in cb:
            out.append(f"body {name}: in generated, missing from committed file")
    # Order too: the array order is this script's to choose and it is the
    # loader's bodies vector, so a hand-reorder of the JSON is drift as well.
    cn = [x["name"] for x in committed.get("bodies", [])]
    gn = [x["name"] for x in obj.get("bodies", [])]
    if cn != gn and sorted(cn) == sorted(gn):
        out.append("bodies: same names in a different order")
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

def author_star_temperature(obj):
    """#687: a star's output is ONE authored number (surface.temperature) that every
    body's insolation derives from, so nothing internal can catch it being wrong --
    the #684/#686 meta-law again. Both these systems author it the same way: the home
    world receives Earth's sun, inverted through solar.h's L = 4 pi R^2 sigma T^4 at
    that world's own rail -- the same reason its air authors 101.3 kPa of dry air, its
    sea is fresh water and its tilt 23.44 deg.

    The wiki's star row states a temperature RANGE instead (Kerbol 4014.95 - 9621.2 K,
    which is a photosphere-to-corona pair, not a surface) and even its LOW end would put
    5,451 W/m^2 on a world built like Earth. It is printed as a disagreement, not used:
    which statement to honour is a design call, not a scrape. Both lines print whether or
    not anything agrees (#685's rule)."""
    star = next(b for b in obj["bodies"] if b["type"] == "star")
    home = next(b for b in obj["bodies"] if b["name"] == obj["home"])
    w = (home.get("inertial") or {}).get("orb_ang_speed")
    assert w, home["name"] + ": no orbital speed to size the star by"
    a = semimajor_axis(G * star["mass"], w)
    T = star_temperature_from_flux(EARTH_SOLAR_FLUX, a, star["radius"])
    star.setdefault("surface", {})["temperature"] = round(T, 2)
    print('  star      %-8s T %9.2f K -> %7.1f W/m2 at %s (%.6g m), from'
          ' "%s receives Earth\'s sun"'
          % (star["name"], T, EARTH_SOLAR_FLUX, home["name"], a, home["name"]))
    rng = WIKI_THERMAL.get(star["name"])
    # Both ends or neither: load_wiki_thermal allows a lone min or max, and printing
    # '%9.2f' of None here would crash the generator over a cosmetic CSV gap.
    if rng and rng[1] is not None and rng[2] is not None:
        lo, hi = rng[1], rng[2]
        flux_lo = SIGMA_SB * lo ** 4 * (star["radius"] / a) ** 2
        print('  wiki      %-8s states %9.2f - %9.2f K, which would put %7.1f W/m2'
              ' there instead -- named, not resolved (#687)'
              % (star["name"], lo, hi, flux_lo))


# #735: the shipped system authored no albedo for any body, so the atlas' starlight
# block (#687) printed the light and dashed everything under it on the worlds players
# actually fly. These are AUTHORED BY ANALOGY, not scraped: no KSP source states a Bond
# albedo at all (ksp_bodies.csv has no albedo column, #722). Each KSP body is paired with
# the solar-system body it is already built like -- Kerbin authors 101.325 kPa of dry air,
# 23.44 deg and 288.15 K by the same analogy -- and the number comes out of
# res/systems/solar_system.json, which takes it from the RSS fact sheets. Reading the
# reference file rather than retyping its figures is #636's rule: two files each holding
# their own copy of Earth's albedo drift apart in silence.
#
# BOND, never geometrical: solar_system.json's whole convention (make_solar_system.py:
# fact_surface) is the sheets' Bond column, and the values below show why the distinction
# is not pedantry -- the sheets state Jupiter's geometrical albedo at 0.538 against a Bond
# of 0.343, and Venus's at 0.689 against a Bond of 0.77, and it is the Bond column the
# (1-A)S/4 law wants. (Both pairs read from venusfact.html and jupiterfact.html.)
#
# (analogue, override): override is None to take the analogue's authored value. No row needs
# an override now: the four bare rocks (Mun, Ike, Dres, Tylo) borrow the Moon's 0.11, and our
# solar file AUTHORS that figure since #737 -- read out of solar_system.json like every other
# value here, from moonfact.html's Bulk parameters. The Moon is the only airless rocky body in
# the corpus with a stated Bond albedo, so Moho's Mercury aside, every bare rock here borrows
# it. That flattening is nearly free: bb_T goes as (1-A)^1/4, so A of 0.068 instead of 0.11
# moves Moho's airless mean by 5 K on 440 K -- under 2%, and less further out. Where the
# pairing IS a bright one (Eeloo's ices, Pluto again) it is made honestly or not at all.
#
# Deliberately LEFT UNAUTHORED, because a dash is the true answer (#727) and the shipped
# system keeps an example of one for the dash-law cases to bite on:
#   Gilly, Bop, Pol  irregular bodies no source describes -- a captured comet and two
#                    captured nothing-bits; inventing 0.0x for them is decoration.
#   Minmus, Vall     icy. The corpus states a VISUAL GEOMETRICAL albedo for the icy
#                    satellites (Europa 0.68, Enceladus 1.0) and no Bond albedo, and
#                    geometric -> Bond needs a phase integral nothing in the corpus
#                    gives. Filed rather than guessed (#736).
DASH = object()   # a named sentinel, NOT a bare None: the None inside ("Moon", None)
                  # means "take the analogue's value", and one value meaning both is how
                  # a body silently stops being priced
KSP_ALBEDOS = {
    "Moho":   ("Mercury", None),   # hot, airless, grey
    "Eve":    ("Venus",   None),   # thick CO2 sky over a hot rock
    "Kerbin": ("Earth",   None),   # Earth-shaped by construction
    "Mun":    ("Moon",    None),   # airless rock; 0.11 read from the solar file's Moon
    "Minmus": DASH,                # icy -- see the note above
    "Shay":   ("Earth",   None),   # authored air, sea and clouds: an Earth-shaped moon
    "Duna":   ("Mars",    None),
    "Ike":    ("Moon",    None),
    "Dres":   ("Moon",    None),   # belt dwarf, same bare-rock bucket
    "Jool":   ("Jupiter", None),
    "Laythe": ("Earth",   None),   # air and sea, on 53 W/m2: its wiki temperatures do not fit (#738)
    "Vall":   DASH,                # icy
    "Tylo":   ("Moon",    None),
    "Bop":    DASH,
    "Pol":    DASH,
    "Gilly":  DASH,
    "Eeloo":  ("Pluto",   None),   # dwarf of bright ices; inherits Pluto's self-contradicting sheet too
}

def solar_reference_albedos():
    """{name: authored Bond albedo} from the reference system file."""
    import json as _json
    path = os.path.join(ROOT, "res", "systems", "solar_system.json")
    with open(path) as f:
        obj = _json.load(f)
    out = {}
    for b in obj["bodies"]:
        a = (b.get("surface") or {}).get("albedo")
        if a is not None:
            out[b["name"]] = a
    return out

def mean_orbit_to_star(body, by_name):
    """The MEAN orbit of the nearest ancestor of `body` that orbits the star, the way
    atlasDossier() picks it: a moon reads its PLANET's rail, because its own distance to
    the sun swings over its month and no single figure there is honest."""
    cur = body
    while cur.get("orbits"):
        par = by_name.get(cur["orbits"])
        assert par, "body %s orbits %r, which this system does not have" % (
            cur["name"], cur["orbits"])
        if par.get("type") == "star":
            n = (cur.get("inertial") or {}).get("orb_ang_speed") or 0.0
            if n <= 0.0 or par.get("mass", 0.0) <= 0.0:
                return None, None
            return semimajor_axis(G * par["mass"], n), cur["name"]
        cur = par
    return None, None

def author_ksp_albedos(obj):
    """#735: put a Bond albedo on the shipped bodies, and print what it costs them.

    The print is the reviewable half. bb_T is what the star's own light and this albedo
    imply -- an airless global mean, NOT this body's ground -- so against the wiki's
    stated night/day temperature pair it says whether the two authored halves of this
    world agree about its sun. Where they do not, the line NAMES the disagreement and
    nothing resolves it, exactly as author_star_temperature() does for the star: Laythe's
    airless mean sits 86 K below the mildest reading its own wiki row states, which is a
    fact about the file and not a bug in the arithmetic, so it is filed as one."""
    by_name = {b["name"]: b for b in obj["bodies"]}
    star = next(b for b in obj["bodies"] if b.get("type") == "star")
    sT = (star.get("surface") or {}).get("temperature") or 0.0
    assert sT > 0.0, "author_ksp_albedos() must run after author_star_temperature()"
    lum = 4.0 * math.pi * star["radius"] ** 2 * SIGMA_SB * sT ** 4
    ref = solar_reference_albedos()
    # Every non-star body is either priced or named. A body added to the system and left
    # out of this table would silently regain the hole #735 just filled -- and the shipped
    # file is the one place the atlas' dash is a player-facing thing.
    named = {b["name"] for b in obj["bodies"] if b.get("type") != "star"}
    assert named == set(KSP_ALBEDOS), (
        "KSP_ALBEDOS and this system disagree: only in the system %s, only in the table %s"
        % (sorted(named - set(KSP_ALBEDOS)), sorted(set(KSP_ALBEDOS) - named)))
    for name, spec in KSP_ALBEDOS.items():
        body = by_name.get(name)
        assert body, "KSP_ALBEDOS names a body this system does not have: " + name
        a_star, rail = mean_orbit_to_star(body, by_name)
        flux = (lum / (4.0 * math.pi * a_star * a_star)) if a_star else 0.0
        if spec is DASH:
            # The light still prints for a body with no albedo -- the atlas says the same
            # thing, and it is where a reader sees that the dash is one row, not the block.
            print('  albedo    %-8s dash%s  (no honest analogue: see the KSP_ALBEDOS note)'
                  % (name, '  %7.1f W/m2 at %s' % (flux, rail) if a_star else ''))
            continue
        analogue, override = spec
        a = override if override is not None else ref.get(analogue)
        assert a is not None, ('no authored albedo for %r in solar_system.json, and no '
                               'override for %s' % (analogue, name))
        assert 0.0 <= a < 1.0, "%s: albedo %g is not a Bond albedo" % (name, a)
        body.setdefault("surface", {})["albedo"] = a
        src = ("from solar_system.json's %s" % analogue) if override is None \
            else ("the corpus's %s value, stated but unauthored there" % analogue)
        if not a_star:
            print('  albedo    %-8s %5.3f  %s' % (name, a, src))
            continue
        bb = blackbody_temperature(flux, a)
        # The CSV's temperature pair is a NIGHT/DAY extreme, not a mean, so it is never
        # averaged into a number this file authors (#687's rule). Printed instead: the
        # airless mean landing OUTSIDE the pair says the two authored halves of that body
        # disagree about its sun, which is the sanity check #735 asked for and the reason
        # Laythe and Jool are filed rather than quietly left out (#738).
        rng = WIKI_THERMAL.get(name)
        note = ''
        if rng and rng[1] is not None and rng[2] is not None:
            lo, hi = rng[1], rng[2]
            note = ('  wiki night/day %9.2f - %9.2f K against an airless mean of %9.2f K%s'
                    % (lo, hi, bb,
                       '' if lo <= bb <= hi else '  <-- OUTSIDE the pair, filed'))
        print('  albedo    %-8s %5.3f  %7.1f W/m2 at %s -> %7.1f K  %s%s'
              % (name, a, flux, rail, bb, src, note))


# #625 (roadmap rung A): an air body's DATUM TEMPERATURE, authored rather than implied.
#
# Until now three of this file's six airs stated a SCALE HEIGHT, and the trio lock
# (#686) had already spent it: T0 = g H M / R, so Eve's H 7000 was a 610.70 K surface
# and Jool's H 20000 a 44.63 K cloud top -- 67 K colder than Jool's own sunlight. No
# downstream rung (a skin, a season, a part that heats up) can vary a temperature that
# an exponential profile cannot state, so the fix is not tidiness.
#
# A table takes the datum off the scale height, and then the temperature has to come
# from somewhere. Where no source states a mean (all six of these -- #722: the wiki's
# temperature column is a night/day PAIR, never a mean) the rule here is the one Kerbin
# already fits by accident of authorship:
#
#     datum T = (the airless global mean this file's OWN star gives at this body's
#               authored Bond albedo, #735) + (an authored gap, per body, below)
#
# so the light half and the air half of every row stay reconcilable (#687's meta-law)
# and the atlas' gap row prints the authored figure instead of an accident. Kerbin: its
# authored 288.15 K is 255.1 K of sunlight + 33.0 K, which is Earth's published gap.
# Duna's is +5.0 K, which is Mars's. The two figures that were already right by luck now
# have a stated source, and the ones that were wrong (Eve, Jool) have a knob.
#
# The gap is the only free number, and each one is a claim:
#   Eve +65 K   a column of Kerbin's MASS (p0/g 11,900 kg/m2 against Kerbin's 10,329)
#               that is pure CO2 rather than 0.04% CO2. Kerbin's gap is +33 K; Venus's,
#               at 63x that column, is +510 K. +65 K is a claim about a thin-ish green-
#               house gas column, not a scaled one, and it keeps the datum inside the
#               wiki's 160-420 K pair and 128 K under its day-side extreme.
#   Duna +5 K   Mars's own published gap, for 204 kg/m2 of air -- 2% of Kerbin's column.
#               Its old H implied +(-0.4) K, i.e. no greenhouse at all.
#   Jool +40 K  NOT a greenhouse: a giant has no ground to greenhouse, and its 1 bar
#               level sits above its black-body because of the heat inside it. Jupiter's
#               published 1-bar 165 K is +55 K over its own sunlight; Jool is a smaller,
#               cooler giant. #738 is the ticket for whether the atlas may call any of
#               these "greenhouse" at all, and it names Laythe's +175 K as the worst
#               case of that label (an ocean world on 53 W/m2, tidal heat at Jool).
# The two lapses are the bodies' own convective limits, not US76's: a lapse steeper than
# g/cp is a column that cannot hold itself up (cp from this file's own gamma and mix --
# Eve's limit is 20.4 K/km and it authors 9.0, which is what Venus's lower air is
# observed at; Duna's limit is 3.6 and it authors 3.5; Jool's is 0.66 and it authors
# 0.66, a fully convective interior). The LID above the tropopause is drawn where the
# column would otherwise freeze its own air out: CO2 deposits at ~206 K at Eve's partial
# pressure (its tropopause lands at 210.7 K) and at ~148 K at Duna's (its lid starts at
# 151.8 K) -- the same argument #700's Duna note makes against the 61.6 K column this
# replaces. Jool's lid at 112 K stays above methane's frost point at its 0.5% share.
#
# `top` is the ceiling, and it is chosen by the band gate (utils/check_atmo_band.py,
# which every `make test` runs): where a circular orbit first feels 10..1000 Pa of q,
# Kerbin's own edge being 97.8 Pa. Eve's published 90 km and Jool's published 200 km
# are NOT adoptable -- with a real temperature and 1.7 g, Eve's whole air is inside
# ~10 scale heights and 90 km leaves 1e-9 Pa. Duna's published 50 km IS adopted, and it
# happens to sit in band (139.9 Pa). The printed q below is what to read; the gate is
# what refuses the build if one of these numbers drifts.
KSP_AIR = {
    "Eve":    {"gap": 65.0, "via": "CO2 greenhouse",
               "lapse": -0.0090, "trop": 9000.0, "top": 31000.0},
    "Duna":   {"gap": 5.0, "via": "CO2 greenhouse (Martian)",
               "lapse": -0.0035, "trop": 18000.0, "top": 50000.0},
    "Jool":   {"gap": 40.0, "via": "internal heat, no ground",
               "lapse": -0.00066, "trop": 60000.0, "top": 600000.0},
    # Authored tables, worn for #700's reason (an ocean world needs the air its sea can
    # sit under): Kerbin's US76 rows, datum 288.15 K. Nothing is emitted here; the pass
    # prints their gap against their own light so the row is reviewed the same way.
    "Kerbin": {"table": "US76, datum 288.15 K"},
    "Shay":   {"table": "Kerbin's table, 550 km moon (#700)"},
    "Laythe": {"table": "Kerbin's table, sea world (#700); +175 K is Jool's tides, #738"},
}

def author_ksp_air_thermal(obj):
    """#625: emit the three exponential airs as authored layer tables, and print every
    air body's datum beside the light it is built on. Print first, refuse second: a pass
    that says nothing is indistinguishable from one that checked nothing (#685)."""
    by_name = {b["name"]: b for b in obj["bodies"]}
    star = next(b for b in obj["bodies"] if b.get("type") == "star")
    sT = (star.get("surface") or {}).get("temperature") or 0.0
    assert sT > 0.0, "author_ksp_air_thermal() must run after author_star_temperature()"
    lum = 4.0 * math.pi * star["radius"] ** 2 * SIGMA_SB * sT ** 4
    air = {b["name"] for b in obj["bodies"]
           if (b.get("surface") or {}).get("atmosphere")}
    # Same coverage discipline as KSP_ALBEDOS: a new air body left out of this table
    # would silently regain a scale-height thermometer, and the atlas would print its
    # temperature as fact.
    assert air == set(KSP_AIR), (
        "KSP_AIR and this system's airs disagree: only in the system %s, only in the "
        "table %s" % (sorted(air - set(KSP_AIR)), sorted(set(KSP_AIR) - air)))
    for name, spec in KSP_AIR.items():
        b = by_name[name]
        at = b["surface"]["atmosphere"]
        a = (b.get("surface") or {}).get("albedo")
        assert a is not None, ("%s has air but no albedo: its datum temperature is built "
                               "on the light, so the two halves must both exist (#735)"
                               % name)
        a_star, rail = mean_orbit_to_star(b, by_name)
        assert a_star, "%s has air but no rail to the star to read its light from" % name
        flux = lum / (4.0 * math.pi * a_star * a_star)
        bb = blackbody_temperature(flux, a)
        pair = wiki_pair_note(name, at["layers"][0]["T"]) if "table" in spec else ""
        if "table" in spec:
            T0 = at["layers"][0]["T"]
            print('  air T     %-8s %7.1f K of light + %6.1f K  -> %7.2f K datum, '
                  '%d levels: %s%s' % (name, bb, T0 - bb, T0, len(at["layers"]),
                                       spec["table"], pair))
            continue
        assert "scale_height" not in at and "layers" not in at and "height" not in at, (
            "%s: KSP_AIR owns this body's profile, so the surface block must not also "
            "state one (two sources, one of them silent)" % name)
        T0 = round(bb + spec["gap"], 2)
        trop_T = round(T0 + spec["lapse"] * spec["trop"], 2)
        assert 0.0 < spec["trop"] < spec["top"], "%s: tropopause above the ceiling" % name
        assert trop_T > 0.5 * T0, (
            "%s: a lapse of %.5f K/m leaves %.2f K at %.0f m -- the lid is colder than "
            "half the datum, which is not a troposphere" % (
                name, spec["lapse"], trop_T, spec["trop"]))
        at["layers"] = [{"alt": 0.0, "T": T0, "lapse": spec["lapse"]},
                        {"alt": spec["trop"], "T": trop_T, "lapse": 0.0}]
        at["height"] = spec["top"]
        pair = wiki_pair_note(name, T0)
        print('  air T     %-8s %7.1f K of light + %6.1f K (%s) -> %7.2f K datum, '
              'lapse %5.2f K/km to %.0f km, lid %7.2f K, top %.0f km%s'
              % (name, bb, spec["gap"], spec["via"], T0, -1000.0 * spec["lapse"],
                 spec["trop"] / 1000.0, trop_T, spec["top"] / 1000.0, pair))

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
    # These bodies are hand-typed: radius, mass and g come from three wiki figures on
    # three different pages. A body with air states its gravity twice (g weighs the
    # air's own column, mass is what an orbit answers to), and the two only part
    # company aloft -- so nothing but this check ever sees the disagreement. The loader
    # refuses the same thing at load (atmo.h:kAtmoGravityTol, #689); refusing here
    # names the typo at the keyboard instead of at boot.
    bad = []
    # Both generators size a star with sigma and weigh an air column with G, and both
    # spell those constants as COPIES of the loader's. Check that first: a generator
    # that has drifted from src/ produces files that pass every other check here and
    # describe a different universe in play (#687/#707).
    cbad = check_constants_match_src(verbose=True)
    if cbad:
        for m in cbad:
            print('DRIFT   constants: ' + m)
        raise SystemExit('utils/ and src/ disagree about a physical constant')
    # Not a per-file question: this one checks the CONVERSION every file's air was
    # written with, on synthetic bodies no generator emits (#739). Its own refusal
    # message, because the tally below prints 'body gravity' and would mislabel it.
    abad = check_anchor_conversion(verbose=True)
    if abad:
        for m in abad:
            print('DRIFT   anchor conversion: ' + m)
        raise SystemExit('utils/atmo_units.py cannot convert an authored density '
                         'into the datum pressure the loader inverts')
    for obj, _path in targets:
        author_star_temperature(obj)
        bad += check_body_gravity(obj["bodies"], verbose=True)
    # #735, and only for the shipped system: the legacy Eerbon file stays unauthored on
    # purpose. It is a frozen fixture (tests/test_belts.cpp builds on it), not a showcase,
    # and leaving it dashed keeps the absent-reading path loaded by a file the tests use.
    author_ksp_albedos(ksp)
    # #625 after #735 on purpose: an authored air temperature is built on the light, so
    # the albedos have to be in the object before the airs are.
    author_ksp_air_thermal(ksp)
    if bad:
        for m in bad:
            print('DRIFT   body gravity: ' + m)
        raise SystemExit('%d bodies author a g that their own mass and radius deny'
                         % len(bad))
    if args.check:
        ok = True
        for obj, path in targets:
            ok = check(obj, path) and ok
        raise SystemExit(0 if ok else 1)
    for obj, path in targets:
        write(obj, path)

main()
