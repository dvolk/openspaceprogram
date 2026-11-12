#!/usr/bin/env python3
"""Generate real-Solar-System star-system JSONs from NASA NSSDC fact sheets (utils/rss/html/).
mu, semimajor axis and the SOIs are derived by the loader: Kepler III for the orbit, and
"soi_law": "hill" + the nesting lift for the spheres (src/bodylimits.h). No SOIs are emitted
except the Sun's authored universe bound."""
import re, math, json, os, sys, html as htmllib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sci_dist import stamp_science_mults, stamp_transfer_dv, phase0
# #690: the geopotential/geometric conversion for layer tables lives in ONE module that
# both system generators import -- each --check only compares a file against its own
# output, so a second spelling of the law would drift in silence (the #636 lesson).
from atmo_units import (resolve_layer_heights, anchor_sea_level_pressure,
                       check_scraped_gravity,
                        SIGMA_SB, semimajor_axis, star_temperature_from_flux,
                        blackbody_temperature, check_constants_match_src)
# #724 rung C: the surface temperature law and its one knob, in ONE module both system
# generators import. The law is also spelled in C++ (src/groundtemp.h) for the game, and
# tests/test_groundtemp.cpp pins the two against the same numbers -- the atmo.h /
# atmo_units.py arrangement, which is the only way two spellings of one law can be caught
# drifting (#636).
from ground_model import (parse_diurnal_pair, parse_kelvin_range, fit_even_share_pair,
                          calibration_from_fitted, default_even_share,
                          SRC_PAIR_NIGHT, SRC_PAIR_DAY, SRC_PAIR_BOTH,
                          pair_solve, pair_anchor_check, invent_missing_albedos,
                          ground_anchor, even_share_air_note)

G      = 6.674e-11      # m^3 / kg / s^2  (matches the loader's G)
AU     = 1.496e11       # m
DAY    = 86400.0        # s
HR     = 3600.0         # s
D2R    = math.pi / 180.0
TWO_PI = 2.0 * math.pi
GOLDEN = 2.39996322972865332   # golden angle, rad; spreads epoch anomalies
SMALL_BODY_RHO = 2000.0        # kg/m^3, nominal small-moon density for estimates

SUN_MASS   = 1.989e30   # kg
SUN_RADIUS = 6.9634e8   # m

# IAU Working Group on Cartographic Coordinates and Rotational Elements,
# 2015 report (Archinal et al.), Table 1 (Pluto from Table 3) -- vendored
# at utils/rss/WGCCRE2015reprint1.pdf. Rotational elements:
#   alpha0/delta0: ICRF equatorial coords of the IAU north pole at J2000;
#   W0: prime meridian angle at the standard epoch (2000 Jan 1 12h TDB);
#   dW deg/day: rotation sense (negative = retrograde; the IAU north pole
#     is then opposite the angular-momentum pole our +Y represents).
# Periodic (amp, phase_deg) terms are evaluated at T = d = 0 -- NOT
# negligible for Mars (0.42 deg / 1.59 deg / 0.58 deg); the fact sheets'
# pole rows are these same expressions evaluated at epoch. The 2015
# report dropped Earth; its row is IERS: W0 is GMST at J2000.0, measured
# from the vernal equinox (not from node Q like the others).
# Fields: (alpha0, a_sinterms, delta0, d_costerms, W0, w_sinterms, dW).
WGCCRE = {
    'Mercury': (281.0103, [], 61.4155, [], 329.5988,
                [(0.01067257, 174.7910857), (-0.00112309, 349.5821714),
                 (-0.00011040, 164.3732571), (-0.00002539, 339.1643429),
                 (-0.00000571, 153.9554286)], 6.1385108),
    'Venus':   (272.76, [], 67.16, [], 160.20, [], -1.4813688),
    'Earth':   (0.0, [], 90.0, [], 280.46061837, [], 360.98564736629),
    'Mars':    (317.269202, [(0.000068, 198.991226), (0.000238, 226.292679),
                             (0.000052, 249.663391), (0.000009, 266.183510),
                             (0.419057, 79.398797)],
                54.432516, [(0.000051, 122.433576), (0.000141, 43.058401),
                            (0.000031, 57.663379), (0.000005, 79.476401),
                            (1.591274, 166.325722)],
                176.049863, [(0.000145, 129.071773), (0.000157, 36.352167),
                             (0.000040, 56.668646), (0.000001, 67.364003),
                             (0.000001, 104.792680), (0.584542, 95.391654)],
                350.891982443297),
    'Jupiter': (268.056595, [(0.000117, 99.360714), (0.000938, 175.895369),
                             (0.001432, 300.323162), (0.000030, 114.012305),
                             (0.002150, 49.511251)],
                64.495303, [(0.000050, 99.360714), (0.000404, 175.895369),
                            (0.000617, 300.323162), (-0.000013, 114.012305),
                            (0.000926, 49.511251)],
                284.95, [], 870.5360000),
    'Saturn':  (40.589, [], 83.537, [], 38.90, [], 810.7939024),
    'Uranus':  (257.311, [], -15.175, [], 203.81, [], -501.1600928),
    'Neptune': (299.36, [(0.70, 357.85)], 43.46, [(-0.51, 357.85)],
                249.978, [(-0.48, 357.85)], 541.1397757),
    # Table 3 (dwarf planets): no periodic terms. Note (m) defines Pluto's
    # 0 deg meridian as the mean sub-Charon meridian; main() re-anchors the
    # phase to Charon's actual epoch rail position anyway (#144/#147).
    'Pluto':   (132.993, [], -6.163, [], 302.695, [], 56.3625225),
}

OBLIQ_J2000 = 23.4392911   # deg, J2000 mean obliquity of the ecliptic


def _wgccre_epoch(name):
    """WGCCRE elements at t=0 = 2000-01-01 00:00 (the calendar's epoch).
    The report's standard epoch is that date at 12h TDB, so W carries
    d = -0.5: the calendar anchors MIDNIGHT to t=0, and evaluating W at
    12h would put local noon at midnight. Precession over half a day is
    negligible, so alpha/delta stay at T = 0.
    Returns (alpha_deg, delta_deg, W_deg, prograde)."""
    a0, at, d0, dt, w0, wt, dw = WGCCRE[name]
    a = a0 + sum(amp * math.sin(math.radians(ph)) for amp, ph in at)
    d = d0 + sum(amp * math.cos(math.radians(ph)) for amp, ph in dt)
    w = w0 - 0.5 * dw + sum(amp * math.sin(math.radians(ph)) for amp, ph in wt)
    return a, d, w, dw > 0.0


def _railvec_eq(x, y, z):
    """ICRF equatorial vector -> our rail frame. A PROPER rotation
    (det +1): ecliptic via obliquity eps, then rail = (x_ecl, z_ecl,
    -y_ecl). Rail longitude atan2(-z, x) = ecliptic longitude, +Y =
    ecliptic north -- the only embedding consistent with the rail's
    prograde motion sweeping longitude UP (system.cpp railAz)."""
    e = math.radians(OBLIQ_J2000)
    xe = x
    ye = y * math.cos(e) + z * math.sin(e)
    ze = -y * math.sin(e) + z * math.cos(e)
    return (xe, ze, -ye)

def _rail_from_eq(alpha_deg, delta_deg):
    a, d = math.radians(alpha_deg), math.radians(delta_deg)
    return _railvec_eq(math.cos(d) * math.cos(a),
                       math.cos(d) * math.sin(a),
                       math.sin(d))


def _railAz(a):
    """The loader's railAz(a) = R_Y(+a) (system.cpp): maps local +X to
    rail longitude +a = atan2(-z, x). 3x3 as row tuples."""
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))

def _mm(a, b):
    return tuple(tuple(a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]
                       for c in range(3)) for r in range(3))

def _mv(m, v):
    return tuple(m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2]
                 for r in range(3))


def wgccre_orientation(name, axial_tilt, incl, raan):
    """(#143) (tilt_azimuth, spin_phase0) in radians, placing the IAU
    rotation pole and prime meridian at t=0, expressed in the loader's
    #141 fields. Those fields live in the body's INERTIAL (orbital) frame
    -- the rot frame is a child of the inertial frame, so the loader
    composes initial_orient UNDER orient = railAz(raan) * R_X(incl) --
    hence the sky-frame pole/meridian are pre-rotated by inv(orient).
    axial_tilt stays the fact sheet's obliquity-to-orbit."""
    a, d, w, prograde = _wgccre_epoch(name)
    piau_sky = _rail_from_eq(a, d)
    # Node Q per the report's section 2: the node of the body's equator on
    # the ICRF EQUATOR at RA alpha0 + 90 deg (the reference plane there is
    # the celestial equator, not the ecliptic -- the text never says
    # ecliptic). W is measured easterly (about the IAU pole) from Q to the
    # prime meridian. Earth is the exception: its W is GMST, from the
    # vernal equinox (rail +X).
    if name == 'Earth':
        q = (1.0, 0.0, 0.0)
    else:
        ar = math.radians(a)
        q = _railvec_eq(-math.sin(ar), math.cos(ar), 0.0)
    # Prime meridian at epoch: Q rotated about the IAU pole by W
    # (Rodrigues; p and q are unit and perpendicular).
    wr = math.radians(w)
    cw, sw = math.cos(wr), math.sin(wr)
    px, py, pz = piau_sky
    cross = (py * q[2] - pz * q[1],
             pz * q[0] - px * q[2],
             px * q[1] - py * q[0])
    r0_sky = tuple(cw * q[i] + sw * cross[i] for i in range(3))
    # Sky frame -> the body's orbital frame.
    inv = _mm(_rx(-incl), _railAz(-raan))
    piau = _mv(inv, piau_sky)
    r0 = _mv(inv, r0_sky)
    # Our +Y (and axial_tilt) tracks the ANGULAR-MOMENTUM pole; for a
    # retrograde rotator (dW < 0) the IAU "north" pole is the opposite end
    # (Venus obliquity 177 deg, Uranus 98 deg).
    p = piau if prograde else tuple(-c for c in piau)
    # Tripwire: in the orbital frame the report's pole must sit at the
    # fact sheet's obliquity-to-orbit -- both describe the same physical
    # pole, and precession/data-source differences are arcmin-scale.
    ang = math.acos(max(-1.0, min(1.0, p[1])))
    assert abs(ang - axial_tilt) <= math.radians(1.0), \
        name + ': WGCCRE pole vs fact-sheet obliquity mismatch'
    assert math.sin(axial_tilt) > 1e-6, name + ': degenerate axial_tilt'
    tilt_az = math.atan2(-p[2], p[0])
    # Undo the loader's composition initial_orient =
    # railAz(tilt_az) * Rz_tilt(axial_tilt) * railAz(spin_phase0) to
    # recover spin_phase0 from the desired epoch prime-meridian direction.
    u = _mv(_railAz(-tilt_az), r0)
    ct, st = math.cos(axial_tilt), math.sin(axial_tilt)
    v = (ct * u[0] - st * u[1], st * u[0] + ct * u[1], u[2])
    spin_phase0 = math.atan2(-v[2], v[0])
    # Round-trip through the loader's exact composition: the authored
    # triple must land the pole and the meridian where the report says
    # (catches transcription and inversion slips, which are otherwise
    # silent numerical errors).
    rz = ((ct, st, 0.0), (-st, ct, 0.0), (0.0, 0.0, 1.0))
    io = _mm(_railAz(tilt_az), _mm(rz, _railAz(spin_phase0)))
    def _deg(a_, b_):
        dot = sum(a_[i] * b_[i] for i in range(3))
        return math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    assert _deg(_mv(io, (0.0, 1.0, 0.0)), p) < 1.5, \
        name + ': authored fields miss the report pole'
    assert _deg(_mv(io, (1.0, 0.0, 0.0)), r0) < 1.5, \
        name + ': authored fields miss the report meridian'
    return tilt_az, spin_phase0

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BASE = os.path.join(HERE, 'rss', 'html')
OUT  = os.path.join(ROOT, 'res', 'systems')

def read(name):
    with open(os.path.join(BASE, name), encoding='utf-8', errors='replace') as f:
        return f.read()

def strip_tags(s):
    s = re.sub(r'<[^>]+>', ' ', s)
    s = htmllib.unescape(s)
    return re.sub(r'\s+', ' ', s).strip()

def norm(s):
    """Lowercase alphanumeric-only form of a label, for loose matching."""
    return re.sub(r'[^a-z0-9]', '', strip_tags(s).lower())

def cells(row):
    out = []
    for m in re.finditer(r'<t([hd])[^>]*>(.*?)</t\1>', row, re.S):
        out.append(strip_tags(m.group(2)))
    return out

def table_rows(block):
    rows = []
    for tr in re.findall(r'<tr.*?</tr>', block, re.S):
        c = cells(tr)
        if c:
            rows.append(c)
    return rows

def num(s):
    """Parse a clean numeric cell ('30 x 20 x 17' -> mean); None if no number."""
    if s is None:
        return None
    s = s.strip().replace(',', '').replace('&nbsp;', ' ')
    if not s or s in ('S', '-', '\u2014'):
        return None
    vals = []
    for p in re.split(r'\s*[x\u00d7]\s*', s):
        m = re.search(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', p)
        if m:
            vals.append(float(m.group(0)))
    return (sum(vals) / len(vals)) if vals else None

def true_anomaly(M, e):
    """Solve Kepler's equation  M = nu - e*sin(nu)  for nu (Newton-Raphson)."""
    M = M % TWO_PI
    if M > math.pi:
        M -= TWO_PI
    nu = M + e * math.sin(M)
    for _ in range(40):
        f  = nu - e * math.sin(nu) - M
        fp = 1.0 - e * math.cos(nu)
        dn = f / fp
        nu -= dn
        if abs(dn) < 1e-13:
            break
    return nu

def section_values(h, marker):
    """{normalized-label: value} for the data block after `marker`."""
    i = h.find(marker)
    if i == -1:
        raise ValueError('marker not found: %r' % marker)
    m_tbl = re.search(r'<table.*?</table>', h[i:], re.S)
    m_pre = re.search(r'<pre.*?</pre>', h[i:], re.S)
    cands = [m for m in (m_tbl, m_pre) if m]
    if not cands:
        raise ValueError('no data block after %r' % marker)
    m = min(cands, key=lambda x: x.start())
    block = m.group(0)
    vals = {}
    if block.lstrip().startswith('<table'):
        for r in table_rows(block):
            if len(r) >= 2 and r[0].strip():
                v = num(r[1])
                if v is not None:
                    vals[norm(r[0])] = v
    else:
        for line in block.split('\n'):
            lt = strip_tags(line).replace(',', '')
            if not lt:
                continue
            # value = the number at the end of the line; label = everything before it
            m = re.search(r'(-?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*$', lt)
            if not m:
                continue
            v = float(m.group(1))
            label = lt[:m.start()]
            if label.strip():
                vals[norm(label)] = v
    return vals

def get(vals, label_substr):
    for k, v in vals.items():
        if label_substr in k:
            return v
    return None

def clean_name(name):
    """'Naiad (NIII)' -> 'Naiad'."""
    return name.split('(')[0].strip()

def is_named(primary):
    """A moon is 'named' if it has a proper name (not a provisional S/xxxx designation)."""
    p = primary.strip()
    if p.startswith(('S/', 'T/')):
        return False
    if re.match(r'^[ST]\d', p):
        return False
    return bool(re.search(r'[A-Za-z]', p))

def radius_from_cell(s):
    """Pluto 'Other Moons' Mean Diameter column: values are diameters, so radius = mean / 2."""
    if s is None:
        return None
    v = num(s)
    return (v / 2.0 * 1e3) if v is not None else None

def estimate_mass(radius_m, rho=SMALL_BODY_RHO):
    return rho * (4.0 / 3.0) * math.pi * radius_m ** 3

def parse_planet(page):
    h = read(page)
    bulk  = section_values(h, 'Bulk parameters')
    orb   = section_values(h, 'Orbital parameters')
    j2000 = section_values(h, 'Mean Orbital Elements')

    mass_kg   = get(bulk, 'mass1024') * 1e24
    radius_m  = get(bulk, 'volumetricmeanradius') * 1e3
    g         = get(bulk, 'surfacegravity')
    period_d  = get(orb, 'siderealorbitperiod')
    rot_hr    = get(orb, 'siderealrotationperiod')
    obliq_deg = get(orb, 'obliquitytoorbit')

    a_m  = get(j2000, 'semimajoraxis') * AU
    e    = get(j2000, 'orbitaleccentricity')
    i    = get(j2000, 'orbitalinclination') * D2R
    # The loader authors RAIL LONGITUDES (atan2(-z, x) about parent +Y;
    # system.cpp railAz), which are ecliptic longitudes, so the fact sheets'
    # physical angles go in UN-negated (#146): +X = vernal equinox,
    # +Y = ecliptic north, incl keeps its sign.
    node   = get(j2000, 'longitudeofascendingnode') * D2R
    varp   = get(j2000, 'longitudeofperihelion') * D2R
    L      = get(j2000, 'meanlongitude') * D2R

    period_s = period_d * DAY
    # Orbital sense lives in inclination (fact sheets use i > 90 deg, never a
    # signed period); load_system rejects negative rates (issue #139).
    assert period_s > 0.0, 'sidereal orbit period must be positive'
    w        = TWO_PI / period_s
    raan     = node
    omega    = varp - node                  # physical arg of periapsis
    # t=0 is 2000-01-01 00:00 but the fact-sheet elements are epoch
    # J2000.0 = that date at 12h: roll the mean anomaly back half a day
    # (matches _wgccre_epoch's d = -0.5; keeps midnight anchored at t=0).
    nu0      = true_anomaly((L - varp) - 0.5 * w * DAY, e)

    return dict(mass_kg=mass_kg, radius_m=radius_m, g=g,
                # obliquity > 90 deg encodes retrograde; keep the rate positive
                # or the body's calendar goes invalid (D -> 0).
                rot_s=(abs(rot_hr) * HR if rot_hr else None),
                obliquity=(obliq_deg * D2R if obliq_deg is not None else 0.0),
                a=a_m, e=e, i=i, raan=raan, omega=omega, nu0=nu0,
                w=w)

# ---------------------------------------------------------------------------
# the air a fact sheet states in PROSE (#685)
# ---------------------------------------------------------------------------
# A scale-height column hangs on ONE authored number: since #739 the file states
# sea_level_pressure, and the loader resolves the rest of it -- rho0 = p0/(g H), and
# through H = R T/(M g) the datum temperature, the sound speed and every density aloft.
# A mistyped datum is still invisible to every refusal we have: all those numbers agree
# with the wrong one perfectly. That is how #684 shipped, with Venus's datum written 92
# -- its surface PRESSURE in bar, typed into the density field the file used to author --
# which made the whole column 42% too fat and its dossier read 129 atm. #739 does not
# make that class of typo impossible on a scale-height body (the sheets publish a datum
# DENSITY, so this catalogue still starts from one and converts), but it does make the
# pressure the stated figure rather than a consequence -- which is the half the
# temperature field needs (#724). A guard has to compare two statements, and
# the second one is already on disk: the same fact sheets this generator scrapes
# for mass, radius and gravity state the air a few lines below the bulk table,
# in PROSE rather than a table, so section_values() never sees it.
#
# Bands, and what makes them that wide:
#   density 30%   three sheets state a SURFACE density (Venus ~65., Earth 1.217,
#                 Mars ~0.016 +/- 0.006 -- the sheet's own error bar is +/-37%)
#                 and four more state a density at their 1-bar level, which is
#                 these giants' datum (fact_air reads whichever the sheet has).
#                 The slip this catches measured +42%; before the 1-bar row was
#                 parsed, Saturn and Neptune sat -47% and +100% invisible (#700).
#   pressure 40%  p0 here is DERIVED (rho0 g H), so it carries two authored
#                 numbers. Mars sat +35% here until #698 fixed its column; the row
#                 reads +3.2% now. Mars's sheet also states its pressure as
#                 "variable from 4.0 to 8.7 mb depending on season" -- a 37%
#                 spread on the sheet's own side.
#   scale height 10%  a directly published figure, authored here in metres. This
#                 is the tight one: every shipped body is inside it, the widest
#                 reading being Saturn +0.8%. FACT_AIR_KNOWN, which used to hold
#                 two waivers, is empty. Quote the printed rows, not this comment
#                 -- they move when an authored number moves, and this line is a
#                 snapshot of one run.
#   base temp 10% a layered table's first row against the sheet's "Average
#                 temperature", which is a global mean rather than a sea-level
#                 datum. Earth reads +0.1%.
#   mass 30%     the sheet's "Total mass of atmosphere" row against 4 pi R^2 p0 / g
#                 -- the same column weighed from a second direction, which is the
#                 only way this guard can see p0, radius and g disagree rather than
#                 merely disagree with one number. Mars sat +33% here, which is what
#                 filed #698; that row reads +1.6% since the column was fixed.
#
# Skipped, with the reason kept beside the code so a future widening is a
# decision rather than a drift:
#   the giants' surface pressure -- their sheets say ">>1000 bars" while our
#   datum is the 1-bar level (Jupiter's column reads 112 kPa, ~1.12 bar).
#   Comparing those compares two different definitions of "surface";
#   any body that authors no density (Mercury, the Moon) -- nothing to contradict;
#   any body with no *fact.html (Titan comes from the giant-planet satellite
#   tables, which carry no air rows);
#   a layered body's mass row (Earth) -- this guard derives p0 from rho0 x g x H,
#   and a layer table authors no H; its datum is pinned in tests/test_scenarios;
#   and every world in gen_systems.py, whose air is a design with no sheet to
#   disagree with it.
FACT_AIR_BANDS = {'density': 0.30, 'p0': 0.40, 'scale_height': 0.10,
                  'base_T': 0.10, 'mass': 0.30}

# (body, quantity) -> (the authored value this waiver was written against, why).
# A waiver covers THAT number, not that body forever: change the authored value by
# more than WAIVER_TOL and the guard refuses again, with a message saying the waiver
# predates it. That matters most where a body has few guarded numbers -- a giant's
# sheet states a scale height, a 1-bar density and a molecular weight but no surface
# pressure, so such a world can have exactly two figures this guard can read.
WAIVER_TOL = 0.02
# Empty, and it was not always. The only two waivers here were Uranus's scale height
# and its 1-bar density, and both were written against one unmade decision -- what a
# giant's air top IS (#682). They said "a world change, filed as #695 rather than made
# at 11 pm"; #695 is that change, taken once the top was authored beside it, so this
# row now agrees with its sheet instead of arguing with it. Note what made the pair
# landable together: authoring `height` decouples the top from H, so H 20 -> 27.7 km
# no longer drags the rot frame, the spawn beds and the atlas disk behind it.
# Anything new that needs a waiver names the authored number it argues about, in the
# same breath as the number.
FACT_AIR_KNOWN = {}

AIR_PRESSURE_UNITS = {'bar': 1e5, 'bars': 1e5, 'mb': 1e2, 'millibar': 1e2,
                      'millibars': 1e2, 'microbar': 0.1, 'microbars': 0.1,
                      'torr': 133.322, 'pa': 1.0, 'pascal': 1.0, 'pascals': 1.0}

def sheet_text(raw):
    """One fact-sheet line with the markup folded the way the prose means it:
    <sup>3</sup> becomes ^3 (kg/m^3, 10^-15), <sub> is dropped (N2), tags become
    spaces and entities resolve. strip_tags() is not enough here: it turns
    'kg/m<sup>3</sup>' into 'kg/m 3', which reads as two numbers."""
    s = re.sub(r'<sup>(.*?)</sup>', r'^\1', raw, flags=re.S)
    s = re.sub(r'<sub>(.*?)</sub>', '', s, flags=re.S)
    s = re.sub(r'<[^>]+>', ' ', s)
    s = htmllib.unescape(s)
    s = re.sub(r'\s+', ' ', s).strip()
    # 'kg/m<span><sup>3</sup></span>' leaves 'kg/m ^3'; the caret belongs to what
    # precedes it, or the unit reads as two numbers.
    return re.sub(r'\s+\^', '^', s)

def sheet_line(h, label):
    """The text after `label` on the line that starts with it (case-insensitive).
    None if the sheet does not say."""
    for raw in h.split('\n'):
        s = sheet_text(raw)
        if s.lower().startswith(label.lower()):
            return s[len(label):].strip()
    return None

def sheet_quantity(text, want):
    """'~65. kg/m^3' -> (65.0, 65.0) in kg/m^3; '19.1 - 20.3 km' -> a pair in m;
    '>>1000 bars' and '<~5 x 10^-15 bar' -> None: the sheet declines to state it,
    which is not the same as stating zero."""
    if text is None:
        return None
    if re.search(r'>>|<<|<~|~<', text):
        return None
    t = text.replace('~', ' ').replace('+/-', ' ')
    # A thousands separator is not a decimal point: "25,000 kg" must not read as
    # 25 and 0. Only reachable today on bodies this guard skips, but a guard that
    # prints a number 1000x off is worse than one that prints none.
    t = re.sub(r'(?<=\d),(?=\d\d\d\b)', '', t)
    # '3 x 10^-15' first, or the exponent is swept up as a value of its own.
    sci = re.search(r'\d+(?:\.\d+)?\s*x\s*10\s*\^?\s*-?\d+', t)
    if sci:
        m = re.match(r'(\d+(?:\.\d+)?)\s*x\s*10\s*\^?\s*(-?\d+)', sci.group(0))
        vals = [float(m.group(1)) * 10.0 ** int(m.group(2))]
        t = t[:sci.start()] + t[sci.end():]
    else:
        vals = [float(v) for v in re.findall(r'\d+\.?\d*|\.\d+', t)]
    if not vals:
        return None
    unit = re.search(r'kg/m\^?3|kg|bars?|millibars?|microbars?|mb|torr|K\b|C\b|km\b|m\b', t)
    if unit is None:
        return None
    u = unit.group(0)
    if want == 'Pa':
        f = AIR_PRESSURE_UNITS.get(u.lower())
    elif want == 'm':
        f = 1e3 if u == 'km' else (1.0 if u == 'm' else None)
    elif want == 'kg/m^3':
        f = 1.0 if u.startswith('kg/m') else None
    elif want == 'kg':
        f = 1.0 if u == 'kg' else None
    else:
        f = 1.0 if u == 'K' else None
    if f is None:
        return None
    # A stated RANGE is "19.1 - 20.3 km" -- spaces around the dash, because
    # Mercury's "Average temperature: 440 K (167 C) (590-725 K, sunward side)"
    # has a hyphenated range in a parenthetical that is NOT the stated value.
    # Read the pair from the matched text rather than from vals[1], or a second
    # number anywhere in the line becomes the upper bound. (The value itself is
    # still the first number on the line and the unit the first recognised one:
    # these sheets are consistent, and a line like "10 miles (16 km)" would
    # parse wrong here. Nothing in utils/rss/html does that today.)
    rng = re.search(r'(\d+(?:\.\d+)?)\s+-\s+(\d+(?:\.\d+)?)', t)
    lo = float(rng.group(1)) if rng else vals[0]
    hi = float(rng.group(2)) if rng else vals[0]
    if hi < lo:
        lo, hi = hi, lo
    return lo * f, hi * f

def fact_air(page):
    """What a fact sheet states about the air, in SI: {density, p0, scale_height,
    base_T, upper_T, mass} -> (lo, hi) or absent.
    A body with no "Surface density" row may still state the density AT ITS 1-BAR
    LEVEL, and for the giants that IS our datum -- these sheets head their gravity
    column "Gravity (mean, 1 bar)" for the same reason, and check_fact_air already
    falls back on G*M/R^2 for them. So read that row as the density statement, in
    this order, and never both: the giants' sheets state no surface density because
    they have no surface. Their p0 comparison stays skipped either way -- their
    "Surface pressure" row says ">>1000 bars", which is a different definition of
    surface, not a number we are 1000x off. (#685, and the gap #700 found: Saturn
    and Neptune were authored -47 % and +100 % against these rows with nothing
    printing it, because the parser only looked for the terrestrial label.)

    #625 added two rows, both of them raw material for a layer TABLE rather than a
    scale height:
      upper_T  "Temperature at 0.1 bar:". Four giant sheets state it beside their
               1-bar temperature, and a pair of temperatures at a pair of pressures
               ten times apart is a profile, not a point -- author_air_profile()
               puts the seam where the sheet's OWN scale height says a decade of
               pressure sits, so the table between them is the sheet's, not ours.
      base_T   now also reads "Temperature at 1 bar:" (the giants' datum temperature)
               and "Surface temperature:" (Pluto states its air's temperature that
               way, as a range: 24 - 38 K).
    What is NOT parsed is the sheets' "Mean molecular weight:" row -- it carries no
    unit ("Mean molecular weight: 43.45", bare), and a unit-less number is the kind of
    loose read this parser has refused since it started. The mix's resolved molar mass
    lives in C++ and is pinned in tests/test_scenarios.cpp instead."""
    h = read(page)
    out = {}
    for key, labels, want in (('density', ('Surface density:', 'Density at 1 bar:'),
                               'kg/m^3'),
                              ('p0', ('Surface pressure',), 'Pa'),
                              ('scale_height', ('Scale height:',), 'm'),
                              ('base_T', ('Average temperature:', 'Temperature at 1 bar:',
                                          'Surface temperature:'), 'K'),
                              ('upper_T', ('Temperature at 0.1 bar:',), 'K'),
                              # The weight of the whole column stated in kg instead
                              # of Pa. Five sheets say it, three of them on bodies
                              # that have air here. See check_fact_air for why this
                              # is the row worth adding (#698).
                              ('mass', ('Total mass of atmosphere:',), 'kg')):
        for label in labels:
            q = sheet_quantity(sheet_line(h, label), want)
            if q is not None:
                out[key] = q
                break
    return out

def check_fact_air(planets, parsed, verbose=True):
    """Compare each authored datum against the sheet it came from. Returns the
    list of disagreements; empty means the catalogue and the sheets agree inside
    the bands above. `verbose` prints the table -- a guard that prints nothing is
    indistinguishable from a guard that checked nothing, so --check prints it."""
    rows, bad, waived = [], [], []
    for page, name, _seed, surface, _sea in planets:
        atmo = (surface or {}).get('atmosphere')
        if not atmo:
            continue
        sheet = fact_air(page)
        authored = {}
        rho0 = atmo.get('sea_level_density')      # nothing authors this since #739
        H = atmo.get('scale_height')
        # #739: what the FILE states is the datum PRESSURE. The sheets still state a
        # datum DENSITY (plus a "Surface pressure" row and a "Total mass of
        # atmosphere" row), so the guard keeps weighing all three -- on a scale-height
        # column the density falls back out of the pressure with no gas constant at
        # all, because the datum T that H implies is g H M / R and p0 = rho0 (R/M) T0,
        # so rho0 = p0 / (g H). A LAYER TABLE is the one comparison this guard loses:
        # it has no H to divide by, and recovering a density from its pressure needs
        # the molar mass, which utils/gas_mixes.py deliberately keeps in C++. Earth's
        # datum is pinned in tests/test_scenarios.cpp instead.
        p0_file = atmo.get('sea_level_pressure')
        if p0_file is not None:
            authored['p0'] = p0_file
        if rho0 is not None and p0_file is None:
            authored['density'] = rho0
        # The field this body's air is weighed in: G*M/R^2, which is what the loader
        # derives now that a file states gravity once (#708). The sheet's OWN gravity row
        # is not used here -- four sheets head that column "Gravity (mean, 1 bar)" and
        # leave that scrape None anyway -- it is weighed AGAINST this arithmetic by
        # check_scraped_gravity instead.
        body = parsed.get(name) or {}
        g = (G * body['mass_kg'] / (body['radius_m'] * body['radius_m'])
             if body.get('mass_kg') and body.get('radius_m') else None)
        if H is not None:
            authored['scale_height'] = H
            p0 = p0_file
            if p0 is None and rho0 is not None and g:
                p0 = rho0 * g * H
            if p0 is not None and g:
                # Invert with the SAME g the file's pressure was converted with:
                # anchor_sea_level_pressure weighs the column on the DATUM sphere
                # (#697), and this guard's g is the mean-radius figure. Every catalogue
                # body authors sea_level 0 today so
                # the two are one number -- but the moment one does not, an uncorrected
                # g here misses the density by exactly the (mean/datum)^2 factor, which
                # is the split #697 exists to prevent.
                sea = float((surface or {}).get('sea_level', 0.0) or 0.0)
                rr = body.get('radius_m')
                g_at = g if (sea <= 0.0 or not rr) else g * (rr / (rr + sea)) ** 2
                authored['density'] = p0 / (g_at * H)
                authored['p0'] = p0
                # The sheet's "Total mass of atmosphere" row is the SAME statement
                # as its pressure row -- the weight of the column per unit area IS
                # the surface pressure -- written in kg and spread over the whole
                # body. So it is the only published air number that ties our three
                # independent quantities together: p0 (from rho0 and H), the radius
                # and g. Mars is what it catches: its density row is 25% and its
                # pressure row 35% off, and its mass row says +33% -- three sheet
                # rows, one authored number, all three disagreeing the same way
                # (#698). Venus's three statements land inside 0.3% of each other,
                # which is what a healthy row looks like. Earth's mass row is out of
                # this guard's reach: it authors a layer table, so there is no
                # authored scale height to derive p0 from and its datum pressure is
                # pinned in tests/test_scenarios.cpp instead.
                if body.get('radius_m'):
                    authored['mass'] = (4.0 * math.pi * body['radius_m'] ** 2
                                        * p0 / g)
        layers = atmo.get('layers')
        if layers:
            authored['base_T'] = layers[0]['T']
        if not authored:
            # A rim and nothing else: the file draws an atmosphere shell but
            # authors no density and no height, so there is no column here for the
            # sheet to disagree with. Pluto is the case -- its sheet states 1.3 Pa
            # and a ~18 km scale height, and this catalogue ignores both. Print it
            # rather than dropping the row: an ignored air is a decision, and it
            # should look like one in the table.
            says = ', '.join('%s %.3g' % (k, v[0]) for k, v in sorted(sheet.items()))
            rows.append('%-9s no column authored; sheet states %s'
                        % (name, says or 'nothing'))
            continue
        cells = []
        for key in ('density', 'p0', 'scale_height', 'base_T', 'mass'):
            if key not in authored:
                continue
            got = sheet.get(key)
            if got is None:
                cells.append('%s -' % key)
                continue
            lo, hi = got
            band = FACT_AIR_BANDS[key]
            v = authored[key]
            if lo <= 0.0:
                # A band is a percentage, so it needs a positive base. A sheet
                # stating zero states nothing; say so rather than divide.
                cells.append('%s -' % key)
                continue
            # Compare against the NEAREST bound of what the sheet states. A point
            # estimate has lo == hi, so this stays the plain percentage; a stated
            # range ("19.1 - 20.3 km") reads 0% while we sit inside it, which is
            # what the verdict below already means.
            near = lo if v < lo else (hi if v > hi else v)
            off = (v - near) / near
            inside = (v >= lo * (1.0 - band) and v <= hi * (1.0 + band))
            cells.append('%s %+.1f%%' % (key, 100.0 * off))
            if inside:
                continue
            known = FACT_AIR_KNOWN.get((name, key))
            # A waiver covers the value it was written against, not the pair
            # forever. Move the authored number past WAIVER_TOL and the waiver is
            # stale: refuse, and name the value it pinned. A giant's sheet guards a
            # scale height and, since #700, a 1-bar density; the scale height is the
            # one that decides where their air ends, so it stays guarded -- an
            # argument filed against 20 km must not shelter a future 200 km typo.
            # FACT_AIR_KNOWN has been empty since #695 closed the only two waivers,
            # which is the goal rather than the default: an empty table only means
            # something because of this rule.
            if known is not None and abs(v - known[0]) <= WAIVER_TOL * known[0]:
                cells[-1] += ' (waived)'
                waived.append('%s %s: %s' % (name, key, known[1]))
                continue
            stale = ('' if known is None else
                     ' -- the waiver on this pair pins %s, so it predates this value'
                     % ('%.4g' % known[0]))
            bad.append('%s: authors %s %s, %s fact sheet states %.4g%s (%+.1f%%, band %d%%)%s'
                       % (name, key, '%.4g' % v, page, lo,
                          '' if hi == lo else '-%.4g' % hi,
                          100.0 * off, round(100 * band), stale))
            cells[-1] += ' !!'
        rows.append('%-9s %s' % (name, '  '.join(cells)))
    if verbose:
        for r in rows:
            print('  fact air  %s' % r)
        # Only a waiver that actually fired gets a line: a table of standing
        # excuses reads like the guard has given up on those bodies.
        for why in waived:
            print('  waived    %s' % why)
    return bad


# WIND: the prose row #685 left unparsed, which is the half #688 names as its own
# prerequisite ("Wind has no published source in utils/rss/ at all... Fix that first").
# Six of the sheets that carry air state a wind, right beside the density and
# scale-height rows fact_air() already reads. None of them reached the generator, so
# every authored wind row was a number nothing could argue with.
#
# What the pages state is not one shape, and the guard has to know that:
#   Venus   "Wind speeds: 0.3 to 1.0 m/s (surface)"                a level, a range
#   Mars    "Wind speeds: 2-7 m/s (summer), 5-10 m/s (fall),
#            17-30 m/s (dust storm) (Viking Lander sites)"         three ranges, one level
#   Earth   "Wind speeds: 0 to 100 m/s"                            a range, no level
#   Uranus  "Wind speeds: 0-250 m/s"                               a range, no level
#   Neptune "Wind speeds: 0-580 m/s"                               a range above the
#                                                                  loader's own net
#   Jupiter "Wind speeds" then "Up to 150 m/s (<30 degrees latitude)"
#           "Up to 40 m/s (>30 degrees latitude)"
#   Saturn  "Wind speeds" then "Up to 400 m/s (<30 degrees latitude)"
#           "Up to 150 m/s (>30 degrees latitude)"
#           a LATITUDE split, which #767's lat bands can hold -- and the reason a
#           GENERATED belt field is refused outright under these two sheets
# So the one comparison that is honest across all six is a CEILING: no component this
# file authors anywhere may exceed the fastest speed that body's own sheet states. That
# is the same class of slip as #684 typing a surface pressure in bar into a density
# field, and unlike the loader's flat 500 m/s net it is per body -- Earth's column would
# happily carry 500 m/s, which is 5x what Earth's sheet states.
#
# What is deliberately NOT checked, with the reason:
#   authored vs published MINIMUM -- a wind row is a steady profile, not a weather
#     forecast, and a body whose sheet states "0 to 100" is not claiming the air stops;
#   a body that states wind and authors none -- that is the gap the printed table keeps
#     visible rather than a refusal, because authoring a row owes an e2e case that pins
#     what it does (tests/test_scenarios.cpp's kWindsAuthored) and half these worlds have
#     no flight case to re-run yet.
FACT_WIND_KNOWN = {
    # (body, row altitude) -> (the component this waiver was written against, why).
    # Same staleness rule as FACT_AIR_KNOWN: move the row past WAIVER_TOL and the
    # waiver stops covering it.
    ('Venus', 65000.0): (100.0, "cloud-top superrotation. venusfact.html's wind row "
                                "stops at its surface, so this figure is cited from "
                                "published Venus climatology -- see the block that "
                                "authors it, and note the deck this file renders is "
                                "at 55 km, not 65 km (#714)"),
    # Titan is the other side of the same coin: not one row above a ceiling, but a whole
    # table under a page that states nothing. altitude None means "this body's whole
    # table", and the pinned figure is then its FASTEST row -- there is no sheet to weigh
    # a row against, so a waiver keyed on one row would cover the others by accident. The
    # first version of that branch asked about the first row written, and a probe showed
    # what that meant: reorder the table, or add a 900 m/s row, and it rode through on a
    # waiver written for the deck (the loader's only net is atmo.h:kWindMaxSpeed, a flat
    # 1000 m/s). It stays a waiver rather than a rule change because the next body to do
    # this should have to say why too.
    ('Titan', None): (100.0, "no page in this corpus states a wind for Titan -- it has no "
                             "fact page of its own, and the satellite sheet its air comes "
                             "from has no wind line. All three rows are published "
                             "climatology, cited where they are authored (#772)"),
}

# 'Wind speeds' is the label; the speeds are the numbers that end a 'm/s' phrase.
# A range ("0 to 100 m/s", "2-7 m/s") and a ceiling ("Up to 150 m/s") all end the same
# way, so one pattern reads the top of each, which is all a ceiling guard needs.
FACT_WIND_LABEL = 'wind speeds'
WIND_MPS = re.compile(r'(\d+(?:\.\d+)?)\s*m/s')
# Jupiter and Saturn do not state ONE wind: they state a ceiling PER LATITUDE BAND
# ("Up to 150 m/s (<30 degrees latitude) Up to 40 m/s (>30 degrees latitude)"). That is
# latitude structure -- and the thing a generated belt field (#775) would replace with
# derived numbers. Named separately so the guard can tell "the page gave us a range" from
# "the page gave us a map". This is a Jupiter/Saturn-shaped net, not a general one: it
# matches "<speed> m/s (<op> <lat> degrees latitude)" and would miss "30-60 degrees
# latitude", "near the equator", or a speed written with a thousands comma. Widening it is
# worth doing when a sheet that phrased it another way starts mattering; self_test_fact_wind
# below pins what this one catches, so a rewrite cannot silently lose the two it has.
WIND_MPS_LAT = re.compile(r"(?i)up to ([\d.]+)\s*m/s\s*\(\s*([<>]=?)\s*([\d.]+)\s*"
                          r"(?:degrees|deg)?\s*latitude\s*\)")


def fact_wind(page):
    """What a fact sheet states about wind: {'max': m/s, 'text': the page's own words,
    'latbands': [(speed, op, lat_deg), ...] possibly empty}, or {} when it states none.
    Jupiter and Saturn put the label on a line of its own and
    the two latitude bands under it, so lines keep being pulled in while they state a
    speed and are not a new labelled row. The raw text comes back because the printed
    row should quote the page, not a parsed guess."""
    h = read(page)
    lines = h.split('\n')
    for i, raw in enumerate(lines):
        s = sheet_text(raw)
        if not s.lower().startswith(FACT_WIND_LABEL):
            continue
        text = s[len(FACT_WIND_LABEL):].strip().lstrip(':').strip()
        j = i + 1
        while j < len(lines):
            t = sheet_text(lines[j])
            if 'm/s' not in t or ':' in t:
                break
            text = (text + ' ' + t).strip()
            j += 1
        vals = [float(v) for v in WIND_MPS.findall(text)]
        if not vals:
            # The label exists and no number follows it: the page declines to state a
            # speed. Not the same as stating zero.
            return {}
        lat = [(float(m.group(1)), m.group(2), float(m.group(3)))
               for m in WIND_MPS_LAT.finditer(text)]
        return {'max': max(vals), 'text': text, 'latbands': lat}
    return {}


def check_fact_wind(planets, verbose=True, parsed=None):
    """Compare every authored wind row against the wind that body's fact sheet states.
    Returns the disagreements. Prints one line per body with air, including the bodies
    whose sheet states a wind and which author none -- a guard that prints only the
    failures is indistinguishable from one that found none."""
    rows, bad, waived = [], [], []
    scaling = []
    for page, name, _seed, surface, _sea in planets:
        atmo = (surface or {}).get('atmosphere')
        if not atmo:
            continue
        sheet = fact_wind(page)
        wind = atmo.get('wind') or []
        gen = atmo.get('wind_gen')
        comps = [(float(w['alt']), max(abs(w.get('east', 0.0)), abs(w.get('north', 0.0))))
                 for w in wind]
        top = max((c for _alt, c in comps), default=None)
        quote = sheet.get('text', '')
        if len(quote) > 58:
            quote = quote[:55] + '...'
        rows.append('%-9s sheet %-22s authors %s'
                    % (name,
                       ('%g m/s  %s' % (sheet['max'], quote)) if sheet else 'none',
                       ('%g m/s, %d rows%s'
                        % (top, len(comps), ' [%s]' % gen if gen else ''))
                       if comps else 'none'))
        # A GENERATED column is still bounded by this guard, for a reason worth stating:
        # windGenerate scales every belt by amp <= 1 and the equatorial belt carries the
        # anchors' own values, so the widest wind in the emitted table IS the column's top
        # row, weighed here. What the ceiling cannot weigh is WHERE a speed lands, and a
        # sheet that bounds wind BY LATITUDE says more than a generated field may claim.
        if gen and sheet.get('latbands'):
            bad.append('%s: the sheet states wind per latitude band (%s) while the file'
                       ' generates its own belts -- author the published bands (#767/#772),'
                       ' because a generated field puts derived numbers inside a latitude'
                       ' bound the page set'
                       % (name, '; '.join('%g m/s %s%g deg lat' % b
                                          for b in sheet['latbands'])))
        # #775's diagnostic, printed and never enforced: what the rotation-based scaling
        # says about the latitudinal structure this body's air can hold. The table on #775
        # shows it tracks Venus (no belts at all), Jupiter and Saturn and overpredicts Mars
        # and the ice giants, so it is a hint for whoever authors a wind -- not a law the
        # loader may act on. ASKED, not shipped: atmo.h:windGenerate caps the count at the
        # 64-row budget and nowhere else now that the amplitude is normalized over whatever
        # count survives, so this number is the raw Rhines ask.
        if comps and parsed is not None:
            d = parsed.get(name) or {}
            R, per = d.get('radius_m'), d.get('rot_s')
            if R and per:
                # U the way windGenerate takes it -- the fastest hypot in the column, not
                # the per-row component max the sheet guard above uses.
                u = max(math.hypot(float(w.get('east', 0.0)), float(w.get('north', 0.0)))
                        for w in wind)
                if u > 0.0:
                    om = 2.0 * math.pi / per
                    ro = u / (2.0 * om * R)
                    beta = 2.0 * om * math.cos(math.radians(30.0)) / R
                    jet = math.degrees(math.sqrt(u / beta) / R)
                    asked = int(math.floor(90.0 / jet))   # windGenerate's count, same floor
                    scaling.append('%-9s Ro %-9.4g Rhines scale %-8.4g deg'
                                   ' -> %d belts/hemi asked' % (name, ro, jet, asked))
        if not comps:
            continue
        if not sheet:
            # Nothing published to weigh a row against, so this waiver is per BODY and covers
            # the whole table: its pinned figure is the fastest row it was written for, and a
            # row past that reddens the check. Keying it on one row's altitude -- which is
            # what the first version did -- let a reordered table, or a 900 m/s row nobody
            # argued for, ride through on a waiver written for the deck.
            m = ('%s authors wind rows and %s states no wind at all' % (name, page))
            known = FACT_WIND_KNOWN.get((name, None))
            if known is None:
                bad.append(m)
                continue
            if top > known[0] * (1.0 + WAIVER_TOL):
                bad.append('%s: its fastest row is %g m/s, past the %g m/s this body\'s '
                           'waiver was written against -- a table under a page that states '
                           'nothing still owes a ceiling argument' % (m, top, known[0]))
                continue
            waived.append('%s (%d rows, at most %g m/s): %s'
                          % (m, len(comps), top, known[1]))
            continue
        # A sheet that bounds wind BY LATITUDE says more than how fast: it says WHERE, and the
        # ceiling loop below cannot see that. Moving Jupiter's or Saturn's +/-30 deg split out
        # to +/-45 keeps every row under the published top and loads fine -- the split is what
        # the page found, so weigh the band EDGES against it too (#772). Two refusals:
        #   structure erased -- the table answers for the whole sphere where the page did not;
        #   structure moved -- a band edge sitting nowhere the page puts one.
        # A whole-sphere row sharing a table with banded rows is legal: banding the deck and
        # leaving a high level global is authoring, not erasing.
        if sheet.get('latbands'):
            pub = sorted(set(abs(b[2]) for b in sheet['latbands']))
            banded = [w for w in wind if w.get('lat') is not None]
            if not banded:
                bad.append('%s: the sheet bounds wind BY LATITUDE (%s) while this file answers'
                           ' for the whole sphere -- author the published split (#767/#772)'
                           % (name, '; '.join('%g m/s %s%g deg lat' % b
                                              for b in sheet['latbands'])))
            for w in banded:
                for edge in (float(w['lat'][0]), float(w['lat'][1])):
                    if abs(abs(edge) - 90.0) < 1e-9:
                        continue          # a pole half is the tiling, not a published bound
                    if not any(abs(abs(edge) - p) <= 2.0 for p in pub):
                        bad.append('%s: a wind band edge at %+g deg matches no latitude bound'
                                   ' %s states (%s)'
                                   % (name, edge, page,
                                      ', '.join('%+g deg' % p for p in pub)))
        for alt, comp in comps:
            if comp <= sheet['max']:
                continue
            known = FACT_WIND_KNOWN.get((name, alt))
            if known is not None and abs(comp - known[0]) <= WAIVER_TOL * known[0]:
                waived.append('%s wind at %g m: %s' % (name, alt, known[1]))
                continue
            stale = ('' if known is None else
                     ' -- the waiver on this row pins %g m/s, so it predates this value'
                     % known[0])
            bad.append('%s: authors %g m/s of wind at alt %g m; %s states at most '
                       '%g m/s (%s)%s'
                       % (name, comp, alt, page, sheet['max'], sheet['text'], stale))
    if verbose:
        for r in rows:
            print('  fact wind %s' % r)
        for s in scaling:
            print('  wind scaling %s' % s)
        for why in waived:
            print('  waived    %s' % why)
    return bad


def self_test_fact_wind():
    """Prove the wind guard can fail, which is the only way to know it is a gate and not
    decoration. This file has no test harness of its own, so the mutation runs here against
    the sheets cached in the repo (utils/rss/html, tracked): Jupiter's page states wind per
    latitude band, Neptune's states a bare range, the satellite page Titan comes from states
    none. Returns the ways the guard has stopped behaving -- empty means every mutation
    landed."""
    fails = []
    jup = fact_wind('jupiterfact.html')
    nep = fact_wind('neptunefact.html')
    if len(jup.get('latbands') or []) != 2:
        fails.append('jupiterfact.html no longer parses as two latitude bands: %r' % (jup,))
    if nep.get('latbands'):
        fails.append('neptunefact.html\'s bare range now parses as latitude bands: %r'
                     % (nep,))
    # Banded rows, so the probes below say something about GENERATION and the latitude split
    # rather than about row speeds. They are inside Jupiter's own +/-30 deg bound.
    rows = [{'alt': 0.0, 'east': 10.0, 'lat': [-30.0, 30.0]},
            {'alt': 60000.0, 'east': 90.0, 'lat': [-30.0, 30.0]}]
    # A generated table under a sheet that bounds wind BY LATITUDE must be refused...
    got = check_fact_wind([('jupiterfact.html', 'Jupiter', 1.0,
                            {'atmosphere': {'wind': rows, 'wind_gen': 'rhines'}}, 0.0)],
                          verbose=False)
    if not any('per latitude band' in m for m in got):
        fails.append('a generated table under Jupiter\'s banded sheet was not refused: %r'
                     % (got,))
    # ... the same rows authored by hand must not be (both under the sheet's 150 m/s and
    # inside the latitude band it states),
    got = check_fact_wind([('jupiterfact.html', 'Jupiter', 1.0,
                            {'atmosphere': {'wind': rows}}, 0.0)], verbose=False)
    if got:
        fails.append('the same rows authored by hand were refused: %r' % (got,))
    # ... and a generated table under a bare RANGE is legal, since the sheet bounds speed
    # and says nothing about where it lands.
    got = check_fact_wind([('neptunefact.html', 'Neptune', 1.0,
                            {'atmosphere': {'wind': rows, 'wind_gen': 'rhines'}}, 0.0)],
                          verbose=False)
    if any('latitude band' in m for m in got):
        fails.append('a bare-range sheet refused a generated table: %r' % (got,))
    # The other kind of waiver: a body under a page that states NOTHING is covered up to the
    # speed its waiver names and no further. This probes with the fastest row NOT first,
    # because the first version of that branch looked only at the first row written and so
    # covered every other row by accident -- a 900 m/s row nothing argued for, straight past
    # the loader's flat kWindMaxSpeed net and into the game.
    got = check_fact_wind([('saturniansatfact.html', 'Titan', 1.0,
                            {'atmosphere': {'wind': [{'alt': 60000.0, 'east': 900.0},
                                                     {'alt': 0.0, 'east': 1.0}]}}, 0.0)],
                          verbose=False)
    if not any('waiver was written against' in m for m in got):
        fails.append('a 900 m/s row under a no-sheet waiver was not refused: %r' % (got,))
    # And the same shape inside the waiver must pass, or the guard is just refusing moons.
    got = check_fact_wind([('saturniansatfact.html', 'Titan', 1.0,
                            {'atmosphere': {'wind': [{'alt': 20000.0, 'east': 30.0},
                                                     {'alt': 0.0, 'east': 1.0}]}}, 0.0)],
                          verbose=False)
    if got:
        fails.append('a no-sheet table inside its waiver was refused: %r' % (got,))
    # The banded sheet says WHERE as well as how fast, and the ceiling loop cannot see that.
    # Widening Jupiter's own +/-30 deg split to +/-45 keeps every row AT the published top,
    # so nothing else objects -- and the game ships an equatorial jet 15 deg too wide.
    got = check_fact_wind([('jupiterfact.html', 'Jupiter', 1.0, {'atmosphere': {'wind': [
        {'alt': 0.0, 'east': 150.0, 'lat': [-45.0, 45.0]},
        {'alt': 0.0, 'east': 40.0, 'lat': [-90.0, -45.0]},
        {'alt': 0.0, 'east': 40.0, 'lat': [45.0, 90.0]}]}}, 0.0)], verbose=False)
    if not any('matches no latitude bound' in m for m in got):
        fails.append('a wind band edge the sheet never put there was not refused: %r' % (got,))
    # And a table that drops the split entirely, answering for the whole sphere where the
    # page bounded it by latitude. Under the sheet's top, loads fine, flattens the finding.
    got = check_fact_wind([('jupiterfact.html', 'Jupiter', 1.0,
                            {'atmosphere': {'wind': [{'alt': 0.0, 'east': 40.0}]}}, 0.0)],
                          verbose=False)
    if not any('bounds wind BY LATITUDE' in m for m in got):
        fails.append('a whole-sphere table under a banded sheet was not refused: %r' % (got,))
    return fails


# #625 (roadmap rung A, the catalogue half): seven of this file's airs stated a SCALE
# HEIGHT, and the trio lock (#686) had already spent it -- T0 = g H M / R, so Jupiter's
# H 27 km read 186.73 K beside its sheet's own 165 K, Saturn's 60 km read 173.51 K
# beside 134 K, and the load-time thermal audit printed "two rows, -39.51 K apart"
# (#738's family). A scale height cannot state a temperature profile, so no later rung
# can vary one. What the sheets state instead is better than what we were typing:
#   * the giants state a PAIR -- "Temperature at 1 bar" and "Temperature at 0.1 bar" --
#     beside a scale height. A pair of temperatures ten times apart in pressure IS a
#     profile, so the lapse and the seam altitude below are derived from those three
#     rows, not invented (author_air_profile, just after this guard);
#   * Venus and Mars state one temperature and a pressure. The profile's SHAPE is then
#     ours, and each one's is argued where it is authored;
#   * Pluto's rows already agree with each other (its ~18 km scale height at N2 and its
#     own gravity is 37.6 K, the warm end of its stated 24 - 38 K) -- converting it is
#     the no-op that proves the rule rather than the exception.
# Datum pressure: the sheet's "Surface pressure" row where it states one; otherwise the
# 1 bar level, which is what a giant's density, gravity and temperature rows are all
# stated AT (the sheets head their gravity column "Gravity (mean, 1 bar)"). That is a
# convention, not a measurement, and it replaces what used to be a plucked rho0 x g x H.
# The ceiling is the band gate's answer, not the sheet's -- a sheet's air top is a
# rendering limit, and #682's gate is what makes an orbit decay somewhere (#702).
SOLAR_AIR_SHAPE = {
    # Venus: -7.8 K/km to a 65 km tropopause at 230 K. Not in utils/rss/ (our Venus
    # sheet stops at its surface rows) -- it is the published Venus profile, cited the
    # same way the 100 m/s superrotation row at 65 km already is, and the two now
    # describe the same deck instead of a 737 K floor under it.
    'Venus':   {'lapse': -0.0078, 'trop': 65000.0, 'lid_T': 230.0, 'top': 116000.0},
    # Mars: -4.0 K/km, a lid at 148 K. Its own convective limit is g/cp = 4.5 K/km, and
    # 148 K is where CO2 deposits at its own 636 Pa -- so the column stops cooling
    # before it can snow its own air out, the same argument #700 used against Duna's
    # old 61.6 K sky.
    'Mars':    {'lapse': -0.0040, 'trop': 16500.0, 'top': 60000.0},
    # The four giants: no authored shape at all -- the seam comes from their two
    # temperature rows and their own scale height row.
    'Jupiter': {'top': 245000.0},
    'Saturn':  {'top': 400000.0},
    'Uranus':  {'top': 250000.0},
    'Neptune': {'top': 200000.0},
    # Pluto: one isothermal level. Its sheet's SECOND scale-height row ("~50 km above
    # 30 km") is a mesosphere that starts above this body's ceiling, so it is left
    # unread rather than turned into a 100 K lid at 1.3 Pa; the warm end of its stated
    # 24 - 38 K is the end its own 18 km row supports.
    'Pluto':   {'lapse': 0.0, 'top': 20000.0, 'T_end': 'hi'},
}

# The airs that keep a scale height, named so the only way to get one is to say why.
SOLAR_AIR_EXPONENTIAL = {
    # The one air in the catalogue with no *fact.html to read (it comes from the
    # giant-planet satellite pages, which state no air rows). Its H 20 km implies 90.54 K
    # at this N2/CH4 mix and its own 1.354 m/s2, which is inside the published 94 K
    # neighbourhood, so nothing absurd is being said -- but its temperature, its datum
    # pressure and its ceiling (the defaulted 10 H = 200 km) are one invented number, and
    # that makes it the last body in the catalogue where an H is a thermometer.
    'Titan': 'no fact page; its implied 90.5 K is already a sane air',
}

def _atmo_of(bodies, name):
    for b in bodies:
        if b['name'] == name:
            return (b.get('surface') or {}).get('atmosphere') or {}
    return {}

def check_air_coverage(bodies, verbose=True):
    """Every air this catalogue emits states a profile, unless it is named in
    SOLAR_AIR_EXPONENTIAL with the reason it still states a scale height. The twin of
    gen_systems.py's `air == set(KSP_AIR)`: without it, a body added to `planets` or to
    MOON_ATMOS gets an exponential by default, which is #625's bug wearing a new name --
    one H doing double duty as a column and as the body's only temperature statement. Note
    the check is on the SCALE HEIGHT, not on SOLAR_AIR_SHAPE membership: Earth's US76 table
    is typed in the surface block rather than emitted by author_air_profile, and it is
    exactly as authored."""
    air, bad = [], []
    for b in bodies:
        atmo = (b.get('surface') or {}).get('atmosphere')
        if not atmo:
            continue
        if not (atmo.get('sea_level_pressure', 0.0) > 0.0
                or atmo.get('sea_level_density', 0.0) > 0.0):
            continue
        air.append(b['name'])
        if 'scale_height' in atmo and b['name'] not in SOLAR_AIR_EXPONENTIAL:
            bad.append('%s has air and states a scale_height, so its temperature is a '
                       'by-product of it (#625). Author a profile, or name the body in '
                       'SOLAR_AIR_EXPONENTIAL with the reason it keeps one' % b['name'])
        if 'scale_height' not in atmo and 'layers' not in atmo:
            bad.append('%s has air, and states neither a scale height nor a profile' %
                       b['name'])
    for name in SOLAR_AIR_EXPONENTIAL:
        if name not in air:
            bad.append('SOLAR_AIR_EXPONENTIAL names %s, which emits no air -- the waiver '
                       'is stale' % name)
    if verbose:
        print('  air cover %d bodies: %d wear a profile, %d state a scale height (%s)'
              % (len(air), sum(1 for n in air if 'layers' in _atmo_of(bodies, n)),
                 sum(1 for n in air if n in SOLAR_AIR_EXPONENTIAL),
                 '; '.join('%s: %s' % (n, SOLAR_AIR_EXPONENTIAL[n])
                           for n in SOLAR_AIR_EXPONENTIAL if n in air) or 'none'))
    return bad


def author_air_profile(name, page, surface, data, verbose=True):
    """Turn one body's sheet rows into an authored layer table, and print what each row
    cost. Returns the complaints; empty means every number came from a row."""
    atmo = (surface or {}).get('atmosphere')
    if not atmo or name not in SOLAR_AIR_SHAPE:
        return []
    shape = SOLAR_AIR_SHAPE[name]
    # One source per profile. A body in this table that also carries an exponential (or
    # a hand-typed table) would have two, and only one of them would ever be read.
    assert 'scale_height' not in atmo and 'layers' not in atmo and 'height' not in atmo, (
        '%s: SOLAR_AIR_SHAPE authors this air\'s profile AND its ceiling, so its '
        'surface block must state neither scale_height, layers, nor height' % name)
    sheet = fact_air(page)
    bad = []
    # Which rows a body needs depends on how its shape is got. A FITTED body (the four
    # giants) takes its lapse from the sheet's scale-height row; a body whose shape is
    # authored (Venus, Mars, Pluto) never reads that row, so refusing for want of it would
    # be a guard naming a number it does not use -- and a re-scraped page that dropped the
    # row would stop Venus building with a message that is false about Venus.
    fitted = 'lapse' not in shape
    for key in ['base_T'] + (['scale_height'] if fitted else []):
        if sheet.get(key) is None:
            bad.append('%s: %s states no %s row, and this body\'s profile is built '
                       'from it' % (name, page, key))
    if bad:
        return bad
    lo, hi = sheet['base_T']
    if hi > lo:
        # A stated RANGE is not a mean (#687/#722's rule), so pick an end and say which.
        end = shape.get('T_end')
        if end not in ('lo', 'hi'):
            bad.append('%s: %s states %s K as a range and SOLAR_AIR_SHAPE does not name '
                       'which end this air uses' % (name, page, '%g - %g' % (lo, hi)))
            return bad
        T0 = hi if end == 'hi' else lo
    else:
        T0 = lo
    # The sheet's own scale height (averaged over its stated span) is what sets the lapse:
    # L = (T0 / H_mid) ln(Tu/T0) / ln(10), which is the lapse a column would have if that H
    # were the datum's isothermal scale height (H = Rs T0 / g). The seam then follows from
    # the two temperatures: seam = (Tu - T0) / L = ln(10) H_mid (T0-Tu)/(T0 ln(T0/Tu)),
    # 1.84-2.02 x H_mid across the four giants -- NOT ln(10) H_mid, which is where an
    # ISOTHERMAL level would put a decade of pressure. And on these four columns the decade
    # does not land at the seam at all, because the emitted column's scale height comes from
    # the authored mix and is shorter than the sheet's H. This is a FIT, and its error is
    # exactly the sheet's own internal inconsistency, measured (utils/atmo_profile.py
    # reading the emitted file): where our column reaches the sheet's stated cold
    # temperature it is at 9.1 kPa on Uranus and 10.9 kPa on Neptune -- 9% either side, the
    # closest in the catalogue, because their H rows agree with their own T/M/g to a few
    # percent -- but 7.4 kPa on Jupiter (-26%) and 5.2 kPa on Saturn (-48%), whose H rows
    # imply 186 K and 172 K rather than the 165 K and 134 K their temperature rows state.
    # Filed as #740; the clean derivation needs the resolved molar mass on this side of the
    # boundary, which utils/gas_mixes.py deliberately keeps in C++. What is NOT a fit: the
    # two temperatures and the 1 bar datum, which are rows.
    H = sheet['scale_height']
    Hmid = 0.5 * (H[0] + H[1])
    # Datum pressure: the sheet's own surface-pressure row where it states one. The four
    # giants state "Surface pressure: >>1000 bars", which is the deep interior and not the
    # level this file's datum is, so the parser leaves it unparsed and the 1 bar level --
    # the level the giants' density, gravity and temperature rows are all stated at, and
    # what their gravity column header says ("Gravity (mean, 1 bar)") -- is what this
    # defaults to. Named in the print, because a default and a reading that print the same
    # number are indistinguishable in the log and not in the file.
    p0row = sheet.get('p0')
    if p0row:
        p0, p0how = p0row[0], "the sheet's surface-pressure row"
    else:
        p0, p0how = 100000.0, 'defaulted to the 1 bar level the sheet states its rows at'
    if 'lapse' in shape:
        L = shape['lapse']
        trop = shape.get('trop')
        if trop is None:
            layers = [{'alt': 0.0, 'T': T0, 'lapse': L}]
            seam, Tu, how = None, None, 'one isothermal level'
        else:
            # Same refusal gen_system.py makes for the KSP airs: a lid at or above the
            # ceiling is a level the loader will reject (system.cpp refuses it), and one
            # below zero is a table whose first level is not the datum.
            assert 0.0 < trop < shape['top'], (
                '%s: its lid sits at %.0f m, outside the 0..%.0f m column' % (
                    name, trop, shape['top']))
            Tu = shape.get('lid_T', round(T0 + L * trop, 2))
            layers = [{'alt': 0.0, 'T': T0, 'lapse': L},
                      {'alt': trop, 'T': Tu, 'lapse': 0.0}]
            seam, how = trop, 'authored lapse'
    else:
        if sheet.get('upper_T') is None:
            bad.append('%s: %s states one temperature, so this profile needs an authored '
                       'lapse in SOLAR_AIR_SHAPE' % (name, page))
            return bad
        # A RANGE here is the same trap as a range on the datum row, so the same rule:
        # pick an end out loud. Inert today (all four giants state one number each).
        ulo, uhi = sheet['upper_T']
        if uhi > ulo:
            end = shape.get('T_end')
            if end not in ('lo', 'hi'):
                bad.append('%s: %s states its 0.1 bar temperature as %s K and '
                           'SOLAR_AIR_SHAPE does not name which end this air uses'
                           % (name, page, '%g - %g' % (ulo, uhi)))
                return bad
            Tu = uhi if end == 'hi' else ulo
        else:
            Tu = ulo
        L = round((T0 / Hmid) * math.log(Tu / T0) / math.log(10.0), 8)
        seam = round((Tu - T0) / L, 1)
        assert 0.0 < seam < shape['top'], (
            '%s: the fitted seam sits at %.1f m, outside the 0..%.0f m column -- the two '
            'temperature rows and the scale-height row do not describe one air' % (
                name, seam, shape['top']))
        layers = [{'alt': 0.0, 'T': T0, 'lapse': L},
                  {'alt': seam, 'T': Tu, 'lapse': 0.0}]
        how = "from the sheet's 1 bar / 0.1 bar / scale-height rows"
    atmo['sea_level_pressure'] = p0
    atmo['layers'] = layers
    atmo['height'] = shape['top']
    if verbose:
        print('  air shape %-9s T0 %7.2f K  p0 %11.4g Pa  %s (%s)'
              % (name, T0, p0, how, p0how))
        if seam is not None:
            print('            %-9s lapse %7.3f K/km to %8.0f m -> %7.2f K lid, '
                  'top %8.0f m' % ('', 1000.0 * L, seam, Tu, shape['top']))
        else:
            print('            %-9s lapse %7.3f K/km throughout, top %8.0f m'
                  % ('', 1000.0 * L, shape['top']))
    return bad

# ---------------------------------------------------------------------------
# What the sheets say about the GROUND's thermal state (#687), and what we author
# beside it. A star's output is ONE authored number that every body's flux derives
# from, so the guard is the same shape as #685's for air: read the published rows,
# print the comparison, refuse when it moves.
# ---------------------------------------------------------------------------

def fact_surface(page):
    """What a fact sheet states about the thermal state of the GROUND -- or, on a
    giant, of the 1-bar level that its air datum already IS:
      albedo      Bond albedo (dimensionless; the Bulk-parameters row)
      irradiance  W/m^2 crossing a surface NORMAL to the beam, at that body
      bb_T        K, the sheet's own black-body temperature
      avg_T       K, the prose 'Average temperature:' line -- or, on a giant, its
                  'Temperature at 1 bar', which is the level our datum IS (#689)

    The sheet's black-body row turns out to BE solar.h's law with this table's own
    Bond albedo -- Earth 254.0 K off 1361.0 W/m^2 and 0.294, Mercury 439.6 K off
    9082.7 and 0.068 -- so the /4 (redistribute over the sphere) and the BOND (not
    geometrical) albedo are the corpus's convention, not our invention. That is what
    lets check_fact_surface() use those rows as cross-checks on the one number the
    corpus cannot state: the star's own temperature.

    A missing key means the sheet declines to say, which is NOT the same as saying
    zero; the loader dashes an unauthored field for exactly that reason."""
    h = read(page)
    v = section_values(h, 'Bulk parameters')
    out = {'albedo': v.get('bondalbedo'),
           'irradiance': v.get('solarirradiancewm2'),
           'bb_T': v.get('blackbodytemperaturek'),
           # The sheet's "Diurnal temperature range" row, which is a PAIR of kelvin ends
           # ("283 K to 293 K"), and on Venus a stated nothing ("~0"). It is the only place
           # the corpus talks about a ground SWINGING, which is what ground_model's one
           # knob is fitted to. None means the sheet says nothing about a swing.
           'diurnal': parse_diurnal_pair(sheet_line(h, 'Diurnal temperature range')),
           # Pluto spells its ground a range and puts the unit after both ends ("Surface
           # temperature: 24 - 38 K"), which the "N K" pattern above reads as ONE number and
           # throws the 24 away. It is the only page in the corpus that does this, and it is
           # the only body whose ground the corpus states at all without also stating a mean
           # -- which is exactly the case where a range prices the anchor and the knob
           # instead of testing them (ground_model.pair_solve).
           'surf_range': parse_kelvin_range(sheet_line(h, 'Surface temperature:'))}
    # The prose line, deliberately NOT sheet_quantity(): that one reads a stated
    # range, and Mercury writes 'Average temperature: 440 K (167 C) (590-725 K,
    # sunward side)' -- the hyphenated pair is the day-side extreme, not the average.
    # The first number followed by K is the stated value on every sheet in the corpus.
    # Giants do not write an "average": they write 'Temperature at 1 bar', which is
    # the level this file's datum IS (atmo.h's datum = the body's sea level = the
    # 1-bar level, #689), so it is the same statement under a different label and
    # fact_air() already reads that row for the air's base_T.
    out['avg_T'] = None
    for label in ('Average temperature:', 'Temperature at 1 bar:'):
        line = sheet_line(h, label)
        m = re.search(r'(\d+(?:\.\d+)?)\s*K\b', line) if line else None
        if m:
            # A digit BEFORE the matched figure means the line states something else
            # first -- a range, a count, an unlabeled number -- and reading the first
            # `N K` after it would be a guess. Loud, because a silently wrong mean
            # temperature is worse than a missing one (#685's lesson).
            if re.search(r'\d', line[:m.start()]):
                print('WARNING fact surface: %s puts a number before "%s %s" -- '
                      'check the scrape before trusting the mean read off it'
                      % (page, label, line[:60]))
            out['avg_T'] = float(m.group(1))
            break
    return out


# Our flux at a body is that body's own published irradiance row carried from Earth
# by its own published period, so the only error available is the scrape's rounding
# and the circularised rail: 0.2 % on every planet today. 5 % leaves room for an
# eccentric orbit's mean while still catching a mistyped stellar temperature or a
# moved rail.
FACT_SURFACE_BANDS = 0.05

# A waiver is written against the VALUE it was granted on, not the body forever --
# the air check's rule (#695), and the reason FACT_SURFACE_KNOWN being small means
# something. Keyed by (body, field); the pinned number is what the authored file
# looked like when the argument was accepted.
FACT_SURFACE_KNOWN = {
    ('Pluto', 'bb'): (0.72,
        "its Bulk-parameters rows disagree with each other: Bond albedo 0.72 and "
        "black-body 37.5 K cannot both come from its own 0.873 W/m^2 row, which "
        "gives 32.2 K at that albedo and 36.9 K with no albedo at all. The sheet "
        "says both and does not say which it means, so we author the albedo -- the "
        "row with physics in it, and the one every other body's bb row answers to "
        "to within half a percent -- and print the 14% difference rather than "
        "quietly dropping one of the two rows (#687)"),
}


# The pair cross-check's waivers (ground_model.pair_anchor_check), in the same shape as the
# surface guard's above: granted against the VALUE they were written for, so a moved number
# revokes one rather than leaving it open-ended. One entry, and it is the same contradiction
# FACT_SURFACE_KNOWN already holds: Pluto's sheet gives three thermal numbers and no two of
# them agree, and the range among them is the only one that speaks about the SURFACE.
PAIR_ANCHOR_KNOWN = {
    ('Pluto', 29.62): (
        "the range 24-38 K solves an anchor 8.1% below the black-body this system derives from "
        "the same page's own 0.873 W/m^2 and 0.72 albedo (32.23 K) -- and that page's own "
        "black-body row says 37.5 K, which is 16% above the albedo's answer. Three rows, no two "
        "consistent; the range is the only one about the ground, so the ground follows the "
        "range and the disagreement stays printed rather than being resolved by choosing "
        "(#687's Pluto waiver, arguing the same contradiction one key over)"),
}


def star_rail(b, by):
    """(star body or None, semimajor axis m or 0.0): the rail the STAR's light at this body
    is measured on. For a moon that is its PARENT's rail -- moonfact states 1361.0 W/m2
    because the Moon goes round the Sun with the Earth, not because it holds a solar orbit
    of its own, and weighing it on its own 384,400 m rail would print a flux six orders too
    high (#737). The dossier labels the row "sunlight (parent's orbit)" for the same
    reason, in src/gameui.cpp."""
    inertial = b.get('inertial') or {}
    parent = by.get(b.get('orbits'))
    if parent is None:
        return None, 0.0
    w = inertial.get('orb_ang_speed')
    if parent.get('type') == 'star':
        if w and parent.get('mass'):
            return parent, semimajor_axis(G * parent['mass'], w)
        return None, 0.0
    gp = by.get(parent.get('orbits'))
    pw = (parent.get('inertial') or {}).get('orb_ang_speed')
    if gp is not None and gp.get('type') == 'star' and gp.get('mass') and pw:
        return gp, semimajor_axis(G * gp['mass'], pw)
    return None, 0.0


def flux_at_rail(star, a, T_sun):
    """[W/m^2] The star's flux on the rail star_rail() found, from the ONE authored stellar
    temperature (#687: L = 4 pi R^2 sigma T^4, spread over 4 pi a^2)."""
    return SIGMA_SB * T_sun ** 4 * (star['radius'] / a) ** 2


def check_fact_surface(bodies, sheets, sun, verbose=True):
    """#687: weigh the ONE authored stellar temperature against the rows the corpus
    DOES state. For each body with a Bulk-parameters table: the flux this system
    implies at that body's rail against its published 'Solar irradiance', and the
    black-body temperature that flux and our authored albedo imply against its
    published 'Black-body temperature'. Prints a row per body either way (#685: a
    guard that prints nothing is indistinguishable from one that checked nothing)."""
    bad = []
    T_sun = (sun.get('surface') or {}).get('temperature')
    if not T_sun:
        return ['the star authors no surface.temperature, so no body in this '
                'system receives any insolation']
    by_name = {b['name']: b for b in bodies}
    rows = []
    waived = []
    for b in bodies:
        sheet = sheets.get(b['name'])
        if not sheet:
            continue
        # The star's rail, and for a moon its PARENT's rail, is star_rail()'s rule; the
        # irradiance row means "the STAR's light here", so the flux has to be weighed on
        # that rail and not on the moon's own (#737).
        star, a = star_rail(b, by_name)
        if star is None:
            continue
        flux = flux_at_rail(star, a, T_sun)
        cells = []
        if sheet.get('irradiance'):
            off = (flux - sheet['irradiance']) / sheet['irradiance']
            cells.append('flux %10.1f W/m2 (sheet %10.1f) %+.2f%%'
                         % (flux, sheet['irradiance'], 100.0 * off))
            if abs(off) > FACT_SURFACE_BANDS:
                bad.append('%s: this system implies %.4g W/m^2 at its rail, the '
                           'fact sheet states %.4g (%+.1f%%, band %d%%)'
                           % (b['name'], flux, sheet['irradiance'], 100.0 * off,
                              round(100 * FACT_SURFACE_BANDS)))
        else:
            cells.append('flux %10.1f W/m2 (sheet -)' % flux)
        albedo = (b.get('surface') or {}).get('albedo')
        if albedo is not None and sheet.get('bb_T'):
            bb = blackbody_temperature(flux, albedo)
            off = (bb - sheet['bb_T']) / sheet['bb_T']
            cells.append('bb %6.1f K (sheet %6.1f) %+.2f%%'
                         % (bb, sheet['bb_T'], 100.0 * off))
            if abs(off) > FACT_SURFACE_BANDS:
                known = FACT_SURFACE_KNOWN.get((b['name'], 'bb'))
                # Same expiry rule as the air waivers: a waiver covers the number it
                # was argued against, so moving the albedo voids the argument. The row
                # still PRINTS, tagged (waived): a guard that goes quiet on the one body
                # it let through is the #685 failure mode it exists to prevent.
                if known is not None and abs(albedo - known[0]) <= WAIVER_TOL * known[0]:
                    cells[-1] += ' (waived)'
                    waived.append('%s bb: %s' % (b['name'], known[1]))
                else:
                    stale = ('' if known is None else
                             ' -- the waiver here pins an albedo of %g, so it predates %g'
                             % (known[0], albedo))
                    bad.append('%s: authored albedo %g with that flux imply a %.4g K '
                               'black-body, the fact sheet states %.4g (%+.1f%%, band %d%%)%s'
                               % (b['name'], albedo, bb, sheet['bb_T'], 100.0 * off,
                                  round(100 * FACT_SURFACE_BANDS), stale))
        else:
            cells.append('bb - (%s)' % ('no authored albedo' if albedo is None
                                        else 'sheet states no black-body row'))
        rows.append('%-9s a %12.4g m  %s' % (b['name'], a, '  '.join(cells)))
    if verbose:
        for r in rows:
            print('  surface   %s' % r)
        for why in waived:
            print('  waived    %s' % why)
    return bad

# ---------------------------------------------------------------------------
# moons
# ---------------------------------------------------------------------------
def parse_sat_table_all(page):
    """Every satellite on a giant-planet fact sheet (bulk + orbital tables joined)."""
    h = read(page)
    bulk = orb = None
    for t in re.findall(r'<table.*?</table>', h, re.S):
        if 'Bulk parameters' in t:
            bulk = t
        elif 'Orbital parameters' in t:
            orb = t
    if not bulk or not orb:
        raise ValueError('bulk/orbital table not found in ' + page)
    b = table_rows(bulk)
    o = table_rows(orb)

    def g(r, i):
        return r[i] if len(r) > i else None

    orb_map = {}
    for r in o:
        if not r or not r[0].strip():
            continue
        nm = clean_name(r[0])
        a = num(g(r, 1)); per = num(g(r, 3)); inc = num(g(r, 5)); ec = num(g(r, 6))
        if a is not None or per is not None:
            orb_map[nm] = dict(a=a, period_d=per, i=inc, e=ec)

    out = []
    for r in b:
        if not r or not r[0].strip():
            continue
        radius = num(g(r, 2))
        if radius is None:
            continue
        mass = num(g(r, 1))
        clean = clean_name(r[0])
        om = orb_map.get(clean, {})
        out.append(dict(
            name=clean,
            mass_kg=(mass * 1e20 if mass is not None else None),
            radius_m=radius * 1e3,
            a=(om['a'] * 1e6 if om.get('a') is not None else None),
            period_s=(om['period_d'] * DAY if om.get('period_d') else None),
            i=((om.get('i') or 0.0) * D2R),
            e=(om.get('e') or 0.0),
            measured=(mass is not None),
            named=is_named(clean)))
    return out

def parse_earth_moon():
    h = read('moonfact.html')
    bulk = section_values(h, 'Bulk parameters')
    orb  = section_values(h, 'Orbital parameters')
    return dict(
        name='Moon',
        mass_kg=get(bulk, 'mass1024') * 1e24,
        radius_m=get(bulk, 'volumetricmeanradius') * 1e3,
        a=get(orb, 'semimajoraxis') * 1e9,
        period_s=get(orb, 'revolutionperiod') * DAY,
        e=get(orb, 'orbiteccentricity'),
        i=(get(orb, 'inclinationtoecliptic') or 0.0) * D2R,
        measured=True, named=True)

def parse_mars_moons(names):
    """Two moons side by side in one table: label | <first> | <second>."""
    h = read('marsfact.html')
    i = h.find('Satellites of Mars')
    rows = table_rows(re.search(r'<table.*?</table>', h[i:], re.S).group(0))
    out = {}
    for ci, name in enumerate(names):
        def val(label_substr):
            for r in rows:
                if r and label_substr in norm(r[0]):
                    return num(r[1 + ci]) if len(r) > 1 + ci else None
            return None
        mass_kg = val('mass1015')
        if mass_kg is not None:
            mass_kg *= 1e15
        ax = [val('subplanetaryaxisradius'), val('alongorbitaxisradius'),
              val('polaraxisradius')]
        ax = [x for x in ax if x is not None]
        out[name] = dict(
            name=name,
            mass_kg=mass_kg,
            radius_m=((sum(ax) / len(ax) * 1e3) if ax else None),
            a=val('semimajoraxis') * 1e3,
            period_s=val('siderealorbitperiod') * DAY,
            i=(val('orbitalinclination') or 0.0) * D2R,
            e=(val('orbitaleccentricity') or 0.0),
            measured=(mass_kg is not None), named=True)
    return out

def parse_pluto_moons_all():
    """Charon from its own block (has mass); the rest from 'Other Moons'."""
    h = read('plutofact.html')
    out = []
    ch = section_values(h, 'Charon (P1)')
    out.append(dict(
        name='Charon',
        mass_kg=get(ch, 'mass1021') * 1e21,
        radius_m=get(ch, 'equatorialradius') * 1e3,
        a=get(ch, 'meandistance') * 1e3,
        period_s=get(ch, 'siderealorbitperiod') * DAY,
        i=(get(ch, 'orbitalinclination') or 0.0) * D2R,
        e=get(ch, 'orbitaleccentricity') or 0.0,
        measured=True, named=True))
    i = h.find('Other Moons of Pluto')
    rows = table_rows(re.search(r'<table.*?</table>', h[i:], re.S).group(0))
    for r in rows:
        if not r or not r[0].strip():
            continue
        a = num(r[1]); per = num(r[2])
        if a is None or per is None:
            continue
        clean = clean_name(r[0])
        out.append(dict(
            name=clean,
            mass_kg=None,
            radius_m=radius_from_cell(r[4] if len(r) > 4 else None),
            a=a * 1e3, period_s=per * DAY, i=0.0, e=0.0,
            measured=False, named=is_named(clean)))
    return out

def make_body(name, type_, parent, data, *, surface=None, seed=0.0,
              has_sea=False, rings=None, parent_body=None, home_body=None):
    radius_m = data['radius_m']
    mass_kg  = data['mass_kg']
    # #708: the file states gravity ONCE, as mass and radius, and the loader derives g
    # from them. This g is only ever used to convert a sheet's rounded air density into
    # the datum pressure the loader inverts, so it has to be the number the loader will
    # divide by -- not the sheet's separate gravity row, which is checked for
    # agreement against this one instead (check_scraped_gravity).
    g = G * mass_kg / (radius_m * radius_m) if mass_kg and radius_m else None

    body = {'name': name, 'type': type_}
    if parent:
        body['orbits'] = parent
    body['radius'] = radius_m
    body['mass'] = mass_kg
    body['seed'] = seed
    body['has_sea'] = has_sea
    if surface is not None:
        body['surface'] = resolve_layer_heights(surface, radius_m)
        # #739: a body's JSON states its air's datum PRESSURE; the densities the
        # catalogue below is built from are sheet rows and are converted here.
        anchor_sea_level_pressure(body['surface'], radius_m, g)

    # SOIs are derived by the loader (bodylimits.h); only the root authors
    # one (its universe bound).
    inertial = {}
    if parent:
        inertial['orb_ang_speed'] = data['w']
        if data.get('e') is not None:
            inertial['ecc'] = data['e']
        inertial['arg_peri'] = data.get('omega', 0.0)
        inertial['orb_incl'] = data.get('i', 0.0)
        inertial['lon_asc_node'] = data.get('raan', 0.0)
        inertial['true_anomaly0'] = data.get('nu0', 0.0)
        if data.get('incl_ref') == 'equator':
            # #147: orb_incl/lon_asc_node are referred to the PARENT'S
            # EQUATOR, not its orbital plane (loader composes the moon's
            # orient under the parent's tilt). Default "orbit" omitted:
            # hand-authored systems load as before.
            inertial['incl_ref'] = 'equator'
    else:
        inertial['soi'] = data['soi']
        inertial['orb_ang_speed'] = 0.0
    body['inertial'] = inertial

    if data.get('rot_s'):
        body['rotating'] = {
            'rot_ang_speed': TWO_PI / data['rot_s'],
            'axial_tilt': data.get('obliquity', 0.0),
        }
        if name in WGCCRE:
            # #143: real epoch orientation from the vendored IAU report.
            taz, sp0 = wgccre_orientation(name, data.get('obliquity', 0.0),
                                          data.get('i', 0.0),
                                          data.get('raan', 0.0))
            body['rotating']['spin_phase0'] = sp0
            body['rotating']['tilt_azimuth'] = taz
        elif data.get('sync_phase0') is not None:
            # #144: synchronous moons -- the phase is derived from the
            # orbital epoch state, not drawn. The tilt stays node-locked
            # (Cassini-type state), so omitting tilt_azimuth is correct.
            body['rotating']['spin_phase0'] = data['sync_phase0']
        else:
            # #141: epoch spin phase and obliquity-node longitude. The fact
            # sheets do not publish prime meridian at epoch, so draw a
            # reproducible pseudo-random angle per body (seeded by name)
            # instead of leaving both node-locked at 0.
            body['rotating']['spin_phase0'] = phase0(name, 'spin')
            if data.get('obliquity'):
                # tilt_azimuth only where a tilt exists: with zero tilt it
                # degenerates into a pure phase shift (loader comment,
                # #141).
                body['rotating']['tilt_azimuth'] = phase0(name, 'tilt')
    # No rot_s -> no rotating block: the loader's dummy frame (zero spin,
    # derived near-body SOI) covers it.
    if rings is not None:
        body.setdefault('surface', {})['rings'] = rings
    if parent_body is not None and home_body is not None:
        stamp_transfer_dv(body, parent_body, home_body)
    return body

def rock(r, c1, c2):
    # relief capped at 25% of radius so tiny moons are not 167% relief
    amp = min(int(r * 0.25), max(500, int(r * 0.02)))
    return {'amplitude': max(50, amp),
            'palette': [[0.0, c1], [1.0, c2]]}

DEFAULT_COLOR = ([0.55, 0.55, 0.55], [0.72, 0.72, 0.72])
MOON_COLORS = {
    'Moon':      ([0.50, 0.50, 0.50], [0.70, 0.70, 0.70]),
    'Phobos':    ([0.30, 0.25, 0.20], [0.45, 0.40, 0.35]),
    'Deimos':    ([0.35, 0.30, 0.25], [0.50, 0.45, 0.40]),
    'Io':        ([0.70, 0.60, 0.20], [0.85, 0.75, 0.30]),
    'Europa':    ([0.70, 0.70, 0.75], [0.85, 0.85, 0.85]),
    'Ganymede':  ([0.45, 0.42, 0.40], [0.65, 0.62, 0.58]),
    'Callisto':  ([0.35, 0.33, 0.32], [0.55, 0.52, 0.50]),
    'Amalthea':  ([0.60, 0.35, 0.25], [0.80, 0.55, 0.45]),
    'Himalia':   ([0.45, 0.45, 0.48], [0.62, 0.62, 0.65]),
    'Mimas':     ([0.60, 0.60, 0.62], [0.80, 0.80, 0.80]),
    'Enceladus': ([0.80, 0.80, 0.85], [0.95, 0.95, 0.95]),
    'Tethys':    ([0.60, 0.60, 0.62], [0.80, 0.80, 0.82]),
    'Dione':     ([0.55, 0.55, 0.58], [0.75, 0.75, 0.78]),
    'Rhea':      ([0.55, 0.55, 0.58], [0.78, 0.78, 0.80]),
    'Iapetus':   ([0.25, 0.20, 0.15], [0.55, 0.50, 0.45]),
    'Titan':     ([0.70, 0.50, 0.20], [0.85, 0.65, 0.35]),
    'Phoebe':    ([0.30, 0.28, 0.25], [0.45, 0.43, 0.40]),
    'Janus':     ([0.55, 0.55, 0.57], [0.72, 0.72, 0.74]),
    'Epimetheus':([0.55, 0.55, 0.57], [0.72, 0.72, 0.74]),
    'Hyperion':  ([0.55, 0.52, 0.48], [0.75, 0.72, 0.68]),
    'Miranda':   ([0.50, 0.55, 0.60], [0.70, 0.75, 0.80]),
    'Ariel':     ([0.50, 0.55, 0.60], [0.70, 0.75, 0.80]),
    'Umbriel':   ([0.45, 0.50, 0.55], [0.65, 0.70, 0.75]),
    'Titania':   ([0.50, 0.55, 0.60], [0.70, 0.75, 0.80]),
    'Oberon':    ([0.48, 0.52, 0.57], [0.68, 0.72, 0.77]),
    'Triton':    ([0.60, 0.65, 0.70], [0.80, 0.85, 0.88]),
    'Nereid':    ([0.55, 0.55, 0.58], [0.75, 0.75, 0.78]),
    'Proteus':   ([0.50, 0.50, 0.53], [0.70, 0.70, 0.73]),
    'Charon':    ([0.50, 0.50, 0.50], [0.70, 0.70, 0.72]),
}

# WHAT THE AIR IS MADE OF (#552 A4b, #630), by MOLE. A body authors its mix and the
# loader resolves molar mass, gamma and the oxidizer share a jet gate reads from it
# (src/atmo.h:resolveMix), so 'molar_mass'/'gamma' are no longer authorable keys --
# the mix is the fact and those two are its consequences. Fractions need not sum to 1;
# they are normalized by their own total. The species names must exist in the game's
# gas table (atmo.h:gasSpecies): N2, O2, CO2, H2, He, CH4, H2O, Ar.
# A named mix that the OTHER system generator could also invent is imported from
# utils/gas_mixes.py, so there is one of each (#636); a mix only this catalogue
# measures stays here. Suffixes are load-bearing: the solar system's Uranus and
# gen_systems' Jool are both H2/He/CH4 skies with different fractions, so they are
# two names, not one name twice.
from gas_mixes import DRY_AIR, CO2_HEAVY, H2_HE, H2_HE_CH4_SOLAR
N2_METHANE = [{'gas': 'N2', 'fraction': 0.984},
              {'gas': 'CH4', 'fraction': 0.014},
              {'gas': 'H2', 'fraction': 0.002}]

# Which page states a moon's air. MOON_ATMOS is outside the `planets` list every fact
# guard walks, so this is how check_fact_wind reaches a mooned wind table at all (#772).
# Listed rather than scraped: a mooned wind row with no entry here is a refusal below,
# not a silent pass.
MOON_AIR_PAGES = {'Titan': 'saturniansatfact.html'}

MOON_ATMOS = {
    'Titan':  {'color': [0.85, 0.60, 0.25], 'thickness': 40000,
               'power': 3.0, 'intensity': 0.8,
               'sea_level_density': 5.3, 'scale_height': 20000,
               # Titan's air is the densest of this catalogue's moons -- and it still cannot
               # burn: nitrogen, methane, a trace of hydrogen, no free oxygen at all. Venus
               # carries twelve times more air at its own datum; this is not the heavy sky.
               'composition': N2_METHANE,
               # Wind (#772). No page in this corpus states a wind for Titan -- it has no
               # fact page of its own and its data comes from the giant-planet satellite
               # sheet, which has no wind line -- so all three figures are climatology and
               # FACT_WIND_KNOWN carries the waiver that says so. The shape is the one
               # Titan's published climatology agrees on: a near-calm boundary layer under
               # stratospheric flow an order of magnitude faster.
               # Weighing it matters more here than on the giants because of the density:
               # 5.3 kg/m3 at the datum is 4x Earth's sea level, so 100 m/s of deck wind
               # would be 26.5 kPa. A m/s there is 3 Pa; 30 m/s at 20 km is 0.88 kPa in
               # 1.96 kg/m3; the 100 m/s at 60 km sits in 0.28 kg/m3 for 1.4 kPa, Mach 0.51
               # on the 195 m/s sound speed this column implies (its derived 90.5 K datum
               # temperature, the one SOLAR_AIR_EXPONENTIAL above exists to allow).
               # Those aloft figures come out of drag.h's GEOPOTENTIAL depth, R h/(R+h) --
               # 58.6 km of it at 60 km on this body -- rather than alt/H, which would say
               # 0.264 kg/m3 and 1.3 kPa and be 7% away from what the game computes.
               # NO wind_gen, and this is the case the rotation scaling was never for:
               # Titan is tidally locked, so at this column's own U = 100 m/s its Rossby
               # number is 4.26 and its Rhines scale is 127 deg of latitude -- wider than
               # the hemisphere, and past the Ro ~2.14 at which atmo.h says the ask is
               # simply zero belts, which is where windGenerate returns the anchors
               # unchanged (bands 1). Author the column plainly instead.
               'wind': [{'alt': 0, 'east': 1.0, 'north': 0.0},
                        {'alt': 20000, 'east': 30.0, 'north': 0.0},
                        {'alt': 60000, 'east': 100.0, 'north': 0.0}]},
    'Triton': {'color': [0.60, 0.70, 0.85], 'thickness': 5000,
               'power': 5.0, 'intensity': 0.3},
    'Io':     {'color': [0.85, 0.75, 0.40], 'thickness': 4000,
               'power': 5.0, 'intensity': 0.25},
}

def ring(name, r_in_km, r_out_km, thickness_km, albedo, opacity=0.8):
    return {'name': name, 'inner': r_in_km * 1e3, 'outer': r_out_km * 1e3,
            'thickness': thickness_km * 1e3, 'albedo': albedo,
            'opacity': opacity}

# Albedos boosted from the (very dark) real values so the bands read as rings in-game.
JUP_RINGS = [
    ring('Halo', 89400, 123000, 10000, 0.15),
    ring('Main', 123000, 128940, 100, 0.30),
    ring('Amalthea', 128940, 181350, 2600, 0.12),
    ring('Thebe', 181350, 280000, 8800, 0.12),
]

# Cassini Division (~117,500-122,000 km) is the gap between B and A (no band)
SAT_RINGS = [
    ring('C-ring', 74500, 92000, 1000, 0.35, 0.60),
    ring('B-ring', 92000, 117500, 3000, 0.75, 0.90),
    ring('A-ring', 122000, 137000, 200, 0.55, 0.70),
]

def build_base(verbose_air=False):
    """Sun + 9 planets, identical across every scope file."""
    bodies = []
    parsed = {}

    sun = make_body('Sun', 'star', None,
        # 1e15 m, up from 1e14: this is the universe bound, not a physical sphere, and the
        # five far-apoapsis rows the asteroid pass brings (2005 VX3 out to 3.7e14 m) sit
        # outside the old one. Kerbol already authors 1e18 in gen_systems.py, so the real
        # system was the tight one.
        dict(mass_kg=SUN_MASS, radius_m=SUN_RADIUS, soi=1.0e15, rot_s=None),
        # No atmosphere rim on the star: the limb shader's day/night term falls
        # back to a fixed world direction and paints a half-eclipse crescent.
        surface={'palette': [[0.0, [1.0, 0.8, 0.35]], [1.0, [1.0, 1.0, 0.75]]]})
    bodies.append(sun)
    parsed['Sun'] = dict(mass_kg=SUN_MASS)

    planets = [
        ('mercuryfact.html', 'Mercury', 1.0,
         {'amplitude': 3000, 'palette': [[0.0, [0.35, 0.33, 0.32]],
                                         [0.5, [0.55, 0.52, 0.50]],
                                         [1.0, [0.75, 0.72, 0.68]]]}, False),
        ('venusfact.html', 'Venus', 2.0,
         {'amplitude': 2000, 'palette': [[0.0, [0.80, 0.65, 0.40]],
                                         [1.0, [0.90, 0.80, 0.55]]],
          # the whole planet sits under a thick sulfuric-acid haze
          # The sheet's density row, converted to the pressure the file states
          # (#739): 65 kg/m3 at g 8.87 on H 16 km is 9.22 MPa, which is
          # Venus's real 92 BAR (91 atm), the value utils/rss/html/venusfact.html
          # states beside the mass and radius this tuple scrapes. 92 -- that
          # pressure in BAR -- sat in this field for a while and made the dossier
          # read 129 atm, because the pair was two statements of one column
          # (#684). H 16 km and the CO2 mix already give the right 742 K.
          'atmosphere': {'color': [0.90, 0.75, 0.45], 'thickness': 30000,
                         'power': 3.0, 'intensity': 0.8,
                         # #625: SOLAR_AIR_SHAPE emits this air's table -- a 737 K datum
                         # (the sheet's own "Average temperature", the same row its
                         # SURFACE temperature comes from, so the load-time audit now
                         # prints "one published row states both" where it used to print
                         # "two rows, -4.64 K apart"), -7.8 K/km to a 230 K lid at 65 km,
                         # ceiling 116 km. The datum pressure is the sheet's "92 bars" row
                         # itself rather than rho0 x g x H, and what it resolves -- 65.2
                         # kg/m3 -- is the sheet's "~65 kg/m3" to 0.3%. The lid is the
                         # published Venus tropopause, cited the way the 100 m/s wind row
                         # at the same 65 km already is; with it, the deck sits in 0.2 kg/m3
                         # of air instead of 1.1, so the superrotation row finally pushes on
                         # a hull at about the pressure Venus's cloud tops actually feel.
                         # The band gate is no longer waived here: 116 km is 275 Pa of
                         # orbital q, inside 10..1000, where the isothermal 16 km column
                         # left 99.8 kPa at 160 km and an excuse instead of a ceiling.
                         # Near-pure CO2 with nitrogen on top. Venus has the densest
                         # air a jet could swallow in this catalogue and no oxidizer
                         # in any of it (#630).
                         'composition': [{'gas': 'CO2', 'fraction': 0.965},
                                         {'gas': 'N2', 'fraction': 0.035}],
                         # The first body in this catalogue to author wind (#688).
                         # Two rows, each with a different source, and the branch
                         # between them is ours:
                         #  * 1 m/s at the datum is venusfact.html's "Wind speeds:
                         #    0.3 to 1.0 m/s (surface)" -- the top of its range,
                         #    because the WHOLE range is 3..33 Pa of dynamic pressure
                         #    in 65 kg/m3 of air, against the 3.3 kPa a hull at 10 m/s
                         #    bills in the same air. Nothing the player can feel.
                         #  * 100 m/s at 65 km is the cloud-top superrotation: the
                         #    deck laps the planet in ~4 days, ~60x faster than the
                         #    ground. NOT in utils/rss/ -- our Venus sheet stops at
                         #    the surface row, so this figure is cited from published
                         #    Venus climatology and is the number that makes Venus
                         #    worth flying in: our column puts rho ~1.1 kg/m3 at 65
                         #    km, so 100 m/s of air is ~5.6 kPa of dynamic pressure,
                         #    6x what a hull at 40 m/s there is fighting. Reality's
                         #    column is ~10x thinner at 65 km, so this model bills the
                         #    deck harder than Venus does -- that is the single
                         #    exponential (#686), not the wind's doing.
                         # "Cloud top" is reality's deck, NOT this file's: the clouds
                         # shell below sits 5000 m above the highest ground, ~6 km up,
                         # and its drift is a pattern-units creep nothing links to
                         # these rows (#714). Do not read this row as "where the
                         # rendered clouds are".
                         # Signs: these rows are GROUND-relative and positive east =
                         # the sense the body's own spin carries the ground, which is
                         # what superrotation is. Every popular account calls it
                         # "westward", meaning sun-relative on a retrograde rotator
                         # (Venus obliquity 177 deg, so its sun rises in the west);
                         # authoring those words verbatim would point the whole deck
                         # backwards. A linear branch from 1 to 100 m/s is a stub for
                         # a real circulation profile -- it puts ~25 m/s at 16 km,
                         # where the real stagnation region below the deck is calmer.
                         # A third row around 30 km is the fix once someone has a
                         # source for it; e2e flight/688-venus-wind-outruns-the-hull
                         # spawns at that height and will move when it lands.
                         'wind': [{'alt': 0, 'east': 1.0, 'north': 0.0},
                                  {'alt': 65000, 'east': 100.0, 'north': 0.0}]},
          # Deck on the DATUM axis like the air profile (#719), at 55 km -- reality's
          # main cloud layer starts there, and the wind's 100 m/s row sits at 65 km, so
          # the deck now renders inside the air this table says is racing along. It used
          # to be 5000 m above the highest ground, which put Venus's sky at ~7 km, in
          # ~10 m/s of air, under a comment about cloud-top superrotation.
          'clouds': {'color': [0.92, 0.82, 0.55], 'height': 55000,
                     'coverage': 0.9, 'freq': 8.0}}, False),
        ('earthfact.html', 'Earth', 3.0,
         {'amplitude': 8000, 'sea_level': 0.0, 'sea_color': [0.0, 0.18, 0.50],
          # Authored macro relief (GEBCO 2008 merge;
          # utils/heightmaps/gen_earth_hm.py + staging/heightmaps/earth/NOTES.md).
          # Fine noise is residual only.
          'heightmap': 'res/heightmaps/earth/earth_hm.i16',
          'detail_amplitude': 250,
          # The ocean shell is a 128-ring UV sphere, so its flat faces sag
          # ~radius/13300 below sea level: ~480 m here, and the sea floor
          # pokes through the water. The flat sea (floor clamped to sea
          # level, terrain painted sea_color) has no waves or reflections
          # but works at any scale. Kerbin (600 km, ~45 m of sag) keeps the
          # shell.
          'ocean': 'flat',
          'palette': [[0.0, [0.55, 0.50, 0.35]],
                      [0.12, [0.12, 0.42, 0.15]],
                      [0.35, [0.30, 0.42, 0.18]],
                      [0.60, [0.45, 0.40, 0.30]],
                      [0.82, [0.60, 0.58, 0.55]],
                      [1.0, [0.95, 0.95, 0.97]]],
          # Earth is the one body that authors a LAYER TABLE instead of a scale height
          # (#552 A4): the 1976 standard atmosphere's altitude/temperature levels, with
          # the lapse rate in K/m (the standard states K/km). Since #739 the datum
          # pressure IS authored -- the standard's own 101,325 Pa row, with the density
          # 1.2250018 that pair resolves to rather than the 1.225 the table rounds it to.
          # Each level's base pressure above the first is still NOT authored: the loader
          # integrates them hydrostatically (atmo.h:resolveAtmoLevels).
          # Troposphere to 11 km, isothermal tropopause to 20, stratosphere to 47,
          # isothermal stratopause to 51, mesosphere to the top. The standard's 85 km row
          # (186.87 K) needs no level of its own: that row is 85 km of GEOPOTENTIAL, which
          # on this body is 86,149 m geometric -- above the top the body states. Its air
          # stops at 85 km GEOMETRIC = 83,881 m of geopotential, and the last level lands
          # there on 188.888 K, which is what the standard says at that same height. height
          # keeps the top at 85 km, exactly what the old scale_height 8500 derived (x10), so
          # no science band or rotating frame moves. The composition is written rather than
          # defaulted because the profile's pressures are only right for the gas they
          # were measured in, and DRY_AIR is that gas: it resolves to 0.0289650
          # kg/mol, which puts the datum at 101,324.9 Pa -- 0.1 Pa off the standard's
          # own 101,325, because that figure IS dry air at 1.225 kg/m3 and 288.15 K.
          # #690: the alts AND lapse rates below are the standard's OWN numbers, and both are
          # stated per GEOPOTENTIAL height, while a system file's altitudes are geometric metres
          # -- so the table says 'heights': 'geopotential' and resolve_layer_heights() converts
          # them at emit time, against this body's radius. The written file therefore speaks one
          # unit: 11 km becomes 11,019.0 m, 51 km becomes 51,411.6 m, and the 20 km level's
          # 0.001 K/m becomes 0.00099185 K/m (0.99185 of it, as a chord across the span that
          # level lapses through). On Earth the altitudes move a few hundred metres; the same
          # rows on a 600 km world would move by kilometres, which is why the conversion is here
          # and not left to whoever pastes a table in. base_T stays the published temperature
          # either way, so the dossier rows and the test pins keep reading standard numbers.
          'atmosphere': {'color': [0.30, 0.50, 1.00], 'thickness': 15000,
                         'power': 4.0, 'intensity': 0.7,
                         # US76's own sea-level pair: 101,325 Pa at 288.15 K. The
                         # 1.225 kg/m3 every table rounds from it is the CONSEQUENCE,
                         # so the file states the pair and the loader resolves the
                         # density (#739). Converting the rounded number back would
                         # need the gas table on this side of the C++ boundary, which
                         # utils/gas_mixes.py keeps there on purpose.
                         'sea_level_pressure': 101325.0, 'height': 85000,
                         'composition': DRY_AIR,
                         'heights': 'geopotential',
                         'layers': [
                             {'alt': 0,     'T': 288.15, 'lapse': -0.0065},
                             {'alt': 11000, 'T': 216.65, 'lapse': 0.0},
                             {'alt': 20000, 'T': 216.65, 'lapse': 0.001},
                             {'alt': 32000, 'T': 228.65, 'lapse': 0.0028},
                             {'alt': 47000, 'T': 270.65, 'lapse': 0.0},
                             {'alt': 51000, 'T': 270.65, 'lapse': -0.0028},
                             {'alt': 71000, 'T': 214.65, 'lapse': -0.002}],
                         # Wind (#688). earthfact.html states "Wind speeds: 0 to 100
                         # m/s" -- a range with NO level attached to it, which is why
                         # the profile below is ours and every row says what it is
                         # standing on:
                         #  * 5 m/s at the datum is the everyday surface flow over
                         #    ocean, where friction is what holds a wind down. A fifth
                         #    of the sheet's top, and the sheet's top is not a surface
                         #    figure.
                         #  * 40 m/s at 11 km is a mid-latitude jet core, NOT the sheet's
                         #    100 m/s. That 100 is the extreme end of a weather range --
                         #    an upper-level current in a storm -- and authoring it as a
                         #    permanent global flow would put 1.8 kPa of dynamic pressure
                         #    on every hull at this body's own 10.5 km cloud deck
                         #    (0.36 kg/m3 there), six times what a hull at 40 m/s through
                         #    the same air is already fighting. The jet is the number
                         #    worth having; check_fact_wind keeps either row from ever
                         #    exceeding the page.
                         # Signs: +east, the sense a mid-latitude jet runs. Earth's table
                         # carries no latitude bands -- #767 gave WindLevel that axis and
                         # nobody has used it on this body -- so this row is eastward over
                         # the equator too, where the trade winds blow the other way.
                         # That is a stub, not a claim. Jupiter and Saturn are what
                         # authoring the split looks like when the source states one, and
                         # Earth's own split is the row-set still missing here.
                         # Above 11 km the table holds 40 m/s to the 85 km top, which is
                         # windAt()'s documented rule (the air thins out, it does not calm
                         # down) and is under 0.05 Pa of dynamic pressure by the
                         # stratopause.
                         'wind': [{'alt': 0, 'east': 5.0, 'north': 0.0},
                                  {'alt': 11000, 'east': 40.0, 'north': 0.0}]},
          # 10.5 km ASL: reality's tropopause deck, and the same height the old
          # peaks-relative number produced off 8 km of relief (#719).
          'clouds': {'color': [1.0, 1.0, 1.0], 'height': 10500,
                     'coverage': 0.55, 'freq': 10.0}}, True),
        ('marsfact.html', 'Mars', 4.0,
         {'amplitude': 5000, 'palette': [[0.0, [0.50, 0.25, 0.15]],
                                         [0.5, [0.65, 0.35, 0.22]],
                                         [1.0, [0.80, 0.55, 0.40]]],
          'atmosphere': {'color': [0.85, 0.55, 0.35], 'thickness': 12000,
                         'power': 4.0, 'intensity': 0.5,
                         # #625: SOLAR_AIR_SHAPE emits this air's table from the sheet:
                         # a 214 K datum (its "Average temperature"), -4.0 K/km to a lid
                         # at 16.5 km where the lid's 148 K is CO2's frost point at this
                         # body's own 636 Pa, ceiling 60 km (86 Pa of q at a circular
                         # orbit -- Kerbin's edge is 97.8). The datum pressure is the
                         # sheet's "Surface pressure: 6.36 mb" row itself, and the
                         # density -- 0.0155 kg/m3, inside the sheet's 0.016 +/- 0.006 --
                         # is what the loader resolves from it.
                         # #698, and the first row the mass check decided: this file used
                         # to author 0.020 kg/m3 on an 11.5 km scale height -- datum 858 Pa
                         # and an air of 3.32e16 kg. The sheet's own rows (0.016 +/- 0.006,
                         # 11.0 km, 6.1-6.3 mb, ~2.5e16 kg) agree with each other and with
                         # what this now authors: datum 636 Pa, an air of 2.46e16 kg (-1.5%),
                         # T0 214 K. The old pair was 25% heavy on density, 35% heavy on
                         # pressure and 33% heavy on mass -- one authored number, three
                         # published statements, all three saying the same thing.
                         # Thin CO2 with a 0.13% oxygen trace -- enough to measure,
                         # not enough to run a turbine on.
                         'composition': CO2_HEAVY,
                         # Wind (#688). marsfact.html is the only sheet in this corpus
                         # that states a wind AT A LEVEL: "2-7 m/s (summer), 5-10 m/s
                         # (fall), 17-30 m/s (dust storm) (Viking Lander sites)". Those
                         # are lander-site readings, so this row is the datum row, and
                         # 10 m/s is the top of the two non-storm ranges -- the sheet's
                         # 30 m/s is a dust storm, which is weather rather than the
                         # steady profile a wind table states, and is available to author
                         # later if the game ever grows seasons.
                         # What it does NOT do is push: 0.0155 kg/m3 at 10 m/s is 0.8 Pa
                         # of dynamic pressure, a twentieth of what a hull at 20 m/s
                         # through this air already pays. The row is still worth authoring
                         # because a wind is a velocity, not a force (#688) -- a hull
                         # trimmed at 20 m/s of AIRSPEED here tracks 30 m/s over the
                         # ground, and that is what the surface map, the airspeed tape and
                         # a landing approach all read.
                         # One row, so windAt() holds 10 m/s all the way to the 60 km
                         # ceiling. Reality's Mars has faster air aloft (and westward
                         # trades near the ground); the sheet says nothing above the
                         # landers, so neither do we.
                         'wind': [{'alt': 0, 'east': 10.0, 'north': 0.0}]}},
         False),
        # gas giants: band ramp dark (pole/edge) -> light (equator/centre)
        # Their air is authored by what it is made of: H2/He for the giants, and the
        # ice giants the same mix with a methane crown. Note what that corrects: these
        # bodies used to type molar_mass 0.023 for the ice giants, which is not what
        # H2/He/CH4 means -- 2.3% methane by MOLE barely moves a 0.0026 kg/mol
        # mixture, and 0.023 implied an air 9x heavier than an ice giant has. The
        # derived temperature is linear in that number, so Uranus read 498 K in the
        # atlas (#625's complaint) and now reads 57 K. See #625 for the scale heights.
        ('jupiterfact.html', 'Jupiter', 5.0,
         {'bands': True, 'band_count': 11,
          'palette': [[0.0, [0.50, 0.40, 0.32]], [1.0, [0.85, 0.78, 0.65]]],
          'atmosphere': {'color': [0.85, 0.78, 0.65], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         # #625: SOLAR_AIR_SHAPE emits this table from three rows of
                         # the sheet -- 165 K at 1 bar, 112 K at 0.1 bar, H 27 km -- so
                         # the lapse (-1.03 K/km) and the seam (51,542 m) are fitted
                         # through the sheet's own rows -- see author_air_profile for what
                         # that fit costs this body (7.4 kPa where the sheet reads 10). The datum is
                         # the 1 bar level, which is what its density, gravity and
                         # temperature rows are all stated at; that resolves 0.1617 kg/m3,
                         # the sheet's 0.16 to 1%. What was here -- 0.16 with H 27 km --
                         # read a 186.73 K datum, 13% off its own sheet, under a 1.12 bar
                         # barometer. Ceiling 245 km is the band gate's answer (108.6 Pa
                         # of q at a circular orbit, beside Kerbin's 97.8): a lapsing
                         # column thins faster aloft than the isothermal one #682 tuned,
                         # so the ceiling comes in from 390 km and shellEdge 468 -> 294 km.
                         # A giant's ceiling is where the game draws its one hard zero,
                         # not where its envelope ends (#701).
                         # Jupiter keeps its sheet's 0.16 even though the trio lock
                         # wants 0.143 at the 1-bar level (H 27 km, the sheet's own
                         # "Gravity (mean, 1 bar)" 25.92, which is our g to four
                         # digits). That leaves the datum barometer at 1.12 bar, a 12%
                         # miss where Saturn's sheet disagreed with itself by 28% and
                         # we chose the column there: inside the guard's 30% band, so
                         # source fidelity wins and --check prints the difference.
                         'composition': H2_HE,
                         # Wind (#772). jupiterfact.html is one of the two sheets in this
                         # corpus that states wind PER LATITUDE BAND: "Up to 150 m/s (<30
                         # degrees latitude) Up to 40 m/s (>30 degrees latitude)". That
                         # pair is exactly what #767's lat bands were cut for -- both halves
                         # are half-open upward, so [-90,-30), [-30,30), [30,90] tile the
                         # globe with no gap and no overlap, and the loader's coverage error
                         # spells out that very tiling (system.cpp).
                         # These rows carry the page's numbers at THIS body's datum, the
                         # 1-bar level: 150 m/s over 0.162 kg/m3 is 1.8 kPa of dynamic
                         # pressure, and Mach 0.16 on the 936 m/s sound speed this mix and
                         # 165 K give (sqrt(gamma Rs T), drag.h). Both are inside the band
                         # the aero law is pinned in -- a hull at Kerbin cruise already
                         # pays 1.6 kPa -- so a Jupiter wind moves the flow without
                         # reaching the transonic blend at Mach 0.75. The two outer rows
                         # cost a fourteenth of that: 40 m/s in 0.162 kg/m3 is 130 Pa.
                         # NO wind_gen here, and check_fact_wind refuses it by name: the
                         # page says WHERE the speed lives, and a Rhines-scale field would
                         # put derived numbers inside a latitude bound the sheet set.
                         'wind': [{'alt': 0, 'east': 150.0, 'north': 0.0,
                                   'lat': [-30.0, 30.0]},
                                  {'alt': 0, 'east': 40.0, 'north': 0.0,
                                   'lat': [-90.0, -30.0]},
                                  {'alt': 0, 'east': 40.0, 'north': 0.0,
                                   'lat': [30.0, 90.0]}]}},
         False),
        ('saturnfact.html', 'Saturn', 6.0,
         {'bands': True, 'band_count': 9,
          'palette': [[0.0, [0.62, 0.52, 0.38]], [1.0, [0.90, 0.82, 0.65]]],
          'atmosphere': {'color': [0.90, 0.82, 0.65], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         # #625 closes #705 by accident. This body authored 0.15 kg/m3
                         # on H 60 km, and the comment here explained why the sheet's
                         # 0.19 was ignored: the trio lock says a 1-bar datum with that H
                         # weighs 0.149, and the sheet's rows disagreed by 28%. They
                         # disagree because of the 173.5 K that H implied. Take the
                         # sheet's OWN temperature at 1 bar (134 K) and its 0.19 falls out
                         # of 1 bar and this mix to 1.5%: the datum reads 1.000 bar and
                         # resolves 0.1929 kg/m3, and the argument has nothing left to
                         # waive. Profile from its three rows (134 K, 84 K at 0.1 bar,
                         # H 59.5 km): -0.46 K/km to a seam at 109,462 m. Ceiling 400 km
                         # (257 Pa of q); the 800 km #682 chose against an isothermal
                         # column now reads 1e-6 Pa there. Same caveat as the other
                         # giants (#701).
                         # Saturn's envelope is the most H2-rich of them (0.963), but
                         # H2 with no oxidizer is a fuel store, not a jet air.
                         # 0.15, not the sheet's 0.19: this file's datum IS the 1-bar
                         # level (the same convention its gravity row states -- "mean,
                         # 1 bar" -- and that #689's gravity fallback assumes), and the
                         # trio lock says rho at 1 bar is p0/(g H) = 0.149 here. The
                         # sheet's own density row disagrees with its own H 60 km,
                         # T 173.5 K and M 2.15 g/mol by 28%, which is #685's measured
                         # finding that the giants' sheets are 4-19% self-inconsistent;
                         # of its three statements we honour the two that fix the
                         # column, so the barometer at this datum reads 1.007 bar.
                         # check_fact_air still prints the -21% against the density row
                         # (inside its 30% band) rather than hiding it. (#700)
                         'composition': [{'gas': 'H2', 'fraction': 0.963},
                                         {'gas': 'He', 'fraction': 0.032},
                                         {'gas': 'CH4', 'fraction': 0.005}],
                         # Wind (#772). saturnfact.html states the same latitude
                         # split Jupiter's page does, with faster numbers:
                         # "Up to 400 m/s (<30 degrees latitude) Up to 150
                         # m/s (>30 degrees latitude)". Both halves at the
                         # 1-bar datum, tiled the way #767 tiles them.
                         # This is the most energetic wind in the
                         # catalogue: 400 m/s over 0.193 kg/m3 is 15.4 kPa
                         # of dynamic pressure at the deck -- about four
                         # times what a hull at Kerbin cruise already pays
                         # (1.6-3.8 kPa) and eight times Jupiter's row.
                         # Mach 0.47 on the 853 m/s this mix gives at 134 K,
                         # so the lift law stays on its subsonic branch and
                         # under the transonic blend at Mach 0.75; a hull
                         # that adds its own 150 m/s to a 400 m/s headwind
                         # is at 29 kPa and Mach 0.64 before any gust, which
                         # is why #716 (nothing caps an aero impulse) wants
                         # settling before anyone flies fast in this air.
                         # The two outer rows cost an order of magnitude
                         # less: 150 m/s in the same air is 2.2 kPa.
                         # No wind_gen: the sheet bounds wind by latitude,
                         # and check_fact_wind refuses a generated field
                         # under such a sheet.
                         'wind': [{'alt': 0, 'east': 400.0, 'north': 0.0,
                                   'lat': [-30.0, 30.0]},
                                  {'alt': 0, 'east': 150.0, 'north': 0.0,
                                   'lat': [-90.0, -30.0]},
                                  {'alt': 0, 'east': 150.0, 'north': 0.0,
                                   'lat': [30.0, 90.0]}]}},
         False),
        ('uranusfact.html', 'Uranus', 7.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.40, 0.65, 0.70]], [1.0, [0.70, 0.85, 0.88]]],
          'atmosphere': {'color': [0.70, 0.85, 0.88], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         # #695, landing with the giants' air tops: the sheet's own PAIR,
                         # 0.42 kg/m^3 at its 1-bar level and 27.7 km. Those two are one
                         # statement, not two -- rho g H = 0.42 x 9.007 x 27.7e3 =
                         # 104.8 kPa, i.e. a 1.048 bar datum, which is the level this
                         # file's datum IS. The old 0.60 / 20 km was what was left beside
                         # the typed molar_mass 0.023 that #630 removed; it read a 57 K
                         # datum where the sheet's own M 2.64 and H 27.7 km say 79 K.
                         # It could not land before the top decision, because on a body
                         # with no authored height the scale height IS the top: H 27.7 km
                         # moves 10 H from 200 to 277 km and with it the rot frame, every
                         # spawn bed and the atlas disk. Both FACT_AIR_KNOWN waivers named
                         # that one decision, so both are gone now.
                         # #625: 76 K at 1 bar, 53 K at 0.1 bar and H 27.7 km make this
                         # table -- -0.43 K/km to a seam at 53,552 m -- and the 1 bar
                         # datum resolves 0.4179 kg/m3 against the sheet's 0.42. The old
                         # pair read 79.24 K, 4% over its own sheet. Ceiling 250 km
                         # (174 Pa of q), in from 370 km because a lapsing column thins
                         # faster aloft; shellEdge 444 -> 300 km (#701's caveat again).
                         # Uranus's own H2/He/CH4 proportions, not gen_systems' Jool
                         # (#636: same three gases, a different sky).
                         'composition': H2_HE_CH4_SOLAR,
                         # Wind (#772). uranusfact.html states one bare range -- "0-250
                         # m/s" -- with no latitude structure and no level, so the two
                         # ends of that range become this column's two rows and the
                         # latitude structure is the generator's (#775).
                         # The page's 250 m/s is not put at the deck: the two air tables
                         # this file already has (Earth's jet at 11 km, Venus's
                         # superrotation at 65 km) both put their fast flow above the
                         # surface, and a steady profile is not a forecast (#688), so the
                         # deck takes the low end and the ceiling goes to 30 km. What
                         # that costs the aero law: 100 m/s over 0.418 kg/m3 is 2.1 kPa
                         # at the datum (Mach 0.17 on the 584 m/s this mix gives at 76 K),
                         # 250 m/s over 0.146 kg/m3 is 4.6 kPa at 30 km (Mach 0.47) --
                         # the same order as a Kerbin cruise, so no transonic blend.
                         # THE GENERATOR, allowed here because the sheet says nothing
                         # about latitude: with this body's 17.24 h day and 25,362 km
                         # radius, U = 250 m/s gives Ro 0.049 and a Rhines scale of 13.6
                         # deg at 30 deg latitude, so 6 belts per hemisphere -> 13 bands x
                         # 2 anchors = 26 rows, inside the loader's 64. --check prints
                         # that row. #775's own table says this scaling overpredicts the
                         # ice giants (and Uranus rolls at 98 deg obliquity, which the
                         # generator cannot represent), so the belts are layout, not
                         # another fact the page states.
                         'wind': [{'alt': 0, 'east': 100.0, 'north': 0.0},
                                  {'alt': 30000, 'east': 250.0, 'north': 0.0}],
                         'wind_gen': 'rhines'}},
         False),
        ('neptunefact.html', 'Neptune', 8.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.18, 0.35, 0.75]], [1.0, [0.50, 0.65, 0.90]]],
          'atmosphere': {'color': [0.50, 0.65, 0.90], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         # #625: 72 K at 1 bar, 55 K at 0.1 bar, and a sheet that
                         # states its scale height as a RANGE (19.1-20.3 km) -- the seam sits
                         # where that lapse meets 55 K, 39,766 m, and the datum
                         # resolves 0.4345 kg/m3 against the sheet's 0.45. Ceiling 200 km
                         # (324 Pa of q); shellEdge 324 -> 240 km (#701).
                         # Neptune's sheet states 0.45 kg/m^3 at its 1-bar level and
                         # that IS what its own H 19.1-20.3 km implies at our g
                         # (1e5 / (11.274 x 20000) = 0.443), so this row needed no
                         # judgement -- the old 0.90 was simply wrong, and #700 could
                         # only name it by hand because check_fact_air had no eyes for
                         # a "Density at 1 bar" row. It now reads +1.4% and the datum
                         # barometer says 1.015 bar.
                         'composition': [{'gas': 'H2', 'fraction': 0.80},
                                         {'gas': 'He', 'fraction': 0.19},
                                         {'gas': 'CH4', 'fraction': 0.015}],
                         # Wind (#772). neptunefact.html gives a bare range like Uranus's:
                         # "0-580 m/s" -- no latitude structure, so the column is authored
                         # and the belts come from the generator (#775).
                         # 580 m/s is NOT authored anywhere, and that is a deliberate cap
                         # under the sheet's ceiling -- which the guard allows (it only
                         # refuses exceeding it; Earth's row does the same, 40 m/s at 11 km
                         # rather than its sheet's 100). The reason is what the number costs
                         # at this body's 1-bar datum: 580 m/s over 0.434 kg/m3 is 73 kPa of
                         # dynamic pressure and Mach 1.01 on the 574 m/s sound speed this
                         # mix gives at 72 K, i.e. the lift law sits in the middle of its
                         # transonic blend (kTransonicMach 0.75 to kSupersonicMach 1.25),
                         # and #716 has nothing capping an aero impulse -- the blow-up it
                         # documents happened at 810 kPa. A row that starts a hull off at
                         # Mach 1 in dense air is a physics statement the aero law has not
                         # earned yet, so the fast row goes 300 m/s at 25 km: 6.1 kPa and
                         # Mach 0.57, twice a Kerbin cruise and still subsonic. The deck
                         # takes the range's low end, 120 m/s = 3.1 kPa, Mach 0.21.
                         # The generator over this column asks for 6 belts per hemisphere
                         # (Ro 0.056, Rhines scale 14.6 deg at 30 deg latitude on a 16.11 h
                         # day), so 13 bands x 2 anchors = 26 rows -- under the loader's 64,
                         # and the widest wind in that table is still the 300 m/s authored
                         # here, because windGenerate scales belts by amp <= 1.
                         'wind': [{'alt': 0, 'east': 120.0, 'north': 0.0},
                                  {'alt': 25000, 'east': 300.0, 'north': 0.0}],
                         'wind_gen': 'rhines'}},
         False),
        ('plutofact.html', 'Pluto', 9.0,
         {'amplitude': 2000, 'palette': [[0.0, [0.55, 0.45, 0.38]],
                                         [0.5, [0.70, 0.62, 0.55]],
                                         [1.0, [0.85, 0.82, 0.78]]],
          'atmosphere': {'color': [0.70, 0.75, 0.85], 'thickness': 8000,
                         'power': 5.0, 'intensity': 0.3,
                         # Pluto's air is AUTHORED now, not a painted haze (#700). Every
                         # number below it is the sheet's own: "Surface Pressure: ~13
                         # microbar", "Scale height: ~18 km lower atmosphere", "99% molecular
                         # nitrogen, 0.5% methane". Its 0.05% CO has no entry in
                         # atmo.h::GASES, so the two gases we CAN name are authored as
                         # written and the trace is left out rather than folded into them.
                         # Density is the one figure the sheet cannot hand us: a system file
                         # the datum pressure below is this density's consequence (#739)
                         # (#684) and it is derived -- 1.3 Pa / (0.62 x 18000) = 1.164e-4.
                         # check_fact_air then weighs OUR p0 against the published 1.3 Pa
                         # instead of printing "no column authored", and weighs the 18 km
                         # scale height against its own row.
                         # The trio (#686) closes on the sheet's numbers instead of against
                         # them: 18 km of N2 at g 0.62 implies T0 = g H M / R = 37.5 K,
                         # inside its published 24-38 K and 1.3% under its top.
                         # The ceiling is 20 km, NOT the 30 km where the sheet says that 18 km
                         # scale height stops applying: #682's band wants 10..1000 Pa of q on
                         # a circular orbit just under the top, and this column gives ~14 Pa at
                         # 20 km but ~8 Pa at 30 km -- under the floor, i.e. air that could
                         # never push on anything a rail puts through it. The band gate
                         # (utils/check_atmo_band.py) prints the row; measure, do not quote
                         # this comment.
                         # Nothing else moves: shellEdge = max(kMinShell 100 km, 1.2 x 20 km)
                         # is the 100 km floor airless Pluto already had, so its rot frame,
                         # SOI, spawn bands and atlas disk are what they were. What it gains is
                         # flying beds, and 1.16e-4 of air 20 km up brakes about as hard as
                         # nothing at all.
                         # #625: the conversion that changes nothing, which is the
                         # point -- the sheet's 1.3 Pa, its ~18 km and its warm-end 38 K
                         # are already one consistent column (0.000115 kg/m3 resolved
                         # against the 1.164e-4 this used to type). One isothermal level
                         # at 38 K under a 20 km ceiling; its sheet's second scale-height
                         # row ("~50 km above 30 km") is a mesosphere above this top and
                         # is left unread rather than invented.
                         'composition': [{'gas': 'N2', 'fraction': 0.99},
                                         {'gas': 'CH4', 'fraction': 0.005}]}}, False),
    ]
    rings_by_name = {'Jupiter': JUP_RINGS, 'Saturn': SAT_RINGS}
    sheets = {}
    air_bad = []          # #625: profiles a sheet refuses to support
    for page, name, seed, surface, has_sea in planets:
        data = parse_planet(page)
        parsed[name] = data
        # #687: the ground's thermal state, authored from the same page that states
        # the air over it. albedo is the sheet's Bond (not geometrical) albedo, and
        # temperature its prose average -- which on a giant is the temperature AT the
        # 1-bar level, the same level this file's datum is. Bodies the sheets give no
        # number for are left UNAUTHORED (the loader dashes them), never zeroed.
        sheet = fact_surface(page)
        sheets[name] = sheet
        if sheet['albedo'] is not None:
            surface['albedo'] = sheet['albedo']
        if sheet['avg_T'] is not None:
            surface['temperature'] = sheet['avg_T']
        # #625: the same page's air rows, turned into a profile instead of a scale
        # height. Before the body is built, so the emitted file carries the table and
        # the guard below sees one authored air, not two.
        air_bad += author_air_profile(name, page, surface, data,
                                      verbose=verbose_air)
        bodies.append(make_body(name, 'planet', 'Sun', data,
                                surface=surface, seed=seed, has_sea=has_sea,
                                rings=rings_by_name.get(name)))
    # The star's output is ONE authored number every body's flux derives from, so
    # nothing internal can see it go wrong -- the #684/#686 meta-law again. Author it
    # from a row the corpus DOES state: Earth's 'Solar irradiance 1361.0 W/m^2', read
    # at Earth's own rail and inverted through solar.h's L = 4 pi R^2 sigma T^4.
    # check_fact_surface() then weighs what that one temperature implies at every
    # OTHER body against that body's own irradiance row: one authored number, eight
    # published cross-checks. The KSP catalogue's Kerbol row (utils/ksp_bodies.csv:
    # 4014.95 - 9621.2 K) disagrees with this by design -- that world's rail is no AU and
    # its wiki "temperature" pair is a photosphere-to-corona range, not a surface (#687).
    # gen_systems prints its own version of that disagreement instead of resolving it.
    earth = next(b for b in bodies if b['name'] == 'Earth')
    assert sheets['Earth']['irradiance'], 'no Earth irradiance row to fix the star by'
    a_earth = semimajor_axis(G * SUN_MASS, earth['inertial']['orb_ang_speed'])
    T_sun = star_temperature_from_flux(sheets['Earth']['irradiance'],
                                       a_earth, SUN_RADIUS)
    sun['surface']['temperature'] = round(T_sun, 2)
    if verbose_air:
        print('  star      Sun T %10.2f K  from Earth %10.1f W/m2 at %12.4g m'
              % (T_sun, sheets['Earth']['irradiance'], a_earth))
    # The air each of those pages states in prose, against the air this script
    # authors beside it (#685). Refuses before any file is written.
    bad = check_fact_air(planets, parsed, verbose=verbose_air)
    bad += air_bad
    if bad:
        for m in bad:
            print('DRIFT   fact air: ' + m)
        raise SystemExit('%d authored air datum(s) disagree with the fact sheet '
                         'that states them (bands: FACT_AIR_BANDS)' % len(bad))
    # ... and the air's own motion (#688), which the guard above never looked at.
    # Moons author their air in MOON_ATMOS, outside the `planets` list this guard walks,
    # so a mooned wind table would reach the game unweighed -- which is not how Titan's
    # rows should pass, since their page states no wind at all and that is precisely the
    # case FACT_WIND_KNOWN exists to make someone argue for (#772).
    winded = list(planets)
    for mname, matm in MOON_ATMOS.items():
        if not matm.get('wind'):
            continue
        mpage = MOON_AIR_PAGES.get(mname)
        if mpage is None:
            raise SystemExit('%s authors wind rows in MOON_ATMOS with no page in '
                             'MOON_AIR_PAGES to weigh them against' % mname)
        winded.append((mpage, mname, 0.0, {'atmosphere': matm}, False))
    bad = check_fact_wind(winded, verbose=verbose_air, parsed=parsed)
    if bad:
        for m in bad:
            print('DRIFT   fact wind: ' + m)
        raise SystemExit('%d authored wind row(s) exceed what the fact sheet that '
                         'states them allows (waivers: FACT_WIND_KNOWN)' % len(bad))
    # And the guard's own red mutation (#775): the generated-vs-published-latitude-band
    # rule has nothing else in the tree that can turn it off, so prove here that it fires.
    bad = self_test_fact_wind()
    if bad:
        for m in bad:
            print('SELF-FAIL fact wind: ' + m)
        raise SystemExit('the fact-wind guard can no longer be made to fail, so it is '
                         'decoration rather than a gate')
    # ... and the light that air sits under (#687): the star's one authored
    # temperature against every body's published irradiance and black-body rows.
    bad = check_fact_surface(bodies, sheets, sun, verbose=verbose_air)
    if bad:
        for m in bad:
            print('DRIFT   fact surface: ' + m)
        raise SystemExit('%d insolation figure(s) disagree with the fact sheets '
                         'that state them (band %d%%)'
                         % (len(bad), round(100 * FACT_SURFACE_BANDS)))
    # ... and the one figure about gravity the pages state independently of mass and
    # radius (#708). A file no longer types `g` -- the loader derives it -- so a
    # mis-scraped MASS now sets a body's surface gravity too, and the sheet's own gravity
    # row is the only remaining witness. Weighing it here names the page that produced
    # the disagreement rather than leaving it in play.
    bad = check_scraped_gravity(parsed, verbose=verbose_air)
    if bad:
        for m in bad:
            print('DRIFT   scraped gravity: ' + m)
        raise SystemExit('%d fact-sheet gravity rows disagree with the same page\'s own'
                         ' mass and radius' % len(bad))
    # transfer_dv for a planet needs home (the heliocentric hop); stamp now
    # that every base body exists.
    home = next(b for b in bodies if b['name'] == 'Earth')
    for b in bodies:
        if b['name'] != 'Sun':
            stamp_transfer_dv(b, sun, home)
    # The fact-sheet rows stay alongside the bodies rather than inside them: a body carries
    # what this file AUTHORS, and the sheet is what the corpus STATES. emit() needs the
    # second to fit the ground's swing (#724 rung C) and to keep weighing one against the
    # other, which is the whole point of keeping them apart.
    return bodies, parsed, sheets

# Which satellites get an authored albedo, and the page and block each reading came from.
# Deliberately TWO entries, and the shortness is a fact about the corpus, not an oversight:
# every satellite page in utils/rss/html/ states a "Visual geometric albedo" and NO Bond
# albedo (joviansatfact.html, saturniansatfact.html, uraniansatfact.html,
# neptuniansatfact.html), and geometric -> Bond needs a phase integral nothing here gives
# (#736). The rule #735's authorship leans on is BOND, never geometrical, so Europa and
# Titan keep their dash in the dossier because the corpus declines to answer, not because
# this script forgot to ask. The two pages that do say the word are the Moon (its own
# Bulk-parameters table) and Charon (a block inside Pluto's page -- with a "Geometric
# Albedo 0.42" row sitting two lines above the "Bond Albedo 0.25" one, which is exactly the
# trap #735 was filed over).
SOLAR_SAT_ALBEDOS = {
    'Moon':   ('moonfact.html',  'Bulk parameters'),
    'Charon': ('plutofact.html', 'Charon (P1)'),
}

# /* #749: the satellites whose pages state a VISUAL GEOMETRIC albedo and no Bond albedo. The
#    satellite fact sheets carry four columns (mass, radius, density, albedo) and the albedo they
#    carry is the wrong kind -- geometric, not Bond -- which is why these bodies dashed a Bond
#    albedo for so long (#736). The conversion is A_bond = q * p_geo, and q is NOT invented here:
#    it is measured below from the ten pages in this corpus that state BOTH, and its spread (a
#    factor of three) is printed every run so nobody mistakes the product for a reading. */

SOLAR_SAT_GEOMETRIC = {
    'Io':        ('joviansatfact.html',   'Io'),
    'Europa':    ('joviansatfact.html',   'Europa'),
    'Ganymede':  ('joviansatfact.html',   'Ganymede'),
    'Callisto':  ('joviansatfact.html',   'Callisto'),
    'Mimas':     ('saturniansatfact.html', 'Mimas'),
    'Enceladus': ('saturniansatfact.html', 'Enceladus'),
    'Tethys':    ('saturniansatfact.html', 'Tethys'),
    'Dione':     ('saturniansatfact.html', 'Dione'),
    'Rhea':      ('saturniansatfact.html', 'Rhea'),
    'Titan':     ('saturniansatfact.html', 'Titan'),
    'Iapetus':   ('saturniansatfact.html', 'Iapetus'),
    'Miranda':   ('uraniansatfact.html',  'Miranda'),
    'Ariel':     ('uraniansatfact.html',  'Ariel'),
    'Umbriel':   ('uraniansatfact.html',  'Umbriel'),
    'Titania':   ('uraniansatfact.html',  'Titania'),
    'Oberon':   ('uraniansatfact.html',   'Oberon'),
    'Triton':    ('neptuniansatfact.html', 'Triton'),
    # Phobos and Deimos are a two-column table inside marsfact, not rows.
    'Phobos':    ('marsfact.html',        'Phobos'),
    'Deimos':    ('marsfact.html',        'Deimos'),
}

# The fact-sheet pages whose Bulk-parameters block states BOTH albedos, so the phase integral
# can be MEASURED from the corpus instead of typed. Adding a page here changes the constant,
# which is the point: the constant belongs to the corpus, not to this file.
PHASE_INTEGRAL_PAGES = ('mercuryfact.html', 'venusfact.html', 'earthfact.html',
                        'marsfact.html', 'jupiterfact.html', 'saturnfact.html',
                        'uranusfact.html', 'neptunefact.html', 'plutofact.html',
                        'moonfact.html')

def corpus_phase_integral(pages=PHASE_INTEGRAL_PAGES, verbose=False):
    """[(q, note)] The corpus's own implied phase integrals, q = A_bond / p_geo, and the MEDIAN.

    A Bond albedo is what the energy laws need and a geometric albedo is what the satellite
    sheets state, so the ratio between them is the one number standing between a satellite and
    an anchor. Ten pages in this corpus state both, and their ratios scatter from 0.48 to 1.47
    -- the corpus does NOT support a single conversion, and any q is a decision. The median is
    the decision (the mean, 0.86, is dragged by Mars's 1.47), and the spread is printed so the
    converted albedo reads as what it is: a converted figure whose uncertainty is a few percent
    in the anchor (T goes as (1-A)^1/4)."""
    ratios = []
    missing = []
    for page in pages:
        v = section_values(read(page), 'Bulk parameters')
        geo, bond = v.get('geometricalbedo'), v.get('bondalbedo')
        if geo and bond:
            ratios.append((page[:8], bond / geo))
        else:
            missing.append(page)
    # EVERY named page, not "enough of them": this median prices 19 satellite albedos and
    # every anchor downstream of them, so a page quietly losing one of its two rows must stop
    # the build rather than move the median. --check would catch the file drift today; this
    # catches it at the cause, and names the page.
    assert not missing, (
        'the phase integral reads BOTH albedos from every page it names; these state one or '
        'neither: %s' % missing)
    assert len(ratios) == len(pages), 'phase-integral pages counted %d of %d' % (
        len(ratios), len(pages))
    vals = sorted(r for _, r in ratios)
    q = vals[len(vals) // 2] if len(vals) % 2 else 0.5 * (
        vals[len(vals) // 2 - 1] + vals[len(vals) // 2])
    note = ('q %.3f = the median of %d (bond, geometric) pairs this corpus states '
            '(%s); range %.2f-%.2f, mean %.3f'
            % (q, len(vals), ' '.join('%s %.2f' % (n, r) for n, r in ratios),
               vals[0], vals[-1], sum(vals) / len(vals)))
    if verbose:
        print('  phase     %s' % note)
    return q, note

def albedo_cell(v, raw, page, row):
    """[(value, cell text)] One albedo cell, RANGE-CHECKED as a geometric albedo.

    Why a range and not just `0 <= A < 1` on the product: the last cell of a satellite row is
    the albedo column TODAY, and a scrape that appends a column -- a mass in Earth masses is
    ~1e-4 -- would otherwise be read as an albedo of 1e-4, pass every downstream assert (q x
    1e-4 is a perfectly legal Bond albedo) and dim that moon's anchor in silence. This is the
    one place the number is known to BE an albedo, so it is the place to insist."""
    assert 0.004 <= v <= 1.2, (
        '%s/%s: the cell this parse took as a geometric albedo is %g (%r), which is not one'
        % (page, row, v, raw))
    return v, ' '.join(raw.split())


def sat_geometric_albedo(page, row, verbose=False):
    """[(value, the cell's own text)] The visual geometric albedo one satellite's row states,
    or a hard error. The cell text comes back too because it goes into the file's provenance
    string: for Iapetus the number we use is a MEAN of two hemispheres, and that sentence has
    to travel with the albedo rather than stay in the generator's stdout (#687).
    Row tables put the name in the first cell; the two-column Mars table puts labels in the
    first cell and one value per column, so both shapes end in the same read: the number in
    the 'geometric albedo' row/column for this name."""
    h = read(page)
    if page == 'marsfact.html':
        i = h.find('Satellites of Mars')
        rows = table_rows(re.search(r'<table.*?</table>', h[i:], re.S).group(0))
        # The heading sits outside the table, so the name row is the one whose label cell is
        # empty: ['', 'Phobos', 'Deimos'].
        cols = [r for r in rows if r and not norm(r[0]) and any(norm(c) for c in r[1:])]
        names = [norm(c) for c in (cols[0][1:] if cols else [])]
        assert names, 'marsfact: no name row in the satellites table'
        ci = names.index(norm(row)) if norm(row) in names else None
        assert ci is not None, 'marsfact: no %s column in the satellites table' % row
        for r in rows:
            if r and 'geometricalbedo' in norm(r[0]):
                raw = r[1 + ci]
                v = num(raw)
                assert v is not None, 'marsfact: %s geometric albedo cell is not a number' % row
                return albedo_cell(v, raw, page, row)
        raise AssertionError('marsfact: no Geometric albedo row in the satellites table')
    for tr in re.findall(r'<tr.*?</tr>', h, re.S):
        c = cells(tr)
        if not c:
            continue
        # The name cell carries the designation too -- 'Titania (III)', 'Io (I)' -- and a
        # PREFIX match would let 'Titan' read Titania's row on a page holding both, so strip
        # the designation and compare whole names.
        head = norm(c[0].split('(')[0])
        if head == norm(row) and len(c) >= 2:
            v = c[-1]
            nums = [num(x) for x in (v.split('/') if '/' in v else [v])]
            nums = [x for x in nums if x is not None]
            if not nums:
                continue
            if len(nums) > 1:
                # Iapetus states "0.05 / 0.5": a leading and a trailing hemisphere, and our
                # sphere has one albedo. The mean is the only reading that does not silently
                # pick a hemisphere, and it prints so the choice is visible (#687).
                if verbose:
                    print('  albedo    %-8s the sheet states %s (two hemispheres); mean %g used'
                          % (row, v.strip(), sum(nums) / len(nums)))
                return albedo_cell(sum(nums) / len(nums), v, page, row)
            return albedo_cell(nums[0], v, page, row)
    raise AssertionError('%s: no row for %s in the satellite table (the scrape moved?)'
                         % (page, row))

def author_diurnal(bodies, by, sun, sheets, sat_sheet, verbose=True):
    """#724 rung C: author the surface model's ONE knob from the diurnal pair the corpus
    states, and print the verdict for every body -- refusals and their residuals included.

    One knob means a stated pair is OVER-determined, so it becomes a test rather than a fit:
    fit on the night end (the model's anti-solar floor, the end it gets structurally right)
    and PREDICT the day end. See utils/ground_model.py for the law and for why the energy
    anchor is exact; the gate is why a lander-site pair (Mars's) is refused instead of
    swallowed into a global number."""
    T_sun = (sun.get('surface') or {}).get('temperature')
    rows = []
    fitted, pressures, anchors = {}, {}, {}

    def price(b, f, src, T0, why, note, verbose=verbose):
        at = (b.get('surface') or {}).get('atmosphere') or {}
        b.setdefault('surface', {})['even_share'] = f
        b['surface']['even_share_src'] = src
        extra = even_share_air_note(b['name'], f, at.get('sea_level_pressure'))
        if extra:
            extra = '\n%18s%s' % ('', extra)
        if verbose:
            rows.append('%-9s diurnal %.4f  anchor %7.2f K (%s)%s\n%18s%s'
                        % (b['name'], f, T0, why, extra, '', note))

    for b in bodies:
        if b.get('type') == 'star':
            continue                      # its 'temperature pair' is a photosphere range
        at0 = ((b.get('surface') or {}).get('atmosphere') or {})
        pressures[b['name']] = at0.get('sea_level_pressure') or 0.0
        # The anchor FIRST, for every ground-bearing body whether or not a page states a
        # pair: the default pass below needs to know which bodies have a ground to price at
        # all, and asking it per body avoids a second copy of the anchor chain (#636).
        sheet = sheets.get(b['name']) or sat_sheet.get(b['name'])
        # Two spellings of "this page states a range": the diurnal row, and Pluto's surface
        # row. They are read as ONE because the pass below asks what the page claims about
        # the ground's two ends, not which heading it used.
        pair = (sheet.get('diurnal') or sheet.get('surf_range')) if sheet else None
        star, a = star_rail(b, by)
        flux = flux_at_rail(star, a, T_sun) if star else None
        albedo = (b.get('surface') or {}).get('albedo')
        # The derived black-body, from the SAME pinned spelling the surface guard above
        # uses -- ground_anchor() takes it as an argument rather than computing its own.
        derived = (blackbody_temperature(flux, albedo)
                   if (flux and albedo is not None) else 0.0)
        T0, why = ground_anchor(b, derived)
        anchors[b['name']] = (T0, why)
        # A stated RANGE with no stated mean prices BOTH unknowns exactly (ground_model's
        # pair_solve), and it beats the ladder's air-datum and derived rungs because a page
        # that gives the ground's two extremes is talking about the ground. The Moon and
        # Pluto are the whole set: every other page either states an average -- where the pair
        # stays a TEST of one knob, which is #727's point and unchanged below -- or states
        # nothing. Banded giants are in this loop now: their 1-bar datum IS the surface the
        # file authors, and what stops a pair being read for them is the same thing that
        # stops it for a Kerbal air -- the page never claims the ends are a night and a day.
        if pair is not None and pair[1] > pair[0] \
                and (not (b.get('surface') or {}).get('temperature')
                     or str(((b.get('surface') or {}).get('temperature_src')
                             or '')).startswith('SOLVED')):
            # The `or SOLVED` half is idempotence, and emit() needs it: the four variants
            # share their planet dicts, so a body solved on the core pass arrives here with a
            # temperature already on it. Without this it falls through to the night-end fit
            # below and keeps its (identical) number under someone else's provenance string --
            # an emitted file whose words depend on which variant ran first. That is what it
            # did before this line existed: Pluto came out of the three non-core variants
            # saying "fitted from the night end" for a knob the stated range priced.
            f, T_pair = pair_solve(pair[0], pair[1])
            ok, note = pair_anchor_check(T_pair, derived)
            waiver = PAIR_ANCHOR_KNOWN.get((b['name'], round(T_pair, 2)))
            if ok or waiver:
                surf = b.setdefault('surface', {})
                # The anchor lands in the key every consumer already reads -- surfmap, the
                # air-follow coupling, the dossier -- and says what it is in the sibling src
                # string, the way albedo_src and even_share_src do. A fourth rung in the
                # anchor ladder would mean spelling this algebra in C++ for nothing (#636).
                surf['temperature'] = T_pair
                surf['temperature_src'] = (
                    'SOLVED from the stated range %.2f-%.2f K, not a stated mean'
                    % (pair[0], pair[1]))
                surf['even_share'] = f
                surf['even_share_src'] = SRC_PAIR_BOTH
                fitted[b['name']] = (f, SRC_PAIR_BOTH)
                if verbose:
                    rows.append('%-9s diurnal %.4f  anchor %7.2f K (SOLVED from the stated '
                                'range %.2f-%.2f K)%s\n%18s%s'
                                % (b['name'], f, T_pair, pair[0], pair[1],
                                   ('  WAIVED: ' + waiver) if waiver else '', '', note))
                continue
            if verbose:
                rows.append('%-9s diurnal -   %7.2f K from the range, but %s'
                            % (b['name'], T_pair, note))
        if pair is None:
            if verbose and sheet:
                rows.append('%-9s diurnal -   the page states no temperature range'
                            % b['name'])
            continue
        if T0 is None:
            if verbose:
                rows.append('%-9s diurnal -   pair %.1f-%.1f K but %s'
                            % (b['name'], pair[0], pair[1], why))
            continue
        f, src, note = fit_even_share_pair(T0, pair[0], pair[1])
        if f is None:
            # Not a refusal any more (#749): the pair is printed as unmeetable by BOTH ends
            # and the body falls through to the default pass below. A knob that contradicts
            # the sheet it came from is worse than a default that announces itself.
            rows.append('%-9s diurnal -   anchor %7.2f K (%s)\n%18s%s'
                        % (b['name'], T0, why, '', note))
            continue
        fitted[b['name']] = (f, src)
        price(b, f, src, T0, why, note)
    # --- the default pass (#749). A body whose source states no pair at all, or states one
    # our law cannot meet, still gets a knob -- from a NAMED rule whose two constants are
    # MEASURED from this run's own fits (Earth's for the air, the Moon's for the airless
    # floor), so no number here is typed (#636). The file carries the words, and the dossier
    # prints DEFAULT beside the knob (#722's rule that a computed figure says so). Only stars
    # stay out: a star's photosphere is not a ground under a star. Banded giants are IN --
    # their 1-bar datum is the surface this file authors for them, and a 1-bar air is exactly
    # what the air-mass rule says spreads day into night.
    pair_fits = {n: f for n, (f, src) in fitted.items()
                 if src in (SRC_PAIR_NIGHT, SRC_PAIR_DAY, SRC_PAIR_BOTH)}
    if pair_fits:
        p_ref, floor, cal_note = calibration_from_fitted(pair_fits, pressures)
        if verbose:
            rows.append('  %s' % cal_note)
        for b in bodies:
            if b.get('type') == 'star':
                continue
            if (b.get('surface') or {}).get('even_share') is not None:
                continue
            T0, why = anchors.get(b['name'], (None, None))
            if T0 is None:
                continue        # nothing anchors this ground; the readouts dash regardless
            if b['name'] not in sheets and b['name'] not in sat_sheet:
                verbose_def = False     # a catalogue moon with no page: no line per moon
            else:
                verbose_def = verbose
            p = pressures.get(b['name'], 0.0)
            f, dsrc = default_even_share(p, p_ref, floor)
            price(b, f, dsrc, T0, why,
                  'no source pair to fit: %s, from %s at %g Pa'
                  % (dsrc, 'the authored air column' if p > 0.0 else 'no air', p),
                  verbose=verbose_def)
    if verbose:
        for r in rows:
            print('  %s' % r)


def satellite_sheets(verbose=True):
    """{satellite name: {albedo, irradiance, bb_T}} for the satellites whose page states a
    BOND albedo -- shaped like fact_surface()'s output so the same surface guard can weigh
    them. A missing heading or a missing albedo row is an assertion, not a dash: a silently
    unpriced moon is the bug this exists to fix (#737), whereas a page that states no Bond
    albedo is simply not in SOLAR_SAT_ALBEDOS.

    No satellite gets an authored GROUND temperature here: the corpus's satellite rows stop
    at the albedo, so the dossier dashes surface temp and the gap for them (#722)."""
    out = {}
    q, q_note = corpus_phase_integral(verbose=verbose)
    for name, (page, marker) in sorted(SOLAR_SAT_ALBEDOS.items()):
        v = section_values(read(page), marker)   # raises if the block moved
        a = v.get('bondalbedo')
        assert a is not None, (
            '%s: %s has a %r block but no Bond albedo row in it -- either the scrape '
            'moved or the corpus stopped saying' % (name, page, marker))
        out[name] = {'albedo': a,
                     'irradiance': v.get('solarirradiancewm2'),
                     'bb_T': v.get('blackbodytemperaturek'),
                     # The same diurnal-pair row, when the page has one. moonfact states one
                     # for the Moon's EQUATOR ("95 K to 390 K"), which is the comparison the
                     # model's own extremes answer to -- the anti-solar point is on the
                     # sub-solar latitude, at the equator at equinox.
                     'diurnal': parse_diurnal_pair(
                         sheet_line(read(page), 'Diurnal temperature range'))}
        if verbose:
            geom = v.get('geometricalbedo')
            print('  sat sheet %-7s albedo %6.3f  from %s (%s)%s'
                  % (name, a, page, marker,
                     '; that block also states Geometric Albedo %g' % geom
                     if geom is not None else ''))
    # ... and the rest, converted (#736/#749). Stated beats converted: the two bodies whose
    # pages state a Bond albedo outright keep it, and nothing here overwrites a reading.
    for name, (page, row) in sorted(SOLAR_SAT_GEOMETRIC.items()):
        assert name not in out, (
            '%s: stated in SOLAR_SAT_ALBEDOS and converted in SOLAR_SAT_GEOMETRIC' % name)
        p_geo, cell = sat_geometric_albedo(page, row, verbose=verbose)
        a = q * p_geo
        assert 0.0 <= a < 1.0, (
            '%s: converted albedo %g (q %.3f x geometric %g) is not a Bond albedo'
            % (name, a, q, p_geo))
        out[name] = {'albedo': a, 'irradiance': None, 'bb_T': None, 'diurnal': None,
                     # %.3f, not %g: this string is a dossier row, and six decimals of a
                     # factor the corpus itself disagrees about is false precision on screen.
                     # The CELL's own text, not the parsed number, so a two-hemisphere mean
                     # says so where the number alone would read like a measurement.
                     'albedo_src': 'geom %s, q=%.3f' % (cell, q)}
        if verbose:
            print('  sat conv  %-9s albedo %6.3f  = geometric %4.2f x q %.3f  (%s)'
                  % (name, a, p_geo, q, page))
    return out

def build_moon(m, parent_body=None, home_body=None, sat_sheets=None):
    """Turn a master-moon record into a game body (mass estimated if absent)."""
    name = m['name']
    radius_m = m['radius_m']
    mass_kg = m['mass_kg'] if m['measured'] else estimate_mass(radius_m)
    # Same policy as parse_planet: positive rate, sense in the inclination.
    assert not m['period_s'] or m['period_s'] > 0.0, name + ': orbital period must be positive'
    w = TWO_PI / m['period_s'] if m['period_s'] else 0.0
    # orientation (raan/omega) isn't in the fact sheets; epoch anomaly
    # spread stashed by main()'s add() (the Pluto/Charon lock reads it too)
    nu0 = m['nu0']
    # SOIs are derived by the loader (soi_law hill + the nesting lift, which
    # is what makes the tiniest moons landable).
    # #144: every regular moon is tidally locked (the fact sheets' rotation
    # column says "Synchronous"), so spin at the mean motion with the near
    # side facing the parent at t=0: the longitude-0 point must sit at the
    # ANTIPODE of the parent's position longitude (arg_peri + nu0 = nu0
    # here), hence -pi. A uniform spin on an eccentric Kepler rail
    # reproduces the first-order physical libration (~+-2e rad) for free.
    # tilt_azimuth stays omitted: node-locked is exactly the Cassini state
    # these moons sit in. The e < 0.1 gate keeps captured irregulars (Phoebe,
    # Nereid, the retrograde swarms) unspun as before; Hyperion (e=0.12) is
    # left unspun on purpose -- its rotation is chaotic, not synchronous. The
    # Moon's 1.5 deg equator-to-orbit tilt is omitted (the fact sheet's
    # 6.7 deg is referred to the ecliptic, not our rail; #143 covers
    # real tilts once the moon rows carry them).
    sync = m['period_s'] and m['e'] is not None and m['e'] < 0.1
    data = dict(radius_m=radius_m, mass_kg=mass_kg, w=w,
                e=m['e'], omega=0.0, i=m['i'], raan=0.0, nu0=nu0,
                incl_ref=m.get('incl_ref', 'orbit'),
                rot_s=m['period_s'] if sync else None,
                sync_phase0=nu0 - math.pi if sync else None)
    c1, c2 = MOON_COLORS.get(name, DEFAULT_COLOR)
    surface = rock(radius_m, c1, c2)
    if name in MOON_ATMOS:
        surface['atmosphere'] = MOON_ATMOS[name]
    # #737: the satellites whose pages state a BOND albedo (SOLAR_SAT_ALBEDOS: the Moon,
    # Charon). #749: the rest get a CONVERTED one, from the geometric albedo their row does
    # state times the phase factor this corpus implies for itself (SOLAR_SAT_GEOMETRIC,
    # corpus_phase_integral) -- and the conversion says so in the file, because a converted
    # figure and a reading look identical as a bare number.
    a = (sat_sheets or {}).get(name)
    if a is not None:
        surface['albedo'] = a['albedo']
        if a.get('albedo_src'):
            surface['albedo_src'] = a['albedo_src']
    return make_body(name, 'moon', m['parent'], data, seed=1000 + m['idx'],
                     surface=surface, parent_body=parent_body,
                     home_body=home_body)

# =====================================================================================
# ASTEROIDS: a second corpus, with the holes in it named rather than papered over
# =====================================================================================
"""utils/rss/asteroids/ (fetch.py caches JPL SBDB + LCDB V4.0; parse.py derives
asteroids.json offline) brings 419 minor planets. They ride the machinery this file already
has: heliocentric Kepler elements at an epoch, which make_body spells into inertial{}, and
the same surface laws the moons get.

THE CENSUS IS PRINTED, NOT QUOTED. This comment used to carry its own copy of the counts,
and it drifted ~160 rows behind the corpus without anything noticing. load_asteroids prints
the same facts under --check instead -- `asteroids: %d rows`, `tiers core %d measured %d
named %d full %d`, `rails stated ... INVENTED ...`, `radius stated ... INVENTED ...` with
the per-kind breakdown, `mass stated ... INVENTED by spectral class` -- so what is quoted
here is only the SHAPE of the corpus, which is what the tiering law turns on:

    almost every row carries a rail (a, e, i, node, peri, M at an epoch); the handful
        that do not get a cohort-invented one, named as invented
    sizes come in three qualities -- MEASURED (radar/occultation diameter, a triaxial
        ellipsoid), derived (H and an albedo), and stated outright
    a slice of rows state NOTHING about size -- discovery-page rows
    a measured mass exists for only a couple of dozen bodies; the rest of the masses are
        the spectral-class density rule, which is an invention and prints as one
    names, numbers and spectral classes cover a minority each

So "the measured ones" cannot be a tier of 68 the way it is for the moons: the corpus has
never held 68 measured asteroids. The four asteroid groups therefore NEST the way the four
moon variants already nest (core subset measured subset named subset full -- checked at the
bottom of load_asteroids, because four files that are not a ladder are four different cut
lists), and each is named for what it actually selects. The bottom rung is CURATED rather
than a size threshold -- see the LADDER_CATEGORIES comment below for why a "100 km across"
floor stopped being a floor at 419 rows. As the corpus stands that ladder is 54 / 208 /
297 / 419 minor planets, which puts the four files at 85 / 276 / 476 / 659 bodies: roughly
doubling each step, which is the size the tiers were picked for. solar_system.json gets the
rocks a player has heard of; _full gets every one the corpus can place.

Invention follows invent_missing_albedos(): a named rule, never a per-body hand-nudged
number, an assert that it cannot overwrite a stated row, and one printed line per rule.
"""

ASTEROIDS_JSON = os.path.join(HERE, 'rss', 'asteroids', 'asteroids.json')

# The catalogue's own epoch: the root's epoch_utc, as a Julian date. SBDB states each
# orbit at its OWN epoch (2008 for one row, 2024 for another), so the mean anomaly has to
# be rolled to t=0 before it means anything here. TDB-vs-UTC (64 s at this epoch) is a
# 5e-4 of a degree on the fastest orbit in the set: not worth a conversion.
CATALOGUE_EPOCH_JD = 2451544.5

# kg/m^3. Measured values for the classes the corpus carries (the sources are the ones the
# data pass cites: radar+mass pairs for the big ones, family means for the rest). These are
# INVENTED per body -- the row states a class, not a density -- so a metal rock and a coal
# one stop sharing one number, which is what #764 is about.
ASTEROID_DENSITY = {
    'C': 1900.0, 'B': 2100.0, 'G': 2200.0, 'Cg': 2200.0, 'Cb': 2200.0, 'Ch': 2400.0,
    'D': 2200.0, 'T': 2000.0, 'P': 2000.0, 'F': 2000.0,
    'S': 2700.0, 'Q': 2700.0, 'R': 2700.0, 'K': 2700.0, 'L': 3000.0,
    'X': 2700.0, 'Xe': 2700.0, 'M': 5300.0, 'V': 3400.0, 'A': 3200.0,
}
# Geometric albedos by class, same status (INVENTED where the row states no pV).
ASTEROID_GEOMETRIC = {
    'C': 0.05, 'B': 0.10, 'G': 0.10, 'Cg': 0.10, 'Cb': 0.10, 'Ch': 0.09,
    'D': 0.04, 'T': 0.06, 'P': 0.06, 'F': 0.05,
    'S': 0.20, 'Q': 0.22, 'R': 0.22, 'K': 0.20, 'L': 0.20,
    'X': 0.12, 'Xe': 0.12, 'M': 0.15, 'V': 0.42, 'A': 0.14,
}
ASTEROID_CLASS_DEFAULT = 'C'   # the most common class, and the flattest guess

# Surface palette by class: carbonaceous dark and neutral, primitive slightly redder,
# silicaceous brighter and redder still, Vesta's basalt pale, metals grey.
ASTEROID_COLORS = {
    'C': ([0.28, 0.27, 0.26], [0.42, 0.41, 0.40]),
    'B': ([0.30, 0.30, 0.31], [0.46, 0.46, 0.47]),
    'G': ([0.30, 0.29, 0.28], [0.45, 0.44, 0.43]),
    'D': ([0.24, 0.19, 0.15], [0.38, 0.31, 0.25]),
    'T': ([0.26, 0.21, 0.17], [0.40, 0.33, 0.27]),
    'P': ([0.25, 0.22, 0.19], [0.39, 0.35, 0.30]),
    'F': ([0.26, 0.25, 0.24], [0.40, 0.39, 0.38]),
    'S': ([0.42, 0.36, 0.28], [0.62, 0.54, 0.43]),
    'Q': ([0.44, 0.38, 0.30], [0.64, 0.56, 0.45]),
    'R': ([0.46, 0.40, 0.33], [0.66, 0.58, 0.48]),
    'V': ([0.55, 0.52, 0.48], [0.76, 0.73, 0.69]),
    'X': ([0.40, 0.39, 0.40], [0.58, 0.57, 0.58]),
    'M': ([0.45, 0.44, 0.46], [0.66, 0.65, 0.67]),
    'A': ([0.48, 0.40, 0.32], [0.68, 0.58, 0.48]),
}

# Two rows are MOONS of asteroids, and the pages state the moon but never its orbit. The
# parent is spelled out here rather than parsed out of the name: "243 Ida I Dactyl" is
# Dactyl of 243 Ida, and the data pass keeps the whole phrase as the display name.
ASTEROID_MOON_PARENTS = {'243 Ida I Dactyl': 'Ida',
                         '152830 Dinkinesh I Selam': 'Dinkinesh'}

# Which size kinds count as the row MEASURING its own body: radar or occultation chords, a
# size-page radius stated with its own uncertainty, and an effective diameter whose LCDB
# source code says it was measured (T thermal, W WISE, K AKARI, S SIMPS, L from a details
# row). Anything back-derived from H and an assumed albedo -- LCDB DiamSource C/D, the TNO
# list's D column, the H arithmetic below -- is already a guess, so a body that only has one
# is not in the 'measured mass' tier.
MEASURED_SIZE_KINDS = ('triaxial mean', 'two-axis mean', 'diam_eff_km', 'size page radius',
                       'lcdb diam (measured)')
LCDB_MEASURED_DIAM = ('L', 'S', 'T', 'W', 'K')


def true_anomaly_kepler(M, e):
    """[rad] Kepler's equation done in two steps: solve M = E - e sin E for the eccentric
    anomaly, then get the true anomaly from E.

    WHY A SECOND SOLVER. The one above (true_anomaly) iterates M = nu - e sin nu, which is
    Kepler's equation with the TRUE anomaly standing in for the eccentric one, so its answer
    is a pseudo-E rather than a true anomaly. Measured over the nine planets (e <= 0.249),
    that puts the shipped epoch phase off the fact sheet's own mean longitude by up to
    5.5 deg: Pluto +5.53, Saturn -2.23, Mars +2.02, Mercury +1.13, Earth -0.05. Small, and
    consistent enough that nothing in the game notices -- but not the rounding error a reader
    would assume from the name. The asteroid rows are the ones that cannot afford it: their M
    comes from JPL and their e reaches 0.996, where the one-step iteration is not a solver at
    all (fp -> 1 - e -> 0, and it diverges by radians). So the asteroids use this one and the
    planets do not, which is a trap rather than a decision -- nothing at the call site says
    which is safe. Filed as #784 to fold the two together and re-pin the phases that move."""
    M = M % TWO_PI
    if M > math.pi:
        M -= TWO_PI
    E = M + e * math.sin(M)
    for _ in range(60):
        f = E - e * math.sin(E) - M
        fp = 1.0 - e * math.cos(E)
        if fp <= 1e-12:
            break
        dE = f / fp
        E -= dE
        if abs(dE) < 1e-13:
            break
    # Half-angle form: stable as e -> 1, where tan(nu/2) blows up.
    return 2.0 * math.atan2(math.sqrt(1.0 + e) * math.sin(E / 2.0),
                            math.sqrt(1.0 - e) * math.cos(E / 2.0))


def asteroid_num(row):
    """float or None, tolerating the corpus's rounded and quoted cells."""
    v = row.get('value') if isinstance(row, dict) else row
    if isinstance(v, (int, float)):
        return float(v)
    if isinstance(v, str):
        m = re.search(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', v)
        return float(m.group(0)) if m else None
    return None


def asteroid_radius_m(row, pv=None):
    """[m] The stated size, in the corpus's own order of confidence, or None.

    1. a triaxial ellipsoid -- radar or occultation chords, the only shape this corpus
       measures rather than computes;
    2. the size page's radius, which comes with its own uncertainty;
    3. an effective diameter: the wiki's (parse.py derives it from the axes the page states),
       then LCDB's adopted one -- WITH its source code, because LCDB says whether it measured
       the rock or divided H by an assumed albedo;
    4. the diameter column of the TNO list, which most rows get the same way (their own
       Remarks cell names the assumed albedo);
    5. H with an albedo fitted from the corpus's own (size, H) pairs -- arithmetic, but the
       body's OWN brightness, which is what beats a cohort median by orders of magnitude.
       Under the cohort rule this replaces, 2004 LG came out 662x too big and Chiron 38x too
       small, and 122 rows had nothing better.

    This engine draws spheres and #763 is the ticket for triaxial bodies, so every kind
    returns a mean radius."""
    dim = row.get('dimensions_km')
    if isinstance(dim, dict) and dim.get('value'):
        vals = [float(x) for x in re.findall(r'[-+]?\d*\.?\d+', str(dim['value']))
                if float(x) > 0.0]
        if vals:
            # Geometric mean of the axes the page states, which is the volume-equivalent
            # radius for three and the best mean-radius estimate for the rows that state
            # only a/b (Antiope, Kleopatra, Hektor...: the third axis is unresolved). The
            # volume-equivalent radius is what mass and surface gravity both want, and the
            # parse already labels the three-axis cases DERIVED.
            mean = math.exp(sum(math.log(v) for v in vals) / len(vals))
            return mean * 500.0, ('triaxial mean' if len(vals) >= 3 else 'two-axis mean')
    sp = row.get('size_page')
    if isinstance(sp, dict) and sp.get('radius_m'):
        return float(sp['radius_m']), 'size page radius'
    for field in ('diam_eff_km', 'diameter_km'):
        cell = row.get(field)
        v = asteroid_num(cell)
        if v:
            return v * 500.0, field
    lc = row.get('lcdb') or {}
    if lc.get('diam_eff_km'):
        kind = ('lcdb diam (measured)' if lc.get('diam_src') in LCDB_MEASURED_DIAM
                else 'lcdb diam (H and an assumed albedo, DiamSource %s)'
                     % lc.get('diam_src', '?'))
        return float(lc['diam_eff_km']) * 500.0, kind
    tno = row.get('tno') or {}
    if tno.get('diam_km'):
        return float(tno['diam_km']) * 500.0, 'TNO list diameter'
    h = asteroid_h_mag(row)
    if h is not None and pv:
        d_km = 1329.0 / math.sqrt(pv) * 10.0 ** (-h / 5.0)
        if d_km > 0.0:
            return d_km * 500.0, 'H x fitted albedo'
    return None, None


def asteroid_h_mag(row):
    """[mag] The body's absolute magnitude from whichever source states it: the wiki cell,
    LCDB's adopted value, or MPCORB's. 341 of the 419 rows have one, and it is the single
    number that sizes the rest."""
    for v in ((row.get('h_mag') or {}).get('value'),
              (row.get('lcdb') or {}).get('h_mag'),
              (row.get('mporb') or {}).get('H')):
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                continue
    return None


def asteroid_orbit_class(row):
    """MPCORB's dynamical class ('MBA', 'Jupiter Trojan', 'Distant Object'...) -- the only
    classification 170 of these rows carry. It is an ORBIT class, not a composition class,
    which is exactly why it is a fallback and not a replacement for a spectral type."""
    return (row.get('mporb') or {}).get('Orbit_type')


def asteroid_class(row):
    """[str] The leading spectral class the row states ('C' out of 'C/B'), or the corpus
    default. Both the colour palette and the density table key on this."""
    cell = row.get('spectral_type')
    raw = cell.get('value') if isinstance(cell, dict) else cell
    cls = re.split(r'[/\s]', str(raw or ''))[0]
    return cls if cls in ASTEROID_COLORS else ASTEROID_CLASS_DEFAULT


def row_orbit(row):
    """[elements] The osculating elements to fly this row on: the page's SBDB block when it
    quoted one, else the MPCORB block parse.py folded in -- whose keys were normalised to
    SBDB's names in utils/rss/asteroids/mporb.py:ELEMENTS, so this reads one shape.

    SBDB first even where MPCORB also has an orbit: 245 of the 255 rows that have both agree
    on the epoch outright, and where they disagree MPCORB is the weaker solution (its U is
    worse on 2016 DF2 and 2005 VX3, the two rows whose orbit is genuinely unsettled). For the
    160 rows the wiki list never heard of, MPCORB is the only rail there is."""
    o = row.get('sbdb')
    if o and o.get('a_au') is not None and o.get('e') is not None:
        return o
    el = (row.get('mporb') or {}).get('elements')
    if el and el.get('a_au') is not None and el.get('e') is not None:
        return el
    return None


# The size page's Type cell leads with 'dwarf planet' for the bodies it claims are round
# under their own gravity: Ceres, Eris, Haumea, Makemake, Gonggong, Quaoar, Orcus, Sedna.
# Anchored at the START because the cell is scraped HTML and some rows carry citation
# garbage -- Aya's contains a paper title, "Exploration Strategy for the Ice Dwarf Planets".
# The IAU's five are listed too so a dwarf that reaches us only through MPCORB (no size-page
# row) is still typed as what it is rather than as a large rock.
DWARF_PLANET_NOTE = re.compile(r'^\s*dwarf planet\b', re.I)
IAU_DWARF_PLANETS = ('Ceres', 'Eris', 'Haumea', 'Makemake')


def asteroid_is_dwarf(row, name):
    """True when the corpus calls this body a dwarf planet.

    It changes one word in the game -- the dossier, and the loader accepts it as a body type.
    That is the whole reason it exists: a dossier that calls Eris an asteroid while calling
    Pluto a planet is a lie the file can see. Pluto is not here and must never be; the
    planetary file authors it, and the catalogue keys bodies by name."""
    note = (row.get('size_page') or {}).get('type_note') or ''
    return bool(DWARF_PLANET_NOTE.match(note)) or name in IAU_DWARF_PLANETS


def geometric_albedo_fits(raw):
    """({spectral class: pV}, {MPCORB dynamical class: pV}) fitted from the corpus's own
    pairs of (stated size, H), by the inverse of the standard relation:
        pV = (1329 / D_km)^2 * 10^(-2H/5)

    Why fit rather than type: the ASTEROID_GEOMETRIC table below is a handbook's rounded
    guess, and the rows that need it are precisely the ones no handbook measured. A median
    over the rows that DO state both numbers is the corpus describing itself -- the same move
    as cohort_median and corpus_phase_integral. Classes with one pair are left out, so a
    single oddball cannot set a family's albedo, and a fitted pV outside 0.01-1.2 means the
    size and the H are not describing the same body, so it is dropped rather than averaged.
    """
    import collections
    pairs = collections.defaultdict(list)
    rejected = 0
    for row in raw:
        h = asteroid_h_mag(row)
        r, kind = asteroid_radius_m(row)          # no pv -> no H-derived sizing
        if h is None or not r or kind == 'H x fitted albedo':
            continue
        d_km = 2.0 * r / 1000.0
        if d_km <= 0.0:
            continue
        p = (1329.0 / d_km) ** 2 * 10.0 ** (-2.0 * h / 5.0)
        if not 0.01 <= p <= 1.2:
            rejected += 1
            continue
        cell = row.get('spectral_type')
        stated = cell.get('value') if isinstance(cell, dict) else cell
        if stated:
            # keyed the same way the lookup below will ask -- but only when the row states
            # one, or the 250 rows with no class would set the C-class albedo by vote.
            pairs[('cls', asteroid_class(row))].append(p)
        oc = asteroid_orbit_class(row)
        if oc:
            pairs[('orb', oc)].append(p)
    fits = {}
    for key, vals in pairs.items():
        if len(vals) >= 2:
            vals = sorted(vals)
            fits[key] = vals[len(vals) // 2]
    return fits, rejected


def invent_asteroid_radius_m(name, sections, cohort_median, global_median):
    """[m] The 120 rows the corpus gives a rail for and NOTHING else.

    Not a magnitude derivation -- there is no magnitude row either, so there is nothing on
    the left-hand side of that formula. The rule is the cohort: a rock is the size of the
    other rocks in the section the page put it in (retrograde, inclined, brightest...),
    which is the corpus describing its own population rather than me picking a number. If
    none of its sections has a sized member, the whole set's median stands in."""
    med = cohort_median.get(next((s for s in sections if s in cohort_median), ''), None)
    return (med if med is not None else global_median), 'INVENTED cohort median'


def invent_asteroid_density(cls):
    """[kg/m^3] The row states a spectral class, not a density, so the class is the rule
    and the number is ours. Only the mass estimate depends on it -- but that is what a
    player's descent profile depends on, so one flat 2000 for a metal rock and a coal one
    (#764) is the thing this replaces."""
    return ASTEROID_DENSITY.get(cls, ASTEROID_DENSITY[ASTEROID_CLASS_DEFAULT])


def invent_asteroid_albedo(cls, q):
    """[Bond] Same rule as the satellites (#749): a class-typical GEOMETRIC albedo times
    the phase integral this corpus implies for itself. INVENTED, and says so."""
    return ASTEROID_GEOMETRIC.get(cls, ASTEROID_GEOMETRIC[ASTEROID_CLASS_DEFAULT]) * q


def asteroid_surface(a):
    """The surface block: palette and albedo by class, relief scaled to the body.

    This deliberately does NOT reuse rock(), whose 50 m floor is right for a 300 m moon and
    absurd on this set's smallest row (a 3-4 m object): 50 m of relief on a 2 m radius is
    25 of the radius. Fraction-of-radius, floored at 1 m. #763 is the ticket that replaces
    "one heuristic for every small body" with something that knows about craters."""
    r = a['radius_m']
    amp = min(0.25 * r, max(min(500.0, 0.15 * r), min(1.0, 0.5 * r)))
    c1, c2 = ASTEROID_COLORS.get(a['cls'], ASTEROID_COLORS[ASTEROID_CLASS_DEFAULT])
    surface = {'amplitude': amp, 'palette': [[0.0, c1], [1.0, c2]]}
    surface['albedo'] = a['albedo']
    surface['albedo_src'] = a['albedo_src']
    return surface


def load_asteroids(q, existing_names, verbose=True):
    """[rows] The 419 minor-planet rows turned into what build_asteroid wants, tiered and
    inventoried. `q` is the corpus's own phase integral, shared with the satellites so one
    number converts geometric to Bond albedo across the whole file."""
    with open(ASTEROIDS_JSON) as f:
        raw = json.load(f)['bodies']
    assert raw, 'asteroids.json is empty'
    # Pluto is a PLANET in this file -- WGCCRE Table 3, its own pole rows, its moon system.
    # The corpus's own list carries a row for it too, and the collision pass below would
    # "solve" that by renaming it (134340) Pluto, which is not a second world but the same
    # one spelled twice. parse.py drops the row; this is the invariant that keeps it dropped.
    assert not any(r['name'] == 'Pluto' for r in raw), \
        'Pluto must not be in the minor-planet corpus: the planetary file already authors it'

    # The albedo fits come before anything that sizes a row: a row that states no size is
    # sized from its absolute magnitude, and that arithmetic needs an albedo. Fitting pV from
    # the rows that state BOTH a size and an H is what makes the arithmetic the corpus's own
    # rather than a handbook's; the medians come out along composition (carbonaceous ~0.05,
    # stony ~0.2, icy ~0.7) and along distance.
    fits, fit_rejected = geometric_albedo_fits(raw)

    def pv_guess(row):
        """[geometric albedo] The pV to convert this row's H into a radius: the corpus's own
        median for its stated spectral class, else for its MPCORB dynamical class, else the
        typed handbook table. It is a guess about a family, never about this body, which is
        why a row that states a size gets its OWN pV back out of (size, H) further down."""
        cell = row.get('spectral_type')
        stated = cell.get('value') if isinstance(cell, dict) else cell
        return ((fits.get(('cls', asteroid_class(row))) if stated else None)
                or fits.get(('orb', asteroid_orbit_class(row)))
                or ASTEROID_GEOMETRIC.get(asteroid_class(row),
                                          ASTEROID_GEOMETRIC[ASTEROID_CLASS_DEFAULT]))

    # Cohort medians FIRST: the invention for the size-less rows reads them, and they have
    # to be measured from the rows that DO state a size, not typed. Keyed by ROW INDEX, not
    # by name: the collision pass below renames (52) Europa to '(52) Europa', and a
    # name-keyed table would lose the renamed rows' stated sizes and invent cohort medians
    # for bodies NASA measured with radar.
    sized = {}
    for idx, row in enumerate(raw):
        r, kind = asteroid_radius_m(row, pv_guess(row))
        if r:
            sized[idx] = (r, kind)
    by_section = {}
    for idx, row in enumerate(raw):
        hit = sized.get(idx)
        if not hit:
            continue
        for sec in (row.get('sections') or []):
            by_section.setdefault(sec, []).append(hit[0])
    cohort_median = {s: sorted(v)[len(v) // 2] for s, v in sorted(by_section.items())}
    all_r = sorted(v for v, _ in sized.values())
    global_median = all_r[len(all_r) // 2] if all_r else 0.0

    rows, invented_radius, invented_mass, invented_albedo = [], 0, 0, 0
    rail_tally = {}
    derived_albedos = 0
    for idx, row in enumerate(raw):
        name = row['name']
        number = row.get('number')
        parent = ASTEROID_MOON_PARENTS.get(name, 'Sun')
        if name in existing_names:
            # Real collision, not a hypothetical: asteroid (52) Europa and (9) Metis share
            # a name with Jupiter's moons, and the catalogue keys bodies by name (so do
            # saves, src/save.h). The number the data already carries disambiguates it.
            assert number, '%s: collides with a body already in the file and has no ' \
                           'number to disambiguate it' % name
            name = '(%s) %s' % (number, name)
            assert name not in existing_names, '%s: still collides' % name

        r, kind = sized.get(idx, (None, None))
        if r is None:
            r, kind = invent_asteroid_radius_m(name, row.get('sections') or [],
                                               cohort_median, global_median)
            invented_radius += 1
        assert r and r > 0.0, '%s: no radius, stated or invented' % name

        cls_raw = row.get('spectral_type')
        cls = (cls_raw.get('value') if isinstance(cls_raw, dict) else cls_raw) or ''
        cls = re.split(r'[/\s]', str(cls))[0]        # 'C/B' -> 'C': the leading class
        cls = cls if cls in ASTEROID_COLORS else ASTEROID_CLASS_DEFAULT

        mass = asteroid_num(row.get('mass_kg'))
        if mass is None:
            mass = invent_asteroid_density(cls) * (4.0 / 3.0) * math.pi * r ** 3
            invented_mass += 1

        # The albedo. Stated by a handful of rows; for the rest a body with BOTH a stated
        # size and a stated H still answers for itself, because that pair implies a pV
        # through the same formula that sized it (pV = (1329/D)^2 10^(-2H/5)). That is not a
        # guess about a family -- it is this body's own brightness divided by its own disk --
        # so it is labelled DERIVED, not INVENTED, and it keeps Eris' frost and a C-type coal
        # apart instead of flattening both to a class table.
        p_geo = asteroid_num(row.get('albedo_pV'))
        p_src = 'stated'
        if p_geo is None and kind not in ('INVENTED cohort median', 'H x fitted albedo'):
            h = asteroid_h_mag(row)
            if h is not None:
                p_try = (1329.0 / (2.0 * r / 1000.0)) ** 2 * 10.0 ** (-2.0 * h / 5.0)
                # 0.02 floor: the darkest known surfaces (D-type, ~0.03) sit just above it
                # and an out-of-band pV means the size and the magnitude are describing
                # different objects -- a mis-join, which must fall through to the class rule.
                if 0.02 <= p_try <= 1.2:
                    p_geo, p_src = p_try, 'DERIVED from %s + H %.2f' % (kind, h)
                    derived_albedos += 1
        if p_geo is not None:
            albedo = p_geo * q
            albedo_src = '%s: pV %.3f x q %.3f' % (p_src, p_geo, q)
        else:
            albedo = invent_asteroid_albedo(cls, q)
            albedo_src = 'INVENTED: %s-class geometric x q %.3f' % (cls, q)
            invented_albedo += 1
        assert 0.0 <= albedo < 1.0, '%s: albedo %g is not a Bond albedo' % (name, albedo)

        rot_h = asteroid_num(row.get('rotation_period_h')) or \
                asteroid_num(row.get('wiki_rotation_h'))

        o = row_orbit(row)
        if o is not None:
            key = 'SBDB' if o is row.get('sbdb') else 'MPCORB'
            rail_tally[key] = rail_tally.get(key, 0) + 1
        rail = None
        if o:
            e = float(o['e'])
            assert 0.0 <= e < 1.0, '%s: e %g is not an elliptical orbit' % (name, e)
            n_deg_d = asteroid_num(o.get('mean_motion_deg_d'))
            if not n_deg_d:
                per_d = asteroid_num(o.get('period_d'))
                n_deg_d = (360.0 / per_d) if per_d else None
            assert n_deg_d, '%s: no mean motion and no period' % name
            epoch = asteroid_num(o.get('epoch_jd')) or CATALOGUE_EPOCH_JD
            M0 = ((asteroid_num(o.get('mean_anom_deg')) or 0.0)
                  + n_deg_d * (CATALOGUE_EPOCH_JD - epoch)) * D2R
            rail = dict(w=n_deg_d * D2R / DAY, e=e,
                        i=(asteroid_num(o.get('i_deg')) or 0.0) * D2R,
                        raan=(asteroid_num(o.get('node_deg')) or 0.0) * D2R,
                        omega=(asteroid_num(o.get('argperi_deg')) or 0.0) * D2R,
                        nu0=true_anomaly_kepler(M0, e))

        rows.append(dict(name=name, raw_name=row['name'], display=name, parent=parent,
                         radius_m=r, radius_src=kind,
                         mass_kg=mass, cls=cls, albedo=albedo, albedo_src=albedo_src,
                         rot_s=(rot_h * HR if rot_h else None), rail=rail,
                         sections=row.get('sections') or [], idx=idx,
                         dwarf=asteroid_is_dwarf(row, row['name']),
                         measured=bool(kind in MEASURED_SIZE_KINDS
                                       or asteroid_num(row.get('mass_kg')) is not None),
                         named=bool(row.get('number')
                                    or (name and not name[0].isdigit()))))

    # The bottom rung of the ladder is CURATED, not a size threshold. With 419 rows the old
    # "visited or 100 km across" rule let every mined 100+ km Kuiper object walk in -- 123 of
    # 419, so the smallest file was 30% of the biggest and the ladder had no bottom rung left
    # to climb. What makes a body stand in for its category is that the corpus already sorts
    # them: `sections` ARE the categories, and the biggest member of a category is the one a
    # player would name if asked for it. So the smallest variant is: every body a spacecraft
    # has flown to, every dwarf planet, and the CATEGORY_DEPTH biggest members of each
    # category. The deeper rungs stay what they were -- measured size or mass, then anything
    # with a number or a name -- so the ladder reads ~50 / 210 / 291 / 419.
    #
    # LADDER_CATEGORIES is the sampling list, and it deliberately leaves out the sections that
    # say where a row came from rather than what it is ('size_page') and the axis buckets that
    # re-sort bodies already filed under another category ('coorbital_close' -- its members are
    # in 'coorbital' too). Ranking is per category by radius, so a category's rank-1 body is
    # inside every deeper cut and the tiers nest by construction.
    CATEGORY_DEPTH = 2
    LADDER_CATEGORIES = {'brightest', 'largest', 'massive', 'landmark', 'inclined',
                         'retrograde', 'fast_rot', 'slow_rot', 'comets', 'bright',
                         'sungrazer', 'coorbital', 'high_i', 'hungaria',
                         'phocaea', 'trojan', 'kuiper'}
    # Two sections are hand-picked lists rather than populations to sample: the flight-proven
    # destinations, and the famous co-orbitals (2020 CD3 and 2024 PT5 are in the corpus
    # BECAUSE we chose them, so ranking that category by size would pick our deliberately
    # added minimoons back OUT of the smallest file by the opposite criterion).
    LADDER_ALWAYS = {'visited', 'coorbital_famed'}
    rank = {}
    for sec in sorted(LADDER_CATEGORIES):
        pool = sorted((a for a in rows if sec in a['sections']),
                      key=lambda a: (-a['radius_m'], a['name']))
        for i, a in enumerate(pool, start=1):
            rank[a['name']] = min(rank.get(a['name'], 9999), i)
    core = [a for a in rows
            if a['dwarf'] or (LADDER_ALWAYS & set(a['sections']))
            or rank.get(a['name'], 9999) <= CATEGORY_DEPTH]
    core_names = {a['name'] for a in core}
    meas = core_names | {a['name'] for a in rows if a['measured']}
    named = meas | {a['name'] for a in rows if a['named']}
    for a in rows:
        a['core'], a['measured'], a['named'] = (a['name'] in core_names,
                                                a['name'] in meas, a['name'] in named)
    # The invariant the four files depend on. A moon whose parent this variant does not
    # emit is a load-time throw, so the parents of the two asteroid moons must be core.
    assert core_names <= meas <= named <= {a['name'] for a in rows}, \
        'asteroid tiers must nest: core subset measured subset named subset full'
    for a in rows:
        if a['parent'] != 'Sun':
            parent = next((p for p in rows if p['name'] == a['parent']), None)
            assert parent and parent['core'], \
                '%s: its parent %s is not in the core tier, so a variant could emit the ' \
                'moon without the world it orbits' % (a['name'], a['parent'])
            # No rail row at all for these two: a circular orbit at 6 parent radii, which
            # is where Dactyl actually sits (Lucy measured ~6 Ida radii), and the PERIOD
            # is not invented -- Kepler III out of that distance and the parent's mass.
            if a['rail'] is None:
                a['rail'] = invented_moon_rail(a['name'], parent)

    # ... and the rows that are heliocentric but state no orbit anywhere (2010 EQ169's own
    # page: "orbit is not well-known"). Cohort median again, so the rule that sizes these
    # bodies places them too.
    rail_by_section = {}
    for a in rows:
        if a['rail'] is None or (a['rail'] or {}).get('invented') or a['parent'] != 'Sun':
            continue
        o = next((r for r in raw if r['name'] == a['raw_name']), None)
        el = row_orbit(o) if o else None
        if not el:
            continue
        for sec in a['sections']:
            rail_by_section.setdefault(sec, []).append(
                (float(el['a_au']), float(el['e']), float(el.get('i_deg') or 0.0)))
    for a in rows:
        if a['rail'] is not None or a['parent'] != 'Sun':
            continue
        a['rail'] = invented_heliocentric_rail(a['name'], a['sections'], rail_by_section)

    assert all(a['rail'] for a in rows), \
        'every asteroid row needs a rail, stated or invented: %s' % \
        [a['name'] for a in rows if not a['rail']]

    if verbose:
        print('asteroids: %d rows from %s' % (len(rows), os.path.relpath(ASTEROIDS_JSON, ROOT)))
        print('  tiers  core %d  measured %d  named %d  full %d'
              % (len(core_names), len(meas), len(named), len(rows)))
        # Which body stands in for which category is a judgement, so it gets printed rather
        # than buried in a set comprehension. 4 per line, because 19 categories is one long
        # line otherwise.
        reps = []
        for sec in sorted(LADDER_CATEGORIES):
            pool = sorted((a for a in rows if sec in a['sections']),
                          key=lambda a: (-a['radius_m'], a['name']))[:CATEGORY_DEPTH]
            if pool:
                reps.append('%s: %s' % (sec, ' '.join(a['name'] for a in pool)))
        for i in range(0, len(reps), 4):
            print('  reps  %s' % '  |  '.join(reps[i:i + 4]))
        print('  type   dwarf planet %d (%s)'
              % (sum(1 for a in rows if a['dwarf']),
                 ' '.join(a['name'] for a in rows if a['dwarf'])))
        print('  rails  stated %d  INVENTED %d (the two asteroid moons, and any row whose '
              'page states no orbit)'
              % (sum(1 for a in rows if not (a['rail'] or {}).get('invented')),
                 sum(1 for a in rows if (a['rail'] or {}).get('invented'))))
        # Which source actually places a body matters now that two do: SBDB for the rows the
        # wiki page quoted, MPCORB for the 160 the mining rules found.
        print('         %s' % '  '.join('%s %d' % kv for kv in sorted(rail_tally.items())))
        kinds = {}
        for a in rows:
            kinds[a['radius_src']] = kinds.get(a['radius_src'], 0) + 1
        print('  radius stated %d  INVENTED %d  [%s]'
              % (len(sized), invented_radius,
                 ', '.join('%s %d' % kv for kv in sorted(kinds.items(),
                                                         key=lambda kv: -kv[1]))))
        print('         cohort medians: %s'
              % ', '.join('%s %s' % (s, fmt_km(m)) for s, m in cohort_median.items()))
        print('  mass   stated %d  INVENTED by spectral class (%s)'
              % (len(rows) - invented_mass,
                 ' '.join('%s %g' % (c, ASTEROID_DENSITY[c]) for c in
                          sorted(set(a['cls'] for a in rows)))))
        # pV three ways: a page states it, the body's own (size, H) implies it, or the class
        # table stands in. The middle one is new and is what most of the mined rows have.
        print('  albedo stated pV %d  DERIVED from size+H %d  INVENTED by class %d'
              % (len(rows) - invented_albedo - derived_albedos, derived_albedos,
                 invented_albedo))
        print('         pV fitted from the corpus (>= 2 rows): %d  [%s]'
              % (len(fits), ' '.join('%s:%s %.3f' % (k[0][0], k[1], v)
                                     for k, v in sorted(fits.items(), key=lambda kv: str(kv[0])))))
        print('         rejected (size and H disagree): %d' % fit_rejected)
        print('  spin   stated %d  none authored %d (the loader leaves those unspun, as '
              'the irregular moons)'
              % (sum(1 for a in rows if a['rot_s']), sum(1 for a in rows if not a['rot_s'])))
    return rows


def fmt_km(m):
    return ('%.3g km' % (m / 1000.0)) if m >= 1000 else ('%.0f m' % m)


def invented_moon_rail(name, parent_row):
    """[rail] The two asteroid moons the pages never place. See load_asteroids: the
    DISTANCE is the invention (6 parent radii, anchored on the one such orbit that has
    been measured), the mean motion follows from it under Kepler III."""
    a = 6.0 * parent_row['radius_m']
    n = math.sqrt(G * parent_row['mass_kg'] / (a ** 3))
    assert a > parent_row['radius_m'] * 1.5, '%s: invented orbit inside its parent' % name
    rail = dict(w=n, e=0.0, i=0.0, raan=0.0, omega=0.0, nu0=phase0(name, 'anomaly'))
    rail['invented'] = 'circular at 6 parent radii (%s); period from Kepler III' % fmt_km(a)
    return rail


def invented_heliocentric_rail(name, sections, rail_by_section):
    """[rail] The rows that state no orbit anywhere -- 2010 EQ169's page says so outright
    ("orbit is not well-known") and SBDB has nothing to say either.

    Cohort median again: the same rule that sizes these bodies places them, so one rule
    rather than two kinds of guess. The angles are the reproducible spread this file
    already draws where no epoch orientation is stated (#141), and the mean motion follows
    from the invented axis under Kepler III rather than being picked."""
    pool = next((rail_by_section[s] for s in sections if s in rail_by_section), None)
    assert pool, '%s: no cohort with a stated orbit to place it by' % name
    a_au = sorted(p[0] for p in pool)[len(pool) // 2]
    e    = sorted(p[1] for p in pool)[len(pool) // 2]
    i    = sorted(p[2] for p in pool)[len(pool) // 2]
    assert 0.0 < a_au and 0.0 <= e < 1.0, '%s: cohort rail makes no sense' % name
    a = a_au * AU
    rail = dict(w=math.sqrt(G * SUN_MASS / a ** 3), e=e, i=i * D2R,
                raan=phase0(name, 'node'), omega=phase0(name, 'peri'),
                nu0=phase0(name, 'anomaly'))
    rail['invented'] = 'cohort median: a %.2f AU, e %.2f, i %.1f deg' % (a_au, e, i)
    return rail


def build_asteroid(a, by, home):
    """One asteroid (or asteroid moon) as a game body: heliocentric like a planet, airless
    rock like a moon, and the same transfer/science stamps as both."""
    rail = a['rail']
    assert rail, '%s: no rail' % a['name']
    data = dict(radius_m=a['radius_m'], mass_kg=a['mass_kg'],
                w=rail['w'], e=rail['e'], omega=rail['omega'], i=rail['i'],
                raan=rail['raan'], nu0=rail['nu0'], rot_s=a['rot_s'])
    if a['parent'] != 'Sun':
        type_ = 'moon'
    else:
        # A dwarf planet draws exactly like an asteroid and is indistinguishable from one in
        # every law the engine applies. It is in the file because the DOSSIER says so: the
        # dossier already calls Pluto a planet, and calling Eris an asteroid next to it is a
        # contradiction a player can see. src/system.cpp:BodyType::DwarfPlanet.
        type_ = 'dwarf_planet' if a['dwarf'] else 'asteroid'
    return make_body(a['name'], type_, a['parent'], data, seed=2000 + a['idx'],
                     surface=asteroid_surface(a),
                     parent_body=by.get(a['parent']), home_body=home)


def _comparable(doc):
    """Copy without the derived science fields (sci_dist.py re-stamps them,
    so they must not count as source drift here)."""
    out = dict(doc)
    out['bodies'] = [
        {k: v for k, v in b.items()
         if k not in ('science_mult', 'transfer_dv')}
        for b in doc.get('bodies', [])]
    return out


def emit(base_bodies, moons, pred, out_path, label, sheets, dry=False, asteroids=None):
    # #724 rung D: NO body in this file may author an air-follow knob. The value is invented
    # (no source says how far an air follows its ground), and this file's standing is that its
    # numbers come from the fact sheets -- utils/gen_systems.py:KSP_AIR_FOLLOW is where that
    # kind of number goes. The assert keeps the argument from rotting into an accident.
    for b in base_bodies:
        at = (b.get('surface') or {}).get('atmosphere')
        assert not at or 'air_follow' not in at, (
            "%s: solar_system.json must not author air_follow (an invented number in the "
            "corpus file)" % b['name'])
    bodies = list(base_bodies)
    by = {b['name']: b for b in bodies}
    home = by['Earth']
    n = 0
    skipped = 0
    # The satellites the corpus prices in BOND (#737). Printed once, on the core variant,
    # beside the other rows this generator prints rather than in build_moon, because the
    # rule that picks them is one decision and emit() runs four times.
    sat_sheet = satellite_sheets(verbose=label.startswith('core'))
    for m in moons:
        if not pred(m):
            continue
        if not m['a'] or not m['period_s']:
            skipped += 1
            continue
        bodies.append(build_moon(m, parent_body=by[m['parent']], home_body=home,
                                 sat_sheets=sat_sheet))
        by[m['name']] = bodies[-1]
        n += 1
    # Asteroids, on the SAME predicate: their rows carry the same 'measured'/'named' flags
    # and main() folds the core tier into the same name set, so one cut list selects both
    # corpora -- which is what keeps the four files one ladder rather than two ladders that
    # happen to share a filename prefix. Heliocentric ones first: an asteroid moon needs
    # its parent already in `by`, exactly as a moon needs its planet.
    n_ast = 0
    for a in sorted(asteroids or [], key=lambda a: a['parent'] != 'Sun'):
        if not pred(a):
            continue
        if a['parent'] not in by:
            raise SystemExit("asteroid '%s' orbits '%s', which this variant does not emit"
                             % (a['name'], a['parent']))
        bodies.append(build_asteroid(a, by, home))
        by[a['name']] = bodies[-1]
        n_ast += 1
    # The moons get the same scrape-side rule quietly (#708) -- and today it compares
    # NOTHING, because no satellite parser in this file reads a gravity row (they take a
    # mass and a radius), so check_scraped_gravity skips every moon. It stays wired so
    # that the day a parser DOES scrape one the rule is already in the path; the trap it
    # guards against is the one it used to be: comparing the EMITTED g against the mass,
    # which became a tautology the moment the generator computed g itself.
    bad = check_scraped_gravity({m['name']: m for m in moons}, verbose=False)
    if bad:
        for m in bad:
            print('DRIFT   scraped gravity: ' + m)
        raise SystemExit('%d satellite-page gravity rows disagree with the same page\'s'
                         ' own mass and radius' % len(bad))
    # ... and the two satellites the corpus states rows for, weighed at their PARENT's rail
    # (#737). This runs in emit() rather than beside the planets because a moon has no rail
    # until the Kepler rows are built, and its sheet lives inside another body's page.
    # The albedo layer FIRST: the swarm pages state no albedo, and the anchor ladder's third
    # rung needs one, so this is what stands between 209 worlds and a dashed ground row. It
    # invents, it says so in the value's src string, and it runs before both the surface
    # guard and the diurnal pass so neither sees a hole that is really a decision. Printed on
    # every variant, not just the core: the core has no swarm to invent for, so a core-only
    # print would announce nothing about the 209 bodies the invention is for.
    invent_missing_albedos(bodies)
    bad = check_fact_surface(bodies, sat_sheet, by['Sun'],
                             verbose=label.startswith('core'))
    if bad:
        for m in bad:
            print('DRIFT   fact surface: ' + m)
        raise SystemExit('%d satellite insolation figure(s) disagree with the fact sheet '
                         'that states them (band %d%%)'
                         % (len(bad), round(100 * FACT_SURFACE_BANDS)))
    # The ground's swing, from the diurnal pairs the pages state (#724 rung C). Runs with the
    # moons appended like the checks above, and after the star's one authored temperature
    # exists -- an unanchored body's black-body is what a moon with no stated ground reading
    # has to swing around.
    author_diurnal(bodies, by, by['Sun'], sheets, sat_sheet,
                   verbose=label.startswith('core'))
    # ... and that every air in the file is one of the two kinds on purpose. It runs here,
    # with the moons appended, rather than beside the planets: Titan's air comes from
    # MOON_ATMOS, and a coverage guard that cannot see the whole catalogue would report the
    # one exception it exists to name as stale. Printed once, on the core variant -- emit()
    # runs per variant, and four copies of a roster row is noise in a log that already
    # prints a `wrote` line for each.
    bad = check_air_coverage(bodies, verbose=label.startswith('core'))
    if bad:
        for m in bad:
            print('DRIFT   air coverage: ' + m)
        raise SystemExit('%d air bodies are not accounted for as profiles (#625)'
                         % len(bad))
    # epoch_year 2000: the fact-sheet elements (and our derived orbital
    # phases) are J2000-referenced, so the calendar clock matches the sky.
    # epoch_utc (#202): proleptic-Gregorian civil clock. t=0 is already
    # 2000-01-01 00:00 UT (the _wgccre_epoch d=-0.5 convention).
    doc = {'home': 'Earth', 'soi_law': 'hill', 'epoch_year': 2000,
           'epoch_utc': '2000-01-01T00:00:00Z',
           # A directory of skybox_{px,nx,py,ny,pz,nz}.png (src/system.cpp).
           # The real sky for the real Solar System: Deep Star Maps 2020 at
           # gain 3.0 (see res/skybox/dsm_1024_g3_ds2/skybox_params.json).
           'skybox': 'res/skybox/dsm_1024_g3_ds2',
           # Debris belts (root "belts", src/system.cpp): named annuli around
           # the Sun, drawn on the Tracking map. Same entry shape as a body's
           # surface.rings band. The real main belt (2.1..3.3 AU) and Kuiper
           # belt (30..50 AU); the name is documentation, the array order is
           # draw order, and the map assumes they lie in the Sun's equatorial
           # plane.
           'belts': [{'name': 'asteroid belt', 'inner': AU * 2.1, 'outer': AU * 3.3},
                     {'name': 'Kuiper belt', 'inner': AU * 30.0, 'outer': AU * 50.0}],
           # Launch pads. The real sites live in ONE table --
           # res/data/earth_launch_sites.json is a "world_buildings" fragment
           # the loader merges (src/buildings.cpp) -- so a coordinate has one
           # home instead of four copies, one per solar_system variant. The
           # merged list's first VAB_LAUNCH_PAD is the game's default site, so
           # the table's order is deliberate: Baikonur.
           'world_buildings_files': ['res/data/earth_launch_sites.json'],
           # A second pad at the geographic north pole (flag POLAR_PAD),
           # where the flat-sea clamp puts the Arctic ice exactly at sea
           # level. Inline because it belongs to this system, not to the
           # real-world table -- and inline keeps it out of the default-site
           # race.
           'world_buildings': [
               {'id': 'polar_pad', 'type': 'space_port',
                'display_name': 'Polar pad', 'body': 'Earth',
                'lat_deg': 90.0, 'lon_deg': 0.0, 'height_m': 5.0,
                'heading_deg': 0.0,
                'flags': ['VAB_LAUNCH_PAD', 'POLAR_PAD']},
           ],
           'bodies': bodies}
    stamp_science_mults(doc)
    if dry:
        return doc          # --check: the caller compares, nothing is written
    with open(out_path, 'w') as f:
        json.dump(doc, f, indent=2)
        f.write('\n')
    print('wrote %-32s  %3d bodies (%3d moons, %3d asteroids)%s  [%s]' %
          (out_path, len(bodies), n, n_ast,
           ('  (%d skipped: no orbit data)' % skipped) if skipped else '', label))
    return n

def main():
    import argparse
    ap = argparse.ArgumentParser(
        description="Generate res/systems/solar_system*.json, or --check "
                    "that the committed files match this script.")
    ap.add_argument("--check", action="store_true",
                    help="verify the committed JSONs still match, without "
                         "writing anything; exit non-zero on any drift")
    args = ap.parse_args()

    # Before anything is compared: this script weighs a column with G and sizes a star
    # with sigma, and both are copies of the loader's literals. If they have parted
    # company, every number below is self-consistent and wrong (#687/#707).
    bad = check_constants_match_src(verbose=args.check)
    if bad:
        for m in bad:
            print('DRIFT   constants: ' + m)
        raise SystemExit('utils/ and src/ disagree about a physical constant')

    base_bodies, parsed, fact_sheets = build_base(verbose_air=args.check)

    moons = []
    def add(*ms):
        for m in ms:
            m['idx'] = len(moons)
            # Epoch anomaly spread (fact sheets carry no moon raan/omega);
            # stashed so main()'s Pluto/Charon lock and build_moon cannot
            # desync on the formula. Arbitrary, so the direction is a
            # convention: negative keeps the spread where the pre-#146
            # azimuth convention put it.
            m['nu0'] = (-m['idx'] * GOLDEN) % TWO_PI
            moons.append(m)

    add(dict(parse_earth_moon(), parent='Earth'))
    mm = parse_mars_moons(['Phobos', 'Deimos'])
    add(dict(mm['Phobos'], parent='Mars'), dict(mm['Deimos'], parent='Mars'))
    for m in parse_sat_table_all('joviansatfact.html'):
        add(dict(m, parent='Jupiter'))
    for m in parse_sat_table_all('saturniansatfact.html'):
        add(dict(m, parent='Saturn'))
    for m in parse_sat_table_all('uraniansatfact.html'):
        add(dict(m, parent='Uranus'))
    for m in parse_sat_table_all('neptuniansatfact.html'):
        add(dict(m, parent='Neptune'))
    for m in parse_pluto_moons_all():
        add(dict(m, parent='Pluto'))

    # #147: inclination reference per moon row. The fact sheets refer
    # REGULAR moons' inclination to the parent's EQUATOR and irregular
    # (captured) moons' to the ECLIPTIC; each orbital table lists regular
    # first and irregular after -- uraniansatfact spells the boundary out
    # ("Satellites from Francisco out referenced to ecliptic plane instead
    # of equator"), and the values agree (Io 0.04 deg vs Phoebe 174.8 deg).
    # Per parent, the cut index: '*' before the first row (Earth's Moon
    # says "Inclination to ecliptic" outright), '' after the last (Mars
    # and Pluto are all regular; Charon's row says "inclination to
    # Pluto"), else the first irregular name. Triton is irregular (157.3
    # deg to the ecliptic) even though it is big and synchronous.
    ECLIPTIC_REF_FROM = {'Earth': '*', 'Mars': '', 'Pluto': '',
                         'Jupiter': 'Themisto', 'Saturn': 'Kiviuq',
                         'Uranus': 'Francisco', 'Neptune': 'Triton'}
    by_parent = {}
    for m in moons:
        by_parent.setdefault(m['parent'], []).append(m)
    assert set(by_parent) == set(ECLIPTIC_REF_FROM), \
        'moon parents drifted from ECLIPTIC_REF_FROM: %s' % \
        sorted(set(by_parent) ^ set(ECLIPTIC_REF_FROM))
    for p, b in ECLIPTIC_REF_FROM.items():
        ms = by_parent[p]
        names = [m['name'] for m in ms]
        if b == '*':
            cut = 0
        elif b == '':
            cut = len(ms)
        else:
            assert b in names, \
                '%s: irregular boundary %r missing from the sheet' % (p, b)
            cut = names.index(b)
            # A boundary at index 0 means the table reordered (regulars
            # first is the convention the cut relies on); Triton-first
            # Neptunian tables are common in the wild.
            assert cut > 0, \
                '%s: boundary %r is first in table order (regulars no ' \
                'longer precede irregulars?)' % (p, b)
        for k, m in enumerate(ms):
            m['incl_ref'] = 'orbit' if k >= cut else 'equator'

    n_named = sum(1 for m in moons if m['named'])
    n_meas  = sum(1 for m in moons if m['measured'])
    print('master moon table: %d moons (%d named, %d with measured mass)'
          % (len(moons), n_named, n_meas))
    print()

    # #144 follow-up: Pluto and Charon are MUTUALLY tidally locked, and
    # the IAU report DEFINES Pluto's lon 0 as the mean sub-Charon meridian
    # (and Charon's as sub-Pluto). #147 put Charon on Pluto's EQUATOR, so
    # Pluto's tilt is common-mode between its figure equator and Charon's
    # rail: the lon-0 POINT can sit exactly sub-Charon, and the phase is
    # just Charon's epoch rail longitude. (Pre-#147 Charon orbited the
    # ORBITAL plane, ~54 deg off the equator, so only a meridian-plane
    # compromise existed.) The POLE stays the real Table 3 one; only the
    # PHASE is game-anchored, because our Charon epoch phase is the
    # synthetic golden-angle spread, not the real ephemeris.
    charon = next(m for m in moons if m['name'] == 'Charon')
    pluto = next(b for b in base_bodies if b['name'] == 'Pluto')
    pluto['rotating']['spin_phase0'] = charon['nu0']

    core = {'Moon', 'Phobos', 'Deimos', 'Io', 'Europa', 'Ganymede',
            'Callisto', 'Mimas', 'Enceladus', 'Tethys', 'Dione', 'Rhea',
            'Iapetus', 'Titan', 'Miranda', 'Ariel', 'Umbriel', 'Titania',
            'Oberon', 'Triton', 'Charon'}
    # The second corpus. Its tiers are built to nest with the moon tiers (load_asteroids
    # asserts it), and its core tier joins the SAME name set below, so one cut list selects
    # both corpora in every variant -- which is what makes the four files one ladder.
    q, _ = corpus_phase_integral(verbose=False)
    asteroids = load_asteroids(q, {b['name'] for b in base_bodies} | {m['name'] for m in moons})
    core |= {a['name'] for a in asteroids if a['core']}
    print()
    targets = [
        # emit() keys its verbose prints off label.startswith('core'), so the label must
        # keep that prefix. The count is the tier's, not a typed constant: it used to say
        # (21), which was the moon count from before the asteroid corpus had tiers at all.
        (lambda m: m['name'] in core,
         os.path.join(OUT, 'solar_system.json'), 'core (%d names)' % len(core)),
        (lambda m: m['measured'],
         os.path.join(OUT, 'solar_system_measured.json'), 'measured mass'),
        (lambda m: m['named'],
         os.path.join(OUT, 'solar_system_named.json'), 'all named'),
        (lambda m: True,
         os.path.join(OUT, 'solar_system_full.json'), 'full catalog'),
    ]
    if args.check:
        ok = True
        for pred, path, label in targets:
            doc = emit(base_bodies, moons, pred, path, label, fact_sheets, dry=True,
                       asteroids=asteroids)
            with open(path) as f:
                committed = json.load(f)
            same = _comparable(committed) == _comparable(doc)
            # That compare only checks the world_buildings_files REFERENCE;
            # a fragment that does not exist still matches it and throws at
            # load, so check the file behind it too (#226's trap).
            missing = [n for n in doc.get('world_buildings_files', [])
                       if not os.path.exists(os.path.join(ROOT, n))]
            # Same trap for heightmaps: a body naming a .i16 that is not there matches the
            # committed file byte for byte, then throws at LOAD (terragen.h refuses to fall
            # back to noise). Only res/ paths -- a map outside res/ is player-fetched data
            # and its absence is not drift (#754).
            missing += [b['surface']['heightmap'] for b in doc['bodies']
                        if b.get('surface', {}).get('heightmap', '').startswith('res/')
                        and not os.path.exists(os.path.join(ROOT,
                                                            b['surface']['heightmap']))]
            good = same and not missing
            print(('OK      ' if good else 'DRIFT   ') + path)
            for n in missing:
                print('    world_buildings_files: %r not found under %s'
                      % (n, ROOT))
            ok = ok and good
        return 0 if ok else 1
    for pred, path, label in targets:
        emit(base_bodies, moons, pred, path, label, fact_sheets, asteroids=asteroids)

    # ---- verification table for the 'named' set ----
    # hill = the PRE-lift law value; the game's SOI is
    # max(hill, rotSOI + lift) computed by the loader (src/bodylimits.h).
    print()
    print('%-14s %-9s %12s %12s %13s %12s' %
          ('moon', 'parent', 'radius(m)', 'mass(kg)', 'a(m)', 'hill(m)'))
    for m in moons:
        if not m['named'] or not m['a'] or not m['period_s']:
            continue
        radius_m = m['radius_m']
        mass_kg = m['mass_kg'] if m['measured'] else estimate_mass(radius_m)
        pmass = parsed[m['parent']]['mass_kg']
        a = m['a']
        hill = a * (mass_kg / (3.0 * pmass)) ** (1.0/3.0)
        print('%-14s %-9s %12.4g %12.4g %13.4g %12.4g' %
              (m['name'], m['parent'], radius_m, mass_kg, a, hill))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
