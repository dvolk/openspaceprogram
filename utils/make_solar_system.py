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
from atmo_units import resolve_layer_heights, check_body_gravity

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
# A scale-height column hangs on ONE authored number: sea_level_density. The
# loader derives the rest of it -- p0 = rho0 g H, and through H = R T/(M g) the
# datum temperature, the sound speed and every density aloft -- so a mistyped
# datum is invisible to every refusal we have: all those numbers agree with the
# wrong one perfectly. That is how #684 shipped, with Venus's sea_level_density
# typed 92 -- its surface PRESSURE in bar -- which made the whole column 42% too
# fat and its dossier read 129 atm. A guard has to compare two statements, and
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
#                 numbers. Mars sits +35%, and its sheet states its pressure as
#                 "variable from 4.0 to 8.7 mb depending on season" -- a 37%
#                 spread on the sheet's own side.
#   scale height 10%  a directly published figure, authored here in metres. This
#                 is the tight one; one body sits outside it knowingly, see
#                 FACT_AIR_KNOWN.
#   base temp 10% a layered table's first row against the sheet's "Average
#                 temperature", which is a global mean rather than a sea-level
#                 datum. Earth reads +0.1%.
#
# Skipped, with the reason kept beside the code so a future widening is a
# decision rather than a drift:
#   the giants' surface pressure -- their sheets say ">>1000 bars" while our
#   datum is the 1-bar level (Jupiter's column reads 112 kPa, ~1.12 bar).
#   Comparing those compares two different definitions of "surface";
#   any body that authors no density (Mercury, the Moon) -- nothing to contradict;
#   any body with no *fact.html (Titan comes from the giant-planet satellite
#   tables, which carry no air rows);
#   and every world in gen_systems.py, whose air is a design with no sheet to
#   disagree with it.
FACT_AIR_BANDS = {'density': 0.30, 'p0': 0.40, 'scale_height': 0.10, 'base_T': 0.10}

# (body, quantity) -> (the authored value this waiver was written against, why).
# A waiver covers THAT number, not that body forever: change the authored value by
# more than WAIVER_TOL and the guard refuses again, with a message saying the waiver
# predates it. That matters most where a body has few guarded numbers -- a giant's
# sheet states a scale height, a 1-bar density and a molecular weight but no surface
# pressure, so Uranus has exactly two figures this guard can read, and both are
# waived to the same decision.
WAIVER_TOL = 0.02
FACT_AIR_KNOWN = {
    ('Uranus', 'scale_height'): (
        20000.0,
        "sheet says 27.7 km, we author 20 km. The sheet is self-consistent: its "
        "mean molecular weight 2.64 is what our H2/He/CH4 mix resolves to, and 27.7 "
        "km with that M on our g 9.007 is a 79 K 1-bar level. We read 57 K, which "
        "is a tropopause temperature, not a datum one; this H is what was left "
        "beside the typed molar_mass 0.023 that #630 removed. NOT fixed here "
        "because it moves the air top (10 H) 200 -> 277 km and with it the "
        "rotating-frame switch, every spawn bed and the sky rim of a whole world -- "
        "a world change, filed as #695 rather than made at 11 pm."),
    # The same world's density, and the reason these two cannot land apart. The
    # sheet's 0.42 kg/m^3 is right only at the sheet's own 27.7 km scale height:
    # 0.42 x 9.007 x 27.7e3 = 104.8 kPa, i.e. a 1-bar datum, which is the level
    # these giants' gravity rows name. Against the H we still author (20 km) the
    # sheet's density would put this datum at 0.75 bar -- so "fix the density,
    # leave the scale height" makes the row WORSE, and both moves belong with the
    # giants' air-top decision (#700 step 3), which is what #695 is waiting on.
    ('Uranus', 'density'): (
        0.60,
        "sheet says 0.42 kg/m^3 at its 1-bar level, we author 0.60. Lands with the "
        "scale-height waiver above: rho and H trade off against the same datum "
        "pressure, and only the pair (0.42, 27.7 km) reads 1 bar."),
}

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
    unit = re.search(r'kg/m\^?3|bars?|millibars?|microbars?|mb|torr|K\b|C\b|km\b|m\b', t)
    if unit is None:
        return None
    u = unit.group(0)
    if want == 'Pa':
        f = AIR_PRESSURE_UNITS.get(u.lower())
    elif want == 'm':
        f = 1e3 if u == 'km' else (1.0 if u == 'm' else None)
    elif want == 'kg/m^3':
        f = 1.0 if u.startswith('kg/m') else None
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
    base_T} -> (lo, hi) or absent.
    A body with no "Surface density" row may still state the density AT ITS 1-BAR
    LEVEL, and for the giants that IS our datum -- these sheets head their gravity
    column "Gravity (mean, 1 bar)" for the same reason, and check_fact_air already
    falls back on G*M/R^2 for them. So read that row as the density statement, in
    this order, and never both: the giants' sheets state no surface density because
    they have no surface. Their p0 comparison stays skipped either way -- their
    "Surface pressure" row says ">>1000 bars", which is a different definition of
    surface, not a number we are 1000x off. (#685, and the gap #700 found: Saturn
    and Neptune were authored -47 % and +100 % against these rows with nothing
    printing it, because the parser only looked for the terrestrial label.)"""
    h = read(page)
    out = {}
    for key, labels, want in (('density', ('Surface density:', 'Density at 1 bar:'),
                               'kg/m^3'),
                              ('p0', ('Surface pressure',), 'Pa'),
                              ('scale_height', ('Scale height:',), 'm'),
                              ('base_T', ('Average temperature:',), 'K')):
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
        rho0 = atmo.get('sea_level_density')
        H = atmo.get('scale_height')
        if rho0 is not None:
            authored['density'] = rho0
        if H is not None:
            authored['scale_height'] = H
            # p0 is what the loader would derive from these two (atmo.h), on the
            # same gravity this generator scraped for the body itself. Four sheets
            # head their gravity column "Gravity (mean, 1 bar)" instead -- Jupiter,
            # Saturn, Uranus, Neptune -- and leave that scrape None. Fall back on
            # the arithmetic the LOADER uses when a body authors no g (G*M/R^2
            # reproduces the emitted Uranus value, 9.0073) rather than dropping
            # the comparison: a silent '-' is the kind of hole #684 walked through.
            body = parsed.get(name) or {}
            g = body.get('g')
            if g is None and body.get('mass_kg') and body.get('radius_m'):
                g = G * body['mass_kg'] / (body['radius_m'] * body['radius_m'])
            if rho0 is not None and g:
                authored['p0'] = rho0 * g * H
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
        for key in ('density', 'p0', 'scale_height', 'base_T'):
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
            # stale: refuse, and name the value it pinned. The giants' altitude is
            # the only air number their sheets guard, so it has to stay guarded --
            # a filed 20 km argument must not shelter a future 200 km typo.
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
    g = data.get('g')
    if g is None and mass_kg and radius_m:
        g = G * mass_kg / (radius_m * radius_m)

    body = {'name': name, 'type': type_}
    if parent:
        body['orbits'] = parent
    body['radius'] = radius_m
    body['mass'] = mass_kg
    body['g'] = round(g, 4) if g else 9.81
    body['seed'] = seed
    body['has_sea'] = has_sea
    body['power_scaler'] = 1        # unused by the loader; kept for parity
    if surface is not None:
        body['surface'] = resolve_layer_heights(surface, radius_m)

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

MOON_ATMOS = {
    'Titan':  {'color': [0.85, 0.60, 0.25], 'thickness': 40000,
               'power': 3.0, 'intensity': 0.8,
               'sea_level_density': 5.3, 'scale_height': 20000,
               # Titan's air is the densest in the catalogue and still cannot burn:
               # nitrogen, methane, a trace of hydrogen, no free oxygen at all.
               'composition': N2_METHANE},
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
        dict(mass_kg=SUN_MASS, radius_m=SUN_RADIUS, soi=1.0e14, rot_s=None),
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
          # The density, not the pressure: a file cannot author a pressure, so
          # p0 = rho0 g H falls out of this number -- 9.22 MPa here, which is
          # Venus's real 92 BAR (91 atm), the value utils/rss/html/venusfact.html
          # states beside the mass and radius this tuple scrapes. 92 -- that
          # pressure in BAR -- sat in this field for a while and made the dossier
          # read 129 atm, because the pair was two statements of one column
          # (#684). H 16 km and the CO2 mix already give the right 742 K.
          'atmosphere': {'color': [0.90, 0.75, 0.45], 'thickness': 30000,
                         'power': 3.0, 'intensity': 0.8,
                         'sea_level_density': 65.0, 'scale_height': 16000,
                         # Near-pure CO2 with nitrogen on top. Venus has the densest
                         # air a jet could swallow in this catalogue and no oxidizer
                         # in any of it (#630).
                         'composition': [{'gas': 'CO2', 'fraction': 0.965},
                                         {'gas': 'N2', 'fraction': 0.035}]},
          'clouds': {'color': [0.92, 0.82, 0.55], 'height': 5000,
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
          # the lapse rate in K/m (the standard states K/km). Pressure and each level's
          # base pressure are NOT authored -- the loader integrates them hydrostatically
          # from sea_level_density (atmo.h:resolveAtmoLevels), which is why the profile
          # lands on 101,324.9 Pa at the datum rather than an authored 101,325.
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
                         'sea_level_density': 1.225, 'height': 85000,
                         'composition': DRY_AIR,
                         'heights': 'geopotential',
                         'layers': [
                             {'alt': 0,     'T': 288.15, 'lapse': -0.0065},
                             {'alt': 11000, 'T': 216.65, 'lapse': 0.0},
                             {'alt': 20000, 'T': 216.65, 'lapse': 0.001},
                             {'alt': 32000, 'T': 228.65, 'lapse': 0.0028},
                             {'alt': 47000, 'T': 270.65, 'lapse': 0.0},
                             {'alt': 51000, 'T': 270.65, 'lapse': -0.0028},
                             {'alt': 71000, 'T': 214.65, 'lapse': -0.002}]},
          'clouds': {'color': [1.0, 1.0, 1.0], 'height': 2500,
                     'coverage': 0.55, 'freq': 10.0}}, True),
        ('marsfact.html', 'Mars', 4.0,
         {'amplitude': 5000, 'palette': [[0.0, [0.50, 0.25, 0.15]],
                                         [0.5, [0.65, 0.35, 0.22]],
                                         [1.0, [0.80, 0.55, 0.40]]],
          'atmosphere': {'color': [0.85, 0.55, 0.35], 'thickness': 12000,
                         'power': 4.0, 'intensity': 0.5,
                         'sea_level_density': 0.020, 'scale_height': 11500,
                         # Thin CO2 with a 0.13% oxygen trace -- enough to measure,
                         # not enough to run a turbine on.
                         'composition': CO2_HEAVY}},
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
                         'sea_level_density': 0.16, 'scale_height': 27000,
                         # Jupiter keeps its sheet's 0.16 even though the trio lock
                         # wants 0.143 at the 1-bar level (H 27 km, the sheet's own
                         # "Gravity (mean, 1 bar)" 25.92, which is our g to four
                         # digits). That leaves the datum barometer at 1.12 bar, a 12%
                         # miss where Saturn's sheet disagreed with itself by 28% and
                         # we chose the column there: inside the guard's 30% band, so
                         # source fidelity wins and --check prints the difference.
                         'composition': H2_HE}},
         False),
        ('saturnfact.html', 'Saturn', 6.0,
         {'bands': True, 'band_count': 9,
          'palette': [[0.0, [0.62, 0.52, 0.38]], [1.0, [0.90, 0.82, 0.65]]],
          'atmosphere': {'color': [0.90, 0.82, 0.65], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.15, 'scale_height': 60000,
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
                                         {'gas': 'CH4', 'fraction': 0.005}]}},
         False),
        ('uranusfact.html', 'Uranus', 7.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.40, 0.65, 0.70]], [1.0, [0.70, 0.85, 0.88]]],
          'atmosphere': {'color': [0.70, 0.85, 0.88], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.60, 'scale_height': 20000,
                         # Uranus's own H2/He/CH4 proportions, not gen_systems' Jool
                         # (#636: same three gases, a different sky).
                         'composition': H2_HE_CH4_SOLAR}},
         False),
        ('neptunefact.html', 'Neptune', 8.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.18, 0.35, 0.75]], [1.0, [0.50, 0.65, 0.90]]],
          'atmosphere': {'color': [0.50, 0.65, 0.90], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.45, 'scale_height': 20000,
                         # Neptune's sheet states 0.45 kg/m^3 at its 1-bar level and
                         # that IS what its own H 19.1-20.3 km implies at our g
                         # (1e5 / (11.274 x 20000) = 0.443), so this row needed no
                         # judgement -- the old 0.90 was simply wrong, and #700 could
                         # only name it by hand because check_fact_air had no eyes for
                         # a "Density at 1 bar" row. It now reads +1.4% and the datum
                         # barometer says 1.015 bar.
                         'composition': [{'gas': 'H2', 'fraction': 0.80},
                                         {'gas': 'He', 'fraction': 0.19},
                                         {'gas': 'CH4', 'fraction': 0.015}]}},
         False),
        ('plutofact.html', 'Pluto', 9.0,
         {'amplitude': 2000, 'palette': [[0.0, [0.55, 0.45, 0.38]],
                                         [0.5, [0.70, 0.62, 0.55]],
                                         [1.0, [0.85, 0.82, 0.78]]],
          'atmosphere': {'color': [0.70, 0.75, 0.85], 'thickness': 8000,
                         'power': 5.0, 'intensity': 0.3}}, False),
    ]
    rings_by_name = {'Jupiter': JUP_RINGS, 'Saturn': SAT_RINGS}
    for page, name, seed, surface, has_sea in planets:
        data = parse_planet(page)
        parsed[name] = data
        bodies.append(make_body(name, 'planet', 'Sun', data,
                                surface=surface, seed=seed, has_sea=has_sea,
                                rings=rings_by_name.get(name)))
    # The air each of those pages states in prose, against the air this script
    # authors beside it (#685). Refuses before any file is written.
    bad = check_fact_air(planets, parsed, verbose=verbose_air)
    if bad:
        for m in bad:
            print('DRIFT   fact air: ' + m)
        raise SystemExit('%d authored air datum(s) disagree with the fact sheet '
                         'that states them (bands: FACT_AIR_BANDS)' % len(bad))
    # ... and the field that air sits in, stated twice in the same file: a typed `g`
    # beside a mass (#689). The loader refuses the same disagreement
    # (atmo.h:kAtmoGravityTol); catching it here names the scrape that produced it.
    bad = check_body_gravity(bodies, verbose=verbose_air)
    if bad:
        for m in bad:
            print('DRIFT   body gravity: ' + m)
        raise SystemExit('%d bodies author a g that their own mass and radius deny'
                         % len(bad))
    # transfer_dv for a planet needs home (the heliocentric hop); stamp now
    # that every base body exists.
    home = next(b for b in bodies if b['name'] == 'Earth')
    for b in bodies:
        if b['name'] != 'Sun':
            stamp_transfer_dv(b, sun, home)
    return bodies, parsed

def build_moon(m, parent_body=None, home_body=None):
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
    return make_body(name, 'moon', m['parent'], data, seed=1000 + m['idx'],
                     surface=surface, parent_body=parent_body,
                     home_body=home_body)

def _comparable(doc):
    """Copy without the derived science fields (sci_dist.py re-stamps them,
    so they must not count as source drift here)."""
    out = dict(doc)
    out['bodies'] = [
        {k: v for k, v in b.items()
         if k not in ('science_mult', 'transfer_dv')}
        for b in doc.get('bodies', [])]
    return out


def emit(base_bodies, moons, pred, out_path, label, dry=False):
    bodies = list(base_bodies)
    by = {b['name']: b for b in bodies}
    home = by['Earth']
    n = 0
    skipped = 0
    for m in moons:
        if not pred(m):
            continue
        if not m['a'] or not m['period_s']:
            skipped += 1
            continue
        bodies.append(build_moon(m, parent_body=by[m['parent']], home_body=home))
        by[m['name']] = bodies[-1]
        n += 1
    # The base planets were checked with their rows printed; the moons get the same
    # rule quietly (#689), because Titan is the one air in the catalogue with no fact
    # sheet behind it and its g/mass pair is the only statement it makes.
    bad = check_body_gravity(bodies, verbose=False)
    if bad:
        for m in bad:
            print('DRIFT   body gravity: ' + m)
        raise SystemExit('%d emitted bodies author a g that their own mass and radius '
                         'deny (atmo.h:kAtmoGravityTol)' % len(bad))
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
    print('wrote %-32s  %3d bodies (%3d moons)%s  [%s]' %
          (out_path, len(bodies), n,
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

    base_bodies, parsed = build_base(verbose_air=args.check)

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
    targets = [
        (lambda m: m['name'] in core,
         os.path.join(OUT, 'solar_system.json'), 'core (21)'),
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
            doc = emit(base_bodies, moons, pred, path, label, dry=True)
            with open(path) as f:
                committed = json.load(f)
            same = _comparable(committed) == _comparable(doc)
            # That compare only checks the world_buildings_files REFERENCE;
            # a fragment that does not exist still matches it and throws at
            # load, so check the file behind it too (#226's trap).
            missing = [n for n in doc.get('world_buildings_files', [])
                       if not os.path.exists(os.path.join(ROOT, n))]
            good = same and not missing
            print(('OK      ' if good else 'DRIFT   ') + path)
            for n in missing:
                print('    world_buildings_files: %r not found under %s'
                      % (n, ROOT))
            ok = ok and good
        return 0 if ok else 1
    for pred, path, label in targets:
        emit(base_bodies, moons, pred, path, label)

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
