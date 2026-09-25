#!/usr/bin/env python3
"""Generate real-Solar-System star-system JSON files from the NASA NSSDC
fact sheets cached in utils/rss/html/.

Contract with the game loader (src/system.cpp / system.h):
  * reads name/type/orbits/radius/mass/g/seed/has_sea/surface{...}/
    inertial{soi, orb_ang_speed, orb_incl, lon_asc_node, ecc, arg_peri,
    true_anomaly0}/rotating{soi, rot_ang_speed, axial_tilt}.
  * mu = G*mass and the orbital semimajor axis a = cbrt(mu_parent / w^2) are
    DERIVED, so we supply real masses + real periods and the game reproduces
    the true orbital radii (Kepler's third law closes the loop).
  * Inertial SoIs are Hill spheres: a * (m_body / (3 m_parent))^(1/3).
  * Bodies without a "rotating" section get the loader's default near-body
    SOI (radius + 100 km, zero spin).
  * Unknown keys are ignored, so we can embed ring data for future use.

We emit one system file per moon-scope so you can A/B them in-game:
  res/systems/solar_system.json            the 21-moon core (baseline)
  res/systems/solar_system_measured.json   only moons with a measured mass
  res/systems/solar_system_named.json      every NAMED moon (drops S/xxxx specks)
  res/systems/solar_system_full.json       every moon in the fact sheets

Notes:
  * Moons without a measured mass get one estimated from radius at a nominal
    small-body density (ice/rock ~2000 kg/m^3) so the loader always has a mass.
  * Retrograde planets (Venus/Uranus/Pluto): the fact sheets give a negative
    rotation period AND an obliquity > 90 deg. We use the period magnitude so
    the spin rate stays positive (a negative rate would make the body's
    calendar invalid) and let the >90 deg tilt carry the retrograde.
  * Each moon's near-body (surface) SOI is radius + 100 km (the loader's
    convention), and its inertial (orbit) SOI is the Hill sphere -- lifted to
    radius + 200 km when the Hill sphere is smaller, since the SOI is a frame
    boundary, not a physical limit, and the smallest moons are un-landable
    otherwise (the 10 km frame-switch hysteresis would eat their Hill sphere).

Sources: utils/rss/html/*.html (NASA NSSDC planetary fact sheets, cached 2026).
Run from anywhere:  python3 utils/make_solar_system.py
"""
import re, math, json, os, html as htmllib

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

# Lives in utils/ (like gen_systems.py). The NASA NSSDC fact sheets are the
# input (utils/rss/html/); the generated systems go to res/systems/ where the
# game loads them (src/system.cpp + the res/systems/ directory scan).
HERE = os.path.dirname(os.path.abspath(__file__))   # utils/
ROOT = os.path.dirname(HERE)                          # repo root
BASE = os.path.join(HERE, 'rss', 'html')
OUT  = os.path.join(ROOT, 'res', 'systems')

# ---------------------------------------------------------------------------
# small HTML helpers
# ---------------------------------------------------------------------------
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
    """Parse a clean numeric cell (e.g. '0.33010' or '30 x 20 x 17' -> mean).
    Returns None if no number is present."""
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
    """Return {normalized-label: value} for the data block (table or <pre>)
    that follows `marker`. Table rows -> value = 2nd cell; pre lines -> value
    = last number on the line (so labels like '10^24 kg' don't pollute it)."""
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
            # value = the number at the end of the line; label = everything
            # before it (so labels like 'Mass (10^24 kg)' keep their digits).
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
    """'Naiad (NIII)' -> 'Naiad'; 'Philophrosyne (LVIII, S/2003 J15)' -> 'Philophrosyne'."""
    return name.split('(')[0].strip()

def is_named(primary):
    """A moon is 'named' if it has a proper name (not a provisional S/xxxx or
    S5605a2-style designation)."""
    p = primary.strip()
    if p.startswith(('S/', 'T/')):
        return False
    if re.match(r'^[ST]\d', p):        # S5605a2, T522499, S2428b, ...
        return False
    return bool(re.search(r'[A-Za-z]', p))

def radius_from_cell(s):
    """Pluto 'Other Moons' 'Mean Diameter (km)' column: the single values are
    diameters (Styx 10.5 -> 5.25 km, matching real Styx ~5.5 km) and the
    triaxial cells are diameters too (Nix 48.4x33.8x31.4 -> mean/2 = 18.9 km,
    matching real Nix). So radius = mean / 2 in both cases. (The giant-planet
    sheets use a 'Radius (km)' column of semi-axes and are handled separately.)"""
    if s is None:
        return None
    v = num(s)
    return (v / 2.0 * 1e3) if v is not None else None

def estimate_mass(radius_m, rho=SMALL_BODY_RHO):
    return rho * (4.0 / 3.0) * math.pi * radius_m ** 3

# ---------------------------------------------------------------------------
# planets
# ---------------------------------------------------------------------------
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
    raan = get(j2000, 'longitudeofascendingnode') * D2R
    varp = get(j2000, 'longitudeofperihelion') * D2R
    L    = get(j2000, 'meanlongitude') * D2R

    period_s = period_d * DAY
    w        = TWO_PI / period_s
    omega    = varp - raan                   # arg of periapsis (in-plane)
    nu0      = true_anomaly(L - varp, e)
    soi      = a_m * (mass_kg / (3.0 * SUN_MASS)) ** (1.0/3.0)

    return dict(mass_kg=mass_kg, radius_m=radius_m, g=g,
                # obliquity > 90 deg already encodes retrograde; keep the rate
                # positive or the body's calendar goes invalid (D -> 0).
                rot_s=(abs(rot_hr) * HR if rot_hr else None),
                obliquity=(obliq_deg * D2R if obliq_deg is not None else 0.0),
                a=a_m, e=e, i=i, raan=raan, omega=omega, nu0=nu0,
                w=w, soi=soi)

# ---------------------------------------------------------------------------
# moons
# ---------------------------------------------------------------------------
def parse_sat_table_all(page):
    """Return every satellite listed in a giant-planet fact sheet as a list of
    dicts, joining the bulk table (mass, radius) and the orbital table (a,
    period, i, e) by clean name. Columns: bulk name|mass(10^20kg)|radius(km);
    orbital name|a(10^3km)|a(radii)|period(days)|rot(days)|i(deg)|e."""
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
            continue                       # section label / separator row
        mass = num(g(r, 1))
        clean = clean_name(r[0])
        om = orb_map.get(clean, {})
        out.append(dict(
            name=clean,
            mass_kg=(mass * 1e20 if mass is not None else None),
            radius_m=radius * 1e3,
            a=(om['a'] * 1e6 if om.get('a') is not None else None),   # 10^3 km -> m
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
        a=get(orb, 'semimajoraxis') * 1e9,          # 10^6 km -> m
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
    """Charon from its own block (has mass); the rest from 'Other Moons'
    (diameter or triaxial axes, no mass)."""
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

# ---------------------------------------------------------------------------
# emit a game body
# ---------------------------------------------------------------------------
def make_body(name, type_, parent, data, *, surface=None, seed=0.0,
              has_sea=False, rings=None):
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
        body['surface'] = surface

    inertial = {'soi': data['soi']}
    if parent:
        inertial['orb_ang_speed'] = data['w']
        if data.get('e') is not None:
            inertial['ecc'] = data['e']
        inertial['arg_peri'] = data.get('omega', 0.0)
        inertial['orb_incl'] = data.get('i', 0.0)
        inertial['lon_asc_node'] = data.get('raan', 0.0)
        inertial['true_anomaly0'] = data.get('nu0', 0.0)
    else:
        inertial['orb_ang_speed'] = 0.0
    body['inertial'] = inertial

    if data.get('rot_s'):
        body['rotating'] = {
            'soi': data.get('rotating_soi', radius_m + 100e3),
            'rot_ang_speed': TWO_PI / data['rot_s'],
            'axial_tilt': data.get('obliquity', 0.0),
        }
    elif data.get('rotating_soi'):
        # Moon: near-body SOI = radius + 100 km (build_moon); the inertial /
        # orbit SOI is lifted to sit just outside it when the Hill sphere is
        # too small for the 10 km frame-switch hysteresis to fit.
        body['rotating'] = {'soi': data['rotating_soi'],
                            'rot_ang_speed': 0.0, 'axial_tilt': 0.0}
    if rings is not None:
        # Rings are a surface property (like atmosphere / clouds), so they
        # live in the surface block the loader reads (system.cpp: sv["rings"]).
        body.setdefault('surface', {})['rings'] = rings
    return body

def rock(r, c1, c2):
    # Keep relief proportional on small moons: floor at 500 m but cap at 25%
    # of the radius, so a 300 m moon isn't 167% relief (a lumpy wreck).
    amp = min(int(r * 0.25), max(500, int(r * 0.02)))
    return {'amplitude': max(50, amp),
            'palette': [[0.0, c1], [1.0, c2]]}

# ---- moon surface colours (fallback grey for the long tail) ----
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

# Moons with a real (or famously hazy) atmosphere get a limb rim on top of
# rock(); most of the ~230 moons are airless, so this stays a short list.
MOON_ATMOS = {
    'Titan':  {'color': [0.85, 0.60, 0.25], 'thickness': 40000,
               'power': 3.0, 'intensity': 0.8,
               'sea_level_density': 5.3, 'scale_height': 20000},
    'Triton': {'color': [0.60, 0.70, 0.85], 'thickness': 5000,
               'power': 5.0, 'intensity': 0.3},   # N2 + methane haze
    'Io':     {'color': [0.85, 0.75, 0.40], 'thickness': 4000,
               'power': 5.0, 'intensity': 0.25},  # thin SO2 exosphere
}

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def ring(name, r_in_km, r_out_km, thickness_km, albedo, opacity=0.8):
    return {'name': name, 'inner': r_in_km * 1e3, 'outer': r_out_km * 1e3,
            'thickness': thickness_km * 1e3, 'albedo': albedo,
            'opacity': opacity}

# Albedos are boosted from the (very dark) real values so the bands read as
# rings in-game: the real Jupiter rings are ~0.01-0.02 albedo, which would
# render black. The band geometry (inner / outer km) is the real extent.
JUP_RINGS = [
    ring('Halo', 89400, 123000, 10000, 0.15),
    ring('Main', 123000, 128940, 100, 0.30),
    ring('Amalthea', 128940, 181350, 2600, 0.12),
    ring('Thebe', 181350, 280000, 8800, 0.12),
]

# Saturn's rings -- the famous ones. The Cassini Division (~117,500-122,000
# km) is the gap between the B- and A-rings (no band there). Albedos are the
# real band brightnesses (the B-ring is the brightest / densest).
SAT_RINGS = [
    ring('C-ring', 74500, 92000, 1000, 0.35, 0.60),   # inner, dark, thin
    ring('B-ring', 92000, 117500, 3000, 0.75, 0.90),  # bright, dense
    ring('A-ring', 122000, 137000, 200, 0.55, 0.70),  # outer
]

def build_base():
    """Sun + 9 planets, identical across every scope file. Returns
    (bodies, parsed) where parsed[name] carries each parent's mass_kg."""
    bodies = []
    parsed = {}

    sun = make_body('Sun', 'star', None,
        dict(mass_kg=SUN_MASS, radius_m=SUN_RADIUS, soi=1.0e14, rot_s=None),
        # No atmosphere rim: the limb shader's day/night term falls back to a
        # fixed world direction for a star, painting a half-eclipse crescent.
        # The photosphere palette is enough (KSP's Kerbol has no rim either).
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
          'atmosphere': {'color': [0.90, 0.75, 0.45], 'thickness': 30000,
                         'power': 3.0, 'intensity': 0.8,
                         'sea_level_density': 92.0, 'scale_height': 16000},
          'clouds': {'color': [0.92, 0.82, 0.55], 'height': 5000,
                     'coverage': 0.9, 'freq': 8.0}}, False),
        ('earthfact.html', 'Earth', 3.0,
         {'amplitude': 8000, 'sea_level': 0.0, 'sea_color': [0.0, 0.18, 0.50],
          'palette': [[0.0, [0.55, 0.50, 0.35]],   # sandy coast
                      [0.12, [0.12, 0.42, 0.15]],  # lowland forest
                      [0.35, [0.30, 0.42, 0.18]],  # grassland
                      [0.60, [0.45, 0.40, 0.30]],  # brown highlands
                      [0.82, [0.60, 0.58, 0.55]],  # rock
                      [1.0, [0.95, 0.95, 0.97]]],  # snow peaks
          'atmosphere': {'color': [0.30, 0.50, 1.00], 'thickness': 15000,
                         'power': 4.0, 'intensity': 0.7,
                         'sea_level_density': 1.225, 'scale_height': 8500},
          'clouds': {'color': [1.0, 1.0, 1.0], 'height': 2500,
                     'coverage': 0.55, 'freq': 10.0}}, True),
        ('marsfact.html', 'Mars', 4.0,
         {'amplitude': 5000, 'palette': [[0.0, [0.50, 0.25, 0.15]],
                                         [0.5, [0.65, 0.35, 0.22]],
                                         [1.0, [0.80, 0.55, 0.40]]],
          # thin dusty CO2 haze
          'atmosphere': {'color': [0.85, 0.55, 0.35], 'thickness': 12000,
                         'power': 4.0, 'intensity': 0.5,
                         'sea_level_density': 0.020, 'scale_height': 11500}},
         False),
        # gas giants: band ramp = dark (pole / band edge) -> light (equator /
        # band centre), sampled by the triangle wave in BandColor. The "air"
        # is the whole body, so each also gets a broad, soft limb rim (KSP's
        # Jool does too); the rim tint matches the light band.
        ('jupiterfact.html', 'Jupiter', 5.0,
         {'bands': True, 'band_count': 11,
          'palette': [[0.0, [0.50, 0.40, 0.32]], [1.0, [0.85, 0.78, 0.65]]],
          'atmosphere': {'color': [0.85, 0.78, 0.65], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.16, 'scale_height': 27000}},
         False),
        ('saturnfact.html', 'Saturn', 6.0,
         {'bands': True, 'band_count': 9,
          'palette': [[0.0, [0.62, 0.52, 0.38]], [1.0, [0.90, 0.82, 0.65]]],
          'atmosphere': {'color': [0.90, 0.82, 0.65], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.10, 'scale_height': 60000}},
         False),
        ('uranusfact.html', 'Uranus', 7.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.40, 0.65, 0.70]], [1.0, [0.70, 0.85, 0.88]]],
          'atmosphere': {'color': [0.70, 0.85, 0.88], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.60, 'scale_height': 20000}},
         False),
        ('neptunefact.html', 'Neptune', 8.0,
         {'bands': True, 'band_count': 7,
          'palette': [[0.0, [0.18, 0.35, 0.75]], [1.0, [0.50, 0.65, 0.90]]],
          'atmosphere': {'color': [0.50, 0.65, 0.90], 'thickness': 90000,
                         'power': 3.0, 'intensity': 0.6,
                         'sea_level_density': 0.90, 'scale_height': 20000}},
         False),
        ('plutofact.html', 'Pluto', 9.0,
         {'amplitude': 2000, 'palette': [[0.0, [0.55, 0.45, 0.38]],
                                         [0.5, [0.70, 0.62, 0.55]],
                                         [1.0, [0.85, 0.82, 0.78]]],
          # a faint nitrogen haze
          'atmosphere': {'color': [0.70, 0.75, 0.85], 'thickness': 8000,
                         'power': 5.0, 'intensity': 0.3}}, False),
    ]
    # Rings are a surface property: only Jupiter + Saturn get them (the
    # other gas giants' rings are too faint to bother with in v1).
    rings_by_name = {'Jupiter': JUP_RINGS, 'Saturn': SAT_RINGS}
    for page, name, seed, surface, has_sea in planets:
        data = parse_planet(page)
        parsed[name] = data
        bodies.append(make_body(name, 'planet', 'Sun', data,
                                surface=surface, seed=seed, has_sea=has_sea,
                                rings=rings_by_name.get(name)))
    return bodies, parsed

def build_moon(m, parsed):
    """Turn a master-moon record into a game body (mass estimated if absent)."""
    name = m['name']
    radius_m = m['radius_m']
    mass_kg = m['mass_kg'] if m['measured'] else estimate_mass(radius_m)
    pmass = parsed[m['parent']]['mass_kg']
    a = m['a']
    w = TWO_PI / m['period_s'] if m['period_s'] else 0.0
    hill = a * (mass_kg / (3.0 * pmass)) ** (1.0/3.0) if a and mass_kg else a * 0.05
    # orientation (raan/omega) isn't in the fact sheets; spread the epoch
    # anomaly by the golden angle for a natural, non-degenerate layout.
    nu0 = (m['idx'] * GOLDEN) % TWO_PI
    # Near-body (surface) SOI = the loader's own convention (radius + 100 km),
    # which is always landable: switchFrames (vehicle.cpp) enters the surface
    # frame at dist < soi - 10 km, so this leaves a 90 km corridor above the
    # ground. The inertial (orbit) SOI is the Hill sphere, but for the smallest
    # moons the Hill sphere is so small the 10 km frame-switch hysteresis eats
    # it (soi - 10 km < radius: the ship crashes into the moon before it can
    # enter the surface frame). Those are the un-landable ones, so lift the
    # orbit SOI to just outside the surface SOI. The SOI is a frame boundary,
    # not a physical limit -- the big moons keep their true Hill value, which
    # already dwarfs radius + 200 km.
    surf = radius_m + 100e3
    soi = max(hill, surf + 100e3)
    data = dict(radius_m=radius_m, mass_kg=mass_kg, soi=soi, w=w,
                e=m['e'], omega=0.0, i=m['i'], raan=0.0, nu0=nu0,
                rot_s=None, rotating_soi=surf)
    c1, c2 = MOON_COLORS.get(name, DEFAULT_COLOR)
    surface = rock(radius_m, c1, c2)
    if name in MOON_ATMOS:
        surface['atmosphere'] = MOON_ATMOS[name]
    return make_body(name, 'moon', m['parent'], data, seed=1000 + m['idx'],
                     surface=surface)

def emit(base_bodies, parsed, moons, pred, out_path, label):
    bodies = list(base_bodies)
    n = 0
    skipped = 0
    for m in moons:
        if not pred(m):
            continue
        if not m['a'] or not m['period_s']:
            skipped += 1
            continue
        bodies.append(build_moon(m, parsed))
        n += 1
    doc = {'home': 'Earth', 'bodies': bodies}
    with open(out_path, 'w') as f:
        json.dump(doc, f, indent=2)
        f.write('\n')
    print('wrote %-32s  %3d bodies (%3d moons)%s  [%s]' %
          (out_path, len(bodies), n,
           ('  (%d skipped: no orbit data)' % skipped) if skipped else '', label))
    return n

def main():
    base_bodies, parsed = build_base()

    moons = []
    def add(*ms):
        for m in ms:
            m['idx'] = len(moons)
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

    n_named = sum(1 for m in moons if m['named'])
    n_meas  = sum(1 for m in moons if m['measured'])
    print('master moon table: %d moons (%d named, %d with measured mass)'
          % (len(moons), n_named, n_meas))
    print()

    core = {'Moon', 'Phobos', 'Deimos', 'Io', 'Europa', 'Ganymede',
            'Callisto', 'Mimas', 'Enceladus', 'Tethys', 'Dione', 'Rhea',
            'Iapetus', 'Titan', 'Miranda', 'Ariel', 'Umbriel', 'Titania',
            'Oberon', 'Triton', 'Charon'}
    emit(base_bodies, parsed, moons,
         lambda m: m['name'] in core, os.path.join(OUT, 'solar_system.json'), 'core (21)')
    emit(base_bodies, parsed, moons,
         lambda m: m['measured'], os.path.join(OUT, 'solar_system_measured.json'), 'measured mass')
    emit(base_bodies, parsed, moons,
         lambda m: m['named'], os.path.join(OUT, 'solar_system_named.json'), 'all named')
    emit(base_bodies, parsed, moons,
         lambda m: True, os.path.join(OUT, 'solar_system_full.json'), 'full catalog')

    # ---- verification table for the 'named' set ----
    print()
    print('%-14s %-9s %12s %12s %13s %12s' %
          ('moon', 'parent', 'radius(m)', 'mass(kg)', 'a(m)', 'soi(m)'))
    for m in moons:
        if not m['named'] or not m['a'] or not m['period_s']:
            continue
        radius_m = m['radius_m']
        mass_kg = m['mass_kg'] if m['measured'] else estimate_mass(radius_m)
        pmass = parsed[m['parent']]['mass_kg']
        a = m['a']
        soi = a * (mass_kg / (3.0 * pmass)) ** (1.0/3.0)
        print('%-14s %-9s %12.4g %12.4g %13.4g %12.4g' %
              (m['name'], m['parent'], radius_m, mass_kg, a, soi))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
