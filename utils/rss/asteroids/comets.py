#!/usr/bin/env python3
"""Derive utils/rss/asteroids/comets.json: comet nuclei we can actually author a body for.

Sources (also listed in urls.txt):
  https://minorplanetcenter.net/Extended_Files/allcometels.json.gz
      -> raw/allcometels.json.gz   (200 KB gzipped, 1.5 MB as text, reissued daily. Unlike
         the 175 MB MPCORB blob this one is small enough to commit, and MPC's header asks
         for redistribution with attribution -- which every consumer below carries.)
  https://en.wikipedia.org/wiki/List_of_Solar_System_objects_by_size
      -> raw/objects_by_size.html  (already cached for the asteroids and read by sizes.py.
         It is the ONLY place we have a comet nucleus radius: 12 of them, 60 km to 300 m.)

What the MPC file is: 4,666 ORBIT records, one per APPARITION, not one per comet. A row
states Perihelion_dist (AU), e, Peri (argument of perihelion), Node, i, the perihelion date
(Year/Month/Day_of_perihelion -- negative years are real: C/-146 P1 is the Comet of 146 AD),
and for modern rows an Epoch plus the brightness law H and G. Orbit_type is C (long period /
non-periodic), P (periodic), D (disintegrated or lost), A (reclassified as an asteroid) or
I (interstellar).

Two things the file does NOT give, and both decide what can be built:

  * NO nucleus size, anywhere. For a comet, absolute magnitude H is the brightness of the
    COMA -- how much gas the nucleus is throwing off, not how big the rock is. That is what
    the slope parameter G (usually 4.0, the classic m = H + 5 log Delta + 10 log r) is there
    to describe. So the trick parse.py uses over 419 asteroid rows, D = 1329/sqrt(pV) *
    10^(-H/5), is not imprecise here, it is meaningless: Halley at H 5.3 sizes to ~20 km at
    pV 0.04 against a flyby-measured 5.75 km, and a dormant nucleus and an active one of the
    same size differ by orders. So a comet enters this file only when a page states its
    nucleus radius. That is a deliberate limit on the roster, not an oversight.
  * closed orbits only. 2,351 rows carry e exactly 1.0 (single-sighting ancient comets, where
    a parabola IS the whole solution) and 354 are hyperbolic. a = q/(1-e) for those is not a
    number: C/2022 F1 (ATLAS) at e 0.999997 gives a = 373,777 AU, an apoapsis two thirds of a
    parsec out. Those rows are refused and printed, never clamped -- a clamped ellipse is a
    different orbit than the one the MPC computed.

Joining a size row to an orbit row uses the MPC's own identity (id_key): the number+letter for
numbered comets ('67P'), or the full 'C/1995 O1' designation for the rest. Where the size page
writes a name instead, COMET_ALIASES says which MPC row it means -- and those were checked
against the file rather than remembered, because two of them are not what they look like.
"""
import argparse
import calendar
import gzip
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sizes  # noqa: E402  -- the by-size page parser, already used by parse.py

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')
ELEMENTS_GZ = os.path.join(RAW, 'allcometels.json.gz')
SIZE_HTML = os.path.join(RAW, 'objects_by_size.html')
OUT = os.path.join(HERE, 'comets.json')

# The epoch the catalogue is a snapshot of. Keep in step with make_solar_system.py.
CATALOGUE_EPOCH_JD = 2451544.5
AU_M = 1.495978707e11
YR_D = 365.256363

# The size page names some bodies instead of giving a designation the MPC can be matched on.
# Each value is the MPC identity key (id_key below) of the row meant. Checked against the
# file, not from memory: the Great Comet of 1811 is C/1811 F1 there, Tsuchinshan-ATLAS is
# C/2023 A3 (C/2022 N1 is a different comet, Attard-Maury), and the MPC writes "9P/Tempel"
# where the size page writes "Tempel 1".
COMET_ALIASES = {
    "Halley's Comet": "1P",
    "Tempel 1": "9P",
    "Great comet": "C/1811 F1",
    "Tsuchinshan–ATLAS": "C/2023 A3",
}
# Sized comets deliberately NOT authored. Printed, never dropped silently: a reader of the
# report should see that the largest known comet nucleus is missing ON PURPOSE.
REFUSED = {
    "Bernardinelli-Bernstein":
        "the MPC's own solution is e = 1.00421, hyperbolic: no ellipse to author",
    "ATLAS":
        "3I/ATLAS is interstellar (e = 6.12); the loader authors closed Keplerian rails only",
}

_ID_NUM = re.compile(r'^(\d{1,4}[PDCXAI])/')
_ID_LETTER = re.compile(r'^([PCDAI])\s*/\s*(\d{3,4}\s+[A-Z0-9]{1,4})')


def id_key(name):
    """The MPC's own identity for a row: '67P' for a numbered periodic comet (every
    apparition shares it), 'C/1995 O1' for a single apparition of a long-period one.

    Splitting on '/' is NOT enough: every 'C/...' row would then group under the single key
    'C', which is how the file lumps 3,735 unrelated comets together."""
    m = _ID_NUM.match(name.strip())
    if m:
        return m.group(1)
    m = _ID_LETTER.match(name.strip())
    if m:
        return '%s/%s' % (m.group(1), ' '.join(m.group(2).split()))
    return name.strip()


def proper_name(name):
    """'1P/Halley' -> 'Halley'; 'C/1995 O1 (Hale-Bopp)' -> 'Hale-Bopp'."""
    rest = name.split('/', 1)[1] if '/' in name else name
    paren = re.search(r'\((.*?)\)', rest)
    return (paren.group(1) if paren else rest).strip()


def jd_of(y, m, d):
    """Julian Date for a signed year, month and possibly fractional day. Accurate to a
    fraction of a day over this file's span (146 BC to 5766 AD), far finer than the elements."""
    y = int(y)
    d = float(d)
    day = int(d)
    frac = d - day
    if y >= 1:
        base = calendar.timegm((y, m, day, 12, 0, 0)) / 86400.0 + 2440587.5
    else:
        # timegm refuses BC years. Only two rows are that old; step back year by year.
        base = (calendar.timegm((1, m, day, 12, 0, 0)) / 86400.0 + 2440587.5
                - (1 - y) * 365.2425)
    return base + frac


def mean_motion_deg_d(a_au):
    """Kepler's third law in AU and years, out in the deg/day the loader's rail wants."""
    return 360.0 / (a_au ** 1.5 * YR_D)


def family(orbit_type, a_au, period_y, q_au):
    """The dynamical family, said coarsely on purpose: it is a variety key for choosing a
    roster, not a classifier with a paper behind it."""
    if orbit_type == 'I':
        return 'interstellar'
    if orbit_type == 'D':
        return 'disintegrated'
    if orbit_type == 'A':
        return 'reclassified asteroid'
    if a_au is None:
        return 'parabolic'
    if q_au < 0.1:
        return 'sungrazer'
    if period_y is not None and period_y < 20.0:
        return 'jupiter-family'
    if period_y is not None and period_y < 200.0:
        return 'halley-type'
    return 'long-period'


def radius_rows():
    """[dict] The by-size page's comet rows: name, designation, radius, mass, notes."""
    if not os.path.exists(SIZE_HTML):
        return []
    text = open(SIZE_HTML, encoding='utf-8', errors='replace').read()
    return [e for e in sizes.objects(text)
            if 'comet' in (e.get('type_note') or '').lower()]


def elements():
    with gzip.open(ELEMENTS_GZ, 'rt', encoding='utf-8') as f:
        return json.load(f)


def pick_apparition(rows):
    """The orbit row to build a rail from: the apparition whose perihelion is NEAREST the
    catalogue epoch, because the catalogue is a snapshot and the nearest published solution is
    the least extrapolated one. A periodic comet's elements change every return (non-
    gravitational outgassing kicks them), so this choice is visible in the output as the
    perihelion date rather than hidden."""
    best, best_d = None, None
    for r in rows:
        t = jd_of(int(r['Year_of_perihelion']), int(r['Month_of_perihelion']),
                  int(float(r['Day_of_perihelion'])))
        d = abs(t - CATALOGUE_EPOCH_JD)
        if best_d is None or d < best_d:
            best, best_d = r, d
    return best


def eccentric_anomaly(M, e):
    """Solve M = E - e sin E (Newton), the same two-step the generator uses for the asteroids
    (#784 is the ticket that folds the two solvers together). Stable up to e -> 1."""
    E = M
    for _ in range(80):
        E = E - (E - e * math.sin(E) - M) / (1.0 - e * math.cos(E))
    return E


def heliocentric_r(a_au, e, t_peri_jd, jd):
    """Radius vector magnitude (AU) at a date, from a and the perihelion date the MPC gives."""
    n_deg_d = mean_motion_deg_d(a_au)
    M = math.radians((n_deg_d * (jd - t_peri_jd)) % 360.0)
    E = eccentric_anomaly(M, e)
    nu = 2.0 * math.atan2(math.sqrt(1.0 + e) * math.sin(E / 2.0),
                          math.sqrt(1.0 - e) * math.cos(E / 2.0))
    return a_au * (1.0 - e * e) / (1.0 + e * math.cos(nu))


def build(verbose=True):
    rows = elements()
    sized = radius_rows()          # one parse of the size page, not one per call site
    by_key = {}
    for r in rows:
        by_key.setdefault(id_key(r['Designation_and_name']), []).append(r)

    out = []
    refused = []
    for e in sorted(sized, key=lambda e: e['name']):
        display = e['name']
        if display in REFUSED:
            refused.append((display, REFUSED[display]))
            continue
        key = id_key(e['designation'] or '') if e['designation'] else None
        if not key or key not in by_key:
            key = COMET_ALIASES.get(display)
        cands = by_key.get(key)
        if not cands:
            refused.append((display, 'no MPC orbit row for identity %r (size page says %r)'
                            % (key, e['designation'] or display)))
            continue
        r = pick_apparition(cands)
        q = float(r['Perihelion_dist'])
        ecc = float(r['e'])
        if not (0.0 <= ecc < 1.0):
            refused.append((display, 'the MPC row states e = %g: no ellipse to author' % ecc))
            continue
        a = q / (1.0 - ecc)
        t_jd = jd_of(int(r['Year_of_perihelion']), int(r['Month_of_perihelion']),
                     int(float(r['Day_of_perihelion'])))
        period_y = a ** 1.5
        r_ep = heliocentric_r(a, ecc, t_jd, CATALOGUE_EPOCH_JD)
        orbit_name = r['Designation_and_name']
        out.append(dict(
            name=display,
            desig=orbit_name,
            key=id_key(orbit_name),
            orbit_type=r.get('Orbit_type'),
            family=family(r.get('Orbit_type'), a, period_y, q),
            radius_m=e['radius_m'],
            radius_text=e['radius_text'],
            radius_src='Wikipedia List of Solar System objects by size, table %d' % e['table'],
            mass_kg=e['mass_kg'],
            mass_text=e['mass_text'],
            mass_src=('stated on the size page, table %d' % e['table']) if e['mass_kg']
                     else 'NOT STATED -- a density rule would have to invent it',
            q_au=q, e=ecc, a_au=a, Q_au=a * (1.0 + ecc), period_y=period_y,
            incl_deg=float(r['i']), node_deg=float(r['Node']), peri_deg=float(r['Peri']),
            perihelion_jd=t_jd,
            perihelion='%+04d-%02d-%04.1f' % (int(r['Year_of_perihelion']),
                                              int(r['Month_of_perihelion']),
                                              float(r['Day_of_perihelion'])),
            epoch_jd=(jd_of(int(r['Epoch_year']), int(r['Epoch_month']),
                            int(float(r['Epoch_day']))) if r.get('Epoch_year') else None),
            H=r.get('H'), G=r.get('G'),
            apparitions=len(cands),
            r_at_epoch_au=r_ep,
            type_note=e['type_note'],
        ))

    if verbose:
        tally = {}
        for r in rows:
            tally[r.get('Orbit_type')] = tally.get(r.get('Orbit_type'), 0) + 1
        print('comets: %d MPC orbit records in %d identities' % (len(rows), len(by_key)))
        print('  Orbit_type:', ' '.join('%s %d' % kv for kv in sorted(tally.items())))
        print('  nucleus radii stated: %d   authorable: %d' % (len(sized), len(out)))
        for name, why in refused:
            print('  REFUSED %s: %s' % (name, why))
        print('  %-26s %-16s %8s %8s %9s %9s %9s %7s %6s'
              % ('comet', 'family', 'radius_m', 'q AU', 'e', 'P yr', 'Q AU', 'r@epoch',
                 'i deg'))
        for c in sorted(out, key=lambda c: (c['family'], -c['radius_m'])):
            print('  %-26s %-16s %8.0f %8.3f %9.5f %9.1f %9.1f %7.2f %6.1f'
                  % (c['name'][:26], c['family'], c['radius_m'], c['q_au'], c['e'],
                     c['period_y'], c['Q_au'], c['r_at_epoch_au'], c['incl_deg']))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--write', action='store_true', help='write comets.json')
    ap.add_argument('--check', action='store_true',
                    help='derive without writing and fail if comets.json differs')
    args = ap.parse_args()
    out = build(verbose=True)
    doc = {'bodies': out}
    if args.write:
        with open(OUT, 'w') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
            f.write('\n')
        print('wrote %s (%d comets)' % (OUT, len(out)))
        return 0
    if args.check:
        same = json.load(open(OUT)) == doc
        print(('OK      ' if same else 'DRIFT   ') + OUT)
        return 0 if same else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
