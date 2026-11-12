#!/usr/bin/env python3
"""Derive utils/rss/asteroids/comets.json: comet nuclei we can actually author a body for.

Sources (also listed in urls.txt):
  https://minorplanetcenter.net/Extended_Files/allcometels.json.gz
      -> raw/allcometels.json.gz   (200 KB gzipped, 1.5 MB as text, reissued daily. Unlike
         the 175 MB MPCORB blob this one is small enough to commit, and MPC's header asks
         for redistribution with attribution -- which every consumer below carries.)
  https://en.wikipedia.org/wiki/List_of_Solar_System_objects_by_size
      -> raw/objects_by_size.html  (already cached for the asteroids and read by sizes.py.
         It is where most of our comet nucleus radii come from: 12 of them, 60 km to 300 m.)
  https://en.wikipedia.org/wiki/Great_comet
      -> raw/great_comet.html      (read by great_rows() below. The ONLY source we hold for a
         long-period nucleus size -- Hale-Bopp, Ikeya-Seki, McNaught, NEOWISE -- and its
         "Dimensions" column is a DIAMETER, which the by-size page's own comparison proves:
         C/1811 F1 is "30 - 40 km" here and "15 - 20" (a radius) there, Halley "11 km" here
         against 5.75 km there. Halved, never halved twice.)

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
GREAT_HTML = os.path.join(RAW, 'great_comet.html')
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


# Words that carry no identity. They sit in both columns of a CORRECT join ("Great Comet of
# 1882" / "(Great September comet)") and in nothing that tells one comet from another, so a
# name test that counted them would pass the join it is meant to refuse.
NAME_STOPWORDS = frozenset({'great', 'comet', 'comets', 'of', 'the', 'and', 'large',
                            'small', 'day', 'night', 'north', 'northern',
                            'south', 'southern', 'century'})


def name_tokens(s):
    """[set] The identity-bearing words of a comet's name, lower-cased."""
    return {w for w in re.findall(r'[a-z]+', (s or '').lower())
            if w not in NAME_STOPWORDS}


def great_rows():
    """[dict] The 'Great comet' article's list of great comets: Designation, Name,
    Dimensions (km), Perihelion date. Shaped like sizes.objects() rows so build() joins
    both pages the same way; it is the only source we hold for a long-period nucleus size
    (the by-size page has 12 comets and not one of them is a great comet of the last
    century).

    Three things this page does that the by-size page does not, each handled here:
      * Dimensions is a DIAMETER. Halving it is not a guess, it is what the page itself
        says against the size page: C/1811 F1 is '30 - 40 km' here and '15 - 20' (a
        radius) there; Halley is '11 km' here against 5.75 km there.
      * Its Designation column is not always the MPC's. The page labels Hyakutake
        'C/1996 B1'; MPC's C/1996 B1 is Szczepanski, and Hyakutake is C/1996 B2. The two
        perihelia are 1996 and 1996, so dates cannot catch that -- only the NAMES can,
        which is why an entry from this page carries name_guard and build() refuses a
        join whose names share no identity word. Same policy as parse.sbdb_identity_ok().
      * Halley's apparitions are 27 rows under designations MPC never adopted ('P/1066
        G1'), all the SAME nucleus repeating 11 km. They join nothing, which is the right
        answer rather than a gap: the corpus authors one Halley, at 1P's elements.
    """
    if not os.path.exists(GREAT_HTML):
        return []
    text = open(GREAT_HTML, encoding='utf-8', errors='replace').read()
    return _great_table(text)


_GREAT_HDR = ('Designation', 'Name', 'Dimensions', 'Perihelion')


def _cell_text(raw):
    s = re.sub(r'<sup[^>]*>.*?</sup>', '', raw, flags=re.S)   # footnote markers
    s = re.sub(r'<[^>]+>', '', s)
    return re.sub(r'\s+', ' ', s).strip()


def _table_cells(row_html):
    """[cells] The d and h cells of one table row, tags stripped."""
    return [_cell_text(m.group(2))
            for m in re.finditer(r'<t([dh])[^>]*>(.*?)</t\1>', row_html, re.S)]


def _great_table(text):
    """[rows] Parse the table, NOT its index: the article reorders and adds tables."""
    for tab in re.findall(r'<table[^>]*>.*?</table>', text, re.S):
        rows = re.findall(r'<tr[^>]*>(.*?)</tr>', tab, re.S)
        col = {}
        for raw in rows:
            hdr = _table_cells(raw)
            want = [next((i for i, h in enumerate(hdr) if h.startswith(k)), None)
                    for k in _GREAT_HDR]
            if all(v is not None for v in want):
                col = dict(zip(('desig', 'name', 'dim', 'peri'), want))
                break
        if not col:
            continue
        out = []
        for raw in rows:
            c = _table_cells(raw)
            if len(c) <= max(col.values()):
                continue
            # U+2212 MINUS SIGN, not a hyphen: every BC row on the page.
            desig = c[col['desig']].replace('\u2212', '-').strip()
            name = c[col['name']].strip()
            # '20 - 25' is a stated span (the size page's own convention), not a value with
            # a stray dash; '8.6 km[13]' keeps its number after the tag strip above.
            nums = [float(x) for x in re.findall(r'\d+(?:\.\d+)?', c[col['dim']])]
            if not nums or desig.startswith('Designation'):
                continue
            diam_km = sum(nums) / len(nums)
            out.append(dict(
                name=('%s (%s)' % (desig, name)) if name else desig,
                designation=desig,
                wiki_name=name,
                radius_m=diam_km * 500.0,      # a DIAMETER, so half of it in metres
                radius_text=c[col['dim']],
                mass_kg=None,
                mass_text=None,
                table=0,
                type_note='great comet list (Dimensions is a diameter)',
                radius_src='Wikipedia Great comet (halved: the column is a diameter, %s)'
                           % c[col['dim']],
                name_guard=True,
                peri_text=c[col['peri']],
            ))
        return out
    return []


def sized_rows(verbose=True):
    """[dict] Every page we hold that states a nucleus size, deduplicated by the MPC
    identity the row resolves to -- not by the page's spelling, which is why the dedup runs
    after the alias pass. The by-size page wins: it states a radius outright, while the
    great-comet page states a diameter and several of its Dimensions cells are the article's
    own rough figures.

    The loser still gets to object. A size row states no perihelion date, so it cannot be
    date-checked against the MPC apparition build() picks; the page that lost the dedup may
    have one, and build() prints the disagreement as a NOTE rather than letting two pages
    place the same comet 3,000 years apart in silence."""
    out, seen, kept = [], {}, {}
    for e in radius_rows() + great_rows():
        key = resolve_key(e)
        if key in seen:
            if verbose:
                print('  size source dedup: %s also sized by %s (kept %s)'
                      % (key, e.get('radius_src', '')[:40], seen[key]))
            if e.get('peri_text') and not kept[key].get('peri_text'):
                kept[key]['other_peri_text'] = e['peri_text']
            continue
        seen[key] = ('size page' if not e.get('name_guard') else 'great comet page')
        kept[key] = e
        out.append(e)
    return out


def resolve_key(e):
    """[str] Which MPC identity a size row means: the page's own designation when it is
    shaped like one, else COMET_ALIASES. The alias step has to run before anything
    deduplicates, because the size page writes 'Great Comet of 1811' where the great-comet
    page writes 'C/1811 F1'."""
    desig = (e.get('designation') or '').strip()
    if desig and (_ID_NUM.match(desig) or _ID_LETTER.match(desig)):
        return id_key(desig)
    return COMET_ALIASES.get(e['name'], id_key(desig) if desig else e['name'])


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
    sized = sized_rows(verbose=verbose)   # one parse per page, not one per call site
    by_key = {}
    for r in rows:
        by_key.setdefault(id_key(r['Designation_and_name']), []).append(r)

    out = []
    refused = []
    noted = []
    for e in sorted(sized, key=lambda e: e['name']):
        display = e['name']
        if display in REFUSED:
            refused.append((display, REFUSED[display]))
            continue
        key = resolve_key(e)
        cands = by_key.get(key)
        if not cands:
            refused.append((display, 'no MPC orbit row for identity %r (%s says %r)'
                            % (key, 'great comet page' if e.get('name_guard')
                               else 'size page', e['designation'] or display)))
            continue
        r = pick_apparition(cands)
        orbit_name = r['Designation_and_name']
        # A page that lost the size dedup may still date the apparition differently. That is
        # not a refusal -- the MPC row IS the rail, and its date sets the epoch phase -- but
        # it gets printed. For C/1811 F1 the page says 1811 and MPC's row says +5339, so the
        # phase at our catalogue epoch is an extrapolation of 3,339 years along what the row
        # itself calls a 2,944-year orbit.
        other = e.get('other_peri_text')
        if other:
            oyr = re.search(r'\d{3,4}', other)
            if oyr and abs(int(oyr.group(0)) - int(r['Year_of_perihelion'])) > 1:
                noted.append((display, 'the page that also sizes it dates perihelion %s while '
                                       'MPC\'s %s is %+04d (%d years apart): the rail follows '
                                       'MPC'
                              % (other, key, int(r['Year_of_perihelion']),
                                 abs(int(oyr.group(0)) - int(r['Year_of_perihelion'])))))
        # The page's Designation column is editorial, not the MPC's. Two comets discovered
        # in the same half-month share a year and a number space, so the ONLY thing that
        # catches 'C/1996 B1 = Hyakutake' (MPC's C/1996 B1 is Szczepanski) is the name.
        # It only bites when both sides carry an identity word: name_tokens keeps letters,
        # not digits, so "Great Comet of 1843" reduces to nothing and the date test below is
        # the only one running for rows shaped like that.
        if e.get('name_guard'):
            mine, theirs = name_tokens(e.get('wiki_name')), name_tokens(proper_name(orbit_name))
            if mine and theirs and not mine & theirs:
                refused.append((display, 'the page writes %r where MPC\'s %s is %r'
                                % (e.get('wiki_name'), key, proper_name(orbit_name))))
                continue
            # And the page's own perihelion date has to be the apparition we railed on.
            yr = re.search(r'\d{3,4}', e.get('peri_text') or '')
            if yr and abs(int(yr.group(0)) - int(r['Year_of_perihelion'])) > 1:
                refused.append((display, 'the page dates perihelion %s, MPC\'s %s is %+04d'
                                % (yr.group(0), key, int(r['Year_of_perihelion']))))
                continue
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
        out.append(dict(
            name=display,
            desig=orbit_name,
            key=id_key(orbit_name),
            orbit_type=r.get('Orbit_type'),
            family=family(r.get('Orbit_type'), a, period_y, q),
            radius_m=e['radius_m'],
            radius_text=e['radius_text'],
            radius_src=e.get('radius_src')
                       or 'Wikipedia List of Solar System objects by size, table %d' % e['table'],
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
        for name, why in noted:
            print('  NOTE %s: %s' % (name, why))
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
