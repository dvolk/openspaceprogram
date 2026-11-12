#!/usr/bin/env python3
"""Parse Wikipedia's "List of Solar System objects by size" into stated radii/masses.

Why a second wiki page: the exceptional-asteroids list carries no size at all for the
distant tail -- all 91 rows MPCORB classes "Distant Object" have no diameter, no
spectral class and only 4 an albedo -- and absolute magnitude alone cannot size them,
because the albedo spans 0.04 (a coal) to 0.96 (Eris' frost), a factor 5 in RADIUS.
This page states a radius per body, split by class, with the mass and a Type cell that
also carries the spectral class, binarity and even ring claims.

The row layout is: <a href="/wiki/38628_Huya" title="38628 Huya">Huya</a> then a
<small>38628</small>. The HREF is the only place the number and the name are reliably
separated -- stripping the tags concatenates them into "Huya38628" -- so the href is
what the keys come from.

Tables are per class and the RADIUS UNIT IS NOT THE SAME IN EVERY ONE: tables up to 5
are in km, table 6 is in metres. The unit is read from the header, never assumed.
"""
import html
import re

# Radius header cells read "Radius[note 2](km)" / "Radius[note 2](m)".
RADIUS_UNIT = re.compile(r'\(\s*(km|m)\s*\)', re.I)
MEANINGFUL_ROW = re.compile(r'<td', re.I)


def strip(cell):
    """Tag-free, entity-free text of one cell."""
    cell = re.sub(r'<style.*?</style>', '', cell, flags=re.S)
    cell = re.sub(r'<script.*?</script>', '', cell, flags=re.S)
    cell = re.sub(r'<[^>]+>', '', cell)
    return ' '.join(html.unescape(cell).split())


def href_key(cell):
    """(number, name, designation) from the body cell's wiki link.

    '/wiki/38628_Huya'   -> (38628, 'Huya', None)
    '/wiki/2010_JO179'   -> (None-ish number from <small>, '2010 JO179')
    '/wiki/95P_Chiron'   -> ('2060 or 95P' style bodies keep the anchor text)
    """
    m = re.search(r'<a[^>]*href="[^"]*?/wiki/([^"#]+)"[^>]*>(.*?)</a>', cell, re.S)
    if not m:
        t = strip(cell)
        return None, t, None
    title, text = m.group(1), strip(m.group(2))
    title = title.replace('_', ' ').strip()
    # The title is "<number> <name>" for numbered bodies, else the bare designation.
    # "2025 UC11" is a DESIGNATION, not number 2025: a leading 19xx-21xx followed by a
    # two-letter-plus-digits suffix is a provisional designation, and reading the year
    # as a number invents a body 2025 that does not exist.
    # "(574372) 2010 JO179" puts the number in parentheses, which is the page's way of
    # saying "numbered, but listed under its designation".
    number = None
    designation = title
    name = text
    mpar = re.match(r'^\(\s*(\d{1,7})\s*\)\s+(.*)$', title)
    mnum = re.match(r'^(\d{1,7})\s+(.*)$', title)
    if mpar:
        number, designation = int(mpar.group(1)), mpar.group(2).strip()
    elif mnum and not (1900 <= int(mnum.group(1)) <= 2100
                       and re.match(r'^[A-Z]{1,2}\d*$', mnum.group(2))):
        number, designation = int(mnum.group(1)), mnum.group(2).strip()
    elif re.match(r'^\d{4}\s+[A-Z]{1,2}\d*$', title):
        designation = title          # unnumbered: the designation IS the whole title
    else:
        mm = re.match(r'^([A-Z]?\d+[PCDX]?[/\-\s]*\d*)\s+(.*)$', title)
        if mm:            # cometary/periodic titles such as "95P Chiron"
            designation = mm.group(1).strip()
    # <small>NNNNN</small> carries the number for named bodies whose title lacks it.
    if number is None:
        s = re.search(r'<small[^>]*>\s*(\d{1,7})\s*</small>', cell)
        if s:
            number = int(s.group(1))
    return number, name, designation


def parse_radius(cell):
    """'398±12', '371+39−42', '≈ 375±75', '0.205 - 0.465', '–' -> (value, note).

    A range like '+39−42' is asymmetric: take the plain central number and keep the
    original text as the note, because the game needs one number and a reader needs to
    see what the page actually claimed.
    """
    t = strip(cell)
    if not t or t in ('–', '-', '—'):
        return None, None
    # "0.205 - 0.465" is a stated span with no central value: the midpoint is the only
    # defensible single number, where the first token would silently be the low end.
    span = re.match(r'^\s*(\d+(?:\.\d+)?)\s*[-\u2013]\s*(\d+(?:\.\d+)?)\s*$', t)
    if span:
        return (float(span.group(1)) + float(span.group(2))) / 2.0, t
    m = re.search(r'(\d+(?:\.\d+)?)', t)
    if not m:
        return None, None
    return float(m.group(1)), t


def parse_mass(cell):
    t = strip(cell)
    if not t or t in ('–', '-', '—'):
        return None, None
    m = re.search(r'(\d+(?:\.\d+)?)', t)
    return (float(m.group(1)), t) if m else (None, None)


def objects(html_text):
    """[dict] One entry per data row, with keys name, number, designation, radius_km,
    radius_text, mass_text, type_note, table, and the lookup keys it answers to."""
    out = []
    tables = re.findall(r'<table[^>]*>.*?</table>', html_text, re.S)
    for ti, table in enumerate(tables):
        head_rows = re.findall(r'<tr[^>]*>.*?</tr>', table, re.S)[:2]
        # The unit sits inline in most tables ("Radius[note 2](km)") but in a SECOND
        # header row in the first one -- and that first table is where the dwarf planets
        # live, so missing it silently drops Eris, Makemake and Haumea. Aligning by
        # column index does not work either: that table gives Radius TWO sub-columns
        # (km and Earth radii), so the unit row is offset from the label row. Reading the
        # header in document order and taking the first unit after the word "Radius"
        # handles both shapes.
        header_text = ' | '.join(strip(c)
                                 for tr in head_rows
                                 for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', tr, re.S))
        unit = None
        at = header_text.lower().find('adius')
        if at >= 0:
            u = RADIUS_UNIT.search(header_text, at)
            if u:
                unit = u.group(1).lower()
        if unit is None:
            continue                      # navboxes and the like
        for tr in re.findall(r'<tr[^>]*>.*?</tr>', table, re.S):
            if not MEANINGFUL_ROW.search(tr):
                continue
            cells = re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', tr, re.S)
            if len(cells) < 3:
                continue
            number, name, designation = href_key(cells[0])
            r, rtext = parse_radius(cells[2])
            if r is None:
                continue
            radius_m = r if unit == 'm' else r * 1000.0
            mass, mtext = (parse_mass(cells[3]) if len(cells) > 3 else (None, None))
            # The Type column is not at a fixed index (the dwarf-planet table has more
            # columns than the small-body tables), so scan everything past the radius.
            tnote = None
            for c in cells[3:]:
                s = strip(c)
                if s and ('asteroid' in s or 'centaur' in s or 'trojan' in s
                          or 'plutino' in s or 'cubewano' in s or 'SDO' in s
                          or 'moon' in s or 'comet' in s or 'dwarf' in s
                          or 'satellite' in s or 'transneptunian' in s
                          or 'scattered' in s or 'Kuiper' in s):
                    tnote = s
                    break
            out.append({'table': ti, 'number': number, 'name': name,
                        'designation': designation, 'radius_m': radius_m,
                        'radius_text': rtext, 'mass_1e18_kg': mass,
                        'mass_text': mtext, 'type_note': tnote})
    return out


def index(entries):
    """Everything an entry answers to: number, lowercase name, lowercase designation."""
    by_number, by_name = {}, {}
    for e in entries:
        if e['number'] is not None:
            by_number[e['number']] = e
        for key in (e['name'], e['designation']):
            if key:
                by_name.setdefault(' '.join(key.lower().split()), e)
    return by_number, by_name


# Table 0 is the planets and the large moons as well as the dwarf planets, and the page
# disambiguates them the wiki way: "Europa (moon)", "Mercury (planet)". Those entries are
# not minor planets, and joining one to a minor planet row would attach Titan's radius to
# an asteroid, so they are filtered out here rather than matched more cleverly later. The
# three entries the parenthetical misses (Moon, Umbriel, Ariel -- the page just states
# their type in a column the parser reads as prose) are named explicitly, because the game
# has an authored moon with each of those names.
NOT_A_MINOR_PLANET = re.compile(r'\((?:moon|planet|natural satellite)\)\s*$', re.I)
HOMONYMOUS_MOONS = {'moon', 'umbriel', 'ariel', 'sun'}


def is_minor_planet(e):
    """False for the page's planets and moons."""
    for s in (e['designation'], e['name']):
        if s and (NOT_A_MINOR_PLANET.search(s)
                  or ' '.join(s.lower().split()) in HOMONYMOUS_MOONS):
            return False
    return 'moon of' not in (e['type_note'] or '').lower()


# --- List_of_trans-Neptunian_objects -------------------------------------------------
# One 1,049-row table, and unlike the size page it carries an ORBIT as well as a size:
# Designation | Year | Site | Discoverer | D (km) | Class | a | e | i | q | Q | Remarks
# | Refs. The two header rows use colspans (Discovery spans three, Orbital description
# six) and D spans both rows, so a data row has 13 cells while the headers do not --
# which is why the columns below are named by position and checked, not guessed.
TNO_COLS = ('desig', 'year', 'site', 'discoverer', 'diam_km', 'class', 'a_au', 'e',
            'i_deg', 'q_au', 'Q_au', 'remarks', 'refs')
TNO_NCOLS = len(TNO_COLS)
_REF_MARK = re.compile(r'\[\d+\]')
_TNO_NUM = re.compile(r'^\(\s*(\d{1,7})\s*\)\s*(.*)$')
_TNO_NUM2 = re.compile(r'^(\d{1,7})\s+(.*)$')
# Remarks is a semicolon list of claims: "albedo: 0.074; BRmag: 1.24; taxonomy: BR",
# "binary: 213 km", "triple: 150 km, 350 km; rings", "contact binary".
_REMARK_KV = re.compile(r'([a-z_]+)\s*:\s*([^;]+)')


def tno_rows(html_text):
    """[dict] The TNO list as rows: designation, number, D (km), class, a/e/i/q/Q in AU
    and degrees, plus the Remarks claims pulled out (albedo, taxonomy, binary, rings).

    Every row here is unnumbered or numbered by its Designation cell; the number is what
    makes it joinable to MPCORB, which is where a real osculating rail would come from
    (this table has no node, argperi or mean anomaly, so its five numbers describe an
    orbit but do not fix one).
    """
    table = max(re.findall(r'<table[^>]*>.*?</table>', html_text, re.S), key=len)
    out = []
    for tr in re.findall(r'<tr[^>]*>.*?</tr>', table, re.S):
        if '<td' not in tr.lower():
            continue
        cells = [strip(c) for c in re.findall(r'<t[dh][^>]*>(.*?)</t[dh]>', tr, re.S)]
        if len(cells) != TNO_NCOLS:
            continue        # a colspan header row or a footer, not data
        row = dict(zip(TNO_COLS, cells))
        desig = _REF_MARK.sub('', row['desig']).strip()
        number, rest = None, desig
        m = _TNO_NUM.match(desig) or _TNO_NUM2.match(desig)
        if m:
            number, rest = int(m.group(1)), m.group(2).strip()
        row['number'] = number
        row['designation'] = rest or desig
        for k in ('diam_km', 'a_au', 'e', 'i_deg', 'q_au', 'Q_au'):
            t = _REF_MARK.sub('', row[k]).strip()
            row[k] = float(t) if re.match(r'^\d+(\.\d+)?$', t) else None
        rem = row['remarks']
        claims = {}
        for k, v in _REMARK_KV.findall(rem.lower()):
            claims.setdefault(k, v.strip())
        row['class'] = row['class'].lower().strip() or None
        row['albedo'] = _as_float(claims.get('albedo'))
        row['taxonomy'] = claims.get('taxonomy')
        row['binary'] = ('binary' in rem.lower() or 'triple' in rem.lower()
                         or 'contact binary' in rem.lower() or None)
        row['rings'] = 'rings' in rem.lower() or None
        out.append(row)
    return out


def _as_float(t):
    if t is None:
        return None
    m = re.search(r'\d+(\.\d+)?', t)
    return float(m.group(0)) if m else None


if __name__ == '__main__':
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else 'List_of_Solar_System_objects_by_size.html'
    text = open(path, encoding='utf-8').read()
    rows = tno_rows(text)        # shape-detected, not by page title: the wiki page spells
    if rows:                     # "Trans-Neptunian" with a non-breaking hyphen
        rows = tno_rows(text)
        print('%d TNO rows' % len(rows))
        n_d = sum(1 for r in rows if r['diam_km'] is not None)
        n_num = sum(1 for r in rows if r['number'] is not None)
        print('with a stated D: %d, numbered: %d' % (n_d, n_num))
        per = {}
        for r in rows:
            per[r['class']] = per.get(r['class'], 0) + 1
        print('classes:', sorted(per.items(), key=lambda kv: -kv[1])[:14])
        bad = [r for r in rows if r['a_au'] and r['q_au']
               and abs(r['a_au'] * (1 - (r['e'] or 0)) - r['q_au']) > 0.05]
        print('rows where a(1-e) != q: %d' % len(bad))
        for r in bad[:5]:
            print('   ', r['desig'], r['a_au'], r['e'], r['q_au'], r['Q_au'])
        print('biggest 12:')
        for r in sorted((r for r in rows if r['diam_km']), key=lambda r: -r['diam_km'])[:12]:
            print('  %-24s D %8.1f %-18s a %6.2f e %.3f i %6.2f alb %s'
                  % (r['desig'], r['diam_km'], r['class'], r['a_au'], r['e'], r['i_deg'],
                     r['albedo']))
        sys.exit(0)
    ents = objects(text)
    print('%d entries' % len(ents))
    per = {}
    for e in ents:
        per[e['table']] = per.get(e['table'], 0) + 1
    print('per table:', sorted(per.items()))
    for e in ents[:3] + ents[-3:]:
        print('  #%-7s %-22s r %10.0f m  %s' % (e['number'], e['name'], e['radius_m'],
                                                (e['type_note'] or '')[:40]))
