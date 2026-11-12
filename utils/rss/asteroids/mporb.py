#!/usr/bin/env python3
"""Pull the useful slice out of the MPCORB extended JSON.

Source: https://minorplanetcenter.net/Extended_Files/mpcorb_extended.json.gz
1.58M records, 175 MB gzipped / 983 MB as text, reissued daily. The plain MPCORB.DAT
fixed-width file carries the same orbits, but the extended JSON also carries H, G, the
uncertainty parameter U, the observation count and arc, the perihelion/apoapsis
distance, the perihelion passage time and a dynamical class ("MBA", "Jupiter Trojan",
"Distant Object") -- and its epoch is a Julian Date rather than a packed code.

It is far too big for git, so this module's job is to extract only what the corpus
answers to and let parse.py fold that into asteroids.json, which IS committed. The
blob stays in raw/mpcorb/ and is gitignored.

Two things the extraction is careful about:
  * Streaming. The file is one huge JSON array; loading it takes ~4 GB of RAM, which a
    laptop on a train (the stated reason raw/ exists) does not have. So records are
    cut by brace depth and parsed one at a time.
  * Identity. MPCORB numbers 1.58M objects and names ~26k of them, so a substring or a
    case-folded collision is not hypothetical -- (52) Europa and (9) Metis already
    collide with moons in the planetary catalogue, and comets carry the same
    designations twice. A record is accepted only on an exact number, or an exact
    principal/other designation, or an exact name.
"""
import gzip
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "raw")
MPCORB_DIR = os.path.join(RAW, "mpcorb")
MPCORB_GZ = os.path.join(MPCORB_DIR, "mpcorb_extended.json.gz")

# What we keep. Orbits for the rail, H for the size, the quality codes so a weak orbit
# can be refused, and the dynamical class -- which is the only classification the
# corpus has for the two thirds of its rows that state no spectral type.
KEEP = ("Number", "Name", "Principal_desig", "Other_desigs",
        "H", "G", "U", "Num_obs", "Num_opps", "Arc_years", "rms",
        "Orbit_type", "Last_obs", "Computer")

# MPCORB's osculating elements, renamed to the keys asteroids.json already uses for its
# SBDB blocks -- one shape for the generator to read out of either source. Note that
# MPCORB's "Peri" is the ARGUMENT of perihelion (omega), not the perihelion distance;
# that distance is "Perihelion_dist".
ELEMENTS = (("a", "a_au"), ("e", "e"), ("i", "i_deg"), ("Node", "node_deg"),
            ("Peri", "argperi_deg"), ("M", "mean_anom_deg"),
            ("n", "mean_motion_deg_d"), ("Epoch", "epoch_jd"),
            ("Orbital_period", "period_d"), ("Perihelion_dist", "q_au"),
            ("Aphelion_dist", "Q_au"), ("Tp", "tp_jd"))

_NUM_RE = re.compile(r'^\(\s*(\d+)\s*\)$')


def structural_depth(line):
    """(+1/-1 per) unmatched braces outside double-quoted strings.

    MPCORB values do not contain braces today, but a designator or a reference that did
    would silently truncate a record -- and a truncated record parsed as JSON would
    raise, not mislead. Counting only structural braces keeps that a parse error.
    """
    depth = 0
    in_str = False
    esc = False
    for ch in line:
        if esc:
            esc = False
            continue
        if ch == '\\':
            esc = True
            continue
        if ch == '"':
            in_str = not in_str
            continue
        if in_str:
            continue
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
    return depth


def iter_records(path=MPCORB_GZ):
    """[dict] One MPCORB record at a time; the whole file never sits in memory."""
    if not os.path.exists(path):
        raise SystemExit("mporb: %s is missing -- run fetch.py first" % path)
    depth = 0
    buf = []
    with gzip.open(path, "rt", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if depth == 0:
                s = line.strip()
                if not s or s in ('[', ']', '],'):
                    continue
            d = structural_depth(line)
            if depth == 0 and '{' not in line:
                continue
            buf.append(line)
            depth += d
            if depth <= 0:
                txt = ''.join(buf).strip().rstrip(',')
                buf = []
                depth = 0
                if txt:
                    yield json.loads(txt)


def number_of(rec):
    m = _NUM_RE.match(str(rec.get("Number") or "").strip())
    return int(m.group(1)) if m else None


def keys_of(rec):
    """Every exact key this record answers to (numbers as int, text lowercased)."""
    out = set()
    n = number_of(rec)
    if n is not None:
        out.add(('n', n))
    for field in ("Principal_desig",):
        v = rec.get(field)
        if v:
            out.add(('d', ' '.join(str(v).lower().split())))
    for v in (rec.get("Other_desigs") or []):
        if v:
            out.add(('d', ' '.join(str(v).lower().split())))
    v = rec.get("Name")
    if v:
        out.add(('nm', ' '.join(str(v).lower().split())))
    return out


def slim(rec):
    """The committed projection of one MPCORB record: only the fields we use, with the
    elements under "elements" in the SBDB-shaped key names (see ELEMENTS)."""
    out = {}
    for k in KEEP:
        v = rec.get(k)
        if v is None or v == '' or v == []:
            continue
        if k == "Number":
            n = number_of(rec)
            if n is not None:
                out["number"] = n
            continue
        if k == "Other_desigs":
            out["aliases"] = list(v)
            continue
        out[k] = v
    el = {}
    for src, dst in ELEMENTS:
        v = rec.get(src)
        if v is not None and v != '':
            el[dst] = v
    if el:
        out["elements"] = el
    return out


def extract(wanted, path=MPCORB_GZ, bucket=None, bucket_cap=None):
    """({key: slim}, {bucket name: [slim]}) -- one streaming pass over the whole file.

    wanted : iterable of ('n', int) / ('d', str) / ('nm', str), exactly as keys_of yields
             them. Every matching record lands in the first dict under each key it
             answers to. NOTE that `wanted` is only a lookup list -- it does NOT make a
             record ineligible for a bucket, because in practice the caller looks up the
             rows it already has AND the candidate lists it mines for new ones. What makes
             a record a candidate is the bucket predicate.
    bucket : called as bucket(rec, keys) -- keys being keys_of(rec), which the pass has
                 already computed; returns a list of (NAME, rank) pairs, or None/[] .
                 Records in a bucket are kept even when nothing wanted them, and this is
                 the only way to find a body the caller has never heard of (brightest,
                 sungrazers, a named dynamical family...). rank=None means H, which is what
                 a size-oriented rule wants; an explicit rank lets a rule order on
                 something else -- a co-orbital search wants |a - 1| smallest, and ranking
                 that by brightness keeps the kilometre rocks and drops the minimoons. A
                 record MAY be in several buckets, which is how one record can be both
                 "biggest" and "closest to Earth's orbit" without the rules fighting.
    bucket_cap : int, or {name: int}. Inside a bucket the cap keeps the smallest rank, so a
                 heap never holds more than a rule's worth of candidates out of 1.58M.
    A record with no rank (no H and none given) ranks last. Results come back in rank order.
    """
    import heapq
    wanted = set(wanted)
    hits = {}
    heaps = {}
    caps = bucket_cap if isinstance(bucket_cap, dict) else {}
    seq = 0
    for rec in iter_records(path):
        keys = keys_of(rec)
        matched = keys & wanted
        b = bucket(rec, keys) if bucket else None
        if not matched and not b:
            continue
        s = slim(rec)
        if matched:
            for k in matched:
                hits[k] = s
        if not b:
            continue
        for name, rank in b:
            rank = rec.get("H") if rank is None else rank
            cap = caps.get(name, bucket_cap if isinstance(bucket_cap, int) else None)
            seq += 1
            entry = (rank if rank is not None else 99.0, seq, keys, s)
            heap = heaps.setdefault(name, [])
            if cap is None or len(heap) < cap:
                heapq.heappush(heap, entry)
            elif entry[0] < heap[0][0]:
                heapq.heapreplace(heap, entry)
    picks = {b: [s for _, _, _, s in sorted(heap, key=lambda e: (e[0], e[1]))]
             for b, heap in heaps.items()}
    return hits, picks


def _self_test():
    """The three ways this module goes silently wrong -- a bad brace cut, a sloppy
    identity key, a bucket that drops what it kept -- none need the 175 MB blob."""
    assert structural_depth('  { "a": "x{y", "b": 1 },') == 0
    assert structural_depth('{ "ref": "a } brace in a string"') == 1
    assert structural_depth('    {') == 1
    r = {"Number": "(2)", "Name": "Encke", "Principal_desig": "2P",
         "Other_desigs": ["1786 I", "1786 aa"], "H": 8.2, "a": 2.216,
         "Peri": 314.5, "e": 0.617, "Junk": "x"}
    assert number_of(r) == 2
    assert ("n", 2) in keys_of(r) and ("nm", "encke") in keys_of(r)
    assert ("d", "1786 i") in keys_of(r)
    s = slim(r)
    assert s["number"] == 2 and s["H"] == 8.2 and "Junk" not in s
    assert s["aliases"] == ["1786 I", "1786 aa"]
    assert s["elements"]["argperi_deg"] == 314.5 and s["elements"]["a_au"] == 2.216
    # A record with no Number must not become one, and a name that is also a moon's name
    # still has to key exactly -- (52) Europa and (9) Metis collide with our moons.
    assert number_of({"Number": ""}) is None
    assert ("nm", "europa") in keys_of({"Name": "Europa", "Principal_desig": "1979 S"})

    # And the extractor, on a three-record file: one wanted hit, and buckets that DO see
    # wanted records -- Ceres is wanted for its orbit and still the brightest thing here.
    import gzip
    import tempfile
    recs = [{"Number": "(1)", "Name": "Ceres", "Principal_desig": "1 Ceres",
             "H": 3.34, "a": 2.77, "e": 0.076},
            {"Number": "(4)", "Name": "Vesta", "Principal_desig": "4 Vesta",
             "H": 3.20, "a": 2.36, "e": 0.089},
            {"Number": "(9999)", "Principal_desig": "1999 XX1", "H": 18.0,
             "a": 2.4, "e": 0.1}]
    fd, tmp = tempfile.mkstemp(suffix=".json.gz", dir=tempfile.gettempdir())
    os.close(fd)
    try:
        with gzip.open(tmp, "wt", encoding="utf-8") as f:
            f.write("[\n" + ",\n".join(json.dumps(r) for r in recs) + "\n]\n")
        hits, picks = extract({('n', 1), ('nm', 'ceres')}, tmp,
                              bucket=lambda rec, keys: [('big', None),
                                                        ('close', abs(rec['a'] - 1.0))]
                              if (rec.get('H') or 99) < 10 else [],
                              bucket_cap={'big': 2, 'close': 1})
        assert hits[('n', 1)]['Name'] == 'Ceres' and hits[('nm', 'ceres')] is hits[('n', 1)]
        assert [s['Name'] for s in picks['big']] == ['Vesta', 'Ceres'], picks
        # 'close' ranks on the axis, not on brightness: Vesta (2.36 AU) beats Ceres
        # (2.77 AU), even though Ceres is the brighter of the two.
        assert [s['Name'] for s in picks['close']] == ['Vesta'], picks
        hits2, picks2 = extract({('n', 1)}, tmp,
                                bucket=lambda rec, keys: []
                                if ('n', 1) in keys else [('big', None)],
                                bucket_cap={'big': 2})
        label = lambda s: s.get('Name') or s['Principal_desig']
        assert [label(s) for s in picks2['big']] == ['Vesta', '1999 XX1'], picks2
    finally:
        os.remove(tmp)
    print("mporb self-test ok")


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        _self_test()
        sys.exit(0)
    n = 0
    first = None
    for r in iter_records():
        if first is None:
            first = r
        n += 1
    print('%d records; first = %s %s H %s a %s'
          % (n, first.get('Number'), first.get('Name'), first.get('H'), first.get('a')))
