#!/usr/bin/env python3
"""Derive utils/rss/asteroids/asteroids.json from the cached raw sources; no network.

raw/exceptional_asteroids.html (Wikipedia "List of exceptional asteroids") carries the
PHYSICAL character -- diameter, triaxial dimensions, mass with its stated precision,
rotation period, spectral type, discovery -- raw/sbdb/<slug>.json (JPL Small-Body
Database) carries the RAIL -- osculating elements at an epoch, spkid, orbit class --
and raw/lc_*.csv (the Asteroid Lightcurve Database, see lcdb.py) carries the best
adopted PHYSICS: period with a reliability code, effective diameter with its source,
albedo, taxonomic class, H and G, binary status, family. Nothing has all three, and
SBDB has no physical parameters at all, so the merge is the point of this script.

Every field records where it came from ("src" per field), because the sources disagree
at the level the page does not warn about: the wiki's "Mean distance from Sun" is a
rounded semi-major axis, its "Diameter" column mixes geometric means with single
chords (see --check's DIAMETER notes), and its "Class" column is a Tholen SPECTRAL
class, not an orbit class. --check re-derives and compares, and cross-checks the wiki's
orbit rows against SBDB's.

Fields the sources disagree about are RESOLVED into one value the generator can read,
with the winner's provenance in "src" and the losers still sitting next to it:
  rotation_period_h  LCDB adopted period (reliability code U) else the wiki's
  albedo_pV          LCDB adopted albedo unless AlbSource is 'A' (assumed from class)
  density_kg_m3      mass over the best shape candidate
and one more is DERIVED, because the game wants a sphere and the best shape is an
ellipsoid:
  diam_eff_km        geometric mean of the wiki's three axes (equal-volume diameter)
"""
import argparse
import glob
import json
import math
import os
import re
import sys
import unicodedata
import warnings

import pandas as pd

import lcdb
import mporb      # MPCORB: H, dynamical class, orbit quality, candidate bodies
import sizes      # the two wiki tables that state a size where the first one is blank

warnings.filterwarnings("ignore")

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "raw")
WIKI_HTML = os.path.join(RAW, "exceptional_asteroids.html")
SBDB_DIR = os.path.join(RAW, "sbdb")
# Two more wiki tables, both there for one field the first page does not carry: a SIZE
# for the bodies the exceptional-asteroids list leaves size-less (see sizes.py).
SIZES_HTML = os.path.join(RAW, "objects_by_size.html")
TNO_HTML = os.path.join(RAW, "tno_list.html")
OUT = os.path.join(HERE, "asteroids.json")

# h2/h3 heading text -> short section key. Only these sections are read; the rest of
# the page (Timeline, Landmark names, References...) is prose or nav tables.
# 'Minor planets that were misnamed and renamed' is deliberately NOT here: its two
# columns are 'Minor planet name' and a paragraph of prose, and a row names two bodies
# ("525 Adelaide and 1171 Rusthawelia"), so it contributes stories, not bodies.
SECTIONS = {
    "Largest by diameter": "largest",
    "Most massive": "massive",
    "Brightest from Earth": "brightest",
    "Slowest rotators": "slow_rot",
    "Fastest rotators": "fast_rot",
    "Retrograde": "retrograde",
    "Highly inclined": "inclined",
    "Spacecraft targets": "visited",
    "Landmark asteroids": "landmark",
    "Numbered minor planets that are also comets": "comets",
}

# Rows that look like bodies but are not: the Kirkwood-gap rows in 'Largest by
# diameter' and the 'Total' row in 'Most massive'.
NOT_A_BODY = re.compile(r"resonance|^\s*Total\s*$|^\s*—\s*$")

# "(65803) Didymos" / "1 Ceres" / "2024 TF3". A leading 4-digit year followed by a
# provisional suffix is NOT a number: 2014 RC is unnumbered, 846 Lipperta is numbered.
# The provisional suffix is [A-Z]{1,2} plus AT MOST two digits, and it has to be
# excluded before the number rule, not after: the page's "2010 EQ169" has three digits,
# which is not a valid suffix either, so treating its year as a permanent number is how
# it ends up matching asteroid 2010 Chebyshev.
PAREN_DES = re.compile(r"^\(\s*(\d+)\s*\)\s*(.*)$")
PLAIN_DES = re.compile(r"^(\d+)\s+(\S.*)$")
PROVISIONAL = re.compile(r"^\d{4}\s+[A-Z]{1,2}\d*$")


def clean(s):
    """Normalise a raw wiki cell: unicode dashes/nbsp, footnote refs, whitespace."""
    if s is None:
        return None
    if isinstance(s, float) and pd.isna(s):
        return None
    s = str(s)
    s = s.replace("\u00a0", " ").replace("\u2009", " ").replace("\u2212", "-")
    s = s.replace("\u00b1", " +- ").replace("\u2248", "~").replace("\u00b0", " deg ")
    s = re.sub(r"\[\d+\]", "", s)
    s = re.sub(r"\[citation needed\]", "", s)
    s = re.sub(r"\s+", " ", s).strip()
    return s or None


def num(s):
    """(value, uncertainty, approx, note) from a cell like '939.4 +- 0.2 (if bilobe)'.

    Takes the FIRST number in the cell and an immediately following '+- x'; anything
    else in the cell survives as `note`, so a qualifier is never silently dropped.
    """
    s = clean(s)
    if s is None or s in ("n.a.", "n/a", "--", "-"):
        return None, None, False, None
    approx = s.startswith("~")
    m = re.search(r"([-+]?\d[\d, ]*(?:\.\d+)?(?:[eE][-+]?\d+)?)", s)
    if not m:
        return None, None, approx, s
    value = float(m.group(1).replace(",", "").replace(" ", ""))
    unc = None
    um = re.search(r"\+-\s*(\d[\d, .]*(?:[eE][-+]?\d+)?)", s[m.end():])
    if um:
        unc = float(um.group(1).replace(",", "").replace(" ", ""))
    note = s[m.end():].strip() or None
    return value, unc, approx, note


def dims(s):
    """'572.6 x 557.2 x 446.4 +- 0.2' -> ([572.6, 557.2, 446.4], 0.2, None).

    Three things the first version got wrong. It threw away the uncertainty the last
    axis carries (these numbers feed diam_eff_km and the derived density); one
    unreadable part ('225x173x?') lost all three axes; and it read 87 Sylvia's cell
    '363x249x191 or 374x248x194' as five axes of one body, when it is two alternative
    shape fits for the same body. Now: first alternative kept, everything else survives
    as a note."""
    s = clean(s)
    if s is None:
        return None, None, None
    alts = re.split(r"\s+\bor\b\s+", s)
    parts = re.split(r"[x\u00d7]", alts[0])
    vals, unc, dropped = [], None, 0
    for p in parts:
        n, u, _approx, _note = num(p)
        if n is None:
            dropped += 1
            continue
        vals.append(n)
        if u is not None:
            unc = u
    if len(vals) < 2:
        return None, None, None
    notes = []
    if dropped:
        notes.append("%d of %d axes unreadable" % (dropped, len(parts)))
    if len(vals) > 3:
        notes.append("cell lists %d axes, kept the first three" % len(vals))
        vals = vals[:3]
    if len(alts) > 1:
        notes.append("cell also offers %s" % "; ".join(a.strip() for a in alts[1:]))
    return vals, unc, " -- ".join(notes) if notes else None


def pct(s):
    """'39.2%' -> (0.392, False); '~ 1%' -> (0.01, True). The '~' survives because the
    column is 'Approx. proportion of all asteroids' -- a share of an estimate."""
    s = clean(s)
    if s is None or "%" not in s:
        return None, False
    v, _u, approx, _n = num(s)
    return (None, approx) if v is None else (v / 100.0, approx)


# A satellite label: '243 Ida I Dactyl', '152830 Dinkinesh I Selam'. The MPC numbers a
# moon's PRIMARY, so taking the leading number here files the primary's orbit and
# diameter under the moon's name -- and gives the dataset two records with one spkid.
SATELLITE = re.compile(r"\s+[IVX]+\s+\S+$|\s+[IVX]+$")


def desig(s):
    """(number or None, name) from a wiki designation cell."""
    s = clean(s)
    if s is None:
        return None, None
    if SATELLITE.search(s):
        return None, s
    m = PAREN_DES.match(s)
    if m:
        return int(m.group(1)), (m.group(2).strip() or None)
    m = PLAIN_DES.match(s)
    if m and not PROVISIONAL.match(s):
        return int(m.group(1)), m.group(2).strip()
    return None, s


def slug(query):
    """Cache filename stem for a query string: '1 Ceres' -> '1_ceres'.

    Transliterated first: '2867 Šteins' otherwise slugs to '2867_teins' with the
    vowel simply gone, which reads like a typo and could collide with a real name.
    """
    s = unicodedata.normalize("NFKD", query or "?")
    s = "".join(c for c in s if not unicodedata.combining(c))
    s = s.replace("(", "").replace(")", "")
    s = re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_").lower()
    return s


def flatten(df):
    """One-level column names; for a split header keep the informative part."""
    cols = []
    for c in df.columns:
        if isinstance(c, tuple):
            a, b = (str(x) for x in c)
            cols.append(b if a == b or a in ("", "nan") else "%s %s" % (a, b))
        else:
            cols.append(str(c))
    df = df.copy()
    df.columns = cols
    return df


def wiki_sections(html):
    """[(heading, slice-of-html)] split on h2/h3, so each table is read under the
    heading that names it instead of by table index (which shifts when the page does)."""
    marks = [(m.start(), re.sub(r"<[^>]+>", "", m.group(0)).strip())
             for m in re.finditer(r"<h[23][^>]*>.*?</h[23]>", html)]
    out = []
    for i, (pos, title) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else len(html)
        out.append((title, html[pos:end]))
    return out


def cell(row, *names):
    """First present column among `names` (prefix match, so 'Semi- major axis (AU)'
    survives the page's own hyphenation)."""
    for want in names:
        for c in row.index:
            if re.sub(r"\s+", " ", str(c)).strip().lower().startswith(want.lower()):
                v = row[c]
                if isinstance(v, float) and pd.isna(v):
                    continue
                return v
    return None


def wiki_rows(html):
    """{slug: record} merged over every section, each field tagged with its section."""
    bodies = {}
    for title, chunk in wiki_sections(html):
        key = SECTIONS.get(title)
        if key is None:
            continue
        for df in pd.read_html(chunk if "<table" in chunk else "<table>" + chunk):
            for _, raw in flatten(df).iterrows():
                label = cell(raw, "Name", "Asteroid", "Minor planet designation")
                label = clean(label)
                if label is None or NOT_A_BODY.search(label):
                    continue
                number, name = desig(label)
                q = label
                s = slug(q)
                rec = bodies.setdefault(s, {
                    "query": q, "name": name or label, "number": number,
                    "sections": [], "fields": {},
                })
                if key not in rec["sections"]:
                    rec["sections"].append(key)
                put = rec["fields"].setdefault   # first section to state a field wins

                dcell = cell(raw, "Diameter (km) (geometric mean)")
                if clean(dcell) is None:
                    dcell = cell(raw, "Diameter (km)", "Diameter")
                v, unc, approx, note = num(dcell)
                if v is not None:
                    put("diameter_km", {"value": v, "unc": unc, "approx": approx,
                                        "note": note, "src": key})
                d, dunc, dnote = dims(cell(raw, "Dimensions (km)"))
                src = key
                if not d:
                    # The 'Spacecraft targets' table has no Dimensions column and writes
                    # the ellipsoid straight into the Diameter cell ("56 ×24×21" for Ida).
                    # Reading only the first number there is what makes the page look like
                    # it quotes a longest axis -- it quotes the same axes as everywhere.
                    d, dunc, dnote = dims(clean(dcell))
                    src = key + " (diameter cell)"
                if d:
                    put("dimensions_km", {"value": d, "unc": dunc, "note": dnote,
                                          "src": src})
                m, munc, mapp, mnote = num(cell(raw, "Mass"))
                if m is not None:
                    # 'Most massive' states its own Precision column -- '0.0004%
                    # (259.075-259.077)', '27% (19-34)', and literally '?' for 532
                    # Herculina. That column is the only thing saying how much a mass is
                    # worth, so it has to travel with the number.
                    prec = clean(cell(raw, "Precision"))
                    put("mass_1e18_kg", {"value": m, "unc": munc, "approx": mapp,
                                         "note": mnote, "precision": prec, "src": key})
                p, papp = pct(cell(raw, "Approx. proportion of all asteroids"))
                if p is not None:
                    put("mass_share", {"value": p, "approx": papp, "src": key})
                rot = num(cell(raw, "Rotation period (hours)", "Rotation period hours"))
                if rot[0] is None:
                    rot = num(cell(raw, "Rotation period"))
                if rot[0] is not None:
                    put("wiki_rotation_h", {"value": rot[0], "unc": rot[1],
                                           "note": rot[3], "src": key})
                spec = clean(cell(raw, "Spectral type", "Class"))
                if spec:
                    put("spectral_type", {"value": spec, "src": key})
                h = num(cell(raw, "Abs. mag (H)", "Absolute magnitude"))
                if h[0] is not None:
                    put("h_mag", {"value": h[0], "note": h[3], "src": key})
                inc = num(cell(raw, "Inclination"))
                if inc[0] is not None:
                    put("wiki_inclination_deg", {"value": inc[0], "src": key})
                sma = num(cell(raw, "Semi- major axis (AU)", "Semi-major axis",
                               "Mean distance from Sun"))
                if sma[0] is not None:
                    put("wiki_sma_au", {"value": sma[0], "src": key})
                ecc = num(cell(raw, "Eccentricity"))
                if ecc[0] is not None:
                    put("wiki_ecc", {"value": ecc[0], "src": key})
                mag = num(cell(raw, "Magnitude when brightest"))
                if mag[0] is not None:
                    put("best_mag", {"value": mag[0], "note": mag[3], "src": key})
                craft = clean(cell(raw, "Spacecraft"))
                if craft:
                    put("spacecraft", {"value": craft,
                                       "years": clean(cell(raw, "Year(s)")),
                                       "closest_km": num(cell(raw, "Closest approach (km)"))[0],
                                       "closest_radii": num(cell(raw, "Closest approach (asteroid radii)"))[0],
                                       "notes": clean(cell(raw, "Notes")),
                                       "landmarks": clean(cell(raw, "Landmark")),
                                       "src": key})
                cmt = clean(cell(raw, "Comment"))
                if cmt and cmt != "\u2014":
                    rec.setdefault("comments", []).append("%s: %s" % (key, cmt))
    # A configured section that yields nothing means the page renamed a heading or
    # changed a label column, and the corpus would quietly shrink. Same rule as the
    # planet generator: structure is an assertion, a data conflict is a note.
    seen = {k for r in bodies.values() for k in r["sections"]}
    assert seen == set(SECTIONS.values()), \
        "wiki: sections produced no bodies: %s" % sorted(set(SECTIONS.values()) - seen)
    return bodies


def wiki_objects(html):
    """The bodies the page names, as SBDB query strings (fetch.py's work list)."""
    rows = wiki_rows(html)
    return [{"query": r["query"], "name": r["name"], "number": r["number"],
             "sections": r["sections"]}
            for _, r in sorted(rows.items())]


def sbdb_records():
    """{slug: parsed SBDB orbit/object/discovery} from raw/sbdb/*.json."""
    out = {}
    for path in sorted(glob.glob(os.path.join(SBDB_DIR, "*.json"))):
        stem = os.path.basename(path)[:-5]
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        orb = d.get("orbit", {})
        el = {e["name"]: e for e in orb.get("elements", [])}

        def ev(name):
            e = el.get(name)
            return float(e["value"]) if e and e.get("value") not in (None, "") else None

        obj = d.get("object", {})
        disc = d.get("discovery", {})
        out[stem] = {
            "shortname": obj.get("shortname"), "fullname": obj.get("fullname"),
            "spkid": obj.get("spkid"), "des": obj.get("des"),
            "kind": obj.get("kind"), "neo": obj.get("neo"), "pha": obj.get("pha"),
            "orbit_class": (obj.get("orbit_class") or {}).get("name"),
            "orbit_class_code": (obj.get("orbit_class") or {}).get("code"),
            "a_au": ev("a"), "e": ev("e"), "q_au": ev("q"), "Q_au": ev("ad"),
            "i_deg": ev("i"), "node_deg": ev("om"), "argperi_deg": ev("w"),
            "mean_anom_deg": ev("ma"), "tp_jd_tdb": ev("tp"),
            "period_d": ev("per"), "mean_motion_deg_d": ev("n"),
            "epoch_jd": float(orb["epoch"]) if orb.get("epoch") else None,
            "equinox": orb.get("equinox"), "orbit_id": orb.get("orbit_id"),
            "soln_date": orb.get("soln_date"), "first_obs": orb.get("first_obs"),
            "last_obs": orb.get("last_obs"),
            "data_arc_d": float(orb["data_arc"]) if orb.get("data_arc") else None,
            "condition_code": orb.get("condition_code"),
            "moid_au": float(orb["moid"]) if orb.get("moid") else None,
            "moid_jup_au": float(orb["moid_jup"]) if orb.get("moid_jup") else None,
            "discovery": {k: disc.get(k) for k in ("who", "location", "date", "citation")}
                         if disc else None,
        }
    return out


def albedo_from(h_mag, d_km):
    """Geometric albedo p_V from absolute magnitude and diameter (the standard
    relation, inverted): D = 1329 km / sqrt(p) * 10^(-H/5)."""
    if h_mag is None or not d_km:
        return None
    p = (1329.0 / (d_km * 10.0 ** (h_mag / 5.0))) ** 2
    return p if 0.0 < p <= 1.0 else None


# Nothing in this corpus has a believable bulk density outside this band: ice-rich
# bodies sit near 1000 kg/m3, iron near 5000. Outside it, the mass or the shape is
# wrong rather than the body exotic -- 532 Herculina's '?'-precision mass over the
# wiki's 168 km cell gives 9264, denser than osmium.
DENSITY_BAND = (900.0, 5500.0)


def density_from(mass_1e18, diam_km):
    """Bulk density [kg/m3] from mass and an effective diameter.

    Takes a diameter rather than a shape because shape_candidates() has already turned
    every shape on offer into its equal-volume diameter -- an ellipsoid's volume is
    (4/3)pi(a/2)(b/2)(c/2), which is exactly what the geometric mean gives."""
    if mass_1e18 is None or not diam_km:
        return None
    vol = 4.0 / 3.0 * math.pi * (diam_km * 1000.0 / 2.0) ** 3
    return mass_1e18 * 1e18 / vol if vol > 0 else None


def triaxial_eff(r):
    """Effective diameter of a triaxial row: the geometric mean of the three axes is
    the diameter of the equal-volume sphere. None when the row has fewer than 3 axes
    (Hektor's 403x201 bilobe, Castalia's 1.8x0.8), which is itself a gap worth seeing."""
    tri = (r.get("dimensions_km") or {}).get("value")
    return (math.prod(tri[:3]) ** (1.0 / 3.0)) if tri and len(tri) >= 3 else None


def shape_candidates(f, lc):
    """[(label, effective diameter km)] for every shape the sources give, best first.

    Order is not a tie-break, it is physics: a triaxial row comes from a shape model,
    radar or an occultation chord set, and its geometric mean IS the equal-volume
    diameter. The wiki's single 'Diameter' cell is often the LONGEST axis of that same
    ellipsoid (Ida: 56 in the cell, 56x24x21 in Dimensions), which overstates the volume
    by up to 8x. LCDB's adopted value is only as good as its DiamSource flag, so it
    comes last unless nothing better exists.
    """
    out = []
    eff = triaxial_eff(f)
    if eff is not None:
        out.append(("DERIVED geometric mean of triaxial %s"
                    % "x".join("%g" % x for x in f["dimensions_km"]["value"][:3]), eff))
    d = (f.get("diameter_km") or {}).get("value")
    if d:
        out.append(("wiki Diameter cell (%s)" % (f["diameter_km"].get("src") or "?"), d))
    if lc.get("diam_eff_km"):
        out.append(("LCDB adopted, DiamSource %s" % lc.get("diam_src", "?"),
                    lc["diam_eff_km"]))
    return out


def rotation_resolved(f, lc):
    """Rotation period [h], preferring LCDB: it covers 144 of these 259 bodies where the
    page covers fewer than a third, and unlike the page it says how much to trust the
    number -- U is the reliability code (3 = one unambiguous lightcurve, 2+ and below =
    slipping or merged solutions) and PFlag qualifies it ('S' sidereal, default synodic,
    '<'/'>' a bound).

    Sidereal is what a spin rate wants and LCDB flags which one it is; for these bodies
    the synodic/sidereal difference is far below the rounding, so it is carried as a
    note rather than corrected."""
    p = lc.get("period_h")
    if p:
        src = "LCDB adopted (U %s" % lc.get("period_quality", "?")
        if lc.get("period_flag"):
            src += ", flag %s" % lc["period_flag"]
        return {"value": p, "src": src + ")"}
    w = f.get("wiki_rotation_h") or {}
    if w.get("value"):
        return {"value": w["value"], "src": "wiki %s table" % w.get("src", "?")}
    return None


def sbdb_identity_ok(rec, obj):
    """Does the cached SBDB object actually answer the query we asked?

    SBDB parses a malformed designation by falling back to its leading number: the page's
    '2010 EQ169' -- a provisional suffix never has three digits, and the row is flagged
    citation needed -- came back as 2010 Chebyshev (1969 TL4), a main-belter with i=2.4
    filed under a retrograde row. The cache file is named for the WIKI row, so without
    this check a wrong orbit sits under the right name forever. fullname carries both the
    permanent number and the designation (612078 (1998 UQ1)), which is what makes it the
    thing to test rather than des."""
    fullname = (obj.get("fullname") or "").strip()
    des = (obj.get("des") or "").strip()
    n = rec.get("number")
    if n is not None:
        # Compare NUMBERS, not substrings: a substring test clears the wrong object,
        # because 253 is "found" inside "(2015 TG253)" and 7 inside "(1999 RQ7)".
        for s in (fullname, des):
            m = re.match(r"^\(?\s*(\d+)", s)
            if m and int(m.group(1)) == n:
                return True
    blob = (fullname + " " + des).lower()
    for s in (rec.get("query"), rec.get("name")):
        s = (s or "").strip().lower()
        if s and s in blob:
            return True
    return False


def merge():
    """asteroids.json: one record per body, wiki fields + SBDB rail + LCDB physics."""
    bodies = wiki_rows(open(WIKI_HTML, encoding="utf-8").read())
    sbdb = sbdb_records()
    if lcdb.have_tables():
        lidx, lbidx = lcdb.load()
    else:
        lidx, lbidx = None, None
        print("note    no LCDB tables in %s -- physical fields fall back to the wiki"
              " (see urls.txt: the archive has to be fetched by hand)" % RAW)
    records = []
    join_notes = []
    reject_notes = []
    for s, rec in sorted(bodies.items(), key=lambda kv: (-1 if kv[1]["number"] is None
                                                         else kv[1]["number"], kv[0])):
        out = {"name": rec["name"], "number": rec["number"], "query": rec["query"],
               "sections": rec["sections"]}
        if rec.get("comments"):
            out["comments"] = rec["comments"]
        f = rec["fields"]
        out.update(f)          # every wiki field, verbatim with its own "src"
        if "mass_1e18_kg" in f:
            out["mass_kg"] = {"value": f["mass_1e18_kg"]["value"] * 1e18,
                              "src": f["mass_1e18_kg"]["src"]}
        lc = {}
        if lidx is not None:
            row, note = lcdb.lookup(lidx, rec)
            if note:
                join_notes.append("%s: %s" % (rec["name"], note))
            if row:
                lc = lcdb.adopted(row)
                out["lcdb"] = lc
                poles = lcdb.poles_for(lbidx["lc_spinaxis.csv"], rec)
                if poles:
                    out["lcdb_poles"] = poles
                moons = lcdb.binaries_for(lbidx["lc_binary.csv"], rec)
                if moons:
                    out["lcdb_binaries"] = moons
                npa = lcdb.npa_for(lbidx["lc_npa.csv"], rec)
                if npa:
                    out["lcdb_tumbling"] = npa
        shapes = shape_candidates(f, lc)
        if shapes and shapes[0][0].startswith("DERIVED"):
            out["diam_eff_km"] = {"value": shapes[0][1], "src": shapes[0][0]}
        rot = rotation_resolved(f, lc)
        if rot:
            out["rotation_period_h"] = rot

        # Albedo: LCDB has one for nearly everything, but AlbSource 'A' means the class
        # was assumed, so it cannot beat the two-step H + D arithmetic -- UNLESS the
        # diameter we would divide by is LCDB's own back-computation of that assumed
        # albedo (DiamSource C/D), in which case the round trip just relabels the guess
        # as "DERIVED". Those rows keep the ASSUMED provenance instead.
        h = lc.get("h_mag")
        if h is None:
            h = (f.get("h_mag") or {}).get("value")
        p = lc.get("albedo_pV")
        circular = bool(shapes) and shapes[0][0].startswith("LCDB adopted") \
            and lc.get("diam_src") in ("C", "D")
        if p is not None and lc.get("alb_src") != "A":
            out["albedo_pV"] = {"value": p, "src": "LCDB adopted (AlbSource %s)"
                                % lc.get("alb_src", "?")}
        elif h is not None and shapes and not circular:
            p = albedo_from(h, shapes[0][1])
            if p is not None:
                out["albedo_pV"] = {"value": p,
                                    "src": "DERIVED from H + %s" % shapes[0][0]}
        elif p is not None:
            out["albedo_pV"] = {"value": p, "src": "LCDB adopted (AlbSource A: ASSUMED"
                                " from class %s -- treat as a placeholder)"
                                % lc.get("class", "?")}
        rho = density_from((f.get("mass_1e18_kg") or {}).get("value"),
                           shapes[0][1] if shapes else None)
        if rho is not None:
            if DENSITY_BAND[0] <= rho <= DENSITY_BAND[1]:
                out["density_kg_m3"] = {"value": rho,
                                        "src": "DERIVED from mass + %s"
                                               % (shapes[0][0] if shapes else "?")}
            else:
                reject_notes.append(
                    "%s: density %.0f kg/m3 is outside the believable band %g-%g, so the"
                    " mass or the shape is wrong -- no density authored (see SANITY)"
                    % (rec["name"], rho, DENSITY_BAND[0], DENSITY_BAND[1]))
        if s in sbdb:
            if sbdb_identity_ok(out, sbdb[s]):
                out["sbdb"] = sbdb[s]
            else:
                print("  IDENTITY %s: SBDB answered %r (%s) for this query -- no rail"
                      % (out["name"], sbdb[s].get("fullname"), sbdb[s].get("spkid")))
        records.append(out)
    for n in join_notes:
        print("  JOIN  %s" % n)
    for n in reject_notes:
        print("  REFUSE %s" % n)
    return records


# --- MPCORB, and the two wiki tables that state a size -------------------------------
# One streaming pass over 1.58M MPCORB records serves every row at once (mporb.py), which
# is also the only way it can be done. What the pass is FOR, in order of how much it buys:
#   H for the 111 rows that state no magnitude anywhere, which turns a cohort-median guess
#     at a size into arithmetic on the body's own brightness;
#   a dynamical class for the 115 rows that state no spectral type;
#   stated radii for the dwarf planets, the big Kuiper bodies, the Trojans and the centaurs
#     -- all of which the exceptional-asteroids page leaves size-less;
#   whole new rows for bodies the wiki page never listed (see CANDIDATE_BUCKETS).
# The one thing MPCORB is NOT used for is the rail of an existing row: 245 of the 255
# matched records carry the same epoch as our SBDB blocks, and where the two disagree the
# SBDB solution is the one already cross-checked against the wiki (urls.txt has the run).

# Names the planetary file already authors as worlds. MPCORB carries (134340) Pluto, and
# the catalogue keys bodies by NAME (so do saves, src/save.h), so a Pluto row here would
# not be a second data point, it would be a collision the generator would "solve" by
# renaming it (134340) Pluto. Ceres is already a corpus row and never gets here.
ALREADY_AUTHORED = {"pluto"}

# What to go looking for among the 1.58M records the wiki page never mentions, and how
# many of each. Every criterion is something the player can see from the Tracking map: a
# big rock, an extreme orbit, a family the 259 barely touch. MPCORB's U == 0 (its best
# uncertainty class) gates all of them, because a body tracked over two oppositions is a
# body whose orbit we would have to invent anyway.
# Brightness ranks most of them, and it is deliberately NOT what ranks the co-orbital
# family: Earth's quasi-satellites and minimoons (2016 HO3, 2020 CD3, 2024 PT5) are H 24
# to H 32, so an H ranking keeps the kilometre rocks sharing the orbit and drops exactly
# the bodies that make the family famous. That rule is therefore asked for twice, once per
# ranking. The small end is real -- H 31.7 is a metre across -- and the engine has to grow
# up around bodies that small rather than the catalogue hiding them.
CANDIDATE_BUCKETS = {
    "bright": 34,          # the biggest minor planets there are, family aside
    "sungrazer": 6,        # q < 0.1 AU: passes inside Mercury
    "coorbital": 8,        # |a - 1| < 0.05 and e < 0.5, brightest first
    "coorbital_close": 12,  # the same band, closest to 1 AU first: the minimoons
    "high_i": 6,           # i >= 90 deg: retrograde and polar orbits
    "hungaria": 4,         # a 1.75-2.1 AU, the innermost belt family
    "phocaea": 4,          # a 2.2-2.5 AU
    "trojan": 12,          # Jupiter's camp -- the wiki list carries two of some 12,000
}
# Kuiper bodies to add on top of those buckets: the TNO list has ~1,050 with a stated
# diameter, and the ratified depth is a headline set plus 40-60 numbered ones. Ranked by
# that diameter.
TNO_ADDS = 50
# Earth's co-orbital family, curated BY NAME. These cannot be mined: MPCORB's elements are
# an osculating HELIOCENTRIC orbit, and what makes a minimoon interesting -- that it spends
# part of its orbit inside Earth's Hill sphere -- is not an |a - 1| cut, because the 12
# tightest |a - 1| in 1.58M records are other bodies. So the family is named here and each
# member's rail and H come from MPCORB like every other row. Sizes land out of H alone:
# Kamo'oalewa comes to ~70 m, which is what New Horizons' follow-up imaging measured, so the
# arithmetic is trustworthy even at that end. The smallest of them is ~1 m across, and that
# is a real body with a real orbit -- the engine has to grow around it, not the other way.
FAMED_COORBITALS = ("2010 TK7", "2020 XL5", "2016 HO3", "2020 CD3", "2024 PT5",
                    "2003 YN107", "2002 AA29")
# Every rule that names a row's only section, so report() can tell an added row from a
# wiki row without a flag column.
CANDIDATE_RULES = set(CANDIDATE_BUCKETS) | {"kuiper", "size_page", "coorbital_famed"}
# MPCORB's uncertainty parameter, 0 (a well-determined orbit) to 9. The buckets already
# require 0; the two rules that mine a size table instead of a dynamical criterion accept
# anything up to this, because the TNO list's big bodies are exactly the ones MPC scores on
# opposition COUNT (Sedna: 26 oppositions, U 5, and its orbit is textbook). Above it the
# rail we would author is a guess with a decimal point on it -- U 7-8 objects like
# 2015 BP519 (a 486 AU orbit) move by AU per year of ephemeris.
MAX_ORBIT_U = 6


def mpcorb_buckets(rec):
    """[(rule, rank)] for one MPCORB record; rank None means rank by H. See
    CANDIDATE_BUCKETS for what the rules are for and why the co-orbital one appears twice.
    'bright' is the catch-all, so its heap cap is what keeps the 34 biggest rather than the
    first 34 seen."""
    if str(rec.get("U")) != "0":
        return []
    if rec.get("H") is None or rec.get("a") is None or rec.get("e") is None:
        return []
    a, e = float(rec["a"]), float(rec["e"])
    i = float(rec.get("i") or 0.0)
    q = rec.get("Perihelion_dist")
    t = rec.get("Orbit_type") or ""
    out = []
    if q is not None and float(q) < 0.1:
        out.append(("sungrazer", None))
    if abs(a - 1.0) < 0.05 and e < 0.5:
        out.append(("coorbital", None))
        out.append(("coorbital_close", abs(a - 1.0)))
    if i >= 90.0:
        out.append(("high_i", None))
    if t == "Hungaria":
        out.append(("hungaria", None))
    if t == "Phocaea":
        out.append(("phocaea", None))
    if t == "Jupiter Trojan":
        out.append(("trojan", None))
    out.append(("bright", None))
    return out


def text_key(s):
    """Lowercased, whitespace-collapsed key ('' for nothing)."""
    return " ".join((s or "").lower().split())


def row_wanted_keys(row):
    """Every key one row answers to in MPCORB. The number leads; a name is only trusted
    alongside it, because (52) Europa and (9) Metis are named for moons the game already
    has and MPCORB's own (52) is the asteroid."""
    out = set()
    n = row.get("number")
    if isinstance(n, int):
        out.add(("n", n))
    for field in ("name", "query"):
        s = text_key(row.get(field))
        if not s:
            continue
        bare = re.sub(r"^\(?\d+\)?\s+", "", s)
        for cand in {s, bare}:
            if cand:
                out.add(("d", cand))
                out.add(("nm", cand))
    return out


def mporb_identity_ok(row, rec):
    """Refuse a join where both sides carry a number and they differ. A designation or a
    name match clears it, so this only fires on the case that matters: MPCORB listing a
    DIFFERENT numbered body under a name we also use."""
    n = row.get("number")
    if not isinstance(n, int):
        return True
    rn = rec.get("number")
    return rn is None or rn == n


def size_page_index(entries):
    """{key: entry} for the size page: by number, by designation, and by bare NAME for the
    entries that are actually minor planets (sizes.is_minor_planet). The name keys are what
    reach Eris and Makemake, which the page lists by name with no number at all; the moons
    it also lists are what makes the filter necessary rather than cosmetic."""
    out = {}
    for e in entries:
        if not e['radius_m'] or not sizes.is_minor_planet(e):
            continue
        for k in size_entry_keys(e):
            out.setdefault(k, e)
    return out


def tno_index(rows):
    """{key: row} for the TNO list, same rule: number or exact designation."""
    out = {}
    for r in rows:
        if r["number"] is not None:
            out[("n", r["number"])] = r
        for key in (r["designation"], r["desig"]):
            k = text_key(re.sub(r"^\(?\d+\)?\s+", "", key or ""))
            if k:
                out.setdefault(("d", k), r)
    return out


def size_entry_keys(e):
    """Keys a size-page entry answers to: its number, its designation, its name (minus the
    wiki's '(moon)'/'(dwarf planet)' disambiguation)."""
    out = set()
    if e["number"] is not None:
        out.add(("n", e["number"]))
    for key in (e["designation"], e["name"]):
        k = text_key(re.sub(r"\s*\((?:moon|planet|dwarf planet)\)\s*$", "", key or ""))
        if k:
            out.add(("d", k))
            out.add(("nm", k))
    return out


def slim_keys(hit):
    """Keys an MPCORB record answers to, in the same shape, for the two wiki joins."""
    out = set()
    n = hit.get("number")
    if isinstance(n, int):
        out.add(("n", n))
    for v in [hit.get("Principal_desig")] + list(hit.get("aliases") or []):
        k = text_key(v)
        if k:
            out.add(("d", k))
            out.add(("nm", k))
    k = text_key(hit.get("Name"))
    if k:
        out.add(("nm", k))
    return out


def size_block(e):
    """The size-page fields as a corpus block, with the page's own text kept alongside the
    number so a reader can see what it actually claimed ('371+39−42')."""
    return {"radius_m": e["radius_m"], "radius_text": e["radius_text"],
            "mass_kg": e["mass_kg"], "type_note": e["type_note"],
            "src": "Wikipedia List of Solar System objects by size, table %d" % e["table"]}


def row_h_mag(row):
    """H from anywhere on the row -- the wiki cell, LCDB's adopted value, or the MPCORB
    block. Whichever it is, it is this body's brightness, which is what bounds its size."""
    for v in ((row.get("h_mag") or {}).get("value"),
              (row.get("lcdb") or {}).get("h_mag"),
              (row.get("mporb") or {}).get("H")):
        if v is not None:
            return float(v)
    return None


def size_is_plausible(radius_m, h_mag, other_km=None):
    """Does a stated radius belong to the body this H describes?

    The join guard, and it does not need to be tight: with the albedo band a real body can
    have (0.02 for a comet-like coal, 1.0 for Eris' frost), H bounds the radius within a
    factor ~7. A size-page entry that lands outside that band is the same WORD describing a
    different object -- the page's 'Ariel' is Uranus' moon at 579 km, and an asteroid
    called Ariel is a few km across. Returns False only when there is an H to argue with,
    and the 2x margins are there because H itself carries a few tenths of uncertainty.
    """
    if h_mag is None or not radius_m:
        if other_km and radius_m:
            return 0.4 <= (other_km * 500.0) / radius_m <= 2.5
        return True
    lo = 1329.0 / math.sqrt(1.0) * 10 ** (-float(h_mag) / 5.0) * 500.0     # pV 1.0
    hi = 1329.0 / math.sqrt(0.02) * 10 ** (-float(h_mag) / 5.0) * 500.0    # pV 0.02
    return 0.5 * lo <= radius_m <= 2.0 * hi


def augment(records):
    """Mutate the rows with an "mporb" block and a "size_page" block; return [row] for the
    bodies MPCORB and the size tables know about but the exceptional list never did."""
    tno_rows = []
    if os.path.exists(TNO_HTML):
        tno_rows = sizes.tno_rows(open(TNO_HTML, encoding="utf-8").read())
    else:
        print("note    no %s -- no Kuiper rows (run fetch.py)" % TNO_HTML)
    size_entries = []
    if os.path.exists(SIZES_HTML):
        size_entries = sizes.objects(open(SIZES_HTML, encoding="utf-8").read())
    else:
        print("note    no %s -- no stated radii for the distant tail (run fetch.py)"
              % SIZES_HTML)

    corpus_keys = set()
    for r in records:
        corpus_keys |= row_wanted_keys(r)
    wanted = set(corpus_keys)
    for r in tno_rows:
        if r["number"] is not None:
            wanted.add(("n", r["number"]))
        for key in (r["designation"], r["desig"]):
            k = text_key(re.sub(r"^\(?\d+\)?\s+", "", key or ""))
            if k:
                wanted.add(("d", k))
                wanted.add(("nm", k))
    for e in size_entries:
        if not sizes.is_minor_planet(e):
            continue
        for k in size_entry_keys(e):
            wanted.add(k)
    for desig in FAMED_COORBITALS:
        wanted.add(("d", text_key(desig)))

    if not os.path.exists(mporb.MPCORB_GZ):
        print("note    no %s -- no H, no dynamical class, no new candidates (run fetch.py)"
              % mporb.MPCORB_GZ)
        hits, picks = {}, {}
    else:
        def bucket(rec, keys):
            """A candidate is a record the corpus has NO row for. The test is necessary,
            not tidy: `wanted` now includes the two candidate tables, so without it the
            'brightest 34' bucket returns nothing -- every bright body in MPCORB is in one
            of the lists we are also looking up."""
            if keys & corpus_keys:
                return []
            return mpcorb_buckets(rec)
        hits, picks = mporb.extract(wanted, bucket=bucket, bucket_cap=CANDIDATE_BUCKETS)

    # 1. The existing rows.
    joined_h = joined_cls = 0
    for r in records:
        rec = next((hits[k] for k in row_wanted_keys(r) if k in hits), None)
        if rec is None:
            continue
        if not mporb_identity_ok(r, rec):
            print("  REFUSE %s: MPCORB has %s (%s) -- a different number, so not this body"
                  % (r["name"], rec.get("Number"), rec.get("Name")))
            continue
        r["mporb"] = rec
        if rec.get("H") is not None and "h_mag" not in r and "lcdb" not in r:
            joined_h += 1
        if rec.get("Orbit_type") and "spectral_type" not in r:
            joined_cls += 1

    # 2. The size page, for the rows it answers to.
    sidx = size_page_index(size_entries)
    sized_here = 0
    for r in records:
        e = next((sidx[k] for k in row_wanted_keys(r) if k in sidx), None)
        if e is None:
            continue
        other = (r.get("diam_eff_km") or {}).get("value") or \
                (r.get("diameter_km") or {}).get("value")
        if not size_is_plausible(e["radius_m"], row_h_mag(r), other):
            print("  REFUSE %s: the size page's %.0f km does not belong to a body with "
                  "H %s -- a namesake, not this object"
                  % (r["name"], e["radius_m"] / 1000.0, row_h_mag(r)))
            continue
        r["size_page"] = size_block(e)
        sized_here += 1

    # 3. New rows. Every candidate needs an MPCORB record with a rail and an H, so the
    # generator has both a place to put the body and a way to size it. Which rule found it
    # is only a label: the size page and the TNO list are looked up for every one of them,
    # because Eris walks in on the 'bright' rule and still needs a stated radius -- sizing
    # it from H alone would make it four times its real size.
    taken = {text_key(r["name"]) for r in records}
    taken |= {r["number"] for r in records if isinstance(r.get("number"), int)}
    ti = tno_index(tno_rows)
    extra = []
    too_weak = []

    def add(hit, why):
        name = " ".join((hit.get("Name") or hit.get("Principal_desig") or "").split())
        key = text_key(name)
        number = hit.get("number")
        if not name or key in ALREADY_AUTHORED or key in taken:
            return None
        try:
            u = int(str(hit.get("U")))
        except (TypeError, ValueError):
            u = 99
        if u > MAX_ORBIT_U:
            too_weak.append("%s (U %s, %s)" % (name or number, hit.get("U"),
                                               hit.get("Principal_desig")))
            return None
        if isinstance(number, int) and number in taken:
            return None
        keys = slim_keys(hit)
        sent = next((sidx[k] for k in keys if k in sidx), None)
        if sent is not None and not size_is_plausible(sent["radius_m"], hit.get("H")):
            print("  REFUSE %s: the size page's %.0f km does not belong to a body with "
                  "H %s -- a namesake, not this object"
                  % (name, sent["radius_m"] / 1000.0, hit.get("H")))
            sent = None
        row = {"name": name, "number": number, "query": name, "sections": [why],
               "mporb": hit,
               "comments": "Added from MPCORB (%s) for the %s rule; not on the "
                           "exceptional-asteroids page." % (hit.get("Principal_desig"), why)}
        if sent is not None:
            row["size_page"] = size_block(sent)
        trow = next((ti[k] for k in keys if k in ti), None)
        if trow is not None:
            row["tno"] = {k: trow[k] for k in ("diam_km", "class", "albedo", "taxonomy",
                                               "binary", "rings", "a_au", "e", "i_deg",
                                               "q_au", "Q_au", "year")
                          if trow.get(k) is not None}
        taken.add(key)
        if isinstance(number, int):
            taken.add(number)
        extra.append(row)
        return row

    counts = {}
    # The named family first, so a member that also falls in a mined bucket keeps the
    # curated rule as its section.
    missing_famed = []
    for desig in FAMED_COORBITALS:
        hit = hits.get(("d", text_key(desig)))
        if hit is None:
            missing_famed.append(desig)
            continue
        add(hit, "coorbital_famed")
    counts["coorbital_famed"] = sum(1 for r in extra
                                    if r["sections"][0] == "coorbital_famed")
    if missing_famed:
        print("  MISS  co-orbital family not in MPCORB: %s" % ", ".join(missing_famed))
    for b in sorted(CANDIDATE_BUCKETS):
        counts[b] = sum(1 for hit in picks.get(b, []) if add(hit, b) is not None)

    # 4. The Kuiper belt: the TNO list ranked by the diameter IT states, keeping the ones
    # MPCORB has a first-class orbit for.
    got = 0
    for r in sorted((r for r in tno_rows if r["diam_km"] and r["number"] is not None),
                    key=lambda r: -r["diam_km"]):
        if got >= TNO_ADDS:
            break
        hit = hits.get(("n", r["number"]))
        if hit is None or hit.get("H") is None:
            continue
        if add(hit, "kuiper") is not None:
            got += 1
    counts["kuiper"] = got

    # ... and the size page's own headline bodies that the TNO list misses: the centaurs,
    # the largest Trojans, the named scattered-disc objects. Ranked by its own radius.
    got = 0
    for e in sorted((e for e in size_entries
                     if e["number"] is not None and sizes.is_minor_planet(e)
                     and e["radius_m"] >= 100e3), key=lambda e: -e["radius_m"]):
        hit = next((hits[k] for k in size_entry_keys(e) if k in hits), None)
        if hit is None or hit.get("H") is None:
            continue
        if add(hit, "size_page") is not None:
            got += 1
    counts["size_page"] = got

    print("mporb   %d distinct records matched, %d rows given an H they lacked, %d a "
          "dynamical class; %d new rows (%s)"
          % (len({id(v) for v in hits.values()}), joined_h, joined_cls, len(extra),
             ", ".join("%s %d" % (k, v) for k, v in sorted(counts.items()))))
    if too_weak:
        print("  SKIP  %d candidates whose MPCORB orbit is worse than U %d: %s"
              % (len(too_weak), MAX_ORBIT_U, ", ".join(too_weak[:8])))
    print("sizes   %d size-page entries (%d joined, %d only as new rows), %d TNO rows"
          % (len(size_entries), sized_here,
             sum(1 for r in extra if "size_page" in r), len(tno_rows)))
    return extra


def check_crosscheck(records):
    """The wiki's rounded orbit rows against SBDB's elements -- the disagreement a
    reader would otherwise inherit silently."""
    bad = 0
    for r in records:
        sb = r.get("sbdb")
        if not sb:
            continue
        # The absolute tolerance goes first: a near-zero eccentricity or inclination is
        # not "infinitely off" merely because the relative gap is large.
        for wk, jpl, rel_tol, abs_tol in (("wiki_sma_au", "a_au", 0.02, 0.02),
                                          ("wiki_ecc", "e", 0.05, 0.005),
                                          ("wiki_inclination_deg", "i_deg", 0.05, 0.05)):
            if wk not in r:
                continue
            wv = r[wk]["value"]
            jv = sb.get(jpl)
            if jv is None or abs(wv - jv) <= abs_tol:
                continue
            rel = abs(wv - jv) / max(abs(jv), abs_tol)
            if rel > rel_tol:
                print("  CROSS %s: wiki %s = %g, SBDB %s = %g (%.1f%% off)"
                      % (r["name"], wk, wv, jpl, jv, 100 * rel))
                bad += 1
    return bad


def check_diameters(records):
    """Wiki 'Diameter' against LCDB's EFFECTIVE diameter.

    They disagree a lot for elongated bodies, and the disagreement is not noise: the
    page quotes the LONGEST axis (or a single chord) where LCDB quotes the diameter of
    the equal-volume sphere. The triaxial row decides which is which -- the geometric
    mean of three axes IS the effective diameter, so when it agrees with LCDB and the
    page's number agrees with the longest axis, the page is quoting an axis. Authoring
    a sphere from the page's number then inflates the body by up to 2x in radius.
    """
    bad = 0
    for r in records:
        wk = r.get("diameter_km")
        lc = r.get("lcdb", {}).get("diam_eff_km")
        if not wk or lc is None:
            continue
        wv = wk["value"]
        if abs(wv - lc) <= 0.10 * lc:
            continue
        tri = r.get("dimensions_km", {}).get("value")
        geo = triaxial_eff(r)
        print("  DIAMETER %s: wiki %g%s vs LCDB eff %g (%s); triaxial %s -> eff mean %s"
              % (r["name"], wv, "" if wk.get("note") is None else " (%s)" % wk["note"],
                 lc, r["lcdb"].get("diam_src", "?"),
                 "x".join("%g" % x for x in tri) if tri else "-",
                 ("%g" % geo) if geo else "-"))
        bad += 1
    return bad


def check_axes(records):
    """Where the wiki's 'Diameter' number is not this body's size.

    The cell is read as its FIRST number and the page does not order the axes -- Eros is
    "34 x 11 x 11" in the brightness table and "13 x 13 x 33" in the visited table -- so
    the leading number can be the shortest axis. Flag a body whose diameter number is
    neither one of the axes nor the equal-volume mean of them.

    Which row wins is document order (fields are written with setdefault), so this
    corpus is only as stable as the page's heading order; an explicit section priority
    is the fix if Wikipedia ever reshuffles."""
    bad = 0
    for r in records:
        wk = r.get("diameter_km")
        tri = (r.get("dimensions_km") or {}).get("value")
        if not wk or not tri:
            continue
        wv = wk["value"]
        geo = triaxial_eff(r)
        if any(abs(wv - x) <= 0.1 * x for x in tri + ([geo] if geo else [])):
            continue
        print("  AXES %s: wiki Diameter %g is neither an axis (%s) nor the equal-volume"
              " mean (%s)" % (r["name"], wv, "x".join("%g" % x for x in tri),
                              ("%g" % geo) if geo else "-"))
        bad += 1
    return bad


def sanity(records):
    """Numbers that are arithmetically fine and physically silly.

    merge() already refuses to author a density outside DENSITY_BAND; this says WHY, by
    checking every shape candidate rather than only the one we would use. The wiki's
    'Most massive' table carries a row whose stated precision is literally '?' (532
    Herculina: '~ 23' x10^18 kg), and its diameter rows disagree with LCDB's by enough
    that the density swings over 2x depending on which you divide by -- so one number
    passing the band would be luck, not knowledge.
    """
    bad = 0
    for r in records:
        m = r.get("mass_1e18_kg")
        if not m:
            continue
        rhos = [density_from(m["value"], d)
                for _, d in shape_candidates(r, r.get("lcdb", {}))]
        rhos = [x for x in rhos if x is not None]
        if not rhos:
            continue
        lo, hi = min(rhos), max(rhos)
        if DENSITY_BAND[0] <= lo and hi <= DENSITY_BAND[1]:
            continue
        print("  SANITY %s: density %.0f-%.0f kg/m3 across %d shape candidates from"
              " mass %s e18 kg (stated precision %r, unc %s, note %r) -- do not author"
              " this body's mass"
              % (r["name"], lo, hi, len(rhos), m.get("value"), m.get("precision"),
                 m.get("unc"), m.get("note")))
        bad += 1
    return bad


def report(records):
    have = lambda k: sum(1 for r in records if k in r)
    print("%d bodies from the wiki page; %d with an SBDB record; %d with an LCDB record"
          % (len(records), sum(1 for r in records if "sbdb" in r),
             sum(1 for r in records if "lcdb" in r)))
    for k, label in (("diameter_km", "diameter (wiki cell)"),
                     ("dimensions_km", "triaxial shape"),
                     ("diam_eff_km", "effective diameter (DERIVED)"),
                     ("mass_kg", "mass"), ("wiki_rotation_h", "rotation (wiki)"),
                     ("rotation_period_h", "rotation (resolved)"),
                     ("spectral_type", "spectral type"), ("h_mag", "abs. magnitude H"),
                     ("albedo_pV", "albedo"), ("density_kg_m3", "density (DERIVED)"),
                     ("spacecraft", "spacecraft visited"), ("best_mag", "brightest from Earth")):
        print("  %-26s %3d" % (label, have(k)))
    for k, label in (("period_h", "rotation period"), ("diam_eff_km", "effective diameter"),
                     ("albedo_pV", "geometric albedo"), ("class", "taxonomic class"),
                     ("h_mag", "abs. magnitude H"), ("binary", "binary flag"),
                     ("family", "orbital family")):
        print("  %-26s %3d" % ("LCDB " + label,
                               sum(1 for r in records if k in r.get("lcdb", {}))))
    print("  %-26s %3d" % ("spin-axis solutions", sum(1 for r in records if "lcdb_poles" in r)))
    print("  %-26s %3d" % ("binary records", sum(1 for r in records if "lcdb_binaries" in r)))
    print("  %-26s %3d" % ("tumbling (NPA) records",
                           sum(1 for r in records if "lcdb_tumbling" in r)))
    # MPCORB and the two size tables -- what the extra sources contribute, and the class
    # they contribute, which for most rows is a DYNAMICAL class rather than a spectral one.
    print("  %-26s %3d" % ("MPCORB block", sum(1 for r in records if "mporb" in r)))
    print("  %-26s %3d" % ("MPCORB H", sum(1 for r in records
                                           if (r.get("mporb") or {}).get("H") is not None)))
    print("  %-26s %3d" % ("MPCORB U=0", sum(1 for r in records
                                             if str((r.get("mporb") or {}).get("U")) == "0")))
    print("  %-26s %3d" % ("size-page radius", sum(1 for r in records if "size_page" in r)))
    print("  %-26s %3d" % ("TNO-list row", sum(1 for r in records if "tno" in r)))
    print("  %-26s %3d" % ("rows from MPCORB rules",
                           sum(1 for r in records
                               if r["sections"] and set(r["sections"]) <= CANDIDATE_RULES)))
    tally = {}
    for r in records:
        t = (r.get("mporb") or {}).get("Orbit_type")
        if t:
            tally[t] = tally.get(t, 0) + 1
    print("  %-26s %s" % ("dynamical class", " ".join("%s:%d" % kv for kv in
                                                      sorted(tally.items(),
                                                             key=lambda kv: -kv[1]))))
    # What the LCDB values are actually worth: 'A' albedo is assumed from the class and
    # 'C'/'D' diameter is arithmetic on H, not a measurement.
    for key, label in (("diam_src", "LCDB diameter by DiamSource"),
                       ("alb_src", "LCDB albedo by AlbSource"),
                       ("class_src", "LCDB class by CSource")):
        tally = {}
        for r in records:
            v = r.get("lcdb", {}).get(key)
            if v:
                tally[v] = tally.get(v, 0) + 1
        print("  %-26s %s" % (label, " ".join("%s:%d" % kv for kv in
                                              sorted(tally.items(), key=lambda kv: -kv[1]))))
    print("-" * 100)

    def val(d, key):
        """A number or None -- never `a or b`, which turns a real 0.0 into a fallback."""
        v = (d or {}).get(key)
        return v

    def diam(r):
        """(effective diameter, source tag). 'wiki' means the cell may be an axis rather
        than a size, which is exactly why it is tagged and not silently used."""
        for v, tag in (((r.get("diam_eff_km") or {}).get("value"), "tri"),
                       ((r.get("lcdb") or {}).get("diam_eff_km"), "lcdb"),
                       ((r.get("diameter_km") or {}).get("value"), "wiki")):
            if v is not None:
                return v, tag
        return None, None

    def orbit(r, jpl, wk):
        v = val(r.get("sbdb"), jpl)
        return v if v is not None else val(r.get(wk), "value")

    print("%-22s %9s %10s %7s %6s %8s %8s %s" %
          ("name", "diam km", "mass kg", "a AU", "ecc", "inc", "rot h", "spec"))
    sized = [(diam(r), r) for r in records]
    sized = [(d, r) for d, r in sized if d[0] is not None]
    sized.sort(key=lambda x: -x[0][0])
    for (d, tag), r in sized[:25]:
        mass = val(r.get("mass_kg"), "value")
        p = val(r.get("rotation_period_h"), "value")
        print("%-22s %6.1f %-3s %10s %7.3f %6.3f %8.2f %8s %s"
              % (r["name"], d, tag, ("%.2e" % mass) if mass else "-",
                 orbit(r, "a_au", "wiki_sma_au") or 0.0,
                 orbit(r, "e", "wiki_ecc") or 0.0,
                 orbit(r, "i_deg", "wiki_inclination_deg") or 0.0,
                 ("%.1f" % p) if p else "-",
                 val(r.get("spectral_type"), "value") or "-"))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report", action="store_true", help="print the coverage + big-body table")
    ap.add_argument("--check", action="store_true",
                    help="re-derive and compare against asteroids.json on disk")
    ap.add_argument("--no-write", action="store_true", help="derive without writing")
    args = ap.parse_args()

    if not os.path.exists(WIKI_HTML):
        raise SystemExit("parse: %s missing -- run fetch.py first" % WIKI_HTML)
    records = merge()
    records += augment(records)
    drift = check_crosscheck(records)
    diam = check_diameters(records)
    axes = check_axes(records)
    dodgy = sanity(records)

    blob = json.dumps({"bodies": records}, indent=1, sort_keys=True)
    if args.no_write:
        print("derived %d records (%d wiki/SBDB cross-check gaps, %d wiki/LCDB diameter"
              " gaps, %d diameter cells that are an axis, %d implausible derived densities)"
              % (len(records), drift, diam, axes, dodgy))
        if args.report:
            report(records)
        return 0
    if args.check:
        if not os.path.exists(OUT):
            raise SystemExit("check: %s missing -- run without --check once" % OUT)
        with open(OUT, encoding="utf-8") as f:
            if f.read().strip() != blob.strip():
                raise SystemExit("check: %s is stale -- re-run parse.py" % OUT)
        print("OK      %s (%d bodies, %d cross-check notes, %d diameter notes, %d axis"
              " notes, %d implausible masses)"
              % (OUT, len(records), drift, diam, axes, dodgy))
        if args.report:
            report(records)
        return 0
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(blob + "\n")
    print("wrote   %s (%d bodies, %d cross-check notes, %d diameter notes, %d axis notes,"
          " %d implausible masses)" % (OUT, len(records), drift, diam, axes, dodgy))
    if args.report:
        report(records)
    return 0


if __name__ == "__main__":
    sys.exit(main())
