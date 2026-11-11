#!/usr/bin/env python3
"""Read the cached Asteroid Lightcurve Database (Warner, Harris & Pravec 2021, V4.0).

The LCDB is a better PHYSICAL source than the Wikipedia page for almost everything:
one adopted record per object for rotation period (with a reliability code U), effective
diameter (with its source), geometric albedo, taxonomic class, absolute magnitude H and
phase slope G, binary status, and orbital family -- and 34,967 objects, not 259. It has
no mass and no orbit, so it does not replace SBDB or the wiki's mass table; the three
sources are stacked by what each is authoritative for.

raw/ holds the upstream zip plus the four tables we read (the 117 MB of per-observation
detail rows are deliberately not extracted). SBN Archive sits behind a Cloudflare
challenge, so fetch.py CANNOT download this -- it has to be fetched by hand and
unzipped into raw/ (see urls.txt).

Source flags decide what a value is worth (pds_readme tables 4.1.4.6/4.1.4.7/4.1.4.4),
and LCDB prints them rather than hiding them, which is why adopted() keeps them:
  DiamSource  C calculated from albedo and H | D derived from albedo and H (Harris &
              Harris 1997) | L taken from a details-table entry (a specific study) |
              S SIMPS | T thermal IR | W WISE | K AKARI
  AlbSource   A ASSUMED from the orbital group or taxonomic type | D derived from H and
              diameter | L/S/T/W/K as above
An 'A' albedo is a guess, not a measurement: Bennu, Ryugu, Eros, Hermes and 2012 DA14
all sit at exactly 0.20 because they are all classed S*, while Bennu's measured value is
0.049. Same trap on the diameter side: Bennu's adopted Diam is 0.213 km against a radar
mean of 0.49 km. So the adopted column is a starting point, never an authority.

Poles are carried VERBATIM, not converted to an obliquity. lc_spinaxis gives several
candidate poles per body (the 180 deg ambiguity is normal), each with a Q quality code,
a Model flag and a reference, and the readme documents the pair as ecliptic
longitude/latitude (pds_readme, the Long1/Lat1 rows) -- but the best-Q row is not always
the one matching the adopted period (Hygiea's Q=3 row carries SidPer 27.62 h against a
summary period of 13.83 h), and picking a pole is a decision that needs validating
against the body's own orbital plane before it can become an axial_tilt. #765.
"""
import csv
import os
import re
import unicodedata

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "raw")

# What this module actually reads. lc_familylookup.csv is cached but unread: it is keyed
# by FamNumber (a family, not a body) and only says which default albedo a family gets,
# which is where the 'A' albedos above come from.
TABLES = ("lc_summary.csv", "lc_spinaxis.csv", "lc_binary.csv", "lc_npa.csv")

# The bundle's declared missing-value markers. NOT "any negative": a spin-axis latitude
# is legitimately down to -90, and a blanket negative test throws away 47% of the pole
# solutions -- all from one hemisphere -- which would then look like LCDB said so.
SENTINELS = ("", "-", "-9", "-9.99")

PROVISIONAL = re.compile(r"^\d{4} [A-Z]{1,2}\d{0,2}$")


def have_tables():
    return all(os.path.exists(os.path.join(RAW, t)) for t in TABLES)


def _num(s):
    """One cell as a float, or None when it is a declared missing marker.

    Column-specific nonsense (the -1/-99 that appear as filler inside the pole columns)
    is caught by the range checks where those columns are read, not here."""
    if s is None:
        return None
    s = s.strip()
    if s in SENTINELS:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def table(name, key="Number"):
    """One PDS4 CSV: keyword preamble, a header row, units/types/sentinel/description
    rows, a '# Data' marker, then the rows. Asserts the layout so a future version fails
    here instead of parsing prose as numbers."""
    path = os.path.join(RAW, name)
    if not os.path.exists(path):
        raise SystemExit("lcdb: %s missing -- unzip it from the upstream archive "
                         "(see urls.txt; SBN Archive blocks scripted downloads)" % path)
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rows = [r for r in csv.reader(fh)]
    h = next((i for i, r in enumerate(rows) if r and r[0] == key), None)
    if h is None:
        raise SystemExit("lcdb: %s has no %r header column -- the layout changed"
                         % (name, key))
    m = next((i for i, r in enumerate(rows[h:], h) if r and r[0] == "# Data"), None)
    if m is None:
        raise SystemExit("lcdb: %s has no '# Data' marker -- the layout changed" % name)
    header = rows[h]
    data = [r for r in rows[m + 1:] if r and r[0].strip()]
    assert re.fullmatch(r"-?\d+|-", data[0][0]), \
        "lcdb: %s layout changed; first data row is %r" % (name, data[0][:2])
    for r in data:
        assert len(r) == len(header), \
            "lcdb: %s row %r has %d fields, header has %d" % (name, r[:2], len(r), len(header))
    return [dict(zip(header, r)) for r in data]


def _norm_name(s):
    """Fold a name for comparison. LCDB is ASCII and the wiki is not: '2867 Šteins' vs
    Steins, 'Wilson–Harrington' (en dash) vs Wilson-Harrington, '514107 Kaʻepaokaʻāwela'
    vs Ka`epaoka`awela. Transliterate, then keep only letters and digits."""
    s = unicodedata.normalize("NFKD", (s or ""))
    return re.sub(r"[^a-z0-9]+", "", s.lower())


def _ident(row):
    """(permanent number, folded name, folded MPC designation) for one LCDB row. A
    satellite row carries the PRIMARY's number and its own name, which is the trap."""
    number = row["Number"] if row.get("Number") not in ("-9", "", None) else None
    name = _norm_name(row.get("Name"))
    desig = _norm_name(row.get("Desig")) if row.get("Desig") not in ("-", "", None) else ""
    return number, name, desig


def index(rows):
    """One row per key, for the summary table (one adopted record per object)."""
    out = {}
    for r in rows:
        number, name, desig = _ident(r)
        if number:
            out.setdefault(("n", number), r)
        for s in (name, desig):
            if s:
                out.setdefault(("s", s), r)
    return out


def index_by_body(rows):
    """ALL rows per key, for the tables with several records per object (pole solutions,
    binaries, NPA solutions). Folding a name per body per row would be 259 x 5866 regex
    passes at this corpus size and far worse at the full 34,967, so fold once here."""
    out = {}
    for r in rows:
        number, name, desig = _ident(r)
        if number:
            out.setdefault(("n", number), []).append(r)
        for s in (name, desig):
            if s:
                out.setdefault(("s", s), []).append(r)
    return out


def _rows_for(bidx, rec):
    """Every row of a multi-record table that belongs to one of our bodies."""
    hit = []
    if rec.get("number"):
        for r in bidx.get(("n", str(rec["number"])), []):
            if _name_ok(rec.get("name"), r):
                hit.append(r)
    if not hit:
        for s in (rec.get("name"), rec.get("query")):
            if s:
                hit.extend(bidx.get(("s", _norm_name(s)), []))
    return hit


def _name_ok(ours, row):
    """Guard for a number-based join. LCDB gives a satellite the PRIMARY's permanent
    number (Ida I Dactyl sits on row Number=243, Name=Ida), so matching a moon to its
    parent's row is the failure mode: it would author a 1.4 km moon with a 28 km
    diameter and Ida's rotation period.

    A Name that is itself a provisional designation is not a disagreement -- LCDB
    labels some numbered rows by the designation they were discovered under (98943 is
    named '2001 CC21', which is 98943 Torifune), and the permanent number is authority.
    """
    ours = _norm_name(ours)
    if not ours:
        return True
    for c in (row.get("Name"), row.get("Desig")):
        c = (c or "").strip()
        if c in ("", "-"):
            continue
        # The provisional test is case-sensitive, so it runs on the raw string.
        if _norm_name(c) == ours or PROVISIONAL.match(c):
            return True
    return False


def lookup(idx, rec):
    """(LCDB summary row or None, note). The note explains a rejected join so a silent
    miss and a wrong-identity match do not look the same."""
    if rec.get("number"):
        row = idx.get(("n", str(rec["number"])))
        if row:
            if _name_ok(rec.get("name"), row):
                return row, None
            return None, ("LCDB row Number=%s Name=%s is not %r -- a satellite row"
                          " carries its primary's number"
                          % (row["Number"], row["Name"], rec.get("name")))
    for s in (rec.get("name"), rec.get("query")):
        if s:
            row = idx.get(("s", _norm_name(s)))
            if row:
                return row, None
    return None, None


def adopted(row):
    """The summary record's adopted physical values, each with the source flag the
    LCDB itself carries -- so a Calculated diameter never reads like a measured one."""
    out = {}
    for col, field, conv in (("Period", "period_h", _num), ("AmpMin", "amp_min_mag", _num),
                             ("AmpMax", "amp_max_mag", _num), ("Diam", "diam_eff_km", _num),
                             ("Albedo", "albedo_pV", _num), ("H", "h_mag", _num),
                             ("G", "phase_G", _num)):
        v = conv(row[col])
        if v is not None:
            out[field] = v
    for col, field in (("U", "period_quality"), ("Class", "class"),
                       ("CSource", "class_src"), ("DiamSource", "diam_src"),
                       ("DiamFlag", "diam_flag"), ("HSource", "h_src"),
                       ("AlbSource", "alb_src"), ("PFlag", "period_flag"),
                       ("IsBinary", "binary"), ("Pole", "pole_reported"),
                       ("Family", "family")):
        v = row[col].strip()
        if v not in ("-", ""):
            out[field] = v
    return out


def poles_for(bidx, rec):
    """Every reported spin-axis solution for one body, verbatim.

    Several per body is normal: the 180 deg pole ambiguity plus independent studies.
    Q is the LCDB's own quality code (higher is better, '-' means unqualified), Model
    says a shape model backs it.
    """
    out = []
    for r in _rows_for(bidx, rec):
        poles = []
        for i in (1, 2, 3, 4):
            lon = _num(r["Long%d" % i])
            lat = _num(r["Lat%d" % i])
            # A real pole is 0-360 in longitude and -90..90 in latitude; that range also
            # rejects the -1/-99 filler these columns use, which _num cannot see.
            if lon is None or lat is None or not 0.0 <= lon < 360.0 \
                    or not -90.0 <= lat <= 90.0:
                continue
            lon_err, lat_err = _num(r["Long%dErr" % i]), _num(r["Lat%dErr" % i])
            poles.append({"lon_deg": lon, "lat_deg": lat,
                          "lon_err": lon_err if lon_err and lon_err >= 0 else None,
                          "lat_err": lat_err if lat_err and lat_err >= 0 else None})
        if not poles:
            continue
        out.append({"poles": poles, "sidereal_period_h": _num(r["SidPer"]),
                    "quality": r["Q"].strip(), "model": r["Model"].strip(),
                    "ref": r["ShortRef"].strip(), "date_obs": r["DateObs"].strip()})
    return out


def binaries_for(bidx, rec):
    out = []
    for r in _rows_for(bidx, rec):
        out.append({"type": r["BinType"].strip(), "orbital_period_h": _num(r["OrbPer"]),
                    "sep_over_primary_diam": _num(r["ADp"]), "ref": r["ShortRef"].strip(),
                    "date_obs": r["DateObs"].strip()})
    return out


def npa_for(bidx, rec):
    """Non-principal-axis (tumbling) rotation solutions: a body with two periods is not
    spinning about a stable axis, which is the most physical thing in the bundle after
    the summary table. PAR is the solution quality (Pravec et al 2005)."""
    out = []
    for r in _rows_for(bidx, rec):
        out.append({"period_h": _num(r["DetPeriod"]), "period_err_h": _num(r["DetPerErr"]),
                    "amp_mag": _num(r["DetAmp"]),
                    "secondary_period_h": _num(r["DetPer2"]),
                    "secondary_period_err_h": _num(r["DetPer2Err"]),
                    "secondary_amp_mag": _num(r["DetAmp2"]),
                    "solution_quality": r["PAR"].strip(),
                    "ref": r["ShortRef"].strip(), "date_obs": r["DateObs"].strip()})
    return out


def load():
    """Everything parse.py needs: the summary index plus row indexes for the three
    multi-record tables."""
    idx = index(table("lc_summary.csv"))
    bidx = {name: index_by_body(table(name))
            for name in ("lc_spinaxis.csv", "lc_binary.csv", "lc_npa.csv")}
    return idx, bidx
