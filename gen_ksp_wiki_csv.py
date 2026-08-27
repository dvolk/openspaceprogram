#!/usr/bin/env python3
# Build a rich per-body CSV from the individual KSP wiki pages.
#
# Each body page (https://wiki.kerbalspaceprogram.com/wiki/<Name>) carries an
# infobox table with far more fields than the old ksp_system.csv (apoapsis,
# periapsis, arg. of periapsis, long. asc. node, mean anomaly, synodic
# period, orbital velocity, mu, density, escape velocity, solar day,
# synchronous orbit, atmospheric pressure/height, temperatures, oxygen, and
# the per-biome science multipliers).
#
# We parse that infobox with pandas.read_html, normalise the values (drop
# [Note N] refs, strip digit-group spaces, turn 6.1e12-style "x10N" into
# e-notation) and emit one row per body with a unit in each column name.
#
# "Axial tilt" is NOT on the individual pages, so we merge that one column
# back in from the old ksp_system.csv to keep the new file a true superset.
#
# Output: ksp_wiki_bodies.csv (ksp_system.csv is left untouched).
import re
import time
import warnings

import pandas as pd

warnings.filterwarnings("ignore")

WIKI = "https://wiki.kerbalspaceprogram.com/wiki/{name}"
BODIES = [
    "Kerbol", "Moho", "Eve", "Gilly", "Kerbin", "Mun", "Minmus",
    "Duna", "Ike", "Dres", "Jool", "Laythe", "Vall", "Tylo",
    "Bop", "Pol", "Eeloo",
]
OLD_CSV = "ksp_system.csv"
OUT_CSV = "ksp_wiki_bodies.csv"

# ---------------------------------------------------------------------------
# value normalisation
# ---------------------------------------------------------------------------
def clean(s):
    """Normalise a raw wiki cell: drop footnote refs, tidy unicode spaces."""
    if s is None:
        return None
    if isinstance(s, float) and pd.isna(s):
        return None
    s = str(s)
    s = s.replace("\u2009", " ").replace("\u2009", " ").replace("\xa0", " ")
    s = re.sub(r"\s*\[Note\s*\d+\]", "", s)   # [Note 1] etc.
    s = re.sub(r"\s+", " ", s).strip()
    return s

def to_number(s):
    """Pull the leading number out of a cleaned cell; keep e-notation."""
    if s is None:
        return None
    s = s.strip()
    if s in ("\u221e", "inf", "infinity"):
        return "inf"
    # 5.0x10-6 / 6.15x1012  ->  5.0e-6 / 6.15e12
    s = re.sub(r"\u00d710-(\d+)", r"e-\1", s)
    s = re.sub(r"\u00d710(\d+)", r"e\1", s)
    # drop digit-group spaces ("10 811" -> "10811")
    s = re.sub(r"(?<=\d) (?=\d)", "", s)
    s = s.replace(",", "")
    m = re.match(r"^(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)", s)
    return m.group(1) if m else None

def split_two(s):
    """'16.7 m/s2 (1.701 g)' -> ['16.7', '1.701']; '10811 - 11029 m/s' -> [lo, hi]."""
    if s is None:
        return [None, None]
    parts = re.split(r"\s[-\u2013]\s|\s\(", s)
    out = []
    for p in parts:
        n = to_number(p)
        if n is not None:
            out.append(n)
    while len(out) < 2:
        out.append(None)
    return out[:2]

def split_temp(s):
    """'-113.13 °C 160.02 K' -> [-113.13, 160.02] (the two values are separated
    by the °C unit, not by a dash or parenthesis)."""
    if s is None:
        return [None, None]
    if "\u00b0C" in s:
        head, _, tail = s.partition("\u00b0C")
        return [to_number(head), to_number(tail)]
    return [to_number(s), None]

# ---------------------------------------------------------------------------
# infobox parsing
# ---------------------------------------------------------------------------
def get_infobox(name):
    url = WIKI.format(name=name)
    for t in pd.read_html(url):
        col0 = t[0].astype(str).tolist()
        if any("Semi-major axis" in c or "Equatorial radius" in c for c in col0):
            return t
    raise RuntimeError(f"no infobox table found on {url}")

def parse_body(name):
    t = get_infobox(name)
    header = None
    fields = {}           # field name -> [value, value, ...] in order
    for _, r in t.iterrows():
        a, c = r[0], r[2]
        if str(a) == str(c) == str(r[1]):
            # name / caption / section header / footnote row
            if header is None and re.match(r"^(Star|Planet|Dwarf planet|Moon)( of \w+)?$", str(a)):
                header = str(a)
            continue
        field = clean(a)
        val = clean(c)
        if field is None:
            continue
        fields.setdefault(field, []).append(val)
    return header, fields

def header_info(header):
    """'Planet of Kerbol' -> ('Planet', 'Kerbol'); 'Star' -> ('Star', '')."""
    if not header:
        return "", ""
    if header == "Star":
        return "Star", ""
    m = re.match(r"^(Dwarf planet|Planet|Moon) of (\w+)$", header)
    if m:
        return m.group(1), m.group(2)
    return header, ""

def pick(fields, field, unit=None):
    """First value for `field`; if `unit` given, the first value containing it."""
    vals = fields.get(field)
    if not vals:
        return None
    if unit is not None:
        for v in vals:
            if v and unit in v:
                return to_number(v)
    return to_number(vals[0])

def build_row(name, header, fields):
    btype, orbits = header_info(header)
    lo, hi = split_two(fields.get("Orbital velocity", [None])[0] if fields.get("Orbital velocity") else None)
    g1, g2 = split_two(fields.get("Surface gravity", [None])[0] if fields.get("Surface gravity") else None)
    tmin1, tmin2 = split_temp(fields.get("Temperaturemin", [None])[0] if fields.get("Temperaturemin") else None)
    tmax1, tmax2 = split_temp(fields.get("Temperaturemax", [None])[0] if fields.get("Temperaturemax") else None)

    def raw(field):
        v = fields.get(field)
        return v[0] if v else None

    return {
        "name": name,
        "type": btype,
        "orbits": orbits,
        "semi_major_axis_m": pick(fields, "Semi-major axis"),
        "apoapsis_m": pick(fields, "Apoapsis"),
        "periapsis_m": pick(fields, "Periapsis"),
        "eccentricity": pick(fields, "Orbital eccentricity"),
        "inclination_deg": pick(fields, "Orbital inclination"),
        "arg_periapsis_deg": pick(fields, "Argument of periapsis"),
        "long_asc_node_deg": pick(fields, "Longitude of the ascending node"),
        "mean_anomaly_rad": pick(fields, "Mean anomaly"),
        "orbital_period_s": pick(fields, "Sidereal orbital period"),
        "synodic_period_s": pick(fields, "Synodic orbital period"),
        "orbital_velocity_min_mps": lo,
        "orbital_velocity_max_mps": hi,
        "longest_time_eclipsed_s": pick(fields, "Longest time eclipsed"),
        "radius_m": pick(fields, "Equatorial radius"),
        "circumference_m": pick(fields, "Equatorial circumference"),
        "surface_area_m2": pick(fields, "Surface area"),
        "mass_kg": pick(fields, "Mass"),
        "mu_m3s2": pick(fields, "Standard gravitational parameter"),
        "density_kgm3": pick(fields, "Density"),
        "surface_gravity_mps2": g1,
        "surface_gravity_g": g2,
        "escape_velocity_mps": pick(fields, "Escape velocity"),
        "rotation_period_s": pick(fields, "Sidereal rotation period"),
        "solar_day_s": pick(fields, "Solar day"),
        "rotational_velocity_mps": pick(fields, "Sidereal rotational velocity"),
        "synchronous_orbit_km": pick(fields, "Synchronous orbit"),
        "soi_m": pick(fields, "Sphere of influence"),
        "atmosphere_present": raw("Atmosphere present"),
        "atmospheric_pressure_kpa": pick(fields, "Atmospheric pressure", unit="kPa"),
        "atmospheric_pressure_atm": pick(fields, "Atmospheric pressure", unit="atm"),
        "atmospheric_height_m": pick(fields, "Atmospheric height", unit="m"),
        "temperature_min_c": tmin1,
        "temperature_min_k": tmin2,
        "temperature_max_c": tmax1,
        "temperature_max_k": tmax2,
        "oxygen_present": raw("Oxygen present"),
        "science_surface": pick(fields, "Surface"),
        "science_splashed": pick(fields, "Splashed"),
        "science_lower_atm": pick(fields, "Lower atmosphere"),
        "science_upper_atm": pick(fields, "Upper atmosphere"),
        "science_near_space": pick(fields, "Near space"),
        "science_outer_space": pick(fields, "Outer space"),
        "science_recovery": pick(fields, "Recovery"),
    }

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    # old CSV gives the one column the wiki pages lack (axial tilt)
    old = pd.read_csv(OLD_CSV)
    old.columns = [str(c).strip().lower().replace(" ", "_") for c in old.columns]
    tilt = {}
    for n, v in zip(old["name"].astype(str), old["axial_tilt"]):
        tilt[n] = to_number(clean(v))

    rows = []
    for i, name in enumerate(BODIES):
        try:
            header, fields = parse_body(name)
            row = build_row(name, header, fields)
            row["axial_tilt_deg"] = tilt.get(name)   # merged from old CSV
            rows.append(row)
            print(f"[{i+1:2d}/{len(BODIES)}] {name:8s}  fields={len(fields)}")
        except Exception as e:   # noqa: BLE001 - keep going, report per-body
            print(f"[{i+1:2d}/{len(BODIES)}] {name:8s}  FAILED: {e}")
        time.sleep(0.5)

    df = pd.DataFrame(rows)
    # move the identity columns to the front
    front = ["name", "type", "orbits"]
    df = df[[c for c in front if c in df.columns] + [c for c in df.columns if c not in front]]
    df.to_csv(OUT_CSV, index=False)
    print(f"\nwrote {OUT_CSV}  ({len(df)} bodies, {len(df.columns)} columns)")

if __name__ == "__main__":
    main()
