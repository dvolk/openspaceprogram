#!/usr/bin/env python3
"""Reproduce every number in ../atmo_roadmap.md. Read-only.

Three tables:
  1. what every air body in both shipped systems asserts at its datum, resolved
     through the loader's own arithmetic;
  2. those datums against the NASA fact sheets already vendored in utils/rss/html/
     (scraped, not transcribed -- the values in the report are this script's output);
  3. the three Kerbin datum options from section 4, and the dynamic pressure each
     leaves at the body's air top.

CAVEAT worth stating rather than hiding: the gas table and the hydrostatic
integration below are MIRRORED from src/atmo.h, not read from it. If atmo.h moves
and this does not, this script's numbers go stale in the same way the report's do.
"""
import json, math, os, re, html

ROOT = os.path.dirname(os.path.abspath(__file__))          # .../reports/<this>/support
REPO = os.path.join(ROOT, "..", "..", "..")
SYSS = ("res/systems/solar_system.json", "res/systems/ksp_system.json")
HTML = "utils/rss/html"
kGasR = 8.314462618

# atmo.h:gasSpecies -- name, M [kg/mol], gamma, oxidizer
SPECIES = {
    "N2": (0.0280134, 1.40, False), "O2": (0.0319988, 1.40, True),
    "CO2": (0.0440095, 1.30, False), "H2": (0.00201588, 1.40, False),
    "He": (0.0040026, 1.667, False), "CH4": (0.0160425, 1.32, False),
    "H2O": (0.0180153, 1.33, False), "Ar": (0.039948, 1.667, False),
}

def resolve_mix(mix):
    """atmo.h:resolveMix -- M linear in mole fraction, gamma from the mole-weighted
    molar heat capacity (heat capacities add, ratios do not)."""
    tot = sum(m["fraction"] for m in mix)
    M = cp = ox = 0.0
    for m in mix:
        mm, g, isox = SPECIES[m["gas"]]
        x = m["fraction"] / tot
        M += x * mm
        cp += x * (kGasR * g / (g - 1.0))
        if isox:
            ox += x * mm
    return M, cp / (cp - kGasR), ox / M

def level_pressure(L, alt, base_p, g, Rs):
    """atmo.h:levelPressure -- log1p so the isothermal limit holds."""
    dh = alt - L["alt"]
    if L["lapse"] == 0.0:
        return base_p * math.exp(-g * dh / (Rs * L["T"]))
    T = L["T"] + L["lapse"] * dh
    if not (T > 0.0):
        return 0.0
    return base_p * math.exp(-(g / (L["lapse"] * Rs)) * math.log1p(L["lapse"] * dh / L["T"]))

class Column:
    """One body's resolved profile: the datum plus the integrate-downward pass
    atmo.h:resolveAtmoLevels does at load."""
    def __init__(self, g, rho0, mix, layers, height, scale_height):
        self.g, self.rho0 = g, rho0
        self.M, self.gamma, self.ox = resolve_mix(mix)
        self.Rs = kGasR / self.M
        if layers:
            self.L = layers
            self.T0, self.p0 = layers[0]["T"], rho0 * self.Rs * layers[0]["T"]
            self.H = None
            self.top = height
        else:
            self.H = scale_height
            self.T0 = g * scale_height * self.M / kGasR   # THE THREE ARE LOCKED
            self.p0 = rho0 * g * scale_height
            self.top = height or 10.0 * scale_height
            self.L = [{"alt": 0.0, "T": self.T0, "lapse": 0.0}]
        self.a0 = math.sqrt(self.gamma * kGasR * self.T0 / self.M)
        self.bases = [self.p0]
        for i in range(1, len(self.L)):
            self.bases.append(level_pressure(self.L[i - 1], self.L[i]["alt"],
                                             self.bases[-1], self.g, self.Rs))
    def at(self, alt):
        if alt >= self.top:
            return 0.0, 0.0, 0.0          # hard zero at the air top
        for i, l in enumerate(self.L):
            hi = self.L[i + 1]["alt"] if i + 1 < len(self.L) else self.top
            if alt < hi:
                T = l["T"] + l["lapse"] * (alt - l["alt"])
                p = level_pressure(l, alt, self.bases[i], self.g, self.Rs)
                return T, p, p / (self.Rs * T)
        return 0.0, 0.0, 0.0
    def eff_H(self):
        """Datum scale height the column implies: Rs T0 / g."""
        return self.Rs * self.T0 / self.g

def sheet(body):
    """Scrape the atmosphere block of utils/rss/html/<body>fact.html. Values we care
    about come from prose lines, so this regexes text, not a table."""
    path = os.path.join(REPO, HTML, body.lower() + "fact.html")
    if not os.path.exists(path):
        return None
    t = re.sub(r"<[^>]+>", " ", open(path, encoding="utf-8", errors="replace").read())
    t = html.unescape(t)
    t = re.sub(r"\s+", " ", t)
    out = {}
    def grab(pat, key, cast=float):
        m = re.search(pat, t, re.I)
        if m:
            out[key] = cast(m.group(1))
    # The sheets write "~65. kg/m^3", "0.016 +/- 0.006", "~214 K" -- so allow the
    # tilde and an error bar between the number and its unit.
    NUM = r"~?(\d+\.?\d*)"
    BAR = r"(?: ?\+/- ?[\d.]+)?"
    grab(r"Surface pressure: ?" + NUM + r" ?bars", "p_pa", lambda s: float(s) * 1e5)
    grab(r"Surface pressure: ?" + NUM + r" ?mb", "p_pa", lambda s: float(s) * 100)
    grab(r"Surface density: ?" + NUM + BAR + r" ?kg", "rho")
    grab(r"Scale height: ?" + NUM + r" ?km", "H", lambda s: float(s) * 1000)
    grab(r"Average temperature: ?" + NUM + r" ?K", "T")
    grab(r"Mean molecular weight: ?" + NUM, "M")
    rng = re.search(r"Diurnal temperature range: ?([\d.]+) K to ([\d.]+) K", t, re.I)
    if rng:
        out["diurnal"] = (float(rng.group(1)), float(rng.group(2)))
    wnd = re.search(r"Wind speeds: ?([\d.]+) to ([\d.]+) m/s", t, re.I)
    if wnd:
        out["wind"] = (float(wnd.group(1)), float(wnd.group(2)))
    return out

def main():
    cols = {}
    print("=" * 100)
    print("1. what each shipped air body asserts at its own datum")
    print("=" * 100)
    hdr = "%-6s %-8s %-9s %7s %8s %8s %8s %7s %8s %11s %7s %8s %9s"
    print(hdr % ("sys", "body", "form", "g", "rho0", "H m", "M g/mol", "gamma",
                 "T0 K", "p0 Pa", "a m/s", "top km", "H_eff m"))
    for f in SYSS:
        d = json.load(open(os.path.join(REPO, f)))
        for b in d["bodies"]:
            a = (b.get("surface") or {}).get("atmosphere")
            if not a or a.get("sea_level_density") is None:
                continue
            c = Column(b["g"], a["sea_level_density"], a["composition"],
                       a.get("layers"), a.get("height"), a.get("scale_height"))
            cols[(f.split("/")[-1][:5], b["name"])] = c
            form = "layers%d" % len(a["layers"]) if a.get("layers") else "exp"
            print(hdr % (f.split("/")[-1][:5], b["name"], form, b["g"],
                         a["sea_level_density"],
                         "%8.0f" % c.H if c.H else "       -",
                         c.M * 1000, c.gamma, c.T0, c.p0, c.a0, c.top / 1000,
                         c.eff_H()))
    print()
    print("=" * 100)
    print("2. real-world datums vs the vendored fact sheet (scraped, not typed)")
    print("=" * 100)
    for name in ("Venus", "Earth", "Mars"):
        key = next(k for k in cols if k[1] == name)
        c = cols[key]
        s = sheet(name)
        print("%s: scraped %s" % (name, sorted(s)))
        def cmp(label, ours, theirs, fmt):
            if theirs is None:
                print("   %-9s " + fmt + "  vs  n/a (the sheet states no such line)") % ours
                return
            off = 100.0 * (ours - theirs) / theirs
            print(("   %-9s " + fmt + "   vs " + fmt + "  %+.1f %%") % (label, ours, theirs, off))
        cmp("T0", c.T0, s.get("T"), "%.2f K  ")
        cmp("p0", c.p0, s.get("p_pa"), "%.0f Pa ")
        cmp("rho0", c.rho0, s.get("rho"), "%.4g  ")
        cmp("M", c.M * 1000, s.get("M"), "%.3f  ")
        cmp("H_eff", c.eff_H(), s.get("H"), "%.0f m  ")
        if "diurnal" in s:
            lo, hi = s["diurnal"]
            print("   %-12s %s K -- the game has no diurnal term, so it is off by the whole range"
                  % ("diurnal", ("%0.0f-%0.0f" % (lo, hi)) if name != "Venus" else "~0"))
        if "wind" in s:
            print("   %-12s %s m/s -- the game has no wind; anemometer reads ship velocity only"
                  % ("wind", "%.1f-%.1f" % s["wind"]))
        print()
    print("=" * 100)
    print("3. Kerbin's datum, three ways (section 4). g 9.81, dry air, rho0 1.225, top 70 km")
    print("=" * 100)
    EARTHL = [{"alt": 0, "T": 288.15, "lapse": -0.0065}, {"alt": 11000, "T": 216.65, "lapse": 0.0},
              {"alt": 20000, "T": 216.65, "lapse": 0.001}, {"alt": 32000, "T": 228.65, "lapse": 0.0028},
              {"alt": 47000, "T": 270.65, "lapse": 0.0}, {"alt": 51000, "T": 270.65, "lapse": -0.0028},
              {"alt": 71000, "T": 214.65, "lapse": -0.002}]
    dry = [{"gas": "N2", "fraction": 0.78084}, {"gas": "O2", "fraction": 0.20946},
           {"gas": "Ar", "fraction": 0.00934}, {"gas": "CO2", "fraction": 0.00033}]
    M, _, _ = resolve_mix(dry)
    g, rho0, top = 9.81, 1.225, 70000.0
    H288 = 288.15 * kGasR / (g * M)
    print("a 288.15 K datum needs scale height %.0f m (today: 5500 m)" % H288)
    A = Column(g, rho0, dry, None, top, 5500)
    B = Column(g, rho0, dry, None, top, H288)
    C = Column(g, rho0, dry, EARTHL, top, None)
    print("%-9s %10s %10s | %10s %10s | %10s %10s" %
          ("alt m", "A T K", "A rho", "B T K", "B rho", "C T K", "C rho"))
    for alt in (0, 2000, 5000, 11000, 20000, 40000, 60000, top - 1):
        Ta, _, da = A.at(alt); Tb, _, db = B.at(alt); Tc, _, dc = C.at(alt)
        print("%-9d %10.2f %10.4g | %10.2f %10.4g | %10.2f %10.4g" % (alt, Ta, da, Tb, db, Tc, dc))
    print("\nresidual air at the top, and q at Kerbin circular orbit:")
    R = next(b["radius"] for b in json.load(
        open(os.path.join(REPO, "res/systems/ksp_system.json")))["bodies"]
        if b["name"] == "Kerbin")
    vc = math.sqrt(g * R * R / (R + top))     # v_circ = sqrt(mu / r), mu = g R^2
    print("  Kerbin radius %.0f km -> circular speed at the %.0f km top: %.0f m/s"
          % (R / 1000, top / 1000, vc))
    for nm, c in (("A H 5500", A), ("B H 8432", B), ("C Earth table", C)):
        _, p, r = c.at(top - 1)
        print("  %-14s rho %10.4g  p %10.4g Pa  q %9.1f Pa" % (nm, r, p, 0.5 * r * vc * vc))

    print()
    print("=" * 100)
    print("4. section 3b: is each fact sheet internally one hydrostatic isothermal column,")
    print("   and how do the giants' 1-bar datums compare to ours?")
    print("=" * 100)
    def extra(body):
        t = re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]+>", " ",
              open(os.path.join(REPO, HTML, body + "fact.html"),
                   encoding="utf-8", errors="replace").read())))
        def g(pat, scale=1.0):
            m = re.search(pat, t, re.I)
            return float(m.group(1)) * scale if m else None
        return dict(rho1=g(r"Density at 1 bar: ?~?(\d+\.?\d*)"),
                    T1=g(r"Temperature at 1 bar: ?~?(\d+\.?\d*) ?K"),
                    H1=g(r"Scale height: ?~?(\d+\.?\d*)(?: ?- ?\d+\.?\d*)? ?km", 1000.0),
                    M=g(r"Mean molecular weight: ?~?(\d+\.?\d*)(?: ?- ?\d+\.?\d*)?"),
                    avgT=g(r"Average temperature: ?~?(\d+\.?\d*) ?K"),
                    # The giants state gravity as "Gravity (mean, 1 bar) (m/s2)",
                    # the rocky planets as "Surface gravity (mean) (m/s2)". Both are
                    # read here so the comparison below does not depend on which one
                    # make_solar_system.py's scrape happens to pick up.
                    gsheet=g(r"(?:Surface gravity|Gravity) ?\([^)]*\) ?\(m/s ?2 ?\) ?(\d+\.?\d*)"))
    grav = {nm: extra(nm.lower())["gsheet"]
            for nm in ("Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune")}
    print("  the sheet's stated gravity vs the g we author (the giants' is stated at 1 bar;")
    print("  make_body()'s G*mass/radius^2 fallback reproduces it -- checked, not assumed)")
    print("  %-8s %10s %10s %8s" % ("body", "sheet g", "our g", "off"))
    for nm in ("Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune"):
        c = cols[("solar", nm)]
        print("  %-8s %10.4g %10.4g %+7.2f %%" % (nm, grav[nm], c.g,
              100 * (c.g - grav[nm]) / grav[nm]))
    print()
    print("  is the sheet self-consistent?  H implied by its own M, T, g  vs  the H it states")
    print("  %-8s %8s %7s %8s %12s %10s %8s" %
          ("body", "M g/mol", "T K", "g m/s2", "H implied", "H stated", "agree"))
    for nm in ("Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune"):
        s = extra(nm.lower()); gg = grav[nm]
        T = s["avgT"] or s["T1"]
        if not (s["M"] and T and s["H1"] and gg):
            print("  %-8s incomplete from the sheet: M %s T %s H %s g %s"
                  % (nm, s["M"], T, s["H1"], gg))
            continue
        imp = kGasR / (s["M"] / 1000.0) * T / gg
        print("  %-8s %8.3f %7.1f %8.3f %12.0f %10.0f %+7.1f %%" %
              (nm, s["M"], T, gg, imp, s["H1"], 100 * (imp - s["H1"]) / s["H1"]))
    print("\n  (Neptune states ranges -- H 19.1-20.3 km, M 2.53-2.69 -- and the row uses the")
    print("   low end of each; with M at 2.69 its own H and T agree inside its own range.)")
    print("\n  our authored giant datums vs the sheet's values AT 1 BAR (our datum for a giant)")
    print("  %-8s %11s %9s %8s | %9s %9s %8s | %8s %9s %8s" %
          ("body", "rho 1 bar", "ours", "off", "H sheet", "ours", "off", "T 1 bar", "our T0", "off"))
    for nm in ("Jupiter", "Saturn", "Uranus", "Neptune"):
        s = extra(nm.lower()); c = cols[("solar", nm)]
        def d(a, bb):
            return "%+7.1f %%" % (100 * (a - bb) / bb) if bb else "      n/a"
        print("  %-8s %11s %9.4g %8s | %9s %9s %8s | %8s %9.1f %8s" %
              (nm, s["rho1"], c.rho0, d(c.rho0, s["rho1"]), s["H1"], c.H, d(c.H, s["H1"]),
               s["T1"], c.T0, d(c.T0, s["T1"])))
    print("\n  bodies the guard must NOT skip for the wrong reason: Pluto states 13 microbar,")
    print("  surface T 24-38 K, H ~18 km (50 km above 30 km), M ~28, N2 99/CH4 0.5 -- and we")
    print("  author no sea_level_density for it at all; Titan has no *fact.html to author from.")
    print("\nshellEdge law (bodylimits.h): max(100 km, 1.2 x top) -> %.0f km for a 70 km top"
          % (max(100e3, 1.2 * top) / 1000))

main()
