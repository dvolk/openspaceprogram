#!/usr/bin/env python3
# sci_dist.py -- science distance fields for system JSONs.
#
# Writes two per-body keys:
#   transfer_dv      [m/s]  the useful approach leg:
#                           planet (orbits the star)  home -> planet
#                           moon (orbits anything else) parent -> moon
#                           home / root               0
#                           Display + path composition (moons of moons later).
#   science_mult     [1..3] score weight, quantized to 0.1 (half-up).
#                           Baked at generation: the game reads this and
#                           does NOT recompute it.
#
# science_mult default = lerp(1, 3, pathDv / maxPathDv) over every body, where
# pathDv is home -> target = sum of transfer_dv along the chain from target up
# to home (planets' transfer_dv is already the home hop, so the sum stops
# there). The star/root has no transfer edge; it is scored at a flat 2.0.
# Non-home bodies are floored at 1.1: a separate body never ties home.
# Sibling hops with degenerate (near-equal SMA) Hohmann cost use the
# co-orbital phasing term instead (utils/sci_phase.py, issue #128).
# A body that should sit off the computed default (e.g. a trojan's mult
# tuned above its drift dv for destination value) is an OVERRIDE in the
# generator (gen_systems.py KSP_SCIENCE_MULT_OVERRIDES), not a post-
# generation JSON edit.
#
# SMa from the mean angular rate (Kepler III), matching system.cpp.
import hashlib
import math
import sys

from sci_phase import (is_co_orbital, orb_w, phase_gap_rad, phasing_delta_v)

G = 6.674e-11
K_DIST_MULT_MIN = 1.0
K_DIST_MULT_MAX = 3.0
K_NONHOME_MULT_FLOOR = 1.1


def phase0(name, salt):
    # Deterministic pseudo-random angle ~[0, 2pi] seeded by body name
    # (#141 epoch spin phases). sha256, not hash(): stable across runs
    # and machines. Shared by both system generators on purpose: the same
    # body name must draw the same angle everywhere.
    h = hashlib.sha256((salt + '|' + name).encode()).digest()
    return int.from_bytes(h[:8], 'big') / 2**64 * 2.0 * math.pi


def hohmann_delta_v(r1, r2, mu):
    if r1 <= 0.0 or r2 <= 0.0 or mu <= 0.0:
        return 0.0
    a = (r1 + r2) / 2.0
    v1c = math.sqrt(mu / r1)
    v2c = math.sqrt(mu / r2)
    v1t = math.sqrt(mu * (2.0 / r1 - 1.0 / a))
    v2t = math.sqrt(mu * (2.0 / r2 - 1.0 / a))
    return abs(v1t - v1c) + abs(v2t - v2c)


def _quantize_01(m):
    """Half-up to 0.1 (matches C++ lround on positives; not bankers)."""
    return math.floor(m * 10.0 + 0.5) / 10.0


def score_dist_mult(dv, max_dv):
    if max_dv <= 0.0:
        return K_DIST_MULT_MIN
    t = min(1.0, max(0.0, dv / max_dv))
    m = K_DIST_MULT_MIN + (K_DIST_MULT_MAX - K_DIST_MULT_MIN) * t
    return _quantize_01(m)


class Node:
    __slots__ = ("name", "parent", "children", "sma", "mu", "radius", "body")

    def __init__(self, body):
        self.body = body
        self.name = body["name"]
        self.parent = None
        self.children = []
        self.sma = 0.0
        self.mu = G * float(body.get("mass") or 0.0)
        self.radius = float(body.get("radius") or 0.0)


def build_nodes(bodies):
    """Wire the parent/child tree + SMa (Kepler III). Returns name -> Node."""
    by = {}
    for b in bodies:
        name = b["name"]
        if name in by:
            raise ValueError("sci_dist: duplicate body name %r" % name)
        by[name] = Node(b)
    for b in bodies:
        parent_name = b.get("orbits") or ""
        if not parent_name:
            continue
        n = by[b["name"]]
        p = by[parent_name]
        n.parent = p
        p.children.append(n)
        w = float((b.get("inertial") or {}).get("orb_ang_speed") or 0.0)
        n.sma = (p.mu / (w * w)) ** (1.0 / 3.0) if w != 0.0 else 0.0
    return by


def body_sma(body, parent):
    """SMa [m] from mean angular rate (Kepler III), matching system.cpp."""
    w = float((body.get("inertial") or {}).get("orb_ang_speed") or 0.0)
    mu = G * float(parent.get("mass") or 0.0)
    return (mu / (w * w)) ** (1.0 / 3.0) if w != 0.0 else 0.0


def _sibling_leg_dv(home_body, home_sma, body, sma, mu_parent):
    """Home -> sibling-planet heliocentric hop [m/s]. A Hohmann between
    near-equal SMAs is degenerate (the phase gap never closes), so those
    hops price the phasing drift instead (sci_phase.py, issue #128)."""
    if is_co_orbital(home_sma, sma):
        return phasing_delta_v(sma, orb_w(body), phase_gap_rad(home_body, body))
    return hohmann_delta_v(home_sma, sma, mu_parent)


def transfer_dv(node, home):
    """The approach leg [m/s]: home->planet for planets, parent->moon for moons."""
    if node is home or node.parent is None:
        return 0.0
    # Planet under the star (same parent as home): the heliocentric hop.
    if node.parent.parent is None and home.parent is node.parent:
        return _sibling_leg_dv(home.body, home.sma, node.body, node.sma,
                               node.parent.mu)
    # Moon (or moon-of-moon): last leg from the parent's parking orbit. A
    # PLANET reaching this line means home is not a planet of the star;
    # stamp_transfer_dv warns about that (same bodies, same fall-through).
    return hohmann_delta_v(node.parent.radius, node.sma, node.parent.mu)


def stamp_transfer_dv(body, parent, home):
    """Set body['transfer_dv'] at creation. parent is a body dict (None = root);
    home is the home body dict (or None). Safe to call again (overwrite)."""
    if parent is None or (home is not None
                          and body.get("name") == home.get("name")):
        body["transfer_dv"] = 0.0
        return
    sma = body_sma(body, parent)
    mu = G * float(parent.get("mass") or 0.0)
    parent_is_star = not parent.get("orbits")
    if (parent_is_star and home is not None
            and home.get("orbits") == parent.get("name")):
        body["transfer_dv"] = round(
            _sibling_leg_dv(home, body_sma(home, parent), body, sma, mu), 1)
        return
    if parent_is_star:
        # A planet reaching here means home is not a planet of this star (a
        # moon home, or no home), so there is no heliocentric hop to price and
        # the moon-leg formula below silently prices a hop out of the star's
        # own radius: ~34 km/s for every planet in ksp_system.json, Mun->Kerbin
        # included. Warn, but keep the number -- the value model is unchanged.
        sys.stderr.write(
            "sci_dist: home %s is not a planet of %s, so %s has no heliocentric"
            " hop: priced with the moon-leg formula (from %s's radius)\n"
            % (home.get("name") if home else "<none>", parent.get("name"),
               body.get("name"), parent.get("name")))
    body["transfer_dv"] = round(
        hohmann_delta_v(float(parent.get("radius") or 0.0), sma, mu), 1)


def path_dv(home, target):
    """Home -> target path cost [m/s]: sum of transfer_dv up the chain to home.

    A planet's transfer_dv is already the home hop, so the sum stops there
    (Ike = (Duna->Ike) + (home->Duna); Mun = (home->Mun))."""
    if home is target or target.parent is None:
        return 0.0
    total = 0.0
    n = target
    while n is not None and n is not home:
        total += transfer_dv(n, home)
        if n.parent is not None and n.parent.parent is None:
            break   # n orbits the star: its leg already came from home
        n = n.parent
    return total


def stamp_science_mults(doc):
    """Set science_mult on every body (needs the full system for max path)."""
    assert _quantize_01(K_NONHOME_MULT_FLOOR) == K_NONHOME_MULT_FLOOR
    by = build_nodes(doc["bodies"])
    home_name = doc.get("home") or ""
    home = by.get(home_name)
    if home is None:
        for n in by.values():
            if n.parent is not None:
                home = n
                break
    dvs = {name: path_dv(home, n) for name, n in by.items()} if home else {}
    max_dv = max(dvs.values()) if dvs else 0.0
    for name, n in by.items():
        body = n.body
        if "transfer_dv" not in body:
            body["transfer_dv"] = round(
                transfer_dv(n, home) if home else 0.0, 1)
        # The star/root has no transfer edge from home; score it at a flat
        # 2.0 (generator-overridable like every other mult).
        if n.parent is None and n is not home:
            body["science_mult"] = 2.0
        else:
            m = score_dist_mult(dvs.get(name, 0.0), max_dv)
            # A separate body never ties home, whatever the geometry.
            if n is not home and m < K_NONHOME_MULT_FLOOR:
                m = K_NONHOME_MULT_FLOOR
            body["science_mult"] = m
    return by


def stamp_system(doc):
    """Add both science fields to every body (call once the list is complete)."""
    by = build_nodes(doc["bodies"])
    home_name = doc.get("home") or ""
    home = by.get(home_name)
    home_body = home.body if home else None
    for b in doc["bodies"]:
        parent_name = b.get("orbits") or ""
        parent_body = None
        if parent_name:
            for p in doc["bodies"]:
                if p.get("name") == parent_name:
                    parent_body = p
                    break
        stamp_transfer_dv(b, parent_body, home_body)
    stamp_science_mults(doc)
    return doc


def annotate(doc):
    """CLI / fixup: recompute both fields on every body in `doc` (in place)."""
    stamp_system(doc)
    return doc


def check_fields(doc, path):
    """Loader/UI invariants on the COMMITTED fields (no recompute): every
    body carries a finite science_mult > 0; non-home orbiting bodies clear
    the floor; transfer_dv >= 0 and is 0 only for home/root (the Atlas
    renders 0 as 'no approach'). Returns the list of violations."""
    home = doc.get("home") or ""
    bad = []
    for b in doc.get("bodies", []):
        name = b.get("name", "?")
        m = b.get("science_mult")
        if not isinstance(m, (int, float)) or not math.isfinite(m) or m <= 0.0:
            bad.append("%s: science_mult %r not finite > 0" % (name, m))
        elif b.get("orbits") and name != home and m < K_NONHOME_MULT_FLOOR:
            bad.append("%s: non-home science_mult %r below floor %r"
                       % (name, m, K_NONHOME_MULT_FLOOR))
        dv = b.get("transfer_dv", 0.0)
        if not isinstance(dv, (int, float)) or not math.isfinite(dv) or dv < 0.0:
            bad.append("%s: transfer_dv %r not finite >= 0" % (name, dv))
        elif dv == 0.0 and b.get("orbits") and name != home:
            bad.append("%s: transfer_dv 0 but not home (Atlas: 'no approach')"
                       % name)
    return bad


def self_test():
    """The moon-home fall-through must warn, and must not change the number."""
    import contextlib
    import io
    star = {"name": "Star", "mass": 1.0e26, "radius": 1.0e8}
    planet = {"name": "Pla", "orbits": "Star",
              "inertial": {"orb_ang_speed": 1.0e-7}}
    moon = {"name": "Moon", "orbits": "Pla",
            "inertial": {"orb_ang_speed": 1.0e-5}}
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        doc = stamp_system({"home": "Moon", "bodies": [star, planet, moon]})
    dv = {b["name"]: b["transfer_dv"] for b in doc["bodies"]}
    # The number is still the nonsense one: a Hohmann out of the star's own
    # radius. The warning is the fix, not a behaviour change.
    want = round(hohmann_delta_v(star["radius"], body_sma(planet, star),
                                 G * star["mass"]), 1)
    assert dv["Pla"] == want, (dv["Pla"], want)
    assert "Moon" in err.getvalue() and "Pla" in err.getvalue(), err.getvalue()
    # A planet home -- every shipped system -- stays silent.
    err2 = io.StringIO()
    with contextlib.redirect_stderr(err2):
        doc2 = stamp_system({"home": "Pla",
                             "bodies": [dict(star), dict(planet), dict(moon)]})
    assert err2.getvalue() == "", err2.getvalue()
    assert doc2["bodies"][1]["transfer_dv"] == 0.0    # Pla is home
    print("sci_dist self-test ok: moon-home warns, planet-home silent")


if __name__ == "__main__":
    # Preview table: sci_dist.py <system.json> [...]
    # --check: invariants on the committed fields, no recompute, no writes.
    # --self-test: the moon-home warning, on a synthetic 3-body system.
    import json
    if sys.argv[1:2] == ["--self-test"]:
        self_test()
        sys.exit(0)
    if sys.argv[1:2] == ["--check"]:
        failed = False
        for path in sys.argv[2:]:
            with open(path) as f:
                doc = json.load(f)
            bad = check_fields(doc, path)
            if bad:
                failed = True
                print("CHECK FAIL %s:" % path)
                for line in bad:
                    print("    %s" % line)
            else:
                print("CHECK OK   %s" % path)
        sys.exit(1 if failed else 0)
    for path in sys.argv[1:]:
        with open(path) as f:
            doc = json.load(f)
        annotate(doc)
        by = build_nodes(doc["bodies"])
        home = doc.get("home")
        rows = []
        for name, n in by.items():
            b = n.body
            rows.append((b["science_mult"], b["transfer_dv"], name,
                         n.parent.name if n.parent else "-",
                         n.sma / 1000.0, name == home))
        rows.sort()
        print("\n=== %s  home=%s  bodies=%d ===" %
              (path, home, len(by)))
        print("%-16s %-12s %12s %12s %6s  %s" %
              ("body", "parent", "sma (km)", "xfer_dv(m/s)", "mult", "kind"))
        for mult, xdv, name, parent, sma_km, is_home in rows:
            kind = "home" if is_home else ("root" if parent == "-" else "body")
            print("%-16s %-12s %12.1f %12.1f %6.1f  %s" %
                  (name, parent, sma_km, xdv, mult, kind))
