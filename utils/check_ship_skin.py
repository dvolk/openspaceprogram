#!/usr/bin/env python3
"""#153 as a BUILD refusal: a surface attach whose contact is not on the parent's skin.

A "surface" edge says a part is bolted to the side of another part. The shorthand
("angle" + "z", no "point"/"normal") computes where that bolt lands from the parent's
cross-section -- and the loader takes the cross-section from the parent's mesh BOUNDING
BOX (src/shipdef.cpp:1138 meshHalfSpan), which is the widest slice the parent ever has,
at any height. For a cylinder that is the skin. For a cone it is a lie at every height
except the widest one: `capsule` is 1.0 m across at its base and 0.25 m at its top, so a
part bolted at mid-height lands 0.375 m out in the air, and the rudder on `skiff`'s jet
cone floats 0.17 m off the hull it claims to touch.

That is not only cosmetic. The part's convex hull joins the ship's compound shape, so a
floating part is a floating collision blob and an extra term in the drag silhouette, and
the parallel-axis term in the principal-axis solve uses the (too far out) child position
-- on `skiff`, `skiff_gear` and `skiff_trim` that is a moment arm ~0.17 m longer than
authored. 24 of the 77 surface edges the shipped roster weighs sit outside tolerance today;
each is waived below at the value measured when this gate was written, so the gate catches
the NEXT one instead of re-litigating those.

WHAT THIS FILE DOES: it re-computes every shipped surface edge with the same ray rule the
loader uses (ellipseContact, shipdef.cpp:1458) and weighs the result against the parent
mesh's ACTUAL skin along that ray at that height. It is a second spelling of the rule, not
a call into it -- the same reason utils/atmo_profile.py re-spells atmo.h: a gate built on
the loader's own numbers could only ever catch a mistyped authoring value, while an
independent one can also catch the rule drifting.

Only ROUND parents are weighed. The loader's ellipse is a surface of revolution: on a plate
it lands on the face and the tip exactly (#457) and buries a diagonal mount inside the
plate on purpose, because the alternative -- a box -- would swing the contact normal 90 deg
as the azimuth crossed a corner. A radial skin measure would call `skiff`'s ailerons buried
by 2.8 m when they are on the wing's face plane, so plate parents are counted as skipped.

THE SECOND MECHANISM, weighed here too (and named on #153): synthesizeNodes (shipdef.h:328)
puts a part's surface node at -radius, again the WIDEST radius rather than the skin at the
node's own height. The child's node is what lands on the contact, so a part whose node
stands off its own skin hangs that far OUTSIDE the contact no matter how honest the contact
is -- 23 of the 70 round-cross-section part types do, up to 1.12 m for
`nose_cap_r2.25h1.125`, 0.84 m for `capsule_r2.25h4.5`, 0.24 m for plain `engine`. Fixing
it moves geom_fingerprint (nodes are hashed, #456), so every existing save would report a
reshaped ship; that is why it is a decision and not a drive-by.

WAIVERS are VALUE-bound, exactly like check_atmo_band.py's: each row carries the gap it was
written against. A waived row that gets WORSE fails; a waived row that no longer offends
fails too, so the list cannot quietly fossilise once #153's rule lands. Regenerate the two
lists with --emit-allow after changing the rule or the data.

TOLERANCE: 2 cm. Below that the difference is inside the chord error of the mesh's own
vertex spacing (the skin here is interpolated between vertex z-levels) and inside what a
player can see; above it, `skiff`'s elevators, `power_test`'s RTG and company are already
visible. --tol overrides it for an author asking "what if".

Usage:  python3 utils/check_ship_skin.py                 # gate: ships + catalog
        python3 utils/check_ship_skin.py --verbose       # every weighed row
        python3 utils/check_ship_skin.py --emit-allow    # the WAIVERS literals, sorted
        python3 utils/check_ship_skin.py --self-test     # the checker's own pins
Exit:   0 clean, 1 offenders (or a stale waiver), 2 bad input.
"""
import argparse
import collections
import contextlib
import io
import json
import math
import os
import sys

TOL_M = 0.02

# ---------------------------------------------------------------------------
# The mesh, read the way the loader reads it
# ---------------------------------------------------------------------------

def read_mesh_vertices(path):
    """The 'v' lines of a .obj, as src/shipdef.cpp:264-289 scans them (an indented
    vertex line still counts; a line that will not parse costs an extent, not a
    load failure). Returns [] for a missing file: the catalog never required one."""
    vs = []
    try:
        with open(path, "r", errors="replace") as f:
            for line in f:
                s = line.lstrip(" \t")
                if not s.startswith("v ") and not s.startswith("v\t"):
                    continue
                t = s.split()
                if len(t) < 4:
                    continue
                try:
                    vs.append((float(t[1]), float(t[2]), float(t[3])))
                except ValueError:
                    continue
    except OSError:
        return []
    return vs


def half_span(vs):
    """Symmetrised half-extents, like PartDef::mesh_min/mesh_max: the LARGER half on
    both sides, because the part origin is the stack axis."""
    hx = max(max(abs(v[0]) for v in vs), 0.0)
    hy = max(max(abs(v[1]) for v in vs), 0.0)
    hz = max(max(abs(v[2]) for v in vs), 0.0)
    return hx, hy, hz


def ray_levels(vs, ca, sa):
    """z-level -> farthest extent along the ray (ca, sa). The skin along the attach
    azimuth is what a bolt has to land on; a cone's max radius and its extent along a
    tilted ray are the same number, but a plate's are not."""
    lv = collections.defaultdict(float)
    for x, y, z in vs:
        d = x * ca + y * sa
        k = round(z, 9)
        if d > lv[k]:
            lv[k] = d
    return sorted(lv.items())


def interp_levels(lv, z):
    """Linear in z between vertex levels, clamped outside them. A mesh samples its
    own silhouette densely, so the chord error of this line is the 2 cm tolerance."""
    if not lv:
        return 0.0
    if z <= lv[0][0]:
        return lv[0][1]
    if z >= lv[-1][0]:
        return lv[-1][1]
    for i in range(1, len(lv)):
        if lv[i][0] >= z:
            z0, v0 = lv[i - 1]
            z1, v1 = lv[i]
            if z1 == z0:
                return v1
            f = (z - z0) / (z1 - z0)
            return v0 + f * (v1 - v0)
    return lv[-1][1]


def ellipse_t(hx, hy, ca, sa):
    """ellipseContact's ray length, spelled as src/shipdef.cpp:1458 spells it -- the
    round shortcut included, because that shortcut is what makes every shipped contact
    land where it does."""
    if abs(hx - hy) <= 1e-9 * max(hx, hy):
        return hx
    if ca == 0.0:
        return hy
    if sa == 0.0:
        return hx
    return (hx * hy) / math.sqrt(ca * ca * hy * hy + sa * sa * hx * hx)


# ---------------------------------------------------------------------------
# Ship edges
# ---------------------------------------------------------------------------

def part_ids(entries):
    """Part ids, defaulting them the way shipdef.cpp:1005-1010 does (part name plus a
    per-name counter), so a waiver key names the same row after a reload."""
    auto = collections.Counter()
    ids = []
    for pv in entries:
        pid = pv.get("id", "")
        if not pid:
            name = pv["part"]
            auto[name] += 1
            pid = "%s_%d" % (name, auto[name])
        ids.append(pid)
    return ids


def weigh_ships(ships, defs, mesh_dir, tol, verbose):
    """Every surface edge in `ships`. Returns (rows, skipped): a row is a dict, and a
    row's `gap` is the contact radius minus the parent's skin along the ray -- positive
    floats, negative is buried inside the parent.

    A NON-round parent is skipped rather than weighed. The loader's ellipse model is a
    surface of revolution: for a plate it lands on the face and on the tip exactly
    (#457, pinned in --self-test) but buries a diagonal mount inside the plate on
    purpose, because the alternative -- a box -- would swing the contact normal 90 deg
    as the azimuth crossed a corner. Weighing a plate against a radial skin would call
    `skiff`'s ailerons buried by 2.8 m when they are on the wing's face plane."""
    rows = []
    skipped = 0
    for slug, doc in sorted(ships.items()):
        entries = doc.get("parts", [])
        ids = part_ids(entries)
        byid = dict(zip(ids, range(len(entries))))
        for i, pv in enumerate(entries):
            if pv.get("attach") != "surface":
                continue
            pid = ids[i]
            parent = pv.get("parent")
            if parent is None:
                # The loader defaults to the previous entry; weigh the same default.
                parent = ids[i - 1] if i > 0 else None
            if parent is None or parent not in byid:
                skipped += 1
                continue
            ppv = entries[byid[parent]]
            pdef = defs.get(ppv["part"])
            if pdef is None:
                skipped += 1
                continue
            vs = read_mesh_vertices(os.path.join(mesh_dir, pdef.get("mesh", "")))
            if not vs:
                # No mesh: the loader falls back to the authored radius, which is the
                # same number the contact is built from. Nothing to weigh.
                skipped += 1
                continue
            hx, hy, hz = half_span(vs)
            if abs(hx - hy) > 1e-9 * max(hx, hy):
                skipped += 1
                continue
            z = float(pv.get("z", 0.0))
            if "point" in pv and "normal" in pv:
                px, py = float(pv["point"][0]), float(pv["point"][1])
                r = math.hypot(px, py)
                if r <= 1e-12:
                    skipped += 1
                    continue
                ca, sa = px / r, py / r
                kind = "explicit"
            else:
                a = math.radians(float(pv.get("angle", 0.0)))
                ca, sa = math.cos(a), math.sin(a)
                r = ellipse_t(hx, hy, ca, sa)
                kind = "angle"
            lv = ray_levels(vs, ca, sa)
            skin = interp_levels(lv, z)
            rows.append({"ship": slug, "part": pv["part"], "id": pid,
                         "parent": parent, "parent_part": ppv["part"], "kind": kind,
                         "z": z, "angle": float(pv.get("angle", 0.0)),
                         "offset": float(pv.get("offset", 0.0)),
                         "contact": r, "skin": skin, "gap": r - skin,
                         "hz": hz, "testship": bool(doc.get("testship", False))})
    return rows, skipped


def weigh_nodes(defs, mesh_dir, tol):
    """How far each part type's OWN surface-attach node sits outside its own skin. The
    child's node is what lands on the contact, so this much air is added to every mount
    of that part, whichever parent it is bolted to.

    Only round cross-sections are weighed, for the same reason as the ship edges: a plate
    part's side node is a placement on its own face-plane, and the radial measure says
    nothing about it."""
    rows = []
    for name, d in sorted(defs.items()):
        if d.get("fuel_link"):
            continue
        vs = read_mesh_vertices(os.path.join(mesh_dir, d.get("mesh", "")))
        if not vs:
            continue
        hx, hy, _ = half_span(vs)
        if abs(hx - hy) > 1e-9 * max(hx, hy):
            continue
        nodes = d.get("nodes") or []
        srf = [n for n in nodes if n.get("surface")]
        if not srf:
            # synthesizeNodes: side node at -radius, at z = 0.
            srf = [{"pos": [-float(d.get("radius", 0.0)), 0.0, 0.0]}]
        lv = ray_levels(vs, -1.0, 0.0)
        pos = srf[0]["pos"]
        skin = interp_levels(lv, float(pos[2]))
        off = abs(float(pos[0]))
        rows.append({"part": name, "node_z": float(pos[2]), "node_off": off,
                     "skin": skin, "gap": off - skin, "authored": bool(nodes)})
    return rows


# ---------------------------------------------------------------------------
# Waivers (regenerate with --emit-allow)
# ---------------------------------------------------------------------------

CONTACT_WAIVERS = {
    "power_test:rtg_1->capsule_1": 0.8438,
    "surplus_test:rtg_1->capsule_1": 0.8438,
    "baropod:barometer_1->capsule_1": 0.3750,
    "thermopod:thermometer_1->capsule_1": 0.3750,
    "skiff_gear:nose_gear->pod": 0.3125,
    "endurance_racer:tank_r1h2_4->engine_1": 0.2405,
    "endurance_racer:tank_r1h2_3->engine_1": 0.2405,
    "endurance_racer:tank_r1h2_2->engine_1": 0.2389,
    "skiff:elev_l->engine": 0.1666,
    "skiff:elev_r->engine": 0.1666,
    "skiff_gear:elev_l->engine": 0.1666,
    "skiff_gear:elev_r->engine": 0.1666,
    "skiff_trim:elev_l->engine": 0.1666,
    "skiff_trim:elev_r->engine": 0.1666,
    "finring:ring_0->engine": 0.1666,
    "finring:ring_1->engine": 0.1666,
    "finring:ring_2->engine": 0.1666,
    "finring:ring_3->engine": 0.1666,
    "skiff:rudder->engine": 0.0597,
    "skiff_gear:rudder->engine": 0.0597,
    "skiff_trim:rudder->engine": 0.0597,
    "skiff:fin->engine": 0.0358,
    "skiff_gear:fin->engine": 0.0358,
    "skiff_trim:fin->engine": 0.0358,
}
NODE_WAIVERS = {
    "nose_cap_r2.25h1.125": 1.1250,
    "capsule_r2.25h4.5": 0.8438,
    "nose_cap_r1.5h0.75": 0.7500,
    "adapter_r1to2.25": 0.6250,
    "adapter_r2.25to1": 0.6250,
    "capsule_r1.5h3": 0.5625,
    "engine_r2.25h4.5": 0.5375,
    "orbital_engine_r2.25h2.25": 0.5375,
    "chute": 0.5000,
    "nose_cap": 0.5000,
    "adapter_r1.5to2.25": 0.3750,
    "adapter_r2.25to1.5": 0.3750,
    "capsule": 0.3750,
    "engine_r1.5h3": 0.3583,
    "orbital_engine_r1.5h1.5": 0.3583,
    "adapter_r1.5to1": 0.2500,
    "adapter_r1to1.5": 0.2500,
    "engine": 0.2389,
    "jet": 0.2389,
    "nuclear_engine": 0.2389,
    "orbital_engine": 0.2389,
    "capsule_r0.5h1.5": 0.1875,
    "jet_r0.25h1": 0.0597,
}


def gate(label, rows, waivers, key_of, tol, verbose):
    """Rows outside tolerance must appear in `waivers` at the value they were measured
    when the waiver was written; a waiver that no longer offends is reported too."""
    bad = []
    stale = []
    weighed = set()
    for row in rows:
        k = key_of(row)
        gap = row["gap"]
        weighed.add(k)
        if abs(gap) <= tol:
            if k in waivers:
                stale.append((k, waivers[k], gap))
            continue
        w = waivers.get(k)
        if w is None:
            bad.append((row, "not waived"))
        elif gap > w + 1e-4:
            # 1e-4: a waiver is written to the nearest 0.1 mm by --emit-allow.
            bad.append((row, "worse than waived %.4f" % w))
    # A waiver naming a row that is no longer weighed at all (a part renamed, an edge
    # deleted) is as stale as one whose gap closed.
    for k, w in waivers.items():
        if k not in weighed:
            stale.append((k, w, None))
    if verbose:
        for row in rows:
            print("  %-9s %-16s %-22s -> %-20s z=%6.2f ang=%5.0f contact=%6.3f skin=%6.3f gap=%7.3f"
                  % (row.get("ship", row.get("part")), row.get("part"), row.get("id", ""),
                     row.get("parent", row.get("parent_part", "")), row.get("z", row.get("node_z", 0.0)),
                     row.get("angle", 0.0), row.get("contact", row.get("node_off", 0.0)),
                     row["skin"], row["gap"]))
    over = [r for r in rows if abs(r["gap"]) > tol]
    print("%s: %d rows weighed, %d outside %.0f cm, %d waived"
          % (label, len(rows), len(over), tol * 100.0,
             sum(1 for r in over if key_of(r) in waivers)))
    for row, why in bad:
        if "ship" in row:
            where = "%s on %s at z=%.2f" % (row["ship"], row["part"], row["z"])
        else:
            where = "part %s surface node at z=%.2f" % (row["part"], row["node_z"])
        print("  OFFENDER %-40s gap %+.3f m (%s)  [%s: contact %.3f vs skin %.3f]"
              % (key_of(row), row["gap"], why, where,
                 row.get("contact", row.get("node_off", 0.0)), row["skin"]))
    for k, w, gap in stale:
        print("  STALE WAIVER %-37s written against %+.3f m, now %s -- tighten the list"
              % (k, w, "no such row" if gap is None else "%+.3f m" % gap))
    return len(bad) + len(stale)


# ---------------------------------------------------------------------------
# Self-test: the checker's own rule
# ---------------------------------------------------------------------------

def _cone_mesh(path, r_base, r_top, hz, steps=8):
    with open(path, "w") as f:
        for i in range(steps + 1):
            z = -hz + 2.0 * hz * i / steps
            r = r_base + (r_top - r_base) * (i / steps)
            for k in range(4):
                a = math.pi / 2.0 * k
                f.write("v %.6f %.6f %.6f\n" % (r * math.cos(a), r * math.sin(a), z))


def self_test(tmpdir):
    fails = []
    counts = [0]

    def expect(want, got, what):
        counts[0] += 1
        if want != got:
            fails.append("%s: want %r got %r" % (what, want, got))

    def quiet_gate(*a, **kw):
        """The gate's offender lines are the point of some of these pins, so run it with
        its output parked rather than scrolling past the self-test verdict."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            return gate(*a, **kw)

    # A cone: the shorthand lands on the widest slice, the skin is narrower above it.
    cone = os.path.join(tmpdir, "cone.obj")
    _cone_mesh(cone, 1.0, 0.0, 1.0)
    defs = {"cone": {"name": "cone", "mesh": "cone.obj", "radius": 1.0, "height": 2.0}}
    ships = {"conic": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "kid", "attach": "surface", "parent": "c1",
         "angle": 0.0, "z": 0.0}]}}
    rows, skipped = weigh_ships(ships, defs, tmpdir, TOL_M, False)
    expect(1, len(rows), "one surface edge weighed")
    if rows:
        # Box half-span 1.0 at every z; the skin at z=0 is 0.5 on this cone.
        expect(True, abs(rows[0]["contact"] - 1.0) < 1e-9, "contact rides the box radius")
        expect(True, abs(rows[0]["skin"] - 0.5) < 1e-6, "skin is the cone at that height")
        expect(True, abs(rows[0]["gap"] - 0.5) < 1e-6, "a floating row reads +0.5 m")

    # Waivers are value-bound: same row, worse than waived, is an offender.
    key = lambda r: "%s:%s->%s" % (r["ship"], r["id"], r["parent"])
    expect(1, quiet_gate("cone", rows, {"conic:kid->c1": 0.4}, key, TOL_M, False),
           "a gap worse than its waiver fails")
    expect(0, quiet_gate("cone", rows, {"conic:kid->c1": 0.5}, key, TOL_M, False),
           "a gap at its waived value passes")
    expect(1, quiet_gate("cone", [], {"conic:kid->c1": 0.5}, key, TOL_M, False),
           "a waiver that no longer offends is reported")

    # A plate parent is NOT weighed, and the reason is pinned: the ellipse ray lands a
    # face mount on the face and a tip mount on the tip (#457), but a diagonal mount
    # sits inside the plate -- a verdict a radial skin measurement would call an error.
    plate = os.path.join(tmpdir, "plate.obj")
    with open(plate, "w") as f:
        for x in (-2.0, 2.0):
            for y in (-0.05, 0.05):
                for z in (-1.0, 1.0):
                    f.write("v %.6f %.6f %.6f\n" % (x, y, z))
    pdefs = {"plate": {"name": "plate", "mesh": "plate.obj", "radius": 2.0, "height": 2.0}}
    pships = {"plated": {"parts": [
        {"part": "plate", "id": "p1"},
        {"part": "plate", "id": "onface", "attach": "surface", "parent": "p1",
         "angle": 90.0, "z": 0.0},
        {"part": "plate", "id": "ondiag", "attach": "surface", "parent": "p1",
         "angle": 45.0, "z": 0.0}]}}
    prow, pskip = weigh_ships(pships, pdefs, tmpdir, TOL_M, False)
    expect(0, len(prow), "a plate parent is not weighed")
    expect(2, pskip, "and it is counted as skipped")
    vs = read_mesh_vertices(plate)
    hpx, hpy, hpz = half_span(vs)
    k = math.sqrt(0.5)
    expect(True, abs(ellipse_t(hpx, hpy, 0.0, 1.0) - 0.05) < 1e-12, "face ray = thickness")
    expect(True, abs(ellipse_t(hpx, hpy, 1.0, 0.0) - 2.0) < 1e-12, "tip ray = span")
    skin_diag = interp_levels(ray_levels(vs, k, k), 0.0)
    expect(True, ellipse_t(hpx, hpy, k, k) < skin_diag - 1.3,
           "a diagonal plate mount sits inside the plate by design")

    # An explicit point/normal pair is weighed too, and can be buried.
    bships = {"buried": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "deep", "attach": "surface", "parent": "c1",
         "point": [0.1, 0.0, 0.0], "normal": [1.0, 0.0, 0.0]}]}}
    brows, _ = weigh_ships(bships, defs, tmpdir, TOL_M, False)
    expect(1, len(brows), "an explicit edge is weighed")
    expect(True, brows and brows[0]["gap"] < 0.0, "a buried contact reads negative")

    # A part with no mesh cannot be weighed, and is counted rather than sworn at.
    nomesh = {"nomesh": {"parts": [
        {"part": "ghost", "id": "g1"},
        {"part": "ghost", "id": "g2", "attach": "surface", "parent": "g1", "angle": 0.0}]}}
    _, nskip = weigh_ships(nomesh, {"ghost": {"name": "ghost", "mesh": "nope.obj",
                                             "radius": 1.0, "height": 2.0}},
                           tmpdir, TOL_M, False)
    expect(1, nskip, "a meshless parent is skipped, not failed")

    # The node half: a tapered part's synthesized srf node stands off its own skin.
    nrows = weigh_nodes({"cone": {"name": "cone", "mesh": "cone.obj", "radius": 1.0,
                                  "height": 2.0}}, tmpdir, TOL_M)
    expect(1, len(nrows), "one node row")
    expect(True, nrows and abs(nrows[0]["gap"] - 0.5) < 1e-6,
           "the side node at -radius floats by radius - skin(0)")

    return counts[0], fails


# ---------------------------------------------------------------------------

def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--parts", default="res/data/parts.json")
    ap.add_argument("--ships", default="res/ships")
    ap.add_argument("--mesh-dir", default="res", help="root that part 'mesh' paths resolve under")
    ap.add_argument("--tol", type=float, default=TOL_M, help="tolerance in metres")
    ap.add_argument("--verbose", action="store_true", help="print every weighed row")
    ap.add_argument("--emit-allow", action="store_true",
                    help="print the CONTACT_WAIVERS / NODE_WAIVERS literals")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            nchecks, fails = self_test(td)
        for f in fails:
            print("SELF-TEST FAIL " + f)
        print("self-test: %d checks, %d failures" % (nchecks, len(fails)))
        return 1 if fails else 0

    try:
        cat = json.load(open(args.parts))
    except (OSError, ValueError) as e:
        print("check_ship_skin: cannot read %s: %s" % (args.parts, e))
        return 2
    defs = {p["name"]: p for p in cat["parts"]}

    ships = {}
    for nm in sorted(os.listdir(args.ships)):
        if not nm.endswith(".json"):
            continue
        try:
            ships[nm[:-5]] = json.load(open(os.path.join(args.ships, nm)))
        except (OSError, ValueError) as e:
            print("check_ship_skin: cannot read %s: %s" % (nm, e))
            return 2

    rows, skipped = weigh_ships(ships, defs, args.mesh_dir, args.tol, args.verbose)
    nrows = weigh_nodes(defs, args.mesh_dir, args.tol)

    if args.emit_allow:
        print("CONTACT_WAIVERS = {")
        for r in sorted((r for r in rows if abs(r["gap"]) > args.tol),
                        key=lambda r: -abs(r["gap"])):
            print('    "%s:%s->%s": %.4f,' % (r["ship"], r["id"], r["parent"], r["gap"]))
        print("}")
        print("NODE_WAIVERS = {")
        for r in sorted((r for r in nrows if abs(r["gap"]) > args.tol),
                        key=lambda r: -abs(r["gap"])):
            print('    "%s": %.4f,' % (r["part"], r["gap"]))
        print("}")
        return 0

    bad = 0
    print("%d ship defs, %d part defs, %d surface edges weighed "
          "(%d skipped: no mesh, no def, or a plate parent)"
          % (len(ships), len(defs), len(rows), skipped))
    bad += gate("surface contacts", rows, CONTACT_WAIVERS,
                lambda r: "%s:%s->%s" % (r["ship"], r["id"], r["parent"]),
                args.tol, args.verbose)
    bad += gate("surface nodes", nrows, NODE_WAIVERS, lambda r: r["part"],
                args.tol, args.verbose)
    if bad:
        print("check_ship_skin: %d problems -- see #153" % bad)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
