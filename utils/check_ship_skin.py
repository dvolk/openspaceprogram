#!/usr/bin/env python3
"""#153 as a BUILD refusal: a surface attach whose contact is not on the parent's skin.

A "surface" edge says a part is bolted to the side of another part. The shorthand
("angle" + "z", no "point"/"normal") computes where that bolt lands from the parent's
cross-section -- and the loader takes that cross-section from the parent's mesh BOUNDING
BOX (src/shipdef.cpp:1138 meshHalfSpan): the widest half-width the parent reaches at ANY
height, paired with the authored height. For a cylinder those agree. For a cone the pair
is a lie at every height except the widest one: `capsule`'s bounding box says 1.0 m of
half-width, while its skin at z = 0 is 0.625 m, so a part bolted at mid-height lands
0.375 m out in the air, and the rudder on `skiff`'s jet cone floats 0.16 m off the hull it
claims to touch.

That is not only cosmetic. The part's convex hull joins the ship's compound shape, so a
floating part is a floating collision blob and an extra term in the drag silhouette, and
the parallel-axis term in the principal-axis solve uses the (too far out) child position
-- on `skiff`, `skiff_gear` and `skiff_trim` that is a moment arm ~0.17 m longer than
authored. 28 of the 83 surface edges the shipped roster weighs sit outside tolerance today
(res/ships AND e2e/vehicles -- the fixtures are the hulls the e2e cases fly, so they move
with any rule change too); each is waived below at the value measured when this gate was
written, so the gate catches the NEXT one instead of re-litigating those.

WHAT THIS FILE DOES: it re-computes every shipped surface edge with the same ray rule the
loader uses (ellipseContact, shipdef.cpp:1458) and weighs the result against the parent
mesh's ACTUAL skin along that ray at that height. It is a second spelling of the rule, not
a call into it -- the same reason utils/atmo_profile.py re-spells atmo.h: a gate built on
the loader's own numbers could only ever catch a mistyped authoring value, while an
independent one can also catch the rule drifting.

THE SKIN MEASURE: half-spans symmetrised from the mesh vertices (so an off-centre mesh
reads as wide as its widest side and is never called falsely floating), then the outline
support function max(ca*x + sa*y) at the authored height, interpolated between vertex
z-levels and clamped outside them, compared against the loader's ray length. The measure
is therefore lenient by a few percent mid-slope, where vertex rings are far apart --
enough to catch a 0.24 m offset without claiming a mesh bulges. A mesh that runs past its
authored cylinder is normal (engine bells flare) and is not refused on its own; the one
range refusal copied from the loader is a shorthand row whose authored `z` runs past the
end of the parent (shipdef.cpp:1150), which an authored "point" is allowed to do.

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

Usage:  python3 utils/check_ship_skin.py                 # gate: res/ships + e2e/vehicles
        python3 utils/check_ship_skin.py --verbose        # every weighed row
        python3 utils/check_ship_skin.py --emit-allow     # the WAIVERS literals, sorted
        python3 utils/check_ship_skin.py --self-test      # the checker's own pins
Exit:   0 clean, 1 offenders (or a stale waiver, or a malformed def), 2 bad input.
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
    tilted ray are the same number, but a plate's are not.

    This is a SUPPORT function (the tangent distance), not the ray's exit point, so on a
    polygon cross-section it reads up to a few percent LONGER than the surface actually
    is -- the 2 cm tolerance absorbs that along with the z interpolation below.

    The accumulator starts at 0.0, so a level with no vertex on this side of the axis
    reads 0 rather than a negative support: the right verdict for a mesh modelled off the
    axis (the contact then floats by very nearly the whole radius)."""
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


def as_num(v, default=None):
    """A JSON number as float, else `default`. The loader's wantNumber refuses a string
    where python would happily float() it, so a non-number is reported rather than
    guessed -- and a bool is not a number to either of them."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return default
    return float(v)


def as_vec3(v):
    """A 3-number array as a tuple, else None (the loader's parse_vec3 throws on
    anything else, so a half-authored "point" is a load error, not a shorthand)."""
    if not isinstance(v, list) or len(v) != 3:
        return None
    out = []
    for x in v:
        n = as_num(x)
        if n is None:
            return None
        out.append(n)
    return tuple(out)


# ---------------------------------------------------------------------------
# Ship edges
# ---------------------------------------------------------------------------

def part_ids(entries):
    """Part ids, defaulting them the way shipdef.cpp:1005-1010 does (part name plus a
    per-name counter), so a waiver key names the same row after a reload. A malformed
    entry gets a distinct placeholder here and the weigh loop reports it; distinct, so a
    pair of them cannot masquerade as the duplicate-id error the loader raises."""
    auto = collections.Counter()
    ids = []
    for n, pv in enumerate(entries):
        if not isinstance(pv, dict):
            ids.append("#bad%d" % n)
            continue
        pid = pv.get("id", "")
        if not pid:
            name = pv.get("part", "#nopart%d" % n)
            auto[name] += 1
            pid = "%s_%d" % (name, auto[name])
        ids.append(pid)
    return ids


def weigh_ships(ships, defs, mesh_dir):
    """Every surface edge in `ships`. Returns (rows, skipped, refused): a row is a dict
    whose `gap` is the contact radius minus the parent's skin along the ray -- positive
    floats, negative is buried inside the parent. `refused` is a list of lines about data
    the C++ loader itself would reject (a half-authored explicit contact, a `z` off the
    end of the parent, a duplicate part id): weighing it anyway would be weighing a ship
    the game refuses to load.

    A NON-round parent is skipped rather than weighed. The loader's ellipse model is a
    surface of revolution: for a plate it lands on the face and on the tip exactly
    (#457, pinned in --self-test) but buries a diagonal mount inside the plate on
    purpose, because the alternative -- a box -- would swing the contact normal 90 deg as
    the azimuth crossed a corner. Weighing a plate against a radial skin would call
    `skiff`'s ailerons buried by 2.8 m when they are on the wing's face plane.

    A row's `offset` (m) is NOT folded into the gap: it pushes the CHILD off the contact
    along the normal, which is a deliberate standoff, while what has to be on the parent's
    skin is the contact itself. It is carried so an offender line can name it."""
    rows = []
    skipped = 0
    refused = []
    for slug, doc in sorted(ships.items()):
        if not isinstance(doc, dict) or not isinstance(doc.get("parts", []), list):
            refused.append("%s: no \"parts\" array" % slug)
            continue
        entries = doc["parts"]
        ids = part_ids(entries)
        if len(set(ids)) != len(ids):
            # The loader throws "duplicate part id" (shipdef.cpp:1013).
            refused.append("%s: duplicate part id" % slug)
            continue
        byid = {}
        for n, pid in enumerate(ids):
            byid[pid] = n
        for i, pv in enumerate(entries):
            if not isinstance(pv, dict) or "part" not in pv:
                refused.append("%s entry %d: no \"part\"" % (slug, i))
                continue
            if pv.get("attach") != "surface":
                continue
            pid = ids[i]
            parent = pv.get("parent")
            if parent is None:
                # The loader defaults to the previous entry; weigh the same default.
                parent = ids[i - 1] if i > 0 else None
            if parent is None or parent not in byid or byid[parent] > i:
                # Including a parent defined LATER in the file: the loader refuses that
                # order (shipdef.cpp:1032-1042), so there is nothing to weigh.
                skipped += 1
                continue
            ppv = entries[byid[parent]]
            pdef = defs.get(ppv.get("part"))
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
            # "point" or "normal" alone is the explicit branch for the loader too -- it
            # parses both and throws on the missing one -- so treat it as explicit here
            # rather than falling back to the shorthand and reporting a plausible lie.
            explicit = "point" in pv or "normal" in pv
            if explicit:
                pt, nm = as_vec3(pv.get("point")), as_vec3(pv.get("normal"))
                if pt is None or nm is None:
                    refused.append('%s:%s->%s: "point"/"normal" is not two 3-number arrays'
                                   % (slug, pid, parent))
                    continue
                px, py, z = pt
                r = math.hypot(px, py)
                if r <= 1e-12:
                    # On the axis: no azimuth to weigh against. The loader would still
                    # throw later ("contact normal must be non-zero" or a degenerate
                    # pose), and this gate has nothing to say.
                    skipped += 1
                    continue
                ca, sa = px / r, py / r
                kind = "explicit"
            else:
                z = as_num(pv.get("z", 0.0))
                ang = as_num(pv.get("angle", 0.0))
                if z is None or ang is None:
                    refused.append('%s:%s->%s: "z"/"angle" is not a number'
                                   % (slug, pid, parent))
                    continue
                a = math.radians(ang)
                ca, sa = math.cos(a), math.sin(a)
                r = ellipse_t(hx, hy, ca, sa)
                kind = "angle"
                # The loader computes the contact and then refuses the row if the authored
                # height runs past the end of the parent (shipdef.cpp:1150). That check
                # lives in the shorthand branch ONLY: an authored "point" carries its own
                # height and may sit past a part on purpose (skiff's rudder is 0.2 m below
                # the end of its jet cone). Mirror it where the loader has it, or the gate
                # invents refusals.
                if abs(z) > hz + 1e-9:
                    refused.append("%s:%s->%s: z %.2f is off the end of %s (spans +-%.2f m)"
                                   % (slug, pid, parent, z, ppv["part"], hz))
                    continue
            lv = ray_levels(vs, ca, sa)
            skin = interp_levels(lv, z)
            rows.append({"ship": slug, "part": pv["part"], "id": pid,
                         "parent": parent, "parent_part": ppv["part"], "kind": kind,
                         "z": z, "angle": as_num(pv.get("angle", 0.0), 0.0),
                         "offset": as_num(pv.get("offset", 0.0), 0.0),
                         "contact": r, "skin": skin, "gap": r - skin, "hz": hz})
    return rows, skipped, refused


def weigh_nodes(defs, mesh_dir):
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
        if not srf and nodes:
            # synthesizeNodes returns early when nodes were authored, and
            # findSurfaceNode() then yields null, so this part can never be a surface
            # child (shipdef.cpp:1170-1176 refuses the edge). Nothing to weigh.
            continue
        if not srf:
            # synthesizeNodes: side node at -radius, at z = 0. The catalog's own default
            # for radius is 1.0 (shipdef.cpp:398), not 0.
            radius = as_num(d.get("radius", 1.0), 1.0)
            srf = [{"pos": [-radius, 0.0, 0.0]}]
        pos = srf[0].get("pos")
        pv3 = as_vec3(pos)
        if pv3 is None:
            continue
        lv = ray_levels(vs, -1.0, 0.0)
        skin = interp_levels(lv, pv3[2])
        off = abs(pv3[0])
        rows.append({"part": name, "node_z": pv3[2], "node_off": off,
                     "skin": skin, "gap": off - skin})
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
    "level_jet:rudder_1->jet_1": 0.2389,
    "level_jet:rudder_2->jet_1": 0.2389,
    "level_jet:elevator_1->jet_1": 0.2389,
    "level_jet:elevator_2->jet_1": 0.2389,
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
    "skiff:rudder->engine": 0.1626,
    "skiff_gear:rudder->engine": 0.1626,
    "skiff_trim:rudder->engine": 0.1626,
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


def gate(label, rows, waivers, key_of, tol, verbose, check_stale=True):
    """Rows outside tolerance must appear in `waivers` at the magnitude they were measured
    when the waiver was written.

    The comparison is on MAGNITUDE, not on the signed value: a row waived at +0.312 m of
    float that later measures -0.900 m has gone from floating to buried inside the parent,
    which is a different defect of the same size and must not go green because its sign
    flipped. A waiver whose row no longer offends -- or no longer exists at all -- is
    reported too, so the list cannot fossilise. `check_stale` is off for an exploratory
    --tol other than the default, where every waiver in the file would otherwise read
    stale."""
    bad = []
    stale = []
    weighed = set()
    for row in rows:
        k = key_of(row)
        gap = row["gap"]
        weighed.add(k)
        if abs(gap) <= tol:
            if k in waivers and check_stale:
                stale.append((k, waivers[k], gap))
            continue
        w = waivers.get(k)
        if w is None:
            bad.append((row, "not waived"))
        elif abs(gap) > abs(w) + 1e-4:
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
            where = "%s on %s at z=%.2f (%s" % (row["ship"], row["part"], row["z"],
                                                row["kind"])
            if row.get("offset", 0.0):
                where += ", offset %.2f m stands the child off on purpose" % row["offset"]
            where += ")"
        else:
            where = "part %s surface node at z=%.2f" % (row["part"], row["node_z"])
        print("  OFFENDER %-40s gap %+.3f m (%s)  [%s: contact %.3f vs skin %.3f]"
              % (key_of(row), row["gap"], why, where,
                 row.get("contact", row.get("node_off", 0.0)), row["skin"]))
    for k, w, gap in stale:
        print("  STALE WAIVER %-37s written against %+.4f m, now %s -- tighten the list"
              % (k, w, "no such row" if gap is None else "%+.4f m" % gap))
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
         "angle": 0.0, "z": 0.0},
        {"part": "cone", "id": "upkid", "attach": "surface", "parent": "c1",
         "angle": 0.0, "z": 0.5}]}}
    rows, skipped, refused = weigh_ships(ships, defs, tmpdir)
    expect(2, len(rows), "two surface edges weighed")
    expect(0, len(refused), "and nothing refused")
    byid = {r["id"]: r for r in rows}
    if rows:
        # Box half-span 1.0 at every z; the skin at z=0 is 0.5 on this cone.
        expect(True, abs(byid["kid"]["contact"] - 1.0) < 1e-9, "contact rides the box radius")
        expect(True, abs(byid["kid"]["skin"] - 0.5) < 1e-6, "skin is the cone at that height")
        expect(True, abs(byid["kid"]["gap"] - 0.5) < 1e-6, "a floating row reads +0.5 m")
    # Height matters, so it is pinned: the same shorthand 0.5 m up the cone floats 0.75,
    # not 0.5. Every fixture at z=0 would leave the whole z dimension unexercised.
    expect(True, abs(byid["upkid"]["skin"] - 0.25) < 1e-6, "the skin tracks the authored z")
    expect(True, abs(byid["upkid"]["gap"] - 0.75) < 1e-6, "and so does the gap")

    # Waivers are value-bound: same row, worse than waived, is an offender.
    key = lambda r: "%s:%s->%s" % (r["ship"], r["id"], r["parent"])
    expect(1, quiet_gate("cone", rows, {"conic:kid->c1": 0.4, "conic:upkid->c1": 0.75},
                         key, TOL_M, False),
           "a gap worse than its waiver fails")
    expect(0, quiet_gate("cone", rows, {"conic:kid->c1": 0.5, "conic:upkid->c1": 0.75},
                         key, TOL_M, False),
           "a gap at its waived value passes")
    expect(1, quiet_gate("cone", [], {"conic:kid->c1": 0.5}, key, TOL_M, False),
           "a waiver that no longer offends is reported")
    # The row still there but now ON the skin is the other half of that rule, and it needs
    # its own pin: the loop above only sees rows that vanished outright.
    tight = [{"ship": "conic", "part": "cone", "id": "kid", "parent": "c1",
              "kind": "angle", "z": 0.0, "angle": 0.0, "offset": 0.0,
              "contact": 0.5, "skin": 0.5, "gap": 0.0, "hz": 1.0}]
    expect(1, quiet_gate("tight", tight, {"conic:kid->c1": 0.5}, key, TOL_M, False),
           "a waiver whose row came back on the skin is reported")
    expect(0, quiet_gate("tight", tight, {}, key, TOL_M, False),
           "an unwaived row on the skin is not a problem")
    # --tol other than the default turns that check off, or every waiver in the file
    # would read stale at once.
    expect(0, quiet_gate("tight", tight, {"conic:kid->c1": 0.5}, key, 0.9, False,
                         check_stale=False),
           "an exploratory --tol does not call every waiver stale")
    # Magnitude, not sign: a waived +0.5 row that has gone to -0.9 is buried inside the
    # parent, which is the same size of wrong and must not read green.
    deep = [{"ship": "conic", "part": "cone", "id": "kid", "parent": "c1",
             "kind": "angle", "z": 0.0, "angle": 0.0, "offset": 0.0,
             "contact": 0.4, "skin": 1.3, "gap": -0.9, "hz": 1.0}]
    expect(1, quiet_gate("deep", deep, {"conic:kid->c1": 0.5}, key, TOL_M, False),
           "a waived row that flipped to buried fails")

    # An explicit point/normal pair is weighed too -- at the POINT's height, because the
    # loader never reads "z" on that branch -- and it can be buried.
    bships = {"buried": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "deep", "attach": "surface", "parent": "c1",
         "point": [0.1, 0.0, 0.0], "normal": [1.0, 0.0, 0.0]},
        {"part": "cone", "id": "low", "attach": "surface", "parent": "c1",
         "point": [1.0, 0.0, -0.7], "normal": [1.0, 0.0, 0.0], "z": 0.0}]}}
    brows, bskip, brefused = weigh_ships(bships, defs, tmpdir)
    bidx = {r["id"]: r for r in brows}
    expect(2, len(brows), "two explicit edges weighed")
    expect(True, "deep" in bidx and bidx["deep"]["gap"] < 0.0, "a buried contact reads negative")
    # z comes from point[2] = -0.7, where the cone's skin is 0.85, and the "z": 0.0 that
    # the loader ignores must NOT be what gets weighed (that would read gap 0.5).
    expect(True, "low" in bidx and abs(bidx["low"]["z"] + 0.7) < 1e-9,
           "the explicit branch weighs the POINT's height")
    expect(True, "low" in bidx and abs(bidx["low"]["gap"] - 0.15) < 1e-6,
           "so a contact 1.0 out at z=-0.7 floats by 0.15, not 0.5")

    # Data the loader itself refuses is reported, not reinterpreted as shorthand.
    halfauth = {"halfauth": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "pt", "attach": "surface", "parent": "c1",
         "point": [0.9, 0.0, -0.4]}]}}
    hrows, hskip, hrefused = weigh_ships(halfauth, defs, tmpdir)
    expect(0, len(hrows), 'a "point" with no "normal" is not weighed as a shorthand')
    expect(1, len(hrefused), 'and it is reported as a load error')
    overz = {"overz": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "far", "attach": "surface", "parent": "c1",
         "angle": 0.0, "z": 5.0}]}}
    orows, oskip, orefused = weigh_ships(overz, defs, tmpdir)
    expect(1, len(orefused), 'a "z" past the end of the parent is named')
    dup = {"dup": {"parts": [
        {"part": "cone", "id": "c1"},
        {"part": "cone", "id": "c1", "attach": "surface", "parent": "c1", "angle": 0.0}]}}
    _, _, drefused = weigh_ships(dup, defs, tmpdir)
    expect(1, len(drefused), "a duplicate part id is named, as the loader throws on it")

    # The auto-id counter the waiver keys are built from: two un-named entries of the same
    # part number _1, _2 exactly as shipdef.cpp:1005-1010 does.
    expect(["cone_1", "cone_2"],
           part_ids([{"part": "cone"}, {"part": "cone", "attach": "surface"}]),
           "auto part ids follow the loader's counter")

    # The two mesh helpers, pinned on their own. half_span is SYMMETRISED -- the loader's
    # meshHalfSpan pairs the larger of |mesh_min| and |mesh_max| -- so an off-axis mesh
    # reads as wide as its widest side rather than falsely floating. interp_levels CLAMPS
    # outside the vertex levels: a mount below the lowest ring sits on that ring's skin,
    # not on an extrapolation of the slope, which is exactly how skiff's rudder at z=-0.7
    # is weighed against a jet cone that ends at -0.5.
    expect((1.0, 0.0, 0.0), half_span([(-1.0, 0.0, 0.0), (0.2, 0.0, 0.0)]),
           "half_span takes the larger side, whichever way it points")
    lv2 = [(0.0, 1.0), (1.0, 0.5)]
    expect(1.0, interp_levels(lv2, -3.0), "below the lowest ring the skin clamps")
    expect(0.5, interp_levels(lv2, 2.0), "and above the highest")
    expect(True, abs(interp_levels(lv2, 0.5) - 0.75) < 1e-12, "between, it interpolates")

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
    prow, pskip, _ = weigh_ships(pships, pdefs, tmpdir)
    expect(0, len(prow), "a plate parent is not weighed")
    expect(2, pskip, "and it is counted as skipped")
    vs = read_mesh_vertices(plate)
    hpx, hpy, hpz = half_span(vs)
    k = math.sqrt(0.5)
    k30 = math.sqrt(0.75)
    expect(True, abs(ellipse_t(hpx, hpy, 0.0, 1.0) - 0.05) < 1e-12, "face ray = thickness")
    expect(True, abs(ellipse_t(hpx, hpy, 1.0, 0.0) - 2.0) < 1e-12, "tip ray = span")
    # A third azimuth, off both axes: the only way to pin the general form's hx/hy
    # placement, since at 45 deg swapping them is symmetric.
    expect(True, abs(ellipse_t(hpx, hpy, k30, 0.5) - 0.0999065) < 1e-6,
           "the 30 deg ray is hx-long, not hy-long (swapped it would read 0.05772)")
    skin_diag = interp_levels(ray_levels(vs, k, k), 0.0)
    expect(True, ellipse_t(hpx, hpy, k, k) < skin_diag - 1.3,
           "a diagonal plate mount sits inside the plate by design")
    # And on the child side the same scoping applies: a plate part's synthesized side node
    # is a placement on its own face plane, so weigh_nodes leaves it alone rather than
    # reporting the gap between that node and a radial skin it was never measured against.
    expect(0, len(weigh_nodes(pdefs, tmpdir)), "a plate part's own side node is not weighed")

    # A part with no mesh cannot be weighed, and is counted rather than sworn at.
    nomesh = {"nomesh": {"parts": [
        {"part": "ghost", "id": "g1"},
        {"part": "ghost", "id": "g2", "attach": "surface", "parent": "g1", "angle": 0.0}]}}
    _, nskip, _ = weigh_ships(nomesh, {"ghost": {"name": "ghost", "mesh": "nope.obj",
                                                "radius": 1.0, "height": 2.0}}, tmpdir)
    expect(1, nskip, "a meshless parent is skipped, not failed")

    # The node half: a tapered part's synthesized srf node stands off its own skin.
    nrows = weigh_nodes({"cone": {"name": "cone", "mesh": "cone.obj", "radius": 1.0,
                                  "height": 2.0}}, tmpdir)
    expect(1, len(nrows), "one node row")
    expect(True, nrows and abs(nrows[0]["gap"] - 0.5) < 1e-6,
           "the side node at -radius floats by radius - skin(0)")
    # Authored nodes with none flagged surface: synthesizeNodes never ran, and
    # findSurfaceNode() is null, so the part cannot be a surface child at all.
    expect(0, len(weigh_nodes({"cn": {"name": "cn", "mesh": "cone.obj", "radius": 1.0,
                                      "height": 2.0,
                                      "nodes": [{"id": "top", "pos": [0, 0, 1],
                                                 "dir": [0, 0, 1]}]}}, tmpdir)),
           "a part with authored nodes and no surface node is not weighed")

    return counts[0], fails


# ---------------------------------------------------------------------------

def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--parts", default="res/data/parts.json")
    ap.add_argument("--ships", nargs="+", default=["res/ships", "e2e/vehicles"],
                    help="directories of ship defs: the roster AND the e2e fixtures, "
                         "which go through the same loader by the same --startship flag")
    ap.add_argument("--mesh-dir", default="res", help="root that part 'mesh' paths resolve under")
    ap.add_argument("--tol", type=float, default=TOL_M,
                    help="tolerance in metres; an exploratory --tol also stops the "
                         "stale-waiver check, since every waiver would read stale at it")
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
        defs = {p["name"]: p for p in cat["parts"]}
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        print("check_ship_skin: cannot read %s: %s" % (args.parts, e))
        return 2

    ships = {}
    for d in args.ships:
        try:
            names = sorted(os.listdir(d))
        except OSError as e:
            print("check_ship_skin: cannot list %s: %s" % (d, e))
            return 2
        for nm in names:
            if not nm.endswith(".json"):
                continue
            try:
                doc = json.load(open(os.path.join(d, nm)))
            except (OSError, ValueError) as e:
                print("check_ship_skin: cannot read %s: %s" % (nm, e))
                return 2
            if nm[:-5] in ships:
                # Waiver keys are "<file slug>:...", so two files of the same name in two
                # scanned dirs would collide in them. Refuse rather than merge.
                print("check_ship_skin: %s found in two scanned dirs" % nm)
                return 2
            ships[nm[:-5]] = doc

    try:
        rows, skipped, refused = weigh_ships(ships, defs, args.mesh_dir)
        nrows = weigh_nodes(defs, args.mesh_dir)
    except (AttributeError, KeyError, TypeError, ValueError) as e:
        # A data shape neither this gate nor the loader was written for. Say which, and
        # leave 2: python's own traceback exits 1, which is the same code "offenders"
        # returns, and a gate that cannot tell "broken" from "guilty" gets ignored.
        print("check_ship_skin: unexpected data shape: %r" % (e,))
        return 2

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
          "(%d skipped: no mesh, no def, a plate parent, or a dangling parent)"
          % (len(ships), len(defs), len(rows), skipped))
    bad += gate("surface contacts", rows, CONTACT_WAIVERS,
                lambda r: "%s:%s->%s" % (r["ship"], r["id"], r["parent"]),
                args.tol, args.verbose, check_stale=(args.tol == TOL_M))
    bad += gate("surface nodes", nrows, NODE_WAIVERS, lambda r: r["part"],
                args.tol, args.verbose, check_stale=(args.tol == TOL_M))
    for line in refused:
        print("  REFUSED %s -- the loader would not load this" % line)
    bad += len(refused)
    if bad:
        print("check_ship_skin: %d problems -- see #153" % bad)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
