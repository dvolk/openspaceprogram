#!/usr/bin/env python3
"""Generate a rocket engine BODY as a .obj: a stack shell over a de Laval bell.

NOT WIRED INTO THE CATALOGUE (2026-11-25): the shipped engine meshes are the hand-authored
family, and this generator is not pinned -- see SHELVED in check_mesh_pins.py. It stays
because the hand family's nozzle is an hourglass (below) and because the attach law that shapes
engine art (#153) is worth seeing in numbers before anyone reshapes the art again. To use it:
run it, unwrap what it writes, and repaint engine.png for the new layout -- the six bodies that
share that texture need ONE layout between them, which is what the hand family has and what a
per-body unwrap destroys (measured: worst uv drift 1.036 of the canvas across the family,
against 0.000 for the hand-authored set).

Eight meshes -- engine, engine_r1.5h3, engine_r2.25h4.5, engine_r0.25h1, the three orbital
engines and the nuclear engine. They came out of one Blender file scaled per size class, which
is why every one of them has the same wrong nozzle: sampling radius against height on the
shipped engine.obj gives r=1.0 at the stack end, 0.761 at midbody, a pinch to r=0.127, then
0.254, 0.318, and r=0.35 at the exit plane. So the wall narrows going down and widens by 38%
at the very end -- exit radius 2.8x the throat, an hourglass rather than a bell. A real nozzle
converges to a throat and then opens out, and the whole point of the shape is the ratio between
those two radii, so this file exists to compute it rather than to nudge control points.

The geometry is one surface of revolution, so it goes through gen_lathe.py -- this file only
decides the rings. The nozzle occupies the bottom of the envelope, the chamber the top:

    +H/2   |          |   R          stack wall: where the part above bolts on
           |          |
           |          |              chamber (this band covers the part's midplane, see below)
           |   \\      |              shoulder: one band, R down to the throat
           |    \\    /
           |     \\__/               throat, the narrowest ring, r_t, below z = 0
           |      \\                 nozzle: r(z) = r_t + (r_e - r_t) * f(t), f saturating,
           |                       opening downward -- steepest just under the throat
    -H/2   |_______\\__|             exit ring r_e, then the exit disc

The shape is the ratio first and the curve second: eps = (r_e / r_t)^2, area expansion ratio. Sea-
level nozzles are short and stubby (eps ~4-6, a bell that would be overexpanded at the pad); vacuum
nozzles open wide (eps ~10-16). Same generator, different number in the pin.

The wall saturates rather than steepening: f(t) = (1 - exp(-k t)) / (1 - exp(-k)) for t = how far
down the bell, k = --bell-turn. A bell turns hardest just below the throat and then runs a shallow,
near-straight skirt, and that is what the curve gives -- chords of 51.7, 25.0, 9.7 deg from the
axis on engine (r1 h2) at 3 bands. The power law this replaces (t**2) left the throat near-axially
and opened out toward the exit instead: 11.9, 32.4, 46.6 deg, a wall that gets worse going down.
That reads as a funnel, not a nozzle, and it is what the owner caught. The curve costs one ring per
--bell-bands, so it is bought with triangles and not with texels: the pins ship 3 bands (162 faces
at 32 segments, against 624 for a fuselage).

Three laws the profile cannot break:

- The exit ring stays INSIDE the part's declared radius: r_e <= radius, so a nozzle never pokes
  into the part mounted beside it and a stack's silhouette from the side is unchanged. Nozzle
  depth comes out of the part's own height instead.
- The throat sits BELOW z = 0, so the part's midplane is on the chamber wall. synthesizeNodes
  (shipdef.h:328) puts a part's side-attach node at z = 0 and ellipseContact takes the attach
  radius from meshHalfSpan, the WIDEST span at any height (shipdef.cpp:1138) -- so when the
  pinch is at the midplane every side mount of that part hangs off nothing. With the throat at
  z = 0 exactly, endurance_racer's tanks went from 0.239 m to 0.633 m off the engine skin and
  engine_r2.25h4.5's own node row from 0.538 m to 1.423 m. With the throat at -0.35H they land
  on the skin: 0.000 m.
- Segment count matches the rest of the stack family, 32, and is a multiple of 4. A polygon
  with a vertex at angle 0 and n not a multiple of 4 does not reach both axes (a decagon:
  half-span 1.000 along X, 0.951 along Y), and utils/check_ship_skin.py weighs only round
  cross-sections, so a 10-segment engine simply stops being checked -- the measurement hole
  reads as an improvement. 32 is also what every other stack part uses (tank_r1h3, capsule,
  the adapters, the shrouds), so the silhouette where an engine meets a tank has no facet seam.

The UVs come from Blender's Smart UV Project (uv_smart_blender.py), not from gen_lathe.py's
shelf packer. A lathe's one-island-per-face is exact and needs no Blender, but at 32 segments a
band's 32 quads land as 32 scattered patches, so nobody can draw a ring around a nozzle or a
stripe down a chamber in an image editor -- and the hand-authored parts are not laid out that
way either (capsule.obj is 3 islands: the wall as one strip, plus two caps). Smart UV joins
faces that project the same way. Measured on this profile at 32 segments, 3 bell bands:

    islands      162 -> 12
    canvas fill  0.575 -> 0.785
    px/m         88..88 (spread 1.00) -> 76..102 (spread 1.29)

The density wander is the cost of the join, and it is the same order as the art that already
ships: tank_r1h3.obj measures 67..82 px/m, spread 1.22, against the gate's 2.0 ceiling
(check_mesh_pins.py DENSITY_MAX_SPREAD). Spread is the gate's measure -- worst face over the
part median, both ways -- and 1.32 is the worst of the eight bodies (orbital_engine); the others
land 1.13..1.29. Geometry does not move -- same 192 verts, 162 faces, 160 wall quads and 2 ngon
discs, one normal per face, same volume to the printed precision -- and 4.5.9's unwrap is
deterministic, so the byte-exact pin still works: two runs of the same argv produced identical
files. The angle limit and the gutter are pinned as argv, which makes changing either a diff
rather than a silent re-pack.

The axis pair on the way out of Blender is obj_import's own, restated rather than left to
defaults (uv_smart_blender.py). `up_axis="Z"` -- the intuitive choice for a world-Z-up file --
turns the part 90 deg, and nothing downstream notices except an engine lying on its side. So the
generator checks that the unwrap only moved texture coordinates: `v`/`f` counts and each face's
corner cycle (the exporter pools 576 `vt` lines into 317, so face identity is the only honest
compare), and that the stack wall and the exit ring still sit at z = +/-H/2 at radii R and r_e.
A bounding box cannot prove a part stands up: engine, engine_r1.5h3 and engine_r2.25h4.5 all box
out as cubes, and a turned cube boxes out the same size.

Blender is a dependency of this generator -- and since the engine bodies went out of the pinned
set, of nothing that `make test` runs. It is not a build or CI dependency: lint.yml runs
check_mesh_pins.py --committed, which regenerates nothing. It is found at
~/opt/blender-4.5.9-linux-x64/blender or at $BLENDER.

Not to be confused with the part's `shroud`, which gen_lathe.py still makes: that is a straight
tube at 98% of body radius, drawn only when a part hangs off the exhaust face
(src/render.cpp:1724, src/vehicle.cpp:7102) to hide the engine so a stack reads as one
cylinder. The nozzle lives in the body.

    gen_engine.py DST --radius 1 --height 2 --eps 6 --exit-frac 0.75 --bell-frac 0.35
    gen_engine.py DST --radius 1 --height 1 --eps 10 --exit-frac 0.75 --name OrbitalEngine

#1190: `jet` and `jet_r0.25h1` borrow meshes/engine.obj and meshes/engine_r0.25h1.obj, so the
air-breathers wear these nozzles too until they get their own duct profile.
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile

import gen_lathe

# How the bell's wall opens between throat and exit. r = r_t + (r_e - r_t) * f(t), t = how far
# down the bell. f must SATURATE: a real de Laval bell does its turning just below the throat and
# then runs a shallow near-straight skirt. A power law (t**p) does the opposite -- it leaves the
# throat axially and steepens into the exit ring -- which is what made the shipped bell splay open
# at the lip. Chords from the axis on engine (r1 h2, 3 bands, mean 32.4 deg):
#
#     t**2 (what shipped)      11.9 -> 32.4 -> 46.6     flare gets worse going down
#     this, k = 3              51.7 -> 25.0 ->  9.7     shoulder under the throat, then a skirt
#     this, k = 6              58.8 -> 12.6 ->  1.7     shoulder, then nearly a straight tube
#
# k is pinned as argv (--bell-turn) rather than fixed, so tightening the shoulder is a diff.
BELL_TURN = 3.0

BLENDER = os.environ.get("BLENDER", os.path.expanduser("~/opt/blender-4.5.9-linux-x64/blender"))
UNWRAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uv_smart_blender.py")
# A background Blender that hangs would otherwise hang `make test` with it. 120 s is about 60x
# what this takes (2 s for 130 faces), so it only ever fires on a real wedge.
UNWRAP_TIMEOUT_S = 120


def bell_shape(t, turn):
    """How far the wall has opened at fraction t of the way down the bell: it turns hardest at
    the throat and saturates toward the exit ring. See BELL_TURN for the measured chords."""
    return (1.0 - math.exp(-turn * t)) / (1.0 - math.exp(-turn))


def bell_rings(radius, height, exit_frac, eps, bell_frac, stack_frac, bell_bands, turn):
    """The profile as [(radius, z), ...] low z first: exit ring, bell interior, throat,
    one converging band up to the stack wall, stack wall."""
    r_e = exit_radius(radius, exit_frac)
    r_t = r_e / (eps ** 0.5)
    z_throat = -height / 2.0 + bell_frac * height
    bell_len = bell_frac * height
    # Walk down the bell from the throat. The first chord below the throat is the steepest one
    # and each next one lies flatter, which is what a bell skirt looks like.
    up = []
    for i in range(bell_bands + 1):
        t = i / float(bell_bands)
        up.append((r_t + (r_e - r_t) * bell_shape(t, turn), z_throat - bell_len * t))
    up.append((radius, z_throat + stack_frac * height))
    up.append((radius, height / 2.0))
    up.sort(key=lambda t: t[1])
    # A ring that lands on the midplane should read as 0 in the header rather than as
    # -5.55112e-17: these are exact fractions of the height, and the noise is the sum.
    return [(0.0 if abs(r) < 1e-12 else r, 0.0 if abs(z) < 1e-12 else z) for r, z in up]


def profile_arg(rings):
    return ",".join("%.6f@%.6f" % (r, z) for r, z in rings)


def smart_unwrap(src, dst, angle_deg, margin):
    """Blender's Smart UV Project over a lathe's geometry, in a headless run."""
    if not os.path.exists(BLENDER):
        sys.exit("%s: no Blender. This generator hands its geometry to Blender for the UV pass "
                 "(uv_smart_blender.py); set BLENDER=/path/to/blender. CI is unaffected -- it "
                 "runs check_mesh_pins.py --committed, which regenerates nothing." % BLENDER)
    try:
        r = subprocess.run([BLENDER, "--background", "--factory-startup", "--python", UNWRAP,
                            "--", src, dst, repr(angle_deg), repr(margin)],
                           capture_output=True, text=True, timeout=UNWRAP_TIMEOUT_S)
    except subprocess.TimeoutExpired as exc:
        sys.exit("Smart UV Project on %s was still running after %d s, so %s was not written: "
                 "%s" % (src, UNWRAP_TIMEOUT_S, dst, exc))
    if r.returncode or not os.path.exists(dst):
        sys.exit("Smart UV Project failed on %s (exit %d):\n%s\n%s"
                 % (src, r.returncode, r.stdout, r.stderr))
    return read_body(dst)


def read_body(path):
    """The .obj text with the exporter's own comment lines dropped, so the caller can put its
    provenance header where the coverage scan expects it."""
    with open(path, encoding="utf-8") as fh:
        return "".join(ln for ln in fh if not ln.startswith("#"))


# These three read the OBJ text directly rather than going through check_mesh_pins.read_faces:
# the gate inspects one committed file and returns index tables, while this has to compare two
# texts of the same surface side by side, which needs coordinates and corner cycles.
def count(lines, pfx):
    return sum(1 for ln in lines if ln.startswith(pfx))


def face_cycles(lines):
    """Each face's vertex cycle, rotated so its lowest index leads. The exporter renumbers
    and pools v/vt/vn (576 vt lines become 317), so the only way to compare the two meshes as
    surfaces is by which corners each face names."""
    out = []
    for ln in lines:
        if not ln.startswith("f "):
            continue
        vi = [int(t.split("/")[0]) for t in ln.split()[1:]]
        k = vi.index(min(vi))
        out.append(tuple(vi[k:] + vi[:k]))
    return set(out)


def bbox(lines):
    lo, hi = [None] * 3, [None] * 3
    for ln in lines:
        if not ln.startswith("v "):
            continue
        for k in range(3):
            x = float(ln.split()[1 + k])
            if lo[k] is None or x < lo[k]:
                lo[k] = x
            if hi[k] is None or x > hi[k]:
                hi[k] = x
    return lo, hi


def verts(lines):
    return [tuple(float(x) for x in ln.split()[1:4]) for ln in lines if ln.startswith("v ")]


def main(argv=None):
    ap = argparse.ArgumentParser(description="lathe a rocket engine body: a stack shell "
                                             "over an expansion-ratio bell, unwrapped for "
                                             "painting")
    ap.add_argument("dst", help="destination .obj")
    ap.add_argument("--radius", type=float, required=True, help="the part's radius (the "
                    "stack wall, and the widest the bell may get)")
    ap.add_argument("--height", type=float, required=True, help="the part's height")
    ap.add_argument("--eps", type=float, default=6.0,
                    help="area expansion ratio (r_exit/r_throat)^2; ~4-6 sea level, ~10-16 vacuum")
    ap.add_argument("--exit-frac", type=float, default=0.75,
                    help="exit radius as a fraction of the part radius; must be <= 1 so the "
                         "bell stays inside the envelope")
    ap.add_argument("--bell-frac", type=float, default=0.35,
                    help="how much of the height the nozzle takes, measured up from the exit "
                         "plane: the throat lands at -height/2 + bell_frac*height, so < 0.5 "
                         "keeps the pinch below the part's midplane (see the header)")
    ap.add_argument("--stack-frac", type=float, default=0.15,
                    help="how much of the height the shoulder takes -- the band from the "
                         "throat out to the stack wall; the straight chamber gets the rest")
    ap.add_argument("--bell-bands", type=int, default=3,
                    help="rings across the nozzle (2 = exit and throat, a straight wall; 3 buys "
                         "the curve, and costs triangles rather than texels now that the canvas "
                         "follows the part's size)")
    ap.add_argument("--bell-turn", type=float, default=BELL_TURN,
                    help="how fast the wall turns at the throat: the chord angles saturate down "
                         "the skirt, so a bigger number is a tighter shoulder and a straighter "
                         "skirt (see BELL_TURN)")
    ap.add_argument("--segments", type=int, default=32,
                    help="rings about the axis; 32 is what every other stack part uses")
    ap.add_argument("--uv-angle", type=float, default=66.0,
                    help="Smart UV Project's angle limit in degrees: how far apart two face "
                         "normals may be and still share a patch")
    ap.add_argument("--uv-margin", type=float, default=0.008,
                    help="gutter between patches, as a fraction of the canvas")
    ap.add_argument("--caps", choices=["both", "base", "top", "none"], default="both",
                    help="which ends are closed; an open exit reads as a hole, so both")
    ap.add_argument("--name", default="Engine", help="object name written into the file")
    a = ap.parse_args(argv)
    if a.radius <= 0.0 or a.height <= 0.0:
        ap.error("--radius and --height are sizes, not shapes: both > 0")
    if a.eps <= 1.0:
        ap.error("--eps is an expansion ratio: > 1 (1 is a straight tube, which is a shroud)")
    if not 0.0 < a.exit_frac <= 1.0:
        ap.error("--exit-frac is a fraction of the part radius: 0 < exit-frac <= 1, or the "
                 "bell leaves the envelope and collides with the part mounted beside it")
    if not 0.0 < a.bell_frac < 1.0 or not 0.0 <= a.stack_frac < 1.0:
        ap.error("--bell-frac is 0..1 of the height and --stack-frac 0..1 of it")
    if a.bell_frac + a.stack_frac >= 1.0:
        ap.error("bell-frac (%g) + stack-frac (%g) leaves no height for the converging band"
                 % (a.bell_frac, a.stack_frac))
    if a.bell_bands < 1:
        ap.error("--bell-bands >= 1 (exit ring, that many steps up, throat ring)")
    if a.bell_turn < 0.1:
        ap.error("--bell-turn >= 0.1: the shape is (1 - exp(-turn*t)) / (1 - exp(-turn)), so a "
                 "turn of 0 has no room to saturate")
    if a.segments % 4:
        ap.error("--segments must be a multiple of 4: a polygon with a vertex at angle 0 and "
                 "any other count misses one axis (a decagon spans 1.000 in X, 0.951 in Y), "
                 "and utils/check_ship_skin.py only weighs round cross-sections, so the part "
                 "silently stops being checked for floating side mounts")
    if not 1.0 <= a.uv_angle <= 179.0:
        ap.error("--uv-angle is an angle in degrees, 1..179")
    if not 0.001 <= a.uv_margin <= 0.05:
        ap.error("--uv-margin is a fraction of the canvas: 0.001 <= margin <= 0.05 "
                 "(0.001 is one texel at 512^2)")

    rings = bell_rings(a.radius, a.height, a.exit_frac, a.eps, a.bell_frac,
                       a.stack_frac, a.bell_bands, a.bell_turn)
    # The lathe sorts and validates the profile, so check here only what a lathe cannot see:
    # that the ring list describes a bell (widest at the exit plane, narrowest at a throat in
    # the middle) inside the size the caller asked for.
    r_e = exit_radius(a.radius, a.exit_frac)
    assert abs(rings[0][0] - r_e) <= 1e-9 and abs(rings[0][1] + a.height / 2.0) <= 1e-9, \
           "the exit ring is not the widest ring at the bottom of the envelope"
    assert abs(rings[-1][1] - a.height / 2.0) <= 1e-9 and abs(rings[-1][0] - a.radius) <= 1e-9
    assert max(r for r, _z in rings) <= a.radius + 1e-9, "the bell left the envelope"
    throat = min(rings, key=lambda t: t[0])
    assert throat is not rings[0] and throat is not rings[-1], \
           "no throat: the profile is a cone or a tube, not a bell"
    assert throat[1] < 0.0, \
           "the throat is at the part's midplane, where synthesizeNodes puts its side node: " \
           "every side mount of this part would hang off the pinch instead of the wall (#153)"

    with tempfile.TemporaryDirectory(prefix="gen_engine-") as td:
        lathe = os.path.join(td, "lathe.obj")
        # gen_lathe names the header; Blender drops that file's comments, so the header that
        # reaches res/ is the one written below. The name here only helps when reading a
        # leftover temp file.
        gen_lathe.main([lathe, "--profile", profile_arg(rings), "--segments", str(a.segments),
                        "--caps", a.caps, "--name", a.name], generator="gen_engine.py")
        with open(lathe, encoding="utf-8") as fh:
            made = fh.read().splitlines(True)
        body = smart_unwrap(lathe, os.path.join(td, "uv.obj"), a.uv_angle, a.uv_margin)
        unwrapped = body.splitlines(True)
        # The unwrap should only move texture coordinates. Welding, triangulating or dropping a
        # face here would silently change the part, and the byte pin cannot tell a re-mesh from
        # a re-UV since both rewrite the file.
        for pfx in ("v ", "f "):
            assert count(made, pfx) == count(unwrapped, pfx), \
                   "Smart UV Project changed the %s count: %d -> %d" \
                   % (pfx.strip(), count(made, pfx), count(unwrapped, pfx))
        assert face_cycles(made) == face_cycles(unwrapped), \
               "Smart UV Project changed the surface: faces welded, split or re-wound"
        # ...and that the part is still standing up. obj_import rotates a world-Z-up file into
        # Blender's Y-up world; only the matching axis pair on the way out puts it back.
        src_lo, src_hi = bbox(made)
        out_lo, out_hi = bbox(unwrapped)
        for k in range(3):
            assert abs(src_lo[k] - out_lo[k]) < 1e-6 and abs(src_hi[k] - out_hi[k]) < 1e-6, \
                   "the unwrap moved the part's extent along %s: %.4f..%.4f -> %.4f..%.4f" \
                   % ("xyz"[k], src_lo[k], src_hi[k], out_lo[k], out_hi[k])
        # That one only compares the file with itself, so it cannot see a part that turned
        # quarter-way: engine (r1 h2), engine_r1.5h3 and engine_r2.25h4.5 box out as cubes, and
        # a rotated cube boxes out exactly as wide and as tall. Say the law from the profile
        # instead. The stack wall is the top ring (R at z = +H/2) and the nozzle exit is the
        # bottom ring (r_e at z = -H/2); a turned part still puts vertices in those planes, but
        # at the wrong distance from the axis -- quarter-turning (1,1,0) about X lands it at
        # (1,-1,1), a radius of 1.414 where the profile says 1.
        out_v = verts(unwrapped)
        for z_want, r_want, what in ((a.height / 2.0, a.radius, "stack wall"),
                                     (-a.height / 2.0, exit_radius(a.radius, a.exit_frac),
                                      "nozzle exit")):
            ring = [p for p in out_v if abs(p[2] - z_want) < 1e-6]
            assert len(ring) == a.segments, \
                   "only %d vertices at z = %+.4f, expected the %d of the %s ring: the axis " \
                   "round-trip laid the part on its side (uv_smart_blender.py must restate " \
                   "obj_import's own axis pair)" % (len(ring), z_want, a.segments, what)
            rad = [math.hypot(p[0], p[1]) for p in ring]
            assert min(rad) > r_want - 1e-6 and max(rad) < r_want + 1e-6, \
                   "the %s ring at z = %+.4f spans radius %.4f..%.4f, expected %.4f: the " \
                   "profile moved or the envelope left the part" \
                   % (what, z_want, min(rad), max(rad), r_want)

        header = ("# generated by gen_engine.py (profile %s segs=%d caps=%s "
                  "smart_uv=%gdeg/%g)\n"
                  % (",".join("%g@%g" % t for t in rings), a.segments, a.caps,
                     a.uv_angle, a.uv_margin))
        with open(a.dst, "w", encoding="utf-8") as fh:
            fh.write(header + body)


def exit_radius(radius, exit_frac):
    return exit_frac * radius


if __name__ == "__main__":
    main()
