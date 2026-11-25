#!/usr/bin/env python3
"""Generate a rocket engine BODY as a .obj: a stack shell over a de Laval bell.

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
           |      \\                 nozzle: r(z) = r_t + (r_e - r_t) * t**p, opening downward
    -H/2   |_______\\__|             exit ring r_e, then the exit disc

The shape is the ratio, not the curve: eps = (r_e / r_t)^2, area expansion ratio. Sea-level
nozzles are short and stubby (eps ~4-6, a bell that would be overexpanded at the pad); vacuum
nozzles open wide (eps ~10-16). Same generator, different number in the pin. p=2 (BELL_POWER)
holds the wall near-axial where it leaves the throat and opens it out toward the exit, which is
what makes a bell read as a bell rather than a cone. The curve costs one ring per --bell-bands,
so it is bought with triangles, not with texels: the pins ship 2 bands (130 faces at 32
segments, against 624 for a fuselage).

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
faces that project the same way. Measured on this profile at 32 segments:

    islands      130 -> 13
    canvas fill  0.606 -> 0.779
    px/m         88..88 (spread 1.00) -> 77..103 (spread 1.35)

The density wander is the cost of the join, and it is the same order as the art that already
ships: tank_r1h3.obj measures 67..82 px/m, spread 1.22, against the gate's 2.0 ceiling
(check_mesh_pins.py DENSITY_MAX_SPREAD). Geometry does not move -- same 160 verts, 130 faces,
128 wall quads and 2 ngon discs, one normal per face, same volume to the printed precision --
and 4.5.9's unwrap is deterministic, so the byte-exact pin still works: two runs of the same
argv produced identical files. The angle limit and the gutter are pinned as argv, which makes
changing either a diff rather than a silent re-pack.

Blender is a dependency of this generator, and so of `make test` (test-art) and of any art run;
it is NOT a dependency of lint.yml, which runs check_mesh_pins.py --committed and regenerates
nothing. It is found at ~/opt/blender-4.5.9-linux-x64/blender or at $BLENDER.

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
import os
import subprocess
import sys
import tempfile

import gen_lathe

# What the bell's wall looks like between throat and exit: r = r_t + (r_e - r_t) * t**p.
# p=1 is a cone; p>1 holds the wall near-axial at the throat and opens it out toward the exit,
# which is the shape a bell has. Fixed here rather than exposed -- at this segment count the
# difference between p=1.8 and p=2.4 is under a millimetre of silhouette.
BELL_POWER = 2.0

BLENDER = os.environ.get("BLENDER", os.path.expanduser("~/opt/blender-4.5.9-linux-x64/blender"))
UNWRAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uv_smart_blender.py")


def bell_rings(radius, height, exit_frac, eps, bell_frac, stack_frac, bell_bands):
    """The profile as [(radius, z), ...] low z first: exit ring, bell interior, throat,
    one converging band up to the stack wall, stack wall."""
    r_e = exit_radius(radius, exit_frac)
    r_t = r_e / (eps ** 0.5)
    z_throat = -height / 2.0 + bell_frac * height
    bell_len = bell_frac * height
    # Walk down the bell from the throat: t is how far down, so the radius grows as t**p --
    # slow at first (the wall leaves the throat near-axially) and fast near the exit ring.
    up = []
    for i in range(bell_bands + 1):
        t = i / float(bell_bands)
        up.append((r_t + (r_e - r_t) * t ** BELL_POWER, z_throat - bell_len * t))
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
    r = subprocess.run([BLENDER, "--background", "--factory-startup", "--python", UNWRAP,
                        "--", src, dst, repr(angle_deg), repr(margin)],
                       capture_output=True, text=True)
    if r.returncode or not os.path.exists(dst):
        sys.exit("Smart UV Project failed on %s (exit %d):\n%s\n%s"
                 % (src, r.returncode, r.stdout, r.stderr))
    return read_body(dst)


def read_body(path):
    """The .obj text with the exporter's own comment lines dropped, so the caller can put its
    provenance header where the coverage scan expects it."""
    with open(path, encoding="utf-8") as fh:
        return "".join(ln for ln in fh if not ln.startswith("#"))


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


def main():
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
    ap.add_argument("--bell-bands", type=int, default=2,
                    help="rings across the nozzle (1 = exit and throat, a straight wall; 2 "
                         "buys the curve, and costs triangles rather than texels now that the "
                         "canvas follows the part's size)")
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
    a = ap.parse_args()
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
                       a.stack_frac, a.bell_bands)
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
