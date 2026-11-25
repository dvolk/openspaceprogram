#!/usr/bin/env python3
"""Generate a tank-adapter .obj: frustum joining two stack radii, axis +Z, CCW winding.

The topology follows the art it sits beside (res/meshes/tank_r1h3.obj, capsule.obj -- the
Blender-authored family): the wall is one QUAD per segment, each end is ONE NGON, every
face carries its own normal, and every face gets its own UV island.

That is not decoration. The engine never notices -- Mesh::AssImpFromFile runs
aiProcess_Triangulate, so quads and ngons collapse to triangles on import, and a mesh
whose UVs are all one point still renders fine when its texture is a flat fill. Two
people notice, and so does the light:

  * Whoever paints the part. A per-face island means each face owns a patch of the PNG
    and starts as a solid colour to draw panel lines into. The old output wrapped u once
    around the whole canvas and parked both cap centres at (0.5, 0.5): the islands
    overlapped to a summed area of 2.9 (paint one panel, several change) and the ends had
    no canvas at all.
  * The shader. Sharing one normal between a wall corner and the cap corner that meets it
    at a right angle blends a flat end into a shallow dome -- measured 90 deg off the
    cap's own face normal, up to 138 deg on the steep adapters.

Islands sit on a margin'd grid because the min filter is GL_LINEAR_MIPMAP_LINEAR
(src/texture.cpp): without the inset, mip levels bleed a panel into its neighbour.
A cylinder is a frustum whose top radius equals its base, so the straight-walled parts
(fuselages of revolution, the flat-nosed capsule) come from here too.

Usage: gen_adapter.py DST --r1 1.5 --r2 1 --height 0.75 [--segments 32] [--name Adapter]
"""

import argparse
import math


def unit(v):
    L = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / L, v[1] / L, v[2] / L) if L > 0 else (0.0, 0.0, 1.0)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dst", help="destination .obj")
    ap.add_argument("--r1", type=float, default=1.0,
                    help="radius at the -Z end")
    ap.add_argument("--r2", type=float, default=3.0,
                    help="radius at the +Z end")
    ap.add_argument("--height", type=float, default=1.5,
                    help="frustum height (default: max(r1, r2) / 2)")
    ap.add_argument("--segments", type=int, default=32)
    ap.add_argument("--name", default="Adapter", help="object name written into the file")
    a = ap.parse_args()
    if a.r1 <= 0 or a.r2 <= 0 or a.height <= 0 or a.segments < 3:
        ap.error("r1/r2/height must be > 0, segments >= 3")

    r1, r2, h, n = a.r1, a.r2, a.height, a.segments
    zbase, ztop = -h / 2.0, h / 2.0

    # Two rings. The wall quads and both cap ngons share them, so the position table is
    # 2n long however many segments there are -- an end's centre vertex is not needed
    # once the end is one face.
    verts, ang = [], []
    for r, z in ((r1, zbase), (r2, ztop)):
        for i in range(n):
            th = 2.0 * math.pi * i / n
            ang.append(th)
            verts.append((r * math.cos(th), r * math.sin(th), z))

    # Wall quads run bottom, bottom+1, top+1, top. The bottom ngon walks the base ring
    # backwards so that every edge the end shares with the wall runs the opposite way in
    # the two faces: that is what keeps the shell closed and the winding consistent. A
    # triangle fan around a centre vertex would pass that test too, but it would hand the
    # painter 32 slivers where there is one disc.
    faces = [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)]
    faces.append(tuple(range(n - 1, -1, -1)))
    faces.append(tuple(range(n, 2 * n)))

    # One normal per face -- flat shading, as the authored parts do it, and the reason a
    # flat end stays flat under the light.
    normals = []
    for k, f in enumerate(faces):
        if k >= n:
            normals.append((0.0, 0.0, -1.0 if k == n else 1.0))
        else:
            normals.append(unit(cross(sub(verts[f[1]], verts[f[0]]),
                                      sub(verts[f[2]], verts[f[0]]))))

    # Per-face islands, laid out on the smallest grid that holds them, each inset by a
    # margin. Wall quads fill their cell; an end maps its ring to a disc inside its cell.
    grid = int(math.ceil(math.sqrt(len(faces))))
    cell = 1.0 / grid
    span = cell * (1.0 - 2.0 * 0.0625)
    inset = (cell - span) * 0.5
    uvs = []
    for k, f in enumerate(faces):
        cx, cy = (k % grid) * cell, (k // grid) * cell
        for j, vi in enumerate(f):
            if k < n:
                u = 0.0 if j in (0, 3) else 1.0
                v = 0.0 if j in (0, 1) else 1.0
            else:
                # Both ends map their ring onto the same disc inside their cell. The
                # bottom face is wound the other way (see the reversed ring above), so
                # walking a disc in CORNER order runs opposite ways at the two ends: a
                # stroke that follows the corner order mirrors between them. Flat fills
                # and radial gradients do not care; a painted seam that has to line up
                # across a cap edge would.
                u = 0.5 + 0.5 * math.cos(ang[vi])
                v = 0.5 + 0.5 * math.sin(ang[vi])
            uvs.append((cx + inset + u * span, cy + inset + v * span))

    # The uv table is per corner, so face k's corners run from base(k) in order.
    def base(k):
        return sum(len(f) for f in faces[:k])

    out = ["# generated by gen_adapter.py (frustum r1=%.6g r2=%.6g h=%.6g segs=%d)"
           % (r1, r2, h, n),
           "o %s" % a.name]
    for v in verts:
        out.append("v %.6f %.6f %.6f" % v)
    for t in uvs:
        out.append("vt %.6f %.6f" % t)
    for v in normals:
        out.append("vn %.6f %.6f %.6f" % v)
    for k, f in enumerate(faces):
        out.append("f " + " ".join("%d/%d/%d" % (vi + 1, base(k) + j + 1, k + 1)
                                   for j, vi in enumerate(f)))
    out.append("")

    with open(a.dst, "w") as fh:
        fh.write("\n".join(out))
    print("%s: %d verts, %d faces (%d wall quads + 2 ngons), %d uv islands "
          "(r1=%.6g r2=%.6g h=%.6g)"
          % (a.dst, len(verts), len(faces), n, len(faces), r1, r2, h))


if __name__ == "__main__":
    main()
