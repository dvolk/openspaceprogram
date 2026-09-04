#!/usr/bin/env python3
"""Rescale a part .obj mesh.

Part meshes are authored centered at the origin with the stack/thrust axis
along +Z. The base parts are 2 m cubes (radius 1 m, height 2 m), so a part of
radius r and height h is a rescale by

    sx = sy = r        sz = h / 2

e.g.  rescale_obj.py res/engine.obj res/engine_r1.5h3.obj --sx 1.5 --sy 1.5 --sz 1.5

Positions scale by (sx, sy, sz); normals by the inverse transpose
(diagonal scale -> (1/sx, 1/sy, 1/sz)) and are re-normalized, which is exact
for axis-aligned scales. UVs and faces pass through untouched, so textures
and the index layout are preserved.
"""

import argparse
import math


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src", help="source .obj (2 m cube part)")
    ap.add_argument("dst", help="destination .obj")
    ap.add_argument("--sx", type=float, required=True, help="x scale (radius)")
    ap.add_argument("--sy", type=float, required=True, help="y scale (radius)")
    ap.add_argument("--sz", type=float, required=True, help="z scale (height/2)")
    a = ap.parse_args()
    if min(a.sx, a.sy, a.sz) <= 0:
        ap.error("scales must be > 0")

    out = []
    for line in open(a.src):
        t = line.split()
        if t and t[0] == "v" and len(t) >= 4:
            x, y, z = (float(v) * s for v, s in zip(t[1:4], (a.sx, a.sy, a.sz)))
            line = "v %.6f %.6f %.6f\n" % (x, y, z)
        elif t and t[0] == "vn" and len(t) >= 4:
            nx, ny, nz = (float(v) * s for v, s in zip(t[1:4], (1 / a.sx, 1 / a.sy, 1 / a.sz)))
            L = math.sqrt(nx * nx + ny * ny + nz * nz)
            line = "vn %.6f %.6f %.6f\n" % (nx / L, ny / L, nz / L)
        out.append(line)

    with open(a.dst, "w") as f:
        f.writelines(out)
    print("%s -> %s (scale %g, %g, %g)" % (a.src, a.dst, a.sx, a.sy, a.sz))


if __name__ == "__main__":
    main()
