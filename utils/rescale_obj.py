#!/usr/bin/env python3
"""Rescale a part .obj: sx=sy=radius, sz=height/2 (2 m base cube). Normals by inverse transpose."""

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
