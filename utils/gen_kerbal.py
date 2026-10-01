#!/usr/bin/env python3
"""Generate res/meshes/kerbal.obj + res/textures/kerbal.png: the EVA placeholder capsule (axis +Z)."""

import argparse
import math
import os

import trimesh

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RADIUS = 0.2         # m, cross-section
CYL_HEIGHT = 0.35    # m, cylinder section (total height = 0.75)
SEGMENTS = 16

KERBAL_RGB = (60, 200, 60)


def build_capsule():
    m = trimesh.creation.capsule(height=CYL_HEIGHT, radius=RADIUS,
                                 count=[SEGMENTS, SEGMENTS])
    # facet ring is inscribed, so cross-section undershoots 2*RADIUS slightly
    ext = m.extents
    assert abs(ext[0] - 2 * RADIUS) < 0.01, ext
    assert abs(ext[1] - 2 * RADIUS) < 0.01, ext
    assert abs(ext[2] - (CYL_HEIGHT + 2 * RADIUS)) < 1e-6, ext
    assert m.is_watertight
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry-run", action="store_true",
                    help="print the geometry without writing")
    a = ap.parse_args()

    m = build_capsule()
    volume = float(m.volume)
    print("kerbal capsule: r=%.2f m, cyl=%.2f m, total=%.2f m, "
          "volume=%.4f m^3, %d triangles" %
          (RADIUS, CYL_HEIGHT, CYL_HEIGHT + 2 * RADIUS, volume,
           len(m.faces)))

    if a.dry_run:
        print("[dry-run] not writing")
        return

    # trimesh's OBJ export omits the normals the loader wants; write v/vn/f
    # by hand. No UVs: the loader falls back to (0,0), sampling the flat fill.
    obj_path = os.path.join(REPO_ROOT, "res", "meshes", "kerbal.obj")
    with open(obj_path, "w") as f:
        f.write("# gen_kerbal.py: the EVA placeholder capsule\n")
        for v in m.vertices:
            f.write("v %.8f %.8f %.8f\n" % (v[0], v[1], v[2]))
        for n in m.vertex_normals:
            f.write("vn %.6f %.6f %.6f\n" % (n[0], n[1], n[2]))
        for face in m.faces:
            f.write("f %d//%d %d//%d %d//%d\n" % (
                face[0] + 1, face[0] + 1,
                face[1] + 1, face[1] + 1,
                face[2] + 1, face[2] + 1))
    print("wrote %s" % obj_path)

    from PIL import Image
    img = Image.new("RGB", (64, 64), KERBAL_RGB)
    png_path = os.path.join(REPO_ROOT, "res", "textures", "kerbal.png")
    img.save(png_path)
    print("wrote %s" % png_path)


if __name__ == "__main__":
    main()
