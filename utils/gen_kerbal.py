#!/usr/bin/env python3
"""Generate res/kerbal.obj + res/kerbal.png: the EVA placeholder character.

A green "cucumber" -- a capsule on the part convention (origin centered,
long axis = +Z): radius RADIUS, cylindrical section CYL_HEIGHT, hemispherical
caps (total height CYL_HEIGHT + 2*RADIUS). The catalog entry is derived from
the mesh by utils/gen_parts.py like any other part; the ship def that builds
a kerbal is res/ships/kerbal.json.

    python3 utils/gen_kerbal.py            # writes res/kerbal.obj + .png
    python3 utils/gen_kerbal.py --dry-run  # print the geometry, write nothing
"""

import argparse
import math
import os

import trimesh

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RADIUS = 0.35        # m, cross-section
CYL_HEIGHT = 1.0     # m, cylindrical section (total = CYL_HEIGHT + 2*RADIUS)
SEGMENTS = 16        # around; the caps use the same count

# bright placeholder green; the parts shader samples the flat texture
KERBAL_RGB = (60, 200, 60)


def build_capsule():
    m = trimesh.creation.capsule(height=CYL_HEIGHT, radius=RADIUS,
                                 count=[SEGMENTS, SEGMENTS])
    # part convention: long axis = +Z, centered at the origin. The facet
    # ring is inscribed, so the cross-section undershoots 2*RADIUS by a
    # hair; the height is exact.
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

    # trimesh's own OBJ export omits vertex normals, and the game's mesh
    # loader expects them -- write v/vn/f by hand (no UVs: the loader
    # falls back to (0,0), which samples the flat green texture)
    obj_path = os.path.join(REPO_ROOT, "res", "kerbal.obj")
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

    # flat green texture (no UVs in the mesh: the loader falls back to
    # (0,0), which samples this one colour)
    from PIL import Image
    img = Image.new("RGB", (64, 64), KERBAL_RGB)
    png_path = os.path.join(REPO_ROOT, "res", "kerbal.png")
    img.save(png_path)
    print("wrote %s" % png_path)


if __name__ == "__main__":
    main()
