#!/usr/bin/env python3
"""Generate orbital-building meshes: a cross-shaped station and a box space dock.
Simple v1 shapes (the torus-hab collision case is noted in the plan). Authored in
the building's local frame; body-fixed attitude points local +Z at the planet.
"""
import argparse
import os
import numpy as np
import trimesh

from flatten_normals import flatten
from gen_out import add_args, ship

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def station():
    # A central hub with two perpendicular arms (a cross). Non-convex overall.
    # NOTE: v1 collision is the single convex hull of this mesh (its bounding
    # box), so the gaps between the arms are solid to a ship. A compound of the
    # individual boxes (true cross collision) is a follow-up.
    hub = trimesh.creation.box(extents=[6, 6, 10])
    arm_x = trimesh.creation.box(extents=[40, 3, 3])
    arm_y = trimesh.creation.box(extents=[3, 40, 3])
    return trimesh.util.concatenate([hub, arm_x, arm_y])


def dock():
    # A simple box dock with a hollow-ish mouth suggested by a smaller block.
    body = trimesh.creation.box(extents=[20, 20, 30])
    return body


def main():
    ap = argparse.ArgumentParser(
        description="Generate res/meshes/{station,spacedock}.obj (v1 building shapes)")
    add_args(ap, ROOT)
    a = ap.parse_args()

    for name, mesh in [("station", station()), ("spacedock", dock())]:
        dst = ship(a.out, "meshes", name + ".obj", force=a.force)
        # trimesh writes area-weighted corner normals, so a box station shades like a
        # balloon. flatten() gives every face its own normal (#1178) and copies the
        # positions through untouched, which is what keeps the collision hull and the
        # volume-derived mass exactly where they were.
        text, _ = flatten(mesh.export(file_type="obj", include_normals=True))
        with open(dst, "w") as f:
            f.write(text)
        print("%s: extents %s" % (dst, np.round(mesh.extents, 2)))


if __name__ == "__main__":
    main()
