#!/usr/bin/env python3
"""Generate the landing-gear .obj: a wheel (axle along X) on a short strut that
rises along +Z to the mount face. Origin at the axle centre. The part frame is
the one src/wheel.h / Vehicle::applyGroundForces assume: axle X, rolling
direction +Y, strut up +Z. The surface-attach node is authored at the strut top
(parts.json "nodes"), so the gear hangs below a fuselage belly.
"""
import os

import numpy as np
import trimesh

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

R = 0.35          # wheel radius (m) -- also PartDef.wheel_radius
WIDTH = 0.22      # wheel width along the axle (X)
STRUT = 0.4       # strut length up +Z
STRUT_TOP = 0.65  # mount face height (the surface node sits here)


def main():
    wheel = trimesh.creation.cylinder(radius=R, height=WIDTH, sections=24)
    # trimesh's cylinder axis is Z; turn it onto X so the axle is X.
    wheel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    strut = trimesh.creation.box(extents=[0.12, 0.12, STRUT])
    strut.apply_translation([0, 0, STRUT_TOP - STRUT / 2])
    gear = trimesh.util.concatenate([wheel, strut])
    dst = os.path.join(REPO_ROOT, "res", "meshes", "gear.obj")
    gear.export(dst)
    print("%s: wheel R=%.2f axle X, strut top Z=%.2f, extents %s"
          % (dst, R, STRUT_TOP, np.round(gear.extents, 3)))


if __name__ == "__main__":
    main()
