#!/usr/bin/env python3
"""Generate the balloon-platform meshes: a hexagonal deck (the collider) and
the full visual (deck + six balloons on long tethers). The deck top is local
z=0 -- the building origin convention (space_port.obj): a spawn rests its gear
on the pose origin. Balloons and tethers are VISUAL ONLY: the collision mesh is
the deck alone, so a ship passes through the balloons (v1, by design).

Sizing story (reports/air-buildings2026_11_14): at 56 km on Venus a 10 t
platform needs ~13,500 m3 of helium; six balloons r=8.5 m carry ~15,400 m3
(~14% margin). Tethers ~60 m for visual impact.
"""
import argparse
import os
import numpy as np
import trimesh

from gen_out import add_args, ship


DECK_R = 40.0      # hex circumradius: 80 m across corners
DECK_T = 2.0       # slab thickness, below z=0
BALLOON_R = 8.5
BALLOON_Z = 68.0   # balloon centre height -> ~60 m tethers
TETHER_R = 0.15
TETHER_RING = 32.0   # tether anchor ring, inside the deck edge
N = 6


def deck():
    d = trimesh.creation.cylinder(radius=DECK_R, height=DECK_T, sections=N)
    d.apply_translation([0, 0, -DECK_T / 2.0])   # top at z=0
    return d


def visual():
    parts = [deck()]
    for i in range(N):
        a = 2.0 * np.pi * i / N
        x, y = TETHER_RING * np.cos(a), TETHER_RING * np.sin(a)
        b = trimesh.creation.icosphere(subdivisions=3, radius=BALLOON_R)
        b.apply_translation([x, y, BALLOON_Z])
        parts.append(b)
        tether_h = BALLOON_Z - BALLOON_R
        t = trimesh.creation.cylinder(radius=TETHER_R, height=tether_h, sections=6)
        t.apply_translation([x, y, tether_h / 2.0])
        parts.append(t)
    return trimesh.util.concatenate(parts)


def main():
    ap = argparse.ArgumentParser(
        description="Generate res/meshes/balloon_platform*.obj (sizing story in the header)")
    add_args(ap)
    a = ap.parse_args()

    d = deck()
    v = visual()
    dst_d = ship(a.out, "meshes", "balloon_platform_deck.obj", force=a.force)
    dst_v = ship(a.out, "meshes", "balloon_platform.obj", force=a.force)
    d.export(dst_d, include_normals=True)
    v.export(dst_v, include_normals=True)
    print("%s: extents %s (collider)" % (dst_d, np.round(d.extents, 2)))
    print("%s: extents %s (visual)" % (dst_v, np.round(v.extents, 2)))


if __name__ == "__main__":
    main()
