#!/usr/bin/env python3
"""What Blender sees in a part mesh: poly types, UV islands, blended hard edges.

This is the editor's-eye view that the byte-level gates cannot give. The engine never
notices any of it -- Mesh::AssImpFromFile runs aiProcess_Triangulate, so quads and ngons
collapse to triangles on import and a collapsed UV set still renders fine when the
texture is one flat color. A person opening the part in Blender notices all of it: a
triangulated wall unwraps into zig-zag slivers, a mesh whose UVs are all (0.5, 0.5)
addresses one texel and so cannot be painted at all, and a cap corner that carries the
wall's radial normal shades the flat end of a frustum as a shallow dome.

Numbers this reports, and what the shipped art does about them:

  quad/ngon counts   the Blender-authored family (tank_r1h3.obj, capsule.obj) is 32 wall
                     quads + 2 ngon caps; every generator-authored mesh is all triangles.
  uv_islands         one per face in the authored family (34), so each face owns a patch
                     of the PNG; gen_fuselage.py collapses to 1 island covering 0 area.
  uv_area            the summed area of the islands. Over 1.0 means islands overlap each
                     other across the canvas: paint one panel, several change.
  max_smooth_gap     the largest angle between a loop normal and its own face normal.
                     0.0 is one normal per face (the house style); 90 deg means a cap is
                     being smoothed into the wall it meets at a right angle.

Blender is NOT a build or CI dependency -- lint.yml is stdlib-only on purpose and the
enforcing gate for these properties lives in utils/check_mesh_pins.py. This script is
for whoever is painting a part, or for checking a generator change by eye. It needs a
local Blender; on this box that is ~/opt/blender-4.5.9-linux-x64/blender (LTS 4.5.9,
the version whose numbers the tickets quote).

Usage:
  <blender> --background --factory-startup --python utils/inspect_mesh_blender.py -- \
      res/meshes/capsule_cyl_r1h2.obj res/meshes/tank_r1h3.obj ...
"""
import math
import sys

import bpy

args = sys.argv
paths = args[args.index("--") + 1:] if "--" in args else args[1:]


def uv_islands(me):
    """Connected components of faces across UV space.

    Two faces are the same island when the edge they share carries the same UV on both
    sides; a seam (the same vertex at two different UVs) breaks the join. Blender 4.5's
    Python API has no MeshEdge.link_loops, so loops are indexed by their vertex pair."""
    uv = me.uv_layers.active
    if uv is None:
        return None
    by_edge = {}
    for p in me.polygons:
        li = p.loop_indices
        for k in range(len(li)):
            a = me.loops[li[k]].vertex_index
            b = me.loops[li[k - 1]].vertex_index
            by_edge.setdefault((a, b) if a < b else (b, a), []).append((li[k], p.index))
    parent = list(range(len(me.polygons)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for sides in by_edge.values():
        if len(sides) != 2:
            continue
        (la, fa), (lb, fb) = sides
        if tuple(uv.data[la].uv) == tuple(uv.data[lb].uv):
            ra, rb = find(fa), find(fb)
            if ra != rb:
                parent[rb] = ra
    return len({find(p.index) for p in me.polygons})


def uv_area(me):
    """Summed UV-space area of every face. Over 1.0 with a 0..1 canvas means overlap."""
    uv = me.uv_layers.active
    if uv is None:
        return 0.0
    total = 0.0
    for p in me.polygons:
        vs = [tuple(uv.data[li].uv) for li in p.loop_indices]
        a = 0.0
        for i in range(len(vs)):
            x1, y1 = vs[i]
            x2, y2 = vs[(i + 1) % len(vs)]
            a += x1 * y2 - x2 * y1
        total += abs(a) * 0.5
    return total


def max_smooth_gap(me):
    """Largest angle, in degrees, between a loop normal and its face's own normal."""
    worst = 0.0
    for p in me.polygons:
        for li in p.loop_indices:
            d = max(-1.0, min(1.0, me.loops[li].normal.dot(p.normal)))
            worst = max(worst, math.degrees(math.acos(d)))
    return worst


for path in paths:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.obj_import(filepath=path)
    for ob in [o for o in bpy.context.scene.objects if o.type == "MESH"]:
        me = ob.data
        tri = sum(1 for p in me.polygons if len(p.vertices) == 3)
        quad = sum(1 for p in me.polygons if len(p.vertices) == 4)
        ngon = len(me.polygons) - tri - quad
        isl = uv_islands(me)
        print("%-36s verts=%-4d polys=%-4d tri=%-4d quad=%-4d ngon=%-3d "
              "uv_islands=%-5s uv_area=%.3f max_smooth_gap=%5.1fdeg"
              % (path.split("/")[-1], len(me.vertices), len(me.polygons),
                 tri, quad, ngon, "?" if isl is None else isl,
                 uv_area(me), max_smooth_gap(me)))
