#!/usr/bin/env python3
"""Unwrap a generated mesh the way a person would in the editor: Smart UV Project.

gen_lathe.py lays one island per face, packed by its own shelf packer. That is exact and
script-reproducible, and it is unusable to paint: engine.obj at 32 segments is 130 islands
scattered over the canvas, so a ring around a nozzle or a stripe down a chamber cannot be
drawn -- each band comes back as 32 separate patches. The hand-authored family does not look
like that (capsule.obj is 3 islands: the wall as one strip and two caps), which is why that
art could be painted at all.

Smart UV Project joins faces that project in the same direction, so each band of a lathe
unrolls as a few long arc patches instead of 32 single quads, and it packs them tighter than
the shelf packer does. Measured on this profile at 32 segments (tmp/night7, 4.5.9):

    islands  130 -> 13      canvas fill 0.606 -> 0.779
    px/m across the part  88..88 (spread 1.00) -> 77..104 (spread 1.29)

The density wander is the cost, and it is the size of the debt the hand-authored parts
already carry: tank_r1h3.obj measures 67..82 px/m, spread 1.22, and the gate's ceiling
(check_mesh_pins.py DENSITY_MAX_SPREAD) is 2.0. Spread is the gate's measure -- worst face over
the part median -- and 1.29 is the worst of the eight bodies, the others 1.10..1.28. Geometry
does not move: same 130 faces, same quads and ngon discs, one normal per face, same volume to
the exported precision. The exporter does write `s 1` smoothing groups; this project's assimp
ignores them (middleware/assimp/code/AssetLib/Obj/ObjFileParser.cpp:405 skips the field), so the
flat shading comes from the one `vn` per face, as everywhere else in the catalogue.

Determinism matters here because check_mesh_pins.py byte-compares a regenerated engine
against the committed one. Blender 4.5.9's Smart UV Project is deterministic: two runs on the
same input produced byte-identical .obj files, in a different cwd and with a different TMPDIR
(md5 36dd58a3c74a6fc2e8d39b7dfec6d045, matching the committed file). The angle limit and margin
are pinned as argv in PINS, so a change to either is a diff.

This script does NOT write the provenance header; the caller that invoked Blender does, so
the file keeps claiming the generator that made it (check_mesh_pins.py HEADER scans for that
first line) and keeps the profile that produced it readable.

Blender is not a build or CI dependency -- lint.yml runs check_mesh_pins.py --committed,
which inspects shipped meshes and regenerates nothing. Blender is a dependency of `make test`
(test-art) and of any run of gen_engine.py, on this box
~/opt/blender-4.5.9-linux-x64/blender.

Usage:
  <blender> --background --factory-startup --python utils/art/uv_smart_blender.py -- \
      IN.obj OUT.obj [ANGLE_DEG] [MARGIN]
"""
import math
import sys

import bpy

args = sys.argv
tail = args[args.index("--") + 1:]
src, dst = tail[0], tail[1]
angle_deg = float(tail[2]) if len(tail) > 2 else 66.0
margin = float(tail[3]) if len(tail) > 3 else 0.008

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=src)
meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
assert len(meshes) == 1, "expected one mesh in %s, found %d" % (src, len(meshes))
ob = meshes[0]
# The object keeps the transform the importer gave it. obj_import reads our world-Z-up files
# into Blender's Y-up world by rotating the object; the exporter's axis pair below undoes
# exactly that. Zeroing the rotation here -- which the render probes in reports/ do, because a
# camera wants the part upright in the frame -- lays the finished part on its side: measured on
# an orbital engine (r1 h1), up_axis="Z" or a zeroed rotation both land the 1 m axis along X.

# The UV layer gen_lathe.py wrote is dropped rather than overwritten: Smart UV Project should
# see the same input whether or not the mesh came off a lathe with islands already on it.
for layer in list(ob.data.uv_layers):
    ob.data.uv_layers.remove(layer)

for o in bpy.context.selected_objects:
    o.select_set(False)
ob.select_set(True)
bpy.context.view_layer.objects.active = ob

bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.uv.smart_project(angle_limit=math.radians(angle_deg), island_margin=margin,
                         correct_aspect=True, scale_to_bounds=False)
bpy.ops.object.mode_set(mode="OBJECT")

# The exporter's axis pair is obj_import's own, restated rather than left to defaults: this is
# what puts the part back world-Z-up. A pair that does not match the import rotates the part
# (see the note above), and nothing downstream notices except the part lying on its side.
bpy.ops.wm.obj_export(filepath=dst, export_materials=False, export_uv=True,
                      export_normals=True, export_triangulated_mesh=False,
                      forward_axis="NEGATIVE_Z", up_axis="Y")
print("unwrapped %s -> %s: %d faces, angle_limit %g deg, margin %g"
      % (src, dst, len(ob.data.polygons), angle_deg, margin))
