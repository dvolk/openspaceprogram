import sys
import numpy as np
import trimesh

open_mesh = "--open" in sys.argv
path = [a for a in sys.argv[1:] if a != "--open"][0]

mesh = trimesh.load_mesh(path, process=False)
# OBJ faces carry per-corner v/vt/vn triples: the same geometric rim vertex
# has different UV/normal indices on the smooth side than on the flat cap,
# so trimesh splits it into separate corners and reports phantom boundary
# edges. Merge by position so the watertight test sees true topology.
# (merge_tex/merge_norm=True means "ignore" tex/norm differences.)
mesh.merge_vertices(merge_tex=True, merge_norm=True)

checks = {
    "finite": (
        np.isfinite(mesh.vertices).all()
        and np.isfinite(mesh.faces).all()
    ),
}
if not open_mesh:
    # --open: intentionally open surfaces (e.g. the engine plume billboard)
    checks["watertight"] = mesh.is_watertight
checks["winding_consistent"] = mesh.is_winding_consistent
if not open_mesh:
    checks["volume"] = mesh.is_volume

for name, ok in checks.items():
    print(f"{name}: {'OK' if ok else 'FAIL'}")

if not all(checks.values()):
    sys.exit(1)
