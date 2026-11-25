# utils/art — the part-art toolchain

Mesh and texture generators, the gate that keeps their output honest, and the one-off mesh
tools. Everything that writes into `res/meshes/` or `res/textures/` for a **part** lives here;
terrain, skybox and solar-system data generation stays a level up (or in `heightmaps/`,
`skybox/`), because those pipelines read sources this tree does not track.

No `blender/` subdirectory: the whole toolchain is expected to end up running inside Blender, so
a subdirectory marking which scripts do that today would only record a temporary state.

## Blender

4.5.9 LTS (`~/opt/blender-4.5.9-linux-x64/blender`) on this machine. Headless form, and note the
`--` that separates Blender's own arguments from the script's:

```
~/opt/blender-4.5.9-linux-x64/blender --background --factory-startup \
    --python utils/art/inspect_mesh_blender.py -- res/meshes/capsule.obj
```

Under the Blender binary `sys.path[0]` is `.../4.5/scripts/startup`, not the script's directory,
so a script here cannot import its siblings — it has to be self-contained or pass everything
through argv. `python3 utils/art/inspect_mesh_blender.py` cannot import `bpy` at all.

## Gates

- `python3 utils/art/check_mesh_pins.py` — regenerate every pinned asset into a temp tree and
  weigh the bytes, then weigh the shipped catalogue against the house rules (flat normals, a
  file-face budget, uv paintability, texel density). The gate itself is stdlib, but the generators
  it runs need `trimesh`/`numpy`, so the regeneration half only runs under `make test` (test-art
  says so out loud when the modules are missing). 28 assets from 11 generators, ~3 s.
  **Advisory by default** (owner's call, 2026-11-25): drift and rule hits print `WARN:` and
  `make test` stays green; `--strict` reddens them. A self-test failure — a rule that stopped
  being able to fail — is red in both modes.
- `python3 utils/art/check_mesh_pins.py --committed` — the same rules over the shipped text, no
  generators, stdlib only, 0.13 s over the 82 shipped meshes. This is the half CI runs
  (`.gitea/workflows/lint.yml`).
- `python3 utils/check_ship_skin.py` — #153: does a `surface` attach land on its parent's skin,
  or float beside it. Advisory too, same reasoning: the offender pile is the attach rule itself,
  and it moves whenever a mesh is edited. `--strict` reddens the gaps; a row the loader REFUSES
  (that ship does not load at all) is an error either way.
- `check_mesh.py` inspects one mesh, `inspect_mesh_blender.py` says what Blender sees in one,
  `rescale_obj.py` rewrites one. No gate runs those three; they are for looking.

## Editing art by hand

That is the normal path now. 26 assets are **shelved** — the eight engine bodies, their shared
`textures/engine.png`, the eight engine shrouds, the six adapters and the three nose caps. They
are edited in an editor, not generated: regenerating them re-laid their UVs and moved paint that
was already authored onto different faces. The generators stay here (`gen_engine.py` computes a
real bell from an expansion ratio if you ever want to compare one), and an asset comes back
under the gate by re-adding its PINS row.

1. Edit the mesh or the PNG. Nothing in the toolchain writes into `res/` behind your back: the
   generators that name their own output refuse without `--force` (`gen_out.py`), and the gate
   only reads.
2. If you changed a mesh, re-stamp the save fixtures: `python3 utils/art/restamp.py`. It runs
   the sweep in `tests/test_shipload.cpp`, reads the hexes that test prints rather than
   re-deriving them, and rewrites the fixture JSON in place. A stale stamp is exactly what makes
   the loader refuse a saved ship, so do this before trusting an e2e save case.
3. Six of the engine bodies share `res/textures/engine.png`, and share one UV layout with each
   other — measured identical face for face, worst deviation 0.000 of the canvas. That is what
   lets one striped texture paint six sizes. Re-unwrapping a single body on its own breaks it
   (measured 1.036 of the canvas drift across the family), so unwrap the whole family the same
   way or not at all.
4. A generated file says so on its first line. The shelved ones still carry the old claim; the
   gate excuses them (the `SHELVED` list), so leaving it or rewriting it is your call — the
   claim is stale either way.

## Writing an asset

- `gen_out.py` is the write guard (#1051). `add_args(ap)` gives a generator `--out ROOT` (write
  under another tree) and `--force`; `ship(out_root, "meshes", "x.obj")` returns the path, makes
  the directory, and **refuses to replace a shipped file without `--force`**. Six generators name
  their own output (`gen_gear`, `gen_kerbal`, `gen_ladder`, `gen_leg`, `gen_platform`,
  `gen_station`) and depend on this; the rest take a destination path as an argument, which is
  already an explicit intent.
- One exception to that vocabulary: `gen_parts.py --out` is the **parts.json file path**, not a
  ROOT (it has always named its own single file, and `--check` weighs that same path). Pass it a
  tree and you get a file named after the tree.
- `REPO_ROOT` comes from `gen_out.py` — one spelling, because every script here sits the same
  depth below the repo. Do not re-derive it per file.
- A generated file says so on its first line: `# generated by gen_wing.py (...)`. The gate fails a
  `res/meshes/*.obj` that claims to be generated without being pinned or excused with a reason,
  and `flatten_normals.py` (per-face normals, #1178) preserves that line.
- The pin table, the excuse table (`STYLE_OK`) and the self-tests are in `check_mesh_pins.py`.
