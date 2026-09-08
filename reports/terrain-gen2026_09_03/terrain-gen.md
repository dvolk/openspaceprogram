# Terrain generation — code and data

Snapshot of how planet/moon/sun terrain is generated, rendered, and used for
gameplay, as of 2026-09-03. All terrain is procedural: there are no heightmap
textures or mesh files; every body's surface is an analytic star function in
its own rotating frame, meshed on the fly in a patch tree.

## Architecture

Three layers, split so the math can be tested without GL/Bullet/game state:

```
src/terragen.h       pure math (glm + STL only): noise, height + color
                     functions, palettes, the grid builder.
                     tests/test_terrain.cpp pins it.
src/terrain.h        GeoPatch (LOD node) + TerrainBody (per-body state)
src/terrain.cpp      implementations: patch tree ops, the async subdivision
                     job, collision hookup, terrain shadow, atmosphere mesh.
src/system.cpp       load_system(): JSON -> Surface params -> root patches
src/job.h            JobRunner: the single worker thread terrain gen posts to
```

Consumers (all read the *analytic* height function, not the meshes):
`eva.cpp` (grounding + floor guard), `ships.cpp` (spawn/pad placement),
`vehicle.h` (rails classification), `gameui.cpp` (altitude readouts),
`tick.cpp` (crash checks), `surfmap.cpp` (the 2-D surface map),
`terrain.cpp` (LOD distance, terrain shadow), `radialtest.cpp`.

## The math (terragen.h)

**Noise** — `terrainNoise3d`: fractal Brownian motion of 3-D simplex
(`glm::simplex`), per-body defaults: 12 octaves, persistence 0.6, scale
doubling per octave starting at 2. Input is `p * frequency + seed_offset`;
`seed_offset = vec3(seed * 100)` from the body's `seed` JSON field.

**Height** — `terrainHeight(p)`, a star function in the body's rotating
frame (the frame its meshes are built in):

```
noise  = fbm(p) * amplitude            (amplitude: raw noise scale, [m])
if has_sea and noise < sea_level: noise = sea_level
return radius + scale(noise)
scale(n) = n * (n / 3000)^power        (sign-safe)
```

`ref_height = 3000` (marked "guess" in the code) is the magic number the
exponent is normalized against. Two consequences:

- `amplitude` is NOT the peak relief. Effective relief of a full-amplitude
  peak is `amplitude * (amplitude/3000)^power` (computed into
  `surface.max_height` in `system.cpp`). For the KSP system:
  Kerbin/Duna/Shay/Tylo (amp 2500, power 3) ~1.4 km; Eve/Laythe (amp 3000,
  power 3) 3 km; Eeloo (amp 5000, power 3) ~23 km; moons (power 1)
  0.1–1.3 km (Pol 120 m → Minmus 1.3 km).
- With the data's `sea_level = 0`, an ocean is a perfectly flat sphere at
  exactly `radius` (scale(0) = 0).

**Gas giants** — `bands = true` (Jool) short-circuits everything: height is
exactly `radius`, color is a latitude triangle wave through the palette
(`band_count` stripes pole-to-pole, odd count => bright equator).

**Color** — `terrainSurfaceColor(p)`: per-vertex, baked at grid-build time
(the same function the 2-D surface map samples, so they match):

```
h = terrainHeightUnscaled(p)                       (radius + clamped noise)
h += fbm1(p * radius + seed_offset) * 100 / 2      (±50 m color jitter)
if palette empty: colour_func(h, radius-1, radius+3000)   (type fallback)
else:            palette( (h - radius - sea_level) / max_height )
c = 0.5*c + brightness(c)                          (desaturate/brighten)
if has_sea and h <= radius + sea_level: sea_color
```

Type fallbacks (`colour_func`, assigned by `system.cpp`): `GetColourSun`
(flat yellow), `GetColourMoon` (flat grey — dead in practice, every moon in
the data has a palette), `GetColourEarth` (4-segment height ramp).

## Geometry and LOD (terrain.h / terrain.cpp)

**Patch tree.** `TerrainBody::Create()` builds six root patches (depth 1)
synchronously at load — one per face of the cube whose 8 corners
(normalized `(±1,±1,±1)`) are the patch corner directions; bilinear blends
normalized back onto the sphere tile each face. Each patch has 4 children
built from its 4 corners + 4 edge midpoints + the normalized center
(`subdivideCorners`), up to `max_depth = 14`.

**Grid** — `buildGridGeom` (terragen.h): fixed 25×25 vertices (625), 24×24
quads. With a skirt (all children) the grid is 27×27 and the indices split
into an inner block (24×24×6 = 3456) + a skirt ring (600). Normals are 3×3
finite differences that sample the *analytic* heightfield one cell past the
patch boundary, so neighbouring patches at different depths compute matching
seam normals.

**Skirts + stencil.** Cracks open between adjacent patches at different
subdivision depths (their edge polylines sample the heightfield at different
points). Each child patch carries a one-cell skirt ring dropped to the patch
lowest radius × 0.999995, with normal/color copied from the adjacent edge.
`TerrainBody::Draw` renders two passes with a stencil mask: pass 1 draws
terrain and stamps stencil=1; pass 2 draws skirts only where stencil==0 —
i.e. the cracks and the limb. No depth comparison is involved (the float32
view transform can't resolve the skirt's tiny depth margin at range; z-fight
zippering is why root patches get *no* skirt at all).

**LOD** — `GeoPatch::Update`, per frame: projected width in screen pixels
(small-angle: `radius * |v0-v3| / dist * viewport_h / fov_h`, with `dist`
to the patch's *own* surface point at its centroid). Subdivide while
wider than `args.terrain_px` (default 512; `--terrain-px` 32..1024, live
"Terrain detail" slider in the debug window); collapse below half of that —
the hysteresis band stops flapping. The px budget (not metres/degrees)
follows FOV, zoom and resolution automatically.

**Async subdivision.** `requestSubdivide` posts a JobRunner job: the worker
builds the four children's `GridGeom`s (pure math over a `TerrainParams`
snapshot), and returns a main-thread continuation that attaches them (GL
upload + Bullet collision). The parent keeps drawing while the job is in
flight, so there is never a hole. Race handling: `TerrainBody::alive`
(`std::set<GeoPatch*>`, ctor-insert/dtor-erase) + the
`subdivide_in_flight` flag let the continuation discard its grids when a
zoom-out collapsed the parent while the job was building. The worker can
never touch game state (job.h's pure-work/publish contract).

**Collision.** Leaf patches at `depth >= max_depth` (14) get a
`btBvhTriangleMeshShape` (static, 0.5 m margin) over the grid triangles;
`~GeoPatch` removes it from the Bullet world. Collision therefore exists
only under the camera at max LOD — which is why the analytic height
function, not the collision mesh, is the ground truth for standing on the
ground (see below). The old `depth > max_depth` test was never true and
silently added no collision; the `>=` in the ctor is the fix.

**Rendering.** Per-vertex color, one directional light clamped to a 0.05
floor, log depth buffer (outerra-style, `C=11`, `far=1e13`) in
`res/terrainShader.vs/.fs`. View×Model is computed in double precision in
the render frame (origin at the ship COM), then truncated to float for the
shader. The atmosphere rim (optional per body) is a separate 128×128 UV
shell at `radius + max_height + thickness`, Fresnel limb glow lit by the
sun direction (so the rim brightens on the day side), transparent with
depth-write off, drawn after the skybox. See
`reports/atmosphere2026_08_25` for that design.

## Data

`Surface` (per body, value-copyable so a worker can snapshot it):
`amplitude` (2500 m), `octaves` (12), `persistence` (0.6), `frequency` (1.0),
`power` (3), `has_sea`, `sea_level` (0), `sea_color`, `palette` (stops),
`max_height` (computed), `seed_offset` (from `seed`), `bands` / `band_count`
(Jool), `atmosphere` (color/thickness/power/intensity).

**JSON** — `res/ksp_system.json` (default `--system`), 18 bodies:
Kerbol + 8 planets + 9 moons. Per body: identity (name/type/orbits),
physics (radius/mass/g), orbit (inertial block), spin (rotating block), and
the terrain: legacy top-level `seed`, `has_sea`, `power_scaler` (defaults)
plus a `surface` block (amplitude, palette, sea_color, atmosphere) that
overrides. `res/old_system.json` is the 3-body Eerbon test world (seed 0,
no palettes — the type-fallback colors).

| body   | type   | radius km | amplitude m | sea | palette stops | bands | atmo thk m |
|--------|--------|-----------|-------------|-----|---------------|-------|------------|
| Kerbol | star   | 261 600   | (default 2500) | – | 2 | – | – |
| Moho   | planet | 250       | 800         | –   | 3 | –   | –          |
| Eve    | planet | 700       | 3000        | –   | 3 | –   | 25 000     |
| Gilly  | moon   | 13        | 400         | –   | 3 | –   | –          |
| Kerbin | planet | 600       | (default 2500) | yes | 4 | – | 15 000   |
| Mun    | moon   | 200       | 1500        | –   | 2 | –   | –          |
| Minmus | moon   | 60        | 2000        | –   | 2 | –   | –          |
| Shay   | planet | 550       | (default 2500) | yes | 0 (fallback) | – | 22 000 |
| Duna   | planet | 320       | (default 2500) | – | 3 | – | 8 000    |
| Ike    | moon   | 130       | 1000        | –   | 2 | –   | –          |
| Dres   | planet | 138       | 2000        | –   | 2 | –   | –          |
| Jool   | planet | 6 000     | (default 2500) | – | 2 | 9 | 90 000   |
| Laythe | moon   | 500       | 3000        | yes | 3 | –   | 12 000     |
| Vall   | moon   | 300       | 2000        | –   | 2 | –   | –          |
| Tylo   | moon   | 600       | (default 2500) | – | 2 | – | –        |
| Bop    | moon   | 65        | 800         | –   | 2 | –   | –          |
| Pol    | moon   | 44        | 600         | –   | 2 | –   | –          |
| Eeloo  | planet | 210       | 5000        | –   | 2 | –   | –          |

No body sets `octaves`/`persistence`/`frequency`/`sea_level` — all run at
defaults. The data is generated by `utils/gen_systems.py` (the per-body
catalog + `SURFACES` table are hardcoded there; orbital elements come from
`ksp_bodies.csv`); see `reports/ksp-data2026_08_27` for that pipeline.

## Where the terrain is load-bearing

- **EVA** (`eva.cpp`): grounded = Bullet contact OR analytic altitude
  within the standing band; the analytic floor guard snaps the kerbal
  back to standing height where no collision leaf is loaded.
- **Ships**: spawn/pad placement (`ships.cpp`, `radialtest.cpp`) sample
  `GetTerrainHeight` for the landing site.
- **Altitude** in HUD/debug (`gameui.cpp`, `tick.cpp`): `distance -
  GetTerrainHeight(dir)` — this is the number the player sees.
- **Rails** (`vehicle.h::inTerrainBand`): periapsis dipping into the
  terrain band disqualifies a ship from warp.
- **Terrain shadow** (`ComputeTerrainShadow`): ray from the part toward the
  sun; cheap sphere reject, then marching the chord against the analytic
  height function in the rotating frame (8–128 steps); hard 1.0 / 0.15
  result (0.15 matches `partsShader`'s light floor so a shadowed part reads
  as night). One test point per object, by design.
- **Surface map** (`surfmap.cpp`, M key): 256×128 equirectangular sweep of
  the *same* `terrainSurfaceColor` on the worker, optionally shaded by the
  terminator at the request instant.

## Tests

- `tests/test_terrain.cpp` (`make test`): 7 groups — height finite/bounded,
  sea floor, color range, all-sea color exact, bands (height == radius,
  equator ≠ pole), 25×25 grid vertices on the height field + index range,
  skirted 27×27 grid with the ring strictly below the terrain.
- `tests/test_surfmap.cpp`: the map's projection/shading math.
- e2e: `01-smoke` (boots with terrain), `26-eva-walk` (analytic terrain
  grounding), `23/24-surfmap`. The e2e env renders in software GL, which is
  why the CLI comment recommends coarse LOD there (`--terrain-px 1024`).

## Perf notes

- A patch build is ~625 vertices × (2×12-octave fbm for height+color) +
  ~2500 stencil samples for normals ≈ 40–50k simplex evals, all on the
  worker thread; the main thread pays only the GL upload (≤729 verts, ≤4056
  indices) and, at depth 14, one BVH build.
- Per frame per patch: a couple of trig ops + one `GetTerrainHeight` for the
  LOD distance.
- **Flag:** `TerrainBody::params()` value-copies `Surface` (which owns a
  `std::vector<PaletteStop>`) on *every* `GetTerrainHeight` /
  `SurfaceColor` call. That's a heap allocation in hot paths: EVA and the
  HUD call it per frame, and `ComputeTerrainShadow` calls it once per ray
  step (up to 128 × per object). The snapshot is invariant after
  `load_system`, so it could be stored once and shared — cheap win if those
  paths ever show up in `--perf`.

## Odds and ends

- `ref_height = 3000 // guess` appears in `terragen.h` and its twin
  `amplitude/3000` in `system.cpp`'s `max_height` — one magic number, two
  files; change it in one place only and the palettes + atmosphere shell
  radius silently desync.
- Grid resolution (25) and `max_depth` (14) are constants in
  `terragen.h` / `terrain.h`, not data.
- Two JSON spellings of one field: top-level legacy `power_scaler` vs the
  surface block's `power` (the latter wins). The data only uses
  `power_scaler`.
- `GetColourMoon` (flat grey) and the white `PaletteColor` fallback are
  dead in the shipped data — every non-star body has a palette except
  Shay/Eerbon, and both are `has_sea` bodies whose fallback path is
  `GetColourEarth` anyway.
- `requestSubdivide` computes `subdivideCorners` on the worker *and* again
  in the main-thread continuation (kept there so the continuation only
  carries the grids).
- The star (Kerbol) is a normal `TerrainBody` too — six root patches,
  ~2 km relief on a 261 600 km sphere (effectively smooth) — drawn with the
  sun shader. It has a dummy zero-spin rotating frame (no `rotating` JSON
  block).
- Teardown order: `main` joins the JobRunner before freeing the bodies, so
  no in-flight terrain continuation can touch a dead `TerrainBody`
  (survival of the `alive` set + flag check is the belt; the join is the
  suspenders).

## File map

| file | role |
|------|------|
| `src/terragen.h` | pure terrain math: noise, height/color, palettes, `buildGridGeom` |
| `src/terrain.h` | `GeoPatch`, `TerrainBody`, `StaticBuilding` declarations |
| `src/terrain.cpp` | patch tree ops, async subdivision, collision, shadow, atmo mesh |
| `src/system.cpp` | JSON → `Surface` params → `Create()` root patches |
| `src/job.h/.cpp` | the single worker thread + continuation handoff |
| `src/render.cpp` | per-frame `planet->Update(px, jobs)` + `planet->Draw(...)` |
| `src/physics.cpp` | `AddTerrainCollision` (BVH triangle mesh) |
| `src/eva.cpp`, `src/ships.cpp`, `src/vehicle.h`, `src/gameui.cpp`, `src/tick.cpp` | analytic-height consumers |
| `src/surfmap.cpp/.h` | 2-D surface map (same color function) |
| `res/terrainShader.vs/.fs` | per-vertex color + log depth |
| `res/atmosphereShader.vs/.fs` | Fresnel limb shell |
| `res/ksp_system.json`, `res/old_system.json` | body data (terrain params included) |
| `tests/test_terrain.cpp` | pure-math tests |
| `utils/gen_systems.py` | data generation (catalog + surfaces hardcoded) |
