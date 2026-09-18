# Code ugliness audit

Snapshot of 2026-09-18, commit `d3bd68b` ("scene: the Tracking Station").
Scope: all of `src/` (88 files, ~28.3k lines), `tests/`, `e2e/`, `Makefile`.

Method: four parallel full-file deep reads (simulation core, rendering, UI/game
flow, persistence/tests/build), mechanical scans (function lengths, casts,
global state, include graph), and spot-verification of every HIGH finding
against the live tree. Findings are ranked by how much they hurt: a finding
is "ugly" here if it makes a *next* change slower, riskier, or impossible to
verify — not because it is stylistically impure.

What the codebase does well (so the ugliness is judged against it): the
asset registries (mesh/shader/texture) are shared, cached, and
placeholder-on-failure; the code is unusually and consistently well-commented
— most "why" questions are already answered in-line; the recent scene-stack
and window-table refactors (`scene.h`, `uiwins.h`) are genuinely good; there
are no `#if 0` blocks, no commented-out function bodies hiding, and the test
coverage is broad (39 test sources + 66 e2e cases). The ugliness is
concentrated, not diffuse.

## Executive summary

**The top 10, most ugly first:**

1. **Five 500+ line god functions** own the game's spine: `drawUIReadouts`
   (gameui.cpp:162, 1321 lines, ~15 windows in one function), `main`
   (main.cpp:79, 1003), `parse_cli` (cli.cpp:13, 712), `draw3d`
   (render.cpp:101, 530), `tick` (tick.cpp:18, 519 — the *only* function in
   the file). Nothing in any of them is individually testable.
2. **`Vehicle` is a 3.7k-line god class** with ~110 declared methods
   (vehicle.h) owning fuel, docking, aero, ECS, staging, SOI frames, rails,
   and telemetry — and it exposes Bullet internals (`btTransform`,
   `btCompoundShape *`) as public members, so every includer compiles Bullet.
3. **`drawTrackingMap` is a 78% byte-duplicate of `drawUIMap`**
   (gameui.cpp:2813 vs 1679; 287 of 370 lines identical, verified). The
   header comment concedes it "a COPY of the flight window's draw code."
   Every orbit-rendering bug is now a two-place fix.
4. **Two ~100-field god structs**: `GameArgs` (cli.h:13, ~88 fields — display,
   17 log toggles, 9 headless test hooks, proximity, free-cam, all at one
   level) and `Game` (game.h:219, ~143 members across ~20 responsibilities,
   including 15 raw render-resource pointers it doesn't manage).
5. **Raw new/delete ownership chains with no single owner** (vehicle.cpp:102
   `new Part` → dtor 1272; `new Body`/`new btRigidBody` 588–594, `delete hull`
   553; global `physics = new PhysicsEngine` at physics.cpp:149 whose real
   destructor never runs; five different containers own ships). Zero smart
   pointers in the shipping sim path; the registries leak by design.
6. **Physics constants duplicated with no single source**: G = `6.674e-11`
   in 4 places (vehicle.cpp:456/1227/1667, system.cpp:34), friction `4.0` in
   2, `kRcsIsp = 220.0` in 2 (eva.cpp:23 vs vehicle.h:721), SoI hysteresis
   `±10000` in 4, and the physics timestep `1.0/50.0` still carries
   `// TODO explain why 50` (game.h:316).
7. **Type-tag selects the C++ subclass**: `def.parts[0].def->type ==
   "kerbal"` picks `new Kerbal` vs `new Vehicle` (ships.cpp:102) even though
   the schema documents `type` as "free-form label (display only)" and every
   other behavior is field-driven; 9 `static_cast<Kerbal *>` sites are guarded
   by `isEva()` instead. Renaming a catalog part silently changes physics.
8. **GL uniform indices hardcoded at 6+ draw sites** into whatever list
   main.cpp registered (render.cpp:447, body.cpp:20, terrain.cpp:243,
   billboard.cpp:71) — reordering one registration silently re-addresses all
   of them — while the name-based API exists and is ignored.
9. **Bolted-on latch idioms instead of small tables**: 9 headless test hooks
   as repeated `int xxxMs; bool xxxFired` pairs (game.h; fired in main.cpp
   and vab.cpp), 6 per-log throttle timestamps (game.h:335–347), 6
   function-local `static` throttle variables inside `tick`
   (tick.cpp:338–502), 21 copy-pasted `if(slotFired(...))` blocks
   (events.cpp:150–456).
10. **GL error checking is a per-file lottery**: 36 `check_gl_error` calls in
    display.cpp, 28 in postfx.cpp, **0** in render.cpp/terrain/skybox/pick/
    camera; the one check that exists can't fail (`//exit(1);` commented out,
    gldebug.cpp:32) and is compiled out under `GLDEBUG`.

**One-paragraph verdict.** This is a working, well-tested, well-commented
prototype whose architecture is still the main loop's architecture: a handful
of huge functions in a handful of god units, with ownership and constants
agreed in comments rather than in types. Nothing here is exotic — the fixes
are the standard ones (split the five god functions, extract the two map
bodies, one constants table, one ownership rule, name-based GL uniforms) and
the project's own rules ("very early development, don't worry about breaking
changes", "performance is very important", "look for opportunities to
simplify") authorize doing them now, before the fleet scales.

## 1. God units — files, functions, classes

| Unit | Location | Size | What it mixes |
|---|---|---|---|
| `drawUIReadouts` | gameui.cpp:162 | **1321 lines** | ~15 windows (HUD, Settings, Transfer, Porkchop, SurfaceMap, Debug, Orbital, Telemetry, Surface, ShipList, VesselInfo, Controls, Autopilot, Resources) as `drawWin` lambdas sharing a ~60-line alias block |
| `main` | main.cpp:79 | **1003 lines** | CLI parse, SDL/GL/ImGui/ImPlot init, 8 shader creations, PostFX, system load, 5-branch fleet build, VAB, camera, ~80 lines of perf lambdas, the whole frame loop, 6 headless-hook blocks |
| `parse_cli` | cli.cpp:13 | **712 lines** | all flag parsing + validation + defaults in one function |
| `draw3d` | render.cpp:101 | **530 lines** | camera basis (3 modes + 40-line EVA branch), shake, planet/terrain draws, ~100 lines of ship telemetry math, atmosphere/cloud/ocean, two plume passes, planner, key dispatch, ~80 lines of indicators, physics debug |
| `tick` | tick.cpp:18 | **519 lines** | the frame accumulator, per-ship stats, 6 `static` log throttles, substep sizing, gravity/drag/RCS loops, docking, SOI handoff, 8+ `printf` instruments — and it is the file's *only* function |
| `drawUIMap` / `drawTrackingMap` | gameui.cpp:1679/2813 | 417 / 368 lines | see §2.1 — the two are one function copied |
| `drawVabUI` | gameui.cpp:2387 | 360 lines | VAB panel, part list, node/attach UI, symmetry, fuel links |
| `load_system` | system.cpp:14 | 372 lines | parse + build of the whole System (bodies, orbits, terrain, SOI) |
| `load_parts_catalog` | shipdef.cpp:62 | 302 lines | ~15× repeated `value()+throw` field-parsing boilerplate |
| `Vehicle` (class) | vehicle.h | **~110 methods, 1155-line header** | rigid-body lifecycle, COM/invariant math, fuel groups/links/drain, ECS, staging, docking absorb/extract, RCS, aero, gravity, SOI frame switching, rails, scenario spawning, telemetry, drawing |
| `GameArgs` (struct) | cli.h:13 | **~88 fields** | display, sim-input arrays, 17+ log toggles, 9 headless hooks, proximity, free-cam vectors, test-ship flags |
| `Game` (struct) | game.h:219 | **~143 members** | borrowed subsystems, camera + shake, scene stack, VabState, 9 test hooks, clock, toasts, fixed-step, JobRunner, 6 log gates, input flags, ship/kerbal refs, 15 render-resource pointers, draw toggles, key map, ShipView, 5 perf TimeSeries, focus targets, settings, map state, surfmap state |
| `terrain.h` (header) | terrain.h | 560 lines | `TerrainBody` declares patches, 3 shells, ships, pads, 2 frames, a calendar, a shader, a color-func pointer, an `alive` set — and its `Draw`/`DrawOcean`/`DrawAtmosphere`/`DrawClouds` GL code lives inline in the class |
| `terragen.h` (header) | terragen.h | 690 lines | header-only terrain generation incl. `buildGridGeom` (184 lines) and a 2048×1024 FBM-bake lambda |

**The pattern:** the five functions in rows 1–5 are the game. Each one grew
by accretion — every feature touched the nearest big function instead of
getting a home of its own. The fix shape is the same for each: the internal
section banners (`/* ship telemetry */`, `/* indicators */`, ...) already
mark the seams; each section is one function with a small struct argument.

## 2. Duplication — the most expensive ugliness

### 2.1 The two orbital maps are one function copied (HIGH)

`drawTrackingMap` (gameui.cpp:2813) vs `drawUIMap` (gameui.cpp:1679):
**287 of 370 lines byte-identical** (verified by line-set diff). Same
trajectory/apse sampling, plane selection, palette, SOI rings, the
every-body orbit loop with its "bracket the nearest sample" logic, ship
dot/ring, other-ships loop, transfer-conic draw. Diffs are only the size
(`kMapSize` 360 vs `mapW/mapH`), the options setup, and tracking omitting
the controls block. The comment at gameui.cpp:2836 concedes "Diverged from
drawUIMap's opening on purpose" — but the body is shared. Extract one
`drawOrbitMapBody(map, view, focus, {w, h, showControls})`.

### 2.2 Cross-file constant duplication (HIGH)

| Constant | Copies |
|---|---|
| Gravitational G `6.674e-11` | vehicle.cpp:456, 1227, 1667, system.cpp:34 |
| Friction `4.0` | physics.cpp:288, vehicle.cpp:543 (comment: "RegisterObject's value") |
| `kRcsIsp = 220.0` | eva.cpp:23, vehicle.h:721 (vehicle.cpp:1592's comment even points at eva.cpp) |
| SoI hysteresis `±10000` | vehicle.cpp:2392, 2411, 2548, 2563 |
| Terrain band `3000.0` | vehicle.cpp:2460, 2487 |
| Rest height `+0.6` (terrain 0.5 + hull 0.1, tied by comment only) | vehicle.cpp:223, eva.h:78 |
| Resource-name table | gameui.cpp:1462 and 1566 — *already drifting*: "Electric charge" vs "EC" |
| Scenario name list | `kScenarios` (vehicle.cpp:271) + `--scenario` help + `IsMember` (cli.cpp:38) — "add a scenario in ALL THREE places" |
| Slot name/label/group | three ~45-line switches over the same enum (keys.cpp:191/226/259) |
| `asset_key()` | identical 3-line function in mesh.cpp:85, shader.cpp:265, texture.cpp:84 |
| "build ModelView in double, cast to float" | body.cpp:11–16, terrain.cpp:209–213, render.cpp:439/489, terrain.h:339–341/384–385/433–434, billboard.cpp:52–58 |
| Frame→inertial transform (`v = O*v + vrel; p = O*p + prel`) | tick.cpp:429–441, vehicle.cpp:2444, transferplanner.cpp:~28/~49 — four hand-maintained copies; one sign error gives a wrong orbit plot, no compile error |
| Slew braking law | `Kerbal::slewTo` (eva.cpp:166–192) vs `Vehicle::slewToward` — two copies of the same deceleration curve |
| Twin structs `BuildPart`/`ShipPart` | ~15 identical fields + hand-written copies in both directions (shipdef.cpp:115 lines for one direction); the `haveFrom`/`haveTo` fuel-link scan appears 3× (shipdef.cpp ~923/980/1110) |
| SOI frame switching | `switchFrames` (2389) ≈ `railsSwitchFrames` (2546) ≈ `moveToFrame` (2341) ≈ `moveToRailFrame` (2574) — four near-identical functions, a SoI rule change is four edits |
| Menu chrome | `drawMenuWindow` (gameui.cpp:2157) ≈ `drawSpaceCenterMenu` (2258) — same width math, same invisible-color `text_button`, same font push/pop, same VERSION footer; the comment concedes the duplication |
| Engine plume ≈ RCS plume ≈ `DrawModelAt` | render.cpp:407–463 ≈ render.cpp:465–525 ≈ body.cpp:3–57 — ~80% identical, the two plumes even set uniforms by position (0..6) |
| `InitMesh` ×4 overloads | mesh.cpp:147–344 — same GL ritual, 4 near-duplicate vertex structs, `vs`/`is` CPU-copy block triplicated |

### 2.3 Bolted-on latch idioms instead of small tables (MED)

- **9 headless test hooks** as `int xxxMs; bool xxxFired` pairs on `Game`
  (game.h) + 9 near-identical `if(!fired && now>=ms){fired=true; act();}`
  blocks — 5 in main.cpp:870–905 (reload/newGame/quitTitle/spaceCenter/
  tracking) and 4 in vab.cpp:648–688 (place/load/launch/close). One
  `struct Hook{int ms; bool fired; std::string arg; void(*act)(Game&);}` + a
  polled vector replaces all nine.
- **6 log-gate timestamps** on `Game` (game.h:334–352), each annotated
  "Same gate, independent clock" — one `std::array<LogGate, N>` replaces
  them.
- **6 function-local `static` throttles inside `tick`** (tick.cpp:338, 349,
  360, 371, 382, 502) mixing sim time and `SDL_GetTicks()` — state hidden in
  a function body, untestable, and a reason `tick` can't be split without
  thinking.
- **21 `if(slotFired(...))` blocks** in `flightKeyActions`
  (events.cpp:150–456) — a `Slot → handler` table would be the same code
  with one edit point; meanwhile `vabKeyActions`/`hubKeyActions` bypass the
  slot table entirely with hardcoded `SDL_SCANCODE_*` (events.cpp:359, 388),
  so editor keys aren't rebindable.

## 3. Ownership and resource management

**The rule the code currently runs on: nothing is owned, everything is
documented in comments.**

- **Three-level raw chain in the sim core**: `Vehicle` owns `Part`s
  (`new Part` vehicle.cpp:102, dtor 1270–1277), `Part` owns `Body` (~Part,
  part.h:67), `Body` owns `btRigidBody`/`btCompoundShape` (vehicle.cpp:558,
  580, 588, 594; `delete hull` 553). Ships are owned by **five different
  containers** (`TerrainBody::ships`, `pads`, crew lists) — no smart
  pointers in this path. `spawn_crew_kerbal` even reads `kb->mass` *after*
  `RemoveBody(kb)` (ships.cpp:191/194), which only works because `Body`
  outlives its `btBody` — an invariant held by comment.
- **The physics engine is a global with a dead destructor**: `physics =
  new PhysicsEngine` (physics.cpp:149), class has a real dtor (180–184)
  that never runs — "leak at exit, pretend it's RAII". `RemoveTerrainCollision`
  (physics.cpp:215–224) manually deletes interface+shape+motionState after a
  `static_cast<btTriangleMeshShape *>`, ownership in a comment.
- **GL registries leak by design**: `get_mesh`/`placeholder_box` (mesh.cpp:132,
  94) never free; the comment at mesh.cpp:78–82 claims "the GL context
  teardown reclaims the objects" — the static maps are never cleared and
  teardown only reclaims GPU memory, not the dangling pointers.
  `Renderer::~Renderer()` is empty (display.cpp:200–202) — window and
  context never destroyed. `Texture::id` has no default (texture.h:8), so a
  failed construction path deletes a garbage handle.
- **Three allocation styles in one class**: `Mesh::vs`/`is` are `new[]`,
  `m_vertexArrayBuffers` is `malloc` (mesh.h:103; the mallocs in
  mesh.cpp:157/206/260/319), `Billboard` owns its quad via `delete mesh`
  (billboard.cpp:37).
- **Skybox doesn't own its resources**: file-scope `GLuint` globals
  (skybox.cpp:13–14) deleted by `~Skybox` — a second instance clobbers the
  first; `loadCubemap` dereferences a possibly-null `IMG_Load` result
  (skybox.cpp:32).
- **Unbounded, never-evicted caches**: `static std::map<const void*,
  OrbitSampleCache> orbit_caches` (gameui.cpp:72) and
  `std::map<const PartDef*, VabAsset> g_vabAssets` (vab.cpp:30, holding
  `new btConvexHullShape`/`btCollisionObject` that are never freed) — a
  slow leak over a long session.

**Fix shape:** one ownership rule with smart pointers — `Vehicle` owns
`Part`s, `Part` owns `Body`, lists hold `unique_ptr`; the engine is an owned
member (or passed struct); registries get explicit `shutdown()`. Early in
development this is cheap now and gets exponentially more expensive after
docking/crew/EVA multiply the allocation sites.

## 4. Constants, magic numbers, and hidden knobs

Beyond the duplicated constants in §2.2:

- **`const double dt = 1.0/50.0; // TODO explain why 50`** (game.h:316) —
  the stability-critical physics timestep, read by the whole tick path,
  unexplained. One line of rationale or a named `kPhysicsHz` fixes it.
- **`GameArgs` carries 17+ log toggles and 9 test hooks** alongside display
  and proximity settings — adding a flag is a coin-flip about where it
  belongs (§1).
- **A hidden test hook in shipped code**: `hull_margin()` reads
  `getenv("OSP_HULL_MARGIN")` in `BuildHull` (physics.cpp:38).
- **Debug prints in the normal path**: `switchFrames`/`railsSwitchFrames`
  emit `@@@ ... switching frame` printf on every SoI crossing
  (vehicle.cpp:2392 et al.); 223 `printf`/`fprintf` calls across src/ with
  no logging layer and only 7 `assert`s — the two error-reporting channels
  of the codebase are a console and a coin flip.
- **Magic-number soup in rendering** (all unnamed): detail tint
  `vec4(0.8,0.8,0.8,1)` (terrain.h:499), shadow floor `0.15f` + 100 m step
  (terrain.cpp:333/339), billboard distance `10.0` (billboard.cpp:71),
  `glLineWidth(4)` (render.cpp:610), RCS `sx=0.5, sz=0.75` (render.cpp:483),
  VAB studio light `vec3(0.4,0.8,0.35)` (render.cpp:657), zoom `exp(-amt*0.25)`
  and pole clamp `1.52` (camera.cpp:164–191), grid `size = 49` and
  `radius + 3000` palette band (terragen.h:517, 353), ray length `1e7` and
  `bestDist = 1e300` (pick.cpp:65, 120), hot pink `255,105,180`
  (texture.cpp:104) and a lighter pink `255,192,203` (mesh.h:60), and
  main.cpp:419's
  `// TODO should these be different colors?` next to a `vec4(1,1,1,1)`
  reused by ~10 `mk_billboard` calls.
- **Scattered `static const` config with no single home**: `kRailsWarp`,
  `kToastLife/Visible`, `kPickClickPx/Ms`, `kDockCapture/Align/MaxV`
  (game.h:53–74), `kSettingsFile` (settings.h), `margin/spacing/max_wait`
  as mutable `Manager` fields (ui.h).

## 5. Leaky abstractions and API shape

- **Bullet leaks through the public API of the game model**: `body.h`
  defines `BT_USE_DOUBLE_PRECISION` and includes all of Bullet, so every
  includer of `vehicle.h` (i.e. nearly everything) compiles it; `Vehicle`
  exposes `btTransform principal`, `btCompoundShape *compoundShape()`,
  `static btTransform toBt(...)` as public members. `RegisterObject` takes
  `glm::vec3` (float) pos/rot in a double world where `rot` is actually an
  Euler triple (`btQuaternion euler_rot(rot.x, rot.y, rot.z)`,
  physics.cpp:262–264) — the signature lies about its units.
- **`Kerbal::rcsDir` shadows `Vehicle::rcsDir`** (eva.h:47 vs vehicle.h:703)
  — base-class code touching `rcsDir` silently means the base member.
- **Type-tag polymorphism** (§Executive 7): `ships.cpp:102` reads
  `def.parts[0].def->type == "kerbal"` to pick the subclass. The catalog
  documents `type` as display-only; every other behavior (thruster/wheel/
  RCS/capsule) is field-driven. One polymorphic decision bypasses the
  system, and its 9 `static_cast<Kerbal *>` consumers (tick.cpp:515,
  eva.cpp:34, save.cpp:90/485, ships.cpp:201, game.cpp:444/451/461,
  events.cpp:271) test `isEva()` instead of the tag — so the cast is safe
  only while `Kerbal` ⇔ `isEva()` stay in sync, by comment.
- **`XferTarget.name` is a `const char *` dangling into a per-frame
  `std::string`** (transferplanner.h:22–26) — the lifetime contract is
  implicit; the planner also carries ~25 public UI-state members
  (`pcCustomDep`, `pcDepLo`, …) that belong to the UI layer.
- **Inconsistent null-guarding in one class**: `getThrust`/`GetActiveThrust`
  guard `m_parent`, but `getTWR`/`getFullThrustTWR`/`getMaxTWR` deref it
  unguarded (vehicle.cpp) — three of five thrust/TWR functions crash on a
  parentless ship where the other two return zero.
- **`TransferPlanner::transfer_semi_major_of`** (transfer.h:51) returns `0.0`
  with the comment "retain a stub here to prevent linker errors if used
  elsewhere" — zero callers; a dead function whose only purpose is to keep
  itself alive. Same disease: `print_mat` (body.cpp:79), commented-out
  `angleFacing` (physics.cpp:471–475), `Camera::GetView_()` (camera.cpp:55,
  defined, called nowhere), `Billboard::frame` (billboard.h:14, never
  set or read), the whole v1 drag API (drag.h's `dragForce`/`dragForceAOA`/
  `offAxisFactor`/`dynamicPressure` — called only by test_drag.cpp).
- **Naming**: `Vehicle` mixes three casing conventions (`getMass` /
  `GetVel` / `setVelocity` / `SetFriction` / `getDeltaV` / `GetActiveThrust`)
  and five overlapping thrust/TWR functions; physics free-functions mix
  `addTerrainCollision`/`AddPhysicsBody`/`physics_tick`/`NeverSleep`/
  `ApplyForce`/`GetVelocity`/`setPosRot`; `Vehicle::attachSurface` must call
  the free `::attachSurface` (vehicle.cpp:829–831) with a comment explaining
  why; `struct Camera;` forward-declares a `class` (skybox.h:5).
- **Idioms**: ~500 C-style casts across src/ (vs `static_cast` being
  available), `and`/`or` operators mixed with `&&` within the same function
  (render.cpp:327), `if(x == true)` ×6 (render.cpp:251–395), `== nullptr`
  and `!= NULL` mixed in one file (terrain.cpp:25 vs terrain.cpp:79). 42
  `NULL` vs 414 `nullptr` — mostly consistent, a few stragglers.
- **`#define GLM_ENABLE_EXPERIMENTAL` in a public header** (camera.h:3)
  pollutes every TU; render.cpp:29 re-defines it mid-file after its
  includes.
- **`ui::Manager` is a Meyers singleton** (ui.h:96: `static Manager& Get()`),
  plus `inline bool& ResetFlag() { static bool f; }` (ui.h:43) — the whole
  window open/layout state in a hidden global; hidden `static` state also
  lurks inside draw functions: `static float dpi_pending`,
  `static Texture* pc_tex/bar_tex/sm_tex`, `static char nameBuf/savePath`,
  `static int selected/loadSel`, `static std::vector ships`,
  `static time_t shipDirMtime` (gameui.cpp:180, 748–759, 885–886,
  2310–2311, 2420–2442) — unresettable, untestable, and the recent
  uiwins.h refactor explicitly moved *away* from this pattern while these
  remain.
- **UI reaches into sim internals**: `ShipView` exists to be "the per-frame
  state the readouts show," yet the HUD reads `ship->m_parent->
  GetTerrainHeight(...)` and `ship->frame->isRotFrame()` (gameui.cpp:227)
  and the part window reads `partBody->mass`, `def->fullThrust()`,
  `def->fuel_rate`, `def->resources.current[r]` directly (gameui.cpp:
  1560–1600).
- **GL handles cross module boundaries**: raw `GLuint` globals in skybox,
  `m_program` read publicly by skybox.cpp:78 (shader.h:53), `camera.h`
  exposes `view`/`projection`/`pos`/`forward`/`up` public and pick.cpp:38–56
  reads them directly.
- **`#pragma GCC diagnostic ignored "-Wpedantic"`** (uiwins.cpp:29–30)
  suppresses a warning to use GCC/Clang designated-array initializers for
  the window table — a real goal (drift-proof enum↔table) paid for with
  non-ISO syntax.
- **`Slot::Space` does two jobs** (events.cpp:270–290): arms the EVA jump
  *and* stages, scene-dependent — one rebindable control, two meanings.
- **`fmt_time`** (a general UI formatter) lives in siminput.cpp:82, the
  synthetic-input module.

## 6. Performance traps (the project says this matters a lot)

- **`tick()` re-snapshots the fleet every substep**: `all =
  collectVehicles(g.sys)` (tick.cpp:209) inside the `while (accumulator >=
  dt)` loop — a fresh vector allocation up to 2000×/frame. It's documented
  (dangling-pointer avoidance after docking deletes), but the first substep
  could fill a persistent vector and later substeps refresh in place.
- **`collectVehicles` is O(fleet) and is called per-frame from UI** (ships.cpp:27,
  allocates a new vector walking all bodies×ships×crew): in
  `drawTrackingShipList`, the ShipList window, 3–4× inside
  `select_ship`/`remove_ship`, and **once per decoupler** in `stage()`'s
  dedup — O(fleet²) on a heavy staging event.
- **`getMass()` is O(parts) every tick with a known TODO**:
  `/* TODO should be cached per frame */` (vehicle.h:521);
  `refreshCompound` (called every tick) and `getDeltaV` (twice per frame)
  both invoke it.
- **Fuel drain rebuilds a `std::map` per engine per substep**
  (`fuelDrainLayers`/`consumeResourceMass`, vehicle.cpp:~52-line scans) —
  N engines × substeps map allocations per tick at high time-warp.
- **`checkCompoundInvariants` runs on every compound rebuild**
  (vehicle.cpp:641, called at 609; 86-line O(parts) matrix re-assembly + compare) and
  rebuilds happen on every fuel-burn threshold crossing — a full
  re-derivation inside the sim loop whenever things are healthy.
- **Per-frame string building in UI**: the Settings window rebuilds its
  `items` string from `displayModes()` every frame (gameui.cpp:313–323);
  `fmt_cal_time`/`fmt_time` allocate fresh strings per call per frame.
- **Zoom-out mass-deletes the whole patch subtree in one frame**
  (terrain.cpp:257–268) — a hitch that scales with patch count; the
  continuation also recomputes `subdivideCorners` (terrain.cpp:143)
  after the worker already computed it (terrain.cpp:124).
- **`pickShipPart` is O(ships × parts) Bullet raycasts** (pick.cpp:120–145)
  — fine for one ship, will hurt with the fleet the project plans to scale.
- **PostFX resolves uniform names with linear `strcmp` scans every frame**
  (postfx.cpp:311–373 → shader.cpp:169–176) — the name→index map is fixed at
  `AddEffect` time and could be resolved once.
- **`draw3d` does non-drawing work**: `planner.update` and one-shot key-flag
  consumption inside the draw function (render.cpp:517–533).
- **`Renderer` ctor is ~185 lines** (display.cpp:15) and uses
  `assert(m_window)` (display.cpp:97) as the failure path — vanishes under
  NDEBUG → null-deref in release; `glewInit` failure prints and continues
  (display.cpp:124–130).
- **`Mesh::InitMesh` ×4 overloads** (mesh.cpp:147–344) — the first is the
  only one with `check_gl_error` calls; the `vs`/`is` CPU copies are
  `new double[n*3]` + `new int[n]` + `memcpy`, triplicated.
- **`Billboard::Draw` round-trips the view rotation through float32**
  (billboard.cpp:55–58: `dmat4 → mat3 → transpose → back`) and draw3d calls
  it 8–12×/frame with the same shader — state churn for a rotation-only use.
- **`shader.cpp` caps uniforms at 16** (`MAX_NUM_UNIFORMS`, shader.cpp:46–51)
  with a stderr note — a 17th uniform fails at runtime, not compile time.
- **`Renderer::onResize` prints on every resize event** (display.cpp:204) —
  a window drag floods the console; `SaveScreenshot` uses
  `new unsigned char[w*h*4]` (display.cpp:356–361) where a vector would do.

## 7. Error handling and dead code

- **GL error checking is a per-file lottery** (counts of
  `check_gl_error`): display.cpp 36, postfx.cpp 28, shader.cpp 14,
  physics.cpp 13, mesh.cpp 12 (first `InitMesh` overload only),
  main.cpp 10, events.cpp 4 — and **0** in render.cpp, texture.cpp,
  skybox.cpp, terrain.cpp/h, billboard.cpp, pick.cpp, camera.cpp,
  render's two plume passes, and every terrain draw. The check itself
  prints and continues (`//exit(1);` commented out, gldebug.cpp:32) and
  gldebug.h:3–7 compiles it out entirely under `GLDEBUG`. Pick one
  policy and apply it.
- **The shader registry caches failure and draws nothing forever**:
  `get_shader` stores the program even when link fails ("its draws will be
  invisible", shader.cpp:296–303); `LoadShader` returns an empty string on a
  missing file; `FromFile` is `void`. Mesh and texture both have visible
  placeholders; a broken shader is a silent permanent black-out.
- **Stale comment contradicts live behavior**: shipdef.cpp:534
  "stage: reserved for staging … no runtime effect yet" — staging is fully
  live (fuel groups, `droppedPartsAtStage`, decouplers). A comment asserting
  "no runtime effect" on a live feature invites someone to "clean up" the
  field.
- **Dead code inventory** (all verified): `Camera::GetView_()`
  (camera.cpp:55); `Billboard::frame` (billboard.h:14); commented-out
  `ImGui::Text` debug block (render.cpp:370–379); stranded
  `// surf pos??` (render.cpp:305); commented-out `horizon_indicator`
  (render.cpp:606–607); `transfer_semi_major_of` stub (transfer.h:51);
  `print_mat` (body.cpp:79); commented-out `angleFacing` (physics.cpp:
  471–475); v1 drag API pinned only by its test (drag.h); `mesh.cpp:29`
  prints a bool as a count (`printf("scene meshes: %d", HasMeshes())`);
  shader.cpp:20 and terrain.cpp:121/252 log per-load/per-LOD noise (the
  last two fire on every zoom).
- **`TODO` inventory** (8 in src/): physics.cpp:69, 71, 471;
  game.h:316 (`1.0/50.0`); gameui.cpp:429 (dpi re-fit); vehicle.h:521
  (cache `getMass`); main.cpp:419 (billboard colors); system.h:31 (are the
  docs needed?). Separate from these: one stale non-TODO comment
  (shipdef.cpp:534, above) and two historical "Outerra logZ" references
  (camera.cpp:6, display.cpp:161) that are fine to leave.
- **`Skybox` pastes one 2D image on all 6 cubemap faces** (skybox.cpp:92–98)
  and never unbinds the cubemap (every other pass does); it looks up the
  `"skybox"` uniform location **every frame** (skybox.cpp:78) instead of
  registering it.

## 8. Persistence, tests, and build

This is the *least* ugly area — the save path and the Makefile are recent,
deliberate work, and most of what's here is minor. Noted for completeness.

**What's good (so it isn't "fixed"):** `save.h`/`save.cpp` separate the pure,
headless-testable JSON (de)serialization from the `Game`-coupled
capture/restore, and `load_game` is genuinely transactional — it detaches the
live fleet, builds the new one, and only commits on success, with a
carefully-reasoned rollback that avoids the double-free of an aboard kerbal
(save.cpp:341–430, heavily commented). The Makefile uses pattern rules
("three pattern rules replace the eight per-file ones" — Makefile:188–189)
and builds the ~32 test binaries in parallel.

**The real findings, minor:**

- **The save-format version field is decorative** (save.h:128 `int format =
  1;`): it is written to `save.json`, read back, and round-tripped by
  `tests/test_save.cpp` — but `load_game` never *checks* it. A future
  "format 2" would load through the format-1 path and either silently
  misparse or throw a confusing error. The code even concedes it
  (save.cpp:382: "the parts catalog moves and nothing here is versioned").
  One `if(meta.format != 1) throw ...` would make the field honest.
- **24 test files each `#define` their own `CHECK` macro** (every
  `tests/test_*.cpp` except the `.c` ones) — there is no shared test-harness
  header, so the assertion idiom is re-invented 24 times with small
  differences (`CHECK` vs `CHECK_TRUE` vs `CHECK_NEAR`). A single
  `tests/check.h` would unify it and make the `.c` tests use it too.
- **`struct Ship` is duplicated in 5 test files** (test_dock.cpp:68,
  test_fuel.cpp:72, test_inertia.cpp:108, test_power.cpp:71,
  test_staging.cpp:46) — each defines its own miniature ship fixture for the
  fuel/power/inertia math. A shared `tests/ship_fixture.h` would dedupe it.
- **7 `.c` test files are compiled as C++** (`-std=c++20` on `tests/*.c`,
  e.g. Makefile:509 `test_vertexless.c`) — a language-mixing shortcut; they
  should be `.cpp` or the rule should be `-std=c11`.
- **The `test:` target keeps two hand-synced lists**: the `TESTS` variable
  (Makefile:449) and the run-recipe `./test_frames ./test_spawn …` lines
  (Makefile:460+) are separate; adding a test means editing both or the new
  one builds but never runs (or runs but never builds).
- **`system.h:31` has a self-questioning TODO** (`// TODO are these detailed
  docs needed?`) sitting directly above the very detailed docs it questions —
  either the answer is "yes, keep them" (drop the TODO) or "no" (trim them).

## 9. Recommended fixes, in priority order

The ordering criterion: *risk × leverage* — where a small change removes the
most future cost, first. All of these are safe under the project's "no
backward compatibility, very early development" rule.

1. **Extract the shared orbital-map body** (§2.1). One function, two thin
   callers. Removes the single most likely source of a fixed-twice bug.
2. **One constants header** (or `System` fields) for G, friction, `kRcsIsp`,
   SoI hysteresis, terrain band, rest offset, `kPhysicsHz` + rationale
   (§2.2, §4). Ten minutes, kills a whole bug class.
3. **Split the five god functions** (§1) into their banner-marked sections,
   one file at a time, `tick` first (it's the simulation spine and its 6
   `static` throttles become members of a `TickState` struct as a side
   effect). Each split is mechanically reviewable — no behavior change,
   just function extraction.
4. **Replace the latch idioms with tables** (§2.3): 9 test hooks → `Hook`
   vector; 6 log gates → `LogGate` array; 21 key blocks → dispatch table.
   Each is ~30 lines of code *removed*.
5. **One ownership rule + smart pointers in the sim core** (§3): `Vehicle`
   owns `Part`s (`unique_ptr`), `Part` owns `Body`, ship lists hold
   `unique_ptr`; engine becomes an owned member. Do it before docking/crew
   multiply the allocation sites.
6. **Name-based GL uniforms everywhere; one plume/model helper; one GL
   error policy** (§2.2, §5, §7): the name API already exists
   (shader.h:57–60); `DrawModelAt` already exists (body.cpp:3–57).
7. **Derive `Kerbal` from data, not the `type` tag** (§5): a documented
   `kind` field (or `capsule` presence) + one `static_cast` choke point,
   replacing 9 scattered `isEva()`-guarded casts.
8. **Confine Bullet behind `Body`** (§5): `vehicle.h` stops exposing
   `btTransform`/`btCompoundShape *`; `RegisterObject` takes a real pose.
   Compile times drop across the board (Bullet is heavy).
9. **Split `GameArgs` and `Game`** (§1): `DisplayOpts`/`LogToggles`/
   `TestHooks`/`Proximity`; `InputState`/`RenderResources`/`MapState` nested
   structs. Mechanical, but do it *after* the latch tables (step 4) so the
   structs are small when they move.
10. **Delete the dead code** (§7) in one pass — every item is verified to
    have no callers.

## 10. Repo hygiene (small, but real)

- **`obj_asan_stale/` is not in `.gitignore`** — a stale build tree sitting
  next to the four live ones (`obj`, `obj_test`, `obj_asan`, `obj_tsan`);
  git status shows it as untracked noise.
- **Two scratch files live at repo root, untracked**: `SCENE_API.txt`
  (23 KB) and `commit_review_findings.txt` (16 KB). They are notes, not
  code — but root-level untracked files are the kind of thing that gets
  `git add -A`'d by accident. `tmp/` (992 items) is correctly ignored;
  these two should be too (or moved into `tmp/`).
- **`saves/` is not in `.gitignore`** (currently empty, so harmless — until
  the first real save appears at repo root).
- **32 `test_*` binaries + 3 game binaries sit at repo root** — all
  correctly gitignored, but the Makefile builds into the working tree
  rather than a single `build/` prefix; five separate `obj*` directories
  for sanitizer variants is the price paid, and `obj_asan_stale` is the
  scar left by a CXXFLAGS change (Makefile:571 even documents it).

## Appendix A — mechanical scan results (2026-09-18)

| Signal | Value |
|---|---|
| Total src lines (.cpp + .h) | 28,303 across 88 files (39 .cpp + 49 .h) |
| Largest file | gameui.cpp (3182) |
| Longest function | drawUIReadouts, 1321 lines (gameui.cpp:162) |
| Functions > 300 lines | 10 (all listed in §1 table) |
| `Game` struct fields | ~143 (game.h:219–648) |
| `GameArgs` fields | 89 (cli.h:13–166, incl. the nested `cli_given` struct) |
| `Vehicle` declared methods | ~110 (vehicle.h) |
| C-style casts (rough count) | ~500 |
| `and`/`or` as C++ operators | ~7 code lines (frame.cpp:43, render.cpp:327, terrain.cpp:254–255, vehicle.cpp:2237, +2); the thousands of raw "and"/"or" matches are English in comments/strings |
| `printf`/`fprintf` in src/ | 223 |
| `assert` in src/ | 7 |
| `nullptr` vs `NULL` | 414 vs 42 |
| `#pragma once` vs include guards | 43 vs 4 headers |
| `#if 0` blocks | 0 |
| `new`/`delete` in src/ | pervasive in sim core + GL layer; only 5 smart pointers (`unique_ptr<Shader>` postfx.h:93; `shared_ptr` handoffs terrain.h:305, terrain.cpp:125, transferplanner.cpp:269, surfmap.cpp:134) |
| TODOs in src/ | 8 (all listed in §7) |
| C++ standard / compiler | C++20, g++, `-Wall -Wextra -Wpedantic` + 3 suppressions; no clang-format/clang-tidy |

## Appendix B — what is *not* ugly

Recorded so the audit isn't read as a complaint:

- **Comments are excellent.** Nearly every non-obvious decision (the
  Reverse-Z choice, the compound-shape invariants, the fuel-link ownership
  rules, the SoI hysteresis band) carries a "why" comment. The ugliness is
  structural, not neglectful.
- **The asset layer (mesh/shader/texture registries)** is the best part:
  shared, cached, placeholder-on-failure, asset-keyed.
- **Recent refactors are landing in the right direction**: the scene stack
  (`scene.h`), the window table (`uiwins.h`), the transactional
  `load_game` — each removed a whole class of the smells above.
- **Testing is broad and the e2e harness is real**: 39 test sources
  (32 .cpp + 7 .c), 66 e2e cases with synthetic input, ASan/TSan builds
  in the Makefile.
- **No `#if 0` rot, no commented-out feature branches, no vendored
  copy-pasted UI code.** The codebase is clean in the ways that are cheap
  to check and ugly in the ways that are expensive — which is exactly where
  to spend the fixes.
