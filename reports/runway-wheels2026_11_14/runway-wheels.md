# Runway, wheels, and horizontal spawns — staged plan

Status: **PLAN — no game code written.** Every code reference below was verified
against master `cb9285e8` (2026-11-14), and the whole plan went through an
adversarial review pass (findings folded in: margin-aware clearance, the real
flag-consumer list, the test_buildings pins, EVA-on-runway, brake-load gating).
The runway siting numbers were measured, not guessed: the probe is kept at
`tmp/runway_probe.cpp` (re-runnable, pure `terragen.h` math, no game state). One
bug found on the way is filed as **#846** and fixed by stage 1.

The goal, in one sentence: **a plane built in the VAB can launch onto a runway,
taxi, take off, land, and brake — using the same one-body Vehicle it already
flies with.** Everything else (rovers, retracting gear, wheel spin animation) is
explicitly out of scope, with hooks noted where DESIGN.md asks for them later.

---

## 1. Where we are (what the plan builds on)

- **Planes fly but never start on the ground.** Starts are debug air beds
  (`flying-low/slow/cruise` in `kScenarios`, vehicle.cpp:231) or standing on the
  tail on a pad. DESIGN.md's first future-work item is exactly this gap.
- **Horizontal pad spawn is already 80% built.** `ShipDef.attitude {pitch, yaw}`
  (#461/#460, shipdef.h:586-599, parsed shipdef.cpp:862-898, pitch 0..180) tilts
  the nose off the site vertical with wings level. `e2e/vehicles/level_jet.json`
  (pitch 90) spawns horizontal on the pad TODAY; `flight/461` pins it. What is
  missing: a flat deck, wheels, and letting the **site** decide the pose instead
  of every def authoring its own tilt.
- **One body, forces at offsets.** A Vehicle is one `btRigidBody` + one
  `btCompoundShape` of part hulls (vehicle.h:225-260, rebuildCompound
  vehicle.cpp:1422). Thrust, RCS, aero and the EVA walk law all apply forces to
  that one body at part offsets. Parts have no rigid bodies of their own.
- **Buildings are data.** Catalog `res/data/buildings.json` + instances in the
  system JSON's `world_buildings` (Kerbin's live in the fragment
  `res/data/ksp_launch_sites.json`) → `StaticBuilding`, a static convex hull at
  lat/lon/height/heading (buildings.cpp:283-307). The flag `VAB_LAUNCH_PAD` is
  read in more places than it first appears: `find_launch_site`
  (buildings.cpp:310), the VAB site picker (gameui.cpp:6125) and `vabLaunch`
  (vab.cpp:720), the surface-map site marks (gameui.cpp:2163) and site count
  (surfmap.cpp:111), the save-load preflight `systemHasCrewPad`
  (save.cpp:1393), the startship site-error listing (vehicle.cpp:1074), and —
  the load-bearing one — **`homeCrewPad`** (vehicle.cpp:1183), which feeds the
  crew pool (`mintCrewPool`, `takeCrew`, `returnCrew`). **A runway that carries
  this flag inherits all of them for free, and what keeps them safe is that the
  DEFAULT site stays `main_space_port`** (first VAB_LAUNCH_PAD in fragment
  order): the crew pool, recovery and the save preflight all resolve the
  default, not the runway.
- **Grounded/parking already works.** `isGrounded()` (vehicle.cpp:6780) =
  speed < `kShipGroundSpeed` (3 m/s) in the rotating frame AND COM within
  `kShipGroundBand` (100 m) of the analytic terrain (constants.h:170-171). A
  parked ship freezes on rails; a warp into the ground parks it. A plane resting
  on a runway inherits all of this unchanged (the runway top is ~1 m above the
  terrain, well inside the 100 m band; a rolling plane > 3 m/s correctly refuses
  rails).
- **EVA is the precedent for a ground law.** `Kerbal : Vehicle` overrides the
  per-substep `applyControlForces` (eva.h), and grounds on "Bullet contact OR
  analytic terrain height" because collision meshes only exist at max-LOD
  leaves. The wheel pass copies that two-source grounding pattern.
- **The substep loop has a clean hook.** tick.cpp:315-339, per non-railed ship:
  `beginForceRecordSubstep → processGravity → powerTick → applyControlForces →
  applyAeroForce → physics_tick(h)`. Forces are re-applied every substep; Bullet
  clears them on exit. A wheel pass slots in beside aero.

## 2. The open question: can btCompoundShape / Vehicle accommodate wheels?

**Yes — without changing the one-body architecture.** A wheel is a *force
applier*, exactly like a thruster or a lifting surface: each substep, find its
contact, push on the hull at that offset. No per-part bodies, no joints, no
 Bullet vehicle class.

What this keeps working for free:

- **Staging / docking / undocking** — a wheel is an ordinary Part; the compound
  rebuilds as it always does. A dropped booster takes its wheels with it.
- **Save/load** — v1 adds **no persistent state** (fixed gear; brakes and steer
  are transient input state like throttle). Wheels save as ordinary parts.
- **Mass properties** — the principal-axis solve already weighs every part.
- **Aero** — the wheel hull joins the silhouette like any small part; its drag
  is honestly tiny. No special case.

The one physical subtlety: **the wheel's collision hull stays in the compound
and acts as a bump stop — and the clearance arithmetic MUST include collision
margins.** The game's margin law (physics.cpp:23-27): a hull and what it rests
on each carry one and the two SUM to the visible gap; defaults are 0.1 m hull +
0.1 m building = **0.2 m** (physics.h:52-60). Two consequences for the wheel:

- A convex `rayTest` hits the **margin-inflated** deck, i.e. 0.1 m above the
  visual runway top. The spring's "ground" is that inflated surface.
- The wheel hull is inflated by its own margin too. With axle height
  `d = radius + travel − compression` above the ray hit, the inflated hull
  bottom clears the inflated deck only when **`travel − compression >
  hull_margin + support_margin + slop`** — with defaults, > 0.25 m. The naive
  "static compression = half of 0.3 m travel" leaves 0.15 m and guarantees the
  friction-4.0 scrub this plan warns about. Either size travel ≈ 0.35-0.4 m
  with ~0.1 m static compression, or author a small `hull_margin` on the gear
  def (PartDef already has the field, shipdef.h) to shrink the envelope. The
  clearance unit test (§7) must pin this WITH the margins in the arithmetic.

Analytical tire forces carry the ship; the hull only touches when the
suspension bottoms out — a crash/scrape, where Bullet contact is the right
answer. Note `hullFriction` (4.0) is body-wide: this vendored Bullet has no
per-child friction (`btCompoundShape.h` has no `setChildFriction`), so a
bottomed-out wheel scrubs hard. Acceptable for v1; the analytical forces provide
the real grip.

**Deliberately not used:**

- `btRaycastVehicle` — it is a `btActionInterface` updated inside
  `stepSimulation`, so it would apply wheel forces outside the game's force
  appliers (no F4 recording, no force-overlay boundary), is chassis-centric
  rather than hull-agnostic, and cannot be unit-tested as pure math the way
  gimbal.h/rcs.h/drag.h are. The decision is about fitting the project's law
  structure, not about Bullet's capability.
- Per-wheel `btRigidBody` + constraints — breaks the one-body invariant the
  whole Vehicle class (and its tests) are built on.
- Terrain flattening under the runway — a terragen change with render/collision/
  map fallout. The survey (§4) found a site flat enough for a thick slab; no
  terrain code moves. (Flattening becomes a pre-requisite for *player-placed*
  buildings later — noted in §6.)

**Stability caveat (the one real risk).** The spring forces are explicit.
Substeps are ≤ `kMaxSubStep` = 0.1 s but never fewer than 3 per tick
(timestep.h): h ≈ 5.6 ms at 1x, up to ≈ 56 ms at 10x warp. Size the springs soft
(static compression ≈ half travel, damping ratio ~0.5-0.7) and **soak at 1x and
10x**; if a parked plane rings, soften the spring or grow the substep count for
hulls with loaded wheels. Do not ship a spring that is only stable at 1x.

## 3. Design decisions (the short list)

| Question | Decision |
|---|---|
| Runway | New building type: a long box slab, **900 × 30 × 4 m**, top at local z=0 (same origin convention as `space_port`, whose mesh top is z=0). Static convex hull, like every building. |
| Runway flags | `["VAB_LAUNCH_PAD", "RUNWAY"]`. The first flag makes picker/map/find_launch_site/recovery work unchanged; `RUNWAY` is what selects the horizontal pose. Default site stays `main_space_port` (fragment order, buildings.h:70-75). |
| Spawn pose | **Site-driven**, as suggested: a `RUNWAY` site stands any hull nose-along-runway, wings level. Pad sites keep today's def-attitude behaviour. Def `attitude` is ignored on a runway (document it). |
| Wheel part | New flat PartDef fields (the field-driven convention, part.h:201-217): `wheel_radius` > 0 → a gear wheel. **Name the predicate `isGearWheel()` — `isWheel()` already means reaction wheel** (part.h:209). Spring/damp/travel as def fields is a deliberate choice (the convention; `gen_parts.py` can derive them from a load rating) even though v1 ships one gear size — constants in wheel.h would also be defensible. |
| Ground law | New `Vehicle::applyGroundForces(h)` beside aero in the substep loop; a cached wheel list (rebuilt on geometry change, like `aeroSystems`) makes it a no-op for wheel-less hulls. Pure law in a new `src/wheel.h` (the gimbal.h/rcs.h/drag.h pattern) + `tests/test_wheels.cpp`. |
| Contact | Per-wheel Bullet `rayTest` down from the axle (hits terrain mesh, runway, other hulls) with an **analytic terrain fall-through** when the ray misses (EVA precedent). Wrap the ray in physics.h/cpp so vehicle code stays Bullet-free. |
| Tires | Slip-based, clamped by the friction circle: lateral grip + rolling resistance + brake force, applied at the contact point. Steering turns the nose wheel's rolling direction. **No drive force in v1** — the channel is the ground-vehicle hook (§6). |
| Brakes | New `Slot::Brakes`, default **B**, latching, **ON at spawn** so a plane holds still; on LOAD, default on only when the ship is near-static (a save mid-takeoff-roll must not load with locked brakes). HUD indicator. |
| Steering | Existing yaw keys (A/D) steer the nose wheel while any wheel is loaded, authority fading to zero by ~30-40 m/s so it can't flip the plane at speed. No new keys. |
| Gear retraction / wheel spin visuals | **Out of scope** (DESIGN.md: animations are future work). Fixed gear; spin can be render-only from ground speed later. |

Part-frame convention for the gear mesh (fix it once, author to it):
**axle along local X, rolling direction local +Y, strut up local +Z**, origin at
the axle centre, surface node at the strut top. Ship defs aim local +Y along the
ship nose with the usual surface-attach `roll`/`angle`.

## 4. Runway siting — measured, not guessed

The KSC pad itself sits on a slope: profiles through it drop ~2.4% to the east
and ~5.9% to the north (probe output). A 900 m runway there would need a ~20 m
slab. But the terrain is *smooth* at runway scale — deviation from a best-fit
plane is ~0.04 m over 900 m — so the slab only has to absorb the local relief,
and a flat-enough site exists nearby.

Survey: centres ±4 km around KSC on a 200 m grid, azimuths 0..175° step 5°,
footprint 900 × 30 m sampled every 25 m on three lines (centre ±15 m), full-detail
`terrainHeight` (the same function physics/spawning query; at Kerbin's max LOD
all 9 octaves are resolved, so the collision mesh agrees). Ranked by footprint
relief, the best near site:

| lat_deg | lon_deg | azimuth | relief | slope | dist from KSC |
|---|---|---|---|---|---|
| **0.34377** | **270.03819** | **5° (≈east)** | **0.56 m** | 0.01% | 2.67 km |
| 0.34377 | 270.05729 | 0° | 0.75 m | −0.01% | 2.47 km |
| 0.05729 | 270.15279 | 35° | 0.75 m | 0.02% | 2.78 km |
| −0.09549 | 270.53475 | 70° | 0.44 m | −0.01% | 4.77 km |

**Recommended site** (the bold row): centre ground **249.06 m** above the base
radius, footprint min/max 248.50 / 249.06 m. With `height_m: 0.5` the slab top
sits at 249.56 m — 0.5 m above the highest ground under the footprint — and a
4 m slab bottom (245.56 m) is buried ≥ 2.9 m everywhere. No floating ends, no
clipping. `heading_deg: 5` puts the long axis (building local +X, spun by
`buildingOrient`, buildings.cpp:245-252) along the surveyed azimuth.

One caveat on that correspondence: `buildingOrient`'s base is `faceAlong(up)`,
whose horizontal axes come from a "most orthogonal coordinate axis" pick
(vehicle.cpp:409-419). At THIS site the pick lands on X≈east, Y≈north, so
heading 5° = the probe's east-based azimuth 5°; at another site the pick can
flip and heading is no longer east-based. The spawn formula (§5 stage 1) reads
the *actual* building basis, so ship↔mesh alignment is always right — only the
survey↔JSON correspondence is site-specific. Worth a comment in the fragment.

The probe is kept: `g++ -O2 -std=c++20 -I src -I middleware/glm
tmp/runway_probe.cpp -o tmp/runway_probe && ./tmp/runway_probe`. Re-run it if the
site is moved. **Visual check: ask the user to eyeball the placed runway in-game
before tuning anything else** (QWEN.md: visual confirmation is the user's call).

## 5. The stages

Each stage lands on its own, in order. Stages 0-1 give a visible runway and a
horizontal spawn even before wheels exist (the plane rests on its belly — a fine
interim state, and exactly what `level_jet` does on the pad today).

### Stage 0 — the runway building (data + assets, ~no code)

1. Mesh: a box 900 × 30 × 4 m, x ∈ [−450, 450], y ∈ [−15, 15], z ∈ [−4, 0]
   (top at z=0, like `space_port.obj`). Hand-author the .obj or add a tiny
   `utils/gen_runway.py` (trimesh, the `gen_wing.py` pattern). Run
   `utils/check_mesh.py` on it (watertight, consistent winding).
2. Texture: flat asphalt grey; `utils/gen_flat_tex.py` exists for exactly this.
   A centreline stripe is optional polish, not v1.
3. `res/data/buildings.json`: add
   `{"name": "runway", "display_name": "Runway", "mesh": "meshes/runway.obj", "texture": "textures/runway.png"}`.
4. `res/data/ksp_launch_sites.json`: add an instance
   ```json
   { "id": "ksc_runway", "type": "runway", "display_name": "KSC runway",
     "body": "Kerbin", "lat_deg": 0.34377, "lon_deg": 270.03819,
     "height_m": 0.5, "heading_deg": 5.0,
     "flags": ["VAB_LAUNCH_PAD", "RUNWAY"] }
   ```
   (Keep it AFTER `main_space_port` so the default site is unchanged.)
5. No C++ changes: catalog parse, placement, picker, map marks, `site=` CLI all
   work off the existing flag. `tests/test_buildings.cpp` needs TWO concrete
   extensions (not just "a new entry"):
   - the ksp-system count pin `CHECK(sys.world_buildings.size() == 4)`
     (test_buildings.cpp:365) becomes 5;
   - `check_pad_grounding` (test_buildings.cpp:77-110) hard-codes the
     space_port footprint (halfX 13, y −41..15, slab depth 10) and runs on
     EVERY world building (line 418) — make the footprint/depth per-type
     (runway: 450 × 15, depth 4) or the runway is checked against the wrong
     box. With the right footprint the §4 numbers pass (deck clearance
     0.5..1.06 m, inside the 4 m slab).

**Accept:** boot shows the runway in the surface map site list;
`--startship racer,res/ships/racer.json,Kerbin,pad,site=ksc_runway` stands the
racer on the deck (vertical, for now); `make test` green. One e2e canary.

### Stage 1 — horizontal spawn on RUNWAY sites (+ #846 fix)

1. `place_vehicle_def` (vehicle.cpp:892-925): when `site->hasFlag("RUNWAY")`,
   force the pose the pitch-90 branch already computes:
   ```cpp
   const glm::dmat3 site_basis = buildingOrient(pad_dir, heading_rad);
   pad_orient = faceAlong(site_basis * glm::dvec3(1, 0, 0), pad_dir);
   ```
   Nose along the runway long axis, +Y up, wings level. Skip the def-attitude
   block on a runway (comment why). Pads keep today's path untouched.
2. **Fix #846 in the same stroke:** `build_ship` (vehicle.cpp:182-203) measures
   the support along frame-S Z (`lo.getZ()`), which is only "up" for a vertical
   ship. Hand it the support direction (world up; the caller has `pad_dir`) and
   take the min projection of each part's inflated AABB corners onto
   `transpose(orient) * up`. Two callers only: vehicle.cpp:969 and save.cpp:1162
   (identity orient, margin 0 — passes `orient * Z` and is unchanged). This also
   removes the settle-drop `flight/461` currently tolerates.
3. Unit-test the support math (a tilted box rests on its corner, not its face)
   in `tests/test_spawn.cpp` or a small new test; extend `flight/461`'s claim if
   the settle wording changes.

**Accept:** `level_jet` (or any def) on `ksc_runway` spawns nose-horizontal,
wings level, at resting height — a 160-style no-free-fall check
(`max(alt)-min(alt) < 0.15`, `v < 0.5`); `flight/461` and `flight/160` still pass
on the pad. 1-2 e2e cases.

### Stage 2 — the wheel part + ground force law (the core)

1. **PartDef** (shipdef.h, loader shipdef.cpp, `gen_parts.py`): flat fields
   `wheel_radius` (m, >0 → gear), `wheel_travel` (m, default 0.3), `wheel_spring`
   (N/m), `wheel_damp` (N·s/m), `wheel_steer` (deg, 0 = fixed). Loader refuses
   negative/absurd values the way neighbouring fields do; `gen_parts.py --check`
   must stay green. `Part::isGearWheel()` next to `isWheel()` — keep the names
   apart (reaction wheels!).
2. **Mesh + part**: strut + wheel cylinder to the §3 convention (axle X, roll
   +Y, strut +Z, origin at axle). A `utils/gen_gear.py` in the `gen_wing.py`
   style. One size is enough for v1 (radius ~0.35 m, width ~0.2 m); the palette
   lists it automatically (gameui.cpp:6343 is a flat catalog loop) and surface
   attach + symmetry place a pair without any VAB code change.
3. **`src/wheel.h`** — the pure law (no Bullet, no Game; the gimbal.h pattern),
   roughly:
   ```
   struct WheelContact { bool grounded; double compression; dvec3 point, normal; };
   // spring: F_n = max(0, k*x + c*xdot) along the contact normal
   // tire:   v_c = v_com + omega x r;  split into roll/lateral at the patch
   //         F_lat = -C_lat * v_lat,  F_long = -(C_rr + brake*C_brake) * N * tanh(v_fwd/eps)
   //         clamp the pair to the friction circle mu*N
   // steer:  roll direction turned about the normal by the steer angle
   ```
   Constants (grip μ≈0.9, C_rr≈0.02, C_lat a few kN per m/s) live here, one
   spelling, tunable. **Starting spring numbers** for the skiff at its pinned
   **569 kg** (472 kg of parts, tanks full, + one 97 kg kerbal — flight/490's
   CLAIM): weight ≈ 5.6 kN, ~85% on the two mains → ≈ 2.4 kN per main →
   k ≈ 8-16 kN/m for ~0.1-0.15 m static compression, c ≈ 2ζ√(k·m_eff) with
   ζ≈0.6. These are starting points — the 10x soak (§2) and the margin-aware
   clearance rule (§2) decide them.
4. **`Vehicle::applyGroundForces(h)`** (vehicle.cpp, declared in vehicle.h):
   cached `std::vector<Part*> gearParts` rebuilt in `rebuildCompound` (the same
   place `aeroSystems` regroups); early-out when empty. Per wheel: world axle
   pose via `partWorldPose`, ray down `radius + travel + slop` through a new
   `physics.h` wrapper (`groundRay(from, to, &hit, &normal, skipBody)` —
   `btCollisionWorld::rayTest`, one reused callback, **no allocations**); on a
   miss, the analytic terrain fall-through (`GetTerrainHeight`, radial distance,
   normal = up — the EVA two-source pattern). Spring + tire forces applied with
   `ApplyForce(hull, contact - comPos, F)`; record into `lastForceVecs` under
   `recordForceVecs` so the **F4 overlay shows wheel forces for free** (every
   other applier does this explicitly, e.g. vehicle.cpp:3927).
5. **Hook it up**: tick.cpp substep loop, after `applyAeroForce` (order among
   force appliers is free — nothing integrates until `physics_tick`; velocity-
   dependent terms read the same fresh state aero does).
6. **A plane to fly**: `res/ships/skiff_gear.json` = `skiff.json` + nose wheel
   under the pod and two mains under the wings (surface attach, symmetry-style
   mirrored `roll`). The skiff is the airframe the flight model was tuned on
   (e2e INDEX flight/490): 569 kg with crew and fuel, jet static thrust ≈ 5.6 kN
   (`jet_fan_thrust` 5625 N at v=0) → ~9.9 m/s², rotation in the low tens of m/s,
   a couple hundred metres of roll — the 900 m deck is ample.
7. **Tests**: `tests/test_wheels.cpp` pins the pure law (spring sign/clamp,
   friction-circle clamp, zero force airborne, steer direction, no force when
   the ray misses and terrain is far). Ship-load tests pin the new fields.

**Accept:** unit tests green; e2e (pick the next free flight numbers, regenerate
`e2e/INDEX.md` with `python3 e2e/index.py`):
- *runway-rest*: spawn on the runway, 5 s, no sink/no slide (`v < 0.2`, alt steady);
- *runway-takeoff*: `--sim-press 2000,12000,T` (hold thrust from t=2 s), assert
  **ground speed from `--dbg-log`'s `v`** climbs, then alt climbs past gear
  height and the hull is flying. Do NOT read speed off the drag log's |v| —
  that is AIRSPEED (tick.cpp:923-928) and Kerbin authors a surface wind, so a
  runway pointing into it reads a headwind offset. Drag-log `up`/`alt_asl` are
  fine for the pose/altitude half (flight/461 reads `up`);
- a 10x-warp parked soak (no ringing — §2).
Manual: fly it yourself, then **ask the user to look** (QWEN.md).

### Stage 3 — brakes, steering, UI

1. `Slot::Brakes` in keys.h (+ name/label/group rows in keys.cpp, default **B**
   — verified free, flight group), latching toggle in events.cpp like
   `ThrustLatch` (events.cpp:400-411), armed per tick in tick.cpp. Transient
   `Vehicle::brakesOn`: **default true at spawn; on load, true only when the
   ship is near-static** — v1 persists nothing, so an unconditional default
   would load a save taken mid-takeoff-roll with locked brakes and skid it.
   (Persisting the bit later follows the chute/trim pattern, §7.) The wheel law
   reads it.
2. Nose-wheel steer: while any wheel is loaded, yaw demand (the same normalized
   -1..1 the attitude laws use) → steer angle on `wheel_steer > 0` wheels,
   tapered to zero by ~30-40 m/s. The lateral tire force at the nose contact
   yaws the hull — no separate torque law.
3. HUD/vessel-window **BRK** indicator when brakes are on (the HUD already has a
   surface mode, constants.h:186); a per-wheel compression/load row in the part
   window is nice-to-have, not required.
4. e2e: *runway-brakes* (thrust roll, then B → v decays to ~0 before the deck
   ends); *runway-steer* (A/D at low speed bends the ground track; at high speed
   the taper keeps it straight). Follow `flight/41-rebind` for the key plumbing.

**Accept:** the three e2e above + `make test`; a full manual taxi/takeoff/land/
brake loop, user-confirmed.

### Stage 4 — corpus, docs, loose ends

- `res/ships/README.md`: a short law for gear placement (mains slightly behind
  the COM, nose wheel forward, roll axis aimed along the nose) — the corpus
  README is where ship-design law lives.
- Save/load round-trip with gear (no new fields — pin that it stays that way),
  recovery of a landed plane (existing `recoverActive` path, crew returns to the
  home pad), warp-parking on the runway (existing `isGrounded`).
- DESIGN.md: tick "Runway building, wheels for airplanes" off the future list.
- Close #846 (stage 1 fixed it).

## 6. Intersections with DESIGN.md's future work

- **Ground vehicles (wheels, suspension, brakes):** this plan *is* their
  foundation. The wheel law is hull-agnostic (any Vehicle with gear parts);
  suspension, brakes and steering already exist after stage 3. The one missing
  channel is **drive**: a `wheel_motor` def field + throttle→longitudinal force
  at the patch when grounded. Deliberately not built now — but the tire law's
  longitudinal slot is where it lands, so a rover later is a part + a force
  term, not a new physics system.
- **Player buildings:** runways are catalog buildings today; player placement
  reuses the same StaticBuilding path and flags. The flatness problem recurs
  there (you can't survey every player site) — **terrain flattening around
  buildings is the pre-requisite to design then**, not now.
- **Animations:** gear retraction, wheel spin, steering visuals all wait for the
  animation system. The wheel law exposes compression and ground speed per
  wheel, which is everything a render-only spin/retract needs later.
- **Failure:** per-wheel contact force is available for gear-collapse /
  overload failure when the failure system lands. Nothing to do now.
- **Propeller planes / helicopters:** the same gear works (taildraggers, skids
  as zero-steer wheels). Propulsion is a separate part kind, out of scope.
- **Atmosphere rails / balloons:** no interaction — a railed hull runs no
  substeps, so the wheel pass simply doesn't run, like aero.

## 7. Risks and gotchas (the things that will bite)

1. **Explicit-spring stability at 10x warp** — §2. The one real physics risk;
   soak it, don't assume it.
2. **`isWheel()` is a reaction wheel.** The new predicate must be
   `isGearWheel()` (or similar) or every attitude-authority consumer starts
   treating landing gear as torque drums.
3. **Terrain LOD gap:** collision meshes exist only at max-LOD leaves; the
   analytic fall-through is mandatory, not optional (EVA learned this). The
   runway itself is a static body and always ray-hittable.
4. **Double support / margins:** if the ride height is sized wrong, the wheel
   hull rides the deck and friction 4.0 scrubs the plane to a stop. The
   clearance rule is margin-aware (§2): `travel − compression > hull_margin +
   support_margin + slop`, and the ray hits the margin-inflated deck. The
   rest-height e2e (no slide, no sink) catches a violation; the unit test must
   pin the arithmetic WITH the margins, not against the visual deck.
5. **One deck, one ship:** `findPadOccupant` (pick.cpp:226) samples a pad-sized
   grid — `lat[] = ±9` on local X, `along[] = ±30` on local Y (pick.cpp:236-264).
   The runway's long axis is X, so it guards only ±9 m ALONG the deck (±30 m
   across): a ship parked at the far end won't block a launch. Fine for v1;
   note it, don't fix it.
6. **`isGrounded` reads analytic terrain, not the deck** — a parked plane is
   ~1 m "above terrain", inside the 100 m band, so rails parking works. A plane
   rolling > 3 m/s refuses rails (correct). Don't tighten the band for this.
7. **Hot path:** cache the gear list; reuse the ray callback; no allocations per
   substep (the project rule). A few rays × a few wheels × substeps is
   microseconds — cheap, but only if it allocates nothing.
8. **Save format:** v1 adds no persistent state. If gear deployment or a saved
   brake state arrives later, follow the `chute`/`trim` optional-field pattern
   (save.h:76-84) — absent = default, never a refusal.
9. **e2e budget:** 1-5 cases per stage (QWEN.md), regenerate INDEX.md, run
   `make test` + the relevant cases before each commit. Don't push.
10. **EVA on the runway works, but is contact-only.** A kerbal on the slab is
   held on top by Bullet contact (`BodyInContact` is a whole-world contactTest
   including static buildings, physics.cpp:343-357); the analytic band
   (`restAlt + 0.25 m`, eva.cpp:28) never covers a deck ~0.5-1 m above terrain,
   and the floor guard only fires 0.4 m BELOW rest altitude (eva.cpp:118-126),
   so it cannot snap a kerbal into the slab. Two consequences: a kerbal SAVED
   standing on the runway loads in EVA_SPACE for one tick until the first
   `armInput` contact flips it back (`settleContact` is terrain-only,
   eva.cpp:36-44 — a documented quirk, not a new bug); and nobody should "fix"
   the wheel fall-through by analogy with EVA's analytic band — the wheel ray
   already sees the slab, EVA's band deliberately does not.
11. **No static stiction in v1.** The brake/rolling terms are viscous
   (`tanh(v/eps)`), so "holds still" is really "creeps imperceptibly" on a slope
   (~1e-4 m/s at the site's 0.01% grade — the `v < 0.2` rest test passes). Fine
   for v1; a future "why does it drift" is this, not a bug hunt. Add a stiction
   band only if a steeper site ever needs it.

## 8. Verification commands (the working loop)

```
bear -- make                      # build
make test                         # unit tests (incl. new test_wheels.cpp)
python3 e2e/run.py <new selectors> flight/160 flight/461   # targeted e2e
SDL_VIDEODRIVER=offscreen ./osp --timeout 20 \
  --startship skiffgear,res/ships/skiff_gear.json,Kerbin,pad,site=ksc_runway \
  --sim-press 2000,12000,T --dbg-log --orbit-interval 0.2   # scripted takeoff roll
```

(`T` = hold thrust, `R`/`F` = throttle rate, `A`/`D` = yaw → nose-wheel steer,
`B` = brakes once stage 3 lands. `--ui-click`/`--ui-list` for the VAB route.)

## 9. Reflection — what would have made this easier

- The attitude-tilt mechanism (#461) already did most of "horizontal spawn" but
  DESIGN.md still listed the whole item as future work — a one-line "partial:
  def-attitude tilt exists, see flight/461" in DESIGN.md would have saved the
  rediscovery. Keep DESIGN.md's future list honest about partial state.
- `VAB_LAUNCH_PAD` is a stringly-typed flag spelled in ~8 files. A tiny flag
  registry (or at least a comment in buildings.h naming every consumer) would
  make "add a flag, inherit the picker/map/recovery" a documented move instead
  of a grep.
- `build_ship`'s support math silently assumed "ship axis = up" — the kind of
  invariant that deserves an assert or a comment at the measurement site. #846
  is the cost of it being implicit.