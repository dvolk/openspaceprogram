# Air-relative buildings: a balloon platform in Venus's clouds

Status: **IMPLEMENTED (v1)** — built and tested 2026-11-14/15. The staged plan
below is as proposed; §10 records what implementation added beyond it (the deck
carry and the moving-ground fixes, which were the bulk of the real work).
Date: 2026-11-14. Companion to `reports/orbital-buildings2026_11_14` (the
scaffolding this reuses) and `reports/runway-wheels2026_11_14` (the horizontal
spawn + gear it stands on).

## 0. How much work from here?

Roughly **two thirds of the orbital-buildings v1 effort**. The expensive parts
already exist and are reused unchanged: the driven-pose `StaticBuilding`
machinery, the pure-f(time) discipline, the proximity gate, the frame-aware
draw, the paused-spawn fix, the horizontal spawn attitude, the VAB site picker,
and the wind law itself (`windAt`, `windVelRot`). Only two genuinely new pieces:

1. a small pure law — **zonal drift** (`buildingair.h`, ~60 lines + tests), and
2. a small plumbing change — **collision mesh separate from visual mesh** (the
   deck collides, the balloons don't): one optional parameter threaded through
   four functions (`create_body` → `RegisterPhysicsBody` → `RegisterObject` →
   `BuildHull`).

Everything else is a branch beside an existing orbital branch.

## 1. What we're building (v1)

A **hexagonal launch platform floating at ~56 km on Venus, carried by balloons**,
drifting east with the super-rotating wind (~86 m/s). It is a VAB launch site:
a plane spawns horizontal on its deck, at rest relative to the platform — and
therefore at rest relative to the air, zero airspeed, no aero load.

Same authoring model as orbital buildings: **predefined in the system JSON**,
not player-built. Same physics model: **mass-0 static collider, pose driven,
proximity-gated** — no buoyancy simulation, no balloon physics, no docking.
The balloons are scenery. This is the user's standing decision for buildings
(performance, and "colliding with a station shouldn't impart rotation").

Out of v1: player construction, multipart platforms, balloon collision,
docking ports, weathervaning, catapult launch, map markers, and rails for
ships parked on the platform (§8.1 — requested, deliberately deferred).

## 2. What the code already gives us (evidence)

| Fact | Where |
|---|---|
| Venus is fully authored: 92 bar CO₂, 2-level T profile, wind 1→100 m/s east over 0–65 km, cloud deck 55 km, air top 116 km | `res/systems/solar_system.json:133` |
| Default system is `ksp_system.json` (no Venus) — run `--system res/systems/solar_system.json` | `src/cli.cpp:58` |
| Wind law is pure data: `windAt(rows, alt, lat, lon)` → (east, north, up) m/s **relative to the ground**, narrowest-band, linear in alt | `src/atmo.h:479` |
| Profile → rotating-frame velocity at a point already exists: `windVelRot(b, W, pos_rot)` — the spawn branch calls it directly | `src/vehicle.cpp:505` |
| Driven-pose building: `tickPose`/`poseFrame`, pose is a pure f(clock) | `src/terrain.h:734`, `src/terrain.cpp:64` |
| Pose + gate + `[bldg]` log run every tick **including paused** | `src/tick.cpp:246-290` |
| Proximity gate: collider in the world only within `prox_fly_on` (2 km), out beyond `prox_fly_off` (10 km) | `src/tick.cpp:257` |
| Frame-aware `Draw` (fixes #847) — works for any `poseFrame()` | `src/terrain.cpp:81` |
| Ships in air can never rail: `canRail() = isGrounded() \|\| !inAtmosphere()`; `isGrounded` is a terrain-band test (100 m, 3 m/s) | `src/vehicle.cpp:6916-6931`, `src/constants.h:170` |
| Ground rays hit **any** world body except the ship's own hull — wheels rest on a building collider exactly as on the runway | `src/physics.cpp:360` (`GroundRayCallback`) |
| Horizontal spawn: `RUNWAY` flag forces nose-along-deck, wings level | `src/vehicle.cpp:919` |
| Orbital spawn branch: inherit the site's velocity, re-place | `src/vehicle.cpp:1040` |
| Collision shape = convex hull of the mesh's vertices (`btConvexHullShape`) | `src/physics.cpp:283` (`BuildHull`) |
| VAB body + site combos are body-filtered; a Venus pad with `VAB_LAUNCH_PAD` appears automatically; `find_launch_site` is per-body so Earth's default is untouched | `src/gameui.cpp:6089,6113`, `src/buildings.cpp` |
| `[dbg]` logs the ship in **its own frame**; `[bldg]` logs the pose in `poseFrame()` — for an air platform both are the rotating frame, so 855-style distance CHECKs work unchanged | `src/tick.cpp:626,282` |
| `faceAlong(dir)` picks its roll reference by **discrete comparison** — a moving `dir` can flip the pick and snap the attitude 90° | `src/vehicle.cpp:419` (and the comment at 433) |

## 3. The air at the platform (game's own laws)

`tmp/venus_air_probe.cpp` (kept, re-runnable) evaluates Venus's authored
profile through `resolveAtmoLevels` / `levelPressure` / `airDensity` / `windAt`
— no re-derivation. Venus: μ = 3.2484e14 m³/s², g = 8.87 m/s² at datum,
M = 0.04345 kg/mol (CO₂ 0.965 / N₂ 0.035), ρ_datum = 65.2 kg/m³.

| alt | T | p | ρ | wind east | sound |
|---|---|---|---|---|---|
| 50 km | 347 K (74 °C) | 109 kPa | 1.64 kg/m³ | 77 m/s | 294 m/s |
| 55 km | 308 K (35 °C) | 54 kPa | 0.92 kg/m³ | 85 m/s | 277 m/s |
| **56 km** | **300 K (27 °C)** | **47 kPa** | **0.81 kg/m³** | **86 m/s** | **274 m/s** |
| 57 km | 292 K (19 °C) | 40 kPa | 0.72 kg/m³ | 88 m/s | 270 m/s |
| 60 km | 269 K (−4 °C) | 25 kPa | 0.48 kg/m³ | 92 m/s | 259 m/s |

**Altitude choice: 56 km.** Venus gets *hotter* downward (lapse −7.8 K/km):
the shirt-sleeves band is 55–57 km, just above the 55 km cloud deck; below
~52 km it is >58 °C. 56 km is room temperature at half an atmosphere — the
classic Venus balloon-habitat altitude — and the denser air buys ~40% smaller
balloons than 60 km. Temperature is currently flavor + physics (density, sound
speed); crew heat damage is not implemented (DESIGN.md failure list), so this
is an aesthetic + sizing choice. It is one authoring number; the engine does
not care.

**Buoyancy (flavor, not simulated).** A helium balloon at ambient p, T lifts
(ρ_air − ρ_He)·g per m³. At 56 km: 6.44 N/m³ (0.74 kg/m³ at the local g =
8.71 m/s²). A 10 t platform needs ~13,500 m³ — a single sphere r ≈ 14.8 m, or
**six balloons r ≈ 8.1 m** on cables, which is the look we want over a ~80 m
hex deck. S3 authors r ≈ 8.5 m (~14% buoyancy margin) on **long tethers,
~60 m** — balloons well above the deck for visual impact (user request).
(H2 is ~5% better; He is the safe authoring story.)

**Drift.** The platform rides the wind relative to the ground: 86.3 m/s east
at 56 km → a lap of Venus in ~5.1 days at the equator, ~2.96°/hr at lat 10°.
This is the load-bearing number for the pose law (§4.1) and for why the
existing parked-rails mode cannot be reused (§8.1).

## 4. Design decisions

### 4.1 Pose law: frozen-field zonal drift — pure f(g.time)

The wind field is **steady** (`windAt` has no time argument). v1 law:

- the platform holds its authored **lat** and **alt**;
- `lon(t) = wrap(lon0 + rate·t)`, with `rate = east / (r·cos lat)` sampled
  **once at the anchor** at placement;
- attitude: local-horizontal, authored heading (§4.2);
- velocity (for spawn inheritance): `windVelRot` at the platform position —
  it samples the same field, so ship and platform agree by construction.

For Venus's authored wind (east-only, whole-sphere band, linear in alt) this
is **exact, not an approximation**: the platform stays at constant lat/alt, so
the sampled wind never goes stale. It preserves the invariant the orbital work
established: pose is a pure function of `g.time`, so save/load and arbitrary
warp reproduce it, and buildings stay **unpersisted** (re-instantiated from
JSON). At 1e7 s of warp the wrapped angle is still sub-mm precise in double.

**Limitation (documented, ticketed):** north/up components and band-crossing
are not advected. A future body that authors meridional wind under a platform
would diverge from true advection. Full advection needs persistence or
epoch-reintegration — that is the DESIGN.md "atmosphere rails" item (§8.2).
Rejected for v1: per-tick numerical advection breaks pure-f(time) (warp steps,
load replay), needs a save format for buildings, and buys nothing on Venus.

### 4.2 Attitude: continuous ENU basis, NOT `buildingOrient`

`buildingOrient` → `faceAlong(up)` picks its roll reference by comparing
|dot| against the three coordinate axes — a **discrete** pick. A static pad
never moves so it never snaps; a platform drifting through longitude crosses
pick boundaries and would **snap ~90° in roll** mid-flight. Verified
numerically: at lat 10° the pick flips at ±10.2° around **every** 90°
meridian (8 flips per lap); from the authored lon0 = 0 the first snap lands
~3.4 h after spawn. The air law therefore builds its basis directly:

```
orient = [east(up) | north(up) | up] · rotZ(heading)
```

`equirectEast`/`equirectNorth` are continuous everywhere off the exact pole
(the pole fallback is documented in equirect.h). Heading 0 → deck +X = east;
author **heading 180** so the deck's +X (the spawn nose direction) points
**west, into the wind**. The spawn's two-argument `faceAlong(nose, up)` is
already continuous (explicit up reference, no pick), so the RUNWAY attitude
line composes with this safely.

### 4.3 Physics: static collider, gated — same as orbital

Mass-0 static body, pose driven each tick, present only near a ship. A ship
bounces off the deck; the platform never moves or rotates in response. No
buoyancy force, no balloon bodies. (The kinematic option is still blocked by
#848 anyway.)

**One required gate change:** the gate today keys on the **active** ship only
(`tick.cpp:257`). That was sufficient for orbital buildings because any
non-active ship near a dock is out of the air and therefore railed. An air
platform breaks that: a ship parked on it **cannot rail** (§2, `canRail`), so
when the player switches away, the parked ship stays **live** — and the gate
would remove the collider out from under it, dropping it through the deck.
v1 therefore gates on **any live ship** near the building, not just the active
one. Cost: a scan of `g.vehicles` per driven building per tick; live ships are
few (only in-atmo ships stay live). No behavior change for the shipped orbital
sites (their non-active neighbors are railed); the fix also closes the same
latent hole for any dock ever authored *inside* an atmosphere. Pinned by an
e2e (§6, S4).

### 4.4 Collision mesh ≠ visual mesh (the only new plumbing)

`BuildHull` hulls the mesh's vertices. A mesh that includes the balloons would
hull the **whole envelope** — the deck would sit *inside* the collider and a
ship could never rest on it. So:

- visual mesh: hex deck + slab + six balloons + cables;
- collision mesh: the hex slab alone (convex, exact);
- `BuildingDef` gains optional `"collision_mesh"`; `create_body` gains an
  optional collision-mesh parameter threaded to `RegisterObject`/`BuildHull`
  (default = the visual mesh, so every existing caller is unchanged).

Chosen over the "register with one mesh then swap `body->mesh`" trick: four
explicit signatures beat a subtle post-hoc swap a local agent must discover.
Verified safe: `BuildHull` is the only physics reader of `body->mesh`
(`captureHullVerts` reads the shape, `effectiveHullMargin` reads a scalar).
Balloons and tethers are non-colliding in v1 (a ship can pass through them —
user-confirmed); balloon collision is a follow-up if it ever matters.

### 4.5 Spawn: vertical pad pose + wind velocity, rotating frame

In `place_vehicle_def`'s `on_pad` block, for an air site:

1. compute the pose at `g.time` from the pure law (do not trust the stored
   pose — same discipline as the orbital branch);
2. `pad_dir = up(t)`; `pad_orient` = the **vertical pad pose** (the ENU basis's
   +Z is up), so a rocket stands on its tail like on the space pad. (The
   proposal first reused the RUNWAY horizontal line; corrected in implementation
   — a platform is a pad, not a runway. A per-ship orientation heuristic is #851.)
   The def's optional `attitude` tilt still applies;
3. `base = posePos` — the mesh is authored with the **deck top at local z=0**
   (slab below), the same origin convention as `space_port.obj`, so
   `build_ship` rests the hull's lowest point exactly on the deck;
4. after `build_ship`: `v->setVelocity(windVelRot(hb, wind, posePos))`, frame
   stays `hb->rot_frame` (56 km is inside the atmosphere shell, where ships
   integrate), `setSoi(rot_frame)`. No re-placement needed — unlike orbital,
   the ship is *supported*, not free.

Brakes on at spawn (existing). Zero airspeed → zero aero load; the suspension
holds against gravity (wheel clearance rule from the runway work applies:
travel − compression > 0.2 m margin envelope + slop).

**Brakes must persist (review finding).** The load path today re-derives
`brakesOn = |vel| < 3 m/s` (`save.cpp:1089`) — a parked platform ship saves
with ~86 m/s rotating-frame velocity, so it would load with brakes OFF and
creep. v1 persists the flag in the ship save instead of re-deriving it (also
correct for a runway ship saved rolling with brakes on).

**Departure is a drop, not a roll.** An ~80 m hex deck is far too short for a
takeoff roll (ρ 0.81 kg/m³ → stall ~1.25× Earth sea level). v1: taxi off the
edge and dive — the dense lower atmosphere makes recovery easy, and a
balloon-base launch *should* feel like a drop. Open question §9.1 (catapult /
bigger deck are alternatives, not planned).

### 4.6 Reused unchanged

Proximity gate hysteresis, `[bldg]` log line (poseFrame is the rotating frame
here), frame-aware `Draw`, parse scaffolding (mutual exclusion, contextual
errors), VAB picker, `findPadOccupant`'s ray machinery (but see S5 — it must
read the driven pose). Buildings stay unpersisted; the only save change is
the ship's `brakesOn` flag (§4.5).

## 5. Authoring form

New fragment `res/data/venus_launch_sites.json`, named by
`solar_system.json`'s `world_buildings_files` (mirrors `earth_launch_sites.json`):

```json
{
  "id": "venus_cloud_platform",
  "type": "balloon_platform",
  "display_name": "Venus cloud platform",
  "body": "Venus",
  "air": { "alt_m": 56000, "lat_deg": 10.0, "lon_deg": 0.0 },
  "heading_deg": 180.0,
  "flags": [ "VAB_LAUNCH_PAD", "AIR_PLATFORM" ]
}
```

- `"air"` is **mutually exclusive** with `"orbit"` and with surface
  `lat_deg`/`lon_deg` (parse refuses the combinations, contextual errors).
- Validation (parse + `validate_world_buildings`, so a cross-system load
  refuses **before** teardown, #302): body has a physical atmosphere **with a
  wind table** (`windAt` asserts non-empty); `0 < alt_m < atmo top`;
  `lat_deg` in [-90, 90] and **|lat| ≤ 85** (the drift rate divides by cos lat
  — a polar platform is a singularity, refused rather than clamped); finite
  numbers via the existing `optNum` pattern.
- Catalog (`res/data/buildings.json`): `balloon_platform` →
  `meshes/balloon_platform.obj` (visual), `meshes/balloon_platform_deck.obj`
  (collision), `textures/balloon_platform.png`.
- First `VAB_LAUNCH_PAD` on Venus = this platform (there are no other Venus
  sites); Earth's default (baikonur) is untouched — `find_launch_site` filters
  by body and fragments merge before inline entries.

## 6. Staged plan

Each stage lands independently testable; sizes are relative to the orbital
feature's stages.

**S1 — pure drift law (small).** `src/buildingair.h`: `driftRate(east, r,
lat)`, `lonAt(lon0, rate, t)` (wrap via `wrapLonDeg`), `poseAt(alt, lat, lon0,
rate, heading, t)` → {pos, orient, up} on the continuous ENU basis, `velEast`
helper. glm + STL only, no Bullet/Game — the `buildingorbit.h` pattern.
`tests/test_buildingair.cpp`: rate matches the probe (86.29 / (r cos lat));
antimeridian wrap; pose(t) == pose(t + lap) to precision; heading 180 points
deck +X west; zero wind → static; attitude continuity across a faceAlong pick
boundary (the §4.2 snap, pinned absent).

**S2 — parse + placement + drive (small–medium).** `WorldBuilding` air fields;
parse validation (§5); `StaticBuilding` air state (anchor lat/lon/alt, drift
rate, heading) + `tickPose` air branch + `poseFrame()` = rotating frame;
placement samples `windAt` at the anchor, `RemoveBody` + `tickPose(0)` like
orbital. Gate/log filters become `orbital || air` (a `driven()` helper).
**Gate keys on any live ship** (§4.3). `tests/test_buildings.cpp`: parse
rejections (air+orbit, air+latlon, no-atmo body, no-wind body, alt ≥ top,
|lat| > 85, non-finite), placement drift-rate pin. The existing surface-pad
checks (projection round-trip, `check_pad_grounding`, pad overlap) gate on
`!orbital` today — an air entry carries lat/lon and would be checked as a
phantom 5 m surface pad, so they must skip air too (review finding).

**S3 — meshes + collision split (medium; the only new machinery).**
`utils/gen_platform.py`: hex deck (80 m across, 2 m slab, top at z=0), six
balloons r ≈ 8.5 m on **~60 m tethers** well above the deck (visual impact,
user request), `include_normals=True`; deck-only collision mesh.
`create_body` optional collision-mesh parameter → `RegisterPhysicsBody` →
`RegisterObject` → `BuildHull`. Catalog entry. **Visual check is the user's
call** (offscreen run + screenshot).

**S4 — spawn + e2e (small–medium).** `AIR_PLATFORM` branch (§4.5); share the
RUNWAY attitude line. e2e (selectors, `--system res/systems/solar_system.json`):
- `flight/858-platform-spawn-rests`: ship spawns on the deck, ship↔platform
  distance stays bounded (855 pattern, both logs in the rotating frame),
  |v| ≈ 86 m/s, |pos| ≈ R+56 km — not falling. (Check the pos RADIUS: `[dbg]`
  alt= is AGL, and Venus has 2 km of relief — review finding.)
- `flight/859-platform-drifts`: `[bldg]` longitude advances at the analytic
  rate (pos after t matches `east·t` along the latitude circle); pure f(time):
  two runs at different warp agree.
- `flight/860-platform-paused-present`: 857 pattern — paused spawn, collider
  live, ship beside it.
- `flight/861-parked-ship-keeps-deck`: spawn ship A on the platform, ship B
  far away (two `--startship` entries), make B active; A must **not** fall
  (the §4.3 gate fix, pinned).

**S5 — map/UI exclusions + occupant guard (small).** Surfmap pin/count and the
surface window skip air sites (a baked pin at the anchor longitude would lie
within the hour — same treatment as orbital). `findPadOccupant`: for air sites
read the **driven pose** (`posePos`/`poseOrient`) instead of `origin()`/`dir`,
and **scale the ray grid to the deck** — the surface grid (±9 m lateral,
±30 m along) cannot see a ship parked near the edge of an 80 m hex, so two
launches could stack (review finding). Keeps one-deck-one-ship. Dossier/label
shows altitude.

**S6 — regression + soak (medium).** `make test`; e2e 854–857 (orbital
untouched), 160/461/850/851 (pad/runway), new 858–861; warp soak with a ship
on the platform (pose pure → stable); quicksave/quickload round-trip (ship
rests on the re-instantiated platform at the same `g.time`); fly-to from orbit
(gate across the inertial→rotating shell boundary); head-on collision bounce
(856 pattern).

## 7. Edge cases & risks

- **Wheel margin envelope** on the deck: 0.1 hull + 0.1 support = 0.2 m; gear
  compression must exceed it (runway rule). The deck is flat, so no tilted-pad
  rest-height issue (#846).
- **Tire creep**: viscous tires have no static stiction → mm/s creep on the
  deck; brakes hold; aero at cm/s airspeed in 0.81 kg/m³ air is negligible.
- **Falling off**: a ship that leaves the deck inherits ~86 m/s east = zero
  airspeed, then gravity takes it; drag builds as it falls. Recovery from the
  platform is **not** available in v1 (`recover` requires `isGrounded()`,
  `game.cpp:2470`) — documented; the fix rides with §8.1.
- **Warp while parked**: capped below rails warp (10×) while the parked ship
  is ACTIVE; switched away, `updateProximity` caps at `prox_warp` (default
  **1×**) because the parked ship stays live. Worse than it looks — another
  reason §8.1 matters. Acceptable for v1.
- **`isGrounded()` is false on the platform** (terrain-band test), beyond the
  recovery consequence above: the map draws the parked ship's co-rotation
  circle as an orbit plane (`render.cpp:514`), the transfer planner offers a
  plane normal from its wind velocity (`transferplanner.cpp:458`), and the HUD
  situation reads Flying. Known v1 cosmetics; the `isGrounded` extension in
  §8.1 fixes them together.
- **Pole singularity**: refused at parse (|lat| ≤ 85), not clamped.
- **No-wind body**: refused at parse/validate — `windAt` asserts non-empty.
- **Cross-system load**: air validation lives in parse + `validate_world_buildings`
  (pre-teardown, #302), not in the placement loop.
- **Two platforms**: independent gates; no interaction.
- **Save/load**: buildings stay unpersisted; the ship's rotating-frame pose +
  wind velocity, the persisted `brakesOn` (§4.5), and the platform's pure pose
  reproduce the rest state exactly.

## 8. Follow-ups (kept open, not v1)

### 8.1 Parked-on-platform rails / building frames (deferred; #850)

**v1 stopgap shipped:** the **deck carry** (§10) holds a braked ship rigidly on
the moving deck without rails. The fuller idea, captured as **#850**, is to give
buildings their own frame (a ~10 km "SoI bubble") so a landed ship is at rest in
it like on a planet — which would dissolve the carry, the `brakesOn` save case,
and the `isGrounded` cosmetics, and unify with docking (#848). The original
building-anchored-rails sketch is kept below as the narrower version:

Today a ship on the platform can never rail (§2). The existing parked mode
freezes a ship **in the rotating frame** — but the platform moves through that
frame at 86 m/s, so a ship frozen the old way would be left behind in midair
within seconds. The right shape is a **building-anchored freeze**:

- `canRail()` admits "resting on a driven building": extend `isGrounded()` (or
  a sibling) with a geometric deck-box test + relative speed < 3 m/s;
- a new rail flavor stores **building id + local offset + relative orient**;
  `railsTick` re-places the ship at `buildingPose(t) + offset` — pure f(time),
  warp-safe; the save stores the id + offset (buildings re-instantiate by id);
- `leaveRails` restores the platform's velocity (the ship is moving with the
  wind, not at rest).

This also unlocks rails warp while parked, recovery, and landed science on the
platform, and composes with the gate (a railed ship needs no collider). It is
a bounded change but touches the rail/save path — hence its own stage later,
not smuggled into v1.

### 8.2 Others

- **Full wind advection** (north/up, band-crossing) — the DESIGN.md
  "atmosphere rails" item; needs persistence or epoch reintegration.
- Balloon collision (compound shapes); buoyant balloon **parts** for player
  craft; multipart platforms; weathervaning; catapult launch; docking ports.
- Map markers: orbital stations (still deferred from the orbital work) and a
  live-drifting platform pin on the surface map.
- `isGrounded`/science/recovery on platforms (rides with §8.1 / #850).
- **#849** — the throttle-lever-vs-actual-thrust naming trap the carry exposed
  (lever defaults to 1.0; `availableThrust`/`producedThrust` are potential, only
  `m_thrust`/`armedThrust` mean "firing"). Document + an `isThrusting()` accessor.
- **#850** — buildings as frame-bearing bodies (the SoI bubble), the principled
  replacement for the deck carry, unified with docking (#848).
- **#851** — pick spawn attitude from site + ship (plane horizontal, rocket
  vertical) instead of one rule per site.

## 9. Open questions — RESOLVED (user, 2026-11-14)

"Your picks look good to me" — all four approved as recommended:

1. **Departure**: drop off the edge.
2. **Deck size / look**: 80 m hex, six balloons r ≈ 8.5 m, **long tethers
   (~60 m) for visual impact** (user addition). Balloons/tethers non-colliding
   — user-confirmed. Visual confirmation is the user's once S3 lands.
3. **Site**: lat 10°, lon 0°, 56 km, heading 180 (nose into the wind).
4. **Gate change**: any live ship instead of active-only — approved.

Review-forced decision (no user input needed): persist `brakesOn` (§4.5).

## 10. Implementation notes (what building actually took)

The pose/parse/placement/spawn scaffolding went in as planned (§6 S1–S5). The
bulk of the real work was making a ship **rest on a moving deck** — a problem the
proposal under-priced. A static deck is stationary to Bullet, but the platform
rides the wind at 86 m/s, so every ground interaction had to learn that the
ground itself moves.

**The moving-ground problem.** A ship parked on the deck shares its 86 m/s
frame velocity. Three ground paths each assumed a stationary surface and scrubbed
the ship off:
- the **wheel law** read tire slip against frame velocity → fixed to read
  `vel − surfaceVel` (`Body::surfaceVel`, set by `tickPose`);
- the **EVA walk law** drove horizontal velocity to zero in the rotating frame →
  fixed to walk relative to the deck (a short ground ray finds `surfaceVel`);
- **Bullet's contact solver** maps every static body to one zero-velocity fixed
  body, so hull friction scrubbed a gearless hull/kerbal. Kinematic was tried and
  reproduces #848's pass-through, so the deck is **static and frictionless**
  (`SetFriction(body, 0)`); grip comes from the wheel/EVA laws and the carry.

**The deck carry** (`Vehicle::tickDeckCarry`, vehicle.cpp). A braked ship at rest
relative to a moving deck is driven rigidly from the deck's pose each substep —
the "poor man's SoI bubble" (#850 is the principled version). Details that each
fixed a user-reported bug:
- engagement is a down-ray reaching just past the **hull's lowest AABB point**,
  not a fixed COM distance — a fixed reach misses the deck under a tall upright
  rocket (review finding), leaving it grip-less on the frictionless deck; the
  scaled reach finds the deck at any height while a hovering ship still misses.
  Works for wheel-less hulls, which skip `applyGroundForces`. Release on
  brakes-off, **actual thrust** (`m_thrust`, not the lever or `availableThrust` —
  #849), RCS translation, or EVA walk/jump; a frame change / leaving the world
  also drops it;
- velocity is the deck's **analytic** `surfaceVel + surfaceAngVel × r`, not a
  position finite-difference (a difference cancels the motion or spikes across
  pause/warp changes);
- the hull is aimed `vel*h` **behind** the deck point because `physics_tick`
  integrates the hull but not the static deck — otherwise the ship rides one
  substep ahead and jumps relative to the deck whenever warp changes `h`;
- the carry **settles while paused** too, so its offset is measured against the
  frozen deck — otherwise the first unpause of a new game jumps by ~`v*h` (later
  pause/unpause cycles reuse the stored offset and do not jump).

**Substep pose drive.** Driven buildings advance on the substep clock
(`tSub = g.time − step + (i+1)*h`, ending exactly on `g.time`), not once per tick:
a 14 m/tick teleport at 10× bounced the gear and let a parked ship slide. The
per-tick drive runs only when paused or at rails warp (no substeps there).

**Spawn attitude (user correction):** ships stand **vertical** on the platform
like the space pad, not horizontal like a runway — the proposal's reuse of the
RUNWAY line was wrong for a launchpad (and tipped tall rockets). A per-ship
orientation heuristic (plane → horizontal, rocket → vertical) is **#851**.

**Gameplay note (not a bug):** the skiff is jet-powered and Venus's air has no
oxidizer, so its jet makes zero thrust there (#630's gate) — it cannot taxi or
climb from the platform under jets. A rocket (e.g. the user's
`mars_explorer_rcstest3`) thrusts fine; departure for a jet is RCS or a drop.

**Adversarial review fixes folded in:** tall-ship engagement (AABB-scaled ray,
above); RCS translation releases the carry for ships too (not just EVA); a
frame-change / out-of-world guard drops the carry; `heading_deg`/`height_m` now
finite-checked (a NaN heading was a NaN pose); `findPadOccupant` air grid widened
to the 40 m circumradius; stale comments corrected.

**Verified:** `make test` green (new `test_buildingair`, air parse/validation in
`test_buildings`); e2e 858–862 (spawn rests, drifts at warp, paused-present,
parked-ship-keeps-deck, tall-ship-rests) + orbital 854–857 + runway 850–853 + pad
160/461; geared skiff, gearless hull, kerbal and a tall upright rocket
(`heavy_one`) all rest rigidly at 1× and across 1×↔10× switches and first
unpause; thrust releases the carry. Tickets filed: #849 (throttle semantics),
#850 (building frames), #851 (spawn orientation).
