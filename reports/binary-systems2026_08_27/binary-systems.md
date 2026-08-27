# Adding binary systems (planet-planet, moon-moon, planet-moon)

Date: 2026-08-27
Status: scoping / design report, no game code written. All code references
verified against the current tree; `binary_numbers.py` (this directory)
reproduces the §7 physics numbers.

## TL;DR

A "binary" (two comparable-mass bodies orbiting a common **barycenter**, with
that barycenter orbiting a parent) is **not expressible in the current data or
frame model**, which assumes every body orbits exactly one *fixed* parent on a
circle. The fix that fits this codebase (and matches the proven Pioneer
reference) is to add a **barycenter / gravpoint frame node**: a body with no
terrain that carries the pair's sphere-of-influence, with the two components as
its children orbiting it at the common pair angular rate.

The three "types" the request lists (planet-planet, moon-moon, planet-moon) are
**one mechanism** — the barycenter node — differing only in the members' `type`
(shader/palette) and where the barycenter sits in the hierarchy (under a star
vs. under a planet).

The one real design decision is **what gravity a ship feels while between the
two components** (inside the barycenter SOI, outside both members' SOIs):

- **Option A — combined-mass gravpoint (recommended first cut).** The barycenter
  body has `mass = mA + mB`; a ship in its SOI feels a single central pull with
  `mu = G(mA+mB)` toward the barycenter point. Drop-in: reuses `applyGravity`,
  the Kepler "rails", the HUD orbit fit, and SOI switching **unchanged**.
  Physically an approximation (there is no mass at the barycenter) but consistent
  with the existing SOI-as-nested-two-body-problems model, and exact in the
  far field.
- **Option B — restricted three-body (realism upgrade).** A ship in the barycenter
  SOI feels both components' pulls (`a = aA + aB`). Physically correct, yields
  Lagrange points and realistic transfers *between* the members — but breaks the
  single-`mu` assumption that `applyGravity`, `propagateKepler` (rails), and the
  HUD orbit fit all rest on, so those must be generalized or suspended in the
  binary region.

Recommendation: ship **Option A** end-to-end first (low risk, full compatibility
with the existing exact-coasting + HUD + SOI machinery), then layer **Option B**
on top as the realism pass.

Note: this is a *binary* (barycenter pair), which is a different configuration
from the existing **Eden L4 trojan** (KSP system), where two bodies share the
*same* orbit around the star at equal period. Both can coexist; the machinery
below is independent of the trojan.

## 1. What "binary" means here

Two bodies of comparable mass bound to each other, orbiting their common
center of mass (the **barycenter**); the barycenter itself orbits a parent
(star, or a planet for a moon pair). Distinguished by the members' `type`:

| pair            | members' `type`   | barycenter orbits |
|-----------------|-------------------|-------------------|
| planet-planet   | planet + planet   | a star            |
| moon-moon       | moon + moon       | a planet          |
| planet-moon     | planet + moon     | a star (or planet)|

Mechanically identical. When the barycenter falls **outside** the larger
member's surface (mass ratio high enough), it is a "double planet" (the
Pluto–Charon case); when inside, it reads as an ordinary large-moon system.

## 2. How the current architecture represents a system

Verified against the tree:

- **Frame tree = orbit hierarchy.** Every body gets an INERTIAL (non-rotating)
  frame plus a ROTATING (near-body) frame as its child (`src/main.cpp`
  `load_system`, pass 1). The parent/child wiring is the orbit chain
  (pass 2): `star → planet(inertial) → planet(rot) → moon(inertial) → ...`.
  `Frame.body` is a `TerrainBody*` and is **always non-null** today.
- **Motion is analytic circular "rails".** `Frame::UpdateOrbitRails`
  (`src/frame.cpp`) rotates each inertial frame's `pos` by
  `orb_ang_speed * timestep` about local +Y (constant radius, constant
  angular rate — no eccentricity), and each rotating frame spins by
  `fmod(rot_ang_speed*time, 2π)` about `spin_axis`. `UpdateRootRelative`
  composes: `root_pos = parent->root_orient * orient * pos + parent->root_pos`.
- **One fixed parent per body.** A body's `pos` is relative to exactly one
  parent, treated as a fixed central mass. There is **no barycenter concept**
  and no body that two others orbit.
- **SOI = nested two-body problems.** `Frame.soi` (inertial = true SOI;
  rotating = near-body = `radius + 100 km`). Per tick, `switchFrames()`
  (`src/main.cpp`) walks parent/children; `resolve_frame_by_soi()` picks the
  innermost SOI by `root_pos` distance. A ship is integrated in its SOI
  frame's coordinates.
- **Gravity is single-source.** `Vehicle::applyGravity()` applies only the
  SOI body's (`m_parent`) `1/r²` central pull (plus Coriolis+centrifugal
  fictitious terms when in a rotating frame). No companion gravity.
- **Exact coasting ("rails").** `propagateKepler()` (`src/orbit.h`) advances a
  coasting ship on an exact two-body conic (single `mu`) at any time-accel;
  `canRail`/`goOnRails`/`railsTick` park the Bullet bodies and re-derive pose
  from the conic. The HUD "ORBITAL" window and the 2-D "Orbital map" both
  fit a single conic from `(pos, vel, mu)` via `computeOrbitElements`.
- **Data.** `gen_systems.py` → `system.json` / `ksp_system.json`. Per body:
  `name, type, orbits, radius, mass, g, seed, has_sea, power_scaler, surface,
  moves, inertial{soi,pos,orb_ang_speed,orb_incl}, rotating{soi,
  rot_ang_speed,axial_tilt}`. `ksp_system.csv` carries an `Ecc.` column that
  the current (circular-only) model **ignores**.
- **Rendering.** `planets = sys.bodies`; each body's transform is
  `translate(GetPositionRelTo(ship->frame)) * rotFrame->orient`. Camera focus
  targets = the ship + every body (cycled with `G`). Spawn scenarios are all
  centered on the home body.

## 3. What is missing / breaks for a binary

1. **No barycenter node.** Two comparable bodies must orbit a point that is
   *not* either body. The tree has no body-less (or combined-mass) parent to
   hang them under.
2. **`Frame.body` assumed non-null.** Many sites dereference `frame->body->` /
   `m_parent->` (`mu`, `radius`, `g`, `mass`, `GetTerrainHeight`,
   `CountPatches`, `name` — see §9). A truly body-less barycenter frame would
   crash these. (Solution: give the barycenter a real `TerrainBody` with no
   terrain — the gravpoint — so every dereference stays valid, exactly as the
   star already gets a body.)
3. **Gravity between the members.** `applyGravity` only knows `m_parent`. With
   a gravpoint that is fine (Option A); for true three-body (Option B) it must
   sum the two members.
4. **Rails + HUD assume one `mu`.** Option B requires suspending/generalizing
   them in the binary region. Option A does not.

## 4. Proposed design: a barycenter (gravpoint) frame node

Insert one node between the parent and the two members:

```
            (Option A gravpoint)                 (members)
parent ── barycenter(inertial, SOI=pair) ── A(inertial) ── A(rot)
 (star)   mass = mA+mB, no terrain        └─ B(inertial) ── B(rot)
        soi encloses both; members on opposite sides, same ω
```

- The **barycenter** is a `TerrainBody` (so all `->body->` derefs work) with
  `mass = mA + mB`, `mu = G(mA+mB)`, **no terrain** (skip `Create()`), no
  atmosphere, and a `soi` that encloses both members. It behaves like the
  star in that it needs no meaningful rotating frame (a zero-spin dummy is
  fine and reuses the existing "star" path).
- The **members** are ordinary bodies (their own inertial + rotating frames,
  terrain, SOI), but their `orbits` field points at the **barycenter** instead
  of the parent. Each member's inertial `pos` is `(±r_i, 0, 0)` — opposite
  sides — with the **same** `orb_ang_speed = ω_pair`, so they stay 180° apart
  as `UpdateOrbitRails` rotates both about +Y.
- The barycenter's own `pos`/`orb_ang_speed` describe its orbit about the
  parent (star or planet), exactly as a planet's does today. The hierarchy
  composes through the existing `UpdateRootRelative`, so the members' inertial
  orbits remain fixed planes in space while the barycenter circles the parent.

### 4.1 Formulas (both members, rigid pair)

With separation `a = r_A + r_B` and total mass `M = mA + mB`:

- Member radii about the barycenter: `r_A = a·mB/M`, `r_B = a·mA/M`
  (barycenter offset from A's center is `r_A`; from B's, `r_B`).
- Pair period (Kepler III): `P = 2π·sqrt(a³ / (G·M))`; common rate
  `ω = 2π/P = sqrt(G·M / a³)`.
- "Double planet" test: barycenter is outside A's surface iff
  `r_A > radius_A`, i.e. `a·mB/M > radius_A`.

### 4.2 Data model (gen_systems.py + JSON)

Add a `type: "barycenter"` body. Members reference it via `orbits`:

```jsonc
{
  "name": "Kerbos", "type": "barycenter", "orbits": "Kerbol",
  "mass": 7.5e22,                      // = mA + mB; drives mu for Option A
  "radius": 1333000,                   // display/cull only; no terrain
  "inertial": { "soi": 4000000, "pos": [0,0,-2.0e10], "orb_ang_speed": 6.3e-7 }
  // no "rotating" section -> zero-spin dummy, like the star
}
{ "name": "Kerbin", "type": "planet", "orbits": "Kerbos",
  "radius": 600000, "mass": 5.0e22, ...
  "inertial": { "soi": 84159290, "pos": [ 667000, 0, 0], "orb_ang_speed": 7.9e-4 } },
{ "name": "Eden",   "type": "planet", "orbits": "Kerbos",
  "radius": 400000, "mass": 2.5e22, ...
  "inertial": { "soi": 40000000, "pos": [-1333000, 0, 0], "orb_ang_speed": 7.9e-4 } }
```

`load_system` changes:
- Recognize `type: "barycenter"`: create a `TerrainBody` **without** calling
  `Create()` (no patches/collision), set `mass`/`mu`/`soi`, mark it
  `no_terrain` so the render/`Update` loops and `focusTargets` can special-case
  it (or just let its empty patch list draw nothing).
- Everything else (frame wiring, SOI, calendar-from-rates, focus targets,
  `resolve_frame_by_soi`) already walks the tree generically and picks the
  barycenter up for free.
- **SOI sizing:** the barycenter `soi` must at minimum enclose the outer
  member (`r_B + radius_B`). Pioneer's proven bound is `max child orbital
  distance × 1.1` (`references/pioneer/src/Space.cpp:772`, `MakeFramesFor`) —
  a good default. Keep the members' SOIs nested inside with the existing
  ±10 km switch margins.

### 4.3 Gravity (the design decision)

- **Option A (gravpoint, recommended first cut).** Barycenter body mass
  `= mA+mB`. A ship in the barycenter SOI feels one central pull,
  `mu = G(mA+mB)`, toward the frame origin — `applyGravity`, `propagateKepler`,
  `computeOrbitElements`, and `switchFrames` all work **unchanged**. When the
  ship enters a member's nested SOI it switches to that member's real gravity
  (same discontinuity the SOI model already has going star→planet). This is
  exactly Pioneer's shipping model: patched conics, gravity from the current
  frame's body only, no three-body term
  (`references/pioneer/src/DynamicBody.cpp:192-230`, `CalcExternalForce`).
- **Option B (restricted three-body, realism).** While the ship's frame is the
  barycenter, `applyGravity` sums the pulls of **both** members (their
  positions in the ship frame come from `member->frame->GetPositionRelTo`).
  Consequences to accept: (a) a coasting ship in the binary region can no
  longer use the exact Kepler rails — `canRail()` must refuse there and the
  ship is always integrated (bounded cost; the region is finite); (b) the HUD
  "ORBITAL"/"Orbital map" single-conic fit is no longer valid — either hide it
  in the barycenter region or switch it to a short numerically-propagated
  trajectory (honest, and shows the interesting three-body path). This is what
  produces Lagrange points and realistic member-to-member transfers.

## 5. Realism considerations

- **Barycenter location & double-planet.** `r_A = a·mB/M`. High mass ratio puts
  the barycenter outside the primary (a genuine double planet); low ratio keeps
  it inside (reads as a big moon). Both are naturally produced by the same
  formula — choose member masses to taste.
- **Kepler III sets the pair period** from separation + total mass (§4.1).
  Separation and period are not independent: pick `a` (or `P`) and the other
  follows. This also bounds how "fast" the members sweep around each other.
- **Stability / Hill sphere.** The pair must sit well inside the parent's Hill
  sphere for the barycenter: `R_H = a_parent·(M/(3·M_parent))^{1/3}`; keep the
  separation `a` and the barycenter SOI comfortably below `~0.5·R_H` (prograde
  satellite stability limit). For a moon-moon pair, the same constraint applies
  at the moon level (pair inside the primary moon's Hill sphere about the
  planet). §7.4 has a worked check.
- **SOI nesting.** `member SOI < barycenter SOI < R_H`, with the existing
  `±10 km` switch margins. The barycenter SOI must be large enough that a ship
  transferring between members does not eject into the parent mid-burn.
- **Circular-only limitation.** The frame rails are circular (constant `ω`,
  constant radius). A realistic eccentric binary (e.g. Pluto–Charon e≈0.025, or
  a deliberately "hot" pair) is **not** representable today: `UpdateOrbitRails`
  has no eccentricity. The `ksp_system.csv` `Ecc.` column is already ignored
  for the same reason. v1 should be circular pairs; an eccentric pair needs the
  Kepler-solver treatment that `propagateKepler` already does for ships,
  generalized to drive a frame's `pos`. Pioneer does exactly that: its frames
  are driven by a Kepler solver (`OrbitalPosAtTime`, Newton–Raphson on the
  eccentric anomaly; `references/pioneer/src/Frame.cpp:481-497`,
  `src/Orbit.cpp:73,149`) rather than constant `ω`, so its binary pairs can be
  eccentric.
- **Placement of other bodies near the pair (data authoring).** A third body
  too close to a binary gets strongly perturbed; the authoring rule is to keep
  other planets' discs well outside the pair's orbit. Pioneer's rule of thumb:
  nothing inside `2.5×` the binary separation
  (`SAFE_DIST_FROM_BINARY = 5/2`,
  `references/pioneer/src/galaxy/StarSystemGenerator.cpp:24-25`; planet discs
  placed outside `orbMax × 2.5` at `:914-919`).
- **Inclination / mutual tilt.** Plane tilt is a single `orb_incl` (rotation
  about the parent's +X). Giving the barycenter and the members different
  `orb_incl` yields a limited (two-parameter) mutual inclination — enough for
  small tilts and consistent with the current model. Full 3D orientation (a
  general orbital plane, as Pioneer's `SetPlane` allows) is a later extension.
- **Tidal locking.** Members commonly end up spin-orbit locked to the pair.
  The `rotating.rot_ang_speed` is independent, so set a member's spin to
  `ω_pair` to model lock; the existing axial-tilt machinery then gives the
  right pole orientation.

## 6. Gameplay considerations

- **New destinations & transfers.** Each member is a landable/launchable world.
  Member-to-member transfer is the signature maneuver — cheap relative to a
  fresh interplanetary trip, analogous to Kerbin→Mun, and (with Option B)
  shaped by real three-body dynamics.
- **Lagrange points / co-orbit play (Option B).** With both pulls active,
  L1–L5 emerge naturally; a ship can park near a Lagrange point or shepherd
  between the members. A strong "why binary" hook.
- **Double-planet home.** One member can be `home`; the companion is a
  first-class destination with its own pad, orbit scenarios, and calendar.
  `sys.home`/spawn logic already key off a named body, so this is data-driven.
- **Calendar.** Each member already gets a calendar from its spin (day) and the
  barycenter's rate can supply the shared "year"; a tidally-locked member's
  day = the pair period is a nice touch.
- **UI / map / HUD.** Focus targets (`G` cycle) should include both members (and
  optionally the barycenter point). The 2-D "Orbital map" and "ORBITAL" window
  are single-conic; in the binary region either show the member-relative orbit
  (Option A) or a propagated path (Option B). The map could draw both members'
  circles about the barycenter.
- **Spawn scenarios.** `kScenarios` are home-centered; add a `companion-orbit`
  (orbit the *other* member) so a player can start the binary transfer cold.

## 7. Worked examples (numbers verified)

Reproducible: `binary_numbers.py` in this directory (stdlib only,
`python3 binary_numbers.py`) prints every number below.

Constants: `G = 6.674e-11`. `P = 2π√(a³/GM)`, `r_A = a·mB/M`, `r_B = a·mA/M`.

### 7.1 Planet-planet "double planet"
Primary `5.0e22 kg`, radius 600 km; companion `2.5e22 kg`, radius 400 km;
separation `a = 2000 km`.
- `M = 7.5e22`, `P ≈ 7940 s ≈ 132 min`, `ω ≈ 7.9e-4 rad/s`.
- `r_A ≈ 667 km`, `r_B ≈ 1333 km`. Barycenter is **667 km from the primary's
  center, outside its 600 km radius → double planet.**
- Barycenter SOI must exceed `r_B + radius_B ≈ 1733 km` (use ~4000 km).

### 7.2 Pluto–Charon analog (mass ratio 0.122, P ≈ 6.4 d)
Primary `4.5e21 kg` (Duna-like), companion `0.122×`. Matching `P = 6.39 d`
gives `a ≈ 13765 km`, `r_A ≈ 1497 km` (outside a 320 km radius → double
planet), `r_B ≈ 12268 km`.

### 7.3 Moon-moon pair
Primary moon `1.0e20 kg` (radius 200 km), companion `5.0e19 kg` (radius
120 km), `a = 500 km`.
- `M = 1.5e20`, `P ≈ 22200 s ≈ 6.2 h`.
- `r_A ≈ 167 km`, `r_B ≈ 333 km`. Barycenter inside the primary moon (ordinary
  binary-moon look). The pair itself orbits the planet (e.g. at ~20 000 km).

### 7.4 Stability check (planet-planet pair at Kerbin's distance)
Kerbol `1.757e28 kg`; pair at `a_parent = 1.36e10 m`, `M = 7.5e22`.
- Hill radius `R_H = 1.36e10·(7.5e22/(3·1.757e28))^{1/3} ≈ 153 000 km`.
- `0.5·R_H ≈ 76 500 km` — the `2000 km` separation is **~38× inside** the
  stability limit. Comfortable; the barycenter SOI (a few thousand km) is
  nowhere near the parent's. A moon-moon pair must be checked at the *moon*
  level instead (pair inside the primary moon's Hill sphere about the planet).

## 8. Test plan

- **Unit (pure math, like `tests/test_frames.cpp` / `test_orbit.cpp`):**
  - Barycenter geometry: given `(mA, mB, a)`, assert member `pos` are opposite
    and `|pos_A| = a·mB/M`, `|pos_B| = a·mA/M`; common `ω` from Kepler III.
  - Frame composition: after `UpdateOrbitRails`, member world positions stay
    180° apart and circle the barycenter; barycenter circles the parent.
  - SOI nesting: `resolve_frame_by_soi` returns the member when inside it, the
    barycenter between them, the parent outside.
  - Option B gravity: at a test point, `a_A + a_B` matches a direct two-source
    computation (and reduces to Option A in the far field).
- **e2e (Xvfb, `e2e/cases/`):** load a binary system; assert both members render
  and stay opposed across ticks; spawn on a member, fly to the barycenter
  region, cross into the other member's SOI, and confirm the frame-switch log
  lines; `--orbit-log` sanity in the binary region.

## 9. Implementation phases (scope, no time estimates)

1. **Gravpoint node + Option A (recommended first cut).**
   - `gen_systems.py`: emit a `barycenter` body + re-point two members at it.
   - `load_system`: handle `type:"barycenter"` (no terrain), keep everything
     else tree-generic. Guard the render/`Update`/focus loops for a
     no-terrain body.
   - Verify `switchFrames`/`resolve_frame_by_soi`/rails/HUD work unchanged with
     the combined-mass SOI body.
   - Unit + e2e per §8 (Option A subset).
   - *Blast radius:* mostly `gen_systems.py` + `load_system` + small render
     guards. `applyGravity`/`propagateKepler`/HUD untouched.
2. **Realism pass — Option B.**
   - Generalize `applyGravity` to sum both members when the ship's frame is a
     barycenter; store the member pair on the gravpoint body.
   - `canRail()` refuses in the barycenter region (always integrate there).
   - HUD "ORBITAL"/"Orbital map": member-relative (Option A display) or a
     propagated trajectory (Option B).
   - Unit test for the two-source gravity; e2e transfer between members.
3. **Extensions (optional, later).** Eccentric pairs (drive a frame's `pos`
   with the existing Kepler solver), full 3D mutual inclination, Lagrange-point
   markers/waypoints, binary-aware spawn scenarios.

## 10. Risks / open questions

- **Option A physical honesty.** Combined-mass-at-barycenter is an
  approximation; a ship *between* the members feels a pull toward a point that
  has no mass. Acceptable for v1 (consistent with the SOI model) but the
  "interesting" region is exactly where it is least accurate — Option B is the
  fix. Decide how early to invest in B.
- **Rails loss in Option B.** Coasting ships in the binary region lose exact
  Kepler coasting and are always integrated — a bounded but real perf/behavior
  change at high time-accel.
- **HUD semantics.** No single-conic orbit exists in a true binary; the HUD
  needs a defined behavior per region (member-relative vs. propagated path).
- **Circular-only.** Eccentric binaries are out of scope for v1 (frame rails
  are circular). Flag if a requested pair needs e > 0.
- **Data authoring.** Separation, period, and masses are coupled (Kepler III);
  `gen_systems.py` should solve for the missing one to avoid hand-tuned
  inconsistency. Also keep other bodies ≥ `2.5×` the pair's separation (§5) so
  the pair isn't strongly perturbing a neighbor.

## 11. Side findings (flagged per QWEN.md)

- **`moves` is dead.** Loaded into `body->moves` (`src/main.cpp:502`) but never
  read anywhere. Likely an early "does this body orbit" flag superseded by the
  frame tree. Candidate for removal (data + struct + doc line).
- **`sys.moon` is nearly dead.** Set at load and printed, but not used by any
  spawn/SOI logic. A binary "companion" is a more general concept; if a
  member-of-pair reference is ever needed, this is the slot to generalize
  (or drop).
- **`Ecc.` column unused.** `ksp_system.csv` carries eccentricity that the
  circular-only rails ignore — same limitation that blocks eccentric binaries.

## 12. Key code references

| concern | location |
|---|---|
| frame tree + SOI + load | `src/main.cpp` `load_system` (pass 1/2), `System` struct |
| frame struct (body, soi, pos, orient, rates) | `src/frame.h` |
| circular rails + spin + root-relative compose | `src/frame.cpp` `UpdateOrbitRails`, `UpdateRootRelative` |
| per-tick SOI switching | `src/main.cpp` `Vehicle::switchFrames`, `resolve_frame_by_soi` |
| single-source gravity + fictitious terms | `src/main.cpp` `Vehicle::applyGravity` |
| exact two-body coasting (rails) | `src/orbit.h` `propagateKepler`; `src/main.cpp` `canRail`/`goOnRails`/`railsTick` |
| orbit fit / HUD elements | `src/orbit.h` `computeOrbitElements`; `src/main.cpp` "ORBITAL" + "Orbital map" windows |
| render per-body transform | `src/main.cpp` `for(auto&& planet : planets)` draw block |
| camera focus targets | `src/main.cpp` `focusTargets` / `focusWorldPos` |
| spawn scenarios (home-centered) | `src/main.cpp` `kScenarios` / `spawn_vehicle` |
| data generation | `gen_systems.py` → `system.json`, `ksp_system.json`; `ksp_system.csv` |
| worked-example numbers (§7) | `binary_numbers.py` (this directory, stdlib only) |
| reference: binary data + GRAVPOINT concept | `references/pioneer/data/systems/custom/14_sirius.lua`; `src/lua/LuaConstants.cpp:133` (GRAVPOINT doc) |
| reference: barycenter math | `references/pioneer/src/Orbit.cpp:18` `OrbitalPeriodTwoBody`, `:289` `SetShapeAroundBarycentre`; `src/galaxy/StarSystemGenerator.cpp:1236` `MakeBinaryPair` (mass-ratio axes, 180° phase) |
| reference: per-tick propagation (eccentric-capable) | `references/pioneer/src/Frame.cpp:481-497`; `src/Orbit.cpp:73,149` (`OrbitalPosAtTime`, Newton–Raphson on the eccentric anomaly) |
| reference: frame/SOI sizing | `references/pioneer/src/Space.cpp:758-835` `MakeFramesFor`; gravpoint frame radius = max child distance × 1.1 (`:772`) |
| reference: ship gravity (current frame's body only) | `references/pioneer/src/DynamicBody.cpp:192-230` `CalcExternalForce` |
| reference: binary-safe distance for other bodies | `references/pioneer/src/galaxy/StarSystemGenerator.cpp:24-25` (`SAFE_DIST_FROM_BINARY = 5/2`), `:914-919` (planet discs outside `orbMax × 2.5`) |
| reference: binary simulation test | `references/pioneer/src/test/SimulationTests.cpp:95,159` (Gliese 852: "frame 0 is gravpoint") |
