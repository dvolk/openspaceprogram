# Orbital buildings — space docks & stations, staged plan

Status: **IMPLEMENTED (v1)** on top of master `757d0f24` (post-runway,
2026-11-14). Scope decisions came from a Q&A with the user; the representation
choice below is the load-bearing one and it was *reversed* by the user's
constraints (see §2). An adversarial review pass caught and corrected one
load-bearing assumption — the collision frame is **inertial**, not rotating
(§6.1). Two implementation deviations from the plan, both deliberate:
- **Static, not kinematic, colliders for v1.** A teleported kinematic body did
  not block the ship in this Bullet build; a repositioned mass-0 static body
  does, and is all "immovable scenery" needs. Kinematic (to impart the station's
  motion for docking) is filed as **#848**. See §6.2.
- **Pose is driven even while paused.** The pose/gate run outside the
  `time_accel != 0` tick gate, so a dock is present the instant a ship spawns
  beside it on a paused game (user-reported; pinned by e2e `flight/857`).
The orbital-map marker (§4 stage 0) is deferred — spawn-from-dock and the known
orbits cover v1 navigation. e2e: `flight/854..857`; unit: `test_buildingorbit`.

The goal, in one sentence: **predefined buildings that live in orbit — a space
dock you can launch a ship from, and stations you can fly to — that move on
their orbit, hold a chosen attitude (body-fixed / sky-fixed / spinning), and act
as immovable scenery a ship collides with, cheaply enough to one day scale to
cities.** Docking, multipart rotating stations, balloon bases and player
construction are explicitly follow-ups (§5), with the design keeping them open.

---

## 1. Where we are (what the plan builds on — all verified)

- **The frame tree already propagates orbits.** `Frame::UpdateOrbitRails(time)`
  (frame.h) derives each orbiting node's `pos`/`vel` from its epoch state with
  `propagateKepler` (orbit.h) — exact two-body for *any* dt, the same call that
  lets idle ships coast on rails. `railStateFromElements(a, e, arg_peri,
  true_anomaly, mu, …)` builds an epoch state in the body-rail convention
  (orbital plane XZ, normal +Y). **Orbital motion for a building is mostly free:
  reuse this propagation, don't write new orbital mechanics.**
- **Rails + proximity already do "park far / wake near."** An idle ship coasts
  on its conic with its body *out* of the Bullet world (`goOnRails`,
  vehicle.cpp:6906); `Game::updateProximity` (game.cpp:2036) wakes a railed ship
  when the active ship comes within `prox_fly_on` (default **2000 m**, cli.h:158)
  and caps warp below `kRailsWarp` while anything is live. **This is exactly the
  presence model an orbital building needs: a collider only when you're near.**
  (The wake's `switchFrames` re-home only drops a ship into the *rotating* frame
  if it is inside the atmosphere shell — see the frame correction in §6.1; at
  orbital altitude the ship stays inertial.)
- **Buildings are ALREADY immovable, mass-0 colliders.** `place_world_buildings`
  (buildings.cpp:285) does `create_body(m, shader, t, 0,0,0, 0)` — mass 0, a
  static body — and `setPosRot(sp->body, sp->origin(), buildingOrient(up,
  heading))`. A ship bumps off a pad; the pad never moves. **"Immovable scenery"
  is the existing building model, not a new one.** What's missing for orbit is
  only that the pose *changes with time* and the body should exist *only when
  you're near*.
- **Buildings are surface-attached today.** `WorldBuilding` (buildings.h) carries
  `lat_deg/lon_deg/height_m/heading_deg`; `StaticBuilding::origin()` is a point in
  the body's **rotating frame** (terrain.h:753). An orbit doesn't fit that shape
  — it needs orbital placement instead (§3).
- **Spawn already knows how to start a ship in orbit with velocity.** The start
  beds (vehicle.cpp:~590-705) compute circular speed by vis-viva, a position on
  the orbit, resolve the frame with `resolve_frame_by_soi`, and express position
  **and velocity** in it (subtracting the frame's co-rotation `GetStasisVelocity`).
  `inertial-orbit`/`high-orbit` are existing orbital beds. **Spawn-from-dock reuses
  this: the ship must inherit the dock's orbital velocity, not start at rest.**
- **Docking is `absorbShip` (parts merge), and a "station" is already a hull in
  the code's grain.** `Game::updateDocking` (game.cpp:2117) calls
  `a->absorbShip(b, …)`; the `--dock-test` fixture (docktest.h) builds its
  *station* as a `Vehicle` co-moving on a circular orbit. So ship↔station docking
  has a working precedent — but note that fixture station is a *dynamic* Vehicle,
  which is precisely what we do NOT want for a gameplay building (§2).
- **No kinematic-body support yet.** Nothing in src/ sets `CF_KINEMATIC_OBJECT`.
  A building that *moves along its orbit but cannot be pushed* needs a kinematic
  body (mass 0 + kinematic flag, driven by `setWorldTransform` each tick). That is
  a small, contained addition to the physics wrapper (§6.2).
- **Precision is not a concern.** Bullet is double-precision (btcommon.h) and
  coordinates are body-local (the frame tree carries the sun-relative offset
  separately as `root_pos`), so orbital radii (~1e6–1e7 m) are far inside double
  precision. No floating-origin work is needed for this.

## 2. The representation decision — and why it's NOT a Vehicle

The first instinct (and the runway report's author's) was "a station is a
rail-riding `Vehicle` hull — docking for free." **The user's constraints reverse
that**, and they're right:

> "I don't think I want space 'buildings' to have detailed physics… colliding
> with a space station shouldn't impart rotation… does it use RCS to stabilize?
> does it have fuel?… we might have lots of buildings later… like entire cities."

A `Vehicle` is a **dynamic** body with finite mass: hit it and it translates and
rotates, which is exactly the docking nightmare described (now the station needs
RCS, fuel, attitude control…). And per-part physics at city scale is expensive.

**Decision: an orbital building is an immovable, infinite-mass KINEMATIC collider
— the existing building model, extended to orbit.** Concretely:

- It is **not a Vehicle**: no control, no throttle/SAS/staging, no fuel, no RCS,
  no crew, not a selectable/flyable vessel, not counted among "your ships."
- Its pose is **driven by us** each tick (Kepler position + attitude law), never
  by forces. Mass 0 + kinematic flag ⇒ a ship colliding with it stops/bounces;
  the building's orbit and attitude are **untouched**. No stabilization problem.
- Its collider is **proximity-gated**: present in the Bullet world only when the
  active ship is within the wake radius; otherwise it's just a propagated pose +
  a map marker. **This is what makes cities affordable** — static/kinematic
  colliders don't integrate, and distant buildings cost essentially nothing.

What this keeps open (the user's follow-ups):
- **Docking** (§5): a port is a fixed frame on the building. Docking to an
  *immovable* station is **simpler** than ship↔ship (snap the ship to the port and
  let it follow the station), not harder. The building does not need to be a
  Vehicle to carry a port definition.
- **Multipart rotating stations** (§5): a static core + spinning hab is a couple
  of kinematic colliders (or one compound with a rotating subset) — a natural
  extension of "a building is kinematic geometry," not a rewrite.
- **Cities**: many cheap static colliders, presence-gated. The model scales.

## 3. Design decisions (the short list)

| Question | Decision |
|---|---|
| Representation | Immovable **kinematic** building (NOT a Vehicle). Mass 0, driven pose, proximity-gated collider. §2. |
| Placement | New **orbital** placement on `WorldBuilding`, mutually exclusive with lat/lon. The JSON may offer a simple **circular authoring form** (`alt_km`, `inc_deg`, `phase_deg`, plane) — that is *sugar*: it converts to a full epoch state (`railStateFromElements`) and the **engine propagates a full Kepler conic** (`propagateKepler`, elliptic/hyperbolic alike), so authoring full elements for an eccentric orbit just works. No separate "circular-only" engine path. |
| Attitude | `attitude.mode`: **body-fixed** (nadir-pointing, tracks the orbit like ISS), **sky-fixed** (constant in the inertial frame, like a telescope), **spin** (constant rate about a defined axis, on top of a base mode). All are `orient(t)` functions. |
| Collision shape | **Compound of convex pieces** (the vehicle compound already does this). A single convex hull is fine for a box dock / cross station; a **torus is NOT convex** and Bullet triangle-mesh shapes can't move, so a torus hab = a ring of convex boxes. §6.3. |
| Presence | Collider exists only within the proximity wake radius (reuse `prox_fly_on`); re-expressed in the approaching ship's frame. Far away: propagated pose + map marker only. |
| Spawn (space dock) | Dock carries `VAB_LAUNCH_PAD` + a new `ORBITAL` flag. Spawn at the dock's position **+ its orbital velocity** (reuse the bed machinery), at a **clear offset outside the hull**, with a dock-defined pose. §6.4/6.5. |
| Default site | The dock must NOT become `homeCrewPad`/the default launch site — keep it after `main_space_port` in fragment order, exactly as the runway did. Crew pool / recovery / save-preflight resolve the default, unchanged. |
| Scope (v1) | **Predefined** in system JSON (like the runway), **single rigid body** per building, **no docking / multipart / balloons / player construction**. Those are §5 follow-ups. |

## 4. The stages

Each stage lands on its own, in order. Stages 0–1 give a visible, correctly-moving
station before any physics; stage 2 adds collision; stage 3 makes it a launch site.

### Stage 0 — orbital placement + propagation + render + map marker (no physics)

1. Schema: extend `WorldBuilding` (buildings.h) with an optional orbital block
   (`orbit: {alt_km, inc_deg, phase_deg}` for v1 circular; leave room for full
   elements) and `attitude` (§3). Parse/validate in `parse_world_buildings`
   (buildings.cpp:163): orbital XOR surface placement, sane ranges. A new
   `ORBITAL` flag marks orbital instances.
2. A new `OrbitalBuilding` (or an orbital branch of `StaticBuilding`) that, each
   tick, derives its inertial position via `propagateKepler` from its epoch state
   and its orientation from the attitude law (stage 1; identity for now).
   **Propagation runs on the frame-tree clock every tick at any warp** — it must
   not hang off the Bullet substep loop, which doesn't run at rails warp (§6.7).
   **Invariant: pose AND attitude are pure functions of `g.time`** (`phase0 +
   rate*t`, never an accumulated per-tick phase) so a save/load or warp hop is
   deterministic — the frame tree already models exactly this (frame.cpp:110-122),
   and buildings are re-instantiated from the system on load, not persisted (§6.13).
3. Render the mesh at the propagated pose. **The draw must be frame-aware:**
   `StaticBuilding::Draw` passes no frame xform (terrain.cpp:75), so a pad's
   rotating-frame coords are read directly in the render frame (`ship->frame`) —
   from an inertial ship that is already spin-stale for surface pads today. An
   orbital building must pass the building-frame → render-frame transform through
   `DrawModelAt`'s `xform` param (body.h:27-30), and its shadow/sunlight (computed
   through the rotating frame, terrain.cpp:67-73) needs the same care. Add an
   **orbital map marker** (the surface map counts `VAB_LAUNCH_PAD` pads,
   surfmap.cpp:111 — the orbital view needs its own marker, not a surface pin).
4. No collider yet. Add a sample station to a system fragment.

**Accept:** a station appears in orbit, advances along its conic (position at t=0
and t=T pinned by e2e against an independent `propagateKepler`), shows on the
orbital map; `make test` green. 1 e2e canary.

### Stage 1 — attitude modes + spin

1. `orient(t)` for **body-fixed** (one axis to the body centre, one along velocity
   — the local-vertical/local-horizontal frame), **sky-fixed** (constant inertial
   orientation), and **spin** (base mode + constant rate about an authored axis).
   Pure functions in a small header (the gimbal.h/wheel.h pattern), unit-tested.
2. Wire the station's orientation to the chosen mode; spin phase advances on the
   same clock as the orbit.

**Accept:** unit tests pin each mode's orientation over time (body-fixed stays
nadir-pointing through an orbit; sky-fixed stays inertially constant; spin advances
at the authored rate); e2e pins a spinning station's angle. 1–2 e2e.

### Stage 2 — proximity-gated kinematic collider (the physics)

1. **Kinematic body support** in the physics wrapper: a mass-0 body with
   `CF_KINEMATIC_OBJECT` + `DISABLE_DEACTIVATION`, driven by `setWorldTransform`
   (and interpolation transform) each tick. Small, contained (physics.cpp/.h,
   body.h). This is the one new physics primitive.
2. **Compound-convex collision** for the building (a single convex for v1 simple
   shapes; the torus-as-ring-of-boxes path noted for later).
3. **Presence gating:** when the active ship is within the wake radius, add the
   building's kinematic body to the world at the propagated pose; remove it when
   the ship leaves (reuse `AddBody`/`RemoveBody`, physics.cpp:387-400, the same
   pair `goOnRails`/`leaveRails` use). **Express the orbital collider in the
   body's INERTIAL frame** (§6.1): an orbiting ship integrates inertial, and there
   is ONE shared Bullet world, so a rotating-frame station would sit at a
   spin-angle-displaced "ghost" position to the approaching ship. Surface pads
   stay rotating; the two regimes split at the atmosphere shell. Still verify the
   collision empirically (fly into it) — but inertial is the answer, not a guess.
4. Collision behaviour: ship stops/bounces; building orbit + attitude unchanged.
   Keep the pose write on the double-precision `setPosRot` path (the float
   `create_body`/registration args are 0,0,0 placeholders — §6.12).

**Accept:** e2e flies a ship into the station and asserts (a) the ship's relative
velocity is arrested/deflected, (b) the station's orbital elements and attitude
are *unchanged* by the impact; a far station has no collider (presence gate).
`make test` + 1–2 e2e. **This is the stage to soak and eyeball — ask the user.**

### Stage 3 — space dock as a VAB launch site

1. A dock instance carries `VAB_LAUNCH_PAD` + `ORBITAL`; `find_launch_site`
   (buildings.cpp:310) finds it with no new flag logic. Keep it after the default
   pad so `homeCrewPad`/crew-pool/recovery/save-preflight are unchanged. BUT this
   is a **genuinely new spawn branch, not just ordering** — the whole `on_pad`
   path is surface-shaped and must be bypassed for an orbital site:
   - `StaticBuilding::origin()` reads **terrain height** along `dir`
     (terrain.cpp:58-62) — meaningless in orbit; the orbital branch uses the
     propagated pose instead.
   - `place_vehicle_def` derives pose from `site->dir` + `buildingOrient` + a
     support margin read off the site hull (vehicle.cpp:897-935) — all surface
     assumptions; the orbital branch sets pose + velocity directly.
   - The VAB site picker only appears for `on_pad` scenarios at all
     (gameui.cpp:6117) — an orbital dock needs the picker reachable for an orbital
     start, not a pad start.
   - `findPadOccupant` (pick.cpp:226-279, called vab.cpp:729) raycasts a deck box
     from `site->origin()` in the rotating frame — for a dock it would probe the
     terrain point *under* the dock. The one-ship-per-pad rule needs an orbital
     analogue or an exemption.
2. **Orbital spawn:** reuse the bed machinery to place the ship at the dock's
   position **with the dock's orbital velocity**, at a **clear offset outside the
   hull** (a dock-mouth spawn point authored on the instance), with a dock-defined
   pose, expressed in the **inertial frame** (§6.1). The ship inherits the dock's
   velocity and stays alongside it. (Same velocity at a 50 m offset is a *slightly*
   different conic, so there is a slow relative drift — fine for a spawn; true
   station-keeping, placing the ship on the identical conic, is a follow-up.)
3. **Suppress the surface-map pin** for orbital pads (gameui.cpp:2162-2167 maps
   `p->dir`, default (0,1,0) → a bogus north-pole pin; surfmap.cpp:111 counts it
   too) AND add the orbital-view marker. Recovery still returns crew to the home
   pad (v1).

**Accept:** e2e spawns a ship from the dock and asserts it stays co-orbiting with
the dock (relative distance bounded over time, i.e. it inherited the velocity);
`flight/160`/`461`/`850` pad+runway regressions still pass. 1–2 e2e.

### Stage 4 — corpus, docs, loose ends

- A sample **station** (sky-fixed telescope + a spinning cross) and a **space
  dock** in a system fragment; `res/ships`/buildings README law for authoring
  orbital instances; DESIGN.md tick-off; close any issues filed along the way.
- Save/load: predefined orbital buildings are part of the system (like terrain),
  so no per-game persistence is added — pin that it stays that way.

## 5. Intersections with the user's follow-ups (kept open, not built)

- **Buildings with docking ports.** A port = a fixed position+orientation on the
  building. Docking to an immovable station = snap the ship to the port and let it
  follow the station's pose (simpler than ship↔ship `absorbShip`). The building
  representation here does not preclude a later "ports vector" on buildings. Spin
  is NOT a blanket blocker: **a port ON the spin axis is dockable** — it sits on the
  axis so it does not translate, only rolls about its own normal; the ship matches
  the station's spin and approaches along the axis (the 2001 maneuver). It is the
  **off-axis** port, sweeping around in a circle, that is hard — that is the case a
  non-spinning core (multipart, below) solves. So an axial port on a single spinner
  is a legitimate v1+ docking target; no static core required for it.
- **Multipart buildings (rotating hab + static core).** A natural extension: a
  building becomes a small set of kinematic colliders (or one compound with a
  rotating subset), each with its own attitude law. v1 is a single rigid body;
  nothing here paints us into a corner.
- **In-atmo air-relative buildings (balloon bases).** A third placement mode
  (altitude + drift with the air) alongside surface and orbital. The placement
  model should stay an extensible "how is this building's pose derived" choice;
  v1 implements surface (existing) + orbital only.
- **Player construction / persistence.** A much larger feature (build in orbit,
  persist orbit/attitude/spin, likely an economy). v1 is predefined-only; if this
  ever lands, orbital buildings gain a saved state, which the schema should leave
  room for but not implement now.
- **Rendezvous/target synergy (free).** Because stations are propagated orbital
  entities, the existing target-orbit map view and rendezvous guidance can target
  them — a nice payoff worth confirming once stage 0 lands.

## 6. Risks and gotchas (the things that will bite)

1. **Frame consistency for collision (THE risk) — ANSWERED: inertial.** A ship
   and the building must share a frame to collide, and there is ONE global Bullet
   world (physics.h:47). The rotating frame's SOI is only the **atmosphere shell**,
   not the body SOI: `rf->soi = seaLevelRadius + shellEdge(atmoTop)`
   (system.cpp:1841, bodylimits.h:33-35) ≈ **R + 102 km on Kerbin**. `soiTarget`
   (vehicle.cpp:6800) only re-homes into that rotating child *inside* the shell;
   above it (≈ R + 112 km with `kSoiMargin`) an orbiting ship **stays inertial**
   and feels pure central gravity (fictitious accel is gated on `isRotFrame()`,
   vehicle.cpp:2894-2901). So **the orbital collider goes in the INERTIAL frame**;
   a rotating-frame station would be displaced by the body's spin angle × radius
   (hundreds of m/min at 400 km) — a ghost the ship misses or hits wrongly. Two
   traps that caused the original wrong guess: the `updateProximity` comment
   "re-express in the SOI's rotating frame" (game.cpp:2053) is only true *inside*
   the shell; and the `--dock-test` station uses `rot-orbit` ≈ 87 km
   (docktest.cpp:42), *inside* the shell, so that docking precedent says nothing
   about orbital altitude. Surface pads stay rotating; the regimes split at the
   shell. Still fly into it in stage 2 to confirm, but inertial is the answer.
2. **Kinematic vs static — resolved as static for v1 (#848).** The plan assumed a
   moving-but-immovable body must be kinematic. In practice a teleported
   `CF_KINEMATIC_OBJECT` body did NOT block the ship in this Bullet build (it
   passed straight through; `addRigidBody` files it under `StaticFilter` and into
   `m_nonStaticRigidBodies`, where the integrator interferes with the driven
   pose), while a repositioned mass-0 **static** body blocks correctly. v1 uses
   static — it is all "immovable scenery you bounce off" needs. `MakeKinematic`
   exists in the wrapper for the docking follow-up, which needs the station to
   impart its motion to an attached ship; that is #848.
3. **A torus is not convex.** Single-convex collision (the current building path)
   can't represent a torus, and Bullet triangle-mesh shapes can't move. Use a
   compound of convex pieces (ring of boxes) for a torus hab; v1 simple shapes
   (box dock, cross station) are fine as one convex.
4. **Orbital spawn MUST inherit velocity.** A ship spawned at the dock with ~0
   velocity immediately falls onto a different orbit. Reuse the bed machinery
   (vis-viva + frame expression), don't hand-roll. This is the orbital analogue of
   the runway's "spawn at rest height" — get it wrong and nothing works.
5. **Spawn "inside" is a trap.** Spawning inside a solid collider ejects the ship
   violently. v1 spawns at a **clear offset outside** the hull (a dock mouth).
   True-inside needs a hollow collider or a collision-grace window — follow-up.
6. **Default-site / crew-pool preservation.** The dock carries `VAB_LAUNCH_PAD`,
   which `homeCrewPad`, `find_launch_site`, the save preflight and the picker all
   read. Keep the dock after `main_space_port` so the default is unchanged (the
   runway already proved this pattern). Recovery returns crew to the home pad.
7. **Propagation must run at rails warp (and while paused).** The Bullet substep
   loop doesn't run at `>= kRailsWarp`; a building whose pose only advances there
   would freeze while the clock runs. Derive the building pose every tick,
   independent of Bullet — and outside the `time_accel != 0` gate, so a dock is
   present at a paused spawn too (implemented; e2e `flight/857`). Note: proximity
   caps warp below `kRailsWarp` only when a *ship* is live, so a lone railed ship
   can still coast through a station at high warp (rails ignores discrete
   colliders, as it does terrain) — an accepted edge, not a v1 blocker.
8. **Spin + docking** (§5): an **on-axis** port only rolls about its normal, so it
   is dockable by matching the spin (no static core needed); an **off-axis** port
   sweeps in a circle and is the hard case a non-spinning core solves. v1 spin is
   physical/visual; docking is deferred.
9. **Map markers: suppress the surface pin AND add an orbital marker.** An
   orbital pad carrying `VAB_LAUNCH_PAD` would draw a **bogus surface pin** at
   `p->dir` (default (0,1,0) → north pole, gameui.cpp:2162-2167) and be counted by
   surfmap.cpp:111. Filter orbital pads out of the surface pin/count path, and add
   an orbital-view marker. Both halves are needed.
10. **Performance / cities.** Presence-gating means only nearby buildings have
    colliders, and static/kinematic colliders don't integrate — so a city is
    affordable. Keep the per-tick propagation cheap (one `propagateKepler` per
    building) and avoid per-building heap churn in the hot path.
11. **Simplicity mandate.** The user explicitly wants code a local agent can read
    and modify (a prior EVA implementation was too clever). Keep the attitude laws
    as small pure functions, the kinematic addition contained, and comments
    explaining *why*. No premature abstraction for the multipart/docking future —
    just don't preclude it.
12. **Double-precision pose writes.** `create_body`/`RegisterObject` take *float*
    positions (body.h:109, physics.cpp:249); surface buildings pass 0,0,0 and
    overwrite via the double `setPosRot`. The orbital path must keep using the
    double setter — orbital radii in a float registration arg would lose metres.
13. **No persistence needed, but determinism is a constraint.** World buildings
    are re-instantiated from the system JSON on boot and on system install
    (main.cpp:662, game.cpp:1601), NOT saved; only the crew pool is saved
    (save.cpp:1705). The clock is restored on load (`g.setTime`, save.cpp:1699), so
    pose/attitude as pure `f(g.time)` (§4 stage 0) reloads exactly. If a future
    implementation accumulates spin phase per tick instead, loads and warp hops
    drift — keep it a pure function.
14. **Future docking inherits a same-frame gate.** `updateDocking` refuses when
    `b->frame != a->frame` (game.cpp:2138). A building-port docking path must put
    ship and station in one frame (the inertial frame, §6.1) — worth remembering
    when that follow-up lands, not a v1 blocker.

## 7. Verification commands (the working loop)

```
bear -- make                      # build
make test                         # unit tests (attitude laws, placement parse)
python3 e2e/run.py <new selectors> flight/160 flight/461 flight/850   # regressions
SDL_VIDEODRIVER=offscreen ./osp --timeout 30 --dbg-log --orbit-interval 1 \
  --startship probe,res/ships/<probe>.json,Kerbin,inertial-orbit       # fly to a station
```
(Orbital spawn-from-dock gets its own `--startship …,site=<dock>` path in stage 3.
Use `--prox-log` to watch the presence gate engage/release. Ask the user to eyeball
a rendezvous and a collision once stage 2 lands.)

## 8. Reflection — what would make this easier

- The building model is *already* immovable mass-0 scenery; the only thing missing
  for orbit is a time-varying pose and gated presence. A one-line note in
  buildings.h that "a building is a kinematic/static collider, never a dynamic
  body" would have made the representation decision immediate instead of a
  debate — and would stop a future agent from reaching for `Vehicle`.
- `StaticBuilding` is named/typed as surface-only (`dir`/`height_m`/`origin()` in
  the rotating frame). Splitting "a building's pose source" (surface vs orbital vs
  air) from "a building's collider/draw" would let orbital and balloon bases drop
  in without touching the surface path. Worth doing as part of stage 0, not before.
- The dock-test already builds a station as a dynamic `Vehicle`; that's fine for a
  docking fixture but is the *opposite* of the gameplay building model. A comment
  there distinguishing "fixture station (dynamic, for absorbShip tests)" from
  "gameplay station (immovable kinematic)" would prevent confusion.
- **A misleading comment actively caused the wrong frame guess here.**
  `updateProximity` says "Re-express the woken ship in its SOI's rotating frame"
  (game.cpp:2053), but that only holds *inside the atmosphere shell*; above it the
  ship stays inertial. The comment states a conditional as a law. Worth a one-line
  fix ("…rotating frame if inside the shell, else inertial") so the next reader
  doesn't repeat this report's original mistake. The deeper lesson: the
  rotating-vs-inertial split at the shell is load-bearing and under-documented —
  a short note in frame.h or constants.h naming the shell as the surface-frame
  boundary would pay for itself.
