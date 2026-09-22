# Option A scope: floating origin for the physics sim

Date: 2026-09-22
Status: scoped, not started. Companion to precision-scaling.md (render-side fix, done — commit 4267963).

## Why

The render path is now exact at any distance, but the SIM still integrates
ships in absolute in-frame metres. Note that for the Kerbol inertial frame,
in-frame coords ARE root coords (Kerbol is the tree root, root_pos = 0):
a ship between planets in the Kerbol SOI carries ~1e13-1e17 m into Bullet,
and a ship deep inside any big SOI carries that SOI's scale. At oort (1e15)
the hull `btTransform` quantizes to ~0.125 m and at interstellar (1e17) to
~22 m: thrusting integration, contacts, docking and rails handoff all
degrade even though the picture looks right. Goal: keep Bullet-world
coordinates small (< ~1e6 m) near the active ship at ANY system scale.

## Design: per-body physics offset at the Bullet boundary

`phys_offset` (dvec3, default 0), keyed **per body** — shared by the body's
inertial AND rotating frame nodes (a `Frame` accessor that walks to its
body). Contract:

    bullet_world_coord = frame_coord - phys_offset(body)

Per-body, not per-frame-node, because both nodes coexist in the one shared
Bullet world and must stay aligned: flying ships sit in the INERTIAL node
even inside the atmosphere (`inTerrainBand` only gates freezing,
vehicle.cpp:2719-2730), while terrain collision bodies and pads are
anchored in the ROTATING node's coords (which, per terrain.h:51-55 and
physics.cpp:230-262, the collision world treats as body-fixed — the spin
lives only in the render transform). An inertial-only offset would slide
the ship off the terrain.

- The offset is applied ONLY at the hull-transform read/write boundary
  (`setPosRot`, `placeShip`, `comPos`, `frameS`, `GetPosition`, velocity
  setters don't need it — velocities are differences). Everything that
  speaks frame coords (render, map view, conics, gravity, aero, EVA,
  switchFrames, save/load) is untouched and never learns the offset exists.
- **Rebase**: when the active ship's Bullet coords exceed a threshold,
  `offset += delta` and EVERYTHING anchored in that body shifts by
  `-delta` in Bullet storage: live ship hulls (one `setWorldTransform` +
  broadphase refresh each; velocities unchanged — a pure translation) and
  any loaded static bodies (terrain collision patches, pads/buildings).
  Statics must move too: body radii can exceed the threshold (Eve ~7e6 m),
  so a rebase can legitimately fire with terrain loaded. In practice
  terrain is only loaded near a surface, and near-surface coords of normal
  bodies stay below threshold, so the static-shift path is rare — but it
  must be correct. Railed ships: no-op (their state is frame coords,
  written through the boundary on wake).
- Offset stays 0 for bodies nobody has flown far from. The main non-zero
  case is exactly the between-planets one: the Kerbol inertial node while
  the active ship cruises far from the sun (in-frame coords there ARE root
  coords — Kerbol is the tree root). Gravity keeps working because
  `partPos`/`comPos` add the offset back — every force is computed from
  body-centred frame coords; only Bullet's internal storage (integrator,
  broadphase, solver) sees the shifted small coords.
- Caveat to verify in stage 2: if a rotating node's `orient` ever differs
  from its inertial parent's inside physics (spin applied to collision),
  the shared offset would need rotating by `GetOrientRelTo` per node. The
  current code says the collision world does not spin — confirm and
  document.

### Why not the alternatives

- **Shift world content / re-root frames**: gravity (`-b1b2 / r²`,
  vehicle.cpp:1373), aero altitude, EVA radial, `switchFrames`'s
  `length(com)` SOI test all assume frame origin = body centre. Moving the
  origin breaks every one of them.
- **Rails-first (old option B)**: can't do collisions/EVA/attitude at range.
- The offset design has one precedent to copy: `Vehicle::moveToFrame`
  (vehicle.cpp:2605) already teleports a hull + fixes velocity bookkeeping
  mid-tick without disturbing the shared world.

### Known wrinkles (accepted)

- One shared Bullet world + per-body offsets: two LIVE ships in DIFFERENT
  bodies' frames would have skewed relative geometry inside Bullet. True
  today already (they're in different frames), docking refuses cross-frame
  (game.cpp:961), and proximity converges live ships into one frame.
  Document, don't solve.
- Pick rays cast against hull transforms inside Bullet (pickShipChild):
  the ray must be built in offset coords. Caught in stage 4's audit.

## Implementation stages

Each stage is one small session, independently committable, with the full
gate (make test + e2e battery) at the end.

### Stage 0 — sim-side jitter instrument (~½ session)
- Env-gated (e.g. `PHYSDBG=1`) per-tick print of the active ship's hull
  transform + part-to-part relative deltas in Bullet coords; plus a
  headless measurement script (oort + interstellar, brief thrust via
  --sim-press) saving before-numbers to tmp/. This is the tape measure for
  every later stage. Expect ~0.125 m (oort) / ~22 m (interstellar) snaps.

### Stage 1 — choke the hull transform reads/writes (~1 session, pure refactor)
- Route every direct `getCenterOfMassTransform()` / raw hull write on SHIP
  hulls through the existing primitives (`comPos`, `frameS`, `GetPosition`,
  `setPosRot`, `placeShip`). ~14 sites in vehicle.cpp, plus stragglers in
  game.cpp (5), eva.cpp (1), pick.cpp (1), save.cpp (3), docktest.cpp (2),
  radialtest.cpp (1). Terrain/pad/static bodies are NOT ships — leave them.
- Zero behavior change; verified by test + e2e battery.

### Stage 2 — add the offset plumbing (~1 session, still zero behavior)
- `phys_offset` on TerrainBody (or Frame with a walk-to-body accessor) +
  apply ±offset inside the stage-1 primitives when the body is a ship hull
  in that frame, and in `AddTerrainCollision`/pad placement for statics.
  Confirm the rotating-node orientation caveat from the design section.
  Offset is never set nonzero yet.
- Verify: test + e2e; PHYSDBG numbers unchanged.

### Stage 3 — the rebase trigger (~1 session, the actual feature)
- In the tick, after `updateProximity` and BEFORE `UpdateOrbitRails`
  (epoch-ordering constraint, tick.cpp:222-253): if active ship is live and
  `|com_bullet| > threshold`, rebase: `offset += delta`, shift all LIVE
  ships in the body's frames by `-delta`, and shift any loaded statics
  (terrain patches, pads) the same way. One log line per rebase.
- Careful: do NOT use `placeShip` for the shift (its `proceedToTransform`
  zeroes velocities) — `setWorldTransform` + broadphase refresh +
  `activate()`, keeping velocities as-is; copy the broadphase dance from
  placeShip/AddBody.
- Verify: PHYSDBG at oort + interstellar now sub-mm near the ship;
  thrust/coast orbit unchanged to within integration tolerance
  (--orbit-log); e2e battery.

### Stage 4 — edge-case audit (~1 session, a checklist of small fixes)
- [ ] SOI switch into/out of an offset frame: `moveToFrame`,
      `railsSwitchFrames`, `moveToRailFrame` — frame-coord math is
      offset-free, but every hull pose write must land through stage-2
      primitives (should be automatic after stage 1).
- [ ] Proximity wake (`leaveRails` → `writeRailPose`): writes through the
      primitive → automatic; verify a woken ship lands at the right place
      in an offset frame.
- [ ] Docking merge (`absorbShip`): same-frame, both ships shifted equally
      → invisible; verify.
- [ ] EVA leave/enter (eva.cpp): controller part poses through
      `partWorldPose` → frameS → automatic; verify walk + re-enter.
- [ ] pickShipChild: build the Bullet ray in offset coords.
- [ ] GLDebugDrawer: reads raw bt transforms — subtract offset so the
      wireframe still wraps the ship (renderOrigin no longer cancels it).
- [ ] Save/load: format is frame-relative already (save.cpp:252) →
      transparent; verify save-at-oort round-trip keeps the same conic.
- [ ] Near-surface on a big-radius body (radius > threshold, Eve-like):
      rebase can fire with terrain loaded — verify contacts survive the
      static shift, or gate the threshold per body (e.g.
      max(1e6, 4*radius)) and document.
- [ ] Terrain LOD between rebases: patches added while offset ≠ 0 land at
      `anchor - offset` (AddTerrainCollision applies the current offset);
      collapsed patches removed cleanly.
- [ ] Staging / compound rebuild mid-offset-frame.
- [ ] Frozen grounded ships (rotating frame, that body's offset
      practically 0): untouched.

### Stage 5 — threshold, tuning + a regression scenario (~½ session)
- Threshold sanity: at 1 km/s a 1e6 m threshold rebases every ~17 min of
  flight; at 0.1c every ~10 s. A rebase is one setWorldTransform per live
  ship — cheap either way. Confirm, adjust, document the number.
- New e2e case: interstellar scenario + --sim-press thrust + --orbit-log
  tolerance assertion, so the property is regression-tested.

Total: ~4-5 sessions. Stages 0-2 are behavior-neutral and land safely even
if 3+ slips.

## Out of scope (deliberately)

- **Option D** (double-double for rails/conics): separate effort; needed
  for trajectory PREDICTION accuracy at 1e17, not for local physics.
- Re-rooting the frame tree / shifting celestial coords.
- Terrain or pad offsets (never needed).
- Render side (done, 4267963) and its sub-ULP residuals (pick ray origin,
  orbit→free pop) — those get fixed FOR FREE by stage 3 in the active
  ship's frame, since Bullet and cam.pos both become small; revisit the
  pick.cpp caveat comment afterwards.

## Success criteria

1. PHYSDBG at oort + interstellar: hull/part coords near the active ship
   stay < ~1e6 m regardless of absolute distance; part-to-part relative
   deltas smooth at double-ULP-of-1e6 (~1e-10 m) scale.
2. Thrust/coast trajectories match pre-rebase physics to integration
   tolerance across a rebase event (--orbit-log).
3. Full e2e battery + new interstellar regression case pass.
4. Save/load at oort round-trips bit-stable conics.
