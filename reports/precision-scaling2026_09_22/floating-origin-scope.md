# Option A scope: floating origin for the physics sim

Date: 2026-09-22
Status: scoped, not started. Companion to precision-scaling.md (render-side fix, done — commit 4267963).

## Why

The render path is now exact at any distance, but the SIM still integrates
ships in absolute in-frame metres. At oort (1e15) the hull `btTransform`
quantizes to ~0.125 m and at interstellar (1e17) to ~22 m: thrusting
integration, contacts, docking and rails handoff all degrade even though
the picture looks right. Goal: keep Bullet-world coordinates small (< ~1e6 m)
near the active ship at ANY system scale.

## Design: per-frame physics offset at the Bullet boundary

`Frame::phys_offset` (dvec3, default 0). Contract:

    bullet_world_coord = frame_coord - phys_offset

- The offset is applied ONLY at the hull-transform read/write boundary
  (`setPosRot`, `placeShip`, `comPos`, `frameS`, `GetPosition`, velocity
  setters don't need it — velocities are differences). Everything that
  speaks frame coords (render, map view, conics, gravity, aero, EVA,
  switchFrames, save/load) is untouched and never learns the offset exists.
- **Rebase**: when the active ship's Bullet coords exceed a threshold
  (~1e6 m), `offset += delta` and every LIVE ship in the same frame gets its
  hull shifted by `-delta` (one `setWorldTransform` + broadphase refresh
  each; velocities unchanged — a pure translation). Railed ships: no-op
  (their state is frame coords, written through the boundary on wake).
  Terrain/pads: no-op — they live in ROTATING frames, which never get a
  nonzero offset (their coords are body-radius-scale anyway).
- Offset stays 0 for rotating frames and any frame nobody has flown far in.

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

- One shared Bullet world + per-frame offsets: two LIVE ships in DIFFERENT
  frames would have skewed relative geometry inside Bullet. True today
  already (they're in different frames), docking refuses cross-frame
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
- `Frame::phys_offset` + apply ±offset inside the stage-1 primitives when
  the body is a ship hull in that frame. Offset is never set nonzero yet.
- Verify: test + e2e; PHYSDBG numbers unchanged.

### Stage 3 — the rebase trigger (~1 session, the actual feature)
- In the tick, after `updateProximity` and BEFORE `UpdateOrbitRails`
  (epoch-ordering constraint, tick.cpp:222-253): if active ship is live and
  `|com_bullet| > 1e6`, rebase as described above. One log line per rebase.
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
- [ ] Staging / compound rebuild mid-offset-frame.
- [ ] Frozen grounded ships (rotating frame, offset 0): untouched.

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
