# Part damage — design & staged plan

Date: 2026-09-26
Status: proposal (not implemented). What's-there / what's-missing / staged plan.
Scope order (per the brief): **collisions first**, then heat, radiation, random
failures.

## TL;DR

There is **no damage, health, heat, radiation, or failure model anywhere** —
zero domain hits for `damage|health|integrity|durability|heat|thermal|melting|
radiation|temperature|reentry` across `src/` and `res/`. Collisions are
handled entirely by Bullet's default solver with **no collision groups, no
contact callbacks, and no damage bookkeeping**. When two ships (or a ship and
the ground) hit, the solver pushes them apart (friction 4.0, default
restitution) and *that is the whole story*: no log line, no toast, no sound,
no state change, no destruction. A ship that lands hard simply comes to rest
and is parked on rails; nothing breaks, nothing is deleted, no event fires.

The deeper gap is **contact observability**: the game never reads the
magnitude of an impact. The only contact-aware code in the codebase is the
boolean `BodyInContact` (`physics.cpp:336-349`, used for the EVA grounded
latch), which discards the manifold points. So even a "damage on hard
landing" feature has no input to key off today — **the first real work is
measuring impacts**, then a damage law, then the failure modes built on it.

The encouraging part: almost every *hook* a damage system needs already
exists. The substep force loop (`tick.cpp`), the per-part aero walk
(`applyAeroForce`, which already computes `q = ½ρv²` — the exact input a
heating model wants), the compound-rebuild machinery (so a broken part *can*
be removed), the toast channel (the "PART BROKEN" surface), the permissive
save format (adding `hp` is backward-compatible by construction), and the
existing single-rigid-body + `extractSubtreeAsShip` model (so a broken part
can *become debris* with no new physics). What's missing is the data, the
measurement, and the laws.

---

## 1. What's there

### 1a. Collision handling (the whole of it)

Single global world, created once at boot (`main.cpp:265-266` →
`create_physics` `physics.cpp:146-148`); constructor at `physics.cpp:158-178`:

```cpp
collisionConfiguration = new btDefaultCollisionConfiguration();
dispatcher = new btCollisionDispatcher(collisionConfiguration);
overlappingPairCache = new btDbvtBroadphase();
solver = new btSequentialImpulseConstraintSolver;
dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
dynamicsWorld->setGravity(btVector3(0, 0, 0));
dynamicsWorld->setApplySpeculativeContactRestitution(true);   // :171
```

- Double precision (`BT_USE_DOUBLE_PRECISION`, `physics.cpp:1`).
- **No contact listener is installed anywhere** — `grep
  dispatchContactCallback|getContactManifolds` → zero hits. The only contact
  query is the boolean:

```cpp
// physics.cpp:336-349
struct AnyContactCallback : public btCollisionWorld::ContactResultCallback {
    bool any = false;
    btScalar addSingleResult(btManifoldPoint &mp, ...) { any = true; return 0; }
};
bool PhysicsEngine::BodyInContact(Body *b) {
    AnyContactCallback cb;
    dynamicsWorld->contactTest(b->btBody, cb);   // discards the manifold points
    return cb.any;
}
```
  used only for the EVA grounded check (`eva.cpp:53,60`). The debug drawer's
  contact hook is a TODO stub (`physics.cpp:70`).
- **No collision groups/masks** — `grep setCollisionFlags|collisionFilter`
  → zero hits. Every body collides with every other by default.
- Materials: `m_friction = 4.0` (`physics.cpp:288`), preserved across
  compound rebuilds (`vehicle.cpp:553,617`); EVA kerbals frictionless
  (`ships.cpp:129,193`). **No per-body restitution** is ever set.
- Hulls are convex hulls of part meshes (`BuildHull`, `physics.cpp:300-330`,
  default margin 0.1 m). Comment at `physics.cpp:306-309`: *"Bullet has no
  collision algorithm for concave-vs-concave pairs … anything that moves must
  stay convex."*
- Terrain: static `btBvhTriangleMeshShape`, one per LOD patch
  (`AddTerrainCollision`, `physics.cpp:227-251`).

### 1b. What happens today on an impact

- **Ship-vs-ship collisions are live and exercised** whenever two ships are
  within proximity radius (`updateProximity`, `game.cpp:1109`). The
  docking report documents this as a *designed-in failure mode*:
  `reports/docking2026_09_07/docking.md:99-111` — *"Ships collide with each
  other today … the contact solver pushes them apart (friction 4.0). There
  are no collision groups anywhere in the codebase … a slow, aligned approach
  can be captured before contact (capture radius 1.5 m ≫ 0.2 m), while a fast
  or misaligned one bounces off — a KSP-like failure mode for free,"* and
  `:285-296` — *"Keep ship–ship collisions as they are (no collision-group
  plumbing, and bumping is a feature)."* **A damage model formalizes exactly
  this behavior instead of leaving it to the solver.**
- **A ship is one rigid body** (compound of part hulls, `vehicle.h:156`), so
  there are **no per-part collisions within a ship** — `vehicle.cpp:447-449`:
  *"Bullet generates no contacts at all between the children of a compound."*
  A part cannot be individually hit while attached; only the whole ship's hull
  can. (This matters: "which part took the hit" must be inferred from the
  contact point, not read from a per-part contact.)
- **Docking is intent-driven and purely geometric** — it never reads Bullet
  contacts: `updateDocking` (`game.cpp:1191-1281`) checks face-center distance
  `< 1.5 m`, alignment `≥ cos 15°`, relative speed `≤ 2.0 m/s` (`game.h:75-77`),
  then `absorbShip` (merge) + `delete b` (`game.cpp:1276`). Called once per
  tick after the substeps (`tick.cpp:320`).
- **EVA**: the kerbal is a one-part `Vehicle` (`eva.h:36`) with a live dynamic
  hull in the same world; it collides with terrain/pads/ships via the default
  solver. Contact is used exactly two ways — the grounded latch
  (`eva.cpp:45-61`) and an analytic floor-snap guard (`eva.cpp:108-120`).
  **No knockback, no impulse readout, no suit damage.**
- **Dropped stages / debris are full live ships** (`Game::stage`,
  `game.cpp:1332-1435`, via `extractSubtreeAsShip` `vehicle.cpp:2244+`): they
  fly, collide via the default solver, come to rest, and are parked on rails.
  **When they hit the ground they just stop — nothing is destroyed, deleted,
  or reported.**
- The only ship-deletion paths are intentional: docking merge
  (`game.cpp:1276`), player "remove" (`game.cpp:1437+`), and item pickup
  (`game.cpp:1004`).

### 1c. Part model — no health field

`PartDef` (`shipdef.h:231-477`) and `Part` (`part.h:57-224`) carry **no HP,
durability, condition, temperature, or strength field.** Per-part state that
does exist: tank contents (`resources`), `stage`, `fuelGroup`, `armedThrust`
(transient), containment edges, authored pose. Behavior is field-derived
(`isThruster()` = `fuel_rate>0 && exhaust_velocity>0`, `part.h:143-146`), so
"a part that is broken" is a new derived state, not a new behavior category.

### 1d. Heat / atmosphere — the input already exists

- `drag.h` is pure math: `airDensity`, `dragForce`, **`dynamicPressure`
  (`q = ½ρv²`)** — the exact quantity a kinetic-heating model wants, and it is
  *already computed* in the aero path. `liftCurve`/`liftForce`, `partCd`,
  `jetThrust`. **No heating term, no temperature, no reentry code** (grep
  `reentry|heating|temperature` → 0 domain hits).
- `Vehicle::applyAeroForce` (`vehicle.cpp:1544`) walks parts per substep with
  `q`, `ρ`, `|v|` available and stores `lastAeroForce/lastDragRho/lastDragAlt`
  (`vehicle.h:411-421`) for `--drag-log`. **This is the per-substep hook where
  a per-part heating rate slots in** with no new inputs.
- `AtmosphereParams` (`terragen.h:78-95`): density + scale height only; no
  temperature profile.
- **Rails gap:** railed ships skip aero entirely (a known, documented gap —
  `reports/atmospheric-drag2026_09_11` §6). A reentry on rails warp would
  feel no drag *or* heat today; a heating model inherits that gap.

### 1e. Radiation, RNG, events, save, UI

- **Radiation: none.** Zero hits in `src/` and `res/`; no radiation fields in
  the systems JSONs.
- **RNG:** only `std::rand()` (camera shake, `render.cpp:84-85`) and one
  function-local `mt19937` (title-backdrop pick, `game.cpp:349`). **No
  gameplay RNG, no saved RNG state** (`SaveMeta`, `save.h:182-197`). A failure
  system would be the first seeded, saveable RNG.
- **Events:** `events.cpp/h` is SDL input dispatch, not a game-event system.
  The player-facing status channel is the **toast system** — `Game::toast`
  (`game.cpp:217-236`), bounded queue (3 s / 3 deep), mirrored to stdout as
  `[toast] …`, drawn at `gameui.cpp:2304-2335`. It already carries
  staging/docking/SoI messages — the natural "PART BROKEN" channel.
- **UI:** the Vessel window (`gameui.cpp:1563-1579`, ship-level rows) and the
  per-part window (`drawPartWindows`, `gameui.cpp:1718-1900+`, lists Mass,
  Size, Torque/Thrust, resources, docking, crew) are where a ship "Condition"
  row and a per-part condition bar would slot in.
- **Save:** per-part `SavePart` (`save.h:83-100`) with a **permissive reader**
  (`savePartFromJson`, `save.h:263-285`: "an absent/unknown key keeps the
  struct's default") — so **adding `hp`/`condition` keys is
  backward-compatible by construction.** Capture loop `save.cpp:205-227`;
  restore loop `buildShipFromSaveParts` `save.cpp:345+`.

### 1f. Tests — the collision path is untested

- **No e2e case drives two ships (or a ship and the ground) into each other
  and asserts an impact outcome.** The impact-adjacent cases are
  `02-staging-basic`, `07-spin-regression`, `26-eva-walk`, `27-eva-jump`,
  `39/40-prox-*`, `45/46/47-dock*`, `79-dock-save-load` — none asserts a
  collision.
- **Unit tests avoid the physics world by design** (e.g. `test_dock.cpp`:
  "no physics world and no GL context"); the closest is `test_dock`'s
  geometry invariants. A contact-impulse law would be the first test that
  needs a live Bullet world (cf. `test_thrust.cpp`, which already uses one).

---

## 2. What's missing

| Concern | Status |
|---|---|
| Contact observability (impact magnitude) | **Absent** — only boolean `BodyInContact` (`physics.cpp:336`); no listener, no impulse/penetration readout. |
| Per-part HP / condition state | **Absent** — `Part` (`part.h:57`), `PartDef` (`shipdef.h:231`), `SavePart` (`save.h:85`) have no such field. |
| A damage law (impact → damage) | **Absent** — nothing maps an impact to a state change. |
| Destruction / breakage path | **Absent** — staging/undock/drops *split* into live ships; nothing breaks or is deleted. |
| Collision groups / masks | **Absent** — all bodies default-group; ship-ship bumping is an unmodeled side effect. |
| Heat / reentry model | **Absent** — `q` is computed but never consumed for heat; no temperature, no melting. |
| Radiation model | **Absent** — no model, no data. |
| Gameplay RNG (reliability rolls) | **Absent** — only camera-shake `rand` + one title pick; no seeded/saved RNG. |
| Touchdown / crash detection | **Absent** — `BodyInContact` exists but ships never consult it (see `parachutes` report §1e). |
| Crew / kerbal injury | **Absent** — `character.md:307` proposed it ("gives parachutes stakes"); not built. |
| Damage UI (condition rows/bars) | **Absent** — no "Condition" in the Vessel or part windows. |
| Collision / damage tests | **Absent** — no e2e impact case; no unit test touches a Bullet contact. |

The load-bearing gap is the **first row**: without measuring impact magnitude,
every other row is speculation. The rest are data + law + UI on top of it.

---

## 3. The model

Three independent damage sources, all feeding one per-part condition value:

```
condition(p) ∈ [0, 1]      (1 = intact, 0 = destroyed)
  collision:  condition -= f(impact_impulse, part_strength)     [Stage 1-2]
  heat:       condition -= g(temperature, time)  if T > T_melt   [Stage 3]
  radiation:  condition -= h(dose_rate, shielding)               [Stage 4]
  random:     condition -= roll(reliability, dt)                 [Stage 5]
```

**Impact measurement.** After each `physics_tick(h)`, read the world's contact
manifolds (`dynamicsWorld->getContactManifolds()`), and for each manifold
involving a ship hull, sum `btManifoldPoint::m_normalImpulse` over its
contact points → the impulse delivered to that body this substep
(`N·s`). Map the manifold's contact position back to the *part* whose hull
contains it (nearest-part by contact point, since a ship is one compound
body — §1b) so the damage lands on the part that was hit, not the ship as a
whole. Expose `lastImpactImpulse` per `Body` for `--impact-log` telemetry.
This is a ~30-line addition to `physics.{h,cpp}` and the foundation for
everything else.

**Collision damage law (v1).** Per-part `strength` (J or N·s it can absorb),
`Part::condition` (0..1, default 1). On an impact with impulse `J` on part
`p`: `damage = max(0, J − threshold(p)) / strength(p)`, applied to
`condition`. At `condition <= 0` the part **breaks**: v1 makes it *inert*
(disable its thrust/torque/drag/lift, gray it out, fire a toast) — removing
its hull from the compound is possible (`rebuildCompound` exists) but is a
Stage-2 decision, because it changes COM/inertia mid-flight and interacts
with the fuel-group recompute.

**Heat law (Stage 3).** Per-part temperature, `dH/dt = η·q·v·A_p/(m_p·c)`
(S Stanton-number model) using the `q` `applyAeroForce` already computes;
`T` integrates per substep; above `T_melt` the part's `condition` drains.
This is the one source that is *rate-based* rather than event-based.

**Radiation (Stage 4) and random (Stage 5)** are rate-based drains gated on
body data / a seeded RNG respectively — cheap once the `condition` field and
its UI exist.

---

## 4. Staged plan

Ordered by the brief (collisions first). Each stage is independently
shippable; later stages assume the `condition` field from Stage 1.

### Stage 0 — contact observability (the foundation)

- `physics.{h,cpp}`: after each `physics_tick`, iterate
  `dynamicsWorld->getContactManifolds()`; for manifolds touching a ship hull,
  sum `m_normalImpulse` → `Body::lastImpactImpulse` (N·s) and record the
  contact position. New `--impact-log` CLI (`cli.{h,cpp}`, pattern:
  `--drag-log`) printing `[impact] J=… pos=…` per hit.
- `tests/test_impact.cpp` (first test with a live Bullet world, cf.
  `test_thrust.cpp`): two bodies, drive one into the other, assert
  `lastImpactImpulse > 0` and scales with closing speed.
- **Exit:** impact magnitude is *measured* and logged; `make test` green.
  Nothing else changes — this is pure observability.

### Stage 1 — per-part condition + collision damage

- `PartDef` (`shipdef.h`): `double strength;` (0 = invulnerable, default per
  part class). Load in `shipdef.cpp`.
- `Part` (`part.h`): `double condition = 1.0;` (next to `armedThrust`);
  derived `isBroken()` = `condition <= 0`.
- Damage application: in the per-tick boundary (where `updateDocking` runs,
  `tick.cpp:320`), for each ship with `lastImpactImpulse > 0`, map the
  contact point to a part and apply the §3 law. **Gate on relative
  closing speed** so slow resting contact (friction, docking approach) does
  not chip parts — only *impacts* do.
- Broken-part behavior (v1 = inert): `applyThrustForce`/`applyRotationForce`/
  `applyAeroForce`/lift all skip `isBroken()` parts; the part renders grayed
  (a `condition`-tinted or a "broken" variant); a toast fires once per break.
- `SavePart` (`save.h`): `double condition = 1.0;` + permissive read/write;
  restore in `save.cpp`. (`test_save` round-trip extended.)
- UI: a ship-level "Condition" row (min over parts) in the Vessel window
  (`gameui.cpp:1563`) and a per-part condition bar in `drawPartWindows`
  (`gameui.cpp:1718`).
- **Exit:** a ship driven into the ground (or another ship) hard enough to
  exceed a part's strength loses that part's function and shows a toast +
  condition bar; a slow landing does not. `make test` + `make e2e` green;
  new e2e `97-collision-damage.txt` (two ships, fast approach, assert a
  `[toast]` break + the part inert) and `98-collision-soft.txt` (slow
  approach, assert no break).

### Stage 2 — breakage → debris + crew injury

- **Broken part detaches** (optional upgrade from inert): on
  `condition <= 0`, `extractSubtreeAsShip` the part (and its child side) into
  debris, recompute fuel groups, rebuild the compound. This is the
  "engine falls off" outcome; it reuses the staging machinery verbatim.
  Decide the COM/inertia interaction before enabling.
- **Touchdown / crash detection:** per substep, `BodyInContact(ship->hull)`
  (the EVA pattern, `eva.cpp:53`) + closing speed → a landed/crashed state on
  the `Vehicle`. This is also the hook the `parachutes` report Stage 4 needs
  (retract on landing; crash if too fast).
- **Crew / kerbal injury** (`character.md:307`): impact impulse + G above a
  threshold injures the kerbal (a `Kerbal` HP field), giving parachutes and
  soft landings stakes.
- **Exit:** a hard landing destroys the ship and its crew; a chuted slow
  landing survives. e2e pair (fast vs. slow) asserting the two outcomes.

### Stage 3 — heat / reentry

- Per-part `temperature`; `dH/dt = η·q·v·A_p/(m_p·c)` in `applyAeroForce`
  (reusing its `q`), above `T_melt` drain `condition`. `--heat-log`.
- A reentry e2e (`99-reentry-heat.txt`): a ship entering an atmosphere at
  high speed sheds part condition over time; below a velocity it survives.
- **Note:** inherits the rails gap (railed ships feel no aero, so no heat) —
  document it, don't silently fix it here.
- **Exit:** reentry is a survivability decision, not a free pass.

### Stage 4 — radiation

- Per-body radiation field in the systems JSONs (`system.cpp` parse), a dose
  rate by distance/altitude, crew + part `condition` drain, shielding by
  intervening mass (a later refinement).
- **Exit:** a long stay on an irradiated body is costly.

### Stage 5 — random failures

- A **seeded, saveable gameplay RNG** (first of its kind): `SaveMeta` gains a
  `rng_seed`; a `Game::rng()` used for all stochastic rolls (so saves are
  reproducible).
- Per-part `reliability` (failures per hour); on a roll, trigger a failure
  mode chosen from the part's class (engine cut, tank leak, wheel loss, chute
  stuck) — each is just a `condition` or behavior flip, so they reuse the
  Stage-1 machinery.
- **Exit:** long missions degrade; a save/load reproduces the same rolls.

---

## 5. Open questions

1. **Inert vs. detach on break (Stage 1 vs 2).** v1 recommends *inert*
   (disable + gray out) because detaching changes COM/inertia mid-flight and
   interacts with fuel groups. Confirm we want that ordering.
2. **Impact threshold.** What closing speed / impulse separates "bump" from
   "damage"? The docking report's numbers (first contact ~0.2 m gap, 2 m/s
   capture gate) are the natural reference — a docking approach must *not*
   damage. Pin the threshold against `45-dock`.
3. **Which part takes a hit.** A ship is one compound body; the hit part is
   inferred from the contact point (nearest hull). Is that good enough, or do
   we want the *whole ship's* condition to drop (simpler, less precise)?
   Recommendation: nearest-part, with a ship-level "Condition" = min.
4. **Does a broken engine still consume fuel / produce thrust?** v1: no
   (inert = fully disabled). Decide whether a "damaged but running" tier is
   wanted.
5. **Rails + heat.** A railed reentry feels no drag *or* heat (known gap).
   Accept for v1 (document), or add a cheap analytic heating term to
   `railsTick`? Recommendation: accept + document.
6. **Radiation shielding.** v1 ignores shielding (open dose). Add
   mass-based shielding only if a design needs it.
7. **RNG reproducibility vs. surprise.** A seeded RNG makes saves
   reproducible but means two playthroughs from the same save diverge only by
   player action. Confirm that's the desired feel (KSP is deterministic; a
   "chaos" mode could reseed).

## Verification plan (per stage)

- Stage 0: `test_impact` (live Bullet world) pins impulse > 0 and speed
  scaling; `--impact-log` shows the hit; `make test` green.
- Stage 1: e2e `97-collision-damage` (hard, assert break + inert) +
  `98-collision-soft` (slow, assert none); `test_save` round-trips
  `condition`; Vessel/part-window condition check. `make test` + `make e2e`
  (incl. `45-dock` — docking must not damage) green.
- Stage 2: fast-vs-slow landing e2e pair; crew-injury check; debris persists.
- Stage 3: `99-reentry-heat` asserts condition drains above a velocity and
  holds below; `--heat-log`.
- Stage 4: a dose e2e on an irradiated body; crew HP drains.
- Stage 5: a seeded-failure e2e; save/load reproduces the roll.

---

*Snapshot + proposal, not implemented. Scope (in stage order):
`src/physics.{h,cpp}` (manifold read + `lastImpactImpulse`), `src/vehicle.*`
(broken-part gating, landed state), `src/shipdef.{h,cpp}` + `src/part.h`
(`strength`, `condition`), `src/save.{h,cpp}` (`condition`), `src/game.{h,cpp}`
(damage apply, toasts, RNG), `src/keys/cli` (`--impact-log`, `--heat-log`),
`src/gameui.cpp` (condition rows/bars), `res/data/parts.json` (per-part
`strength`), `res/systems/*.json` (radiation, Stage 4),
`tests/test_impact.cpp` (new), `e2e/cases/97…99-*.txt` (new). No
rendering-pipeline or network changes.*
