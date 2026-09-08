# Docking — scope and design

Snapshot of the docking scope as of 2026-09-07 (commit `4f2e387`,
"add proximity physics"). Written to plan docking support: two ships that
bring their docking ports together lock into one rigid ship, and can come
apart again. It records the ship model docking sits on (so the *why* of each
decision is visible), a concrete v1 design with the exact merge/undock
operations, the full bookkeeping list a merge touches, the test plan, and
the open questions.

All code references are `src/vehicle.h` (the ship class), `src/part.h` /
`src/shipdef.h` (the part model), `src/game.h` / `src/game.cpp` (the running
game), unless noted.

## TL;DR

Docking in this codebase is **not a physics problem**. A ship is a single
rigid body (`Vehicle::hull`, a compound of the part hulls) and a part is
geometry plus topology: an authored local pose in the ship frame and one
parent edge in the part tree. So docking is *move the target ship's parts
into the survivor's part list, rebase their poses with one rigid transform,
add one parent edge, rebuild the compound* — and undocking is the inverse.
No Bullet constraints, no new physics world, no per-part bodies. The real
work is the bookkeeping a merge touches (crew, fuel groups, stage counters,
fleet lists, part windows) and tuning the capture condition so the existing
ship–ship collision bounces a failed approach instead of breaking a good
one.

The one new concept is the **seam**: the parent edge that joins the two
ships at the dock. Everything else — fuel barriers, the EC pool, staging,
picking, rendering — is existing machinery pointed at the new tree.

---

## The model docking sits on

### One rigid body per ship

A ship is a single `btRigidBody` whose shape is a `btCompoundShape` of the
part hulls (`Vehicle::hull`, `vehicle.h` "the ship as ONE rigid body"). A
`Part` owns a `Body` with a render model, a collision hull and a mass, but
**no rigid body** (`body.h`: "A ship PART has no rigid body of its own").
A part's pose is never stored in the world — it is *derived*: the hull's
COM transform, mapped back through `principal` into frame S (the root
part's frame at build time), then through the part's authored
`localPos` / `localRot` (`partWorldPose`, `vehicle.h`).

Consequences that make docking cheap:

- **Rebuilding is the only "weld" operation.** `rebuildCompound()` rebuilds
  the compound + the rigid body from the current part list, carrying frame
  S and the velocity across. Staging, runtime spawn and burn-triggered
  mass refresh all just call it. Docking is the same call with a different
  part list.
- **The rebuild is self-checking.** `checkCompoundInvariants()` recomputes
  the mass properties and every child pose independently and asserts the
  compound reproduces the part assembly. A merge with a botched rebase
  (transposed rotation, wrong origin) fails here, in the tests and in the
  game.
- **`placeShip` / `placeShipAtCom`** put the whole ship with one write;
  every part follows from its authored local pose.

### The part tree is the topology

`Part::parent` is the authoritative topology ("This is the adjacency
droppedPartsAtStage and buildFuelGroups walk, so it is the authoritative
source for the tree", `part.h`). It drives:

- **Staging** — `separateStage` (`vehicle.h`) drops a decoupler plus its
  child-side subtree. Crucially it **deletes** the dropped parts: they do
  not become a ship. There is no "extract a subtree into a new Vehicle"
  primitive anywhere yet.
- **Fuel groups** — `buildFuelGroups` is a connected-component walk of the
  tree with `fuel_barrier` parts (decouplers) as walls; one-way
  `fuelLinks` bridge groups (a DAG, cycles rejected in `build_ship`).
- **Fuel drain** — layered, pro-rata across the group
  (`fuelDrainLayers` / `consumeResourceMass`).

### Face-snap math already exists

`attachPose` (`shipdef.cpp`) computes the child pose for the four attach
modes (down/up/radial/side) from the part radii and heights, with anchors
that must coincide — the comment says it is "the same function the future
VAB snap uses". Docking reuses its conventions: parts meet face-to-face,
the joint sits on the shared face, and the geometry is fully determined by
the part sizes.

### Ships are already proximity-aware

`Game::updateProximity` (`game.cpp`) wakes ships parked on the rails when
they come within `--prox-fly-on` (2 km) / `--prox-ground-on` (10 m) of the
active ship, parks them again beyond the off radius, and caps the warp on a
close approach (`--prox-warp`). The state "two ships are both live in the
physics world, close together" is therefore the *normal* state near the
active ship — docking does not need to manufacture it.

### The one physics fact docking has to respect

One global `btDiscreteDynamicsWorld` (`physics.cpp`), and every ship hull
is a dynamic rigid body in it with the default collision group/mask.
**Ships collide with each other today**: hulls are convex hulls inflated by
the margin (default 0.1 m, `hull_margin()` in `physics.cpp`; the pad
placement assumes terrain 0.5 + hull 0.1, `vehicle.cpp`), so two ships'
hulls first touch with roughly a 0.2 m face gap, and the contact solver
pushes them apart (friction 4.0). There are no collision groups anywhere in
the codebase.

That is both a constraint and a feature: a slow, aligned approach can be
captured *before* contact (capture radius 1.5 m ≫ 0.2 m), while a fast or
misaligned one bounces off — a KSP-like failure mode for free.

### What does not exist

No docking code of any kind (the "dock" grep hits in `src/` are all
"viewport" / "supports" false positives), no dock part in
`res/parts.json`, and no ship def carrying one.

---

## Design (v1)

### A docking port is a part

- `PartDef.docking_port` (bool), beside `decoupler` / `fuel_barrier`
  (`shipdef.h`); `Part::isDockingPort()` beside `isDecoupler()` (`part.h`).
  Behavior stays derived from the def — the rule that lets new part kinds
  land by editing `parts.json` alone.
- Catalog: three parts (`docking_port`, `docking_port_r1.5`,
  `docking_port_r2.25`) matching the existing size ladder. Reuse existing
  geometry (the decoupler or adapter `.obj` + texture) — no new art.
  `fuel_barrier: true` on every one, exactly like the decouplers: the two
  sides of a dock stay separate fuel groups (the KSP default — docked ships
  don't share propellant without a line). The existing `fuel_link`
  mechanism is the clean v2 for transfer across the seam; note its drain
  model requires a DAG, so a two-way transfer needs care.
- **The port is the part's end face closest to the other ship.** For a part
  P and the other ship's port Q, the port normal is +Z if
  `dot(+Z, Q−P) > 0`, else −Z, and the port center is the corresponding
  face center (`partPos + partRot * (0,0,±h/2)`). No face field in the
  data model; nose and tail ports both work, and a port attached
  mid-stack works. Radial (side-face) ports are a follow-up.
- Ship defs are unchanged by all this — a port is just another part,
  attached like any other (`down`/`up`/`radial`). Two test ships carry
  them (a "station" and a "probe"; see Testing).

### What counts as a dock

Checked once per tick, at a tick boundary — the same discipline staging has
("call at a tick boundary, not mid-substep"). Home: a new
`Game::updateDocking()` called in `tick()` next to `updateProximity()`
(after the substep loop, so both ships' poses are settled).

For each unordered pair of ships that are both **live** (not `onRails`,
not `isEva()`, not `isCrewAboard()`) and in the **same frame** (else toast
— a frame mismatch here means an SOI boundary between two ships a metre
apart, which is a bug, not a case), take the best qualifying port pair:

| condition | v1 value |
|---|---|
| port-center distance | < 1.5 m |
| each port axis at the other port | `dot(axis, dir) > cos 15°` |
| relative port-point velocity (`partVel` of the two ports) | < 2 m/s |

Both ships need a port part in v1 (the "one-sided" variant — active ship
portless, target ported — only needs the survivor rule generalized; noted
under Open questions).

On success: **survivor = the active ship** (the one the player is flying;
`g.ship` keeps pointing at it, so no control handoff, the camera simply
follows a shifted COM). The merge below, a toast, and one log line the e2e
cases grep:

```
[dock] t=123.4 a="probe" b="station" d=0.82 m v=0.94 m/s
```

### The merge (the core primitive)

`A` = survivor, `B` = target, `P_A` = A's port part, `R_B` = B's root
(`rootPart()`). All poses read through the existing accessors
(`frameS`, `partPos`, `GetVelocity`, `GetAngVelocity`).

**1. Rigid rebase.** Every part of B gets a new authored pose in A's frame
S. With A's frame S at `(p_A, R_A)` and B's at `(p_B, R_B)` (world
coordinates, both ships live in the same frame), the uniform transform is

```
T_rot  = R_Aᵀ · R_B
T_pos  = R_Aᵀ · (p_B − p_A)
localPos'(q)  = T_rot · localPos(q) + T_pos      for every q in B
localRot'(q)  = T_rot · localRot(q)
```

One rigid transform, exact — no per-part approximation. World poses are
unchanged by construction: `p_A + R_A·localPos'(q) == p_B + R_B·localPos(q)`.

**2. Topology: reparent B's root under A's port.** `R_B->parent = P_A`.
That is the *only* edge change. B's internal tree is untouched, so the
merged ship is still a single rooted tree with A's root as the unique
parentless part — `rootPart()`, `buildFuelGroups`, `droppedPartsAtStage`
and every other parent-walk keep working unmodified. (Reparenting B's
*port* part instead would leave B's root parentless — two disconnected
trees in one Vehicle — wrong. And the split result is identical either
way, since the whole of B is on the far side of the joint regardless of
where P_B sits in B.)

**3. Record the seam** (for undock): `A->seams.push_back({P_A, R_B,
B->name})` — the two parts of the joint plus the target's name, so an
undock can hand the extracted ship its original name back.

**4. Bookkeeping** — the part that actually takes time; a merge touches
every per-ship subsystem:

| thing | what the merge does |
|---|---|
| compound | `A->rebuildCompound()` — `checkCompoundInvariants` re-pins the rebase |
| fuel groups | `A->buildFuelGroups()` — the port parts are barriers, so the groups stay exactly A's ∪ B's (no new flow across the dock) |
| fuel links | keep both ships' `fuelLinks` (endpoints are stable `Part*`); nothing crosses the seam in v1 |
| EC | **nothing** — `powerTick` sums gen/draw/batteries over `parts`, so the pools merge automatically |
| stage counters | `totalStages_` = max over all parts; `activeStage_ = max(a, b)` — see below |
| thrust/rot | `A->clearThrust(); A->clearRotCmd();` (the split just happened, same rule as staging) |
| crew | for each `Kerbal *k` in `B->crew`: `k->aboard = A; k->aboardPart = A_old_parts_size + k->aboardPart;` then move the entries into `A->crew` (`aboardPart` is an index into `aboard->parts`, `eva.h` — append order makes the rebase an offset) |
| part windows | `g.dropPartWindowsFor(B)` (its `PartSel` entries would dangle) |
| fleet lists | remove B from `m_parent->ships` (its `Vehicle` is then deleted; the dtor unregisters the hull — B is live, not railed) |
| game pointers | `g.kerbal` / `g.lastShip` → null if they pointed at B |
| name/def | survivor keeps its name; `defPath` semantics — see Open questions |

**5. Velocity.** Perfectly inelastic — the physically correct merge:

```
v_merged = (m_A·v_A + m_B·v_B) / (m_A + m_B)     (COM velocities, same frame)
ω_merged = ω_A                                   (survivor's spin)
```

`rebuildCompound` carries the old velocity across, so `SetVelocity(hull,
v_merged)` after the rebuild. Angular momentum of B's parts about A's new
COM is dropped — negligible at these masses and separations, and it is
the same approximation class as staging (which drops everything).

### The stage-counter rule (why `max`, not `min`)

`activeStage_` is not just a readout — `GetActiveThrust` gates ignition on
`stage <= activeStage()`, and `separateStage` fires the decouplers *at*
the counter. After a merge, every part from both ships shares one counter.

- **`min(a, b)` is wrong.** An engine that was already lit on A has
  `stage <= a`; with the counter set to `min(a,b) < a` it reads as
  *unignited* and its thrust vanishes the moment it docks.
- **`max(a, b)` keeps every previously-lit engine lit** (a lit engine has
  `stage <= its ship's counter <= max`) — that's the rule.
- Known edge: a *remaining* decoupler whose stage falls below the merged
  counter (possible when the two ships dock mid-staging at different
  stages) can never fire — the counter has already passed it. Rare,
  invisible unless the player notices a stage that never drops; document
  it in the ship def comment rather than adding machinery.

### Undock

Key `U` (one-shot, like staging; `events.cpp` already has the pattern) →
pop the most recent seam `{P_A, R_B, name}`:

- **Dropped side** = `R_B` plus its child-side subtree — the exact
  `droppedPartsAtStage` walk, stage-free. It is, by construction of the
  merge, precisely the target ship's original parts.
- **Extract into a new Vehicle** — the first "subtree as ship" primitive
  in the codebase (staging stays cut+delete; it doesn't need this). It is
  the merge rebase inverted, so it is not much code:
  1. rebase the dropped side's poses into its own new frame S′ (the root's
     frame): `localPos′(q) = R_rootᵀ·(localPos(q) − localPos(root))`,
     `localRot′(q) = R_rootᵀ·localRot(q)`;
  2. new `Vehicle`: parts, `R_B->parent = nullptr`, the internal edges
     unchanged, `controller` = first wheel or root (the
     `controllerIndex()` convention), `init()` (fuel groups + compound),
     `placeShip` at the side's current world pose, `enterWorld()`;
  3. velocity of the extracted ship = the rigid-body velocity of its COM
     *before* the split (`v_A + ω_A × (com_B_world − com_A_world)`); the
     survivor keeps its COM velocity. Momentum is not exactly conserved
     (same approximation class as the merge);
  4. crew whose capsule part is in the dropped side moves to the new
     Vehicle (index rebase, in the other direction); the survivor's
     `rebuildCompound()`; the name comes from the seam record;
     `defPath = ""` (a runtime design — the spawn button can't reproduce
     it); a `[undock]` log line.

### Collisions and failure modes

Keep ship–ship collisions as they are (no collision-group plumbing, and
bumping is a feature). The numbers work: capture needs the port centers
within 1.5 m, and the hulls first touch at a ~0.2 m gap — so a slow,
aligned approach is captured before any contact, while a fast or tilted one
hits the hulls, the solver bounces them (relative velocity jumps past
2 m/s), and they fly apart with no dock. That is the KSP failure mode,
free.

If the bounce feels bad in practice, the follow-up is a **soft capture**:
a spring force between the two port centers inside the capture radius
(KSP's magnet), applied in the per-substep force pass. It is a
self-contained addition — the merge itself never changes.

### What does not need to change

- **Rendering** — `Vehicle::Draw` walks `parts` at derived poses.
- **Picking** — `pickShipPart` walks parts and the compound's children;
  `compoundParts` is rebuilt with the compound.
- **HUD / orbit / surface / transfer readouts** — all COM- or
  `GetVel()`-based; the COM shifts and they follow.
- **Ship list window, F6 cycling, proximity** — all walk the bodies'
  ship lists; the target is simply gone from them.
- **EVA / boarding** — the transitions read `aboard` / `aboardPart`, both
  rebased.

---

## Testing

### Unit — `tests/test_dock.cpp` (headless, `test_staging` style)

Hand-built ships with `body = nullptr` (the staging test's pattern — pure
graph/pose logic, no Bullet world, `onRails = true` to keep the dtor off
`RemoveBody`). Cases:

- **Rebase exactness**: after the merge, every part's *world* pose is
  unchanged (compose A's frame S with the new local poses and compare to
  the pre-merge world poses, to ~1e-12).
- **Topology**: the merged ship has one root (A's); `R_B`'s parent is
  `P_A`; B's internal edges intact; `droppedPartsAtStage`-style walks from
  `R_B` return exactly B's parts.
- **Fuel groups**: the groups after the merge are exactly A's ∪ B's (the
  seam parts are barriers); no group spans the seam.
- **Crew rebase**: a kerbal aboard B's capsule keeps the right capsule
  part after the merge (index + pointer).
- **Stage rule**: lit engines stay lit (the `min` counter regression);
  `totalStages_` = max; the stuck-decoupler edge documented.
- **Undock round trip**: undock after a dock restores the part partition,
  the world poses, and the tree shapes of both ships.

### E2E — cases 41–43 (the battery runs to 40 today)

A `--dock-test` hook, patterned on `--radial-test`
(`radialtest.h/.cpp`): builds two dock-capable ships straight from the
catalog in the same `rot-orbit`, nose-to-tail on the stack axis, at a
chosen separation — deterministic, no pad noise. (Both ships need the
catalog port parts, so the hook also exercises the new parse path.)

- **41-dock** — spawn 1.0 m apart, aligned, co-moving (relative velocity
  0): the condition holds on the first tick → auto-dock. `EXPECT [dock]`,
  fleet count −1 (grep the `Building ship` / active-ship lines), combined
  mass. `FORBID GL_`, `FORBID error:`.
- **42-dock-approach** — spawn 3 m apart (outside capture), arm throttle,
  thrust toward the target. The throttle detail: `R` adds 1% **per tick
  while held** (`adjustThrottle`, polled in `tick.cpp`), so a long hold
  saturates to full — the sim-press must hold R for ~2 ticks only (1–2%),
  then hold `I`. Numbers for a ~2,700 kg ship at ~50 kN full thrust:
  1% ≈ 0.19 m/s², closing 3 m gives v ≈ 1.1 m/s — inside the 2 m/s
  threshold with margin. Verify the zero/short-duration R press against
  the tick's polled-key path during implementation (the `isDown` OR-window
  in `tick.cpp` is the channel `--sim-press` feeds held keys through).
- **43-undock** — dock as in 41, press `U` (the staging-key pattern in
  `events.cpp`), expect `[undock]`, fleet count back to 2, both ships'
  masses and geometries intact.

`make test` gains the unit binary (a new line in the `test:` target,
linking like `test_staging`); `make e2e` picks up the cases automatically.

---

## Open questions

- **Momentum on undock.** Both sides get rigid-velocity-ish states that
  don't exactly conserve momentum. Acceptable for v1 (staging is a worse
  approximation and nobody notices); a momentum-conserving split is a
  bounded refinement.
- **Merged ship's `defPath`.** The spawn button
  (`ships.spawn_ship`, `gameui.cpp`) would re-spawn only the survivor's
  original design. Options: leave it (misleading), or blank it
  (test-ship semantics, spawn button disabled). Leaning blank.
- **One-sided docking** (only the target has a port). Generalizing the
  survivor rule ("the ported ship, preferring the active one") covers
  it; deferred because both-ported is the only case the test ships need.
- **Ground docking.** Allowed in v1 (no extra code); gate on altitude if
  pad-dock interactions turn out odd.
- **Radial ports.** Side-face ports need the face field the v1 rule
  avoids; follow-up.
- **Naming.** Survivor keeps its name (KSP renames the composite);
  cosmetic.

## Phasing

1. **Data model** — `docking_port` flag, catalog parts, two ship defs,
   parse tests (headless, `test_shipload` style). Small.
2. **Dock** — `updateDocking()` + the merge + `[dock]` line + e2e 41/42.
   The core; everything in the bookkeeping table.
3. **Undock** — seams + the extract-as-ship primitive + e2e 43.
4. **Later** — soft-capture magnet, fuel transfer across the seam (via
   `fuel_link`, mind the DAG rule), one-sided docking, HUD dock indicator
   (a port in range lights up), port visuals.
