# Fuel + staging — how it works and how to make it KSP-like

Snapshot of the fuel and staging model as of 2026-09-05, plus a proposed
refactor. Written to prepare the "dark work" of making fuel transfer and
staging behave like KSP: an engine should burn the propellant it is
*physically connected to* (its fuel group), not whatever happens to share a
stage number; decouplers should act as fuel barriers; and stage numbers should
control *ignition* and *drop* timing without dictating *which tanks feed which
engine*.

All code references are `src/vehicle.h` (the ship class) and `src/part.h` /
`src/shipdef.h` (the part model) unless noted.

## TL;DR

Today, **fuel, ignition, and drop are all keyed on the same thing: the
part's `stage` number.** An engine burns the tanks whose `stage` matches its
own (`consumeResourceMass(..., p->stage)`), it ignites when the stage counter
reaches its `stage` (`ApplyThrust`), and the ship's parts drop by decoupler
(`separateStage`). The first two are the problem: because fuel pools are
grouped by stage *number* rather than by *connection*, an engine drains the
wrong tanks (its neighbours' stage) and, once that stage's tanks are dropped,
it starves even though its own tanks are full and right next to it.

The fix is to **separate "fuel" from "stage."** Build *fuel groups* from the
part graph — a fuel group is a connected component of the part tree with the
decouplers removed (decouplers are fuel barriers, exactly like KSP). An engine
draws from its own fuel group's tanks, pro-rata, regardless of stage. Stage
numbers then do only two things: decide when an engine ignites, and (via the
decoupler's own stage) decide when a part drops. That one change makes
`heavy_two` behave the way you expected: the central engine burns the central
tanks it's attached to, and it keeps burning after the boosters fall off.

---

## The current model

### Parts, tanks, and resources

A ship is a tree of `Part`s (`vehicle.h`); each non-root part has exactly one
parent weld recorded in `constraintLinks` (parent → child), so the part graph
is a rooted tree. A `Part` is *behavior-driven by its `PartDef`* (`part.h:44-60`):

- `isThruster()` — `def->fuel_rate > 0 && def->exhaust_velocity > 0`
- `isTank()` — any `def->capacity[r] > 0`
- `isDecoupler()` — `def->decoupler`
- `isWheel()` — `def->torque > 0`

These are independent booleans, so a part could in principle be a tank *and*
an engine, though the catalog doesn't ship any such part.

Each tank part owns a `ResourceContent` (`shipdef.h:100-113`): per-resource
`current[]` and `capacity[]`, indexed by `ResourceType`
(Hydrogen, LOX, Hydrazine, Electric charge, Oxygen, Water, Food). Only tanks
are seeded — in `Vehicle::init()` (`vehicle.h:240-250`) each tank's
`capacity[r] = current[r] = def->capacity[r]`. Non-tank parts carry all-zero
resources. Fuel is a *mass on the part's rigid body*: consuming it also does
`p->body->mass -= take; SetMass(p->body, p->body->mass)` (`vehicle.h:290-291`),
so a drained tank literally loses mass and the ship's COM/inertia track it.

### The stage counter

`Vehicle` keeps a **monotonic stage counter**, `activeStage_`
(`vehicle.h:335-342`):

- starts at the **lowest** stage number on the ship (so a ship whose first
  engine is stage 2 still lifts off),
- advances by one on each SPACE via `advanceStage()` (clamped at
  `totalStages_` = the highest stage number at build time),
- `totalStages_` is only for the "stage X of N" HUD readout.

There are no per-stage fuel pools as a separate object; the pools are *defined
implicitly* by the `stage` field at consumption time (below).

### Fuel consumption — the crux

`consumeResourceMass(type, amt, stage)` (`vehicle.h:277-295`) is the only place
fuel moves. It:

1. sums `current[type]` over **every tank whose `p->stage == stage`**,
2. refuses if the stage's total is less than `amt`,
3. otherwise drains **pro-rata** across those tanks (each tank loses
   `amt * have / total`, clamped to what it holds).

The comment says it all: *"stages are self-contained: an engine burns its OWN
stage's propellant."* **That is the design decision that produces the
unexpected behavior.** The "pool" is not a physical thing; it is the set of
tanks that happen to carry the same `stage` number as the engine. There is no
notion of which tanks are *connected* to which engine, and no notion of a
decoupler blocking fuel. (I grepped `src/` for fuel-transfer / fuel-group /
"transfer" logic — the only "transfer" code is the orbital Lambert solver in
`transfer.h`/`transferplanner.cpp`, unrelated to propellant.)

### Ignition

`ApplyThrust(step)` (`vehicle.h:857-877`) runs once per physics tick. For each
thruster it:

- skips engines not yet ignited (`p->stage > activeStage_`),
- computes this tick's flow `flow = rate() * thruster_util * step`,
- calls `consumeResourceMass(Hydrogen, flow, p->stage)` **and**
  `consumeResourceMass(LOX, flow, p->stage)` — i.e. it draws from the tanks
  on the **engine's own stage**,
- if both succeed, arms the thruster's thrust for the tick.

So ignition is `stage <= counter` (an engine, once lit, stays lit as the
counter only ever increases), and **fuel source is the engine's stage number.**
Both are `stage`.

### Drop

`droppedPartsAtStage(stage)` (`vehicle.h:651-681`) returns the parts that would
fall when `stage` triggers: for each decoupler on that stage, the decoupler
itself **plus its child-side subtree** (BFS over `constraintLinks` from the
decoupler's children). `separateStage(stage)` (`vehicle.h:687+`) cuts the weld
to the parent, removes the dropped parts from Bullet and the ship, and deletes
them. Drop is **decoupler-driven**, keyed on the *decoupler's* stage — this
part is already the KSP shape we want (a decoupler on its stage drops the
parts below it). Note the drop logic never looks at engine/tank `stage`; it is
purely "decoupler on this stage → cut it, drop its child side."

So today the three axes are:

| axis            | keyed on                          | code                       |
|-----------------|-----------------------------------|----------------------------|
| fuel source     | engine's `stage` number           | `consumeResourceMass(..,stage)` |
| ignition        | engine's `stage` vs. counter      | `ApplyThrust`              |
| drop            | decoupler's `stage` vs. counter   | `droppedPartsAtStage`      |

The fuel row is the odd one out: it is keyed on the *engine's* stage rather
than on *connection*, and it does not treat decouplers as barriers.

---

## The two behaviors you saw (heavy_two)

`res/ships/heavy_two.json` is a 4-stage column. The spine (root → tip) is:

```
capsule_1(4) ─ rw1(4) ─ central_tank0(4) ─ central_engine0(4)
  ─ decupler_r1(3) ─ adapter1(3) ─ rw2(3) ─ central_tank1(3) ─ central_engine1(3)
  ─ decupler_r1.5(2) ─ adapter2(2) ─ rw3(2) ─ central_tank2(2)
        ├─ central_tank3(2) ─ central_engine2(2 → you set 1)
        ├─ rdecupler1(1) ─ rtank11(1) ─ rtank12(1) ─ rengine1(1)
        └─ rdecupler2(1) ─ rtank21(1) ─ rtank22(1) ─ rengine2(1)
```

The two radial boosters hang off `central_tank2` through their own radial
decouplers; everything central (tanks 0/1/2/3, engines 0/1/2, capsule, wheels)
is on the spine *above* the radial decouplers.

### (a) Stage-1 central engine drains the radial tanks

You set `central_engine2` to stage 1 so it fires at launch with the boosters.
It fires (stage 1 ≤ counter 1) but the **central tanks don't drain** — the
radial ones do. Root cause, line by line:

- `ApplyThrust` sees `central_engine2.stage == 1` and calls
  `consumeResourceMass(Hydrogen, flow, /*stage=*/1)`.
- That walks the parts and sums **only** tanks with `p->stage == 1` — which are
  `rtank11/12/21/22`, the booster tanks. `central_tank3` is stage 2, so it is
  skipped (`if(p->stage != stage) continue;`).
- So the central engine — physically welded to `central_tank3`, which is full —
  draws the propellant from the two tanks it is *not* attached to. The
  connection is simply not consulted anywhere.

The code is doing exactly what it was written to do ("an engine burns its own
stage's propellant"); the stage just happens to group the wrong tanks together
for this geometry.

### (b) After staging, central tanks full but engine dead

Press SPACE: counter 1 → 2. The stage-1 decouplers (`rdecupler1`, `rdecupler2`)
fire and drop their child side — the booster tanks and engines are gone from
the ship. Now:

- `central_engine2` is still lit (stage 1 ≤ 2, so ignition is fine) and still
  calls `consumeResourceMass(.., stage=1)`.
- But there are **no more stage-1 tanks** on the ship (they dropped with the
  boosters), so the stage-1 pool is empty, `total < amt`, and it returns false.
- `central_tank2/3` (stage 2) are full and sitting directly above the engine,
  but they are stage 2, so the stage-1 engine can never see them.

Your read — "it doesn't fire because it's on a previous stage" — is the
intuitive summary, but the precise cause is **fuel-pool starvation**: the
ignition rule *does* keep a lower-stage engine lit (`stage <= counter`), it's
that its stage's tanks are gone and its real tanks are a different stage. The
"dead engine" is a fuel problem wearing a stage costume.

Both symptoms are the same root: **fuel is allocated by stage number, not by
connection, and decouplers don't block anything.**

---

## The KSP reference model

KSP separates two concerns that this codebase currently fuses:

### Fuel groups (connection-based)

Parts are nodes in the part graph. Two parts can transfer a resource to each
other only if they are in the same **fuel group**, where a fuel group is a
connected component of the part graph **with the decouplers removed** (a
decoupler is a fuel barrier — propellant does not flow across it). In
practice:

- Tanks and the structural parts between them conduct fuel, so a tank, its
  engine, and the tanks stacked between them form one group.
- A decoupler splits the graph: the tanks above it and the tanks below it are
  different groups, and an engine on one side cannot drain the other.
- This is exactly your intuition: *"decouplers don't transfer fuel … only fuel
  tanks [and their connected neighbours] do."*

An engine draws its propellant **from its own fuel group**, pro-rata across the
group's tanks, **independent of stage.**

### Staging (timing-only)

Stage numbers control *time*, not *topology*:

- **Ignition:** an engine starts burning when the stage counter reaches its
  stage, and (as here) keeps burning while it has fuel.
- **Drop:** a part falls when the decoupler above/below it (the one whose child
  side it is on) triggers at its stage.

Crucially, *an engine's stage and its fuel group are independent axes.* That is
what lets a designer put an engine on an early stage (fire it at launch) while
feeding it from a tank that drops later — or, conversely, have an upper engine
keep burning off a lower tank until that tank's decoupler fires. The stage
number says *when*; the fuel group says *what*.

### Why that fixes heavy_two

Under the fuel-group model, with `central_engine2` left on its natural stage:

- **Fuel group of `central_engine2`** = {central_tank3, (central_tank2), ...} —
  the spine tanks, because the radial decouplers are *below* `central_tank2`
  and the spine has no decoupler between the engine and its tanks. It burns
  `central_tank3`/`central_tank2`, **not** the boosters. (a) fixed.
- **After SPACE** the boosters drop, but `central_engine2`'s fuel group (the
  spine tanks) is untouched — it is full, so the engine keeps burning. (b)
  fixed.
- **Boosters** each have their own fuel group {rtank11, rtank12} and
  {rtank21, rtank22} (split by their radial decouplers from the spine), so they
  burn their own tanks at launch.

And because stage is now *only* ignition timing, you can set
`central_engine2` to stage 1 to make it light up at launch without any fuel
strangeness — it will simply burn the spine tanks it's attached to.

---

## The gap, in one picture

```
                    TODAY                          KSP (target)
  ┌────────────────────────────────┐   ┌────────────────────────────────┐
  │  fuel source  = engine.stage   │   │  fuel source  = fuel group     │
  │  ignition     = engine.stage   │   │  ignition     = engine.stage   │
  │  drop         = decoupler.stage│   │  drop         = decoupler.stage│
  └────────────────────────────────┘   └────────────────────────────────┘
        fuel + ignition fused                fuel decoupled from stage
        on `stage`; decoupler                decoupler is a fuel barrier
        invisible to fuel
```

Only the **fuel source** row needs to change. Ignition and drop already have
the right shape (stage = when, decoupler = what falls). The work is to replace
"tanks matching my stage" with "tanks in my connected component (decouplers
removed)."

---

## Proposed refactor

A focused change, mostly local to `vehicle.h`. Sketch:

### 1. Build fuel groups at construction

Add a helper that, given the part tree and the decoupler set, returns the
fuel-group id of each part. A fuel group is a connected component of
`constraintLinks` after removing every edge that touches a decoupler (the
decoupler is a wall, so neither its parent side nor its child side can reach
through it). Concretely:

- Build the parent→child adjacency (same as `droppedPartsAtStage` already does).
- Do a BFS/union-find over the parts, **skipping any part that is a decoupler**
  (do not cross it, and it is not itself a fuel node). Tanks, engines, and
  structural parts all conduct; only decouplers are walls.
- Assign each conducting part an integer group id. Store it on the `Part`
  (e.g. `int fuelGroup`) or in a side map `Part* -> int`.

This runs once in `init()` and again after every `separateStage()` (the graph
shrinks when parts drop, so recompute — cheap, the ship is small).

### 2. Consume from the engine's fuel group

Change `consumeResourceMass(type, amt, stage)` to
`consumeResourceMass(type, amt, Part *engine)`:

- pool = the set of tanks `p` where `p->fuelGroup == engine->fuelGroup`
  (and `p->isTank()`),
- same pro-rata drain across the pool's tanks (keep the symmetric-drain
  behavior that avoids the radial-tank spin),
- same mass off the body.

`ApplyThrust` then calls
`consumeResourceMass(Hydrogen, flow, p)` and
`consumeResourceMass(LOX, flow, p)` — no stage in the fuel path at all.

### 3. Leave stage for what it already is

- Ignition (`ApplyThrust`: `p->stage > as` skip) — unchanged.
- Drop (`droppedPartsAtStage` / `separateStage`) — unchanged; it's already
  decoupler-driven. After a drop, recompute fuel groups (step 1) so survivors'
  pools are correct.

### 4. Small consistency touches

- `getDeltaV()` / `getFuelMass()` (`vehicle.h:297-312`) sum tanks
  ship-wide; keep as-is for the HUD total, but they are now decoupled from the
  per-engine pool, which is correct (the HUD shows the ship's total propellant).
- The Vessel Parts window already prints per-tank Hydrogen/LOX
  (`gameui.cpp:1240-1248`, `1413-1416`) — that stays accurate and is the best
  live check that "the right tanks are draining."

### What does *not* change

- The pro-rata drain, the mass-shedding, the per-part body mass — all stay.
- Reaction wheels, the thrust application, the plume gating (`armedThrust`) —
  untouched.
- The "stage X of N" readout and the stage counter — untouched.

---

## Edge cases to decide while we're in there

- **Multi-resource tanks / mixed propellants.** The current code consumes
  Hydrogen and LOX as independent pools but both must succeed for the engine
  to fire. With fuel groups, the same rule applies *per group*: the group must
  cover both. A group that has H2 but no LOX (or vice versa) can't run. Decide
  whether that's "engine off" (current behavior) or partial — I'd keep the
  current all-or-nothing per tick.
- **An engine with no tank in its fuel group.** E.g. a part def that is a
  thruster but isolated behind decouplers from any tank. Current code: it just
  never draws (pool empty → false). Keep that (engine is inert), but it's worth
  a load-time warning so a designer doesn't ship a ghost engine.
- **Tank + engine in one part.** Allowed by the model (`isTank()` and
  `isThruster()` are independent). In the fuel-group model such a part is a
  fuel node that both holds and consumes — it should draw its own contents
  first, then the group's. Trivial, but note it so the drain order is sane.
- **Two-stage decoupler in one group boundary.** Already handled: each
  decoupler is its own wall, so N decouplers give N+1 groups along the spine.
- **Fuel group recompute cost.** Trivial for current ship sizes (tens of
  parts); if ships grow to thousands, cache and only recompute on staging.

---

## Open questions (for coffee)

1. **Decoupler as the *only* fuel barrier, or a per-part flag?** KSP's rule is
   effectively "decouplers don't transfer." Do we want a general
   `PartDef::fuel_barrier` flag (so some future structural part could also be a
   barrier), or is "isDecoupler() ⇒ barrier" enough? I'd start with the
   decoupler rule and add the flag only if a part needs it.
2. **Should an engine on an earlier stage be *allowed* to keep burning from a
   later stage's tank?** The fuel-group model says yes (stage is timing only).
   Confirm that's the desired semantic — it's what makes heavy_two's central
   engine behave, but it does mean "stage" no longer bounds fuel.
3. **Does the user intend `central_engine2` to stay after the boosters drop?**
   In heavy_two it's above the radial decouplers, so it survives — good. But
   if a design ever wants an *early-stage* engine to *drop* with the boosters
   while still feeding from a *later* tank, that's a fuel-group/ignition combo
   the current drop rule (child-side of the decoupler) won't do. Probably fine
   to defer; flagging it.
4. **Load-time validation.** Add a ship-load check that every thruster's fuel
   group contains at least one tank with the required propellant, and warn
   otherwise. Cheap, catches the "ghost engine" class of design errors.

## Verification plan (once implemented)

- `heavy_two` with `central_engine2` on stage 1: at launch, the **central**
  tanks (2/3) drain and the radial tanks drain in parallel; the central engine
  plume shows and its tanks drop in the Vessel Parts window.
- Press SPACE: boosters fall off; central engine keeps burning off the (still
  full) central tanks.
- A decoupler between a tank and its engine (synthetic ship): the engine must
  *not* drain the far-side tank.
- `make clean && make test && make e2e` green; the existing staging e2e case
  (`e2e/cases/02-staging-basic.txt`) still passes.

---

*This is a snapshot + proposal, not yet implemented. The refactor is scoped to
`vehicle.h` (fuel-group construction + a changed `consumeResourceMass`
signature) and `ApplyThrust`; nothing in physics, rendering, or the UI changes.*
