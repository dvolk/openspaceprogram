# Inventory for kerbals and parts — design review

Date: 2026-09-20
Status: design + work plan. Phase 1 (identity + the two live bugs) is built;
see `addendum.md` for what landed. Phases 2-5 remain.
Rev 2: corrected after a full fact-check pass against the source. Changelog in §8.

## 0. What was asked

Add inventory to kerbals (and to parts). The proposal on the table was a
two-type refactor:

- `GameObject` — Part, Kerbal, InventoryItem. Has physics *properties* but no
  active simulation of its own.
- `PhysicsObject` — a wrapper for 1+ GameObjects that registers them with
  Bullet and handles Kepler orbits.

So: a ship is a PhysicsObject over many part GameObjects fused into a
`btCompoundShape`; a capsule part *contains* a kerbal GameObject; the kerbal
*contains* inventory GameObjects; an inventory item may contain further
GameObjects (a backpack). A kerbal leaving a ship gets wrapped in a
1-part PhysicsObject; a dropped item likewise; re-boarding discards the
wrapper and adds the mass back to the capsule.

Open question raised: how would IVA / in-ship physics movement work later?
A `PhysicsObject2` with Bullet but no orbital mechanics? A flag?

The request was for a brutal evaluation, because getting it wrong is costly.

**Verdict, short form:** the *containment* idea is right and worth doing.
The *GameObject/PhysicsObject* split is the wrong axis, is not required for
inventory, and would cost substantially more while putting the hard-won
physics core at risk for zero inventory benefit. Do containment now; do the
physics split only if and when IVA lands.

---

## 1. Current state (facts, with line references)

### 1.1 The types as they stand

| type | file | what it is |
|---|---|---|
| `Body` | `src/body.h` | render assets (shared, non-owning) + `btCollisionShape` (owned) + mass + `btRigidBody *btBody` **which is null for ship parts** (`body.h:57`) |
| `Part` | `src/part.h` | owns a `Body`, non-owning `const PartDef *def`, `std::string id`, `ResourceContent resources`, `stage`, `fuelGroup`, `armedThrust`, `Part *parent` (tree edge), authored `localPos`/`localRot` in frame S |
| `Vehicle` | `src/vehicle.h` / `.cpp` (1155 + 2589 lines) | `vector<Part*> parts`, ONE `Body *hull` with a `btCompoundShape`, plus frames/SOI/rails/Kepler, fuel groups + links, EC pool, staging, docking, control laws, instrumentation |
| `Kerbal` | `src/eva.h` | `: Vehicle`. One part. EVA control law + `Vehicle *aboard` / `size_t aboardPart` |

Behavior is already **derived from def fields**, not stored: `isThruster()`,
`isJet()`, `isWheel()`, `isRcs()`, `isDecoupler()`, `isDockingPort()`,
`isFuelBarrier()`, `isCapsule()`, `isTank()`, `isBattery()`
(`src/part.h:71-101`). This is the pattern inventory should extend, not replace.

A ship is a **single rigid body**: one `btRigidBody` whose shape is a compound
of the part hulls at their authored poses, re-based through `principal` into
the principal-inertia frame (`src/vehicle.h:129-155`). Part hulls are added as
compound children at `vehicle.cpp:563` with `masses[i] = p->body->mass`
(`:564`), and the one body is created at `vehicle.cpp:594`. Parts get no rigid
body of their own — `create_part_body` (`body.cpp:66-77`) builds shape + mass
and stops there. Part poses are *derived* (`partWorldPose`), and
`checkCompoundInvariants()` re-derives the mass properties, the COM and the
child poses; it is called from `rebuildCompound` at `vehicle.cpp:609`, so it
runs on every build, stage and burn-triggered refresh.
`tests/test_inertia.cpp:15-16, 86, 288, 491` pins the analytic parallel-axis
form against it independently of Bullet.

### 1.2 A kerbal already is a Part

`res/parts.json`:

```json
{ "name": "kerbal", "type": "kerbal", "display_name": "Kerbal",
  "mesh": "kerbal.obj", "texture": "kerbal.png",
  "mass": 97.05, "radius": 0.199, "height": 0.75,
  "capacity": { "hydrazine": 10.0 } }
```

and `res/ships/kerbal.json` is `{ "name": "kerbal", "parts": [ { "part": "kerbal" } ] }`.
So a kerbal is built through the ordinary `build_ship` path from an ordinary
catalog entry. An inventory item would be exactly the same kind of thing.

### 1.3 Crew boarding / EVA today

- `Game::kerbalEVA` — `src/game.cpp:485`
- `Game::kerbalBoard` — `src/game.cpp:558`
- `Game::toggle_eva` (V key) — `src/game.cpp:619`; key routing `src/events.cpp:218-224`
- UI buttons — `src/gameui.cpp:1629-1662`
- Startup/VAB spawn aboard — `Ships::spawn_crew_kerbal`, `src/ships.cpp:165-207`
- Load reconstructs aboard state — `buildKerbalFromSave`, `src/save.cpp:259-312`
- `partCrew()` is a linear scan over `ship->crew` comparing `aboardPart == part` — `src/game.cpp:458`

Board: `addPartMass(capPart, +kerbalMass)` (`game.cpp:578`), `placeShipAtCom`,
`RemoveBody(kb)`, then **`onRails = true; railFrozen = true`**
(`game.cpp:585-586`), list move, `aboard`/`aboardPart`/`crew` updated, and
control handed to the ship if the player was flying the kerbal
(`game.cpp:607-610`).

EVA out: `addPartMass(capPart, -kerbalMass)` (`game.cpp:499`), pose computed
from the capsule's live pose, frame/list move, `AddPhysicsBody(kb)`,
`onRails = railFrozen = false`, control handoff.

### 1.4 The index problem

`Kerbal::aboardPart` is a bare `size_t` index into `aboard->parts`
(`src/eva.h:60`). Every operation that renumbers that vector must therefore
route through `crewRebase`:

- base no-op — `src/vehicle.cpp:889`, decl `src/vehicle.h:470`
- `Kerbal` override — `src/eva.h:66-72`
- `absorbShip`'s map (`i -> aSize + i`) — `src/vehicle.cpp:1895-1901`, call at `1899`
- `extractSubtreeAsShip`'s **two** maps — `goReindex` (`vehicle.cpp:1987, 1994`)
  and `stayReindex` (`vehicle.cpp:2020, 2024`), dispatched at `2030-2031`
- dropped parts are deliberately collected *in original order* purely so those
  indices stay meaningful (comment `vehicle.cpp:1984-1985`, loop `1988-1994`)
- `Game::stage()`'s crew guard scans `a->parts` by index (`game.cpp:915-920`)
- the part window's `PartSel::part` is an index captured at pick time, valid
  only because every renumbering op calls `dropPartWindowsFor`
  (`game.cpp:240`; the four call sites are `825, 874, 979, 1012`)
- **save persists the raw int**: `s.aboard_part = (int)k->aboardPart`
  (`save.cpp:92`), field at `save.h:122`, round-trip at `save.cpp:292-308`.
  Load range-checks it at `save.cpp:292` but **never checks the target is
  actually a capsule** — there is no `crew_capacity > 0` test anywhere on that
  path.

### 1.5 Mass is duplicated while a kerbal is aboard

`addPartMass` is literally two lines (`vehicle.cpp:1215-1218`):
`p->body->mass += delta; rebuildCompound();`. Nothing touches the kerbal's own
hull mass. `kerbalBoard` reads `kerbalMass = kb->mass` (`game.cpp:575`) and
adds it; `kerbalEVA` reads the *same, unchanged* `kb->mass` (`game.cpp:496`)
and subtracts it. So while aboard the mass exists in two places, and only the
capsule's copy is simulated.

That duplication is load-bearing in three defensive places:

- `select_ship` refuses aboard crew outright: "un-parking it here would
  double-count its mass" (`game.cpp:403-412`)
- `remove_ship` refuses ships carrying crew (`game.cpp:993`, refusal at `1000-1007`)
- load must **deliberately skip** `addPartMass` because the crew mass is
  already baked into the capsule's saved mass (`save.cpp:251-257`, and
  `save.h:53` documents "the capsule's includes any aboard crew")

That last one is the worst of the three: a ± mutation that must be *remembered
not to happen* on one specific code path.

### 1.6 "Not simulated right now" has no single answer

It is a conjunction of conventions: `aboard != nullptr`, `onRails`,
`railFrozen`, `hullInWorld()`. The authoritative one is `hullInWorld()`
(`vehicle.cpp:471-474`), which asks Bullet for the broadphase handle rather
than trusting a flag — the rationale and the crash it prevents are documented
at `vehicle.h:162-166` ("…a rebuild once deleted a REGISTERED body and left a
dangling proxy that crashed the next updateSingleAabb").

**The leak, and it is worse than "stale".** `rail_pos` and `rail_vel` have
**no default member initializer** (`vehicle.h:333-334`) and are written only
in `goOnRails()` (`vehicle.cpp:2500-2501, 2505-2506`) and `moveToRailFrame()`
(`:2576-2577`). The build does not define `GLM_FORCE_CTOR_INIT` (Makefile
`CXXFLAGS`, line 73), so glm leaves `vec3` members **indeterminate**
(`middleware/glm/glm/detail/setup.hpp:841`, `type_vec3.inl:11-17`). Three
boarding paths set the parked flags:

| path | sets `onRails`/`railFrozen` | `rail_pos` / `rail_vel` |
|---|---|---|
| `Ships::spawn_crew_kerbal` | `ships.cpp:192-193` | **indeterminate** (never railed) |
| `buildKerbalFromSave`, aboard branch | `save.cpp:305-306` | **indeterminate** (never railed) |
| `Game::kerbalBoard` | `game.cpp:585-586` | stale-but-finite (last written by `select_ship`'s `goOnRails`, `game.cpp:418`) |

`leaveRails()` (`vehicle.cpp:2528-2536`) tests only `onRails`, which is `true`
for all three, so it would call `writeRailPose()` →
`setPosRot(hull, rail_pos, …)` (`vehicle.cpp:2431`) with indeterminate doubles.
`rail_orient`/`railRot` *are* NSDMI'd to identity (`vehicle.h:335, 341`), so
only the position/velocity halves are garbage.

This is unreachable today **by convention, not by type**, and the convention is
spread thin. There are ten `leaveRails()` call sites. Seven are guarded
(`game.cpp:409`, `:687`, `:850`, `:895`, `:1045`, `:1050`, `events.cpp:210`);
the other three (`tick.cpp:117`, `tick.cpp:139`, `events.cpp:485`) plus
`game.cpp:715` have **no crew guard at all** and rely entirely on the structural
invariant that `g.ship` is never an aboard kerbal — enforced only by
`select_ship`'s refusal and by `kerbalBoard` handing control away.

### 1.7 Two live bugs, both reachable with stock content

**(a) Duplicate `Part::id` after docking.** Ids are enforced unique *within
one ship def* (`shipdef.cpp:409-420`, explicit `throw` at `417-420`;
auto-generated as `"<catalog name>_<n>"` at `411-416`, and both `idToIndex` and
`autoCount` are per-file locals). But `absorbShip` (`vehicle.cpp:1862-1926`)
mutates only `localPos`/`localRot` on B's parts (`:1877-1880`) and **never
writes `q->id`**, so a docked ship can hold two `capsule_1`s. Save resolves
references by string id through maps built with plain `operator[]`
(`save.cpp:205-206`: `idToIndex[sp.id] = i; idToPart[sp.id] = p;`), which
silently overwrite on duplicate.

The stock collision surface is wide: docking requires a port on **both** ships
(`game.cpp:771-772, 791-792`), `res/ships/small_docker.json` is the only stock
def with one, so the stock repro is **two `small_docker`s — where every id
collides**. And 11 of the 17 stock defs declare `"controller": "capsule_1"`.

Two concrete failures, both constructed and checked against the code:

*Repro A — wrong controller, silent.* Dock A ← B, both `small_docker`. Save
writes `s.controller = "capsule_1"` (`save.cpp:128`). On load
`idToPart["capsule_1"]` has been overwritten by B's copy (`save.cpp:206`), so
`v->controller` (`save.cpp:208-211`) resolves to **B's capsule** at the far end
of the merged stack. Per `vehicle.h:112-115` the camera basis and stick frame
are built from the controller's local axes — so after a dock→save→load
round-trip the controls and camera come from the wrong part. No diagnostic.

*Repro B — a ship becomes permanently un-undockable.* A docks B, then A docks
C (all `small_docker`), giving `A->seams = [{A.dp, B.root, "B"},
{A.dp, C.root, "C"}]` (`vehicle.cpp:1916`). Both seams serialize to
`port="docking_port"`, `root="docking_port"` (`save.cpp:146-147`). On load,
last-write-wins makes `idToPart["docking_port"]` C's copy, so **both** seams
reconstruct as `{C.dp, C.dp}` (`save.cpp:245`). The first `undock` pops
`seams.back()` and works by luck. The second pops seam 0 and calls
`extractSubtreeAsShip(C.dp)` — but `C.dp` is no longer in `A->parts`, so the
`found` scan (`vehicle.cpp:1931-1932`) fails, returns `nullptr`, and
`game.cpp:866-869` reports "Cannot undock" and returns **before**
`seams.pop_back()` at `:871`. The stale seam is never cleared. **B can never be
undocked.**

What is *not* broken here, for accuracy: **parent** resolution is safe. The
saved order is always parent-before-child (enforced by `shipdef.cpp:422-425`,
preserved by `absorbShip`'s append at `vehicle.cpp:1904` and by
`extractSubtreeAsShip`'s order preservation at `1988-1994`/`2047-2049`, and
re-checked by the `throw` at `save.cpp:197-202`), and `idToIndex[sp.id] = i`
is written at `save.cpp:205` *after* the `attach` at `:203` — so a child always
resolves against the correct preceding copy. The reachable victims are the
references resolved **after** the loop: controller (`208-211`), fuel links
(`212-221`) and dock seams (`241-246`). Fuel links are not reachable with stock
content — the only def with a docking port has no `fuel_link` parts, and the
two defs that do (`heavy_asp.json`, `heavy_two.json`) have no port — but they
become reachable via the VAB or a new def. Note also that a duplicate id makes
the fuel-link `throw` at `save.cpp:215-219` **unreachable**: that fires only on
an *absent* id, so a duplicate silently mis-resolves instead.

**(b) `DockSeam` holds raw `Part*` and is never pruned.** `seams` has exactly
three functional touch points: push at `vehicle.cpp:1916`; read/pop at
`game.cpp:851, 864, 871`; serialize at `save.cpp:144-148` and `241-246`.
`extractSubtreeAsShip` (`vehicle.cpp:1928-2073`) rebuilds parts, hull, fuel
links, crew, controller, stage counters and drain state — **never `seams`**. No
destructor, caller or validation pass prunes them either (`~Vehicle`
`vehicle.cpp:1269-1278`, `Game::stage()` `game.cpp:893-986`, and the load path
all leave them alone).

Reachability needs a decoupler that is an *ancestor* of the seam's `port`. A
docking port is not itself a decoupler (`res/parts.json`: the three ports set
`docking_port` + `fuel_barrier`, not `decoupler`), so it cannot be a stage root
— but it can sit below one. **The codebase already anticipates this**:
`game.cpp:765` ("the ship was removed or **the port staged away**") and
`game.cpp:787` ("else it went stale — **staged away**").

Concrete sequence:

1. Ship A built as `capsule → decoupler D (stage N) → docking_port P`. A docks
   B → `A->seams = [{P, B.root, "B"}]`.
2. Stage N → `droppedPartsAtStage` (`vehicle.cpp:1834-1860`) returns
   `{D, P, B.root, …B}`; `extractSubtreeAsShip(D)` (`game.cpp:968`) moves them
   into a new ship `out`. **A keeps two `Part*` it no longer owns.**
3. `save_game(A)` → `v->seams[0].port->id` (`save.cpp:146`) reads a Part owned
   by `out`. Still alive, so no crash — but the id is not in A's part list, so
   on load `save.cpp:244` silently `continue`s → **the seam vanishes**.
4. `remove_ship(out)` (`game.cpp:1028`, `delete v`) → `~Vehicle` →
   `vehicle.cpp:1277 delete p` frees P and B.root. A's seam now genuinely dangles.
5. `save_game(A)` → `v->seams[0].port->id` → **use-after-free read**,
   ASan-detectable via `obj_asan`.

**(c) Bonus, same root cause: `absorbShip` does not copy `seams`.** It moves
`parts`, `fuelLinks` and `crew` (`vehicle.cpp:1904-1907`) but not `seams`. When
the active ship absorbs a target that already had seams, `delete b`
(`game.cpp:840`) destroys that seam record while its parts live on in the
absorber — so those joints become permanently un-undockable. No dangling
pointer, but the same "seams are not maintained" defect.

### 1.8 Resources

- `ResourceType` — `src/shipdef.h:134-144`: Hydrogen, LOX, EC, Oxygen, Water,
  Food, Hydrazine, JetFuel
- `ResourceContent { float current[Num], capacity[Num]; }` — `shipdef.h:146-156`
- `PartDef::capacity` — `shipdef.h:341`
- Amounts are per-`Part` instance state (`Part::resources`, `part.h:41`),
  seeded full in `Vehicle::init` (`vehicle.cpp:843-850`; `init()`'s own comment
  at `:838-842` warns against re-seeding). Splits keep burned contents because
  `extractSubtreeAsShip` calls `finalize()` (`vehicle.cpp:2063`), not `init()`.
- Fuel groups are connected components of the part tree across conducting
  parts, split by `fuel_barrier` (`buildFuelGroups`, `vehicle.cpp:893-925`);
  links bridge groups one-way (`vehicle.h:298-305`); drain is layered by hop
  distance and pro-rata within a layer (`fuelDrainLayers` `vehicle.cpp:938-980`,
  `consumeResourceMass` `vehicle.cpp:981-1031`, which writes
  `p->body->mass -= share` at `:1026` with no rebuild)
- `refreshCompound()` runs once per ship per tick (`tick.cpp:258`) and is what
  picks up that drift, against `kComRebuildTol = 0.01` m and
  `kMassRebuildFrac = 1e-3` (`vehicle.h:232-233`, used at `vehicle.cpp:635-636`)
- **No cross-ship resource transfer exists anywhere.** Exhaustive grep for
  writes to `resources.current[]` across `src/` returns exactly five sites:
  `vehicle.cpp:843-850` (init seed), `:1022` (`consumeResourceMass`, iterates
  `parts`), `:1113` (`drainEC`, iterates `parts`), `:1136` (`chargeEC`, iterates
  `parts`), `save.cpp:191` (load). No routine takes a second `Vehicle*`. All
  three docking ports are `fuel_barrier: true` in `res/parts.json`, so docked
  ships keep separate fuel systems by construction.
- The EVA suit draws from the kerbal's own single-part pool —
  `src/eva.cpp:152-158`, `consumeResourceMass(…Hydrazine…, parts[0])` at `:155`
- **Kerbal suit resources are not persisted.** `saveShipFromVehicle` returns at
  `save.cpp:101` for crew, *before* the part loop at `106-121` — no parts, no
  fuel. `buildKerbalFromSave` rebuilds via `build_ship` (`save.cpp:269`) →
  `init()` → full re-seed. `save.cpp:251-253` says so explicitly. Harmless
  today; becomes free fuel the moment consumables matter.

### 1.9 Fleet and ownership

`src/fleet.h/cpp` is only the startup JSON manifest parser (`fleet.h:6-9`);
semantic resolution is in `main.cpp`. Runtime ownership: each *free* ship (and
each *free* kerbal) lives in `TerrainBody::ships` of its SOI body
(`terrain.h:170`, with `terrain.h:165-169` documenting that aboard crew are
*not* there); aboard kerbals are owned by their carrier in `Vehicle::crew`
(`vehicle.h:106`), deleted first in `~Vehicle` (`vehicle.cpp:1269-1279`, crew at
`1272-1273`, parts at `1277`). The canonical enumeration is `collectVehicles`
(`ships.cpp:28-36`), which emits each ship immediately followed by its crew —
an ordering save/load depends on (`save.cpp:384-386`).

### 1.10 Where `Kerbal : Vehicle` is load-bearing

Nine distinct categories, all of which would need to move if a kerbal stopped
being a `Vehicle`:

1. tick pipeline (`tick.cpp:235-302`) — gravity, aero, power, rails, SOI
   switching, compound refresh all come free via `collectVehicles`
   (`all` = `collectVehicles(g.sys)` at `tick.cpp:34`, re-snapshotted at `:211`)
2. `collectVehicles` ordering (`ships.cpp:28-36`) — save/load, F6 cycling,
   dedup all consume it
3. **27** real `isEva()` / `isCrewAboard()` call sites: `game.cpp` 13,
   `events.cpp` 4, `tick.cpp` 3, `save.cpp` 3, `gameui.cpp` 3, `render.cpp` 1
   (at `render.cpp:263`). Excludes the two virtual definitions
   (`vehicle.cpp:885, 887`) and the overrides (`eva.h:51-52`). Note
   `render.cpp:364-365` is a *comment documenting the absence* of an
   aboard-skip there, not a dispatch site.
4. control law (`applyControlForces` override `eva.h:87-89`, `evaArmCommands`)
5. physics/parking — hull, compound, `RemoveBody`, `goOnRails`,
   `placeShipAtCom`, `partPos`, `partVel`
6. picking (`pick.cpp:110-155`)
7. save format — kerbals are `SaveShip` entries with `is_crew`
8. ownership/teardown — carrier-owned crew deletion, the `isCrewAboard`
   rollback pass in `load_game` (`save.cpp:400-421`, test at `:416`)
9. camera/HUD/map (`game.cpp:301`, `gameui.cpp:1296-1300, 2773-2777`)

---

## 2. Evaluation of the proposal

### 2.1 What is right

**Containment is the correct fix for §1.4.** Replacing an index with a
containment edge deletes `crewRebase`, all three reindex maps, the
"collect in original order" constraint, and the raw int in the save format.
Inventory is *the same edge* as crew — one mechanism, two features. That two
independent needs pull out the same abstraction is the strongest evidence it
is the right one.

**"Has physics properties but no active simulation" already exists.** It is
`Body::btBody == nullptr` (`body.h:57`, documented at `body.h:135-138`). Parts
already work this way. So this part of the proposal is a rename, not new
machinery — which is exactly why the rename is not worth paying for (§2.2).

**Drop/pickup as wrapper create/destroy is right.** A free-floating item
genuinely is a 1-part ship with its own trajectory, and the codebase already
has that primitive (`extractSubtreeAsShip` + `enterWorld`).

### 2.2 `GameObject` is a bad name

It is Unity vocabulary for "a scene entity with components attached." What is
being described is "a node in an assembly/containment tree that owns mass,
resources and a collision hull." The name makes every future reader import the
wrong mental model.

Worse, `Body` already exists and already means precisely "render assets +
collision hull + mass + optional rigid body." The result would be
`GameObject` → owns a `Body` → which holds the physics: three nouns for two
concepts.

And no new noun is needed. A kerbal already *is* a Part (§1.2). An inventory
item is a Part. A backpack is a Part with capacity. `part.h:71-101` already
derives behavior from def fields — the extension is `isContainer()`.

**Recommendation: keep `Part`. Generalize it.**

### 2.3 `PhysicsObject` conflates two independent axes

`Vehicle` currently does six jobs:

| | job |
|---|---|
| a | one `btRigidBody` + `btCompoundShape`; `rebuildCompound`, `refreshCompound`, `checkCompoundInvariants` |
| b | force laws: gravity with the COM-offset correction, thrust, per-part aero, RCS, reaction wheels |
| c | celestial mechanics: `Frame`, SOI, rails, Kepler (`rail_pos/vel/orient`, `writeRailPose`, `railsTick`, `leaveRails`) |
| d | resource network: fuel groups, fuel links, drain layers, EC pool |
| e | assembly topology: part tree, staging, absorb/extract, dock seams |
| f | command state + instrumentation logs |

(a)+(b) is **Bullet simulation**. (c) is **trajectory**. These are different
axes. Calling the union `PhysicsObject` welds them together — and the reach
for "`PhysicsObject2` = Bullet but no orbital mechanics" is the direct
symptom of that weld.

**Recommendation: don't add a type. Split the axis.**

- `Assembly` = (a)+(b)+(e). "What is this rigid thing made of, and what pushes it."
- `Trajectory` = (c). "Where is it, and who integrates its pose."
- (d)+(f) stay on the ship type; they are ship systems, not physics.

Every case then falls out with **no flag and no second type**:

| thing | Assembly | Trajectory |
|---|---|---|
| ship in flight | yes | yes |
| ship on rails | yes (hull parked) | yes |
| EVA kerbal | yes, 1 member | yes |
| dropped item | yes, 1 member | yes |
| kerbal aboard | **no** — contained by a capsule Part | **no** — rides the ship's |
| inventory item | **no** — contained | **no** |
| IVA kerbal in the cabin | yes, 1 member | **no** — pose slaved to a cabin frame |

### 2.4 The IVA question answers itself

"Bullet physics but no orbital mechanics" is not a flag. It is the statement
*this Assembly's pose is derived from a parent frame rather than integrated.*
That is frame parenting, and the machinery already exists: `Frame` is a
parent/children tree (`frame.h:10-15`) with relative `pos`/`vel`/`orient`
(`:22-25`) plus `GetPositionRelTo` / `GetVelocityRelTo` / `GetOrientRelTo`
(`:63-65`), `GetStasisVelocity` (`:99`) and `GetFictitiousAccel` (`:115`).

An IVA interior is a `Frame` rigidly carried by the ship's Assembly. The
kerbal's Assembly lives in it and collides against a cabin collision volume;
the **ship's** Trajectory does the orbital work for both. Entering IVA is
re-homing the kerbal Part from "contained by capsule" to "root of a small
Assembly in the cabin frame" — the EVA transition minus the Trajectory.

Caveat, stated honestly: `Frame::body` is a `TerrainBody*` (`frame.h:14`) and
the frame tree is per-body today, so an assembly-carried node needs either a
null body or a small extension to `Frame`. That is a real but local cost, and
far smaller than maintaining a parallel `PhysicsObject` hierarchy.

### 2.5 Mass must become derived, not ± bookkept

See §1.5. Under nesting (kerbal→capsule→ship, item→backpack→kerbal) the
`addPartMass` ± pattern becomes strictly worse.

```
effectiveMass(p) = p->body->mass + Σ effectiveMass(c)  for c in p->contents
```

`rebuildCompound` reads `effectiveMass`; `addPartMass` disappears; the three
defensive guards in §1.5 disappear; the save stops baking crew mass into the
capsule.

Two decisions that must be made explicitly:

- **Where contained mass sits.** A folded kerbal is currently smeared over the
  *capsule hull's shape* — it goes into `body->mass` (`vehicle.cpp:1216`), and
  the compound child is `p->body->shape` with `masses[i] = p->body->mass`
  (`vehicle.cpp:563-564`), so added mass inherits the capsule hull's geometry.
  With `contents` you can do better for free: contribute each child as a point
  mass at its authored local pose via parallel-axis. `rebuildCompound` already
  walks the poses. A backpack then actually moves the COM.
  **`checkCompoundInvariants()` (`vehicle.cpp:609`) and
  `tests/test_inertia.cpp` must be updated in lockstep** — that invariant check
  is the only thing keeping this representation honest, so it is not optional.
- **`refreshCompound`'s thresholds** (`kComRebuildTol = 0.01` m,
  `kMassRebuildFrac = 1e-3`, `vehicle.h:232-233`) currently trip on
  burn-driven drift. They must read *derived* mass, or a kerbal drinking from
  a suit tank silently stops re-centering the COM — reintroducing the spurious
  `comOffset × F` torque that `--tq-log` exists to hunt (`cli.cpp:319`,
  consumed `tick.cpp:495-496`, implemented `vehicle.cpp:1665`).

### 2.6 One invariant, enforced hard

> Every Part is either **attached** to exactly one Assembly, or **contained**
> in exactly one Part. Never both, never neither.

Two states, not three. "Floating free" is *not* a Part state — it is an
Assembly with one attached member and its own Trajectory. That is what `drop()`
creates and `pickUp()` destroys.

This is precisely what §1.6 lacks. Under the new design the aboard-kerbal
`leaveRails` bug cannot be written, because a contained Part has no Trajectory
to leave rails on.

### 2.7 `Kerbal : Vehicle` is right in one mode and wrong in the other

The premise "the current design for ships/kerbals isn't appropriate" is only
half correct. The one-part-Vehicle identity is a deliberate, documented choice
(`src/eva.h:5-12`, `reports/eva2026_09_02/`) that buys the fleet list, F6
cycling, rails, SOI bookkeeping, HUD, gravity, aero, picking and save *for
free* — see the nine categories in §1.10. For a kerbal **on EVA** the
equivalence is genuinely true and it earns its keep.

It is wrong only for a kerbal **aboard**: a `Vehicle` that is not simulated is
a contradiction, and §1.4/§1.5/§1.6 are all symptoms of that one contradiction.

So the sharp goal is not "stop Kerbal being a Vehicle." It is: **make "aboard"
stop being a Vehicle state at all** — a contained Part. Then the inheritance
applies only when it is true.

---

## 3. Recommended architecture

```
Part                      (renamed from nothing; generalized)
  Body *body              render assets + collision hull + mass (owned)
  const PartDef *def      catalog spec (non-owning)
  uint64_t uid            globally unique instance id          [NEW, phase 1]
  std::string id          def-authoring id (per-def unique only)
  ResourceContent         per-instance tank contents
  Part *parent            assembly tree edge
  Part *container         containment edge, XOR with owner      [NEW, phase 2]
  vector<Part*> contents  non-owning in phase 2; see step 2.1   [NEW, phase 2]
  Assembly *owner         the assembly this part is attached to [NEW, phase 2]
  localPos / localRot     authored pose in frame S
  double effectiveMass()  body->mass + Σ contents               [NEW, phase 3]

Assembly                  (the physics half of today's Vehicle) [phase 5]
  vector<Part*> parts     all have owner == this
  Body *hull              one btRigidBody, btCompoundShape
  rebuildCompound / refreshCompound / checkCompoundInvariants
  force laws: gravity, thrust, aero, RCS, wheels
  topology: staging, absorb, extract, seams

Trajectory                (the celestial half of today's Vehicle) [phase 5]
  Frame *frame, TerrainBody *m_parent, sun
  onRails / railFrozen / rail_pos / rail_vel / rail_orient / railRot
  goOnRails / leaveRails / railsTick / writeRailPose / switchFrames

Ship : Assembly           (d) + (f): fuel network, EC, control, instrumentation
  Trajectory traj
```

`Kerbal` becomes a `Part` with `type == "kerbal"`. On EVA it is the sole member
of a transient `Assembly` + `Trajectory`. Aboard, it is contained by a capsule
Part and has neither. An inventory item is a `Part` contained by a kerbal or a
cargo part; dropped, it becomes a 1-member Assembly + Trajectory.

---

## 4. Work plan

Steps are deliberately small: each is independently committable, each has a
named verification, and no step depends on a later one. Per project rules,
run `make test` and the relevant e2e cases before each commit.

### Phase 0 — hygiene (no behavior change)

**0.1 — Fix the misleading comments and the stray `/* eh */`.**
Two places document a "parked (rails) pose relative to the cluster COM …
Written by `goOnRails()`, read by `writeRailPose()`" field that the
single-body rewrite deleted: the file header at **`part.h:12`** and the
dangling comment at **`part.h:62-64`**. `part.h` is the first file anyone reads
for this refactor, so this is actively harmful. Separately,
`vehicle.h:517` has `float getFuelMass(const std::vector /* eh */ <enum ResourceType>& types);`
and its out-of-line definition at `vehicle.cpp:1033` carries the matching
stray space (`const std::vector <enum ResourceType>&`) — fix both in one commit.
*Files:* `src/part.h`, `src/vehicle.h`, `src/vehicle.cpp`. *Verify:* `make test`.

### Phase 1 — identity (fixes the live bugs; prerequisite for everything)

**1.1 — Add a globally unique instance id to `Part`.**
`uint64_t uid` on `Part`, assigned from a monotonic counter wherever a `Part`
is constructed: `build_ship_structure` (`vehicle.cpp:102`, decl `vehicle.h:1087`)
and `buildShipFromSaveParts` (`save.cpp:184`). Never reused, never renumbered.
Leave `std::string id` alone — it stays the def-authoring key and keeps
`shipdef.cpp:409-420`'s per-def uniqueness check.
*Files:* `src/part.h`, `src/vehicle.cpp`, `src/save.cpp`. *Verify:* new
assertion in `tests/test_shipload.cpp` that every part in a built ship has a
distinct uid; `make test`.

**1.2 — Persist and resolve by uid in the save format.**
Add `uid` to the per-part record (`src/save.h`; the loop is `save.cpp:106-121`
inside `saveShipFromVehicle`, `save.cpp:84-155`). On load, build
`map<uint64_t, Part*>` and resolve controller, fuel links and dock seams
through **uid** instead of the string id (`save.cpp:196-246`; parent resolution
at `196-204` may stay id-based since §1.7(a) shows it is correct, but uid is
simpler and uniform). Replace the silent `operator[]` inserts at
`save.cpp:205-206` with checked inserts that report duplicates. No back-compat
needed.
*Files:* `src/save.h`, `src/save.cpp`. *Verify:* `tests/test_save.cpp`
round-trip; add a case that builds a ship with two same-named parts and
round-trips it.

**1.3 — Add an e2e for dock → save → load.**
This is the case §1.7(a) says is already broken. Reproduce **Repro A** (two
`small_docker`s; after round-trip the controller must still be A's capsule) and
**Repro B** (dock B then C; both must remain undockable after round-trip).
Confirm the failures first, then confirm 1.1+1.2 fix them.
*Files:* `e2e/cases/`, `tests/test_dock.cpp`. *Verify:* the new case passes;
`test_dock` gains a two-same-part-name merge assertion.

**1.4 — Maintain `seams` through topology changes.**
Two fixes, one commit:
- `extractSubtreeAsShip` (`vehicle.cpp:1928-2073`): drop any seam whose `port`
  or `root` is in the dropped set. A seam split across the cut means the joint
  no longer exists — drop it and say so in a comment.
- `absorbShip` (`vehicle.cpp:1862-1926`): move `B->seams` into A alongside
  `parts`/`fuelLinks`/`crew` (`:1904-1907`), fixing §1.7(c).

*Files:* `src/vehicle.cpp`. *Verify:* new case in `tests/test_dock.cpp` shaped
exactly as §1.7(b)'s sequence — **A built as `capsule → decoupler → docking_port`**,
dock A+B, stage the decoupler, `remove_ship` the split-off, then save A. Run
under ASan (`obj_asan`); step 5 of §1.7(b) is the use-after-free this catches.
A second case: dock A+B where B already has a seam, then undock it through A.

> Note: staging away *B's* subtree does **not** reproduce this — B's root's only
> ancestor is A's port, which stays. The decoupler must be an ancestor of A's
> own docking port.

### Phase 2 — containment (the actual prize)

**2.1 — Add the containment edge, unused.**
`vector<Part*> contents` and `Part *container` on `Part`, plus
`Vehicle *owner`. Add an assert for the XOR invariant from §2.6.

**Ownership rule — read this before writing code.** `contents` is
**non-owning** throughout Phase 2-4. A contained crew Part is already owned by
its `Kerbal` Vehicle via `Vehicle::crew`, and `~Vehicle` deletes crew *before*
parts (`vehicle.cpp:1272-1273` then `:1277`). If `~Part` also deleted
`contents`, `delete ship` would free the kerbal's root Part twice — and the
crew-first ordering makes that double free deterministic, not incidental.
Inventory items introduced in Phase 4 have no other owner, so they need one:
give `Part` an explicit `vector<Part*> ownedContents` for items it owns, and
keep `contents` as the non-owning traversal list, **or** defer ownership to
Phase 5 when a contained kerbal stops being a Vehicle. Pick one and document
it in `part.h`; do not leave it implicit.

No caller changes yet, so behavior is identical.
*Files:* `src/part.h`, `src/vehicle.cpp`. *Verify:* `make test` (should be a
no-op); add a headless unit test that builds a containment chain and asserts
the XOR invariant and the traversal.

**2.2 — `aboardPart`: index → pointer.**
Change `size_t aboardPart` to `Part *aboardPart` (`src/eva.h:60`). A `Part*`
survives both `absorbShip` and `extractSubtreeAsShip` because the `Part`
objects are never reallocated — verified: every `new Part` in the product tree
is at `vehicle.cpp:102` and `save.cpp:184`, neither merge nor split calls
either, and neither deletes a `Part` (`delete p` appears only at
`vehicle.cpp:1277` in `~Vehicle` and `save.cpp:198` in the load-error path).
Both move **pointers only**: `parts.insert(parts.end(), B->parts…)` (`:1904`),
`nv->parts = nvParts` (`:1997`), `parts.swap(keep)` (`:2049`). `rebuildCompound`
recreates the *hull* `Body` (`:553, :588`) and `compoundParts` (`:554, :566`)
but never a `Part`.

With `Part::owner` from 2.1, `aboard` becomes `aboardPart->owner` and can be
dropped as a separate field — but note `Vehicle::crew` (the ownership list,
§1.9) stays regardless; see 2.4.

Then **delete** `crewRebase` (base `vehicle.cpp:889`, decl `vehicle.h:470`,
override `eva.h:66-72`), `goReindex` (`vehicle.cpp:1987, 1994`),
`stayReindex` (`vehicle.cpp:2020, 2024`), the dispatch at `vehicle.cpp:2030-2031`,
the map in `absorbShip` (`vehicle.cpp:1895-1901`), and the "collect in original
order for stable indices" constraint (comment `vehicle.cpp:1984-1985`).
*Files:* `src/eva.h`, `src/vehicle.h`, `src/vehicle.cpp`, `src/game.cpp`,
`src/ships.cpp`, `src/save.cpp`. *Verify:* `tests/test_crew.cpp`,
`tests/test_dock.cpp`, `tests/test_staging.cpp`; e2e for board → dock → EVA.

**2.3 — Save the containment edge structurally.**
Replace `aboard_part` (int index, `save.h:122`, written `save.cpp:92`, read
`save.cpp:292-308`) with the container's **uid**. On load, validate the target
is a capsule (`crew_capacity > 0`) — §1.4 notes this check does not exist
today, so a reordered save can park a kerbal in an arbitrary part.
*Files:* `src/save.h`, `src/save.cpp`. *Verify:* `tests/test_save.cpp`; add a
case that hand-edits a save to park a kerbal in a fuel tank and asserts a clean
load error rather than silent corruption.

**2.4 — Register crew in `capsulePart->contents`.**
`kerbalBoard` (`game.cpp:558`) pushes the kerbal's root Part into
`capPart->contents`; `kerbalEVA` (`game.cpp:485`) removes it. **Keep
`Vehicle::crew` as the sole owner of aboard `Kerbal` vehicles** — that is
correct and unchanged, and §1.10 item 8 depends on it; `contents` is a
non-owning back-reference per 2.1. Rewrite `partCrew()` (`game.cpp:458`) to
read `contents` instead of scanning by index.
*Files:* `src/game.cpp`, `src/ships.cpp`, `src/save.cpp`. *Verify:*
`tests/test_crew.cpp`; ASan run of `delete ship` with crew aboard (this is the
double free 2.1 warns about); manual: board, EVA, re-board, F6 cycling still
skips aboard crew.

### Phase 3 — derived mass

**3.1 — Add `Part::effectiveMass()`, unwired.**
Recursive: `body->mass + Σ contents->effectiveMass()`. Add it and unit-test it,
but **do not** wire it into `rebuildCompound` yet. Phase 2 has already
populated `contents` with crew, so wiring it here would change every ship's
mass before the compensating `addPartMass` removal lands, and
`checkCompoundInvariants()` would fail in between. As dead code with a test,
this step is a genuine no-op.
*Files:* `src/part.h`. *Verify:* new headless test in `tests/test_crew.cpp` or
a new `tests/test_mass.cpp`: build a capsule containing a kerbal, assert
`effectiveMass == capsule + kerbal`; `make test` otherwise unchanged.

**3.2 — Wire it up and delete `addPartMass`, atomically.**
One commit, because the number-changing half and the compensating half must
land together or `checkCompoundInvariants()` (`vehicle.cpp:609`, called from
`rebuildCompound`) fails between them:
- `rebuildCompound` reads `effectiveMass()` instead of `p->body->mass`
  (`vehicle.cpp:564`)
- delete `addPartMass` (`vehicle.h:600`, `vehicle.cpp:1215-1218`) and its
  **three** call sites: `game.cpp:499`, `game.cpp:578`, `ships.cpp:194`
- stop baking crew mass into the capsule's saved mass — update `save.h:53`'s
  contract and the "deliberately NOT called" logic at `save.cpp:251-257`
- update `checkCompoundInvariants()` and `tests/test_inertia.cpp` in the same commit

Then re-examine the two defensive guards that existed only because of the
double count: `select_ship`'s aboard-crew refusal (`game.cpp:403-412`) and
`remove_ship`'s crew refusal (`game.cpp:1000-1007`). They may now be removable,
or reducible to a real ownership rule — decide explicitly and comment the
decision. Do not delete them just because the mass reason is gone.
*Files:* `src/part.h`, `src/vehicle.h`, `src/vehicle.cpp`, `src/game.cpp`,
`src/ships.cpp`, `src/save.h`, `src/save.cpp`. *Verify:* `tests/test_crew.cpp`,
`tests/test_inertia.cpp`, `tests/test_save.cpp`; e2e: EVA out and confirm the
ship's mass drops by exactly the kerbal's; `--tq-log` shows no `|w|` ramp.

**3.3 — (optional polish) Contained mass as a point mass at its local pose.**
Parallel-axis contribution at the child's authored pose instead of smearing
over the container hull's shape (§2.5). Deferrable; only matters once
backpacks and cargo are off-center.
*Files:* `src/vehicle.cpp`, `tests/test_inertia.cpp`. *Verify:*
`checkCompoundInvariants()` + the analytic form in `test_inertia`.

### Phase 4 — inventory

**4.1 — Persist kerbal suit resources.**
Independent of everything above and small. Today the 10 kg of hydrazine
re-seeds full on every load (§1.8). Do this **before** inventory items become
consumables, or save-scumming is free fuel.
*Files:* `src/save.h`, `src/save.cpp` (`saveShipFromVehicle`'s early return at
`:101`, `buildKerbalFromSave` at `:259-312`). *Verify:* `tests/test_save.cpp`:
EVA, burn some suit fuel, save, load, assert the reduced amount.

**4.2 — `PartDef` gains inventory capacity; `Part::isContainer()`.**
Follow the existing field-driven pattern (`part.h:71-101`) exactly. Add the
catalog field, parse it in `shipdef.cpp`, add `isContainer()`. Add a cargo part
and give the kerbal suit a small capacity. No behavior yet.
*Files:* `src/shipdef.h`, `src/shipdef.cpp`, `src/part.h`, `res/parts.json`.
*Verify:* `tests/test_shipload.cpp`; `make test`.

**4.3 — Transfer between containers.**
Pure re-parenting: pop from one `contents`, push to the other, set
`container`. No physics, no Assembly, no Trajectory involved. Enforce capacity,
and move ownership with the item (per 2.1's rule — an inventory item has no
other owner). Add UI to the existing part window (`src/gameui.cpp`; the
plumbing already exists — `openPartWindow` `game.cpp:225`, and note
`dropPartWindowsFor` `game.cpp:240` for invalidation).
*Files:* new `src/inventory.cpp` + header, `src/game.cpp`, `src/gameui.cpp`.
*Verify:* new `tests/test_inventory.cpp` (headless): kerbal→ship, ship→kerbal,
ship→ship via a docked pair, capacity refusal, and an ownership/ASan case.

**4.4 — Drop and pick up.**
Drop: remove from `contents`, build a 1-part Vehicle at the kerbal's world
pose with the kerbal's velocity, `enterWorld()`, push to `m_parent->ships`.
Reuse `extractSubtreeAsShip`'s placement logic — the `v + ω × r` derivation at
`vehicle.cpp:1967-1971` and the write-out at `vehicle.cpp:2069-2071`
(`placeShip` + `setVelocity` + `SetAngVelocity`) — so a dropped item inherits
the ship's spin correctly. For a 1-part drop root == COM, so `placeShip` and
`placeShipAtCom` coincide and the velocity expression transfers verbatim.
Pick up is the inverse: `RemoveBody`, destroy the Vehicle, re-parent into
`contents`.
*Files:* `src/game.cpp`, `src/vehicle.cpp`. *Verify:* e2e with `--sim-press`:
EVA, drop an item, confirm it does not fall through terrain and co-moves with
the ship; board, pick it back up, confirm the ship's mass returns to the
pre-drop value.

**4.5 — Kerbal consumes from inventory.**
Extend the EVA suit draw (`src/eva.cpp:152-158`, currently `parts[0]` only,
`consumeResourceMass` at `:155`) to walk `contents` for the resource, in a
defined order. Decide whether ship tanks can feed a boarded kerbal's suit
(recommend: yes, and it is a 4.3 transfer, not a new path).
*Files:* `src/eva.cpp`, `src/vehicle.cpp`. *Verify:* `tests/test_eva.cpp`,
`tests/test_fuel.cpp`; e2e: EVA with an empty suit and a full spare tank in
inventory, confirm RCS still works and the tank drains.

**4.6 — Save/load inventory.**
Serialize `contents` recursively by uid, nested under the container. Ordering:
a contained part must be reconstructed after its container, so emit depth-first
— and check this against `collectVehicles`' ship-then-crew ordering
(`ships.cpp:28-36`), which load depends on (`save.cpp:384-386`).
*Files:* `src/save.h`, `src/save.cpp`. *Verify:* `tests/test_save.cpp`:
round-trip a kerbal holding two items, one of which holds a third.

### Phase 5 — deferred: the Assembly / Trajectory split

**Do not schedule this until IVA is actually on the roadmap.** Phases 1-4
deliver the inventory feature without touching the physics architecture.

If it happens, the scope is §2.3 + §2.4 + §1.10: split `Vehicle` into
`Assembly` (jobs a, b, e) and `Trajectory` (job c), keep `Ship` for (d) and
(f), and make `Kerbal` a `Part` that is *promoted* into a transient Assembly
when free. This is also where 2.1's deferred ownership question resolves — a
contained kerbal stops being a Vehicle, so `contents` can become uniformly
owning. Extend `Frame` to support an assembly-carried node (§2.4 caveat) for
the IVA interior. Expect churn in `tick.cpp`, `game.cpp`, `save.cpp`,
`render.cpp`, `pick.cpp`, `gameui.cpp`, `events.cpp`, `ships.cpp`, `vab.cpp`,
including all 27 `isEva()`/`isCrewAboard()` dispatch sites (§1.10 item 3).

Split it into its own phased plan at that time. Do not attempt it as one change.

---

## 5. What is explicitly *not* recommended

- **Renaming `Part` → `GameObject`.** ~1.4k lines across 56 files mention a
  `part`-family identifier (232 lines name the exact type `Part`), plus a
  `Body`/`GameObject` naming collision and Unity baggage — for no functional
  gain (§2.2).
- **`PhysicsObject` as a single type carrying Bullet + Kepler.** It is the
  conflation that creates the IVA confusion in the first place (§2.3).
- **A `PhysicsObject2`, or a `hasOrbitalMechanics` flag.** Both are symptoms.
  Trajectory presence is structural, not a boolean (§2.4).
- **Bundling the physics split with the inventory work.** It multiplies the
  cost and risks the compound invariants, the COM-offset torque fix and the
  rails handoff for zero inventory benefit (§4, phase 5).
- **Keeping `addPartMass` ± bookkeeping under nesting.** See §1.5, §2.5.

---

## 6. Risks

| risk | mitigation |
|---|---|
| `contents` ownership vs `Vehicle::crew` ownership — **double free** | step 2.1: `contents` is non-owning in Phase 2-4; `Vehicle::crew` is the sole owner until Phase 5. `~Vehicle` deletes crew at `vehicle.cpp:1272-1273` *before* parts at `:1277`, so an owning `~Part` would make the double free deterministic. ASan test in step 2.4 |
| `checkCompoundInvariants()` / `tests/test_inertia.cpp` desync when mass becomes derived | step 3.2 lands the wire-up, the `addPartMass` removal, the save-mass change and both invariant updates in **one** commit; step 3.1 is unwired dead code so it cannot break them |
| Phase 2 touches docking, staging and saving at once | 2.2 is pointer-only (mechanical), 2.3 is save-only, 2.4 is registration-only — three commits, not one |
| save format churn across phases 1-4 | no back-compat required (project rule); land each phase's save change with a round-trip test before moving on |
| removing the `select_ship` / `remove_ship` crew guards too eagerly | 3.2 says decide explicitly and comment; if a guard encodes a real ownership rule rather than a mass workaround, keep it and say why |
| Phase 1.4's test doesn't actually reproduce the bug | the decoupler must be an **ancestor of A's own docking port**; staging away B's subtree frees no seam endpoint (note under step 1.4) |
| `Frame` extension for IVA | deferred to phase 5; not on the inventory critical path |

---

## 7. Other things noticed (flagged per project convention)

- `src/part.h:12` and `src/part.h:62-64` — stale comments describing a deleted
  rails-pose field (§4, step 0.1).
- `src/vehicle.h:517` and `src/vehicle.cpp:1033` — `/* eh */` and the matching
  stray space inside a type name (§4, step 0.1).
- `src/docktest.cpp`, `src/radialtest.cpp` — test scaffolding living in the
  product tree next to `src/`. They also contain two of the four `new Part`
  sites (`docktest.cpp:68`, `radialtest.cpp:72`), so step 1.1's uid assignment
  must either cover them or they will silently build parts with `uid == 0`.
- `Game::stage()` de-dups split-ship names inline (`game.cpp:944-957`) rather
  than via `Ships::dedupName` (`ships.cpp:143-155`) — duplicated logic.
- `absorbShip` ignores `crewRebase`'s return value (`vehicle.cpp:1899`);
  harmless only because its map at `:1897` covers all of B's indices. Moot
  after 2.2.
- Kerbal save entries don't persist `home` / `scenario` / `slot`
  (`save.cpp:263-267` hard-codes `home = g.home`, `scenario = nullptr`). A
  kerbal that EVA'd to another body keeps its pose body but loses scenario
  bookkeeping.
- `save.cpp:242-245` silently `continue`s on an unresolvable dock seam id,
  while the fuel-link path at `save.cpp:212-219` `throw`s. Inconsistent failure
  policy for the same class of problem — and §1.7(a) Repro B shows the silent
  path can wedge a ship permanently.

---

## 8. Rev 2 changelog

Corrections made after a line-by-line fact-check of rev 1 against the source:

- **§5 / §2.2:** "~28k lines of churn" was wrong by ~20x — 28,708 is the total
  size of `src/`. The real rename surface is ~1.4k lines across 56 files
  (232 lines name the exact type `Part`). The naming argument stands on the
  Unity-baggage and `Body`-collision grounds, not on churn volume.
- **§1.7(a):** removed the claim that duplicate ids can mis-resolve a part's
  *parent*. Parent resolution is correct: `idToIndex` is written at
  `save.cpp:205` *after* the `attach` at `:203`, and the saved order is
  invariantly parent-before-child. The reachable victims are the references
  resolved after the loop — controller, fuel links, dock seams. Added two
  constructed repros (wrong controller; permanently un-undockable ship) and
  noted that the fuel-link `throw` is unreachable from a duplicate id, and that
  fuel links are not reachable at all with stock content.
- **§1.7(c):** added — `absorbShip` does not copy `seams`, so an absorbed
  ship's joints become permanently un-undockable.
- **§1.6:** "stale rail state" understated it — `rail_pos`/`rail_vel` have no
  NSDMI and no `GLM_FORCE_CTOR_INIT`, so two of the three boarding paths leave
  them genuinely indeterminate. "Three guarded call sites" was an undercount:
  ten `leaveRails()` sites, seven guarded, and the three unguarded tick/event
  sites rely on an unstated `g.ship` invariant. Also corrected the
  `hullInWorld` crash-note citation to `vehicle.h:162-166`.
- **§4 step 2.1 / 2.4 / §6:** rev 1 encoded a **double free** — `~Part` owning
  `contents` while `Vehicle::crew` also owns the contained kerbal. Fixed:
  `contents` is non-owning through Phase 2-4, with the ownership question
  explicitly deferred to Phase 5.
- **§4 steps 3.1 / 3.2:** rev 1 contradicted itself (3.1 called itself both a
  no-op and a number-changer) and broke the plan's own "no step depends on a
  later one" rule, since Phase 2 already populates `contents`. Fixed: 3.1 is
  unwired dead code plus a test; 3.2 does the wire-up, the `addPartMass`
  removal, the save-mass change and both invariant updates atomically. Also
  "four call sites" → three.
- **§4 step 1.4:** the proposed test staged away *B's* subtree, which frees no
  seam endpoint. Corrected to require a decoupler that is an ancestor of A's
  own docking port.
- **§1.10 item 3:** "~13 dispatch sites" → 27. `render.cpp:364-365` was cited
  as a dispatch site; it is a comment documenting the *absence* of one. The
  real site is `render.cpp:263`.
- **Citations corrected:** `part.h:79-101` → `71-101` (3 places);
  `game.cpp:587-588` → `585-586`; `vehicle.cpp:2072-2073` → `2069-2071`;
  `vehicle.cpp:1990-1991` → `1984-1994`; `game.cpp:302-305` → `301`;
  `vehicle.h:301-307` → `298-305`; `vehicle.cpp:935-980` → `938-980`;
  `save.cpp:87-158` → `84-155` (record loop `106-121`); `save.cpp:205-246` →
  `196-246`; `eva.h:87-90` → `87-89`; `ships.cpp:166-208` → `165-207`;
  `tick.cpp:238-302` → `235-302`; `vehicle.h` 1156 → 1155 lines.
- **All ten substantive technical claims verified as written** — single rigid
  body, duplicated aboard mass, the duplicate-id bug, the unpruned `seams`,
  the rail-state hazard, unpersisted suit resources, no cross-ship transfer,
  per-def id uniqueness, `Part*` stability across merge/split (the linchpin of
  step 2.2), and the `refreshCompound` thresholds.
