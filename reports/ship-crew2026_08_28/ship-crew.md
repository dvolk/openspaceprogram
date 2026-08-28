# Adding crew to ships

Date: 2026-08-28
Scope: crew as **game logic + UI data only**. No 3D model, no cockpit interior,
no mass/inertia contribution, no life-support simulation. Related context:
`reports/character2026_08_28` (character scoping) — this report covers only the
ship-crew slice.

## 1. What crew is, in one paragraph

A crew is a small list of named people aboard a ship, at most one of whom is
the pilot. It is pure data: it changes nothing in the physics world (mass,
delta-v, TWR, thrust, inertia all stay exactly as they are) and draws nothing.
It is *attached to a ship instance* — not to the ship design — so two racers
can carry different crews, a runtime-spawned copy inherits the crew of the
ship it was copied from, and removing a ship removes its crew. Crew has one
real consequence in the game logic: **staging the controller part (the
cockpit) away loses the crew** — which makes staging a decision instead of a
free one, and reuses the existing `controller` semantics rather than
inventing a new "seat" concept.

## 2. What the codebase gives us

Facts verified against the tree at the time of writing (line numbers are
`src/main.cpp` unless noted):

- **Ship = `Vehicle`** (main.cpp:935). Per-part state is parallel vectors —
  `partDefs`, `partStages`, `partResources` — all set by `build_ship` (1975)
  before `init()`, and all rebuilt by `separateStage` (1382). Crew state fits
  this pattern exactly, except it does not vary per part, so it is one vector
  on the ship, not a parallel array.
- **Instance vs design already split.** `res/ships/*.json` is the design
  (part tree); `res/fleet.json` is the instance (`ship`, `name`, `body`,
  `scenario`, parsed in `src/fleet.cpp` into `FleetEntry`, `src/fleet.h`).
  `Vehicle::name`/`defPath` (936-938) keep the split at runtime. Crew belongs
  on the instance side, next to `name`/`body`/`scenario`.
- **`controller` is the existing "occupied part"** (950, 1069: "the cockpit
  part"). It drives camera focus, `NeverSleep`, and the staging fallback in
  `separateStage` (step 5, ~1445): if the controller part is dropped,
  `controllerIndex` falls back to part 0. That branch is precisely where crew
  loss hooks in — the "was the cockpit dropped?" computation
  (`split.newIndexOf[oldCi] < 0`) already exists.
- **Every ship in `res/ships/` has exactly one capsule** (`res/parts.json`,
  3 sizes; `capsule`, `capsule_r3h6`, `capsule_r5h10`). 4 of 6 ship defs name
  it as `controller`; **racer and transporter omit `controller`, so the
  default (last part) is `engine_1`** — see §6.
- **Staging state machine** is complete: `activeStage()` (1181),
  `numStages()` (1199), `separateStage()` (1382), pure
  `computeStageSplit` (`src/shipdef.cpp:358`, unit-tested in
  `tests/test_stage.cpp`), e2e anchor `Stage: dropped`
  (`e2e/cases/02-staging-basic.txt`).
- **Runtime spawn/remove** (`spawn_ship` 4072, `remove_ship` 4088,
  `select_ship` 4036, `place_ship` 3385 — returns the new ship's index, so
  state can be assigned on the returned `Vehicle` without signature churn)
  is the established path for "a new ship appears" and needs only a one-line
  crew assignment each.
- **Resources scaffold**: `ResourceType` (`src/shipdef.h:80`) already defines
  `EC, Oxygen, Water, Food` with full `ResourceContent` capacity plumbing
  (`Vehicle::partResources`, RESOURCES window). Unused today; the natural
  hook for a later life-support rule. No part in `res/parts.json` uses them.
- **No crew concept exists yet**: grep for `crew|pilot|occupant|seat|astronaut`
  across `src/` and `res/` returns nothing (only the "Autopilot" UI label).
  No save path exists either (the only "save" in `src/` is PNG screenshots),
  so crew persistence is simply "whatever the JSON says at start".
- **UI pattern**: `ui::Options` + `add_ui_window(name, label, opts)`
  (main.cpp:3910-3929) + `ui::Window(name, opts, body)` in the render section
  (`src/ui.h`, inlined Begin/End). VESSEL (5562), SHIP PARTS (5578) and SHIPS
  (5520) are the per-ship display windows crew joins. A new window is ~20
  lines and automatically joins the Windows menu and TAB toggle.
- **Test patterns**: GL-free unit tests (tests/test_fleet, test_shipload,
  test_stage), `--selftest-stage/rails/spawn` CLI flags (2946-2960, bodies
  4125-4235) that print `selftest-*: ... OK/FAIL` anchors, and e2e cases
  (`e2e/run.py`) that assert on **stdout only** — so every crew rule gets a
  printed anchor, and the UI is verified by eye.

## 3. Design decisions

### 3.1 Crew lives on the ship instance (fleet.json), not the design

**Decision:** `"crew"` is a new optional field of a `fleet.json` entry.

```json
{
  "ships": [
    { "ship": "res/ships/racer.json", "name": "racer", "scenario": "pad",
      "crew": [
        { "name": "Jeb", "pilot": true },
        { "name": "Bo" }
      ] }
  ]
}
```

Rationale:
- Instance-level fields (`name`, `body`, `scenario`) already live here; crew
  is the same kind of fact. The ship def stays a pure geometry/behavior
  description — putting crew in `res/ships/*.json` would couple "who is
  aboard" to "how it is built".
- A runtime-spawned copy is an *instance* too; its crew is defined by
  inheritance from the source ship (§3.4), not by the def.
- A global roster (`res/crew.json`) is a clean follow-up (§7) and does not
  block this: a crew entry is just `{name, pilot}` today.

Rejected alternative: ship-def-level `"crew"`. One fewer file to touch, but
then two instances of the same def are forced to share a crew and the
spawn-inheritance rule becomes implicit.

### 3.2 Crew member: name + role, one pilot max

```cpp
struct CrewMember {          // src/fleet.h (GL-free, like the rest of fleet)
    std::string name;
    bool pilot = false;
};
```

Validation in `load_fleet` (same `std::runtime_error` style as the rest of the
parser, naming the file + entry):
- `crew` optional; absent or empty = unmanned ship (tanker stays that way);
- each entry: non-empty `name` string, `pilot` optional bool;
- **names unique within a ship** (two "Jeb"s aboard one ship is a data bug);
- **at most one `pilot` per ship**; a ship may have crew and no pilot
  (unmanned-but-staffed is a data state we allow; the UI says "no pilot").

### 3.3 The one rule with game consequence: staging the cockpit loses the crew

`separateStage` (1382) already computes, in step 5, whether the controller
part survived the split (`split.newIndexOf[oldCi] < 0`). The hook is:

```
if (the controller part is in the dropped set AND crew is non-empty)
    crew is lost; print the anchor; clear the crew.
```

- The ship itself keeps flying (existing fallback: `controllerIndex = 0`) —
  crew loss is data loss, not a crash. This is deliberate for early dev;
  "unmanned ship still controllable" is an open question (§7), not a bug.
- Note the rule is tied to the **controller part**, not "any capsule". That
  is what makes the racer/transporter `controller` default worth fixing
  (§6.1): with `controller` = engine, staging the engine would lose the crew.

Why this is the right granularity: it costs ~4 lines in a function that
already special-cases the controller, it is fully determined by existing
state (no new bookkeeping), and it is exactly the situation the fallback
branch anticipates.

### 3.4 Spawn / remove / select semantics

- **Spawn a copy** (`spawn_ship` 4072, SHIPS-window button): the copy
  **inherits the source ship's crew** (a plain copy of the data; the "same
  person" is not yet an identity — see §7). Printed, so e2e can anchor it.
- **Remove a ship** (`remove_ship` 4088): the crew goes with the `Vehicle`
  (member of the deleted object). No roster to return them to; nothing to do.
- **Select / F6** (`select_ship` 4036): crew stays with its ship; no change.
- **Rails / warp / SOI switch**: crew is inert data; no interaction.

### 3.5 Explicit non-goals

- No mass: `getMass()` (1168), `getDeltaV()` (1161), TWR, thrust, inertia
  (`getInertia()`) are untouched. A crewed ship weighs exactly as much as an
  unmanned one.
- No 3D presence: no model, no seat, no cockpit interior, no per-person
  rendering.
- No life support: `EC/Oxygen/Water/Food` capacities stay unused (the
  scaffold is there for a later rule: consume per crew member per simulated
  time, kill or degrade the crew at 0).
- No crew actions, no AI, no per-person survival simulation (G-loads,
  reentry heating).
- No save/load: the game has no save path at all; crew state at start =
  what the JSON says.

## 4. UI

All three touch points follow existing patterns; no new window machinery.

### 4.1 VESSEL window (main.cpp:5562)

After the `Stage:` line:

```
Crew: 2
Pilot: Jeb
```

Unmanned ships print `Crew: 0 (unmanned)` and no pilot line.

### 4.2 SHIPS window (main.cpp:5520)

Each ship's button label gains a crew count when crewed: `racer (2)`. The
legend line under the list becomes:
`click name - select    x - remove    (n) - n crew aboard`.
(The label is stdout-irrelevant; e2e anchors never reference it.)

### 4.3 CREW window (new)

Registered like its siblings (main.cpp:3910): `o_crew = info_opts(
ui::Slot::MiddleLeft)`, `o_crew.default_open = false` (closed by default,
like TELEMETRY), `add_ui_window("CREW", "Crew", o_crew)`. Body (drawn in the
render section next to VESSEL/SHIP PARTS, modeled on SHIP PARTS at 5578):

```
Ship: racer
Jeb — pilot
Bo  — crew
```

or `No crew aboard 'tanker'.` for an unmanned ship. ~20 lines.

### 4.4 Stdout anchors (what e2e actually sees)

E2E asserts on stdout only, so the rules print:

- At start, per crewed ship (fleet loop, main.cpp:3657):
  `Crew aboard 'racer': Jeb (pilot), Bo`
- On spawn with inherited crew (spawn_ship):
  `Spawned 'racer #2' (ship 2 of 5); crew inherited: Jeb (pilot), Bo`
- On crew loss (separateStage, step 5):
  `Crew: 2 lost aboard 'stager' (controller dropped)`
- Unmanned ships print nothing at start (absence is the signal).

## 5. Tests

1. **Unit (GL-free)** — extend `tests/test_fleet.cpp` (already links
   `src/fleet.cpp`, plain CHECK macro, `make test`):
   - crew parses: names, pilot flag, order preserved;
   - absent `crew` → empty vector;
   - errors: two pilots, duplicate name, empty name, `crew` not an array,
     entry not an object — each throws naming the file.
2. **Crew-loss path needs a ship where the controller is on a droppable
   stage.** None of the six current ships can do it: in `stager.json` the
   capsule is on stage 2 (the top), and dropping the *only* remaining stage
   is refused (`dropped == n` guard in `separateStage`). So the test plan
   adds one small def, `res/ships/capsule_low.json`:

   ```json
   { "name": "capsule_low",
     "controller": "capsule_1",
     "parts": [
       { "part": "capsule",        "id": "capsule_1", "stage": 1 },
       { "part": "reaction_wheel", "id": "wheel_1",   "parent": "capsule_1", "stage": 1 },
       { "part": "fuel_tank",      "id": "tank_1",    "parent": "wheel_1",   "stage": 2 },
       { "part": "engine",         "id": "engine_1",  "parent": "tank_1",    "stage": 2 } ] }
   ```

   Dropping stage 1 removes the controller → crew lost, the stage-2 payload
   keeps flying. (New file only — no existing ship def changes for this.)
   Not added to the default fleet; reached via `--ship`.
3. **Selftest** — `--selftest-crew` flag, modeled on `--selftest-spawn`
   (2959, 4193): verify the startup fleet's crew against `fleet.json`
   (which ships are crewed, who pilots), then drive the crew-loss path on a
   controller-on-active-stage ship and check the crew is gone afterwards;
   print `selftest-crew: all checks passed` / `FAIL (...)`.
4. **E2E** — new case `e2e/cases/18-crew.txt` (format per
   `e2e/cases/02-staging-basic.txt`):

   ```
   NAME crew-basic
   ARGS --ship res/ships/capsule_low.json --scenario pad --time-accel 1 --timeout 12
   ARGS --sim-press 2000,100,SPACE
   EXPECT Crew aboard
   EXPECT crew lost
   EXPECT Stage: dropped
   FORBID GL_
   FORBID error:
   ```

   (A second case without the SPACE press, asserting only the start anchor,
   is optional; the first covers both anchors.)
5. **UI**: verified by eye (per project convention, screenshots only on
   request). The VESSEL/SHIPS/CREW lines are pure `ImGui::Text` over
   already-tested state.

## 6. Oddities found while mapping this (flag per project convention)

1. **racer and transporter have an engine as their "cockpit".** Both omit
   `controller`, so the default (last part, `ShipDef::controllerIndex()`,
   `src/shipdef.h:187`) resolves to `engine_1`. Everything keyed on
   `controller` — camera focus, `NeverSleep`, the staging fallback, and now
   crew loss — is attached to an engine. Fix as part of this work: add
   `"controller": "capsule_1"` to `res/ships/racer.json` and
   `res/ships/transporter.json` (data-only, one line each).
2. **Stale "reserved" comments.** The `shipdef.h` header and the `stage`
   comment in `load_ship_def` (`src/shipdef.cpp`) still say stage is
   "RESERVED for staging … no runtime effect yet". Staging is fully
   implemented (`separateStage`, `tests/test_stage.cpp`, e2e case 02). Update
   the comments while these files are open for crew.
3. **SHIPS window registration vs draw.** The SHIPS window *draw* is
   unconditional (main.cpp:5520, since the spawn/remove commit), but its
   menu registration is still `if (ships.size() > 1)` (3921). With a
   single-ship fleet the window is drawn open by default yet absent from the
   Windows menu. Make the registration unconditional too.
4. **`Body::mass` duplication.** `consumeResourceMass` (1138) adjusts
   `parts[i]->mass` with the comment `/* why does Body have mass at all? */`,
   and `getFuelMass` carries a `/* eh */` in its signature. The ship's mass
   is really the sum of Bullet rigid-body masses; `Body::mass` is a shadow
   copy. Out of scope here, but it is a landmine for anything (crew mass
   included) that later wants to add weight to a ship.
5. **`Vehicle::getMass()` is unmarked** ("TODO should be cached per frame",
   1167) — fine for now; just don't call it in a hot loop.

## 7. Open questions / follow-ups (not in scope)

- **Crew identity (roster).** Today a spawned copy *duplicates* the crew
  data — two ships can claim the same "Jeb". The natural follow-up is a
  global `res/crew.json` roster: crew entries reference roster names, a
  person is aboard at most one ship, and removing a ship returns its crew to
  the roster. This report's schema (`{name, pilot}`) stays valid under it.
- **Crew transfer UI.** Swap pilot / move crew between two crewed ships from
  the SHIPS or CREW window. Needs the roster for identity to make sense.
- **Life support.** `ResourceType::{EC, Oxygen, Water, Food}` already exist
  (shipdef.h:80) with capacity plumbing; a per-crew consumption rule is the
  obvious next use.
- **Pilot as a capability.** Should an unmanned (or pilot-less) ship be
  flyable? Kept: yes — the player is the fallback pilot; crew is a record,
  not a permission. Revisit if crew actions land.
- **Crew consequences.** G-loads, reentry heat, cabin pressure — all need
  physics presence, which is explicitly out of scope here.

## 8. Concrete edit list

| # | File | Change |
|---|------|--------|
| 1 | `src/fleet.h` | `CrewMember` struct; `FleetEntry::crew`; header schema comment |
| 2 | `src/fleet.cpp` | parse + validate `crew` (unique names, ≤1 pilot) |
| 3 | `src/main.cpp` | `Vehicle::crew` + `pilot()` helper (near 935) |
| 4 | `src/main.cpp` | fleet loop (3657): assign `fe.crew` after `place_ship`, print anchor |
| 5 | `src/main.cpp` | `spawn_ship` (4072): inherit source crew, print anchor; SHIPS button passes it |
| 6 | `src/main.cpp` | `separateStage` (~1445): crew-loss hook in the controller-dropped branch |
| 7 | `src/main.cpp` | VESSEL (5562) crew/pilot lines; SHIPS (5520) label count + legend |
| 8 | `src/main.cpp` | `o_crew` options + `add_ui_window("CREW", …)` (3910) + window body |
| 9 | `src/main.cpp` | `--selftest-crew` flag (2959) + body (4193 pattern) |
| 10 | `res/fleet.json` | crew on racer (crewed) / others (unmanned stays tanker) |
| 11 | `res/ships/racer.json`, `res/ships/transporter.json` | explicit `"controller": "capsule_1"` |
| 12 | `res/ships/capsule_low.json` | new test def, controller on a droppable stage (§5.2) |
| 13 | `tests/test_fleet.cpp` | crew parse + error tests |
| 14 | `e2e/cases/18-crew.txt` | staging-loses-crew case (§5.4) |
| 15 | `src/shipdef.h`, `src/shipdef.cpp` | fix stale "RESERVED … no runtime effect" comments (§6.2) |
| 16 | `src/main.cpp` | make SHIPS registration unconditional (§6.3) |

Verification: `make clean && make test && make e2e` before committing (per
project convention).
