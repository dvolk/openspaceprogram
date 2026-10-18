# State handling: saves/loads, scene transitions, and what to do about it

Snapshot taken 2026-10-18, on top of `a1e3236`. Ideas only — nothing here is
implemented unless it names a commit. Three bugs found along the way were fixed
because they were crashes in code the vehicle-list refactor had just touched;
everything else is left as notes for discussion.

---

## 1. Why this was worth looking at

The refactor series (`a5fe935` → `bdf31ff`) moved vehicles from
`TerrainBody::ships` into one flat `Game::vehicles`, and #297 gave them a single
death path (`retire()` + one deferred `sweepRetired()` per frame). That fixed the
worst instance of a pattern that is still everywhere: **state that is invalidated
by hand, where the only defence is remembering to invalidate it.**

The adversarial review over the refactor found two live memory bugs. Fixing them
meant reading the teardown and transition paths properly, which surfaced the rest
of this.

### Fixed during this pass

| | |
|---|---|
| `ea8ecca` | `ui_remove_vehicle` printed `active unchanged: <ship>` reading `ship` blind. A game can legitimately hold vessels with **none** of them active (`recoverActive()` detaches the last one without a handoff), so that read segfaulted. Reproduced at exit 139; case 257 pins it. |
| `ea8ecca` | `unloadGame()` deleted the vehicles but left `TransferPlanner::xferTargets` pointing into them — a row's `name` is a `const char*` into `Vehicle::name` and `.ship` **is** the Vehicle. Valgrind: `Invalid read of size 1` in `selectTargetByName`, block freed in `Game::unloadGame()`. New `dropTargets()`; verified load-bearing by disabling it and watching the error return. |
| `ea8ecca` | `newGame()` was the one bulk reset that skipped `sweepRetired()`, so its `!vehicles.empty()` guard could not see a retire queued the same frame. |
| `a1e3236` | The #231 pad-clear card captured the pad's `TerrainBody*` even though `game.h:312` states the rule for every `Question`: *capture identity, never a pointer*. Reproducible **bus error** (exit 135) via `--switch-system` while the card is up; case 258 pins it. |

Four comments also claimed "`~Vehicle` dereferences its frame". It does not — it
calls `RemoveBody` and frees its own `Body`s. The ordering invariant (vehicles die
before bodies) is still right, for a different reason: a **live** `Vehicle` holds
`m_parent` and its `frame`, both owned by the body. Restated rather than deleted.

---

## 2. How state actually works today

### The reset points are six, and they are not nested

```
boot            main.cpp:613-722   locals first, then Game ctor, then heavy phase
newGame         game.cpp:757       requires an empty world
switchSystem    game.cpp:998       load new system -> unloadGame -> delete old bodies -> title
unloadGame      game.cpp:958       the only real teardown
load_game       save.cpp:812       detach old, build new, adopt, delete old
scene change    scene.cpp:203-273  push / pop / setBaseScene
```

`quitToTitle` is `unloadGame` + `enterTitle`. `startNewGame` is a system switch
plus `newGame`. So there is really **one** teardown (`unloadGame`) and four
callers with different ideas about what else needs clearing. That asymmetry is
the source of most of what follows: `unloadGame` clears world-facing state and
nothing else, and each caller adds its own handful of extras by hand.

### The frame

```
sweepRetired -> timeout/save -> EVENTS -> hooks(preScene) -> scene.update
-> hooks(postUpdate) -> jobs.poll -> audio -> scene.refresh -> hooks(postRefresh)
-> scene.paint -> scene.drawUi -> drawQuestion -> drawToasts -> ImGui::Render -> swap
```

The scene is re-read at each phase boundary (`main.cpp:1404`, `1422`, `1472`)
because a launch collapses the stack. `refresh` is 3D *state*, `paint` is pixels
(#212). Two scenes (`tracking`, `research`) deliberately do not paint at all.

### The scene stack

`SceneFrame` holds a parked `CameraSnapshot` describing the scene **below** it.
`pushScene` captures, `popScene` restores, `setBaseScene` (behind `enterFlight` /
`enterTitle` / `enterSpaceCenter`) discards — deliberately, because "the ship
being flown now is not the one the excursion started from". `goScene` (the `1/2/3/4`
shortcuts) pops and therefore **restores**; `enterFlight` collapses and therefore
**discards**. Pressing `2` throws away the hub's framing; clicking "Back to Space
Center" keeps it. Both are reachable from the same UI state.

Window state is one global `ui::Manager` static keyed by window *name*, so open
state and layout carry across every transition; only *which windows exist* is
per-scene (the `WinSet` gate, evaluated at call time against `curScene`).

### Save/load

`format 4`. `save.json` plus one file per vehicle, keyed by **slug** (`v0`, `v1`, …)
rather than name (#266). Three crew representations for three states — seated (a
`seats[]` element of its capsule), standing (`crew_pool[]`), walking (its own
`ships/vN.json` with `is_crew`) — and a walking EVA kerbal is fully saveable.
Part `uid`s are deliberately **not** stable across a load; every cross-part
reference is remapped through `savedUidToPart`, which is exactly why `part_sels`
is destroyed on load.

---

## 3. Confirmed problems, filed

| # | one line |
|---|---|
| **#298** | Boot `--load` across systems seeds `focusTargets` **twice**. Measured: 63 entries where 32 is correct. `switchSystem` rebuilds the list during boot, then `main.cpp:969` appends `sys.bodies` again — and `Game::sys` is a reference to main's local, which the switch reassigned. Case 91 runs exactly this command and cannot notice. |
| **#299** | `newGame()` resets neither the clock nor the game identity. A New Game starts at the previous game's epoch; and because `--new-game` calls `newGame()` directly, a later quicksave writes into the **previous game's** `saves/<id>/` directory. |
| **#300** | `SceneDef::sim` is a dead flag — documented as "does the clock advance here", initialised six times, never read. Every scene advances the clock. The comment is actively misleading. |
| **#301** | A **refused** load still moves the running game's clock, warp and exhaust scale: `setTime` → `syncRails()` + `invalidateClockStampedCaches()` happens at `save.cpp:855`, before anything can throw. |
| **#302** | Cross-system loads are **not** transactional: the system switch runs `unloadGame()` (fleet deleted, `science.reset()`) before `load_game` can refuse. One bad byte in a cross-system save destroys the running career. Made worse by basename-only system matching and a verbatim recorded path with no `resdir::path()`. |
| **#296** | Commented with the concrete inventory of raw references that outlive their subject (see §4). |

---

## 4. The pattern worth naming

Five open cases, all the same shape:

1. `pose_jitter_prev_ship` / `_prev_frame` (`game.h:470-471`) — a `Vehicle*` and a
   `Frame*` that nothing nulls at any teardown. Currently safe only because
   `render.cpp:568-598` compares them as identities and never dereferences.
2. `CameraSnapshot::focusBody` (`scene.h:42`) — parked raw `TerrainBody*` in the
   scene stack, dangling the moment `switchSystem` deletes the old bodies. Safe
   only because `setBaseScene` writes `camValid` without reading them.
3. `part_sels` (`game.h:506`) — `Vehicle*` + part index, surviving every scene
   transition because only `flightDrawUi` draws it.
4. `drawUIReadouts`'s local `Vehicle *ship` (`gameui.cpp:340`) — captured, then
   `select_vehicle` / `ui_remove_vehicle` change `Game::ship` mid-function, so
   `W_VesselInfo` reads a **retired** Vehicle that is alive only because
   `sweepRetired` has not run yet.
5. `ShipView::ship` (`game.h:88`) — already #235.

In every case the reference is fine and the *invalidation* is manual. #297 gave
`Vehicle` a lifetime rule; these have none. That is the thing I'd want to fix
structurally rather than case by case.

---

## 5. Ideas, roughly ranked

### A. Give the teardown one owner instead of six partial ones

`unloadGame` clears world-facing state; `newGame`, `switchSystem`, `load_game` and
`quitToTitle` each add their own extras. The gaps fall out of that: `newGame` and
`quitToTitle` never call `invalidateClockStampedCaches()`, so `cache_epoch` is
unchanged and an in-flight Surface Map sweep posted *before* a reset still passes
its epoch guard and publishes *into* the new game (`surfmap.cpp:139-156`).
`surfmap.compute_requested` and `xferPlanner.pc_compute_requested` are never
cleared by anything. `thrust_latched` survives quit, new game, switch and recover.
`toasts` and `question` survive everything.

Shape: one `Game::resetToTitle()` / `Game::resetWorld()` pair that is the only
place this list exists, with the callers reduced to what is genuinely
caller-specific (the crew pool for a new game, the system for a switch). Cheap,
unexciting, removes a whole class of "and this one forgot".

The honest counter-argument: a big reset function can also hide *intent*, and some
of the current asymmetry is deliberate (a system switch preserves the clock; a
load preserves nothing). Worth deciding which asymmetries are policy before
collapsing them.

### B. Decide what a `Question` may hold, and enforce it

`game.h:312` already states the rule. One card broke it and it was a
reproducible crash. Two options:

- keep the rule and make violating it hard — e.g. `ask()` takes ids/strings only,
  and callbacks look things up through a small resolver, so a pointer is not
  available to capture; or
- drop the pending card at every teardown, which is one line and also fixes any
  card we have not thought about.

I'd do the second now and the first if we grow more cards. Note the modal scrim
currently blocks the *interactive* path to a switch, so this class is
hook-reachable rather than player-reachable — which is exactly why the e2e
harness is the right place to pin it.

### C. Make loads actually transactional

Two separate issues (#301, #302) with one root: the destructive part of a load
happens before the fallible part. The fix for #302 is to parse and build the whole
save **before** switching systems — `load_game` already builds the new world
alongside the old one and only swaps at the end (`save.cpp:992-993`), so the
machinery exists; it is the system switch that jumps the gun.

Related, smaller: `save.json` is not written atomically and the ship files are
overwritten in place *before* it (`save.cpp:783-788`, `writeJson` truncating),
with the orphan sweep after. A crash mid-save leaves the previous `save.json`
pointing at partly-overwritten ship files. Write-to-temp-then-rename is the whole
fix.

Also: two `assert`s on the load path (`save.cpp:781`, `:392`) **abort rather than
refuse**, and there is no `-DNDEBUG` anywhere, so the release build is the assert
build. A malformed save should be a refusal, not a crash.

### D. The camera/focus index problem

`focusBody` is an index into a vector that `syncShipFocus` inserts into and erases
from, that `switchSystem` rebuilds, and that #298 shows can be seeded twice.
`restoreCamera` resolves a body pointer back to an index with **no fallback** if
the body is gone (`scene.cpp:170-174`), after which `render.cpp:301-302`'s assert
fires in debug and indexes out of range in release.

This wants focus to be an identity (a name, like `TargetRef` already is for the
transfer planner) with the index derived per frame — the same fix #276 applied to
`xfer_sel`. #298 is the visible symptom, not the cause.

### E. Telemetry series are never cleared

`ShipView::energy_series` / `angmom_series` (`game.h:135-136`) are only ever
pushed to; `TimeSeries` (`siminput.h:13-44`) has no reset. A load moves the clock
**backwards**, so the Telemetry window plots a non-monotonic x-axis spanning the
previous ship, the previous game and the pre-load epoch. One `clear()` at the
right reset points; needs (A) to know where those are.

### F. Mid-frame transitions

Most transitions happen in the events phase or a hook phase, which is clean. A
large set happen inside `sc.drawUi` — after `refresh` and `paint` already ran for
the outgoing scene. Consequences:

- `winInScene` resolves `curScene` at call time, so a `popScene` inside the first
  widget group re-scopes every *later* window in the same frame. "Fly" from the
  Tracking Station collapses to Flight and the flight HUD draws in a frame whose
  paint was the tracking map — which paints nothing at all.
- `unloadGame` frees Vehicles **immediately**, bypassing the retire/sweep
  deferral everything else relies on. It is safe today only because
  `drawSaveLoad` is last in every `*DrawUi` and `drawQuestion` runs after it. Any
  reordering turns the stale `Vehicle *ship` locals into use-after-free.

Not a bug, a fragility. If we ever want a transition from the middle of a widget
group, the cheap guard is deferring scene changes to the top of the next frame —
the same trick #297 used for Vehicle death, and it would make the two lifetimes
consistent.

### G. Small things that are worth an hour each

- `ensure_ui_visible()` on every transition overwrites every `Persistent`
  window's open state with `default_open`, silently discarding what the player
  opened from the Windows panel. `Transient` windows are exempt — an asymmetry
  with no comment saying so.
- `main.cpp:1466`'s "No scene opts out of the paint yet" is stale and its warning
  is already live: F12 in tracking/research captures only the clear colour plus
  imgui.
- `curScene(g).keys(...)` is dispatched *after* the scene-neutral slot block in
  the same key event (`events.cpp:541-558`), so a shortcut key is re-interpreted
  by the destination scene. Harmless with today's bindings.
- `save.cpp`'s part array order is load-bearing (a child must follow its parent,
  and the file index doubles as the `Vehicle::parts` index) but nothing on disk
  records that it matters. Part windows and `--vab-assign` address parts by index.
- Dead keys in the schema: `"slot"` in six fixtures with no reader;
  `parts`/`experiments`/`kerbals`/`time_accel` written and never consumed.
- `--drag-cd` is not in the save while `exhaust_scale` is, so a save's
  aerodynamics are not reproducible while its thrust is.
- `onRails` is re-tested rather than restored (`vehicle.cpp:3050-3053`): a
  suborbital arc saved on the rails comes back integrated.

---

## 6. What I'd do first, if pushed

1. **#298** — it is confirmed, one line, and makes the G-cycle visibly wrong.
2. **Atomic saves** (§5C, the write-temp-then-rename part) — cheap insurance
   against losing work, independent of everything else.
3. **#302** — a load that destroys a career is the worst user-visible failure here.
4. **(A)** the single teardown owner — least fun, highest ceiling on the number of
   future bugs prevented.
5. **(D)** focus-by-identity — bigger, and it subsumes #298 properly.

Everything in §5G is an hour each and can wait for a rainy day.

---

## 7. Verification state

`make -j8` clean, no warnings. `make test` exit 0, 15 suites, 0 failures.
37/37 targeted e2e green, including the two new cases (257, 258). Full e2e has
not been run, per your steer. The three fixes were each confirmed by a failing
repro first: exit 139, valgrind `Invalid read`, exit 135 respectively.
