# UI + scene architecture for a multi-mode game — analysis and refactor plan

Snapshot of the UI / game-mode architecture as of 2026-09-17 (commit `7ed16c0`,
"update QWEN.md"). Written because the UI system was designed for one game mode
(the flight scene) and we now have three places the player can be — flight, the
VAB, and the game-start menu — with more coming. The symptom that prompted this:
the start menu renders the flight windows behind it (all empty, no ship), and the
menu itself is closable even when there is no game to go back to.

Code references are `src/game.h` / `src/game.cpp` (the `Game` state + the window
registry + the control transitions), `src/main.cpp` (the boot + the main loop's
scene branches), `src/events.cpp` (the input dispatch), `src/gameui.cpp` (the
imgui pass), `src/ui.h` (the window wrapper), `src/vab.cpp` / `src/vab.h` (the
editor + its scene transitions), `src/save.cpp` / `src/save.h` (the scene field
in the save format), unless noted.

Decisions already taken (this document assumes them):

1. **Full scene stack** — scenes push and pop, so a scene returns to whatever
   opened it, rather than a single `currentScene` variable.
2. **Flight implies an active vessel** — a shipless boot and a shipless save both
   land in Title, which makes the no-ship flight state unreachable.

## TL;DR

`Scene` is one enum doing four jobs (what simulates, what the 3D pass draws, what
the UI draws, what input means), and a fifth thing — *is a game in progress* — is
pretending to be a scene. The start menu is not a place; it is the conjunction of
three unrelated facts that each drift independently:

```
scene == Flight   &&   g.ship == nullptr   &&   ui::IsOpen("Main Menu")
```

That conjunction is the actual bug. The flight windows draw because they only
know `scene == Flight`; the menu is closable because it is just a window; every
vessel window has to null-guard `ship` individually (7 `"No active ship."`
sites); and `main.cpp:549` has to force the menu open at boot to paper over the
whole thing.

The fix is to make *where you are* a single first-class value that owns its
update, its draw, its input, **its window set** and its state, and to keep those
in a stack. Two rules then do all the work:

- **A scene's window set is enforced at draw time.** The Title scene's `drawUi`
  only issues `ui::Window` for Title windows, so the flight windows cannot appear
  there — by construction, not by guarding. Nothing needs to close them.
- **Flight is only entered with a vessel.** The 7 guards and the forced
  `SetOpen("Main Menu")` both delete.

The main menu splits in two, which is what fixes the closability complaint: the
**Title scene's** root UI is not closable (there is nothing to go back to), while
**Flight's** Esc menu is a pause window (closable, with Resume on it).

The refactor is staged so that the first three stages are invisible to the player
and independently testable, and stage 4 is the payoff. Net effect on the code is
strongly negative in lines: ~34 `vab_*` fields, 19 `ui::Options` fields, the
`UiWin` registry and `setup_ui_windows()` (~128 lines), 9 camera-park fields and
their hand-rolled restore, 7 scene branches in `main.cpp`, 2 in `events.cpp`, and
the 7 null guards all collapse into one static table per scene.

---

## 1. What exists today

### 1.1 The scene variable

```cpp
// game.h:163-173
enum class Scene { Flight, Vab };
struct Game { ... Scene scene = Scene::Flight; ... };
```

`g.scene` is read or written at 27 sites across four files. The load-bearing ones:

| Site | What it decides |
|---|---|
| `main.cpp:868` | LOGIC phase: the VAB hover/pick/place block, or `tick(game)` |
| `main.cpp:948` | the clear colour (studio gray vs black) |
| `main.cpp:956` | the 3D pass: `drawVab` vs `draw3d` |
| `main.cpp:974` | the imgui pass: `drawVabUI` vs the five flight draw calls |
| `main.cpp:853,863` | the `--vab-load` / `--vab-launch` headless hooks |
| `main.cpp:503` | applying `--vab-body` / `--vab-scenario` at boot |
| `events.cpp:438` | keys: `vabKeyActions` vs `flightKeyActions` |
| `events.cpp:464` | RMB click-to-pick a part (Flight only) |
| `save.cpp:343,361` | `meta.scene` round-trip ("flight" \| "vab") |
| `vab.cpp:534,588,603` | the transitions themselves (`vabLaunch`, `vabOpen`, `vabClose`) |

So adding a mode today means touching five places in `main.cpp`, two in
`events.cpp`, one in `save.cpp`, the window registry, and then hoping the
per-window null guards cover the new state.

### 1.2 The third mode is not a mode

Booting with neither `--ship` nor `--body` produces what the comments call the
"orbit view" state (`main.cpp:547-551`):

```cpp
ui::SetOpen("Main Menu", true);
printf("[boot] no ship: orbit view of %s\n", home->name.c_str());
```

That is the game-start main menu. It is `Scene::Flight` with `g.ship == nullptr`
and one window forced open. Consequences, all of them structural rather than
bugs to be fixed one at a time:

- `drawUIReadouts`, `drawUIMap`, `drawPartWindows` and `drawSaveLoad` all run,
  because the branch is on `scene` alone. Seven windows open with
  `if(ship == nullptr) { ImGui::Text("No active ship."); return; }`
  (`gameui.cpp:1088, 1195, 1255, 1330, 1420, 1453, 1703`), the HUD guards inline
  (`gameui.cpp:216`), and the Surface Map branches on `ship` at three more sites.
- The menu is closable (`Esc`, or its own "Back to game" button) because it is an
  ordinary window with no idea that it is the only UI standing. Closing it leaves
  an empty planet with no visible way to reach the VAB or a save.
- `sim` runs (the clock advances, bodies propagate) which is *desirable* — it is
  what makes the backdrop alive — but it is accidental rather than chosen.
- The state is also reachable at runtime from `load_game` of a shipless save,
  which is why `Game::syncShipFocus()` has to handle entering *and* leaving it.

### 1.3 The UI layer has no ownership model

`src/ui.h` is a good, well-factored window wrapper: 9 slots, sibling anchoring
(`left_of` / `right_of` / `below`), fixed and fixed-width variants, and a
generation counter for full relayout. Its state is one global
`name → WinState` map. There is no concept of a *set* of windows, and nothing
above it supplies one:

- `Game` carries **19 loose `ui::Options` fields** (`o_hud`, `o_orbit`, …
  `o_mainmenu`), all assigned once in `Game::setup_ui_windows()`
  (`game.cpp:38-166`, ~128 lines).
- `Game::ui_windows` is a **second**, parallel registry of `UiWin{name, label,
  opts, in_windows_list}` used by the TAB toggle and the "Windows" checkbox
  panel. `UiWin::opts` is a *copy* of the corresponding `o_*` struct and is read
  in exactly one place — `toggle_windows()` at `game.cpp:165`, for
  `default_open` only. Two sources of truth for the same window that can silently
  drift; the other ~95% of the copied struct is dead weight.
- `ui_visible` is one global TAB flag, and `drawSaveLoad` / `drawVabUI` each
  early-return on it by hand.
- The VAB's two main panels bypass the wrapper entirely: `ImGui::Begin("VAB")`
  and `ImGui::Begin("Palette")` (`gameui.cpp:2425, 2557`) are raw imgui windows
  with no slot layout, no open state, no reset and no registry entry, while the
  VAB TopBar right above them (`gameui.cpp:2363`) is a proper `ui::Window`. Two
  UI systems running in parallel inside one scene.

### 1.4 Scene state lives flat on the god object

`Game` is ~200 fields. The mode-specific clusters:

- **34 `vab_*` fields** (`game.h:176-240`): the build tree, the launch config,
  hover/selection, the whole ghost-preview state (10 fields), symmetry and snap
  flags, link mode, subassemblies, and the LMB edge-detect.
- **9 camera-park fields** (`vab_camSaved`, `vab_camMode`, `vab_camPos`,
  `vab_camFwd`, `vab_camUp`, `vab_camDistance`, `vab_camYaw`, `vab_camPitch`,
  `vab_camFocusBody`) plus the hand-rolled save in `vabOpen` and restore in
  `vabClose` (`vab.cpp:545-639`) — including an `else if(g.ship != nullptr)`
  fallback that exists purely because a `--vab` boot has no parked pose.
- **~10 map / surfmap fields** and the perf series.

`vabOpen` also doubles as "re-aim the editor camera after a Load", which is why
it carries an `entering` flag (`vab.cpp:587`) to suppress its own toast — a
transition function with a "am I really transitioning?" argument.

---

## 2. Diagnosis

Three separable problems, in the order they should be fixed:

**(a) "Where you are" is not a thing.** It is four booleans and a pointer,
spread over `scene`, `ship`, `ui::IsOpen("Main Menu")` and five switch sites.
Every new mode multiplies the combinations, and the combinations are what break.

**(b) Windows have no owner.** Nothing in the system knows that ORBITAL is a
flight window and Settings is everywhere. The registry is flat and global, so the
only way to keep a window from appearing somewhere is to guard inside its body —
which is exactly the 7 `"No active ship."` sites.

**(c) Mode state has no home.** It accumulates on `Game` with a mode prefix, and
mode transitions are hand-written per mode (the camera park being the clearest
example).

(a) and (b) are the ones causing the reported symptoms. (c) is what makes each
future mode expensive.

---

## 3. Target design

### 3.1 Data

```cpp
// scene.h
enum class SceneId : int { Title, Flight, Vab, COUNT };

/* One window belonging to one or more scenes. The options live HERE now --
   static, one home, no copy. Shared windows (Settings, Controls, Save/Load,
   Game Debug Info, Telemetry) simply appear in more than one table. */
struct WinDef {
    const char *name;      // imgui window id (unique game-wide)
    const char *label;     // the "Windows" panel row
    ui::Options opts;      // slot / anchoring / size / flags / default_open
    bool inList;           // in the Windows panel, vs toggled from its own context
};

enum class Backdrop { Sky, Studio };   // clear colour + skybox, or the VAB gray

struct SceneDef {
    const char *name;      // "title" / "flight" / "vab"  (logs, save meta)
    bool sim;              // top-of-stack && sim  ->  tick() runs
    Backdrop backdrop;
    void (*enter)(Game &);   // pushed onto the stack
    void (*exit)(Game &);    // popped, or replaced
    void (*update)(Game &);                       // the LOGIC phase
    void (*draw3d)(Game &, TransferPlanner &);
    void (*drawUi)(Game &, TransferPlanner &);    // draws exactly wins[]
    void (*keys)(Game &, SDL_Scancode, Uint16 mod, bool repeat);
    const WinDef *wins; size_t nWins;             // THE window set
};

extern const SceneDef kScenes[(size_t)SceneId::COUNT];
```

The stack itself, on `Game`:

```cpp
struct CameraSnapshot {          // replaces the 9 vab_cam* fields
    CameraMode mode; glm::dvec3 pos, fwd, up;
    double distance, yaw, pitch;
    TerrainBody *focusBody;      // a body, not an index (the list shifts)
};

struct SceneFrame {
    SceneId id;
    CameraSnapshot cam;          // the camera as the scene below left it
};

std::vector<SceneFrame> sceneStack;   // back() is live; never empty
```

A function-pointer table rather than a virtual `Scene` base: it matches the
existing "free functions taking `Game&`" style, keeps all state inspectable in
one place, and costs one indirect call per phase per frame. Scene-local state
groups into structs on `Game` (`VabState g.vabui`) instead of moving onto heap
scene objects.

### 3.2 Stack operations and their rules

```cpp
void pushScene(Game &, SceneId);     // snapshot the camera, enter() the new top
void popScene(Game &);               // exit() the top, restore the camera
void replaceScene(Game &, SceneId);  // exit() the top, enter() the new one
void enterFlight(Game &);            // collapse the whole stack to [Flight]
const SceneDef &curScene(const Game &);
bool sceneOnStack(const Game &, SceneId);
```

Rules, chosen to keep the semantics boring:

1. **Only the top scene updates, draws and receives input.** Lower scenes are
   fully suspended — no draw-behind, no second 3D pass. If we ever want a
   see-through overlay (a dimmed flight view behind a pause menu), that is a
   *window* in the top scene, not a stacked scene.
2. **The stack is never empty.** `popScene` at depth 1 is a no-op (plus a toast).
   `replaceScene` swaps the single entry. This is what makes `--vab`-with-no-fleet
   behave: it boots `[Title, Vab]`, so "Back to game" pops to Title — correct,
   because there is no game to go back to — where today it drops you into an
   empty flight scene with a closable menu.
3. **Pushing an id already on the stack is refused** (toast + a debug assert).
   VAB-inside-VAB is a bug, not a feature.
4. **Any action that establishes or changes the loaded game collapses the stack
   to `[Flight]`** — `enterFlight`. New Game, Load, and `vabLaunch` all call it.
   One rule, no case analysis over "what was underneath".
5. `sim` is `curScene(g).sim`. Entering the VAB freezes the clock exactly as
   today (the "simulation is paused" toast moves into `vabEnter`). Title keeps
   `sim = true` so the planet spins behind the menu — with no ships that is
   O(bodies) and it is what makes the backdrop alive.
6. Transitions log one line each, as e2e anchors: `[scene] flight -> vab (push)`,
   `[scene] vab -> flight (pop)`, `[scene] -> title (replace)`.

### 3.3 Window sets: enforced at draw time, not by mutating state

This is the key simplification, and it is what makes the stack cheap:

- `SceneDef::drawUi` issues `ui::Window(...)` for exactly its own `wins[]`.
  A flight window while you are in the VAB is simply not drawn.
- **No transition touches window open state.** `ui::Manager` keeps the open flag
  and the user's position for every window, drawn or not. Pop back to Flight and
  everything is exactly as you left it — strictly better than today *and* simpler
  (no open-state snapshotting, no per-scene reset).
- The "Windows" checkbox panel and `toggle_windows()` (TAB) iterate
  `curScene(g).wins` instead of one global registry, so the panel only ever shows
  toggles that do something *here*.
- `ui_visible` (TAB) stays a single global "hide the chrome" flag — it is really
  "clean screenshot", and the early-returns move into the scene `drawUi` entry
  points instead of being repeated per draw function.
- Sibling anchoring already tolerates an absent source: `ui::Manager::axis_from`
  returns true and the slot placement stands when the source is closed or
  unknown (`ui.h`). No change needed.

Deletions this enables: the 19 `o_*` fields, `UiWin`, `ui_windows`,
`setup_ui_windows()`, and the copy-drift between them. `WinDef` tables are static
and live next to the scene that uses them.

### 3.4 The three scenes

**Title** — the game-start place. Backdrop: the world, camera 3 radii out on the
home body (today's orbit view, unchanged). `sim = true`.

- `wins`: `TitleMenu` (fixed, centred, **`closable = false`**), Settings,
  Controls, Game Debug Info, Telemetry, Save/Load.
- `TitleMenu`: New Game → `startGame(default)` + `enterFlight`; Load → the
  Save/Load window; VAB → `pushScene(Vab)`; Settings; Controls; Quit.
- `keys`: Esc does nothing (there is nothing to go back to). Screenshot /
  wireframe / TAB stay global.
- The Save/Load window in Title shows the Load half only — there is no game to
  capture. Gate the Save half on `sceneOnStack(g, SceneId::Flight)`.

**Flight** — invariant: `g.ship != nullptr` for as long as Flight is on the
stack, checked in `enterFlight` (refuse + toast + a debug assert).

- `wins`: today's set (HUD, Windows, Orbital, Surface, Resources, Orbital Map,
  Surface Map, Vessel Info, Ship List, Autopilot, Transfer, Porkchop, Settings,
  Controls, Game Debug Info, Telemetry, Save/Load) plus **PauseMenu**, which
  replaces "Main Menu".
- `PauseMenu`: Esc-toggled, closable. Resume, Save/Load, Settings, Controls, VAB
  (push), Game Debug Info, Telemetry, Quit to Title (greyed until stage 5), Quit
  game. Both menus share a small helper for the common rows, but they are two
  window definitions — that split is the point.
- `keys` = today's `flightKeyActions`, minus the `Menu` slot's window juggling
  (it toggles PauseMenu) and minus anything the invariants make unreachable.
- The 7 `"No active ship."` guards and the HUD's `if(ship)` delete; the Surface
  Map's three `ship` conditionals simplify.

**Vab** — `sim = false`, `Backdrop::Studio`.

- `wins`: VAB TopBar, VAB, Palette — and the latter two should move from raw
  `ImGui::Begin` (`gameui.cpp:2425, 2557`) into the wrapper as `fixed` /
  `initial_size` entries, so the editor chrome gets the same slot layout, DPI
  scaling and reset behaviour as everything else. (Optional; see §9.)
- `enter` = today's `vabOpen` minus the camera park (now the stack's job) and
  minus the `entering` flag (a Load re-aim becomes a plain `vabAimCamera(g)`,
  not a transition).
- `exit` = today's `vabClose` minus the camera restore and minus its
  `else if(g.ship != nullptr)` fallback, which the stack makes unnecessary.
- `update` = the hover / place / link-click block currently inline in main's
  LOGIC phase (`main.cpp:868-905`), moved to `vab.cpp` where the rest of the
  editor lives.
- `keys` = today's `vabKeyActions`, with Esc gaining a third step: link mode →
  disarm → **pop**. (A behaviour change; flagged in §10.)

### 3.5 `main.cpp` after

```cpp
const SceneDef &sc = curScene(game);
emit_sim_events(game);
poll_events(game);                 // routes to sc.keys / the top scene's mouse
sc.update(game);                   // tick() | vabUpdate() | titleUpdate()
game.jobs.poll();
if (game.redraw) {
    ...
    sc.backdrop == Backdrop::Studio ? display.Clear(0.72f,...) : display.Clear(0,0,0,1);
    sc.draw3d(game, xferPlanner);
    ...
    sc.drawUi(game, xferPlanner);
    drawToasts(game);              // scene-neutral
    ...
}
```

Seven branches become zero.

### 3.6 Save format

Drop `meta.scene` (`save.h:134,322,336`, `save.cpp:343,361`,
`tests/test_save.cpp:203,214`). A save records a *game*, not a place: loading
always lands on `[Flight]`, or `[Title]` when the save has no active ship. The
VAB build tree is not part of a save, so restoring `scene == "vab"` today puts
you in an editor whose contents came from somewhere else — incoherent, and the
field's only real use disappears with the stack.

### 3.7 `startGame()`

`main.cpp:315-460` — the fleet-entry assembly, the `--ship` / `--fleet` /
`--radial-test` / `--dock-test` / `--load` branches, `apply_scenarios`, the
active-ship pick, the park-idle-ships-on-rails loop and the focus-target build —
becomes one function:

```cpp
struct StartRequest { /* fleet file, ship files, body, scenario, load dir, tests */ };
bool startGame(Game &, const StartRequest &);   // false + a message on failure
```

The boot path calls it with a request built from `GameArgs`; Title's **New Game**
and **Load** call it too. This is the enabling piece for a Title scene that can
actually start a game, and it is worth doing regardless — it is ~150 lines of
boot logic currently unreachable from anywhere but `main()`.

---

## 4. What this deletes

| Deleted | Where | Replaced by |
|---|---|---|
| `enum Scene` + `Game::scene` | `game.h:163-173` | `Game::sceneStack` |
| 7 `if(scene == Vab)` branches | `main.cpp:503,853,863,868,948,956,974` | `curScene(g).update/draw3d/drawUi` + `SceneDef` fields |
| 2 scene branches | `events.cpp:438,464` | `curScene(g).keys` / top-scene mouse |
| 19 `ui::Options o_*` fields | `game.h` | static `WinDef` tables |
| `UiWin` + `ui_windows` + `setup_ui_windows()` (~128 lines) | `game.h:406-414`, `game.cpp:38-166` | `SceneDef::wins` |
| the `UiWin::opts` copy (drift hazard) | `game.cpp:126,133,165` | one table |
| 9 `vab_cam*` fields + hand-rolled park/restore | `game.h`, `vab.cpp:545-639` | `CameraSnapshot` on the `SceneFrame` |
| `vabOpen`'s `entering` flag | `vab.cpp:587` | `vabAimCamera()` vs `pushScene()` |
| 34 loose `vab_*` fields | `game.h:176-240` | `struct VabState g.vabui` |
| 7 `"No active ship."` guards | `gameui.cpp:1088,1195,1255,1330,1420,1453,1703` | the Flight invariant |
| `ui::SetOpen("Main Menu", true)` at boot | `main.cpp:549` | Title is a scene |
| `numFocusTargets` (a stored `.size()`) | `game.h:404` + 4 sites | `focusTargets.size()` |
| `meta.scene` round-trip | `save.h:134,322,336`, `save.cpp:343,361` | loading lands on Flight/Title |
| the VAB input block in the LOGIC phase | `main.cpp:868-905` | `SceneDef::update` |

---

## 5. Migration stages

Each stage builds, passes `make test` and the full e2e battery, and is separately
committable. Stages 1–3 are invisible to the player.

**Stage 1 — groundwork, no behaviour change.**
Group `VabState`; add `CameraSnapshot` + `Game::parkCamera()/restoreCamera()` and
convert `vabOpen`/`vabClose` to them; move the VAB mouse/hover/place block out of
main's LOGIC into `vabUpdate(Game&)` in `vab.cpp`; extract `startGame()`;
delete `numFocusTargets`; fold `UiWin::opts` into a single registry (interim —
the tables replace it in stage 3). Files: `game.h`, `game.cpp`, `vab.cpp`,
`vab.h`, `main.cpp`. This is the largest diff and the lowest risk: pure moves.

**Stage 2 — the scene table.**
Add `scene.h` / `scene.cpp` with `SceneDef`, `kScenes[]`, the ops for the two
existing scenes, and the stack (`pushScene` / `popScene` / `replaceScene` /
`enterFlight` / `curScene`). `vabOpen`/`vabClose`/`vabLaunch` become thin
wrappers over the stack ops. `main.cpp` and `events.cpp` dispatch through
`curScene`. Still two scenes; `Title` does not exist yet, so the no-ship boot
stays as-is. Files: `scene.h/.cpp` (new), `main.cpp`, `events.cpp`, `vab.cpp`,
`game.h`, `Makefile`.

**Stage 3 — window sets.**
`WinDef` tables per scene; `drawUi` draws exactly its set; the Windows panel and
`toggle_windows()` iterate `curScene(g).wins`; delete the 19 `o_*` fields,
`UiWin`, `ui_windows` and `setup_ui_windows()`. Optional: bring the VAB and
Palette panels into the wrapper. Files: `scene.h`, `game.h`, `game.cpp`,
`gameui.cpp`, `ui.h` (only if a `forEachName` accessor is wanted — with
draw-time enforcement it should not be).

**Stage 4 — the Title scene (the payoff).**
Add `SceneId::Title`, its `SceneDef`, and the split of `drawMainMenu` into
`drawTitleMenu` (root UI, not closable) and `drawPauseMenu` (Flight window,
Esc-toggled). Shipless boot → `[Title]`; `--vab` with no fleet → `[Title, Vab]`;
shipless `load_game` → `[Title]`. Add the `enterFlight` invariant check and
delete the 7 guards. Wire **New Game** to `startGame`. Files: `scene.h/.cpp`,
`main.cpp`, `gameui.cpp`, `gameui.h`, `save.cpp`, `events.cpp`, `keys.cpp` (the
`Menu` slot's label/behaviour).

**Stage 5 — later.**
Quit-to-title (needs `unloadGame()`: `clearFleet` + a physics-world clear + job
cancellation + `part_sels.clear()` + a camera reset); Space Center / Tracking
Station as further `SceneId`s — the stack is what makes them cheap.

---

## 6. e2e impact

No existing case pins the affected lines. `01-smoke.txt` boots with no ship and
asserts only `Main loop starting` plus the `FORBID` set, so it keeps passing when
that boot becomes Title. The `[boot] no ship: orbit view of %s` line
(`main.cpp:550`) is not asserted anywhere; it becomes `[scene] -> title`.

Behaviour changes worth a case each:

- `52/56/57/58/59` (VAB): boot as `[Title, Vab]` instead of "orbit view +
  editor". Their `[vab] entered the editor (sim paused)` and `[vab] launched ...`
  anchors still fire; `--vab-launch` now collapses the stack via `enterFlight`.
- New: `60-title-boot` — bare `./osp`, expect `[scene] -> title`, forbid the
  flight windows' anchors.
- New: `61-title-newgame` — Title → New Game → Flight. Needs a headless hook:
  `--title-action newgame|load|vab|quit --at MS`, mirroring the existing
  `--vab-launch MS` pattern (`cli.cpp`). Clicking a button via `--sim-mouse`
  coordinates would be brittle.
- New: `62-vab-back-to-title` — `--vab-empty`, pop, expect `[scene] vab -> title`.
- New: `63-load-shipless` — a shipless fixture save loads into Title rather than
  an empty flight scene.
- `tests/test_save.cpp:203,214` must drop the `scene` round-trip assertion.

---

## 7. Risks and gotchas

- **Stack emptiness** is the one invariant that can crash: every `popScene` and
  `curScene` call site must tolerate depth 1. Rule 2 in §3.2, plus an assert.
- **`drawPartWindows` is not registered** — part windows use raw `ImGui::Begin`
  (`gameui.cpp:1535`) and are driven by `g.part_sels`, so window sets do not
  govern them. They must stay inside `flightDrawUi`, and `enterFlight` should
  clear `part_sels` (a New Game with stale entries from a previous fleet would
  dangle the moment the vehicles are freed).
- **`focusTargets[].name` is a `const char*`** into `b->name.c_str()` (or a
  literal for the "ship" entry). Anything that rebuilds the system (stage 5's
  `unloadGame`) invalidates them. Not a problem for stages 1–4; a landmine later.
- **The camera snapshot must be taken *before* the new scene aims the camera**,
  and restored only on pop (not on replace) — `enterFlight` deliberately drops
  the parked pose because `select_ship`/`syncShipFocus` re-aim anyway.
- **`ui::Manager` state outlives scenes** by design (§3.3). If a window is
  renamed, its old state entry lingers in the map forever. Harmless, but worth
  knowing; a `Manager::clear()` on `unloadGame` is the eventual answer.
- **Perf is a non-issue**: one indirect call per phase, static tables, no
  per-frame allocation. `sceneStack` should `reserve(4)`.
- **Save compatibility**: dropping `meta.scene` is a format change. Acceptable
  under the no-back-compat rule; the committed fixture `e2e/fixtures/save_racer`
  keeps working because the reader already treats the field as optional
  (`save.h:336`).

---

## 8. Other smells worth fixing while in there

Flagged per the standing instruction; none of these block the refactor, but
stages 1 and 3 touch all of them.

- **Function-local `static` UI state** in the draw pass: `nameBuf` / `selected`
  in `drawSaveLoad` (`gameui.cpp:2226-2227`), `savePath` / `ships` /
  `shipDirMtime` / `loadSel` in `drawVabUI` (`gameui.cpp:2332-2354`), plus
  `dpi_pending` and the two texture-dimension caches. The `saves[-1]`
  out-of-bounds read fixed in `dc8e020` came from exactly this pattern — state
  that no transition resets and no owner validates. These belong on `Game`
  (`UiState` / `VabState`) where a scene transition can reset them.
- **`savePath` is seeded once** from the build name and never re-seeded when a
  different ship is Loaded into the build, so Save can target a stale path.
- **The VAB input runs in the LOGIC phase** off `SDL_GetMouseState` plus *last
  frame's* `ImGui::GetIO().WantCaptureMouse`; the comment at `main.cpp:872-876`
  acknowledges the one-frame staleness. Moving it into `SceneDef::update` is
  where that should be resolved properly (poll the mouse in the event phase and
  store it on `Game`).
- **Two UI systems in parallel**: `ui::Window` vs raw `ImGui::Begin` for the VAB
  and Palette panels (§1.3).
- **`kRailsWarp`, `kToastLife`, `kPickClickPx`, the docking constants** are
  `static const` in `game.h` — internal linkage, one copy per translation unit.
  `constexpr` with a single home is free.
- **`Game` borrows 8 subsystems and owns ~200 fields.** The scene work does not
  fix that, but `VabState` / `UiState` / `CameraSnapshot` are the first
  meaningful chunking, and they are the natural seams if `Game` ever splits into
  a session (system + fleet + clock) and a view (camera + UI).

---

## 9. Open decisions

1. **Quit-to-title** is deferred to stage 5 because it needs fleet teardown.
   Until then the Flight pause menu carries a greyed entry (or none).
2. **Esc in the VAB popping the stack** as its third step (§3.4) changes the
   meaning of an existing binding — today the third Esc does nothing.
3. **Title's Save/Load** hides the Save half vs disabling it with a hint.
4. **Whether Title simulates.** Recommended yes (living backdrop, O(bodies) with
   no ships), but it does mean the clock runs on the title screen, and a New Game
   then starts at whatever `t` the menu sat at rather than 0. If that matters,
   either freeze Title or reset the clock in `startGame`.
5. **VAB / Palette into the `ui::` wrapper** (stage 3, optional) — better
   consistency and DPI behaviour, at the cost of re-tuning their layout.
