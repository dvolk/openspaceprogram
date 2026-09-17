# Verification pass — corrections and design amendments to `ui-scenes.md`

Quality pass over `ui-scenes.md` (same directory), run 2026-09-17 against HEAD
(`7ed16c0`). The original is an immutable snapshot and is **not** edited; read
this alongside it. Where the two disagree, this file wins.

Three parts: **A** — amendments that change the proposed design (the original is
wrong or incomplete in ways that would bite during implementation); **B** —
factual errata; **C** — live bugs found in passing, independent of the refactor.

Most specific claims in the original checked out: every scene-switch site in the
§1.1 table, the 7 `"No active ship."` guard locations, the 19 `o_*` fields,
`UiWin::opts` being read only at `game.cpp:165`, raw `ImGui::Begin` at
`gameui.cpp:1535/2425/2557` vs `ui::Window` at `2363`, `axis_from` tolerating an
absent sibling anchor (`ui.h:214-216`), `save.h:336` already treating `scene` as
optional on read (so dropping it does not break the committed
`e2e/fixtures/save_racer`), no e2e pin on `[boot] no ship:`, the 9 `vab_cam*`
field names, the §8 statics, and `--vab` with no `--ship/--fleet/--body`
producing no active ship.

---

## A. Design amendments

### A1. `closable = false` is not enough — windows need four roles

The original's §3.4 makes TitleMenu unclosable via `ui::Options::closable =
false`. That only hides the X button: `ui.h:337` passes `p_open = nullptr`, but
`ui::SetOpen(name, false)` still closes it and `Window()` then early-returns
(`ui.h:269-272`). Combined with §3.3 (TAB and the Windows panel iterate the
current scene's set) and §3.4 (TitleMenu is in Title's set), **pressing TAB on
the title screen would close the only UI standing** — reproducing the exact
symptom the refactor exists to fix. `ui::ResetGui()` (`gameui.cpp:2182`,
`events.cpp:311`) has the same reach.

Amendment — `WinDef` carries a role, and the roles drive every bulk operation:

```cpp
enum class WinRole : unsigned char {
    Root,        // the scene's identity (TitleMenu). Forced open every frame by
                 // the scene's drawUi; excluded from TAB and the Windows panel.
    Chrome,      // scene furniture (the Windows panel, VAB TopBar). Always drawn
                 // with the scene; not TAB-toggleable, not in the panel.
    Transient,   // modal-ish (PauseMenu, Save/Load). Closed by ANY transition.
    Persistent,  // everything else. TAB + panel toggle it; state survives
                 // transitions untouched.
};
```

- **Root** is enforced by the scene's `drawUi` calling `ui::SetOpen(name, true)`
  immediately before drawing it — one hash lookup per frame, and then no
  combination of TAB, `ResetGui()` or a stray `SetOpen(false)` can blank the
  title screen.
- **Transient** is the single, well-defined exception to "transitions do not
  touch window state": `pushScene` / `popScene` / `replaceScene` close the
  outgoing top's transient windows. Without it, pushing Vab from Flight leaves
  PauseMenu `open == true` in the Manager, and popping back ("Back to game",
  `gameui.cpp:2365`) lands you on a **running sim with the pause menu up**.
  Today `gameui.cpp:2172` avoids this by closing the menu before `vabOpen`; a
  role-based rule does not rely on every button remembering to.
- **Persistent** keeps the round-trip behaviour the original wanted.
- `ui_visible == false` (TAB) skips Persistent + Transient + Chrome, and **not**
  Root. Consequence: the title screen cannot be screenshot-cleaned with TAB.
  Accepted for now; a dedicated clean-screenshot mode is the answer if it
  matters (§D).

### A2. One `WinDef` per window game-wide, referenced by index

The original's §3.3 says a shared window "appears in more than one table". That
half-applies: `Manager::state()` (`ui.h:104-116`) rewrites `default_open` and
`fixed` from the caller's `Options` every frame, but slot / offset / anchors /
`initial_size` latch only at relayout (`ui.h:285-318`). Two tables carrying
differing `Options` for the same window would flip-flop its `default_open` per
scene while its position stayed wherever the first relayout put it.

Amendment:

```cpp
enum Win : int { W_Hud, W_Windows, W_Orbital, /* ... */ W_Count };
extern const WinDef kWins[W_Count];        // ONE entry per window, game-wide
struct SceneDef { /* ... */ const Win *wins; size_t nWins; };   // indices
```

Each window's `ui::Options` then has exactly one home, which is also what finally
kills the `o_*` / `UiWin::opts` duplication. The HUD's special case in
`toggle_windows()` (`game.cpp:167`) folds into the table as an ordinary
Persistent entry.

### A3. `load_game` must never touch the scene stack — and it has three arms, not two

The original's §3.6 says loading "lands on `[Flight]`, or `[Title]` when the save
has no active ship". Two problems:

- **It cannot.** At boot `load_game` runs at `main.cpp:351`, before the camera
  exists (`main.cpp:479-481`) and before `focusTargets` is built (`527-540`) —
  which is why `syncShipFocus` already guards `camera != nullptr`
  (`game.cpp:405-407`). A scene push needs both.
- **It is not exception-safe.** `clearFleet(g)` (`save.cpp:363`) deletes the
  fleet and nulls `ship` *before* the per-ship reads at `371-377`, which can
  throw; `drawSaveLoad` catches and only toasts (`gameui.cpp:2280-2282`). A
  corrupt save therefore leaves a live Flight scene with no fleet and no ship —
  a state the new invariant declares unreachable.

Amendment: **scene decisions live only in `startGame()` and the menu buttons;
never in the persistence layer.** Delete `save.cpp:361` rather than moving it,
and give the caller three arms:

| outcome | scene |
|---|---|
| load succeeded, active ship | `enterFlight(g)` |
| load succeeded, no active ship | `replaceScene(g, Title)` |
| load **failed** | stay put (runtime) / fall back to Title (boot) |

Making `load_game` transactional (parse everything, then clear) is a separate fix
— see C4.

### A4. `startGame()`'s real extent, and the boot ordering it forces

The original's §3.7 gives `main.cpp:315-460`. Wrong range: `412-460` is
mesh / texture / billboard / shader setup that has nothing to do with starting a
game, and the block it *does* need is omitted. `startGame()` is:

- the fleet assembly — `main.cpp:318-410` (fleet entries, the `--ship` /
  `--fleet` / `--radial-test` / `--dock-test` / `--load` branches,
  `apply_scenarios`, the active-ship pick, the park-idle-ships-on-rails loop);
- the camera construction + initial aim — `main.cpp:461-484`;
- the focus-target build + initial focus pick — `main.cpp:527-551`.

The ordering matters: **the initial scene push must come after `startGame`**, so
a `CameraSnapshot` is never taken against an empty `focusTargets`. That is what
makes a `[Title, Vab]` boot work, and it lets the snapshot drop
`vab.cpp:562-570`'s bounds-check-plus-`ship ? nullptr : home` fallback entirely
(see A3's boot-order note — that fallback, not the `vabClose` one, is the live
boot-order artefact).

### A5. The redraw bit needs stating explicitly

`main.cpp:912` (`game.redraw = true`) is the only reason the VAB renders at all —
with `tick` skipped, nothing else marks the frame. `SceneDef` does not need a new
field for it; the existing rule generalises exactly:

```cpp
sc.update(game);
if (!sc.sim) { game.redraw = true; }   // a frozen scene has no tick to mark it
```

Worth writing down, because if `vabUpdate` is expected to set it and does not,
the editor stops drawing — and only the `--vab-launch` e2e cases would notice.

### A6. Two staging corrections

**(i) The `vabOpen` → `vabAimCamera` split belongs in stage 1, not stage 2.**
`vabLoad` calls `vabOpen` purely to re-aim at the replaced tree
(`vab.cpp:494-501`). Stage 2's rule 3 (refuse to push a scene already on the
stack) would silently turn that into a no-op, and **e2e case 59 would stay
green** — its anchors are the `[vab] loaded` / `[vab] launched` prints — while the
camera stopped following a Load. Split it in stage 1 and add a `[vab] re-aim`
print as an anchor.

**(ii) Stage 1 must carry the headless VAB hooks with the block it moves.**
`--vab-place` lives *inside* `main.cpp:888-899` and needs `overUI` plus the
main-local `vab_place_fired` (`main.cpp:555`); `--vab-load` / `--vab-launch`
(`853-867`) sit outside the block but are scene-gated and use `main.cpp:556-557`.
Move the three `*_fired` bools onto `Game` (or a small `TestHooks` struct on it)
and fire them from `vabUpdate` at the same points — `--vab-place` in particular
must still fire *after* `vabUpdateHover` (the reason is in the comment at
`main.cpp:884-887`). Dropping it breaks case 56 with no e2e-visible error beyond
a missing `[vab] place hook:` line.

### A7. Stages 1–3 are not all invisible — stage 3 changes TAB

`toggle_windows()` (`game.cpp:164-166`) currently touches only the 15 registered
windows plus the HUD. `Windows`, `Main Menu` and `VAB TopBar` are **not** in
`ui_windows`, and `ui_visible` gates only `drawSaveLoad` (`2220`) and
`drawVabUI` (`2304`) — not `drawUIReadouts`, `drawUIMap`, `drawPartWindows` or
`drawMainMenu`. So today TAB does not hide the Windows panel or the main menu.
Putting `Windows` in Flight's table (original §3.4) would make TAB close the
panel that lists the toggles. A1's `Chrome` role is what prevents that; with it,
stage 3 is behaviour-preserving except that TAB additionally hides the part
windows and the main menu — which should be called out as an intended change, not
discovered later.

### A8. The VAB Esc chain must become conditional

Original §3.4 proposes "Esc: link mode → disarm → pop". The current code
(`events.cpp:351-360`) disarms **unconditionally**, even when nothing is armed,
so as written the pop would be a *fourth* press and feel broken:

```cpp
if (g.vabui.linkMode)  { cancel link mode; }
else if (armed)        { disarm; }
else                   { popScene(g); }
```

### A9. New e2e hook

`62-vab-back-to-title` (original §6) has no headless pop hook, and
`--sim-press MS,0,ESC` three times is brittle against A8. Add `--vab-close MS`,
mirroring the existing `--vab-load-at` / `--vab-launch` / `--vab-place` pattern in
`cli.cpp`.

---

## B. Errata

| Original claim | Correct |
|---|---|
| "34 `vab_*` fields (`game.h:176-240`)" | **39** (40 counting `BuildShip vab`), `game.h:179-246` |
| "the whole ghost-preview state (10 fields)" | **13** (`ghostValid, ghostSurface, ghostPos, ghostRot, ghostPoint, ghostNormal, ghostParentNode, ghostChildNode, ghostRoll, ghostRollUsed, ghostClones, ghostAssembly, ghostRoot`) |
| "`g.scene` … 27 sites across four files" | **15** code sites (main 7, events 2, save 2, vab 4 at `534/587/588/603`). 27 counted comment lines. §1.1 also omits `vab.cpp:587` |
| §2(a) "five switch sites" | seven in `main.cpp`, fourteen overall — pick one and use it |
| "`numFocusTargets` … + 4 sites" | **5**: `game.cpp:403`, `events.cpp:222`, `main.cpp:540`, `main.cpp:543`, `vab.cpp:613`. Note `events.cpp:222` is `% g.numFocusTargets` — a div-by-zero if the list is ever empty, and `focusTargets.size()` carries the same hazard, so replacing the field must not drop the non-empty assumption |
| "the Surface Map branches on `ship` at three more sites" | **5**: `gameui.cpp:797-798, 814, 936, 1041, 1075` |
| "`main.cpp:868-905` is the VAB input block" | **868-912** (`911` `vab_lmb_prev`, `912` `redraw = true`). 905 truncates before the LMB edge-detect the move must carry |
| `vabClose`'s `else if(g.ship != nullptr)` fallback "exists purely because a `--vab` boot has no parked pose" | It is **unreachable dead code** today: `main.cpp:481` sets `game.camera` before `vabOpen` (`495/501`), and `vabOpen` parks unconditionally when `camera != nullptr` (`vab.cpp:553-554`), so `vab_camSaved` is always true entering `vabClose`. Deleting `vab.cpp:628-634` is still right — just not for that reason. The live boot-order fallback is the `focusBody` one at `vab.cpp:562-570` (A4) |
| "Seven windows open with `if(ship == nullptr)`" | 7 are *guarded*, but only **5 are open** at the no-ship boot: Orbital (`1194`), Surface (`1254`), Vessel Info (`1329`), Resources (`1452`), Orbital Map (`1702`) — `default_open` is true (`ui.h:78`). Game Debug Info (`1087`) and Autopilot (`1419`) are `default_open = false`. The rest of the visible chrome is the `Windows` panel (`240`) and the HUD (`216`, inline guard at `217`) |
| "`setup_ui_windows()` (`game.cpp:38-166`, ~128 lines)" | **38-160, 123 lines**; 166 is inside `toggle_windows` |
| §3.7's `main.cpp:315-460` | see A4 |
| "the other ~95% of the copied struct is dead weight" | `ui::Options` has ~11 fields, one of which is read: ~91% |
| "Stages 1–3 are invisible to the player" | see A7 |

---

## C. Live bugs found in passing

All four predate the refactor and are independent of it. C1 and C2 are the
serious ones.

### C1. `Game::remove_ship` dereferences the ship it just deleted

`game.cpp:1043-1058`. `all` is snapshotted before `delete v` (`1044`), then the
handoff loop runs:

```cpp
Vehicle *x = all[i];
if(x->isCrewAboard()) { continue; }   // x may BE v -- deleted one statement ago
if(x == v) { seen = true; continue; }
```

`isCrewAboard()` is **virtual** (`vehicle.h:462`), so this is a virtual dispatch
through a freed object's vptr — on *every* removal of the active ship, not just a
corner case. The identity check is one statement too late. The reverse loop
(`1061-1066`) and the N/M count (`1073-1078`) both test `== v` first and are
safe. Fix: hoist the `x == v` test above the dereference.

`--selftest-spawn` step 3 exercises exactly this path (`main.cpp:680-690`). A
reproduction under `osp_asan` was started and is logging to
`tmp/asan_selftest_spawn_2026_09_17.log`; note that binary is from 2026-09-13 and
predates HEAD, so a clean run would not disprove the finding — `remove_ship` has
not been touched since.

### C2. `remove_ship` can leave `ship` dangling

If every remaining vehicle has crew aboard, both handoff loops leave
`next == nullptr`, the `if(next != nullptr)` block (`1068`) is skipped, and
`ship` still points at the deleted vehicle. The only guard is `all.size() <= 1`
(`1012`). Reachable from the Ship List (`gameui.cpp:1310`) with a two-ship fleet
whose non-active ship is crewed.

This matters for the refactor: it means the stage-4 invariant check must be
"ship is non-null **and live**", not merely non-null, and that the correct
fallback when no controllable ship remains is `replaceScene(Title)` — which is
what makes C2 stage-4 work rather than a separate patch.

### C3. `vabLaunch` leaks the camera park

`vab.cpp:507-543` sets `g.scene = Scene::Flight` directly at `534` and never
clears `vab_camSaved`, so the pre-launch camera pose survives the launch.
Re-entering the VAB then skips the park (`vab.cpp:553` guards on
`!g.vab_camSaved`), and "Back to game" restores a pose from *before* the launch.
Stack rule 4 (`enterFlight` collapses the stack and drops the frame) fixes this
structurally — the strongest single argument for that rule, and worth citing when
it is implemented.

### C4. `load_game` clears the fleet before it can fail

`save.cpp:363` (`clearFleet`) runs before the per-ship reads at `371-377` that
can throw. See A3. Making it parse-everything-then-clear turns a corrupt save
from "silent total loss of the running game" into "load refused, game intact".

---

## D. Amended open decisions

The original's §9 list stands, with these changes:

1. *(unchanged)* Quit-to-title deferred to stage 5 (needs `unloadGame()`).
2. Esc-in-VAB popping the stack — now with A8's conditional chain. Still a
   binding-meaning change.
3. Title's Save/Load: hide vs disable the Save half.
4. Whether Title simulates (recommended yes). If yes, a New Game starts at
   whatever `t` the menu sat at, not 0 — decide whether `startGame` resets the
   clock.
5. VAB / Palette panels into the `ui::` wrapper (stage 3, optional).
6. **New:** clean screenshots on the title screen. A1 exempts Root windows from
   TAB, so the title menu cannot be hidden for a screenshot. Options: leave it,
   or add an explicit clean-screenshot mode that suppresses Root for one frame.
