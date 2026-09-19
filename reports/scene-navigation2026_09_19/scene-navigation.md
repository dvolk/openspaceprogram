# Scene navigation & the "views, not pause-states" rework

Date: 2026-09-19
Status: **PLAN — design agreed, not yet implemented.** This record captures the
decision and the phase breakdown as settled on this date; it is a snapshot of the
plan, not a log of the work (the per-phase commits and any `verification.md`
written during implementation are the record of what actually happened).

---

## 1. What is wrong today

The scene stack (introduced in `reports/ui-scenes2026_09_17`) works, but the
*use* of it is awkward. Three things compound:

1. **Five duplicated menus.** Every scene draws the same shell footer
   (`drawMenuWindow`, `gameui.cpp:2185`: Save/Load, Settings, Controls, Quit
   game) plus its own nav block. The nav blocks are thin — mostly "Back" (which
   is Esc) and "Quit to title." So the player sees the same four buttons in five
   places, and has to learn which nav rows live in which menu.

2. **"Pause" is a scene property.** `SceneDef::sim` does double duty: it means
   both "the clock advances" and "the ship is live." That is why the hub and the
   Tracking Station are *frozen* (`sim=false`, `update=stillUpdate`), and why
   Esc-into-them reads as a pause menu. A frozen backdrop is the least
   interesting way to look at your own solar system.

3. **The one destructive action is everywhere.** "Quit to title"
   (`quitToTitle` → `unloadGame`, which *deletes the fleet*) sits in 4 of the 5
   menus with no confirmation, one Esc away from a live flight.

## 2. The target model

**Scenes are views, not pause-states.** The world always simulates; time is a
global knob; piloting belongs to one scene; Esc walks up a fixed tree.

- The world always simulates — the planet turns and ships coast in **every**
  scene (the Title scene already does this; extend it to the hub + tracking).
- **Time is a global knob** — `,` / `.` (warp down/up) work from anywhere.
  "Pause" is `time_accel == 0`, a clock state, not a scene state.
- **Piloting is Flight-only** — WASD/T/RCS steer the ship only in the Flight
  view. Everywhere else the ship is being watched, not driven.
- **Esc always goes up the tree** — one rule, no per-scene menu-toggle
  semantics.
- **The Space Center is the only in-game menu** (the hub). The Title screen is
  the only pre-game menu.

```
        Title                         (New Game / Load / Settings / Quit)
          │
          └── Space Center            (THE hub menu: VAB, Tracking, Resume,
              │                        Save/Load, Settings, Return to title, Quit)
              ├── VAB                 (build + LAUNCH; Esc = back to hub)
              └── Tracking Station    (live orbits; select a ship, Fly; Esc = back)

        Flight                        (piloting; Esc = up to Space Center)
```

| | Today | Target |
|---|---|---|
| Hub / Tracking sim | frozen | **live** (planet turns, orbits move) |
| `,` `.` warp | flight / title only | **every scene** |
| WASD/T in hub | (n/a — frozen) | inert (ship coasts) |
| "Pause" | a scene property | a clock state, reachable from anywhere |
| Menus | 5 | **2** (Title + hub) |
| Esc | toggle menu / pop / cancel, per scene | **always up the tree** |

This is closer to KSP (whose map shows live orbits) and removes the per-scene
menu duplication entirely.

## 3. Current-state facts that drive the plan

Verified against the code on this date:

- `SceneDef::sim` gates whether `tick` runs at all (the scene's `update` is
  `tick` for live scenes, `stillUpdate` for the frozen hub/tracking —
  `scene.cpp`). `stillUpdate` is shared by both frozen scenes.
- The ship-control block in `tick` reads the held keys **scene-independently**
  (`tick.cpp:176+`, the `if(g.ship){ ...Command(...) }` region). Nothing today
  stops WASD from steering the background ship if the hub ever ran the sim.
- The time-warp one-shots (`WarpUp`/`WarpDown`) live in `flightKeyActions`
  (`events.cpp`), so they only fire in flight/title. The scene-neutral slots
  (Screenshot / Wireframe / ToggleWindows) are already dispatched in
  `poll_events` before the scene's key map — the natural home for the warp keys.
- The Tracking Station's map reads `g.view` (the active-ship snapshot), which is
  computed **as a side effect of the 3D pass** (`draw3d`, `render.cpp:118`;
  declared `render.h:19`). The tracking scene skips that pass on purpose
  (`trackingDraw3d` is a no-op, `scene.cpp:96`) and only *worked* because the sim
  was frozen. Make the sim live and the map goes stale until this is split out.
- Five menu draw sites: `drawTitleMenu` / `drawPauseMenu` /
  `drawSpaceCenterMenu` / `drawVabMenu` / `drawTrackingMenu` (`gameui.cpp`),
  each a `drawMenuWindow` with a nav block; window membership in the `WinSet`
  table (`uiwins.h`) and the `WinRole` Root/Transient bookkeeping.
- Headless e2e hooks that touch this surface: `--new-game`, `--space-center`,
  `--tracking`, `--tracking-close`, `--quit-title` (`game.h`, `main.cpp`), and
  the scene e2e cases under `e2e/cases/`.

## 4. Phased plan

Each phase is independently shippable and testable (`make test` + the relevant
e2e cases before committing). Order matters only in that **Phase 1 is a
prerequisite** for the live-sim phases; 2 and 3 may land in either order; 4 is
the user-facing change; 5 is the sweep.

### Phase 1 — Foundation: piloting gate + global time-warp
*Size: S. Risk: low. Prerequisite for 2–3.*

Goal: make the machinery correct so the sim can run in a non-flight scene
**without** the ship being steered, and so the warp keys work anywhere. No scene
semantics change yet (hub/tracking still frozen), so this phase is safe on its
own.

- Add a `pilot` flag to `SceneDef` (`scene.h`), mirroring `sim`. Flight =
  `true`; Title/hub/tracking/VAB = `false`.
- Gate the ship-control block in `tick` on `curScene(g).pilot` (in addition to
  `g.ship`). The free-camera WASD and the world-advance stay ungated.
- Move `WarpUp`/`WarpDown` out of `flightKeyActions` into the scene-neutral
  slot block in `poll_events` (`events.cpp`, next to Screenshot/Wireframe/
  ToggleWindows).

Verify: `make test`; flight piloting + warp unchanged; hub/tracking (still
frozen) show no regression. Rollback: revert the gate + the key move.

### Phase 2 — Live Tracking Station
*Size: M (the one fiddly piece). Risk: medium.*

Goal: the map shows **live** orbits — ships moving along their conics — with the
sim running.

- Split the `g.view` computation out of `draw3d` into a standalone
  `updateShipView(Game&)` (`render.cpp` / `render.h`): the orbit snapshot the
  map + readouts need (pos/vel, `o`, `mu`, `orbit_pos/vel`, the series).
  `draw3d` calls it (behavior unchanged); it is now also callable on its own.
  Care: it is currently interleaved with the camera/render-frame setup — separate
  the pure "compute the ship's state" from the "aim the camera / build the render
  frame" work.
- Tracking scene: `sim=true`, `update` advances the world (calls `tick`) **and**
  refreshes `g.view` (`updateShipView`). `draw3d` stays a no-op (no world draw —
  the full-screen map covers it).

Verify: orbits move while the sim runs; select-ship still works; `,`/`.` pause
and resume the map live; the `--tracking` e2e case passes. Rollback: revert
tracking to `stillUpdate` + `sim=false`.

### Phase 3 — Live hub + VAB
*Size: S–M. Risk: low–medium.*

Goal: uniform "the world always runs" — the planet turns behind the hub, and the
warp keys are meaningful in the VAB (so "pause it manually" works).

- Space Center: `sim=true`, `update=tick`. Planet rotates; warp works; Esc still
  pops (Phase 4 re-points it).
- VAB: make it live — its `update` runs the editor step **and** the world tick
  (call `tick`), so time passes behind the studio backdrop and the warp keys have
  an effect. Confirm the main loop calls each scene's `update` exactly once per
  frame so `tick` is not double-stepped.
- `stillUpdate` becomes unused after this phase (removed in Phase 5).

Verify: planet turns in the hub; VAB editor (place/rotate/detach) unaffected;
warp in hub + VAB. Rollback: revert to `stillUpdate` + `sim=false`.

### Phase 4 — Navigation & menus (the UX change)
*Size: M–L. Risk: medium (touches the window table + e2e).*

Goal: the target model's navigation — Esc-up, two menus, Fly-from-tracking, a
safe "return to title."

- **Esc-up wiring** (per-scene `keys`): Flight → `pushScene(SpaceCenter)`;
  Space Center → `popScene` (or Title when it is the floor); VAB → `popScene`;
  Tracking → `popScene`; Title → top (no-op). Replaces the menu-toggle Esc in
  flight/tracking and the cancel→disarm→menu Esc in the VAB (keep the
  cancel-link / disarm steps first, then pop).
- **Collapse to two menus.** Remove `W_PauseMenu`, `W_VabMenu`,
  `W_TrackingMenu` and their nav blocks; keep `W_TitleMenu` +
  `W_SpaceCenterMenu`. Update the `WinSet` scene sets, the `WinRole`
  Root/Transient bookkeeping (`uiwins.h`/`.cpp`), and the `drawUi` call order in
  `scene.cpp`.
- **Hub menu:** add **Return to title** with a confirmation step (second click /
  "discard game?") since it is now one Esc from a live flight; keep **Quit**
  (exit app) distinct.
- **Tracking:** add a **Fly** button (select the ship + `enterFlight`) beside
  the active ship; keep "click name = select."
- **VAB:** a Back button in the top bar (Esc already pops); keep LAUNCH.
- **e2e:** re-point the `--space-center` / `--tracking` / `--tracking-close` /
  `--quit-title` hooks at the new paths and update the scene cases so the suite
  still exercises every transition.

Verify: full scene e2e suite; manual Esc-up walk from every scene; Fly-from-
tracking lands in a live flight. Rollback: last phase — revert as a unit.

### Phase 5 — Quality pass + cleanup
*Size: S.*

- Spawn a subagent for a full quality pass over the change (bugs, QoL).
- Delete now-dead code: `stillUpdate`, the removed menu draw helpers, any
  orphaned `Win` ids / `WinSet` entries.
- Final `make test` + full e2e; confirm the version footer / menus render in both
  remaining menus.

## 5. Deliberately not doing (for now)

- **Global number/function-key scene teleports.** Deferred. Esc-up + the hub
  already reach any scene in one key + one click, and a "jump to X" is a third
  transition kind (it must discard parked camera poses and possibly a half-finished
  VAB build) plus a second navigation system to learn. If wanted later, implement
  as a guarded collapse-to reusing the `enter*` family, rebindable, with toasts on
  refusal.
- **Direct Flight → VAB** (today's "Go to VAB"). Dropped in favor of hub → VAB,
  since the VAB builds *new* ships rather than editing the flying one; the hub is
  the natural home. Revisit only if "build another ship mid-flight" proves painful.

## 6. Open decision (recommendation stated)

- **VAB live vs paused.** Recommendation: **live** (Phase 3), for uniformity and
  so the warp keys are meaningful there. Cost: the active ship's physics steps
  invisibly while editing — negligible (one ship) and pause-able with `,`.
