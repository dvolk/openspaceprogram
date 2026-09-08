# Ship cockpits — crew, controls, MFDs, windows

Date: 2026-09-08
Status: proposal (not started)
Scope: a cockpit you can sit in — visible crew in their seats, a first-person
view, clickable flight controls, multi-function displays, and windows you can
see through. Written as an incremental plan: each phase is independently
shippable, e2e-tested, and leaves the tree green (`make test` + `make e2e`).

## TL;DR

"Ship cockpits" decomposes into five visible things: **crew visible** in their
seats, **clickable controls**, **MFDs** (nav/plan/sys screens), **windows**
(glass you see the outside through), and the **cockpit view** itself (a
first-person camera at the pilot seat). The last one is the prerequisite for
all of the first four — you cannot see the crew, the panels, the MFDs, or the
windows unless the camera can be *inside* the capsule. So the plan is ordered
around that: **get inside first** (cockpit camera + a hollow capsule + seat
positions), then add the things you can now see, in order of risk: crew in
seats, glass windows, clickable controls, MFDs, and finally the in-scene
interactive panels that make it feel like a real cockpit.

The key finding, in the spirit of the other reports: **most of the substrate
already exists.**

- **Crew already exist as first-class game objects.** A kerbal is a one-part
  `Kerbal : Vehicle` (`src/eva.h`); aboard crew live on `ship->crew`, are
  parked (body out of the physics world, mass folded into the capsule part),
  and are *deliberately not drawn* while aboard (`src/render.cpp`: "the aboard
  crew live on their ship, not here"). "Crew visible" is therefore mostly
  *un-hiding* them and giving them a seat position, not building a crew system.
- **The cockpit already has a named home: the `controller` part.** The
  `controller` (`src/vehicle.h`, `ShipDef::controllerIndex()` in
  `src/shipdef.h`) is already "the cockpit" — it defines the camera basis and
  the stick frame. It is, in every crewed ship, the capsule. A cockpit is a
  natural extension of `controller`, not a new concept.
- **Every MFD's data is already computed once per frame.** `ShipView`
  (`src/game.h`) holds orbit/surface/attitude/telemetry, the `TransferPlanner`
  holds plan/transfer, and the resources/EC state is on the parts. The HUD,
  ORBITAL, SURFACE, RESOURCES, TRANSFER, TELEMETRY and Autopilot windows
  (`src/gameui.cpp`) already render all of it. An MFD is a *different
  presentation* of data that is already live, not new simulation.
- **Clickable controls already exist in one form.** The Autopilot window's
  toggle buttons, the part window's EVA/Board/Target-docking buttons, and the
  SHIPS window's select/remove buttons are already clickable, already bound to
  the real `Vehicle::Command()` / `Game` transitions, and already e2e-testable
  via `--sim-mouse`. "Clickable controls" in the low-risk sense is mostly
  *re-presenting* existing actions in a cockpit layout.
- **Transparency is already in the pipeline.** The atmosphere rim
  (`src/terrain.h:251`) and the engine plume (`src/render.cpp:309`) both draw
  with `GL_BLEND` + `glDepthMask(false)` after the opaque pass. A glass window
  material is a new shader variant on a proven path, not new rendering
  architecture.

So the new work is: **a cockpit camera mode, seat positions, a hollow
capsule, a glass material, and cockpit-laid-out UI** — plus, later, in-scene
interactive panels. None of it touches the physics core, the frames, the fuel
or power systems, or the part/ship data model in any breaking way.

The plan is staged into seven phases, each shippable and tested. Phases 0–2
deliver a *visible* cockpit (sit inside, see the crew, see the outside through
glass). Phases 3–4 deliver *clickable controls* and *MFDs* in the reliable
imgui form. Phase 5 delivers the in-scene interactive end-state. Phase 6 is
polish (animation, seat assignment, sound, damage).

---

## What exists today (the building blocks we reuse)

Verified against the tree. This is the reason the new work is a presentation
layer, not a rewrite.

**Crew (`src/eva.h`, `src/game.cpp`, `src/ships.cpp`)**
- A kerbal is a `Kerbal : Vehicle` with one part (`kerbal.json`, a 1.7 m green
  "cucumber", `utils/gen_kerbal.py`). It rides the fleet, F6 cycling, rails,
  SOI and the HUD unchanged.
- Aboard state is `Kerbal::aboard` (the ship) + `Kerbal::aboardPart` (index
  into `aboard->parts`, the capsule). `Vehicle::crew` is the `std::vector`
  of aboard crew. Queries: `shipCrew` / `partCrew` / `freeKerbals`
  (`src/game.h`, defined in `src/game.cpp`).
- **Boarding/EVA transitions** (`Game::kerbalBoard` / `Game::kerbalEVA`,
  `src/game.cpp:505`/`:432`): move the kerbal's mass onto/off the capsule
  (`Vehicle::addPartMass`, `src/vehicle.h:1139`), park/unpark the body
  (`placeShipAtCom` + `RemoveBody`/`AddPhysicsBody`), and move it between the
  ship's `crew` list and the body's `ships` list.
- **Aboard kerbals are parked at the capsule's COM** (`kerbalBoard`:
  `k->placeShipAtCom(capPos, capRot)` with the capsule's world pose) and are
  **not drawn** — `src/render.cpp:133` skips them on purpose. There is **no
  seat position** anywhere; every aboard kerbal overlaps the capsule center.
- Startup crew: `Ships::spawn_crew` parks one kerbal in each capsule
  (`src/ships.cpp:202`).

**The cockpit concept (`src/vehicle.h`, `src/shipdef.h`)**
- `Vehicle::controller` is the "cockpit part" (`src/vehicle.h`: "the
  controller part (the cockpit, or the first reaction wheel by default)").
  `ShipDef::controllerIndex()` defaults it to the first part with reaction
  torque. Every crewed ship names its capsule as `controller`.
- The `controller` drives the **camera basis** (`src/render.cpp`:
  `camera->ref` is built from `ship->partAxis(ship->controller, …)`) and the
  **stick frame** (`applyRotationForce`). So the cockpit is already the
  anchor for "which way is up/nose from the seat."

**Camera (`src/camera.h`, `src/camera.cpp`)**
- One `Camera`, two modes: `CAM_ORBIT` (a turntable around a focus) and
  `CAM_FREE` (a 6DOF fly-by). `ComputeView()` emits the live mode's view;
  switching is a mode change, not a pose copy. Toggled by `C`.
- `renderOrigin` = the active ship's COM; the view is built in that frame so
  the float32 MVP cast quantizes ship-relative numbers (the floating-origin
  work, `reports/floating-origin2026_08_25`).
- `CAM_FREE` is *not* anchored to the ship — it is a free flier. **There is no
  camera mode that sits at a part's seat.** That is the core new camera work.

**Picking (`src/pick.h`, `src/pick.cpp`)**
- `pickRay` (pixel → ray in the render frame), `pickBody` (ray vs one rigid
  body's hull via Bullet's convex cast), `pickShipPart` (nearest part across
  every ship). Right-click a part → `pickAt` → `openPartWindow`
  (`src/game.cpp`) → a `PartSel` popup (`src/gameui.cpp:drawPartWindows`).
- The pick path already resolves a pixel to a *specific part*. Extending it to
  resolve a pixel to a *specific control element inside a part* is the
  mechanism Phase 5 needs.

**UI (`src/ui.h`, `src/gameui.cpp`)**
- `ui::Window(name, opts, body)` is the imgui wrapper (slots, layout,
  reset). Windows: HUD, Windows, Settings, TRANSFER, Game Debug Info, ORBITAL,
  TELEMETRY, SURFACE, SHIPS, VESSEL, Controls, Autopilot, RESOURCES, plus the
  part popups and the Orbital Map.
- **Clickable controls already exist**: the Autopilot window's mode toggles
  (bound to `ship->setSlewRequest`), the part window's EVA / Board /
  Target-docking buttons (bound to `g.kerbalEVA` / `g.kerbalBoard` /
  `dockTargetPort`), the SHIPS window's select/remove, and the Controls
  window's rebinding. These are the template for a "Cockpit" control surface.
- **MFD data sources**: `ShipView` (`src/game.h`) for NAV/SYS, the
  `TransferPlanner` for PLAN, the parts' resources + `powerTick` for SYS.

**Rendering (`src/body.h`, `src/render.cpp`, `res/partsShader.*`)**
- A part is one `Model` (mesh + shader + texture); `Body::DrawAt` draws it at
  a passed-in matrix (`src/body.h`). The parts shader is opaque: texture ×
  directional light, log-depth (`res/partsShader.vs/.fs`). No transparency,
  no per-part second mesh.
- **Transparency precedent**: the atmosphere rim enables `GL_BLEND`,
  `GL_SRC_ALPHA / ONE_MINUS_SRC_ALPHA`, `glDepthMask(false)` (`src/terrain.h:251`),
  and the plume uses additive blending (`src/render.cpp:309`). Both are drawn
  after the opaque pass. A glass material is this pattern with a window mesh.
- The maneuver indicators (prograde/retro/radial/normal/front) are
  `Billboard`s (`src/billboard.cpp`) — the visual elements a NAV MFD would
  fold into a 2-D navball.

**PostFX (`src/postfx.h`, `res/fx_*.fs`)**
- CRT, grain, gamma, color, sharpen, all toggleable at runtime. The CRT effect
  is a ready-made "MFD screen" aesthetic (scanlines) if we render an MFD to a
  texture and paste it on an in-scene panel (Phase 5).

**Input (`src/keys.h`, `src/events.cpp`, `src/siminput.h`)**
- `Slot` enum is the rebindable action space (PitchUp/Down, Yaw, Roll,
  Thrust, ThrottleUp/Down, KillRot, RCS directions, ToggleCamMode, …). A
  cockpit control button maps to the same `Slot`/`Command()` the keyboard
  does, so mouse-driven controls are *identical* to keyboard controls and
  testable with `--sim-mouse`.
- `--sim-press` / `--sim-mouse` / `--sim-mode` already drive the whole input
  path for e2e. A cockpit is e2e-testable with the existing harness.

**Data + mesh generation (`utils/gen_parts.py`, `utils/gen_kerbal.py`)**
- The catalog and meshes are generated by scripts (trimesh), so adding a
  hollow capsule interior, a window band, seat geometry, or a panel mesh is a
  generation change, not a hand-authored `.obj`. `capsule.obj` is a **solid
  capsule today — no interior, no windows**.

**Testing**
- GL-free unit tests (`tests/test_*.cpp`, `make test`); e2e battery
  (`e2e/run.py` + `e2e/cases/*.txt`, `make e2e`) that asserts on **stdout
  anchors** and drives input with `--sim-press`/`--sim-mouse`. 47 cases today.
- New cockpit behavior gets printed anchors (`[cockpit]`, `[mfd]`, `[crew]`
  seat lines) so the e2e cases can assert without reading pixels.

**What does not exist**
- No camera mode anchored to a part / seat. No seat positions. No hollow
  capsule or interior. No transparent window material. No "Cockpit" UI
  surface. No MFD. No in-scene interactive panels. No audio. No crew
  animation. (Crew *data* and the boarding/EVA transitions do exist.)

---

## The end state, and how it decomposes

Target: right-clicking or keying into the active ship's cockpit puts the
camera at the pilot seat; you see the crew member(s) in their seats, a panel
of clickable flight controls, one or more MFD screens (nav / plan / sys), and
the outside world through a window — and you can fly the ship by clicking.

The five named features and their prerequisites:

| feature | what it needs | prerequisite |
|---|---|---|
| **crew visible** | seat positions, draw aboard crew at their seats | cockpit view (to see them) |
| **windows** | a transparent window mesh + a hollow capsule + depth-sorted transparent draw | cockpit view (to look through) |
| **clickable controls** | a control surface bound to the existing `Command()`/`Slot` system | none (imgui form) / the cockpit view + in-scene panels (3-D form) |
| **MFDs** | a presentation of `ShipView`/planner/resource data | none (imgui form) / in-scene screens (3-D form) |
| **cockpit view** | a camera mode anchored to the `controller` part's pilot seat + a hollow capsule to look around in | — (it is the prerequisite) |

So the **cockpit view is the foundation**, and everything else is content
placed inside it. The plan builds the foundation first.

---

## Key design decisions

### D1 — The cockpit *is* the `controller` part, and it must be a capsule

The game already has a named "cockpit" (`Vehicle::controller`). We keep that
as the single source of truth for "where the seat is":

- A **cockpit** = the `controller` part *when it is also a capsule*
  (`Part::isCapsule()`, `src/part.h`). The pilot seat lives in the capsule;
  the other seats (crew_capacity − 1) are the passenger seats.
- Ships whose `controller` is *not* a capsule (an unmanned tanker whose
  controller defaults to an engine) have **no cockpit view** — the camera
  key is a no-op and the cockpit UI is hidden. This is correct: you can't sit
  in an engine. It also sidesteps the `racer`/`transporter` controller
  oddity noted in `reports/ship-crew2026_08_28` §6.1 — the cockpit feature
  simply does not apply to ships with no capsule controller.
- Consequence: no new "cockpit" data type. A cockpit is a *view of an existing
  capsule part*. Adding cockpit support to a new part kind is still
  "edit `parts.json` alone" (give it `crew_capacity` and name it
  `controller`).

### D2 — Seats are data on the capsule part; a kerbal holds a seat index

Today every aboard kerbal overlaps the capsule COM. We introduce a **seat
position** so crew can be seen in distinct seats and the cockpit camera can
sit at the pilot seat:

- `PartDef.seats`: a small list of `{ pos, rot }` in the part's local frame
  (origin-centered, +Z = nose, the existing part convention). The count equals
  `crew_capacity` (the catalog already sets both: `capsule` = 1 seat,
  `capsule_r1.5h3` = 3, `capsule_r2.25h4.5` = 6). Seat 0 is the **pilot seat**
  (the one the cockpit camera uses). The rest are laid out by the mesh
  generator (`utils/gen_parts.py`) around the capsule interior.
- `Kerbal.seatIndex`: which seat this kerbal occupies (replaces the implicit
  "everyone at COM"). Assigned at `spawn_crew`/`kerbalBoard` (first free seat)
  and read by the render pass to place the kerbal.
- The **parked pose becomes the seat pose**: `kerbalBoard` / `spawn_crew`
  park the kerbal at `partWorldPose(capsule) ∘ seats[seatIndex]` instead of the
  capsule COM. This is a *pose change only* — the mass transfer, the
  aboard/EVA state, the crew-list bookkeeping all stay exactly as they are.
  `Kerbal::crewRebase` (the merge/split index fix, `src/eva.h`) is untouched
  because it reindexes `aboardPart`, not seats.
- **GL-free and unit-testable**: seat resolution (which seat is free, seat pose
  = part pose ∘ seat local pose, pilot seat = seat 0) is pure math, testable in
  `tests/` the way `test_crew.cpp` already pins the capsule/seat-count rule.

### D3 — The cockpit camera is a third `CameraMode` anchored to the pilot seat

`src/camera.h` already holds two modes in one object and switches cleanly. We
add `CAM_COCKPIT`:

- Its pose is a **function of the active ship's cockpit part**, recomputed
  every frame from `ship->partWorldPose(controller)` and
  `seats[0]` (the pilot seat): position = seat position, orientation = seat
  orientation (nose out the front). It rides the ship rigidly — no turntable,
  no free drift — so the outside world and the interior are both visible and
  the view is stable under thrust/turn (the ship's attitude *is* the view).
- Look-around: a small mouse-driven yaw/pitch offset *about the seat*
  (reusing the `RotateY`/`Pitch` feel, bounded so you can look around the cabin
  but not leave it), so the pilot can glance at the MFDs and the crew without
  a camera teleport.
- Toggled by a new `Slot` (default a key, e.g. `F` or `V`-adjacent; rebindable
  like every other slot). Entering it requires a cockpit (D1); with no cockpit
  it is a no-op + a toast.
- **This is the load-bearing piece.** It is the first phase because crew,
  windows, and in-scene MFDs are all *seen from* the cockpit camera. It is
  also independently useful (a new camera mode) and low-risk: it reads existing
  pose accessors and adds one branch to `ComputeView()`.
- The `renderOrigin` stays the ship COM (unchanged), so the floating-origin
  precision work is untouched; the cockpit view is just a different camera
  basis in the same render frame.

### D4 — "Crew visible" = draw the aboard crew at their seat poses

The render pass (`src/render.cpp`) currently skips aboard crew. We add a draw
for them:

- For each ship, for each `Kerbal` in `ship->crew`, draw the kerbal's model at
  its **seat pose** (D2). The kerbal is a one-part `Vehicle`, so this is the
  same `partWorldPose`-style matrix build `Vehicle::Draw` already does — we
  just build the seat pose and draw the model there.
- The kerbal's body is out of the physics world while aboard, so there is no
  physics cost; this is pure draw. It is drawn with the opaque pass (the kerbal
  is opaque), so no transparency is needed for this phase.
- The kerbal is drawn *inside* the capsule, so it is only visible when the
  capsule has an interior to look into (D5) — which is why the cockpit camera
  + hollow capsule (Phase 0) come first.
- **Oddity to flag**: the kerbal mesh is 1.7 m tall (`utils/gen_kerbal.py`)
  but the base capsule interior is only 2 m tall and 2 m wide. A seated 1.7 m
  figure does not fit a 2 m capsule the way KSP's does. The seat poses +
  capsule interior (Phase 0/1) must either scale the kerbal into a seated pose
  or the capsule must be treated as "roomy enough for a seated kerbal." This is
  a content/scale decision, not an architecture one — flag it so it is decided
  deliberately, not discovered in review.

### D5 — A hollow capsule (interior) is content, generated by the existing script

The capsule is a solid shell today; you cannot see inside it. For the cockpit
view to show anything, the capsule needs an **interior**:

- Simplest first step: generate the capsule as a **hollow shell** (a slightly
  smaller inner surface, drawn double-sided, or an interior "cabin" mesh) so
  the cockpit camera sees a cabin, not the inside of a wall. This is a change
  to `utils/gen_parts.py` (trimesh), producing a new `capsule*.obj` (or a
  companion interior mesh). No engine change — a part is still one
  mesh+shader+texture (or, if we add an interior as a second mesh, a small
  `Part`/`Model` extension; see Phase 0 notes).
- The **window** (Phase 2) is a region of that shell marked transparent.
- Collision is unaffected: the hull is the convex hull of the part mesh
  (`BuildPartHull`, `src/body.h`), and a hollow shell has the same convex hull
  as the solid one. So the interior is pure visual.

### D6 — Windows are a transparent material on a window mesh, drawn after opaque

"Windows" = see the outside through the capsule wall:

- A **window mesh** (a band/visor on the capsule front, generated by
  `gen_parts.py`) + a **glass material**: a parts-shader variant that outputs
  alpha and is drawn with `GL_BLEND` + `glDepthMask(false)` *after* the opaque
  pass — the exact pattern the atmosphere rim and plume already use
  (`src/terrain.h:251`, `src/render.cpp:309`). The world behind is visible
  because the glass writes color but not depth.
- Render order: opaque parts → interior shell → glass → (UI). The glass is the
  last 3-D thing drawn, so it correctly overlays the outside world.
- **Depth-sorting note (flag):** transparent objects ideally sort back-to-front.
  With one convex glass band and an opaque cabin behind it, the simple
  "glass last" order is correct for the cockpit view. If we later add multiple
  windows on different ships close together, we may need a per-window sort —
  deferred, and only if it is visibly wrong.
- No new GL state machine: the existing `glEnable(GL_BLEND)` / `glDepthMask`
  toggles are reused.

### D7 — Clickable controls: imgui cockpit panel first, in-scene panels later

"Clickable controls" has two honest levels, and we ship the reliable one first:

- **Level 1 (Phase 3) — a "Cockpit" imgui panel.** A window laid out as a
  cockpit panel that presents the *existing* control actions as clickable
  widgets, bound to the same `Slot`/`Command()` the keyboard uses: a throttle
  slider + Thrust button, the RCS direction toggles, the Autopilot mode
  toggles (reusing the existing Autopilot-window buttons verbatim), Kill-rot,
  and stage. This reuses `ui::Window` and the exact button→action pattern
  already in `drawPartWindows`/the Autopilot window. It is **identical in
  effect** to the keyboard (same `Command()` path), so it is trivially
  e2e-testable with `--sim-mouse` clicks and cannot diverge from the
  keyboard-controlled ship.
- **Level 2 (Phase 5) — in-scene interactive panels.** Render the controls as
  meshes inside the cabin and make them clickable by extending the pick
  path (`src/pick.cpp`) from "which part" to "which control element." This is
  the ambitious "click the physical switch" end-state. It is deliberately last
  because it is the highest-risk piece (sub-part hit-testing, per-element
  draw, hover state) and everything it needs (the actions, the data, the
  camera) is already in place by then.
- The two levels share the **action layer**: a cockpit control is always a
  `Slot`/`Command()` (or an existing `Game` transition), whether it is
  triggered by a key, an imgui button, or a 3-D panel click. That is what keeps
  them consistent and testable.

### D8 — MFDs are presentations of already-computed state, in pages

An MFD is not new simulation; it is the existing `ShipView` + planner +
resource data laid out as a cockpit screen, in switchable **pages**:

- **NAV** — attitude indicator / navball (from `view.pitch/roll` + the
  prograde/retro/radial/normal directions already used by the maneuver
  billboards), airspeed/altitude, vertical speed, heading. All in `ShipView`.
- **PLAN** — the transfer/Δv/TOF readouts and the porkchop, from the
  `TransferPlanner` (already computed in the 3-D pass for the TRANSFER window).
- **SYS** — the resources (H2/LOX/mono/EC) + power balance, from the parts +
  `powerTick` (already shown in RESOURCES).
- **Phase 4 (imgui form):** three MFD windows (or one window with a page
  selector) rendering these, reusing the exact readout code from
  ORBITAL/SURFACE/RESOURCES/TRANSFER. A "page" is just which page's body is
  drawn. Testable: the values are already printed anchors.
- **Phase 5 (in-scene form):** render the active MFD page to a texture (FBO or
  a screen-space projection) and paste it on a panel mesh in the cabin, with
  the CRT PostFX for the screen aesthetic. Clicking the panel's page buttons
  switches the page (via the D7 Level-2 pick path).
- The MFD data is computed once per frame regardless of whether it is shown,
  so an MFD has **zero simulation cost** — it is a view.

### D9 — What a cockpit does *not* change

In the spirit of the docking report's "what does not need to change":

- **Physics** — the ship is still one rigid body; the cockpit is a view of a
  part. No new bodies, constraints, or forces.
- **Fuel / power / staging** — untouched. Crew mass still moves via
  `addPartMass`; life support still drains via `powerTick`.
- **The part/ship data model** — a cockpit adds `seats` (and a window/panel
  mesh) to a part; it does not change the part tree, the attach modes, or the
  behavior-derivation rule.
- **Fleet / SOI / rails / warp** — a cockpit is per-active-ship view state; it
  parks on rails and coasts exactly like today.
- **The HUD and the existing windows** — the cockpit *adds* surfaces; it does
  not replace the HUD/ORBITAL/etc. (Both can be open; the player chooses.)

---

## Phased plan

Order: **0 → 1 → 2 → 3 → 4 → (5 ∥ 6)**. Phases 0–2 make the cockpit
*visible*; 3–4 make it *usable*; 5 makes it *diegetic*; 6 polishes. Each phase
is independently shippable, leaves the tree green, and has an e2e case.

### Phase 0 — Get inside: cockpit camera + hollow capsule + seats

*The foundation everything else is seen through. No crew, no glass, no panels
yet — just "I can sit in the cockpit and look around the cabin."*

- **Data**: `PartDef.seats` (list of local `{pos, rot}`, count =
  `crew_capacity`, seat 0 = pilot) in `src/shipdef.h`; parse + validate in
  `src/shipdef.cpp`; a default seat layout (pilot at the front, others behind)
  so existing capsules work without mesh edits. `Kerbal.seatIndex` in
  `src/eva.h`.
- **Mesh**: `utils/gen_parts.py` emits a hollow capsule (interior shell) for
  the three capsule sizes; `parts.json` regenerated. (If the interior is a
  second mesh, the minimal `Part`/`Model` extension to draw two meshes per
  part lands here — keep it as small as possible; the alternative is a single
  mesh with the interior baked in.)
- **Camera**: `CAM_COCKPIT` mode in `src/camera.h/.cpp` (pose = cockpit part
  pose ∘ `seats[0]`, bounded mouse look-around); a new `Slot` (default key) +
  `events.cpp` toggle; the cockpit requires a cockpit part (D1) else no-op +
  toast.
- **Seating**: `spawn_crew` / `kerbalBoard` assign a seat index and park at
  the seat pose (D2) — the pose change only, all other boarding bookkeeping
  unchanged.
- **Stdout anchors**: `[cockpit] enter/exit` with the seat pose; `[crew]` seat
  assignment lines. Add `cockpit` to `e2e/run.py`'s CHECK namespace.
- **Unit tests** (GL-free): seat resolution (free-seat assignment, seat 0 =
  pilot, seat pose = part pose ∘ seat local), the cockpit-eligibility rule
  (capsule controller = has cockpit; engine controller = none).
- **E2E**: `50-cockpit-enter.txt` — a crewed ship on the pad; `--sim-press` the
  cockpit key; `EXPECT [cockpit] enter`, `EXPECT` the seat pose matches the
  capsule part pose; `FORBID GL_`, `FORBID error:`.
- **Exit:** pressing the cockpit key puts the camera at the pilot seat inside
  the capsule; the cabin interior is visible; the outside is visible (the
  capsule is no longer a solid wall to the inside). Verified by eye (per
  convention) + the pose anchor in e2e.

### Phase 1 — Crew visible in their seats

*The crew you can see sitting in the cockpit.*

- **Render**: `src/render.cpp` — for each ship, draw each aboard `Kerbal` at
  its seat pose (D4). Opaque pass; the existing skip becomes a draw.
- **Seat assignment**: the first N free seats fill in order (pilot seat = the
  `pilot`, if the roster marks one — see the open question on pilot identity);
  `partCrew`/`shipCrew` unchanged. A `[crew] seated` anchor per kerbal.
- **Kerbal scale/sit** (D4 flag): decide the seated-kerbal scale so a 1.7 m
  figure fits the cabin; a seated pose or a scale factor on the kerbal model
  while aboard.
- **Unit tests**: extend `tests/test_crew.cpp` — seat assignment for a
  1/3/6-seat capsule, pilot-seat priority, full-capsule refusal (already
  present) now also reports the seat state.
- **E2E**: `51-crew-visible.txt` — a 3-seat capsule with 3 crew, cockpit key
  pressed; `EXPECT` three `[crew] seated` lines with distinct seats; the
  render is by-eye. A case asserting the kerbal is *not* drawn when EVA'd out
  (it re-enters the body list and is drawn there instead).
- **Exit:** in the cockpit view you see the crew member(s) sitting in their
  seats; EVA one and they leave the cabin (the existing transition, now with a
  visible seat they vacate).

### Phase 2 — Windows / glass

*See the outside world through the capsule wall.*

- **Material**: a parts-shader variant with alpha output + the
  `GL_BLEND`/`glDepthMask(false)` draw (D6), drawn after the opaque pass
  (the atmosphere/plume pattern).
- **Mesh**: `utils/gen_parts.py` marks a window band on the capsule front (a
  UV region or a separate window mesh); `parts.json`/mesh regenerated.
- **Render order**: opaque parts → interior → glass. `src/render.cpp` gains the
  transparent-draw step (reusing the existing blend toggles).
- **Stdout anchor**: a `[cockpit] glass` line naming the window part (so e2e
  can assert the glass exists and is drawn). The *look* of the glass is
  by-eye (per convention — screenshots only on request).
- **E2E**: `52-windows.txt` — cockpit view on a body with a visible surface /
  another ship in view; `EXPECT [cockpit] glass`, `FORBID GL_`. (Asserting the
  outside world is *visible through* the glass is a pixel check, which the
  harness does not do — that one is by-eye.)
- **Exit:** from the cockpit, the outside world is visible through a
  transparent window; the cabin and crew are in front of it.

### Phase 3 — Clickable controls (imgui cockpit panel)

*Fly the ship by clicking, in the reliable imgui form.*

- **UI**: a "Cockpit" window (`src/gameui.cpp`, `ui::Window`) laid out as a
  panel: a throttle slider + **Thrust** button (bound to `ThrottleUp/Down` +
  `Thrust` slots), the **RCS** direction toggles, the **Autopilot** mode
  toggles (reusing the existing Autopilot buttons), **Kill-rot**, and **Stage**.
  Every widget calls the same `Slot`/`Command()`/`Game` transition the keyboard
  does (D7 Level 1), so a click is indistinguishable from a key.
- **Binding**: map each widget to its `Slot`; drive it through the existing
  `slotFired`/`slotHeld`-equivalent action path so the latch/edge semantics
  match the keyboard exactly.
- **Stdout anchors**: the existing `[dbg]`/`[attlog]` lines already reflect
  the result of a control (thrust → `|v|` changes; RCS → attitude changes), so
  a cockpit click is asserted through those — no new anchor strictly needed,
  but a `[cockpit] control <slot>` line helps the e2e read.
- **E2E**: `53-cockpit-controls.txt` — `--sim-mouse` click on the Thrust
  button (the button's screen rect is known from the layout), hold `I`;
  `EXPECT` the `[dbg]` `|v|` rises; a second case clicks Kill-rot and
  `EXPECT` `|w|` → ~0 (`[attlog]`). `FORBID GL_`.
- **Exit:** the ship can be flown (throttle, RCS, autopilot, kill-rot, stage)
  entirely by clicking the cockpit panel, with the same effect as the keys.

### Phase 4 — MFDs (imgui form)

*Nav / plan / sys screens, in pages.*

- **UI**: an MFD window (or one window + page selector) with three pages (D8):
  **NAV** (navball from `view` attitude + the prograde/retro/radial/normal
  directions; speed/alt/VS/HDG), **PLAN** (transfer Δv/TOF + porkchop, from
  the `TransferPlanner`), **SYS** (resources + power, from the parts +
  `powerTick`). Reuse the exact readout code from ORBITAL/SURFACE/RESOURCES/
  TRANSFER. A page button switches which page's body is drawn.
- **Data**: nothing new — `ShipView` is already computed per frame by
  `src/render.cpp`; the planner is already updated in the pass.
- **Stdout anchor**: an `[mfd] page=<nav|plan|sys>` line on page change (the
  values themselves are already printed by the existing windows' anchors, so
  the e2e can assert the MFD shows the *same* numbers as the ORBITAL/RESOURCES
  windows).
- **E2E**: `54-mfd.txt` — `--sim-mouse` click the PLAN page; `EXPECT [mfd]
  page=plan`; assert the Δv/TOF shown matches the TRANSFER window's values
  (both read the planner). `FORBID GL_`.
- **Exit:** the cockpit has working NAV/PLAN/SYS screens, switchable by
  clicking, showing the same live data the existing windows show.

### Phase 5 — In-scene interactive cockpit (the diegetic end-state)

*Controls and MFDs as clickable objects inside the cabin, not imgui windows.*

This is the highest-risk, highest-reward phase and is deliberately last. It
turns Phases 3–4 (which already have the actions and the data) into 3-D
objects.

- **Panels as meshes**: the control panel + MFD screen are meshes in the cabin
  (generated by `gen_parts.py`, or a dedicated `res/cockpit/` set). The MFD
  screen is a quad whose texture is the active page (D8 Level 2): rendered to
  a texture (FBO) or screen-projected, updated per frame, optionally run
  through the CRT PostFX for the screen look.
- **Sub-part picking**: extend `src/pick.cpp` from "which part" to "which
  control element" — a control/MFD button is a small pickable quad with an
  id; `pickShipPart` generalizes to `pickControl` (the same Bullet cast path,
  against the panel's child quads). A click fires the bound `Slot`/`Command()`
  (D7 Level 2) — the *same* action as the imgui button and the key.
- **Hover / active state**: the picked control highlights (a per-element
  uniform or a separate highlight draw), so the cursor feedback the user
  expects is there.
- **Stdout anchor**: a `[cockpit] click <element>` line (so e2e can assert a
  3-D click fired the right action through the existing `[dbg]`/`[attlog]`/
  `[mfd]` anchors).
- **E2E**: `55-cockpit-3d.txt` — `--sim-mouse` click on the on-screen position
  of a 3-D Thrust control (computed from the cockpit view + panel pose), hold
  `I`; `EXPECT` `[cockpit] click thrust` + the `[dbg]` `|v|` rises. A case
  clicks the MFD's PLAN button; `EXPECT [mfd] page=plan`. `FORBID GL_`.
- **Risks** (why it is last): sub-part hit-testing against moving panels,
  per-element draw + hover, the MFD-to-texture pass (an FBO or projection
  each frame), and transparent panel sorting. Each is bounded and testable in
  isolation; landing them after the actions/data/camera exist means this phase
  is presentation, not plumbing.
- **Exit:** in the cockpit view, the controls and MFDs are physical objects
  you can click, and clicking them flies the ship and switches screens — the
  full "clickable controls + MFDs" end-state, diegetically.

### Phase 6 — Polish (optional, independent)

- **Crew animation**: an idle / hands-on-stick pose for the seated kerbal
  (a second kerbal mesh or a pose), and a "pilot" kerbal whose hands are on
  the stick. Pure content.
- **Seat assignment UI**: in the part window, choose which kerbal takes which
  seat (currently first-free). Needs the roster/pilot identity (open question).
- **Sound**: the game has **no audio today**. Engine, RCS, and cockpit
  ambience would be a new subsystem (an SDL2 audio device + a small mixer),
  independent of everything above. Flagged as a large, separate effort.
- **Crew consequences**: G-loads, reentry heating, cabin pressure / hull
  breach (the "crew consequences" open question in
  `reports/ship-crew2026_08_28` §7). Needs the physics presence the cockpit
  gives it a home for, but is its own simulation.
- **Hatch / airlock**: a part the kerbal EVA's *through* (today `kerbalEVA`
  teleports the kerbal beside the capsule). A nicer transition, not required.

---

## The data model changes (summary)

Small and additive; the part/ship model's behavior-derivation rule is
untouched.

| change | where | note |
|---|---|---|
| `PartDef.seats` (list of local `{pos, rot}`) | `src/shipdef.h/.cpp` | count = `crew_capacity`; seat 0 = pilot; GL-free, unit-tested |
| `Kerbal.seatIndex` | `src/eva.h` | replaces the implicit "at COM" park; assigned at board/spawn |
| parked pose = seat pose | `src/game.cpp` (board/EVA), `src/ships.cpp` (spawn_crew) | pose change only; mass/aboard/crew-list bookkeeping unchanged |
| cockpit part = `controller` when it is a capsule | `src/vehicle.h` (a `hasCockpit()` helper) | D1; no new part type |
| `CAM_COCKPIT` + seat pose + look-around | `src/camera.h/.cpp` | D3; reuses the mode-switch design |
| glass material + window mesh | `res/partsShader.*` (variant), `utils/gen_parts.py` | D6; reuses the existing blend path |
| "Cockpit" + "MFD" UI | `src/gameui.cpp` | D7/D8; reuses `ui::Window` + existing readouts |
| `pickControl` (sub-part) | `src/pick.cpp/.h` | Phase 5; generalizes the existing part pick |
| panel/MFD meshes + MFD-to-texture | `utils/gen_parts.py` (or `res/cockpit/`), `src/render.cpp` | Phase 5 |

Nothing here changes the part tree, the attach modes, fuel groups, the EC
pool, staging, the fleet lists, or the compound-body invariant
(`Vehicle::checkCompoundInvariants` still pins the geometry).

---

## Rendering work, in order

1. **Phase 0** — a second (interior) mesh per capsule part, or a baked-in
   interior; draw it in the opaque pass. (Smallest possible `Part`/`Model`
   extension if it is a second mesh.)
2. **Phase 2** — a transparent glass draw after the opaque pass
   (`GL_BLEND`, `glDepthMask(false)`), the atmosphere/plume pattern.
3. **Phase 5** — panel + MFD-screen meshes; the MFD page rendered to a texture
   (FBO or screen projection) each frame; per-control hover highlight;
   transparent-panel draw order.

The float32/`renderOrigin` precision scheme, the log-depth, the terrain LOD,
and the compound-body rendering are all untouched.

---

## Testing strategy (per project convention)

Every rule prints a stdout anchor so e2e can assert without pixels; pure math
is unit-tested headless; the visual result is verified by eye (screenshots
only on request).

- **Unit (GL-free, `make test`)**: seat resolution + pilot-seat priority +
  full-capsule (extend `tests/test_crew.cpp`); the cockpit-eligibility rule
  (capsule vs engine controller); the cockpit camera seat-pose math (extend
  `tests/test_orbitcam.cpp`-style pure-math coverage); the MFD page→data
  mapping (which `ShipView`/planner fields back each page) as a small
  pure-math test.
- **E2E (`make e2e`)**: cases `50`–`55` above, each driving the cockpit with
  `--sim-press` (keys) and `--sim-mouse` (clicks) and asserting on
  `[cockpit]` / `[crew]` / `[mfd]` / the existing `[dbg]` / `[attlog]`
  anchors. `FORBID GL_` and `FORBID error:` on every case.
- **By-eye**: the cockpit view, the visible crew, the glass, and the in-scene
  panels are visual — per convention, ask the user to check (or iterate with
  screenshots only if requested). The F12 screenshot path
  (`src/main.cpp`, `./tmp/osp_*.png`) already exists for this.
- **Selftest (optional)**: a `--selftest-cockpit` flag (the
  `--selftest-spawn` pattern) that enters the cockpit, checks the seat pose
  against the capsule part pose, clicks a control, and checks the resulting
  `[dbg]` change — a single deterministic in-process pass.

---

## Risks / open questions

- **Kerbal vs cabin scale (D4).** A 1.7 m kerbal in a 2 m capsule is tight.
  Decide deliberately: seat the kerbal (a seated pose / scale) or make the
  cabin roomier. A content decision, but it should be made in Phase 0/1, not
  discovered in review.
- **Second mesh per part.** A hollow capsule (and later the panels) may be a
  second mesh on a part. The `Part`/`Model` path is one-mesh today. Keep the
  extension minimal (a list of drawables, or a baked-in interior) and pin it
  with a test; do not let it grow into a general multi-mesh part system before
  we need one.
- **Transparent draw order (D6).** "Glass last" is correct for one convex band
  behind an opaque cabin. Multiple windows on close ships, or a window in
  front of another ship's hull, may expose a sorting artifact. Defer until it
  is visibly wrong; the fix is a per-window depth sort, bounded.
- **MFD-to-texture (Phase 5).** Rendering an MFD page to a texture each frame
  (FBO or projection) is new GL work and the most likely source of a
  GL-state bug in the plan. Isolate it behind one function, test it headless
  where possible, and gate it behind the Phase 5 e2e.
- **Pilot identity.** `reports/ship-crew2026_08_28` notes the roster/pilot
  identity is an open question (a spawned copy *duplicates* crew data). Seat
  assignment "pilot takes seat 0" assumes a pilot is marked. Until the roster
  lands, seat 0 is just "the first seat" and the cockpit camera uses it
  regardless of who sits there. The cockpit works either way; the *pilot*
  semantics wait on the roster.
- **Unmanned ships.** Ships with no capsule controller (a tanker) have no
  cockpit (D1) — the cockpit key is a no-op + a toast. That is correct, but
  make the no-op loud enough (a toast) that a player is not confused.
- **Controller default oddity.** `reports/ship-crew2026_08_28` §6.1 flagged
  that `racer`/`transporter` could resolve their `controller` to an engine.
  The cockpit feature makes the controller's identity more visible (it is now
  "the seat"). Ensure every ship meant to have a cockpit names its capsule as
  `controller` explicitly (data-only).
- **No audio.** "Cockpit" often implies sound. There is no audio subsystem
  today; it is a large, separate effort (Phase 6, optional) and should not be
  assumed in "done."

---

## What does not need to change

- **Physics core / compound body** — the cockpit is a view of a part; no new
  bodies, constraints, or forces. `checkCompoundInvariants` keeps pinning the
  geometry.
- **Fuel / power / staging** — untouched; crew mass and life support keep their
  existing paths.
- **Frames / SOI / rails / warp** — a cockpit is per-active-ship view state;
  it parks and coasts exactly like today.
- **The HUD and existing windows** — the cockpit adds surfaces; it does not
  replace the HUD / ORBITAL / SURFACE / RESOURCES / TRANSFER / TELEMETRY /
  Autopilot windows.
- **The input model** — a cockpit control is a `Slot`/`Command()`; the
  rebindable key map, `--sim-press`, and the `Command()` path are unchanged.

---

## Key references

- `src/vehicle.h` — `controller`, `crew`, `partWorldPose`, `addPartMass`,
  `Command()`, `checkCompoundInvariants`.
- `src/eva.h` / `src/game.cpp` — `Kerbal`, `aboard`/`aboardPart`,
  `kerbalBoard`/`kerbalEVA`, `partCrew`/`shipCrew`/`freeKerbals`.
- `src/shipdef.h` / `src/shipdef.cpp` — `PartDef`, `crew_capacity`,
  `controllerIndex()` (where `seats` and `hasCockpit()` land).
- `src/part.h` — `isCapsule()`, behavior derivation.
- `src/camera.h` / `src/camera.cpp` — `CameraMode`, `ComputeView()`,
  `renderOrigin` (where `CAM_COCKPIT` lands).
- `src/render.cpp` — the 3-D pass, the aboard-crew skip (`:133`), the
  transparent-draw precedent (`:309`), `Vehicle::Draw`'s pose build.
- `src/pick.h` / `src/pick.cpp` — `pickRay`/`pickBody`/`pickShipPart` (where
  `pickControl` lands).
- `src/gameui.cpp` — the Autopilot / part-window / SHIPS clickable patterns,
  the ORBITAL/SURFACE/RESOURCES/TRANSFER readouts an MFD reuses.
- `src/ui.h` — `ui::Window` (the cockpit/MFD window wrapper).
- `src/keys.h` / `src/events.cpp` / `src/siminput.h` — the `Slot` action
  space, the key dispatch, `--sim-press`/`--sim-mouse`.
- `src/body.h`, `res/partsShader.vs/.fs` — the part draw path + shader (the
  glass variant).
- `src/terrain.h` (`:251`), `src/billboard.cpp` — the transparency + billboard
  precedents.
- `src/postfx.h`, `res/fx_crt.fs` — the MFD-screen aesthetic.
- `utils/gen_parts.py`, `utils/gen_kerbal.py` — the capsule + kerbal mesh
  generation (where the interior, window, seats, and panels are authored).
- `e2e/run.py` + `e2e/cases/` — the harness, the CHECK namespace, the
  `--sim-*` input (where cases `50`–`55` land).
- `reports/ship-crew2026_08_28/ship-crew.md` — the crew model this builds on
  (and the pilot-identity / crew-consequences open questions).
- `reports/eva2026_09_02/eva.md` — the kerbal-as-Vehicle design + the
  "animations / crew" future hooks.
- `reports/docking2026_09_07/docking.md`,
  `reports/autopilot2026_09_04/autopilot.md` — the house style this report
  follows (phased, shippable, e2e-tested, "what does not need to change").

---

## Concrete edit list (per phase)

| phase | file | change |
|---|---|---|
| 0 | `src/shipdef.h/.cpp` | `PartDef.seats`; parse/validate; default layout |
| 0 | `src/eva.h` | `Kerbal.seatIndex` |
| 0 | `utils/gen_parts.py` | hollow-capsule interior mesh; regenerate `capsule*.obj` + `parts.json` |
| 0 | `src/vehicle.h` | `hasCockpit()` (controller is a capsule) |
| 0 | `src/camera.h/.cpp` | `CAM_COCKPIT` + seat pose + bounded look-around |
| 0 | `src/keys.h/.cpp`, `src/events.cpp` | the cockpit `Slot` + toggle |
| 0 | `src/game.cpp`, `src/ships.cpp` | seat assignment + park-at-seat (pose change only) |
| 0 | `tests/test_crew.cpp`, `e2e/cases/50-cockpit-enter.txt`, `e2e/run.py` | seat/eligibility unit tests; enter e2e; `cockpit` CHECK ns |
| 1 | `src/render.cpp` | draw aboard crew at seat poses (the skip becomes a draw) |
| 1 | `src/gameui.cpp` (optional) | seat readout in the part window |
| 1 | `tests/test_crew.cpp`, `e2e/cases/51-crew-visible.txt` | seat-assignment tests; crew-visible e2e |
| 2 | `res/partsShader.*` (variant), `src/render.cpp` | glass material + transparent draw after opaque |
| 2 | `utils/gen_parts.py` | window band; regenerate |
| 2 | `e2e/cases/52-windows.txt` | glass-exists e2e (look is by-eye) |
| 3 | `src/gameui.cpp` | the "Cockpit" control panel (throttle/RCS/autopilot/kill-rot/stage) bound to `Slot`/`Command()` |
| 3 | `e2e/cases/53-cockpit-controls.txt` | click-to-fly e2e (assert via `[dbg]`/`[attlog]`) |
| 4 | `src/gameui.cpp` | the MFD window (NAV/PLAN/SYS pages) reusing the existing readouts |
| 4 | `e2e/cases/54-mfd.txt` | MFD page + value e2e (assert against the planner/existing anchors) |
| 5 | `src/pick.cpp/.h` | `pickControl` (sub-part hit-test) |
| 5 | `src/render.cpp`, `res/cockpit/` (new), `utils/gen_parts.py` | panel + MFD-screen meshes; MFD-to-texture; hover highlight |
| 5 | `e2e/cases/55-cockpit-3d.txt` | 3-D click-to-control + MFD e2e |
| 6 | (various) | crew animation, seat-assignment UI, audio, crew consequences, hatch (all optional, independent) |

Verification for every phase, per project convention: `make clean && make test
&& make e2e` green before committing.
