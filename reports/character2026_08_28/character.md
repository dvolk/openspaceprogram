# Walkable character: scoping (P0 = walking cylinder)

**Question (2026-08-28):** Scope adding a character that can walk around on the
ground. **P0:** a cylinder that moves along the ground with WASD + mouselook
camera (same control feel as the ship, like KSP's Kerbals). **Ultimate goal:**
KSP feature parity — animations, climbing, jetpack, EVA while flying,
parachutes, IVA (needs cockpits) — maybe beyond KSP.

**Short answer:** P0 fits the existing architecture with almost no new systems:
a character **is** a one-part `Vehicle` (the ship system already does
physics, rendering, fleet selection, camera-follow, HUD and warp-parking for
any part tree), plus a **walk controller** that replaces the
attitude/thrust command path with "horizontal velocity + stay on the ground".
The two things that make it cheap here are (1) the **analytic terrain height
function** (`GetTerrainHeight`, main.cpp:2305) — ground contact is a closed-form
lookup, no raycasts needed — and (2) the **parts catalog's behavior-by-field
presence** model (shipdef.h), which means the jetpack, parachute and legs of
the later phases are mostly *content + a few new behavior fields*, not new
engines. The genuinely new work is the EVA-side state machine
(attach/climb/enter-ship) and the character-vs-ship surface following.

**Effort shape:** P0 is a small change (a few hundred lines, one content mesh,
one e2e case). The parity roadmap is medium-to-large, and its phases are
ordered so each one reuses the last.

---

## 1. What the codebase gives us (ground truth)

| Fact | Where |
|---|---|
| A ship = `Vehicle`: a tree of welded `Body` parts (convex-hull collision), a `controller` part, thrusters + reaction wheels derived from part fields. One part is a valid ship (no attach loop runs) | `class Vehicle` main.cpp:935; `build_ship` main.cpp:1957; hull in `RegisterObject` physics.cpp:238-295 |
| Behavior is **data-driven by field presence**: `torque` → wheel, `fuel_rate`+`exhaust_velocity` → thruster, `capacity` → tank. "New part kind = edit parts.json, no source changes" is an explicit design goal | shipdef.h (header comment); `PartDef` shipdef.h:116 |
| Control path is single-gated: input → `ship->Command(ShipCmd, simActive)` → armed state → forces/torques re-applied **every substep** before `physics_tick` | main.cpp:1463 (`Command`), 1285-1330 (apply*), loop 4589-4610 |
| Per-tick input: WASDQE → Pitch/Yaw/Roll sticks, I → thrust, X → kill-rot, B/N → prograde/retrograde slew, R/F → throttle — all in one `isDown()` block that is trivially branchable | main.cpp:4510-4541 |
| Gravity is per-ship, re-applied per substep (central 2-body + Coriolis/centrifugal in rotating frames); a grounded ship in the surface frame already "works" | `processGravity` main.cpp:1276 |
| **Terrain height is analytic** everywhere: `radius + scaled noise`, sampled by direction vector — used for altitude, shadows, pad placement. Not just where collision meshes exist | `GetTerrainHeight` main.cpp:2305; `ComputeTerrainShadow` main.cpp:2324 |
| Terrain *collision* is per-LOD-patch `btBvhTriangleMeshShape` (margin 0.5 m), patches subdivide/cull **around the camera** — so collision exists wherever the camera (i.e. the active ship) is looking | `AddTerrainCollision` physics.cpp:203; `GeoPatch::Update` main.cpp:856 |
| Ship placement on the pad: lowest point at pad top, lifted by terrain 0.5 + hull 0.1 = 0.6 m so inflated shapes just touch — the exact ground-rest offset a walker needs | `build_ship` main.cpp:1993-1999 |
| Space port is a static collision body on the pad (the walker will hit it for free) | `StaticBuilding` main.cpp:2260; placement 3611-3619 |
| Camera: `OrbitCamera` (focus + RMB-drag look at 200 px/rad + wheel zoom) follows the **active ship's COM**; `FreeCamera` (6DOF) exists and C toggles. Render origin = active ship COM | camera.h:36-95; look handling main.cpp:4427-4446; renderOrigin main.cpp:4708 |
| Fleet = N `Vehicle`s; F6/SHIPS-window cycles the active one; idle ships park on rails, and a **grounded** ship *freezes* (railFrozen) — warp already tolerates ships sitting on the pad | `select_ship` main.cpp:3998; `goOnRails` main.cpp:1795 (grounded = periapsis ≤ radius+3000, :1823) |
| `ResourceType` already has **Oxygen, Water, Food, EC** alongside H2/LOX — the life-support/crew hook exists in the data model but nothing consumes it yet | shipdef.h:89 |
| E2E harness runs `./osp` under Xvfb with `--sim-press` / `--sim-mouse`, asserts on stdout (`[dbg]` already logs t/pos/alt/vel per ship) | e2e/run.py; `[dbg]` log main.cpp:~4640 |
| Part meshes are generated Python (trimesh) — a cylinder `.obj` + catalog entry is a 10-line script addition | utils/gen_parts.py, utils/gen_nose_cap.py |

Consequence: **everything a walker needs except the walk control model already
exists and is exercised daily.** The ship is the hard case (multi-part,
attitude control, staging, warp); a one-part upright body on the ground is a
strict subset of that — except the control law.

---

## 2. P0 design: the walking cylinder

### 2.1 Representation — a one-part `Vehicle`, flagged walkable

**Recommendation: the character is a `Vehicle` with one part and a
`walkable` flag.** Not a new `Character` class.

- The `ships` vector, selection (F6), camera follow, render origin, HUD,
  shadow computation, rail parking, and the e2e fleet machinery are all typed
  `Vehicle*` and all keyed on "active ship". A parallel `Character` class
  would need a base-class refactor or a second fleet — both worse than a flag.
- `build_ship` with n=1 parts is already a valid path (the attach loop is a
  no-op; placement puts the lowest point at the pad top + 0.6 m, main.cpp:1993).
- The flag gates exactly one region: the per-tick input block (main.cpp:4510-4541)
  and the HUD. Everything else (physics registration, drawing, `processGravity`,
  rail eligibility) is identical.
- Later phases fall out of the same representation: a **jetpack is a part**
  (thruster + tank fields), so an EVA kerbal is literally a two-part ship
  (kerbal + jetpack) using the *existing flight controls*. A **parachute is a
  part** with a new `drag` behavior field. KSP parity then reads: "character
  = ship whose crew model is different", which is exactly how KSP does it
  (Kerbals are vessels).

Data:
- `res/kerbal.obj` — cylinder, r = 0.35 m, h = 1.8 m (Kerbal proportions),
  authored origin-centered with +Z up like every part; generated via trimesh
  next to `utils/gen_parts.py`.
- `res/parts.json` — entry `"kerbal"`, type `kerbal`, mass ~60 kg, radius/height
  as above, **no** behavior fields for P0.
- `res/ships/kerbal.json` — `{ "name": "Kerbal", "parts": [ { "part": "kerbal" } ], "walkable": true }`.
- `shipdef.h/.cpp` — parse `walkable` (ship-level bool, like `controller`);
  `Vehicle::walkable` set in `build_ship`/`init`.

Spawn: a fleet entry (`--fleet` or `res/fleet.json`) with scenario `pad`
already lands it on the pad 20 m laterally from slot 0 (main.cpp:3635).
No new CLI needed; the existing "ships sharing a body+scenario get a slot"
mechanism does the work. Validation: reject a walkable ship in an orbit
scenario (nonsense, and `spawn_vehicle` would misplace it).

### 2.2 Physics — semi-kinematic walker (the one real design decision)

A free cylinder in Bullet **rolls** the moment you push it (it's round) and
slides/tumbles on slopes, so pure dynamics is the wrong model. KSP's Kerbals
don't roll — the legs act as a pinned pivot. We replicate that without legs:

**Per substep, before `physics_tick` (mirroring the existing
`processGravity` / `applyThrustForce` / `applyRotationForce` trio):**

1. `processGravity()` — **reuse unchanged** (central gravity + fictitious
   terms in the rotating surface frame; the ship already integrates a grounded
   body in exactly this frame, so the math is proven).
2. **Attitude is code-driven, never solved:** zero the body's angular velocity
   each substep (kills roll/tumble) and drive yaw toward the target with a
   torque (reuse the `ApplyTorqueRelZ` machinery — for a standing part local
   +Z is up, so yaw = Z-axis rotation; the ship's "yaw" is Y-axis, note the
   difference). Pitch/roll stay zero: the character is always upright.
   This is the same pattern as the rails park (pose imposed, solver only does
   translation + contacts), so it is consistent with existing code.
3. **Ground constraint (the KSP "legs stick" equivalent):** sample
   `GetTerrainHeight(normalize(pos))`; if the body's lowest point is within a
   small snap distance of the analytic surface **and** the radial velocity is
   small (grounded), set the position to
   `dir * (terrain + h/2 + 0.6)` — the *same* rest offset the pad placement
   uses (terrain 0.5 + hull 0.1, main.cpp:1993) — and zero the radial velocity.
   This gives crisp slope-following (the character climbs as the ground
   rises), no sinking, no hover. The 0.6 m offset matters: the terrain
   *collision* surface is inflated 0.5 m (physics.cpp:217), so snapping to the
   analytic surface alone would leave the hull 0.5 m above the ground.
4. **Horizontal motion by force, not velocity-set:** `ApplyCentralForce`
   along `input_dir * walk_speed` (normalized, ~2 m/s walk; tune by feel).
   Force + Bullet contact means the **space port and the ship block the
   walker for free** (both are collision bodies); direct `SetVelocity` would
   let the character drive through the pad.

Airborne case (thrown off a cliff, or later: jumping): when the grounded
test fails, steps 2-3 reduce to "upright + gravity" and Bullet integrates the
ballistic arc — the same code, no branch explosion. Landing = grounded test
passes again → snap.

Why not pure dynamics + friction: friction 4.0 (physics.cpp:285) would let a
round body roll and would make slope walking a tuning fight; the analytic
height function is strictly more deterministic and costs one `noise3d`
evaluation per substep (negligible).

**Character-vs-ship collisions:** both are dynamic convex bodies in the same
world — Bullet resolves it. A 60 kg walker vs a 30 t ship is a fine mass
ratio for the solver. Nothing to build.

### 2.3 Camera & input (P0)

- **Camera: zero new code.** The active-ship camera already follows the
  active `Vehicle`'s COM and supports RMB-drag look + wheel zoom
  (main.cpp:4427-4446). Selecting the kerbal (F6) orbits the kerbal. Set the
  default `OrbitCamera` distance small (~4 m) when the active ship is walkable
  so the cylinder reads as a character, not a rocket.
  - KSP reference: Kerbals get a **first-person** view by default with the
    body turning with the mouse. Our P0 uses the ship's third-person scheme
    per the request ("same as ship"); a first-person mode is a P1 toggle
    (`FreeCamera` at head height already exists and C already toggles to it).
- **Input (when the active ship is walkable), in the existing `isDown()` block:**
  - **W/S/A/D → move** in the camera's local frame, W projected onto the
    horizontal plane (exactly how `FreeCamera` WASD works today,
    main.cpp:4493-4505 — copy the frame math).
  - **Mouse: RMB-drag look** (existing), and the character's **yaw follows
    the camera's horizontal bearing** (KSP third-person: the body turns with
    the view). A/D could alternatively steer yaw while W moves forward
    (facing-relative, KSP-first-person style) — pick one; camera-relative
    movement is the less surprising P0.
  - **X → stop** (zero horizontal velocity) maps nicely onto KillRot's slot.
  - I/B/N/R/F are no-ops in walk mode (or reserved: I = jump, P1).
  - The `ship->onRails` wake-on-input logic (main.cpp:4513-4524) already covers
    warp: touching a control key re-enters physics and drops warp to 1.
- **HUD:** when walkable, swap the orbit-elements block (garbage on the
  surface: sma ≈ radius) for alt / ground-speed / facing. One `if (ship->walkable)`
  branch at the telemetry site (main.cpp:~4800+).

### 2.4 Warp & fleet interaction (should just work — verify, don't assume)

- Idle kerbal on the pad: `goOnRails` classifies by periapsis
  (≤ radius+3000 = grounded, main.cpp:1823) → **railFrozen park**, same as a
  pad ship. Warp with the kerbal on the ground is legal.
- Warp while walking: warp only engages via `.` when *all* ships are
  rail-eligible; a *moving* walker has periapsis ≈ radius + tiny → still
  grounded-classified → freezes mid-step, resumes on input. Acceptable (KSP
  equivalent: time warp with an EVA kerbal just freezes them). Verify the
  freeze/resume round-trip in e2e.
- Ship launches while kerbal stands on the pad: independent bodies, no
  coupling. Kerbal walks *under* a landing ship: Bullet contact decides.

### 2.5 P0 edge cases to pin down in code review

1. **Snap distance** must be < 5 m (pad height) or a walker standing *on the
   pad* snaps through it into the terrain. Use ~0.3 m, and gate the snap on
   "no ship-part collision body within snap range" — or simpler: only snap
   when the analytic-terrain point is also within the contact solver's reach
   (check the ship isn't between). In practice the pad is 5 m up, snap 0.3 m,
   so it cannot trigger; the ship-on-ground case is the real one and the
   "grounded" velocity gate handles it.
2. **Sea-level bodies** (`has_sea`): the analytic height clamps to the sea
   level, so the walker walks on flat "water". Fine for P0; mark no-swimming.
3. **Terrain LOD:** collision patches follow the *camera*; the camera follows
   the active ship = the walker, so the walker always has collision underfoot.
   An *idle* walker far from the camera has no collision — irrelevant (it is
   frozen on rails) but worth a comment.
4. **Rotating-frame drift:** the walker lives in `home->rot_frame` like a pad
   ship; `processGravity`'s fictitious terms are ~ω·v (mm/s² at walk speed) —
   ignore, but keep the frame so the ship/walker share one world.

### 2.6 Testing (e2e, headless)

New case `e2e/cases/walk.txt` (the harness already parses `[dbg]`):
- `--fleet` with a kerbal entry + a ship, `--sim-press` holding **W** for 2 s:
  CHECK `last(dbg).alt` within ±1 m of pad level (stayed on the ground),
  horizontal displacement > 1 m, `|vel|` bounded (no explosion).
- **Look test:** `--sim-mouse` RMB-drag over the view → CHECK the kerbal's
  yaw changed (add a `yaw=` field to the `[dbg]` line for walkable ships —
  one printf, testable).
- **Stability soak:** 20 s of random WASD + look, FORBID NaN/`error`,
  alt stays in [pad−1, pad+50] (didn't fall through, didn't fly off).
- **Warp round-trip:** `.` up to 1000, EXPECT "frozen on rails" for the kerbal,
  then `,` back and W-press again — moves again.
- Plus the standing `make clean && make test && make e2e` gate.

---

## 3. Roadmap to KSP parity (ordered by reuse, not by KSP's menu)

Each phase is sized by how much of *new state machine* vs *new physics* it is.
The physics side is the cheap part of this project: the parts catalog already
says "new behavior = new optional field".

### P1 — Feel (small)
- **Jump** (I or SPACE): radial impulse when grounded; the airborne path of
  §2.2 already exists. KSP parity: Kerbals jump.
- **Run** (Shift): 2-3× walk speed.
- **First-person camera** toggle: `FreeCamera` at head height, yaw = camera
  yaw, body hidden behind the near plane. `C` already toggles camera modes.
- **Better body:** KSP silhouette (capsule + head) — still one part (one
  welded mesh), zero physics change. The cylinder stays as the P0 stand-in.

### P2 — Animations (small-medium)
KSP: Kerbals have a walk/run leg animation. Options, cheapest first:
- **Procedural 2-part legs:** kerbal = 3 parts (torso, leg, leg) welded at
  hips/knees; drive the leg pose per tick with a sine wave at walk speed
  (the `attachPose`/weld machinery, shipdef.h, is exactly the "future VAB
  snap" the comments anticipate). Walking speed → stride frequency is a one-line map.
- **Mesh-swap gait** (2-4 pre-baked leg poses): zero joints, dumb but stable.
- Skeletal animation (assimp is vendored) is the escape hatch; not needed for KSP parity.
Legs also fix the *visual* foot-contact problem and make climbing (P4) natural.

### P3 — Jetpack + EVA-in-flight (mostly reuse — the big win)
KSP: a Kerbal with a jetpack is a free-flying 6DOF vessel using the *same*
WASDQE + I controls as the ship, with RCS. In our model that is:
- **Jetpack = a part**: `fuel_rate` + `exhaust_velocity` + `capacity` (small
  tank) + maybe a small `torque` for RCS-like authority. Catalog entry, done.
- **Kerbal + jetpack = a two-part ship** (`ships/kerbal_eva.json`, attach
  radial or down). The existing flight control path (WASDQE sticks, I thrust,
  X kill-rot, B/N slew) then *is* the EVA control scheme — zero new control
  code.
- **Mode logic:** grounded → walk controller; airborne & >X m/s or >Y m alt
  → flight controller; landing reverses it. That state switch is the only new
  logic, and it mirrors the existing grounded/free-fall rail split (main.cpp:1823).
- Jetpack fuel = the tank part; mass sheds as it burns (existing `consumeResourceMass`).
KSP gap we *do* fill beyond parity: jetpack works in vacuum (KSP's jetpack is
atmospheric RCS only, Kerbals coast in vacuum — arguably a KSP limitation we
can skip).

### P4 — Climbing / grabbing (medium — the main new system)
KSP: an EVA Kerbal can grab a vessel's surface and walk along it.
- **Attach state:** when the kerbal is within ~1 m of a ship part and the
  player confirms (or auto-grab on contact), park the kerbal's *relative pose*
  to that part — exactly the `rail_rel_pos/rail_rel_rot` pattern the rails
  already use (main.cpp:974-980), but relative to a *part*, not a frame.
- **Surface following:** parts are parametric (radius × height cylinders —
  `PartDef.radius/height`, shipdef.h), so "a point on the hull" =
  (azimuth, z) — moving along the surface is 2D math on a cylinder, no mesh
  walking needed. KSP's climbing is the same trick (vessel surfaces are
  parametric).
- **Release:** detach → ballistic (already have it).
- This is the foundation for **IVA** (P5) and for docking-adjacent features.

### P5 — IVA (medium-large, needs the "inside" concept)
KSP: enter the cockpit, become the pilot.
- **Cockpit already exists as data**: the `controller` part (shipdef.h:178) is
  the designated cockpit; nothing renders inside it yet (vab-drawing report
  covers the VAB side).
- **Enter/exit:** kerbal in P4-attach state on the cockpit part + confirm →
  `Vehicle::crew` state "inside ship S"; camera moves to a fixed offset in
  the controller part's local frame; control input re-maps from kerbal
  commands to `ship->Command(...)` (the same enum, different emitter).
- **Leave:** reverse; kerbal appears on the hull (P4 pose).
- New concepts: hatches (which parts are enterable), crew slots (one pilot per
  ship), and the "the ship's controls are the kerbal's controls while inside"
  mapping. The `Command()` gate (main.cpp:1463, "single path from control to
  physics") is deliberately built for exactly this — an autopilot or a crew
  member can drive through the same door.

### P6 — Parachute (small, once P3 exists)
- **Part behavior field `drag`** (m²): quadratic drag force opposing
  velocity, active while deployed. Deploy rule: key press, or auto when
  (alt < X and speed < Y). Descent = existing dynamics + drag.
- KSP: Kerbals deploy a chute to slow descent; same model. The field-presence
  pattern (like `torque`/`fuel_rate`) means a parachute is a catalog entry +
  ~30 lines of force code.

### P7 — Beyond KSP (open menu, all cheap given P1-P6)
- **Crew roster:** N named kerbals (the fleet mechanism is already
  multi-ship); each with its own jetpack/chute inventory.
- **Life support:** `ResourceType` *already contains* Oxygen/Water/Food/EC
  (shipdef.h:66-73) — nothing consumes them today. Crew consuming O2/food per
  day (the calendar exists, calendar.h), starvation/death, resupply from tanks.
  This is KSP's 1.6+ life-support mod territory — a genuine beyond-parity win
  and it reuses the existing resource plumbing (`consumeResourceMass`).
- **Health/G-damage:** hard landings (impact impulse is already measurable —
  the spin diagnostics read contact impulses, physics.h:11-17) injure the
  kerbal. Cheap, and it gives parachutes stakes.
- **Carrying:** a kerbal "holds" a part (P4 attach, single slot) — KSP can't
  do this; it's what makes EVA actually useful.
- **Character mass in Δv:** the kerbal's 60 kg already counts in ship mass
  when aboard (welded part) — KSP parity for free.

---

## 4. Risks & open questions

| # | Item | Status / mitigation |
|---|---|---|
| 1 | Cylinder rolls/tumbles in pure Bullet dynamics | Solved by design (§2.2): attitude code-driven, ω zeroed per substep. Verify with the soak e2e (no lateral drift, no spin) |
| 2 | Snap-through the pad or a landed ship | Snap gated on grounded test + 0.3 m distance; pad is 5 m up so unreachable; ship case handled by the velocity gate. Add the "walker on a landed ship" case to e2e |
| 3 | Jitter between Bullet contact and analytic snap | The snap *is* the rest condition (same 0.6 m offset the pad uses), so they agree at rest; if jitter shows, widen the grounded velocity gate before touching the offset |
| 4 | Walk control vs flight control mode switch (P3) could flicker near the boundary | Hysteresis on the grounded test (enter grounded at 0.3 m / 2 m/s, leave at 1.5 m / 5 m/s) — same trick as the rail grounded band (radius+3000) |
| 5 | HUD shows orbit elements for a surface walker | Branch on `walkable` (P0). Cheap; do it in P0 so the first human test isn't confusing |
| 6 | `walkable` in an orbit scenario | Reject at fleet parse (one validation line, next to the existing scenario checks) |
| 7 | KSP details I can't verify from the vendored refs (`references/` has Pioneer + Orbiter, not KSP source) | Where this report states KSP behavior (Kerbal mass ~60 kg, jetpack = RCS in atmosphere, chute deploy rules, climbing, IVA), treat as *design targets to confirm against a running KSP install* before the phase lands, not as facts |
| 8 | P4 surface-following on *non-cylindrical* parts (capsules, nose caps) | Parts stay parametric (radius/height); follow the **cylinder envelope** of the part, not the mesh. KSP does the same (vessel grab points are on the part's envelope). Accept the visual gap until P2 legs exist |

---

## 5. First steps (P0, in order)

1. `utils/gen_kerbal.py` (trimesh cylinder r0.35 h1.8) → `res/kerbal.obj` +
   catalog entry in `res/parts.json` + `res/ships/kerbal.json` with
   `"walkable": true`; parse the flag in `shipdef.cpp` (mirror `controller`).
2. `Vehicle`: `walkable` flag + `applyWalkControl()` (the §2.2 substep block:
   reuse `processGravity`, zero ω, yaw torque, ground snap, horizontal force)
   + `yaw` state for the HUD/log.
3. Main loop: branch the input block (main.cpp:4510-4541) on
   `ship->walkable` (WASD → move vector, X → stop); camera: small default
   orbit distance + yaw-follow; HUD branch; `[dbg]` gains `yaw=` when walkable.
4. Fleet entry + e2e cases from §2.6 (move / look / soak / warp round-trip /
   walker-on-landed-ship).
5. `make clean && make test && make e2e`, then a human feel-test of walk
   speed / look sensitivity / camera distance and tune the three constants.

Then P1 (jump/run/first-person/silhouette) is the natural follow-up commit;
P3 (jetpack EVA) is where the project stops looking like a toy, and by P5 the
"character" has cost roughly one state machine (P4) and one mapping (P5) —
the physics was free all along.
