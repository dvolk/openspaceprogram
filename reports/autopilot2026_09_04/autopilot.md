# Ascent & Landing Autopilot — Scope and Staged Work Plan

Date: 2026-09-04
Status: proposal (not started)

## TL;DR

Add two mission autopilots on top of the existing ship-control primitives:

1. **Ascent** — take off from a pad and insert into a **circular orbit** the user
   selects by **altitude** (inclination is a later option, see below).
2. **Landing / suicide burn** — from **circular orbit** or **suborbital** flight,
   come down and touch down gently.

Both **auto-stage** when the live stage runs dry, and both are written to be
**robust to the atmospheric effects** some bodies will have once drag/lift are
modelled (today `surface.atmosphere` is a visual Fresnel rim only).

The key finding: **most of the machinery already exists.** There is a proven,
authority-bounded attitude autopilot (`slew`), a real throttle + fuel + staging
model, exact two-body orbit math (`orbit.h`), a Lambert/Hohmann transfer solver
(`transfer.h`), and a clean idiom for lifting a ship's state into its SOI
inertial frame (`shipInertial()`). The new work is a **guidance state machine**
that reuses those primitives — not a new physics core.

The plan is staged into five phases, each independently shippable and e2e-tested.
Phase 1 (ascent to a circular orbit) and Phase 2 (landing) are the headline
features; inclination, atmosphere, and polish follow.

---

## What exists today (the building blocks we reuse)

Everything the autopilot needs is already in the codebase. This is the reason the
new work is a guidance layer, not a physics rewrite.

**Ship control (`src/vehicle.h`)**
- **Attitude autopilot** — `slew` / `slewRequest` (a `SlewMode`), applied every
  tick by `applyRotationForce()` → `slewToward(dir, h)` (slew the nose to an
  arbitrary direction, braking-curve rate, torque capped at `maxTorque()`) and
  `killRotStep(h)`. This is the *exact* "point the nose at a direction" primitive
  the guidance layer needs, and it already damps the third-axis spin (the
  prograde-wobble fix). The Autopilot window (`src/gameui.cpp`, `ui::Window
  ("Autopilot", …)`) already toggles these modes.
- **Thrust** — `Command(ShipCmd(Thrust))` → `ApplyThrust(step)` arms
  `armedThrust` and drains the **active stage's** tanks pro-rata
  (`consumeResourceMass`); `applyThrustForce()` re-applies the force before every
  substep. `thruster_util` (0..1) is the throttle.
- **Staging** — `activeStage()` (lowest stage number still present),
  `numStages()`, `separateStage(stage)` (drop every part on that stage, cut the
  welds, remove the bodies). The SPACE key handler (`src/events.cpp`) already
  calls `separateStage(activeStage())`. Stages are labelled 1 (booster) → N
  (top), dropped low-to-high — standard rocket order.
- **Delta-v / TWR** — `getDeltaV()` (Tsiolkovsky over the remaining fuel),
  `getTWR()`, `getFullThrustTWR()`, `getMaxTWR()`, `getMass()`, `getFuelMass()`.
- **Rails** — `onRails`, `goOnRails()`, `leaveRails()`, `railsTick()`. A coasting
  ship is parked out of Bullet and advanced analytically on its conic; control
  input wakes it (`tick.cpp` calls `leaveRails()` on any command).

**The tick / command flow (`src/tick.cpp`)**
- Per fixed tick: `clearThrust()` + `clearRotCmd()`, then arm commands from the
  held keys (`Command(…)`), then integrate in substeps (re-applying gravity +
  thrust + rotation before each substep). The engaged `slewRequest` is applied
  *after* `clearRotCmd()` so it holds across ticks — the same hook a persistent
  mission autopilot should use.

**Orbit math (pure, header-only, tested)**
- `src/orbit.h` — `computeOrbitElements(pos, vel, mu)` (a, e, peri, apo, inc,
  period, true anomaly, time-to-apsis), `propagateKepler(pos, vel, mu, dt)`
  (universal-variable propagation, any conic, any dt), `railStateFromElements`.
- `src/transfer.h` — `solveLambert`, `planTransfer` (min-dv parent→child),
  `porkchopGrid`. All header-only so they pin without Bullet/GL.

**Frames (`src/frame.h`)**
- Inertial vs rotating nodes, `getNonRotFrame()`, `GetOrientRelTo`,
  `GetPositionRelTo`, `GetVelocityRelTo`, `GetStasisVelocity`.
- The **`shipInertial()` idiom** (`src/transferplanner.cpp`) lifts a ship's COM +
  velocity from its (possibly rotating) frame into the SOI body's inertial frame
  and returns `(r, v, mu)` — *this* is the state the guidance laws consume.

**World (`src/terrain.h`, `src/terragen.h`)**
- `TerrainBody` carries `mu`, `radius`, `g`, `soi`. `surface.atmosphere`
  (`AtmosphereParams`) is **visual only** today: `enabled`, `thickness`,
  `color/power/intensity` for a Fresnel rim. There is **no** drag or lift force.
  Bodies with `atmosphere.enabled`: Eve, Kerbin, Shay, Duna, Jool, Laythe.

**Launch site (`src/ships.cpp`)**
- The pad is placed at `pad_dir ≈ +Z` (near the body's equator) and the ship is
  oriented `faceAlong(pad_dir)` — **nose radially outward** ("up"). Spin axis is
  +Y (north), so "east" (the direction of rotation) is the tangent to the
  equator. This matters for how inclination maps to launch direction (below).

**Test harness**
- Pure-math unit tests in `tests/` (`test_orbit.cpp`, `test_transfer.cpp`,
  `test_attitude.cpp`, `test_slew3d.cpp`, …) run via `make test`.
- E2E battery `e2e/run.py` + `e2e/cases/*.txt`: launches `./osp` under Xvfb,
  drives it with `--sim-press` / `--sim-mouse`, and asserts on parsed stdout
  (`[orbitlog]`, `[dbg]`, `[attlog]`, …). `make e2e`.
- CLI flags (`src/cli.h`/`cli.cpp`) already carry `--body`, `--scenario`,
  `--ship`, `--transfer-target`, `--orbit-log`, `--exhaust-scale`, `--sim-*`.

---

## The two missions

### Ascent → circular orbit

Goal: from a pad, reach a **circular** orbit at the user's chosen **altitude**
`alt` (target radius `r_t = R + alt`, circular speed `v_t = sqrt(mu / r_t)`).

Phases (a guidance state machine; each phase = a nose target + a throttle + a
staging decision, computed per tick from the ship's inertial state):

- **A. Vertical climb.** Nose = radial-out, throttle = full. Climb to a
  pitch-over altitude `h_pitch`. (With a real atmosphere later, this altitude is
  where the pitch program is drag-optimized; without one, a fixed value is fine.)
- **B. Pitch program (gravity turn).** Rotate the nose from radial-out toward
  **prograde (east)** as altitude rises — nose ≈
  `normalize(radial·cos θ + prograde·sin θ)`, with θ ramped 0°→~90° over
  `[h_pitch, h_pitch_end]`. Full throttle, **auto-stage** as stages empty. This
  builds the horizontal velocity and raises the orbit's apoapsis.
- **C. Circularization.** When the computed orbit
  (`computeOrbitElements`) has apoapsis ≈ `r_t` and the ship is near periapsis,
  burn **prograde** to raise periapsis up to `r_t` → circular. Then cut the
  engine and **park on rails**.

Why this shape: it is the minimum-energy profile, each phase is unambiguous to
implement *and* to assert (apoapsis rises in B, eccentricity → 0 in C), and it
reuses the existing prograde slew + throttle + staging primitives directly.

Feasibility: before engaging, the planner checks `getDeltaV()` (plus a staging
margin) against the required `Δv` (a closed-form estimate from the target orbit)
and refuses/warns if the ship can't make it.

### Landing / suicide burn

Goal: from **circular orbit** or a **suborbital** trajectory, touch down gently
(vertical velocity small, no hard impact).

Phases:

- **D. Deorbit** (only if in a closed orbit above the surface). Burn
  **retrograde** to bring the periapsis down into the surface band. (From a
  suborbital trajectory, skip this and go to E.)
- **E. Suicide burn.** As the ship falls, ignite **retrograde (anti-prograde)**
  and throttle to hold a target descent rate. Ignition is timed so the ship
  decelerates to a soft stop at the surface — see the "closed-loop descent"
  design note below (this is the drag-robust choice). **Auto-stage** as needed.
- **F. Final approach / touchdown.** As altitude → 0 and velocity → 0, cut the
  engine and let it settle. Detect touchdown (ground contact / the altitude
  band `inTerrainBand()` already uses) and stop.

### Auto-staging (shared by both missions)

The one rule, computed per tick from the ship's tanks:

> If the guidance wants thrust (`throttle > 0`) **and** the active stage's
> remaining propellant is below a small threshold (e.g. < ~0.5 s of full flow)
> **and** a higher stage with a thruster exists → `separateStage(activeStage())`.

This reuses `activeStage()` / `separateStage()` verbatim. The threshold avoids a
dead zone (engine stops firing the instant a tank empties) and fires *before*
the stage is fully dry. It is a pure function of (tanks, stage map, "need
thrust") and is unit-testable with no physics.

---

## Does inclination make sense?

**Yes — physically it is meaningful, but it is a Phase-3 option, not a v1 knob.**

- **Meaningful.** Inclination sets the orbital plane relative to the body's
  equator. The game is already 3D, bodies have spin axes, polar orbits already
  exist as spawn scenarios (`high-polar`), and the transfer planner solves
  arbitrary planes — so inclined orbits are fully supported by the physics.
- **Coupled to launch azimuth + site latitude.** To *choose* an inclination you
  choose a launch azimuth; the two are linked through the launch site's
  latitude. At the current pad (near the equator) the link is the simple one,
  **azimuth ≈ inclination**, so it is a clean parameter *there*.
- **Recommendation.** v1 (Phase 1) is **altitude only, equatorial launch** —
  the minimum-energy orbit and the same convention as the existing `rot-orbit`
  scenario. Phase 3 adds inclination by rotating the prograde target vector by
  the chosen angle about the radial axis (equivalently, choosing the launch
  azimuth), which is a small, well-contained extension of the Phase-1 pitch
  program.

So the answer to "does inclination make sense?": **yes, and it slots in
cleanly — but defer it so v1 stays a single well-understood parameter (altitude).**

---

## Atmospheric effects (future — design for it now)

Today `surface.atmosphere` is a visual rim; there is no aerodynamic force. When
drag/lift are added, two of the guidance laws are the ones that care:

1. **Descent / suicide burn.** The robust choice is a **closed-loop** law:
   throttle so the vertical velocity approaches the target descent rate, rather
   than an open-loop "burn for N s at altitude H." A feedback law is *inherently*
   robust to an unknown drag term (drag just changes how much throttle is
   needed), whereas an open-loop timing is sensitive to it. **We implement the
   closed-loop form from the start** so adding drag later is a physics change,
   not a guidance rewrite.
2. **Ascent pitch program.** With a heavy atmosphere the optimal pitch-over
   altitude drops (fly more vertical to shed drag). We keep the pitch program as
   a **parameterized altitude curve** so the atmosphere work can retune it per
   body without touching the state machine.

Concrete "design-for-it" hooks to add now (cheap, in the plan):
- A per-substep **aero force hook** beside `applyControlForces()` (a no-op
  today) so drag/lift slot in without re-plumbing the tick.
- Keep the guidance **frame- and body-agnostic**: it reads `(r, v, mu, g, mass,
  thrust, ve)` and the body's `surface.atmosphere.enabled` flag, so an
  atmospheric body is just "the same laws with a force term added."

---

## Architecture

Follow the established **pure-math / game-side** split (the same shape as
`orbit.h` + `TransferPlanner`, and `evamath.h` + `Kerbal`):

- **`src/guidance.h`** (new, header-only, GL/Bullet-free) — the pure laws:
  inertial-state extraction (the `shipInertial()` idiom), circular-orbit target
  math (`r_t`, `v_t`, the circularization `Δv`), the pitch-program nose target,
  the descent feedback law, and the staging decision. Pinned by
  `tests/test_guidance.cpp`.
- **`GuidancePlanner`** (new, small class — `src/guidance.cpp` or a section of
  `game.cpp`) — owns the engaged mission (`ASCENT` / `LAND` / `COAST`), the
  target (altitude, later inclination), the current phase, and per-tick state.
  `update(Game&, Vehicle&)` → a `GuidanceCommand { noseDir, throttle, stageNow,
  coast, phase, progress, message }`.
- **`Vehicle` hooks** (surgical additions to `src/vehicle.h`):
  - a new `SlewGuidance` mode + a `glm::dvec3 guidanceDir` member;
    `slewTargetDir()` returns `guidanceDir` for that mode. This lets the planner
    drive the *existing* `slewToward()` for arbitrary (non-prograde/retro)
    targets with no change to the attitude law.
  - nothing else: throttle is the existing `thruster_util`, staging is the
    existing `separateStage()`.
- **`tick.cpp` integration** — when a mission is engaged: wake the ship from
  rails (`leaveRails()`), apply the `GuidanceCommand` (set `slew = SlewGuidance`,
  `guidanceDir`, `thruster_util`, and `separateStage()` if `stageNow`), and let
  the existing substep loop do the physics. The guidance runs **once per tick**
  (cheap pure math); the per-substep work is the thrust/slew that already exists.
- **UI** — extend the existing Autopilot window (`src/gameui.cpp`): an
  "Insert to orbit" button + altitude slider (and later an inclination slider),
  and a "Land" button. Show the current phase + a progress/delta-v readout.
- **CLI (for e2e)** — new flags mirroring the existing style:
  `--autopilot ascent|land`, `--orbit-alt <m>`, (Phase 3) `--orbit-inc <deg>`.
  Plus an **`[autopilotlog]`** stdout line (like `[orbitlog]`) reporting
  phase/throttle/progress/target so `e2e/run.py` can assert on the mission
  directly. Add an `autopilot` entry to `run.py`'s CHECK namespace.

This keeps the new surface area small: one new pure-math header, one small
planner class, a few surgical `Vehicle` hooks, one `tick.cpp` branch, one window
section, a few CLI flags, and one test file. No changes to the physics core,
frames, or the existing attitude law.

---

## Staged work plan

Each phase is independently shippable, has an e2e case, and leaves the tree
green (`make test` + `make e2e`). Suggested order and dependencies:
**0 → 1 → 2 → (3 ∥ 4)**. Phase 3 (inclination) and Phase 4 (atmosphere) are
independent of each other and both build on 1 + 2.

### Phase 0 — Foundation: plumbing, pure math, and the test surface
*No mission behaviour yet — just the scaffolding everything else stands on.*

- Add `src/guidance.h`: inertial-state extraction (reuse the `shipInertial()`
  idiom), circular-orbit target math, the staging decision function. Header-only.
- Add the `Vehicle` hooks: `SlewGuidance` mode + `guidanceDir` member + the
  `slewTargetDir()` case. (Small, surgical; the attitude law is untouched.)
- Add CLI flags `--autopilot`, `--orbit-alt`; wire the `[autopilotlog]` stdout
  line; add `autopilot` to `e2e/run.py`'s CHECK namespace.
- Add `tests/test_guidance.cpp`: inertial transform round-trip, circular-orbit
  `v_t`, the staging decision (empty/dry/need-thrust cases).
- **Exit:** builds; `make test` green; a trivial "coast" mission engages and
  logs a `[autopilotlog]` line; `make e2e` smoke still passes.

### Phase 1 — Ascent to a circular orbit (headline #1)
- Implement the ascent state machine (A vertical climb → B pitch program → C
  circularization) in `GuidancePlanner`, driving the Phase-0 hooks.
- **Auto-staging** (the shared rule) active throughout.
- Delta-v feasibility check (refuse/warn if `getDeltaV()` can't cover the target).
- UI: "Insert to orbit" + altitude slider in the Autopilot window; phase +
  progress readout.
- **E2E:** from the pad on a body where the test ship has TWR > 1 (Moho, per the
  existing `03-thrust-ascent` note — Kerbin is too heavy for every current ship),
  engage ascent; assert via `[autopilotlog]` + `[orbitlog]` that the orbit
  reaches the target altitude and is ~circular (`ecc < tol`,
  `peri ≈ apo ≈ r_t`). A multi-stage case asserts `Stage: dropped` appears and
  the orbit is still reached.
- **Exit:** a ship that starts on the pad reaches a circular orbit at the chosen
  altitude, auto-staging as needed, fully automated.

### Phase 2 — Landing / suicide burn (headline #2)
- Implement the landing state machine (D deorbit → E suicide burn → F
  approach/touchdown), reusing auto-staging.
- **Closed-loop descent law** (throttle to hold target descent rate) — the
  drag-robust form, from the start.
- Touchdown detection via the existing `inTerrainBand()` / ground-contact path.
- UI: "Land" button; phase + descent-rate readout.
- **E2E:** (a) from a circular orbit (`rot-orbit` scenario), engage land; assert
  the ship reaches the surface with low vertical velocity and the mission ends
  in a settled state (`[dbg]` alt → ~0, `v` → small; `[autopilotlog]` phase →
  landed). (b) From a suborbital trajectory (a deorbit ellipse), same assertion.
- **Exit:** a ship in orbit (or suborbital) lands softly, automated, on a
  body with TWR > 1 for the descent.

### Phase 3 — Inclination / launch azimuth *(optional, after 1+2)*
- Add `--orbit-inc` (degrees) to ascent; rotate the prograde target vector by the
  inclination about the radial axis (choose the launch azimuth).
- Reuse the Phase-1 pitch program; only the target-plane construction changes.
- **E2E:** launch to a polar orbit (90°) and to a mid inclination; assert the
  orbital inclination (`[orbitlog]` `inc`) matches the requested value within
  tolerance.
- **Exit:** the user can select the orbital plane, not just the altitude.

### Phase 4 — Atmospheric bodies *(gated on the aero model)*
- **Blocked** until drag/lift exist (today `surface.atmosphere` is a visual rim).
  This phase is the autopilot's share of that work:
  - wire the per-substep aero force hook into the tick (it is a no-op until the
    force is implemented);
  - retune the ascent pitch program per body (`surface.atmosphere`-aware);
  - confirm the **closed-loop descent** law still lands on an atmospheric body
    (it should — that is why we chose the feedback form).
- **E2E:** ascent + landing on an atmospheric body (Kerbin / Duna) with drag
  active; assert the autopilot still reaches orbit / lands softly.
- **Exit:** the autopilot works on bodies with air, not just airless ones.

---

## Risks / open questions

- **TWR for the e2e bodies.** Ascent needs TWR > 1 at liftoff; Kerbin is too
  heavy for every current ship (existing `03-thrust-ascent` uses Moho). Phase 1's
  e2e should pin a body + ship pair that actually lifts off (or use
  `--exhaust-scale` as the existing cases do) so the test is deterministic.
- **Pitch-program tuning.** The gravity-turn curve (A→B) needs a few runs to look
  right and not punch into the surface or overshoot the target. Keep it a
  parameterized curve so it is tunable per body (and per atmosphere in Phase 4)
  without code changes.
- **Staging dead zone.** If the last stage is the one that must carry the
  circularization burn, the staging rule must not drop a stage the ship can't
  survive the burn without — guard on "higher stage has thrust" and on the
  delta-v budget.
- **Rails interaction.** The guidance must `leaveRails()` before it thrusts and
  drop the time warp (the existing control-input path already does this); make
  sure an engaged mission can't leave the ship warped on rails while it "thinks"
  it is thrusting.
- **Crew lock.** Staging a stage that carries a crewed capsule is refused
  (existing `events.cpp` rule). The autopilot's staging call should go through
  the same guard so it never strands a crewed capsule mid-mission.

## Key references

- `src/vehicle.h` — `slew`/`slewTargetDir()`/`slewToward()`, `Command()`,
  `ApplyThrust()`/`applyThrustForce()`, `activeStage()`/`separateStage()`,
  `getDeltaV()`/`getTWR()`, `onRails`/`leaveRails()`, `inTerrainBand()`.
- `src/tick.cpp` — command arming, substepped integration, the `slewRequest`
  apply-after-clear hook, leave-rails-on-input.
- `src/orbit.h` — `computeOrbitElements`, `propagateKepler`,
  `railStateFromElements`.
- `src/transfer.h` — `solveLambert`, `planTransfer` (the delta-v/transfer math
  the feasibility check reuses).
- `src/transferplanner.cpp` — the `shipInertial()` idiom (state → SOI inertial).
- `src/frame.h` — `getNonRotFrame()`, `Get*RelTo()`, `GetStasisVelocity()`.
- `src/terrain.h`, `src/terragen.h` — `TerrainBody` (`mu`/`radius`/`g`/`soi`),
  `surface.atmosphere` (visual-only today).
- `src/ships.cpp` — pad placement (`pad_dir ≈ +Z`, `faceAlong` → nose out), the
  equatorial launch site.
- `src/vehicle.cpp` — `spawn_vehicle` (circular-orbit scenario: `v = sqrt(mu/r)`,
  prograde construction — the same target the autopilot must reach).
- `src/gameui.cpp` — the Autopilot window (where the UI lands).
- `src/cli.h`/`cli.cpp` — the flag pattern (`--body`, `--scenario`,
  `--transfer-target`, `--sim-*`).
- `e2e/run.py` + `e2e/cases/` — the harness and the `[orbitlog]`/`[dbg]`
  assertions the new cases follow.
