# Ascent autopilot: findings and staged plan — 2026-11-16

Snapshot taken while sequencing issue **#927** ("Ascent autopilot v2 — launch from
bodies with atmosphere"). No code was changed to produce this report; everything below
is read from source or measured from `--ascent-log` / `--drag-log` runs whose commands
and raw output are in this directory.

Files here:

| file | what |
|---|---|
| `autopilot-v2-findings-and-plan.md` | this report |
| `927_probe_kerbin.log` | `mars_explorer_mk1`, Kerbin pad, ask 150 km, 8x warp — the failure |
| `927_probe_moho10.log` | same ship, Moho pad, ask 100 km, 10x warp — the success |
| `927_probe_moho.log` | same at 16x warp — the on-rails dead zone (a probe mistake, kept because it is instructive) |
| `927_probe_drag.log` | Kerbin flight with `--drag-log` — the drag/angle-of-attack numbers |

---

## 0. The question

#927 asks for an ascent autopilot that reaches orbit from Kerbin and Duna in
`mars_explorer_mk1`, accepting a 150-200 km result. v1 (#610, closed) flies a
pad-to-circular-orbit profile today and its own header admits what it leaves out
(`src/guidance.h:209-214`):

> no atmosphere (drag/pressure are not even in `GuidanceState`), no descent or landing,
> no transfers, **no staging** (it will burn a single stage until the stage's thrust
> reads zero and then Fail), no plane change after launch, and no mid-course re-burn.

Two of those clauses turned out to matter far more than the ticket text assumed.

---

## 1. What was measured

Same ship (`res/ships/mars_explorer_mk1.json`: 30 parts, 6 decouplers, stage labels
`{4: 9 parts, 3: 1, 2: 2}`, 18 parts untaged hence stage 1; 5x `engine_r2.25h4.5` +
1 `orbital_engine_r1.5h1.5` + 1 `nuclear_engine`), three beds:

| bed | warp | outcome |
|---|---|---|
| Moho, ask 100 km | 10x | `phase=done` at t=590 s, `peri_asl=100018 m`, `apo_asl=100629 m`, `why="periapsis at the ask, apoapsis inside the band"` |
| Kerbin, ask 150 km | 8x | `phase=failed` at t=404 s, `why="the tanks are dry"`. Apoapsis **never** above 331 m in 400 s at `lever=1.00`. `accel` (availableAccel) rose 12.11 → 29.39 m/s² as it burned itself light |
| Moho, ask 100 km | 16x | `phase=pad` for 2824 s, `why="ship is on rails: no commands sent"` |

Reading the Kerbin failure: `accel_max` climbing while apoapsis stands still is the
signature of mass being thrown away, not of a stage that cannot push.

The 16x row is a probe mistake worth recording: `kRailsWarp = 11` (`src/game.h:57`)
puts every hull on rails at 11 and above, and `AscentLaw::step()` refuses to command a
railed hull (`src/guidance.cpp:128-133`). **Warp 10 is the top live step.** Any future
autopilot probe must stay at or below it.

---

## 2. Finding A — the law never stages, and that is worth ~1 km/s

`flyAscent` writes exactly four commands and none of them is a stage
(`src/guidance.cpp:394-412`): `ShipCmd(Sas, SlewFree)` (a latch), `ShipCmd(Aim)`,
`ShipCmd(Thrust)`, `ShipCmd(ThrottleSet)`. `grep` over `src/guidance.cpp` finds no
`advanceStage`, no decoupler, no `Game::stage`.

Consequence, in stage semantics: `activeStage_` starts at `totalStages_` and a rocket
engine is ignited iff `stage >= activeStage_` (vehicle.h:1469-1472). So an armed
`mars_explorer_mk1` flies its **stage-4 side boosters only**, carries the dry mass of
the sustainer, the orbital stage and the nuclear stage all the way up, and Fails when
the boosters run out — with three stages still full of propellant aboard.

Rough budget for that hull (`res/data/parts.json`, thrust = rate x `ve` per
`shipdef.h:580`): ~69.4 t propellant on ~37 t dry. One un-staged burn at the
mass-flow-weighted `ve` of the big engines (~4.4 km/s) is **~4.7 km/s**; the staged
sequence is **~5.85 km/s**. Staging is therefore worth about **1.1 km/s** here — a
margin, not a capability, which is exactly what the Moho row above shows (the un-staged
burn clears Moho's ~700 m/s orbit needs with room, and clears nothing on Kerbin).

---

## 3. Finding B — inside air, v1's phase machine cannot advance at all

This is the finding that reframes #927. It is not that the law lacks a max-Q throttle
bucket; it is that its **transition condition is unreachable** once drag is real.

`Ascent -> Coast` fires on apoapsis reaching the ask (`src/guidance.cpp:213`):

```cpp
if(elliptic && apo_asl >= target.apo_asl) { phase_ = AscentPhase::Coast; }
```

and `Done` is a pair of apsides (`:201-204`). Now the measured profile on Kerbin
(`927_probe_drag.log`):

```
t=132.9s alt=5156 m |v|=152.1 m/s |F|=923.9 kN |tau|=1.0e5 Nm AoA=28.9 deg A=137.2 m2 rho=0.7265
t=165.3s alt=6067 m |v|=170.6 m/s |F|=999.9 kN |tau|=9.4e3  Nm AoA=25.3 deg A=131.1 m2 rho=0.6586
t=197.7s alt=6368 m |v|=110.1 m/s |F|=433.8 kN |tau|=1.9e5  Nm AoA=44.5 deg A=158.6 m2 rho=0.6373
```

~1000 kN of the ~1265 kN on tap is spent pushing 141 m² of asparagus through the air,
so the hull is pinned near 170 m/s. The osculating apoapsis of a hull held at terminal
speed is a couple of hundred metres **wherever it is** — which is precisely what the
`[ascent]` rows print (`apo_asl` 227-331 m for the entire flight). Therefore:

- the `Ascent` phase never ends, because ending it requires an apoapsis the drag will
  not allow;
- the pitch schedule cannot help: `pathAngle(alt, ask) = atan2(max(1000, 0.03*ask), alt)`
  (`src/guidance.cpp:18, 36-39`) holds the stack 85-87 deg off the horizon for the first
  minute of a 150 km ask — head-on into the air, never building the downrange speed that
  would buy apoapsis cheaply;
- the `throttle` law (`speedCap`, `:53-56`) is a pure two-body vis-viva number and has
  no idea it is fighting an atmosphere at all.

So v2's real change is **what the climb is keyed on**, not the addition of an air term.
Any plan that starts by adding a max-Q bucket to the existing transition will still sit
in `Ascent` forever.

## 3b. Finding C — steering at high dynamic pressure is a live question, not a theory

The same rows show angle of attack wandering to **44.5 deg** with aero torque reaching
**1.9e5 N m**. #399 ("a rocket cannot hold one degree of angle of attack: wheels lose to
aero by ~20x") is partly stale by its own EDIT — most engines now have TVC — but its
measurements were taken on this exact hull, and #403 ("a stack that starts tumbling
tumbles harder") remains unaddressed. Whether this needs work is a *measurement*
question, which is why the instrument ticket comes before the law rewrite.

---

## 4. The staging machinery: what exists, and what does not

Verified line by line, because #928's shape depends on it.

**A stage press is pointer-safe.** `Vehicle::extractSubtreeAsShip(Part *root, ...)`
(`src/vehicle.cpp:5395`) BFS's `root` **plus its descendants** (`:5401-5419`); if that
set is the whole ship it returns `nullptr` and asserts the tree invariant (`:5421-5429`).
The original ship keeps everything else — `src/vehicle.cpp:5550-5553`:

```cpp
std::vector<Part *> keep;
for(Part *p : parts) { if(!droppedSet.count(p)) { keep.push_back(p); } }
parts.swap(keep);
```

With `res/ships/mars_explorer_mk1.json` and `res/ships/stager.json`, `parts[0]` is the
capsule and every `parent` names the part **above** it, so children are the lower stack:
the decoupler and the boosters below it leave, the flying stack stays on the original
`Vehicle *`. **`Game::ascent_ship` does not need repointing.** (It is cleared only on
hull death, `game.cpp:445`, or world teardown, `:1400`.)

Two side effects to respect: if the dropped subtree carried the controller, the kept
ship adopts a new one (`vehicle.cpp:5555-5556`, `controller = seatOf(...)`), and the
kept ship is rebuilt mid-tick (`rebuildCompound()`, `buildFuelGroups()`,
`dropDrainWindow()`, `:5554-5558`). The law re-reads `accel_max`, `has_propellant` and
the conic every tick (`guidance.cpp:349-361`), so it will pick the new numbers up on the
next call — no cached state to invalidate.

**Ignition is spelled inline in five places**, not one predicate: `vehicle.cpp:2916`
(`producedThrust`), `:5816` (`ceilingSum`, behind `availableThrust`/`availableAccel`/
`availableAccelAlong`), `:5898` (`canBurn`), `:5928` (`ApplyThrust`), `:6197` (the
attitude/TVC walk). Any change to stage-gating has to keep all five agreeing.

**There is no per-stage burn time anywhere.** `struct StageRow` (`src/staging.h:14-24`)
carries `deltaV`, `minTWR`, `maxTWR`, `massStart`, `massEnd`, `engines`, `drops` — no
seconds. `computeStaging(const BuildShip &, ...)` (`staging.h:32`) takes a **`BuildShip`**,
not a `Vehicle`, and assumes **tanks start full**, so it cannot answer "how long until
this flying hull runs dry". Its internals do model flow (`SimPart::mdot`,
`staging.cpp:19`; `runBurn`, `staging.cpp:452`) but `BurnAcc` (`staging.cpp:431-439`)
keeps no elapsed time. Cost is real: 84 us for a one-group ship, 473 us for 240 groups
(`staging.h:36-42`), which is why the Staging window caches against `stagingSignature`.

The cheapest honest answer to "seconds to burnout of the ignited stage" is a small loop
mirroring `canBurn()` (`vehicle.cpp:5886-5909`): for each `p->isThruster() &&
p->stage >= activeStage_`, divide `availableResourceMass(r, p)` (`vehicle.h:1377`) by
`p->def->propellant_rate[r]` (`shipdef.h:360`) at full throttle, take the minimum over
engines. Parts-walk only, no allocation — and note `availableAccel()`, which the driver
already pays every tick, is itself two part walks plus an atmosphere read
(`vehicle.cpp:5805-5807, 5866`).

**The player door does more than step a counter.** `Game::stage()` (`game.cpp:2343-2458`)
wakes a railed hull and drops warp (`:2349-2355`), **refuses if crew ride a capsule on
the dropped stage** (`:2357-2371`), deploys stowed chutes first so a chute on a dropping
subtree goes with it (`:2373-2381`), extracts each staged decoupler as a new vessel named
`<parent> <decoupler label>` de-duplicated against the roster (`:2399-2410`), and steps
the counter even on a decoupler-less stage (`:2436`). It never writes `Game::ship`. An
autopilot that stages must go through this door (or a `Vehicle*`-taking twin of it), not
call `advanceStage()` itself, or it will silently skip the crew rule and the chute rule.

**Coverage today:** `tests/test_guidance.cpp` is the only test that touches
`AscentLaw`/`flyAscent`; staging is tested graph-wise (`tests/test_staging.cpp`),
per-stage delta-v (`tests/test_staging_dv.cpp`), and flight-wise without an autopilot
(`e2e/cases/flight/02-staging-basic.txt`, `541-booster-root-decoupler.txt`,
`38-asparagus-flight.txt`). **No existing test combines the two.** The
`flight/610-ascent-100km` case flies `racer`, whose 4 parts carry no `stage` tag at all —
the case passes because its ship has nothing to stage.

Good test hull: `res/ships/stager.json` — 7 parts, one decoupler, two labels
(`capsule_1 > wheel_1 > tank_upper > engine_upper` at stage 1,
`dec_1 > tank_boost > engine_boost` at stage 2). At spawn `activeStage_ = 2`, so the
booster is the lit stage and one press moves 3 parts away.

The unit harness cannot yet model a stage drop: `simulate(...)` (`tests/test_guidance.cpp:57`)
drains mass continuously from a single `MDOT`/`PROPPANT` pair (`:79`); a new closed-loop
test needs a stage table (thrust, mdot, propellant, dry mass per stage) so the synthetic
flight can step mass discontinuously at the tick the law asks for a drop.

---

## 5. Instrumentation: what the guidance work needs to see

`--ascent-log` prints `t phase lever pitch peri_asl apo_asl inc_eq node_r accel g why`
(`tick.cpp:514-581`). There is **no dynamic pressure, no flight-path angle, and no loss
account** anywhere in the ascent telemetry; the numbers in section 3b had to be
reconstructed from a second log (`--drag-log`) that cannot be joined to the first.

#675 already lays out the shape and the law to keep: `Q` is already formed per surface
and thrown away, the flight-path angle wants the local-vertical frame `ShipView` has,
and the three losses are **integrals over a burn**, so they belong where the forces are
(a small struct beside the flight model / `Vehicle::flog`), ticked with the substep,
reset on a load (#304), and published — so the recorder stays a pure recorder (#662) and
a loss curve cannot disagree with the forces the Game Debug window shows.

MechJeb does exactly this, and it is on disk at
`references/ksp-mods/MechJeb2/MechJeb2/MechJebModuleFlightRecorder.cs:293-297`:

```csharp
GravityLosses  += dt * Dot(-OrbitalVelocity.normalized, GravityForce);
DragLosses     += dt * DragAcceleration;
DeltaVExpended += dt * CurrentThrustAcceleration;
SteeringLosses += dt * CurrentThrustAcceleration * (1 - Dot(OrbitalVelocity.normalized, Forward));
```

Three places **not** to copy it:

1. It bills gravity loss while coasting too, with no burn gate — fine as a flight
   odometer, misleading as a score, and v1 spends most of a profile in `Coast`.
2. `OrbitalVelocity.normalized` is undefined at the pad (the probe read 7.6 m/s at
   t=3.5 s), which is exactly when v2 steers hardest. #900 learned the sibling lesson
   about orbit normals on a near-radial climb.
3. To score the **autopilot** rather than the pilot, the reference direction should be
   what the law asked for (`ShipCmd(Aim)`, `guidance.cpp:395`) rather than the velocity
   vector. Same arithmetic, and it separates "the law wanted a bad path" from "the hull
   could not hold the path" — today unanswerable, because `[slew]`'s law error and
   `[drag]`'s `AoA=` live in two logs that cannot be joined.

---

## 6. Constraints settled during this session (not open questions)

- **The air top stays a hard boundary.** `Vehicle::canRail()` (`vehicle.cpp:7097`) is
  `isGrounded() || !inAtmosphere()`, and `inAtmosphere()` (`:7060`) is
  `radius_from_centre - seaLevelRadius < atm.top()`. The top is simultaneously the perf
  wall and the reason high orbits are stable. The uncapped mode exists
  (`drag.h:31-33`, `height = 0` means "the tail runs down to `kRhoFloor`") and every
  shipped body declines it. Do **not** answer #816/#682 by raising `height`: Kerbin's
  honest thinness-based top is ~200 km, which would put this plan's target orbits inside
  the shell and move the rails gate with it. The smooth-edge idea is parked as **#929**.
- **SAS must not out-fly the player, and substeps are an engine detail.** #421's ~2 deg
  prograde wobble is the accepted cost of that rule, decided in that ticket's discussion
  rather than by physics. Any v2 that "fixes" the wobble by giving the guidance law a
  finer control rate than the pilot has is a policy reversal, not a fix.
- **The autopilot driving warp (#617) follows v2, it does not lead it.**
  `Game::enter_rails_warp()` (`game.cpp:2065` region) refuses while any ship is in an
  atmosphere, and a rails tick stepping over the air is #340's hazard. The cheap half —
  an armed law never sitting railed, reusing the existing wake paths at
  `tick.cpp:150-156` and `:166-175` — is one line at `Game::armAscent` (`game.cpp:460`).

---

## 7. Staged plan

Each stage has an exit criterion expressed as a number, because the failure mode of this
feature is "it looked like it worked".

### Stage 0 — a fixed baseline (done, in this directory)

One scenario, pinned and repeatable, flown before anything changes:

```
SDL_VIDEODRIVER=offscreen ./osp --timeout 400 --time-accel 8 \
  --startship mars_explorer_mk1,res/ships/mars_explorer_mk1.json,Kerbin,pad \
  --ascent 150e3 --ascent-log --drag-log
SDL_VIDEODRIVER=offscreen ./osp --timeout 500 --time-accel 10 \
  --startship mars_explorer_mk1,res/ships/mars_explorer_mk1.json,Moho,pad \
  --ascent 100e3 --ascent-log
```

Baseline: Kerbin `failed / the tanks are dry`, apo never above 331 m. Moho `done`,
peri 100,018 m.

**Exit criterion:** met. The Moho number is the control: any change that breaks it is a
regression, and it costs 60 s to re-run.

### Stage 1 — #675: publish Q, the flight-path angle, and the three losses

Additive only; changes no behaviour. Three accumulators beside the flight model ticked
with the substep, `lastDynPressure` beside the existing `lastDrag*` fields, the
flight-path angle derived where the local-vertical frame lives, and five new
`--ascent-log` columns (`q= gamma= grav= drag= steer=`). Per-burn identity via the
existing stage segmentation (`TimeSeries::stage()`), reset on load.

**Unit pin (the one the body asks for):** the decomposition must add up —
`Δv_expended − (gravity + drag + steering)` tracking the orbital energy gained, asserted
in `tests/` against a synthetic burn, because "three separate places computing three
losses" is how they stop adding up.

**Exit criterion:** the Stage 0 Kerbin log re-run prints a loss budget, and
`steer + drag + grav + (orbital energy gained) ≈ Δv_expended` inside tolerance. This is
the first time the flight is *scored* rather than merely described.

### Stage 2 — #928: the law asks for a stage; the driver performs it

Design, with the parts deliberately split:

- **The pure law decides, the driver acts.** `AscentCommand` gains one field — a request
  to drop the current stage — so `AscentLaw::step()` stays free of `Vehicle` and the
  unit test can feed it a synthetic stage table. The driver (`flyAscent`) or
  `tick.cpp:211` performs it, through a `Vehicle*`-taking twin of `Game::stage()` so the
  crew refusal and the chute rule are not bypassed.
- **The trigger, first cut:** burnout of the ignited stage — "seconds to dry" computed
  by the small loop described in section 4, dropping the stage a little before it stops.
  This alone recovers the ~1.1 km/s on `mars_explorer_mk1`, because its booster is the
  stage that burns first.
- **The interlock:** close the lever, stage, re-open — the pattern `releaseAscent()`
  (`guidance.cpp:323-329`) already demonstrates, and needed because armed thrust is
  cleared per tick while the lever latches separately.
- **Deliberately not in the first cut:** an efficiency-based trigger (drop a stage whose
  TWR stops paying). It is the better rule and the harder one; it needs Stage 1's
  numbers to tune honestly.

Risks to carry, all verified above: pointer safety (fine), controller adoption on a
dropped pod (handled by the extract), mid-tick mass/inertia/fuel-group rebuild (the law
re-reads per tick), crew aboard a dropped capsule (the door refuses, and the autopilot
must accept the refusal rather than retry every tick), chutes riding the dropped subtree
(the door deploys them), and a mission armed on a hull that never had stages (`racer`,
the existing 610 case — must stay green unchanged).

**Tests:** a closed-loop stage-table case in `tests/test_guidance.cpp` (does the profile
reach `done` with one fewer stage carried at apoapsis, and does it refuse to drop a stage
when there is none); an e2e case flying `stager` on Moho with the autopilot armed, pinned
on `phase=done` plus the ship count rising by one and the masses summing across the
split; `flight/610-ascent-100km` unchanged as the no-staging control.

**Exit criterion:** Kerbin Stage 0 scenario reaches a materially higher apoapsis and the
loss columns show the mass-step accounted for; Moho stays `done`; Kerbin still does not
reach orbit — **that is expected**, and it is the evidence that Stage 3/4 are needed.

### Stage 3 — the decision gate: does SAS have to change?

Re-fly Stage 0's Kerbin profile with Stages 1-2 in, and read the numbers rather than
watching the ship:

- if `steer` (with the reference being the law's own `Aim`) is a large share of the
  deficit, the hull cannot hold what the law asks — that is a steering-authority/SAS
  question, and #421/#399/#403 move onto the path;
- if `drag` dominates and `steer` is small, the problem is the trajectory law, and SAS
  stays untouched;
- if `grav` dominates, the throttle profile and the pitch schedule are the problem.

**Exit criterion:** one of the three branches is chosen with the numbers pasted into
#927. This is the answer to "are the first two tickets enough, or do we change SAS too",
and it should not be guessed before Stage 1 exists.

### Stage 4 — #927 itself

The design decisions Stage 3 will have informed, listed here so they are decided and not
discovered:

1. **What ends the climb.** Apoapsis cannot be the trigger in air (section 3).
   Candidates: downrange distance, or speed against a drag-aware ceiling, or a target
   dynamic-pressure-limited energy path. This is the core change.
2. **What the law sees about the air.** `GuidanceState` gains density and `q` — read at
   the hull, not modelled by the law — and `speedCap` gains a drag term or a
   dynamic-pressure ceiling.
3. **The pitch schedule.** `atan2(0.03*ask, alt)` is tuned for vacuum; in air the lean
   has to come earlier and lower, before the hull is at terminal speed.
4. **Where staging's trigger goes once the law knows `q`** — the TWR/drag-aware rule
   deferred from Stage 2.
5. **Ask validation:** "periapsis above the shell top" as part of what `Done` means
   (Kerbin 85 km, Duna 50 km), plus #615's honest verdict when a stage cannot reach the
   ask, and #614's terrain floor.

Ship side, deferred by agreement: Earth needs a hull that can climb its own gravity well
(no current ship does it without >1 exhaust multiplier). Kerbin's top at 85 km and a
150-200 km ask are the target; Kerbin pad TWR for `mars_explorer_mk1` is ~1.21, which is
thin for the current near-vertical opening and may itself force a better launch profile.

### Explicit non-goals

- #617 (the autopilot driving warp / rails coasts) — after, and narrower than it sounds,
  because `enter_rails_warp` refuses while any ship is in an atmosphere.
- #929 (the air-shell taper) — parked as a fudge.
- #425 (one attitude allocator) — would help, not blocking: the law already goes through
  `Vehicle::Command`.
- #617/#611/#619/#614 ask-validation work — after v2 exists to be validated against.

---

## 8. Risks and unknowns

1. **Kerbin may need a better hull as well as a better law.** ~1.21 pad TWR against
   141 m² of stack is a hard profile. If Stage 4 lands and Kerbin still fails, the
   answer may be a ship, not a law change.
2. **Steering authority at max-Q is unquantified for the current engine mix.** #399's
   numbers predate widespread TVC and its own EDIT says so; Stage 3 is where we re-measure
   instead of re-arguing.
3. **Five inline ignition predicates.** A staging bug can hide in one and not the others;
   worth collapsing when the law starts leaning on staging.
4. **Determinism at the air top.** A profile whose apoapsis sits near 85 km flips
   between drag and no drag in one tick (#816's knife-edge), which can make a v2 e2e case
   flaky for reasons unrelated to the law. Keep the test asks well clear of it.
5. **No per-stage flow model exists**, so "seconds to burnout" is new code on the
   guidance path; it walks parts every tick next to `availableAccel()`'s two walks, and
   performance at fleet scale is a standing project concern.

---

## 9. Ticket map

| ticket | state | labels | role in this plan |
|---|---|---|---|
| #610 | closed | — | v1, the law being extended |
| #612 | closed | — | arming the law from the Guidance window |
| #613 | closed | — | the rails stall; legibility half done, resume half → #617 |
| **#928** | open | Enhancement, High, M, Orbit+Vehicle | **Stage 2** — the law never fires a stage |
| **#675** | open | Enhancement, Medium, S, Orbit+UI | **Stage 1** — Q, flight-path angle, three loss totals |
| **#927** | open | Enhancement, High, L, Atmosphere+Orbit+Vehicle | **Stage 4** — the atmospheric law |
| **#421** | open | Enhancement, Medium, M, Orbit | **Stage 3's** candidate if steering is the deficit |
| #615, #611, #619, #614 | open | — | ask validation, after v2 |
| **#617** | open | Feature, Medium, L, Orbit | autopilot-driven warp; after v2 |
| **#816** | open | Bug, Medium, S, Atmosphere | off this path (decision recorded on the ticket) |
| **#929** | open | Enhancement, Low, S, Atmosphere | the parked taper |
