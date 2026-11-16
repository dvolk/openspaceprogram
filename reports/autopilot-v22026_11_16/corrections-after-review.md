# Corrections after adversarial review — same day (2026-11-16)

The report in this directory (`autopilot-v2-findings-and-plan.md`) is left exactly as
written, per the project's rule that a report is an immutable snapshot. This file is its
erratum: an adversarial fact-check pass found two load-bearing errors and a batch of
detail drift. Where the report is wrong, it is wrong in a way that would have sent the
next step off course, so the corrections matter more than the snapshot.

Scratch re-runs that established these are copied here too: `repro_kerbin.log`,
`repro_moho10.log` (independent re-flights of the Stage 0 scenarios).

## Two errors that change the plan

### 1. "Apoapsis never above 331 m" is false — and the failure shape is different

Report §1/§3 said the Kerbin probe's apoapsis "never" exceeded 331 m and concluded the
hull was pinned at terminal speed, so the apoapsis-keyed `Ascent -> Coast` transition
could never fire.

All 395 `[ascent]` rows of `927_probe_kerbin.log`:

| t | apo_asl | pitch |
|---|---|---|
| 6.9 s | 330.6 m | 85.9 deg |
| 249.9 s | 2107.7 m | 69.4 deg |
| **185.1 s (peak)** | **6660.4 m** | ~34 deg |
| ~270 s onward | decaying: 6611, 5910, 4460, 2859, 1347, 626, 227 | **climbing back to 87.1 deg** |
| first `failed` | — | `t=368.3 s` |

An independent re-flight (`repro_kerbin.log`, `--timeout 115 --time-accel 8`) peaks at
6660.56 m. So apoapsis *did* climb — to about 6.7 km, still twenty times short — and then
**decayed while the commanded pitch climbed to 87 deg**. The `927_probe_drag.log` tail
shows where it ended: `t=359.4s alt_asl=228.3 m |v|=0.01 m/s` with
`[ascent] phase=ascent lever=1.00 pitch=87.1`. The hull is sitting still on Kerbin and
the law is still asking for a climb.

The real mechanism: `pathAngle(alt, ask) = atan2(max(1000, 0.03*ask), alt)`
(`src/guidance.cpp:36-39`) is a function of **altitude**, so it is not one-directional in
time. Any time the hull loses altitude — and drag takes it there — the schedule demands a
steeper nose, which trades the little forward speed it had for height it cannot hold. A
pitch-up spiral, ending on the surface. Filed as its own ticket (steer-a-sinking-ship-up),
with the second half of it: the law never *notices* it has landed and keeps flying,
because the `Pad -> Ascent` transition at `:208` already fired and nothing re-checks
`s.grounded` (also filed).

That retracts Finding B's framing. What survives of it: the claim that the *only* route
out of `Ascent` is apoapsis reaching the ask (`:213`) is airtight — `Coast` is entered
nowhere else, `Circ` only from `Coast`, and the top-of-tick `Done` test needs
`peri <= apo >= ask` — but the empirical reason apoapsis stalls is the profile, not a
terminal-speed pin. The consequence for v2 stands, just for a better reason: a schedule
that can go backwards, with no staging and no drag term, does not converge to orbit.

### 2. "Staging is worth ~1.1 km/s" is off by about 7x

Report §2 hand-derived ~5.85 km/s staged and ~4.7 km/s un-staged. The repo has a tool
that books this (`--fleet-report`, the VAB's own staging math), and its answer is:

```
mars_explorer_mk1.json      30  106424.0  12386.4    1.21  solvent on paper
    stage  4: 106424.0 ->  53542.2 kg, dv  3022.6, TWR 1.21-2.41, 5 engines, drops
    stage  3:  25005.3 ->  14428.9 kg, dv  2419.3, TWR 1.03-1.79, 1 engine, drops
    stage  2:   9519.8 ->   4819.1 kg, dv  2995.4, TWR 0.40-0.79, 1 engine, drops
    stage  1:   3529.2 ->   2275.7 kg, dv  3949.1, TWR 0.87-1.34, 1 engine
```

**12.4 km/s booked** against ~4.4 km/s from one un-staged burn. Staging is worth about
**8 km/s** on this hull, not 1.1. (The un-staged half of the estimate was roughly right;
the staged half was the error.) Kerbin needs ~2.2 km/s of circular speed at 150 km plus
losses, so the hull was never short of propellant — the law cannot spend it.

Related correction in the same section: the report said the ship Fails "with three stages
still full of propellant aboard". Stage 4 on this hull lights **5** engines including the
core sustainer (`engine_r2.25h4.5_13` is stage 4), and they draw the core tanks through
fuel links: the log's last live row reads `accel=29.39`, which against
`5 x 253,088 N` (`part.h:331`) implies a mass of ~43 t — so ~63 t of the 69.4 t of
propellant had already been burnt. It Fails near the end of what it could burn, not with
three stages untouched.

Two consequences for the plan, both real:

- The first-cut staging trigger ("seconds to burnout of the ignited stage") fires late on
  this hull, because its ignited engines are fed from the core — the authored drop point
  is at *half* the propellant (106.4 t -> 53.5 t). The trigger is still right in kind, but
  it must be measured as **pool mass divided by the summed rates of every engine that can
  reach that pool**, per resource. The per-engine reading
  (`availableResourceMass(r, p) / propellant_rate[r]` for each engine, min over engines)
  **over-counts**, because `availableResourceMass` answers "what is reachable by *this*
  engine" (`src/vehicle.cpp:5893` comment) and five engines sharing one pool each see the
  whole pool.
- Stage 2's exit criterion ("a materially higher apoapsis") can be satisfied by any change
  that stops the pitch-up spiral, so it does not isolate staging. It needs a pinned stage
  event (a logged press time, or the `stg=` segmentation) beside it.

## Detail corrections

| report said | actually |
|---|---|
| `phase=failed` at t=404 s | first `failed` row is **t=368.3 s**; 403.7 s is a later sample |
| the 16x run sat in pad "for 2824 s" | pad rows run t=12.8 -> **3532.0 s**; 2824 is one row's timestamp |
| `canRail()` at `vehicle.cpp:7097` | the function is **7096-7102**, the expression at **7101** |
| `controller = seatOf(...)` at `:5555-5556` | `:5557` (`:5555` is the `controllerDropped` flag) |
| rebuild calls at `:5554-5558` | `rebuildCompound()` 5556, `clearThrust()` 5558, `buildFuelGroups()` **5559**, `dropDrainWindow()` **5560** |
| `flyAscent` writes four commands at `:394-412` | :394, :395, :408 and **413** — the range cut off the fourth |
| re-reads at `guidance.cpp:349-361` | `accel_max` :358, `has_propellant = v.canBurn()` **:367**; the conic is computed inside `step()` |
| MechJeb at `references/...` relative path | it lives **outside the repo** at `/home/ubuntu/references/ksp-mods/MechJeb2/...` and is not a submodule |
| `ascent_ship` cleared only at `game.cpp:445` / `:1400` | also by `Game::cancelAscent()` at **:479** |
| "`--drag-log` … cannot be joined to the first" | they are the **same run and clock** — `927_probe_drag.log` prints `[ascent]` and `[drag]` rows at identical `t`. The gap is that nothing *plots* them together, and `--record` already carries both series across a staging event |
| "the pitch schedule holds 85-87 deg for the first minute" | it holds that band for ~10 s; 74.6 deg by 31 s, 56.9 deg by 64 s |
| "warp 10 is the top live step" | right, but for a different reason: a hull **in an atmosphere cannot rail** (`canRail()`), and `main.cpp:1159-1180` clamps a start warp >= 11 to 10 when any ship is in air (`events.cpp:649` does the same for WarpUp). The 16x dead zone is only reachable above the air shell |
| Kerbin's honest air top is "~200 km" | unsourced. #816's own analysis treats the 85 km cut as a deliberate anchor ("the top is where satellites start feeling drag", checked by `utils/check_atmo_band.py`) |
| the rails fix is "one line at `Game::armAscent`" | arming cannot prevent a *later* rails transition (`main.cpp:1177`, `events.cpp:649-652`, `game.cpp:1679/2312`, `tick.cpp:254`); the guard belongs in the warp path, and the existing wake paths are 4-6 lines each |
| "no cached state to invalidate" on a stage drop | `AscentLaw` caches `circ_node_r_`, `launch_h_`, `pitch_`; a drop during `Circ` sizes the burn from the new `accel_max` against the old node |
| `releaseAscent()` demonstrates the stage interlock | it demonstrates "do not leave the lever open" — it has no re-open half |
| `flight/610` is the ascent case | **five** cases consume `[ascent]`: `flight/610`, `flight/615` (also untagged: `tanker`, 7 parts), `flight/894`, `flight/900`, `ui/612-guidance-window-ascent` |
| the Stage 0 Kerbin command produced `927_probe_kerbin.log` | that file has **zero** `[drag]` rows; the drag probe was a separate 90 s run whose command the report never printed |
| the Moho baseline "costs 60 s to re-run" | ~120 s of wall on this machine to get past the verdict |
| #421's labels include `Kind/Enhancement` | it carries `Priority/Medium, Size/M, Topic/Orbit` only (no `Kind/*` — it predates the sweep) |

## What the review confirmed, so it does not have to be re-checked

The v1 footer quote (`guidance.h:209-214`); the `Ascent -> Coast` quote (`guidance.cpp:213`);
no stage call anywhere in `guidance.cpp`; `flyAscent` called from exactly one site
(`tick.cpp:213`); ignition gated inline at five places and nowhere else;
`extractSubtreeAsShip` keeps the flying stack on the original `Vehicle *` (`:5550-5554`),
so `ascent_ship` needs no repointing; `Game::stage()` never writes `Game::ship`;
`StageRow` has no burn-time field and `computeStaging` takes a `BuildShip` with tanks
full; `mars_explorer_mk1` is 30 parts / 6 decouplers / 5+1+1 engines with the capsule as
`parts[0]`, ~69.4 t propellant on ~37 t dry; `stager.json` and `racer.json` as described
(`racer` carries no `stage` tag); the `[ascent]` column list; `tests/test_guidance.cpp`
being the only test that touches the law and having no way to model a mass step; the
Descent/Transfer entries being greyed out; `--ascent` reachable from the UI; the three
`[drag]` rows and the `accel` 12.11 -> 29.39 range; the Moho `done` result and its
periapsis.

## Net effect on the staged plan

Stages 0, 1, 2 and 3 stand, with three changes:

1. Stage 2's trigger must be computed per fuel pool with the rates of all engines that
   reach it summed, not per engine.
2. Stage 2 gets a second exit criterion that pins the stage event itself, so its gain is
   attributable.
3. Stage 1 must handle `e2e/run.py` explicitly: `ASCENT_RE` (`e2e/run.py:151-163`) is
   positional and ends at `why="...`; a column inserted anywhere but the end matches
   nothing, an empty list makes every `all(...)` check vacuously true, and run.py
   documents that exact trap at `:703-710`. Stage 1 therefore touches `ASCENT_RE`,
   `parse_ascent` (:1159-1173), the `--self-test` format pins (:2927-2953) and the five
   consuming cases. It also gains `alt=` and `v=` columns — the report had to reconstruct
   the ship's own altitude and speed from a second log, which is the same gap #675 is
   about.
