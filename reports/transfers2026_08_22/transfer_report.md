# Orbital Transfers Without Eyeballing — Investigation

**Scope.** What a transfer looks like in OSP today, what is missing relative to the
"KSP-lite" goal (letter.txt), and a prioritised list of **simple** improvements.
Every code claim is verified against `src/` (file:line below, tree at `cb4cf08`);
every orbital number is closed-form and reproduced by `verify_transfers.py`
(same constants the game uses: G = 6.674e-11 at L487/L1031, masses from
`ksp_system.json` / `gen_systems.py`).
**Bottom line first:** the sim already contains nearly all the raw material for
mechanical transfers — the conic fit is computed every frame (L2594-2676),
auto-pointing to prograde/retrograde already works (B/N, L2372-2373), and the
SOI hand-off is correct and tested. What is missing is a small set of
**HUD/planning** features: a transfer calculator (Δv + time-of-flight + when to
burn), a functioning autopilot (the buttons exist and do nothing), a Δv budget
display that matches the engine (it is ~5× off), and a target-body concept.
None of these touch the physics or the frame tree — they are all pure math +
ImGui, so they are safe to build and easy to verify headlessly.

---

## 1. TL;DR

| Finding | Evidence |
|---|---|
| Transfer loop is 100% manual: point with u/o/y/h/l/j, burn with `i`, watch ApA/PeA creep, warp with `,`/`.` | controls help L2986-3006; thrust L2360; warp L2186-2199 |
| Auto-pointing to prograde/retrograde **already exists** (B/N) but is undocumented | L2372-2373 → `RotateToward` L1156-1165 |
| Autopilot window exists with 6 attitude buttons; **all six are no-ops** (return value discarded) | L3008-3016; hidden behind a "DUMB-ASS" checkbox L2869 |
| The per-frame conic fit (a, e, r, v, inc, Raan, argPe, true/Ecc/Mean anomaly, times) already exists and is fed to the HUD — free raw material for any planner | L2594-2676 |
| 3D navball markers (pro/retro/radial±/normal±) already drawn as always-visible billboards | L2773-2792 |
| **Δv display is inconsistent with the thrust model by ~5×** (VESSEL window 1,691 m/s vs engine budget ≈ 8,934 m/s) — the player cannot plan burns against it | `getDeltaV` L969-973 (ve=10123, fuel/2) vs `GetMaxThrust` L1091-1094 (ve=40492, "raised 4x") |
| No target-body concept; `G` cycles **camera** targets only | `focusTargets` L2077-2083, G handler L2239-2248 |
| Orbital map draws only the current orbit at a hand-set scale (default 6000 m/px) | L3045-3100 |
| Closed-form transfer numbers for both shipped systems (Kerbal + Eerbon) are all reachable in < 15 lines of math — verified in `verify_transfers.py` | §4 |

---

## 2. What a transfer looks like today

The sim is a **patched-conic** game: the ship always belongs to one frame,
feels **only the parent body's gravity** (`applyGravity` L1029-1055; no
three-body pull), plus Coriolis+centrifugal while in a rotating frame
(`GetFictitiousAccel`, frame.h L109-112). Crossing a SOI boundary (±10 km
hysteresis) auto-switches frames with the stasis-velocity correction, tested in
`tests/test_spawn.cpp` (L2420-2458, `moveToFrame` L1214-1257). Bodies move
analytically (`UpdateOrbitRails`, frame.cpp L48-80), so a target's future
position is known in closed form.

So "Kerbin → Mun" today goes like this:

1. Get to a 100 km orbit (launch + orbit insertion — already eyeballed).
2. Point the nose prograde (u/o/y/h… or the **undocumented** B key).
3. Hold `i`, throttle with the **undocumented** R/F, and **watch the ORBITAL
   window** until ApA ≈ 12,000 km (the Mun's orbital radius — a fact the player
   must know or guess; it is never shown anywhere as "target distance").
4. Release, warp with `.` until the ship reaches the Mun's SOI (2,430 km — also
   nowhere displayed), cross it, frame switches.
5. Point retrograde (N), burn until PeA is a comfortable altitude, repeat.

Every step that could be *told* to the player ("burn 842 m/s prograde now,
capture needs 362 m/s, 7.4 h to go") is instead *guessed* from ApA numbers the
player must mentally convert between radii and the target's orbit. That is the
eyeballing.

### 2.1 What already works (do not rebuild)

- Per-frame conic fit from the **inertial** state (`orbit_pos/orbit_vel`
  transform L2619-2630, element solve L2632-2676) — incl. true anomaly,
  eccentric anomaly, mean anomaly, time to peri/apoapsis (`PeT`/`ApT`).
- 3D attitude reference markers always on screen (L2773-2792).
- `B`/`N` → `RotateToward(±vel)` (L2372-2373, L1156-1165): the autopilot's
  core primitive, already there.
- `x` → `KillRot` (L2361); throttle R/F (L2375-2376); time-warp `,`/`.`
  (L2186-2199, ×10 steps).
- `computeOrbitElements` (L1716-1737) + `--orbit-log` (L2483-2512): a
  headless measurement channel — the same channel the stability report used.
- SOI hand-off + frame-switch invariants, with tests.

---

## 3. Gap analysis vs KSP-lite

| KSP-lite capability | OSP status | Effort to close |
|---|---|---|
| Attitude reference (navball) | 3D billboards only; no 2D ball | 2D ball = later (see §6) |
| Auto-attitude (SAS) | B/N one-shot torque; no hold | **small** — wire the existing dead buttons |
| Target body + ApS/PeS | none (G = camera target only) | **small-medium** — frame tree already exposes target `root_pos`/`root_vel` |
| Maneuver planning (nodes) | none | **medium** — closed-form for coplanar Hohmann (see §5) |
| Δv budget per burn | VESSEL window Δv, **~5× wrong** (see §4.3) | **trivial** — make it consistent |
| Time to burn / T-CA countdown | ApT/PeT shown as raw seconds | **trivial** — format + "burn now" hint |
| Map view of current + target + transfer orbit | current orbit only, manual scale | **small** — same window, add 1-2 polylines |
| Transfer-window awareness (interplanetary) | none | **small** — bodies move analytically; synodic math is closed-form |

---

## 4. The numbers (all verified by `verify_transfers.py`)

### 4.1 Kerbin 100 km LKO → Mun (the canonical first transfer)

| Quantity | Value |
|---|---|
| Δv₁ (prograde, at periapsis) | **841.6 m/s** (2,246 → 3,088 m/s) |
| Δv₂ (retrograde, capture) | **362.4 m/s** (180 → 542 m/s) |
| Time of flight | **7.43 h** (26,749 s) |
| Cross-check | KSP's own TMI from 100 km is ≈ 840 m/s — the game's constants reproduce KSP's number, so the calculator can be validated against a known answer |

### 4.2 Kerbin → Duna (interplanetary)

| Quantity | Value |
|---|---|
| Heliocentric Hohmann Δv (departure) | 918.5 m/s at Kerbin's distance |
| Combined departure burn from 100 km LKO (v∞=918.5, v_esc(LKO)=3,176.6) | **1,060.5 m/s** |
| Arrival v∞ at Duna | 826.2 m/s |
| Duna 100 km orbit capture (v_circ = 847 m/s) | 629.0 m/s |
| Time of flight | **75.5 days** |
| Transfer window (synodic period) | every **227.4 days** |
| Duna lead angle at departure | **44.4°** |

### 4.3 The Δv budget the player sees today does not match the engine

Ship: 0.5 kg capsule + 1.0 kg reaction wheel + 3.0 kg engine (L1971-1977),
1.0 kg H₂ + 1.0 kg LOX per engine (L933-934) → 6.5 kg wet / 4.5 kg dry.

- **Engine budget** (thrust model L1091-1110: 404.9 N applied per tick while
  consuming 2 × 0.01/60 kg per tick at 50 ticks/s, L2110): Tsiolkovsky with
  ve_eff = impulse/mass-lost ≈ 24,295 m/s → **≈ 8,934 m/s**.
- **Displayed** (`getDeltaV` L969-973: ve = 10,123 m/s, *fuel/2*, i.e. only one
  propellant counted): **1,691 m/s** — **~5.3× lower** than what the engine
  can actually deliver.

Consequences: (a) a player who trusts the VESSEL window thinks a Duna trip
(~2 km/s) is unaffordable when it is comfortably affordable; (b) the two
formulas disagree on both the exhaust velocity (10,123 vs 40,492 — the latter
has a comment "raised 4x for easier orbit insertion", the former was never
updated) and the propellant mass (both propellants are consumed, L1100-1101).
This is the single cheapest trust fix in this report.

Also note: the ORBITAL window's ApA/PeA are **radii** (L2648-2649, L2914-2916),
not altitudes; KSP convention is signed altitude. The SURFACE window does show
ASL, but the transfer-relevant numbers (ApA/PeA) do not.

---

## 5. Recommendations, in priority order

All items are HUD/math only. Suggested order = the order in which they stack
on each other.

### R1 — Make the autopilot actually work *(~half a day; unlocks everything else)*

Wire the six dead buttons (L3008-3016) to `RotateToward` (L1156) — the
Prograde/Retrograde ones are literally one line each
(`if (ImGui::Button("Prograde")) ship->RotateToward(vel);`, with a sticky
toggle if the "SAS hold" behaviour is wanted). Add the undocumented keys
B/N/R/F to the Controls help (L2986-3006). Optionally add a "SAS" toggle that
re-applies `KillRot` (L2361) every tick so attitude is held — that is the
whole of KSP's reaction-wheel assist.

**Why first:** every other recommendation ends in "burn prograde 842 m/s"; the
player still has to hand-point and hand-fly that burn today.

### R2 — Fix the Δv display (trivial)

Make `getDeltaV` (L969) use the same exhaust velocity and the same propellant
mass as the thrust model (L1091-1110). One constant + one line. Show the
remaining Δv next to every planned burn (R4/R5) as "cost vs budget".
(If Denis prefers the lower number for game balance, the fix is the same —
change `GetMaxThrust`/fuel flow instead — but the two must agree.)

### R3 — Transfer calculator ("TRANSFER" MFD window) *(~1 day)*

A new ImGui window (same pattern as ORBITAL, L2911):

- **Target body** dropdown — reuse `focusTargets` (L2077) or walk the frame
  tree; for the current central body this covers moon transfers (Kerbin→Mun),
  for the star it covers interplanetary (Kerbin→Duna).
- **Compute and show** (closed-form, ~30 lines, no new dependencies):
  - Δv₁ to raise apoapsis to the target's orbital radius (Hohmann), and Δv₂
    to circularise at the target — the §4 numbers;
  - time of flight;
  - **burn direction**: prograde/retrograde (coplanar case) and the plane
    change needed (target orbit inclination is already in the frame tree —
    `orb_incl`, gen_systems.py);
  - **feasibility**: Δv₁+Δv₂ vs remaining Δv (R2);
  - for interplanetary targets: next window countdown + lead angle (§4.2 —
    the bodies' `orb_ang_speed` values are already loaded, and
    `UpdateOrbitRails` (frame.cpp L48-80) gives the target's future position
    analytically, so "where will the target be at ToF" is exact).
- All of this is a **pure function** of (current state, current body, target
  body) — unit-test it in `tests/` exactly like `test_frames.cpp` does,
  asserting against the `verify_transfers.py` numbers (841.6 / 362.4 / 7.43 h
  etc.). The test target already builds without rendering/Bullet (Makefile
  L73-78), so this plugs straight into `make test`.

### R4 — Show the result: predicted orbit on the Orbital map *(~half a day)*

The "Orbital map" window (L3045-3100) already draws the current ellipse as an
ImGui polyline. Add:

- the **transfer ellipse** (same code, second polyline, different colour);
- the **target body's orbit** (its radius is a constant — a circle);
- periapsis/apoapsis markers + the burn point;
- auto-scale: set `div` from ApA of the widest orbit instead of the manual
  slider (the slider can stay as an override).

This converts "watch ApA creep" into "the predicted line reaches the target
circle" — the eyeball becomes a confirmation, not a search.

### R5 — "Burn to plan" (maneuver-node-lite) *(~1 day)*

A state machine over the existing primitives: *target Δv* (from R3, or a
slider) + *direction* (from R1) → `RotateToward` hold + `ApplyThrust` +
integrate Δv spent (thrust × time / mass, or simply fuel consumed × ve_eff) →
auto-cut when the budget is spent. No maneuver nodes, no predicted-state
bookkeeping — just "fly the burn for me". KSP's node is the planning part
(R3/R4); this is the execution part. Verify headlessly: `--orbit-log`
already prints a/e/periapsis every interval, so a scripted prograde burn from
the `rot-orbit` scenario must show apoapsis converging on the planned value —
same method as the stability report's driver loop.

### R6 — Target-relative readout (ApS/PeS) *(later, ~1-2 days)*

With a real target (R3's dropdown doubles as the target): show distance,
closing rate, and the ship's peri/apoapsis **around the target** — the frame
tree already carries the target's inertial `root_pos`/`root_vel`
(frame.h L47-48), so the target-relative state is two subtractions + a conic
fit. This is KSP's "target body" feature and the natural next step after R1-R5.

### Explicitly out of scope for this pass

- Full maneuver-node UI (multiple stacked nodes, plane-change nodes) — R3/R4/R5
  cover the 90% case (coplanar Hohmann) with a fraction of the UI work.
- A 2D navball widget — the 3D billboards (L2773-2792) already provide the
  attitude reference; revisit if Denis finds them hard to read.
- Lambert-solver / multi-impulse / bi-elliptic transfers, N-body gravity,
  atmospheric drag — none exist in the sim today and none are needed for the
  "KSP-lite" goal; the stability report (§refs) is the place to take
  integrator-level concerns.

---

## 6. Verification plan (per the house workflow)

1. **Headless math tests.** `tests/test_transfers.cpp` (new) asserting the
   R3 pure function against `verify_transfers.py` output (841.6 m/s ± 0.1,
   362.4 m/s ± 0.1, 7.43 h ± 1 s, Duna window 227.4 d ± 0.1, lead 44.4° ± 0.1).
   Add to the `test:` target (Makefile L73). `make test` stays green.
2. **Headless burn test** for R5: `./osp --body Kerbin --scenario rot-orbit
   --time-accel 100 --orbit-log --orbit-interval 1` with the burn scripted
   (or driven via the new key), assert apoapsis reaches the planned radius
   within tolerance and energy drift stays inside the stability-report bounds
   (≤ ~2 %/day, rotating-frame time is short here).
3. **Denis in the game** after each of R1→R5: buttons point the ship, Δv
   number is sane, TRANSFER window matches the §4 table, the map shows the
   transfer ellipse, the auto-burn lands the orbit.

### Risks / things to keep in mind

- **Frame conventions**: the calculator must use the same inertial state the
  HUD does (L2619-2630: stored velocity + stasis, mapped via
  `GetOrientRelTo`). Using the raw stored (frame-coordinate) velocity in a
  rotating frame mis-solves the conic — the 2×-stasis bug class that
  `tests/test_spawn.cpp` pins.
- **Time acceleration**: coast phases want high `ta`; per the stability
  report, keep `h ≤ 0.1 s` (i.e. `ta ≤ 10⁴`) while in rotating frames.
  Interplanetary coasts live in the star's inertial frame, where the drift is
  ~5 orders smaller — so R3's long ToFs are fine.
- R2 changes a visible number (Δv goes **up** ~5×). That is a gameplay-balance
  knob — decide which side to move before shipping.
- `GetMaxFuelRate` has a `fixme T == ??` (L1089) — the fuel-flow units are
  unverified; R2's consistency fix will make this visible.

---

## 7. References

- `letter.txt` — the "Kerbal space program lite" goal.
- `verify_transfers.py` (this directory) — all numbers in §4.
- `references/Orbital Mechanics for Engineering Students.pdf` — Hohmann
  transfer theory (the "Orbital Maneuvers" chapter).
- `reports/stability2026_08_20/stability_report.md` — time-accel bounds for
  coast phases.
- `tests/test_spawn.cpp`, `tests/test_frames.cpp` — frame-switch invariants
  and the test style to copy for `test_transfers.cpp`.
- KSP wiki (Kerbin→Mun): TMI ≈ 840 m/s from 100 km — the cross-check that the
  game's constants reproduce the reference game's answer.
