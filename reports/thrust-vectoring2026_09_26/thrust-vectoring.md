# Thrust vectoring (engine gimballing) — design & staged plan

Date: 2026-09-26
Status: proposal (not implemented). What's-there / what's-missing / staged plan.

## TL;DR

Today the thrust vector of every engine is **rigidly locked to that part's
authored +Z axis** (`vehicle.cpp:1511`, `ft = partAxis(p, 2) * armedThrust`).
The only way to "steer" a ship is the reaction-wheel torque loop
(`applyRotationForce`), which needs the part to carry a `torque` value and the
ship to be powered, and which is pure `ApplyTorque` — it never touches thrust
direction. There is **no gimbal, no thrust-vector parameter, no per-part
mutable control state, and no autopilot that uses thrust** anywhere in the
codebase (grep for `gimbal|tvc` returns zero hits; every `steer|deflect` hit
is the aero control-surface path — lift-based, zero in vacuum — or the
unrelated EVA walk-steer).

Thrust vectoring (TVC) is the natural complement: rotate the thrust vector a
small angle about two in-plane axes, applied at the engine, so the offset
force produces the attitude torque. It is the one authority that works in
vacuum without a reaction wheel and without EC, and it is what makes a single
engine + no-wheel ship controllable. The good news: the force-application
machinery (`ApplyForce` at a part-offset lever + the spurious-torque
cancellation) is already exactly the shape TVC needs — we only change the
**direction** of `ft`, not the plumbing.

---

## 1. What's there

### 1a. How thrust is applied (the line TVC replaces)

`Vehicle::applyThrustForce()` — `src/vehicle.cpp:1502-1524`, called before
**every** physics substep (see §1d):

```cpp
// src/vehicle.cpp:1502-1523 (abridged)
void Vehicle::applyThrustForce() {
    const glm::dvec3 com = comPos();
    glm::dvec3 ftotal(0.0);
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->armedThrust == 0.0f) { continue; }
        /* Along the engine's own +Z, applied AT the engine: an off-axis or
           tilted engine torques the ship directly, ... */
        const glm::dvec3 ft = partAxis(p, 2) * (double)p->armedThrust; // :1511
        ApplyForce(hull, partPos(p) - com, ft);                        // :1512
        ftotal += ft;
    }
    lastThrustForce = ftotal;
    ...
    ApplyTorque(hull, -glm::cross(dcom, ftotal)); // :1522 spurious-torque cancel
}
```

Key facts:

- **Axis:** `partAxis(p, 2)` = the world direction of the part's authored
  stack axis (`partAxis` at `vehicle.cpp:944`, `partRot` at `:938-942`). The
  thrust direction is **hardcoded to the part axis**; there is no deviation
  term.
- **Where:** `ApplyForce` = Bullet `applyForce(force, rel_pos)` on the single
  ship rigid body `hull`, lever arm `partPos(p) - com`
  (`src/physics.cpp:380-385`). A force applied off-COM produces a real torque
  `rel_pos × F` — that is exactly the mechanism TVC exploits.
- **Spurious-torque cancel** (`:1522`): the thrust lever is referenced to the
  hull origin, which lags the true COM during a burn, so
  `−(comOffset × F)` is subtracted. **Any TVC change must keep this
  cancellation honest** — e2e `48-com-torque.txt` pins it (`|ω| < 1e-3`
  through a burn + coast).
- **Single rigid body:** the whole ship is one `btRigidBody`
  (`Vehicle::hull`, `vehicle.h:110-160`); parts have no bodies
  (`body.h:45-52`). So TVC force goes to `hull` with a part-offset lever,
  exactly like today's thrust.

### 1b. How attitude is controlled today (what TVC would complement)

`Vehicle::applyRotationForce(double h)` — `src/vehicle.cpp:1807-1845`:

```cpp
// src/vehicle.cpp:1807-1845 (abridged)
void Vehicle::applyRotationForce(double h) {
    if(firstWheel() == nullptr) { return; }   // needs a torque>0 part
    if(!powered_) { return; }                 // power gate (EC)
    if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
        Part *rw0 = firstWheel();
        const glm::dvec3 pitchAxis = -partAxis(rw0, 0); // right (W/S)
        const glm::dvec3 yawAxis   = -partAxis(rw0, 1); // up (A/D)
        const glm::dvec3 rollAxis  =  partAxis(rw0, 2); // nose (Q/E)
        const glm::dvec3 worldAxis = (double)stick[0] * rollAxis
            + (double)stick[1] * pitchAxis + (double)stick[2] * yawAxis;
        ApplyTorque(hull, maxTorque() * worldAxis);     // :1832 — pure torque
    }
    if(slew == SlewKillRot) { killRotStep(h); }
    else if(slew != SlewNone) { slewToward(slewTargetDir(), h); }
}
```

- The wheels are **virtual**: any part with `def->torque > 0` is a wheel
  (`part.h:155`); `maxTorque()` sums all (`vehicle.cpp:2621-2627`). The
  `firstWheel()`'s axes define the stick frame.
- `stick[3]`: x=roll(Q/E), y=pitch(W/S), z=yaw(A/D); set ±1 in `Vehicle::Command`
  (`vehicle.cpp:2493-2499`); cleared each tick by `clearRotCmd` (`vehicle.cpp:2111`).
- **The autopilot slews run in this same function** — `slewToward`/`killRotStep`
  (`vehicle.cpp:2653-2710`) — and drive attitude **exclusively through
  reaction-wheel torque**. They never touch thrust direction.
- **Two other attitude/translation authorities exist:** RCS translation
  (`applyRcsForce`, `vehicle.cpp:1870`, gated on hydrazine) and aero control
  surfaces (deflection-driven, in `applyAeroForce`, `vehicle.cpp:1720-1790`,
  **zero authority in vacuum**).

### 1c. Part model — no gimbal field, no mutable state

- `PartDef` (`src/shipdef.h:231-477`) has `torque`, `fuel_rate`,
  `exhaust_velocity`, `rcs_thrust`, `control_area/control_axis/cl_control/
  max_deflection`, etc. — **no field for gimbal angle, gimbal rate, or gimbal
  authority.** The closest "steering" params are the control-surface fields,
  which are aero-only.
- `Part` (`src/part.h:57-224`) — the **only** per-part transient dynamic is
  `armedThrust` (float, N for the tick, `part.h:84`). There is **no mutable
  runtime state for a gimbal angle.** The authored pose `localPos`/`localRot`
  is fixed at attach time (`part.h:132-133`) and only rebased when the ship's
  topology changes (docking merge, stage separation) — never mutated in flight.
- `res/data/parts.json` (62 parts): engines (`engine`, `orbital_engine`, `jet`)
  carry no steering parameters at all.

### 1d. The tick — where forces are applied

`src/tick.cpp`, fixed logic tick `g.dt = 1/50 s` (`game.h:397`); physics path
(`tick.cpp:265-322`): per substep, per non-railed ship —

```
s->processGravity();        // per-part point-mass gravity + Coriolis/centrifugal
s->applyAeroForce(h);       // drag + lift + control surfaces
s->powerTick(h);            // EC balance; sets the powered_ gate
s->applyControlForces(h);   // = applyThrustForce() + applyRotationForce(h) + applyRcsForce(h)
```
then `physics_tick(h)` (one Bullet `stepSimulation`, `physics.cpp:197`).
Bullet clears accumulated forces/torques on each step, so **everything is
re-applied per substep** — a TVC force slots into `applyThrustForce` with no
new tick plumbing.

### 1e. Rendering — no per-part animation state

- Parts: `Vehicle::Draw` (`vehicle.cpp:2430-2475`), pose from
  `partPoseRelCom(p, pp, pr)` (`vehicle.cpp:920-930`).
- Plume: `render.cpp:436-497` — a quad drawn at `partPoseRelCom(p)`
  (`render.cpp:466-472`) scaled by the part's radius/height, for each
  thruster with `armedThrust > 0`. **It uses the un-gimballed part pose.**
- Shroud: drawn at the part's own pose (`vehicle.cpp:2465-2473`).
- A gimballed engine needs (a) a new mutable per-part angle, (b) a draw pass
  that rotates the engine mesh + shroud about its attach point, and (c) the
  plume pose updated to follow the gimballed axis.

### 1f. Existing tests / e2e that pin the current model

- `e2e/cases/48-com-torque.txt` — pins the spurious-torque cancellation
  (`|ω| < 1e-3` through burn + coast). **TVC must not regress this.**
- `e2e/cases/22-attitude-physics.txt` — pins the rigid-body wheel-torque
  model (inertia, no arcade gimbal).
- `tests/test_attitude.cpp` / `test_slew3d.cpp` — pin the `slewToward` /
  `killRotStep` braking-curve law + authority bound.
- `tests/test_thrust.cpp` — substep thrust delivery, fuel-flow rate.

---

## 2. What's missing

| Concern | Status |
|---|---|
| Gimbal / TVC parameter on `PartDef` | **Absent** — no angle/rate/authority field (`shipdef.h:231-477`). |
| Per-part mutable gimbal state | **Absent** — only `armedThrust` (`part.h:84`). |
| Thrust direction deviation | **Absent** — hardcoded to `partAxis(p,2)` (`vehicle.cpp:1511`). |
| TVC in the control loop | **Absent** — `applyControlForces` = thrust + wheel torque + RCS only. |
| Autopilot using thrust | **Absent** — autopilot is wheel-torque-only (`vehicle.cpp:2653-2710`). |
| Gimbal rendering (engine + shroud + plume) | **Absent** — all use the authored pose. |
| Per-engine gimbal limits / HUD | **Absent** — no key, no part-window field, no catalog value. |
| Tests for thrust-direction change | **Absent** — only the spurious-torque + wheel-torque pins above. |

The one thing that is already right: **the force-application path.** TVC does
not need a new `ApplyForce`, a new lever arm, or a new tick hook — it changes
the **direction** of `ft` in `applyThrustForce`, and Bullet's off-COM
`applyForce` produces the torque for free.

---

## 3. The model

A gimbal rotates the thrust vector by a small angle `θ` about two orthogonal
in-plane axes (in KSP, "pitch" and "yaw" of the nozzle), giving a force

```
F = R(θ) · (T · ẑ_part)
```

where `R(θ)` is a 2-DOF rotation in the plane perpendicular to the part axis,
`T` is the armed thrust magnitude, and `ẑ_part` is today's `partAxis(p,2)`.
Applied at `partPos(p)`, the off-COM lever `rel_pos × F` produces the
attitude torque. Because the deviation is small (≤ ~10°), the axial component
`T·cos θ ≈ T` — thrust loss is second-order and can be ignored in v1 (or
recovered by dividing by `cos θ` if we want exact thrust).

Two design axes to decide:

- **Authority model.** In vacuum with no wheel, TVC is the *only* attitude
  authority. Combined with wheels, it should **add** authority (KSP-style:
  all actuators sum), not replace. The natural split: TVC contributes
  `rel_pos × F` torque, wheels contribute `maxTorque()` — both into the same
  rigid body, both rate-limited.
- **Power.** Wheels are gated on `powered_` (EC). Should TVC actuators draw
  EC too? Recommendation: **no** in v1 — a mechanical gimbal is the classic
  "works when the power is dead" fallback; gate it only on the engine being
  armed. Add an optional `power_draw` later if we want the knob.

---

## 4. Staged plan

Each stage is independently shippable and testable; later stages assume
earlier ones.

### Stage 0 — pure math core (no GL, no Bullet)

New header `src/gimbal.h` (pattern: `src/drag.h`, `src/orbit.h`):

- `gimbalDir(base, pitch, yaw)` — rotate a part axis by a 2-DOF gimbal angle,
  returning the world thrust direction. Pure glm.
- `gimbalStep(current[2], target[2], rate, h)` — rate-limited move of the
  current angle toward the target (the actuator slew).
- `gimbalTorque(rel_pos, T, dir)` — `rel_pos × (T·dir)` (the authority term,
  exposed so the attitude law can budget it).

Pin with `tests/test_gimbal.cpp` (GL/Bullet-free, like `test_drag.cpp`):
`gimbalDir` with zero angle = base; small-angle torque ≈ `rel_pos × T·dir`;
rate-limit monotonic and bounded; symmetry.

**Exit:** `make test` green; the math is trusted and decoupled.

### Stage 1 — minimal runtime (thrust direction only)

- `PartDef` (`shipdef.h`): add `double gimbal_max;` (deg, 0 = no TVC) and
  `double gimbal_rate;` (deg/s, actuator slew). Load in `shipdef.cpp`
  (pattern: `max_deflection` at `shipdef.cpp:341-343`).
- `Part` (`part.h`): add `double gimbal[2] = {0,0};` (current angle, deg) —
  the mutable state, next to `armedThrust`.
- Control input: map the existing `stick[]` to a gimbal **target** angle
  `stick * gimbal_max`. (v1: stick drives TVC; this is the same input that
  already drives wheels, so a ship with both gets both.)
- `applyThrustForce` (`vehicle.cpp:1511`): replace
  `ft = partAxis(p,2) * armedThrust` with
  `ft = gimbalDir(partAxis(p,2), p->gimbal[0], p->gimbal[1]) * armedThrust`,
  advancing `p->gimbal` by `gimbalStep(..., gimbal_rate, h)` first.
- **Keep the spurious-torque cancellation** (`:1522`) — it operates on
  `ftotal`, which is unchanged in form.
- Catalog: give the `engine` / `orbital_engine` parts a `gimbal_max` /
  `gimbal_rate` in `res/data/parts.json` (e.g. 5° / 90°/s).

**Exit:** a ship with an engine and **no reaction wheel** can pitch/yaw in
vacuum by holding W/S / A/D while thrusting. `make test` + `make e2e` green.
New e2e `95-tvc-vacuum.txt`: a wheel-less ship, `T` + `S`, assert attitude
changes (`--att-log` nose direction) while `|ω|` stays bounded; assert
`48-com-torque` still passes.

### Stage 2 — rendering

- `Vehicle::Draw` (shroud block `vehicle.cpp:2465-2473`): when a part has a nonzero
  gimbal, rotate the engine mesh **and** its shroud about the attach node by
  the gimbal angle (build the local rotation from `gimbalDir`'s basis).
- `render.cpp:466-472`: draw the plume along the **gimballed** axis
  (replace `partPoseRelCom`'s axis with `gimbalDir(...)`).
- **Exit:** the nozzle visibly swings and the plume follows; `55-engine-shroud`
  still passes (shroud attaches on the exhaust face).

### Stage 3 — autopilot integration

- `slewToward` / `killRotStep` (`vehicle.cpp:2653-2710`): add a TVC authority
  term. The attitude law currently budgets `α = maxTorque()/I_eff`; extend it
  to include the TVC torque `gimbalTorque(...)` so the braking curve and
  authority cap account for both. Drive the gimbal target from the slew
  error (prograde/retrograde/radial), not just the stick.
- **Exit:** the Autopilot window (`gameui.cpp:1652-1686`) Prograde/Retrograde
  slews a wheel-less ship in vacuum. `test_attitude` / `test_slew3d` extended
  with a TVC-authority case.

### Stage 4 — QoL / data

- Per-engine `gimbal_max` / `gimbal_rate` shown + editable in the part window
  (`drawPartWindows`, `gameui.cpp:1718`) alongside the existing `Torque` /
  `Thrust` blocks, and in the VAB part list.
- HUD: current gimbal angle + "TVC armed" indicator.
- Optional `power_draw` gate on the actuator (the knob we deferred in §3).
- Per-engine throttle / individual gimbal (only if a design needs it; the
  current single-throttle model is fine).
- **Exit:** part window + VAB show per-engine `gimbal_max`/`gimbal_rate`; HUD
  shows the live angle; `make test` + `make e2e` full battery green.

---

## 5. Open questions

1. **Combined or exclusive authority?** Recommendation: TVC and wheels
   **sum** (both act on the same rigid body). Confirm we don't want a
   "TVC-only" mode that disables wheels.
2. **Thrust loss at full deflection.** `cos 5° ≈ 0.996` — ignore in v1. If we
   ever allow > 15°, recover the axial component.
3. **Gimbal sign convention.** KSP's pitch/yaw signs vs. this codebase's
   stick frame (`vehicle.cpp:1823-1825`). Pin in `test_gimbal` to avoid a
   sign flip surprise in Stage 1.
4. **Should a gimbaled engine with `armedThrust == 0` still slew back to
   center?** Recommendation: yes — the actuator recenters when the engine is
   off (cheap, and it makes the ship settle).

## Verification plan (per stage)

- Stage 0: `make test` — `test_gimbal` pins.
- Stage 1: `make test` + `make e2e` (incl. `48-com-torque`, `22-attitude-physics`);
  new `95-tvc-vacuum.txt`.
- Stage 2: `make e2e` incl. `55-engine-shroud`; visual check of the swing.
- Stage 3: `test_attitude` / `test_slew3d` extended; Autopilot window on a
  wheel-less ship.
- Stage 4: `make test` + `make e2e` full battery; part-window / HUD check.

---

*Snapshot + proposal, not implemented. Scope: `src/gimbal.h` (new),
`src/shipdef.{h,cpp}`, `src/part.h`, `src/vehicle.{h,cpp}`
(`applyThrustForce`, `applyRotationForce`, `slewToward`), `src/render.cpp`,
`src/gameui.cpp`, `res/data/parts.json`, `tests/test_gimbal.cpp` (new),
`e2e/cases/95-tvc-vacuum.txt` (new). No physics-world, save-format, or
network changes.*
