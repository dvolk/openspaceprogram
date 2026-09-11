# Aerodynamics — lift, accurate drag, and centre of pressure

Date: 2026-09-11
Status: **design / scope — NOT implemented.** This is the plan that follows
`reports/atmospheric-drag2026_09_11` (v1 drag, implemented). It scopes the
next step: angle-of-attack-dependent drag, aerodynamic **lift**, and an
off-COM **centre of pressure** so forces produce a pitching moment.

## TL;DR

v1 drag is a *central* quadratic force at the COM: it decelerates the ship but
never turns it, and a wing does nothing. This adds the three things a
KSP-like game needs for believable atmospheric flight, in one coherent model:

```
flow frame   v_b = R_shipᵀ·v_rel ;  α (pitch AoA), β (sideslip) ;  q = ½·ρ·v²
lift         L = q·S_L·Cl(α)  ⊥ the flow, in the vertical plane
drag         D = q·S_D·Cd(α)  opposite the flow,  Cd(α) = Cd₀ + k·sin²α + induced
moment       τ = r_cop × F    (force at the centre of pressure, not the COM)
apply        ApplyCentralForce(hull, F)  +  ApplyTorque(hull, r_cop × F)
```

It is **per-part** (KSP-style): each part contributes a drag area, a baseline
`Cd`, a lift area + lift curve, and a centre-of-pressure point. The ship's
drag area and CoP *compose* as parts are added and dropped (staging shifts the
CoP — the reason a rocket stays weathervane-stable until its fins burn off).
The pure-math law stays GL/Bullet-free in `src/drag.h` (extended, or a new
`src/aero.h`) so `tests/` can pin it, exactly like v1. Cost is O(parts) per
substep — the same order as today's `dragArea()`.

The model to copy is **Orbiter's airfoil surface** (`Src/Orbiter/Vessel.cpp:
4099–4225`): force = `q·S·(CL·lift_dir + CD·drag_dir)` applied at an authored
point, torque = `r × F`, with induced drag `Cl²/(π·AR·e)`. Pioneer
(`src/Ship.cpp:478–555`) is a cautionary lower bound (per-axis drag + a faked
stability torque, no CoP) — fine for feel, wrong for physics.

---

## 1. What exists today

- **v1 drag** (`src/drag.h`, `Vehicle::applyAtmosphericDrag`): a central
  force `F = −v̂·½·ρ(alt)·Cd·A·v²`, single global `Cd` (`--drag-cd`, default
  1.2), `A = dragArea() = Σ parts (2·radius·height)`. Applied at the COM, so
  **no torque, no attitude dependence, no lift.** See the linked report for the
  full v1 design, data, and the rails-path gap (§6 there).
- **The hook is already per-substep.** `tick.cpp:284` re-applies
  `applyAtmosphericDrag(h)` before every Bullet substep (Bullet clears
  accumulated forces each `stepSimulation`). Adding a torque term slots into
  this exact pattern — no new tick machinery.
- **The flow frame already exists in the code.** A ship in the air is in its
  body's rotating frame, so `v_rel = GetVel()` *is* air-relative (no stasis
  term — see `drag.h`). The ship's world orientation is
  `partRot(rootPart())` (`vehicle.cpp:714`), whose columns are the ship's
  X/Y/Z (right/up/nose) in world axes — enough to project `v_rel` into the
  body frame and read off α, β.
- **No lift anywhere.** `res/*.json` has no `lift`/`wing`/`span`/`chord`/`cop`
  field; `PartDef` has none. Lift is entirely new surface.
- **Force + torque application is available.** Both `ApplyCentralForce(hull, F)`
  (already used by v1 drag) and `ApplyTorque(hull, τ)` (already used by the
  thrust COM-offset correction, `vehicle.cpp:1241`) act on the `hull`.

## 2. The model

Ship body axes: **X = right, Y = up, Z = nose** (forward). `R_ship =
partRot(rootPart())` (world = ship · local). All coefficients are authored
per-part and summed; the force and moment are computed once per substep from
the live attitude.

### 2.1 Flow frame (the foundation)

```
v_rel  = GetVel()                       air-relative (rot frame; see drag.h)
v      = |v_rel|
v_b    = R_shipᵀ · v_rel               velocity in body axes (x right, y up, z nose)
q      = ½ · ρ(alt) · v²               dynamic pressure  (ρ from the existing airDensity)
α      = atan2(v_b.y, v_b.z)           pitch angle of attack (flow vs nose, Y–Z plane)
β      = atan2(v_b.x, v_b.z)           sideslip (X–Z plane)
```

A ship flying straight nose-forward has `v_b ≈ (0,0,v)` → α = β = 0. Pitching
the nose up gives α > 0. (Sign conventions for α/β and the lift direction are
deliberately matched to Orbiter — `aoa = atan2(−airvel.y, airvel.z)` — so the
lift vector points "up" (+Y) for forward flight; the exact sign is pinned by a
unit test, §6.)

### 2.2 Forces (body frame, then rotated to world)

```
lift_dir  = normalize(0, v_b.z, −v_b.y)     ⊥ flow, in the Y–Z (vertical) plane
side_dir  = normalize(v_b.z, 0, −v_b.x)     ⊥ flow, in the X–Z plane (sideslip)
drag_dir  = −normalize(v_b)                 opposite the freestream

L = q · S_L · Cl(α)      lift (0 for a part with no lift area)
S = q · S_D · Cs(β)      side force (optional; 0 in the base model)
D = q · S_D · Cd(α)      drag

Cd(α) = Cd₀ + k·sin²α + Cl(α)²/(π·AR·e)      parasite + weathervane + induced
Cl(α) = cl_slope · α   for |α| < α_stall, soft stall (drop) beyond

F_body = L·lift_dir + S·side_dir + D·drag_dir
F      = R_ship · F_body
```

Notes:
- **Induced drag** `Cl²/(π·AR·e)` ties lift to drag (a wing that generates
  lift also drags). Optional — skip it for the minimal model and the base law
  is just `Cd₀ + k·sin²α`.
- **The `k·sin²α` term is the cheap, high-value drag fix**: an off-attitude
  ship (banked, pitched) presents more area and drags more. This is the
  "weathervane drag" that v1's attitude-independent `−v̂` force lacks, and it
  is present *even for a rocket with no wings*.
- **A rocket has no lift**: with `S_L = 0`, `L = 0`. A rocket's aero behaviour
  then comes from AoA-drag + the CoP moment (§2.3) — which is exactly the
  weathervane/stability behaviour a rocket needs.

### 2.3 Centre of pressure and the moment

The aero force does not act at the COM — it acts at the **centre of pressure**
(CoP), the drag-weighted mean of the parts' pressure centres:

```
CoP_body  = Σ (cop_i · A_drag_i) / Σ A_drag_i      (cop_i in the ship frame)
r_cop     = R_ship · (CoP_body − comOffset)         vector COM → CoP, world axes
τ         = r_cop × F                                moment about the COM
```

- **Stability follows from the sign.** A rocket's big area is the fins, near
  the tail, so the CoP sits *behind* the COM. A +α perturbation puts the drag
  force behind the COM and rotates the nose back into the flow → restoring →
  the ship **weathervanes to prograde** (stable). Flip the CoP ahead of the
  COM (or drop the fins via staging) and it becomes unstable. This is the whole
  pitch-stability story, for free, from the CoP location.
- **`τ = r_cop × F`** is the exact relation (a force `F` at position `r` from
  the COM produces moment `r × F`). Orbiter writes it as `crossp(F, r)` — the
  same magnitude, the sign convention differs; the unit test in §6 fixes which
  one we use.

### 2.4 Application

Replace the single central-force call in `applyAtmosphericDrag` with the
force-and-moment pair, applied to the `hull`:

```
ApplyCentralForce(hull, F)          // the linear part (what v1 already does)
ApplyTorque(hull, r_cop × F)        // the moment part (new)
```

Both are re-applied before every substep (the `tick.cpp:284` hook), so the
moment integrates over the substep exactly like thrust/gravity do today.

## 3. Where the code goes

| Piece | Location | Change |
|---|---|---|
| Aero law (pure math) | `src/drag.h` (extend) **or** new `src/aero.h` | flow frame (α, β, q), `liftForce`, AoA-aware `dragForce`, `aeroMoment` (`r × F`). Keep `airDensity` as-is |
| Per-part aero data | `src/shipdef.h` `PartDef` + `src/part.h` | +`drag_area`, `cd`, `k_drag`, `lift_area`, `cl_slope`, `cl_max`, `stall_angle`, `cop` (all optional, 0/absent = v1 behaviour) |
| Data parse | `src/shipdef.cpp` (catalog parse) | read the new fields (default 0), like the existing optional fields |
| Aggregation | `src/vehicle.{h,cpp}` | replace/augment `dragArea()` with `aeroSums()` → `{S_drag, S_lift, CoP_body}`; rename `applyAtmosphericDrag` → `applyAeroForce` (or keep the name, widen the body) |
| Force + moment | `src/vehicle.cpp` | compute F + τ from §2; `ApplyCentralForce` + `ApplyTorque` |
| Substep hook | `src/tick.cpp:284` | unchanged call (now returns/records force **and** moment) |
| Instruments | `src/tick.cpp` (`--drag-log`), `src/cli.{h,cpp}` | log α, β, q, L, D, τ; new `--aero-log` or extend the existing gate |
| Unit test | `tests/test_drag.cpp` (extend) | lift ⊥ flow, `|L|=q·S·Cl`, induced ∝ `Cl²`, `Cd` grows with α, `τ = r×F` sign, zero in vacuum, CoP composition |
| e2e | `e2e/cases/` (new) | winged ship holds/climbs on α; a rocket's α decays (weathervane); **49-drag-ascent still passes** |
| Part catalog | `res/parts.json` | add a wing + fin part (or add `lift_area`/`cop` to an existing capsule) so there is something to fly |

## 4. Data model

New **optional** fields on a `PartDef` (absent ⇒ 0 ⇒ that part is aero-inert for
that term, so existing parts keep behaving exactly as v1):

```json
{ "name": "wing",
  "mass": 200,  "radius": 0.5,  "height": 0.4,
  "drag_area": 2.0,     // m² — replaces the silhouette contribution
  "cd": 0.04,           // baseline (parasite) drag coefficient
  "k_drag": 0.4,        // sin²α coefficient (weathervane drag)
  "lift_area": 20.0,    // m² — >0 makes this a lifting surface
  "cl_slope": 0.10,     // lift-curve slope per radian
  "cl_max": 1.2,        // stall clamp
  "stall_angle": 0.42,  // rad (~24°)
  "cop": 0.0            // CoP point along this part's axis [m], ship frame }

{ "name": "fin",
  "mass": 40,  "radius": 1.0,  "height": 2.0,
  "drag_area": 4.0,  "cd": 0.6,  "k_drag": 0.3,
  "lift_area": 8.0,  "cl_slope": 0.12,  "cl_max": 1.4,  "stall_angle": 0.5,
  "cop": -3.0 }        // behind the COM → the stabiliser
```

Aggregation (KSP-style, composes with staging):

```
S_drag  = Σ drag_area
S_lift  = Σ lift_area
CoP     = Σ (cop_i · drag_area_i) / Σ drag_area_i     drag-weighted, in the ship frame
```

The `--drag-cd` global knob stays as a master multiplier (A/B and feel tuning),
exactly like today; the per-part `cd` is the authored baseline it scales.

## 5. Phases

Ordered by value-per-effort; each is independently shippable and testable.

| Phase | Adds | Files | Risk |
|---|---|---|---|
| **0 — flow frame** | `v_b`, α, β, `q` as pure math; unit-tested | `drag.h`/`aero.h`, `test_drag.cpp` | Low — no behaviour change |
| **1 — accurate drag** | AoA-aware `Cd(α)` (`Cd₀ + k·sin²α`); per-part `drag_area`/`cd` (replaces the silhouette sum) | `drag.h`, `vehicle.{h,cpp}`, `shipdef.{h,cpp}`, `part.h` | Low — direct refinement of v1; drag now depends on attitude |
| **2 — lift + CoP moment** | `Cl(α)` + lift ⊥ flow, induced drag, force-at-CoP → pitching moment; wing/fin parts | + lift data, `aeroSums()` CoP aggregation, `res/parts.json`, e2e | Medium — the real feature; makes gliders fly and rockets weathervane |
| **3 — polish (optional)** | Mach/wave drag, stall curves, elevator/aileron control surfaces, two-layer density, **rail-path drag** (the v1 §6 gap) | various | Low value/each; add when felt |

Phase 1 is deliberately separable: it makes drag honest about attitude without
committing to the CoP/lift machinery, and it reuses the existing central-force
application. Phase 2 is where the `ApplyTorque` + CoP aggregation + wing parts
land.

## 6. Verification

- **`tests/test_drag.cpp`** (extend, GL/Bullet-free): lift is ⊥ the flow and
  `|L| = q·S_L·Cl(α)`; lift is 0 for a part with `lift_area = 0`; induced drag
  ∝ `Cl²`; `Cd(α)` is monotone in `|α|`; `τ = r_cop × F` with the **sign pinned**
  (a +α with the CoP behind the COM gives a restoring pitch moment); the whole
  thing is 0 in vacuum / at rest; CoP composition (`Σ cop·A / Σ A`) is exact for
  a two-part ship.
- **e2e** (`make e2e`):
  - *Lift:* a winged ship on Duna/Kerbin, level and moving, holds or climbs
    altitude while α > 0 (the `--drag-log`/`--aero-log` shows `L > 0`).
  - *Weathervane:* a rocket given an initial α > 0 with its CoP behind the COM
    sees α decay toward 0 (pitch stability) — the missing v1 behaviour.
  - *Regression:* **49-drag-ascent** still passes (drag still acts on Duna).
- **Instruments:** extend the `--drag-log` line to print `α, β, q, L, D, τ` so
  "is the aero model live and stable?" is a one-flag question, matching the
  existing `--att-log`/`--drag-log` convention.

## 7. Assumptions / simplifications

- **Air-relative velocity = `GetVel()`**, same as v1 (a ship in the air is in
  the body's rotating frame). If a future change lets a ship feel air outside
  the rot frame, the `frame.h` stasis transform must be applied to `v_rel`
  first.
- **Linear lift curve + hard/soft stall.** `Cl = cl_slope·α` up to `cl_max` at
  `stall_angle`, then a drop. No Reynolds/Mach dependence on `Cl` (Orbiter's
  polars are authored as *code*; here they are data + a simple curve — cheaper
  and enough at game scale).
- **Induced drag optional.** `Cl²/(π·AR·e)` needs an aspect ratio `AR` and
  efficiency `e`; skip it in the base model, add it if the lift/drag ratio
  feels off.
- **2D aero (α only) in the base model.** Sideslip β and side force are in the
  math but can be left at 0 until 3D flight (banked turns) actually needs them.
- **CoP is drag-weighted, not a full pressure integral.** It captures the
  stability sign and staging shift, not the exact pressure distribution — the
  same "good enough" bar as v1's silhouette area.
- **No compressibility (Mach / wave drag)** in Phases 0–2. It is a Phase 3
  polish; KSP itself does not model it.

## 8. What we are *not* doing (and why)

- **Not a full CFD / panel method.** Orbiter's airfoil model is the ceiling we
  copy; we do not go to its Reynolds-dependent code-polars or per-surface
  control-surface coupling in the base scope.
- **No per-part `dragCubeSize` (KSP's 6-coeff drag cube).** A single `cd` +
  `k_drag` (sin²α) captures the attitude dependence we need with far less data;
  the 6-component cube is the "more accurate still" lever if this under-shoots.
- **No rail-path aero in Phases 0–2.** The v1 §6 gap (no drag under warp)
  carries over until Phase 3; ascent/descent — where lift/drag matter — run at
  warp 1–10 on the physics path, which is covered.
- **No control surfaces (elevator/aileron) in the base model.** The stick
  already slews the nose via the reaction wheels; aero-stability (CoP moment)
  is what's missing, not manual pitch authority.

## 9. Reference models (read these)

- **Orbiter airfoil surface** — the model to copy: `Src/Orbiter/Vessel.cpp`
  `UpdateAerodynamicForces` (:4099–4225); airframe frame + dynamic pressure
  `Src/Orbiter/Vesselbase.cpp:78–98`; force+torque plumbing `Vessel::AddForce`
  `Src/Orbiter/Vessel.h:1316–1322` (`Amom += crossp(F, r)`); induced/wave drag
  helpers `Src/Orbiter/OrbiterAPI.cpp:873–884`; a concrete polar + surface in
  `Src/Vessel/DeltaGlider/DeltaGlider.cpp:106–129, 1178`.
- **Pioneer** — the cautionary lower bound: per-axis quadratic drag + a faked
  `v̂ × (−forward)` stability torque, no CoP: `src/Ship.cpp:478–555`.
- **v1 drag (this repo)** — `reports/atmospheric-drag2026_09_11` and the code
  cited in §1/§3.

## 10. Key local code references

| Concern | Location |
|---|---|
| Drag law (pure math, v1) | `src/drag.h` — `airDensity`, `dragForce` |
| Force application (v1) | `src/vehicle.cpp` `applyAtmosphericDrag` / `dragArea` |
| Ship world rotation (for the flow frame) | `src/vehicle.cpp:714` `partRot(rootPart())` |
| Central-force + torque API | `ApplyCentralForce` / `ApplyTorque` on `hull` (`vehicle.cpp:1241, 1303`) |
| Substep hook | `src/tick.cpp:284` (after `processGravity()`) |
| Part data model | `src/shipdef.h` `PartDef`, `src/part.h` |
| Part catalog | `res/parts.json` |
| Unit test (v1) | `tests/test_drag.cpp` |
| e2e (v1) | `e2e/cases/49-drag-ascent.txt` |
| Body atmosphere data | `res/ksp_system.json` (`sea_level_density`, `scale_height`) |
