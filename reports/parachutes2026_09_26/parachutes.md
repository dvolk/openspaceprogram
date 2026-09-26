# Parachutes — design & staged plan

Date: 2026-09-26
Status: proposal (not implemented). What's-there / what's-missing / staged plan.

## TL;DR

There is **no parachute, no drogue, no deploy state, and no
landing/crash/touchdown detection anywhere** (grep for `parachute|chute|
drogue|deploy|dome` across `src/` and `res/` finds only "sky dome" comments
and a body named Eurydome). A ship that falls fast simply collides with the
terrain, comes to rest, and nothing happens — no crash, no part damage, no
message.

But almost all of the plumbing a parachute needs already exists and is clean:

- **A per-part drag law** — `drag.h` (`airDensity`, `dragForce`) +
  `Vehicle::applyAeroForce` (`vehicle.cpp:1544`) already apply
  `½·ρ·v²·Cd·A` per substep, gated by the body's atmosphere and `kRhoFloor`.
- **Radial attachment** — the node-based attach model gives every part a
  synthesized `srf` surface node, and radial stacks are already exercised
  (`central_radial.json`, `glider.json`). A chute on the side of a capsule is
  just a `surface`-attach child.
- **A part-window with button blocks** — `drawPartWindows`
  (`gameui.cpp:1718`) already renders docking and crew blocks with buttons; a
  Deploy/Retract block is a direct analog.
- **A save slot** — `SavePart` (`save.h:83-100`) persists per-part state; one
  more bool field is the pattern.
- **A staging path** — if we ever want a "cut-loose" drogue,
  `extractSubtreeAsShip` (`vehicle.cpp:2244`) already detaches a subtree as an
  independent vehicle. (For a tethered drag chute we don't need it — the chute
  stays welded in the one rigid body; deployment is a flag flip, not a
  topology change.)

So a parachute is: **one new part type + one derived predicate + one
per-part drag term + one persistent deploy flag + one UI button + one save
field.** The only genuinely new behavior is the *deployed state that gates a
drag term*; the second (optional) one — touchdown/crash detection — the game
does not have at all today (see `part-damage` report).

---

## 1. What's there

### 1a. Part catalog — no chute-like part

`res/data/parts.json` (62 parts): capsules ×3, reaction wheels ×3, batteries,
RTGs, `engine` ×3, `orbital_engine` ×3, `jet` + `jet_tank` ×3, `fuel_tank`
+ 10 sizes, `mono_tank` ×3, `rcs` ×3, adapters ×6, `decoupler` ×3 +
`decoupler_radial`, `docking_port` ×3, `nose_cap` ×3, `kerbal`, `cargo`,
`wing`, `rudder`, `elevator`, `aileron`, `fuel_link`.

`PartDef` (`src/shipdef.h:231-477`) — behavior is **derived**, not stored:
`isThruster/isJet/isWheel/isRcs/isDecoupler/isCapsule/isContainer/...`
(`src/part.h:150-210`). A parachute is a new derived check:

```cpp
// part.h:150-176 (existing pattern)
bool isWheel() const    { return def != nullptr && def->torque > 0.0; }
bool isRcs() const      { return def != nullptr && def->rcs_thrust > 0.0; }
...
// parachute would be:
bool isParachute() const{ return def != nullptr && def->parachute; }   // NEW
```

No part, mesh, or texture named chute/parachute/drogue exists in `res/`.

### 1b. The drag model to hook into

`src/drag.h` (pure math, 372 lines, pinned by `tests/test_drag.cpp`):

```cpp
// drag.h:54
inline double airDensity(const DragAtmosphere &a, double alt) {
    if(a.sea_level_density <= 0.0 || a.scale_height <= 0.0) { return 0.0; }
    if(alt <= 0.0) { return 0.0; }
    return a.sea_level_density * std::exp(-alt / a.scale_height);
}
// drag.h:65
inline glm::dvec3 dragForce(const DragAtmosphere &a, double cd, double area,
    double alt, const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || cd <= 0.0 || area <= 0.0) return glm::dvec3(0.0);
    return -glm::normalize(v_rel) * (0.5 * rho * cd * area * v * v);
}
```

`Vehicle::applyAeroForce(double h)` (`src/vehicle.cpp:1544`) is where it is
applied, per substep:

- master gate `drag_cd <= 0` (`vehicle.cpp:1561`), body must have a physical
  atmosphere (`:1567-1571`), `alt <= 0` (`:1590`), `rho < kRhoFloor`
  (`:1595`, `drag.h:45` = 1e-15), `v == 0` (`:1601`);
- **ship-level area**: one union convex hull `aeroHull` projected along the
  flow — `A_ship = projectedArea(aeroHull, vhatS)` (`vehicle.cpp:1687`);
- **per-part weighting**: each part's own silhouette weights the centre of
  pressure and the area-weighted Cd mean, using its 3-anchor `partCd`
  (`drag.h:270`) blend (`vehicle.cpp:1668-1684`);
- the force is applied **at the centre of pressure** so the weathervane
  torque is preserved (`vehicle.cpp:1692-1701`);
- lift is a separate per-part term (`vehicle.cpp:1704-1720`) — **this per-part
  loop is the exact pattern a chute drag term copies.**

CLI knobs: `--drag-cd` (`cli.cpp:219`, default 1.2, 0 = aero off),
`--drag-log` (`cli.cpp:225`, prints `[drag]` telemetry incl. `F`, `rho`, `AoA`,
`A`). No `--parachute` knob exists.

**Consequence:** a deployed chute does **not** appear in the current ship
drag (its stowed hull is tiny; the deployed area is a design parameter, not
geometry). The clean integration is an **additive per-part drag term** that
fires only for deployed chute parts: `F_chute = ½·ρ·v²·Cd_chute·A_chute`,
opposite the flow, applied at the chute's part position (off-COM, which
stabilizes — the chute sits behind/above the COM and weathervanes the ship
into the flow, exactly like a real drogue).

### 1c. Attachment — radial is already real

- `struct Node` (`shipdef.h:215-225`); `synthesizeNodes()`
  (`shipdef.h:290-312`) auto-creates `top`/`bottom`/`srf` (surface,
  `surface=true`) — **every part gets a radial attach point for free.**
- `ShipPart` surface edges carry `contactPoint` + `contactNormal` + `roll`
  (`shipdef.h:513+`).
- Already exercised: `central_radial.json` (a radial decoupler + booster on
  the side of a tank), `glider.json` (wings/rudders surface-attached).
- The VAB supports surface attach + radial symmetry cloning (`vab.cpp:244-259`,
  `:340-350`).

### 1d. Staging / detachment (for the "cut-loose" variant)

- `Game::stage` (`game.cpp:~1330-1435`): collects `droppedPartsAtStage(st)`
  (`vehicle.cpp:2122`), then for each decoupler
  `extractSubtreeAsShip(...)` (`vehicle.cpp:2244-2424`) + `enterWorld()`.
- Detached parts **become independent vehicles** with the rigid velocity of
  their own COM + the parent's angular velocity, no separation impulse
  (`vehicle.cpp:2293-2297`, `:2413-2422`).
- A **tethered** drag chute needs none of this — it stays in the one rigid
  body and deployment is bookkeeping on the `Part`, not a physics handle.

### 1e. Lifecycle / ground state

- Spawn: VAB launch (`vab.cpp:588`), `spawn_vehicle` (`vehicle.cpp:332`),
  docking/undock, EVA.
- **There is NO "landed" boolean, NO touchdown event, NO crash detection.**
  The only near-ground concept is `inTerrainBand()` (`vehicle.cpp:2832`,
  periapsis ≤ radius + 3000 m), used only to decide whether a railed ship
  freezes or coasts (`canRail`, `vehicle.cpp:2840`).
- EVA *does* have a grounded state — `eva.cpp:45-63`,
  `k->grounded = BodyInContact(k->hull) || alt < rest + 0.25 m` — and
  `BodyInContact` (`physics.h:86`, `physics.cpp:485`) **exists but is not
  consulted for ships.** That is the hook for "retract on landing" / "crash
  if too fast."
- Ground contact for a ship is pure Bullet collision (compound hull vs
  terrain, friction 4.0, `physics.cpp:288`). No landing gear, no brake.

### 1f. Atmosphere

`AtmosphereParams` (`terragen.h:78-95`): `sea_level_density`, `scale_height`
(0 = no physical air). Bodies with air (ksp_system.json): Eve 1.7/7000,
Kerbin 1.225/5500, Shay 1.225/6000, Duna 0.12/4000, Jool 2.0/20000, Laythe
1.225/5500. No hard top; the `kRhoFloor` gate is the effective ceiling.
**A chute's deploy altitude gate can reuse `airDensityAtCom()`
(`vehicle.cpp:1526`) + `kRhoFloor` — no new "atmosphere top" concept needed.**

### 1g. Keys / UI / save

- Keybindings (`keys.cpp:22-87`): `space` = "Stage / jump" is the only
  candidate near deploy; **no gear/brake/deploy slot exists.** A new
  `Slot::deploy` (or a mouse button in the part window) is additive.
- `drawPartWindows` (`gameui.cpp:1718`): per-part popup with conditional
  blocks — Torque, Jet/Thrust, resources, **docking (Arm/Target buttons)**,
  **crew (EVA/Board buttons)**. A parachute block is a direct analog.
- Save: `SavePart` (`save.h:83-100`) — `part, uid, id, parent, stage, pos,
  rot, mass, hull_margin, fuel[], inventory[]`. Permissive read/write
  (`save.h:~262-300`) so new optional fields load from old saves.
- Tests: `test_drag.cpp` (drag law pins), `test_staging.cpp` (detachment
  tree logic), `test_save.cpp` (round-trip). e2e: `49-drag-ascent.txt`
  (asserts `[drag]` F>0, rho>0), `80-glider-aoa-swing.txt`,
  `02-staging-basic.txt`.

---

## 2. What's missing

| Concern | Status |
|---|---|
| Parachute part def / mesh / texture | **Absent** — not in `parts.json`, `res/meshes`, `res/textures`. |
| `PartDef::parachute` + `chute_area`/`Cd` | **Absent** — `shipdef.h:231-477` has no such field. |
| `isParachute()` predicate | **Absent** — `part.h:150-210`. |
| Deploy state (persistent) | **Absent** — no `Part::deployed`; only transient `armedThrust` (`part.h:84`). |
| Chute drag term | **Absent** — `applyAeroForce` has no per-part additive area. |
| Deploy/Retract input (key + UI button) | **Absent** — no slot, no part-window block. |
| Touchdown / crash detection | **Absent** — `BodyInContact` exists (`physics.h:86`) but ships never consult it. |
| Save persistence for deploy state | **Absent** — `SavePart` has no such field. |
| Tests / e2e | **Absent** — no chute case; `49-drag-ascent` is the template. |

---

## 3. The model

**State.** `Part::deployed` (bool, default false) — a *design* state, unlike
`armedThrust` it survives save/load and staging (a chuted stage that separates
travels with its `Part*`, so the flag automatically moves — decide semantics:
a deployed chute on a dropped booster stays deployed, or auto-retracts;
recommendation: stays deployed, simpler and more physical).

**Force.** In `applyAeroForce`, after the existing ship drag term, add per
deployed chute part:

```
F_chute = -v̂ · ½ · ρ(alt) · Cd_chute · A_chute · |v|²
```

applied at `partPos(p) - com` via the existing `ApplyForce` (off-COM →
stabilizing weathervane torque, for free). Gated by the same atmosphere /
`kRhoFloor` / altitude checks already in the function — a chute in vacuum does
nothing, and a chute in thin air produces a small force, exactly like the rest
of the aero.

**Sizing.** KSP's 1.25 m parachute is ~30 m². A capsule-class chute:
`A_chute ≈ 30 m²`, `Cd ≈ 1.0`. Reference check on Kerbin (ρ₀ = 1.225,
H = 5500): a 2700 kg ship at 100 m/s, alt 5 km (ρ ≈ 0.25 kg/m³) —
`F ≈ ½·0.25·1.0·30·10000 ≈ 37.5 kN ≈ 13.9 g` of deceleration. At 50 m/s the
force is a quarter (~3.5 g). Terminal velocity of a 2700 kg ship under a 30 m²
chute on Kerbin ≈ `sqrt(2·m·g / (ρ·Cd·A))` ≈ **60–90 m/s at 5 km** — i.e. the
chute brings a hard reentry into a survivable-ish descent. Those are tunable
catalog values, not constants.

**Deploy/Retract.** A `Game` method flips `Part::deployed` (and wakes a
railed ship, the way input does at `tick.cpp:104-131`). v1: manual Deploy /
Retract from the part window + an optional `deploy` key. Auto-deploy on a
stage number is a natural later addition (the stage machinery already knows
"when").

**Touchdown (optional, Stage 4).** `BodyInContact(ship->hull)` per
substep → if in contact and `|v|` below a threshold → mark landed, auto-retract
chutes; if above a threshold → route into the damage model (see the
`part-damage` report — this is the collision-damage entry point the chute
report depends on).

---

## 4. Staged plan

### Stage 1 — part type + deploy state (no physics yet)

- `PartDef` (`shipdef.h`): `bool parachute; double chute_area; double chute_cd;`
  (+ optional `deploy_min_alt` / `deploy_max_alt`, default 0 = any altitude
  with air). Load in `shipdef.cpp`.
- `Part` (`part.h`): `bool deployed = false;` next to `armedThrust`.
  `isParachute()` predicate.
- `res/data/parts.json`: `parachute` (1.25 m class, `chute_area 30`) + a
  `parachute_large` (`chute_area 60`). Mesh: a simple dome/canopy OBJ +
  texture (or a placeholder cone — the placeholder-cube fallback at
  `mesh.cpp:85-111` already exists).
- `SavePart` (`save.h`): `bool deployed = false;` + permissive read/write;
  restore in `save.cpp`.
- **Exit:** a ship with a chute loads, saves, and round-trips
  (`test_save` extended); the part window shows a (disabled-for-now) chute
  block. `make test` green.

### Stage 2 — the drag term + deploy control

- `applyAeroForce` (`vehicle.cpp:1544`): per deployed `isParachute()` part,
  add `F_chute` at the part position (pattern: the per-part lift loop
  `vehicle.cpp:1704-1720`). Expose `lastChuteForce` for the `--drag-log`
  line.
- `Game::deployParachute(Part*)` / `retractParachute(Part*)` — flip the flag,
  wake rails. New `Slot::deploy` in `keys.cpp` (default e.g. `B`, which is
  free in `keys.cpp`; or part-window mouse only).
- `drawPartWindows` (`gameui.cpp:1718`): chute block with **Deploy / Retract**
  buttons + state readout (deployed / stowed, chute area).
- **Exit:** e2e `96-parachute-descent.txt` (pattern: `49-drag-ascent.txt`) —
  a chuted capsule in a Kerbin-like atmosphere, deploy, assert
  `[drag]`/`[chute]` F grows and the ship's vertical speed drops to below the
  no-chute terminal velocity. `make test` + `make e2e` green.

### Stage 3 — feel & data

- Deploy/redeploy altitude gates (`deploy_min_alt`/`deploy_max_alt`) using
  `airDensityAtCom()`.
- Drogue + main (two chute parts, sequential deploy) — falls out naturally
  from the per-part model; just catalog data.
- Chute **burst**: a short high-Cd / low-Cd transition on deploy (a rate
  limit on the effective area, `A_eff → A` over ~0.5 s) so the force ramps
  instead of spiking — a `chute_deploy_time` field.
- Render the deployed canopy (the mesh exists from Stage 1; scale/visibility
  by `deployed`).
- **Exit:** a descent feels like a descent; e2e asserts the ramp.

### Stage 4 — touchdown integration (depends on `part-damage`)

- `BodyInContact(ship->hull)` check per substep (the EVA pattern,
  `eva.cpp:53`): landed state on the `Vehicle`, auto-retract, and
  **touchdown-velocity → damage** routed to the damage model.
- "Crashed" message via the toast channel (`Game::toast`, `game.cpp:217-236`
  — `events.cpp` is SDL input dispatch, not a game-event system).
- **Exit:** a fast landing destroys the capsule (damage report Stage 2); a
  chuted slow landing survives and reports "landed."

---

## 5. Open questions

1. **Stowed chute drag.** A stowed parachute still presents a small area —
   today it contributes its hull silhouette to `aeroHull`. Keep that (it's
   free) or model a `stowed_area`? Recommendation: keep the hull contribution.
2. **Chute on a ship with wings.** Lift + chute drag compose linearly in
   `applyAeroForce` — fine, but a glider with a deployed chute is an odd
   design; no special handling needed, just note it.
3. **Deploy in vacuum.** Physically pointless; the `kRhoFloor` gate already
   makes the force zero. Should the Deploy button be *disabled* above the
   air (clearer UX) or allowed (force simply zero)? Recommendation: allowed,
   with the button showing "no air" in the readout.
4. **Auto-retract on landing vs. user control.** Recommendation: auto-retract
   on landed state (Stage 4), user control until then.
5. **Chute as a fuel barrier?** KSP parachutes don't block fuel. Keep them
   transparent to fuel groups (they're not `fuel_barrier`).

## Verification plan (per stage)

- Stage 1: `test_save` round-trip with `deployed` set; `test_shipload` loads
  the new defs; `make test` green.
- Stage 2: new e2e `96-parachute-descent.txt`; `49-drag-ascent` still passes;
  `--drag-log` shows the chute term.
- Stage 3: e2e asserts the area ramp; visual check of the canopy.
- Stage 4: with the damage model — a fast vs. slow landing e2e pair.

---

*Snapshot + proposal, not implemented. Scope: `src/shipdef.{h,cpp}`,
`src/part.h`, `src/vehicle.cpp` (`applyAeroForce`), `src/game.{h,cpp}`
(deploy/retract), `src/keys.{h,cpp}`, `src/gameui.cpp` (part window),
`src/save.{h,cpp}`, `res/data/parts.json`, `res/meshes/` + `res/textures/`
(canopy), `tests/test_save.cpp` (extended), `e2e/cases/96-parachute-descent.txt`
(new). Touchdown integration is deferred to the `part-damage` report.*
