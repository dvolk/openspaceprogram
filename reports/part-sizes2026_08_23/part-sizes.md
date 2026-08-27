# Part sizes: 1 m / 3 m / 5 m radius, 1 m / 3 m / 5 m height

**Question (Denis, 2026-08-23):** KSP-style part sizes — different radii (1, 3, 5 m)
and different heights (1, 3, 5 m). What should that look like in JSON? Design with
staging and radial parts (later) in mind. And: can we make simple meshes in the
current style (KISS = rescale the current ones)?

**Short answer:** Yes, meshes — done (§6). JSON: two new **optional** fields on
the part catalog, `"radius"` and `"height"` in metres, defaulting to (1, 2) — the
current 2 m cube — so all existing content is untouched. The only code change is
parameterizing the two weld anchors (hardcoded ±1 m today) by each part's height.
That anchor parameterization is *exactly* what radial parts needs next, so this
work is phase 0 of radial, not parallel work.

**Status (2026-08-23, end of round 4):** code round DONE — `radius`/`height` in
`shipdef`, anchors parameterized (`GlueTogether(parent, child, ap, ac)`), plume
scaled per thruster, `controller` field now applied. Catalog per Denis:
capsules and engines keep the **original proportions (height = 2 × radius** —
r1h2 / r3h6 / r5h10), fuel tanks **the full r × h grid**, reaction wheels **thin
discs with h = 25 % of the radius** — **20 parts** total, all named after their
size. Round 4 also fixed a real physics bug the disc exposed: the fixed 0.5 m
convex-hull margin made every welded pair of part hulls overlap by 1.0 m, and
the contact solver's correction impulse tumbled a low-inertia thin part (the
0.25 m wheel) — margin is now **0.1 m** (0.2 m overlap), verified stable
headless for every ship. Big/Tall rebuilt for the taller capsule/engine; tests
green, headless unpaused smoke green (all 5 ships steady, no separation).

---

## 1. What the code actually does today (ground truth)

| Fact | Where | Verified |
|---|---|---|
| All part meshes are exactly `[-1,1]³` (2 m cube, radius 1 m, height 2 m) | `res/capsule.obj` (64 v), `res/reaction_wheel.obj` (64 v), `res/engine.obj` (192 v) | bbox measured: x, y, z all ∈ [−1, 1] |
| Part axes: local +Z = thrust/stack axis, centered on the part center | `build_ship` `main.cpp:1417-1418` | |
| Mesh pipeline: Assimp import → GL buffers **and** a `double* vs` copy that Bullet builds a **convex hull** from, margin 0.1 m (was 0.5 — see §4) | `mesh.cpp` `AssImpFromFile`; `physics.cpp:227-234` | |
| → **collision shape is derived from the mesh vertices**: a rescaled `.obj` resizes both rendering and physics with zero code | | |
| Stack spacing: ship offsets are absolute metres (2 m apart in `res/ships/basic.json`); the 2 m is *enforced* by the weld anchors: parent `(0,0,−h_p/2)`, child `(0,0,+h_c/2)` | `GlueTogether` `physics.cpp:269-292` (anchors :277-280); call site `main.cpp:848` | |
| The VISIBLE meshes touch exactly (faces meet) — that's what you see in game. The INVISIBLE collision hulls (mesh + 0.1 m margin, `physics.cpp:233`) overlap **0.2 m per joint** (2 × margin); the weld pins the pose, the small residual overlap is absorbed without destabilising even a thin part | `physics.cpp:233` | the hull overlap is size-independent = 2 × margin (see §4) |
| Engine plume is drawn in each thruster's local space; the plume mesh (`z ∈ [−5,−1]`) is authored to start exactly at the tail of a 2 m engine | `main.cpp:2907-2925`, `res/engine_plume.obj` | |
| `ShipDef::controller` is parsed + validated but **never applied** — `Vehicle::init` hardcodes `controller = parts.back()` (`main.cpp:860`) | `shipdef.cpp`, `main.cpp:860` | known dead field (from the radial-parts scoping, since deleted from the tree) |

Consequence: the sim layer is already size-agnostic (thrust = part local +Z,
torque = part local axes, hull/inertia from the mesh). Exactly three things are
hardwired to the 2 m size: the two weld anchors, the plume's authored tail, and the
ship-JSON offsets (content, not code).

---

## 2. Size model

Two independent dimensions, both per-part, both in metres:

- **`radius` r** — cross-section (x/y extent `2r`). Governs how wide the part is,
  and (later) where its radial ports sit: on the surface, at distance `r` from the axis.
- **`height` h** — extent along the stack/thrust axis (z extent `h`). Governs
  stacking spacing and (later) where a staging cut lands: the part's faces are at
  `±h/2` in its local frame.

The listed classes are r ∈ {1, 3, 5}, h ∈ {1, 3, 5}; the existing parts are the
(r1, h2) legacy class and keep working unchanged. Any (r, h) pair is legal —
nothing enforces class compatibility, so a 1 m capsule on a 5 m tank is a valid
stack (it just looks stepped; KSP-style *adapter* parts are pure content — a short
stepped mesh with the two radii, §7).

Everything downstream derives from `r`, `h` + the mesh:

| Quantity | Source |
|---|---|
| collision hull, moment of inertia | mesh (already automatic) |
| axial weld anchors | `±h/2` (new, §4) |
| radial port distance (future) | `r` (this field is what radial needs) |
| staging cut plane (future) | part position `± h/2` |
| plume origin (future-ish) | tail = `−h/2` |

---

## 3. JSON (recommended)

**Parts catalog** — two optional fields; defaults keep all current entries valid:

```json
{
  "name": "engine_l",
  "type": "engine",
  "mesh": "engine_r5h2.obj",
  "texture": "engine.png",
  "mass": 1000,
  "radius": 5.0,
  "height": 2.0,
  "fuel_rate": 1.0,
  "exhaust_velocity": 40492
}
```

- `"radius"` default 1.0, `"height"` default 2.0 → the existing 4 entries are
  unchanged (no migration).
- Validation in `shipdef.cpp` (GL-free, headless-tested): both `> 0`.
- The `.obj` must be authored at the declared size (the hull uses it anyway); the
  fields are the *physics/anchor* truth, the mesh the *geometry* truth. Optional
  safety net: `build_ship` could warn if the loaded mesh bbox disagrees with the
  declared (r, h) — cheap to add, since the vertices are in hand.
- Naming: `<base>_<size>` free-form (like `type` today) — e.g. `engine`,
  `engine_m`, `engine_l`; height variants `engine_m_h3`, …

**Ship defs — no new fields.** Offsets stay absolute metres from the ship base; the
spacing rule generalizes:

```
offset(parent) − offset(child)  =  (h_parent + h_child) / 2        [child below parent]
```

Today's 2 m spacing is the (2, 2) case — existing ships are numerically unchanged.

**Alternatives considered, rejected:**

1. *Size classes* (`"size": "medium"` + a global table): extra indirection, can't
   express a mixed class (r3, h5) without new table entries, and the project is
   field-driven — explicit numbers fit.
2. *Scale factors in JSON, code scales the normalized mesh at load*: viable (the
   Assimp vertex vectors could be scaled before upload; the hull `vs` copy would
   scale too), keeps `res/` small, but adds a code path to the mesh loader —
   including normal transformation, a subtle spot — and the `.obj` on disk no
   longer matches what renders. Rejected.
3. *Derive r/h from the mesh bbox at load*: single source of truth, but the anchor
   needs the number in `shipdef.cpp`, which is deliberately GL-free and can't read
   `.obj` there. Rejected.

---

## 4. Code change (small, ~60 lines)

1. `shipdef.h/.cpp` — `PartDef.radius`, `PartDef.height` (defaults 1/2), parse +
   validate, header comment.
2. `physics.h/.cpp` — `GlueTogether(Body *parent, Body *child, glm::dvec3 ap, glm::dvec3 ac)`:
   anchors become parameters (today hardcoded `(0,0,−1)` / `(0,0,+1)`).
3. `main.cpp:848` — `attachDown` passes `(0,0,−h_parent/2)` / `(0,0,+h_child/2)`
   from the parallel `partDefs`.
4. `main.cpp:2911` (plume draw) — per-thruster scale matrix `(r, r, h/2)`: the
   plume tail lands on the engine tail at any size, and the plume grows with the
   engine (no per-size plume meshes needed). Needs the thruster's (r, h) stored in
   `Vehicle::init` — it already walks `partDefs` there, one more parallel vector.
5. `tests/test_shipload.cpp` — radius/height defaults, explicit values, error paths.

**Stability note (the real risk — it materialised, round 4):** the *visible*
meshes touch exactly at the seam (that's what you see in game). But the
*invisible* collision hulls (each part's mesh expanded by the hull margin)
overlap by `2 × margin` at **every** welded joint, independent of part size —
the faces meet, so each hull reaches `margin` past the seam and the two hulls
interpenetrate by `margin + margin`. Bullet's contact solver fires a correction
impulse on every substep to resolve that overlap, and the weld fights it.

With the original 0.5 m margin (1.0 m overlap) this was invisible for the
chunky 2 m parts — their moment of inertia is large enough to absorb the
impulse. But it exposed a real bug the moment a **thin** part entered the stack
(the 0.25 m reaction-wheel disc, round 3): the same impulse on a low-inertia
part flung it and the ship tumbled on the pad (the Alpha ship — "the wheel
separates and rolls on its own"). A headless A/B (disc vs a 2 m cube in the
same ship, and a margin sweep 0.5 → 0.05) isolated it to the margin: at 0.5 m
the disc ships tumble, at 0.1 m (0.2 m overlap) every ship — disc and chunky
alike — is rock-stable. The margin is now **0.1 m** (tunable via
`OSP_HULL_MARGIN`), and thin parts are safe to stack.

Nothing in the frame/physics core, no new behavior fields, no new part types.

---

## 5. What this sets up (staging, radial)

- **Radial parts** (scoped 2026-08-23, report since deleted from the tree): the
  radial weld anchor sits on the parent's surface at distance `r` from its axis,
  weld distance `(r_parent + r_child)`, roll about the radial axis. With the
  `radius` field those are one-liners; without it they'd be hardcoded 1 m. The
  anchor-parameterization from §4.2 is the same function call.
- **Staging**: a decoupler is just a thin part (small `h`, e.g. 0.5–1 m) + a detach
  action at that plane — the cut plane is exactly the part's face, defined by
  `height`. Also worth folding in: the dead `controller` field (§1) — a staged
  ship's cockpit isn't the last part, so the controller must actually be applied
  (`main.cpp:860`).

---

## 6. Meshes — done

**Confirming the question: yes**, the current style is trivially rescalable. The
parts are plain axis-symmetric `.obj` (Blender export: `v` / `vt` / `vn` / `f`,
Assimp-imported), centered at the origin with +Z along the stack axis. Rescaling
is an exact transform:

```
position  v  ->  v · (r, r, h/2)
normal    vn ->  normalize( vn · (1/r, 1/r, 2/h) )   (inverse-transpose of a diagonal scale)
uv, faces ->  untouched
```

**Generated** (rounds 1–4): `rescale_obj.py` at the repo root (next to
`gen_systems.py`), plus the size set for the three part shapes = **20 part
meshes**:
- **capsule / engine** — one per radius, **height = 2 × radius** (the original
  proportions, uniform scale): `capsule{,_r3h6,_r5h10}.obj`,
  `engine{,_r3h6,_r5h10}.obj` (round 4 replaced the round-3 h=2 variants —
  `*_r3h2`, `*_r5h2` — which were removed as orphaned).
- **reaction wheel** — thin discs, h = 25 % of the radius:
  `reaction_wheel_r{1,3,5}h{0.25, 0.75, 1.25}.obj`.
- **fuel tank** — the full r × h grid, reusing the reaction-wheel shape:
  `reaction_wheel.obj` (legacy r1h2) + `reaction_wheel_r{1,3,5}h{1,3,5}.obj`
  + `reaction_wheel_r3h2.obj`.

Verified: bounding boxes land exactly on (±r, ±r, ±h/2), normals unit length
(the rescaler renormalizes — the base files carry Blender's non-unit normals).
The existing `engine.obj` et al. (r1h2) are untouched and remain the legacy
(r1, h2) class.

---

## 7. First content (created — for the flight test)

Scope settled (Denis, round 3): capsules and engines come in **one size per
radius**, fuel tanks cover **the full r × h grid**, reaction wheels are **thin
discs with h = 25 % of the radius**. `parts.json` is now **20 parts** (behaviour
numbers are a starting point to tune in play):

| family | parts | sizes / behaviour |
|---|---|---|
| capsule | `capsule`, `capsule_r3h6`, `capsule_r5h10` | r 1/3/5 m, height = 2 × radius; + 200 N·m wheel |
| reaction_wheel | `reaction_wheel`, `reaction_wheel_r3h0.75`, `reaction_wheel_r5h1.25` | discs r 1/3/5 m, h 0.25/0.75/1.25 m; 2000 N·m |
| engine | `engine`, `engine_r3h6`, `engine_r5h10` | r 1/3/5 m, height = 2 × radius; 1×/2×/4× flow → 80 984 / 161 968 / 323 936 N |
| fuel_tank | `fuel_tank` + `tank_r{1,3,5}h{1,3,5}` (+ `tank_r3h2`) | the full r × h grid; 1000 kg H₂ + 1000 kg LOX each |

Mass is flat across sizes (capsule 500, wheel 1000, engine 1000, tank 2000 kg);
thrust scales with radius; tank capacity is flat for a first pass — all three
are candidates for tuning in play.

Because the r1 wheel is now a 0.25 m disc, the spacing around it in the Basic
and Explorer ships shrank from 2 m to (2 + 0.25)/2 = 1.125 m; those ships keep
their mass/thrust/fuel (Basic is still 4500 kg / 80 984 N / 2000 kg fuel).

Two test ships, both in `res/fleet.json` on the pad (Delta and Echo):

- **Big** — `res/ships/big.json`: `capsule` (r1h2) → `tank_r3h2` →
  `engine_r5h10`. The 1 → 3 → 5 radius step; the h-10 engine makes the bottom
  step taller — spacing (h_top + h_bottom)/2 gives offsets 14.5 / 12.5 / 6.5.
- **Tall** — `res/ships/tall.json`: `capsule_r3h6` → `tank_r5h5` →
  `engine_r5h10`. A 5 m-tall tank between the two h-6 / h-10 parts: spacing
  (h_top + h_bottom)/2 gives 19.5 / 14.0 / 6.5.

Run: `./osp --fleet res/fleet.json` — Delta and Echo stand on the pad alongside
Alpha (the fleet has no CLI default, so the `--fleet` flag is required).

---

## 8. Status (end of round 4, 2026-08-23)

- **Naming:** parts named after their size, like the mesh files (Denis, round 2)
  — `engine_r5h10`, not `engine_l`.
- **Catalog scope (Denis, round 4):** capsules/engines one per radius with
  **height = 2 × radius** (the original proportions, r1h2 / r3h6 / r5h10); fuel
  tanks the full r × h grid; reaction wheel height = 25 % of the radius (thin
  discs). 20 parts total.
- **Code round:** done (round 2) — §4 items 1–5 plus the `controller` field
  now actually applied (`Vehicle::init`). `make test` green
  (shipload/fleet/calendar/thrust/rotation + 333 unit checks).
- **Physics fix (round 4):** the fixed 0.5 m convex-hull margin was making every
  welded part pair overlap by 1.0 m; the contact solver's per-substep impulse
  tumbled a thin low-inertia part (the 0.25 m wheel disc) — the Alpha ship bug.
  Margin is now **0.1 m** (0.2 m overlap, `OSP_HULL_MARGIN`), stable for all ships.
- **Verification:** `make test` green; headless unpaused smoke (Xvfb,
  `--fleet res/fleet.json`, 45 s) — all 5 ships steady, no separation or tumble.
