# Drag — area facing the flow (projected-area drag)

Date: 2026-09-20
Status: **design / scope — NOT implemented.** Refines the drag model of
`reports/atmospheric-drag2026_09_11` (v1) and
`reports/aerodynamics2026_09_11` (Phase 1 weathervane term).

## TL;DR

Per-part drag today is

```
F = −v̂ · q · (A·cd + A·k·(1 − (v̂·n̂_root)²))     A = drag_area or 2·r·h
```

where A is a FIXED side-on rectangle and the only attitude dependence is one
global multiplier keyed to the ROOT part's nose. A wing bolted to a tank side
drags exactly the same whether it is flat to the flow or turned edge-on, and a
rocket ascending vertically is charged its side area (2·r·h) instead of its
end-on area (π·r²).

The fix is the standard form, with the area made geometric:

```
F_i = −v̂ · q · Cd_i · A_proj,i(v̂)
```

`A_proj(v̂)` = the part's convex-hull silhouette facing the flow. For any
convex body this is exact and cheap to evaluate:

```
A_proj(d̂) = Σ_faces A_f · max(0, n̂_f · d̂)
```

(front faces tile the silhouette exactly — the standard convex-polyhedron
projection result). The hull faces are extracted ONCE from the part's existing
collision hull (`btConvexHullShape`) at ship build; per substep it is one
matrix·vector plus ~20 dot products per part. Attitude dependence, per-part
orientation, and the weathervane behaviour all fall out of the geometry — the
fudged `k` term and the fixed `drag_area` are retired, `cd` stays as the
per-part coefficient and `--drag-cd` as the global feel knob (still the
"0 = no aero" master switch).

---

## 1. What is wrong with the current model

The current law (`Vehicle::applyAeroForce`, `vehicle.cpp:1410–1420`, with the
math in `drag.h` `partDrag` / `offAxisFactor`):

1. **The area is attitude-independent.** A part's A is `drag_area` or the
   side-view rectangle 2·r·h, whatever way it points. A rocket flying straight
   up presents its END faces (π·r² per tank) but is charged 2·r·h — for a
   2 m × 3 m tank that is 9 m² against a true 7.1 m² (27% high); for a short
   wide part (a decoupler, r=1 h=0.25) the model charges 0.5 m² where the
   true end-on area is 3.14 m² (6× LOW). The error swings both ways with
   aspect ratio, because the real area is
   `A(θ) = 2·r·h·|sin θ| + π·r²·cos²θ` for a cylinder (θ = flow vs axis), not
   a constant.
2. **The attitude term is global, not per-part.** The weathervane factor
   `k·(1 − (v̂·n̂)²)` uses the ROOT part's nose for EVERY part. The glider's
   wings (attached to a tank side, normal radial, chord along the ship axis —
   `res/ships/glider.json`) respond to how the whole ship points, not how the
   WING points: a wing rolled 90° (span into the flow, drags heavily in
   reality) is indistinguishable from one flat (drags almost nothing).
3. **`k` is a fudge.** Default `k = 1.0` (`--drag-k`) simply doubles drag when
   the ship is off-nose. It is a proxy for problem 2 that the projected-area
   model subsumes with real geometry.

Lift, stall, control surfaces, and the CoP moment (force applied at each
part's position) are all fine and stay as-is. This change touches the DRAG
AREA only.

## 2. The model

For part i with convex hull faces `{n̂_f, A_f}` (part-local frame) and flow
direction `v̂` (world):

```
v̂_local = R_iᵀ · v̂                          (R_i = partRot(p), one mat3·vec)
A_proj,i = Σ_f A_f · max(0, n̂_f · v̂_local)  (exact silhouette, convex body)
F_drag,i = −v̂ · q · Cd_i · A_proj,i          (the standard drag law)
```

`q = ½·ρ·v²` and the force application (at the part position, so the CoP
torque is unchanged) are exactly as today.

Why the hull faces:

- **It is the game's own physical shape.** Parts already collide as the convex
  hull of their mesh (`PhysicsEngine::BuildHull`, `physics.cpp:298`); drag
  area and collision silhouette then agree by construction.
- **Exact, for every shape in the catalog.** Cylinders (tanks, engines),
  frustums (adapters, nose caps — which carry no tip radius in `PartDef`, so
  an analytic formula would need new data), the delta wing (a flat prism whose
  silhouette goes from 0.2 m² edge-on to 2 m² face-on automatically), and even
  the kerbal. No shape classification, no per-part-type formulas.
- **Consistent with the physics of a hull with a margin:** the stored hull
  faces are pre-margin (Bullet applies `m_margin` in the support mapping, not
  to the stored geometry), so the area is the true hull of the mesh — the
  0.1 m collision inflation does not inflate drag area.
- **Resolution:** the tank/capsule cross-sections are 32-gons (64 vertices /
  2 rings), so end-on areas are within ~1% of the true circle.

Consequences (all desired):

- A rocket ascending vertically presents end faces; pitching over sweeps the
  side area in, monotonically — the weathervane behaviour now comes from
  geometry, per part.
- Wings drag almost nothing flat (they present thickness × chord) and a lot
  broadside — with the wing's OWN orientation, not the root's nose.
- Staging still shifts the ship's drag (the dropped tail took its area with
  it), now with the right areas.

**Known approximation (same as today):** the per-part sum double-counts
overlapping silhouettes (an engine inside its shroud counts twice). KSP does
the same (per-part drag cubes); a whole-ship convex hull is wrong for winged
craft (it would include the empty volume between wing and fuselage). Leave it.

## 3. Where the code goes

| Piece | Location | Change |
|---|---|---|
| `projectedArea(faces, d̂)` + `AeroFace` | `src/drag.h` | New pure math (glm only, testable like the rest of the header) |
| Face extraction | `src/physics.cpp` (or a helper in `vehicle.cpp`) | New: pull faces from the part's `btConvexHullShape` (`getFaceCount` / `getFaceVertexCount` / `getFaceVertex`), fan-triangulate n-gons, emit `(normal, area)` in part-local frame. Called once per part at `build_ship` time, AFTER the hull exists (`BuildHull`) |
| Per-part storage | `src/part.h` `Part` | `std::vector<AeroFace> aeroFaces;` (~20 faces × 32 B ≈ 0.6 KB/part; optionally cached per `Mesh*` since the `get_mesh` registry shares meshes) |
| Drag application | `src/vehicle.cpp` `applyAeroForce` | Per part: `v̂_local = partRot(p)ᵀ·v̂`, `A = projectedArea(...)`, `F = −v̂·q·cd·A`. Drop the `offAxis` and `k` terms from the drag path (`aeroFrame` stays — lift reads `alpha`) |
| Field retirement | `src/shipdef.{h,cpp}`, `src/cli.{h,cpp}`, `src/vehicle.h` | Remove `drag_area`, `k_drag`, `--drag-k`, `AeroFrame::offAxis` (and the `dragForce` / `dragForceAOA` / `offAxisFactor` / `partDrag`-with-`k` family in `drag.h` — the per-part line is two expressions now; keep `dragForce` only if a test still wants the v1 law, otherwise retire all four) |
| Instruments | `src/tick.cpp` `--drag-log` | Drop `K=`, add the ship's total `A = Σ A_proj,i` — "how much area is the ship showing the air right now" makes the attitude dependence visible in the log |
| Part data | `res/parts.json` | Drop `drag_area` / `k_drag` from wing/rudder/elevator/aileron; re-tune their `cd` (see §6) |
| Unit tests | `tests/test_drag.cpp` | Replace the `offAxis` / `dragForceAOA` / `partDrag` blocks with `projectedArea` pins (§7); density/lift/stall/control blocks stay |
| e2e | `e2e/cases/` | 49-drag-ascent re-verified; new AoA-swing case (§7) |

## 4. What is retired, and why that is safe

- **`k_drag` / `--drag-k` (weathervane term):** the projected area IS the
  weathervane term, computed honestly per part instead of one global fudge.
  Keeping both would double-count attitude dependence.
- **`drag_area` (fixed area override):** its only consumers are the four
  wing-family parts, and for plates the hull silhouette is strictly better
  (it is the planform face-on and ~0 edge-on — the authored constant was a
  compromise between those two). `cd` remains the authoring knob.
- **`AeroFrame::offAxis`:** only the drag path reads it.

Per QWEN.md this project is past the point where backward compatibility
matters; no ship def sets these fields (they live in `res/parts.json`), and no
UI surface displays them (verified: the only consumers are
`shipdef.{h,cpp}`, `cli.{h,cpp}`, `vehicle.{h,cpp}`, `tick.cpp`, `drag.h`,
`test_drag.cpp`).

## 5. Cost

- **Build time:** one hull-face walk per part at `build_ship` (or once per
  shared mesh). Negligible.
- **Per substep:** per part, one mat3·vec (9 muls) + F dot/max (F ≈ 10–30).
  A 40-part ship at the max-warp substep rate (~1000/s) is ≈ 3 MFLOP/s —
  next to nothing against the Bullet world step that already runs every
  substep. No per-frame allocation.
- **Memory:** ~0.6 KB/part (less with a per-mesh cache).

## 6. Tuning consequences (expect and absorb, don't fight)

Areas move, so drags move:

| Ship / attitude | Old area×cd | New area×cd (cd unchanged) | Note |
|---|---|---|---|
| Tank r1 h2, nose-on (vertical ascent) | 1.2 × 4.0 | 1.2 × 3.1 | ~21% less — 49-drag-ascent still sees F > 0 |
| Decoupler r1 h0.25, nose-on | 1.2 × 0.5 | 1.2 × 3.1 | 6× MORE (short wide part: true end-on area is a full circle) |
| Wing, flat to flow | 0.08 | ~0.01 | 10× less — the old value was propped up by k=0.8 |
| Wing, 90° to flow | 0.84 | ~0.08 | 10× less — a thin airfoil at 90° really is ~1.2 × planform, and the old 0.84 was the k-term, not area |

Net: **wings drag ~10× less than today.** That is the physically honest
number (thin airfoil Cd ≈ 0.01–0.02 flat), and it means the glider's L/D
improves a lot. Two levers to bring the feel back, in order of preference:

1. **Re-tune the wing-family `cd` up** (`res/parts.json`, 0.04 → ~0.1):
   restores flat-flight drag to the real thin-airfoil value at almost no cost,
   keeps the honest attitude swing.
2. **Induced drag (Phase 3 below):** a wing that generates lift MUST drag
   more — `D += q·S_lift·Cl²/(π·AR·e)` — the standard polar term, and the
   correct brake on the glider's improved L/D. With `k` gone, this is what
   replaces the wing's attitude drag in the cruise regime.

Rocket ships are barely affected (end-on areas are within ~25% of the old
silhouettes), so 49-drag-ascent and the ascent feel are safe.

## 7. Phases

Each independently shippable; all small.

| Phase | Adds | Files | Risk |
|---|---|---|---|
| **1 — geometry** | `AeroFace` + `projectedArea` in `drag.h`; hull-face extraction at build; per-part face storage; unit tests pinning plate/cube/cylinder silhouettes. No behaviour change yet (old drag path untouched) | `drag.h`, `part.h`, `vehicle.cpp` (build), `physics.cpp`, `test_drag.cpp` | Low |
| **2 — switch** | `applyAeroForce` uses `A_proj`; retire `k_drag` / `--drag-k` / `drag_area` / `offAxis` and the v1 composite law family; `--drag-log` prints total `A`; `res/parts.json` re-tuned (wing `cd` 0.04 → ~0.1, `drag_area`/`k_drag` dropped); e2e: 49-drag-ascent + new glider AoA-swing case | `vehicle.{h,cpp}`, `shipdef.{h,cpp}`, `cli.{h,cpp}`, `tick.cpp`, `parts.json`, `tests/`, `e2e/cases/` | Low — the whole model is one expression per part |
| **3 — induced drag (recommended)** | `D += q·S_lift·Cl²/(π·AR·e)` for lifting parts; new optional `aspect_ratio` field on `PartDef` (0 = none; wing: span 2 m / area 2 m² → AR 2). Restores honest lift-drag coupling and the glider's cruise brake | `drag.h`, `shipdef.{h,cpp}`, `vehicle.cpp`, `parts.json`, `test_drag.cpp` | Low |
| **4 — polish (defer)** | Mach/wave drag; KSP-style per-axis drag cubes if some shape still feels wrong; the rail-path drag gap (v1 §6, still open) | various | Low value until felt |

## 8. Verification

- **`tests/test_drag.cpp`** (extend, GL/Bullet-free — synthetic face lists):
  - a 2 × 4 plate (normal +Z): `A_proj(ẑ) = 8` exactly; `A_proj(x̂) = 0`;
    `A_proj(normalised(1,0,1)) = 8/√2`;
  - a side-2 cube: `A_proj(axis) = 4`; `A_proj((1,0,1)/√2) = 4√2` (the
    rectangle silhouette — pins the front-face tiling);
  - a 32-gon prism (r=1, h=3): end-on ≈ π (within 1%), side-on ≈ 2·3 (within
    the apothem factor) — pins "cylinder limit";
  - the drag line: `F = −v̂·q·cd·A_proj`, anti-parallel to v, zero for
    degenerate input;
  - existing density / lift / stall / control-surface blocks untouched.
- **e2e** (`make e2e`):
  - **49-drag-ascent** still passes (F > 0, rho > 0 — the vertical rocket now
    presents end faces, non-zero; the ~21% drag drop must not break the case's
    12 s window — check the ascent timeline if it does);
  - **new — glider AoA swing:** the glider on a body with atmosphere, moving,
    prograde → record `[drag] F`; pitch the nose (sim-press W) to ~45° at the
    same speed/altitude band → assert F rises ≥ 2× (broadside vs edge-on
    wing). This is the direct "area facing the velocity" acceptance test;
  - 51-jet-vtol re-run (jets are on a rocket stack; expect negligible change);
  - `--drag-log` on a hand-flown glider: `A` climbs smoothly as the nose
    comes off the flow (instrument eyeball check, per QWEN.md).
- **Build/verify:** `make test` + the e2e set before committing (QWEN.md).

## 9. Assumptions / simplifications

- **Air-relative velocity is `GetVel()`**, as v1 (rot-frame ship).
- **One `Cd` per part**, applied to the true projected area. A real airfoil's
  drag coefficient varies with its own AoA (peaks ~1.2–1.5 near stall); a
  single Cd cannot capture both the flat and the 90° regimes — we tune for the
  cruise regime (Phase 2 retune) and let induced drag (Phase 3) supply the
  lift-coupled part. KSP's own drag cubes make the same choice.
- **Per-part sum, not a ship hull** (overlapping silhouettes double-count):
  see §2. Same as today, same as KSP.
- **No compressibility (Mach/wave drag)** — unchanged from v1, Phase 4.
- **Convex hull as the geometric source of truth:** non-convex meshes (a
  nozzle curve) are convexified for collision already, so drag and collision
  share one shape. A concave mesh's TRUE silhouette would differ slightly;
  irrelevant at this scale.

## 10. Rejected alternative: analytic solid-of-revolution formulas

`A(θ) = 2·r·h·|sinθ| + π·r²·cos²θ` (cylinder) is exact for cylinders and one
dot product per part, but: adapters/nose caps are frustums whose tip radius is
NOT in `PartDef` (only the max radius is — `res/parts.json`), the kerbal and
wing are not solids of revolution at all, and it would need a per-part shape
classification + new fields to be right. The hull-face sum costs ~15 flops
more per part, needs zero new data, and is exact for every mesh the game
already ships. The analytic formula stays available as a test oracle (the
32-gon-prism pins above compare against it within 1%).

## 11. Key local code references

| Concern | Location |
|---|---|
| Current per-part drag (to replace) | `src/vehicle.cpp:1410–1420` (`applyAeroForce`) |
| Current law (pure math) | `src/drag.h` — `offAxisFactor`, `dragForceAOA`, `partDrag` |
| Per-part world axes (for `v̂_local`) | `src/vehicle.cpp:765` — `partAxis` / `partRot` |
| Hull (face source) | `src/physics.cpp:298` — `BuildHull` → `body->shape` (`btConvexHullShape`) |
| Part storage | `src/part.h:51` — `struct Part` |
| Substep hook (unchanged) | `src/tick.cpp:290` — `applyAeroForce(h)` per substep |
| Instrument | `src/tick.cpp:448` — `--drag-log` |
| Wing geometry (plate case) | `res/wing.obj` (delta, chord 2 × span 2 × t 0.1, normal = local Y) |
| Ships using the wing family | `res/ships/glider.json` |
| e2e (regression) | `e2e/cases/49-drag-ascent.txt`, `51-jet-vtol.txt` |
| v1 design | `reports/atmospheric-drag2026_09_11` |
| Phase-1 weathervane design (retired by this) | `reports/aerodynamics2026_09_11` §2.2 |
