# Rocket landing legs — design + staged plan (2026-11-21)

One radially-mounted leg part, surface-attached to a fuel tank's side near its
bottom, splayed ~20° so a rocket stands on its feet with the engine bell clear.
Fixed legs for v1 (no deploy/retract). Test ship: a legged copy of `racer`.

Decisions made in discussion: fixed legs; report first, then prototype (stage 1
by the original agent, later stages by local agents).

## 1. What already exists (the load-bearing findings)

### The attach question is already answered by the surface node

A part authors at most one node with `"surface": true` (`src/shipdef.h:163`,
`Node`). Surface attach places THAT node's `pos` on the parent's hull contact
point with its `dir` pointing into the parent; the rest of the mesh hangs off
it (`attachSurface`, `src/shipdef.cpp:1443`). The node IS the "which bit
bolts to the hull" answer — no new attach machinery is needed for a leg.

The airplane gear is already the plate-at-one-extreme precedent:
`utils/gen_gear.py` authors the mesh origin at the wheel axle with the strut
rising to z=0.65, and `res/data/parts.json` authors the `srf` node at the
strut top (`[0,0,0.65]`, dir `[0,0,1]`). Bolted to a belly, the strut top sits
on the hull and the wheel dangles below.

Mating rotation (`surfaceMatingRot`, `src/shipdef.cpp:1408`) keeps the child's
+Z parallel to the parent's stack axis projected into the contact plane. On a
vertical tank the leg's +Z stays VERTICAL, so the splay angle must be baked
into the mesh (strut runs down-and-out from plate to foot). Q/E roll turns
about the contact normal — on a tank wall that swings the leg fore/aft, not
in/out, so there is no user-facing splay adjustment (fine for v1).

VAB placement (`vabUpdateHover`/`vabPlace`, `src/vab.cpp:190,281`) and radial
symmetry (`radialSymmetryClones`, `src/shipdef.cpp:1548`) already work for any
part with a surface node: arm the leg, hover the tank, click; 3/4× symmetry
rings the parent's axis with congruent clones.

### The ground law is reusable, not rewriteable

The ship is ONE rigid body: a `btCompoundShape` of per-part convex hulls
(`Vehicle::rebuildCompound`, `src/vehicle.cpp:~1600`). Gear ground interaction
is a per-substep raycast + force applier, `Vehicle::applyGroundForces`
(`src/vehicle.cpp:4750`), calling `wheel::force` (`src/wheel.h`): spring-damper
along the ground normal + slip-based tire forces clamped to a friction circle,
with momentum clamps (`stopRate`, `normalStopRate`, issues #770/#862) that keep
the EXPLICIT integration from pumping at coarse substeps. Bullet hull contact
is only a bump-stop backstop ("the wheel hulls ride as bump stops").

A leg rides the same law: ray straight down from the foot (part −Z is vertical
by the mating convention), compression against pad thickness + travel,
`wheel::force` with `brake=1` for footpad grip. The clamps come for free.

Do NOT mark legs as gear (`wheel_radius > 0` / `type == "gear"`):
`type == "gear"` drives the horizontal-spawn heuristic on runways and air
platforms (`src/vehicle.cpp:961-966`) — a legged rocket must stay vertical
there. Legs get their own marker field and predicate.

### Spawn placement is free

`build_ship` (`src/vehicle.cpp:183`) stands the hull on its lowest AABB corner
along the LOCAL VERTICAL, on the pad top + support margin. With legs the feet
are the lowest point, so a legged rocket automatically stands on its feet,
bell clear. No spawn code change.

Margins: pad/building hull margin defaults to 0.1 m (`hullMargin()`,
`src/physics.cpp:36`); raw terrain margin is 0.5 m (`terrainMargin()`). BOTH
are env-tunable at runtime: `OSP_HULL_MARGIN` and `OSP_TERRAIN_MARGIN`
(`envMargin`, `src/physics.cpp:28`). Leg travel 0.4 m works on the pad; on raw
terrain the suspension bottoms out and the hull rides the bump stops — exactly
the gear's documented behaviour — but a lower `OSP_TERRAIN_MARGIN` is a
runtime knob for terrain-landing tests (stage 2), not a code change.

### The catalog is generated, and pinned

- `utils/gen_parts.py` generates `res/data/parts.json` (`--check` gates drift).
  Part rows live in `PARTS`, non-geometric fields in `EXTRA_FIELDS`, and
  per-type derivation in `generate()` (gear branch at line ~628).
- `utils/gen_gear.py` generates `res/meshes/gear.obj` (trimesh; `--out` tree).
- `utils/check_mesh_pins.py` regenerates every claimed-generated asset into a
  temp tree and byte-compares; a new generated mesh needs a `PINS` row (and
  `IN_PLACE` membership if it takes `--out`).
- `tests/test_shipload.cpp:276-308` pins the catalog: mesh box centred on the
  part axis in X/Y (1e-6), `meshHalfLength ≈ height/2` (1%), authored radius ≈
  `meshHalfSpan` (1%), synthesized srf node on the −X mesh face. Gear is
  exempted from the last two (off-centre strut). A splayed leg is inherently
  off-centre in X, so it needs the same kind of documented exemptions plus a
  leg-specific node pin.

## 2. Design

### Geometry (sized for the racer)

Racer stack, all r=1 m: capsule h2 (1149.8 kg) + reaction_wheel h0.25
(100.5 kg) + fuel_tank h2 (83.6 dry + 835.7 prop) + engine h2 (500 kg).
≈2670 kg, 6.25 m tall. Bell exit 5.25 m below capsule centre; tank bottom
3.25 m below.

Leg part frame: **origin at the footpad centre, +Z up** (parallel to the tank
axis when mounted). Splay baked into the mesh:

- vertical drop L = 2.6 m, splay 20° → horizontal offset s = L·tan20° ≈ 0.946 m
- footpad: flat cylinder r=0.3 m, thickness 0.1 m, centred at the origin
  (contact face at z=−0.05)
- strut: 0.1×0.1 m box section from foot to plate, length L/cos20° ≈ 2.767 m,
  tilted 20° toward −X
- plate: 0.3×0.3 m box, 0.08 m thick along X, centred at (−s, 0, L); its
  hull-side face at x = −(s+0.04) ≈ −0.986
- `srf` node: pos = (−0.986, 0, 2.6) (the plate's hull-side face, like the
  gear's node sits on the strut top face), dir = (−1, 0, 0), surface = true

Mounted on the racer's tank at tank-local z=−0.75 (0.25 m above the tank's
bottom edge): contact at ship z=−3.0, foot at ship z=−5.6 → **0.35 m below the
bell exit**. Foot ring radius ≈ 1.0 + 0.986 ≈ 1.99 m.

Mesh box: x ∈ [−0.986, +0.3], y ∈ [−0.3, +0.3], z ∈ [−0.05, 2.64]. Authored
radius ≈ 0.986, height ≈ 2.69 (both derived from the mesh by gen_parts.py).

### Stability (why 20° is a reasonable start)

COM ≈ 1.6 m below capsule centre (full tanks) → ≈4.0 m above the feet.
Tip-over margin = atan(footprint inradius / COM height):

- 4 legs (square, inradius = ring·cos45° ≈ 1.41 m): **≈19°**
- 3 legs (triangle, inradius = ring·cos60° ≈ 1.0 m): **≈14°**

Splay is a generator parameter so it can be tuned after watching it stand.
4× symmetry is the recommended default for the test ship.

### Suspension numbers (starting point, tune in stage 2)

Static load ≈ 2670·9.81/4 ≈ 6.5 kN per leg.

- `leg_spring` 60000 N/m → ≈0.11 m static compression
- `leg_travel` 0.4 m → ≈3.5 m/s touchdown absorbed before bottoming out
  (½mv² vs 4·½k(x_max²−x_static²))
- `leg_damp` 8000 N s/m → ζ ≈ 0.63 (c_crit = 2√(k·m_leg), m_leg ≈ 667 kg)
- explicit-stability headroom (wheel.h's limits): damper h < m/(n·c) = 0.083 s,
  spring h < 2/√(k/(m/n)) = 0.21 s — both outside the coarsest shipped substep
  (0.0556 s at warp 10)
- mass: 40 kg declared per leg

### New def fields (mirror the gear block, `src/shipdef.cpp:831`)

```
leg_foot    m    distance from part origin to the pad contact face along −Z
                 (pad half-thickness; 0.05). > 0 makes the part a leg.
leg_travel  m    crush stroke (> 0 required when leg_foot > 0)
leg_spring  N/m  (> 0 required when leg_foot > 0)
leg_damp    N s/m
```

`PartDef::isLeg() { return leg_foot > 0.0; }` (+ `Part::isLeg()` forward in
`src/part.h`, same shape as `isGearWheel`). Validation mirrors the gear's:
all ≥ 0; foot > 0 requires travel and spring > 0.

### Physics plumbing (~40 lines)

- `Vehicle::legParts` + `groupLegParts()` beside `gearParts`/`groupGearParts`
  (`src/vehicle.h:319`, `src/vehicle.cpp:1837`), called from
  `refreshAeroGeom` the same way.
- `applyGroundForces`: early-out becomes `gearParts.empty() &&
  legParts.empty()`; add a leg loop after the gear loop. Same structure,
  minus steer, with:
  - ray origin = part origin (the foot), down = −(pRot·Z) (vertical by
    construction), reach = leg_foot + leg_travel + 0.2
  - compression = clamp((leg_foot + leg_travel) − dist, 0, leg_travel)
  - `wheel::force` with `in.brake = 1.0` (footpad grip; the friction circle
    and both momentum clamps apply unchanged)
  - forward = part +Y laid into the ground plane (arbitrary for a pad but
    must be non-degenerate; same guard as the gear)
  - clamp denominators = `legParts.size()` (the gear loop keeps
    `gearParts.size()`; a mixed gear+leg hull slightly over-spends the shared
    momentum pot — rare, documented, not worth a second contact pass)
  - the analytic terrain fall-through and building-bubble branches are copied
    verbatim from the gear loop (they are frame logic, not wheel logic)

### Test ship

`res/ships/racer_legs.json` — a copy of `racer.json` plus four legs:

```json
{ "part": "leg", "id": "leg_1", "parent": "fuel_tank_1",
  "attach": "surface", "angle": 0,   "z": -0.75, "stage": 1 }
```

…at angles 0/90/180/270. The `angle`+`z` shorthand resolves the contact on the
parent's mesh ellipse (`src/shipdef.cpp:1116`); z=−0.75 is inside the tank's
±1.0 mesh half-length. `childNode` defaults to the surface node.

**Do not edit `racer.json` in place** — ~500 lines of e2e fixtures reference
it; added mass/drag/height would break goldens.

## 3. Staged plan

### Stage 1 — the part exists and the racer stands on it (prototype)

Work order (each step compiles/pins before the next):

1. `utils/gen_leg.py` (new, model on `gen_gear.py`): trimesh footpad + tilted
   strut + plate; parameters `--drop 2.6 --splay 20 --pad-radius 0.3
   --pad-thick 0.1 --plate 0.3 --plate-thick 0.08 --strut 0.1`; provenance
   header (`# generated by gen_leg.py (...)`) for the mesh-pin gate; `--out`
   tree support via `gen_out.add_args/ship`. Writes `res/meshes/leg.obj`.
   Texture: reuse `textures/gear.png` for v1.
2. `utils/gen_parts.py`: `PARTS` row `("leg", "leg", "meshes/leg.obj",
   "textures/gear.png")`; `EXTRA_FIELDS["leg"]` = mass 40, leg_travel 0.4,
   leg_spring 60000, leg_damp 8000, leg_foot 0.05, and the authored `srf`
   node (pos/dir above); `generate()` branch for ptype "leg" (declared mass,
   radius/height from mesh, leg fields from EXTRA_FIELDS). Regenerate
   `res/data/parts.json`; gate with `python3 utils/gen_parts.py --check`.
3. `utils/check_mesh_pins.py`: add `gen_leg.py` to `IN_PLACE` and a `PINS` row
   for `meshes/leg.obj`. Run `python3 utils/check_mesh_pins.py`.
4. `src/shipdef.h`: the four fields + `isLeg()`; `src/shipdef.cpp`: ctor
   initializer list (line ~34), parse+validate block after the gear's
   (line ~849).
5. `src/part.h`: `isLeg()` forward beside `isGearWheel()` (line ~228).
6. `tests/test_shipload.cpp`: exempt legs (`isLeg()`) from the X/Y centring
   pin (line ~294), the half-length pin (~288) and the synthesized −X node pin
   (~306), each with a comment in the gear exemption's style; add a leg pin:
   srf node pos ≈ (−0.986, 0, 2.6) and dir ≈ (−1,0,0) (1e-3), i.e. the node
   sits on the plate's hull-side face.
7. `src/vehicle.h` + `src/vehicle.cpp`: `legParts`, `groupLegParts()`, the leg
   loop in `applyGroundForces` (section 2).
8. `res/ships/racer_legs.json` (section 2).
9. Verify:
   - `bear -- make` then `make test` (all existing tests stay green; the
     catalog pins now cover the leg).
   - Run: `SDL_VIDEODRIVER=offscreen ./osp --timeout 15 --startship
     racerlegs,res/ships/racer_legs.json,Kerbin,pad` with a temporary
     `[leg]` debug print (per-foot compression + force) to confirm: four feet
     share the load ≈6.5 kN each, compression settles ≈0.11 m, the hull stays
     vertical and does not walk. Remove or keep the print per project style
     (the gear has none; prefer removing after the e2e oracle is settled).
   - One new e2e case (spawn-and-settle; model on
     `e2e/cases/vab/165-vab-pad-neighbour-clear.txt` for the `--startship`
     shape): launch the legged racer on the pad, assert it is still vertical
     and near its spawn point after ~10 s, FORBID `FAILED`/`error:`. Pick the
     oracle from whatever stable print the run shows (e.g. a settle/attitude
     line); keep it to ONE case.
   - Ask the user for a visual check (QWEN.md): legs clear the bell, plates
     flush on the tank, feet flat on the pad.

Acceptance: the legged racer stands still on its four feet on the pad, bell
clear; `make test` green; mesh-pin and parts-check gates green; one e2e case.

### Stage 2 — landing dynamics (local agent)

- Hop-and-land e2e over the legged racer: throttle up briefly (`--sim-press`
  triples), cut, fall back, touch down. Tune `leg_spring`/`leg_damp`/
  `leg_travel` so a 3–5 m/s touchdown settles without bottoming out or
  tipping; re-check the explicit-stability headroom (section 2) after any
  gain change. Regenerate parts.json + gates after tuning.
- Watch the convex-wedge bump stop: the leg's Bullet hull is the convex hull
  of plate+strut+foot, so on tilted contact the strut wedge can touch before
  the pad does. Acceptable (gear-class behaviour); note anything surprising.
- Slope behaviour: the tire law is slip DAMPING, not stiction (wheel.h's
  comment says so explicitly) — a parked lander creeps slowly downhill.
  Document; if it matters, the fix is the one wheel.h names: raise the
  stopRate cap with `max(stopRate*|v|, kStatic*N)`. That is a gear-wide
  change, so it gets its own issue, not a leg-local hack.

### Stage 3 — VAB UX + polish (local agent)

- One e2e case: arm the leg from the palette (`--ui-click "Palette/..."`),
  place it on the tank with 4× symmetry, verify the part count (model on
  `e2e/cases/vab/278-vab-click-places.txt`; `--ui-list MS` dumps the live
  widget registry for writing paths).
- Optional: a leg indicator in the flight UI beside the gear one
  (`src/gameui.cpp:3359`), parts-window prose, a dedicated leg texture.
- Optional: a longer leg variant for the 3 m / 4.5 m engines (drop ≈ 3.6 /
  5.1 m by the same bell-clearance arithmetic). Same code, new generator
  argv + catalog row + pins.

### Stage 4 — deploy/retract (future, own design)

Stowed pose + animation + key binding + save state; drag relief during
ascent. Deliberately out of v1: it needs an actuator/state design the fixed
leg does not. Re-discuss before starting.

## 4. Risks / accepted limitations

- **No stiction** → slow creep on slopes (gear shares this). Flat ground fine.
- **One size** fits the 2 m engine class; bigger engines need longer legs
  (stage 3).
- **Fixed legs** drag during ascent and look odd; deployment is stage 4.
- **Centring-pin exemption**: `meshHalfSpan` symmetrises the leg's off-centre
  box, so surface-attaching ONTO a leg would read a wrong contact ellipse.
  Degenerate case; ignore for v1 (a refusal can come later).
- **Mixed gear+leg hulls** over-spend the momentum clamp slightly (separate
  denominators). Rare; documented in the leg loop comment.
- **Raw terrain** (0.5 m margin > 0.4 m travel): suspension bottoms out, hull
  rides the bump stops — the gear's documented behaviour, not a leg bug.
  `OSP_TERRAIN_MARGIN`/`OSP_HULL_MARGIN` tune this at runtime for tests.

## 5. Commands cheat-sheet

```
bear -- make                          # build
make test                             # unit tests
python3 utils/gen_parts.py --check    # catalog drift gate
python3 utils/check_mesh_pins.py      # generated-art drift gate
python3 e2e/run.py <selector>         # e2e (keep to 1-3 cases per feature)
SDL_VIDEODRIVER=offscreen ./osp --timeout 15 \
  --startship racerlegs,res/ships/racer_legs.json,Kerbin,pad
```

Issue tracking: `python3 utils/tea.py` (new/ls/show/comment/close). File
issues for anything odd found along the way (e.g. the stiction gap if stage 2
confirms it matters).