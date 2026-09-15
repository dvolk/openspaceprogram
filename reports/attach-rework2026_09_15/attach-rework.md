# Part attachment rework — node/surface model + VAB drawing foundation

Date: 2026-09-15
Status: implemented through Phase 3a; Phase 3b-3d (interactive editor) planned.
Supersedes the gap analysis in `reports/part-attachment2026_09_15/` (that report
remains the KSP comparison; this one records what was built and what's next).

Commits: `d6f17e4` (Phase 0), `bd74fe0` (Phase 1), `87e4114` (Phase 2a),
`6314770` (Phase 2b), `684616c` (jet control-surface angles),
`adc9af1` (Phase 3a drawing foundation).

## TL;DR

Attachment went from a procedural `AttachMode {Down,Up,Radial,Side}` + cylinder
`radius`/`height` model to KSP's hybrid, with **one** solver underneath:

- **Stack edges** mate two named **nodes** (`Node{id,pos,dir}`) — position
  coincide, directions anti-parallel, roll about the mating axis.
- **Surface edges** place the child's **surface node** at a contact point +
  outward normal on the parent (KSP `srfAttach`), via the same solver with a
  synthetic parent node.
- `attachNodes()` is the single source of geometry; `attachSurface()` wraps it;
  `attachPose(down/up)` is a thin shim. The procedural `Radial`/`Side` paths are
  deleted. `AttachMode` is now `{Down, Up, Surface}`.

Parts with no explicit `nodes` in JSON get synthesized `top`/`bottom` (at
±height/2) plus a side surface node (at −radius, inward), so every existing
cylinder part and every existing ship def kept working with zero edits, and the
migrations were geometry-exact.

## The model

### Part side (`src/shipdef.h`)
```cpp
struct Node { std::string id; glm::dvec3 pos; glm::dvec3 dir; bool surface=false; };
// PartDef::nodes            (empty in JSON -> synthesizeNodes())
// PartDef::findNode(id)     stack nodes by id
// PartDef::findSurfaceNode() the single surface node
// PartDef::synthesizeNodes() top(+h/2,+Z), bottom(-h/2,-Z), srf(-radius,-X,surface)
```
Explicit `nodes` in `res/parts.json` override synthesis (for off-axis hub ports,
perpendicular radial decouplers, wing roots — none authored yet).

### Ship-def edge (`res/ships/*.json`) — a tagged union
- **Stack:** `parentNode`/`childNode` ids (default down: parent `bottom`/child
  `top`; up: parent `top`/child `bottom`), `angle` = roll about the mating axis.
- **Surface:** `attach:"surface"` + contact, either explicit
  `point`/`normal` in the **parent's local frame** (what the editor raycast
  writes) or the cylinder shorthand `angle`[+`z`] on the parent radius;
  `roll` about the normal; `offset` along it; `childNode` defaults to the
  child's surface node.
- Node ids and contacts are validated at load (a typo is a load error, not a
  build-time null deref).

### Solver (`src/shipdef.cpp`)
`attachNodes(parentPos, parentRot, parentNode, childNode, roll, offset)`:
coincide node positions (pushed by `offset` along the parent node dir),
anti-align directions via a minimal-arc `rotationFromTo`, then `roll` about the
mating axis. For synthesized axial nodes the arc is the identity, so a stack
child inherits the parent's orientation — which is exactly what made the
synthesis a drop-in migration for the old `Down`/`Up`.

`attachSurface(...)` builds a synthetic parent node `{pos: contact, dir: normal}`
and calls `attachNodes`. A cylinder child's side node mated at clock θ lands its
center at `rP + rChild` along θ — identical to the old procedural `Side`, so the
`side`→`surface` ship migration was a pure string change.

### Why the wing migration was safe
`applyAeroForce` builds ONE ship-level flow frame from the **root** part's axes;
a lifting surface contributes only scalars (`lift_area`, `cl`, `control_area`,
...) plus its **position**. Re-orienting a wing changes rendering, the collision
hull and the inertia tensor — not lift/control direction. And the migration
preserved position exactly, so `ri` (control leverage/sign) was unchanged.
`wing.obj`'s local frame is already the airplane body frame (X=span, Y=normal,
+Z=apex/forward), so identity-relative surface attach yields the correct flat
wing (span lateral, chord fore-aft). On a nose-up pad it *looks* vertical; in
level flight it is flat — verified by screenshot + reasoning, and the pad
appearance is inherent to nose-up parking.

## Per-phase summary

- **Phase 0 (`d6f17e4`)** — collapsed three attach paths onto `attachPose`;
  removed the weld-era `parentAnchor`/`childAnchor` + coincidence throw.
- **Phase 1 (`bd74fe0`)** — stack nodes + synthesis + node-ref ship edges;
  `attachNodes`; `heavy_asp` needed zero edits (default-down maps to bottom/top).
- **Phase 2a (`87e4114`)** — surface attach (`attachSurface`, point+normal or
  cylinder shorthand); `AttachMode::Side` deleted; all 8 `side` ships migrated
  (string change, geometry identical).
- **Phase 2b (`6314770`)** — wings/rudders/elevators migrated `radial`→`surface`;
  procedural `Radial` deleted; `--radial-test radial` mode removed (redundant).
- **jet fix (`684616c`)** — rudders→90/270 (vertical), elevators→0/180 (flat);
  they were role-swapped. Control signs unaffected (moments come from fore/aft
  `ri.z`). Glider was already correct.
- **Phase 3a (`adc9af1`)** — physics-free `DrawModelAt(mesh,shader,texture,
  matrix, DrawOpts{alpha,tint})` extracted from `Body::DrawAt`; `partsShader`
  gains `u_alpha`/`u_tint` (slots 4/5); ghost = alpha<1 (blend on, depth-write
  off); plume draws pin shadow/alpha/tint.

## Verification

`make test` green throughout (full suite, 0 failures). e2e: spin-regression,
fuel-link-symmetric, asparagus-drain/flight, dock, jet-vtol, thrust-ascent all
pass after each phase. Every migrated ship headless-loads clean. Pad screenshots
confirm opaque rendering unchanged by 3a and the wing/control-surface
orientations.

## Deliberately left / deferred

- **Wing UVs are degenerate** (`vt 0 0`) so wing/rudder/elevator textures sample
  one texel — deliberate for now (quick test parts).
- **`attachRules` / node `size` / per-node `crossfeed`** not stored yet; they
  matter for editor gating and the fuel rework, not for data-driven ships.
- **Perpendicular surface attach** (a real radial decoupler, booster axis
  outward) needs an explicit surface node whose `dir` runs along the part axis;
  no catalog part has one yet. Returns with the editor (3b), which can author
  and place them.
- **`central_radial.json`** migrated but remains a nonsensical ship (unreferenced
  anywhere); candidate for deletion.

## Phase 3 plan (VAB editor)

- **3a ✅** drawing foundation (above).
- **3b** build-tree representation (physics-free `ShipDef`-like tree, poses from
  the node solver — no Bullet); hangar scene drawn via `DrawModelAt`; orbit
  camera; raycast part-picking against part hull shapes built WITHOUT rigid
  bodies (`BuildPartHull` gives the shape); port markers from each part's nodes.
- **3c** palette + ghost preview + snap: hover a port, ghost at
  `attachNodes`/`attachSurface` pose, click to add to the build tree.
- **3d** delete / re-orient / re-stage; serialize the build tree back to the
  `ShipDef` JSON schema (inverse of `load_ship_def`); Launch = `build_ship()`.

Architecture (per `reports/vab-drawing2026_08_24`): the editor edits a
**physics-free build tree**; Launch converts it to the flight `Vehicle` via the
existing `build_ship()`. Keeps the ghost preview fast and save trivial.
