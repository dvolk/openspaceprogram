# Part attachment — OSP vs KSP, gap analysis and next steps

Snapshot of the ship part attachment system as of 2026-09-15 (commit
`f1a33ba`, "ocean: separate water surface mesh with specular and waves").
Written to compare our attachment model against stock Kerbal Space Program,
identify what's present, what's missing, what's actively risky, and to scope
an incremental path forward.

Code references are `src/part.h` (the part instance + tree edge),
`src/shipdef.h` / `src/shipdef.cpp` (the part catalog, attach schema, pose
solver), `src/vehicle.h` / `src/vehicle.cpp` (ship assembly + physics body),
`src/body.h` / `src/physics.cpp` (rigid body / collision hull),
`src/game.cpp` (docking + staging), `src/pick.cpp` (part picking), unless
noted. The KSP reference this is measured against is in
`ksp-attachment-reference.txt` alongside this file.

## TL;DR

Our attachment model is **procedural, not node-based**. A part carries only
two geometric scalars (`radius`, `height`) and a ship edge carries an
`AttachMode {Down,Up,Radial,Side}` + `angle` + `offset`; `attachPose()`
solves the child pose from those at build time.

KSP is a **hybrid** of two attach paths, and it's worth being precise about
both because our gaps map onto them separately:

- **Stack attach (node ↔ node).** Each part declares a table of named,
  directional, sized `node_stack_*` nodes (top, bottom, side, ...). The editor
  shows them as gizmos and snaps node-to-node: positions coincide, direction
  vectors go anti-parallel. This is the "you can see nodes and parts snap to
  them" path.
- **Surface attach (node ↔ anywhere on a collider).** Each part also has a
  single `node_attach`. For surface attach KSP raycasts onto the *parent's*
  collider and places the child's `node_attach` at that **arbitrary hit point**
  — no predefined node on the parent (the parent only needs the
  `allowSrfAttach` flag); the child is rotated so its `node_attach` direction
  aligns to the surface. This is the "attach anything anywhere" path that wings,
  struts, panels, lights and radial decouplers use.

So the accurate statement is: **the child is always anchored by one of its own
nodes, but the parent side is either a predefined stack node or a free point on
the parent's surface.** "Anywhere" means anywhere on the parent's *surface*,
governed by the child's `node_attach` — not literally node-free. Our
`Radial`/`Side` modes are a crude stand-in for surface attach, but only at
`radius` distance on a cylinder; we have no raycast-onto-collider free placement
at all. That hybrid is the root of almost every gap below: no real surface
attach, no size classes, no `attachRules` compatibility gating, no symmetry, and
a hard assumption that every part is a +Z-aligned cylinder.

Physics-wise we are **simpler and more stable than KSP by design**: the whole
ship is one `btRigidBody` with a `btCompoundShape` (one convex hull per part).
No joints, no welds, no per-part bodies, no noodling. Staging already matches
KSP semantics well — a decoupler drops itself plus its child-side subtree,
which becomes a real flying vessel.

The recommended next step is **not** to chase KSP's joint physics (ours is
better for stability). It is to **introduce a real attach-node table** as the
authoring/pose layer, which fixes wing orientation, removes the cylinder
assumption, and is the foundation every later feature (surface attach, VAB,
symmetry) depends on. The single-body simulation can stay untouched.

---

## What we have

### Part tree (topology)

- `Part* parent` (`part.h:41`) — topology only, carries no physics handle,
  the authoritative source for the tree.
- `Vehicle::rootPart()` (`vehicle.cpp:694`) finds the part with
  `parent == nullptr`.
- Def-time, `ShipPart::parent` is an **index into an earlier entry**
  (`shipdef.h:373`, resolved `shipdef.cpp:373-386`). Construction order means
  **cycles are structurally impossible**.

### Attachment vocabulary (procedural)

`enum class AttachMode { Down, Up, Radial, Side }` (`shipdef.h:351-364`):

```
Down    face-to-face on the parent's -Z face (a plain stack)
Up      face-to-face on the parent's +Z face
Radial  child axis perpendicular; child's base face on the parent's side
Side    parallel axes, side by side
```

A ship edge carries `attach` (mode) + `angle` (degrees around the parent's
stack axis, 0 = parent +X) + `offset` (gap in metres). `attachPose()`
(`shipdef.cpp:468-540`) returns `AttachPose { childPos, childRot,
parentAnchor, childAnchor }`. The whole geometric vocabulary of a part is
`radius` + `height` (`shipdef.h:207-215`), documented as "+Z = stack axis:
radius = cross-section (x/y extent 2r), height = extent along the stack axis."

### Physics — one compound rigid body

`Vehicle::rebuildCompound()` (`vehicle.cpp:491-577`) builds a single
`btCompoundShape` with one `btConvexHullShape` child per part (referenced, not
copied), calls `calculatePrincipalAxisTransform`, re-bases children into the
COM/principal frame, and creates **one** `btRigidBody`. The header
(`vehicle.h:120-160`) is explicit: *"There are no per-part rigid bodies and no
welds between them."* A part's world pose is always derived:
`partWorldPose = frameS() * localPose` (`vehicle.cpp:701-706`).
`compoundParts[i]` maps a collision child index back to a `Part`
(`pick.cpp:96-108`). Re-called on staging, docking, spawn, and when a burn
shifts mass enough (`refreshCompound`, tolerances `kComRebuildTol=0.01 m`,
`kMassRebuildFrac=1e-3`); `checkCompoundInvariants()` (`vehicle.cpp:608-692`)
re-asserts mass properties on every rebuild. The Bullet world has zero gravity
(`physics.cpp:170-190`); gravity/tides are applied per-part as forces.

### Staging / decoupling

- `PartDef::decoupler` (`shipdef.h:251`) marks a staging boundary; decouplers
  are forced `fuel_barrier` (`shipdef.cpp:249-251`).
- `Vehicle::droppedPartsAtStage(stage)` (`vehicle.cpp:1803-1830`) collects each
  decoupler on that stage plus its **child-side subtree** (BFS over
  `Part::parent`). The decoupler itself is dropped — *"flies off with the stage
  like a KSP separator."*
- `Game::stage()` (`game.cpp:844`) fires decouplers shallowest-first; each
  dropped subtree becomes a **new flying ship** via `extractSubtreeAsShip`
  (not deleted).

### Docking (runtime attach)

`PartDef::docking_port` (`shipdef.h:258`) + `Game::updateDocking`
(`game.cpp:705-800`) runs a proximity/alignment capture test, then
`Vehicle::absorbShip` (`vehicle.cpp:1831`) merges two ships into one rigid body
and records a `DockSeam{port, root, name}` (`vehicle.h:825-829`) so `undock()`
(`game.cpp:805-831`) splits it back via `extractSubtreeAsShip`.

### Fuel plumbing

`Vehicle::buildFuelGroups()` (`vehicle.cpp:862-894`) does connected-component
grouping over `Part::parent`, treating `fuel_barrier` parts as walls;
`fuel_link` virtual parts (`shipdef.h:264-269`) add one-way directed edges
(`Vehicle::FuelLink{from,to}`), resolved and cycle-checked in `build_ship`
(`vehicle.cpp:135-200`).

### Data files

- `res/parts.json` — flat `{"parts":[...]}` catalog, 60+ entries.
  Attachment-relevant fields: `radius` (default 1.0), `height` (default 2.0).
  **There is no `node_*`, no `srfNName`, no attach-point list, no size class,
  no allow/disallow.**
- `res/ships/*.json` — carry the edges. Per-part fields (`shipdef.h:88-108`,
  parsed `shipdef.cpp:330-460`): `part` (catalog name), `id`, `parent` (id of
  an earlier part), `attach`, `angle`, `offset`, `stage`. Fuel links use
  `from`/`to`.

---

## Gap analysis vs KSP

| Area | KSP | OSP | Verdict |
|---|---|---|---|
| Attachment primitive | **Hybrid**: stack nodes (`node_stack_*`) mate node-to-node; surface attach places the child's single `node_attach` at a free raycast point on the parent collider | Two scalars (`radius`,`height`) + procedural `AttachMode`/`angle`/`offset` | **Missing** — biggest divergence |
| Surface attach | Child `node_attach` placed at raycast hit anywhere on parent collider, oriented to surface (wings, struts, panels, lights, radial decouplers); parent needs only `allowSrfAttach` | Approximated by `Radial`/`Side` keyed off `radius`; cylinder-only, no free placement | **Missing** |
| Size classes | `bulkheadProfiles` size0..3 (0.625/1.25/2.5/3.75 m) gate mating | None | **Missing** |
| `attachRules` | `stack, srfAttach, allowStack, allowSrfAttach, allowCollision` compatibility gating | None — anything bolts to anything | **Missing** |
| Symmetry / mirror | `symMethod = Radial\|Mirror`, `mirrorRefAxis`, `mir`, `sym=` counterpart links, autostrut | None; bilateral symmetry hand-authored via `angle` | **Missing** |
| Editor (VAB/SPH) | Full interactive placement + snap + symmetry | None. `attachPose()` documented as "the same function the *future* VAB snap uses" | **Missing** |
| Struts / fuel lines | `CompoundPart` surface-attached child that adds an **extra joint** between two existing parts without changing the tree | `fuel_link` is a virtual directed edge, not a physical strut | **Partial** |
| Massless-part physics | Massless parts (`PhysicsSignificance=1`) skipped by joints; mass folded into nearest physical ancestor | Every part gets a hull in the compound | **Missing** (moot under single-body) |
| Joint model | One Unity `FixedJoint` per attachment; `breakingForce`/`breakingTorque` failure thresholds; soft constraints -> "noodling" | Single rigid compound body; no failure | **Different — OSP is more stable** |
| Part tree | Cross-references between part keys in `.craft` (`link`, `attN`, `srfN`, `sym`); **no `parent = N`** | `Part* parent` pointer + def-time index | Equivalent topology |
| Staging | `istg`/`dstg`/`sepI`/`sqor`/`sidx`; decoupler splits vessel below `explosiveNodeID` into a new vessel | `droppedPartsAtStage` -> `extractSubtreeAsShip` | **Matches semantics well** |
| Docking | `ModuleDockingNode`, `nodeType` size0..2 (only identical mate), `referenceAttachNode` | Proximity/alignment capture -> `absorbShip` / `DockSeam` | Present, own convention |

### Where OSP is genuinely better

- **Single compound rigid body** is far more stable than KSP's noodling joint
  stacks — no jitter, no joint explosion on long vessels. Keep it.
- **Staging-as-subtree-extraction** is clean and already correct: decoupler
  flies off, child subtree becomes a real flying vessel.
- **Construction-order parents** make cycles impossible by design.

---

## What's terrible (or actively risky)

1. **`radius`/`height` assume a +Z-aligned cylinder.** Wings, control surfaces
   and any asymmetric part reuse them as a bounding approximation. Wing/rudder
   parts in `parts.json` are authored as 1×2 "cylinders" despite flat meshes —
   **attachment faces may not match visible geometry.** This is a
   correctness/visual bug that will bite the moment an editor exists.
2. **Three parallel attach implementations that can drift.**
   - `attachPose()` (data-driven, used by `build_ship`)
   - `attachDown/attachRadial/attachSide` (`vehicle.cpp:761-803`, hardcoded
     angle 0, used only by test builders)
   - `radialtest.cpp`'s hand-rolled `link()` with literal poses
   Same geometry, three copies.
3. **Vestigial anchors.** `AttachPose::parentAnchor/childAnchor` and the
   coincidence check (`shipdef.cpp:531-537`) are leftovers from the deleted
   weld system. They constrain nothing now; `radialtest.cpp:104-109` literally
   names its anchor params `/*paAnchor*/`, `/*pbAnchor*/` and ignores them.
4. **Docking uses its own ad-hoc ±Z-face convention** (`game.cpp:757-770`),
   independent of the `AttachMode`/anchor model. Another one-off.

---

## Next steps (scoped, incremental)

### Phase 0 — cheap cleanups (low risk, do first)
- Collapse the three attach paths into one: make the test builders and the
  `attachDown/attachRadial/attachSide` wrappers call `attachPose()`.
- Delete the vestigial anchor fields + coincidence check, or keep them only if
  a future staging-cut/VAB genuinely needs the coincidence point.

### Phase 1 — stack nodes (the load-bearing change)
- Add a `nodes[]` table to `PartDef`:
  `{ id, pos(vec3), dir(vec3), size(int), method, crossfeed, rigid }`, parsed
  from `parts.json`. Keep `radius`/`height` as fallback for parts that don't
  declare nodes.
- Change ship-def edges from `AttachMode` to **node-id → node-id** references
  (parent node, child node). `attachPose()` becomes: coincide the two node
  positions, anti-align directions, resolve roll. This unlocks non-axial hubs
  (4-way/6-way) and correct wing orientation.
- Add `attachRules` gating so incompatible parts can't connect.

### Phase 2 — surface attach + size classes
- Implement the **second** attach path: a child's `node_attach` placed at a
  free point on the parent (raycast onto the collider in an editor, or an
  explicit surface hit point + normal in a ship def), oriented to the surface —
  prerequisite for wings/struts/radial parts done right. This is distinct from
  Phase 1's node-to-node stack mating; both paths coexist, as in KSP.
- Add `bulkheadProfiles`/size classes for editor compatibility.

### Phase 3 — editor (VAB)
- Build the interactive placement loop on the node system: pick a part,
  raycast onto existing nodes, snap, preview pose via `attachPose()`. Add
  symmetry (radial/mirror) here, since it's an editor concern.

### Phase 4 — extras
- Struts/fuel-lines as compound parts that add cross-joints.
- Massless-part physics collapse, if we ever move off the single-body model.

### Recommendation
Do **Phase 0 + Phase 1** next. The stack-node table is the load-bearing change:
it removes the cylinder assumption for stacked parts, enables non-axial hubs and
correct node-driven orientation, and is the foundation every later phase depends
on. Wings/struts/radial parts specifically need **Phase 2**'s surface-attach path
(free placement on the parent), so treat Phase 1 and Phase 2 as the two halves of
matching KSP's hybrid. The single-body physics can stay throughout; nodes are
about *authoring and pose*, not about how we simulate.

---

## Appendix — KSP attachment model (condensed)

Full detail with field tables, examples and sources is in
`ksp-attachment-reference.txt`. Headlines:

- **Node syntax:** `node_<id> = px,py,pz, ox,oy,oz, [size], [method],
  [crossfeed], [rigid]` (6/7/9/10 fields, never 8). Orientation is a direction
  vector, not required normalized; node rotation =
  `Quaternion.LookRotation(orientation)`. `size` defaults to 1.
- **Stack vs surface:** `attachRules = stack, srfAttach, allowStack,
  allowSrfAttach, allowCollision`. Surface attach uses the *child's*
  `node_attach` placed at a raycast hit on the parent. Craft records
  `attm = 0` (stack) / `1` (surface).
- **`.craft` hierarchy:** no `parent = N`; the tree is cross-references
  (`link`, `attN`, `srfN`, `sym`) between `<partName>_<persistentId>` keys.
  `attN` is bidirectional. Staging via `istg`/`dstg`/`sepI`/`sqor`/`sidx`.
- **Orientation:** child node direction is anti-parallel to parent's, positions
  coincide; roll about the node axis is unconstrained by the vectors and
  resolved by `secondaryOrientation`/editor roll-snap. Symmetry via
  `symMethod = Radial|Mirror`.
- **Joints:** one Unity `FixedJoint` per attachment, only between parts with
  their own rigidbody — the **physics tree is a collapsed version of the part
  tree** (massless parts folded into the nearest physical ancestor).
  `breakingForce`/`breakingTorque` are failure thresholds, not stiffness;
  joints are soft, hence noodling. Struts (`CompoundPart`) add an extra joint
  without changing the part tree.
