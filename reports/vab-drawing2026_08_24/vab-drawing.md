# Drawing ships & parts — a step toward a KSP-like VAB

Date: 2026-08-24
Status: investigation / design proposal (no code written)

## TL;DR

The hard parts of a KSP-style VAB are **already in the codebase**: the part
catalog, the ship-as-a-tree data model, and — crucially — a pure, unit-tested
`attachPose()` function that computes exactly where a child part sits relative
to its parent for every attach mode. `attachPose()` is even documented as
"the same function the future VAB snap uses."

What is **not** there yet is the *authoring* rendering layer: a way to draw a
ship (or a single part, or a translucent "ghost" part) at an arbitrary pose
**without** a Bullet rigid body, a way to tint/alpha a part for selection and
ghost preview, and a way to **pick** which part/face the mouse is over.

The good news: all of that is small, additive work that reuses the existing
`Mesh`/`Texture`/`Model`/`Shader`/`OrbitCamera` machinery. The recommended
first step (this report) is to **add a physics-free "draw a part at a pose"
path + ghost/selection shader state + part caching**, which makes a static
ship drawable in a hangar and a single part drawable in a palette. Picking and
edit/save are the following steps.

---

## 1. What "drawing" means for a VAB

A VAB (Vehicle Assembly Building) is a hangar where the player:

1. Orbits/zooms a camera around a ship sitting on a pad.
2. Sees the ship **as a collection of selectable parts**.
3. Picks a part from a **palette** (catalog of part types).
4. Clicks a **face/port** on an existing part; the new part **snaps** into the
   correct pose (a translucent *ghost* previews it first).
5. Deletes / re-orients / re-stages parts.
6. Saves the result as a ship def and "launches" it.

So "drawing ships and parts" in the VAB context is *not* the same as the
current flight rendering. It needs to support:

| Capability | Flight rendering (today) | VAB rendering (goal) |
|---|---|---|
| Ship on pad | yes (physics bodies) | yes, **static** |
| Single part in isolation (palette) | no | **yes** |
| Translucent "ghost" preview part | no | **yes** |
| Highlight a selected part | no | **yes** |
| Mark available attach points/ports | no | **yes** |
| Hangar / pad environment | pad only | pad + (optional) hangar |
| Orbit camera around build site | yes (`OrbitCamera`) | **reuse** |
| Pick a part/face under the mouse | **no** | **yes** |

The camera and the part-mesh rendering are already reusable. Everything marked
**yes (goal) / no (today)** is what this report scopes.

---

## 2. Current state: how ships & parts are drawn today

### 2.1 Data model (GL-free, unit-tested) — `src/shipdef.h`, `src/shipdef.cpp`

- `PartDef` — one catalog entry: `name`, `mesh`, `texture`, `mass`,
  `radius`, `height`, plus behavior fields (`torque`, `fuel_rate`,
  `exhaust_velocity`, `capacity`). Behavior is *field-driven*, not label-driven.
- `ShipDef` — a **tree** of `ShipPart` in construction order. Part 0 is the
  root; each other part names its `parent` (must be defined earlier — this is
  what keeps it a tree and makes cycles impossible).
- `AttachMode` — `Down` / `Up` / `Radial` / `Side`.
- `ShipPart` — `{ part (catalog name), id, def*, parent (index), attach,
  angle (deg about parent stack axis), offset (m gap), stage }`.
- `attachPose(parentPos, parentRot, parentDef, childDef, mode, angle, offset)`
  → `AttachPose { childPos, childRot, parentAnchor, childAnchor }`.
  Pure `glm` double-precision math. Enforces the invariant that the weld
  anchors coincide in world space. **This is the VAB snap function.**
  (`shipdef.h:190`, impl `shipdef.cpp:267`.)
- `PartsCatalog` + `load_parts_catalog()`, `load_ship_def()` — JSON parse +
  validation. Tested headless by `tests/test_shipload.cpp` (includes
  `attachPose` geometry across every mode/angle/offset and the anchor
  coincidence invariant).

**Key point:** the entire "where does a part go" problem is already solved and
tested, independent of GL and Bullet. A VAB needs to *call* `attachPose()` —
not re-derive it.

### 2.2 Runtime representation — `class Vehicle` (`src/main.cpp:804`)

One **Bullet rigid body per part**, welded by 6-DOF constraints:

- `std::vector<Body*> parts` — the parts (each a `Body`).
- `std::vector<const PartDef*> partDefs` — parallel to `parts`.
- `std::vector<int> partStages` — parallel to `parts`.
- `std::vector<void*> constraints` + `constraintLinks` — the welds and which
  part indices they join (used by staging to cut the right links).
- `attach()` / `attachDown()` / `attachRadial()` / `attachSide()` — weld a
  child to a parent at the local anchor points (`main.cpp:862`+).
- `separateStage()` — the inverse: cut welds, delete parts, remap indices.
  This is a working example of *mutating* a built ship's part set.

### 2.3 Build: ShipDef → Bodies — `build_ship()` (`src/main.cpp:1590`)

This is the function that turns a `ShipDef` into the `Vehicle`:

1. Computes every part's pose in a canonical frame by chaining `attachPose()`
   from the root (`main.cpp:1608-1620`).
2. Finds the ship's lowest point along the stack axis and shifts it so the
   bottom sits on the pad top (`main.cpp:1622-1640`).
3. **Per part**: `new Mesh; mesh->FromFile(res/<mesh>)`,
   `load_texture(res/<texture>)`, `new Model; model->FromData(...)`,
   `create_body(model, ..., mass, false)`, `setPosRot(...)`, then
   `ship->attach(...)` or `ship->setRoot(...)` (`main.cpp:1642-1660`).
4. `ship->init()` — seeds propellant, builds thruster/wheel vectors.

**Observation (perf/memory):** step 3 reloads the *same* `.obj`/`.png` from
disk and re-uploads the same GPU buffers for **every instance** of a part type.
A ship with 7 tanks loads `reaction_wheel.obj` 7 times and makes 7 GPU
vertex/VAO copies. For a VAB (many instances of the same part + ghost + palette
previews) this should be **cached per part type** (see §5.4).

### 2.4 Per-part draw — `Body::Draw()` (`src/body.h:39`)

```cpp
void Draw(const Camera* camera, glm::vec3 & sunlightVec, float shadow,
          const glm::dmat4 &xform = identity) {
    UpdateModelMatrix();                      // reads the Bullet transform
    ModelView   = View * xform * model_matrix;
    MVP         = Projection * ModelViewFloat;
    model->shader->Bind();
    setUniform_mat4(0, MVP);  setUniform_mat4(1, ModelFloat);
    setUniform_vec3(2, sunlightVec);  setUniform_vec1(3, shadow);
    glBindTexture(... model->texture->id);
    model->mesh->Draw();
}
```

`Vehicle::Draw()` (`main.cpp:1278`) loops over `parts` and calls `part->Draw()`
per part, computing a per-part terrain shadow. So **each part is one draw call**
that re-binds the (shared) `partsShader`, sets 4 uniforms, binds its texture,
and draws its mesh.

`Model` is just `{ Mesh*, Shader*, Texture* }` (`src/model.h`). `Mesh` owns the
VAO/VBO and exposes `vs`/`is` (CPU copies used for the Bullet hull) —
`src/mesh.h`. `Shader` is a small uniform/index wrapper (`src/shader.h`).

**Observation:** `Body::Draw` *requires* a `btRigidBody` (it reads the pose from
Bullet via `UpdateModelMatrix()`). There is **no** "draw this `Model` at this
model matrix" path. A VAB ghost / palette part / static ship part has no
Bullet body, so it cannot be drawn with the current code. This is the single
most important gap for *drawing* (see §5.1).

### 2.5 Camera — `src/camera.h`, `src/camera.cpp`

- `OrbitCamera` — orbits a `focusPoint`, `Follow(p)` each frame, RMB-drag to
  look, wheel to zoom. This is **exactly** the VAB "look around the ship"
  camera. Reuse as-is; just `Follow()` the ship's pad position.
- `FreeCamera` — 6-DOF fly-through. Useful for walking around a hangar.
- Toggle with `C` (`main.cpp:3164`).

### 2.6 Pad / environment — `StaticBuilding` (`src/main.cpp:1882`)

A `StaticBuilding` is a static (non-dynamic) `Body` (the `space_port.obj`
pad) that draws itself only when the active ship is on its parent body. It's
the existing "static scenery" pattern — a hangar wall/floor would be another
`StaticBuilding` (or a plain `Model` drawn at a fixed matrix, once §5.1 exists).

### 2.7 UI — ImGui (SDL2 + OpenGL3 backend)

Already integrated (`main.cpp:2458`). Many windows exist (VESSEL, SHIP PARTS,
RESOURCES, SHIPS list, etc.). A VAB "parts palette" and "part properties" panel
are just more ImGui windows. The `SHIPS` list window (`main.cpp:3915`) already
shows the `Selectable()` pattern for list UI.

### 2.8 What does NOT exist

- **No picking / raycast.** `grep` for `rayTest|picking|intersect|frustum` in
  `src/` returns nothing. We cannot currently tell which part is under the
  mouse. (Bullet *does* build a `btConvexHullShape` per part —
  `physics.cpp:246` — which is reusable for picking, §6.)
- **No per-draw alpha/tint.** `partsShader` has a `shadow` uniform but no
  alpha or color-tint uniform, so ghost/selection highlighting is not possible
  without a shader change (§5.3).
- **No "draw model at matrix"** (§2.4).
- **No part-type mesh/texture cache** (§2.3).
- **No hangar geometry** (only the pad).

---

## 3. Gap analysis (what "drawing" still needs)

Ordered by dependency (each step builds on the previous):

1. **Draw a `Model` at an arbitrary matrix** (no Bullet body) — the foundation.
2. **Cache `Mesh`+`Texture` per part type** — makes (1) cheap and shared.
3. **Alpha + tint uniforms** in `partsShader` — ghost & selection.
4. **Draw a `ShipDef` (or `Vehicle`) statically** — a hangar-ready ship.
5. **Draw a single catalog part in isolation** — palette preview.
6. **Draw attach-point / port markers** — where parts can snap.
7. **Picking** (mouse → part → face) — the interactive glue.
8. **Ghost preview + snap** (call `attachPose`) — the actual VAB action.
9. **Edit + save** (mutate `ShipDef`, serialize back to JSON).

Steps 1–6 are the "drawing" scope this report focuses on. Steps 7–9 are the
subsequent VAB steps (sketched here so the drawing design supports them).

---

## 4. Recommended architecture: separate *build* from *flight*

Two representations, one source of truth:

- **Build representation (authoring):** a `ShipDef`-like tree of part
  instances with poses from `attachPose()`. **No Bullet, no physics.** This is
  what the VAB edits and draws. It is essentially the existing `ShipDef` +
  `attachPose()` — already present and tested.
- **Flight representation (runtime):** the existing `Vehicle` (Bullet bodies,
  welds, thrust, staging), built *from* the build representation by the
  existing `build_ship()`.

The VAB edits the **build** representation; "Launch" runs `build_ship()` to
produce the `Vehicle` and hands control to the flight loop. This:

- Reuses `attachPose()` (the snap), `build_ship()` (launch), `OrbitCamera`
  (view), `Mesh/Texture/Model` (draw) — all existing.
- Keeps the authoring preview **fast** (no Bullet hull construction, no
  solver), which matters because a VAB redraws the ghost every mouse move.
- Makes "save" trivial: serialize the build tree back to the `ShipDef` JSON
  schema (the inverse of `load_ship_def()`).

> Alternative (less clean): treat the existing physics `Vehicle` *as* the VAB
> model and add/remove/re-weld parts on it (the `separateStage()` / `attach()`
> code shows it's possible). This avoids a second representation but forces the
> authoring preview to carry Bullet bodies and makes ghost drawing awkward.
> Recommended only if the team wants to minimize new code and accept the cost.

---

## 5. Concrete drawing design

### 5.1 `DrawModelAt()` — draw a `Model` at an arbitrary pose (foundation)

Extract the matrix/uniform/texture logic out of `Body::Draw()` into a free
function that takes an explicit model matrix (and ghost/selection state).
`Body::Draw()` becomes a thin wrapper that feeds it the Bullet matrix.

```cpp
// src/draw.h (new)
struct DrawOpts {
    float  alpha = 1.0f;        // 1 = opaque, <1 = ghost
    glm::vec4 tint = glm::vec4(1,1,1,1);  // selection highlight color
    bool   outline = false;     // (optional) selected-part outline
};

// Draw one Model (mesh+shader+texture) under an explicit model matrix.
// No Bullet body required. Reused by: static ship parts, ghost preview,
// palette part preview, hangar scenery, port markers.
void DrawModelAt(const Camera* cam, Model* model,
                 const glm::dmat4& modelMatrix,
                 const glm::vec3& sunlight, float shadow,
                 const DrawOpts& opts = {});
```

- `Body::Draw()` = `DrawModelAt(cam, model, xform * model_matrix, light, shadow)`.
- A **ghost** = `DrawModelAt(cam, cachedModel, ghostPose, light, 1.0f,
  {alpha=0.4f})`.
- A **selected** part = `{tint = (1, 0.8, 0.2, 1)}` or an outline.
- A **palette preview** = `DrawModelAt(cam, cachedModel, identity, light, 1.0f)`
  inside a small viewport (or rendered to an icon texture, §5.5).

This is the single highest-leverage change: it unblocks every other drawing
feature and is small.

### 5.2 Port model = `attachPose` parameters

Don't invent a new port format. A **port** on a parent part is exactly a
`(mode, angle, offset)` triple — the inputs to `attachPose()`:

- **Down** port = the parent's −Z face (child stacks below).
- **Up** port = the parent's +Z face (child stacks above).
- **Radial** port = a point on the parent's side at `angle` (child axis ⊥).
- **Side** port = a point on the parent's side at `angle` (child axis ∥).

So "the ports available on part P" = {Down, Up} ∪ {Radial@θ, Side@θ : θ ∈
discrete set}. For the UI, discretize the ring (e.g. every 45°) or allow a
continuous angle via drag. The snap is then simply:

```cpp
AttachPose ap = attachPose(P.worldPos, P.worldRot, *P.def, *newDef,
                           mode, angleDeg, offset /*0*/);
// ap.childPos / ap.childRot is the ghost pose.
```

This keeps a single source of truth for geometry (the tested `attachPose()`),
and matches the existing `ShipDef` schema exactly, so a placed part serializes
straight into a `ShipPart` (`part`, `parent`, `attach`, `angle`, `offset`,
`stage`).

### 5.3 Shader: add alpha + tint

`res/partsShader.fs` currently multiplies `tex_color * light * shadow`. Add two
uniforms:

```glsl
uniform float u_alpha;   // default 1.0
uniform vec4  u_tint;    // default (1,1,1,1)
...
gl_FragColor = tex_color * clamp(dot(...), min, max) * shadow * u_tint;
gl_FragColor.a = u_alpha;   // needs blending enabled when alpha < 1
```

- Register `u_alpha`, `u_tint` in the `partsShader` uniform list
  (`main.cpp:2477` `registerUniforms`).
- In `DrawModelAt()`, set them from `DrawOpts`; when `alpha < 1`, enable
  `glBlend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)` and disable depth-write for
  the ghost pass (standard translucent ordering: draw opaque parts first, then
  ghosts).
- Backwards-compatible: defaults (1.0, white) reproduce today's output.

### 5.4 Cache `Mesh`+`Texture` per part type

`build_ship()` and the VAB both draw the same part type many times. Cache the
GPU resources once per catalog part (keyed by `PartDef.name` or by
`(mesh,texture)` pair) and reuse the `Model`/VAO:

```cpp
// e.g. a PartAssets cache owned by the catalog / app:
struct PartAssets { Mesh* mesh; Texture* tex; Model* model; };
std::map<std::string, PartAssets> g_partAssets;   // keyed by PartDef.name
```

- First use of a part type: load + upload once.
- Every instance / ghost / preview: `DrawModelAt()` with the shared `Model` —
  just bind the shared VAO + texture and draw.
- Benefits: less startup time, less GPU memory, and the ghost/preview are
  *free* (no extra upload). This is the "performance is very important" win
  the project cares about, and it's required for a smooth ghost-preview that
  redraws on every mouse move.
- Note: `Mesh::FromFile(name, copyData=true)` copies vertex data into
  `vs`/`is` for the Bullet hull. For the *authoring* preview we don't need
  hulls; we could keep one CPU copy per part type (shared) rather than per
  instance.

### 5.5 Palette part preview (optional polish)

- **v1:** ImGui `Selectable()` text list of catalog parts (the `SHIPS` window
  pattern). No 3D preview. Sufficient to drive the VAB.
- **v2:** render each part type to a small square texture once (draw the cached
  `Model` into an FBO from a fixed camera) and use it as the list icon (KSP
  style). Cheap because §5.4 already caches the `Model`.

### 5.6 Port markers

Reuse the existing **`Billboard`** machinery (`src/billboard.h`, already used
for the prograde/retrograde markers) or simple line/quad markers: at each
available port on the *selected* part, draw a small billboard/ring at
`P.worldPos + P.worldRot * portLocalPos` with the port's normal. Color-code by
mode (e.g. axial vs radial). This is pure drawing — no picking required to
*show* them; picking is only needed to *choose* one.

### 5.7 The VAB scene (composition)

A `VabScene` (or a mode flag in `main`) that per frame:

1. `OrbitCamera::Follow(ship pad COM)` + `ComputeView()`.
2. `display.Clear(...)`; draw the **hangar/pad** (`StaticBuilding` /
   `DrawModelAt` at fixed matrices).
3. Draw the **build representation**: for each part in the tree,
   `DrawModelAt(cam, cachedModel, partWorldPose, light, shadow)` (tint the
   selected part).
4. If a palette part is armed and a port is hovered: draw the **ghost**
   (`DrawModelAt` with `alpha=0.4`) at the `attachPose()` result.
5. Draw **port markers** on the selected part.
6. ImGui: parts palette, part properties (stage, controller, delete), and a
   "Launch" / "Save" button.

No physics tick is required while in the VAB (the build representation is
static); the sim stays paused. "Launch" calls `build_ship()` and resumes.

---

## 6. Picking: the interactive glue (next step, sketched)

To make the drawing *interactive* we need mouse → (part, face). Three options:

1. **Bullet `closestRayTest`** (simplest *if* the parts have hulls). Each
   flight part already has a `btConvexHullShape` (`physics.cpp:246`). Cast a ray from the camera through
   the NDC mouse point against the ship's collision shapes; the hit returns the
   part (shape owner) and the **world hit point**. Map the hit point into the
   part's local frame to infer the face/port:
   ```
   local = P.worldRot⁻¹ * (hitWorld - P.worldPos)
   if |local.z| dominates  -> axial: Down if local.z<0 else Up
   else                    -> side: angle = atan2(local.y, local.x)
   ```
   Reuses existing data; no new mesh math. (In the *build* representation there
   are no Bullet bodies yet — either build lightweight static hulls for picking
   only, or use option 2.)
2. **CPU ray–triangle** against `Mesh::vs`/`is` (already in memory). No Bullet
   needed; more code; fine for a few dozen parts.
3. **GL legacy picking** (`glSelectBuffer`) — deprecated, avoid.

Recommendation: **option 2** for the physics-free build representation (simple,
no Bullet dependency in the VAB), falling back to **option 1** if we keep the
physics `Vehicle` as the VAB model. Either way, the click→port mapping in §5.2
is the same.

---

## 7. Incremental plan

| Step | Deliverable | Reuses | New |
|---|---|---|---|
| **1** | `DrawModelAt()` + shader `u_alpha`/`u_tint` | `Mesh/Texture/Model/Shader`, `partsShader` | `src/draw.{h,cpp}`, 2 shader uniforms, blend state |
| **2** | Per-part-type `Mesh/Texture/Model` cache | `load_texture`, `Mesh::FromFile` | `PartAssets` map; refactor `build_ship` to use it |
| **3** | Draw a `ShipDef` statically in a hangar scene (orbit cam + pad) | `attachPose`, `OrbitCamera`, `StaticBuilding` | `DrawShipDef()`, a `VabScene`/mode flag |
| **4** | Palette list (ImGui) + single-part preview | `PartsCatalog`, `DrawModelAt`, cache | one ImGui window |
| **5** | Port markers on the selected part | `Billboard` / `DrawModelAt` | marker draw |
| **6** | Picking (mouse → part → face) | `Mesh::vs/is` (or Bullet hulls) | ray–triangle / `closestRayTest` |
| **7** | Ghost preview + snap via `attachPose` | `attachPose`, `DrawModelAt` (alpha) | ghost state, confirm action |
| **8** | Edit ops (add/remove/re-orient/stage/controller) | `ShipDef` schema | mutate the build tree |
| **9** | Save (serialize `ShipDef` → JSON) + Launch (`build_ship`) | `load_ship_def` (inverse), `build_ship` | `save_ship_def()` |

Steps 1–3 already give a **drawable, orbitable ship in a hangar** — the
concrete "drawing ships and parts" outcome. Steps 4–7 make it a VAB. Steps 8–9
make it a *complete* VAB.

---

## 8. Performance notes

- **Cache GPU resources per part type** (§5.4): the biggest win. Turns
  "N instances of part X" from N uploads into 1. Required for a smooth ghost
  preview (redraw per mouse-move) and for large ships.
- **Physics-free authoring preview**: a VAB that doesn't build Bullet hulls or
  run the solver avoids per-frame solver cost and startup hull-construction
  cost while assembling. The flight `Vehicle` is only built at Launch.
- **Draw calls**: 1 per part today; fine for tens of parts. A VAB adds ghost +
  markers + palette previews — still tens, not thousands. Batching same-material
  parts into one VAO is a *future* optimization, not needed now.
- **Ghost blending**: one extra blend-enabled pass for translucent parts; keep
  it minimal (ghost only) to avoid sorting complexity.
- **Double-precision matrices** are already used for `View * Model` (see
  `Body::Draw`); keep that in `DrawModelAt()` for consistency with the existing
  log-depth far-plane handling (`partsShader.vs` `far = 1e13`).

---

## 9. Risks / open questions

1. **Which representation does the VAB edit?** (build tree vs physics `Vehicle`)
   — recommended: build tree (§4). Needs a decision before step 3.
2. **Hangar geometry**: is a full hangar in scope, or is the open pad
   (existing `space_port.obj` + a grid floor) enough for v1? Recommendation:
   open pad first; hangar later (it's just more `StaticBuilding` scenery).
3. **Lighting in the hangar**: flight uses a sun direction
   (`TerrainBody::SunlightDir`). A hangar wants a fixed indoor light. Easy with
   `DrawModelAt()` (pass a constant `sunlight`), but decide the look.
4. **Port discretization**: continuous angle (drag) vs discrete slots (click).
   Discrete is simpler and matches the JSON `angle` field; continuous is nicer.
   Recommend discrete 45° slots for v1.
5. **Controller / stage editing** requires the VAB to expose `ShipDef.controller`
   and `ShipPart.stage` — both already in the schema, just no UI yet.
6. **Back-face culling** is on (`display.cpp` `glCullFace(GL_BACK)`). Ghost
   parts drawn translucent should render both faces (disable culling for the
   ghost pass) or look hollow.
7. **`Mesh::vs`/`is` CPU copies** exist for Bullet hulls; if we go physics-free
   for authoring, we can skip the per-instance copies (one shared copy per part
   type is enough for picking).

---

## 10. Files to touch (estimated)

| File | Change |
|---|---|
| `src/draw.h`, `src/draw.cpp` | **new**: `DrawModelAt()` + `DrawOpts` |
| `src/body.h` | `Body::Draw()` delegates to `DrawModelAt()` |
| `res/partsShader.fs` | add `u_alpha`, `u_tint` uniforms |
| `src/shader.h/.cpp` | no change needed — `setUniform_vec4` already exists (`shader.cpp:99`) |
| `src/main.cpp` | register new uniforms; part-asset cache; `VabScene`/mode; palette UI; `DrawShipDef()` |
| `src/shipdef.h/.cpp` | (maybe) `save_ship_def()` inverse of `load_ship_def()`; port helper |
| `tests/test_shipload.cpp` | extend for `save_ship_def()` round-trip (if added) |
| `Makefile` | pick up new `src/draw.cpp` (already globs `src/*.cpp` — no change) |

Most changes are additive; the only edits to existing code are `body.h`
(1-line delegation) and the `partsShader.fs` (2 uniforms). Low risk.

---

## Appendix A: Key existing code (verified)

- `attachPose()` — `src/shipdef.h:190`, `src/shipdef.cpp:267` (the snap).
- `ShipDef` / `PartDef` / `AttachMode` — `src/shipdef.h`.
- `Vehicle` (parts, welds, staging) — `src/main.cpp:804`.
- `build_ship()` (ShipDef → Bodies) — `src/main.cpp:1590`.
- `Body::Draw()` (per-part draw, Bullet-coupled) — `src/body.h:39`.
- `Vehicle::Draw()` (part loop) — `src/main.cpp:1278`.
- `partsShader.vs/.fs` (log-depth, directional light) — `res/`.
- `OrbitCamera` / `FreeCamera` — `src/camera.h`.
- `StaticBuilding` (pad) — `src/main.cpp:1882`.
- `Billboard` (3D markers) — `src/billboard.h`.
- Part catalog — `res/parts.json` (23 parts); ship defs — `res/ships/*.json`.
- Bullet hull per part — `src/physics.cpp:246`.
- Tests for the data model — `tests/test_shipload.cpp`.

## Appendix B: Reference game part/attach models

- **Orbiter** (`references/orbiter/Src/Orbiter/Vessel.h`): parts attach via
  `AttachmentSpec { ref (pos), dir (approach), rot (longitudinal align),
  toparent, id }`. Attachment points are **parent/child** (a hierarchy), matched
  by `id`. Docking ports are peer-to-peer. This is the closest analog to KSP
  ports: a port = position + normal + roll + compatibility id. Our
  `(mode, angle, offset)` port model is a simpler equivalent that already
  drives `attachPose()`.
- **Pioneer** (`references/pioneer/`): ships are **prebuilt flat assets**
  (`data/ships/*.json`) with "hardpoint" equipment slots — *not* a KSP-style
  part-assembly VAB. Its `editor` is a star-system editor, not a ship VAB. So
  Pioneer is not a direct VAB reference; Orbiter's attachment model is.
