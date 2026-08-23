# Radial parts: attaching parts to the side of a ship

**Scope (2026-08-23):** what does it take to attach parts radially — to the side of a
part — instead of only stacking them axially? Data model, placement, physics, content,
tests.

**Short answer:** small. The sim is already placement-agnostic — thrust, wheel torque,
fuel, inertia, rendering and frame-switching all work per part in the part's local frame.
Only three things are stack-specific: (a) the ship-def JSON (one `offset` scalar, no
parent/side), (b) `build_ship` placing everything on one axis, (c) the two hardcoded
axial weld anchors. Change those three and radial parts work. Recommended split:
**Phase 1** = side-attached parts with parallel orientation (first content: radial fuel
tanks); **Phase 2** = per-part `roll` + a radial RCS engine.

One bonus finding: the JSON `controller` field is parsed and validated but **never
applied** — the real controller is hardcoded to the last part (`main.cpp:857`). Small
fix, do it in Phase 1.

---

## 1. Ground truth (verified 2026-08-23)

| What | Where | Stack-specific? |
|---|---|---|
| Ship def = linear stack; one `offset` per part (m from base, along the stack axis) | `src/shipdef.h` (`ShipPart`), `res/ships/basic.json` | **Yes — the only stack-specific data** |
| Placement: every part at `base + orient*(0,0,offset)`, all parts share `orient` | `build_ship`, `main.cpp:1399` | **Yes** |
| Weld: `btGeneric6DofConstraint`, angular limits locked (rigid), anchors **hardcoded** to parent-local `(0,0,-1)` / child-local `(0,0,+1)` | `PhysicsEngine::GlueTogether`, `physics.cpp:272` | **Yes, anchors only** — the constraint machinery is direction-agnostic |
| Thrust = along the thruster's **local +Z** | `ApplyCentralForceForward` `physics.cpp:414`; `main.cpp:1023` | No |
| Wheel torque = along the wheel's **local X/Y/Z** | `applyRotationForce`, `main.cpp` (Vehicle) | No |
| Fuel: ship-level pool — `consumeResourceMass` scans all tanks, no routing | `main.cpp` (Vehicle) | No — a side tank is "connected" automatically |
| Inertia: per-part local inertia **+ parallel-axis term** about the ship COM | `getInertia`, `main.cpp:1215` | No — already correct for arbitrary placement |
| Mass / COM / gravity / frame-switching: all per-part positions | `applyGravity`, `moveToFrame`, `get_center_of_mass` | No |
| Rendering: per-part body transform | `Vehicle::Draw` | No |
| Headless parse/validate tests | `tests/test_shipload.cpp` | extend |

Geometry facts:

- **All current part meshes are exactly `[-1,1]³`** in their local frame (measured:
  `capsule.obj`, `engine.obj`, `reaction_wheel.obj`). Local +Z is the long/thrust axis;
  the flange faces are at ±1.
- **Spacing is encoded in the weld anchors, not the JSON.** Anchor pair
  `(0,0,-1)`/`(0,0,+1)` forces child-center = parent-center ∓ 2 m. The JSON offsets
  (12.5 / 10.5 / 8.5 / 6.5) merely have to stay consistent with that. A radial weld with
  `+d`/`−d` anchors gives child-center = parent-center + 2 m out along `d` — same
  flange-touching geometry, sideways.
- **The anchor-pair machinery already encodes any relative offset**: for parallel frames,
  anchor-pair `ap`/`ac` puts the child at `parent + (ap − ac)`. So an axial-shifted radial
  part (1 m forward along the stack) is a free follow-up — same code, no new constraint type.
- **Roll math is clean** (verified): rolling the child about the radial axis `d` through
  the weld point leaves the weld point *and* the child center (`P + 2·orient·d`) fixed —
  only the part's local frame rotates: `R_child = R_parent ∘ Rot(d, roll)`, and the child
  anchor stays `−d`. Roll 0° = thrust along the ship axis; 90° = thrust ±radial (out/in
  RCS); between = tangential. All quaternion-routed (the known GLM↔Bullet row-major trap).
- **Dead code — don't touch:** `Vehicle::Detach`, `setPosition`/`SetPosition` (no callers).
- **Nose** = local +Z of the first reaction wheel (slew/prograde) and of the controller
  (navball). Radial parts don't move the nose as long as the core stack keeps it.
- **`controller` is dead data:** `ShipDef::controller` / `controllerIndex()` are parsed
  and validated (`shipdef.cpp`), but `Vehicle::init()` sets `controller = parts.back()`
  unconditionally (`main.cpp:857`) and nothing applies the def field. The JSON field
  currently does nothing.

**Conclusion: the only stack-specific code is the JSON shape, one placement line, and two
anchor constants. The physics/sim layer doesn't care where parts sit.**

---

## 2. Recommended design

### 2.1 Ship-def JSON (Phase 1 syntax)

Two new fields on a part entry; existing entries keep exactly today's meaning:

```json
{
  "name": "Basic-Radial",
  "parts": [
    { "part": "capsule",        "offset": 12.5 },
    { "part": "reaction_wheel", "offset": 10.5 },
    { "part": "fuel_tank",      "offset": 8.5 },
    { "part": "engine",         "offset": 6.5 },
    { "part": "fuel_tank",      "attach": 2, "side": 90 },
    { "part": "fuel_tank",      "attach": 2, "side": 270 }
  ]
}
```

- **`attach`** — parent part index (required on a radial entry). A part *without*
  `attach` glues to the previous part in the list, axially — **identical to today**, so
  every existing ship def keeps working unchanged.
- **`side`** — direction around the stack axis, degrees. 0° = ship-frame +X, 90° = +Y
  (the radial plane is the part's local XY; +Z is the nose). Any angle is legal in code;
  90° multiples are the useful ones with 2 m parts.
- **Radial distance: fixed 2 m** center-to-center — the same flange convention as axial
  spacing (flanges at ±1 m touch). Not a field, for the same reason axial spacing isn't.
- A radial part sits at its **parent's axial position** (its own `offset` is ignored for
  radial entries; validated as equal-if-present, to avoid confusion).

**Validation** (headless, `shipdef.cpp`, in the existing throw-on-bad-data style):

- `attach` in `[0, current index)` — no self-reference, no cycle.
- `side` in `[0, 360)`.
- Two radial parts on the same parent must be **≥ 60° apart**: chord = 4·sin(Δ/2) m, and
  2 m parts need ≥ 2 m separation → Δ ≥ 60°. So four ports (90°) is the natural maximum —
  a clean, checkable rule.
- If the def uses radial parts, `controller` should name a core-stack part (see §2.4).

### 2.2 Placement + weld

- `GlueTogether(parent, child, anchorP, anchorC)` — anchors become parameters; the
  existing call passes `(0,0,-1)` / `(0,0,+1)`. Constraint type, angular lock, solver:
  unchanged.
- **Axial:** unchanged.
- **Radial**, `d = (cos side, sin side, 0)` in the ship frame (child frame parallel to
  parent's):
  - child position = parent position + `2·orient·d`;
  - anchors: parent-local `+d`, child-local `−d`.

### 2.3 Orientation

- **Phase 1: radial parts keep the parent's orientation** (pure translation). Thrust
  stays along the ship axis, wheel torque along the ship axes, the mesh reads as a
  sideways copy. That's exactly what tanks/wheels want, and it keeps Phase 1 minimal.
- **Phase 2: `roll`** — degrees about the radial axis (default 0 = Phase 1). Per §1 the
  math is a single quaternion composition; the weld and the center don't move.

### 2.4 Controller fix (small, Phase 1)

Apply the already-parsed `ShipDef::controller` in `Vehicle::init()`
(`controller = parts[def.controllerIndex()]` instead of `parts.back()`), and make the
default (field omitted) the last **axial** part rather than the last part in the list.
One-line class of change; removes dead data; makes radial defs robust.

### 2.5 Content

- **Phase 1 content:** radial fuel tanks on the basic ship — zero new behavior surface
  (fuel pool, mass, inertia all already work per part). The cheapest possible proof.
- **Phase 2 content:** a small radial RCS engine in `parts.json` (field-driven:
  `mass` + `fuel_rate` + `exhaust_velocity` — JSON only, no source changes), used with
  `roll` for out/in/tangential thrust.

---

## 3. Phases

### Phase 1 — side attachment (the core ask)

1. `shipdef.h/.cpp` — `ShipPart` + `attach`, `side`; parse, validate (§2.1), docs.
2. `physics.h/.cpp` — anchor parameters on `GlueTogether` (defaulting to today's pair).
3. `main.cpp` — `Vehicle::attachDown` → attach-with-anchors; `build_ship` computes
   (pos, orient, anchor pair) per part; controller fix (§2.4).
4. `res/ships/basic-radial.json` — basic + two side tanks on the tank stage.
5. `tests/test_shipload.cpp` — radial parse + resolution; error paths (bad attach,
   same parent + same side, < 60° separation); aggregates unchanged (mass/thrust/torque/fuel).
6. **In-game (Denis):** pad + orbit scenarios; full throttle; stick + prograde/
   retrograde/kill-rot; time warp; SOI switch. Watch the constraint solver at high warp
   (the only new physics is an offset weld vs the coaxial one).

**Surface: ~150 lines incl. tests, across 5 files + 1 JSON. Small.**

### Phase 2 — roll + RCS

1. `shipdef` — `roll` field (default 0 → Phase 1 behavior).
2. `main.cpp` — child orient = parent ∘ `Rot(d, roll)`, quaternion-routed.
3. `parts.json` — radial RCS engine part.
4. Tests — roll parse/validation; aggregates; ship with RCS.
5. In-game — RCS burns; attitude control with tanks at various rolls.

### Phase 3 — optional, only if needed

- Per-part radial distance (`"dist"`) if 2 m proves too coarse.
- Axial-shifted radial parts (free per §1 — JSON + one placement line).
- Named catalog ports (KSP-style) if content outgrows what `side` can express.

---

## 4. Risks

1. **Offset-weld solver stability** — the only new physics. Bullet handles offset 6DOF
   welds routinely; the real risk is at high time acceleration (more substeps, drift).
   Mitigation: the Phase 1 test matrix includes warp + long burns; the per-substep
   force re-application pattern is untouched.
2. **Overlap** — the 60° rule covers same-parent pairs; different parents, same side,
   adjacent axial positions land exactly 2 m apart (touching, fine). A part mesh growing
   past its 2 m cube breaks the flange-touching visual, not the physics (anchors are
   just offsets) — and invalidates the 60° rule.
3. **Quaternion conversions** (roll) — the known GLM↔Bullet row-major trap; route
   through quaternions like every other orientation path.
4. **Stale `.o` after struct change** — `ShipPart` grows; the Makefile tracks headers
   (`-MMD -MP`), so a plain `make` is safe, but a clean build is cheap insurance.

---

## 5. Decisions needed

1. **Phase 1 = side-attached, parallel-oriented parts, first content radial fuel
   tanks** — OK as the smallest proof?
2. **JSON syntax** as in §2.1 (`attach` + `side`, fixed 2 m distance, 0° = +X)?
3. **Roll in Phase 2** (recommended), or day one — e.g. if the first radial part you
   want is an RCS engine rather than a tank?
