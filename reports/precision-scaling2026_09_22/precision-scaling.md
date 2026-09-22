# Precision scaling: bigger solar systems without jitter

Date: 2026-09-22
Status: render-side fix IMPLEMENTED and verified (see UPDATE 2); physics floating origin still a future effort

## UPDATE 2 (same day): fix implemented + verified

Implemented the two render-path fixes below:

1. **`Vehicle::partPoseRelCom`** (src/vehicle.cpp/.h): part pose relative to
   the hull COM computed purely in ship-local quantities —
   `sRot * (localPos - comS)` (comS = `principal`'s origin) — never
   materializing absolute coords.
2. **`Vehicle::Draw`**: per-part models built from `partPoseRelCom`; the
   ship's absolute COM moved into `xformShip = xform * translate(comAbs)`
   as ONE common shift. For the active ship `comAbs == renderOrigin`
   bit-for-bit (both are `comPos()`), so the shift is exactly 0; other
   same-frame ships get an exact small difference (Sterbenz); cross-frame
   ships keep a single rigid ≤ULP shift (parts stay rigid among themselves).
3. **`Camera::ComputeView` (orbit mode)**: view translation built as
   `(focusPoint - renderOrigin) + off` instead of `(focusPoint + off) -
   renderOrigin`; `forward` derived from `-off` instead of the cancelling
   `focusPoint - pos`. The absolute `pos` is still stored for free-cam
   transitions and picking.
4. **Engine plume + RCS puff** (render.cpp): built COM-relative
   (`partPoseRelCom` / `z * sz`), drawn `View * Model` with no `-ro`
   product — the plumes are active-ship-only, where COM == renderOrigin.
5. **Instrument kept**: `PRECDBG=1` env gate in `DrawModelAt` (body.cpp)
   prints per-draw ModelView translation, double vs float32, capped at
   4000 lines. Zero cost when unset (one cached branch). This is the tool
   for the future floating-origin work.

Verification (logs in tmp/):
- oort repro (`--scenario oort --sim-mouse 0,1000,0,1000,3`): part
  ModelViews went from 4 grid-snapped unique values (z flipping
  -9.875 ↔ -10, exactly ULP(1e15)) to **1** unique value per part over
  451 frames; coords carry the full double mantissa; float cast err ~5e-7 m
  (the floor at ~10 m view distance).
- interstellar + 3 s pitch (`--scenario interstellar --sim-press
  1000,3000,W`): view-space values change smoothly (330-470 unique values
  over ~530 frames), max consecutive-frame delta **1e-14 m**; float err
  ~4e-8 m. Pre-fix this scenario put parts on a 22 m grid.
- `make test`: all pass. `make e2e` (4 jobs): 17/18, dock-approach failed
  under parallel load and **passes on isolated rerun** (timing-flaky,
  unrelated — sim untouched); dock/undock/dock-save-load pass.

Known residuals (accepted for now):
- Pick-ray origin vs rendered eye diverge by ≤ ULP(pos)/2 in orbit mode
  (pickRay uses the rounded absolute cam.pos; the view uses the exact
  relative eye): ~6 cm at oort, ~6-11 m at interstellar — and interstellar
  picking is dominated by Bullet's own absolute-coord quantization anyway.
  Same root cause: the orbit→free transition can pop by that ε at extreme
  range. Both documented at the pick.cpp/vab.cpp/camera.h sites.
- Camera shake (`focusPoint += shake_off`, render.cpp) still adds in
  absolute coords — sub-ULP shakes vanish at oort+. Cosmetic, rare.
- Sun drawn from oort range has a legitimately huge view translation with
  ~1e6-1e7 m float cast error — sub-pixel, single rigid body.
- Sim itself (Bullet integration, rails) still runs in absolute coords:
  option A (floating origin) remains the next step for SIM correctness
  beyond ~1e15.

---

## UPDATE (same day): root cause found

Instrumented the oort repro
(`./osp --body Kerbol --scenario oort --ship res/ships/racer.json
--sim-mouse 0,1000,0,1000,3`, log: `tmp/precision_oort_mv.log`):

1. **Sim side is rigid.** Part-to-part relative poses from
   `partWorldPose` are bit-constant across frames at oort (0 nonzero deltas
   in 547 frames) — for the spawn attitude, part offsets lie along the small
   coordinate axis, so the ULP rounding is common-mode.
2. **View side snaps.** Per-part `ModelView` translations (env-gated print
   in `DrawModelAt`, `PRECDBG=1`) take only 4 unique values per mesh across
   the run: z flips between -9.875 and -10 — **exactly ULP(1e15) = 0.125 m**
   — and y flips by 0.0078 (the same snap through the view rotation). The
   whole ship jumps ±1 ULP relative to the camera, frame to frame. Float
   cast error for ship parts is exactly 0; the quantization happens BEFORE
   the render-frame subtraction.
3. **Entry point: the camera chain.** `Camera::ComputeView` computes
   `pos = focusPoint + ref*orbitOffset()*distance` with `focusPoint` =
   `Game::focusWorldPos()` in ABSOLUTE frame coords (~1e15 at oort). The
   addition rounds onto the 0.125 m grid; `pos - renderOrigin` is then an
   exact difference of grid values, so the camera-to-ship vector is
   quantized. Per-frame wobble of focus/offset flips the rounding → jitter.
   Scales with distance: ~1 mm at neptune, 0.125 m at oort, ~22 m at
   interstellar.
4. **Second ingress at interstellar + rotation:** once attitude spreads
   part offsets onto the huge axis, `partWorldPose`'s
   `pos = sPos + sRot*localPos` rounds EACH part independently onto the
   ULP(1e17)=22 m grid → parts scatter against each other ("abstract
   art"). Demonstrated in isolation: `tmp/precision_repro.cpp` (static
   per-part displacement + ±1 ULP flips as rotated offsets cross rounding
   boundaries).
5. Incidental: the sun drawn from oort range carries a legitimately huge
   view-space translation (~1e15) with ~2.7e7 m float cast error — harmless
   (sub-pixel angularly, single rigid body), but noted.

**Fix (render-side, cheap, no floating origin needed for the visuals):**
- Camera: build the view from `cam_rel = (focusPoint - renderOrigin) +
  ref*offset*distance` — the subtraction of nearby doubles is exact
  (Sterbenz), so the camera-to-ship vector never touches the ULP grid.
  Same for `forward` (derive from the offset, not `focusPoint - pos`).
- Parts: build draw model matrices relative to the render origin —
  `sRot*(localPos - comS)` with comS the COM in the ship's S frame — so
  huge absolute part positions are never materialized on the render path.

Physics-side floating origin (option A below) remains the fix for SIM
correctness at oort/interstellar (integration, contacts, rails handoff),
but the visual jitter is entirely the two render-path issues above.

---


## Problem

We want the option of solar systems bigger than the shipped KSP-like one.
At our current system scale, ship parts visibly jitter against each other at
Neptune distance (~4.5e12 m), it's very noticeable at Oort (~1e15 m), and at
interstellar (1e17 m) the ship becomes abstract art. Test scenarios:
`neptune`, `oort`, `interstellar` (hardcoded table, `src/vehicle.cpp:284-289`,
selected via `--scenario`).

## Current state of the precision pipeline

Investigated end-to-end (simulation → part transforms → rendering):

- **Everything is double up to the camera.** World positions in the frame
  tree (`src/frame.h`) and part poses (`src/part.h:132-133`) are
  `dvec3`/`dmat3`. A ship is ONE compound `btRigidBody`; part world poses are
  derived, never stored (`src/vehicle.h:117-146`, `src/vehicle.cpp:866-872`).
- **Rendering is already camera-relative.** `camera->renderOrigin` (a `dvec3`,
  `src/camera.h:28-35`) is set each frame to the active ship's COM
  (`src/render.cpp:330-334`). The view matrix is built in double from
  `pos - renderOrigin`, and every draw site composes
  `View * translate(-renderOrigin) * Model` as a `dmat4` and narrows to
  `mat4` only afterwards (`DrawModelAt`, `src/body.cpp:3-16`; terrain bakes
  anchor-relative, `src/terrain.cpp:201-216`; physics debug lines subtract
  renderOrigin in double, `src/physics.cpp:128-141`). Shaders only ever see
  float32 render-frame numbers.
- **Physics has no floating origin.** Bullet is built with
  `BT_USE_DOUBLE_PRECISION` (asserted, `src/physics.cpp:163-165`), but hull
  coordinates are ABSOLUTE metres in the SOI body frame. Gravity is applied
  manually per substep (`setGravity(0)` + re-apply, `src/physics.cpp:189-197`).
- **Rails already sidestep Bullet** for coasting ships (analytic two-body
  conic, `dvec3` state, `src/vehicle.h:332-355`).

## Symptom matches the double ULP ladder

The observed severity scaling matches absolute-coordinate double precision
exactly (the ladder already documented in `src/vehicle.h:1139-1154`):

| scenario     | distance | double ULP        | observed               |
|--------------|----------|-------------------|------------------------|
| neptune      | 4.5e12 m | ~1 mm             | slight jitter up close |
| oort         | 1e15 m   | ~0.22 m           | very noticeable        |
| interstellar | 1e17 m   | ~22 m             | abstract art           |

**Unexplained detail:** parts jittering *against each other* is NOT fully
explained by a quantized hull transform — a single quantized transform should
wobble the whole ship rigidly, and the camera-relative render (renderOrigin =
the same ship's COM) should hide even that. There is likely a second bug in a
draw site, the COM computation, or a per-part precision path that skips the
double composition. Debug first (per QWEN.md: add debug prints, run the
scenarios) so the fix addresses what we actually see, not what the ULP table
predicts.

## Options considered

- **A. Floating origin for physics (ship-centred Bullet world).** Periodic
  rebase: when `|pos|` exceeds a threshold (say 1e6 m), translate the physics
  world so the active ship sits near the origin. Unusually cheap here: one
  compound body per ship, manual gravity, anchor-relative terrain — a rebase
  is "shift hull transform + terrain anchors + frame offsets". Near the ship,
  double ULP is sub-nanometre at any system scale; planets carry the big
  coordinates and tolerate coarse ULP because they're enormous. Cost:
  moderate; audit everything that reads absolute physics coords (frame tree
  `root_pos`, rails handoff, picking, save/load). **CHOSEN.**
- **B. Rails-first at distance (the KSP approach).** Only land ships into
  Bullet near bodies; integrate everything else analytically. **REJECTED:**
  redundant once A exists (it was KSP's workaround for *not* having a
  floating origin), and it can't support ship-ship collisions, attitude
  control under thrust at distance, or EVA — those need a shared physics
  world regardless of distance.
- **C. Hierarchical relative coordinates.** Nothing stores absolute metres
  from the system root; absolute is a transient derived quantity. Not a
  separate work item — it's the data layout A naturally forces: a rebase
  becomes "change the offset between the ship's frame and Bullet's world"
  instead of "teleport every body". Already exist in embryo (SOI frame tree,
  ships live in `ship->frame` coords).
- **D. Double-double for orbit propagation.** Hi/lo double pair (~100 lines
  or libqd) for OUR analytic code only: Kepler rail propagation
  (`orbit_pos0`/`orbit_vel0`) and frame-tree ephemeris accumulation. At
  interstellar, a plain-double 1e17 m state has 22 m ULP; long coasts amplify
  that into km-scale trajectory error, breaking orbit prediction even if
  rendering is fixed. A few hundred FLOPs per substep — free. **Bullet can
  never use it** (double is Bullet's ceiling; DD would mean forking the
  solver), but with A Bullet only sees small local coords where plain double
  suffices. **DEFERRED** until interstellar trajectory prediction matters.
- **E. Unit rescaling (km).** Shifts the ladder 3 digits only; wrecks
  Bullet's 0.05–10 m object-scale sweet spot. **REJECTED.**
- **F. int128 fixed-point positions.** Covers any scale but custom arithmetic
  everywhere, no SIMD, Bullet can't use it. **REJECTED.**

Precedent: KSP never truly solved this (rails + scaled-space rendering hide
it); Star Citizen converged on exactly A (ship-local origin).

## Plan

1. **Debug the neptune part-vs-part jitter first.** Confirm the precision
   ingress point with debug prints before building anything. Check: per-part
   draw paths, COM/renderOrigin computation, any float32 intermediate,
   hull `btTransform` quantization per substep.
2. **Implement A** (floating origin / periodic rebase), with C as the
   framing: ship-local Bullet coords, bodies and frames keep double
   body-relative state.
3. **D later** if interstellar conic prediction needs it.

## Residual known-small issues (noted during investigation)

- `ModelFloat` "Normal" uniform carries a huge quantized translation column;
  harmless (shaders use only the 3x3 rotation), `src/body.cpp:16`.
- 24-bit depth with reverse-Z; near-surface precision already scoped as a
  32F follow-up (`src/display.cpp:60-64`, `tmp/depth_migration_scope.txt`).
- Prior related report: `reports/floating-origin2026_08_25/` (immutable).
