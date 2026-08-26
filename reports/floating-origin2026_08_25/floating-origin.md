# Floating origin for physics — do we need one?

Date: 2026-08-25
Status: investigation (no code written)

## TL;DR

**The physics does not need a floating origin.** Bullet is built with
`USE_DOUBLE_PRECISION=ON` (`middleware/bullet3/build/CMakeCache.txt`, and
`#define BT_USE_DOUBLE_PRECISION true` in `src/body.h`), and the ship is
always re-anchored to its dominant body's frame on SOI crossings. Double
precision has ~9–10 orders of magnitude of headroom at every distance the
game reaches, so there is no physics precision bug to fix.

**The rendering does need a floating origin — and will, hard, once we add a
second planet.** The GPU pipeline is float32. The ship's on-screen position is
quantized to `ULP(distance-from-frame-origin)`, so the ship *swims* (jitters)
by:

| Where the ship is | Distance from frame origin | float32 ULP | Ship swim |
|---|---|---|---|
| Low orbit (600 km) | 6.6e5 m | 0.08 m | **~0.5%** of a ~15 m ship — fine |
| Eerbon→Moon transit (current max) | 1.4e7 m | 1.7 m | **~11%** — visible jitter |
| `high-orbit` scenario (0.85×SOI) | 7e7 m | 8.4 m | **~56%** — bad |
| Interplanetary (Eerbon's orbit, once a 2nd planet exists) | 1.4e10 m | **1.6 km** | **broken** (100× the ship) |

(Ship size: parts are 0.25–10 m tall; a typical 1–2 part ship is ~10–30 m,
so "~15 m" is a fair central estimate for the percentages.)

The fix is to make the **render frame follow the ship** (the classic floating
origin) instead of staying centered on the planet/star. This is a
**rendering-only change** — the physics can stay exactly where it is. It is the
prerequisite for interplanetary travel.

---

## 1. What a "floating origin" is

A floating origin periodically re-centers the simulation's coordinate origin
on the object of interest (the player's ship), so that all the numbers the
*finite-precision* parts of the pipeline touch stay small (near 0) rather than
growing to the scale of the whole solar system. KSP does this for exactly the
reason below: its physics is single-precision and its GPU is float32, so a ship
at 1 AU from the origin would be unresolvable.

There are two independent precision consumers in this codebase, and they have
very different needs:

1. **The physics integrator (Bullet)** — what the question literally asks about.
2. **The renderer (OpenGL, float32 matrices)** — where the real constraint is.

## 2. The current precision architecture

Everything relevant is already deliberately structured around reference frames
(`src/frame.h`, `src/frame.cpp`):

- **A frame tree** rooted at the star. Each body has an *inertial* frame and a
  *rotating* (near-body) frame, each with its own SOI. `root_pos`/`root_vel`
  are the body's universe (star-frame) state; `GetPositionRelTo`/
  `GetVelocityRelTo`/`GetOrientRelTo` convert between any two frames.
- **The ship lives in one frame** (`Vehicle::frame`). Its Bullet rigid bodies
  are positioned in that frame's local coordinates (see `spawn_vehicle` in
  `src/main.cpp`, which expresses the spawn state in the resolved frame and
  calls `ship->moveToFrame(frame)`).
- **SOI re-anchoring is already a discrete floating origin.** `Vehicle::
  switchFrames()` (`src/main.cpp`) runs every tick: if the ship's COM leaves
  the current frame's SOI it moves to the parent frame; if it enters a child's
  SOI it moves to that child. `moveToFrame()` transforms every part's position
  *and* velocity (including the rotating-frame stasis correction) into the new
  frame and calls `setPosRot`/`SetVelocity`. So the physics origin already
  "floats" to the ship's dominant body, with a ±10 km hysteresis to avoid
  thrashing.
- **Rails.** At `time_accel >= kRailsWarp` (10000×) the ship is removed from
  the Bullet world entirely and coasted analytically on its Kepler conic
  (`goOnRails`/`railsTick`/`propagateKepler` in `src/orbit.h`). That is exact
  for any step size — a time-domain escape hatch, not a spatial one, but it
  confirms the integrator is not the precision bottleneck.

## 3. Physics precision: no floating origin needed

Bullet is **double precision** (`USE_DOUBLE_PRECISION=ON`). The ship stays
within ~1 SOI of its frame origin by construction (`switchFrames`). The ULP of
a double at the distances we reach:

| Distance | double ULP |
|---|---|
| 1.4e7 m (Eerbon→Moon) | **3e-9 m** (nanometres) |
| 1.4e10 m (interplanetary) | **3e-6 m** (microns) |

Collision and constraint solving need millimetre–centimetre accuracy. We are
**nine to ten orders of magnitude** inside that, even in interplanetary space
where the origin is the star and the ship is 1.4e10 m out. Welds/constraints
operate on relative offsets (a ship is ~20 m across), which are exact at any
origin. The gravity term `F = -G M m p/r³` and the rotating-frame fictitious
terms (`GetFictitiousAccel`) are all computed in doubles with no cancellation
at these scales.

**Conclusion for the physics: it is fine as-is.** A floating origin would buy
the integrator nothing measurable. Do not spend the complexity on it for
physics reasons.

## 4. Rendering precision: this is where it breaks

The GPU is float32. The pipeline casts the double view/model matrices to float
before upload:

- `src/body.h` `Body::Draw`: `glm::dmat4 ModelView = View * xform *
  model_matrix; glm::mat4 ModelViewFloat = ModelView;`
- `src/main.cpp` (`StaticBuilding::Draw`, terrain): same double-then-float
  cast.

So every coordinate magnitude in the view/model matrices is **quantized to
float32**. A point at distance `X` from the frame origin carries a position
error of ~`ULP(X) = X·2⁻²³`. The camera and the ship are both far from the
origin, and their *relative* position (what the screen shows) is the
difference of two quantized numbers, so the ship wobbles by ~`ULP(X)`.

Measured:

```
Eerbon low orbit      X=6.6e5 m   ULP=0.08 m   ship swim  ~0.5%   fine
Eerbon->Moon (max)    X=1.44e7 m  ULP=1.7 m    ship swim  ~11%    visible jitter
high-orbit scenario   X=7.05e7 m  ULP=8.4 m    ship swim  ~56%    bad
interplanetary        X=1.36e10 m ULP=1.6 km   ship swim  broken
```

### The log-depth hack does *not* fix this

`res/*Shader.vs` already apply the Outerra logarithmic-z trick
(`logz = log(gl_Position.w*C+1)*FC; gl_Position.z = (2*logz-1)*w`) with
`far = 1e13`. That is the right fix for **depth-buffer** precision across the
enormous `zNear=1 / zFar=1e13` range (`src/main.cpp` lines ~3400–3408) — it
kills z-fighting. But it only rewrites `gl_Position.z`. The `x`, `y`, `w`
(position) are still the quantized float32 values from `MVP * position`. So the
ship still swims in *screen position*. **Log depth and position-swimming are
two different problems; we have solved the first only.**

This is consistent with what the code already shows it fighting: the terrain
skirt/stencil workaround in `src/main.cpp` (`TerrainBody::Draw`) exists
specifically because "the float32 view transform can't resolve … at range (its
rounding is of the same order as the skirt depth margin, which z-fights)."

## 5. Why the current SOI re-anchoring isn't enough

The physics origin floats to the *nearest body*, which is great while the ship
is near some body (the ship is then within ~1 SOI of the origin). The gap is
**open space between bodies**:

- Today the system is Sun + Eerbon + Moon, and the Moon orbits *inside*
  Eerbon's SOI, so the ship is always in some body's SOI and within ~1.4e7 m
  of an origin. That gives the ~11% swim above — a visible quality issue, not a
  hard break.
- The moment a **second planet** is added, a transfer between them spends most
  of its time *outside both* SOIs, in the star's frame, ~1.4e10 m from the
  origin. The ship then swims by ~1.6 km — it is unrenderable. This is the
  "scale up the game a lot" case `QWEN.md` warns about, and it is exactly the
  regime where a body-centered origin fails and a ship-centered origin is
  required.

## 6. Recommendation

Add a floating origin **for the renderer**, and leave the physics alone.

**Concretely:** make the *render frame* a frame whose origin sits on the
active ship (and which carries the ship's attitude), rather than the ship's
SOI frame. Concretely this means:

- Keep `Vehicle::frame` (the physics frame) exactly as it is — SOI-based,
  double precision. No change to `moveToFrame`, `switchFrames`, rails, or the
  Bullet world.
- At render time, build the view from a ship-centered frame. The ship's parts
  then render near the origin (ULP ≈ 0), the camera (which already orbits the
  ship's COM via `camera->Follow(focusWorldPos(focusBody))`) is near the
  origin, and the bodies render at their large offsets — but they are huge, so
  their `~ULP(offset)` swim (1.7 m on a 6e5 m planet, 1.6 km on a 2.6e8 m
  star) is invisible.
- The per-body transforms already compute `planet->frame->
  GetPositionRelTo(ship->frame)` (`src/main.cpp` render section); generalising
  "ship->frame" to "ship-centered frame" is a local change to that transform
  construction and to `Vehicle::Draw`'s `xform`.

**Why ship-centered, not body-centered, for the render frame:** it is the only
origin that is small *and* stays small in interplanetary space. A body-centered
origin is small only near a body.

**Effort / risk:** moderate and contained. The physics, staging, rails, and
frame tree are untouched; the change is in the view-transform construction and
the a few places that build body/model matrices for the GPU (`Body::Draw`,
`StaticBuilding::Draw`, terrain patch draw). The main things to re-verify after
the change are: terrain LOD/patch selection (it keys off camera distance, which
is a relative quantity and is invariant under re-centering, so it should be
unaffected), the log-depth `far` constant (still fine — it is a relative
depth), and the shadow/`SunlightDir` direction (a normalized direction,
origin-invariant).

**Sequencing:** not urgent for the current Sun+Eerbon+Moon system (the cost is
~11% ship jitter on long transits). It becomes **required before adding a
second planet / interplanetary travel**. If interplanetary play is near-term,
do it first; otherwise it can wait, but it should be on the pre-interplanetary
checklist.

## 7. What we do *not* need

- **Do not** re-center the Bullet world on a timer "for physics reasons" —
  double precision already has 9–10 orders of magnitude of headroom, and the
  SOI re-anchoring already keeps the ship within ~1 SOI of the origin. Adding
  a continuous physics re-center would be pure complexity with no measurable
  benefit.
- **Do not** expect the existing log-depth shader to fix ship jitter — it only
  fixes depth precision, not position quantization.
- **Do not** switch Bullet to single precision to "match" the renderer — that
  would *create* the physics precision problem we are discussing, for no gain.

## 8. Key code references

| Concern | Location |
|---|---|
| Bullet double precision | `middleware/bullet3/build/CMakeCache.txt` (`USE_DOUBLE_PRECISION:BOOL=ON`); `src/body.h` (`BT_USE_DOUBLE_PRECISION`) |
| Frame tree / rel transforms | `src/frame.h`, `src/frame.cpp` |
| SOI re-anchoring (discrete floating origin) | `src/main.cpp` `Vehicle::switchFrames`, `Vehicle::moveToFrame` |
| Ship spawn into a frame | `src/main.cpp` `spawn_vehicle`, `resolve_frame_by_soi` |
| Rails (analytic coasting) | `src/main.cpp` `goOnRails`/`railsTick`; `src/orbit.h` `propagateKepler` |
| Double→float render cast | `src/body.h` `Body::Draw`; `src/main.cpp` `StaticBuilding::Draw`, terrain `Draw` |
| Log-depth (fixes depth, not position) | `res/partsShader.vs`, `terrainShader.vs`, `sunShader.vs` |
| zNear/zFar | `src/main.cpp` lines ~3400–3408 (`camZNear=1`, `camZFar=1e13`) |
| Float32 skirt/z-fight workaround | `src/main.cpp` `TerrainBody::Draw` (stencil mask) |
| Camera focus / follow | `src/main.cpp` `focusWorldPos`, `camera->Follow`; `src/camera.cpp` |
