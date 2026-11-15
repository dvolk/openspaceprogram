# Building frames (#850): evaluation & staged plan

Status: **PROPOSED** (2026-11-15). Evaluation of the SoI-bubble idea filed as
#850, against master `431c6aca`. Companion to
`reports/orbital-buildings2026_11_14` and `reports/air-buildings2026_11_14`
(read for context; every load-bearing claim below was re-verified in the
current code). No code changed by this report.

## 0. Verdict

**Yes — do it.** The bubble is the right shape, and the code is closer to it
than the issue's "XL refactor" framing fears. The frame-switch algebra
(`moveToFrame`, `comStateIn`, `moveToRailFrame`) is *already* general enough
for a moving, rotating origin — it is written in `root_pos/root_vel/
root_orient` and needs only the frame's **total** angular velocity instead of
its own spin. What is actually body-centric is a short, enumerable list:

1. `omega()` is own-spin only; every stasis / angular-velocity / fictitious
   reader assumes the parent chain is inertial (true today: inertial →
   rotating, two levels). A building frame adds a third level.
2. Gravity points at the **frame origin** (`applyGravity`, vehicle.cpp:2936).
   A building frame's origin is the building, not the body centre.
3. `GetFictitiousAccel` has Coriolis + centrifugal only. A moving origin adds
   the **−A_origin** term (the origin's own acceleration).
4. A scatter of `|comPos()|` readers that assume origin == body centre
   (`inAtmosphere`, `isGrounded`, HUD altitude, …).
5. The Bullet world has **no explicit frame owner**: terrain colliders are
   baked in rotating-frame coordinates and stay aligned only because
   collision leaves exist near the ground, where the ship's frame *is* the
   rotating frame. A ship in a bubble breaks that implicit invariant.
6. Save/spawn/status plumbing keyed on `rotating` bool + body name.

Everything else — rendering (`GetBodyDrawTransform`, `CameraInBodyFrame`),
the SoI walk (`soiTarget`, `resolve_frame_by_soi`), proximity, the frozen-rail
park mechanism — is already generic and works for a building node unchanged.

**The payoff is bigger than "remove the deck carry".** Inside the bubble the
building is *static*. That turns "ship resting on a moving deck" into "ship
resting on a pad" — the exact situation the whole game already handles:

- the deck carry (`tickDeckCarry` + `carriedBy/carryOffset/carryRot`, the
  substep hook, the pause-settle, the `v*h` pre-compensation) is **deleted**;
- the frictionless-deck hack is deleted — Bullet's zero-velocity static
  solver is *correct* for a deck that is genuinely static, so ordinary
  friction and contact return;
- wheel/EVA `surfaceVel` reads become zero in the bubble (the deck is at
  rest); the moving-ground special cases dissolve;
- `isGrounded` → brakes, science, recovery, HUD "Landed", orbit-plane overlay
  suppression all fall out for free, as on a planet;
- **parked rails become possible**: `goOnRails`' grounded branch already
  freezes the ship *in its current frame* — frozen in a building frame, it
  rides the building through rails warp with zero new rail machinery. The
  air report's §8.1 "building-anchored rails" (a new rail flavor storing
  building id + offset) is mostly unnecessary: the bubble *is* that flavor;
- the `brakesOn` save special case goes away (parked really is v=0);
- warp while parked is no longer capped at `prox_warp` 1× — a frozen ship is
  on rails, so rails warp is legal;
- spawn-on-platform becomes "place at v=0 in the bubble frame" — the
  wind-velocity inheritance dance disappears;
- it is the principled path to **#848 docking** ("docked = joined the
  station's frame") and multipart rotating stations (buildings sharing one
  frame), and it keeps player construction (DESIGN.md) open: a placed
  building spawns a frame node.

One physics nicety falls out for free: in an orbital station's frame the
−A_origin term cancels gravity at the station, so a released ship drifts on
Hill/Clohessy–Wiltshire relative dynamics (weightless, tidal drift) instead
of "falling toward Venus". On the air platform the same term is ~1e-3 m/s²
and the ship feels essentially full Venus g, held by the deck. Both are
*correct*, from one mechanism.

**Sizing:** XL is right — comparable to ~1.5× the orbital-buildings v1
effort, but front-loaded into one assert-guarded, behavior-preserving
generalization (S0 below). The deck carry remains the working stopgap until
S4; nothing regresses mid-plan.

## 1. Evidence (verified in code, master 431c6aca)

| Fact | Where |
|---|---|
| Frame tree: two levels per body (inertial + rotating), laws are Kepler (`orb_ang_speed`) or fixed-axis spin (`rot_ang_speed`) only | frame.cpp `UpdateOrbitRails`; system.cpp:2034–2192 |
| Point velocity in root space = `root_vel + root_orient*(v + ω×p)`; `moveToFrame` adds old stasis, subtracts new — **valid for a moving origin** if ω is the frame's total angular velocity | frame.h `GetStasisVelocity` comment; vehicle.cpp:6850–6960 |
| `omega()` = own spin only; angular-velocity switch and `comStateIn` use it — correct today only because ancestors are inertial | frame.h; vehicle.cpp:6944, 6985 |
| Gravity is central about the frame origin, per part, with tidal lever | vehicle.cpp `applyGravity` 2936–2975 |
| Fictitious = Coriolis + centrifugal, gated on `isRotFrame()` | frame.h `GetFictitiousAccel`; vehicle.cpp:2956, 5003 |
| `airRelativeVel` **asserts** the rot frame shares the ship frame's origin — breaks for a bubble; wind is authored in the rot frame | vehicle.cpp:3199–3225 |
| `isGrounded` = rot frame + speed < 3 m/s + COM within 100 m of analytic terrain (radius from frame origin) | vehicle.cpp:7050; constants.h:196 |
| `canRail` = `isGrounded() || !inAtmosphere()`; grounded `goOnRails` freezes **in the current frame** (`rail_pos = com_frame`, frame untouched) | vehicle.cpp:7060–7110 |
| `soiTarget`: nearest containing child wins; exit at `soi + kSoiMargin`, entry at `soi − kSoiMargin`; **kSoiMargin = 10 km absolute** | vehicle.cpp:6962; constants.h:18 |
| `railAdvance` SoI test uses `skipSameBody=true` — a building frame (body == parent body) is skipped by warp-coasting conics; frozen rails never advance | vehicle.cpp:7200, 7263 |
| One Bullet world; terrain collision baked at rotating-frame anchors; **no re-expression on frame switch** — aligned only by the implicit "collision leaves exist ⟹ ship in rot frame" invariant | physics.cpp `AddTerrainCollision`; render.cpp:430; terrain.cpp:222 |
| Rendering fully relativized: `planet->transform = GetBodyDrawTransform(rf)`, LOD via `CameraInBodyFrame(transform, …)`, camera origin = ship COM | render.cpp:425–445; terrain.h:670 |
| Driven pose laws are pure f(t), already produce pos+orient; `tickPose` also sets `surfaceVel/surfaceAngVel` for the carry | buildingair.h, buildingorbit.h; terrain.cpp:64–101; body.h:52 |
| Proximity gate keys on **any live ship**; railed ships need no collider; frozen parked ship therefore needs no deck collider while switched away | tick.cpp:263–290 |
| Save: `pose.rotating` bool + body name resolve the frame on load; `brakesOn` persisted as the platform special case | save.cpp:467, 1084, 1091 |
| `resolve_frame_by_soi` recurses nearest-child on `root_pos` — finds building nodes with no change | vehicle.cpp:215–230 |
| Docking requires `a->frame == b->frame` — a docked ship joining the station frame satisfies it by construction | game.cpp:2138 |
| `kRailsWarp = 11` (physics warp 1–10, rails ≥ 11); `prox_warp` default 1× caps warp while any non-active ship is live | game.h:57; game.cpp `updateProximity` |

## 2. Alternatives considered

- **B — Building-anchored rails only** (air report §8.1): deck-box
  `isGrounded` + a rail flavor storing building id + offset. Bounded, and it
  delivers warp parking / recovery / Landed. But it **keeps the deck carry**
  for live physics, keeps every moving-ground special case, and does nothing
  for docking. Two mechanisms where the bubble gives one. *Escape hatch:* if
  the S0 audit uncovers body-centric assumptions far beyond §0's list, B is
  the fallback that still fixes the user-visible parking complaints.
- **C — Kinematic deck colliders (#848)**: make Bullet's solver see the
  deck's velocity. Tried twice already; this Bullet build passes ships
  through teleported kinematic bodies (orbital report §6.2). Even fixed, it
  only addresses the contact layer — no rails, no status, no docking. And
  the bubble **moots it**: inside the bubble the deck is static, which is
  exactly what the current static collider already is.
- **D — Keep the carry, fix bugs as they surface**: the user still hits
  problems after the fix string. The carry is a per-substep ray + engagement
  heuristic (brakes/thrust/EVA semantics, warp transitions, pause settling);
  that class of fragility is inherent to driving a ship kinematically from
  a contact test, not a bug backlog to burn down.
- **E — Building as a massless TerrainBody**: reuses the body frame machinery
  but drags in terrain/atmo/LOD weight, and gravity would point at the
  building (mu≈0) — the offset-gravity work is needed anyway. Not simpler.

**Recommendation: A (#850), staged as below, with B as the escape hatch.**
Sequencing note from the issue ("best done when the frame tree has a reason
to grow non-body nodes — docking"): the air platform's ongoing fragility *is*
the reason now; docking then inherits a finished mechanism instead of
co-designing it.

## 3. Physics in the bubble (what S0 must implement)

Ship at local (p, v) in a frame with origin acceleration A (inertial,
expressed in local axes) and **total** angular velocity Ω (local axes):

```
a_local = Oᵀ(F/m − A) − 2Ω×v − Ω×(Ω×p) − dΩ/dt×p
```

- **Gravity center**: F = gravity toward the *body centre*, which in building
  coordinates is `gravityCenter = −orientᵀ·pos` (the parent is the body's own
  frame, origin at body centre). For body frames this is 0 — identical to
  today. `applyGravity` reads it per part; the tidal lever is unchanged.
- **Total omega**: `Ω_child = ω_own + orientᵀ·Ω_parent`, accumulated in
  `UpdateRootRelative`. For the existing tree Ω == ω_own (assert it).
  Migrate the stasis / angular-velocity / fictitious readers to Ω.
- **Origin acceleration**: analytic per pose law, composed up the tree in
  `UpdateRootRelative` (parent spin transport terms included). Orbital:
  `−mu/r²·r̂` (Kepler). Air: `−rate²·(r cos lat)` cylindrical-radial in the
  rot frame, plus the 2ω_p×v and ω_p×(ω_p×p) transport of the parent spin.
  Patched-conic consistency: the body's inertial origin acceleration stays
  ≈0 (sun ignored), exactly as gravity ignores the sun today.
- **Euler term (dΩ/dt)**: zero for every v1 law — LVLH Ω = orbit normal·n is
  constant in LVLH axes; spin adds a constant local axis; the air ENU basis
  at fixed lat holds the planet axis constant. Assert constancy in the
  driven update; a future weathervaning law must add the term.
- **Air**: `airRelativeVel` becomes a root-space subtraction — ship inertial
  velocity (existing formula with Ω) minus air inertial velocity (body
  co-rotation + wind sampled at the ship's rot-frame position), expressed in
  the ship frame. The shared-origin assert is replaced by the full transform.

Sanity pins (unit tests, headless `test_frames` pattern): a ship at rest at
an orbital station's origin feels **zero** net acceleration; a ship at rest
on the platform feels g ≈ 8.7 m/s² down (±1e-3); a 3-level switch
(inertial → rot → bubble) preserves root pos/vel/attitude/spin to the
existing continuity tolerances.

## 4. Multiple buildings nearby — when to share a frame

A shared frame is physically correct **only for buildings mutually at rest**
(same orbit + attitude + spin phase; or same lat/alt/wind for platforms).
Two platforms at different altitudes ride different winds — in one shared
frame, one of the decks *moves*, and the moving-deck problem returns for
that pair. So:

1. **Per-building frame by default.** Each driven building gets a node:
   air → child of the body's rot frame; orbital → child of the inertial
   frame (matching `poseFrame()`). `soiTarget`'s nearest-wins already
   resolves overlapping bubbles.
2. **Grounding pin** (the robust overlap rule): a ship resting on a driven
   building's deck targets *that building's* frame, even if a neighbour's
   centre is nearer. "You share the frame of the ground beneath you."
   Implemented in `switchFrames` with the carry's existing ground-ray
   machinery, gated on low relative speed. Authoring validation: bubble
   radius > building bounding radius + margin, so a deck-ship is always
   inside its own bubble (no flicker).
3. **Authored clusters (deferred, designed-for)**: a `frame_group` key lets
   mutually-at-rest buildings share one frame node — one rails state for a
   whole station cluster / city, no handoffs walking between decks. Parse
   validates mutual rest. The frame node owns a *list* of buildings from day
   one, so clustering is wiring, not redesign. This is also the shape
   multipart rotating stations (#848 follow-up) will want: static core +
   spinning hab = one frame with a rotating sub-collider, or two frames.
4. Optional refinement, not needed with the pin: clamp a bubble to half the
   nearest-neighbour distance so bubbles tile without overlap.

Performance: one pose-law evaluation per driven building per tick (already
paid today) plus a cheap frame-node update; `soiTarget` scans the body's
children per ship per tick — fine for dozens, a spatial index when
DESIGN.md's cities arrive.

**Scope decision:** parking/grounded semantics in v1 apply to **air
platforms** (gravity holds the ship to the deck). In an orbital station's
bubble a ship is weightless — "resting on" it is really *docking*, deferred
to #848, which becomes "snap into the station frame + freeze". Orbital
bubbles still deliver: correct station-relative physics, static collider,
simplified spawn (v=0 in the station frame), and the docking runway.

## 5. Staged plan

Each stage lands independently; S0–S1 are behavior-preserving for existing
content (assert-guarded identities), S2 is the first visible change, S3 the
payoff, S4 the deletion. The deck carry keeps working until S4.

### S0 — frame core generalization (no building nodes yet)

- `Frame`: stored total omega + origin acceleration + gravity center,
  accumulated in `UpdateRootRelative`; a driven-node pose hook (enum or
  `StaticBuilding*` back-pointer; frame.h already forward-declares
  TerrainBody, add StaticBuilding). `UpdateOrbitRails` gains the driven
  branch (pose law supplies pos/vel/orient/ω/A — extend `buildingair.h` /
  `buildingorbit.h` to return vel/omega/accel; they stay pure glm headers).
- Migrate readers: `moveToFrame` stasis + angular velocity, `comStateIn`,
  `GetFictitiousAccel` (+ −A term), `applyGravity` (offset center).
  `airRelativeVel` full transform (drop the shared-origin assert).
- **Identity asserts**: for every body frame, Ω == ω_own, A ≈ 0, gravity
  center == 0 — the existing tree must not move by one ulp of behavior.
- Audit + fix the `|comPos()` body-centre readers (`inAtmosphere`,
  `isGrounded` radius, HUD altitude, science situation, surfmap position).
  Grep checklist: `comPos()`, `get_center_of_mass()` under `glm::length`,
  `frame->pos`, `isRotFrame()` gates.
- Tests: extend `tests/test_frames.cpp` (headless): 3-level tree, circular
  driven frame (rest ⇒ zero net accel), air-like driven frame (rest ⇒ g),
  switch continuity through all three levels, ω composition against an
  analytic two-spin case. `make test` + the full frame-switch e2e battery.

**Accept:** zero behavior change on existing content (identities hold, all
existing e2e green); new unit tests pin the generalized math.

### S1 — world consistency + SoI margins

- Explicit **world frame** (= active ship's frame). On change, re-express
  Bullet world content: terrain collision leaves (one transform write per
  leaf — meshes are anchor-relative, so this is cheap), building colliders
  (`poseFrame → world frame`), other live hulls (today they self-switch per
  tick; make the switch atomic for the whole world to kill the 1-tick
  mismatch docking/collision can see).
- **Per-frame SoI margin**: `min(kSoiMargin, soi/10)` — the flat 10 km
  margin currently eats a 10 km bubble whole (entry radius soi − margin ≤ 0;
  a ship could never enter). Body SoIs keep today's 10 km.
- `goOnRails` rule inside a building frame: **frozen (grounded) or exit** —
  a vacuum coast hands off to the parent inertial conic (preserves
  spawn-from-dock fleet warp); never a conic about the station with the
  planet's mu.
- Tests: unit (margin scaling, rail handoff rule); e2e: frame switch with
  terrain collision present stays aligned (land near a shell boundary).

**Accept:** world content provably aligned across switches; a small-SoI
node is enterable; existing pad/runway/orbital e2e green.

### S2 — building frame nodes live (fly-through)

- Create the node at building placement (air → rot child, orbital →
  inertial child), bubble radius authored with a sane default (~10 km),
  validation: radius > building extent + margin. `soiTarget` /
  `resolve_frame_by_soi` find it unchanged.
- Driven update wired to the pose laws (S0's extended headers); the
  building's `tickPose` and its frame node share ONE law evaluation per
  substep (no double propagate, no divergence).
- Fly-through e2e: ship crosses a station bubble and a platform bubble at
  1× and 10×; `[mvf]` continuity tripwires silent; trajectory continuous;
  deck collider static in the bubble (no ghost, no scrub).

**Accept:** bubbles enter/exit cleanly at both warp regimes; physics inside
is correct (S0 pins); no regression outside bubbles.

### S3 — grounded, parked rails, status, save (the payoff)

- `isGrounded` in a bubble: geometric deck test (deck top at local z=0 by
  the existing authoring convention; footprint + band + speed < 3 m/s) —
  no ray, the deck is static and known. Air platforms only (§4 scope).
- **Grounding pin** in `switchFrames` (§4 rule 2).
- Frozen rails in the bubble: `goOnRails` grounded branch already does it;
  verify rails warp rides the building (frame tree propagates at rails
  warp), `leaveRails` restores v=0, control wake works.
- Recovery (`isGrounded` gate), science situation, HUD Landed, orbit-plane
  suppression: verify they fall out; fix any that don't.
- Save/load: building frame id + local pose (buildings re-instantiate by id
  — existing discipline); **delete the `brakesOn` special case** (parked is
  v=0, re-derivable).
- e2e: spawn → park → rails-warp 1000× → wake (ship still on deck, same
  local offset); save/load round-trip parked; switch-away-and-back; recovery
  from the platform; two ships on one platform.

**Accept:** the air report's §7 "warp while parked" and `isGrounded`
cosmetics rows are closed; parked ship survives the warp/pause/save matrix.

### S4 — delete the carry and the moving-ground hacks

- Remove `tickDeckCarry`, `carriedBy/carryOffset/carryRot`, the substep hook,
  the pause-settle pass, the `v*h` pre-compensation.
- `surfaceVel/surfaceAngVel` readers (wheel law, EVA walk): relativize into
  the ship frame — zero when ship and deck share the building frame; the
  rot-frame fallback keeps working for any deck seen from outside its
  bubble.
- Restore deck **friction** (the frictionless workaround existed only
  because the deck moved through the ship's frame).
- Spawn-on-platform: place at v=0 directly in the bubble frame (delete the
  wind-velocity inheritance); spawn-from-dock likewise in the station frame.
- EVA on the deck: ordinary static-ground law (verify walk/jump).
- e2e: the 858–862 suite re-run **with the carry gone** (rest, drift,
  paused-present, parked-keeps-deck, tall-ship) plus thrust-release and
  brakes-off rollout on the deck.

**Accept:** 858–862 green without the carry; `grep tickDeckCarry` empty;
friction restored; spawn paths simplified.

### S5 — soak, corpus, docs

- Soak matrix: 1×↔10×↔rails parked; pause/unpause; quicksave/load; two
  platforms close together (overlap + pin); fly-in landing from orbit
  (inertial → rot → bubble double handoff); head-on collision bounce
  (856 pattern) in the bubble.
- DESIGN.md tick-off; #850 closed; #848 re-scoped to "docking = join the
  station frame" (the kinematic-collider question is moot in-bubble);
  authoring docs for the bubble radius; this report's status → IMPLEMENTED
  with deviations recorded.

### Deferred (designed-for, not built)

- `frame_group` clusters (§4 rule 3) — when a multi-building site is authored.
- **#848 docking**: docked ship joins the station frame, frozen rails =
  docked park; axial-port spin matching per the orbital report §5.
- Multipart rotating stations; player construction (frame node per placed
  building); balloon *parts* carrying their own frames (the DESIGN.md
  "atmosphere rails" item — the same mechanism generalizes to any driven
  entity).

## 6. Risks & gotchas

1. **kSoiMargin eats small bubbles** (found here): entry band is
   `soi − 10 km`; a 10 km bubble is unenterable. Scale the margin per frame
   (S1). This alone would have made the feature silently dead.
2. **`skipSameBody` in `railAdvance`** skips building frames (their `body`
   is the parent body). Accepted: warp-coasting conics ignore bubbles, same
   as they ignore discrete colliders today; frozen ships never advance, so
   parking is unaffected.
3. **Conic-about-the-station trap**: `goOnRails` in a vacuum bubble would
   build a Kepler conic about the station origin with the planet's mu —
   nonsense. S1's frozen-or-exit rule closes it; without it, space-dock
   fleet warp regresses to a 1× prox cap.
4. **Own-omega vs total-omega**: every stasis/angular/fictitious reader must
   migrate; missing one shows up as a velocity error proportional to the
   *planet's* spin at the ship's radius (≈175 m/s on Kerbin — loud, and the
   `moveToFrame` tripwires catch it). Identity asserts on the body tree make
   misses abort in tests, not in flight.
5. **Forgetting −A_origin** presses a station ship into the deck at ~8 m/s²
   (should be weightless). The S0 zero-net-accel unit pin catches it.
6. **Terrain-collider alignment** (§0 item 5): the old implicit invariant
   dies with bubbles; S1's explicit world frame replaces it. Do not skip —
   a ship falling out of a low bubble would fall through ghost terrain.
7. **Euler term**: v1 laws have constant local Ω (asserted). Weathervaning
   or spin-rate changes must add dΩ/dt — documented at the assert.
8. **Bubble entry ≠ capture**: the handoff absorbs relative velocity into
   coordinates; landing still requires braking it off, exactly like a
   planet SoI. No auto-dock until #848. Set expectations accordingly.
9. **Rails-warp hop through a bubble**: endpoint-only SoI test, the same
   documented blind spot as planets (#164). Accepted.
10. **Save format**: building id + local pose; a save referencing a building
    id the system no longer defines must refuse loudly (the #302
    pre-teardown validation pattern), not drop the ship at the body centre.
11. **Double law evaluation**: `tickPose` and the frame node must share one
    pose evaluation per substep, or collider and frame diverge by a substep
    — the exact shudder the substep drive was built to kill.
12. **`airRelativeVel`'s assert** is a tripwire, not a bug: it fires the
    first time a bubble ship samples air. Generalize it in S0, don't delete
    it silently.
13. **Precision**: unchanged — doubles, body-local coordinates, bubble
    offsets ~km. The `continuityTol` scaling (#489) already covers the
    larger root distances.

## 7. DESIGN.md intersections

- **"Buildings in space … docking … rotating … multi-part"**: the bubble is
  the docking mechanism and the cluster frame is the multipart mechanism.
  #848's kinematic-collider framing is superseded (in-bubble the deck is
  static).
- **"Atmosphere rails (balloon bases in venusian clouds)"**: frozen rails in
  a platform bubble *is* atmosphere rails for buildings. Free-floating
  balloon parts are the same mechanism applied to a driven entity — the
  speculative extension stays open.
- **"Player buildings"**: a placed building spawns a frame node; nothing in
  this plan hardcodes building frames to system-load time (keep it that way
  in S2).
- **"Ground vehicles"**: a rover on a platform uses the ordinary static-
  ground wheel law in the bubble — free.
- **"Cities, pops"**: per-building frame cost is one pose law + node update;
  `soiTarget`'s child scan is the first thing that needs a spatial index at
  city scale (noted, not built).
- **"N-body physics" (speculative)**: the patched-conic A≈0 convention for
  body origins stays consistent; a future n-body mode would feed real
  origin accelerations into the same S0 term.

## 8. Reflection — what would have made this easier

- frame.h should have **named its two invariants** ("origin == body centre",
  "parent chain inertial above the rot frame") the way it names the rail
  epoch. Both are load-bearing, both are invisible, and both are what this
  refactor generalizes. The orbital report's §8 already asked for the
  shell-boundary note; this is the same lesson one level up.
- The deck carry's comment block is excellent archaeology but it is a
  *symptom log*: five fixes for one missing abstraction. A one-line note at
  `tickDeckCarry` saying "this is a manual rest frame; #850 is the real one"
  (now present, via the issue) is what finally made the cost visible.
- The Bullet world having **no named frame owner** let the terrain-collider
  alignment invariant live implicitly for months. Naming the world frame
  (S1) is worth doing even if the bubble plan stalls.
- An audit checklist of "body-centric reads" (`|com|`, `frame->pos`,
  `isRotFrame` gates) maintained next to frame.h would turn S0's scariest
  stage into a grep-and-fix sweep.
