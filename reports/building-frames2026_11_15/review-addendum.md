# Building-frames plan — adversarial review addendum

Status: **BINDING** (2026-11-15). Corrections to `building-frames.md` from the
adversarial review pass (subagent, full code verification). Where this file
and the plan conflict, THIS file wins. The plan's verdict and stage structure
survive; the "short enumerable list" in §0 was roughly half the real list.
The original report is left untouched (immutable-snapshot convention).

## Corrections that change the implementation

### C1 (BLOCKER) — root_vel needs the parent-spin transport term

`Frame::UpdateRootRelative` (frame.cpp:100) computes
`root_vel = parent->root_orient * orient * vel + parent->root_vel`. For a
frame whose PARENT rotates (the air bubble under the body rot frame) the
origin also moves with the parent's spin: the correct composition is

```
root_vel = P.root_vel + P.root_orient * (orient*vel + cross(Ω_P_local, orient*pos))
```

Zero for every frame today (moving frames have inertial parents; rotating
frames have pos=0), ~177 m/s for a Kerbin-radius air bubble if omitted. The
plan's identity asserts do NOT catch it. S0 must add the term generally and
pin it with a two-spin analytic unit test. `system.cpp:41–53` (updateChain)
duplicates the composition for the body chain — leave it, comment it.

Driven-frame state convention (decided here, keep it dumb): the pose laws
supply pos/vel/accel **in parent axes**; the driven branch of
UpdateOrbitRails writes root_* directly:

```
root_pos    = P.root_pos + P.root_orient * p
root_vel    = P.root_vel + P.root_orient * (v + cross(Ω_P, p))
root_orient = P.root_orient * orient
root_accel  = P.root_accel + P.root_orient * (a + 2*cross(Ω_P,v) + cross(Ω_P,cross(Ω_P,p)))
omega_total = ω_own_local + transpose(orient) * Ω_P
```

`pos`/`vel` fields are still filled (local = transpose(orient) * parent-axis
value) for any direct reader; root_* is authoritative. Body frames keep
root_accel = 0 (patched-conic convention, consistent with ignoring the sun).

### C2 (BLOCKER) — aero altitude is body-centric

`applyAeroForce` (vehicle.cpp:~3282) and `comAirProfile` (~3131) compute
`alt = |com| − seaLevelRadius`. In a bubble |com| is metres → alt ≈ −R →
`airDensity` floors at sea-level density (drag.h:139): a platform ship would
be billed 65 kg/m³ instead of ~0.8. Altitude must come from the COM
relativized into the body rot frame — the `airReadingAt` pattern
(vehicle.cpp:~3164) already does this correctly; reuse it. Also the
air-follow branch asserts `frame == m_parent->frame || frame == rot_frame`
(~3325) — fires for bubble ships on air_follow bodies. Both in S0.

### C3 (MAJOR) — getNonRotFrame/getRotFrame semantics decide the conic cluster

Building node: `rotating = false`, `rot_frame = body->rot_frame`,
`body = parent TerrainBody`, plus a `StaticBuilding *building` back-pointer.
**`getNonRotFrame()` walks to `body->frame` for a building node** (and
`getRotFrame()` returns the body's rot frame via the stored pointer). Then
every consumer that lifts state via `comStateIn(getNonRotFrame())` — HUD
conic, orbit map, rel-vel markers, flyAscent, transfer planner, SAS holds,
inTerrainBand, dv-log, ascent logs — computes the correct BODY-centred state
with no per-site changes. `drawOrbitPlane`'s assert
(`m_parent->frame == frame->getNonRotFrame()`, render.cpp:504) then holds.
The two-branch pattern `if(isRotFrame()) … else if(hasRotFrame()) …`
(render.cpp:142/222, tick.cpp:909) also works: a building node has
hasRotFrame() true. Verify each site; do not assume.

Bonus: `goOnRails`' non-grounded branch builds its conic in
`getNonRotFrame()` — with the walk-up semantics that is the BODY inertial
frame, so the plan's "frozen-or-exit" rule falls out with NO special case:
vacuum bubble ship rails onto a body-centred conic (exits the bubble);
in-atmo bubble ship rails only when deck-grounded (frozen in the bubble).
The "conic about the station" trap is closed by the semantics, not a gate.

### C4 (MAJOR) — the rotating flag / non-inertial gate

Decided: building nodes are `rotating=false` (they are not body spin frames).
The fictitious-force gates (vehicle.cpp:2956, 5003) must become
`frame->isNonInertial()` = `rotating || building != nullptr` — a sky-fixed
station has Ω=0 but A≠0, so gating on Ω alone is wrong. isGrounded,
airRelativeVel, HUD surface mode key on the frame kind explicitly.
Save: `pose.rotating` bool + a new optional building-id field.

### C5 (MAJOR) — EVA/wheel ground laws need deck-local up and floor

`Kerbal::settleContact` (eva.cpp:36–58), applyEva's floor guard (~113–127)
and the wheel analytic fall-through (vehicle.cpp:~4440) use
`normalize(pos)` + `GetTerrainHeight` — in a bubble that snaps to the planet
surface ~6000 km away. In a building frame: ground = deck ray ONLY (no
analytic terrain fall-through — off the deck edge is free fall, correct),
up = deck-local +Z, floor = deck plane. settleContact (also run on every
kerbal load, save.cpp:1789) must learn the same. This is S3/S4 work, not
"verify walk/jump".

### C6 (MAJOR) — e2e 858–862 re-author at S2/S3, not S4

858/859/860/862 pin `[dbg]` rot-frame pos/vel (|v|≈86, |pos|≈R+56 km); once
deck spawns land in the bubble those CHECKs invert (|v|≈0, deck-local pos).
861 pins platform `live==1` while the parked ship is live; after S3 the
parked ship is frozen rails and `live==0` is CORRECT. Re-author expectations
when the bubble lands (S2/S3 acceptance). `[dbg]` alt becomes body-AGL
(relativized) so altitude CHECKs survive; vel stays frame-local (honest).

### C7 (MAJOR) — world frame: name it, but do NOT force-switch other ships

The terrain invariant is "any hull that can CONTACT terrain is in the rot
frame" (proximity + SoI geometry), not "leaves only exist near ground" —
camera LOD can build leaves while the active ship is inertial. S1:
- name the world frame (= active ship's frame); re-express terrain leaves +
  building colliders when it changes; transform NEW leaves into the current
  world frame at construction (they are baked at rot-frame anchors).
- do NOT force-switch neighbour hulls: a parked neighbour must stay frozen
  in its bubble frame or the warp payoff dies (force-switching it to the
  world frame makes isGrounded fail → canRail false → live forever → 1×
  cap). Contact implies frame agreement (two hulls within contact distance
  resolve to the same SoI), the same argument that keeps terrain aligned
  today. Document the invariant at the world-frame definition.
- tickPose's collider write composes poseFrame → world frame; when world ==
  the building's own frame that composition is the CONSTANT deck-local pose
  (identity for the platform). Per-substep tickPose remains for
  out-of-bubble consumers; the frame node snapshots per TICK like the rest
  of the tree (rail_time asserts enforce tree-wide epoch equality — a
  per-substep frame node would break them). Plan risk #11 is reworded
  accordingly: there is no divergence because the deck is static in-bubble.

### C8 (MINOR) — moveToRailFrame is not general; assert instead

It has no stasis terms — correct only because rail conics live in Ω=0
frames. Keep the invariant: assert omega_total == 0 for any rail frame
(frozen bubble rails never reach it). Do not claim generality.

### C9 (MINOR) — smaller items folded into the stages

- Bubble entry leaves m_parent unchanged → no SOI warp-drop, no flog event.
  Deliberate: a bubble is not a body change. (S2)
- Grounding pin runs every tick inside switchFrames; retain the ground-ray
  helper when the carry dies (S4). isGrounded in a bubble: deck test, then
  relativized terrain fallback (handles a low bubble over ground gracefully).
  Authoring: air bubble must not reach the terrain (alt − soi > max_height).
- Spawn beds: windy-bed `assert(frame->isRotFrame())` (vehicle.cpp:715) and
  speed_load's |target|−sea_r (~737) — route building-frame resolves to the
  parent rot frame. (S1/S2)
- Eccentric body-fixed stations have θ̇ = h/r² varying → Euler term ≠ 0.
  v1 authors circular only (buildings.cpp:428): validate BodyFixed ⇒
  circular at parse, assert in the law. SkyFixed+spin and air are constant-Ω
  for any orbit. (S0/S2)
- docktest fixture stays inertial — untouched.

## Verified correct (no change)

SoI margin math (risk #1 real, fix sound); switch-algebra shape given C1;
frozen-rails mechanism incl. save re-rail on load; zero-net-accel LVLH pin
(signs); constant local Ω for all v1 circular/air laws; airRelativeVel
shared-origin assert; proximity gate semantics; findPadOccupant and surfmap
ship dot fully relativized; recoverActive falls out via isGrounded;
gravity-center offset formula.
