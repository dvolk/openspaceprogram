# Are we re-inventing a physics engine? — issue #1092, 2026-11-25

Snapshot of what the physics code actually is, taken to answer #1092 ("our kerbal,
ladders, airplane gears, landing legs — could that be offloaded to Bullet?").
Every claim below is a file:line in the tree at `eb170e14`; the greps that re-derive
them are at the end.

## Short answer

Bullet is not doing physics in this game; it is doing **integration and collision
bookkeeping**. It owns exactly one thing per live ship — a single rigid body — and the
static world it can hit. Every *contact law*, every *actuator*, every *force law* the
issue names is ours, and the reasons are written in the code, not guessed.

So the honest answer to "are we re-inventing a physics engine" is: **we are inventing
force laws, not a solver.** A solver is the part Bullet is good at and we already use.
What we wrote ourselves is the part Bullet does not have: a suspension/tire/stiction law,
tidal gravity per part, one-way actuator allocation, per-part silhouette aero, patched
conics on rails, and a docking merge with momentum bookkeeping.

Below the fold — why it was done this way, benefits, drawbacks, cost of switching.

## What Bullet actually does today

| Role | Where |
|---|---|
| One world, **gravity deliberately zero** | `physics.cpp:182` — gravity is a per-part force, see below |
| `btDefaultCollisionConfiguration` / `btCollisionDispatcher` / `btDbvtBroadphase` / plain `btSequentialImpulseConstraintSolver`; no solver-iteration tuning anywhere | `physics.cpp:176-180` |
| `setApplySpeculativeContactRestitution(true)` — no comment says why | `physics.cpp:183` |
| One step per OSP substep: `stepSimulation(h, 1, h)`, and Bullet clears the force accumulator on exit, so the tick re-applies everything before every call | `physics.cpp:202` |
| A ship is **one `btRigidBody` whose shape is a `btCompoundShape` of the part convex hulls** — "No per-part bodies, no welds" | `vehicle.h:252-265`, `rebuildCompound` `vehicle.cpp:1559` |
| Terrain: static `btBvhTriangleMeshShape` per collision leaf of the cubed-sphere patch tree, only at `max_depth` | `physics.cpp:252-253`, `terrain.h:252` |
| Buildings/pads: mass-0 **static** convex colliders, gated by proximity | `buildings.cpp:501`, `game.cpp:2259` |
| Queries: `rayTest` (leg/ground contacts, `isGrounded`, the ground probe), `contactTest` (the suit's grounded check), `rayTestSingle` (UI picking) | `groundContact` `vehicle.cpp:5182`, `isGrounded` `8797`, `contactTest` `physics.cpp:382`, `pick.cpp:83` |
| Friction coefficients on hulls: default `4.0`, deliberately not Bullet's `0.5` | `physics.h:69` |
| Hull/terrain collision **margins default to 0**, env-tunable, with a long history comment | `physics.cpp:23-40` |

And what Bullet has that we use **zero** of (verified by grep over `src/`):
`applyCentralImpulse`/`applyImpulse`/`applyTorqueImpulse` — 0 hits;
`btMultiBody`, `btPoint2PointConstraint`, `btFixedConstraint`, `btHingeConstraint`,
`btKinematicCharacterController`, CCD (`setCcdMotionThreshold`, swept sphere) — 0 hits;
collision filter groups/masks — none (the only `needsCollision` override is the ray-skip
callback).

## What is hand-rolled, and what Bullet feature it stands in for

| Hand-rolled law | Where | Bullet stand-in | Why ours instead |
|---|---|---|---|
| Per-part Newtonian gravity + rotating-frame (Coriolis/centrifugal/stasis) terms, with the COM-offset torque cancelled | `applyGravity` `vehicle.cpp:3291`, `GetFictitiousAccel` `frame.h:170-183` | `setGravity` | Gravity must be per part so the lever gives tidal torque — "the only legitimate external torque on a rigid ship" (`vehicle.cpp:3310-3311`). A uniform world gravity cannot say that. |
| On-rails Kepler propagation; Bullet is not stepped at all at warp ≥ 11 (`kRailsWarp`) | `railAdvance` `vehicle.cpp:8971`, `propagateKepler` `orbit.h:454`, branch at `tick.cpp:520`, `game.h:57` | Bullet integration for coasters | Exactness and speed: "A warp stop cannot model an impact (Bullet would tunnel at this speed)" (`vehicle.cpp:9038`). |
| The ground law for gear and legs: suspension spring + damper, slip-based tyre, **static hold** against the substep's own accumulated push, one friction circle, momentum clamps | `wheel.h` (whole file), caller `Vehicle::applyGroundForces` `vehicle.cpp:5218`, contact ray `groundContact` `5182` | friction manifolds / `btMultiBody` wheels / slider-constraint legs | `wheel.h:17-24`: "resting on Bullet contact instead would scrub and pump at coarse substeps." Stiction: a slip-proportional tyre parks a hull at a permanent crawl (#1184, `wheel.h:150-171`). |
| Brakes (a held state that turns the stiction fore-and-aft on) | `brakesOn` `vehicle.h:2164`, applied `vehicle.cpp:5290,5358` | rolling resistance / brake torques | Same law, one more term. |
| EVA walking: frictionless feet + a central force `mass*(walkDir*2.5 − v_tangent)/h` clamped at 10 m/s² | `eva.cpp:557-558`, `eva.h:17-21` | `btKinematicCharacterController` | `eva.h:17-21`: foot friction pairs with the COM walk force "into a tipping couple the attitude authority cannot out-torque." |
| EVA jump (direct velocity set + latch) and the floor guard (analytic snap to rest height) | `eva.cpp:550`, `restAlt` `eva.cpp:57-64` | contacts keeping the capsule up | The margins must be the ones the contact uses or "the floor guard fights the solver in a bounce limit cycle (#850)" (`eva.cpp:62`). |
| Ladder hang/climb: a PD position servo pulling the suit to a moving hold point off the ladder line, segment transfer with hysteresis | `nearestLadder` `eva.cpp:171`, `resolveLadder` `217`, `applyLadder` `311` | `btPoint2PointConstraint` | `eva.cpp:30-32`: "the hang is a force servo, so imperfectly placed segments compose — the kerbal is PULLED to the line, and a transfer across a gap or a lateral offset is a smooth slide, not a teleport." |
| Attitude control: stick/SAS demand, slew and kill-rot laws, one torque allocation over reaction-wheel and control-surface rows; **wheels have no momentum state yet** | `applyRotationForce` `vehicle.cpp:5454`, `actuateAttitude` `8192`, `actuators.h` | nothing in Bullet | `vehicle.cpp:8276`: "Until that store exists there is no wheel speed to multiply, so torque is the best proportionality this model can state" (#579 still open). |
| Engine gimbals: a rate-limited universal-joint law; thrust applied at the engine offset | `gimbal.h:1-22`, `applyThrustForce` `vehicle.cpp:3350` | `btHingeConstraint`/`btSliderConstraint` per engine | Pure math so it can be pinned without a world; a joint would be kinematic bookkeeping we then have to undo. |
| RCS ring: NNLS allocation over one-way nozzles, solving for force and zero spin; wheels → TVC → RCS priority | `rcs.h`, `rcsPlan` `vehicle.cpp:5670`, `applyRcsForce` `5740` | — | `actuators.h:43`: "Merging the two into one weighted solve is how a ship ends up refusing to translate because it is busy turning." |
| Aerodynamics: per-part silhouette projection, union drag area, centre of pressure, lift with incidence and local flow, side/crossflow, induced drag, control-surface steering | `applyAeroForce` `vehicle.cpp:3602`, `drag.h`, `vehicle.h:289` | — (Bullet has no aero) | A hull's silhouette is not its collision hull: "not a convex hull drawn over the merged vertices, which invents airframe between the parts". |
| Parachute: analytic canopy Cd·A at an authored anchor with a weathervane, deployment accel limiter shrinking effective area; the canopy is not a collider | `vehicle.cpp:4719-4751`, `constants.h:131` (`kChuteAccelLimit = 50`) | cloth / a jointed canopy | The limiter is physical *and* numerical: it keeps the weathervane oscillation inside the substep's stability envelope. |
| Docking: intent plus three geometric gates (capture distance, 5° cone, 0.15 m/s), then a rigid **merge** with a pose snap, inelastic velocity blend and angular-momentum bookkeeping | `vehicle.h:2481-2503`, `dockRead` `vehicle.cpp:6453`, `dockSituation` `6537`, `absorbShip` `6630` | `btPoint2PointConstraint`/weld | There is no joint in the game at all: a docked pair becomes one compound. The gates exist because the merge is "a weld with no damper" (`vehicle.h:2479`). |
| Undock/staging: extract a subtree into a new hull, inherit its COM velocity and the parent's spin — **no separation impulse** | `extractSubtreeAsShip` `vehicle.cpp:6870` | spring/ejection impulses | Deliberate: decoupling impulse is a design decision that has not been taken, not a physics gap. |
| Frame switching (SOI hops, building bubbles): the world is re-expressed under the ship rather than the ship moved through a fixed world | `moveToFrame` `vehicle.cpp:8557`, `switchFrames` `8707`, `syncWorldFrame` `game.cpp:2239` | — | Patched conics need patched frames; a single inertial world cannot hold an orbiting SoI bubble. |
| Mass properties: refresh the one compound's COM/principal frame as fuel burns, gated at 0.01 m / 1e-3 mass | `rebuildCompound` `vehicle.cpp:1559`, `refreshMassProps` `1734`, `vehicle.h:428-438` | Bullet recompute | `vehicle.h:428-438`: re-solves the mirror in place and "allocates nothing". |
| Every hull is `NeverSleep`, sleeping thresholds 0 | `vehicle.cpp:1702`, `physics.cpp:592` | solver deactivation | Mechanically forced: support is an explicit force each substep, not a resting contact. |
| Grounded/proximity predicates computed analytically or by ray, not from persistent manifolds | `isGrounded` `vehicle.cpp:8797`, `gapTo` `8754`, `distanceTo` `8769` | contact cache | A ship floats above its own collision hull (ride height), so there is often no manifold to read. |

## Why it was done this way

The reasons are in the comments, not in folklore:

1. **Resting contact cannot carry a ship at our substep.** Substeps go from 1/60 s
   (`cli.h:253`, `physics_hz = 60`) up to `kMaxSubStep = 0.1` s (`timestep.h:13`) as warp
   rises — 0.0556 s at warp 10. A stiff spring explicit over 0.056 s either over-corrects
   or never arrives: `wheel.h:82-90` computes the damper's own stability bound
   (`h > m/(n*wheel_damp)` ≈ 0.073 s for shipped gear) and `wheel.h:110-125` the brake
   gain's. So we keep the float and pay the visual ride height instead
   (`wheel.h:17-24`, #1072).
2. **Per-part physics is the point.** Gravity, drag, lift, thrust, RCS and fins all act at
   a part's offset, and the *differences* are the game: tidal torque, the aero centre of
   pressure, TVC's lever, the RCS ring's zero-spin solve. Bullet gives one body; the
   per-part decomposition is ours either way, so the laws that need it are ours too.
3. **The actuators are one-way and budgeted.** Nozzles push but never pull, wheels have a
   torque cap, fins must not be charged twice ("would charge the hull twice for one fin",
   `vehicle.cpp:8235`). That is an allocator, and Bullet has no such concept.
4. **Warp.** At rails warp nothing is integrated; at warp 10 an impact would tunnel
   (`vehicle.cpp:9038`). A solver's envelope is set by the smallest feature and the largest
   speed; the game's range is six orders of magnitude.
5. **One body per ship was a deliberate fix, not an omission.** `physics.cpp:30-33`
   records the era when "every WELDED part was its own rigid body" and "the solver's
   per-substep impulse tumbled a thin wheel disc"; `--compound-check` (`cli.cpp:907-913`)
   still exists to print "the weld wobble the compound removes".
6. **Some Bullet features do not work as needed here.** A teleported kinematic body
   "did not block the ship in this Bullet build", so buildings are static colliders and
   `MakeKinematic` sits unused with that warning (`physics.cpp:553-559`). Bullet also has
   "no collision algorithm for concave-vs-concave pairs, so anything that moves must stay
   convex" (`physics.cpp:316`) — which is why terrain collision exists only at max-LOD
   leaves and why an analytic fall-through guard stands beneath it.

## Benefits we get

- **Warp.** Time-accel to 10 with physics, 11+ on rails. Solver stiffness would set the
  ceiling; here the ceiling is a force-law stability bound that is written down and
  asserted.
- **Determinism and testability.** Every law is a header-only pure function that can be
  pinned without a world or GL: `wheel.h`, `gimbal.h`, `rcs.h`, `drag.h`, `actuators.h`,
  plus the 563-case e2e battery driving the real tick loop.
- **The laws match the design.** Suspension travel, ride height, brake stiction, one-way
  bells, per-part air, deployment-limited chutes are game concepts, and they are
  *authored* (`res/data/parts.json`), not solver parameters.
- **Predictable cost.** No manifolds to grow, no island sleeping to fight; the allocation
  discipline (#645/#668/#581: `rcs::Scratch`, `InlineVec` rows, `SlotProbe` by reference)
  is possible precisely because we know every byte touched per substep.

## Drawbacks, honestly

1. **Two physics systems meet at a seam, and the seam is a tuning invariant.** The ground
   law keeps the hull's collision hull *off* the ground by the ride height
   (`--leg-log gap=` ≈ 0.05-0.06 m for both shipped parts), so Bullet contacts usually do
   not fire for a parked hull. If the suspension bottoms — compression clamps at travel,
   `vehicle.cpp:5255,5338` — the ride allowance `(travel - compression)` goes to zero, the
   wheel's convex child meets the surface, and Bullet's impulses would stack on top of a
   maxed spring. The margin comment knows the geometry — with the old 0.5 m margin
   "wheels bottomed out and the hull rode as a bump stop" (`physics.cpp:37`) — but nothing
   in the ground law says which support wins in that regime, or whether both are ever live
   at once.
2. **Ordering is load-bearing.** `GetAccumulatedForce` must be read before the ground
   applies its own share or "a read inside either one would have the ground cancel its own
   hold" (`vehicle.cpp:5243`, with the read itself at `physics.cpp:460`), and Bullet clears
   the accumulator on exit. That is three files that must stay in agreement.
3. **Friction has three different answers.** Our ground law caps at `kGrip = 0.9`
   (`wheel.h:34`); hull hulls carry `4.0` (`physics.h:69`) for the cases where Bullet
   contacts *do* run; the suit is 0. Filed as #1195.
4. **We do not get free things Bullet would give:** no CCD (so fast thin things are
   unguarded and warp impacts are parked instead), no persistent manifolds (so "is it
   grounded" is a ray every substep), no joints (so a landed stack on a slope is only as
   good as our static hold, #1184), no restitution tuning.
5. **Momentum is not always conserved where a solver would have conserved it.** The dock
   snap moves mass without moving momentum, so "the weld carries an unmodelled impulse" —
   measured ~30 kg m²/s to ~5.1e3 kg m²/s — acknowledged and deferred to #1167
   (`vehicle.cpp:6686`).
6. **Documentation drift.** `vehicle.cpp:32` claimed "One rigid body per part, welded
   parent-first in the def's construction order" — a design abandoned years ago, sitting on
   top of the function that builds one compound. Fixed in this pass.

## Cost of switching

Three different switches hide in the issue's question, and they cost very differently.

**(a) Legs and gear onto Bullet contacts.** Cheap to try, probably a loss. You would delete
`wheel::force`'s spring/tyre/hold terms and let the solver support the hull, which means
(a) removing the ride height so the hull actually touches, i.e. the visual float goes away
(#1072's ask); (b) accepting solver behaviour at 0.056 s substeps, which is exactly what
`wheel.h:17-24` says scrubs and pumps; (c) re-deriving brakes and stiction, which Bullet
does not model as a latched hold. Estimated: days to try, and the e2e cases
`flight/770`, `775`, `850-853`, `1158-*`, `1184` are the judge. The one genuinely attractive
part is bump stops and the bottom-out regime, which Bullet would handle for free.

**(b) The kerbal onto `btKinematicCharacterController`.** Small, and probably a real win
for walking on terrain — but the ladder and the suit jets are the substance of EVA, and a
character controller does not have them: the ladder is a PD servo to a moving hold point on
a *moving ship* in a bubble frame (`eva.cpp:217-345`), and the capsule's friction is zero
by design (`eva.h:17-21`). Switching the walking half means the suit ends up with two
ground models, which is the same seam problem we already have with wheels. Suggested
sequence: only after the ground-vehicle drive channel lands, since that forces one ground
model to be chosen.

**(c) Docking onto a constraint instead of a merge.** This is the only place a Bullet
feature would replace design rather than re-implement it, and it is already argued in the
code: #848 ("docking = join the station frame") chose frames over joints, `MakeKinematic`
is parked with a warning for exactly this follow-up (`physics.cpp:553-559`), and #1167 has
already quantified the merge's missing impulse. A real joint would fix #1167 for free and
give multi-station clusters; it would also mean ships that are several bodies, which is
the decision that took the compound body away in the first place. Cost: large, and it
should be driven by wanting clusters, not by wanting a joint.

## What I would change now (cheap, and this pass does the first two)

- Fix the stale per-part-body comment (`vehicle.cpp:32`). **done here**
- Add one line to DESIGN.md saying what Bullet is and is not, so the question stops
  needing to be asked. **done here**
- Filed: #1195 (three unrelated friction coefficients). Two seams worth a look and not yet
  filed: the bottom-out double-support above, and the uncommented
  `setApplySpeculativeContactRestitution(true)` at `physics.cpp:183`. The missing CCD is not
  a bug — it is warp working as designed — but it is why `vehicle.cpp:9038` exists.

## Re-deriving the claims

```bash
# the Bullet API we do NOT touch
grep -rn "apply\(Central\)\?Impulse\|applyTorqueImpulse\|btMultiBody\|btPoint2Point\|btFixedConstraint\|btHingeConstraint\|btKinematicCharacter\|CcdMotion\|SweptSphere" src/ | wc -l    # 0
grep -n "stepSimulation\|setGravity\|SpeculativeContactRestitution" src/physics.cpp
# one body per ship, and what its shape is
grep -n "btRigidBody hull\|btCompoundShape" src/vehicle.h
grep -n "void Vehicle::rebuildCompound" src/vehicle.cpp
# the ground law and its callers
grep -n "wheel::force\|applyGroundForces\|GetAccumulatedForce" src/*.cpp src/*.h
```
