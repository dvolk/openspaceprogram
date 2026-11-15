# Building frames (#850) — as-built notes

Status: **IMPLEMENTED** (2026-11-15). Companion to building-frames.md (plan)
and review-addendum.md (binding corrections). This file records what actually
landed and where it deviated from the plan.

## Stages landed

S0–S4 as planned; S5 verification done (suites below), docs in this directory,
#850 closed. Deferred items unchanged: `frame_group` clusters, #848 docking
(re-scoped: join the station frame), multipart rotating stations, player
construction, balloon-part frames.

## Deviations & discoveries (the plan did not know these)

1. **Building colliders live in their OWN frame, not the world frame.**
   The plan (and addendum C7) put all static content in the active ship's
   frame. That breaks a live NON-active ship on a deck while the active ship
   is far away in another frame: the hull is in bubble coordinates, the deck
   in the active frame's — they never meet (found via flight/861: the parked
   skiff fell through the deck on wake). The correct invariant:
   - terrain leaves → world frame (active ship's; camera-LOD means only a
     camera-near ship can contact them, and proximity keeps such ships in
     the same frame);
   - building colliders → the building's own bubble frame whenever live
     (any ship that can TOUCH the deck is inside the bubble, which is much
     larger than the deck). `StaticBuilding::writeColliderPose` writes the
     identity pose when live. This also kills the per-substep jitter (below).

2. **Per-substep deck jitter.** Composing the per-substep `posePos(tSub)`
   with the per-TICK frame snapshot moved the deck by up to v*step within
   each tick (~14 m at 10x) and shook parked ships off. Writing the deck's
   constant bubble-local pose directly (it is static by construction) fixes
   it. Plan risk #11's "share one law evaluation" was the wrong diagnosis;
   the addendum C7 rewording was right.

3. **Bullet broadphase is built at add time.** A collider teleported AFTER
   `AddPhysicsBody` is invisible to rays until the first sim step. The gate
   now poses the deck BEFORE adding it (`gateBuildingColliders`), so the
   boot/commit rails check and isGrounded see it immediately.

4. **Boot/commit ordering.** Rails-warp boot and save-commit now drive
   building poses + gate + (boot) capture spawns into their SoI frames
   BEFORE the canRail/goOnRails check, so a deck spawn freezes at boot and a
   saved frozen ship re-freezes on load. `driveBuildingPoses` /
   `gateBuildingColliders` are named Game methods (were inline in tick).

5. **Deck friction restored** (pulled forward from S4 to S2): the
   frictionless hack existed only for the pre-bubble moving deck. In the
   bubble the deck is genuinely static, so ordinary friction grips. Without
   it, a hull whose wheels don't touch (tail-standing skiff) had zero lateral
   grip and slid off at warp.

6. **Spawn attitude (user-reported "jump on first unpause").** The carry had
   propped up an unstable equilibrium: a spaceplane on its tail tips on the
   first physics step (it always did on the pad; the platform hid it). Air
   platforms now spawn gear-equipped ships horizontal on their gear
   (`has_gear` branch in place_vehicle_def), gearless hulls vertical. #851
   updated; surface pads still use the old one-rule-per-site pose.

7. **EVA (user-reported).** Three body-centric reads fixed:
   - `kerbalEVA` placed the walker at the planet's terrain radius in bubble
     coordinates (a ~6000 km teleport). Now deck-local, at capsule height
     (pad semantics: step out of the hatch, fall to the deck).
   - `Kerbal::restAlt` assumed terrainMargin; the deck's effective margin is
     different, so the floor guard fought the contact in a bounce limit
     cycle. restAlt now reads the deck's `effectiveHullMargin` (env-respecting,
     no hardcoded values — the margin-removal debate is #865).
   - The EVA orbit camera's "up" was normalize(com) — the radial from the
     BUILDING origin in a bubble, swinging as the kerbal walks (camera roll).
     Now the bubble's +Z (deck normal).
   - EVA walk/wheel ground laws: the surfaceVel machinery is deleted entirely
     (ground is stationary in the ship's frame everywhere now); the analytic
     terrain fall-through is disabled in bubbles (off the deck = airborne).

8. **Spawns simplified.** Air-platform spawn: build at the deck's rot-frame
   pose, `moveToFrame` into the bubble, zero velocity IN the bubble (zeroing
   before would transform a rot-rest velocity, which is the wind). Orbital
   spawn: bare bubble-local pose (0,0,-50) + identity + v=0 — no Kepler
   propagation needed. Both are the pad case now.

## Second adversarial pass (post-implementation) — fixes landed

1. **flight/855 was missed in the re-authoring** (cross-frame distance check,
   same class as 857). Re-authored to bubble-local bounds; green. The first
   verification sweep had not actually re-run it — the suite list below is
   the corrected, fully-run one.
2. **EVA floor guard was an infinite deck plane**: off the deck edge the suit
   snapped back to z=rest and hovered. `Kerbal::overDeck()` (a ray down hits
   the deck body) now gates the snap AND the grounded test in
   armInput/settleContact — off the edge is free fall, verified by walking a
   kerbal off (tmp/eva_edge.log: walks to −35 m, falls to terminal velocity).
3. `posInBodyRot` falls back to the body's INERTIAL node when there is no rot
   frame (was: identity — a latent body-centric hole for airless bodies).
4. Assert in UpdateRootRelative: a frame parented under a bubble must compose
   accel/gravity centre (none exists yet; the driven branch recurses).
5. Stale comments fixed (tick substep drive, writeColliderPose header);
   grounding-pin comment no longer overclaims (stay rule, not target rule);
   dead `bubbleSoi` field removed; [planelog]/Orbital window label bubbles
   "bubble"/'B' instead of "inertial"/'I'.

## User-reported: cross-system load refused a bubble pose (fixed)

Quicksave on the platform → return to menu → load: "saved ship 'skiff' has a
pose in building 'venus_cloud_platform' that Venus does not carry". The
cross-system branch of `Game::loadFrom` BUILDS the save against the incoming
system before `installSystem` places its buildings, so `resolvePoseFrame`
saw empty pads. (The boot `--load` path was unaffected: it switches system
first.) Fix: `place_world_buildings` is now idempotent (skips ids already on
the body), and the cross branch places the incoming buildings BEFORE the
build; `installSystem`'s later placement is a no-op for them. Pinned by
save/867 (fixture save_platform, title "Load last save" across systems);
the EVA-kerbal variant verified manually too.

## User-reported: dead controls after walking off the deck (fixed)

The off-edge floor-guard fix above had an early `return` that exited
`applyEva` entirely once the falling suit's z dropped below the snap
threshold — skipping the SPACE-mode RCS and camera-facing laws every
substep. The suit fell tumbling, deaf to W. The guard now only fires when
actually over the deck (`!onDeck() || overDeck()` in the condition); off the
edge it falls through to the mode laws. Verified: rcs=1 and a steady
camera-facing attitude during the fall; crew/866 extended to walk off the
edge and pin the burns.

## Verification

- `make test`: 64 binaries green, including new headless frame pins
  (two-spin transport, weightless-at-station, platform-feels-g, 3-level
  switch round trips, body-frame identities).
- Feature suite: flight/854–865 (re-authored 855, 857–862 for bubble-local
  expectations; new 863 dock-bubble, 864 parked-rails-warp at 1000x, 865
  save/load) + crew/866 (EVA stands/walks/no-bounce) — all green.
- Regressions: pad 160, runway 850, SoI warp 18, dock save/load 79, EVA
  26/215, orbital 854/856 — all green.
- User verification: EVA on the platform with heavy_asp (spawn, camera,
  bounce) confirmed fixed; off-edge fall verified manually.

## Follow-ups

#848 re-scoped (docking = join frame), #851 updated (spawn attitude),
#862 wheel-substep stability, #863 rails-warp bubble capture, #864 EVA
offset vs stack width, #865 margin removal debate.

## Reflection

The frame algebra was already general; the cost was in the *consumers* that
assumed origin == body centre (gravity, aero altitude, EVA floor, camera up,
spawn placement) and in the *world-content alignment* invariant that had
lived implicitly. Naming the world frame and writing the alignment rules
down (terrain → world frame, decks → own frame) is what made it tractable.
A one-line invariant comment at each frame field would have saved the
adversarial round-trip; the fields now carry them.
