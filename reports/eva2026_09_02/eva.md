# EVA scope: kerbal on ground + in space

Scoped 2026-09-02, then implemented. Debug-scope EVA (extravehicular
activity): a green "cucumber" placeholder character that can walk on a body's
surface (ASDW + space to jump) and fly in space with an RCS-like mode
(ASDW translation, QE yaw, facing the camera). Deliberately shaped so the
later kerbal features — boarding ships, inventory, animations — can grow out
of it without rework.

## Core design decision: a kerbal is a one-part `Vehicle`

`~Vehicle()` is already `virtual` — inheritance was anticipated. Making the
kerbal a `Kerbal : Vehicle` subclass with a single part buys the entire
existing machinery for free:

- **Frames/SOI**: `switchFrames()`, gravity + Coriolis/centrifugal
  (`applyGravity()`), SOI warp-drop — all per-ship, work on any Vehicle.
- **Rails**: a standing kerbal parks `railFrozen` (grounded); a drifting
  kerbal coasts its conic. Idle kerbals park on `select_ship()` like ships.
- **Fleet/UI**: F6 cycling, SHIPS window, HUD/ORBITAL/SURFACE readouts (all
  computed off the `controller` part axes — a one-part kerbal just works),
  picking (RMB part window is harmless now and is where the kerbal
  right-click UI hooks in later), render pass, teardown.
- **Physics**: one rigid body, convex hull of the cucumber mesh — the
  existing `RegisterObject` path, capsule-ish hull vs terrain/pad triangle
  meshes. No physics.cpp changes needed for the shape itself.

Rejected alternative: a separate kerbal list beside `g.ships` would
duplicate the gravity/frame/rails/camera/draw loops for zero benefit. KSP
itself models EVA kerbals as vessels.

## The shape

- `utils/gen_kerbal.py` generates `res/kerbal.obj` (capsule: radius 0.35 m,
  cylinder 1.0 m, hemispherical caps, ~1.7 m tall, centered origin, long
  axis = +Z like every part) and `res/kerbal.png` (flat green).
- `utils/gen_parts.py` gains a `kerbal` catalog entry: mass from the mesh
  volume at a kerbal density (~90 kg), no torque/thrust/capacity.
- `res/ships/kerbal.json` is a one-part ship def, so the fleet path
  (`--ship res/ships/kerbal.json`, fleet.json) spawns kerbals with zero
  extra code.
- `Ships::place_ship` constructs a `Kerbal` instead of a `Vehicle` when the
  def contains a kerbal part.

## Ground mode — ASDW walking + space jump

**Grounded detection is two-layered.** Terrain collision meshes are a legacy
camera-proximity feature: only max-LOD leaf patches (`depth >= max_depth`)
carry collision, and subdivision is screen-size driven, so contact alone
can't be trusted where LOD hasn't caught up (this becomes ship-proximity
based at some point; until then EVA works with what exists):

1. **Contact**: a `contactTest` query on the kerbal body (works against the
   pad mesh too, which is always present — fleet kerbals spawn on the pad).
2. **Analytic**: `TerrainBody::GetTerrainHeight` is exact everywhere; the
   kerbal is also grounded when its altitude above the analytic surface is
   within the standing band. The analytic height doubles as a position
   clamp so the kerbal can't fall through unloaded LOD.

Per tick, armed once and applied before every substep (the thrust pattern):

- **Walk**: WASD → camera-relative direction projected onto the surface
  tangent plane (up = radial). Force-steer the horizontal velocity toward a
  walk-speed cap (~2.5 m/s); no input → damp horizontal velocity.
- **Jump**: space, edge-triggered; an upward (radial) velocity kick.
- **Upright**: authority-bounded torque aligning the kerbal's up axis to
  the local vertical, yaw toward the walk direction — the same law style as
  the ship's `slewToward`/`killRotStep`.

## Space mode (RCS) — auto-engaged when not grounded

- **Facing**: target attitude = camera basis, nose (local +Z) along
  `camera->forward`; Q/E adds yaw about the view axis. Authority-bounded PD
  torque (fixed EVA authority — the kerbal has no reaction wheels).
- **Translation**: WASD forces along the camera axes with a soft speed cap
  (~3 m/s). No fuel in the debug scope; `partResources` is the hook.
- **Camera (load-bearing)**: the orbit camera normally chases the
  controller's attitude; with a kerbal that's a feedback loop (kerbal faces
  camera, camera rides kerbal). While a kerbal is the focus, `camera->ref`
  is the surface frame instead — mouse orbits around the kerbal, kerbal
  turns to face the view.

## Spawn / switch

`V` toggles EVA: from a ship, spawn (once) or re-select the kerbal and take
control; from the kerbal, hand control back to the last ship. Spawning: on
a surface, on the terrain below/beside the ship at the analytic rest
height; in free fall, co-located beside the ship with matching velocity.

## Rails / warp

Standing kerbal = `railFrozen` park; drifting kerbal = conic coast —
`canRail()` already classifies both. A kerbal mid-jump is suborbital and
refuses rails warp exactly like a descending ship. No new code.

## Verification

- `--eva-log`: `[evalog]` lines (mode, pos, vel) for e2e assertions.
- Unit tests for the pure math (target attitude, grounded classification,
  walk-direction projection).
- e2e: pad + orbit scenarios with the kerbal def; sim-pressed WASD/space
  must move it; `make clean && make test && make e2e` green before commit.

## Future hooks

- **Boarding**: proximity + control handoff (`select_ship` template), later
  a constraint to a hatch part; a kerbal state enum lives on the subclass.
- **Inventory**: `partResources` + `ResourceType` already exist per part.
- **Animations**: mesh/texture stay on the PartDef/Model path — swap the
  cucumber for a real model without touching physics or control.
- **Crew**: a capacity field on `PartDef` makes ship↔kerbal transfers
  data-driven.
