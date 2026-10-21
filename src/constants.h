// constants.h -- the shared gameplay constants, one documented home.
// Each entry states its units, what changing it changes IN THE GAME, and
// roughly where the effect is implemented. Pure values (no includes), so
// the pure-math headers and the tests can all pull them in.

#pragma once

// -- frames / SOI ------------------------------------------------------------

/* Dead band [m] on both sides of every frame SOI: a ship leaves its frame
   above soi + margin and enters a child below child soi - margin, so a ship
   loitering exactly at a boundary cannot flip frames every tick.
   Changes: where surface <-> orbital physics flips, and the science
   HighOrbit boundary with it.
   Where: Vehicle::soiTarget (vehicle.cpp). */
inline constexpr double kSoiMargin = 10e3;

/* Floor [m] for the near-body (rotating-frame) shell above sea level, for
   bodies whose atmosphere is shorter (or absent). 100 km is the shell every
   shipped body was authored with, so shipped extents do not move.
   Changes: the surface-frame extent, the science HighOrbit boundary and the
   orbit-scenario spawn radii on small or airless bodies.
   Where: shellEdge (bodylimits.h). */
inline constexpr double kMinShell = 100e3;

/* The low-orbit science band's width above the atmosphere top, as a
   fraction of it: the shell edge is max(kMinShell, (1 + this) * top()).
   Changes: how far above the air LowOrbit extends before HighOrbit, i.e.
   the near-body shell size on tall-atmosphere bodies.
   0.2 mirrors kFlyingLowFrac: FlyingLow is the bottom 20% of the air,
   LowOrbit a 20%-of-top band above it.
   Where: shellEdge (bodylimits.h). */
inline constexpr double kLowOrbitFrac = 0.2;

/* The `inertial-orbit` scenario's spawn radius, in near-body shells:
   1.25 x the shell puts it just outside the rotating frame (the scenario's
   whole point). The inertial-SOI lift keys off it so the bed always resolves
   inside the body's own frame -- without that, lifted tiny moons would spawn
   the ship in the PARENT's frame at walking speed.
   Changes: the inertial-orbit spawn radius and the inertial SOI floor on
   tiny bodies.
   Where: kScenarios (vehicle.cpp), inertialSoi (bodylimits.h). */
inline constexpr double kInertialOrbitFrac = 1.25;

// -- atmosphere --------------------------------------------------------------

/* The derived atmosphere top in scale heights: a body that authors
   sea_level_density + scale_height but no height gets top() = this * H,
   i.e. ~e^-10 of sea-level density -- a sane hard top for a body with no
   authored one.
   Changes: where the air stops for drag AND where the science Flying*
   bands end, on every body without an authored top.
   Where: AtmosphereParams::top (terragen.h). */
inline constexpr double kAtmoScaleHeights = 10.0;

/* Aero skip floor [kg/m^3]: below this density the per-part silhouette
   pass is not worth computing. A PERFORMANCE floor, not a physical one
   (the physical top is the atmosphere height).
   Changes: CPU cost only.
   Where: callers of airDensity (vehicle.cpp, drag.h). */
inline constexpr double kRhoFloor = 1e-15;

/* Parachute deployment limiter [m/s^2]: a deployed canopy never
   decelerates the ship by more than this (Vehicle::applyAeroForce clamps
   the canopy's EFFECTIVE area to a_lim·m/q, so the pull saturates at
   a_lim). Physical -- real canopies are deployment-limited -- and
   numerical: it caps the weathervane torque at r·m·a_lim, keeping the
   restoring oscillation inside the aero substep's stability envelope at
   any dynamic pressure.
   Changes: descent g-load under canopy, and the q at which the full
   canopy area finally inflates. */
inline constexpr double kChuteAccelLimit = 50.0;

// -- science situations -------------------------------------------------------

/* FlyingLow/FlyingHigh split, as a fraction of the atmosphere top:
   FlyingLow is the bottom 20% of the air.
   Changes: which flying band a science run scores as (1.1 vs 1.2 weight),
   and the barometer's biome-identity range.
   Where: situationFor (science.h). */
inline constexpr double kFlyingLowFrac = 0.2;

/* LowOrbit/HighOrbit split margin [m] above the near-body shell edge.
   Changes: the altitude where high-orbit science (1.5 weight) starts.
   Equal to kSoiMargin on purpose: HighOrbit then begins exactly where a
   ship leaves the rotating frame.
   Where: orbitCutAlt (science.h). */
inline constexpr double kOrbitCutMargin = 10e3;

// -- grounded / rails ----------------------------------------------------------

/* isGrounded tolerances: the COM within kShipGroundBand [m] of the
   analytic terrain AND the speed under kShipGroundSpeed [m/s] in the
   rotating frame.
   Changes: what reads as Landed for science (a slow low hover reads as
   Landed), what can be Recovered from the surface, and rails
   eligibility. The speed term must stay above a walking kerbal's speed
   (kWalkSpeed, eva.cpp): a free kerbal is its own vehicle, and one that
   reads as neither grounded nor orbiting refuses rails warp for every
   ship.
   Where: Vehicle::isGrounded (vehicle.cpp). */
inline constexpr double kShipGroundBand = 100.0;   // m
inline constexpr double kShipGroundSpeed = 3.0;    // m/s

/* The "in the terrain band" periapsis depth [m]: a conic whose periapsis
   is at or below radius + this reads as flying, not orbiting.
   Changes: whether a prox-woken ship uses the ground or the flying radii.
   Where: Vehicle::inTerrainBand (game.cpp updateProximity).
   Rails do NOT use this: they refuse the air, and stop a conic at the
   ground. */
inline constexpr double kTerrainBand = 3000.0;

// -- HUD ------------------------------------------------------------------------

/* Below this ASL altitude [m] (and inside the rotating frame) the top bar
   shows SURFACE readouts -- terrain altitude + ground speed; above it, or
   in the inertial frame, ORBITAL readouts -- ASL + orbital speed.
   Where: the W_Hud window (gameui.cpp). */
inline constexpr double kSurfaceModeAlt = 30e3;

// -- orbital maps ---------------------------------------------------------------

/* Wheel-zoom bounds [m/pixel] for the orbital maps (flight + tracking), one
   shared pair so the two windows cannot drift apart. The upper bound decides
   whether a whole planetary system fits on screen at once, and the lower one
   how far into a body's neighbourhood a wheel can go.
   Changes: only how far the map scrolls -- no physics, no science.
   Where: the wheel-zoom clamps (gameui.cpp). */
inline constexpr float kMapMinScale = 100.0f;          // 10^2
inline constexpr float kMapMaxScale = 3.16227766e10f;  // 10^10.5

/* The map's initial and "Reset view" zoom, expressed as the RADIUS of what
   the view is fitted to, in pixels. That radius is the ship's apoapsis while
   its conic is elliptic and its current radius once the conic opens (ecc >= 1:
   escaping, or a degenerate radial state), never less than the focus body's
   own radius -- so a low orbit opens framed on the orbit and a ship on the pad
   opens framed on the planet. Kerbin on the pad lands within a few hundred
   metres of the old flat 6000 m/pixel the map was tuned at (its rotation puts
   the apoapsis a hair above the surface). Extents past kMapMaxScale or under
   kMapMinScale clamp to the wheel's bounds instead.
   Changes: what the map looks like on arrival at a body and after Reset view.
   Where: fitMapView (gameui.cpp), MapState::fit_body/fit_ship (game.h). */
inline constexpr float kMapFitRadiusPx = 100.0f;

/* Ascending / descending node marker colors, linear-ish sRGB [0,1] per
   channel. Kept as bare channels because the two places that draw them want
   different alphas and different color types: the orbit map fills opaque
   ImVec4 rings, the 3D orbital-plane overlay strokes translucent glm::vec4
   ticks. Same hue in both is the point -- one green/red pair means "ascending
   / descending" everywhere, and a reader of either can check the other.
   Changes: only what those markers look like.
   Where: kColNodeAn/kColNodeDn + drawMapNodes (gameui.cpp), drawOrbitPlane
   (render.cpp). */
inline constexpr float kNodeAnRgb[3] = {0.35f, 0.95f, 0.45f};
inline constexpr float kNodeDnRgb[3] = {0.95f, 0.30f, 0.25f};
