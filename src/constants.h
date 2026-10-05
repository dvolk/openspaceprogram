// constants.h -- the shared gameplay constants, one documented home.
// Each entry states its units, what changing it changes IN THE GAME, and
// roughly where the effect is implemented. Pure values (no includes), so
// the pure-math headers (drag.h, science.h, terragen.h, bodylimits.h) and
// the tests can all pull them in.

#pragma once

// -- frames / SOI ------------------------------------------------------------

/* Dead band [m] on both sides of every frame SOI: a ship leaves its frame
   above soi + margin and enters a child below child soi - margin, so a ship
   loitering exactly at a boundary cannot flip frames every tick.
   Changes: where surface <-> orbital physics flips (and with it the science
   HighOrbit boundary, which is one margin above the near-body shell).
   Where: Vehicle::soiTarget (vehicle.cpp), used by switchFrames and the
   sampling loop in railsTick; the loader's SOI nesting lift (system.cpp). */
inline constexpr double kSoiMargin = 10e3;

/* Rails-warp SoI sampling. A rail state is tested for an SoI crossing only
   at the END of a step (Vehicle::railsTick), so one step longer than a
   body's SoI chord carries the ship clean through it unnoticed: at 1e7x a
   50 Hz tick is 200 ks, so a 25 km/s interplanetary ship covers 5e9 m --
   more than a planet's whole Hill sphere, and 100x most moons'. The advance
   is therefore split until each sub-step covers at most this fraction of the
   smallest SoI the ship could be handed to (8 samples across its chord),
   capped at kRailsMaxSubSteps as the cost bound. The cap binds in every
   shipped system -- one tiny moon sets the scale for the whole frame -- so
   what this resolves is a planet's sphere (~19 samples at 1e7x) and NOT any
   moon: the pitch is ~1.6e8 m, and the frame tree is frozen for the tick, so
   the moons themselves move further than that.
   Where: Vehicle::railsTick (vehicle.cpp), railsSubSteps (orbit.h). */
inline constexpr double kRailsSoiFrac = 0.25;
inline constexpr int kRailsMaxSubSteps = 32;

/* Floor [m] for the near-body (rotating-frame) shell above sea level, for
   bodies whose atmosphere is shorter (or absent). 100 km is the historical
   flat shell every shipped body was authored with, so Kerbin and all
   airless bodies keep their exact old extents.
   Changes: the surface-frame extent, the science HighOrbit boundary, and
   the orbit-scenario spawn radii (fractions of the shell) on small or
   airless bodies.
   Where: shellEdge (bodylimits.h), applied by the loader (system.cpp). */
inline constexpr double kMinShell = 100e3;

/* The low-orbit science band's width above the atmosphere top, as a
   fraction of it: the shell edge is max(kMinShell, (1 + this) * top()).
   Changes: how far above the air LowOrbit extends before HighOrbit, i.e.
   the near-body shell size on tall-atmosphere bodies (Jool, the RSS
   giants). 0.2 mirrors kFlyingLowFrac: FlyingLow is the bottom 20% of the
   air, LowOrbit is a 20%-of-top band above it.
   Where: shellEdge (bodylimits.h). */
inline constexpr double kLowOrbitFrac = 0.2;

/* The `inertial-orbit` scenario's spawn radius, in near-body shells:
   1.25 x the shell puts it just outside the rotating frame (the scenario's
   whole point). The inertial-SOI lift keys off it (bodylimits.h
   inertialSoi) so the bed always resolves inside the body's own frame --
   without that, lifted tiny moons (Phobos, Gilly) would spawn the ship in
   the PARENT's frame at walking speed.
   Changes: the inertial-orbit spawn radius, and the inertial SOI floor on
   tiny bodies.
   Where: kScenarios (vehicle.cpp), inertialSoi + validateBodyLimits
   (bodylimits.h). */
inline constexpr double kInertialOrbitFrac = 1.25;

// -- atmosphere --------------------------------------------------------------

/* The derived atmosphere top in scale heights: a body that authors
   sea_level_density + scale_height but no height gets top() = this * H
   (~e^-10 of sea-level density). Lands within ~25% of the authored tops
   on shipped bodies, so an unauthored atmosphere still has a sane hard
   top.
   Changes: where the air stops for drag AND where the science Flying*
   bands end, on every body without an authored top (all RSS giants).
   Where: AtmosphereParams::top (terragen.h). */
inline constexpr double kAtmoScaleHeights = 10.0;

/* Aero skip floor [kg/m^3]: below this density the per-part silhouette
   pass is not worth computing. A PERFORMANCE floor, not a physical one
   (the physical top is the atmosphere height).
   Changes: only CPU cost (and the exact altitude where drag readouts
   round to zero).
   Where: callers of airDensity (vehicle.cpp applyAeroForce/airDensityAtCom,
   drag.h). */
inline constexpr double kRhoFloor = 1e-15;

// -- science situations -------------------------------------------------------

/* FlyingLow/FlyingHigh split, as a fraction of the atmosphere top:
   FlyingLow is the bottom 20% of the air (KSP-style).
   Changes: which flying band a science run scores as (1.1 vs 1.2 weight),
   and the barometer's biome-identity range.
   Where: situationFor (science.h). */
inline constexpr double kFlyingLowFrac = 0.2;

/* LowOrbit/HighOrbit split margin [m] above the near-body shell edge.
   Changes: the altitude where high-orbit science (1.5 weight) starts.
   Equal to kSoiMargin on purpose: HighOrbit then begins exactly where a
   ship leaves the rotating frame (orbitCutAlt, science.h).
   Where: orbitCutAlt (science.h), poseSituation (game.h). */
inline constexpr double kOrbitCutMargin = 10e3;

// -- grounded / rails ----------------------------------------------------------

/* isGrounded tolerances: the COM within kShipGroundBand [m] of the
   analytic terrain AND the speed under kShipGroundSpeed [m/s] in the
   rotating frame.
   Changes: what reads as Landed for science (a slow low hover reads as
   Landed), what can be Recovered from the surface, and rails
   eligibility. The speed term must stay above a walking kerbal's
   2.5 m/s (kWalkSpeed, eva.cpp): a free kerbal is its own vehicle, and
   one that reads as neither grounded nor orbiting refuses rails warp for
   the whole fleet.
   Where: Vehicle::isGrounded (vehicle.cpp). */
inline constexpr double kShipGroundBand = 100.0;   // m
inline constexpr double kShipGroundSpeed = 3.0;    // m/s

/* The "in the terrain band" periapsis depth [m]: a conic whose periapsis
   is at or below radius + this is not an orbit (isOrbiting() == false),
   so rails need isGrounded() instead.
   Changes: which ships rails-warp as orbiters vs refuse (airborne ships
   with a deep periapsis stay off rails).
   Where: Vehicle::inTerrainBand / isOrbiting (vehicle.cpp). */
inline constexpr double kTerrainBand = 3000.0;

// -- HUD ------------------------------------------------------------------------

/* Below this ASL altitude [m] (and inside the rotating frame) the top bar
   shows SURFACE readouts -- terrain altitude + ground speed; above it, or
   in the inertial frame, it shows ORBITAL readouts -- ASL + orbital speed.
   Changes: where the HUD altitude/speed readout flips meaning.
   Where: the W_Hud window (gameui.cpp). */
inline constexpr double kSurfaceModeAlt = 30e3;

// -- orbital maps ---------------------------------------------------------------

/* Wheel-zoom bounds [m/pixel] for the orbital maps (flight + tracking), one
   shared pair so the two windows cannot drift apart. The upper bound decides
   whether a whole planetary system fits on screen at once (10^10.5 spans
   ~2e13 m across a 600 px map).
   Changes: only how far the map scrolls in and out -- no physics, no science.
   Where: the wheel-zoom clamps in the flight map and tracking map windows
   (gameui.cpp). */
inline constexpr float kMapMinScale = 100.0f;          // 10^2
inline constexpr float kMapMaxScale = 3.16227766e10f;  // 10^10.5

/* The map's scale [m/pixel] at startup and after "Reset view". Sits well
   inside the zoom bounds: enough to see the focus body plus its immediate
   moons without any scrolling.
   Changes: what the map looks like before the player touches the wheel.
   Where: Game::map_scale (game.h), the flight map's "Reset view" (gameui.cpp). */
inline constexpr float kMapDefaultScale = 6000.0f;
