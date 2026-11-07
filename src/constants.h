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
   bodies whose atmosphere is shorter (or absent). It is the shell every
   airless shipped body gets, and was every body's until #686 gave Kerbin 85 km
   of air, whose 1.2x shell (102 km) is the first to exceed this floor.
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
   It is a FALLBACK, not the law: #682's band says a ceiling should sit where a
   circular orbit still feels 10..1000 Pa of q, and for 12 of the 14 shipped air
   bodies that is an authored `height`. Only two still ride this default -- Venus
   (160 km, where the band's 1.0 kPa lip would sit at 238 km) and Titan (200 km at
   800 Pa, on the lip) -- and utils/check_atmo_band.py refuses any other body that
   lands out of band, with Venus waived BY VALUE and the reason beside it.
   Changes: where the air stops for drag AND where the science Flying*
   bands end, on every body without an authored top.
   Where: AtmosphereParams::top (terragen.h). */
inline constexpr double kAtmoScaleHeights = 10.0;

/* The molar gas constant R [J/(mol K)]. On an EXPONENTIAL body one scale height IS
   an isothermal atmosphere, so the authored rho0 / scale_height plus the body's g
   already imply a temperature, a pressure and a sound speed through this and
   molar_mass [kg/mol] -- nothing else needs authoring (#552 A2). On a LAYER body
   (#552 A4) R is still the bridge, the other way round: the temperature is authored
   and R/M is the specific gas constant the column is integrated with
   (atmo.h:levelPressure), which is why p0 = rho0 (R/M) T0 needs no authored pressure.
   Changes: the derived readouts and the layered column; rho and top() do not contain
   it on an exponential body.
   Where: airTemperature / airPressure / airSoundSpeed (drag.h), resolveAtmoLevels (atmo.h). */
inline constexpr double kGasR = 8.314462618;

/* What "air" means when a body does not say: dry air, 28.964 g/mol and cp/cv 1.4.
   These are the hand-written spelling for a DragAtmosphere built outside a system
   file (test_drag's fixtures, drag.h's own member defaults). A body that DOES come
   from a file gets its gas from the mix it authors, and a body that authors no mix
   gets atmo.h:dryAirMix -- which resolves to 0.0289650 kg/mol and gamma 1.40146, not
   to these two, because the loader runs one arithmetic and not two (#630). The two
   spellings differ by 0.003%: dry air's argon and CO2 are what the extra accounts for.
   Changes: the temperature a body reports when its def says nothing; never rho.
   Where: AtmosphereParams, DragAtmosphere. */
inline constexpr double kAirMolarMass = 0.028964;  // [kg/mol]
inline constexpr double kAirGamma = 1.4;           // cp/cv, diatomic

/* The intake an air-breathing engine is RATED at [kg/m^3]: standard sea-level air.
   A part's authored jet_fan_thrust and fuel rate are one machine's numbers at one
   intake density, so that is what the jet gate divides by (#630). It used to divide
   by the BODY's own sea-level density, which rates the engine against whatever planet
   it happens to be standing on: a jet at Mars's datum ran its full authored fan
   thrust on 0.016 kg/m3 of CO2 because that is 1.0 of Mars's own sea level.
   Every air-breathing body in the shipped catalogue already authors this density
   (Kerbin, Earth, Laythe, Shay: 1.225), so on the worlds with living jets the number
   is the one already in the file -- it is not a new constant, it is the one the
   bodies were given.
   Changes: jet thrust and fuel burn on any body whose sea-level density is NOT
   1.225 (Eve 1.7, Jool 2.0, Titan 5.3: capped either way; Duna 0.015, Mars 0.016:
   both are CO2, so the oxidizer half of the gate already ends them -- #700 made
   Duna's column 8x thinner at the ground and it is still a dead engine there).
   Where: drag.h:jetAirGate. */
inline constexpr double kAirSeaLevelDensity = 1.225;  // [kg/m^3]

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

/* Angle-of-attack display floor [m/s]: below this air-relative speed the flow
   has no usable direction, so AoA is not a flight quantity. A ship parked in
   air still carries ~1e-4 m/s of integrator jitter against the co-rotating
   atmosphere, which reads as a full +/-90 deg AoA on a nose-up hull.
   Display only -- the aero laws still see every frame, however slow.
   Where: Vehicle::applyAeroForce (lastDragAlphaValid), SURFACE window,
   [surfinfo], --drag-log. */
inline constexpr double kAoAFlowFloor = 0.1;

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
