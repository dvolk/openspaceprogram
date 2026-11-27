// constants.h -- the shared gameplay constants, one documented home.
// Each entry states its units, what changing it changes IN THE GAME, and
// roughly where the effect is implemented. Pure values (no includes), so
// the pure-math headers and the tests can all pull them in. A value that
// changes nothing in the game -- a "not authored" sentinel, a parse bound --
// belongs beside the law it guards instead (see kNoCoord in equirect.h).

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
   sea_level_pressure + scale_height but no height gets top() = this * H,
   i.e. ~e^-10 of sea-level density -- a sane hard top for a body with no
   authored one.
   It is a FALLBACK, not the law: #682's band says a ceiling should sit where a
   circular orbit still feels 10..1000 Pa of q. In the shipped corpus only Titan
   rides this default (its 20 km scale height puts the ceiling at 200 km, where a
   circular orbit still feels 800 Pa -- on the band's lip); every other air body
   authors its own `height`, and utils/check_atmo_band.py is what weighs them and
   refuses one that lands out of band, waiving BY VALUE only where a published
   ceiling is deliberately below it. Pluto's 20 km (#700) is the first authored top
   taken from a published PRESSURE rather than a published ceiling; it is not the
   first under kMinShell -- the thin CO2 worlds (Mars, Duna) already inherit that
   floor, which is what makes a THIN air
   cheap: no orbit, spawn band or atlas disk moves until a top passes
   kMinShell / (1 + kLowOrbitFrac) = 83.3 km.
   Changes: where the air stops for drag AND where the science Flying*
   bands end, on every body without an authored top.
   Where: AtmosphereParams::top (terragen.h). */
inline constexpr double kAtmoScaleHeights = 10.0;

/* The molar gas constant R [J/(mol K)]. On an EXPONENTIAL body one scale height IS
   an isothermal atmosphere, so the authored sea_level_pressure / scale_height plus the
   body's g already imply a temperature and a density through this and molar_mass
   [kg/mol] -- nothing else needs authoring (#552 A2, and #739 chose pressure as the
   authored one of the pair: on this branch the gas cancels, since the datum T that H
   implies is g H M / R, so rho0 = p0 / (g H) needs no R at all). On a LAYER body
   (#552 A4) R is still the bridge, the other way round: level 0's temperature and the
   authored p0 are both stated, and R/M is the specific gas constant the column is
   integrated with (atmo.h:levelPressure) and the datum density divided by
   (#739: rho0 = p0 M / (R T0)).
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
   1.225. Eve 3.55, Titan 5.3, Venus 65: the fan clamps at its rating and the denser air
   only buys ram. Jool 0.188 and Venus and Titan's traces: no free oxygen at all, so
   there is nothing to meter and the engine is dead wherever the air is thick. Duna
   0.015, Mars 0.016: CO2 with a 0.096 % O2 trace, which jetBurnCap ends -- #700 made
   Duna's column 8x thinner at the ground and it is still a dead engine there. (An
   earlier draft of this comment quoted Eve at 1.7 and Jool at 2.0 kg/m3; both are wrong
   against the shipped system files -- #1235.)
   Where: drag.h:jetIntakeGate. */
inline constexpr double kAirSeaLevelDensity = 1.225;  // [kg/m^3]

/* Jetfuel's heat of combustion [J/kg], kerosene-like. The ONLY number in the game that
   says where an air-breathing engine's momentum comes from: drag.h:jetEngine spends
   Q * burn on moving air, and BOTH of the engine's answers -- the fan's momentum and the
   hot stream's -- are capped by it (#1206). Without it the law is a pump: #1206's demo
   part (fan 3000, 3.0 kg/s, v_e 3200, 0.35 m2, 43.75 kg) reported 64.7 kN at 2400 m/s in
   20 km air while handing 178 MW to the flow on a 10 MW fire, and 956 kN / 2.0 GW at sea
   level on 129 MW.
   It is spent at full value -- no thermal loss, no nozzle loss, no turbine loss -- so the
   bound it sets is generous. It exists to make an unaffordable engine impossible, not to
   model a burner; the inlet's thermal choke is still open (#1207).
   What it pays for now, all in one place (drag.h:jetEngine):
     - the FAN. An ideal actuator disc making T through A of air runs it at
       sqrt(T / 2 rho A) and wants T times that in power, so a fan now buys its thrust out
       of the same Q * burn. The shipped 2 m jet's 90 kN disc costs 29.2 MW at Kerbin sea
       level (0.678 kg/s) and the pump covers that; on Eve's datum the same fan wants
       17.1 MW while its trace oxygen affords 4.0, so the fan runs at 19.7 kN instead of
       90 (#1233, which this closed).
     - the hot stream: min(the fuel the captured air can burn, what the exhaust promise
       actually costs).
     - the fuel's own momentum, implicitly, while v_e <= sqrt(2 Q) = 9274 m/s --
       shipdef.cpp refuses a catalogue jet above that.
   What it still does NOT bill: the inlet's entrance drag (#1207), and the difficulty
   knobs -- --exhaust-scale (0.5-5.0) multiplies a jet's thrust OUTSIDE this cap, by
   design, exactly as it does a rocket's.
   Changes vs the previous law: a DRY-air jet barely moves -- the shipped 2 m part makes
   122.5 kN at 275 m/s where it made 123.1 kN, because its draw drops from the plate's
   1.2 kg/s to the 0.989 kg/s its momentum actually costs. A TRACE-air jet moves a lot:
   on Eve's datum (3.55 kg/m3, share 0.000959) the same part goes 3318 N -> 34.4 kN,
   because the 0.094 kg/s its oxygen buys now pays for the fan as well as the stream.
   tests/test_jet.cpp asserts the whole-engine identity, Q * burn = disc power + the
   flow's kinetic-energy gain, so the two halves cannot drift apart again.
   Where: drag.h:jetEngine, drag.h:jetRamExitSpeed, shipdef.cpp. */
inline constexpr double kJetFuelHeatPerKg = 43.0e6;

/* Kilograms of OXIDIZER needed to burn one kilogram of jetfuel (dimensionless).
   Kerosene burns C12H26 + 18.5 O2 -> 12 CO2 + 13 H2O: 592/170 = 3.48 by mass, and
   RP-1 literature quotes 3.4-3.5. This is STOICHIOMETRY, not the engine's operating
   mixture: a real combustor runs 25-100 kg of AIR per kg of fuel for thermal and
   turbine reasons, so this number is a floor on how much fuel the air can accept,
   never a target. What it buys is the ability to say that a CO2 sky is not an
   on/off switch: Eve's 0.096 % O2 trace hands a burner 342 kg/s x 0.000959 / 3.48 =
   0.094 kg/s of fuel at her datum, where dry air hands the same intake 27 kg/s.
   Changes: the oxidizer half of drag.h's jet gate -- a factor in [0,1] -- becomes
   this ceiling in kg/s. A jet on a trace world goes from "0.4 % of rating" to
   "what the oxygen in the air it actually swallows can burn", which on Eve is
   ~6.6x what the old factor allowed.
   Where: drag.h:jetBurnCap. */
inline constexpr double kStoichO2PerKgJetfuel = 3.48;

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

/* Sideslip degeneracy cone [deg]: beta is the yaw of the flow INSIDE the hull's
   nose-right plane, so it is only a fact once the flow projects onto that plane.
   When the flow lies along the hull's UP axis the projection collapses to a point
   and both arguments of atan2(right, nose) vanish together, leaving whatever the
   integrator's jitter points at: a hull parked nose-up in wind read -92.9, -160.0
   and +85.7 deg in three successive samples while its AoA sat rock solid at -90.0
   (#787). A flow within this many degrees of up has no yaw to report.
   An ANGLE, not a speed like kAoAFlowFloor above -- and that speed floor is why
   this was never caught: the parked hull's air-relative speed is several m/s of
   wind, which clears 0.1 m/s easily, while a 1500 m/s flow straight up a hull's
   mast is just as directionless in yaw.
   Display and logging only: no force reads beta, and past +-75 deg the silhouette
   chart refuses the direction and answers exactly (#463), so a noisy beta there
   costs nothing but a misleading number.
   Where: drag.h::aeroFrame (AeroFrame::betaValid), the SURFACE window's Beta row and
   [surfinfo] beta=, --drag-log's Beta=, and the flight log's beta= column, which
   carries the dash as a NaN through recorder.h (fed at tick.cpp). */
inline constexpr double kBetaIllCondDeg = 1.0;

/* sin(kBetaIllCondDeg), folded once. aeroFrame runs per hull per substep and the sin
   used to be computed there on every call, the same scope that stopped folding
   180 / M_PI inline (vehicle.cpp). C++20 has no constexpr sin, so the number is written
   out and tests/test_drag.cpp pins it against std::sin of the degree constant above:
   changing one without the other reddens a test. */
inline constexpr double kBetaIllCondSin = 0.01745240643728351;

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

/* [C] The radius of the ring meshes that the reference circles and the orbital-plane overlay
   share: main.cpp builds them, render.cpp draws them CAMERA-CENTRED (the view's translation is
   dropped), so that radius is pure depth -- scaling the mesh about the eye moves no pixel on
   screen and only decides who wins the depth test (#1180). The build site and the scaling site
   must agree, which is why it is a constant and not main.cpp's local 1000.
   Where: the ring meshes (main.cpp), the overlay block and drawOrbitPlane (render.cpp). */
inline constexpr double kRingMeshRadiusM = 1000.0;

/* [C] The offset between kelvin and degrees Celsius, spelled ONCE (#707). Every file, law and
   log in this project is kelvin; this exists only for the readouts a player reads as weather --
   a ground you could stand on is more legible as -58 C than as 215 K. Never feed it to a law:
   radiation and the air's thermodynamics are kelvin-only, and a T - 273.15 in an equation is a
   bug, not a unit.
   Where: the surface map's cursor row and the atlas' K-with-C rows (gameui.cpp). */
inline constexpr double kCelsiusOffset = 273.15;
inline double celsiusOf(double kelvin) { return kelvin - kCelsiusOffset; }
