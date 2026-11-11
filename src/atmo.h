// atmo.h -- an atmosphere's own three facts: what its air is MADE of (gasSpecies,
// resolveMix), how it STACKS with altitude (AtmoLayer, levelPressure) and how it MOVES
// (WindLevel, windAt, #688). All of it is pure data plus the arithmetic that turns
// authored numbers into the ones the law reads, so no caller re-derives them. Read by
// the authored half (terragen.h:AtmosphereParams, parsed from a system file), the
// loader that resolves it (system.cpp), the aero law that samples its profile
// (drag.h:airDensity/airPressure/airTemperature/airSoundSpeed/jetAirGate) and the ONE
// place its motion enters the flow (Vehicle::airRelativeVel).
//
// It lives in its own header because all three readers (terragen.h, system.cpp, drag.h)
// need the exact same structs, and none of them may include another as a peer:
// terragen.h is the terrain generator, drag.h is the aero law. constants.h is the other
// leaf they share, and it holds numbers rather than types. What is HERE is level-LOCAL
// -- one AtmoLayer plus the body's g and its gas. Deciding WHICH level an altitude
// falls in belongs to drag.h, because that is where a body's table, its top and its
// flat floor live.

#pragma once

#include <algorithm>
#include <cassert>   // assert() is used below; the header must be includable first
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <glm/glm.hpp>  // windAt returns a horizontal pair as one dvec2

#include "constants.h"  // kGasR, for the hydrostatic integration below
#include "equirect.h"   // wrapLonDeg: the ONE lon-wrap law, which a wind band is keyed by

/* One level of an altitude-temperature profile: from `base_alt` up to the next
   level's base_alt (or to the body's air top for the last one),
       T(alt) = base_T + lapse * (alt - base_alt)          [K], lapse in K/m
   A level is NOT a slab of constant temperature: a lapse of -0.0065 means 6.5 K
   per kilometre, which is what makes the density fall faster than the pressure
   and is the whole reason one authored scale_height cannot also state a
   temperature (drag.h's "THE THREE ARE LOCKED" block). lapse 0 = an isothermal
   level, which is what a real stratosphere does and what the whole of #552 A2's
   single exponential already was.

   `base_p` is the pressure at `base_alt` [Pa], filled in when the system file loads
   by integrating the level above hydrostatically (resolveAtmoLevels, called by
   system.cpp). Level 0's is the ONE exception: it IS the file's authored
   `sea_level_pressure` (#739), and the datum DENSITY that every law in drag.h reads
   is the consequence, resolved by the loader as rho0 = p0 M / (R base_T(0)). Before
   #739 the density was authored and this number derived, which made sea-level
   pressure a consequence of the datum temperature -- so warming the air created
   weight, the wrong sign for anything diurnal or seasonal (#724).
   Authoring a base_p for a level ABOVE the first would add a number the law already
   implies and let a def quietly break hydrostatic equilibrium at a level boundary;
   the load pass is where the pair is made consistent.

   Load invariants (system.cpp refuses a table that breaks them, because a bad one
   is a divide by zero or a negative temperature several steps later, not a load
   message the player can read):
     - the first level's base_alt is 0, i.e. the sea-level DATUM -- the same datum
       airDensity measures from (terrain.h:seaLevelRadius), and the level below the
       datum stays FLAT at sea_level_density exactly as #245 made it;
     - base_alt strictly increasing, and the last level based BELOW the body's air
       top, so no level sits in the vacuum `height` already declared;
     - base_T > 0 and T still > 0 at the level's own ceiling, so a lapse cannot
       walk the temperature through zero inside the air;
     - base_T continuous: the temperature a level starts at is the one the level
       above leaves off at, within kAtmoSeamTol;
     - lapse above -g / Rs, the point past which the air gets DENSER as you climb;
     - no warming level whose lapse coincides with the body's own gravity falloff
       (kAtmoPoleTol), which is where the column integral divides by zero.
   The column is weighed in the body's 1/r^2 field, not a uniform one: `g` is the
   field at the datum and the hull is pulled by less as it climbs (#683). That
   needs the body's sea-level radius, which is why resolveAtmoLevels and
   levelPressure both take it and why DragAtmosphere carries it. */
struct AtmoLayer {
    /* [m above the sea-level datum], GEOMETRIC -- like every other altitude in a system
       file (the vacuum cut, the science bands, the shell). Published standards state their
       rows at GEOPOTENTIAL heights, so a table copied from one has to be converted before it
       is written: z = R Z / (R - Z), the inverse of geopotentialDepth below. That happens in
       utils/make_solar_system.py, which marks such a source heights: "geopotential" and emits
       geometric metres; tests/test_scenarios.cpp pins the numbers it produced (#690). Earth's
       "51 km" row lands at 51,411.6 m here -- the same row pasted onto a 600 km world would sit
       at 55.7 km, so this is not a rounding question for a KSP-sized body.
       A `lapse` is K per unit of that SAME height coordinate, so it converts with its row --
       as a CHORD, the published gradient times dZ/dz ACROSS THE SPAN the level lapses
       through: (Z_next - Z) / (z_next - z), with the last level's span running to the body's
       air top. That lands every base_T back on the published temperature. The tempting
       shortcuts are the published gradient beside converted bases -- Earth's column then
       arrives 1.09 K off its own 71.8 km row -- and the tangent (R / (R + base_alt))^2 read
       AT the row, which drifts 0.18 K on Earth and 1.34 K over a 600 km world's troposphere.
       The seam rule below refuses the first outright and the second on a small body. */
    double base_alt = 0.0;
    double base_T = 0.0;     // [K] at base_alt
    double lapse = 0.0;      // [K/m]; 0 = isothermal level
    double base_p = 0.0;     // [Pa] at base_alt; RESOLVED at load, not authored
};

/* T [K] at alt inside level L. Zero means unusable (a NaN altitude, or a lapse the
   loader did not already refuse), and every caller fails CLOSED on it. */
inline double levelTemperature(const AtmoLayer &L, double alt) {
    const double T = L.base_T + L.lapse * (alt - L.base_alt);
    return (T > 0.0) ? T : 0.0;
}

/* THE AIR'S OWN MOTION (#688). A wind is a VELOCITY, never a force term -- both
   reference engines surveyed for #688 author it that way (Orbiter's `windVelocity`),
   and the reason is mechanical: the flow at a surface is `v_ship - v_air`, so once the
   air has a velocity the aero law, Mach, the jet gate and the air-relative prograde
   marker all read it for free, while a fake force would give a hull that flies as if
   the air were still but reads an airspeed that lies.

   One row per altitude, like AtmoLayer, because the honest published picture is a
   PROFILE, not a number: Venus drifts ~1 m/s at its surface and ~100 m/s at its cloud
   deck, and a body whose only wind is at cloud level is exactly what a platform riding
   that deck needs (#712). `alt` is GEOMETRIC metres above the sea-level datum -- the
   same coordinate AtmoLayer bases on and airDensity measures from -- and east/north are
   [m/s] in the local horizontal basis (equirect.h:equirectEast/equirectNorth), which is
   the ground's own frame. So these numbers are RELATIVE TO THE GROUND: the co-rotation
   is not in them and must not be, or every spinning body would author its own spin into
   every row and the datum would change with the clock.

   windAt() asks a place as well as an altitude, and the answer is ONE BAND's profile:
   among the rows that cover the place, the NARROWEST band that covers it answers for it
   at every altitude, interpolating LINEARLY between the two of its rows that span alt
   and holding the nearest end outside them -- below the first row (a body whose first
   row is a jet aloft has no reason to claim the surface is calm) and above the last (the
   air thins out, it does not calm down). Specificity rather than altitude alone is what
   makes "a whole-sphere column, overridden where the sheets differ" mean what it says:
   picking the nearest row below and above from ALL covering rows lets a wide column
   punch back up through a belt's own span, which turns the profile into a sawtooth.
   A loader refuses an unsorted row list WITHIN a band, a row missing or misspelling a
   key, a non-finite number, a table past 64 rows (windAt scans linearly on the aero
   path), two rows at one altitude answering for overlapping ground, a set of bands that
   leaves any place on the body without an answer, or a COMPONENT past kWindMaxSpeed --
   per axis, so a row may total up to kWindMaxSpeed*sqrt2. That bound is a typo net, not
   physics: see the number's own comment below. */
struct WindLevel {
    double alt = 0.0;    // [m above the sea-level datum], GEOMETRIC
    double east = 0.0;   // [m/s] toward increasing lon, relative to the ground
    double north = 0.0;  // [m/s] toward increasing lat, relative to the ground
    /* The RISING half (#767 again): [m/s] along the local up, so positive is the air
       climbing past the ground. The general circulation is a closed cell -- air rises
       where the Hadley cells converge (Earth's ITCZ, Venus's subsolar point, the gas
       giants' equatorial upwellings) and sinks in the subtropics and at the poles --
       and that vertical leg is what a wind table could not say until now. It is also
       tiny: Earth's synoptic updraft is ~1 cm/s and Juno/Cassini read Jupiter's
       secondary circulation at mm/s to a cm/s, four orders under the jets it closes.
       Nothing in utils/rss/html publishes one, so no body authors it yet; the axis is
       here so that when a source arrives it lands in a row, not in a new law. It is
       not decorative: a downdraft moves a hull's apparent wind, and with it alpha,
       lift and the descent rate. */
    double up = 0.0;
    /* WHERE the row answers (#767), in DEGREES: lat in [lat0, lat1), lon in [lon0, lon1).
       The defaults are the whole sphere, which is what a row with no "lat"/"lon" key means
       and what every table authored before #767 says. Jupiter and Saturn publish their
       winds PER LATITUDE ("Up to 150 m/s (<30 deg lat)", "Up to 40 m/s (>30 deg lat)"),
       and holding that globally would put a 150 m/s flow over their poles, so the axis is
       authored rather than inferred. Both bands are HALF-OPEN UPWARD because that is what
       makes the published "<30 deg" and ">30 deg" pair tile the planet with no overlap and
       no gap; +90 is included so the row that reaches the pole owns it. Two rows answering
       for the same place at the same altitude are a question asked twice, so the loader
       refuses that rather than picking a winner -- which is what keeps the lookup below
       unambiguous at every altitude.

       lon is stored as a start plus an EASTWARD width, so a band may cross the
       antimeridian: [350, 10] is 20 deg wide, and the whole sphere is width 360.
       wrapLonDeg() puts lon0 in [0, 360) at load. */
    double lat0 = -90.0;
    double lat1 = 90.0;
    double lon0 = 0.0;    // [deg), wrapped at load
    double lon1 = 360.0;  // lon0 + eastward width, so in (lon0, lon0 + 360]
};

inline double windRowSpanDeg(const WindLevel &W) {
    return W.lon1 - W.lon0;   // wrapped at load into (0, 360]
}

/* The row budget: windAt() scans the table linearly once per air-relative read per hull,
   so an unbounded table is a loop on the aero path. Named because two places spend it --
   the loader refuses it, and the generator (#775) budgets its belts against it -- and two
   literal 64s would drift apart. */
inline constexpr size_t kWindMaxRows = 64;

/* How much of the body a row answers for [deg^2] -- the specificity windAt() selects
   by. It is only ever compared, never read as a number, so the lat/lon product is fine:
   a belt is narrower than the column it overrides on both axes at once. */
inline double windBandAreaDeg2(const WindLevel &W) {
    return (W.lat1 - W.lat0) * windRowSpanDeg(W);
}

/* Do these two rows answer for exactly the same patch of the body (#767)? Spelled once
   because the loader's "rise within your band" rule, its "no two rows answer one place
   at one alt" rule and surfmap.h's per-band ramp walk all need the same question, and
   three spellings would drift. */
inline bool windSameBand(const WindLevel &A, const WindLevel &B) {
    return A.lat0 == B.lat0 && A.lat1 == B.lat1 && A.lon0 == B.lon0 && A.lon1 == B.lon1;
}

/* Does this row answer for the place (lat, lon), given in DEGREES? Both axes are
   HALF-OPEN upward -- [lat0, lat1) and [lon0, lon1) -- because that is what makes
   Jupiter's published "<30 deg" and ">30 deg" bands tile the sphere with no overlap and
   no gap; only the +90 edge is included, so the row that reaches the pole owns it.
   The lon test rotates the point east into the band's own start rather than splitting
   the band, so a band across the antimeridian is one row, not two.

   lon is wrapped HERE rather than at each caller, because the loader's promise that
   every place has an answer is made over lon in [0, 360) and two callers land just
   outside it: the surface map's right-hand edge is u == 1, and equirect.h's inverse
   unwraps a direction a hair west of the prime meridian to a lon that rounds onto a
   full turn. A 360 that no half-open band covers is not a place with no wind -- it is
   the same meridian as 0 -- so it must not reach the assert in windAt(). */
inline bool windRowCovers(const WindLevel &W, double lat_deg, double lon_deg) {
    if(lon_deg < 0.0 || lon_deg >= 360.0) { lon_deg = wrapLonDeg(lon_deg); }
    if(lat_deg < W.lat0) { return false; }
    if(lat_deg >= W.lat1 && !(W.lat1 == 90.0 && lat_deg == 90.0)) { return false; }
    if(windRowSpanDeg(W) >= 360.0) { return true; }
    const double L = lon_deg < W.lon0 ? lon_deg + 360.0 : lon_deg;
    return L >= W.lon0 && L < W.lon1;
}

/* Loosest bound that still catches a mistyped row. Every published sheet in
   utils/rss/html is weighed against the authored rows by check_fact_wind
   (utils/make_solar_system.py), so this is only the net under that, and its number is
   the corpus's own ceiling: Neptune's sheet says 0-580 m/s, which is what the fastest
   published wind actually is (#766 -- the bound used to be 500, justified by a comment
   claiming the corpus tops out at "a few tens of m/s", which was simply not true).
   A row at the cap is not weather: 1000 m/s is Mach ~2.9 in Neptune's 1-bar column
   (rho ~0.05 kg/m3, so q ~25 kPa) and Mach ~2.9 at Earth's sea level, where the same
   number is q ~610 kPa on a hull. Nothing downstream caps what that impulse does --
   that hole is #716 -- so raising this bound is about admitting published data, not
   about inviting anyone to author 1000 m/s on a body with air thick enough to care. */
inline constexpr double kWindMaxSpeed = 1000.0;   // m/s, per horizontal component

/* The same kind of net for the RISING component, and it has to be a different number.
   The horizontal bound sits above the corpus's published ceiling (Neptune's 580 m/s,
   #766), so it cannot catch a vertical decimal point: nothing publishes an updraft
   over a cm/s, which means a row authored 0.05 and typed 50 is a 1000x error that the
   horizontal net would wave through -- and 50 m/s of rising air is not circulation, it
   is a geyser that also rewrites every descent profile on the body. 100 m/s keeps four
   orders of headroom over anything a source has ever measured while still refusing the
   typo. Raise it with a published number, not with a row. */
inline constexpr double kWindMaxRise = 100.0;     // m/s, up/down

/* ── THE GENERATOR (#775) ────────────────────────────────────────────────────────
   A column of anchors plus how fast the body spins says what KIND of latitudinal
   structure its air can have before anyone draws a band. Two numbers do the work:

       Ro = U / (2 Omega R)                          is rotation relevant at all?
       L  = sqrt(U / beta), beta = 2 Omega cos(lat)/R    the Rhines scale: rotation
                                                        allows jets no narrower than L.

   Measured against the bodies where the answer is known (tmp/wind_scaling.py, table on
   #775; utils/make_solar_system.py prints the same numbers per body in --check, using
   each body's fastest AUTHORED row as U): Venus Ro 27.6 -> no jets, and Venus has none;
   Jupiter Ro 0.006 -> ~19 asked, spacecraft count 10-22; Earth Ro 0.043 -> 7 asked
   against 2-3 real cells. Mars and the ice giants overpredict -- their jets are
   eddy-driven and weak rather than Rhines-limited -- which is exactly why this ships on
   INVENTED bodies first and not on the ones whose fact sheets the dossier quotes.

   What comes out is ordinary WindLevel rows, so windAt, the surface sweep, the Surface
   window and [surfinfo] cannot tell the difference and none of them grew a branch. The
   generator is a ROW FACTORY, not a second law of wind: system.cpp re-runs the coverage
   law and the 64-row budget on the OUTPUT, and the tiling invariants are asserted here,
   so a generator bug refuses instead of painting nonsense. Provenance rides beside the
   table (terragen.h) so the dossier can say "generated" rather than passing a guess off
   as a published figure. */

/* Rossby number: the flow's own inertia over the planet's rotation. At or above 1 the
   spin never gets a vote. A non-spinning body is infinity, not a divide by zero. */
inline double windRossby(double U, double rot_ang_speed, double radius) {
    if(rot_ang_speed <= 0.0 || radius <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return U / (2.0 * rot_ang_speed * radius);
}

/* The Rhines scale at one latitude, in DEGREES of latitude along a meridian. No spin or
   no wind gives the whole sphere, which is the same answer as "no jets". */
inline double windRhinesDeg(double U, double rot_ang_speed, double radius, double lat_deg) {
    if(U <= 0.0 || rot_ang_speed <= 0.0 || radius <= 0.0) { return 180.0; }
    const double beta = 2.0 * rot_ang_speed
                      * std::cos(lat_deg * std::numbers::pi / 180.0) / radius;
    return std::sqrt(U / beta) / radius * (180.0 / std::numbers::pi);
}

/* What the generator decided, for the dossier and --check to echo rather than re-derive. */
struct WindGenInfo {
    int bands = 0;         // latitude bands emitted (1 == the column, unchanged)
    double ro = 0.0;       // Rossby number at the column's fastest row
    double jet_deg = 0.0;  // Rhines scale at 30 deg, in degrees of latitude
    int asked = 0;         // belts per hemisphere the Rhines scale asked for, before clamps
    bool clamped = false;  // the 64-row budget cut that count
};

/* A rotator this slow gets no latitudinal structure at all: rotation never constrains
   the flow, so the honest picture IS the authored column. Venus sits at Ro 27.6, and
   Earth at 0.043, so the threshold is not close to deciding anything real -- it is there
   to stop a slow spinner being banded by a Rhines count that means nothing without it. */
inline constexpr double kWindGenRoBanded = 3.0;

/* Poleward amplitude decay per band, and the floor under it. Away from the strongest jet
   the real ones fall off roughly this way; the floor is what stops a Rhines count of 19
   turning into 19 rows of air nobody could feel. Both are tuning, not laws -- the table
   on #775 is what they are tuned against. */
inline constexpr double kWindGenDecay = 0.72;
inline constexpr double kWindGenFloor = 0.10;

/* How far a band edge may wander from an even spacing, as a fraction of the spacing.
   Belts in every published picture are unevenly spaced; an exact grid reads as fake. */
inline constexpr double kWindGenJitter = 0.15;

/* splitmix64. Deterministic, exact in integer arithmetic, no RNG state and no clock, so
   one body seed generates one belt layout on every platform and in every test run. */
inline uint64_t windGenHash(uint64_t seed, int k) {
    uint64_t z = seed + 0x9E3779B97F4A7C15ull + 0x517C1CE1ull * (uint64_t)k;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

/* The generator's seed for ONE body: FNV-1a over its name, mixed with the body's own
   seed. The name matters because two bodies can share a seed and a spin -- in a system
   built from a template that is the normal case -- and two worlds with identical belts
   would read as one world copied. Out of the loader so a test can pin that property
   without building a whole system around it. */
inline uint64_t windGenSeed(const std::string &name, double body_seed) {
    uint64_t h = 1469598103934665603ull;
    for(const char c : name) { h ^= (uint64_t)(unsigned char)c; h *= 1099511628211ull; }
    return windGenHash(h ^ (uint64_t)std::llround(body_seed), 0);
}

/* Expand a column of whole-sphere anchors into latitude bands.

   U is the FASTEST horizontal speed in the column, not its average: the scaling is about
   the jet the author is describing, and averaging it against a surface row of 1 m/s would
   hide it. A column with no horizontal wind has no jet for rotation to limit, so it comes
   back exactly as authored -- banded calm air is structure out of nothing.

   The bands mirror about the equator, and "mirror" is per COMPONENT, which is the part
   that is easy to get wrong:
     east  -- symmetric. A zonal wind is a scalar under an equatorial mirror, so the ±|j|
              bands share a sign, and each band further out alternates against its neighbour.
     north -- ANTI-symmetric. Air moving away from the equator does so in BOTH hemispheres;
              give the ±|j| bands the same `north` and the field drags mass across the
              equator, buying a source at one band edge and a sink at the other.
     up    -- symmetric in the hemispheres, alternating with the belts: rising in one,
              sinking in the next. Air rising at every latitude has nowhere to come from,
              and the hand-authored cell in res/systems/wind_playground.json ("Hadley") is
              the same pattern -- +0.02 over the equator, -0.015 in the flanking belts.
   The one band straddling lat 0 keeps the anchors' own signs: inside a band that wraps the
   equator there is no antisymmetric answer to give. Amplitudes decay poleward by
   kWindGenDecay, so every emitted band sits at or above kWindGenFloor.

   system.cpp re-checks the OUTPUT against the coverage law and the 64-row budget, so this
   must satisfy the loader's laws itself: bands tile [-90, 90] with no gap or overlap, one
   row per anchor per band, and the anchors' own alt order inside every band. */
inline std::vector<WindLevel> windGenerate(const std::vector<WindLevel> &anchors,
                                           double rot_ang_speed, double radius,
                                           uint64_t seed, WindGenInfo &info) {
    assert(!anchors.empty());
    for(const WindLevel &A : anchors) {
        assert(A.lat0 == -90.0 && A.lat1 == 90.0);   // anchors are columns, not bands
    }
    double U = 0.0;
    for(const WindLevel &A : anchors) { U = std::max(U, std::hypot(A.east, A.north)); }
    info.ro = windRossby(U, rot_ang_speed, radius);
    info.jet_deg = windRhinesDeg(U, rot_ang_speed, radius, 30.0);
    info.asked = 0;
    info.clamped = false;
    if(U <= 0.0) {
        info.bands = 1;                              // calm air: nothing to band
        return anchors;
    }

    /* How many belts per hemisphere the Rhines scale asks for -- FLOOR, because half a
       Rhines scale across a whole hemisphere is zero belts. Then the row cap answers back:
       system.cpp refuses a table over 64 rows because windAt scans linearly on the aero
       path, and this emits one row per anchor per band. */
    int nb = (int)std::floor(90.0 / info.jet_deg);
    info.asked = nb;
    const int room = (int)(kWindMaxRows / anchors.size());
    if(2 * nb + 1 > room) { nb = (room - 1) / 2; info.clamped = true; }
    if(info.ro >= kWindGenRoBanded) { nb = 0; }

    /* And the decay floor answers back too: past jmax the amplitude is under it, so those
       belts are dropped and the outermost KEPT band owns everything to the pole. Dropping
       a band and leaving its latitudes out would break the coverage the loader insists on
       -- a faint belt is a wide belt, never a hole. */
    int jmax = 0;
    for(double a = kWindGenDecay; a >= kWindGenFloor; a *= kWindGenDecay) { ++jmax; }
    if(nb > jmax) { nb = jmax; }

    if(nb <= 0) {
        info.bands = 1;
        return anchors;
    }

    /* Even division of the hemisphere, then jitter. Edges sit at (k + 0.5) * w from the
       equator so that ONE band straddles lat 0 rather than an edge landing on it. */
    const double w = 90.0 / (double)nb;
    std::vector<double> edge((size_t)nb);
    for(int k = 0; k < nb; k++) {
        const double u = (double)(windGenHash(seed, k) >> 11) / 9007199254740992.0;
        edge[(size_t)k] = (k + 0.5) * w + (u * 2.0 - 1.0) * kWindGenJitter * w;
    }
    for(int k = 0; k < nb; k++) {
        if(edge[(size_t)k] > 89.0) { edge[(size_t)k] = 89.0; }   // keep the polar cap real
        if(edge[(size_t)k] < 1.0) { edge[(size_t)k] = 1.0; }
    }
    for(int k = 1; k < nb; k++) {
        if(edge[(size_t)k] <= edge[(size_t)k - 1]) {
            edge[(size_t)k] = edge[(size_t)k - 1] + 1.0;   // jitter may cross; belts may not
        }
    }
    /* Both guards above hold only while kWindGenJitter < 0.5 (consecutive nominal edges
       are w apart, jitter spans 2 * 0.15 * w) and the clamp keeps the outermost edge off
       the pole. Retune jitter past that and this asserts rather than emitting a band with
       lat0 >= lat1, which the loader would then refuse as its own bug. */
    assert(edge[(size_t)nb - 1] < 90.0);

    // -90, -e[nb-1], ... , -e[0], e[0], ... , e[nb-1], 90  ->  2*nb+1 bands, band nb is
    // the equatorial one, so a band's distance from the equator is |band - nb|.
    std::vector<double> E;
    E.reserve((size_t)(2 * nb + 2));
    E.push_back(-90.0);
    for(int k = nb - 1; k >= 0; k--) { E.push_back(-edge[(size_t)k]); }
    for(int k = 0; k < nb; k++) { E.push_back(edge[(size_t)k]); }
    E.push_back(90.0);

    std::vector<WindLevel> out;
    out.reserve(anchors.size() * (size_t)(2 * nb + 1));
    for(int b = 0; b <= 2 * nb; b++) {
        const int j = std::abs(b - nb);
        const double amp = std::pow(kWindGenDecay, (double)j);
        const double sign = (j % 2 == 0) ? 1.0 : -1.0;
        const double hemi = (b < nb) ? -1.0 : 1.0;   // the mirror, for the meridional leg
        for(const WindLevel &A : anchors) {
            WindLevel W = A;
            W.lat0 = E[(size_t)b];
            W.lat1 = E[(size_t)b + 1];
            W.lon0 = 0.0;
            W.lon1 = 360.0;
            W.east = A.east * amp * sign;
            W.north = A.north * amp * sign * hemi;
            W.up = A.up * amp * sign;
            out.push_back(W);
        }
    }
    assert(!out.empty());
    info.bands = (int)out.size() / (int)anchors.size();
    return out;
}

/* The air's velocity at (alt, lat, lon). lat/lon in DEGREES -- the same convention
   equirect.h unwraps to (a lon just outside [0, 360) is wrapped by windRowCovers), and
   every caller already has the place in hand: the aero path reads it off the hull's
   spin-frame direction, the surface sweep off its own pixel. There is no alt-only
   spelling on purpose: with banded rows, "what is the wind at 11 km" has no answer
   until someone says where.
   Returned as (east, north, up) in m/s -- the local horizontal basis of equirect.h
   plus the radial. The NARROWEST band covering the place answers for it, and within
   that band the profile is interpolated LINEARLY in alt, exactly as it was when every
   row was whole-sphere. Outside the band's rows the nearest end is held. */
inline glm::dvec3 windAt(const std::vector<WindLevel> &W, double alt,
                         double lat_deg, double lon_deg) {
    assert(!W.empty());   // the caller's gate, AtmosphereParams::has_wind(), is the law
    /* A place that is not a place gets no answer. Returning the nearest row instead
       would put a confident real wind under a hull whose position is NaN, and the
       NaN would then be a force several steps later with nothing left to trace. */
    assert(std::isfinite(alt) && std::isfinite(lat_deg) && std::isfinite(lon_deg));
    const WindLevel *lo = nullptr, *hi = nullptr;
    double best = 0.0;   // the narrowest band so far that covers this place
    for(const WindLevel &Wl : W) {
        if(!windRowCovers(Wl, lat_deg, lon_deg)) { continue; }
        /* The narrowest covering band owns the place (see the struct comment): a belt
           answers inside itself at every altitude, and the wider column it overrides
           answers elsewhere. Comparing areas rather than picking nearest-below/above
           across everything is what stops a whole-sphere row sitting between a belt's
           two rows from punching a sawtooth through the belt's own span. Rows of
           EQUAL area that both cover the place are considered together -- the loader
           cannot produce a meaningful difference between two same-sized bands over one
           place, and refusing that case would cost an author more rules than it saves. */
        const double area = windBandAreaDeg2(Wl);
        if(best > 0.0) {
            if(area > best) { continue; }
            if(area < best) { lo = nullptr; hi = nullptr; }
        }
        best = area;
        /* One row per altitude covers any place -- the loader refuses two rows at one
           alt whose bands overlap -- so "nearest below" and "nearest above" are the
           whole rule, and there is no tie to break. */
        if(Wl.alt <= alt && (lo == nullptr || Wl.alt > lo->alt)) { lo = &Wl; }
        if(Wl.alt >= alt && (hi == nullptr || Wl.alt < hi->alt)) { hi = &Wl; }
    }
    /* Outside the table the nearest end is HELD: the air thinning out and the air
       standing still are different claims, and a body whose only row is a jet aloft has
       said nothing about its surface. Either end can be missing -- below every covering
       row there is no "nearest below" -- so borrow the other one before asserting. */
    if(lo == nullptr) { lo = hi; }
    if(hi == nullptr) { hi = lo; }
    assert(lo && hi);   // the loader requires the bands to cover the whole sphere
    if(lo == hi || lo->alt == hi->alt) {
        return glm::dvec3(lo->east, lo->north, lo->up);
    }
    const double f = (alt - lo->alt) / (hi->alt - lo->alt);
    return glm::dvec3(lo->east + f * (hi->east - lo->east),
                      lo->north + f * (hi->north - lo->north),
                      lo->up + f * (hi->up - lo->up));
}

/* The weight of a column in the field the hull actually falls in (#683).
   `g` is the field at the DATUM, but the hull is pulled by
       g(z) = g radius^2 / (radius + z)^2
   (Vehicle::applyGravity: G m parent_mass / r^2 per part), and the integral of that
   field, measured in metres of the datum's g, is
       int_0^z g(z') dz' = g · geopotentialDepth(radius, z),
       geopotentialDepth(radius, z) = radius z / (radius + z)
   -- geopotential height, the substitution every standard-atmosphere table and
   Orbiter's layer models open with. It is the whole reason a KSP-sized world needs
   it: Kerbin's air top sits at 14.2 % of the body's radius since #686, where the
   field is already 23 % weaker, and a uniform-g column thins the top of that air
   by 4.6x -- 2.26 Pa at the 85 km edge weighed, 0.50 Pa uniform (#683's per-body
   table ran 1.04x on Jupiter to 4.3x on Eve; #700's warm tables on small worlds went
   past all of it -- 9.3x at Laythe's 105 km ceiling and 6.7x at Shay's 95 km, because
   a warm column on a 500 km body reaches 21 % of the radius. Duna's 5.4x went the
   other way, to 2.2x, once #700 fixed its scale height. #682 step 3 then moved the
   five giants' ceilings OUT to where the band says orbiting air still pushes, and
   every gain there went UP, not down -- a ceiling further out always weighs more
   against uniform g: Jupiter 1.04 -> 1.08, Neptune 1.08 -> 1.16, Saturn 1.11 -> 1.20,
   Uranus 1.08 -> 1.21, Jool 1.38 -> 1.83. That table's ends are still Laythe 9.3x and
   Jupiter 1.08x, which is the point: the gain tracks how far a ceiling reaches in
   body radii, and the giants' ceilings reach 0.6-4.6 %).
   radius <= 0 asks for the UNIFORM field instead, which is what a hand-built
   DragAtmosphere with no world gets; a body loaded from a system file always
   carries its seaLevelRadius. */
inline double geopotentialDepth(double radius, double z) {
    if(!(radius > 0.0)) { return z; }
    assert(radius + z > 0.0);   // callers floor at the datum; they do not divide by it
    return radius * z / (radius + z);
}

/* p [Pa] at alt inside level L, from hydrostatics (dp/dh = -rho g) and the ideal
   gas law (rho = p / (Rs T), Rs = R/M) with T linear in h. Eliminating h:
       dp/p = -(g / Rs) dh / T
   The SIGN is the part to re-check rather than trust: on a cooling level (a
   negative lapse, the troposphere) the temperature ratio runs the other way and
   the exponent stays negative, so p falls. Flipping either one makes the air get
   denser as you climb, and nothing downstream notices -- the density is derived
   from p, so a wrong sign never contradicts itself, it just describes Venus on
   Earth.
   ISOTHERMAL LEVEL (lapse == 0). T leaves the integral, and with the field above
       ln(p / base_p) = -(g / (Rs base_T)) [geopotentialDepth(z) - geopotentialDepth(base_alt)]
   For radius <= 0 the depths are just altitudes and this is the textbook
   p = base_p exp(-g dh / (Rs base_T)).
   LAPSE LEVEL. Here the textbook form
       p = base_p * (base_T / T) ^ (g / (lapse Rs))
   is ONLY right for a uniform field: T is authored per GEOMETRIC metre while the
   weight is 1/(radius+z)^2, and the two do not share an antiderivative. With
   u = radius + z, so T = lapse·u + C and C = base_T - lapse·(radius + base_alt):
       int du / (u^2 (lapse·u + C)) = -1/(C u) + (lapse/C^2) · ln((lapse·u + C)/u)
   evaluated at u and at the level's own base (where lapse·u_b + C IS base_T, so
   the log's second term is taken there rather than subtracted, which is where the
   cancellation would otherwise happen).
   Do NOT "fix" this by converting alt to geopotential height before the level
   maths, as Orbiter does: that makes the authored lapse per geopotential metre
   instead of per geometric one, and the error then grows with the lapse level's
   own depth -- 0.4 % at the top of Earth's 6371 km-radius mesosphere, 20 % at the
   same level on a 600 km body, 3.8 % at the ceiling of the 300 km two-level
   fixture pinned in tests/test_drag.cpp:test_gravity_falloff. The closed form
   above is the exact integral (it matches quadrature to ~1e-9; the geopotential-
   everywhere spelling does not).
   The uniform-field lapse form is kept as its own branch, and it is written with
   log1p rather than pow(base, exponent) so it KEEPS the isothermal limit: pow's
   base is a 1+-epsilon quotient, and at a 1e-16 lapse the author gets 1.5% off
   the answer its own table implies, in a branch whose whole job is to approach
   the isothermal case. log1p(lapse dh / base_T) IS log(T / base_T) by definition,
   so the two spellings agree to 7e-12 Pa on Earth's levels -- and a limit worth
   having is a limit that holds. */
inline double levelPressure(const AtmoLayer &L, double alt, double g, double Rs,
                            double radius) {
    const double dh = alt - L.base_alt;
    if(L.lapse == 0.0) {
        const double depth = geopotentialDepth(radius, alt)
                           - geopotentialDepth(radius, L.base_alt);
        return L.base_p * std::exp(-g * depth / (Rs * L.base_T));
    }
    // T > 0 is what makes log1p's argument > -1 and the log below's argument > 0;
    // a lapse steep enough to walk T through zero (which the loader refuses) would
    // produce a NaN here instead.
    if(!(levelTemperature(L, alt) > 0.0)) { return 0.0; }
    if(!(radius > 0.0)) {   // uniform field -- the textbook form
        return L.base_p * std::exp(-(g / (L.lapse * Rs))
                                   * std::log1p(L.lapse * dh / L.base_T));
    }
    const double u = radius + alt, u_b = radius + L.base_alt;
    const double C = L.base_T - L.lapse * u_b;
    const double k = g * radius * radius / Rs;
    const double F_u = -1.0 / (C * u)
                     + (L.lapse / (C * C)) * std::log(levelTemperature(L, alt) / u);
    const double F_b = -1.0 / (C * u_b) + (L.lapse / (C * C)) * std::log(L.base_T / u_b);
    return L.base_p * std::exp(-k * (F_u - F_b));
}

/* How far a level's authored base_T may sit from the temperature the level above
   leaves off at [K]. Not physics -- a data-acceptance tolerance. A profile written
   by hand rounds its boundary temperatures, and half a kelvin is the slack that
   takes; a missing level or a number copied in from a different standard is tens of
   kelvin off, which is exactly the mistake worth refusing. Earth's authored 1976
   table is continuous to the last digit, so this only ever catches the errors. */
inline constexpr double kAtmoSeamTol = 0.5;

/* How close to degenerate a level's lapse may come to the field's own falloff [K].
   See resolveAtmoLevels: C = base_T - lapse(radius + base_alt) is the distance
   between the two poles of the column integral -- the planet's CENTRE and the
   altitude where this level's T would hit 0 K -- and when C is 0 they coincide and
   levelPressure divides by it. A WARMING level on a small body can aim at that;
   reaching it takes a lapse tuned to within 1.6e-6 K/m of T/(radius+alt) on a
   Kerbin-sized world, and by |C| = 1 K the closed form has already given up four
   digits (against quadrature: 4.5e-9 relative at |C| = 1.1, 1e-4 at 0.1, 4e-3 at
   1e-3, garbage at 0). So the loader refuses the band instead of the law carrying
   a second spelling of itself for a case no authored table lands in.
   The band is ABSOLUTE in kelvin, so in lapse terms it is 1 K / (radius + base_alt)
   -- 1.6e-6 K/m on Kerbin, 3e-4 K/m on a 3 km air moon. That wider window is still
   200x narrower than the degenerate lapse there (T / (radius + alt) ~ 0.07 K/m,
   against the 1e-3..1e-2 K/m of any level anyone would author), which is the
   judgement behind the constant. tests/test_drag.cpp accepts |C| = 1.1 K and pins
   its density against a quadrature number; it refuses |C| = 0.5 K. */
inline constexpr double kAtmoPoleTol = 1.0;

/* The field a body's air is weighed IN: mu / seaLevelRadius()^2 (#683, #697, #708).
   Not a typed number -- the loader derives it from the same mu every arc answers to.
   The column's falloff is g radius^2/(radius+z)^2 with z counted from the sea-level
   DATUM sphere (terrain.h:seaLevelRadius()), while a hull falling through that air is
   pulled by G mass/(radius+z)^2. Those are ONE field only if the air's g is that field
   at the datum; otherwise the disagreement MULTIPLIES with altitude -- ln p spans 10.7
   scale heights across Kerbin's air, so a 5 % g is 41 % off on the density at its 85 km
   edge -- and play becomes a barometer that disagrees with a ballistic arc. Deriving it
   makes that class unauthorable, and a raised sea stops being a 4 % barometer bug
   because the datum sphere goes into the division directly. Both readers spell it the
   same way -- system.cpp (the rho0 anchor and resolveAtmoLevels) and
   vehicle.cpp:dragAtm -- so a profile cannot step between load and flight. */

/* Fill in each level's base_p [Pa] by integrating the level ABOVE it down to this
   level's base_alt. Level 0's base_p IS the file's authored `sea_level_pressure`
   (#739); the datum density the laws in drag.h read is the consequence, which
   system.cpp resolves as rho0 = p0 M / (R base_T(0)). Only the levels ABOVE the
   first are derived here -- authoring one of those would put a second, possibly
   contradictory, statement of the same column in the file (drag.h reads p and rho
   out of ONE integration, so a def that broke hydrostatics at a boundary would show
   up as a density jump nothing explains).

   `radius` is the body's SEA-LEVEL radius [m], the sphere this profile's altitudes
   are measured from; it is what makes the column weigh the 1/r^2 field the hull
   falls in rather than a uniform one (#683, atmo.h:geopotentialDepth). Pass it
   from the same TerrainBody the g came from: a level table resolved with one radius
   and read with another steps the air at every seam. 0 integrates the uniform field.

   Returns false when the table cannot be integrated, so the loader can name the body
   rather than load a profile that divides by zero or walks T through 0 K mid-air.
   The checks are: level 0 based at the datum; altitudes strictly rising and the last
   level starting below the body's air `top`; a positive temperature at every base and
   at every ceiling (a ceiling is the next level's base or the air top, whichever comes
   first); a temperature CONTINUOUS across every seam; no level whose air gets denser
   as you climb; and no warming level aimed at the degenerate lapse above. */
inline bool resolveAtmoLevels(std::vector<AtmoLayer> &L, double sea_level_pressure,
                              double molar_mass, double g, double top,
                              double radius) {
    if(L.empty() || !(sea_level_pressure > 0.0) || !(molar_mass > 0.0)
       || !(g > 0.0) || !(top > 0.0)) { return false; }
    if(!(L[0].base_alt == 0.0)) { return false; }    // the profile is anchored at the datum
    if(!(L.back().base_alt < top)) { return false; } // the last level needs air above it to lapse through
    const double Rs = kGasR / molar_mass;
    for(size_t i = 0; i < L.size(); ++i) {
        if(i > 0 && !(L[i].base_alt > L[i - 1].base_alt)) { return false; }
        if(!(L[i].base_T > 0.0)) { return false; }
        // d(rho)/dh = -(rho / T)(g / Rs + lapse), so the air gets DENSER with
        // altitude once lapse reaches -g / Rs. Refused: real air overturns long
        // before that (the convective limit is g / cp, stricter still), so a table
        // that does it is an authoring mistake rather than a hostile world.
        if(L[i].lapse <= -g / Rs) { return false; }
        const double ceiling = std::min((i + 1 < L.size()) ? L[i + 1].base_alt : top, top);
        if(ceiling > L[i].base_alt && levelTemperature(L[i], ceiling) <= 0.0) {
            return false;   // this lapse walks T through zero inside the air
        }
        // The seam. base_p is resolved rather than authored precisely so the column
        // cannot step at a boundary; T is the OTHER half of that statement, and T is
        // what rho divides by, so a level based on the wrong temperature is a density
        // cliff the pressure curve never shows -- a 100 K seam at 11 km measured a
        // 38 % step in rho with p identical to 13 digits. kAtmoSeamTol is the slack a
        // hand-written profile gets for rounding; a missing level or a number copied
        // from a different standard is nowhere near it.
        if(i > 0
           && std::fabs(levelTemperature(L[i - 1], L[i].base_alt) - L[i].base_T)
              > kAtmoSeamTol) {
            return false;
        }
        // The other divide-by-zero in the column (kAtmoPoleTol's note above). Only a
        // WARMING level can hit it: a cooling one has C = base_T + |lapse| u_b, which
        // is at least base_T away from zero.
        if(L[i].lapse > 0.0 && radius > 0.0
           && std::fabs(L[i].base_T - L[i].lapse * (radius + L[i].base_alt))
              < kAtmoPoleTol) {
            return false;
        }
        if(i == 0) {
            // The anchor: the pressure the file authored, at the sphere the altitudes
            // measure from. rho at that point is this divided by the level's own
            // temperature (system.cpp resolves it into sea_level_density), so p and
            // rho describe one column and the file cannot state a weight twice.
            L[0].base_p = sea_level_pressure;
        } else {
            L[i].base_p = levelPressure(L[i - 1], L[i].base_alt, g, Rs, radius);
            if(!(L[i].base_p > 0.0)) { return false; }
        }
    }
    return true;
}

/* WHAT THE AIR IS MADE OF (#552 A4b, #630).

   A body used to state two hand-typed scalars -- molar_mass and gamma -- and those
   two were the whole statement about chemistry. They are not independent numbers:
   they are what a MIX resolves to, and a mix is the thing a player could act on
   ("this world has no free oxygen" is a fact about a world, and today nothing in the
   game can say it). So a system file authors the mix and the loader derives the
   scalars from it, exactly like resolveAtmoLevels cascades base_p from an authored
   temperature profile plus one authored datum pressure: one authored statement, its
   consequences filled in at load,
   and no second number that can contradict it.

   M [kg/mol] and gamma = cp/cv are the values each gas reaches at the temperatures
   OUR air sits at: CO2 1.30 rather than the 1.29-1.33 it moves through, H2/He at
   their room-temperature values. A gas's gamma is temperature-dependent well before
   any body in the game crosses the temperature where that matters (Jool's air is 44.6 K
   in the atlas), so pinning one number per gas is the same honesty level as one mean
   molar mass below the homopause -- and it is ONE number per gas, in one place,
   rather than one per body per editor session. */
struct GasSpecies {
    const char *name;    // what a system file types, and what the atlas prints
    double molar_mass;   // [kg/mol]
    double gamma;        // cp/cv
    bool oxidizer;       // can a turbine burn against this gas? (O2, today)
};

inline const GasSpecies *gasSpecies(const std::string &name) {
    static const GasSpecies table[] = {
        {"N2",  0.0280134, 1.40,  false},
        {"O2",  0.0319988, 1.40,  true },
        {"CO2", 0.0440095, 1.30,  false},
        {"H2",  0.00201588, 1.40, false},   // a fuel, not an oxidizer
        {"He",  0.0040026, 1.667, false},
        {"CH4", 0.0160425, 1.32,  false},
        {"H2O", 0.0180153, 1.33,  false},
        {"Ar",  0.039948,  1.667, false},
    };
    for(const GasSpecies &g : table) {
        if(name == g.name) { return &g; }
    }
    return nullptr;
}

/* One line of a body's authored mix: a gas and how much of the air is that gas, by
   MOLE (`fraction`). The pointer is into gasSpecies()'s static table, so a mix is
   cheap to carry and cannot name a gas the table does not know -- system.cpp turns
   the file's strings into this, and refuses a name the table does not have. */
struct MixFraction {
    const GasSpecies *gas = nullptr;
    double fraction = 0.0;   // mole fraction as authored; need not sum to 1
};

/* The air a body gets when its file says nothing: dry Earth air, oxygen and all.
   This is the mix behind constants.h's kAirMolarMass / kAirGamma defaults -- the
   numbers there are this mix's consequences, kept because hand-written test systems
   state a scale height and nothing else. */
inline const MixFraction *dryAirMix(int &count) {
    static const MixFraction air[] = {
        {gasSpecies("N2"),  0.78084},
        {gasSpecies("O2"),  0.20946},
        {gasSpecies("Ar"),  0.00934},
        {gasSpecies("CO2"), 0.00033},
    };
    count = (int)(sizeof air / sizeof air[0]);
    return air;
}

/* Resolve a mix into the three numbers the rest of the game reads:
     molar_mass [kg/mol]   = sum(x_i M_i)             -- linear in mole fraction
     gamma                 -- from the MOLE-WEIGHTED molar heat capacity, NOT a
                              weighted average of the gammas: cp_i = R g_i/(g_i-1)
                              per mole, cp_mix = sum(x_i cp_i), then
                              gamma_mix = cp_mix / (cp_mix - R). Heat capacities add;
                              ratios do not, so a mole-weighted AVERAGE of the gammas
                              is a different number -- Jool's H2/He/CH4 air averages to
                              1.437 where this gives 1.423. Only 0.5% on its sound
                              speed, but it is wrong in kind, and it is the arithmetic
                              a reader would otherwise re-invent here.
     oxidizer_share        = the MASS fraction of the air that is oxidizer gas,
                              sum(x_i M_i) over oxidizers / molar_mass. It is a mass
                              fraction because an intake swallows a VOLUME of air and
                              pays for it in kilograms (drag.h:jetAirGate), while the
                              composition is authored in MOLE fractions.
   Fractions need not sum to 1: they are normalized by their own total, so a body can
   author "the rest is N2" as 1.0 or as 0.027. Returns false with the reason for a
   mix that cannot be resolved -- empty, an unknown gas, a negative fraction, or a
   total of zero -- so the loader names the body instead of loading air with no gas
   in it (a divide by a zero molar mass, several steps later). */
inline bool resolveMix(const std::vector<MixFraction> &mix, double &molar_mass,
                       double &gamma, double &oxidizer_share, std::string &why) {
    if(mix.empty()) { why = "the mix is empty"; return false; }
    double total = 0.0;
    for(const MixFraction &m : mix) {
        if(m.gas == nullptr) { why = "a gas name is not in the species table"; return false; }
        // !(x >= 0) catches a negative fraction AND a NaN. A JSON file cannot hand
        // us +inf -- its reader refuses an out-of-range literal outright ("number
        // overflow parsing '1e400'", pinned in test_scenarios) -- but a mix built in
        // code can, and a huge-but-finite set of them can overflow the total to inf,
        // which lands on the zero-molar-mass refusal below.
        if(!(m.fraction >= 0.0)) { why = "a fraction is negative or NaN"; return false; }
        total += m.fraction;
    }
    if(!(total > 0.0)) { why = "every fraction is zero"; return false; }
    double cp = 0.0, oxMolar = 0.0;
    molar_mass = 0.0;
    for(const MixFraction &m : mix) {
        const double x = m.fraction / total;
        molar_mass += x * m.gas->molar_mass;
        cp += x * (kGasR * m.gas->gamma / (m.gas->gamma - 1.0));
        if(m.gas->oxidizer) { oxMolar += x * m.gas->molar_mass; }
    }
    if(!(molar_mass > 0.0)) { why = "the resolved molar mass is zero"; return false; }
    // cp > R for any gas (cp - cv = R, and cv > 0), so this divide is safe once
    // every fraction and gamma in [1, 2] has passed the checks above.
    gamma = cp / (cp - kGasR);
    oxidizer_share = oxMolar / molar_mass;
    return true;
}

/* The same mix resolved from the default dry air, so the reference an intake is
   rated against comes from the ONE table rather than from a typed constant that has
   to be re-derived whenever the table moves (drag.h:jetAirGate). */
inline double dryAirOxidizerShare() {
    static const double share = [] {
        int n = 0;
        const MixFraction *air = dryAirMix(n);
        std::vector<MixFraction> v(air, air + n);
        double m = 0.0, g = 0.0, ox = 0.0;
        std::string why;
        return resolveMix(v, m, g, ox, why) ? ox : 0.0;
    }();
    return share;
}
