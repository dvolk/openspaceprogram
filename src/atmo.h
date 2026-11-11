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
#include <string>
#include <vector>

#include <glm/glm.hpp>  // windAt returns a horizontal pair as one dvec2

#include "constants.h"  // kGasR, for the hydrostatic integration below

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

   windAt() interpolates LINEARLY between the two rows spanning alt and holds the
   nearest end's value outside the table -- below the first row (a body whose first row
   is a jet aloft has no reason to claim the surface is calm) and above the last (the
   air thins out, it does not calm down). A loader refuses an unsorted row list, a row
   missing or misspelling a key, a non-finite number, a table past 64 rows (windAt
   scans linearly on the aero path), or a COMPONENT past kWindMaxSpeed -- per axis, so a
   row may total up to 500*sqrt2. That bound is a typo net, not physics -- a steady
   500 m/s flow is a Mach-1.5 wind everywhere at once, which is a decimal point in the
   wrong place, not weather. */
struct WindLevel {
    double alt = 0.0;    // [m above the sea-level datum], GEOMETRIC
    double east = 0.0;   // [m/s] toward increasing lon, relative to the ground
    double north = 0.0;  // [m/s] toward increasing lat, relative to the ground
};

/* Loosest bound that still catches a mistyped row: the fastest wind on any published
   target in utils/rss/html is a few tens of m/s, a terrestrial hurricane tops out near
   113, and every aero law downstream reads it as a Mach number. */
inline constexpr double kWindMaxSpeed = 500.0;   // m/s, per component

inline glm::dvec2 windAt(const std::vector<WindLevel> &W, double alt) {
    assert(!W.empty());   // the caller's gate, AtmosphereParams::has_wind(), is the law
    if(alt <= W.front().alt) { return glm::dvec2(W.front().east, W.front().north); }
    if(alt >= W.back().alt) { return glm::dvec2(W.back().east, W.back().north); }
    // Rows are sorted at load, so a linear scan finds the span; a table is a handful
    // of rows and this runs once per hull per substep, not once per surface.
    for(size_t i = 1; i < W.size(); ++i) {
        if(alt <= W[i].alt) {
            const double span = W[i].alt - W[i - 1].alt;
            assert(span > 0.0);   // the loader's strictly-increasing rule
            const double f = (alt - W[i - 1].alt) / span;
            return glm::dvec2(W[i - 1].east + f * (W[i].east - W[i - 1].east),
                              W[i - 1].north + f * (W[i].north - W[i - 1].north));
        }
    }
    return glm::dvec2(W.back().east, W.back().north);   // unreachable; not a silent 0
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
