// atmo.h -- an atmosphere's altitude/temperature PROFILE: the one type a layered
// atmosphere is made of, plus the level-local math that type implies. Read by the
// authored half (terragen.h:AtmosphereParams, parsed from a system file), the
// loader that resolves it (system.cpp) and the aero law that samples it
// (drag.h:airDensity/airPressure/airTemperature/airSoundSpeed).
//
// It lives in its own header because all three need the exact same struct and
// none of them may include another as a peer: terragen.h is the terrain generator,
// drag.h is the aero law. constants.h is the other leaf they share, and it holds
// numbers rather than types. What is HERE is level-LOCAL -- one AtmoLayer plus the
// body's g and its gas. Deciding WHICH level an altitude falls in belongs to drag.h,
// because that is where a body's table, its top and its flat floor live.

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

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

   `base_p` is NOT authored. It is the pressure at `base_alt` [Pa], filled in when
   the system file loads by integrating the level above hydrostatically from the
   sea-level anchor p0 = sea_level_density * (R/M) * base_T(0) (resolveAtmoLevels,
   called by system.cpp).
   Authoring it would add a number that the law already implies and let a def
   quietly break hydrostatic equilibrium at a level boundary; the load pass is
   where the pair is made consistent, and the density there stays EXACTLY the
   authored sea_level_density at the datum.

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
     - lapse above -g / Rs, the point past which the air gets DENSER as you climb. */
struct AtmoLayer {
    double base_alt = 0.0;   // [m above the sea-level datum]
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

/* p [Pa] at alt inside level L, from hydrostatics (dp/dh = -rho g) and the ideal
   gas law (rho = p / (Rs T), Rs = R/M) with T linear in h. Eliminating h:
       dp/p = -(g / Rs) dh / T   and   dh = dT / lapse
       =>  ln(p / base_p) = (g / (Rs lapse)) · ln(base_T / T)
   which gives the two textbook forms:
       lapse != 0:  p = base_p * (base_T / T) ^ (g / (lapse Rs))
       lapse  == 0: p = base_p * exp(-g (alt - base_alt) / (Rs base_T))
   The SIGN is the part to re-check rather than trust: on a cooling level (a
   negative lapse, the troposphere) the ratio base_T / T is ABOVE 1 and the
   exponent is NEGATIVE, so p falls. Flipping either one makes the air get denser
   as you climb, and nothing downstream notices -- the density is derived from p,
   so a wrong sign never contradicts itself, it just describes Venus on Earth.
   The isothermal form IS the lapse form's limit as lapse goes to zero, and is
   spelled separately rather than near-factored: the exponent becomes a 0/0-shaped
   ratio. The lapse form is written with log1p rather than pow(base, exponent) so
   it KEEPS that limit: pow's base is a 1+-epsilon quotient, and at a 1e-16 lapse
   the author gets 1.5% off the answer its own table implies, in a branch whose
   whole job is to approach the isothermal case. log1p(lapse dh / base_T) IS
   log(T / base_T) by definition, so the two spellings agree to 7e-12 Pa on
   Earth's levels -- and a limit worth having is a limit that holds. */
inline double levelPressure(const AtmoLayer &L, double alt, double g, double Rs) {
    const double dh = alt - L.base_alt;
    if(L.lapse == 0.0) {
        return L.base_p * std::exp(-g * dh / (Rs * L.base_T));
    }
    // T > 0 is what makes log1p's argument > -1; a lapse steep enough to walk T
    // through zero (which the loader refuses) would produce a NaN here instead.
    if(!(levelTemperature(L, alt) > 0.0)) { return 0.0; }
    return L.base_p * std::exp(-(g / (L.lapse * Rs))
                               * std::log1p(L.lapse * dh / L.base_T));
}

/* How far a level's authored base_T may sit from the temperature the level above
   leaves off at [K]. Not physics -- a data-acceptance tolerance. A profile written
   by hand rounds its boundary temperatures, and half a kelvin is the slack that
   takes; a missing level or a number copied in from a different standard is tens of
   kelvin off, which is exactly the mistake worth refusing. Earth's authored 1976
   table is continuous to the last digit, so this only ever catches the errors. */
inline constexpr double kAtmoSeamTol = 0.5;

/* Fill in each level's base_p [Pa] by integrating the level ABOVE it down to this
   level's base_alt, starting from the sea-level anchor the authored numbers imply:
       p0 = sea_level_density * (R / M) * base_T(0)      (ideal gas, at the datum)
   That is why base_p is not authored: it is a consequence, and authoring it would
   put a second, possibly contradictory, statement of the same column in the file
   (drag.h reads p and rho out of ONE integration, so a def that broke hydrostatics
   at a boundary would show up as a density jump nothing explains).

   Returns false when the table cannot be integrated, so the loader can name the body
   rather than load a profile that divides by zero or walks T through 0 K mid-air.
   The checks are: level 0 based at the datum; altitudes strictly rising and the last
   level starting below the body's air `top`; a positive temperature at every base and
   at every ceiling (a ceiling is the next level's base or the air top, whichever comes
   first); a temperature CONTINUOUS across every seam; and no level whose air gets
   denser as you climb. */
inline bool resolveAtmoLevels(std::vector<AtmoLayer> &L, double sea_level_density,
                              double molar_mass, double g, double top) {
    if(L.empty() || !(sea_level_density > 0.0) || !(molar_mass > 0.0)
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
        if(i == 0) {
            L[0].base_p = sea_level_density * Rs * L[0].base_T;
        } else {
            L[i].base_p = levelPressure(L[i - 1], L[i].base_alt, g, Rs);
            if(!(L[i].base_p > 0.0)) { return false; }
        }
    }
    return true;
}
