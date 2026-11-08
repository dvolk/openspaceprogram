#pragma once
/* solar.h -- the star's flux, in ONE place (#687).
   Pure data plus arithmetic: no bodies, no frames, no globals. Everything here is
   a free function over numbers and directions, so a unit test can pin it without a
   world loaded (the atmo.h / drag.h shape).

   THE LAW. A star is a sphere radiating as a black body, so its output is
       L = 4 pi R^2 sigma T^4                     [W]
   and the flux crossing a sphere of radius d around it is
       S(d) = L / (4 pi d^2) = sigma T^4 (R/d)^2  [W/m^2]
   which is just "the star's fraction of the sky". Authoring T (one number per
   star, on surface.temperature) is therefore authoring L, and the flux at every
   body follows from the orbit the file already states.

   WHAT EVERY BUYER TAKES FROM HERE.
    * a ground radiometer / a surface temperature: S times the cosine of the sun's
      elevation (incidentFlux), and the absorbed share is S (1 - albedo);
    * a part's sun-facing skin (#553): the SAME incidentFlux, with the part's own
      normal and its own absorptivity -- NOT the air's, which is why this is not
      derived from the aero path;
    * a seasonal cycle (roadmap rung 13): S is the only term that varies with the
      orbit, so it is the quantity a subsolar-latitude law multiplies.
   One flux, three buyers -- that is why it gets a module before any of them.

   WHAT THIS IS *NOT*, said out loud so nobody re-files it as a bug:
    * NO EXTINCTION. S is top-of-atmosphere. Flying at sea level under Venus's
      92-bar sky dims nothing here, and the 55 km deck is optically nothing. An
      atmosphere absorbing sunlight is its own law with its own optical depth.
    * NO ECLIPSE. A body behind a body still receives full S. The renderer has a
      sun-shadow test (terrain.h DrawRings/ring shadows) that this does not consult.
    * NO ALBEDO PHYSICS. Albedo is a per-body authored number that enters only
      blackbodyTemperature below; it does not warm or cool anything yet.
    * NO REDISTRIBUTION model beyond the factor 4 in blackbodyTemperature -- the
      global-mean convention (intercept pi r^2, radiate 4 pi r^2). A substellar
      point or a night-side floor is a different convention with a different
      factor, and picking one silently is how "the day-side temperature" ends up
      meaning three things in one file.

   THE META-LAW (#684/#686, applying to us again): a star's T is ONE authored
   number, so no internal check can see it go wrong -- L and every flux in the
   system are derived from it. The second statement lives in the generator:
   utils/rss/html/<body>fact.html states "Solar irradiance (W/m^2)" and
   "Black-body temperature (K)" per body, and utils/make_solar_system.py --check
   weighs our derived S against them (#685's discipline on the new axis). */

#include <algorithm>   // max
#include <cassert>
#include <cmath>
#include <numbers>

#include <glm/glm.hpp>

/* CODATA 2018, exact-in-SI digits. Named so no call site writes the literal --
   the same reason the gravitational constant has a ticket (#707): four spellings
   of one constant, four chances to disagree. */
inline constexpr double kStefanBoltzmann = 5.670374419e-8;   // [W / m^2 K^4]

/* [m] Below this, "how far from the star" has no answer worth a number -- the sun
   itself, or a degenerate system. ONE floor, so solarFlux and sunSample cannot
   disagree about what counts as a distance: spelled separately, a direct
   solarFlux(L, 1e-12) returns an enormous finite flux with every assert happy while
   sunSample would have called the same point invalid. */
inline constexpr double kSunMinDistance_m = 1e-9;

/* [W] A star's total output from its own radius and effective temperature.
   T = 0 is legal and means "no star" (airless fixtures, a body authored before
   the number existed); a negative T is a typo, and T^4 would hide it. */
inline double starLuminosity(double radius_m, double temp_K) {
    assert(radius_m >= 0.0 && std::isfinite(radius_m));
    assert(temp_K >= 0.0 && std::isfinite(temp_K));
    if(radius_m <= 0.0 || temp_K <= 0.0) { return 0.0; }
    const double t2 = temp_K * temp_K;
    return 4.0 * std::numbers::pi * radius_m * radius_m
         * kStefanBoltzmann * t2 * t2;
}

/* [W/m^2] Flux crossing a sphere of radius dist_m. 0 means "no star to receive",
   which the callers print as a dash, not as a temperature of 0 K. */
inline double solarFlux(double luminosity_W, double dist_m) {
    assert(std::isfinite(luminosity_W) && std::isfinite(dist_m));
    if(luminosity_W <= 0.0 || dist_m <= kSunMinDistance_m) { return 0.0; }
    return luminosity_W / (4.0 * std::numbers::pi * dist_m * dist_m);
}

/* [W/m^2] On a flat surface with unit normal n, lit by a source in unit
   direction to_sun: the cosine, floored at 0 (a night side collects nothing; a
   NEGATIVE cosine must not read as the surface being lit from behind).
   n and to_sun are assumed unit -- callers normalise once and reuse. */
inline double incidentFlux(double flux, const glm::dvec3 &n,
                           const glm::dvec3 &to_sun) {
    if(flux <= 0.0) { return 0.0; }
    return flux * std::max(0.0, glm::dot(n, to_sun));
}

/* [K] The global-mean black-body temperature of a fast rotator at flux S with
   Bond albedo albedo: absorbed power pi r^2 S (1-A), emitted 4 pi r^2 sigma T^4.
   This is the convention the fact sheets' "Black-body temperature (K)" row uses
   (Earth: S 1361.0, A 0.294 -> 255 K against the sheet's 254.0). It is NOT a
   surface temperature -- Earth's mean is 288 K and the 34 K gap is the greenhouse
   this game does not model. albedo < 1 asserts: A >= 1 absorbs nothing, which is
   an authoring error, not a cold planet. */
inline double blackbodyTemperature(double flux, double albedo) {
    assert(albedo >= 0.0 && albedo < 1.0);
    if(flux <= 0.0) { return 0.0; }
    const double emit = flux * (1.0 - albedo) / 4.0;
    return std::pow(emit / kStefanBoltzmann, 0.25);
}

/* One sample of the star's light at one point, in universe-root (== star) axes.
   `to_sun` is the unit direction FROM the point TO the star; it is (0,0,0) when
   the point is at the star (the sun itself, or a degenerate system), and then
   flux/distance print as dashes rather than as infinity. */
struct SunSample {
    double dist_m = 0.0;
    double flux = 0.0;              // [W/m^2], normal to the beam
    glm::dvec3 to_sun = glm::dvec3(0.0);
    bool valid = false;
};

inline SunSample sunSample(double star_luminosity, const glm::dvec3 &star_root,
                           const glm::dvec3 &point_root) {
    SunSample ss;
    const glm::dvec3 d = point_root - star_root;
    ss.dist_m = glm::length(d);
    if(star_luminosity <= 0.0 || ss.dist_m <= kSunMinDistance_m) { return ss; }
    ss.to_sun = d * (-1.0 / ss.dist_m);
    ss.flux = solarFlux(star_luminosity, ss.dist_m);
    ss.valid = true;
    return ss;
}

/* [rad] A source's elevation above the local horizon at a point whose outward
   normal is `up` (unit). asin of the dot, so +pi/2 is overhead and a negative
   reading is night. Clamped: a non-unit up (a hull at the exact centre of a
   body) must not make asin return NaN. */
inline double sunElevation(const glm::dvec3 &up, const glm::dvec3 &to_sun) {
    const double s = std::clamp(glm::dot(up, to_sun), -1.0, 1.0);
    return std::asin(s);
}
