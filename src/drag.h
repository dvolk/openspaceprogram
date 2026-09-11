#pragma once

// drag.h -- the atmospheric-drag law as pure math (glm + <cmath> only: no
// Bullet, no GL, no game state), so tests/ can pin it without the render
// chain (like surfmap.h, orbit.h, evamath.h).
//
//   airDensity(...)  the exponential density model: rho(alt) = rho0 * e^(-alt/H)
//   dragForce(...)   the force a body feels: -v̂ · ½ · rho · Cd · A · |v|²
//
// The frame assumption that makes this clean: a ship inside a body's
// atmosphere is always in that body's ROTATING frame (the atmosphere is
// far shallower than the SoI, so no SoI handoff happens in the air). The
// air co-rotates with the planet, so in that frame the air is at rest and
// the ship's frame velocity IS the air-relative velocity -- no stasis /
// frame-conversion term is needed. See reports/atmospheric-drag.

#include <cmath>

#include <glm/glm.hpp>

/* The physical half of a body's atmosphere (the render half -- colour,
   Fresnel power -- lives in AtmosphereParams, terragen.h). Both zero means
   "no drag" (a body may draw a limb rim without an atmosphere for physics). */
struct DragAtmosphere {
    double sea_level_density = 0.0;  // kg/m^3 at alt 0; 0 = no drag
    double scale_height = 0.0;       // [m]; density /e-fold altitude
    DragAtmosphere() {}
    DragAtmosphere(double rho0, double H)
        : sea_level_density(rho0), scale_height(H) {}
};

/* Density [kg/m^3] at `alt` metres above the surface:
     rho(alt) = sea_level_density · exp(−alt / scale_height)
   Below the surface (alt <= 0) there is no air to push through, and a
   degenerate atmosphere (no density, no scale height) reads as none.
   The exponential is self-limiting -- at alt = 8·H the density is ~0.03%
   of sea level -- so no hard "atmosphere top" is needed. */
inline double airDensity(const DragAtmosphere &a, double alt) {
    if(a.sea_level_density <= 0.0 || a.scale_height <= 0.0) { return 0.0; }
    if(alt <= 0.0) { return 0.0; }
    return a.sea_level_density * std::exp(-alt / a.scale_height);
}

/* The drag force (a vector) on a body of cross-sectional `area` [m^2] and
   coefficient `cd`, moving at `v_rel` through air of density rho(alt):
     F = −v̂ · ½ · rho(alt) · cd · area · |v_rel|²
   Opposite the motion, quadratic in speed. Zero for any degenerate input
   (no atmosphere, no speed, no area, cd 0) so callers need no guards. */
inline glm::dvec3 dragForce(const DragAtmosphere &a, double cd, double area,
                            double alt, const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || cd <= 0.0 || area <= 0.0) {
        return glm::dvec3(0.0);
    }
    return -glm::normalize(v_rel) * (0.5 * rho * cd * area * v * v);
}
