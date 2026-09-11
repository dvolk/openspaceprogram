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

/* The off-axis (weathervane) factor: how far the ship is deflected from its
   nose axis, as the square of that angle -- 1 - (v̂·n̂)².
     0 = the nose points straight into the flow (v̂ along n̂, prograde)
     1 = the ship is flying exactly sideways (v̂ ⟂ n̂)
   Both a pitch (W/S) and a bank (A/D) deflection count -- any attitude away
   from prograde presents more area to the air. Zero for degenerate input.
   This is the "the ship turned off its nose, so it drags more" term that
   v1's attitude-independent -v̂ force lacks. */
inline double offAxisFactor(const glm::dvec3 &v_rel, const glm::dvec3 &nose) {
    const double v = glm::length(v_rel);
    const double n = glm::length(nose);
    if(v <= 0.0 || n <= 0.0) { return 0.0; }
    const double c = glm::dot(v_rel, nose) / (v * n);
    const double s2 = 1.0 - c * c;
    return (s2 < 0.0) ? 0.0 : s2;   // clamp to [0, 1]
}

/* The AoA-aware drag force: the v1 parasite term plus a weathervane term
   that grows with the square of the ship's deflection from its nose:
     F = -v̂ · ½ · rho(alt) · v² · (A0 + AK · (1 - (v̂·n̂)²))
   A0 = the parasite (attitude-independent) drag area, AK = the weathervane
   drag area (both already folded from the parts' area x coefficient -- see
   Vehicle::applyAeroForce). With AK = 0 this is exactly dragForce with
   cd·area = A0, so the v1 law is the AK = 0 special case. Zero for any
   degenerate input (no air, no speed, no area, bad nose axis). */
inline glm::dvec3 dragForceAOA(const DragAtmosphere &a, double A0, double AK,
                               double alt, const glm::dvec3 &v_rel,
                               const glm::dvec3 &nose) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || (A0 + AK) <= 0.0) {
        return glm::dvec3(0.0);
    }
    const double area = A0 + AK * offAxisFactor(v_rel, nose);
    return -glm::normalize(v_rel) * (0.5 * rho * area * v * v);
}

/* The aerodynamic flow frame: the ship's air-relative velocity decomposed
   against its body axes (x right, y up, z nose) plus the derived angles the
   force laws consume. `right`, `up`, `nose` (the ship's body axes) and
   `v_rel` must be in the SAME frame (world). Pure math (glm + <cmath>) so
   tests/ can pin it without Bullet/GL. Phase 2 (lift) reads alpha off this. */
struct AeroFrame {
    double v = 0.0;         // speed |v_rel|
    double alpha = 0.0;     // pitch angle of attack (flow vs nose, Y-Z plane), rad
    double beta = 0.0;      // sideslip (flow vs nose, X-Z plane), rad
    double offAxis = 0.0;   // 1 - (v̂·n̂)², total deflection from the nose (0..1)
    bool valid = false;     // false when v=0 or the nose axis is degenerate
};

inline AeroFrame aeroFrame(const glm::dvec3 &v_rel,
                           const glm::dvec3 &right, const glm::dvec3 &up,
                           const glm::dvec3 &nose) {
    AeroFrame f;
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0) { return f; }
    f.v = v;
    f.alpha   = std::atan2(glm::dot(v_rel, up), glm::dot(v_rel, nose));
    f.beta    = std::atan2(glm::dot(v_rel, right), glm::dot(v_rel, nose));
    f.offAxis = offAxisFactor(v_rel, nose);
    f.valid = true;
    return f;
}

/* The dynamic pressure q = 0.5 · rho(alt) · |v_rel|² -- the factor common to
   the lift and drag laws (the air's kinetic energy per unit volume). Pulled
   out so a set of parts shares one q instead of each recomputing rho. Zero
   for any degenerate input (no air, or at rest in the air). */
inline double dynamicPressure(const DragAtmosphere &a, double alt,
                              const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    if(rho <= 0.0) { return 0.0; }
    return 0.5 * rho * glm::dot(v_rel, v_rel);
}

/* The lift direction: the ship's up axis (nose x right, the wing normal)
   with its component along the flow removed -- the axis the lift acts along.
   Lift is the aero force OUT of the relative flow, and for a wing aligned
   with the ship that is the up axis projected perpendicular to the flow.
   Perpendicular to v_rel by construction, and on the ship's "up" side, so a
   section at positive AoA pushes up and one at negative AoA pushes down (the
   sign is carried by CL, see liftForce). Zero for degenerate input. */
inline glm::dvec3 liftDirection(const glm::dvec3 &v_rel,
                                const glm::dvec3 &right,
                                const glm::dvec3 &nose) {
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0 || glm::length(right) <= 0.0) {
        return glm::dvec3(0.0);
    }
    const glm::dvec3 up   = glm::cross(nose, right);   // the ship's +Y
    const glm::dvec3 vhat = v_rel / v;
    const glm::dvec3 l    = up - glm::dot(up, vhat) * vhat;  // up, out of flow
    const double len = glm::length(l);
    if(len <= 0.0) { return glm::dvec3(0.0); }
    return l / len;
}

/* The lift force on one part:
     L = q · S · cl · alpha · liftDir
   q = dynamic pressure (0.5·rho·v²), S = the part's lift_area [m²], cl =
   its lift-curve slope (dimensionless, per radian), alpha = the angle of
   attack (rad). A symmetric section (CL0 = 0) has no lift at zero AoA; the
   sign of the force follows alpha (negative AoA pushes down). No stall
   clamp yet (Phase 3). Zero for any degenerate input. */
inline glm::dvec3 liftForce(double q, double S, double cl, double alpha,
                            const glm::dvec3 &liftDir) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0) { return glm::dvec3(0.0); }
    return liftDir * (q * S * cl * alpha);
}

/* The drag on one part, given the shared flow factors (q = 0.5·rho·v²,
   offAxis = 1-(v̂·n̂)², vhat = v̂):
     F = -vhat · q · (area·cd + area·k·offAxis)
   This is the per-part form of dragForceAOA with the shared factors (the
   density, the speed, the off-axis penalty) pulled out, so a set of parts'
   forces SUM to exactly the composite dragForceAOA -- the v1 law at AK = 0.
   Zero for degenerate input. */
inline glm::dvec3 partDrag(double q, const glm::dvec3 &vhat, double offAxis,
                           double area, double cd, double k) {
    const double a = area * cd + area * k * offAxis;
    if(q <= 0.0 || a <= 0.0) { return glm::dvec3(0.0); }
    return -vhat * (q * a);
}
