#pragma once

// drag.h -- the atmospheric-drag law as pure math (glm + <cmath> only: no
// Bullet, no GL, no game state), so tests/ can pin it without the render chain.
//
//   airDensity(...)  the exponential density model: rho(alt) = rho0 * e^(-alt/H)
//   dragForce(...)   the force a body feels: -v̂ · ½ · rho · Cd · A · |v|²
//
// The air co-rotates with the planet: it is at rest in the body's ROTATING
// frame, so there the ship's frame velocity IS the air-relative velocity.
// In any other frame Vehicle::airRelativeVel subtracts the co-rotation term
// (issues #60/#97). The loader sizes near-body shells to contain their air
// (bodylimits.h), so live ships hit that branch only transiently or via
// saves written under an older shell model.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

#include "constants.h"  // kRhoFloor

/* The physical half of a body's atmosphere (the render half lives in
   AtmosphereParams, terragen.h). Both zero means "no drag". */
struct DragAtmosphere {
    double sea_level_density = 0.0;  // kg/m^3 at alt 0; 0 = no drag
    double scale_height = 0.0;       // [m]; density /e-fold altitude
    /* The hard top [m above sea level]: at or above it the air is vacuum.
       0 = no cutoff (the exponential tail runs down to kRhoFloor). */
    double height = 0.0;
    DragAtmosphere() {}
    DragAtmosphere(double rho0, double H, double top = 0.0)
        : sea_level_density(rho0), scale_height(H), height(top) {}
};

/* Density [kg/m^3] at `alt` metres above the surface:
     rho(alt) = sea_level_density · exp(−alt / scale_height)
   At or above `height` the air stops dead (a hard top). */
inline double airDensity(const DragAtmosphere &a, double alt) {
    if(a.sea_level_density <= 0.0 || a.scale_height <= 0.0) { return 0.0; }
    if(alt <= 0.0) { return 0.0; }
    if(a.height > 0.0 && alt >= a.height) { return 0.0; }
    return a.sea_level_density * std::exp(-alt / a.scale_height);
}

/* The drag force on a body of cross-sectional `area` [m^2] and coefficient
   `cd`, moving at `v_rel` through air of density rho(alt):
     F = −v̂ · ½ · rho(alt) · cd · area · |v_rel|²
   Zero for any degenerate input so callers need no guards. */
inline glm::dvec3 dragForce(const DragAtmosphere &a, double cd, double area,
                            double alt, const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || cd <= 0.0 || area <= 0.0) {
        return glm::dvec3(0.0);
    }
    return -glm::normalize(v_rel) * (0.5 * rho * cd * area * v * v);
}

/* The aerodynamic flow frame: the ship's air-relative velocity decomposed
   against its body axes (x right, y up, z nose). */
struct AeroFrame {
    double v = 0.0;         // speed |v_rel|
    double alpha = 0.0;     // pitch angle of attack (flow vs nose, Y-Z plane), rad
    double beta = 0.0;      // sideslip (flow vs nose, X-Z plane), rad
    bool valid = false;     // false when v=0 or the nose axis is degenerate
};

inline AeroFrame aeroFrame(const glm::dvec3 &v_rel,
                           const glm::dvec3 &right, const glm::dvec3 &up,
                           const glm::dvec3 &nose) {
    AeroFrame f;
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0) { return f; }
    f.v = v;
    f.alpha = std::atan2(glm::dot(v_rel, up), glm::dot(v_rel, nose));
    f.beta  = std::atan2(glm::dot(v_rel, right), glm::dot(v_rel, nose));
    f.valid = true;
    return f;
}

/* The dynamic pressure q = 0.5 · rho(alt) · |v_rel|² -- the factor common to
   the lift and drag laws. Zero for any degenerate input. */
inline double dynamicPressure(const DragAtmosphere &a, double alt,
                              const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    if(rho <= 0.0) { return 0.0; }
    return 0.5 * rho * glm::dot(v_rel, v_rel);
}

/* The lift direction: the ship's up axis with its component along the flow
   removed -- perpendicular to v_rel, on the ship's "up" side. */
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

/* The lift coefficient CL(α) of a symmetric section, with a soft stall.
   Linear up to the stall angle, then a cosine droop to zero by 2·stallAngle.
   A = 0 disables the stall (pure linear). CL is continuous (no force jump). */
inline double liftCurve(double cl, double alpha, double stallAngle) {
    if(cl <= 0.0) { return 0.0; }
    const double a = std::fabs(alpha);
    const double A = stallAngle;
    double c;
    if(A <= 0.0 || a <= A) {
        c = cl * a;                                          // linear (up to stall)
    } else {
        const double t = (a - A) / A;                        // 0 at A, 1 at 2A
        c = (t >= 1.0) ? 0.0 : cl * A * std::cos(t * std::numbers::pi * 0.5);  // droop
    }
    return (alpha < 0.0) ? -c : c;                           // sign follows α
}

/* The lift force on one part: L = q · S · CL(α) · liftDir. */
inline glm::dvec3 liftForce(double q, double S, double cl, double alpha,
                            const glm::dvec3 &liftDir, double stallAngle = 0.0) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0) { return glm::dvec3(0.0); }
    return liftDir * (q * S * liftCurve(cl, alpha, stallAngle));
}

/* The 2-D cross product (z-component) of (b-a) x (c-a). */
inline double cross2(const glm::dvec2 &a, const glm::dvec2 &b,
                     const glm::dvec2 &c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/* The projected (silhouette) area of a body seen along `dir`, from the
   vertices of its CONVEX HULL. The silhouette of a convex body is exactly
   the convex hull of its projected vertices. Using the HULL's vertices (not
   the raw mesh's triangles) makes it exact for a NON-convex mesh too. The
   result is symmetric in dir. Pure math so tests/ can pin it without GL/Bullet. */
inline double projectedArea(const std::vector<glm::dvec3> &hullVerts,
                            const glm::dvec3 &dir) {
    const double len = glm::length(dir);
    if(len <= 0.0 || hullVerts.size() < 3) { return 0.0; }
    const glm::dvec3 d = dir / len;
    // A 2-D basis (u, w) of the plane perpendicular to d.
    const glm::dvec3 ref = (std::fabs(d.x) < 0.9) ? glm::dvec3(1, 0, 0)
                                                  : glm::dvec3(0, 1, 0);
    const glm::dvec3 u = glm::normalize(glm::cross(d, ref));
    const glm::dvec3 w = glm::cross(d, u);
    // Scratch buffers reused across calls (this runs every physics substep
    // per part -- fresh vectors here are pure churn).
    static thread_local std::vector<glm::dvec2> p;
    p.resize(hullVerts.size());
    for(size_t i = 0; i < hullVerts.size(); i++) {
        p[i] = glm::dvec2(glm::dot(hullVerts[i], u), glm::dot(hullVerts[i], w));
    }
    // Andrew's monotone chain: the convex hull of p (CCW, collinear dropped).
    // (glm::dvec2 has no operator<, so an explicit x-then-y comparator.)
    std::sort(p.begin(), p.end(), [](const glm::dvec2 &a, const glm::dvec2 &b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });
    static thread_local std::vector<glm::dvec2> h;
    h.clear();
    for(size_t i = 0; i < p.size(); i++) {
        while(h.size() >= 2
              && cross2(h[h.size() - 2], h[h.size() - 1], p[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(p[i]);
    }
    const size_t lowerSize = h.size() + 1;
    for(int i = (int)p.size() - 1; i >= 0; i--) {
        while(h.size() >= lowerSize
              && cross2(h[h.size() - 2], h[h.size() - 1], p[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(p[i]);
    }
    h.pop_back();                              // drop the duplicated start point
    if(h.size() < 3) { return 0.0; }           // degenerate (collinear) hull
    // Shoelace area of the hull polygon.
    double a = 0.0;
    for(size_t i = 0; i < h.size(); i++) {
        const size_t j = (i + 1) % h.size();
        a += h[i].x * h[j].y - h[j].x * h[i].y;
    }
    return 0.5 * std::fabs(a);
}

/* The per-part DRAG COEFFICIENT as the part faces the flow. A part declares
   one cd for each of three anchors (nose / side / base); this blends them by
   the angle between the part's nose axis and the flow:
     cd(c) = drag_side·(1−c²) + drag_forward·(max(c,0)²) + drag_backward·(max(−c,0)²)
   A convex blend: hits each anchor exactly at its angle, always within the
   part's own min..max cd. */
inline double partCd(double cdForward, double cdSide, double cdBackward,
                     double cosAxisFlow) {
    double c = cosAxisFlow;
    if(c > 1.0) { c = 1.0; }
    else if(c < -1.0) { c = -1.0; }
    const double c2 = c * c;
    const double cw = (c > 0.0) ? c2 : 0.0;   // forward weight (nose-in)
    const double bw = (c < 0.0) ? c2 : 0.0;   // backward weight (base-in)
    return cdSide * (1.0 - c2) + cdForward * cw + cdBackward * bw;
}

/* The force on one DEFLECTED control surface: the lift law with the
   DEFLECTION in place of the angle of attack (LINEAR -- no stall; a control
   surface is steered by how far it is deflected, bounded by its travel).
   The steering moment about the COM is NOT here: it comes from applying this
   force AT the surface's position (Vehicle::applyAeroForce). */
inline glm::dvec3 controlForce(double q, double S, double cl, double delta,
                               const glm::dvec3 &dir) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0 || delta == 0.0) { return glm::dvec3(0.0); }
    return dir * (q * S * cl * delta);
}

/* The control-surface DEFLECTION EFFECTIVENESS: the part's dedicated
   cl_control if set (>0), else its lift-curve slope cl (old behaviour). */
inline double controlCl(double cl, double clControl) {
    return (clControl > 0.0) ? clControl : cl;
}

/* The JET ENGINE (air-breathing) thrust in newtons:
     T(v, rho) = T_fan·d  +  ṁ_f·v_e·d  +  rho·A·v·(v_e − v)   d = min(rho/rho_sea, 1)
   d gates the fan + fuel terms on available air: in vacuum (rho = 0)
   EVERY term is 0. The net is clamped at 0 (a jet never reverses). */
inline double jetThrust(double v, double rho, double rho_sea,
                        double T_fan, double m_f, double v_e, double A) {
    if(rho <= 0.0 || rho_sea <= 0.0) { return 0.0; }   /* no air -> no thrust */
    const double d = (rho / rho_sea < 1.0) ? rho / rho_sea : 1.0;
    const double speed = (v < 0.0) ? 0.0 : v;
    const double T = T_fan * d + m_f * v_e * d + rho * A * speed * (v_e - speed);
    return (T < 0.0) ? 0.0 : T;
}

/* The control-surface DEFLECTION SIGN for a surface at position `ri` (rel.
   to the COM). The steering torque is ri x F, so a tail (behind the CG) and
   a canard (ahead) need OPPOSITE deflections for the same torque. This
   returns the sign that matches the reaction wheel's convention for EITHER
   position. `targetSign` is the SIGN of the wheel's torque about that axis
   for a positive stick (pitch/yaw = -1, roll = +1). */
inline double controlDeflectionSign(const glm::dvec3 &ri,
                                    const glm::dvec3 &forceDir,
                                    const glm::dvec3 &aboutAxis,
                                    double targetSign = -1.0) {
    const double d = glm::dot(glm::cross(ri, forceDir), aboutAxis);
    return (d >= 0.0) ? targetSign : -targetSign;
}
