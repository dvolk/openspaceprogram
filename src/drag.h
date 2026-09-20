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

#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

/* The aerodynamic flow frame: the ship's air-relative velocity decomposed
   against its body axes (x right, y up, z nose) plus the derived angles the
   force laws consume. `right`, `up`, `nose` (the ship's body axes) and
   `v_rel` must be in the SAME frame (world). Pure math (glm + <cmath>) so
   tests/ can pin it without Bullet/GL. Phase 2 (lift) reads alpha off this. */
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

/* The lift coefficient CL(α) of a symmetric section, with a soft stall
   (Phase 3). The curve is linear up to the stall angle, then the flow
   separates and the lift collapses:
     |α| <= A        CL = cl · α                           (linear)
     A < |α| < 2·A   CL = cl · A · cos( (π/2)·(|α|−A)/A )  (soft droop)
     |α| >= 2·A      CL = 0                                (deep stall)
   where A = stall_angle (rad). The peak is cl·A, reached at |α| = A; a real
   wing loses lift beyond it (this is why a stalled craft sinks instead of
   holding altitude), and a fully sideways wing (|α| ≈ 90°) is edge-on and
   generates none. A = 0 disables the stall -- pure linear, exactly the
   Phase 2 law -- so a part with no stall_angle is unchanged. Symmetric in
   |α| (a symmetric section stalls up and down the same); the sign of CL
   follows α. CL is continuous in α (no force jump), so the CoP torque
   carries no impulse. */
inline double liftCurve(double cl, double alpha, double stallAngle) {
    if(cl <= 0.0) { return 0.0; }
    const double a = std::fabs(alpha);
    const double A = stallAngle;
    double c;
    if(A <= 0.0 || a <= A) {
        c = cl * a;                                          // linear (up to stall)
    } else {
        const double t = (a - A) / A;                        // 0 at A, 1 at 2A
        c = (t >= 1.0) ? 0.0 : cl * A * std::cos(t * M_PI * 0.5);  // droop
    }
    return (alpha < 0.0) ? -c : c;                           // sign follows α
}

/* The lift force on one part:
     L = q · S · CL(α) · liftDir
   q = dynamic pressure (0.5·rho·v²), S = the part's lift_area [m²], and
   CL(α) = liftCurve(cl, α, stallAngle). A symmetric section (CL0 = 0) has
   no lift at zero AoA; the sign of the force follows α (negative AoA pushes
   down), and the soft stall collapses the lift past the stall angle.
   stallAngle = 0 keeps the Phase 2 linear law. Zero for any degenerate
   input. */
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
   vertices of its CONVEX HULL (part-local frame). The silhouette of a convex
   body is exactly the convex hull of its projected vertices, so this is the
   area of that 2-D hull:
     A_proj(dir) = area(conv({ (v.u, v.w) : v in hullVerts }))
   where (u, w) is a 2-D basis of the plane perpendicular to dir. Computed
   with Andrew's monotone chain (O(n log n)) + the shoelace area.
   Using the HULL's vertices -- not the raw mesh's triangles -- is what makes
   it exact for a NON-convex mesh too (the engine is a hollow tube with a
   nozzle: its triangle sum is the wrong silhouette). The hull is convex by
   construction and is the SAME shape the collision uses (BuildPartHull), so
   the drag area and the collision silhouette agree by construction. The
   result is symmetric in dir (a convex body's silhouette is the same seen
   from either side), so the sign of `dir` is irrelevant. `dir` need not be
   unit -- it is normalized here; a zero dir or fewer than 3 (non-collinear)
   vertices gives 0. Pure math (glm + <algorithm>) so tests/ can pin it
   without GL/Bullet. */
inline double projectedArea(const std::vector<glm::dvec3> &hullVerts,
                            const glm::dvec3 &dir) {
    const double len = glm::length(dir);
    if(len <= 0.0 || hullVerts.size() < 3) { return 0.0; }
    const glm::dvec3 d = dir / len;
    // A 2-D basis (u, w) of the plane perpendicular to d (ref chosen off d
    // so cross(d, ref) is not degenerate).
    const glm::dvec3 ref = (std::fabs(d.x) < 0.9) ? glm::dvec3(1, 0, 0)
                                                  : glm::dvec3(0, 1, 0);
    const glm::dvec3 u = glm::normalize(glm::cross(d, ref));
    const glm::dvec3 w = glm::cross(d, u);
    // Project the hull vertices onto (u, w).
    std::vector<glm::dvec2> p(hullVerts.size());
    for(size_t i = 0; i < hullVerts.size(); i++) {
        p[i] = glm::dvec2(glm::dot(hullVerts[i], u), glm::dot(hullVerts[i], w));
    }
    // Andrew's monotone chain: the convex hull of p (CCW, collinear dropped).
    // (glm::dvec2 has no operator<, so an explicit x-then-y comparator.)
    std::sort(p.begin(), p.end(), [](const glm::dvec2 &a, const glm::dvec2 &b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });
    std::vector<glm::dvec2> h;
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

/* The per-part DRAG COEFFICIENT as the part faces the flow -- the
   "bluntness" that, times the silhouette area, gives the drag. A part
   declares one cd for each of the three ways it can present to the flow:
     drag_forward   its NOSE (the +stack axis) into the flow   (c = +1)
     drag_side      its SIDE, axis perpendicular to the flow   (c = 0)
     drag_backward  its BASE into the flow                     (c = -1)
   and this blends them by the angle between the part's nose axis and the
   flow. c = cos of that angle (the dot of the unit nose axis and the unit
   flow), so:
     cd(c) = drag_side·(1−c²) + drag_forward·(max(c,0)²)
            + drag_backward·(max(−c,0)²)
   The three weights (1−c², max(c,0)², max(−c,0)²) are non-negative and sum
   to 1, so cd is a CONVEX blend: it hits each anchor exactly at its angle
   (nose / broadside / base) and eases between them, always staying within
   the part's own min..max cd. A symmetric part (all three equal, or just
   the shared `drag` set) returns that value for every angle. Pure math
   (no glm) so tests/ can pin it without GL/Bullet. */
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

/* The force on one DEFLECTED control surface (an elevator / rudder /
   aileron): the lift law with the DEFLECTION in place of the angle of
   attack --
     F = q · S · cl · delta
   where S is the control area, cl is the deflection effectiveness (per
   radian, the lift-curve slope of the surface), and delta is the deflection
   (rad, driven by the player's control input and bounded by the surface's
   max_deflection). It is LINEAR (no stall) -- a control surface is steered
   by how far it is deflected, not by an AoA limit, and its deflection is
   bounded by its travel (max_deflection), not by flow separation. The sign
   follows delta (deflected one way pushes the ship one way, the other way
   the opposite -- symmetric, like a symmetric section).

   The steering moment about the COM is NOT here: it comes from applying
   this force AT the surface's position (see Vehicle::applyAeroForce), so a
   surface's leverage is its distance from the COM -- a tail behind the CG
   pitches/yaws the ship, a canard ahead pitches it the other way. Zero for
   any degenerate input (no air, no speed, no area, no deflection). */
inline glm::dvec3 controlForce(double q, double S, double cl, double delta,
                               const glm::dvec3 &dir) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0 || delta == 0.0) { return glm::dvec3(0.0); }
    return dir * (q * S * cl * delta);
}

/* The control-surface DEFLECTION EFFECTIVENESS: the part's dedicated
   cl_control if it is set (>0), else its lift-curve slope cl (so a part
   that only declares cl keeps working -- the old behaviour). This
   un-overloads the two historically-shared uses of "cl": the lift-curve
   slope (with the soft stall) and the deflection effectiveness (linear,
   bounded by the travel). A part that is both a wing and a control surface
   can now give each its own number. Zero when neither is set. */
inline double controlCl(double cl, double clControl) {
    return (clControl > 0.0) ? clControl : cl;
}

/* The JET ENGINE (air-breathing) thrust in newtons, from the momentum
   balance of an engine that burns fuel against FREE air (no onboard
   oxidizer). At airspeed v through air of density rho:

     T(v, rho) = T_fan·d  +  ṁ_f·v_e·d  +  rho·A·v·(v_e − v)     d = min(rho/rho_sea, 1)
                └ static ┘  └ fuel mom ┘  └ ram / air momentum ┘

   T_fan   static (fan) thrust at sea level [N] -- the VTOL floor: a
           stationary jet still pushes, so a plane can take off vertically
           (no runways/wheels yet). A "true turbojet" would set this to 0.
   ṁ_f     fuel mass flow at full throttle [kg/s] (the H2 draw; air is free).
           The fuel-momentum term ṁ_f·v_e is small (the fuel is a fraction
           of the exhaust mass).
   v_e     the REAL exhaust velocity [m/s] (~500-600 for a turbofan core,
           not a rocket's 4000+). NOT a thrust-encoding knob.
   A       effective intake/capture area [m^2]: how much air the engine
           ingests per unit speed.
   rho·A·v·(v_e − v)  the AIR momentum: zero at rest, peaks near v = v_e/2,
           falls to 0 at v = v_e, and would go negative (the engine drags)
           beyond. The net is clamped at 0, so a jet never reverses.
   d = min(rho/rho_sea, 1) gates the fan + fuel terms on available air, so
   in vacuum (rho = 0) EVERY term is 0: a jet cannot thrust (or burn) in
   space. rho_sea is the body's sea-level density, so a thin-atmosphere
   body's jets stay proportional to that body's own air.
   Zero (or clamped to 0) for any degenerate input. Pure math (no glm) so
   tests/ can pin it without Bullet/GL. */
inline double jetThrust(double v, double rho, double rho_sea,
                        double T_fan, double m_f, double v_e, double A) {
    if(rho <= 0.0 || rho_sea <= 0.0) { return 0.0; }   /* no air -> no thrust */
    const double d = (rho / rho_sea < 1.0) ? rho / rho_sea : 1.0;
    const double speed = (v < 0.0) ? 0.0 : v;
    const double T = T_fan * d + m_f * v_e * d + rho * A * speed * (v_e - speed);
    return (T < 0.0) ? 0.0 : T;
}

/* The control-surface DEFLECTION SIGN for a surface at position `ri` (rel.
   to the COM). The steering torque is ri x F, so a tail (behind the CG)
   and a canard (ahead of it) need OPPOSITE deflections for the same
   steering torque -- a fixed force direction is position-dependent. This
   returns the sign (-1 or +1) that, multiplied by the stick input, makes
   the torque about `aboutAxis` match the reaction wheel's convention for
   EITHER a tail or a canard:
     sign = targetSign * ( dot(cross(ri, forceDir), aboutAxis) >= 0 ? +1 : -1 )
   `forceDir` is the force direction and `aboutAxis` the axis the torque
   should be about. `targetSign` (+1 or -1) is the SIGN of the wheel's
   torque about that axis for a positive stick -- pitch (W/S) is -right and
   yaw (A/D) is -up, but roll (Q/E) is +nose, so roll passes +1 and
   pitch/yaw pass -1 (the default). A surface exactly at the COM (dot 0)
   has zero lever -> no steering moment, so its sign is moot (returns
   targetSign). Pure math (glm) so tests/ can pin it without Bullet/GL. */
inline double controlDeflectionSign(const glm::dvec3 &ri,
                                    const glm::dvec3 &forceDir,
                                    const glm::dvec3 &aboutAxis,
                                    double targetSign = -1.0) {
    const double d = glm::dot(glm::cross(ri, forceDir), aboutAxis);
    return (d >= 0.0) ? targetSign : -targetSign;
}
