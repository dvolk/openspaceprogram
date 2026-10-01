#pragma once
// Two-body orbital elements from a (pos, vel) state in the body's INERTIAL
// frame. Header-only pure math. Angles in radians; plane = XY (normal +Z).
// time_to_peri / time_to_apo: seconds to the NEXT passage, -1 = never.

#include <cmath>
#include <numbers>
#include <glm/glm.hpp>

struct OrbitElements {
    double distance = 0.0;      // m, radius from focus
    double speed = 0.0;         // m/s
    double semi_major = 0.0;    // m (negative for hyperbolic trajectories)
    double ecc = 0.0;           // eccentricity
    double periapsis = 0.0;     // m, radius at periapsis
    double apoapsis = 0.0;      // m, radius at apoapsis (-1 for non-elliptic)
    double inclination = 0.0;   // rad, from the +Z axis
    double period = 0.0;        // s (-1 for non-elliptic trajectories)
    double ang_momentum = 0.0;  // |h|, m^2/s
    double energy = 0.0;        // specific orbital energy, J/kg
    double radial_vel = 0.0;    // m/s, + = receding from the focus
    double raan = 0.0;          // rad [0, 2pi), ascending node (0 if equatorial)
    double arg_periapsis = 0.0; // rad [0, 2pi), (0 if circular or equatorial)
    double true_anomaly = 0.0;  // rad [0, 2pi) (0 if circular)
    double ecc_anomaly = 0.0;   // rad: eccentric anomaly E (elliptic) or
                                //      hyperbolic anomaly H (hyperbolic)
    double mean_anomaly = 0.0;  // rad: E - e sin E (elliptic) or
                                //      e sinh H - H (hyperbolic)
    double time_to_peri = 0.0;  // s until next periapsis (-1 = none)
    double time_to_apo = 0.0;   // s until next apoapsis (-1 = none)
};

inline double wrapAngleToPositive(const double theta) {
    return theta >= 0.0 ? theta : std::numbers::pi * 2 + theta;
}

inline glm::dvec3 projectVecOntoPlane(const glm::dvec3 &vec, const glm::dvec3 &normal) {
    return vec - glm::dot(vec, normal) * normal;
}

inline OrbitElements computeOrbitElements(const glm::dvec3 &pos, const glm::dvec3 &vel, double mu) {
    const double distance = glm::length(pos);
    const double speed = glm::length(vel);
    const glm::dvec3 h = glm::cross(pos, vel);
    const double h_len = glm::length(h);

    OrbitElements o;
    o.distance = distance;
    o.speed = speed;
    o.energy = 0.5 * speed * speed - mu / distance;
    o.semi_major = 1.0 / (2.0 / distance - speed * speed / mu);
    o.ang_momentum = h_len;
    const glm::dvec3 ecc_vec = glm::cross(vel, h) / mu - pos / distance;
    o.ecc = glm::length(ecc_vec);
    o.radial_vel = glm::dot(pos, vel) / distance;
    // Stays finite in the parabolic limit where (1-e)*a is 0 * inf.
    o.periapsis = h_len * h_len / (mu * (1.0 + o.ecc));
    o.inclination = h_len > 0.0 ? acos(glm::clamp(h.z / h_len, -1.0, 1.0)) : 0.0;

    // Undefined for equatorial/circular orbits; report 0, not NaN.
    const glm::dvec3 node = glm::cross(glm::dvec3(0.0, 0.0, 1.0), h);
    const double node_len = glm::length(node);
    o.raan = node_len > 0.0 ? wrapAngleToPositive(atan2(node.y, node.x)) : 0.0;
    o.arg_periapsis = 0.0;
    if(node_len > 0.0 && o.ecc > 1e-9) {
        const double c = glm::dot(node, ecc_vec) / (node_len * o.ecc);
        o.arg_periapsis = acos(glm::clamp(c, -1.0, 1.0));
        if(ecc_vec.z < 0.0) { o.arg_periapsis = std::numbers::pi * 2 - o.arg_periapsis; }
    }

    // atan2 of (e sin nu, e cos nu) picks the quadrant directly.
    o.true_anomaly = 0.0;
    if(o.ecc > 1e-9 && h_len > 0.0) {
        const double c = glm::dot(ecc_vec, pos) / (o.ecc * distance);
        const double s = o.radial_vel * h_len / (mu * o.ecc);
        o.true_anomaly = wrapAngleToPositive(atan2(s, c));
    }

    if(o.ecc < 1.0) {
        o.apoapsis = (1.0 + o.ecc) * o.semi_major;
        o.period = 2.0 * std::numbers::pi * sqrt(o.semi_major * o.semi_major * o.semi_major / mu);
        // atan2 form is quadrant-safe (no acos + branch flip).
        o.ecc_anomaly = wrapAngleToPositive(
            atan2(sqrt(1.0 - o.ecc * o.ecc) * sin(o.true_anomaly),
                  o.ecc + cos(o.true_anomaly)));
        o.mean_anomaly = o.ecc_anomaly - o.ecc * sin(o.ecc_anomaly);
        // Countdowns to the NEXT passage; at an apsis that is a full period, never 0.
        const double t_since_peri = (o.mean_anomaly / (2.0 * std::numbers::pi)) * o.period;
        o.time_to_peri = o.period - t_since_peri;
        o.time_to_apo = 0.5 * o.period - t_since_peri;
        if(o.time_to_apo <= 0.0) { o.time_to_apo += o.period; }
    } else if(o.ecc > 1.0) {
        // Hyperbolic: asinh is quadrant-safe; nu stays inside the asymptotes.
        o.apoapsis = -1.0;
        o.period = -1.0;
        const double sh = sqrt(o.ecc * o.ecc - 1.0) * sin(o.true_anomaly)
                        / (1.0 + o.ecc * cos(o.true_anomaly));
        o.ecc_anomaly = asinh(sh);
        o.mean_anomaly = o.ecc * sh - o.ecc_anomaly;   // e sinh H - H
        const double a_abs = -o.semi_major;
        const double t_from_peri = o.mean_anomaly * sqrt(a_abs * a_abs * a_abs / mu);
        // Wrapped nu > pi is inbound: periapsis still ahead. Outbound, gone forever.
        o.time_to_peri = (o.true_anomaly > std::numbers::pi) ? -t_from_peri : -1.0;
        o.time_to_apo = -1.0;
    } else {
        // Exactly parabolic (measure zero): no period, no passage timing.
        o.apoapsis = -1.0;
        o.period = -1.0;
        o.time_to_peri = -1.0;
        o.time_to_apo = -1.0;
    }
    return o;
}

// Stumpff C(z)/S(z): power series near z = 0, closed forms away from it.
inline double stumpffC(const double z) {
    if(z > 1e-6) { return (1.0 - cos(sqrt(z))) / z; }
    if(z < -1e-6) { return (cosh(sqrt(-z)) - 1.0) / (-z); }
    return 1.0 / 2.0 - z / 24.0 + z * z / 720.0;
}

inline double stumpffS(const double z) {
    if(z > 1e-6) { const double s = sqrt(z); return (s - sin(s)) / (s * s * s); }
    if(z < -1e-6) { const double s = sqrt(-z); return (sinh(s) - s) / (s * s * s); }
    return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0;
}

/* Propagate a two-body state by dt (may be negative) via universal variables.
   Exact for any dt -- what lets idle ships coast "on rails" at any time accel.
   pos0/vel0 taken BY VALUE so in-place calls (p, v out = p, v in) are safe. */
inline void propagateKepler(glm::dvec3 pos0, glm::dvec3 vel0,
                            const double mu, double dt,
                            glm::dvec3 &pos, glm::dvec3 &vel) {
    if(dt == 0.0) { pos = pos0; vel = vel0; return; }

    const double r0 = glm::length(pos0);
    const double v0_2 = glm::dot(vel0, vel0);
    const double vr0 = glm::dot(pos0, vel0) / r0;   // radial speed, + = receding
    const double alpha = 2.0 / r0 - v0_2 / mu;      // 1/a; negative on hyperbolic

    // Fold whole periods out of dt: the Newton solve must span at most one
    // revolution (it stalls on the multi-revolution equation).
    if(alpha > 0.0) {
        const double a = 1.0 / alpha;
        const double T = 2.0 * std::numbers::pi * sqrt(a * a * a / mu);
        dt = fmod(dt, T);
        if(dt == 0.0) { pos = pos0; vel = vel0; return; }
    }

    /* Newton on the universal Kepler equation (Bate, Mueller & White).
       Can miss near a zero of C(z): dF flattens, the step overshoots and
       the iteration oscillates. F is strictly increasing with a unique
       root, so the bisection fallback below always recovers it. */
    const double sqrt_mu = sqrt(mu);
    const double target = sqrt_mu * dt;
    const double k1 = r0 * vr0 / sqrt_mu;   // 0 at an apsis
    const double k2 = 1.0 - alpha * r0;     // >= 1 for any bound state
    auto F = [alpha, r0, k1, k2, target](double chi) {
        const double z = alpha * chi * chi;
        return k1 * chi * chi * stumpffC(z)
             + k2 * chi * chi * chi * stumpffS(z)
             + r0 * chi - target;
    };
    auto dF = [alpha, r0, k1, k2](double chi) {
        const double z = alpha * chi * chi;
        return r0 + k2 * chi * chi * stumpffC(z)
             + k1 * chi * (1.0 - z * stumpffS(z));
    };

    double chi = target / r0;               // small-dt limit: RHS ~ r0 chi
    for(int iter = 0; iter < 30; iter++) {
        const double step = F(chi) / dF(chi);
        chi -= step;
        if(fabs(step) < 1e-10 * r0) { break; }
    }
    if(fabs(F(chi)) > 1e-6 * target) {      // Newton missed (see above)
        // Expand the outer bracket end until the signs straddle the root.
        double lo, hi;
        if(target > 0.0) {
            lo = 0.0; hi = target / r0;
            for(int i = 0; i < 100 && F(hi) < 0.0; i++) { hi *= 2.0; }
        } else {
            hi = 0.0; lo = target / r0;
            for(int i = 0; i < 100 && F(lo) > 0.0; i++) { lo *= 2.0; }
        }
        for(int i = 0; i < 70; i++) {       // 2^-70 << double epsilon
            const double mid = 0.5 * (lo + hi);
            if(F(mid) < 0.0) { lo = mid; } else { hi = mid; }
        }
        chi = 0.5 * (lo + hi);
    }

    const double z = alpha * chi * chi;
    const double chi2 = chi * chi;
    const double f = 1.0 - (chi2 / r0) * stumpffC(z);
    const double g = dt - (chi2 * chi / sqrt_mu) * stumpffS(z);
    pos = f * pos0 + g * vel0;
    const double r = glm::length(pos);
    const double fdot = (sqrt_mu / (r * r0)) * chi * (z * stumpffS(z) - 1.0);
    const double gdot = 1.0 - (chi2 / r) * stumpffC(z);
    vel = fdot * pos0 + gdot * vel0;
}

/* Epoch state from elements, in the BODY-RAIL convention: orbital plane = XZ
   (normal +Y). Plane tilt is NOT applied here -- the frame's orient carries it.
   a > 0, 0 <= e < 1 (bodies don't escape). Returns false on bad input. */
inline bool railStateFromElements(double a, double e,
                                  double arg_peri, double true_anomaly,
                                  double mu,
                                  glm::dvec3 &pos, glm::dvec3 &vel) {
    if(!(a > 0.0) || !(mu > 0.0) || !(e >= 0.0 && e < 1.0)) { return false; }

    const double p = a * (1.0 - e * e);             // semi-latus rectum
    const double r = p / (1.0 + e * cos(true_anomaly));
    const double phi = arg_peri + true_anomaly;     // in-plane position angle
    const glm::dvec3 rhat(cos(phi), 0.0, sin(phi));
    pos = r * rhat;

    // Transverse component is h/r, NOT the total vis-viva speed (that
    // over-counts whenever the radial part is nonzero).
    const double s = sqrt(mu / p);
    const double vr = s * e * sin(true_anomaly);
    const double vt = s * (1.0 + e * cos(true_anomaly));
    vel = vr * rhat + vt * glm::cross(glm::dvec3(0.0, 1.0, 0.0), rhat);
    return true;
}
