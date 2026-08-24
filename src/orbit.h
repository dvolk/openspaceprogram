#pragma once
// Two-body orbital elements of a (pos, vel) state relative to a central
// body with gravitational parameter mu. pos/vel must be in the body's
// INERTIAL (non-rotating) frame, where the trajectory is a Kepler conic.
//
// Header-only pure math (like calendar.h) so tests/test_orbit.cpp can pin
// it without rendering/Bullet. Consumers: the ORBITAL HUD window, the
// orbital map, and the --orbit-log periodic printout.
//
// Conventions:
// - Angles in radians. The orbital reference plane is the frame's XY plane
//   (normal +Z); the reference direction is +X (matches the spawn code,
//   which puts periapsis along +Z and inclines about X).
// - time_to_peri / time_to_apo are seconds until the NEXT passage. -1 =
//   the event never happens: a hyperbolic trajectory has no apoapsis, and
//   once it has swung past periapsis there is no future periapsis either.

#include <cmath>
#include <glm/glm.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    return theta >= 0.0 ? theta : M_PI * 2 + theta;
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
    /* h^2 / (mu (1+e)) == (1-e) a on an ellipse, but stays finite in the
       parabolic limit where (1-e) a is 0 * inf. */
    o.periapsis = h_len * h_len / (mu * (1.0 + o.ecc));
    o.inclination = h_len > 0.0 ? acos(glm::clamp(h.z / h_len, -1.0, 1.0)) : 0.0;

    /* The ascending node and argument of periapsis are undefined for
       equatorial (|n| ~ 0) and circular (e ~ 0) orbits; report 0 instead
       of NaN. */
    const glm::dvec3 node = glm::cross(glm::dvec3(0.0, 0.0, 1.0), h);
    const double node_len = glm::length(node);
    o.raan = node_len > 0.0 ? wrapAngleToPositive(atan2(node.y, node.x)) : 0.0;
    o.arg_periapsis = 0.0;
    if(node_len > 0.0 && o.ecc > 1e-9) {
        const double c = glm::dot(node, ecc_vec) / (node_len * o.ecc);
        o.arg_periapsis = acos(glm::clamp(c, -1.0, 1.0));
        if(ecc_vec.z < 0.0) { o.arg_periapsis = M_PI * 2 - o.arg_periapsis; }
    }

    /* True anomaly from (cos, sin): cos from the eccentricity vector, sin
       from the radial velocity (r_dot = (mu/h) e sin nu). atan2 fixes the
       quadrant directly. */
    o.true_anomaly = 0.0;
    if(o.ecc > 1e-9 && h_len > 0.0) {
        const double c = glm::dot(ecc_vec, pos) / (o.ecc * distance);
        const double s = o.radial_vel * h_len / (mu * o.ecc);
        o.true_anomaly = wrapAngleToPositive(atan2(s, c));
    }

    if(o.ecc < 1.0) {
        /* Elliptic: closed orbit. */
        o.apoapsis = (1.0 + o.ecc) * o.semi_major;
        o.period = 2.0 * M_PI * sqrt(o.semi_major * o.semi_major * o.semi_major / mu);
        /* E = atan2(sqrt(1-e^2) sin nu, e + cos nu) is quadrant-safe, so
           no acos + branch flip. */
        o.ecc_anomaly = wrapAngleToPositive(
            atan2(sqrt(1.0 - o.ecc * o.ecc) * sin(o.true_anomaly),
                  o.ecc + cos(o.true_anomaly)));
        o.mean_anomaly = o.ecc_anomaly - o.ecc * sin(o.ecc_anomaly);
        /* Time since periapsis, then the countdowns to the next passage of
           each apsis. At an apsis the countdown reports the full period to
           the NEXT return of that apsis (never 0). */
        const double t_since_peri = (o.mean_anomaly / (2.0 * M_PI)) * o.period;
        o.time_to_peri = o.period - t_since_peri;
        o.time_to_apo = 0.5 * o.period - t_since_peri;
        if(o.time_to_apo <= 0.0) { o.time_to_apo += o.period; }
    } else if(o.ecc > 1.0) {
        /* Hyperbolic: one periapsis passage, no apoapsis.
           sinh H = sqrt(e^2-1) sin nu / (1 + e cos nu); asinh is
           quadrant-safe. nu stays inside the asymptote angle, so
           1 + e cos nu > 0 always. */
        o.apoapsis = -1.0;
        o.period = -1.0;
        const double sh = sqrt(o.ecc * o.ecc - 1.0) * sin(o.true_anomaly)
                        / (1.0 + o.ecc * cos(o.true_anomaly));
        o.ecc_anomaly = asinh(sh);
        o.mean_anomaly = o.ecc * sh - o.ecc_anomaly;   // e sinh H - H
        const double a_abs = -o.semi_major;
        const double t_from_peri = o.mean_anomaly * sqrt(a_abs * a_abs * a_abs / mu);
        /* nu > pi (wrapped) is the inbound leg: radial velocity < 0, so
           periapsis is still ahead. Outbound, it is gone forever. */
        o.time_to_peri = (o.true_anomaly > M_PI) ? -t_from_peri : -1.0;
        o.time_to_apo = -1.0;
    } else {
        /* Exactly parabolic (measure zero in practice): one periapsis, no
           period. Don't attempt passage timing. */
        o.apoapsis = -1.0;
        o.period = -1.0;
        o.time_to_peri = -1.0;
        o.time_to_apo = -1.0;
    }
    return o;
}
