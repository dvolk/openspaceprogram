// wheel.h -- the landing-gear ground law, as pure math (glm + STL only: no
// Bullet, no Game), so tests can pin it without the physics chain. Same shape
// as gimbal.h / rcs.h / drag.h.
//
// A wheel is a FORCE APPLIER on the one-body hull, like a thruster or a lifting
// surface: each substep the caller (Vehicle::applyGroundForces) finds the
// contact, fills an Input, and applies force() at the contact patch. Two parts:
//   * a spring-damper along the ground normal (the suspension), and
//   * a slip-based tire force in the ground plane (grip, rolling resistance,
//     brake), clamped to the friction circle mu*N.
// There is no drive force in v1 -- the longitudinal slot is where a rover's
// motor lands later (DESIGN.md "ground vehicles").
#pragma once

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>

namespace wheel {

// Tire constants -- one spelling, tuned here.
inline constexpr double kGrip       = 0.9;    // friction coefficient mu
inline constexpr double kRollResist = 0.02;   // rolling-resistance coefficient
inline constexpr double kLatStiff   = 4000.0; // N per (m/s) of lateral slip
inline constexpr double kBrakeGrip  = 0.8;    // brake friction coefficient
inline constexpr double kSlipEps    = 0.3;    // m/s, tanh knee (viscous near 0)

// Everything force() needs about one wheel this substep. The caller computes
// the contact (ray + analytic fall-through) and the patch velocity; this header
// only turns those into a force.
struct Input {
    double compression;      // m, (radius+travel) - rayDistance, clamped [0,travel]
    double compressRate;     // m/s, d(compression)/dt (positive = closing)
    double spring;           // N/m
    double damp;             // N s/m
    glm::dvec3 normal;       // unit ground normal
    glm::dvec3 forward;      // unit rolling direction, in the ground plane (steered)
    glm::dvec3 contactVel;   // hull velocity at the patch (world)
    double brake;            // 0..1 brake demand
};

// The force the ground applies to the hull at the contact patch (world N).
// Zero when uncompressed (the wheel is airborne).
inline glm::dvec3 force(const Input &in) {
    if(in.compression <= 0.0) { return glm::dvec3(0.0); }

    // Suspension: push along the normal, never pull (a wheel cannot stick).
    double N = in.spring * in.compression + in.damp * in.compressRate;
    if(N < 0.0) { N = 0.0; }

    // Tire: split the patch velocity into rolling / lateral components.
    const glm::dvec3 n = in.normal;
    const glm::dvec3 f = in.forward;
    const glm::dvec3 l = glm::cross(n, f);
    const double vF = glm::dot(in.contactVel, f);
    const double vL = glm::dot(in.contactVel, l);
    double fLat  = -kLatStiff * vL;
    double fLong = -(kRollResist + in.brake * kBrakeGrip) * N
                   * std::tanh(vF / kSlipEps);

    // Friction circle: the tangential pair together cannot exceed mu*N.
    const double mag = std::sqrt(fLat * fLat + fLong * fLong);
    const double cap = kGrip * N;
    if(mag > cap && mag > 0.0) {
        const double s = cap / mag;
        fLat *= s;
        fLong *= s;
    }

    return n * N + l * fLat + f * fLong;
}

} // namespace wheel
