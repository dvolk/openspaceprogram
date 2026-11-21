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
//
// RIDE HEIGHT: compression is measured from FULL EXTENSION (radius+travel),
// so the spring holds the contact face (travel - compression) ABOVE the ray
// hit -- the visual wheel/foot floats by that much, plus the support's
// collision margin. The float is the price of spring-only support: resting
// on Bullet contact instead would scrub and pump at coarse substeps. A part
// whose face must sit ON the ground needs a ride allowance in its mesh or a
// compression reference at the face, not a tweak here (#1072).
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
    /* kg/s: the hull's effective mass at this patch divided by the physics
       substep, over the number of ground contacts sharing that mass. 0 turns
       the clamp off. See force(). Defaulted because "off" is the direction a
       garbage value can land in, and a small positive one guts the brakes. */
    double stopRate = 0.0;
    /* Same budget along the strut normal, spent on the STRUT's closing rate
       (#862). 0 turns that clamp off. See force(). */
    double normalStopRate = 0.0;
};

// The force the ground applies to the hull at the contact patch (world N).
// Zero when uncompressed (the wheel is airborne).
inline glm::dvec3 force(const Input &in) {
    if(in.compression <= 0.0) { return glm::dvec3(0.0); }

    /* Suspension: push along the normal, never pull (a wheel cannot stick).
       The damper gets the same treatment as the tire pair (#862). It is an
       EXPLICIT force held for a whole substep, so damp*compressRate over-corrects
       the closing rate once the strut's own gain passes what one substep can
       spend: damp*h > m/n, i.e. h > m/(n*wheel_damp) = 658.65/9000 = 0.073 s for
       shipped gear under a 658 kg hull -- a hand-set --physics-hz 30. The strut
       then pumped: every substep reversed the closing rate, and a parked hull
       walked across the balloon deck at that substep while holding still at 60 Hz.
       The SPRING is deliberately not capped -- it is what holds the hull up, and
       its own explicit-stability limit, h < 2/sqrt(k/(m/n)) = 0.19 s here, sits
       outside the substeps the shipped loop can make. */
    double damper = in.damp * in.compressRate;
    if(in.normalStopRate > 0.0) {
        const double mom = in.normalStopRate * std::fabs(in.compressRate);
        if(std::fabs(damper) > mom) { damper = std::copysign(mom, damper); }
    }
    double N = in.spring * in.compression + damper;
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
    double cap = kGrip * N;
    /* Nor can a contact spend more than the slip momentum in front of it. Both
       tire terms are EXPLICIT forces, held for a whole substep, so a gain past
       mass/substep over-corrects the patch velocity and flips its sign. A braked
       hull has exactly that gain: the tanh knee makes the brake worth
       (kRollResist + kBrakeGrip)/kSlipEps = 2.7 N per N of load per m/s, and a
       parked load is m*g per wheel, so ask over one substep's budget is 2.7*g*h --
       no hull mass in it. At Venus g with h = 0.0556 s (warp 10, the coarsest
       substep the shipped loop makes -- warp 11+ rides rails) that is 1.32, i.e.
       over 1 from warp 8 up. The brake then reversed the slip every step, pumped
       the suspension (N 935 <-> 2885 N per wheel) and pitched the hull
       +/-0.22 rad/s, and the friction circle rectified that oscillation into a
       parked skiff walking 0.24 m/s across a balloon deck (#770; #862's
       tire-impulse idea). Stopping the slip is what the ground is asked to do
       there, so cap the pair at its share of the hull's momentum. What that cap
       makes of the tire is worth naming: at zero slip it is pure damping, so it no
       longer HOLDS a parked hull against a steady force. Whoever adds stiction on
       top has to raise the cap with max(stopRate*|v|, kStatic*N) or this line
       cancels it in silence. */
    if(in.stopRate > 0.0) {
        const double mom = in.stopRate * std::sqrt(vF * vF + vL * vL);
        if(mom < cap) { cap = mom; }
    }
    if(mag > cap && mag > 0.0) {
        const double s = cap / mag;
        fLat *= s;
        fLong *= s;
    }

    return n * N + l * fLat + f * fLong;
}

} // namespace wheel
