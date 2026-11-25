// wheel.h -- the landing-gear ground law, as pure math (glm + STL only: no
// Bullet, no Game), so tests can pin it without the physics chain. Same shape
// as gimbal.h / rcs.h / drag.h.
//
// A wheel is a FORCE APPLIER on the one-body hull, like a thruster or a lifting
// surface: each substep the caller (Vehicle::applyGroundForces) finds the
// contact, fills an Input, and applies force() at the contact patch. Three parts:
//   * a spring-damper along the ground normal (the suspension),
//   * a slip-based tire force in the ground plane (grip, rolling resistance,
//     brake),
//   * a static hold against the force the REST of the substep applied, while the
//     patch creeps (#1184),
// all of it inside one friction circle mu*N.
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
    /* N: the tangential force the REST of this substep has already handed the
       hull -- gravity's slope component, air, thrust -- as this contact's share
       of it. A contact answers it up to what its friction circle leaves, while
       its patch creeps: that is static friction, and without it the tire pair
       is pure damping, so a steady side load parks a hull at a permanent crawl
       (#1184). Zero means "nothing to hold", which is also what a caller on a
       body with no air and a level pad reads. Defaulted so the pins that predate
       it read unchanged. */
    glm::dvec3 push{0.0, 0.0, 0.0};
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
    const double slipMag = std::sqrt(vF * vF + vL * vL);
    double fLat  = -kLatStiff * vL;
    double fLong = -(kRollResist + in.brake * kBrakeGrip) * N
                   * std::tanh(vF / kSlipEps);

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
       tire-impulse idea). This budget bounds the two VELOCITY terms; the static
       hold below is not a gain on velocity, so it answers to the circle alone. */
    if(in.stopRate > 0.0) {
        const double mag = std::sqrt(fLat * fLat + fLong * fLong);
        const double mom = in.stopRate * slipMag;
        if(mag > mom && mag > 0.0) {
            const double s = mom / mag;
            fLat *= s;
            fLong *= s;
        }
    }

    /* Static friction (#1184): a surface holds a patch still, it does not merely
       damp it. Kerbin authors 5 m/s of surface wind, ~380 N of air on a parked
       2.2 t lander; against a tire pair proportional to SLIP that parks the hull
       at a permanent crawl of load/gain, and since the budget above makes the
       gain m/h, the crawl grows with the substep: 0.01 m/s at warp 1, 0.05 m/s
       at warp 10, which is 21 m of track and the edge of the pad in three
       minutes of sim. A balloon deck reads no wind at all (its bubble moves with
       the air), which is why the same hull sat still there and the bug looked
       pad-specific.
       The hold cancels the push the rest of THIS substep handed the hull, so it
       is exact rather than one step behind: the tick loop runs gravity, control
       and air, then this, and nothing integrates until physics_tick. Along the
       rolling direction only a BRAKED contact holds -- a free wheel is meant to
       roll -- while across it every contact holds, which is what a tyre does.
       Stiction only exists while the patch CREEPS, so the hold goes off past the
       kSlipEps band the tanh knee already uses. Without that gate the hold would
       answer a SLIDING contact too: a braked hull rolling out at 60 m/s is under
       kinetic friction, and cancelling this substep's air force there would
       remove the drag from its stopping distance -- and a throttled braked plane
       from its acceleration -- in silence. Above the band the law is exactly
       what it was before this block existed.
       The share is the linear push over the contacts that can hold, not the
       couple that push would actually produce at this patch: the leg springs
       carry the tipping, and the moments here are small (416 Nm on a hull whose
       leg pair restores at ~60 kNm/rad). */
    const glm::dvec3 pushT = in.push - n * glm::dot(in.push, n);
    double hLat = 0.0, hLong = 0.0;
    if(slipMag <= kSlipEps) {
        hLat = -glm::dot(pushT, l);
        if(in.brake > 0.0) { hLong = -glm::dot(pushT, f); }
    }

    /* Friction circle: over the SUM, because a contact cannot spend mu*N damping
       and mu*N holding on top of that. */
    const double mag = std::sqrt((fLat + hLat) * (fLat + hLat)
                                 + (fLong + hLong) * (fLong + hLong));
    if(mag > kGrip * N && mag > 0.0) {
        const double s = kGrip * N / mag;
        fLat *= s;
        fLong *= s;
        hLat *= s;
        hLong *= s;
    }

    return n * N + l * (fLat + hLat) + f * (fLong + hLong);
}

} // namespace wheel
