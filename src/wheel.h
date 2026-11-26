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

/* A strut is authored ONCE, against the airframe it was tuned with, and a hull a
   few times heavier out-rates it: the spring can only ever push spring*travel, so
   the wheel sinks onto its bump stops and the hull rests on BULLET COLLISION
   instead -- the scrub-and-pump regime this file's RIDE HEIGHT note refuses. That
   is not a heavier plane being unfriendly; it is the ground law stopping working,
   and it reads to the player as a plane glued to the runway (a 4.1 t spaceplane on
   the skiff's 1.15 t of strut: 9.2 kN of spring against 39.8 kN of weight).
   So past its designed ride the strut sizes itself to the load it carries. Under
   the designed ride nothing happens: a light hull keeps exactly the suspension it
   was authored with, bit for bit.
   The damper scales LINEARLY with k. Along this branch k rises with the hull mass,
   and the heave mode is omega = sqrt(g/ride) -- mass cancels -- so a damping ratio
   c/(2*sqrt(k*m)) that stays put needs c to rise with mass too, i.e. with k. The
   tempting sqrt(k) leaves a 4 t plane at zeta 0.26 against the skiff's 0.64: some
   40% overshoot and a couple of seconds of heave ring on every touchdown, which is
   exactly the hull this rule exists to rescue.
   What the rule does NOT buy: the share is an EVEN split, so a corner carrying
   more than travel/ride = 1.42x the mean is back on its bump stops whatever the
   sizing did. Measured worst corners are 1.23x (the skiff, e2e 1158-b) and 1.24x
   (the spaceplane), so the margin is ~15%, and it is load imbalance, not mass,
   that runs out first. Substep size does not move the static case: parked, the
   spaceplane reads the same 0.131 worst corner and 0.019 m of tyre-to-deck
   clearance at warp 1 as at warp 10's h = 0.0556 s. What coarse substeps do cost
   is DYNAMIC stroke -- a warp-10 touchdown arrives with more than the leftover
   0.019 m to spend and drops onto collision, which no static sizing can answer
   (#1216).
   kRideStroke is a measured datum, not a pin: the skiff's loaded main compresses
   0.106..0.109 of its 0.15, so an even share of 0.106 is just past where that hull
   engages at all (it engages at hull weight > 3*25000*0.106/g = 811 kg).
   kRideMaxFraction stops the rule sizing a strut for a stroke the part cannot
   reach -- a short-travel wheel would be parked on its own bump stop, stiffer.
   The DRAWN wheel still does not telescope, so ride height and landing stroke
   remain one quantity (#1166) -- this lifts the load ceiling, not the float. Both
   ground loops call it on ONE share: a hull's weight divides between every wheel
   and footpad it has, so standing on both does not get sized twice. */
inline constexpr double kRideStroke = 0.106;  // m of static compression a strut is sized for
inline constexpr double kRideMaxFraction = 0.75;   // of travel, the stroke a sized strut may use
inline void sizeToStrut(double staticShareN, double travel, double &spring, double &damp) {
    if(staticShareN <= 0.0 || spring <= 0.0 || travel <= 0.0) { return; }
    const double ride = std::min(kRideStroke, kRideMaxFraction * travel);
    const double need = staticShareN / ride;
    if(need <= spring) { return; }
    damp *= need / spring;
    spring = need;
}

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
    /* N: the force the REST of this substep has already handed the hull --
       gravity, air, thrust -- as this contact's share of it. Pass the whole
       vector; force() throws away the part along the normal, which is the
       suspension's business. A contact answers what is left, up to what its
       friction circle leaves, while its patch creeps: that is static friction,
       and without it the tire pair is pure damping, so a steady side load parks
       a hull at a permanent crawl (#1184). Zero means "nothing to hold", which
       is also what a caller on a body with no air and a level pad reads.
       Defaulted so the pins that predate it read unchanged.
       A DRIVE force must not go in here: the hold cancels a zero-slip load up to
       mu*N, so a rover's motor fed through this would be braked by its own
       wheels at rest. The longitudinal slot (top of this header) is where the
       wheel decides what it wants, distinct from what the hull was handed. */
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
       spend: damp*h > m/n, i.e. h > m/(n*wheel_damp) = 658.11/9000 = 0.073 s for
       shipped gear under a 658 kg hull (flight/1158-b pins that mass) -- a hand-set
       --physics-hz 30. The strut
       then pumped: every substep reversed the closing rate, and a parked hull
       walked across the balloon deck at that substep while holding still at 60 Hz.
       The SPRING is deliberately not capped -- it is what holds the hull up. Under
       the authored pair the heave limit h < 2/sqrt(k/(m/n)) = 0.19 s sits outside
       the substeps the shipped loop can make, and sizeToStrut does not move that
       margin much: along its branch omega pins at sqrt(g/ride) = 9.7 rad/s, so the
       limit is 0.21 s whatever the mass. What CAN run past it is the pitch mode
       that the gear struts also spring -- omega there is omega_heave x (gear arm /
       radius of gyration), so a hull with gear out at the ends and mass bunched in
       the middle reaches the h*omega = 2 limit nearer 0.1 s, inside what the loop
       makes (max h ~0.117 s by substepCount). Unmeasured so far; if it ever bites,
       the bound is the caller's own impulseMass(hull, rel, n) / (h*h). */
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
       damp it. Kerbin authors 5 m/s of surface wind, 386 N of air on a parked
       2.2 t lander; against a tire pair proportional to SLIP that parks the hull
       at a permanent crawl of load/gain, and it got worse the coarser the
       substep was -- measured with this block compiled out, racerlegs on the
       Kerbin pad: 0.20 m in 19 s at warp 1 (0.010 m/s), and at warp 10 it walked
       18 m in 191 s, which is off the pad edge and into a fall (|v| 0.95 m/s
       coming down).
       A balloon deck reads no wind at all (its bubble moves with the air), which
       is why the same hull sat still there and the bug looked pad-specific.
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
       What that buys, and it is tuning nobody has weighed yet: a legged hull can
       now stand on a slope up to atan(kGrip) = 42 deg, where before any slope at
       all slid it. It also means a parked hull takes mu*N before it budges --
       0.9 * 5.4 kN = 4.9 kN for one footpad of a 2.2 t lander on four legs -- so
       pushing one off a pad is a legs/staging action, not a gentle thrust one.
       The share is the linear push over the contacts that can hold, not the
       couple that push would actually produce at this patch: the tipping goes
       into the leg struts, not into stiction, and it is small -- a parked
       racerlegs reads 421 Nm of wind couple in --drag-log and leans 0.13 deg off
       its deck normal, where pads that bit the couple unevenly would keep it
       level (#1193). */
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
