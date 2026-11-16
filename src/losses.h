// losses.h -- what a burn SPENT, as opposed to what it did to the orbit (#675).
//
// A loss is an integral over a burn, so it is not a tick reading: it cannot be
// recomputed at each reader, which is how three numbers stop adding up to the
// delta-v the tanks paid for. So the arithmetic lives here, once, and the physics
// pass calls it with the forces it actually handed the integrator.
//
// With v_hat the direction of the hull's ORBITAL velocity (the conic's, not the
// ground's -- what a burn buys is measured against the conic, as --dv-log's
// dvAlong_ already does):
//   dv    =  |F_t| h / m
//   steer = (|F_t| - F_t . v_hat) h / m      pointing off the velocity line
//   grav  = -(F_g . v_hat) h / m             holding up against the body
//   drag  = -(F_a . v_hat) h / m             what the air took along that line
// and term by term
//   dv - steer - grav - drag = (F_t + F_g + F_a) . v_hat h / m,
// which is the change in SPEED that substep bought. Sum both sides across a flight
// and the gap is the integrator's discretisation and nothing else -- the pin
// tests/test_guidance.cpp puts on it.
//
// Three deviations from the recorder this is modelled on (MechJeb's
// MechJebModuleFlightRecorder.cs:293-297, mapped in #675):
//   * Its gravity and drag totals run whether or not an engine is lit, so a long
//     coast charges gravity for nothing. These are BURN-GATED: a loss is the price
//     of a burn, and a flight with no burn has no losses to plot.
//   * Its drag loss is the MAGNITUDE of the drag acceleration. Ours is that force
//     projected on v_hat, which is what makes the four numbers close: a lifting
//     hull's air force is not anti-parallel to its velocity, and the remainder
//     belongs in the orbit, not in a loss.
//   * Its v_hat is an unguarded normalize, which at zero speed hands the whole
//     thrust to steering loss. These have a floor, below which the substep charges
//     its impulse and nothing else.

#pragma once

#include <cmath>
#include <glm/glm.hpp>

#include "orbit.h"   // conicSpeedOk: the same floor the conic readouts already ask

struct AscentLosses {
    double dv = 0.0;      // m/s of thrust impulse the tanks paid for
    double grav = 0.0;    // m/s spent holding up
    double drag = 0.0;    // m/s the air took along the velocity line
    double steer = 0.0;   // m/s thrown away by pointing off the velocity line
    double burn_s = 0.0;  // s of burning these totals cover
    /* The per-burn identity: which burn is being flown now, counting up from 0.
       The totals themselves are cumulative since the hull existed -- a sawtooth
       loss curve is not a flight profile -- and this is what lets a reader split
       them by differencing across a boundary, the way [rec]'s stg= chapters its
       timeline. */
    int burns = 0;
    bool lit = false;     // the last substep had thrust (the rising edge's memory)

    /* True when a substep moved anything. */
    bool burning() const { return lit; }
};

/* One substep of one hull. `m` is the mass the integrator is accelerating and `h`
   its duration; the three forces are what the physics pass applied, in any one
   frame -- they are all dotted with the same v_hat, so a shared rotation of the
   whole set cancels. A rail hop is not a substep: a railed hull is not stepped, so
   it is not billed (the known rails gap, same as drag). */
inline void lossStep(AscentLosses &L, const glm::dvec3 &vel, double m, double h,
                     const glm::dvec3 &fThrust, const glm::dvec3 &fGrav,
                     const glm::dvec3 &fAero) {
    const double fMag = glm::length(fThrust);
    if(!(m > 0.0) || !(h > 0.0)) { return; }
    if(!(fMag > 0.0)) { L.lit = false; return; }   // coasting: nothing to charge
    if(!L.lit) { L.lit = true; L.burns++; }

    const double speed = glm::length(vel);
    const double inv_m_h = h / m;
    L.dv += fMag * inv_m_h;
    L.burn_s += h;
    /* Below a floor there is no velocity line to score against, so the substep
       charges its impulse and nothing else. On a body that turns this is not the
       pad case: a parked hull already has the ground's own speed, and a vertical
       launch really does throw that away -- which is why the split says so rather
       than hiding it. The floor is orbit.h's conicSpeedOk, measured against the
       contact integrator's residue (3..7e-5 m/s parked on a non-spinning body),
       which is exactly the noise that would otherwise decide the sign here. */
    if(!conicSpeedOk(speed)) { return; }
    const glm::dvec3 vhat = vel * (1.0 / speed);
    L.steer += (fMag - glm::dot(fThrust, vhat)) * inv_m_h;
    L.grav += -glm::dot(fGrav, vhat) * inv_m_h;
    L.drag += -glm::dot(fAero, vhat) * inv_m_h;
}
