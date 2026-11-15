// buildingorbit.h -- the pure pose/attitude math for an orbital building
// (space dock / station). glm + orbit.h only: no Bullet, no Game, so tests can
// pin it without the physics chain (the wheel.h / gimbal.h pattern).
//
// An orbital building is immovable scenery: its pose is DRIVEN from its orbit
// each tick, never by forces. Everything here is a pure function of the sim
// clock `t` (phase0 + rate*t, never an accumulated phase) so a save/load or a
// warp hop reproduces the exact same pose -- the frame tree's own invariant.
#pragma once

#include <cassert>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "orbit.h"   // railStateFromElements, planeOrient, propagateKepler

namespace buildingorbit {

// The base attitude a building holds as it orbits. Spin (below) rides on top.
enum class Mode {
    SkyFixed,    // constant in the inertial frame (a telescope).
    BodyFixed,   // tracks the orbit, nadir-pointing (ISS / Tiangong).
};

/* Epoch state of a CIRCULAR orbit of `radius` (from the body centre), inclined
   `inc` with ascending node at `raan`, at true anomaly `phase` at t=0. This is
   authoring sugar: the result is a full (pos, vel) state the engine propagates
   with propagateKepler, so an eccentric orbit authored as full elements works
   through the same propagation. Angles in radians. Returns false on bad input. */
inline bool circularEpochState(double radius, double inc, double raan, double phase,
                               double mu, glm::dvec3 &pos, glm::dvec3 &vel) {
    glm::dvec3 p, v;
    if(!railStateFromElements(radius, 0.0, 0.0, phase, mu, p, v)) { return false; }
    const glm::dmat3 O = planeOrient(inc, raan);
    pos = O * p;
    vel = O * v;
    return true;
}

/* The body-fixed (LVLH) frame at an orbit state: +Z toward the body (nadir),
   +Y along the orbit normal, +X completing the right hand. A station authored
   with its "down" along +Z hangs nadir-pointing. */
inline glm::dmat3 lvlhOrient(const glm::dvec3 &pos, const glm::dvec3 &vel) {
    const glm::dvec3 rhat = glm::normalize(pos);
    const glm::dvec3 hhat = glm::normalize(glm::cross(pos, vel));
    const glm::dvec3 z = -rhat;                       // nadir
    const glm::dvec3 y = hhat - z * glm::dot(hhat, z); // normal, perp to nadir
    const glm::dvec3 x = glm::cross(y, z);
    return glm::dmat3(x, y, z);
}

/* The building's orientation at time `t`: the base mode, then a constant spin
   about a LOCAL axis on top. spinRate rad/s (0 = no spin); spinAxisLocal is a
   unit axis in the building's own frame. An on-axis docking port only rolls
   about its normal under this spin, so it stays dockable (match the rate). */
inline glm::dmat3 orientAt(Mode mode, const glm::dvec3 &pos, const glm::dvec3 &vel,
                           const glm::dvec3 &spinAxisLocal, double spinRate, double t) {
    glm::dmat3 base = (mode == Mode::BodyFixed)
                    ? lvlhOrient(pos, vel)
                    : glm::dmat3(1.0);
    if(spinRate != 0.0) {
        base = base * glm::dmat3(glm::rotate(glm::dmat4(1.0), spinRate * t, spinAxisLocal));
    }
    return base;
}

struct Pose { glm::dvec3 pos; glm::dmat3 orient; };

/* The full pose at time `t`, in the body's INERTIAL frame: propagate the epoch
   conic, then hold the chosen attitude. Pure f(t). */
inline Pose poseAt(const glm::dvec3 &epochPos, const glm::dvec3 &epochVel, double mu,
                   Mode mode, const glm::dvec3 &spinAxisLocal, double spinRate, double t) {
    glm::dvec3 p, v;
    propagateKepler(epochPos, epochVel, mu, t, p, v);
    return Pose{ p, orientAt(mode, p, v, spinAxisLocal, spinRate, t) };
}

/* The frame-node state (#850): pose plus the velocity, acceleration and own
   spin the frame tree composes. pos/vel/accel in the body's INERTIAL frame
   (the parent of an orbital building's node); omega_own in the building's
   LOCAL axes. Pure f(t), like poseAt. */
struct State {
    glm::dvec3 pos, vel, accel;
    glm::dmat3 orient;
    glm::dvec3 omega_own;
};

inline State stateAt(const glm::dvec3 &epochPos, const glm::dvec3 &epochVel, double mu,
                     Mode mode, const glm::dvec3 &spinAxisLocal, double spinRate, double t) {
    glm::dvec3 p, v;
    propagateKepler(epochPos, epochVel, mu, t, p, v);
    const glm::dmat3 O = orientAt(mode, p, v, spinAxisLocal, spinRate, t);
    const double r = glm::length(p);
    /* Kepler acceleration: the -A term built from this is what makes a ship
       at rest in the station's frame weightless. */
    const glm::dvec3 a = -(mu / (r * r * r)) * p;
    glm::dvec3 w(0.0);
    if(mode == Mode::BodyFixed) {
        /* LVLH tracks the orbit at the mean motion n about the orbit normal,
           which is +Y in the LVLH axes lvlhOrient builds. Constant in local
           axes ONLY for a circular orbit (eccentric theta-dot = h/r^2 varies
           -- the Euler term this law does not carry); parse refuses the
           combination, this asserts it. */
        assert(std::abs(glm::dot(v, v) * r - mu) < 1e-6 * mu
               && "body-fixed attitude needs a circular orbit (#850)");
        w += glm::dvec3(0.0, std::sqrt(mu / (r * r * r)), 0.0);
    }
    if(spinRate != 0.0) { w += spinRate * spinAxisLocal; }
    return State{ p, v, a, O, w };
}

} // namespace buildingorbit
