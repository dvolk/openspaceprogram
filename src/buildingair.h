// buildingair.h -- the pure pose math for an AIR-RELATIVE building (a balloon
// platform riding the wind). glm + equirect.h only: no Bullet, no Game, so
// tests can pin it without the physics chain (the buildingorbit.h pattern).
//
// The platform holds its authored latitude and altitude and drifts east at the
// zonal wind sampled ONCE at the anchor. For a steady, purely zonal wind --
// Venus's authored superrotation -- that is the EXACT advection path, and like
// buildingorbit.h everything is a pure function of the sim clock t, so a
// save/load or a warp hop reproduces the pose. North/up wind and band-crossing
// are not advected in v1 (reports/air-buildings2026_11_14 §4.1).
#pragma once

#include <cmath>
#include <numbers>

#include <glm/glm.hpp>

#include "equirect.h"   // equirectDir/East/North, wrapLonDeg: the ONE lon/lat law

namespace buildingair {

/* Zonal drift rate [rad/s]: east wind [m/s] over the latitude circle's radius
   r cos(lat). The caller refuses |lat| > 85 deg at parse -- cos(lat) -> 0 at
   the pole is a singularity, not a place a platform drifts through. */
inline double driftRate(double east, double r, double lat_rad) {
    return east / (r * std::cos(lat_rad));
}

/* Longitude [deg] at time t, wrapped to [0, 360) by the one wrap law. */
inline double lonAt(double lon0_deg, double rate, double t) {
    constexpr double r2d = 180.0 / std::numbers::pi;
    return wrapLonDeg(lon0_deg + rate * t * r2d);
}

struct Pose { glm::dvec3 pos; glm::dmat3 orient; glm::dvec3 up; };

/* The full pose at time t in the body's ROTATING frame (where the wind is
   authored and an in-atmo ship integrates). r = seaLevelRadius + alt.

   Attitude is the CONTINUOUS local-horizontal basis [east | north | up] turned
   by heading about up -- deliberately NOT buildingOrient/faceAlong(up), whose
   discrete roll-axis pick snaps ~90 deg when a moving up crosses a pick
   boundary (vehicle.cpp:419; at lat 10 deg, 8 times per lap). Heading 0 puts
   the deck's +X east; heading pi puts it west, into the wind. */
inline Pose poseAt(double r, double lat_rad, double lon0_deg, double rate,
                   double heading_rad, double t) {
    const double lon = lonAt(lon0_deg, rate, t) * std::numbers::pi / 180.0;
    const glm::dvec3 up = equirectDir(lon, lat_rad);
    const glm::dvec3 east = equirectEast(up);
    const glm::dvec3 north = equirectNorth(up, east);
    const double c = std::cos(heading_rad), s = std::sin(heading_rad);
    // Spin about local +Z, same column spelling as buildingOrient.
    const glm::dmat3 spin(glm::dvec3(c, s, 0.0),
                          glm::dvec3(-s, c, 0.0),
                          glm::dvec3(0.0, 0.0, 1.0));
    return Pose{ up * r, glm::dmat3(east, north, up) * spin, up };
}

/* The frame-node state (#850): pose plus the velocity, acceleration and own
   spin the frame tree composes. pos/vel/accel in the body's ROTATING frame
   (the parent of an air building's node, where the wind is authored);
   omega_own in the building's LOCAL axes. Pure f(t), like poseAt. */
struct State {
    glm::dvec3 pos, vel, accel;
    glm::dmat3 orient;
    glm::dvec3 omega_own;
};

inline State stateAt(double r, double lat_rad, double lon0_deg, double rate,
                     double heading_rad, double t) {
    const Pose p = poseAt(r, lat_rad, lon0_deg, rate, heading_rad, t);
    /* Drift velocity: east at rate * (r cos lat) -- the same wind the pose
       law was sampled from, so ship and deck agree by construction. */
    const glm::dvec3 vel = equirectEast(p.up) * (rate * r * std::cos(lat_rad));
    /* Centripetal acceleration of the latitude circle, toward the planet
       axis (equirect +Y). pos minus its axis component IS the cylindrical
       radial, magnitude r cos lat -- no normalize needed. */
    const glm::dvec3 axis(0.0, 1.0, 0.0);
    const glm::dvec3 accel = -(rate * rate) * (p.pos - axis * glm::dot(p.pos, axis));
    /* Own spin relative to the rotating frame: the drift rate about the
       planet axis, in local axes. Constant there at fixed latitude -- the
       ENU basis carries the axis at a fixed angle, so no Euler term. */
    const glm::dvec3 omega_own = glm::transpose(p.orient) * (rate * axis);
    return State{ p.pos, vel, accel, p.orient, omega_own };
}

} // namespace buildingair
