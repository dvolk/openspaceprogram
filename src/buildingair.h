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

} // namespace buildingair
