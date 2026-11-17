// test_buildingair -- the pure air-relative building pose math
// (src/buildingair.h). No Bullet, no GL: these pins keep the wind-riding pose
// an honest pure function of the clock, on the CONTINUOUS ENU basis (the
// faceAlong pick snap is the bug this law exists to avoid). Parse/placement
// wiring lives in buildings.cpp / terrain.cpp and is covered by test_buildings
// + the e2e.
//
// Build & run (from repo root): make test_buildingair   (or: make test)

#include <cmath>
#include <cstdio>
#include <numbers>

#include <glm/glm.hpp>

#include "buildingair.h"
#include "tchecks.h"

int main() {
    using namespace buildingair;
    const double d2r = std::numbers::pi / 180.0;
    // Venus cloud platform: 56 km up, lat 10 deg, 86.29 m/s east wind
    // (tmp/venus_air_probe.cpp, the game's own atmosphere law).
    const double r = 6051800.0 + 56000.0;
    const double lat = 10.0 * d2r;
    const double east = 86.29;

    printf("== driftRate ==\n");
    const double rate = driftRate(east, r, lat);
    CHECK_NEAR(rate, east / (r * std::cos(lat)), 1e-15, "rate = east / (r cos lat)");
    // ~2.96 deg/hr at lat 10 (the probe's number).
    CHECK_NEAR(rate * 3600.0 / d2r, 2.959, 0.002, "deg/hr matches the probe");
    CHECK_NEAR(driftRate(0.0, r, lat), 0.0, 1e-15, "no wind, no drift");

    printf("== lonAt ==\n");
    const double degPerSec = d2r;   // 1 deg/s
    CHECK_NEAR(lonAt(350.0, degPerSec, 20.0), 10.0, 1e-9, "wraps past 360");
    CHECK_NEAR(lonAt(10.0, degPerSec, -20.0), 350.0, 1e-9, "wraps below 0");
    CHECK_NEAR(lonAt(0.0, degPerSec, 0.0), 0.0, 1e-12, "t=0 is the anchor");
    CHECK_TRUE(lonAt(0.0, rate, 10.0) > 0.0, "east wind drifts lon eastward");

    printf("== poseAt: pure f(t), holds lat/alt ==\n");
    const double heading = 180.0 * d2r;   // deck +X west, into the wind
    Pose a = poseAt(r, lat, 0.0, rate, heading, 0.0);
    CHECK_NEAR(glm::length(a.pos), r, 1e-6, "radius = sea level + alt");
    CHECK_NEAR(std::asin(a.up.y), lat, 1e-12, "latitude held");
    CHECK_NEAR(glm::length(a.pos / r - a.up), 0.0, 1e-12, "up is radial");
    const double lap = 2.0 * std::numbers::pi / rate;
    Pose b = poseAt(r, lat, 0.0, rate, heading, lap);
    CHECK_NEAR(glm::length(a.pos - b.pos), 0.0, 1e-3, "one lap returns to start");
    Pose c = poseAt(r, lat, 0.0, rate, heading, 1234.5);
    Pose d = poseAt(r, lat, 0.0, rate, heading, 1234.5);
    CHECK_NEAR(glm::length(c.pos - d.pos), 0.0, 1e-12, "pose is a pure function of t");
    // A long warp hop stays precise: 1e7 s is ~116 laps.
    Pose w1 = poseAt(r, lat, 0.0, rate, heading, 1.0e7);
    Pose w2 = poseAt(r, lat, 0.0, rate, heading, 1.0e7 + lap);
    CHECK_NEAR(glm::length(w1.pos - w2.pos), 0.0, 1e-3, "warp-scale t stays periodic");

    printf("== poseAt: zero wind is static ==\n");
    Pose s0 = poseAt(r, lat, 42.0, 0.0, heading, 0.0);
    Pose s1 = poseAt(r, lat, 42.0, 0.0, heading, 1.0e6);
    CHECK_NEAR(glm::length(s0.pos - s1.pos), 0.0, 1e-9, "no wind, no motion");

    printf("== attitude: continuous ENU, heading into the wind ==\n");
    // Orthonormal, right-handed, +Z up.
    CHECK_NEAR(glm::length(a.orient[0]), 1.0, 1e-12, "x unit");
    CHECK_NEAR(glm::length(a.orient[1]), 1.0, 1e-12, "y unit");
    CHECK_NEAR(glm::dot(a.orient[0], a.orient[1]), 0.0, 1e-12, "x perp y");
    CHECK_NEAR(glm::dot(glm::cross(a.orient[0], a.orient[1]), a.orient[2]), 1.0,
               1e-12, "right-handed");
    CHECK_NEAR(glm::dot(a.orient[2], a.up), 1.0, 1e-12, "+Z is up");
    // Heading 180: deck +X = west = -east.
    const glm::dvec3 east0 = equirectEast(a.up);
    CHECK_NEAR(glm::length(a.orient * glm::dvec3(1, 0, 0) + east0), 0.0, 1e-12,
               "heading 180 points deck +X west");
    // Heading 0: deck +X = east.
    Pose h0 = poseAt(r, lat, 0.0, rate, 0.0, 0.0);
    CHECK_NEAR(glm::length(h0.orient * glm::dvec3(1, 0, 0) - east0), 0.0, 1e-12,
               "heading 0 points deck +X east");

    /* THE SNAP THIS LAW AVOIDS: faceAlong(up)'s discrete roll pick flips at
       lon ~10.16 deg at lat 10 (|dot(Z,up)| overtakes |dot(Y,up)|). The ENU
       basis must glide across it: 0.5 deg of longitude is ~0.5 deg of basis
       turn, not a 90 deg snap. */
    const double r1 = d2r;   // 1 deg/s
    Pose p1 = poseAt(r, lat, 0.0, r1, 0.0, 9.9);
    Pose p2 = poseAt(r, lat, 0.0, r1, 0.0, 10.4);
    CHECK_NEAR(glm::length(p1.orient[0] - p2.orient[0]), 0.0, 0.02,
               "deck X continuous across the faceAlong pick boundary");
    CHECK_NEAR(glm::length(p1.orient[1] - p2.orient[1]), 0.0, 0.02,
               "deck Y continuous across the faceAlong pick boundary");
    CHECK_NEAR(glm::length(p1.up - p2.up), 0.0, 0.02,
               "up continuous across the faceAlong pick boundary");

    printf("test_buildingair: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
