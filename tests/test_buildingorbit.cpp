// test_buildingorbit -- the pure orbital-building pose/attitude math
// (src/buildingorbit.h). No Bullet, no GL: these pins keep the driven pose a
// honest pure function of the clock. The placement/parse/collision wiring lives
// in buildings.cpp / terrain.cpp and is covered by test_buildings + the e2e.
//
// Build & run (from repo root): make test_buildingorbit   (or: make test)

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "buildingorbit.h"
#include "tchecks.h"

int main() {
    using namespace buildingorbit;
    const double mu = 1.0e12;
    const double r = 1.0e6;
    const double v_circ = std::sqrt(mu / r);
    const double period = 2.0 * M_PI * std::sqrt(r * r * r / mu);

    printf("== circularEpochState ==\n");
    glm::dvec3 p, v;
    CHECK_TRUE(circularEpochState(r, 0.0, 0.0, 0.0, mu, p, v), "epoch state builds");
    CHECK_NEAR(glm::length(p), r, 1e-6, "circular radius = r");
    CHECK_NEAR(glm::length(v), v_circ, 1e-6, "circular speed = sqrt(mu/r)");
    CHECK_NEAR(glm::dot(p, v), 0.0, 1e-3, "circular: pos perp vel");

    // Inclination tilts the plane: the orbit normal is no longer +Y.
    glm::dvec3 pi, vi;
    CHECK_TRUE(circularEpochState(r, glm::radians(51.6), 0.0, 0.0, mu, pi, vi),
               "inclined epoch builds");
    const glm::dvec3 h = glm::normalize(glm::cross(pi, vi));
    CHECK_NEAR(glm::length(pi), r, 1e-6, "inclined radius preserved");
    CHECK_TRUE(std::fabs(h.y) < 0.99, "inclined orbit normal tilts off +Y");

    printf("== poseAt: propagation is a pure f(t) ==\n");
    const glm::dvec3 spinAxis(0.0, 0.0, 1.0);
    Pose a = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, 0.0);
    Pose b = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, period);
    CHECK_NEAR(glm::length(a.pos - b.pos), 0.0, 1e-3, "one period returns to start");
    Pose c = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, period * 0.5);
    CHECK_NEAR(glm::length(a.pos - c.pos), 2.0 * r, 1e-3, "half period is opposite");
    // Deterministic: same t, same pose.
    Pose d = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, 123.456);
    Pose e = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, 123.456);
    CHECK_NEAR(glm::length(d.pos - e.pos), 0.0, 1e-12, "pose is a pure function of t");

    printf("== lvlhOrient: body-fixed frame ==\n");
    const glm::dmat3 L = lvlhOrient(p, v);
    // +Z is nadir (toward the body centre).
    CHECK_NEAR(glm::dot(L[2], -glm::normalize(p)), 1.0, 1e-9, "+Z points nadir");
    // Orthonormal, right-handed.
    CHECK_NEAR(glm::length(L[0]), 1.0, 1e-9, "x unit");
    CHECK_NEAR(glm::length(L[1]), 1.0, 1e-9, "y unit");
    CHECK_NEAR(glm::dot(L[0], L[1]), 0.0, 1e-9, "x perp y");
    CHECK_NEAR(glm::dot(glm::cross(L[0], L[1]), L[2]), 1.0, 1e-9, "right-handed");

    // Body-fixed stays nadir-pointing all the way around the orbit.
    for(double f = 0.0; f < 1.0; f += 0.25) {
        Pose q = poseAt(p, v, mu, Mode::BodyFixed, spinAxis, 0.0, period * f);
        const glm::dvec3 nadir = -glm::normalize(q.pos);
        CHECK_NEAR(glm::dot(q.orient[2], nadir), 1.0, 1e-6, "body-fixed tracks nadir");
    }

    printf("== sky-fixed vs spin ==\n");
    // Sky-fixed orientation is constant (no spin).
    Pose s0 = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, 0.0);
    Pose s1 = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, 0.0, period * 0.3);
    CHECK_NEAR(glm::length(s0.orient[0] - s1.orient[0]), 0.0, 1e-9, "sky-fixed constant");
    // Spin about local +Z advances at the authored rate: after t the local +X
    // has turned by rate*t about +Z.
    const double rate = 0.1;   // rad/s
    const double t = 7.0;
    Pose sp = poseAt(p, v, mu, Mode::SkyFixed, spinAxis, rate, t);
    const glm::dvec3 x0(1.0, 0.0, 0.0);
    const glm::dvec3 expect(std::cos(rate * t), std::sin(rate * t), 0.0);
    CHECK_NEAR(glm::length(sp.orient * x0 - expect), 0.0, 1e-9, "spin advances rate*t");

    printf("test_buildingorbit: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
