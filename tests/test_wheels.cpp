// test_wheels -- the landing-gear ground law (src/wheel.h), as pure math.
// No Bullet, no GL: wheel::force turns a contact + patch velocity into a force,
// and these pins keep that turn honest. The contact-finding (ray + analytic
// fall-through) and the one-body application live in Vehicle::applyGroundForces
// and are covered end-to-end by the runway e2e cases.
//
// Build & run (from repo root): make test_wheels   (or: make test)

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "wheel.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if(!(cond)) { g_failures++; printf("FAIL: %s\n", msg); }               \
    } while(0)

#define CHECK_NEAR(actual, expected, tol, msg)                                 \
    do {                                                                       \
        g_checks++;                                                            \
        const double _a = (actual), _e = (expected);                           \
        if(!std::isfinite(_a) || std::fabs(_a - _e) > (tol)) {                 \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g, tol %.3g)\n",               \
                   msg, _a, _e, (double)(tol));                                \
        }                                                                      \
    } while(0)

// A wheel on flat ground: normal +Z, rolling +Y, at rest unless told otherwise.
static wheel::Input rest(double compression) {
    wheel::Input in;
    in.compression = compression;
    in.compressRate = 0.0;
    in.spring = 10000.0;
    in.damp = 1000.0;
    in.normal = glm::dvec3(0.0, 0.0, 1.0);
    in.forward = glm::dvec3(0.0, 1.0, 0.0);
    in.contactVel = glm::dvec3(0.0);
    in.brake = 0.0;
    return in;
}

int main() {
    printf("== wheel::force ==\n");
    const glm::dvec3 Z(0.0, 0.0, 1.0);

    // Airborne: no compression, no force (whatever the velocity).
    {
        wheel::Input in = rest(0.0);
        in.contactVel = glm::dvec3(0.0, 50.0, -5.0);
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "airborne -> zero");
        in.compression = -0.1;
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "negative compression -> zero");
    }

    // Static spring: F = k*x along the normal, nothing in the plane.
    {
        wheel::Input in = rest(0.1);
        const glm::dvec3 F = wheel::force(in);
        CHECK_NEAR(F.z, 10000.0 * 0.1, 1e-9, "spring push = k*x");
        CHECK_NEAR(F.x, 0.0, 1e-12, "static: no lateral");
        CHECK_NEAR(F.y, 0.0, 1e-12, "static: no rolling");
    }

    // Damper: closing fast adds push; opening fast subtracts, but never pulls.
    {
        wheel::Input in = rest(0.1);
        in.compressRate = 1.0;   // closing
        CHECK_NEAR(wheel::force(in).z, 10000.0 * 0.1 + 1000.0 * 1.0, 1e-9,
                   "damper adds on closing");
        in.compressRate = -1.0;  // opening hard: k*x - c = 1000 - 1000 = 0
        CHECK_NEAR(wheel::force(in).z, 0.0, 1e-9, "damper subtracts on opening");
        in.compressRate = -5.0;  // would go negative -> clamped to 0 (no stick)
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "suspension never pulls");
    }

    // Rolling resistance opposes forward motion (and is small).
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(0.0, 20.0, 0.0);   // rolling forward
        const glm::dvec3 F = wheel::force(in);
        CHECK_TRUE(F.y < 0.0, "rolling resistance opposes forward");
        CHECK_TRUE(std::fabs(F.y) < 0.1 * F.z, "rolling resistance is a small fraction of load");
    }

    // Brake opposes forward motion, harder than rolling resistance.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(0.0, 20.0, 0.0);
        const double roll = wheel::force(in).y;
        in.brake = 1.0;
        const double braked = wheel::force(in).y;
        CHECK_TRUE(braked < roll, "brake pushes back harder than rolling");
        CHECK_TRUE(braked < 0.0, "brake opposes forward");
    }

    // Lateral grip opposes sideways slip.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(5.0, 0.0, 0.0);   // slipping +X (lateral)
        const glm::dvec3 F = wheel::force(in);
        CHECK_TRUE(F.x < 0.0, "lateral grip opposes slip");
        CHECK_NEAR(F.y, 0.0, 1e-9, "pure lateral slip: no rolling force");
    }

    // Friction circle: a huge slip cannot exceed mu*N in the plane.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(500.0, 500.0, 0.0);
        const glm::dvec3 F = wheel::force(in);
        const double N = F.z;
        const double tang = std::sqrt(F.x * F.x + F.y * F.y);
        CHECK_TRUE(tang <= wheel::kGrip * N + 1e-6, "tangential capped at mu*N");
        CHECK_NEAR(N, 10000.0 * 0.1, 1e-9, "normal unaffected by slip");
    }

    // The normal component is always along the ground normal (a tilted deck).
    {
        wheel::Input in = rest(0.1);
        const glm::dvec3 n = glm::normalize(glm::dvec3(0.1, 0.0, 1.0));
        in.normal = n;
        in.forward = glm::normalize(glm::dvec3(0.0, 1.0, 0.0) - n * glm::dot(glm::dvec3(0.0, 1.0, 0.0), n));
        const glm::dvec3 F = wheel::force(in);
        const glm::dvec3 tang = F - n * glm::dot(F, n);
        CHECK_NEAR(glm::length(tang), 0.0, 1e-9, "static force is along the normal");
        CHECK_TRUE(glm::dot(F, n) > 0.0, "pushes out of the deck");
    }

    printf("test_wheels: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
