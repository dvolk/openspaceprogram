//
// Atmospheric drag law (src/drag.h, header-only pure math): the exponential
// density model and the quadratic force. Pinned here without Bullet/GL so the
// law is independent of the render/physics chain (like test_surfmap,
// test_orbit).
//
//   airDensity:  rho(alt) = rho0 * exp(-alt/H); below-surface -> 0; a
//                degenerate atmosphere -> 0; rho(H) = rho0/e; monotone down.
//   dragForce:   opposite the motion; |F| = 0.5 * rho * cd * A * v^2;
//                scales as v^2; zero on any degenerate input.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "drag.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                 \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s\n", msg);                                        \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, rel_tol, msg)                            \
    do {                                                                      \
        g_checks++;                                                           \
        double _a = (actual), _e = (expected);                                \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > rel_tol * std::fabs(_e)) { \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);          \
        }                                                                     \
    } while (0)

static void test_density() {
    printf("== airDensity: exponential model ==\n");

    const DragAtmosphere a { 1.225, 5500.0 };  // a Kerbin-ish atmosphere

    // Below / at the surface there is no air.
    CHECK_NEAR(airDensity(a, 0.0), 0.0, 0.0, "alt=0 -> 0");
    CHECK_NEAR(airDensity(a, -100.0), 0.0, 0.0, "alt<0 -> 0");

    // Just above the surface, the density is (essentially) sea level.
    CHECK_NEAR(airDensity(a, 1.0), 1.225 * std::exp(-1.0 / 5500.0), 1e-12,
               "rho(1 m) == rho0 * e^(-1/H)");

    // One scale height: the density has fallen to rho0 / e.
    CHECK_NEAR(airDensity(a, 5500.0), 1.225 / std::exp(1.0), 1e-12,
               "rho(H) == rho0 / e");

    // Monotone: denser lower down.
    CHECK_TRUE(airDensity(a, 500.0) > airDensity(a, 5000.0),
               "rho is monotone decreasing with altitude");

    // Deep in the air it is negligible (no hard "top" needed).
    CHECK_TRUE(airDensity(a, 8 * 5500.0) < 0.01 * 1.225,
               "rho(8H) < 1% of sea level");

    // A degenerate atmosphere (no density or no scale height) reads as none.
    const DragAtmosphere none0 { 0.0, 5500.0 };
    const DragAtmosphere noneH { 1.225, 0.0 };
    CHECK_NEAR(airDensity(none0, 100.0), 0.0, 0.0, "no density -> 0");
    CHECK_NEAR(airDensity(noneH, 100.0), 0.0, 0.0, "no scale height -> 0");
}

static void test_force() {
    printf("== dragForce: direction, magnitude, v^2 scaling ==\n");

    const DragAtmosphere a { 1.225, 5500.0 };
    const double cd = 1.2;
    const double A = 4.0;      // m^2
    const double alt = 100.0;  // m
    const double rho = airDensity(a, alt);

    // Straight up: the force is straight down, the right magnitude.
    const glm::dvec3 vup(0.0, 300.0, 0.0);
    const glm::dvec3 f_up = dragForce(a, cd, A, alt, vup);
    const double F_expect = 0.5 * rho * cd * A * 300.0 * 300.0;
    CHECK_NEAR(f_up.y, -F_expect, 1e-12, "F is opposite the motion");
    CHECK_NEAR(f_up.x, 0.0, 0.0, "no sideways component (1)");
    CHECK_NEAR(f_up.z, 0.0, 0.0, "no sideways component (2)");
    CHECK_NEAR(glm::length(f_up), F_expect, 1e-12, "|F| == 0.5 rho cd A v^2");

    // An oblique velocity: still exactly opposite it, same magnitude.
    const glm::dvec3 vob(100.0, 200.0, -100.0);
    const glm::dvec3 f_ob = dragForce(a, cd, A, alt, vob);
    const double v = glm::length(vob);
    const double F_ob = 0.5 * rho * cd * A * v * v;
    CHECK_NEAR(glm::length(f_ob), F_ob, 1e-12, "|F| at oblique v");
    // f_ob is anti-parallel to v_ob: the cross product is zero.
    CHECK_TRUE(glm::length(glm::cross(f_ob, vob)) < 1e-6 * F_ob,
               "F is anti-parallel to v");
    CHECK_TRUE(glm::dot(f_ob, vob) < 0.0, "F opposes v (dot < 0)");

    // Quadratic in speed: 2x speed -> 4x force.
    const glm::dvec3 f_2v = dragForce(a, cd, A, alt, vup * 2.0);
    CHECK_NEAR(glm::length(f_2v), 4.0 * glm::length(f_up), 1e-12,
               "|F(2v)| == 4 |F(v)|");

    // Degenerate inputs -> zero force (no guards needed at the call site).
    CHECK_TRUE(dragForce(a, cd, A, alt, glm::dvec3(0.0)) == glm::dvec3(0.0),
               "zero velocity -> zero force");
    CHECK_TRUE(dragForce(a, 0.0, A, alt, vup) == glm::dvec3(0.0),
               "cd=0 -> zero force");
    CHECK_TRUE(dragForce(a, cd, 0.0, alt, vup) == glm::dvec3(0.0),
               "area=0 -> zero force");
    const DragAtmosphere none { 0.0, 5500.0 };
    CHECK_TRUE(dragForce(none, cd, A, alt, vup) == glm::dvec3(0.0),
               "no atmosphere -> zero force");
}

int main() {
    test_density();
    printf("\n");
    test_force();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
