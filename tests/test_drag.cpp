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

static void test_offAxis() {
    printf("== offAxisFactor: 1 - (v*nose)^2 ==\n");
    const glm::dvec3 nose(0.0, 0.0, 1.0);   // +Z nose

    // Prograde (v along the nose) and retrograde (anti-parallel): no penalty.
    CHECK_NEAR(offAxisFactor(glm::dvec3(0.0, 0.0, 200.0), nose), 0.0, 1e-12,
               "prograde -> 0");
    CHECK_NEAR(offAxisFactor(glm::dvec3(0.0, 0.0, -200.0), nose), 0.0, 1e-12,
               "retrograde (anti-parallel) -> 0");

    // Sideways (v perpendicular to the nose): the full penalty.
    CHECK_NEAR(offAxisFactor(glm::dvec3(200.0, 0.0, 0.0), nose), 1.0, 1e-12,
               "sideways (+X) -> 1");
    CHECK_NEAR(offAxisFactor(glm::dvec3(0.0, 200.0, 0.0), nose), 1.0, 1e-12,
               "sideways (+Y) -> 1");

    // 45 degrees: cos^2 = 0.5, so the off-axis factor is 0.5.
    const glm::dvec3 v45(200.0, 0.0, 200.0);
    CHECK_NEAR(offAxisFactor(v45, nose), 0.5, 1e-12, "45 deg -> 0.5");

    // A ratio: invariant under independent scaling of v and nose.
    CHECK_NEAR(offAxisFactor(v45 * 3.0, nose * 2.0), offAxisFactor(v45, nose),
               1e-12, "scale-invariant");

    // Degenerate inputs -> 0.
    CHECK_NEAR(offAxisFactor(glm::dvec3(0.0), nose), 0.0, 0.0, "zero v -> 0");
    CHECK_NEAR(offAxisFactor(glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0)), 0.0, 0.0,
               "zero nose -> 0");
}

static void test_aeroFrame() {
    printf("== aeroFrame: alpha, beta, off-axis ==\n");
    // Ship axes in world: right=+X, up=+Y, nose=+Z (identity orientation).
    const glm::dvec3 right(1.0, 0.0, 0.0), up(0.0, 1.0, 0.0), nose(0.0, 0.0, 1.0);

    // Prograde: all zero, valid.
    AeroFrame f0 = aeroFrame(glm::dvec3(0.0, 0.0, 300.0), right, up, nose);
    CHECK_TRUE(f0.valid, "prograde is valid");
    CHECK_NEAR(f0.v, 300.0, 1e-12, "speed = |v|");
    CHECK_NEAR(f0.alpha, 0.0, 1e-12, "alpha 0 (prograde)");
    CHECK_NEAR(f0.beta, 0.0, 1e-12, "beta 0 (prograde)");
    CHECK_NEAR(f0.offAxis, 0.0, 1e-12, "off-axis 0 (prograde)");

    // Pure pitch: v has a +Y component -> alpha = atan2(vy, vz) = 45 deg.
    AeroFrame fp = aeroFrame(glm::dvec3(0.0, 300.0, 300.0), right, up, nose);
    CHECK_NEAR(fp.alpha, M_PI / 4.0, 1e-12, "alpha = 45 deg for (0,300,300)");
    CHECK_NEAR(fp.beta, 0.0, 1e-12, "beta 0 (pure pitch)");
    CHECK_NEAR(fp.offAxis, 0.5, 1e-12, "off-axis 0.5 (45 deg pitch)");

    // Pure bank: v has a +X component -> beta = atan2(vx, vz) = 45 deg.
    AeroFrame fb = aeroFrame(glm::dvec3(300.0, 0.0, 300.0), right, up, nose);
    CHECK_NEAR(fb.beta, M_PI / 4.0, 1e-12, "beta = 45 deg for (300,0,300)");
    CHECK_NEAR(fb.alpha, 0.0, 1e-12, "alpha 0 (pure bank)");

    // Degenerate: zero velocity -> not valid, all zero.
    AeroFrame fn = aeroFrame(glm::dvec3(0.0), right, up, nose);
    CHECK_TRUE(!fn.valid, "zero v -> not valid");
    CHECK_NEAR(fn.offAxis, 0.0, 0.0, "zero v -> off-axis 0");
}

static void test_dragForceAOA() {
    printf("== dragForceAOA: parasite (A0) + weathervane (AK) ==\n");
    const DragAtmosphere a { 1.225, 5500.0 };
    const double alt = 100.0;
    const double rho = airDensity(a, alt);
    const glm::dvec3 nose(0.0, 0.0, 1.0);
    const double A0 = 4.8;    // parasite area (e.g. cd 1.2 x area 4.0)
    const double AK = 2.0;    // weathervane area
    const double v = 300.0;

    // AK = 0 reproduces the v1 law exactly (the parasite-only special case).
    {
        const glm::dvec3 vel(0.0, 0.0, v);
        const glm::dvec3 f_aoa = dragForceAOA(a, A0, 0.0, alt, vel, nose);
        const glm::dvec3 f_v1  = dragForce(a, 1.2, A0 / 1.2, alt, vel);
        CHECK_TRUE(glm::length(f_aoa - f_v1) < 1e-9, "AK=0 == v1 dragForce");
    }

    // Prograde (nose along v): off-axis 0 -> the parasite term only.
    {
        const double F = glm::length(dragForceAOA(a, A0, AK, alt,
                                                  glm::dvec3(0.0, 0.0, v), nose));
        CHECK_NEAR(F, 0.5 * rho * A0 * v * v, 1e-12, "prograde -> A0 only");
    }

    // Sideways (nose perpendicular to v): the full weathervane term.
    {
        const double F = glm::length(dragForceAOA(a, A0, AK, alt,
                                                  glm::dvec3(v, 0.0, 0.0), nose));
        CHECK_NEAR(F, 0.5 * rho * (A0 + AK) * v * v, 1e-12, "sideways -> A0 + AK");
    }

    // Monotone in deflection at EQUAL speed (so only the off-axis factor
    // differs): prograde (0) < 45 deg (0.5) < sideways (1).
    {
        const double s = v / std::sqrt(2.0);   // (s,0,s) has |v| = v, 45 deg
        const double Fp  = glm::length(dragForceAOA(a, A0, AK, alt,
                                                    glm::dvec3(0.0, 0.0, v), nose));
        const double F45 = glm::length(dragForceAOA(a, A0, AK, alt,
                                                    glm::dvec3(s, 0.0, s), nose));
        const double Fs  = glm::length(dragForceAOA(a, A0, AK, alt,
                                                    glm::dvec3(v, 0.0, 0.0), nose));
        CHECK_TRUE(Fp < F45 && F45 < Fs, "drag grows with deflection");
    }

    // Still exactly opposite the motion, for any deflection.
    {
        const glm::dvec3 vel(300.0, 200.0, 100.0);
        const glm::dvec3 f = dragForceAOA(a, A0, AK, alt, vel, nose);
        CHECK_TRUE(glm::dot(f, vel) < 0.0, "F opposes v (deflected)");
        CHECK_TRUE(glm::length(glm::cross(f, vel)) < 1e-6 * glm::length(f),
                   "F anti-parallel to v (deflected)");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(dragForceAOA(a, A0, AK, alt, glm::dvec3(0.0), nose) == glm::dvec3(0.0),
               "zero v -> zero");
    CHECK_TRUE(dragForceAOA(a, 0.0, 0.0, alt, glm::dvec3(0.0, 0.0, v), nose) == glm::dvec3(0.0),
               "no area (A0=AK=0) -> zero");
    CHECK_TRUE(dragForceAOA(DragAtmosphere(0.0, 5500.0), A0, AK, alt,
                            glm::dvec3(0.0, 0.0, v), nose) == glm::dvec3(0.0),
               "no atmosphere -> zero");
}

int main() {
    test_density();
    printf("\n");
    test_force();
    printf("\n");
    test_offAxis();
    printf("\n");
    test_aeroFrame();
    printf("\n");
    test_dragForceAOA();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
