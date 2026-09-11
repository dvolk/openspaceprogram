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

static void test_liftDirection() {
    printf("== liftDirection: the wing normal, out of the flow ==\n");
    const glm::dvec3 right(1.0, 0.0, 0.0), nose(0.0, 0.0, 1.0);  // identity

    // Prograde (flow along the nose): the lift direction is the up axis.
    const glm::dvec3 ld0 = liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                                         right, nose);
    CHECK_NEAR(ld0.x, 0.0, 0.0, "prograde liftDir x == 0");
    CHECK_NEAR(ld0.y, 1.0, 1e-12, "prograde liftDir y == 1 (up)");
    CHECK_NEAR(ld0.z, 0.0, 0.0, "prograde liftDir z == 0");
    CHECK_NEAR(glm::length(ld0), 1.0, 1e-12, "liftDir is a unit vector");

    // Always perpendicular to the flow (lift is the force OUT of the flow).
    {
        const glm::dvec3 v(300.0, 100.0, 200.0);
        const glm::dvec3 ld = liftDirection(v, right, nose);
        CHECK_TRUE(std::fabs(glm::dot(ld, v)) < 1e-9,
                   "liftDir is perpendicular to v");
        CHECK_NEAR(glm::length(ld), 1.0, 1e-12, "deflected liftDir is unit");
    }

    // A banked ship: the lift direction follows the (banked) up axis, not
    // the world up. Rolled 90 deg about the nose: right=+Y, up=nose x right.
    {
        const glm::dvec3 right2(0.0, 1.0, 0.0), nose2(0.0, 0.0, 1.0);
        const glm::dvec3 v(0.0, 0.0, 300.0);  // prograde for this banked ship
        const glm::dvec3 ld2 = liftDirection(v, right2, nose2);
        const glm::dvec3 up2 = glm::cross(nose2, right2);  // = -X
        CHECK_NEAR(glm::length(ld2 - up2), 0.0, 1e-12,
                   "banked liftDir follows the banked up axis");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(liftDirection(glm::dvec3(0.0), right, nose) == glm::dvec3(0.0),
               "zero v -> zero");
    CHECK_TRUE(liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                             glm::dvec3(0.0), nose) == glm::dvec3(0.0),
               "zero right -> zero");
    CHECK_TRUE(liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                             right, glm::dvec3(0.0)) == glm::dvec3(0.0),
               "zero nose -> zero");
}

static void test_liftForce() {
    printf("== liftForce: |L| = q*S*CL(a), sign follows a, stalls past A ==\n");
    const glm::dvec3 liftDir(0.0, 1.0, 0.0);  // up
    const double q = 60.0;      // dynamic pressure (Pa)
    const double S = 4.0;       // m^2
    const double cl = 6.0;      // per radian
    const double alpha = 0.10;  // rad
    const double L_expect = q * S * cl * alpha;

    // Positive AoA: the lift is along liftDir, the right magnitude.
    const glm::dvec3 L = liftForce(q, S, cl, alpha, liftDir);
    CHECK_NEAR(L.y, L_expect, 1e-12, "|L| == q*S*cl*alpha (positive AoA)");
    CHECK_NEAR(L.x, 0.0, 0.0, "lift has no x component");
    CHECK_NEAR(L.z, 0.0, 0.0, "lift has no z component");

    // Negative AoA: the same magnitude, opposite direction (down).
    const glm::dvec3 Ln = liftForce(q, S, cl, -alpha, liftDir);
    CHECK_NEAR(Ln.y, -L_expect, 1e-12, "negative AoA pushes down");

    // Zero AoA: a symmetric section (CL0 = 0) generates no lift.
    CHECK_TRUE(liftForce(q, S, cl, 0.0, liftDir) == glm::dvec3(0.0),
               "zero AoA -> zero lift");

    // With a stall angle A: below stall it is still the linear law, and at
    // 2A it is a deep stall (zero lift) -- the vector path honours liftCurve.
    {
        const double A = 0.20;
        const glm::dvec3 L_lin  = liftForce(q, S, cl, 0.10, liftDir, A);
        CHECK_NEAR(L_lin.y, q * S * cl * 0.10, 1e-12, "below stall: linear");
        const glm::dvec3 L_deep = liftForce(q, S, cl, 2.0 * A, liftDir, A);
        CHECK_TRUE(L_deep == glm::dvec3(0.0), "deep stall (2A) -> zero lift");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(liftForce(0.0, S, cl, alpha, liftDir) == glm::dvec3(0.0),
               "no dynamic pressure -> zero");
    CHECK_TRUE(liftForce(q, 0.0, cl, alpha, liftDir) == glm::dvec3(0.0),
               "no lift area -> zero");
    CHECK_TRUE(liftForce(q, S, 0.0, alpha, liftDir) == glm::dvec3(0.0),
               "no cl (a rocket) -> zero");
}

static void test_liftCurve() {
    printf("== liftCurve: linear up to stall, soft droop, deep stall ==\n");
    const double cl = 6.0;        // per radian
    const double A  = 0.30;       // stall angle (rad)
    const double clmax = cl * A;  // peak CL, reached at |alpha| = A

    // Below stall: exactly linear (the Phase 2 law), sign follows alpha.
    CHECK_NEAR(liftCurve(cl, 0.05, A), cl * 0.05, 1e-12, "below stall: linear");
    CHECK_NEAR(liftCurve(cl, -0.05, A), -cl * 0.05, 1e-12, "below stall: linear (-)");

    // Zero AoA: no lift (a symmetric section).
    CHECK_NEAR(liftCurve(cl, 0.0, A), 0.0, 0.0, "zero AoA -> 0");

    // At the stall angle: the peak (cl*A), continuous from both sides.
    CHECK_NEAR(liftCurve(cl, A, A), clmax, 1e-12, "at stall angle: peak");
    CHECK_NEAR(liftCurve(cl, -A, A), -clmax, 1e-12, "at stall angle: peak (-)");

    // Just past stall: the droop has begun (below the peak, still positive).
    CHECK_NEAR(liftCurve(cl, A * 1.1, A),
               cl * A * std::cos(0.1 * M_PI * 0.5), 1e-12, "past stall: droop");

    // Monotone decrease through the droop: peak > 1.1A > 1.5A > 2A (=0).
    CHECK_TRUE(liftCurve(cl, A, A) > liftCurve(cl, A * 1.1, A), "droop: A > 1.1A");
    CHECK_TRUE(liftCurve(cl, A * 1.1, A) > liftCurve(cl, A * 1.5, A),
               "droop: 1.1A > 1.5A");
    CHECK_TRUE(liftCurve(cl, A * 1.5, A) > 0.0, "droop: 1.5A > 0");

    // At 2x the stall angle the droop is complete (deep stall); beyond it, and
    // at a fully sideways 90 deg, the wing is edge-on and generates no lift.
    CHECK_NEAR(liftCurve(cl, A * 2.0, A), 0.0, 0.0, "at 2A: deep stall -> 0");
    CHECK_NEAR(liftCurve(cl, A * 3.0, A), 0.0, 0.0, "beyond 2A: still 0");
    CHECK_NEAR(liftCurve(cl, M_PI * 0.5, A), 0.0, 0.0, "90 deg -> 0");

    // Symmetric in |alpha|: equal magnitude up or down, sign follows alpha.
    CHECK_NEAR(std::fabs(liftCurve(cl, 0.2, A)), std::fabs(liftCurve(cl, -0.2, A)),
               1e-12, "symmetric magnitude");
    CHECK_TRUE(liftCurve(cl, 0.2, A) > 0.0 && liftCurve(cl, -0.2, A) < 0.0,
               "sign follows alpha");

    // No stall (A = 0): pure linear for ALL alpha (the Phase 2 law, no droop).
    CHECK_NEAR(liftCurve(cl, 1.0, 0.0), cl * 1.0, 1e-12, "no stall: linear (+)");
    CHECK_NEAR(liftCurve(cl, -1.0, 0.0), -cl * 1.0, 1e-12, "no stall: linear (-)");

    // Degenerate: no slope -> no lift.
    CHECK_NEAR(liftCurve(0.0, 0.1, A), 0.0, 0.0, "cl = 0 -> 0");
    CHECK_NEAR(liftCurve(-1.0, 0.1, A), 0.0, 0.0, "cl < 0 -> 0");
}

static void test_partDrag() {
    printf("== partDrag: per-part, sums to the composite dragForceAOA ==\n");
    const DragAtmosphere a { 1.225, 5500.0 };
    const double alt = 100.0;
    const glm::dvec3 vrel(200.0, 50.0, 300.0);
    const glm::dvec3 nose(0.0, 0.0, 1.0);
    const double q = dynamicPressure(a, alt, vrel);
    const double offAxis = offAxisFactor(vrel, nose);
    const glm::dvec3 vhat = glm::normalize(vrel);

    // Two parts: the per-part drags sum to the composite (same area x cd and
    // area x k) -- so the v1 law is preserved, just distributed per part.
    {
        const double area1 = 2.0, cd1 = 1.0, k1 = 0.5;
        const double area2 = 3.0, cd2 = 2.0, k2 = 1.0;
        const glm::dvec3 fsum =
            partDrag(q, vhat, offAxis, area1, cd1, k1) +
            partDrag(q, vhat, offAxis, area2, cd2, k2);
        const double A0 = area1 * cd1 + area2 * cd2;
        const double AK = area1 * k1 + area2 * k2;
        const glm::dvec3 fcomp = dragForceAOA(a, A0, AK, alt, vrel, nose);
        CHECK_TRUE(glm::length(fsum - fcomp) < 1e-9,
                   "sum of partDrag == composite dragForceAOA");
    }

    // A single part with k = 0 is exactly the v1 dragForce (cd * area).
    {
        const double area = 4.0, cd = 1.2;
        const glm::dvec3 f = partDrag(q, vhat, offAxis, area, cd, 0.0);
        const glm::dvec3 f_v1 = dragForce(a, cd, area, alt, vrel);
        CHECK_TRUE(glm::length(f - f_v1) < 1e-9, "k=0 part == v1 dragForce");
    }

    // Opposite the flow, always (for any off-axis factor).
    {
        const glm::dvec3 f = partDrag(q, vhat, offAxis, 4.0, 1.2, 1.0);
        CHECK_TRUE(glm::dot(f, vrel) < 0.0, "partDrag opposes v");
        CHECK_TRUE(glm::length(glm::cross(f, vrel)) < 1e-6 * glm::length(f),
                   "partDrag anti-parallel to v");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(partDrag(0.0, vhat, offAxis, 4.0, 1.2, 1.0) == glm::dvec3(0.0),
               "no q -> zero");
    CHECK_TRUE(partDrag(q, vhat, offAxis, 0.0, 1.2, 1.0) == glm::dvec3(0.0),
               "no area -> zero");
}

static void test_controlForce() {
    printf("== controlForce: the deflection-driven steering force ==\n");
    const double q = 100.0;  // dynamic pressure
    const double S = 2.0;    // control area
    const double cl = 6.0;   // deflection effectiveness (per radian)
    const glm::dvec3 dir(0.0, 1.0, 0.0);  // force direction (the up axis)

    // Linear in the deflection: double the deflection -> double the force.
    {
        const glm::dvec3 f1 = controlForce(q, S, cl, 0.10, dir);
        const glm::dvec3 f2 = controlForce(q, S, cl, 0.20, dir);
        CHECK_TRUE(glm::length(f2 - 2.0 * f1) < 1e-9, "linear in delta");
        CHECK_TRUE(glm::length(f1) == q * S * cl * 0.10, "magnitude = q*S*cl*delta");
    }

    // The sign follows the deflection (symmetric, like a symmetric section).
    {
        const glm::dvec3 fpos = controlForce(q, S, cl, +0.10, dir);
        const glm::dvec3 fneg = controlForce(q, S, cl, -0.10, dir);
        CHECK_TRUE(glm::length(fpos + fneg) < 1e-9, "odd in delta");
        CHECK_TRUE(glm::dot(fpos, dir) > 0.0, "positive delta -> +dir");
        CHECK_TRUE(glm::dot(fneg, dir) < 0.0, "negative delta -> -dir");
    }

    // The force is exactly along `dir` (no lateral component).
    {
        const glm::dvec3 f = controlForce(q, S, cl, 0.15, dir);
        CHECK_TRUE(glm::length(glm::cross(f, dir)) < 1e-6 * glm::length(f),
                   "parallel to dir");
    }

    // Scales with q, area, and cl (each linearly).
    {
        CHECK_TRUE(glm::length(controlForce(2.0*q, S, cl, 0.1, dir))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir)),
                   "linear in q");
        CHECK_TRUE(glm::length(controlForce(q, 2.0*S, cl, 0.1, dir))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir)),
                   "linear in area");
        CHECK_TRUE(glm::length(controlForce(q, S, 2.0*cl, 0.1, dir))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir)),
                   "linear in cl");
    }

    // Degenerate inputs -> zero (no air, no area, no effectiveness, no deflection).
    CHECK_TRUE(controlForce(0.0, S, cl, 0.1, dir) == glm::dvec3(0.0), "no q -> zero");
    CHECK_TRUE(controlForce(q, 0.0, cl, 0.1, dir) == glm::dvec3(0.0), "no area -> zero");
    CHECK_TRUE(controlForce(q, S, 0.0, 0.1, dir) == glm::dvec3(0.0), "no cl -> zero");
    CHECK_TRUE(controlForce(q, S, cl, 0.0, dir) == glm::dvec3(0.0), "no delta -> zero");
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
    printf("\n");
    test_liftDirection();
    printf("\n");
    test_liftForce();
    printf("\n");
    test_liftCurve();
    printf("\n");
    test_partDrag();
    printf("\n");
    test_controlForce();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
