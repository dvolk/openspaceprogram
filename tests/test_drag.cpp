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

static void test_aeroFrame() {
    printf("== aeroFrame: alpha (pitch AoA), beta (sideslip) ==\n");
    // Ship axes in world: right=+X, up=+Y, nose=+Z (identity orientation).
    const glm::dvec3 right(1.0, 0.0, 0.0), up(0.0, 1.0, 0.0), nose(0.0, 0.0, 1.0);

    // Prograde: all zero, valid.
    AeroFrame f0 = aeroFrame(glm::dvec3(0.0, 0.0, 300.0), right, up, nose);
    CHECK_TRUE(f0.valid, "prograde is valid");
    CHECK_NEAR(f0.v, 300.0, 1e-12, "speed = |v|");
    CHECK_NEAR(f0.alpha, 0.0, 1e-12, "alpha 0 (prograde)");
    CHECK_NEAR(f0.beta, 0.0, 1e-12, "beta 0 (prograde)");

    // Pure pitch: v has a +Y component -> alpha = atan2(vy, vz) = 45 deg.
    AeroFrame fp = aeroFrame(glm::dvec3(0.0, 300.0, 300.0), right, up, nose);
    CHECK_NEAR(fp.alpha, M_PI / 4.0, 1e-12, "alpha = 45 deg for (0,300,300)");
    CHECK_NEAR(fp.beta, 0.0, 1e-12, "beta 0 (pure pitch)");

    // Pure bank: v has a +X component -> beta = atan2(vx, vz) = 45 deg.
    AeroFrame fb = aeroFrame(glm::dvec3(300.0, 0.0, 300.0), right, up, nose);
    CHECK_NEAR(fb.beta, M_PI / 4.0, 1e-12, "beta = 45 deg for (300,0,300)");
    CHECK_NEAR(fb.alpha, 0.0, 1e-12, "alpha 0 (pure bank)");

    // Degenerate: zero velocity -> not valid, all zero.
    AeroFrame fn = aeroFrame(glm::dvec3(0.0), right, up, nose);
    CHECK_TRUE(!fn.valid, "zero v -> not valid");
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

static void test_controlDeflectionSign() {
    printf("== controlDeflectionSign: position-dependent steering sign ==\n");
    // Ship axes: nose=+Z, right=+X, up=+Y. The reaction wheel's convention:
    // W (stick[1]=+1) -> torque about -right (nose up); A/D -> about -up.
    const glm::dvec3 nose(0,0,1), right(1,0,0), up(0,1,0);

    // A TAIL surface (behind the CG, ri ~ -nose) with W (stick[1]=+1) must
    // produce a -right torque (nose UP). The deflection sign makes the force
    // F = sign * stick[1] * up, and the torque is ri x F.
    {
        const glm::dvec3 ri = -nose;  // tail
        const double sign = controlDeflectionSign(ri, up, right);  // pitch
        const glm::dvec3 F = up * (sign * 1.0);  // W: stick[1]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, right) < 0.0, "tail W -> -right torque (nose up)");
    }

    // A CANARD surface (ahead of the CG, ri ~ +nose) with W must ALSO
    // produce a -right torque (nose UP) -- the deflection sign flips.
    {
        const glm::dvec3 ri = +nose;  // canard
        const double sign = controlDeflectionSign(ri, up, right);  // pitch
        const glm::dvec3 F = up * (sign * 1.0);  // W: stick[1]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, right) < 0.0, "canard W -> -right torque (nose up)");
    }

    // The tail and canard signs are OPPOSITE (that's the whole point).
    {
        const double tailSign   = controlDeflectionSign(-nose, up, right);
        const double canardSign = controlDeflectionSign(+nose, up, right);
        CHECK_TRUE(tailSign == -canardSign, "tail and canard signs are opposite");
    }

    // Yaw: a tail with stick[2]=+1 must produce a -up torque (matching the
    // wheel's A/D convention about -up).
    {
        const glm::dvec3 ri = -nose;  // tail
        const double sign = controlDeflectionSign(ri, right, up);  // yaw
        const glm::dvec3 F = right * (sign * 1.0);  // stick[2]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, up) < 0.0, "tail yaw -> -up torque");
    }

    // Roll: an aileron pair, laterally offset (ri ~ +/-right), force in the
    // up plane. The wheel's Q (stick[0]=+1) is a +nose torque, so BOTH
    // ailerons roll that way (right up / left down -- differential). The roll
    // targetSign is +1 (the wheel's roll is +nose, unlike pitch/yaw's -axis).
    {
        const double rRight = controlDeflectionSign(right,  up, nose, +1.0);
        const double rLeft  = controlDeflectionSign(-right, up, nose, +1.0);
        const glm::dvec3 Fright = up * (rRight * 1.0);  // stick[0]=+1
        const glm::dvec3 Fleft  = up * (rLeft  * 1.0);  // stick[0]=+1
        CHECK_TRUE(glm::dot(glm::cross(right, Fright), nose) > 0.0,
                   "right aileron Q -> +nose torque");
        CHECK_TRUE(glm::dot(glm::cross(-right, Fleft), nose) > 0.0,
                   "left aileron Q -> +nose torque");
        CHECK_TRUE(rRight == -rLeft, "aileron pair deflects opposite (roll)");
        CHECK_TRUE(Fright + Fleft == glm::dvec3(0.0),
                   "aileron pair: net force cancels (pure roll)");
    }

    // A surface exactly at the COM (zero lever) -> sign is moot; the helper
    // returns targetSign (here -1, the default) -- defined, no crash, no
    // zero-div.
    {
        const double sign = controlDeflectionSign(glm::dvec3(0.0), up, right);
        CHECK_TRUE(sign == 1.0 || sign == -1.0, "COM surface: sign is +-1 (defined)");
        CHECK_TRUE(sign == -1.0, "COM surface: returns targetSign (-1 default)");
    }
}

static void test_controlCl() {
    printf("== controlCl: deflection effectiveness (cl_control, fallback cl) ==\n");
    // cl_control > 0 -> it is the deflection effectiveness (the part's own
    // number), even if cl (the lift slope) is also set.
    CHECK_TRUE(controlCl(0.0, 6.0) == 6.0, "cl_control set -> used");
    CHECK_TRUE(controlCl(6.0, 4.0) == 4.0, "cl_control overrides cl");
    // cl_control = 0 (unset) -> fall back to cl: the old single-"cl" behaviour
    // for a part that only declares cl.
    CHECK_TRUE(controlCl(6.0, 0.0) == 6.0, "cl_control 0 -> falls back to cl");
    CHECK_TRUE(controlCl(0.0, 0.0) == 0.0, "neither set -> 0");
}

// --- Projected (silhouette) area of a body, from its convex-hull vertices.
//
// projectedArea(hullVerts, dir) is the area of the convex hull of the
// projected vertices -- the body's silhouette facing dir. Pinned against
// hand-computed values for a cube, a 32-gon prism (the "cylinder limit" the
// tanks are built from), and a thin box (the wing: face-on = the planform,
// edge-on = thickness * length). All vertex sets are built by hand -- no
// Bullet, no mesh -- so the law is pinned independently of the extraction.

static std::vector<glm::dvec3> cubeVerts() {
    // Side-2 cube, centred at the origin: 8 vertices.
    std::vector<glm::dvec3> v;
    v.push_back(glm::dvec3(-1, -1, -1));
    v.push_back(glm::dvec3( 1, -1, -1));
    v.push_back(glm::dvec3( 1,  1, -1));
    v.push_back(glm::dvec3(-1,  1, -1));
    v.push_back(glm::dvec3(-1, -1,  1));
    v.push_back(glm::dvec3( 1, -1,  1));
    v.push_back(glm::dvec3( 1,  1,  1));
    v.push_back(glm::dvec3(-1,  1,  1));
    return v;
}

static std::vector<glm::dvec3> prismVerts(int n = 32, double r = 1.0,
                                          double h = 3.0) {
    // A regular n-gon prism (a faceted cylinder): two rings of n vertices.
    std::vector<glm::dvec3> v;
    for(int k = 0; k < n; k++) {
        const double th = 2.0 * M_PI * k / n;
        const glm::dvec3 xy(r * std::cos(th), r * std::sin(th), 0.0);
        v.push_back(glm::dvec3(xy.x, xy.y, -h / 2.0));
        v.push_back(glm::dvec3(xy.x, xy.y,  h / 2.0));
    }
    return v;
}

static std::vector<glm::dvec3> boxVerts(double hx, double hy, double hz) {
    // Box of half-extents (hx, hy, hz): 8 vertices.
    std::vector<glm::dvec3> v;
    for(int i = 0; i < 8; i++) {
        v.push_back(glm::dvec3((i & 1) ?  hx : -hx,
                               (i & 2) ?  hy : -hy,
                               (i & 4) ?  hz : -hz));
    }
    return v;
}

static void test_projectedArea() {
    printf("== projectedArea: silhouette = hull of the projected vertices ==\n");

    // Cube: axis-on = one face (4); (1,0,1) shows two faces at 45 deg ->
    // 4 / sqrt(2) each -> 8 / sqrt(2) = 4 sqrt(2).
    const std::vector<glm::dvec3> cube = cubeVerts();
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.0, 0.0, 1.0)), 4.0, 1e-9,
               "cube axis-on == 4");
    CHECK_NEAR(projectedArea(cube, glm::dvec3(1.0, 0.0, 1.0)),
               8.0 / std::sqrt(2.0), 1e-9, "cube (1,0,1) == 4 sqrt(2)");

    // Symmetry: a convex body's silhouette is the same from either side.
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.3, -0.5, 0.8)),
               projectedArea(cube, glm::dvec3(-0.3, 0.5, -0.8)), 1e-9,
               "silhouette symmetric in dir (cube)");

    // 32-gon prism (the "cylinder limit"): end-on ~ pi (within 1%), side-on
    // ~ 2 r h (within 1%). Pins that the hull-of-vertices reproduces the
    // analytic cylinder areas the game's tanks are built from.
    const std::vector<glm::dvec3> prism = prismVerts(32, 1.0, 3.0);
    const double end = projectedArea(prism, glm::dvec3(0.0, 0.0, 1.0));
    CHECK_TRUE(std::fabs(end - M_PI) < 0.01 * M_PI,
               "prism end-on within 1% of pi (cylinder limit)");
    const double side = projectedArea(prism, glm::dvec3(1.0, 0.0, 0.0));
    CHECK_TRUE(std::fabs(side - 6.0) < 0.01 * 6.0,
               "prism side-on within 1% of 2 r h (cylinder limit)");
    // More sides -> closer to the cylinder: 6-gon is coarser, 32-gon tighter.
    const double end6 = projectedArea(prismVerts(6, 1.0, 3.0),
                                      glm::dvec3(0.0, 0.0, 1.0));
    CHECK_TRUE(std::fabs(end - M_PI) < std::fabs(end6 - M_PI),
               "32-gon end-on closer to pi than the 6-gon");

    // Thin box (the wing: span 2 x, thickness 0.1 y, chord 2 z): face-on
    // (along the thin axis) = the planform (4); edge-on = thickness * length
    // (0.2). This is the 10x thin-wing ratio the drag model relies on.
    const std::vector<glm::dvec3> wing = boxVerts(1.0, 0.05, 1.0);
    CHECK_NEAR(projectedArea(wing, glm::dvec3(0.0, 1.0, 0.0)), 4.0, 1e-9,
               "wing face-on == planform (4)");
    CHECK_NEAR(projectedArea(wing, glm::dvec3(1.0, 0.0, 0.0)), 0.2, 1e-9,
               "wing edge-on (x) == thickness * chord (0.2)");
    CHECK_NEAR(projectedArea(wing, glm::dvec3(0.0, 0.0, 1.0)), 0.2, 1e-9,
               "wing edge-on (z) == thickness * span (0.2)");

    // A STACK of coaxial parts (a rocket: 3 boxes end-to-end along Z) is ONE
    // body: its silhouette prograde is a single end face (2x2=4), not 3 end
    // faces (the per-part sum would give 3x4=12 -- the stacking over-count
    // this model removes). Side-on the stack's length adds up (2 * 9 = 18),
    // which is correct for a long body seen from the side. This is why the
    // drag AREA is computed over the ship's combined vertices, not summed
    // per part (reports/projected-drag).
    {
        std::vector<glm::dvec3> stack;
        for(int i = -1; i <= 1; i++) {          // 3 boxes, each 3 long, along Z
            const double z0 = 3.0 * i;
            for(int k = 0; k < 8; k++) {
                stack.push_back(glm::dvec3((k & 1) ?  1.0 : -1.0,
                                           (k & 2) ?  1.0 : -1.0,
                                           z0 + ((k & 4) ? 1.5 : -1.5)));
            }
        }
        CHECK_NEAR(projectedArea(stack, glm::dvec3(0.0, 0.0, 1.0)), 4.0, 1e-9,
                   "stacked rocket: ONE end face prograde (4), not 3 (12)");
        CHECK_NEAR(projectedArea(stack, glm::dvec3(1.0, 0.0, 0.0)), 18.0, 1e-9,
                   "stacked rocket: full side length side-on (18)");
        // The prograde->side swing is the honest ~4.5x (18/4), not the ~1x a
        // per-part sum would give for a prograde stack.
        CHECK_TRUE(projectedArea(stack, glm::dvec3(1.0,0,0))
                   > 2.0 * projectedArea(stack, glm::dvec3(0,0,1)),
                   "stack: side-on drags >2x prograde (attitude matters)");
    }

    // Degenerate inputs -> 0.
    CHECK_NEAR(projectedArea(std::vector<glm::dvec3>(), glm::dvec3(0.0, 0.0, 1.0)),
               0.0, 0.0, "empty vertex list -> 0");
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.0)), 0.0, 0.0, "zero dir -> 0");
    CHECK_NEAR(projectedArea({glm::dvec3(0.0, 0.0, 0.0),
                              glm::dvec3(1.0, 0.0, 0.0)},
                             glm::dvec3(0.0, 0.0, 1.0)), 0.0, 0.0,
               "2 vertices -> 0");
    // Collinear 3 vertices -> a degenerate (line) hull -> 0.
    {
        std::vector<glm::dvec3> line;
        line.push_back(glm::dvec3(0.0, 0.0, 0.0));
        line.push_back(glm::dvec3(1.0, 0.0, 0.0));
        line.push_back(glm::dvec3(2.0, 0.0, 0.0));
        CHECK_NEAR(projectedArea(line, glm::dvec3(0.0, 1.0, 0.0)), 0.0, 0.0,
                   "collinear vertices -> 0");
    }
}

static void test_partCd() {
    printf("== partCd: 3-anchor directional blend (nose / side / base) ==\n");

    // A cone-like part (the capsule): sleek nose (0.35), blunt side (0.8),
    // blunt base (1.3) -- the flat heat shield does the re-entry braking.
    const double fwd = 0.35, side = 0.80, bwd = 1.30;

    // Each anchor is hit EXACTLY at its angle (c = cos of nose-axis vs flow):
    //   c = +1  -> nose into the flow  -> drag_forward
    //   c =  0  -> broadside           -> drag_side
    //   c = -1  -> base into the flow  -> drag_backward
    CHECK_NEAR(partCd(fwd, side, bwd,  1.0), fwd,  1e-12, "c=+1 -> forward anchor");
    CHECK_NEAR(partCd(fwd, side, bwd,  0.0), side, 1e-12, "c=0 -> side anchor");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.0), bwd,  1e-12, "c=-1 -> backward anchor");

    // CONVEX blend: the weights (side(1-c^2), fwd(max(c,0)^2), bwd(max(-c,0)^2))
    // are non-negative and sum to 1, so cd stays within the anchor range for
    // every angle -- no overshoot, no dip below the sleekest anchor.
    const double mn = 0.35, mx = 1.30;
    for(double c = -1.0; c <= 1.0 + 1e-9; c += 0.05) {
        const double cd = partCd(fwd, side, bwd, c);
        CHECK_TRUE(cd >= mn - 1e-12 && cd <= mx + 1e-12,
                   "cd in [min,max] anchor range (sweep)");
    }

    // The weights sum to 1 (a true convex combination, not scaled/offset) --
    // pin the exact value at two 45 deg angles:
    //   c = +1/sqrt2 (nose-in):  side*0.5 + fwd*0.5 + bwd*0
    //   c = -1/sqrt2 (base-in):  side*0.5 + fwd*0 + bwd*0.5
    CHECK_NEAR(partCd(fwd, side, bwd,  1.0 / std::sqrt(2.0)),
               0.5 * side + 0.5 * fwd, 1e-12, "c=+1/sqrt2 -> .5 side + .5 fwd");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.0 / std::sqrt(2.0)),
               0.5 * side + 0.5 * bwd, 1e-12, "c=-1/sqrt2 -> .5 side + .5 bwd");

    // With fwd < side < bwd the blend eases nose->base and is monotone up as
    // the part turns from nose-first to base-first (the weathervane asymmetry).
    CHECK_TRUE(partCd(fwd, side, bwd,  1.0) < partCd(fwd, side, bwd, 0.0),
               "forward < side (sleeker nose than broadside)");
    CHECK_TRUE(partCd(fwd, side, bwd, 0.0) < partCd(fwd, side, bwd, -1.0),
               "side < backward (broadside blunter than the base)");
    CHECK_TRUE(partCd(fwd, side, bwd, 0.5) < partCd(fwd, side, bwd, -0.5),
               "nose-in cd < base-in cd (fwd < bwd)");

    // A SYMMETRIC part (all three anchors equal -- the shared `drag` fallback
    // path) is that value at EVERY angle.
    {
        const double s = 1.0;
        for(double c = -1.0; c <= 1.0 + 1e-9; c += 0.1) {
            CHECK_NEAR(partCd(s, s, s, c), s, 1e-12, "symmetric part -> s");
        }
    }

    // A thin DISC: the opposite asymmetry to a cone -- sleek edge-on
    // (fwd/bwd low), blunt face-on (side high).
    {
        const double df = 0.2, ds = 1.1, db = 0.2;
        CHECK_TRUE(partCd(df, ds, db, 0.0) > partCd(df, ds, db, 1.0),
                   "disc: face-on > edge-on");
        CHECK_TRUE(partCd(df, ds, db, 1.0) < partCd(df, ds, db, 0.5),
                   "disc: edge (c=1) < 45 deg");
    }

    // Robustness: a slightly-out-of-range cosine (a non-unit axis or flow) is
    // clamped, not amplified -- c=+1.5 behaves like the forward anchor.
    CHECK_NEAR(partCd(fwd, side, bwd,  1.5), fwd, 1e-12, "c>1 clamps to forward");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.5), bwd, 1e-12, "c<-1 clamps to backward");
}

int main() {
    test_density();
    printf("\n");
    test_force();
    printf("\n");
    test_aeroFrame();
    printf("\n");
    test_liftDirection();
    printf("\n");
    test_liftForce();
    printf("\n");
    test_liftCurve();
    printf("\n");
    test_projectedArea();
    printf("\n");
    test_partCd();
    printf("\n");
    test_controlForce();
    printf("\n");
    test_controlDeflectionSign();
    printf("\n");
    test_controlCl();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
