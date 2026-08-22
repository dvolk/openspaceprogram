//
// Regression tests for the physical rotation model (2026-08-22):
//
//   1. Rotation authority is a physical quantity, like thrust:
//        alpha = maxTorque() / I   (rad/s^2),  I in kg m^2 from Bullet,
//      where maxTorque() = (number of reaction wheels) x the wheel's rated
//      torque (2000 N m each, the value the ship code has always applied).
//      The manual stick commands full rated torque; the autopilot
//      (prograde/retrograde slew, kill-rot) is bounded by the SAME
//      authority -- no command can be more forceful than a maxed manual
//      stick. (The thrust analogue: T = mdot x ve N.)
//
//   2. Slew law, per SUBSTEP (src/main.cpp Vehicle::slewToward):
//         w_des = min(sqrt(2*alpha*E), E/(2h))      (the braking curve,
//                                                     capped so a substep
//                                                     at w_des cannot
//                                                     cross the target)
//         dw    = clamp(w_des - w, -alpha*h, +alpha*h)
//         (x, w) integrated semi-implicitly over h, h = step/n,
//          n = max(3, round(step/0.1)) -- the main loop's substep count.
//      This is the per-substep form of the law test_attitude.cpp pins
//      per-tick (v_des = sqrt(2*A*|x|/step), dv = clamp(v_des - v, -A, A),
//      A = the per-tick domega authority). The two agree when the step is
//      re-evaluated n times instead of once.
//
//   3. WHY the substep form is what main.cpp uses: the per-tick form
//      LIMIT-CYCLES when the authority is high enough that the continuous
//      braking solution settles within a few ticks (a = 5e-1 in the
//      test_attitude parameterization a = A*step/x0): the ODE becomes
//      stiff relative to the tick and explicit per-tick Euler oscillates
//      through the target forever. The per-tick form fails 15 of the 216
//      configurations of the test_attitude grid and 22 of 288 with the
//      1000x-warp step (20 s) added; the per-substep form passes ALL 288
//      (verified by this test). The no-crossing cap E/(2h) is what makes
//      the stiff case settle: at full target rate one substep covers at
//      most half the error, so the error shrinks monotonically once the
//      ship is near the target. tests/test_attitude.cpp pins the same
//      per-substep law (updated 2026-08-22 to the form main.cpp runs).
//
// What this test asserts:
//   1. Slew (per-substep law) converges from 5/30/90 deg, v0 in
//      {0, +1, -1, -2} x v_safe, for a in {5e-6 .. 5e-1} x step in
//      {0.02, 0.2, 2.0, 2.0 s} (1x/10x/100x/1000x warp) x x0 in
//      {5, 30, 90} deg: within a bounded tick budget, <= 3 target
//      crossings, no NaN/Inf -- including every configuration where the
//      per-tick form fails (22 of 288).
//   2. The per-tick form fails in at least one configuration of that grid
//      -- i.e. the harness is capable of catching the limit-cycle
//      (vacuous-test guard), and the per-substep form is doing real work.
//   3. Authority bound: the commanded domega per substep never exceeds
//      alpha*h -- the autopilot is never more forceful than a maxed
//      manual stick.
//   4. KillRot (per-substep law) decays the spin monotonically, no sign
//      flip, at every warp.
//   5. getInertiaDiag (src/physics.cpp, the REAL function) reads back the
//      shape-derived inertia, scales linearly with mass through SetMass,
//      and is not the identity tensor. GetAngVelocity round-trips.
//   6. Torque delivery, on a REAL Bullet world (mirrors test_thrust.cpp):
//      torque re-applied before EVERY substep delivers the full
//      alpha*T; applied once per tick (the old pattern) delivers
//      alpha*T/n -- the harness must catch the 1/n loss.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

// OSP runs Bullet on the double-precision path (physics.cpp asserts
// sizeof(btScalar) == 8). The macro must be set before ANY bullet header.
#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "body.h"
#include "physics.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(actual, expected, rel_tol, msg)                             \
    do {                                                                       \
        g_checks++;                                                            \
        double _a = (actual), _e = (expected);                                 \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > rel_tol * std::fabs(_e)) { \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);           \
        }                                                                      \
    } while (0)

/*
 * The main loop's substep count (src/main.cpp): n = max(3, round(step/0.1)),
 * capped at 2000. h = step/n.
 */
static int main_loop_n(double step) {
    const double kMaxSubStep = 0.1;
    int n = 3;
    int need = (int)(step / kMaxSubStep + 0.5);
    if(need > n) { n = need; }
    if(n > 2000) { n = 2000; }
    return n;
}

struct RunResult {
    bool converged;
    int crossings;     // times the trajectory crossed the target
    bool bad;          // NaN/Inf
    double max_dow_ratio; // max |dw| / (alpha*h) over all substeps
};

// The slew law on a 1-D error angle, x' = w, x0 > 0 (target at x = 0).
// per_substep = true: the main.cpp implementation (law re-evaluated every
// substep h = step/n, plus the no-crossing cap |w_des| <= |x|/(2h)).
// per_substep = false: the per-tick variant of the same law (h = step,
// no cap) -- kept here so the SAME harness can show it failing where the
// per-substep form passes (the non-vacuous guard below).
// A = the per-tick domega authority (rad/s per tick) of test_attitude,
// i.e. alpha = A/step (rad/s^2).
static RunResult run_slew(double A, double step, double x0, double v0,
                          int budget, bool per_substep) {
    double x = x0, v = v0;
    int crossings = 0;
    const int n = per_substep ? main_loop_n(step) : 1;
    const double h = step / n;
    const double alpha = A / step; // rad/s^2
    double max_ratio = 0.0;

    for(int t = 0; t < budget; t++) {
        for(int i = 0; i < n; i++) {
            const double sign = (x >= 0.0) ? 1.0 : -1.0;
            double w_des = -sign * std::sqrt(2.0 * alpha * std::fabs(x));
            if(per_substep) {
                // no-crossing cap: at the target rate one substep covers
                // at most half the current error
                const double cap = std::fabs(x) / (2.0 * h);
                if(std::fabs(w_des) > cap) { w_des = -sign * cap; }
            }
            double dw = w_des - v;
            const double Amax = alpha * h;
            if(dw > Amax) { dw = Amax; }
            if(dw < -Amax) { dw = -Amax; }
            if(std::fabs(dw) > Amax * max_ratio) { max_ratio = std::fabs(dw) / Amax; }
            v += dw;
            if(!std::isfinite(x) || !std::isfinite(v)) {
                return RunResult{false, crossings, true, max_ratio};
            }
            const double x_new = x + v * h;
            if(!std::isfinite(x_new)) {
                return RunResult{false, crossings, true, max_ratio};
            }
            if(x_new * x < 0.0) { crossings++; }
            x = x_new;
        }

        if(std::fabs(x) < M_PI / 180.0) { // within 1 deg: must STAY there
            double x2 = x, v2 = v;
            bool stays = true;
            for(int m = 0; m < 200 && stays; m++) {
                for(int i = 0; i < n && stays; i++) {
                    const double s2 = (x2 >= 0.0) ? 1.0 : -1.0;
                    double w2 = -s2 * std::sqrt(2.0 * alpha * std::fabs(x2));
                    if(per_substep) {
                        const double cap2 = std::fabs(x2) / (2.0 * h);
                        if(std::fabs(w2) > cap2) { w2 = -s2 * cap2; }
                    }
                    double dw2 = w2 - v2;
                    const double A2 = alpha * h;
                    if(dw2 > A2) { dw2 = A2; }
                    if(dw2 < -A2) { dw2 = -A2; }
                    v2 += dw2;
                    if(!std::isfinite(x2) || !std::isfinite(v2)) { stays = false; break; }
                    x2 += v2 * h;
                    if(!std::isfinite(x2) || std::fabs(x2) > 5.0 * M_PI / 180.0) {
                        stays = false;
                    }
                }
            }
            if(stays) {
                return RunResult{true, crossings, false, max_ratio};
            }
        }
    }
    return RunResult{false, crossings, false, max_ratio};
}

// KillRot's per-substep law (src/main.cpp killRotStep), 1-D:
//   dw = -v * min(1, alpha*h/|v|)  ->  |v| drops by min(|v|, alpha*h)
// Monotonic, no sign flip, reaches exactly 0.
static bool killrot_converges(double A, double step, double v0, int budget) {
    const int n = main_loop_n(step);
    const double h = step / n;
    const double alpha = A / step;
    double v = v0;
    for(int t = 0; t < budget; t++) {
        for(int i = 0; i < n; i++) {
            if(v == 0.0) { return true; }
            const double Amax = alpha * h;
            double dw = -v * std::min(1.0, Amax / std::fabs(v));
            v += dw;
            if(!std::isfinite(v)) { return false; }
            if((v > 0.0) != (v0 > 0.0) && v != 0.0) { return false; } // sign flip
        }
        if(std::fabs(v) < 0.001 * std::fabs(v0)) { return true; }
    }
    return false;
}

/*
 * 1 + 2 + 3. The slew law, full grid (superset of test_attitude's grid:
 * adds step = 2.0 s = 1000x warp), per-substep form must converge
 * everywhere; the per-tick form must fail somewhere (non-vacuous).
 */
static void test_slew_law() {
    printf("== Slew: per-substep braking law, 1x..1000x warp ==\n");
    // a = A*step/x0: the per-tick authority relative to the initial error
    // (test_attitude's parameterization). The standard ship (I ~ 8444
    // kg m^2, one 2000 N m wheel) sits at a ~ 2.4e-4 at 1x warp, so the
    // sweep brackets the real operating range on both sides.
    const double a_grid[] = {5e-6, 5e-5, 5e-4, 5e-3, 5e-2, 5e-1};
    const double step_grid[] = {0.02, 0.2, 2.0, 20.0}; // 1x, 10x, 100x, 1000x
    const double x0_grid[] = {5.0, 30.0, 90.0};        // deg
    const double v0_grid[] = {0.0, 1.0, -1.0, -2.0};   // v_safe multiples

    int old_fail = 0, runs = 0;
    double worst_ratio = 0.0;
    for(double a : a_grid) {
        for(double step : step_grid) {
            for(double x0deg : x0_grid) {
                const double x0 = x0deg * M_PI / 180.0;
                const double A = a * x0 / step;
                const double v_safe = std::sqrt(2.0 * A * x0 / step);
                for(double v0f : v0_grid) {
                    RunResult r = run_slew(A, step, x0, v0f * v_safe, 3000, true);
                    char buf[192];
                    snprintf(buf, sizeof buf,
                             "slew a=%.0e step=%.2f x0=%.0fdeg v0=%.1fv_safe: "
                             "converged=%d crossings=%d bad=%d",
                             a, step, x0deg, v0f, (int)r.converged,
                             r.crossings, (int)r.bad);
                    CHECK_TRUE(r.converged && !r.bad && r.crossings <= 3, buf);
                    worst_ratio = std::max(worst_ratio, r.max_dow_ratio);
                    runs++;

                    RunResult ro = run_slew(A, step, x0, v0f * v_safe, 3000, false);
                    if(!(ro.converged && !ro.bad && ro.crossings <= 3)) {
                        old_fail++;
                    }
                }
            }
        }
    }
    printf("   (%d configurations; per-tick form failed %d of them)\n",
           runs, old_fail);

    // Non-vacuous guard: the same harness must show the per-tick variant
    // failing in at least one configuration of this grid -- otherwise the
    // harness could not distinguish the two forms. (Empirically it fails
    // the whole a = 5e-1 row plus the 1000x-warp stiff cases: 22 of 288.)
    CHECK_TRUE(old_fail > 0,
               "per-tick law must fail in at least one configuration "
               "(if this passes, the harness cannot catch the limit-cycle)");

    // Authority bound (invariant 3): the commanded domega per substep
    // never exceeds alpha*h, i.e. the autopilot never commands more than
    // a maxed manual stick (full rated torque).
    CHECK_TRUE(worst_ratio <= 1.0 + 1e-9,
               "commanded domega must never exceed the wheel authority "
               "alpha*h (autopilot <= maxed manual stick)");
}

/*
 * 4. KillRot: bounded monotonic decay, no sign-flip jitter, all warps.
 */
static void test_killrot_law() {
    printf("== KillRot: bounded decay, no sign-flip jitter ==\n");
    const double A_grid[] = {0.01, 0.1, 1.0};
    const double step_grid[] = {0.02, 0.2, 2.0, 20.0};
    const double v0_grid[] = {1.0, 10.0};
    for(double A : A_grid) {
        for(double step : step_grid) {
            for(double v0 : v0_grid) {
                // |v| drops by min(|v|, A/n) per substep: ~|v0|/A substeps
                // = (|v0|/A)/n ticks; give 2x margin + slack.
                const int budget = 2 * (int)(std::fabs(v0) / A) + 20;
                char buf[160];
                snprintf(buf, sizeof buf,
                         "KillRot A=%.2f step=%.2f v0=%.1f converges without flip",
                         A, step, v0);
                CHECK_TRUE(killrot_converges(A, step, v0, budget), buf);
            }
        }
    }
}

/*
 * 5. getInertiaDiag + GetAngVelocity: the REAL functions from
 * src/physics.cpp -- the readers the rotation law depends on.
 */
static void test_inertia_and_angvel_readers() {
    printf("== getInertiaDiag / GetAngVelocity: real Bullet readers ==\n");

    btBoxShape shape(btVector3(1.0, 1.0, 1.0));
    const double m0 = 4.0;
    const double m1 = m0 / 2.0;

    btVector3 I0;
    shape.calculateLocalInertia(m0, I0);
    if(I0.getX() <= 0.0) {
        printf("SKIP: shape inertia is degenerate\n");
        return;
    }

    {
        btRigidBody::btRigidBodyConstructionInfo ci(m0, 0, &shape, I0);
        btRigidBody *rb = new btRigidBody(ci);

        Body b;
        b.model = nullptr;   // no GL model in a headless test
        b.btBody = rb;
        b.mass = m0;

        // Reads back exactly the shape-derived inertia (a unit box is
        // 2m/3 about every axis) -- not a default/identity tensor.
        const glm::dvec3 I = getInertiaDiag(&b);
        CHECK_NEAR(I.x, I0.getX(), 1e-9, "getInertiaDiag x == shape I_x");
        CHECK_NEAR(I.y, I0.getY(), 1e-9, "getInertiaDiag y == shape I_y");
        CHECK_NEAR(I.z, I0.getZ(), 1e-9, "getInertiaDiag z == shape I_z");
        CHECK_TRUE(!(I == glm::dvec3(1.0, 1.0, 1.0)),
                   "inertia must not be the identity tensor");

        // SetMass recomputes the inertia from the shape; the reader must
        // see the new (halved) values, still not the identity.
        SetMass(&b, m1);
        const glm::dvec3 Ih = getInertiaDiag(&b);
        CHECK_NEAR(Ih.x, 0.5 * I0.getX(), 1e-9, "I_x(m/2) == 0.5 * I_x(m)");
        CHECK_NEAR(Ih.y, 0.5 * I0.getY(), 1e-9, "I_y(m/2) == 0.5 * I_y(m)");
        CHECK_NEAR(Ih.z, 0.5 * I0.getZ(), 1e-9, "I_z(m/2) == 0.5 * I_z(m)");

        // The law reads the body's angular velocity every substep; the
        // reader must round-trip exactly.
        rb->setAngularVelocity(btVector3(1.0, 2.0, 3.0));
        const glm::dvec3 w = GetAngVelocity(&b);
        CHECK_NEAR(w.x, 1.0, 1e-9, "GetAngVelocity x round-trip");
        CHECK_NEAR(w.y, 2.0, 1e-9, "GetAngVelocity y round-trip");
        CHECK_NEAR(w.z, 3.0, 1e-9, "GetAngVelocity z round-trip");
    } // b's destructor deletes rb
}

/*
 * 6. Torque delivery, on a real Bullet world (mirrors test_thrust.cpp's
 * delivered_velocity): the torque must be re-applied before EVERY
 * substep; applied once per tick it only acts during the first substep,
 * cutting the delivered domega to 1/n.
 *
 * Mirrors the src/main.cpp tick structure:
 *   step = dt * time_accel
 *   n    = max(3, round(step / 0.1))
 *   h    = step / n
 *   for t in ticks:
 *       [old: apply tau once]
 *       for i in n:
 *           [new: apply tau each substep]
 *           stepSimulation(h)
 *
 * Returns the final angular velocity along the torque axis.
 */
static double delivered_angular_velocity(double tau, double m, double time_accel,
                                         bool per_substep, int ticks) {
    btDefaultCollisionConfiguration conf;
    btCollisionDispatcher dispatcher(&conf);
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &conf);
    world.setGravity(btVector3(0, 0, 0));

    btBoxShape shape(btVector3(1.0, 1.0, 1.0));
    btVector3 I;
    shape.calculateLocalInertia(m, I);
    btRigidBody::btRigidBodyConstructionInfo ci(m, 0, &shape, I);
    btRigidBody *rb = new btRigidBody(ci);
    world.addRigidBody(rb);

    const double dt = 1.0 / 50.0;
    const double step = dt * time_accel;
    const int n = main_loop_n(step);
    const double h = step / n;

    for(int t = 0; t < ticks; t++) {
        if(!per_substep) {
            rb->applyTorque(btVector3(0, 0, tau)); // old: once per tick
        }
        for(int i = 0; i < n; i++) {
            if(per_substep) {
                rb->applyTorque(btVector3(0, 0, tau)); // new: every substep
            }
            world.stepSimulation((float)h, 1, (float)h);
        }
    }

    const double w = rb->getAngularVelocity().getZ();
    world.removeRigidBody(rb);
    delete rb;
    return w;
}

static void test_torque_delivery() {
    printf("== Torque delivery: torque re-applied per substep ==\n");

    const double tau = 2000.0; // N m, the wheel rating
    const double m = 4.5;

    struct Case {
        double ta;   // time acceleration
        int ticks;   // physics ticks (50/ta per simulated second)
        int n;       // substeps per tick, per the main-loop formula
    };
    const Case cases[] = {
        {1.0,  50, 3},  // 1.0 s of sim time
        {10.0, 5,  3},  // 1.0 s of sim time
        {100.0, 1,  20}, // 1.0 s of sim time
    };

    for(const Case &c : cases) {
        const double T = c.ticks * (1.0 / 50.0) * c.ta; // simulated seconds
        const double I = (2.0 / 3.0) * m;               // unit box
        const double w_full = tau / I * T;

        const double w_new = delivered_angular_velocity(tau, m, c.ta, true, c.ticks);
        const double w_old = delivered_angular_velocity(tau, m, c.ta, false, c.ticks);

        char buf[192];
        snprintf(buf, sizeof buf,
                 "NEW pattern ta=%g: delivered w == tau/I*T (%.1f rad/s)",
                 c.ta, w_full);
        CHECK_NEAR(w_new, w_full, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "OLD pattern ta=%g: delivered w == tau/I*T/n (%.1f rad/s) "
                 "[harness must see the 1/n loss]", c.ta, w_full / c.n);
        CHECK_NEAR(w_old, w_full / c.n, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "ta=%g: new pattern delivers the FULL torque (old lost %d-1 of %d substeps)",
                 c.ta, c.n - 1, c.n);
        CHECK_TRUE(w_new > 0.99 * w_full && w_old < 0.5 * w_full, buf);
    }
}

int main() {
    test_slew_law();
    printf("\n");
    test_killrot_law();
    printf("\n");
    test_inertia_and_angvel_readers();
    printf("\n");
    test_torque_delivery();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
