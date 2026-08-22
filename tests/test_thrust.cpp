//
// Regression tests for the three thrust-calculation fixes (2026-08-22):
//
//   1. Substep thrust delivery (src/main.cpp tick loop + Vehicle::applyThrustForce)
//      Bullet clears all accumulated forces on every stepSimulation, so the
//      thrust must be re-applied before EVERY substep (like gravity). Previously
//      it was applied once per tick and only acted during the first substep,
//      cutting the delivered thrust to 1/n (n = substeps per tick: n=3 at 1x
//      warp, n grows with time acceleration, so it got worse at warp).
//      Pinned here against a REAL Bullet world:
//        - new pattern (force per substep): delivered dv ~= (F/m)*T for any n
//        - old pattern (force once per tick): delivered dv ~= (F/m)*T/n -- the
//          harness MUST catch the old bug (vacuous-test guard, as in
//          test_attitude.cpp).
//
//   2. Fuel flow (src/main.cpp Vehicle::ApplyThrust / consumeResourceMass)
//      A tick's fuel = (kg/s) x the tick's simulated time. Ticks per simulated
//      second = 50/time_accel, so the burn rate per simulated second must equal
//      the nominal kg/s at ANY warp. Previously the per-call amount was divided
//      by a hardcoded 60 ("since fps = 60") while the logic tick runs at 50 Hz,
//      so the ship burned 50/60 of the nominal rate -- inconsistent with the
//      thrust model (809.8 N = 0.02 kg/s x 40492 m/s, both propellants).
//
//   3. SetMass inertia (src/physics.cpp) -- tests the REAL function.
//      setMassProps(mass, I) uses I AS-IS. Previously a fixed btVector3(1,1,1)
//      was passed, so every fuel consumption silently reset the part's moment
//      of inertia to the identity tensor, destroying the mesh-derived inertia
//      set at registration. Now the inertia is recomputed from the collision
//      shape (as RegisterObject does). Pinned: inertia must scale linearly
//      with mass for a fixed shape, and must NOT be the identity.
//
// Thrust-model constants: the standard ship (see test_attitude.cpp):
//   thrust 809.8 N = 0.02 kg/s (H2 + LOX) x 40492 m/s,
//   2.0 kg propellant (1.0 H2 + 1.0 LOX), dry mass 4.5 kg,
//   logic tick dt = 1/50 s (src/main.cpp). The F/mdot below are harness
//   constants for the delivery-pattern tests -- any F works there.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

// OSP runs Bullet on the double-precision path (physics.cpp asserts
// sizeof(btScalar) == 8). The macro must be set before ANY bullet header,
// as src/physics.cpp and src/body.h do.
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
 * 3. SetMass: the real function, from src/physics.cpp.
 */
static void test_setmass_inertia() {
    printf("== SetMass: inertia follows the mass (not the identity) ==\n");

    btBoxShape shape(btVector3(1.0, 1.0, 1.0));
    // unit box: I = 2m/3, so m0 = 4.0 keeps I0 and I1 away from (1,1,1)
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

        SetMass(&b, m1);

        // read the (inverse) local inertia back out of the rigid body
        const btVector3 inv = rb->getInvInertiaDiagLocal();
        const btVector3 I1(1.0 / inv.getX(), 1.0 / inv.getY(), 1.0 / inv.getZ());

        // A fixed shape's inertia scales LINEARLY with its mass.
        CHECK_NEAR(I1.getX(), 0.5 * I0.getX(), 1e-9, "I_x(1.5 kg) == 0.5 * I_x(3 kg)");
        CHECK_NEAR(I1.getY(), 0.5 * I0.getY(), 1e-9, "I_y(1.5 kg) == 0.5 * I_y(3 kg)");
        CHECK_NEAR(I1.getZ(), 0.5 * I0.getZ(), 1e-9, "I_z(1.5 kg) == 0.5 * I_z(3 kg)");

        // And it must NOT be the identity tensor (the old bug).
        CHECK_TRUE(!(I1 == btVector3(1, 1, 1)),
                   "inertia after SetMass must not be the identity tensor");

        // b's destructor deletes rb.
    }

    // Harness guard: the OLD code (setMassProps(m, (1,1,1))) leaves the
    // inertia exactly (1,1,1) -- if that holds, this test can tell the
    // old implementation from the new one (not vacuous).
    {
        btRigidBody::btRigidBodyConstructionInfo ci(m0, 0, &shape, I0);
        btRigidBody *rb = new btRigidBody(ci);
        rb->setMassProps(m1, btVector3(1, 1, 1));
        const btVector3 inv = rb->getInvInertiaDiagLocal();
        const btVector3 Iold(1.0 / inv.getX(), 1.0 / inv.getY(), 1.0 / inv.getZ());
        CHECK_TRUE(Iold == btVector3(1, 1, 1),
                   "harness: old setMassProps pattern must leave inertia (1,1,1)");
        delete rb;
    }
}

/*
 * 1. Substep thrust delivery, on a real Bullet world.
 *
 * Mirrors the src/main.cpp tick structure:
 *   step = dt * time_accel
 *   n    = max(3, round(step / 0.1))
 *   h    = step / n
 *   for t in ticks:
 *       [old: apply F once]
 *       for i in n:
 *           [new: apply F each substep]
 *           stepSimulation(h)
 *
 * Returns the final velocity along the thrust axis.
 */
static double delivered_velocity(double F, double m, double time_accel,
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
    const double kMaxSubStep = 0.1;
    int n = 3;
    int need = (int)(step / kMaxSubStep + 0.5);
    if(need > n) { n = need; }
    const double h = step / n;

    for(int t = 0; t < ticks; t++) {
        if(!per_substep) {
            rb->applyCentralForce(btVector3(0, 0, F)); // old: once per tick
        }
        for(int i = 0; i < n; i++) {
            if(per_substep) {
                rb->applyCentralForce(btVector3(0, 0, F)); // new: every substep
            }
            world.stepSimulation((float)h, 1, (float)h);
        }
    }

    const double v = rb->getLinearVelocity().getZ();
    world.removeRigidBody(rb);
    delete rb;
    return v;
}

static void test_substep_delivery() {
    printf("== Thrust delivery: force re-applied per substep ==\n");

    // Standard-ship numbers (test_attitude.cpp): 404.9 N on 4.5 kg dry.
    const double F = 404.92;
    const double m = 4.5;

    struct Case {
        double ta;      // time acceleration
        int ticks;      // physics ticks (50/ta per simulated second)
        int n;          // substeps per tick, per the main-loop formula
    };
    const Case cases[] = {
        {1.0,  50, 3},  // 1.0 s of sim time
        {10.0, 5,  3},  // 1.0 s of sim time
        {25.0, 2,  5},  // 1.0 s of sim time
    };

    for(const Case &c : cases) {
        const double T = c.ticks * (1.0 / 50.0) * c.ta; // simulated seconds
        const double v_full = F / m * T;

        const double v_new = delivered_velocity(F, m, c.ta, true, c.ticks);
        const double v_old = delivered_velocity(F, m, c.ta, false, c.ticks);

        char buf[192];
        snprintf(buf, sizeof buf,
                 "NEW pattern ta=%g: delivered v == F/m*T (%.1f m/s)", c.ta, v_full);
        CHECK_NEAR(v_new, v_full, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "OLD pattern ta=%g: delivered v == F/m*T/n (%.1f m/s) "
                 "[harness must see the 1/n loss]", c.ta, v_full / c.n);
        CHECK_NEAR(v_old, v_full / c.n, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "ta=%g: new pattern delivers the FULL thrust (old lost %d-1 of %d substeps)",
                 c.ta, c.n - 1, c.n);
        CHECK_TRUE(v_new > 0.99 * v_full && v_old < 0.5 * v_full, buf);
    }
}

/*
 * 2. Fuel flow: burn per simulated second must equal the nominal kg/s at
 * any time acceleration (mirrors Vehicle::ApplyThrust' per-tick flow).
 */
static double fuel_per_simsec(double mdot, double time_accel, bool new_model) {
    const double dt = 1.0 / 50.0;
    const double ticks_per_simsec = 50.0 / time_accel;
    const double per_tick = new_model
        ? mdot * (dt * time_accel)  // new: kg/s x tick's simulated time
        : mdot / 60.0;             // old: hardcoded "/60 (since fps = 60)"
    return ticks_per_simsec * per_tick;
}

static void test_fuel_flow() {
    printf("== Fuel flow: nominal kg/s at any warp ==\n");

    const double mdot = 0.01; // kg/s per tank (GetMaxFuelRate)
    const double ta_grid[] = {1.0, 5.0, 10.0, 25.0, 100.0};

    for(double ta : ta_grid) {
        char buf[160];
        snprintf(buf, sizeof buf,
                 "NEW model ta=%g: burn == nominal %.3f kg/s (warp-independent)", ta, mdot);
        CHECK_NEAR(fuel_per_simsec(mdot, ta, true), mdot, 1e-9, buf);

        // OLD model: per-tick was mdot/60, tick rate per sim-sec was 50/ta,
        // so burn per sim-sec = mdot*50/(60*ta) -- WRONG on two counts: the
        // 50/60 factor (logic tick is 50 Hz, not 60) AND it degrades with warp.
        const double old_expect = mdot * 50.0 / (60.0 * ta);
        snprintf(buf, sizeof buf,
                 "OLD model ta=%g: burn == %.4g kg/s [harness must see the loss + warp leak]",
                 ta, old_expect);
        CHECK_NEAR(fuel_per_simsec(mdot, ta, false), old_expect, 1e-9, buf);
        // The two old-model defects, made explicit:
        CHECK_TRUE(fuel_per_simsec(mdot, ta, false) < mdot,
                   "OLD model burns LESS than nominal (was inconsistent)");
    }
}

int main() {
    test_setmass_inertia();
    printf("\n");
    test_substep_delivery();
    printf("\n");
    test_fuel_flow();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
