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
//   2. Fuel flow (src/vehicle.cpp Vehicle::ApplyThrust / consumeResourceMass)
//      A tick's fuel = (kg/s) x the tick's simulated time. Ticks per simulated
//      second = physics_hz/time_accel, so the burn rate per simulated second
//      equals the nominal kg/s at ANY warp AND at any --physics-hz. Previously
//      the per-call amount was divided by a hardcoded 60 ("since fps = 60")
//      while the logic tick then ran at 50 Hz, so the ship burned 50/60 of the
//      nominal rate -- inconsistent with the thrust model (49984 N = 11.36 kg/s
//      x 4400 m/s, both propellants).
//
//   3. getInertiaDiag (src/physics.cpp) -- tests the REAL function. A part has
//      no rigid body, so its inertia comes from its shape at the Body's mass.
//      Previously a fixed btVector3(1,1,1)
//      was passed, so every fuel consumption silently reset the part's moment
//      of inertia to the identity tensor, destroying the mesh-derived inertia
//      set at registration. Now the inertia is recomputed from the collision
//      shape (as RegisterObject does). Pinned: inertia must scale linearly
//      with mass for a fixed shape, and must NOT be the identity.
//
// Thrust-model constants: the standard ship (see test_attitude.cpp):
//   thrust 49984 N = 11.36 kg/s (H2 + LOX, 5.68 each) x 4400 m/s,
//   835.66 kg propellant (417.83 H2 + 417.83 LOX), dry mass 1833.92 kg,
//   logic tick dt = 1/--physics-hz (default 1/60 s). The F/mdot below are
//   harness constants for the delivery-pattern tests -- any F works there.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

// OSP runs Bullet on the double-precision path (physics.cpp asserts
// sizeof(btScalar) == 8) -- btcommon.h settles that macro before the
// include, exactly as src/physics.cpp does.
#include "btcommon.h"

#include "body.h"
#include "cli.h"      // GameArgs (the shipped --physics-hz default)
#include "physics.h"
#include "timestep.h"  // substepCount() -- the tick loop's substep plan

#include <deque>

#include "vehicle.h"  // #400: Vehicle::producedThrust / availableThrust / availableAccel
#include "terrain.h"  // and the body whose air a jet breathes
#include <glm/gtc/constants.hpp>  // glm::radians, for an engine with a gimbal cone

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
        /* Relative, except at an expectation of exactly zero (#510):              \
           rel_tol * 0.0 would demand bit-exactness, which is not what a           \
           caller writing 1e-12 means. 0.0 AS a tolerance stays exact, so          \
           a bit-identical pin keeps its teeth. */                                 \
        const double _tol = (_e == 0.0) ? rel_tol                                  \
                                        : rel_tol * std::fabs(_e);                 \
        if (!std::isfinite(_a) || !std::isfinite(_e)                              \
            || std::fabs(_a - _e) > _tol) {                                       \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);           \
        }                                                                      \
    } while (0)

/*
 * 3. getInertiaDiag: the real function, from src/physics.cpp. A ship part has
 *    no rigid body of its own any more, so this reads the SHAPE at the Body's
 *    mass -- and it is the independent reference that Vehicle::
 *    checkCompoundInvariants holds the compound's child inertias against.
 *    (SetMass, which this section used to test, is gone: a ship's mass follows
 *    from its parts through rebuildCompound, so nothing sets the mass of a
 *    registered body at runtime.)
 */
static void test_inertia_diag() {
    printf("== getInertiaDiag: inertia follows the mass (not the identity) ==\n");

    // heap shape: ~Body frees it (a stack one would be `delete`d out from
    // under the stack)
    btBoxShape *shape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
    // unit box: I = 2m/3, so m0 = 4.0 keeps I0 and I1 away from (1,1,1)
    const double m0 = 4.0;

    btVector3 I0;
    shape->calculateLocalInertia(m0, I0);
    if(I0.getX() <= 0.0) {
        printf("SKIP: shape inertia is degenerate\n");
        delete shape;
        return;
    }

    Body b;
    // mesh/shader/texture default to null (a headless test has no GL)
    b.btBody = nullptr;  // a part is not a simulated object of its own
    b.shape = shape;     // ... but it does have a collision hull (owned)
    b.mass = m0;

    const glm::dvec3 I = getInertiaDiag(&b);
    CHECK_NEAR(I.x, I0.getX(), 1e-9, "I_x(m) == the shape's I_x");
    CHECK_NEAR(I.y, I0.getY(), 1e-9, "I_y(m) == the shape's I_y");
    CHECK_NEAR(I.z, I0.getZ(), 1e-9, "I_z(m) == the shape's I_z");
    CHECK_TRUE(!(I == glm::dvec3(1.0, 1.0, 1.0)),
               "inertia must not be the identity tensor");

    /* A fixed shape's inertia scales LINEARLY with its mass, and it must track
       Body::mass -- the only mass a part has now. This is the check a fixed
       btVector3(1,1,1) inertia (the old SetMass bug) fails. */
    b.mass = m0 / 2.0;
    const glm::dvec3 Ih = getInertiaDiag(&b);
    CHECK_NEAR(Ih.x, 0.5 * I0.getX(), 1e-9, "I_x(m/2) == 0.5 * I_x(m)");
    CHECK_NEAR(Ih.y, 0.5 * I0.getY(), 1e-9, "I_y(m/2) == 0.5 * I_y(m)");
    CHECK_NEAR(Ih.z, 0.5 * I0.getZ(), 1e-9, "I_z(m/2) == 0.5 * I_z(m)");
    CHECK_TRUE(!(Ih == glm::dvec3(1.0, 1.0, 1.0)),
               "inertia at half the mass must not be the identity tensor");

    /* A massless body keeps the (1,1,1) placeholder RegisterObject gives it,
       rather than a degenerate zero tensor that would make it un-rotatable. */
    b.mass = 0.0;
    CHECK_TRUE(getInertiaDiag(&b) == glm::dvec3(1.0, 1.0, 1.0),
               "massless body: the (1,1,1) placeholder, not a zero tensor");
}

/*
 * 1. Substep thrust delivery, on a real Bullet world.
 *
 * Mirrors the src/tick.cpp structure:
 *   step = dt * time_accel
 *   n    = substepCount(step)
 *   h    = step / n
 *   for t in ticks:
 *       [old: apply F once]
 *       for i in n:
 *           [new: apply F each substep]
 *           stepSimulation(h)
 *
 * Returns the final velocity along the thrust axis.
 */
static double delivered_velocity(double F, double m, double dt,
                                 double time_accel, bool per_substep,
                                 int ticks) {
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

    const double step = dt * time_accel;
    const int n = substepCount(step);
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

    // Standard-ship numbers (test_attitude.cpp): 49984 N on 1833.92 kg dry.
    const double F = 49984.0;
    const double m = 1833.92;

    /* Model the tick rate the game actually runs (--physics-hz default) and
       derive the tick and substep counts from it, so the case list stays
       correct when the rate changes instead of pinning one rate's numbers. */
    const double tick_hz = GameArgs{}.physics_hz;
    const double dt = 1.0 / tick_hz;

    const double ta_grid[] = {1.0, 10.0, 25.0};
    for(double ta : ta_grid) {
        const int ticks = (int)(tick_hz / ta + 0.5);  // ticks covering ~1 s
        const int n = substepCount(dt * ta);          // substeps per tick
        const double T = ticks * dt * ta;             // simulated seconds
        const double v_full = F / m * T;

        const double v_new = delivered_velocity(F, m, dt, ta, true, ticks);
        const double v_old = delivered_velocity(F, m, dt, ta, false, ticks);

        char buf[192];
        snprintf(buf, sizeof buf,
                 "NEW pattern ta=%g: delivered v == F/m*T (%.1f m/s)", ta, v_full);
        CHECK_NEAR(v_new, v_full, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "OLD pattern ta=%g: delivered v == F/m*T/n (%.1f m/s) "
                 "[harness must see the 1/n loss]", ta, v_full / n);
        CHECK_NEAR(v_old, v_full / n, 1e-3, buf);

        snprintf(buf, sizeof buf,
                 "ta=%g: new pattern delivers the FULL thrust (old lost %d-1 of %d substeps)",
                 ta, n - 1, n);
        CHECK_TRUE(v_new > 0.99 * v_full && v_old < 0.5 * v_full, buf);
    }
}

/*
 * 2. Fuel flow: burn per simulated second must equal the nominal kg/s at
 * any time acceleration (mirrors Vehicle::ApplyThrust' per-tick flow).
 */
static double fuel_per_simsec(double mdot, double time_accel, bool new_model) {
    /* The NEW model is rate-agnostic: ticks/s x the tick's simulated time is
       exactly kg/s at any --physics-hz. The OLD model is pinned to the 50 Hz
       tick it happened at -- the only rate where the hardcoded /60 leaves the
       50/60 shortfall the harness must catch. */
    const double hz = new_model ? GameArgs{}.physics_hz : 50.0;
    const double dt = 1.0 / hz;
    const double ticks_per_simsec = hz / time_accel;
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

/*
 * 4. THE TWO THRUST NUMBERS (#400). producedThrust() is the engine command read back
 *    (rated x throttle x the air as it is); availableThrust() is what the ignited stage
 *    COULD do at full throttle in the air it actually has; availableAccel() is that over
 *    the current mass. Guidance needs the last two: the prototype in
 *    reports/autopilots2026_10_24 sized a circularization burn with a_max =
 *    getThrust()/mass while coasting, read its own zero throttle every tick, took its own
 *    "coast" branch and never reignited -- a burn that silently never happened.
 *
 * Hand-built so every number is hand-checkable. Pod 1000 kg; a 20000 N engine (5 kg/s x
 * 4000 m/s) at STAGE 3; a 50000 N engine (10 x 5000) at stage 1; a jet at stage 1 (fan
 * 100 N, 1 kg/s at ve 1000, intake 0.04 m^2). Total 1250 kg. The ship sits 1000 m above
 * the body's sea level and is not moving, so rho = E = exp(-Z/8000) of sea level, where
 * Z is that altitude's geopotential depth in this 600 km body's own field -- 998.3 m,
 * not 1000 m (#683). The jet's ram term is zero; no tank is modelled because these
 * reads are POTENTIALS -- they never consult the fuel.
 *   stage 3 is the first live stage, so only the 20000 N engine is available at first,
 *   and that number does not care about air at all;
 *   the jet at rest in that air is E*(fan + mdot*ve) = 1100E, and at its best speed
 *   (v = ve/2, where drag.h's ram term tops out) it is E*1100 + rho*A*(ve/2)^2 =
 *   1100E + 10000E = 11100E; in vacuum, neither -- air is what it breathes.
 * Since #630 those two terms are NOT billed by the same share of the air. The fan and
 * the fuel follow jetAirGate, which divides by the machine's RATING intake (1.225 kg/m3,
 * constants.h) rather than by this body's own sea level: this rig's 1.0 kg/m3 datum is
 * 0.816 of a rated intake, so the fan term is 1100E/1.225 = 898E. The ram term is the
 * air the intake actually captures -- raw rho, which here is E kg/m3 -- and it is paid
 * for by jetOxGate, 1.0 for a body that authors no composition (dry air). Both halves
 * are hand-checkable in the same way; they just no longer move together, which is what
 * a CO2 world looks like (test_jet.cpp pins that case).
 */
struct ThrustRig {
    std::deque<PartDef> defs;
    Vehicle *v = nullptr;
};

/* A part for this rig: `rate` x `ve` is the rated thrust, and rate 0 (the pod) simply
   means it is not a thruster. `jet` makes it air-breathing instead, and `gimbal_max`
   (rad) is how far its nozzle may swing off the mount -- 0 means locked. `h2_only`
   makes a rocket burn hydrogen alone (same rated thrust, one propellant instead of
   two), which is how the canBurn cases below build an engine a tank can half-feed.
   `barrier` walls the part off from the fuel graph (Part::isFuelBarrier -> no fuel
   group, so no drain layers at all). */
static Part *mkRigPart(std::deque<PartDef> &defs, const char *name, double mass,
                       double rate, double ve, int stage, bool jet = false,
                       double gimbal_max = 0.0, bool h2_only = false,
                       bool barrier = false) {
    Body *b = new Body;
    b->btBody = nullptr;
    b->shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    b->mass = mass;
    defs.push_back(PartDef());
    PartDef &d = defs.back();
    d.name = name;
    d.mass = (float)mass;
    d.exhaust_velocity = ve;
    d.gimbal_max = gimbal_max;
    d.fuel_barrier = barrier;
    if(jet) {
        d.jet = true;
        d.setPropellantRate(ResourceType::JetFuel, rate);
        d.jet_fan_thrust = 100.0;
        d.jet_intake_area = 0.04;
    } else if(h2_only) {
        d.setPropellantRate(ResourceType::Hydrogen, rate);
    } else {
        d.setPropellantRate(ResourceType::Hydrogen, rate * 0.5);
        d.setPropellantRate(ResourceType::LOX, rate * 0.5);
    }
    d.synthesizeNodes();
    Part *p = new Part;
    p->body = b;
    p->def = &d;
    p->stage = stage;
    return p;
}

/* A body whose sea level is its own surface, so a ship at `radius` + 1000 sits in 1000 m
   of air -- or in nothing, if rho_sea is 0. */
static TerrainBody *mkPressureBody(double rho_sea) {
    TerrainBody *body = new TerrainBody;
    body->name = "thrustball";
    body->radius = 600000.0f;
    body->g = 9.81;
    /* dragAtm weighs the air in mu at the datum sphere, not in g (#708), so a
       hand-built body carries the mu the loader would have derived. */
    body->mu = body->g * (double)body->radius * (double)body->radius;
    body->surface.sea_level = 0.0f;
    // hand-built: a FILE authors sea_level_pressure and the loader resolves this (#739)
    body->surface.atmosphere.sea_level_density = (float)rho_sea;
    body->surface.atmosphere.scale_height = 8000.0;
    return body;
}

static void test_produced_and_available_thrust() {
    printf("== producedThrust / availableThrust / availableAccel (#400) ==\n");
    const double ALT = 1000.0;
    double E = 0.0;    // rho/rho_sea where the ship actually sits; set after placing
    TerrainBody *body = mkPressureBody(1.0);
    ThrustRig r;
    r.v = new Vehicle;
    r.v->name = "twothrusts";
    Part *pod = mkRigPart(r.defs, "pod", 1000.0, 0.0, 0.0, 1);
    r.v->setRoot(pod);
    Part *eng3 = mkRigPart(r.defs, "engine3", 100.0, 5.0, 4000.0, 3);
    r.v->attachMode(eng3, 0, AttachMode::Down, 0.0);
    Part *eng1 = mkRigPart(r.defs, "engine1", 100.0, 10.0, 5000.0, 1);
    r.v->attachMode(eng1, 0, AttachMode::Down, 0.0);
    Part *jet = mkRigPart(r.defs, "jet", 50.0, 1.0, 1000.0, 1, /*jet=*/true);
    r.v->attachMode(jet, 0, AttachMode::Down, 0.0);
    r.v->init();
    CHECK_TRUE(r.v->numStages() >= 3, "the rig spans three stages");
    CHECK_TRUE(r.v->activeStage() == 3, "and the counter starts at the top, so stage 3"
               " is what is live to begin with");

    r.v->m_parent = body;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 600000.0 + ALT), glm::dmat3(1.0));
    /* The air the model reads is the air at the COM, and a compound's COM is not its
       transform origin (the engines hang below it), so the altitude -- and therefore the
       air fraction E every jet number below uses -- is MEASURED here rather than assumed
       to be ALT. The COM's own geometry is test_inertia's business.
       The column is weighed in THIS body's 1/r^2 field (#683), so E decays with the
       geopotential depth R z/(R+z): 998.34 m of column at 1000 m up a 600 km ball,
       which makes the air 0.021 % thicker than the old uniform-g reading. */
    const double zc = glm::length(r.v->get_center_of_mass()) - 600000.0;
    E = std::exp(-(600000.0 * zc / (600000.0 + zc)) / 8000.0);

    /* COASTING: the whole point of #400. The throttle is zero, so the read-back is zero,
       and a law that plans with it has no way to know the engine is fine. */
    r.v->setThrottle(0.0);
    CHECK_NEAR(r.v->producedThrust(), 0.0, 0.0, "nothing produced at zero throttle");
    CHECK_NEAR(r.v->availableThrust(), 20000.0, 1e-12,
               "20000 N available anyway: the stage-3 engine at full throttle, and a"
               " rocket does not care that there is air here");
    CHECK_NEAR(r.v->availableAccel(), 16.0, 1e-12,
               "over 1250 kg that is 16 m/s^2 of a_max -- the number a burn plan needs"
               " while the throttle is still at zero");

    /* The two numbers move differently, and that is what makes them two. */
    r.v->setThrottle(0.5);
    CHECK_NEAR(r.v->producedThrust(), 10000.0, 1e-12, "half a throttle, half the"
               " production -- of the live stage only (the stage-1 engines and the jet"
               " are gated by the stage counter)");
    CHECK_NEAR(r.v->availableThrust(), 20000.0, 1e-12, "available does not move with"
               " the throttle");
    CHECK_NEAR(r.v->availableAccel(), 16.0, 1e-12, "so a_max is stable while the law"
               " writes the throttle -- no feedback through its own output (#400's"
               " flip-flop)");
    CHECK_NEAR(r.v->getFullThrustTWR() / r.v->getTWR(), 2.0, 1e-12,
               "the Vessel window's current-over-full TWR pair is produced-over-"
               "available, not two spellings of one number");

    /* Stage down: the rest becomes available, the jet included. */
    r.v->advanceStage();
    r.v->advanceStage();
    CHECK_TRUE(r.v->activeStage() == 1, "the counter grinds down one press at a time");
    CHECK_NEAR(r.v->availableThrust(),
               70000.0 + 1100.0 * E / kAirSeaLevelDensity + 10000.0 * E, 1e-9,
               "20000 + 50000 + the jet at ITS BEST SPEED in THIS air: (100 + 1*1000)"
               " x E/1.225 (the rating intake, #630) + rho*0.04*500*500, with rho = E");
    CHECK_NEAR(r.v->producedThrust(),
               0.5 * (70000.0 + 1100.0 * E / kAirSeaLevelDensity), 1e-9,
               "at half throttle the jet is asked for what it gives standing still"
               " (fan + mdot*ve, gated by the air against its 1.225 kg/m3 rating), because"
               " producedThrust uses the speed the ship actually has");
    CHECK_NEAR(r.v->availableAccel(),
               (70000.0 + 1100.0 * E / kAirSeaLevelDensity + 10000.0 * E) / 1250.0, 1e-9,
               "and the a_max that stage-down just bought, at the same mass");
    /* #607: an ask up the stack costs nothing, and it must cost nothing for THIS rig
       too -- three engines, one of them air-fed, in real air. availableAccelAlong is
       the same loop as availableThrust, so if the directional read scored the jet off,
       forgot --exhaust-scale or divided by anything but the current mass, this pin is
       the one that says so. */
    CHECK_NEAR(r.v->availableAccelAlong(glm::dvec3(0.0, 0.0, 1.0)), r.v->availableAccel(),
               1e-12, "up the stack with the jet and the air in the sum: the two reads"
               " still agree");
    CHECK_NEAR(r.v->availableAccelAlong(glm::dvec3(0.0, 0.0, -1.0)), -r.v->availableAccel(),
               1e-12, "and agree backwards");
    CHECK_TRUE(r.v->availableThrust() > r.v->producedThrust(),
               "at half throttle available still leads produced");

    /* Air is a fuel, not a decoration: in vacuum the jet contributes nothing to either
       number, and the rockets do not notice. */
    TerrainBody *vac = mkPressureBody(0.0);
    r.v->m_parent = vac;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 600000.0 + ALT), glm::dmat3(1.0));
    CHECK_NEAR(r.v->availableThrust(), 70000.0, 1e-12, "the jet's air money is gone in"
               " vacuum: the stage-1 pair is all there is");
    CHECK_NEAR(r.v->producedThrust(), 35000.0, 1e-12, "and the same at half throttle");
    r.v->setThrottle(1.0);
    CHECK_NEAR(r.v->producedThrust(), r.v->availableThrust(), 1e-12,
               "rockets at full throttle: the two agree, because they are the same"
               " engines read with and without the throttle");
    r.v->m_parent = body;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 600000.0 + ALT), glm::dmat3(1.0));
    CHECK_TRUE(r.v->availableThrust() > r.v->producedThrust(),
               "with air back under it, and the ship standing still, available is the"
               " CEILING: the ram term it counts is the one a ship flying at ve/2 would"
               " get, which this one is not. That is the right answer for sizing a burn"
               " and the wrong one for a landing predictor -- hence two numbers.");

    /* The stage gate is the other half of "available": a part below the counter is not
       available at all -- the same rule producedThrust has always applied, and why an
       out-of-sequence stage number makes an engine unreachable (#299's authoring
       bound). */
    r.v->advanceStage();   /* minStage_ is 1, so this does nothing */
    CHECK_TRUE(r.v->activeStage() == 1, "the counter stops at the lowest stage");
    delete r.v;
    delete body;
    delete vac;
}

/*
 * 4b. THE SAME CEILING, POINTED SOMEWHERE (#607). availableAccelAlong(dir) is the
 *     availableAccel() sum again, with each engine weighted by how far its MOUNT and
 *     gimbal cone let it push along dir at all. It exists because availableAccel() has
 *     no direction: a law that asks for sideways or retrograde delta-v reads the full
 *     number and under-runs the burn, and a law with no propellant read at all -- which
 *     is what guidance.cpp has today -- can only dodge that by aiming up the stack.
 *     Rig: the smallest ship that has a directed ceiling, a 1900 kg pod and one 20000 N
 *     engine at the live stage, so every answer is 10 m/s^2 times a share you can do in
 *     your head. The stage gate and the air are noise HERE -- 4c below is the rig that
 *     can see them.
 */
static Vehicle *mkOneEngineRig(std::deque<PartDef> &defs, double cone_deg,
                               double gimbal_deg = 0.0) {
    Vehicle *v = new Vehicle;
    v->name = "along";
    Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
    v->setRoot(pod);
    Part *eng = mkRigPart(defs, "engine3", 100.0, 5.0, 4000.0, 3, false, glm::radians(cone_deg));
    eng->gimbal[0] = glm::radians(gimbal_deg);
    v->attachMode(eng, 0, AttachMode::Down, 0.0);
    v->init();
    v->m_parent = mkPressureBody(1.0);
    v->placeShip(glm::dvec3(0.0, 0.0, 600000.0), glm::dmat3(1.0));
    return v;
}

static void test_available_accel_along() {
    printf("== availableAccelAlong: the ceiling with a direction on it (#607) ==\n");
    std::deque<PartDef> defs;
    Vehicle *v = mkOneEngineRig(defs, 0.0);
    CHECK_TRUE(v->activeStage() == 3, "the 20000 N engine is the live stage");
    const double a_full = 10.0;  // 20000 N over 2000 kg, by hand
    CHECK_NEAR(v->availableAccel(), a_full, 1e-12, "10 m/s^2 straight up the stack");

    /* Straight up the stack is the one ask the two reads MUST agree on: they sum the
       same per-engine ceiling, which is why that ceiling is one function. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 0.0, 1.0)), v->availableAccel(),
               1e-12, "an engine pointed at the ask: availableAccelAlong =="
               " availableAccel -- one per-engine ceiling, two sums");
    /* With no cone the share is a plain cosine, so sideways is nothing and backwards
       is NEGATIVE rather than 0 -- a law that read 0 here would plan a retrograde burn
       as a coast. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1.0, 0.0, 0.0)), 0.0, 1e-12,
               "90 deg off an ungimballed engine: nothing");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 0.0, -1.0)), -a_full, 1e-12,
               "180 deg off: the same authority with the wrong sign");
    /* dir is a direction, not a magnitude: its length must not leak into the answer. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, -300.0, 400.0)), 0.8 * a_full,
               1e-12, "an un-normalised 3-4-5 ask still gets cos(theta) = 0.8");
    /* A zero ask is a real state, not a caller bug: a law's aim is zero until it aims.
       It has to be answered at the door rather than left to the math, because the math
       does not stay NaN on the way out: normalize(0) is NaN, glm::clamp does not rescue
       it, std::max(0.0, NaN) is 0.0, so cos(acos(NaN)) is 1 -- the read would hand the
       law its FULL ceiling along a direction nobody asked for. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0)), 0.0, 0.0,
               "no aim asks for no acceleration");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(std::nan(""), 0.0, 1.0)), 0.0, 0.0,
               "and neither does a NaN aim: the same door the throttle's own NaN guard"
               " uses, because a NaN component takes the same road to 1.0");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1e-320, 0.0, 0.0)), 0.0, 0.0,
               "an ask so small its length underflows to zero is the same case, and"
               " length() > 0.0 catches it with the other two");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(INFINITY, 0.0, 0.0)), 0.0, 0.0,
               "an infinite component is the nastiest of them: length is inf, which is"
               "> 0, and normalize gives inf*(1/inf) = NaN, so it takes the SAME road"
               " to full authority. The isfinite term is what stops it");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1e308, 0.0, 0.0)), 0.0, 0.0,
               "a huge but finite ask overflows length() and reads 0 too: it has a"
               " perfectly good direction, and failing closed is cheaper here than a"
               " scale-safe normalize for a number this game does not produce");

    /* The same engine with a 30 deg gimbal cone: the cone is where the nozzle may
       swing from its mount, so an ask inside it costs nothing and one 90 deg off
       keeps cos(90-30). */
    Vehicle *tv = mkOneEngineRig(defs, 30.0);
    const glm::dvec3 thirty_off(0.0, -std::sin(glm::radians(30.0)),
                                std::cos(glm::radians(30.0)));
    CHECK_NEAR(tv->availableAccelAlong(thirty_off), a_full, 1e-12,
               "an ask exactly gimbal_max off the mount is reachable: the boundary of"
               " the cone costs nothing");
    CHECK_NEAR(tv->availableAccelAlong(glm::dvec3(0.0, -1.0, 0.0)), 0.5 * a_full, 1e-12,
               "sideways with a 30 deg cone: 90 - 30, so cos 60");
    /* And the cone hangs off the MOUNT, not off wherever the steering happens to have
       the nozzle this tick. The engine below is swung +20 deg, so a live-deflection
       read would score the two sideways asks 7.66 and 1.74 -- the read is about the
       hardware's mounting, and a ceiling that moved with the stick would let a law
       size its burn with the player's WASD. */
    Vehicle *sv = mkOneEngineRig(defs, 30.0, 20.0);
    CHECK_NEAR(sv->availableAccelAlong(glm::dvec3(0.0, -1.0, 0.0)), 0.5 * a_full, 1e-12,
               "a swung nozzle still reads its mount: 90 deg off the MOUNT is cos 60"
               " even with the gimbals already 20 deg off centre");
    CHECK_NEAR(sv->availableAccelAlong(glm::dvec3(0.0, 1.0, 0.0)), 0.5 * a_full, 1e-12,
               "and the other side is the same number -- the deflection does not lean"
               " the ceiling one way");
    CHECK_NEAR(sv->availableAccelAlong(glm::dvec3(0.0, 0.0, -1.0)), -a_full
               * std::cos(glm::radians(30.0)), 1e-12,
               "retrograde with a 30 deg cone: 180 - 30 off, and still negative");
    delete sv->m_parent;
    delete sv;
    delete tv->m_parent;
    delete tv;
    delete v->m_parent;
    delete v;
}

/*
 * 4c. TWO ENGINES, TWO MOUNTS, TWO STAGES. One coaxial engine cannot see the stage
 *     gate, the sum, or the per-engine mount: an implementation that scored every
 *     engine against the ROOT's axis, took the max instead of summing, stopped at the
 *     first thruster, or ignored the stage counter passes all of 4b. A 20000 N engine
 *     up the stack at stage 3 and a 5000 N one along the hull's +X at stage 1, over a
 *     2000 kg pod, so the numbers stay in your head: 10 and 2.5 m/s^2.
 */
static void test_available_accel_along_two_mounts() {
    printf("== availableAccelAlong: the sum, the gate and the mount (#607) ==\n");
    std::deque<PartDef> defs;
    Vehicle *v = new Vehicle;
    v->name = "twomount";
    Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
    v->setRoot(pod);
    Part *ax = mkRigPart(defs, "engine3", 50.0, 5.0, 4000.0, 3);
    v->attachMode(ax, 0, AttachMode::Down, 0.0);
    Part *lat = mkRigPart(defs, "engine1", 50.0, 1.0, 5000.0, 1);
    /* Raw attach() because no stack mode expresses this pose: the part's own +Z along
       the hull's +X. dmat3(columns) takes local X, Y, Z in ship-local axes, and these
       three are orthonormal the right way round. */
    v->attach(lat, 0, glm::dvec3(0.0, 2.0, 0.0),
              glm::dmat3(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0),
                         glm::dvec3(1.0, 0.0, 0.0)));
    v->init();
    v->m_parent = mkPressureBody(1.0);
    v->placeShip(glm::dvec3(0.0, 0.0, 600000.0), glm::dmat3(1.0));
    CHECK_TRUE(v->activeStage() == 3, "and it starts with only the stack engine lit");

    /* The stage gate is half of "available": the lateral engine is not available yet,
       in any direction. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 0.0, 1.0)), 10.0, 1e-12,
               "up the stack: the 20000 N engine, over 2000 kg");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1.0, 0.0, 0.0)), 0.0, 1e-12,
               "along the lateral engine's own mount: still nothing, because the stage"
               " counter has not reached it");

    v->advanceStage();
    v->advanceStage();
    CHECK_TRUE(v->activeStage() == 1, "both lit now");
    CHECK_NEAR(v->availableAccel(), 12.5, 1e-12, "25000 N over 2000 kg");
    /* Each engine against its OWN mount, and the two added rather than maxed or taken
       in order. */
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 0.0, 1.0)), 10.0, 1e-12,
               "up the stack: the axial engine at full share and the lateral one at"
               " nothing -- so 10, not 12.5 (every engine on the root's axis) and not"
               " the max");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1.0, 0.0, 0.0)), 2.5, 1e-12,
               "along the lateral mount: the 5000 N engine is what is available there"
               " -- not 0 (first thruster only), not 10 (max)");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(-1.0, 0.0, 0.0)), -2.5, 1e-12,
               "and signed the other way");

    /* The difficulty scale lands on both reads by the same route. */
    v->exhaust_scale = 0.5;
    CHECK_NEAR(v->availableAccel(), 6.25, 1e-12, "--exhaust-scale 0.5 halves the"
               " direction-free number");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1.0, 0.0, 0.0)), 1.25, 1e-12,
               "and the directional one, because it is the same sum divided by the same"
               " mass");
    v->exhaust_scale = 1.0;

    /* And the ask is in frame S -- the same axes partAxis()/GetPos() speak -- not in
       the hull's own axes. Yaw the hull 90 deg about world X: the stack engine now
       points along world Y, the lateral one still along X. A read that compared the
       ask against the ship-local mount would keep answering as if the hull were still
       upright, which is how a law that thinks in a non-rotating frame ends up dividing
       by the wrong ceiling (guidance.cpp rotates its aim for exactly this reason). */
    v->placeShip(glm::dvec3(0.0, 0.0, 600000.0),
                 glm::dmat3(glm::angleAxis(glm::radians(90.0), glm::dvec3(1.0, 0.0, 0.0))));
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, -1.0, 0.0)), 10.0, 1e-12,
               "the world axis the stack mount swung onto (+90 deg about X tips +Z"
               " toward -Y, the same convention test_gimbal pins): full authority");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 1.0, 0.0)), -10.0, 1e-12,
               "and the same number with the wrong sign on the other side of it");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(0.0, 0.0, 1.0)), 0.0, 1e-12,
               "and nothing along the axis it used to point along");
    CHECK_NEAR(v->availableAccelAlong(glm::dvec3(1.0, 0.0, 0.0)), 2.5, 1e-12,
               "the lateral mount did not move under a rotation about its own axis");
    delete v->m_parent;
    delete v;
}

/*
 * 4d. CAN THIS STAGE ACTUALLY BURN (#607 part 2). The three reads above are POTENTIALS:
 *     they rate the ignited stage whatever the tanks say, and that is what a burn plan
 *     and the VAB's rating rows want (test_shipload pins getMaxTWR() staying a rating
 *     with the oxidizer missing). The bug was the missing OTHER answer: a dry rig rates
 *     20000 N and 10 m/s^2 below while ApplyThrust's own sizing
 *     (s = min over propellants of available/desired) sends it to zero, so a throttle law
 *     divides its ask by a number the hull will never deliver and has no way to notice.
 *     canBurn() is that answer, asked per ignited engine over the resources THAT engine
 *     drains -- which is a tighter question than the mask pair guidance.cpp used to keep
 *     for itself, and the last-but-one case below is the difference: a live engine
 *     missing one of two propellants, with a fed engine still staged below it.
 *     These hulls have no body at all: the air is not what is in question here.
 */

/* A tank for these rigs. Capacity lives in the DEF, so init() tops it up, and the fuel
   has mass (Part::effectiveMass counts contents) -- which is why every case below names
   its own hull instead of adding a tank to one that already ran init(). Draining and
   pro-rata layering are test_fuel's business; the only question here is whether a burn
   is possible at all. */
static Part *mkRigTank(std::deque<PartDef> &defs, const char *name, double dry,
                       double h2, double lox, double jet_fuel = 0.0) {
    Body *b = new Body;
    b->btBody = nullptr;
    b->shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    b->mass = dry;
    defs.push_back(PartDef());
    PartDef &d = defs.back();
    d.name = name;
    d.mass = (float)dry;
    d.capacity[(int)ResourceType::Hydrogen] = h2;
    d.capacity[(int)ResourceType::LOX] = lox;
    d.capacity[(int)ResourceType::JetFuel] = jet_fuel;
    d.synthesizeNodes();
    Part *p = new Part;
    p->body = b;
    p->def = &d;
    p->stage = 1;
    return p;
}

static void test_can_burn() {
    printf("== canBurn: the gated twin of the ceiling reads (#607) ==\n");
    {   /* The ticket's probe, made permanent: no tank anywhere on the hull. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "dryprobe";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *eng = mkRigPart(defs, "engine", 100.0, 5.0, 4000.0, 1);
        v->attachMode(eng, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_NEAR(v->availableThrust(), 20000.0, 1e-12,
                   "the potential still rates the engine: it is a RATING, and a plan is"
                   " allowed to ask what this stage could do");
        CHECK_NEAR(v->availableAccel(), 10.0, 1e-12, "20000 N over 2000 kg");
        CHECK_TRUE(!v->canBurn(), "and nothing can burn: there is no propellant on the"
                   " ship. A law that sizes its throttle with the number above divides"
                   " by a promise the hull will never keep");
        /* The other side of the gap, at the reachable half of it. ApplyThrust is
           protected (the tests here model its flow rather than call it), but its rule
           is one line: size s = available/desired and skip the engine at s <= 0, so the
           draw is refused before anything is armed. consumeResourceMass is the same
           question asked of the pools, and a dry hull answers no. */
        CHECK_TRUE(!v->consumeResourceMass(ResourceType::Hydrogen, 0.001, eng).ok,
                   "the draw a tick would ask for is refused outright, which is the s = 0"
                   " that leaves armedThrust untouched: 20000 N rated, 0 N delivered, and"
                   " nothing above says so. That gap is why canBurn() exists");
        delete v;
    }
    {   /* Same hull, one tank of both propellants in its fuel group. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "fueled";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *eng = mkRigPart(defs, "engine", 100.0, 5.0, 4000.0, 1);
        v->attachMode(eng, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 100.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_TRUE(v->canBurn(), "hydrogen AND LOX, in a group the engine can drain");
        CHECK_NEAR(v->availableThrust(), 20000.0, 1e-12,
                   "and the rating did not move: fuel is not part of a rating");
        delete v;
    }
    {   /* Half a mix. ApplyThrust sizes a burn by the SCARCEST propellant (s = min over
           the resources this engine drains), so an engine with fuel and no oxidizer is a
           hole, not a weak engine. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "nolox";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *eng = mkRigPart(defs, "engine", 100.0, 5.0, 4000.0, 1);
        v->attachMode(eng, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 0.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_TRUE(!v->canBurn(), "100 kg of hydrogen and no oxidizer: nothing lights."
                   " The union mask guidance.cpp used to keep summed the ship's fuel by"
                   " RESOURCE and read this as propellant");
        delete v;
    }
    {   /* Reach, not amount. This engine is its own fuel barrier, so it has no drain
           layers (fuelDrainLayers, group -1) and can reach nothing -- while the hull
           carries enough of BOTH propellants for the old read to say yes. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "barrier";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *eng = mkRigPart(defs, "engine", 100.0, 5.0, 4000.0, 1, /*jet=*/false,
                              /*gimbal_max=*/0.0, /*h2_only=*/false, /*barrier=*/true);
        v->attachMode(eng, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 100.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_NEAR(v->availableThrust(), 20000.0, 1e-12, "rated, and it would fly --");
        CHECK_TRUE(!v->canBurn(), "-- if it could be fed. Full tanks of both propellants"
                   " sit on the hull and this engine is walled off from every one of"
                   " them; a ship-wide pool cannot see that, per-engine reach can");
        delete v;
    }
    {   /* The stage gate. The live stage (3) is an H2/LOX engine; below it sits a
           hydrogen-only engine fed by a hydrogen-only tank. At stage 3 neither answer is
           available from a ship-wide pool: the live engine lacks its oxidizer, and the
           engine that COULD burn is not ignited yet. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "stagegate";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *eng3 = mkRigPart(defs, "engine3", 100.0, 5.0, 4000.0, 3);
        v->attachMode(eng3, 0, AttachMode::Down, 0.0);
        Part *eng1 = mkRigPart(defs, "engine1", 100.0, 1.0, 4000.0, 1,
                               /*jet=*/false, /*gimbal_max=*/0.0, /*h2_only=*/true);
        v->attachMode(eng1, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 0.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_TRUE(v->activeStage() == 3, "the counter starts at the top stage, and a"
                   " part is live at stage >= the counter");
        CHECK_NEAR(v->availableThrust(), 20000.0, 1e-12,
                   "the live engine is the 20000 N one, and it rates healthy");
        CHECK_TRUE(!v->canBurn(), "but the live engine has no LOX, and the engine that"
                   " could run on hydrogen alone is not ignited -- a full tank below the"
                   " stage line cannot vote");
        v->advanceStage();   // the counter walks DOWN through every stage, and stage 2
                             // has no engine on this hull
        CHECK_TRUE(!v->canBurn(), "one stage down is not yet the engine that can burn");
        v->advanceStage();
        CHECK_TRUE(v->activeStage() == 1, "and now the hydrogen-only engine is ignited");
        CHECK_TRUE(v->canBurn(), "the same 100 kg of hydrogen, one stage lower, is a"
                   " burn: this is the read a law needs after a stage quit mid-burn");
        delete v;
    }
    {   /* EXISTS, not ALL -- and what that leaves uneven (#639). Two engines on the SAME
           live stage: a 20000 N H2/LOX one and a 4000 N hydrogen-only one, over a tank
           with hydrogen and no LOX. The hydrogen engine can burn; the big one cannot.
           canBurn() is true (one dry engine does not veto a stage that can fly) while
           the ceiling sums BOTH, so a law gating on this read and sizing with that one
           plans with 24000 N of which 4000 N will ever appear. Pinned as the pair works
           today, not as anyone wants it to work. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "mixedset";
        Part *pod = mkRigPart(defs, "pod", 1900.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *engA = mkRigPart(defs, "engineA", 100.0, 5.0, 4000.0, 1);
        v->attachMode(engA, 0, AttachMode::Down, 0.0);
        Part *engB = mkRigPart(defs, "engineB", 100.0, 1.0, 4000.0, 1, /*jet=*/false,
                               /*gimbal_max=*/0.0, /*h2_only=*/true);
        v->attachMode(engB, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 0.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_TRUE(v->canBurn(), "the hydrogen-only engine has what it needs");
        CHECK_NEAR(v->availableThrust(), 24000.0, 1e-12,
                   "while the ceiling is a sum over every ignited engine, fed or not:"
                   " it bills the unfed 20000 N one too (#639)");
        delete v;
    }
    {   /* Jets are engines, and availability is per ENGINE. This hull carries 200 kg of
           rocket propellant, which no part of it can burn: the only engine is a jet. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "jethull";
        Part *pod = mkRigPart(defs, "pod", 400.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *jet = mkRigPart(defs, "jet", 50.0, 1.0, 1000.0, 1, /*jet=*/true);
        v->attachMode(jet, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 100.0, 100.0, 100.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        CHECK_TRUE(!v->canBurn(), "200 kg of hydrogen and LOX on board and not one gram"
                   " of the fuel this engine burns");
        delete v;
        std::deque<PartDef> d2;
        Vehicle *fv = new Vehicle;
        fv->name = "jethullfueled";
        Part *pod2 = mkRigPart(d2, "pod", 400.0, 0.0, 0.0, 1);
        fv->setRoot(pod2);
        Part *jet2 = mkRigPart(d2, "jet", 50.0, 1.0, 1000.0, 1, /*jet=*/true);
        fv->attachMode(jet2, 0, AttachMode::Down, 0.0);
        Part *jt = mkRigTank(d2, "tank", 50.0, 0.0, 0.0, 500.0);
        fv->attachMode(jt, 0, AttachMode::Down, 0.0);
        fv->init();
        CHECK_TRUE(fv->canBurn(), "500 kg of jet fuel: this read asks about the FUEL and"
                   " the reach, never about the air -- the air is the ceiling's zero, and"
                   " a jet that cannot breathe still answers true here");
        delete fv;
    }
}

/*
 * 4e. A JET BURNS ITS LAST GRAMS (#638). consumeResourceMass is drain-or-nothing: it
 *     refuses a draw the pools cannot cover. The rocket branch has always sized its draw
 *     first (s = available/desired, then draw desired*s), so a rocket throttles its own
 *     final tick. The jet branch used to draw the whole tick's flow, get refused, and --
 *     because armedThrust is only written on a successful draw -- go quiet for ever with
 *     fuel still in the Vessel window. Both branches now ask Vehicle::burnShare().
 *     The expectations are RATIOS measured off the hull itself: the COM is not the
 *     transform origin, so the air -- and with it the flow and the thrust -- is whatever
 *     this hull actually sits in, and that cancels out of a half-tick comparison.
 */
static void test_jet_last_grams() {
    printf("== a jet burns its last grams (#638) ==\n");
    {
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "jetlast";
        Part *pod = mkRigPart(defs, "pod", 400.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *jet = mkRigPart(defs, "jet", 50.0, 1.0, 1000.0, 1, /*jet=*/true);
        v->attachMode(jet, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 50.0, 0.0, 0.0, 500.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        TerrainBody *air = mkPressureBody(1.225);
        v->m_parent = air;
        v->placeShip(glm::dvec3(0.0, 0.0, 600000.0), glm::dmat3(1.0));
        v->setThrottle(1.0);
        double *jf = &tank->resources.current[(int)ResourceType::JetFuel];

        v->ApplyThrust_TESTONLY(1.0 / 60.0);
        const double flow = 500.0 - *jf;          /* one full tick's draw, as measured */
        const float full = jet->armedThrust;
        CHECK_TRUE(flow > 0.0, "a full tank funds the whole tick");
        /* What the draw SHOULD be, from the law and not from itself. The first version
           of this pin compared *jf with 500.0 - flow after defining flow as
           500.0 - *jf -- an identity that passes any draw. The COM is not the
           transform origin, so the density the hull sees is measured; the gate is what
           that density earns (drag.h), and a funded tank means the fuel share is 1. */
        const double gate = jetAirGate(v->airDensityAtCom(),
                                       air->surface.atmosphere.oxidizer_share);
        CHECK_NEAR(gate, 1.0, 1e-3, "sea-level air opens the gate");
        CHECK_NEAR(flow, jet->jetFuelRate() * (1.0 / 60.0) * gate, 1e-12,
                   "and a funded jet draws its rated flow through that gate -- the gate"
                   " once, and no fuel share on a tank that covers the tick");
        CHECK_TRUE(full > 0.0, "so the engine arms at full thrust");

        /* Half a tick left. Before #638 this tick was refused outright. */
        *jf = 0.5 * flow;
        v->ApplyThrust_TESTONLY(1.0 / 60.0);
        CHECK_NEAR(jet->armedThrust, 0.5 * full, 1e-3,
                   "the sliver tick burns at the share it can pay for, not at full"
                   " thrust and not at zero: momentum the fuel did not buy is the same"
                   " debt #630 closed on the ram term");
        CHECK_NEAR(*jf, 0.0, 0.0,
                   "and the tank is EMPTY -- the last grams are reachable now, where they"
                   " used to sit unreachable under one tick of flow for ever");
        CHECK_TRUE(!v->canBurn(),
                   "and the read agrees: an empty jet canBurn() no more than it thrusts");

        /* The tick after that: nothing to draw, so nothing armed. clearThrust() is what
           the real loop does before thrusting (tick.cpp); without it the part keeps last
           tick's armedThrust and the pin passes on a stale value. */
        v->clearThrust();
        v->ApplyThrust_TESTONLY(1.0 / 60.0);
        CHECK_NEAR(jet->armedThrust, 0.0, 0.0, "a dry jet arms nothing, quietly");
        CHECK_NEAR(*jf, 0.0, 0.0, "and a dry tick takes nothing from an empty tank");
        delete v->m_parent;
        delete v;
    }
    {
        /* THIN AIR, same hull, 0.3 x the rating condition. Two scalars live in this
           branch and only one of them belongs on the flow: the AIR GATE sizes the tick
           (an intake moves a volume), the FUEL SHARE sizes what of that tick the tanks
           can pay for. A second gate hidden in the draw would read as a thrifty engine
           that still flies at full thrust. */
        std::deque<PartDef> defs;
        Vehicle *v = new Vehicle;
        v->name = "jethigh";
        Part *pod = mkRigPart(defs, "pod", 400.0, 0.0, 0.0, 1);
        v->setRoot(pod);
        Part *jet = mkRigPart(defs, "jet", 50.0, 1.0, 1000.0, 1, /*jet=*/true);
        v->attachMode(jet, 0, AttachMode::Down, 0.0);
        Part *tank = mkRigTank(defs, "tank", 50.0, 0.0, 0.0, 500.0);
        v->attachMode(tank, 0, AttachMode::Down, 0.0);
        v->init();
        TerrainBody *thin = mkPressureBody(0.3675);
        v->m_parent = thin;
        v->placeShip(glm::dvec3(0.0, 0.0, 600000.0), glm::dmat3(1.0));
        v->setThrottle(1.0);
        double *jf = &tank->resources.current[(int)ResourceType::JetFuel];

        const double gate = jetAirGate(v->airDensityAtCom(),
                                       thin->surface.atmosphere.oxidizer_share);
        CHECK_NEAR(gate, 0.3, 0.01,
                   "thin air gates the jet without closing it (0 < gate < 1)");
        v->ApplyThrust_TESTONLY(1.0 / 60.0);
        CHECK_NEAR(500.0 - *jf, jet->jetFuelRate() * (1.0 / 60.0) * gate, 1e-12,
                   "the draw carries the air gate exactly once -- the fuel share is 1"
                   " with a full tank, so anything else here is a double penalty");
        CHECK_TRUE(jet->armedThrust > 0.0, "and it still flies");
        delete v->m_parent;
        delete v;
    }
    /* The sizing itself, the shape both branches now share. */
    CHECK_NEAR(Vehicle::burnShare(10.0, 4.0), 1.0, 0.0, "pools cover the tick -> full");
    CHECK_NEAR(Vehicle::burnShare(2.0, 4.0), 0.5, 0.0, "half a tick -> half");
    CHECK_NEAR(Vehicle::burnShare(0.0, 4.0), 0.0, 0.0, "nothing to draw -> no burn");
    CHECK_NEAR(Vehicle::burnShare(10.0, 0.0), 0.0, 0.0,
               "a zero-length tick is no burn, the way the rocket branch always read it");
    CHECK_NEAR(Vehicle::burnShare(std::nan(""), 4.0), 0.0, 0.0,
               "a NaN availability fails CLOSED -- (x < 1.0) would answer fully covered");
}

/* THE THROTTLE HAS A SETPOINT CHANNEL, AND IT SAID WHO IS HOLDING IT (#400 F3).
   availableAccel() (F1) tells a law what a burn could do; this is the half that lets it
   ASK for the burn. The throttle used to be a public float, so the only way in for a law
   was to write it directly, and the only way the player had was a RATE (ThrottleUp/Down
   at 0.6/s). Both worked; neither could see the other. A law that re-writes the field
   every tick silently out-ramps a player holding R, and a law that reads the field back
   cannot tell its own answer from the player's -- the same invisible fight D-2 settled
   for the stick, where a held key owns its axis and the law keeps the rest.
   Fixture: a 1000 kg pod and one 20000 N engine, in vacuum, so every thrust number is
   20000 x the throttle and nothing else can move it. */
static void test_the_throttle_setpoint_channel() {
    printf("== the throttle setpoint, and who holds the lever (#400) ==\n");
    ThrustRig r;
    r.v = new Vehicle;
    r.v->name = "throttlelever";
    Part *pod = mkRigPart(r.defs, "pod", 1000.0, 0.0, 0.0, 1);
    r.v->setRoot(pod);
    Part *eng = mkRigPart(r.defs, "engine", 100.0, 5.0, 4000.0, 1);
    r.v->attachMode(eng, 0, AttachMode::Down, 0.0);
    r.v->init();
    TerrainBody *vac = mkPressureBody(0.0);
    r.v->m_parent = vac;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 600000.0 + 1000.0), glm::dmat3(1.0));
    Vehicle *v = r.v;

    /* The setpoint goes through the funnel, not around it (#424's door). */
    v->setThrottle(0.4);
    CHECK_NEAR(v->throttle(), 0.4f, 1e-9, "a named write, in the units the readouts use");
    CHECK_NEAR(v->producedThrust(), 20000.0 * 0.4f, 1e-3,
               "and the thrust follows: 0.4 of a 20000 N stage (0.4f, because the"
               " throttle is a float, so the product is not exactly 8000)");
    v->Command(ShipCmd(ThrottleSet, 0.25f), true, 1.0);
    CHECK_NEAR(v->throttle(), 0.25f, 1e-9,
               "a law sends it as a ShipCmd, the same door the keys use");
    CHECK_NEAR(v->producedThrust(), 5000.0, 1e-9, "one quarter of rated thrust");

    /* A law's ask saturates rather than being a sender bug: it divides an acceleration
       it wants by availableAccel(), and wanting more than the stage can give is normal. */
    v->setThrottle(1.7);
    CHECK_NEAR(v->throttle(), 1.0f, 0.0, "a 300% ask lands at 100%, not 300%");
    v->setThrottle(-0.3);
    CHECK_NEAR(v->throttle(), 0.0f, 0.0, "and a negative one lands at zero");
    v->setThrottle(std::nan(""));
    CHECK_NEAR(v->throttle(), 0.0f, 0.0, "NaN fails CLOSED, like every control-state door");
    CHECK_NEAR(v->producedThrust(), 0.0, 0.0,
               "so a NaN cannot ride into the fuel rates or the save file as a number"
               " the readouts would print");

    /* The player's channel is unchanged: a RATE, so the ramp feels the same at any
       --physics-hz. Two half-second presses at 0.6/s is 0.6. */
    v->Command(ShipCmd(ThrottleUp), true, 0.5);
    v->Command(ShipCmd(ThrottleUp), true, 0.5);
    CHECK_NEAR(v->throttle(), 0.6f, 1e-6, "the keys still ramp, 0.6 per second");
    CHECK_TRUE(v->throttleKeyHeld(),
               "and the ship can say that the PLAYER is on the lever this tick");

    /* The flag is per-tick intent, like stick[]: the next tick without the keys does not
       keep claiming the player has it. */
    v->clearThrust();
    CHECK_TRUE(!v->throttleKeyHeld(), "clearThrust() is the tick boundary");
    v->Command(ShipCmd(ThrottleSet, 0.9f), true, 1.0);
    CHECK_TRUE(!v->throttleKeyHeld(),
               "the law's own channel never claims the lever from the player");

    /* PAUSED is paused for the law too: a setpoint must not fire behind a frozen clock
       the way the keys already do not. */
    v->Command(ShipCmd(ThrottleSet, 0.3f), false, 1.0);
    CHECK_NEAR(v->throttle(), 0.9f, 1e-9, "simActive false: nothing written");

    /* THE RULE the autopilot will use, stated as code: yield while a key is held,
       resume when it is up. This is the behaviour F3 exists to make possible -- without
       the flag the law has no way to know it lost, and the two writers fight. */
    v->clearThrust();
    v->setThrottle(0.0);                           // from a known stop
    v->Command(ShipCmd(ThrottleUp), true, 0.5);    // the player takes 0.3
    CHECK_NEAR(v->throttle(), 0.3f, 1e-6, "one half-second press from zero");
    auto lawTurn = [&v]() { if(!v->throttleKeyHeld()) { v->setThrottle(1.0); } };
    lawTurn();
    CHECK_NEAR(v->throttle(), 0.3f, 1e-6,
               "the law stands aside while the key is down -- the player keeps the"
               " lever, exactly as a held stick key owns its axis");
    v->clearThrust();                              // the key comes up
    lawTurn();
    CHECK_NEAR(v->throttle(), 1.0f, 0.0,
               "and takes it back the tick the key is released, without being asked");

    /* The lever is LATCHED, not per-tick intent: only the who-holds-it claim resets at
       the tick boundary. A refactor that zeroed the throttle beside that flag -- the
       natural place to reach for, since they sit together -- would cut a latched burn
       every tick, and the flag pins above read the same line so they would not notice. */
    v->setThrottle(0.6);
    v->clearThrust();
    v->clearThrust();
    CHECK_NEAR(v->throttle(), 0.6f, 0.0,
               "the setpoint survives the tick boundary; only the key claim resets");

    /* Coast is the most ordinary thing a law asks for -- and ShipCmd's default amount is
       0.0f, so a forgotten argument means coast too. Neither the zero ask nor the rate
       channel's ceiling was ever reached here. */
    v->setThrottle(0.7);
    v->Command(ShipCmd(ThrottleSet), true, 1.0);
    CHECK_NEAR(v->throttle(), 0.0f, 0.0, "ThrottleSet with the default amount is coast");
    v->setThrottle(0.7);
    v->Command(ShipCmd(ThrottleSet, 0.0f), true, 1.0);
    CHECK_NEAR(v->throttle(), 0.0f, 0.0, "and an explicit 0 lands the same way");
    v->setThrottle(0.8f);
    for(int i = 0; i < 5; i++) { v->Command(ShipCmd(ThrottleUp), true, 0.5); }
    CHECK_NEAR(v->throttle(), 1.0f, 0.0, "the keys ramp to 1.0 and no further");

    /* Losing the pilot cuts the throttle through the same door. */
    v->clearThrust();
    v->setThrottle(0.8);
    v->releaseControl();
    CHECK_NEAR(v->throttle(), 0.0f, 0.0, "releaseControl() cuts it");
    delete v;
    delete vac;
}

int main() {
    test_inertia_diag();
    printf("\n");
    test_substep_delivery();
    printf("\n");
    test_fuel_flow();
    test_produced_and_available_thrust();
    printf("\n");
    test_available_accel_along();
    printf("\n");
    test_available_accel_along_two_mounts();
    printf("\n");
    test_can_burn();
    printf("\n");
    test_jet_last_grams();
    printf("\n");
    test_the_throttle_setpoint_channel();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
