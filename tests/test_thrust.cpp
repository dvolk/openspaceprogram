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
        if (!std::isfinite(_a) || std::fabs(_a - _e) > _tol) {                    \
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
 * the body's sea level and is not moving, so rho = E = exp(-1000/8000) of sea level and
 * the jet's ram term is zero; no tank is modelled because these reads are POTENTIALS --
 * they never consult the fuel.
 *   stage 3 is the first live stage, so only the 20000 N engine is available at first,
 *   and that number does not care about air at all;
 *   the jet at rest in that air is E*(fan + mdot*ve) = 1100E, and at its best speed
 *   (v = ve/2, where drag.h's ram term tops out) it is E*1100 + rho*A*(ve/2)^2 =
 *   1100E + 10000E = 11100E; in vacuum, neither -- air is what it breathes.
 */
struct ThrustRig {
    std::deque<PartDef> defs;
    Vehicle *v = nullptr;
};

/* A part for this rig: `rate` x `ve` is the rated thrust, and rate 0 (the pod) simply
   means it is not a thruster. `jet` makes it air-breathing instead. */
static Part *mkRigPart(std::deque<PartDef> &defs, const char *name, double mass,
                       double rate, double ve, int stage, bool jet = false) {
    Body *b = new Body;
    b->btBody = nullptr;
    b->shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    b->mass = mass;
    defs.push_back(PartDef());
    PartDef &d = defs.back();
    d.name = name;
    d.mass = (float)mass;
    d.exhaust_velocity = ve;
    if(jet) {
        d.jet = true;
        d.setPropellantRate(ResourceType::JetFuel, rate);
        d.jet_fan_thrust = 100.0;
        d.jet_intake_area = 0.04;
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
    body->surface.sea_level = 0.0f;
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
       to be ALT. The COM's own geometry is test_inertia's business. */
    E = std::exp(-(glm::length(r.v->get_center_of_mass()) - 600000.0) / 8000.0);

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
    CHECK_NEAR(r.v->availableThrust(), 70000.0 + 11100.0 * E, 1e-9,
               "20000 + 50000 + the jet at ITS BEST SPEED in THIS air: E*(100 + 1*1000)"
               " + rho*0.04*500*500, with rho = E");
    CHECK_NEAR(r.v->producedThrust(), 0.5 * (70000.0 + 1100.0 * E), 1e-9,
               "at half throttle the jet is asked for what it gives standing still"
               " (fan + mdot*ve, gated by the air), because producedThrust uses the"
               " speed the ship actually has");
    CHECK_NEAR(r.v->availableAccel(), (70000.0 + 11100.0 * E) / 1250.0, 1e-9,
               "and the a_max that stage-down just bought, at the same mass");
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
    test_the_throttle_setpoint_channel();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
