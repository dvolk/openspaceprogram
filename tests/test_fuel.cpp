//
// Headless test for Vehicle::consumeResourceMass (src/vehicle.h): a tick's
// fuel must drain PRO-RATA across the active stage's tanks, not from the
// first tank that can cover the flow. First-tank-first drained one tank to
// empty before its siblings (with a radial layout: the central tank, then
// each side tank in turn), shifting the ship's mass distribution and
// torquing the ship under thrust. Pro-rata shares -- proportional to each
// tank's contents -- keep a symmetric cluster draining together, and let a
// flow that no single tank can cover still fire (the pool supplies it).
//
// Links the REAL SetMass (src/physics.cpp, as test_thrust.cpp does), so the
// part-mass shedding + Bullet inertia update is exercised too. No GL
// context is needed.
//
// Runs from the repo root:
//   make test   (or: ./test_fuel)

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include <cmath>
#include <cstdio>
#include <vector>

#include "vehicle.h"   // Vehicle + PartDef + ResourceContent (inline)

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

/* A ship of `n` tank parts, each holding cap[i] kg of BOTH propellants
   (the catalog tanks do), on the given stages (empty -> all stage 1;
   otherwise stages.size() must equal n). The PartDefs and collision
   shapes outlive the Vehicle (partDefs point into them; Bullet keeps the
   shape pointer in the rigid body). */
struct Ship {
    Vehicle *v;
    std::vector<PartDef> defs;
    std::vector<btBoxShape *> shapes;
};

static Ship makeShip(const std::vector<float> &cap,
                     const std::vector<int> &stages) {
    Ship s;
    s.v = new Vehicle;
    const size_t n = cap.size();
    s.defs.reserve(n);
    for(size_t i = 0; i < n; i++) {
        const double m = 100.0 + (double)cap[i]; /* dry mass + fuel */
        btBoxShape *shape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
        btVector3 I;
        shape->calculateLocalInertia(m, I);
        btRigidBody::btRigidBodyConstructionInfo ci(m, 0, shape, I);
        Body *b = new Body;
        b->model = nullptr;   // no GL model in a headless test
        b->btBody = new btRigidBody(ci);
        b->mass = m;
        s.v->parts.push_back(b);
        s.shapes.push_back(shape);
        PartDef d;
        d.capacity[(int)ResourceType::Hydrogen] = cap[i];
        d.capacity[(int)ResourceType::LOX] = cap[i];
        s.defs.push_back(d);
        s.v->partDefs.push_back(&s.defs.back());
        s.v->partStages.push_back(stages.empty() ? 1 : stages[i]);
    }
    s.v->controllerIndex = 0;
    s.v->init();   // seeds partResources from the defs' capacities
    return s;
}

static void destroyShip(Ship &s) {
    /* onRails = true keeps the destructor off RemoveBody (the physics
       engine is not constructed in a headless test). */
    s.v->onRails = true;
    delete s.v;
    for(size_t i = 0; i < s.shapes.size(); i++) { delete s.shapes[i]; }
    s.shapes.clear();
}

/* The core fix: 4 equal tanks (a central + 3 radial, the heavy_one layout)
   all drain by the same amount. The OLD first-tank-first code emptied tank
   0 and left 100 kg in the rest -- the "first tank still 3/4 full" guard
   below is what this test must catch on the old implementation. */
static void test_prorata_equal_tanks() {
    printf("== Pro-rata drain: 4 equal tanks ==\n");
    Ship s = makeShip({100, 100, 100, 100}, {1, 1, 1, 1});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f);
    CHECK_TRUE(ok, "consume 100 kg from 4 x 100 kg tanks");

    for(int i = 0; i < 4; i++) {
        char buf[96];
        snprintf(buf, sizeof buf, "tank %d drained 25 kg (not first-only)", i);
        CHECK_NEAR(s.v->partResources[(size_t)i].current[(int)ResourceType::Hydrogen],
                   75.0, 1e-5, buf);
        snprintf(buf, sizeof buf, "tank %d's part shed its 25 kg", i);
        CHECK_NEAR(s.v->parts[(size_t)i]->mass, 175.0, 1e-5, buf);
    }
    CHECK_TRUE(s.v->partResources[0].current[(int)ResourceType::Hydrogen] > 50.0f,
               "first tank NOT drained first (old behavior)");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::LOX], 100.0, 1e-6,
               "LOX untouched by the H2 draw");
    destroyShip(s);
}

/* Unequal tanks: the shares are proportional to each tank's CONTENTS
   (200:100 -> 20:10), not an equal split (15:15) and not first-only (30:0).
   Proportional-to-contents is what keeps the ratio constant, so the tanks
   empty simultaneously and a symmetric ship stays symmetric. */
static void test_prorata_unequal_tanks() {
    printf("== Pro-rata drain: unequal tanks, shares proportional to contents ==\n");
    Ship s = makeShip({200, 100}, {1, 1});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 30.0f);
    CHECK_TRUE(ok, "consume 30 kg from 200 + 100 kg tanks");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::Hydrogen], 180.0, 1e-5,
               "big tank drained 20 (200 -> 180)");
    CHECK_NEAR(s.v->partResources[1].current[(int)ResourceType::Hydrogen], 90.0, 1e-5,
               "small tank drained 10 (100 -> 90)");
    destroyShip(s);
}

/* A flow that no SINGLE tank can cover but the combined pool can: the
   OLD code refused it (stranded 20 kg of usable fuel); the new code
   fires and drains both. */
static void test_stranded_fuel() {
    printf("== Stranded fuel: flow covered by the pool, no single tank ==\n");
    Ship s = makeShip({60, 60}, {1, 1});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f);
    CHECK_TRUE(ok, "pool-covered flow fires (vacuity guard: old code refused)");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "tank 0 drained 50");
    CHECK_NEAR(s.v->partResources[1].current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "tank 1 drained 50");
    destroyShip(s);
}

/* A flow above the combined total: refused, and NOTHING is drained
   (no partial drain, no half-burn). */
static void test_insufficient_total() {
    printf("== Insufficient total: refused, no partial drain ==\n");
    Ship s = makeShip({60, 60}, {1, 1});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 130.0f);
    CHECK_TRUE(!ok, "flow above the total is refused");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tank 0 untouched");
    CHECK_NEAR(s.v->partResources[1].current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tank 1 untouched");
    CHECK_NEAR(s.v->parts[0]->mass, 160.0, 1e-6, "part 0 mass untouched");
    destroyShip(s);
}

/* Consuming exactly the total: allowed, all tanks land on zero (not a
   negative float round-off). */
static void test_full_drain() {
    printf("== Full drain: consume exactly the total ==\n");
    Ship s = makeShip({50, 50}, {1, 1});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f);
    CHECK_TRUE(ok, "consuming exactly the total is allowed");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tank 0 empty");
    CHECK_NEAR(s.v->partResources[1].current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tank 1 empty");
    CHECK_TRUE(s.v->partResources[0].current[(int)ResourceType::Hydrogen] >= 0.0f,
               "no negative contents (float rounding)");
    CHECK_TRUE(s.v->partResources[1].current[(int)ResourceType::Hydrogen] >= 0.0f,
               "no negative contents (float rounding)");
    destroyShip(s);
}

/* The ACTIVE (lowest-numbered) stage burns its own propellant only: the
   upper stage's tanks are untouched. */
static void test_stage_gating() {
    printf("== Stage gating: the upper stage's tanks are untouched ==\n");
    Ship s = makeShip({50, 50, 1000}, {1, 1, 2});

    const bool ok = s.v->consumeResourceMass(ResourceType::Hydrogen, 80.0f);
    CHECK_TRUE(ok, "consume 80 kg from the active stage (2 x 50)");
    CHECK_NEAR(s.v->partResources[0].current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "stage-1 tank 0 drained 40");
    CHECK_NEAR(s.v->partResources[1].current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "stage-1 tank 1 drained 40");
    CHECK_NEAR(s.v->partResources[2].current[(int)ResourceType::Hydrogen], 1000.0, 1e-6,
               "stage-2 tank untouched");
    CHECK_NEAR(s.v->parts[2]->mass, 1100.0, 1e-6, "stage-2 part mass untouched");
    destroyShip(s);
}

/* ApplyThrust calls consumeResourceMass once PER ENGINE per propellant
   per tick: after several draws the cluster must still be in step with
   itself (no tank drifting out of line). */
static void test_repeated_draws_stay_symmetric() {
    printf("== Repeated draws (per engine, per propellant) stay symmetric ==\n");
    Ship s = makeShip({100, 100, 100}, {1, 1, 1});

    for(int k = 0; k < 4; k++) { /* 2 engines x 2 ticks, 10 kg each */
        if(!s.v->consumeResourceMass(ResourceType::Hydrogen, 10.0f)) {
            CHECK_TRUE(false, "draw refused mid-burn");
            break;
        }
    }
    /* 4 draws x 10 kg = 40 kg off 300 -> 260 total, 260/3 per tank. */
    for(int i = 0; i < 3; i++) {
        char buf[96];
        snprintf(buf, sizeof buf, "tank %d still in step (100 -> 260/3)", i);
        CHECK_NEAR(s.v->partResources[(size_t)i].current[(int)ResourceType::Hydrogen],
                   260.0 / 3.0, 1e-5, buf);
    }
    destroyShip(s);
}

int main() {
    test_prorata_equal_tanks();
    printf("\n");
    test_prorata_unequal_tanks();
    printf("\n");
    test_stranded_fuel();
    printf("\n");
    test_insufficient_total();
    printf("\n");
    test_full_drain();
    printf("\n");
    test_stage_gating();
    printf("\n");
    test_repeated_draws_stay_symmetric();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
