//
// Headless test for Vehicle::consumeResourceMass / fuelPool (src/vehicle.h).
//
// The fuel model is CONNECTION-based, not stage-based: an engine draws the
// propellant of its FUEL GROUP (buildFuelGroups) -- the connected component
// of the part tree it sits in, with fuel barriers (PartDef::fuel_barrier,
// e.g. decouplers) as walls that split the groups. Stage numbers gate only
// WHEN an engine ignites; they no longer decide WHICH tanks feed it.
//
// These tests pin:
//   * an engine drains the tanks in its own fuel group, pro-rata (not
//     first-tank-first -- that shifts the mass distribution and torques the
//     ship under thrust);
//   * a fuel barrier SPLITS the groups, so an engine never draws fuel from
//     across it (the heavy_two fix: the central engine burns the central
//     tanks, not the boosters' tanks across the radial decoupler);
//   * with fuel links, the drain is LAYER by hop distance (furthest
//     layer first), pro-rata ACROSS a layer: the symmetric star (two arms,
//     both one hop out -- heavy_two) drains both arms together, and the
//     dual chain C->B->A, D->E->A drains {C,D}, then {B,E}, then A;
//   * a flow no single tank can cover still fires (the pool supplies it),
//     and a flow above the total is refused with no partial drain.
//
// Links the REAL SetMass (src/physics.cpp, as test_thrust.cpp does), so the
// part-mass shedding + Bullet inertia update is exercised too. No GL context
// is needed. The ships here are built by hand: parts + weld links (just the
// Part* adjacency for buildFuelGroups -- no Bullet constraint, so it stays
// headless).
//
// Runs from the repo root:
//   make test   (or: ./test_fuel)

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include <cmath>
#include <cstdio>
#include <deque>
#include <utility>
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

/* A hand-built ship. `defs` must outlive the Vehicle (Part::def points
   into it). A deque, not a vector: push_back on a vector reallocates and
   would dangle every Part::def handed out so far -- a deque never
   invalidates references to existing elements, so no reserve() bookkeeping
   is needed at the call sites. Each Part wraps its rigid body and owns its
   collision shape (freed by ~Body). */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* Add one part. h2/lox > 0 -> a tank; engine -> a thruster; barrier -> a
   fuel wall (decoupler). Returns the new Part (index parts.size()-1). */
static Part *addPart(Ship &s, float h2, float lox, bool engine = false,
                     bool barrier = false) {
    const double m = 100.0 + (double)h2 + (double)lox;   /* dry mass + fuel */
    btBoxShape *shape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
    btVector3 I;
    shape->calculateLocalInertia(m, I);
    btRigidBody::btRigidBodyConstructionInfo ci(m, 0, shape, I);
    Body *b = new Body;
    b->model = nullptr;    /* no GL model in a headless test */
    b->btBody = new btRigidBody(ci);
    b->shape = b->btBody->getCollisionShape();
    b->mass = m;

    PartDef d;
    d.capacity[(int)ResourceType::Hydrogen] = h2;
    d.capacity[(int)ResourceType::LOX] = lox;
    if(engine) { d.fuel_rate = 1.0; d.exhaust_velocity = 100.0; }
    d.fuel_barrier = barrier;
    s.defs.push_back(d);

    Part *p = new Part;
    p->body = b;
    p->def = &s.defs.back();
    s.v->parts.push_back(p);
    return p;
}

/* Join two parts into the same fuel group by setting the part-tree edge
   (b becomes a's child). buildFuelGroups walks Part::parent, so that single
   pointer is all the grouping needs -- no Bullet constraint, no anchors, and
   the test stays headless. Every chain below is linear, so no part is ever
   given two parents. */
static void link(Ship &s, Part *a, Part *b) {
    (void)s;
    b->parent = a;
}

/* A headless ship has no build_ship() to name a controller part, so name one
   (the first part) first; init() then seeds the tanks, builds the fuel groups
   and builds the ship's single rigid body. enterWorld() is deliberately NOT
   called -- there is no physics world here, which is exactly why it is a
   separate step from init(). */
static void initShip(Ship &s) {
    s.v->controller = s.v->parts[0];
    s.v->init();
}

static void destroyShip(Ship &s) {
    /* onRails = true keeps the destructor off RemoveBody (no physics world
       here). ~Vehicle deletes each Part; ~Part frees the Body (rigid body +
       owned shape). The PartDefs in s.defs outlive the Parts. */
    s.v->onRails = true;
    delete s.v;
}

/* A tank -> tank -> tank chain with an engine at the root: all four in one
   fuel group. An engine drains the group's tanks PRO-RATA (proportional to
   contents), not first-tank-first. */
static void test_prorata_in_group() {
    printf("== Pro-rata drain within one fuel group ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, /*engine=*/true);
    Part *t0 = addPart(s, 100, 100);
    Part *t1 = addPart(s, 100, 100);
    Part *t2 = addPart(s, 100, 100);
    link(s, eng, t0); link(s, t0, t1); link(s, t1, t2);
    initShip(s);   /* seeds tanks + builds fuel groups */

    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 90.0f, eng),
               "consume 90 kg from the engine's fuel group");
    Part *tanks[3] = {t0, t1, t2};
    for(int i = 0; i < 3; i++) {
        char buf[64];
        snprintf(buf, sizeof buf, "tank %d drained 30 (pro-rata in group)", i);
        CHECK_NEAR(tanks[i]->resources.current[(int)ResourceType::Hydrogen],
                   70.0, 1e-5, buf);
        snprintf(buf, sizeof buf, "tank %d's part shed its 30 kg", i);
        CHECK_NEAR(tanks[i]->body->mass, 270.0, 1e-5, buf);
    }
    CHECK_NEAR(t0->resources.current[(int)ResourceType::LOX], 100.0, 1e-6,
               "LOX untouched by the H2 draw");
    destroyShip(s);
}

/* Unequal tanks: the shares are proportional to each tank's CONTENTS
   (200:100 -> 20:10), not an equal split (15:15) and not first-only (30:0).
   Proportional-to-contents keeps the ratio constant, so the tanks empty
   simultaneously and a symmetric ship stays symmetric. */
static void test_prorata_unequal() {
    printf("== Pro-rata: unequal tanks, shares proportional to contents ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *big = addPart(s, 200, 200);
    Part *small = addPart(s, 100, 100);
    link(s, eng, big); link(s, big, small);
    initShip(s);

    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 30.0f, eng),
               "consume 30 kg from 200 + 100 kg tanks");
    CHECK_NEAR(big->resources.current[(int)ResourceType::Hydrogen], 180.0, 1e-5,
               "big tank drained 20 (200 -> 180)");
    CHECK_NEAR(small->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-5,
               "small tank drained 10 (100 -> 90)");
    destroyShip(s);
}

/* THE heavy_two fix: a fuel barrier (decoupler) SPLITS the fuel groups, so
   an engine on one side never draws from the tanks on the other side. Here
   engineA and tankA are on one side of the wall, engineB and tankB on the
   other. engineA must drain tankA only; engineB must drain tankB only. */
static void test_barrier_splits_groups() {
    printf("== Fuel barrier splits groups: each engine draws its side only ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 100, 100);
    Part *wall = addPart(s, 0, 0, /*engine=*/false, /*barrier=*/true);
    Part *tankB = addPart(s, 100, 100);
    Part *engB = addPart(s, 0, 0, true);
    link(s, engA, tankA);
    link(s, tankA, wall);
    link(s, wall, tankB);
    link(s, tankB, engB);
    initShip(s);

    CHECK_TRUE(!tankA->isFuelBarrier() && wall->isFuelBarrier(),
               "wall is the only fuel barrier");

    /* engineA drains only tankA (NOT tankB, across the wall). */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engineA draws 50 kg from its own group");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-5,
               "tankA drained 50");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankB untouched (the barrier blocks the draw)");

    /* engineB drains only tankB (NOT tankA). */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 30.0f, engB),
               "engineB draws 30 kg from its own group");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 70.0, 1e-5,
               "tankB drained 30");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-6,
               "tankA unchanged by engineB's draw");
    destroyShip(s);
}

/* A flow that no SINGLE tank can cover but the combined group can: the
   group supplies it (drains both), not refused. */
static void test_stranded_fuel() {
    printf("== Stranded fuel: flow covered by the group, no single tank ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *t0 = addPart(s, 60, 60);
    Part *t1 = addPart(s, 60, 60);
    link(s, eng, t0); link(s, t0, t1);
    initShip(s);

    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f, eng),
               "group-covered flow fires");
    CHECK_NEAR(t0->resources.current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "tank 0 drained 50");
    CHECK_NEAR(t1->resources.current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "tank 1 drained 50");
    destroyShip(s);
}

/* A flow above the combined group total: refused, and NOTHING is drained
   (no partial drain, no half-burn). */
static void test_insufficient_total() {
    printf("== Insufficient total: refused, no partial drain ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *t0 = addPart(s, 60, 60);
    Part *t1 = addPart(s, 60, 60);
    link(s, eng, t0); link(s, t0, t1);
    initShip(s);

    CHECK_TRUE(!s.v->consumeResourceMass(ResourceType::Hydrogen, 130.0f, eng),
               "flow above the group total is refused");
    CHECK_NEAR(t0->resources.current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tank 0 untouched");
    CHECK_NEAR(t1->resources.current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tank 1 untouched");
    CHECK_NEAR(t0->body->mass, 220.0, 1e-6, "tank 0 mass untouched");
    destroyShip(s);
}

/* Consuming exactly the group total: allowed, all tanks land on zero (not a
   negative float round-off). */
static void test_full_drain() {
    printf("== Full drain: consume exactly the group total ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *t0 = addPart(s, 50, 50);
    Part *t1 = addPart(s, 50, 50);
    link(s, eng, t0); link(s, t0, t1);
    initShip(s);

    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f, eng),
               "consuming exactly the group total is allowed");
    CHECK_NEAR(t0->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tank 0 empty");
    CHECK_NEAR(t1->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tank 1 empty");
    CHECK_TRUE(t0->resources.current[(int)ResourceType::Hydrogen] >= 0.0f,
               "no negative contents (float rounding)");
    CHECK_TRUE(t1->resources.current[(int)ResourceType::Hydrogen] >= 0.0f,
               "no negative contents (float rounding)");
    destroyShip(s);
}

/* ApplyThrust draws once PER ENGINE per propellant per tick: after several
   draws the cluster must still be in step with itself (no tank drifting out
   of line). */
static void test_repeated_draws_stay_symmetric() {
    printf("== Repeated draws (per engine, per propellant) stay symmetric ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *t0 = addPart(s, 100, 100);
    Part *t1 = addPart(s, 100, 100);
    Part *t2 = addPart(s, 100, 100);
    link(s, eng, t0); link(s, t0, t1); link(s, t1, t2);
    initShip(s);

    for(int k = 0; k < 4; k++) {   /* 2 engines x 2 ticks, 10 kg each */
        if(!s.v->consumeResourceMass(ResourceType::Hydrogen, 10.0f, eng)) {
            CHECK_TRUE(false, "draw refused mid-burn");
            break;
        }
    }
    /* 4 draws x 10 kg = 40 kg off 300 -> 260 total, 260/3 per tank. */
    Part *tanks[3] = {t0, t1, t2};
    for(int i = 0; i < 3; i++) {
        char buf[64];
        snprintf(buf, sizeof buf, "tank %d still in step (100 -> 260/3)", i);
        CHECK_NEAR(tanks[i]->resources.current[(int)ResourceType::Hydrogen],
                   260.0 / 3.0, 1e-5, buf);
    }
    destroyShip(s);
}

/* A fuel link (from -> to) means fuel flows from->to, so the engine in `to`'s
   group can draw from `from`'s group. The linked source is drained FIRST
   (furthest source first), then the engine's own group. */
static void test_fuel_link_one_way() {
    printf("== Fuel link: one-way (linked source drains first) ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 100, 100);
    Part *tankB = addPart(s, 100, 100);
    link(s, engA, tankA);
    initShip(s);   /* groups: engA+tankA=0, tankB=1 */

    /* Fuel link: tankB's group -> engA's group (fuel flows from B to A). */
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankB, engA});

    /* Engine A drains tankB (linked source, dist 1) first, then tankA (dist 0). */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engine A draws 50 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-5,
               "tankB drained 50 (linked source, drained first)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (own group, drained second)");

    /* Drain 80 more: tankB has 50 left (drains fully), then tankA drains 30. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 80.0f, engA),
               "engine A draws 80 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankB empty");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 70.0, 1e-5,
               "tankA drained 30");
    destroyShip(s);
}

/* Two-hop chain C->B->A: the furthest source (C) drains first, then B, then A.
   This exercises the transitive drain order (the "C->B->A drains C, then B,
   then A" rule). The links are tank->tank (not just tank->engine). */
static void test_fuel_link_two_hop() {
    printf("== Fuel link: two-hop C->B->A (furthest first, tank->tank) ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 100, 100);
    Part *tankB = addPart(s, 100, 100);
    Part *tankC = addPart(s, 100, 100);
    link(s, engA, tankA);
    initShip(s);   /* groups: engA+tankA=0, tankB=1, tankC=2 */

    /* Fuel links: C feeds B, B feeds A (tank->tank chain). */
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankC, tankB});
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankB, engA});

    /* Engine A drains C (dist 2), then B (dist 1), then A (dist 0). */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engine A draws 50 kg");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-5,
               "tankC drained 50 (furthest, drained first)");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankB untouched");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched");

    /* Drain 120 more: C has 50 left (drains fully), then B drains 70. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 120.0f, engA),
               "engine A draws 120 kg");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankC empty");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 30.0, 1e-5,
               "tankB drained 70");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched");

    /* Drain 120 more: B has 30 left (drains fully), then A drains 90. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 120.0f, engA),
               "engine A draws 120 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankB empty");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 10.0, 1e-5,
               "tankA drained 90");
    destroyShip(s);
}

/* Two symmetric sources at the SAME hop distance (the heavy_two star: two
   radial arms, both one hop from the central engine's group). They are ONE
   layer and drain TOGETHER, pro-rata -- NOT one arm to empty then the
   other: that serial drain split the mass distribution and spun the ship. */
static void test_fuel_link_star_symmetric() {
    printf("== Fuel link: symmetric star (same-distance arms drain together) ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 100, 100);
    Part *tankB = addPart(s, 100, 100);
    Part *tankC = addPart(s, 100, 100);
    link(s, engA, tankA);
    initShip(s);   /* groups: engA+tankA=0, tankB=1, tankC=2 */

    /* Fuel links: B -> A and C -> A (both arms feed the engine's group). */
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankB, engA});
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankC, engA});

    /* 60 kg off the {B,C} layer (both at dist 1): 30 each, A untouched. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 60.0f, engA),
               "engine A draws 60 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 70.0, 1e-5,
               "tankB drained 30 (pro-rata with its sibling arm)");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 70.0, 1e-5,
               "tankC drained 30 (pro-rata with its sibling arm)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (own group is the last layer)");

    /* 150 more: the {B,C} layer has 70+70 left -- BOTH drain to empty,
       70 each, in step -- then the {A} layer takes the final 10. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 150.0f, engA),
               "engine A draws 150 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankB empty (drained with its sibling, not before it)");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankC empty (drained with its sibling)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-5,
               "tankA drained 10 (own group, last)");
    destroyShip(s);
}

/* Two chains feeding one engine: C->B->A and D->E->A. The layers are the
   hop-distance levels: {C,D} (2 hops), {B,E} (1 hop), {A} (own group).
   C and D drain together (pro-rata) until BOTH are empty; then B and E
   together; finally A. NOT C-to-empty then D, and NOT B or E before D. */
static void test_fuel_link_dual_chain() {
    printf("== Fuel link: dual chains C->B->A, D->E->A (layer by layer) ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 100, 100);
    Part *tankB = addPart(s, 100, 100);
    Part *tankC = addPart(s, 100, 100);
    Part *tankD = addPart(s, 100, 100);
    Part *tankE = addPart(s, 100, 100);
    link(s, engA, tankA);
    initShip(s);   /* groups: engA+tankA=0, B=1, C=2, D=3, E=4 */

    /* Fuel links: C feeds B, B feeds A; D feeds E, E feeds A. */
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankC, tankB});
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankB, engA});
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankD, tankE});
    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankE, engA});

    /* 50 kg off the {C,D} layer (both at dist 2): 25 each, nothing else. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engine A draws 50 kg");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 75.0, 1e-5,
               "tankC drained 25 (pro-rata with D, furthest layer)");
    CHECK_NEAR(tankD->resources.current[(int)ResourceType::Hydrogen], 75.0, 1e-5,
               "tankD drained 25 (pro-rata with C)");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankB untouched (its layer is next)");
    CHECK_NEAR(tankE->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankE untouched (its layer is next)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (own group, last layer)");

    /* 150 more: the {C,D} layer (150 left) drains fully -- 75 each, in
       step -- B, E and A untouched. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 150.0f, engA),
               "engine A draws 150 kg");
    CHECK_NEAR(tankC->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankC empty");
    CHECK_NEAR(tankD->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankD empty (with C, not C first)");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankB untouched (its layer is next)");
    CHECK_NEAR(tankE->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankE untouched (its layer is next)");

    /* 100 more: the {B,E} layer (100 left): 50 each, A untouched. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 100.0f, engA),
               "engine A draws 100 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-5,
               "tankB drained 50 (pro-rata with E)");
    CHECK_NEAR(tankE->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-5,
               "tankE drained 50 (pro-rata with B)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (last layer)");

    /* 50 more: {B,E} has 50+50 left -- 25 each, A still untouched. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engine A draws 50 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 25.0, 1e-5,
               "tankB drained 25 (pro-rata with E)");
    CHECK_NEAR(tankE->resources.current[(int)ResourceType::Hydrogen], 25.0, 1e-5,
               "tankE drained 25 (pro-rata with B)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (all closer layers still have fuel)");

    /* 50 more: B and E drain to empty (25 each). */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 50.0f, engA),
               "engine A draws 50 kg");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankB empty");
    CHECK_NEAR(tankE->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "tankE empty (with B)");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 100.0, 1e-6,
               "tankA untouched (its layer is last)");

    /* 60 more: only the {A} layer has fuel: A drains 60. */
    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 60.0f, engA),
               "engine A draws 60 kg");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 40.0, 1e-5,
               "tankA drained 60 (own group, last)");
    destroyShip(s);
}

/* A flow above the total of ALL source groups (linked + own): refused,
   nothing drained. */
static void test_fuel_link_insufficient() {
    printf("== Fuel link: insufficient total across all groups ==\n");
    Ship s; s.v = new Vehicle;
    Part *engA = addPart(s, 0, 0, true);
    Part *tankA = addPart(s, 60, 60);
    Part *tankB = addPart(s, 60, 60);
    link(s, engA, tankA);
    initShip(s);   /* groups: engA+tankA=0, tankB=1 */

    s.v->fuelLinks.push_back(Vehicle::FuelLink{tankB, engA});

    /* Total = 60 (A) + 60 (B) = 120. Request 130: refused. */
    CHECK_TRUE(!s.v->consumeResourceMass(ResourceType::Hydrogen, 130.0f, engA),
               "flow above the total is refused");
    CHECK_NEAR(tankA->resources.current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tankA untouched");
    CHECK_NEAR(tankB->resources.current[(int)ResourceType::Hydrogen], 60.0, 1e-6,
               "tankB untouched");
    destroyShip(s);
}

int main() {
    test_prorata_in_group();
    printf("\n");
    test_prorata_unequal();
    printf("\n");
    test_barrier_splits_groups();
    printf("\n");
    test_stranded_fuel();
    printf("\n");
    test_insufficient_total();
    printf("\n");
    test_full_drain();
    printf("\n");
    test_repeated_draws_stay_symmetric();
    printf("\n");
    test_fuel_link_one_way();
    printf("\n");
    test_fuel_link_two_hop();
    printf("\n");
    test_fuel_link_star_symmetric();
    printf("\n");
    test_fuel_link_dual_chain();
    printf("\n");
    test_fuel_link_insufficient();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
