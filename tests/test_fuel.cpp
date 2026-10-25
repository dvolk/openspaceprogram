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
//     and a flow above the total is refused with no partial drain;
//   * the mass and delta-v figures keep the store's precision -- a draw under
//     a float ULP at 200 t still moves them (#382).
//
// Links the REAL SetMass (src/physics.cpp, as test_thrust.cpp does), so the
// part-mass shedding + Bullet inertia update is exercised too. No GL context
// is needed. The ships here are built by hand: parts + weld links (just the
// Part* adjacency for buildFuelGroups -- no Bullet constraint, so it stays
// headless).
//
// Runs from the repo root:
//   make test   (or: ./test_fuel)

#include "btcommon.h"

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
   fuel wall (decoupler); rcsThrust > 0 -> an RCS pod (hyd kg of hydrazine
   in its own capacity). Returns the new Part (index parts.size()-1). */
static Part *addPart(Ship &s, float h2, float lox, bool engine = false,
                     bool barrier = false, double rcsThrust = 0.0,
                     float hyd = 0.0f) {
    /* DRY structure mass only. The fuel (h2/lox) is NOT baked into the body:
       init() seeds it into resources.current, and Part::effectiveMass adds it
       back (body dry + contents), so the ship's mass is 100 + h2 + lox either
       way. The box shape's local inertia is per-kg, scaled to effectiveMass by
       the compound build -- the same "shape lighter than effectiveMass" case
       crew already exercises. */
    const double m = 100.0;
    btBoxShape *shape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
    btVector3 I;
    shape->calculateLocalInertia(m, I);
    btRigidBody::btRigidBodyConstructionInfo ci(m, 0, shape, I);
    Body *b = new Body;
    b->btBody = new btRigidBody(ci);
    b->shape = b->btBody->getCollisionShape();
    b->mass = m;

    PartDef d;
    d.capacity[(int)ResourceType::Hydrogen] = h2;
    d.capacity[(int)ResourceType::LOX] = lox;
    if(engine) {
        d.setPropellantRate(ResourceType::Hydrogen, 1.0);
        d.setPropellantRate(ResourceType::LOX, 1.0);
        d.exhaust_velocity = 100.0;
    }
    d.fuel_barrier = barrier;
    d.rcs_thrust = rcsThrust;
    d.capacity[(int)ResourceType::Hydrazine] = hyd;
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
        snprintf(buf, sizeof buf, "tank %d's effective mass shed its 30 kg", i);
        CHECK_NEAR(tanks[i]->effectiveMass(), 270.0, 1e-5, buf);
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
    CHECK_NEAR(t0->effectiveMass(), 220.0, 1e-6, "tank 0 effective mass untouched");
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

/* This test's engine is a chemical one drawing H2 and LOX at equal rates, so
   a tick's burn is limited by its scarcest propellant. THE regression this
   pins: the old code drained
   each propellant in a separate drain-or-nothing call joined by `and`, so
   with H2 >= flow > LOX the H2 was drained (mass off the ship) and the LOX
   draw refused -- a leak, no thrust. Now the burn is sized from
   min(H2, LOX, flow) BEFORE draining, so the two drain in LOCKSTEP and the
   scarcest one limits the burn (thrust scales with the achieved flow).
   Driven through the real ApplyThrust via the public test accessor. */
static void test_applythrust_scarcest_limits_burn() {
    printf("== ApplyThrust: scarcest propellant limits the burn (no leak) ==\n");

    /* LOX is the limiting propellant: H2 100 kg, LOX 10 kg. Each of the
       engine's propellant rates is 1.0 kg/s (addPart), thruster_util 1.0, so
       step 50 -> desired 50 kg each -- above LOX (10) and below H2 (100):
       the leak scenario. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, /*engine=*/true);
        Part *tank = addPart(s, 100, 10);
        link(s, eng, tank);
        initShip(s);

        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(tank->resources.current[(int)ResourceType::LOX], 0.0, 1e-9,
                   "LOX fully consumed (10 -> 0)");
        /* Lockstep: H2 drained exactly what LOX did (10), NOT the full flow
           (50). The old leaky code drained 50 of H2 and left LOX at 10. */
        CHECK_NEAR(tank->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-6,
                   "H2 drained 10 (lockstep with LOX), not the full 50 (the leak)");
        /* The engine fired at a partial thrust proportional to the 10/50 burn
           (full = 2*1.0*100 = 200 N; 200 * 10/50 = 40 N). */
        CHECK_NEAR(eng->armedThrust, 40.0, 1e-5, "partial thrust = 200 * (10/50)");
        destroyShip(s);
    }

    /* Mirror: H2 is the limiting propellant (H2 10, LOX 100). Same lockstep,
       the other way round. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, true);
        Part *tank = addPart(s, 10, 100);
        link(s, eng, tank);
        initShip(s);

        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(tank->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
                   "H2 fully consumed (10 -> 0)");
        CHECK_NEAR(tank->resources.current[(int)ResourceType::LOX], 90.0, 1e-6,
                   "LOX drained 10 (lockstep with H2), not the full 50");
        destroyShip(s);
    }

    /* Both plentiful (H2 100, LOX 100): the burn is the full flow (50), both
       drain 50, full thrust -- unchanged from before the fix. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, true);
        Part *tank = addPart(s, 100, 100);
        link(s, eng, tank);
        initShip(s);

        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(tank->resources.current[(int)ResourceType::Hydrogen], 50.0, 1e-6,
                   "H2 drained the full flow (100 -> 50)");
        CHECK_NEAR(tank->resources.current[(int)ResourceType::LOX], 50.0, 1e-6,
                   "LOX drained the full flow (100 -> 50)");
        CHECK_NEAR(eng->armedThrust, 200.0, 1e-5, "full thrust (both plentiful)");
        destroyShip(s);
    }
}

/* #371: the two propellants live in SEPARATE tanks holding different amounts,
   so the scarce one would run out first. ApplyThrust sizes the whole burn by
   s = min over propellants of (avail / desired) BEFORE draining, so the tanks
   drain in lockstep and the scarce propellant caps the burn. The leak this
   pins: drain H2 first, fail the LOX draw, and the H2 is gone with
   armedThrust still 0 -- the ship lighter by propellant that bought no
   delta-v. Both scarcity orders, a crossfeed layer that reaches the plentiful
   tank first, and two engines sharing one pool in the same tick. */
static void test_applythrust_unbalanced_tanks() {
    printf("== ApplyThrust: separate propellant tanks, scarce one caps the burn ==\n");

    /* LOX is scarce: 10 kg against 100 kg of H2. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, true);
        Part *h2tank = addPart(s, 100, 0);
        Part *loxtank = addPart(s, 0, 10);
        link(s, eng, h2tank); link(s, h2tank, loxtank);
        initShip(s);
        s.v->dv_log_enabled = true;

        const double mass0 = s.v->getMass();
        CHECK_NEAR(mass0, 410.0, 1e-9, "3 dry parts (100 kg) + 100 H2 + 10 LOX");
        s.v->ApplyThrust_TESTONLY(50.0);   /* wants 50 kg of each propellant */

        CHECK_NEAR(loxtank->resources.current[(int)ResourceType::LOX], 0.0, 1e-9,
                   "LOX tank emptied (10 -> 0)");
        CHECK_NEAR(h2tank->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-6,
                   "H2 drained 10 in lockstep, not the full 50 (the leak)");
        CHECK_NEAR(eng->armedThrust, 40.0, 1e-5, "partial thrust = 200 * (10/50)");
        CHECK_NEAR(s.v->propSpent_, 20.0, 1e-6, "propSpent counts both propellants");
        CHECK_NEAR(mass0 - s.v->getMass(), 20.0, 1e-5,
                   "the ship shed exactly the propellant it booked");

        /* Next tick: LOX is gone, so s = 0 -- nothing may be drained (the
           remaining H2 is stranded, not quietly burned as if it had oxidizer).
           ApplyThrust never clears armedThrust (the tick loop's clearThrust
           does), so reset it to see the no-thrust outcome. */
        eng->armedThrust = 0.0f;
        s.v->ApplyThrust_TESTONLY(50.0);
        CHECK_NEAR(h2tank->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-6,
                   "no oxidizer: the next tick drains no H2 at all");
        CHECK_NEAR(s.v->propSpent_, 20.0, 1e-6, "and books no more propellant");
        CHECK_NEAR(eng->armedThrust, 0.0, 1e-9, "and arms no thrust");
        destroyShip(s);
    }

    /* Mirror: H2 is scarce (10 kg) against 100 kg of LOX. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, true);
        Part *h2tank = addPart(s, 10, 0);
        Part *loxtank = addPart(s, 0, 100);
        link(s, eng, h2tank); link(s, h2tank, loxtank);
        initShip(s);
        s.v->dv_log_enabled = true;

        const double mass0 = s.v->getMass();
        CHECK_NEAR(mass0, 410.0, 1e-9, "3 dry parts (100 kg) + 10 H2 + 100 LOX");
        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(h2tank->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
                   "H2 tank emptied (10 -> 0)");
        CHECK_NEAR(loxtank->resources.current[(int)ResourceType::LOX], 90.0, 1e-6,
                   "LOX drained 10 in lockstep, not the full 50");
        CHECK_NEAR(eng->armedThrust, 40.0, 1e-5, "partial thrust = 200 * (10/50)");
        CHECK_NEAR(s.v->propSpent_, 20.0, 1e-6, "propSpent counts both propellants");
        CHECK_NEAR(mass0 - s.v->getMass(), 20.0, 1e-5,
                   "the ship shed exactly the propellant it booked");
        destroyShip(s);
    }

    /* Crossfeed: the PLENTIFUL H2 sits one layer FURTHER out (a fuel link), so
       the layer-by-layer drain would reach it before the scarce near LOX. An
       unscaled drain would take the full 50 kg of H2 and then fail on LOX. */
    {
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, 0, 0, true);
        Part *loxtank = addPart(s, 0, 10);   /* engine's own group: layer 0 */
        Part *h2tank = addPart(s, 100, 0);   /* own group, linked: layer 1 */
        link(s, eng, loxtank);               /* h2tank is NOT in the part tree */
        initShip(s);
        s.v->fuelLinks.push_back(Vehicle::FuelLink{h2tank, loxtank});
        s.v->dv_log_enabled = true;

        const double mass0 = s.v->getMass();
        CHECK_NEAR(mass0, 410.0, 1e-9, "3 dry parts (100 kg) + 100 H2 + 10 LOX");
        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(h2tank->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-6,
                   "far H2 layer drained 10 (not the full 50)");
        CHECK_NEAR(loxtank->resources.current[(int)ResourceType::LOX], 0.0, 1e-9,
                   "near LOX layer drained 10 to empty");
        CHECK_NEAR(eng->armedThrust, 40.0, 1e-5, "partial thrust = 200 * (10/50)");
        CHECK_NEAR(s.v->propSpent_, 20.0, 1e-6, "propSpent counts both propellants");
        CHECK_NEAR(mass0 - s.v->getMass(), 20.0, 1e-5,
                   "the ship shed exactly the propellant it booked");
        destroyShip(s);
    }

    /* Two engines draw the same tank set in ONE tick. Each engine re-sizes
       from the live tanks, so the second sees LOX at 0 and stands down: the
       H2 is drained once, not once per engine, and no thrust is armed off an
       empty pool. */
    {
        Ship s; s.v = new Vehicle;
        Part *tank = addPart(s, 100, 10);
        Part *eng1 = addPart(s, 0, 0, true);
        Part *eng2 = addPart(s, 0, 0, true);
        link(s, tank, eng1); link(s, tank, eng2);
        initShip(s);
        s.v->dv_log_enabled = true;

        const double mass0 = s.v->getMass();
        s.v->ApplyThrust_TESTONLY(50.0);

        CHECK_NEAR(tank->resources.current[(int)ResourceType::Hydrogen], 90.0, 1e-6,
                   "shared H2 drained once (10), not once per engine");
        CHECK_NEAR(tank->resources.current[(int)ResourceType::LOX], 0.0, 1e-9,
                   "shared LOX emptied by the first engine");
        CHECK_NEAR(eng1->armedThrust, 40.0, 1e-5, "first engine: partial thrust");
        CHECK_NEAR(eng2->armedThrust, 0.0, 1e-9, "second engine: empty pool, no thrust");
        CHECK_NEAR(s.v->propSpent_, 20.0, 1e-6, "the shared draw is booked once");
        CHECK_NEAR(mass0 - s.v->getMass(), 20.0, 1e-5,
                   "and shed exactly that");
        destroyShip(s);
    }
}

/* The figures a player plans a burn with must not be narrower than the store
   they are built from (#382). A float sum of a 200 t ship has ULP 0.0156 kg, so
   a gram of propellant vanishes from getMass() -- and from the delta-v that
   mass feeds -- even though the tank holds it exactly. */
static void test_mass_figure_sees_a_sub_float_ulp_draw() {
    printf("== getMass/getDeltaV: precision the store can hold, the figure can too ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *tank = addPart(s, 100000, 100000);   /* 200 t of propellant */
    link(s, eng, tank);
    initShip(s);

    const double mass0 = s.v->getMass();
    const double dv0 = s.v->getDeltaV();
    CHECK_NEAR(mass0, 200200.0, 1e-9, "2 dry parts (100 kg) + 200 t of propellant");

    CHECK_TRUE(s.v->consumeResourceMass(ResourceType::Hydrogen, 0.001, eng),
               "a 1 g draw fits");
    CHECK_NEAR(mass0 - s.v->getMass(), 0.001, 1e-6,
               "the mass figure moves by the gram, not by nothing");
    CHECK_TRUE(dv0 - s.v->getDeltaV() > 0.0,
               "delta-v moves too: the propellant inside the log is not quantized");
    destroyShip(s);

    /* The same draw against a 1 kg tank, where the stored value is nowhere near
       its double ULP, so the check can see the REQUEST's width: 0.001f is
       0.0010000000475 kg, which a 200 t tank hides under 3 ULP of its own. */
    Ship s2; s2.v = new Vehicle;
    Part *eng2 = addPart(s2, 0, 0, true);
    Part *small = addPart(s2, 1, 1);
    link(s2, eng2, small);
    initShip(s2);
    CHECK_TRUE(s2.v->consumeResourceMass(ResourceType::Hydrogen, 0.001, eng2),
               "the same 1 g draw fits");
    CHECK_NEAR(small->resources.current[(int)ResourceType::Hydrogen], 0.999, 1e-13,
               "and it is a double draw, not a float-rounded one");
    destroyShip(s2);
}

/* The last dribble of a burn. ApplyThrust sizes the burn with
   s = min(avail / desired) and then RECOMPUTES the draw as desired * s, which is
   not the same number as avail: 25 * (7/25) = 7.000000000000000888. The draw is
   refused unless the pool covers it, so that ULP used to fail the draw and trip
   assert(drained && "s must cover every draw") -- on the last tick of an
   ordinary burn. */
static void test_applythrust_last_dribble_fits() {
    printf("== ApplyThrust: the sized draw must not overshoot the tank ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 0, 0, true);
    Part *tank = addPart(s, 7, 7);   /* 7 kg of each, against a 25 kg request */
    link(s, eng, tank);
    initShip(s);

    s.v->ApplyThrust_TESTONLY(25.0);
    CHECK_NEAR(tank->resources.current[(int)ResourceType::Hydrogen], 0.0, 1e-9,
               "the H2 dribble burns instead of being refused");
    CHECK_NEAR(tank->resources.current[(int)ResourceType::LOX], 0.0, 1e-9,
               "and so does the LOX one");
    CHECK_TRUE(eng->armedThrust > 0.0, "the engine still fires on the remainder");
    destroyShip(s);
}

/* Force overlay (F4, gameui.cpp drawForceOverlay): Vehicle::lastForceVecs
   carries the force sites the LAST recorded substep actually applied to the
   hull. Pins:
     * recording happens ONLY while recordForceVecs is set -- the sim hot
       path pays nothing while the overlay is off, yet the forces still land;
     * applyControlForces clears once per substep, so a no-command substep
       leaves NO ghost arrows;
     * the recorded vectors are the applied forces: the thrust entries sum
       to lastThrustForce, and the single pod's entry equals lastRcsAch.
   One engine + one legacy RCS pod (empty rcs_nozzles -> the six axis bells),
   both parked at the origin: engine thrust +Z, rcsDir +Z allocated onto the
   -Z bell (the ship is pushed opposite the bell). */
static void test_force_overlay_recording() {
    printf("== Force overlay: lastForceVecs records the substep's applied forces ==\n");
    Ship s; s.v = new Vehicle;
    Part *eng = addPart(s, 100, 100, /*engine=*/true);
    Part *pod = addPart(s, 0, 0, false, false, /*rcsThrust=*/60.0, /*hyd=*/10.0);
    link(s, eng, pod);
    initShip(s);
    /* Park the ship 1 km out: partPos is then 1000-ish while partPos - com
       stays ~0, so the pos pins below cannot be satisfied by an absolute
       partPos regression. */
    s.v->placeShip(glm::dvec3(0.0, 0.0, 1000.0), glm::dmat3(1.0));

    s.v->ApplyThrust_TESTONLY(50.0);            // tanks plentiful -> full 200 N
    CHECK_NEAR(eng->armedThrust, 200.0, 1e-5, "engine armed at full thrust");
    s.v->rcsDir = glm::dvec3(0.0, 0.0, 1.0);    // the pod allocates 60 N along +Z
    s.v->recordForceVecs = true;
    s.v->applyControlForces(1.0 / 60.0);

    int nThrust = 0, nRcs = 0;
    glm::dvec3 thrustSum(0.0), rcsSum(0.0);
    for(const Vehicle::ForceVec &fv : s.v->lastForceVecs) {
        CHECK_TRUE(fv.kind == Vehicle::kForceThrust || fv.kind == Vehicle::kForceRcs,
                   "every recorded entry carries a known kind");
        if(fv.kind == Vehicle::kForceThrust) {
            nThrust++; thrustSum += fv.vec;
            CHECK_TRUE(glm::length(fv.pos - (s.v->partPos(eng) - s.v->comPos())) < 1e-9,
                       "thrust arrow pos is the Bullet arm (partPos rel COM)");
        } else {
            nRcs++; rcsSum += fv.vec;
            CHECK_TRUE(glm::length(fv.pos - (s.v->partPos(pod) - s.v->comPos())) < 1e-9,
                       "rcs arrow pos is the Bullet arm (partPos rel COM)");
        }
    }
    CHECK_TRUE(nThrust == 1, "one engine -> one thrust arrow");
    CHECK_TRUE(nRcs == 1, "one firing pod -> one rcs arrow (per pod, not per bell)");
    CHECK_TRUE(glm::length(thrustSum - s.v->lastThrustForce) < 1e-9,
               "thrust entries sum to the force Bullet got");
    CHECK_TRUE(glm::length(rcsSum - s.v->lastRcsAch) < 1e-9,
               "rcs entries sum to the achieved RCS force");
    CHECK_NEAR(glm::length(rcsSum), 60.0, 1e-5, "the pod delivered the demanded 60 N");

    /* A substep with no armed commands clears: the overlay must not keep
       drawing last substep's arrows. */
    s.v->clearThrust();
    s.v->clearRcs();
    s.v->applyControlForces(1.0 / 60.0);
    CHECK_TRUE(s.v->lastForceVecs.empty(), "no-command substep -> no arrows");

    /* Overlay off: forces are still applied, nothing is recorded. */
    s.v->recordForceVecs = false;
    s.v->ApplyThrust_TESTONLY(50.0);
    s.v->rcsDir = glm::dvec3(1.0, 0.0, 0.0);
    s.v->applyControlForces(1.0 / 60.0);
    CHECK_TRUE(s.v->lastForceVecs.empty(), "recording off -> nothing captured");
    CHECK_NEAR(glm::length(s.v->lastThrustForce), 200.0, 1e-9,
               "the engine still fired (200 N) while unrecorded");
    CHECK_NEAR(glm::length(s.v->lastRcsAch), 60.0, 1e-5,
               "the pod still fired (60 N) while unrecorded");
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
    printf("\n");
    test_applythrust_scarcest_limits_burn();
    printf("\n");
    test_applythrust_unbalanced_tanks();
    printf("\n");
    test_mass_figure_sees_a_sub_float_ulp_draw();
    printf("\n");
    test_applythrust_last_dribble_fits();
    printf("\n");
    test_force_overlay_recording();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
