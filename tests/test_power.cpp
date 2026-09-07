//
// Headless test for the electrical (KSP-style EC) system (src/vehicle.h).
//
// The EC model is a SHARED POOL across the ship's battery parts (a part is a
// battery when capacity[EC] > 0 -- the capsule and the battery parts). The
// charge lives in their resources.current[EC], like propellant. Each substep
// (powerTick) does two things:
//   * gate   -- the reaction wheels (attitude control) draw power, so they
//               work only if the ship can supply it. Life support (the
//               constant draw) has priority: the wheels need power left over
//               (excess generation) or stored charge. A ship with NO EC
//               system at all is ungated (its wheels work as before).
//   * balance-- generation (RTGs) charges the pool; the constant draw (life
//               support) and the active draw (the wheels, only while they
//               are commanded) drain it.
//
// These tests pin:
//   * the gate: no EC -> ungated; empty battery + no gen -> gated; a charged
//     battery or an RTG covering the draw -> ungated; life support drains
//     the battery and, once it is empty, the ship becomes uncontrolled;
//   * the balance: an RTG charges the pool, life support drains it, and an
//     active stick adds the wheels' draw; charging/drain clamp to the pool
//     (never below 0, never above capacity);
//   * EC has NO mass: draining/charging never touches a part's mass
//     (unlike propellant).
//
// The ships here are built by hand (parts, no links needed -- the EC pool is
// ship-global, not fuel-group-based), and powerTick/drainEC/chargeEC are
// called directly, so the test stays headless (no Bullet world, no GL).
//
// Runs from the repo root:
//   make test   (or: ./test_power)

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
   would dangle every Part::def handed out so far. Each Part wraps its rigid
   body and owns its collision shape (freed by ~Body). */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* Add one power part. Any of the EC fields may be 0 (a wheel has torque +
   power_draw, an RTG has power_gen, a capsule has power_draw_constant +
   capacity[EC], a battery has only capacity[EC]). Returns the new Part. */
static Part *addPowerPart(Ship &s, double power_draw, double const_draw,
                          double power_gen, double ec_wh, double torque) {
    const double m = 100.0;   /* dry mass (EC carries no mass) */
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
    d.power_draw = power_draw;
    d.power_draw_constant = const_draw;
    d.power_gen = power_gen;
    d.torque = torque;
    if(ec_wh > 0.0) { d.capacity[(int)ResourceType::EC] = (float)ec_wh; }
    s.defs.push_back(d);

    Part *p = new Part;
    p->body = b;
    p->def = &s.defs.back();
    s.v->parts.push_back(p);
    return p;
}

/* A hand-built ship has no build_ship() to name a controller part, so name
   one (the first part) first; init() then seeds the tanks (the EC pool) and
   builds the ship's rigid body. enterWorld() is deliberately NOT called --
   there is no physics world here. */
static void initShip(Ship &s) {
    s.v->controller = s.v->parts[0];
    s.v->init();
}

static void destroyShip(Ship &s) {
    s.v->onRails = true;
    delete s.v;
}

/* Sum the EC charge across the pool (the parts' resources.current[EC]). */
static double totalEC(Ship &s) {
    double q = 0.0;
    for(Part *p : s.v->parts) {
        q += p->resources.current[(int)ResourceType::EC];
    }
    return q;
}

/* A ship with a reaction wheel but NO EC system at all: the gate is off and
   the wheel works as before (no regression for ships that never had power). */
static void test_gate_no_ec_ship_ungated() {
    printf("== Gate: a ship with no EC system is ungated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, /*draw=*/1000.0, /*const=*/0.0, /*gen=*/0.0,
                 /*ec=*/0.0, /*torque=*/2000.0);   // a wheel, no power system
    initShip(s);

    s.v->powerTick(1.0);
    CHECK_TRUE(s.v->powered_, "no-EC ship is ungated (powered_ stays true)");
    destroyShip(s);
}

/* Empty battery + no generation: the wheels are dead (the core "no power ->
   uncontrolled" rule). */
static void test_gate_empty_battery_no_gen() {
    printf("== Gate: empty battery, no gen -> gated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, /*draw=*/1000.0, /*const=*/0.0, /*gen=*/0.0,
                 /*ec=*/0.0, /*torque=*/2000.0);   // a wheel
    Part *bat = addPowerPart(s, /*draw=*/0.0, /*const=*/0.0, /*gen=*/0.0,
                             /*ec=*/1000.0, /*torque=*/0.0);  // a battery
    initShip(s);
    CHECK_NEAR(totalEC(s), 1000.0, 1e-9, "battery seeded full at build");

    s.v->drainEC(1000.0);   // run the battery empty
    CHECK_NEAR(totalEC(s), 0.0, 1e-9, "battery drained empty");

    s.v->powerTick(1.0);
    CHECK_TRUE(!s.v->powered_, "empty battery + no gen -> gated (wheels dead)");
    CHECK_NEAR(totalEC(s), 0.0, 1e-9, "still empty (nothing to drain)");
    (void)bat;
    destroyShip(s);
}

/* A charged battery (no gen) keeps the ship powered: the wheels run off the
   stored charge. */
static void test_gate_full_battery() {
    printf("== Gate: charged battery -> ungated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel
    addPowerPart(s, 0.0, 0.0, 0.0, 1000.0, 0.0);      // a battery
    initShip(s);

    s.v->powerTick(1.0);
    CHECK_TRUE(s.v->powered_, "charged battery -> ungated");
    destroyShip(s);
}

/* An RTG that generates (even with no battery) keeps the ship powered: the
   wheel runs off the generation. */
static void test_gate_rtg_covers() {
    printf("== Gate: an RTG covering the draw -> ungated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel
    addPowerPart(s, 0.0, 0.0, 300.0, 0.0, 0.0);       // an RTG
    initShip(s);

    s.v->powerTick(1.0);
    CHECK_TRUE(s.v->powered_, "RTG generation -> ungated");
    destroyShip(s);
}

/* Life support has priority over the wheels: a capsule's constant draw
   (100 W) drains its built-in battery (2000 Wh), and once the charge is
   gone the ship becomes uncontrolled -- even though the wheel could use
   power, there is none left for it. */
static void test_life_support_drains_then_gates() {
    printf("== Life support drains the battery, then gates the ship ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel
    addPowerPart(s, 0.0, 100.0, 0.0, 2000.0, 0.0);    // a capsule (100 W + 2000 Wh)
    initShip(s);
    CHECK_NEAR(totalEC(s), 2000.0, 1e-9, "capsule battery seeded full");

    /* While there is stored charge the ship is powered (the wheel may run). */
    s.v->powerTick(1.0);
    CHECK_TRUE(s.v->powered_, "powered while the battery still has charge");

    /* Run the battery fully down (100 W life support): the ship gates. */
    s.v->drainEC(2000.0);
    CHECK_NEAR(totalEC(s), 0.0, 1e-9, "battery drained to empty");
    s.v->powerTick(1.0);
    CHECK_TRUE(!s.v->powered_, "no charge + no gen -> gated (uncontrolled)");
    destroyShip(s);
}

/* An RTG charges the pool: 300 W for one hour adds 300 Wh to a half-empty
   1000 Wh battery (clamped to capacity, not above it). */
static void test_rtg_charges_battery() {
    printf("== Balance: an RTG charges the pool ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 0.0, 0.0, 300.0, 1000.0, 0.0);    // RTG + 1000 Wh battery
    initShip(s);

    s.v->drainEC(500.0);   // start half-empty
    CHECK_NEAR(totalEC(s), 500.0, 1e-9, "battery half-empty");

    s.v->powerTick(3600.0);  // 300 W x 1 h = 300 Wh
    CHECK_NEAR(totalEC(s), 800.0, 1e-6, "battery gained 300 Wh (500 -> 800)");

    /* Charging clamps at capacity: a full battery cannot over-charge. */
    s.v->chargeEC(1000.0);
    s.v->powerTick(3600.0);  // another 300 Wh requested
    CHECK_NEAR(totalEC(s), 1000.0, 1e-6, "battery clamped at capacity");
    destroyShip(s);
}

/* Life support drains the pool: 100 W for one hour removes 100 Wh from a
   full 2000 Wh battery. */
static void test_life_support_drains_pool() {
    printf("== Balance: life support drains the pool ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 0.0, 100.0, 0.0, 2000.0, 0.0);    // a capsule
    initShip(s);
    CHECK_NEAR(totalEC(s), 2000.0, 1e-9, "capsule battery seeded full");

    s.v->powerTick(3600.0);  // 100 W x 1 h = 100 Wh
    CHECK_NEAR(totalEC(s), 1900.0, 1e-6, "battery lost 100 Wh (2000 -> 1900)");
    destroyShip(s);
}

/* An active stick adds the wheels' draw: with a 100 W capsule and a 1000 W
   wheel, commanding attitude drains 1100 W (not just the 100 W life
   support). With the stick off it drains only the 100 W. */
static void test_active_wheel_draws() {
    printf("== Balance: an active stick adds the wheels' draw ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel (1000 W)
    addPowerPart(s, 0.0, 100.0, 0.0, 5000.0, 0.0);    // a capsule (100 W + 5000 Wh)
    initShip(s);
    CHECK_NEAR(totalEC(s), 5000.0, 1e-9, "battery seeded full");

    /* Stick active: 100 W (life) + 1000 W (wheel) = 1100 W for 1 h. */
    s.v->stick[1] = 1.0f;   // pitch command -> the wheel is active
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 3900.0, 1e-6, "active wheel drains 1100 Wh");
    s.v->stick[1] = 0.0f;

    /* Stick off: only the 100 W life support for 1 h. */
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 3800.0, 1e-6, "stick off drains only life support");
    destroyShip(s);
}

/* The pool drains pro-rata across batteries (the shares are proportional to
   each battery's charge), and never below zero. Two equal batteries each
   give up half. */
static void test_drain_prorata_across_batteries() {
    printf("== Balance: the pool drains pro-rata across batteries ==\n");
    Ship s; s.v = new Vehicle;
    Part *b0 = addPowerPart(s, 0.0, 0.0, 0.0, 1000.0, 0.0);   // 1000 Wh
    Part *b1 = addPowerPart(s, 0.0, 0.0, 0.0, 1000.0, 0.0);   // 1000 Wh
    initShip(s);

    s.v->drainEC(600.0);   // 600 Wh off 2000: 300 each (pro-rata)
    CHECK_NEAR(b0->resources.current[(int)ResourceType::EC], 700.0, 1e-5,
               "battery 0 drained 300 (1000 -> 700)");
    CHECK_NEAR(b1->resources.current[(int)ResourceType::EC], 700.0, 1e-5,
               "battery 1 drained 300 (1000 -> 700)");

    /* Drain past the total: clamped to zero, no negative float round-off. */
    s.v->drainEC(5000.0);
    CHECK_NEAR(b0->resources.current[(int)ResourceType::EC], 0.0, 1e-9,
               "battery 0 empty");
    CHECK_NEAR(b1->resources.current[(int)ResourceType::EC], 0.0, 1e-9,
               "battery 1 empty");
    CHECK_TRUE(b0->resources.current[(int)ResourceType::EC] >= 0.0f,
               "battery 0 not negative");
    CHECK_TRUE(b1->resources.current[(int)ResourceType::EC] >= 0.0f,
               "battery 1 not negative");
    destroyShip(s);
}

/* EC has NO mass: draining and charging the pool must not change a part's
   mass (unlike propellant). */
static void test_ec_no_mass_change() {
    printf("== EC has no mass: draining/charging leaves mass unchanged ==\n");
    Ship s; s.v = new Vehicle;
    Part *bat = addPowerPart(s, 0.0, 0.0, 0.0, 1000.0, 0.0);   // a battery
    initShip(s);
    const double m0 = bat->body->mass;

    s.v->drainEC(400.0);
    CHECK_NEAR(bat->body->mass, m0, 1e-12, "drain: mass unchanged");
    s.v->chargeEC(800.0);
    CHECK_NEAR(bat->body->mass, m0, 1e-12, "charge: mass unchanged");
    CHECK_NEAR(bat->resources.current[(int)ResourceType::EC], 1000.0, 1e-6,
               "charge restored the pool to full");
    destroyShip(s);
}

int main() {
    test_gate_no_ec_ship_ungated();
    printf("\n");
    test_gate_empty_battery_no_gen();
    printf("\n");
    test_gate_full_battery();
    printf("\n");
    test_gate_rtg_covers();
    printf("\n");
    test_life_support_drains_then_gates();
    printf("\n");
    test_rtg_charges_battery();
    printf("\n");
    test_life_support_drains_pool();
    printf("\n");
    test_active_wheel_draws();
    printf("\n");
    test_drain_prorata_across_batteries();
    printf("\n");
    test_ec_no_mass_change();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
