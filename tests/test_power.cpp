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
//     the battery and, once it is empty, the ship becomes uncontrolled; the
//     gate asks what the drums COST (the enable mask, keyed or not), so an RTG
//     that beats life support but not the drums cannot steer on an empty pool
//     (#642); the cut latches until the pool banks a maneuver -- or half a
//     store, for a pool too small to bank one -- instead of chattering at
//     substep rate;
//   * the balance: an RTG charges the pool, life support drains it, and an
//     active stick adds the wheels' draw; charging/drain clamp to the pool
//     (never below 0, never above capacity);
//   * who pays: a drum bills its power_draw wherever it lives, including a
//     command pod that is also life support and the battery (#596);
//   * the rails path: a hull coasting on its conic is billed once per WARP HOP,
//     by railsTick itself (#647) -- the air bills and the RTG credits, the drums
//     do not, because there is no control pass out there; the bill runs even when
//     the park is frozen, and not at all for a step of 0 or a hull under physics;
//     the brown-out latch keeps running across the warp, and both of its
//     thresholds are durations, so a minute-long hop cannot raise the cut;
//   * EC has NO mass: draining/charging never touches a part's mass
//     (unlike propellant);
//   * the store's precision: a substep's worth of Wh still moves a big
//     battery, which is what powerTick actually hands it (#379).
//
// The ships here are built by hand (parts, no links needed -- the EC pool is
// ship-global, not fuel-group-based), and powerTick/drainEC/chargeEC are
// called directly, so the test stays headless (no Bullet world, no GL).
//
// Runs from the repo root:
//   make test   (or: ./test_power)

#include "btcommon.h"

#include <cmath>
#include <cstdio>
#include <deque>
#include <utility>
#include <vector>

#include "vehicle.h"   // Vehicle + PartDef + ResourceContent (inline)
#include "timestep.h"  // kMaxSubStep, which kWheelsCutS deliberately equals

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
    b->btBody = new btRigidBody(ci);
    b->shape = b->btBody->getCollisionShape();
    b->mass = m;

    PartDef d;
    d.power_draw = power_draw;
    d.power_draw_constant = const_draw;
    d.power_gen = power_gen;
    d.torque = torque;
    if(ec_wh > 0.0) { d.capacity[(int)ResourceType::EC] = ec_wh; }
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
   wheel runs off the generation. The RTG has to outrun the WHEELS too, not just
   the constant draw (#642): a hull with no store cannot bank anything, so with a
   flat bill it either can afford its drums or cannot run them at all. That is
   binary in a way a player will feel -- a 300 W rtg and a 1000 W block used to
   steer for free -- and #648 is what makes the middle ground exist. */
static void test_gate_rtg_covers() {
    printf("== Gate: an RTG covering the draw -> ungated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel
    addPowerPart(s, 0.0, 0.0, 1500.0, 0.0, 0.0);      // an RTG that pays for the drums
    initShip(s);

    s.v->powerTick(1.0);
    CHECK_TRUE(s.v->powered_, "RTG generation -> ungated");
    destroyShip(s);
}

/* The other side of the same pairing: a store-less hull whose RTG is smaller than
   its drums gets no wheel authority at all, keyed or not. Before #642 this hull
   steered forever off a 300 W generator running 1000 W of drums. */
static void test_gate_rtg_too_small_no_store() {
    printf("== Gate: an RTG smaller than the drums, no store -> gated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel
    addPowerPart(s, 0.0, 0.0, 300.0, 0.0, 0.0);       // an RTG that cannot pay for it
    initShip(s);

    s.v->powerTick(1.0);
    CHECK_TRUE(not s.v->powered_, "no store and no surplus: the drums cannot run");
    destroyShip(s);
}

/* The gate used to ask only whether generation beat LIFE SUPPORT, so any hull
   whose RTG outran its capsule steered forever: the pool reached 0 Wh, drainEC()
   had nothing to take, and the shortfall was dropped on the floor while the drums
   kept working (#642, #418's other half). It asks what the drums COST now, so an
   RTG that pays for breathing but not for turning cannot turn. */
static void test_gate_pays_the_wheels_demand() {
    printf("== Gate: an RTG that covers life support but not the wheels -> gated ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);   // a wheel: 1000 W
    addPowerPart(s, 0.0, 100.0, 200.0, 0.0, 0.0);     // 100 W life support + a 200 W RTG
    initShip(s);

    /* The gate asks the COST, not the bill: with an empty pool these drums are
       unaffordable whether or not anyone is holding a key, so the hull is gated
       before the first command. A verdict that moved with the stick would let the
       Power row promise authority the next keypress refuses. */
    s.v->powerTick(1.0);
    CHECK_TRUE(not s.v->powered_, "drums the hull cannot afford gate it, stick or no stick");
    s.v->Command(ShipCmd(Pitch, 1.0f), true);
    s.v->powerTick(1.0);
    CHECK_TRUE(not s.v->powered_, "200 W pays 100 W of air but not 1000 W of drums");
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9,
               "a gated drum bills nothing, ask held");
    /* What the gate asks is not the billed figure: the billed figure is the gate's
       own verdict, so reading it back would let the wheels re-arm the substep after
       cutting them. */
    CHECK_NEAR(s.v->wheelDemandWatts(), 1000.0, 1e-9,
               "the cost survives the gate that refused it");
    /* Letting go of the stick is not a power fix: the same hull stays gated. */
    s.v->Command(ShipCmd(Pitch, 0.0f), true);
    s.v->powerTick(1.0);
    CHECK_TRUE(not s.v->powered_, "releasing the ask is not a power fix (#642)");
    CHECK_TRUE(s.v->wheels_brown_out_, "and the cut stays latched, not waiting for a key");
    destroyShip(s);
}

/* The boundary #642 warns about: cut the wheels and the unspent surplus starts
   recharging the pool, which re-arms them, which drains it again. One threshold
   makes that a relaxation oscillator at the SUBSTEP rate -- a shuddering hull and
   a duty cycle set by how finely the sim happens to step. So the cut latches: the
   drums stay off until the pool banks kWheelsRecoverS of commanding. */
static void test_brown_out_latches_then_rearms() {
    printf("== Gate: a dry pool latches the wheels off, then a banked pool re-arms ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);    // wheel: 1000 W
    addPowerPart(s, 0.0, 100.0, 500.0, 20.0, 0.0);     // 100 W air, 500 W RTG, 20 Wh
    initShip(s);
    s.v->Command(ShipCmd(Pitch, 1.0f), true);

    /* 20 Wh against a 600 W shortfall (100 air + 1000 drums - 500 RTG) is 120 s of
       commanding, i.e. 15000 substeps at the sim's own h. */
    const double h = 1.0 / 125.0;
    int ticks = 0;
    while(s.v->powered_ && ticks < 20000) { s.v->powerTick(h); ticks++; }
    CHECK_TRUE(not s.v->powered_, "the pool runs out mid-maneuver -> wheels cut");
    CHECK_TRUE(ticks > 14500 && ticks < 15500,
               "20 Wh lasts about 120 s of commanding, not 0 and not forever");

    /* Held off while the surplus rebuilds the pool: it re-arms only at a banked
       maneuver, so the cut spans many substeps rather than one. The cut lasts
       2.78 Wh / (400 W surplus) = 25 s -- a decision the player can watch, where a
       single threshold would have chattered at 125 Hz. The store is deliberately
       20 Wh: half of it (10 Wh) sits ABOVE the bank, so the bank is what decides. */
    const double recover_wh = Vehicle::kWheelsRecoverS * 1000.0 / 3600.0;  // 2.78 Wh
    CHECK_NEAR(Vehicle::kWheelsRecoverS, 10.0, 0.0,
               "the hysteresis window is 10 s of commanding");
    CHECK_NEAR(Vehicle::kWheelsRecoverS / Vehicle::kWheelsCutS, 100.0, 1.0,
               "the re-arm bank is 100x the burst the cut asks for, so the cycle is "
               "the power balance's rather than the step rate's");
    int cut = 0;
    while(not s.v->powered_ && cut < 20000) { s.v->powerTick(h); cut++; }
    CHECK_TRUE(s.v->powered_, "a banked pool brings the wheels back");
    CHECK_NEAR(totalEC(s), recover_wh, 0.05,
               "it re-arms at the recovery margin, not at the first Wh back");
    CHECK_NEAR(cut * h, recover_wh * 3600.0 / 400.0, 0.02,
               "the cut is one ~25 s drop, not a substep-rate flicker");
    s.v->Command(ShipCmd(Pitch, 0.0f), true);
    destroyShip(s);
}

/* A hull coasting on rails is billed once per WARP HOP, from railsTick (#647) -- the
   substep loop that bills everything else skips railed hulls, so before this a warped
   transfer spent nothing at all. Two things have to be true out there: the air and the
   generation still bill, and the drums do not, because there is no control pass to turn
   them even if the stick is still held from before the warp. */
static void test_rails_hop_bills_air_not_drums() {
    printf("== Rails: a warp hop bills the air, never the drums ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);     // a wheel: 1000 W
    addPowerPart(s, 0.0, 100.0, 0.0, 1000.0, 0.0);      // 100 W air + a 1000 Wh store
    initShip(s);
    s.v->Command(ShipCmd(Pitch, 1.0f), true);           // held into the warp
    s.v->onRails = true;

    s.v->powerTick(3600.0);   // one hour in a single hop
    CHECK_NEAR(totalEC(s), 900.0, 0.5,
               "an hour of rails costs the 100 Wh of air, not the 1100 Wh of air plus drums");
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9,
               "a railed hull's drums are not engaged, so the readout bills nothing either");
    CHECK_NEAR(s.v->wheelDemandWatts(), 1000.0, 1e-9,
               "while the gate still weighs what those drums would cost");
    CHECK_TRUE(s.v->powered_, "900 Wh left and a 100 W bill is not a brown-out");

    /* The latch runs across the warp. It used to be frozen there (nothing called
       powerTick), so a hull could leave the rails uncontrolled with a full pool, or
       stay controlled with an empty one. */
    for(int i = 0; i < 12; i++) { s.v->powerTick(3600.0); }
    CHECK_NEAR(totalEC(s), 0.0, 1e-9, "the pool runs out on the rails");
    CHECK_TRUE(s.v->wheels_brown_out_, "and the latch fires there rather than freezing");
    CHECK_TRUE(not s.v->powered_, "so the hull comes off the rails uncontrolled");
    destroyShip(s);
}

/* The other rails hazard: the cut threshold must be a DURATION, not the caller's step.
   Off rails the step is bounded by kMaxSubStep, so scaling by it is invisible; a warp
   hop is minutes, and a step-scaled threshold would demand 16.7 Wh here -- more than
   the whole store -- and brown out every hull that warped. */
static void test_rails_step_does_not_move_the_cut() {
    printf("== Rails: a minute-long hop does not raise the cut threshold ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 5.0, 2000.0);     // 1000 W drums + a 5 Wh store
    initShip(s);
    s.v->Command(ShipCmd(Pitch, 1.0f), true);
    s.v->onRails = true;

    s.v->powerTick(60.0);   // a minute of warp in one hop
    CHECK_TRUE(s.v->powered_,
               "the cut asks for kWheelsCutS of commanding, not for this hop's Wh");
    CHECK_NEAR(Vehicle::kWheelsCutS, kMaxSubStep, 0.0,
               "the burst the pool must cover is one max substep -- stated, not inherited");
    destroyShip(s);
}

/* And the wiring itself: railsTick(), not powerTick() called by hand. railsTick is the
   only thing that bills a railed hull, and two of its clauses have no other cover. The
   bill runs BEFORE the railFrozen return, so a hull a warp parked on the ground still
   spends the air -- it is off the substep loop like every other railed hull. It does not
   run for a step of 0, which is Game::recoverActive re-homing a hull from the UI with no
   time passed, nor for a hull that is not on rails, which the substep loop bills: billing
   that one here too is how a hop that reached the air mid-tick paid for itself twice. */
static void test_rails_park_bills_per_hop() {
    printf("== Rails: railsTick bills the hop, frozen park included ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 0.0, 100.0, 0.0, 1000.0, 0.0);   // 100 W air + a 1000 Wh store
    initShip(s);
    s.v->onRails = true;
    s.v->railFrozen = true;                         // parked by a warp landing

    s.v->railsTick(0.0, 3600.0);
    CHECK_NEAR(totalEC(s), 900.0, 0.5,
               "a frozen park still spends the air, one hop at a time");
    s.v->railsTick(0.0, 0.0);
    CHECK_NEAR(totalEC(s), 900.0, 1e-9,
               "a UI re-home passes step 0: no time passed, so nothing to bill");
    s.v->onRails = false;
    s.v->railsTick(0.0, 3600.0);
    CHECK_NEAR(totalEC(s), 900.0, 1e-9,
               "a hull under physics is billed by the substep loop, never twice");
    CHECK_TRUE(s.v->powered_, "900 of 1000 Wh is no reason to cut the wheels");
    destroyShip(s);
}

/* The other re-arm clause: a store too small to ever bank a whole maneuver. A 1 kW
   block wants 2.78 Wh back; a 3 Wh pool cannot hold a margin that size and still be
   worth anything, so half a store re-arms it instead. Without that clause the bank
   would be unreachable and the hull permanently browned out -- no shipped part
   combination reaches it today (the smallest store is 2000 Wh), which is exactly why
   it needs a pin rather than a footnote. */
static void test_brown_out_rearm_backstop() {
    printf("== Gate: a store too small to bank re-arms at half full ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 1000.0, 0.0, 0.0, 0.0, 2000.0);    // wheel: 1000 W
    addPowerPart(s, 0.0, 100.0, 500.0, 3.0, 0.0);      // 100 W air, 500 W RTG, 3 Wh
    initShip(s);
    s.v->Command(ShipCmd(Pitch, 1.0f), true);

    const double h = 1.0 / 125.0;
    int ticks = 0;
    while(s.v->powered_ && ticks < 20000) { s.v->powerTick(h); ticks++; }
    CHECK_TRUE(not s.v->powered_, "a 3 Wh pool runs out after ~18 s of commanding");
    int cut = 0;
    while(not s.v->powered_ && cut < 20000) { s.v->powerTick(h); cut++; }
    CHECK_TRUE(s.v->powered_, "and it re-arms rather than staying cut forever");
    CHECK_NEAR(totalEC(s), 0.5 * 3.0, 0.05, "re-armed at half the store");
    CHECK_TRUE(totalEC(s) < Vehicle::kWheelsRecoverS * 1000.0 / 3600.0,
               "half a store bound the bank, as it must on a small pool");
    s.v->Command(ShipCmd(Pitch, 0.0f), true);
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

    /* Stick active: 100 W (life) + 1000 W (wheel) = 1100 W for 1 h. The stick
       is only reachable through Command (#575), which is also the point: the
       wheel is made to work by a pitch command, and the same call at zero
       releases it. */
    s.v->Command(ShipCmd(Pitch, 1.0f), true);
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 3900.0, 1e-6, "active wheel drains 1100 Wh");
    s.v->Command(ShipCmd(Pitch, 0.0f), true);

    /* Stick off: only the 100 W life support for 1 h. */
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 3800.0, 1e-6, "stick off drains only life support");

    /* The same sum the power block on the Game Debug overlay prints (#575), pinned on
       both gates: the wheel bills its draw only while the stick is working it AND the
       hull has power. The row used to re-derive this and dropped the power half, so an
       unpowered hull showed a draw it was not paying. */
    s.v->Command(ShipCmd(Pitch, +1.0f), true);
    CHECK_NEAR(s.v->wheelDrawWatts(), 1000.0, 1e-9, "commanded: the wheel's 1000 W");
    s.v->powered_ = false;
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9,
               "no power: an electric actuator bills nothing, stick or no stick");
    s.v->powered_ = true;
    /* The third gate (#425 D-2): a hull that has the drums switched off has no wheel
       rows to spend, so it must not pay for them either. Wheels::wheelsPowered() is
       the one door both halves ask through. */
    s.v->wheels_enabled = false;
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9,
               "switched off: a drum that is not working bills no EC");
    s.v->wheels_enabled = true;
    CHECK_NEAR(s.v->wheelDrawWatts(), 1000.0, 1e-9, "switched back on: billing again");
    s.v->Command(ShipCmd(Pitch, 0.0f), true);
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9, "released stick: nothing to bill");
    destroyShip(s);
}

/* A pod's own drums pay (#596). Every shipped capsule carries a torque rating, so
   Part::isWheel() -- which asks the rating, not the part type -- counts the pod as a
   wheel and wheelDrawWatts() bills it like any other housing. No capsule used to author
   a power_draw, so commanding a pod-only hull added nothing to its drain: the Vessel
   window's Active row read 0.0 W and its pool fell at life-support rate however hard the
   stick worked, while the same authority through a wheel block ate the pool that much
   faster. The housing does not set the bill -- the rating does. (That the gate closes
   over a pod at all is life support's doing and test_life_support_drains_then_gates
   already pins it; what is pinned here is that the drums belong on the bill.) */
static void test_a_pods_own_drums_pay() {
    printf("== Balance: a pod's built-in drums bill their own power ==\n");
    Ship s; s.v = new Vehicle;
    /* One part doing three jobs, like every capsule in the catalog: 100 W of life
       support, the ship's only 2000 Wh battery, and 200 N m of built-in drums. */
    addPowerPart(s, /*draw=*/100.0, /*const=*/100.0, /*gen=*/0.0,
                 /*ec=*/2000.0, /*torque=*/200.0);
    initShip(s);
    CHECK_NEAR(totalEC(s), 2000.0, 1e-9, "pod battery seeded full");

    s.v->Command(ShipCmd(Pitch, 1.0f), true);
    CHECK_NEAR(s.v->wheelDrawWatts(), 100.0, 1e-9,
               "on a wheel-less hull the pod's own drums are the Active row");
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 1800.0, 1e-6, "commanding a pod drains life support AND drums");
    s.v->Command(ShipCmd(Pitch, 0.0f), true);
    s.v->powerTick(3600.0);
    CHECK_NEAR(totalEC(s), 1700.0, 1e-6, "coasting drains life support alone");

    /* With the pool empty the gate closes and the pod's own drums are dead. Life support
       empties a pod either way (#418); what this rig adds is that the drums are part of
       what drains it, and that a gated drum bills nothing even with the stick held. */
    s.v->drainEC(totalEC(s));
    s.v->powerTick(1.0);
    CHECK_TRUE(not s.v->powered_, "an empty pod pool gates its own drums");
    s.v->Command(ShipCmd(Pitch, 1.0f), true);
    CHECK_NEAR(s.v->wheelDrawWatts(), 0.0, 1e-9,
               "a gated drum bills nothing even with the stick held");
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

/* A substep's worth of Wh must survive the store. powerTick runs per substep,
   so on a big battery one substep is a tiny number: power_test's 600 W capsule
   behind a 298.45 W RTG nets 301.55 W, and at h=1/180 s that is 4.65e-4 Wh.
   resources.current used to be float, where half an ULP at 12000 is 4.9e-4 --
   so `have - share` rounded straight back to `have` and the ship lost NOTHING
   while coasting, forever: the EC gate was decorative and the battery bar
   stayed green (#379). Every other test here calls powerTick with h = 1 s or
   3600 s, which is why none of them saw it. */
static void test_substep_drain_survives_the_store() {
    printf("== Balance: a substep's Wh survives a big battery ==\n");
    Ship s; s.v = new Vehicle;
    addPowerPart(s, 0.0, 600.0, 298.45, 12000.0, 0.0);  // capsule + RTG + 12 kWh
    initShip(s);
    CHECK_NEAR(totalEC(s), 12000.0, 1e-9, "battery seeded full");

    const double h = 1.0 / 180.0;   // the substep the sim actually takes
    for(int i = 0; i < 1800; i++) { s.v->powerTick(h); }   // 10 s of flight

    /* 301.55 W for 10 s = 0.8376 Wh. Small in absolute terms, and the whole
       point is that it is not zero. */
    CHECK_NEAR(totalEC(s), 12000.0 - 301.55 * 10.0 / 3600.0, 1e-6,
               "1800 substeps of a 301.55 W net draw move the charge");
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
    test_gate_rtg_too_small_no_store();
    printf("\n");
    test_gate_pays_the_wheels_demand();
    printf("\n");
    test_brown_out_latches_then_rearms();
    printf("\n");
    test_brown_out_rearm_backstop();
    printf("\n");
    test_rails_hop_bills_air_not_drums();
    printf("\n");
    test_rails_step_does_not_move_the_cut();
    printf("\n");
    test_rails_park_bills_per_hop();
    printf("\n");
    test_life_support_drains_then_gates();
    printf("\n");
    test_rtg_charges_battery();
    printf("\n");
    test_life_support_drains_pool();
    printf("\n");
    test_active_wheel_draws();
    printf("\n");
    test_a_pods_own_drums_pay();
    printf("\n");
    test_drain_prorata_across_batteries();
    printf("\n");
    test_substep_drain_survives_the_store();
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
