// The attitude CAPABILITY MODEL and the COMMAND FUNNEL (#425 steps B and C):
// Vehicle::attitudeAuthority() and Vehicle::attitudeTorqueAbout() -- one answer to
// "what can turn this ship right now, and how much of it reaches THIS axis" -- plus
// Vehicle::Command(), the one door from input to control state that the model has to
// be reachable through (#424).
//
// The CONTROL-STATE DOORS get pinned here too: #575's slew restore and the armed RCS
// demand (Vehicle::armRcs, protected since #578) are reachable through a probe, so a
// door that stops doing its job shows up here. The SUIT's own doors have no pin in
// this binary -- what they do (the arm, the jump, the saved yaw offset) is covered by
// the e2e crew/eva cases, which run the real tick loop.
//
// Why this file exists: before it, three places (the slew law, the kill-rot law and
// the [slew] trace) each re-added wheelTorqueSum() + tvcAuthority()'s tvcMax, and none of
// them asked which axis. That is right about an axis TVC can lever and wrong about
// the stack axis, where a lateral nozzle deflection produces nothing at all -- which
// gimbal.h tvcSplit has always known, since it drops that component and hands it to
// the wheels. So the law budgetted authority the actuators would never deliver, and
// an autopilot could not ask the question #400 wants asked. This model is the number
// #425's allocator and #426's surface rows get built on, so it is pinned here as a
// production call, not as a re-implementation: tests/test_attitude.cpp and
// tests/test_slew3d.cpp integrate the LAW and would not notice vehicle.cpp changing.
//
// Fixture arithmetic (so the pins are hand-checkable, not read back off the code):
// two 100 kg boxes stacked on their nodes -- root half-height 1.5, engine half-height
// 1.0 -- so the engine's centre sits at z = -2.5 and Bullet's compound COM sits at
// z = -1.25. The engine's lever along the stack axis is therefore exactly 1.25 m:
//     heffMag  = T * |h|          = 800 * 1.25        = 1000    N m / rad
//     tvcTorque= heffMag * sin(gimbal_max) = 1000 * sin(0.1)    N m
//
// No physics world and no GL: init() runs rebuildCompound (the one hull body) but
// never enterWorld, exactly like test_dock / test_inertia / test_fuel.

#include "btcommon.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/geometric.hpp>

#include "vehicle.h"   // Vehicle + Part (inline); pulls in Bullet (btcommon.h)

#include "terrain.h"   // #426 handover test: a body with an atmosphere under it

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

#define CHECK_NEAR(actual, expected, tol, msg)                                 \
    do {                                                                       \
        g_checks++;                                                            \
        double _a = (actual), _e = (expected);                                 \
        /* BOTH sides. Only _a was guarded, so a pin written the other way round  \
           -- CHECK_NEAR(1.0, computed, ...) -- silently passed on a computed     \
           NaN, because fabs(1.0 - NaN) > tol is false. That is the SPELLING     \
           that fails open: a "diff > tol" test cannot see a NaN, while          \
           "!(diff <= tol)" can. #604 put this guard in every test binary's      \
           CHECK_NEAR, so a new test file wants it from its first line -- a       \
           mutation battery that reports green on checks that never ran is the   \
           cost of forgetting it. */                                             \
        if (!std::isfinite(_a) || !std::isfinite(_e)                            \
            || std::fabs(_a - _e) > (tol)) {                                    \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g, tol %.3g)\n",               \
                   msg, _a, _e, (double)(tol));                                \
        }                                                                      \
    } while (0)

/* Hand-built defs live in a deque: push_back on a vector would dangle every
   Part::def handed out so far (as in test_dock). */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* The armed RCS demand is protected, and on a SHIP its magnitude never surfaces --
   rcsWorldDir() normalizes on the way out -- so the only way to pin the invariant the
   ship and suit paths share is to read the field. Re-exposed through a subclass rather
   than widening the production API, as in test_inertia.cpp. armRcs came along when #578
   made it protected too: its callers are Command and Kerbal::armInput, both inside. */
struct RcsProbe : Vehicle {
    using Vehicle::rcsDir;
    using Vehicle::armRcs;
};

/* A box part with the fields this model reads: `torque` (>0 makes it a reaction
   wheel, `wheelAxes` says how many drums it holds -- 3 for a triad, which is what
   levers the ship's side and up lines too), `gimbal_max` (>0 makes its thrust
   steerable) and `ctlArea` (>0 makes it a control surface, with the travel and slope
   the aero step reads). The defs come in as the deque rather than a Stack so a
   fixture that is not a stack can share the builder. */
static Part *mkPart(std::deque<PartDef> &defs, const char *name, double mass, double hz,
                    double torque, double gimbalMax, int wheelAxes = 1,
                    double ctlArea = 0.0) {
    Body *b = new Body;
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(0.5f, 0.5f, (float)hz));
    b->mass   = mass;

    defs.push_back(PartDef());
    PartDef &d = defs.back();
    d.name   = name;
    d.mass   = (float)mass;
    d.radius = 0.5f;
    d.height = (float)(2.0 * hz);
    d.torque = torque;
    d.wheel_axes = wheelAxes;
    d.gimbal_max = gimbalMax;
    if(ctlArea > 0.0) {
        d.control_area = ctlArea;
        d.max_deflection = 0.35;
        d.cl = 5.0;
        d.cl_control = 5.0;
        d.stall_angle = 0.5;
        d.control_axis = ControlAxis::Pitch;
    }
    d.synthesizeNodes();

    Part *p = new Part;
    p->body = b;
    p->def  = &d;
    p->stage = 1;
    return p;
}

static void destroyShip(Vehicle *v) {
    /* ~Vehicle skips RemoveBody unless hullInWorld(), which is false here because
       init() built the compound but nothing entered it into a world (and this
       binary has no physics global to remove it from anyway). onRails is not what
       gates that -- the broadphase handle is. */
    delete v;
}

/* The fixture's stack: root at the ship origin, engine 2.5 m below it, both at
    rest at the world origin with an identity pose -- so the ship axes ARE the
    world axes and the pins can name them. */
struct Stack {
    Ship s;
    Part *root;
    Part *eng;
    glm::dvec3 x, y, z;
};

static Stack mkStack(double rootTorque, double engThrust, double gimbalMax) {
    Stack st;
    st.s.v = new Vehicle;
    st.s.v->name = "authority";
    st.root = mkPart(st.s.defs, "root", 100.0, 1.5, rootTorque, 0.0);
    st.s.v->setRoot(st.root);
    st.eng = mkPart(st.s.defs, "engine", 100.0, 1.0, 0.0, gimbalMax);
    st.s.v->attachDown(st.eng);
    st.s.v->init();
    st.s.v->placeShip(glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    st.eng->armedThrust = (float)engThrust;
    st.x = st.s.v->partAxis(st.root, 0);
    st.y = st.s.v->partAxis(st.root, 1);
    st.z = st.s.v->partAxis(st.root, 2);
    return st;
}

/* --- the fixture's own geometry, so a wrong lever shows up as a red check --- */
static void test_fixture_lever_is_what_we_think() {
    printf("== fixture: the engine sits 1.25 m off the COM along the stack axis ==\n");
    Stack st = mkStack(0.0, 800.0, 0.1);
    const glm::dvec3 com = st.s.v->comPos();
    const double lever = std::fabs(glm::dot(st.s.v->partPos(st.eng) - com, st.z));
    CHECK_NEAR(lever, 1.25, 1e-9, "1.25 m of stack-axis lever, by hand");
    const Vehicle::AttitudeAuthority a = st.s.v->attitudeAuthority();
    CHECK_NEAR(a.heffMag, 800.0 * 1.25, 1e-9, "heffMag = T*|h| = 1000 N m/rad");
    CHECK_NEAR(a.tvcTorque, 1000.0 * std::sin(0.1), 1e-9,
               "tvcTorque = heffMag*sin(gimbal_max)");
    CHECK_TRUE(a.wheels.empty(), "a root with no `torque` is not a wheel");
    destroyShip(st.s.v);
}

/* --- the shape of the two actuators --------------------------------------- */
static void test_tvc_is_a_disk_not_a_ball() {
    printf("== TVC levers the transverse plane only (#425) ==\n");
    Stack st = mkStack(0.0, 800.0, 0.1);
    Vehicle *v = st.s.v;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    const double tvc = a.tvcTorque;

    /* THE pin this whole step exists for: about the stack axis, a gimballed
       engine delivers nothing. The laws used to credit it with tvcMax there.
       Exact, not approximate: with axis == stackAxis every component of the cross
       product has identical operands, so b is 0 in IEEE, not 0 to within rounding. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 0.0, 1e-12,
               "no TVC authority about the stack axis (roll)");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), tvc, 1e-12, "full TVC pitch authority");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.y), tvc, 1e-12, "full TVC yaw authority");

    const glm::dvec3 mixed = glm::normalize(st.z + st.x);
    CHECK_NEAR(v->attitudeTorqueAbout(a, mixed), tvc * std::sqrt(0.5), 1e-9,
               "45 deg between the axes: sin(45) of it");
    /* The axis is a direction, not a magnitude: the laws normalise, and a caller
       that forgets must not be silently granted more. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x * 7.0),
               v->attitudeTorqueAbout(a, st.x), 1e-12, "scale-invariant in the axis");
    CHECK_NEAR(v->attitudeTorqueAbout(a, glm::dvec3(0.0, 0.0, 0.0)), 0.0, 0.0,
               "a zero axis buys nothing");
    /* And neither does a NaN one. slew_log used to normalise a possibly-zero target
       into exactly that; the law's early-out must survive it rather than propagate
       NaN into an acceleration. */
    const double nan = std::nan("");
    CHECK_NEAR(v->attitudeTorqueAbout(a, glm::dvec3(nan, nan, nan)), 0.0, 0.0,
               "a NaN axis buys nothing (and must not poison the caller)");
    /* The mirror case, and the nastier one: a NaN STACK axis. Clamping a NaN with
       std::fmin(1.0, nan) returns 1.0, which would credit the gimballed engine with
       its full torque about EVERY axis -- roll included. Hand-built here because the
       only way to get one from the hull is a corrupt transform, and the trap is in
       the arithmetic, not the fixture. */
    Vehicle::AttitudeAuthority junk;
    junk.wheels.push_back({ st.x, 5.0 });   // a drum ON the asked axis, so its 5 N m counts
    junk.tvcTorque = 100.0;
    junk.stackAxis = glm::dvec3(nan, nan, nan);
    CHECK_NEAR(v->attitudeTorqueAbout(junk, st.x), 5.0, 0.0,
               "a NaN stack axis grants no TVC -- not all of it at once");

    /* Authority follows the throttle: nothing armed, nothing to spend. The mixer
       arms thrust per substep, so this is what a flameout mid-slew reads. */
    st.eng->armedThrust = 0.0f;
    const Vehicle::AttitudeAuthority dry = v->attitudeAuthority();
    CHECK_TRUE(dry.tvcTorque == 0.0 && dry.heffMag == 0.0,
               "an unarmed engine contributes no attitude authority");
    destroyShip(st.s.v);

    /* A steerable engine cannot steer itself either: gimbal_max 0 leaves the
       thrust line fixed, however hard it pushes. */
    Stack fixed = mkStack(0.0, 800.0, 0.0);
    CHECK_TRUE(fixed.s.v->attitudeAuthority().tvcTorque == 0.0,
               "gimbal_max 0: thrust, but no TVC authority");
    destroyShip(fixed.s.v);

    /* A nozzle authored past 90 deg used to reach this model and hand it a NEGATIVE
       tvcTorque, which the laws read through `budget <= 0` as "this ship cannot
       turn" -- live wheels silenced by a silly number. It is refused at the boundary
       now (test_shipload's gimbal_max case) and asserted at the read, so there is
       nothing left to pin from here: such a def aborts instead of mis-budgetting, and
       the band between 90 and 180 deg, where the model used to score a wider swing as
       LESS authority, cannot be authored at all. */
}

/* --- the pair, and the gate that turns half of it off --------------------- */
static void test_wheels_and_tvc_together() {
    printf("== a stacked drum and a TVC disk, summed per axis ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    Vehicle *v = st.s.v;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    const double tvc = 1000.0 * std::sin(0.1);
    CHECK_NEAR(a.wheelRatingSum(), 200.0, 1e-12, "the wheel's rated torque");
    CHECK_NEAR(a.tvcTorque, tvc, 1e-9, "and the engine's, side by side in one answer");

    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 200.0, 1e-9,
               "about the stack axis the ship has its wheels and NOTHING else");
    /* #576: the drum in this fixture is the ROOT, mounted along the stack, so it
       levers roll and nothing else. The transverse axes belong to TVC alone -- the
       number used to be 200 + tvc, crediting a roll-only drum with pitch. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), tvc, 1e-9,
               "a stacked drum adds nothing to a transverse axis (#576)");

    /* The wheels are electric: no EC, no wheel row. Everything about the stack
       axis then goes to zero -- which is the #418 reading a player sees as a
       dead ship that will not turn, and is the truth rather than a fudge. */
    v->powered_ = false;
    const Vehicle::AttitudeAuthority dead = v->attitudeAuthority();
    CHECK_TRUE(dead.wheels.empty(), "unpowered wheels contribute nothing");
    /* The parts still carry their rating -- only the rows the laws and the Vessel
       window read go away. That is the difference the window has to get right (#572),
       and why a flat battery reads as 0 N m in the window and in flight alike (#418). */
    CHECK_NEAR(actuatorTorqueAbout(dead.wheels, st.z), 0.0, 0.0,
               "and no axis gets a wheel's rating back");
    CHECK_NEAR(v->attitudeTorqueAbout(dead, st.z), 0.0, 0.0,
               "no authority at all about the stack axis");
    CHECK_NEAR(v->attitudeTorqueAbout(dead, st.x), tvc, 1e-9,
               "the engines still turn it while they burn");
    destroyShip(st.s.v);
}

/* --- what the model is for ------------------------------------------------ */
static void test_a_ship_with_nothing_has_nothing() {
    printf("== no wheel, no armed engine: the answer is zero, not a fudge ==\n");
    Stack st = mkStack(0.0, 0.0, 0.0);
    Vehicle *v = st.s.v;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_TRUE(a.wheels.empty() && a.tvcTorque == 0.0 && a.heffMag == 0.0,
               "a coasting ship has no attitude authority");
    /* The laws' early-out (`budget <= 0` means "cannot turn, do not pretend").
       Pinned per axis, not as a sum: a pair of sign errors that cancel would pass a
       summed check, and the early-out is what makes a dead ship read as dead. */
    CHECK_TRUE(v->attitudeTorqueAbout(a, st.x) == 0.0, "no authority about pitch");
    CHECK_TRUE(v->attitudeTorqueAbout(a, st.z) == 0.0, "no authority about roll");
    destroyShip(st.s.v);
}

/* --- which wheel number, and whose frame ---------------------------------- */
/* Every wheel on the hull is a ROW, not a running total: a one-wheel ship cannot
   tell "the model kept only the biggest" from "the model summed them", so a fixture
   with one wheel would pass while the laws budgeted a third of the authority they
   need -- hence the second wheel here. The Vessel window used to print the largest
   instead of the sum (#572); it prints what the rows buy per axis now, so the swap
   this test guards is the model keeping one wheel's row and dropping the other. */
static void test_two_wheels_add_up() {
    printf("== the wheel row sums the wheels ==\n");
    Stack st = mkStack(200.0, 0.0, 0.0);
    Vehicle *v = st.s.v;
    Part *w2 = mkPart(st.s.defs, "wheel2", 100.0, 1.0, 50.0, 0.0);
    v->attachDown(w2);
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_TRUE(a.wheels.size() == 2, "both wheels are rows, not one summed number");
    CHECK_NEAR(a.wheelRatingSum(), 250.0, 1e-12, "200 + 50 N m of wheel, both of them");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 250.0, 1e-9,
               "two drums on the stack line buy the sum about it");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), 0.0, 0.0,
               "and nothing about an axis neither drum is on (#576)");
    destroyShip(st.s.v);
}

/* A CLUSTER COVERS THE AXES ITS DRUMS POINT AT (#576's payoff, and the shape #426's
   fin ring will have: several rows, several lines, one allocator deciding what a
   demand can get). No shipped hull looks like this yet -- every capsule and reaction
   wheel mounts stacked -- so a cluster has to be built by hand, with the two off-stack
   drums rotated by hand the same way the sideways_wheel fixture does. */
static void test_a_cluster_covers_an_axis_it_points_at() {
    printf("== a drum only covers an axis it points at ==\n");
    Ship s;
    s.v = new Vehicle;
    s.v->name = "cluster";
    /* Three drums on three lines: 100 N m on the stack (the root's own), 50 N m
       pointed along the ship's -Y, 20 N m pointed along the ship's +X. */
    Part *root = mkPart(s.defs, "root", 100.0, 1.5, 100.0, 0.0);
    s.v->setRoot(root);
    Part *drumY = mkPart(s.defs, "drum_y", 100.0, 1.0, 50.0, 0.0);
    s.v->attachDown(drumY);
    Part *drumX = mkPart(s.defs, "drum_x", 100.0, 1.0, 20.0, 0.0);
    s.v->attachDown(drumX);
    /* attachDown() solves localRot off the stack node, so the tilt goes on after it,
       exactly as the sideways-wheel fixture below does. */
    drumY->localRot = glm::dmat3(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);  // +90 about X: +Z -> -Y
    drumX->localRot = glm::dmat3(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);  // +90 about Y: +Z -> +X
    s.v->init();
    s.v->placeShip(glm::dvec3(0.0, 200.0, 0.0), glm::dmat3(1.0));
    const Vehicle::AttitudeAuthority a = s.v->attitudeAuthority();
    const glm::dvec3 upX = s.v->partAxis(root, 0);
    const glm::dvec3 upY = s.v->partAxis(root, 1);
    const glm::dvec3 upZ = s.v->partAxis(root, 2);
    CHECK_TRUE(a.wheels.size() == 3, "three rows on three lines");
    CHECK_NEAR(a.wheelRatingSum(), 170.0, 1e-12, "the row sum is 100 + 50 + 20");
    CHECK_NEAR(s.v->attitudeTorqueAbout(a, upZ), 100.0, 1e-9, "the stack axis only has the root's drum");
    CHECK_NEAR(s.v->attitudeTorqueAbout(a, -upY), 50.0, 1e-9, "and the -Y drum is worth its rating about -Y");
    CHECK_NEAR(s.v->attitudeTorqueAbout(a, upX), 20.0, 1e-9, "and the +X drum is the only thing covering the pitch axis");
    CHECK_NEAR(s.v->attitudeTorqueAbout(a, -upX), 20.0, 1e-9, "a line is two-sided, so -pitch gets the same drum");
    /* The number a hull-wide demand is measured against: along the body diagonal each
       drum brings its cos, so 170/sqrt(3). This is what the old sum got WRONG by
       claiming 170 about every axis. */
    CHECK_NEAR(s.v->attitudeTorqueAbout(a, glm::dvec3(1.0, 1.0, 1.0)), 170.0 / std::sqrt(3.0), 1e-9,
               "about a diagonal, every drum brings its cos");
    /* And the delivery side, so a law's plan and the actuators' answer agree: a demand
       along the pitch axis can only get the 20 N m drum, NOT the 170 the hull owns. */
    std::vector<double> sp;
    const glm::dvec3 got = actuatorSolve(a.wheels, 1000.0 * upX, sp);
    CHECK_NEAR(glm::length(got), 20.0, 1e-9, "a demand the drum cannot follow is answered with 20 N m");
    CHECK_NEAR(glm::dot(got, upZ), 0.0, 1e-9, "and it cannot leak into the roll axis it was not asked for");
    destroyShip(s.v);
}

/* A TRIAD HOUSING IS THREE DRUMS ON THREE LINES (#593). #576 made the drum count a
   content decision rather than a fudge: one stacked drum levers the stack axis only,
   so a hull of them cannot point itself, while a triad housing -- three orthogonal
   drums at the same rating, which is what every shipped wheel part declares -- is
   three rows. This is the pin that says the shipped per-axis number is UNCHANGED
   from the old isotropic sum, and that the triad's coverage belongs to the housing
   rather than to a mount: roll the box and its three lines roll with it, where a
   single drum would drop out of the pitch and yaw columns. */
static void test_a_triad_housing_covers_three_lines() {
    printf("== a triad housing is three rows on three lines ==\n");
    Stack st = mkStack(200.0, 0.0, 0.0);      // a wheel on the root, no TVC anywhere
    Vehicle *v = st.s.v;
    /* Part::def is const (the catalog owns the defs in the game); the hand-built
       defs here live in a deque so their addresses hold. */
    for(PartDef &d : st.s.defs) { if(d.name == std::string("root")) { d.wheel_axes = 3; } }
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_TRUE(a.wheels.size() == 3, "three drums, three rows");
    CHECK_NEAR(a.wheelRatingSum(), 600.0, 1e-12, "three drums at the rating each");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 200.0, 1e-9, "one drum on the stack line");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), 200.0, 1e-9, "one on the pitch line");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.y), 200.0, 1e-9, "one on the yaw line");
    /* Now the housing is TILTED 45 deg about a transverse axis -- not rolled about
       the stack. A roll permutes a triad onto the same three hull lines, so every
       check above would also pass for an implementation that used the HULL basis
       instead of the part's own axes; a tilt separates them, and the numbers below
       are what only part-mounted drums produce (a hull-basis model reads 200 about
       every axis here). A def cannot do this to a stacked part -- only an authored
       point/normal contact can -- which is why the fixture hand-sets localRot, the
       same way the cluster fixture above does. */
    const double k = std::sqrt(0.5);
    st.root->localRot = glm::dmat3(1.0, 0.0, 0.0,  0.0, k, k,  0.0, -k, k);
    const Vehicle::AttitudeAuthority r = v->attitudeAuthority();
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.x), 200.0, 1e-9,
               "the drum that stays on the tilt axis still buys its rating");
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.y), 200.0 * std::sqrt(2.0), 1e-9,
               "a tilted housing leans on the axes it swung toward (#593)");
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.z), 200.0 * std::sqrt(2.0), 1e-9,
               "and the same about the third");
    /* And delivery agrees with the budget: a demand far past one rating along one
       line is answered with that line's drum -- not with the 600 N m the housing
       owns, and not with two drums leaning on an axis they do not point at. */
    std::vector<double> sp;
    const glm::dvec3 got = actuatorSolve(r.wheels, 5000.0 * st.x, sp);
    CHECK_NEAR(glm::length(got), 200.0, 1e-9, "one line buys its own rating");
    destroyShip(v);
}

/* Command arms ONE key and leaves the others standing, exactly as the keyboard does
   (tick.cpp clears the stick at the tick top), so a case that wants exactly one key
   held has to release the other two first. */
static void holdStick(Vehicle *v, ShipCmdType key, float amount) {
    v->Command(ShipCmd(Roll, 0.0f), true);
    v->Command(ShipCmd(Pitch, 0.0f), true);
    v->Command(ShipCmd(Yaw, 0.0f), true);
    if(amount != 0.0f) { v->Command(ShipCmd(key, amount), true); }
}

/* THE STICK IS INTENT, NOT WIRING (#425 step D, and the structural close of #420).
   The pitch and yaw keys used to drive the drums directly in applyRotationForce AND
   be mixed by hand into the gimbals in applyThrustForce (`gimbal_max * latShip +
   the autopilot's aim`), so each actuator took the stick at full rate and only a
   per-engine clamp sorted out the excess -- which is exactly the double-bill #420
   complained about. Now the keys become ONE demanded torque and actuateAttitude
   splits it. This test pins the TVC half of that answer; the next pins the wheel
   half, which needs a fixture whose drums cover the demanded axis. The stick is
   armed through Command(), the player's own door (#424). */
static void test_the_stick_demands_one_torque() {
    printf("== the stick is one demand through one allocator (#425 D) ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);   // a stacked drum AND a gimballed engine
    Vehicle *v = st.s.v;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    const double latFull = std::sin(0.1);    // the stop, as the split converts it to rad
    CHECK_NEAR(a.tvcTorque, 1000.0 * latFull, 1e-9, "the engine's authority, for the pins below");

    /* Pitch held: the mixer used to aim at exactly gimbal_max along -Y. The split
       reaches the stop from the demanded torque instead, so the deflection is
       sin(gimbal_max) -- 0.17% off for this fixture's 0.1 rad stop, 0.127% off the
       shipped part's 0.0873 -- and the direction is the same. */
    holdStick(v, Pitch, +1.0f);
    v->applyRotationForce(0.02);
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9,
               "a held key drives the gimbals to their stop, by way of the split");
    CHECK_NEAR(glm::dot(v->tvcLat_, st.y), -latFull, 1e-9, "and the same way as before (#571)");
    CHECK_NEAR(glm::dot(v->tvcLat_, st.x), 0.0, 1e-9, "no cross-axis aim");
    CHECK_NEAR(glm::dot(v->tvcLat_, st.z), 0.0, 1e-9, "and no aim along the stack");

    /* Roll is wheels-only: a lateral deflection can never lever the stack axis, so
       the allocator aims the gimbals nowhere however hard the key is held. */
    holdStick(v, Roll, +1.0f);
    v->applyRotationForce(0.02);
    CHECK_NEAR(glm::length(v->tvcLat_), 0.0, 1e-12, "a roll demand buys no gimbal deflection");

    /* Two keys, one bound: the demand is the sum of the two axes' capabilities, so
       the transverse part is past the TVC stop and the aim SATURATES at it. (The old
       mixer normalized latShip too, so this is not a change in what a two-key hold
       buys; it is the new path being bound where the old one was bound.) */
    v->Command(ShipCmd(Roll, 0.0f), true);
    v->Command(ShipCmd(Pitch, +1.0f), true);
    v->Command(ShipCmd(Yaw, +1.0f), true);
    v->applyRotationForce(0.02);
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9,
               "two keys still buy one stop, not two");
    /* Pitch alone aims -Y and yaw alone aims +X (a deflection torques about the axis
       90 deg from it), so a both-keys demand aims the gimbals between them. */
    CHECK_NEAR(glm::dot(v->tvcLat_, st.x - st.y) / std::sqrt(2.0), latFull, 1e-9,
               "and the aim points where the demand pointed");

    /* Power gates the drums, never the gimbals: a dead ship still steers while it
       burns (#418), which it could only do before because the mixer read the stick
       itself -- the gate now lives in the demand's wheel term. */
    v->powered_ = false;
    v->applyRotationForce(0.02);
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9,
               "an unpowered hull still aims its gimbals from the stick");

    /* Nothing held, nothing aimed: applyRotationForce clears the aim every substep,
       so a released stick recentres the gimbals. */
    v->powered_ = true;
    v->Command(ShipCmd(Pitch, 0.0f), true);
    v->Command(ShipCmd(Yaw, 0.0f), true);
    v->applyRotationForce(0.02);
    CHECK_NEAR(glm::length(v->tvcLat_), 0.0, 1e-12, "a released stick leaves no aim");
    destroyShip(v);
}

/* THE WHEELS ARE BILLED ONCE A SUBSTEP, WHOEVER ASKS (#420's structural claim).
   The check above can only see the gimbals: with a single drum on the stack axis, a
   pitch demand buys no wheel torque at all, so the wheel half of the demand was
   invisible -- which is how the double-bill survived the first pass at this. The
   wheel half IS visible here: no world steps, but Bullet accumulates the torques
   applied to the hull body, so getTotalTorque() reads the allocator's answer.
   The fixture is therefore a TRIAD housing (three orthogonal 200 N m drums), where a
   pitch demand costs 200 N m of drum and 99.8 N m of nozzle. Two consumers in one
   substep used to mean two trips through actuateAttitude, each solving its own wheel
   share out of a FULL command: latched kill-rot plus a held key asked a 200 N m drum
   for -400 N m. Now the demands sum and the drums are asked once. */
static void test_the_wheels_are_billed_once() {
    printf("== the wheels are billed once a substep, whoever asks (#420) ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    for(PartDef &d : st.s.defs) { if(d.name == std::string("root")) d.wheel_axes = 3; }
    Vehicle *v = st.s.v;
    const double latFull = std::sin(0.1);
    const double tvcMax = 1000.0 * latFull;      // the fixture's TVC ceiling
    CHECK_NEAR(v->attitudeAuthority().tvcTorque, tvcMax, 1e-9,
               "the fixture's TVC ceiling, for the pins below");
    /* What the hull was actually torqued by, along the axis the pitch key turns it
       about -- the ship's own X, the "right" axis; the -Y in the test above is where
       the nozzles point, not where the torque acts. Reading the accumulator CLEARS it,
       so a substep's answer is taken once and projected. */
    auto billed = [&]() {
        const btVector3 t = v->hull->btBody->getTotalTorque();
        v->hull->btBody->clearForces();
        return glm::dvec3(t.x(), t.y(), t.z());
    };
    auto billedPitch = [&]() { return glm::dot(billed(), st.x); };
    auto billedRoll = [&]() { return glm::dot(billed(), st.z); };
    v->hull->btBody->clearForces();

    /* One consumer: the stick over-asks (200 N m of drum + 99.8 N m of nozzle), so
       the drum pays its rating and the gimbals go to the stop. */
    holdStick(v, Pitch, +1.0f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), -200.0, 1e-9, "the drum pays its rating and no more");
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9, "the gimbals take the rest, one stop");

    /* Two consumers, one substep: kill-rot on a spinning hull demands I*w/h, which is
       thousands of N m, and the stick asks on top of that. The demands sum BEFORE the
       split, so the bill is unchanged from the stick asking alone. Red if a law gets
       its own trip through the allocator. */
    v->hull->btBody->setAngularVelocity(btVector3(1.0f, 0.0f, 0.0f));
    v->Command(ShipCmd(KillRot), true);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), -200.0, 1e-9, "law + key together still bill the drum once");
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9, "and the gimbals once");

    /* Neither consumer is dropped on the way to that bound. A latched law does not
       get to own the substep: hold ROLL (an axis kill-rot has no interest in here)
       while it runs and the drum must still answer the key. Red if the demand the
       pilot contributed is thrown away. */
    holdStick(v, Pitch, 0.0f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), -200.0, 1e-9, "the law's demand still reaches the drum");
    v->hull->btBody->setAngularVelocity(btVector3(0.001f, 0.0f, 0.0f));  /* barely turning */
    holdStick(v, Roll, +0.5f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedRoll(), +100.0, 1e-9,
               "half a roll key still buys half a drum under a latched law");

    v->clearRotCmd();                            /* the tick's release: slew and stick */
    v->hull->btBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), 0.0, 1e-12, "nothing asked, nothing billed");

    /* One demand per substep must not let a saturated LINE spend an idle one. The
       first version of #420 summed the demands and left tvcSplit's SCALAR wheel
       clamp in place: it preserves the remainder's direction, so a kill-rot ask
       thousands of N m long scaled the pilot's roll component down with it -- 2.29 N m
       of a 200 N m drum. The bound belongs to actuatorSolve, per line, which is what
       lets the pitch drum saturate while the roll drum still answers in full. */
    v->hull->btBody->setAngularVelocity(btVector3(1.0f, 0.0f, 0.0f));   /* a fast tumble */
    v->Command(ShipCmd(KillRot), true);
    holdStick(v, Roll, +1.0f);
    v->applyRotationForce(1.0 / 120.0);
    const glm::dvec3 tumble = billed();
    CHECK_NEAR(glm::dot(tumble, st.x), -200.0, 1e-9, "the tumble still bills the pitch drum");
    CHECK_NEAR(glm::dot(tumble, st.z), +200.0, 1e-9,
               "and the roll drum the pilot asked for, undiluted (#420)");
    v->clearRotCmd();

    /* And what a PARTIAL stick buys, pinned as the split's current POLICY for the
       pilot rather than as physics. One key at fraction `a` asks for a x the hull's
       whole capability (200 N m of drum + 99.8 N m of nozzle), and tvcSplit spends
       TVC first: below a_crit = tvcMax/cap = 0.333 the drum contributes nothing,
       above it the nozzles are already at their stop. The total is right either way -- who
       delivers it is the question #421 decides, and this is the pin that notices. */
    const double cap = 200.0 + tvcMax;
    holdStick(v, Pitch, 0.25f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), 0.0, 1e-9, "a quarter stick leaves the drum idle");
    CHECK_NEAR(glm::length(v->tvcLat_), 0.25 * cap / 1000.0, 1e-9,
               "and the nozzle carries all of it, part-way to the stop");
    holdStick(v, Pitch, 0.5f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(billedPitch(), -(0.5 * cap - tvcMax), 1e-9,
               "half stick: the drum only tops up what the nozzle cannot reach");
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9, "with the nozzle already at its stop");
    destroyShip(v);
}

/* PILOT vs SAS ON ONE AXIS (#421, #425 step D-2). Since #420 the pilot's demand and a
   latched law's ride on ONE vector, so the only thing left to decide is who gets an
   axis -- and that is a control preference, not physics, so BOTH ends get pinned.
   The fixture tumbles about its own X (the pitch key's axis) with kill-rot latched:
   the law asks I*w/h, which is thousands of N m. Pitch +1 also asks -X, so a +1 key
   and the law agree and the reading cannot tell them apart -- hold the key the OTHER
   way (-1 -> +X) and they fight for the line. */
static void test_the_pilot_and_the_law_split_by_axis() {
    printf("== pilot vs SAS: a held key owns its axis (#421) ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    for(PartDef &d : st.s.defs) { if(d.name == std::string("root")) d.wheel_axes = 3; }
    Vehicle *v = st.s.v;
    auto billed = [&]() {
        const btVector3 t = v->hull->btBody->getTotalTorque();
        v->hull->btBody->clearForces();
        return glm::dvec3(t.x(), t.y(), t.z());
    };
    auto pitchFight = [&](SasPolicy policy, float stick = -1.0f, double wmag = 1.0) {
        v->clearRotCmd();
        v->hull->btBody->setAngularVelocity(btVector3((float)wmag, 0.0f, 0.0f));
        v->Command(ShipCmd(KillRot), true);        /* the law: thousands of N m, -X */
        holdStick(v, Pitch, stick);                /* the pilot: +X, against it */
        v->sas_policy = policy;
        v->applyRotationForce(1.0 / 120.0);
        return glm::dot(billed(), st.x);
    };
    CHECK_NEAR(pitchFight(SasStickOwnsAxis), +200.0, 1e-9,
               "default: the held key owns its axis, so the drum turns the pilot's way");
    CHECK_NEAR(pitchFight(SasKeepsAsk), -200.0, 1e-9,
               "alternate: the law keeps its ask, and the key is inside a demand the drum cannot follow");

    /* How MUCH of the law's ask a key takes away is |stick|, not all-or-nothing.
       Keys only ever send +-1 through tick.cpp today, so this shape is unpinned by
       anything else -- and either spelling passes every other check here. Two
       consequences, both pinned because they are the ones a partial key would
       silently change:
       - against a law that is still pushing hard, HALF a key does not win the axis.
         The mask leaves half of an ask that is already tens of times the drum, so
         the allocator saturates the other way. A binary mask reads +200 here.
       - with the law nearly quiescent, the law still speaks THROUGH the mask: double
         the spin and the part of its ask the key left standing doubles too, so the
         answer moves. A binary mask makes these two readings identical. */
    CHECK_NEAR(pitchFight(SasStickOwnsAxis, -0.5f), -200.0, 1e-9,
               "half a key loses an axis the law is still pushing hard on");
    const double quiet1 = pitchFight(SasStickOwnsAxis, -0.5f, 1e-3);
    const double quiet2 = pitchFight(SasStickOwnsAxis, -0.5f, 2e-3);
    CHECK_TRUE(quiet2 < quiet1 - 1e-6,
               "a partial key leaves the law a partial ask, and that ask scales with the spin");

    /* The rule is PER AXIS, not a switch that suspends the autopilot: an axis nobody
       is steering stays the law's under both policies, so a pilot rolling while SAS
       holds attitude still gets the roll AND still gets the law's correction. */
    auto rollUnderLaw = [&](SasPolicy policy) {
        v->clearRotCmd();
        v->hull->btBody->setAngularVelocity(btVector3(1.0f, 0.0f, 0.0f));
        v->Command(ShipCmd(KillRot), true);
        holdStick(v, Roll, +1.0f);                 /* an axis the law has no use for */
        v->sas_policy = policy;
        v->applyRotationForce(1.0 / 120.0);
        return billed();
    };
    const glm::dvec3 axis = rollUnderLaw(SasStickOwnsAxis);
    CHECK_NEAR(glm::dot(axis, st.z), +200.0, 1e-9, "the pilot keeps the steered axis");
    CHECK_NEAR(glm::dot(axis, st.x), -200.0, 1e-9, "and the law keeps the one nobody holds");
    const glm::dvec3 summed = rollUnderLaw(SasKeepsAsk);
    CHECK_NEAR(glm::dot(summed, st.z), +200.0, 1e-9, "the alternate gives the same roll");
    CHECK_NEAR(glm::dot(summed, st.x), -200.0, 1e-9, "and the same law correction");
    v->clearRotCmd();
    destroyShip(v);
}

/* A SWITCHED-OFF DRUM IS NOT A ROW (#425 step D-2). The wheels' ENABLE mask goes
   through the same door as the power gate (#418), so the capability model, the laws,
   the stick's own scaling and the Vessel window all lose the drums at once -- there is
   no second bookkeeping place where a disabled drum is still creditable. What a
   wheels-off hull keeps is its gimbals; what it loses is roll, which no lateral
   deflection can lever (#419). */
static void test_a_switched_off_drum_is_not_a_row() {
    printf("== the wheels' enable mask: no rows, no promise (#425 D-2) ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    for(PartDef &d : st.s.defs) { if(d.name == std::string("root")) d.wheel_axes = 3; }
    Vehicle *v = st.s.v;
    const double latFull = std::sin(0.1);
    const double tvcMax = 1000.0 * latFull;
    auto billed = [&]() {
        const btVector3 t = v->hull->btBody->getTotalTorque();
        v->hull->btBody->clearForces();
        return glm::dvec3(t.x(), t.y(), t.z());
    };

    CHECK_TRUE(v->wheels_enabled, "a ship starts with its drums on");
    CHECK_NEAR(v->attitudeAuthority().wheelRatingSum(), 600.0, 1e-9,
               "on, the fixture is three 200 N m rows");

    v->wheels_enabled = false;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_TRUE(a.wheels.empty(), "switched off: not a row left to spend");
    CHECK_NEAR(actuatorTorqueAbout(a.wheels, st.z), 0.0, 0.0, "no wheel authority about any axis");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), tvcMax, 1e-9,
               "what the hull can bring to pitch is the nozzles only");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 0.0, 0.0,
               "and about roll, nothing at all: exact, not rounded (#419)");

    holdStick(v, Pitch, +1.0f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(glm::dot(billed(), st.x), 0.0, 1e-12, "the switched-off drums bill nothing");
    CHECK_NEAR(glm::length(v->tvcLat_), latFull, 1e-9,
               "and the whole ask routes to the gimbals, at their stop");

    holdStick(v, Roll, +1.0f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(glm::length(v->tvcLat_), 0.0, 1e-12, "a roll key buys no aim either");
    CHECK_NEAR(glm::length(billed()), 0.0, 1e-12, "nor any torque: this hull cannot roll");

    v->wheels_enabled = true;
    holdStick(v, Roll, +1.0f);
    v->applyRotationForce(1.0 / 120.0);
    CHECK_NEAR(glm::dot(billed(), st.z), 200.0, 1e-9, "switched back on, roll answers again");
    destroyShip(v);
}

/* ONE FRAME FOR THE WHOLE ATTITUDE PATH (#571). The stack axis TVC levers belongs
   to the ROOT part, and since #571 every consumer asks attitudeRef() for it: the
   mixer, attitudeAuthority, the manual stick, rcsWorldDir, the two traces, and
   slewDemand's facing. Before it, the laws and the stick took firstWheel() -- a
   frame picked by parts-list order. Which half of that was observable:
     - the stick, YES: a surface mount keeps a part's +Z parallel to its parent's
       and ROLLS it about the stack, so a wheel on a tank's side has the ship's nose
       and the ship's UP axis as its "right" one -- W yaws, A pitches. Measured on
       res/ships/sideways_wheel.json (e2e flight/571), not here: this binary has no
       physics world, so ApplyTorque is not observable.
     - the fixture below, in a shape a def CANNOT produce: localRot is set by hand so
       the wheel's +Z is 90 deg off the stack. That is what makes the disk normal and
       the facing decidable here; a shipped mount can only roll a wheel, not tilt it.
     - killRotDemand and the killrot trace row: NO -- they used the reference part only
       to read the hull's rate, which is the same for every part (hullAngVel(), #574).
   What a model that took the wheel's +Z as the TVC disk's normal WOULD do is here and
   now pinned: it re-opens the hole step B closed, crediting TVC with roll. */
static void test_one_frame_for_the_attitude_path() {
    printf("== one frame: the ROOT's axes, for the disk normal and the facing ==\n");
    Ship s;
    s.v = new Vehicle;
    s.v->name = "gear";
    Part *root = mkPart(s.defs, "root", 100.0, 1.5, 0.0, 0.0);     // no wheel here
    s.v->setRoot(root);
    Part *eng = mkPart(s.defs, "engine", 100.0, 1.0, 0.0, 0.1);    // gimballed, armed below
    s.v->attachDown(eng);
    Part *wheel = mkPart(s.defs, "wheel", 100.0, 0.5, 300.0, 0.0); // a wheel, on its side
    s.v->attachDown(wheel);
    /* +90 deg about X: the wheel's own +Z becomes the ship's -Y, so the wheel's
       axis and the stack axis are 90 deg apart and the two choices cannot be confused.
       Set before init() so the hull compound is built with it. */
    wheel->localRot = glm::dmat3(1.0, 0.0, 0.0,  0.0, 0.0, 1.0,  0.0, -1.0, 0.0);
    s.v->init();
    s.v->placeShip(glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    eng->armedThrust = 800.0f;

    Vehicle *v = s.v;
    const glm::dvec3 rootZ = v->partAxis(root, 2);
    const glm::dvec3 wheelZ = v->partAxis(wheel, 2);
    CHECK_NEAR(glm::dot(rootZ, wheelZ), 0.0, 1e-12,
               "the fixture: wheel axis at 90 deg to the stack axis");
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_NEAR(glm::dot(a.stackAxis, rootZ), 1.0, 1e-12,
               "the authority is stated in the ROOT's frame");
    /* #571 proper: the fixture has a wheel AND a root whose axes differ by 90 deg,
       so "whose axes are the ship's" is decidable here, and the answer is the root's.
       Which checks bite against which break (tmp/aero/n571_mutate2.py): pointing the
       accessor at the wheel reddens the stack-axis pin, the identity pin, the facing
       pin and the actuator pin below -- the last because the TVC lever is
       dot(partPos - com, stackAxis), so a rolled frame zeroes it -- while the two
       attitudeTorqueAbout checks stay green, since with tvcTorque down to 0 there is
       nothing left for the split to disagree about. What those two DO catch is a
       stackAxis that disagrees with the lever while the lever still reads, which is
       the subtler break and the one step B's mutation set exercised. */
    CHECK_TRUE(v->attitudeRef() == root, "the ship's frame is the ROOT part");
    CHECK_NEAR(glm::dot(v->partAxis(v->attitudeRef(), 2), rootZ), 1.0, 1e-12,
               "and the facing it hands the laws is the stack axis, not the wheel's");
    CHECK_NEAR(a.wheelRatingSum(), 300.0, 1e-12, "the drum is a row on this hull");
    CHECK_TRUE(a.tvcTorque > 0.0, "and the gimballed engine is live");
    /* About the stack axis: NOTHING. Two reasons, and this fixture separates them:
       TVC cannot lever the stack axis at all (#425), and the drum on this hull is
       rolled 90 deg off it, so it cannot lever it either (#576). Before #576 the
       number here was the wheel's full 300, credited to a drum that physically
       cannot push that way. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, rootZ), 0.0, 0.0,
               "no authority about the stack axis: not TVC, and not a rolled drum");
    /* #576's other half, on the same fixture: the drum's OWN axis is where its
       rating lives -- and that axis is transverse to the stack, so TVC counts there
       in full too. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, wheelZ), 300.0 + a.tvcTorque, 1e-9,
               "the rolled drum is worth its rating about its own axis");
    /* Half-way between the drum's line and the stack: the drum is creditable to that
       cosine and the disk to the sine of the same angle, which is the arithmetic
       actuators.h shares with the actuator side. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, glm::normalize(wheelZ + v->partAxis(root, 2))),
               (300.0 + a.tvcTorque) * std::sqrt(0.5), 1e-9,
               "each row is creditable by its own cosine about a mixed axis");
    /* #573: the DISPLAYED attitude asks the same question as the laws. Seat the crew in
       the rolled wheel -- which is exactly where the loader's fallback used to put them,
       since controllerIndex() took the first part with torque > 0 -- and the axes a
       heading/pitch/roll readout is computed from must still describe the hull the keys
       turned. This reddens if attitudeBasis() itself goes back to asking `controller`;
       render.cpp's HUD, navball and chase basis follow that one accessor, so they move
       with it -- but no check HERE reaches render.cpp, so a HUD-side break shows up in
       e2e ([surfinfo]'s pitch/roll/hdg), not in this binary. */
    const glm::dvec3 rootX = v->partAxis(root, 0);
    const glm::dvec3 rootUp = v->partAxis(root, 1);
    v->controller = wheel;
    const glm::dmat3 disp = v->attitudeBasis();      // [right, up, nose]
    CHECK_NEAR(glm::dot(disp[2], rootZ), 1.0, 1e-12, "the displayed nose is the stack axis");
    CHECK_NEAR(glm::dot(disp[1], rootUp), 1.0, 1e-12, "and the displayed up is the ship's");
    CHECK_NEAR(glm::dot(disp[0], rootX), 1.0, 1e-12, "and its right");
    CHECK_NEAR(glm::dot(disp[2], v->partAxis(wheel, 2)), 0.0, 1e-12,
               "none of them is the seat's: that part is rolled 90 deg about the stack");
    CHECK_NEAR(glm::dot(disp[1], v->partAxis(wheel, 1)), 0.0, 1e-12,
               "read in the seat's frame, the HUD would call a pitch a yaw");
    destroyShip(s.v);
}

/* THE COMMAND FUNNEL (#424, #425 design point 5): Command() is the only door from
   input to the ship's control state, so the RULES of the channels live in it rather
   than being re-derived at each call site. Three things were elsewhere before:
     - the SAS engage/disengage rule (mutually exclusive, held until toggled) was
       spelled out in the Autopilot window, in the Z/B key handler and in --autopilot,
       while Command's Prograde/Retrograde cases latched `slew` directly and had no
       callers at all -- dead code that read as the funnel.
     - `amount` was thrown away: Pitch/Yaw/Roll took only its SIGN, so an amount of 0
       meant FULL tilt and a half amount did not exist.
     - the ship's frame (attitudeRef) -- pinned above, and the reason this funnel could
       not be built before #571.
   What this pins is the STATE the funnel leaves behind. That the wheel multiplies
   `stick` by its rating is observable only with a physics world (this binary has
   none); e2e flight/571 pins the full-travel case, and 571's fixture pin in
   test_shipload keeps that hull honest. */
static void test_the_command_funnel() {
    printf("== the funnel: one door, and what latches behind it ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    Vehicle *v = st.s.v;

    /* An attitude amount is a NORMALIZED demand, kept as sent (see ShipCmd). */
    v->Command(ShipCmd(Pitch, +0.5f), true);
    CHECK_NEAR(v->stickAxis(1), 0.5f, 1e-12, "half a demand arms half a stick");
    v->Command(ShipCmd(Yaw, -0.75f), true);
    CHECK_NEAR(v->stickAxis(2), -0.75f, 1e-12, "and keeps the sign of it");
    v->Command(ShipCmd(Roll, +1.0f), true);
    CHECK_NEAR(v->stickAxis(0), 1.0f, 1e-12, "full stick is still full");
    /* The trap the old spelling left: sign-only meant `amount 0` == full tilt. A
       zero demand arms nothing, which is what an autopilot at equilibrium sends. */
    v->Command(ShipCmd(Pitch, 0.0f), true);
    CHECK_NEAR(v->stickAxis(1), 0.0f, 1e-12, "zero demand arms nothing");
    /* No out-of-range pin on purpose: normCmd() asserts, so a debug build aborts
       before it can check the clamp. Out of range is a sender bug, not an input. */

    /* The funnel itself refuses when the sim is not running. Note this is the
       STICK/throttle path only: the three SAS callers (window, Z/B, --autopilot)
       pass simActive = true on purpose, because a latched mode is state the player
       left set -- tick.cpp:158 is what gates its EFFECT on game_running, so moving
       that gate here would make a paused click do nothing at all. */
    v->clearRotCmd();
    v->Command(ShipCmd(Pitch, +1.0f), false);
    CHECK_NEAR(v->stickAxis(1), 0.0f, 1e-12, "input while paused arms nothing");

    /* Commands latch until the tick clears them -- the per-tick/per-substep split
       the autopilot has to share (see the #425 latch note). */
    v->Command(ShipCmd(Pitch, -0.25f), true);
    CHECK_NEAR(v->stickAxis(1), -0.25f, 1e-12, "armed for the whole tick");
    v->clearRotCmd();
    CHECK_TRUE(v->stickAxis(0) == 0.0f && v->stickAxis(1) == 0.0f && v->stickAxis(2) == 0.0f,
               "clearRotCmd releases the stick");

    /* SAS: engage, and the same command again disengages. One rule, one place. */
    v->Command(ShipCmd(Sas, SlewPrograde), true);
    CHECK_TRUE(v->slewRequestMode() == SlewPrograde, "Sas engages the mode");
    CHECK_TRUE(v->slewMode() == SlewNone,
               "engaging is a REQUEST; tick.cpp copies it into the engaged copy");
    v->Command(ShipCmd(Sas, SlewPrograde), true);
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "the same command releases it");

    /* Mutually exclusive: a second mode replaces the first rather than stacking. */
    v->Command(ShipCmd(Sas, SlewPrograde), true);
    v->Command(ShipCmd(Sas, SlewRetrograde), true);
    CHECK_TRUE(v->slewRequestMode() == SlewRetrograde, "modes are mutually exclusive");

    /* The latch survives the per-tick clear; the HELD override does not. Both
       spellings of kill-rot exist on purpose: X holds, the window latches. */
    v->clearRotCmd();
    CHECK_TRUE(v->slewRequestMode() == SlewRetrograde, "the latch survives clearRotCmd");
    v->Command(ShipCmd(KillRot), true);
    CHECK_TRUE(v->slewMode() == SlewKillRot && v->slewRequestMode() == SlewRetrograde,
               "held kill-rot overrides the latch without overwriting it");
    v->clearRotCmd();
    CHECK_TRUE(v->slewMode() == SlewNone && v->slewRequestMode() == SlewRetrograde,
               "releasing X releases the override, not the latch");
    v->Command(ShipCmd(Sas, SlewKillRot), true);
    CHECK_TRUE(v->slewRequestMode() == SlewKillRot, "the window's Kill rotation latches");

    /* Channels stay separate: an attitude demand never touches the latch, and a
       mode never touches the stick (#425's scope: throttle, rcsDir and staging stay
       their own channels too). Released through Command rather than the restore door:
       the latch is input state, and only a save load has any business restoring it. */
    v->Command(ShipCmd(Sas, SlewKillRot), true);
    v->clearRotCmd();
    v->Command(ShipCmd(Pitch, +1.0f), true);
    CHECK_TRUE(v->slewRequestMode() == SlewNone && v->slewMode() == SlewNone,
               "the stick does not engage SAS");
    v->Command(ShipCmd(Sas, SlewNormal), true);
    CHECK_TRUE(v->stickAxis(0) == 0.0f && v->stickAxis(1) == 1.0f && v->stickAxis(2) == 0.0f,
               "SAS does not move the stick");
    v->Command(ShipCmd(Sas, SlewNormal), true);
    v->clearRotCmd();

    /* applySlewRequest() owns the gate that used to sit next to it in tick.cpp: a
       paused sim engages nothing however the window is set, and the copy says what
       drives THIS tick rather than what was latched last tick. */
    v->Command(ShipCmd(Sas, SlewPrograde), true);
    CHECK_TRUE(!v->applySlewRequest(false), "paused: no mode drives the tick");
    CHECK_TRUE(v->slewMode() == SlewNone, "and the engaged copy says so");
    CHECK_TRUE(v->applySlewRequest(true), "running: the latch drives the tick");
    CHECK_TRUE(v->slewMode() == SlewPrograde, "so the copy is the latched mode");
    v->Command(ShipCmd(Sas, SlewPrograde), true);
    CHECK_TRUE(!v->applySlewRequest(true) && v->slewMode() == SlewNone,
               "released: the copy drops in the same step");

    /* What the wheels bill (actuatorsEngaged) -- both halves, because the Vessel
       window's Power row and powerTick() share this one rule and used to spell it
       separately. */
    v->Command(ShipCmd(Pitch, +1.0f), true);
    CHECK_TRUE(v->actuatorsEngaged(), "a commanded stick works the wheels");
    v->clearRotCmd();
    v->powered_ = false;
    CHECK_TRUE(!v->actuatorsEngaged(),
               "no power: the wheels are not working, stick or no stick");
    v->powered_ = true;
    v->Command(ShipCmd(Sas, SlewRetrograde), true);
    CHECK_TRUE(!v->actuatorsEngaged(), "a latch that the tick has not applied yet");
    v->applySlewRequest(true);
    CHECK_TRUE(v->actuatorsEngaged(),
               "under SAS the wheels work with a released stick");
    v->Command(ShipCmd(Sas, SlewRetrograde), true);
    v->applySlewRequest(true);

    /* The save door's range check. A save is hand-editable data, and a non-mode latches
       on wake: the ship leaves the rails while slewTargetDir() has nothing to aim at, so
       it runs at 1x and never turns, forever. save.cpp's validator refuses such a file
       outright; this is the belt, and it is the only thing that stops a caller of
       restoreSlewRequest from handing the laws a value no mode owns. */
    v->restoreSlewRequest((SlewMode)99);
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "a non-mode latches nothing");
    v->restoreSlewRequest((SlewMode)-1);
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "nor does a negative one");
    v->applySlewRequest(true);
    CHECK_TRUE(v->slewMode() == SlewNone, "so nothing drives the tick");
    v->restoreSlewRequest(SlewAntiNormal);
    CHECK_TRUE(v->slewRequestMode() == SlewAntiNormal, "a real mode does latch");
    v->Command(ShipCmd(Sas, SlewAntiNormal), true);   // release through the input door
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "and Command still owns the release");
    destroyShip(v);
}

/* THE ARMED RCS DEMAND IS A DIRECTION (#575). Command's Rcs* cases each write one
   axis sign and hand the triple to armRcs(), so the stored demand is UNIT however many
   keys are held at once. Nothing on a ship can see that -- rcsWorldDir() normalizes
   again -- which is exactly why it needs a pin: the EVA suit's rcsWorldDir (eva.h)
   returns the field as it stands and multiplies its rated thrust by it, so a door that
   left the field at a cube corner would buy sqrt(3) x the thrust there. Step D is about
   to start spending this same field as a capability, so the invariant has to be one
   rule with a name before then. */
static void test_the_armed_rcs_demand_is_a_direction() {
    printf("== arming: keys held together still demand one direction ==\n");
    RcsProbe p;
    p.Command(ShipCmd(RcsNose, +1.0f), true);
    CHECK_NEAR(glm::length(p.rcsDir), 1.0, 1e-12, "one key: a unit demand");
    p.Command(ShipCmd(RcsUp, +1.0f), true);
    const double pair = 1.0 / std::sqrt(2.0);
    CHECK_NEAR(p.rcsDir.z, pair, 1e-12, "two keys: the first keeps its share");
    CHECK_NEAR(p.rcsDir.y, pair, 1e-12, "two keys: an even share");
    p.Command(ShipCmd(RcsRight, +1.0f), true);
    CHECK_NEAR(glm::length(p.rcsDir), 1.0, 1e-12,
               "three keys held together: unit, not a cube corner");
    /* The diagonal, not merely the octant (#580). Each key used to renormalize the
       ANSWER the previous key left rather than the whole request, so the last key
       armed kept the full share and the earlier ones shrank to half: N+I+L came out
       (.707,.5,.5) on rcs_box_pair, with the first key the worst treated. */
    const double even = 1.0 / std::sqrt(3.0);
    CHECK_NEAR(p.rcsDir.z, even, 1e-12, "three keys: the first armed keeps its share");
    CHECK_NEAR(p.rcsDir.y, even, 1e-12, "three keys: an even diagonal");
    CHECK_NEAR(p.rcsDir.x, even, 1e-12, "three keys: the last armed keeps its share");
    CHECK_TRUE(p.rcsDir.x > 0.0 && p.rcsDir.y > 0.0 && p.rcsDir.z > 0.0,
               "and it stays in the octant the three keys asked for");
    p.clearRcs();
    CHECK_NEAR(glm::length(p.rcsDir), 0.0, 1e-12, "the tick's release leaves no demand");
    p.Command(ShipCmd(RcsUp, -1.0f), true);
    CHECK_NEAR(p.rcsDir.y, -1.0, 1e-12, "a lone key arms its own axis, at full sign");
    CHECK_NEAR(glm::length(p.rcsDir), 1.0, 1e-12, "and the demand has no magnitude in it");
    p.armRcs(glm::dvec3(0.0));
    CHECK_NEAR(glm::length(p.rcsDir), 0.0, 1e-12, "arming zero is off, not a tiny burn");
    /* A key and its opposite for the same axis: whoever asked last is what the axis
       does, like a centred stick rather than a summed pedal pair. */
    p.clearRcs();
    p.Command(ShipCmd(RcsNose, +1.0f), true);
    p.Command(ShipCmd(RcsNose, -1.0f), true);
    CHECK_NEAR(p.rcsDir.z, -1.0, 1e-12, "a key and its opposite: the last one asked wins");
    /* Handing the ship over releases the demand as well as the stick and the
       throttle. tick.cpp clears RCS for the ACTIVE ship only, while applyRcsForce
       runs for every live ship off the rails, so a released ship that cannot rail
       -- an atmosphere keeps it off rails -- would otherwise fly on its last keys
       and empty its tanks doing it (#580's review). */
    p.clearRcs();
    p.Command(ShipCmd(RcsUp, +1.0f), true);
    CHECK_NEAR(glm::length(p.rcsDir), 1.0, 1e-12, "armed before the hand-over");
    p.releaseControl();
    CHECK_NEAR(glm::length(p.rcsDir), 0.0, 1e-12, "a released ship keeps no demand");
    p.Command(ShipCmd(RcsUp, +1.0f), true);
    CHECK_NEAR(glm::length(p.rcsDir), 1.0, 1e-12,
               "and the release did not disarm the keys for the next ship either");
}

/* #426 slice 2 at the VEHICLE level: surface rows that get SPENT, and a setpoint that
   has to reach the part that earned it. Everything else in this file asks a pure
   function, and no test anywhere gave a Vehicle an atmosphere to sit on, so the halves
   of the handover that only exist together were unpinned: the fin's share of a solve
   that must NOT also arrive as hull torque (a plate pushes on the air, so applying the
   solver's answer twice charges the hull twice for one fin), the ask that has to land
   in the same substep as the aero step that spends it, and the two early-outs -- no
   airspeed, aero switched off with --drag-cd 0 -- that have to agree with what the
   capability model promises. */
struct FinProbe : Vehicle {
    using Vehicle::stick;          // fake held keys
    using Vehicle::killRotDemand;  // the latched law's ask
    using Vehicle::slewDemand;     // and its braking-curve twin
    using Vehicle::getInertia;     // the tensor the laws multiply
    using Vehicle::slewTargetDir;  // what the engaged mode steers at (#606)
};

/* A body with nothing but the fields airDensityAtCom() reads: a 600 km radius, a
   sea-level atmosphere, and the gravity that atmosphere comes with, so a ship at
   601 km sits in 1000 m of air. The gravity is not decoration (#552 A2): the air law
   solves T/p/a from g x H, and the loader refuses to load a body that has air without
   g > 0 -- so an air fixture without it is a world the game cannot make, and
   applyAeroForce asserts as much. Nothing here quotes T, p or a, and rho/q/forces are
   g-free, so the value only has to be plausible: 9.81, the Kerbin the rig copies. */
static TerrainBody *mkAirBall() {
    TerrainBody *body = new TerrainBody;
    body->name = "airball";
    body->radius = 600000.0f;
    body->surface.sea_level = 0.0f;
    body->g = 9.81;
    body->surface.atmosphere.sea_level_density = 1.225;
    body->surface.atmosphere.scale_height = 8000.0;
    return body;
}

/* Pod + elevator, 1000 m up, flying +X at 100 m/s. The pod is a 100 N m TRIAD (a
   single drum would lever the stack axis only, and this asks about pitch); the 2 m^2
   elevator hangs off the stack -- 1.7 m below the pod's centre, so 1.55 m off the COM.
   Off the COM on purpose: a part AT the COM has a degenerate rig by construction
   (#495) and could only ever show zero. At 1000 m rho is 1.08 kg/m^3, so q is 5.4e3
   N/m^2 and the row is worth ~8e4 N m per radian against the pod's 100 N m. */
struct Rig {
    FinProbe *v;
    std::deque<PartDef> defs;
    Part *pod;
    Part *fin;
};

static Rig mkRig(double drumTorque, double finRollDeg) {
    Rig r;
    r.v = new FinProbe;
    r.v->name = "finhandover";
    r.pod = mkPart(r.defs, "pod", 100.0, 1.5, drumTorque, 0.0, 3);
    r.v->setRoot(r.pod);
    r.fin = mkPart(r.defs, "elevator", 10.0, 0.2, 0.0, 0.0, 1, 2.0);
    /* The stack edge's angle is a roll about the mating axis, so 180 deg bolts the
       same plate with the other face to the air: the mount column that
       drag.h::controlRigLever reads as its push direction flips. */
    r.v->attachMode(r.fin, 0, AttachMode::Down, finRollDeg);
    r.v->init();
    return r;
}

static void test_a_fin_row_is_spent_and_hands_its_ask_to_the_air() {
    printf("== a fin row: spent by the solve, handed to the air ==\n");
    TerrainBody *body = mkAirBall();
    Rig a = mkRig(100.0, 0.0);
    a.v->m_parent = body;
    a.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    SetVelocity(a.v->hull, glm::dvec3(100.0, 0.0, 0.0));

    /* No airspeed, no rows. Player-visible half of slice 2: on a held key at rest or in
       vacuum, plates used to throw themselves at nothing. */
    SetVelocity(a.v->hull, glm::dvec3(0.0, 0.0, 0.0));
    CHECK_TRUE(a.v->attitudeAuthority().surfaces.empty(),
               "no airspeed, no surface rows");
    SetVelocity(a.v->hull, glm::dvec3(100.0, 0.0, 0.0));

    const glm::dvec3 side = a.v->partAxis(a.pod, 0);   // the pitch line
    const glm::dvec3 up = a.v->partAxis(a.pod, 1);     // the yaw line
    const Vehicle::AttitudeAuthority au = a.v->attitudeAuthority();
    CHECK_TRUE(au.wheels.size() == 3, "the triad pod is three rows, one per ship line "
               "(#576's levers, #593's drum count)");
    CHECK_TRUE(au.surfaces.size() == 1, "and the plate is one row of its own");
    CHECK_TRUE(au.surfaceParts.size() == au.surfaces.size(),
               "one part per row -- that is how a setpoint finds its way back to the "
               "part that earned it");
    CHECK_NEAR(0.35, au.surfaces[0].limit, 1e-12,
               "a surface row's limit is travel in radians, not a torque");
    const glm::dvec3 row = au.surfaces[0].axis;
    printf("   row = (%.0f, %.0f, %.0f) N m per rad of command\n",
           row.x, row.y, row.z);
    CHECK_TRUE(glm::length(row) > 100.0 * 100.0,
               "in air the plate outsells the pod's drum by a hundredfold, so every "
               "number below says which of the two is doing the work");
    CHECK_NEAR(0.0, glm::dot(row, a.v->partAxis(a.pod, 2)), 1e-6 * glm::length(row),
               "and a plate on the stack levers nothing about the stack");
    const double pitchAsk = a.v->attitudeTorqueAbout(au, side);
    printf("   authority about pitch = %.0f N m; about yaw = %.0f N m\n",
           pitchAsk, a.v->attitudeTorqueAbout(au, up));
    CHECK_NEAR(0.35 * std::fabs(glm::dot(row, side)) + 100.0, pitchAsk,
               1e-9 * pitchAsk,
               "the promise is the plate at its stop plus the drum that shares its "
               "line -- nothing else, and a hundred times the drum is the reader's "
               "cue for which of the two is doing the work below");

    /* THE HULL IS TORQUED BY THE DRUM ONLY. One demand along pitch, drum and plate on
       the same line: the solve spends the line, then actuateAttitude subtracts the
       plate's share (setpoint * axis) and hands that setpoint to the air instead. Drop
       the subtraction and the hull is charged twice for one plate -- this number jumps
       from the drum's 100 N m to thousands. Pointing at the wrong setpoint index does
       the same, since the subtraction then misses. */
    a.v->stick[1] = 1.0f;   // the pitch key
    a.v->hull->btBody->clearForces();
    a.v->applyRotationForce(1.0 / 120.0);
    const btVector3 tau = a.v->hull->btBody->getTotalTorque();
    const double mech = std::fabs(glm::dot(glm::dvec3(tau.x(), tau.y(), tau.z()), side));
    printf("   hull torque about pitch = %.1f N m (demanded %.0f, drum 100)\n",
           mech, pitchAsk);
    CHECK_NEAR(100.0, mech, 1.0,
               "the drum's rating is all the hull gets from a solve that promised "
               "thousands: the rest is the air's bill (#426)");

    /* THE SAME SUBSTEP. applyRotationForce only stashes the ask; applyAeroForce spends
       it. That is what let the tick move control ahead of aero -- a latched law's
       demand and the plate that answers it have to share one substep or every fin has
       an actuator lag inside a deadbeat law. */
    CHECK_TRUE(a.fin->controlDefl == 0.0,
               "until the aero step runs, the part has not moved");
    a.v->applyAeroForce(1.0 / 120.0);
    const double deflA = a.fin->controlDefl;
    printf("   elevator after the aero step = %+.4f rad (stop 0.35)\n", deflA);
    CHECK_NEAR(0.35, std::fabs(deflA), 1e-6,
               "the solver's setpoint arrived as travel, and the demand was big enough "
               "to pin it at the stop");

    /* A released key recentres the plate: surfaceLat_ is cleared every substep rather
       than left over from the last one that asked. */
    a.v->stick[1] = 0.0f;
    a.v->hull->btBody->clearForces();
    a.v->applyRotationForce(1.0 / 120.0);
    a.v->applyAeroForce(1.0 / 120.0);
    CHECK_NEAR(0.0, a.fin->controlDefl, 1e-12, "no ask, no travel");

    /* --drag-cd 0 is the master aero switch: applyAeroForce returns before its surface
       block, while airDensityAtCom() answers the authored shell and never consults
       drag_cd. Budgeting the rows anyway spends a line on plates that can never move
       and takes the drum's share with them -- the hull gets less than it was promised,
       for a reason no readout says. Same lesson as #418's power gate. */
    a.v->drag_cd = 0.0;
    CHECK_TRUE(a.v->attitudeAuthority().surfaces.empty(),
               "with aero switched off, a control surface has no authority at all");
    a.v->stick[1] = 1.0f;
    a.v->hull->btBody->clearForces();
    a.v->applyRotationForce(1.0 / 120.0);
    {
        const btVector3 t2 = a.v->hull->btBody->getTotalTorque();
        const double mechOff = std::fabs(glm::dot(glm::dvec3(t2.x(), t2.y(), t2.z()), side));
        printf("   hull torque about pitch, --drag-cd 0 = %.1f N m\n", mechOff);
        CHECK_NEAR(100.0, mechOff, 1.0, "and the drum keeps its whole line");
    }
    destroyShip(a.v);

    /* A plate mounted the other way: the label is the same, the rig is not. The row must
       keep its direction (a positive COMMAND still pitches the same way) while the
       travel it takes to get there flips. This is #495's inversion guard, at the level
       where the row is built. */
    Rig b = mkRig(100.0, 180.0);
    b.v->m_parent = body;
    b.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    SetVelocity(b.v->hull, glm::dvec3(100.0, 0.0, 0.0));
    const Vehicle::AttitudeAuthority bu = b.v->attitudeAuthority();
    CHECK_TRUE(bu.surfaces.size() == 1, "the flipped plate is still a row");
    CHECK_TRUE(glm::dot(row, bu.surfaces[0].axis) > 0.0,
               "and its row points the same way: the rig sign, not the mount, decides "
               "what a positive command does");
    b.v->stick[1] = 1.0f;
    b.v->hull->btBody->clearForces();
    b.v->applyRotationForce(1.0 / 120.0);
    b.v->applyAeroForce(1.0 / 120.0);
    printf("   flipped plate: %+.4f rad against %+.4f rad\n",
           b.fin->controlDefl, deflA);
    CHECK_NEAR(-deflA, b.fin->controlDefl, 1e-6,
               "while the travel it needs to get there is the other way round");
    destroyShip(b.v);

    /* NOTHING MECHANICAL: a hull whose only actuator is a fin. The capability model
       says it can turn, so a latched law has to ask it for something. killRotDemand's
       early-out predates surface rows and tested wheels and gimbals only, so a fin-only
       hull got a silent zero -- the law off, while the readout quotes tens of kN m.
       slewDemand asks the question through attitudeTorqueAbout and never had the hole. */
    Rig g = mkRig(0.0, 0.0);
    g.v->m_parent = body;
    g.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    SetVelocity(g.v->hull, glm::dvec3(100.0, 0.0, 0.0));
    SetAngVelocity(g.v->hull, glm::dvec3(0.0, 0.0, 0.5));
    {
        const Vehicle::AttitudeAuthority gu = g.v->attitudeAuthority();
        CHECK_TRUE(gu.wheels.empty() && gu.tvcTorque == 0.0 && !gu.surfaces.empty(),
                   "no drum and no gimbal, but a plate in the air");
        CHECK_TRUE(g.v->attitudeTorqueAbout(gu, side) > 0.0,
                   "and the capability model says it can pitch");
        const glm::dvec3 kr = g.v->killRotDemand(1.0 / 120.0, gu);
        printf("   killRotDemand on a fin-only hull = %.1f N m\n",
               glm::length(kr));
        CHECK_TRUE(glm::length2(kr) > 0.0,
                   "so kill-rot asks it for something: 'no authority at all' has to "
                   "mean no ROWS, and since #426 the surfaces are rows");
        g.v->drag_cd = 0.0;
        CHECK_NEAR(0.0, g.v->attitudeTorqueAbout(g.v->attitudeAuthority(), side), 0.0,
                   "and with aero switched off it has no attitude authority at all");
        g.v->stick[1] = 1.0f;
        g.v->hull->btBody->clearForces();
        g.v->applyRotationForce(1.0 / 120.0);
        const btVector3 t3 = g.v->hull->btBody->getTotalTorque();
        const double mechG = glm::length(glm::dvec3(t3.x(), t3.y(), t3.z()));
        printf("   fin-only hull, held key, --drag-cd 0: hull torque %.4f N m\n",
               mechG);
        CHECK_NEAR(0.0, mechG, 0.0,
                   "nothing promised, nothing spent -- the rows must not exist, which "
                   "is what the check above is for");
    }
    destroyShip(g.v);
    delete body;
}

/* THE AIM IS MEASURED, NOT RE-DERIVED (#400 stage F2). Vehicle::pointingTo() answers
   "what does aiming at this direction mean for this hull" once: the error, the axis
   that closes it, the inertia about that axis and the authority-limited rate. Before
   it, the slew law and the [slew] trace each computed all four ~25 lines apart, and
   the trace's copy of the target test was looser -- normalize(0) -> NaN -> a row that
   read alpha=0.000 beside a ship actually slewing (src/vehicle.cpp::slew_log). Guidance
   is the third reader, and it is the reader that cannot afford a fourth opinion.
   The rig is the #426 one with --drag-cd 0 (no surface rows), so its whole attitude
   authority is its 100 N m TRIAD and every number below is hand-arithmetic: about a
   body axis the triad brings 100 N m, about a 45 deg axis in its plane 100*sqrt(2),
   because actuators.h::actuatorTorqueAbout sums limit*|dot(row, ask)|. */
static void test_the_aim_is_measured_once() {
    printf("== pointingTo: one measurement, three readers ==\n");
    TerrainBody *body = mkAirBall();
    Rig r = mkRig(100.0, 0.0);
    r.v->m_parent = body;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    r.v->drag_cd = 0.0;
    const Vehicle::AttitudeAuthority au = r.v->attitudeAuthority();
    CHECK_TRUE(au.wheels.size() == 3 && au.surfaces.empty(),
               "a triad and nothing else: the air is out of this answer on purpose");
    const glm::dvec3 nose = r.v->partAxis(r.pod, 2);   // identity pose: +Z
    const glm::dmat3 I = r.v->getInertia();

    /* 90 deg about a body axis: dir +X, so axis = cross(nose, dir) = +Y. */
    const glm::dvec3 dir(1.0, 0.0, 0.0);
    const Vehicle::Pointing p = r.v->pointingTo(dir, au);
    CHECK_TRUE(p.valid, "a real target gives a real answer");
    CHECK_NEAR(p.angle, glm::pi<double>() * 0.5, 1e-12, "a quarter turn is a quarter turn");
    CHECK_NEAR(glm::length(p.facing), 1.0, 1e-12, "facing is the frame, and it is unit");
    CHECK_NEAR(glm::length(p.axis), 1.0, 1e-12, "axis is unit -- the laws multiply it by a rate");
    const glm::dvec3 wantAxis = glm::normalize(glm::cross(nose, dir));
    CHECK_NEAR(glm::dot(p.axis, wantAxis), 1.0, 1e-12,
               "axis = normalize(cross(facing, target)): the short way round");
    CHECK_NEAR(glm::dot(p.axis, p.facing), 0.0, 1e-12,
               "and it is transverse -- this law cannot ask for roll, which is the "
               "allocator's job (#420), not the aim's");
    CHECK_NEAR(glm::dot(wantAxis, I * wantAxis), p.Ieff, 1e-9,
               "Ieff is the hull's inertia ABOUT THE AXIS THIS ASK NEEDS, not a scalar");
    CHECK_NEAR(p.alphaMax * p.Ieff, 100.0, 1e-6,
               "the torque the braking curve scales with is the drum's 100 N m, read "
               "about this axis -- nothing else is left in the rig");

    /* The axis matters: 90 deg the other way, aimed 45 deg between two drum axes. */
    const glm::dvec3 dir45 = glm::normalize(glm::dvec3(1.0, 1.0, 0.0));
    const Vehicle::Pointing q = r.v->pointingTo(dir45, au);
    CHECK_NEAR(q.alphaMax * q.Ieff, 100.0 * std::sqrt(2.0), 1e-6,
               "about a 45 deg axis the triad brings 100*sqrt(2), so a law that "
               "budgetted one number for every ask would brake too hard here");

    /* The three degeneracies, kept apart from each other. */
    const Vehicle::Pointing same = r.v->pointingTo(nose, au);
    CHECK_TRUE(same.valid && same.angle < 1e-12,
               "an aligned hull is valid with angle 0 -- 'already there' is not "
               "'no target', and guidance has to tell them apart");
    const Vehicle::Pointing away = r.v->pointingTo(-nose, au);
    CHECK_TRUE(away.valid, "a target behind the nose is still a target");
    CHECK_NEAR(away.angle, glm::pi<double>(), 1e-9, "and it is a half turn");
    CHECK_NEAR(glm::length(away.axis), 1.0, 1e-12,
               "the half-turn branch still hands back a UNIT axis: cross() is ~0 here, "
               "so normalizing it used to be the NaN door");
    CHECK_NEAR(glm::dot(away.axis, away.facing), 0.0, 1e-12,
               "perpendicular to the stack, which is the only kind of axis that can "
               "close a half turn");
    CHECK_NEAR(away.alphaMax * away.Ieff, 100.0, 1e-6,
               "and the axis it picks is one the drums can actually drive, so a half "
               "turn still has a braking curve -- a degenerate axis would leave this 0 "
               "and the law would quietly hold position 180 deg out");
    const Vehicle::Pointing none = r.v->pointingTo(glm::dvec3(0.0), au);
    CHECK_TRUE(!none.valid, "no direction at all is not a direction");
    CHECK_NEAR(0.0, none.Ieff, 0.0, "and it buys no numbers -- all zero, no NaN");
    CHECK_NEAR(0.0, none.alphaMax, 0.0, "so a law reading it holds position instead of "
               "pretending to slew");

    /* One answer per reader, not one per call site: the [slew] trace asks the
       one-argument form, which builds its own authority. */
    const Vehicle::Pointing logged = r.v->pointingTo(dir);
    CHECK_TRUE(logged.valid, "the log's form works off the same hull");
    CHECK_NEAR(logged.angle, p.angle, 1e-15, "same E");
    CHECK_NEAR(glm::dot(p.axis, logged.axis), 1.0, 1e-15, "same axis");
    CHECK_NEAR(logged.Ieff, p.Ieff, 1e-12, "same Ieff");
    CHECK_NEAR(logged.alphaMax, p.alphaMax, 1e-15, "same alphaMax");

    /* And the LAW spends these numbers rather than its own: at rest the demand is the
       braking-curve rate through the whole tensor, so the vector below is a hand
       check that the axis and the rate the law aims for are the primitive's. */
    const double h = 1.0 / 120.0;
    const double w_des = std::sqrt(2.0 * p.alphaMax * p.angle);
    CHECK_TRUE(w_des < p.angle / (2.0 * h),
               "this substep is rate-limited by the actuators, not by the substep cap");
    const glm::dvec3 tau = r.v->slewDemand(dir, h, au);
    const glm::dvec3 want = I * (p.axis * w_des) / h;
    printf("   slewDemand = (%.1f %.1f %.1f) N m, want (%.1f %.1f %.1f)"
           " (alpha %.3f rad/s^2, w_des %.3f rad/s)\n",
           tau.x, tau.y, tau.z, want.x, want.y, want.z, p.alphaMax, w_des);
    /* All three components, not one: this rig's tensor is diagonal in world axes, so
       its x column is zero for BOTH the aim axis (+Y) and the stack axis (+Z) and a
       single-component check passes on a law aimed down the wrong axis. */
    CHECK_NEAR(glm::length(tau - want), 0.0, 1e-9 * glm::length(want),
               "the law's torque is exactly I*(axis*w_des)/h -- one axis, one rate, "
               "no second opinion");
    const glm::dvec3 tauAway = r.v->slewDemand(-nose, h, au);
    const glm::dvec3 wantAway =
        I * (away.axis * std::sqrt(2.0 * away.alphaMax * away.angle)) / h;
    CHECK_NEAR(glm::length(tauAway - wantAway), 0.0, 1e-9 * glm::length(wantAway),
               "a half turn is the same law on the same measurement: the axis the "
               "primitive picks is the axis the law drives");
    CHECK_NEAR(glm::length(r.v->slewDemand(nose, h, au)), 0.0, 0.0,
               "aligned: the aim is valid and the law still asks for nothing");
    CHECK_NEAR(glm::length(r.v->slewDemand(glm::dvec3(0.0), h, au)), 0.0, 0.0,
               "and with no target the law does nothing rather than slewing on garbage");
    destroyShip(r.v);
    delete body;
}

/* The #400 review pass. Every pin here is a mutation that survived the whole suite:
   a NaN ask, an overflowing ask, the antipodal threshold, and the one rate the slew
   law deliberately does NOT damp. */
static void test_the_aim_keeps_its_doors_shut() {
    printf("== the aim's doors, and the roll the law leaves alone (#400 review) ==\n");
    TerrainBody *body = mkAirBall();
    Rig r = mkRig(100.0, 0.0);
    r.v->m_parent = body;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    r.v->drag_cd = 0.0;
    const Vehicle::AttitudeAuthority au = r.v->attitudeAuthority();
    const glm::dvec3 nose = r.v->partAxis(r.pod, 2);   // identity pose: +Z
    const double h = 1.0 / 120.0;

    /* A NaN direction used to come back valid with a NaN angle and axis; 1e300
       overflowed length2 to inf, normalized to ~0, and read as 90 deg OFF the nose --
       wrong rather than NaN, which is worse to notice. Both fail closed now (#609),
       because Pointing is public and guidance is told to read it. */
    const Vehicle::Pointing bad = r.v->pointingTo(glm::dvec3(0.0, 0.0, std::nan("")), au);
    CHECK_TRUE(!bad.valid, "a NaN direction is not a direction");
    CHECK_NEAR(bad.angle, 0.0, 0.0, "and it reports no error rather than a NaN one");
    const Vehicle::Pointing blow = r.v->pointingTo(glm::dvec3(1e300, 0.0, 0.0), au);
    CHECK_TRUE(!blow.valid, "an overflowing direction is not a direction either");
    CHECK_NEAR(blow.angle, 0.0, 0.0, "it does not get to read as a quarter turn");

    /* Almost astern is a normal ask and must still use the REAL cross product. Widening
       the antipodal threshold from 1e-12 to 1e-2 -- so every ask within 8.6 deg of a
       half turn takes the fallback -- left the suite green, because nothing here asked.
       Which axis the exactly-astern case picks is decided below (#609 part 2); that it
       is unit and transverse is not up for grabs. */
    const double a = 0.5 * glm::pi<double>() / 180.0;
    const glm::dvec3 hair = glm::normalize(glm::dvec3(0.0, std::sin(a), -std::cos(a)));
    const Vehicle::Pointing close = r.v->pointingTo(hair, au);
    CHECK_TRUE(close.valid, "half a degree off astern is an ask like any other");
    CHECK_NEAR(close.angle, glm::pi<double>() - a, 1e-9, "and reads half a degree off pi");
    CHECK_NEAR(glm::dot(close.axis, glm::normalize(glm::cross(nose, hair))), 1.0, 1e-9,
               "so it turns the short way about the real axis, not the fallback");
    const Vehicle::Pointing astern = r.v->pointingTo(-nose, au);
    CHECK_NEAR(glm::length(astern.axis), 1.0, 1e-12, "the exactly-astern fallback is unit");
    CHECK_NEAR(glm::dot(astern.axis, nose), 0.0, 1e-12, "and transverse like any other axis");
    /* WHICH line the fallback picks is no longer a coin flip (#609 part 2). It used to
       choose a WORLD axis by where the nose happened to sit -- fabs(facing.y) > 0.9 --
       so the same ask on the same hull turned about a different line as the hull rolled
       across |y| = 0.9, and a microradian of dither at antipodes alternated +Y with -Y
       on consecutive substeps, each time spending the same torque about another line.
       It is the hull's own right now: pitch over, which is what "point retrograde"
       means to a pilot, and the same answer in hull terms at any world orientation. */
    CHECK_NEAR(glm::dot(astern.axis, r.v->partAxis(r.pod, 0)), 1.0, 1e-12,
               "exactly astern turns about the hull's own right");
    r.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0),
                   glm::dmat3(glm::angleAxis(glm::radians(90.0), glm::dvec3(0.0, 1.0, 0.0))));
    const glm::dvec3 noseY = r.v->partAxis(r.pod, 2);   // now world +X, so facing.y = 0
    const Vehicle::Pointing astY = r.v->pointingTo(-noseY, r.v->attitudeAuthority());
    CHECK_NEAR(astY.angle, glm::pi<double>(), 1e-9,
               "fixture guard: after the yaw the ask really is exactly antipodal");
    CHECK_NEAR(glm::dot(astY.axis, r.v->partAxis(r.pod, 0)), 1.0, 1e-12,
               "and the fallback follows the hull: its own right, now world -Z");
    CHECK_NEAR(glm::dot(astY.axis, glm::dvec3(0.0, 1.0, 0.0)), 0.0, 1e-12,
               "so it is not the world +Y the old selector answered with (the pin above "
               "already fixes the line; this one only names the world axis that moved)");
    r.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));

    /* The law damps the TRANSVERSE rate only: roll is the wheels' line and the player's,
       not the aim's (#576). Handing the law the whole rate instead passed 202 checks --
       and a demand that fought roll would hold the roll hostage to the aim. */
    r.v->hull->btBody->setAngularVelocity(btVector3(0.0f, 0.0f, 2.0f));  // roll about +Z
    const glm::dvec3 dir(1.0, 0.0, 0.0);
    const Vehicle::Pointing p = r.v->pointingTo(dir, au);
    const glm::dvec3 tau = r.v->slewDemand(dir, h, au);
    CHECK_NEAR(glm::dot(tau, nose), 0.0, 1e-9,
               "a rolling hull gets no torque ABOUT ITS NOSE from an aim ask");
    CHECK_NEAR(glm::dot(tau, p.axis), glm::length(tau), 1e-9,
               "and all of what it does get is along the aim axis");
    CHECK_NEAR(glm::length(r.v->slewDemand(glm::dvec3(0.0), h, au)), 0.0, 0.0,
               "no target buys nothing even while spinning: the law is not a kill-rot");
    destroyShip(r.v);
    delete body;
}

/* THE OTHER HALF OF #609 PART 2, which the review pass caught. At antipodes the axis
   is FREE, and a free choice can land on a line nothing levers: attitudeTorqueAbout()
   is directional (#576), so about an uncovered line alphaMax is 0 and slewDemand
   holds position for ever without ever saying so. The hull's own right stays the
   convention -- the pin above proves that when the right line IS covered -- and steps
   aside only when it buys nothing. */
static void test_an_antipodal_ask_steps_off_a_dead_line() {
    printf("== at antipodes, a line nothing levers is not an option ==\n");
    FinProbe *v = new FinProbe;     // slewDemand is protected; the probe opens it
    v->name = "deadline";
    std::deque<PartDef> defs;
    Part *root = mkPart(defs, "root", 100.0, 1.5, 0.0, 0.0);      // no torque: not a wheel
    v->setRoot(root);
    Part *drum = mkPart(defs, "drum", 100.0, 1.0, 50.0, 0.0);     // ONE drum, one line
    v->attachDown(drum);
    drum->localRot = glm::dmat3(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);  // +Z -> -Y
    v->init();
    v->placeShip(glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    const Vehicle::AttitudeAuthority au = v->attitudeAuthority();
    const glm::dvec3 right = v->partAxis(root, 0);
    const glm::dvec3 up = v->partAxis(root, 1);
    const glm::dvec3 nose = v->partAxis(root, 2);
    CHECK_TRUE(!root->isWheel(), "the root is not a drum, so it covers no line itself");
    CHECK_NEAR(v->attitudeTorqueAbout(au, right), 0.0, 1e-12,
               "the fixture: the one drum lies along up, so hull right is dead");
    CHECK_NEAR(v->attitudeTorqueAbout(au, up), 50.0, 1e-9,
               "and hull up is worth that drum's full rating");
    const Vehicle::Pointing p = v->pointingTo(-nose, au);
    CHECK_TRUE(p.valid, "astern is an ask like any other");
    CHECK_NEAR(glm::dot(p.axis, up), 1.0, 1e-12,
               "so the free choice at antipodes takes the hull's UP, not its right");
    CHECK_NEAR(glm::dot(p.axis, right), 0.0, 1e-12, "and leaves the dead line alone");
    CHECK_NEAR(p.alphaMax * p.Ieff, 50.0, 1e-9,
               "the rate budget is that drum over the inertia about the axis it chose");
    CHECK_TRUE(glm::length(v->slewDemand(-nose, 1.0 / 120.0, au)) > 0.0,
               "and the law asks for torque: a dead axis used to mean silence");
    destroyShip(v);
}

/* ... but it steps aside ONLY when the right buys nothing. With both lines covered,
   the answer stays the hull's right even when the up line is worth MORE: "whichever
   buys more" would flip the invented axis as the wheel budget moved (wheels off, a
   dead drum, EC sag), which is the per-substep alternation #609 was about. */
static void test_the_fallback_convention_does_not_chase_the_bigger_number() {
    printf("== the antipodal convention is the hull's right, not the biggest drum ==\n");
    Ship s;
    s.v = new Vehicle;
    s.v->name = "twodrums";
    Part *root = mkPart(s.defs, "root", 100.0, 1.5, 0.0, 0.0);
    s.v->setRoot(root);
    Part *drumUp = mkPart(s.defs, "drum_up", 100.0, 1.0, 50.0, 0.0);
    s.v->attachDown(drumUp);
    drumUp->localRot = glm::dmat3(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);  // +Z -> -Y
    Part *drumRight = mkPart(s.defs, "drum_right", 20.0, 1.0, 20.0, 0.0);
    s.v->attachDown(drumRight);
    drumRight->localRot = glm::dmat3(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);  // +Z -> +X
    s.v->init();
    s.v->placeShip(glm::dvec3(0.0, 0.0, 0.0), glm::dmat3(1.0));
    const Vehicle::AttitudeAuthority au = s.v->attitudeAuthority();
    const glm::dvec3 right = s.v->partAxis(root, 0);
    const glm::dvec3 up = s.v->partAxis(root, 1);
    const glm::dvec3 nose = s.v->partAxis(root, 2);
    CHECK_NEAR(s.v->attitudeTorqueAbout(au, right), 20.0, 1e-9, "the right line is covered, thinly");
    CHECK_NEAR(s.v->attitudeTorqueAbout(au, up), 50.0, 1e-9, "and the up line is worth more");
    const Vehicle::Pointing p = s.v->pointingTo(-nose, au);
    CHECK_NEAR(glm::dot(p.axis, right), 1.0, 1e-12,
               "so the fallback stays the hull's right -- a convention that chased the "
               "bigger number would alternate lines as the drum budget moved");
    destroyShip(s.v);
}

/* #606: the aim channel. Until now a law could MEASURE a turn (pointingTo) but could
   not ASK for one, because slewDemand()/slewTargetDir() are protected and the only
   attitude channels were the six named SlewModes plus the stick. */
static void test_a_law_can_name_its_own_aim() {
    printf("== Sas/SlewFree: a law names the direction it steers at (#606) ==\n");
    TerrainBody *body = mkAirBall();
    Rig r = mkRig(100.0, 0.0);
    r.v->m_parent = body;
    r.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
    r.v->drag_cd = 0.0;
    const Vehicle::AttitudeAuthority au = r.v->attitudeAuthority();
    const glm::dvec3 nose = r.v->partAxis(r.pod, 2);   // identity pose: +Z
    const double h = 1.0 / 120.0;

    /* An aim arrives through Command, like every other control (#424's one door) -- and
       by itself it latches NOTHING, so a law that forgets ShipCmd(Sas, SlewFree) is
       visibly inert rather than secretly flying. */
    r.v->Command(ShipCmd(Aim, glm::dvec3(1.0, 0.0, 0.0)), true, 1.0);
    CHECK_TRUE(r.v->slewFreeDir() == glm::dvec3(1.0, 0.0, 0.0),
               "ShipCmd(Aim, dir) writes the aim");
    CHECK_TRUE(r.v->slewRequestMode() == SlewNone && r.v->slewMode() == SlewNone,
               "and aims at nothing until a mode is latched");

    /* A paused tick writes nothing, setpoint channels included. */
    r.v->Command(ShipCmd(Aim, glm::dvec3(0.0, 1.0, 0.0)), false, 1.0);
    CHECK_TRUE(r.v->slewFreeDir() == glm::dvec3(1.0, 0.0, 0.0),
               "a paused sim does not take a law's aim");

    /* The latch is the same toggle rule as every other mode (#424): one press engages,
       the same press again releases. A law that re-sent the Sas command every tick would
       flicker -- which is why the aim is a separate channel. */
    r.v->Command(ShipCmd(Sas, SlewFree), true, 1.0);
    CHECK_TRUE(r.v->slewRequestMode() == SlewFree, "SlewFree latches like any mode");
    CHECK_TRUE(r.v->applySlewRequest(true) && r.v->slewMode() == SlewFree,
               "and the once-per-tick copy engages it");
    CHECK_TRUE(r.v->slewTargetDir() == glm::dvec3(1.0, 0.0, 0.0),
               "the engaged mode steers at the law's vector, not one the class invented");
    r.v->Command(ShipCmd(Sas, SlewFree), true, 1.0);
    CHECK_TRUE(r.v->slewRequestMode() == SlewNone, "and the same command releases it");
    r.v->Command(ShipCmd(Sas, SlewFree), true, 1.0);

    /* The aim is STATE, not per-tick intent: clearRotCmd() drops the stick and the
       engaged copy, and must not forget the direction a law named. */
    r.v->clearRotCmd();
    CHECK_TRUE(r.v->slewFreeDir() == glm::dvec3(1.0, 0.0, 0.0),
               "the aim outlives the tick, like the latch it belongs to");

    /* An aim belongs to the mode that asked for it: while a NAMED mode is engaged,
       the free vector must not hijack its target. */
    r.v->Command(ShipCmd(Sas, SlewRadialOut), true, 1.0);
    r.v->applySlewRequest(true);
    CHECK_NEAR(glm::dot(glm::normalize(r.v->slewTargetDir()), nose), 1.0, 1e-9,
               "radial-out still means radial-out with a stale aim lying around");
    r.v->Command(ShipCmd(Sas, SlewRadialOut), true, 1.0);   // release
    r.v->Command(ShipCmd(Sas, SlewFree), true, 1.0);
    r.v->applySlewRequest(true);

    /* The channel is honest about having no target: prograde of a ship sitting still is
       not a direction, and a law must be able to say so rather than being handed last
       tick's answer. pointingTo()'s door (#609) fails it closed and the law asks for
       nothing -- no NaN reaches the allocator. */
    r.v->Command(ShipCmd(Aim, glm::dvec3(0.0)), true, 1.0);
    CHECK_TRUE(!r.v->pointingTo(r.v->slewTargetDir(), au).valid,
               "an empty aim is not a direction");
    CHECK_NEAR(glm::length(r.v->slewDemand(r.v->slewTargetDir(), h, au)), 0.0, 0.0,
               "and the law asks for no torque from it");
    r.v->Command(ShipCmd(Aim, glm::dvec3(0.0, 0.0, std::nan(""))), true, 1.0);
    CHECK_NEAR(glm::length(r.v->slewDemand(r.v->slewTargetDir(), h, au)), 0.0, 0.0,
               "nor from a NaN one");

    /* And the whole point: what the free channel delivers is the SAME torque the law
       would compute directly, so a guidance law adds no second physics path. */
    const glm::dvec3 dir(1.0, 0.0, 0.0);
    r.v->Command(ShipCmd(Aim, dir), true, 1.0);
    const glm::dvec3 via_channel = r.v->slewDemand(r.v->slewTargetDir(), h, au);
    const glm::dvec3 direct = r.v->slewDemand(dir, h, au);
    CHECK_NEAR(glm::length(via_channel - direct), 0.0, 0.0,
               "aim-through-SlewFree and a direct ask are one answer, not two laws");
    CHECK_NEAR(glm::length(via_channel), 22365.6, 1.0,
               "and it is the 100 N m triad's braking torque, in N m");
    destroyShip(r.v);
    delete body;
}

int main() {
    test_fixture_lever_is_what_we_think();
    test_tvc_is_a_disk_not_a_ball();
    test_wheels_and_tvc_together();
    test_a_ship_with_nothing_has_nothing();
    test_two_wheels_add_up();
    test_a_cluster_covers_an_axis_it_points_at();
    test_a_triad_housing_covers_three_lines();
    test_the_stick_demands_one_torque();
    test_the_wheels_are_billed_once();
    test_the_pilot_and_the_law_split_by_axis();
    test_a_switched_off_drum_is_not_a_row();
    test_one_frame_for_the_attitude_path();
    test_the_command_funnel();
    test_the_armed_rcs_demand_is_a_direction();
    test_a_fin_row_is_spent_and_hands_its_ask_to_the_air();
    test_the_aim_is_measured_once();
    test_the_aim_keeps_its_doors_shut();
    test_an_antipodal_ask_steps_off_a_dead_line();
    test_the_fallback_convention_does_not_chase_the_bigger_number();
    test_a_law_can_name_its_own_aim();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_authority: all checks passed\n");
    return 0;
}
