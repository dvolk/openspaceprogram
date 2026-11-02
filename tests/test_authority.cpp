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
        if (!std::isfinite(_a) || std::fabs(_a - _e) > (tol)) {                \
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

/* A box part with the two fields this model reads: `torque` (>0 makes it a
   reaction wheel) and `gimbal_max` (>0 makes its thrust steerable). */
static Part *mkPart(Ship &s, const char *name, double mass, double hz,
                    double torque, double gimbalMax) {
    Body *b = new Body;
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(0.5f, 0.5f, (float)hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name   = name;
    d.mass   = (float)mass;
    d.radius = 0.5f;
    d.height = (float)(2.0 * hz);
    d.torque = torque;
    d.gimbal_max = gimbalMax;
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
    st.root = mkPart(st.s, "root", 100.0, 1.5, rootTorque, 0.0);
    st.s.v->setRoot(st.root);
    st.eng = mkPart(st.s, "engine", 100.0, 1.0, 0.0, gimbalMax);
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
    CHECK_NEAR(a.wheelCapacity(), 200.0, 1e-12, "the wheel's rated torque");
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
    Part *w2 = mkPart(st.s, "wheel2", 100.0, 1.0, 50.0, 0.0);
    v->attachDown(w2);
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_TRUE(a.wheels.size() == 2, "both wheels are rows, not one summed number");
    CHECK_NEAR(a.wheelCapacity(), 250.0, 1e-12, "200 + 50 N m of wheel, both of them");
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
    Part *root = mkPart(s, "root", 100.0, 1.5, 100.0, 0.0);
    s.v->setRoot(root);
    Part *drumY = mkPart(s, "drum_y", 100.0, 1.0, 50.0, 0.0);
    s.v->attachDown(drumY);
    Part *drumX = mkPart(s, "drum_x", 100.0, 1.0, 20.0, 0.0);
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
    CHECK_NEAR(a.wheelCapacity(), 170.0, 1e-12, "the row sum is 100 + 50 + 20");
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
    CHECK_NEAR(a.wheelCapacity(), 600.0, 1e-12, "three drums at the rating each");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 200.0, 1e-9, "one drum on the stack line");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), 200.0, 1e-9, "one on the pitch line");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.y), 200.0, 1e-9, "one on the yaw line");
    /* The same hull with the housing rolled 90 deg about the stack: the box turns,
       so its drums move between the transverse lines and the coverage does not.
       A single drum mounted this way reads 0 about the axis it just left. */
    st.root->localRot = glm::dmat3(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    const Vehicle::AttitudeAuthority r = v->attitudeAuthority();
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.z), 200.0, 1e-9, "still one drum on the stack line");
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.x), 200.0, 1e-9, "still one on a transverse line");
    CHECK_NEAR(v->attitudeTorqueAbout(r, st.y), 200.0, 1e-9, "still one on the third");
    /* And delivery agrees with the budget: a demand far past one rating along one
       line is answered with that line's drum -- not with the 600 N m the housing
       owns, and not with two drums leaning on an axis they do not point at. */
    std::vector<double> sp;
    const glm::dvec3 got = actuatorSolve(r.wheels, 5000.0 * st.x, sp);
    CHECK_NEAR(glm::length(got), 200.0, 1e-9, "one line buys its own rating");
    destroyShip(v);
}

/* ONE FRAME FOR THE WHOLE ATTITUDE PATH (#571). The stack axis TVC levers belongs
   to the ROOT part, and since #571 every consumer asks attitudeRef() for it: the
   mixer, attitudeAuthority, the manual stick, rcsWorldDir, the two traces, and
   slewToward's facing. Before it, the laws and the stick took firstWheel() -- a
   frame picked by parts-list order. Which half of that was observable:
     - the stick, YES: a surface mount keeps a part's +Z parallel to its parent's
       and ROLLS it about the stack, so a wheel on a tank's side has the ship's nose
       and the ship's UP axis as its "right" one -- W yaws, A pitches. Measured on
       res/ships/sideways_wheel.json (e2e flight/571), not here: this binary has no
       physics world, so ApplyTorque is not observable.
     - the fixture below, in a shape a def CANNOT produce: localRot is set by hand so
       the wheel's +Z is 90 deg off the stack. That is what makes the disk normal and
       the facing decidable here; a shipped mount can only roll a wheel, not tilt it.
     - killRotStep and the killrot trace row: NO -- they used the reference part only
       to read the hull's rate, which is the same for every part (hullAngVel(), #574).
   What a model that took the wheel's +Z as the TVC disk's normal WOULD do is here and
   now pinned: it re-opens the hole step B closed, crediting TVC with roll. */
static void test_one_frame_for_the_attitude_path() {
    printf("== one frame: the ROOT's axes, for the disk normal and the facing ==\n");
    Ship s;
    s.v = new Vehicle;
    s.v->name = "gear";
    Part *root = mkPart(s, "root", 100.0, 1.5, 0.0, 0.0);     // no wheel here
    s.v->setRoot(root);
    Part *eng = mkPart(s, "engine", 100.0, 1.0, 0.0, 0.1);    // gimballed, armed below
    s.v->attachDown(eng);
    Part *wheel = mkPart(s, "wheel", 100.0, 0.5, 300.0, 0.0); // a wheel, on its side
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
    CHECK_NEAR(a.wheelCapacity(), 300.0, 1e-12, "the drum is a row on this hull");
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

int main() {
    test_fixture_lever_is_what_we_think();
    test_tvc_is_a_disk_not_a_ball();
    test_wheels_and_tvc_together();
    test_a_ship_with_nothing_has_nothing();
    test_two_wheels_add_up();
    test_a_cluster_covers_an_axis_it_points_at();
    test_a_triad_housing_covers_three_lines();
    test_one_frame_for_the_attitude_path();
    test_the_command_funnel();
    test_the_armed_rcs_demand_is_a_direction();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_authority: all checks passed\n");
    return 0;
}
