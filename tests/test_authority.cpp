// The attitude CAPABILITY model (#425 step B): Vehicle::attitudeAuthority() and
// Vehicle::attitudeTorqueAbout() -- one answer to "what can turn this ship right
// now, and how much of it reaches THIS axis".
//
// Why this file exists: before it, three places (the slew law, the kill-rot law and
// the [slew] trace) each re-added maxTorque() + tvcAuthority()'s tvcMax, and none of
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
    CHECK_TRUE(a.wheelTorque == 0.0, "a root with no `torque` is not a wheel");
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
    junk.wheelTorque = 5.0;
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
    printf("== wheels are isotropic, TVC is not, and the sum is per-axis ==\n");
    Stack st = mkStack(200.0, 800.0, 0.1);
    Vehicle *v = st.s.v;
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    const double tvc = 1000.0 * std::sin(0.1);
    CHECK_NEAR(a.wheelTorque, 200.0, 1e-12, "the wheel's rated torque");
    CHECK_NEAR(a.tvcTorque, tvc, 1e-9, "and the engine's, side by side in one answer");

    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 200.0, 1e-9,
               "about the stack axis the ship has its wheels and NOTHING else");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.x), 200.0 + tvc, 1e-9,
               "transverse: both actuators can pull the same way");

    /* The wheels are electric: no EC, no wheel row. Everything about the stack
       axis then goes to zero -- which is the #418 reading a player sees as a
       dead ship that will not turn, and is the truth rather than a fudge. */
    v->powered_ = false;
    const Vehicle::AttitudeAuthority dead = v->attitudeAuthority();
    CHECK_TRUE(dead.wheelTorque == 0.0, "unpowered wheels contribute nothing");
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
    CHECK_TRUE(a.wheelTorque == 0.0 && a.tvcTorque == 0.0 && a.heffMag == 0.0,
               "a coasting ship has no attitude authority");
    /* The laws' early-out (`budget <= 0` means "cannot turn, do not pretend").
       Pinned per axis, not as a sum: a pair of sign errors that cancel would pass a
       summed check, and the early-out is what makes a dead ship read as dead. */
    CHECK_TRUE(v->attitudeTorqueAbout(a, st.x) == 0.0, "no authority about pitch");
    CHECK_TRUE(v->attitudeTorqueAbout(a, st.z) == 0.0, "no authority about roll");
    destroyShip(st.s.v);
}

/* --- which wheel number, and whose frame ---------------------------------- */
/* The wheel row is the SUM (maxTorque), while the Vessel window prints
   GetWheelTorque() -- the LARGEST wheel. They agree on a one-wheel ship, so
   nothing else in the tree would notice one being swapped for the other, and a
   two-wheel ship would silently lose a third of the authority the laws budget. */
static void test_two_wheels_add_up() {
    printf("== the wheel row sums the wheels; the Vessel window shows the biggest ==\n");
    Stack st = mkStack(200.0, 0.0, 0.0);
    Vehicle *v = st.s.v;
    Part *w2 = mkPart(st.s, "wheel2", 100.0, 1.0, 50.0, 0.0);
    v->attachDown(w2);
    const Vehicle::AttitudeAuthority a = v->attitudeAuthority();
    CHECK_NEAR(a.wheelTorque, 250.0, 1e-12, "200 + 50 N m of wheel, both of them");
    CHECK_NEAR(v->attitudeTorqueAbout(a, st.z), 250.0, 1e-9,
               "and the wheels are isotropic: all of it about the stack axis");
    destroyShip(st.s.v);
}

/* The mixer's stack axis belongs to the ROOT part, not to whatever wheel the
   attitude reference happens to use. slewToward/killRotStep/slew_log pick
   firstWheel() as their FACING reference and fall back to rootPart(); a model
   that took the wheel's +Z as the TVC disk's normal would re-open exactly the
   hole step B closed -- crediting TVC with roll -- on any ship whose wheel is
   mounted at an angle to the stack (landing gear, a pod docked sideways). */
static void test_the_stack_axis_is_the_roots() {
    printf("== the TVC disk is normal to the ROOT's stack axis, not the wheel's ==\n");
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
    CHECK_TRUE(a.wheelTorque == 300.0 && a.tvcTorque > 0.0,
               "both actuators are live on this hull");
    /* About the stack axis: wheels only. If the model read the wheel's axis, this
       would be 300 + tvc (the wheel's axis is perpendicular to the stack, so TVC
       would count in full) -- the bug, caught. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, rootZ), 300.0, 1e-9,
               "no TVC about the stack axis even when a wheel leans elsewhere");
    /* And about the wheel's own axis, which is transverse to the stack, TVC does
       count in full. */
    CHECK_NEAR(v->attitudeTorqueAbout(a, wheelZ), 300.0 + a.tvcTorque, 1e-9,
               "TVC counts where the mixer can actually lever");
    destroyShip(s.v);
}

int main() {
    test_fixture_lever_is_what_we_think();
    test_tvc_is_a_disk_not_a_ball();
    test_wheels_and_tvc_together();
    test_a_ship_with_nothing_has_nothing();
    test_two_wheels_add_up();
    test_the_stack_axis_is_the_roots();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_authority: all checks passed\n");
    return 0;
}
