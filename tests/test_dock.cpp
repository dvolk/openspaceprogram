// Headless test for the docking merge/split primitives (src/vehicle.h):
// Vehicle::absorbShip (one ship rigidly absorbs another through a docking
// port) and Vehicle::extractSubtreeAsShip (the general "a part of this ship
// becomes a ship" split that undock -- and later dropped stages -- use).
//
// What is pinned:
//   - REBASE EXACTNESS: absorbShip is a rigid merge. Every absorbed part
//     keeps its exact WORLD pose (partWorldPose before == after), even though
//     its ship-local pose is re-based into the survivor's frame S and the
//     survivor's COM (hence hull transform) shifts to take on the new mass.
//   - SEAM: the joint is recorded (survivor port, absorbed root, its name),
//     which is what undock splits back apart.
//   - TOPOLOGY: the absorbed ship's root now hangs off the survivor's port;
//     the part list is the union.
//   - ROUND-TRIP: absorbShip then extractSubtreeAsShip(seam.root) restores
//     BOTH ships' parts to their original world poses -- the split is the
//     exact inverse of the merge on geometry. This is the invariant the
//     future "dropped stage becomes a ship" relies on.
//   - IDENTITY: Part::uid is minted per instance and stays distinct across a
//     merge of two ships whose Part::ids collide on EVERY part -- the case
//     two ships built from one def produce, since absorbShip does not rename.
//     `id` is provably ambiguous there (asserted, not assumed) and uid is
//     not, which is what makes uid the key save/load resolves by. Also pins
//     that a split neither remints nor reuses a uid.
//
// No physics world and no GL context: ships are built with init() (which
// runs rebuildCompound -- the one hull rigid body -- but NOT enterWorld) and
// placed with placeShip, exactly like test_inertia/test_fuel.
// extractSubtreeAsShip deliberately leaves enterWorld to its caller so this
// split can run headless.
//
// Runs from the repo root:
//   make test   (or: ./test_dock)

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdio>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "vehicle.h"   // Vehicle + Part (inline); body.h pulls in Bullet

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

/* A hand-built ship. `defs` is a deque, not a vector: push_back on a vector
   reallocates and would dangle every Part::def handed out so far. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* A box part (no rigid body of its own -- the ship is ONE body and the part
   is a child of its compound, carrying a collision shape, a mass and an
   authored pose). `port` marks a docking port. Parent/local pose are set by
   setRoot/attachDown, not here. */
static Part *mkPart(Ship &s, const char *name, double mass,
                    double hx, double hy, double hz, bool port) {
    Body *b = new Body;
    // mesh/shader/texture default to null (no GL in a headless test)
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(hx, hy, hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name         = name;
    d.mass         = (float)mass;
    d.radius       = (float)hx;
    d.height       = (float)(2.0 * hz);
    d.docking_port = port;
    d.synthesizeNodes();   // hand-built def: give it the axial stack nodes
                           // attachDown mates (a catalog part gets these from
                           // load_parts_catalog)

    Part *p = new Part;
    p->body  = b;
    p->def   = &d;
    p->stage = 1;
    return p;
}

/* tank (root) + docking port stacked on its rear (-Z) face -- the station. */
static const double kTankHz = 1.5;    // half-height (3 m tank)
static const double kPortHz = 0.125;  // half-height (0.25 m port)

static void destroyShip(Vehicle *v) {
    /* onRails keeps ~Vehicle off RemoveBody: nothing here was ever added to
       a physics world (there is none in this test). */
    v->onRails = true;
    delete v;
}

/* A part's world position, via the ship that currently owns it. */
static glm::dvec3 worldPos(Vehicle *owner, Part *p) {
    glm::dvec3 pos; glm::dmat3 rot;
    owner->partWorldPose(p, pos, rot);
    return pos;
}

/* --- absorb: rebase exactness + seam + topology -------------------------- */
static void test_absorb() {
    printf("== absorbShip: rigid rebase, seam, topology ==\n");

    /* A: tank (root) + port below it, at the world origin. */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);          // portA at local z = -(3.0+0.25)/2 = -1.625
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* B: port (root) + tank below it, placed so B's port face meets A's port
       face (A's port rear face is at world z = -1.625 - 0.125 = -1.75; B's
       port front face is at its origin + 0.125, so B sits at z = -1.875). */
    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);          // tankB at local z = -1.625
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    /* Capture B's world poses before the merge; they must survive it. */
    const glm::dvec3 portB0 = worldPos(B.v, portB);
    const glm::dvec3 tankB0 = worldPos(B.v, tankB);
    const glm::dvec3 tankA0 = worldPos(A.v, tankA);
    const glm::dvec3 portA0 = worldPos(A.v, portA);

    A.v->absorbShip(B.v, portA);

    /* Union of the parts; B is left an empty shell. */
    CHECK_TRUE(A.v->parts.size() == 4, "absorb: survivor holds all four parts");
    CHECK_TRUE(B.v->parts.empty(), "absorb: absorbed ship is an empty shell");

    /* Topology: B's root now hangs off A's port. */
    CHECK_TRUE(portB->parent == portA, "absorb: absorbed root hangs off the port");

    /* Seam recorded for undock. */
    CHECK_TRUE(A.v->seams.size() == 1, "absorb: one seam recorded");
    if(A.v->seams.size() == 1) {
        CHECK_TRUE(A.v->seams[0].port == portA, "absorb: seam port is the survivor's");
        CHECK_TRUE(A.v->seams[0].root == portB, "absorb: seam root is the absorbed root");
        CHECK_TRUE(A.v->seams[0].name == "B", "absorb: seam remembers the absorbed name");
    }

    /* Rebase exactness: every part keeps its world pose (the merge is rigid;
       only the ship-local poses and the survivor's COM/hull transform move). */
    const glm::dvec3 tankA1 = worldPos(A.v, tankA);
    const glm::dvec3 portA1 = worldPos(A.v, portA);
    const glm::dvec3 portB1 = worldPos(A.v, portB);
    const glm::dvec3 tankB1 = worldPos(A.v, tankB);
    CHECK_NEAR(glm::length(tankA1 - tankA0), 0.0, 1e-9, "absorb: tankA world pose kept");
    CHECK_NEAR(glm::length(portA1 - portA0), 0.0, 1e-9, "absorb: portA world pose kept");
    CHECK_NEAR(glm::length(portB1 - portB0), 0.0, 1e-9, "absorb: portB world pose kept");
    CHECK_NEAR(glm::length(tankB1 - tankB0), 0.0, 1e-9, "absorb: tankB world pose kept");

    /* The merged COM sits between the two tanks' COMs (mass-weighted), and
       the hull mass is the sum -- a sanity check that rebuildCompound ran. */
    CHECK_NEAR(A.v->hull->mass, 2100.0, 1e-6, "absorb: hull mass is the union");

    destroyShip(B.v);   // empty shell (its old compound references, not owns)
    destroyShip(A.v);
}

/* --- round-trip: absorb then extract restores both ships' geometry ------- */
static void test_roundtrip() {
    printf("== extractSubtreeAsShip: undock round-trip ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    const glm::dvec3 tankA0 = worldPos(A.v, tankA);
    const glm::dvec3 portA0 = worldPos(A.v, portA);
    const glm::dvec3 portB0 = worldPos(B.v, portB);
    const glm::dvec3 tankB0 = worldPos(B.v, tankB);

    A.v->absorbShip(B.v, portA);
    destroyShip(B.v);   // the shell is gone; its parts live on A now
    CHECK_TRUE(A.v->seams.size() == 1, "roundtrip: seam present after absorb");

    /* Undock: split the seam's subtree (B's old root) back into a ship. */
    Vehicle *out = A.v->extractSubtreeAsShip(A.v->seams[0].root, "B");
    CHECK_TRUE(out != nullptr, "roundtrip: extract returned a ship");
    if(out == nullptr) { destroyShip(A.v); return; }

    /* The survivor keeps its own two parts; the new ship has the other two. */
    CHECK_TRUE(A.v->parts.size() == 2, "roundtrip: survivor keeps two parts");
    CHECK_TRUE(out->parts.size() == 2, "roundtrip: extracted ship has two parts");
    CHECK_TRUE(out->rootPart() == portB, "roundtrip: extracted root is B's old root");

    /* Geometry restored: both ships' parts are back at their original world
       poses -- the split is the exact inverse of the merge. */
    CHECK_NEAR(glm::length(worldPos(A.v, tankA) - tankA0), 0.0, 1e-9,
               "roundtrip: tankA world pose restored");
    CHECK_NEAR(glm::length(worldPos(A.v, portA) - portA0), 0.0, 1e-9,
               "roundtrip: portA world pose restored");
    CHECK_NEAR(glm::length(worldPos(out, portB) - portB0), 0.0, 1e-9,
               "roundtrip: portB world pose restored");
    CHECK_NEAR(glm::length(worldPos(out, tankB) - tankB0), 0.0, 1e-9,
               "roundtrip: tankB world pose restored");

    destroyShip(out);
    destroyShip(A.v);
}

/* --- uid: identity that survives a merge which collides every id ---------- */

/* A two-part ship: tank + docking port, stacked face to face. `portIsRoot`
   picks which end is the root, so the caller can build the mated pair (one
   ship port-first, the other tank-first) that test_absorb docks.

   Part::id is set to the SAME strings for every ship built here -- which is
   exactly what two ships built from one def look like, since shipdef.cpp
   auto-generates "<catalog name>_<n>" per file and absorbShip never renames. */
static void mkTankPort(Ship &s, const char *shipName, double z, bool portIsRoot,
                       Part **tankOut, Part **portOut) {
    s.v = new Vehicle;
    s.v->name = shipName;
    Part *tank = mkPart(s, "fuel_tank", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *port = mkPart(s, "docking_port", 50.0, 1.0, 1.0, kPortHz, true);
    tank->id = "fuel_tank_1";
    port->id = "docking_port";
    if(portIsRoot) {
        s.v->setRoot(port);
        s.v->attachDown(tank);
        s.v->controller = port;
    } else {
        s.v->setRoot(tank);
        s.v->attachDown(port);
        s.v->controller = tank;
    }
    s.v->init();
    s.v->placeShip(glm::dvec3(0.0, 0.0, z), glm::dmat3(1.0));
    *tankOut = tank;
    *portOut = port;
}

/* True when every part in `v` has a nonzero uid and no two share one. */
static bool uidsUnique(const Vehicle *v) {
    std::set<uint64_t> seen;
    for(const Part *p : v->parts) {
        if(p->uid == 0) { return false; }
        if(!seen.insert(p->uid).second) { return false; }
    }
    return true;
}

/* True when at least two parts in `v` share a Part::id -- the collision that
   makes `id` unusable as a key once ships merge. */
static bool idsCollide(const Vehicle *v) {
    std::set<std::string> seen;
    for(const Part *p : v->parts) {
        if(!seen.insert(p->id).second) { return true; }
    }
    return false;
}

static void test_uid() {
    printf("== uid: unique per instance, distinct across an id-colliding merge ==\n");

    /* Same geometry as test_absorb: A tank-first at the origin, B port-first
       mated onto A's port face (A's port rear face is at z = -1.75, B's port
       front face is its origin + 0.125, so B sits at z = -1.875). */
    Ship A, B;
    Part *tankA = nullptr, *portA = nullptr, *tankB = nullptr, *portB = nullptr;
    mkTankPort(A, "A", 0.0,     false, &tankA, &portA);
    mkTankPort(B, "B", -1.875,  true,  &tankB, &portB);

    /* Minted, nonzero, and distinct before anything merges -- the two ships
       are independent builds, so a per-ship counter would pass the first and
       fail this one. */
    CHECK_TRUE(uidsUnique(A.v), "uid: ship A's parts are all distinct and nonzero");
    CHECK_TRUE(uidsUnique(B.v), "uid: ship B's parts are all distinct and nonzero");
    {
        std::set<uint64_t> both;
        for(const Part *p : A.v->parts) { both.insert(p->uid); }
        for(const Part *p : B.v->parts) { both.insert(p->uid); }
        CHECK_TRUE(both.size() == A.v->parts.size() + B.v->parts.size(),
                   "uid: distinct ACROSS two independently built ships");
    }

    /* Both ships were authored with the same ids, so the collision is present
       before the merge too -- assert it, or the check below proves nothing. */
    CHECK_TRUE(portA->id == portB->id, "uid: the two ships' port ids collide by construction");
    CHECK_TRUE(tankA->id == tankB->id, "uid: the two ships' tank ids collide by construction");

    A.v->absorbShip(B.v, portA);

    /* The property save/load resolution depends on: after the merge the
       string ids are ambiguous but the uids are not. */
    CHECK_TRUE(A.v->parts.size() == 4, "uid: merged ship holds all four parts");
    CHECK_TRUE(idsCollide(A.v), "uid: merged ship's string ids DO collide (the bug uid avoids)");
    CHECK_TRUE(uidsUnique(A.v), "uid: merged ship's uids are still all distinct");

    /* A split must not remint or reuse: the parts keep the uids they had. */
    const uint64_t portBuid = portB->uid;
    const uint64_t tankBuid = tankB->uid;
    Vehicle *out = A.v->extractSubtreeAsShip(portB, "B");
    CHECK_TRUE(out != nullptr, "uid: split succeeded");
    if(out != nullptr) {
        CHECK_TRUE(portB->uid == portBuid, "uid: split does not remint the root's uid");
        CHECK_TRUE(tankB->uid == tankBuid, "uid: split does not remint a child's uid");
        CHECK_TRUE(uidsUnique(out), "uid: split-off ship's uids are distinct and nonzero");
        CHECK_TRUE(uidsUnique(A.v), "uid: survivor's uids are distinct and nonzero");
        std::set<uint64_t> both;
        for(const Part *p : A.v->parts) { both.insert(p->uid); }
        for(const Part *p : out->parts) { both.insert(p->uid); }
        CHECK_TRUE(both.size() == 4, "uid: survivor and split-off share no uid");
        destroyShip(out);
    }

    destroyShip(B.v);   /* the empty shell absorbShip left behind */
    destroyShip(A.v);
}

int main() {
    test_absorb();
    test_roundtrip();
    test_uid();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_dock: all checks passed\n");
    return 0;
}
