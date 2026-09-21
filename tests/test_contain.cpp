// test_contain: the containment edge (Part::owner / container / contents,
// src/part.h) and its invariant check (Vehicle::checkPartInvariants,
// src/vehicle.cpp) -- step 2.1 of the inventory design report
// (reports/inventory-design2026_09_20/).
//
// What is pinned:
//   - OWNER: setRoot/attach wire every part's owner to its vehicle, and
//     absorbShip / extractSubtreeAsShip re-point it to the survivor / the
//     split-off ship. A Part* is stable through both, so only the pointer
//     moves -- which is what lets a contained part's container stay valid
//     across a merge or a split without any reindex machinery.
//   - THE EDGE: container/contents back-reference each other, and only a
//     character's part (isEva) may be contained, in a real capsule
//     (crew_capacity > 0). The check passes on a correctly wired chain and
//     FAILS on each of: a one-way edge (container set but not listed, and
//     the mirror), a contained non-character, a character in a non-capsule.
//
// The game populates no containment yet (step 2.4 wires the board/EVA
// transitions), so this test sets the edges by hand -- exactly the way
// those transitions will -- and deletes both vehicles at the end. The
// double-free the report warns about (an owning ~Part, with ~Vehicle
// deleting crew before parts) is a step-2.4 ASan case.
//
// A local TestCrew : Vehicle stands in for Kerbal: eva.cpp includes
// game.h, which drags the whole game into a headless test link. The
// invariant is keyed on the isEva() virtual, so the stand-in exercises the
// same code path the real Kerbal will take.
//
// No physics world and no GL context, exactly like test_dock: ships are
// built with init() (runs rebuildCompound -- which now asserts the
// containment invariant -- but not enterWorld).
//
// Runs from the repo root:
//   make test   (or: ./test_contain)

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include <cstdio>
#include <deque>
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

/* The stand-in for Kerbal (see the header). One part, isEva true -- the
   shape of every character vehicle. */
struct TestCrew : Vehicle {
    bool isEva() const override { return true; }
    bool isCrewAboard() const override { return true; }
};

/* A hand-built part. `defs` is a deque, not a vector: push_back on a vector
   reallocates and would dangle every Part::def handed out so far. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

static Part *mkPart(Ship &s, const char *name, double mass, double hx,
                    double hy, double hz, int crewCapacity, bool dockingPort) {
    Body *b = new Body;
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(hx, hy, hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name         = name;
    d.mass         = (float)mass;
    d.radius       = (float)hx;
    d.height       = (float)(2.0 * hz);
    d.crew_capacity = crewCapacity;
    d.docking_port  = dockingPort;
    d.synthesizeNodes();   // axial stack nodes (attachDown mates them)

    Part *p = new Part;
    p->body  = b;
    p->def   = &d;
    p->stage = 1;
    return p;
}

static void destroyVehicle(Vehicle *v) {
    /* onRails keeps ~Vehicle off RemoveBody: nothing here was ever added to
       a physics world (there is none in this test). */
    v->onRails = true;
    delete v;
}

int main() {
    /* Ship A: tank (root) + capsule + docking port stacked (attachDown
       stacks each part below the last, so the port is A's rear face).
       Crew C: one part. */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tank  = mkPart(A, "tank", 1000.0, 1.0, 1.0, 1.5, 0, false);
    Part *cap   = mkPart(A, "capsule", 100.0, 1.0, 1.0, 0.5, 1, false);
    Part *portA = mkPart(A, "portA", 50.0, 1.0, 1.0, 0.125, 0, true);
    A.v->setRoot(tank);
    A.v->attachDown(cap);
    A.v->attachDown(portA);
    A.v->controller = tank;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship C; C.v = new TestCrew; C.v->name = "C";
    Part *cPart = mkPart(C, "suit", 97.05, 0.2, 0.2, 0.375, 0, false);
    C.v->setRoot(cPart);
    C.v->controller = cPart;
    C.v->init();
    C.v->placeShip(glm::dvec3(10.0), glm::dmat3(1.0));

    /* --- owner wiring (the attach primitives) ---------------------------- */
    {
        printf("== owner: attached to exactly one vehicle ==\n");
        bool allA = true;
        for(Part *p : A.v->parts) { if(p->owner != A.v) { allA = false; } }
        CHECK_TRUE(allA, "every part of A has owner == A");
        CHECK_TRUE(cPart->owner == C.v, "the crew part has owner == C");
        CHECK_TRUE(tank->container == nullptr && cap->container == nullptr
                   && portA->container == nullptr,
                   "freshly built parts are contained nowhere");
        CHECK_TRUE(A.v->checkPartInvariants(), "A passes the invariant unwired");
        CHECK_TRUE(C.v->checkPartInvariants(), "C passes the invariant unwired");
    }

    /* --- a wired chain: contained in the capsule, both directions -------- */
    cPart->container = cap;
    cap->contents.push_back(cPart);
    {
        printf("== containment chain: edge + traversal + invariant ==\n");
        CHECK_TRUE(A.v->checkPartInvariants(), "A passes with the chain wired");
        CHECK_TRUE(C.v->checkPartInvariants(), "C passes with the chain wired");
        /* traversal: capsule -> contained part -> its vehicle -> back */
        CHECK_TRUE(cap->contents.size() == 1, "the capsule holds exactly one");
        CHECK_TRUE(cap->contents[0] == cPart, "contents[0] is the crew part");
        CHECK_TRUE(cap->contents[0]->container == cap, "back-reference holds");
        CHECK_TRUE(cap->contents[0]->owner == C.v, "owner names the crew vehicle");
        CHECK_TRUE(C.v->parts.size() == 1 && C.v->parts[0] == cPart,
                   "the crew vehicle still owns its own part");
    }

    /* --- the check must FAIL where the edge breaks ----------------------- */
    {
        printf("== invariant violations are caught ==\n");
        /* one-way, container side: claims a container that does not list it */
        cap->contents.erase(cap->contents.begin());
        CHECK_TRUE(!C.v->checkPartInvariants(),
                   "container set but not listed: C fails");
        /* one-way, contents side: listed, but points back at nothing */
        cap->contents.push_back(cPart);
        cPart->container = nullptr;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "listed but container null: A fails");
        cPart->container = cap;
        cap->contents.erase(cap->contents.begin());

        /* a non-character contained (the ship's own tank in the capsule) */
        tank->container = cap;
        cap->contents.push_back(tank);
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a non-kerbal part contained: A fails");
        tank->container = nullptr;
        cap->contents.erase(cap->contents.end() - 1);

        /* a character in a non-capsule (the tank, crew_capacity 0) */
        cPart->container = tank;
        CHECK_TRUE(!C.v->checkPartInvariants(),
                   "contained in a non-capsule: C fails");

        /* owner desync: the crew part claims a vehicle that is not its own */
        cPart->container = cap;
        cPart->owner = A.v;
        CHECK_TRUE(!C.v->checkPartInvariants(),
                   "crew part with the wrong owner: C fails");
        cPart->owner = C.v;
        cap->contents.push_back(cPart);   // back to a correctly wired chain

        CHECK_TRUE(A.v->checkPartInvariants() && C.v->checkPartInvariants(),
                   "restored chain passes again");
    }

    /* --- owner re-pointing across merge and split ------------------------ */
    {
        printf("== owner survives absorbShip and extractSubtreeAsShip ==\n");
        /* B: its own capsule + tank, docked into A below the capsule. The
           crew moves in with the ship's crew list is Kerbal's job; here the
           edge that must survive is the containment one, which needs no
           reindex at all (a Part* is stable). */
        Ship B; B.v = new Vehicle; B.v->name = "B";
        Part *bTank = mkPart(B, "bTank", 500.0, 1.0, 1.0, 1.0, 0, false);
        Part *bCap  = mkPart(B, "bCapsule", 100.0, 1.0, 1.0, 0.5, 1, false);
        B.v->setRoot(bCap);
        B.v->attachDown(bTank);
        B.v->controller = bCap;
        B.v->init();
        B.v->placeShip(glm::dvec3(0.0, 0.0, -5.0), glm::dmat3(1.0));

        /* the crew part rides in B's capsule BEFORE the dock (already wired) */
        cPart->container = bCap;
        cap->contents.clear();
        bCap->contents.push_back(cPart);

        A.v->absorbShip(B.v, portA);
        delete B.v;

        bool allA = true;
        for(Part *p : A.v->parts) { if(p->owner != A.v) { allA = false; } }
        CHECK_TRUE(allA, "after absorb, every part (B's included) owns to A");
        CHECK_TRUE(bCap->owner == A.v, "the crew's capsule re-pointed to A");
        CHECK_TRUE(cPart->container == bCap, "the crew's container is unchanged");
        CHECK_TRUE(A.v->checkPartInvariants(), "A passes after absorb");
        CHECK_TRUE(C.v->checkPartInvariants(), "C passes after absorb");

        /* split the docked side back off: the crew's capsule goes with it */
        Vehicle *nv = A.v->extractSubtreeAsShip(bCap, "B");
        CHECK_TRUE(nv != nullptr, "the split succeeds");
        if(nv != nullptr) {
            CHECK_TRUE(bCap->owner == nv, "the capsule re-pointed to the new ship");
            CHECK_TRUE(cPart->container == bCap,
                       "the crew's container still names the capsule");
            CHECK_TRUE(nv->checkPartInvariants(), "the split-off passes");
            CHECK_TRUE(C.v->checkPartInvariants(), "C passes after the split");
            CHECK_TRUE(A.v->checkPartInvariants(), "A passes after the split");
            nv->onRails = true;
            delete nv;
        }
    }

    /* teardown: delete the crew first, as ~Vehicle does for its own crew --
       the ordering the non-owning contents edge must tolerate. */
    destroyVehicle(C.v);
    destroyVehicle(A.v);

    if(g_failures == 0) {
        printf("test_contain: all %d checks passed\n", g_checks);
        return 0;
    }
    printf("test_contain: %d/%d FAILED\n", g_failures, g_checks);
    return 1;
}
