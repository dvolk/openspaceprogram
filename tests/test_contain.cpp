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

#include <cmath>
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

/* The stand-in for Kerbal (see the header). One part, isEva true -- the
   shape of every character vehicle. `cap` is what Kerbal::aboardPart is:
   the capsule it sits in (set by the test the way board/EVA would). */
struct TestCrew : Vehicle {
    Part *cap = nullptr;   // the capsule it is parked in; nullptr = free
    bool isEva() const override { return true; }
    bool isCrewAboard() const override { return (cap != nullptr); }
    Part *capsulePart() const override { return cap; }
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

    /* --- the crew list follows its capsule across merge and split -------- */
    /* The board->dock->EVA seam step 2.2 calls out: a kerbal aboard a ship
       that then docks (absorbShip) moves with the ship's crew list, and when
       the docked side splits back off (extractSubtreeAsShip) the kerbal
       follows its capsule to the split-off ship. The capsule is a Part*
       (stable through both), so the crew block reads the side straight off
       capsulePart()->owner -- no reindex. A TestCrew stand-in (capsulePart()
       overridden) plays the Kerbal; the real one is unreachable from a
       headless link (eva.cpp drags in game.h). */
    {
        printf("== crew follows its capsule across absorb + split ==\n");
        /* P: capsule + docking port (the survivor). Q: capsule + tank (the
           docked side carrying the crew). */
        Ship P; P.v = new Vehicle; P.v->name = "P";
        Part *pCap  = mkPart(P, "pCapsule", 100.0, 1.0, 1.0, 0.5, 2, false);
        Part *portP = mkPart(P, "portP", 50.0, 1.0, 1.0, 0.125, 0, true);
        P.v->setRoot(pCap);
        P.v->attachDown(portP);
        P.v->controller = pCap;
        P.v->init();
        P.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

        Ship Q; Q.v = new Vehicle; Q.v->name = "Q";
        Part *qCap  = mkPart(Q, "qCapsule", 100.0, 1.0, 1.0, 0.5, 2, false);
        Part *qTank = mkPart(Q, "qTank", 500.0, 1.0, 1.0, 1.0, 0, false);
        Q.v->setRoot(qCap);
        Q.v->attachDown(qTank);
        Q.v->controller = qCap;
        Q.v->init();
        Q.v->placeShip(glm::dvec3(0.0, 0.0, -5.0), glm::dmat3(1.0));

        /* the kerbal: parked in Q's capsule, on Q's crew list (as board does) */
        Ship T; T.v = new TestCrew; T.v->name = "T";
        Part *tSuit = mkPart(T, "suit", 97.05, 0.2, 0.2, 0.375, 0, false);
        T.v->setRoot(tSuit);
        T.v->controller = tSuit;
        T.v->init();
        T.v->placeShip(glm::dvec3(20.0), glm::dmat3(1.0));
        static_cast<TestCrew *>(T.v)->cap = qCap;
        Q.v->crew.push_back(T.v);

        CHECK_TRUE(Q.v->crew.size() == 1, "Q starts with the kerbal aboard");

        /* dock Q into P: the kerbal moves with Q's crew list */
        P.v->absorbShip(Q.v, portP);
        delete Q.v;
        CHECK_TRUE(P.v->crew.size() == 1, "after absorb, the kerbal is on P's crew");
        CHECK_TRUE(P.v->crew[0] == T.v, "it is the same kerbal");
        CHECK_TRUE(static_cast<TestCrew *>(T.v)->capsulePart() == qCap,
                   "its capsule still names Q's capsule");
        CHECK_TRUE(qCap->owner == P.v, "the capsule re-pointed to P");

        /* split the docked side (Q's capsule + tank) back off: the kerbal
           must follow its capsule to the split-off ship */
        Vehicle *nv = P.v->extractSubtreeAsShip(qCap, "Q");
        CHECK_TRUE(nv != nullptr, "the split succeeds");
        if(nv != nullptr) {
            CHECK_TRUE(nv->crew.size() == 1,
                       "the kerbal followed its capsule to the split-off");
            CHECK_TRUE(nv->crew[0] == T.v, "it is the same kerbal");
            CHECK_TRUE(P.v->crew.empty(), "P no longer holds the kerbal");
            CHECK_TRUE(qCap->owner == nv, "the capsule re-pointed to the split-off");
            CHECK_TRUE(nv->checkPartInvariants(), "the split-off passes the invariant");
            CHECK_TRUE(P.v->checkPartInvariants(), "P passes the invariant");
            nv->onRails = true;
            delete nv;   // frees the kerbal (it is in nv->crew now)
        }
        destroyVehicle(P.v);   // frees P's parts; the kerbal is already freed
    }

    /* --- delete a ship with crew aboard, edge wired (the 2.1 double-free) -- */
    /* The exact scenario 2.1 warns about, now that 2.4 wires the edge: the
       kerbal is in S->crew (sole owner) AND its root part is in cap->contents
       (non-owning back-reference). ~Vehicle deletes crew first (vehicle.cpp:
       1337-1338) then parts (:1342); if contents were owning, ~Part(cap) would
       free the already-freed kerbal root part a second time -- deterministic,
       because the crew-first ordering always precedes it. The non-owning
       contents only frees its pointer array on teardown, so a clean exit under
       ASan (tmp/test_contain_asan) is the proof. */
    {
        printf("== delete ship with crew aboard (edge wired, non-owning) ==\n");
        Ship S; S.v = new Vehicle; S.v->name = "S";
        Part *sTank = mkPart(S, "sTank", 500.0, 1.0, 1.0, 1.0, 0, false);
        Part *sCap  = mkPart(S, "sCapsule", 100.0, 1.0, 1.0, 0.5, 1, false);
        S.v->setRoot(sCap);
        S.v->attachDown(sTank);
        S.v->controller = sCap;
        S.v->init();
        S.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

        Ship K; K.v = new TestCrew; K.v->name = "K";
        Part *kSuit = mkPart(K, "kSuit", 97.05, 0.2, 0.2, 0.375, 0, false);
        K.v->setRoot(kSuit);
        K.v->controller = kSuit;
        K.v->init();
        K.v->placeShip(glm::dvec3(30.0), glm::dmat3(1.0));
        static_cast<TestCrew *>(K.v)->cap = sCap;

        /* board, exactly as kerbalBoard does: ownership + edge, both ways */
        S.v->crew.push_back(K.v);
        sCap->contents.push_back(kSuit);
        kSuit->container = sCap;

        CHECK_TRUE(S.v->checkPartInvariants(), "S passes with the crew wired");
        CHECK_TRUE(K.v->checkPartInvariants(), "K passes with the crew wired");

        /* the double-free check: crew freed before parts; the non-owning
           contents must not free the (now dangling) suit a second time. */
        destroyVehicle(S.v);   // deletes K (in S.v->crew) then S's parts
    }

    /* --- effectiveMass: a part carries what is parked inside it (3.1) --- */
    /* The derived value phase 3 wires into the compound (3.2): a part's mass
       plus the effectiveMass of every contained part, recursively. A capsule
       with one kerbal is body + suit; a part with nothing in it is just its
       body mass. */
    {
        printf("== effectiveMass: a part carries what is parked inside it ==\n");
        Ship M; M.v = new Vehicle; M.v->name = "M";
        Part *mTank = mkPart(M, "mTank", 500.0, 1.0, 1.0, 1.0, 0, false);
        Part *mCap  = mkPart(M, "mCapsule", 100.0, 1.0, 1.0, 0.5, 1, false);
        M.v->setRoot(mCap);
        M.v->attachDown(mTank);
        M.v->controller = mCap;
        M.v->init();
        M.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

        /* empty: effectiveMass is just the body mass */
        CHECK_NEAR(mTank->effectiveMass(), 500.0, 1e-12,
                   "effectiveMass: an empty part is its body mass");
        CHECK_NEAR(mCap->effectiveMass(), 100.0, 1e-12,
                   "effectiveMass: an empty capsule is its body mass");

        Ship N; N.v = new TestCrew; N.v->name = "N";
        Part *nSuit = mkPart(N, "nSuit", 97.05, 0.2, 0.2, 0.375, 0, false);
        N.v->setRoot(nSuit);
        N.v->controller = nSuit;
        N.v->init();
        N.v->placeShip(glm::dvec3(40.0), glm::dmat3(1.0));

        /* park the kerbal in the capsule (the containment edge, both ways) */
        static_cast<TestCrew *>(N.v)->cap = mCap;
        M.v->crew.push_back(N.v);
        mCap->contents.push_back(nSuit);
        nSuit->container = mCap;

        CHECK_NEAR(mCap->effectiveMass(), 197.05, 1e-12,
                   "effectiveMass: a capsule carries its kerbal (100 + 97.05)");
        CHECK_NEAR(nSuit->effectiveMass(), 97.05, 1e-12,
                   "effectiveMass: the contained kerbal is its own body mass");
        CHECK_NEAR(M.v->parts[0]->effectiveMass() + mTank->effectiveMass(),
                   697.05, 1e-12,
                   "effectiveMass: the ship's parts sum to ship + kerbal");
        /* M.v owns N.v (it is in M.v->crew): deleting the ship deletes the
           crew via ~Vehicle -- do NOT delete N.v separately (double free). */
        destroyVehicle(M.v);
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
