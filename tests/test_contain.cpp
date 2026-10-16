// test_contain: the containment edge (Part::owner / container / contents /
// seats, src/part.h) and its invariant check (Vehicle::checkPartInvariants,
// src/vehicle.cpp).
//
// What is pinned:
//   - OWNER: setRoot/attach wire every part's owner to its vehicle, and
//     absorbShip / extractSubtreeAsShip re-point it -- for the attached parts
//     AND for everything held in them. A Part* is stable through both, so
//     only the pointer moves, which is what lets a seat stay parked in its
//     capsule across a merge or a split with no reindex machinery.
//   - THE EDGE: container/contents back-reference each other for cargo, and
//     seats hold people. Only a kerbal-type part may be a seat -- and only in
//     a real capsule (crew_capacity > 0). A person is never cargo.
//   - A SEAT IS NOT A PART OF A VEHICLE: nothing in a parts list may be
//     held. A suit in both lists would be freed twice (~Vehicle's parts and
//     ~Part's seats), which is the failure this check exists to catch.
//
// A seat is a kerbal Part held by its capsule (part.h): the capsule owns the
// suit, and the suit owns its KerbalProfile. There is no Kerbal vehicle to
// model here -- minting one IS going EVA (Game::kerbalEVA) -- so unlike the
// pre-seat version of this test there is no TestCrew stand-in and no crew list.
//
// No physics world and no GL context, exactly like test_dock: ships are built
// with init() (runs rebuildCompound -- which now asserts the containment
// invariant -- but not enterWorld).
//
// Runs from the repo root:
//   make test   (or: ./test_contain)

#include "btcommon.h"

#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>

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

/* A hand-built part. `defs` is a deque, not a vector: push_back on a vector
   reallocates and would dangle every Part::def handed out so far. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

static Part *mkPart(Ship &s, const char *name, const char *type, double mass,
                    double hx, double hy, double hz, int crewCapacity,
                    int inventoryCapacity, bool dockingPort) {
    Body *b = new Body;
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(hx, hy, hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name         = name;
    d.type         = type;
    d.mass         = (float)mass;
    d.radius       = (float)hx;
    d.height       = (float)(2.0 * hz);
    d.crew_capacity = crewCapacity;
    d.inventory_capacity = inventoryCapacity;
    d.docking_port  = dockingPort;
    d.synthesizeNodes();   // axial stack nodes (attachDown mates them)

    Part *p = new Part;
    p->body  = b;
    p->def   = &d;
    p->stage = 1;
    return p;
}

// Park a suit in a capsule exactly as Ships::fill_seat / Game::kerbalBoard do:
// the seats edge, both back-pointers, and the owned identity.
static void fillSeat(Part *cap, Part *suit, Vehicle *ship, uint64_t profile) {
    suit->kerbal = std::make_unique<KerbalProfile>();
    suit->kerbal->id = profile;
    suit->kerbal->name = "Crew" + std::to_string(profile);
    cap->addSeat(suit);
    suit->owner = ship;
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
       The suit is a loose part: a seat occupant belongs to no vehicle's parts
       list, so it is built but never attached. */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tank  = mkPart(A, "tank", "fuel", 1000.0, 1.0, 1.0, 1.5, 0, 0, false);
    Part *cap   = mkPart(A, "capsule", "capsule", 100.0, 1.0, 1.0, 0.5, 1, 0, false);
    Part *portA = mkPart(A, "portA", "dock", 50.0, 1.0, 1.0, 0.125, 0, 0, true);
    A.v->setRoot(tank);
    A.v->attachDown(cap);
    A.v->attachDown(portA);
    A.v->controller = tank;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship S; S.v = nullptr;   // a suit is never a vehicle
    Part *suit = mkPart(S, "suit", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);

    /* --- owner wiring (the attach primitives) ---------------------------- */
    {
        printf("== owner: attached to exactly one vehicle ==\n");
        bool allA = true;
        for(Part *p : A.v->parts) { if(p->owner != A.v) { allA = false; } }
        CHECK_TRUE(allA, "every part of A has owner == A");
        CHECK_TRUE(suit->owner == nullptr, "a loose suit belongs to no vehicle yet");
        CHECK_TRUE(tank->container == nullptr && cap->container == nullptr
                   && portA->container == nullptr,
                   "freshly built parts are contained nowhere");
        CHECK_TRUE(!suit->isSeat(), "a suit with no container is not a seat");
        CHECK_TRUE(A.v->checkPartInvariants(), "A passes the invariant unwired");
    }

    /* --- a seat: edge + ownership + derived occupancy ------------------- */
    fillSeat(cap, suit, A.v, 7);
    {
        printf("== a seat: edge, ownership, occupancy ==\n");
        CHECK_TRUE(A.v->checkPartInvariants(), "A passes with the seat wired");
        /* the seat edge: capsule -> occupant -> back */
        CHECK_TRUE(cap->contents.empty(), "a seat is NOT cargo (contents stays empty)");
        CHECK_TRUE(cap->seats.size() == 1, "the capsule holds exactly one seat");
        CHECK_TRUE(cap->seats[0] == suit, "seats[0] is the suit");
        CHECK_TRUE(cap->seats[0]->container == cap, "back-reference holds");
        CHECK_TRUE(suit->owner == A.v, "owner names the vehicle the capsule rides");
        CHECK_TRUE(suit->isSeat(), "the occupant reads as a seat");
        CHECK_TRUE(suit->kerbal != nullptr && suit->kerbal->id == 7,
                   "the suit owns who sits in it");
        CHECK_TRUE(cap->seatCount() == 1, "seatCount: one seat taken");
        CHECK_TRUE(A.v->parts.size() == 3, "the seat is NOT in the parts list");
    }

    /* --- the check must FAIL where the edge breaks ----------------------- */
    {
        printf("== invariant violations are caught ==\n");
        /* listed in seats, but the back-reference points elsewhere */
        suit->container = tank;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "listed but container points elsewhere: A fails");
        suit->container = cap;

        /* listed in seats, but points back at nothing */
        suit->container = nullptr;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "listed but container null: A fails");
        suit->container = cap;

        /* a non-kerbal part parked in a seat (swap the real occupant out so
           the type check fires before over-occupancy) */
        Part *loose = mkPart(A, "looseTank", "fuel", 10.0, 0.5, 0.5, 0.5, 0, 0, false);
        cap->removeSeat(suit);
        cap->addSeat(loose);
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a non-kerbal in a seat: A fails");
        cap->removeSeat(loose);
        cap->addSeat(suit);
        suit->owner = A.v;
        loose->owner = nullptr;
        delete loose;

        /* a kerbal part parked as cargo (the suit's pockets) */
        Part *stow = mkPart(S, "stowaway", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        suit->contents.push_back(stow);
        stow->container = suit;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a kerbal in inventory: A fails");
        suit->contents.pop_back();
        stow->container = nullptr;
        delete stow;

        /* a kerbal part parked in a non-capsule (the tank, crew_capacity 0).
           addSeat refuses a non-capsule (no free slot), so wire it by hand --
           the invariant is the backstop for a hand-built tree. */
        cap->removeSeat(suit);
        tank->seats.push_back(suit);
        suit->container = tank;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a seat occupant in a non-capsule: A fails");
        tank->removeSeat(suit);
        cap->addSeat(suit);

        /* the double-free shape: a suit that is BOTH attached to a vehicle and
           held. ~Vehicle's parts and ~Part's seats would each free it, so the
           check refuses the state outright. */
        A.v->parts.push_back(suit);
        suit->owner = A.v;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a part both attached and held: A fails");
        A.v->parts.pop_back();

        /* over-occupancy: two people in a one-seat capsule. The live board and
           the loader both refuse it (addSeat finds no free slot), so the check
           is the backstop for a hand-wired tree -- effectiveMass and the save
           walk would carry the extra person without complaint. */
        Part *second = mkPart(S, "suit2", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        second->kerbal = std::make_unique<KerbalProfile>();
        second->kerbal->id = 8;
        second->kerbal->name = "Crew8";
        cap->seats.push_back(second);   // past the free-slot guard
        second->container = cap;
        second->owner = A.v;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "two occupants in a one-seat capsule: A fails");
        cap->removeSeat(second);
        second->owner = nullptr;

        /* depth: a kerbal-typed part parked in the SUIT's pockets as a seat.
           A walk that stopped at depth 1 saw nothing wrong (a suit IS a
           container), but the save writer then drops it as a seat, so the
           person vanishes on the next save. Hand-wired past addSeat. */
        Part *pocket = mkPart(S, "suit3", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        suit->seats.push_back(pocket);
        pocket->container = suit;
        CHECK_TRUE(!A.v->checkPartInvariants(),
                   "a kerbal seat inside a suit's pocket: A fails");
        suit->removeSeat(pocket);
        pocket->owner = nullptr;
        delete second;
        delete pocket;

        CHECK_TRUE(A.v->checkPartInvariants(), "restored seat passes again");
    }

    /* --- the seat follows its capsule across merge and split ------------- */
    /* The board->dock->EVA seam: a kerbal parked in a ship that then docks
       (absorbShip) rides with its capsule, and when the docked side splits back
       off (extractSubtreeAsShip) it follows the capsule to the split-off ship.
       The capsule is a Part* (stable through both), so there is nothing to
       reindex -- only the occupant's `owner` has to move, and the
       contents/seats walk in absorbShip / extractSubtreeAsShip does that. */
    {
        printf("== the seat follows its capsule across absorb + split ==\n");
        /* P: capsule + docking port (the survivor). Q: capsule + tank (the
           docked side carrying the seat). */
        Ship P; P.v = new Vehicle; P.v->name = "P";
        Part *pCap  = mkPart(P, "pCapsule", "capsule", 100.0, 1.0, 1.0, 0.5, 2, 0, false);
        Part *portP = mkPart(P, "portP", "dock", 50.0, 1.0, 1.0, 0.125, 0, 0, true);
        P.v->setRoot(pCap);
        P.v->attachDown(portP);
        P.v->controller = pCap;
        P.v->init();
        P.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

        Ship Q; Q.v = new Vehicle; Q.v->name = "Q";
        Part *qCap  = mkPart(Q, "qCapsule", "capsule", 100.0, 1.0, 1.0, 0.5, 2, 0, false);
        Part *qTank = mkPart(Q, "qTank", "fuel", 500.0, 1.0, 1.0, 1.0, 0, 0, false);
        Q.v->setRoot(qCap);
        Q.v->attachDown(qTank);
        Q.v->controller = qCap;
        Q.v->init();
        Q.v->placeShip(glm::dvec3(0.0, 0.0, -5.0), glm::dmat3(1.0));

        Ship T; T.v = nullptr;
        Part *tSuit = mkPart(T, "tSuit", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        fillSeat(qCap, tSuit, Q.v, 11);
        CHECK_TRUE(qCap->seatCount() == 1, "Q starts with one seat taken");

        /* dock Q into P: the seat rides with Q's capsule */
        P.v->absorbShip(Q.v, portP);
        delete Q.v;
        CHECK_TRUE(qCap->owner == P.v, "the capsule re-pointed to P");
        CHECK_TRUE(tSuit->owner == P.v, "the seat's owner re-pointed to P");
        CHECK_TRUE(tSuit->container == qCap, "its capsule still names Q's capsule");
        CHECK_TRUE(qCap->seatCount() == 1, "P's copy of Q still holds the seat");
        CHECK_TRUE(P.v->checkPartInvariants(), "P passes after absorb");

        /* split the docked side (Q's capsule + tank) back off: the seat must
           follow its capsule to the split-off ship */
        Vehicle *nv = P.v->extractSubtreeAsShip(qCap, "Q");
        CHECK_TRUE(nv != nullptr, "the split succeeds");
        if(nv != nullptr) {
            CHECK_TRUE(qCap->owner == nv, "the capsule re-pointed to the split-off");
            CHECK_TRUE(tSuit->owner == nv, "the seat's owner followed its capsule");
            CHECK_TRUE(tSuit->container == qCap,
                       "the seat's container still names the capsule");
            CHECK_TRUE(nv->checkPartInvariants(), "the split-off passes");
            CHECK_TRUE(P.v->checkPartInvariants(), "P passes after the split");
            nv->onRails = true;
            delete nv;   // frees qCap, which frees the seat
        }
        destroyVehicle(P.v);
    }

    /* --- delete a ship with a seat aboard (the ownership proof) ---------- */
    /* The seat is in sCap->seats (ownership). ~Vehicle deletes parts, and
       ~Part deletes seats, so the suit is freed exactly once -- a clean exit
       under ASan (tmp/test_contain_asan) is the proof. A seat that were ALSO
       in the parts list would be freed twice, which the check above refuses. */
    {
        printf("== delete ship with a seat aboard (owned seats) ==\n");
        Ship SH; SH.v = new Vehicle; SH.v->name = "S";
        Part *sTank = mkPart(SH, "sTank", "fuel", 500.0, 1.0, 1.0, 1.0, 0, 0, false);
        Part *sCap  = mkPart(SH, "sCapsule", "capsule", 100.0, 1.0, 1.0, 0.5, 1, 0, false);
        SH.v->setRoot(sCap);
        SH.v->attachDown(sTank);
        SH.v->controller = sCap;
        SH.v->init();
        SH.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

        Ship K; K.v = nullptr;
        Part *kSuit = mkPart(K, "kSuit", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        fillSeat(sCap, kSuit, SH.v, 0);
        sCap->seats[0]->kerbal->name = "Anonymous";
        CHECK_TRUE(sCap->seatCount() == 1, "S has one seat taken");
        CHECK_TRUE(SH.v->checkPartInvariants(), "S passes with the seat wired");
        destroyVehicle(SH.v);   // ~Part(sCap) frees the suit
    }

    /* --- effectiveMass: a part carries what is parked inside it ---------- */
    /* The derived value the compound reads (rebuildCompound): a part's mass plus
       the effectiveMass of every contained part, recursively. A capsule with one
       kerbal is body + suit; a part with nothing in it is just its body mass.
       This is the ONLY thing putting a seated kerbal into a ship's mass, COM,
       inertia and delta-v -- its Body is never in the world. */
    {
        printf("== effectiveMass: a part carries what is parked inside it ==\n");
        Ship M; M.v = new Vehicle; M.v->name = "M";
        Part *mTank = mkPart(M, "mTank", "fuel", 500.0, 1.0, 1.0, 1.0, 0, 0, false);
        Part *mCap  = mkPart(M, "mCapsule", "capsule", 100.0, 1.0, 1.0, 0.5, 1, 0, false);
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
        CHECK_NEAR(M.v->hull->mass, 600.0, 1e-9,
                   "the compound mass is the parts, no seat yet");

        Ship N; N.v = nullptr;
        Part *nSuit = mkPart(N, "nSuit", "kerbal", 97.05, 0.2, 0.2, 0.375, 0, 3, false);
        fillSeat(mCap, nSuit, M.v, 0);

        CHECK_NEAR(mCap->effectiveMass(), 197.05, 1e-12,
                   "effectiveMass: a capsule carries its kerbal (100 + 97.05)");
        CHECK_NEAR(nSuit->effectiveMass(), 97.05, 1e-12,
                   "effectiveMass: the contained kerbal is its own body mass");
        CHECK_NEAR(mCap->effectiveMass() + mTank->effectiveMass(), 697.05, 1e-12,
                   "effectiveMass: the ship's parts sum to ship + kerbal");
        /* The compound only re-reads effectiveMass on a rebuild, which is why
           every live seat change calls rebuildCompound (fill_seat, kerbalBoard,
           kerbalEVA). A load does not: buildSeatParts parks the seats before
           Vehicle::finalize builds the compound the first time. */
        M.v->rebuildCompound();
        CHECK_NEAR(M.v->hull->mass, 697.05, 1e-9,
                   "rebuildCompound: the seat rides the capsule's mass");
        destroyVehicle(M.v);   // ~Part(mCap) frees the suit
    }

    /* teardown: the loose suit is owned by A's capsule, so deleting A frees it;
       delete A first and the suit must not be touched again. */
    destroyVehicle(A.v);

    if(g_failures == 0) {
        printf("test_contain: all %d checks passed\n", g_checks);
        return 0;
    }
    printf("test_contain: %d/%d FAILED\n", g_failures, g_checks);
    return 1;
}
