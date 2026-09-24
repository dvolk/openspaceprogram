// test_inventory: phase 4.3 + the phase-4 quality pass -- inventory items in
// containers, via the public transfer API (src/inventory.h).
//
// What is pinned:
//   - ADD: inventoryAdd wires the ownership edge (ownedContents), the
//     traversal edge (contents) and the back-reference (container), and
//     the item rides the container's vehicle (item->owner).
//   - GUARDS: an already-contained item cannot be added again (double add),
//     a non-container cannot hold an item, capacity is enforced, and a
//     containment cycle (X holds Y, then X into Y) is refused -- every
//     refusal leaves the item exactly where it was (no orphan).
//   - TRANSFER: re-parents and re-points the owner to the new container's
//     vehicle; a failed transfer (full / non-container / free item) moves
//     nothing.
//   - REMOVE: clears all three edges and the owner; a free item is a no-op.
//   - effectiveMass recurses through nested items.
//   - Vehicle::init() (rebuildCompound) and checkPartInvariants PASS with
//     items in a container -- the quality pass found the invariant rejected
//     them, which would have aborted load of any save holding inventory.
//   - checkPartInvariants FAILS where the edge is broken: an item listed in
//     a non-container's contents, and an item owned by a part with
//     inventory_capacity 0.
//   - OWNERSHIP: deleting the ship frees its container's items (and their
//     nested items) exactly once (~Part deletes ownedContents).
//   - absorbShip / extractSubtreeAsShip re-point every item in the moved
//     containers to the destination ship (the owner is the container's
//     vehicle; a stale one would dangle when the shell is deleted).
//   - inventoryDrain: the pocket draw drains the inventory subtree, DFS (a
//     tank nested in a pocket crate is found) and all-or-nothing (a draw the
//     pocket can't cover drains nothing).
//   - a containment cycle is refused WITHOUT orphaning the item (the cycle
//     is pre-checked, so a refused transfer moves nothing), and the cycle
//     guard sees the crew edge (contents holds crew too).
//
// Headless (no Game / physics world), like test_contain: ships are built
// with init() (runs rebuildCompound -- which asserts the containment
// invariant -- but not enterWorld).
//
// Runs from the repo root:
//   make test   (or: ./test_inventory)

#define BT_USE_DOUBLE_PRECISION true
#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>

#include "vehicle.h"   // Vehicle + Part (inline)
#include "inventory.h"

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
   reallocates and would dangle every Part::def handed out so far. `invCap`
   is inventory_capacity (> 0 -> a container); `tankHz` > 0 makes it a
   hydrazine tank (the def's capacity vector, seeded like the catalog does).
   A `Ship` with v == nullptr is just a def bag for standalone items. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

static Part *mkPart(Ship &s, const char *name, double mass, double hx,
                    double hy, double hz, int invCap, bool dockingPort,
                    float tankHz = 0.0f) {
    Body *b = new Body;
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(hx, hy, hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name             = name;
    d.mass             = (float)mass;
    d.radius           = (float)hx;
    d.height           = (float)(2.0 * hz);
    d.inventory_capacity = invCap;
    d.docking_port     = dockingPort;
    if(tankHz > 0.0f) {
        d.capacity.resize((int)ResourceType::Num, 0.0f);
        d.capacity[(int)ResourceType::Hydrazine] = tankHz;
    }
    d.synthesizeNodes();   // axial stack nodes (attachDown mates them)

    Part *p = new Part;
    p->body  = b;
    p->def   = &d;
    p->stage = 1;
    return p;
}

/* onRails keeps ~Vehicle off RemoveBody: nothing here was ever added to a
   physics world (there is none in this test). */
static void destroyVehicle(Vehicle *v) {
    v->onRails = true;
    delete v;
}

int main() {
    Ship S1; S1.v = new Vehicle; S1.v->name = "S1";
    Part *C  = mkPart(S1, "crate", 250.0, 1.0, 1.0, 1.0, 2, false);
    S1.v->setRoot(C);
    S1.v->controller = C;

    Ship I; I.v = nullptr;              // def bag for standalone items
    Part *T  = mkPart(I, "tankA", 88.99, 1.0, 1.0, 1.0, 0, false, 78.54f);
    Part *N  = mkPart(I, "nut",   10.0,  1.0, 1.0, 1.0, 0, false);

    /* --- add: the three edges + the owner, and the guards ---------------- */
    {
        printf("== inventoryAdd: edges, owner, guards ==\n");
        CHECK_TRUE(inventoryAdd(T, C), "the crate accepts the tank");
        CHECK_TRUE(C->ownedContents.size() == 1 && C->ownedContents[0] == T,
                   "ownership edge: T in C's ownedContents");
        CHECK_TRUE(C->contents.size() == 1 && C->contents[0] == T,
                   "traversal edge: T in C's contents");
        CHECK_TRUE(T->container == C, "back-reference: T->container is C");
        CHECK_TRUE(T->owner == S1.v, "the item rides the container's vehicle");
        CHECK_NEAR(C->effectiveMass(), 338.99, 1e-12,
                   "the crate carries the tank (250 + 88.99)");

        /* the guards, each of which must leave the item exactly where it was */
        CHECK_TRUE(!inventoryAdd(T, C), "double-add refused");
        CHECK_TRUE(T->container == C && C->ownedContents.size() == 1,
                   "refused double-add changes nothing");
        CHECK_TRUE(!inventoryAdd(N, T), "a non-container cannot hold an item");
        CHECK_TRUE(N->container == nullptr, "N is still free");

        CHECK_TRUE(inventoryAdd(N, C), "the second item fits (2/2)");
        Part *M = mkPart(I, "crateM", 5.0, 1.0, 1.0, 1.0, 5, false);
        CHECK_TRUE(!inventoryAdd(M, C), "at capacity: refused");
        CHECK_TRUE(M->container == nullptr, "refused add leaves M free");
        delete M;   // nobody will take it

        /* a cycle: X holds Y, then X into Y would make effectiveMass
           recurse forever and ~Part double-free */
        Part *X = mkPart(I, "xcrate", 50.0, 1.0, 1.0, 1.0, 10, false);
        Part *Y = mkPart(I, "ycrate", 50.0, 1.0, 1.0, 1.0, 10, false);
        CHECK_TRUE(inventoryAdd(Y, X), "Y into X");
        CHECK_TRUE(!inventoryAdd(X, Y), "X into Y refused (cycle)");
        CHECK_TRUE(X->container == nullptr, "X is still free after the refusal");
        CHECK_TRUE(!inventoryAdd(X, X), "self-add refused");
        delete X;   // frees Y (ownedContents) -- the ownership path in action
        (void)Y;
    }

    /* --- transfer: re-parent + owner re-pointing ------------------------- */
    {
        printf("== inventoryTransfer: re-parent, owner follows the container ==\n");
        Ship S2; S2.v = new Vehicle; S2.v->name = "S2";
        Part *B  = mkPart(S2, "crateB", 100.0, 1.0, 1.0, 1.0, 1, false);
        S2.v->setRoot(B);
        S2.v->controller = B;

        CHECK_TRUE(inventoryTransfer(T, B), "T moves C -> B");
        CHECK_TRUE(T->container == B, "back-reference re-pointed");
        CHECK_TRUE(T->owner == S2.v, "owner re-pointed to B's vehicle");
        CHECK_TRUE(C->ownedContents.size() == 1 && C->ownedContents[0] == N,
                   "C keeps only N");

        /* B is full (1/1): the failed transfer must not orphan N */
        CHECK_TRUE(!inventoryTransfer(N, B), "into a full container: refused");
        CHECK_TRUE(N->container == C, "N is still in C");
        CHECK_TRUE(!inventoryTransfer(N, T), "into a non-container: refused");
        CHECK_TRUE(N->container == C, "N is still in C");

        Part *F = mkPart(I, "free", 1.0, 1.0, 1.0, 1.0, 0, false);
        CHECK_TRUE(!inventoryTransfer(F, B), "a free item has no source");
        delete F;

        CHECK_TRUE(inventoryTransfer(T, C), "T moves back B -> C");
        CHECK_TRUE(T->owner == S1.v, "owner back to S1");
        CHECK_TRUE(B->ownedContents.empty() && B->contents.empty(), "B emptied");
        destroyVehicle(S2.v);   // B empty -- frees nothing but B
    }

    /* --- remove: all edges + the owner clear; a free item is a no-op ----- */
    {
        printf("== inventoryRemove: edges and owner cleared ==\n");
        inventoryRemove(N);
        CHECK_TRUE(N->container == nullptr, "back-reference cleared");
        CHECK_TRUE(N->owner == nullptr, "owner cleared");
        CHECK_TRUE(std::find(C->ownedContents.begin(), C->ownedContents.end(),
                             N) == C->ownedContents.end(),
                   "not in ownedContents");
        CHECK_TRUE(std::find(C->contents.begin(), C->contents.end(),
                             N) == C->contents.end(),
                   "not in contents");
        inventoryRemove(N);        // removing a free item: no-op, no crash
        delete N;
        CHECK_TRUE(C->ownedContents.size() == 1 && C->ownedContents[0] == T,
                   "C keeps only T");

        Part *R = mkPart(I, "removeMe", 7.0, 1.0, 1.0, 1.0, 0, false);
        CHECK_TRUE(inventoryAdd(R, C), "R into C (2/2 again)");
        inventoryRemove(R);
        CHECK_TRUE(R->container == nullptr && R->owner == nullptr,
                   "R fully detached");
        CHECK_TRUE(inventoryAdd(R, C), "R back into C");
        CHECK_TRUE(C->ownedContents.size() == 2, "C holds T and R at the end");
    }

    /* --- effectiveMass recurses through nested items --------------------- */
    {
        printf("== effectiveMass: nested items ==\n");
        Ship S5; S5.v = new Vehicle; S5.v->name = "S5";
        Part *E  = mkPart(S5, "nesthost", 100.0, 1.0, 1.0, 1.0, 2, false);
        S5.v->setRoot(E);
        S5.v->controller = E;

        Part *NC = mkPart(I, "nestcrate", 20.0, 1.0, 1.0, 1.0, 5, false);
        Part *NT = mkPart(I, "nesttank", 30.0, 1.0, 1.0, 1.0, 0, false, 50.0f);
        CHECK_TRUE(inventoryAdd(NT, NC), "tank into the sub-crate");
        CHECK_TRUE(inventoryAdd(NC, E), "sub-crate into the crate");
        CHECK_NEAR(NC->effectiveMass(), 50.0, 1e-12,
                   "the sub-crate carries its tank (20 + 30)");
        CHECK_NEAR(E->effectiveMass(), 150.0, 1e-12,
                   "the crate carries the sub-crate chain (100 + 50)");
        CHECK_TRUE(NT->owner == S5.v, "the nested item rides the ship");
        destroyVehicle(S5.v);   // frees E -> NC -> NT, one owner each
    }

    /* --- init() with items: the invariant must PASS (quality-pass bug) ---
       The pre-fix invariant rejected items in contents (they are not a
       character's part), so rebuildCompound -- asserted on every build,
       stage and burn refresh, i.e. on load of any save with inventory --
       would abort. A vehicle that inits with items in its crate is the
       regression pin. */
    {
        printf("== init() + checkPartInvariants pass with items ==\n");
        S1.v->init();
        S1.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));
        CHECK_TRUE(S1.v->checkPartInvariants(), "S1 passes with its items");
        CHECK_NEAR(S1.v->getMass(), 345.99, 1e-3,
                   "the ship's mass carries every item (250 + 88.99 + 7)");
        destroyVehicle(S1.v);   // frees C -> T, R
    }

    /* --- the invariant must FAIL where the edge breaks -------------------- */
    {
        printf("== invariant violations are caught ==\n");
        Ship S3; S3.v = new Vehicle; S3.v->name = "S3";
        Part *P  = mkPart(S3, "plain", 100.0, 1.0, 1.0, 1.0, 0, false);
        Part *K  = mkPart(S3, "kcrate", 60.0, 1.0, 1.0, 1.0, 1, false);
        S3.v->setRoot(P);
        S3.v->attachDown(K);
        S3.v->controller = P;
        S3.v->init();
        S3.v->placeShip(glm::dvec3(20.0), glm::dmat3(1.0));
        CHECK_TRUE(S3.v->checkPartInvariants(), "S3 passes unwired");

        Part *IT = mkPart(I, "stray", 5.0, 1.0, 1.0, 1.0, 0, false);
        /* (a) listed in a non-container's contents: not an owned item (not
           in ownedContents), not a character's part (no owner) -> fail */
        P->contents.push_back(IT);
        IT->container = P;
        CHECK_TRUE(!S3.v->checkPartInvariants(),
                   "item in a non-container's contents: fails");
        P->contents.pop_back();
        IT->container = nullptr;
        /* (b) owned by a part with inventory_capacity 0 -> fail */
        P->ownedContents.push_back(IT);
        P->contents.push_back(IT);
        IT->container = P;
        CHECK_TRUE(!S3.v->checkPartInvariants(),
                   "item owned by a non-container: fails");
        P->ownedContents.pop_back();
        P->contents.pop_back();
        IT->container = nullptr;
        /* (c) owned by a container but not listed for traversal: the mass
           would be freed in ~Part but silently absent from effectiveMass ->
           fail */
        K->ownedContents.push_back(IT);
        IT->container = K;
        CHECK_TRUE(!S3.v->checkPartInvariants(),
                   "owned but not listed: fails");
        K->ownedContents.pop_back();
        IT->container = nullptr;
        CHECK_TRUE(S3.v->checkPartInvariants(), "restored S3 passes");
        destroyVehicle(S3.v);
        delete IT;   // nobody owns it any more
    }

    /* --- absorb + split re-point the item owners --------------------------
       The item's owner is its container's vehicle. A dock (absorbShip) or
       a split (extractSubtreeAsShip) moves the container to a new ship; the
       items in its inventory must follow, or they dangle when the caller
       deletes the shell. */
    {
        printf("== item owners follow absorbShip / extractSubtreeAsShip ==\n");
        Ship DA; DA.v = new Vehicle; DA.v->name = "DA";
        Part *AC = mkPart(DA, "acrate", 250.0, 1.0, 1.0, 1.0, 2, false);
        Part *AP = mkPart(DA, "aport",  50.0,  1.0, 1.0, 0.125, 0, true);
        DA.v->setRoot(AC);
        DA.v->attachDown(AP);
        DA.v->controller = AC;

        Ship DB; DB.v = new Vehicle; DB.v->name = "DB";
        Part *BC = mkPart(DB, "bcrate", 120.0, 1.0, 1.0, 1.0, 2, false);
        DB.v->setRoot(BC);
        DB.v->controller = BC;

        Part *AIT = mkPart(I, "ait", 40.0, 1.0, 1.0, 1.0, 0, false, 60.0f);
        Part *BIT = mkPart(I, "bit", 40.0, 1.0, 1.0, 1.0, 0, false, 60.0f);
        CHECK_TRUE(inventoryAdd(AIT, AC), "AIT into AC");
        CHECK_TRUE(inventoryAdd(BIT, BC), "BIT into BC");
        CHECK_TRUE(BIT->owner == DB.v, "BIT rides DB");

        DA.v->init();
        DA.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));
        DB.v->init();
        DB.v->placeShip(glm::dvec3(0.0, 0.0, -6.0), glm::dmat3(1.0));

        DA.v->absorbShip(DB.v, AP);
        CHECK_TRUE(BC->owner == DA.v, "the container re-pointed to DA");
        CHECK_TRUE(BIT->owner == DA.v, "the item owner re-pointed to DA");
        CHECK_TRUE(AIT->owner == DA.v, "the crate's own item unchanged");
        CHECK_TRUE(DA.v->checkPartInvariants(), "DA passes after absorb");
        delete DB.v;   // empty shell -- must not touch the item a second time

        Vehicle *nv = DA.v->extractSubtreeAsShip(BC, "DB");
        CHECK_TRUE(nv != nullptr, "the split succeeds");
        if(nv != nullptr) {
            CHECK_TRUE(BC->owner == nv, "the container re-pointed to the split");
            CHECK_TRUE(BIT->owner == nv, "the item owner followed the split");
            CHECK_TRUE(AIT->owner == DA.v, "AC's item stays with DA");
            CHECK_TRUE(nv->checkPartInvariants(), "the split-off passes");
            CHECK_TRUE(DA.v->checkPartInvariants(), "DA passes after the split");
            destroyVehicle(nv);
        }
        destroyVehicle(DA.v);
    }

    /* --- inventoryDrain: the pocket draw (phase 4.5) --------------------
       The pocket tanks are not in the ship's fuel groups, so
       consumeResourceMass can't reach them. inventoryDrain drains the
       inventory subtree, DFS (a tank nested in a pocket crate is found --
       the quality-pass BUG: the old flat loop read only direct children's
       OWN fuel, so a nested tank was invisible and the kerbal could not
       thrust at all) and all-or-nothing (a draw the pocket can't cover
       thrusts nothing, so a partial draw never applies a full-force kick
       for less propellant -- the #3 inconsistency with the suit). */
    {
        printf("== inventoryDrain: DFS + all-or-nothing ==\n");
        const int res = (int)ResourceType::Hydrazine;

        // (flat) a tank directly in the pocket
        Ship S6; S6.v = new Vehicle; S6.v->name = "S6";
        Part *H = mkPart(S6, "host", 100.0, 1.0, 1.0, 1.0, 1, false);
        S6.v->setRoot(H);
        S6.v->controller = H;
        Part *FT = mkPart(I, "flat", 88.99, 1.0, 1.0, 1.0, 0, false, 78.54f);
        FT->resources.current[res] = 78.54f;   // seed the instance fuel (the def sets only capacity)
        CHECK_TRUE(inventoryAdd(FT, H), "flat tank into the pocket");
        const double ftMass0 = FT->effectiveMass();
        CHECK_TRUE(inventoryDrain(H, res, 5.0f), "a flat pocket tank covers the draw");
        CHECK_NEAR(FT->resources.current[res], 73.54f, 1e-5,
                   "the flat tank shed 5 kg of fuel");
        CHECK_NEAR(FT->effectiveMass(), ftMass0 - 5.0, 1e-9,
                   "the flat tank shed 5 kg of mass");
        destroyVehicle(S6.v);   // frees H -> FT

        // (nested) a tank inside a crate in the pocket
        Ship S6b; S6b.v = new Vehicle; S6b.v->name = "S6b";
        Part *H2 = mkPart(S6b, "host2", 100.0, 1.0, 1.0, 1.0, 1, false);
        S6b.v->setRoot(H2);
        S6b.v->controller = H2;
        Part *PC = mkPart(I, "pocketcrate", 20.0, 1.0, 1.0, 1.0, 1, false);
        Part *PT = mkPart(I, "pockettank", 50.0, 1.0, 1.0, 1.0, 0, false, 50.0f);
        PT->resources.current[res] = 50.0f;   // seed the instance fuel
        CHECK_TRUE(inventoryAdd(PT, PC), "tank into the pocket crate");
        CHECK_TRUE(inventoryAdd(PC, H2), "the pocket crate into the pocket");
        CHECK_NEAR(inventorySubtreeResource(H2, res), 50.0f, 1e-6,
                   "the pocket sees the nested tank's fuel");
        const double ptMass0 = PT->effectiveMass();
        CHECK_TRUE(inventoryDrain(H2, res, 7.0f), "the nested pocket tank covers the draw");
        CHECK_NEAR(PT->resources.current[res], 43.0f, 1e-6,
                   "the nested tank shed 7 kg of fuel");
        CHECK_NEAR(PT->effectiveMass(), ptMass0 - 7.0, 1e-9,
                   "the nested tank shed 7 kg of mass");
        destroyVehicle(S6b.v);   // frees H2 -> PC -> PT

        // (all-or-nothing) a 2 kg pocket cannot cover a 5 kg draw: it drains
        // nothing (the old flat loop drained 2 kg AND applied the full force)
        Part *PE = mkPart(I, "pockempty", 10.0, 1.0, 1.0, 1.0, 1, false);
        Part *PN = mkPart(I, "pinny", 30.0, 1.0, 1.0, 1.0, 0, false, 2.0f);
        PN->resources.current[res] = 2.0f;    // seed the instance fuel
        CHECK_TRUE(inventoryAdd(PN, PE), "a 2 kg tank into a lone crate");
        CHECK_TRUE(!inventoryDrain(PE, res, 5.0f),
                   "a 2 kg pocket cannot cover a 5 kg draw");
        CHECK_NEAR(PN->resources.current[res], 2.0f, 1e-6,
                   "the refused draw drained nothing");
        CHECK_NEAR(PN->effectiveMass(), 32.0, 1e-9, "the refused draw shed no mass");
        CHECK_TRUE(inventoryDrain(PE, res, 2.0f), "the same pocket covers a 2 kg draw");
        CHECK_NEAR(PN->resources.current[res], 0.0f, 1e-6, "the 2 kg draw empties the tank");
        delete PE;   // standalone: frees PN
    }

    /* --- transfer: a cycle is refused BEFORE the item is removed ---------
       The old order (remove, then addToContainer's cycle check) left the
       item orphaned when the cycle fired: no container, owner nulled, and
       its whole subtree leaked. The cycle is now pre-checked, so a refused
       transfer moves nothing. */
    {
        printf("== inventoryTransfer: a cycle is refused without orphaning ==\n");
        Ship S7; S7.v = new Vehicle; S7.v->name = "S7";
        Part *C = mkPart(S7, "outer", 250.0, 1.0, 1.0, 1.0, 3, false);
        S7.v->setRoot(C);
        S7.v->controller = C;
        Part *A = mkPart(I, "mid",   20.0, 1.0, 1.0, 1.0, 2, false);
        Part *B = mkPart(I, "inner", 20.0, 1.0, 1.0, 1.0, 1, false);
        CHECK_TRUE(inventoryAdd(A, C), "mid crate into the outer");
        CHECK_TRUE(inventoryAdd(B, A), "inner crate into the mid");
        // B is inside A: moving A into B would close a cycle
        CHECK_TRUE(!inventoryTransfer(A, B), "moving A into its own descendant B: refused");
        CHECK_TRUE(A->container == C, "A is still in C (not orphaned)");
        CHECK_TRUE(B->container == A, "B is still in A (not orphaned)");
        CHECK_TRUE(A->owner == S7.v, "A still rides the ship");
        destroyVehicle(S7.v);   // frees C -> A -> B
    }

    /* --- the cycle guard sees the crew edge ------------------------------
       contents holds crew too (non-owning), and a kerbal's suit is itself a
       container. inSubtree must walk contents (not just ownedContents), or
       a capsule added into its own crew kerbal closes a cycle the guard
       misses. Wired by hand (a headless test has no real Kerbal): KP is in
       C's contents but not its ownedContents -- exactly the crew edge. */
    {
        printf("== inSubtree sees the crew edge (cycle guard) ==\n");
        Part *CAP = mkPart(I, "capsule", 100.0, 1.0, 1.0, 1.0, 3, false);
        Part *KP  = mkPart(I, "kerbal",  90.0, 1.0, 1.0, 1.0, 3, false);
        CAP->contents.push_back(KP);   // the crew edge (non-owning)
        KP->container = CAP;
        CHECK_TRUE(!inventoryAdd(CAP, KP),
                   "adding the capsule into its crew kerbal: refused (cycle)");
        CHECK_TRUE(CAP->container == nullptr, "the capsule is still free (not orphaned)");
        delete CAP;   // CAP owns nothing (KP is crew, non-owning)
        delete KP;
    }

    if(g_failures == 0) {
        printf("test_inventory: all %d checks passed\n", g_checks);
        return 0;
    }
    printf("test_inventory: %d/%d FAILED\n", g_failures, g_checks);
    return 1;
}
