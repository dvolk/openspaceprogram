// test_inventory: phase 4.3 -- transfer between containers.
// Runs from the repo root:
//   make test   (or: ./test_inventory)
//
// Headless (no Game / Bullet): builds Part + PartDef directly and exercises
// the inventoryTransfer / inventoryRemove / ownership logic.

#include "part.h"
#include "inventory.h"

#include <cstdio>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

int main() {
    // --- build two containers + one item ----------------------------------
    // container A: capacity 2
    PartDef defA;
    defA.name = "crateA";
    defA.mass = 100.0f;
    defA.inventory_capacity = 2;
    Body *bodyA = new Body;
    bodyA->mass = 100.0;
    Part *A = new Part;
    A->body = bodyA;
    A->def = &defA;

    // container B: capacity 1
    PartDef defB;
    defB.name = "crateB";
    defB.mass = 50.0f;
    defB.inventory_capacity = 1;
    Body *bodyB = new Body;
    bodyB->mass = 50.0;
    Part *B = new Part;
    B->body = bodyB;
    B->def = &defB;

    // item: a small part (no container, no crew)
    PartDef defItem;
    defItem.name = "item1";
    defItem.mass = 10.0f;
    Body *bodyItem = new Body;
    bodyItem->mass = 10.0;
    Part *item = new Part;
    item->body = bodyItem;
    item->def = &defItem;

    // --- add item to A ------------------------------------------------------
    // manually set up the containment (simulating a pickup into A)
    A->ownedContents.push_back(item);
    A->contents.push_back(item);
    item->container = A;

    CHECK(A->ownedContents.size() == 1);
    CHECK(A->contents.size() == 1);
    CHECK(item->container == A);

    // --- transfer item A -> B -----------------------------------------------
    bool ok = inventoryTransfer(item, B);
    CHECK(ok);
    CHECK(A->ownedContents.empty());
    CHECK(A->contents.empty());
    CHECK(B->ownedContents.size() == 1);
    CHECK(B->ownedContents[0] == item);
    CHECK(B->contents.size() == 1);
    CHECK(item->container == B);

    // --- capacity refusal: B is full (capacity 1, has 1 item) ---------------
    PartDef defItem2;
    defItem2.name = "item2";
    defItem2.mass = 5.0f;
    Body *bodyItem2 = new Body;
    bodyItem2->mass = 5.0;
    Part *item2 = new Part;
    item2->body = bodyItem2;
    item2->def = &defItem2;

    // try to add item2 to B (full) -- should fail
    B->ownedContents.push_back(item2);
    B->contents.push_back(item2);
    item2->container = B;
    // now B has 2 items, but capacity is 1 -- this is an invalid state.
    // Let's reset and test capacity properly:
    B->ownedContents.clear();
    B->contents.clear();
    item2->container = nullptr;

    // re-add item to B (capacity 1)
    B->ownedContents.push_back(item);
    B->contents.push_back(item);
    item->container = B;

    // now try to transfer item2 into B (full)
    // first, put item2 in A
    A->ownedContents.push_back(item2);
    A->contents.push_back(item2);
    item2->container = A;

    bool full = inventoryTransfer(item2, B);
    CHECK(!full);   // B is full (capacity 1, has item)
    CHECK(item2->container == A);   // still in A

    // --- transfer to a non-container (should fail) --------------------------
    PartDef defNonCont;
    defNonCont.name = "plain";
    defNonCont.mass = 1.0f;
    Body *bodyNC = new Body;
    bodyNC->mass = 1.0;
    Part *nonCont = new Part;
    nonCont->body = bodyNC;
    nonCont->def = &defNonCont;

    bool badDest = inventoryTransfer(item2, nonCont);
    CHECK(!badDest);   // nonCont has inventory_capacity 0

    // --- inventoryRemove ----------------------------------------------------
    inventoryRemove(item);
    CHECK(B->ownedContents.empty());
    CHECK(B->contents.empty());
    CHECK(item->container == nullptr);

    // --- ownership: deleting the container frees the item -------------------
    // item2 is still in A (the failed transfers above left it there).
    // Delete A -- ~Part must free item2 (ownedContents) without a double free.
    CHECK(item2->container == A);
    CHECK(A->ownedContents.size() == 1);
    delete A;   // ~Part deletes ownedContents (item2) + bodyA
    // if we got here without a crash, ownership works

    // clean up the rest
    delete B;
    delete nonCont;
    delete item;

    if(failures == 0) {
        printf("test_inventory: all checks passed\n");
        return 0;
    }
    printf("test_inventory: %d FAILURES\n", failures);
    return 1;
}
