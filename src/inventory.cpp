// inventory.cpp -- phase 4.3: transfer inventory items between containers.

#include "inventory.h"
#include "part.h"

#include <algorithm>

void inventorySetOwner(Part *item, Vehicle *owner) {
    if(item == nullptr) { return; }
    item->owner = owner;
    for(Part *c : item->ownedContents) { inventorySetOwner(c, owner); }
}

static bool removeFromContainer(Part *item) {
    Part *src = item->container;
    if(src == nullptr) { return false; }
    // pop from ownedContents (ownership)
    auto &oc = src->ownedContents;
    auto it = std::find(oc.begin(), oc.end(), item);
    if(it != oc.end()) {
        oc.erase(it);
    }
    // pop from contents (traversal)
    auto &c = src->contents;
    auto it2 = std::find(c.begin(), c.end(), item);
    if(it2 != c.end()) {
        c.erase(it2);
    }
    item->container = nullptr;
    // the item -- and anything in its own inventory (a crate in a crate) --
    // no longer rides the container's vehicle (dropItem re-points the
    // subtree to its own 1-part ship; addToContainer to the new container's
    // owner)
    inventorySetOwner(item, nullptr);
    return true;
}

/* Is `target` `root` itself or anywhere inside its containment subtree?
   Adding root into target would close a containment cycle: effectiveMass()
   walks contents and would recurse forever, and ~Part would double-free.
   Walk `contents` (the list effectiveMass actually walks), not just
   ownedContents -- contents also holds crew, and a kerbal's suit is itself
   a container, so a cycle can run through that edge (a capsule with a
   crewed kerbal, then the capsule added into the kerbal's suit). */
static bool inSubtree(Part *root, Part *target) {
    for(Part *c : root->contents) {
        if(c == target) { return true; }
        if(inSubtree(c, target)) { return true; }
    }
    return false;
}

float inventorySubtreeResource(Part *root, int res) {
    if(root == nullptr) { return 0.0f; }
    float total = 0.0f;
    for(Part *c : root->ownedContents) {
        total += c->resources.current[res];
        total += inventorySubtreeResource(c, res);
    }
    return total;
}

/* Drain up to `amt` from root's subtree, DFS, returning how much was taken. */
static float drainSubtree(Part *root, int res, float amt) {
    if(root == nullptr || amt <= 0.0f) { return 0.0f; }
    float drained = 0.0f;
    for(Part *c : root->ownedContents) {
        if(drained >= amt) { break; }
        float have = c->resources.current[res];
        if(have > 0.0f) {
            const float take = (have < amt - drained) ? have : (amt - drained);
            c->resources.current[res] = have - take;
            /* body->mass is the DRY structure (fuel rides effectiveMass via
               resources.current), so a drain only decrements the contents. */
            drained += take;
        }
        if(drained < amt) { drained += drainSubtree(c, res, amt - drained); }
    }
    return drained;
}

bool inventoryDrain(Part *root, int res, float amt) {
    if(root == nullptr || amt <= 0.0f) { return false; }
    // all-or-nothing like the suit: a draw the subtree can't cover thrusts
    // nothing (so a partial draw never gets a full-force kick)
    if(inventorySubtreeResource(root, res) < amt) { return false; }
    drainSubtree(root, res, amt);
    return true;
}

static bool addToContainer(Part *item, Part *dest) {
    if(dest == nullptr || item == nullptr) { return false; }
    // an already-contained item must be removed first (inventoryTransfer
    // does that) -- a double add would list it twice and ~Part would free it
    // twice
    if(item->container != nullptr) { return false; }
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    // capacity check: count owned items (not crew -- they are in contents
    // but not in ownedContents)
    if((int)dest->ownedContents.size() >= dest->def->inventory_capacity) {
        return false;
    }
    if(dest == item || inSubtree(item, dest)) { return false; }
    dest->ownedContents.push_back(item);
    dest->contents.push_back(item);
    item->container = dest;
    // the item -- and its own inventory subtree -- rides the container's
    // vehicle (the container's owner may be null -- a part held by no
    // vehicle -- in which case so is the item)
    inventorySetOwner(item, dest->owner);
    return true;
}

bool inventoryTransfer(Part *item, Part *dest) {
    if(item == nullptr || dest == nullptr) { return false; }
    if(item == dest) { return false; }
    if(item->container == nullptr) { return false; }
    // check dest capacity AND the cycle BEFORE removing from source (a failed
    // transfer must not orphan the item): dest must be a valid, non-full
    // container, and it must not sit inside item's own subtree -- that would
    // close a containment cycle, and addToContainer would then refuse it
    // AFTER removeFromContainer already detached item, leaving it with no
    // container and its whole subtree leaking.
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    if((int)dest->ownedContents.size() >= dest->def->inventory_capacity) { return false; }
    if(inSubtree(item, dest)) { return false; }
    removeFromContainer(item);
    return addToContainer(item, dest);
}

bool inventoryAdd(Part *item, Part *dest) {
    return addToContainer(item, dest);
}

void inventoryRemove(Part *item) {
    if(item == nullptr) { return; }
    removeFromContainer(item);
}
