#include "inventory.h"
#include "part.h"

#include <algorithm>

void inventorySetOwner(Part *item, Vehicle *owner) {
    if(item == nullptr) { return; }
    item->owner = owner;
    for(Part *c : item->contents) { inventorySetOwner(c, owner); }
    for(Part *suit : item->seats) { inventorySetOwner(suit, owner); }
}

static bool removeFromContainer(Part *item) {
    Part *src = item->container;
    if(src == nullptr) { return false; }
    auto &c = src->contents;
    auto it = std::find(c.begin(), c.end(), item);
    if(it != c.end()) {
        c.erase(it);
    }
    item->container = nullptr;
    // the item no longer rides the container's vehicle
    inventorySetOwner(item, nullptr);
    return true;
}

// Walks contents AND seats: a cycle can run through a seated kerbal's suit.
static bool inSubtree(Part *root, Part *target) {
    for(Part *c : root->contents) {
        if(c == target) { return true; }
        if(inSubtree(c, target)) { return true; }
    }
    for(Part *suit : root->seats) {
        if(suit == target) { return true; }
        if(suit != nullptr && inSubtree(suit, target)) { return true; }
    }
    return false;
}

float inventorySubtreeResource(Part *root, int res) {
    if(root == nullptr) { return 0.0f; }
    float total = 0.0f;
    for(Part *c : root->contents) {
        total += c->resources.current[res];
        total += inventorySubtreeResource(c, res);
    }
    return total;
}

/* Drain up to `amt` from root's subtree, DFS, returning how much was taken. */
static float drainSubtree(Part *root, int res, float amt) {
    if(root == nullptr || amt <= 0.0f) { return 0.0f; }
    float drained = 0.0f;
    for(Part *c : root->contents) {
        if(drained >= amt) { break; }
        float have = c->resources.current[res];
        if(have > 0.0f) {
            const float take = (have < amt - drained) ? have : (amt - drained);
            c->resources.current[res] = have - take;
            /* body->mass is DRY structure (fuel rides effectiveMass through
               resources.current), so a drain only touches the contents. */
            drained += take;
        }
        if(drained < amt) { drained += drainSubtree(c, res, amt - drained); }
    }
    return drained;
}

bool inventoryDrain(Part *root, int res, float amt) {
    if(root == nullptr || amt <= 0.0f) { return false; }
    // all-or-nothing like the suit: a draw the subtree can't cover thrusts nothing
    if(inventorySubtreeResource(root, res) < amt) { return false; }
    drainSubtree(root, res, amt);
    return true;
}

static bool addToContainer(Part *item, Part *dest) {
    if(dest == nullptr || item == nullptr) { return false; }
    // a double add would list it twice and ~Part would free it twice
    if(item->container != nullptr) { return false; }
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    // capacity is cargo only; seats never eat item slots (#274)
    if((int)dest->contents.size() >= dest->def->inventory_capacity) {
        return false;
    }
    if(dest == item || inSubtree(item, dest)) { return false; }
    dest->contents.push_back(item);
    item->container = dest;
    inventorySetOwner(item, dest->owner);
    return true;
}

bool inventoryTransfer(Part *item, Part *dest) {
    if(item == nullptr || dest == nullptr) { return false; }
    if(item == dest) { return false; }
    if(item->container == nullptr) { return false; }
    // Check capacity AND the cycle before removing from source: a failed
    // transfer must not orphan the item.
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    if((int)dest->contents.size() >= dest->def->inventory_capacity) { return false; }
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
