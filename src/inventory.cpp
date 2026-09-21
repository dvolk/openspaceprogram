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

/* Is `target` `root` itself or anywhere inside its inventory subtree?
   Adding root into target would close a containment cycle: effectiveMass()
   would recurse forever and ~Part would double-free. */
static bool inSubtree(Part *root, Part *target) {
    for(Part *c : root->ownedContents) {
        if(c == target) { return true; }
        if(inSubtree(c, target)) { return true; }
    }
    return false;
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
    // check dest capacity BEFORE removing from source (a failed transfer
    // must not orphan the item)
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    if((int)dest->ownedContents.size() >= dest->def->inventory_capacity) { return false; }
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
