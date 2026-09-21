// inventory.cpp -- phase 4.3: transfer inventory items between containers.

#include "inventory.h"
#include "part.h"

#include <algorithm>

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
    return true;
}

static bool addToContainer(Part *item, Part *dest) {
    if(dest == nullptr || item == nullptr) { return false; }
    if(dest->def == nullptr || dest->def->inventory_capacity <= 0) { return false; }
    // capacity check: count owned items (not crew -- they are in contents
    // but not in ownedContents)
    if((int)dest->ownedContents.size() >= dest->def->inventory_capacity) {
        return false;
    }
    dest->ownedContents.push_back(item);
    dest->contents.push_back(item);
    item->container = dest;
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
