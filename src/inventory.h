#pragma once

// inventory.h -- transfer inventory items between containers.

struct Part;
class Vehicle;

// Re-points `item`'s owner AND its inventory/seat subtree (crate-in-crate):
// the item rides the vehicle of its OUTERMOST container.
void inventorySetOwner(Part *item, Vehicle *owner);

// Transfer `item` into `dest`; false if dest is full or item has no container.
bool inventoryTransfer(Part *item, Part *dest);

// Add `item` to `dest`'s inventory (ownership + traversal). Enforces capacity.
bool inventoryAdd(Part *item, Part *dest);

// Remove `item` from its container (ownership + traversal). NOT freed -- the
// caller owns it (drop/pickup). False if it has no container or is a seat
// occupant: a person leaves its capsule through removeSeat (EVA), never here.
bool inventoryRemove(Part *item);

// Sum resource `res` over `root`'s inventory subtree (cargo only -- seat suit
// tanks belong to the person, not the container). Pocket tanks sit outside
// the ship's fuel groups, so the EVA pocket draw reads them here.
double inventorySubtreeResource(Part *root, int res);

// Drain up to `amt` of resource `res` from `root`'s inventory subtree.
// All-or-nothing: a partial draw never applies a full-force kick.
bool inventoryDrain(Part *root, int res, double amt);
