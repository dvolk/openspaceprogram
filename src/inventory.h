#pragma once

// inventory.h -- transfer inventory items between containers.

struct Part;   // forward-declared to keep this header light
class Vehicle;

// Re-point `item`'s owner AND its inventory subtree (crate-in-crate). The item
// rides the vehicle of its OUTERMOST container.
void inventorySetOwner(Part *item, Vehicle *owner);

// Transfer `item` into `dest`. Enforces dest's capacity. False if dest is full
// or item has no current container.
bool inventoryTransfer(Part *item, Part *dest);

// Add `item` to `dest`'s inventory (ownership + traversal). Enforces capacity.
bool inventoryAdd(Part *item, Part *dest);

// Remove `item` from its container (ownership + traversal). The item is NOT
// freed -- returned to the caller (for drop/pickup).
void inventoryRemove(Part *item);

// Sum resource `res` across `root`'s inventory subtree (DFS over
// ownedContents). Pocket tanks are outside the ship's fuel groups, so this is
// how the EVA pocket draw reads them.
float inventorySubtreeResource(Part *root, int res);

// Drain up to `amt` of resource `res` from `root`'s inventory subtree.
// All-or-nothing: a partial draw never applies a full-force kick.
bool inventoryDrain(Part *root, int res, float amt);
