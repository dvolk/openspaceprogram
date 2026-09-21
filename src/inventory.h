#pragma once

// inventory.h -- phase 4.3: transfer inventory items between containers.
//
// An inventory item is a Part (a cargo crate, a spare tank) that is contained
// in another Part (a container, PartDef::inventory_capacity > 0). The
// container OWNS the item (Part::ownedContents) and the item appears in the
// container's non-owning traversal list (Part::contents, walked by
// effectiveMass and the UI).
//
// Transfer is pure re-parenting: pop from the old container's ownedContents
// + contents, push to the new container's ownedContents + contents, set the
// item's container pointer. No physics, no Assembly, no Trajectory.

struct Part;   // forward-declared to keep this header light
class Vehicle;

/* Re-point `item`'s owner AND the owner of every item in its inventory
   subtree (a crate in a crate). The item rides the vehicle of its
   OUTERMOST container, so moving that container -- or dropping the item
   (dropItem) -- moves every nested owner with it. nullptr when the
   subtree leaves every vehicle. */
void inventorySetOwner(Part *item, Vehicle *owner);

// Transfer `item` (currently in its container's inventory) into `dest`.
// Enforces dest's capacity (dest->def->inventory_capacity). Returns true on
// success, false if dest is full or item has no current container.
bool inventoryTransfer(Part *item, Part *dest);

// Add `item` to `dest`'s inventory (ownership + traversal). Enforces
// capacity. Returns true on success, false if dest is full or not a
// container. Used by pickup (4.4) to re-parent a dropped item.
bool inventoryAdd(Part *item, Part *dest);

// Remove `item` from its container (ownership + traversal). The item is NOT
// freed -- it is returned to the caller (for drop/pickup in 4.4).
void inventoryRemove(Part *item);
