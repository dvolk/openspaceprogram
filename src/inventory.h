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

// Transfer `item` (currently in its container's inventory) into `dest`.
// Enforces dest's capacity (dest->def->inventory_capacity). Returns true on
// success, false if dest is full or item has no current container.
bool inventoryTransfer(Part *item, Part *dest);

// Remove `item` from its container (ownership + traversal). The item is NOT
// freed -- it is returned to the caller (for drop/pickup in 4.4).
void inventoryRemove(Part *item);
