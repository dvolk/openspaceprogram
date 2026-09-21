#pragma once

// part.h -- Part: one part INSTANCE of a ship, and its per-part state.
//
// A Part pairs a physics/render Body with the catalog spec (PartDef) it was
// built from, and carries the per-part state that used to live in the
// per-index vectors kept parallel to Vehicle::parts:
//   - the propellant tank contents (ResourceContent),
//   - the stage number (from the ship def, not the catalog),
//   - the transient per-tick armed thrust,
//   - the part-tree edge (parent) and the authored ship-local pose.
//
// A Part also carries its own globally unique identity (uid, minted by the
// constructor). Unlike the def-authored `id` -- unique only within one ship
// def -- a uid stays unique when ships merge, which is what makes it usable
// as the key for cross-part references.
//
// Behavior (thruster / reaction wheel / RCS / capsule) is DERIVED from the
// PartDef, not stored: a Part is a thruster iff its def has
// fuel_rate + exhaust_velocity, a wheel iff it has torque, RCS iff it has
// rcs_thrust, a capsule iff it has crew_capacity. That is what lets Vehicle drop the old m_thrusters /
// m_reaction_wheels / m_thruster* / m_wheel* vectors and the rebuildBehavior
// bookkeeping that kept them in sync.
//
// Ownership: Part OWNS its Body (deletes it in ~Part). The PartDef is
// non-owning (the catalog outlives the ship). Vehicle owns the Part (deletes
// each Part in ~Vehicle). The containment edge (container/contents, below)
// is NON-OWNING in both directions: ~Part must never delete contents -- a
// contained kerbal is owned by its Kerbal vehicle (Vehicle::crew), and
// ~Vehicle deletes crew BEFORE its parts, so an owning ~Part would double
// free it deterministically.

#include <atomic>
#include <cstdint>

#include "body.h"      // Body (complete type -- ~Part deletes it)
#include "shipdef.h"   // PartDef, ResourceContent

class Vehicle;   // Part::owner (the parts list it is attached to)

/* Process-wide monotonic counter for Part::uid. A function-local static in an
   inline function is ONE counter across every translation unit, and wrapping
   it keeps the value unforgeable -- callers can mint a uid, not edit the
   sequence. Never reused, never renumbered, so a Part* and its uid identify
   the same part for the whole run.

   Atomic, not because a Part is built off-thread today (the JobRunner is the
   only worker thread and its job bodies are pure math, and every `new Part`
   is on the main thread -- verified), but so that a part-level job becoming a
   thing later is a one-line non-bug rather than a silent data race + possible
   uid collision. The uncontended fetch_add is effectively free. */
inline uint64_t nextPartUid() {
    static std::atomic<uint64_t> n{0};
    return ++n;
}

struct Part {
    Body *body;                 // OWNED (the rigid body + hull shape; the render assets it holds are registry-shared)
    const PartDef *def;         // non-owning; points into the PartsCatalog
    /* The engine shroud (see PartDef.shroud): the open-cylinder wrap drawn
       OVER this part when a part is attached on its exhaust face (a child
       below). NON-OWNING (the get_mesh/get_texture registries own them,
       shared by every part of the same size) and null for parts without a
       shroud declared in the catalog. */
    Mesh *shroud = nullptr;
    Texture *shroud_texture = nullptr;
    /* The GLOBALLY unique instance identity, minted by the constructor from
       nextPartUid(). Distinct across every part of every ship in the process,
       and stable for the part's lifetime -- which `id` below is NOT, so this
       is the key that cross-part references (save/load, fuel links, dock
       seams, the containment edge) should name a part by. Never 0. */
    uint64_t uid;

    /* The instance id from the ship def. Unique only WITHIN one ship def
       (shipdef.cpp enforces that and auto-generates "<catalog name>_<n>"),
       so it is NOT a key once ships merge: absorbShip moves another ship's
       parts in without renaming them, and two ships built from the same def
       collide on every id. Authoring-facing (the def file, the VAB, the
       diagnostics); use `uid` to identify a part. */
    std::string id;
    ResourceContent resources;  // tank contents (all-zero for non-tank parts)
    int stage = 1;              // from the ship def (1 = single stage)
    int fuelGroup = -1;         // fuel-group id (Vehicle::buildFuelGroups); -1 = a fuel barrier, in no group
    float armedThrust = 0.0f;   // N armed this tick (disarmed by clearThrust)

    /* The part-tree edge: the part this one is welded to (nullptr for the
       root). Topology only -- it carries no physics handle. This is the
       adjacency droppedPartsAtStage and buildFuelGroups walk, so it is the
       authoritative source for the tree; a Part* is stable for the ship's
       lifetime, so staging needs no index remapping. */
    Part *parent = nullptr;

    /* --- the containment edge (inventory design report, phase 2) --------
       Every Part is attached to exactly one Vehicle (its `owner`) and is
       contained in at most one Part. Aboard, a kerbal's part is contained
       in its capsule: it rides in `container`'s `contents` instead of being
       simulated in its own right. Phase 4's inventory items are the SAME
       edge under a suit or cargo part -- one mechanism, two features.

       `owner` is the back-pointer to the Vehicle::parts list (which stays
       the ownership list): setRoot/attach set it, absorbShip re-points the
       absorbed ship's parts to the survivor, extractSubtreeAsShip re-points
       the dropped side to the new ship. A Part* is stable through both, so
       only this pointer moves.

       `container`/`contents` are NON-OWNING (see the header): the contained
       kerbal's Part is owned by its Kerbal vehicle (Vehicle::crew). For
       inventory items (phase 4), the container part OWNS them via
       `ownedContents` (below) -- they have no other owner.

       Vehicle::checkPartInvariants enforces this on every build, stage and
       burn-triggered refresh (rebuildCompound), next to
       checkCompoundInvariants. */
    Vehicle *owner = nullptr;      // attached: the Vehicle whose parts list holds this
    Part *container = nullptr;     // contained: the part this one is parked in (a capsule, or a container for an inventory item)
    std::vector<Part *> contents;  // contained: the parts parked in this one (non-owning)
    /* OWNED inventory items (phase 4.3): the parts this one holds in its
       inventory (a cargo crate, a spare tank). ~Part deletes them. Crew
       kerbals are NOT here (they are owned by Vehicle::crew); this is for
       items that have no other owner. An item in ownedContents is ALSO in
       `contents` (the non-owning traversal list that effectiveMass() and
       the UI walk) -- two pointers to the same Part, one for lifetime and
       one for traversal. */
    std::vector<Part *> ownedContents;

    /* Authored pose in the SHIP-LOCAL frame S, where S is the root part's
       frame at build time: the root gets zero/identity and every other part
       is placed relative to it (build_ship's pos[]/rot[], from attachPose --
       geometry pinned numerically by test_shipload). Pure geometry: fixed at
       attach time, independent of mass, fuel and crew, never mutated after.
       A part's world pose is DERIVED from these (see Vehicle). */
    glm::dvec3 localPos = glm::dvec3(0.0);
    glm::dmat3 localRot = glm::dmat3(1.0);

    Part() : body(nullptr), def(nullptr), uid(nextPartUid()) { }
    ~Part() {
        for(Part *c : ownedContents) { delete c; }  // owned inventory items
        delete body;
    }

    /* --- derived behavior (see the header comment): field-driven, so the
       checks are independent and a part may carry any combination --- */
    bool isThruster() const {
        return def != nullptr
            && def->fuel_rate > 0.0 && def->exhaust_velocity > 0.0;
    }
    /* an air-breathing thruster (a jet engine): a thruster flagged jet.
       Same thrust pipeline (armedThrust / applyThrustForce), but the
       arming in Vehicle::ApplyThrust sets the thrust from the
       air-breathing momentum balance (drag.h jetThrust) and draws jet
       fuel (a separate resource) only -- air is the free oxidizer, so no
       LOX and no delta-v. In vacuum (no air) it arms zero thrust: a jet
       cannot thrust in space. */
    bool isJet() const { return isThruster() && def->jet; }
    bool isWheel() const { return def != nullptr && def->torque > 0.0; }
    bool isRcs() const { return def != nullptr && def->rcs_thrust > 0.0; }
    bool isDecoupler() const { return def != nullptr && def->decoupler; }
    bool isDockingPort() const { return def != nullptr && def->docking_port; }
    bool isFuelBarrier() const { return def != nullptr && def->fuel_barrier; }
    bool isCapsule() const { return def != nullptr && def->crew_capacity > 0; }
    bool isContainer() const { return def != nullptr && def->inventory_capacity > 0; }
    /* phase 4: is this part an inventory item of `container` -- i.e. listed
       in its ownedContents? `contents` holds BOTH crew and items, so this is
       what tells them apart: a crew member's part is contained but owned by
       its character vehicle, an item is contained and owned by the
       container itself. */
    bool ownedBy(Part *container) const {
        if(container == nullptr) { return false; }
        for(Part *c : container->ownedContents) { if(c == this) { return true; } }
        return false;
    }
    bool isTank() const {
        if(def == nullptr) { return false; }
        for(size_t i = 0; i < def->capacity.size(); i++) {
            if(def->capacity[i] > 0.0f) { return true; }
        }
        return false;
    }
    /* a battery: EC storage (capacity[EC] > 0). isTank() is ALSO true for a
       battery (it has capacity), so the fuel system seeds + fuel-groups it
       like any tank; the power system reads its EC as the charge. */
    bool isBattery() const {
        return def != nullptr && def->capacity[(int)ResourceType::EC] > 0.0f;
    }

    /* --- derived behavior values (the old per-thruster / per-wheel
       vectors, now read straight off the def) --- */
    double thrust() const { return def->fullThrust(); }  // rocket rated thrust (N); jets use jetThrust
    double rate() const { return def->fuel_rate; }        // kg/s per tank
    double wheelTorque() const { return def->torque; }    // N m, rated
    double rcsThrust() const { return def->rcs_thrust; }  // N, rated translation authority
    double exhaustVelocity() const { return def->exhaust_velocity; }
    double powerDraw() const { return def->power_draw; }          // W, only while active (a reaction wheel)
    double powerDrawConstant() const { return def->power_draw_constant; }  // W, all the time (capsule life support)
    double powerGen() const { return def->power_gen; }            // W, constant source (an RTG)

    /* The part's mass INCLUDING what is parked inside it (the containment
       edge, phase 2): its own body mass plus the effectiveMass of every
       contained part, recursively. Phase 3 wires this into the compound
       (rebuildCompound / compoundCom / checkCompoundInvariants) and the
       force paths (applyGravity / fictitious / getMass) so a capsule's mass
       carries its crew without baking it into the body mass (the addPartMass
       mechanism it replaced). The traversal is non-owning -- the contained
       parts are owned by their own vehicles (Vehicle::crew), so this only
       reads, never frees. checkPartInvariants keeps the containment edge a
       tree (each contained part has at most one container), so the recursion
       is acyclic in every reachable state. */
    double effectiveMass() const {
        double m = body->mass;
        for(Part *c : contents) { m += c->effectiveMass(); }
        return m;
    }
};
