#pragma once

// part.h -- Part: one part INSTANCE of a ship, and its per-part state.
//
// A Part pairs a physics/render Body with the catalog spec (PartDef) it was
// built from, and carries the per-part state that used to live in the
// per-index vectors kept parallel to Vehicle::parts:
//   - the propellant tank contents (ResourceContent),
//   - the stage number (from the ship def, not the catalog),
//   - the transient per-tick armed thrust,
//   - the part-tree edge (parent) and the authored ship-local pose,
//   - the parked (rails) pose relative to the cluster COM.
//
// Behavior (thruster / reaction wheel / capsule) is DERIVED from the
// PartDef, not stored: a Part is a thruster iff its def has
// fuel_rate + exhaust_velocity, a wheel iff it has torque, a capsule iff it
// has crew_capacity. That is what lets Vehicle drop the old m_thrusters /
// m_reaction_wheels / m_thruster* / m_wheel* vectors and the rebuildBehavior
// bookkeeping that kept them in sync.
//
// Ownership: Part OWNS its Body (deletes it in ~Part). The PartDef is
// non-owning (the catalog outlives the ship). Vehicle owns the Part (deletes
// each Part in ~Vehicle / separateStage).

#include "body.h"      // Body (complete type -- ~Part deletes it)
#include "shipdef.h"   // PartDef, ResourceContent

struct Part {
    Body *body;                 // OWNED (the rigid body + render model)
    const PartDef *def;         // non-owning; points into the PartsCatalog
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

    /* Authored pose in the SHIP-LOCAL frame S, where S is the root part's
       frame at build time: the root gets zero/identity and every other part
       is placed relative to it (build_ship's pos[]/rot[], from attachPose --
       geometry pinned numerically by test_shipload). Pure geometry: fixed at
       attach time, independent of mass, fuel and crew, never mutated after.
       A part's world pose is DERIVED from these (see Vehicle). */
    glm::dvec3 localPos = glm::dvec3(0.0);
    glm::dmat3 localRot = glm::dmat3(1.0);

    /* parked (rails) pose relative to the cluster COM, in cluster axes.
       Written by goOnRails(), read by writeRailPose(); identity/zero for a
       part not currently railed. */

    Part() : body(nullptr), def(nullptr) { }
    ~Part() { delete body; }

    /* --- derived behavior (see the header comment): field-driven, so the
       checks are independent and a part may carry any combination --- */
    bool isThruster() const {
        return def != nullptr
            && def->fuel_rate > 0.0 && def->exhaust_velocity > 0.0;
    }
    bool isWheel() const { return def != nullptr && def->torque > 0.0; }
    bool isDecoupler() const { return def != nullptr && def->decoupler; }
    bool isFuelBarrier() const { return def != nullptr && def->fuel_barrier; }
    bool isCapsule() const { return def != nullptr && def->crew_capacity > 0; }
    bool isTank() const {
        if(def == nullptr) { return false; }
        for(size_t i = 0; i < def->capacity.size(); i++) {
            if(def->capacity[i] > 0.0f) { return true; }
        }
        return false;
    }

    /* --- derived behavior values (the old per-thruster / per-wheel
       vectors, now read straight off the def) --- */
    double thrust() const { return def->fullThrust(); }  // N at full throttle
    double rate() const { return def->fuel_rate; }        // kg/s per tank
    double wheelTorque() const { return def->torque; }    // N m, rated
    double exhaustVelocity() const { return def->exhaust_velocity; }
};
