#pragma once

// part.h -- Part: one part INSTANCE of a ship, and its per-part state.
//
// Behavior (thruster / reaction wheel / RCS / capsule) is DERIVED from the
// PartDef, not stored.
//
// Ownership: Part OWNS its Body; Vehicle owns the Part; the PartDef is
// non-owning (the catalog outlives the ship). The containment edge
// (container/contents) is NON-OWNING both ways -- ~Part must never delete
// contents: a contained kerbal is owned by its Kerbal vehicle.

#include <atomic>
#include <cstdint>

#include "body.h"      // Body (complete type -- ~Part deletes it)
#include "science.h"   // Experiment (science data held on kerbal suits)
#include "shipdef.h"   // PartDef, ResourceContent

class Vehicle;   // Part::owner (the parts list it is attached to)

/* Process-wide monotonic counter for Part::uid. Atomic so a part-level job
   later is not a data race. */
inline uint64_t nextPartUid() {
    static std::atomic<uint64_t> n{0};
    return ++n;
}

/* Parachute deployment state (Part::chute). Stowed: inert structure, no aero.
   Deployed: def.chute_area joins the aero pass as an additive drag term at the
   part (Vehicle::applyAeroForce). One-shot; a Semi state slots in here. */
enum ChuteState {
    ChuteStowed = 0,
    ChuteDeployed = 1
};

struct Part {
    Body *body;                 // OWNED (render assets are registry-shared)
    const PartDef *def;         // non-owning; points into the PartsCatalog
    /* The engine shroud: NON-OWNING (the registries own them); null without one. */
    Mesh *shroud = nullptr;
    Texture *shroud_texture = nullptr;
    /* The parachute canopy overlay: NON-OWNING, null for non-chute parts.
       Resolved at build/load, drawn while chute == ChuteDeployed. */
    Mesh *chute_mesh = nullptr;
    Texture *chute_texture = nullptr;
    /* GLOBALLY unique instance identity. Distinct across every part of every
       ship -- `id` below is NOT once ships merge. Never 0. */
    uint64_t uid;

    /* The instance id from the ship def. Unique only WITHIN one ship def.
       Authoring-facing; use `uid` to identify a part. */
    std::string id;
    ResourceContent resources;  // tank contents (all-zero for non-tank parts)
    /* Science findings held on this part (Part::canHold). Saved with the part. */
    std::vector<Experiment> experiments;
    int stage = 1;              // from the ship def (1 = single stage)
    int fuelGroup = -1;         // fuel-group id (Vehicle::buildFuelGroups); -1 = a fuel barrier
    float armedThrust = 0.0f;   // N armed this tick (disarmed by clearThrust)
    /* Parachute state (ChuteState); inert for non-chute parts. Set by
       Vehicle::deployChutes on the stage press; save-persisted. */
    ChuteState chute = ChuteStowed;

    /* The part-tree edge: the part this one is welded to (nullptr for the
       root). Topology only -- no physics handle. */
    Part *parent = nullptr;

    /* --- the containment edge ------------------------------------------
       Every Part is attached to exactly one Vehicle (`owner`) and contained in
       at most one Part. `container`/`contents` are NON-OWNING; inventory items
       are OWNED by the container via `ownedContents` -- they have no other owner.
       Vehicle::checkPartInvariants enforces this on every build/stage/refresh. */
    Vehicle *owner = nullptr;      // attached: the Vehicle whose parts list holds this
    Part *container = nullptr;     // contained: the part this one is parked in
    std::vector<Part *> contents;  // contained: the parts parked in this one (non-owning)
    /* OWNED inventory items (~Part deletes them). A capsule's seat occupants are
       here too -- a seat is containment that knows who sits in it, below. An
       item here is ALSO in `contents`. */
    std::vector<Part *> ownedContents;

    /* --- a seat: the containment edge, plus an identity -------------------
       A seated kerbal is NOT a Vehicle. Its suit Part is parked in the capsule
       (in `contents`, owned through `ownedContents`, exactly like an inventory
       item) and carries who sits in the seat. Minting a Kerbal vehicle IS going
       EVA (Game::kerbalEVA); deleting one IS boarding. Occupancy is derived,
       never stored -- see isSeat() -- so there is no third pointer to keep in
       step with the containment edge. These fields are meaningful only for an
       occupant. */
    uint64_t seatProfile = 0;  // the CrewRoster entry this person IS; 0 = anonymous
    /* Display name for an ANONYMOUS occupant only (a save that predates the
       roster), which the roster has no entry to name. When seatProfile is set
       the roster owns the name and this stays empty. */
    std::string seatName;

    /* Pose in the VEHICLE's frame S -- one S per Vehicle, anchored at its root
       part (Vehicle::rootPart), NOT at this part. Written at attach time; only
       absorbShip / extractSubtreeAsShip re-base it later, moving a docked ship's
       parts into the survivor's S and back on split. The world pose is DERIVED
       from these (Vehicle::partWorldPose); the parent edge is topology, never a
       pose frame. */
    glm::dvec3 localPos = glm::dvec3(0.0);
    glm::dmat3 localRot = glm::dmat3(1.0);

    Part() : body(nullptr), def(nullptr), uid(nextPartUid()) { }
    ~Part() {
        for(Part *c : ownedContents) { delete c; }  // owned inventory items
        delete body;
    }

    /* --- derived behavior (field-driven; checks are independent) --- */
    bool isThruster() const {
        return def != nullptr
            && def->totalPropellantRate() > 0.0 && def->exhaust_velocity > 0.0;
    }
    /* an air-breathing thruster (a jet engine): burns jet fuel only (air is
       the free oxidizer). Dead in vacuum. */
    bool isJet() const { return isThruster() && def->jet; }
    bool isWheel() const { return def != nullptr && def->torque > 0.0; }
    bool isRcs() const { return def != nullptr && def->rcs_thrust > 0.0; }
    bool isDecoupler() const { return def != nullptr && def->decoupler; }
    bool isDockingPort() const { return def != nullptr && def->docking_port; }
    bool isChute() const { return def != nullptr && def->chute_area > 0.0; }
    bool isFuelBarrier() const { return def != nullptr && def->fuel_barrier; }
    bool isCapsule() const { return def != nullptr && def->crew_capacity > 0; }
    bool isContainer() const { return def != nullptr && def->inventory_capacity > 0; }
    /* A seat OCCUPANT: a suit Part parked in a capsule. Derived from the def
       type and the containment edge, so it cannot drift from either and needs
       no flag of its own. */
    bool isSeat() const {
        return container != nullptr && def != nullptr && def->type == "kerbal";
    }
    /* The seat occupants parked in this part (empty unless it is a capsule). */
    std::vector<Part *> seats() const {
        std::vector<Part *> out;
        for(Part *c : contents) { if(c->isSeat()) { out.push_back(c); } }
        return out;
    }
    /* Seats taken, for the capacity check (`def->crew_capacity` is the ceiling). */
    int seatCount() const {
        int n = 0;
        for(Part *c : contents) { if(c->isSeat()) { n++; } }
        return n;
    }
    /* Is this part an inventory item of `container` (in its ownedContents)?
       Seat occupants are owned too, so this does NOT separate them from items
       -- isSeat() does. */
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
       battery, so the fuel system seeds + fuel-groups it like any tank. */
    bool isBattery() const {
        return def != nullptr && def->capacity[(int)ResourceType::EC] > 0.0f;
    }

    /* Can this part hold finding `e` under its PartDef.experiment_storage role
       (shipdef.h ExpStorage)? Never the exact same finding twice; the role sets
       the ceiling. */
    bool canHold(const Experiment &e) const {
        if(def == nullptr) { return false; }
        return canHoldFinding(def->experiment_storage, def->experiment_family,
                              experiments, e);
    }

    /* Store finding `e` on this part: the full canHold guard + append. */
    bool addExperiment(const Experiment &e) {
        if(!canHold(e)) { return false; }
        experiments.push_back(e);
        return true;
    }

    /* Can this part RECEIVE a finding by transfer? Courier and Container
       take findings in; an Instrument only PRODUCES its own. */
    bool canReceive() const {
        return def != nullptr
            && (def->experiment_storage == ExpStorage::Courier
                || def->experiment_storage == ExpStorage::Container);
    }

    /* --- derived behavior values (read straight off the def) --- */
    double thrust() const { return def->fullThrust(); }  // rocket rated thrust (N); jets use jetThrust
    double jetFuelRate() const { return def->propellant_rate[(int)ResourceType::JetFuel]; }
    double wheelTorque() const { return def->torque; }    // N m, rated
    double rcsThrust() const { return def->rcs_thrust; }  // N, rated translation authority
    double exhaustVelocity() const { return def->exhaust_velocity; }
    double powerDraw() const { return def->power_draw; }          // W, only while active
    double powerDrawConstant() const { return def->power_draw_constant; }  // W, all the time
    double powerGen() const { return def->power_gen; }            // W, constant source

    /* Mass INCLUDING propellant contents and whatever is parked inside (the
       containment edge, recursively). Body mass is DRY structure only; EC is
       excluded (energy, no mass). */
    double effectiveMass() const {
        double m = body->mass;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(r == (int)ResourceType::EC) { continue; }   // energy, no mass
            m += (double)resources.current[r];
        }
        for(Part *c : contents) { m += c->effectiveMass(); }
        return m;
    }
};
