#pragma once

// part.h -- Part: one part INSTANCE of a ship, and its per-part state.
//
// Behavior (thruster / reaction wheel / RCS / capsule) is DERIVED from the
// PartDef, not stored.
//
// Ownership: Part OWNS its Body; Vehicle owns the Part; the PartDef is
// non-owning (the catalog outlives the ship). Containment is ownership:
// ~Part deletes what it holds -- inventory items in `contents` and seat
// suits in `seats`. There is no non-owning containment edge.

#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "body.h"      // Body (complete type -- ~Part deletes it)
#include "crew.h"      // KerbalProfile (identity owned by a kerbal suit)
#include "science.h"   // Experiment (science data held on kerbal suits)
#include "shipdef.h"   // PartDef, ResourceContent

class Vehicle;   // Part::owner (the parts list it is attached to)
struct Part;

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

/* A kerbal is a Part (its suit) at exactly one place: a capsule seat, a
   walking Kerbal vehicle, or an unassigned suit standing on the home pad
   (StaticBuilding::crew). Identity is OWNED by the suit: one person, one
   profile, no roster lookup. */

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

    /* --- containment (OWNED both ways) ---------------------------------
       Every Part is attached to exactly one Vehicle (`owner`) and held by
       at most one Part. Inventory items live in `contents` and back-point
       through `container`; seat occupants live in `seats`. ~Part frees both.
       Vehicle::checkPartInvariants enforces this on every build/stage/refresh. */
    Vehicle *owner = nullptr;      // attached: the Vehicle whose parts list holds this
    Part *container = nullptr;     // held by: inventory parent or the capsule
    /* Inventory items parked in this one. OWNED -- ~Part deletes them. */
    std::vector<Part *> contents;
    /* Seat occupants (capsules only). OWNED -- ~Part deletes the suits. A suit
       is NOT in `contents`: a person is not cargo (issue #274). */
    std::vector<Part *> seats;
    /* Who this suit IS (kerbal-type parts only). Owned: the person dies with
       the suit. Null for every non-person part. */
    std::unique_ptr<KerbalProfile> kerbal;

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
        for(Part *c : contents) { delete c; }
        for(Part *s : seats) { delete s; }
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
    bool isFuelBarrier() const { return def != nullptr && def->isFuelBarrier(); }
    bool isCapsule() const { return def != nullptr && def->crew_capacity > 0; }
    bool isContainer() const { return def != nullptr && def->inventory_capacity > 0; }
    /* How much of that resource the part can hold, straight off the def: it is
       table data, not per-part state, so a copy here could only drift (#383).
       The def table itself is float (PartDef::capacity), so this widens rather
       than adds precision. Every Part is minted with a def (make_part), so a
       missing one is a bug, not a state to survive. */
    double capacityOf(int r) const { assert(def != nullptr); return def->capacity[r]; }
    /* A PERSON that is being held: a kerbal-typed Part with a container. Named
       for what it actually tests, because the predicate must NOT tighten to
       `container->isCapsule()` (#291). The two edges mean different things:
       `seats` is how a capsule holds a suit, `contents` is how anything holds
       cargo, and this is the guard on the second one -- removeFromContainer
       refuses on it, and checkContainmentTree reports "a person is not cargo"
       through it. Both would stop firing for a kerbal part held by a part that
       is not a capsule, which is exactly the state they exist to catch.
       Derived from the def type and the hold edge, so it cannot drift from
       either. */
    bool isContainedPerson() const {
        return container != nullptr && def != nullptr && def->type == "kerbal";
    }
    /* Seats taken (non-null slots). `def->crew_capacity` is the ceiling. */
    int seatCount() const {
        int n = 0;
        for(const Part *s : seats) { if(s != nullptr) { n++; } }
        return n;
    }
    /* Size `seats` to crew_capacity once the def is known (Part is default-
       constructed, then def is assigned). Null slots are empty seats; the
       index is stable for IVA (seat 0 stays seat 0). */
    void ensureSeats() {
        if(def == nullptr) { return; }
        const size_t cap = (def->crew_capacity > 0) ? (size_t)def->crew_capacity : 0;
        if(seats.size() < cap) { seats.resize(cap, nullptr); }
        assert(seats.size() == cap);
    }
    /* Park `suit` in `slot` (default: first free). Takes ownership; sets the
       hold edge. False if the capsule is full or the slot is taken. */
    bool addSeat(Part *suit, size_t slot = (size_t)-1) {
        assert(suit != nullptr && suit->container == nullptr);
        ensureSeats();
        if(slot == (size_t)-1) {
            for(slot = 0; slot < seats.size() && seats[slot] != nullptr; slot++) {}
        }
        if(slot >= seats.size() || seats[slot] != nullptr) { return false; }
        seats[slot] = suit;
        suit->container = this;
        return true;
    }
    /* Unpark `suit`; nulls its slot and returns it with the hold edge cleared.
       The caller owns it (EVA mints a Kerbal vehicle around it). Null if it
       was not seated here. */
    Part *removeSeat(Part *suit) {
        for(size_t i = 0; i < seats.size(); i++) {
            if(seats[i] == suit) {
                seats[i] = nullptr;
                suit->container = nullptr;
                return suit;
            }
        }
        return nullptr;
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

    /* Mass INCLUDING propellant contents and whatever is parked inside (items
       and seats, recursively). Body mass is DRY structure only; EC is excluded
       (energy, no mass). */
    double effectiveMass() const {
        double m = body->mass;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(r == (int)ResourceType::EC) { continue; }   // energy, no mass
            m += resources.current[r];
        }
        for(Part *c : contents) { m += c->effectiveMass(); }
        for(Part *s : seats) {
            if(s != nullptr) { m += s->effectiveMass(); }
        }
        return m;
    }
};

class Shader;   // make_part / make_suit build the Body's render assets with it

/* The ship def a NEW suit is minted from (make_suit, via the crew pool). A save
   does NOT write it: a seat's part entry names the catalog PART by name
   (SavePart::part), and buildSeatParts resolves that against the parts file.
   Only a WALKING kerbal has a defPath, and buildKerbalFromSave now refuses one
   unless the def is a single kerbal part. */
constexpr const char *const kKerbalShipDef = "res/ships/kerbal.json";

/* The ONE way a Part is minted from a catalog def (#278): render assets,
   collision hull, id, resources, findings, the chute canopy and the engine
   shroud. Everything the call sites used to differ in -- a save's own
   hull_margin / fuel / findings vs the catalog's for a fresh part -- is an
   argument, so a field added to Part lands in every path or none. `fuel` and
   `experiments` may be null: a fresh part then takes capacity from the def,
   nothing in it, and no findings. Callers keep their own refusals (the e2e
   cases pin those messages), stage + chute deploy state, and identity.
   Mass is deliberately NOT an argument (#389): it is catalog data, so a Part's
   mass is always its def's and there is no path that mints a part weighing
   something else. The save used to carry its own copy and win on load, which
   froze a career ship's mass against a rebalanced catalog and let stale fixture
   copies drift for months unnoticed. */
Part *make_part(const PartDef &pd, double hull_margin,
                const std::vector<double> *fuel,
                const std::vector<Experiment> *experiments,
                const std::string &id, Shader *partsshader);

/* Mint a suit Part from a kerbal ship def (the occupant of a seat). Its Body is
   created but never enters the physics world -- a seat's mass rides the
   containment edge through Part::effectiveMass. Tanks are seeded full like
   Vehicle::init does; a load overwrites the contents from the save.

   Split from the load (#291): the crew pool mints one suit per roster entry, so
   a single-call make_suit parsed res/ships/kerbal.json once per person (8 file
   parses at boot, and again on every New Game). Load once with load_suit_def,
   then mint as many suits as needed. `def` must come from load_suit_def: the
   one-part / kerbal-type checks live there, so make_suit cannot be handed a
   def that would mint a half-person. */
ShipDef load_suit_def(const std::string &defPath, const PartsCatalog &cat);
Part *make_suit(const ShipDef &def, Shader *partsshader);
