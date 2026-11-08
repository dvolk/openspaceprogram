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
    /* Engine-gimbal state (rad): [0] pitch about the part's local X, [1] yaw
       about its local Y. Transient like armedThrust (not saved); driven by
       Vehicle::applyThrustForce through the src/gimbal.h law. */
    double gimbal[2] = {0.0, 0.0};
    /* Control-surface deflection (rad, signed; positive = the torque the part's
       control_axis steers, matching the aero law's sign flip). Transient like
       gimbal (not saved): applyAeroForce zeroes it every substep and rewrites
       it for each deflecting surface that substep reaches; goOnRails zeroes it
       too (nothing runs there). Render-only -- the force uses the same number
       locally; this field exists so Vehicle::Draw can swing the mesh. */
    double controlDefl = 0.0;
    /* TRIM (deg, #492): the standing command this control surface is held at when
       nothing is asking it for travel. Saved with the part (ShipPart::trim authored
       it, the part window edits it), unlike the deflection above, which is
       per-substep. Applied in applyAeroForce, where it joins the solver's setpoint
       (#426) in the same units -- fractions of this part's travel -- before
       controlDeflection resolves the sign per instance, so a mirrored elevator pair
       and a canard all take the SAME number here and still produce the same torque.
       That is the whole point of putting trim next to the COMMAND rather than making
       it another mount rotation like `roll`. Degrees of surface, so it means the same
       on a 20 deg elevator and a 5 deg one; the sum of setpoint and trim is capped at
       the part's own travel -- which since #426 means a plate already driven to its
       stop by the solver has no travel left for trim (#603, filed not fixed). */
    double trimDeg = 0.0;
    /* Per-nozzle thrust (N) fired THIS RCS substep, indexed by
       PartDef::rcsNozzleCount()/rcsNozzleDir() (authored order, else the
       legacy six). Transient like controlDefl: applyRcsForce zeroes it every
       substep and rewrites the nozzles that fired; goOnRails zeroes it.
       Render-only -- the per-nozzle plumes (render.cpp). */
    double rcsFired[PartDef::kMaxRcsNozzles] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    /* What the aero law read for THIS surface on the substep that last ran
       (#485). Not what the SURFACE window's AoA says: since #453 the angle comes
       from this part's own mount (incidence, dihedral, a rotated fin), and since
       #483 from the flow AT ITS OWN OFFSET, so a rotating airframe gives every
       surface a different alpha and a different q -- up to several degrees apart
       across a twin pair mid-swing on the shipped jet. Degrees, and the Pa of the
       flow standing at this part.
       Diagnostic as shipped -- nothing in the law reads them TODAY, so they cannot
       drift into being load-bearing. The field below is the first one written the
       other way round. applyAeroForce clears aeroValid per
       substep: an invalid part showed no usable flow (no air, a mount with no
       usable axis, or a surface too edge-on to make lift), and a ship that never
       ran the pass keeps whatever it last read -- which is why every reader goes
       through aeroValid. */
    bool aeroValid = false;
    float aeroAlphaDeg = 0.0f;
    float aeroQ = 0.0f;
    /* Mach AT THIS SURFACE (#552 A3): the flow speed localFlow already measured,
       over the sound speed the body's air implies. Printed on the [surface] row
       beside a=/q= today, and unlike the two above it is written to be READ:
       #543's flow-regime law and #487's damping are meant to take this number
       rather than re-derive a speed the pass already has, so the log and the law
       cannot drift into disagreeing about what Mach an airframe is flying. Same
       lifetime caveat, same reason: a part this pass skipped keeps last substep's
       value with aeroValid false, so a reader that wants it for a FORCE goes
       through aeroValid too. */
    float aeroMach = 0.0f;
    /* The watts the air is taking OFF this surface (#730): this surface's own q x its own
       CdA x the flow speed at its own mount, for BOTH bills the aero pass charges it --
       the form drag of #455 and the induced drag of #556, which lands at the same mount,
       on the same q, anti-parallel to the same local flow. No authored material number
       anywhere, which is the point: this is deliberately NOT a skin temperature, since a
       T would want cp and eps per material class plus a time constant, and this game has
       no failure that could spend them yet (#728). Most of these watts leave in the wake
       rather than in the material, which is the honest reason the row cannot be a
       temperature until something authors the share that does not.
       Two bills are missing on purpose. The hull's silhouette (#461) is one ship-wide
       figure with no part to charge, so a part the hull carries quotes nothing rather than
       a share it never paid; and a deployed canopy's bill has no reader -- nothing prints
       a chute part today. Lift itself moves no watts (perpendicular to the flow), but the
       price of making lift IS the induced term, and that one is counted here.
       Unlike the trio above, this is ZEROED each substep rather than left standing: a
       reader could SUM it across a ship, and a stale wattage inside a sum is a wrong total
       rather than a wrong detail. Readers still gate on aeroValid. goOnRails zeroes both
       fields AND clears aeroValid together -- no aero substep runs on rails, and half a
       reset is how a part window renders a stale figure against a zeroed peak.
       The peak is the worst substep since the ship last went on rails -- so it does NOT
       survive a warp or an atmosphere exit -- and it has no durable home yet (no
       Flight Summary row); it is a live-panel figure. */
    float aeroDragWatts = 0.0f;        // [W] this substep
    float aeroDragWattsPeak = 0.0f;    // [W] worst substep since the ship last went on rails
    /* Which LIFTING SYSTEM this surface belongs to (#556/#555), as an index into
       Vehicle::aeroSystems; -1 = not a lifting or steering surface. Written by
       Vehicle::groupLiftingSystems at every geometry rebuild (the same fingerprint
       the silhouette cache uses), so it follows docking, staging, part damage and
       an in-field fit-out. The lift slope and the induced bill both read it, so a
       stale value would be a wrong force -- the reason it is rebuilt rather than
       computed in the aero pass. */
    int aeroSystem = -1;
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
       A straight read -- the def table went double with the contents it bounds
       (#379, #530), so there is no widening left to warn about. Every Part is
       minted with a def (make_part), so a missing one is a bug, not a state to
       survive. */
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
            if(def->capacity[i] > 0.0) { return true; }
        }
        return false;
    }
    /* a battery: EC storage (capacity[EC] > 0). isTank() is ALSO true for a
       battery, so the fuel system seeds + fuel-groups it like any tank. The test
       goes through PartDef::batteryWh() (#644), which is the same number the VAB's
       `store` term sums -- which parts count as the pool and what the pool holds
       must not be two spellings. */
    bool isBattery() const {
        return def != nullptr && def->batteryWh() > 0.0;
    }
    /* Top every resource up to the def's capacity. A fresh ship and a fresh
       suit both start full, and it is one rule, not two -- only the contents
       are state, never the capacity (#383). A non-tank's capacity is all zero,
       so this is a no-op there and callers need not ask isTank() first. */
    void fillTanks() {
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            resources.current[r] = capacityOf(r);
        }
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
    double wheelTorque() const { return def->torque; }    // N m, rated PER DRUM
    double rcsThrust() const { return def->rcs_thrust; }  // N, rated translation authority
    double exhaustVelocity() const { return def->exhaust_velocity; }
    /* Every electrical term comes from the PartDef accessor that owns it (#644), so a
       live hull and the VAB's buildPower() row sum identical arithmetic: what a part
       costs or makes is stated once, and the build screen and the flight log are two
       printings of it rather than two sums. (Which PARTS are on the hull, the EC gate
       and what the drums actually bill stay the sim's -- powerTick.) */
    /* The housing's package figure, ungated: what one wheel block asks for when its
       busiest drum works to its limit. Named apart from powerActiveW() below because
       the #648 bill scales THIS figure by utilisation, while the gate below prices a
       part only if it is a drum at all -- the two are easy to confuse and one of them
       charges the pool wrongly. */
    double powerPackageW() const { return def->power_draw; }
    /* The same figure with the wheel gate inside it -- what a hull OWES the pool for its
       drums, which is how a pod's built-in triad ends up billed like a bought block (#596).
       Null-guarded like isWheel(), whose posture this replaced in wheelDemandWatts(): a
       part with no def owes nothing rather than taking the sim down. */
    double powerActiveW() const { return def == nullptr ? 0.0 : def->powerActiveW(); }
    double powerDrawConstant() const { return def->powerConstW(); }  // W, all the time
    double powerGen() const { return def->powerGenW(); }             // W, constant source

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
