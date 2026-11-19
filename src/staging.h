// staging.h -- VAB staging analysis: per-stage delta-v and TWR, fuel links
// (asparagus) included. Pure math over BuildShip; the burn model matches
// flight (see staging.cpp).

#pragma once

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "shipdef.h"   // BuildShip, PartDef, ResourceType

// One row of the staging table (a stage-number period, flight order).
struct StageRow {
    int stage = 1;
    double deltaV = 0.0;     // m/s, vacuum
    double minTWR = 0.0;     // TWR at ignition (heaviest)
    double maxTWR = 0.0;     // TWR at the drop (lightest)
    double massStart = 0.0;  // kg, at ignition
    double massEnd = 0.0;    // kg, at the drop
    int engines = 0;         // rocket engines firing
    bool drops = false;      // separates parts (vs a pure burn)
};

/* Per-part seeding, for staging a ship that EXISTS rather than one being
   authored (#948). A flight Vehicle holds what it holds: tanks partly drained,
   crew and inventory riding the hull. Index-aligned with ship.parts; an empty
   member keeps the as-designed figure (full tanks, PartDef dry mass).
   The caller owns the split: pass fuelKg = what the part holds of the burnable
   resources (see stagingPropellantMask) and dryKg = everything else about that
   part, so dryKg + fuelKg equals the mass the ship really carries for it. */
struct StagingSeed {
    std::vector<double> fuelKg;
    std::vector<double> dryKg;
};

// Simulate the staged burn of `ship` against surface
// gravity `g` (0 skips TWR). `exhaust_scale` scales thrust only, never the
// fuel burn. Stage periods: the stage numbers carrying a decoupler, PLUS the highest
// and the lowest whether or not they carry one (the walk asserts it spans
// minStage..maxStage, and a label with nothing to separate is skipped later -- which is
// why a ship can fly N labels while the table lists fewer rows).
// Without `seed`, tanks start FULL -- the VAB's question.
std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale = 1.0,
                                     const StagingSeed &seed = {});

/* Which resources the staged burn may spend: the union over the build's rocket
   engines, jets excluded. A caller filling StagingSeed::fuelKg MUST weigh the
   part with this same rule, or a live ship's rows disagree with the VAB's.
   Same #41 limitation as computeStaging: ship-wide, not per fuel group. */
void stagingPropellantMask(const BuildShip &ship, bool *burn);

/* What a build's authored labels DO, as opposed to what computeStaging says they
   COST (#1010). Appends one short human-readable line per problem to `warns`
   (empty after the call means the order works); it does not sort or clear the
   caller's list.

   A label is a position on the press counter and nothing else, so every rule here
   is about presses: the counter starts at the highest label and steps down one
   integer per press, and the presses a ship can make are the integers hi down to
   lo+1. Reports a press that does nothing (a gap, or a label whose parts answer to
   nothing), a DECOUPLER riding the lowest label (the counter never stands there, so
   it needs a press past the last one counted -- a chute there is the normal way to
   author one, so it is not reported), and a decoupler whose side is every part in
   the ship (the split is refused and the label is spent, #541). Warn-only and
   deliberately so -- see the comment at the definition for what a refuse would
   punish today and which rule is the candidate. */
void validateStaging(const BuildShip &ship, std::vector<std::string> &warns);

/* A content hash of everything computeStaging reads about `ship`, plus the two
   scalars. The Staging window redraws every frame on a build that only changes
   when the player edits it, so its rows are cached against this (#293):
   revalidating is one pass over the parts with no allocation, recomputing is a
   burn simulation (84 us for a one-group ship, 473 us for a 240-group one).
   Only the link ENDPOINTS are hashed: computeStaging resolves a link through
   the part ids and ignores its own id, and an id that appears in no link
   cannot change a row. */
uint64_t stagingSignature(const BuildShip &ship, double g, double exhaust_scale);

// Burnable propellant of the part (kg): capacity of the resources flagged in
// `burn`.
double partPropellantMass(const PartDef &def, const bool *burn);

// Inert mass: PartDef::mass (dry, excludes propellant) plus every resource
// not flagged burnable.
inline double partDryMass(const PartDef &def, const bool *burn) {
    assert(def.capacity.size() == (size_t)ResourceType::Num);  // the loop indexes kResources
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(!resourceHasMass((ResourceType)r)) { continue; }
        if(burn[r]) { continue; }                        // burnable: sp.fuel
        m += (double)def.capacity[r];
    }
    return m;
}
