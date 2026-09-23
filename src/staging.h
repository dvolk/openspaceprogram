// staging.h -- VAB staging analysis: per-stage delta-v and TWR, with fuel
// links (asparagus) accounted for.
//
// Pure math over BuildShip (no GL / Bullet), so the VAB table and the unit
// tests share one implementation. The burn model matches flight's rocket
// drain:
//   * fuel groups (decouplers / barriers split them),
//   * one-way fuel links drained FURTHEST-layer-first (Vehicle::
//     fuelDrainLayers) -- outer asparagus tanks empty before the core,
//   * engines light when the stage counter reaches their stage,
//   * a stage press drops the decouplers on that stage (Vehicle::
//     droppedPartsAtStage: the decoupler + its child-side subtree).
//
// The VAB estimate fires each stage when the parts it would drop have
// emptied every propellant atom any active engine can still draw from
// them (so asparagus drops the spent outer boosters on time). An inert
// drop (a payload separator -- nothing drainable in the set) burns the
// remaining reachable propellant instead of ending the period at zero
// length. The last stage burns whatever the still-lit engines can reach.
//
// Vacuum model: only H2+LOX burns (jets need air, so they contribute no
// vacuum delta-v and their JetFuel stays as carried mass). Hydrazine,
// O2, water and food are likewise inert mass here. EC has no mass.
//
// Mass: PartDef::mass is the WET mass (dry + full tanks).
#pragma once

#include <vector>

#include "shipdef.h"   // BuildShip, PartDef, ResourceType

// One row of the staging table (one stage-number period, flight order).
struct StageRow {
    int stage = 1;
    double deltaV = 0.0;     // m/s, vacuum
    double minTWR = 0.0;     // TWR at ignition (heaviest) on the reference g
    double maxTWR = 0.0;     // TWR just before the drop (lightest)
    double massStart = 0.0;  // kg, at ignition / previous drop
    double massEnd = 0.0;    // kg, just before this stage's drop
    int engines = 0;         // rocket engines firing during the burn
    bool drops = false;      // this stage separates parts (vs a pure burn)
};

// Simulate the staged burn of `ship` (tanks start FULL) against surface
// gravity `g` (m/s^2 -- pass the system home body's g; pass 0 to skip
// TWR and still get delta-v). Rows are in flight order: the first burn
// first. Empty build -> empty vector.
//
// Stage periods are the numbers that carry a decoupler, plus the highest
// stage number (the final burn). A stage that only lights engines sits
// between two drops: those engines join the burn of the next period that
// actually lasts (so their thrust shows up in that row's TWR / delta-v).
std::vector<StageRow> computeStaging(const BuildShip &ship, double g);

// Burnable propellant folded into PartDef::mass (kg): H2 + LOX. JetFuel,
// hydrazine, O2, water and food are carried but never burned in vacuum;
// EC is storage in Wh, not kg. Neither is subtracted from dry mass.
double partPropellantMass(const PartDef &def);

// Inert mass: wet minus burnable propellant (structure + crew + mono +
// life support + jet fuel). May be slightly negative on a hand-made def;
// callers clamp when it matters.
inline double partDryMass(const PartDef &def) {
    return def.mass - partPropellantMass(def);
}
