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
// Mass: PartDef::mass is the DRY structure (res/data/parts.json mass excludes
// propellant). Propellant rides capacity: H2+LOX becomes the burnable
// pool (partPropellantMass), every other resource is inert mass folded
// into partDryMass, EC has none.
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
// TWR and still get delta-v). `exhaust_scale` multiplies every engine's
// thrust (the New Game difficulty / Vehicle::exhaust_scale) so the table
// matches what a launch will actually produce; the fuel burn does not
// scale. Rows are in flight order: the first burn first. Empty build ->
// empty vector.
//
// Stage periods are the numbers that carry a decoupler, plus the highest
// stage number (the final burn). A stage that only lights engines sits
// between two drops: those engines join the burn of the next period that
// actually lasts (so their thrust shows up in that row's TWR / delta-v).
std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale = 1.0);

// Burnable propellant of the part (kg): H2 + LOX from its capacity.
// JetFuel, hydrazine, O2, water and food are carried but never burned in
// vacuum; EC is storage in Wh, not kg.
double partPropellantMass(const PartDef &def);

// Inert mass: the DRY structure (PartDef::mass, which excludes propellant)
// plus the non-burnable resources it carries (crew + mono + life support +
// jet fuel), from capacity. EC has no mass. The complement of
// partPropellantMass over the part's full mass (def.mass + capacity).
inline double partDryMass(const PartDef &def) {
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(r == (size_t)ResourceType::EC) { continue; }        // energy, no mass
        if(r == (size_t)ResourceType::Hydrogen) { continue; }  // burnable: sp.fuel
        if(r == (size_t)ResourceType::LOX) { continue; }       // burnable: sp.fuel
        m += (double)def.capacity[r];
    }
    return m;
}
