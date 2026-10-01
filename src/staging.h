// staging.h -- VAB staging analysis: per-stage delta-v and TWR, with fuel
// links (asparagus) accounted for.
//
// Pure math over BuildShip (no GL / Bullet). The burn model matches flight's
// rocket drain: fuel groups, furthest-layer-first fuel links, stage-gated
// ignition, and decoupler drops of the child-side subtree.
// Vacuum model: only the propellants the ship's ROCKET engines draw burn;
// jets contribute no vacuum delta-v. EC has no mass.

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
// gravity `g` (0 skips TWR). `exhaust_scale` multiplies every engine's
// thrust (the difficulty knob); the fuel burn does not scale.
// Stage periods are the numbers that carry a decoupler, plus the highest
// stage number (the final burn). A stage that only lights engines joins the
// burn of the next period that actually lasts.
std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale = 1.0);

// Burnable propellant of the part (kg): the capacity of the resources flagged
// in `burn`. Resources no engine burns, jet fuel, hydrazine, O2, water and
// food are carried but inert here; EC is Wh.
double partPropellantMass(const PartDef &def, const bool *burn);

// Inert mass: the DRY structure (PartDef::mass, which excludes propellant)
// plus every resource NOT flagged burnable in `burn` (and not EC).
inline double partDryMass(const PartDef &def, const bool *burn) {
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(r == (size_t)ResourceType::EC) { continue; }  // energy, no mass
        if(burn[r]) { continue; }                        // burnable: sp.fuel
        m += (double)def.capacity[r];
    }
    return m;
}
