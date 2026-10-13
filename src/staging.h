// staging.h -- VAB staging analysis: per-stage delta-v and TWR, fuel links
// (asparagus) included. Pure math over BuildShip; the burn model matches
// flight (see staging.cpp).

#pragma once

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

// Simulate the staged burn of `ship` (tanks start FULL) against surface
// gravity `g` (0 skips TWR). `exhaust_scale` scales thrust only, never the
// fuel burn. Stage periods: the stage numbers carrying a decoupler, plus the
// highest.
std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale = 1.0);

// Burnable propellant of the part (kg): capacity of the resources flagged in
// `burn`.
double partPropellantMass(const PartDef &def, const bool *burn);

// Inert mass: PartDef::mass (dry, excludes propellant) plus every resource
// not flagged burnable.
inline double partDryMass(const PartDef &def, const bool *burn) {
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(r == (size_t)ResourceType::EC) { continue; }  // energy, no mass
        if(burn[r]) { continue; }                        // burnable: sp.fuel
        m += (double)def.capacity[r];
    }
    return m;
}
