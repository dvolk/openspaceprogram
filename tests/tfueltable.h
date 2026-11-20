#pragma once
/* One assertion about the fuel-group MEMBER table (#984): the CSR the live drain walks
   (Vehicle::memberStart_ / memberOf_) must list exactly what a scan of `parts` would
   list -- same members, same order, nothing lost -- and the total has to account for
   every grouped tank.

   test_fuel.cpp pinned this for buildFuelGroups() alone, which pins the FUNCTION and not
   the discipline: the safety argument is that every caller which can change a group id
   or move a Part* rebuilds the table. test_dock.cpp and test_contain.cpp already build
   the merge and the split, so asking them the same question is a call, not a fixture.
   Order is part of the contract because the drain sums a layer pro-rata and a scan reads
   `parts` in layout order; reordering is harmless today, so this says "in scan order"
   rather than pretending the order is load-bearing. */

#include <cstdio>
#include <vector>

#include "tchecks.h"
#include "vehicle.h"

inline void checkFuelTableMatchesScan(const Vehicle *v, const char *when) {
    int listed = 0;
    for(int g = 0; g + 1 < (int)v->memberStart_.size(); g++) {
        std::vector<int> want, got;
        for(size_t i = 0; i < v->parts.size(); i++) {
            if(v->parts[i]->isTank() && v->parts[i]->fuelGroup == g) { want.push_back((int)i); }
        }
        got.assign(v->memberOf_.begin() + v->memberStart_[(size_t)g],
                   v->memberOf_.begin() + v->memberStart_[(size_t)g + 1]);
        /* Name the group AND what a pass looks like: one sentence reused for every group
           printed identical text twice when two went wrong, with no id to say which tank
           vanished. A green group that expected nothing says so too. */
        char step[192];
        snprintf(step, sizeof step, "%s: group %d has %d tank(s) in scan order", when, g, (int)want.size());
        CHECK_TRUE(want == got, step);
        listed += (int)got.size();
    }
    int grouped = 0;
    for(size_t i = 0; i < v->parts.size(); i++) {
        if(v->parts[i]->isTank() && v->parts[i]->fuelGroup >= 0) { grouped++; }
    }
    char total[192];
    snprintf(total, sizeof total, "%s: all %d grouped tank(s) appear in some group", when, grouped);
    CHECK_TRUE(listed == grouped, total);
}
