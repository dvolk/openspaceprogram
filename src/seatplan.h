// seatplan.h -- the VAB seat plan's pure rules, free of Game so the e2e hooks
// that used to smuggle a forbidden plan can be unit tests instead.
//
// A seat plan is the player's last word on the crew (VabState::seatAssigns):
// one entry per named seat. Two rules guard it, and both used to be reachable
// only through a test hook that wrote the plan behind the window's back:
//
//   prune   -- an entry whose part left the build is dropped (issue #288).
//              vabLaunch calls this before it reads the plan, so a detach and
//              a launch in the same frame cannot see a stale entry (case 244).
//   problem -- one person cannot sit in two seats, and every named person must
//              be standing on the pad. The window greys the double seat, so the
//              refusal is the backstop for a plan that goes stale (case 237).
#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// Where VabState::seatAssigns lives once the Game-side wrapper is gone.
struct SeatAssign {
    std::string partId;
    int slot = 0;            // 0-based seat within the capsule
    uint64_t profileId = 0;  // roster id worn by the standing suit; 0 = empty
};

/* Drop assignments whose seat is no longer in the build. `hasSeat` answers
   "does (partId, slot) name a seat of the current build"; `nameOf` maps a
   profile id to its display name for the log line ("" if unknown). Returns the
   number dropped. */
inline int pruneSeatPlan(std::vector<SeatAssign> &assigns,
                         const std::function<bool(const std::string &, int)> &hasSeat,
                         const std::function<std::string(uint64_t)> &nameOf) {
    int dropped = 0;
    for(size_t i = assigns.size(); i-- > 0; ) {
        const SeatAssign &a = assigns[i];
        if(hasSeat(a.partId, a.slot)) { continue; }
        const std::string who = nameOf(a.profileId);
        // Named HERE, per entry: a summary cannot say which of several parts
        // went, and naming one of them reads as blaming the wrong one.
        printf("[vab] dropped the seat plan for '%s'%s (it left the build)\n",
               a.partId.c_str(), who.empty() ? "" : (" '" + who + "'").c_str());
        fflush(stdout);
        assigns.erase(assigns.begin() + (std::ptrdiff_t)i);
        dropped++;
    }
    return dropped;
}

/* "" if every assignment can be honoured at launch; else why to refuse.
   `isStanding` answers "is this profile on the pad"; `nameOf` as above (the
   raw roster id is used when the name is unknown). Checked BEFORE the ship
   exists: a half-launched ship with an unfillable seat is worse than a refusal
   the player can fix in the window. */
inline std::string seatPlanProblem(const std::vector<SeatAssign> &assigns,
                                   const std::function<bool(uint64_t)> &isStanding,
                                   const std::function<std::string(uint64_t)> &nameOf) {
    for(const SeatAssign &a : assigns) {
        if(a.profileId == 0) { continue; }   // a chosen empty seat names nobody
        std::string who = nameOf(a.profileId);
        if(who.empty()) { who = "roster id " + std::to_string(a.profileId); }
        if(!isStanding(a.profileId)) { return "'" + who + "' is not at the launch pad"; }
        for(const SeatAssign &b : assigns) {
            if(&b == &a) { continue; }
            if(b.profileId == a.profileId) {
                return "'" + who + "' cannot sit in two seats";
            }
        }
    }
    return "";
}
