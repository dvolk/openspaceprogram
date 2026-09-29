// science.h -- experiments and the science score (header-only pure C++).
//
// An Experiment is a small value type used as a uniqueness key: type + body +
// situation + biome. v1 has one type ("observation") and two situations
// (low/high orbit), named at the UI edge like
//   "Low orbit observation of Midlands on Mun".
// Kerbals hold experiments on their suit Part (unlimited in v1); recovery
// merges them into Game::recovered and adds scoreOf once per unique key.
// Later: diminishing returns, more types/situations, instrument parts.
//
// Pure containers + logic -- no game types -- so tests can pin uniqueness,
// naming and the altitude cut without linking Vehicle / Bullet (flightlog.h).

#pragma once

#include <string>
#include <vector>

/* Where the vessel is, as the v1 experiment matrix sees it: two altitude
   bands over the SoI body. NOT true orbital state -- a suborbital hop still
   produces an "orbit observation" (the name is the experiment title, not a
   dynamics claim). Later: Landed / Splashed / Flying / Escape. */
enum class SciSituation : unsigned char {
    LowOrbit,
    HighOrbit,
};

struct Experiment {
    std::string type = "observation";  // experiment family (extensible id)
    std::string body;                  // SoI body name
    SciSituation situation = SciSituation::LowOrbit;
    std::string biome;                 // biomeName() of the ground below

    bool operator==(const Experiment &o) const {
        return type == o.type && body == o.body
            && situation == o.situation && biome == o.biome;
    }
    bool operator!=(const Experiment &o) const { return !(*this == o); }
};

inline const char *situationName(SciSituation s) {
    return (s == SciSituation::HighOrbit) ? "high orbit" : "low orbit";
}

inline const char *situationId(SciSituation s) {
    return (s == SciSituation::HighOrbit) ? "high_orbit" : "low_orbit";
}

inline SciSituation situationFromId(const std::string &id) {
    return (id == "high_orbit") ? SciSituation::HighOrbit : SciSituation::LowOrbit;
}

// ASCII first-letter cap for the display name ("midlands" -> "Midlands").
inline std::string capitalizeFirst(std::string s) {
    if(!s.empty() && s[0] >= 'a' && s[0] <= 'z') {
        s[0] = (char)(s[0] - 'a' + 'A');
    }
    return s;
}

// "Low orbit observation of Midlands on Mun"
inline std::string experimentName(const Experiment &e) {
    return capitalizeFirst(situationName(e.situation)) + " " + e.type
         + " of " + capitalizeFirst(e.biome) + " on " + e.body;
}

// v1: every unique experiment is worth one point. The wrapper is the seam
// for a per-type table or diminishing returns later.
inline int scoreOf(const Experiment &) { return 1; }

inline bool holdsExperiment(const std::vector<Experiment> &v, const Experiment &e) {
    for(const Experiment &x : v) {
        if(x == e) { return true; }
    }
    return false;
}

// Append if not already held. True when stored (false = duplicate).
inline bool addExperiment(std::vector<Experiment> &v, const Experiment &e) {
    if(holdsExperiment(v, e)) { return false; }
    v.push_back(e);
    return true;
}

// Merge `src` into `dst`, scoring only the keys `dst` did not already hold.
// Returns the science points gained (sum of scoreOf over the new ones).
inline int mergeExperiments(std::vector<Experiment> &dst,
                            const std::vector<Experiment> &src) {
    int gained = 0;
    for(const Experiment &e : src) {
        if(addExperiment(dst, e)) { gained += scoreOf(e); }
    }
    return gained;
}

/* The low/high cut [m above sea level]: the body's physical atmosphere top
   when it has one (AtmosphereParams::top), else half the body radius. One
   home for the policy so the situation label and any later "is this space?"
   check cannot disagree. */
inline double situationCutAlt(double atmoTop, double radius) {
    return (atmoTop > 0.0) ? atmoTop : 0.5 * radius;
}

inline SciSituation situationFromAltitude(double altAsl, double atmoTop,
                                          double radius) {
    return (altAsl < situationCutAlt(atmoTop, radius))
               ? SciSituation::LowOrbit
               : SciSituation::HighOrbit;
}
