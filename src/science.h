// science.h -- experiments, the career score, and the value model.
// (header-only pure C++)
//
// An Experiment is a small value type used as a uniqueness key: type + body +
// situation + biome, named at the UI edge like
//   "Landed observation of Midlands on Mun".
// Situations (v2): Landed (grounded), LowOrbit / HighOrbit (altitude bands).
// The airborne "Flying" / "Splashed" situations are the next phase.
//
// The Career owns the score + the unique experiments recovered (each with a
// recover count). Recovering an experiment scores it via scoreOf: base x
// situation weight x body weight, halved per repeat (floor 1) -- the
// "don't farm the same observation" pressure.
//
// Pure containers + logic -- no game types -- so tests can pin the value
// curve, the Landed classifier, and the career accounting without linking
// Vehicle / Bullet.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

/* Where the experiment happens. Landed = grounded on a surface; LowOrbit /
   HighOrbit = the two altitude bands over the SoI body (NOT a true orbital
   state -- a suborbital hop still reads as an "orbit observation"). The
   in-atmosphere "Flying" situation is a later phase. */
enum class SciSituation : unsigned char {
    Landed,
    LowOrbit,
    HighOrbit,
};

struct Experiment {
    std::string type = "observation";  // experiment family (extensible id)
    std::string body;                  // SoI body name
    SciSituation situation = SciSituation::Landed;
    std::string biome;                 // biomeName() of the ground below

    bool operator==(const Experiment &o) const {
        return type == o.type && body == o.body
            && situation == o.situation && biome == o.biome;
    }
    bool operator!=(const Experiment &o) const { return !(*this == o); }
};

inline const char *situationName(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:    return "landed";
        case SciSituation::LowOrbit:  return "low orbit";
        case SciSituation::HighOrbit: return "high orbit";
    }
    return "low orbit";
}

inline const char *situationId(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:    return "landed";
        case SciSituation::LowOrbit:  return "low_orbit";
        case SciSituation::HighOrbit: return "high_orbit";
    }
    return "low_orbit";
}

inline SciSituation situationFromId(const std::string &id) {
    if(id == "landed")     { return SciSituation::Landed; }
    if(id == "high_orbit") { return SciSituation::HighOrbit; }
    return SciSituation::LowOrbit;
}

// ASCII first-letter cap for the display name ("midlands" -> "Midlands").
inline std::string capitalizeFirst(std::string s) {
    if(!s.empty() && s[0] >= 'a' && s[0] <= 'z') {
        s[0] = (char)(s[0] - 'a' + 'A');
    }
    return s;
}

// "Landed observation of Midlands on Mun"
inline std::string experimentName(const Experiment &e) {
    return capitalizeFirst(situationName(e.situation)) + " " + e.type
         + " of " + capitalizeFirst(e.biome) + " on " + e.body;
}

// ---- list helpers (a vector of experiments) -------------------------------
inline bool holdsExperiment(const std::vector<Experiment> &v,
                            const Experiment &e) {
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

// ---- the value model ------------------------------------------------------
// Base value per experiment family (extensible; v2 has one family). A new
// family is an explicit entry here -- unknown types fall back to the base
// rather than silently inheriting one.
inline int baseValue(const std::string &type) {
    if(type == "observation") { return 10; }
    return 10;   // unknown family: the base
}

// Situation weight: the "how hard to be there" factor. Landed is the baseline;
// reaching orbit -- and high orbit -- is worth a little more.
inline double situationWeight(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:    return 1.0;
        case SciSituation::LowOrbit:  return 1.25;
        case SciSituation::HighOrbit: return 1.5;
    }
    return 1.0;
}

// Body weight: the home body is 1.0; every other body is the `frontier`
// weight (v2: one flat frontier factor, no per-body table yet).
inline constexpr double kFrontierWeight = 2.0;

/* The science for recovering `e` for the (prevCount+1)th time. Full value on
   first recovery (base x situation x body); each repeat halves it, floor 1 --
   the pressure against farming the same observation. `bodyWeight` is the
   frontier factor (home 1.0, other bodies the `frontier` weight in v2). */
inline int scoreOf(const Experiment &e, int prevCount, double bodyWeight) {
    int v = (int)std::lround(baseValue(e.type) * situationWeight(e.situation)
                             * bodyWeight);
    v = std::max(1, v);
    for(int i = 0; i < prevCount && v > 1; i++) { v /= 2; }
    return v;
}

/* A recovered experiment in the career: the experiment + how many times it
   has been recovered (>=1). The count drives diminishing returns; list order
   is first-seen (the Research Lab archive). */
struct RecoveredExp {
    Experiment e;
    int count = 1;
};

inline RecoveredExp *findRecovered(std::vector<RecoveredExp> &v,
                                   const Experiment &e) {
    for(RecoveredExp &r : v) {
        if(r.e == e) { return &r; }
    }
    return nullptr;
}

/* The career's science: the running score + the unique experiments recovered
   (with their recover counts). Pure -- the game feeds it harvested
   experiments and it owns the score and the diminishing-returns state, so
   the three cannot desync: a point is only ever the scoreOf of a real
   recovery (fixes the v1 score/recovered-list desync, issue #62). */
struct Career {
    int score = 0;
    std::vector<RecoveredExp> recovered;

    // Recover one harvested experiment. Returns the points it scored.
    int recover(const Experiment &e, double bodyWeight) {
        RecoveredExp *r = findRecovered(recovered, e);
        const int prev = (r != nullptr) ? r->count : 0;
        const int gained = scoreOf(e, prev, bodyWeight);
        if(r != nullptr) {
            r->count = prev + 1;
        } else {
            recovered.push_back(RecoveredExp{e, 1});
        }
        score += gained;
        return gained;
    }
};

/* The outcome of one recovery: the points it scored, split into fresh (first
   time) and repeat (already recovered, scored down) -- for the Flight
   Summary. `fresh` lists the first-time keys, in loot order. */
struct RecoverSummary {
    int gained = 0;
    int repeat = 0;
    std::vector<Experiment> fresh;
};

/* Score a whole recovery of a bag of experiments (the ship's parts + every
   aboard crew's suit) against the career, muting it. The bag is de-duplicated
   by key first, so ONE observation held by N kerbals banks ONCE at its current
   value -- v1's per-recovery dedup, preserved under the value model (without
   this, N crew = a farming exploit: 15 + 7 + 4 + ... per observation).
   `homeName` is the 1.0-weight body; every other body gets `frontier`. */
inline RecoverSummary recoverMany(Career &c, const std::vector<Experiment> &loot,
                                  const std::string &homeName, double frontier) {
    RecoverSummary out;
    std::vector<Experiment> uniq;
    for(const Experiment &e : loot) {
        if(!holdsExperiment(uniq, e)) { uniq.push_back(e); }
    }
    for(const Experiment &e : uniq) {
        const double bw = (e.body == homeName) ? 1.0 : frontier;
        const bool isNew = findRecovered(c.recovered, e) == nullptr;
        const int g = c.recover(e, bw);
        out.gained += g;
        if(isNew) {
            out.fresh.push_back(e);
        } else {
            out.repeat += g;
        }
    }
    return out;
}

// ---- the situation classifier --------------------------------------------
/* The low/high cut [m above sea level]: the body's physical atmosphere top
   when it has one (AtmosphereParams::top), else half the body radius. One
   home for the policy so the situation label and any later "is this space?"
   check cannot disagree. */
inline double situationCutAlt(double atmoTop, double radius) {
    return (atmoTop > 0.0) ? atmoTop : 0.5 * radius;
}

/* The situation from the vessel's live state: grounded -> Landed; otherwise
   the altitude band (below the cut -> LowOrbit, above -> HighOrbit). A ship
   in ascent or a suborbital hop is not grounded, so it still reads as an
   orbit observation -- the airborne "Flying" situation is the next phase.
   Edge: isGrounded() is a tolerance (COM within ~100 m of terrain AND slow),
   so a slow low-hover reads as Landed, not LowOrbit -- acceptable now, and
   the "Flying" situation is where a real airborne state lands (issue #70).
   (v1's situationFromAltitude is superseded: it read a landed ship as an
   "orbit observation", issue #75.) */
inline SciSituation situationFor(bool grounded, double altAsl,
                                 double atmoTop, double radius) {
    if(grounded) { return SciSituation::Landed; }
    return (altAsl < situationCutAlt(atmoTop, radius))
               ? SciSituation::LowOrbit
               : SciSituation::HighOrbit;
}
