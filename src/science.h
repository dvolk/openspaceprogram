// science.h -- experiments, the career score, and the value model.
// (header-only pure C++)
//
// An Experiment is one run of an observation, recorded by a kerbal on a ship.
// Its IDENTITY -- type + body + situation + biome -- is the uniqueness key
// (named at the UI edge like "Landed observation of Midlands on Mun"); the
// provenance (ran_at, recovered_at, kerbal, ship) is extra data that ==
// ignores. Situations (v2): Landed (grounded), LowOrbit / HighOrbit (altitude
// bands); the airborne "Flying" / "Splashed" situations are the next phase.
//
// The Career owns the score + the append-only log of every bank (recovered).
// Recovering an experiment scores it via scoreOf: base x situation weight x
// body weight, halved once per earlier bank of the same key (floor 1) -- the
// "don't farm the same observation" pressure. The log keeps one entry per
// bank, so the Research Lab shows a full record of collections.
//
// Pure containers + logic -- no game types -- so tests can pin the value
// curve, the Landed classifier, and the career accounting without linking
// Vehicle / Bullet.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "calendar.h"   // Calendar + fmt_cal_compact (the Lab row's stamps)

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
    // identity -- the uniqueness key; == compares ONLY these four
    std::string type = "observation";  // experiment family (extensible id)
    std::string body;                  // SoI body name
    SciSituation situation = SciSituation::Landed;
    std::string biome;                 // biomeName() of the ground below
    // provenance -- == ignores these; a record of one run of that key
    double ran_at = 0.0;       // sim clock: when the kerbal recorded it
    double recovered_at = 0.0; // sim clock: when it was banked (0 = not yet)
    std::string kerbal;        // who ran it
    std::string ship;          // aboard ship name; "" = free EVA

    // == is key-only, on purpose: two runs of the same observation by
    // different kerbals/ships dedup to one bank; they differ only in the
    // provenance fields above. (A reader assuming full-struct == is
    // surprised -- this is deliberate, not an omission.)
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

// How many entries of the same key a log holds (== is key-only, so provenance
// never affects the count) -- the diminishing-returns prevCount.
inline int countKey(const std::vector<Experiment> &v, const Experiment &e) {
    int n = 0;
    for(const Experiment &x : v) {
        if(x == e) { ++n; }
    }
    return n;
}

// ---- the value model ------------------------------------------------------
// Base value per experiment family. A new family is an explicit entry here --
// unknown types fall back to the base rather than silently inheriting one.
// "observation" is the kerbal suit's run (the baseline); "materials study" is
// the Materials Pod's instrument run (PartDef.experiment_family) -- a
// dedicated science part, so it is worth MORE per situation (25 vs 10).
inline int baseValue(const std::string &type) {
    if(type == "observation") { return 10; }
    if(type == "materials study") { return 25; }
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

/* The career's science: the running score + the append-only log of every
   bank. `recovered` holds one Experiment (with its provenance) per bank, in
   bank order -- the Research Lab's full collection record. Pure -- the game
   feeds it harvested experiments and it owns the score and the
   diminishing-returns state together, so they cannot desync: a point is only
   ever the scoreOf of a real bank (fixes the v1 score/recovered-list desync,
   issue #62).

   `version` is bumped on EVERY mutation (a bank, a Load, a reset) so a derived
   cache (the Lab's pre-built rows, issue #87) can tell when it is stale: the
   Lab's "the log can't change for the scene's life" invariant is otherwise
   only enforced by UI coincidence (a failed Load from the Lab does change it).
   All mutation goes through these three methods, so no site can forget to
   bump it. */
struct Career {
    int score = 0;
    std::vector<Experiment> recovered;   // the log, in bank order
    std::size_t version = 0;             // bumped on every mutation above

    // Bank one harvested experiment (e carries its provenance, incl.
    // recovered_at). The (N+1)th bank of a key scores scoreOf(e, N, bodyWeight)
    // -- N is how many are already in the log (key-only count). Returns the
    // points it scored.
    int recover(const Experiment &e, double bodyWeight) {
        const int prev = countKey(recovered, e);
        const int gained = scoreOf(e, prev, bodyWeight);
        recovered.push_back(e);
        score += gained;
        ++version;
        return gained;
    }

    // Replace the whole career (a Load). One entry point for the bulk write
    // (score + log) so the version bump is not forgotten at the call site.
    void setFrom(int newScore, std::vector<Experiment> newLog) {
        score = newScore;
        recovered = std::move(newLog);
        ++version;
    }

    // A fresh career (New Game / unload). Clears + bumps, so a Lab cache built
    // from the old log is detected stale.
    void reset() {
        score = 0;
        recovered.clear();
        ++version;
    }
};

/* The outcome of one recovery: the points it scored, split into fresh (first
   bank of that key, ever) and repeat (already in the log, scored down) -- for
   the Flight Summary. `fresh` lists the first-time entries, in loot order. */
struct RecoverSummary {
    int gained = 0;
    int repeat = 0;
    std::vector<Experiment> fresh;
};

/* Bank a whole recovery of a bag of experiments (the ship's parts + every
   aboard crew's suit) into the career, appending one log entry per key. The
   bag is de-duplicated by key first, so ONE observation held by N kerbals
   banks ONCE at its current value -- v1's per-recovery dedup, preserved under
   the value model (without this, N crew = a farming exploit: 15 + 7 + 4 + ...
   per observation). `homeName` is the 1.0-weight body; every other body gets
   `frontier`; `now` (the sim clock) is stamped as each entry's recovered_at.
   Each credited entry keeps the first bag-occurrence's provenance (who ran
   it, when, on which ship). */
inline RecoverSummary recoverMany(Career &c, const std::vector<Experiment> &loot,
                                  const std::string &homeName, double frontier,
                                  double now) {
    RecoverSummary out;
    std::vector<Experiment> uniq;
    for(const Experiment &e : loot) {
        if(!holdsExperiment(uniq, e)) { uniq.push_back(e); }
    }
    for(const Experiment &e0 : uniq) {
        Experiment e = e0;
        e.recovered_at = now;   // stamp the bank time on the logged entry
        const double bw = (e.body == homeName) ? 1.0 : frontier;
        const bool isNew = !holdsExperiment(c.recovered, e);
        const int g = c.recover(e, bw);
        out.gained += g;
        if(isNew) {
            out.fresh.push_back(e);   // the new key, as banked
        } else {
            out.repeat += g;
        }
    }
    return out;
}

// ---- the Lab display (issue #87: build once, render many) -----------------
// One row of the Research Lab, pre-built so the per-frame render does no
// string building. `line1` is the primary line (the home-calendar bank stamp
// + the experiment name); `line2` the dimmed provenance sub-line (when it was
// run, by whom, on which ship; "" = a free-EVA kerbal), empty when there is
// none to show. Kept as a small struct rather than a bare string because the
// row has two lines of different weight.
struct LabEntry {
    std::string line1;
    std::string line2;
};

// The Lab's rows for a log of banks: one entry per bank, in bank order. `cal`
// is the home calendar the stamps are drawn in; a 0 calendar (no home yet)
// degrades to name-only, matching the live path. This is the single source
// for the Lab's text -- the render just walks it, and a future pagination is
// a slice of the result and a sort a reorder (issue #90's GC is what bounds
// the length). Pure: takes the log + a calendar, returns display strings, so
// it is testable without a Game.
inline std::vector<LabEntry> labEntries(const std::vector<Experiment> &recovered,
                                        const Calendar &cal) {
    std::vector<LabEntry> out;
    out.reserve(recovered.size());
    for(const Experiment &e : recovered) {
        LabEntry r;
        char stamp[64];
        if(e.recovered_at > 0.0 &&
           fmt_cal_compact(cal, e.recovered_at, stamp, sizeof stamp)) {
            r.line1 = stamp;
            r.line1 += "  ";
        }
        r.line1 += experimentName(e);
        if(e.ran_at > 0.0 &&
           fmt_cal_compact(cal, e.ran_at, stamp, sizeof stamp)) {
            r.line2 = "ran ";
            r.line2 += stamp;
        }
        if(!e.kerbal.empty()) {
            if(!r.line2.empty()) { r.line2 += "   "; }
            r.line2 += e.kerbal;
        }
        if(!e.ship.empty()) {
            if(!r.line2.empty()) { r.line2 += "   "; }
            r.line2 += e.ship;
        } else if(!e.kerbal.empty()) {
            r.line2 += "   (EVA)";
        }
        out.push_back(std::move(r));
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
