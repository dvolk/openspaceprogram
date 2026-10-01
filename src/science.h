// science.h -- experiments, the career score, and the value model.
// (header-only pure C++)
//
// An Experiment is one run of an observation, recorded by a kerbal on a ship.
// Its IDENTITY -- type + body + situation, plus the biome only in situations
// where the family is biome-specific (ExperimentDef: a crew report in landed
// + flying + low orbit; a materials study in landed + flying; a barometer in
// landed + flying-low) -- is the uniqueness key (named at the UI edge like
// "Landed observation of Midlands on Mun"); the provenance (ran_at,
// recovered_at, kerbal, ship) is extra data that == ignores. Situations (v3):
// Landed (grounded); FlyingLow / FlyingHigh (airborne in the atmosphere --
// the bottom 20% of the air, or the rest); LowOrbit / HighOrbit (above the
// atmosphere, or over an airless body -- split at the body's SoI edge + 10km).
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

/* Where the experiment happens. Landed = grounded on a surface. FlyingLow /
   FlyingHigh = airborne in the atmosphere (the bottom kFlyingLowFrac of the
   air, or the rest). LowOrbit / HighOrbit = above the atmosphere (or over an
   airless body), split at the body's SoI edge + 10km (orbitCutAlt). A ship's
   situation is purely altitude-based: grounded, in the air, or in space. */
enum class SciSituation : unsigned char {
    Landed,
    FlyingLow,
    FlyingHigh,
    LowOrbit,
    HighOrbit,
};

/* An experiment family ("type"): its value, the situations it can be run in,
   and the situations where its finding is biome-specific. A new family is one
   entry in the registry below -- the single home for per-experiment rules
   (they used to be scattered if-chains keyed on the type string). A reading
   is biome-specific only where you can tell biomes apart: a crew report in
   landed + flying + low orbit (high orbit is "of the planet"); a materials
   study in landed + flying; a barometer in landed + flying-low (upper air and
   space give a global "of the planet" reading, though it still RUNS anywhere).
   `valid_in` is the hook for situation-gated instruments (a seismometer:
   landed only). */
struct ExperimentDef {
    std::string type;                        // "observation", "materials study"
    int base_value = 10;                      // the base of scoreOf
    std::vector<SciSituation> valid_in;         // situations it can RUN in
    std::vector<SciSituation> biome_specific_in; // where the biome is part of the identity
    bool validIn(SciSituation s) const {
        return std::find(valid_in.begin(), valid_in.end(), s) != valid_in.end();
    }
    bool biomeSpecificIn(SciSituation s) const {
        return std::find(biome_specific_in.begin(), biome_specific_in.end(), s)
            != biome_specific_in.end();
    }
};

inline const std::vector<ExperimentDef> &experimentDefs() {
    static const std::vector<ExperimentDef> defs = {
        { "observation", 10,
          { SciSituation::Landed, SciSituation::FlyingLow, SciSituation::FlyingHigh,
            SciSituation::LowOrbit, SciSituation::HighOrbit },
          { SciSituation::Landed, SciSituation::FlyingLow, SciSituation::FlyingHigh,
            SciSituation::LowOrbit } },
        { "materials study", 25,
          { SciSituation::Landed, SciSituation::FlyingLow, SciSituation::FlyingHigh,
            SciSituation::LowOrbit, SciSituation::HighOrbit },
          { SciSituation::Landed, SciSituation::FlyingLow, SciSituation::FlyingHigh } },
        { "barometer", 10,
          { SciSituation::Landed, SciSituation::FlyingLow, SciSituation::FlyingHigh,
            SciSituation::LowOrbit, SciSituation::HighOrbit },
          { SciSituation::Landed, SciSituation::FlyingLow } },
    };
    return defs;
}
// The def for a type, or null (an unknown family: base 10, biome-specific
// everywhere, valid everywhere -- the safe fallback so an unregistered type
// behaves like a plain surface instrument, never silently merging findings).
inline const ExperimentDef *defFor(const std::string &type) {
    for(const ExperimentDef &d : experimentDefs()) {
        if(d.type == type) { return &d; }
    }
    return nullptr;
}

// Base value (was the baseValue if-chain). Unknown family -> the base.
inline int baseValue(const std::string &type) {
    const ExperimentDef *d = defFor(type);
    return (d != nullptr) ? d->base_value : 10;
}
// Is the biome part of the finding's identity for `type` in `situation`?
// Unknown family -> yes, in EVERY situation (the KSP common case and the
// pre-registry behavior). A def opts a family OUT per situation; the default
// never merges biome-distinct findings on a guess (that is silent data loss).
inline bool biomeSpecificIn(const std::string &type, SciSituation s) {
    const ExperimentDef *d = defFor(type);
    return (d != nullptr) ? d->biomeSpecificIn(s) : true;
}
// Can this experiment be RUN in `situation`? Unknown family -> yes.
inline bool experimentValidIn(const std::string &type, SciSituation s) {
    const ExperimentDef *d = defFor(type);
    return (d == nullptr) || d->validIn(s);
}

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
    // The biome is in the key only where the family is biome-specific
    // (biomeSpecificIn): a high-orbit crew report is "of the planet" for
    // every biome (KSP: one reading per orbit segment), so two such runs dedup.
    bool operator==(const Experiment &o) const {
        if(type != o.type || body != o.body || situation != o.situation) {
            return false;
        }
        if(!biomeSpecificIn(type, situation)) { return true; }
        return biome == o.biome;
    }
    bool operator!=(const Experiment &o) const { return !(*this == o); }
};

inline const char *situationName(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:     return "landed";
        case SciSituation::FlyingLow:  return "flying low";
        case SciSituation::FlyingHigh: return "flying high";
        case SciSituation::LowOrbit:   return "low orbit";
        case SciSituation::HighOrbit:  return "high orbit";
    }
    return "low orbit";
}

inline const char *situationId(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:     return "landed";
        case SciSituation::FlyingLow:  return "flying_low";
        case SciSituation::FlyingHigh: return "flying_high";
        case SciSituation::LowOrbit:   return "low_orbit";
        case SciSituation::HighOrbit:  return "high_orbit";
    }
    return "low_orbit";
}

inline SciSituation situationFromId(const std::string &id) {
    if(id == "landed")      { return SciSituation::Landed; }
    if(id == "flying_low")  { return SciSituation::FlyingLow; }
    if(id == "flying_high") { return SciSituation::FlyingHigh; }
    if(id == "high_orbit")  { return SciSituation::HighOrbit; }
    return SciSituation::LowOrbit;   // "low_orbit" + unknown (old saves)
}

// ASCII first-letter cap for the display name ("midlands" -> "Midlands").
inline std::string capitalizeFirst(std::string s) {
    if(!s.empty() && s[0] >= 'a' && s[0] <= 'z') {
        s[0] = (char)(s[0] - 'a' + 'A');
    }
    return s;
}

// "Landed observation of Midlands on Mun" -- the "of <Biome>" clause only
// where the family is biome-specific (biomeSpecificIn) AND the finding has a
// biome. A star / banded giant has none, so it reads global "of the body":
// "Low orbit observation on Kerbol".
inline std::string experimentName(const Experiment &e) {
    std::string n = capitalizeFirst(situationName(e.situation)) + " " + e.type;
    if(biomeSpecificIn(e.type, e.situation) && !e.biome.empty()) {
        n += " of " + capitalizeFirst(e.biome);
    }
    n += " on " + e.body;
    return n;
}

// ---- list helpers (a vector of experiments) -------------------------------
// How a part stores findings (PartDef.experiment_storage; the role a
// Part::canHold checks). Capacity is per experiment FAMILY (a part holds N
// findings of each type); no part ever holds the exact same finding twice.
//   Instrument -- 1 finding, its OWN family only; filled by running it, taken
//                 out (a producer, not a container).
//   Courier    -- 1 finding per family, any family; taken and stored (a
//                 kerbal's suit, shuttling findings between parts).
//   Container  -- unlimited per family, any family (a capsule: bulk storage).
//   None       -- holds nothing (most parts; the default).
enum class ExpStorage : unsigned char { None, Instrument, Courier, Container };

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

// How many findings of the same FAMILY a part holds (counted by `type`, not
// subject) -- the per-family capacity ceiling in Part::canHold (a courier
// holds 1 of each type; a container, unlimited).
inline int countFamily(const std::vector<Experiment> &v,
                       const std::string &type) {
    int n = 0;
    for(const Experiment &x : v) {
        if(x.type == type) { ++n; }
    }
    return n;
}

// Can a part with role `role` (and, for an instrument, its own `ownFamily`),
// already holding `held`, accept finding `e`? Pure (Part::canHold delegates
// here) so the ceiling is testable without a Part. The universal rule first:
// never the exact same finding twice (holdsExperiment, == is key-only). Then
// the per-FAMILY ceiling (countFamily):
inline bool canHoldFinding(ExpStorage role, const std::string &ownFamily,
                           const std::vector<Experiment> &held,
                           const Experiment &e) {
    switch(role) {
        case ExpStorage::Instrument:
            if(e.type != ownFamily) { return false; }   // own family only
            break;
        case ExpStorage::Courier:
        case ExpStorage::Container:
            break;   // any family
        case ExpStorage::None:
        default:
            return false;
    }
    if(holdsExperiment(held, e)) { return false; }      // no exact duplicate
    if(role == ExpStorage::Container) { return true; }  // unlimited per family
    return countFamily(held, e.type) < 1;               // Instrument/Courier: 1 per family
}

// ---- the value model ------------------------------------------------------
// (baseValue lives with ExperimentDef above: the per-family base is data, not
// a switch -- "observation" 10, "materials study" 25.)

// Situation weight: the "how hard to be there" factor. Landed is the baseline;
// flying (in the air) is a little more; orbit the most, high orbit the most.
inline double situationWeight(SciSituation s) {
    switch(s) {
        case SciSituation::Landed:     return 1.0;
        case SciSituation::FlyingLow:  return 1.1;
        case SciSituation::FlyingHigh: return 1.2;
        case SciSituation::LowOrbit:   return 1.25;
        case SciSituation::HighOrbit:  return 1.5;
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
// Flying-low is the bottom slice of the atmosphere (KSP-style); the rest of
// the air is flying-high. Named so the 20% lives in one home, not a literal.
inline constexpr double kFlyingLowFrac = 0.2;

/* The low/high ORBIT cut [m above sea level]: the higher of the body's
   near-body SoI edge and its atmosphere top, plus a margin. Both must clear
   -- the low-orbit band is the space just above whichever is the ceiling.
   `soi` is the NEAR-BODY (rotating-frame) SoI, not the inertial orbital
   sphere (that one is hundreds of times bigger and would make the high-orbit
   band nearly unreachable). One home for the policy so the situation label
   and any "is this space?" check cannot disagree. (Kerbin: SoI edge 100km >
   atmo 70km, so 100km + 10km = a 110km cut, and a ship at 85km -- the
   rot-orbit scenario -- is a LOW orbit.) */
inline constexpr double kOrbitCutMargin = 10e3;   // m above the higher edge
inline double orbitCutAlt(double soi, double radius, double atmoTop) {
    const double edge = std::max(soi - radius, atmoTop);
    return edge + kOrbitCutMargin;
}

/* The situation from the vessel's live state. Grounded -> Landed. Airborne in
   the atmosphere (altAsl below the atmo top) -> FlyingLow (the bottom
   kFlyingLowFrac of the air) or FlyingHigh. Above the atmosphere (or over an
   airless body) -> LowOrbit (below orbitCutAlt) or HighOrbit. A ship in
   ascent, a suborbital hop, or a ballistic reentry reads by its altitude: in
   the air it is Flying, in space it is the orbit band -- there is no
   separate "falling" situation. Edge: isGrounded() is a tolerance (COM within
   ~100 m of terrain AND slow), so a slow low-hover reads as Landed.
   (Supersedes the v2 classifier, which read every non-landed ship as an
   "orbit observation" -- issue #75 -- and the v1 situationFromAltitude.) */
inline SciSituation situationFor(bool grounded, double altAsl, double atmoTop,
                                 double orbitCut) {
    if(grounded) { return SciSituation::Landed; }
    if(atmoTop > 0.0 && altAsl < atmoTop) {
        return (altAsl < kFlyingLowFrac * atmoTop)
                   ? SciSituation::FlyingLow
                   : SciSituation::FlyingHigh;
    }
    return (altAsl < orbitCut) ? SciSituation::LowOrbit : SciSituation::HighOrbit;
}
