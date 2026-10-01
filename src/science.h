// science.h -- experiments, the career score, and the value model.
// Header-only pure C++ so tests can pin the value curve without the game.
//
// Experiment identity = type + body + situation (+ biome where the family is
// biome-specific); provenance is extra data that == ignores. Recovering
// scores via scoreOf, halved per earlier bank of the same key (floor 1).

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "calendar.h"   // Calendar + fmt_cal_compact (the Lab row's stamps)

/* Where the experiment happens. FlyingLow/FlyingHigh split the atmosphere at
   kFlyingLowFrac; LowOrbit/HighOrbit split at orbitCutAlt. Altitude-based. */
enum class SciSituation : unsigned char {
    Landed,
    FlyingLow,
    FlyingHigh,
    LowOrbit,
    HighOrbit,
};

/* An experiment family: value, situations it runs in, and where the biome is
   part of the identity (only where you can tell biomes apart). */
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
// Unknown family -> null (fallback: base 10, biome-specific everywhere).
inline const ExperimentDef *defFor(const std::string &type) {
    for(const ExperimentDef &d : experimentDefs()) {
        if(d.type == type) { return &d; }
    }
    return nullptr;
}

inline int baseValue(const std::string &type) {
    const ExperimentDef *d = defFor(type);
    return (d != nullptr) ? d->base_value : 10;
}
// Unknown family -> yes in every situation (never silently merge findings).
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
    // identity -- the uniqueness key; == compares ONLY these
    std::string type = "observation";
    std::string body;
    SciSituation situation = SciSituation::Landed;
    std::string biome;
    // provenance -- == ignores these
    double ran_at = 0.0;
    double recovered_at = 0.0; // 0 = not yet banked
    std::string kerbal;
    std::string ship;          // "" = free EVA

    // == is key-only on purpose: two runs of the same observation dedup to
    // one bank. Biome is in the key only where the family is biome-specific.
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

// "Landed observation of Midlands on Mun" -- "of <Biome>" only where the
// family is biome-specific and the finding has a biome.
inline std::string experimentName(const Experiment &e) {
    std::string n = capitalizeFirst(situationName(e.situation)) + " " + e.type;
    if(biomeSpecificIn(e.type, e.situation) && !e.biome.empty()) {
        n += " of " + capitalizeFirst(e.biome);
    }
    n += " on " + e.body;
    return n;
}

// How a part stores findings (PartDef.experiment_storage). Capacity is per
// experiment FAMILY. Instrument = producer (own family, 1); Courier = 1 per
// family; Container = unlimited per family; None = holds nothing.
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

// Count of the same key in a log (== is key-only) -- the diminishing-returns prevCount.
inline int countKey(const std::vector<Experiment> &v, const Experiment &e) {
    int n = 0;
    for(const Experiment &x : v) {
        if(x == e) { ++n; }
    }
    return n;
}

// How many findings of the same FAMILY a part holds -- the per-family ceiling.
inline int countFamily(const std::vector<Experiment> &v,
                       const std::string &type) {
    int n = 0;
    for(const Experiment &x : v) {
        if(x.type == type) { ++n; }
    }
    return n;
}

// Can a part with `role` (and, for an instrument, its own `ownFamily`) accept
// finding `e`? Pure so the ceiling is testable without a Part.
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

// Landed is the baseline; flying a little more; orbit the most.
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

// Home body is 1.0; every other body is the `frontier` weight.
inline constexpr double kFrontierWeight = 2.0;

/* scoreOf: full value on first recovery, each repeat halves it (floor 1). */
inline int scoreOf(const Experiment &e, int prevCount, double bodyWeight) {
    int v = (int)std::lround(baseValue(e.type) * situationWeight(e.situation)
                             * bodyWeight);
    v = std::max(1, v);
    for(int i = 0; i < prevCount && v > 1; i++) { v /= 2; }
    return v;
}

/* Career: score + append-only bank log. `version` is bumped on EVERY mutation
   so derived caches (the Lab's rows) can detect staleness. */
struct Career {
    int score = 0;
    std::vector<Experiment> recovered;
    std::size_t version = 0;

    // The (N+1)th bank of a key scores scoreOf(e, N, bodyWeight).
    int recover(const Experiment &e, double bodyWeight) {
        const int prev = countKey(recovered, e);
        const int gained = scoreOf(e, prev, bodyWeight);
        recovered.push_back(e);
        score += gained;
        ++version;
        return gained;
    }

    // Bulk write (a Load) -- one entry point so the version bump is not forgotten.
    void setFrom(int newScore, std::vector<Experiment> newLog) {
        score = newScore;
        recovered = std::move(newLog);
        ++version;
    }

    // New Game / unload: clears + bumps so a Lab cache is detected stale.
    void reset() {
        score = 0;
        recovered.clear();
        ++version;
    }
};

/* One recovery outcome: points split into fresh vs repeat, for the Flight Summary. */
struct RecoverSummary {
    int gained = 0;
    int repeat = 0;
    std::vector<Experiment> fresh;
};

/* Bank a bag of experiments. Deduped by key first so ONE observation held by
   N kerbals banks ONCE (without this, N crew = a farming exploit). */
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

// ---- the Lab display ------------------------------------------------------
// Pre-built Research Lab row (two lines: name + dimmed provenance).
struct LabEntry {
    std::string line1;
    std::string line2;
};

// One entry per bank. A 0 calendar degrades to name-only.
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
// Flying-low is the bottom slice of the atmosphere (KSP-style).
inline constexpr double kFlyingLowFrac = 0.2;

// Low/high ORBIT cut: the higher of the near-body SoI edge and the atmosphere
// top, plus a margin. `soi` is the NEAR-BODY (rotating-frame) SoI, not the
// inertial orbital sphere (that one would make high-orbit nearly unreachable).
inline constexpr double kOrbitCutMargin = 10e3;
inline double orbitCutAlt(double soi, double radius, double atmoTop) {
    const double edge = std::max(soi - radius, atmoTop);
    return edge + kOrbitCutMargin;
}

/* Situation from live state: grounded -> Landed; air -> FlyingLow/High; above
   atmo -> Low/HighOrbit. Purely altitude-based (no "falling" situation).
   Edge: isGrounded() is a tolerance, so a slow low-hover reads as Landed. */
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
