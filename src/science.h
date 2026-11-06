// science.h -- experiments, the career score, and the value model.
// Header-only (types + pure value model) so tests pin the curve without the
// game; the family TABLE is data: res/data/experiments.json.
//
// Experiment identity = type + body + situation (+ biome where the family is
// biome-specific); provenance is extra data that == ignores. Recovering
// scores via scoreOf, halved per earlier bank of the same key (floor 1).

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>    // snprintf (formatMeasurement, #665)
#include <map>
#include <string>
#include <vector>

#include "calendar.h"   // Calendar + fmt_cal_compact (the Lab row's stamps)
#include "constants.h"  // kFlyingLowFrac, kOrbitCutMargin

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
    /* What the family MEASURES, "" when nothing does (#665). A family is a
       measurement only when an instrument part is behind it: a kerbal's
       "observation" happens in the air but nobody in it read a gauge, so the
       run carries no number. Data, so adding an instrument stays a res/ edit;
       the loader rejects anything but the two names below. */
    std::string measures;                     // "" | "pressure" | "temperature"
    bool validIn(SciSituation s) const {
        return std::find(valid_in.begin(), valid_in.end(), s) != valid_in.end();
    }
    bool biomeSpecificIn(SciSituation s) const {
        return std::find(biome_specific_in.begin(), biome_specific_in.end(), s)
            != biome_specific_in.end();
    }
};

// The loaded family table, set once at startup by loadExperimentDefs
// (science.cpp) -- adding an instrument is a res/ edit, not a recompile.
// Unknown families fall back through the helpers below: base 10,
// biome-specific + valid everywhere.
inline std::vector<ExperimentDef> &experimentDefs() {
    static std::vector<ExperimentDef> defs;
    return defs;
}
// Throws std::runtime_error naming the file + offending field on any
// bad/missing data.
void loadExperimentDefs(const char *path);
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
// Unknown family -> yes everywhere (never silently merge findings).
inline bool biomeSpecificIn(const std::string &type, SciSituation s) {
    const ExperimentDef *d = defFor(type);
    return (d != nullptr) ? d->biomeSpecificIn(s) : true;
}
inline bool experimentValidIn(const std::string &type, SciSituation s) {
    const ExperimentDef *d = defFor(type);
    return (d == nullptr) || d->validIn(s);
}

/* What an instrument part read at the moment of the run (#665). PROVENANCE, not
   identity: == ignores it, so a barometer run at 12 kPa and one at 20 kPa are
   still the same finding and still dedup to one bank. If a measured number ever
   wants to be worth something, the family table is where that gets said (a
   spread on base_value), not here.
   `has` is not the same question as `pressure == 0.0`: vacuum IS 0 Pa, and a
   family that measures nothing (a kerbal observation) has no reading at all.
   ALL of it is stored together even though a barometer prints only its half --
   the air state is one measurement at one instant, and a reading with no place
   attached is not repeatable. lon/lat are the near body's ROTATING-frame
   azimuth (equirect.h, the same convention the surface map uses), so a bank log
   can be plotted. A hull in orbit moves, which is why the pair is ran_at +
   lon/lat rather than lon/lat alone. */
struct Measurement {
    bool has = false;
    double pressure = 0.0;      // Pa, at the part's own altitude
    double temperature = 0.0;   // K
    double alt_asl = 0.0;       // m above the body's sea-level datum
    double lon = 0.0;           // rad in [0, 2pi) -- equirect.h
    double lat = 0.0;           // rad in [-pi/2, pi/2]
};

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
    Measurement meas;          // what the instrument read, if any (#665)

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

/* The recorded number, as ONE spelling every reader prints: the part window's
   finding rows, the Lab's provenance line, the Flight Summary and the [science]
   log. SI only (kPa, K) because a log line gets parsed and a second unit in a
   field is a parser's bad day -- the same rule --drag-log's air columns follow.
   A window that wants a Celsius gloss adds its own. Answers an empty string for
   a family that measures nothing, so a caller prints the finding name alone. */
inline void formatMeasurement(const Experiment &e, char *buf, size_t n) {
    if(buf == nullptr || n == 0) { return; }
    buf[0] = '\0';
    if(!e.meas.has) { return; }
    const ExperimentDef *d = defFor(e.type);
    const std::string what = (d != nullptr) ? d->measures : std::string();
    if(what == "pressure") {
        snprintf(buf, n, "%.1f kPa", e.meas.pressure / 1000.0);
    } else if(what == "temperature") {
        snprintf(buf, n, "%.1f K", e.meas.temperature);
    }
}

/* The same reading with a gap in front of it, for a line that already ends in the
   finding's name ("... on Kerbin  89.3 kPa"), and "" when the family measured
   nothing -- so every reader (the [science] log, the part window, the Lab, the
   Flight Summary) appends ONE string and none of them can drift. */
inline std::string measured(const Experiment &e) {
    char buf[64];
    formatMeasurement(e, buf, sizeof buf);
    return buf[0] != '\0' ? std::string("  ") + buf : std::string();
}

// How a part stores findings (PartDef.experiment_storage). Capacity is per
// experiment FAMILY: Instrument = producer (own family, 1), Courier = 1 per
// family, Container = unlimited, None = nothing.
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

// Body weight = TerrainBody::science_mult (system JSON, hand-editable).
// Unknown body -> 1.0 (home baseline).
inline double bodyWeightOf(const std::map<std::string, double> &bodyMult,
                           const std::string &body) {
    auto it = bodyMult.find(body);
    return (it != bodyMult.end()) ? it->second : 1.0;
}

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

/* Deduped by key first, so ONE observation held by N kerbals banks ONCE
   (without this, N crew = a farming exploit).
   bodyMult: body name -> science_mult (see bodyWeightOf). */
inline RecoverSummary recoverMany(Career &c, const std::vector<Experiment> &loot,
                                  const std::map<std::string, double> &bodyMult,
                                  double now) {
    RecoverSummary out;
    std::vector<Experiment> uniq;
    for(const Experiment &e : loot) {
        if(!holdsExperiment(uniq, e)) { uniq.push_back(e); }
    }
    for(const Experiment &e0 : uniq) {
        Experiment e = e0;
        e.recovered_at = now;   // stamp the bank time on the logged entry
        const double bw = bodyWeightOf(bodyMult, e.body);
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

// The body's research weight (science_mult) as a plain-english tier; the
// exact value stays available (AtlasRow::valueExact) for a tooltip.
inline const char *valueWord(double v) {
    if(v < 1.1) { return "routine"; }
    if(v < 1.3) { return "modest"; }
    if(v < 1.5) { return "moderate"; }
    if(v < 1.7) { return "notable"; }
    if(v < 1.9) { return "significant"; }
    if(v < 2.2) { return "substantial"; }
    if(v < 2.6) { return "exceptional"; }
    return "extraordinary";
}

// One System Atlas dossier line (see buildAtlasRows): a label/value pair, or
// a section title when header is set. Pre-built as display strings, so
// --atlas-dump and the window cannot disagree.
struct AtlasFact {
    std::string label;
    std::string value;
    bool header = false;
};

// One pre-built System Atlas row (the lab renders these; see buildAtlasRows
// for the discovered% definition and its limits).
struct AtlasRow {
    std::string rawName;      // the body's plain name (selection key)
    std::string name;         // indented + "(home)" marker (display)
    std::string valueWord;    // "notable" etc.
    double valueExact = 0.0;  // the raw science_mult (tooltip)
    long dv = 0;              // approach Δv (rounded to 50), or 0
    int discovered = 0;       // 0-100
    std::vector<AtlasFact> facts;
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
        // What the instrument read, when the family had one (#665). It rides the
        // name line because a Lab full of findings is a table you scan for numbers.
        r.line1 += measured(e);
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

// Low/high ORBIT cut [m ASL]: the near-body shell edge plus a margin. `soi` is
// the NEAR-BODY (rotating-frame) SoI, not the inertial orbital sphere (that
// would make high-orbit nearly unreachable); `sea_level_radius` is the body's
// seaLevelRadius() -- the loader puts the shell on that sphere (system.cpp),
// so the cut sits at the same datum as every other altitude. The shell always
// contains the atmosphere (bodylimits.h shellEdge), so no atmoTop term is needed.
inline double orbitCutAlt(double soi, double sea_level_radius) {
    // Both args are double, so a swap compiles and makes the cut hugely
    // negative (everything reads HighOrbit). The SoI is always outside the body.
    assert(soi >= sea_level_radius);
    return (soi - sea_level_radius) + kOrbitCutMargin;
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
