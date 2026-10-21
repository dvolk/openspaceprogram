// crew.cpp -- the roster: parsing + lookup (declarations in crew.h).

#include "crew.h"

#include <cmath>      // floor (the Kronecker sequence below)
#include <cstdio>
#include <fstream>
#include <numbers>    // phi / sqrt2 / sqrt3
#include <stdexcept>

#include <nlohmann/json.hpp>

const char *skill_name(Skill s) {
    switch(s) {
        case SkillPiloting:    return "piloting";
        case SkillScience:     return "science";
        case SkillEngineering: return "engineering";
        default: break;
    }
    return "unknown";
}

const KerbalProfile *CrewRoster::find(uint64_t id) const {
    for(const KerbalProfile &p : profiles) {
        if(p.id == id) { return &p; }
    }
    return nullptr;
}

namespace {
/* HSV to RGB with h in [0,1). */
glm::vec3 hsv_rgb(float h, float s, float v) {
    const float seg = h * 6.0f;
    const int i = (int)seg % 6;
    const float f = seg - (float)i;
    const float p = v * (1.0f - s);
    const float q = v * (1.0f - f * s);
    const float t = v * (1.0f - (1.0f - f) * s);
    switch(i) {
        case 0: return glm::vec3(v, t, p);
        case 1: return glm::vec3(q, v, p);
        case 2: return glm::vec3(p, v, t);
        case 3: return glm::vec3(p, q, v);
        case 4: return glm::vec3(t, p, v);
        default: return glm::vec3(v, p, q);
    }
}
}

/* Three dials on the cast's look, and only these:
   kKerbalGreenMix  how much of the species green every suit wears -- the dial
                    for "varied" against "all kerbal-green";
   sat, val below   how colourful, how bright.
   Hue is left to run the whole wheel: it is what tells two suits apart, so
   narrowing it is the one change that costs separation.

   A LERP weight, not a scale: 0 is the raw picked hue (reads as candy), 1 is
   everyone in the texture's own green (reads as one clone). The mix is what
   makes a crew look like one species with different complexions -- sage and
   khaki, not neon. It has one known cost, and at 0.35 it reaches the shipped
   cast: a hue near magenta is the green's COMPLEMENT, so the mix cancels it
   toward grey -- id 3 (Corvin) lands on #7a747a, chroma 0.06, and id 11 joins
   it. That is what the chroma floor in tests/test_roster.cpp is for, and it is
   red right now. The fix is NOT to raise the mix back: stopping the hue draw
   short of the complementary band costs nothing -- hue = 0.8 * unit(...) keeps
   every shade of a 64-kerbal cast above 0.21 chroma and measures BETTER on
   separation (0.143 for the cast against 0.128 with the whole wheel). */
const float kKerbalGreenMix = 0.35f;

/* The three picks come from a Kronecker sequence -- the fractional parts of id
   times three irrationals. Roster ids are small and consecutive, and a hash of
   them still lands some pairs a hair apart in every channel; this sequence
   spreads consecutive integers near-optimally, so 8 kerbals leave 0.10 of
   channel separation and a 32-kerbal cast still leaves 0.05. Being pure
   arithmetic, a shade is reproducible from the id alone: nothing to carry
   through the roster file, the save, or a rebuilt world -- though only within
   one -march setting, since the arithmetic below contracts to an FMA on avx2
   targets. The float32 result never changes the printed byte (checked for ids
   1..500), so that is a note, not a hazard.

   Hue takes the phi axis, so it gets the best-spread sequence. Saturation and
   value stay in narrow bands: below 0.45 a suit reads as grey concrete, above
   0.9 as a neon sticker, and the shader's dark side (min_light 0.15) crushes a
   low-value suit into the background. Past id ~2^51 the products leave the
   double's exact range and the fraction collapses to zero, so a hand-edited
   roster could invent a duplicate shade. */
glm::vec3 kerbal_complexion(uint64_t id) {
    if(id == 0) { return kKerbalBaseGreen; }   // anonymous: the texture's own green
    const double d = (double)id;
    auto unit = [](double x) { return x - std::floor(x); };   // 0..1
    const float hue = (float)unit(d * std::numbers::phi_v<double>);
    const float sat = 0.45f + 0.40f * (float)unit(d * std::numbers::sqrt2_v<double>);
    const float val = 0.55f + 0.30f * (float)unit(d * std::numbers::sqrt3_v<double>);
    const glm::vec3 pick = hsv_rgb(hue, sat, val);
    return pick * (1.0f - kKerbalGreenMix) + kKerbalBaseGreen * kKerbalGreenMix;
}

/* The suit texture is one flat green, so the tint is whatever turns that green
   into the complexion -- which is why a blue or a crimson kerbal costs the
   shader a multiplier over 3 on one channel and under 0.3 on another. */
glm::vec3 kerbal_tint(uint64_t id) {
    return kerbal_complexion(id) / kKerbalBaseGreen;
}

const KerbalProfile *CrewRoster::byName(const std::string &name) const {
    for(const KerbalProfile &p : profiles) {
        if(p.name == name) { return &p; }
    }
    return nullptr;
}

CrewRoster load_crew_roster(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("kerbals: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("kerbals: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("kerbals") || !doc["kerbals"].is_array()
       || doc["kerbals"].empty()) {
        throw std::runtime_error(std::string("kerbals: no kerbals in ") + path);
    }

    CrewRoster roster;
    const nlohmann::json &arr = doc["kerbals"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];
        if(!pv.is_object()) {
            throw std::runtime_error("kerbals: entry " + std::to_string(i)
                                     + " of " + path + " is not an object");
        }

        KerbalProfile p;
        /* Type-checked, not value(): a number or object under "name" raises
           nlohmann's own type_error, which names neither the file nor the entry. */
        if(!pv.contains("name") || !pv["name"].is_string()) {
            throw std::runtime_error("kerbals: entry " + std::to_string(i)
                                     + " of " + path + ": \"name\" must be a string");
        }
        p.name = pv["name"].get<std::string>();
        if(p.name.empty()) {
            throw std::runtime_error("kerbals: entry " + std::to_string(i)
                                     + " of " + path + ": \"name\" is empty");
        }
        const std::string ctx = "kerbals: " + p.name + ": ";

        /* The id is explicit, not the row number: a save names these, so
           inserting or reordering a row must not silently re-point an existing
           save at a different person. */
        if(!pv.contains("id") || !pv["id"].is_number_unsigned()
           || pv["id"].get<uint64_t>() == 0) {
            throw std::runtime_error(ctx + "\"id\" must be a positive integer");
        }
        p.id = pv["id"].get<uint64_t>();
        if(roster.find(p.id) != nullptr) {
            throw std::runtime_error(ctx + "duplicate id");
        }
        if(roster.byName(p.name) != nullptr) {
            throw std::runtime_error(ctx + "duplicate name");
        }

        // Absent = average; a roster entry that says nothing about a skill is
        // fine, one that says 0 or 9 is a typo.
        static const char *kSkillKeys[NumSkills] = { "piloting", "science",
                                                     "engineering" };
        for(int s = 0; s < NumSkills; s++) {
            if(!pv.contains(kSkillKeys[s])) { continue; }
            const nlohmann::json &sv = pv[kSkillKeys[s]];
            /* 3.5 or "five" is a typo in a data file, and value(key, 3) would
               answer it with nlohmann's type_error instead of our message. A
               number too wide for int lands outside 1..5 either way. */
            if(!sv.is_number_integer()) {
                throw std::runtime_error(ctx + std::string("\"") + kSkillKeys[s]
                                         + "\" must be an integer 1..5");
            }
            const long long v = sv.get<long long>();
            if(v < 1 || v > 5) {
                throw std::runtime_error(ctx + std::string("\"") + kSkillKeys[s]
                                         + "\" must be 1..5, got "
                                         + std::to_string(v));
            }
            p.skill[s] = (int)v;
        }
        roster.profiles.push_back(p);
    }
    return roster;
}
