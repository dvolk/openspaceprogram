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

/* Four dials on the cast's look, and only these:
   kKerbalGreenMix  how much of the species green every suit wears -- the dial
                    for "varied" against "all kerbal-green";
   kKerbalHueArc    how much of the colour wheel that hue may draw from;
   sat, val below   how colourful, how bright.
   Hue is what tells two suits apart, so it gets as much of the wheel as the
   mix can survive -- which is the whole question, because the mix has a cost
   and the arc is what pays it.

   The mix is a LERP weight, not a scale: 0 is the raw picked hue (reads as
   candy), 1 is everyone in the texture's own green (reads as one clone). It is
   what makes a crew look like one species with different complexions -- sage
   and khaki, not neon. Its cost: a hue near the green's COMPLEMENT cancels
   toward grey. The green sits at 120 degrees, so the danger band is centred on
   300 and runs roughly 269-332 across these sat/val ranges. Given the whole
   wheel at 0.35, id 3 (Corvin) lands on #7a7473 at chroma 0.056 -- a suit the
   colour of concrete -- which is what the chroma floor in
   tests/test_roster.cpp refuses.

   The arc stops the draw at 288 degrees, which is 12 degrees SHORT of the
   complement rather than clear of the band, so it buys range rather than
   immunity: worst chroma measures 0.212 through id 64, 0.155 through 256,
   0.102 through 1024, because {id*phi} is dense and walks back over the edge
   (id 809). Skipping a band CENTRED on 300 instead -- hue = frac(0.933 + 0.8 *
   unit(id*phi)) -- holds 0.24 out past id 4096 and separates the cast better
   (0.17 against 0.14). It was not taken because it also lifts the cast's
   WORST chroma from 0.23 to 0.48, which is the candy level the player backed
   away from. Past a couple hundred named kerbals that trade is worth
   revisiting; until then the floor is scoped to 64 ids, not to infinity. */
const float kKerbalGreenMix = 0.35f;
const float kKerbalHueArc = 0.8f;

/* The three picks come from a Kronecker sequence -- the fractional parts of id
   times three irrationals. Roster ids are small and consecutive, and a hash of
   them still lands some pairs a hair apart in every channel; this sequence
   spreads consecutive integers near-optimally, so 8 kerbals leave 0.14 of
   channel separation, a 32-kerbal cast 0.06 and a 64-kerbal one 0.02. Being
   pure arithmetic, a shade is reproducible from the id alone: nothing to carry
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
    const float hue = kKerbalHueArc * (float)unit(d * std::numbers::phi_v<double>);
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
