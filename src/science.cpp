// science.cpp -- the experiment family table (res/data/experiments.json).
// science.h keeps the value model + the unknown-family fallback; this is
// where the DATA lives, so a new instrument is a res/ edit, not a recompile.

#include "science.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

/* Strict situation-id parse. situationFromId is permissive (an old save's
   unknown id degrades to LowOrbit); a JSON typo must not silently become
   one band, so the loader checks the known ids itself. */
static bool parse_situation(const std::string &s, SciSituation &out) {
    for(int i = 0; i <= (int)SciSituation::HighOrbit; i++) {
        if(s == situationId((SciSituation)i)) {
            out = (SciSituation)i;
            return true;
        }
    }
    return false;
}

void loadExperimentDefs(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("experiments: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("experiments: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("experiments")
       || !doc["experiments"].is_array() || doc["experiments"].empty()) {
        throw std::runtime_error(std::string("experiments: no experiments in ") + path);
    }

    std::vector<ExperimentDef> defs;
    const nlohmann::json &arr = doc["experiments"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("experiments: entry ") + std::to_string(i)
                                     + " of " + path + " is not an object");
        }
        ExperimentDef d;
        d.type = ev.value("type", std::string(""));
        if(d.type.empty()) {
            throw std::runtime_error(std::string("experiments: entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"type\"");
        }
        const std::string ctx = "experiments: " + d.type + ": ";
        for(const ExperimentDef &x : defs) {
            if(x.type == d.type) {
                throw std::runtime_error(ctx + "duplicate type");
            }
        }

        if(!ev.contains("base_value") || !ev["base_value"].is_number_integer()
           || ev["base_value"].get<int>() <= 0) {
            throw std::runtime_error(ctx + "\"base_value\" must be an integer > 0");
        }
        d.base_value = ev["base_value"].get<int>();

        auto parse_list = [&](const char *key, std::vector<SciSituation> &out,
                              bool required) {
            if(!ev.contains(key)) {
                if(required) {
                    throw std::runtime_error(std::string(ctx) + std::string("\"") + key
                                             + "\" must be a non-empty array of situation ids");
                }
                return;
            }
            const nlohmann::json &sv = ev[key];
            if(!sv.is_array() || sv.empty()) {
                throw std::runtime_error(std::string(ctx) + std::string("\"") + key
                                         + "\" must be a non-empty array");
            }
            for(const nlohmann::json &v : sv) {
                if(!v.is_string()) {
                    throw std::runtime_error(std::string(ctx) + std::string("\"") + key
                                             + "\" entries must be situation-id strings");
                }
                const std::string id = v.get<std::string>();
                SciSituation s;
                if(!parse_situation(id, s)) {
                    throw std::runtime_error(std::string(ctx) + std::string("\"") + key
                                             + "\": unknown situation id \"" + id
                                             + "\" (expected landed, flying_low, flying_high, "
                                               "low_orbit, high_orbit)");
                }
                out.push_back(s);
            }
        };
        // valid_in is required: an instrument that runs nowhere is a data bug,
        // not a valid def. biome_specific_in may be empty (never biome-bound).
        parse_list("valid_in", d.valid_in, true);
        parse_list("biome_specific_in", d.biome_specific_in, false);

        defs.push_back(std::move(d));
    }
    experimentDefs() = std::move(defs);
}
