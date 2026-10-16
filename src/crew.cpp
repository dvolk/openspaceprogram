// crew.cpp -- the roster: parsing + lookup (declarations in crew.h).

#include "crew.h"

#include <cstdio>
#include <fstream>
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
