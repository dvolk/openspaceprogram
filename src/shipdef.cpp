#include "shipdef.h"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

PartDef::PartDef()
    : mass(0.0), torque(0.0), fuel_rate(0.0), exhaust_velocity(0.0) {
    capacity.resize((int)ResourceType::Num, 0.0f);
}

const PartDef *PartsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].name == name) { return &parts[i]; }
    }
    return nullptr;
}

static int resource_index_from_string(const std::string &s, const char *ctx) {
    if(s == "hydrogen") { return (int)ResourceType::Hydrogen; }
    if(s == "lox") { return (int)ResourceType::LOX; }
    if(s == "ec") { return (int)ResourceType::EC; }
    if(s == "oxygen") { return (int)ResourceType::Oxygen; }
    if(s == "water") { return (int)ResourceType::Water; }
    if(s == "food") { return (int)ResourceType::Food; }
    throw std::runtime_error(std::string(ctx) + ": unknown resource '" + s
                             + "' (expected: hydrogen, lox, ec, oxygen, water, food)");
}

PartsCatalog load_parts_catalog(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("parts: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("parts: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("parts: no parts in ") + path);
    }

    PartsCatalog cat;
    const nlohmann::json &arr = doc["parts"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        PartDef d;
        d.name = pv.value("name", std::string(""));
        if(d.name.empty()) {
            throw std::runtime_error(std::string("parts: entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"name\"");
        }
        const char *ctx = (std::string("parts: ") + d.name + ": ").c_str();
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(std::string(ctx) + "duplicate part name");
        }

        d.type = pv.value("type", std::string(""));   // free-form label (display only)
        d.mesh = pv.value("mesh", std::string(""));
        d.texture = pv.value("texture", std::string(""));
        if(d.mesh.empty() || d.texture.empty()) {
            throw std::runtime_error(std::string(ctx) + "missing \"mesh\"/\"texture\"");
        }
        d.mass = pv.value("mass", -1.0);
        if(d.mass <= 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"mass\" must be > 0 (kg)");
        }

        /* Behavior is field-driven (see shipdef.h): each optional field is
           validated on its own, and they combine freely. A part with none
           of them is a passive mass (e.g. a bare capsule). */
        d.torque = pv.value("torque", 0.0);
        if(d.torque < 0.0) {
            throw std::runtime_error(std::string(ctx) + "\"torque\" must be >= 0 (N m)");
        }

        bool has_rate = pv.contains("fuel_rate");
        bool has_ve = pv.contains("exhaust_velocity");
        d.fuel_rate = pv.value("fuel_rate", 0.0);
        d.exhaust_velocity = pv.value("exhaust_velocity", 0.0);
        if(has_rate != has_ve) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"fuel_rate\" and \"exhaust_velocity\" must be given together");
        }
        if(has_rate && (d.fuel_rate <= 0.0 || d.exhaust_velocity <= 0.0)) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"fuel_rate\" and \"exhaust_velocity\" must be > 0");
        }

        if(pv.contains("capacity")) {
            if(!pv["capacity"].is_object() || pv["capacity"].empty()) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must be a non-empty object");
            }
            for(auto it = pv["capacity"].begin(); it != pv["capacity"].end(); ++it) {
                d.capacity[resource_index_from_string(it.key(), ctx)] =
                    it.value().get<float>();
            }
            double cap_total = 0.0;
            for(size_t r = 0; r < d.capacity.size(); r++) { cap_total += d.capacity[r]; }
            if(cap_total <= 0.0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must total > 0 (kg)");
            }
        }

        cat.parts.push_back(d);
    }
    return cat;
}

ShipDef load_ship_def(const char *path, const PartsCatalog &catalog) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("ship: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("ship: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("ship: no parts in ") + path);
    }

    ShipDef def;
    def.name = doc.value("name", std::string(""));
    def.controller = -1;
    if(doc.contains("controller")) {
        def.controller = doc["controller"].get<int>();
    }

    const nlohmann::json &arr = doc["parts"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        ShipPart sp;
        sp.def = nullptr;
        sp.part = pv.value("part", std::string(""));
        sp.offset = pv.value("offset", -1.0);
        if(sp.part.empty()) {
            throw std::runtime_error(std::string("ship: part entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"part\"");
        }
        sp.def = catalog.find(sp.part);
        if(sp.def == nullptr) {
            std::string avail;
            for(size_t k = 0; k < catalog.parts.size(); k++) {
                if(k) { avail += ", "; }
                avail += catalog.parts[k].name;
            }
            throw std::runtime_error(std::string("ship: unknown part '") + sp.part
                                     + "' in " + path + " (catalog has: " + avail + ")");
        }
        if(sp.offset < 0.0) {
            throw std::runtime_error(std::string("ship: part '") + sp.part + "' in " + path
                                     + ": \"offset\" must be >= 0 (m)");
        }
        def.parts.push_back(sp);
    }

    if(def.controller >= (int)def.parts.size()) {
        throw std::runtime_error(std::string("ship: controller index ") + std::to_string(def.controller)
                                 + " in " + path + " out of range (parts: " + std::to_string(def.parts.size()) + ")");
    }
    return def;
}
