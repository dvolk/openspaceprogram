#include "fleet.h"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

Fleet load_fleet(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("fleet: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("fleet: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("ships") || !doc["ships"].is_array()
       || doc["ships"].empty()) {
        throw std::runtime_error(std::string("fleet: no ships in ") + path);
    }

    Fleet fleet;
    const nlohmann::json &arr = doc["ships"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("fleet: ship entry ") + std::to_string(i)
                                     + " of " + path + " must be an object");
        }

        FleetEntry e;
        e.ship = ev.value("ship", std::string("res/ships/racer.json"));
        if(e.ship.empty()) {
            throw std::runtime_error(std::string("fleet: ship entry ") + std::to_string(i)
                                     + " of " + path + ": \"ship\" must be a non-empty path");
        }
        e.name = ev.value("name", std::string(""));
        e.body = ev.value("body", std::string(""));
        e.scenario = ev.value("scenario", std::string(""));
        fleet.ships.push_back(e);
    }
    return fleet;
}
