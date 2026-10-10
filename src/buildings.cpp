// buildings.cpp -- building catalog + world instances (see buildings.h).
#include "buildings.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <stdexcept>

#include "body.h"     // create_body
#include "equirect.h" // equirectDir
#include "mesh.h"     // get_mesh
#include "physics.h"  // setPosRot
#include "resdir.h"   // resdir::path
#include "system.h"   // System
#include "terrain.h"  // TerrainBody, StaticBuilding
#include "texture.h"  // get_texture
#include "vehicle.h"  // faceAlong

const BuildingDef *BuildingsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < buildings.size(); i++) {
        if(buildings[i].name == name) { return &buildings[i]; }
    }
    return nullptr;
}

BuildingsCatalog load_buildings_catalog(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("buildings: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("buildings: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("buildings") || !doc["buildings"].is_array()
       || doc["buildings"].empty()) {
        throw std::runtime_error(std::string("buildings: no buildings in ") + path);
    }

    BuildingsCatalog cat;
    const nlohmann::json &arr = doc["buildings"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &bv = arr[i];
        BuildingDef d;
        d.name = bv.value("name", std::string(""));
        if(d.name.empty()) {
            throw std::runtime_error(std::string("buildings: entry ")
                                     + std::to_string(i) + " of " + path
                                     + ": missing \"name\"");
        }
        const std::string ctx = "buildings: " + d.name + ": ";
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(ctx + "duplicate building name");
        }
        d.display_name = bv.value("display_name", std::string(""));
        d.mesh = bv.value("mesh", std::string(""));
        d.texture = bv.value("texture", std::string(""));
        if(d.mesh.empty() || d.texture.empty()) {
            throw std::runtime_error(ctx + "missing \"mesh\"/\"texture\"");
        }
        cat.buildings.push_back(d);
    }
    return cat;
}

bool WorldBuilding::hasFlag(const std::string &f) const {
    for(size_t i = 0; i < flags.size(); i++) {
        if(flags[i] == f) { return true; }
    }
    return false;
}

std::string WorldBuilding::label(const BuildingsCatalog &cat) const {
    if(!display_name.empty()) { return display_name; }
    if(const BuildingDef *d = cat.find(type)) { return d->label(); }
    return type;
}

glm::dvec3 WorldBuilding::dir() const {
    const double lon = lon_deg * std::numbers::pi / 180.0;
    const double lat = lat_deg * std::numbers::pi / 180.0;
    return equirectDir(lon, lat);
}

std::vector<WorldBuilding> parse_world_buildings(const nlohmann::json &doc,
                                                 const std::string &path) {
    std::vector<WorldBuilding> out;
    if(!doc.contains("world_buildings")) { return out; }
    const nlohmann::json &arr = doc["world_buildings"];
    if(!arr.is_array()) {
        throw std::runtime_error(std::string("system: \"world_buildings\" in ")
                                 + path + " must be an array");
    }
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("system: world_buildings entry ")
                                     + std::to_string(i) + " of " + path
                                     + " must be an object");
        }
        WorldBuilding w;
        w.id = ev.value("id", std::string(""));
        w.type = ev.value("type", std::string(""));
        w.display_name = ev.value("display_name", std::string(""));
        w.body = ev.value("body", std::string(""));
        const std::string ctx = "system: world_buildings[" + std::to_string(i)
                                + "] of " + path + ": ";
        if(w.id.empty()) { throw std::runtime_error(ctx + "missing \"id\""); }
        if(w.type.empty()) { throw std::runtime_error(ctx + "missing \"type\""); }
        if(w.body.empty()) { throw std::runtime_error(ctx + "missing \"body\""); }
        for(size_t k = 0; k < out.size(); k++) {
            if(out[k].id == w.id) {
                throw std::runtime_error(ctx + "duplicate id '" + w.id + "'");
            }
        }
        if(!ev.contains("lat_deg") || !ev["lat_deg"].is_number()
           || !ev.contains("lon_deg") || !ev["lon_deg"].is_number()) {
            throw std::runtime_error(ctx + "needs numeric \"lat_deg\"/\"lon_deg\"");
        }
        w.lat_deg = ev["lat_deg"].get<double>();
        w.lon_deg = ev["lon_deg"].get<double>();
        if(w.lat_deg < -90.0 || w.lat_deg > 90.0) {
            throw std::runtime_error(ctx + "\"lat_deg\" must be in [-90, 90]");
        }
        w.height_m = 5.0;
        w.heading_deg = 0.0;
        if(ev.contains("height_m")) {
            if(!ev["height_m"].is_number()) {
                throw std::runtime_error(ctx + "\"height_m\" must be a number");
            }
            w.height_m = ev["height_m"].get<double>();
        }
        if(ev.contains("heading_deg")) {
            if(!ev["heading_deg"].is_number()) {
                throw std::runtime_error(ctx + "\"heading_deg\" must be a number");
            }
            w.heading_deg = ev["heading_deg"].get<double>();
        }
        if(ev.contains("flags")) {
            if(!ev["flags"].is_array()) {
                throw std::runtime_error(ctx + "\"flags\" must be an array of strings");
            }
            for(const nlohmann::json &fv : ev["flags"]) {
                if(!fv.is_string() || fv.get<std::string>().empty()) {
                    throw std::runtime_error(ctx + "\"flags\" entries must be non-empty strings");
                }
                w.flags.push_back(fv.get<std::string>());
            }
        }
        out.push_back(w);
    }
    return out;
}

glm::dmat3 buildingOrient(const glm::dvec3 &up, double heading_rad) {
    const glm::dmat3 base = faceAlong(up);
    const double c = std::cos(heading_rad), s = std::sin(heading_rad);
    // Spin about the building's local +Z (the up axis).
    return base * glm::dmat3(glm::dvec3(c, s, 0.0),
                             glm::dvec3(-s, c, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
}

void place_world_buildings(System &sys, const BuildingsCatalog &cat,
                           Shader *shader, TerrainBody *sun) {
    for(const WorldBuilding &w : sys.world_buildings) {
        const BuildingDef *def = cat.find(w.type);
        if(def == nullptr) {
            std::string avail;
            for(size_t i = 0; i < cat.buildings.size(); i++) {
                if(i) { avail += ", "; }
                avail += cat.buildings[i].name;
            }
            throw std::runtime_error("buildings: '" + w.id + "' has unknown type '"
                                     + w.type + "' (catalog has: " + avail + ")");
        }
        TerrainBody *hb = sys.find(w.body);
        if(hb == nullptr) {
            throw std::runtime_error("buildings: '" + w.id + "' names unknown body '"
                                     + w.body + "'");
        }
        for(auto *p : hb->pads) {
            if(p->id == w.id) {
                throw std::runtime_error("buildings: duplicate instance id '"
                                         + w.id + "' on " + hb->name);
            }
        }
        const glm::dvec3 up = w.dir();
        Mesh *m = get_mesh(std::string("res/") + def->mesh);
        Texture *t = get_texture(std::string("res/") + def->texture);
        StaticBuilding *sp = new StaticBuilding;
        sp->body = create_body(m, shader, t, 0, 0, 0, 0);
        sp->parent = hb;
        sp->sun = sun;
        sp->id = w.id;
        sp->display_name = w.label(cat);
        sp->flags = w.flags;
        sp->dir = up;
        sp->height_m = w.height_m;
        sp->heading_deg = w.heading_deg;
        const double heading = w.heading_deg * std::numbers::pi / 180.0;
        setPosRot(sp->body, sp->origin(), buildingOrient(up, heading));
        hb->pads.push_back(sp);
    }
}

StaticBuilding *find_launch_site(TerrainBody *body, const std::string &siteId,
                                 bool preferPolar) {
    if(body == nullptr) { return nullptr; }
    /* Explicit pick: must be a launch pad. pad-polar only accepts it when the
       site is itself polar, so a sticky equatorial pick cannot pre-empt the
       polar bed (preferPolar falls through to the default). */
    if(!siteId.empty()) {
        for(auto *p : body->pads) {
            if(p->id != siteId) { continue; }
            if(!p->hasFlag("VAB_LAUNCH_PAD")) { return nullptr; }
            if(!preferPolar || p->hasFlag("POLAR_PAD")) { return p; }
            break;
        }
        if(!preferPolar) { return nullptr; }
    }
    StaticBuilding *polar = nullptr, *first = nullptr;
    for(auto *p : body->pads) {
        if(!p->hasFlag("VAB_LAUNCH_PAD")) { continue; }
        if(first == nullptr) { first = p; }
        if(preferPolar && polar == nullptr && p->hasFlag("POLAR_PAD")) {
            polar = p;
        }
    }
    return preferPolar && polar != nullptr ? polar : first;
}
