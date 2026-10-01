// ships.cpp -- ship construction + placement (see ships.h).
//
// The ships themselves are owned by the bodies they sit in
// (TerrainBody::ships) or, when aboard, by their ship (Vehicle::crew).
#include "ships.h"

#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "body.h"     // create_body
#include "eva.h"      // Kerbal (the crew characters)
#include "mesh.h"     // get_mesh
#include "physics.h"  // setPosRot
#include "resdir.h"   // resdir::path
#include "shipdef.h"  // load_ship_def, ShipDef, PartsCatalog
#include "system.h"   // System (buildDebugStartShips / spawn_vehicle resolve bodies)
#include "texture.h"  // get_texture
#include "vehicle.h"  // build_ship, faceAlong, spawn_vehicle, scenario_by_name, Vehicle

// Ships sharing a (body, scenario) orbit get this much separation along the
// orbit binormal so they don't spawn on top of each other.
static const double ORBIT_SLOT_SPACING = 20.0;

// Every start-ship entry must name all four of these; a missing or empty one
// is a config error (the game does not guess).
static const char *const kStartShipFields[] = {"ship", "name", "body", "scenario"};

DebugStartShips loadDebugStartShips(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("start ships: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("start ships: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("ships") || !doc["ships"].is_array()
       || doc["ships"].empty()) {
        throw std::runtime_error(std::string("start ships: no ships in ") + path);
    }

    DebugStartShips list;
    const nlohmann::json &arr = doc["ships"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("start ships: entry ")
                                     + std::to_string(i) + " of " + path
                                     + " must be an object");
        }

        DebugStartShip e;
        for(const char *const field : kStartShipFields) {
            if(!ev.contains(field) || !ev[field].is_string()
               || ev[field].get<std::string>().empty()) {
                throw std::runtime_error(std::string("start ships: entry ")
                                         + std::to_string(i) + " of " + path
                                         + " is missing required field \""
                                         + field + "\"");
            }
        }
        e.ship = ev["ship"].get<std::string>();
        e.name = ev["name"].get<std::string>();
        e.body = ev["body"].get<std::string>();
        e.scenario = ev["scenario"].get<std::string>();
        list.ships.push_back(e);
    }
    return list;
}

void collectVehiclesInto(System &sys, std::vector<Vehicle *> &out) {
    out.clear();
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) {
            out.push_back(s);
            for(auto *c : s->crew) { out.push_back(c); }
        }
    }
}

std::vector<Vehicle *> collectVehicles(System &sys) {
    std::vector<Vehicle *> out;
    collectVehiclesInto(sys, out);
    return out;
}

Ships::Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun)
    : part_catalog(load_parts_catalog(resdir::path(parts_file).c_str())),
      partsshader(partsshader),
      sun(sun)
{
}

void Ships::place_pad(TerrainBody *hb, bool polar, const glm::dvec3 &dir, double pad_height)
{
    for(auto *p : hb->pads) {
        if(p->parent == hb && p->polar == polar) { return; }
    }
    const glm::dvec3 start = dir * (double)hb->GetTerrainHeight(dir);
    // The pad's render assets are SHARED (the registries own them).
    Mesh *m = get_mesh("res/meshes/space_port.obj");
    Texture *t = get_texture("res/textures/space_port.png");
    StaticBuilding *sp = new StaticBuilding;
    sp->body = create_body(m, partsshader, t, 0, 0, 0, 0);
    setPosRot(sp->body, start + dir * pad_height, faceAlong(dir));
    sp->parent = hb;
    sp->sun = sun;
    sp->polar = polar;
    hb->pads.push_back(sp);
}

Vehicle *Ships::place_ship(const std::string &shipDefPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys,
                           double t)
{
    return place_ship_def(load_ship_def(resdir::path(shipDefPath).c_str(), part_catalog),
                          shipDefPath, wantName, hb, sc, sys, t);
}

Vehicle *Ships::place_ship_def(const ShipDef &def, const std::string &defPath,
                               const std::string &wantName,
                               TerrainBody *hb, const ScenarioDef *sc, System &sys,
                               double t)
{
    // slot = how many ships already sit on this (body, scenario)
    int slot = 0;
    for(auto *s : collectVehicles(sys)) {
        if(s->home == hb && s->scenario == sc) { slot++; }
    }

    // name: the caller's, else the def's; de-duplicated across the system
    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    nm = dedupName(sys, nm);

    // the pad top is this far above the terrain surface (space_port.obj
    // spans local z in [-10, 0], placed at dir * (terrain + pad_height))
    const double pad_height = 5.0;
    const bool pad_polar = sc->on_pad && sc->polar;
    const glm::dvec3 pad_dir = pad_polar
        ? glm::dvec3(0.0, 1.0, 0.0)
        : glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    const glm::dmat3 pad_orient = faceAlong(pad_dir);
    place_pad(hb, false, glm::normalize(glm::dvec3(0.005, 0.005, 1.0)), pad_height); // default site
    if(pad_polar) { place_pad(hb, true, pad_dir, pad_height); }                      // polar site

    /* the kerbal def (root part type "kerbal") builds the EVA subclass
       (src/eva.h); every other def builds a plain ship */
    const bool is_kerbal = !def.parts.empty()
        && def.parts[0].def->type == "kerbal";
    Vehicle *v = is_kerbal ? static_cast<Vehicle *>(new Kerbal) : new Vehicle;
    v->name = nm;
    v->defPath = defPath;
    v->home = hb;
    v->scenario = sc;
    v->slot = slot;
    v->m_parent = hb;
    v->sun = sun;
    v->frame = hb->rot_frame;
    // lateral pad slot (pad local X) so pad ships stand side by side; for
    // orbit scenarios this is only staging -- spawn_vehicle repositions.
    const glm::dvec3 base = pad_dir * ((double)hb->GetTerrainHeight(pad_dir) + pad_height)
        + pad_orient * glm::dvec3(20.0 * (double)slot, 0.0, 0.0);
    build_ship(v, def, partsshader, base, pad_orient);
    if(is_kerbal) {
        // frictionless feet: foot friction would pair with the walk force
        // into a tipping couple (see src/eva.cpp).
        SetFriction(v->hull, 0.0);
    }
    v->setVelocity(glm::dvec3(0, 0, 0));
    // The bookkeeping re-home: enters the body's ships list and starts the
    // flight journal at `t`.
    v->setSoi(hb->rot_frame, t);
    return v;
}

Vehicle *Ships::spawn_ship(const std::string &defPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys,
                           double t)
{
    Vehicle *v = place_ship(defPath, wantName, hb, sc, sys, t);
    spawn_vehicle(v, *sc, hb, sys, ORBIT_SLOT_SPACING * (double)v->slot, t);
    v->goOnRails();
    // N = v's position in the canonical order, M = the whole fleet.
    int n = 0, i = 0;
    for(auto *x : collectVehicles(sys)) { n++; if(x == v) { i = n; } }
    printf("Spawned '%s' (ship %d of %d)\n", v->name.c_str(), i, n);
    return v;
}

std::string Ships::dedupName(System &sys, const std::string &nm)
{
    std::string candidate = nm;
    int n = 2;
    while(true) {
        bool taken = false;
        for(auto *s : collectVehicles(sys)) {
            if(s->name == candidate) { taken = true; break; }
        }
        if(!taken) { return candidate; }
        candidate = nm + " #" + std::to_string(n);
        n++;
    }
}

// One crew kerbal aboard (ship, part): build a kerbal, park it inside the
// capsule (out of the physics world), register it in the capsule's
// containment edge so the ship's mass carries it.
Kerbal *Ships::spawn_crew_kerbal(Vehicle *ship, size_t part, System &sys, double t) {
    if(part >= ship->parts.size()) { return nullptr; }
    const PartDef *capDef = ship->parts[part]->def;
    if(capDef->crew_capacity <= 0) { return nullptr; }
    Part *capPart = ship->parts[part];

    ShipDef def = load_ship_def(resdir::path("res/ships/kerbal.json").c_str(),
                                part_catalog);
    Kerbal *k = new Kerbal;
    k->name = dedupName(sys, "kerbal");
    k->defPath = "res/ships/kerbal.json";
    k->home = ship->home;
    k->scenario = ship->scenario;
    k->m_parent = ship->m_parent;
    k->sun = sun;
    k->frame = ship->frame;

    // build it AT the capsule COM (it will be parked there, inside the ship)
    const glm::dvec3 capCom = ship->partPos(capPart);
    const glm::dmat3 capOrient = ship->partRot(capPart);
    build_ship(k, def, partsshader, capCom, capOrient);
    SetFriction(k->hull, 0.0);   // frictionless feet (see place_ship)

    // park inside the capsule (out of the physics world). The ship is
    // heavier with crew aboard through the containment edge (effectiveMass).
    Body *kb = k->hull;
    k->placeShipAtCom(capCom, capOrient);
    RemoveBody(kb);
    k->onRails = true;
    k->railFrozen = true;
    k->aboardPart = capPart;
    // aboardPart is set first -- setSoi keys its ships-list membership on it.
    k->setSoi(ship->frame, t);

    ship->crew.push_back(k);
    // register the containment edge (both directions). Vehicle::crew stays
    // the sole owner; contents is a non-owning back-reference (part.h).
    capPart->contents.push_back(k->parts[0]);
    k->parts[0]->container = capPart;
    // rebuild so the compound carries the crew mass
    ship->rebuildCompound();
    int aboard = 0;
    for(auto *c : ship->crew) {
        if(static_cast<Kerbal *>(c)->aboardPart == capPart) { aboard++; }
    }
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           k->name.c_str(), ship->name.c_str(), part,
           aboard, capDef->crew_capacity);
    return k;
}

void Ships::spawn_crew(Vehicle *ship, System &sys, double t) {
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->parts[i]->def->crew_capacity > 0) {
            spawn_crew_kerbal(ship, i, sys, t);
        }
    }
}

void Ships::apply_scenarios(System &sys, double t) {
    for(auto *b : sys.bodies) {
        /* Snapshot, not a reference: spawn_vehicle can re-home a ship into
           ANOTHER body's list (setSoi moves it). */
        const std::vector<Vehicle *> ships = b->ships;
        for(auto *s : ships) {
            if(s->scenario == nullptr) { continue; }
            spawn_vehicle(s, *s->scenario, s->home, sys, ORBIT_SLOT_SPACING * (double)s->slot, t);
        }
    }
}

void Ships::add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot,
                     double t)
{
    v->home = home;
    v->scenario = sc;
    v->slot = slot;
    v->setSoi(v->frame, t);   // register + journal (the builder set frame)
}

Vehicle *Ships::buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                     System &sys, double t)
{
    Vehicle *first = nullptr;
    for(size_t i = 0; i < entries.size(); i++) {
        const DebugStartShip &fe = entries[i];
        TerrainBody *hb = sys.find(fe.body);
        if(hb == nullptr) {
            std::string avail;
            for(size_t k = 0; k < sys.bodies.size(); k++) {
                if(k) { avail += ", "; }
                avail += sys.bodies[k]->name;
            }
            throw std::runtime_error("start ships: ship entry " + std::to_string(i)
                                     + ": unknown body '" + fe.body
                                     + "' (available: " + avail + ")");
        }
        const ScenarioDef *sc = scenario_by_name(fe.scenario);
        Vehicle *v = place_ship(fe.ship, fe.name, hb, sc, sys, t);
        // startup crew: one kerbal aboard each capsule
        spawn_crew(v, sys, t);
        if(first == nullptr) { first = v; }
    }
    return first;
}
