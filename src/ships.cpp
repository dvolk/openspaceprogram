// ships.cpp -- ship construction + placement (see ships.h).
//
// The ships themselves are owned by the bodies they sit in
// (TerrainBody::ships) or, when aboard, by their ship (Vehicle::crew);
// this file only builds them and places them there. Each space pad gets
// its own model (like a ship part), so a pad is torn down with its body --
// no shared-model lifetime to manage.
#include "ships.h"

#include <cstdio>
#include <stdexcept>

#include "body.h"     // create_body
#include "eva.h"      // Kerbal (the crew characters)
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "physics.h"  // setPosRot
#include "shipdef.h"  // load_ship_def, ShipDef, PartsCatalog
#include "system.h"   // System (build_fleet / spawn_vehicle resolve bodies)
#include "texture.h"  // load_texture
#include "vehicle.h"  // build_ship, faceAlong, spawn_vehicle, scenario_by_name, Vehicle

std::vector<Vehicle *> collectVehicles(System &sys) {
    std::vector<Vehicle *> out;
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) {
            out.push_back(s);
            for(auto *c : s->crew) { out.push_back(c); }
        }
    }
    return out;
}

Ships::Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun)
    : part_catalog(load_parts_catalog(parts_file.c_str())),
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
    // Each pad owns its own model (a ship part would be built the same
    // way): unique mesh + texture, the part shader shared. That means the
    // pad's Body can be deleted freely in ~TerrainBody -- ~Body frees its
    // own model + rigid body -- with nothing left to leak.
    Mesh *m = new Mesh;
    m->FromFile("./res/space_port.obj", true);
    Texture *t = load_texture("./res/space_port.png");
    Model *model = new Model;
    model->FromData(m, partsshader, t);
    StaticBuilding *sp = new StaticBuilding;
    sp->body = create_body(model, 0, 0, 0, 0, false);
    setPosRot(sp->body, start + dir * pad_height, faceAlong(dir));
    sp->parent = hb;
    sp->sun = sun;
    sp->polar = polar;
    hb->pads.push_back(sp);
}

Vehicle *Ships::place_ship(const std::string &shipDefPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys)
{
    ShipDef def = load_ship_def(shipDefPath.c_str(), part_catalog);

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
    v->defPath = shipDefPath;
    v->home = hb;
    v->scenario = sc;
    v->slot = slot;
    v->m_parent = hb;
    v->sun = sun;
    v->frame = hb->rot_frame;
    // lateral pad slot (pad local X, 20 m apart) so pad ships stand side
    // by side; for orbit scenarios this is only staging -- spawn_vehicle
    // repositions along the orbit binormal and the part offsets relative
    // to the ship's own COM are what survive.
    const glm::dvec3 base = pad_dir * ((double)hb->GetTerrainHeight(pad_dir) + pad_height)
        + pad_orient * glm::dvec3(20.0 * (double)slot, 0.0, 0.0);
    build_ship(v, def, partsshader, base, pad_orient);
    if(is_kerbal) {
        // frictionless feet: the walk steering is a force applied at the
        // COM, and foot friction would pair with it into a tipping couple
        // that rolls the standing capsule over (see src/eva.cpp).
        SetFriction(v->controller->body, 0.0);
    }
    v->setVelocity(glm::dvec3(0, 0, 0));
    hb->ships.push_back(v);
    return v;
}

Vehicle *Ships::spawn_ship(const std::string &defPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys)
{
    Vehicle *v = place_ship(defPath, wantName, hb, sc, sys);
    spawn_vehicle(v, *sc, hb, sys, 100.0 * (double)v->slot);
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

/* One crew kerbal aboard (ship, part): build a kerbal, park it inside the
   capsule (out of the physics world), fold its mass into the capsule part
   (the ship is heavier with crew aboard), set its aboard state and store
   it on the ship (Vehicle::crew). The parked position is bookkeeping only
   -- the transitions (game.cpp) recompute the kerbal's pose when it EVAs,
   so a later scenario reposition of the ship (apply_scenarios) does not
   need to touch it. */
Kerbal *Ships::spawn_crew_kerbal(Vehicle *ship, size_t part, System &sys) {
    if(part >= ship->parts.size()) { return nullptr; }
    const PartDef *capDef = ship->parts[part]->def;
    if(capDef->crew_capacity <= 0) { return nullptr; }
    Body *cap = ship->parts[part]->body;

    ShipDef def = load_ship_def("./res/ships/kerbal.json", part_catalog);
    Kerbal *k = new Kerbal;
    k->name = dedupName(sys, "kerbal");
    k->defPath = "./res/ships/kerbal.json";
    k->home = ship->home;       // the ship was just placed (bookkeeping set)
    k->scenario = ship->scenario;
    k->m_parent = ship->m_parent;
    k->sun = sun;
    k->frame = ship->frame;

    // build it AT the capsule COM (it will be parked there, inside the ship)
    const glm::dvec3 capCom = GetPosition(cap);
    const glm::dmat3 capOrient = GetOrient(cap);
    build_ship(k, def, partsshader, capCom, capOrient);
    SetFriction(k->controller->body, 0.0);   // frictionless feet (see place_ship)

    // park inside the capsule (out of the physics world) + fold its mass
    // into the capsule part (the ship is heavier with crew aboard)
    Body *kb = k->parts[0]->body;
    setPosRot(kb, capCom, capOrient);
    RemoveBody(kb);
    k->onRails = true;
    k->railFrozen = true;
    cap->mass += k->parts[0]->body->mass;
    SetMass(cap, cap->mass);
    k->aboard = ship;
    k->aboardPart = part;

    ship->crew.push_back(k);
    int aboard = 0;
    for(auto *c : ship->crew) {
        if(static_cast<Kerbal *>(c)->aboardPart == part) { aboard++; }
    }
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           k->name.c_str(), ship->name.c_str(), part,
           aboard, capDef->crew_capacity);
    return k;
}

void Ships::spawn_crew(Vehicle *ship, System &sys) {
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->parts[i]->def->crew_capacity > 0) {
            spawn_crew_kerbal(ship, i, sys);
        }
    }
}

void Ships::apply_scenarios(System &sys) {
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) {
            if(s->scenario == nullptr) { continue; }
            spawn_vehicle(s, *s->scenario, s->home, sys, 100.0 * (double)s->slot);
        }
    }
}

void Ships::add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot)
{
    v->home = home;
    v->scenario = sc;
    v->slot = slot;
    home->ships.push_back(v);
}

Vehicle *Ships::build_fleet(const std::vector<FleetEntry> &entries, System &sys,
                            TerrainBody *home, const std::string &default_scenario)
{
    Vehicle *first = nullptr;
    for(size_t i = 0; i < entries.size(); i++) {
        const FleetEntry &fe = entries[i];
        TerrainBody *hb;
        if(fe.body.empty()) {
            hb = home; // the CLI --body resolution (or the system home)
        } else {
            hb = sys.find(fe.body);
            if(hb == nullptr) {
                std::string avail;
                for(size_t k = 0; k < sys.bodies.size(); k++) {
                    if(k) { avail += ", "; }
                    avail += sys.bodies[k]->name;
                }
                throw std::runtime_error("fleet: ship entry " + std::to_string(i)
                                         + ": unknown body '" + fe.body
                                         + "' (available: " + avail + ")");
            }
        }
        const ScenarioDef *sc =
            scenario_by_name(fe.scenario.empty() ? default_scenario : fe.scenario);
        Vehicle *v = place_ship(fe.ship, fe.name, hb, sc, sys);
        // startup crew: one kerbal aboard each capsule (the "characters in
        // ships" state; the EVA/Board transitions in game.cpp move them)
        spawn_crew(v, sys);
        if(first == nullptr) { first = v; }
    }
    return first;
}
