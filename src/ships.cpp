// ships.cpp -- the runtime fleet: the ships in play, their per-ship
// bookkeeping, the space pads, and the place / spawn / remove operations.
//
// The active-ship selection, the camera and the rails-warp state live in
// main.cpp (sim/UI state); this file only owns the fleet data and the pure
// fleet operations. The space-port model is shared by every pad (one per
// body+site), so it is built once here and reused on demand.
#include "ships.h"

#include <cstdio>
#include <stdexcept>

#include "body.h"     // create_body
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "physics.h"  // setPosRot
#include "shipdef.h"  // load_ship_def, ShipDef, PartsCatalog
#include "system.h"   // System (build_fleet / spawn_vehicle resolve bodies)
#include "texture.h"  // load_texture
#include "vehicle.h"  // build_ship, faceAlong, spawn_vehicle, scenario_by_name, Vehicle

Ships::Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun)
    : part_catalog(load_parts_catalog(parts_file.c_str())),
      partsshader(partsshader),
      sun(sun),
      space_port_model(nullptr)
{
    // The space-port model is shared by every pad, so it is built once here
    // and reused on demand. Its mesh + texture are owned by the model
    // (FromData); the shader is borrowed (shared with the ships).
    Mesh *m = new Mesh;
    m->FromFile("./res/space_port.obj", true);
    Texture *t = load_texture("./res/space_port.png");
    space_port_model = new Model;
    space_port_model->FromData(m, partsshader, t);
}

Ships::~Ships() {
    clear();
}

void Ships::clear() {
    for(auto &kv : space_ports) { delete kv.second; }
    space_ports.clear();
    for(auto *s : ships) { delete s; }
    ships.clear();
    ship_homes.clear();
    ship_sc.clear();
    ship_slots.clear();
}

void Ships::place_pad(TerrainBody *hb, bool polar, const glm::dvec3 &dir, double pad_height)
{
    const std::pair<TerrainBody *, bool> key(hb, polar);
    if(space_ports.find(key) != space_ports.end()) { return; }
    const glm::dvec3 start = dir * (double)hb->GetTerrainHeight(dir);
    StaticBuilding *sp = new StaticBuilding;
    sp->body = create_body(space_port_model, 0, 0, 0, 0, false);
    setPosRot(sp->body, start + dir * pad_height, faceAlong(dir));
    sp->parent = hb;
    sp->sun = sun;
    space_ports[key] = sp;
}

int Ships::place_ship(const std::string &shipDefPath, const std::string &wantName,
                      TerrainBody *hb, const ScenarioDef *sc)
{
    ShipDef def = load_ship_def(shipDefPath.c_str(), part_catalog);

    // slot = how many ships already sit on this (body, scenario)
    int slot = 0;
    for(size_t i = 0; i < ships.size(); i++) {
        if(ship_homes[i] == hb && ship_sc[i] == sc) { slot++; }
    }

    // name: the caller's, else the def's; de-duplicated across the fleet
    // (first ship keeps the bare name, later ones get #2, #3 ..)
    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    {
        std::string candidate = nm;
        int n = 2;
        while(true) {
            bool taken = false;
            for(size_t i = 0; i < ships.size(); i++) {
                if(ships[i]->name == candidate) { taken = true; break; }
            }
            if(!taken) { break; }
            candidate = nm + " #" + std::to_string(n);
            n++;
        }
        nm = candidate;
    }

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

    Vehicle *v = new Vehicle;
    v->name = nm;
    v->defPath = shipDefPath;
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
    v->setVelocity(glm::dvec3(0, 0, 0));
    ships.push_back(v);
    ship_homes.push_back(hb);
    ship_sc.push_back(sc);
    ship_slots.push_back(slot);
    return (int)ships.size() - 1;
}

int Ships::spawn_ship(const std::string &defPath, const std::string &wantName,
                      TerrainBody *hb, const ScenarioDef *sc, System &sys)
{
    int idx = place_ship(defPath, wantName, hb, sc);
    spawn_vehicle(ships[idx], *ship_sc[idx], ship_homes[idx], sys,
                  100.0 * (double)ship_slots[idx]);
    ships[idx]->goOnRails();
    printf("Spawned '%s' (ship %d of %d)\n",
           ships[idx]->name.c_str(), idx + 1, (int)ships.size());
    return idx;
}

void Ships::apply_scenarios(System &sys)
{
    for(size_t i = 0; i < ships.size(); i++) {
        spawn_vehicle(ships[i], *ship_sc[i], ship_homes[i], sys,
                      100.0 * (double)ship_slots[i]);
    }
}

void Ships::erase_ship(size_t idx)
{
    if(idx >= ships.size()) { return; }
    Vehicle *v = ships[idx];
    ships.erase(ships.begin() + idx);
    ship_homes.erase(ship_homes.begin() + idx);
    ship_sc.erase(ship_sc.begin() + idx);
    ship_slots.erase(ship_slots.begin() + idx);
    delete v;   // dtor detaches welds + unregisters the bodies
}

void Ships::add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot)
{
    ships.push_back(v);
    ship_homes.push_back(home);
    ship_sc.push_back(sc);
    ship_slots.push_back(slot);
}

void Ships::build_fleet(const std::vector<FleetEntry> &entries, System &sys,
                        TerrainBody *home, const std::string &default_scenario)
{
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
        place_ship(fe.ship, fe.name, hb, sc);
    }
}
