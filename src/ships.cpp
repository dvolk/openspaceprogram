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
#include "eva.h"      // Kerbal (spawn_kerbal_near)
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
    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    nm = dedupName(nm);

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
        SetFriction(v->controller, 0.0);
    }
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

std::string Ships::dedupName(const std::string &nm)
{
    std::string candidate = nm;
    int n = 2;
    while(true) {
        bool taken = false;
        for(size_t i = 0; i < ships.size(); i++) {
            if(ships[i]->name == candidate) { taken = true; break; }
        }
        if(!taken) { return candidate; }
        candidate = nm + " #" + std::to_string(n);
        n++;
    }
}

int Ships::spawn_kerbal_near(Vehicle *near)
{
    // `near`'s fleet record: the kerbal inherits its home body + scenario
    size_t ni = 0;
    while(ni < ships.size() && ships[ni] != near) { ni++; }
    TerrainBody *hb = (ni < ships.size()) ? ship_homes[ni] : nullptr;
    const ScenarioDef *sc = (ni < ships.size()) ? ship_sc[ni] : nullptr;

    ShipDef def = load_ship_def("./res/ships/kerbal.json", part_catalog);

    Kerbal *k = new Kerbal;
    k->name = dedupName("kerbal");
    k->defPath = "./res/ships/kerbal.json";
    k->m_parent = near->m_parent;
    k->sun = sun;
    k->frame = near->frame;

    const glm::dvec3 com = near->get_center_of_mass();
    const glm::dvec3 upDir = glm::normalize(com);
    // the coordinate axis most orthogonal to the radial = the spawn offset
    const glm::dvec3 refs[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    int best = 0;
    for(int i = 1; i < 3; i++) {
        if(fabs(glm::dot(refs[i], upDir)) < fabs(glm::dot(refs[best], upDir))) { best = i; }
    }
    const glm::dvec3 tangent =
        glm::normalize(refs[best] - glm::dot(refs[best], upDir) * upDir);

    glm::dvec3 base;
    if(near->frame->isRotFrame()) {
        /* a surface spawn: stand on the SAME FLOOR as the ship beside it.
           The floor radius is the ship's lowest part point along the
           radial (the pad top for pad ships, the terrain + margins
           otherwise); never below the analytic terrain. */
        double lowest = 0.0;   // relative to the ship COM, downward
        for(size_t i = 0; i < near->parts.size(); i++) {
            const double z = glm::dot(GetPosition(near->parts[i]) - com, upDir);
            lowest = std::min(lowest, z - near->partDefs[i]->height / 2.0);
        }
        const double floorR = std::max(glm::length(com) + lowest,
            (double)near->m_parent->GetTerrainHeight(glm::vec3(upDir)));
        base = upDir * floorR + tangent * 5.0;
    } else {
        // free fall: co-moving beside the ship
        base = com + tangent * 5.0;
    }
    /* standing: the cucumber's long axis (nose, column 2) = radial,
       face (column 1) = the offset direction. (Attitude torque cannot
       stand a body up through its ground contact, so the spawn pose
       must already be upright.) */
    const glm::dvec3 right = glm::cross(tangent, upDir);
    const glm::dmat3 orient = glm::dmat3(right, tangent, upDir);
    build_ship(k, def, partsshader, base, orient);
    SetFriction(k->controller, 0.0);   // frictionless feet (see place_ship)
    if(near->frame->isRotFrame()) { k->setVelocity(glm::dvec3(0.0)); }
    else { k->setVelocity(near->GetVel()); }

    ships.push_back(k);
    ship_homes.push_back(hb);
    ship_sc.push_back(sc);
    ship_slots.push_back(0);
    printf("Spawned kerbal '%s' beside '%s' (ship %d of %d)\n",
           k->name.c_str(), near->name.c_str(), (int)ships.size(), (int)ships.size());
    return (int)ships.size() - 1;
}

/* One crew kerbal aboard (ship, part): build a kerbal, park it inside the
   capsule (out of the physics world), fold its mass into the capsule part
   (the ship is heavier with crew aboard), and set its aboard state. The
   parked position is bookkeeping only -- the transitions (game.cpp) recompute
   the kerbal's pose when it EVAs, so a later scenario reposition of the ship
   (apply_scenarios) does not need to touch it. */
int Ships::spawn_crew_kerbal(Vehicle *ship, size_t part) {
    if(part >= ship->parts.size()) { return -1; }
    const PartDef *capDef = ship->partDefs[part];
    if(capDef->crew_capacity <= 0) { return -1; }
    Body *cap = ship->parts[part];

    // the ship's home/scenario bookkeeping (it was just placed in the fleet)
    size_t si = 0;
    while(si < ships.size() && ships[si] != ship) { si++; }
    TerrainBody *hb = (si < ships.size()) ? ship_homes[si] : ship->m_parent;
    const ScenarioDef *sc = (si < ships.size()) ? ship_sc[si] : nullptr;

    ShipDef def = load_ship_def("./res/ships/kerbal.json", part_catalog);
    Kerbal *k = new Kerbal;
    k->name = dedupName("kerbal");
    k->defPath = "./res/ships/kerbal.json";
    k->m_parent = ship->m_parent;
    k->sun = sun;
    k->frame = ship->frame;

    // build it AT the capsule COM (it will be parked there, inside the ship)
    const glm::dvec3 capCom = GetPosition(cap);
    const glm::dmat3 capOrient = GetOrient(cap);
    build_ship(k, def, partsshader, capCom, capOrient);
    SetFriction(k->controller, 0.0);   // frictionless feet (see place_ship)

    // park inside the capsule (out of the physics world) + fold its mass
    // into the capsule part (the ship is heavier with crew aboard)
    Body *kb = k->parts[0];
    setPosRot(kb, capCom, capOrient);
    RemoveBody(kb);
    k->onRails = true;
    k->railFrozen = true;
    cap->mass += k->parts[0]->mass;
    SetMass(cap, cap->mass);
    k->aboard = ship;
    k->aboardPart = part;

    ships.push_back(k);
    ship_homes.push_back(hb);
    ship_sc.push_back(sc);
    ship_slots.push_back(0);
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           k->name.c_str(), ship->name.c_str(), part,
           1, capDef->crew_capacity);
    return (int)ships.size() - 1;
}

void Ships::spawn_crew(Vehicle *ship) {
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->partDefs[i]->crew_capacity > 0) {
            spawn_crew_kerbal(ship, i);
        }
    }
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
        const int idx = place_ship(fe.ship, fe.name, hb, sc);
        // startup crew: one kerbal aboard each capsule (the "characters in
        // ships" state; the EVA/Board transitions in game.cpp move them)
        spawn_crew(ships[idx]);
    }
}
