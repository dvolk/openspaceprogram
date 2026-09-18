// save.cpp -- the Game-coupled capture/restore (save_game / load_game) + the
// save-directory helpers. The pure JSON (de)serialization lives in save.h
// (header-only, so tests/test_save.cpp links it with no Game / Bullet); this
// file needs the fleet (Game / Ships / Vehicle / Kerbal) to read and rebuild
// it, so it is the only part of the save feature that touches the game.
//
// capture:  walk the fleet in canonical order (collectVehicles), read each
//           vehicle's authoritative state (part tree + fuel + mass + pose +
//           staging + docking + crew) into a SaveShip, and write the meta
//           (dir/save.json) + one file per vehicle (dir/ships/<slug>.json).
// restore:  clear the current fleet (ships + crew), rebuild each vehicle in
//           the save's order (ships before their crew), then resolve the
//           cross-references (the active ship, the dock targets) and put
//           each ship into the world state it was saved in (live or railed).

#include "save.h"

#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>   // std::pair (the detached per-body fleet lists)

#include "body.h"      // Body, create_part_body
#include "eva.h"       // Kerbal
#include "game.h"      // Game, Scene
#include "mesh.h"      // get_mesh
#include "physics.h"   // RemoveBody, AddPhysicsBody, SetAngVelocity, GetAngVelocity
#include "part.h"      // Part
#include "ships.h"     // Ships, collectVehicles
#include "shipdef.h"   // PartsCatalog, PartDef, ResourceType, ShipDef, scenario_by_name
#include "system.h"    // System
#include "terrain.h"   // TerrainBody
#include "texture.h"   // get_texture
#include "vehicle.h"   // Vehicle, build_ship, SlewMode, ScenarioDef

namespace {

// ---- small helpers ----------------------------------------------------------

std::string slug(size_t i) { return "v" + std::to_string(i); }

std::string nowString() {
    time_t t = time(nullptr);
    char buf[64];
    struct tm lt;
    localtime_r(&t, &lt);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &lt);
    return std::string(buf);
}

void writeJson(const std::string &path, const nlohmann::json &j) {
    std::ofstream f(path.c_str());
    if(!f.is_open()) { throw std::runtime_error("save: cannot write " + path); }
    f << j.dump(2) << "\n";
    f.flush();
    if(!f.good()) { throw std::runtime_error("save: error writing " + path); }
}

nlohmann::json readJsonFile(const std::string &path) {
    std::ifstream f(path.c_str());
    if(!f.is_open()) { throw std::runtime_error("load: cannot open " + path); }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error("load: bad JSON in " + path + ": " + e.what());
    }
    return doc;
}

// Resolve a scenario name to its def; "" -> none. An unknown name is a bad
// save (a scenario renamed or removed since), so throw with the name.
const ScenarioDef *resolveScenario(const std::string &name) {
    if(name.empty()) { return nullptr; }
    return scenario_by_name(name);
}


// ---- capture (Game -> SaveShip) --------------------------------------------

SaveShip saveShipFromVehicle(Vehicle *v) {
    SaveShip s;
    s.name = v->name;
    s.defPath = v->defPath;
    s.is_crew = v->isEva();
    if(s.is_crew) {
        Kerbal *k = static_cast<Kerbal *>(v);
        s.aboard = (k->aboard != nullptr) ? k->aboard->name : "";
        s.aboard_part = (int)k->aboardPart;
        // a free (EVA) kerbal lives in the world -- save its pose like a
        // ship's (an aboard one's pose is unused on load).
        s.pose.body = (v->m_parent != nullptr) ? v->m_parent->name : "";
        s.pose.rotating = v->frame->isRotFrame();
        v->frameS(s.pose.pos, s.pose.rot);
        s.pose.vel = v->GetVel();
        s.pose.angvel = GetAngVelocity(v->hull);
        s.onRails = v->onRails;
        return s;
    }
    s.home = (v->home != nullptr) ? v->home->name : "";
    s.scenario = (v->scenario != nullptr) ? v->scenario->name : "";
    s.slot = v->slot;
    for(size_t i = 0; i < v->parts.size(); i++) {
        Part *p = v->parts[i];
        SavePart sp;
        sp.part = p->def->name;
        sp.id = p->id;
        sp.parent = (p->parent != nullptr) ? p->parent->id : "";
        sp.stage = p->stage;
        sp.pos = p->localPos;
        sp.rot = p->localRot;
        sp.mass = p->body->mass;
        sp.hull_margin = p->body->hull_margin;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            sp.fuel.push_back((double)p->resources.current[r]);
        }
        s.parts.push_back(sp);
    }
    for(size_t k = 0; k < v->fuelLinks.size(); k++) {
        SaveFuelLink lk;
        lk.from = v->fuelLinks[k].from->id;
        lk.to = v->fuelLinks[k].to->id;
        s.fuel_links.push_back(lk);
    }
    if(v->controller != nullptr) { s.controller = v->controller->id; }

    // the pose in the ship's CURRENT frame (the SOI body's inertial frame
    // for a coasting ship, its rotating surface frame for a grounded one)
    s.pose.body = (v->m_parent != nullptr) ? v->m_parent->name : "";
    s.pose.rotating = v->frame->isRotFrame();
    v->frameS(s.pose.pos, s.pose.rot);
    s.pose.vel = v->GetVel();
    s.pose.angvel = GetAngVelocity(v->hull);

    s.onRails = v->onRails;
    s.throttle = v->thruster_util;   // the throttle (m_thrust is a per-tick "fired" flag)
    s.active_stage = v->activeStage();
    s.total_stages = v->numStages();
    s.slew_request = (int)v->slewRequest;

    for(size_t k = 0; k < v->seams.size(); k++) {
        SaveDock dk;
        dk.port = v->seams[k].port->id;
        dk.root = v->seams[k].root->id;
        dk.name = v->seams[k].name;
        s.docks.push_back(dk);
    }
    s.dock_target_ship = (v->dockTargetShip != nullptr) ? v->dockTargetShip->name : "";
    s.dock_target_port = (v->dockTargetPort != nullptr) ? v->dockTargetPort->id : "";
    s.dock_arm_port = (v->dockArmPort != nullptr) ? v->dockArmPort->id : "";
    return s;
}

// ---- restore (SaveShip -> Game) --------------------------------------------

// Rebuild one ship from its saved part tree. The parts are built with the
// low-level setRoot/attach primitives (the SOLVED poses, not a re-derived
// attach spec -- a docking seam's relative geometry is not recoverable from
// an attach spec, so the save carries the geometry it actually is). finalize()
// then re-derives the fuel groups + the compound body; the saved stage
// bookkeeping (a ship may have staged) is restored over finalize's defaults.
Vehicle *buildShipFromSaveParts(Game &g, const SaveShip &s) {
    const PartsCatalog &cat = g.ships.catalog();
    Vehicle *v = new Vehicle;
    v->name = s.name;
    v->defPath = s.defPath;
    std::map<std::string, size_t> idToIndex;
    std::map<std::string, Part *> idToPart;
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        const PartDef *pd = cat.find(sp.part);
        if(pd == nullptr) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has a part "
                                     "'" + sp.part + "' that is no longer in the parts file");
        }
        Mesh *mesh = get_mesh("./res/" + pd->mesh);
        Texture *tex = get_texture("./res/" + pd->texture);
        Body *b = create_part_body(mesh, g.partsshader, tex,
                                   (float)sp.mass, sp.hull_margin);
        Part *p = new Part;
        p->body = b;
        p->def = pd;
        p->id = sp.id;
        p->stage = sp.stage;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            p->resources.capacity[r] = pd->capacity[r];
            p->resources.current[r] = (r < (int)sp.fuel.size()) ? (float)sp.fuel[r] : 0.0f;
        }
        if(i == 0) {
            v->setRoot(p);
        } else {
            std::map<std::string, size_t>::const_iterator it = idToIndex.find(sp.parent);
            if(it == idToIndex.end()) {
                delete p;   // not yet attached to v->parts (so ~Vehicle won't take it); ~Part drops the Body
                delete v;
                throw std::runtime_error("load: saved ship '" + s.name + "' part '" +
                                         sp.id + "' has an unknown parent '" + sp.parent + "'");
            }
            v->attach(p, it->second, sp.pos, sp.rot);
        }
        idToIndex[sp.id] = i;
        idToPart[sp.id] = p;
    }
    if(!s.controller.empty()) {
        std::map<std::string, Part *>::const_iterator it = idToPart.find(s.controller);
        if(it != idToPart.end()) { v->controller = it->second; }
    }
    for(size_t k = 0; k < s.fuel_links.size(); k++) {
        std::map<std::string, Part *>::const_iterator f = idToPart.find(s.fuel_links[k].from);
        std::map<std::string, Part *>::const_iterator t = idToPart.find(s.fuel_links[k].to);
        if(f == idToPart.end() || t == idToPart.end()) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has a fuel link "
                                     "to an unknown part");
        }
        v->fuelLinks.push_back(Vehicle::FuelLink{ f->second, t->second });
    }
    v->finalize();
    v->activeStage_ = s.active_stage;
    v->totalStages_ = s.total_stages;

    v->home = g.sys.find(s.home);
    if(v->home == nullptr) { v->home = g.home; }
    v->scenario = resolveScenario(s.scenario);
    v->slot = s.slot;
    v->sun = g.sun;
    v->m_parent = g.sys.find(s.pose.body);
    if(v->m_parent == nullptr) { v->m_parent = v->home; }
    v->frame = s.pose.rotating ? v->m_parent->rot_frame : v->m_parent->frame;

    v->placeShip(s.pose.pos, s.pose.rot);
    v->setVelocity(s.pose.vel);
    SetAngVelocity(v->hull, s.pose.angvel);
    v->thruster_util = s.throttle;   // restore the throttle (was saved from thruster_util)
    v->setSlewRequest((SlewMode)s.slew_request);

    for(size_t k = 0; k < s.docks.size(); k++) {
        std::map<std::string, Part *>::const_iterator portIt = idToPart.find(s.docks[k].port);
        std::map<std::string, Part *>::const_iterator rootIt = idToPart.find(s.docks[k].root);
        if(portIt == idToPart.end() || rootIt == idToPart.end()) { continue; }
        v->seams.push_back(Vehicle::DockSeam{ portIt->second, rootIt->second, s.docks[k].name });
    }
    v->m_parent->ships.push_back(v);
    return v;
}

// Rebuild one kerbal (a one-part Vehicle) from its ship def + aboard state.
// The kerbal's fuel (its RCS suit) is NOT saved -- it seeds full on load,
// like the startup spawn_crew_kerbal. An aboard kerbal is parked inside its
// capsule (out of the world); its mass is ALREADY in the capsule part's
// saved mass, so addPartMass is deliberately NOT called (that would double-
// count the crew).
Kerbal *buildKerbalFromSave(Game &g, const SaveShip &s,
                            std::map<std::string, Vehicle *> &byName) {
    const PartsCatalog &cat = g.ships.catalog();
    ShipDef def = load_ship_def(s.defPath.c_str(), cat);
    Kerbal *k = new Kerbal;
    k->name = s.name;
    k->defPath = s.defPath;
    k->home = g.home;
    k->scenario = nullptr;
    k->m_parent = g.home;
    k->sun = g.sun;
    k->frame = g.home->rot_frame;
    build_ship(k, def, g.partsshader, glm::dvec3(0.0), glm::dmat3(1.0));
    if(s.aboard.empty()) {
        // free (on EVA): live in the world, at its saved pose
        TerrainBody *body = g.sys.find(s.pose.body);
        if(body == nullptr) { body = g.home; }
        k->home = body;
        k->m_parent = body;
        k->frame = s.pose.rotating ? body->rot_frame : body->frame;
        k->placeShip(s.pose.pos, s.pose.rot);
        k->setVelocity(s.pose.vel);
        SetAngVelocity(k->hull, s.pose.angvel);
        body->ships.push_back(k);
        // a free kerbal saved on the rails (coasting at high warp) stays
        // parked -- the ships' phase-2 pass skips crew.
        if(s.onRails) { k->goOnRails(); }
    } else {
        std::map<std::string, Vehicle *>::const_iterator it = byName.find(s.aboard);
        if(it == byName.end()) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' is aboard an unknown "
                                     "ship '" + s.aboard + "'");
        }
        Vehicle *ship = it->second;
        if(s.aboard_part >= (int)ship->parts.size()) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' aboard part " +
                                     std::to_string(s.aboard_part) + " out of range");
        }
        Part *cap = ship->parts[s.aboard_part];
        const glm::dvec3 capCom = ship->partPos(cap);
        const glm::dmat3 capOrient = ship->partRot(cap);
        k->home = ship->home;
        k->m_parent = ship->m_parent;
        k->frame = ship->frame;
        k->placeShipAtCom(capCom, capOrient);
        RemoveBody(k->hull);
        k->onRails = true;
        k->railFrozen = true;
        k->aboard = ship;
        k->aboardPart = s.aboard_part;
        ship->crew.push_back(k);
    }
    return k;
}

} // namespace

// ---- the public entry points (declared in save.h) ---------------------------
// (ensure_dir / list_saves / delete_save are inline in save.h now -- they are
// pure file-system ops with no game state, so the headless test reaches them.)

void save_game(Game &g, const std::string &dir) {
    ensure_dir(dir);
    ensure_dir(dir + "/ships");
    SaveMeta meta;
    meta.system = g.args.system_file;
    meta.parts = g.args.parts_file;
    meta.time = g.time;
    meta.time_accel = g.time_accel;
    meta.active_ship = (g.ship != nullptr) ? g.ship->name : "";
    meta.saved_at = nowString();

    std::vector<Vehicle *> fleet = collectVehicles(g.sys);
    for(size_t i = 0; i < fleet.size(); i++) {
        SaveShip s = saveShipFromVehicle(fleet[i]);
        meta.ships.push_back(slug(i));
        writeJson(dir + "/ships/" + slug(i) + ".json", saveShipToJson(s));
    }
    writeJson(dir + "/save.json", saveMetaToJson(meta));
    printf("Saved %zu ship(s) to %s\n", fleet.size(), dir.c_str());
}

void load_game(Game &g, const std::string &dir) {
    SaveMeta meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    g.time = meta.time;
    g.time_accel = meta.time_accel;

    /* Transactional: everything that can fail is reading or building, and
       neither needs the old fleet DELETED first -- only out of the bodies'
       ship lists, which the detach below does. So a load that throws leaves
       the running game exactly as it was. It used to delete the fleet first
       and discover the failure afterwards, which left the player with nothing
       to fly and no way back. */

    // Read every ship file. A truncated or missing ships/<name>.json is what a
    // crash or a full disk mid-save actually produces.
    std::vector<SaveShip> saves;
    saves.reserve(meta.ships.size());
    for(size_t i = 0; i < meta.ships.size(); i++) {
        saves.push_back(saveShipFromJson(
            readJsonFile(dir + "/ships/" + meta.ships[i] + ".json")));
    }

    /* Detach the running fleet from the bodies but keep it ALIVE until the
       load commits. It has to be out of the way first because the builders
       append the new vehicles to those same lists (buildShipFromSaveParts ends
       in `v->m_parent->ships.push_back(v)`), and it has to stay alive because
       deleting it here is exactly what used to make a failed load
       unrecoverable.

       Detaching rather than clearing also means nothing else needs saving:
       g.ship, g.kerbal, g.lastShip and g.part_sels all still point at live
       vehicles throughout the build, so a refusal can put the lists back and
       the game carries on untouched. (The builders read the catalog, the
       shader, the system and the home body -- never the active ship.) */
    std::vector<std::pair<TerrainBody *, std::vector<Vehicle *>>> detached;
    for(TerrainBody *b : g.sys.bodies) {
        detached.emplace_back(b, b->ships);
        b->ships.clear();
    }

    /* Build every vehicle. The other realistic failure is a save naming a part
       the catalog no longer has -- the parts catalog moves and nothing here is
       versioned -- which buildShipFromSaveParts throws for.

       collectVehicles orders a ship before its crew, so a crew's aboard ship
       is already in byName when the crew is built -- the same invariant the
       cleanup below relies on. */
    std::map<std::string, Vehicle *> byName;
    std::vector<Vehicle *> built;
    built.reserve(saves.size());
    try {
        for(size_t i = 0; i < saves.size(); i++) {
            const SaveShip &s = saves[i];
            Vehicle *v = s.is_crew ? buildKerbalFromSave(g, s, byName)
                                   : buildShipFromSaveParts(g, s);
            built.push_back(v);
            byName[v->name] = v;
        }
    } catch(...) {
        /* Put the body lists back BEFORE deleting anything, so no list ever
           holds a freed pointer: the new vehicles are in those lists too
           (the builders put them there), and ~Vehicle does not unlink itself.

           Then delete what was built -- but not an aboard crew character,
           because ~Vehicle owns its crew and deleting both the ship and its
           kerbals would be a double free. A free (EVA) kerbal is not aboard
           anything and is deleted here like any other vehicle.

           The ownership test is a SEPARATE pass, because isCrewAboard() is
           virtual and the answer has to be read while everything is still
           alive: deleting a ship frees the kerbals aboard it, so testing them
           afterwards would call a virtual function through freed memory. (The
           same trap remove_ship had -- see its handoff loop.) */
        for(auto &d : detached) { d.first->ships = d.second; }
        std::vector<char> ownedByShip(built.size(), 0);
        for(size_t i = 0; i < built.size(); i++) {
            if(built[i]->isCrewAboard()) { ownedByShip[i] = 1; }
        }
        for(size_t i = 0; i < built.size(); i++) {
            if(!ownedByShip[i]) { delete built[i]; }
        }
        throw;
    }

    // The load committed, so the old fleet goes -- the deletion that the
    // detach above deferred. part_sels holds Part* into it, so that goes first.
    g.part_sels.clear();     // Part* into the old fleet -- drop before deleting
    for(auto &d : detached) {
        for(Vehicle *v : d.second) { delete v; }
    }
    g.ship = nullptr;
    g.kerbal = nullptr;
    g.lastShip = nullptr;
    g.focusBody = 0;

    // phase 2: resolve the cross-references + the world state, now that
    // every vehicle exists. The dock target was saved by NAME (its port by
    // id on the target), so it is resolved here; the kerbal's aboard ship
    // was resolved at build time.
    for(size_t i = 0; i < saves.size(); i++) {
        const SaveShip &s = saves[i];
        if(s.is_crew) { continue; }
        Vehicle *v = byName[s.name];
        if(!s.dock_target_ship.empty()) {
            std::map<std::string, Vehicle *>::const_iterator t = byName.find(s.dock_target_ship);
            if(t != byName.end()) {
                v->dockTargetShip = t->second;
                if(!s.dock_target_port.empty()) {
                    for(size_t p = 0; p < t->second->parts.size(); p++) {
                        if(t->second->parts[p]->id == s.dock_target_port) {
                            v->dockTargetPort = t->second->parts[p];
                            break;
                        }
                    }
                }
            }
        }
        if(!s.dock_arm_port.empty()) {
            for(size_t p = 0; p < v->parts.size(); p++) {
                if(v->parts[p]->id == s.dock_arm_port) { v->dockArmPort = v->parts[p]; break; }
            }
        }
        // the world state the ship was saved in: railed ships park (coast
        // or freeze), live ships enter the physics world.
        // buildShipFromSaveParts left the hull out of the world, so this is
        // the one place it is added (or parked). A railed ship that is not
        // rail-eligible (shouldn't happen -- the save only parked eligible
        // ones) stays live rather than being stranded.
        if(s.onRails) {
            if(!v->goOnRails()) { v->enterWorld(); }
        } else {
            v->enterWorld();
        }
    }

    // the active ship
    Vehicle *active = nullptr;
    if(!meta.active_ship.empty()) {
        std::map<std::string, Vehicle *>::const_iterator it = byName.find(meta.active_ship);
        if(it != byName.end()) { active = it->second; }
    }
    if(active == nullptr && !byName.empty()) {
        active = byName.begin()->second;
    }
    g.ship = active;
    g.kerbal = (active != nullptr && active->isEva()) ? static_cast<Kerbal *>(active) : nullptr;
    g.lastShip = nullptr;
    // The save may enter OR leave the no-ship state: keep the "ship" focus
    // entry in sync and point the camera at the ship, or home if none.
    g.syncShipFocus();
    printf("Loaded game from %s (active: %s)\n", dir.c_str(),
           (active != nullptr) ? active->name.c_str() : "(none)");
}
