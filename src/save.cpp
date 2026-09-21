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

// The live Part a saved uid names, but only if it is one of `owner`'s. The
// uid map spans the whole save, so the uid alone does not prove the part
// belongs to the ship named alongside it -- and updateDocking assumes a dock
// target's port really is on the target, and an aboard kerbal's capsule
// really is on the ship it names. 0 or a miss gives nullptr. (Declared in the
// namespace, before the crew builder, so both load paths resolve through it.)
Part *findSavedPart(const std::map<uint64_t, Part *> &byUid, uint64_t uid,
                    const Vehicle *owner) {
    if(uid == 0 || owner == nullptr) { return nullptr; }
    std::map<uint64_t, Part *>::const_iterator it = byUid.find(uid);
    if(it == byUid.end()) { return nullptr; }
    for(size_t i = 0; i < owner->parts.size(); i++) {
        if(owner->parts[i] == it->second) { return it->second; }
    }
    return nullptr;
}


// ---- capture (Game -> SaveShip) --------------------------------------------

SaveShip saveShipFromVehicle(Vehicle *v) {
    SaveShip s;
    s.name = v->name;
    s.defPath = v->defPath;
    s.is_crew = v->isEva();
    if(s.is_crew) {
        Kerbal *k = static_cast<Kerbal *>(v);
        Vehicle *ship = k->aboard();
        s.aboard = (ship != nullptr) ? ship->name : "";
        // the capsule is named by uid (not its index in the ship's part list):
        // stable across a merge/split and order-independent on load. 0 = free.
        s.aboard_part = (k->aboardPart != nullptr) ? k->aboardPart->uid : 0;
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
        sp.uid = p->uid;
        sp.id = p->id;
        sp.parent = (p->parent != nullptr) ? p->parent->uid : 0;
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
        lk.from = v->fuelLinks[k].from->uid;
        lk.to = v->fuelLinks[k].to->uid;
        s.fuel_links.push_back(lk);
    }
    if(v->controller != nullptr) { s.controller = v->controller->uid; }

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
        dk.port = v->seams[k].port->uid;
        dk.root = v->seams[k].root->uid;
        dk.name = v->seams[k].name;
        s.docks.push_back(dk);
    }
    s.dock_target_ship = (v->dockTargetShip != nullptr) ? v->dockTargetShip->name : "";
    s.dock_target_port = (v->dockTargetPort != nullptr) ? v->dockTargetPort->uid : 0;
    s.dock_arm_port = (v->dockArmPort != nullptr) ? v->dockArmPort->uid : 0;
    return s;
}

// ---- restore (SaveShip -> Game) --------------------------------------------

// Rebuild one ship from its saved part tree. The parts are built with the
// low-level setRoot/attach primitives (the SOLVED poses, not a re-derived
// attach spec -- a docking seam's relative geometry is not recoverable from
// an attach spec, so the save carries the geometry it actually is). finalize()
// then re-derives the fuel groups + the compound body; the saved stage
// bookkeeping (a ship may have staged) is restored over finalize's defaults.
//
// `savedUidToPart` (optional) receives this SAVE's uid -> the rebuilt Part.
// It is the caller's, not a local, because it has to span every ship file --
// a dock target's port lives in another ship. The rebuilt Parts keep the fresh
// uids their own constructors minted; the saved uids are only this file's keys
// (see save.h).
Vehicle *buildShipFromSaveParts(Game &g, const SaveShip &s,
                                std::map<uint64_t, Part *> *savedUidToPart) {
    const PartsCatalog &cat = g.ships.catalog();
    /* Resolve the scenario BEFORE constructing the vehicle: it throws on an
       unknown name, and doing it first is what keeps a refused load from
       leaking -- a throw after `new Vehicle` (and before this ship is in the
       load's `built` list) would free nothing, since the rollback only walks
       `built`. The scenario needs only s.scenario, no built state. */
    const ScenarioDef *scn = resolveScenario(s.scenario);
    Vehicle *v = new Vehicle;
    v->name = s.name;
    v->defPath = s.defPath;
    std::map<uint64_t, size_t> uidToIndex;
    std::map<uint64_t, Part *> uidToPart;
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        /* Refuse an unidentified or doubly-identified part rather than guess.
           uid 0 means a save written before parts carried identity, so none
           of its references can be resolved at all; a repeated uid is a
           corrupt or hand-edited file. Both are exactly the ambiguity that
           made id-keyed resolution silently pick the wrong part, so they are
           errors here instead of a last-writer-wins overwrite. */
        if(sp.uid == 0) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' part '" + sp.id
                                     + "' has no uid (save predates part identity)");
        }
        if(uidToPart.find(sp.uid) != uidToPart.end()) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has two parts "
                                     "with uid " + std::to_string(sp.uid));
        }
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
            std::map<uint64_t, size_t>::const_iterator it = uidToIndex.find(sp.parent);
            if(it == uidToIndex.end()) {
                delete p;   // not yet attached to v->parts (so ~Vehicle won't take it); ~Part drops the Body
                delete v;
                throw std::runtime_error("load: saved ship '" + s.name + "' part '" +
                                         sp.id + "' has an unknown parent uid " +
                                         std::to_string(sp.parent));
            }
            v->attach(p, it->second, sp.pos, sp.rot);
        }
        uidToIndex[sp.uid] = i;
        uidToPart[sp.uid] = p;
        if(savedUidToPart != nullptr
           && !savedUidToPart->insert(std::make_pair(sp.uid, p)).second) {
            /* p is attached now, so ~Vehicle frees it. Two ship files claiming
               one uid cannot come from a save this game wrote: every part in
               the fleet had a distinct process-wide uid when it was captured. */
            delete v;
            throw std::runtime_error("load: two ships claim part uid "
                                     + std::to_string(sp.uid));
        }
    }
    /* A named controller that is not in the ship is corruption, not something
       to paper over with finalize()'s default: the controller is what the
       camera basis and the stick frame are built from, so a silent fallback
       flies the ship from the wrong part's axes. */
    if(s.controller != 0) {
        std::map<uint64_t, Part *>::const_iterator it = uidToPart.find(s.controller);
        if(it == uidToPart.end()) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name
                                     + "' names a controller uid "
                                     + std::to_string(s.controller) + " it does not have");
        }
        v->controller = it->second;
    }
    for(size_t k = 0; k < s.fuel_links.size(); k++) {
        std::map<uint64_t, Part *>::const_iterator f = uidToPart.find(s.fuel_links[k].from);
        std::map<uint64_t, Part *>::const_iterator t = uidToPart.find(s.fuel_links[k].to);
        if(f == uidToPart.end() || t == uidToPart.end()) {
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
    v->scenario = scn;   // resolved before `new Vehicle` (see top of this fn)
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

    /* A seam names two of THIS ship's parts, and both must be present. This
       used to `continue` past a seam it could not resolve, which is worse than
       a loud failure: the joint stays physically docked while the record that
       would undock it is gone, so the ship becomes permanently un-undockable
       (undock finds nothing to pop and reports "Cannot undock" forever). */
    for(size_t k = 0; k < s.docks.size(); k++) {
        std::map<uint64_t, Part *>::const_iterator portIt = uidToPart.find(s.docks[k].port);
        std::map<uint64_t, Part *>::const_iterator rootIt = uidToPart.find(s.docks[k].root);
        if(portIt == uidToPart.end() || rootIt == uidToPart.end()) {
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has a dock seam ("
                                     + s.docks[k].name + ") naming a part it does not have");
        }
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
// count the crew). `savedUidToPart` spans every ship file (built before the
// crew, which always follows its ship) and is what the aboard capsule's uid
// resolves through.
Kerbal *buildKerbalFromSave(Game &g, const SaveShip &s,
                            std::map<std::string, Vehicle *> &byName,
                            const std::map<uint64_t, Part *> &savedUidToPart) {
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
    /* build_ship can throw (a bad parent / controller / fuel link in the def)
       after some parts are attached to k. k is not in the load's `built` list
       yet, so the rollback would not free it -- delete it here. ~Vehicle drops
       the partially-attached parts; hull is null at this point (finalize has
       not run) and ~Vehicle handles that. */
    try {
        build_ship(k, def, g.partsshader, glm::dvec3(0.0), glm::dmat3(1.0));
    } catch(...) {
        delete k;
        throw;
    }
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
        /* The capsule is named by uid, not index. aboard_part 0 is the "absent"
           sentinel -- a save that predates uid-keyed crew -- so it is refused
           rather than silently parked in part 0. A nonzero uid must name one of
           THIS ship's parts (a reordered/foreign save is corruption, not a miss
           to paper over), and that part must actually be a capsule: the old
           index format had no such check, so a reordered save could park a
           kerbal in a fuel tank (report 1.4). */
        if(s.aboard_part == 0) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' is aboard ship '"
                                     + s.aboard + "' without a capsule uid (save "
                                     "predates uid-keyed crew)");
        }
        Part *cap = findSavedPart(savedUidToPart, s.aboard_part, ship);
        if(cap == nullptr) {
            delete k;
            /* Distinguish the two failure modes: a uid no ship has (unknown /
               mistyped) vs a uid that IS in the save but on a different ship
               (reordered crew). The second is the reorder-corruption case. */
            const bool knownUid =
                savedUidToPart.find(s.aboard_part) != savedUidToPart.end();
            const std::string where = knownUid
                ? " but that uid belongs to a different ship"
                : ", which no ship in this save has (unknown uid)";
            throw std::runtime_error("load: crew '" + s.name + "' is parked in part "
                                     "uid " + std::to_string(s.aboard_part) +
                                     " of ship '" + s.aboard + "'" + where);
        }
        if(cap->def->crew_capacity <= 0) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' is parked in '"
                                     + cap->def->name + "' of ship '" + s.aboard
                                     + "', which is not a capsule (crew_capacity 0)");
        }
        /* A live board refuses a full capsule (game.cpp kerbalBoard); a load
           must too, or a hand-edited save can park more kerbals in a seat than
           the capsule has -- a state the game can't otherwise reach. The count
           is the capsule's contents (every contained part is a kerbal), built
           up one save entry at a time, so this fires on the one that overflows. */
        if((int)cap->contents.size() >= cap->def->crew_capacity) {
            delete k;
            throw std::runtime_error("load: capsule '" + cap->def->name
                                     + "' of ship '" + s.aboard + "' is full ("
                                     + std::to_string(cap->def->crew_capacity)
                                     + " seat(s)); crew '" + s.name + "' would exceed it");
        }
        const glm::dvec3 capCom = ship->partPos(cap);
        const glm::dmat3 capOrient = ship->partRot(cap);
        k->home = ship->home;
        k->m_parent = ship->m_parent;
        k->frame = ship->frame;
        k->placeShipAtCom(capCom, capOrient);
        RemoveBody(k->hull);
        k->onRails = true;
        k->railFrozen = true;
        k->aboardPart = cap;
        ship->crew.push_back(k);
        /* step 2.4: register the containment edge (the kerbal's part is
           parked in the capsule, both directions). Vehicle::crew stays the
           sole owner; contents is a non-owning back-reference (2.1). */
        cap->contents.push_back(k->parts[0]);
        k->parts[0]->container = cap;
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
    /* The save's uid -> the rebuilt Part, spanning EVERY ship file: a dock
       target's port lives in another ship, so phase 2 cannot resolve it from
       one ship's local map. Keyed by the saved uid -- the rebuilt Part carries
       a fresh uid of its own and is not addressable by the saved one anywhere
       else. Distinct across files because the whole fleet was live in one
       process when it was captured; a collision is a corrupt save and
       buildShipFromSaveParts throws for it. */
    std::map<uint64_t, Part *> savedUidToPart;
    std::vector<Vehicle *> built;
    built.reserve(saves.size());
    try {
        for(size_t i = 0; i < saves.size(); i++) {
            const SaveShip &s = saves[i];
            Vehicle *v = s.is_crew ? buildKerbalFromSave(g, s, byName, savedUidToPart)
                                   : buildShipFromSaveParts(g, s, &savedUidToPart);
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
    // every vehicle exists. The dock target was saved by NAME with its port by
    // uid, so it is resolved here; the kerbal's aboard ship was resolved at
    // build time.
    //
    // A dock INTENT that names a part which is gone stays lenient (the target
    // just is not restored), unlike the dock SEAM in buildShipFromSaveParts
    // which throws: a stale target is an ordinary runtime state --
    // updateDocking already validates and drops one whose ship or port went
    // away -- whereas a dropped seam silently strands a real joint.
    for(size_t i = 0; i < saves.size(); i++) {
        const SaveShip &s = saves[i];
        if(s.is_crew) { continue; }
        Vehicle *v = byName[s.name];
        if(!s.dock_target_ship.empty()) {
            std::map<std::string, Vehicle *>::const_iterator t = byName.find(s.dock_target_ship);
            if(t != byName.end()) {
                v->dockTargetShip = t->second;
                v->dockTargetPort = findSavedPart(savedUidToPart, s.dock_target_port,
                                                  t->second);
            }
        }
        v->dockArmPort = findSavedPart(savedUidToPart, s.dock_arm_port, v);
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
