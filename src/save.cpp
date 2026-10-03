// save.cpp -- the Game-coupled capture/restore (save_game / load_game) +
// the save-directory helpers. The pure JSON (de)serialization lives in
// save.h (header-only).

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
#include "resdir.h"    // resdir::path
#include "ships.h"     // Ships, collectVehicles
#include "shipdef.h"   // PartsCatalog, PartDef, ResourceType, ShipDef, scenario_by_name
#include "system.h"    // System
#include "terrain.h"   // TerrainBody
#include "texture.h"   // get_texture
#include "vehicle.h"   // Vehicle, build_ship, SlewMode, ScenarioDef

namespace {

// ---- small helpers ----------------------------------------------------------

std::string slug(size_t i) { return "v" + std::to_string(i); }

// The save's real-world timestamp: UTC so it is unambiguous across
// boxes/timezones. gmtime is standard C -- main-thread only.
std::string nowString() {
    time_t t = time(nullptr);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", gmtime(&t));
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
// save, so throw with the name.
const ScenarioDef *resolveScenario(const std::string &name) {
    if(name.empty()) { return nullptr; }
    return scenario_by_name(name);
}

// The live Part a saved uid names, but only if it is one of `owner`'s (the
// uid map spans the whole save, so the uid alone does not prove the part
// belongs to the ship named alongside it). 0 or a miss gives nullptr.
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


// ---- inventory items ----------------------------------------------------
// An item is a standalone Part owned by its container (Part::ownedContents),
// and a container may itself be an item -- capture and restore are
// recursive. Both paths walk the SAME depth-first order.

// One item (and its nested items) from the live Part to SavePart.
SavePart saveItemPart(Part *c) {
    SavePart si;
    si.part = c->def->name;
    si.uid = c->uid;
    si.id = c->id;
    si.mass = c->body->mass;
    si.hull_margin = c->body->hull_margin;
    for(int r = 0; r < (int)ResourceType::Num; r++) {
        si.fuel.push_back((double)c->resources.current[r]);
    }
    si.experiments = c->experiments;
    for(Part *n : c->ownedContents) { si.inventory.push_back(saveItemPart(n)); }
    return si;
}

// Rebuild `saved` (and each entry's nested items) as items of `container`.
// Throws on an unknown def or a container over capacity; whatever was wired
// before the throw hangs off `container` (the caller frees it by deleting
// the owning vehicle).
void buildInventoryItems(Game &g, const std::vector<SavePart> &saved,
                         Part *container, const std::string &shipName) {
    const PartsCatalog &cat = g.ships.catalog();
    // a live add refuses a full container; a load must too (a hand-edited
    // save must not exceed inventory_capacity)
    if((int)saved.size() > container->def->inventory_capacity) {
        throw std::runtime_error("load: '" + shipName + "' part '"
                                 + container->def->name + "' holds "
                                 + std::to_string(saved.size())
                                 + " inventory item(s), more than its "
                                 + std::to_string(container->def->inventory_capacity)
                                 + "-slot capacity");
    }
    for(size_t i = 0; i < saved.size(); i++) {
        const SavePart &si = saved[i];
        const PartDef *pd = cat.find(si.part);
        if(pd == nullptr) {
            throw std::runtime_error("load: '" + shipName + "' inventory item '"
                                     + si.part + "' is not in the parts file");
        }
        Mesh *m = get_mesh("res/" + pd->mesh);
        Texture *t = get_texture("res/" + pd->texture);
        Body *b = create_part_body(m, g.partsshader, t,
                                   (float)si.mass, si.hull_margin);
        Part *item = new Part;
        item->body = b;
        item->def = pd;
        item->id = si.id;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            item->resources.capacity[r] = pd->capacity[r];
            item->resources.current[r] = (r < (int)si.fuel.size())
                ? (float)si.fuel[r] : 0.0f;
        }
        item->experiments = si.experiments;
        container->ownedContents.push_back(item);
        container->contents.push_back(item);
        item->container = container;
        item->owner = container->owner;   // the vehicle the container is on
        buildInventoryItems(g, si.inventory, item, shipName);
    }
}

// How many inventory items a part carries (itself counted as 1 per level):
// the load summary line reports the BUILT count, so a silently dropped
// nested item shows up as a smaller number.
size_t countPartInventory(Part *p) {
    size_t n = p->ownedContents.size();
    for(Part *c : p->ownedContents) { n += countPartInventory(c); }
    return n;
}

// ---- capture (Game -> SaveShip) --------------------------------------------

SaveShip saveShipFromVehicle(Vehicle *v) {
    SaveShip s;
    s.name = v->name;
    s.defPath = v->defPath;
    s.is_crew = v->isEva();
    s.flog = v->flog;   // the vessel's flight journal (shared by ships + crew)
    if(s.is_crew) {
        Kerbal *k = static_cast<Kerbal *>(v);
        Vehicle *ship = k->aboard();
        s.aboard = (ship != nullptr) ? ship->name : "";
        // the capsule is named by uid (not its index): stable across a
        // merge/split and order-independent on load. 0 = free.
        s.aboard_part = (k->aboardPart != nullptr) ? k->aboardPart->uid : 0;
        // a free (EVA) kerbal lives in the world -- save its pose like a
        // ship's (an aboard one's pose is unused on load).
        s.pose.body = (v->m_parent != nullptr) ? v->m_parent->name : "";
        s.pose.rotating = v->frame->isRotFrame();
        v->frameS(s.pose.pos, s.pose.rot);
        s.pose.vel = v->GetVel();
        s.pose.angvel = GetAngVelocity(v->hull);
        s.onRails = v->onRails;
        /* the suit tank contents (the kerbal's part 0 is the suit). Saved
           so a kerbal that burned some does not get a free re-seed. */
        if(!v->parts.empty()) {
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                s.suit_fuel.push_back((double)v->parts[0]->resources.current[r]);
            }
            // the suit's inventory items (depth-first)
            for(Part *c : v->parts[0]->ownedContents) {
                s.suit_inventory.push_back(saveItemPart(c));
            }
            // science: experiments recorded on the suit
            s.suit_experiments = v->parts[0]->experiments;
        }
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
        sp.experiments = p->experiments;
        // nested inventory items (depth-first: an item's own items are
        // emitted inside it)
        for(Part *c : p->ownedContents) {
            sp.inventory.push_back(saveItemPart(c));
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
// an attach spec). `savedUidToPart` (optional) receives this SAVE's uid ->
// the rebuilt Part; it is the caller's because it must span every ship file
// (a dock target's port lives in another ship).
Vehicle *buildShipFromSaveParts(Game &g, const SaveShip &s,
                                std::map<uint64_t, Part *> *savedUidToPart) {
    const PartsCatalog &cat = g.ships.catalog();
    // Resolve the scenario BEFORE constructing the vehicle: it throws, and
    // doing it first is what keeps a refused load from leaking (the rollback
    // only walks `built`).
    const ScenarioDef *scn = resolveScenario(s.scenario);
    Vehicle *v = new Vehicle;
    v->name = s.name;
    v->defPath = s.defPath;
    std::map<uint64_t, size_t> uidToIndex;
    std::map<uint64_t, Part *> uidToPart;
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        /* Refuse an unidentified or doubly-identified part rather than
           guess: uid 0 is a save that predates part identity; a repeated uid
           is a corrupt or hand-edited file. */
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
        Mesh *mesh = get_mesh("res/" + pd->mesh);
        Texture *tex = get_texture("res/" + pd->texture);
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
        p->experiments = sp.experiments;
        if(i == 0) {
            v->setRoot(p);
        } else {
            std::map<uint64_t, size_t>::const_iterator it = uidToIndex.find(sp.parent);
            if(it == uidToIndex.end()) {
                delete p;   // not yet attached to v->parts (so ~Vehicle won't take it)
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
            /* p is attached now, so ~Vehicle frees it. Two ship files
               claiming one uid cannot come from a save this game wrote. */
            delete v;
            throw std::runtime_error("load: two ships claim part uid "
                                     + std::to_string(sp.uid));
        }
        /* Reconstruct the inventory items (depth-first, nested included).
           A refused item frees through `delete v` -- the items wired so far
           hang off p and go with it. */
        try {
            buildInventoryItems(g, sp.inventory, p, s.name);
        } catch(const std::exception &) {
            delete v;
            throw;
        }
    }
    /* A named controller that is not in the ship is corruption: a silent
       fallback would fly the ship from the wrong part's axes. */
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
    // Restore the journal BEFORE setSoi, so setSoi's observe is a no-op on
    // the unchanged body and the mission history survives the load.
    v->flog = s.flog;
    // The re-home: enters the body's ships list and observes the SoI body.
    TerrainBody *pb = g.sys.find(s.pose.body);
    if(pb == nullptr) { pb = v->home; }
    v->setSoi(s.pose.rotating ? pb->rot_frame : pb->frame, g.time);

    v->placeShip(s.pose.pos, s.pose.rot);
    v->setVelocity(s.pose.vel);
    SetAngVelocity(v->hull, s.pose.angvel);
    v->thruster_util = s.throttle;   // restore the throttle (was saved from thruster_util)
    v->setSlewRequest((SlewMode)s.slew_request);

    /* A seam names two of THIS ship's parts, and both must be present.
       Skipping an unresolvable seam leaves the joint physically docked
       while the undock record is gone -- the ship becomes permanently
       un-undockable. */
    for(size_t k = 0; k < s.docks.size(); k++) {
        std::map<uint64_t, Part *>::const_iterator portIt = uidToPart.find(s.docks[k].port);
        std::map<uint64_t, Part *>::const_iterator rootIt = uidToPart.find(s.docks[k].root);
        if(portIt == uidToPart.end() || rootIt == uidToPart.end()) {
            v->detachSoiList();   // setSoi listed it above -- never leave a
            delete v;             // freed pointer in a body list, even transiently
            throw std::runtime_error("load: saved ship '" + s.name + "' has a dock seam ("
                                     + s.docks[k].name + ") naming a part it does not have");
        }
        v->seams.push_back(Vehicle::DockSeam{ portIt->second, rootIt->second, s.docks[k].name });
    }
    return v;
}

// Rebuild one kerbal (a one-part Vehicle) from its ship def + aboard state.
// The kerbal's fuel (its RCS suit) is NOT saved -- it seeds full on load.
// An aboard kerbal is parked inside its capsule (out of the world); its mass
// is carried by the containment edge, so the capsule's compound is rebuilt
// AFTER the edge is set. `savedUidToPart` spans every ship file and is what
// the aboard capsule's uid resolves through.
Kerbal *buildKerbalFromSave(Game &g, const SaveShip &s,
                            std::map<std::string, Vehicle *> &byName,
                            const std::map<uint64_t, Part *> &savedUidToPart) {
    const PartsCatalog &cat = g.ships.catalog();
    ShipDef def = load_ship_def(resdir::path(s.defPath).c_str(), cat);
    Kerbal *k = new Kerbal;
    k->name = s.name;
    k->defPath = s.defPath;
    k->home = g.home;
    k->scenario = nullptr;
    k->m_parent = g.home;
    k->sun = g.sun;
    k->frame = g.home->rot_frame;
    /* build_ship can throw after some parts are attached; k is not in the
       load's `built` list yet, so the rollback would not free it -- delete
       it here. ~Vehicle drops the partially-attached parts. */
    try {
        build_ship(k, def, g.partsshader, glm::dvec3(0.0), glm::dmat3(1.0));
    } catch(...) {
        delete k;
        throw;
    }
    /* Restore the suit tank contents (build_ship's init() re-seeded them
       full). Only the contents -- effectiveMass() reads them from
       resources.current. */
    if(!s.suit_fuel.empty() && !k->parts.empty()) {
        Part *suit = k->parts[0];
        for(size_t r = 0; r < s.suit_fuel.size() && r < (size_t)ResourceType::Num; r++) {
            const float cur = (float)s.suit_fuel[r];
            suit->resources.current[r] = cur;
        }
    }
    // science: experiments recorded on the suit (unlimited)
    if(!s.suit_experiments.empty() && !k->parts.empty()) {
        k->parts[0]->experiments = s.suit_experiments;
    }
    /* The suit's inventory items (depth-first). The compound was built
       before the items existed, so rebuild it. */
    if(!s.suit_inventory.empty() && !k->parts.empty()) {
        try {
            buildInventoryItems(g, s.suit_inventory, k->parts[0], s.name);
        } catch(const std::exception &) {
            delete k;
            throw;
        }
        k->rebuildCompound();
    }
    // Restore the journal BEFORE either setSoi below, so its observe is a
    // no-op on the unchanged body.
    k->flog = s.flog;
    if(s.aboard.empty()) {
        // free (on EVA): live in the world, at its saved pose
        TerrainBody *body = g.sys.find(s.pose.body);
        if(body == nullptr) { body = g.home; }
        k->home = body;
        // The re-home: enters the body's ships list and observes the SoI body.
        k->setSoi(s.pose.rotating ? body->rot_frame : body->frame, g.time);
        k->placeShip(s.pose.pos, s.pose.rot);
        k->setVelocity(s.pose.vel);
        SetAngVelocity(k->hull, s.pose.angvel);
        // a free kerbal saved on the rails stays parked (phase-2 skips crew)
        if(s.onRails) { k->goOnRails(); }
    } else {
        std::map<std::string, Vehicle *>::const_iterator it = byName.find(s.aboard);
        if(it == byName.end()) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' is aboard an unknown "
                                     "ship '" + s.aboard + "'");
        }
        Vehicle *ship = it->second;
        /* The capsule is named by uid, not index. aboard_part 0 is the
           "absent" sentinel and is refused. A nonzero uid must name one of
           THIS ship's parts and must actually be a capsule. */
        if(s.aboard_part == 0) {
            delete k;
            throw std::runtime_error("load: crew '" + s.name + "' is aboard ship '"
                                     + s.aboard + "' without a capsule uid (save "
                                     "predates uid-keyed crew)");
        }
        Part *cap = findSavedPart(savedUidToPart, s.aboard_part, ship);
        if(cap == nullptr) {
            delete k;
            /* Distinguish: a uid no ship has vs a uid that IS in the save
               but on a different ship (the reorder-corruption case). */
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
        /* A live board refuses a full capsule; a load must too. The count is
           the capsule's CREW (inventory items share contents but are not
           seated), built up one save entry at a time. */
        int crewInCap = 0;
        for(Part *c : cap->contents) {
            if(!c->ownedBy(cap)) { crewInCap++; }
        }
        if(crewInCap >= cap->def->crew_capacity) {
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
        // The bookkeeping re-home (aboardPart is set first -- setSoi keys
        // its ships-list membership on it).
        k->setSoi(ship->frame, g.time);
        ship->crew.push_back(k);
        // Register the containment edge (both directions). Vehicle::crew
        // stays the sole owner.
        cap->contents.push_back(k->parts[0]);
        k->parts[0]->container = cap;
        // The ship's compound was built before this kerbal existed: rebuild
        // so it carries the crew's mass (derived from the edge, not baked).
        ship->rebuildCompound();
    }
    return k;
}

} // namespace

// ---- the public entry points (declared in save.h) ---------------------------

void save_game(Game &g, const std::string &dir) {
    ensure_dir(dir);
    ensure_dir(dir + "/ships");
    SaveMeta meta;
    // The system the game is ACTUALLY running (the boot --system, or a live
    // switch), not args.system_file (stale after a swap).
    meta.system = g.systemPath.empty() ? g.args.system_file : g.systemPath;
    meta.parts = g.args.parts_file;
    meta.experiments = g.args.experiments_file;
    meta.time = g.time;
    meta.time_accel = g.time_accel;
    meta.active_ship = (g.ship != nullptr) ? g.ship->name : "";
    meta.exhaust_scale = g.args.exhaust_scale;
    meta.science_score = g.science.score;
    meta.recovered = g.science.recovered;
    meta.saved_at = nowString();

    std::vector<Vehicle *> fleet = collectVehicles(g.sys);
    for(size_t i = 0; i < fleet.size(); i++) {
        SaveShip s = saveShipFromVehicle(fleet[i]);
        meta.ships.push_back(slug(i));
        writeJson(dir + "/ships/" + slug(i) + ".json", saveShipToJson(s));
    }
    writeJson(dir + "/save.json", saveMetaToJson(meta));
    printf("Saved %zu ship(s) to %s (game '%s')\n", fleet.size(), dir.c_str(),
           g.gameName.c_str());
}

void load_game(Game &g, const std::string &dir) {
    SaveMeta meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    /* setTime, not a bare assignment: the clock jumps here and a load always
       starts paused, so no tick would re-derive the bodies' orbits and spin
       from the new epoch. Done BEFORE the fleet is rebuilt: a ship restored
       into the ROTATING frame that then parks as coasting captures its
       inertial rail state out of these transforms. */
    g.setTime(meta.time);
    // A load always starts paused. The save still records time_accel (the
    // round-trip field); it is simply not restored here.
    g.time_accel = 0;
    // The save's difficulty (exhaust-velocity scale). A save is not portable
    // across scales, so the file wins over Settings -- except when
    // --exhaust-scale was given (cli_given beats files).
    if(!g.args.cli_given.exhaust_scale) {
        g.args.exhaust_scale = meta.exhaust_scale;
    }

    /* Transactional: a load that throws leaves the running FLEET exactly as
       it was (the old fleet is only detached from the bodies' ship lists,
       not deleted). The clock, the frames, time_accel and exhaust_scale
       above are NOT rolled back, so a refused load leaves the OLD fleet
       paused at the SAVE's epoch. */

    // Read every ship file. A truncated or missing ships/<name>.json is what
    // a crash or a full disk mid-save produces.
    std::vector<SaveShip> saves;
    saves.reserve(meta.ships.size());
    for(size_t i = 0; i < meta.ships.size(); i++) {
        saves.push_back(saveShipFromJson(
            readJsonFile(dir + "/ships/" + meta.ships[i] + ".json")));
    }

    /* Detach the running fleet from the bodies but keep it ALIVE until the
       load commits: out of the way first (the builders append to those same
       lists), alive so a failed load is recoverable. Detaching rather than
       clearing also means nothing else needs saving: g.ship etc. stay valid
       throughout the build. */
    std::vector<std::pair<TerrainBody *, std::vector<Vehicle *>>> detached;
    for(TerrainBody *b : g.sys.bodies) {
        detached.emplace_back(b, b->ships);
        b->ships.clear();
    }

    /* Build every vehicle. collectVehicles orders a ship before its crew,
       so a crew's aboard ship is already in byName when the crew is built. */
    std::map<std::string, Vehicle *> byName;
    /* The save's uid -> the rebuilt Part, spanning EVERY ship file (a dock
       target's port lives in another ship). Keyed by the saved uid; the
       rebuilt Part carries a fresh uid of its own. */
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
           holds a freed pointer. Then delete what was built -- but not an
           aboard crew character (~Vehicle owns its crew). The ownership test
           is a SEPARATE pass: isCrewAboard() is virtual and must be read
           while everything is still alive. */
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

    // science (absent in a pre-science save: score 0, nothing recovered).
    // Success path: a refused load must not clobber the running career
    // score / recovered log (fleet rolls back in the catch, identity below).
    g.science.setFrom(meta.science_score, meta.recovered);

    // The load committed -- adopt the game identity from the save's location,
    // here so a REFUSED load leaves the running identity untouched. A slot
    // under saves/<game>/ belongs to that game; a save anywhere else leaves
    // the running identity.
    {
        namespace fs = std::filesystem;
        const fs::path gamedir = fs::path(dir).parent_path();
        if(gamedir.parent_path().filename() == "saves") {
            const std::string dn = gamedir.filename().string();
            if(!dn.empty()) {
                g.gameId = dn;
                g.gameName = gameDirName(dn);
            }
        }
    }
    // The old fleet goes -- the deletion the detach above deferred.
    // part_sels holds Part* into it, so that goes first.
    g.part_sels.clear();     // Part* into the old fleet -- drop before deleting
    g.clearFlightSummary();  // a prior recover's summary is not this save's
    for(auto &d : detached) {
        for(Vehicle *v : d.second) { delete v; }
    }
    g.ship = nullptr;
    g.kerbal = nullptr;
    g.lastShip = nullptr;
    g.focusBody = 0;

    // phase 2: resolve the cross-references + the world state. A dock INTENT
    // that names a part which is gone stays lenient (unlike the dock SEAM in
    // buildShipFromSaveParts, which throws: a dropped seam silently strands
    // a real joint).
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
        // the world state the ship was saved in: railed ships park, live
        // ships enter the physics world. buildShipFromSaveParts left the
        // hull out of the world, so this is the one place it is added (or
        // parked). A railed ship that is not rail-eligible stays live.
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
    // The BUILT inventory count: a silently dropped nested item prints a
    // smaller number than the save carried.
    size_t invItems = 0;
    for(size_t i = 0; i < built.size(); i++) {
        for(Part *p : built[i]->parts) { invItems += countPartInventory(p); }
    }
    if(invItems > 0) {
        printf("Loaded %zu inventory item(s) with the fleet\n", invItems);
    }
    printf("Loaded game from %s (active: %s)\n", dir.c_str(),
           (active != nullptr) ? active->name.c_str() : "(none)");
}
