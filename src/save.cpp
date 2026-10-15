// save.cpp -- the Game-coupled capture/restore (save_game / load_game).

#include "save.h"

#include <algorithm>   // std::find (save_game's orphan ship-file sweep)
#include <cassert>
#include <cstdio>
#include <ctime>
#include <filesystem>  // directory_iterator in save_game
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>   // std::pair

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

/* The slug a vehicle got in THIS save, for a reference written into another
   ship's file (a kerbal's ship, a dock target). save_game assigns every slug
   before writing any file, so a miss means the referent is not in the fleet. */
std::string slugOf(const std::map<const Vehicle *, std::string> &slugs,
                   const Vehicle *v) {
    if(v == nullptr) { return ""; }
    std::map<const Vehicle *, std::string>::const_iterator it = slugs.find(v);
    return it == slugs.end() ? std::string() : it->second;
}

// UTC stamp, unambiguous across timezones. gmtime's static buffer: main thread only.
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

// "" -> none; an unknown name throws (scenario_by_name).
const ScenarioDef *resolveScenario(const std::string &name) {
    if(name.empty()) { return nullptr; }
    return scenario_by_name(name);
}

// The live Part a saved uid names, only if it is one of `owner`'s: the uid map
// spans the whole save, so a uid alone does not prove ownership.
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
// An item is a standalone Part owned by its container (Part::ownedContents);
// a container may itself be an item, so capture and restore recurse in one
// shared depth-first order.

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
    /* A seat occupant shares ownedContents with the items (part.h): it is crew,
       not cargo, and is captured by its own SaveShip entry. */
    for(Part *n : c->ownedContents) {
        if(!n->isSeat()) { si.inventory.push_back(saveItemPart(n)); }
    }
    return si;
}

// Rebuild `saved` (and its nested items) as items of `container`. Throws on an
// unknown def or over-capacity; items wired before the throw hang off
// `container`, so the caller frees them with the owning vehicle.
void buildInventoryItems(Game &g, const std::vector<SavePart> &saved,
                         Part *container, const std::string &shipName) {
    const PartsCatalog &cat = g.ships.catalog();
    // a live add refuses a full container; a load must too
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
        /* A kerbal-type part is a person, not cargo. Parked anywhere other than
           a capsule it is a seat occupant in a non-capsule, which
           checkPartInvariants refuses -- and rebuildCompound asserts on that, so
           the save would abort the process. A hand-edited save is still just
           data: refuse it here, where refusing is an option. */
        if(pd->type == "kerbal") {
            throw std::runtime_error("load: '" + shipName + "' inventory item '"
                                     + si.part + "' is a crew part, not cargo");
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

// Recursive count: the load summary prints the BUILT total, so a silently
// dropped nested item shows up as a smaller number. A seat occupant is crew,
// not cargo, so it is not counted -- but what its suit carries is.
size_t countPartInventory(Part *p) {
    size_t n = 0;
    for(Part *c : p->ownedContents) {
        if(!c->isSeat()) { n++; }
        n += countPartInventory(c);
    }
    return n;
}

// ---- capture (Game -> SaveShip) --------------------------------------------

SaveShip saveShipFromVehicle(Vehicle *v,
                             const std::map<const Vehicle *, std::string> &slugs) {
    SaveShip s;
    s.name = v->name;
    s.defPath = v->defPath;
    s.is_crew = v->isEva();
    s.flog = v->flog;
    if(s.is_crew) {
        /* A Kerbal vehicle exists ONLY while walking (eva.h): a seated person is
           a seat on a capsule Part and is captured by saveSeatOccupant below. */
        Kerbal *k = static_cast<Kerbal *>(v);
        s.profile_id = k->profileId;
        s.pose.body = (v->m_parent != nullptr) ? v->m_parent->name : "";
        s.pose.rotating = v->frame->isRotFrame();
        v->frameS(s.pose.pos, s.pose.rot);
        s.pose.vel = v->GetVel();
        s.pose.angvel = GetAngVelocity(v->hull);
        s.onRails = v->onRails;
        /* The pose holds the ATTITUDE the space law was chasing, but that law
           steers toward camera-facing + this offset every substep: without it
           the suit slews back to camera-facing on load (see Kerbal::applyEva). */
        s.view_yaw = k->viewYaw;
        // part 0 is the suit; without this a partly-burned tank re-seeds full
        if(!v->parts.empty()) {
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                s.suit_fuel.push_back((double)v->parts[0]->resources.current[r]);
            }
            for(Part *c : v->parts[0]->ownedContents) {
                s.suit_inventory.push_back(saveItemPart(c));
            }
            s.suit_experiments = v->parts[0]->experiments;
        }
        return s;
    }
    s.home = (v->home != nullptr) ? v->home->name : "";
    s.scenario = (v->scenario != nullptr) ? v->scenario->name : "";
    for(size_t i = 0; i < v->parts.size(); i++) {
        Part *p = v->parts[i];
        SavePart sp;
        sp.part = p->def->name;
        sp.uid = p->uid;
        sp.id = p->id;
        sp.parent = (p->parent != nullptr) ? p->parent->uid : 0;
        sp.stage = p->stage;
        sp.chute = p->chute;
        sp.pos = p->localPos;
        sp.rot = p->localRot;
        sp.mass = p->body->mass;
        sp.hull_margin = p->body->hull_margin;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            sp.fuel.push_back((double)p->resources.current[r]);
        }
        sp.experiments = p->experiments;
        for(Part *c : p->ownedContents) {
            if(!c->isSeat()) { sp.inventory.push_back(saveItemPart(c)); }
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

    // the pose in the ship's CURRENT frame
    s.pose.body = (v->m_parent != nullptr) ? v->m_parent->name : "";
    s.pose.rotating = v->frame->isRotFrame();
    v->frameS(s.pose.pos, s.pose.rot);
    s.pose.vel = v->GetVel();
    s.pose.angvel = GetAngVelocity(v->hull);

    s.onRails = v->onRails;
    s.throttle = v->thruster_util;   // m_thrust is a per-tick "fired" flag
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
    /* Every ship delete clears the other ships' dock intents first (game.cpp),
       so a non-null target is in the fleet. Assert it: the slug map is keyed by
       pointer, so a recycled address would resolve to a VALID slug for the wrong
       ship, and the lenient fallback below would hide that. */
    assert(v->dockTargetShip == nullptr || slugs.count(v->dockTargetShip) != 0);
    /* A dock INTENT, not a seam: if the target somehow has no slug, dropping
       the intent loses nothing the player cannot re-target. */
    s.dock_target_ship = slugOf(slugs, v->dockTargetShip);
    s.dock_target_port = (v->dockTargetPort != nullptr) ? v->dockTargetPort->uid : 0;
    s.dock_arm_port = (v->dockArmPort != nullptr) ? v->dockArmPort->uid : 0;
    return s;
}

/* A seat occupant (part.h) as a crew SaveShip -- the same shape a walking
   kerbal's file has, so an existing save still loads. There is NO pose: a seat
   has no world pose of its own (the loader parks it from its capsule), and no
   view_yaw (a seat has no camera). `flog` is the ship's journal, so the person
   reads as having gone where their ship went. */
SaveShip saveSeatOccupant(Game &g, const Vehicle *ship, const Part *cap,
                          const Part *suit,
                          const std::map<const Vehicle *, std::string> &slugs) {
    SaveShip s;
    s.name = seatName(g.roster, suit);
    s.defPath = Ships::kKerbalShipDef;
    s.is_crew = true;
    s.aboard_ship = slugOf(slugs, ship);
    /* The capsule's own vehicle is in the fleet walk that produced these slugs,
       so a miss means the walk and the containment edge disagree -- and "" would
       silently land the person on EVA. */
    assert(!s.aboard_ship.empty());
    s.aboard_part = cap->uid;
    s.profile_id = suit->seatProfile;
    s.onRails = true;
    s.flog = ship->flog;
    for(int r = 0; r < (int)ResourceType::Num; r++) {
        s.suit_fuel.push_back((double)suit->resources.current[r]);
    }
    for(Part *c : suit->ownedContents) {
        if(!c->isSeat()) { s.suit_inventory.push_back(saveItemPart(c)); }
    }
    s.suit_experiments = suit->experiments;
    return s;
}

// ---- restore (SaveShip -> Game) --------------------------------------------

// Parts are rebuilt from the SOLVED poses, not a re-derived attach spec: a
// docking seam's relative geometry is not recoverable from an attach spec.
// `savedUidToPart` (optional) is the caller's because it must span every ship
// file -- a dock target's port lives in another ship.
Vehicle *buildShipFromSaveParts(Game &g, const SaveShip &s,
                                std::map<uint64_t, Part *> *savedUidToPart) {
    const PartsCatalog &cat = g.ships.catalog();
    // Resolve the scenario BEFORE constructing: it throws, and the rollback
    // only walks `built`.
    const ScenarioDef *scn = resolveScenario(s.scenario);
    Vehicle *v = new Vehicle;
    v->name = s.name;
    v->defPath = s.defPath;
    std::map<uint64_t, size_t> uidToIndex;
    std::map<uint64_t, Part *> uidToPart;
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        /* Refuse rather than guess: uid 0 is a save predating part identity,
           a repeated uid is a corrupt or hand-edited file. */
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
        // the chute canopy overlay is resolved at build time, so the load must too
        if(!pd->chute_mesh.empty()) {
            p->chute_mesh = get_mesh(std::string("res/") + pd->chute_mesh);
            p->chute_texture = get_texture(std::string("res/") + pd->chute_texture);
        }
        p->id = sp.id;
        p->stage = sp.stage;
        p->chute = (sp.chute > 0) ? ChuteDeployed : ChuteStowed;
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
                delete p;   // not yet attached, so ~Vehicle won't free it
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
            /* p is attached now, so ~Vehicle frees it. */
            delete v;
            throw std::runtime_error("load: two ships claim part uid "
                                     + std::to_string(sp.uid));
        }
        // A refused item frees through `delete v`: the items wired so far
        // hang off p and go with it.
        try {
            buildInventoryItems(g, sp.inventory, p, s.name);
        } catch(const std::exception &) {
            delete v;
            throw;
        }
    }
    /* A named controller missing from the ship is corruption: a silent
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
    v->scenario = scn;
    // Vehicle::slot is deliberately NOT restored: the next placement recomputes it.
    v->sun = g.sun;
    // BEFORE setSoi, whose observe is then a no-op on the unchanged body.
    v->flog = s.flog;
    TerrainBody *pb = g.sys.find(s.pose.body);
    if(pb == nullptr) { pb = v->home; }
    v->setSoi(s.pose.rotating ? pb->rot_frame : pb->frame, g.time);

    v->placeShip(s.pose.pos, s.pose.rot);
    v->setVelocity(s.pose.vel);
    SetAngVelocity(v->hull, s.pose.angvel);
    v->thruster_util = s.throttle;
    v->setSlewRequest((SlewMode)s.slew_request);

    /* Both parts of a seam must be present: skipping an unresolvable seam
       leaves the joint physically docked with no undock record, so the ship
       becomes permanently un-undockable. */
    for(size_t k = 0; k < s.docks.size(); k++) {
        std::map<uint64_t, Part *>::const_iterator portIt = uidToPart.find(s.docks[k].port);
        std::map<uint64_t, Part *>::const_iterator rootIt = uidToPart.find(s.docks[k].root);
        if(portIt == uidToPart.end() || rootIt == uidToPart.end()) {
            v->detachSoiList();   // never leave a freed pointer in a body list
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has a dock seam ("
                                     + s.docks[k].name + ") naming a part it does not have");
        }
        v->seams.push_back(Vehicle::DockSeam{ portIt->second, rootIt->second, s.docks[k].name });
    }
    return v;
}

// build_ship's init() re-seeds the suit tank full, so the saved contents must
// be restored after it. A WALKING kerbal only -- a seated one never reaches
// here (buildSeatFromSave parks a suit Part in the capsule instead, part.h).
Kerbal *buildKerbalFromSave(Game &g, const SaveShip &s,
                            const std::map<std::string, Vehicle *> &bySlug,
                            const std::map<uint64_t, Part *> &savedUidToPart) {
    /* The roster is where identity lives, so a kerbal naming one must find it
       there: rebuilding past a missing entry would silently strip that person's
       skills and name. 0 = a save predating the roster, which stays anonymous.
       Checked before the build so a throw frees nothing but `k` itself. */
    const KerbalProfile *profile = (s.profile_id != 0)
        ? g.roster.find(s.profile_id) : nullptr;
    if(s.profile_id != 0 && profile == nullptr) {
        throw std::runtime_error("load: kerbal '" + s.name + "' names roster id "
                                 + std::to_string(s.profile_id)
                                 + ", which the roster has no entry for");
    }
    const PartsCatalog &cat = g.ships.catalog();
    ShipDef def = load_ship_def(resdir::path(s.defPath).c_str(), cat);
    Kerbal *k = new Kerbal;
    k->name = s.name;
    k->profileId = s.profile_id;
    /* The roster owns the name, so it wins over the saved one: a rename in the
       save's own roster reaches the fleet instead of leaving two spellings. */
    if(profile != nullptr) { k->name = profile->name; }
    k->defPath = s.defPath;
    k->home = g.home;
    k->scenario = nullptr;
    k->m_parent = g.home;
    k->sun = g.sun;
    k->frame = g.home->rot_frame;
    /* build_ship can throw with parts attached; k is not in the load's
       `built` list yet, so the rollback would not free it. */
    try {
        // base 0 + no pad lift: the saved pose is applied below
        build_ship(k, def, g.partsshader, glm::dvec3(0.0), glm::dmat3(1.0), 0.0);
    } catch(...) {
        delete k;
        throw;
    }
    // Only the contents -- effectiveMass() reads them from resources.current.
    if(!s.suit_fuel.empty() && !k->parts.empty()) {
        Part *suit = k->parts[0];
        for(size_t r = 0; r < s.suit_fuel.size() && r < (size_t)ResourceType::Num; r++) {
            const float cur = (float)s.suit_fuel[r];
            suit->resources.current[r] = cur;
        }
    }
    if(!s.suit_experiments.empty() && !k->parts.empty()) {
        k->parts[0]->experiments = s.suit_experiments;
    }
    // The compound was built before the items existed: rebuild it.
    if(!s.suit_inventory.empty() && !k->parts.empty()) {
        try {
            buildInventoryItems(g, s.suit_inventory, k->parts[0], s.name);
        } catch(const std::exception &) {
            delete k;
            throw;
        }
        k->rebuildCompound();
    }
    // BEFORE either setSoi below, whose observe is then a no-op.
    k->flog = s.flog;
    k->viewYaw = s.view_yaw;
    /* Walking: the saved pose is the kerbal's own. A crew file that names a ship
       is a SEAT and never reaches here (load_game routes it to
       buildSeatFromSave). */
    assert(s.aboard_ship.empty());
    TerrainBody *body = g.sys.find(s.pose.body);
    if(body == nullptr) { body = g.home; }
    k->home = body;
    k->setSoi(s.pose.rotating ? body->rot_frame : body->frame, g.time);
    k->placeShip(s.pose.pos, s.pose.rot);
    k->setVelocity(s.pose.vel);
    SetAngVelocity(k->hull, s.pose.angvel);
    // a railed kerbal stays parked (phase 2 skips crew)
    if(s.onRails) { k->goOnRails(); }
    /* The identity binding is the one fact about a loaded kerbal the fleet list
       cannot show, so make it observable: a save that silently rebuilds
       anonymous people is exactly the bug this line exists to catch. */
    printf("[crew] loaded '%s' on EVA\n", k->name.c_str());
    return k;
}

/* A seated crew entry rebuilt as a seat occupant (part.h): a suit Part parked in
   its capsule and owned through ownedContents, so the capsule frees it. No
   Kerbal vehicle exists until someone goes EVA (Game::kerbalEVA). Throws to
   refuse the load on a dangling reference, a non-capsule, or a full capsule. */
void buildSeatFromSave(Game &g, const SaveShip &s,
                       const std::map<std::string, Vehicle *> &bySlug,
                       const std::map<uint64_t, Part *> &savedUidToPart) {
    /* Identity lives in the roster, so a kerbal naming one must find it there:
       rebuilding past a missing entry would silently strip that person's skills
       and name. 0 = a save predating the roster, which stays anonymous under the
       name the file carries. */
    const KerbalProfile *profile = (s.profile_id != 0)
        ? g.roster.find(s.profile_id) : nullptr;
    if(s.profile_id != 0 && profile == nullptr) {
        throw std::runtime_error("load: kerbal '" + s.name + "' names roster id "
                                 + std::to_string(s.profile_id)
                                 + ", which the roster has no entry for");
    }
    std::map<std::string, Vehicle *>::const_iterator it = bySlug.find(s.aboard_ship);
    if(it == bySlug.end()) {
        throw std::runtime_error("load: crew '" + s.name + "' is aboard ship file \""
                                 + s.aboard_ship + "\", which save.json's ships list "
                                 "has no entry for (or lists after this crew)");
    }
    Vehicle *ship = it->second;
    if(s.aboard_part == 0) {
        throw std::runtime_error("load: crew '" + s.name + "' is aboard ship '"
                                 + ship->name + "' without a capsule uid (save "
                                 "predates uid-keyed crew)");
    }
    Part *cap = findSavedPart(savedUidToPart, s.aboard_part, ship);
    if(cap == nullptr) {
        // Distinguish an unknown uid from one that lives on another ship.
        const bool knownUid =
            savedUidToPart.find(s.aboard_part) != savedUidToPart.end();
        const std::string where = knownUid
            ? " but that uid belongs to a different ship"
            : ", which no ship in this save has (unknown uid)";
        throw std::runtime_error("load: crew '" + s.name + "' is parked in part "
                                 "uid " + std::to_string(s.aboard_part) +
                                 " of ship '" + ship->name + "'" + where);
    }
    if(cap->def->crew_capacity <= 0) {
        throw std::runtime_error("load: crew '" + s.name + "' is parked in '"
                                 + cap->def->name + "' of ship '" + ship->name
                                 + "', which is not a capsule (crew_capacity 0)");
    }
    /* A live board refuses a full capsule; a load must too. */
    if(cap->seatCount() >= cap->def->crew_capacity) {
        throw std::runtime_error("load: capsule '" + cap->def->name
                                 + "' of ship '" + ship->name + "' is full ("
                                 + std::to_string(cap->def->crew_capacity)
                                 + " seat(s)); crew '" + s.name + "' would exceed it");
    }
    Part *suit = g.ships.make_suit(s.defPath);
    // Only the contents -- effectiveMass() reads them from resources.current.
    for(size_t r = 0; r < s.suit_fuel.size() && r < (size_t)ResourceType::Num; r++) {
        suit->resources.current[r] = (float)s.suit_fuel[r];
    }
    suit->experiments = s.suit_experiments;
    suit->seatProfile = s.profile_id;
    if(profile == nullptr) { suit->seatName = s.name; }
    /* The seat edge before the items: buildInventoryItems copies the container's
       owner onto each item. An item throw leaves everything wired into the
       capsule, which the caller's rollback frees with the ship. */
    cap->contents.push_back(suit);
    cap->ownedContents.push_back(suit);
    suit->container = cap;
    suit->owner = ship;
    buildInventoryItems(g, s.suit_inventory, suit, s.name);
    // The compound was built before the seat existed: the crew mass rides it.
    ship->rebuildCompound();
    printf("[crew] loaded '%s' aboard '%s'\n", seatName(g.roster, suit).c_str(),
           ship->name.c_str());
}

} // namespace

// ---- the public entry points (declared in save.h) ---------------------------

void save_game(Game &g, const std::string &dir) {
    ensure_dir(dir);
    ensure_dir(dir + "/ships");
    SaveMeta meta;
    // the live system path; args.system_file is stale after a system swap
    meta.system = g.systemPath.empty() ? g.args.system_file : g.systemPath;
    meta.parts = g.args.parts_file;
    meta.experiments = g.args.experiments_file;
    meta.kerbals = g.args.kerbals_file;
    /* The roster travels in the save, so the people a save names are the people
       it saw -- editing kerbals.json afterwards cannot re-point an old save at
       different strangers. */
    for(const KerbalProfile &p : g.roster.profiles) {
        SaveKerbal sk;
        sk.id = p.id;
        sk.name = p.name;
        for(int r = 0; r < NumSkills; r++) { sk.skill[r] = p.skill[r]; }
        meta.roster.push_back(sk);
    }
    meta.time = g.time;
    meta.time_accel = g.time_accel;
    meta.exhaust_scale = g.args.exhaust_scale;
    meta.science_score = g.science.score;
    meta.recovered = g.science.recovered;
    meta.saved_at = nowString();

    /* The save's fleet in slug order: every vessel, each immediately followed by
       the seats in it. A seat is not a Vehicle (part.h) but still gets a file,
       and this keeps the slot -- and so the slug numbering -- that the writer
       used when crew were vehicles. */
    struct Entry {
        Vehicle *v;          // the vessel
        const Part *suit;    // set for a seat entry: its occupant
        const Part *cap;     // the capsule holding that seat
    };
    std::vector<Entry> fleet;
    {
        /* collectVehicles yields only flyable vessels, so the seats interleave
           here rather than in the walk itself. */
        for(Vehicle *v : collectVehicles(g.sys)) {
            fleet.push_back({v, nullptr, nullptr});
            for(Part *p : v->parts) {
                for(Part *suit : p->seats()) { fleet.push_back({v, suit, p}); }
            }
        }
    }
    /* Slugs for the WHOLE fleet first, before a single file is written: a
       ship's file references its crew's ship and its dock target by slug, and
       the referent can sit later in the walk than the referrer. A seat has no
       slug of its own because nothing references one. */
    std::map<const Vehicle *, std::string> slugs;
    for(size_t i = 0; i < fleet.size(); i++) {
        if(fleet[i].suit == nullptr) { slugs[fleet[i].v] = slug(i); }
    }
    /* The active ship is a vehicle the player is flying, so it is in the fleet
       the walk just took. A miss means the two disagree, and "" would silently
       promote built.front() on the next load. */
    assert(g.ship == nullptr || slugs.count(g.ship) != 0);
    meta.active_ship = slugOf(slugs, g.ship);
    for(size_t i = 0; i < fleet.size(); i++) {
        const Entry &e = fleet[i];
        SaveShip s = (e.suit != nullptr)
            ? saveSeatOccupant(g, e.v, e.cap, e.suit, slugs)
            : saveShipFromVehicle(e.v, slugs);
        meta.ships.push_back(slug(i));
        writeJson(dir + "/ships/" + slug(i) + ".json", saveShipToJson(s));
    }
    writeJson(dir + "/save.json", saveMetaToJson(meta));
    /* Drop ship files this fleet no longer has. A save directory is REUSED
       (quicksave rotation, --resave), so a shrunk fleet leaves orphan
       ships/vN.json that nothing reads but that make the tree lie about its
       fleet -- e2e 181's whole point is diffing a resave against its input.
       Done AFTER the writes, so a throw partway leaves the previous files
       alone rather than deleting first and writing half. */
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        fs::directory_iterator it(dir + "/ships", ec), end;
        for(; !ec && it != end; it.increment(ec)) {
            if(it->path().extension() != ".json") { continue; }
            const std::string stem = it->path().stem().string();
            if(std::find(meta.ships.begin(), meta.ships.end(), stem)
               == meta.ships.end()) {
                fs::remove(it->path(), ec);
            }
        }
    }
    printf("Saved %zu ship(s) to %s (game '%s')\n", fleet.size(), dir.c_str(),
           g.gameName.c_str());
}

void load_game(Game &g, const std::string &dir) {
    SaveMeta meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    /* The save's roster REPLACES the --kerbals file's, so a save stays itself
       after the data file is edited; an absent roster (a save predating the
       roster) keeps the file's. Done before the fleet build, which resolves
       every kerbal's profile id against this.
       The swap is one-way and permanent for the session: newGame and
       switchSystem do not re-read --kerbals, so a roster a save brought in stays
       in charge afterwards. Harmless while one roster file serves every save; if
       saves ever carry different crews, the reload belongs in newGame. */
    if(!meta.roster.empty()) {
        g.roster.profiles.clear();
        for(const SaveKerbal &sk : meta.roster) {
            KerbalProfile p;
            p.id = sk.id;
            p.name = sk.name;
            for(int r = 0; r < NumSkills; r++) { p.skill[r] = sk.skill[r]; }
            g.roster.profiles.push_back(p);
        }
    }
    /* setTime, not a bare assignment: the clock jumps here and the load starts
       paused, so no tick re-derives the bodies' orbits/spin from the new epoch.
       Done BEFORE the fleet: a ship restored into the ROTATING frame and then
       parked as coasting captures its inertial rail state from these transforms. */
    g.setTime(meta.time);
    g.time_accel = 0;
    // A save is not portable across exhaust scales, so the file wins over
    // Settings -- unless --exhaust-scale was given (cli_given beats files).
    if(!g.args.cli_given.exhaust_scale) {
        g.args.exhaust_scale = meta.exhaust_scale;
    }

    /* Transactional: a load that throws leaves the running FLEET exactly as it
       was (only detached from the bodies' ship lists, not deleted). NOT rolled
       back: the clock, frames, time_accel and exhaust_scale above, nor the
       roster swap -- once save.json has been read there is nothing to restore
       the --kerbals file's roster from. */

    std::vector<SaveShip> saves;
    saves.reserve(meta.ships.size());
    for(size_t i = 0; i < meta.ships.size(); i++) {
        saves.push_back(saveShipFromJson(
            readJsonFile(dir + "/ships/" + meta.ships[i] + ".json")));
    }

    /* Detach the running fleet from the bodies but keep it ALIVE until the load
       commits: out of the lists the builders append to, and g.ship etc. stay
       valid throughout the build. */
    std::vector<std::pair<TerrainBody *, std::vector<Vehicle *>>> detached;
    for(TerrainBody *b : g.sys.bodies) {
        detached.emplace_back(b, b->ships);
        b->ships.clear();
    }

    /* Keyed by SLUG, filled as the build walks the list: save_game writes a ship
       before its crew, so a crew file's referent is already built. Names cannot
       key anything -- two vehicles may share one (#266). */
    std::map<std::string, Vehicle *> bySlug;
    // Keyed by the SAVED uid; the rebuilt Part carries a fresh uid of its own.
    std::map<uint64_t, Part *> savedUidToPart;
    std::vector<Vehicle *> built(saves.size(), nullptr);
    /* The first vessel in save order, for the active-ship fallback below. A seat
       has no slot here: it is owned by the capsule it was parked in. */
    Vehicle *firstBuilt = nullptr;
    try {
        for(size_t i = 0; i < saves.size(); i++) {
            const SaveShip &s = saves[i];
            if(s.is_crew && !s.aboard_ship.empty()) {
                buildSeatFromSave(g, s, bySlug, savedUidToPart);
                continue;
            }
            built[i] = s.is_crew
                ? buildKerbalFromSave(g, s, bySlug, savedUidToPart)
                : buildShipFromSaveParts(g, s, &savedUidToPart);
            bySlug[meta.ships[i]] = built[i];
            if(firstBuilt == nullptr) { firstBuilt = built[i]; }
        }
    } catch(...) {
        /* Put the body lists back BEFORE deleting anything, so no list ever
           holds a freed pointer. Then delete what was built: every seat is owned
           by the capsule it was parked in, so its ship frees it, and a seat whose
           ship was not built yet threw before allocating anything. */
        for(auto &d : detached) { d.first->ships = d.second; }
        for(Vehicle *v : built) { delete v; }
        throw;
    }

    // Success path only: a refused load must not clobber the running career
    // score / recovered log.
    g.science.setFrom(meta.science_score, meta.recovered);

    // Game identity comes from the save's location, adopted here so a REFUSED
    // load leaves the running identity untouched.
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
    // The deletion the detach above deferred. part_sels holds Part* into it.
    g.part_sels.clear();
    g.clearFlightSummary();  // a prior recover's summary is not this save's
    for(auto &d : detached) {
        for(Vehicle *v : d.second) { delete v; }
    }
    g.ship = nullptr;
    g.kerbal = nullptr;
    g.lastShip = nullptr;
    g.focusBody = 0;

    /* Phase 2: cross-references + world state. A dock INTENT naming a gone part
       stays lenient, unlike a dock SEAM (a dropped seam strands a real joint). */
    for(size_t i = 0; i < saves.size(); i++) {
        const SaveShip &s = saves[i];
        if(s.is_crew) { continue; }
        // built[i] IS saves[i]. A name lookup here used to resolve a duplicate
        // name to whichever vehicle was built last, wiring the wrong ship's
        // dock intent and rail state.
        Vehicle *v = built[i];
        if(!s.dock_target_ship.empty()) {
            std::map<std::string, Vehicle *>::const_iterator t = bySlug.find(s.dock_target_ship);
            if(t != bySlug.end()) {
                v->dockTargetShip = t->second;
                v->dockTargetPort = findSavedPart(savedUidToPart, s.dock_target_port,
                                                  t->second);
            }
        }
        v->dockArmPort = findSavedPart(savedUidToPart, s.dock_arm_port, v);
        // buildShipFromSaveParts left the hull out of the world: this is the one
        // place it is added or parked. A non-rail-eligible ship stays live.
        if(s.onRails) {
            if(!v->goOnRails()) { v->enterWorld(); }
        } else {
            v->enterWorld();
        }
    }

    // the active ship
    /* A slug naming a SEAT resolves to nothing in bySlug: a seated kerbal is not
       a vehicle (part.h), and honouring it would load the player into a state
       they cannot leave without an EVA. A walking kerbal is fine -- that is how
       an EVA-only save resumes. */
    Vehicle *active = nullptr;
    if(!meta.active_ship.empty()) {
        std::map<std::string, Vehicle *>::const_iterator it = bySlug.find(meta.active_ship);
        if(it != bySlug.end()) { active = it->second; }
    }
    /* Nothing recorded (or a slug the save no longer has): take the first vessel
       in canonical fleet order, which is what a fresh boot makes active. This
       used to be byName.begin(), i.e. the alphabetically first NAME -- a kerbal
       called 'Ash' beat a ship called 'racer'. */
    if(active == nullptr) { active = firstBuilt; }
    g.ship = active;
    g.kerbal = (active != nullptr && active->isEva()) ? static_cast<Kerbal *>(active) : nullptr;
    g.lastShip = nullptr;
    // The save may enter OR leave the no-ship state.
    g.syncShipFocus();
    size_t invItems = 0;
    for(size_t i = 0; i < built.size(); i++) {
        if(built[i] == nullptr) { continue; }   // a seat slot
        for(Part *p : built[i]->parts) { invItems += countPartInventory(p); }
    }
    if(invItems > 0) {
        printf("Loaded %zu inventory item(s) with the fleet\n", invItems);
    }
    printf("Loaded game from %s (active: %s)\n", dir.c_str(),
           (active != nullptr) ? active->name.c_str() : "(none)");
}
