// save.cpp -- the Game-coupled capture/restore (save_game / load_game).

#include "save.h"

#include <algorithm>   // std::find (save_game's orphan ship-file sweep)
#include <cassert>
#include <cstdio>
#include <ctime>
#include <filesystem>  // directory_iterator in save_game
#include <fstream>
#include <map>
#include <memory>   // unique_ptr (BuiltSave's build/commit halves)
#include <set>
#include <stdexcept>
#include <string>
#include <utility>   // std::pair

#include "body.h"      // Body
#include "eva.h"       // Kerbal
#include "game.h"      // Game, Scene
#include "physics.h"   // RemoveBody, AddPhysicsBody, SetAngVelocity, GetAngVelocity
#include "part.h"      // Part, make_part, make_suit
#include "resdir.h"    // resdir::path
#include "shipdef.h"   // PartsCatalog, PartDef, ResourceType, ShipDef, scenario_by_name
#include "system.h"    // System
#include "terrain.h"   // TerrainBody
#include "vehicle.h"   // Vehicle, build_ship, SlewMode, ScenarioDef

namespace {

// ---- small helpers ----------------------------------------------------------

std::string slug(size_t i) { return "v" + std::to_string(i); }

/* The slug a vehicle got in THIS save, for a reference written into another
   ship's file (a kerbal's ship, a dock target). save_game assigns every slug
   before writing any file, so a miss means the referent is not in the list. */
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

/* Write-then-rename (#304). An ofstream truncates its target the moment it
   opens, so a crash or a full disk partway through the write used to leave the
   slot holding a half-written file that the PREVIOUS save.json still pointed
   at. Writing to <path>.tmp and renaming means a reader sees either the whole
   old file or the whole new one: rename within one filesystem is atomic. Every
   failure path unlinks the temp; save_game's orphan sweep collects a leftover
   .tmp as well, since a process killed between the write and the rename leaves
   one no reader would ever open. */
void writeJson(const std::string &path, const nlohmann::json &j) {
    /* Serialize BEFORE opening anything. dump() throws on invalid UTF-8 in any
       string it walks -- and names arrive from UI text or a hand-edited file --
       so a throw with the temp already open would leave it behind: the orphan
       sweep filters by extension and never collects a .tmp. */
    const std::string text = j.dump(2) + "\n";
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp.c_str());
        if(!f.is_open()) { throw std::runtime_error("save: cannot write " + path); }
        f << text;
        /* close() is where a full disk or an NFS error actually surfaces, and
           the destructor closes after this scope -- i.e. after the rename. An
           unchecked close failure would install a truncated temp, which is the
           exact outcome this function exists to prevent. */
        f.close();
        if(!f.good()) {
            std::error_code ec;
            std::filesystem::remove(tmp, ec);   // best effort: the throw is the answer
            throw std::runtime_error("save: error writing " + path);
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if(ec) {
        std::error_code rm;
        std::filesystem::remove(tmp, rm);
        throw std::runtime_error("save: cannot install " + path + ": " + ec.message());
    }
}

/* One roster id, one lookup, one sentence. The builders throw with these words
   and preflightSave refuses with them, so the two paths cannot disagree about
   either the rule or the wording (#302) -- the same reason validateShipParts
   exists. 0 = a save predating the roster, which stays anonymous under the name
   the file carries. */
static std::string refuseUnknownRosterId(const CrewRoster &roster, uint64_t id,
                                         const std::string &who) {
    if(id == 0 || roster.find(id) != nullptr) { return ""; }
    return "load: " + who + " names roster id " + std::to_string(id)
           + ", which the roster has no entry for";
}

/* One roster entry, one wearer -- over a set of ids rather than a built world.
   refuseDoubleClaim applies it to the parts the load built; preflightSave
   applies it to the ids the SAVE names, which is the same set: a suit's id is
   its seat's seat_profile and a kerbal's is its profile_id, whatever the roster
   swap does to the name beside it (#302). Anonymous people (id 0) claim
   nothing. */
static std::string refuseDoubleClaimId(std::set<uint64_t> &claimed, uint64_t id) {
    if(id == 0 || claimed.insert(id).second) { return ""; }
    return "load: two people claim roster id " + std::to_string(id);
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
// An item is a standalone Part owned by its container (Part::contents);
// a container may itself be an item, so capture and restore recurse in one
// shared depth-first order.

SavePart saveItemPart(const Part *c) {
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
    for(Part *n : c->contents) {
        si.inventory.push_back(saveItemPart(n));
    }
    return si;
}

/* A seat occupant (part.h) as a SavePart of its capsule's `seats`. Same capture
   as an inventory item -- a suit is a Part like any other -- plus the identity
   the suit owns (Part::kerbal). */
SavePart saveSeatPart(const Part *suit) {
    SavePart sp = saveItemPart(suit);
    if(suit->kerbal != nullptr) {
        sp.seat_profile = suit->kerbal->id;
        sp.seat_name = suit->kerbal->name;
    }
    if(suit->container != nullptr) {
        for(size_t i = 0; i < suit->container->seats.size(); i++) {
            if(suit->container->seats[i] == suit) {
                sp.seat_slot = (int)i;
                break;
            }
        }
    }
    return sp;
}

/* A suit's pockets are validated by the inventory rules below, which a seat
   array therefore needs before it is defined. */
static std::string validateInventoryArray(const PartsCatalog &cat,
                                          const std::string &shipName,
                                          const PartDef &pd,
                                          const std::vector<SavePart> &saved);

/* The seat-array rules, world-free. `pd` is the CONTAINER's def: buildSeatParts
   passes the built capsule's, preflightSave looks it up in the catalog, and it
   is the same def either way -- which is the point. One implementation for both
   paths, so a cross-system load cannot switch systems and only then refuse
   (#302). The roster-id rule sits with the builder because only it holds the
   cast list; preflightSave has the same rule via refuseUnknownRosterId. */
static std::string validateSeatArray(const PartsCatalog &cat,
                                     const std::string &shipName,
                                     const PartDef &pd,
                                     const std::vector<SavePart> &saved) {
    if(saved.empty()) { return ""; }   // the overwhelmingly common case
    if(pd.crew_capacity <= 0) {
        return "load: '" + shipName + "' part '" + pd.name + "' holds "
               + std::to_string(saved.size())
               + " seat(s) but is not a capsule (crew_capacity 0)";
    }
    // a live board refuses a full capsule; a load must too
    if((int)saved.size() > pd.crew_capacity) {
        return "load: '" + shipName + "' capsule '" + pd.name + "' has "
               + std::to_string(saved.size()) + " seat(s), more than its "
               + std::to_string(pd.crew_capacity) + "-seat capacity";
    }
    /* Explicit seat_slot values must be in range and unique. addSeat would
       refuse the second claim of a slot only once the capsule is BUILT, which
       on a cross-system load is after the switch already tore the career down
       (#305). The capacity rule above cannot see this: two entries can name
       the same slot while the count still fits. */
    std::set<int> namedSlots;
    for(const SavePart &sp : saved) {
        if(sp.seat_slot < 0) { continue; }
        if(sp.seat_slot >= pd.crew_capacity) {
            return "load: '" + shipName + "' capsule '" + pd.name
                   + "' names seat slot " + std::to_string(sp.seat_slot)
                   + " but has only " + std::to_string(pd.crew_capacity)
                   + " seat(s)";
        }
        if(!namedSlots.insert(sp.seat_slot).second) {
            return "load: '" + shipName + "' capsule '" + pd.name
                   + "' has two seats naming slot " + std::to_string(sp.seat_slot);
        }
    }
    for(const SavePart &sp : saved) {
        const PartDef *sd = cat.find(sp.part);
        if(sd == nullptr) {
            return "load: '" + shipName + "' seat part '" + sp.part
                   + "' is not in the parts file";
        }
        /* A seat is a PERSON, and only a kerbal-type Part is one (Part::isSeat).
           Anything else here is a hand-edited file, not a game state. */
        if(sd->type != "kerbal") {
            return "load: '" + shipName + "' seat part '" + sp.part + "' is a '"
                   + sd->type + "' part, not a kerbal seat";
        }
        /* A seat cannot hold seats, so a nested array is corruption rather than
           a state the game can be in -- and dropping it silently would lose a
           person. */
        if(!sp.seats.empty()) {
            return "load: '" + shipName + "' seat part '" + sp.part + "' has "
                   + std::to_string(sp.seats.size()) + " seat(s) of its own";
        }
        // A suit carries pockets, and buildSeatParts fills them (saveSeatPart
        // writes them back), so the same inventory rules apply one level down.
        if(std::string why = validateInventoryArray(cat, shipName, *sd, sp.inventory);
           !why.empty()) {
            return why;
        }
    }
    return "";
}

/* The inventory-array rules, world-free, recursing through nested containers
   exactly as buildInventoryItems does. Deliberately NOT the uid rules: an item's
   uid never enters savedUidToPart, so refusing a repeated item uid here would
   reject saves that load today. */
static std::string validateInventoryArray(const PartsCatalog &cat,
                                          const std::string &shipName,
                                          const PartDef &pd,
                                          const std::vector<SavePart> &saved) {
    // a live add refuses a full container; a load must too
    if((int)saved.size() > pd.inventory_capacity) {
        return "load: '" + shipName + "' part '" + pd.name + "' holds "
               + std::to_string(saved.size()) + " inventory item(s), more than its "
               + std::to_string(pd.inventory_capacity) + "-slot capacity";
    }
    for(const SavePart &si : saved) {
        /* Seats belong to capsules, and buildSeatParts only ever runs for an
           ATTACHED part -- so a `seats` array on an inventory item (or on a
           nested container) has nowhere to be parked. Dropping it would lose a
           person without a word. */
        if(!si.seats.empty()) {
            return "load: '" + shipName + "' inventory item '" + si.part + "' holds "
                   + std::to_string(si.seats.size())
                   + " seat(s); only a capsule has seats";
        }
        const PartDef *ipd = cat.find(si.part);
        if(ipd == nullptr) {
            return "load: '" + shipName + "' inventory item '" + si.part
                   + "' is not in the parts file";
        }
        /* A kerbal-type part is a person, not cargo. Parked anywhere other than
           a capsule it is a seat occupant in a non-capsule, which
           checkPartInvariants refuses -- and rebuildCompound asserts on that, so
           the save would abort the process. A hand-edited save is still just
           data: refuse it here, where refusing is an option. */
        if(ipd->type == "kerbal") {
            return "load: '" + shipName + "' inventory item '" + si.part
                   + "' is a crew part, not cargo";
        }
        if(std::string why = validateInventoryArray(cat, shipName, *ipd, si.inventory);
           !why.empty()) {
            return why;
        }
    }
    return "";
}

// Rebuild `saved` (and its nested items) as items of `container`. Throws on an
// unknown def or over-capacity; items wired before the throw hang off
// `container`, so the caller frees them with the owning vehicle.
void buildInventoryItems(Game &g, const std::vector<SavePart> &saved,
                         Part *container, const std::string &shipName) {
    /* Every rule but the wiring lives in validateInventoryArray, which
       preflightSave also runs before a cross-system switch (#302). */
    if(std::string why = validateInventoryArray(g.parts, shipName, *container->def,
                                                saved); !why.empty()) {
        throw std::runtime_error(why);
    }
    const PartsCatalog &cat = g.parts;
    for(size_t i = 0; i < saved.size(); i++) {
        const SavePart &si = saved[i];
        const PartDef *pd = cat.find(si.part);   // validated above
        Part *item = make_part(*pd, si.mass, si.hull_margin,
                               &si.fuel, &si.experiments, si.id, g.partsshader);
        container->contents.push_back(item);
        item->container = container;
        item->owner = container->owner;   // the vehicle the container is on
        buildInventoryItems(g, si.inventory, item, shipName);
    }
}

/* One SavePart -> a suit Part: body + def + fuel + findings + the identity it
   OWNS (Part::kerbal). A capsule's seat occupant and a kerbal standing on the
   home pad are the same Part; only the hold edge differs, so each caller
   validates and wires its own. `rosterSrc` is the roster entry the save names
   (null = anonymous): the roster owns the spelling and the skills, so a found
   entry wins over the name the file carries. */
Part *makeSuitPart(Game &g, const SavePart &sp, const PartDef *pd,
                   const KerbalProfile *rosterSrc) {
    Part *suit = make_part(*pd, sp.mass, sp.hull_margin,
                           &sp.fuel, &sp.experiments, sp.id, g.partsshader);
    KerbalProfile prof;
    prof.id = sp.seat_profile;
    prof.name = sp.seat_name;
    if(rosterSrc != nullptr) { prof = *rosterSrc; }
    suit->kerbal = std::make_unique<KerbalProfile>(std::move(prof));
    return suit;
}

/* Rebuild `saved` as the seat occupants (part.h) of the capsule `cap`: suit
   Parts parked in `cap->seats` and owned by the capsule, so it frees them.
   No Kerbal vehicle exists until someone goes EVA (Game::kerbalEVA). Throws to
   refuse the load on a non-capsule, an over-full capsule, a non-kerbal part or a
   roster id the roster has no entry for: a hand-edited save is still just data,
   and refusing is an option HERE -- parking anything else in a capsule breaks
   checkPartInvariants, which rebuildCompound asserts on two calls later.
   Called before Vehicle::finalize, so the compound already carries the crew
   mass (Part::effectiveMass reads the seats edge). */
void buildSeatParts(Game &g, const std::vector<SavePart> &saved, Part *cap,
                    const std::string &shipName) {
    if(saved.empty()) { return; }   // the overwhelmingly common case
    /* The shape rules live in validateSeatArray, which preflightSave also runs
       before a cross-system switch (#302). What stays here needs the roster or
       the built capsule. */
    if(std::string why = validateSeatArray(g.parts, shipName, *cap->def, saved);
       !why.empty()) {
        throw std::runtime_error(why);
    }
    for(size_t i = 0; i < saved.size(); i++) {
        const SavePart &sp = saved[i];
        /* Identity lives in the roster, so naming one means finding it there:
           rebuilding past a missing entry would silently strip that person's
           skills and name. Same rule and same words as preflightSave (#302). */
        if(std::string why = refuseUnknownRosterId(g.roster, sp.seat_profile,
                                                  "'" + shipName + "' seat");
           !why.empty()) {
            throw std::runtime_error(why);
        }
        const KerbalProfile *profile = (sp.seat_profile != 0)
            ? g.roster.find(sp.seat_profile) : nullptr;
        Part *suit = makeSuitPart(g, sp, g.parts.find(sp.part), profile);
        /* The seat edge before the pockets: buildInventoryItems copies the
           container's owner onto each item. A throw leaves everything wired into
           the capsule, which the caller's rollback frees with the ship. */
        if(!cap->addSeat(suit, (sp.seat_slot >= 0) ? (size_t)sp.seat_slot : (size_t)-1)) {
            delete suit;
            throw std::runtime_error("load: '" + shipName + "' capsule '"
                                     + cap->def->name + "' has no free seat for '"
                                     + sp.part + "'");
        }
        suit->owner = cap->owner;   // the vehicle the capsule rides
        buildInventoryItems(g, sp.inventory, suit, shipName);
        printf("[crew] loaded '%s' aboard '%s'\n",
               seatName(suit).c_str(), shipName.c_str());
    }
}

// Recursive count: the load summary prints the BUILT total, so a silently
// dropped nested item shows up as a smaller number. A seat occupant is crew,
// not cargo, so it is not counted -- but what its suit carries is.
size_t countPartInventory(Part *p) {
    size_t n = 0;
    for(Part *c : p->contents) {
        n += countPartInventory(c);
        n++;
    }
    for(Part *suit : p->seats) {
        if(suit != nullptr) { n += countPartInventory(suit); }
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
           a seat on a capsule Part and is captured in that part's `seats`. */
        Kerbal *k = static_cast<Kerbal *>(v);
        s.profile_id = k->profileId();
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
            for(Part *c : v->parts[0]->contents) {
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
        for(Part *c : p->contents) {
            sp.inventory.push_back(saveItemPart(c));
        }
        /* Seat occupants (part.h) ride in the part that holds them: a person
           is a seat on a capsule, not a vehicle of their own. */
        for(Part *suit : p->seats) {
            if(suit != nullptr) {
                sp.seats.push_back(saveSeatPart(suit));
            }
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
       so a non-null target is in Game::vehicles. Assert it: the slug map is
       keyed by pointer, so a recycled address would resolve to a VALID slug for
       the wrong ship, and the lenient fallback below would hide that. */
    assert(v->dockTargetShip == nullptr || slugs.count(v->dockTargetShip) != 0);
    /* A dock INTENT, not a seam: if the target somehow has no slug, dropping
       the intent loses nothing the player cannot re-target. */
    s.dock_target_ship = slugOf(slugs, v->dockTargetShip);
    s.dock_target_port = (v->dockTargetPort != nullptr) ? v->dockTargetPort->uid : 0;
    s.dock_arm_port = (v->dockArmPort != nullptr) ? v->dockArmPort->uid : 0;
    return s;
}

// ---- restore (SaveShip -> Game) --------------------------------------------

/* A save's body names resolve against the world they land in. A NON-EMPTY name
   the system lacks is a dangling reference -- the body was renamed or removed
   under the save -- and substituting a different body re-reads the saved
   position in the wrong frame. Reproduced: a ship parked 10 km above Minmus
   loads 540 km inside Kerbin and the game reports it as "flying low" (#303).
   Every other layer already refuses this (system.cpp's orbit and home lookups,
   buildings.cpp, --startship); the save layer was the one that papered over it.

   An EMPTY name is not dangling: the writer emits "" for a vehicle with no home
   or no SOI parent, so the caller keeps its fallback. */
static TerrainBody *resolveBody(const System &sys, const std::string &name,
                                const std::string &what, const char *field) {
    TerrainBody *b = sys.find(name);
    if(b == nullptr) {
        throw std::runtime_error("load: " + what + " has a " + field
                                 + " body '" + name
                                 + "' that is not in this system");
    }
    return b;
}

/* The save checks that need no world: a uid present, unique within the ship and
   across the whole save, every part name still in the catalog, every parent uid
   already seen (the writer emits parents before children), a scenario the table
   knows, and every uid the ship references -- controller, fuel link, dock seam
   -- one of its own parts. Returns "" when the ship is loadable, else the reason
   in the same words the builders used to throw, so both paths refuse identically.

   buildShipFromSaveParts relies on this; preflightSave runs it over the whole
   save BEFORE a cross-system switch tears the running game down (#302). */
static std::string validateShipParts(const SaveShip &s, const PartsCatalog &cat,
                                     std::set<uint64_t> &saveUids) {
    std::set<uint64_t> prior;   // uids at earlier indices of THIS ship
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        /* Refuse rather than guess: uid 0 is a save predating part identity,
           a repeated uid is a corrupt or hand-edited file. */
        if(sp.uid == 0) {
            return "load: saved ship '" + s.name + "' part '" + sp.id
                   + "' has no uid (save predates part identity)";
        }
        if(prior.count(sp.uid) != 0) {
            return "load: saved ship '" + s.name + "' has two parts with uid "
                   + std::to_string(sp.uid);
        }
        if(cat.find(sp.part) == nullptr) {
            return "load: saved ship '" + s.name + "' has a part '" + sp.part
                   + "' that is no longer in the parts file";
        }
        if(i > 0 && prior.count(sp.parent) == 0) {
            return "load: saved ship '" + s.name + "' part '" + sp.id
                   + "' has an unknown parent uid " + std::to_string(sp.parent);
        }
        prior.insert(sp.uid);
        if(saveUids.insert(sp.uid).second == false) {
            return "load: two ships claim part uid " + std::to_string(sp.uid);
        }
    }
    /* References the ship resolves against its OWN uid set. The builders look
       these up the same way, so checking them here is free -- and it is what
       keeps a corrupt file from costing a career on the cross-system path. */
    if(s.controller != 0 && prior.count(s.controller) == 0) {
        return "load: saved ship '" + s.name + "' names a controller uid "
               + std::to_string(s.controller) + " it does not have";
    }
    for(const SaveFuelLink &fl : s.fuel_links) {
        if(prior.count(fl.from) == 0 || prior.count(fl.to) == 0) {
            return "load: saved ship '" + s.name + "' has a fuel link between uids "
                   + std::to_string(fl.from) + " and " + std::to_string(fl.to)
                   + ", parts it does not have";
        }
    }
    for(const SaveDock &d : s.docks) {
        if(prior.count(d.port) == 0 || prior.count(d.root) == 0) {
            return "load: saved ship '" + s.name + "' has a dock seam ("
                   + std::to_string(d.port) + ", " + std::to_string(d.root)
                   + ") naming parts it does not have";
        }
    }
    return "";
}

/* The kerbal-def checks, world-free: the def file must exist and describe
   exactly one kerbal part. `def` (optional) takes the parsed def so the builder
   does not read the file twice. "" = fine.

   preflightSave runs this before a cross-system switch: a missing def used to
   throw only after the teardown, which cost the career (#302). */
static std::string validateKerbalDef(const SaveShip &s, const PartsCatalog &cat,
                                     ShipDef *def) {
    ShipDef parsed;
    try {
        parsed = load_ship_def(resdir::path(s.defPath).c_str(), cat);
    } catch(const std::exception &e) {
        return e.what();
    }
    /* The same rule make_suit states for a suit, and it matters twice as much
       here: build_ship would happily build a four-part ship from a racer def and
       call the result a person. Its extra parts never reach the writer's
       suit_inventory, so they vanish on the next save, and Game::kerbalBoard
       asserts a kerbal has exactly one part the moment anyone boards. */
    if(parsed.parts.size() != 1) {
        return "load: kerbal '" + s.name + "' has def '" + s.defPath
               + "', which is not exactly one part (got "
               + std::to_string(parsed.parts.size()) + ")";
    }
    if(parsed.parts[0].def->type != "kerbal") {
        return "load: kerbal '" + s.name + "' has def '" + s.defPath
               + "', which is a '" + parsed.parts[0].def->type
               + "' part, not a kerbal";
    }
    if(def != nullptr) { *def = std::move(parsed); }
    return "";
}

/* The crew-pool checks that need no world: a pool entry is a person standing on
   the pad, so it must be a known kerbal part carrying no seats of its own --
   the same rule buildSeatParts gives a seat (a non-kerbal here breaks
   checkPartInvariants; a nested seat array would silently lose a person). */
static std::string validateCrewPool(const std::vector<SavePart> &pool,
                                    const PartsCatalog &cat,
                                    const CrewRoster &roster) {
    for(const SavePart &sp : pool) {
        const PartDef *pd = cat.find(sp.part);
        if(pd == nullptr) {
            return "load: crew pool entry '" + sp.part
                   + "' is not in the parts file";
        }
        if(pd->type != "kerbal") {
            return "load: crew pool entry '" + sp.part + "' is a '" + pd->type
                   + "' part, not a kerbal";
        }
        if(!sp.seats.empty()) {
            return "load: crew pool entry '" + sp.part + "' has "
                   + std::to_string(sp.seats.size()) + " seat(s) of its own";
        }
        if(std::string why = refuseUnknownRosterId(roster, sp.seat_profile,
                                                  "crew pool entry '" + sp.part
                                                  + "'"); !why.empty()) {
            return why;
        }
        // What the suit carries: load_game fills its pockets with
        // buildInventoryItems under the name "crew pool", so same rules, same
        // words, one level down.
        if(std::string why = validateInventoryArray(cat, "crew pool", *pd,
                                                    sp.inventory); !why.empty()) {
            return why;
        }
    }
    return "";
}

// Parts are rebuilt from the SOLVED poses, not a re-derived attach spec: a
// docking seam's relative geometry is not recoverable from an attach spec.
// `savedUidToPart` (optional) is the caller's because it must span every ship
// file -- a dock target's port lives in another ship.
Vehicle *buildShipFromSaveParts(Game &g, const LoadWorld &w, const SaveShip &s,
                                std::map<uint64_t, Part *> *savedUidToPart) {
    const PartsCatalog &cat = g.parts;
    /* Validate before constructing: nothing to unwind, and the same walk is what
       preflightSave runs before a cross-system switch (#302). */
    std::set<uint64_t> saveUids;
    if(std::string why = validateShipParts(s, cat, saveUids); !why.empty()) {
        throw std::runtime_error(why);
    }
    // Resolve the scenario BEFORE constructing: it throws, and the rollback
    // only walks `built`.
    const ScenarioDef *scn = resolveScenario(s.scenario);
    Vehicle *v = new Vehicle;
    v->name = s.name;
    v->defPath = s.defPath;
    std::map<uint64_t, size_t> uidToIndex;
    std::map<uint64_t, Part *> uidToPart;   // controller / seams / fuel links
    for(size_t i = 0; i < s.parts.size(); i++) {
        const SavePart &sp = s.parts[i];
        const PartDef *pd = cat.find(sp.part);
        assert(pd != nullptr);   // validateShipParts refused it otherwise
        Part *p = make_part(*pd, sp.mass, sp.hull_margin,
                            &sp.fuel, &sp.experiments, sp.id, g.partsshader);
        // What only a save knows: where the part sits in the staging order and
        // whether its chute was deployed when the ship was saved.
        p->stage = sp.stage;
        p->chute = (sp.chute > 0) ? ChuteDeployed : ChuteStowed;
        if(i == 0) {
            v->setRoot(p);
        } else {
            std::map<uint64_t, size_t>::const_iterator it = uidToIndex.find(sp.parent);
            assert(it != uidToIndex.end());   // validateShipParts refused it
            v->attach(p, it->second, sp.pos, sp.rot);
        }
        uidToIndex[sp.uid] = i;
        uidToPart[sp.uid] = p;
        if(savedUidToPart != nullptr
           && !savedUidToPart->insert(std::make_pair(sp.uid, p)).second) {
            /* This IS the cross-ship uid check on a same-system load: the
               validateShipParts call above gets a fresh set per ship, so its
               cross-ship half only bites in preflightSave, which that path never
               runs. The caller's map spans every ship, so it catches what the
               per-ship walk cannot. */
            delete v;   // ~Vehicle frees the parts, including p
            throw std::runtime_error("load: two ships claim part uid "
                                     + std::to_string(sp.uid));
        }
        // A refused item or seat frees through `delete v`: everything wired so
        // far hangs off p (items in its inventory, people in its seats) and goes
        // with it.
        try {
            buildInventoryItems(g, sp.inventory, p, s.name);
            buildSeatParts(g, sp.seats, p, s.name);
        } catch(const std::exception &) {
            delete v;
            throw;
        }
    }
    /* The tree is complete, so each attached part's shroud can be reported the
       way the def-spawn path reports its own (vehicle.cpp). A load that resolves
       no shroud loses a cylinder the player can see, and until #278's fold it
       resolved none at all -- silently. Hand-authored fixtures carry empty part
       ids, so the ship name goes on the line too, and it is flushed: a load that
       refuses further down (the controller check below) would otherwise take the
       line with it in a redirected run. */
    for(Part *p : v->parts) {
        if(p->shroud == nullptr) { continue; }
        printf("[shroud] %s: %s (%s over %s) ship='%s'\n", p->id.c_str(),
               v->hasChildBelow(p) ? "shrouded" : "bare",
               p->def->shroud.c_str(), p->def->mesh.c_str(), s.name.c_str());
        fflush(stdout);
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
    /* The containment tree is complete here, so this is the last point where a
       bad one can still be REFUSED: finalize() -> rebuildCompound() asserts the
       same invariants, and an assert is not a refusal. A hand-edited save can
       nest parts deeper than checkContainmentTree's cap, or park a seat where no
       capsule is, and neither is reachable from the live game. The [part] lines
       already named the offender. */
    if(!v->checkPartInvariants()) {
        delete v;
        throw std::runtime_error("load: saved ship '" + s.name
                                 + "' has a broken containment tree (see [part] above)");
    }
    v->finalize();
    v->activeStage_ = s.active_stage;
    v->totalStages_ = s.total_stages;

    const std::string who = "saved ship '" + s.name + "'";
    /* An empty name is not dangling -- the writer emits "" for a vehicle with no
       home / no SOI parent -- but it is unreachable from a live save, so a load
       that lands here is a hand-authored file quietly re-parking the ship on
       another body. Say so (#303). */
    if(s.home.empty()) {
        printf("[load] '%s' saved no home body: using %s\n",
               s.name.c_str(), w.home->name.c_str());
        v->home = w.home;
    } else {
        v->home = resolveBody(w.sys, s.home, who, "home");
    }
    v->scenario = scn;
    // Vehicle::slot is deliberately NOT restored: the next placement recomputes it.
    v->sun = w.sun;
    // BEFORE setSoi, whose observe is then a no-op on the unchanged body.
    v->flog = s.flog;
    if(s.pose.body.empty()) {
        printf("[load] '%s' saved no pose body: placing around %s\n",
               s.name.c_str(), v->home->name.c_str());
    }
    TerrainBody *pb = s.pose.body.empty() ? v->home
                                          : resolveBody(w.sys, s.pose.body, who, "pose");
    v->setSoi(s.pose.rotating ? pb->rot_frame : pb->frame, w.time);

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
            /* `v` has not been adopted into Game::vehicles yet (load_game
               commits the whole built list on success), so freeing it here
               strands nothing. */
            delete v;
            throw std::runtime_error("load: saved ship '" + s.name + "' has a dock seam ("
                                     + s.docks[k].name + ") naming a part it does not have");
        }
        v->seams.push_back(Vehicle::DockSeam{ portIt->second, rootIt->second, s.docks[k].name });
    }
    return v;
}

// build_ship's init() re-seeds the suit tank full, so the saved contents must
// be restored after it. A WALKING kerbal only -- a seated one is a `seats`
// element of its capsule's part entry, not a vehicle (part.h).
Kerbal *buildKerbalFromSave(Game &g, const LoadWorld &w, const SaveShip &s) {
    /* The roster is where identity lives, so a kerbal naming one must find it
       there: rebuilding past a missing entry would silently strip that person's
       skills and name. Checked before the build so a throw frees nothing but `k`
       itself. Same rule and same words as preflightSave (#302). */
    if(std::string why = refuseUnknownRosterId(g.roster, s.profile_id,
                                              "kerbal '" + s.name + "'");
       !why.empty()) {
        throw std::runtime_error(why);
    }
    const KerbalProfile *profile = (s.profile_id != 0)
        ? g.roster.find(s.profile_id) : nullptr;
    /* A ShipDef owns nothing, so throwing here frees nothing. The rule and its
       wording live in validateKerbalDef, which preflightSave also runs (#302). */
    ShipDef def;
    if(std::string why = validateKerbalDef(s, g.parts, &def); !why.empty()) {
        throw std::runtime_error(why);
    }
    Kerbal *k = new Kerbal;
    k->name = s.name;
    /* The roster owns the name, so it wins over the saved one: a rename in the
       save's own roster reaches the kerbal instead of leaving two spellings. */
    if(profile != nullptr) { k->name = profile->name; }
    k->defPath = s.defPath;
    k->home = w.home;
    k->scenario = nullptr;
    k->m_parent = w.home;
    k->sun = w.sun;
    k->frame = w.home->rot_frame;
    /* build_ship can throw with parts attached; k is not in the load's
       `built` list yet, so the rollback would not free it. */
    try {
        // base 0 + no pad lift: the saved pose is applied below
        build_ship(k, def, g.partsshader, glm::dvec3(0.0), glm::dmat3(1.0), 0.0);
    } catch(...) {
        delete k;
        throw;
    }
    /* Identity is owned by the suit Part (part.h). */
    if(!k->parts.empty()) {
        KerbalProfile prof;
        prof.id = s.profile_id;
        prof.name = k->name;
        if(profile != nullptr) { prof = *profile; }
        k->parts[0]->kerbal = std::make_unique<KerbalProfile>(std::move(prof));
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
    /* Walking: the saved pose is the kerbal's own. A seated person is not a
       vehicle at all -- they are a `seats` element of their capsule (part.h),
       built by buildSeatParts. */
    TerrainBody *body = s.pose.body.empty()
        ? w.home
        : resolveBody(w.sys, s.pose.body, "saved kerbal '" + s.name + "'", "pose");
    k->home = body;
    k->setSoi(s.pose.rotating ? body->rot_frame : body->frame, w.time);
    k->placeShip(s.pose.pos, s.pose.rot);
    k->setVelocity(s.pose.vel);
    SetAngVelocity(k->hull, s.pose.angvel);
    /* NOT goOnRails here: the commit runs it after setTime + the heavy phase,
       because isGrounded reads terrain height and a build against an
       uninstalled system has no ground yet (#305). The save's `onRails` is
       restored in the commit's phase 2, like every ship. */
    /* The identity binding is the one fact about a loaded kerbal the vehicle
       list cannot show, so make it observable: a save that silently rebuilds
       anonymous people is exactly the bug this line exists to catch. */
    printf("[crew] loaded '%s' on EVA\n", k->name.c_str());
    return k;
}

/* One roster entry, one wearer. A save that names one id twice LOADS -- both
   people print, both ride the compound -- and the duplicate only surfaces as a
   wrong pick or a shrink later. So refuse it here, where the ids are read.
   `pool` is the home pad's standing crew (save.json's crew_pool): they wear
   their ids too, and a pool entry clashing with another pool entry or with
   someone already in Game::vehicles is the same corruption. Anonymous people
   (id 0) may repeat: they claim nothing. */
static void refuseDoubleClaim(const std::vector<Vehicle *> &built,
                              const std::vector<Part *> &pool) {
    std::set<uint64_t> worn;
    for(const Part *suit : pool) {
        if(suit == nullptr || suit->kerbal == nullptr) { continue; }
        if(std::string why = refuseDoubleClaimId(worn, suit->kerbal->id);
           !why.empty()) {
            throw std::runtime_error(why);
        }
    }
    for(const Vehicle *v : built) {
        if(v->isEva()) {
            if(std::string why = refuseDoubleClaimId(
                   worn, static_cast<const Kerbal *>(v)->profileId());
               !why.empty()) {
                throw std::runtime_error(why);
            }
            continue;
        }
        for(const Part *p : v->parts) {
            for(const Part *suit : p->seats) {
                if(suit == nullptr || suit->kerbal == nullptr) { continue; }
                if(std::string why = refuseDoubleClaimId(worn, suit->kerbal->id);
                   !why.empty()) {
                    throw std::runtime_error(why);
                }
            }
        }
    }
}

/* The roster swap in load_game belongs to its transaction. A load that refuses
   must leave the cast list the RUNNING world wears: the suits standing on the
   pad and the people in seats keep the profile ids they were minted with, so a
   roster that no longer names them makes the NEXT save one this very loader
   refuses ("crew pool entry names roster id N") -- a career the game wrote and
   cannot read. Restores the prior list on scope exit unless committed. */
struct RosterRollback {
    CrewRoster &roster;
    std::vector<KerbalProfile> prior;
    bool swapped = false;     // did this load actually replace the cast list?
    bool committed = false;

    explicit RosterRollback(CrewRoster &r) : roster(r), prior(r.profiles) { }
    void commit() { committed = true; }
    ~RosterRollback() {
        if(committed || !swapped) { return; }
        // Same voice as the other refusal diagnostics (Game::loadFrom): a
        // refused load says what it left alone.
        printf("[load] roster restored: %zu kerbal(s)\n", prior.size());
        fflush(stdout);
        roster.profiles = std::move(prior);
    }
};

/* The roster a load binds people to: the save's own when it carries one -- so a
   save stays itself after --kerbals is edited -- otherwise the running cast list
   (a save predating the roster). Every roster id the save names is looked up in
   THIS, so preflightSave must check against it too: checking the list standing
   now would refuse a save that merely disagrees with an edited data file, which
   is the exact thing a save carries its roster to survive (#302). */
CrewRoster rosterFromSave(const SaveMeta &meta, const CrewRoster &running) {
    if(meta.roster.empty()) { return running; }
    CrewRoster next;
    for(const SaveKerbal &sk : meta.roster) {
        KerbalProfile p;
        p.id = sk.id;
        p.name = sk.name;
        for(int r = 0; r < NumSkills; r++) { p.skill[r] = sk.skill[r]; }
        next.profiles.push_back(p);
    }
    return next;
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
    /* Unassigned kerbals standing on the home pad (the building pool). */
    if(StaticBuilding *home = homeCrewPad(g.sys)) {
        for(Part *suit : home->crew) {
            if(suit != nullptr) { meta.crew_pool.push_back(saveSeatPart(suit)); }
        }
    }
    meta.time = g.time;
    meta.time_accel = g.time_accel;
    meta.exhaust_scale = g.args.exhaust_scale;
    meta.science_score = g.science.score;
    meta.recovered = g.science.recovered;
    meta.saved_at = nowString();

    /* The save's ships in slug order: every VEHICLE. A seated person is not in
       this list -- they are a `seats` element of their capsule's part entry
       (part.h), so a slug never names a person. Slugs follow spawn order, so a
       load that adopts in save order round-trips v0/v1/.. exactly. */
    const std::vector<Vehicle *> &vehicles = g.vehicles;
    /* Slugs for the WHOLE list first, before a single file is written: a
       ship's file references its dock target by slug, and the referent can sit
       later in the walk than the referrer. */
    std::map<const Vehicle *, std::string> slugs;
    for(size_t i = 0; i < vehicles.size(); i++) { slugs[vehicles[i]] = slug(i); }
    /* The active ship is a vehicle the player is flying, so it is in the list
       the walk just took. A miss means the two disagree, and "" would silently
       promote built.front() on the next load. */
    assert(g.ship == nullptr || slugs.count(g.ship) != 0);
    meta.active_ship = slugOf(slugs, g.ship);
    for(size_t i = 0; i < vehicles.size(); i++) {
        SaveShip s = saveShipFromVehicle(vehicles[i], slugs);
        meta.ships.push_back(slug(i));
        writeJson(dir + "/ships/" + slug(i) + ".json", saveShipToJson(s));
    }
    writeJson(dir + "/save.json", saveMetaToJson(meta));
    /* Drop ship files this game no longer has. A save directory is REUSED
       (quicksave rotation, Save-As), so a shrunk ship list leaves orphan
       ships/vN.json that nothing reads but that make the tree lie about its
       ships. No case pins the sweep yet (#309); a reused slot is where the
       stale file would show up. Done AFTER the writes, so a throw partway
       leaves the previous files alone rather than deleting first and writing
       half. */
    {
        namespace fs = std::filesystem;
        std::error_code ec;
        fs::directory_iterator it(dir + "/ships", ec), end;
        for(; !ec && it != end; it.increment(ec)) {
            const std::string ext = it->path().extension().string();
            /* A leftover <slug>.json.tmp is a writeJson that died between the
               write and the rename. Nothing ever reads it, so it is debris. */
            if(ext == ".tmp") {
                std::error_code rm;
                fs::remove(it->path(), rm);
                continue;
            }
            if(ext != ".json") { continue; }
            const std::string stem = it->path().stem().string();
            if(std::find(meta.ships.begin(), meta.ships.end(), stem)
               == meta.ships.end()) {
                std::error_code rm;
                fs::remove(it->path(), rm);
            }
        }
    }
    printf("Saved %zu ship(s) to %s (game '%s')\n", vehicles.size(), dir.c_str(),
           g.gameName.c_str());
}

/* True if the system has a VAB_LAUNCH_PAD. Checks the world_buildings list,
   not TerrainBody::pads, so it works on a system that is loaded but not yet
   installed (place_world_buildings is what fills pads -- and a cross-system
   preflight must not depend on it). Same data find_launch_site reads, one
   stage earlier. */
static bool systemHasCrewPad(const System &sys) {
    for(const WorldBuilding &w : sys.world_buildings) {
        if(w.hasFlag("VAB_LAUNCH_PAD")) { return true; }
    }
    return false;
}

/* Parse the whole save and run every check that can run before the world
   changes, building nothing. Deliberately the SAME parser, catalog, scenario
   table and validators the builders use, not a second set of rules: each check
   is one function that both paths call, so they cannot drift.

   `sys` is the system the ships will land in, so the body references check too --
   which is what keeps a dangling body name from costing the career behind the
   switch (#302), and from silently parking a ship inside the wrong body (#303).

   `roster` is here for the same reason: a save naming a roster entry the roster
   lacks used to be the builder's business only, so a CROSS-SYSTEM load of it
   switched systems -- tearing the running career down -- and only then refused.
   The ids are checked against the roster the LOAD will bind to (rosterFromSave),
   not the one standing now: a save carries its own cast list precisely so an
   edited --kerbals file cannot break it, and refusing that would be a new bug
   dressed up as a fix.

   What still throws only AFTER the switch is what genuinely needs a BUILT world:
   checkPartInvariants (the containment tree of a solved compound), Vehicle::
   addSeat refusing a slot the built capsule already used, and a part that parses
   but fails to build. Each costs a career on the cross-system path exactly as a
   dangling body name did, and each is a state the game cannot reach -- the rules
   above are the ones a hand-edited or older file hits. */
std::string preflightSave(const std::string &dir, const PartsCatalog &cat,
                          const System &sys, const CrewRoster &roster) {
    SaveMeta meta;
    try {
        meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    } catch(const std::exception &e) {
        return e.what();
    }
    /* A standing cast needs somewhere to stand. load_game throws this once the
       ships are built, which on the cross-system path is after the switch
       already tore the running game down (#302). systemHasCrewPad, not
       homeCrewPad: the incoming system's pads are not placed yet. */
    if(!meta.crew_pool.empty() && !systemHasCrewPad(sys)) {
        return "load: the save has " + std::to_string(meta.crew_pool.size())
               + " kerbal(s) standing at the launch site, but this system has no "
               "VAB_LAUNCH_PAD to stand them on";
    }
    const CrewRoster bind = rosterFromSave(meta, roster);
    if(std::string why = validateCrewPool(meta.crew_pool, cat, bind);
       !why.empty()) {
        return why;
    }
    std::set<uint64_t> saveUids;   // spans every ship, as the load's map does
    /* Roster ids span every ship AND the pad's pool, so the set does too --
       refuseDoubleClaim checks the same rule over the built list, which is what
       a same-system load gets (#302). */
    std::set<uint64_t> claimed;
    for(const SavePart &sp : meta.crew_pool) {
        if(std::string why = refuseDoubleClaimId(claimed, sp.seat_profile);
           !why.empty()) {
            return why;
        }
    }
    for(size_t i = 0; i < meta.ships.size(); i++) {
        SaveShip s;
        try {
            s = saveShipFromJson(
                readJsonFile(dir + "/ships/" + meta.ships[i] + ".json"));
        } catch(const std::exception &e) {
            return e.what();
        }
        if(std::string why = validateShipParts(s, cat, saveUids); !why.empty()) {
            return why;
        }
        if(s.is_crew) {
            ShipDef def;
            if(std::string why = validateKerbalDef(s, cat, &def); !why.empty()) {
                return why;
            }
            if(std::string why = refuseUnknownRosterId(bind, s.profile_id,
                                                      "kerbal '" + s.name + "'");
               !why.empty()) {
                return why;
            }
            if(std::string why = refuseDoubleClaimId(claimed, s.profile_id);
               !why.empty()) {
                return why;
            }
            /* A person's pockets hang off the one-part def validateKerbalDef
               just parsed -- buildKerbalFromSave fills them, and never reads the
               ship's `parts` array, which is why the walk below skips it. */
            if(!def.parts.empty()) {
                if(std::string why = validateInventoryArray(cat, s.name,
                                                           *def.parts[0].def,
                                                           s.suit_inventory);
                   !why.empty()) {
                    return why;
                }
            }
        } else {
            /* The seat and inventory rules the builders otherwise enforce only
               once a Part exists, from the same defs, plus the roster ids those
               builders look up (#302). */
            for(const SavePart &sp : s.parts) {
                const PartDef *pd = cat.find(sp.part);   // validateShipParts checked it
                if(std::string why = validateSeatArray(cat, s.name, *pd, sp.seats);
                   !why.empty()) {
                    return why;
                }
                if(std::string why = validateInventoryArray(cat, s.name, *pd,
                                                            sp.inventory);
                   !why.empty()) {
                    return why;
                }
                for(const SavePart &seat : sp.seats) {
                    if(std::string why = refuseUnknownRosterId(bind,
                                                              seat.seat_profile,
                                                              "'" + s.name
                                                              + "' seat");
                       !why.empty()) {
                        return why;
                    }
                    if(std::string why = refuseDoubleClaimId(claimed,
                                                            seat.seat_profile);
                       !why.empty()) {
                        return why;
                    }
                }
            }
        }
        try {
            resolveScenario(s.scenario);   // throws for a name the table lacks
        } catch(const std::exception &e) {
            return e.what();
        }
        // The same rule and the same words the builders use. A crew file's
        // `home` is never read (saveShipFromJson returns before it), so "" there
        // is normal, not dangling.
        const std::string who = "saved ship '" + s.name + "'";
        try {
            if(!s.home.empty()) { resolveBody(sys, s.home, who, "home"); }
            if(!s.pose.body.empty()) {
                resolveBody(sys, s.pose.body, who, "pose");
            }
        } catch(const std::exception &e) {
            return e.what();
        }
    }
    return "";
}

/* What a successful build produced, waiting to be installed. Nothing here is
   in the live world yet: on a cross-system load the incoming system is not
   even installed, so a refusal costs nothing (#305). Completes the opaque
   BuiltSave the header forward-declares.

   Owns the built vehicles and the pad's new pool. The destructor frees them,
   so a throw between a successful build and the commit cannot leak -- and
   discardSaveWorld is just `delete`. The roster swap rides along: a build
   that is never committed must put the running cast list back (the "career
   the game wrote and cannot read" failure). */
struct BuiltSave {
    SaveMeta meta;
    std::vector<SaveShip> saves;              // parsed, for the commit's phase 2
    std::vector<Vehicle *> vehicles;          // save order
    std::vector<Part *> pool;                 // the home pad's new standing cast
    std::map<std::string, Vehicle *> bySlug;
    std::map<uint64_t, Part *> savedUidToPart;
    std::string dir;
    // Roster rollback (RosterRollback's job, deferred to the commit).
    CrewRoster *roster = nullptr;
    std::vector<KerbalProfile> rosterPrior;
    bool rosterSwapped = false;

    ~BuiltSave() {
        for(Vehicle *v : vehicles) { delete v; }
        for(Part *p : pool) { delete p; }
        if(rosterSwapped && roster != nullptr) {
            printf("[load] roster restored: %zu kerbal(s)\n", rosterPrior.size());
            fflush(stdout);
            roster->profiles = std::move(rosterPrior);
        }
    }
    void commitRoster() { rosterSwapped = false; }
};

/* Build the whole save against `w`. Throws on any rule -- including the three
   that used to fire only after a cross-system switch had already torn the
   running career down (checkPartInvariants, addSeat, make_part/build_ship).
   The live game is untouched except a roster swap that rolls back on throw.
   Does NOT move the clock: the commit does, so a refusal leaves it alone. */
BuiltSave *buildSaveWorld(Game &g, const LoadWorld &w, const std::string &dir) {
    /* A load replaces the world wholesale, so nothing from the old one may
       still be waiting to die: retire() parked these out of Game::vehicles,
       where the commit's delete would never find them. */
    g.sweepRetired();
    std::unique_ptr<BuiltSave> out(new BuiltSave);
    out->dir = dir;
    out->meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    SaveMeta &meta = out->meta;
    /* The save's clock is the epoch the builders stamp setSoi with. The
       caller's w.time is only a hint (a cross-system load does not yet know
       the save's time when it prepares the target system). */
    LoadWorld ww{ w.sys, w.home, w.sun, meta.time };
    /* The save's roster REPLACES the --kerbals file's, so a save stays itself
       after the data file is edited; an absent roster (a save predating the
       roster) keeps the file's. Done before the ships are built, which resolves
       every kerbal's profile id against this.
       On SUCCESS the swap is permanent for the session: newGame and
       switchSystem do not re-read --kerbals, so a roster a save brought in stays
       in charge afterwards. On a REFUSAL BuiltSave's destructor rolls it back --
       the standing cast still wears the old ids. */
    out->roster = &g.roster;
    if(!meta.roster.empty()) {
        out->rosterPrior = g.roster.profiles;
        out->rosterSwapped = true;
        g.roster = rosterFromSave(meta, g.roster);
    }
    /* The building pool is the save's (format 4: crew_pool). Built OFF the pad
       and committed only once the whole load succeeds: a refused load must
       leave the running cast standing, exactly as it leaves the running ships.
       No fallback mint -- an empty pool is a spent cast, a legal state.
       The pad itself is resolved at commit: a cross-system build runs before
       place_world_buildings, so homeCrewPad is not meaningful yet. */
    if(!meta.crew_pool.empty() && !systemHasCrewPad(w.sys)) {
        throw std::runtime_error("load: the save has " + std::to_string(meta.crew_pool.size())
                                 + " kerbal(s) standing at the launch site, but this "
                                 + "system has no VAB_LAUNCH_PAD to stand them on");
    }

    for(size_t i = 0; i < meta.ships.size(); i++) {
        out->saves.push_back(saveShipFromJson(
            readJsonFile(dir + "/ships/" + meta.ships[i] + ".json")));
    }

    std::vector<Vehicle *> &built = out->vehicles;
    built.resize(out->saves.size(), nullptr);
    for(size_t i = 0; i < out->saves.size(); i++) {
        const SaveShip &s = out->saves[i];
        built[i] = s.is_crew
            ? buildKerbalFromSave(g, ww, s)
            : buildShipFromSaveParts(g, ww, s, &out->savedUidToPart);
        out->bySlug[meta.ships[i]] = built[i];
    }
    /* The pad's cast is built in the same transaction as the ships: the
       ship-file parse above can throw too, and a pool built before it would
       leak. Building it here also lets refuseDoubleClaim see the ids these
       suits wear. The part / type / seats rule -- roster ids included --
       lives in validateCrewPool, which preflightSave also runs before a
       cross-system switch (#302), so the loop below can trust every id it
       reads. */
    if(std::string why = validateCrewPool(meta.crew_pool, g.parts, g.roster);
       !why.empty()) {
        throw std::runtime_error(why);
    }
    for(const SavePart &sp : meta.crew_pool) {
        const PartDef *pd = g.parts.find(sp.part);   // validated above
        const KerbalProfile *src = (sp.seat_profile != 0)
            ? g.roster.find(sp.seat_profile) : nullptr;
        Part *suit = makeSuitPart(g, sp, pd, src);
        out->pool.push_back(suit);
        // What the suit carries rides home with it (saveSeatPart writes it).
        buildInventoryItems(g, sp.inventory, suit, "crew pool");
    }
    /* Needs the WHOLE built list, so it runs after the loop: two files can
       each load fine and still claim one roster entry between them. The
       pad's new pool wears ids too -- it is checked here and still
       uncommitted. */
    refuseDoubleClaim(built, out->pool);
    /* The build cannot refuse from here. The roster stays swapped until the
       commit adopts it -- BuiltSave's destructor rolls it back if the caller
       discards this (an install that throws after the build, say). */
    return out.release();
}

/* Adopt a successful build into the live game. Takes ownership of `b`.
   `fleetAlreadyGone` when installSystem already tore the old vehicles down
   (a cross-system load): resetWorld emptied Game::vehicles, so there is
   nothing left to delete. Moves the clock to the save's epoch and THEN parks
   the rails, because goOnRails captures the inertial state from the frame
   transforms as of that instant -- and isGrounded reads terrain, which a
   pre-install build cannot. */
void commitSaveWorld(Game &g, BuiltSave *bp, bool fleetAlreadyGone) {
    std::unique_ptr<BuiltSave> b(bp);
    SaveMeta &meta = b->meta;
    /* setTime, not a bare assignment: the clock jumps here and the load starts
       paused, so no tick re-derives the bodies' orbits/spin from the new epoch.
       AFTER the build: a refused load never moves it, so Game::loadFrom no
       longer needs its snapshot/restore (#301). */
    g.setTime(meta.time);
    g.time_accel = 0;
    // A save is not portable across exhaust scales, so the file wins over
    // Settings -- unless --exhaust-scale was given (cli_given beats files).
    if(!g.args.cli_given.exhaust_scale) {
        g.args.exhaust_scale = meta.exhaust_scale;
    }

    StaticBuilding *homePad = homeCrewPad(g.sys);
    if(homePad != nullptr) {
        for(Part *p : homePad->crew) { delete p; }
        homePad->crew = std::move(b->pool);
        if(!homePad->crew.empty()) {
            printf("[crew] %zu kerbal(s) standing at '%s' (from save)\n",
                   homePad->crew.size(), homePad->id.c_str());
        }
    } else {
        for(Part *p : b->pool) { delete p; }
    }
    b->pool.clear();

    // Success path only: a refused load must not clobber the running career
    // score / recovered log.
    g.science.setFrom(meta.science_score, meta.recovered);

    /* Game identity comes from the save's location, adopted here so a REFUSED
       load leaves the running identity untouched. Only a game dir under a
       saves/ parent carries one: that is what keeps a Game Debug fixture load
       (e2e/fixtures/<slot>, save.h kFixtureRoot) from adopting an identity a
       later Save would write back into the repo. A root named saves/ at any
       depth counts -- deliberate, and the only root that should. */
    {
        namespace fs = std::filesystem;
        const fs::path gamedir = fs::path(b->dir).parent_path();
        if(gamedir.parent_path().filename() == "saves") {
            const std::string dn = gamedir.filename().string();
            if(!dn.empty()) {
                g.gameId = dn;
                g.gameName = gameDisplayName(dn);
            }
        }
    }
    /* The save's ships take over the container in SAVE order, which is what
       makes a load -> resave round trip keep its v0/v1/.. put. part_sels holds
       Part* into the old ships. */
    g.part_sels.clear();
    g.clearFlightSummary();  // a prior recover's summary is not this save's
    if(!fleetAlreadyGone) {
        for(Vehicle *v : g.vehicles) { delete v; }
    }
    g.vehicles = b->vehicles;
    /* Adopted: ~BuiltSave must not free them. Everything below reads the fleet
       from g.vehicles -- b->vehicles is empty from here. */
    b->vehicles.clear();
    b->commitRoster();
    g.ship = nullptr;
    g.kerbal = nullptr;
    g.lastShip = nullptr;
    g.focusBody = 0;

    /* Phase 2: cross-references + world state. A dock INTENT naming a gone part
       stays lenient, unlike a dock SEAM (a dropped seam strands a real joint). */
    for(size_t i = 0; i < b->saves.size(); i++) {
        const SaveShip &s = b->saves[i];
        Vehicle *v = g.vehicles[i];
        if(!s.is_crew) {
            if(!s.dock_target_ship.empty()) {
                std::map<std::string, Vehicle *>::const_iterator t =
                    b->bySlug.find(s.dock_target_ship);
                if(t != b->bySlug.end()) {
                    v->dockTargetShip = t->second;
                    v->dockTargetPort = findSavedPart(b->savedUidToPart,
                                                      s.dock_target_port, t->second);
                }
            }
            v->dockArmPort = findSavedPart(b->savedUidToPart, s.dock_arm_port, v);
        }
        // A rebuilt kerbal wears the constructor's "standing" default until its
        // first tick, and the load starts paused: settle it from the pose.
        // BEFORE the rails branch -- goOnRails moves the pose into the body's
        // inertial node, and settleContact's terrain lookup wants the rotating
        // frame (railsTick converts for the same reason). enterWorld leaves the
        // pose where build put it, so either way it is valid here.
        if(v->isEva()) { static_cast<Kerbal *>(v)->settleContact(); }
        // build left the hull out of the world: this is the one place it is
        // added or parked. A non-rail-eligible ship stays live. Crew too --
        // a walking kerbal that was saved live is not on rails.
        if(s.onRails) {
            if(!v->goOnRails()) { v->enterWorld(); }
        } else {
            v->enterWorld();
        }
    }

    // the active ship
    /* Three states, and the save says which one it is (#270):
       - a slug: fly that vessel. saveMetaFromJson refuses a slug the "ships"
         list has no entry for -- NOT preflightSave, which the boot --load and
         quickload paths never run -- so a recorded name always resolves.
       - "" (key present): a fleet with nothing flown -- exactly what
         recoverActive leaves, since it does no handoff. Honour it; picking a
         vessel here would open a game the file never described.
       - no key at all: a hand-edited or foreign file. Fall back to the first
         vessel in save order, which is what a fresh boot makes active. This used
         to be byName.begin(), i.e. the alphabetically first NAME -- a kerbal
         called 'Ash' beat a ship called 'racer'. */
    Vehicle *active = nullptr;
    std::string activeSlug;
    if(!meta.active_ship.empty()) {
        std::map<std::string, Vehicle *>::const_iterator it =
            b->bySlug.find(meta.active_ship);
        assert(it != b->bySlug.end());   // saveMetaFromJson refused it otherwise
        active = it->second;
        activeSlug = meta.active_ship;
    } else if(!meta.active_ship_recorded && !g.vehicles.empty()) {
        active = g.vehicles[0];   // save order, so the slug is ships[0]
        activeSlug = meta.ships[0];
    }
    g.ship = active;
    g.kerbal = (active != nullptr && active->isEva()) ? static_cast<Kerbal *>(active) : nullptr;
    g.lastShip = nullptr;
    // The save may enter OR leave the no-ship state.
    g.syncShipFocus();
    size_t invItems = 0;
    for(size_t i = 0; i < g.vehicles.size(); i++) {
        for(Part *p : g.vehicles[i]->parts) { invItems += countPartInventory(p); }
    }
    if(invItems > 0) {
        printf("Loaded %zu inventory item(s) with the ships\n", invItems);
    }
    /* Name alone cannot tell two same-named ships apart (212), so the slug
       rides along outside the parens: the load picks by slug, and this is what
       says WHICH one. Cases match "(active: <name>)" as a substring. */
    if(active != nullptr) {
        printf("Loaded game from %s (active: %s) [%s]\n", b->dir.c_str(),
               active->name.c_str(), activeSlug.c_str());
    } else {
        printf("Loaded game from %s (active: (none))\n", b->dir.c_str());
    }
}

void load_game(Game &g, const std::string &dir) {
    /* Same-system entry (boot --load, and the in-game path after
       ensureSystemForSave already switched). Build against the live system,
       then commit. A cross-system load builds against the INCOMING system
       before installSystem, using these two halves directly (Game::loadFrom,
       #305). */
    SaveMeta meta = saveMetaFromJson(readJsonFile(dir + "/save.json"));
    LoadWorld w{ g.sys, g.home, g.sun, meta.time };
    BuiltSave *b = buildSaveWorld(g, w, dir);
    commitSaveWorld(g, b, false);
}

void discardSaveWorld(BuiltSave *b) {
    // ~BuiltSave frees the vehicles, the pool, and rolls the roster back.
    delete b;
}
