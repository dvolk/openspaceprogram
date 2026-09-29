#pragma once

// save.h -- saving + loading a running game (the live fleet + crew + clock).
//
// A save is a DIRECTORY:
//   dir/save.json          the global state + the ordered ship list
//   dir/ships/<slug>.json  one file per vehicle (a ship or a kerbal)
//
// The world is deterministic from the clock (the bodies' orbits/spin are
// functions of the analytic time, see frame.h), so saving the clock re-derives
// the whole system -- no per-body integration state is stored. What IS
// authoritative and must be saved is the fleet: each ship's part structure +
// tank contents + mass + pose + staging + docking + crew, and which ship is
// active. Everything else (the parts' physics, the compound body, the fuel
// groups) is re-derived by the build path on load.
//
// A ship's pose is saved in the ship's CURRENT frame (the SOI body's inertial
// frame for a coasting ship, its rotating surface frame for a grounded one)
// plus a `rotating` flag -- restoring exactly that frame is simpler and more
// faithful than normalizing to inertial. A part's pose is its ship-local
// frame-S pose (the root is always identity), so the whole part tree --
// including a docking seam or a dropped stage's rebase -- round-trips as the
// geometry it actually is, not a re-derived attach spec.
//
// The pure JSON (de)serialization below (SaveMeta / SaveShip <-> nlohmann) is
// header-only, as are the save-directory helpers (ensure_dir / list_saves /
// delete_save -- std::filesystem ops, no game state): both are
// unit-testable headless (tests/test_save.cpp) with no Game / Bullet link.
// The Game-coupled capture/restore (save_game / load_game) lives in save.cpp.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "flightlog.h"   // FlightLog / FlightEvent (pure containers, no game types)
#include "science.h"     // Experiment (science data on parts + the recover score)

struct Game;   // save_game / load_game take one; forward-declared so this
               // header stays free of game.h (and the unit test stays light)

/* Cross-part references are keyed by Part::uid, NOT by the def-authored
   instance id. `id` is unique only within one ship def, and a docked ship
   carries two ships' parts in one list without renaming either -- so two ships
   built from the same def collide on EVERY id, and resolving through an
   id-keyed map silently picked whichever duplicate was inserted last: a wrong
   controller after a round-trip, and a seam that reconstructed as the wrong
   part so undock failed and left the ship permanently un-undockable. A uid is
   minted per instance and distinct process-wide, so it stays a key across a
   merge.

   A saved uid is THIS SAVE's key for the part, not a value to restore: on load
   the rebuilt Part keeps the fresh uid its constructor minted, and the file's
   uids are used only to wire the references back up. That keeps the live uid
   sequence monotonic with no restore-and-bump-the-counter dance. Every part in
   one save has a distinct uid because they were all live in one process when
   it was written -- which is what lets the CROSS-ship references (a dock
   target's port) resolve through a single map. */

// A part instance in a saved ship. `part` is the catalog (def) name, `uid`
// the identity every reference names it by (must be nonzero -- a part with no
// uid is a save that predates part identity and is refused on load), `id` the
// def-authored instance id (kept: authoring, the VAB, diagnostics) and
// `parent` the parent's uid (0 = this is the root, which is legitimate).
// Note the two different meanings of 0: for a part's own `uid` it is a load
// error, but for a *reference* (parent, controller, dock ports) it is the
// legitimate "absent" default that load accepts. `pos`/`rot` are
// the part's ship-local frame-S pose (the root is identity -- setRoot forces
// it -- so they are the no-op defaults for part 0). `mass` is the part's OWN
// body mass (kg) -- a capsule's saved mass is just its own; aboard crew are
// separate SaveShips (is_crew) tracked through the containment edge, so their
// mass is NOT baked into the capsule here (phase 3 dropped the addPartMass
// bake; the compound derives it from the edge on load). `fuel` the
// per-ResourceType current tank contents (kg; the capacity is the def's), and
// `hull_margin` the part's resolved collision margin (the ship-level override
// already applied, read from Part::body->hull_margin) so the rebuilt hull
// matches the one that was saved.
struct SavePart {
    std::string part;
    uint64_t uid = 0;        // the key every cross-part reference uses; 0 = absent
    std::string id;          // def-authored instance id; NOT unique across a merge
    uint64_t parent = 0;     // the parent's uid; 0 for the root
    int stage = 1;
    glm::dvec3 pos = glm::dvec3(0.0);
    glm::dmat3 rot = glm::dmat3(1.0);
    double mass = 0.0;
    double hull_margin = -1.0;
    std::vector<double> fuel;   // one entry per ResourceType (Num)
    /* Science experiments held on this part (v1: kerbal suits, via
       SaveShip::suit_experiments -- kept here too so a future instrument
       part round-trips the same way). Empty = none. */
    std::vector<Experiment> experiments;
    /* Nested inventory items (phase 4.6): the parts parked in this part's
       inventory (Part::ownedContents). Serialized depth-first (a contained
       item is emitted inside its container, so load reconstructs the
       container before its items). Empty for parts with no inventory. */
    std::vector<SavePart> inventory;
};

// A one-way fuel link between two parts' fuel groups (from -> to), by uid.
struct SaveFuelLink {
    uint64_t from = 0;
    uint64_t to = 0;
};

// A ship's pose in its CURRENT frame (see the header). `body` is the SOI
// body's name, `rotating` whether that is the body's rotating surface frame.
struct SavePose {
    std::string body;
    bool rotating = false;
    glm::dvec3 pos = glm::dvec3(0.0);
    glm::dmat3 rot = glm::dmat3(1.0);
    glm::dvec3 vel = glm::dvec3(0.0);
    glm::dvec3 angvel = glm::dvec3(0.0);
};

// One dock seam (the joint of a ship this one absorbed; the last is the
// most recent / outermost dock, which undock selects).
struct SaveDock {
    uint64_t port = 0;      // this ship's port part uid
    uint64_t root = 0;      // the absorbed ship's root part uid
    std::string name;       // the absorbed ship's display name (restored on undock)
};

// One vehicle: a ship (is_crew false) or a kerbal (is_crew true). The ship
// fields are empty for a kerbal; the crew fields for a ship. `pose` is
// shared: a free (EVA) kerbal's world pose (an aboard one's is unused).
struct SaveShip {
    std::string name;
    std::string defPath;
    bool is_crew = false;
    SavePose pose;
    /* The vessel's flight journal (SoI enter/leave history + start instant).
       Persisted so a recovered vessel after a load shows its FULL mission,
       not a journal that restarts at the load instant. Restored into
       Vehicle::flog BEFORE the load's setSoi, so that setSoi's observe is a
       no-op on an unchanged body (the journal continues where it left off).
       A default (not-started) log is skipped on write and, on load, starts
       fresh at the placement -- so an old save with no flog still works. */
    FlightLog flog;

    // ship (is_crew false)
    std::string home;         // home body name
    std::string scenario;     // scenario name ("" = none)
    int slot = 0;
    std::vector<SavePart> parts;
    std::vector<SaveFuelLink> fuel_links;
    uint64_t controller = 0;  // part uid (0 = the default rule)
    bool onRails = false;     // coasting on the rails (ships; also a free kerbal)
    float throttle = 0.0f;
    int active_stage = 1;
    int total_stages = 1;
    int slew_request = 0;     // SlewMode (vehicle.h)
    std::vector<SaveDock> docks;
    /* The dock target is named by SHIP NAME, its port by uid: the target is a
       separate save file, so there is no shared part scope to name it in, and
       ship names are the fleet's key (dedupName keeps them unique). The port
       resolves through the load's one uid->Part map, which spans every file
       because the uids were minted in one process. */
    std::string dock_target_ship;    // "" = no target
    uint64_t dock_target_port = 0;   // part uid on the target ship (0 = none)
    uint64_t dock_arm_port = 0;      // this ship's own port part uid (0 = none)

    // crew (is_crew true)
    std::string aboard;       // the ship's name ("" = free / on EVA)
    // the capsule Part the kerbal sits in, named by uid (NOT by index into the
    // ship's part list). A uid is stable across a merge/split and, unlike an
    // index, names the exact part regardless of the list's order -- and 0 is
    // the "absent" sentinel, so a save that predates uid-keyed crew is refused
    // on load rather than silently parked in part 0. Load also validates the
    // target is a capsule (crew_capacity > 0); the index format had no such
    // check, so a reordered save could park a kerbal in any part.
    uint64_t aboard_part = 0; // the aboard ship's container part uid (0 = free / on EVA)
    /* The kerbal's suit tank contents (kg per ResourceType, one entry per
       type). Saved so a kerbal that burned some EVA propellant does not get a
       free re-seed on load (phase 4.1). Empty = the save predates this field
       and the suit is left at its init() re-seed (full). */
    std::vector<double> suit_fuel;
    /* The kerbal's suit inventory (phase 4.6): the items parked in the suit
       (Part::ownedContents), serialized the same depth-first way as a ship
       part's `inventory`. Empty = no items (or a save that predates the
       field). The suit's OWN fuel is suit_fuel; the suit part itself is
       rebuilt from its def, so only its contents are saved here. */
    std::vector<SavePart> suit_inventory;
    /* The kerbal's suit experiments (Part::experiments on the suit).
       Unlimited in v1. Empty = none (or a save that predates the field). */
    std::vector<Experiment> suit_experiments;
};

// The global save state (dir/save.json). `ships` is the ordered list of
// slugs (ships/<slug>.json), in the canonical fleet order (collectVehicles).
struct SaveMeta {
    int format = 1;
    std::string saved_at;    // wall-clock (human-readable; not parsed back)
    std::string system;      // the system file (res/systems/ksp_system.json)
    std::string parts;       // the parts catalog file (res/data/parts.json)
    double time = 0.0;       // the analytic sim clock (s)
    int time_accel = 1;      // recorded for the round-trip; load starts paused
    std::string active_ship; // display name ("" = none)
    /* Engine-performance difficulty: multiplies every engine's exhaust
       velocity (thrust + delta-v scale by it, the fuel burn does not). Chosen
       on the New Game setup window and stored here so a reload restores the
       same difficulty -- a save is not portable across scales. */
    float exhaust_scale = 1.0f;
    /* Science: total score from recovered experiments + the unique keys
       already counted (recovery scores each once; later diminishing
       returns). Both default empty/0 for a save that predates science. */
    int science_score = 0;
    std::vector<Experiment> recovered;
    std::vector<std::string> ships;
};

// ---- pure JSON (de)serialization (inline: header-only, unit-testable) ----
// Reads are permissive (j.contains(k) && type check) so a hand-edited or
// older/newer file never crashes the load -- an absent/unknown key keeps the
// struct's default.

inline std::vector<double> mat3ToVec(const glm::dmat3 &m) {
    std::vector<double> v(9);
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) { v[c * 3 + r] = m[c][r]; }
    }
    return v;
}

inline glm::dmat3 mat3FromVec(const nlohmann::json &j) {
    if(!j.is_array() || j.size() != 9) { return glm::dmat3(1.0); }
    return glm::dmat3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>(),
                      j[3].get<double>(), j[4].get<double>(), j[5].get<double>(),
                      j[6].get<double>(), j[7].get<double>(), j[8].get<double>());
}

inline glm::dvec3 vec3FromJson(const nlohmann::json &j) {
    if(!j.is_array() || j.size() != 3) { return glm::dvec3(0.0); }
    return glm::dvec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

/* Read a saved part uid. A uid is only ever a non-negative integer (the writer
   emits unsigned integers, and >2^63 round-trips exactly via number_unsigned),
   so that is the only accepted form. Everything else -- a float (truncates:
   5.9 would silently become uid 5, a VALID key, mis-resolving to the wrong
   part), a negative (wraps to a huge unsigned, never 0, so it sails past the
   "absent" sentinel and the duplicate check), or a non-number -- is returned
   as `absent` (0 by default) and refused loudly on load. This is what keeps
   the strict loader's "0 means absent" invariant from being defeated by a
   corrupt value that `is_number()` would have happily admitted. */
inline uint64_t readUid(const nlohmann::json &j, const char *key, uint64_t absent = 0) {
    if(!j.contains(key)) { return absent; }
    const nlohmann::json &v = j.at(key);
    if(v.is_number_unsigned()) { return v.get<uint64_t>(); }
    if(v.is_number_integer()) {
        int64_t i = v.get<int64_t>();
        return i >= 0 ? (uint64_t)i : absent;
    }
    return absent;
}

inline nlohmann::json experimentToJson(const Experiment &e) {
    nlohmann::json j;
    j["type"]      = e.type;
    j["body"]      = e.body;
    j["situation"] = situationId(e.situation);
    j["biome"]     = e.biome;
    return j;
}

inline Experiment experimentFromJson(const nlohmann::json &j) {
    Experiment e;
    if(j.contains("type") && j["type"].is_string()) { e.type = j["type"].get<std::string>(); }
    if(j.contains("body") && j["body"].is_string()) { e.body = j["body"].get<std::string>(); }
    if(j.contains("situation") && j["situation"].is_string()) {
        e.situation = situationFromId(j["situation"].get<std::string>());
    }
    if(j.contains("biome") && j["biome"].is_string()) { e.biome = j["biome"].get<std::string>(); }
    return e;
}

inline nlohmann::json experimentListToJson(const std::vector<Experiment> &v) {
    nlohmann::json a = nlohmann::json::array();
    for(const Experiment &e : v) { a.push_back(experimentToJson(e)); }
    return a;
}

inline std::vector<Experiment> experimentListFromJson(const nlohmann::json &j) {
    std::vector<Experiment> v;
    if(!j.is_array()) { return v; }
    for(auto &&ej : j) {
        if(ej.is_object()) { v.push_back(experimentFromJson(ej)); }
    }
    return v;
}

inline nlohmann::json savePartToJson(const SavePart &p) {
    nlohmann::json j;
    j["part"]   = p.part;
    j["uid"]    = p.uid;
    j["id"]     = p.id;
    if(p.parent != 0) { j["parent"] = p.parent; }
    j["stage"]  = p.stage;
    if(p.pos != glm::dvec3(0.0)) { j["pos"] = std::vector<double>({p.pos.x, p.pos.y, p.pos.z}); }
    if(p.rot != glm::dmat3(1.0)) { j["rot"] = mat3ToVec(p.rot); }
    j["mass"]   = p.mass;
    if(p.hull_margin >= 0.0) { j["hull_margin"] = p.hull_margin; }
    j["fuel"]   = p.fuel;
    if(!p.experiments.empty()) { j["experiments"] = experimentListToJson(p.experiments); }
    if(!p.inventory.empty()) {
        nlohmann::json inv = nlohmann::json::array();
        for(auto &&sp : p.inventory) { inv.push_back(savePartToJson(sp)); }
        j["inventory"] = inv;
    }
    return j;
}

inline SavePart savePartFromJson(const nlohmann::json &j) {
    SavePart p;
    if(j.contains("part") && j["part"].is_string()) { p.part = j["part"].get<std::string>(); }
    p.uid = readUid(j, "uid");
    if(j.contains("id") && j["id"].is_string()) { p.id = j["id"].get<std::string>(); }
    p.parent = readUid(j, "parent");
    if(j.contains("stage") && j["stage"].is_number()) { p.stage = j["stage"].get<int>(); }
    if(j.contains("pos") && j["pos"].is_array()) { p.pos = vec3FromJson(j["pos"]); }
    if(j.contains("rot") && j["rot"].is_array()) { p.rot = mat3FromVec(j["rot"]); }
    if(j.contains("mass") && j["mass"].is_number()) { p.mass = j["mass"].get<double>(); }
    if(j.contains("hull_margin") && j["hull_margin"].is_number()) { p.hull_margin = j["hull_margin"].get<double>(); }
    if(j.contains("fuel") && j["fuel"].is_array()) {
        for(auto &&f : j["fuel"]) { if(f.is_number()) { p.fuel.push_back(f.get<double>()); } }
    }
    if(j.contains("experiments")) {
        p.experiments = experimentListFromJson(j["experiments"]);
    }
    if(j.contains("inventory") && j["inventory"].is_array()) {
        for(auto &&sp : j["inventory"]) { if(sp.is_object()) { p.inventory.push_back(savePartFromJson(sp)); } }
    }
    return p;
}

inline nlohmann::json savePoseToJson(const SavePose &p) {
    nlohmann::json j;
    j["body"]     = p.body;
    j["rotating"] = p.rotating;
    j["pos"]      = std::vector<double>({p.pos.x, p.pos.y, p.pos.z});
    j["rot"]      = mat3ToVec(p.rot);
    j["vel"]      = std::vector<double>({p.vel.x, p.vel.y, p.vel.z});
    j["angvel"]   = std::vector<double>({p.angvel.x, p.angvel.y, p.angvel.z});
    return j;
}

inline SavePose savePoseFromJson(const nlohmann::json &j) {
    SavePose p;
    if(j.contains("body") && j["body"].is_string()) { p.body = j["body"].get<std::string>(); }
    if(j.contains("rotating") && j["rotating"].is_boolean()) { p.rotating = j["rotating"].get<bool>(); }
    if(j.contains("pos") && j["pos"].is_array()) { p.pos = vec3FromJson(j["pos"]); }
    if(j.contains("rot") && j["rot"].is_array()) { p.rot = mat3FromVec(j["rot"]); }
    if(j.contains("vel") && j["vel"].is_array()) { p.vel = vec3FromJson(j["vel"]); }
    if(j.contains("angvel") && j["angvel"].is_array()) { p.angvel = vec3FromJson(j["angvel"]); }
    return p;
}

inline nlohmann::json saveFlogToJson(const FlightLog &f) {
    nlohmann::json j;
    j["started"]    = f.started;
    j["start_t"]    = f.start_t;
    j["start_body"] = f.start_body;
    j["last_body"]  = f.last_body;
    nlohmann::json evs = nlohmann::json::array();
    for(const FlightEvent &e : f.events) {
        nlohmann::json ej;
        ej["t"]     = e.t;
        ej["enter"] = e.enter;
        ej["body"]  = e.body;
        evs.push_back(ej);
    }
    j["events"] = evs;
    return j;
}

inline FlightLog saveFlogFromJson(const nlohmann::json &j) {
    FlightLog f;
    if(j.contains("started") && j["started"].is_boolean()) { f.started = j["started"].get<bool>(); }
    if(j.contains("start_t") && j["start_t"].is_number()) { f.start_t = j["start_t"].get<double>(); }
    if(j.contains("start_body") && j["start_body"].is_string()) { f.start_body = j["start_body"].get<std::string>(); }
    if(j.contains("last_body") && j["last_body"].is_string()) { f.last_body = j["last_body"].get<std::string>(); }
    if(j.contains("events") && j["events"].is_array()) {
        for(auto &&ej : j["events"]) {
            if(!ej.is_object()) { continue; }
            FlightEvent e;
            if(ej.contains("t") && ej["t"].is_number()) { e.t = ej["t"].get<double>(); }
            if(ej.contains("enter") && ej["enter"].is_boolean()) { e.enter = ej["enter"].get<bool>(); }
            if(ej.contains("body") && ej["body"].is_string()) { e.body = ej["body"].get<std::string>(); }
            f.events.push_back(e);
        }
    }
    return f;
}

inline nlohmann::json saveShipToJson(const SaveShip &s) {
    nlohmann::json j;
    j["name"]     = s.name;
    j["defPath"]  = s.defPath;
    j["is_crew"]  = s.is_crew;
    // Shared by ships and crew. Skipped when the journal never started, so a
    // fresh vessel writes no flog and an old save (no flog key) loads clean.
    if(s.flog.started) { j["flog"] = saveFlogToJson(s.flog); }
    if(s.is_crew) {
        if(!s.aboard.empty()) { j["aboard"] = s.aboard; }
        j["aboard_part"] = s.aboard_part;
        // a free (EVA) kerbal's pose (an aboard one's is unused on load)
        j["pose"] = savePoseToJson(s.pose);
        j["onRails"] = s.onRails;
        if(!s.suit_fuel.empty()) { j["suit_fuel"] = s.suit_fuel; }
        if(!s.suit_inventory.empty()) {
            nlohmann::json inv = nlohmann::json::array();
            for(auto &&sp : s.suit_inventory) { inv.push_back(savePartToJson(sp)); }
            j["suit_inventory"] = inv;
        }
        if(!s.suit_experiments.empty()) {
            j["suit_experiments"] = experimentListToJson(s.suit_experiments);
        }
        return j;
    }
    j["home"]         = s.home;
    if(!s.scenario.empty()) { j["scenario"] = s.scenario; }
    j["slot"]         = s.slot;
    nlohmann::json parts = nlohmann::json::array();
    for(auto &&p : s.parts) { parts.push_back(savePartToJson(p)); }
    j["parts"] = parts;
    if(!s.fuel_links.empty()) {
        nlohmann::json links = nlohmann::json::array();
        for(auto &&l : s.fuel_links) {
            nlohmann::json lj; lj["from"] = l.from; lj["to"] = l.to;
            links.push_back(lj);
        }
        j["fuel_links"] = links;
    }
    if(s.controller != 0) { j["controller"] = s.controller; }
    j["pose"]         = savePoseToJson(s.pose);
    j["onRails"]      = s.onRails;
    j["throttle"]     = s.throttle;
    j["active_stage"] = s.active_stage;
    j["total_stages"] = s.total_stages;
    j["slew_request"] = s.slew_request;
    if(!s.docks.empty()) {
        nlohmann::json docks = nlohmann::json::array();
        for(auto &&d : s.docks) {
            nlohmann::json dj; dj["port"] = d.port; dj["root"] = d.root; dj["name"] = d.name;
            docks.push_back(dj);
        }
        j["docks"] = docks;
    }
    if(!s.dock_target_ship.empty()) { j["dock_target_ship"] = s.dock_target_ship; }
    if(s.dock_target_port != 0) { j["dock_target_port"] = s.dock_target_port; }
    if(s.dock_arm_port != 0) { j["dock_arm_port"] = s.dock_arm_port; }
    return j;
}

inline SaveShip saveShipFromJson(const nlohmann::json &j) {
    SaveShip s;
    if(j.contains("name") && j["name"].is_string()) { s.name = j["name"].get<std::string>(); }
    if(j.contains("defPath") && j["defPath"].is_string()) { s.defPath = j["defPath"].get<std::string>(); }
    if(j.contains("is_crew") && j["is_crew"].is_boolean()) { s.is_crew = j["is_crew"].get<bool>(); }
    // Shared by ships and crew; absent (an old save) leaves flog default, so
    // the load's setSoi starts a fresh journal at the placement.
    if(j.contains("flog") && j["flog"].is_object()) { s.flog = saveFlogFromJson(j["flog"]); }
    if(s.is_crew) {
        if(j.contains("aboard") && j["aboard"].is_string()) { s.aboard = j["aboard"].get<std::string>(); }
        // uid-keyed (not an index): a string/float/negative reads as 0, the
        // "absent" sentinel load refuses -- the same strict handling as every
        // other part reference (controller, dock ports, fuel links).
        s.aboard_part = readUid(j, "aboard_part");
        if(j.contains("pose") && j["pose"].is_object()) { s.pose = savePoseFromJson(j["pose"]); }
        if(j.contains("onRails") && j["onRails"].is_boolean()) { s.onRails = j["onRails"].get<bool>(); }
        if(j.contains("suit_fuel") && j["suit_fuel"].is_array()) {
            for(auto &&f : j["suit_fuel"]) { if(f.is_number()) { s.suit_fuel.push_back(f.get<double>()); } }
        }
        if(j.contains("suit_inventory") && j["suit_inventory"].is_array()) {
            for(auto &&sp : j["suit_inventory"]) { if(sp.is_object()) { s.suit_inventory.push_back(savePartFromJson(sp)); } }
        }
        if(j.contains("suit_experiments")) {
            s.suit_experiments = experimentListFromJson(j["suit_experiments"]);
        }
        return s;
    }
    if(j.contains("home") && j["home"].is_string()) { s.home = j["home"].get<std::string>(); }
    if(j.contains("scenario") && j["scenario"].is_string()) { s.scenario = j["scenario"].get<std::string>(); }
    if(j.contains("slot") && j["slot"].is_number()) { s.slot = j["slot"].get<int>(); }
    if(j.contains("parts") && j["parts"].is_array()) {
        for(auto &&p : j["parts"]) { if(p.is_object()) { s.parts.push_back(savePartFromJson(p)); } }
    }
    if(j.contains("fuel_links") && j["fuel_links"].is_array()) {
        for(auto &&l : j["fuel_links"]) {
            if(!l.is_object()) { continue; }
            SaveFuelLink lk;
            lk.from = readUid(l, "from");
            lk.to = readUid(l, "to");
            s.fuel_links.push_back(lk);
        }
    }
    s.controller = readUid(j, "controller");
    if(j.contains("pose") && j["pose"].is_object()) { s.pose = savePoseFromJson(j["pose"]); }
    if(j.contains("onRails") && j["onRails"].is_boolean()) { s.onRails = j["onRails"].get<bool>(); }
    if(j.contains("throttle") && j["throttle"].is_number()) { s.throttle = j["throttle"].get<float>(); }
    if(j.contains("active_stage") && j["active_stage"].is_number()) { s.active_stage = j["active_stage"].get<int>(); }
    if(j.contains("total_stages") && j["total_stages"].is_number()) { s.total_stages = j["total_stages"].get<int>(); }
    if(j.contains("slew_request") && j["slew_request"].is_number()) { s.slew_request = j["slew_request"].get<int>(); }
    if(j.contains("docks") && j["docks"].is_array()) {
        for(auto &&d : j["docks"]) {
            if(!d.is_object()) { continue; }
            SaveDock dk;
            dk.port = readUid(d, "port");
            dk.root = readUid(d, "root");
            if(d.contains("name") && d["name"].is_string()) { dk.name = d["name"].get<std::string>(); }
            s.docks.push_back(dk);
        }
    }
    if(j.contains("dock_target_ship") && j["dock_target_ship"].is_string()) { s.dock_target_ship = j["dock_target_ship"].get<std::string>(); }
    s.dock_target_port = readUid(j, "dock_target_port");
    s.dock_arm_port = readUid(j, "dock_arm_port");
    return s;
}

inline nlohmann::json saveMetaToJson(const SaveMeta &m) {
    nlohmann::json j;
    j["format"]      = m.format;
    j["saved_at"]    = m.saved_at;
    j["system"]      = m.system;
    j["parts"]       = m.parts;
    j["time"]        = m.time;
    j["time_accel"]  = m.time_accel;
    if(!m.active_ship.empty()) { j["active_ship"] = m.active_ship; }
    j["exhaust_scale"] = m.exhaust_scale;
    if(m.science_score != 0) { j["science_score"] = m.science_score; }
    if(!m.recovered.empty()) { j["recovered"] = experimentListToJson(m.recovered); }
    j["ships"]       = m.ships;
    return j;
}

inline SaveMeta saveMetaFromJson(const nlohmann::json &j) {
    SaveMeta m;
    if(j.contains("format") && j["format"].is_number()) { m.format = j["format"].get<int>(); }
    if(j.contains("saved_at") && j["saved_at"].is_string()) { m.saved_at = j["saved_at"].get<std::string>(); }
    if(j.contains("system") && j["system"].is_string()) { m.system = j["system"].get<std::string>(); }
    if(j.contains("parts") && j["parts"].is_string()) { m.parts = j["parts"].get<std::string>(); }
    if(j.contains("time") && j["time"].is_number()) { m.time = j["time"].get<double>(); }
    if(j.contains("time_accel") && j["time_accel"].is_number()) { m.time_accel = j["time_accel"].get<int>(); }
    if(j.contains("active_ship") && j["active_ship"].is_string()) { m.active_ship = j["active_ship"].get<std::string>(); }
    if(j.contains("exhaust_scale") && j["exhaust_scale"].is_number()) {
        // Clamp like the CLI range (0.5-5): a hand-edited 0 would zero every
        // engine's thrust, and a 1e6 would make the game unplayable.
        m.exhaust_scale = j["exhaust_scale"].get<float>();
        if(m.exhaust_scale < 0.5f) { m.exhaust_scale = 0.5f; }
        if(m.exhaust_scale > 5.0f) { m.exhaust_scale = 5.0f; }
    }
    if(j.contains("science_score") && j["science_score"].is_number()) {
        m.science_score = j["science_score"].get<int>();
    }
    if(j.contains("recovered")) {
        m.recovered = experimentListFromJson(j["recovered"]);
    }
    if(j.contains("ships") && j["ships"].is_array()) {
        for(auto &&s : j["ships"]) { if(s.is_string()) { m.ships.push_back(s.get<std::string>()); } }
    }
    return m;
}

/* The bodies the saved fleet sits on, in fleet order: each ship file's
   pose.body (where the ship IS), falling back to its home body (where it was
   built) when the pose names no body. Unique, first-wins. Empty when the save
   is missing or unreadable. The boot --load path uses this to build exactly
   those bodies' heavy phase synchronously -- the player is on them, wherever
   the save put the fleet (loading a save landed on a non-home body must not
   leave them streaming). Header-only like saveMetaFromJson (pure file reads),
   so the unit test can exercise it. */
inline std::vector<std::string> saveShipBodies(const std::string &dir) {
    std::vector<std::string> bodies;
    nlohmann::json meta;
    {
        std::ifstream f(dir + "/save.json");
        if(!f) { return bodies; }
        try { meta = nlohmann::json::parse(f, nullptr, true); }
        catch(const std::exception &) { return bodies; }
    }
    if(!meta.contains("ships") || !meta["ships"].is_array()) { return bodies; }
    for(auto &&slug : meta["ships"]) {
        if(!slug.is_string()) { continue; }
        nlohmann::json s;
        {
            std::ifstream f(dir + "/ships/" + slug.get<std::string>() + ".json");
            if(!f) { continue; }
            try { s = nlohmann::json::parse(f, nullptr, true); }
            catch(const std::exception &) { continue; }
        }
        std::string body;
        if(s.contains("pose") && s["pose"].is_object() &&
           s["pose"].contains("body") && s["pose"]["body"].is_string()) {
            body = s["pose"]["body"].get<std::string>();
        }
        if(body.empty() && s.contains("home") && s["home"].is_string()) {
            body = s["home"].get<std::string>();
        }
        if(!body.empty() &&
           std::find(bodies.begin(), bodies.end(), body) == bodies.end()) {
            bodies.push_back(body);
        }
    }
    return bodies;
}

// ---- the save-directory helpers (inline: pure file-system, no game state) --
// All std::filesystem, so portable: the create/list/delete ops that used to
// be POSIX (dirent + `rm -rf` via a shell) are now cross-platform calls, and
// deleting no longer shells out at all.

// Create dir (and any missing parents) if it does not exist. No-op if it does.
// Non-throwing: a creation failure (e.g. an unwritable data dir) surfaces at
// the subsequent file write, which names the actual file -- matching the old
// mkdir(2) behavior of ignoring the mkdir error.
inline void ensure_dir(const std::string &dir) {
    if(dir.empty()) { return; }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
}

// List the save names under base (e.g. a game dir), sorted; empty if base is
// missing or empty. A name is a subdirectory that holds a save.json.
inline std::vector<std::string> list_saves(const std::string &base) {
    std::vector<std::string> names;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(base, ec);
    if(ec) { return names; }   // base missing or not a directory
    for(const auto &entry : it) {
        const std::string name = entry.path().filename().string();
        if(name == "." || name == "..") { continue; }
        if(fs::is_directory(entry) && fs::exists(entry.path() / "save.json")) {
            names.push_back(name);
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

// ---- games: the <stamp>-<name> dir a game's saves live under -------------
// A "game" is one playthrough (one New Game start). Its slots live under
// saves/<stamp>-<name>/: the dir name carries the START stamp (YYYYMMDD_HHMMSS,
// local time -- it is the dir the player browses, so local, not the UTC
// saved_at) and the user-chosen display name. Stamp-first, so `ls` sorts
// the games chronologically (the UI list uses the same order). Same-named
// games coexist: each start mints a fresh stamp, bumping the second while the
// dir already exists (see newGameDir).

// The local-time stamp newGameDir prepends: YYYYMMDD_HHMMSS.
inline std::string gameStamp(time_t t) {
    struct tm m;
    localtime_r(&t, &m);
    char buf[32];
    strftime(buf, sizeof buf, "%Y%m%d_%H%M%S", &m);
    return buf;
}

// A fresh game dir under base for a game named `name`: <stamp>-<name>. The
// stamp starts at `now` and bumps one second at a time while the dir exists
// (two same-named games started the same second must not share a dir --
// their slots would mix). "" if 60 bumps do not clear it.
inline std::string newGameDir(const std::string &base, const std::string &name,
                              time_t now = time(nullptr)) {
    namespace fs = std::filesystem;
    for(int bump = 0; bump < 60; bump++) {
        const std::string dir = base + "/" + gameStamp(now + bump) + "-" + name;
        std::error_code ec;
        if(!fs::exists(dir, ec)) { return dir; }
    }
    return "";
}

// The display name of a game dir name: strip exactly ONE leading
// <YYYYMMDD_HHMMSS>- (newGameDir prepends exactly one, so a name that itself
// starts with stamp-like digits still round-trips: a game NAMED
// "20260101_000000-restart" lists as itself). A dir without a leading stamp
// -- renamed by hand -- keeps its whole name.
inline std::string gameDirName(const std::string &dirName) {
    static const size_t kStamp = 15;   // 8 digits + '_' + 6 digits
    // Need the stamp (0..14), a dash (15), and a non-empty name (16+).
    if(dirName.size() <= kStamp + 1 || dirName[kStamp] != '-') {
        return dirName;
    }
    for(size_t i = 0; i < kStamp; i++) {
        // Explicit '0'..'9', not std::isdigit (locale-dependent: under a
        // non-C locale non-ASCII digits would count and a name could be
        // mis-stripped).
        const char c = dirName[i];
        if((i == 8) ? (c != '_') : (c < '0' || c > '9')) {
            return dirName;
        }
    }
    return dirName.substr(kStamp + 1);
}

// One listed game: `dirName` the dir under base (the identity the slots live
// under), `name` the display label (the stamp stripped; a duplicate label --
// two same-named games -- gets its stamp appended so the rows stay
// distinguishable).
struct GameEntry {
    std::string dirName;
    std::string name;
};

// List the games under base (e.g. the data dir's saves/): the subdirectories
// holding at least one slot (list_saves non-empty), sorted by dirName --
// chronological, like `ls`. A manifest-less slot dir (a legacy flat save
// directly under base) is NOT a game, and neither is a game that never saved;
// both stay loadable by full path.
inline std::vector<GameEntry> list_games(const std::string &base) {
    std::vector<GameEntry> games;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(base, ec);
    if(ec) { return games; }   // base missing or not a directory
    for(const auto &entry : it) {
        if(!fs::is_directory(entry)) { continue; }
        const std::string dirName = entry.path().filename().string();
        if(list_saves(entry.path().string()).empty()) { continue; }
        games.push_back({dirName, gameDirName(dirName)});
    }
    std::sort(games.begin(), games.end(),
              [](const GameEntry &a, const GameEntry &b) {
                  return a.dirName < b.dirName;
              });
    // Duplicate labels (two same-named games) get their stamp appended
    // (YYYYMMDD_HHMMSS -> "YYYY-MM-DD HH:MM"; a stamped-out dir falls back
    // to its whole dir name). Count from dirName (immutable), not name:
    // appending as we go would un-duplicate the entries already labelled.
    for(size_t i = 0; i < games.size(); i++) {
        const std::string orig = gameDirName(games[i].dirName);
        int n = 0;
        for(const GameEntry &e : games) {
            if(gameDirName(e.dirName) == orig) { n++; }
        }
        if(n <= 1) { continue; }
        const std::string &dn = games[i].dirName;
        std::string stamp = dn;
        if(dn.size() > 16 && gameDirName(dn).size() == dn.size() - 16) {
            stamp = dn.substr(0, 4) + "-" + dn.substr(4, 2) + "-" +
                    dn.substr(6, 2) + " " + dn.substr(9, 2) + ":" +
                    dn.substr(11, 2);
        }
        games[i].name += " (" + stamp + ")";
    }
    return games;
}

// Resolve a bare slot name under base (the data dir's saves/): base/<slot>
// (a legacy flat save) first, then the UNIQUE base/<game>/<slot> (the
// two-tier layout). "" when missing or ambiguous (two games hold the slot).
inline std::string find_slot(const std::string &base, const std::string &slot) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if(fs::exists(base + "/" + slot + "/save.json", ec)) {
        return base + "/" + slot;
    }
    std::string found;
    int n = 0;
    auto it = fs::directory_iterator(base, ec);
    if(ec) { return ""; }
    for(const auto &entry : it) {
        if(!fs::is_directory(entry)) { continue; }
        ec.clear();
        if(fs::exists(entry.path().string() + "/" + slot + "/save.json", ec)) {
            found = entry.path().string();
            n++;
        }
    }
    return n == 1 ? found + "/" + slot : "";
}

// ---- quicksaves: a rotating pool of 100 slots in a game dir -------------
// quicksave-00 .. quicksave-99. F5 (quicksave) writes quicksave-<max NN + 1>
// (quicksave-00 for an empty game dir); once the pool is full -- or max NN is
// 99 -- it OVERWRITES the oldest slot by mtime. F9 (quickload) loads the
// NEWEST by mtime. Newest/oldest are mtime, not NN: after a wrap (or a hand
// deletion) the number order no longer matches the age order, and mtime is
// the directory's own truth -- no counter file to desync or lose.
inline int quicksaveNN(const std::string &slot) {
    static const std::string kPfx = "quicksave-";
    if(slot.size() != kPfx.size() + 2) { return -1; }
    if(slot.compare(0, kPfx.size(), kPfx) != 0) { return -1; }
    const char a = slot[kPfx.size()], b = slot[kPfx.size() + 1];
    if(a < '0' || a > '9' || b < '0' || b > '9') { return -1; }
    return (a - '0') * 10 + (b - '0');
}
inline std::string quicksaveName(int nn) {
    if(nn < 0 || nn > 99) { return ""; }   // the pool is 00..99; "" = no slot
    char buf[16];
    std::snprintf(buf, sizeof buf, "quicksave-%02d", nn);
    return buf;
}

// The pool's live slots in dir: name + the last-write time of its save.json
// (a dir must hold a save.json to count -- same rule as list_saves).
//
// The FILE's mtime, not the dir's: a quicksave OVERWRITES the slot in place
// (save_game rewrites save.json and ships/), which advances the file's mtime
// but not the slot dir's -- a dir's mtime only moves when entries are
// added/removed, so dir age would freeze a reused slot at its CREATION time
// and the pool would keep re-overwriting the same oldest-created slot (and
// F9 would load the newest-created, not the newest-written, save).
//
// Non-throwing throughout: a stat failure (a slot deleted mid-scan, perms)
// skips the entry instead of escaping quicksave()/quickload() into the
// uncaught main loop.
inline std::vector<std::pair<std::string, std::filesystem::file_time_type>>
quicksaveSlots(const std::string &dir) {
    std::vector<std::pair<std::string, std::filesystem::file_time_type>> out;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(dir, ec);
    if(ec) { return out; }   // dir missing or not a directory
    for(const auto &entry : it) {
        if(!fs::is_directory(entry)) { continue; }
        const std::string name = entry.path().filename().string();
        if(quicksaveNN(name) < 0) { continue; }
        const fs::path meta = entry.path() / "save.json";
        if(!fs::exists(meta, ec)) { continue; }
        ec.clear();
        const fs::file_time_type t = fs::last_write_time(meta, ec);
        if(ec) { continue; }   // stat failed: skip rather than throw
        out.push_back({name, t});
    }
    return out;
}

// Pool ordering: by save.json mtime, an mtime TIE (a 1s-resolution
// filesystem, a cp/rsync'd saves tree) broken on the slot number --
// deterministic either way, unlike directory_iterator order.
inline bool quicksaveOlder(
        const std::pair<std::string, std::filesystem::file_time_type> &a,
        const std::pair<std::string, std::filesystem::file_time_type> &b) {
    if(a.second != b.second) { return a.second < b.second; }
    return quicksaveNN(a.first) < quicksaveNN(b.first);
}

// The slot the next quicksave writes: quicksave-<max NN + 1>, or quicksave-00
// for an empty/missing dir; when the pool is full -- or max NN is 99 (a
// deletion left a hole past the top) -- the oldest live slot by mtime.
inline std::string nextQuicksave(const std::string &dir) {
    const auto slots = quicksaveSlots(dir);
    int maxNN = -1;
    for(const auto &s : slots) {
        const int nn = quicksaveNN(s.first);
        if(nn > maxNN) { maxNN = nn; }
    }
    std::string pick;
    if((int)slots.size() < 100 && maxNN + 1 < 100) {
        pick = quicksaveName(maxNN + 1);
    } else if(!slots.empty()) {
        const auto oldest =
            std::min_element(slots.begin(), slots.end(), quicksaveOlder);
        pick = quicksaveName(quicksaveNN(oldest->first));
    }
    return pick.empty() ? quicksaveName(0) : pick;
}

// The newest quicksave in dir by mtime ("" when it has none).
inline std::string latestQuicksave(const std::string &dir) {
    const auto slots = quicksaveSlots(dir);
    if(slots.empty()) { return ""; }
    const auto newest =
        std::max_element(slots.begin(), slots.end(), quicksaveOlder);
    return newest->first;
}

// Delete the save at dir (recursive, no shell). Refuses a path not strictly
// under the given base (a guard against a typo'd delete wiping something
// else): the lexical relative path must be non-empty, not "." (dir == base),
// have no root (a different drive / absolute escape), and contain no ".."
// component at any depth (a ".." anywhere would climb out of the base).
inline void delete_save(const std::string &dir, const std::string &base) {
    namespace fs = std::filesystem;
    const fs::path rel = fs::path(dir).lexically_relative(fs::path(base));
    const bool escapes = rel.empty() || rel == fs::path(".") ||
                         rel.has_root_path() ||
                         std::any_of(rel.begin(), rel.end(),
                                     [](const fs::path &e) { return e == ".."; });
    if(escapes) {
        throw std::runtime_error("delete_save: refusing to delete '" + dir
                                 + "' (not under '" + base + "/')");
    }
    std::error_code ec;
    fs::remove_all(dir, ec);
    if(ec) {
        throw std::runtime_error("delete_save: failed to delete '" + dir
                                 + "': " + ec.message());
    }
}

// Capture the live game state (the fleet + crew + clock) into dir.
// dir is created if missing. Throws std::runtime_error naming the file on a
// write failure or an unresolvable ship (e.g. a part whose catalog name is
// no longer in the parts file).
void save_game(Game &g, const std::string &dir);

// Replace the live game state with the one in dir: delete the current fleet
// (ships + crew), rebuild it from the files, and restore the active ship +
// the clock. Used both at startup (CLI --load, where the fleet was never
// built) and at runtime (the in-game Load menu, where it replaces the fleet).
void load_game(Game &g, const std::string &dir);
