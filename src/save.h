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
// delete_save -- plain POSIX file-system ops, no game state): both are
// unit-testable headless (tests/test_save.cpp) with no Game / Bullet link.
// The Game-coupled capture/restore (save_game / load_game) lives in save.cpp.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

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
// it -- so they are the no-op defaults for part 0). `mass` is the part's
// authoritative mass (kg; the capsule's includes any aboard crew), `fuel` the
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
};

// The global save state (dir/save.json). `ships` is the ordered list of
// slugs (ships/<slug>.json), in the canonical fleet order (collectVehicles).
struct SaveMeta {
    int format = 1;
    std::string saved_at;    // wall-clock (human-readable; not parsed back)
    std::string system;      // the system file (res/ksp_system.json)
    std::string parts;       // the parts catalog file (res/parts.json)
    double time = 0.0;       // the analytic sim clock (s)
    int time_accel = 1;
    std::string active_ship; // display name ("" = none)
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

inline nlohmann::json saveShipToJson(const SaveShip &s) {
    nlohmann::json j;
    j["name"]     = s.name;
    j["defPath"]  = s.defPath;
    j["is_crew"]  = s.is_crew;
    if(s.is_crew) {
        if(!s.aboard.empty()) { j["aboard"] = s.aboard; }
        j["aboard_part"] = s.aboard_part;
        // a free (EVA) kerbal's pose (an aboard one's is unused on load)
        j["pose"] = savePoseToJson(s.pose);
        j["onRails"] = s.onRails;
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
    if(s.is_crew) {
        if(j.contains("aboard") && j["aboard"].is_string()) { s.aboard = j["aboard"].get<std::string>(); }
        // uid-keyed (not an index): a string/float/negative reads as 0, the
        // "absent" sentinel load refuses -- the same strict handling as every
        // other part reference (controller, dock ports, fuel links).
        s.aboard_part = readUid(j, "aboard_part");
        if(j.contains("pose") && j["pose"].is_object()) { s.pose = savePoseFromJson(j["pose"]); }
        if(j.contains("onRails") && j["onRails"].is_boolean()) { s.onRails = j["onRails"].get<bool>(); }
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
    if(j.contains("ships") && j["ships"].is_array()) {
        for(auto &&s : j["ships"]) { if(s.is_string()) { m.ships.push_back(s.get<std::string>()); } }
    }
    return m;
}

// ---- the save-directory helpers (inline: pure file-system, no game state) --

// Create dir (and any missing parents) if it does not exist. No-op if it does.
inline void ensure_dir(const std::string &dir) {
    if(dir.empty()) { return; }
    std::string cur;
    for(size_t i = 0; i < dir.size(); i++) {
        cur += dir[i];
        if(dir[i] == '/' && cur.size() > 1) { mkdir(cur.c_str(), 0755); }
    }
    mkdir(dir.c_str(), 0755);
}

// List the save names under base (e.g. "saves"), sorted; empty if base is
// missing or empty. A name is a subdirectory that holds a save.json.
inline std::vector<std::string> list_saves(const std::string &base) {
    std::vector<std::string> names;
    DIR *d = opendir(base.c_str());
    if(d == nullptr) { return names; }
    struct dirent *e;
    while((e = readdir(d)) != nullptr) {
        std::string name = e->d_name;
        if(name == "." || name == "..") { continue; }
        if(access((base + "/" + name + "/save.json").c_str(), F_OK) == 0) {
            names.push_back(name);
        }
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    return names;
}

// Delete the save at dir (rm -rf). Refuses a path not under the given base
// (a guard against a typo'd delete wiping something else). The prefix check
// is string-based, so it would still pass "base/../evil" (rm -rf follows the
// ".." and escapes the base) -- a path component of ".." is rejected too.
inline void delete_save(const std::string &dir, const std::string &base) {
    std::string prefix = base;
    if(!prefix.empty() && prefix.back() != '/') { prefix += "/"; }
    if(dir.size() <= prefix.size() || dir.compare(0, prefix.size(), prefix) != 0) {
        throw std::runtime_error("delete_save: refusing to delete '" + dir
                                 + "' (not under '" + base + "/')");
    }
    size_t i = prefix.size();
    while(i < dir.size()) {
        size_t slash = dir.find('/', i);
        std::string comp = dir.substr(i, (slash == std::string::npos)
                                          ? std::string::npos : slash - i);
        if(comp == "..") {
            throw std::runtime_error("delete_save: refusing to delete '" + dir
                                     + "' ('..' would escape '" + base + "/')");
        }
        if(slash == std::string::npos) { break; }
        i = slash + 1;
    }
    std::string cmd = "rm -rf '" + dir + "'";
    if(system(cmd.c_str()) != 0) {
        throw std::runtime_error("delete_save: failed to delete '" + dir + "'");
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
