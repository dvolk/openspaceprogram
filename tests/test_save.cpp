// test_save: the pure save/load JSON (de)serialization (src/save.h).
// Runs from the repo root (needs no res/ -- it round-trips in-memory data):
//   make test   (or: ./test_save)
//
// Covers: SaveMeta + SaveShip <-> nlohmann round-trip (every field), the
// permissive reads (an absent/unknown key keeps the struct's default, an
// older/newer file never crashes), and the mat3/vec3 serialization.
//
// The Game-coupled capture/restore (save_game / load_game in save.cpp) needs
// Game / Ships / Bullet, so it is NOT covered here (the e2e save-load case
// exercises it headless through the game).

#include "save.h"

#include <cmath>
#include <cstdio>
#include <string>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }
static bool vnear(const glm::dvec3 &a, const glm::dvec3 &b) {
    return glm::length(a - b) < 1e-9;
}
static bool mnear(const glm::dmat3 &a, const glm::dmat3 &b) {
    const glm::dmat3 d = a - b;
    double s = 0;
    for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) s += d[i][j] * d[i][j];
    return std::sqrt(s) < 1e-9;
}

int main() {
    // --- a non-trivial ship (2 parts, a fuel link, a dock seam, a target) --
    SaveShip ship;
    ship.name = "racer";
    ship.defPath = "res/ships/racer.json";
    ship.is_crew = false;
    ship.home = "Kerbin";
    ship.scenario = "rot-orbit";
    ship.slot = 2;

    // part 0: the root (identity pose -- the save omits pos/rot for it)
    SavePart root;
    root.part = "capsule";
    root.id = "capsule_1";
    root.parent = "";
    root.stage = 1;
    root.pos = glm::dvec3(0.0);
    root.rot = glm::dmat3(1.0);
    root.mass = 1246.87;
    root.hull_margin = -1.0;   // unset (omitted in the JSON)
    for(int r = 0; r < 3; r++) {
        root.fuel.push_back(r == 2 ? 1999.8 : 0.0);   // EC = index 2
    }
    ship.parts.push_back(root);

    // part 1: a child with a non-identity pose + fuel
    SavePart tank;
    tank.part = "fuel_tank";
    tank.id = "fuel_tank_1";
    tank.parent = "capsule_1";
    tank.stage = 2;
    tank.pos = glm::dvec3(0.0, 0.0, -2.25);
    tank.rot = glm::dmat3(1.0);
    tank.mass = 896.7;
    tank.hull_margin = 0.1;
    for(int r = 0; r < 3; r++) {
        tank.fuel.push_back((r == 0 || r == 1) ? 406.58 : 0.0);   // Hydrogen=0, LOX=1
    }
    ship.parts.push_back(tank);

    ship.fuel_links.push_back(SaveFuelLink{ "fuel_tank_1", "capsule_1" });
    ship.controller = "capsule_1";

    // a non-trivial pose in a rotating frame
    ship.pose.body = "Kerbin";
    ship.pose.rotating = true;
    ship.pose.pos = glm::dvec3(11030.05, -354.31, 684912.32);
    ship.pose.rot = glm::dmat3(
        0.0002, 0.9999, -2.6e-9,
        3.2e-5, -4e-9, 0.9999,
        0.9999, -0.0002, -3.2e-5);
    ship.pose.vel = glm::dvec3(2489.5, -79.4, -39.3);
    ship.pose.angvel = glm::dvec3(1.5e-9, 2.16e-5, -9.28e-5);

    ship.onRails = false;
    ship.throttle = 0.5f;
    ship.active_stage = 2;
    ship.total_stages = 3;
    ship.slew_request = 1;   // SlewMode SlewPrograde (vehicle.h); the int is what saves
    ship.docks.push_back(SaveDock{ "capsule_1", "capsule_1", "station" });
    ship.dock_target_ship = "station";
    ship.dock_target_port = "port_1";
    ship.dock_arm_port = "capsule_1";

    nlohmann::json j = saveShipToJson(ship);
    SaveShip out = saveShipFromJson(j);
    CHECK(out.name == ship.name);
    CHECK(out.defPath == ship.defPath);
    CHECK(out.is_crew == ship.is_crew);
    CHECK(out.home == ship.home);
    CHECK(out.scenario == ship.scenario);
    CHECK(out.slot == ship.slot);
    CHECK(out.parts.size() == ship.parts.size());
    for(size_t i = 0; i < out.parts.size() && i < ship.parts.size(); i++) {
        const SavePart &a = out.parts[i];
        const SavePart &b = ship.parts[i];
        CHECK(a.part == b.part);
        CHECK(a.id == b.id);
        CHECK(a.parent == b.parent);
        CHECK(a.stage == b.stage);
        CHECK(vnear(a.pos, b.pos));
        CHECK(mnear(a.rot, b.rot));
        CHECK(near(a.mass, b.mass));
        CHECK(near(a.hull_margin, b.hull_margin));
        CHECK(a.fuel.size() == b.fuel.size());
        for(size_t r = 0; r < a.fuel.size() && r < b.fuel.size(); r++) {
            CHECK(near(a.fuel[r], b.fuel[r]));
        }
    }
    CHECK(out.fuel_links.size() == ship.fuel_links.size());
    if(out.fuel_links.size() == 1 && ship.fuel_links.size() == 1) {
        CHECK(out.fuel_links[0].from == ship.fuel_links[0].from);
        CHECK(out.fuel_links[0].to == ship.fuel_links[0].to);
    }
    CHECK(out.controller == ship.controller);
    CHECK(out.pose.body == ship.pose.body);
    CHECK(out.pose.rotating == ship.pose.rotating);
    CHECK(vnear(out.pose.pos, ship.pose.pos));
    CHECK(mnear(out.pose.rot, ship.pose.rot));
    CHECK(vnear(out.pose.vel, ship.pose.vel));
    CHECK(vnear(out.pose.angvel, ship.pose.angvel));
    CHECK(out.onRails == ship.onRails);
    CHECK(near(out.throttle, ship.throttle));
    CHECK(out.active_stage == ship.active_stage);
    CHECK(out.total_stages == ship.total_stages);
    CHECK(out.slew_request == ship.slew_request);
    CHECK(out.docks.size() == ship.docks.size());
    if(out.docks.size() == 1 && ship.docks.size() == 1) {
        CHECK(out.docks[0].port == ship.docks[0].port);
        CHECK(out.docks[0].root == ship.docks[0].root);
        CHECK(out.docks[0].name == ship.docks[0].name);
    }
    CHECK(out.dock_target_ship == ship.dock_target_ship);
    CHECK(out.dock_target_port == ship.dock_target_port);
    CHECK(out.dock_arm_port == ship.dock_arm_port);

    // --- a crew member (the ship fields are empty) -------------------------
    SaveShip crew;
    crew.name = "kerbal";
    crew.defPath = "./res/ships/kerbal.json";
    crew.is_crew = true;
    crew.aboard = "racer";
    crew.aboard_part = 0;
    SaveShip crewOut = saveShipFromJson(saveShipToJson(crew));
    CHECK(crewOut.is_crew == true);
    CHECK(crewOut.name == "kerbal");
    CHECK(crewOut.aboard == "racer");
    CHECK(crewOut.aboard_part == 0);
    CHECK(crewOut.parts.empty());   // the ship fields are not written for a crew

    // a free (EVA) kerbal carries its pose (the load restores it)
    SaveShip eva;
    eva.name = "kerbal";
    eva.defPath = "./res/ships/kerbal.json";
    eva.is_crew = true;
    eva.aboard = "";
    eva.pose.body = "Duna";
    eva.pose.rotating = true;
    eva.pose.pos = glm::dvec3(2500.0, -32000.0, 1500.0);
    eva.pose.rot = glm::dmat3(
        0.999, 0.0, 0.043,
        0.0, 1.0, 0.0,
        -0.043, 0.0, 0.999);
    eva.pose.vel = glm::dvec3(1.5, -2.25, 0.75);
    eva.pose.angvel = glm::dvec3(0.01, -0.02, 0.03);
    eva.onRails = true;   // coasting on the rails (the load re-parks it)
    SaveShip evaOut = saveShipFromJson(saveShipToJson(eva));
    CHECK(evaOut.is_crew == true);
    CHECK(evaOut.aboard.empty());
    CHECK(evaOut.pose.body == "Duna");
    CHECK(evaOut.pose.rotating == true);
    CHECK(vnear(evaOut.pose.pos, eva.pose.pos));
    CHECK(mnear(evaOut.pose.rot, eva.pose.rot));
    CHECK(vnear(evaOut.pose.vel, eva.pose.vel));
    CHECK(vnear(evaOut.pose.angvel, eva.pose.angvel));
    CHECK(evaOut.onRails == true);

    // --- the meta ----------------------------------------------------------
    SaveMeta meta;
    meta.format = 1;
    meta.saved_at = "2026-09-16T06:21:44";
    meta.system = "res/ksp_system.json";
    meta.parts = "res/parts.json";
    meta.time = 4.459999999999993;
    meta.time_accel = 1;
    meta.active_ship = "racer";
    meta.ships.push_back("v0");
    meta.ships.push_back("v1");
    SaveMeta metaOut = saveMetaFromJson(saveMetaToJson(meta));
    CHECK(metaOut.format == meta.format);
    CHECK(metaOut.saved_at == meta.saved_at);
    CHECK(metaOut.system == meta.system);
    CHECK(metaOut.parts == meta.parts);
    CHECK(near(metaOut.time, meta.time));
    CHECK(metaOut.time_accel == meta.time_accel);
    CHECK(metaOut.active_ship == meta.active_ship);
    CHECK(metaOut.ships.size() == 2);
    CHECK(metaOut.ships[0] == "v0");
    CHECK(metaOut.ships[1] == "v1");

    // --- permissive reads: an empty / partial document never crashes -------
    SaveMeta emptyMeta = saveMetaFromJson(nlohmann::json::object());
    CHECK(emptyMeta.format == 1);            // the defaults survive
    CHECK(emptyMeta.ships.empty());
    CHECK(emptyMeta.active_ship.empty());

    SaveShip emptyShip = saveShipFromJson(nlohmann::json::object());
    CHECK(emptyShip.name.empty());
    CHECK(emptyShip.is_crew == false);
    CHECK(emptyShip.parts.empty());
    CHECK(emptyShip.active_stage == 1);      // the default stage bookkeeping

    // a ship that mentions a key with the wrong type keeps the default
    nlohmann::json bad = nlohmann::json::object();
    bad["slot"] = "two";   // a string where a number is expected
    bad["throttle"] = true;  // a bool where a number is expected
    SaveShip badShip = saveShipFromJson(bad);
    CHECK(badShip.slot == 0);
    CHECK(badShip.throttle == 0.0f);

    // a part that omits the optional keys keeps the struct defaults
    nlohmann::json part = nlohmann::json::object();
    part["part"] = "engine";
    part["id"] = "engine_1";
    SavePart partOut = savePartFromJson(part);
    CHECK(partOut.part == "engine");
    CHECK(partOut.id == "engine_1");
    CHECK(partOut.parent.empty());
    CHECK(partOut.stage == 1);
    CHECK(vnear(partOut.pos, glm::dvec3(0.0)));
    CHECK(mnear(partOut.rot, glm::dmat3(1.0)));
    CHECK(near(partOut.hull_margin, -1.0));

    // the mat3/vec3 helpers round-trip
    nlohmann::json mvec = mat3ToVec(ship.pose.rot);
    CHECK(mvec.size() == 9);
    CHECK(mnear(mat3FromVec(mvec), ship.pose.rot));

    // --- the save-directory helpers (pure file-system, no game) -----------
    // Work in a scratch dir under tmp/ so we never touch a real saves/.
    const std::string base = "tmp/test_save_dir";
    system(("rm -rf '" + base + "'").c_str());

    // ensure_dir: creates the dir and any missing parents (no-op if exists).
    ensure_dir(base + "/a/b/c");
    CHECK(access((base + "/a/b/c").c_str(), F_OK) == 0);
    ensure_dir(base + "/a/b/c");   // again: no throw, still there
    CHECK(access((base + "/a/b/c").c_str(), F_OK) == 0);

    // list_saves: a subdir WITH a save.json is listed (sorted); one without
    // is skipped, and so is a plain file at the top level.
    system(("mkdir -p '" + base + "/bravo'").c_str());   // no save.json -> skip
    system(("touch '" + base + "/note.txt'").c_str());   // a file, not a dir
    system(("mkdir -p '" + base + "/alpha' && echo '{}' > '" + base + "/alpha/save.json'").c_str());
    system(("mkdir -p '" + base + "/charlie' && echo '{}' > '" + base + "/charlie/save.json'").c_str());
    std::vector<std::string> saves = list_saves(base);
    CHECK(saves.size() == 2);
    if(saves.size() == 2) {
        CHECK(saves[0] == "alpha");    // sorted: alpha < charlie
        CHECK(saves[1] == "charlie");
    }

    // delete_save: refuses a path not under base, and a ".." component that
    // would escape base; deletes a valid in-base path and leaves the rest.
    bool refused = false;
    try { delete_save("tmp/otherplace", base); }
    catch(const std::exception &) { refused = true; }
    CHECK(refused);

    bool refusedDot = false;
    try { delete_save(base + "/../evil", base); }
    catch(const std::exception &) { refusedDot = true; }
    CHECK(refusedDot);
    CHECK(access("tmp/evil", F_OK) != 0);   // the ".." escape never happened

    delete_save(base + "/alpha", base);
    CHECK(access((base + "/alpha").c_str(), F_OK) != 0);   // alpha is gone
    CHECK(access((base + "/charlie/save.json").c_str(), F_OK) == 0);  // charlie stays

    system(("rm -rf '" + base + "'").c_str());   // clean up the scratch dir

    if(failures) {
        printf("test_save: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("test_save: OK\n");
    return 0;
}
