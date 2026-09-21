// test_save: the pure save/load JSON (de)serialization (src/save.h).
// Runs from the repo root (needs no res/ -- it round-trips in-memory data):
//   make test   (or: ./test_save)
//
// Covers: SaveMeta + SaveShip <-> nlohmann round-trip (every field), the
// permissive reads (an absent/unknown key keeps the struct's default, an
// older/newer file never crashes), and the mat3/vec3 serialization.
//
// Also the uid keying of the cross-part references (see save.h): two parts
// sharing an `id` -- the shape a docked pair of same-def ships saves as --
// still round-trip as two distinguishable parts, and a string where a uid is
// expected reads back as 0, the "absent" value load refuses outright rather
// than resolving by guesswork.
//
// The Game-coupled capture/restore (save_game / load_game in save.cpp) needs
// Game / Ships / Bullet, so it is NOT covered here (the e2e save-load case
// exercises it headless through the game).

#include "save.h"
#include "shipdef.h"   // ResourceType (phase 4.6 inventory fuel index)

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
    root.uid = 101;          // the key every cross-part reference uses
    root.id = "capsule_1";
    root.parent = 0;         // 0 = the root
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
    tank.uid = 102;
    tank.id = "fuel_tank_1";
    tank.parent = 101;       // the capsule, by uid
    tank.stage = 2;
    tank.pos = glm::dvec3(0.0, 0.0, -2.25);
    tank.rot = glm::dmat3(1.0);
    tank.mass = 896.7;
    tank.hull_margin = 0.1;
    for(int r = 0; r < 3; r++) {
        tank.fuel.push_back((r == 0 || r == 1) ? 406.58 : 0.0);   // Hydrogen=0, LOX=1
    }
    ship.parts.push_back(tank);

    ship.fuel_links.push_back(SaveFuelLink{ 102, 101 });
    ship.controller = 101;

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
    ship.docks.push_back(SaveDock{ 101, 101, "station" });
    ship.dock_target_ship = "station";
    ship.dock_target_port = 103;   // a part uid on the TARGET ship
    ship.dock_arm_port = 101;

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
        CHECK(a.uid == b.uid);
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
    // aboard_part is the CAPSULE'S uid (not an index into the ship's part
    // list): 101 is the capsule's uid in the ship above, so this is a valid
    // "aboard the capsule" reference. 0 would be the "absent / free" sentinel.
    SaveShip crew;
    crew.name = "kerbal";
    crew.defPath = "./res/ships/kerbal.json";
    crew.is_crew = true;
    crew.aboard = "racer";
    crew.aboard_part = 101;
    SaveShip crewOut = saveShipFromJson(saveShipToJson(crew));
    CHECK(crewOut.is_crew == true);
    CHECK(crewOut.name == "kerbal");
    CHECK(crewOut.aboard == "racer");
    CHECK(crewOut.aboard_part == 101);
    CHECK(crewOut.parts.empty());   // the ship fields are not written for a crew

    // phase 4.1: the suit tank contents round-trip (a kerbal that burned some
    // EVA propellant does not get a free re-seed on load)
    SaveShip crewFuel;
    crewFuel.name = "kerbal";
    crewFuel.defPath = "./res/ships/kerbal.json";
    crewFuel.is_crew = true;
    crewFuel.aboard = "racer";
    crewFuel.aboard_part = 101;
    // 8.5 kg of hydrazine (was 10.0 full) -- the rest are 0
    crewFuel.suit_fuel.push_back(0.0);   // Hydrogen
    crewFuel.suit_fuel.push_back(0.0);   // LOX
    crewFuel.suit_fuel.push_back(0.0);   // EC
    crewFuel.suit_fuel.push_back(0.0);   // Oxygen
    crewFuel.suit_fuel.push_back(0.0);   // Water
    crewFuel.suit_fuel.push_back(0.0);   // Food
    crewFuel.suit_fuel.push_back(8.5);   // Hydrazine
    crewFuel.suit_fuel.push_back(0.0);   // JetFuel
    SaveShip crewFuelOut = saveShipFromJson(saveShipToJson(crewFuel));
    CHECK(crewFuelOut.suit_fuel.size() == 8);
    CHECK(near(crewFuelOut.suit_fuel[6], 8.5));   // Hydrazine
    CHECK(crewFuelOut.suit_fuel[0] == 0.0);       // Hydrogen
    // an absent suit_fuel (a save that predates the field) stays empty
    SaveShip crewOld;
    crewOld.name = "kerbal";
    crewOld.defPath = "./res/ships/kerbal.json";
    crewOld.is_crew = true;
    crewOld.aboard = "racer";
    crewOld.aboard_part = 101;
    SaveShip crewOldOut = saveShipFromJson(saveShipToJson(crewOld));
    CHECK(crewOldOut.suit_fuel.empty());

    // phase 4.6: nested inventory round-trip (a container holding items,
    // one of which holds a third -- depth-first serialization)
    {
        SavePart container;
        container.part = "cargo";
        container.uid = 201;
        container.id = "cargo_1";
        container.mass = 250.0;
        container.fuel = std::vector<double>(8, 0.0);

        // item 1: a mono tank (directly in the container)
        SavePart item1;
        item1.part = "mono_tank_r1";
        item1.uid = 202;
        item1.id = "mono_1";
        item1.mass = 88.99;
        item1.fuel = std::vector<double>(8, 0.0);
        item1.fuel[(int)ResourceType::Hydrazine] = 42.5;
        container.inventory.push_back(item1);

        // item 2: a kerbal suit (directly in the container), holding item 3
        SavePart item2;
        item2.part = "kerbal";
        item2.uid = 203;
        item2.id = "suit_1";
        item2.mass = 97.05;
        item2.fuel = std::vector<double>(8, 0.0);
        item2.fuel[(int)ResourceType::Hydrazine] = 3.0;
        // item 3: a small part nested inside item2
        SavePart item3;
        item3.part = "cargo";
        item3.uid = 204;
        item3.id = "nested_cargo_1";
        item3.mass = 250.0;
        item3.fuel = std::vector<double>(8, 0.0);
        item2.inventory.push_back(item3);
        container.inventory.push_back(item2);

        SavePart containerOut = savePartFromJson(savePartToJson(container));
        CHECK(containerOut.inventory.size() == 2);
        CHECK(containerOut.inventory[0].part == "mono_tank_r1");
        CHECK(containerOut.inventory[0].fuel[(int)ResourceType::Hydrazine] == 42.5);
        CHECK(containerOut.inventory[1].part == "kerbal");
        CHECK(containerOut.inventory[1].inventory.size() == 1);
        CHECK(containerOut.inventory[1].inventory[0].part == "cargo");
        CHECK(containerOut.inventory[1].inventory[0].uid == 204);

        // an absent inventory (a save that predates the field) stays empty
        SavePart noInv;
        noInv.part = "cargo";
        noInv.uid = 205;
        noInv.fuel = std::vector<double>(8, 0.0);
        SavePart noInvOut = savePartFromJson(savePartToJson(noInv));
        CHECK(noInvOut.inventory.empty());
    }

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
    CHECK(partOut.uid == 0);          // absent -- load refuses a part with no uid
    CHECK(partOut.parent == 0);       // 0 = the root
    CHECK(partOut.stage == 1);
    CHECK(vnear(partOut.pos, glm::dvec3(0.0)));
    CHECK(mnear(partOut.rot, glm::dmat3(1.0)));
    CHECK(near(partOut.hull_margin, -1.0));

    /* A reference keyed by STRING where a uid is expected reads back as 0 --
       which is what an old-format save looks like, and why load treats 0 as a
       hard error rather than a miss to paper over. */
    nlohmann::json oldFmt = nlohmann::json::object();
    oldFmt["part"] = "engine";
    oldFmt["uid"] = "engine_1";
    oldFmt["parent"] = "capsule_1";
    SavePart oldOut = savePartFromJson(oldFmt);
    CHECK(oldOut.uid == 0);
    CHECK(oldOut.parent == 0);

    /* The crew's container is uid-keyed too: a non-uid value (a string -- the
       shape an index-era / hand-edited file could have) reads back as the
       "absent" 0, which load refuses instead of silently parking the kerbal
       in part 0. A float would truncate to a valid uid and mis-resolve, so it
       is refused the same way. */
    nlohmann::json badCrew = nlohmann::json::object();
    badCrew["is_crew"] = true;
    badCrew["aboard"] = "racer";
    badCrew["aboard_part"] = "capsule_1";   // a string, not a uid
    CHECK(saveShipFromJson(badCrew).aboard_part == 0);
    nlohmann::json floatCrew = nlohmann::json::object();
    floatCrew["is_crew"] = true;
    floatCrew["aboard"] = "racer";
    floatCrew["aboard_part"] = 5.9;         // a float (would truncate to uid 5)
    CHECK(saveShipFromJson(floatCrew).aboard_part == 0);

    /* Two parts with the SAME id must still round-trip as two distinguishable
       parts. This is the shape a docked pair of same-def ships saves as, and
       the case that made id-keyed resolution silently pick one of them; uid
       is what keeps them apart. */
    {
        SaveShip dup;
        dup.name = "docked";
        dup.is_crew = false;
        SavePart a; a.part = "fuel_tank"; a.uid = 7; a.id = "fuel_tank_1"; a.parent = 0;
        SavePart b; b.part = "fuel_tank"; b.uid = 8; b.id = "fuel_tank_1"; b.parent = 7;
        dup.parts.push_back(a);
        dup.parts.push_back(b);
        dup.controller = 8;                                  // the SECOND one
        dup.fuel_links.push_back(SaveFuelLink{ 7, 8 });
        dup.docks.push_back(SaveDock{ 7, 8, "docked" });
        SaveShip dupOut = saveShipFromJson(saveShipToJson(dup));
        CHECK(dupOut.parts.size() == 2);
        if(dupOut.parts.size() == 2) {
            CHECK(dupOut.parts[0].id == dupOut.parts[1].id);   // ids DO collide
            CHECK(dupOut.parts[0].uid == 7);
            CHECK(dupOut.parts[1].uid == 8);                   // uids do not
            CHECK(dupOut.parts[1].parent == 7);
        }
        CHECK(dupOut.controller == 8);
        CHECK(dupOut.fuel_links.size() == 1);
        if(dupOut.fuel_links.size() == 1) {
            CHECK(dupOut.fuel_links[0].from == 7);
            CHECK(dupOut.fuel_links[0].to == 8);
        }
        CHECK(dupOut.docks.size() == 1);
        if(dupOut.docks.size() == 1) {
            CHECK(dupOut.docks[0].port == 7);
            CHECK(dupOut.docks[0].root == 8);
        }
    }

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
