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

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
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

    // --- flight journal (flog): round-trips so a recovered vessel after a
    // load shows its full mission, not one restarting at the load instant ---
    SaveShip fship;
    fship.name = "racer";
    fship.is_crew = false;
    fship.flog.started = true;
    fship.flog.start_t = 12.5;
    fship.flog.start_body = "Kerbin";
    fship.flog.last_body = "Mun";
    fship.flog.events.push_back(FlightEvent{12.5, true, "Kerbin"});
    fship.flog.events.push_back(FlightEvent{90.0, false, "Kerbin"});
    fship.flog.events.push_back(FlightEvent{90.0, true, "Mun"});
    SaveShip fOut = saveShipFromJson(saveShipToJson(fship));
    CHECK(fOut.flog.started == true);
    CHECK(near(fOut.flog.start_t, 12.5));
    CHECK(fOut.flog.start_body == "Kerbin");
    CHECK(fOut.flog.last_body == "Mun");
    CHECK(fOut.flog.events.size() == 3);
    if(fOut.flog.events.size() == 3) {
        CHECK(near(fOut.flog.events[1].t, 90.0));
        CHECK(fOut.flog.events[1].enter == false);
        CHECK(fOut.flog.events[1].body == "Kerbin");
        CHECK(fOut.flog.events[2].enter == true);
        CHECK(fOut.flog.events[2].body == "Mun");
    }

    // A journal that never started writes NO flog key, and loads back default
    // (so the placement's setSoi starts it fresh) -- and an old save with no
    // flog key at all is indistinguishable from that, hence backward-safe.
    SaveShip fresh;
    fresh.name = "racer";
    fresh.is_crew = false;
    nlohmann::json freshJ = saveShipToJson(fresh);
    CHECK(!freshJ.contains("flog"));
    SaveShip freshOut = saveShipFromJson(freshJ);
    CHECK(freshOut.flog.started == false);
    CHECK(freshOut.flog.events.empty());

    // Crew carry a journal too (the same shared field).
    SaveShip fcrew;
    fcrew.name = "kerbal";
    fcrew.is_crew = true;
    fcrew.aboard = "racer";
    fcrew.aboard_part = 101;
    fcrew.flog.started = true;
    fcrew.flog.start_body = "Kerbin";
    fcrew.flog.last_body = "Kerbin";
    fcrew.flog.events.push_back(FlightEvent{5.0, true, "Kerbin"});
    SaveShip fcOut = saveShipFromJson(saveShipToJson(fcrew));
    CHECK(fcOut.flog.started == true);
    CHECK(fcOut.flog.last_body == "Kerbin");
    CHECK(fcOut.flog.events.size() == 1);

    // A started journal with ZERO events (a vessel placed in no SoI body, or
    // one whose only observe was an empty body) still round-trips: the emit
    // rule keys on `started`, not on events being non-empty.
    SaveShip zship;
    zship.name = "racer";
    zship.is_crew = false;
    zship.flog.started = true;
    zship.flog.start_t = 7.0;
    zship.flog.start_body = "";
    zship.flog.last_body = "";
    nlohmann::json zj = saveShipToJson(zship);
    CHECK(zj.contains("flog"));
    SaveShip zOut = saveShipFromJson(zj);
    CHECK(zOut.flog.started == true);
    CHECK(near(zOut.flog.start_t, 7.0));
    CHECK(zOut.flog.events.empty());

    // A malformed flog degrades to defaults and never throws (the permissive
    // read matches the rest of this file): a non-object flog, a non-array
    // events, and event entries with missing/wrong-typed fields.
    SaveShip flogBad = saveShipFromJson(nlohmann::json::parse(
        R"({"name":"x","is_crew":false,"flog":42})"));
    CHECK(flogBad.flog.started == false);
    CHECK(flogBad.flog.events.empty());
    SaveShip flogBad2 = saveShipFromJson(nlohmann::json::parse(
        R"({"name":"x","is_crew":false,"flog":{"started":true,"events":"nope"}})"));
    CHECK(flogBad2.flog.started == true);
    CHECK(flogBad2.flog.events.empty());
    SaveShip flogBad3 = saveShipFromJson(nlohmann::json::parse(
        R"({"name":"x","is_crew":false,"flog":{"started":true,"events":[{"body":7},{"t":"x","enter":"y","body":"Kerbin"}]}})"));
    CHECK(flogBad3.flog.events.size() == 2);
    if(flogBad3.flog.events.size() == 2) {
        CHECK(flogBad3.flog.events[0].body.empty());   // non-string body -> default
        CHECK(flogBad3.flog.events[1].t == 0.0);        // non-number t -> default
        CHECK(flogBad3.flog.events[1].enter == true);   // non-bool enter -> default (true)
        CHECK(flogBad3.flog.events[1].body == "Kerbin");
    }

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

    // --- science: experiments on a suit + on a part, and the meta score ----
    {
        Experiment e1;
        e1.type = "observation";
        e1.body = "Mun";
        e1.situation = SciSituation::LowOrbit;
        e1.biome = "midlands";
        Experiment e2;
        e2.type = "observation";
        e2.body = "Mun";
        e2.situation = SciSituation::HighOrbit;
        e2.biome = "midlands";

        SaveShip crewSci;
        crewSci.name = "kerbal";
        crewSci.is_crew = true;
        crewSci.aboard = "racer";
        crewSci.aboard_part = 101;
        crewSci.suit_experiments.push_back(e1);
        crewSci.suit_experiments.push_back(e2);
        SaveShip crewSciOut = saveShipFromJson(saveShipToJson(crewSci));
        CHECK(crewSciOut.suit_experiments.size() == 2);
        if(crewSciOut.suit_experiments.size() == 2) {
            CHECK(crewSciOut.suit_experiments[0] == e1);
            CHECK(crewSciOut.suit_experiments[1] == e2);
            CHECK(crewSciOut.suit_experiments[0].situation == SciSituation::LowOrbit);
            CHECK(crewSciOut.suit_experiments[1].situation == SciSituation::HighOrbit);
        }

        // an absent suit_experiments (pre-science save) stays empty
        CHECK(saveShipFromJson(saveShipToJson(crewOld)).suit_experiments.empty());

        // a part's own experiment list (future instrument parts)
        SavePart p;
        p.part = "capsule";
        p.uid = 301;
        p.id = "capsule_1";
        p.experiments.push_back(e1);
        SavePart pOut = savePartFromJson(savePartToJson(p));
        CHECK(pOut.experiments.size() == 1);
        if(pOut.experiments.size() == 1) { CHECK(pOut.experiments[0] == e1); }
        CHECK(savePartFromJson(savePartToJson(SavePart{})).experiments.empty());
    }

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

    // phase 4.6: the SUIT inventory round-trips on the crew ship (the pocket
    // items a kerbal carries free), alongside the suit's own fuel
    {
        SaveShip crewInv;
        crewInv.name = "kerbal";
        crewInv.defPath = "./res/ships/kerbal.json";
        crewInv.is_crew = true;
        crewInv.aboard = "racer";
        crewInv.aboard_part = 101;
        for(int r = 0; r < 8; r++) {
            crewInv.suit_fuel.push_back(r == (int)ResourceType::Hydrazine ? 8.5 : 0.0);
        }
        SavePart pocket;
        pocket.part = "mono_tank_r1";
        pocket.uid = 301;
        pocket.id = "pocket_tank_1";
        pocket.mass = 88.99;
        pocket.fuel = std::vector<double>(8, 0.0);
        pocket.fuel[(int)ResourceType::Hydrazine] = 78.54;
        // and a nested item, depth-first like a ship part's inventory
        SavePart nested;
        nested.part = "cargo";
        nested.uid = 302;
        nested.id = "pocket_cargo_1";
        nested.mass = 250.0;
        nested.fuel = std::vector<double>(8, 0.0);
        pocket.inventory.push_back(nested);
        crewInv.suit_inventory.push_back(pocket);

        SaveShip crewInvOut = saveShipFromJson(saveShipToJson(crewInv));
        CHECK(crewInvOut.suit_inventory.size() == 1);
        if(crewInvOut.suit_inventory.size() == 1) {
            CHECK(crewInvOut.suit_inventory[0].part == "mono_tank_r1");
            CHECK(crewInvOut.suit_inventory[0].uid == 301);
            CHECK(near(crewInvOut.suit_inventory[0].fuel[(int)ResourceType::Hydrazine],
                       78.54));
            CHECK(crewInvOut.suit_inventory[0].inventory.size() == 1);
            if(crewInvOut.suit_inventory[0].inventory.size() == 1) {
                CHECK(crewInvOut.suit_inventory[0].inventory[0].uid == 302);
            }
        }
        CHECK(crewInvOut.suit_fuel.size() == 8);
        CHECK(near(crewInvOut.suit_fuel[6], 8.5));   // the suit's own fuel

        // a crew save without the field (predates it) stays empty
        SaveShip crewNoInv;
        crewNoInv.name = "kerbal";
        crewNoInv.defPath = "./res/ships/kerbal.json";
        crewNoInv.is_crew = true;
        crewNoInv.aboard = "racer";
        crewNoInv.aboard_part = 101;
        SaveShip crewNoInvOut = saveShipFromJson(saveShipToJson(crewNoInv));
        CHECK(crewNoInvOut.suit_inventory.empty());
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
    meta.system = "res/systems/ksp_system.json";
    meta.parts = "res/data/parts.json";
    meta.time = 4.459999999999993;
    meta.time_accel = 1;
    meta.active_ship = "racer";
    meta.exhaust_scale = 2.5f;
    meta.science_score = 7;
    {
        Experiment e;
        e.body = "Mun";
        e.situation = SciSituation::HighOrbit;
        e.biome = "mountains";
        meta.recovered.push_back(e);
    }
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
    CHECK(near(metaOut.exhaust_scale, meta.exhaust_scale));
    CHECK(metaOut.science_score == 7);
    CHECK(metaOut.recovered.size() == 1);
    if(metaOut.recovered.size() == 1) {
        CHECK(metaOut.recovered[0] == meta.recovered[0]);
    }
    CHECK(metaOut.ships.size() == 2);
    CHECK(metaOut.ships[0] == "v0");
    CHECK(metaOut.ships[1] == "v1");

    // --- permissive reads: an empty / partial document never crashes -------
    SaveMeta emptyMeta = saveMetaFromJson(nlohmann::json::object());
    CHECK(emptyMeta.format == 1);            // the defaults survive
    CHECK(emptyMeta.ships.empty());
    CHECK(emptyMeta.active_ship.empty());
    CHECK(near(emptyMeta.exhaust_scale, 1.0));  // missing -> the 1.0 default
    CHECK(emptyMeta.science_score == 0);
    CHECK(emptyMeta.recovered.empty());

    // a hand-edited scale is clamped to the CLI range (0.5-5)
    nlohmann::json wild = nlohmann::json::object();
    wild["exhaust_scale"] = 100.0;
    CHECK(near(saveMetaFromJson(wild).exhaust_scale, 5.0));
    wild["exhaust_scale"] = 0.0;
    CHECK(near(saveMetaFromJson(wild).exhaust_scale, 0.5));

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
    // Setup + assertions use std::filesystem too, like the code under test.
    namespace fs = std::filesystem;
    auto write_file = [](const std::string &p) {
        fs::create_directories(fs::path(p).parent_path());
        std::ofstream(p) << "{}";
    };
    const std::string base = "tmp/test_save_dir";
    fs::remove_all(base);

    // ensure_dir: creates the dir and any missing parents (no-op if exists).
    ensure_dir(base + "/a/b/c");
    CHECK(fs::exists(base + "/a/b/c"));
    ensure_dir(base + "/a/b/c");   // again: no throw, still there
    CHECK(fs::exists(base + "/a/b/c"));

    // list_saves: a subdir WITH a save.json is listed (sorted); one without
    // is skipped, and so is a plain file at the top level.
    fs::create_directories(base + "/bravo");   // no save.json -> skip
    write_file(base + "/note.txt");            // a file, not a dir
    write_file(base + "/alpha/save.json");
    write_file(base + "/charlie/save.json");
    std::vector<std::string> saves = list_saves(base);
    CHECK(saves.size() == 2);
    if(saves.size() == 2) {
        CHECK(saves[0] == "alpha");    // sorted: alpha < charlie
        CHECK(saves[1] == "charlie");
    }

    // delete_save: refuses anything not STRICTLY under base -- a sibling that
    // shares the prefix, a leading "..", a nested ".." that climbs out, and
    // base itself (which would wipe the whole saves dir) -- and deletes a
    // valid in-base path, leaving the rest.
    bool refused = false;
    try { delete_save("tmp/otherplace", base); }
    catch(const std::exception &) { refused = true; }
    CHECK(refused);

    refused = false;
    try { delete_save(base + "/../evil", base); }
    catch(const std::exception &) { refused = true; }
    CHECK(refused);

    refused = false;
    try { delete_save(base + "/alpha/../../evil", base); }
    catch(const std::exception &) { refused = true; }
    CHECK(refused);   // a nested ".." that escapes base

    refused = false;
    try { delete_save(base, base); }
    catch(const std::exception &) { refused = true; }
    CHECK(refused);   // dir == base
    CHECK(!fs::exists("tmp/evil"));   // none of the escapes happened

    delete_save(base + "/alpha", base);
    CHECK(!fs::exists(base + "/alpha"));   // alpha is gone
    CHECK(fs::exists(base + "/charlie/save.json"));  // charlie stays

    fs::remove_all(base);   // clean up the scratch dir

    // saveShipBodies: the bodies the saved fleet sits on, in fleet order
    // (pose.body, falling back to the ship's home; unique, first-wins) --
    // the set the boot --load path sync-builds. Corrupt / missing ship files
    // are skipped, a missing dir yields the empty set rather than a throw.
    {
        const std::string sb = "tmp/test_save_shipbodies";
        fs::remove_all(sb);
        fs::create_directories(sb + "/ships");
        {
            std::ofstream f(sb + "/save.json");
            f << R"({"ships":["v0","v1","v2","v3","v4"]})";
        }
        auto ship_json = [&](const std::string &slug, const std::string &body,
                             const std::string &home) {
            nlohmann::json s = nlohmann::json::object();
            if(!body.empty()) {
                s["pose"] = nlohmann::json::object();
                s["pose"]["body"] = body;
            }
            if(!home.empty()) { s["home"] = home; }
            std::ofstream f(sb + "/ships/" + slug + ".json");
            f << s.dump();
        };
        ship_json("v0", "Mun", "Kerbin");   // pose.body wins over home
        ship_json("v1", "", "Kerbin");      // no pose -> the home fallback
        { std::ofstream f(sb + "/ships/v2.json"); f << "{not json"; }   // corrupt
        ship_json("v3", "Mun", "");         // duplicates v0's body -> deduped
        // v4: listed in the meta but the file is missing
        std::vector<std::string> bodies = saveShipBodies(sb);
        CHECK(bodies.size() == 2);
        if(bodies.size() == 2) {
            CHECK(bodies[0] == "Mun");      // fleet order
            CHECK(bodies[1] == "Kerbin");   // the home fallback
        }
        CHECK(saveShipBodies("tmp/test_save_shipbodies_nope").empty());
        fs::remove_all(sb);
    }

    // --- games: the <stamp>-<name> dirs under saves/ -----------------------
    // The stamp is a local-time round-trip (mktime is localtime_r's inverse),
    // and the display name strips exactly ONE leading <stamp>-.
    {
        struct tm m = {};
        m.tm_year = 2026 - 1900;
        m.tm_mon = 8;      // September
        m.tm_mday = 29;
        m.tm_hour = 17;
        m.tm_min = 40;
        m.tm_sec = 12;
        m.tm_isdst = -1;
        CHECK(gameStamp(mktime(&m)) == "20260929_174012");
    }
    CHECK(gameDirName("20260929_174012-My Career") == "My Career");
    CHECK(gameDirName("20260929_174012-game1") == "game1");
    // A name that itself starts with stamp-like digits still round-trips:
    // exactly one leading stamp is stripped (the one newGameDir prepended).
    CHECK(gameDirName("20260929_174012-20260101_000000-restart")
          == "20260101_000000-restart");
    // Renamed / mangled by hand: no valid leading stamp -> the whole name.
    CHECK(gameDirName("My Career") == "My Career");
    CHECK(gameDirName("2026092-174012-Career") == "2026092-174012-Career");
    CHECK(gameDirName("x20260929_174012-Career") == "x20260929_174012-Career");

    // newGameDir: <stamp>-<name> under base; a taken dir bumps the second.
    {
        const std::string base = "tmp/test_save_gamedirs";
        fs::remove_all(base);
        struct tm m = {};
        m.tm_year = 2026 - 1900;
        m.tm_mon = 8;
        m.tm_mday = 29;
        m.tm_hour = 17;
        m.tm_min = 40;
        m.tm_sec = 12;
        m.tm_isdst = -1;
        const time_t t = mktime(&m);
        CHECK(newGameDir(base, "game1", t) == base + "/20260929_174012-game1");
        // a same-named game started the same second gets the next second
        fs::create_directories(base + "/20260929_174012-game1");
        CHECK(newGameDir(base, "game1", t) == base + "/20260929_174013-game1");
        // ...and a differently-named one may share the original second
        CHECK(newGameDir(base, "solar", t) == base + "/20260929_174012-solar");
        fs::remove_all(base);
    }

    // list_games + find_slot: the two-tier saves/<game>/<slot> layout.
    {
        const std::string base = "tmp/test_save_games/saves";
        fs::remove_all("tmp/test_save_games");
        auto slot = [&](const std::string &game, const std::string &s) {
            fs::create_directories(base + "/" + game + "/" + s);
            std::ofstream f(base + "/" + game + "/" + s + "/save.json");
            f << "{}";
        };
        // two same-named games (different stamps), one other game, a renamed
        // dir, a legacy flat save, and a game that never saved (hidden)
        slot("20260101_100000-Career", "save1");
        slot("20260202_110000-Career", "save1");   // save1 -> ambiguous
        slot("20260303_120000-Solar", "orbit");    // orbit -> unique two-tier
        slot("My Career", "save3");                // hand-renamed dir
        fs::create_directories(base + "/save2");   // legacy flat save
        { std::ofstream f(base + "/save2/save.json"); f << "{}"; }
        fs::create_directories(base + "/20260404_140000-Empty");   // no slots

        std::vector<GameEntry> gs = list_games(base);
        CHECK(gs.size() == 4);
        if(gs.size() == 4) {
            CHECK(gs[0].dirName == "20260101_100000-Career");
            CHECK(gs[0].name == "Career (2026-01-01 10:00)");
            CHECK(gs[1].dirName == "20260202_110000-Career");
            CHECK(gs[1].name == "Career (2026-02-02 11:00)");
            CHECK(gs[2].dirName == "20260303_120000-Solar");
            CHECK(gs[2].name == "Solar");          // a unique label stays bare
            CHECK(gs[3].dirName == "My Career");
            CHECK(gs[3].name == "My Career");
        }

        CHECK(find_slot(base, "save2") == base + "/save2");   // legacy flat
        CHECK(find_slot(base, "orbit")
              == base + "/20260303_120000-Solar/orbit");
        CHECK(find_slot(base, "save3") == base + "/My Career/save3");
        CHECK(find_slot(base, "save1").empty());   // two games hold it
        CHECK(find_slot(base, "missing").empty());
        fs::remove_all("tmp/test_save_games");
    }

    // --- quicksaves: the quicksave-00..99 rotating pool --------------------
    // NN parsing (exactly two digits), next-slot selection (max NN + 1, and
    // overwrite-the-oldest-by-mtime at the wrap), and the newest-by-mtime
    // load target (mtime, not NN: after a wrap the number order lies).
    {
        CHECK(quicksaveNN("quicksave-00") == 0);
        CHECK(quicksaveNN("quicksave-42") == 42);
        CHECK(quicksaveNN("quicksave-99") == 99);
        CHECK(quicksaveNN("quicksave-0") == -1);     // one digit: not a slot
        CHECK(quicksaveNN("quicksave-0a") == -1);
        CHECK(quicksaveNN("quicksave-100") == -1);   // three digits: not one
        CHECK(quicksaveNN("my-quicksave-00") == -1);
        CHECK(quicksaveNN("save1") == -1);
        CHECK(quicksaveName(0) == "quicksave-00");
        CHECK(quicksaveName(42) == "quicksave-42");
        CHECK(quicksaveName(99) == "quicksave-99");
    }
    {
        const std::string base = "tmp/test_save_quicksave/game";
        fs::remove_all("tmp/test_save_quicksave");
        // a pool slot whose save.json is `age` seconds old (the pool orders
        // by the FILE's mtime -- a rewrite moves the file, not the dir).
        // The stream is closed in its own scope BEFORE last_write_time: an
        // ofstream flushes on destruction, and a flush after the mtime set
        // would reset it to "now".
        auto slot = [&](const std::string &s, int age) {
            fs::create_directories(base + "/" + s);
            {
                std::ofstream f(base + "/" + s + "/save.json");
                f << "{}";
            }
            std::error_code ec;
            fs::last_write_time(base + "/" + s + "/save.json",
                fs::file_time_type::clock::now() - std::chrono::seconds(age),
                ec);
        };
        // an empty (or missing) game dir -> the first slot; nothing to load
        CHECK(nextQuicksave(base) == "quicksave-00");
        CHECK(latestQuicksave(base).empty());

        // a few slots: next is max NN + 1 (holes are not chased)
        slot("quicksave-00", 300);
        slot("quicksave-01", 200);
        slot("quicksave-05", 100);
        CHECK(nextQuicksave(base) == "quicksave-06");
        // the load target is the newest by MTIME, not the highest NN
        slot("quicksave-04", 50);
        CHECK(latestQuicksave(base) == "quicksave-04");
        // a non-quicksave slot is invisible to the pool
        slot("save1", 10);
        CHECK(nextQuicksave(base) == "quicksave-06");
        CHECK(latestQuicksave(base) == "quicksave-04");

        // a hole past the top (max NN is 99) wraps: overwrite the oldest
        slot("quicksave-99", 5);
        CHECK(nextQuicksave(base) == "quicksave-00");   // age 300, oldest
        fs::remove_all("tmp/test_save_quicksave");
    }
    {
        // The in-place-rewrite case (the bug a dir-mtime design has): a slot
        // OVERWRITTEN most recently is the newest even though it was CREATED
        // first. save_game rewrites save.json in place, so its mtime -- not
        // the slot dir's -- is the pool's age. Here 00 is created first, 01
        // second, then 00 is rewritten (age 5) and becomes the newest.
        const std::string base = "tmp/test_save_quickrewrite/game";
        fs::remove_all("tmp/test_save_quickrewrite");
        auto slot = [&](const std::string &s, int age) {
            fs::create_directories(base + "/" + s);
            {
                std::ofstream f(base + "/" + s + "/save.json");
                f << "{}";
            }
            std::error_code ec;
            fs::last_write_time(base + "/" + s + "/save.json",
                fs::file_time_type::clock::now() - std::chrono::seconds(age),
                ec);
        };
        slot("quicksave-00", 100);   // created first
        slot("quicksave-01", 50);    // created second
        CHECK(latestQuicksave(base) == "quicksave-01");
        slot("quicksave-00", 5);    // ...then 00 is rewritten (newest)
        CHECK(latestQuicksave(base) == "quicksave-00");
        // a third slot, oldest written: still not the full pool, so next is
        // max NN + 1 (the oldest-overwrite rule is the 100-slot test's)
        fs::create_directories(base + "/quicksave-02");
        {
            std::ofstream f(base + "/quicksave-02/save.json");
            f << "{}";
        }
        std::error_code ec;
        fs::last_write_time(base + "/quicksave-02/save.json",
            fs::file_time_type::clock::now() - std::chrono::seconds(200), ec);
        CHECK(latestQuicksave(base) == "quicksave-00");
        CHECK(nextQuicksave(base) == "quicksave-03");   // max NN + 1
        fs::remove_all("tmp/test_save_quickrewrite");
    }
    {
        // pool full (all 100 slots) -> overwrite the oldest by mtime
        const std::string base = "tmp/test_save_quickfull/game";
        fs::remove_all("tmp/test_save_quickfull");
        for(int nn = 0; nn < 100; nn++) {
            const std::string s = quicksaveName(nn);
            fs::create_directories(base + "/" + s);
            {
                std::ofstream f(base + "/" + s + "/save.json");
                f << "{}";
            }
            std::error_code ec;
            // ages 1..1000 apart, quicksave-37 the oldest, 99 the newest
            const int age = (nn == 37) ? 100000 : (1000 - nn);
            fs::last_write_time(base + "/" + s + "/save.json",
                fs::file_time_type::clock::now() - std::chrono::seconds(age),
                ec);
        }
        CHECK(nextQuicksave(base) == "quicksave-37");
        CHECK(latestQuicksave(base) == "quicksave-99");
        fs::remove_all("tmp/test_save_quickfull");
    }

    if(failures) {
        printf("test_save: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("test_save: OK\n");
    return 0;
}
