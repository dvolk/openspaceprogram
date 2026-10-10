// test_buildings.cpp -- the building catalog + system world_buildings
// (src/buildings.cpp, src/system.cpp).
//
// Pinned here:
//   - res/data/buildings.json parses (at least the space_port type),
//   - world instances: id/type/body/lat/lon required; degrees -> equirectDir;
//     height_m/heading_deg defaults; flags; duplicate ids throw,
//   - buildingOrient: +Z along up, heading spins about that axis,
//   - every shipped system carries at least one VAB_LAUNCH_PAD on its home.

#include "buildings.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "equirect.h"
#include "system.h"
#include "terrain.h"

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static bool expect_throw(const std::function<void()> &fn) {
    try { fn(); }
    catch(const std::runtime_error &) { return true; }
    return false;
}

static bool vnear(const glm::dvec3 &a, const glm::dvec3 &b, double eps = 1e-9) {
    return glm::length(a - b) < eps;
}

static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) { throw std::runtime_error(std::string("open ") + path); }
    return nlohmann::json::parse(f, nullptr, true);
}

int main() {
    // --- catalog ---------------------------------------------------------
    BuildingsCatalog cat = load_buildings_catalog("res/data/buildings.json");
    CHECK(cat.buildings.size() >= 1);
    CHECK(cat.find("space_port") != nullptr);
    CHECK(cat.find("nope") == nullptr);
    {
        const BuildingDef *d = cat.find("space_port");
        CHECK(d != nullptr);
        if(d) {
            CHECK(!d->mesh.empty());
            CHECK(!d->texture.empty());
            CHECK(d->label() == d->display_name || d->display_name.empty());
        }
    }
    CHECK(expect_throw([]() { load_buildings_catalog("res/no_such_buildings.json"); }));

    // --- instance parse --------------------------------------------------
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "a", "type": "space_port", "body": "Earth",
                 "lat_deg": 10.0, "lon_deg": 20.0,
                 "height_m": 7.5, "heading_deg": 45.0,
                 "flags": ["VAB_LAUNCH_PAD"]}
            ]})");
        std::vector<WorldBuilding> ws = parse_world_buildings(doc, "mem");
        CHECK(ws.size() == 1);
        CHECK(ws[0].id == "a");
        CHECK(ws[0].hasFlag("VAB_LAUNCH_PAD"));
        CHECK(!ws[0].hasFlag("NOPE"));
        CHECK(ws[0].height_m == 7.5);
        CHECK(ws[0].heading_deg == 45.0);
        // degrees -> equirectDir(radians)
        const glm::dvec3 want = equirectDir(20.0 * std::numbers::pi / 180.0,
                                            10.0 * std::numbers::pi / 180.0);
        CHECK(vnear(ws[0].dir(), want));
    }
    {
        // defaults: height_m 5, heading 0, no flags
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "a", "type": "t", "body": "B", "lat_deg": 0, "lon_deg": 0}
            ]})");
        std::vector<WorldBuilding> ws = parse_world_buildings(doc, "mem");
        CHECK(ws.size() == 1);
        CHECK(ws[0].height_m == 5.0);
        CHECK(ws[0].heading_deg == 0.0);
        CHECK(ws[0].flags.empty());
    }
    CHECK(parse_world_buildings(nlohmann::json::object(), "mem").empty());
    {
        nlohmann::json doc = nlohmann::json::parse(R"({"world_buildings": {}})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [{"id": "x", "type": "t", "body": "B"}]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); })); // no lat/lon
    }
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "x", "type": "t", "body": "B", "lat_deg": 91, "lon_deg": 0}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "x", "type": "t", "body": "B", "lat_deg": 0, "lon_deg": 0},
                {"id": "x", "type": "t", "body": "B", "lat_deg": 0, "lon_deg": 1}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }

    // --- buildingOrient --------------------------------------------------
    {
        const glm::dvec3 up = glm::normalize(glm::dvec3(0.0, 1.0, 0.0));
        const glm::dmat3 m0 = buildingOrient(up, 0.0);
        // +Z along up (columns are the local axes in the parent frame)
        CHECK(vnear(glm::dvec3(m0[2]), up, 1e-12));
        // heading spins about up: local X rotates
        const glm::dmat3 m90 = buildingOrient(up, std::numbers::pi / 2.0);
        CHECK(vnear(glm::dvec3(m90[2]), up, 1e-12));
        // m90's X should be m0's Y (or -Y) for a +90 deg spin about Z
        CHECK(vnear(glm::dvec3(m90[0]), glm::dvec3(m0[1]), 1e-9)
              || vnear(glm::dvec3(m90[0]), -glm::dvec3(m0[1]), 1e-9));
    }

    // --- find_launch_site ------------------------------------------------
    {
        TerrainBody tb;
        auto mk = [&](const char *id, std::vector<std::string> flags) {
            StaticBuilding *p = new StaticBuilding;
            p->id = id;
            p->display_name = id;
            p->flags = std::move(flags);
            p->parent = &tb;
            tb.pads.push_back(p);
        };
        mk("scenic", {"NOT_A_PAD"});
        mk("main", {"VAB_LAUNCH_PAD"});
        mk("pole", {"VAB_LAUNCH_PAD", "POLAR_PAD"});
        CHECK(find_launch_site(&tb, "", false)->id == "main");
        CHECK(find_launch_site(&tb, "", true)->id == "pole");
        CHECK(find_launch_site(&tb, "pole", false)->id == "pole");
        // a sticky equatorial pick must not pre-empt pad-polar
        CHECK(find_launch_site(&tb, "main", true)->id == "pole");
        CHECK(find_launch_site(&tb, "scenic", false) == nullptr); // not a pad
        CHECK(find_launch_site(&tb, "nope", false) == nullptr);
        CHECK(find_launch_site(nullptr, "", false) == nullptr);
        for(auto *p : tb.pads) { delete p; }
        tb.pads.clear();
    }

    // --- shipped systems carry a home launch pad --------------------------
    for(const auto &entry : std::filesystem::directory_iterator("res/systems")) {
        if(entry.path().extension() != ".json") { continue; }
        const std::string path = entry.path().string();
        // load_system wants the logical "res/..." name for logs
        const std::string logical = "res/systems/" + entry.path().filename().string();
        System sys;
        try {
            sys = load_system(logical.c_str(), nullptr, nullptr);
        } catch(const std::exception &e) {
            printf("FAIL %s: load threw: %s\n", logical.c_str(), e.what());
            failures++;
            continue;
        }
        CHECK(sys.world_buildings.size() >= 1);
        bool homePad = false;
        for(const WorldBuilding &w : sys.world_buildings) {
            if(sys.find(w.body) == nullptr) {
                printf("FAIL %s: '%s' names unknown body '%s'\n",
                       logical.c_str(), w.id.c_str(), w.body.c_str());
                failures++;
            }
            if(sys.home != nullptr && w.body == sys.home->name
               && w.hasFlag("VAB_LAUNCH_PAD")) {
                homePad = true;
            }
            // type must resolve against the real catalog
            if(cat.find(w.type) == nullptr) {
                printf("FAIL %s: '%s' has unknown type '%s'\n",
                       logical.c_str(), w.id.c_str(), w.type.c_str());
                failures++;
            }
        }
        if(!homePad) {
            printf("FAIL %s: no VAB_LAUNCH_PAD on home '%s'\n",
                   logical.c_str(), sys.home ? sys.home->name.c_str() : "(none)");
            failures++;
        }
    }

    if(failures) {
        printf("test_buildings: %d FAILED\n", failures);
        return 1;
    }
    printf("test_buildings: OK\n");
    return 0;
}
