// test_buildings.cpp -- the building catalog + system world_buildings
// (src/buildings.cpp, src/system.cpp).
//
// Pinned here:
//   - res/data/buildings.json parses (at least the space_port type),
//   - world instances: id/type/body/lat/lon required; degrees -> equirectDir;
//     height_m/heading_deg defaults; flags; duplicate ids throw,
//   - buildingOrient: +Z along up, heading spins about that axis,
//   - every shipped system carries at least one VAB_LAUNCH_PAD on its home,
//   - every pad's deck actually reaches the ground under its footprint.

#include "buildings.h"

#include <algorithm>
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

/* A pad deck is a flat tangent-plane slab; the ground under it is not. That
   is why the building is buried at all: height_m lifts the deck clear of the
   surface sampled at the pad's own dir, and the mesh's own depth carries it
   down through whatever the relief does across the rest of the footprint.
   Pinned so a terrain-noise tweak or a moved pad cannot leave a corner of the
   deck floating over a dip, or bury the deck inside a hill (a pad ship would
   spawn in the ground). Footprint and depth are res/meshes/space_port.obj --
   a 12-vertex box with its top face exactly at the building origin
   (x -12.99..12.99, y -41.11..15.00, z -10.00..0.00); re-derive from the
   OBJ's v lines if that mesh changes. */
static void check_pad_grounding(const std::string &logical,
                                const WorldBuilding &w, TerrainBody *hb) {
    const double halfX = 13.0, yMin = -41.0, yMax = 15.0, slabDepth = 10.0;
    const glm::dvec3 up = w.dir();
    const glm::dvec3 org = up * (w.height_m
                                 + (double)hb->GetTerrainHeight(glm::vec3(up)));
    const glm::dmat3 orient =
        buildingOrient(up, w.heading_deg * std::numbers::pi / 180.0);
    double lo = 1e30, hi = -1e30;
    for(double u = -halfX; u <= halfX; u += 2.0) {
        for(double v = yMin; v <= yMax; v += 2.0) {
            // A point on the deck plane: its radius IS the deck height there,
            // so the difference is how far the deck sits above that ground.
            const glm::dvec3 q = org + orient * glm::dvec3(u, v, 0.0);
            const double clear = glm::length(q)
                - (double)hb->GetTerrainHeight(glm::vec3(glm::normalize(q)));
            lo = std::min(lo, clear);
            hi = std::max(hi, clear);
        }
    }
    if(lo <= 0.0) {
        printf("FAIL %s: pad '%s' deck sits %g m BELOW the ground across its "
               "footprint (a pad ship would spawn inside the terrain)\n",
               logical.c_str(), w.id.c_str(), -lo);
        failures++;
    }
    if(hi >= slabDepth) {
        printf("FAIL %s: pad '%s' deck sits %g m above the lowest ground on "
               "its footprint, past the %g m slab -- a corner floats\n",
               logical.c_str(), w.id.c_str(), hi, slabDepth);
        failures++;
    }
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
            TerrainBody *wb = sys.find(w.body);
            if(wb == nullptr) {
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
            // ... and the deck it puts on that body reaches the ground
            if(wb != nullptr) { check_pad_grounding(logical, w, wb); }
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
