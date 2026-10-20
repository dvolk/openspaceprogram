// test_buildings.cpp -- the building catalog + system world_buildings
// (src/buildings.cpp, src/system.cpp).
//
// Pinned here:
//   - res/data/buildings.json parses (at least the space_port type),
//   - world instances: id/type/body/lat/lon required; degrees -> equirectDir;
//     height_m/heading_deg defaults; flags; duplicate ids throw,
//   - "world_buildings_files" fragments: merge order, missing/nested/empty
//     files and cross-source duplicate ids all throw,
//   - the shipped Earth site table (res/data/earth_launch_sites.json) reaches
//     the solar systems, in order, with Baikonur as the default,
//   - every shipped system's DEFAULT site (its first VAB_LAUNCH_PAD) is a
//     real pad rather than the pole, and its site fragment reaches it,
//   - buildingOrient: +Z along up, heading spins about that axis,
//   - a default-constructed TerrainBody is destructible: every owner pointer
//     null and every loader-filled scalar 0 (#227),
//   - every shipped system carries at least one VAB_LAUNCH_PAD on its home,
//   - no two pads on a body are on top of each other,
//   - every pad's lat/lon round-trips through the equirect projection the
//     Surface Map marks sites with,
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

    // --- world_buildings_files: named fragments merge in ------------------
    {
        // The shipped Earth table reaches the solar systems, and the merge
        // order survives: the merged list's first VAB_LAUNCH_PAD IS the
        // game's default site (find_launch_site's fallback), so the inline
        // pole pad must not come first.
        const std::string p = "res/systems/solar_system.json";
        std::vector<WorldBuilding> ws = parse_world_buildings_all(read_json(p.c_str()), p);
        CHECK(ws.size() >= 15);   // 14 real sites + the pole pad
        CHECK(!ws.empty() && ws[0].id == "baikonur");
        bool pole = false;
        for(const WorldBuilding &w : ws) {
            if(w.id == "polar_pad") { pole = true; }
        }
        CHECK(pole);
        // The table's own coordinates survive the parse (lon is east-positive,
        // so a west site is negative -- equirectDir takes either spelling).
        for(const WorldBuilding &w : ws) {
            if(w.id != "alcantara") { continue; }
            CHECK(w.lat_deg == -2.3394 && w.lon_deg == -44.4175);
            CHECK(vnear(w.dir(), equirectDir(-44.4175 * std::numbers::pi / 180.0,
                                             -2.3394 * std::numbers::pi / 180.0)));
        }
    }
    CHECK(expect_throw([]() {
        nlohmann::json doc = nlohmann::json::parse(
            R"({"world_buildings_files": ["res/no_such_sites.json"]})");
        parse_world_buildings_all(doc, "mem");
    }));
    {
        // A fragment names pads, not more fragments.
        std::filesystem::create_directories("tmp");
        const std::string nest = "tmp/test_buildings_nest.json";
        std::ofstream(nest)
            << R"({"world_buildings_files": ["res/data/earth_launch_sites.json"]})";
        nlohmann::json doc;
        doc["world_buildings_files"] = { nest };
        CHECK(expect_throw([&]() { parse_world_buildings_all(doc, "mem"); }));
        std::remove(nest.c_str());
    }
    {
        // A named file with no pads at all is a typo, whether the key is
        // missing or the array is empty: either way the system would quietly
        // fall back to its inline pads.
        std::filesystem::create_directories("tmp");
        const std::string bare = "tmp/test_buildings_bare.json";
        const std::string empty = "tmp/test_buildings_empty.json";
        std::ofstream(bare) << R"({"note": "no world_buildings here"})";
        std::ofstream(empty) << R"({"world_buildings": []})";
        for(const std::string &f : { bare, empty }) {
            nlohmann::json doc;
            doc["world_buildings_files"] = { f };
            CHECK(expect_throw([&]() { parse_world_buildings_all(doc, "mem"); }));
        }
        std::remove(bare.c_str());
        std::remove(empty.c_str());
    }
    {
        // An id used by a fragment AND the inline array would shadow one of
        // them in the VAB picker.
        nlohmann::json doc;
        doc["world_buildings_files"] = { "res/data/earth_launch_sites.json" };
        doc["world_buildings"] = nlohmann::json::array({
            { {"id", "baikonur"}, {"type", "space_port"}, {"body", "Earth"},
              {"lat_deg", 0.0}, {"lon_deg", 0.0} }
        });
        CHECK(expect_throw([&]() { parse_world_buildings_all(doc, "mem"); }));
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

    // --- a body nothing has built yet --------------------------------------
    /* #227: ~TerrainBody deletes patches[]/frame/rot_frame unconditionally and
       load_system registers each body before the first throw it can hit, so a
       half-built body is ALWAYS destructed. With uninitialized members that
       was a free() of stack garbage and this binary aborted.
       The const IS the pin: a const object whose default constructor leaves
       any member unset is ill-formed, so the compiler rejects the line the
       moment anyone adds a member without an initializer. The CHECKs below
       only document which sentinel is meant to be which -- a fresh stack slot
       can pass them by luck, so they are not the guard. */
    {
        const TerrainBody tb;
        CHECK(tb.shader == nullptr);
        CHECK(tb.frame == nullptr);
        CHECK(tb.rot_frame == nullptr);
        for(int i = 0; i < 6; i++) { CHECK(tb.patches[i] == nullptr); }
        CHECK(tb.atmosphere == nullptr && tb.clouds == nullptr
              && tb.ocean == nullptr);
        CHECK(tb.radius == 0.0f);
        CHECK(tb.mu == 0.0);
        CHECK(tb.g == 0.0);
        CHECK(tb.mass == 0.0f);
        CHECK(tb.colour_func == nullptr);
        CHECK(glm::length(tb.sunlightVec) == 0.0f);
        CHECK(!tb.ready);
    }   // ~TerrainBody on a never-attached body must not abort

    /* Same rule for the value types the body hands around (#230): free_shell
       deletes Shell::mesh and ::texture unconditionally (the shader is shared
       and never freed), and RootGeoms crosses to the main thread as a value.
       const makes the compiler reject either the moment a member is added
       without an initializer.
       These two and the TerrainBody above live HERE, not in test_terrain,
       because that binary is deliberately the GL/Bullet-free pure core
       (terragen.h + camera.o): ~TerrainBody is out-of-line in terrain.cpp and
       terrain.h needs Mesh/Texture/Shader. test_buildings is the test that
       links terrain.o, so it is also the only place a half-built body's
       destruction runs -- if ~TerrainBody ever grows a loop over a container,
       the abort lands here, and that is the #227 invariant failing loudly
       rather than a stray crash. */
    {
        const TerrainBody::Shell sh;
        CHECK(sh.mesh == nullptr && sh.shader == nullptr && sh.texture == nullptr);
        const TerrainBody::RootGeoms rg;
        CHECK(rg.max_depth == 0 && rg.max_height == 0.0f);
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
        CHECK(find_launch_site(&tb, "")->id == "main");        // default = first
        CHECK(find_launch_site(&tb, "pole")->id == "pole");    // any named pad
        CHECK(find_launch_site(&tb, "scenic") == nullptr);     // not a pad
        CHECK(find_launch_site(&tb, "nope") == nullptr);
        CHECK(find_launch_site(nullptr, "") == nullptr);
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
        // Every solar_system variant names the same site file, so each must
        // carry the whole table: the >= 1 above and homePad below would both
        // pass on the inline pole pad alone if a variant lost the reference.
        if(logical.find("solar_system") != std::string::npos) {
            CHECK(sys.world_buildings.size() >= 15);
        }
        // The fictional systems name a site fragment too (#226): 3 Kerbol
        // sites + its inline pole pad, 1 Eerbon site + its own. Exact counts
        // so a fragment that stops loading is a failure, not a smaller pass.
        if(logical == "res/systems/ksp_system.json") {
            CHECK(sys.world_buildings.size() == 4);
        }
        if(logical == "res/systems/old_system.json") {
            CHECK(sys.world_buildings.size() == 2);
        }
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
            /* The Surface Map projects a site as equirectLonLat(p->dir), the
               inverse of the equirectDir that built dir from these degrees.
               Pin the round-trip: a convention slip (lon sign, degrees vs
               radians, rotating vs non-rotating frame) would put a marker on
               the far side of the planet, and nothing else watches it. The
               pole pad is the interesting case -- cos(90 deg) is ~6e-17, not
               0, so its longitude survives atan2 and lands back where it was
               authored rather than at an arbitrary meridian. */
            {
                double rlon, rlat;
                equirectLonLat(w.dir(), rlon, rlat);
                const double rlat_deg = rlat * 180.0 / std::numbers::pi;
                // lon unwraps to [0, 2pi), so a west-negative authoring is
                // the same meridian one turn away.
                double rlon_deg = rlon * 180.0 / std::numbers::pi
                                - w.lon_deg;
                if(rlon_deg > 180.0) { rlon_deg -= 360.0; }
                if(rlon_deg < -180.0) { rlon_deg += 360.0; }
                if(std::fabs(rlat_deg - w.lat_deg) > 1e-9
                   || std::fabs(rlon_deg) > 1e-9) {
                    printf("FAIL %s: pad '%s' projects to lat %.10f lon "
                           "%.10f but is authored %.10f/%.10f\n",
                           logical.c_str(), w.id.c_str(), rlat_deg,
                           rlon * 180.0 / std::numbers::pi,
                           w.lat_deg, w.lon_deg);
                    failures++;
                }
            }
            // ... and the deck it puts on that body reaches the ground
            if(wb != nullptr) { check_pad_grounding(logical, w, wb); }
        }
        /* A body's DEFAULT site is the first VAB_LAUNCH_PAD ON THAT BODY
           (find_launch_site filters by body, and so does the VAB dropdown),
           with fragments merged before a system's inline pads -- so on the
           home world this is exactly what the "(default)" entry and a bare
           LAUNCH mean. A home defaulting to the pole is a merge-order bug:
           the polar pad lives in the inline array precisely so it cannot. */
        const WorldBuilding *dflt = nullptr;
        if(sys.home != nullptr) {
            for(const WorldBuilding &w : sys.world_buildings) {
                if(w.body == sys.home->name && w.hasFlag("VAB_LAUNCH_PAD")) {
                    dflt = &w;
                    break;
                }
            }
        }
        CHECK(dflt != nullptr);
        if(dflt != nullptr) {
            CHECK(!dflt->hasFlag("POLAR_PAD"));
            if(logical.find("solar_system") != std::string::npos) {
                CHECK(dflt->id == "baikonur");
            } else {
                CHECK(dflt->id == "main_space_port");
                // The fictional systems' tables are hand-authored data no
                // generator writes, so nothing else watches their numbers:
                // pin the home pad's spot, as alcantara does for Earth's.
                CHECK(dflt->lat_deg == 0.286473 && dflt->lon_deg == 270.286477);
            }
        }
        // Two pads on one body must not overlap: a deck is 26 x 56 m and pad
        // ships stand 20 m apart, so this floor catches a copy-paste
        // coordinate slip while still leaving room for two real pads at one
        // complex (the shipped Earth sites are ~679 km apart at closest).
        for(size_t i = 0; i < sys.world_buildings.size(); i++) {
            for(size_t k = i + 1; k < sys.world_buildings.size(); k++) {
                const WorldBuilding &a = sys.world_buildings[i];
                const WorldBuilding &b = sys.world_buildings[k];
                if(a.body != b.body) { continue; }
                TerrainBody *wb = sys.find(a.body);
                if(wb == nullptr) { continue; }
                // Chord x radius: over these separations it is the arc to
                // within a metre, and we only care about the order of magnitude.
                const double m = glm::length(a.dir() - b.dir())
                               * (double)wb->radius;
                if(m < 200.0) {
                    printf("FAIL %s: pads '%s' and '%s' sit %g m apart on %s\n",
                           logical.c_str(), a.id.c_str(), b.id.c_str(), m,
                           a.body.c_str());
                    failures++;
                }
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
