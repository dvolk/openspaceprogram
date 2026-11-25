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
//   - every pad's deck actually reaches the ground under its footprint,
//   - where the shipped fictional system's pad decks stand above their body's
//     sea-level datum, because comments elsewhere quote those numbers as if
//     they were authored (#983),
//   - no two buildings in one system answer to the same label, and no shipped
//     world needs the placement-time rename that keeps labels unique (#883).

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
#include "tchecks.h"
#include "tload.h"

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
   spawn in the ground). Footprint and depth are the building mesh's box, top
   face at the origin: space_port.obj is x -12.99..12.99, y -41.11..15.00,
   z -10..0; runway.obj (utils/art/gen_runway.py) is x +-450, y +-15, z -4..0.
   Re-derive from the OBJ's v lines if a mesh changes. */
static void check_pad_grounding(const std::string &logical,
                                const WorldBuilding &w, TerrainBody *hb) {
    double halfX = 13.0, yMin = -41.0, yMax = 15.0, slabDepth = 10.0;
    if(w.type == "runway") {
        halfX = 450.0; yMin = -15.0; yMax = 15.0; slabDepth = 4.0;
    }
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
        g_failures++;
    }
    if(hi >= slabDepth) {
        printf("FAIL %s: pad '%s' deck sits %g m above the lowest ground on "
               "its footprint, past the %g m slab -- a corner floats\n",
               logical.c_str(), w.id.c_str(), hi, slabDepth);
        g_failures++;
    }
}

/* A pad deck's height above the body's sea-level DATUM: height_m lifts the deck
   clear of the surface sampled at the pad's own dir, and seaLevelRadius() is the
   sphere the air column and the ocean shell are weighed against (#850). So this is
   the DECK's own altitude; a hull parked on it rides a few metres higher still,
   because build_ship shifts it by the support margin (vehicle.cpp:209) -- which is
   why an in-game spawn at moho_pad reads ~130 m while the deck is 125.9. The
   number is not authored anywhere: it is generated terrain, so it moves when the
   noise does. #983: comments quoted Moho's pad at 911 m while this build puts the
   deck at 125.9 m. */
static double padDeckAsl(const WorldBuilding &w, const TerrainBody *hb) {
    const glm::dvec3 up = w.dir();
    return w.height_m + (double)hb->GetTerrainHeight(glm::vec3(up))
           - hb->seaLevelRadius();
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

    // --- orbital instance parse ------------------------------------------
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 400000, "inc_deg": 51.6, "raan_deg": 10,
                           "phase_deg": 90},
                 "attitude": "body-fixed",
                 "spin": {"rate_deg_s": 2.0, "axis": [0, 0, 1]},
                 "flags": ["ORBITAL"]}
            ]})");
        std::vector<WorldBuilding> ws = parse_world_buildings(doc, "mem");
        CHECK(ws.size() == 1);
        CHECK(ws[0].orbital);
        CHECK(ws[0].orb_alt_m == 400000.0);
        CHECK(ws[0].orb_inc_deg == 51.6);
        CHECK(ws[0].orb_phase_deg == 90.0);
        CHECK(ws[0].att_mode == "body-fixed");
        CHECK(ws[0].spin_rate_deg_s == 2.0);
        CHECK(vnear(ws[0].spin_axis, glm::dvec3(0, 0, 1)));
    }
    {
        // defaults: attitude body-fixed, no spin, inc/raan/phase 0
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 300000}}
            ]})");
        std::vector<WorldBuilding> ws = parse_world_buildings(doc, "mem");
        CHECK(ws.size() == 1 && ws[0].orbital);
        CHECK(ws[0].att_mode == "body-fixed");
        CHECK(ws[0].spin_rate_deg_s == 0.0);
        CHECK(ws[0].orb_inc_deg == 0.0);
    }
    {   // orbital XOR surface lat/lon
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 300000}, "lat_deg": 0, "lon_deg": 0}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // missing alt_m
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin", "orbit": {}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // non-positive alt_m
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": -5}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // bad attitude
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 300000}, "attitude": "sideways"}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // zero spin axis
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 300000},
                 "spin": {"rate_deg_s": 1, "axis": [0, 0, 0]}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // non-numeric angle: a contextual runtime_error, not a json type_error
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "st", "type": "station", "body": "Kerbin",
                 "orbit": {"alt_m": 300000, "inc_deg": "lots"}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }

    // --- air instance parse (balloon platforms) ---------------------------
    {
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 10, "lon_deg": 0},
                 "heading_deg": 180,
                 "flags": ["VAB_LAUNCH_PAD", "AIR_PLATFORM"]}
            ]})");
        std::vector<WorldBuilding> ws = parse_world_buildings(doc, "mem");
        CHECK(ws.size() == 1);
        CHECK(ws[0].air && !ws[0].orbital);
        CHECK(ws[0].air_alt_m == 56000.0);
        CHECK(ws[0].air_lat_deg == 10.0);
        CHECK(ws[0].air_lon_deg == 0.0);
        CHECK(ws[0].heading_deg == 180.0);
        CHECK(ws[0].hasFlag("AIR_PLATFORM"));
    }
    {   // air XOR surface lat/lon
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 10, "lon_deg": 0},
                 "lat_deg": 0, "lon_deg": 0}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // air XOR orbit
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 10, "lon_deg": 0},
                 "orbit": {"alt_m": 300000}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // missing / non-positive alt_m
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"lat_deg": 10, "lon_deg": 0}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
        nlohmann::json doc2 = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 0, "lat_deg": 10, "lon_deg": 0}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc2, "mem"); }));
    }
    {   // missing lat_deg / lon_deg
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lon_deg": 0}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
        nlohmann::json doc2 = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 10}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc2, "mem"); }));
    }
    {   // polar platform: the drift divides by cos(lat), refused not clamped
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 89, "lon_deg": 0}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // non-numeric lat: contextual runtime_error, not a json type_error
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": "north", "lon_deg": 0}}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // attitude/spin are orbital-only; an air platform holds heading_deg
        nlohmann::json doc = nlohmann::json::parse(R"({
            "world_buildings": [
                {"id": "bp", "type": "balloon_platform", "body": "Venus",
                 "air": {"alt_m": 56000, "lat_deg": 10, "lon_deg": 0},
                 "attitude": "body-fixed"}
            ]})");
        CHECK(expect_throw([&]() { parse_world_buildings(doc, "mem"); }));
    }
    {   // the catalog entry carries the deck-only collision mesh
        const BuildingDef *bp = cat.find("balloon_platform");
        CHECK(bp != nullptr);
        if(bp != nullptr) {
            CHECK(bp->collision_mesh == "meshes/balloon_platform_deck.obj");
        }
    }

    // --- air placement validation (needs a real system's bodies) ----------
    {
        System sys;
        if(load_shipped("res/systems/solar_system.json", sys)) {
            auto airOn = [&](const char *body, double alt) {
                WorldBuilding w;
                w.id = "probe"; w.type = "balloon_platform"; w.body = body;
                w.air = true; w.air_alt_m = alt;
                w.air_lat_deg = 10.0; w.air_lon_deg = 0.0;
                return w;
            };
            const std::vector<WorldBuilding> saved = sys.world_buildings;
            // Venus has wind-bearing air up to 116 km: 56 km is a valid float.
            sys.world_buildings = {airOn("Venus", 56000)};
            bool ok = true;
            try { validate_world_buildings(sys, cat); }
            catch(const std::exception &) { ok = false; }
            CHECK(ok);
            // Above the air top: vacuum, nothing to float in.
            sys.world_buildings = {airOn("Venus", 120000)};
            CHECK(expect_throw([&]() { validate_world_buildings(sys, cat); }));
            // The Moon has no atmosphere at all.
            sys.world_buildings = {airOn("Moon", 56000)};
            CHECK(expect_throw([&]() { validate_world_buildings(sys, cat); }));
            // Pluto has physical air but NO wind table: windAt would assert.
            sys.world_buildings = {airOn("Pluto", 5000)};
            CHECK(expect_throw([&]() { validate_world_buildings(sys, cat); }));
            sys.world_buildings = saved;
        }
    }

    /* --- one id, one building, across the whole SYSTEM (#890) -------------
       What is pinned is the SCOPE, not that a clash is refused. validate_world_buildings
       used to carry its own copy of the rule, scoped per body, so two sites sharing an id
       on two different worlds were legal there and illegal at parse. Nothing reaches
       validate from load_system without passing parse first, so the extra term could only
       fire on a hand-built list -- which is where it disagreed with buildings.h's "unique
       across the SYSTEM", and how a comment about ids ended up wrong twice. */
    {
        System sys;
        if(load_shipped("res/systems/solar_system.json", sys)) {
            auto site = [](const char *id, const char *body) {
                WorldBuilding w; w.id = id; w.type = "runway"; w.body = body;
                return w;
            };
            sys.world_buildings = {site("baikonur", "Mars"), site("baikonur", "Venus")};
            CHECK(expect_throw([&]() { validate_world_buildings(sys, cat); }));
            sys.world_buildings = {site("baikonur", "Mars"), site("vostochny", "Venus")};
            bool clean = true;
            try { validate_world_buildings(sys, cat); }
            catch(const std::exception &) { clean = false; }
            CHECK(clean);
        }
    }

    // --- world_buildings_files: named fragments merge in ------------------
    {
        // The shipped Earth table reaches the solar systems in full, and the
        // table's own order survives: its first VAB_LAUNCH_PAD IS the game's
        // default site (find_launch_site's fallback).
        const std::string p = "res/systems/solar_system.json";
        std::vector<WorldBuilding> ws = parse_world_buildings_all(read_json(p.c_str()), p);
        CHECK(ws.size() >= 15);   // 14 Earth ranges + Venus's cloud platform
        CHECK(!ws.empty() && ws[0].id == "baikonur");
        // The table's own coordinates survive the parse (lon is east-positive,
        // so a west site is negative -- equirectDir takes either spelling).
        for(const WorldBuilding &w : ws) {
            if(w.id != "alcantara") { continue; }
            CHECK(w.lat_deg == -2.3394 && w.lon_deg == -44.4175);
            CHECK(vnear(w.dir(), equirectDir(-44.4175 * std::numbers::pi / 180.0,
                                             -2.3394 * std::numbers::pi / 180.0)));
        }
    }
    {
        /* Merge order, said directly instead of borrowed from shipped data: no GENERATED
           system carries an inline pad since #1063 deleted Kerbin's polar one, so the rule
           the polar pad used to demonstrate -- a fragment's pads come BEFORE the system's
           own "world_buildings", which is what keeps the table's first VAB_LAUNCH_PAD as
           the default site -- had no witness left. Build the shape rather than ship a pad
           for it. The inline pass is NOT dead code, though: res/systems/lag_playground.json
           and res/systems/wind_playground.json hand-author inline VAB_LAUNCH_PAD rows, so
           removing it would break those two playgrounds and neither this block nor the
           shipped-system loop above would notice. */
        nlohmann::json frag_only;
        frag_only["world_buildings_files"] = { "res/data/earth_launch_sites.json" };
        const size_t nfrag = parse_world_buildings_all(frag_only, "mem").size();
        nlohmann::json doc;
        doc["world_buildings_files"] = { "res/data/earth_launch_sites.json" };
        doc["world_buildings"] = { nlohmann::json{
            {"id", "inline_pad"}, {"type", "space_port"},
            {"display_name", "Inline pad"}, {"body", "Earth"},
            {"lat_deg", 10.0}, {"lon_deg", 10.0}, {"height_m", 5.0},
            {"flags", nlohmann::json::array({"VAB_LAUNCH_PAD"})},
        } };
        std::vector<WorldBuilding> ws = parse_world_buildings_all(doc, "mem");
        /* Positional rather than an absolute count: the inline row lands after EVERY
           fragment row, so this holds when someone adds a launch range to
           earth_launch_sites.json and reddens when the merge order flips. The front CHECK
           is the player-visible half -- the table, not an inline entry, owns the default
           site. */
        CHECK(ws.size() == nfrag + 1);
        CHECK(ws.front().id == "baikonur");  // the table still owns the default
        CHECK(ws[nfrag].id == "inline_pad");
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
        // missing or the array is empty: either way the system loses its whole
        // site list, since no shipped system has an inline pad to fall back on
        // any more (#1063).
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
        mk("second", {"VAB_LAUNCH_PAD"});
        CHECK(find_launch_site(&tb, "")->id == "main");        // default = first
        CHECK(find_launch_site(&tb, "second")->id == "second");  // any named pad
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
            g_failures++;
            continue;
        }
        CHECK(sys.world_buildings.size() >= 1);
        // Every solar_system variant names the same site file, so each must
        // carry the whole table: the >= 1 above and homePad below would both
        // pass on a single pad if a variant lost the reference.
        if(logical.find("solar_system") != std::string::npos) {
            CHECK(sys.world_buildings.size() >= 15);   // 14 Earth ranges + Venus's platform
        }
        // The fictional systems name a site fragment too (#226): 7 Kerbol sites
        // (pad, runway, polar pad, station, dock, Moho, Duna) and 1 Eerbon site.
        // Exact counts so a fragment that stops loading is a failure, not a smaller
        // pass. For THESE two systems the table is the whole list -- no shipped
        // system authors pads inline since #1063, and #1133 put Kerbin's polar pad
        // back INTO the table -- so the count is also the number of rows the pad
        // selector offers on that world. The two
        // hand-authored playgrounds (res/systems/lag_playground.json,
        // res/systems/wind_playground.json) still add an inline pad of their own and
        // are deliberately NOT in this loop: their counts would move with the tables.
        if(logical == "res/systems/ksp_system.json") {
            CHECK(sys.world_buildings.size() == 7);
        }
        if(logical == "res/systems/old_system.json") {
            CHECK(sys.world_buildings.size() == 1);
        }
        bool homePad = false;
        /* Pad decks the prose quotes (#983), and the accounting that makes the pin
           hard to lose. They are GENERATED -- terrain height at the pad's own dir
           plus its authored height_m -- so nothing else watches them, and "moho_pad
           stands 911 m" sat in comments while this build put its deck at 126 m. A
           noise or coordinate change now fails HERE with the new value printed just
           above it. The seen[] half is the point of writing it this way: matching
           ids alone leaves a renamed or deleted pad pinning NOTHING, and the test
           still says OK (proven: renaming moho_pad to moho_site in a scratch copy
           of the data left this test green). polar_pad is in here at 50.1 m -- the
           dry deck #1007 moved it to from under Kerbin's sea, and the deck #1133
           brought the pad back onto. #1063 deleted the pad and took this pin with
           it, which is the right behaviour for an id no system has and the wrong
           one for a pad that comes back: the number has to be re-measured, not
           remembered, so a terrain change under it during those two weeks would
           fail here rather than quietly inherit a stale figure. */
        struct PadWant { const char *id; double asl; };
        static const PadWant kWant[] = {
            {"main_space_port", 260.1},
            {"ksc_runway", 249.6},
            {"polar_pad", 50.1},
            {"moho_pad", 125.9},
            {"duna_pad", 65.0},
        };
        const bool pinHere = logical == "res/systems/ksp_system.json";
        bool seen[sizeof(kWant) / sizeof(kWant[0])] = {};
        for(const WorldBuilding &w : sys.world_buildings) {
            TerrainBody *wb = sys.find(w.body);
            if(wb == nullptr) {
                printf("FAIL %s: '%s' names unknown body '%s'\n",
                       logical.c_str(), w.id.c_str(), w.body.c_str());
                g_failures++;
            }
            if(sys.home != nullptr && w.body == sys.home->name
               && w.hasFlag("VAB_LAUNCH_PAD")) {
                homePad = true;
            }
            // type must resolve against the real catalog
            if(cat.find(w.type) == nullptr) {
                printf("FAIL %s: '%s' has unknown type '%s'\n",
                       logical.c_str(), w.id.c_str(), w.type.c_str());
                g_failures++;
            }
            /* The Surface Map projects a site as equirectLonLat(p->dir), the
               inverse of the equirectDir that built dir from these degrees.
               Pin the round-trip: a convention slip (lon sign, degrees vs
               radians, rotating vs non-rotating frame) would put a marker on
               the far side of the planet, and nothing else watches it. The
               half that bites is the wrap: Earth's west-negative sites are
               authored as negative lon and project back one turn later
               (alcantara -44.4175 -> 315.5825). The pole pad used to be the
               other interesting one -- cos(90 deg) is ~6e-17, not 0, so its
               longitude survived atan2 rather than landing on an arbitrary
               meridian -- and no shipped pad sits at a pole: #1007 moved
               the one that did off lat 90, #1063 took it out, and #1133 put
               polar_pad back at 88.8 deg rather than 90.
               Driven buildings (orbital, air) have no fixed surface lat/lon:
               skip the projection and the deck-grounding check for them -- an
               air entry carries lat/lon but floats 56 km up, not on the ground. */
            if(!w.orbital && !w.air) {
                double rlon, rlat;
                equirectLonLat(w.dir(), rlon, rlat);
                const double rlat_deg = rlat * 180.0 / std::numbers::pi;
                // lon unwraps to [0, 2pi), so a west-negative authoring is
                // the same meridian one turn away.
                double rlon_deg = rlon * 180.0 / std::numbers::pi
                                - w.lon_deg;
                if(rlon_deg > 180.0) { rlon_deg -= 360.0; }
                if(rlon_deg < -180.0) { rlon_deg += 360.0; }
                if(!std::isfinite(rlat_deg) || !std::isfinite(rlon_deg)
                   || std::fabs(rlat_deg - w.lat_deg) > 1e-9
                   || std::fabs(rlon_deg) > 1e-9) {
                    printf("FAIL %s: pad '%s' projects to lat %.10f lon "
                           "%.10f but is authored %.10f/%.10f\n",
                           logical.c_str(), w.id.c_str(), rlat_deg,
                           rlon * 180.0 / std::numbers::pi,
                           w.lat_deg, w.lon_deg);
                    g_failures++;
                }
            }
            // ... and the deck it puts on that body reaches the ground
            if(!w.orbital && !w.air && wb != nullptr) {
                check_pad_grounding(logical, w, wb);
                /* The number, printed where it earns its keep: the pinned pads, so
                   a stale one sits on the line above its own failure, and any deck
                   under its body's sea level -- the state #1007 moved three shipped
                   pads out of, and a permanent line beats a one-off audit if it ever
                   comes back. All 63 of them every run is noise the suite does not
                   need (#1000 was about exactly this). */
                const double asl = padDeckAsl(w, wb);
                bool pinned = false;
                for(size_t k = 0; k < sizeof(kWant) / sizeof(kWant[0]); k++) {
                    if(pinHere && w.id == kWant[k].id) {
                        seen[k] = true;
                        pinned = true;
                    }
                }
                if(pinned || asl < 0.0) {
                    printf("[padalt] %-28s %-8s %-18s deck_asl=%.1f%s\n",
                           logical.c_str(), w.body.c_str(), w.id.c_str(), asl,
                           asl < 0.0 ? "  BELOW SEA LEVEL" : "");
                }
                /* And the refusing half. A deck under its own sea is not a physics
                   problem -- no term reads the ocean, so the hull sits on the sea
                   floor and works -- it is the player standing a launch pad where
                   they can see water over it, and landing "back at the pad" into a
                   sea with no splash. So a site the VAB offers must clear the water
                   line. Scoped to VAB pads on a body that HAS a sea: a scenic
                   building under water is a designer's business, and orbital and air
                   entries never reach here. check_pad_grounding cannot see this --
                   it weighs the deck against the terrain under the footprint, which
                   a pad is built to sit on, submerged or not. */
                if(asl < 0.0 && wb->surface.has_sea && w.hasFlag("VAB_LAUNCH_PAD")) {
                    printf("FAIL %s: pad '%s' deck sits %.1f m BELOW %s's sea level "
                           "-- a hull on it stands under the ocean shell; move it to "
                           "dry ground (#1007)\n",
                           logical.c_str(), w.id.c_str(), -asl, w.body.c_str());
                    g_failures++;
                }
                if(!pinHere) { continue; }
                for(const auto &e : kWant) {
                    if(w.id != e.id) { continue; }
                    if(std::fabs(asl - e.asl) > 2.0) {
                        printf("FAIL %s: pad '%s' deck is at %.1f m above "
                               "the datum, not the %.1f m the prose quotes "
                               "-- fix the comment or re-measure the pin\n",
                               logical.c_str(), w.id.c_str(), asl, e.asl);
                        g_failures++;
                    }
                }
            }
        }
        /* Every pinned id has to have been SEEN, or the pin above compared nothing.
           The same rule covers the discriminator itself: the want list is keyed to
           this one file because main_space_port also exists on Eerbon (at 26.0 m,
           not the -166.5 m #1007 found it at), so without it the test would fail
           for the wrong reason -- and if the file
           were renamed, both this pin and the ksp-specific counts above would go
           away silently. */
        if(pinHere) {
            for(size_t k = 0; k < sizeof(kWant) / sizeof(kWant[0]); k++) {
                if(!seen[k]) {
                    printf("FAIL %s: no pad '%s' to pin -- renamed, removed, or no "
                           "longer a surface building, so the pin and the prose it "
                           "protects silently vanished\n",
                           logical.c_str(), kWant[k].id);
                    g_failures++;
                }
            }
        }
        /* A body's DEFAULT site is the first VAB_LAUNCH_PAD ON THAT BODY
           (find_launch_site filters by body, and so does the VAB dropdown),
           with fragments merged before a system's inline pads -- so on the
           home world this is exactly what the "(default)" entry and a bare
           LAUNCH mean. Nothing ships an inline pad since #1063, so the merge
           order that keeps an inline pad out of this race is pinned on its own
           above, with a synthetic list. */
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
            if(logical.find("solar_system") != std::string::npos) {
                CHECK(dflt->id == "baikonur");
            } else if(logical == "res/systems/ksp_system.json"
                      || logical == "res/systems/old_system.json") {
                CHECK(dflt->id == "main_space_port");
                /* The fictional systems' tables are hand-authored data no
                   generator writes, so nothing else watches their numbers:
                   pin the home pad's spot, as alcantara does for Earth's.
                   Eerbon's degrees are NOT Kerbin's copied over: the same spot
                   on Eerbon's own noise put its deck 166 m under that world's
                   sea, so it moved to the nearest dry flat ground (#1007). */
                if(logical == "res/systems/ksp_system.json") {
                    CHECK(dflt->lat_deg == 0.286473 && dflt->lon_deg == 270.286477);
                } else {
                    CHECK(dflt->lat_deg == -1.9135 && dflt->lon_deg == 267.8);
                }
            }
            /* A demo file names its own pad. res/systems is also what the in-game
               picker lists, so it can hold a world invented to show a mechanism off
               (wind_playground.json, #767/#775); pinning a catalogue pad's id and
               degrees there would be a rule about data nobody ships. What still
               holds for ANY listable system is above: it loads, the default site is
               on home, is a VAB pad, and projects back to the
               degrees it was authored with. */
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
                // Driven buildings have no fixed surface dir; their separation
                // is an orbit/drift question, not a deck-overlap one.
                if(a.orbital || b.orbital || a.air || b.air) { continue; }
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
                    g_failures++;
                }
            }
        }
        /* A building's label is its transfer-target identity (#883): the Target
           window and --transfer-target match {label, kind}, and that identity has
           no body term, so two buildings in one system answering to one label are
           ONE target -- the combo lists two rows and clicking either resolves the
           first, and once the hull crosses an SoI a same-named station around the
           NEW body inherits the selection. Placement renames a collision the way
           spawn renames a ship; this pins that no SHIPPED world needs the rename,
           so what a player reads is what the author typed. Exact-string, because
           that is what the planner compares: a near-duplicate ("Kerbin Station"
           beside "Kerbin station") is a readable ambiguity in the combo rather
           than a wrong target, so it is not failed here. */
        {
            std::vector<std::string> taken;
            for(const WorldBuilding &w : sys.world_buildings) {
                const std::string base = w.label(cat);
                if(dedupBuildingName(taken, base) != base) {
                    printf("FAIL %s: '%s' and another building in this system "
                           "both answer to \"%s\"\n",
                           logical.c_str(), w.id.c_str(), base.c_str());
                    g_failures++;
                }
            }
        }
        if(!homePad) {
            printf("FAIL %s: no VAB_LAUNCH_PAD on home '%s'\n",
                   logical.c_str(), sys.home ? sys.home->name.c_str() : "(none)");
            g_failures++;
        }
    }

    // --- dedupBuildingName ------------------------------------------------
    /* The rule placement applies (#883), spelled out because it is a RENAME of
       authored data and so has to stay exactly the rule ship names already use
       (vehicle.cpp:dedupVehicleName): the first ask gets the base name, later
       ones " #2", " #3". What it deliberately does NOT do is parse a suffix it
       put there itself: asking for "Science outpost #2" when that is taken
       answers " #2 #2", just as a ship named "X #2" renames to "X #2 #2".
       Consistency with the ship rule is worth more here than tidiness. The list
       is flat across the whole system because TargetRef has no body term: a name
       used around Kerbin is taken around Mun too. */
    {
        std::vector<std::string> taken;
        CHECK(dedupBuildingName(taken, "Science outpost") == "Science outpost");
        CHECK(dedupBuildingName(taken, "Science outpost") == "Science outpost #2");
        CHECK(dedupBuildingName(taken, "Science outpost") == "Science outpost #3");
        CHECK(dedupBuildingName(taken, "Other outpost") == "Other outpost");
        // It records what it handed out: a caller that has to remember to do
        // that itself can forget, and the collision is back with no signal.
        CHECK(taken.size() == 4);
        CHECK(dedupBuildingName(taken, "Science outpost #2")
              == "Science outpost #2 #2");
        CHECK(dedupBuildingName(taken, "Science outpost #2")
              == "Science outpost #2 #3");
    }

    if(g_failures) {
        printf("test_buildings: %d FAILED\n", g_failures);
        return 1;
    }
    printf("test_buildings: OK\n");
    return 0;
}
