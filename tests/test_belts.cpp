// test_belts.cpp -- the system JSON root "belts" array (src/system.cpp).
//
// Debris belts are named annuli orbiting the STAR, drawn on the orbital maps
// (gameui.cpp drawDebrisBelts). The entry shape is deliberately the same as a
// body's "surface.rings" band (name/inner/outer, metres): a belt is that
// object with a different centre, so there is one annulus vocabulary to learn.
// Unlike rings, a malformed belt THROWS rather than being skipped: belts are
// few and hand-authored, and a silently dropped band is invisible in-game.
// Pinned here:
//   - the shipped showcase systems carry both bands, in the right order,
//     with sane radii that reach the loader,
//   - "belts" is optional: a system without it loads with none,
//   - malformed belts throw, naming the belt.
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_belts).

#include "system.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        std::printf("FAIL cannot open %s (run from the repo root)\n", path);
        ++g_failures;
        return nlohmann::json::object();
    }
    return nlohmann::json::parse(f);
}

// Load `path`, expecting success; empty vector on a throw (the caller's
// checks then fail, naming the case).
static std::vector<BeltParams> load_belts(const std::string &path) {
    try {
        System sys = load_system(path.c_str(), nullptr, nullptr);
        return sys.belts;
    } catch(const std::exception &e) {
        std::printf("FAIL %s: unexpected throw: %s\n", path.c_str(), e.what());
        ++g_failures;
        return {};
    }
}

// Write a mutated system (tmp/), load it, expect a throw naming msg_needle.
static void expect_reject(const nlohmann::json &j, const char *tag,
                          const char *msg_needle) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_belts_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    const char *what = nullptr;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        what = e.what();
    }
    check(what && std::strstr(what, msg_needle), tag);
    std::remove(tmp.c_str());
}

// A two-band array in the authored form.
static nlohmann::json two_bands() {
    return nlohmann::json::array({
        { { "name", "asteroid belt" }, { "inner", 2.856e10 }, { "outer", 4.488e10 } },
        { { "name", "Kuiper belt" },  { "inner", 4.08e11 },  { "outer", 6.8e11 } },
    });
}

int main() {
    // --- the shipped systems carry both bands, in draw order --------------
    for(const std::string &slug : { std::string("ksp_system"),
                                    std::string("solar_system"),
                                    std::string("solar_system_measured"),
                                    std::string("solar_system_named"),
                                    std::string("solar_system_full") }) {
        const std::string path = "res/systems/" + slug + ".json";
        const std::vector<BeltParams> belts = load_belts(path);
        check(belts.size() == 2, ("two belts: " + slug).c_str());
        if(belts.size() != 2) { continue; }
        check(belts[0].name == "asteroid belt", ("inner band named: " + slug).c_str());
        check(belts[1].name == "Kuiper belt", ("outer band named: " + slug).c_str());
        // Ordered outward, and both outside the star: a band that swallowed
        // the star or sat inside its own sun would draw as a filled disc.
        for(const BeltParams &b : belts) {
            check(b.inner > 0.0 && b.outer > b.inner,
                  ("0 < inner < outer: " + slug + " " + b.name).c_str());
        }
        // The asteroid band must sit inside the Kuiper band, or the two
        // overlap and the map shows one smeared region.
        check(belts[0].outer < belts[1].inner,
              ("bands do not overlap: " + slug).c_str());
    }

    // --- "belts" is optional ----------------------------------------------
    // old_system is the frozen legacy fixture: no belts, and it must still
    // load (the map simply draws no bands).
    {
        const std::vector<BeltParams> belts = load_belts("res/systems/old_system.json");
        check(belts.empty(), "system without \"belts\" loads with none");
    }

    // --- the authored form reaches the loader ------------------------------
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = two_bands();
        const std::string tmp = "tmp/test_belts_authored.json";
        std::ofstream(tmp) << j.dump();
        const std::vector<BeltParams> belts = load_belts(tmp);
        check(belts.size() == 2, "authored belts load");
        if(belts.size() == 2) {
            check(belts[0].name == "asteroid belt" && belts[0].inner == 2.856e10
                  && belts[0].outer == 4.488e10, "belt 0 fields survive");
            check(belts[1].name == "Kuiper belt" && belts[1].inner == 4.08e11
                  && belts[1].outer == 6.8e11, "belt 1 fields survive");
        }
        std::remove(tmp.c_str());
    }

    // --- malformed data throws --------------------------------------------
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = two_bands()[0];   // one object, not an array
        expect_reject(j, "non-array belts rejected", "must be an array");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array({ { { "inner", 1.0e10 },
                                               { "outer", 2.0e10 } } });
        expect_reject(j, "nameless belt rejected", "non-empty \"name\"");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "asteroid belt" }, { "inner", "28560000000" },
                { "outer", 4.488e10 } } });
        expect_reject(j, "string radius rejected", "numeric \"inner\"");
    }
    {   // the r1/r2 spelling: a plausible typo, and it must not load as a
        // zero-radius band
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "asteroid belt" }, { "r1", 2.856e10 },
                { "r2", 4.488e10 } } });
        expect_reject(j, "r1-r2 spelling rejected", "numeric \"inner\"");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "inside-out belt" }, { "inner", 4.488e10 },
                { "outer", 2.856e10 } } });
        expect_reject(j, "inverted radii rejected", "inside-out belt");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "zero belt" }, { "inner", 0.0 }, { "outer", 4.488e10 } } });
        expect_reject(j, "zero inner rejected", "zero belt");
    }

    if(g_failures == 0) { std::printf("test_belts: OK\n"); return 0; }
    std::printf("test_belts: %d FAILURES\n", g_failures);
    return 1;
}
