// test_fleet: the fleet JSON parse path (src/fleet.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_fleet)
//
// Covers: res/fleet.json parses with the expected entries, field defaults,
// and the error paths (bad file, missing/empty "ships", non-object entry,
// bad JSON).

#include "fleet.h"

#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

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

static void write_file(const char *path, const std::string &content) {
    std::ofstream f(path);
    f << content;
    f.close();
}

int main() {
    // --- the shipped sample -------------------------------------------------
    Fleet f = load_fleet("res/fleet.json");
    CHECK(f.ships.size() == 5);

    CHECK(f.ships[0].ship == "res/ships/racer.json");
    CHECK(f.ships[0].name == "racer");
    CHECK(f.ships[0].body.empty());
    CHECK(f.ships[0].scenario == "pad");

    CHECK(f.ships[1].ship == "res/ships/tanker.json");
    CHECK(f.ships[1].name == "tanker");
    CHECK(f.ships[1].body.empty());
    CHECK(f.ships[1].scenario == "pad");

    CHECK(f.ships[2].ship == "res/ships/stager.json");
    CHECK(f.ships[2].name == "stager");
    CHECK(f.ships[2].body.empty());
    CHECK(f.ships[2].scenario == "pad");

    CHECK(f.ships[3].ship == "res/ships/laythe_explorer.json");
    CHECK(f.ships[3].name == "laythe explorer");
    CHECK(f.ships[3].body.empty());
    CHECK(f.ships[3].scenario == "inertial-orbit");

    CHECK(f.ships[4].ship == "res/ships/transporter.json");
    CHECK(f.ships[4].name == "transporter");
    CHECK(f.ships[4].body.empty());
    CHECK(f.ships[4].scenario == "high-orbit");

    // --- defaults -----------------------------------------------------------
    {
        const char *p = "/tmp/test_fleet_defaults.json";
        write_file(p, "{ \"ships\": [ { \"name\": \"Solo\" } ] }");
        Fleet g = load_fleet(p);
        CHECK(g.ships.size() == 1);
        CHECK(g.ships[0].ship == "res/ships/racer.json");
        CHECK(g.ships[0].name == "Solo");
        CHECK(g.ships[0].body.empty());
        CHECK(g.ships[0].scenario.empty());
        std::remove(p);
    }

    // --- error paths ---------------------------------------------------------
    CHECK(expect_throw([](){ load_fleet("res/no_such_fleet.json"); }));

    // no "ships" key
    {
        const char *p = "/tmp/test_fleet_bad.json";
        write_file(p, "{ }");
        CHECK(expect_throw([&](){ load_fleet(p); }));
    }

    // empty "ships"
    {
        const char *p = "/tmp/test_fleet_bad.json";
        write_file(p, "{ \"ships\": [] }");
        CHECK(expect_throw([&](){ load_fleet(p); }));
    }

    // non-object entry
    {
        const char *p = "/tmp/test_fleet_bad.json";
        write_file(p, "{ \"ships\": [ 42 ] }");
        CHECK(expect_throw([&](){ load_fleet(p); }));
    }

    // malformed JSON
    {
        const char *p = "/tmp/test_fleet_bad.json";
        write_file(p, "{ \"ships\": [ { \"name\": \"x\" }");
        CHECK(expect_throw([&](){ load_fleet(p); }));
        std::remove(p);
    }

    if(failures) {
        printf("test_fleet: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_fleet: all tests passed\n");
    return 0;
}
