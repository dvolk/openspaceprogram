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
    CHECK(f.ships.size() == 3);

    CHECK(f.ships[0].ship == "res/ships/basic.json");
    CHECK(f.ships[0].name == "Alpha");
    CHECK(f.ships[0].body.empty());
    CHECK(f.ships[0].scenario.empty());

    CHECK(f.ships[1].name == "Bravo");
    CHECK(f.ships[1].scenario == "rot-orbit");

    CHECK(f.ships[2].name == "Charlie");
    CHECK(f.ships[2].scenario == "ellipse-peri");

    // --- defaults -----------------------------------------------------------
    {
        const char *p = "/tmp/test_fleet_defaults.json";
        write_file(p, "{ \"ships\": [ { \"name\": \"Solo\" } ] }");
        Fleet g = load_fleet(p);
        CHECK(g.ships.size() == 1);
        CHECK(g.ships[0].ship == "res/ships/basic.json");
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
