// test_shipload: the GL-free ship/part JSON data model (src/shipdef.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_shipload)
//
// Covers: catalog + ship-def parsing, part resolution, the aggregates the
// Vehicle would compute from them (mass, full thrust, wheel torque, fuel),
// the default-controller rule, and the error paths (bad file, unknown part
// name, bad part type, out-of-range controller).

#include "shipdef.h"

#include <cmath>
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

int main() {
    // --- parts catalog ----------------------------------------------------
    PartsCatalog cat = load_parts_catalog("res/parts.json");
    CHECK(cat.parts.size() == 3);
    CHECK(cat.find("nope") == nullptr);

    const PartDef *cap = cat.find("capsule");
    const PartDef *rw  = cat.find("reaction_wheel");
    const PartDef *eng = cat.find("engine");
    CHECK(cap != nullptr && rw != nullptr && eng != nullptr);

    CHECK(cap->type == VesselPartType::Capsule);
    CHECK(cap->mass == 500.0);
    CHECK(cap->mesh == "capsule.obj");
    CHECK(cap->texture == "capsule.png");

    CHECK(rw->type == VesselPartType::ReactionWheel);
    CHECK(rw->mass == 1000.0);
    CHECK(rw->torque == 2000.0);

    CHECK(eng->type == VesselPartType::Engine);
    CHECK(eng->mass == 3000.0);
    CHECK(eng->fuel_rate == 1.0);
    CHECK(eng->exhaust_velocity == 40492.0);
    CHECK(eng->capacity[(int)ResourceType::Hydrogen] == 1000.0f);
    CHECK(eng->capacity[(int)ResourceType::LOX] == 1000.0f);
    CHECK(eng->fullThrust() == 80984.0); // 2 * 1.0 * 40492

    // --- ship def -----------------------------------------------------------
    ShipDef def = load_ship_def("res/ships/basic.json", cat);
    CHECK(def.parts.size() == 3);
    CHECK(def.parts[0].part == "capsule");
    CHECK(def.parts[0].def == cap);
    CHECK(def.parts[1].def == rw);
    CHECK(def.parts[2].def == eng);
    CHECK(def.parts[0].offset == 10.5);
    CHECK(def.parts[1].offset == 8.5);
    CHECK(def.parts[2].offset == 6.5);
    // no "controller" in the file -> defaults to the last part (the engine)
    CHECK(def.controllerIndex() == 2);

    // the aggregates the Vehicle derives from the same data
    double mass = 0, thrust = 0, torque = 0, fuel = 0;
    for(size_t i = 0; i < def.parts.size(); i++) {
        const PartDef *d = def.parts[i].def;
        mass += d->mass;
        if(d->type == VesselPartType::Engine) {
            thrust += d->fullThrust();
            fuel += d->capacity[(int)ResourceType::Hydrogen];
            fuel += d->capacity[(int)ResourceType::LOX];
        }
        if(d->type == VesselPartType::ReactionWheel) { torque += d->torque; }
    }
    CHECK(mass == 4500.0);
    CHECK(thrust == 80984.0);
    CHECK(torque == 2000.0);
    CHECK(fuel == 2000.0);

    // --- error paths ---------------------------------------------------------
    CHECK(expect_throw([](){ load_parts_catalog("res/no_such_file.json"); }));
    CHECK(expect_throw([&](){ load_ship_def("res/no_such_file.json", cat); }));

    // unknown part name in a ship def
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"part\": \"warp_drive\", \"offset\": 1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // out-of-range controller index
    {
        const char *bad = "/tmp/test_shipload_bad.json";
        std::ofstream f(bad);
        f << "{ \"controller\": 5, \"parts\": [ { \"part\": \"capsule\", \"offset\": 1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_ship_def(bad, cat); }));
        std::remove(bad);
    }

    // bad part type in a catalog
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"warp_drive\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    if(failures) {
        printf("test_shipload: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_shipload: all tests passed\n");
    return 0;
}
