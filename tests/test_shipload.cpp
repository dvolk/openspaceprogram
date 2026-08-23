// test_shipload: the GL-free ship/part JSON data model (src/shipdef.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_shipload)
//
// Covers: catalog + ship-def parsing, part resolution, the aggregates the
// Vehicle would compute from them (mass, full thrust, wheel torque, fuel),
// the default-controller rule, and the error paths (bad file, unknown part
// name, out-of-range controller, unpaired thruster fields, empty capacity).

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
    CHECK(cat.parts.size() == 4);
    CHECK(cat.find("nope") == nullptr);

    const PartDef *cap = cat.find("capsule");
    const PartDef *rw  = cat.find("reaction_wheel");
    const PartDef *eng = cat.find("engine");
    const PartDef *ft  = cat.find("fuel_tank");
    CHECK(cap != nullptr && rw != nullptr && eng != nullptr && ft != nullptr);

    // type is a free-form label; behavior comes from the fields
    CHECK(cap->type == "capsule");
    CHECK(cap->mass == 500.0);
    CHECK(cap->mesh == "capsule.obj");
    CHECK(cap->texture == "capsule.png");
    CHECK(cap->torque == 200.0);   // the capsule carries a small reaction wheel
    CHECK(cap->fuel_rate == 0.0 && cap->exhaust_velocity == 0.0);

    CHECK(rw->type == "reaction_wheel");
    CHECK(rw->mass == 1000.0);
    CHECK(rw->torque == 2000.0);

    // engine is the pump: thrust params, but no propellant of its own
    CHECK(eng->type == "engine");
    CHECK(eng->mass == 1000.0);
    CHECK(eng->fuel_rate == 1.0);
    CHECK(eng->exhaust_velocity == 40492.0);
    CHECK(eng->torque == 0.0);
    CHECK(eng->capacity[(int)ResourceType::Hydrogen] == 0.0f); // fuel moved to the tank
    CHECK(eng->capacity[(int)ResourceType::LOX] == 0.0f);
    CHECK(eng->fullThrust() == 80984.0); // 2 * 1.0 * 40492

    // fuel tank is the reservoir: holds the propellant, no thrust params
    CHECK(ft->type == "fuel_tank");
    CHECK(ft->mass == 2000.0);
    CHECK(ft->mesh == "reaction_wheel.obj");   // looks the same as the reaction wheel
    CHECK(ft->texture == "reaction_wheel.png");
    CHECK(ft->torque == 0.0);
    CHECK(ft->fuel_rate == 0.0 && ft->exhaust_velocity == 0.0);
    CHECK(ft->capacity[(int)ResourceType::Hydrogen] == 1000.0f);
    CHECK(ft->capacity[(int)ResourceType::LOX] == 1000.0f);

    // --- ship def -----------------------------------------------------------
    ShipDef def = load_ship_def("res/ships/basic.json", cat);
    CHECK(def.parts.size() == 4);
    CHECK(def.parts[0].part == "capsule");
    CHECK(def.parts[0].def == cap);
    CHECK(def.parts[1].def == rw);
    CHECK(def.parts[2].def == ft);   // the fuel tank
    CHECK(def.parts[3].def == eng);
    CHECK(def.parts[0].offset == 12.5);
    CHECK(def.parts[1].offset == 10.5);
    CHECK(def.parts[2].offset == 8.5);
    CHECK(def.parts[3].offset == 6.5);
    // no "controller" in the file -> defaults to the last part (the engine)
    CHECK(def.controllerIndex() == 3);

    // the aggregates the Vehicle derives from the same data (field-driven,
    // mirroring Vehicle::init)
    double mass = 0, thrust = 0, torque = 0, fuel = 0;
    for(size_t i = 0; i < def.parts.size(); i++) {
        const PartDef *d = def.parts[i].def;
        mass += d->mass;
        if(d->fuel_rate > 0.0 && d->exhaust_velocity > 0.0) { thrust += d->fullThrust(); }
        if(d->torque > 0.0) { torque += d->torque; }
        for(size_t r = 0; r < d->capacity.size(); r++) { fuel += d->capacity[r]; }
    }
    CHECK(mass == 4500.0);   // 500 + 1000 + 1000 (dry engine) + 2000 (tank)
    CHECK(thrust == 80984.0);
    CHECK(torque == 2200.0); // capsule 200 (small wheel) + reaction_wheel 2000
    CHECK(fuel == 2000.0);

    // the explorer: same as basic but with two fuel tanks (double the fuel)
    ShipDef ex = load_ship_def("res/ships/explorer.json", cat);
    CHECK(ex.parts.size() == 5);
    {
        int tanks = 0;
        double ex_mass = 0, ex_fuel = 0, ex_torque = 0;
        for(size_t i = 0; i < ex.parts.size(); i++) {
            const PartDef *d = ex.parts[i].def;
            ex_mass += d->mass;
            if(d->torque > 0.0) { ex_torque += d->torque; }
            bool has_cap = false;
            for(size_t r = 0; r < d->capacity.size(); r++) {
                ex_fuel += d->capacity[r];
                if(d->capacity[r] > 0.0f) { has_cap = true; }
            }
            if(has_cap) { tanks++; }
        }
        CHECK(tanks == 2);
        CHECK(ex_mass == 6500.0);   // 500 + 1000 + 1000 + 2*2000
        CHECK(ex_fuel == 4000.0);
        CHECK(ex_torque == 2200.0); // same capsule + wheel as basic
    }

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

    // thruster fields must be given together (fuel_rate without exhaust_velocity)
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"fuel_rate\": 1.0 } ] }";
        f.close();
        CHECK(expect_throw([&](){ load_parts_catalog(bad); }));
        std::remove(bad);
    }

    // a capacity must total > 0
    {
        const char *bad = "/tmp/test_shipload_badcat.json";
        std::ofstream f(bad);
        f << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"fuel_tank\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"capacity\": { \"hydrogen\": 0, \"lox\": 0 } } ] }";
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
