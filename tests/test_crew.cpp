// test_crew: the crew_capacity field on the part catalog (src/shipdef.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_crew)
//
// A capsule is a part that can hold EVA characters aboard (KSP-style crew
// seats): crew_capacity > 0 means it is one, 0 means it is not. The value
// caps how many characters it can carry. This is the data-model half of the
// crew integration; the aboard-state bookkeeping (Kerbal::aboard) is exercised
// by the e2e EVA/Board cases.

#include "shipdef.h"

#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>

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
    PartsCatalog cat = load_parts_catalog("res/parts.json");

    // the capsule family: crew_capacity set (1, 3, 6)
    const PartDef *cap  = cat.find("capsule");
    const PartDef *cap3 = cat.find("capsule_r3h6");
    const PartDef *cap6 = cat.find("capsule_r5h10");
    CHECK(cap != nullptr && cap3 != nullptr && cap6 != nullptr);
    CHECK(cap->crew_capacity == 1);
    CHECK(cap3->crew_capacity == 3);
    CHECK(cap6->crew_capacity == 6);

    // every other part is not a capsule (crew_capacity 0 = the default)
    const char *nonCapsules[] = { "engine", "fuel_tank", "reaction_wheel",
                                  "kerbal", "nose_cap", "tank_r3h2" };
    for(const char *n : nonCapsules) {
        const PartDef *d = cat.find(n);
        CHECK(d != nullptr);
        if(d == nullptr) { continue; }
        CHECK(d->crew_capacity == 0);
    }

    // an explicit 0 parses (and a part that omits it defaults to 0)
    {
        const char *f = "/tmp/test_crew_cap0.json";
        std::ofstream o(f);
        o << "{ \"parts\": [ { \"name\": \"a\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"crew_capacity\": 0 }, "
             " { \"name\": \"b\", \"type\": \"engine\", "
             "\"mesh\": \"b.obj\", \"texture\": \"b.png\", \"mass\": 1.0 } ] }";
        o.close();
        PartsCatalog c = load_parts_catalog(f);
        CHECK(c.find("a")->crew_capacity == 0);
        CHECK(c.find("b")->crew_capacity == 0);
        std::remove(f);
    }

    // a negative capacity is rejected
    {
        const char *f = "/tmp/test_crew_bad.json";
        std::ofstream o(f);
        o << "{ \"parts\": [ { \"name\": \"x\", \"type\": \"engine\", "
             "\"mesh\": \"a.obj\", \"texture\": \"a.png\", \"mass\": 1.0, "
             "\"crew_capacity\": -1 } ] }";
        o.close();
        CHECK(expect_throw([&](){ load_parts_catalog(f); }));
        std::remove(f);
    }

    if(failures == 0) {
        printf("test_crew: all checks passed\n");
        return 0;
    }
    printf("test_crew: %d FAILURE(S)\n", failures);
    return 1;
}
