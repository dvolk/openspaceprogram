// test_suitdef: the crew pool's suit-def gate (src/part.cpp).
// Runs from the repo root (needs res/):
//   make test   (or: ./build/linux-v2-znver3/tests/test_suitdef)
//
// Covers: load_suit_def's two refusals -- a def that is not exactly one part,
// and a one-part def that is not kerbal-typed -- and that the shipped suit def
// passes the gate.
//
// Why this is pinned at all: mintCrewPool used to call make_suit once per
// roster entry, so the def was loaded and validated eight times at boot. #291
// split that into load_suit_def (once) + make_suit (per person), which moved the
// two refusals onto a path nothing tested. They are player-visible -- the crew
// pool prints "Crew: <what>" and mints nobody -- and make_suit now only asserts
// the contract, so a regression here would silently mint half-people.
//
// make_suit's MINT is not exercised here: it goes through create_part_body,
// which builds a hull and captures its silhouette verts, and no unit test in
// this suite does that (they hand-build Parts). The e2e crew cases cover the
// mint, and this test stays GL-free.

#include "part.h"

#include <cstdio>
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

/* Matching the message, not just the throw: a def can be refused by a
   neighbouring rule for an unrelated reason and the test would still pass. */
static bool refused_saying(const std::function<void()> &fn, const std::string &needle) {
    try {
        fn();
    } catch(const std::runtime_error &e) {
        if(std::string(e.what()).find(needle) == std::string::npos) {
            printf("   refused, but not for the expected reason: %s\n", e.what());
            return false;
        }
        return true;
    }
    printf("   expected a refusal, got none\n");
    return false;
}

int main() {
    PartsCatalog cat = load_parts_catalog("res/data/parts.json");

    /* The shipped suit def: exactly one kerbal-typed part. */
    ShipDef suit;
    try {
        suit = load_suit_def(kKerbalShipDef, cat);
    } catch(const std::exception &e) {
        printf("FAIL load_suit_def(%s) threw: %s\n", kKerbalShipDef, e.what());
        return 1;
    }
    CHECK(suit.parts.size() == 1);
    CHECK(suit.parts[0].def != nullptr);
    CHECK(suit.parts[0].def->type == "kerbal");

    /* SIZE half: a multi-part def cannot be a person. The save writer reads a
       walker's part 0 only, so the rest would be written by no one. */
    CHECK(refused_saying([&] { load_suit_def("res/ships/racer.json", cat); },
                         "is not exactly one part"));

    /* TYPE half: one part, but a command pod rather than a suit. */
    CHECK(refused_saying([&] { load_suit_def("e2e/vehicles/one_capsule.json", cat); },
                         "not a kerbal suit"));

    if(failures) {
        printf("test_suitdef: %d FAILURES\n", failures);
        return 1;
    }
    printf("test_suitdef: OK\n");
    return 0;
}
