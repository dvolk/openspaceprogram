// test_flightlog: the per-vessel flight journal (src/flightlog.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_flightlog.cpp -o test_flightlog && ./test_flightlog)
//
// Pure logic: first observe begins the journal + logs the initial enter;
// a body change emits left/entered; repeats are free; empty sides are
// allowed (a ship with no SoI body).
#include "flightlog.h"

#include <cstdio>
#include <string>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

int main() {
    // --- begin: first observe seeds start + an "entered" for the body ---
    {
        FlightLog log;
        CHECK(!log.started);
        log.observe(10.0, "Kerbin");
        CHECK(log.started);
        CHECK(log.start_t == 10.0);
        CHECK(log.start_body == "Kerbin");
        CHECK(log.last_body == "Kerbin");
        CHECK(log.events.size() == 1);
        CHECK(log.events[0].t == 10.0);
        CHECK(log.events[0].enter);
        CHECK(log.events[0].body == "Kerbin");
    }

    // --- begin with no SoI body: no initial event ---
    {
        FlightLog log;
        log.observe(0.0, "");
        CHECK(log.started);
        CHECK(log.events.empty());
        log.observe(1.0, "");
        CHECK(log.events.empty());
    }

    // --- repeat observes of the same body are free ---
    {
        FlightLog log;
        log.observe(0.0, "Kerbin");
        log.observe(1.0, "Kerbin");
        log.observe(2.0, "Kerbin");
        CHECK(log.events.size() == 1);
    }

    // --- SoI change A -> B: left A then entered B at the same stamp ---
    {
        FlightLog log;
        log.observe(0.0, "Kerbin");
        log.observe(100.0, "Mun");
        CHECK(log.events.size() == 3);
        CHECK(log.events[1].t == 100.0);
        CHECK(!log.events[1].enter);
        CHECK(log.events[1].body == "Kerbin");
        CHECK(log.events[2].t == 100.0);
        CHECK(log.events[2].enter);
        CHECK(log.events[2].body == "Mun");
        CHECK(log.last_body == "Mun");
    }

    // --- leave to empty (no SoI): just "left" ---
    {
        FlightLog log;
        log.observe(0.0, "Kerbin");
        log.observe(50.0, "");
        CHECK(log.events.size() == 2);
        CHECK(!log.events[1].enter);
        CHECK(log.events[1].body == "Kerbin");
        CHECK(log.last_body.empty());
    }

    // --- enter from empty: just "entered" ---
    {
        FlightLog log;
        log.observe(0.0, "");
        log.observe(50.0, "Kerbol");
        CHECK(log.events.size() == 1);
        CHECK(log.events[0].enter);
        CHECK(log.events[0].body == "Kerbol");
    }

    // --- t_begin back-dates the first stamp (high-warp first step) ---
    {
        FlightLog log;
        log.observe(200.0, "Kerbin", 0.0);
        CHECK(log.start_t == 0.0);
        CHECK(log.events.size() == 1);
        CHECK(log.events[0].t == 0.0);
    }

    // --- full tour: home -> moon -> home ---
    {
        FlightLog log;
        log.observe(0.0, "Kerbin");
        log.observe(10.0, "Mun");
        log.observe(20.0, "Kerbin");
        CHECK(log.events.size() == 5);
        CHECK(log.events[0].enter && log.events[0].body == "Kerbin");
        CHECK(!log.events[1].enter && log.events[1].body == "Kerbin");
        CHECK(log.events[2].enter && log.events[2].body == "Mun");
        CHECK(!log.events[3].enter && log.events[3].body == "Mun");
        CHECK(log.events[4].enter && log.events[4].body == "Kerbin");
    }

    if(failures == 0) {
        printf("test_flightlog: all checks passed\n");
        return 0;
    }
    printf("test_flightlog: %d FAILURE(S)\n", failures);
    return 1;
}
