// test_fmt: the UI formatting helpers (src/fmt.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_fmt.cpp -o test_fmt && ./test_fmt)
//
// Pins the exact readout strings: the unit ladder boundaries
// (m / km / Mm / AU / ly), the sign handling, and the overflow-safe
// long long ToF (the old (int) cast was UB at oort/interstellar ToFs).
#include "fmt.h"

#include <cstdio>
#include <cstring>

static int failures = 0;
#define CHECK_DIST(meters, want) do { \
        char _b[64]; \
        const char *_g = fmt_dist(meters, _b, sizeof _b); \
        if(std::strcmp(_g, (want)) != 0) { \
            printf("FAIL %s:%d: fmt_dist(%s) = \"%s\", want \"%s\"\n", \
                   __FILE__, __LINE__, #meters, _g, (want)); \
            failures++; \
        } \
    } while(0)

#define CHECK_TIME(s, want) do { \
        char _b[64]; \
        const char *_g = fmt_time(s, _b, sizeof _b); \
        if(std::strcmp(_g, (want)) != 0) { \
            printf("FAIL %s:%d: fmt_time(%s) = \"%s\", want \"%s\"\n", \
                   __FILE__, __LINE__, #s, _g, (want)); \
            failures++; \
        } \
    } while(0)

int main() {
    // --- fmt_dist: the unit ladder ----------------------------------------
    CHECK_DIST(0.0,      "0.0 m");
    CHECK_DIST(-0.4,     "-0.4 m");      // sign kept (sub-meter)
    CHECK_DIST(999.9,    "999.9 m");     // top of the m tier
    CHECK_DIST(1000.0,   "1.0 km");      // bottom of the km tier
    CHECK_DIST(4.495e6,  "4.5 Mm");      // a low-orbit radius (4495 km)
    CHECK_DIST(999999.9, "1000.0 km");   // top of the km tier
    CHECK_DIST(1e6,      "1.0 Mm");      // bottom of the Mm tier
    CHECK_DIST(1e9,      "1000.0 Mm");   // a high-orbit radius
    CHECK_DIST(1e12,     "6.7 AU");      // bottom of the AU tier
    CHECK_DIST(4.495e12, "30.0 AU");     // the neptune scenario
    CHECK_DIST(1e15,     "6684.6 AU");   // the oort scenario
    CHECK_DIST(1e16,     "1.1 ly");      // bottom of the ly tier
    CHECK_DIST(1e17,     "10.6 ly");     // the interstellar scenario
    CHECK_DIST(-1e13,    "-66.8 AU");    // negative, big tier

    // --- fmt_time: the ToF / period readouts -------------------------------
    CHECK_TIME(0.0,     "00:00:00");
    CHECK_TIME(-5.0,    "00:00:00");     // negative clamps to 0
    CHECK_TIME(3661.0,  "01:01:01");
    CHECK_TIME(86400.0, "1d 00:00:00");
    CHECK_TIME(90061.0, "1d 01:01:01");
    // oort-class ToF (the planner's 3x-period cap at a 1.83e14 s period):
    // the day count (~6.4e9) is PAST INT_MAX (2.1e9) -- the old (int) cast
    // was UB here (INT_MIN on x86-64).
    CHECK_TIME(5.5e14, "6365740740d 17:46:40");

    if(failures == 0) {
        printf("test_fmt: all checks passed\n");
        return 0;
    }
    printf("test_fmt: %d FAILURE(S)\n", failures);
    return 1;
}
