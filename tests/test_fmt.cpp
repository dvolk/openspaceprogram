// test_fmt: the UI formatting helpers (src/fmt.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_fmt.cpp -o test_fmt && ./test_fmt)
//
// Pins the exact readout strings: the unit ladder boundaries
// (m / km / Mm / Gm / AU / ly), the sign handling, and the overflow-safe
// long long ToF (the old (int) cast was UB at oort/interstellar ToFs).
#include "fmt.h"

#include <cstdio>
#include <cstring>
#include <cmath>      // std::nan: a gravity readout must not render it as "0 m/s2"
#include <numbers>

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

// The System Atlas readouts (fmt_speed / fmt_deg / fmt_mass): one macro per
// helper, same exact-string contract as above.
#define CHECK_FMT(fn, arg, want) do { \
        char _b[64]; \
        const char *_g = fn(arg, _b, sizeof _b); \
        if(std::strcmp(_g, (want)) != 0) { \
            printf("FAIL %s:%d: %s(%s) = \"%s\", want \"%s\"\n", \
                   __FILE__, __LINE__, #fn, #arg, _g, (want)); \
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
    CHECK_DIST(1e9,      "1.0 Gm");      // bottom of the Gm tier (a planet's
                                         // orbit: "1000.0 Mm" did not read)
    CHECK_DIST(1.3586e10, "13.6 Gm");    // Kerbin's semi-major axis
    CHECK_DIST(9.999e11, "999.9 Gm");    // top of the Gm tier
    CHECK_DIST(1e12,     "6.7 AU");      // bottom of the AU tier
    CHECK_DIST(4.495e12, "30.0 AU");     // the neptune scenario
    CHECK_DIST(1e15,     "6684.6 AU");   // the oort scenario
    CHECK_DIST(1e16,     "1.1 ly");      // bottom of the ly tier
    CHECK_DIST(1e17,     "10.6 ly");     // the interstellar scenario
    CHECK_DIST(-1e13,    "-66.8 AU");    // negative, big tier
    CHECK_DIST(-2.5e10,  "-25.0 Gm");    // negative, Gm tier

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

    // --- the Atlas dossier readouts ----------------------------------------
    // fmt_speed: m/s below 1 km/s (a lander's dV), km/s above (the numbers
    // a transfer is planned against).
    CHECK_FMT(fmt_speed, 0.0,        "0 m/s");
    CHECK_FMT(fmt_speed, 999.6,      "1000 m/s");   // rounds up, stays m/s
    CHECK_FMT(fmt_speed, 1000.0,     "1.00 km/s");  // the km/s tier
    CHECK_FMT(fmt_speed, 2270.69,    "2.27 km/s");  // a low Kerbin orbit
    CHECK_FMT(fmt_speed, 3232.0,     "3.23 km/s");  // Kerbin escape velocity
    CHECK_FMT(fmt_speed, -250.0,     "-250 m/s");   // sign kept
    /* The rungs under 1 m/s, which the real asteroid catalogue needs. "%.0f m/s"
       printed Kamo'oalewa's escape velocity (0.0211 m/s) and 2020 CD3's (0.00046 m/s)
       as "0 m/s" -- a minimoon's escape velocity is the number a boot or a hook has to
       beat, so rounding it out of the row tells the player the rock has none. Same cut
       as fmt_accel's mm/s tier, and for the same digit reason. */
    CHECK_FMT(fmt_speed, 0.5,        "0.50 m/s");        // where the old ladder read "0 m/s"
    CHECK_FMT(fmt_speed, 9.999,      "10.00 m/s");       // rounds up, same number next tier
    CHECK_FMT(fmt_speed, 12.4,       "12 m/s");          // a nudged thruster
    CHECK_FMT(fmt_speed, 0.1,        "0.10 m/s");        // bottom of the two-decimal tier
    CHECK_FMT(fmt_speed, 0.0999,     "99.90 mm/s");      // top of the mm tier
    CHECK_FMT(fmt_speed, 0.0211,     "21.10 mm/s");      // Kamo'oalewa's escape velocity
    CHECK_FMT(fmt_speed, 0.00046,    "0.46 mm/s");       // 2020 CD3's
    CHECK_FMT(fmt_speed, 9.9e-6,     "9.90e-06 m/s");    // just under the mm tier
    CHECK_FMT(fmt_speed, -0.4,       "-0.40 m/s");       // sign kept below 1 m/s
    // A NaN must not read "0 m/s" either: same misreading, same reason as fmt_accel.
    CHECK_FMT(fmt_speed, std::nan(""), "-");
    {
        static const double worst[] = { 999999.99, -999999.99, 1e6, -1e6,
                                        0.0999, -0.0999, 1e-19, -1e-19,
                                        0.0, std::nan("") };
        for(double v : worst) {
            char wide[128], call[32];
            fmt_speed(v, wide, sizeof wide);
            fmt_speed(v, call, sizeof call);
            if(std::strcmp(wide, call) != 0) {
                printf("FAIL %s:%d: fmt_speed(%g) = \"%s\" does not fit the 32 chars"
                       " every call site passes\n", __FILE__, __LINE__, v, wide);
                failures++;
            }
        }
    }
    // fmt_deg: the authored angles are radians.
    CHECK_FMT(fmt_deg, 0.0,               "0.00 deg");
    CHECK_FMT(fmt_deg, 0.409105,          "23.44 deg");   // Earth's tilt
    CHECK_FMT(fmt_deg, std::numbers::pi,  "180.00 deg");
    CHECK_FMT(fmt_deg, -0.0872665,        "-5.00 deg");
    // fmt_mass: two decades per body class, so scientific notation.
    CHECK_FMT(fmt_mass, 5.291e22, "5.291e+22 kg");
    CHECK_FMT(fmt_mass, 0.0,      "0.000e+00 kg");
    // fmt_accel: the surface-gravity row. "%.2f m/s2" alone printed 151 of the
    // 240 bodies in solar_system_full as "0.00 m/s2", so the ladder drops to
    // mm/s2 and then to scientific rather than rounding a moon out of the game.
    // The m->mm cut sits at 1e-1, well above where the digits run out: 0.0999 is
    // one significant figure in m/s2 and four in mm/s2.
    CHECK_FMT(fmt_accel, 9.81,       "9.81 m/s2");      // Kerbin
    CHECK_FMT(fmt_accel, 274.0,      "274.00 m/s2");    // the Sun
    CHECK_FMT(fmt_accel, 1.63,       "1.63 m/s2");      // a Mun-sized moon
    CHECK_FMT(fmt_accel, 0.1,        "0.10 m/s2");      // bottom of the m tier
    CHECK_FMT(fmt_accel, 0.0999,     "99.90 mm/s2");    // top of the mm tier
    CHECK_FMT(fmt_accel, 0.01,       "10.00 mm/s2");    // where the old ladder read 0
    CHECK_FMT(fmt_accel, 1.6774e-4,  "0.17 mm/s2");     // S/2009 S1
    CHECK_FMT(fmt_accel, 1e-5,       "0.01 mm/s2");     // bottom of the mm tier
    CHECK_FMT(fmt_accel, 9.9e-6,     "9.90e-06 m/s2");  // just below it
    CHECK_FMT(fmt_accel, 4.2e-7,     "4.20e-07 m/s2");  // a dust-grain world
    CHECK_FMT(fmt_accel, 1e6,        "1.00e+06 m/s2");  // the top tier goes sci
    CHECK_FMT(fmt_accel, 0.0,        "0 m/s2");         // a hand-built body
    CHECK_FMT(fmt_accel, -9.81,      "-9.81 m/s2");     // sign kept
    CHECK_FMT(fmt_accel, -0.004,     "-4.00 mm/s2");    // sign kept in the mm tier
    // A NaN must NOT read "0 m/s2": that is the exact misreading this ladder
    // exists to prevent -- a body with no gravity is a fact, a body whose gravity
    // could not be computed is a bug, and the dossier prints them the same row.
    CHECK_FMT(fmt_accel, std::nan(""), "-");
    // Every call site passes a 32-char buffer (gameui.cpp, main.cpp). Pin that the
    // widest row each tier can emit fits, so widening a format cannot silently
    // truncate a readout: compare a roomy buffer against the 32 the game uses.
    {
        static const double worst[] = { 999999.99, -999999.99, 1e6, -1e6,
                                        1e-19, -1e-19, 0.0, std::nan("") };
        for(double v : worst) {
            char wide[128], call[32];
            fmt_accel(v, wide, sizeof wide);
            fmt_accel(v, call, sizeof call);
            if(std::strcmp(wide, call) != 0) {
                printf("FAIL %s:%d: fmt_accel(%g) = \"%s\" does not fit the 32 chars"
                       " every call site passes\n", __FILE__, __LINE__, v, wide);
                failures++;
            }
        }
    }

    if(failures == 0) {
        printf("test_fmt: all checks passed\n");
        return 0;
    }
    printf("test_fmt: %d FAILURE(S)\n", failures);
    return 1;
}
