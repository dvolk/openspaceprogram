//
// The derived body limits (src/bodylimits.h, header-only pure math): the
// near-body shell, the two SOI laws, the nesting lift, and the load-time
// ordering asserts. Pinned here without the game (like test_drag,
// test_science).
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "bodylimits.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                 \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s\n", msg);                                        \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, rel_tol, msg)                            \
    do {                                                                      \
        g_checks++;                                                           \
        double _a = (actual), _e = (expected);                                \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > rel_tol * std::fabs(_e)) { \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);          \
        }                                                                     \
    } while (0)

#define CHECK_THROWS(expr, msg)                                               \
    do {                                                                      \
        g_checks++;                                                           \
        bool _threw = false;                                                  \
        try { expr; } catch(const std::runtime_error &) { _threw = true; }    \
        if (!_threw) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s (no throw)\n", msg);                             \
        }                                                                     \
    } while (0)

#define CHECK_NOTHROW(expr, msg)                                              \
    do {                                                                      \
        g_checks++;                                                           \
        try { expr; } catch(const std::runtime_error &e) {                    \
            g_failures++;                                                     \
            printf("FAIL: %s (threw: %s)\n", msg, e.what());                  \
        }                                                                     \
    } while (0)

static void test_shell_edge() {
    printf("== shellEdge: max(kMinShell, 1.2 * top) ==\n");

    // Airless bodies and short atmospheres keep the historical flat 100 km
    // shell -- Kerbin (top 70 km) is EXACTLY preserved, so daily-driver
    // spawn radii and science cuts do not move.
    CHECK_NEAR(shellEdge(0.0), 100e3, 1e-12, "airless -> kMinShell");
    CHECK_NEAR(shellEdge(70e3), 100e3, 1e-12, "Kerbin (70 km) -> 100 km");

    // Tall atmospheres get top + 20%: the shell contains its own air and
    // LowOrbit is a real band above it.
    CHECK_NEAR(shellEdge(85e3), 102e3, 1e-12, "Earth (85 km) -> 102 km");
    CHECK_NEAR(shellEdge(200e3), 240e3, 1e-12, "Jool (200 km) -> 240 km");
    CHECK_NEAR(shellEdge(600e3), 720e3, 1e-12, "Saturn (600 km) -> 720 km");

    // The invariant the #97 fix rests on: the shell always covers the air
    // plus the frame-switch hysteresis, for EVERY top.
    const double tops[] = { 0.0, 1e3, 10e3, 50e3, 83e3, 84e3, 99e3,
                            100e3, 200e3, 600e3, 1e6 };
    for(double t : tops) {
        char buf[96];
        snprintf(buf, sizeof buf,
                 "shellEdge(%.0f) covers top + kSoiMargin", t);
        CHECK_TRUE(shellEdge(t) >= t + kSoiMargin, buf);
    }
}

static void test_soi_laws() {
    printf("== SOI laws ==\n");

    // patched_conic reproduces the KSP wiki SOIs the old K table hardcoded
    // (Squad generated the wiki values with this same formula).
    CHECK_NEAR(soiPatchedConic(13599840260.0, 5.2915793e22, 1.757e28),
               84159290.0, 2e-3, "Kerbin ~ wiki 84,159 km");
    CHECK_NEAR(soiPatchedConic(12000000.0, 9.7600236e20, 5.2915793e22),
               2429559.1, 2e-3, "Mun ~ wiki 2,430 km");
    CHECK_NEAR(soiPatchedConic(31548000.0, 1.242e17, 1.224e23),
               126120.0, 2e-3, "Gilly ~ wiki 126.1 km");

    // hill reproduces the real solar system's old hardcoded values.
    CHECK_NEAR(soiHill(1.496e11, 5.972e24, 1.989e30),
               1.4964e9, 1e-2, "Earth Hill ~ 1.496 Gm");

    // Degenerate inputs -> 0 (a law value of 0 lets the lift win).
    CHECK_NEAR(soiPatchedConic(0.0, 1.0, 1.0), 0.0, 0.0, "a=0 -> 0");
    CHECK_NEAR(soiHill(1.0, 0.0, 1.0), 0.0, 0.0, "m=0 -> 0");
    CHECK_NEAR(soiPatchedConic(1.0, 1.0, 0.0), 0.0, 0.0, "M=0 -> 0");

    // The law picker + the JSON name parse.
    CHECK_TRUE(soiByLaw(SoiLaw::PatchedConic, 12e6, 9.76e20, 5.292e22)
               == soiPatchedConic(12e6, 9.76e20, 5.292e22), "soiByLaw PC");
    CHECK_TRUE(soiByLaw(SoiLaw::Hill, 12e6, 9.76e20, 5.292e22)
               == soiHill(12e6, 9.76e20, 5.292e22), "soiByLaw Hill");
    CHECK_TRUE(soiLawFromName("hill") == SoiLaw::Hill, "parse hill");
    CHECK_TRUE(soiLawFromName("patched_conic") == SoiLaw::PatchedConic,
               "parse patched_conic");
    CHECK_THROWS(soiLawFromName("bogus"), "unknown law name throws");
}

static void test_lift() {
    printf("== inertialSoi: the nesting + spawn-bed lift ==\n");

    // Gilly: the law value (126.1 km) sits below both floors -- the
    // hysteresis lift (113 + 20 = 133 km) AND the inertial-orbit bed
    // (113 + 25 + 10 = 148 km; the bed spawns at 13 + 125 = 138 km, which
    // must resolve inside the SOI). This is the shipped nesting violator
    // the old data model could not express.
    CHECK_NEAR(inertialSoi(126120.0, 113000.0, 100000.0), 148000.0, 1e-12,
               "Gilly lifted to 148 km (bed floor)");
    // Kerbin: the law value wins by miles.
    CHECK_NEAR(inertialSoi(84159290.0, 700000.0, 100000.0), 84159290.0,
               1e-12, "Kerbin law value wins");
    // Phobos-class collapse: a ~7 km law value still yields an SOI that
    // contains the 136 km inertial-orbit bed.
    CHECK_NEAR(inertialSoi(7256.0, 111167.0, 100000.0), 146167.0, 1e-12,
               "Phobos lifted to rotSoi + 35 km");
    // A tall-atmo shell: the bed floor scales with it (Jool: 0.25*240+10 =
    // 70 km over the rot SOI, above the 20 km hysteresis floor).
    CHECK_NEAR(inertialSoi(2.456e9, 6.24e6, 240e3), 2.456e9, 1e-12,
               "Jool law value wins");
    CHECK_NEAR(inertialSoi(0.0, 6.24e6, 240e3), 6.31e6, 1e-12,
               "Jool-size shell lifts to rotSoi + 70 km");
}

static void test_validate() {
    printf("== validateBodyLimits: the ordering asserts ==\n");

    // Kerbin as derived: fine.
    CHECK_NOTHROW(validateBodyLimits("Kerbin", 600e3, 0.0, 70e3,
                                     700e3, 84159290.0), "Kerbin passes");
    // A star (dummy shell, authored universe-bound SOI): fine.
    CHECK_NOTHROW(validateBodyLimits("Kerbol", 261600000.0, 0.0, 0.0,
                                     261700000.0, 1e18), "star passes");
    // A raised sea level shifts the shell with the datum: fine.
    CHECK_NOTHROW(validateBodyLimits("Wet", 600e3, 10e3, 70e3,
                                     710e3, 84e6), "sea level passes");
    // A lifted tiny moon at exactly the derived floors: fine.
    CHECK_NOTHROW(validateBodyLimits("Gilly", 13e3, 0.0, 0.0,
                                     113e3, 148e3), "lifted Gilly passes");

    // A rot SOI that does not cover the derived shell (the old Jool shape:
    // air to 200 km, shell only 100 km).
    CHECK_THROWS(validateBodyLimits("Joolish", 6e6, 0.0, 200e3,
                                    6.1e6, 2.4e9), "short shell throws");
    // An un-enterable shell (the old system.cpp flat-1e5 default bug).
    CHECK_THROWS(validateBodyLimits("Tiny", 600e3, 0.0, 0.0,
                                    605e3, 1e9), "un-enterable throws");
    // Overlapping hysteresis bands (the old Gilly data).
    CHECK_THROWS(validateBodyLimits("Gillyish", 13e3, 0.0, 0.0,
                                    113e3, 126120.0), "nesting throws");
    // Nesting OK but the inertial-orbit bed (13 + 125 = 138 km) falls
    // outside the SOI: the finding-1 regression, pinned.
    CHECK_THROWS(validateBodyLimits("Phobosish", 13e3, 0.0, 0.0,
                                    113e3, 138e3), "spawn bed throws");
}

int main() {
    test_shell_edge();
    test_soi_laws();
    test_lift();
    test_validate();

    if(g_failures == 0) {
        printf("test_bodylimits: %d checks, 0 failures\n", g_checks);
        return 0;
    }
    printf("test_bodylimits: %d checks, %d failure(s)\n", g_checks, g_failures);
    return 1;
}
