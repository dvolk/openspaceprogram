//
// Jet engine thrust factor (src/drag.h jetThrustFactor, header-only pure
// math): the air-breathing multiplier on a jet's rated thrust -- the speed
// ramp (the VTOL floor) times the density falloff. Pinned here without
// Bullet/GL so the law is independent of the render/physics chain (like
// test_drag).
//
//   f(v, rho) = [ f0 + (1 - f0) * min(v / v_rated, 1) ] * min(rho / rho_sea, 1)
//
//   vacuum (rho = 0)      -> 0 (a jet cannot thrust in space)
//   v = 0 (at rest)       -> f0 (the VTOL floor)
//   v = v_rated           -> 1 at sea-level density
//   v > v_rated           -> the ramp saturates at 1 (no overshoot)
//   rho < rho_sea         -> the falloff is linear in rho/rho_sea
//   degenerate inputs     -> 0
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "drag.h"

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

static void test_floor() {
    printf("== jetThrustFactor: the VTOL floor (zero airspeed) ==\n");
    const double rho = 1.225, rho_sea = 1.225, f0 = 0.3, v_rated = 100.0;

    // At rest at sea level: exactly the floor fraction.
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, f0, v_rated), 0.3, 1e-12,
               "v=0 -> f0");

    // A higher floor -> a higher resting thrust (the floor IS the VTOL
    // capability: 0.5 takes off easier than 0.3).
    CHECK_TRUE(jetThrustFactor(0.0, rho, rho_sea, 0.5, v_rated)
               > jetThrustFactor(0.0, rho, rho_sea, f0, v_rated),
               "f0=0.5 beats f0=0.3 at rest");

    // A floor of 1.0 = a full-thrust VTOL engine (no ramp at all).
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, 1.0, v_rated), 1.0, 1e-12,
               "f0=1 -> 1 at rest");
}

static void test_ramp() {
    printf("== jetThrustFactor: the speed ramp ==\n");
    const double rho = 1.225, rho_sea = 1.225, f0 = 0.3, v_rated = 100.0;

    // Linear between the floor and rated: half speed -> halfway up.
    CHECK_NEAR(jetThrustFactor(50.0, rho, rho_sea, f0, v_rated),
               0.3 + 0.7 * 0.5, 1e-12, "v=v_rated/2 -> halfway up the ramp");

    // Rated speed: the full factor (at sea level).
    CHECK_NEAR(jetThrustFactor(100.0, rho, rho_sea, f0, v_rated), 1.0, 1e-12,
               "v=v_rated -> 1");

    // Supersonic: the ramp SATURATES at 1 (no overshoot past rated).
    CHECK_NEAR(jetThrustFactor(300.0, rho, rho_sea, f0, v_rated), 1.0, 1e-12,
               "v>v_rated -> 1 (saturated)");

    // Monotone: faster airspeed, more thrust (until the saturation).
    CHECK_TRUE(jetThrustFactor(20.0, rho, rho_sea, f0, v_rated)
               < jetThrustFactor(80.0, rho, rho_sea, f0, v_rated),
               "ramp is monotone in v");
}

static void test_density() {
    printf("== jetThrustFactor: the density falloff ==\n");
    const double rho_sea = 1.225, f0 = 0.3, v_rated = 100.0;

    // Half the sea-level density -> half the (resting) thrust.
    CHECK_NEAR(jetThrustFactor(0.0, 0.6125, rho_sea, f0, v_rated),
               0.5 * 0.3, 1e-12, "rho=rho_sea/2 -> factor/2");

    // The falloff is relative to the body's own sea level: a thin
    // atmosphere at its own sea level still reads full.
    CHECK_NEAR(jetThrustFactor(0.0, 0.12, 0.12, f0, v_rated), 0.3, 1e-12,
               "thin atmo at its sea level -> full factor");

    // Higher altitude (lower rho) -> less thrust.
    CHECK_TRUE(jetThrustFactor(0.0, 0.8, rho_sea, f0, v_rated)
               < jetThrustFactor(0.0, 1.1, rho_sea, f0, v_rated),
               "denser air -> more thrust");

    // Denser than sea level clamps the DENSITY multiplier at 1 (no
    // overshoot) -- the total factor is still the ramp (f0 at rest).
    CHECK_NEAR(jetThrustFactor(0.0, 2.0 * rho_sea, rho_sea, f0, v_rated),
               f0, 1e-12, "rho > rho_sea: the density multiplier clamps at 1");
}

static void test_vacuum() {
    printf("== jetThrustFactor: vacuum ==\n");
    const double f0 = 0.3, v_rated = 100.0;

    // No air at all (a body with no atmosphere, or above it): ZERO thrust
    // at ANY speed -- a jet cannot run without air.
    CHECK_NEAR(jetThrustFactor(0.0, 0.0, 1.225, f0, v_rated), 0.0, 0.0,
               "rho=0 at rest -> 0");
    CHECK_NEAR(jetThrustFactor(300.0, 0.0, 1.225, f0, v_rated), 0.0, 0.0,
               "rho=0 at speed -> 0 (no air, no thrust)");
    // A body with no sea-level density to reference is a vacuum too.
    CHECK_NEAR(jetThrustFactor(0.0, 1.225, 0.0, f0, v_rated), 0.0, 0.0,
               "rho_sea=0 -> 0");
}

static void test_degenerate() {
    printf("== jetThrustFactor: degenerate inputs ==\n");
    const double rho = 1.225, rho_sea = 1.225;

    // No rated speed: no ramp to climb -> 0.
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, 0.3, 0.0), 0.0, 0.0,
               "v_rated=0 -> 0");
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, 0.3, -5.0), 0.0, 0.0,
               "v_rated<0 -> 0");
    // A negative floor clamps to 0 (the factor never goes negative).
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, -1.0, 100.0), 0.0, 0.0,
               "f0<0 clamps to 0");
    // A floor above 1 clamps to 1 (no overshoot of rated at rest).
    CHECK_NEAR(jetThrustFactor(0.0, rho, rho_sea, 2.0, 100.0), 1.0, 1e-12,
               "f0>1 clamps to 1");
    // A negative speed reads as zero speed (the floor).
    CHECK_NEAR(jetThrustFactor(-50.0, rho, rho_sea, 0.3, 100.0), 0.3, 1e-12,
               "v<0 -> the floor");
}

int main() {
    test_floor();
    printf("\n");
    test_ramp();
    printf("\n");
    test_density();
    printf("\n");
    test_vacuum();
    printf("\n");
    test_degenerate();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
