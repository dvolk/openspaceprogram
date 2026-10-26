//
// Jet engine thrust (src/drag.h jetThrust, header-only pure math): the
// air-breathing momentum balance of an engine that burns fuel against
// FREE air. Pinned here without Bullet/GL so the law is independent of
// the render/physics chain (like test_drag).
//
//   T(v, rho) = T_fan*d + m_f*v_e*d + rho*A*v*(v_e - v),   d = jetAirGate(rho, rho_sea)
//
//   vacuum (rho = 0)      -> 0 (a jet cannot thrust in space)
//   v = 0 (at rest)       -> T_fan + m_f*v_e (the static/fan thrust)
//   v = v_e/2             -> the ram term peaks (thrust maximum)
//   v = v_e               -> the ram term is back to 0 (thrust = static)
//   v > v_e               -> the ram term turns negative; the net clamps at 0
//                            once it outweighs the static terms
//   rho < rho_sea         -> the fan + fuel terms scale with d (the flow does too)
//   degenerate inputs     -> handled (no negative thrust)
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

/* Reference values (the r1 jet, res/data/parts.json) at Kerbin sea level
   (rho_sea = 1.225). */
static const double T_FAN = 90000.0, M_F = 1.2, V_E = 550.0, A = 0.35;
static const double RHO_SEA = 1.225;

static double T(double v, double rho) {
    return jetThrust(v, rho, RHO_SEA, T_FAN, M_F, V_E, A);
}

static void test_static() {
    printf("== jetThrust: the static (fan) thrust at rest ==\n");

    // At rest at sea level: fan + fuel momentum (no ram yet).
    CHECK_NEAR(T(0.0, RHO_SEA), T_FAN + M_F * V_E, 1e-12,
               "v=0 -> T_fan + m_f*v_e");

    // The fan term dominates the small fuel-momentum term.
    CHECK_TRUE(T(0.0, RHO_SEA) > T_FAN, "static thrust >= the fan thrust");

    // A bigger fan -> a bigger resting thrust (the fan IS the VTOL
    // capability: a 25% bigger fan lifts more).
    CHECK_TRUE(jetThrust(0.0, RHO_SEA, RHO_SEA, 1.25 * T_FAN, M_F, V_E, A)
               > jetThrust(0.0, RHO_SEA, RHO_SEA, T_FAN, M_F, V_E, A),
               "a bigger fan beats the reference fan at rest");
}

static void test_ram() {
    printf("== jetThrust: the ram (air momentum) term ==\n");

    // The ram term rho*A*v*(v_e - v) peaks at v = v_e/2.
    const double vpeak = V_E / 2.0;
    const double peak = T(vpeak, RHO_SEA);
    CHECK_TRUE(peak > T(0.0, RHO_SEA), "thrust rises off the static floor");

    // It falls back to the static value at v = v_e (the ram term is 0 there).
    CHECK_NEAR(T(V_E, RHO_SEA), T_FAN + M_F * V_E, 1e-9, "v=v_e -> back to static");

    // Monotone up to the peak, then down (the physical turbofan shape).
    CHECK_TRUE(T(vpeak * 0.5, RHO_SEA) < peak, "rising toward the peak");
    CHECK_TRUE(T(vpeak * 1.5, RHO_SEA) < peak, "falling past the peak");

    // The maximum is at v = v_e/2 (the derivative of v*(v_e - v) is 0 there).
    const double lo = T(V_E * 0.49, RHO_SEA), hi = T(V_E * 0.51, RHO_SEA);
    CHECK_TRUE(lo <= peak && hi <= peak, "the maximum is at v = v_e/2");
}

static void test_overspeed() {
    printf("== jetThrust: overspeed (v > v_e) never reverses ==\n");

    // Past v_e the ram term is negative (the engine would drag); the net
    // clamps at 0, so a jet never pushes backwards.
    CHECK_TRUE(T(2.0 * V_E, RHO_SEA) <= T_FAN + M_F * V_E, "overspeed <= static");
    CHECK_TRUE(T(10.0 * V_E, RHO_SEA) >= 0.0, "thrust is never negative");
    CHECK_TRUE(T(10.0 * V_E, RHO_SEA) < T(0.0, RHO_SEA), "overspeed < static");
}

static void test_density() {
    printf("== jetThrust: the density falloff ==\n");

    // Half the sea-level density -> the static terms halve (the gate d = 0.5).
    CHECK_NEAR(T(0.0, 0.5 * RHO_SEA), 0.5 * (T_FAN + M_F * V_E), 1e-12,
               "rho=rho_sea/2 -> static/2");

    // The falloff is relative to the body's own sea level: a thin
    // atmosphere at its own sea level still reads full.
    CHECK_NEAR(jetThrust(0.0, 0.12, 0.12, T_FAN, M_F, V_E, A), T_FAN + M_F * V_E, 1e-12,
               "thin atmo at its sea level -> full static");

    // Denser than sea level clamps the gate at 1 (no overshoot).
    CHECK_NEAR(jetThrust(0.0, 2.0 * RHO_SEA, RHO_SEA, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E, 1e-12, "rho > rho_sea: the gate clamps at 1");

    // The gate clamps the fan + fuel at sea level, but the RAM term is
    // linear in rho (not gated by d): at rho = 2*rho_sea the fan is full
    // and the ram doubles.
    CHECK_NEAR(jetThrust(V_E / 2.0, 2.0 * RHO_SEA, RHO_SEA, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E + (2.0 * RHO_SEA) * A * (V_E / 2.0) * (V_E - V_E / 2.0),
               1e-9, "rho > rho_sea: fan clamps at 1, ram follows local rho");
}

static void test_air_gate() {
    printf("== jetAirGate: the share of sea-level air (thrust AND fuel) ==\n");

    CHECK_NEAR(jetAirGate(RHO_SEA, RHO_SEA), 1.0, 1e-12, "sea level -> 1");
    CHECK_NEAR(jetAirGate(0.5 * RHO_SEA, RHO_SEA), 0.5, 1e-12, "half density -> half");
    CHECK_NEAR(jetAirGate(2.0 * RHO_SEA, RHO_SEA), 1.0, 1e-12, "denser clamps at 1");
    CHECK_NEAR(jetAirGate(0.0, RHO_SEA), 0.0, 0.0, "vacuum -> 0 (no thrust, no burn)");
    CHECK_NEAR(jetAirGate(RHO_SEA, 0.0), 0.0, 0.0, "no sea-level reference -> 0");
    CHECK_NEAR(jetAirGate(std::nan(""), RHO_SEA), 0.0, 0.0,
               "NaN density fails closed (no thrust AND no burn)");

    // The flow gate IS the gate the thrust law uses: at any density, static
    // thrust is the gate times the sea-level static, so the tank drains in
    // lockstep with the fan + fuel terms (ApplyThrust multiplies by the same
    // jetAirGate).
    const double gates[] = {0.1, 0.3, 0.75, 1.0};
    for(double g : gates) {
        CHECK_NEAR(T(0.0, g * RHO_SEA),
                   jetAirGate(g * RHO_SEA, RHO_SEA) * (T_FAN + M_F * V_E),
                   1e-12, "flow gate == the thrust's fan gate");
    }
}

static void test_vacuum() {
    printf("== jetThrust: vacuum ==\n");

    // No air at all: ZERO thrust at ANY speed -- a jet cannot run without
    // air, and ApplyThrust burns no fuel (the flow is gated by jetAirGate).
    CHECK_NEAR(jetThrust(0.0, 0.0, RHO_SEA, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at rest -> 0");
    CHECK_NEAR(jetThrust(300.0, 0.0, RHO_SEA, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at speed -> 0 (no air, no thrust)");
    // A body with no sea-level density to reference is a vacuum too.
    CHECK_NEAR(jetThrust(0.0, RHO_SEA, 0.0, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho_sea=0 -> 0");
}

static void test_degenerate() {
    printf("== jetThrust: degenerate inputs ==\n");
    const double rho = RHO_SEA;

    // A negative speed reads as zero speed (the static thrust).
    CHECK_NEAR(jetThrust(-50.0, rho, RHO_SEA, T_FAN, M_F, V_E, A), T_FAN + M_F * V_E,
               1e-12, "v<0 -> the static thrust");

    // No intake area: no ram term, just the static (fan + fuel) thrust.
    CHECK_NEAR(jetThrust(100.0, rho, RHO_SEA, T_FAN, M_F, V_E, 0.0), T_FAN + M_F * V_E,
               1e-12, "A=0 -> static only (no ram)");

    // No fan thrust: the static floor is just the (small) fuel momentum.
    CHECK_NEAR(jetThrust(0.0, rho, RHO_SEA, 0.0, M_F, V_E, A), M_F * V_E, 1e-12,
               "T_fan=0 -> just the fuel momentum");
}

int main() {
    test_static();
    printf("\n");
    test_ram();
    printf("\n");
    test_overspeed();
    printf("\n");
    test_density();
    printf("\n");
    test_air_gate();
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
