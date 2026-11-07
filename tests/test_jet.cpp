//
// Jet engine thrust (src/drag.h jetThrust, header-only pure math): the
// air-breathing momentum balance of an engine that burns fuel against
// FREE air. Pinned here without Bullet/GL so the law is independent of
// the render/physics chain (like test_drag).
//
//   T(v, rho) = T_fan*d + m_f*v_e*d + rho*A*v*(v_e - v),
//   d = jetAirGate(rho, oxidizer_share)   -- min(the intake against the rating
//   density, the oxygen in that intake against dry air's), so an air with no
//   oxidizer is a dead engine at any density (#630).
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
        /* Relative, except at an expectation of exactly zero (#510):              \
           rel_tol * 0.0 would demand bit-exactness, which is not what a           \
           caller writing 1e-12 means. 0.0 AS a tolerance stays exact, so          \
           a bit-identical pin keeps its teeth. */                                 \
        const double _tol = (_e == 0.0) ? rel_tol                                  \
                                        : rel_tol * std::fabs(_e);                 \
        if (!std::isfinite(_a) || !std::isfinite(_e)                              \
            || std::fabs(_a - _e) > _tol) {                                       \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);          \
        }                                                                     \
    } while (0)

/* Reference values (the r1 jet, res/data/parts.json) at Kerbin sea level --
   rho 1.225, which IS constants.h:kAirSeaLevelDensity, the intake the part's
   authored fan thrust and fuel rate are rated at. OX_AIR is what dry air's mix
   (atmo.h:dryAirMix) resolves to for the oxidizer half of the gate: 0.231 kg of
   O2 per kg of air. With dry air the two halves of the gate are the same number,
   which is why every pre-#630 pin below keeps its value. */
static const double T_FAN = 90000.0, M_F = 1.2, V_E = 550.0, A = 0.35;
static const double RHO_SEA = 1.225;
static const double OX_AIR = dryAirOxidizerShare();

static double T(double v, double rho) {
    return jetThrust(v, rho, OX_AIR, T_FAN, M_F, V_E, A);
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
    CHECK_TRUE(jetThrust(0.0, RHO_SEA, OX_AIR, 1.25 * T_FAN, M_F, V_E, A)
               > jetThrust(0.0, RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A),
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

    /* The falloff is ABSOLUTE, not per-body (#630). The law used to divide by the
       body's own sea-level density, so a jet standing on Mars read 1.0 of Mars and
       produced its full authored fan thrust on 0.02 kg/m3 of CO2. A part's
       jet_fan_thrust is one machine's number at ONE intake density, so 0.02 kg/m3
       buys 0.016 of it, whatever planet it is standing on. */
    CHECK_NEAR(jetThrust(0.0, 0.12, OX_AIR, T_FAN, M_F, V_E, A),
               (0.12 / kAirSeaLevelDensity) * (T_FAN + M_F * V_E), 1e-12,
               "0.12 kg/m3 buys 0.098 of the rated fan, not full thrust");

    // Denser than the rating intake clamps the gate at 1 (no overshoot).
    CHECK_NEAR(jetThrust(0.0, 2.0 * RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E, 1e-12, "rho > rating: the gate clamps at 1");

    // The gate clamps the fan + fuel at sea level, but the RAM term is
    // linear in rho (not gated by d): at rho = 2*rho_sea the fan is full
    // and the ram doubles.
    CHECK_NEAR(jetThrust(V_E / 2.0, 2.0 * RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E + (2.0 * RHO_SEA) * A * (V_E / 2.0) * (V_E - V_E / 2.0),
               1e-9, "rho > rho_sea: fan clamps at 1, ram follows local rho");
}

static void test_air_gate() {
    printf("== jetAirGate: the share of the rated intake (thrust AND fuel) ==\n");

    CHECK_NEAR(jetAirGate(RHO_SEA, OX_AIR), 1.0, 1e-12, "the rating intake -> 1");
    CHECK_NEAR(jetAirGate(0.5 * RHO_SEA, OX_AIR), 0.5, 1e-12, "half density -> half");
    CHECK_NEAR(jetAirGate(2.0 * RHO_SEA, OX_AIR), 1.0, 1e-12, "denser clamps at 1");
    CHECK_NEAR(jetAirGate(0.0, OX_AIR), 0.0, 0.0, "vacuum -> 0 (no thrust, no burn)");
    // A body whose air states no oxidizer is a vacuum to a jet, at any density.
    CHECK_NEAR(jetAirGate(RHO_SEA, 0.0), 0.0, 0.0, "no oxidizer in the air -> 0");
    CHECK_NEAR(jetAirGate(std::nan(""), OX_AIR), 0.0, 0.0,
               "NaN density fails closed (no thrust AND no burn)");
    CHECK_NEAR(jetAirGate(RHO_SEA, std::nan("")), 0.0, 0.0,
               "NaN oxidizer fails closed the same way");

    // The flow gate IS the gate the thrust law uses: at any density, static
    // thrust is the gate times the sea-level static, so the tank drains in
    // lockstep with the fan + fuel terms (ApplyThrust multiplies by the same
    // jetAirGate).
    const double gates[] = {0.1, 0.3, 0.75, 1.0};
    for(double g : gates) {
        CHECK_NEAR(T(0.0, g * RHO_SEA),
                   jetAirGate(g * RHO_SEA, OX_AIR) * (T_FAN + M_F * V_E),
                   1e-12, "flow gate == the thrust's fan gate");
    }
}

static void test_vacuum() {
    printf("== jetThrust: vacuum ==\n");

    // No air at all: ZERO thrust at ANY speed -- a jet cannot run without
    // air, and ApplyThrust burns no fuel (the flow is gated by jetAirGate).
    CHECK_NEAR(jetThrust(0.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at rest -> 0");
    CHECK_NEAR(jetThrust(300.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at speed -> 0 (no air, no thrust)");
    /* An air with no oxidizer in it is the other vacuum, and the denser it is the
       more obviously the old law was wrong: Eve's sea-level air is 1.7 kg/m3,
       39% denser than Kerbin's, and every gram of it is CO2. */
    CHECK_NEAR(jetThrust(0.0, 1.7, 0.0, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "1.7 kg/m3 of CO2 -> 0 (no oxidizer, no thrust)");
    CHECK_NEAR(jetThrust(V_E / 2.0, 1.7, 0.0, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "no oxidizer -> no thrust at the ram peak either");
}

static void test_degenerate() {
    printf("== jetThrust: degenerate inputs ==\n");
    const double rho = RHO_SEA;

    // A negative speed reads as zero speed (the static thrust).
    CHECK_NEAR(jetThrust(-50.0, rho, OX_AIR, T_FAN, M_F, V_E, A), T_FAN + M_F * V_E,
               1e-12, "v<0 -> the static thrust");

    // No intake area: no ram term, just the static (fan + fuel) thrust.
    CHECK_NEAR(jetThrust(100.0, rho, OX_AIR, T_FAN, M_F, V_E, 0.0), T_FAN + M_F * V_E,
               1e-12, "A=0 -> static only (no ram)");

    // No fan thrust: the static floor is just the (small) fuel momentum.
    CHECK_NEAR(jetThrust(0.0, rho, OX_AIR, 0.0, M_F, V_E, A), M_F * V_E, 1e-12,
               "T_fan=0 -> just the fuel momentum");
}

/* The half of the gate that only a MIX can state (#630): kilograms of burnable
   oxidizer per kilogram of air, which is what makes "no free oxygen here" a fact the
   engine acts on rather than a fact nobody can say. */
static void test_oxidizer() {
    printf("== jetAirGate: an air with no oxidizer is a dead engine ==\n");

    // Dry air's share, from the species table rather than typed here: if the table
    // moves, the rating the gate divides by moves with it.
    CHECK_NEAR(OX_AIR, 0.2314, 5e-5, "dry air is 23.1% oxidizer by mass");

    // min, not a product. On an oxygen world the intake half and the oxygen half are
    // the same number, so multiplying them would bill thin air twice.
    CHECK_NEAR(jetAirGate(0.3 * RHO_SEA, OX_AIR), 0.3, 1e-12,
               "dry air at 0.3 of the rating density -> 0.3, not 0.09");

    // Mars: 0.02 kg/m3 of CO2 with a 0.13% oxygen trace (res/systems' CO2_HEAVY).
    // The air half says 0.016; the oxygen half says 6.8e-5. The scarcer one wins.
    const double mars_rho = 0.02, mars_ox = 0.000959;
    const double air_half = mars_rho / kAirSeaLevelDensity;
    const double ox_half = mars_rho * mars_ox / (kAirSeaLevelDensity * OX_AIR);
    CHECK_TRUE(ox_half < air_half, "on Mars the oxygen runs out long before the air does");
    CHECK_NEAR(jetAirGate(mars_rho, mars_ox), ox_half, 1e-12,
               "the scarcer half IS the gate");

    // An oxygen-rich air does not buy an engine more than its rated intake: the
    // density half clamps at 1 however much oxidizer is in the air.
    CHECK_NEAR(jetAirGate(3.0 * RHO_SEA, 1.0), 1.0, 1e-12,
               "dense oxygen-rich air clamps at 1, not above");

    // A negative share is not a small one: it fails closed like NaN does.
    CHECK_NEAR(jetAirGate(RHO_SEA, -1.0), 0.0, 0.0, "a negative share fails closed");

    /* jetOxGate is that same limit on its own, with no density in it -- it is what
       pays for the exhaust momentum below. It clamps: extra oxygen does not make a
       fan push harder than its intake. */
    CHECK_NEAR(jetOxGate(OX_AIR), 1.0, 1e-12, "dry air pays in full");
    CHECK_NEAR(jetOxGate(2.0 * OX_AIR), 1.0, 1e-12, "an oxygen-rich air clamps at 1");
    CHECK_NEAR(jetOxGate(0.0), 0.0, 0.0, "no oxidizer pays nothing");

    /* The RAM term pays too, and this is where the two halves of the gate stop being
       one number. At the ram peak (v = ve/2) the exhaust momentum is most of the
       thrust: Mars's datum -- 0.02 kg/m3 of CO2 with a 0.13% oxygen trace -- bills the
       fan 6.8e-5 of its rating, 6.1 N, and the air it captures is 0.02 kg/m3 whatever
       the gas is. A law that threw that air out the back ungated would report 536 N at
       275 m/s for a fuel flow the same gate has already cut to 8e-5 kg/s: momentum out
       of no burn. Hand-computed, the burn pays 8.33 N -- 6.13 fan and fuel, 2.19 ram.
       Same density and speed with dry air's oxygen instead: 2010 N, so the difference
       is the gas and not the thinness. */
    const double v_peak = V_E / 2.0;
    const double co2 = jetThrust(v_peak, mars_rho, mars_ox, T_FAN, M_F, V_E, A);
    CHECK_NEAR(co2, 8.328, 1e-3, "Mars's datum at the ram peak is 8.3 N, not 536 N");
    CHECK_TRUE(co2 < 0.01 * jetThrust(v_peak, mars_rho, OX_AIR, T_FAN, M_F, V_E, A),
               "a CO2 trace sky is under 1% of what the same thin air does with oxygen");
}

/* THE AIR HANDS THE GATE ITS OWN COMPOSITION (#637). Every production caller holds the
   DragAtmosphere its density came from, so the gate takes that rather than a share
   fetched separately off the body -- two fetches can name two different worlds, and
   nothing in the type used to notice. The two-argument form above stays the law; these
   pins are about the door, not about re-deriving the arithmetic. */
static void test_the_air_carries_its_own_composition() {
    printf("== the profile carries the oxidizer share the gate needs (#637) ==\n");

    /* An air-shaped fixture: what matters here is the share that travels with it, not
       the profile (Kerbin's own air became a level table in #686). */
    DragAtmosphere air { RHO_SEA, 5500.0, 70000.0 };
    CHECK_NEAR(air.oxidizer_share, OX_AIR, 0.0,
               "a hand-built profile defaults to dry air, exactly as its molar_mass and "
               "gamma do: a fixture must not be a silently dead engine");

    const double mars_rho = 0.02, mars_ox = 0.000959;
    DragAtmosphere mars { mars_rho, 20000.0, 80000.0 };
    mars.oxidizer_share = mars_ox;

    /* The door is the same law, bit for bit -- a second arithmetic here would be the
       drift this refactor exists to prevent. */
    CHECK_NEAR(jetAirGate(air, RHO_SEA), jetAirGate(RHO_SEA, air.oxidizer_share),
               0.0, "jetAirGate(profile, rho) IS jetAirGate(rho, profile.share)");
    CHECK_NEAR(jetAirGate(mars, mars_rho), jetAirGate(mars_rho, mars_ox), 0.0,
               "on a Mars-shaped sky too");
    CHECK_NEAR(jetThrust(V_E / 2.0, mars_rho, mars, T_FAN, M_F, V_E, A),
               jetThrust(V_E / 2.0, mars_rho, mars_ox, T_FAN, M_F, V_E, A), 0.0,
               "and the engine's door is the same engine");

    /* It reads the FIELD, not a constant: one density, two skies. */
    CHECK_TRUE(jetAirGate(air, mars_rho) > 200.0 * jetAirGate(mars, mars_rho),
               "the same 0.02 kg/m3 gates two ways once the composition travels with the "
               "air -- a door that ignored the share would read dry air everywhere");

    /* A sky with nothing to burn against stays dead through the door, thrust AND burn. */
    DragAtmosphere eve { 1.7, 7000.0, 90000.0 };
    eve.oxidizer_share = 0.0;
    CHECK_NEAR(jetAirGate(eve, 1.7), 0.0, 0.0,
               "1.7 kg/m3 with no oxidizer is a hole in the hull, not an engine");
    CHECK_NEAR(jetThrust(V_E / 2.0, 1.7, eve, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "and it hands back no thrust either");
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
    printf("\n");
    test_oxidizer();
    printf("\n");
    test_the_air_carries_its_own_composition();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
