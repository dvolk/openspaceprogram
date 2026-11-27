//
// Jet engine thrust and fuel flow (src/drag.h jetEngine, header-only pure math): the
// air-breathing momentum balance of an engine that burns fuel against FREE air.
// Pinned here without Bullet/GL so the law is independent of the render/physics
// chain (like test_drag).
//
//   T(v, rho) = T_fan' + (m_air + m_hot) * (v_exit - v)
//     T_fan'  = the fan the FUEL can power, capped at T_fan * jetIntakeGate(rho)
//     m_air   = rho*A*v                       the air the intake captures
//     m_hot   = min(burnCap - fanPower/Q, m_air*(v_e^2 - v^2)/(2Q))
//     v_exit  = jetRamExitSpeed(...) = min(v_e, sqrt(v^2 + 2 Q m_hot / m_air))
//
// Four refusals, each in its own currency (#630, #1206 and this pass):
//   the AIR gates the fan      -- jetIntakeGate(rho) = min(1, rho/1.225), absolute;
//   the CHEMISTRY caps the burn -- jetBurnCap: oxidizer in the swallowed air over
//                                 stoichiometry (constants.h:kStoichO2PerKgJetfuel),
//                                 against the pump;
//   that cap POWERS the fan    -- a disc making T through A needs T^{3/2}/sqrt(2 rho A),
//                                 so the fan's thrust is bought, not authored free;
//   the leftover ENERGY caps the exhaust -- v_exit above.
//
//   vacuum (rho = 0)      -> 0 (a jet cannot thrust in space)
//   share = 0 (Jool)      -> 0 (nothing to burn it against)
//   v = 0 (at rest)       -> T_fan, on the fuel the disc's power costs
//   v = v_e/2             -> the ram term peaks (thrust maximum)
//   v = v_e               -> the ram term is back to 0 (thrust = the fan)
//   v > v_e               -> the ram term turns negative; the net clamps at 0
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "drag.h"
#include "tchecks.h"

/* Reference values (the r1 jet, res/data/parts.json) at Kerbin sea level --
   rho 1.225, which IS constants.h:kAirSeaLevelDensity, the intake the part's authored
   fan thrust and fuel pump are rated at. OX_AIR is what dry air's mix
   (atmo.h:dryAirMix) resolves to: 0.2314 kg of O2 per kg of air. */
static const double T_FAN = 90000.0, M_F = 1.2, V_E = 550.0, A = 0.35;
static const double RHO_SEA = 1.225;
static const double OX_AIR = dryAirOxidizerShare();

static double T(double v, double rho) {
    return jetEngine(v, rho, OX_AIR, T_FAN, M_F, V_E, A).thrust;
}
static double BURN(double v, double rho) {
    return jetEngine(v, rho, OX_AIR, T_FAN, M_F, V_E, A).burn;
}

static void test_static() {
    printf("== jetEngine: the static (fan) thrust at rest, and what it costs ==\n");

    // At rest: the fan, at full rating. No ram, because nothing is captured yet.
    CHECK_REL(T(0.0, RHO_SEA), T_FAN, 1e-12, "v=0 -> the fan rating");

    /* The fan is not free. An ideal actuator disc making 90 kN through 0.35 m2 holds
       323.97 m/s across it and wants 29.16 MW, which is 0.678 kg/s of jetfuel -- more
       than half the pump. The old law spent nothing here: T_fan*d had no mass flow
       attached to it anywhere (#1233). */
    CHECK_REL(jetDiscSpeed(T_FAN, RHO_SEA, A), 323.9695, 1e-6,
              "the disc speed a 90 kN fan holds through 0.35 m2 of sea-level air");
    CHECK_REL(BURN(0.0, RHO_SEA) * kJetFuelHeatPerKg,
              T_FAN * jetDiscSpeed(T_FAN, RHO_SEA, A), 1e-9,
              "the whole burn at rest is the disc's power: 29.16 MW of it");
    CHECK_REL(BURN(0.0, RHO_SEA), 0.678076, 1e-5, "0.678 kg/s just to hold the fan");
    CHECK_TRUE(BURN(0.0, RHO_SEA) > 0.5 * M_F,
               "a jet holding on the pad spends over half its pump on the fan alone");

    // A bigger fan -> a bigger resting thrust, and a bigger bill for it.
    CHECK_TRUE(jetEngine(0.0, RHO_SEA, OX_AIR, 1.25 * T_FAN, M_F, V_E, A).thrust
                   > T(0.0, RHO_SEA),
               "a bigger fan beats the reference fan at rest");
    CHECK_TRUE(jetEngine(0.0, RHO_SEA, OX_AIR, 1.25 * T_FAN, M_F, V_E, A).burn
                   > BURN(0.0, RHO_SEA),
               "and pays for it: fan thrust and fan power come from the same fuel");
}

static void test_ram() {
    printf("== jetEngine: the ram (air momentum) term ==\n");

    /* The ram term m_air*(v_e - v) peaks at v = v_e/2 while the exhaust stays at the
       authored promise, which it does here: the promise's price at 275 m/s is
       0.311 kg/s and the sky allows 1.2 (test_the_promise_is_affordable). */
    const double vpeak = V_E / 2.0;
    const double peak = T(vpeak, RHO_SEA);
    CHECK_TRUE(peak > T(0.0, RHO_SEA), "thrust rises off the static floor");

    // It falls back to the fan at v = v_e (the ram term is 0 there).
    CHECK_REL(T(V_E, RHO_SEA), T_FAN, 1e-9, "v=v_e -> back to the fan");

    // Monotone up to the peak, then down (the physical turbofan shape).
    CHECK_TRUE(T(vpeak * 0.5, RHO_SEA) < peak, "rising toward the peak");
    CHECK_TRUE(T(vpeak * 1.5, RHO_SEA) < peak, "falling past the peak");
    const double lo = T(V_E * 0.49, RHO_SEA), hi = T(V_E * 0.51, RHO_SEA);
    CHECK_TRUE(lo <= peak && hi <= peak, "the maximum is at v = v_e/2");

    // The burn follows the thrust, not the throttle plate: 275 m/s spends more than a
    // hold, because the hot stream is now buying momentum as well as the disc.
    CHECK_REL(BURN(vpeak, RHO_SEA), 0.989122, 1e-5, "the burn at the ram peak");
    CHECK_TRUE(BURN(vpeak, RHO_SEA) > BURN(0.0, RHO_SEA),
               "making thrust costs fuel over merely holding the fan");
}

static void test_overspeed() {
    printf("== jetEngine: overspeed (v > v_e) never reverses ==\n");

    /* Past v_e the nozzle cannot outrun the hull: a 550 m/s exhaust against a 1100 m/s
       ship is a brake, the sum goes negative and clamps at 0. That is what an authored
       nozzle means, and it is why #1206 needs parts authored with v_e past the speed
       they are flown at rather than a rule that caps them. */
    CHECK_TRUE(T(2.0 * V_E, RHO_SEA) <= T_FAN, "overspeed <= the fan rating");
    CHECK_TRUE(T(10.0 * V_E, RHO_SEA) >= 0.0, "thrust is never negative");
    CHECK_TRUE(T(10.0 * V_E, RHO_SEA) < T(0.0, RHO_SEA), "overspeed < the static thrust");
    /* The clamp hides a real drag, and the law still reports the fan's demand; the
       caller (Vehicle::applyThrust) shuts the engine when the thrust is 0, so the ship
       neither gains push nor pays fuel there. Pinned so the two cannot drift apart. */
    CHECK_REL(BURN(2.0 * V_E, RHO_SEA), BURN(0.0, RHO_SEA), 1e-12,
              "a clamped engine still names the fan's demand for fuel");
}

static void test_density() {
    printf("== jetEngine: the density falloff, absolute and not per-body ==\n");

    // Half the sea-level density -> half the rated fan.
    CHECK_REL(T(0.0, 0.5 * RHO_SEA), 0.5 * T_FAN, 1e-12, "rho=rho_sea/2 -> the fan halves");

    /* The falloff is ABSOLUTE (#630). The law used to divide by the body's own
       sea-level density, so a jet standing on Mars read 1.0 of Mars and made its full
       authored fan thrust out of 0.02 kg/m3 of CO2. A part's jet_fan_thrust is one
       machine at ONE intake density, so 0.12 kg/m3 buys 0.098 of it on any planet. */
    CHECK_REL(jetEngine(0.0, 0.12, OX_AIR, T_FAN, M_F, V_E, A).thrust,
              (0.12 / kAirSeaLevelDensity) * T_FAN, 1e-12,
              "0.12 kg/m3 buys 0.098 of the rated fan, not full thrust");

    // Denser than the rating clamps the gate at 1: no overshoot in thrust.
    CHECK_REL(T(0.0, 2.0 * RHO_SEA), T_FAN, 1e-12, "rho > rating: the fan clamps at 1");

    /* ...but not in POWER. The same 90 kN through denser air needs a slower disc, so
       holding station at 2x the rating density costs 20.6 MW instead of 29.2. This is
       the fan's side of the intake physics, and the reason the gate clamps rather than
       paying the fan more air. */
    CHECK_REL(BURN(0.0, 2.0 * RHO_SEA) * kJetFuelHeatPerKg,
              T_FAN * jetDiscSpeed(T_FAN, 2.0 * RHO_SEA, A), 1e-9,
              "denser air holds the same fan for less power: the disc turns slower");

    /* The ram term follows the LOCAL density (not the gate), so dense air buys thrust --
       and the extra air has to be heated too, which is why the hot share (0.622 kg/s,
       not the whole 1.102 kg/s burn) rides the stream. */
    const double air2 = (2.0 * RHO_SEA) * A * (V_E / 2.0);
    const double hot2 = BURN(V_E / 2.0, 2.0 * RHO_SEA)
                        - T_FAN * jetDiscSpeed(T_FAN, 2.0 * RHO_SEA, A) / kJetFuelHeatPerKg;
    CHECK_REL(T(V_E / 2.0, 2.0 * RHO_SEA), T_FAN + (air2 + hot2) * (V_E - V_E / 2.0), 1e-9,
              "rho > rho_sea: fan clamps at 1, ram follows local rho");
}

static void test_air_gate() {
    printf("== jetIntakeGate: the share of the rated intake the AIR allows ==\n");

    CHECK_REL(jetIntakeGate(RHO_SEA), 1.0, 1e-12, "the rating intake -> 1");
    CHECK_REL(jetIntakeGate(0.5 * RHO_SEA), 0.5, 1e-12, "half density -> half");
    CHECK_REL(jetIntakeGate(2.0 * RHO_SEA), 1.0, 1e-12, "denser clamps at 1");
    CHECK_REL(jetIntakeGate(0.0), 0.0, 0.0, "vacuum -> 0");
    CHECK_REL(jetIntakeGate(std::nan("")), 0.0, 0.0, "NaN density fails closed");
    CHECK_REL(jetIntakeGate(-1.0), 0.0, 0.0, "a negative density fails closed");

    /* This gate is ONLY the air now. It used to be jetAirGate, min'd with an oxidizer
       factor in [0,1] that also metered the fuel pump -- which is how a jet at 20 km
       "could not" burn the fuel its own pump was offering. Composition moved to
       jetBurnCap, where it bounds kilograms per second instead of a share. */
    const double gates[] = {0.1, 0.3, 0.75, 1.0};
    for(double g : gates) {
        CHECK_REL(T(0.0, g * RHO_SEA), jetIntakeGate(g * RHO_SEA) * T_FAN, 1e-12,
                  "the static thrust IS the gate times the rating");
    }
}

static void test_vacuum() {
    printf("== jetEngine: two kinds of vacuum ==\n");

    // No air at all: ZERO thrust at ANY speed.
    CHECK_REL(jetEngine(0.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A).thrust, 0.0, 0.0,
              "rho=0 at rest -> 0");
    CHECK_REL(jetEngine(300.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A).thrust, 0.0, 0.0,
              "rho=0 at speed -> 0 (no air, no thrust)");

    /* An air with nothing to burn against is the other vacuum, and the denser it is the
       more obviously an air-only gate would be wrong: Jool's datum is 0.188 kg/m3 of
       H2 and He -- H2 is the fuel, and there is no oxidizer here at all. A duct could
       push that air; a gas turbine cannot run on it. */
    CHECK_REL(jetEngine(0.0, 0.188, 0.0, T_FAN, M_F, V_E, A).thrust, 0.0, 0.0,
              "Jool's air at rest -> 0");
    CHECK_REL(jetEngine(V_E / 2.0, 0.188, 0.0, T_FAN, M_F, V_E, A).burn, 0.0, 0.0,
              "and it burns nothing at the ram peak either");
}

static void test_degenerate() {
    printf("== jetEngine: degenerate inputs ==\n");
    const double rho = RHO_SEA;

    // A negative speed reads as zero speed (the fan, and its bill).
    CHECK_REL(jetEngine(-50.0, rho, OX_AIR, T_FAN, M_F, V_E, A).thrust, T_FAN, 1e-12,
              "v<0 -> the fan rating");

    // No intake area: no ram term and no disc, so nothing flows at all.
    CHECK_REL(jetEngine(100.0, rho, OX_AIR, T_FAN, M_F, V_E, 0.0).thrust, 0.0, 0.0,
              "A=0 -> nothing flows: no fan area, no ram, no burn");

    // No fan, but moving: the intake still swallows air, so the hot stream still works.
    const JetS nofan = jetEngine(100.0, rho, OX_AIR, 0.0, M_F, V_E, A);
    CHECK_REL(nofan.thrust, 19359.4, 1e-5,
              "T_fan=0 at 100 m/s -> ram only (the disc is gone, the mouth is not)");
    CHECK_REL(nofan.burn, 0.145825, 1e-5,
              "and it spends only what that promise costs: 0.146 kg/s, not the pump");
    CHECK_REL(jetEngine(0.0, rho, OX_AIR, 0.0, M_F, V_E, A).thrust, 0.0, 0.0,
              "T_fan=0 at rest -> nothing moves through the engine");
}

/* What only a MIX can state (#630): kilograms of burnable oxidizer per kilogram of air,
   and now what that buys in kilograms per second of fuel rather than a factor in [0,1]. */
static void test_chemistry() {
    printf("== jetBurnCap: the chemistry caps the burn in kg/s, not in a factor ==\n");

    CHECK_REL(OX_AIR, 0.2314, 5e-5, "dry air is 23.1% oxidizer by mass");
    CHECK_REL(kStoichO2PerKgJetfuel, 3.48, 0.0, "kerosene needs 3.48 kg of O2 per kg of fuel");

    // Kerbin sea level, 275 m/s: 117.9 kg/s of captured air, and the fan is dragging
    // 138.9 kg/s through the engine. Its oxygen would pay for 9.2 kg/s of fuel, so the
    // 1.2 kg/s pump binds -- on an oxygen world the sky never notices the engine.
    const double sup = jetAirSupply(V_E / 2.0, RHO_SEA, T_FAN, A);
    CHECK_REL(sup, 138.9019, 1e-5, "air at the burner at the ram peak (fan-driven, not rho*A*v)");
    CHECK_REL(jetBurnCap(sup, OX_AIR, M_F), M_F, 0.0, "dry air: the pump binds, as it should");
    CHECK_REL(sup * OX_AIR / kStoichO2PerKgJetfuel, 9.236427, 1e-5,
              "what dry air's oxygen would pay for: 7.7x the pump");

    /* Eve's datum: 3.552 kg/m3 of CO2 with a 0.096 % O2 trace (share 0.000959), the
       same mix as Mars and Duna. At 275 m/s the intake swallows 341.9 kg/s of it, whose
       oxygen pays for 0.094 kg/s of fuel -- a thirteenth of the pump. The old gate
       reduced the machine to 1.2% of its rating by a factor; this says the same thing in
       the only unit that matters, and it is what then limits the fan's power too. */
    const double EVE_RHO = 3.552, EVE_OX = 0.000959;
    CHECK_REL(jetBurnCap(jetAirSupply(V_E / 2.0, EVE_RHO, T_FAN, A), EVE_OX, M_F),
              0.094213, 1e-5, "Eve's trace oxygen buys 0.094 kg/s of fuel");

    // Mars: same trace, thin air. The fan there is 1142 N, not 90 kN, so it drags
    // 1.76 kg/s through the burner instead of 138.9 -- which is what the cap is priced on.
    const double marsRated = T_FAN * jetIntakeGate(0.015547);
    CHECK_REL(jetAirSupply(V_E / 2.0, 0.015547, marsRated, A), 1.762864, 1e-5,
              "thin air moves the throughput the thin fan can actually push");
    CHECK_REL(jetBurnCap(jetAirSupply(V_E / 2.0, 0.015547, marsRated, A), 0.000959, M_F),
              0.000485801, 1e-6, "Mars's datum buys 0.5 g/s");
    CHECK_REL(jetBurnCap(1000.0, 0.0, M_F), 0.0, 0.0, "no oxidizer buys nothing");
    CHECK_REL(jetBurnCap(1000.0, std::nan(""), M_F), 0.0, 0.0, "a NaN share fails closed");
    CHECK_REL(jetBurnCap(1000.0, -1.0, M_F), 0.0, 0.0, "a negative share fails closed");
    CHECK_REL(jetBurnCap(0.0, OX_AIR, M_F), 0.0, 0.0, "no air, no burn");
    CHECK_REL(jetBurnCap(1000.0, OX_AIR, 0.0), 0.0, 0.0, "a shut pump is a shut pump");

    /* An oxygen-rich sky does not buy more than the pump. */
    CHECK_REL(jetBurnCap(1000.0, 1.0, M_F), M_F, 0.0,
              "pure oxygen does not open a valve the pump was built to close");
}

/* THE FAN IS BOUGHT, NOT AUTHORED (#1233). The disc's power and the nozzle's heat come
   out of one burn, and where the burn is short the FAN is what gets smaller. */
static void test_the_fan_is_paid_for() {
    printf("== the same burn pays for the disc and the nozzle (#1233) ==\n");

    /* Energy identity, unclamped state at sea level: the fuel's heat equals the disc's
       power plus the kinetic energy handed to the captured air. Exact, to the last bit
       the compiler keeps -- this is the whole-engine budget the fan used to sit outside. */
    const double v = V_E / 2.0;
    const JetS e = jetEngine(v, RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A);
    const double air = RHO_SEA * A * v;
    const double exit = jetRamExitSpeed(v, air, e.burn - T_FAN * jetDiscSpeed(T_FAN, RHO_SEA, A)
                                            / kJetFuelHeatPerKg, V_E);
    const double flowKE = 0.5 * air * (exit * exit - v * v);
    CHECK_REL(kJetFuelHeatPerKg * e.burn,
              T_FAN * jetDiscSpeed(T_FAN, RHO_SEA, A) + flowKE, 1e-9,
              "Q * burn == disc power + the energy handed to the captured air");

    /* Eve, where the sky rather than the pump decides: 0.094 kg/s is 4.05 MW, and the
       90 kN disc wants 17.1 MW in her dense air. So the fan comes down to what 4.05 MW
       buys -- 34.4 kN, not the 90 the plate says and not the 3.3 kN the old
       oxidizer-factor law reported (which got there by shrinking the fan AND the burn
       AND the exhaust with one number). */
    const double EVE_RHO = 3.552, EVE_OX = 0.000959;
    const JetS ev = jetEngine(V_E / 2.0, EVE_RHO, EVE_OX, T_FAN, M_F, V_E, A);
    CHECK_REL(ev.thrust, 34427.96, 1e-5, "Eve's datum: a fuel-powered fan, 34.4 kN");
    CHECK_REL(ev.burn, 0.094213, 1e-5, "spent to the last joule on the disc");
    CHECK_TRUE(ev.thrust < 0.4 * T_FAN,
               "a trace-oxygen sky is a weak engine, not a rated one with a sticker on it");
    CHECK_REL(jetEngine(V_E / 2.0, 0.015547, EVE_OX, T_FAN, M_F, V_E, A).thrust, 168.09, 1e-3,
              "and Mars's datum is the same chemistry in a hundred times less air");
}

/* THE PARTS CAN AFFORD THEIR OWN PLATES. The promise's price is m_air (v_e^2 - v^2) /
   (2Q); what the sky and the pump allow is jetBurnCap. Every shipped jet has to clear
   that or its authored v_e is a fiction -- which is the check the catalogue runs. */
static void test_the_promise_is_affordable() {
    printf("== the promise's price against what the sky pays ==\n");

    const double v_ask = V_E / std::sqrt(3.0);   // where m_air (v_e^2 - v^2) tops out
    CHECK_REL(v_ask * (V_E * V_E - v_ask * v_ask), 6.404e7, 1e-3,
              "the hardest momentum ask a v_e 550 ram term makes");
    CHECK_REL(2.0 * kJetFuelHeatPerKg * M_F / (kAirSeaLevelDensity * A), 2.407e8, 1e-3,
              "what its fuel line pays for, density cancelled out");

    /* Margin over a density x speed sweep, dry air: 1.30x for the 2 m jet, 1.32x for
       the 0.5 m one, and the worst point is the DENSE rows -- above the rating the fan
       gate clamps while the intake keeps swallowing. Kerbin at rest itself sits at 1.55x. */
    const double rhos[] = {0.01, 0.0948, 0.502, RHO_SEA, 2.0 * RHO_SEA, 3.552};
    double worst = 1e30, worstRho = 0, worstV = 0;
    for(double rho : rhos) {
        const double fanRated = T_FAN * jetIntakeGate(rho);
        for(int i = 1; i <= 220; i++) {
            const double v = V_E * i / 220.0;
            const double cap = jetBurnCap(jetAirSupply(v, rho, fanRated, A), OX_AIR, M_F);
            const double ask = rho * A * v * (V_E * V_E - v * v) / (2.0 * kJetFuelHeatPerKg);
            if(ask <= 0.0) { continue; }
            if(cap / ask < worst) { worst = cap / ask; worstRho = rho; worstV = v; }
        }
    }
    CHECK_REL(worst, 1.2963, 1e-3, "the 2 m jet's tightest margin in dry air over the sweep");
    CHECK_REL(worstRho, 3.552, 0.0, "set by the densest row, not the thinnest");
    CHECK_REL(worstV, 317.5, 1.0, "at the hardest ask, v = v_e/sqrt(3)");
    CHECK_TRUE(worst > 1.05, "and no shipped jet is capped by its own fuel line in dry air");

    /* The other direction, and it is the interesting one for #1206: a v_e 3200 nozzle
       on a 3 kg/s pump CANNOT fill its promise at sea level -- 428.75 kg/s of captured
       air asks 46 kg/s of fuel to reach 3200 m/s. So it burns what it has and throws
       1265 m/s. A fast jet is a high-altitude machine, and the law says so without a
       flag or a special case. */
    const double supSea = jetAirSupply(1000.0, RHO_SEA, 3000.0, A);
    CHECK_REL(supSea, 428.75, 1e-9, "at 1000 m/s the mouth, not a 3 kN fan, sets the flow");
    CHECK_REL(428.75 * (3200.0 * 3200.0 - 1000.0 * 1000.0) / (2.0 * kJetFuelHeatPerKg),
              46.0657, 1e-5, "what a v_e 3200 nozzle asks for at sea level and 1000 m/s");
    CHECK_REL(jetFuelAffordedExitSpeed(1000.0, 428.75, 3.0), 1265.6023, 1e-5,
              "what 3.0 kg/s actually affords there: 1266 m/s, not 3200");
}

/* Teeth. #1206's real experiment part (tmp/n1099/parts_scram2.json: a 0.5 m jet
   re-declared -- fan 3000, 3.0 kg/s of jetfuel, v_e 3200, A 0.35, 43.75 kg). Under the
   law before #1206 it made 64.7 kN at 2400 m/s in 20 km air on a 0.232 kg/s burn: 178 MW
   handed to the flow out of a 10 MW fire. It now makes 49.8 kN -- and the energy books
   close: 3.0 kg/s is 129 MW, the flow takes 129 MW, and 119.5 MW of it arrives at the
   hull as useful power at that speed. The free ride is gone; the engine is real. */
static void test_the_fast_jet_is_a_machine() {
    printf("== #1206's fast part: a machine, not a free ride ==\n");

    const double VE32 = 3200.0, FAN3 = 3000.0, M3 = 3.0, RHO20 = 0.0948;
    const double air = RHO20 * A * 2400.0;
    const JetS e = jetEngine(2400.0, RHO20, OX_AIR, FAN3, M3, VE32, A);
    CHECK_REL(e.burn, 3.0, 0.0, "the pump binds at 3.0 kg/s (the oxygen would pay for 5.3)");
    CHECK_REL(e.thrust, 49805.09, 1e-5, "49.8 kN at Mach 7 in 20 km air");
    CHECK_REL(jetRamExitSpeed(2400.0, air, e.burn, VE32), 2999.98, 1e-5,
              "and the burn affords 3000 m/s of exhaust, just short of the 3200 plate");

    /* Energy read two ways, the second catching a factor-of-2 slip: the ram term is the
       fuel's heat over the mean of in and out speeds. */
    CHECK_REL(0.5 * air * (2999.98 * 2999.98 - 2400.0 * 2400.0) / 1e6, 129.0, 1e-3,
              "the flow takes exactly the fire's 129 MW");
    CHECK_REL(e.thrust * 2400.0 / 1e6, 119.5, 1e-2,
              "of which 119.5 MW arrives as thrust power at the hull");

    /* What the old law printed, kept as a pin so nobody reintroduces it. The old drain
       was the pump TIMES the same gate (0.0774 * 3.0 = 0.232 kg/s, 10 MW), and the old
       exhaust term used THAT kg/s -- while the ram term billed all 79.6 kg/s of air
       against the full 3200 m/s plate. */
    const double d20 = jetIntakeGate(RHO20);
    const double old = FAN3 * d20 + M3 * VE32 * d20 + air * (VE32 - 2400.0);
    CHECK_REL(old, 64680.6857, 1e-6, "the pre-#1206 number: 64.7 kN on 10 MW of fuel");
    CHECK_TRUE(e.thrust < old, "and today's engine is the honest, smaller one");

    /* A capped engine still behaves like an engine, and its curve has a SHAPE. Below
       ~1600 m/s in 20 km air the intake is the constraint -- more speed swallows more
       air, and the 3 kg/s line cannot fill a v_e 3200 nozzle at 26-40 kg/s of it anyway
       -- so thrust climbs; from there the shrinking (v_e - v) wins and it falls, to the
       fan alone at v = v_e and nothing beyond. */
    double prev = 1e30;
    for(double v : {1600.0, 2000.0, 2400.0, 2800.0, 3100.0}) {
        const double t = jetEngine(v, RHO20, OX_AIR, FAN3, M3, VE32, A).thrust;
        CHECK_TRUE(t < prev, "past its peak a fast jet still falls off with speed");
        prev = t;
    }
    CHECK_TRUE(jetEngine(1600.0, RHO20, OX_AIR, FAN3, M3, VE32, A).thrust
                   > jetEngine(800.0, RHO20, OX_AIR, FAN3, M3, VE32, A).thrust,
               "and below it the intake, not the nozzle, is the limit");
    CHECK_REL(jetEngine(VE32, RHO20, OX_AIR, FAN3, M3, VE32, A).thrust, FAN3 * d20, 1e-9,
              "v = v_e -> the fan only, as it must");

    /* The lever is the fuel line, not the nozzle: the same v_e 550 on a 1 g/s burner
       keeps its fan and loses its ram. */
    const JetS thin = jetEngine(V_E / 2.0, RHO_SEA, OX_AIR, T_FAN, 1e-3, V_E, A);
    CHECK_TRUE(thin.thrust < 0.5 * T(V_E / 2.0, RHO_SEA),
               "a jet on 1 g/s of fuel keeps its fan and loses its ram");

    /* And with the pump shut there is no engine at all: not the fan's rating, because
       the fan has no power source. The pre-#1206 law kept 90 kN and 32.4 kN of ram here. */
    CHECK_REL(jetEngine(V_E / 2.0, RHO_SEA, OX_AIR, T_FAN, 0.0, V_E, A).thrust, 0.0, 0.0,
              "no fuel, no fan: the rating needs a burner behind it");
}

/* THE AIR HANDS THE ENGINE ITS OWN COMPOSITION (#637). Every production caller holds the
   DragAtmosphere its density came from, so the engine takes that rather than a share
   fetched separately off the body -- two fetches can name two different worlds, and
   nothing in the type used to notice. */
static void test_the_air_carries_its_own_composition() {
    printf("== the profile carries the oxidizer share the engine needs (#637) ==\n");

    DragAtmosphere air { RHO_SEA, 5500.0, 70000.0 };
    CHECK_REL(air.oxidizer_share, OX_AIR, 0.0,
              "a hand-built profile defaults to dry air, exactly as its molar_mass and "
              "gamma do: a fixture must not be a silently dead engine");

    const double mars_rho = 0.015547, mars_ox = 0.000959;
    DragAtmosphere mars { mars_rho, 20000.0, 80000.0 };
    mars.oxidizer_share = mars_ox;

    /* The door is the same engine, bit for bit -- a second arithmetic here would be the
       drift this refactor exists to prevent. */
    CHECK_REL(jetEngine(V_E / 2.0, mars_rho, mars, T_FAN, M_F, V_E, A).thrust,
              jetEngine(V_E / 2.0, mars_rho, mars_ox, T_FAN, M_F, V_E, A).thrust, 0.0,
              "jetEngine(profile, ...) IS jetEngine(share, ...)");
    CHECK_REL(jetEngine(V_E / 2.0, mars_rho, mars, T_FAN, M_F, V_E, A).burn,
              jetEngine(V_E / 2.0, mars_rho, mars_ox, T_FAN, M_F, V_E, A).burn, 0.0,
              "and the drain it reports travels through the same door");

    /* It reads the FIELD, not a constant: one density, two skies. Mars's sky is a CO2
       mix with a 0.096 % O2 trace, so the same 0.0155 kg/m3 engines 9.25x harder with
       dry air in the duct than with the gas actually there -- not 241x, which is the
       share ratio, because at Mars density the fan is fuel-starved too, so most of dry
       air's advantage arrives as fan power rather than burner heat. */
    CHECK_TRUE(jetEngine(V_E / 2.0, mars_rho, air, T_FAN, M_F, V_E, A).thrust
                   > 5.0 * jetEngine(V_E / 2.0, mars_rho, mars, T_FAN, M_F, V_E, A).thrust,
               "the same 0.0155 kg/m3 engines two ways once the composition travels with "
               "the air -- a door that ignored the share would read dry air everywhere");

    /* A sky with nothing to burn against stays dead through the door, thrust AND burn. */
    DragAtmosphere jool { 0.188, 60000.0, 100000.0 };
    jool.oxidizer_share = 0.0;
    CHECK_REL(jetEngine(V_E / 2.0, 0.188, jool, T_FAN, M_F, V_E, A).thrust, 0.0, 0.0,
              "0.188 kg/m3 of H2 and He is a hole in the hull, not an engine");
    CHECK_REL(jetEngine(V_E / 2.0, 0.188, jool, T_FAN, M_F, V_E, A).burn, 0.0, 0.0,
              "and it hands back no burn either");
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
    test_chemistry();
    printf("\n");
    test_the_fan_is_paid_for();
    printf("\n");
    test_the_promise_is_affordable();
    printf("\n");
    test_the_fast_jet_is_a_machine();
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
