//
// The air-breathing engine law (src/drag.h jetEngine, header-only pure math): ONE
// stream of air through ONE mouth, at ONE wake speed.
//
//   mdot = 1/2 rho A (v + w)      T = mdot (w - v)      burn = T (v + w) / 2 Q
//
// so T = 1/2 rho A (w^2 - v^2), and the wake's kinetic-energy rate is exactly
// Q * burn. w is the smallest of four ceilings, each an authored number in its own
// currency (#1206, and the pass that found the two-stream law's 2x energy leak):
//   NOZZLE   w <= v_e                          exhaust_velocity
//   MACHINE  w <= sqrt(v^2 + 2 T_fan d / rho A)   jet_fan_thrust, gate d
//   SKY      w <= sqrt(v^2 + Q share / 3.48)   the body's oxidizer
//   FUEL     1/2 rho A (w - v)(w + v)^2 / 2 <= Q * pump
// Every check here either pins a row of that arithmetic or names which ceiling
// binds, so a law or authoring change cannot quietly move the shipped envelope.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>

#include "drag.h"
#include "tchecks.h"

static const double Q = kJetFuelHeatPerKg;
static const double RHO0 = kAirSeaLevelDensity;
static const double OX_AIR = dryAirOxidizerShare();
/* Exactly as res/data/parts.json authors the 2 m jet: 0.35 m2 of mouth (the intake the
   part always declared, 11% of its frontal disc), v_e 700 m/s, pump 1.4 kg/s, and a fan
   rating derived from that CLEANED intake so the part fills its own nozzle at the
   rating air rather than being bound by its own machine. */
static const double A = 0.35, V_E = 700.0, FAN = 105043.75, PUMP = 1.4;
/* The 0.5 m shipped jet (jet_r0.25h1): 0.02 m2, 6.0 kN, 0.08 kg/s. */
static const double AS = 0.02, FAN_S = 6002.5, PUMP_S = 0.08;

/* The stream's kinetic-energy rate, engine frame: 1/2 mdot (w^2 - v^2). */
static double flowKE(double v, double w, double rho, double area = A)
{
    return 0.25 * rho * area * (w - v) * (w + v) * (w + v);
}

static void test_static() {
    printf("== one stream at rest: the pad, and what the pad costs ==\n");

    /* At rest the disc swallows 1/2 rho A w, so 700 m/s through 0.35 m2 at datum is
       150 kg/s of air against 0.86 kg/s of fuel. The engine is mostly air, and the
       whole of its thrust is that air's momentum change -- there is no second,
       fuel-carried stream to bill separately, which is what the previous law did and
       why it printed 87.7 MW of wake from 42.5 MW of fuel at Mach 0.85. */
    const JetS e = jetEngine(0.0, RHO0, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(e.thrust, 105043.750000, 1e-9, "105 kN on the pad");
    CHECK_REL(e.wake, V_E, 1e-12, "the shipped part fills its nozzle at rest");
    CHECK_TRUE(e.bound == kJetBoundNozzle, "and says so: the nozzle is what binds it");
    CHECK_REL(e.air, 150.0625, 1e-12, "the air the duct moves, from the same call");
    CHECK_REL(e.burn, 0.855007267, 1e-9, "0.86 kg/s to hold it, 61% of the pump");
    CHECK_REL(jetAirRate(0.0, e.wake, RHO0, A), 150.0625, 1e-12, "1/2 rho A (v + w)");
    CHECK_REL(e.thrust, jetAirRate(0.0, e.wake, RHO0, A) * (e.wake - 0.0), 1e-12,
              "T = mdot (w - v)");
    CHECK_REL(flowKE(0.0, e.wake, RHO0), Q * e.burn, 1e-9, "the fuel buys the whole wake");
    CHECK_TRUE(e.burn * 175.0 < jetAirRate(0.0, e.wake, RHO0, A),
               "air outweighs fuel 175:1 -- jets are not fuel-powered, they are air-powered");
    printf("note: 150 kg/s of air against %.2f kg/s of fuel; the pad needs %.1f MW\n",
           e.burn, Q * e.burn / 1e6);
}

static void test_falls_with_speed() {
    printf("== thrust falls with speed; no ram hump ==\n");

    struct Row { double v, T, burn; };
    const Row rows[] = {
        { 0.0, 105043.750000, 0.855007267 },
        { 100.0, 102900.000000, 0.957209302 },
        { 275.0, 88831.640625, 1.007102902 },  // the cruise the XR-2 flies
        { 400.0, 70743.750000, 0.904861919 },
        { 550.0, 40195.312500, 0.584234193 },
        { 699.0, 299.910625, 0.00487877866 },  // 1 m/s under the wall
    };
    double prev = 1e30;
    for(const Row &r : rows) {
        const JetS e = jetEngine(r.v, RHO0, OX_AIR, FAN, PUMP, V_E, A);
        CHECK_REL(e.thrust, r.T, 1e-9, "datum thrust");
        CHECK_REL(e.burn, r.burn, 1e-9, "datum fuel rate");
        CHECK_REL(e.wake, V_E, 1e-12, "nozzle-bound across the whole band");
        CHECK_TRUE(e.bound == kJetBoundNozzle, "and names the ceiling that binds it");
        CHECK_REL(jetAirRate(r.v, e.wake, RHO0, A) * (e.wake - r.v), e.thrust, 1e-12,
                  "T = mdot (w - v)");
        CHECK_REL(Q * e.burn, jetStreamPower(e.thrust, r.v, e.wake), 1e-9, "burn = T(v+w)/2Q");
        CHECK_TRUE(e.thrust < prev, "thrust only ever falls with speed");
        prev = e.thrust;
    }
    /* The old law's peak sat at v_e/2 and its tail ran on past v_e -- a fan thrust
       that never stopped, plus a ram term that clamped only when it went negative.
       Here the wall is v_e itself: at 700 m/s the wake and the hull move together,
       so nothing is thrown backwards and nothing is burnt. The law still answers the
       air it swallows there (#1207): a Mach 3 hull pushes 1286 kg/s through a duct
       that can do nothing with it, which is the hole-in-the-hull bill that #1207 is
       about and the reason the verdict, not just the zero, has to be readable. */
    const JetS wall = jetEngine(700.0, RHO0, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(wall.thrust, 0.0, 0.0, "v = v_e: the wake cannot beat the hull");
    CHECK_TRUE(wall.bound == kJetBoundCaught, "and says the hull caught the exhaust");
    const JetS hypersonic = jetEngine(3000.0, RHO0, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(hypersonic.thrust, 0.0, 0.0, "Mach 3 on dry air: a 700 m/s nozzle makes nothing");
    CHECK_TRUE(hypersonic.bound == kJetBoundCaught, "same verdict, three times the wall");
    CHECK_REL(hypersonic.air, 1.225 * A * 3000.0, 1e-12, "rho A v of air through a shut duct");
    CHECK_REL(hypersonic.wake, 3000.0, 1e-12, "leaving at the speed it arrived");
    CHECK_REL(jetEngine(900.0, RHO0, OX_AIR, FAN, PUMP, V_E, A).burn, 0.0, 0.0,
              "and burns nothing doing it");
    CHECK_TRUE(jetEngine(100.0, 0.0, OX_AIR, FAN, PUMP, V_E, A).bound == kJetBoundNoAir,
               "in vacuum the answer is that there is nothing to flow");
    printf("note: 105 kN pad, 88.8 kN at 275 m/s, 40.2 kN at 550, 0 at 700\n");
}

static void test_ceilings() {
    printf("== the four ceilings, one state each ==\n");

    // NOZZLE. At the datum the shipped part is nozzle-bound at every speed it can
    // reach; the other three all leave more room than v_e.
    const double d0 = jetIntakeGate(RHO0);
    CHECK_REL(std::sqrt(0.0 * 0.0 + 2.0 * FAN * d0 / (RHO0 * A)), V_E, 1e-12,
              "the machine is authored to reach exactly v_e at the rating air");
    CHECK_REL(jetChemistryWakeSpeed(0.0, OX_AIR), 2391.371560, 1e-9,
              "dry air lets the wake reach 2391 m/s on chemistry alone");
    CHECK_REL(jetPumpWakeSpeed(0.0, RHO0, A, PUMP), 825.057310, 1e-9,
              "and the fuel line 825 m/s");

    // MACHINE. Denser than the rating air, the mouth could throw more than the fan
    // is rated for, so the wake is clamped BELOW v_e and the thrust stops at the
    // rating -- the same force at every speed until the nozzle wall.
    const double rho2 = 2.0 * RHO0;
    const JetS deep = jetEngine(0.0, rho2, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(deep.wake, 494.974747, 1e-9, "the machine caps the wake under v_e");
    CHECK_TRUE(deep.bound == kJetBoundMachine, "and names the fan as the ceiling");
    CHECK_REL(deep.thrust, FAN, 1e-12, "at exactly the rating");
    CHECK_TRUE(deep.thrust < 0.5 * rho2 * A * V_E * V_E,
               "dense air does not double the thrust for free");
    // A fast nozzle in thin air: machine-bound across the band, holding its rating
    // times the gate while the fuel bill climbs.
    const double rhoUp = 0.0948;
    const double wantMachine = 200000.0 * jetIntakeGate(rhoUp);
    CHECK_REL(wantMachine, 15477.551020, 1e-9, "rating x gate");
    for(double v : {800.0, 1600.0, 2400.0, 3000.0}) {
        const JetS m = jetEngine(v, rhoUp, OX_AIR, 200000.0, 3.0, 3200.0, A);
        CHECK_REL(m.thrust, wantMachine, 1e-9, "machine-bound thrust is speed-independent");
        CHECK_TRUE(m.bound == kJetBoundMachine, "at every speed in this thin sky");
        CHECK_REL(m.wake, std::sqrt(v * v + 2.0 * wantMachine / (rhoUp * A)), 1e-9,
                  "the wake that holds it");
    }

    // SKY. Eve's 0.096% oxidizer puts the ceiling at 153.95 m/s of wake at rest.
    const double EVE_OX = 0.000959;
    CHECK_REL(jetChemistryWakeSpeed(0.0, EVE_OX), 153.946177, 1e-8, "Eve's trace");
    const JetS eve = jetEngine(275.0, 3.552, EVE_OX, FAN, PUMP, V_E, A);
    CHECK_REL(eve.wake, 315.157778, 1e-8, "Eve at 275 m/s");
    CHECK_TRUE(eve.bound == kJetBoundSky, "and names the air's oxidizer as the ceiling");
    CHECK_REL(eve.thrust, 14731.562759, 1e-9, "14.7 kN, not 88.8 kN: the sky binds");
    CHECK_REL(eve.burn, 0.1010923994, 1e-9, "and only 0.101 kg/s of it");
    // Under the sky ceiling T = rho A Q share/3.48, which has no v in it at all.
    const JetS eveRest = jetEngine(0.0, 3.552, EVE_OX, FAN, PUMP, V_E, A);
    CHECK_REL(eveRest.thrust, eve.thrust, 1e-12, "the same thrust at rest");
    CHECK_REL(eveRest.burn, 0.0263705554, 1e-8, "a quarter of the fuel, because the air "
              "arrives slower and the same wake costs less to make");
    // A sky with no oxidizer at all throws nothing backwards, however thick.
    const JetS jool = jetEngine(200.0, 0.188, 0.0, FAN, PUMP, V_E, A);
    CHECK_REL(jool.thrust, 0.0, 0.0, "Jool: hydrogen, nothing to burn it against");
    CHECK_TRUE(jool.bound == kJetBoundSky, "and the card can say so: the sky, not the machine");
    CHECK_REL(jool.air, 0.5 * 0.188 * A * (200.0 + 200.0), 1e-12,
              "the mouth still moves air it cannot burn");
    CHECK_REL(jetEngine(100.0, 65.0, 0.0, FAN, PUMP, V_E, A).thrust, 0.0, 0.0,
              "Venus: 65 kg/m3 and still nothing");
    CHECK_REL(jetChemistryWakeSpeed(200.0, 0.0), 200.0, 0.0,
              "no oxidizer leaves the wake at the hull speed");
    // Mars: the same trace in a 0.0155 kg/m3 atmosphere, so the force is tiny even
    // though the ceiling speed is respectable.
    const JetS mars = jetEngine(200.0, 0.015547, EVE_OX, FAN, PUMP, V_E, A);
    CHECK_REL(mars.wake, 252.387451, 1e-8, "Mars at 200 m/s");
    CHECK_REL(mars.thrust, 64.479619, 1e-6, "64 N -- an air-breathing engine needs an "
              "atmosphere, not merely a sky");

    // FUEL LINE. The pump's ceiling is the cube-root law; it is the only one that
    // needs a solve, and the solve is exact.
    CHECK_REL(jetPumpWakeSpeed(0.0, RHO0, A, PUMP), std::cbrt(4.0 * Q * PUMP / (RHO0 * A)),
              1e-12, "closed form at rest");
    for(double v : {100.0, 300.0, 650.0}) {
        const double w = jetPumpWakeSpeed(v, RHO0, A, PUMP);
        CHECK_REL(flowKE(v, w, RHO0), Q * PUMP, 1e-12, "the fuel line spends exactly its rating");
    }
    CHECK_REL(jetPumpWakeSpeed(100.0, RHO0, A, PUMP), 797.398916, 1e-9, "at 100 m/s");
    CHECK_REL(jetPumpWakeSpeed(650.0, RHO0, A, PUMP), 887.566222, 1e-9,
              "at 650 m/s the line can only raise the wake further");
    CHECK_REL(jetPumpWakeSpeed(0.0, RHO0, A, 0.0), 0.0, 0.0, "no fuel, no wake");
    CHECK_REL(jetPumpWakeSpeed(0.0, 0.0, A, PUMP), 0.0, 0.0, "no air, no wake");
    // The shipped 2 m pump outruns its nozzle, so the fuel line is never the story
    // in a shipped part; starve the small part's pump and it becomes the whole story.
    CHECK_TRUE(jetPumpWakeSpeed(0.0, RHO0, A, PUMP) > V_E, "the shipped pump outruns v_e");
    const JetS starved = jetEngine(0.0, RHO0, OX_AIR, FAN_S, 0.02, V_E, AS);
    CHECK_REL(starved.wake, 519.753536, 1e-9, "a starved pump pulls the wake under v_e");
    CHECK_REL(starved.thrust, 3309.260795, 1e-9, "and the thrust with it");
    CHECK_TRUE(starved.bound == kJetBoundFuelLine, "and names the fuel line, not the fan");
}

static void test_gate_and_density() {
    printf("== the intake gate: a capacity on the machine, not on the air ==\n");

    CHECK_REL(jetIntakeGate(RHO0), 1.0, 0.0, "the rating air is 1.225, absolute");
    CHECK_REL(jetIntakeGate(2.0 * RHO0), 1.0, 0.0, "never above 1");
    CHECK_REL(jetIntakeGate(0.0948), 0.0773877551, 1e-8, "20 km");
    CHECK_REL(jetIntakeGate(0.0), 0.0, 0.0, "vacuum");
    CHECK_REL(jetIntakeGate(-1.0), 0.0, 0.0, "garbage density");

    /* Thin air is not a law change, it is a scaled-down machine: at 20 km the pad
       thrust is the rating times the gate, to the digit. #630 measured that a
       compressor's delivery tracks the mass arriving at the mouth. */
    const JetS high = jetEngine(0.0, 0.0948, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(high.thrust, FAN * jetIntakeGate(0.0948), 1e-12, "rating x gate at rest");
    CHECK_REL(high.thrust, 8129.100000, 1e-9, "8.1 kN at 20 km");
    CHECK_REL(high.wake, V_E, 1e-12, "thin air still lets the nozzle bind first");
    CHECK_REL(high.burn, 0.0661670930, 1e-8, "and 0.066 kg/s to hold it");
    CHECK_REL(jetEngine(275.0, 0.0948, OX_AIR, FAN, PUMP, V_E, A).thrust, 6874.481250, 1e-9,
              "20 km at 275 m/s");
    CHECK_REL(jetEngine(550.0, 0.0948, OX_AIR, FAN, PUMP, V_E, A).thrust, 3110.625000, 1e-9,
              "20 km at 550 m/s");
    /* Half the rating air gives half the thrust, and the whole answer is still
       1/2 rho A v_e^2 -- the law has no separate "density exponent" to tune. */
    CHECK_REL(jetEngine(0.0, 0.5 * RHO0, OX_AIR, FAN, PUMP, V_E, A).thrust, 52521.875000, 1e-9,
              "half density, half thrust");
    CHECK_REL(jetEngine(0.0, 0.5 * RHO0, OX_AIR, FAN, PUMP, V_E, A).thrust,
              0.5 * (0.5 * RHO0) * A * V_E * V_E, 1e-12, "1/2 rho A v_e^2");
    // A mouth is not optional.
    CHECK_REL(jetEngine(0.0, RHO0, OX_AIR, FAN, PUMP, V_E, 0.0).thrust, 0.0, 0.0,
              "no intake area, no engine");
    CHECK_REL(jetEngine(0.0, RHO0, OX_AIR, FAN, PUMP, 0.0, A).thrust, 0.0, 0.0,
              "no nozzle, no engine");
}

static void test_chemistry_is_still_a_gate() {
    printf("== the stoichiometric burn cap, where it still bites ==\n");

    const double air = jetAirRate(275.0, V_E, RHO0, A);
    CHECK_REL(air, 209.015625, 1e-12, "the datum stream at 275 m/s");
    // Under the new law the stoichiometry no longer sets thrust -- the wake does --
    // but it is still the number the vehicle's drain cannot exceed, and it is what
    // makes a trace-oxidizer sky burn thin.
    CHECK_REL(jetBurnCap(air, OX_AIR, 1000.0), air * OX_AIR / kStoichO2PerKgJetfuel, 1e-12,
              "air * share / 3.48 kg/s");
    CHECK_REL(jetBurnCap(air, OX_AIR, 1000.0), 13.898707704, 1e-8,
              "dry air at datum could burn 13.9 kg/s if the pump offered it");
    CHECK_REL(jetBurnCap(air, OX_AIR, PUMP), PUMP, 1e-12,
              "and it does not: on Kerbin the pump binds, the air never notices");
    CHECK_REL(jetBurnCap(air, 0.0, PUMP), 0.0, 0.0, "a dead sky burns nothing");
    CHECK_REL(jetBurnCap(air, 1.0, 0.4), 0.4, 0.0, "the pump still has the last word");
    // And on a sky-bound world the two gates meet: jetChemistryWakeSpeed is exactly
    // where the stream's power ask equals what the swallowed air's oxygen can burn, so
    // the burn the law lands on IS the stoichiometric ceiling, to the digit.
    const double airEve = jetAirRate(275.0, 315.157778, 3.552, A);
    CHECK_REL(airEve, 366.842075, 1e-9, "Eve's mouth swallows 367 kg/s of CO2");
    CHECK_REL(jetBurnCap(airEve, 0.000959, 1000.0), 0.1010923994, 1e-8,
              "whose oxygen pays for 0.101 kg/s of fuel");
    CHECK_REL(jetEngine(275.0, 3.552, 0.000959, FAN, PUMP, V_E, A).burn,
              jetBurnCap(airEve, 0.000959, 1000.0), 1e-9,
              "and that is exactly what the engine burns -- the wake ceiling and the "
              "stoichiometry are the same line, asked in two currencies");
}

static void test_energy_identity() {
    printf("== the law's ledger, over the whole argument space ==\n");

    const double rhos[] = {0.0001, 0.015547, 0.0948, 0.502, 1.225, 2.45, 3.552, 65.0};
    const double oxes[] = {OX_AIR, 0.000959, 0.015547, 0.0};
    int live = 0, dead = 0;
    for(double rho : rhos) for(double ox : oxes) for(int i = 0; i <= 200; i++) {
        const double v = 900.0 * i / 200.0;
        const JetS e = jetEngine(v, rho, ox, FAN, PUMP, V_E, A);
        if(e.thrust <= 0.0) {
            dead++;
            CHECK_REL(e.burn, 0.0, 0.0, "a thrustless engine burns nothing");
            CHECK_TRUE(e.wake <= v + 1e-9, "a thrustless engine throws nothing backwards");
            continue;
        }
        live++;
        /* THE check. Every watt of fuel shows up as wake, to floating point: the
           engine frame's 1/2 mdot (w^2 - v^2) equals Q * burn exactly. The previous
           law's version of this line summed two fictitious streams and passed while
           printing twice the energy it was handed. */
        CHECK_REL(flowKE(v, e.wake, rho), Q * e.burn, 1e-9, "energy closes exactly");
        CHECK_REL(jetAirRate(v, e.wake, rho, A) * (e.wake - v), e.thrust, 1e-12,
                  "momentum closes");
        CHECK_TRUE(e.wake > v, "the wake leaves backwards");
        CHECK_TRUE(e.wake <= V_E + 1e-9, "the nozzle is a ceiling");
        CHECK_TRUE(e.thrust <= FAN * jetIntakeGate(rho) * (1.0 + 1e-9),
                   "the machine is a ceiling");
        CHECK_TRUE(v < V_E, "and thrust needs v < v_e");
    }
    printf("note: %d live states, %d thrustless; every live one closes its ledger\n",
           live, dead);
}

static void test_shipped_parts_fill_their_nozzles() {
    printf("== both shipped jets: authored to fill the nozzle, gated by the sky ==\n");

    // gen_parts.py derives jet_fan_thrust from the CLEANED intake so the machine's
    // own ceiling lands on v_e rather than quietly under it.
    CHECK_REL(FAN, 0.5 * RHO0 * A * V_E * V_E, 1e-12, "the 2 m rating");
    CHECK_REL(FAN_S, 0.5 * RHO0 * AS * V_E * V_E, 1e-12, "the 0.5 m rating");
    CHECK_REL(jetEngine(0.0, RHO0, OX_AIR, FAN_S, PUMP_S, V_E, AS).wake, V_E, 1e-12,
              "the 0.5 m jet fills its nozzle at rest");
    CHECK_REL(jetEngine(0.0, RHO0, OX_AIR, FAN_S, PUMP_S, V_E, AS).thrust, 6002.5, 1e-9,
              "6.0 kN, mass 43.75 kg -> 137 N/kg");
    CHECK_REL(jetEngine(0.0, RHO0, OX_AIR, FAN_S, PUMP_S, V_E, AS).burn, 0.0488575581, 1e-8,
              "61% of its pump at rest");
    CHECK_REL(jetEngine(275.0, RHO0, OX_AIR, FAN_S, PUMP_S, V_E, AS).thrust, 5076.093750, 1e-9,
              "the 0.5 m at cruise");
    // Neither shipped part ever reaches its fuel line or its sky ceiling at the
    // datum: the nozzle is what shuts it off.
    for(double v : {0.0, 275.0, 550.0}) {
        CHECK_TRUE(jetEngine(v, RHO0, OX_AIR, FAN, PUMP, V_E, A).burn < PUMP, "fuel headroom");
        CHECK_TRUE(jetEngine(v, RHO0, OX_AIR, FAN_S, PUMP_S, V_E, AS).burn < PUMP_S,
                   "fuel headroom");
    }
}

static void test_air_is_free() {
    printf("== the mass balance: an air-breathing engine sheds nothing of its own ==\n");

    // The law is deliberately not rocket equation-shaped. The fuel's mass is not
    // thrown; the AIR is. At 275 m/s the engine swallows 209 kg/s and burns 1.01.
    const double v = 275.0;
    const JetS e = jetEngine(v, RHO0, OX_AIR, FAN, PUMP, V_E, A);
    const double air = jetAirRate(v, e.wake, RHO0, A);
    CHECK_REL(air, 209.015625, 1e-12, "the stream");
    CHECK_REL(e.burn, 1.007102902, 1e-9, "the fuel");
    CHECK_TRUE(air > 200.0 * e.burn, "the working mass is the air's, 207x the fuel's");
    /* So the fuel tank is not the working fluid's tank. A 3000 kg jet-fuel load
       (43 m3) buys 2979 s at cruise -- the endurance question -- while the thrust
       comes from air that is never carried. */
    const double endurance = 3000.0 / e.burn;
    CHECK_REL(endurance, 2978.8416, 1e-7, "seconds at cruise on 3000 kg of jetfuel");
    CHECK_TRUE(endurance > 1800.0 && endurance < 3600.0, "a jet crosses the ocean");
}

static void test_regressions() {
    printf("== the mistakes this law replaces ==\n");

    // The #630 intake gate is absolute, not body-relative: it divides by the rating
    // air, so the same column thickness on any body buys the same share of a part.
    CHECK_REL(jetIntakeGate(kAirSeaLevelDensity), 1.0, 0.0, "Earth datum == Kerbin rating air");
    CHECK_REL(jetIntakeGate(1.225 * std::exp(-13000.0 / 6000.0)), 0.114558844, 1e-8,
              "a 6 km scale height at 13 km gates to 0.111 whatever body authored it");
    // A jet on a body with no atmosphere gets nothing, whatever its fan rating.
    CHECK_REL(jetEngine(100.0, 0.0, OX_AIR, 1e6, 10.0, 3000.0, A).thrust, 0.0, 0.0,
              "vacuum: a jet is not a rocket");
    CHECK_REL(jetEngine(100.0, 0.0, OX_AIR, 1e6, 10.0, 3000.0, A).burn, 0.0, 0.0,
              "and it does not idle fuel");
    // The old law kept a fan-force term alive past v_e. Here everything is one
    // stream, so the wall is the wall at every density and every sky.
    for(double rho : {0.015547, 0.0948, 1.225, 3.552}) {
        CHECK_REL(jetEngine(V_E, rho, OX_AIR, FAN, PUMP, V_E, A).thrust, 0.0, 0.0,
                  "v = v_e is a wall in every sky");
        CHECK_TRUE(jetEngine(0.5 * V_E, rho, OX_AIR, FAN, PUMP, V_E, A).thrust >
                       jetEngine(V_E, rho, OX_AIR, FAN, PUMP, V_E, A).thrust,
                   "and thrust falls into it");
    }
    // Negative input speed (a tailwind faster than the hull, in the vehicle's
    // air-speed convention) is treated as rest, not as reverse thrust.
    const JetS tail = jetEngine(-200.0, RHO0, OX_AIR, FAN, PUMP, V_E, A);
    CHECK_REL(tail.thrust, jetEngine(0.0, RHO0, OX_AIR, FAN, PUMP, V_E, A).thrust, 1e-12,
              "a negative air speed is treated at rest");
    CHECK_REL(jetAirRate(-200.0, V_E, RHO0, A), jetAirRate(0.0, V_E, RHO0, A), 1e-12,
              "and swallows what it swallows at rest");
}

int main() {
    printf("\n=== jetEngine: one stream, four ceilings (pure math, src/drag.h) ===\n");
    test_static();
    test_falls_with_speed();
    test_ceilings();
    test_gate_and_density();
    test_chemistry_is_still_a_gate();
    test_energy_identity();
    test_shipped_parts_fill_their_nozzles();
    test_air_is_free();
    test_regressions();
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
