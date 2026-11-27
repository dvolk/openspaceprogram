//
// Jet engine thrust (src/drag.h jetThrust, header-only pure math): the
// air-breathing momentum balance of an engine that burns fuel against
// FREE air. Pinned here without Bullet/GL so the law is independent of
// the render/physics chain (like test_drag).
//
//   T(v, rho) = T_fan*d + m_f*v_e*d + m_air*(v_exit - v),
//   d = jetAirGate(rho, oxidizer_share)   -- min(the intake against the rating
//   density, the oxygen in that intake against dry air's), so an air with no
//   oxidizer is a dead engine at any density (#630).
//   v_exit = jetRamExitSpeed(...) -- the authored v_e, capped at what the burn can
//   heat the captured air to: 1/2 m_air (v_exit^2 - v^2) <= Q m_burnt (#1206).
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
#include "tchecks.h"

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
    CHECK_REL(T(0.0, RHO_SEA), T_FAN + M_F * V_E, 1e-12,
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

    /* The ram term rho*A*v*(v_e - v) peaks at v = v_e/2 -- true while the energy cap
       below stays silent, which it does for this part in this dry air (test_the_burn_
       pays_for_the_exhaust pins the margin). A capped part peaks lower and weaker. */
    const double vpeak = V_E / 2.0;
    const double peak = T(vpeak, RHO_SEA);
    CHECK_TRUE(peak > T(0.0, RHO_SEA), "thrust rises off the static floor");

    // It falls back to the static value at v = v_e (the ram term is 0 there).
    CHECK_REL(T(V_E, RHO_SEA), T_FAN + M_F * V_E, 1e-9, "v=v_e -> back to static");

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
    CHECK_REL(T(0.0, 0.5 * RHO_SEA), 0.5 * (T_FAN + M_F * V_E), 1e-12,
               "rho=rho_sea/2 -> static/2");

    /* The falloff is ABSOLUTE, not per-body (#630). The law used to divide by the
       body's own sea-level density, so a jet standing on Mars read 1.0 of Mars and
       produced its full authored fan thrust on 0.02 kg/m3 of CO2. A part's
       jet_fan_thrust is one machine's number at ONE intake density, so 0.02 kg/m3
       buys 0.016 of it, whatever planet it is standing on. */
    CHECK_REL(jetThrust(0.0, 0.12, OX_AIR, T_FAN, M_F, V_E, A),
               (0.12 / kAirSeaLevelDensity) * (T_FAN + M_F * V_E), 1e-12,
               "0.12 kg/m3 buys 0.098 of the rated fan, not full thrust");

    // Denser than the rating intake clamps the gate at 1 (no overshoot).
    CHECK_REL(jetThrust(0.0, 2.0 * RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E, 1e-12, "rho > rating: the gate clamps at 1");

    // The gate clamps the fan + fuel at sea level, but the RAM term is
    // linear in rho (not gated by d): at rho = 2*rho_sea the fan is full
    // and the ram doubles.
    CHECK_REL(jetThrust(V_E / 2.0, 2.0 * RHO_SEA, OX_AIR, T_FAN, M_F, V_E, A),
               T_FAN + M_F * V_E + (2.0 * RHO_SEA) * A * (V_E / 2.0) * (V_E - V_E / 2.0),
               1e-9, "rho > rho_sea: fan clamps at 1, ram follows local rho");
}

static void test_air_gate() {
    printf("== jetAirGate: the share of the rated intake (thrust AND fuel) ==\n");

    CHECK_REL(jetAirGate(RHO_SEA, OX_AIR), 1.0, 1e-12, "the rating intake -> 1");
    CHECK_REL(jetAirGate(0.5 * RHO_SEA, OX_AIR), 0.5, 1e-12, "half density -> half");
    CHECK_REL(jetAirGate(2.0 * RHO_SEA, OX_AIR), 1.0, 1e-12, "denser clamps at 1");
    CHECK_REL(jetAirGate(0.0, OX_AIR), 0.0, 0.0, "vacuum -> 0 (no thrust, no burn)");
    // A body whose air states no oxidizer is a vacuum to a jet, at any density.
    CHECK_REL(jetAirGate(RHO_SEA, 0.0), 0.0, 0.0, "no oxidizer in the air -> 0");
    CHECK_REL(jetAirGate(std::nan(""), OX_AIR), 0.0, 0.0,
               "NaN density fails closed (no thrust AND no burn)");
    CHECK_REL(jetAirGate(RHO_SEA, std::nan("")), 0.0, 0.0,
               "NaN oxidizer fails closed the same way");

    // The flow gate IS the gate the thrust law uses: at any density, static
    // thrust is the gate times the sea-level static, so the tank drains in
    // lockstep with the fan + fuel terms (ApplyThrust multiplies by the same
    // jetAirGate).
    const double gates[] = {0.1, 0.3, 0.75, 1.0};
    for(double g : gates) {
        CHECK_REL(T(0.0, g * RHO_SEA),
                   jetAirGate(g * RHO_SEA, OX_AIR) * (T_FAN + M_F * V_E),
                   1e-12, "flow gate == the thrust's fan gate");
    }
}

static void test_vacuum() {
    printf("== jetThrust: vacuum ==\n");

    // No air at all: ZERO thrust at ANY speed -- a jet cannot run without
    // air, and ApplyThrust burns no fuel (the flow is gated by jetAirGate).
    CHECK_REL(jetThrust(0.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at rest -> 0");
    CHECK_REL(jetThrust(300.0, 0.0, OX_AIR, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "rho=0 at speed -> 0 (no air, no thrust)");
    /* An air with no oxidizer in it is the other vacuum, and the denser it is the
       more obviously the old law was wrong: Eve's sea-level air is 1.7 kg/m3,
       39% denser than Kerbin's, and every gram of it is CO2. */
    CHECK_REL(jetThrust(0.0, 1.7, 0.0, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "1.7 kg/m3 of CO2 -> 0 (no oxidizer, no thrust)");
    CHECK_REL(jetThrust(V_E / 2.0, 1.7, 0.0, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "no oxidizer -> no thrust at the ram peak either");
}

static void test_degenerate() {
    printf("== jetThrust: degenerate inputs ==\n");
    const double rho = RHO_SEA;

    // A negative speed reads as zero speed (the static thrust).
    CHECK_REL(jetThrust(-50.0, rho, OX_AIR, T_FAN, M_F, V_E, A), T_FAN + M_F * V_E,
               1e-12, "v<0 -> the static thrust");

    // No intake area: no ram term, just the static (fan + fuel) thrust.
    CHECK_REL(jetThrust(100.0, rho, OX_AIR, T_FAN, M_F, V_E, 0.0), T_FAN + M_F * V_E,
               1e-12, "A=0 -> static only (no ram)");

    // No fan thrust: the static floor is just the (small) fuel momentum.
    CHECK_REL(jetThrust(0.0, rho, OX_AIR, 0.0, M_F, V_E, A), M_F * V_E, 1e-12,
               "T_fan=0 -> just the fuel momentum");
}

/* The half of the gate that only a MIX can state (#630): kilograms of burnable
   oxidizer per kilogram of air, which is what makes "no free oxygen here" a fact the
   engine acts on rather than a fact nobody can say. */
static void test_oxidizer() {
    printf("== jetAirGate: an air with no oxidizer is a dead engine ==\n");

    // Dry air's share, from the species table rather than typed here: if the table
    // moves, the rating the gate divides by moves with it.
    CHECK_REL(OX_AIR, 0.2314, 5e-5, "dry air is 23.1% oxidizer by mass");

    // min, not a product. On an oxygen world the intake half and the oxygen half are
    // the same number, so multiplying them would bill thin air twice.
    CHECK_REL(jetAirGate(0.3 * RHO_SEA, OX_AIR), 0.3, 1e-12,
               "dry air at 0.3 of the rating density -> 0.3, not 0.09");

    // Mars: 0.02 kg/m3 of CO2 with a 0.13% oxygen trace (res/systems' CO2_HEAVY).
    // The air half says 0.016; the oxygen half says 6.8e-5. The scarcer one wins.
    const double mars_rho = 0.02, mars_ox = 0.000959;
    const double air_half = mars_rho / kAirSeaLevelDensity;
    const double ox_half = mars_rho * mars_ox / (kAirSeaLevelDensity * OX_AIR);
    CHECK_TRUE(ox_half < air_half, "on Mars the oxygen runs out long before the air does");
    CHECK_REL(jetAirGate(mars_rho, mars_ox), ox_half, 1e-12,
               "the scarcer half IS the gate");

    // An oxygen-rich air does not buy an engine more than its rated intake: the
    // density half clamps at 1 however much oxidizer is in the air.
    CHECK_REL(jetAirGate(3.0 * RHO_SEA, 1.0), 1.0, 1e-12,
               "dense oxygen-rich air clamps at 1, not above");

    // A negative share is not a small one: it fails closed like NaN does.
    CHECK_REL(jetAirGate(RHO_SEA, -1.0), 0.0, 0.0, "a negative share fails closed");

    /* jetOxGate is that same limit on its own, with no density in it. Since #1206 it is
       not a currency of its own any more -- the exhaust momentum is billed by
       jetRamExitSpeed, in energy -- so its only job left is as the oxidizer half of
       jetAirGate. It clamps: extra oxygen does not make a fan push harder than its
       intake. */
    CHECK_REL(jetOxGate(OX_AIR), 1.0, 1e-12, "dry air pays in full");
    CHECK_REL(jetOxGate(2.0 * OX_AIR), 1.0, 1e-12, "an oxygen-rich air clamps at 1");
    CHECK_REL(jetOxGate(0.0), 0.0, 0.0, "no oxidizer pays nothing");

    /* The RAM term pays too, and this is where the two halves of the gate stop being
       one number. At the ram peak (v = ve/2) the exhaust momentum is most of the
       thrust: Mars's datum -- 0.02 kg/m3 of CO2 with a 0.13% oxygen trace -- bills the
       fan 6.8e-5 of its rating, 6.1 N, and the air it captures is 0.02 kg/m3 whatever
       the gas is. A law that threw that air out the back ungated would report 536 N at
       275 m/s for a fuel flow the same gate has already cut to 8.1e-5 kg/s: momentum out
       of no burn. What that burn DOES buy is small but real: 8.1e-5 kg/s of fuel heats
       the 1.925 kg/s of captured flow by 6.5 m/s, so 12.6 N of ram over 6.1 N of fan and
       fuel -- 18.7 N. (This pin read 8.3 N until #1206, paying for the ram with jetOxGate
       as a multiplier: the right refusal, charged a second time and in the wrong
       currency -- see jetRamExitSpeed below.) Same density and speed with dry air's
       oxygen instead: 2010 N, so the difference is the gas and not the thinness. */
    const double v_peak = V_E / 2.0;
    const double co2 = jetThrust(v_peak, mars_rho, mars_ox, T_FAN, M_F, V_E, A);
    CHECK_REL(co2, 18.681, 1e-3, "Mars's datum at the ram peak is 18.7 N, not 536 N");
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
    CHECK_REL(air.oxidizer_share, OX_AIR, 0.0,
               "a hand-built profile defaults to dry air, exactly as its molar_mass and "
               "gamma do: a fixture must not be a silently dead engine");

    const double mars_rho = 0.02, mars_ox = 0.000959;
    DragAtmosphere mars { mars_rho, 20000.0, 80000.0 };
    mars.oxidizer_share = mars_ox;

    /* The door is the same law, bit for bit -- a second arithmetic here would be the
       drift this refactor exists to prevent. */
    CHECK_REL(jetAirGate(air, RHO_SEA), jetAirGate(RHO_SEA, air.oxidizer_share),
               0.0, "jetAirGate(profile, rho) IS jetAirGate(rho, profile.share)");
    CHECK_REL(jetAirGate(mars, mars_rho), jetAirGate(mars_rho, mars_ox), 0.0,
               "on a Mars-shaped sky too");
    CHECK_REL(jetThrust(V_E / 2.0, mars_rho, mars, T_FAN, M_F, V_E, A),
               jetThrust(V_E / 2.0, mars_rho, mars_ox, T_FAN, M_F, V_E, A), 0.0,
               "and the engine's door is the same engine");

    /* It reads the FIELD, not a constant: one density, two skies. */
    CHECK_TRUE(jetAirGate(air, mars_rho) > 200.0 * jetAirGate(mars, mars_rho),
               "the same 0.02 kg/m3 gates two ways once the composition travels with the "
               "air -- a door that ignored the share would read dry air everywhere");

    /* A sky with nothing to burn against stays dead through the door, thrust AND burn. */
    DragAtmosphere eve { 1.7, 7000.0, 90000.0 };
    eve.oxidizer_share = 0.0;
    CHECK_REL(jetAirGate(eve, 1.7), 0.0, 0.0,
               "1.7 kg/m3 with no oxidizer is a hole in the hull, not an engine");
    CHECK_REL(jetThrust(V_E / 2.0, 1.7, eve, T_FAN, M_F, V_E, A), 0.0, 0.0,
               "and it hands back no thrust either");
}

/* THE BURN PAYS FOR THE EXHAUST IN ENERGY (#1206). The law used to pay for the ram term
   with jetOxGate -- a multiplier -- which refuses a CO2 sky but says nothing about how
   much exhaust ANY engine can afford. jetRamExitSpeed does: the kinetic energy handed to
   the captured air cannot outweigh the heat in the fuel, so
        v_exit = min(v_e, sqrt(v^2 + 2 Q m_burnt / m_air)).
   Two things have to be true of that, and both are pinned: nothing shipped sits inside
   the cap in a breathable sky (so #630's and #635's pins keep their values), and a part
   authored past what its fuel line affords stops being a free ride rather than getting a
   discount. In an oxidizer-TRACE sky the cap does bite, hard -- that is the new behaviour
   replacing jetOxGate as a ram multiplier, and it makes Eve's 3.55 kg/m3 of CO2 a weak
   engine (1.2% of rating) rather than a dead one. */
static void test_the_burn_pays_for_the_exhaust() {
    printf("== jetRamExitSpeed: the burn pays for the exhaust in energy (#1206) ==\n");

    /* How tight the ask is. A ram term asks hardest at v = v_e/sqrt(3), where
       v(v_e^2 - v^2) tops out; the fuel line pays 2 Q rate / (1.225 A), the density
       having cancelled out of the ratio. For the r1 jet that is 6.4e7 asked against
       2.4e8 paid -- nearly 4x clear, which is why this cap changes nothing we ship. */
    const double v_ask = V_E / std::sqrt(3.0);
    CHECK_REL(v_ask * (V_E * V_E - v_ask * v_ask), 6.404e7, 1e-3,
              "the hardest momentum ask a v_e 550 ram term makes");
    CHECK_REL(2.0 * kJetFuelHeatPerKg * M_F / (kAirSeaLevelDensity * A), 2.407e8, 1e-3,
              "what its authored fuel line pays for");

    // Silent across the working range: the cap never picks over the authored v_e.
    for(double rho : {0.01, 0.0948, 0.502, RHO_SEA, 2.0 * RHO_SEA}) {
        for(double v : {1.0, 100.0, v_ask, V_E / 2.0, V_E, 2.0 * V_E}) {
            CHECK_REL(jetRamExitSpeed(v, rho * A * v, M_F * jetAirGate(rho, OX_AIR), V_E),
                      V_E, 0.0, "a shipped jet affords its authored v_e at this air and speed");
        }
    }
    /* v = 0 is the one row left out above, and it deserves its own line: an intake that
       captures nothing has no exit speed to speak of, so the guard returns the flight
       speed and the ram term (m_air times the difference) stays 0 either way. */
    CHECK_REL(jetRamExitSpeed(0.0, 0.0, M_F, V_E), 0.0, 0.0,
              "at rest the air the cap would meter never enters the engine");

    /* The affordable exit speed is DENSITY-FREE below the rating: the intake swallows
       rho*A*v and the gated burn is rate*rho/1.225, so rho cancels. This is the shape of
       #1206's wall -- thin air buys thrust and affordability together, so the cap neither
       rescues a jet at 20 km nor moves the speed it dies at. Pinned on the RAW affordable
       speed: jetRamExitSpeed min's it against the authored v_e, which would make both
       sides 550 and the comparison vacuous. */
    const double affordHigh = jetFuelAffordedExitSpeed(V_E / 2.0, 0.0948 * A * (V_E / 2.0),
                                                      M_F * jetAirGate(0.0948, OX_AIR));
    const double affordSea = jetFuelAffordedExitSpeed(V_E / 2.0, RHO_SEA * A * (V_E / 2.0),
                                                     M_F * jetAirGate(RHO_SEA, OX_AIR));
    CHECK_REL(affordHigh, affordSea, 1e-12,
              "the same 275 m/s affords the same exit speed at 20 km as at sea level");
    CHECK_TRUE(affordHigh > 1.5 * V_E,
               "and what it affords is well past what this part promises");
    /* The law's ceiling is min(pay, promise): the part's authored v_e when the fuel line
       is richer than the ask, the fuel line when it is not. */
    CHECK_REL(jetRamExitSpeed(V_E / 2.0, RHO_SEA * A * (V_E / 2.0),
                              M_F * jetAirGate(RHO_SEA, OX_AIR), V_E), V_E, 0.0,
              "the promise is smaller here, so the promise wins");
    CHECK_REL(jetRamExitSpeed(V_E / 2.0, RHO_SEA * A * (V_E / 2.0),
                              M_F * jetAirGate(RHO_SEA, OX_AIR), 4000.0), affordSea, 1e-12,
              "promise a nozzle the fuel cannot fill and the fuel line wins");

    /* Above the rating the gate clamps at 1 while the air keeps arriving, so dense air
       tightens the cap. The 2 m jet starts to bite at 4.60 kg/m3 -- 3 sqrt(3) Q rate /
       (A v_e^3), the density where the richest ask (v = v_e/sqrt(3)) exactly equals the
       gated burn's heat -- and the 0.5 m one at 4.70. Denser than any DRY-air sky in the
       game (Kerbin/Earth/Laythe/Shay 1.225). It is NOT denser than every live sky: Eve,
       whose CO2 datum with a 0.096 % O2 trace resolves to 3.55 kg/m3, is capped hard by
       the trace side -- see the block below -- so "dense" is not the only way here, and
       Jool/Venus/Titan (share 0) are the engines that never start. */
    const double rhoBite = 3.0 * std::sqrt(3.0) * kJetFuelHeatPerKg * M_F
                           / (A * V_E * V_E * V_E);
    CHECK_REL(rhoBite, 4.60, 1e-2, "the density where the 2 m jet's cap first bites");
    CHECK_TRUE(jetRamExitSpeed(v_ask, 4.7 * A * v_ask, M_F * jetAirGate(4.7, OX_AIR), V_E)
                   < V_E,
               "4.7 kg/m3 of dry air bites: the gate clamps, the air does not");
    CHECK_TRUE(jetRamExitSpeed(v_ask, 4.5 * A * v_ask, M_F * jetAirGate(4.5, OX_AIR), V_E)
                   == V_E,
               "and 4.5 does not, so the closed form is the boundary and not a guess");

    /* A trace-oxygen air caps the same part without any help from density. Eve's datum:
       3.552 kg/m3, share 0.000959, so the gate is 0.0120 -- 1.2 % of a rated engine --
       and the burn (0.0144 kg/s) cannot heat the 342 kg/s this intake swallows at
       275 m/s. The fuel affords 281 m/s of exit speed at 275 m/s of flight: 6.5 m/s of
       thrust-producing flow. Note the OLD law reported LESS here (it charged the ram by
       jetOxGate as well as through the gate): 1479 N then, 3318 N now, against the
       95.1 kN the promise alone would give. The engine is honest at either number -- it
       is simply a very weak engine, and now a correctly-priced one. */
    const double EVE_RHO = 3.552, EVE_OX = 0.000959;
    const double eveD = jetAirGate(EVE_RHO, EVE_OX);
    CHECK_REL(eveD, 0.012017, 1e-4, "Eve is a live engine at 1.2% of its rating, not a dead one");
    CHECK_REL(jetFuelAffordedExitSpeed(V_E / 2.0, EVE_RHO * A * (V_E / 2.0), M_F * eveD),
              281.5, 1e-2, "what Eve's trace burn affords at the ram peak");
    CHECK_REL(jetThrust(V_E / 2.0, EVE_RHO, EVE_OX, T_FAN, M_F, V_E, A), 3317.8, 1e-3,
              "Eve's datum thrust, priced once instead of twice");
    CHECK_TRUE(jetThrust(V_E / 2.0, EVE_RHO, EVE_OX, T_FAN, M_F, V_E, A)
                   < 0.05 * (T_FAN * eveD + M_F * V_E * eveD
                             + EVE_RHO * A * (V_E / 2.0) * (V_E - V_E / 2.0)),
               "and under 5% of what the authored nozzle alone promises there");

    /* Teeth. #1206's real experiment part (tmp/n1099/parts_scram2.json: a 0.5 m jet
       re-declared -- fan 3000, 3.0 kg/s of jetfuel, v_e 3200, A 0.35, 43.75 kg) proves
       the law already shapes like a scramjet. It also hands that part 64.7 kN at
       2400 m/s in 20 km air: 178 MW given to the flow on a 10 MW fire. Capped, the same
       part at the same state makes 5.09 kN, and at sea level and 1000 m/s 956 kN becomes
       126 kN (1.98 GW against 129 MW). */
    const double VE32 = 3200.0, FAN3 = 3000.0, M3 = 3.0, RHO20 = 0.0948;
    const double d20 = jetAirGate(RHO20, OX_AIR);
    const double uncapped = FAN3 * d20 + M3 * VE32 * d20
                            + RHO20 * A * 2400.0 * (VE32 - 2400.0);
    const double capped = jetThrust(2400.0, RHO20, OX_AIR, FAN3, M3, VE32, A);
    CHECK_REL(uncapped, 64680.7, 1e-4, "what the old law reported for that part");
    CHECK_REL(capped, 5090.4, 1e-3, "what its 0.232 kg/s burn can afford");
    CHECK_TRUE(capped < 0.08 * uncapped, "the free ride is gone, not trimmed");
    CHECK_REL(jetThrust(1000.0, RHO_SEA, OX_AIR, FAN3, M3, VE32, A), 126477.0, 1e-3,
              "the same part at sea level and 1000 m/s: 126 kN, not 956 kN");

    /* The capped number is the energy bound read sideways: spending Q on the burn gives
       ram = 2 Q m_burnt / (v_exit + v). This is the same statement as the exit-speed cap,
       not an independent one -- it catches a factor-of-2 slip on one side, and the
       absolute pins above are what catch a wrong Q. */
    const double exit2400 = jetRamExitSpeed(2400.0, RHO20 * A * 2400.0, M3 * d20, VE32);
    CHECK_REL(RHO20 * A * 2400.0 * (exit2400 - 2400.0),
              2.0 * kJetFuelHeatPerKg * M3 * d20 / (exit2400 + 2400.0), 1e-9,
              "the ram term is the fuel's energy, read the other way");
    CHECK_REL(capped, FAN3 * d20 + M3 * VE32 * d20
                        + 2.0 * kJetFuelHeatPerKg * M3 * d20 / (exit2400 + 2400.0),
              1e-9, "fan + fuel momentum + the energy-bounded ram");

    /* A capped engine still behaves like an engine: thrust falls off with speed across
       the supersonic range, and at v = v_e exactly the cap has nothing to say (the
       authored exit speed and the speed of flight meet, leaving only the static terms). */
    double prev = 1e30;
    for(double v : {1200.0, 1600.0, 2000.0, 2400.0, 2800.0, 3100.0}) {
        const double t = jetThrust(v, RHO20, OX_AIR, FAN3, M3, VE32, A);
        CHECK_TRUE(t < prev, "a capped engine still falls off with speed");
        prev = t;
    }
    CHECK_REL(jetThrust(VE32, RHO20, OX_AIR, FAN3, M3, VE32, A),
              FAN3 * d20 + M3 * VE32 * d20, 1e-12, "v = v_e -> the static terms, cap or no cap");

    /* The lever is the fuel line, not the nozzle. The SAME v_e 550 on a 1 g/s burner
       keeps its fan and loses its ram: what it can afford is 276 m/s of exit speed at
       275 m/s of flight, so the ram falls under 1% of the 1.2 kg/s engine's. Any future
       fast jet has to be authored as a machine -- a nozzle AND a fuel line -- which is
       what #1206's parts-family step is for. */
    const double ram_thin = jetThrust(V_E / 2.0, RHO_SEA, OX_AIR, T_FAN, 1e-3, V_E, A)
                            - (T_FAN + 1e-3 * V_E);
    CHECK_TRUE(ram_thin < 0.01 * (T(V_E / 2.0, RHO_SEA) - T(0.0, RHO_SEA)),
               "a jet on 1 g/s of fuel keeps its fan and loses its ram");

    /* And with the fuel line shut it keeps only the fan: the ram needs a burn, not just
       a moving intake. The old law kept 32.4 kN of ram here on nothing at all. */
    CHECK_REL(jetThrust(V_E / 2.0, RHO_SEA, OX_AIR, T_FAN, 0.0, V_E, A), T_FAN, 0.0,
              "no fuel, no ram: a fan alone is the fan term");
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
    test_the_burn_pays_for_the_exhaust();
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
