//
// Atmospheric drag law (src/drag.h, header-only pure math): the two atmosphere
// profiles and the quadratic force. Pinned here without Bullet/GL so the law is
// independent of the render/physics chain (like test_surfmap, test_orbit).
//
//   airDensity:  rho(alt) = rho0 * exp(-alt/H) above the sea-level DATUM; at or
//                below the datum the density is rho0, FLAT -- it used to be vacuum,
//                which stripped drag, q and jet thrust from anything under the
//                datum (#245); a degenerate atmosphere -> 0; rho(H) = rho0/e;
//                monotone down; vacuum at and above the hard top (height), and no
//                cutoff at all when height is 0.
//   airTemperature / airPressure / airSoundSpeed: what one exponential IMPLIES
//                (#552 A2). T = g H M / R, p = rho g H, a = sqrt(gamma g H); molar
//                mass moves ONLY the temperature and gamma ONLY the sound speed;
//                rho never sees gravity on this profile, so the seam cannot move a
//                drag number; p rides rho's own scale height (hydrostatic) above the
//                datum and is flat under it; no air or no gravity reads 0.0, never
//                a NaN.
//   the layer table (src/atmo.h, #552 A4): a body that authors {alt, T, lapse}
//                levels instead of a scale height. base_p is resolved at load by
//                integrating hydrostatics from the authored rho0, then rho = p/(Rs T)
//                and T follows the lapse -- so T, a and even rho vary with altitude
//                and gravity. Earth ships this; the pins are the 1976 standard
//                atmosphere's own published column, and the load invariants are
//                refused rather than carried.
//   liftSlopeMach (#543 A5): the slope belongs to the AIR, not the part. Prandtl-
//                Glauert cl/sqrt(1-M^2) to kTransonicMach, thin-airfoil 4/sqrt(M^2-1)
//                from kSupersonicMach, and a straight join in ABSOLUTE slope between
//                -- continuous at both edges, finite at M 1, no threshold at the low
//                end. One number per surface, which liftCurve, liftForce,
//                liftSlopePerQ, controlForce and surfaceTorquePerRadian all spend, so
//                #487's rate damping inherits compressibility instead of carrying it.
//   dragForce:   opposite the motion; |F| = 0.5 * rho * cd * A * v^2;
//                scales as v^2; zero on any degenerate input.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

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

static void test_density() {
    printf("== airDensity: exponential model ==\n");

    const DragAtmosphere a { 1.225, 5500.0 };  // a Kerbin-ish atmosphere

    /* At / below the datum the air is the sea-level density, FLAT (#245). It used
       to read as vacuum there, which is what stripped drag, lift and jet thrust off
       a hull together. Reaching the datum needs no authoring at all: relief is
       SIGNED about the radius (terragen.h:terrainRelief) and only a Flat-sea body
       clamps its floor, so Kerbin's sea beds and Eve's dry valleys already sat under
       a datum that was just the bare radius. e2e/fixtures/systems/245-kerbin-sea.json
       is the case where a PAD sits under one. */
    CHECK_NEAR(airDensity(a, 0.0), 1.225, 0.0, "alt=0 -> rho0, not vacuum");
    CHECK_NEAR(airDensity(a, -100.0), 1.225, 0.0, "alt<0 -> rho0, not vacuum");
    // Flat, not the exponential continued down: that would make the deepest sea the
    // densest air in the game, and it grows without bound (115 kg/m3 by -25 km).
    CHECK_NEAR(airDensity(a, -25000.0), 1.225, 0.0, "a deep sea floor is still rho0");
    // Monotone ACROSS the datum, which the two exact pins above cannot see: a bump
    // sitting at the meeting point would still read rho0 at both of them.
    CHECK_TRUE(airDensity(a, -5000.0) >= airDensity(a, 5000.0),
               "rho does not fall as a hull descends through the datum");
    // And no cliff where the model meets the datum.
    CHECK_NEAR(airDensity(a, 1e-6), 1.225, 1e-9, "the profile meets rho0 at the datum");
    /* The clamp is THIS body's sea level, not a constant. Every atmosphere in this
       file is 1.225, so a hard-coded 1.225 passes them all; Eve (1.7 kg/m3 at
       H 7000, its valley floors down near -amp/3 = -1.7 km) is the one that says
       sea_level_density. */
    const DragAtmosphere eve { 1.7, 7000.0 };
    CHECK_NEAR(airDensity(eve, 0.0), 1.7, 0.0, "Eve at its datum -> its own rho0");
    CHECK_NEAR(airDensity(eve, -1667.0), 1.7, 0.0, "an Eve valley floor -> Eve's rho0");

    // Just above the surface, the density is (essentially) sea level.
    CHECK_NEAR(airDensity(a, 1.0), 1.225 * std::exp(-1.0 / 5500.0), 1e-12,
               "rho(1 m) == rho0 * e^(-1/H)");

    // One scale height: the density has fallen to rho0 / e.
    CHECK_NEAR(airDensity(a, 5500.0), 1.225 / std::exp(1.0), 1e-12,
               "rho(H) == rho0 / e");

    // Monotone: denser lower down.
    CHECK_TRUE(airDensity(a, 500.0) > airDensity(a, 5000.0),
               "rho is monotone decreasing with altitude");

    // Deep in the air it is negligible even with no hard top (height 0).
    CHECK_TRUE(airDensity(a, 8 * 5500.0) < 0.01 * 1.225,
               "rho(8H) < 1% of sea level");

    /* A degenerate atmosphere (no density or no scale height) reads as none --
       BELOW the datum too. The clamp is the interesting branch now, so a refactor
       that hoists it above these guards would start billing a body that authors no
       scale height 1.225 kg/m3 of drag down there. */
    const DragAtmosphere none0 { 0.0, 5500.0 };
    const DragAtmosphere noneH { 1.225, 0.0 };
    CHECK_NEAR(airDensity(none0, 100.0), 0.0, 0.0, "no density -> 0");
    CHECK_NEAR(airDensity(noneH, 100.0), 0.0, 0.0, "no scale height -> 0");
    CHECK_NEAR(airDensity(none0, -100.0), 0.0, 0.0, "no density -> 0 below the datum");
    CHECK_NEAR(airDensity(noneH, -100.0), 0.0, 0.0, "no scale height -> 0 below it");
}

static void test_densityTop() {
    printf("== airDensity: the hard atmosphere top ==\n");

    const DragAtmosphere a { 1.225, 5500.0 };          // no cutoff
    const DragAtmosphere kerbin { 1.225, 5500.0, 70000.0 };  // Kerbin's authored top

    // At and above the top the air is vacuum.
    CHECK_NEAR(airDensity(kerbin, 70000.0), 0.0, 0.0, "rho(top) == 0");
    CHECK_NEAR(airDensity(kerbin, 100000.0), 0.0, 0.0, "rho above the top == 0");
    CHECK_TRUE(airDensity(kerbin, 69000.0) > 0.0, "rho just below the top > 0");

    // Inside the top the cutoff changes nothing: the same exponential.
    CHECK_NEAR(airDensity(kerbin, 5500.0), airDensity(a, 5500.0), 1e-15,
               "inside the top the model is unchanged");
    CHECK_NEAR(airDensity(kerbin, 60000.0),
               1.225 * std::exp(-60000.0 / 5500.0), 1e-12,
               "rho(60 km) == rho0 * e^(-60km/H)");

    // height 0 means no cutoff at all (the fallback for a body that does not
    // author one): 200 km still reads a wisp, not vacuum.
    CHECK_TRUE(airDensity(a, 200000.0) > 0.0, "height 0 -> no cutoff");
    CHECK_NEAR(airDensity(kerbin, 200000.0), 0.0, 0.0,
               "with a top, 200 km is vacuum");

    // The force follows: a 100 km Kerbin orbit is genuine vacuum, so no
    // silhouette pass and no slow orbital decay.
    const glm::dvec3 v { 0.0, 0.0, 2300.0 };
    CHECK_TRUE(dragForce(kerbin, 1.0, 10.0, 100000.0, v) == glm::dvec3(0.0),
               "no drag above the top");
    CHECK_TRUE(glm::length(dragForce(kerbin, 1.0, 10.0, 30000.0, v)) > 0.0,
               "drag below the top");
}

/* The atmosphere the way the game builds it. drag.h's ctor stops at (rho0, H, top)
   and gravity has NO world default there: an atmosphere that does not say what
   planet it is on reads 0 K / 0 Pa / 0 m/s, which is the loud answer, rather than
   quietly inheriting Earth's 9.81 on Eve or Jool. So every thermodynamic rig here
   names its g. */
static DragAtmosphere makeAir(double rho0, double H, double top, double g,
                              double molar = kAirMolarMass, double gam = kAirGamma) {
    DragAtmosphere a { rho0, H, top };
    a.gravity = g;
    a.molar_mass = molar;
    a.gamma = gam;
    return a;
}

static void test_air_thermodynamics() {
    printf("== the air's thermodynamics: what the authored scale height implies (#552 A2) ==\n");

    /* Every expectation below is a literal typed in from an independent
       calculation, not the formula re-run in place: these pins exist so that a
       future edit to ONE of the three spellings (T = g H M / R, p = rho g H,
       a = sqrt(gamma g H)) shows up as a disagreement instead of being agreed with. */
    const DragAtmosphere kerbin = makeAir(1.225, 5500.0, 70000.0, 9.81);
    CHECK_NEAR(airTemperature(kerbin, 0.0), 187.95593796005446, 1e-12,
               "Kerbin's isothermal temperature: g H M / R");
    CHECK_NEAR(airSoundSpeed(kerbin, 0.0), 274.83995342744475, 1e-12,
               "Kerbin's sound speed: sqrt(gamma g H)");
    CHECK_NEAR(airPressure(kerbin, 0.0), 66094.875, 1e-12,
               "Kerbin's sea-level pressure: rho0 g H");

    /* An atmosphere that does not name its planet gets no thermodynamics -- it is
       not quietly Earth. This is the pin that reddens a world default put back into
       drag.h's member initialiser. */
    const DragAtmosphere noworld { 1.225, 5500.0, 70000.0 };
    CHECK_NEAR(airTemperature(noworld, 0.0), 0.0, 0.0,
               "no gravity named -> 0 K, not Earth's 9.81");
    CHECK_NEAR(airSoundSpeed(noworld, 0.0), 0.0, 0.0, "no gravity named -> 0 m/s");
    CHECK_NEAR(airPressure(noworld, 0.0), 0.0, 0.0, "no gravity named -> 0 Pa");
    CHECK_NEAR(airDensity(noworld, 1000.0), 1.225 * std::exp(-1000.0 / 5500.0), 1e-12,
               "... and it still drags: g is not in rho");

    /* The ideal-gas tie spelled the OTHER way: if someone gives p its own formula
       while T keeps the old H, the two stop agreeing. */
    const double rspec = kGasR / kAirMolarMass;
    CHECK_NEAR(airPressure(kerbin, 0.0),
               1.225 * rspec * airTemperature(kerbin, 0.0), 1e-12,
               "p == rho R T / M, spelled through the gas constant");

    /* What a single exponential CAN do: an Earth-shaped body at H 8500 and g 9.82
       lands within ~1.5% of the real sea level with no temperature authored
       anywhere. Earth itself now authors a layer table instead (#552 A4, see
       test_layered_atmosphere); this fixture is the profile every other shipped body
       still uses, and the tolerance says "close", not "USSA". */
    const DragAtmosphere earth = makeAir(1.225, 8500.0, 0.0, 9.82);
    CHECK_NEAR(airTemperature(earth, 0.0), 290.773461987318, 1e-12,
               "H 8500 reads 290.8 K");
    CHECK_NEAR(airTemperature(earth, 0.0), 288.15, 0.015, "within 1.5% of USSA 288.15 K");
    CHECK_NEAR(airSoundSpeed(earth, 0.0), 341.8449941128289, 1e-12,
               "Earth's sound speed sqrt(gamma g H)");
    CHECK_NEAR(airSoundSpeed(earth, 0.0), 340.3, 0.015, "within 1.5% of USSA 340.3 m/s");
    CHECK_NEAR(airPressure(earth, 0.0), 101325.0, 0.015,
               "within 1.5% of USSA 101.325 kPa");

    /* What molar mass does and does not move: ONLY the temperature line. M cancels
       out of p and a, so those two are bit-identical -- a barometric or layer model
       that started using M in the density profile would go red here. The CO2 numbers
       are what the shipped defs now author for Venus / Mars / Duna / Eve. */
    const DragAtmosphere co2 = makeAir(1.225, 5500.0, 70000.0, 9.81, 0.044);
    CHECK_NEAR(airTemperature(co2, 0.0), 285.5289763237949, 1e-12,
               "CO2 air (M 0.044) reads 285.5 K");
    CHECK_NEAR(airSoundSpeed(co2, 0.0), airSoundSpeed(kerbin, 0.0), 0.0,
               "molar mass cancels out of the sound speed");
    CHECK_NEAR(airPressure(co2, 12000.0), airPressure(kerbin, 12000.0), 0.0,
               "molar mass cancels out of the pressure too");

    /* And gamma moves only the sound speed: it is cp/cv, and nothing else reads it.
       1.3 is what the CO2 bodies author; 1.0 is the monatomic limit. */
    const DragAtmosphere co2gam = makeAir(1.225, 5500.0, 70000.0, 9.81, 0.044, 1.3);
    const DragAtmosphere mono = makeAir(1.225, 5500.0, 70000.0, 9.81, kAirMolarMass, 1.0);
    CHECK_NEAR(airSoundSpeed(co2gam, 0.0), 264.842405970041, 1e-12,
               "CO2's gamma 1.3: sqrt(1.3 g H)");
    CHECK_NEAR(airTemperature(co2gam, 0.0), airTemperature(co2, 0.0), 0.0,
               "gamma does not move the temperature");
    CHECK_NEAR(airSoundSpeed(mono, 0.0), 232.28215600859227, 1e-12,
               "monatomic gas: sqrt(1.0 g H)");
    CHECK_NEAR(airTemperature(mono, 0.0), airTemperature(kerbin, 0.0), 0.0,
               "gamma does not move the temperature");
    CHECK_NEAR(airPressure(mono, 3000.0), airPressure(kerbin, 3000.0), 0.0,
               "gamma does not move the pressure");

    /* THE A2 PROMISE: adding the thermodynamics cannot move a drag number. rho is
       g-free, so a body's gravity rides the seam and touches nothing the flight
       model bills. */
    const DragAtmosphere duna = makeAir(1.225, 5500.0, 70000.0, 2.94);
    CHECK_NEAR(airDensity(duna, 0.0), airDensity(kerbin, 0.0), 0.0,
               "rho at the datum does not depend on gravity");
    CHECK_NEAR(airDensity(duna, 12000.0), airDensity(kerbin, 12000.0), 0.0,
               "rho aloft does not depend on gravity");
    CHECK_NEAR(airDensity(duna, -500.0), airDensity(kerbin, -500.0), 0.0,
               "nor does the flat floor under the datum");
    CHECK_NEAR(airSoundSpeed(duna, 0.0), 274.83995342744475 * std::sqrt(2.94 / 9.81), 1e-12,
               "the sound speed does: it scales as sqrt(g)");

    /* p rides rho's OWN scale height -- the same exponential, so the pressure
       gradient balances the weight of the air (hydrostatic) everywhere above the
       datum. Central difference: the truncation error is ~(dh/H)^2/6, hence 1e-4. */
    CHECK_NEAR(airPressure(kerbin, 5500.0), 66094.875 / std::exp(1.0), 1e-12,
               "p(H) == p0/e: pressure falls as fast as density");
    const double h = 10000.0, dh = 100.0;
    const double grad = (airPressure(kerbin, h - dh) - airPressure(kerbin, h + dh)) / (2.0 * dh);
    CHECK_NEAR(grad, airDensity(kerbin, h) * 9.81, 1e-4,
               "-dp/dh == rho g above the datum");

    /* Below the datum BOTH are flat (#245), which is deliberately not hydrostatic:
       under a has_sea body the game has thick air, not water, and no law integrates
       a water column (#618, closed as designed). Pinned so nobody "fixes" it by
       inventing a sub-datum pressure nothing bills. */
    CHECK_NEAR(airPressure(kerbin, -1000.0), airPressure(kerbin, 0.0), 0.0,
               "p is flat below the datum, like rho");
    CHECK_NEAR(airPressure(kerbin, 70000.0), 0.0, 0.0, "no air, no pressure at the top");
    CHECK_NEAR(airPressure(kerbin, 120000.0), 0.0, 0.0, "... nor above it");

    /* T and a stop at the top too, on BOTH profiles (#632). rho and p have always gone
       to zero there, so a Kerbin hull at 120 km used to read 188.0 K of air and a 274.8
       m/s sound speed in vacuum -- which is a Mach number divided by nothing (#543),
       and a different answer from what a layered body gives at its own top. */
    CHECK_NEAR(airTemperature(kerbin, 70000.0), 0.0, 0.0, "no air, no air temperature at the top");
    CHECK_NEAR(airSoundSpeed(kerbin, 70000.0), 0.0, 0.0, "no air, no sound speed at the top");
    CHECK_NEAR(airTemperature(kerbin, 120000.0), 0.0, 0.0, "... nor above it");
    CHECK_NEAR(airSoundSpeed(kerbin, 120000.0), 0.0, 0.0, "... nor above it");
    /* No authored top means no cutoff, which is what the exponential tail means. */
    const DragAtmosphere notop = makeAir(1.225, 5500.0, 0.0, 9.81);
    CHECK_NEAR(airTemperature(notop, 120000.0), airTemperature(kerbin, 0.0), 0.0,
               "height 0: the tail still implies its temperature");
    CHECK_NEAR(airSoundSpeed(notop, 120000.0), airSoundSpeed(kerbin, 0.0), 0.0,
               "... and its sound speed");

    /* Fail CLOSED on a degenerate or hostile atmosphere. Guards are in the
       !(x > 0.0) shape (src/angmath.h) rather than `x <= 0.0`, and the pins here are
       the ones that can tell: a ZERO g or H reads 0 from the arithmetic anyway, so
       only the negative and NaN cases prove the guard is doing anything. A negative
       g unguarded would bill a negative pressure and a NaN sqrt; a NaN would walk
       through `x <= 0.0` (every relational test against NaN is false) and come out
       the other side as a NaN Mach number (#604). */
    const DragAtmosphere none0 { 0.0, 5500.0 };
    const DragAtmosphere negg = makeAir(1.225, 5500.0, 70000.0, -9.81);
    const DragAtmosphere negH = makeAir(1.225, -5500.0, 70000.0, 9.81);
    const double nan = std::nan("");
    const DragAtmosphere badRho = makeAir(nan, 5500.0, 70000.0, 9.81);
    const DragAtmosphere badH = makeAir(1.225, nan, 70000.0, 9.81);
    const DragAtmosphere badG = makeAir(1.225, 5500.0, 70000.0, nan);
    CHECK_NEAR(airTemperature(none0, 0.0), 0.0, 0.0, "no density -> no temperature");
    CHECK_NEAR(airSoundSpeed(none0, 0.0), 0.0, 0.0, "no density -> no sound speed");
    CHECK_NEAR(airTemperature(negg, 0.0), 0.0, 0.0, "negative gravity -> 0 K, not a negative T");
    CHECK_NEAR(airPressure(negg, 0.0), 0.0, 0.0, "negative gravity -> 0 Pa, not a negative p");
    CHECK_NEAR(airSoundSpeed(negg, 0.0), 0.0, 0.0, "negative gravity -> 0 m/s, not a NaN sqrt");
    CHECK_NEAR(airTemperature(negH, 0.0), 0.0, 0.0, "negative scale height -> 0 K");
    CHECK_NEAR(airSoundSpeed(negH, 0.0), 0.0, 0.0, "negative scale height -> 0 m/s");
    CHECK_NEAR(airDensity(negH, 1000.0), 0.0, 0.0, "... and no density either");
    CHECK_NEAR(airTemperature(badRho, 0.0), 0.0, 0.0, "NaN density reads 0 K, not NaN");
    CHECK_NEAR(airTemperature(badH, 0.0), 0.0, 0.0, "NaN scale height reads 0 K");
    CHECK_NEAR(airTemperature(badG, 0.0), 0.0, 0.0, "NaN gravity reads 0 K");
    CHECK_NEAR(airPressure(badH, 1000.0), 0.0, 0.0, "NaN scale height -> 0 Pa");
    CHECK_NEAR(airSoundSpeed(badG, 0.0), 0.0, 0.0, "NaN gravity -> 0 m/s, never a NaN Mach");
    CHECK_NEAR(airDensity(badRho, 1000.0), 0.0, 0.0, "NaN density -> no drag at all");
    CHECK_NEAR(airDensity(badH, 1000.0), 0.0, 0.0, "NaN scale height -> no drag");
    CHECK_NEAR(airDensity(badG, 1000.0), 1.225 * std::exp(-1000.0 / 5500.0), 1e-12,
               "NaN gravity does not poison rho");
    CHECK_TRUE(std::isfinite(airTemperature(badG, 0.0)) && std::isfinite(airSoundSpeed(badG, 0.0))
               && std::isfinite(airPressure(badH, 100.0)) && std::isfinite(airDensity(badRho, 1.0)),
               "every closed read lands on a finite 0.0");
}

static void test_layered_atmosphere() {
    printf("== the layer table: a body that authors T(alt), not a scale height (#552 A4) ==\n");

    /* Earth's authored table (utils/make_solar_system.py): the 1976 standard
       atmosphere. The numbers checked against below are that standard's OWN published
       table, typed in rather than computed here, so a wrong exponent sign or a wrong
       anchor reads as a disagreement instead of agreeing with itself. Earth authors
       g 9.82 where the standard uses 9.80665, and M is rounded to the shipped
       0.028964 kg/mol; together those drift the profile by ~1% up at the stratopause,
       which is what the looser bands there are for. */
    std::vector<AtmoLayer> L = {
        {0.0,     288.15, -0.0065, 0.0},
        {11000.0, 216.65,  0.0,    0.0},
        {20000.0, 216.65,  0.001,  0.0},
        {32000.0, 228.65,  0.0028, 0.0},
        {47000.0, 270.65,  0.0,    0.0},
        {51000.0, 270.65, -0.0028, 0.0},
        {71000.0, 214.65, -0.002,  0.0},
    };
    CHECK_TRUE(resolveAtmoLevels(L, 1.225, kAirMolarMass, 9.82, 85000.0),
               "Earth's table resolves");
    DragAtmosphere air { 1.225, 0.0, 85000.0 };   // no scale height: the table IS the profile
    air.gravity = 9.82;
    air.layers = L.data();
    air.layer_count = (int)L.size();

    /* The anchor. sea_level_density survives at the datum EXACTLY (it is authored);
       the pressure there is derived from it and lands on the standard's 101,325 Pa
       without anyone writing that number in the system file. */
    CHECK_NEAR(airDensity(air, 0.0), 1.225, 0.0, "the datum keeps the authored density");
    CHECK_NEAR(airPressure(air, 0.0), 101325.0, 1e-4, "p0 comes out of rho0 and T0: 101.3 kPa");
    CHECK_NEAR(airTemperature(air, 0.0), 288.15, 0.0, "T at the datum is level 0's, authored");
    CHECK_NEAR(airSoundSpeed(air, 0.0), 340.3, 1e-4, "a at the datum: 340.3 m/s");

    /* The standard's pressure column, 0 to 51 km. */
    CHECK_NEAR(airPressure(air, 11000.0), 22632.0, 0.01, "p at 11 km (tropopause)");
    CHECK_NEAR(airPressure(air, 20000.0), 5474.9, 0.01, "p at 20 km");
    CHECK_NEAR(airPressure(air, 32000.0), 868.02, 0.01, "p at 32 km");
    CHECK_NEAR(airPressure(air, 47000.0), 110.91, 0.02, "p at 47 km (stratopause)");
    CHECK_NEAR(airPressure(air, 51000.0), 66.94, 0.02, "p at 51 km");
    CHECK_NEAR(airDensity(air, 11000.0), 0.36393, 0.01, "rho at 11 km");
    CHECK_NEAR(airDensity(air, 20000.0), 0.088035, 0.01, "rho at 20 km");

    /* Each level's base temperature, bit for bit -- and the standard's 85 km row
       needs no level of its own, because it is where the -2.0 K/km mesospheric lapse
       lands (186.65 K here against its 186.87 K). */
    CHECK_NEAR(airTemperature(air, 11000.0), 216.65, 0.0, "T at 11 km");
    CHECK_NEAR(airTemperature(air, 20000.0), 216.65, 0.0, "isothermal from 11 to 20 km");
    CHECK_NEAR(airTemperature(air, 32000.0), 228.65, 0.0, "warming 20 to 32 km");
    CHECK_NEAR(airTemperature(air, 47000.0), 270.65, 0.0, "warming 32 to 47 km");
    CHECK_NEAR(airTemperature(air, 71000.0), 214.65, 0.0, "cooling 51 to 71 km");
    CHECK_NEAR(airTemperature(air, 84999.0), 186.87, 0.005, "T near the top, where the lapse lands");

    /* WHAT THE LAPSE BUYS: the sound speed is no longer one number for the whole
       body, so the Mach number #543 divides by becomes an altitude question. */
    CHECK_NEAR(airSoundSpeed(air, 11000.0), 295.05, 0.01, "a at 11 km: 295.1 m/s");
    CHECK_TRUE(airSoundSpeed(air, 11000.0) < airSoundSpeed(air, 0.0) - 40.0,
               "the sound bar moves 45 m/s by the tropopause");
    CHECK_NEAR(300.0 / airSoundSpeed(air, 0.0), 0.8815, 0.01, "300 m/s at the datum: Mach 0.88");
    CHECK_NEAR(300.0 / airSoundSpeed(air, 11000.0), 1.0167, 0.01, "the same 300 m/s at 11 km: Mach 1.02");

    /* #245's flat floor is the SAME law under a layered body, not an exception that
       only exists on the exponential path: nothing gets denser under the sea floor. */
    CHECK_NEAR(airDensity(air, -500.0), 1.225, 0.0, "rho flat under the datum");
    CHECK_NEAR(airPressure(air, -500.0), airPressure(air, 0.0), 0.0, "p flat under it too");
    CHECK_NEAR(airTemperature(air, -500.0), 288.15, 0.0, "T flat under it too");

    /* And the air stops at the authored top -- height, because a layer table has no
       scale height for top() to multiply. */
    CHECK_NEAR(airDensity(air, 85000.0), 0.0, 0.0, "vacuum at the top");
    CHECK_NEAR(airPressure(air, 85000.0), 0.0, 0.0, "no pressure at the top");
    CHECK_NEAR(airTemperature(air, 85000.0), 0.0, 0.0, "no temperature at the top");
    CHECK_NEAR(airSoundSpeed(air, 200000.0), 0.0, 0.0, "no sound speed above it");

    /* A level boundary must not step the air. p side: base_p is filled in by
       integrating the level above to this level's base_alt, which is exactly where the
       reader switches levels, so level N+1's base_p is the top of level N by
       construction. T side: NOT automatic -- base_T is authored -- which is why
       resolveAtmoLevels refuses a seam wider than kAtmoSeamTol rather than trusting
       the author (see the refusals below; a 100 K seam is a 38% step in rho with p
       identical to 13 digits). */
    CHECK_NEAR(airDensity(air, 11000.0 - 1e-6), airDensity(air, 11000.0 + 1e-6), 1e-9,
               "no density cliff at a level boundary");
    CHECK_NEAR(airPressure(air, 20000.0 - 1e-6), airPressure(air, 20000.0 + 1e-6), 1e-9,
               "... nor where the lapse changes sign");
    CHECK_NEAR(airTemperature(air, 32000.0 - 1e-6), airTemperature(air, 32000.0 + 1e-6), 1e-9,
               "... nor mid-warming");

    /* Hydrostatic inside both spellings (the central difference costs ~(dh/H)^2/6,
       and H = Rs T / g is ~6.3 km in the tropopause, hence 1e-4). */
    const double dh = 100.0;
    CHECK_NEAR((airPressure(air, 20000.0 - dh) - airPressure(air, 20000.0 + dh)) / (2.0 * dh),
               airDensity(air, 20000.0) * 9.82, 1e-4, "-dp/dh == rho g inside a lapse level");
    CHECK_NEAR((airPressure(air, 15000.0 - dh) - airPressure(air, 15000.0 + dh)) / (2.0 * dh),
               airDensity(air, 15000.0) * 9.82, 1e-4, "-dp/dh == rho g inside an isothermal level");

    /* THE FORCE, on this path -- not just the density that feeds it. #245's real bug
       was a SECOND early-out in applyAeroForce that let rho print fine while the force
       stayed zero, so every profile shape needs the sub-datum force pin of its own. */
    const double cd = 1.2, area = 4.0;
    const glm::dvec3 vup(0.0, 300.0, 0.0);
    const glm::dvec3 f10k = dragForce(air, cd, area, 10000.0, vup);
    CHECK_NEAR(glm::length(f10k), 0.5 * airDensity(air, 10000.0) * cd * area * 90000.0,
               1e-9, "|F| == 0.5 rho cd A v^2 on a layered body");
    CHECK_TRUE(f10k.y < 0.0, "and it pushes against the motion at 10 km");
    CHECK_NEAR(glm::length(dragForce(air, cd, area, -1667.0, vup)),
               glm::length(dragForce(air, cd, area, 0.0, vup)), 1e-12,
               "#245's floor drags on the layered path too");
    CHECK_TRUE(dragForce(air, cd, area, 85000.0, vup) == glm::dvec3(0.0),
               "and stops at the authored top");

    /* THE POINT OF THE TABLE, as a number: density falls faster than pressure when
       the air has a lapse rate. At 20 km the layered profile puts 2.7x the air over
       you that Kerbin's single H = 5500 does (the 2.7x in drag.h's "THE THREE ARE
       LOCKED" block, measured rather than asserted), and LESS air than an isothermal
       column of the same sea-level density would. */
    const DragAtmosphere kerbin_exp = makeAir(1.225, 5500.0, 70000.0, 9.81);
    const DragAtmosphere earth_exp = makeAir(1.225, 8500.0, 0.0, 9.82);
    CHECK_NEAR(airDensity(air, 20000.0) / airDensity(kerbin_exp, 20000.0), 2.7, 0.01,
               "a lapse rate is what puts real air at 20 km over a scale-height body");
    CHECK_TRUE(airDensity(air, 20000.0) < airDensity(earth_exp, 20000.0),
               "lapse air thins faster than isothermal air of the same rho0");

    /* AND WHAT EARTH'S PLAYERS FEEL, since H 8500 is what Earth's file used to carry:
       the table is thicker air up to ~13 km and thinner above, so drag, q and the jet
       gate move on the way through the sky. This is the only place that delta is a
       number rather than a hope -- no e2e case flies a layered body yet (#631). */
    CHECK_NEAR(airDensity(air, 5000.0) / airDensity(earth_exp, 5000.0), 1.081, 0.005,
               "5 km: 8% thicker than the exponential Earth shipped with");
    CHECK_NEAR(airDensity(air, 10000.0) / airDensity(earth_exp, 10000.0), 1.091, 0.005,
               "10 km: 9% -- the peak");
    CHECK_NEAR(airDensity(air, 30000.0) / airDensity(earth_exp, 30000.0), 0.499, 0.01,
               "30 km: half of it");
    CHECK_NEAR(airDensity(air, 50000.0) / airDensity(earth_exp, 50000.0), 0.284, 0.01,
               "50 km: under a third");

    /* A layered profile weighs its column, so gravity DOES move its density: rho is
       p / (Rs T) and p comes from g. That is the one way A4 qualifies #552 A2's
       promise that gravity cannot move a drag number, and the direction is the honest
       one -- a scale height is authored, a column of air is weighed. Weigh it LESS
       and the column puffs out: the same sea-level density, thicker air aloft. */
    std::vector<AtmoLayer> Lhalf = L;
    CHECK_TRUE(resolveAtmoLevels(Lhalf, 1.225, kAirMolarMass, 4.91, 85000.0),
               "the same table resolves on a half-g body");
    DragAtmosphere halfg = air;
    halfg.gravity = 4.91;
    halfg.layers = Lhalf.data();
    halfg.layer_count = (int)Lhalf.size();
    CHECK_NEAR(airDensity(halfg, 0.0), 1.225, 0.0, "the datum is the same air either way");
    CHECK_TRUE(airDensity(halfg, 20000.0) > airDensity(air, 20000.0),
               "half the gravity: a lighter column, so a taller one -- thicker air aloft");

    /* The load invariants (atmo.h), refused rather than carried: each of these is a
       divide by zero or a negative temperature several steps later, and system.cpp
       turns a false into a message that names the body. */
    std::vector<AtmoLayer> off = { {1000.0, 288.15, -0.0065, 0.0},
                                   {11000.0, 216.65, 0.0, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(off, 1.225, kAirMolarMass, 9.82, 85000.0),
               "refused: level 0 must sit at the datum");
    std::vector<AtmoLayer> unsorted = { {0.0, 288.15, -0.0065, 0.0},
                                        {5000.0, 255.65, 0.0, 0.0},
                                        {4000.0, 260.0, 0.0, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(unsorted, 1.225, kAirMolarMass, 9.82, 85000.0),
               "refused: level altitudes must strictly rise");
    std::vector<AtmoLayer> cold = { {0.0, 288.15, -0.0065, 0.0}, {50000.0, 216.65, 0.0, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(cold, 1.225, kAirMolarMass, 9.82, 85000.0),
               "refused: a lapse must not walk T through 0 K inside the air");
    /* The seam. Both of these are two-level tables that integrate happily -- the
       pressure column is smooth through the boundary either way -- so nothing else
       catches them: rho steps because T is what rho divides by. */
    std::vector<AtmoLayer> seam = { {0.0, 288.15, -0.0065, 0.0}, {11000.0, 156.65, 0.0, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(seam, 1.225, kAirMolarMass, 9.82, 85000.0),
               "refused: a level must start where the one below leaves off (60 K gap)");
    std::vector<AtmoLayer> rounded = { {0.0, 288.15, -0.0065, 0.0}, {11000.0, 216.4, 0.0, 0.0} };
    CHECK_TRUE(resolveAtmoLevels(rounded, 1.225, kAirMolarMass, 9.82, 85000.0),
               "0.25 K off is rounding, not a mistake: kAtmoSeamTol accepts it");
    /* d(rho)/dh flips sign at lapse -g/Rs = -0.0342 K/m for this gas at this g. Air
       overturns long before that in reality; a table past it is a typo. */
    std::vector<AtmoLayer> upside = { {0.0, 288.15, -0.05, 0.0}, {3000.0, 138.15, 0.0, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(upside, 1.225, kAirMolarMass, 9.82, 5000.0),
               "refused: -0.05 K/m is past -g/Rs, so the air gets denser as you climb");
    /* A level above the air top is a level nothing can reach: drag.h cuts the air to
       vacuum at `height`, so it is dead weight that also decides the ceiling its own
       lapse is checked against. */
    std::vector<AtmoLayer> dead = L;
    CHECK_TRUE(!resolveAtmoLevels(dead, 1.225, kAirMolarMass, 9.82, 20000.0),
               "refused: a level based above height sits in declared vacuum");
    std::vector<AtmoLayer> one = { {0.0, 288.15, -0.0065, 0.0} };
    CHECK_TRUE(!resolveAtmoLevels(one, 0.0, kAirMolarMass, 9.82, 85000.0),
               "refused: nothing to anchor the column on");
    CHECK_TRUE(!resolveAtmoLevels(one, 1.225, kAirMolarMass, 0.0, 85000.0),
               "refused: no gravity named");
    CHECK_TRUE(!resolveAtmoLevels(one, 1.225, kAirMolarMass, 9.82, 0.0),
               "refused: no air top, so no ceiling to keep T above 0 K");
    std::vector<AtmoLayer> empty;
    CHECK_TRUE(!resolveAtmoLevels(empty, 1.225, kAirMolarMass, 9.82, 85000.0),
               "refused: an empty table is not a profile");

    /* A layered body with no g named fails CLOSED rather than integrating at 0. */
    DragAtmosphere nog { 1.225, 0.0, 85000.0 };
    nog.layers = L.data();
    nog.layer_count = (int)L.size();
    CHECK_NEAR(airDensity(nog, 5000.0), 0.0, 0.0, "layered but no gravity -> no air");
    CHECK_NEAR(airPressure(nog, 5000.0), 0.0, 0.0, "... and no pressure");
}

static void test_force() {
    printf("== dragForce: direction, magnitude, v^2 scaling ==\n");

    const DragAtmosphere a { 1.225, 5500.0 };
    const double cd = 1.2;
    const double A = 4.0;      // m^2
    const double alt = 100.0;  // m
    const double rho = airDensity(a, alt);

    // Straight up: the force is straight down, the right magnitude.
    const glm::dvec3 vup(0.0, 300.0, 0.0);
    const glm::dvec3 f_up = dragForce(a, cd, A, alt, vup);
    const double F_expect = 0.5 * rho * cd * A * 300.0 * 300.0;
    CHECK_NEAR(f_up.y, -F_expect, 1e-12, "F is opposite the motion");
    CHECK_NEAR(f_up.x, 0.0, 0.0, "no sideways component (1)");
    CHECK_NEAR(f_up.z, 0.0, 0.0, "no sideways component (2)");
    CHECK_NEAR(glm::length(f_up), F_expect, 1e-12, "|F| == 0.5 rho cd A v^2");

    // An oblique velocity: still exactly opposite it, same magnitude.
    const glm::dvec3 vob(100.0, 200.0, -100.0);
    const glm::dvec3 f_ob = dragForce(a, cd, A, alt, vob);
    const double v = glm::length(vob);
    const double F_ob = 0.5 * rho * cd * A * v * v;
    CHECK_NEAR(glm::length(f_ob), F_ob, 1e-12, "|F| at oblique v");
    // f_ob is anti-parallel to v_ob: the cross product is zero.
    CHECK_TRUE(glm::length(glm::cross(f_ob, vob)) < 1e-6 * F_ob,
               "F is anti-parallel to v");
    CHECK_TRUE(glm::dot(f_ob, vob) < 0.0, "F opposes v (dot < 0)");

    // Quadratic in speed: 2x speed -> 4x force.
    const glm::dvec3 f_2v = dragForce(a, cd, A, alt, vup * 2.0);
    CHECK_NEAR(glm::length(f_2v), 4.0 * glm::length(f_up), 1e-12,
               "|F(2v)| == 4 |F(v)|");

    // Degenerate inputs -> zero force (no guards needed at the call site).
    CHECK_TRUE(dragForce(a, cd, A, alt, glm::dvec3(0.0)) == glm::dvec3(0.0),
               "zero velocity -> zero force");
    CHECK_TRUE(dragForce(a, 0.0, A, alt, vup) == glm::dvec3(0.0),
               "cd=0 -> zero force");
    CHECK_TRUE(dragForce(a, cd, 0.0, alt, vup) == glm::dvec3(0.0),
               "area=0 -> zero force");
    const DragAtmosphere none { 0.0, 5500.0 };
    CHECK_TRUE(dragForce(none, cd, A, alt, vup) == glm::dvec3(0.0),
               "no atmosphere -> zero force");

    /* And the bill below the datum is the sea-level one (#245). The density pins in
       test_density cannot catch this on their own: the shipped bug had a SECOND
       early-out in applyAeroForce that refused the force while rho printed fine, so
       the force law needs a sub-datum pin of its own. */
    const glm::dvec3 f_sea = dragForce(a, cd, A, 0.0, vup);
    const glm::dvec3 f_sub = dragForce(a, cd, A, -1667.0, vup);
    CHECK_NEAR(glm::length(f_sub), glm::length(f_sea), 1e-12,
               "drag at -1667 m == drag at the datum");
    CHECK_TRUE(glm::length(f_sub) > 0.0, "sub-datum air pushes");
}

static void test_aeroFrame() {
    printf("== aeroFrame: alpha (pitch AoA), beta (sideslip) ==\n");
    // Ship axes in world: right=+X, up=+Y, nose=+Z (identity orientation).
    const glm::dvec3 right(1.0, 0.0, 0.0), up(0.0, 1.0, 0.0), nose(0.0, 0.0, 1.0);

    // Prograde: all zero, valid.
    AeroFrame f0 = aeroFrame(glm::dvec3(0.0, 0.0, 300.0), right, up, nose);
    CHECK_TRUE(f0.valid, "prograde is valid");
    CHECK_NEAR(f0.v, 300.0, 1e-12, "speed = |v|");
    CHECK_NEAR(f0.alpha, 0.0, 1e-12, "alpha 0 (prograde)");
    CHECK_NEAR(f0.beta, 0.0, 1e-12, "beta 0 (prograde)");

    // v leans +Y while the nose stays along +Z: the flight path is ABOVE the
    // nose, so the nose sits below it -> alpha = -45 deg.
    AeroFrame fp = aeroFrame(glm::dvec3(0.0, 300.0, 300.0), right, up, nose);
    CHECK_NEAR(fp.alpha, -M_PI / 4.0, 1e-12, "alpha = -45 deg for (0,300,300)");
    CHECK_NEAR(fp.beta, 0.0, 1e-12, "beta 0 (pure pitch)");

    // The convention that decides whether wings lift: nose pitched 20 deg above
    // a horizontal flight path -> alpha = +20 deg, so liftCurve() pushes the
    // airframe UP (see test_liftForce for the force half of that chain).
    {
        const double th = 20.0 * M_PI / 180.0;
        const glm::dvec3 pitched(0.0, std::sin(th), std::cos(th));
        const glm::dvec3 upT = glm::cross(pitched, right);  // the ship's +Y
        AeroFrame fu = aeroFrame(glm::dvec3(0.0, 0.0, 300.0), right, upT, pitched);
        CHECK_NEAR(fu.alpha, th, 1e-9, "nose above the flow -> alpha > 0");
    }

    // Pure bank: v has a +X component -> beta = atan2(vx, vz) = 45 deg.
    AeroFrame fb = aeroFrame(glm::dvec3(300.0, 0.0, 300.0), right, up, nose);
    CHECK_NEAR(fb.beta, M_PI / 4.0, 1e-12, "beta = 45 deg for (300,0,300)");
    CHECK_NEAR(fb.alpha, 0.0, 1e-12, "alpha 0 (pure bank)");

    // Degenerate: zero velocity -> not valid, all zero.
    AeroFrame fn = aeroFrame(glm::dvec3(0.0), right, up, nose);
    CHECK_TRUE(!fn.valid, "zero v -> not valid");
}

static void test_liftDirection() {
    printf("== liftDirection: the wing normal, out of the flow ==\n");
    const glm::dvec3 right(1.0, 0.0, 0.0), nose(0.0, 0.0, 1.0);  // identity

    // Prograde (flow along the nose): the lift direction is the up axis.
    const glm::dvec3 ld0 = liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                                         right, nose);
    CHECK_NEAR(ld0.x, 0.0, 0.0, "prograde liftDir x == 0");
    CHECK_NEAR(ld0.y, 1.0, 1e-12, "prograde liftDir y == 1 (up)");
    CHECK_NEAR(ld0.z, 0.0, 0.0, "prograde liftDir z == 0");
    CHECK_NEAR(glm::length(ld0), 1.0, 1e-12, "liftDir is a unit vector");

    // Always perpendicular to the flow (lift is the force OUT of the flow).
    {
        const glm::dvec3 v(300.0, 100.0, 200.0);
        const glm::dvec3 ld = liftDirection(v, right, nose);
        CHECK_TRUE(std::fabs(glm::dot(ld, v)) < 1e-9,
                   "liftDir is perpendicular to v");
        CHECK_NEAR(glm::length(ld), 1.0, 1e-12, "deflected liftDir is unit");
    }

    // A banked ship: the lift direction follows the (banked) up axis, not
    // the world up. Rolled 90 deg about the nose: right=+Y, up=nose x right.
    {
        const glm::dvec3 right2(0.0, 1.0, 0.0), nose2(0.0, 0.0, 1.0);
        const glm::dvec3 v(0.0, 0.0, 300.0);  // prograde for this banked ship
        const glm::dvec3 ld2 = liftDirection(v, right2, nose2);
        const glm::dvec3 up2 = glm::cross(nose2, right2);  // = -X
        CHECK_NEAR(glm::length(ld2 - up2), 0.0, 1e-12,
                   "banked liftDir follows the banked up axis");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(liftDirection(glm::dvec3(0.0), right, nose) == glm::dvec3(0.0),
               "zero v -> zero");
    CHECK_TRUE(liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                             glm::dvec3(0.0), nose) == glm::dvec3(0.0),
               "zero right -> zero");
    CHECK_TRUE(liftDirection(glm::dvec3(0.0, 0.0, 300.0),
                             right, glm::dvec3(0.0)) == glm::dvec3(0.0),
               "zero nose -> zero");
}

static void test_liftForce() {
    printf("== liftForce: |L| = q*S*CL(a), sign follows a, stalls past A ==\n");
    const glm::dvec3 liftDir(0.0, 1.0, 0.0);  // up
    const double q = 60.0;      // dynamic pressure (Pa)
    const double S = 4.0;       // m^2
    const double cl = 6.0;      // per radian
    const double alpha = 0.10;  // rad
    const double L_expect = q * S * cl * alpha;

    // Positive AoA: the lift is along liftDir, the right magnitude.
    const glm::dvec3 L = liftForce(q, S, cl, alpha, liftDir);
    CHECK_NEAR(L.y, L_expect, 1e-12, "|L| == q*S*cl*alpha (positive AoA)");
    CHECK_NEAR(L.x, 0.0, 0.0, "lift has no x component");
    CHECK_NEAR(L.z, 0.0, 0.0, "lift has no z component");

    // Negative AoA: the same magnitude, opposite direction (down).
    const glm::dvec3 Ln = liftForce(q, S, cl, -alpha, liftDir);
    CHECK_NEAR(Ln.y, -L_expect, 1e-12, "negative AoA pushes down");

    // Zero AoA: a symmetric section (CL0 = 0) generates no lift.
    CHECK_TRUE(liftForce(q, S, cl, 0.0, liftDir) == glm::dvec3(0.0),
               "zero AoA -> zero lift");

    // With a stall angle A: below stall it is still the linear law, and at
    // 2A it is a deep stall (zero lift) -- the vector path honours liftCurve.
    {
        const double A = 0.20;
        const glm::dvec3 L_lin  = liftForce(q, S, cl, 0.10, liftDir, A);
        CHECK_NEAR(L_lin.y, q * S * cl * 0.10, 1e-12, "below stall: linear");
        const glm::dvec3 L_deep = liftForce(q, S, cl, 2.0 * A, liftDir, A);
        CHECK_TRUE(L_deep == glm::dvec3(0.0), "deep stall (2A) -> zero lift");
    }

    // Degenerate inputs -> zero.
    CHECK_TRUE(liftForce(0.0, S, cl, alpha, liftDir) == glm::dvec3(0.0),
               "no dynamic pressure -> zero");
    CHECK_TRUE(liftForce(q, 0.0, cl, alpha, liftDir) == glm::dvec3(0.0),
               "no lift area -> zero");
    CHECK_TRUE(liftForce(q, S, 0.0, alpha, liftDir) == glm::dvec3(0.0),
               "no cl (a rocket) -> zero");
}

/* The whole chain, from a real geometry to a force in world axes.
   test_aeroFrame pins the ANGLE and test_liftForce pins the FORCE given an
   angle, but the bug either half alone cannot see: a flipped AoA sign still
   passes both, because liftCurve() is odd in alpha, so the mirrored angle
   yields a mirrored force and every pairwise relation still agrees (#448).
   Only the composition says which way a wing actually pushes: a wing whose
   nose sits ABOVE the air it ploughs through must push the airframe skyward. */
static void test_wing_lifts_up() {
    printf("== chain: nose above the flow -> the wing pushes UP ==\n");
    // World: local up is +Y, the ship flies horizontally along +Z at 100 m/s.
    const glm::dvec3 right(1.0, 0.0, 0.0);
    const glm::dvec3 vrel(0.0, 0.0, 100.0);
    const double q = 0.5 * 1.225 * 100.0 * 100.0;  // sea-level Kerbin-ish

    for(double th_deg : {5.0, 15.0}) {
        const double th = th_deg * M_PI / 180.0;
        // The ship's own frame, pitched th ABOVE the flight path.
        const glm::dvec3 nose(0.0, std::sin(th), std::cos(th));
        const glm::dvec3 up   = glm::cross(nose, right);  // the ship's +Y
        const AeroFrame fr = aeroFrame(vrel, right, up, nose);
        CHECK_NEAR(fr.alpha, th, 1e-9, "AoA = the pitch above the flow");

        const glm::dvec3 ld = liftDirection(vrel, right, nose);
        CHECK_NEAR(glm::dot(ld, glm::dvec3(0.0, 1.0, 0.0)), 1.0, 1e-9,
                   "the wing normal points at the sky");
        const glm::dvec3 L = liftForce(q, 4.0, 6.0, fr.alpha, ld, 0.35);
        CHECK_TRUE(L.y > 0.0, "a wing above the flow LIFTS");

        // The mirror: fly with the nose BELOW the flow (a dive) and the same
        // wing drives the airframe down.
        const glm::dvec3 noseD(0.0, -std::sin(th), std::cos(th));
        const glm::dvec3 upD = glm::cross(noseD, right);
        const AeroFrame fd = aeroFrame(vrel, right, upD, noseD);
        const glm::dvec3 Ld = liftForce(q, 4.0, 6.0, fd.alpha,
                                        liftDirection(vrel, right, noseD), 0.35);
        CHECK_NEAR(fd.alpha, -th, 1e-9, "nose below the flow -> AoA < 0");
        CHECK_TRUE(Ld.y < 0.0, "a wing below the flow pushes down");

        // And the same world answer from an AIRFRAME FLOPPED UPSIDE DOWN: roll
        // 180 deg so its up axis points at the ground, keep the nose above the
        // flow. The body AoA and the wing normal both flip, so the force does
        // not -- which is the half a flipped alpha cannot fake, and what breaks
        // if alpha's plane and liftDirection() ever stop agreeing.
        const glm::dvec3 rightInv(-1.0, 0.0, 0.0);
        const glm::dvec3 noseInv(0.0, std::sin(th), std::cos(th));
        const glm::dvec3 upInv = glm::cross(noseInv, rightInv);
        CHECK_TRUE(upInv.y < 0.0, "rolled airframe: its up axis points down");
        const AeroFrame fi = aeroFrame(vrel, rightInv, upInv, noseInv);
        const glm::dvec3 li = liftDirection(vrel, rightInv, noseInv);
        CHECK_TRUE(li.y < 0.0, "the rolled wing normal points at the ground");
        const glm::dvec3 Li = liftForce(q, 4.0, 6.0, fi.alpha, li, 0.35);
        CHECK_NEAR(fi.alpha, -th, 1e-9, "rolled: nose above flow reads AoA < 0");
        CHECK_TRUE(Li.y > 0.0, "rolled airframe, nose above the flow: still LIFTS");
    }
}

/* #453: each surface reads the flow in its OWN frame. Three pins, and the third is
   what stops the first from being a tautology:
   - an ALIGNED mount agrees with the ship-wide frame exactly, so the shipped
     airframes do not move when the law starts asking;
   - a mount with incidence sees a different angle, and a mount with dihedral gets a
     tilted force -- the capability #453 exists to unlock;
   - a wing turned 180 deg about its own nose really does see the mirrored angle and
     the mirrored direction, and the FORCE still comes out identical. That last is
     why jet.json's wing_2 (attach angle 180) keeps lifting the same way: the two
     flips cancel. A flight log cannot tell that state from the law being broken,
     so it lives here. */
static void test_partAero() {
    printf("== partAero: each surface sees the flow in its own frame (#453) ==\n");
    /* Some pins below demand EXACT equality (tolerance 0 against 0.0). That is
       deliberate: the claims are "an aligned mount sees the same number" and "a
       mount flip negates this", both of which hold bitwise for these frames, and
       exactness is what says shipped flight did not move. If a future change to
       liftForce/aeroFrame regroups the arithmetic and one goes red, widen it --
       that is a numerics change, not a physics one. */
    const double q = 0.5 * 1.225 * 100.0 * 100.0;
    const double S = 4.0, cl = 6.0, stall = 0.35;
    // Flying 5 deg nose-up along +Z: the air moves with +Z and leans DOWN, so
    // dot(v_rel, up) < 0 for a nose above the flow (see test_wing_lifts_up).
    const double aoa = glm::radians(5.0);
    const glm::dvec3 vrel(0.0, -100.0 * std::sin(aoa), 100.0 * std::cos(aoa));
    const glm::dmat3 ship(1.0);   // part axes == world axes here

    const PartAero base = partAero(ship, vrel);
    CHECK_TRUE(base.valid, "an aligned mount has a usable frame");
    CHECK_NEAR(base.alpha, aoa, 1e-12, "an aligned mount sees the ship's AoA");
    const glm::dvec3 baseF = liftForce(q, S, cl, base.alpha, base.liftDir, stall);
    const glm::dvec3 shipF = liftForce(q, S, cl, aoa,
                                       liftDirection(vrel, ship[0], ship[2]), stall);
    CHECK_NEAR(glm::length(baseF - shipF), 0.0, 0.0,
               "an aligned mount lifts exactly as the ship-wide frame did");

    // 180 deg about the part's own nose -- how jet.json's wing_2 is mounted.
    {
        const glm::dmat3 flipped(-ship[0], -ship[1], ship[2]);
        const PartAero f = partAero(flipped, vrel);
        CHECK_TRUE(f.valid, "a flipped mount still has a usable frame");
        CHECK_NEAR(f.alpha, -base.alpha, 1e-12, "a 180-deg roll sees the mirrored AoA");
        CHECK_NEAR(glm::length(f.liftDir + base.liftDir), 0.0, 0.0,
                   "and the mirrored lift direction");
        const glm::dvec3 F = liftForce(q, S, cl, f.alpha, f.liftDir, stall);
        CHECK_NEAR(glm::length(F - baseF), 0.0, 0.0,
                   "both flips cancel: the force is unchanged, so shipped lifts move 0");
    }

    // Incidence: the nose 10 deg above the mount's own, i.e. a wing bolted with a
    // wedge. It sees AoA + 10, so it lifts harder at the same attitude.
    {
        const double inc = glm::radians(10.0);
        const glm::dmat3 pitched(
            ship[0],
            std::cos(inc) * ship[1] - std::sin(inc) * ship[2],
            std::sin(inc) * ship[1] + std::cos(inc) * ship[2]);
        const PartAero p = partAero(pitched, vrel);
        CHECK_TRUE(p.valid, "an incident mount has a usable frame");
        CHECK_NEAR(p.alpha, aoa + inc, 1e-12, "incidence adds to the angle of attack");
        CHECK_TRUE(glm::length(p.liftDir - base.liftDir) < 1e-9,
                   "pitch about the mount's own right axis keeps the lift vertical");
        const glm::dvec3 F = liftForce(q, S, cl, p.alpha, p.liftDir, stall);
        CHECK_NEAR(F.y / baseF.y, (cl * (aoa + inc)) / (cl * aoa), 1e-9,
                   "and the wing lifts by its OWN angle");
    }

    // Dihedral: the wing tilted up about the ship's nose axis. Its force tilts out
    // of the vertical -- that tilt IS dihedral stability -- and a mirrored pair
    // still cancels sideways, which is what keeps a symmetric airframe level.
    {
        const double dh = glm::radians(10.0);
        const glm::dmat3 rl(std::cos(dh) * ship[0] + std::sin(dh) * ship[1],
                           -std::sin(dh) * ship[0] + std::cos(dh) * ship[1],
                           ship[2]);
        const glm::dmat3 rr(std::cos(dh) * ship[0] - std::sin(dh) * ship[1],
                            std::sin(dh) * ship[0] + std::cos(dh) * ship[1],
                            ship[2]);
        const PartAero pl = partAero(rl, vrel), pr = partAero(rr, vrel);
        CHECK_TRUE(pl.valid && pr.valid, "dihedral mounts have usable frames");
        const glm::dvec3 Fl = liftForce(q, S, cl, pl.alpha, pl.liftDir, stall);
        const glm::dvec3 Fr = liftForce(q, S, cl, pr.alpha, pr.liftDir, stall);
        CHECK_TRUE(std::fabs(Fl.x) > 0.01 * std::fabs(Fl.y),
                   "a dihedral wing's force tilts out of the vertical");
        CHECK_NEAR(Fl.x + Fr.x, 0.0, 1e-12, "a mirrored dihedral pair cancels sideways");
        CHECK_NEAR(pl.alpha, pr.alpha, 1e-12, "and sees the same angle either way");
    }

    // Unusable frames lift nothing rather than lifting on a garbage axis.
    CHECK_TRUE(!partAero(glm::dmat3(0.0), vrel).valid,
               "a mount with no axes is refused, not guessed");
    CHECK_TRUE(!partAero(ship, glm::dvec3(0.0)).valid, "no flow -> no frame");
    {
        // Flow straight up the mount's own up axis: the lift direction is undefined.
        const PartAero e = partAero(ship, glm::dvec3(0.0, 100.0, 0.0));
        CHECK_TRUE(!e.valid, "a surface with no lift direction is refused");
        CHECK_NEAR(glm::length(e.liftDir), 0.0, 0.0, "and carries no direction out");
    }
}

/* #485: the per-surface ANGLE READOUT. alpha comes out of each surface's own mount
   frame, so a twin pair mounted at attach angle 0 and 180 (jet.json's wing_1/wing_2,
   the skiff's tail_l/tail_r, the glider's rudders at 90/270) carries mirrored signs:
   +2.6 and -2.6 for two surfaces at one physical angle. That is a display bug, not a
   law bug -- the two flips cancel in the FORCE, which is what #453 pinned -- and it is
   worse than showing nothing, because it buries the one difference worth looking at:
   a pair whose angles ACTUALLY differ (one half swept into the flow by a roll or a
   yaw, #483). surfaceAlphaDegForReadout signs the number into ship axes instead. Three
   pins: a mirrored pair reads alike; the sign is the sign of the force that surface
   makes along that ship axis (positive = pushes up for a wing, right for a fin); and a
   real difference still shows. None of this is legible in a flight log -- the log's
   totals are the same whatever the readout prints -- so it lives here. */
static void test_surface_alpha_readout() {
    printf("== surface alpha readout: ship axes, not each mount's own (#485) ==\n");
    const double q = 0.5 * 1.225 * 100.0 * 100.0, S = 4.0, cl = 6.0, stall = 0.35;
    const double aoa = glm::radians(5.0);
    const glm::dvec3 vrel(0.0, -100.0 * std::sin(aoa), 100.0 * std::cos(aoa));
    const glm::dmat3 ship(1.0);
    const glm::dvec3 up(0.0, 1.0, 0.0), right(1.0, 0.0, 0.0);
    /* The axis question, the same helper the aero step calls. In this test the
       mounts are already in the root's frame, so up/right are both the body
       vectors and the world vectors. */
    const auto frameOf = [&](const glm::dmat3 &m) {
        return surfaceReadoutFrame(m[1], up, right);
    };

    // A wing and its twin at attach angle 180: opposite frames, one physical angle.
    {
        const glm::dmat3 twin(-ship[0], -ship[1], ship[2]);   // the same plate at clock 180
        const PartAero a = partAero(ship, vrel);
        const PartAero b = partAero(twin, vrel);
        CHECK_NEAR(surfaceAlphaDegForReadout(a, frameOf(ship), up, right), 5.0, 1e-9,
                   "an aligned wing reads its own AoA, in degrees");
        CHECK_NEAR(surfaceAlphaDegForReadout(b, frameOf(twin), up, right),
                   surfaceAlphaDegForReadout(a, frameOf(ship), up, right), 1e-9,
                   "and its 180-deg twin reads the SAME number, not the mirror");
        // What the sign MEANS: the sign of the force, in the same ship axis the
        // number is signed against. Without this, "+2.6" is a coin flip per part.
        const glm::dvec3 Fa = liftForce(q, S, cl, a.alpha, a.liftDir, stall);
        const glm::dvec3 Fb = liftForce(q, S, cl, b.alpha, b.liftDir, stall);
        CHECK_TRUE(glm::dot(Fa, up) > 0.0 && glm::dot(Fb, up) > 0.0,
                   "positive readout = the surface pushes ship-up");
    }

    // A fin: the plate turned 90 deg about the ship's nose, so its span stands along
    // ship up and its normal faces ship's left; the mirror is a fin at 270 (the
    // glider's rudder pair). Yaw the flow 10 deg and both must read the same 10 deg,
    // signed against ship's RIGHT -- the axis a fin moves the ship along.
    {
        const double psi = glm::radians(10.0);
        const glm::dvec3 vyaw(100.0 * std::sin(psi), 0.0, 100.0 * std::cos(psi));
        const glm::dmat3 fin(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
        const glm::dmat3 fin2(glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0),
                              glm::dvec3(0.0, 0.0, 1.0));
        const PartAero pa = partAero(fin, vyaw), pb = partAero(fin2, vyaw);
        CHECK_TRUE(pa.valid && pb.valid, "both fins of a 90/270 pair have frames");
        CHECK_NEAR(std::fabs(surfaceAlphaDegForReadout(pa, frameOf(fin), up, right)), 10.0,
                   1e-9, "a yawed flow gives a fin its yaw angle, not the ship's AoA");
        CHECK_NEAR(surfaceAlphaDegForReadout(pa, frameOf(fin), up, right),
                   surfaceAlphaDegForReadout(pb, frameOf(fin2), up, right), 1e-9,
                   "and a 90/270 pair reads alike");
        const glm::dvec3 Fa = liftForce(q, S, cl, pa.alpha, pa.liftDir, stall);
        const double sr = glm::dot(Fa, right);
        CHECK_TRUE(std::fabs(sr) > 0.01 * glm::length(Fa),
                   "a fin's force runs along ship's right");
        CHECK_TRUE(sr * surfaceAlphaDegForReadout(pa, frameOf(fin), up, right) > 0.0,
                   "and its sign is the sign of the force along that axis");
    }

    /* A mount 45 deg out of vertical -- half a v-tail, the whole fin mount rotated about
       the ship's nose -- is where the readout's TWO decisions come apart: WHICH SHIP AXIS
       the surface pushes along (the MOUNT's answer) and WHICH WAY ALONG IT (the wind's).
       Pin both, and pin what going wrong on either one costs. Measured over an
       810-direction flow sweep on such a mount: taking the axis from the live liftDir
       instead of the mount changes the axis in 459 of those flows; taking the sign from
       the mount instead of the live force contradicts the force the law actually bills in
       284 of them. Either mistake makes the printed number disagree with the arrow the F4
       overlay draws next to it (#469). */
    {
        const glm::dmat3 finBase(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0),
                                 glm::dvec3(0.0, 0.0, 1.0));
        const glm::dvec3 nose(0.0, 0.0, 1.0);
        for(double tiltDeg : {-45.0, 45.0}) {
            const glm::dmat3 vtTail =
                glm::mat3_cast(glm::angleAxis(glm::radians(tiltDeg), nose)) * finBase;
            const SurfaceReadout rf = frameOf(vtTail);
            const glm::dvec3 axis = rf.axisUp ? up : right;
            int samples = 0, flowAxisFlip = 0, rigSignBad = 0, forceBad = 0, magBad = 0;
            int billed = 0;   // flows where the law actually makes a force to agree with
            for(int azDeg = 0; azDeg <= 178; azDeg += 2) {
                for(int elevDeg = -60; elevDeg <= 60; elevDeg += 15) {
                    const double az = glm::radians((double)azDeg);
                    const double el = glm::radians((double)elevDeg);
                    const glm::dvec3 v = 100.0 * glm::dvec3(std::sin(az) * std::cos(el),
                                                            std::sin(el),
                                                            std::cos(az) * std::cos(el));
                    const PartAero pa = partAero(vtTail, v);
                    if(!pa.valid) { continue; }
                    ++samples;
                    const double readout = surfaceAlphaDegForReadout(pa, rf, up, right);
                    /* The contract against the force the LAW bills, not against the
                       helper's own projection. On most of these flows the plate sits past
                       twice its stall angle and liftCurve answers exactly zero, so there
                       is no force to agree or disagree with -- count those separately and
                       pin that the disagreement test has real samples behind it. */
                    const glm::dvec3 F = liftForce(q, S, cl, pa.alpha, pa.liftDir, stall);
                    if(glm::length(F) > 0.0) {
                        ++billed;
                        if(glm::dot(F, axis) * readout < 0.0) { ++forceBad; }
                    }
                    if(std::fabs(std::fabs(readout) - std::fabs(glm::degrees(pa.alpha)))
                           > 1e-12) { ++magBad; }
                    // And what the two mistakes would do, counted on the same sweep.
                    if((std::fabs(glm::dot(pa.liftDir, up))
                            >= std::fabs(glm::dot(pa.liftDir, right))) != rf.axisUp) {
                        ++flowAxisFlip;
                    }
                    if(rf.rigSign * glm::degrees(pa.alpha)
                           * glm::dot(pa.liftDir, axis) < 0.0) {
                        ++rigSignBad;
                    }
                }
            }
            CHECK_TRUE(samples == 810, "the sweep sampled every flow direction it claims");
            CHECK_TRUE(billed > 100, "enough of those flows make a real force to pin");
            CHECK_TRUE(magBad == 0, "the readout's magnitude is the surface's own angle");
            CHECK_TRUE(forceBad == 0,
                       "and its sign is the sign of the force that surface actually makes");
            CHECK_TRUE(flowAxisFlip == 459,
                       "choosing the axis from the flow would change it 459 times here");
            CHECK_TRUE(rigSignBad > 100,
                       "signing the angle from the mount would contradict the force");
        }

        /* THE AXIS ANSWER IS A BUILD FACT, which is why the call site asks it with the
           part's LOCAL mount rather than with world vectors. Both halves of a +-45 v-tail
           sit exactly on the tie -- |dot(push,up)| and |dot(push,right)| differ by 1e-16 of
           rounding -- and the tie answers UP. Ask the same question in the world frame (the
           mount and the axes both multiplied by the ship's rotation, as an earlier revision
           of this line did) and the answer rides the ship's own attitude: over a 6-degree
           grid of 9300 attitudes it flips on 1578 of them for this mount. A mount off the
           tie (the same fin at 90 deg) never flips. Pinned so nobody "simplifies" the call
           site back to the world frame. */
        const glm::dmat3 vtA = glm::mat3_cast(glm::angleAxis(glm::radians(-45.0), nose))
                                 * finBase;
        const glm::dmat3 vtB = glm::mat3_cast(glm::angleAxis(glm::radians(45.0), nose))
                                 * finBase;
        const glm::dmat3 fin90 = glm::mat3_cast(glm::angleAxis(glm::radians(90.0), nose))
                                   * finBase;
        const SurfaceReadout rfA = frameOf(vtA), rfB = frameOf(vtB);
        CHECK_TRUE(rfA.axisUp && rfB.axisUp, "a tie mount ties to the UP axis");
        CHECK_TRUE(rfA.rigSign * rfB.rigSign < 0.0,
                   "and the halves' riggings push opposite ways along it (the fallback's sign)");
        CHECK_TRUE(std::fabs(std::fabs(glm::dot(vtA[1], up))
                             - std::fabs(glm::dot(vtA[1], right))) < 1e-12
                       && std::fabs(std::fabs(glm::dot(vtB[1], up))
                                    - std::fabs(glm::dot(vtB[1], right))) < 1e-12,
                   "and a +-45 mount really is on the tie (absolute window: CHECK_NEAR's "
                   "tolerance is RELATIVE, so against 0.0 it means bit-exact)");
        int attitudes = 0, flipTie = 0, flipSquare = 0;
        for(int pitch = -90; pitch <= 90; pitch += 6) {
            for(int yaw = 0; yaw < 360; yaw += 6) {
                for(int roll = -60; roll <= 60; roll += 30) {
                    const glm::dmat3 sRot = glm::mat3_cast(
                        glm::angleAxis(glm::radians((double)yaw), up)
                        * glm::angleAxis(glm::radians((double)pitch), right)
                        * glm::angleAxis(glm::radians((double)roll), nose));
                    const auto worldChoice = [&](const glm::dmat3 &localMount) {
                        const glm::dvec3 push = sRot * localMount[1];
                        return std::fabs(glm::dot(push, sRot * up))
                            >= std::fabs(glm::dot(push, sRot * right));
                    };
                    ++attitudes;
                    if(worldChoice(vtA) != rfA.axisUp) { ++flipTie; }
                    if(worldChoice(fin90) != frameOf(fin90).axisUp) { ++flipSquare; }
                }
            }
        }
        CHECK_TRUE(attitudes == 9300, "the attitude grid sampled what it claims");
        CHECK_TRUE(flipTie > 100,
                   "asked in the world frame, a tie mount's axis rides the attitude");
        CHECK_TRUE(flipSquare == 0, "off the tie, the body and world framings agree always");

        /* And the outcome this whole readout exists for, on the mount least like a shipped
           one: the two halves of a +-45 v-tail tie on the axis choice, and their pushes are
           mirrored, so with the tie-break (UP) taken in the body frame they print the SAME
           number for the same flow. Quote one of them against RIGHT instead and that half
           flips sign, which is why the tie-break is stated rather than left to rounding. */
        const glm::dvec3 v10(0.0, -100.0 * std::sin(glm::radians(10.0)),
                             100.0 * std::cos(glm::radians(10.0)));
        const double ra = surfaceAlphaDegForReadout(partAero(vtA, v10), rfA, up, right);
        const double rb = surfaceAlphaDegForReadout(partAero(vtB, v10), rfB, up, right);
        CHECK_NEAR(ra, rb, 1e-9, "a +-45 v-tail pair prints the SAME number");
        CHECK_TRUE(std::fabs(ra) > 1.0, "and it is a real angle, not the tie pinning zero");
        // The tie-break is a decision, not a no-op: quoting the -45 half against RIGHT
        // instead of UP flips its sign (its force projects oppositely on the two axes).
        const PartAero pa45 = partAero(vtA, v10);
        const SurfaceReadout quoteRight{false, glm::dot(vtA[1], right)};
        CHECK_TRUE(surfaceAlphaDegForReadout(pa45, quoteRight, up, right) * ra < 0.0,
                   "the tie-break (UP) decides that half's sign");
    }

    // A pair that genuinely differs must still DIFFER: a rolling airframe sweeps one
    // wing up and the other down through the air (#483), which is roll damping, and
    // the readout is how a player would SEE it. Same mount, different local flow.
    {
        const glm::dvec3 w(0.0, 0.0, 1.0);        // 1 rad/s about the nose
        const glm::dvec3 v0(0.0, 0.0, 100.0);     // level flight, AoA 0
        const LocalFlow la = localFlow(v0, w, glm::dvec3(3.0, 0.0, 0.0), 1.225);
        const LocalFlow lb = localFlow(v0, w, glm::dvec3(-3.0, 0.0, 0.0), 1.225);
        const double ra = surfaceAlphaDegForReadout(partAero(ship, la.vp),
                                                    frameOf(ship), up, right);
        const double rb = surfaceAlphaDegForReadout(partAero(ship, lb.vp),
                                                    frameOf(ship), up, right);
        CHECK_TRUE(std::fabs(ra - rb) > 1.0,
                   "a rolling airframe's two wings read DIFFERENT angles");
        CHECK_NEAR(ra + rb, 0.0, 1e-9, "equal and opposite, because the roll is");
    }

    // An unusable frame reads 0.0 out of the struct's defaults. That is NOT a claim
    // that the surface sits at zero -- tick.cpp declines to quote such a part at all
    // -- so callers gate on Part::aeroValid; this only pins there is nothing to sign.
    CHECK_NEAR(surfaceAlphaDegForReadout(partAero(glm::dmat3(0.0), vrel),
                                         frameOf(ship), up, right),
               0.0, 1e-12, "a refused frame carries no angle to sign");

    /* THE ZERO-PROJECTION FALLBACK: a plate mounted fore-aft (normal along the ship's nose
       -- the mis-rig family #509 flags) in a flow that has no component on ship's up. Its
       live liftDir comes out exactly perpendicular to up, so the wind offers no sign for
       the axis being quoted. The readout keeps the ANGLE (90 deg, which is what the plate
       actually sits at) and takes the sign from the rigging; returning 0.0 would have
       invented an angle out of a display detail. Note that this mount projects on NEITHER
       ship axis, so its rigSign is 0 and the fallback's sign is a convention -- a part like
       this has no readout worth quoting, which is another reason #509 wants it flagged. */
    {
        const glm::dmat3 fin0(glm::dvec3(1,0,0), glm::dvec3(0,0,1), glm::dvec3(0,-1,0));
        const glm::dvec3 v(100.0 * std::sin(glm::radians(20.0)), 0.0,
                           100.0 * std::cos(glm::radians(20.0)));
        const PartAero a = partAero(fin0, v);
        const SurfaceReadout rf = frameOf(fin0);
        CHECK_TRUE(a.valid && std::fabs(glm::degrees(a.alpha)) > 1.0,
                   "a fore-aft fin 20 deg off the nose still has a frame and a real angle");
        CHECK_TRUE(glm::dot(a.liftDir, rf.axisUp ? up : right) == 0.0,
                   "and its live force has no component on the quoted axis (exact, not tiny)");
        CHECK_NEAR(std::fabs(surfaceAlphaDegForReadout(a, rf, up, right)), 90.0, 1e-9,
                   "the readout still quotes the 90 deg it is actually at");
        CHECK_TRUE(rf.rigSign == 0.0,
                   "(a nose-projecting mount answers neither axis question, as documented)");
    }

    /* A NaN flow is not a usable frame. partAero used to accept one -- `dot(l,l) <= 0` is
       FALSE for NaN -- and handed the whole aero pass a NaN force, which vehicle.cpp's own
       kAoAFlowFloor comment had been saying all along. */
    {
        const glm::dvec3 bad = glm::dvec3(std::nan(""), 100.0, 0.0);
        CHECK_TRUE(!partAero(ship, bad).valid, "partAero refuses a NaN flow");
        const glm::dmat3 fin(glm::dvec3(0,1,0), glm::dvec3(-1,0,0), glm::dvec3(0,0,1));
        const PartAero paNaNFlow = partAero(fin, bad);
        CHECK_TRUE(controlDeflection(paNaNFlow, glm::dvec3(0,0,-3.0), fin, up, -1.0,
                                     1.0, 0.35) == 0.0,
                   "so the control path never sees the garbage");
        const PartAero paOk = partAero(fin, glm::dvec3(0.0, -34.2, 93.97));
        CHECK_TRUE(paOk.valid, "the same fin in a real flow has a frame");
        CHECK_TRUE(controlDeflection(paOk, glm::dvec3(0,0,std::nan("")), fin, up, -1.0,
                                     1.0, 0.35) == 0.0,
                   "and a NaN mount distance fails CLOSED rather than granting travel");
    }

    /* For a mount whose push IS a ship axis -- every shipped control surface, authored at
       clock 0/90/180/270 -- the axis choice cannot disagree with the old live-liftDir rule
       until the flow is far off the plate's own plane. That band is why the shipped beds
       log byte-identical [surface] rows either way, and it is narrower than it feels: the
       glider's held-rudder bed reaches 62.7 deg of sideslip, and the flip starts near 64. */
    {
        const glm::dmat3 rudder(glm::dvec3(0,1,0), glm::dvec3(-1,0,0), glm::dvec3(0,0,1));
        const SurfaceReadout rf = frameOf(rudder);
        CHECK_TRUE(!rf.axisUp, "a shipped rudder's readout is quoted against ship's right");
        int inBand = 0, outBand = 0, flipIn = 0, flipOut = 0;
        for(int beta = 0; beta <= 80; beta += 2) {
            for(int aoa = -20; aoa <= 20; aoa += 5) {
                const double b = glm::radians((double)beta), e = glm::radians((double)aoa);
                const glm::dvec3 v = 100.0 * glm::dvec3(std::sin(b) * std::cos(e),
                                                        -std::sin(e),
                                                        std::cos(b) * std::cos(e));
                const PartAero pa = partAero(rudder, v);
                if(!pa.valid) { continue; }
                const bool liveUp = std::fabs(glm::dot(pa.liftDir, up))
                                 >= std::fabs(glm::dot(pa.liftDir, right));
                if(beta <= 60) { ++inBand; if(liveUp != rf.axisUp) { ++flipIn; } }
                else { ++outBand; if(liveUp != rf.axisUp) { ++flipOut; } }
            }
        }
        CHECK_TRUE(inBand > 200 && outBand > 40, "the sideslip sweep sampled both bands");
        CHECK_TRUE(flipIn == 0, "within +-60 deg of sideslip the two rules pick the same axis");
        CHECK_TRUE(flipOut > 0, "past that the old live rule starts picking the other axis");
    }
}

/* #454: the neutral point -- where an angle-of-attack change puts the CHANGE in lift.
   The arithmetic is a weighted average, but the sign conventions are the whole risk,
   and one of them was got wrong first: the production call site weighted each surface
   by slope*area*dot(liftDir, up), which makes a mirrored pair CANCEL exactly (its
   liftDir and its alpha are both flipped), leaving a denominator of ~0 and a drag log
   quoting Np=+15.9 m, then +121.5, then -89.7 for the same skiff. liftSlopePerQ asks
   the law instead of assuming a sign, and these are the cases that say so. */
static void test_neutral_point() {
    printf("== neutral point: where an AoA change puts the change in lift (#454) ==\n");
    const double cl = 6.0, stall = 0.35;
    const glm::dmat3 ship(1.0);       // columns: right +X, up +Y, nose +Z
    const glm::dvec3 up(0.0, 1.0, 0.0), right(1.0, 0.0, 0.0), nose(0.0, 0.0, 1.0);
    const glm::dvec3 vrel(0.0, 0.0, 100.0);   // dead along the nose, AoA 0
    /* applyAeroForce hands liftSlopePerQ the PartAero its force already used
       (vehicle.cpp), so the test samples the same thing the same way. */
    const auto slope = [&](const glm::dmat3 &rot, const glm::dvec3 &flow, double S,
                           const glm::dvec3 &lat) {
        return liftSlopePerQ(partAero(rot, flow), rot, flow, S, cl, stall, lat);
    };

    // One wing AT the pivot: all the extra lift lands there, so no lever.
    {
        LiftSlopeSum s;
        s.add(slope(ship, vrel, 10.0, right), glm::dvec3(0.0), up, right);
        CHECK_NEAR(s.slope(), 60.0, 1e-6,
                   "a 10 m2 wing at 6/rad buys 60 m2 of lift per radian");
        CHECK_TRUE(s.usable(), "a surface in its linear range has a margin to quote");
        CHECK_NEAR(s.margin(), 0.0, 1e-9, "bolted at the pivot, its margin is zero");
    }

    // The mirrored twin -- the shape that broke the first attempt. Both halves lift
    // ship-up, so their slope ADDS; and offsets along SPAN make no pitch moment,
    // because (right x up) is the nose axis, which is not the axis being measured.
    {
        const glm::dmat3 twin(-ship[0], -ship[1], ship[2]);
        LiftSlopeSum s;
        s.add(slope(ship, vrel, 10.0, right), 5.0 * right, up, right);
        s.add(slope(twin, vrel, 10.0, right), -5.0 * right, up, right);
        CHECK_NEAR(s.slope(), 120.0, 1e-6,
                   "a 180-degree twin pair ADDS its slope rather than cancelling it");
        CHECK_NEAR(s.margin(), 0.0, 1e-9, "and a pair offset along its span has no lever");
    }

    // And the SUM refuses to quote a lever when the surfaces cancel. Built here out of
    // two contributions rather than out of mounts, because this is the arithmetic of
    // the sum, not of any law: two surfaces with 60 m2/rad each, pushing opposite ways
    // along ship-up. Where dl is a small difference of large numbers, where their
    // average lands is arithmetic noise -- and the first version of this code printed
    // exactly that as "Np=+15.87 m" for the shipped skiff.
    {
        LiftSlopeSum s;
        s.add(up * 60.0, 5.0 * right, up, right);
        s.add(-up * 60.0, -5.0 * right, up, right);
        CHECK_NEAR(s.slope(), 0.0, 1e-12, "surfaces that disagree cancel");
        CHECK_TRUE(!s.usable(),
                   "and the readout dashes rather than quoting the average of a cancelled sum");
    }

    // A tail behind the pivot is what static margin IS. Wings 60 m2/rad at the pivot,
    // a 12 m2/rad tailplane 5 m aft: dm = 12*5 = 60 m3/rad over dl = 72.
    {
        LiftSlopeSum s;
        s.add(slope(ship, vrel, 10.0, right), glm::dvec3(0.0), up, right);
        s.add(slope(ship, vrel, 2.0, right), -5.0 * nose, up, right);
        CHECK_NEAR(s.slope(), 72.0, 1e-6, "wings plus tailplane, in area per radian");
        CHECK_NEAR(s.margin(), 60.0 / 72.0, 1e-6,
                   "a tailplane 5 m aft puts the lift change BEHIND the COM");
        CHECK_NEAR(s.point(), -s.margin(), 1e-9,
                   "point() is the same lever read ahead-positive");
    }

    // The same surfaces with the small one in FRONT -- a canard -- and the airframe
    // diverges on its own. This is the sign the whole thing turns on.
    {
        LiftSlopeSum s;
        s.add(slope(ship, vrel, 10.0, right), glm::dvec3(0.0), up, right);
        s.add(slope(ship, vrel, 2.0, right), 5.0 * nose, up, right);
        CHECK_NEAR(s.margin(), -60.0 / 72.0, 1e-6, "a tail AHEAD of the wings is a negative margin");
    }

    // A surface ROLLED about the nose -- every shipped tailplane carries +-5 deg of
    // this, which is the only mount tilt in the shipped catalog, and the one the skiff's
    // numbers turn on. Two cosines land in the pitch slope and they are two different
    // facts: the roll tilts the surface's own lift axis off ship-up (one cos), and only
    // the part of the flow tilt that is about the surface's OWN span axis moves its
    // angle of attack (the other). The lever is untouched -- a rolled wing still pushes
    // where it is bolted. Shipped check: the skiff's 2 x 2 m2 tailplanes at 5 deg report
    // 23.82 of their 24.0 m2/rad, and that is the entire 168.0 -> 167.82 residue.
    {
        const glm::dmat3 rolled = glm::mat3_cast(glm::angleAxis(glm::radians(30.0), nose));
        LiftSlopeSum s;
        s.add(slope(rolled, vrel, 10.0, right), -5.0 * nose, up, right);
        CHECK_NEAR(s.slope(), 45.0, 1e-6, "a wing rolled 30 deg buys cos^2 of its slope");
        CHECK_NEAR(s.margin(), 5.0, 1e-6, "and still the same 5 m of lever");
    }

    // A fin answers to yaw, not pitch: its lift runs along ship's right, so a ship
    // whose only surface is a fin has no pitch slope at all, and tick.cpp dashes the
    // margin rather than claiming zero -- zero would say "lift change exactly at the
    // COM", which is a different and wrong statement.
    {
        const glm::dmat3 fin(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
        LiftSlopeSum s;
        s.add(slope(fin, vrel, 4.0, right), -3.0 * nose, up, right);
        CHECK_NEAR(s.slope(), 0.0, 1e-12, "a fin contributes no PITCH slope");
        CHECK_TRUE(!s.usable(), "and its margin is refused, not quoted as 0");
    }

    // Stiffness, not only the lever. The lift curve peaks AT the stall angle, so past
    // it more angle of attack means LESS lift and the slope reverses; by 2x the stall
    // angle the law is identically zero and there is nothing to quote. A symmetric
    // layout keeps quoting a blandly positive margin through all of it -- measured on
    // the jet at AoA 28 and -35 (both past its 20.05 deg peak): SM stayed +0.55 m while
    // the slope fell from 96 towards 0. That is why the log prints the slope too.
    {
        const auto flowAt = [](double a) {
            return glm::dvec3(0.0, -100.0 * std::sin(a), 100.0 * std::cos(a));
        };
        // The slope is a projection on SHIP up, so it carries the cos(alpha) dip --
        // 60.0 at 0 deg, 59.095 at 10. The tolerance used to be 1.0 to swallow this,
        // which read as slop and would have broken at 11 deg for the wrong reason.
        LiftSlopeSum linear;
        linear.add(slope(ship, flowAt(glm::radians(10.0)), 10.0, right),
                   glm::dvec3(0.0), up, right);
        CHECK_NEAR(linear.slope(), 60.0 * std::cos(glm::radians(10.0)), 1e-4,
                   "below the 20.05 deg stall angle the slope is still the linear one, projected");
        LiftSlopeSum past;
        past.add(slope(ship, flowAt(glm::radians(30.0)), 10.0, right),
                 glm::dvec3(0.0), up, right);
        CHECK_TRUE(past.slope() < 0.0,
                   "past the stall peak, more angle of attack means LESS lift");
        CHECK_NEAR(past.margin(), 0.0, 1e-9,
                   "while the margin of a surface at the pivot stays put -- the lever says nothing about this");
        LiftSlopeSum flat;
        flat.add(slope(ship, flowAt(glm::radians(80.0)), 10.0, right),
                 glm::dvec3(0.0), up, right);
        CHECK_NEAR(flat.slope(), 0.0, 1e-9, "past 2x the stall angle the law is identically zero");
        CHECK_TRUE(!flat.usable(), "and a stalled-flat airframe quotes no margin");
    }
}

/* #483: a surface feels the flow AT ITSELF, v_rel + w x r, not the COM's. drag.h
   knows no bodies and no angular velocity, so what is pinned here is the
   COMPOSITION applyAeroForce runs (one cross product, then partAero, then
   liftForce with that surface's own q) and the question a flight golden cannot
   cheaply answer: does the sweep make a moment that OPPOSES the rotation? Get the
   sign wrong and the airframe still flies -- it just feeds every swing back into
   itself -- so "the numbers moved" is not evidence. The wiring is pinned by
   flight/458-glider, whose AoA series used to be 14.8, -108.6, 75.2, -36.5 ...
   (tumbling to the end of the run) and now decays to under a degree by t=10. */
/* #426: a surface's EFFECTIVENESS ROW -- the torque one radian of its COMMAND buys,
   stated as a derivative of the same force law the aero step bills (drag.h
   surfaceTorquePerRadian). Pure geometry: q, S, cl, the offset from the COM, the
   mount's own push normal (#495's build fact, not the live flow) and the authored
   steering axis, which decides only the SENSE -- see the mix pins at the end. */
static void test_surface_effectiveness() {
    printf("== the surface as an actuator row (#426) ==\n");
    /* A tail surface 3 m behind the COM, plate normal +Y, rigged PITCH (aboutAxis 0,
       targetSign -1 -- what controlAxisParams says for an elevator): the push is +Y at
       -Z, so the raw moment is about +X, and a POSITIVE command is the negative of it.
       q=100, S=2, cl=6 -> 1200 N m of force per rad, lever 3 m -> 3600 N m/rad. */
    const glm::dvec3 tail = glm::dvec3(0.0, 0.0, -3.0);
    const glm::dvec3 up(0.0, 1.0, 0.0), right(1.0, 0.0, 0.0), nose(0.0, 0.0, 1.0);
    const glm::dvec3 row = surfaceTorquePerRadian(100.0, 2.0, 6.0, tail, up, right);
    CHECK_NEAR(row.x, -3600.0, 1e-9, "the moment is the push at its offset, per rad");
    CHECK_NEAR(row.y, 0.0, 1e-9, "and nothing else");
    CHECK_NEAR(row.z, 0.0, 1e-9, "and nothing else");
    CHECK_NEAR(row.x, -surfaceTorquePerRadian(100.0, 2.0, 6.0, tail, up, right, +1.0).x,
               1e-9, "targetSign is the authored axis's, exactly as controlDeflection's");

    /* The mirrored twin -- a canard ahead of the COM with its plate flipped -- levers
       the SAME torque for the SAME command. That is #495's mirrored-pair identity
       arriving in the row, and why the setpoint can go straight back into
       controlDeflection without a per-instance sign surviving anywhere else. */
    const glm::dvec3 twin = surfaceTorquePerRadian(100.0, 2.0, 6.0,
                                                   glm::dvec3(0.0, 0.0, 3.0),
                                                   glm::dvec3(0.0, -1.0, 0.0), right);
    CHECK_NEAR(twin.x, row.x, 1e-9, "a mirrored pair levers the same way");

    /* No air, no area, no slope: no row. The row is a q-times-geometry budget, so a
       hull in vacuum or at rest has nothing to spend, which is the right answer
       rather than a missing one. */
    CHECK_TRUE(surfaceTorquePerRadian(0.0, 2.0, 6.0, tail, up, right)
               == glm::dvec3(0.0), "no q: no authority");
    CHECK_TRUE(surfaceTorquePerRadian(100.0, 0.0, 6.0, tail, up, right)
               == glm::dvec3(0.0), "no area: no authority");
    CHECK_TRUE(surfaceTorquePerRadian(100.0, 2.0, 0.0, tail, up, right)
               == glm::dvec3(0.0), "no slope: no authority");

    /* THE DEGENERATE RIG (#495's test, on the row). A plate whose push line runs
       through its own pivot levers nothing, and actuators.h divides a line's share by
       the row's length, so crediting a hair-length row would ask that fin for a hair
       short of an infinite deflection. Same form and same RELATIVE tolerance as
       controlDeflection -- it has to be the same test, or the budget credits a surface
       the aero step then refuses to move. */
    const double big = 1e9;
    CHECK_TRUE(surfaceTorquePerRadian(big, 2.0, 6.0, tail, nose, right)
               == glm::dvec3(0.0), "push line through the pivot: no row at all");
    CHECK_TRUE(surfaceTorquePerRadian(big, 2.0, 6.0, glm::dvec3(0.0, 0.0, 0.0),
                                      up, right)
               == glm::dvec3(0.0), "a steering part on the COM: no row");
    /* A real lever, aimed almost along itself: the tolerance is on the GEOMETRY, not
       on what counts as weak (the same pair of pins controlDeflection carries). */
    const double t14 = 1e-4, t18 = 1e-8;
    const glm::dvec3 near_line14(std::sin(t14), 0.0, -std::cos(t14));
    const glm::dvec3 near_line18(std::sin(t18), 0.0, -std::cos(t18));
    /* A push nearly ALONG the stack levers about UP, so `up` is the axis these rigs
       are asked about: the gate is on the projected lever, and asking it about `right`
       would be a genuinely dead rig rather than an almost-alive one (which is the
       other pin, below). */
    const glm::dvec3 r14 = surfaceTorquePerRadian(1.0, 1.0, 1.0, tail, near_line14, up);
    const glm::dvec3 r18 = surfaceTorquePerRadian(1.0, 1.0, 1.0, tail, near_line18, up);
    /* 3 m * sin(t), not 3 m * t: the tolerance is relative, and the difference
       between sin(1e-4) and 1e-4 is 1.7e-13 -- far inside the geometry's meaning,
       far outside a 1e-12 relative pin. */
    CHECK_NEAR(glm::length(r14), 3.0 * std::sin(t14), 1e-12,
               "a 3 m arm 1e-4 rad off the degenerate line still levers 3e-4 m");
    CHECK_TRUE(r18 == glm::dvec3(0.0), "and 1e-8 rad is degenerate, as it must be");

    /* THE MIX, and where the label stops mattering (#426). A plate pushed 45 deg
       between the ship's up and right levers BOTH pitch and yaw: the raw cross product
       is 3 m long either way, at 3*cos45 about each axis. The row keeps that whole
       vector -- the solver is what decides which way to spend it -- while the SENSE
       comes from the authored axis alone. Hand arithmetic: cross((0,0,-3),
       (cos45,sin45,0)) = (3cos45, -3cos45, 0), so at q=100,S=2,cl=6 each component is
       1200 * 3 * sqrt(0.5) = 2545.58 N m/rad. */
    const double c45 = std::sqrt(0.5);
    const glm::dvec3 push45(c45, c45, 0.0);
    const double comp = 1200.0 * 3.0 * c45;
    const glm::dvec3 asPitch = surfaceTorquePerRadian(100.0, 2.0, 6.0, tail, push45, right);
    const glm::dvec3 asYaw = surfaceTorquePerRadian(100.0, 2.0, 6.0, tail, push45, up);
    CHECK_NEAR(glm::length(asPitch), 3600.0, 1e-9,
               "the reach is the geometry's, not the label's");
    CHECK_NEAR(asPitch.x, -comp, 1e-6, "labeled pitch: its own axis is what it buys");
    CHECK_NEAR(asPitch.y, +comp, 1e-6, "and it buys the other axis too");
    CHECK_NEAR(asYaw.x, -asPitch.x, 1e-6, "labeled yaw: same plate, opposite sense");
    CHECK_NEAR(asYaw.y, -asPitch.y, 1e-6, "because only the SENSE is the label's");

    /* ...and the label is still a WALL where #495 says it is one. This plate levers
       nothing about the axis it is rigged for, so its command sign is undecidable and
       the aero step would refuse to move it: it buys NO row, mix or no mix. */
    CHECK_TRUE(surfaceTorquePerRadian(big, 2.0, 6.0, tail, up, up)
               == glm::dvec3(0.0),
               "a plate that levers only the OTHER axis is dead, rig and budget alike");
}

static void test_local_flow_sweep() {
    printf("== local flow: w x r is damping, and its sign is the whole game ==\n");
    const double S = 4.0, cl = 6.0, stall = 0.35;
    const double rho = 1.225, v = 100.0;
    const glm::dmat3 ship(1.0);                 // right +X, up +Y, nose +Z
    const glm::dvec3 nose = ship[2];
    const glm::dvec3 vrel(0.0, 0.0, v);         // dead along the nose: 0 AoA, 0 beta

    struct Surf { glm::dvec3 vp; double q; PartAero pa; glm::dvec3 F; };
    /* localFlow + partAero + liftForce: the same composition updateAerodynamics
       runs, so this test pins production rather than my transcription of it. */
    const auto surfAt = [&](const glm::dvec3 &r, const glm::dvec3 &w,
                            const glm::dmat3 &rot) {
        Surf s;
        const LocalFlow lf = localFlow(vrel, w, r, rho);
        s.vp = lf.vp;
        s.q = lf.q;
        s.pa = partAero(rot, s.vp);
        s.F = s.pa.valid ? liftForce(s.q, S, cl, s.pa.alpha, s.pa.liftDir, stall)
                         : glm::dvec3(0.0);
        return s;
    };
    const auto surf = [&](const glm::dvec3 &r, const glm::dvec3 &w) {
        return surfAt(r, w, ship);   // the untilted airframe
    };

    // A body that is not spinning is bit-identical to the pre-#483 law, not
    // approximately equal to it.
    {
        const Surf s = surf(glm::dvec3(0.0, 0.0, 3.0), glm::dvec3(0.0));
        CHECK_TRUE(s.vp == vrel, "no spin: the sweep is exactly zero, not nearly");
        // Compared against the SAME expression on the COM flow, so this says "the
        // sweep contributed nothing", not "these two spellings of q agree".
        CHECK_NEAR(s.q, 0.5 * rho * glm::dot(vrel, vrel), 0.0,
                   "and so is the change in dynamic pressure");
        CHECK_NEAR(glm::length(s.F), 0.0, 0.0, "so a 0-deg wing still lifts nothing");
    }

    // localFlow's own bookkeeping: vp2 is the magnitude q was built from, so the
    // induced term's direction costs one sqrt rather than two magnitudes. Pinned
    // because src/vehicle.cpp relies on sqrt(vp2) being that same magnitude.
    {
        const LocalFlow lf = localFlow(vrel, glm::dvec3(0.0, 0.0, 2.0),
                                       glm::dvec3(0.0, 3.0, 0.0), rho);
        CHECK_NEAR(std::sqrt(lf.vp2), glm::length(lf.vp), 1e-12,
                   "sqrt(vp2) is the flow's magnitude, as glm::length computes it");
        CHECK_NEAR(lf.q, 0.5 * rho * lf.vp2, 0.0, "and q is half rho times vp2 exactly");
    }

    // The bug, stated: on the COM's flow every surface of this airframe sees 0 deg,
    // however fast it is rotating. No sweep, no damping.
    {
        const PartAero same = partAero(ship, vrel);
        CHECK_NEAR(same.alpha, 0.0, 0.0, "COM flow: every surface sees the same 0 deg");
        CHECK_NEAR(glm::length(liftForce(0.5 * rho * v * v, S, cl, same.alpha,
                                         same.liftDir, stall)),
                   0.0, 0.0, "COM flow: a pitching airframe gets nothing out of it");
    }

    // Pitch rate, carried by two LIFTING surfaces displaced fore-aft (a tail and a
    // canard). Note what does NOT damp here: a wing on the pitch axis (the shipped
    // wings, at the COM's height) has no fore-aft offset, so its sweep runs along
    // the nose and leaves alpha alone. Pitch damping needs surfaces ahead of or
    // behind the pivot, which is why a control surface with no alpha term (#454)
    // leaves this airframe with so little of it.
    {
        const glm::dvec3 w(0.5, 0.0, 0.0);                    // about the ship's right
        const glm::dvec3 rCanard(0.0, 0.0, 3.0), rTail(0.0, 0.0, -2.5);
        const Surf c = surf(rCanard, w), t = surf(rTail, w);
        CHECK_TRUE(c.pa.valid && t.pa.valid, "the swept frames are usable");
        CHECK_NEAR(c.pa.alpha, std::atan2(3.0 * 0.5, v), 1e-12,
                   "the surface the pitch drives DOWN meets the flow from below");
        CHECK_NEAR(t.pa.alpha, -std::atan2(2.5 * 0.5, v), 1e-12,
                   "and the one being lifted meets it from above");
        CHECK_TRUE(c.pa.alpha * t.pa.alpha < 0.0, "the pair tilts OPPOSITE ways");
        const glm::dvec3 F = c.F + t.F;
        const glm::dvec3 M = glm::cross(rCanard, c.F) + glm::cross(rTail, t.F);
        CHECK_TRUE(glm::dot(M, w) < 0.0, "the pair's moment OPPOSES the pitch rate");
        CHECK_TRUE(glm::length(F) < glm::length(c.F) + glm::length(t.F),
                   "the forces partly cancel: a couple, not a climb");
        // Twice the rate: damping at least doubles, because q rises with the sweep
        // too. Pinning the inequality beats pinning a factor -- the factor is where
        // the law is soft, the sign and growth are where it is wrong or right.
        const Surf c2 = surf(rCanard, 2.0 * w), t2 = surf(rTail, 2.0 * w);
        const glm::dvec3 M2 = glm::cross(rCanard, c2.F) + glm::cross(rTail, t2.F);
        CHECK_TRUE(std::fabs(glm::dot(M2, 2.0 * w)) > 2.0 * std::fabs(glm::dot(M, w)),
                   "damping grows at least as fast as the rate it opposes");
        // It is damping, not a bias: spin the other way and the moment follows.
        const Surf cn = surf(rCanard, -w), tn = surf(rTail, -w);
        const glm::dvec3 Mn = glm::cross(rCanard, cn.F) + glm::cross(rTail, tn.F);
        CHECK_TRUE(glm::dot(Mn, -w) < 0.0, "reversed spin is opposed too");
        CHECK_TRUE(glm::dot(M, Mn) < 0.0, "and the two moments point opposite ways");
    }

    /* INSIDE THE DEEP-STALL DROOP THE SAME TERM FLIPS SIGN -- and a shipped airframe
       sits there. Same pair, same rate, mounts tilted 30 deg up: past the 20.1 deg
       stall angle liftCurve's slope is NEGATIVE until twice it (40.1 deg), so the
       surface the rotation pulls back TOWARD the stall peak gains lift instead of
       losing it. Subtract the omega = 0 moment -- the incidence itself makes a big
       static couple that says nothing about damping -- and the moment the sweep ADDS
       lines up with w: the airframe feeds its swing back in. Measured in situ on both
       shipped airframes, which both pump in this band (glider dM.w up to +413 kW, jet
       up to +3080 W, with the pumping samples at |AoA| 27-41 deg). Pinned as the flip,
       not the size: whether a shipped part should droop at all is #487, not settled
       here. Past 2x stallAngle the curve is zero and the term does nothing at all --
       the third regime, which the self-limiting block below pins. */
    {
        const double a0 = 0.5236;               // 30 deg: past 20.1, inside 40.1
        const glm::dmat3 inc = glm::mat3_cast(glm::angleAxis(a0, ship[0]));
        const glm::dvec3 w(0.5, 0.0, 0.0);
        const glm::dvec3 rCanard(0.0, 0.0, 3.0), rTail(0.0, 0.0, -2.5);
        const Surf c0 = surfAt(rCanard, glm::dvec3(0.0), inc);
        const Surf t0 = surfAt(rTail, glm::dvec3(0.0), inc);
        CHECK_NEAR(std::fabs(c0.pa.alpha), a0, 1e-12,
                   "the mount sits past the stall angle, inside the droop");
        CHECK_TRUE(glm::length(c0.F) > 0.0 && glm::length(t0.F) > 0.0,
                   "a tilted wing at rest still lifts -- only the SWEEP is damping");
        const Surf c = surfAt(rCanard, w, inc), t = surfAt(rTail, w, inc);
        const glm::dvec3 dM = glm::cross(rCanard, c.F - c0.F)
                            + glm::cross(rTail, t.F - t0.F);
        CHECK_TRUE(glm::dot(dM, w) > 0.0,
                   "in the droop the sweep DRIVES the rate it would damp below stall");
        const Surf cn = surfAt(rCanard, -w, inc), tn = surfAt(rTail, -w, inc);
        const glm::dvec3 dn = glm::cross(rCanard, cn.F - c0.F)
                            + glm::cross(rTail, tn.F - t0.F);
        CHECK_TRUE(glm::dot(dn, -w) > 0.0, "and it pumps either way, not just one");
    }

    // Roll rate, one wing either side: the classic roll damping no control input
    // is asked for. The vertical forces cancel exactly, so a symmetric airframe
    // does not start climbing because it rolled.
    {
        const glm::dvec3 w(0.0, 0.0, 0.6);                    // about the ship's nose
        const glm::dvec3 rR(4.0, 0.0, 0.0), rL(-4.0, 0.0, 0.0);
        const Surf a = surf(rR, w), b = surf(rL, w);
        CHECK_NEAR(a.pa.alpha, -std::atan2(4.0 * 0.6, v), 1e-12,
                   "the rising tip's local flow comes up at it");
        CHECK_NEAR(b.pa.alpha, -a.pa.alpha, 0.0, "its twin sees the mirror, exactly");
        const glm::dvec3 F = a.F + b.F;
        CHECK_NEAR(F.y, 0.0, 0.0, "the two lifts cancel vertically");
        CHECK_NEAR(F.x, 0.0, 0.0, "and sideways");
        const glm::dvec3 M = glm::cross(rR, a.F) + glm::cross(rL, b.F);
        CHECK_TRUE(glm::dot(M, w) < 0.0, "while both together oppose the roll");
        // What is left over is the tilted pair pushing along the flow -- small, and
        // the honest consequence of a rolling wing pair, not a bug to chase.
        CHECK_TRUE(std::fabs(F.z) < 0.1 * (glm::length(a.F) + glm::length(b.F)),
                   "the leftover along the flow is small next to the lift");
    }

    // A surface the spin carries SIDEWAYS keeps its angle but gains flow speed, so
    // its force grows. That is why the dynamic pressure is per-surface now rather
    // than one ship-wide q.
    {
        const glm::dvec3 w(0.0, 0.0, 2.0);
        const glm::dvec3 r(0.0, 3.0, 0.0);
        const Surf s = surf(r, w);
        CHECK_NEAR(s.pa.alpha, 0.0, 0.0, "a sideways sweep leaves the ANGLE alone");
        CHECK_TRUE(s.q > 0.5 * rho * v * v, "but the surface still feels a faster flow");
    }

    // Self-limiting, and worth pinning as a consequence rather than discovering it
    // as a surprise: spin a surface backwards through the air faster than the body
    // flies and its own flow reverses, the deep-stall droop takes its force away,
    // and the damping term cannot run away with a fast tumble.
    {
        const glm::dvec3 w(0.0, 30.0, 0.0);                  // yaw, hard
        const glm::dvec3 r(4.0, 0.0, 0.0);                   // a wingtip 4 m out
        const Surf s = surf(r, w);
        CHECK_TRUE(glm::length(glm::cross(w, r)) > v, "the tip outruns the air it flies in");
        CHECK_TRUE(glm::dot(s.vp, nose) < 0.0, "so its flow comes from behind");
        CHECK_NEAR(glm::length(s.F), 0.0, 0.0, "and the droop leaves it no lift to damp with");
    }
}

static void test_liftCurve() {
    printf("== liftCurve: linear up to stall, soft droop, deep stall ==\n");
    const double cl = 6.0;        // per radian
    const double A  = 0.30;       // stall angle (rad)
    const double clmax = cl * A;  // peak CL, reached at |alpha| = A

    // Below stall: exactly linear (the Phase 2 law), sign follows alpha.
    CHECK_NEAR(liftCurve(cl, 0.05, A), cl * 0.05, 1e-12, "below stall: linear");
    CHECK_NEAR(liftCurve(cl, -0.05, A), -cl * 0.05, 1e-12, "below stall: linear (-)");

    // Zero AoA: no lift (a symmetric section).
    CHECK_NEAR(liftCurve(cl, 0.0, A), 0.0, 0.0, "zero AoA -> 0");

    // At the stall angle: the peak (cl*A), continuous from both sides.
    CHECK_NEAR(liftCurve(cl, A, A), clmax, 1e-12, "at stall angle: peak");
    CHECK_NEAR(liftCurve(cl, -A, A), -clmax, 1e-12, "at stall angle: peak (-)");

    // Just past stall: the droop has begun (below the peak, still positive).
    CHECK_NEAR(liftCurve(cl, A * 1.1, A),
               cl * A * std::cos(0.1 * M_PI * 0.5), 1e-12, "past stall: droop");

    // Monotone decrease through the droop: peak > 1.1A > 1.5A > 2A (=0).
    CHECK_TRUE(liftCurve(cl, A, A) > liftCurve(cl, A * 1.1, A), "droop: A > 1.1A");
    CHECK_TRUE(liftCurve(cl, A * 1.1, A) > liftCurve(cl, A * 1.5, A),
               "droop: 1.1A > 1.5A");
    CHECK_TRUE(liftCurve(cl, A * 1.5, A) > 0.0, "droop: 1.5A > 0");

    // At 2x the stall angle the droop is complete (deep stall); beyond it, and
    // at a fully sideways 90 deg, the wing is edge-on and generates no lift.
    CHECK_NEAR(liftCurve(cl, A * 2.0, A), 0.0, 0.0, "at 2A: deep stall -> 0");
    CHECK_NEAR(liftCurve(cl, A * 3.0, A), 0.0, 0.0, "beyond 2A: still 0");
    CHECK_NEAR(liftCurve(cl, M_PI * 0.5, A), 0.0, 0.0, "90 deg -> 0");

    // Symmetric in |alpha|: equal magnitude up or down, sign follows alpha.
    CHECK_NEAR(std::fabs(liftCurve(cl, 0.2, A)), std::fabs(liftCurve(cl, -0.2, A)),
               1e-12, "symmetric magnitude");
    CHECK_TRUE(liftCurve(cl, 0.2, A) > 0.0 && liftCurve(cl, -0.2, A) < 0.0,
               "sign follows alpha");

    // No stall (A = 0): pure linear for ALL alpha (the Phase 2 law, no droop).
    CHECK_NEAR(liftCurve(cl, 1.0, 0.0), cl * 1.0, 1e-12, "no stall: linear (+)");
    CHECK_NEAR(liftCurve(cl, -1.0, 0.0), -cl * 1.0, 1e-12, "no stall: linear (-)");

    // Degenerate: no slope -> no lift.
    CHECK_NEAR(liftCurve(0.0, 0.1, A), 0.0, 0.0, "cl = 0 -> 0");
    CHECK_NEAR(liftCurve(-1.0, 0.1, A), 0.0, 0.0, "cl < 0 -> 0");
}

/* #543 A5. Every number below is what drag.h::liftSlopeMach answers for a cl 6
   section -- cl 6.0 is the shipped value on all five lifting parts in res/, so this
   is the airframe's own table, not a synthetic one. The subsonic and supersonic
   anchors are the closed forms (Prandtl-Glauert and thin-airfoil 4/sqrt(M^2-1));
   the ones inside the band are decimals, because there the law IS its own answer
   -- a straight join between two theories that do not meet. */
static void test_lift_slope_mach() {
    printf("== liftSlopeMach: the slope belongs to the air, not the part (#543 A5) ==\n");
    const double cl = 6.0;   // per radian, as every shipped lifting part authors it

    // INCOMPRESSIBLE IDENTITY. Mach 0 answers the authored slope exactly -- and
    // Prandtl-Glauert is not switched on at a threshold, it is 1 there by arithmetic.
    CHECK_NEAR(liftSlopeMach(cl, 0.0), cl, 0.0, "M 0: the authored slope");
    CHECK_NEAR(liftSlopeMach(cl, -1.0), cl, 0.0, "M < 0 (no flow): authored slope");
    CHECK_NEAR(liftSlopeMach(cl, 0.01), 6.000300022502, 1e-9, "M 0.01: 1.00005 x cl");

    // SUBSONIC, Prandtl-Glauert: cl / sqrt(1 - M^2). The cruise end of the game's
    // envelope: the skiff's 85 m/s is M 0.31 on Kerbin's 274.8 m/s, worth +5.2%.
    CHECK_NEAR(liftSlopeMach(cl, 0.3), 6.289709020332, 1e-9, "M 0.3: 1.0483 x cl");
    CHECK_NEAR(liftSlopeMach(cl, 0.5), cl / std::sqrt(0.75), 1e-12, "M 0.5");
    CHECK_NEAR(liftSlopeMach(cl, 0.7), 8.401680504168, 1e-9, "M 0.7: 1.4003 x cl");
    // The whole point of the correction: it is monotone UP all the way to the band.
    CHECK_TRUE(liftSlopeMach(cl, 0.3) > cl && liftSlopeMach(cl, 0.7) > liftSlopeMach(cl, 0.3),
               "subsonic: rising with Mach");

    // THE BAND. PG is extrapolation past ~0.75 and the sqrt would run to infinity at
    // M 1, so the band joins PG(0.75) to the thin-airfoil value at 1.25 in a straight
    // line of ABSOLUTE slope. Continuous at both edges -- pinned from either side:
    CHECK_NEAR(liftSlopeMach(cl, 0.749999), 9.071131801733, 1e-6, "just under the band");
    CHECK_NEAR(liftSlopeMach(cl, kTransonicMach), 9.071147352221, 1e-9, "band low edge");
    CHECK_NEAR(liftSlopeMach(cl, 0.750001), 9.071139876593, 1e-6, "just over the edge");
    CHECK_NEAR(liftSlopeMach(cl, 1.249999), 5.333340808961, 1e-6, "just under the top edge");
    CHECK_NEAR(liftSlopeMach(cl, kSupersonicMach), 4.0 / std::sqrt(0.5625), 1e-12,
               "band high edge = thin airfoil at 1.25");
    CHECK_NEAR(liftSlopeMach(cl, 1.250001), 5.333321481516, 1e-6, "just over the top edge");
    // What the band BUYS: raw PG at M 0.99 answers 42.5 -- seven times the authored
    // slope, on a section that in reality has a shock on it and is losing lift. The
    // same query through the band answers 7.28, and M 1 is finite for any cl.
    CHECK_NEAR(liftSlopeMach(cl, 0.8), 8.697365950333, 1e-9, "M 0.8: falling");
    CHECK_NEAR(liftSlopeMach(cl, 0.9), 7.949803146555, 1e-9, "M 0.9");
    CHECK_NEAR(liftSlopeMach(cl, 0.99), 7.276996623155, 1e-9, "M 0.99");
    CHECK_NEAR(liftSlopeMach(cl, 1.0), 7.202240342777, 1e-9, "M 1: finite, falling");
    CHECK_NEAR(liftSlopeMach(cl, 1.1), 6.454677539000, 1e-9, "M 1.1");
    CHECK_TRUE(std::isfinite(liftSlopeMach(cl, 1.0)), "M 1 is finite");
    // For a section this steep (any cl above 3.53 -- every shipped part authors 6.0) the
    // curve's peak is the band's low edge, 1.512 cl, and it is falling through M 1.
    // Below that crossover the join keeps rising to its high edge instead; the game has
    // no such part, so the peak stays where the transonic effect actually peaks.
    CHECK_TRUE(liftSlopeMach(cl, 0.74) < liftSlopeMach(cl, kTransonicMach)
               && liftSlopeMach(cl, kSupersonicMach) < liftSlopeMach(cl, kTransonicMach),
               "the curve peaks at the band's low edge");

    // SUPERSONIC, thin airfoil: 4 / sqrt(M^2 - 1) per radian, for ANY cl. Read that
    // again: above the band the authored slope stops mattering -- that is what the
    // theory says (drag.h spells out the consequence and where the real fix is).
    CHECK_NEAR(liftSlopeMach(cl, 1.5), 3.577708764000, 1e-9, "M 1.5");
    CHECK_NEAR(liftSlopeMach(cl, 1.62), 3.138437619572, 1e-9, "M 1.62 (flight/631's entry)");
    CHECK_NEAR(liftSlopeMach(cl, 2.0), 4.0 / std::sqrt(3.0), 1e-12, "M 2");
    CHECK_NEAR(liftSlopeMach(cl, 3.0), std::sqrt(2.0), 1e-12, "M 3: sqrt 2");
    CHECK_NEAR(liftSlopeMach(1.0, 3.0), liftSlopeMach(cl, 3.0), 0.0,
               "supersonic: cl drops out");
    CHECK_TRUE(liftSlopeMach(cl, 3.0) < liftSlopeMach(cl, 1.3), "supersonic: falling");

    // ONE SWEEP, 0 to M 4: no jump and no NaN anywhere. The largest step over a 0.001
    // Mach stride is 0.0155 (the curve has a KINK at the band edge -- PG arrives
    // rising, the join leaves falling -- which costs no force, only a derivative).
    double prev = liftSlopeMach(cl, 0.0);
    bool sweepOk = true;
    for(int i = 1; i <= 4000; ++i) {
        const double m = i * 0.001;
        const double v = liftSlopeMach(cl, m);
        if(!std::isfinite(v) || v <= 0.0 || std::fabs(v - prev) > 0.02) { sweepOk = false; }
        prev = v;
    }
    CHECK_TRUE(sweepOk, "sweep M 0..4: finite, positive, no step over 0.02");
    // Absurd Mach: 4/sqrt(M^2-1) -> 0, and it must not overflow on the way there.
    CHECK_TRUE(liftSlopeMach(cl, 1e300) == 0.0, "M 1e300: 0, not NaN");
    CHECK_NEAR(liftSlopeMach(cl, std::numeric_limits<double>::quiet_NaN()), cl, 0.0,
               "NaN Mach quotes the authored slope rather than poisoning the force");
    // No slope to correct.
    CHECK_NEAR(liftSlopeMach(0.0, 2.0), 0.0, 0.0, "cl 0 -> 0");
    // A part file that authors a non-finite slope stays non-finite through the band's
    // interpolation rather than becoming NaN through inf - inf. It is not VALID -- the
    // aero pass asserts a finite slope (vehicle.cpp) and a dossier that reads inf prints
    // inf -- but the law's own arithmetic does not invent a second failure mode.
    CHECK_TRUE(std::isinf(liftSlopeMach(std::numeric_limits<double>::infinity(), 1.0)),
               "inf cl stays inf");
    CHECK_TRUE(std::isnan(liftSlopeMach(std::numeric_limits<double>::quiet_NaN(), 1.0)),
               "NaN cl quotes NaN rather than the band");

    /* INHERITANCE, which is what the ticket is actually about: the correction is a
       number the call site takes once, so the curve, the control law, the neutral-point
       derivative and #487's authority all move by the SAME factor. Read these as
       HOMOGENEITY pins, not as proof of the wiring: every law here is handed an already
       corrected slope, so a call site that forgets to correct is invisible at this level
       -- that failure only shows where a real hull flies, and e2e/cases/
       flight/631-earth-air-in-flight.txt is where it is pinned (its CLa column misses by
       91% on the pre-A5 binary). What reddens here is a law that grows its OWN Mach rule
       or quietly cancels the correction, which is the split #543's "ONE source" line is
       about. */
    const double A = 0.35;   // the shipped wing stall_angle
    const double f05 = 1.0 / std::sqrt(1.0 - 0.25);   // PG at M 0.5
    CHECK_NEAR(liftCurve(liftSlopeMach(cl, 0.5), 0.1, A),
               liftCurve(cl, 0.1, A) * f05, 1e-12, "liftCurve: scales by the factor");
    // The peak CL and the whole droop scale with it -- documented simplification:
    // the section stalls at the same ANGLE at every Mach.
    CHECK_NEAR(liftCurve(liftSlopeMach(cl, 0.5), A, A), liftCurve(cl, A, A) * f05,
               1e-12, "the stall peak scales too");
    CHECK_NEAR(liftCurve(liftSlopeMach(cl, 0.5), 1.5 * A, A),
               liftCurve(cl, 1.5 * A, A) * f05, 1e-12, "and the droop");
    // A deflected surface is the same curve with the deflection in the angle seat.
    const glm::dvec3 dir(0.0, 1.0, 0.0);
    CHECK_NEAR(glm::length(controlForce(100.0, 2.0, liftSlopeMach(cl, 1.62), 0.1, dir, A)),
               glm::length(controlForce(100.0, 2.0, cl, 0.1, dir, A))
               * (liftSlopeMach(cl, 1.62) / cl), 1e-9,
               "controlForce: a fin's authority moves with the air");
    // And the derivative the neutral point sums: liftSlopePerQ tilts the flow, which
    // is a ROTATION, so |flow| -- the Mach -- is the same at both samples.
    const glm::dmat3 rot(1.0);
    const glm::dvec3 flow(0.0, 0.0, 100.0);   // dead along the nose, AoA 0
    const glm::dvec3 lat(1.0, 0.0, 0.0);
    const PartAero pa = partAero(rot, flow);
    const double s0 = glm::length(liftSlopePerQ(pa, rot, flow, 4.0, cl, A, lat));
    const double sM = glm::length(liftSlopePerQ(pa, rot, flow, 4.0,
                                                liftSlopeMach(cl, 0.5), A, lat));
    CHECK_NEAR(s0, 4.0 * cl, 1e-6, "a 4 m2 wing at 6/rad: the slope is not vacuous");
    CHECK_NEAR(sM, s0 * f05, 1e-9, "liftSlopePerQ: the same factor, one read");
    // The authority budget's row (#487's damping source) is q*S*cl by construction.
    const glm::dvec3 ri(0.0, 0.0, -3.0), push(0.0, 1.0, 0.0), about(1.0, 0.0, 0.0);
    CHECK_NEAR(glm::length(surfaceTorquePerRadian(100.0, 2.0, cl, ri, push, about)),
               600.0 * cl, 1e-9, "a 3 m lever: the row is not vacuous");
    CHECK_NEAR(glm::length(surfaceTorquePerRadian(100.0, 2.0, liftSlopeMach(cl, 2.0),
                                                  ri, push, about)),
               glm::length(surfaceTorquePerRadian(100.0, 2.0, cl, ri, push, about))
               * (liftSlopeMach(cl, 2.0) / cl), 1e-12,
               "surfaceTorquePerRadian: inherits, does not carry");
}

static void test_controlForce() {
    printf("== controlForce: the deflection-driven steering force ==\n");
    const double q = 100.0;  // dynamic pressure
    const double S = 2.0;    // control area
    const double cl = 6.0;   // deflection effectiveness (per radian)
    const double A = 0.26;   // rad: where a deflected plate stops biting (#494)
    const glm::dvec3 dir(0.0, 1.0, 0.0);  // force direction (the up axis)

    // Below the stall angle it is the old law exactly: double the deflection ->
    // double the force.
    {
        const glm::dvec3 f1 = controlForce(q, S, cl, 0.10, dir, A);
        const glm::dvec3 f2 = controlForce(q, S, cl, 0.20, dir, A);
        CHECK_TRUE(glm::length(f2 - 2.0 * f1) < 1e-9, "linear in delta");
        CHECK_NEAR(glm::length(f1), q * S * cl * 0.10, 1e-9, "magnitude = q*S*cl*delta");
    }

    /* THE DROOP (#494): authority peaks at the stall angle and is gone by twice
       it. A surface turned past its stall angle gives the player MORE TRAVEL, not
       more force -- which is what makes the induced bill (the other half of #494)
       safe to charge: the old law would have billed a CL of cl*0.35 = 2.1 on a
       plate that cannot make one. */
    {
        CHECK_NEAR(glm::length(controlForce(q, S, cl, A, dir, A)), q * S * cl * A, 1e-9,
                   "peaks at the stall angle");
        // Past the peak the force falls but keeps its sign, up to 2x.
        CHECK_TRUE(glm::length(controlForce(q, S, cl, 0.35, dir, A))
                   < glm::length(controlForce(q, S, cl, A, dir, A)),
                   "past the stall angle, travel buys less force");
        CHECK_TRUE(glm::dot(controlForce(q, S, cl, 0.35, dir, A), dir) > 0.0,
                   "still bites at the shipped travel stop");
        CHECK_NEAR(glm::length(controlForce(q, S, cl, 2.0 * A, dir, A)), 0.0, 1e-9,
                   "zero by twice the stall angle");
        // Monotonic fall from the peak to the zero, so the stick never reverses.
        double prev = glm::length(controlForce(q, S, cl, A, dir, A));
        for(double d = A; d <= 2.0 * A; d += 0.01) {
            const double now = glm::length(controlForce(q, S, cl, d, dir, A));
            CHECK_TRUE(now <= prev + 1e-12, "the droop never climbs back");
            prev = now;
        }
    }

    // The sign follows the deflection (symmetric, like a symmetric section) --
    // in the droop too, so a surface never pulls against its own stick.
    {
        const glm::dvec3 fpos = controlForce(q, S, cl, +0.10, dir, A);
        const glm::dvec3 fneg = controlForce(q, S, cl, -0.10, dir, A);
        CHECK_TRUE(glm::length(fpos + fneg) < 1e-9, "odd in delta");
        CHECK_TRUE(glm::dot(fpos, dir) > 0.0, "positive delta -> +dir");
        CHECK_TRUE(glm::dot(fneg, dir) < 0.0, "negative delta -> -dir");
        const glm::dvec3 spos = controlForce(q, S, cl, +0.32, dir, A);
        CHECK_TRUE(glm::length(spos + controlForce(q, S, cl, -0.32, dir, A)) < 1e-9,
                   "odd in delta past the stall angle too");
    }

    // The force is exactly along `dir` (no lateral component).
    {
        const glm::dvec3 f = controlForce(q, S, cl, 0.15, dir, A);
        CHECK_TRUE(glm::length(glm::cross(f, dir)) < 1e-6 * glm::length(f),
                   "parallel to dir");
    }

    // Scales with q, area, and cl (each linearly).
    {
        CHECK_TRUE(glm::length(controlForce(2.0*q, S, cl, 0.1, dir, A))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir, A)),
                   "linear in q");
        CHECK_TRUE(glm::length(controlForce(q, 2.0*S, cl, 0.1, dir, A))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir, A)),
                   "linear in area");
        CHECK_TRUE(glm::length(controlForce(q, S, 2.0*cl, 0.1, dir, A))
                   == 2.0 * glm::length(controlForce(q, S, cl, 0.1, dir, A)),
                   "linear in cl");
    }

    // Degenerate inputs -> zero (no air, no area, no effectiveness, no deflection).
    CHECK_TRUE(controlForce(0.0, S, cl, 0.1, dir, A) == glm::dvec3(0.0), "no q -> zero");
    CHECK_TRUE(controlForce(q, 0.0, cl, 0.1, dir, A) == glm::dvec3(0.0), "no area -> zero");
    CHECK_TRUE(controlForce(q, S, 0.0, 0.1, dir, A) == glm::dvec3(0.0), "no cl -> zero");
    CHECK_TRUE(controlForce(q, S, cl, 0.0, dir, A) == glm::dvec3(0.0), "no delta -> zero");
}

static void test_controlDeflectionSign() {
    printf("== controlDeflectionSign: position-dependent steering sign ==\n");
    // Ship axes: nose=+Z, right=+X, up=+Y. The reaction wheel's convention:
    // W (stick[1]=+1) -> torque about -right (nose up); A/D -> about -up.
    const glm::dvec3 nose(0,0,1), right(1,0,0), up(0,1,0);
    /* controlRigLever reads the mount's push column (mount[1], the direction the
       plate pushes at zero angle of attack), so these cases can hold the push
       direction fixed and vary where the part sits. Any proper rotation with that
       column gives the same lever -- which is itself worth pinning, since a part's
       span and chord do not get a vote on which way it steers. */
    const auto pushMount = [](const glm::dvec3 &push) {
        const glm::dvec3 span = (std::fabs(push.z) < 0.9) ? glm::dvec3(0,0,1)
                                                          : glm::dvec3(1,0,0);
        return glm::dmat3(span, push, glm::cross(span, push));
    };
    const auto lever = [&](const glm::dvec3 &ri, const glm::dvec3 &push,
                           const glm::dvec3 &about) {
        const glm::dmat3 mount = pushMount(push);
        CHECK_NEAR(glm::determinant(mount), 1.0, 1e-12, "a mount is a proper rotation");
        return controlRigLever(ri, mount, about);
    };

    // A TAIL surface (behind the CG, ri ~ -nose) with W (stick[1]=+1) must
    // produce a -right torque (nose UP). The deflection sign makes the force
    // F = sign * stick[1] * up, and the torque is ri x F.
    {
        const glm::dvec3 ri = -nose;  // tail
        const double sign = controlDeflectionSign(lever(ri, up, right));  // pitch
        const glm::dvec3 F = up * (sign * 1.0);  // W: stick[1]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, right) < 0.0, "tail W -> -right torque (nose up)");
    }

    // A CANARD surface (ahead of the CG, ri ~ +nose) with W must ALSO
    // produce a -right torque (nose UP) -- the deflection sign flips.
    {
        const glm::dvec3 ri = +nose;  // canard
        const double sign = controlDeflectionSign(lever(ri, up, right));  // pitch
        const glm::dvec3 F = up * (sign * 1.0);  // W: stick[1]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, right) < 0.0, "canard W -> -right torque (nose up)");
    }

    // The tail and canard signs are OPPOSITE (that's the whole point).
    {
        const double tailSign   = controlDeflectionSign(lever(-nose, up, right));
        const double canardSign = controlDeflectionSign(lever(+nose, up, right));
        CHECK_TRUE(tailSign == -canardSign, "tail and canard signs are opposite");
    }

    // Yaw: a tail with stick[2]=+1 must produce a -up torque (matching the
    // wheel's A/D convention about -up).
    {
        const glm::dvec3 ri = -nose;  // tail
        const double sign = controlDeflectionSign(lever(ri, right, up));  // yaw
        const glm::dvec3 F = right * (sign * 1.0);  // stick[2]=+1
        const glm::dvec3 torque = glm::cross(ri, F);
        CHECK_TRUE(glm::dot(torque, up) < 0.0, "tail yaw -> -up torque");
    }

    // Roll: an aileron pair, laterally offset (ri ~ +/-right), force in the
    // up plane. The wheel's Q (stick[0]=+1) is a +nose torque, so BOTH
    // ailerons roll that way (right up / left down -- differential). The roll
    // targetSign is +1 (the wheel's roll is +nose, unlike pitch/yaw's -axis).
    {
        const double rRight = controlDeflectionSign(lever(right,  up, nose), +1.0);
        const double rLeft  = controlDeflectionSign(lever(-right, up, nose), +1.0);
        const glm::dvec3 Fright = up * (rRight * 1.0);  // stick[0]=+1
        const glm::dvec3 Fleft  = up * (rLeft  * 1.0);  // stick[0]=+1
        CHECK_TRUE(glm::dot(glm::cross(right, Fright), nose) > 0.0,
                   "right aileron Q -> +nose torque");
        CHECK_TRUE(glm::dot(glm::cross(-right, Fleft), nose) > 0.0,
                   "left aileron Q -> +nose torque");
        CHECK_TRUE(rRight == -rLeft, "aileron pair deflects opposite (roll)");
        CHECK_TRUE(Fright + Fleft == glm::dvec3(0.0),
                   "aileron pair: net force cancels (pure roll)");
    }

    // A surface exactly at the COM (zero lever) -> the rig cannot decide a sign.
    // The bare helper still answers +-1 (defined, no crash), but controlDeflection
    // asks it for no travel at all -- pinned in test_control_rigging.
    {
        const double sign = controlDeflectionSign(lever(glm::dvec3(0.0), up, right));
        CHECK_TRUE(sign == -1.0, "COM surface: the bare helper returns targetSign");
    }
}

/* #453 pinned at the level that decides the design: the control force leaves
   along each SURFACE's own normal (partAero), while the deflection SIGN is
   still chosen about the SHIP's commanded axis, because the stick promises a
   SHIP torque. A rudder and its twin, mounted 180 deg apart on the stack, have
   opposite plate normals, so they need OPPOSITE deflections to push the SAME
   way -- and choosing the sign about the ship's axis delivers exactly that.
   Choosing it about the PART's own axis does not: a yaw surface's plate normal
   IS its own up, so the triple product controlDeflectionSign measures is zero by
   construction (float noise on a real, slightly non-orthonormal frame), and each
   instance's sign is then decided by that noise. Measured on jet.json and
   glider.json that reverses the yaw force (51450 N -> -51450 N, and the moment
   off by 3.8e5 N m); on the exact frames below the pair cancels outright. Either
   way the stick stops meaning anything -- pin both, because the wrong version
   looks like a tidy simplification. */
static void test_control_pair_frames() {
    printf("== control pair: surface normal, ship-axis command ==\n");
    const glm::dvec3 right(1,0,0), up(0,1,0);
    const double q = 1000.0, S = 2.0, clc = 4.0, maxDefl = 0.35, stick = 1.0;
    // The shipped deflection stall (#494). This test is about GEOMETRY, and the
    // droop cannot change any sign here: maxDefl < 2*A, so full travel still bites.
    const double A = 0.26;
    const glm::dvec3 vrel(0.0, 0.0, 100.0);   // dead ahead: alpha = 0

    /* The shipped rudder pose: the plate's span (mesh +X) lies along the ship's
       up, its face normal (mesh +Y) along the ship's -right, chord along +Z --
       a vertical fin, exactly as res/ships/jet.json bolts it on at 90 deg. */
    const glm::dmat3 mUp(glm::dvec3(0,1,0), glm::dvec3(-1,0,0), glm::dvec3(0,0,1));
    // Its twin, mounted the other way round: the same pose rolled 180 deg about
    // the part's own nose -- which is what a 90/270 (or 0/180) attach angle does.
    // Built from exact columns rather than angleAxis(pi, ...), whose sin(pi) is
    // ~1.2e-16, so the exactness claimed below is real rather than at the noise
    // floor.
    const glm::dmat3 flip(glm::dvec3(-1,0,0), glm::dvec3(0,-1,0), glm::dvec3(0,0,1));
    const glm::dmat3 mDn = mUp * flip;

    struct Surf { double defl; glm::dvec3 F; };
    const auto drive = [&](const glm::dmat3 &mount, const glm::dvec3 &ri,
                           const glm::dvec3 &aboutAxis) {
        const PartAero pa = partAero(mount, vrel);
        /* The aero step's own helper, so these pins hold the wiring and not a
           paraphrase of it (#495 review). At this dead-ahead flow the mount's push
           column and pa.liftDir are the same vector, so the numbers below are
           unchanged; test_control_rigging covers the difference. */
        const double defl = controlDeflection(pa, ri, mount, aboutAxis, -1.0,
                                              stick, maxDefl);
        return Surf{ defl, controlForce(q, S, clc, defl, pa.liftDir, A) };
    };
    const glm::dvec3 riUp(0, 2.0, -3.0), riDn(0, -2.0, -3.0);

    // The normals are opposite -- that is the whole point.
    const glm::dvec3 nUp = partAero(mUp, vrel).liftDir;
    const glm::dvec3 nDn = partAero(mDn, vrel).liftDir;
    CHECK_TRUE(glm::length(nUp + nDn) < 1e-12, "mirrored mounts face opposite ways");

    // Ship-axis command: the pair ADDS, and it yaws.
    const Surf sA1 = drive(mUp, riUp, up);
    const Surf sA2 = drive(mDn, riDn, up);
    CHECK_TRUE(sA1.F == sA2.F, "mirrored pair pushes the SAME way");
    CHECK_TRUE(glm::length(sA1.F + sA2.F) == 2.0 * glm::length(sA1.F),
               "mirrored pair keeps full yaw authority");
    CHECK_TRUE(std::abs(glm::dot(sA1.F + sA2.F, right)) > 0.0, "and that way is a sideslip");
    CHECK_TRUE(glm::dot(glm::cross(riUp, sA1.F) + glm::cross(riDn, sA2.F), up) != 0.0,
               "the pair torques the ship about its yaw axis");

    /* And the MESH follows the force: opposite spans plus opposite deflections
       must land both twins in the same pose (that is why the shipped animation
       did not change), with each chord tip swinging along its own surface's
       force. A hinge chosen from the ship frame instead of the surface's span
       puts that swing 90 deg off -- the rudders sweep in their own plane. */
    const glm::dquat vA1 = controlSurfaceVisualRot(sA1.defl, mUp);
    const glm::dquat vA2 = controlSurfaceVisualRot(sA2.defl, mDn);
    CHECK_TRUE(std::fabs(glm::dot(vA1, vA2)) > 0.999999, "mirrored twins swing alike");
    const glm::dvec3 tip = glm::mat3_cast(vA1) * mUp[2] - mUp[2];
    CHECK_TRUE(glm::dot(tip, sA1.F) > 0.9 * glm::length(tip) * glm::length(sA1.F),
               "the chord tip swings along the force that surface makes");

    // Part-axis command (the tempting wrong simplification). The deciding triple
    // product is zero by construction, so the sign used to be noise; here it comes out
    // exactly and the pair cancels. With the authority gate the same rig is not even
    // asked to travel: a force along an axis cannot turn that axis, whatever the sign.
    const double dShip = glm::dot(glm::cross(riUp, nUp), up);
    const double dPart = glm::dot(glm::cross(riUp, nUp), mUp[1]);
    CHECK_TRUE(std::fabs(dPart) < 1e-15, "part-axis: the sign is decided by float noise");
    CHECK_TRUE(std::fabs(dShip) > 1.0, "ship-axis: the sign is decided by the lever arm");
    const Surf sB1 = drive(mUp, riUp, mUp[1]);
    const Surf sB2 = drive(mDn, riDn, mDn[1]);
    CHECK_TRUE(sB1.defl == 0.0 && sB2.defl == 0.0,
               "no authority about the push axis: the plate is asked for no travel");
    // What the OLD rule did with the same rig, reconstructed here rather than asserted
    // against zeros: it took a coin-flip sign off a triple product that is exactly zero,
    // both twins came out at +travel, and their forces -- opposite by mount -- cancelled.
    const auto oldDefl = [&](const glm::dmat3 &mount, const glm::dvec3 &ri) {
        const glm::dvec3 n = partAero(mount, vrel).liftDir;
        return ((glm::dot(glm::cross(ri, n), mount[1]) >= 0.0) ? 1.0 : -1.0)
             * stick * maxDefl;
    };
    const glm::dvec3 fOld1 = controlForce(q, S, clc, oldDefl(mUp, riUp), nUp, A);
    const glm::dvec3 fOld2 = controlForce(q, S, clc, oldDefl(mDn, riDn), nDn, A);
    CHECK_TRUE(std::fabs(fOld1.x) > 1e-6 && std::fabs(fOld2.x) > 1e-6,
               "the old rule did move them, with real force");
    CHECK_TRUE(glm::length(fOld1 + fOld2) < 1e-12,
               "and the pair cancelled: zero yaw authority, reached by noise");
}

/* WHICH WAY A SURFACE IS RIGGED is a build fact, not a flow fact (#495), and so is how
   much authority that rig has. The deflection used to be re-decided every substep from
   partAero's LIVE liftDir, so the wind got a vote on the pushrod. Pin the replacement
   at the level the aero step actually calls -- drag.h::controlDeflection -- because a
   paraphrase of the call site pins nothing.
   The two rules agree everywhere the shipped airframes actually fly -- measured over
   eight beds and ~150k actuated-surface rows, and their [drag], [ctrl] and [surface]
   logs are byte-identical either way -- so most of this test exists for the two corners
   where they do not:
   - An off-centre fin. Its yaw leverage (for the mount below, -3*liftDir.x + liftDir.z)
     crosses zero near 72 deg of flow angle with clear leverage either side. There the
     old rule reversed the sign, and with it the FULL control force, mid-maneuver and at
     whatever q the ship was flying; the rigging rule keeps the travel, turns the force
     with the flow, and lets the moment pass through zero on its own. (Those flow angles
     put the plate far past its own stall, which is the separate problem in the same
     corner -- controlForce answers deflection alone and so bills a broadside surface its
     full q*S*cl*defl -- filed as #508.)
   - A rig with no authority at all: a plate whose force line runs through the pivot is
     not a control about that axis, and it is now asked for no travel rather than full
     travel with a coin-flip sign. */
static void test_control_rigging() {
    printf("== control rigging: the mount decides, not the flow (#495) ==\n");
    const glm::dvec3 up(0,1,0);
    /* The shipped rudder pose: span along the ship's up, plate normal along
       -right, chord along +nose -- a vertical fin, clock 90. */
    const glm::dmat3 fin(glm::dvec3(0,1,0), glm::dvec3(-1,0,0), glm::dvec3(0,0,1));

    const auto liveLev = [&](const glm::dvec3 &ri, const glm::dvec3 &vrel) {
        return glm::dot(glm::cross(ri, partAero(fin, vrel).liftDir), up);
    };
    // The rule as it stood before #495, kept here only to pin that the two differ.
    // It is deliberately NOT in drag.h any more: taking a direction where only a
    // mount will do is the mistake, not a convenience.
    const auto oldRule = [&](const glm::dvec3 &ri, const PartAero &pa) {
        return (glm::dot(glm::cross(ri, pa.liftDir), up) >= 0.0) ? -1.0 : 1.0;
    };

    // Sweep the wind over the fin, dead ahead to straight astern. The mount never
    // moves, so neither does the answer it is asked.
    const glm::dvec3 riC(0, 0, -3.0);   // a normal, centreline fin: 3 m behind CG
    CHECK_TRUE(std::fabs(controlRigLever(riC, fin, up)) > 1.0,
               "rigged with real yaw authority");
    for(int deg = 0; deg <= 180; deg += 10) {
        const double r = glm::radians((double)deg);
        const glm::dvec3 vrel = 100.0 * glm::dvec3(-std::sin(r), 0.0, std::cos(r));
        const PartAero pa = partAero(fin, vrel);
        const double rigLev = controlRigLever(riC, fin, up);
        const double lev = liveLev(riC, vrel);
        /* The live leverage, by contrast, wanders over the same sweep. For THIS mount
           (a rectangular centreline fin) it only reaches zero exactly broadside, and
           never crosses -- which is precisely why no shipped airframe ever showed the
           flip. Everywhere else the two rules genuinely do agree, and saying so is an
           assertion, not a tautology. */
        if(deg == 90) {
            /* The one flow where the two rules come apart for a centreline fin. The
               live liftDir collapses onto the ship's nose axis (the normalised residue
               of a nearly vanishing vector), so the flow-decided leverage is EXACTLY
               zero -- a cusp that touches zero without crossing, which is why the old
               rule never flipped here. partAero refuses only a residue that is exactly
               zero: the sweep's cos(90 deg) leaves 6.1e-17, so the frame is accepted
               and the force is billed at full strength either way. That is #508. */
            CHECK_TRUE(std::fabs(lev) < 1e-12,
                       "centreline fin: the flow-decided leverage touches zero broadside");
            CHECK_TRUE(std::fabs(rigLev) > 1.0,
                       "centreline fin: the mount's authority does not care about the flow");
        } else {
            CHECK_TRUE(std::fabs(lev) > 1e-3, "centreline fin: no touch anywhere else");
            CHECK_TRUE((lev >= 0.0) == (rigLev >= 0.0),
                       "centreline fin: rig sign and live sign agree");
            CHECK_TRUE(controlDeflection(pa, riC, fin, up, -1.0, 1.0, 0.35)
                           == oldRule(riC, pa) * 0.35,
                       "centreline fin: the shipped answer is unchanged by #495");
        }
    }

    // Exactly broadside there is no lift axis at all, so the frame is unusable and
    // controlDeflection asks for no travel -- for the same reason the force itself is
    // zero, not because a sign had to be guessed.
    {
        const PartAero pa = partAero(fin, glm::dvec3(-100.0, 0.0, 0.0));
        CHECK_TRUE(!pa.valid, "broadside plate: no usable aero frame");
        CHECK_TRUE(glm::length(pa.liftDir) == 0.0, "broadside plate: no lift direction");
        CHECK_TRUE(controlDeflection(pa, riC, fin, up, -1.0, 1.0, 0.35) == 0.0,
                   "unusable frame: no travel asked for");
    }

    /* The mount the old rule could not keep steady: the same fin 1 m to leeward of
       the CG (a builder reaching for lever arm). tan(phi) = 3/1 puts the crossing at
       71.6 deg of flow angle, with clear leverage either side -- 0.63 m then -0.46 m. */
    const glm::dvec3 riOff(-1.0, 0.0, -3.0);
    const auto flowAt = [](double deg) {
        const double r = glm::radians(deg);
        return 100.0 * glm::dvec3(-std::sin(r), 0.0, std::cos(r));
    };
    const double lexB = liveLev(riOff, flowAt(60.0));
    const double lexA = liveLev(riOff, flowAt(80.0));
    CHECK_TRUE(lexB > 0.5 && lexA < -0.4, "off-centre fin: its yaw leverage crosses zero");
    const double rigLevOff = controlRigLever(riOff, fin, up);
    CHECK_TRUE(std::fabs(rigLevOff) > 0.5,
               "the mount keeps real authority across the whole sweep");
    CHECK_TRUE(oldRule(riOff, partAero(fin, flowAt(60.0)))
            != oldRule(riOff, partAero(fin, flowAt(80.0))),
               "flow-decided sign flips across that crossing (the #495 bug)");
    /* And the consequence, not just the sign. With the rigging fixed, a held command
       keeps its travel and the force turns with the flow, so the yaw moment runs
       through zero as the leverage does. With the flow deciding, the travel reverses at
       the same instant, which is why that moment never crosses zero -- it keeps the sign
       the stick was promised, and the FORCE is what jumps, by its whole magnitude, at
       whatever q the ship is flying. */
    const double q = 1000.0, S = 2.0, clc = 6.0, A = 0.26, maxDefl = 0.35;
    const auto momentRig = [&](double deg) {
        const glm::dvec3 v = flowAt(deg);
        const PartAero pa = partAero(fin, v);
        const double defl = controlDeflection(pa, riOff, fin, up, -1.0, 1.0, maxDefl);
        const glm::dvec3 F = controlForce(q, S, clc, defl, pa.liftDir, A);
        return glm::dot(glm::cross(riOff, F), up);
    };
    const auto momentLive = [&](double deg) {
        const glm::dvec3 v = flowAt(deg);
        const PartAero pa = partAero(fin, v);
        const glm::dvec3 F = controlForce(q, S, clc, oldRule(riOff, pa) * maxDefl,
                                          pa.liftDir, A);
        return glm::dot(glm::cross(riOff, F), up);
    };
    CHECK_TRUE(std::fabs(momentRig(60.0)) > 1.0 && std::fabs(momentRig(80.0)) > 1.0,
               "both sides of the crossing carry real authority");
    CHECK_TRUE(momentRig(60.0) * momentRig(80.0) < 0.0,
               "rigged: the moment reverses with the flow, through zero");
    CHECK_TRUE(momentLive(60.0) * momentLive(80.0) > 0.0,
               "flow-decided: the moment keeps the promised sign, never crossing zero");
    CHECK_TRUE(momentRig(60.0) == momentLive(60.0), "before the crossing the two agree");
    CHECK_TRUE(momentRig(80.0) == -momentLive(80.0),
               "past it they hold opposite sides of the same leverage");

    /* AUTHORITY, the other half of the mount's vote. A surface rigged so its force line
       runs through the pivot has no moment about the axis its stick promises: a yaw
       command on a plate that pushes fore-and-aft (clock 0), or a part sitting on the
       CG. Full travel there is not neutral -- the force is still q*S*cl*defl in the
       wind's chosen direction, and the airframe pays for it. Measured on a skiff with
       its rudder turned to clock 0: +20.1 deg of travel and a peak airframe
       |tau| = 2560 N m, above the 1539 N m of the same ship rigged properly; asked for
       no travel, that peak is 246 N m. Ask for no travel and it feels dead.
       A WEAK rig is a different thing and still travels: this is a degeneracy test, not
       an authority threshold (telling the builder is #509). */
    {
        const double cmd = 1.0;
        const glm::dmat3 finClock0(glm::dvec3(1,0,0), glm::dvec3(0,0,1), glm::dvec3(0,-1,0));
        const PartAero paFlat = partAero(finClock0, flowAt(20.0));
        CHECK_TRUE(paFlat.valid, "a clock-0 fin still has a usable frame");
        CHECK_TRUE(std::fabs(controlRigLever(riC, finClock0, up)) < 1e-12,
                   "a clock-0 fin has no yaw leverage at all");
        CHECK_TRUE(controlDeflection(paFlat, riC, finClock0, up, -1.0, cmd, maxDefl) == 0.0,
                   "no authority: no travel asked for");
        const PartAero paC = partAero(fin, flowAt(20.0));
        CHECK_TRUE(controlDeflection(paC, glm::dvec3(0.0), fin, up, -1.0, cmd, maxDefl)
                       == 0.0,
                   "a steering part mounted on the CG is not a steering part");
        const glm::dvec3 riWeak(0.0, 0.0, -0.05);   // 5 cm behind CG: weak, not degenerate
        CHECK_TRUE(controlDeflection(paC, riWeak, fin, up, -1.0, cmd, maxDefl)
                       == -maxDefl, "a weak rig still gets full travel");
        CHECK_TRUE(controlDeflection(paC, riC, fin, up, -1.0, 0.0, maxDefl) == 0.0,
                   "no command, no travel");

        /* The cutoff SCALES with the arm (|lever|^2 <= 1e-12 * |ri|^2), which is what makes
           it a geometry test rather than a floor on leverage. A fin 3 m behind the CG turned
           to within 1e-4 rad of the degenerate line has 3e-4 m of leverage -- a hundred
           times the 3e-6 m cutoff for THAT arm -- and gets full travel; nudge the same fin
           to 1e-8 rad and it is degenerate for that arm. An absolute lever floor would fail
           the first; a fixed angle threshold would fail the second on a short arm. And the
           exactly-180-deg mis-rig is caught even though angleAxis(pi) leaves sin(pi) = 1.2e-16
           of rot in the mount, because the cutoff sits far above that noise. */
        const glm::dvec3 riAft(0.0, 0.0, -3.0);
        const double cutoff = 1e-6 * glm::length(riAft);
        for(double tiltRad : {0.0, 1e-4, 1e-8}) {
            const glm::dmat3 mt = glm::mat3_cast(
                glm::angleAxis(M_PI - tiltRad, up)) * finClock0;
            const PartAero paM = partAero(mt, flowAt(20.0));
            CHECK_TRUE(paM.valid, "a fore-aft fin in a 20 deg flow still has a frame");
            const double lev = controlRigLever(riAft, mt, up);
            const double defl = controlDeflection(paM, riAft, mt, up, -1.0, cmd, maxDefl);
            const bool degenerate = std::fabs(lev) <= cutoff;
            CHECK_TRUE(std::fabs(std::fabs(lev) - 3.0 * std::sin(tiltRad)) < 1e-12,
                       "the rig lever is |ri| * sin(off the degenerate line)");
            CHECK_TRUE((defl == 0.0) == degenerate,
                       "the gate follows the arm-scaled cutoff, not an absolute lever");
            if(tiltRad == 1e-4) {
                CHECK_TRUE(!degenerate && std::fabs(defl) == maxDefl,
                           "a long arm with a small angle still travels fully");
            }
        }
        /* The same tilt on a SHORT arm, which is what separates an arm-scaled cutoff from
           an absolute floor: 3 mm behind the CG at 1e-4 rad gives 3e-7 m of lever, ten
           times the 3e-9 m cutoff for THAT arm, so it travels -- while a fixed 3e-6 m
           floor (what the loop above alone would let a mutant install) gates it. */
        {
            const glm::dvec3 riShort(0.0, 0.0, -3e-3);
            const glm::dmat3 mt = glm::mat3_cast(glm::angleAxis(M_PI - 1e-4, up)) * finClock0;
            const double lev = controlRigLever(riShort, mt, up);
            CHECK_TRUE(std::fabs(lev) > 1e-6 * glm::length(riShort)
                           && std::fabs(lev) < 3e-6,
                       "3e-7 m of lever: above the arm-scaled cutoff, below an absolute one");
            CHECK_TRUE(controlDeflection(partAero(mt, flowAt(20.0)), riShort, mt, up,
                                         -1.0, 1.0, maxDefl) != 0.0,
                       "so a short arm with real leverage still travels");
        }
    }
}

/* The visual hinge must tip the mesh the SAME way the surface's own force
   pushes: controlForce gives F = defl * partAero(...).liftDir (the plate's own
   normal out of the flow), and a plate hinged along its own SPAN moves its chord
   tip along that normal. Pinned on the mounts that actually ship -- including
   the 90/270 pair the jet's and glider's rudders use, where a hinge picked from
   the SHIP frame swings the fin about its own face instead (28 deg off, and
   perpendicular to its force). */
static void test_controlSurfaceVisualRot() {
    printf("== controlSurfaceVisualRot: mesh tip follows the surface's own force ==\n");
    const glm::dvec3 nose(0,0,1), up(0,1,0);
    const double d = 0.3;  // rad; +defl -> +force along the plate's own normal
    // Flying ~5.7 deg nose-up, so the flow sits in the ship's Y/Z plane.
    const double aoa = 0.1;
    const glm::dvec3 vrel(0.0, -100.0 * std::sin(aoa), 100.0 * std::cos(aoa));

    /* A surface mesh is a plate: span +X, normal +Y, chord +Z (fin_r1h2.obj). */
    const glm::dmat3 level(1.0);                                    // horizontal plate
    const glm::dmat3 rudder(glm::dvec3(0,1,0), glm::dvec3(-1,0,0), glm::dvec3(0,0,1));
    const glm::dmat3 flip(glm::dvec3(-1,0,0), glm::dvec3(0,-1,0), glm::dvec3(0,0,1));
    const glm::dmat3 upsideDown = level * flip;                     // 180 deg about its nose
    const glm::dmat3 slant = glm::mat3_cast(glm::angleAxis(glm::radians(30.0), nose));
    const glm::dmat3 mounts[] = { level, rudder, upsideDown, slant };
    const char *names[] = { "level", "shipped rudder", "upside down", "30 deg slant" };

    for(int m = 0; m < 4; m++) {
        const glm::dmat3 &mt = mounts[m];
        const PartAero pa = partAero(mt, vrel);
        CHECK_TRUE(pa.valid, "mount has a usable flow");
        const glm::dvec3 tipPos
            = glm::mat3_cast(controlSurfaceVisualRot(+d, mt)) * mt[2] - mt[2];
        const glm::dvec3 tipNeg
            = glm::mat3_cast(controlSurfaceVisualRot(-d, mt)) * mt[2] - mt[2];
        /* Reversing the command reverses the SWING (the tip's travel along the
           plate normal) and leaves the chord's foreshortening alone -- the
           (cos t - 1) part, second order in t, is even in t. */
        CHECK_TRUE(std::fabs(glm::dot(tipPos, mt[1]) + glm::dot(tipNeg, mt[1])) < 1e-15,
                   (std::string(names[m]) + ": -defl reverses the swing").c_str());
        CHECK_TRUE(std::fabs(glm::dot(tipPos, mt[2]) - glm::dot(tipNeg, mt[2])) < 1e-15,
                   (std::string(names[m]) + ": -defl keeps the chord foreshortening").c_str());
        for(int s = 0; s < 2; s++) {
            const double defl = (s == 0) ? +d : -d;
            const glm::dquat q = controlSurfaceVisualRot(defl, mt);
            const glm::dmat3 R = glm::mat3_cast(q);
            const glm::dvec3 tip = R * mt[2] - mt[2];          // where the chord tip goes
            const glm::dvec3 force = pa.liftDir * defl;        // what the law applies
            const std::string tag = std::string(names[m])
                                  + (defl > 0.0 ? " +defl" : " -defl");
            // The whole point of the helper: mesh motion matches force.
            CHECK_TRUE(glm::dot(tip, force)
                           > 0.9 * glm::length(tip) * glm::length(force),
                       ("tip swings along the force (" + tag + ")").c_str());
            // The SPAN is the hinge line, so it does not move; the plate normal
            // does. Pinning both is what separates a hinge about the span from a
            // spin about the face normal, which is the 90 deg error.
            CHECK_TRUE(glm::dot(R * mt[0], mt[0]) > 0.9999,
                       ("span stays put (" + tag + ")").c_str());
            CHECK_TRUE(glm::dot(R * mt[1], mt[1]) < 0.999,
                       ("normal swings, so this is not a spin about the face ("
                        + tag + ")").c_str());
        }
    }

    /* A mirrored mount reverses its span AND (via controlDeflectionSign) its
       commanded deflection, so the world-space swing is the one the ship-frame
       rule used to produce: that is why the shipped fins do not change pose. */
    {
        const glm::dvec3 tA = glm::mat3_cast(controlSurfaceVisualRot(+d, level)) * nose - nose;
        const glm::dvec3 tB = glm::mat3_cast(controlSurfaceVisualRot(-d, upsideDown)) * nose
                            - nose;
        CHECK_TRUE(glm::length(tA - tB) < 1e-15, "mount flip + -defl == upright +defl");
    }

    /* The shipped rudder's span IS the ship's up, so on that mount the hinge axis
       is the ship's up -- the axis the pre-#453 root-frame code used. Anything
       else on that mount is the bug this pin exists for. */
    {
        const glm::dquat q = controlSurfaceVisualRot(d, rudder);
        const glm::dquat aboutShipUp = glm::angleAxis(-d, up);
        CHECK_TRUE(std::fabs(glm::dot(q, aboutShipUp)) > 0.9999,
                   "a 90-deg-mounted fin hinges on the ship's up, like it used to");
    }
}

static void test_controlCl() {
    printf("== controlCl: deflection effectiveness (cl_control, fallback cl) ==\n");
    // cl_control > 0 -> it is the deflection effectiveness (the part's own
    // number), even if cl (the lift slope) is also set.
    CHECK_TRUE(controlCl(0.0, 6.0) == 6.0, "cl_control set -> used");
    CHECK_TRUE(controlCl(6.0, 4.0) == 4.0, "cl_control overrides cl");
    // cl_control = 0 (unset) -> fall back to cl: the old single-"cl" behaviour
    // for a part that only declares cl.
    CHECK_TRUE(controlCl(6.0, 0.0) == 6.0, "cl_control 0 -> falls back to cl");
    CHECK_TRUE(controlCl(0.0, 0.0) == 0.0, "neither set -> 0");
}

/* #492: TRIM joins the COMMAND, before the sign and before the travel scale. The
   ship-level half of this (a trimmed surface holding a force with nothing pressed,
   the mirrored pair coming out with opposite raw deflections off one authored
   number) is flight/492-elevator-trim's job -- it needs the call-site's sign
   resolution. What is pinned here is the arithmetic that call site does with the
   number, and the one claim that keeps every shipped airframe exactly where it is:
   an unset trim leaves the command EXACTLY what the stick alone was, so the
   surface's neutral early-out still fires and a non-trimmed surface still costs
   nothing. */
static void test_control_trim() {
    printf("== trim: a standing command, capped at the surface's travel (#492) ==\n");
    CHECK_TRUE(controlCommandFrac(0.0, 0.0) == 0.0,
               "no stick, no trim -> exactly zero, so the neutral test still fires");
    // -0.0 is what a slider dragged to the left of centre can hand back, and the
    // caller's early-out compares against 0.0.
    CHECK_TRUE(controlCommandFrac(0.0, -0.0) == 0.0, "negative zero is still neutral");
    // The point of the field: a surface is commanded with nothing held.
    CHECK_NEAR(controlCommandFrac(0.0, 0.5), 0.5, 0.0, "trim alone commands the surface");
    CHECK_NEAR(controlCommandFrac(0.0, -0.25), -0.25, 0.0, "either way");
    // A surface cannot travel past its own stop, however the two are combined...
    CHECK_NEAR(controlCommandFrac(1.0, 0.5), 1.0, 0.0, "stick plus trim stops at full travel");
    CHECK_NEAR(controlCommandFrac(-1.0, -0.5), -1.0, 0.0, "and the same the other way");
    // ...but trim is not a ratchet: holding against it backs the surface off.
    CHECK_NEAR(controlCommandFrac(-1.0, 0.5), -0.5, 0.0,
               "stick against trim wins, reduced -- it does not latch");
    CHECK_NEAR(controlCommandFrac(0.5, 0.25), 0.75, 0.0, "stick and trim add up");
}

// --- Projected (silhouette) area of a body, from its convex-hull vertices.
//
// projectedArea(hullVerts, dir) is the area of the convex hull of the
// projected vertices -- the body's silhouette facing dir. Pinned against
// hand-computed values for a cube, a 32-gon prism (the "cylinder limit" the
// tanks are built from), and a thin box (the wing: face-on = the planform,
// edge-on = thickness * length). All vertex sets are built by hand -- no
// Bullet, no mesh -- so the law is pinned independently of the extraction.

static std::vector<glm::dvec3> cubeVerts() {
    // Side-2 cube, centred at the origin: 8 vertices.
    std::vector<glm::dvec3> v;
    v.push_back(glm::dvec3(-1, -1, -1));
    v.push_back(glm::dvec3( 1, -1, -1));
    v.push_back(glm::dvec3( 1,  1, -1));
    v.push_back(glm::dvec3(-1,  1, -1));
    v.push_back(glm::dvec3(-1, -1,  1));
    v.push_back(glm::dvec3( 1, -1,  1));
    v.push_back(glm::dvec3( 1,  1,  1));
    v.push_back(glm::dvec3(-1,  1,  1));
    return v;
}

static std::vector<glm::dvec3> prismVerts(int n = 32, double r = 1.0,
                                          double h = 3.0) {
    // A regular n-gon prism (a faceted cylinder): two rings of n vertices.
    std::vector<glm::dvec3> v;
    for(int k = 0; k < n; k++) {
        const double th = 2.0 * M_PI * k / n;
        const glm::dvec3 xy(r * std::cos(th), r * std::sin(th), 0.0);
        v.push_back(glm::dvec3(xy.x, xy.y, -h / 2.0));
        v.push_back(glm::dvec3(xy.x, xy.y,  h / 2.0));
    }
    return v;
}

static std::vector<glm::dvec3> boxVerts(double hx, double hy, double hz) {
    // Box of half-extents (hx, hy, hz): 8 vertices.
    std::vector<glm::dvec3> v;
    for(int i = 0; i < 8; i++) {
        v.push_back(glm::dvec3((i & 1) ?  hx : -hx,
                               (i & 2) ?  hy : -hy,
                               (i & 4) ?  hz : -hz));
    }
    return v;
}

static void test_projectedArea() {
    printf("== projectedArea: silhouette = hull of the projected vertices ==\n");

    // Cube: axis-on = one face (4); (1,0,1) shows two faces at 45 deg ->
    // 4 / sqrt(2) each -> 8 / sqrt(2) = 4 sqrt(2).
    const std::vector<glm::dvec3> cube = cubeVerts();
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.0, 0.0, 1.0)), 4.0, 1e-9,
               "cube axis-on == 4");
    CHECK_NEAR(projectedArea(cube, glm::dvec3(1.0, 0.0, 1.0)),
               8.0 / std::sqrt(2.0), 1e-9, "cube (1,0,1) == 4 sqrt(2)");

    // Symmetry: a convex body's silhouette is the same from either side.
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.3, -0.5, 0.8)),
               projectedArea(cube, glm::dvec3(-0.3, 0.5, -0.8)), 1e-9,
               "silhouette symmetric in dir (cube)");

    // 32-gon prism (the "cylinder limit"): end-on ~ pi (within 1%), side-on
    // ~ 2 r h (within 1%). Pins that the hull-of-vertices reproduces the
    // analytic cylinder areas the game's tanks are built from.
    const std::vector<glm::dvec3> prism = prismVerts(32, 1.0, 3.0);
    const double end = projectedArea(prism, glm::dvec3(0.0, 0.0, 1.0));
    CHECK_TRUE(std::fabs(end - M_PI) < 0.01 * M_PI,
               "prism end-on within 1% of pi (cylinder limit)");
    const double side = projectedArea(prism, glm::dvec3(1.0, 0.0, 0.0));
    CHECK_TRUE(std::fabs(side - 6.0) < 0.01 * 6.0,
               "prism side-on within 1% of 2 r h (cylinder limit)");
    // More sides -> closer to the cylinder: 6-gon is coarser, 32-gon tighter.
    const double end6 = projectedArea(prismVerts(6, 1.0, 3.0),
                                      glm::dvec3(0.0, 0.0, 1.0));
    CHECK_TRUE(std::fabs(end - M_PI) < std::fabs(end6 - M_PI),
               "32-gon end-on closer to pi than the 6-gon");

    // Thin box (the wing: span 2 x, thickness 0.1 y, chord 2 z): face-on
    // (along the thin axis) = the planform (4); edge-on = thickness * length
    // (0.2). This is the 10x thin-wing ratio the drag model relies on.
    const std::vector<glm::dvec3> wing = boxVerts(1.0, 0.05, 1.0);
    CHECK_NEAR(projectedArea(wing, glm::dvec3(0.0, 1.0, 0.0)), 4.0, 1e-9,
               "wing face-on == planform (4)");
    CHECK_NEAR(projectedArea(wing, glm::dvec3(1.0, 0.0, 0.0)), 0.2, 1e-9,
               "wing edge-on (x) == thickness * chord (0.2)");
    CHECK_NEAR(projectedArea(wing, glm::dvec3(0.0, 0.0, 1.0)), 0.2, 1e-9,
               "wing edge-on (z) == thickness * span (0.2)");

    // A STACK of coaxial parts (a rocket: 3 boxes end-to-end along Z) is ONE
    // body: its silhouette prograde is a single end face (2x2=4), not 3 end
    // faces (the per-part sum would give 3x4=12 -- the stacking over-count
    // this model removes). Side-on the stack's length adds up (2 * 9 = 18),
    // which is correct for a long body seen from the side. This is why the
    // drag AREA is computed over the ship's combined vertices, not summed
    // per part (reports/projected-drag).
    {
        std::vector<glm::dvec3> stack;
        for(int i = -1; i <= 1; i++) {          // 3 boxes, each 3 long, along Z
            const double z0 = 3.0 * i;
            for(int k = 0; k < 8; k++) {
                stack.push_back(glm::dvec3((k & 1) ?  1.0 : -1.0,
                                           (k & 2) ?  1.0 : -1.0,
                                           z0 + ((k & 4) ? 1.5 : -1.5)));
            }
        }
        CHECK_NEAR(projectedArea(stack, glm::dvec3(0.0, 0.0, 1.0)), 4.0, 1e-9,
                   "stacked rocket: ONE end face prograde (4), not 3 (12)");
        CHECK_NEAR(projectedArea(stack, glm::dvec3(1.0, 0.0, 0.0)), 18.0, 1e-9,
                   "stacked rocket: full side length side-on (18)");
        // The prograde->side swing is the honest ~4.5x (18/4), not the ~1x a
        // per-part sum would give for a prograde stack.
        CHECK_TRUE(projectedArea(stack, glm::dvec3(1.0,0,0))
                   > 2.0 * projectedArea(stack, glm::dvec3(0,0,1)),
                   "stack: side-on drags >2x prograde (attitude matters)");
    }

    // Degenerate inputs -> 0.
    CHECK_NEAR(projectedArea(std::vector<glm::dvec3>(), glm::dvec3(0.0, 0.0, 1.0)),
               0.0, 0.0, "empty vertex list -> 0");
    CHECK_NEAR(projectedArea(cube, glm::dvec3(0.0)), 0.0, 0.0, "zero dir -> 0");
    CHECK_NEAR(projectedArea({glm::dvec3(0.0, 0.0, 0.0),
                              glm::dvec3(1.0, 0.0, 0.0)},
                             glm::dvec3(0.0, 0.0, 1.0)), 0.0, 0.0,
               "2 vertices -> 0");
    // Collinear 3 vertices -> a degenerate (line) hull -> 0.
    {
        std::vector<glm::dvec3> line;
        line.push_back(glm::dvec3(0.0, 0.0, 0.0));
        line.push_back(glm::dvec3(1.0, 0.0, 0.0));
        line.push_back(glm::dvec3(2.0, 0.0, 0.0));
        CHECK_NEAR(projectedArea(line, glm::dvec3(0.0, 1.0, 0.0)), 0.0, 0.0,
                   "collinear vertices -> 0");
    }
}

/* unionProjectedArea(polys): the ship's silhouette as the UNION of its parts'
   projected polygons (#463). Pinned against hand-computed areas, including the
   cases an overlap-subtracting approximation gets wrong (three plates meeting
   in one place) and the case the convex-hull balloon used to inflate (two
   plates with air between them). */
static std::vector<glm::dvec2> rect(double x0, double y0, double x1, double y1) {
    return { glm::dvec2(x0, y0), glm::dvec2(x1, y0),
             glm::dvec2(x1, y1), glm::dvec2(x0, y1) };
}

static void test_zeroTolerance() {
    printf("== the harness itself: what a zero expectation means (#510) ==\n");
    /* CHECK_NEAR is RELATIVE, and against an expectation of exactly zero a relative
       tolerance is void: 1e-12 x 0.0 is 0.0, so every "the tank is empty" and "the
       two lifts cancel" pin here was demanding bit-exactness while reading like
       slack -- and would break on any innocuous reassociation upstream. The macro
       now falls back to an absolute tolerance when the expectation is zero, which
       this case fails under the old definition and passes under the new one. */
    CHECK_NEAR(1e-13, 0.0, 1e-12, "a zero expectation takes the written tolerance"
                                  " as absolute slack");
    /* The other half of the promise: a NONZERO expectation is still relative, so a
       pin on a large number keeps its meaning (0.5 m of slack on a megametre, which
       an absolute 1e-6 would have refused). */
    CHECK_NEAR(1.0e6 + 0.5, 1.0e6, 1e-6, "a nonzero expectation stays relative");
}

static void test_unionArea() {
    printf("== unionProjectedArea: exact union of the parts' silhouettes ==\n");

    // One polygon: its own area. Two apart: the sum (the gap is air, not
    // airframe). One inside another: just the outer one.
    CHECK_NEAR(unionProjectedArea({ rect(-1.0, -1.0, 1.0, 1.0) }), 4.0, 1e-12,
               "one square == its area");
    CHECK_NEAR(unionProjectedArea({ rect(0.0, 0.0, 1.0, 1.0),
                                    rect(3.0, 0.0, 4.0, 1.0) }), 2.0, 1e-12,
               "two disjoint squares == the sum");
    CHECK_NEAR(unionProjectedArea({ rect(0.0, 0.0, 4.0, 4.0),
                                    rect(1.0, 1.0, 2.0, 2.0) }), 16.0, 1e-12,
               "a contained square adds nothing");
    CHECK_NEAR(unionProjectedArea({ rect(0.0, 0.0, 1.0, 1.0),
                                    rect(0.5, 0.0, 1.5, 1.0) }), 1.5, 1e-12,
               "half-overlap charges the overlap once");
    // Edge-to-edge contact shares a line, not an area.
    CHECK_NEAR(unionProjectedArea({ rect(0.0, 0.0, 1.0, 1.0),
                                    rect(1.0, 0.0, 2.0, 1.0) }), 2.0, 1e-12,
               "touching edge to edge == the sum");

    /* Three unit squares sliding over each other: A [-0.5,0.5], C [-0.25,0.75],
       B [0,1] along x, all 1 tall. Slab by slab: 0.25 + 0.25 + 0.5 + 0.25 +
       0.25 = 1.5. Inclusion-exclusion agrees (3 - 1.75 + 0.25), and that is the
       pin: subtracting the pairwise overlaps alone gives 1.25, so a fix that
       double-charged the triple overlap would fail here. */
    CHECK_NEAR(unionProjectedArea({ rect(-0.5, -0.5, 0.5, 0.5),
                                    rect(-0.25, -0.5, 0.75, 0.5),
                                    rect(0.0, -0.5, 1.0, 0.5) }), 1.5, 1e-12,
               "triple overlap counted once (not 1.25)");

    /* Two bars crossing in an X of sorts (a spar over a longeron): the eight
       edge crossings are the only breakpoints that make this slab-exact, so it
       pins the crossing collection. Areas: 0.4 + 0.4 - 0.04 = 0.76. */
    CHECK_NEAR(unionProjectedArea({ rect(-1.0, -0.1, 1.0, 0.1),
                                    rect(-0.1, -1.0, 0.1, 1.0) }), 0.76, 1e-12,
               "crossing bars == sum minus the crossing square");

    // A triangle against a square: the slab measure is linear in x only because
    // the triangle's edges are straight, which is what the midpoint rule leans on.
    // Triangle (0,0)(2,0)(0,2) == area 2; square [1,3]x[0,2] == 4; overlap =
    // the triangle's slice for x in [1,2] (height 2-x -> integral 0.5) -> 5.5.
    {
        std::vector<std::vector<glm::dvec2>> polys;
        polys.push_back({ glm::dvec2(0.0, 0.0), glm::dvec2(2.0, 0.0),
                          glm::dvec2(0.0, 2.0) });
        polys.push_back(rect(1.0, 0.0, 3.0, 2.0));
        CHECK_NEAR(unionProjectedArea(polys), 5.5, 1e-12,
                   "triangle + square == 5.5 (slanted edge, exact)");
    }

    // Degenerate inputs: no polygons, a line, a single point.
    CHECK_NEAR(unionProjectedArea({}), 0.0, 0.0, "no polygons -> 0");
    CHECK_NEAR(unionProjectedArea({ { glm::dvec2(0, 0), glm::dvec2(1, 0),
                                      glm::dvec2(2, 0) } }), 0.0, 1e-12,
               "collinear polygon -> 0");

    /* The bug this replaces: two plates with air between them. Convex-hulling
       their merged vertices draws a skin over the gap and doubles the area;
       the union charges both plates and nothing between them. Built as 3-D
       boxes so the comparison is the one the old code actually did. */
    {
        std::vector<glm::dvec3> merged;
        std::vector<std::vector<glm::dvec2>> polys;
        for(double cx : { 0.0, 3.0 }) {          // two 1 m cubes, 2 m apart
            for(int k = 0; k < 8; k++) {
                merged.push_back(glm::dvec3(cx + ((k & 1) ? 0.5 : -0.5),
                                            (k & 2) ? 0.5 : -0.5,
                                            (k & 4) ? 0.5 : -0.5));
            }
            polys.push_back(rect(cx - 0.5, -0.5, cx + 0.5, 0.5));
        }
        const double balloon = projectedArea(merged, glm::dvec3(0.0, 0.0, 1.0));
        const double real = unionProjectedArea(polys);
        CHECK_NEAR(real, 2.0, 1e-12, "two separated plates == 2 m2");
        CHECK_NEAR(balloon, 4.0, 1e-12, "the balloon over them == 4 m2");
        CHECK_TRUE(real < balloon, "the union is never the balloon");
    }

    // A stack (a rocket): parts overlap along the axis, so the union of their
    // frontal polygons is ONE face -- the same answer the combined-vertex hull
    // gave, which is why rockets barely move when this changes.
    {
        std::vector<std::vector<glm::dvec2>> stack;
        for(int k = 0; k < 3; k++) {
            stack.push_back(rect(-1.0, -1.0, 1.0, 1.0));  // same 2x2 face, 3 high
        }
        CHECK_NEAR(unionProjectedArea(stack), 4.0, 1e-12,
                   "a stack charges one face, not three");
    }
}

/* projectPartHull: the MOUNT belongs to the shape. A part's own area is
   translation-invariant, which is how the per-part law could ignore localPos for
   years; the moment two parts' polygons are unioned, ignoring it piles every
   part's silhouette on the origin and charges a spread ship one part's worth of
   air. This lives in drag.h rather than vehicle.cpp precisely so the ship's
   projection is the one being pinned. */

/* #512: cp's arm must be where the AREA is. The ship projects each part's hull
   into one shared flow plane; the centroid of that polygon is where the part's
   drag pushes, and its distance from the part's MOUNT is the error the law used
   to make. These pins are the parts of that answer: the centroid helper, the
   offset, and the two properties the call site leans on (a centred hull is
   exactly zero, and the offset does not care where the part is bolted). */
static void test_cp_silhouette() {
    printf("== cp: the arm is the silhouette, not the mount (#512) ==\n");

    // The centroid helper, on shapes whose centroid is known by hand.
    {
        glm::dvec2 c(99.0, 99.0);
        polygonCentroid2(rect(-1.0, -1.0, 1.0, 1.0), c);
        CHECK_TRUE(std::fabs(c.x) < 1e-15 && std::fabs(c.y) < 1e-15,
                   "a square's centroid is its centre");
        // Winding must not matter (polygonArea2 is symmetric in winding).
        std::vector<glm::dvec2> cw = rect(0.0, 0.0, 2.0, 4.0);
        std::reverse(cw.begin(), cw.end());
        polygonCentroid2(cw, c);
        CHECK_NEAR(c.x, 1.0, 1e-15, "a 2x4 bar's centroid x, clockwise ring");
        CHECK_NEAR(c.y, 2.0, 1e-15, "a 2x4 bar's centroid y, clockwise ring");
        // A triangle: the vertex mean, which no vertex-average-of-a-quad is.
        polygonCentroid2({ glm::dvec2(0, 0), glm::dvec2(3, 0), glm::dvec2(0, 3) }, c);
        CHECK_NEAR(c.x, 1.0, 1e-15, "a triangle's centroid is the vertex mean (x)");
        CHECK_NEAR(c.y, 1.0, 1e-15, "a triangle's centroid is the vertex mean (y)");
        /* An ASYMMETRIC quad, where the two answers part company. A trapezoid (0,0)
           (4,0) (1,1) (0,1): shoelace area 2.5, centroid (1.4, 0.4); the vertex mean
           is (1.25, 0.5). Without this pin a helper that averages vertices passes
           every shape above (square, bar, triangle) and would pass a symmetric
           silhouette sweep too. */
        polygonCentroid2({ glm::dvec2(0, 0), glm::dvec2(4, 0), glm::dvec2(1, 1),
                           glm::dvec2(0, 1) }, c);
        CHECK_NEAR(c.x, 1.4, 1e-12, "a trapezoid's centroid is the AREA centroid (x)");
        CHECK_NEAR(c.y, 0.4, 1e-12, "a trapezoid's centroid is the AREA centroid (y)");
        CHECK_TRUE(std::fabs(c.x - 1.25) > 0.1 || std::fabs(c.y - 0.5) > 0.1,
                   "which is not its vertex mean");
        // Degenerate rings leave the caller's value alone, so the call site can
        // seed it with the mount and get a zero offset for free.
        c = glm::dvec2(7.0, -3.0);
        polygonCentroid2({ glm::dvec2(0, 0), glm::dvec2(1, 0), glm::dvec2(2, 0) }, c);
        CHECK_TRUE(c.x == 7.0 && c.y == -3.0, "a collinear ring leaves the seed alone");
        polygonCentroid2({ glm::dvec2(0, 0), glm::dvec2(1, 0) }, c);
        CHECK_TRUE(c.x == 7.0 && c.y == -3.0, "and so does a two-point ring");
        /* Pin the OFFSET helper on the same input, not just the helper it calls: a
           mutant that seeds the centroid at the plane origin instead of the mount
           turns a plate-on-edge (zero area) into an instant 4 m cp yank, and only
           this call says so. */
        const glm::dvec2 zoff = silhouetteCpOffset2(
            { glm::dvec2(0, 0), glm::dvec2(1, 0), glm::dvec2(2, 0) },
            glm::dvec2(1.5, 0.0));
        CHECK_TRUE(zoff.x == 0.0 && zoff.y == 0.0,
                   "a degenerate silhouette does not move the cp at all");
    }

    /* A cone seen broadside: the silhouette is a triangle whose centroid sits a
       third of the way from apex to base, so a part whose hull is a cone (a
       capsule, a nose cone, an engine bell) has its whole drag well off the bolt
       that holds it. The pinned hull below is 2 m long (apex z=+1, base z=-1) and
       its mount is at its middle, so the offset is 1/3 m -- a sixth of the cone's
       length, not a third of it; the shipped capsules are a metre or two across,
       which is where the 0.2-0.4 m audit figures come from. Worked out by hand:
       hull apex at z=+1,
       base ring r=1 at z=-1, flow along +x. flowPlane(+x) gives u=+z, w=-y, so
       the projected triangle is (1,0) (-1,-1) (-1,+1) with centroid (-1/3, 0). */
    {
        std::vector<glm::dvec3> cone;
        cone.push_back(glm::dvec3(0.0, 0.0, 1.0));
        for(int i = 0; i < 16; i++) {
            const double th = 2.0 * M_PI * i / 16.0;
            cone.push_back(glm::dvec3(std::cos(th), std::sin(th), -1.0));
        }
        const glm::dvec3 flow(1.0, 0.0, 0.0);
        const FlowPlane pl = flowPlane(flow);
        CHECK_TRUE(pl.valid, "a broadside flow has a plane");
        CHECK_NEAR(glm::dot(pl.u, glm::dvec3(0, 0, 1)), 1.0, 1e-12, "its u runs with +z");
        CHECK_NEAR(glm::dot(pl.w, glm::dvec3(0, -1, 0)), 1.0, 1e-12, "and its w with -y");
        std::vector<glm::dvec2> poly;
        projectPartHull(cone, glm::dmat3(1.0), glm::dvec3(0.0), pl, poly);
        CHECK_TRUE(polygonArea2(poly) > 0.0, "the cone does project to an area");
        const glm::dvec2 off = silhouetteCpOffset2(poly, glm::dvec2(0.0));
        CHECK_NEAR(off.x, -1.0 / 3.0, 1e-12, "a cone's area sits 1/3 of its length"
                                           " toward the base, not at the mount");
        CHECK_TRUE(std::fabs(off.y) < 1e-15,
                   "and does not lean across for a symmetric cone");
        const glm::dvec3 off3 = off.x * pl.u + off.y * pl.w;
        CHECK_TRUE(std::fabs(glm::dot(off3, flow)) < 1e-15,
                   "the offset lies IN the flow plane: a projection cannot say where"
                   " along its own normal the area sits, so the cp keeps the mount there");

        /* Bolt the same cone 5 m along the flow and 3 m up: the offset is a free
           vector, so it must come back identical. (Getting this wrong is what makes
           a per-part centroid tempting to compute against the wrong origin.) */
        std::vector<glm::dvec2> poly2;
        projectPartHull(cone, glm::dmat3(1.0), glm::dvec3(0.0, 3.0, 5.0), pl, poly2);
        const glm::dvec2 m2{glm::dot(glm::dvec3(0.0, 3.0, 5.0), pl.u),
                            glm::dot(glm::dvec3(0.0, 3.0, 5.0), pl.w)};
        const glm::dvec2 off2 = silhouetteCpOffset2(poly2, m2);
        CHECK_NEAR(off2.x, off.x, 1e-12, "the offset does not depend on where the part is");
        CHECK_NEAR(off2.y, off.y, 1e-12, "the offset does not depend on where the part is (y)");
    }

    /* A hull centred on its mount is EXACTLY unchanged by this fix, whichever way
       the flow comes from -- that is why the shipped beds barely move (their wings,
       rudders and tanks are centred), and it is the property that keeps the change
       from being a re-tune. Counted over a direction sweep so a mutant that shifts
       every part a little would fail. */
    {
        std::vector<glm::dvec3> box;
        for(int i = 0; i < 8; i++) {
            box.push_back(glm::dvec3((i & 1) ? 0.6 : -0.6, (i & 2) ? 2.0 : -2.0,
                                     (i & 4) ? 1.0 : -1.0));
        }
        int dirs = 0, nonzero = 0;
        for(int az = 0; az <= 178; az += 2) {
            for(int el = -60; el <= 60; el += 15) {
                const double a = glm::radians((double)az), e = glm::radians((double)el);
                const glm::dvec3 v = glm::dvec3(std::sin(a) * std::cos(e), std::sin(e),
                                                std::cos(a) * std::cos(e));
                const FlowPlane pl = flowPlane(v);
                std::vector<glm::dvec2> poly;
                projectPartHull(box, glm::dmat3(1.0), glm::dvec3(0.0), pl, poly);
                if(polygonArea2(poly) <= 0.0) { continue; }
                ++dirs;
                const glm::dvec2 off = silhouetteCpOffset2(poly, glm::dvec2(0.0));
                /* Not bit-exact: the plane basis carries irrational components, so a
                   symmetric hull's two halves cancel to ~1e-16 rather than to zero.
                   1e-9 on a hull a metre across is eight orders below the error this
                   fix is about. */
                if(std::fabs(off.x) > 1e-9 || std::fabs(off.y) > 1e-9) { ++nonzero; }
            }
        }
        CHECK_TRUE(dirs > 300, "the sweep really sampled the sphere");
        CHECK_TRUE(nonzero == 0, "a hull centred on its mount moves the cp not at all,"
                                 " in any of those directions");
    }

    /* And the shape of the bug a builder actually hits: a plate authored at its
       ROOT (the hull running outboard from the bolt, z 0..8), so its area sits 4 m
       from the mount. Broadside flow, so the whole span is fore-and-aft. */
    {
        std::vector<glm::dvec3> plate;
        for(int i = 0; i < 8; i++) {
            plate.push_back(glm::dvec3((i & 1) ? 0.05 : -0.05, (i & 2) ? 1.0 : -1.0,
                                       (i & 4) ? 8.0 : 0.0));
        }
        const glm::dvec3 flow(1.0, 0.0, 0.0);
        const FlowPlane pl = flowPlane(flow);
        std::vector<glm::dvec2> poly;
        projectPartHull(plate, glm::dmat3(1.0), glm::dvec3(0.0), pl, poly);
        const glm::dvec2 off = silhouetteCpOffset2(poly, glm::dvec2(0.0));
        CHECK_NEAR(glm::length(off), 4.0, 1e-12,
                   "a plate authored at its root pushes 4 m from its mount");
        CHECK_NEAR(glm::dot(off.x * pl.u + off.y * pl.w, glm::dvec3(0, 0, 1)), 4.0, 1e-12,
                   "and for a broadside flow that is 4 m of weathervane arm, out toward"
                   " where the plate actually is (z 0..8, bolt at z 0)");
    }

    /* The frame chain the call site performs (applyAeroForce), pinned against an
       independent route to the same point. Above, everything is identity-aligned,
       so a wrong sRot or a dropped transpose is invisible here -- that is exactly
       how `off.x * pl.u + off.y * pl.w` (ship-local basis, no sRot) would slip
       through, and it would put cp metres off on a yawed ship.
       Code route: silhouetteCpPointS (what applyAeroForce calls) composes the
       offset with the SHIP-LOCAL plane basis and sRot.
       Reference route: rotate the hull VERTS into the ship frame (localRot*h +
       localPos), project those onto plS, take the area centroid -- where the area
       actually is, with no basis-rotation trick involved.
       The contract: the two agree exactly in the plane and differ only along the
       flow, by the mount's own along-flow coordinate. Compared in the ship frame,
       because plS is a ship-local basis and mixing frames in a dot product is how
       this bug would have been written in the first place. */
    {
        const glm::dmat3 sRot = glm::mat3_cast(glm::angleAxis(0.7, glm::dvec3(0, 1, 0))
                                             * glm::angleAxis(0.4, glm::dvec3(0, 0, 1)));
        const glm::dvec3 sPos(11.0, -3.0, 6.0);
        const glm::dmat3 localRot = glm::mat3_cast(glm::angleAxis(0.9, glm::dvec3(1, 0, 0)));
        const glm::dvec3 localPos(0.4, 1.2, -2.0);
        const glm::dvec3 vhatS = glm::normalize(glm::dvec3(0.3, -0.2, -0.93));
        const FlowPlane plS = flowPlane(vhatS);
        std::vector<glm::dvec3> hull;
        for(int i = 0; i < 8; i++) {
            hull.push_back(glm::dvec3((i & 1) ? 0.3 : -0.3, (i & 2) ? 0.5 : -0.5,
                                      (i & 4) ? 2.0 : 0.0));
        }
        std::vector<glm::dvec2> poly;
        projectPartHull(hull, localRot, localPos, plS, poly);
        CHECK_TRUE(polygonArea2(poly) > 0.0, "the tilted part projects to an area");
        const glm::dvec2 off = silhouetteCpOffset2(poly, glm::dvec2(
            glm::dot(localPos, plS.u), glm::dot(localPos, plS.w)));
        const glm::dvec3 cp = silhouetteCpPointS(sRot, sPos + sRot * localPos, off, plS);

        // The same verts, carried into the ship frame and projected there.
        std::vector<glm::dvec3> vertsShip;
        for(const glm::dvec3 &h : hull) { vertsShip.push_back(localRot * h + localPos); }
        std::vector<glm::dvec2> polyV;
        projectHull(vertsShip, plS, polyV);
        CHECK_NEAR(polygonArea2(polyV), polygonArea2(poly), 1e-12,
                   "both routes see the same silhouette area, so both see the same polygon");
        glm::dvec2 cV(0.0);
        polygonCentroid2(polyV, cV);
        const glm::dvec3 ref = cV.x * plS.u + cV.y * plS.w;   // ship frame, through its origin
        // Back to the ship frame -- the sim's own frame-change law, position first
        // (plS is a ship-local basis, so every dot below has to be asked there). A
        // correct composition leaves only the along-flow coordinate; a wrong sRot
        // or a basis mixed across frames leaves metres of this.
        const glm::dvec3 d = glm::transpose(sRot) * (cp - sPos) - ref;
        /* Absolute tolerances: CHECK_NEAR is RELATIVE (#510), and these expectations
           are zero, so the macro would demand bit-exactness. The residuals below are
           the rounding of a rotated basis, ~1e-16 on a hull a few metres across. */
        CHECK_TRUE(std::fabs(glm::dot(d, plS.u)) < 1e-9,
                   "the composed cp lands on the silhouette centroid IN THE PLANE, with a"
                   " rotated ship, a rotated part and an off-centre hull (u)");
        CHECK_TRUE(std::fabs(glm::dot(d, plS.w)) < 1e-9,
                   "(and across the plane) -- sRot, localRot and the local origin all line up");
        CHECK_NEAR(glm::dot(d, vhatS), glm::dot(localPos, vhatS), 1e-9,
                   "and the only leftover is along the flow, which is the mount's own"
                   " coordinate there -- the part a projection cannot see");
    }
}

static void test_partMounts() {
    printf("== projectPartHull: a part's mount position is part of its silhouette ==\n");

    // A 1 m cube's collision verts, as a part holds them (hull-local).
    std::vector<glm::dvec3> box;
    for(int k = 0; k < 8; k++) {
        box.push_back(glm::dvec3((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5,
                                 (k & 4) ? 0.5 : -0.5));
    }
    const glm::mat3 id(1.0);
    const FlowPlane pl = flowPlane(glm::dvec3(0.0, 0.0, 1.0));  // looking down the z axis

    std::vector<std::vector<glm::dvec2>> apart, piled;
    for(const glm::dvec3 &pos : { glm::dvec3(0.0), glm::dvec3(10.0, 0.0, 0.0) }) {
        apart.emplace_back();
        projectPartHull(box, id, pos, pl, apart.back());
        piled.emplace_back();
        projectPartHull(box, id, glm::dvec3(0.0), pl, piled.back());
    }
    CHECK_NEAR(unionProjectedArea(apart), 2.0, 1e-12,
               "two mounts 10 m apart == the sum (the union sees the gap)");
    CHECK_NEAR(unionProjectedArea(piled), 1.0, 1e-12,
               "the same mount twice charges one face");
    // What let the old law get away with it: a mount moves a polygon, it never
    // resizes it.
    CHECK_NEAR(polygonArea2(apart[1]), 1.0, 1e-12, "a mount does not resize a part");

    /* The offset is read in the SHARED basis, not the part's: a part spun on its
       mount still sits where it is mounted. Built without gtc so this test stays
       pure math: a 60 deg spin about the flow axis. */
    const double ca = std::cos(1.0471975516), sa = std::sin(1.0471975516);
    const glm::mat3 spin(ca, sa, 0.0, -sa, ca, 0.0, 0.0, 0.0, 1.0);
    const glm::dvec3 mount(4.0, 0.0, 1.5);
    std::vector<glm::dvec2> upright, turned;
    projectPartHull(box, id, mount, pl, upright);
    projectPartHull(box, spin, mount, pl, turned);
    const auto centre = [](const std::vector<glm::dvec2> &poly) {
        glm::dvec2 c(0.0);
        for(const glm::dvec2 &q : poly) { c += q; }
        return (poly.empty() ? c : c / (double)poly.size());
    };
    // Stated against the plane's own basis, which is what "shared" means here.
    const glm::dvec2 want{glm::dot(mount, pl.u), glm::dot(mount, pl.w)};
    CHECK_TRUE(glm::length(want) > 1.0, "the mount is off the flow axis (else this proves nothing)");
    CHECK_NEAR(centre(upright).x, want.x, 1e-12, "the part lands where it is mounted");
    CHECK_NEAR(centre(upright).y, want.y, 1e-12, "measured in the ship's basis, not the part's");
    CHECK_NEAR(centre(turned).x, want.x, 1e-12, "a spun mount stays put");
    CHECK_NEAR(centre(turned).y, want.y, 1e-12, "and keeps its place");

    /* The undercharge this fixes, at ship scale: parts stacked ALONG the view all
       project the same face, so the union is one face either way (which is why
       rockets barely moved when #463 landed) -- spread ACROSS the view they are
       the whole ship, and a law that drops the mount still sees one face. */
    std::vector<std::vector<glm::dvec2>> stack, spread;
    for(int k = 0; k < 3; k++) {
        stack.emplace_back();
        projectPartHull(box, id, glm::dvec3(0.0, 0.0, k * 2.0), pl, stack.back());
        spread.emplace_back();
        projectPartHull(box, id, glm::dvec3(k * 2.0, 0.0, 0.0), pl, spread.back());
    }
    CHECK_NEAR(unionProjectedArea(stack), 1.0, 1e-12, "a stack along the view == one face");
    CHECK_NEAR(unionProjectedArea(spread), 3.0, 1e-12, "the same parts across the view == three");

    /* Which way the mount TURNS the part is unpinned by everything above: a cube is
       symmetric enough that transposing the rotation changes nothing measurable. So
       pin the identity the implementation stands on -- projecting an offset part
       equals projecting that part's vertices after they have been turned and moved
       -- with an ASYMMETRIC part on an OFF-AXIS mount, and then show the pin bites
       by asking for the transposed mount. Hand-built rotations keep this file
       linking nothing but glm and drag.h. */
    const std::vector<glm::dvec3> wedge{glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(2.0, 0.0, 0.0),
                                        glm::dvec3(0.0, 1.0, 0.0)};
    const double cs = std::cos(1.0), sn = std::sin(1.0);
    const glm::mat3 turnX(1.0, 0.0, 0.0, 0.0, cs, sn, 0.0, -sn, cs);
    const glm::mat3 turnY(cs, 0.0, -sn, 0.0, 1.0, 0.0, sn, 0.0, cs);
    const glm::mat3 turn = turnY * turnX;
    const glm::dvec3 off(3.0, -2.0, 1.0);
    const FlowPlane side = flowPlane(glm::normalize(glm::dvec3(-0.6, 0.7, 0.3)));
    CHECK_TRUE(std::fabs(glm::length(turn[0]) - 1.0) < 1e-6 &&
                   std::fabs(glm::length(turn[1]) - 1.0) < 1e-6 &&
                   std::fabs(glm::dot(turn[0], turn[1])) < 1e-6,
               "the test's turn is a rotation (else the pin means nothing)");
    std::vector<glm::dvec2> fast, brute, backwards;
    projectPartHull(wedge, turn, off, side, fast);
    std::vector<glm::dvec3> world;
    for(const glm::dvec3 &v : wedge) {
        world.push_back(glm::dvec3(turn * v) + off);
    }
    projectHull(world, side, brute);
    /* Both sides of this identity go through the same float-valued mount matrix but
       combine it differently, so they agree to ~2e-8, not to a ulp. The law itself
       is good to a few percent; a mount is not where the precision lives. */
    CHECK_NEAR(polygonArea2(fast), polygonArea2(brute), 1e-6,
               "an offset part projects like its turned, moved vertices");
    CHECK_TRUE(glm::length(centre(fast) - centre(brute)) < 1e-6,
               "and lands on the same polygon, not just the same area");
    projectPartHull(wedge, glm::transpose(turn), off, side, backwards);
    CHECK_TRUE(glm::length(centre(backwards) - centre(brute)) > 1e-3,
               "a transposed mount WOULD disagree -- so the pin above bites");
}

static void test_silhouetteMemo() {
    printf("== SilhouetteMemo: lazy, generation-stamped direction table ==\n");
    SilhouetteMemo memo;
    double out = -1.0;
    // Before reset() there is no table at all: every query is a miss (a ship
    // that never flies never allocates one).
    CHECK_TRUE(memo.empty(), "no table before reset");
    CHECK_TRUE(!memo.lookup(0.3, 0.1, out), "no table -> miss");
    memo.reset();
    CHECK_TRUE(!memo.empty(), "reset allocates the table");
    CHECK_TRUE(!memo.lookup(0.3, 0.1, out), "fresh table: nothing filled -> miss");
    CHECK_TRUE(memo.fillAround(0.3, 0.1, 0, [](double, double) { return 1.0; }) == 0,
               "a zero budget fills nothing (and costs nothing)");

    /* A function LINEAR in both angles: bilinear interpolation reproduces it
       exactly, so any disagreement is the memo's error, not the surface's. */
    const auto exact = [](double a, double b) { return 1.0 + 0.5 * a + 0.25 * b; };
    int calls = 0;
    const auto counted = [&](double a, double b) { calls++; return exact(a, b); };
    CHECK_TRUE(memo.fillAround(0.11, -0.31, 4, counted) == 4,
               "a fresh cell fills its four corners");
    CHECK_TRUE(calls == 4, "each node computed once");
    CHECK_TRUE(memo.lookup(0.11, -0.31, out), "a full cell hits");
    CHECK_NEAR(out, exact(0.11, -0.31), 1e-6, "bilinear is exact on a linear surface");
    // Second query in the same cell: no recomputation.
    calls = 0;
    CHECK_TRUE(memo.lookup(0.13, -0.29, out), "still a hit");
    CHECK_TRUE(calls == 0, "a hit computes nothing");
    CHECK_NEAR(out, exact(0.13, -0.29), 1e-6, "hit reads the cached surface");

    /* Lazy allocation: a geometry change bumps a counter, it does not buy a table.
       The 961-node table (7.7 kB) appears the first time the ship actually asks
       for a direction, so the ships that never fly -- parked, in orbit, the ones
       the player never looks at -- carry nothing. */
    memo.clear();
    CHECK_TRUE(memo.empty(), "clear frees the table");
    memo.invalidate();
    CHECK_TRUE(memo.empty(), "invalidate alone allocates nothing");
    CHECK_TRUE(!memo.lookup(0.2, 0.2, out), "and a query there is a miss");
    calls = 0;
    CHECK_TRUE(memo.fillAround(0.2, 0.2, 1, counted) == 1, "the first fill makes the table");
    CHECK_TRUE(!memo.empty(), "and it is there from then on");
    memo.invalidate();
    CHECK_TRUE(!memo.empty(), "a generation bump keeps the buffer");
    CHECK_TRUE(!memo.lookup(0.2, 0.2, out), "but the old node no longer answers");

    // The budget is a per-call cap, and it spreads the fill over substeps.
    memo.reset();
    calls = 0;
    CHECK_TRUE(memo.fillAround(0.5, 0.5, 1, counted) == 1, "budget 1 fills one node");
    CHECK_TRUE(calls == 1, "and computes exactly one");
    CHECK_TRUE(!memo.lookup(0.5, 0.5, out), "three corners missing -> still a miss");
    for(int i = 0; i < 3; i++) { memo.fillAround(0.5, 0.5, 1, counted); }
    CHECK_TRUE(memo.lookup(0.5, 0.5, out), "the cell completes over four substeps");
    CHECK_TRUE(calls == 4, "and took exactly four computations");

    // Invalidating by generation: nothing is cleared, every node just reads as
    // unfilled again -- which is what a geometry change (staging, docking) pays.
    memo.reset();
    calls = 0;
    // A query near the chart's corner, where the cell index is known: (0, 0).
    const double corner = -SilhouetteMemo::kHalf + 0.05;
    CHECK_TRUE(memo.fillAround(corner, corner, 4, counted) == 4,
               "the corner cell fills");
    const float saved = memo.area[0];
    CHECK_TRUE(saved != 0.0f, "and its (0,0) node holds a number");
    memo.invalidate();
    CHECK_TRUE(!memo.lookup(corner, corner, out),
               "a generation bump invalidates every node at once");
    CHECK_TRUE(memo.area[0] == saved, "invalidate clears nothing (no sweep on a burn)");
    CHECK_TRUE(memo.fillAround(corner, corner, 4, counted) == 4, "and the cell refills");

    // A NaN angle (a NaN axis cannot be refused by AeroFrame::valid) is a miss,
    // not a table full of NaN.
    CHECK_TRUE(!memo.lookup(std::nan(""), 0.0, out), "NaN alpha -> miss");
    CHECK_TRUE(memo.fillAround(0.0, std::nan(""), 4, counted) == 0, "NaN beta fills nothing");

    /* The back hemisphere. aeroFrame's two angles share atan2's denominator,
       dot(v, nose), so a flow from BEHIND the nose reads past +-90 deg in BOTH at
       once -- which is why a fold test written with a hand-fed pair like
       (3.0, 0.2) proves nothing: no ship can fly that pair. The fold reverses the
       flow, wrapping each angle by 180 under its own sign; afterwards the pair
       must still describe the SAME flow line. */
    const glm::dvec3 right(1, 0, 0), up(0, 1, 0), nose(0, 0, 1);
    int onChart = 0, offChart = 0, lineBroken = 0, rim = 0;
    double worstLine = 0.0;
    for(int ix = -4; ix <= 4; ix++) {
        for(int iy = -4; iy <= 4; iy++) {
            for(int iz = -4; iz <= 4; iz++) {
                if(ix == 0 && iy == 0 && iz == 0) { continue; }
                const glm::dvec3 v = glm::normalize(
                    glm::dvec3((double)ix, (double)iy, (double)iz));
                const AeroFrame f = aeroFrame(v, right, up, nose);
                double a = f.alpha, b = f.beta;
                SilhouetteMemo::fold(a, b);
                if(!SilhouetteMemo::charted(a, b)) {
                    offChart++;
                    // Broadside to the nose: both angles sit at exactly +-90, where
                    // flowDirection has nothing to work with (see below).
                    if(std::fabs(f.alpha) > 1.5 && std::fabs(f.beta) > 1.5) { rim++; }
                    continue;
                }
                const glm::dvec3 d = flowDirection(a, b, right, up, nose);
                const double along = std::fabs(glm::dot(d, v));
                if(std::fabs(along - 1.0) > worstLine) { worstLine = std::fabs(along - 1.0); }
                if(along < 0.999) { lineBroken++; }
                onChart++;
            }
        }
    }
    CHECK_TRUE(lineBroken == 0, "the fold keeps the flow line for every cached direction");
    CHECK_TRUE(worstLine < 1e-9, "to machine precision");
    printf("   chart: %d of %d folded directions cached, %d left exact (%d broadside rim)\n",
           onChart, onChart + offChart, offChart, rim);
    CHECK_TRUE(onChart + offChart == 728, "the grid is 9x9x9 minus the origin, all classified");
    CHECK_TRUE(onChart > 100 && offChart > 10,
               "the chart covers most of the sphere but deliberately not all");
    /* The rim itself is worth naming: a flow exactly broadside reads +-90 deg in
       BOTH angles, and flowDirection builds the direction from tan(alpha) and
       tan(beta), both infinite there -- the RATIO of the two is what says which
       broadside direction, so the corner cannot encode it. That is the second
       reason the chart stops at +-75: past there the parameterisation is not just
       coarse, at the limit it is ambiguous. */
    CHECK_TRUE(rim > 0, "the grid contains rim directions");
    {
        double a = 1.5707963267948966, b = 1.5707963267948966;
        SilhouetteMemo::fold(a, b);
        CHECK_TRUE(!SilhouetteMemo::charted(a, b), "the rim is off the chart");
    }

    // One of them, spelled out: straight astern with a little up -- the mid-row of
    // a tumble, and the row the old fold keyed wrong.
    const glm::dvec3 astern = glm::normalize(-nose + up * 0.3);
    const AeroFrame fb = aeroFrame(astern, right, up, nose);
    CHECK_TRUE(std::fabs(fb.alpha) > SilhouetteMemo::kFoldHalf, "from behind: alpha past 90 deg");
    CHECK_TRUE(std::fabs(fb.beta) > SilhouetteMemo::kFoldHalf,
               "and beta is past 90 too (same atan2 denominator)");
    memo.reset();
    calls = 0;
    memo.fillAround(fb.alpha, fb.beta, 4, counted);
    CHECK_TRUE(memo.lookup(fb.alpha, fb.beta, out), "a back-hemisphere direction caches, then hits");
    CHECK_TRUE(calls == 4, "and its four nodes were computed once");
    double ra = fb.alpha, rb = fb.beta;
    SilhouetteMemo::fold(ra, rb);
    CHECK_TRUE(std::fabs(rb) < 1e-12, "astern with up folds to ahead with up, at zero yaw");
    CHECK_NEAR(out, exact(ra, rb), 1e-6, "the nodes answer the reversed flow");
    /* The bug this pins: wrapping alpha alone left beta clamped at +-90, so every
       row past 90 deg read the area of a quartering flow instead of its own. */
    CHECK_TRUE(std::fabs(exact(ra, SilhouetteMemo::kFoldHalf) - out) > 1e-3,
               "beta clamped instead of wrapped is a different answer");
    /* Spelled out as a direction rather than an area, so the pin cannot be satisfied
       by an empty cell: this is the key the old fold used, and the flow it names is
       not the flow being asked about. */
    const double oldA = fb.alpha > 0.0 ? fb.alpha - std::numbers::pi
                                       : fb.alpha + std::numbers::pi;
    const double oldB = fb.beta > 0.0 ? SilhouetteMemo::kFoldHalf : -SilhouetteMemo::kFoldHalf;
    const glm::dvec3 keyedByTheOldFold = flowDirection(oldA, oldB, right, up, nose);
    CHECK_TRUE(std::fabs(glm::dot(keyedByTheOldFold, astern)) < 0.9,
               "the old fold keyed a broadside direction to a flow that is nearly astern");

    /* Off the chart: past +-75 deg the tan() spacing spans an enormous slice of
       direction space, so the memo refuses and the caller's exact union -- which
       it can compute anyway -- answers instead. */
    CHECK_TRUE(!SilhouetteMemo::charted(1.45, 0.0), "83 deg alpha is off the chart");
    CHECK_TRUE(memo.fillAround(1.45, 0.0, 4, counted) == 0, "so it fills nothing");
    CHECK_TRUE(!memo.lookup(1.45, 0.0, out), "and every query there is a miss");
    CHECK_TRUE(memo.fillAround(1.30, 1.30, 4, counted) == 4,
               "inside the range it fills, even in a far corner");
    CHECK_TRUE(memo.lookup(1.30, 1.30, out), "and hits");
    CHECK_NEAR(out, exact(1.30, 1.30), 1e-6, "with no blend error on a linear surface");
}

static void test_flowDirection() {
    printf("== flowDirection: the inverse of AeroFrame's two angles ==\n");

    const glm::dvec3 right(1, 0, 0), up(0, 1, 0), nose(0, 0, 1);
    for(double a = -1.4; a <= 1.4; a += 0.35) {
        for(double b = -1.2; b <= 1.2; b += 0.4) {
            const glm::dvec3 v = flowDirection(a, b, right, up, nose);
            const AeroFrame f = aeroFrame(v, right, up, nose);
            CHECK_NEAR(f.alpha, a, 1e-9, "aeroFrame(flowDirection(a,b)) round-trips alpha");
            CHECK_NEAR(f.beta, b, 1e-9, "and beta");
        }
    }
    // The ship may hold its axes unnormalized only by luck; a zero nose refuses.
    CHECK_NEAR(glm::length(flowDirection(0.0, 0.0, right, up, glm::dvec3(0.0))), 0.0, 0.0,
               "degenerate frame -> zero direction");
}

static void test_inducedCd() {
    printf("== inducedCd: the price of lift (#462) ==\n");

    /* CL=1 on a span-4, area-8 wing: AR = 16/8 = 2, so CDi = 1/(pi e AR) =
       1/(pi * 0.8 * 2). Hand-checked against the textbook formula. */
    CHECK_NEAR(inducedCd(1.0, 4.0, 8.0), 1.0 / (std::numbers::pi * 0.8 * 2.0),
               1e-12, "CL=1, AR=2 == 1/(pi e AR)");
    // Grows with the SQUARE of CL: twice the lift costs four times the drag.
    CHECK_NEAR(inducedCd(2.0, 4.0, 8.0), 4.0 * inducedCd(1.0, 4.0, 8.0), 1e-12,
               "quadratic in CL");
    // A longer, narrower wing of the same area lifts cheaper: AR goes up.
    CHECK_TRUE(inducedCd(1.0, 8.0, 8.0) < inducedCd(1.0, 4.0, 8.0),
               "a higher aspect ratio costs less induced drag");
    // No lift, no lifting surface, no span -> nothing to pay.
    CHECK_NEAR(inducedCd(0.0, 4.0, 8.0), 0.0, 0.0, "CL=0 -> 0");
    CHECK_NEAR(inducedCd(1.0, 4.0, 0.0), 0.0, 0.0, "no planform -> 0");
    CHECK_NEAR(inducedCd(1.0, 0.0, 8.0), 0.0, 0.0, "no span -> 0");
    CHECK_NEAR(inducedCd(1.0, 4.0, 8.0, 0.0), 0.0, 0.0, "a zero Oswald efficiency -> 0");
}

static void test_partCd() {
    printf("== partCd: 3-anchor directional blend (nose / side / base) ==\n");

    // A cone-like part (the capsule): sleek nose (0.35), blunt side (0.8),
    // blunt base (1.3) -- the flat heat shield does the re-entry braking.
    const double fwd = 0.35, side = 0.80, bwd = 1.30;

    // Each anchor is hit EXACTLY at its angle (c = cos of nose-axis vs flow):
    //   c = +1  -> nose into the flow  -> drag_forward
    //   c =  0  -> broadside           -> drag_side
    //   c = -1  -> base into the flow  -> drag_backward
    CHECK_NEAR(partCd(fwd, side, bwd,  1.0), fwd,  1e-12, "c=+1 -> forward anchor");
    CHECK_NEAR(partCd(fwd, side, bwd,  0.0), side, 1e-12, "c=0 -> side anchor");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.0), bwd,  1e-12, "c=-1 -> backward anchor");

    // CONVEX blend: the weights (side(1-c^2), fwd(max(c,0)^2), bwd(max(-c,0)^2))
    // are non-negative and sum to 1, so cd stays within the anchor range for
    // every angle -- no overshoot, no dip below the sleekest anchor.
    const double mn = 0.35, mx = 1.30;
    for(double c = -1.0; c <= 1.0 + 1e-9; c += 0.05) {
        const double cd = partCd(fwd, side, bwd, c);
        CHECK_TRUE(cd >= mn - 1e-12 && cd <= mx + 1e-12,
                   "cd in [min,max] anchor range (sweep)");
    }

    // The weights sum to 1 (a true convex combination, not scaled/offset) --
    // pin the exact value at two 45 deg angles:
    //   c = +1/sqrt2 (nose-in):  side*0.5 + fwd*0.5 + bwd*0
    //   c = -1/sqrt2 (base-in):  side*0.5 + fwd*0 + bwd*0.5
    CHECK_NEAR(partCd(fwd, side, bwd,  1.0 / std::sqrt(2.0)),
               0.5 * side + 0.5 * fwd, 1e-12, "c=+1/sqrt2 -> .5 side + .5 fwd");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.0 / std::sqrt(2.0)),
               0.5 * side + 0.5 * bwd, 1e-12, "c=-1/sqrt2 -> .5 side + .5 bwd");

    // With fwd < side < bwd the blend eases nose->base and is monotone up as
    // the part turns from nose-first to base-first (the weathervane asymmetry).
    CHECK_TRUE(partCd(fwd, side, bwd,  1.0) < partCd(fwd, side, bwd, 0.0),
               "forward < side (sleeker nose than broadside)");
    CHECK_TRUE(partCd(fwd, side, bwd, 0.0) < partCd(fwd, side, bwd, -1.0),
               "side < backward (broadside blunter than the base)");
    CHECK_TRUE(partCd(fwd, side, bwd, 0.5) < partCd(fwd, side, bwd, -0.5),
               "nose-in cd < base-in cd (fwd < bwd)");

    // A SYMMETRIC part (all three anchors equal -- the shared `drag` fallback
    // path) is that value at EVERY angle.
    {
        const double s = 1.0;
        for(double c = -1.0; c <= 1.0 + 1e-9; c += 0.1) {
            CHECK_NEAR(partCd(s, s, s, c), s, 1e-12, "symmetric part -> s");
        }
    }

    // A thin DISC: the opposite asymmetry to a cone -- sleek edge-on
    // (fwd/bwd low), blunt face-on (side high).
    {
        const double df = 0.2, ds = 1.1, db = 0.2;
        CHECK_TRUE(partCd(df, ds, db, 0.0) > partCd(df, ds, db, 1.0),
                   "disc: face-on > edge-on");
        CHECK_TRUE(partCd(df, ds, db, 1.0) < partCd(df, ds, db, 0.5),
                   "disc: edge (c=1) < 45 deg");
    }

    // Robustness: a slightly-out-of-range cosine (a non-unit axis or flow) is
    // clamped, not amplified -- c=+1.5 behaves like the forward anchor.
    CHECK_NEAR(partCd(fwd, side, bwd,  1.5), fwd, 1e-12, "c>1 clamps to forward");
    CHECK_NEAR(partCd(fwd, side, bwd, -1.5), bwd, 1e-12, "c<-1 clamps to backward");
}

static void test_surfaceFormCdA() {
    printf("== surfaceFormCdA: a plate's three ways to meet the flow (#559) ==\n");

    // wing_r2h2 exactly: 8 m2 of planform, 0.1 x 2 m of edge, profile cd 0.03,
    // blunt 1.0. These are the catalogue's numbers, so a drift in either shows up here.
    const double S = 8.0, edge = 0.2, fwd = 0.03, side = 1.0, bwd = 0.03;

    // Chordwise: the profile cd on the planform, and no plate term at all.
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd,  1.0, 0.0), S * fwd, 1e-12,
               "chordwise -> profile cd x planform");
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, -1.0, 0.0), S * bwd, 1e-12,
               "base-in -> backward anchor on the planform");
    // Out of the plate's plane: the planform IS the silhouette, so it pays as a plate.
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, 0.0, 1.0), S * side, 1e-12,
               "face-on -> planform as a flat plate");
    // Along the span: only the edge faces the flow. This is #559 -- #455 billed the
    // planform here too, keyed on the chord axis alone, which made a horizontal wing
    // in sideslip pay 8 m2 of plate for 0.1 m of aluminium (measured: 15.6 m2 of CdA
    // for the 453 bed's two wings, whose real silhouette there is under 1 m2).
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, 0.0, 0.0), edge * side, 1e-12,
               "spanwise -> edge x cd_side");
    CHECK_TRUE(surfaceFormCdA(S, edge, fwd, side, bwd, 0.0, 0.0) < S * side * 0.05,
               "a wing in sideslip pays under 5% of its planform");

    // The three weights are squared cosines of one unit flow against three
    // orthonormal axes, so they sum to 1: with every cd equal and the edge the same
    // size as the planform, EVERY direction bills the planform -- nothing lost, and
    // nothing charged twice.
    {
        const double u = 1.0;
        for(double c = -1.0; c <= 1.0 + 1e-9; c += 0.1) {
            for(double n = 0.0; n * n <= 1.0 - c * c + 1e-9; n += 0.1) {
                CHECK_NEAR(surfaceFormCdA(S, S, u, u, u, c, n), S, 1e-9,
                           "unit cd everywhere -> planform, whatever the direction");
            }
        }
    }

    // 45 deg of incidence in the chord/normal plane (no sideslip): half profile,
    // half plate -- the same sin^2 shape the body law uses, now on the right area.
    {
        const double r = 1.0 / std::sqrt(2.0);
        CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, r, r),
                   S * (0.5 * fwd + 0.5 * side), 1e-12,
                   "45 deg incidence -> .5 profile + .5 plate");
    }

    // Degenerate inputs: no planform -> nothing; no edge (a mesh-less def) -> the
    // spanwise share is simply not charged rather than an invented area.
    CHECK_NEAR(surfaceFormCdA(0.0, edge, fwd, side, bwd, 0.0, 0.0), 0.0, 0.0,
               "no planform -> 0");
    CHECK_NEAR(surfaceFormCdA(S, 0.0, fwd, side, bwd, 0.0, 0.0), 0.0, 0.0,
               "no edge area -> spanwise pays nothing");
    CHECK_NEAR(surfaceFormCdA(S, -1.0, fwd, side, bwd, 0.0, 0.0), 0.0, 0.0,
               "a negative edge is not a charge");
    // Out-of-range cosines clamp (a non-unit axis): the spanwise weight never goes
    // negative to make up for an over-unity pair, and nothing above 1 gets charged.
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, 1.5, 0.0), S * fwd, 1e-12,
               "c>1 clamps to chordwise");
    CHECK_NEAR(surfaceFormCdA(S, edge, fwd, side, bwd, 0.8, 0.7),
               S * (0.64 * fwd + 0.49 * side), 1e-12,
               "c^2+n^2 > 1 -> no negative spanwise term");
}

/* #556: which panels are one wing. The shipped airframes are the test set: two
   half-wings bolted at the centreline are ONE 8 m AR-4 wing, and the fins and the
   tailplane next to them are not. */
static void test_liftingSystems() {
    printf("== classifyLiftingSystems: two panels, one wing (#556) ==\n");
    const glm::dvec3 lat(1.0, 0.0, 0.0);
    const glm::dvec3 fwd(0.0, 0.0, 1.0);
    // A pane: mount (x, 0, z), planform `area`, mesh half-extents (spanHalf,
    // chordHalf), span along `spanAxis` (default ship X), chord along ship Z.
    const auto pane = [](double x, double spanHalf, double area,
                         double chordHalf = 1.0, double z = 0.0,
                         glm::dvec3 spanAxis = glm::dvec3(1.0, 0.0, 0.0)) {
        LiftingPane p;
        p.spanAxis = spanAxis;
        p.chordAxis = glm::dvec3(0.0, 0.0, 1.0);
        p.mount = glm::dvec3(x, 0.0, z);
        p.spanHalf = spanHalf;
        p.chordHalf = chordHalf;
        p.area = area;
        return p;
    };
    std::vector<LiftingSystem> sys;
    const std::vector<LiftingBridge> noBridge;

    // The jet/glider case: two 4 m half-wings mounted at +-2, meeting at x=0.
    {
        const std::vector<LiftingPane> v = { pane(-2.0, 2.0, 8.0), pane(2.0, 2.0, 8.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id.size() == 2 && id[0] == id[1], "co-planar halves -> one system");
        CHECK_TRUE(sys.size() == 1, "one system in the list");
        if(sys.size() == 1) {
            CHECK_NEAR(sys[0].span, 8.0, 1e-12, "span is the union: 8 m");
            CHECK_NEAR(sys[0].area, 16.0, 1e-12, "area is the sum: 16 m2");
            CHECK_TRUE(sys[0].members == 2, "two members");
            // AR 4 rather than two AR-2 systems: the same CL pays half the induced
            // drag, which is the whole point of the ticket.
            CHECK_NEAR(inducedCd(1.0, sys[0].span, sys[0].area),
                       inducedCd(1.0, 8.0, 16.0), 1e-12, "AR 4, not 2 x AR 2");
        }
    }
    // A hairline gap (float error, or a builder leaving 5 cm) still merges.
    {
        const std::vector<LiftingPane> v = { pane(-2.025, 2.0, 8.0), pane(2.025, 2.0, 8.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1], "a 5 cm gap still reads as one wing");
    }
    // A deliberately visible slot does not: two 2% tolerances and a centimetre.
    {
        const std::vector<LiftingPane> v = { pane(-2.06, 2.0, 8.0), pane(2.06, 2.0, 8.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] != id[1], "a 12 cm slot is two wings");
        CHECK_TRUE(sys.size() == 2, "two systems");
        if(sys.size() == 2) {
            CHECK_NEAR(sys[0].span, 4.0, 1e-12, "each half bills its own 4 m span");
        }
    }
    // Dihedral: mirrored 5 deg panels are still one wing, and the span is the
    // PROJECTED one (8 cos 5). This is the skiff's small-panel geometry.
    {
        const double t = 5.0 * std::numbers::pi / 180.0;
        const std::vector<LiftingPane> v = {
            pane(-2.0, 2.0, 8.0, 1.0, 0.0, glm::dvec3(-std::cos(t), std::sin(t), 0.0)),
            pane(2.0, 2.0, 8.0, 1.0, 0.0, glm::dvec3(std::cos(t), std::sin(t), 0.0)),
        };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1], "5 deg dihedral still one wing");
        if(sys.size() == 1) {
            CHECK_NEAR(sys[0].span, 8.0 * std::cos(t), 1e-9, "span is the projection");
        }
    }
    // Vertical fins: span up, so they never join a wing -- and each still bills its
    // OWN span rather than a lateral projection of zero.
    {
        const std::vector<LiftingPane> v = {
            pane(-1.0, 2.0, 8.0),
            pane(1.0, 2.0, 8.0),
            pane(0.0, 0.6635, 2.0, 0.4, 0.5, glm::dvec3(0.0, 1.0, 0.0)),
        };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1] && id[2] != id[0], "a fin is not part of the wing");
        CHECK_TRUE(sys.size() == 2, "wing + fin");
        if(sys.size() == 2) {
            CHECK_NEAR(sys[1].span, 2.0 * 0.6635, 1e-12,
                       "a fin bills its vertical span");
        }
    }
    // A tailplane shares the wing's span and plane but not its chord station, so it
    // is a separate system -- the case the whole chord test exists for.
    {
        const std::vector<LiftingPane> v = { pane(0.0, 2.0, 8.0), pane(0.0, 2.0, 4.0, 1.0, 3.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] != id[1], "a tailplane 3 m aft is not the wing");
    }
    // A flap hinged at the trailing edge touches in chord, so it IS part of the wing.
    {
        const std::vector<LiftingPane> v = { pane(0.0, 2.0, 8.0),
                                             pane(0.0, 2.0, 2.0, 0.4, 1.4) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1], "a flap at the trailing edge joins the wing");
        if(sys.size() == 1) {
            CHECK_NEAR(sys[0].area, 10.0, 1e-12, "its area joins the planform");
        }
    }
    // THE SHIPPED JET, exactly: wings mounted at +-3 with 2 m half-spans, so their
    // roots are 2 m apart, on a fuselage 2 m wide. They are one wing, carried by the
    // hull -- and `span` stays the panels' own width, not the hull's.
    {
        const std::vector<LiftingPane> v = { pane(-3.0, 2.0, 8.0), pane(3.0, 2.0, 8.0) };
        const std::vector<LiftingBridge> hull = { { -1.0, 1.0 } };
        const std::vector<int> id = classifyLiftingSystems(v, hull, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1], "a fuselage bridges the wing roots");
        if(sys.size() == 1) {
            CHECK_NEAR(sys[0].span, 8.0, 1e-12, "span is the panels', not the hull's");
            CHECK_NEAR(sys[0].area, 16.0, 1e-12, "area is the panels'");
        }
    }
    // A bridge that does not reach both roots is not a bridge.
    {
        const std::vector<LiftingPane> v = { pane(-3.0, 2.0, 8.0), pane(3.0, 2.0, 8.0) };
        const std::vector<LiftingBridge> narrow = { { -0.5, 0.5 } };
        const std::vector<int> id = classifyLiftingSystems(v, narrow, lat, fwd, sys);
        CHECK_TRUE(id[0] != id[1], "a hull too narrow to reach the roots joins nothing");
    }
    // Same gap with nothing in it: two wings (so the case above cannot pass just
    // because the gap is small).
    {
        const std::vector<LiftingPane> v = { pane(-3.0, 2.0, 8.0), pane(3.0, 2.0, 8.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] != id[1], "a 2 m gap with nothing in it is two wings");
    }
    // Transitivity: a chain of touching panels is one wing even where the ends do
    // not touch each other.
    {
        const std::vector<LiftingPane> v = { pane(-2.0, 1.2, 4.0), pane(0.0, 1.2, 4.0),
                                             pane(2.0, 1.2, 4.0) };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] == id[1] && id[1] == id[2], "a chain is one system");
        if(sys.size() == 1) {
            CHECK_TRUE(sys[0].members == 3, "three members");
            CHECK_NEAR(sys[0].span, 6.4, 1e-9, "union span across the chain");
        }
    }
    // A plate whose chord is turned out of the fore/aft plane (say, a canard
    // mounted on its edge) is not a wing in this sense and stays on its own.
    {
        LiftingPane odd = pane(0.0, 2.0, 4.0);
        odd.chordAxis = glm::dvec3(0.0, 1.0, 0.0);
        const std::vector<LiftingPane> v = { pane(0.0, 2.0, 8.0), odd };
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id[0] != id[1], "a plate on its edge does not join a wing");
    }
    // Degenerate input.
    {
        const std::vector<LiftingPane> v;
        const std::vector<int> id = classifyLiftingSystems(v, noBridge, lat, fwd, sys);
        CHECK_TRUE(id.empty() && sys.empty(), "no panes, no systems");
    }
}

int main() {
    test_density();
    printf("\n");
    test_densityTop();
    printf("\n");
    test_air_thermodynamics();
    printf("\n");
    test_layered_atmosphere();
    printf("\n");
    test_force();
    printf("\n");
    test_aeroFrame();
    printf("\n");
    test_liftDirection();
    printf("\n");
    test_liftForce();
    printf("\n");
    test_wing_lifts_up();
    printf("\n");
    test_partAero();
    printf("\n");
    test_surface_alpha_readout();
    printf("\n");
    test_neutral_point();
    printf("\n");
    test_surface_effectiveness();
    printf("\n");
    test_local_flow_sweep();
    printf("\n");
    test_liftCurve();
    printf("\n");
    test_lift_slope_mach();
    printf("\n");
    test_projectedArea();
    printf("\n");
    test_zeroTolerance();
    printf("\n");
    test_unionArea();
    printf("\n");
    test_cp_silhouette();
    printf("\n");
    test_partMounts();
    printf("\n");
    test_silhouetteMemo();
    printf("\n");
    test_flowDirection();
    printf("\n");
    test_inducedCd();
    printf("\n");
    test_partCd();
    printf("\n");
    test_surfaceFormCdA();
    printf("\n");
    test_liftingSystems();
    printf("\n");
    test_controlForce();
    printf("\n");
    test_controlDeflectionSign();
    test_control_pair_frames();
    test_control_rigging();
    printf("\n");
    test_controlSurfaceVisualRot();
    printf("\n");
    test_controlCl();
    printf("\n");
    test_control_trim();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
