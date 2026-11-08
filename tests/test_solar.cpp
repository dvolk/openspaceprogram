//
// The star's light (src/solar.h, header-only pure math, #687): a star's output from
// its authored radius and temperature, how that output spreads as 1/d^2, what a
// surface receives at an angle, and the equilibrium temperature a flux and an albedo
// imply. Pinned without the game, like test_drag and test_bodylimits.
//
// The pins are the numbers the VENDORED fact sheets publish (utils/rss/html/), not
// numbers invented here. That is deliberate: utils/make_solar_system.py sizes every
// star by inverting those same rows, so a test with its own expectations would stay
// green while the generator and the law drifted apart in the same direction.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <numbers>

#include <glm/glm.hpp>

#include "solar.h"

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
        const double _tol = (_e == 0.0) ? rel_tol                             \
                                        : rel_tol * std::fabs(_e);            \
        if (!std::isfinite(_a) || !std::isfinite(_e)                          \
            || std::fabs(_a - _e) > _tol) {                                   \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);         \
        }                                                                     \
    } while (0)

/* Sol's authored radius (utils/rss/html/sunfact.html) and the temperature
   utils/make_solar_system.py authors from Earth's irradiance row -- the number now
   committed in res/systems/solar_system*.json. If the generator moves it, this moves
   with it, and the row table below is what catches a move nobody argued for. */
static const double SUN_R   = 6.9634e8;      // [m]
static const double SUN_T   = 5769.41;       // [K]

struct Row {
    const char *name;
    double a;        // [m] mean orbital distance, as the generator derives it
    double flux;     // [W/m^2] the sheet's "Solar irradiance (W/m^2)" row
    double albedo;   // the sheet's "Bond albedo" row
    double bb;       // [K] the sheet's "Black-body temperature (K)" row
    double tol;      // how far our derived numbers may sit from the sheet's
};

int main() {
    const double L = starLuminosity(SUN_R, SUN_T);
    CHECK_TRUE(L > 0.0, "Sol's luminosity is positive");

    /* Eight of the nine rows land inside a percent, and the ninth is Pluto, whose
       OWN two rows disagree with each other: its 37.5 K black-body is its irradiance
       row with no albedo at all (36.9 K at A=0 against 32.2 K at its own 0.72). The
       generator waives that pair and prints it (FACT_SURFACE_KNOWN, #687); the
       tolerance here records the same argument so the waiver cannot be forgotten.
       Where a row is published to two significant figures (Uranus's 3.7 W/m^2) the
       tolerance follows the sheet, not the law. */
    const Row rows[] = {
        {"Mercury", 5.791e10, 9082.7, 0.068, 439.6, 0.01},
        {"Venus",   1.082e11, 2601.3, 0.770, 226.6, 0.01},
        {"Earth",   1.496e11, 1361.0, 0.294, 254.0, 0.01},
        {"Mars",    2.280e11,  586.2, 0.250, 209.8, 0.01},
        {"Jupiter", 7.781e11,   50.3, 0.343, 109.9, 0.02},
        {"Saturn",  1.427e12,   14.8, 0.342,  81.0,  0.02},
        {"Uranus",  2.870e12,    3.7, 0.300,  58.1,  0.05},
        {"Neptune", 4.497e12,    1.5, 0.290,  46.6,  0.05},
        {"Pluto",   5.905e12,    0.9, 0.720,  37.5,  0.20},
    };
    for(const Row &r : rows) {
        const double s = solarFlux(L, r.a);
        CHECK_NEAR(s, r.flux, r.tol, r.name);
        /* The /4 (intercept pi r^2, radiate 4 pi r^2) and the BOND albedo are the
           corpus's convention, not ours: this lands on every sheet's own row. */
        const double bb = blackbodyTemperature(s, r.albedo);
        CHECK_NEAR(bb, r.bb, r.tol, r.name);
    }

    /* The shipped KSP system: Kerbol's authored temperature is the inversion of
       "Kerbin receives Earth's sun" (utils/gen_systems.py, EARTH_SOLAR_FLUX), and
       the number committed in res/systems/ksp_system.json is 2838.1 K. Kerbin's rail
       is the wiki's semimajor axis (utils/ksp_bodies.csv). The wiki's own Kerbol
       range -- 4014.95 - 9621.2 K -- would put ~5,500 W/m^2 on a world built like
       Earth, and the generator prints that disagreement rather than resolving it. */
    CHECK_NEAR(solarFlux(starLuminosity(261600000.0, 2838.1), 13599840256.0),
               1361.0, 0.005, "Kerbin receives Earth's sun");

    /* ---- the spread of one star's light -------------------------------------- */
    /* Inverse square is exact in this law, so the pin is tight: two distances a
       factor 3 apart differ by 9, whatever the star. */
    const double near_flux = solarFlux(L, 1.0e11);
    const double far_flux  = solarFlux(L, 3.0e11);
    CHECK_NEAR(near_flux / far_flux, 9.0, 1e-12, "flux follows 1/d^2");

    /* No star, no flux -- and NOT a zero distance with a zero flux, which a caller
       would print as a real reading. */
    CHECK_NEAR(solarFlux(0.0, 1.0e11), 0.0, 0.0, "no star gives no flux");
    {
        const SunSample ss = sunSample(0.0, glm::dvec3(0.0), glm::dvec3(1.0e10, 0, 0));
        CHECK_TRUE(!ss.valid, "a dark star gives an invalid sample");
        CHECK_NEAR(ss.flux, 0.0, 0.0, "and no flux to mistake for a reading");
    }
    {
        const SunSample at = sunSample(L, glm::dvec3(0.0), glm::dvec3(0.0));
        CHECK_TRUE(!at.valid, "standing at the star centre is not a reading");
    }
    {
        const SunSample ss = sunSample(L, glm::dvec3(0.0), glm::dvec3(0.0, 2.0e11, 0.0));
        CHECK_TRUE(ss.valid, "a point off the star samples the star");
        CHECK_NEAR(ss.dist_m, 2.0e11, 1e-12, "distance is measured to the star");
        /* to_sun points FROM the point TO the star, so from +Y it is -Y. */
        CHECK_NEAR(ss.to_sun.y, -1.0, 1e-12, "to_sun points back at the star");
        CHECK_NEAR(ss.to_sun.x, 0.0, 1e-12, "and carries no other component");
    }

    /* ---- what a surface collects -------------------------------------------- */
    const glm::dvec3 up(0.0, 0.0, 1.0);
    CHECK_NEAR(incidentFlux(1000.0, up, up), 1000.0, 1e-12,
               "facing the beam collects all of it");
    CHECK_NEAR(incidentFlux(1000.0, up, glm::dvec3(1.0, 0.0, 0.0)), 0.0, 0.0,
               "edge-on collects nothing");
    CHECK_NEAR(incidentFlux(1000.0, up, -up), 0.0, 0.0,
               "a night side collects nothing, not a negative heating");
    CHECK_NEAR(incidentFlux(1000.0, up, glm::normalize(glm::dvec3(0.0, 1.0, 1.0))),
               1000.0 * std::sqrt(0.5), 1e-12, "the cosine, not the angle");

    /* ---- the horizon -------------------------------------------------------- */
    CHECK_NEAR(sunElevation(up, up), std::numbers::pi / 2.0, 1e-12,
               "overhead is +90 deg");
    CHECK_NEAR(sunElevation(up, -up), -std::numbers::pi / 2.0, 1e-12,
               "below the feet is -90 deg");
    CHECK_NEAR(sunElevation(up, glm::dvec3(1.0, 0.0, 0.0)), 0.0, 1e-12,
               "on the horizon is 0");
    /* A non-unit `up` (a hull at a body's exact centre) must not answer NaN. */
    CHECK_TRUE(std::isfinite(sunElevation(glm::dvec3(0.0, 0.0, 0.3), up)),
               "a short up vector stays finite");

    /* ---- the guard on an impossible albedo ---------------------------------- */
    /* A body that reflects everything absorbs nothing, so the equilibrium is not a
       number this law will hand back; the loader refuses albedo >= 1 outright.
       Asserts are live in the test build, so this stays a documented comment rather
       than a call -- the pin above on every sheet's own row is the teeth. */

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
