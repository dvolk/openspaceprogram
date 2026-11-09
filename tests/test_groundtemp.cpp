//
// The ground's temperature under a star (src/groundtemp.h, #724 rung C): the one-knob swing
// around an authored anchor, the energy identity that makes the anchor exact, the anchor's
// order of authority, and the sentinel that keeps "no swing authored" apart from "a swing of
// zero". Pure math, pinned without the game -- the test_solar / test_drag shape.
//
// The end points are the numbers utils/ground_model.py FITTED from the corpus and
// utils/gen_systems.py / make_solar_system.py WROTE into res/systems/*.json. That is the
// seam this test exists to hold: the generators fit the knob in python, the game applies it
// in C++, and if either spelling of the law moves, the night end stops landing back on the
// pair it was fitted to. Same arrangement as atmo.h and utils/atmo_units.py (#636).
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

#include "groundtemp.h"

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

/* What the generators authored, and the pair each was fitted to. If a generator re-fits,
   these move and this test asks whether anyone argued for it. Three layers, three failure
   modes: `--check` catches a committed JSON drifting from its generator; this file catches
   either SPELLING of the law (python's and C++'s) moving under the other; the e2e case
   science/724 catches the readout -- the dossier and the solar log -- rendering something
   other than what these numbers imply.
     Earth  earthfact.html  "Diurnal temperature range: 283 K to 293 K", anchor = stated avg
     Moon   moonfact.html   "95 K to 390 K (equator)",       anchor = DERIVED black-body
     Eve    ksp_bodies.csv  night/day 160.02 - 420.00 K,     anchor = the air's datum
     Venus  venusfact.html  "Diurnal temperature range: ~0" -> f = 1, stated 737 K */
struct Authored {
    const char *name;
    double anchor_K;
    double even_share;
    double pair_night_K;   // the end the knob was fitted on
    double pair_day_K;     // the end the model PREDICTS, and how far off is graded
    double max_day_err_K;  // the fit's own gate: 8 % of the anchor, utils/ground_model.py
};

int main() {
    /* ---- the identity the whole model rests on: the anchor is EXACT ----------------
       <T^4> over the sphere is T0^4 for every f, because <cos_inc> is exactly 1/4. By
       quadrature rather than by the closed form: a wrong 1/4, a wrong exponent, or a ramp
       spelled in cos instead of cos^4 all move this. */
    for(const double f : {0.0, 0.3, 0.5, 0.9, 1.0}) {
        const int n = 200000;
        double s4 = 0.0;
        for(int i = 0; i < n; i++) {
            const double th = std::numbers::pi * (i + 0.5) / n;   // whole sphere
            const double c = std::cos(th);
            const double T = groundTemperature(1.0, f, std::max(0.0, c));
            s4 += T * T * T * T * std::sin(th);
        }
        const double mean4 = 0.5 * (std::numbers::pi / n) * s4;   // (1/2) int X sin th dth
        CHECK_NEAR(mean4, 1.0, 2e-5, "the sphere radiates exactly T0^4 whatever f is");
    }
    /* <cos_inc> = 1/4 on its own, the identity above standing on. */
    {
        const int n = 200000;
        double sc = 0.0;
        for(int i = 0; i < n; i++) {
            const double th = std::numbers::pi * (i + 0.5) / n;
            sc += std::max(0.0, std::cos(th)) * std::sin(th);
        }
        CHECK_NEAR(0.5 * (std::numbers::pi / n) * sc, 0.25, 1e-5,
                   "the sphere's mean floored cosine is 1/4");
    }

    /* ---- the two ends, and the relation that forces one from the other -------------- */
    for(const double f : {0.0, 0.25, 0.9, 1.0}) {
        const double T0 = 288.15;
        const double n4 = std::pow(groundTemperature(T0, f, 0.0) / T0, 4.0);
        const double d4 = std::pow(groundTemperature(T0, f, 1.0) / T0, 4.0);
        CHECK_NEAR(d4, 4.0 - 3.0 * n4, 1e-12,
                   "one knob forces the day end from the night end: d^4 = 4 - 3n^4");
    }
    CHECK_NEAR(groundTemperature(288.15, 1.0, 0.0), 288.15, 1e-12,
               "f = 1 is a uniform surface, night included");
    CHECK_NEAR(groundTemperature(288.15, 1.0, 1.0), 288.15, 1e-12,
               "and the same at noon");
    CHECK_NEAR(groundTemperature(270.4, 0.0, 1.0) / 270.4, std::pow(4.0, 0.25), 1e-12,
               "f = 0 at the subsolar point is local radiative balance, 4^1/4 T0");
    CHECK_NEAR(groundTemperature(270.4, 0.0, 0.0), 0.0, 1e-12,
               "and f = 0 in night radiates away -- the missing inertia, not a bug");
    CHECK_TRUE(groundTemperature(288.15, 0.9, 0.25)
                 < groundTemperature(288.15, 0.9, 0.75),
               "more light, warmer ground -- the swing is monotone in the cosine");

    /* ---- the seam: the authored knobs, applied here, land back on their pairs -------- */
    const Authored rows[] = {
        {"Earth",  288.00, 0.9323431643529773, 283.0, 293.0, 0.08 * 288.00},
        {"Moon",   270.33, 0.015251705420212182,  95.0, 390.0, 0.08 * 270.33},
        {"Eve",    291.68, 0.09058795570276411, 160.02, 420.00, 0.08 * 291.68},
        {"Venus",  737.00, 1.0,                 737.0, 737.0, 0.0},
    };
    for(const Authored &r : rows) {
        /* Night first: the generators fit f FROM this end, so the C++ must give it back. A
           drift in either spelling of the law shows up here in tenths of a kelvin. */
        CHECK_NEAR(groundTemperature(r.anchor_K, r.even_share, 0.0), r.pair_night_K, 1e-6,
                   (std::string(r.name) + ": the authored knob reproduces its night end").c_str());
        const double day = groundTemperature(r.anchor_K, r.even_share, 1.0);
        CHECK_TRUE(std::fabs(day - r.pair_day_K) <= r.max_day_err_K + 1e-9,
                   (std::string(r.name)
                    + ": the predicted day end stays inside the fit's gate").c_str());
        printf("  %-6s f %.6f  anchor %7.2f K -> night %6.1f K, day %6.1f K "
               "(the page states %6.1f, %+5.1f K off)\n",
               r.name, r.even_share, r.anchor_K,
               groundTemperature(r.anchor_K, r.even_share, 0.0), day,
               r.pair_day_K, day - r.pair_day_K);
    }

    /* ---- which temperature a body swings around ------------------------------------
       Stated ground, else the datum of the air on that ground, else the derived black-body,
       else nothing (0.0, and the caller dashes). The order is the corpus's authority, and
       most of the shipped system rides on rung two: the KSP worlds state an air datum and no
       ground reading (#625 put that datum on a published temperature). */
    CHECK_NEAR(groundAnchor_K(288.15, 291.68, 255.0), 288.15, 0.0,
               "a stated ground temperature outranks everything");
    CHECK_NEAR(groundAnchor_K(0.0, 214.81, 209.81), 214.81, 0.0,
               "with no ground reading, the air's datum is the ground");
    CHECK_NEAR(groundAnchor_K(0.0, 0.0, 270.33), 270.33, 0.0,
               "with no air either, the derived black-body anchors -- and says so");
    CHECK_NEAR(groundAnchor_K(0.0, 0.0, 0.0), 0.0, 0.0,
               "and with none of the three there is no anchor, so the row dashes");
    /* #729: this is what the game actually passes now -- the FIELD's sentinel, not a bare 0.0.
       If thermalAuthored() were `>= 0.0` the first line below would return -1 K as if a source
       had stated it, and a caller that forgot to check would multiply it into the law. */
    CHECK_NEAR(groundAnchor_K(kThermalUnauthored, 291.68, 255.0), 291.68, 0.0,
               "a sentinel ground temperature falls through to the air's datum");
    CHECK_NEAR(groundAnchor_K(kThermalUnauthored, kThermalUnauthored, 270.33), 270.33, 0.0,
               "a sentinel air datum falls through to the derived black-body");

    /* ---- the sentinel, which is the difference between Venus and Mercury ------------ */
    CHECK_TRUE(!evenShareAuthored(kEvenShareUnauthored),
               "absence is not a number, and not 1.0 either");
    CHECK_TRUE(evenShareAuthored(0.0), "a bare-rock f of exactly 0 is authored data");
    CHECK_TRUE(evenShareAuthored(1.0), "Venus's stated no-range is authored data");
    /* And NO default: the consumers dash rather than choosing a fallback. An earlier
       evenShareOrDefault() returned 1.0 for absence, which printed a constant hot ground
       under a midnight sun -- least defensible exactly where the body is airless -- and made
       Mercury look like Venus. There is no pin for a default, because there is no default:
       a body without a knob has no underfoot reading, which is what the readout tests and
       the '#' columns in --solar-log show. */

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
