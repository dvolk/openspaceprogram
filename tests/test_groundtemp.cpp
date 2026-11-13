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
#include <cstring>
#include <numbers>
#include <string>
#include <vector>

#include "equirect.h"
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

#define CHECK_INT(actual, expected, msg)                                      \
    do {                                                                      \
        g_checks++;                                                           \
        long long _a = (actual), _e = (expected);                             \
        if(_a != _e) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s (got %lld, want %lld)\n", msg, _a, _e);          \
        }                                                                     \
    } while (0)

/* What the generators authored, and the pair each was fitted to. If a generator re-fits,
   these move and this test asks whether anyone argued for it. Three layers, three failure
   modes: `--check` catches a committed JSON drifting from its generator; this file catches
   either SPELLING of the law (python's and C++'s) moving under the other; the e2e case
   science/724 catches the readout -- the dossier and the solar log -- rendering something
   other than what these numbers imply.
     Earth  earthfact.html  "Diurnal temperature range: 283 K to 293 K", anchor = stated avg
     Moon   moonfact.html   "95 K to 390 K (equator)",       anchor = SOLVED from that pair
                                                              (temperature_src says so)
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

    /* ---- the seam: the authored knobs, applied here, land back on their pairs --------
       The pairs are the ones res/systems/*.json ships, not a remembered fit. The Moon's row
       used to carry the FIRST solve (270.33 K / 0.0152517), which lands the day end at
       381.2 K -- inside the fit's 8 % gate, so the test stayed green while auditing a pair
       no shipped file contains. When the generator changes the solve, re-derive the row
       from the shipped body: a stale row audits nothing. */
    const Authored rows[] = {
        {"Earth",  288.00, 0.9323431643529773, 283.0, 293.0, 0.08 * 288.00},
        /* The Moon's pair is not a fit: the generator SOLVES f so that BOTH stated ends fall
           out of one knob (temperature_src: "SOLVED from the stated range 95.00-390.00 K").
           So its day end has no gate to sit inside -- hold it exact and let the loop's 1e-9
           carry. Earth and Eve are still fits against a stated mean, so they keep the 8 %. */
        {"Moon",   276.49697165253286, 0.013935832091621402, 95.0, 390.0, 0.0},
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
    /* WHICH rung answered is its own question, and the readouts (the atlas' anchor row, the
       surface map's legend) name the answer instead of re-spelling the order (#636, #722). A
       derived anchor has to be visibly derived; a legend that says "stated" for a computed
       number is the readout lying about its source. */
    CHECK_INT(groundAnchorRung(288.15, 291.68, 255.0), kAnchorStated,
              "a stated ground reads as stated");
    CHECK_INT(groundAnchorRung(kThermalUnauthored, 214.81, 209.81), kAnchorAirDatum,
              "an air datum reads as an air datum");
    CHECK_INT(groundAnchorRung(kThermalUnauthored, 0.0, 270.33), kAnchorDerived,
              "a derived black-body reads as DERIVED, not as a measurement");
    CHECK_INT(groundAnchorRung(kThermalUnauthored, 0.0, 0.0), kAnchorNone,
              "and nothing at all reads as no anchor");
    CHECK_TRUE(groundAnchorRungName(kAnchorNone) != nullptr,
               "every rung has a word to show");
    CHECK_TRUE(std::strstr(groundAnchorRungName(kAnchorDerived), "DERIVED") != nullptr,
               "the derived rung's name says DERIVED in caps (#722)");

    /* ---- rung D: the air nearest the ground follows it (#724) ----------------------
       One authored number, one weight the column itself supplies. The pins below are the
       three ends a reader would check by eye: no coupling, full coupling at the datum, and
       the blend in between. The weight is the air's own pressure fraction, so "no air" and
       "above the top" both answer 0.0 and the caller gets no coupling for free. */
    CHECK_NEAR(followAirTemperature(288.15, 295.61, 0.0, 1.0), 288.15, 0.0,
               "a follow of 0.0 is a reading: the air keeps its profile");
    CHECK_NEAR(followAirTemperature(288.15, 295.61, 1.0, 1.0), 295.61, 0.0,
               "k=1 at the datum is the ground's own temperature");
    CHECK_NEAR(followAirTemperature(288.15, 295.61, 0.35, 1.0),
               288.15 + 0.35 * (295.61 - 288.15), 0.0,
               "the blend is linear in the knob");
    CHECK_NEAR(followAirTemperature(288.15, 295.61, 0.35, 0.0), 288.15, 0.0,
               "aloft, where the column says no weight, the profile stands alone");
    CHECK_NEAR(followWeight(101325.0, 101325.0), 1.0, 0.0, "at the datum the air is all there");
    CHECK_NEAR(followWeight(0.0, 101325.0), 0.0, 0.0, "above the top: nothing to couple");
    CHECK_NEAR(followWeight(50.0, 0.0), 0.0, 0.0, "no sea-level pressure: no fraction, no coupling");
    CHECK_NEAR(followWeight(2.0 * 101325.0, 101325.0), 1.0, 0.0,
               "below the datum the column is flat, and so is the weight");
    CHECK_TRUE(!followAuthored(kFollowUnauthored),
               "an air nobody priced does not follow its ground at all");
    /* THE COMPOSITION, which is where a loader-legal pair of knobs used to reach an assert.
       even_share 0.0 (a bare rock) at midnight answers a ground temperature of 0.0 K -- that
       is the model's own night end, pinned above -- and air_follow > 0 on such a body is
       legal to author. Handing that 0 K to followAirTemperature's assert would abort the game
       over a night sky, so coupledAirTemperature() answers the profile instead. Nothing else
       in the battery composes the two knobs, which is why this pin is here. */
    CHECK_NEAR(coupledAirTemperature(233.90, 214.81, 0.0, 0.5, 0.33, 0.0), 233.90, 0.0,
               "a bare rock's night has no ground temperature to follow: the profile stands");
    CHECK_NEAR(coupledAirTemperature(233.90, 214.81, 0.30, 0.5, 0.33, 1.0),
               233.90 + 0.5 * 0.33 * (214.81 * std::pow(3.1, 0.25) - 233.90), 1e-9,
               "the composed call is the blend, with the ground from the anchor and cosine");
    CHECK_NEAR(coupledAirTemperature(233.90, 214.81, 0.30, kFollowUnauthored, 0.33, 1.0),
               233.90, 0.0, "no knob: the profile stands, and nothing is asserted about it");
    CHECK_NEAR(coupledAirTemperature(233.90, 214.81, 0.30, 0.5, 0.0, 1.0),
               233.90, 0.0, "no weight (vacuum or above the top): the profile stands");
    CHECK_NEAR(coupledAirTemperature(0.0, 214.81, 0.30, 0.5, 0.33, 1.0),
               0.0, 0.0, "no air at all answers no temperature");
    CHECK_NEAR(followWeight(101325.0, 101325.0), 1.0, 0.0,
               "the weight is fail-closed: a fraction that is not a number answers none");
    CHECK_NEAR(followWeight(kThermalUnauthored, 101325.0), 0.0, 0.0,
               "a negative pressure is not a fraction either");

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

    /* ---- #794: the lag, pinned against a spelling that does not know it --------------
       The header answers the periodic steady state of  tau y' + y = max(a + b cos h, 0)
       in closed form. The only honest way to check a closed form is to relax the same ODE
       with a method that shares no line of its algebra: fixed-step RK4, started from an
       arbitrary value, iterated over many rotations until the transient dies. Everything
       below is that comparison, plus the identities the closed form is supposed to satisfy
       for reasons the solver does not use. */

    /* tau absent or 0 IS today's ground: the unlagged forcing, bit for bit, in both modes.
       (Not the geometry's dot product -- that is a different spelling of the same quantity
       and never agreed to the bit, #636. Nothing wires this into a picture yet, so a shipped
       map cannot move.) */
    {
        const double lat = 0.413, dec = -0.405;
        for(double tau : { 0.0, kLagTauUnauthored }) {
            const GroundLagRow r = groundLagRow(lat, dec, tau);
            CHECK_INT(r.mode, GroundLagRow::kInstant, "no lag authored: the row is instantaneous");
            for(int k = -25; k <= 25; k++) {
                const double h = 0.25 * (double)k;      // deliberately outside [-pi,pi] sometimes
                const double w = std::remainder(h, 2.0 * std::numbers::pi);
                CHECK_TRUE(r.at(h) == std::max(r.a + r.b * std::cos(w), 0.0),
                           "tau 0 answers the unlagged forcing exactly, and wraps its input");
            }
        }
    }

    /* a, b and h pinned against the engine's own sphere. Everything above and below works in
       (a, b, h), and the map will build h from a column's longitude minus the sun's subsolar
       longitude -- both of which come from equirect.h, and neither of which the relaxation
       harness touches (it feeds a and b directly). So a flipped sign or a lat/lon swap there moves
       the hot spot WEST and every lag test still passes. This is the pin that ties the convention
       to the geometry the ground law already uses: the dot product today's cosine comes from, and
       a + b cos h, are the same number at the same place and instant. */
    {
        const double places[][2] = { {0.0, 0.0}, {0.61, 2.2}, {-1.1, -0.7},
                                     {1.5, -3.0}, {-1.553, 3.13}, {0.3, -1.9} };
        const double suns[][2] = { {0.0, 0.0}, {-0.409, 1.9}, {0.409, -2.6},
                                   {1.3, 0.4}, {-1.5, 3.1} };
        for(const double (&pl)[2] : places) {
            for(const double (&su)[2] : suns) {
                const double lat = pl[0], lon = pl[1], dec = su[0], lon_ss = su[1];
                const double dot = glm::dot(equirectDir(lon, lat), equirectDir(lon_ss, dec));
                const double a = std::sin(lat) * std::sin(dec);
                const double b = std::cos(lat) * std::cos(dec);
                CHECK_NEAR(std::max(dot, 0.0), std::max(a + b * std::cos(lon - lon_ss), 0.0),
                           1e-9, "the lag's forcing IS the geometry's cosine at the same place");
            }
        }
    }

    /* The relaxation. Returns the y sampled on n steps of one rotation after `turns` passes. */
    auto relax = [](double a, double b, double tau, int n, int turns,
                    std::vector<double> &out) {
        const auto g = [&](double h) { return std::max(a + b * std::cos(h), 0.0); };
        const auto f = [&](double h, double y) { return (g(h) - y) / tau; };
        const double dh = 2.0 * std::numbers::pi / (double)n;
        /* Seeded away from equilibrium on purpose. Seeding at g(0) makes the polar-night case a
           comparison of zero against zero (its g is zero everywhere), and weakens the transient
           decay the turns exist to exercise. */
        double y = 0.5;
        for(int t = 0; t < turns; t++) {
            for(int k = 0; k < n; k++) {
                const double h = (double)k * dh;
                const double k1 = f(h, y), k2 = f(h + dh * 0.5, y + dh * k1 * 0.5);
                const double k3 = f(h + dh * 0.5, y + dh * k2 * 0.5);
                const double k4 = f(h + dh, y + dh * k3);
                y += dh * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
            }
        }
        /* Record one turn CONTINUING from the converged value. Restarting the recording pass at
           g(0) -- which is what this looked like at first -- re-adds the transient the turns above
           just spent 200 rotations removing, and the "closed form is wrong by 0.3" that follows is
           entirely the harness. out[k] is y at h = k * dh. */
        out.assign((size_t)n, 0.0);
        out[0] = y;
        for(int k = 0; k < n - 1; k++) {
            const double h = (double)k * dh;
            const double k1 = f(h, y), k2 = f(h + dh * 0.5, y + dh * k1 * 0.5);
            const double k3 = f(h + dh * 0.5, y + dh * k2 * 0.5);
            const double k4 = f(h + dh, y + dh * k3);
            y += dh * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
            out[(size_t)k + 1] = y;
        }
    };

    /* Cases chosen for the ways a lag solver goes wrong, not for coverage: a tilted star (the
       asymmetric terminators that broke my first attempt's sunset/sunrise constants), a row
       whose day is most of a rotation, a polar-day row (smooth forcing, so the peak lag is the
       exact atan(tau)), a polar-night row, a tau of about a map pixel and a half at 512 columns,
       a tau longer than a rotation. Tolerance is the relaxation's own grid error at n = 4000,
       not the closed form's -- measured against a form verified by an independent derivation:
       1e-8 or better on every case except tau = 0.02, where dh is a thirteenth of the time
       constant and the answer lands 1.2e-6 off. So the tightest margin here is 8x, not the three
       decades the other seven rows have. */
    struct LagCase { double lat_deg, dec_deg, tau; const char *why; };
    const LagCase kCases[] = {
        {  20.0,  -23.44, 0.42, "temperate winter hemisphere" },
        {  60.0,  -23.44, 3.00, "high latitude, short day, long lag" },
        {  60.0,   23.44, 3.00, "same latitude, summer: the arc is the other way round" },
        {   0.0,    0.00, 1.00, "equinox: the symmetric shortcut is exact ONLY here" },
        {  80.0,   23.44, 1.00, "polar day" },
        {  80.0,  -23.44, 1.00, "polar night" },
        {  35.0,  -12.00, 0.02, "tau well under one map pixel" },
        {  35.0,  -12.00, 8.00, "tau longer than a rotation" },
    };
    std::vector<double> yv;
    for(const LagCase &c : kCases) {
        const double lat = c.lat_deg * std::numbers::pi / 180.0;
        const double dec = c.dec_deg * std::numbers::pi / 180.0;
        const GroundLagRow r = groundLagRow(lat, dec, c.tau);
        const int n = 4000, turns = 200;
        relax(r.a, r.b, c.tau, n, turns, yv);
        double worst = 0.0, worst_h = 0.0;
        for(int k = 0; k < n; k++) {
            const double h = (double)k * 2.0 * std::numbers::pi / (double)n;
            const double got = r.at(h), want = yv[(size_t)k];
            if(std::fabs(got - want) > worst) { worst = std::fabs(got - want); worst_h = h; }
            CHECK_TRUE(got >= -1e-12 && got <= 1.0 + 1e-12,
                       "the lagged forcing stays a share: it feeds the same law as cos_inc");
        }
        CHECK_TRUE(worst < 1e-5, ("closed form matches the relaxation (" + std::string(c.why)
                                  + ")").c_str());
        if(worst >= 1e-5) { printf("       worst %.3e at h=%.1f deg\n", worst, worst_h * 180.0 / std::numbers::pi); }
    }

    /* THE ANCHOR SURVIVES THE LAG, which is the claim the header makes out loud. Averaging the
       ODE over a rotation kills the derivative, so the phase mean of y equals the phase mean of
       the forcing for every tau -- and the sphere mean of that forcing is 1/4, which is what
       makes T0 an anchor rather than a fitted number. Pinned per row and on the sphere. */
    for(double tau : { 0.42, 1.0, 3.0 }) {
        double sphere = 0.0, wsum = 0.0;
        const int nl = 361, np = 720;
        for(int j = 0; j < nl; j++) {
            const double lat = -std::numbers::pi / 2 + std::numbers::pi * (double)j / (double)(nl - 1);
            const GroundLagRow r = groundLagRow(lat, -23.44 * std::numbers::pi / 180.0, tau);
            double pm = 0.0;
            for(int k = 0; k < np; k++) { pm += r.at((double)k * 2.0 * std::numbers::pi / (double)np); }
            pm /= (double)np;
            double gm = 0.0;
            for(int k = 0; k < np; k++) {
                gm += std::max(r.a + r.b * std::cos((double)k * 2.0 * std::numbers::pi / (double)np), 0.0);
            }
            gm /= (double)np;
            /* Absolute, not relative: these means run from ~1e-5 on a winter polar row to 0.4 in
               the summer one, and a relative tolerance on a number that is already essentially
               zero asks the quadrature to be exact. Measured worst gap 1.9e-6. */
            CHECK_TRUE(std::fabs(pm - gm) < 1e-5,
                       "the phase mean of the lagged ground equals the forcing's");
            sphere += std::cos(lat) * pm;
            wsum += std::cos(lat);
        }
        /* Measured at these grids: the two phase means agree to 1.9e-6 absolute and the sphere to
           2.8e-7, and neither gap depends on tau -- both are pure quadrature of the same kinked
           forcing, which is the shape of evidence that the identity is exact and the grid is not. */
        CHECK_NEAR(sphere / wsum, 0.25, 1e-4,
                   "and the sphere still radiates exactly the power the star delivers");
    }

    /* Sign convention and the physical shape: east-positive hour angle means the hot spot rides
       EAST of noon (a body turning west-to-east carries the warm meridian past the star first),
       and the coldest moment of a rotation is in the MORNING -- after sunrise, before noon --
       not at local midnight. That is where a real weather station's minimum sits, and it is not
       obvious: the skin is still losing heat for a while after the star clears the horizon. */
    {
        const double lat = 20.0 * std::numbers::pi / 180.0;
        const double dec = -23.44 * std::numbers::pi / 180.0;
        for(double tau : { 0.42, 1.0, 3.0 }) {
            const GroundLagRow r = groundLagRow(lat, dec, tau);
            const int np = 7200;
            double hmax = 0.0, vmax = -1.0, hmin = 0.0, vmin = 2.0;
            for(int k = 0; k < np; k++) {
                const double h = std::remainder((double)k * 2.0 * std::numbers::pi / (double)np,
                                                2.0 * std::numbers::pi);
                const double v = r.at(h);
                if(v > vmax) { vmax = v; hmax = h; }
                if(v < vmin) { vmin = v; hmin = h; }
            }
            CHECK_TRUE(hmax > 0.0, "the day's peak sits east of the subsolar meridian");
            CHECK_TRUE(hmin > -r.h0 && hmin < 0.0,
                       "the rotation's minimum is between sunrise and noon, never at midnight");
            CHECK_TRUE(hmax < std::atan(tau),
                       "the peak lag stays under atan(tau): the night break pulls it in");
        }
        /* A polar-day row has no terminator to break the forcing, so there -- and only there --
           the peak lag IS exactly atan(tau). Measured: 45.00 deg at tau 1, 26.55 at tau 0.5. */
        for(double tau : { 0.5, 1.0, 2.0 }) {
            const GroundLagRow pd = groundLagRow(80.0 * std::numbers::pi / 180.0,
                                                 23.44 * std::numbers::pi / 180.0, tau);
            CHECK_INT(pd.mode, GroundLagRow::kPolarDay, "80N under a 23.4 tilt is lit all rotation");
            double hmax = 0.0, vmax = -1.0;
            for(int k = 0; k < 20000; k++) {
                const double h = std::remainder((double)k * 2.0 * std::numbers::pi / 20000.0,
                                                2.0 * std::numbers::pi);
                const double v = pd.at(h);
                if(v > vmax) { vmax = v; hmax = h; }
            }
            CHECK_NEAR(hmax, std::atan(tau), 1e-3, "a smooth forcing lags by exactly atan(tau)");
        }
    }

    /* h0 answers "where does this latitude's sun rise and set" in EVERY mode, because the
       follow-up reads it for the terminator and any day-length row. Filling it only in the
       day/night branch leaves a 0.0 that looks like "no day arc" on a row that is lit all
       rotation -- and on an unlagged row, whose real day arc is 1.12 rad at lat 45. */
    {
        const double pi = std::numbers::pi, d2r = pi / 180.0;
        CHECK_NEAR(groundLagRow(80.0 * d2r, 23.44 * d2r, 1.0).h0, pi, 1e-9,
                   "a polar-day row's day arc is the whole rotation");
        CHECK_NEAR(groundLagRow(80.0 * d2r, -23.44 * d2r, 1.0).h0, 0.0, 0.0,
                   "a polar-night row's is nothing");
        const GroundLagRow un = groundLagRow(45.0 * d2r, -23.44 * d2r, 0.0);
        CHECK_NEAR(un.h0, std::acos(-un.a / un.b), 1e-9,
                   "an unlagged row knows its own day arc too");
        /* dec at the pole: b collapses to 0, which must land on a polar mode rather than an acos
           of -a/0. This is also the edge the lat/dec range assert is written for. */
        const GroundLagRow pole = groundLagRow(30.0 * d2r, 0.5 * pi, 1.0);
        CHECK_INT(pole.mode, GroundLagRow::kPolarDay, "a star over the pole lights that hemisphere");
        CHECK_NEAR(pole.h0, pi, 1e-9, "and its rows know it");
        CHECK_TRUE(std::isfinite(pole.at(0.7)) && pole.at(0.7) >= 0.0 && pole.at(0.7) <= 1.0,
                   "the polar answer stays a share");
    }

    /* The landmine, twice over. The obvious spelling forms e^{+h0/tau}, which overflows a double
       once h0/tau exceeds ~709 -- but the smarter-looking fold, x * e^{-(h0-h)/tau}, does NOT
       overflow and is silently a different function (x^2 e^{h/tau}), so it fails without ever
       tripping an assert or producing a NaN. Both are pinned here: finite and in range at a tau
       no real body would use, and equal to the unlagged forcing as tau goes to zero. */
    for(double tau : { 1e-3, 1e-2, 3e-3, 1e-6 }) {
        for(double lat_deg : { -85.0, 0.0, 45.0, 85.0 }) {
            const GroundLagRow r = groundLagRow(lat_deg * std::numbers::pi / 180.0,
                                                -23.44 * std::numbers::pi / 180.0, tau);
            for(int k = 0; k < 36; k++) {
                const double v = r.at((double)k * 0.175);
                CHECK_TRUE(std::isfinite(v) && v >= 0.0 && v <= 1.0,
                           "a legal tiny tau never produces a NaN or a share out of range");
            }
        }
    }

    /* And a small tau really is the instantaneous answer: the guard above has to be harmless,
       not merely finite. */
    {
        const double lat = 0.6, dec = -0.3;
        const GroundLagRow tiny = groundLagRow(lat, dec, 1e-4);
        double worst = 0.0;
        for(int k = 0; k < 720; k++) {
            const double h = (double)k * 2.0 * std::numbers::pi / 720.0;
            const double w = std::remainder(h, 2.0 * std::numbers::pi);
            worst = std::max(worst, std::fabs(tiny.at(h) - std::max(tiny.a + tiny.b * std::cos(w), 0.0)));
        }
        CHECK_TRUE(worst < 1e-3, "a tau under a pixel is the unlagged ground, not merely not-NaN");
    }

    CHECK_TRUE(!lagTauAuthored(kLagTauUnauthored),
               "a body nobody priced has no lag, and that is not a stated 0");
    CHECK_TRUE(lagTauAuthored(0.0) && !lagTauActive(0.0),
               "a stated 0 is a fact and asks for no work -- the pair is one answer");
    /* And NO default: the consumers dash rather than choosing a fallback. An earlier
       evenShareOrDefault() returned 1.0 for absence, which printed a constant hot ground
       under a midnight sun -- least defensible exactly where the body is airless -- and made
       Mercury look like Venus. There is no pin for a default, because there is no default:
       a body without a knob has no underfoot reading, which is what the readout tests and
       the '#' columns in --solar-log show. */

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
