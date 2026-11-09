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
#include <fstream>
#include <numbers>

#include <nlohmann/json.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>   // glm::rotate, for a tilted body

#include "insolation.h"   // starFromBody / subsolarLonLat / skyAtPlace (#724)
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

    /* Eight rows, and all of them land inside a percent or two -- two only looser
       because the sheet itself is published to two significant figures (Uranus's
       3.7 W/m^2). Pluto is NOT in this table: it is spelled out below, because its own
       two rows disagree and a 20 % band would hide anything. */
    const Row rows[] = {
        {"Mercury", 5.791e10, 9082.7, 0.068, 439.6, 0.01},
        {"Venus",   1.082e11, 2601.3, 0.770, 226.6, 0.01},
        {"Earth",   1.496e11, 1361.0, 0.294, 254.0, 0.01},
        {"Mars",    2.280e11,  586.2, 0.250, 209.8, 0.01},
        {"Jupiter", 7.781e11,   50.3, 0.343, 109.9, 0.02},
        {"Saturn",  1.427e12,   14.8, 0.342,  81.0,  0.02},
        {"Uranus",  2.870e12,    3.7, 0.300,  58.1,  0.05},
        {"Neptune", 4.497e12,    1.5, 0.290,  46.6,  0.05},
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

    /* Pluto, spelled out instead of parked at a 20 % tolerance (a band that wide would
       pass on almost any albedo). At ITS OWN authored albedo and ITS OWN irradiance row
       the law answers 32.2 K; the sheet's black-body row says 37.5 K, which is that same
       irradiance with no albedo at all (36.9 K). The sheet contradicts itself, the
       generator waives the pair in writing (FACT_SURFACE_KNOWN), and this pins OUR side
       tightly so a moved albedo still trips it. */
    CHECK_NEAR(blackbodyTemperature(solarFlux(L, 5.905e12), 0.720), 32.2,
               0.01, "Pluto: the figure the waiver argues about");

    /* ---- the shipped data, not a copy of it -------------------------------------- */
    /* The rows above are the corpus. These are OUR files: a hand edit to a star's
       authored temperature, or a regenerate that moves it, must not leave the suite
       green -- every flux in the game derives from that one number, and at this level
       nothing else looks at it (the load-time `solar` audit line is the human's half,
       and make_solar_system.py --check is the generator's). */
    struct StarPin {
        const char *path;
        double temp_K;      // what utils/*.py author, and why
    };
    const StarPin pins[] = {
        {"res/systems/ksp_system.json", 2838.1},    // "Kerbin receives Earth's sun"
        {"res/systems/solar_system.json", 5769.41}, // Earth's irradiance row inverted
    };
    for(const StarPin &p : pins) {
        std::ifstream f(p.path);
        if(!f.is_open()) {
            CHECK_TRUE(false, p.path);   // run from the repo root
            continue;
        }
        const nlohmann::json doc = nlohmann::json::parse(f, nullptr, false);
        double T = 0.0;
        if(!doc.is_discarded() && doc.contains("bodies")) {
            for(const nlohmann::json &b : doc["bodies"]) {
                if(b.value("type", std::string()) == "star"
                   && b.contains("surface")) {
                    T = b["surface"].value("temperature", 0.0);
                }
            }
        }
        CHECK_NEAR(T, p.temp_K, 1e-6, p.path);
    }

    /* ---- the spread of one star's light ----------------------------------------- */
    /* Inverse square is exact in this law, so the pin is tight: two distances a
       factor 3 apart differ by 9, whatever the star. */
    const double near_flux = solarFlux(L, 1.0e11);
    const double far_flux  = solarFlux(L, 3.0e11);
    CHECK_NEAR(near_flux / far_flux, 9.0, 1e-12, "flux follows 1/d^2");

    /* No star, no flux -- and NOT a zero distance with a zero flux, which a caller
       would print as a real reading. */
    CHECK_NEAR(solarFlux(0.0, 1.0e11), 0.0, 0.0, "no star gives no flux");
    /* A distance below the module's own floor is "no answer", not an enormous one:
       both entry points have to agree about that, or a direct solarFlux call returns a
       finite flux that sunSample would have called invalid. */
    CHECK_NEAR(solarFlux(L, kSunMinDistance_m / 10.0), 0.0, 0.0,
               "a zero distance is not a reading");
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

    /* ---- what an impossible albedo does ----------------------------------------- */
    /* A body that reflects everything absorbs nothing, so the equilibrium is not a
       number this law should hand back; blackbodyTemperature asserts the range instead.
       The REFUSAL that protects the data lives in the loader, and it is pinned there:
       tests/test_belts.cpp feeds a negative surface.temperature and an albedo of 1.2
       through load_system and requires both to throw. */

    /* ---- one star direction, which four callers used to spell by hand (#724) ----
       insolation.h exists because the map's terminator (surfmap.cpp), the calendar's
       solar day (system.cpp), the clock's "Solar:" row (gameui.cpp) and the log's
       elevation (tick.cpp) each wrote the same subtract-normalise-transpose step, and
       each had to get the TRANSPOSE right. These pins are that step, the subsolar point
       it defines, and the season bound a future temperature law leans on. */
    {
        const glm::dmat3 ident(1.0);
        const StarFromBody east = starFromBody(glm::dvec3(0.0),
                                               glm::dvec3(5.0e9, 0.0, 0.0), ident);
        CHECK_TRUE(east.valid, "a star 5e9 m away is placeable");
        CHECK_NEAR(east.dist_m, 5.0e9, 1e-12, "the distance rides along with it");
        CHECK_NEAR(east.to_sun.x, 1.0, 1e-12, "identity frame: root +X is +X");

        /* The transpose is the whole module in one line: root_orient carries the ROTATING
           axes into root axes, so a star sitting at root +X, seen through a +90 deg turn
           about +Y, stands at rotating +Z -- and equirect.h puts +Z at lon 270. Getting
           this backwards is silent: every angle stays finite and every day still has a
           noon, just one 180 degrees out of step with the map. */
        const glm::dmat3 quarter(glm::rotate(
            glm::dmat4(1.0), std::numbers::pi / 2.0, glm::dvec3(0.0, 1.0, 0.0)));
        const StarFromBody turned = starFromBody(glm::dvec3(0.0),
                                                 glm::dvec3(5.0e9, 0.0, 0.0), quarter);
        CHECK_NEAR(turned.to_sun.z, 1.0, 1e-12, "the frame step goes the other way");
        double noon_lon = 0.0, noon_lat = 0.0;
        subsolarLonLat(turned, noon_lon, noon_lat);
        CHECK_NEAR(noon_lon, 3.0 * std::numbers::pi / 2.0, 1e-12,
                   "a quarter turn moves local noon to lon 270");
        CHECK_NEAR(noon_lat, 0.0, 1e-12, "a star in the equatorial plane is over the equator");

        /* Local noon is defined by what a plate collects, so the two halves of the module
           have to agree at the point they name. */
        const SkyAtZenith noon = skyAtPlace(turned, noon_lon, noon_lat);
        CHECK_TRUE(noon.valid && noon.lit, "the subsolar point is in daylight");
        CHECK_NEAR(noon.cos_inc, 1.0, 1e-12, "local noon collects the whole beam");
        CHECK_NEAR(noon.elev_rad, std::numbers::pi / 2.0, 1e-12,
                   "and at noon the star is overhead");
        const SkyAtZenith anti = skyAtPlace(turned, noon_lon + std::numbers::pi,
                                            noon_lat);
        CHECK_NEAR(anti.cos_inc, 0.0, 1e-12, "the far side collects nothing");
        CHECK_NEAR(anti.elev_rad, -std::numbers::pi / 2.0, 1e-12,
                   "and the star is underfoot, not skipped");
        CHECK_TRUE(anti.valid && !anti.lit, "night is a reading, not an absence");

        /* The daily mean a redistribution model will one day multiply: sweeping longitude
           under a star in the equatorial plane, the mean cosine over a whole turn is
           1/pi -- the lit half of a sphere intercepts twice a radius' worth of beam. */
        double sum = 0.0;
        const int n = 100000;
        for(int i = 0; i < n; i++) {
            sum += skyAtPlace(east, 2.0 * std::numbers::pi * i / n, 0.0).cos_inc;
        }
        CHECK_NEAR(sum / n, 1.0 / std::numbers::pi, 1e-5,
                   "the daily mean cosine is 1/pi");

        /* SEASONS, pinned before anything uses them. A body tilted by theta, seen from a
           star that runs a full circle in the root XZ plane, keeps its declination inside
           [-theta, +theta] and touches both ends once per circuit. That is the shape a
           seasonal temperature law (roadmap rung D) will be written against, and it is a
           property of the geometry rather than of any authored number -- so it belongs in
           the pure test, where a wrong tilt axis in either the loader or this header goes
           red without a world being loaded. */
        const double tilt = std::numbers::pi * 23.44 / 180.0;
        const glm::dmat3 tipped(glm::rotate(glm::dmat4(1.0), tilt,
                                            glm::dvec3(1.0, 0.0, 0.0)));
        double max_dec = 0.0;
        bool out_of_tropics = false;
        for(int i = 0; i < 400; i++) {
            const double psi = 2.0 * std::numbers::pi * i / 400;
            const StarFromBody sv = starFromBody(
                glm::dvec3(0.0),
                5.0e9 * glm::dvec3(std::cos(psi), 0.0, std::sin(psi)), tipped);
            double dlon = 0.0, dlat = 0.0;
            subsolarLonLat(sv, dlon, dlat);
            if(std::fabs(dlat) > tilt + 1e-12) { out_of_tropics = true; }
            if(std::fabs(dlat) > max_dec) { max_dec = std::fabs(dlat); }
        }
        CHECK_TRUE(!out_of_tropics, "the star never leaves the tropics");
        CHECK_NEAR(max_dec, tilt, 1e-3, "the solstices reach the tilt itself");
        double eq_lon = 0.0, eq_lat = 1.0;
        subsolarLonLat(starFromBody(glm::dvec3(0.0), glm::dvec3(5.0e9, 0.0, 0.0), tipped),
                       eq_lon, eq_lat);
        CHECK_NEAR(eq_lat, 0.0, 1e-12,
                   "with the star in the equatorial plane, the tilt moves no seasons");

        /* Absences, which are the reason the log dashes instead of printing 0. */
        CHECK_TRUE(!starFromBody(glm::dvec3(1.0e6, 0.0, 0.0),
                                 glm::dvec3(1.0e6, 0.0, 0.0), ident).valid,
                   "a body at the star has no direction to report");
        CHECK_TRUE(!starFromBody(glm::dvec3(0.0),
                                 glm::dvec3(kSunMinDistance_m * 0.5, 0.0, 0.0),
                                 ident).valid,
                   "the flux's distance floor is the direction's too");
        CHECK_TRUE(!skyAtZenith(east, glm::dvec3(0.0)).valid,
                   "no zenith: the caller dashes rather than reading overhead");
        const SkyAtZenith edge = skyAtZenith(east, glm::dvec3(0.0, 1.0, 0.0));
        CHECK_NEAR(edge.elev_rad, 0.0, 1e-12, "on the horizon reads zero, not NaN");
        CHECK_TRUE(edge.valid && !edge.lit, "dusk is a reading");
        CHECK_NEAR(edge.cos_inc, 0.0, 1e-12, "and a horizon collects nothing");
    }

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
