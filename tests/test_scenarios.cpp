// test_scenarios.cpp -- the scenario table's dev gate, and the abs_r scenario
// beds must still match the system files they were authored against.
//
// The VAB dropdown hides the dev-only beds unless --dev-scenarios, so the table
// must keep at least one player-facing bed and every bed must still resolve by
// name (see check_vab_gate).
//
// heli-* pins a circular heliocentric radius so the transfer planner can be
// handed a planet-like origin orbit (it only lists children of the orbited
// body). Those radii are hand-copied from a system file's orb_ang_speed +
// parent mass, so any edit to res/systems/*.json silently turns them into
// wrong orbits -- and a wrong origin orbit silently skews every porkchop plot
// measured from it. Re-derive each from the file and compare.
//
// Runs from the repo root (it loads res/systems/*.json).

#include "system.h"
#include "vehicle.h"
#include "frame.h"
#include "body.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

static int failures = 0;

// The semi-major axis a body's authored rail implies.
static double semi_major(const TerrainBody *b) {
    Frame *fr = b->frame;
    if(!fr || fr->orb_ang_speed <= 0.0 || fr->parent_mu <= 0.0) { return -1.0; }
    // n = sqrt(mu/a^3)  ->  a = (mu/n^2)^(1/3)
    const double n = fr->orb_ang_speed;
    return std::cbrt(fr->parent_mu / (n * n));
}

// Scenario names are lowercase throughout the table; body names are not.
static std::string lower(std::string s) {
    for(char &c : s) { c = (char)std::tolower((unsigned char)c); }
    return s;
}

static TerrainBody *find_body(const System &sys, const std::string &name) {
    const std::string want = lower(name);
    for(TerrainBody *b : sys.bodies) {
        if(lower(b->name) == want) { return b; }
    }
    return nullptr;
}

static void check_system(const char *path) {
    System sys = load_system(path, nullptr, nullptr);
    int checked = 0;
    for(size_t i = 0; i < scenario_count(); i++) {
        const std::string sname = scenario_name_at(i);
        if(sname.compare(0, 5, "heli-") != 0) { continue; }
        const std::string planet = sname.substr(5);
        TerrainBody *b = find_body(sys, planet);
        if(b == nullptr) { continue; }   // this system has no such planet
        const double a = semi_major(b);
        if(a <= 0.0) {
            printf("  %s: %s has no rail to derive a from\n", path, planet.c_str());
            failures++;
            continue;
        }
        const ScenarioDef *sc = scenario_by_name(sname);
        const double rel = std::fabs(sc->abs_r - a) / a;
        if(rel > 1e-4) {
            printf("  %s: scenario '%s' abs_r = %.6g m but %s's rail implies "
                   "%.6g m (%.3f%% off)\n",
                   path, sname.c_str(), sc->abs_r, planet.c_str(), a, rel * 100.0);
            failures++;
        }
        checked++;
    }
    if(checked == 0) {
        printf("  %s: no heli-* bed matched a body here (expected at least one "
               "for a solar-system file)\n", path);
        failures++;
    } else {
        printf("  %s: %d heli-* beds match their rails\n", path, checked);
    }
}

/* The VAB dropdown's dev gate. The dropdown shows only the non-dev beds, so the
   table must keep at least one -- and today that is exactly "pad". Hiding is a
   UI filter, never a lookup one: every bed, dev or not, still resolves by name
   for --startship, the save fixtures and this test. */
static void check_vab_gate() {
    int shown = 0;
    for(size_t i = 0; i < scenario_count(); i++) {
        const std::string sname = scenario_name_at(i);
        if(!scenario_dev_only_at(i)) {
            shown++;
            if(sname != "pad") {
                printf("  scenario '%s' is not dev-only: the VAB dropdown would "
                       "offer it to a player without --dev-scenarios\n",
                       sname.c_str());
                failures++;
            }
        }
        try {
            if(scenario_by_name(sname) == nullptr) {
                printf("  scenario '%s' is enumerated but does not resolve\n",
                       sname.c_str());
                failures++;
            }
        } catch(const std::exception &e) {
            printf("  scenario '%s' is enumerated but does not resolve: %s\n",
                   sname.c_str(), e.what());
            failures++;
        }
    }
    if(shown == 0) {
        printf("  no scenario is player-facing: the VAB dropdown would be "
               "empty even without --dev-scenarios\n");
        failures++;
    }
    printf("  %d of %zu beds show in the VAB without --dev-scenarios\n",
           shown, scenario_count());
}

int main() {
    check_vab_gate();
    // The files the heli-* radii were read off. ksp_system/old_system are not
    // checked: they have no Mercury/Earth/Jupiter/Uranus.
    const char *paths[] = {
        "res/systems/solar_system.json",
        "res/systems/solar_system_measured.json",
        "res/systems/solar_system_named.json",
        "res/systems/solar_system_full.json",
    };
    for(const char *p : paths) { check_system(p); }
    if(failures) {
        printf("test_scenarios: %d FAILED\n", failures);
        return 1;
    }
    printf("test_scenarios: OK\n");
    return 0;
}
