// test_scenarios.cpp -- the scenario table's dev gate, and the abs_r scenario
// beds must still match the system files they were authored against.
//
// The VAB dropdown hides the dev-only beds unless --dev-scenarios, so the table
// must keep at least one player-facing bed and every bed must still resolve by
// name (see check_vab_gate).
//
// The flying beds' speeds are load-bearing in both directions: the circular ones
// are what the science bands and the chute-entry cases run on, and the slow one
// is what a plane is judged flying on, so check_flying_beds pins which is which
// and check_spawn_attitude pins the roll a flying bed starts with.
//
// heli-* pins a circular heliocentric radius so the transfer planner can be
// handed a planet-like origin orbit (it only lists children of the orbited
// body). Those radii are hand-copied from a system file's orb_ang_speed +
// parent mass, so any edit to res/systems/*.json silently turns them into
// wrong orbits -- and a wrong origin orbit silently skews every porkchop plot
// measured from it. Re-derive each from the file and compare.
//
// Runs from the repo root (it loads res/systems/*.json).
//
// It also gates one loader invariant that every shipped file shares: the
// rotating frame's SoI is the mean-sea datum plus the derived shell
// (check_system). Only a real system file can catch a body measured off some
// other radius, and the science tiers are read off that difference.

#include "system.h"
#include "vehicle.h"
#include "frame.h"
#include "body.h"
#include "bodylimits.h"
#include "science.h"   // orbitCutAlt: the tier boundary the datum feeds

#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

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

/* The datum the whole altitude ladder stands on (#406): the loader puts the
   near-body shell on the MEAN-SEA sphere, so a body's rotating SoI is exactly
   seaLevelRadius() + shellEdge(atmo top). science.h's low/high orbit cut is
   measured as (soi - datum), so a body whose SoI is measured off another radius
   shifts its science tiers silently -- and only a real system file can show it,
   since that is where radius, sea level and atmosphere are authored. Returns the
   number of bodies that disagree. */
static int check_datum(const char *path, const System &sys) {
    int bad = 0;
    for(TerrainBody *b : sys.bodies) {
        const double want = b->seaLevelRadius()
                          + shellEdge(b->surface.atmosphere.top());
        if(b->rot_frame->soi != want) {
            printf("  %s: %s rot SoI %.1f != datum %.1f + shell %.1f (off by %+.6f m)\n",
                   path, b->name.c_str(), b->rot_frame->soi,
                   b->seaLevelRadius(), want, b->rot_frame->soi - want);
            bad++;
        }
    }
    printf("  %s: %d/%d bodies' rot SoI sits on the mean-sea datum\n",
           path, (int)sys.bodies.size() - bad, (int)sys.bodies.size());
    return bad;
}

/* Every shipped file has sea_level 0 or absent (#412), so the line above cannot
   tell a datum from a radius. Load a MUTATED copy with Kerbin's sea 12 km up:
   the shell must ride with the datum, and the low/high orbit cut -- which is
   what a player's science tier turns on -- must not move. */
static void check_raised_sea() {
    const char *src = "res/systems/ksp_system.json";
    std::ifstream f(src);
    if(!f.is_open()) {
        printf("  cannot open %s (run from the repo root)\n", src);
        failures++;
        return;
    }
    nlohmann::json j = nlohmann::json::parse(f);
    TerrainBody *dry_b = nullptr;
    System dry = load_system(src, nullptr, nullptr);
    dry_b = find_body(dry, "Kerbin");
    if(dry_b == nullptr) {
        printf("  %s: no Kerbin to raise the sea on\n", src);
        failures++;
        return;
    }
    const double dry_sea = dry_b->seaLevelRadius();
    const double dry_soi = dry_b->rot_frame->soi;
    const double dry_cut = orbitCutAlt(dry_soi, dry_sea);

    const double wet_sea_m = 12000.0;
    bool patched = false;
    for(auto &bv : j["bodies"]) {
        if(bv.value("name", std::string()) != "Kerbin") { continue; }
        if(!bv.contains("surface") || !bv["surface"].is_object()) { bv["surface"] = nlohmann::json::object(); }
        bv["surface"]["sea_level"] = wet_sea_m;
        patched = true;
    }
    if(!patched) {
        printf("  %s: no Kerbin entry to patch\n", src);
        failures++;
        return;
    }
    std::filesystem::create_directories("tmp");
    const std::string tmp = "tmp/test_scenarios_raised_sea.json";
    {
        std::ofstream o(tmp);
        o << j.dump();
    }
    System wet;
    try {
        wet = load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        printf("  a body with a 12 km sea level fails to load: %s\n", e.what());
        failures++;
        return;
    }
    TerrainBody *b = find_body(wet, "Kerbin");
    if(b == nullptr) {
        printf("  %s: Kerbin vanished from the mutated system\n", tmp.c_str());
        failures++;
        return;
    }
    if(b->seaLevelRadius() != dry_sea + wet_sea_m) {
        printf("  Kerbin datum %.1f, expected %.1f (sea_level did not reach the body)\n",
               b->seaLevelRadius(), dry_sea + wet_sea_m);
        failures++;
    }
    // The shell rides with the datum...
    if(b->rot_frame->soi != dry_soi + wet_sea_m) {
        printf("  Kerbin rot SoI %.1f, expected %.1f: the shell did not lift with "
               "the datum (off by %+.1f m)\n",
               b->rot_frame->soi, dry_soi + wet_sea_m,
               b->rot_frame->soi - (dry_soi + wet_sea_m));
        failures++;
    }
    // ...so the cut over mean sea, and with it the science tiers, hold still.
    const double wet_cut = orbitCutAlt(b->rot_frame->soi, b->seaLevelRadius());
    if(wet_cut != dry_cut) {
        printf("  Kerbin's low/high orbit cut moved %.1f m with the datum: %.1f -> %.1f\n",
               wet_cut - dry_cut, dry_cut, wet_cut);
        failures++;
    }
    if(check_datum(tmp.c_str(), wet)) { failures++; }
    printf("  %s: a 12 km sea lifts Kerbin's shell with the datum and holds the "
           "orbit cut at %.0f m\n", tmp.c_str(), dry_cut);
}

static void check_system(const char *path) {
    System sys = load_system(path, nullptr, nullptr);
    int checked = 0;
    int datum_bad = 0;
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
    /* The datum the whole altitude ladder stands on (#406), pinned per body for
       every shipped file: see check_datum's note. */
    datum_bad += check_datum(path, sys);

    if(checked == 0) {
        printf("  %s: no heli-* bed matched a body here (expected at least one "
               "for a solar-system file)\n", path);
        failures++;
    } else {
        printf("  %s: %d heli-* beds match their rails\n", path, checked);
    }
    if(datum_bad) { failures += datum_bad; }
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

/* The flying beds' speed contract. flying-low/flying-high are circular ON
   PURPOSE: the science runs score their altitude band, so the bed has to hold
   that altitude with no lift (cases 122/123/126), and 151/152 use the km/s
   entry as the thing they test -- "the deployed chute reaches chute terminal
   from ~1.2 km/s; hull-only would stay >200 m/s". Slow those down and they
   still pass while testing nothing. flying-slow exists so a plane can start
   in air slow enough to fly (#451) -- the table pins the intent; e2e 451 pins
   that its fraction counts against the AIR-relative speed, because only a
   running game knows how fast the air itself is moving. */
static void check_flying_beds() {
    const ScenarioDef *low = nullptr, *high = nullptr, *slow = nullptr;
    try {
        low  = scenario_by_name("flying-low");
        high = scenario_by_name("flying-high");
        slow = scenario_by_name("flying-slow");
    } catch(const std::exception &e) {
        printf("  a flying bed does not resolve: %s\n", e.what());
        failures++;
        return;
    }
    if(low->speed_frac != 0.0 || high->speed_frac != 0.0) {
        printf("  flying-low/high are no longer circular: the science bands stop "
               "holding altitude and the chute entry cases stop having an entry\n");
        failures++;
    }
    if(!(slow->speed_frac > 0.0 && slow->speed_frac < 1.0)) {
        printf("  flying-slow speed_frac = %g: it must be suborbital but moving\n",
               slow->speed_frac);
        failures++;
    }
    if(!slow->atmo_frac || slow->alt_frac != low->alt_frac) {
        printf("  flying-slow is not flying-low's altitude (alt_frac %g, atmo_frac %d): "
               "it must sit in the same air, only slower\n",
               slow->alt_frac, slow->atmo_frac ? 1 : 0);
        failures++;
    }
    /* The fraction needs a circular speed to be a fraction OF, and air to be a
       fraction of AIR. The table is positional and 13 fields wide, so a row that
       grows a field can hand speed_frac to the wrong slot and still compile. */
    for(size_t i = 0; i < scenario_count(); i++) {
        const std::string sname = scenario_name_at(i);
        const ScenarioDef *d = scenario_by_name(sname);
        if(d->speed_frac <= 0.0) { continue; }
        if(d->ell_phase >= 0 || d->esc_frac > 0.0 || !d->atmo_frac) {
            printf("  scenario '%s' sets speed_frac = %g on a bed the fraction "
                   "cannot describe (ell_phase %d, esc_frac %g, atmo_frac %d)\n",
                   sname.c_str(), d->speed_frac, d->ell_phase, d->esc_frac,
                   d->atmo_frac ? 1 : 0);
            failures++;
        }
    }
    printf("  flying beds: low/high circular (entry rigs), slow at %.2f x the "
           "air-relative circular speed\n", slow->speed_frac);
}

/* The roll a flying bed spawns with. The one-argument faceAlong picks its roll
   AXIS from three candidates and keeps the one with the smallest dot, so a dir
   tilting a degree or two off a coordinate plane answers with a 90 deg roll: on
   Kerbin flying-slow, whose air-relative velocity sits 1.5 deg out of the orbit
   plane (the co-rotation term), that stood the wings vertical and sent the lift
   sideways (#451). The up_ref form has nothing to choose and cannot jump. */
static void check_spawn_attitude() {
    const double tilt = 1.5 * 3.14159265358979323846 / 180.0;
    const glm::dvec3 flat(1.0, 0.0, 0.0);
    const glm::dvec3 tilted(std::cos(tilt), std::sin(tilt), 0.0);
    const glm::dvec3 up(0.0, 1.0, 0.0);
    // The axis pick really does jump a quarter turn over 1.5 deg of tilt.
    const glm::dmat3 a = faceAlong(flat), b = faceAlong(tilted);
    const double nose_kept = glm::dot(a[2], b[2]);           // noses 1.5 deg apart
    const double up_kept = glm::dot(a[1], b[1]);
    if(up_kept > 0.1) {
        printf("  faceAlong's roll survived a %.1f deg dir tilt: the test below is "
               "not pinning the hazard (up axes still %.3f apart)\n", tilt * 57.29578, up_kept);
        failures++;
    }
    if(nose_kept < 0.999) {
        printf("  faceAlong stopped holding the nose on dir: cos = %.6f\n", nose_kept);
        failures++;
    }
    // The up_ref form: +Y along the local vertical once its component along the
    // nose is removed -- so it sits tilt degrees off true up, no further. Nose
    // along dir, basis right-handed.
    const glm::dmat3 m = faceAlong(tilted, up);
    const double y_up = glm::dot(m[1], up);
    const double nose_dir = glm::dot(m[2], glm::normalize(tilted));
    const double right = glm::dot(glm::cross(m[1], m[2]), m[0]);
    if(y_up < std::cos(tilt) - 1e-9 || nose_dir < 0.9999 || right < 0.9999) {
        printf("  faceAlong(dir, up) is not level: up=%.6f (want %.6f) nose=%.6f "
               "right=%.6f\n", y_up, std::cos(tilt), nose_dir, right);
        failures++;
    }
    printf("  spawn attitude: faceAlong's axis pick rolls %.0f deg over a %.1f deg "
           "tilt; the up_ref form holds the wings level\n",
           std::acos(up_kept) * 57.29578, tilt * 57.29578);
}

int main() {
    check_vab_gate();
    check_flying_beds();
    check_spawn_attitude();
    // The files the heli-* radii were read off. ksp_system/old_system are not
    // checked: they have no Mercury/Earth/Jupiter/Uranus.
    const char *paths[] = {
        "res/systems/solar_system.json",
        "res/systems/solar_system_measured.json",
        "res/systems/solar_system_named.json",
        "res/systems/solar_system_full.json",
    };
    for(const char *p : paths) { check_system(p); }
    // ...but the datum gate covers every shipped file, including the one the
    // game boots by default.
    for(const char *p : { "res/systems/ksp_system.json",
                          "res/systems/old_system.json" }) {
        System sys = load_system(p, nullptr, nullptr);
        if(check_datum(p, sys)) failures++;
    }
    check_raised_sea();
    if(failures) {
        printf("test_scenarios: %d FAILED\n", failures);
        return 1;
    }
    printf("test_scenarios: OK\n");
    return 0;
}
