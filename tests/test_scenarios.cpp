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
// (check_datum). Only a real system file can catch a body measured off some
// other radius, and the science tiers are read off that difference.
//
// And it gates the air's thermodynamic plumbing (check_air): the T/p/a that drag.h
// derives must come from each body's OWN gravity and authored composition, which
// only a real system file can show -- test_drag pins the math, not the copy.

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

/* The air's thermodynamic reads (#552 A2) reach the player through dragAtm(), the
   ONE place authored atmosphere params become the drag law's view. test_drag owns
   that math; this owns the PLUMBING over the shipped defs, which no other test can
   see: if vehicle.cpp:dragAtm stops copying a body's g or its authored composition,
   every body quietly reports Earth's air -- and Jool, whose H2/He/CH4 sky resolves to
   0.00236 kg/mol, prints dry air's 547 K instead of its own 44.6 K in the atlas. The
   literals are hand-computed off the shipped numbers; T is LINEAR in molar mass while
   p and a do not contain it at all, so Jool's sound speed moves only through its
   gamma and its temperature moves through its M.
   Since #630 the shipped files author a MIX rather than these two scalars, so the
   literals are hand-computed in two steps: resolve the mix by hand (M = sum(x M),
   gamma from the mole-weighted molar heat capacity), then apply the air laws. The
   oxidizer column is the third thing the mix resolves to and the only one a player
   can act on -- it is what drag.h:jetAirGate bills a jet on -- so it is pinned here
   rather than only in the pure-math test: a body whose mix never reaches the gate is
   a jet running on full throttle in a CO2 sky. */
static int check_air(const char *path, const System &sys) {
    struct Want { const char *name; double T; double a; double p; double ox;
                  double tol; };
    static const Want want[] = {
        /* dry air's mix, authored now rather than defaulted: N2 78.084, O2 20.946,
           Ar 0.934, CO2 0.033 %, which resolves to M 0.028965 kg/mol and gamma
           1.40146 (a CO2 molecule lifts the heat capacity, so a real air's gamma is
           a hair over diatomic 1.4). g 9.81, H 5500. */
        {"Kerbin", 187.96215670165995, 274.98291650985726, 66094.875,
                                                  0.23140616, 1e-9},
        // H2 85.5, He 14, CH4 0.5 %: M 0.00236415, gamma 1.42319. g 7.848, H 20000.
        // Jool is the densest air in this system and its oxidizer share is exactly
        // zero -- hydrogen is the fuel, and there is no oxidizer to burn it with.
        {"Jool",   44.630376392655037, 472.63460894186682, 313920.0, 0.0, 1e-9},
        // CO2 95.32, N2 2.7, Ar 1.6, O2 0.13 %: M 0.0434957, gamma 1.30485.
        {"Eve",    610.70058697001423, 390.2911824640027, 198456.3,
                                                   0.00095878, 1e-9},
        /* CO2 96.5, N2 3.5 %: M 0.043450, gamma 1.30265. g 8.87, H 16 km.
           The p column is the whole reason Venus joined this table (#684): a file
           cannot author a pressure, so the authored rho0 IS the pressure knob, and
           Venus carried 92 kg/m3 -- its surface pressure in BAR, typed into the
           density field -- which put the datum at 13.1 MPa, 129 atm, with nothing
           in the file to contradict it. 65 kg/m3 (utils/rss/html/venusfact.html:
           "Surface density: ~65. kg/m^3") gives 9.22 MPa, which is Venus's real
           92 bar to 0.3 %. T and a contain no rho0 and were already right. */
        {"Venus",  741.6441320850256, 429.9672411932908, 9224800.0, 0.0, 1e-9},
        /* Earth authors a LAYER TABLE (#552 A4) AND a mix (#630), so these are the
           shipped file's proof all the way through: the JSON parses, resolveAtmoLevels
           integrates the column at the Rs that mix implies, dragAtm carries the
           pointer, and the four reads answer from the levels. The literals are the
           1976 standard's own sea-level row; dry air's mix resolves to
           0.02896496 kg/mol, which puts the datum at 101,324.9 Pa -- 0.1 Pa from the
           standard's stated 101,325, because that number IS dry air at 1.225 kg/m3
           and 288.15 K. The tolerance is 1e-4 rather than 1e-9 because Earth authors
           g 9.82 where the standard uses 9.80665 -- a third of a percent aloft. */
        {"Earth",               288.15,  340.47075676386669, 101324.85477637536,
                                                  0.23140616, 1e-4},
    };
    int bad = 0, n = 0;
    for(TerrainBody *b : sys.bodies) {
        const AtmosphereParams &a = b->surface.atmosphere;
        if(!(a.sea_level_density > 0.0)) { continue; }  // no physical air to ask
        n++;
        const DragAtmosphere da = dragAtm(a, b->g);
        if(!(da.gravity == b->g) || !(da.molar_mass == a.molar_mass)
           || !(da.gamma == a.gamma) || !(da.oxidizer_share == a.oxidizer_share)
           || da.layer_count != (int)a.layers.size()) {
            printf("  %s: %s dragAtm lost its world: g %.4g vs %.4g, M %.4g vs %.4g,"
                   " gamma %.4g vs %.4g, ox %.6g vs %.6g, %d levels vs %d\n", path,
                   b->name.c_str(),
                   da.gravity, b->g, da.molar_mass, a.molar_mass, da.gamma, a.gamma,
                   da.oxidizer_share, a.oxidizer_share,
                   da.layer_count, (int)a.layers.size());
            bad++;
            continue;
        }
        const double T = airTemperature(da, 0.0);
        const double p = airPressure(da, 0.0);
        const double s = airSoundSpeed(da, 0.0);
        if(!(T > 0.0) || !(p > 0.0) || !(s > 0.0)) {
            printf("  %s: %s has air but answers T %.4g K, p %.4g Pa, a %.4g m/s\n",
                   path, b->name.c_str(), T, p, s);
            bad++;
        }
        /* #625: print what each body's profile ASSERTS beside the numbers that
           assert it. A guessed scale height is invisible in the atlas, which prints
           its temperature as fact, and obvious here -- Uranus (H 20000, M 0.023)
           reads 498 K for an ice giant -- which was a typed molar_mass, and is now
           what an H2/He/CH4 sky resolves to: 57 K (#630). */
        char prof[32];
        if(a.layers.empty()) {
            snprintf(prof, sizeof prof, "H %6.0f m", a.scale_height);
        } else {
            snprintf(prof, sizeof prof, "%d levels", (int)a.layers.size());
        }
        printf("  %s: g %6.3f %s rho0 %-6.3g M %.5f gamma %.3f ox %8.6f -> T %7.1f K,"
               " p %9.1f kPa, a %6.1f m/s\n",
               b->name.c_str(), b->g, prof, a.sea_level_density, a.molar_mass,
               a.gamma, a.oxidizer_share, T, p / 1000.0, s);
        for(const Want &w : want) {
            if(lower(b->name) != lower(w.name)) { continue; }
            // The oxidizer share has an absolute tolerance rather than a relative
            // one: Eve's is 0.00096 and the point of that number is that it is not
            // zero, so a relative band around it would be a rounding band.
            const double ox_tol = (w.ox == 0.0) ? 0.0 : 1e-8;
            if(std::fabs(T - w.T) > w.tol * w.T || std::fabs(s - w.a) > w.tol * w.a
               || std::fabs(p - w.p) > w.tol * w.p
               || !(std::fabs(a.oxidizer_share - w.ox) <= ox_tol)) {
                printf("  %s: %s air T %.4f K (want %.4f), sound speed %.4f m/s"
                       " (want %.4f), p %.1f Pa (want %.1f), oxidizer %.8f"
                       " (want %.8f) -- did a composition edit or a copy go"
                       " half-way?\n",
                       path, w.name, T, w.T, s, w.a, p, w.p,
                       a.oxidizer_share, w.ox);
                bad++;
            }
        }
    }
    printf("  %s: %d air bodies' thermodynamics come from their own world\n", path, n);
    return bad;
}

/* What the loader makes of a MIX (#630): what it refuses, and what it fills in when
   a body says nothing. Refusals matter because the mix is now the only way to state
   a gas -- a file that types molar_mass beside it, or names a gas the species table
   does not have, or writes a fraction that is not a number, would otherwise load and
   then divide the whole body's air by it. Mutated copies go under tmp/ the way
   check_raised_sea does, so the shipped files stay the input. */
static void expect_refuses(const char *label, const std::string &text,
                           const char *want_sub) {
    const std::string path = std::string("tmp/test_scenarios_mix_") + label + ".json";
    {
        std::ofstream o(path);
        o << text;
    }
    try {
        load_system(path.c_str(), nullptr, nullptr);
        printf("  %s: a body with a %s mix LOADED -- this is a load error, not a"
               " flight hazard\n", path.c_str(), label);
        failures++;
    } catch(const std::exception &e) {
        const std::string what = e.what();
        if(want_sub != nullptr && what.find(want_sub) == std::string::npos) {
            printf("  %s: refused, but not with the reason the test names (%s):"
                   " %s\n", path.c_str(), want_sub, what.c_str());
            failures++;
        }
    }
}

static void check_composition() {
    const char *src = "res/systems/ksp_system.json";
    std::ifstream f(src);
    if(!f.is_open()) {
        printf("  cannot open %s (run from the repo root)\n", src);
        failures++;
        return;
    }
    const nlohmann::json base = nlohmann::json::parse(f);
    std::filesystem::create_directories("tmp");

    // A helper that hands back the base file with one atmosphere edit applied.
    auto patched = [&](const char *name, auto &&fn) {
        nlohmann::json j = base;
        for(auto &bv : j["bodies"]) {
            if(bv.value("name", std::string()) != name) { continue; }
            fn(bv["surface"]["atmosphere"]);
        }
        return j;
    };

    // Two statements of one fact: the scalars are derived, so typing them is a
    // contradiction rather than a redundancy.
    expect_refuses("typed_scalars", patched("Kerbin", [](nlohmann::json &a) {
                       a["molar_mass"] = 0.044;
                   }).dump(), "composition");
    expect_refuses("typed_gamma", patched("Kerbin", [](nlohmann::json &a) {
                       a["gamma"] = 1.3;
                   }).dump(), "composition");

    // A gas the species table does not have -- named in the message, because
    // "unknown gas" without the name is the useless half of the error.
    expect_refuses("unknown_gas", patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array(
                           { {{"gas", "SO2"}, {"fraction", 1.0}} });
                   }).dump(), "SO2");

    // A mix that names no gas at all, and one whose fractions are all zero, both
    // resolve to a divide by zero molar mass.
    expect_refuses("empty_mix", patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array();
                   }).dump(), nullptr);
    expect_refuses("zero_mix", patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array(
                           { {{"gas", "N2"}, {"fraction", 0.0}} });
                   }).dump(), nullptr);

    /* The two likeliest author typos, in the two shapes that used to escape: read with
       .value(), a number where a name goes and a string where a number goes both threw
       nlohmann's type_error out of load_system, and that message names neither the body
       nor the key. Both must now come back naming the entry. */
    expect_refuses("gas_not_a_string", patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array(
                           { {{"gas", 5}, {"fraction", 1.0}} });
                   }).dump(), "entry 1 needs a gas name");
    expect_refuses("fraction_not_a_number",
                   patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array(
                           { {{"gas", "N2"}, {"fraction", "0.5"}} });
                   }).dump(), "N2 needs a numeric fraction");
    // An entry that names a gas and forgets the fraction used to contribute 0
    // silently -- and a one-entry mix like that reads back as "every fraction is
    // zero", which names the wrong line.
    expect_refuses("no_fraction", patched("Kerbin", [](nlohmann::json &a) {
                       a["composition"] = nlohmann::json::array(
                           { {{"gas", "O2"}} });
                   }).dump(), "O2 needs a numeric fraction");

    // A fraction too large to be a number. The literal goes in as FILE text, because
    // a C++ 1e400 is already +inf at compile time and nlohmann would write it back
    // out as null. What the file gets is the READER's own refusal -- JSON has no
    // infinity, so out_of_range.406 fires before the mix is ever resolved. That is
    // worth pinning: it says the loader cannot be handed an inf fraction through a
    // number literal, which is why resolveMix only has to care about negatives.
    {
        std::string txt = patched("Kerbin", [](nlohmann::json &a) {
            a["composition"] = nlohmann::json::array(
                { {{"gas", "N2"}, {"fraction", 1.0}} });
        }).dump();
        const size_t at = txt.find("\"fraction\":");   // dump() puts no space after the colon
        if(at == std::string::npos) {
            printf("  huge_fraction: the mutated file lost its fraction\n");
            failures++;
        } else {
            size_t v = at + 11;   // past "fraction":
            size_t end = v;
            while(end < txt.size() && txt[end] != ',' && txt[end] != '}') { end++; }
            txt.replace(v, end - v, "1e400");
            expect_refuses("huge_fraction", txt, "number overflow");
        }
    }

    // And what a body that says NOTHING gets: dry air, oxidizer included. Eve loses
    // its mix here, so it stops being a CO2 world and reads the default -- the same
    // default a hand-written two-line system gets.
    const std::string dflt = "tmp/test_scenarios_mix_default.json";
    {
        std::ofstream o(dflt);
        o << patched("Eve", [](nlohmann::json &a) {
               a.erase("composition");
           }).dump();
    }
    System sys;
    try {
        sys = load_system(dflt.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        printf("  %s: a body with no composition failed to load: %s\n",
               dflt.c_str(), e.what());
        failures++;
        return;
    }
    TerrainBody *eve = find_body(sys, "Eve");
    if(eve == nullptr) {
        printf("  %s: Eve vanished from the mutated system\n", dflt.c_str());
        failures++;
        return;
    }
    const AtmosphereParams &a = eve->surface.atmosphere;
    if(!(std::fabs(a.oxidizer_share - dryAirOxidizerShare()) <= 0.0)) {
        printf("  %s: Eve with no mix reads oxidizer %.8f, not the dry-air default"
               " %.8f\n", dflt.c_str(), a.oxidizer_share, dryAirOxidizerShare());
        failures++;
    }
    // 0.0289649583: dry air's mix resolved by hand, NOT constants.h's 0.028964 --
    // the loader derives the scalars from the mix even when the mix is the default,
    // so there is one arithmetic and not two.
    if(!(std::fabs(a.molar_mass - 0.0289649583) <= 1e-10)) {
        printf("  %s: the default mix resolved to %.10f kg/mol, want 0.0289649583"
               " (hand-computed from N2 78.084 / O2 20.946 / Ar 0.934 / CO2 0.033"
               " %% by mole)\n", dflt.c_str(), a.molar_mass);
        failures++;
    }
    /* gamma is the assertion that PROVES the loader ran the arithmetic. AtmosphereParams
       starts life at constants.h's typed 1.4 and dryAirOxidizerShare() -- so the
       oxidizer check above passes identically whether or not the loader touched the
       body, and only a resolved gamma of 1.4014569 (the CO2 and Ar lift the molar heat
       capacity) rather than the typed 1.4 says the default mix was really resolved. */
    if(!(std::fabs(a.gamma - 1.4014568506) <= 1e-9)) {
        printf("  %s: the default mix resolved to gamma %.10f, want 1.4014568506"
               " (mole-weighted heat capacity, not a typed 1.4)\n",
               dflt.c_str(), a.gamma);
        failures++;
    }
    printf("  %s: a body with no composition gets dry air -- M %.7f kg/mol,"
           " gamma %.7f, oxidizer %.6f\n", dflt.c_str(), a.molar_mass, a.gamma,
           a.oxidizer_share);
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
        if(check_air(p, sys)) failures++;
    }
    // ...and the air bodies whose composition is authored (H2 giants, CO2 worlds) live
    // in the solar files, so the air check runs there too.
    for(const char *p : { "res/systems/solar_system.json" }) {
        System sys = load_system(p, nullptr, nullptr);
        if(check_air(p, sys)) failures++;
    }
    check_composition();
    check_raised_sea();
    if(failures) {
        printf("test_scenarios: %d FAILED\n", failures);
        return 1;
    }
    printf("test_scenarios: OK\n");
    return 0;
}
