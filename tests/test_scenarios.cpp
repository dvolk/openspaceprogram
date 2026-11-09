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
// A bed can be turned onto a map point (--startship lat=/lon=), and that turn is
// pure geometry with one right answer: check_bed_turn pins eight hand-computed turns
// rather than reading the expected vectors off the code.
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
// only a real system file can show -- test_drag pins the math, not the copy. It
// also gates the sphere each column is weighed against, because a profile resolved
// against one body and read against another steps the air at every level seam.

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
   see: if vehicle.cpp:dragAtm stops copying a body's g, the sphere its air is weighed
   against, or its authored composition, every body quietly reports Earth's air -- and
   Jool, whose H2/He/CH4 sky resolves to 0.00236 kg/mol, prints dry air's 547 K
   instead of its own 44.6 K in the atlas. The literals are hand-computed off the
   shipped numbers; T is LINEAR in molar mass while p and a do not contain it at all,
   so Jool's sound speed moves only through its gamma and its temperature moves
   through its M.
   Since #630 the shipped files author a MIX rather than these two scalars, so the
   literals are hand-computed in two steps: resolve the mix by hand (M = sum(x M),
   gamma from the mole-weighted molar heat capacity), then apply the air laws. The
   oxidizer column is the third thing the mix resolves to and the only one a player
   can act on -- it is what drag.h:jetAirGate bills a jet on -- so it is pinned here
   rather than only in the pure-math test: a body whose mix never reaches the gate is
   a jet running on full throttle in a CO2 sky. */
static int check_air(const char *path, const System &sys) {
    /* `rho0` is the datum density the loader RESOLVES from the authored pressure
       (#739) -- the number airDensity(), the dossier and the jet gate read. It used to
       be the authored figure and needed no pin; now it is a computed one, and the two
       branches compute it differently (p0 / (g H) on an exponential, where the gas
       cancels, and p0 M / (R T0) beside a layer table). Without these the table branch
       is unpinned at unit level: hard-coding kAirMolarMass there instead of the body's
       resolved mix leaves every other number in this table exactly as it is, because p0
       comes from level 0's base_p and T, a and everything aloft never divide by the
       resolved rho0. */
    struct Want { const char *name; double T; double a; double p; double ox;
                  double rho0; double tol; };
    /* One number read ALOFT, because every Want literal above sits at the datum,
       where the column has not been walked yet (#683). Kerbin's edge density is
       what orbital drag is made of, and since #686 it comes from a level table, not
       from rho0 * exp(-Z / H): the 71 km level sits at 80,529.3 m geometric here and
       lapses -0.00154453 K/m, so the air at 69,999 m is what integrating that column
       in the body's own 1/r^2 field weighs. The old exponential put 1.375e-5 kg/m3 at
       the same altitude -- 14.7x thinner -- because its 188 K datum had been making
       Kerbin's air cold and thin all the way up. utils/atmo_profile.py integrates the
       same column a second time -- python, by quadrature, sharing no code with atmo.h
       -- and lands on these literals to seven digits; it is also what prints the q at
       the air top that the 85 km decision (#682, #686) was made on: 97.8 Pa on a
       circular orbit there. */
    struct Aloft { const char *name; double alt; double rho; };
    static const Aloft aloft[] = {
        {"Kerbin", 69999.0, 2.0277803e-4},
        /* The two rows in between are the air the game actually flies in, and until #686
           nothing pinned it: the datum read and the 70 km edge were pinned and the cruise
           regime was left to the golden flight bands (485/490/492), which notice a profile
           change only by going red for reasons a reader cannot see. 8500 m is the
           flying-cruise spawn bed (0.1 x Kerbin's 85 km top, #682) and 5000 m is the
           space-port hillside; both are level 0, weighed in the body's own 1/r^2 field. */
        {"Kerbin", 5000.0, 0.7386735114},
        {"Kerbin", 8500.0, 0.5019472257},
        /* The two other worlds that wear the table, one row at their spawn bed (0.1 x
           their own air top, so 9.5 km on Shay and 10.5 km on Laythe) and one just
           under the 71 km row. These are the rows that say the column was weighed in
           the body's field and not Kerbin's: at the same 69,999 m the three worlds
           answer 2.03e-4 kg/m3 (Kerbin, g 9.81), 4.56e-4 (Shay, 8.9933) and 1.37e-3
           (Laythe, 7.848) -- Laythe holds 6.75x the air there, because the exponent
           goes as 1/g and its column is weighed 1.28x more gently over these 70 km.
           A conversion that lost a radius lands outside this spread, and so does a
           table that quietly fell back to one exponential: Shay wearing its old
           H 6000 would read 3.9e-5 at the same height (11.6x thinner than its own
           pinned row) and a warmed H 9197 would read 1.43e-3 (3.1x thicker).
           utils/atmo_profile.py integrates this column a second time (python, by
           quadrature, sharing no code with atmo.h) and is what printed them. */
        {"Shay",   9500.0,  0.4964330},
        {"Shay",   69999.0, 4.561912e-4},
        {"Laythe", 10500.0, 0.5259753},
        {"Laythe", 69999.0, 1.369243e-3},
        /* #625's three authored tables, one row each ALONGSIDE the seam (the lid is
           9 km on Eve, 18 on Duna, 60 on Jool), so what these weigh is the INTEGRATED
           column -- the levels rows above only say where a level starts. The spread is
           the point: at 20 km Eve holds 3.7x the air Duna does at 40 km despite a column
           a fifth as tall, because Eve's 1.7 g packs its 292 K of CO2 into a 3.3 km
           scale height, while Jool's light hot gas is still 1.7e-4 kg/m3 at 400 km.
           A table that fell back to one isothermal level reads off these: Eve worn
           isothermal at its own lid (210.68 K throughout) holds 1.57e-3 kg/m3 at 20 km,
           0.57x this row, with a datum 3.8x denser -- and every number the atlas prints
           about that sky moving with it. */
        {"Eve",    20000.0, 2.77028315e-3},
        {"Duna",   40000.0, 7.48950737e-4},
        {"Jool",   400000.0, 1.69742860e-4},
    };
    /* Where each authored level sits and how fast it lapses, in the GEOMETRIC metres a
       system file speaks (#690). The 1976 standard this table was copied from states both
       numbers per GEOPOTENTIAL height, so the generator converts them: 11 km -> 11,019 m and
       51 km -> 51,411.6 m, each lapse scaled by the chord dZ/dz across the span that level
       lapses through. Nothing else catches the file getting that wrong: make_solar_system.py
       --check only says the json matches the script, and test_drag's US76 pins read at
       converted altitudes, so they pass against either spelling. Keeping a published lapse
       beside converted bases is the near miss the loader does see -- Earth's column then
       arrives 1.09 K off its own 71.8 km row, past kAtmoSeamTol, and the table is refused.
       Level 0 is pinned for its gradient only: its base is 0 by the anchor rule, and it is
       the level every player actually flies in, so it is the one a regression would quietly
       drop -- the chord factor at the datum is 1, so what it must still say is the published
       number TIMES the standard's own row spacing (0.99828 of -0.0065).
       Kerbin's seven rows are pinned because they are the SAME authored table converted
       against a different radius: 600 km instead of 6371 km. Earth's pins alone would stay
       green if the conversion hard-coded a radius or skipped a body, and Kerbin is where the
       map stops being rounding -- its 51 km row moves 4,737.7 m and its troposphere's gradient
       loses 1.8% of itself. #686 put that table in, and gen_systems.py --check cannot see it
       either. */
    struct Level { const char *name; int level; double alt; double lapse; };
    static const Level levels[] = {
        {"Earth", 0, 0.0, -0.00648878},    {"Earth", 1, 11019.0, 0.0},
        {"Earth", 2, 20063.0, 0.00099185}, {"Earth", 3, 32161.5, 0.00276538},
        {"Earth", 4, 47349.3, 0.0},        {"Earth", 5, 51411.6, -0.00274663},
        {"Earth", 6, 71800.2, -0.00195167},
        {"Kerbin", 0, 0.0, -0.00638083},    {"Kerbin", 1, 11205.4, 0.0},
        {"Kerbin", 2, 20689.7, 0.00091511}, {"Kerbin", 3, 33802.8, 0.00244303},
        {"Kerbin", 4, 50994.6, 0.0},        {"Kerbin", 5, 55737.7, -0.00225883},
        {"Kerbin", 6, 80529.3, -0.00154453},
        /* #700 gave the same authored table to Shay (550 km) and Laythe (500 km), so
           the conversion now has four radii to be wrong on. Two rows each, chosen for
           what each one can break: level 0's gradient is the chord across the
           troposphere the game actually flies in, and level 6's BASE is the number
           that gates the air top -- atmo.h refuses a table whose last level sits at
           or above `height`, so 81,524.0 m is why Shay's top could not stay at 60 km
           once these seven rows arrived, and 82,750.6 m is why Laythe's could not
           stay at 50. gen_systems --check only says the committed file matches the
           script; these rows say what the rows have to BE, which is the statement
           that survives someone regenerating the file. */
        {"Shay",   0, 0.0, -0.00637},       {"Shay",   6, 81524.0, -0.00148527},
        {"Laythe", 0, 0.0, -0.006357},      {"Laythe", 6, 82750.6, -0.00141818},
        /* #625's three authored tables. These were exponentials, which have no levels
           to pin, so nothing in this suite used to see a KSP body's SHAPE at all -- and
           the shape is what the rungs after this one read. Two rows each, same reasoning
           as Shay/Laythe above: level 0's lapse is the convective gradient the file's
           KSP_AIR argues for (and it is only refused by atmo.h at ~3.5x that limit,
           #703, so a typo there is a green build with a broken sky), and level 1's base
           is the lid: it must sit below `height`, and the lid's ALTITUDE is where the
           column stops cooling, which decides the density at the ceiling the band gate
           weighs. Eve lapses to 9 km because CO2 deposits at ~206 K at its own partial
           pressure and its lid must stay warmer than that (it reads 210.68 K); Duna's
           lid is at 18 km for the same reason at 600 Pa (151.81 K); Jool is convective
           all the way to 60 km (0.66 K/km IS its dry-adiabatic value). */
        {"Eve",    0, 0.0, -0.009},         {"Eve",    1, 9000.0, 0.0},
        {"Duna",   0, 0.0, -0.0035},        {"Duna",   1, 18000.0, 0.0},
        {"Jool",   0, 0.0, -0.00066},       {"Jool",   1, 60000.0, 0.0},
    };
    static const Want want[] = {
        /* dry air's mix, authored now rather than defaulted: N2 78.084, O2 20.946,
           Ar 0.934, CO2 0.033 %, which resolves to M 0.028965 kg/mol and gamma
           1.40146 (a CO2 molecule lifts the heat capacity, so a real air's gamma is
           a hair over diatomic 1.4).
           #686: Kerbin wears Earth's 1976 table, so its datum reads the standard's own
           sea-level row -- bit-identical to Earth's literals below, because p0 =
           rho0 (R/M) T0 and a = sqrt(gamma (R/M) T0) contain no g and no scale height.
           Before this diff the authored H 5500 fixed the datum instead: T = g H M / R =
           187.96 K, p0 66.1 kPa, a 275 m/s. Nothing about Kerbin's SURFACE said 188 K --
           the trio lock said it, silently, and every Mach number and barometer on the home
           world paid ~24%. The tolerance stays 1e-9 because Kerbin authors g 9.81 and the
           datum does not read g at all; unlike Earth's, these are not the standard's g. */
        /* #739: the file now AUTHORS this pressure (US76's own 101,325 Pa row) and
           the loader resolves the 1.225 kg/m3 from it, so the pin is the authored
           number and the density below it is the consequence. Kerbin's datum used to
           read 101,324.85 Pa because someone typed 1.225 -- the standard's rounding --
           and the pin was that rounding's product to eleven digits. */
        {"Kerbin", 288.15, 340.47075676386669, 101325.0, 0.23140616,
                                            1.2250017557285484, 1e-9},
        /* #700 put Shay and Laythe into the same table Kerbin and Earth wear, so
           their datums are now BIT-identical to Kerbin's above -- p0 = rho0 (R/M) T0
           and a = sqrt(gamma (R/M) T0) contain no g and no radius, and all four
           worlds author rho0 1.225 with dry air at a 288.15 K datum. That is the
           point, not a coincidence: an ocean world's air used to answer 150.4 K
           (Laythe, H 5500) and 188.0 K (Shay, H 6000) because one exponential IS
           isothermal and the trio lock had already spent their scale heights as
           surface temperatures. Neither world has a sea that can hold water under
           an air that cold, and `has_sea` says both have one.
           So the datum rows below prove only one thing -- that a table reached these
           two worlds at all. What tells them apart is aloft, where g weighs the
           column: see their `levels` and `aloft` rows. Tolerance 1e-9 like Kerbin's,
           because none of these four numbers reads g. */
        {"Shay",              288.15,  340.47075676386669, 101325.0, 0.23140616,
                                            1.2250017557285484, 1e-9},
        {"Laythe",            288.15,  340.47075676386669, 101325.0, 0.23140616,
                                            1.2250017557285484, 1e-9},
        /* #625: Eve's profile is authored (a 291.68 K datum, a 9 K/km convective
           lapse, a lid where CO2 would otherwise snow out), so its datum now reads
           the mix and the pressure rather than a scale height. The mix is CO2 95.32,
           N2 2.7, Ar 1.6, O2 0.13 %: M 0.0434957, gamma 1.30485. rho0 3.56 kg/m3 is the
           consequence of 1.96 atm at 1.7 g -- denser than Kerbin's air by 3x, which is
           what the same COLUMN MASS weighs when the gravity is 1.7x. a 269.7 m/s is
           below Kerbin's 340 because a CO2 molecule is heavy, not because the air is
           cold. */
        {"Eve",    291.68, 269.729158001, 198456.3,
                                 0.00095878, 3.55934927952, 1e-9},
        /* Duna's is the row that says the ladder worked: its old H 13.6 km implied
           209.38 K, and the authored figure (this world's own sunlight, 209.81 K, plus
           Mars's published 5 K gap) lands 5.4 K away from a number nobody chose. */
        {"Duna",   214.81, 231.473787876, 600.372,
                                 0.00095878, 0.014621041273, 1e-9},
        // H2 85.5, He 14, CH4 0.5 %: M 0.00236415, gamma 1.42319.
        /* #625 moved Jool onto the convention every other giant in this repo is
           stated at -- the 1 bar level -- because a body with no ground has no other
           non-arbitrary datum. 100 kPa at 151.43 K of this mix resolves 0.188 kg/m3,
           next to Jupiter's published 0.162 at its own 1 bar; the 2.0 kg/m3 and 44.63 K
           this used to read were a plucked density times a plucked scale height, and
           67 K colder than Jool's own sunlight (#738). Its sound speed more than
           doubles: hot, light gas. */
        {"Jool",   151.43, 870.595075693, 100000.0, 0.0, 0.187771483353, 1e-9},
        /* CO2 96.5, N2 3.5 %: M 0.043450, gamma 1.30265. g 8.87, H 16 km.
           The p column is the whole reason Venus joined this table (#684): the file
           authored a rho0 and p0 was its consequence, so an author who typed a
           PRESSURE into the density field got a column 42% too fat, and
           Venus carried 92 kg/m3 -- its surface pressure in BAR, typed into the
           density field -- which put the datum at 13.1 MPa, 129 atm, with nothing
           in the file to contradict it. 65 kg/m3 (utils/rss/html/venusfact.html:
           "Surface density: ~65. kg/m^3") gives 9.22 MPa, which is Venus's real
           92 bar to 0.3 %. T and a contain no rho0 and were already right. */
        {"Venus",  741.6441320850256, 429.9672411932908, 9224800.0, 0.0, 65.0, 1e-9},
        /* Earth authors a LAYER TABLE (#552 A4) AND a mix (#630), so these are the
           shipped file's proof all the way through: the JSON parses, resolveAtmoLevels
           integrates the column at the Rs that mix implies, dragAtm carries the
           pointer, and the four reads answer from the levels. The literals are the
           1976 standard's own sea-level row; dry air's mix resolves to
           0.02896496 kg/mol. Since #739 the file states the standard's own 101,325 Pa
           row outright, so the 1.225 kg/m3 these rows imply is what the loader
           resolves -- 1.2250018, where dry air's four authored fractions sum to
           0.99997 and resolveMix normalizes by their own total, which is worth
           0.04% on the datum density -- rather than what an author typed. The
           tolerance stays 1e-4 rather than 1e-9 because Earth authors
           g 9.82 where the standard uses 9.80665 -- a third of a percent aloft. */
        {"Earth",               288.15,  340.47075676386669, 101325.0, 0.23140616,
                                            1.2250017557285484, 1e-4},
    };
    int bad = 0, n = 0;
    for(TerrainBody *b : sys.bodies) {
        const AtmosphereParams &a = b->surface.atmosphere;
        if(!(a.sea_level_density > 0.0)) { continue; }  // no physical air to ask
        n++;
        const DragAtmosphere da = dragAtm(a, *b);
        // The g a body's air is weighed in is the typed one WALKED to the datum
        // sphere (#697), not the typed one: identical for every shipped world
        // (sea_level 0), 3.9 % apart on a 12 km sea.
        const double want_g = datumGravity(b->g, b->radius, b->seaLevelRadius());
        if(!(da.gravity == want_g) || !(da.radius == b->seaLevelRadius())
           || !(da.molar_mass == a.molar_mass)
           || !(da.gamma == a.gamma) || !(da.oxidizer_share == a.oxidizer_share)
           || da.layer_count != (int)a.layers.size()) {
            printf("  %s: %s dragAtm lost its world: g %.4g vs %.4g, R %.4g vs %.4g,"
                   " M %.4g vs %.4g, gamma %.4g vs %.4g, ox %.6g vs %.6g, %d levels"
                   " vs %d\n", path, b->name.c_str(),
                   da.gravity, want_g, da.radius, b->seaLevelRadius(),
                   da.molar_mass, a.molar_mass, da.gamma, a.gamma,
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
           reads 498 K for an ice giant -- which was a typed molar_mass, and #630's
           H2/He/CH4 mix took it to 57 K. #695 took the sheet's own 27.7 km and its
           0.42 kg/m^3, which land 79.2 K at a 1.048 bar datum: what the sheet's own
           M 2.64 g/mol and H imply at the level its gravity row names. */
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
               || std::fabs(a.sea_level_density - w.rho0) > w.tol * w.rho0
               || !(std::fabs(a.oxidizer_share - w.ox) <= ox_tol)) {
                printf("  %s: %s air T %.12g K (want %.12g), sound speed %.12g m/s"
                       " (want %.12g), p %.12g Pa (want %.12g), resolved rho0 %.12g"
                       " kg/m3 (want %.12g), oxidizer %.8f (want %.8f) -- did a"
                       " composition edit or a copy go half-way?\n",
                       path, w.name, T, w.T, s, w.a, p, w.p,
                       a.sea_level_density, w.rho0, a.oxidizer_share, w.ox);
                bad++;
            }
        }
        for(const Level &w : levels) {
            if(lower(b->name) != lower(w.name)) { continue; }
            const bool present = w.level >= 0 && w.level < (int)a.layers.size();
            const AtmoLayer none;
            const AtmoLayer &L = present ? a.layers[w.level] : none;
            if(!present || std::fabs(L.base_alt - w.alt) > 0.1
                        || std::fabs(L.lapse - w.lapse) > 1e-12) {
                printf("  %s: %s level %d sits at %.1f m lapsing %.8f K/m, want %.1f m"
                       " lapsing %.8f K/m (%d levels in the file). For a table copied from a"
                       " published standard this row is the GEOPOTENTIAL-to-GEOMETRIC"
                       " conversion -- of the altitude AND the gradient -- going missing or"
                       " wrong (#690); for a hand-authored table (Eve, Duna, Jool: #625) it"
                       " is the authored shape itself, which is the half that --check cannot"
                       " see.\n",
                       path, w.name, w.level,
                       present ? L.base_alt : -1.0, present ? L.lapse : 0.0, w.alt, w.lapse,
                       (int)a.layers.size());
                bad++;
            }
        }
        for(const Aloft &w : aloft) {
            if(lower(b->name) != lower(w.name)) { continue; }
            const double r = airDensity(da, w.alt);
            if(std::fabs(r - w.rho) > 1e-4 * w.rho) {
                printf("  %s: %s air at %.0f m reads %.6e kg/m3, want %.6e (x%.3f)."
                       " The column is weighed in the body's own 1/r^2 field, so a"
                       " profile that lost its radius still reads plausible -- x2.8"
                       " too thin at Kerbin's 70 km row, x4.6 at its 85 km top (#683)\n",
                       path, w.name, w.alt, r, w.rho, r / w.rho);
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

/* Like expect_refuses, but for a patch that must LOAD -- and must load with the rows
   still attached. A loader that quietly drops them passes a "did it throw" test and
   leaves a body whose air never moves while its file says it does. A refusal an author
   hits while writing real data is the same class of bug from the other side: the file
   and the law disagree, and only one of them gets read. */
static void expect_loads(const char *label, const std::string &text,
                         const char *body, size_t want_rows) {
    const std::string path = std::string("tmp/test_scenarios_mix_") + label + ".json";
    {
        std::ofstream o(path);
        o << text;
    }
    try {
        const System sys = load_system(path.c_str(), nullptr, nullptr);
        TerrainBody *found = nullptr;
        for(TerrainBody *b : sys.bodies) {
            if(b->name == body) { found = b; break; }
        }
        if(found == nullptr) {
            printf("  %s: body '%s' is not in the loaded system\n", path.c_str(), body);
            failures++;
            return;
        }
        const size_t got = found->surface.atmosphere.wind.size();
        if(got != want_rows) {
            printf("  %s: %s loaded %zu wind rows, want %zu -- the file and the loaded"
                   " body disagree\n", path.c_str(), body, got, want_rows);
            failures++;
        }
    } catch(const std::exception &e) {
        printf("  %s: a legal patch was REFUSED: %s\n", path.c_str(), e.what());
        failures++;
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

    /* The unit marker (#690). A table copied from a published standard states its rows per
       GEOPOTENTIAL metre; that convention is converted where the table is WRITTEN
       (utils/make_solar_system.py) and its marker dropped, so a system file carrying
       heights: "geopotential" is claiming a unit the loader does not apply -- and applying
       geometric metres to it silently is precisely the bug. It has to be an error, not an
       unknown key. */
    expect_refuses("unit_marker", patched("Kerbin", [](nlohmann::json &a) {
                       a["heights"] = "geopotential";
                   }).dump(), "GEOMETRIC");

    /* THE ANCHOR (#739): what a file states about its air is now the datum PRESSURE,
       and `sea_level_density` is no longer an authored key. Refusing it is not tidiness:
       the atmosphere loader DEFAULTS an unknown key rather than rejecting it (#721), so a
       file left holding the old spelling would load as air of zero pressure -- every body
       in that system quietly loses its sky, drag, sound speed and jet gate at once, and
       nothing anywhere refuses. The message names the conversion so an author with an old
       file has the arithmetic, not just the complaint. */
    expect_refuses("authored_density", patched("Kerbin", [](nlohmann::json &a) {
                       a["sea_level_density"] = 1.225;
                   }).dump(), "sea_level_pressure");
    expect_refuses("anchorless_layers", patched("Kerbin", [](nlohmann::json &a) {
                       a.erase("sea_level_pressure");
                   }).dump(), "sea_level_pressure");
    /* The other three ways the anchor can be wrong, pinned so a message cannot rot: an
       exponential with no anchor at all, a NEGATIVE one (which would otherwise read as
       "no air" and survive a decade, the same failure the profile guards exist for), and
       an anchor beside BOTH profiles -- what a half-applied #739 migration leaves behind,
       where the file then states the column twice and disagrees with itself. */
    expect_refuses("anchorless_exponential", patched("Duna", [](nlohmann::json &a) {
                       a.erase("sea_level_pressure");
                   }).dump(), "sea_level_pressure");
    expect_refuses("negative_anchor", patched("Kerbin", [](nlohmann::json &a) {
                       a["sea_level_pressure"] = -101325.0;
                   }).dump(), "sea_level_pressure");
    expect_refuses("anchor_with_two_profiles", patched("Kerbin", [](nlohmann::json &a) {
                       a["scale_height"] = 8432.0;
                   }).dump(), "no scale_height");

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

/* atmosphere.wind (#688): what the loader refuses, and what it must take. A wind is
   the air's OWN velocity, and it feeds a Mach number, a dynamic pressure and every
   aero force on the body, so an unsorted table, an unbounded speed or a row on a body
   with no air must stop at the FILE rather than arrive as a NaN several steps later.
   The positive rows are the same size a bug: real authoring is coming (the published
   figures in utils/rss/html state Venus 0.3-1.0 m/s at its surface, Earth 0-100, Mars
   2-30), and a law that refuses legitimate data is the same failure wearing the other
   face. */
static void check_wind() {
    const char *src = "res/systems/ksp_system.json";
    std::ifstream f(src);
    if(!f.is_open()) {
        printf("  cannot open %s (run from the repo root)\n", src);
        failures++;
        return;
    }
    const nlohmann::json base = nlohmann::json::parse(f);
    std::filesystem::create_directories("tmp");
    auto patched = [&](const char *name, auto &&fn) {
        nlohmann::json j = base;
        for(auto &bv : j["bodies"]) {
            if(bv.value("name", std::string()) != name) { continue; }
            fn(bv["surface"]["atmosphere"]);
        }
        return j;
    };
    auto rows = [](std::initializer_list<nlohmann::json> rs) {
        nlohmann::json a = nlohmann::json::array();
        for(const auto &r : rs) { a.push_back(r); }
        return a;
    };
    auto row = [](double alt, double east, double north) {
        return nlohmann::json{{"alt", alt}, {"east", east}, {"north", north}};
    };

    // Legal authoring loads: a two-row profile, a single row (which is then the wind
    // everywhere on the body), and a calm-but-bounded west wind with a negative north
    // component. Each pins the ROW COUNT, so a loader that drops rows cannot pass.
    expect_loads("wind_profile", patched("Kerbin", [&](nlohmann::json &a) {
                     a["wind"] = rows({row(0.0, 0.0, 0.0), row(17000.0, 20.0, 0.0)});
                 }).dump(), "Kerbin", 2);
    expect_loads("wind_one_row", patched("Kerbin", [&](nlohmann::json &a) {
                     a["wind"] = rows({row(8000.0, -3.5, -1.25)});
                 }).dump(), "Kerbin", 1);
    // A row exactly AT the air's top is fine: windAt holds it above the table rather
    // than claiming the air calms down as it thins out.
    expect_loads("wind_at_top", patched("Kerbin", [&](nlohmann::json &a) {
                     a["wind"] = rows({row(85000.0, 1.0, 0.0)});
                 }).dump(), "Kerbin", 1);
    // A row under the datum is legal too: #245's raised-sea worlds fly below their own
    // sea level, and windAt holds the first row down there rather than inventing calm
    // air under it.
    expect_loads("wind_under_datum", patched("Kerbin", [&](nlohmann::json &a) {
                     a["wind"] = rows({row(-4000.0, 2.0, 0.0), row(20000.0, 6.0, 0.0)});
                 }).dump(), "Kerbin", 2);

    expect_refuses("wind_scalar", patched("Kerbin", [](nlohmann::json &a) {
                       a["wind"] = 5;
                   }).dump(), "must be");
    expect_refuses("wind_empty", patched("Kerbin", [](nlohmann::json &a) {
                       a["wind"] = nlohmann::json::array();
                   }).dump(), "is empty");
    expect_refuses("wind_row_scalar", patched("Kerbin", [](nlohmann::json &a) {
                       a["wind"] = nlohmann::json::array({5});
                   }).dump(), "must be an object");
    /* The typo net: {"easts": 5} would otherwise read as "no wind", which is the one
       silent outcome an author would never notice. Same reason --startships refuses an
       unknown field (#709). */
    expect_refuses("wind_typo_key", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json{{"alt", 0.0}, {"easts", 5.0}}});
                   }).dump(), "unknown field");
    expect_refuses("wind_backwards", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(5000.0, 1.0, 0.0), row(1000.0, 2.0, 0.0)});
                   }).dump(), "does not rise");
    expect_refuses("wind_same_alt", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(0.0, 1.0, 0.0), row(0.0, 2.0, 0.0)});
                   }).dump(), "does not rise");
    expect_refuses("wind_too_fast", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(0.0, 600.0, 0.0)});
                   }).dump(), "500");
    expect_refuses("wind_too_fast_north", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(0.0, 0.0, -600.0)});
                   }).dump(), "500");
    /* A number that is not a number must name the body, the row and the field. Left to
       nlohmann the author reads "type must be number, but is string" with none of
       those, which is how a 15-row table becomes an afternoon. */
    expect_refuses("wind_east_a_string", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json{{"alt", 0.0},
                                                        {"east", "10"},
                                                        {"north", 0.0}}});
                   }).dump(), "\"east\" must be a number");
    expect_refuses("wind_alt_a_string", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json{{"alt", "100"},
                                                        {"east", 1.0},
                                                        {"north", 0.0}}});
                   }).dump(), "\"alt\" must be a number");
    expect_refuses("wind_east_null", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json{{"alt", 0.0},
                                                        {"east", nullptr},
                                                        {"north", 0.0}}});
                   }).dump(), "\"east\" must be a number");
    /* A row missing a key loads as CALM air, which is the same silent nothing the
       unknown-key net exists to stop: {"alt": 100} is not "blowing at 100 m". */
    expect_refuses("wind_row_no_north", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json{{"alt", 0.0}, {"east", 5.0}}});
                   }).dump(), "missing \"north\"");
    expect_refuses("wind_row_empty", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({nlohmann::json::object()});
                   }).dump(), "needs \"alt\", \"east\" and \"north\"");
    // windAt scans linearly once per air-relative read per hull, so a huge table is a
    // loop on the aero path -- the same reason layers is capped.
    expect_refuses("wind_too_many_rows", patched("Kerbin", [&](nlohmann::json &a) {
                       nlohmann::json big = nlohmann::json::array();
                       for(int i = 0; i < 65; ++i) { big.push_back(row(i * 100.0, 1.0, 0.0)); }
                       a["wind"] = big;
                   }).dump(), "too many rows");
    // A row in the vacuum the same file already declared: nothing can ever reach it.
    expect_refuses("wind_above_top", patched("Kerbin", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(90000.0, 1.0, 0.0)});
                   }).dump(), "above the air's top");
    // Moho ships no atmosphere: the patch creates a render rim, and a rim has no air
    // to blow. Refusing this is what keeps "wind" from meaning "a number nobody reads".
    expect_refuses("wind_no_air", patched("Moho", [&](nlohmann::json &a) {
                       a["wind"] = rows({row(0.0, 5.0, 0.0)});
                   }).dump(), "physical air");

    /* Which shipped bodies are allowed to author wind (#688). The mechanism landed
       before the data on purpose: with no row authored, every shipped flight number,
       every e2e flight golden and every drag figure in the corpus was unchanged, so a
       regression in the wind path could not hide behind a re-baselined flight. Data
       comes one body at a time after that, and a body earns its row by pinning what the
       row DOES -- wind that no case pins is a number nothing verifies. So the list is
       small and explicit: Venus, whose wind is pinned by
       e2e flight/688-venus-wind-outruns-the-hull.
       Adding a body here owes three things in the same commit: an e2e case that pins
       the rows' effect on that body, a re-run of every flight case on it that quotes an
       airspeed or a drag figure, and a commit message naming which numbers moved. */
    static const char *const kWindsAuthored[] = {"Venus"};
    constexpr size_t kNumWinds = sizeof(kWindsAuthored) / sizeof(kWindsAuthored[0]);
    for(const auto &de : std::filesystem::directory_iterator("res/systems")) {
        if(de.path().extension() != ".json") { continue; }
        std::ifstream sf(de.path());
        const nlohmann::json j = nlohmann::json::parse(sf);
        for(const auto &bv : j["bodies"]) {
            const std::string bname = bv.value("name", "?");
            size_t slot = kNumWinds;
            for(size_t k = 0; k < kNumWinds; ++k) {
                if(bname == kWindsAuthored[k]) { slot = k; break; }
            }
            const bool has_key = bv.contains("surface")
                && bv["surface"].contains("atmosphere")
                && bv["surface"]["atmosphere"].contains("wind");
            /* "has wind" means rows the loader would take: an array with at least one
               row. system.cpp refuses an empty or scalar table at load, so a shipped
               file with `"wind": []` is a game that will not start, not calm air. */
            const bool authored = has_key
                && bv["surface"]["atmosphere"]["wind"].is_array()
                && !bv["surface"]["atmosphere"]["wind"].empty();
            if(slot == kNumWinds) {
                if(has_key && !authored) {
                    printf("  %s: %s has an atmosphere.wind key that is not a"
                           " non-empty array -- the loader refuses that at load, so the"
                           " shipped game will not start\n",
                           de.path().string().c_str(), bname.c_str());
                    failures++;
                } else if(authored) {
                    printf("  %s: %s authors atmosphere.wind while every shipped flight"
                           " golden on it still assumes none -- add a case that pins"
                           " what the rows do, re-run the flight cases that quote an"
                           " airspeed or a drag figure, and name it in kWindsAuthored,"
                           " all in the same commit\n",
                           de.path().string().c_str(), bname.c_str());
                    failures++;
                }
                continue;
            }
            /* A listed body owes its rows to EVERY shipped file that carries it: the
               four solar_system variants are one catalogue, and an e2e case can only
               pin one of them, so a generator change that dropped Venus's rows from
               three of four would otherwise be invisible here. */
            if(!authored) {
                printf("  %s: %s is listed in kWindsAuthored but this file gives it no"
                       " wind rows -- the generated variants have to agree, since an"
                       " e2e case pins only one of them\n",
                       de.path().string().c_str(), bname.c_str());
                failures++;
            }
        }
    }
}

/* Every shipped file has sea_level 0 or absent (#412), so the line above cannot
   tell a datum from a radius. Load a MUTATED copy with Kerbin's sea 12 km up:
   the shell must ride with the datum, the low/high orbit cut -- which is what a
   player's science tier turns on -- must not move, and the air must be weighed in
   the field that raised sphere actually sits in (#697). */
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
    // The air must be weighed in the field AT the sphere it starts on (#697). This
    // datum sits 12 km further out than any shipped world's, so the field its air is
    // weighed in is (600000/612000)^2 weaker than the typed one -- 4.0 % on g -- and
    // the column MULTIPLIES that: 6.5 % on the density 12 km up, 52 % at the 85 km
    // top (q there 142 Pa rather than 94).
    // Leaving the typed 9.81 in place is the mutation; the literal below is what
    // utils/atmo_profile.py integrates by quadrature, sharing no code with atmo.h.
    // It moved in #739 by +1.4e-6 (0.3435505 -> 0.3435510): the file now authors
    // US76's own 101,325 Pa rather than that standard's rounded 1.225 kg/m3. For a
    // LAYER body the datum density contains no g at all (rho0 = p0 M / (R T0)); the
    // datum g only weighs the integration UP to 12 km, which is what this row is about.
    // utils/atmo_profile.py agrees on the new number, which is the check working.
    const DragAtmosphere wda = dragAtm(b->surface.atmosphere, *b);
    const double rho12 = airDensity(wda, 12000.0);
    if(!(std::fabs(rho12 / 0.3435510 - 1.0) < 1e-6)) {
        printf("  Kerbin's air 12 km above a 12 km sea is %.7f kg/m3, expected"
               " 0.3435510: the column was weighed in the typed g at the MEAN radius"
               " instead of the field at the datum (atmo.h:datumGravity, #697)\n", rho12);
        failures++;
    }
    printf("  %s: its air is weighed in g %.4f at the datum, not the typed %.4f\n",
           tmp.c_str(), wda.gravity, b->g);
    if(check_datum(tmp.c_str(), wet)) { failures++; }
    printf("  %s: a 12 km sea lifts Kerbin's shell with the datum and holds the "
           "orbit cut at %.0f m\n", tmp.c_str(), dry_cut);
}

/* #739 moved which datum figure a file states, and one branch of that inversion has no
   shipped example to prove it: on an EXPONENTIAL column rho0 = p0 / (g H) with the gas
   cancelling out (the datum T the H implies is g H M / R, so M drops of p0 = rho0 (R/M) T0),
   and that g is the DATUM sphere's (atmo.h:datumGravity, #697) rather than the typed one.
   Every shipped air body authors sea_level 0, so there the two g are one number and the
   factor is invisible -- exactly the kind of branch a wrong variable walks through with
   every file still green. So: raise an exponential world's sea. Since #625 made every
   KSP air body a table, the fixture below puts Duna's old H 13.6 km back to have an
   exponential to break. A 12 km sea on Duna is
   7.1 % less g at the datum (its radius is only 320 km), so the same authored 600.372 Pa
   resolves to air 7.6 % denser there, and the check has teeth (the last lines assert that the typed-g answer really is
   a different number, so this test cannot rot into a tautology if someone later raises a
   sea to 0). */
static void check_exponential_anchor() {
    std::ifstream f("res/systems/ksp_system.json");
    if(!f.is_open()) {
        printf("  cannot open res/systems/ksp_system.json (run from the repo root)\n");
        failures++;
        return;
    }
    nlohmann::json j = nlohmann::json::parse(f);
    double p0 = 0.0, H = 0.0, gt = 0.0, r = 0.0;
    bool patched = false;
    for(auto &bv : j["bodies"]) {
        if(bv.value("name", std::string()) != "Duna") { continue; }
        nlohmann::json &at = bv["surface"]["atmosphere"];
        p0 = at["sea_level_pressure"].get<double>();
        if(at.contains("scale_height")) {
            H = at["scale_height"].get<double>();
        } else {
            // #625 made every air body in this file a layer table, and this check is
            // about the EXPONENTIAL branch -- so the fixture makes Duna one again, with
            // the H it used to author. A file with both profiles refuses at load, so
            // the table has to go, not just be ignored.
            at.erase("layers");
            H = 13600.0;
            at["scale_height"] = H;
        }
        gt = bv["g"].get<double>();
        r = bv["radius"].get<double>();
        bv["surface"]["sea_level"] = 12000.0;
        patched = true;
    }
    if(!patched || !(p0 > 0.0)) {
        printf("  res/systems/ksp_system.json: no exponential air body to anchor\n");
        failures++;
        return;
    }
    std::filesystem::create_directories("tmp");
    const std::string tmp = "tmp/test_scenarios_exponential_sea.json";
    {
        std::ofstream o(tmp);
        o << j.dump();
    }
    System wet;
    try {
        wet = load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        printf("  an exponential body with a 12 km sea fails to load: %s\n", e.what());
        failures++;
        return;
    }
    TerrainBody *b = find_body(wet, "Duna");
    if(b == nullptr) {
        printf("  %s: Duna vanished from the mutated system\n", tmp.c_str());
        failures++;
        return;
    }
    const AtmosphereParams &a = b->surface.atmosphere;
    const double g_datum = datumGravity(gt, r, b->seaLevelRadius());
    const double want = p0 / (g_datum * H);
    const double typed = p0 / (gt * H);
    if(!(std::fabs(a.sea_level_density / want - 1.0) < 1e-12)) {
        printf("  Duna's air at a raised datum resolves %.8f kg/m3, expected %.8f from"
               " p0 %.4f / (g %.5f at the datum * H %.0f): the anchor was inverted with"
               " the TYPED g, or not inverted at all (#739)\n",
               a.sea_level_density, want, p0, g_datum, H);
        failures++;
    }
    if(!(std::fabs(want / typed - 1.0) > 1e-4)) {
        printf("  %s: a 12 km sea moves Duna's datum g by %.2g%% -- this check is"
               " measuring nothing\n", tmp.c_str(), 100.0 * (want / typed - 1.0));
        failures++;
    }
    printf("  %s: Duna's authored %.3f Pa over a 12 km sea resolves %.6f kg/m3 at its"
           " datum; the typed g would have read %.6f\n",
           tmp.c_str(), p0, a.sea_level_density, typed);
}

/* A body's gravity, stated twice (#689). The typed `g` weighs that body's air column
   (#683); its mass is the pull every arc about it answers to. The loader compares the
   two for a body with air and refuses past atmo.h:kAtmoGravityTol, because a file
   the air reads both: g resolves the datum density (rho0 = p0/(g H), #739) and weighs
   the 1/r^2 shape of the whole column, while the hull falling through that air is
   pulled by G m / r^2. Equal
   at the datum by construction, the two only part company aloft, where the column
   multiplies the gap -- ln p spans 10.7 scale heights across Kerbin's air, so a 5 %
   typed g is 41 % off on the density at its 85 km edge.
   Both edges pinned: 5 % refuses and NAMES the constant; 1 % loads. The second half is
   not padding -- the shipped worlds sit 0.01-0.03 % apart and Pluto 0.62 %, because both
   of its published figures carry three significant digits. A rule that refused those is
   a rule the catalogue would work around. */
static void check_gravity_pair() {
    std::ifstream f("res/systems/ksp_system.json");
    if(!f.is_open()) {
        printf("  cannot open res/systems/ksp_system.json (run from the repo root)\n");
        failures++;
        return;
    }
    const nlohmann::json base = nlohmann::json::parse(f);
    std::filesystem::create_directories("tmp");
    auto kerbin_g = [&](double mult) {
        nlohmann::json j = base;
        for(auto &bv : j["bodies"]) {
            if(bv.value("name", std::string()) != "Kerbin") { continue; }
            bv["g"] = bv["g"].get<double>() * mult;
        }
        return j.dump();
    };
    expect_refuses("gravity_off_5pc", kerbin_g(1.05), "kAtmoGravityTol");

    const std::string path = "tmp/test_scenarios_gravity_1pc.json";
    {
        std::ofstream o(path);
        o << kerbin_g(1.01);
    }
    System sys;
    try {
        sys = load_system(path.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        printf("  %s: Kerbin typed 1%% off its own implied field failed to load: %s\n",
               path.c_str(), e.what());
        failures++;
        return;
    }
    const TerrainBody *ker = find_body(sys, "Kerbin");
    if(ker == nullptr || !(ker->surface.atmosphere.sea_level_density > 0.0)) {
        printf("  %s: Kerbin loaded without its air\n", path.c_str());
        failures++;
        return;
    }
    printf("  %s: a body typed 1%% off G*mass/radius^2 loads with its air; 5%% refuses\n",
           path.c_str());
}

/* A fixture whose note says "the shipped bodies, verbatim, plus ONE field" is a copy, and
   a copy drifts silently. It did: between #683 and #686, 245-kerbin-sea.json still carried
   Kerbin's old 5500 m scale height and 70 km top while the shipped world moved to a level
   table, and e2e/cases/flight/245 stayed GREEN -- because it was testing an 188 K Kerbin
   that no longer exists, with air pins (188.0 K / 66095 Pa / 275.0 m/s) that matched the
   stale copy rather than the game. The case could not see it; only a pin against the real
   file can. Compare the air profile of every body such a fixture carries. */
static void check_fixture_air() {
    static const char *fixtures[] = {
        "e2e/fixtures/systems/245-kerbin-sea.json",   // Kerbol, Kerbin, Moho, Duna
        "e2e/fixtures/systems/412-moho-sea.json",     // Kerbol, Moho
    };
    for(const char *path : fixtures) {
        std::ifstream fa(path), fs("res/systems/ksp_system.json");
        if(!fa.is_open() || !fs.is_open()) {
            printf("  cannot open %s or res/systems/ksp_system.json (run from the repo root)\n",
                   path);
            failures++;
            continue;
        }
        const nlohmann::json fix = nlohmann::json::parse(fa);
        const nlohmann::json shipped = nlohmann::json::parse(fs);
        int drift = 0;
        for(const nlohmann::json &fb : fix["bodies"]) {
            const std::string name = fb.value("name", std::string());
            const nlohmann::json *sb = nullptr;
            for(const nlohmann::json &b : shipped["bodies"]) {
                if(b.value("name", std::string()) == name) { sb = &b; break; }
            }
            if(sb == nullptr) {
                printf("  %s: %s is not a shipped body -- the note claims verbatim copies\n",
                       path, name.c_str());
                drift++;
                continue;
            }
            const bool fx_air = fb.contains("surface") && fb["surface"].contains("atmosphere");
            const bool sh_air = sb->contains("surface") && (*sb)["surface"].contains("atmosphere");
            if(fx_air != sh_air) {
                printf("  %s: %s has %s atmosphere block, the shipped body has %s\n",
                       path, name.c_str(), fx_air ? "an" : "no", sh_air ? "an" : "no");
                drift++;
                continue;
            }
            if(!fx_air) { continue; }
            const nlohmann::json &fxa = fb["surface"]["atmosphere"];
            const nlohmann::json &sha = (*sb)["surface"]["atmosphere"];
            for(const char *key : {"sea_level_pressure", "height", "scale_height"}) {
                if(fxa.value(key, -1.0) != sha.value(key, -1.0)) {
                    printf("  %s: %s atmosphere %s is %s, shipped is %s\n", path, name.c_str(),
                           key, fxa.contains(key) ? fxa[key].dump().c_str() : "absent",
                           sha.contains(key) ? sha[key].dump().c_str() : "absent");
                    drift++;
                }
            }
            // composition is the one thing these fixtures may legitimately leave out: an
            // omitted mix resolves to dry air, which is what the shipped Kerbin authors.
            const bool fx_layers = fxa.contains("layers"), sh_layers = sha.contains("layers");
            if(fx_layers != sh_layers
               || (fx_layers && fxa["layers"].size() != sha["layers"].size())) {
                printf("  %s: %s authors %d air levels, shipped %d\n", path, name.c_str(),
                       (int)(fx_layers ? fxa["layers"].size() : 0),
                       (int)(sh_layers ? sha["layers"].size() : 0));
                drift++;
                continue;
            }
            if(!fx_layers) { continue; }
            for(size_t i = 0; i < fxa["layers"].size(); ++i) {
                const nlohmann::json &x = fxa["layers"][i], &y = sha["layers"][i];
                if(x != y) {
                    printf("  %s: %s level %zu is %s, shipped %s\n", path, name.c_str(), i,
                           x.dump().c_str(), y.dump().c_str());
                    drift++;
                }
            }
        }
        if(drift) {
            printf("  %s: %d field(s) drifted from res/systems/ksp_system.json. The note "
                   "promises the shipped bodies plus one sea_level; a fixture testing a world "
                   "the game no longer ships tests nothing.\n", path, drift);
            failures++;
        } else {
            printf("  %s: air profile matches the shipped bodies it copies\n", path);
        }
    }
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
    const ScenarioDef *cruise = nullptr, *trim = nullptr;
    try {
        low  = scenario_by_name("flying-low");
        high = scenario_by_name("flying-high");
        slow = scenario_by_name("flying-slow");
        cruise = scenario_by_name("flying-cruise");
        trim = scenario_by_name("flying-trim");
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
    /* flying-trim (#693) is flying-cruise with the speed keyed to the airframe instead
       of to gravity, so the two must agree on everything but the speed law -- that is
       what makes an A/B between them mean something. And its load must be a WING's
       number: 50.9 kg/m^2 is q/g for skiff_trim, whose lift area and mass are both of
       order 10 m^2 and 10^2 kg. Under 1 or over 10^4 is a value that arrived in the
       wrong slot, not a hull anyone ships. */
    if(trim->speed_frac != 0.0 || !(trim->speed_load > 1.0 && trim->speed_load < 1e4)) {
        printf("  flying-trim is not load-keyed (speed_frac %g, speed_load %g): the "
               "point of this bed is that gravity does not set its speed\n",
               trim->speed_frac, trim->speed_load);
        failures++;
    }
    if(trim->alt_frac != cruise->alt_frac || trim->atmo_frac != cruise->atmo_frac
       || trim->polar != cruise->polar || trim->eq_plane != cruise->eq_plane) {
        printf("  flying-trim is not flying-cruise's height band (alt_frac %g vs %g): "
               "the pair is only an A/B if the air they spawn in is the same\n",
               trim->alt_frac, cruise->alt_frac);
        failures++;
    }
    /* A bed carries at most ONE speed key, and whichever it is needs a circular speed in
       air to describe it: speed_frac is a fraction of a circular speed, and speed_load
       turns a lift-area load into an airspeed through rho and g at the bed. The table is
       positional and 14 fields wide, so a row can hand a value to the wrong slot and
       still compile -- this refuses that SHAPE rather than bounding any one number, since
       a fraction over 1 or a load under 1 Pa could both be a bed someone wants tomorrow
       (the bounds this used to carry are what they would refuse). */
    for(size_t i = 0; i < scenario_count(); i++) {
        const std::string sname = scenario_name_at(i);
        const ScenarioDef *d = scenario_by_name(sname);
        const int keys = (d->speed_frac > 0.0 ? 1 : 0) + (d->speed_load > 0.0 ? 1 : 0);
        if(keys == 0) { continue; }
        if(keys > 1 || d->ell_phase >= 0 || d->esc_frac > 0.0 || !d->atmo_frac
           || d->on_pad || d->abs_r > 0.0) {
            printf("  scenario '%s' sets speed_frac = %g / speed_load = %g on a bed "
                   "neither can describe (want at most one key and ell_phase < 0, "
                   "esc_frac 0, atmo_frac, not on_pad, no abs_r -- got ell_phase %d, "
                   "esc_frac %g, atmo_frac %d, on_pad %d, abs_r %g)\n",
                   sname.c_str(), d->speed_frac, d->speed_load, d->ell_phase,
                   d->esc_frac, d->atmo_frac ? 1 : 0, d->on_pad ? 1 : 0, d->abs_r);
            failures++;
        }
    }
    printf("  flying beds: low/high circular (entry rigs), slow at %.2f x the "
           "air-relative circular speed, cruise at %.4f x it, trim at %.1f kg/m^2 of "
           "lift-area load (#693)\n", slow->speed_frac, cruise->speed_frac,
           trim->speed_load);
}

/* A placed bed (--startship lat=/lon=): the bed turns so its circle passes over the
   asked-for point, tilting its plane as little as it can and keeping its velocity
   across the circle. Hand-computed on an equatorial bed -- position +X, velocity +Y,
   plane normal +Z -- where every expected number below is arithmetic, not a re-run of
   bedTurnTo. A point that needs no move, three points along the bed's own circle
   (a quarter turn, a half turn, the point itself), three points 30 deg out of the
   plane (at the bed's azimuth, mirrored below it, and 90 deg around) and a point
   over each of the bed's own poles -- the degenerate case with no node line to tilt
   about -- are the shapes a placement can ask for. */
static void check_bed_turn() {
    const double s30 = 0.5, c30 = std::sqrt(3.0) / 2.0;
    const glm::dvec3 X(1.0, 0.0, 0.0), Y(0.0, 1.0, 0.0), Z(0.0, 0.0, 1.0);
    struct Row {
        const char *what;
        glm::dvec3 rhat, vhat, want;
        double tilt;          // expected plane tilt [deg]
        glm::dvec3 vhat_out;  // expected heading
    };
    const Row rows[] = {
        // The bed is already over the point: nothing moves, so the heading is the
        // heading. This is the row that fails if the phase solve drifts on its own
        // zero-crossing.
        {"already there",  X, Y, X, 0.0,  Y},
        // A quarter turn along the bed's own circle: still no tilt, heading follows.
        {"along the ring", X, Y, Y, 0.0, -X},
        // Half a turn along the bed's own circle: still no tilt, and the phase solve
        // is asked for atan2(0, -1) -- the half-turn it must not answer 0 to.
        {"antipode",       X, Y, -X, 0.0, -Y},
        // 30 deg out of the plane, at the azimuth the bed already sits at: the node
        // line want x n0 is -Y, so the plane tilts 30 deg about it and carries +X
        // onto want by itself -- phase 0, heading still +Y (and still across the
        // circle, since +Y is perpendicular to want).
        {"30 deg over rhat", X, Y, glm::dvec3(c30, 0.0, s30), 30.0, Y},
        // The mirror below the plane: the node line flips to +Y, the tilt is -30 deg
        // (reported as its magnitude), and again the tilt alone reaches the point.
        {"-30 over rhat",  X, Y, glm::dvec3(c30, 0.0, -s30), 30.0, Y},
        // The same 30 deg of tilt 90 deg around the ring: the node line is now +X, so
        // the tilt carries vhat (not rhat) onto want, and the 90 deg phase turn takes
        // it to -X.
        {"30 deg, 90 around", X, Y, glm::dvec3(0.0, c30, s30), 30.0, -X},
        // Over the bed's own poles: no node line, so the plane turns 90 deg about
        // rhat and keeps the meridian the bed already sat in.
        {"north pole",     X, Y, Z,  90.0, -X},
        {"south pole",     X, Y, -Z, 90.0, -X},
    };
    for(const Row &r : rows) {
        double tilt = -1.0;
        const glm::dvec3 vh = bedTurnTo(tilt, r.rhat, r.vhat, r.want);
        // The contract the spawn relies on, checked for every row: the heading is a
        // unit vector across the circle at the point, so the tilted bed is still a
        // circular bed of the same radius and speed.
        const double len = glm::length(vh);
        const double radial = glm::dot(vh, r.want);
        const double want_v = glm::length(vh - r.vhat_out);
        if(std::fabs(tilt - r.tilt) > 1e-9 || std::fabs(len - 1.0) > 1e-12
           || std::fabs(radial) > 1e-12 || want_v > 1e-12) {
            printf("  bed turn %s: tilt %.6f (want %.1f) |v| %.9f radial %.9f "
                   "v_out (%.6f, %.6f, %.6f) want (%.6f, %.6f, %.6f)\n",
                   r.what, tilt, r.tilt, len, radial,
                   vh.x, vh.y, vh.z, r.vhat_out.x, r.vhat_out.y, r.vhat_out.z);
            failures++;
        }
    }
    printf("  bed turn: %d hand-computed turns pinned, the biggest a %.0f deg plane "
           "tilt over a pole\n", (int) (sizeof rows / sizeof rows[0]), 90.0);
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
    check_bed_turn();
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
    check_wind();
    check_gravity_pair();
    check_raised_sea();
    check_exponential_anchor();
    check_fixture_air();
    if(failures) {
        printf("test_scenarios: %d FAILED\n", failures);
        return 1;
    }
    printf("test_scenarios: OK\n");
    return 0;
}
