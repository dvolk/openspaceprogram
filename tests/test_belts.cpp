// test_belts.cpp -- the system JSON root "belts" array (src/system.cpp).
//
// Debris belts are named annuli orbiting the STAR, drawn on the orbital maps
// (gameui.cpp drawDebrisBelts). The entry shape is deliberately the same as a
// body's "surface.rings" band (name/inner/outer, metres): a belt is that
// object with a different centre, so there is one annulus vocabulary to learn.
// Unlike rings, a malformed belt THROWS rather than being skipped: belts are
// few and hand-authored, and a silently dropped band is invisible in-game.
// Pinned here:
//   - the shipped showcase systems carry both bands, in the right order,
//     with sane radii that reach the loader,
//   - ksp_system's outer band starts just inside its outermost rail's
//     apoapsis, so that body grazes the belt rather than living in it,
//   - "belts" is optional: a system without it loads with none,
//   - malformed belts throw, naming the belt.
//   - the #687 surface thermal bounds reject a typo but not an absence, and the
//     two absences are spelled differently: albedo absent is the -1 sentinel, an
//     authored 0.0 is a reading (#726).
//
// Build & run (from repo root): see Makefile ($(TESTDIR)/test_belts).

#include "system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        std::printf("FAIL cannot open %s (run from the repo root)\n", path);
        ++g_failures;
        return nlohmann::json::object();
    }
    return nlohmann::json::parse(f);
}

// Load `path`, expecting success; empty vector on a throw (the caller's
// checks then fail, naming the case).
static std::vector<BeltParams> load_belts(const std::string &path) {
    try {
        System sys = load_system(path.c_str(), nullptr, nullptr);
        return sys.belts;
    } catch(const std::exception &e) {
        std::printf("FAIL %s: unexpected throw: %s\n", path.c_str(), e.what());
        ++g_failures;
        return {};
    }
}

// Write a mutated system (tmp/), load it, expect a throw naming msg_needle.
static void expect_reject(const nlohmann::json &j, const char *tag,
                          const char *msg_needle) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_belts_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    // Copy the message out of the catch: e.what() points into the exception,
    // so a const char* kept past its destruction reads freed memory and the
    // needle match becomes luck of the allocator.
    bool threw = false;
    std::string what;
    try {
        load_system(tmp.c_str(), nullptr, nullptr);
    } catch(const std::exception &e) {
        threw = true;
        what = e.what();
    }
    check(threw && what.find(msg_needle) != std::string::npos, tag);
    std::remove(tmp.c_str());
}

// Write a mutated system (tmp/), load it expecting SUCCESS, and hand the caller
// the loaded System so it can read what the loader KEPT -- which is how a
// default-value law gets pinned, not just the refusal of a bad value.
static void expect_loaded(const nlohmann::json &j, const char *tag,
                          const std::function<void(const System &)> &fn) {
    std::filesystem::create_directories("tmp");
    const std::string tmp = std::string("tmp/test_belts_") + tag + ".json";
    std::ofstream o(tmp);
    o << j.dump();
    o.close();
    try {
        System sys = load_system(tmp.c_str(), nullptr, nullptr);
        fn(sys);
        for(TerrainBody *b : sys.bodies) { delete b; }
    } catch(const std::exception &e) {
        std::printf("FAIL %s: unexpected throw: %s\n", tag, e.what());
        ++g_failures;
    }
    std::remove(tmp.c_str());
}

// A two-band array in the authored form.
static nlohmann::json two_bands() {
    return nlohmann::json::array({
        { { "name", "asteroid belt" }, { "inner", 2.856e10 }, { "outer", 4.488e10 } },
        { { "name", "Kuiper belt" },  { "inner", 4.08e11 },  { "outer", 6.8e11 } },
    });
}

// Apoapsis [m] of the outermost rail about the star, from the authored JSON:
// Kepler III inverts "orb_ang_speed" the same way the loader does (system.cpp
// uses the same G), so a retune of the rail moves this with it.
static double outermost_apoapsis(const nlohmann::json &doc) {
    const double G = 6.674e-11;
    const std::string star = doc["bodies"][0]["name"].get<std::string>();
    const double mu = G * doc["bodies"][0]["mass"].get<double>();
    double apo = 0.0;
    for(const nlohmann::json &b : doc["bodies"]) {
        if(b.value("orbits", "") != star || !b.contains("inertial")) { continue; }
        const double n = b["inertial"].value("orb_ang_speed", 0.0);
        if(n <= 0.0) { continue; }
        const double a = std::cbrt(mu / (n * n));
        apo = std::max(apo, a * (1.0 + b["inertial"].value("ecc", 0.0)));
    }
    return apo;
}

int main() {
    // --- the shipped systems carry both bands, in draw order --------------
    for(const std::string &slug : { std::string("ksp_system"),
                                    std::string("solar_system"),
                                    std::string("solar_system_measured"),
                                    std::string("solar_system_named"),
                                    std::string("solar_system_full") }) {
        const std::string path = "res/systems/" + slug + ".json";
        const std::vector<BeltParams> belts = load_belts(path);
        check(belts.size() == 2, ("two belts: " + slug).c_str());
        /* #794 step 5: the shipped corpus authors ground phase lags now -- ksp_system only, and
           only where utils/ground_model.py's thermal parameter says the flattening and the
           afternoon are worth a knob. The audit still reads the FILES rather than a loaded
           System, for the same reason it always did: a body gaining a tau is a physics decision
           (its map stops peaking under the star, and its dossier's `ground swing` row starts
           quoting ends the map can no longer reach -- #830), and it should arrive as an edit to
           this list, not as a quiet generator change. The four solar corpora stay unlagged
           because make_solar_system.py has not run the pass yet: that is a decision on the
           ticket, and if one of them grows a tau first, this is where it reddens. */
        {
            // Name -> the tau the pass prices for it. Four decimals inside a 1e-3 window: the
            // digits themselves are already pinned exactly by `gen_systems.py --check`, which
            // regenerates the file from the formula, so this audit's job is the ROSTER -- which
            // bodies carry the knob at all, and what travels beside it. A tighter epsilon here
            // would only make a harmless refit of an anchor redder than a wrong world.
            const std::map<std::string, double> lagged = {
                {"Minmus", 0.4243}, {"Ike", 0.2902}, {"Dres", 1.1030}, {"Vall", 2.0185},
                {"Tylo", 0.9762},   {"Bop", 0.5779},  {"Pol", 0.4795},
            };
            const nlohmann::json doc = read_json(path.c_str());
            size_t seen = 0;
            for(const nlohmann::json &b : doc.value("bodies", nlohmann::json::array())) {
                const nlohmann::json sv = b.value("surface", nlohmann::json::object());
                const std::string name = b.value("name", "?");
                const auto it = slug == "ksp_system" ? lagged.find(name) : lagged.end();
                if(it == lagged.end()) {
                    check(!sv.contains("lag_tau"),
                          (slug + "/" + name + ": a phase lag nobody priced in the audit").c_str());
                    continue;
                }
                ++seen;
                const double tau = sv.value("lag_tau", -1.0);
                check(tau > 0.0 && tau <= kLagTauMaxPhase,
                      (slug + "/" + name + ": an authored lag reaches the file").c_str());
                check(std::fabs(tau - it->second) < 1e-3,
                      (slug + "/" + name + ": the authored tau is what the thermal parameter"
                       " prices").c_str());
                check(!sv.contains("atmosphere"),
                      (slug + "/" + name + ": a lag was authored under a sky, and the skin"
                       " derivation does not apply there").c_str());
                /* The lag and the even share are one surface law, and the pass's refusal rule
                   reads the share's provenance before it writes a tau (share_lag_conflict). A
                   lagged body with no share of record would be a body whose flattening was
                   decided by nobody -- so the audit asks that the other half of the pair is
                   there and named too. */
                check(sv.value("even_share", -1.0) >= 0.0 && sv.contains("even_share_src"),
                      (slug + "/" + name + ": a lagged body carries its even share and says"
                       " where it came from").c_str());
                /* The Gamma is INVENTED -- no source in this corpus states a thermal inertia --
                   and the file has to say so next to the number it priced, the same discipline
                   a DERIVED anchor carries (#722). A provenance string that could go quiet is a
                   promise the loader would stop keeping. */
                const std::string src = sv.value("lag_tau_src", "");
                check(src.find("INVENTED") != std::string::npos,
                      (slug + "/" + name + ": the invented Gamma announces itself in the"
                       " file").c_str());
            }
            if(slug == "ksp_system") {
                check(seen == lagged.size(),
                      (slug + ": every body the audit names carries its lag ("
                       + std::to_string(seen) + "/" + std::to_string(lagged.size()) + ")").c_str());
            }
        }
        if(belts.size() != 2) { continue; }
        check(belts[0].name == "asteroid belt", ("inner band named: " + slug).c_str());
        check(belts[1].name == "Kuiper belt", ("outer band named: " + slug).c_str());
        // Ordered outward, and both outside the star: a band that swallowed
        // the star or sat inside its own sun would draw as a filled disc.
        for(const BeltParams &b : belts) {
            check(b.inner > 0.0 && b.outer > b.inner,
                  ("0 < inner < outer: " + slug + " " + b.name).c_str());
        }
        // The asteroid band must sit inside the Kuiper band, or the two
        // overlap and the map shows one smeared region.
        check(belts[0].outer < belts[1].inner,
              ("bands do not overlap: " + slug).c_str());
    }

    // --- the outer band sits at the edge of the planetary system ----------
    // ksp_system's outermost rail is Eeloo (e 0.26, apo 113.6 Gm). Its band
    // starts just inside that apoapsis, so Eeloo grazes the belt's inner edge
    // the way Neptune sits on the shipped solar system's Kuiper inner edge with
    // Pluto (e 0.25) dipping in. A band starting well inside the outermost
    // orbit reads as that orbit's own neighbourhood, not as the far edge of the
    // system -- which is what 72 Gm did, putting Eeloo's whole ellipse inside.
    {
        const std::string path = "res/systems/ksp_system.json";
        const std::vector<BeltParams> belts = load_belts(path);
        const double apo = outermost_apoapsis(read_json(path.c_str()));
        if(belts.size() == 2) {
            check(belts[1].inner < apo && belts[1].inner > 0.9 * apo,
                  "ksp Kuiper inner edge grazes Eeloo's apoapsis");
            check(belts[1].outer > 1.2 * apo,
                  "ksp Kuiper band reaches beyond Eeloo");
        }
    }

    // --- "belts" is optional ----------------------------------------------
    // old_system is the frozen legacy fixture: no belts, and it must still
    // load (the map simply draws no bands).
    {
        const std::vector<BeltParams> belts = load_belts("res/systems/old_system.json");
        check(belts.empty(), "system without \"belts\" loads with none");
    }

    // --- the authored form reaches the loader ------------------------------
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = two_bands();
        const std::string tmp = "tmp/test_belts_authored.json";
        std::ofstream(tmp) << j.dump();
        const std::vector<BeltParams> belts = load_belts(tmp);
        check(belts.size() == 2, "authored belts load");
        if(belts.size() == 2) {
            check(belts[0].name == "asteroid belt" && belts[0].inner == 2.856e10
                  && belts[0].outer == 4.488e10, "belt 0 fields survive");
            check(belts[1].name == "Kuiper belt" && belts[1].inner == 4.08e11
                  && belts[1].outer == 6.8e11, "belt 1 fields survive");
        }
        std::remove(tmp.c_str());
    }

    // --- malformed data throws --------------------------------------------
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = two_bands()[0];   // one object, not an array
        expect_reject(j, "non-array belts rejected", "must be an array");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array({ { { "inner", 1.0e10 },
                                               { "outer", 2.0e10 } } });
        expect_reject(j, "nameless belt rejected", "non-empty \"name\"");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "asteroid belt" }, { "inner", "28560000000" },
                { "outer", 4.488e10 } } });
        expect_reject(j, "string radius rejected", "numeric \"inner\"");
    }
    {   // the r1/r2 spelling: a plausible typo, and it must not load as a
        // zero-radius band
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "asteroid belt" }, { "r1", 2.856e10 },
                { "r2", 4.488e10 } } });
        expect_reject(j, "r1-r2 spelling rejected", "numeric \"inner\"");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "inside-out belt" }, { "inner", 4.488e10 },
                { "outer", 2.856e10 } } });
        expect_reject(j, "inverted radii rejected", "inside-out belt");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["belts"] = nlohmann::json::array(
            { { { "name", "zero belt" }, { "inner", 0.0 }, { "outer", 4.488e10 } } });
        expect_reject(j, "zero inner rejected", "zero belt");
    }

    /* #687: the surface's thermal fields. ABSENT is legal (it means "no source" and the
       readouts dash -- the load of old_system.json above proves that half, since it
       authors no albedo at all). PRESENT-but-silly is a data bug: a negative temperature
       on a star is a T^4 typo that would light its whole system at the wrong flux, and an
       albedo of 1 or more asks a body to reflect more light than reaches it. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["temperature"] = -5.0;   // the star
        expect_reject(j, "negative surface temperature rejected",
                      "\"surface.temperature\"=");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["albedo"] = 1.2;
        expect_reject(j, "albedo past 1 rejected", "\"surface.albedo\"=");
    }
    {
        /* The bound is a typo filter, and this is the case it is sized for: a
           decimal-point mistake on a stellar temperature. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["temperature"] = 2.0e6;
        expect_reject(j, "absurd surface temperature rejected",
                      "\"surface.temperature\"=");
    }

    /* #726: ABSENT has its own spelling, and it is not a legal reading. An albedo of
       0.0 is a body that absorbs everything the star sends -- the HOTTEST case in the
       game -- so it must load as 0.0 and stay distinguishable from a body nobody
       sourced, which reads kAlbedoUnauthored (-1, terragen.h). The loader's bound is
       [0,1), so a file can never author its own absence: that is what stops the two
       states merging by accident again. Before this, an authored 0.0 printed the same
       `albedo=- bb_T=-` row as no key at all. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["albedo"] = -1.0;
        expect_reject(j, "authored sentinel albedo rejected", "\"surface.albedo\"=");
    }
    {
        /* Both halves in one load: old_system.json authors no albedo anywhere, so the
           star gains an authored 0.0 that must survive, while every other body stays
           unsourced and must read the sentinel rather than 0.0. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        const std::string star = j["bodies"][0]["name"].get<std::string>();
        j["bodies"][0]["surface"]["albedo"] = 0.0;
        expect_loaded(j, "authored zero albedo", [&](const System &sys) {
            const TerrainBody *s = sys.find(star);
            check(s != nullptr && s->surface.bond_albedo == 0.0,
                  "authored albedo 0.0 loads as a reading");
            bool all_sentinel = true;
            for(const TerrainBody *b : sys.bodies) {
                if(b->name != star && b->surface.bond_albedo != kAlbedoUnauthored) {
                    all_sentinel = false;
                }
            }
            check(all_sentinel, "unsourced bodies read the sentinel, not 0.0");
        });
    }

    /* #724 rung D: the air-follow knob, on the same sentinel law. Absent means NO coupling;
       0.0 is a legal reading ("this air does not follow its ground"); the sentinel cannot be
       authored. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["atmosphere"]["air_follow"] = 1.5;
        expect_reject(j, "air_follow past 1 rejected", "air_follow");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["atmosphere"]["air_follow"] = kFollowUnauthored;
        expect_reject(j, "authored sentinel air_follow rejected", "air_follow");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        expect_loaded(j, "air_follow absent reads the sentinel", [&](const System &sys) {
            bool all_sentinel = true;
            for(const TerrainBody *b : sys.bodies) {
                if(b->surface.atmosphere.enabled
                   && b->surface.atmosphere.air_follow != kFollowUnauthored) {
                    all_sentinel = false;
                }
            }
            check(all_sentinel, "no air in the fixture authors a coupling");
        });
    }

    /* #749: every ground in the catalogue now carries a swing -- by a fitted pair, by a
       borrowed analogue, or by a rule of ours -- so the knob alone no longer says which. The
       provenance rides in the file beside it (surface.even_share_src) and the dossier prints it
       next to the two ends it explains. Two guards, both about the pairing rather than the
       value: a provenance that is a number explains a figure with a figure, and a provenance
       with no knob describes an answer the file never gave. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["albedo_src"] = "geom 0.68, q=0.681";
        expect_reject(j, "albedo provenance with no albedo rejected", "albedo_src");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["albedo"] = 0.5;
        j["bodies"][1]["surface"]["albedo_src"] = 2.5;
        expect_reject(j, "numeric albedo provenance rejected", "albedo_src");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["even_share"] = 0.5;
        j["bodies"][1]["surface"]["even_share_src"] = 3.0;
        expect_reject(j, "numeric provenance rejected", "even_share_src");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["even_share_src"] = "DEFAULT airless floor";
        expect_reject(j, "provenance with no knob rejected", "even_share_src");
    }
    /* #794: the phase lag's own guards. Seven shipped bodies author one now, so the loader is the
       only thing standing between a units mistake and a world a player loads -- and the mistake it
       exists to catch is a unit one. A fact page that says "the daily max lands at 14:00" invites
       someone to author 14, and 14 radians is a legal-looking number. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 14.0;      // hours, not radians of phase
        expect_reject(j, "a lag longer than its own day has to say where it came from",
                      "needs surface.lag_tau_src");
    }
    {
        /* The provenance rule is unconditional (#794 step 5 review). The first cut asked for the
           sentence only past one rotation, which left the number that actually looks like a
           mistake walking free: 5.0 is a stated afternoon divided by the wrong thing, and it would
           have painted a +64 deg peak with nobody on record claiming it. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 5.0;
        expect_reject(j, "an unexplained lag fails whatever its size",
                      "needs surface.lag_tau_src");
    }
    {
        /* The numeric bound moved off one rotation (#794 step 5: a cold fast world's thermal
           parameter asks for more, and the row is tested to kLagTauRowCeil), so 14 rad is no
           longer the number that fails -- it is the ABSENCE of a sentence that fails. With a
           sentence it loads, because the loader's job is the range and the units claim, not a
           physics veto. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 14.0;
        j["bodies"][1]["surface"]["lag_tau_src"] = "test fixture, past one rotation";
        j["bodies"][1]["surface"]["even_share"] = 0.05;
        expect_loaded(j, "a lag past one rotation loads once it is explained",
                      [&](const System &sys) {
            check(sys.bodies[1]->surface.lag_tau == 14.0,
                  "tau past 2pi is inexpressible no more");
        });
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 1e9;
        expect_reject(j, "the range still refuses nonsense", "radians of phase");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = -1.0;      // the C++ sentinel, pasted into JSON
        expect_reject(j, "a file cannot author its own absence", "radians of phase");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = "1 rad";
        expect_reject(j, "the lag itself must be a number",
                      "\"surface.lag_tau\" must be a number");
    }
    {
        /* A banded world has no radiating skin (groundtemp.h), so a lag there is a knob with
           nothing to slow -- and the map would honour it anyway, so the dossier would print a
           `level lag` for a phantom. The pricing pass already refuses to author one; this is
           the same rule at the door, for the hand-authored file. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["bands"] = true;
        j["bodies"][1]["surface"]["lag_tau"] = 1.0;
        j["bodies"][1]["surface"]["lag_tau_src"] = "test fixture on a gas giant";
        expect_reject(j, "a banded world cannot author a ground lag",
                      "on a banded world");
    }
    {
        /* The zero matters: `lagTauActive` (tau > 0) would let a STATED 0.0 onto a giant, and
           the dossier would print the loudest version of the same phantom -- "this level peaks
           at noon", a claim about a world with no surface to peak on. The rule is about
           authored, not about nonzero. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["bands"] = true;
        j["bodies"][1]["surface"]["lag_tau"] = 0.0;
        expect_reject(j, "a banded world cannot author a zero lag either",
                      "on a banded world");
    }
    {
        /* And a lag needs the law it slows. lag_tau is the response knob of the one-knob law
           that even_share prices, so on a body that states no share it acts on nothing: the
           map, the air coupling and the atlas all refuse the block, while --solar-log would
           still print the number. A file may not carry a knob no readout can show. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 1.0;
        j["bodies"][1]["surface"]["lag_tau_src"] = "test fixture, no share to slow it";
        expect_reject(j, "a lag with no even share has nothing to lag",
                      "needs a \"surface.even_share\"");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 1.0;
        j["bodies"][1]["surface"]["lag_tau_src"] = 3.0;
        expect_reject(j, "numeric lag provenance rejected",
                      "\"surface.lag_tau_src\" must be a string");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau_src"] = "via Earth";
        expect_reject(j, "lag provenance with no lag rejected",
                      "explains a knob that surface.lag_tau does not author");
    }
    {
        /* And the positive half: the knob reaches the struct, INCLUDING a stated zero. A loader
           that quietly dropped a 0.0 (`if(lt > 0.0) s.lag_tau = lt;`) would pass every rejection
           above while erasing the one distinction authored.h makes a point of -- that "this world
           peaks at noon" is a fact, and is not the same statement as nobody saying anything. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][1]["surface"]["lag_tau"] = 1.0;
        j["bodies"][1]["surface"]["lag_tau_src"] = "test fixture";
        j["bodies"][1]["surface"]["even_share"] = 0.05;
        j["bodies"][2]["surface"]["lag_tau"] = 0.0;
        j["bodies"][2]["surface"]["even_share"] = 0.05;
        expect_loaded(j, "an authored lag reaches the body", [&](const System &sys) {
            const TerrainBody *b = sys.bodies[1];
            check(lagTauAuthored(b->surface.lag_tau) && b->surface.lag_tau == 1.0,
                  "surface.lag_tau lands as authored");
            check(b->surface.lag_tau_src == "test fixture", "and its provenance travels with it");
            const TerrainBody *z = sys.bodies[2];
            check(lagTauAuthored(z->surface.lag_tau) && z->surface.lag_tau == 0.0,
                  "a stated zero stays a stated zero");
            check(z->surface.lag_tau_src.empty(), "and needs no provenance to be legal");
            bool others_dash = true;
            for(size_t i = 0; i < sys.bodies.size(); i++) {
                if(i == 1 || i == 2) { continue; }
                if(lagTauAuthored(sys.bodies[i]->surface.lag_tau)) { others_dash = false; }
            }
            check(others_dash, "a body nobody priced carries no lag, and not a 0.0 either");
        });
    }
    {
        /* The claim that matters is about the SHIPPED file, so it is pinned against the
           shipped file: the defaults are in it, they are named, and every world the file gives
           a surface gets a knob -- a banded world included, because this file authors a giant's
           surface AT the 1 bar (#689) and a swing there is as good a swing as any. A generator
           that stopped writing the strings, or that let one world's knob die quietly, reddens
           here. */
        nlohmann::json j = read_json("res/systems/ksp_system.json");
        expect_loaded(j, "the shipped catalogue prices every ground it has",
                      [&](const System &sys) {
            int priced = 0, named = 0, grounded = 0;
            for(const TerrainBody *b : sys.bodies) {
                if(!b->isStar()) { ++grounded; }
                if(!evenShareAuthored(b->surface.even_share)) { continue; }
                ++priced;
                if(!b->surface.even_share_src.empty()) { ++named; }
            }
            // EVERY body with a ground, exactly: a swing dying quietly in the generator would
            // otherwise slide under a `>=` while the dossier lost rows nobody noticed.
            check(priced == grounded,
                  "every ground-bearing body in the shipped file carries a swing");
            check(named == priced,
                  "every authored swing in the shipped file says how it was priced");
            const TerrainBody *kerbin = sys.find("Kerbin");
            check(kerbin != nullptr
                      && kerbin->surface.even_share_src.find("via ") != std::string::npos,
                  "Kerbin's knob names the world it is borrowed from");
            /* #749 also converted albedos: the satellite fact sheets state a VISUAL
               GEOMETRIC albedo, and the corpus's own ten (Bond, geometric) pairs set the
               factor. A converted albedo and a reading are the same three digits on screen
               unless the file says which it is, so the marker has to survive the loader and
               the two-hop chain has to stay legible: Minmus borrows Enceladus, and Enceladus
               was converted before it was borrowed. */
            const TerrainBody *minmus = sys.find("Minmus");
            check(minmus != nullptr
                      && minmus->surface.albedo_src.find("converted") != std::string::npos,
                  "a borrowed converted albedo says it was converted");
            const TerrainBody *moho = sys.find("Moho");
            check(moho != nullptr
                      && moho->surface.even_share_src.find(", defaulted")
                             != std::string::npos,
                  "a borrowed knob that was a DEFAULT at its source says so two hops deep");

            /* #759: the atlas prints values only, so the authoring trail is a fact of the
               JSON and these tests are the only thing that reads it back. A generator that
               renames a rung, or drops the second hop of a borrowed figure (#749), reddens
               HERE rather than in a player's dossier row. */
            System sol = load_system("res/systems/solar_system.json", nullptr, nullptr);
            const char *rungs[][2] = {
                {"Mercury", "DEFAULT airless floor"},
                {"Venus", "stated: no range"},
                {"Mars", "least-squares fit to the pair"},
                {"Earth", "fitted from the night end"},
                {"Moon", "solved from a stated range"},
            };
            for(const auto &rung : rungs) {
                const TerrainBody *b = sol.find(rung[0]);
                check(b != nullptr && b->surface.even_share_src == rung[1],
                      (std::string("the shipped file names which rung priced ")
                       + rung[0]).c_str());
            }
            /* The Moon's and Pluto's anchors are not readings at all -- they were solved from a
               stated night/day pair -- so the honesty lives in the temperature's own src string.
               Nothing else in the tree reads that key back. */
            for(const char *name : { "Moon", "Pluto" }) {
                const TerrainBody *b = sol.find(name);
                check(b != nullptr
                          && b->surface.temperature_src.find("SOLVED from the stated range")
                                 != std::string::npos,
                      (std::string(name)
                       + ": a pair-solved anchor says so beside the number, not only the knob")
                          .c_str());
            }
            const TerrainBody *iap = sol.find("Iapetus");
            check(iap != nullptr
                      && iap->surface.albedo_src.rfind("geom ", 0) == 0
                      && iap->surface.albedo_src.find("q=0.") != std::string::npos,
                  "a converted albedo names the geometric reading and the factor it was times");
            const TerrainBody *mun = sys.find("Mun");
            check(mun != nullptr && albedoAuthored(mun->surface.bond_albedo)
                      && mun->surface.albedo_src.find("converted") == std::string::npos,
                  "a borrowed READING does not claim to be a conversion");
            /* #793: the ground's height term, across every world the game actually ships. It is
               the FILE's own lapse times the FILE's own relief, so a body with a lot of both could
               in principle ask for a negative-kelvin summit -- which is not a cold mountain, it is
               two authored numbers that do not fit together (src/groundtemp.h asserts it, and the
               asserts are live in the release build, so a bake would abort in front of a player).
               Walking the corpus here means the file reddens in `make test`. The gate matters as
               much as the number: a banded giant and a rim-only air (Io, Triton) must answer 0.0,
               because their dossiers state the anchor as the whole answer and a height term would
               contradict them. */
            // The enclosing lambda's `sys` is const and `sol` is not, so the list is spelled.
            for(const System *S : std::vector<const System *>{ &sys, &sol }) {
                for(const TerrainBody *b : S->bodies) {
                    const double peak = (double)b->surface.authoredPeak();
                    if(!b->surface.reliefOn()) {
                        check(b->surface.relief_K(peak) == 0.0
                                  && b->surface.relief_K(1.0e6) == 0.0,
                              (b->name + ": no height term, so no summit effect").c_str());
                        continue;
                    }
                    const double off = b->surface.relief_K(peak);
                    check(peak > 0.0 && off < 0.0,
                          (b->name + ": a body whose air lapses has SOME relief effect").c_str());
                    /* The invariant the readouts assert (#793), checked here so a file that does
                       not fit reddens in `make test` instead of aborting a bake: the summit must
                       stay above 0 K, which means the lapse at the peak cannot be deeper than the
                       ground's own NIGHT END. Same rungs the loader checks (stated ground, else
                       the air's datum); a derived black-body anchor needs the star, and every
                       body that can reach that rung has no air, i.e. no relief. */
                    const double anchor = evenShareAuthored(b->surface.even_share)
                        ? groundAnchor_K(b->surface.temperature,
                                         airDatum_K(b->surface.atmosphere), 0.0)
                        : 0.0;
                    if(anchor > 0.0) {
                        const double night = groundTemperature(anchor, b->surface.even_share, 0.0);
                        check(night + off >= 0.0,
                              (b->name + ": the night end carries its own relief -- "
                                        + std::to_string(night + off) + " K at the peak")
                                  .c_str());
                    }
                    std::printf("  relief %-9s peak %6.0f m -> %+.1f K at the summit\n",
                                b->name.c_str(), peak, off);
                }
            }
            /* Jool used to be the one world that refused its knob. It no longer does: a banded
               world's surface IS its 1 bar level, so it takes the same DEFAULT air-mass rule as
               Jupiter, Saturn, Uranus and Neptune. What still has to be true is that the file
               says WHERE the knob came from -- borrowed from Jupiter, whose own was a DEFAULT,
               is a different claim than a fitted pair. */
            const TerrainBody *jool = sys.find("Jool");
            check(jool != nullptr && evenShareAuthored(jool->surface.even_share)
                      && jool->surface.even_share_src.find("via Jupiter") != std::string::npos,
                  "Jool is priced like its four real analogues, and says who it was borrowed from");
        });
    }

    /* #729: the SAME law on the other field, where the damage was worse. Surface::temperature
       used to default to 0.0 and be tested `> 0.0`, so "nobody sourced this" and "0 K" were one
       state -- and on a STAR that state was a luminosity of 0: a system that loads, prints
       dashes everywhere and is dark. Absence is kThermalUnauthored now (authored.h), the loader
       refuses to author it AND refuses 0.0 (0 K is a blank, not a temperature -- a file that
       means cold can say 40 K), and the one site that would raise the sentinel to the fourth
       power is guarded: starLuminosity asserts a non-negative input, so an unguarded hand-off
       there is an abort rather than a dash. */
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["temperature"] = 0.0;
        expect_reject(j, "authoring 0 K is refused", "\"surface.temperature\"=");
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"]["temperature"] = kThermalUnauthored;
        expect_reject(j, "authored sentinel temperature rejected",
                      "\"surface.temperature\"=");
    }
    {
        /* The star lights the system OFF its authored temperature, so the sentinel's whole
           hazard lives here: no temperature must mean "nothing receives insolation" (the
           warning system.cpp prints) and NOT an assert, while an authored one must survive to a
           real luminosity. Both directions, or the guard at system.cpp is unpinned. */
        nlohmann::json j = read_json("res/systems/old_system.json");
        expect_loaded(j, "the fixture star keeps its light", [&](const System &sys) {
            check(sys.root && sys.root->isStar(), "the fixture root is a star");
            check(thermalAuthored(sys.root->surface.temperature)
                  && sys.root->star_luminosity > 0.0,
                  "an authored stellar temperature lights the system");
            bool all_sentinel = true;
            for(const TerrainBody *b : sys.bodies) {
                if(b != sys.root && b->surface.temperature != kThermalUnauthored) {
                    all_sentinel = false;
                }
            }
            check(all_sentinel,
                  "unsourced bodies read the sentinel, not 0 K (old_system states no ground T)");
        });
    }
    {
        nlohmann::json j = read_json("res/systems/old_system.json");
        j["bodies"][0]["surface"].erase("temperature");
        expect_loaded(j, "a star with no temperature loads dark, not aborted",
                      [&](const System &sys) {
            check(sys.root && !thermalAuthored(sys.root->surface.temperature),
                  "the omitted key reads the sentinel");
            check(sys.root->star_luminosity == 0.0,
                  "and the system is dark by statement, not by crash");
        });
    }

    if(g_failures == 0) { std::printf("test_belts: OK\n"); return 0; }
    std::printf("test_belts: %d FAILURES\n", g_failures);
    return 1;
}
