// test_science: experiments + the career value model (src/science.h).
// Runs from the repo root (loads res/data/experiments.json):
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_science.cpp src/science.cpp -o test_science && ./test_science)
//
// Pure logic: the uniqueness key, suit-level dedup, display names, the value
// model (base x situation x body, diminishing returns), the Career accounting
// (score + the append-only log, one structure that cannot desync), and the
// Landed / altitude-band situation classifier.
#include "science.h"

#include <cstdio>
#include <exception>
#include <map>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static Experiment obs(const char *body, SciSituation sit, const char *biome) {
    Experiment e;
    e.type = "observation";
    e.body = body;
    e.situation = sit;
    e.biome = biome;
    return e;
}

int main() {
    // The family table is data now (res/data/experiments.json).
    try {
        loadExperimentDefs("res/data/experiments.json");
    } catch(const std::exception &e) {
        printf("FAIL setup: %s\n", e.what());
        return 1;
    }

    // --- uniqueness: type+body+situation+biome; any field differs ---
    {
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        const Experiment same = obs("Mun", SciSituation::LowOrbit, "midlands");
        const Experiment otherBiome = obs("Mun", SciSituation::LowOrbit, "lowlands");
        const Experiment otherSit = obs("Mun", SciSituation::Landed, "midlands");
        const Experiment otherBody = obs("Kerbin", SciSituation::LowOrbit, "midlands");
        CHECK(a == same);
        CHECK(a != otherBiome);
        CHECK(a != otherSit);
        CHECK(a != otherBody);
        // A crew report IS biome-specific in low orbit, but NOT in high orbit
        // ("of the planet" -- KSP: one reading per orbit segment), so there the
        // biome is dropped from the key.
        const Experiment hiA = obs("Mun", SciSituation::HighOrbit, "midlands");
        const Experiment hiB = obs("Mun", SciSituation::HighOrbit, "lowlands");
        CHECK(hiA == hiB);   // same segment, different biome -> the same finding
        CHECK(hiA != a);     // different situation (high vs low orbit) -> distinct
    }

    // --- addExperiment / holdsExperiment (suit-level dedup) ---
    {
        std::vector<Experiment> held;
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        CHECK(addExperiment(held, a));
        CHECK(held.size() == 1);
        CHECK(!addExperiment(held, a));   // duplicate refused
        CHECK(held.size() == 1);
        CHECK(holdsExperiment(held, a));
    }

    // --- canHoldFinding: the per-role storage ceiling (Part::canHold) -----
    // A part holds findings per FAMILY (1 for instrument/courier, unlimited
    // for a container), never an exact duplicate, and an instrument only its
    // own family. `held` grows as findings are accepted, mirroring a part.
    {
        const Experiment obsLow  = obs("Kerbin", SciSituation::Landed, "lowlands");
        const Experiment obsHigh = obs("Kerbin", SciSituation::Landed, "highlands");
        Experiment matLow  = obsLow;  matLow.type  = "materials study";
        Experiment matHigh = obsHigh; matHigh.type = "materials study";

        // None: holds nothing
        CHECK(!canHoldFinding(ExpStorage::None, "", {}, obsLow));

        // Instrument: 1 of its OWN family only; a second (even a new biome)
        // is refused, and any other family is refused outright.
        std::vector<Experiment> pod;
        const std::string own = "materials study";
        CHECK(canHoldFinding(ExpStorage::Instrument, own, pod, matLow));
        CHECK(!canHoldFinding(ExpStorage::Instrument, own, pod, obsLow));  // not its family
        pod.push_back(matLow);
        CHECK(!canHoldFinding(ExpStorage::Instrument, own, pod, matHigh)); // family full
        CHECK(!canHoldFinding(ExpStorage::Instrument, own, pod, matLow));  // exact dup

        // Courier (a suit): 1 per family, any family -- an observation AND a
        // materials study, but not two of either.
        std::vector<Experiment> suit;
        CHECK(canHoldFinding(ExpStorage::Courier, "", suit, obsLow));
        suit.push_back(obsLow);
        CHECK(canHoldFinding(ExpStorage::Courier, "", suit, matLow));      // 2nd family OK
        suit.push_back(matLow);
        CHECK(!canHoldFinding(ExpStorage::Courier, "", suit, obsHigh));    // observation full
        CHECK(!canHoldFinding(ExpStorage::Courier, "", suit, matHigh));    // materials full
        CHECK(!canHoldFinding(ExpStorage::Courier, "", suit, obsLow));     // exact dup

        // Container (a capsule): unlimited per family, never an exact duplicate
        std::vector<Experiment> cap;
        CHECK(canHoldFinding(ExpStorage::Container, "", cap, matLow));
        cap.push_back(matLow);
        CHECK(canHoldFinding(ExpStorage::Container, "", cap, matHigh));    // same family, new biome
        cap.push_back(matHigh);
        CHECK(!canHoldFinding(ExpStorage::Container, "", cap, matLow));    // exact dup
        CHECK(canHoldFinding(ExpStorage::Container, "", cap, obsLow));     // another family
    }

    // --- canHoldFinding: a capsule holds ONE finding per key ---------------
    // When the family is NOT biome-specific in the situation (high-orbit
    // observation, low-orbit materials study), several biomes collapse to one
    // key, so a capsule holds one per (body, situation) -- not one per biome.
    // When it IS biome-specific (landed), several biomes are distinct keys.
    {
        // high-orbit observation: biome dropped from the key -> one finding
        std::vector<Experiment> cap;
        const Experiment hA = obs("Kerbin", SciSituation::HighOrbit, "midlands");
        const Experiment hB = obs("Kerbin", SciSituation::HighOrbit, "lowlands");
        CHECK(canHoldFinding(ExpStorage::Container, "", cap, hA));
        cap.push_back(hA);
        CHECK(!canHoldFinding(ExpStorage::Container, "", cap, hB));   // same key now
        // landed observation: biome IS in the key -> several biomes coexist
        const Experiment lA = obs("Kerbin", SciSituation::Landed, "midlands");
        const Experiment lB = obs("Kerbin", SciSituation::Landed, "lowlands");
        CHECK(canHoldFinding(ExpStorage::Container, "", { lA }, lB));   // distinct keys
        // same body+situation but different body -> always distinct
        CHECK(canHoldFinding(ExpStorage::Container, "", { hA },
                             obs("Mun", SciSituation::HighOrbit, "midlands")));
    }

    // --- base value + situation weights ---
    {
        CHECK(baseValue("observation") == 10);
        CHECK(baseValue("materials study") == 25);   // the pod's instrument
        CHECK(baseValue("barometer") == 10);         // a basic instrument
        CHECK(baseValue("unknown family") == 10);    // falls back to the base
        CHECK(situationWeight(SciSituation::Landed) == 1.0);
        CHECK(situationWeight(SciSituation::FlyingLow) == 1.1);
        CHECK(situationWeight(SciSituation::FlyingHigh) == 1.2);
        CHECK(situationWeight(SciSituation::LowOrbit) == 1.25);
        CHECK(situationWeight(SciSituation::HighOrbit) == 1.5);
    }

    // --- ExperimentDef: per-family biome-specificity + availability --------
    // A reading is biome-specific only where biomes are tellable apart: a
    // crew report in landed + flying + low orbit; a materials study in landed
    // + flying; a barometer in landed + flying-low. High orbit (and the
    // barometer's flying-high) are "of the planet" -- never biome-specific.
    {
        // observation (a crew report): landed + flying + low orbit; high global
        CHECK(biomeSpecificIn("observation", SciSituation::Landed));
        CHECK(biomeSpecificIn("observation", SciSituation::FlyingLow));
        CHECK(biomeSpecificIn("observation", SciSituation::FlyingHigh));
        CHECK(biomeSpecificIn("observation", SciSituation::LowOrbit));
        CHECK(!biomeSpecificIn("observation", SciSituation::HighOrbit));
        // materials study: landed + both flying bands; orbit global
        CHECK(biomeSpecificIn("materials study", SciSituation::Landed));
        CHECK(biomeSpecificIn("materials study", SciSituation::FlyingLow));
        CHECK(biomeSpecificIn("materials study", SciSituation::FlyingHigh));
        CHECK(!biomeSpecificIn("materials study", SciSituation::LowOrbit));
        CHECK(!biomeSpecificIn("materials study", SciSituation::HighOrbit));
        // barometer: landed + flying-low only (upper air + space are global)
        CHECK(biomeSpecificIn("barometer", SciSituation::Landed));
        CHECK(biomeSpecificIn("barometer", SciSituation::FlyingLow));
        CHECK(!biomeSpecificIn("barometer", SciSituation::FlyingHigh));
        CHECK(!biomeSpecificIn("barometer", SciSituation::LowOrbit));
        CHECK(!biomeSpecificIn("barometer", SciSituation::HighOrbit));
        // unknown family -> biome-specific in EVERY situation (the KSP common
        // case + pre-registry behavior): never merge biome-distinct findings
        // on a guess. A def opts a family OUT per situation instead.
        CHECK(biomeSpecificIn("unknown", SciSituation::Landed));
        CHECK(biomeSpecificIn("unknown", SciSituation::FlyingLow));
        CHECK(biomeSpecificIn("unknown", SciSituation::FlyingHigh));
        CHECK(biomeSpecificIn("unknown", SciSituation::LowOrbit));
        CHECK(biomeSpecificIn("unknown", SciSituation::HighOrbit));
    }

    // --- ExperimentDef: availability (valid_in) ----------------------------
    // Today all three families run in all five situations; an unregistered
    // family is valid everywhere (the safe fallback). A situation-gated
    // instrument (a seismometer: landed only) would be one def entry.
    {
        CHECK(experimentValidIn("observation", SciSituation::Landed));
        CHECK(experimentValidIn("observation", SciSituation::FlyingHigh));
        CHECK(experimentValidIn("observation", SciSituation::HighOrbit));
        CHECK(experimentValidIn("materials study", SciSituation::Landed));
        CHECK(experimentValidIn("materials study", SciSituation::FlyingLow));
        CHECK(experimentValidIn("materials study", SciSituation::HighOrbit));
        // barometer: "an atmospheric pressure scan can be performed from
        // anywhere" -- valid in all five situations.
        CHECK(experimentValidIn("barometer", SciSituation::Landed));
        CHECK(experimentValidIn("barometer", SciSituation::FlyingLow));
        CHECK(experimentValidIn("barometer", SciSituation::FlyingHigh));
        CHECK(experimentValidIn("barometer", SciSituation::LowOrbit));
        CHECK(experimentValidIn("barometer", SciSituation::HighOrbit));
        CHECK(experimentValidIn("unknown", SciSituation::FlyingHigh));
        const ExperimentDef *o = defFor("observation");
        const ExperimentDef *m = defFor("materials study");
        CHECK(o != nullptr && o->base_value == 10 && o->validIn(SciSituation::HighOrbit));
        CHECK(m != nullptr && m->base_value == 25 && m->biomeSpecificIn(SciSituation::Landed));
        CHECK(defFor("no such family") == nullptr);
    }

    // --- the materials-study family (the pod's experiment) is worth MORE ---
    // base 25 x situation x body; the diminishing-returns halving still applies.
    {
        Experiment m;
        m.type = "materials study";
        m.body = "Kerbin";
        m.situation = SciSituation::Landed;
        m.biome = "lowlands";
        CHECK(scoreOf(m, 0, 1.0) == 25);   // new, home, landed
        CHECK(scoreOf(m, 0, 2.0) == 50);   // a science_mult of 2 doubles it
        // 25 x 1.0 x 1.3 = 32.5 -> lround half-away = 33
        CHECK(scoreOf(m, 0, 1.3) == 33);
        CHECK(scoreOf(m, 1, 1.0) == 12);   // repeat halves (25/2, integer)
        // a materials study is a DIFFERENT key than an observation of the
        // same place -- the two bank independently.
        CHECK(m != obs("Kerbin", SciSituation::Landed, "lowlands"));
    }

    // --- scoreOf: first recovery (prevCount 0) = base x situation x body ---
    {
        CHECK(scoreOf(obs("Kerbin", SciSituation::Landed, "lowlands"), 0, 1.0) == 10);
        CHECK(scoreOf(obs("Kerbin", SciSituation::HighOrbit, "midlands"), 0, 1.0) == 15);
        CHECK(scoreOf(obs("Mun", SciSituation::Landed, "midlands"), 0, 2.0) == 20);
        CHECK(scoreOf(obs("Mun", SciSituation::HighOrbit, "midlands"), 0, 2.0) == 30);
    }

    // --- scoreOf: diminishing returns -- each repeat halves, floor 1 ------
    {
        const Experiment e = obs("Kerbin", SciSituation::Landed, "lowlands");
        CHECK(scoreOf(e, 0, 1.0) == 10);
        CHECK(scoreOf(e, 1, 1.0) == 5);
        CHECK(scoreOf(e, 2, 1.0) == 2);
        CHECK(scoreOf(e, 3, 1.0) == 1);
        CHECK(scoreOf(e, 9, 1.0) == 1);   // floor holds
    }

    // --- Career: score + the log are one structure (no desync) ------------
    // The log appends ONE entry per bank (repeats included); the diminishing
    // count is countKey (how many entries share the key), so score and log
    // cannot desync.
    {
        Career c;
        const Experiment a = obs("Kerbin", SciSituation::Landed, "lowlands");
        CHECK(c.recover(a, 1.0) == 10);   // new, full
        CHECK(c.score == 10);
        CHECK(c.recovered.size() == 1);   // one entry for the first bank
        CHECK(countKey(c.recovered, a) == 1);
        CHECK(c.recover(a, 1.0) == 5);    // repeat (halved)
        CHECK(c.score == 15);
        CHECK(c.recovered.size() == 2);   // a second entry for the same key
        CHECK(countKey(c.recovered, a) == 2);
        CHECK(c.recover(a, 1.0) == 2);    // repeat again
        CHECK(c.score == 17);
        CHECK(c.recovered.size() == 3);
        CHECK(countKey(c.recovered, a) == 3);
        // a different key is independent
        const Experiment b = obs("Kerbin", SciSituation::HighOrbit, "midlands");
        CHECK(c.recover(b, 1.0) == 15);   // new, full
        CHECK(c.recovered.size() == 4);
        CHECK(c.score == 32);
        CHECK(countKey(c.recovered, b) == 1);
        CHECK(holdsExperiment(c.recovered, a));
        CHECK(holdsExperiment(c.recovered, b));
    }

    // --- Career::version: bumped on every mutation (the Lab cache's key) ---
    {
        Career c;
        const std::size_t v0 = c.version;
        c.recover(obs("Kerbin", SciSituation::Landed, "lowlands"), 1.0);
        CHECK(c.version == v0 + 1);          // a bank bumps it
        const std::size_t v1 = c.version;
        c.setFrom(7, { obs("Mun", SciSituation::LowOrbit, "midlands") });
        CHECK(c.version == v1 + 1);          // a Load bumps it
        CHECK(c.score == 7);
        CHECK(c.recovered.size() == 1);
        const std::size_t v2 = c.version;
        c.reset();
        CHECK(c.version == v2 + 1);          // a reset bumps it
        CHECK(c.score == 0);
        CHECK(c.recovered.empty());
    }

    // --- recoverMany: whole-recovery dedup (multi-crew double-banking) ----
    // The bag is de-duplicated by key, so ONE observation held by N kerbals
    // banks ONCE -- not N times (v1's per-occurrence harvest was a farming
    // exploit: 10 + 5 + 2 ... per observation per recovery).
    {
        Career c;
        const Experiment e = obs("Kerbin", SciSituation::Landed, "lowlands");
        // Two kerbals BOTH hold the same key.
        std::vector<Experiment> loot = { e, e };
        const RecoverSummary s =
            recoverMany(c, loot, {{"Kerbin", 1.0}}, 1000.0);
        CHECK(s.gained == 10);            // 10 x 1.0 x 1.0, banked ONCE
        CHECK(s.fresh.size() == 1);
        CHECK(s.repeat == 0);
        CHECK(c.score == 10);
        CHECK(c.recovered.size() == 1);
        CHECK(countKey(c.recovered, e) == 1);
        CHECK(c.recovered[0].recovered_at == 1000.0);   // the bank stamp
        CHECK(c.recovered[0].kerbal.empty());           // provenance preserved

        // A repeat recovery of the same bag: deduped, scored down (halved).
        const RecoverSummary s2 =
            recoverMany(c, loot, {{"Kerbin", 1.0}}, 2000.0);
        CHECK(s2.gained == 5);            // 10 -> 5 (halved), still once
        CHECK(s2.fresh.size() == 0);
        CHECK(s2.repeat == 5);
        CHECK(c.score == 15);
        CHECK(c.recovered.size() == 2);   // the repeat is its own log entry
        CHECK(countKey(c.recovered, e) == 2);

        // A mixed bag: one fresh key (far body) + one repeat key.
        const Experiment m = obs("Mun", SciSituation::LowOrbit, "midlands");
        const RecoverSummary s3 = recoverMany(
            c, { m, m, e }, {{"Kerbin", 1.0}, {"Mun", 2.0}}, 3000.0);
        // fresh Mun low-orbit = 10 x 1.25 x 2.0 = 25; repeat Kerbin landed = 10/4 = 2
        CHECK(s3.gained == 27);
        CHECK(s3.fresh.size() == 1);
        CHECK(s3.repeat == 2);
        CHECK(c.score == 42);
        CHECK(c.recovered.size() == 4);   // e, e, then m and e again
        CHECK(countKey(c.recovered, m) == 1);
    }

    // --- bodyWeightOf: science_mult lookup (unknown -> 1.0) ----------------
    {
        const std::map<std::string, double> mult = {
            {"Kerbin", 1.0}, {"Mun", 1.3}, {"Jool", 2.1}};
        CHECK(bodyWeightOf(mult, "Kerbin") == 1.0);
        CHECK(bodyWeightOf(mult, "Mun") == 1.3);
        CHECK(bodyWeightOf(mult, "Nowhere") == 1.0);
        CHECK(bodyWeightOf({}, "Mun") == 1.0);
    }

    // --- recoverMany: the biome-drop banks ONE finding per segment ---------
    // The whole point of the identity change: two high-orbit observations of
    // different biomes are now the SAME key, so a recovery carrying both banks
    // ONCE -- not twice. (Landed observations of two biomes still bank twice.)
    {
        Career c;
        const Experiment hA = obs("Kerbin", SciSituation::HighOrbit, "midlands");
        const Experiment hB = obs("Kerbin", SciSituation::HighOrbit, "lowlands");
        const RecoverSummary s =
            recoverMany(c, { hA, hB }, {{"Kerbin", 1.0}}, 100.0);
        CHECK(s.gained == 15);            // 10 x 1.5 x 1.0, banked ONCE
        CHECK(s.fresh.size() == 1);
        CHECK(s.repeat == 0);
        CHECK(c.recovered.size() == 1);
        CHECK(countKey(c.recovered, hA) == 1);
        CHECK(hA == hB);                  // the dedup rests on the identity change

        // landed observations of two biomes are distinct keys -> bank TWICE
        Career c2;
        const Experiment lA = obs("Kerbin", SciSituation::Landed, "midlands");
        const Experiment lB = obs("Kerbin", SciSituation::Landed, "lowlands");
        const RecoverSummary s2 =
            recoverMany(c2, { lA, lB }, {{"Kerbin", 1.0}}, 100.0);
        CHECK(s2.gained == 20);           // 10 + 10 (two fresh keys)
        CHECK(s2.fresh.size() == 2);
        CHECK(c2.recovered.size() == 2);
    }

    // --- display name (incl. Landed + flying) ------------------------------
    {
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        CHECK(experimentName(a) == "Low orbit observation of Midlands on Mun");
        const Experiment l = obs("Kerbin", SciSituation::Landed, "ocean");
        CHECK(experimentName(l) == "Landed observation of Ocean on Kerbin");
        const Experiment hi = obs("Kerbin", SciSituation::HighOrbit, "ocean");
        // High orbit: not biome-specific -> "of the planet", no biome in name.
        CHECK(experimentName(hi) == "High orbit observation on Kerbin");
        // Flying: a crew report IS biome-specific in the air, so the biome is
        // in the name here (unlike high orbit).
        const Experiment fl = obs("Kerbin", SciSituation::FlyingLow, "ocean");
        CHECK(experimentName(fl) == "Flying low observation of Ocean on Kerbin");
        const Experiment fh = obs("Kerbin", SciSituation::FlyingHigh, "ocean");
        CHECK(experimentName(fh) == "Flying high observation of Ocean on Kerbin");
        // Materials study: biome-specific in landed + flying; orbit is global.
        Experiment m;  m.type = "materials study"; m.body = "Kerbin";
        m.situation = SciSituation::Landed;   m.biome = "lowlands";
        CHECK(experimentName(m) == "Landed materials study of Lowlands on Kerbin");
        Experiment mfl; mfl.type = "materials study"; mfl.body = "Kerbin";
        mfl.situation = SciSituation::FlyingLow;  mfl.biome = "lowlands";
        CHECK(experimentName(mfl) == "Flying low materials study of Lowlands on Kerbin");
        Experiment mh; mh.type = "materials study"; mh.body = "Kerbin";
        mh.situation = SciSituation::LowOrbit; mh.biome = "lowlands";
        CHECK(experimentName(mh) == "Low orbit materials study on Kerbin");
        // Barometer: biome-specific in landed + flying-low; flying-high and
        // orbit are global "of the planet" (no biome in the name).
        Experiment baroL; baroL.type = "barometer"; baroL.body = "Kerbin";
        baroL.situation = SciSituation::Landed;   baroL.biome = "lowlands";
        CHECK(experimentName(baroL) == "Landed barometer of Lowlands on Kerbin");
        Experiment baroFL; baroFL.type = "barometer"; baroFL.body = "Kerbin";
        baroFL.situation = SciSituation::FlyingLow;  baroFL.biome = "lowlands";
        CHECK(experimentName(baroFL) == "Flying low barometer of Lowlands on Kerbin");
        Experiment baroFH; baroFH.type = "barometer"; baroFH.body = "Kerbin";
        baroFH.situation = SciSituation::FlyingHigh; baroFH.biome = "lowlands";
        CHECK(experimentName(baroFH) == "Flying high barometer on Kerbin");
        Experiment baroH; baroH.type = "barometer"; baroH.body = "Kerbin";
        baroH.situation = SciSituation::HighOrbit; baroH.biome = "lowlands";
        CHECK(experimentName(baroH) == "High orbit barometer on Kerbin");
        // Empty biome (a star / banded giant: no classifiable surface): the
        // "of <Biome>" clause is dropped even where the family is
        // biome-specific -- "Low orbit observation on Kerbol", never "of None"
        // or a dangling "of".
        const Experiment star = obs("Kerbol", SciSituation::LowOrbit, "");
        CHECK(experimentName(star) == "Low orbit observation on Kerbol");
        const Experiment starHi = obs("Kerbol", SciSituation::HighOrbit, "");
        CHECK(experimentName(starHi) == "High orbit observation on Kerbol");
        // Two empty-biome findings of the same key dedup (both "of the body").
        CHECK(star == obs("Kerbol", SciSituation::LowOrbit, ""));
    }

    // --- labEntries: the Lab's pre-built rows (issue #87) ------------------
    // One row per bank, in bank order; the stamps drawn in the home calendar;
    // the provenance sub-line is the kerbal + ship ("(EVA)" when aboard
    // nothing). A 0 calendar degrades to name-only (no home yet).
    {
        const Calendar cal = Calendar::make(100.0, 0.0, 1);   // day = 100s
        // a fully-provenanced bank (ran 25s -> Day 1 06:00; banked 125s -> Day 2)
        Experiment e;
        e.body = "Mun";
        e.situation = SciSituation::LowOrbit;
        e.biome = "midlands";
        e.ran_at = 25.0;
        e.recovered_at = 125.0;
        e.kerbal = "Jebediah";
        e.ship = "racer";
        auto rows = labEntries({ e }, cal);
        CHECK(rows.size() == 1);
        if(rows.size() == 1) {
            CHECK(rows[0].line1 ==
                  "Day 2  06:00  Low orbit observation of Midlands on Mun");
            CHECK(rows[0].line2 == "ran Day 1  06:00   Jebediah   racer");
        }

        // free-EVA: a kerbal, no ship -> "(EVA)"; no ran stamp when ran_at==0
        Experiment eva;
        eva.body = "Kerbin";
        eva.situation = SciSituation::Landed;
        eva.biome = "lowlands";
        eva.kerbal = "Bill";
        auto evaRows = labEntries({ eva }, cal);
        CHECK(evaRows.size() == 1);
        if(evaRows.size() == 1) {
            CHECK(evaRows[0].line1 == "Landed observation of Lowlands on Kerbin");
            CHECK(evaRows[0].line2 == "Bill   (EVA)");
        }

        // a bare bank (pre-provenance save): name-only, no sub-line at all
        Experiment bare;
        bare.body = "Kerbin";
        bare.situation = SciSituation::Landed;
        bare.biome = "ocean";
        auto bareRows = labEntries({ bare }, cal);
        CHECK(bareRows.size() == 1);
        if(bareRows.size() == 1) {
            CHECK(bareRows[0].line1 == "Landed observation of Ocean on Kerbin");
            CHECK(bareRows[0].line2.empty());
        }

        // no home calendar: the stamps drop, the provenance text stays
        auto noCal = labEntries({ e }, Calendar{});
        CHECK(noCal.size() == 1);
        if(noCal.size() == 1) {
            CHECK(noCal[0].line1 == "Low orbit observation of Midlands on Mun");
            CHECK(noCal[0].line2 == "Jebediah   racer");
        }
    }

    // --- situation ids round-trip (incl. landed + flying) ---
    {
        CHECK(std::string(situationId(SciSituation::Landed)) == "landed");
        CHECK(std::string(situationId(SciSituation::FlyingLow)) == "flying_low");
        CHECK(std::string(situationId(SciSituation::FlyingHigh)) == "flying_high");
        CHECK(std::string(situationId(SciSituation::LowOrbit)) == "low_orbit");
        CHECK(std::string(situationId(SciSituation::HighOrbit)) == "high_orbit");
        CHECK(situationFromId("landed") == SciSituation::Landed);
        CHECK(situationFromId("flying_low") == SciSituation::FlyingLow);
        CHECK(situationFromId("flying_high") == SciSituation::FlyingHigh);
        CHECK(situationFromId("high_orbit") == SciSituation::HighOrbit);
        CHECK(situationFromId("low_orbit") == SciSituation::LowOrbit);
        CHECK(situationFromId("") == SciSituation::LowOrbit);   // default
    }

    // --- the orbit cut: shell edge (soi - radius - sea_level) + margin -----
    {
        CHECK(orbitCutAlt(700000.0, 600000.0, 0.0) == 110000.0);    // Kerbin: 100km shell + 10km
        CHECK(orbitCutAlt(6240000.0, 6000000.0, 0.0) == 250000.0);  // Jool: derived 240km shell
        CHECK(orbitCutAlt(710000.0, 600000.0, 10000.0) == 110000.0); // a raised sea shifts with the datum
        CHECK(orbitCutAlt(6371000.0, 6371000.0, 0.0) == 10000.0);   // degenerate pin: a real body cannot load with its shell at its own surface (validateBodyLimits refuses)
        CHECK(kOrbitCutMargin == 10000.0);
        CHECK(kFlyingLowFrac == 0.2);
    }

    // --- situationFor: the five situations from grounded + altitude --------
    {
        const double atmoTop = 70000.0;    // Kerbin's atmosphere
        const double cut = 110000.0;      // SoI edge (100km) + 10km margin
        // grounded wins regardless of altitude (the #75 fix)
        CHECK(situationFor(true, 0.0, atmoTop, cut) == SciSituation::Landed);
        CHECK(situationFor(true, 69999.0, atmoTop, cut) == SciSituation::Landed);
        // airborne in the atmosphere -> flying; the bottom 20% is flying-low
        CHECK(situationFor(false, 0.0, atmoTop, cut) == SciSituation::FlyingLow);
        CHECK(situationFor(false, 13999.0, atmoTop, cut) == SciSituation::FlyingLow);
        CHECK(situationFor(false, 14000.0, atmoTop, cut) == SciSituation::FlyingHigh);
        CHECK(situationFor(false, 69999.0, atmoTop, cut) == SciSituation::FlyingHigh);
        // above the atmosphere -> the orbit band, split at the cut
        CHECK(situationFor(false, 70000.0, atmoTop, cut) == SciSituation::LowOrbit);
        CHECK(situationFor(false, 109999.0, atmoTop, cut) == SciSituation::LowOrbit);
        CHECK(situationFor(false, 110000.0, atmoTop, cut) == SciSituation::HighOrbit);
        CHECK(situationFor(false, 500000.0, atmoTop, cut) == SciSituation::HighOrbit);
        // airless body: no flying band; the altitude band straight off the cut
        CHECK(situationFor(false, 0.0, 0.0, cut) == SciSituation::LowOrbit);
        CHECK(situationFor(false, 109999.0, 0.0, cut) == SciSituation::LowOrbit);
        CHECK(situationFor(false, 110000.0, 0.0, cut) == SciSituation::HighOrbit);
    }

    if(failures == 0) {
        printf("test_science: all OK\n");
        return 0;
    }
    printf("test_science: %d failure(s)\n", failures);
    return 1;
}
