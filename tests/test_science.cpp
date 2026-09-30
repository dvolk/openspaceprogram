// test_science: experiments + the career value model (src/science.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_science.cpp -o test_science && ./test_science)
//
// Pure logic: the uniqueness key, suit-level dedup, display names, the value
// model (base x situation x body, diminishing returns), the Career accounting
// (score + recovered + counts, one structure that cannot desync), and the
// Landed / altitude-band situation classifier.
#include "science.h"

#include <cstdio>
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

    // --- base value + situation weights ---
    {
        CHECK(baseValue("observation") == 10);
        CHECK(situationWeight(SciSituation::Landed) == 1.0);
        CHECK(situationWeight(SciSituation::LowOrbit) == 1.25);
        CHECK(situationWeight(SciSituation::HighOrbit) == 1.5);
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

    // --- Career: score + recovered + counts are one structure (no desync) --
    {
        Career c;
        const Experiment a = obs("Kerbin", SciSituation::Landed, "lowlands");
        CHECK(c.recover(a, 1.0) == 10);   // new, full
        CHECK(c.score == 10);
        CHECK(c.recovered.size() == 1);
        CHECK(c.recovered[0].count == 1);
        CHECK(c.recover(a, 1.0) == 5);    // repeat (halved)
        CHECK(c.score == 15);
        CHECK(c.recovered.size() == 1);   // still one unique key
        CHECK(c.recovered[0].count == 2);
        CHECK(c.recover(a, 1.0) == 2);    // repeat again
        CHECK(c.score == 17);
        CHECK(c.recovered[0].count == 3);
        // a different key is independent
        const Experiment b = obs("Kerbin", SciSituation::HighOrbit, "midlands");
        CHECK(c.recover(b, 1.0) == 15);   // new, full
        CHECK(c.recovered.size() == 2);
        CHECK(c.score == 32);
        CHECK(findRecovered(c.recovered, a) != nullptr);
        CHECK(findRecovered(c.recovered, b) != nullptr);
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
        const RecoverSummary s = recoverMany(c, loot, "Kerbin", 2.0);
        CHECK(s.gained == 10);            // 10 x 1.0 x 1.0, banked ONCE
        CHECK(s.fresh.size() == 1);
        CHECK(s.repeat == 0);
        CHECK(c.score == 10);
        CHECK(c.recovered.size() == 1);
        CHECK(c.recovered[0].count == 1);

        // A repeat recovery of the same bag: deduped, scored down (halved).
        const RecoverSummary s2 = recoverMany(c, loot, "Kerbin", 2.0);
        CHECK(s2.gained == 5);            // 10 -> 5 (halved), still once
        CHECK(s2.fresh.size() == 0);
        CHECK(s2.repeat == 5);
        CHECK(c.score == 15);
        CHECK(c.recovered[0].count == 2);

        // A mixed bag: one fresh key (frontier body) + one repeat key.
        const Experiment m = obs("Mun", SciSituation::LowOrbit, "midlands");
        const RecoverSummary s3 = recoverMany(c, { m, m, e }, "Kerbin", 2.0);
        // fresh Mun low-orbit = 10 x 1.25 x 2.0 = 25; repeat Kerbin landed = 10/4 = 2
        CHECK(s3.gained == 27);
        CHECK(s3.fresh.size() == 1);
        CHECK(s3.repeat == 2);
        CHECK(c.score == 42);
        CHECK(c.recovered.size() == 2);
    }

    // --- display name (incl. Landed) ---
    {
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        CHECK(experimentName(a) == "Low orbit observation of Midlands on Mun");
        const Experiment b = obs("Kerbin", SciSituation::Landed, "ocean");
        CHECK(experimentName(b) == "Landed observation of Ocean on Kerbin");
        const Experiment c = obs("Kerbin", SciSituation::HighOrbit, "ocean");
        CHECK(experimentName(c) == "High orbit observation of Ocean on Kerbin");
    }

    // --- situation ids round-trip (incl. landed) ---
    {
        CHECK(std::string(situationId(SciSituation::Landed)) == "landed");
        CHECK(std::string(situationId(SciSituation::LowOrbit)) == "low_orbit");
        CHECK(std::string(situationId(SciSituation::HighOrbit)) == "high_orbit");
        CHECK(situationFromId("landed") == SciSituation::Landed);
        CHECK(situationFromId("high_orbit") == SciSituation::HighOrbit);
        CHECK(situationFromId("low_orbit") == SciSituation::LowOrbit);
        CHECK(situationFromId("") == SciSituation::LowOrbit);   // default
    }

    // --- the situation cut: atmosphere top wins; else half the radius -----
    {
        CHECK(situationCutAlt(70000.0, 600000.0) == 70000.0);
        CHECK(situationCutAlt(0.0, 200000.0) == 100000.0);
    }

    // --- situationFor: grounded -> Landed; else the altitude band ---------
    {
        // grounded wins regardless of altitude (the #75 fix)
        CHECK(situationFor(true, 0.0, 70000.0, 600000.0) == SciSituation::Landed);
        CHECK(situationFor(true, 69999.0, 70000.0, 600000.0) == SciSituation::Landed);
        // not grounded: the altitude band (below the atmo top -> low orbit)
        CHECK(situationFor(false, 69999.0, 70000.0, 600000.0) == SciSituation::LowOrbit);
        // not grounded, at/above the atmo top -> high orbit
        CHECK(situationFor(false, 70000.0, 70000.0, 600000.0) == SciSituation::HighOrbit);
        // airless body: the cut is half the radius
        CHECK(situationFor(false, 99999.0, 0.0, 200000.0) == SciSituation::LowOrbit);
        CHECK(situationFor(false, 100000.0, 0.0, 200000.0) == SciSituation::HighOrbit);
    }

    if(failures == 0) {
        printf("test_science: all OK\n");
        return 0;
    }
    printf("test_science: %d failure(s)\n", failures);
    return 1;
}
