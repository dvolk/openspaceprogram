// test_science: experiments + science score helpers (src/science.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_science.cpp -o test_science && ./test_science)
//
// Pure logic: uniqueness key, add/merge scoring, display names, the
// low/high altitude cut (atmosphere top, else half the radius).
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
        const Experiment otherSit = obs("Mun", SciSituation::HighOrbit, "midlands");
        const Experiment otherBody = obs("Kerbin", SciSituation::LowOrbit, "midlands");
        CHECK(a == same);
        CHECK(a != otherBiome);
        CHECK(a != otherSit);
        CHECK(a != otherBody);
    }

    // --- addExperiment / holdsExperiment / score ---
    {
        std::vector<Experiment> held;
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        CHECK(addExperiment(held, a));
        CHECK(held.size() == 1);
        CHECK(!addExperiment(held, a));   // duplicate refused
        CHECK(held.size() == 1);
        CHECK(holdsExperiment(held, a));
        CHECK(scoreOf(a) == 1);
    }

    // --- mergeExperiments scores only the new keys ---
    {
        std::vector<Experiment> recovered;
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        const Experiment b = obs("Mun", SciSituation::HighOrbit, "midlands");
        CHECK(addExperiment(recovered, a));
        std::vector<Experiment> loot = {a, b, b};
        CHECK(mergeExperiments(recovered, loot) == 1);  // only b is new
        CHECK(recovered.size() == 2);
        CHECK(mergeExperiments(recovered, loot) == 0);  // nothing new
    }

    // --- display name ---
    {
        const Experiment a = obs("Mun", SciSituation::LowOrbit, "midlands");
        CHECK(experimentName(a) == "Low orbit observation of Midlands on Mun");
        const Experiment b = obs("Kerbin", SciSituation::HighOrbit, "ocean");
        CHECK(experimentName(b) == "High orbit observation of Ocean on Kerbin");
    }

    // --- situation ids round-trip ---
    {
        CHECK(std::string(situationId(SciSituation::LowOrbit)) == "low_orbit");
        CHECK(std::string(situationId(SciSituation::HighOrbit)) == "high_orbit");
        CHECK(situationFromId("high_orbit") == SciSituation::HighOrbit);
        CHECK(situationFromId("low_orbit") == SciSituation::LowOrbit);
        CHECK(situationFromId("") == SciSituation::LowOrbit);
    }

    // --- altitude cut: atmosphere top wins; else half the radius ---
    {
        CHECK(situationCutAlt(70000.0, 600000.0) == 70000.0);
        CHECK(situationCutAlt(0.0, 200000.0) == 100000.0);
        CHECK(situationFromAltitude(69999.0, 70000.0, 600000.0)
              == SciSituation::LowOrbit);
        CHECK(situationFromAltitude(70000.0, 70000.0, 600000.0)
              == SciSituation::HighOrbit);
        CHECK(situationFromAltitude(99999.0, 0.0, 200000.0)
              == SciSituation::LowOrbit);
        CHECK(situationFromAltitude(100000.0, 0.0, 200000.0)
              == SciSituation::HighOrbit);
    }

    if(failures == 0) {
        printf("test_science: all OK\n");
        return 0;
    }
    printf("test_science: %d failure(s)\n", failures);
    return 1;
}
