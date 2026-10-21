// test_roster: the kerbal roster -- parsing, lookup, and the save
// round trip (src/crew.h, src/crew.cpp, save.h's roster keys).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_roster)
//
// The roster is the CAST half of crew: the named people a capsule seat or the
// home pad's pool draws from. WHO is where is the world's business (part.h:
// pad, seat or walk), so this test covers the data model, not the aboard
// bookkeeping (that is tests/test_crew.cpp and the e2e EVA/board cases).

#include "crew.h"
#include "save.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static bool expect_throw(const std::function<void()> &fn) {
    try { fn(); }
    catch(const std::runtime_error &) { return true; }
    return false;
}

// A one-line roster file, so each bad-data case is one string.
static void writeFile(const char *path, const std::string &body) {
    std::ofstream o(path);
    o << body;
    o.close();
}

int main() {
    // ---- the shipped roster ------------------------------------------------
    {
        CrewRoster r = load_crew_roster("res/data/kerbals.json");
        CHECK(!r.profiles.empty());
        for(size_t i = 0; i < r.profiles.size(); i++) {
            const KerbalProfile &p = r.profiles[i];
            CHECK(p.id != 0);
            CHECK(!p.name.empty());
            for(int s = 0; s < NumSkills; s++) {
                CHECK(p.skill[s] >= 1 && p.skill[s] <= 5);
            }
            // find() and byName() agree: one entry per id AND per name.
            const KerbalProfile *byId = r.find(p.id);
            const KerbalProfile *byNm = r.byName(p.name);
            CHECK(byId == &p);
            CHECK(byNm == &p);
        }
        CHECK(r.find(0) == nullptr);
        CHECK(r.byName("nobody in particular") == nullptr);
    }

    // ---- absent skills default; the file need not mention them -------------
    {
        const char *f = "/tmp/test_roster_defaults.json";
        writeFile(f, "{ \"kerbals\": [ { \"id\": 1, \"name\": \"Solo\" } ] }");
        CrewRoster r = load_crew_roster(f);
        CHECK(r.profiles.size() == 1);
        for(int s = 0; s < NumSkills; s++) { CHECK(r.profiles[0].skill[s] == 3); }
        std::remove(f);
    }

    // ---- bad data is refused, naming the file or the person ---------------
    {
        struct { const char *name; const char *body; } bad[] = {
            { "empty",       "{ \"kerbals\": [] }" },
            { "no key",      "{ \"people\": [] }" },
            { "no name",     "{ \"kerbals\": [ { \"id\": 1 } ] }" },
            { "no id",       "{ \"kerbals\": [ { \"name\": \"A\" } ] }" },
            { "zero id",     "{ \"kerbals\": [ { \"id\": 0, \"name\": \"A\" } ] }" },
            { "float id",    "{ \"kerbals\": [ { \"id\": 1.5, \"name\": \"A\" } ] }" },
            { "dup id",      "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\" }, "
                              "{ \"id\": 1, \"name\": \"B\" } ] }" },
            { "dup name",    "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\" }, "
                              "{ \"id\": 2, \"name\": \"A\" } ] }" },
            { "skill low",   "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\", "
                              "\"piloting\": 0 } ] }" },
            { "skill high",  "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\", "
                              "\"engineering\": 9 } ] }" },
            /* Wrong TYPES, not just wrong values: nlohmann's own type_error is
               not a std::runtime_error, so expect_throw below would let it
               escape instead of reporting a refusal. */
            { "name number", "{ \"kerbals\": [ { \"id\": 1, \"name\": 7 } ] }" },
            { "name object", "{ \"kerbals\": [ { \"id\": 1, \"name\": { \"a\": 1 } } ] }" },
            { "skill text",  "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\", "
                              "\"science\": \"five\" } ] }" },
            { "skill float", "{ \"kerbals\": [ { \"id\": 1, \"name\": \"A\", "
                              "\"science\": 3.5 } ] }" },
            { "id text",     "{ \"kerbals\": [ { \"id\": \"one\", \"name\": \"A\" } ] }" },
            { "id negative", "{ \"kerbals\": [ { \"id\": -3, \"name\": \"A\" } ] }" },
        };
        for(auto &b : bad) {
            const std::string path = std::string("/tmp/test_roster_bad.json");
            writeFile(path.c_str(), b.body);
            if(!expect_throw([&](){ load_crew_roster(path.c_str()); })) {
                printf("FAIL roster accepted bad data: %s\n", b.name);
                failures++;
            }
            std::remove(path.c_str());
        }
        // A missing file is refused, not an empty roster.
        CHECK(expect_throw([&](){ load_crew_roster("res/data/nope.json"); }));
    }

    // ---- the shade each kerbal wears ---------------------------------------
    {
        /* The cast is deliberately not all one colour (crew.h): hue is what
           tells two suits apart, so the checks below are about being told
           apart, not about staying green. */
        CHECK(kerbal_complexion(0) == kKerbalBaseGreen);
        CHECK(kerbal_tint(0) == glm::vec3(1.0f));

        /* kKerbalBaseGreen mirrors the literal utils/gen_kerbal.py writes into
           textures/kerbal.png. A mirror with nothing pinning it drifts, and
           then the tint no longer produces the complexion it claims to. */
        CHECK(std::abs(kKerbalBaseGreen.r * 255.0f - 60.0f) < 0.01f);
        CHECK(std::abs(kKerbalBaseGreen.g * 255.0f - 200.0f) < 0.01f);
        CHECK(std::abs(kKerbalBaseGreen.b * 255.0f - 60.0f) < 0.01f);

        for(uint64_t id = 1; id <= 64; id++) {
            const glm::vec3 c = kerbal_complexion(id);
            /* Never a black hole, never blown out: partsShader.fs caps light at
               1.0, so a channel over 1 here is a channel the sun flattens. */
            const float hi = std::max(c.r, std::max(c.g, c.b));
            CHECK(hi >= 0.45f && hi <= 0.92f);
            /* The tint is defined as the quotient, and the shader only ever sees
               the quotient, so the round trip is the contract. */
            const glm::vec3 t = kerbal_tint(id);
            CHECK(t.r > 0.05f && t.g > 0.05f && t.b > 0.05f);
            CHECK(t.r < 4.5f && t.g < 4.5f && t.b < 4.5f);
            CHECK(std::abs(t.r * kKerbalBaseGreen.r - c.r) < 1e-5f);
            CHECK(std::abs(t.g * kKerbalBaseGreen.g - c.g) < 1e-5f);
            CHECK(std::abs(t.b * kKerbalBaseGreen.b - c.b) < 1e-5f);
        }

        /* A suit with no chroma is indistinguishable from concrete, so the
           half-green mix has to keep SOME colour in it. Scoped to 16 ids, not
           64, because the mix is honest about one cost: a hue near magenta is
           the base green's complement and cancels toward grey (crew.h), which
           the phi draw does not reach until past the shipped cast. Widening
           this loop is the point at which the hue arc has to stop short. */
        for(uint64_t id = 1; id <= 16; id++) {
            const glm::vec3 c = kerbal_complexion(id);
            const float hi = std::max(c.r, std::max(c.g, c.b));
            const float lo = std::min(c.r, std::min(c.g, c.b));
            CHECK((hi - lo) / hi >= 0.15f);
        }

        /* The point of the whole thing: two kerbals side by side are not the
           same colour. Measured on the COMPLEXION -- the tint is only a means
           to it, and the shader multiplies a flat texture whose channels differ
           by 3x, so a tint gap says nothing until the base is multiplied in.
           Largest channel difference, because that is what a player sees on a
           0.4m suit. */
        auto apart = [](const glm::vec3 &a, const glm::vec3 &b) {
            return std::max(std::abs(a.r - b.r),
                            std::max(std::abs(a.g - b.g), std::abs(a.b - b.b)));
        };
        CrewRoster r = load_crew_roster("res/data/kerbals.json");
        for(size_t i = 0; i < r.profiles.size(); i++) {
            for(size_t j = i + 1; j < r.profiles.size(); j++) {
                CHECK(apart(r.profiles[i].complexion(),
                            r.profiles[j].complexion()) >= 0.07f);
            }
        }
        // And it holds as the cast grows: ids 1..32 stay told apart.
        for(uint64_t i = 1; i <= 32; i++) {
            for(uint64_t j = i + 1; j <= 32; j++) {
                CHECK(apart(kerbal_complexion(i), kerbal_complexion(j)) >= 0.03f);
            }
        }
    }

    // ---- the save round trip ----------------------------------------------
    {
        CrewRoster r = load_crew_roster("res/data/kerbals.json");
        SaveMeta m;
        m.kerbals = "res/data/kerbals.json";
        for(const KerbalProfile &p : r.profiles) {
            SaveKerbal sk;
            sk.id = p.id;
            sk.name = p.name;
            for(int s = 0; s < NumSkills; s++) { sk.skill[s] = p.skill[s]; }
            m.roster.push_back(sk);
        }
        nlohmann::json j = saveMetaToJson(m);
        CHECK(j.contains("roster") && j["roster"].is_array());
        CHECK(j["roster"].size() == m.roster.size());
        CHECK(std::string(j["kerbals"]) == "res/data/kerbals.json");

        SaveMeta back = saveMetaFromJson(j);
        CHECK(back.roster.size() == m.roster.size());
        CHECK(back.kerbals == m.kerbals);
        for(size_t i = 0; i < m.roster.size(); i++) {
            CHECK(back.roster[i].id == m.roster[i].id);
            CHECK(back.roster[i].name == m.roster[i].name);
            for(int s = 0; s < NumSkills; s++) {
                CHECK(back.roster[i].skill[s] == m.roster[i].skill[s]);
            }
            /* The save carries no shade field, so a load re-derives it from the
               id it does carry. This breaks if a shade ever becomes stored and
               the save forgets to round-trip it. */
            KerbalProfile rebuilt;
            rebuilt.id = back.roster[i].id;
            CHECK(rebuilt.complexion() == r.profiles[i].complexion());
        }

        /* An absent roster key keeps the roster empty, which is how a save
           predating the roster reads: the --kerbals file stays in charge. */
        nlohmann::json bare;
        CHECK(saveMetaFromJson(bare).roster.empty());
    }

    // ---- a hand-edited roster in a save is refused, not half-read ---------
    {
        nlohmann::json j;
        j["roster"] = nlohmann::json::array();
        j["roster"].push_back({ {"id", 1}, {"name", "A"} });
        j["roster"].push_back({ {"id", 1}, {"name", "B"} });
        CHECK(expect_throw([&](){ saveMetaFromJson(j); }));

        nlohmann::json j2;
        j2["roster"] = nlohmann::json::array();
        j2["roster"].push_back({ {"id", 1}, {"name", "A"} });
        j2["roster"].push_back({ {"id", 2}, {"name", "A"} });
        CHECK(expect_throw([&](){ saveMetaFromJson(j2); }));

        nlohmann::json j3;
        j3["roster"] = nlohmann::json::array();
        j3["roster"].push_back({ {"name", "NoId"} });
        CHECK(expect_throw([&](){ saveMetaFromJson(j3); }));

        // Out-of-range skills clamp: they change how a kerbal is suited, not
        // whether the save reads.
        nlohmann::json j4;
        j4["roster"] = nlohmann::json::array();
        j4["roster"].push_back({ {"id", 1}, {"name", "A"},
                                 {"piloting", 99}, {"science", -4} });
        SaveMeta clamped = saveMetaFromJson(j4);
        CHECK(clamped.roster.size() == 1);
        CHECK(clamped.roster[0].skill[SkillPiloting] == 5);
        CHECK(clamped.roster[0].skill[SkillScience] == 1);
        CHECK(clamped.roster[0].skill[SkillEngineering] == 3);
    }

    // ---- a kerbal vehicle's profile id rides its own file -----------------
    {
        SaveShip s;
        s.name = "Ash";
        s.is_crew = true;
        s.profile_id = 3;
        nlohmann::json j = saveShipToJson(s);
        CHECK(j["profile_id"] == 3);
        CHECK(saveShipFromJson(j).profile_id == 3);

        // Absent = 0 = anonymous, which is what a pre-roster save reads as.
        nlohmann::json bare;
        bare["is_crew"] = true;
        CHECK(saveShipFromJson(bare).profile_id == 0);

        // A ship (not crew) never carries one.
        SaveShip ship;
        ship.name = "Racer";
        CHECK(!saveShipToJson(ship).contains("profile_id"));
    }

    if(failures == 0) {
        printf("test_roster: all checks passed\n");
        return 0;
    }
    printf("test_roster: %d FAILURE(S)\n", failures);
    return 1;
}
