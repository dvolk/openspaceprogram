// test_roster: the kerbal roster -- parsing, the claim rule, and the save
// round trip (src/crew.h, src/crew.cpp, save.h's roster keys).
// Runs from the repo root (needs res/):
//   make test   (or: ./test_roster)
//
// The roster is the identity half of crew: the named people a capsule seat
// draws from. WHO is where is derived from the fleet, not stored here, so this
// test covers the data model + the pick rule, not the aboard bookkeeping (that
// is tests/test_crew.cpp and the e2e EVA/board cases).

#include "crew.h"
#include "save.h"

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

    // ---- the claim rule ---------------------------------------------------
    {
        CrewRoster r = load_crew_roster("res/data/kerbals.json");
        std::vector<uint64_t> worn;
        // Nothing worn: the lowest id, so a fixture always gets the same person.
        const KerbalProfile *first = r.claim(worn);
        CHECK(first != nullptr);
        uint64_t lowest = r.profiles[0].id;
        for(const KerbalProfile &p : r.profiles) {
            if(p.id < lowest) { lowest = p.id; }
        }
        CHECK(first != nullptr && first->id == lowest);

        // Wear the first N, and the pick walks up the ids.
        worn.push_back(lowest);
        const KerbalProfile *second = r.claim(worn);
        CHECK(second != nullptr && second->id != lowest);
        worn.push_back(second->id);
        const KerbalProfile *third = r.claim(worn);
        CHECK(third != nullptr && third->id != lowest && third->id != second->id);

        // A spent pool is a legal state, not a failure: nullptr, and the seat
        // stays empty.
        for(const KerbalProfile &p : r.profiles) { worn.push_back(p.id); }
        CHECK(r.claim(worn) == nullptr);

        // An anonymous kerbal (profile id 0, a save predating the roster) does
        // not consume anyone.
        std::vector<uint64_t> anon;
        anon.push_back(0);
        const KerbalProfile *p = r.claim(anon);
        CHECK(p != nullptr && p->id == lowest);
    }

    // ---- the pick ignores file order --------------------------------------
    {
        const char *f = "/tmp/test_roster_order.json";
        writeFile(f, "{ \"kerbals\": [ "
                     "{ \"id\": 9, \"name\": \"Nine\" }, "
                     "{ \"id\": 4, \"name\": \"Four\" }, "
                     "{ \"id\": 7, \"name\": \"Seven\" } ] }");
        CrewRoster r = load_crew_roster(f);
        CHECK(r.profiles.size() == 3);
        const KerbalProfile *p = r.claim(std::vector<uint64_t>());
        CHECK(p != nullptr && p->id == 4);
        std::vector<uint64_t> worn = { 4 };
        p = r.claim(worn);
        CHECK(p != nullptr && p->id == 7);
        std::remove(f);
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
