#pragma once

// crew.h -- the roster: the kerbals that EXIST, whether or not one is in the
// world right now.
//
// A Kerbal vehicle (eva.h) is a kerbal that is IN the world. The roster entry
// is the identity that outlives the vehicle: name + skills, durable through a
// save. Today a vehicle exists for every kerbal, seated or not; once the seat
// becomes a record on the capsule Part (stage 2), the vehicle exists only while
// the kerbal is actually walking around, and this stays the single home of
// identity either way.
//
// There is deliberately NO status / assignment field here. The fleet is the
// truth about who is in the world, so a roster-side copy would drift: one
// delete that forgets to release a profile shrinks the pool forever, silently.
// "Unclaimed" is derived instead -- the lowest-id profile no kerbal vehicle is
// wearing -- so a kerbal always comes back when its vehicle goes away.

#include <cstdint>
#include <string>
#include <vector>

/* The skill scale. 1 = barely trained, 5 = the best in the business. Nothing
   reads these yet: stage 4 makes them load-bearing one skill per issue
   (science first, then piloting, then engineering). They are stored, saved and
   displayed now so the numbers have a home before they have an effect. */
enum Skill { SkillPiloting, SkillScience, SkillEngineering, NumSkills };

const char *skill_name(Skill s);

struct KerbalProfile {
    uint64_t id = 0;
    std::string name;
    int skill[NumSkills] = {3, 3, 3};
};

/* Every kerbal in the game. `id` is what a kerbal vehicle points at
   (Kerbal::profileId), so it is stable for the life of a save; `name` is
   unique, because a kerbal's display name IS its roster name. */
struct CrewRoster {
    std::vector<KerbalProfile> profiles;

    const KerbalProfile *find(uint64_t id) const;
    const KerbalProfile *byName(const std::string &name) const;

    /* The next kerbal for a seat: lowest id absent from `worn` (the profile ids
       vehicles already carry). Deterministic, so a fixture that launches one
       ship always gets the same person. nullptr = the pool is empty, which is
       a legal state: the seat stays empty and the ship flies uncrewed. */
    const KerbalProfile *claim(const std::vector<uint64_t> &worn) const;
};

/* Parse + validate the roster file (res/data/kerbals.json). Throws
   std::runtime_error on a data bug: an empty roster, a missing or duplicate id
   or name, or a skill outside 1..5.

   The save reads the same fields more loosely (save.h: saveKerbalFromJson
   CLAMPS an out-of-range skill instead of refusing it). Deliberate: this file is
   ours, so a bad number is a bug to surface; a save is user-editable input, so a
   bad number is something to survive. */
CrewRoster load_crew_roster(const char *path);
