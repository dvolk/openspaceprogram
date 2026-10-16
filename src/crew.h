#pragma once

// crew.h -- the roster: the kerbals that EXIST, whether or not one is in the
// world right now.
//
// The roster is the CAST, not the world. mintCrewPool turns every entry into a
// suit Part standing on the home pad (part.h), and from then on the suit OWNS
// its identity (Part::kerbal): a seat, a walk and a save all read the person,
// never this file. Assignment IS the world -- on the pad, in a seat, or walking
// (part.h) -- so this roster deliberately carries no status: launch takes from
// the pad FIFO, recover returns, death does not (Ships::takeCrew / returnCrew).

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

/* The cast list. `id` is what a suit's KerbalProfile copies (Part::kerbal), so
   it is stable for the life of a save; `name` is unique, because a kerbal's
   display name IS its roster name. */
struct CrewRoster {
    std::vector<KerbalProfile> profiles;

    const KerbalProfile *find(uint64_t id) const;
    const KerbalProfile *byName(const std::string &name) const;
};

/* Parse + validate the roster file (res/data/kerbals.json). Throws
   std::runtime_error on a data bug: an empty roster, a missing or duplicate id
   or name, or a skill outside 1..5.

   The save reads the same fields more loosely (save.h: saveKerbalFromJson
   CLAMPS an out-of-range skill instead of refusing it). Deliberate: this file is
   ours, so a bad number is a bug to surface; a save is user-editable input, so a
   bad number is something to survive. */
CrewRoster load_crew_roster(const char *path);
