#pragma once

// crew.h -- the roster: the kerbals that EXIST, whether or not one is in the
// world right now.
//
// The roster is the CAST, not the world. mintCrewPool turns every entry into a
// suit Part standing on the home pad (part.h), and from then on the suit OWNS
// its identity (Part::kerbal): a seat, a walk and a save all read the person,
// never this file. Assignment IS the world -- on the pad, in a seat, or walking
// (part.h) -- so this roster deliberately carries no status: launch takes from
// the pad FIFO, recover returns, death does not (vehicle.cpp: takeCrew /
// returnCrew).

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

/* The skill scale. 1 = barely trained, 5 = the best in the business. Nothing
   reads these yet: stage 4 makes them load-bearing one skill per issue
   (science first, then piloting, then engineering). They are stored, saved and
   displayed now so the numbers have a home before they have an effect. */
enum Skill { SkillPiloting, SkillScience, SkillEngineering, NumSkills };

const char *skill_name(Skill s);

/* The suit texture is one flat green, so the tint IS the kerbal's complexion:
   it multiplies the lit color (partsShader.fs), which keeps the shade reading
   as a suit under sun and shadow rather than as a flat sticker. The cast is
   deliberately NOT all kerbal-green -- hue is what tells two of them apart at
   a glance, so each person gets their own (issue #359: a seated or parked suit
   is never drawn, so today that means the one walking kerbal plus the UI dots).
   Both the complexion and the tint are derived from the roster id -- stable for
   the life of a save -- so a shade follows the person through pad, seat and
   walk with nothing to store or save. id 0 is an anonymous suit: one a
   pre-roster save carries, or one minted for a kerbal spawned when the pad
   pool was spent (#291). It wears the texture's own green. */
glm::vec3 kerbal_complexion(uint64_t id);   // the fully lit suit color
glm::vec3 kerbal_tint(uint64_t id);         // what multiplies the texture to get it

/* The suit texture's flat green, exactly as utils/gen_kerbal.py writes it into
   textures/kerbal.png. It is the thing the tint multiplies, and the UI needs it
   to show a swatch: a table row has no texture, only the fully lit complexion.
   tests/test_roster.cpp pins it to that literal: a mirrored constant with no
   owner drifts from the asset it mirrors. */
const glm::vec3 kKerbalBaseGreen = glm::vec3(60.0f, 200.0f, 60.0f) / 255.0f;

struct KerbalProfile {
    uint64_t id = 0;
    std::string name;
    int skill[NumSkills] = {3, 3, 3};

    glm::vec3 tint() const { return kerbal_tint(id); }
    /* The fully lit suit color: base green x tint. */
    glm::vec3 complexion() const { return kerbal_complexion(id); }
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
