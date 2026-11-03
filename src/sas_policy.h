/* sas_policy.h -- WHO GETS AN AXIS when the pilot's stick and a latched law ask for
   the same one (#425 step D-2, #421). Since #420 both arrive as demands on ONE vector
   per substep, so the only question left is how they combine before the allocator
   answers -- and that is a control preference, not physics. Both ends are pinned
   (test_authority):
     SasStickOwnsAxis  DEFAULT. A held key takes its own axis away from the law for as
                       long as it is held; the law keeps the axes nobody is steering.
                       Predictable to the person holding the key, and it cannot be
                       talked out of a maneuver by a law that is asking loudly
                       elsewhere. A law whose error lies ALONG the steered axis is
                       paused until the key comes up -- which is what steering means.
     SasKeepsAsk       The law keeps its full demand and the two simply sum: what the
                       hardware cannot follow is lost from the top of the sum, so on a
                       transverse axis a law asking thousands of N m leaves the pilot
                       well under 1 N m of the gimbals. This is what the game did
                       before the policy had a name; it is kept as the alternate
                       because it lets SAS finish a maneuver the pilot interrupts.
   The wheels' share was already per-line after d6ee482, so an unsteered axis cannot be
   spent by a saturated one either way; the two policies differ on the steered axis.

   Its own header because three doors spell the same choice -- settings.json holds an
   int, --sas-policy a word, the Settings window a combo row -- and inline spellers are
   what keep them from drifting. test_cli links cli.o alone, so these must not live in
   vehicle.cpp. */
#pragma once
#include <cassert>
#include <string>

enum SasPolicy {
    SasStickOwnsAxis = 0,
    SasKeepsAsk = 1
};
/* settings.json's int and the Settings window's combo order are these values. */
static_assert(SasStickOwnsAxis == 0 && SasKeepsAsk == 1,
              "the combo's item order and settings.json's int are the enum values");

inline const char *sasPolicyName(SasPolicy p) {
    return (p == SasKeepsAsk) ? "keep" : "axis";
}

inline SasPolicy sasPolicyFromName(const std::string &n) {
    if(n == "keep") { return SasKeepsAsk; }
    /* Every speller validates before calling: --sas-policy through CLI::IsMember (built
       from sasPolicyName above), settings.json as a range-checked int. Reaching here
       with anything else means a new caller forgot to validate. */
    assert(n == "axis" && "unknown sas policy name");
    return SasStickOwnsAxis;
}
