/* slew_mode.h -- which DIRECTION an autopilot hold points the nose at, and what each
   one is CALLED. It was an enum in the middle of vehicle.h until #498 grew it from seven
   modes to nine in four separate places at once: the trace's name switch, --autopilot's
   help prose, --autopilot's CLI::IsMember list and main.cpp's word->mode if-chain. Two of
   those (the save door and restoreSlewRequest) had already been folded into slewModeOk()
   for exactly that reason, and the word set was the remaining pair that would drift. So
   the enum, its names and its range now live here, and the command line reads one table.

   Its own header rather than a block in vehicle.h for the same reason as sas_policy.h:
   test_cli links cli.o alone, so the spellers must be inline here, and cli.cpp should
   not have to swallow the Vehicle class to know which words it accepts. */
#pragma once

#include <cstring>

/* Autopilot slew targets -- mutually exclusive (last one wins); the stick
   composes with them. Declared above ShipCmd because a ShipCmd now carries one. */
enum SlewMode {
    SlewNone = 0,
    SlewPrograde,
    SlewRetrograde,
    SlewRadialOut,   /* align nose away from the SOI body (radius vector) */
    SlewRadialIn,    /* align nose toward the SOI body */
    SlewNormal,      /* align nose with the orbital-plane normal (r x v) */
    SlewAntiNormal,  /* align nose against it */
    /* Steer at a direction a LAW named, rather than one the class invented (#606).
       The six above are names this class resolves itself; this one is resolved from
       the aim vector that ShipCmd(Aim, dir) carries, because a guidance law can
       MEASURE a turn through pointingTo() but until now could not ASK for one --
       slewDemand()/slewTargetDir() are protected, so the only channels were the six
       names plus the stick, and a pitch-program law had to re-implement the slew law
       outside the class that owns it. */
    SlewFree,
    SlewKillRot,     /* kill the spin */
    /* The AIR's velocity rather than the conic's (#498). Inside an atmosphere on a
       rotating body the conic's velocity is mostly the GROUND's own motion -- Kerbin
       carries the sea-level surface ~175 m/s -- so a prograde hold aims an 80 m/s
       airframe 16 deg off a nose the aero says is dead on the flow, and never
       converges. These two aim at the vector the forces themselves use
       (airRelativeVel), which is what render.cpp draws as the air prograde marker.
       Appended rather than inserted: the latched mode is saved as an int (save.cpp),
       so a new mode must not move the numbers an older save already wrote. */
    SlewAirPrograde,
    SlewAirRetrograde
};

/* The last enumerator, spelled as a NUMBER, because the reason it must stay last is a
   number: save.cpp stores the latched mode as an int, so appending is safe and INSERTING
   one silently relabels what an older save wrote (kill-rot would load as the mode slid in
   underneath it, and slewModeOk below would still wave it through). test_authority pins
   one past the end; this pins where the end is. If this trips, an insert is almost
   certainly the bug -- if the change really was an append, bump it and say which mode. */
static_assert((int)SlewAirRetrograde == 10,
              "SlewMode changed shape: append, never insert (a save stores these ints)");

/* One spelling of what a mode is CALLED (#616): the [slew] trace's mode= field and the
   line Game::select_vehicle prints when a mission parks were keeping separate word
   lists for the same state, which is how a log and the CHECK that reads it drift apart
   without anything noticing. Keep these words -- e2e cases match on them. */
inline const char *slewModeName(SlewMode m) {
    switch(m) {
        case SlewNone:       return "none";
        case SlewPrograde:   return "prograde";
        case SlewRetrograde: return "retrograde";
        case SlewRadialOut:  return "radial-out";
        case SlewRadialIn:   return "radial-in";
        case SlewNormal:     return "normal";
        case SlewAntiNormal: return "anti-normal";
        /* Named, not the fallback: an e2e CHECK on a free-aim law reads these rows,
           and a nameless row cannot be told apart from a construction bug (#606). */
        case SlewFree:       return "free";
        case SlewKillRot:    return "killrot";
        case SlewAirPrograde:   return "air-prograde";
        case SlewAirRetrograde: return "air-retrograde";
    }
    return "slew";   // an int outside the enum, which Command() cannot produce
}

/* Is an int read from OUTSIDE the sim a mode at all? Two doors take one as a plain
   number -- the save loader's stored latch and restoreSlewRequest() -- and both have
   to ask the same question, so it lives here rather than spelled twice against the
   last enumerator (which is what let the pair drift when a mode was added). */
inline bool slewModeOk(int m) {
    return m >= (int)SlewNone && m <= (int)SlewAirRetrograde;
}

/* The words --autopilot accepts, and the mode behind each (#498). This is ONE list,
   consumed by both CLI::IsMember and main.cpp's parse, because until now they were two
   hand-written sets that each had to be edited whenever a mode landed. It is deliberately
   NOT slewModeName()'s set: the trace says "killrot" while the flag says "kill-rot", and
   e2e cases pin both spellings, so the two lists may differ -- they just may no longer
   disagree about WHICH MODES EXIST. SlewNone and SlewFree are absent by design: a command
   line cannot carry the aim vector a free latch needs, and "none" is what no flag means. */
struct SlewModeWord {
    const char *word;   // null on the terminator row
    SlewMode mode;
};

inline const SlewModeWord *slewModeWords() {
    static const SlewModeWord words[] = {
        {"prograde", SlewPrograde},
        {"retrograde", SlewRetrograde},
        {"radial-out", SlewRadialOut},
        {"radial-in", SlewRadialIn},
        {"normal", SlewNormal},
        {"anti-normal", SlewAntiNormal},
        {"kill-rot", SlewKillRot},
        {"air-prograde", SlewAirPrograde},
        {"air-retrograde", SlewAirRetrograde},
        {nullptr, SlewNone},
    };
    return words;
}

/* CLI::IsMember has already narrowed the string to one of these words, so SlewNone
   here means a caller forgot to validate rather than meaning bad input. */
inline SlewMode slewModeFromWord(const char *w) {
    for(const SlewModeWord *mw = slewModeWords(); mw->word != nullptr; ++mw) {
        if(std::strcmp(mw->word, w) == 0) { return mw->mode; }
    }
    return SlewNone;
}
