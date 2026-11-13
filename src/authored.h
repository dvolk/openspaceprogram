/* #726, #729: THE ONE SPELLING OF "NOBODY SOURCED THIS".

   A shipped body carries several scalar readings a source may or may not state. Each one
   used to spell absence as its own most-natural zero, and every one of those zeros is a
   READING:

     albedo 0.0        a body that absorbs everything -- the HOTTEST case in the game
     temperature 0.0   0 K, and for a star a luminosity of 0: "the system is dark"
     even_share 1.0    "this ground does not swing" -- exactly what Venus states, and what
                       Mercury states nothing about

   Absence is therefore a NEGATIVE sentinel, which no legal value can collide with, and the
   loader refuses to author one: a file cannot write its own absence (#726). The readout half
   of the law is #727's -- an absent reading renders as an em-dash, never as 0.0 -- and the
   behaviour half is this file's: absence is a STATE, so two bodies that behave alike can still
   read apart. Venus (a stated range of about zero) and Mercury (no statement) are that pair.

   There is no default behind any of these. A default is a reading the file does not have; the
   argument, per field, is at its site below. */

#pragma once

/* [dimensionless] The share of incident starlight a body reflects back to space. Legal range
   [0,1) (system.cpp's loader bound), so -1 is unreachable from a file. The reason this one
   moved off 0.0 first: a Bond albedo of 0.0 is not a blank, it is an absorber, and the
   --solar-log readout keyed off `albedo > 0.0` printed the same dash for "not authored" and
   "absorbs everything" while the physics ran hot. */
inline constexpr double kAlbedoUnauthored = -1.0;
inline bool albedoAuthored(double a) { return a >= 0.0; }

/* [K] A body's own surface/effective temperature -- for a star, the photospheric T behind the
   WHOLE system's insolation. It used to default to 0.0 and be tested with `> 0.0`, which did
   two different damage at once: a planet's blank read as 0 K, and a star's blank silently made
   a dark sun (starLuminosity takes its fourth power and asserts a non-negative input, so
   feeding it the sentinel is a crash rather than a dash -- hence the guard at the one
   composition site, src/system.cpp). Authoring 0.0 is now refused at load: 0 K is not a
   temperature, it is a blank, and a file that means "cold" can say 40 K.
   NOT to be confused with drag.h:airTemperature returning 0.0 for a VACUUM -- that one is a
   deliberate "no air to have a temperature" and is pinned as such (00db0834).
   The predicate treats 0.0 as absent too, even though the loader cannot produce it: call sites
   that hand a rung of the anchor chain to groundAnchor_K() pass a bare 0.0 for "this rung has
   nothing", and an exact 0 K is not a reading anyone could mean. The sentinel is for the FIELD
   (so a struct says on its face that nobody sourced it); `> 0.0` is for the PREDICATE. */
inline constexpr double kThermalUnauthored = -1.0;      // [K] not 0.0
inline bool thermalAuthored(double t_K) { return t_K > 0.0; }

/* [1] The even share of the surface swing's law (src/groundtemp.h): how much of a body's own
   emission is spread evenly rather than falling where the star lights it. Defaulting absence
   into 1.0 printed Venus's answer for Mercury, and there is no better fallback: "no swing"
   gives a constant hot ground under a midnight sun (least defensible exactly where the body is
   airless), and "bare rock" (0.0) invents a swing no source stated. Absence DASHES. */
inline constexpr double kEvenShareUnauthored = -1.0;
inline bool evenShareAuthored(double f) { return f >= 0.0; }

/* [phase radians] How far the ground's temperature lags the star's light (#794): the angle the
   body turns through while the skin catches up, so 0 is instantaneous and one rotation is 2*pi.
   Absence means NO lag, which -- unlike the knobs above -- is not a reading the file lacks but
   the state the game has always run in, exactly as kFollowUnauthored means "no coupling". It is
   still a negative sentinel so a stated 0.0 ("this ground peaks at noon, and that is a fact")
   cannot be confused with nobody saying anything.
   The loader bounds it at one rotation, and that bound is load-bearing against a units mistake:
   a page that states its daily max at 14:00 invites someone to author `14`, which is hours, not
   radians -- and hours would land inside any range that lets a lag be a lag.
   Hours are NOT the unit here, and a readout that wants them converts with the body's MEASURED
   mean solar day (system.h:measureSolarDay), never the sidereal rotation. */
inline constexpr double kLagTauUnauthored = -1.0;
inline bool lagTauAuthored(double t) { return t >= 0.0; }
/* Whether any work follows. The sentinel and a stated 0.0 both answer "the ground tracks the
   light", and the pair is one fact (#636); the readout asks the other question -- did the file
   state a lag at all -- and dashes on !lagTauAuthored. */
inline bool lagTauActive(double t) { return t > 0.0; }

/* [1] How far the air nearest the ground is allowed to follow the ground's temperature
   (#724 rung D, drag.h/groundtemp.h). Absence means NO coupling -- the air keeps its authored
   profile -- which is both the conservative default and the state that keeps every existing
   flight golden untouched while the mechanism exists. 0.0 is a legal READING ("the air does not
   follow, and that is a fact about this air"), so the sentinel is the negative again. */
inline constexpr double kFollowUnauthored = -1.0;
inline bool followAuthored(double k) { return k >= 0.0; }
/* A caller that wants to know whether ANY work follows should ask this rather than spelling
   `followAuthored(k) && k > 0.0`: the sentinel and a stated 0.0 both mean "nothing to
   compute", and the pair is one fact (#636). The readout asks the OTHER question -- whether
   the file stated a position at all -- and dashes on !followAuthored. */
inline bool followActive(double k) { return k > 0.0; }
