// angmath.h -- the one spelling of "an inverse trig of a bounded ratio", and the
// NaN law that makes having a spelling worth anything (#622). Header-only pure math.

#pragma once

#include <algorithm>
#include <cmath>

/* acosUnit(cosine) and asinUnit(ratio): a dot product of unit vectors, a matrix
   element, a length ratio, fed to the inverse function. Written once because the
   shape is everywhere and each hand-rolled version invites the same false comfort.
   THE CLAMP IS FOR FP DRIFT ONLY. A dot of two unit vectors can land at
   1 + 1e-16, and acos of that is a NaN that reads as "-nan deg" in a window or
   poisons a rotation; that is all the clamp catches. Where a ratio can go past 1
   for a REAL reason -- the sun's angular radius inside the sun -- the clamp is
   doing a different job, and saying so at the site is the caller's job.
   THE CLAMP IS NOT A GUARD. glm::clamp is min(max(x, lo), hi), glm's min/max are
   (y < x) ? y : x and (x < y) ? y : x, and std::clamp is built on the same
   comparison shape. Every relational test against NaN is false, so a NaN walks
   straight through either clamp and the inverse function hands back NaN. That is
   the ANSWER WE WANT -- the number stays visibly broken. Snapping a NaN onto an
   end of the range takes an IEEE std::fmax/std::fmin (they return the NON-NaN
   operand) or an explicit test like !(c > -1.0); a hand-written
   (c < -1 ? -1 : c > 1 ? 1 : c) clamp is NOT one of those, because NaN fails both
   comparisons -- equivalent to std::clamp, and test_angmath.cpp pins that
   equivalence by leaving exactly that mutation invisible.
   What turns a NaN into a confident wrong answer is the step AFTER this one:
   std::max(0.0, NaN) is 0.0, so cos(max(0.0, off - cone)) reads 1.0 -- "all your
   thrust, along nothing" (availableAccelAlong hit exactly this and answers it at
   its door, vehicle.cpp); std::abs(NaN) > 1.0 is false, so a range gate waves a
   NaN through. Both spellings are in the house already, so they are pinned here
   rather than left to be rediscovered.
   FAIL CLOSED AT THE DOOR: gate on !(x > 0.0) (covers zero, negative and NaN) or
   !std::isfinite(x) BEFORE the ratio is ever formed, and make sure the closed
   answer is the small one, not the confident one.
   THESE TAKE A RATIO, NOT A VECTOR: neither function normalises, so a short vector
   gives a small ratio and a wrong angle with no complaint. Normalise (or refuse)
   at the door; that also keeps the cost out of the hot aero and thrust loops. And
   where out-of-domain is not drift -- a sampling cap set below the periapsis it
   was meant to reach -- assert the precondition at the site (orbitsample.h does)
   instead of letting the clamp pretend it was deliberate.
   FLOAT IS DELETED rather than widened: a float ratio silently promoted to double
   moves bits two lines away from where the author meant it. The float mesh code
   (terragen.h's skirt ring, equirect.h's float lon/lat) keeps its own float
   spelling. */
inline double acosUnit(double c) { return std::acos(std::clamp(c, -1.0, 1.0)); }
inline double asinUnit(double s) { return std::asin(std::clamp(s, -1.0, 1.0)); }
inline double acosUnit(float) = delete;
inline double asinUnit(float) = delete;
