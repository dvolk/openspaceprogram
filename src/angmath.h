// angmath.h -- the one spelling of "a cosine into acos", and the NaN law that
// makes having a spelling worth anything (#622). Header-only pure math.

#pragma once

#include <algorithm>
#include <cmath>

/* acos() of a cosine: a dot product of unit vectors, a matrix element, an
   eccentricity term. Written once because the shape is everywhere (~10 sites)
   and because each hand-rolled version invites the same false comfort.
   THE CLAMP IS FOR FP DRIFT ONLY. A dot of two unit vectors can land at
   1 + 1e-16, and acos of that is a NaN that reads as "-nan deg" in a window
   or poisons a rotation; that is all the clamp catches.
   THE CLAMP IS NOT A GUARD. glm::clamp is min(max(x, lo), hi) and glm's min/
   max are (b < a) ? b : a; std::clamp is the same comparison shape. Every
   relational test against NaN is false, so a NaN cosine walks straight
   through either clamp and acos hands back NaN. That is the ANSWER WE WANT -- the
   number stays visibly broken. Snapping a NaN onto an end of acos' range takes an
   IEEE std::fmax/std::fmin (they return the NON-NaN operand) or an explicit
   test like !(c > -1.0); a hand-written (c < -1 ? -1 : c > 1 ? 1 : c) clamp is
   NOT one of those, because NaN fails both comparisons -- equivalent to
   std::clamp, and test_angmath.cpp pins that equivalence by leaving exactly that
   mutation invisible.
   What turns a NaN into a confident wrong answer is the step AFTER this one:
   std::max(0.0, NaN) is 0.0, so cos(max(0.0, off - cone)) reads 1.0 -- "all
   your thrust, along nothing" (availableAccelAlong hit exactly this and
   answers it at its door, vehicle.cpp); std::abs(NaN) > 1.0 is false, so a
   range gate waves a NaN through. Both spellings are in the house already, so
   they are pinned here rather than left to be rediscovered.
   FAIL CLOSED AT THE DOOR: gate on !(x > 0.0) (covers zero, negative, NaN) or
   !std::isfinite(x) BEFORE a cosine is ever formed, and make sure the closed
   answer is the small one, not the confident one.
   CALLERS ASSUME UNIT VECTORS: this does not normalise, so a short vector
   gives a small cosine and a wrong angle with no complaint. Normalise (or
   refuse) at the door; that also keeps the cost out of the hot aero and
   thrust loops.
   terragen.h's skirt-ring site keeps its own std::acos(glm::clamp(..., -1.0f,
   1.0f)): that mesh is float math end to end, and routing it through a double
   would move generated geometry for no gain. */
inline double acosUnit(double c) { return std::acos(std::clamp(c, -1.0, 1.0)); }
