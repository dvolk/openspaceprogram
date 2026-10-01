// bodylimits.h -- the derived per-body limits as pure math (STL only: no
// game state), so the loader computes them and tests/ can pin them without
// the render chain. Design + rationale: tmp/body-limits-plan.txt.
//
//   shellEdge(...)          the near-body shell above the atmosphere
//   soiPatchedConic(...)    classical SOI law: a*(m/M)^(2/5)
//   soiHill(...)            Hill sphere: a*(m/(3M))^(1/3)
//   inertialSoi(...)        the law value, lifted clear of the shell
//   validateBodyLimits(...) the load-time ordering asserts

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "constants.h"

/* Which SOI law a system uses: the system JSON's root-level "soi_law"
   field. patched_conic is the classical sphere-of-influence formula -- it
   reproduces the KSP wiki SOI table (the table was generated with it);
   hill is the tidal-radius alternative (the real solar system's old
   hardcoded values). */
enum class SoiLaw { PatchedConic, Hill };

inline SoiLaw soiLawFromName(const std::string &name) {
    if(name == "hill") { return SoiLaw::Hill; }
    if(name == "patched_conic") { return SoiLaw::PatchedConic; }
    throw std::runtime_error("system: unknown soi_law '" + name
                             + "' (want \"patched_conic\" or \"hill\")");
}

/* The near-body (rotating-frame) shell edge [m above sea level]: the
   atmosphere top plus a 20% low-orbit band, floored at kMinShell so tiny
   and airless bodies keep a workable surface frame. The floor and the
   fraction together guarantee shell >= top + kSoiMargin for EVERY top
   (the floor covers top < 50 km, the fraction the rest), so the air never
   reaches past the rot frame's enter band: the co-rotation assumption of
   the drag path holds by construction (issues #60/#97), and the science
   HighOrbit cut lines up with leaving the surface frame on every body. */
inline double shellEdge(double atmoTop) {
    return std::max(kMinShell, (1.0 + kLowOrbitFrac) * atmoTop);
}

/* The classical patched-conic sphere of influence [m from center]. */
inline double soiPatchedConic(double a, double m, double M) {
    if(a <= 0.0 || m <= 0.0 || M <= 0.0) { return 0.0; }
    return a * std::pow(m / M, 0.4);
}

/* The Hill sphere [m from center]: where the parent's tide starts to
   dominate the body's own gravity. */
inline double soiHill(double a, double m, double M) {
    if(a <= 0.0 || m <= 0.0 || M <= 0.0) { return 0.0; }
    return a * std::cbrt(m / (3.0 * M));
}

inline double soiByLaw(SoiLaw law, double a, double m, double M) {
    return (law == SoiLaw::Hill) ? soiHill(a, m, M)
                                 : soiPatchedConic(a, m, M);
}

/* The inertial SOI [m from center]: the law value, lifted so that
   (a) this frame's enter band (soi - kSoiMargin) stays clear of the
   shell's exit band (rotSoi + kSoiMargin) -- overlapping bands would flap
   a ship between frames -- and (b) the highest standard spawn bed
   (inertial-orbit at kInertialOrbitFrac * shell) still resolves INSIDE
   the body's own frame. The lift is what makes tiny moons workable
   (Gilly's law value 126 km sits below its 113 km shell + beds;
   Phobos's law value is kilometres). */
inline double inertialSoi(double lawSoi, double rotSoi, double shell) {
    return std::max({lawSoi,
                     rotSoi + 2.0 * kSoiMargin,
                     rotSoi + (kInertialOrbitFrac - 1.0) * shell
                         + kSoiMargin});
}

/* The load-time ordering asserts (system.cpp calls this per body). The
   values are derived, so a failure means a code regression or hand-edited
   JSON -- the message names the body and the broken invariant. The shell
   is recomputed from atmoTop (shellEdge is deterministic). */
inline void validateBodyLimits(const std::string &name, double radius,
                               double sea_level, double atmoTop,
                               double rotSoi, double inertSoi) {
    const auto fail = [&name](const std::string &what) {
        throw std::runtime_error("system: " + name + ": " + what);
    };
    const double shell = shellEdge(atmoTop);
    if(shell < atmoTop + kSoiMargin) {
        fail("atmosphere top reaches past the near-body shell's enter band");
    }
    if(rotSoi - kSoiMargin <= radius + sea_level) {
        fail("near-body shell is un-enterable (soi - margin at or below the surface)");
    }
    // 100 m of slack: rotSoi is built from the JSON's DOUBLE radius while
    // TerrainBody::radius is float -- star-scale rounding is tens of metres,
    // real regressions are kilometres.
    if(rotSoi + 100.0 < radius + sea_level + shell) {
        fail("near-body SOI is smaller than the derived shell");
    }
    if(inertSoi < rotSoi + 2.0 * kSoiMargin) {
        fail("inertial SOI does not clear the near-body shell's hysteresis band");
    }
    if(inertSoi <= radius + sea_level + kInertialOrbitFrac * shell) {
        fail("inertial SOI does not contain the inertial-orbit spawn bed");
    }
}
