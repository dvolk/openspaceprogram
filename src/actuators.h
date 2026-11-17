#pragma once

// actuators.h -- turning a DEMANDED attitude torque into per-actuator setpoints
// the actuators can actually deliver (glm + vector only, so tests/ can pin it
// without the render/physics chain, like gimbal.h and drag.h).
//
// #576 is why this exists. A reaction wheel used to be a rating with no
// direction: the capability model credited the summed wheel rating in full about
// whichever axis it was asked for, so a drum mounted along the stack -- which is
// how every capsule and every reaction wheel is bolted on -- bought the hull full
// pitch and yaw from a wheel that can only roll it. The honest answer asks two
// questions over ONE set of rows, so a law cannot plan against one model and be
// delivered by another:
//
//   actuatorTorqueAbout()  what these actuators can bring to bear ABOUT an axis
//                          (the capability the slew law's alpha budgets with);
//   actuatorSolveInto()    what they actually deliver against a demanded vector,
//                          and what each one's share of it is (actuatorSolve() is
//                          the same solve on a one-shot Scratch, for tests).
//
// The two answer different questions over the same rows: the first is the most
// torque they can produce ALONG a direction (each actuator free to allocate
// itself), the second is how they split one specific demanded vector. The first
// is therefore an upper bound on the second, never a promise that the second
// reaches it -- the stick's multi-key demand (#577) is the everyday case where it
// does not, because a demand can lean harder on one line than that line's rating.
// What the shared row set buys is that the law and the actuators cannot disagree
// about WHICH lines exist and how far each reaches.
//
// #426's control surfaces are the same row with a different effect vector -- a
// surface's torque-per-radian-of-command and its max_deflection as the limit -- so
// Vehicle::actuateAttitude now feeds this solver wheels AND surfaces in one call and a
// setpoint comes back in radians for the aero step to spend. The wheel fixtures here
// stay the small-n bed: a ring is the same arithmetic with more rows.
//
// Not the same problem as rcs.h's nozzle allocator, which solves force AND
// torque over ONE-WAY nozzles (nnls, c in [0,1]). This one is torque-only: since
// #595 it can carry one-way lines (a thruster bell winds up a turn and cannot
// brake one the other way), but it never gains a FORCE row. The two allocators stay
// separate on purpose and the priority between them lives in Vehicle, not in a
// merged solve: the player's translation keys get first claim on the bells
// (rcs.h), and the attitude rows Vehicle builds from that same ring carry only the
// capacity the translation solve left. Merging the two into one weighted solve is
// how a ship ends up refusing to translate because it is busy turning.
// Worth knowing both exist before a third demand shape shows up and someone
// merges them.
// It now carries a Scratch of its own too, the same shape as rcs::Scratch:
// actuators::Scratch held per hull (Vehicle::attScratch_) and passed into
// actuatorSolveInto, so an actuated substep allocates nothing after warmup. It used
// to pay ten allocations -- two in actuateAttitude (the concatenated rows, the
// setpoint vector) and eight in the solve, whose six vectors plus the lines vector's
// growth make eight (#645). heaptrack 2026_11_17, 42,000-tick Kerbin ascent: 91,830
// = 10.0 x 9,183 actuateAttitude calls. That bed's hull carries no RCS part, so the
// bell mop-up never ran in it and the count is the single-solve path only; the
// mop-up's buffers are reused by the same Scratch (#595) but are not in that number.
// What the Scratch does NOT do is avoid the row copy: the drums-and-fins solve
// concatenates the authority rows into the hull's own reused attRows_ (40 bytes a
// row), so it allocates nothing but the copy is real work, and the row argument
// stays a CONTAINER TEMPLATE anyway because the mop-up passes an InlineVec straight
// through and actuatorTorqueAbout reads the authority rows in place. #668 took the
// four biggest away before any of this: the AttitudeAuthority row sets (wheels,
// wheelParts, surfaces, surfaceParts) are InlineVec now, so the walk that runs every
// actuation substep AND once per frame per open window allocates nothing (measured
// separately, 2026_11_06 on 20 s of atmospheric flight: 85,386 allocations, 65% of
// that run's steady-state total).
// The UI calls attitudeAuthority() once per FRAME (Game Debug's steering rows
// ask the live answer, Systems asks the hardware one), plus two more per
// --steering-log line. Closed windows draw no body at all (ui.h), so the default
// flight layout pays none of this. #594 landed the other half of the old ask: ONE
// AttitudeAuthority per substep, answered once for the stick and every latched law
// together (#420).

#include <array>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

/* One actuator. `axis` is the line it levers torque along, SCALED by how much a
   unit of its own demand buys: for a wheel that is a unit vector along the drum's
   spin axis and the demand is N m; for #426's surface it is the mount-derived
   torque direction at N m per radian and the demand is radians of deflection.
   `limit` bounds that demand either way (a wheel winds either sense, so the
   direction is a line, not a sense) -- unless `one_way`, which makes the row a bell
   rather than a drum: it can only push ALONG +axis, because thruster gas goes out
   one way (#595). A zero-length or NaN axis buys nothing, and the negated compare
   is what makes that true -- it is the guard for a poisoned row, since a NaN axis
   must never be creditable. An infinite one does get through, and nothing authored
   can produce one, so there is no guard pretending to. */
struct ActuatorRow {
    glm::dvec3 axis = glm::dvec3(0.0);
    double limit = 0.0;
    bool one_way = false;
};

/* The N m these actuators can bring to bear ABOUT `dir` (any length; the
   direction is what counts, and a zero or NaN direction buys nothing). A drum
    leaning 90 deg off the asked axis contributes nothing about it, which is the
    whole content of #576, and two drums on one line buy that line's capacity
    once. This is the most they can produce along `dir` with each free to spend
    itself: actuatorSolve's answer about that direction can only ever be shorter.
    A triad housing (#593) is creditable in more than one direction at once, so
    its number about an oblique axis is the sum of three drums' cosines -- up to
    sqrt(3) x its rating -- which is what three orthogonal drums genuinely can do,
    and also why the old single isotropic number understated an oblique axis.
    A ONE-WAY row is credited only where it can actually push: a bell 91 deg off the
    asked axis buys nothing about it, where a drum on the same line would be credited
    as if it could wind the other way. That asymmetry is #595's capability half, and
    it is what stops a ring fitted the wrong way round from making a hull look
    steerable. */
template<class Rows>
inline double actuatorTorqueAbout(const Rows &rows,
                                 const glm::dvec3 &dir) {
    /* Templated on the row CONTAINER (std::vector or InlineVec, #668) because all
       it needs is size() and a walk: the authority rows live in an InlineVec now,
       and an interface that forced them into a std::vector would put the four
       allocations right back where heaptrack found them. */
    const double d2 = glm::dot(dir, dir);
    if(!(d2 > 0.0)) { return 0.0; }
    const glm::dvec3 u = dir / std::sqrt(d2);
    double t = 0.0;
    for(const ActuatorRow &r : rows) {
        if(!(glm::dot(r.axis, r.axis) > 0.0) || !(r.limit > 0.0)) { continue; }
        const double c = glm::dot(r.axis, u);
        t += r.limit * (r.one_way ? (c > 0.0 ? c : 0.0) : std::fabs(c));
    }
    return t;
}

/* The R/P/Y split of a torque or an actuator set, stated in a hull's attitude
   basis (Vehicle::attitudeBasis(), whose columns are [right, up, nose]). Roll is
   about the nose, pitch about the right wing, yaw about the up axis -- the order
   the flight windows print and the order every readout's "R/P/Y" label promises,
   so it lives here once rather than in each call site's dot products. */
inline std::array<double, 3> torqueRpY(const glm::dvec3 &tau, const glm::dmat3 &ax) {
    return { std::fabs(glm::dot(tau, ax[2])), std::fabs(glm::dot(tau, ax[0])),
             std::fabs(glm::dot(tau, ax[1])) };
}

template<class Rows>
inline std::array<double, 3> actuatorRpY(const Rows &rows,
                                         const glm::dmat3 &ax) {
    return { actuatorTorqueAbout(rows, ax[2]), actuatorTorqueAbout(rows, ax[0]),
             actuatorTorqueAbout(rows, ax[1]) };
}

namespace actuators {

/* One actuator LINE: the direction it levers torque along, the N m all the rows
   grouped onto it can carry together, and whether that reach is a SENSE (+q only,
   a thruster bell: #595) or a line a drum winds either way. */
struct Line {
    glm::dvec3 dir = glm::dvec3(0.0);
    double capacity = 0.0;
    bool one_way = false;   // the line carries +q only (a bell, not a drum: #595)
};

/* Every work buffer the solve uses, as vectors sized to the biggest solve seen.
   actuateAttitude runs every physics SUBSTEP and holds one of these per hull
   (Vehicle::attScratch_), so an actuated substep allocates nothing after warmup;
   actuatorSolve() below wraps a one-shot Scratch for tests and for any caller with
   no hull to hold one. A Scratch belongs to ONE solve at a time: the two solves of
   a substep -- #595's bell mop-up after the drums-and-fins one -- take it in turn,
   never at once. The ROW SET is the caller's, never the Scratch's: passing the
   Scratch's own buffer as `rows` would let out.lines.clear() erase it mid-walk, and
   the signature has nothing to stop that. Every buffer is resized and overwritten
   before it is read, so stale capacity is never stale data; what a caller must NOT
   do is keep the setpoints across a second solve, since they describe the row set of
   the last one. */
struct Scratch {
    std::vector<double> setpoints;  // solution: per-row ask (N m, radians for a fin)
    std::vector<Line> lines;        // the grouped lines, rebuilt every solve
    std::vector<int> lineOf;        // per row: which line it fell on, -1 if a dead row
    std::vector<double> sense;      // per row: +1/-1 along its line's chosen sense
    std::vector<double> share;      // per row: its N m of its line's capacity
    std::vector<double> mag;        // per row: |axis|, its N m per unit of setpoint
    /* Per line: the N m it is carrying. Zeroed at the start of every solve -- not for
       hygiene but because it is where the coordinate descent starts, and the descent's
       answer is not unique (see the near-parallel pair above), so a carried-over q
       makes a hull's per-drum split depend on last substep's demand. */
    std::vector<double> q;
};

}  // namespace actuators

/* Fit `demand` (N m) into what the rows can deliver: one scalar per row, inside
   +/- its limit, so that Sum s_k * axis_k lands as close to `demand` as the
   ratings allow. The Scratch's `setpoints` is resized to rows and answers what each
   actuator is asked to do -- a wheel's N m along its own axis (which is the number
   #579's momentum store wants), a fin's radians -- so the caller can bill the
   actuator rather than only the sum. Returns the DELIVERED vector: it is shorter than
   `demand` wherever the rows cannot follow it, so an axis the hull has no
   actuator for buys nothing rather than a pretend torque.
 *
   Two rows on one line share that line's capacity as a sum, matching
   actuatorTorqueAbout. The solve is cyclic coordinate descent over the lines
   (each step takes that line's best contribution given the others, clamped to its
   capacity). The problem is convex and the sweep count is fixed, so this cannot
   hang and costs a few hundred flops for a handful of rows -- it runs once per
   substep per hull (Vehicle::actuateAttitude), not per part and not per law.
 *
   Two known properties, neither visible today and both load-bearing later. Both
   measured on two 100 N m drums whose lines are THETA apart, asked for 120 N m along
   the bisector, where the fair split is 60/60:
   - Descent converges fast when lines are orthogonal (one sweep) and not at all when
     they are near-parallel but past the grouping tolerance: every theta from 1e-4 to
     0.0175 rad (0.006 .. 1 deg) answers ~100/20 -- 99.62/20.39 at 0.0175 -- and
     crossing the tolerance jumps the split from 60/60 to 100/20 in one step. The
     DELIVERED vector barely notices: cross-track error over |demand| is 0.0033% at
     1e-4 rad and 0.578% at 1 deg -- while the per-row answer is 100 N m on one drum
     and 20 on the other. #579 bills momentum from those setpoints, so it must either
     split a converged line total pro-rata or widen the grouping.
   - Rows grouped into one line take the FIRST row's direction as the line's, and get
     that line's answer split pro-rata by rating, so a row a fraction of a degree off
     the line is credited along the LINE. Summing setpoints back through each row's own
     axis then differs from the returned vector by ~ q * angle: 6e-5 N m at
     theta = 1e-6, zero at theta = 0 (where there is nothing to be off). */
template<class Rows>
inline glm::dvec3 actuatorSolveInto(const Rows &rows, const glm::dvec3 &demand,
                                    actuators::Scratch &out) {
    out.setpoints.assign(rows.size(), 0.0);
    /* A NaN demand asks for nothing, and it has to be answered here: the sweep's
       clamp reads `fabs(want) <= capacity`, which a NaN fails into the NEGATIVE
       limit, i.e. a poisoned demand would command full torque against itself. */
    if(!std::isfinite(demand.x) || !std::isfinite(demand.y)
    || !std::isfinite(demand.z)) { return glm::dvec3(0.0); }
    /* Per row: which line it fell on, whether it points along or against that
       line's chosen sense, and how much of the line's capacity is its share.
       assign() into the Scratch rather than a fresh vector per solve: the capacity
       a hull has already reached is what gets reused, and a resize-down leaves the
       rows past `rows.size()` unread (#645). lines, lineOf, setpoints and q must be
       reset -- dropping any of the first three reddens test_actuators, once by
       answering a 100 N m drum's -70 N m demand with -35, and dropping q's zeroing
       flips the near-parallel split the doc block above states as measured fact. The
       sense/share/mag fills below are defensive by contrast: the solve writes an
       entry for every live row and never reads a dead row's, so no fixture can pin
       them. */
    out.lines.clear();
    out.lineOf.assign(rows.size(), -1);
    out.sense.assign(rows.size(), 0.0);
    out.share.assign(rows.size(), 0.0);
    out.mag.assign(rows.size(), 0.0);
    for(size_t k = 0; k < rows.size(); k++) {
        const double m2 = glm::dot(rows[k].axis, rows[k].axis);
        if(!(m2 > 0.0) || !(rows[k].limit > 0.0)) { continue; }
        const double m = std::sqrt(m2);
        const glm::dvec3 dir = rows[k].axis / m;
        int hit = -1;
        /* A one-way row's line is a SENSE, not a line: two bells that push the hull
           the same way about the same axis share one line's capacity, and two that
           oppose are two lines, because cancelling one with the other costs propellant
           on both. Two-sided rows keep the old fabs() grouping -- two drums mounted
           head to tail are one actuator line with twice the rating. A one-way row
           never joins a two-sided line and vice versa, so a line's clamp is unambiguous
           (the tolerance is for axes that agree to within rounding, not for "nearly
           parallel", which is a real geometry with two nearly-independent lines). */
        const double tol = 1.0 - 1e-9;
        for(size_t j = 0; j < out.lines.size(); j++) {
            if(out.lines[j].one_way != rows[k].one_way) { continue; }
            const double d = glm::dot(dir, out.lines[j].dir);
            if(rows[k].one_way ? d > tol : std::fabs(d) > tol) { hit = (int)j; break; }
        }
        if(hit < 0) {
            out.lines.push_back({ dir, 0.0, rows[k].one_way });
            hit = (int)out.lines.size() - 1;
        }
        const double w = rows[k].limit * m;
        out.lines[hit].capacity += w;
        out.lineOf[k] = hit;
        out.sense[k] = (glm::dot(dir, out.lines[hit].dir) >= 0.0) ? 1.0 : -1.0;
        out.share[k] = w;
        out.mag[k] = m;
    }
    if(out.lines.empty()) { return glm::dvec3(0.0); }

    out.q.assign(out.lines.size(), 0.0);        // N m each line is currently carrying
    for(int sweep = 0; sweep < 32; sweep++) {
        double moved = 0.0;
        for(size_t j = 0; j < out.lines.size(); j++) {
            /* This line's best contribution given every OTHER line's: the residual
               it is free to answer, projected on its own direction, clamped to its
               capacity -- and to its SENSE, which for a bell is [0, capacity]: it can
               wind up the turn, never brake one the other way (#595). */
            glm::dvec3 rest(0.0);
            for(size_t i = 0; i < out.lines.size(); i++) {
                if(i != j) { rest += out.q[i] * out.lines[i].dir; }
            }
            const double want = glm::dot(demand - rest, out.lines[j].dir);
            const double hi = out.lines[j].capacity;
            const double lo = out.lines[j].one_way ? 0.0 : -hi;
            const double take = (want > hi) ? hi : (want < lo ? lo : want);
            moved = std::fmax(moved, std::fabs(take - out.q[j]));
            out.q[j] = take;
        }
        if(moved <= 1e-12 * (glm::length(demand) + 1.0)) { break; }
    }
    glm::dvec3 delivered(0.0);
    for(size_t j = 0; j < out.lines.size(); j++) { delivered += out.q[j] * out.lines[j].dir; }
    for(size_t k = 0; k < rows.size(); k++) {
        const int j = out.lineOf[k];
        if(j < 0) { continue; }      // a dead row is asked to do nothing
        const double along = out.q[j] * (out.share[k] / out.lines[j].capacity);
        out.setpoints[k] = out.sense[k] * along / out.mag[k];
    }
    return delivered;
}

/* Test-side wrapper: one-shot Scratch, the per-row answer copied into the caller's
   vector -- the same bargain rcs.h's rcsAllocate makes. No game code calls it; the
   hull's own Scratch is the only hot caller (#645). The row argument stays a
   container template, so this wrapper takes an InlineVec or a std::vector and
   neither pays for the other's buffers. */
template<class Rows>
inline glm::dvec3 actuatorSolve(const Rows &rows, const glm::dvec3 &demand,
                                std::vector<double> &setpoints) {
    actuators::Scratch once;
    const glm::dvec3 delivered = actuatorSolveInto(rows, demand, once);
    setpoints = once.setpoints;
    return delivered;
}
