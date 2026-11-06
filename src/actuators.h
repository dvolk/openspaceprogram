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
//   actuatorSolve()        what they actually deliver against a demanded vector,
//                          and what each one's share of it is.
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
// torque over ONE-WAY nozzles (nnls, c in [0,1]). This one is torque-only and
// two-sided -- a drum winds either way -- so its bound is a box, not a sign.
// Worth knowing both exist before a third demand shape shows up and someone
// merges them. It also carries no Scratch like rcs::Scratch yet: today's rows are a
// hull's wheels plus its control surfaces -- still a handful -- so these vectors are
// the whole cost. That cost is twelve allocations per hull per substep for a hull that
// has actuators: four in attitudeAuthority (wheels, wheelParts, surfaces, surfaceParts),
// two in actuateAttitude (the concatenated rows, the setpoints) and six in actuatorSolve
// (lines, lineOf, sense, share, mag, q) -- and the UI calls attitudeAuthority() once
// per FRAME (Game Debug's steering rows ask the live answer, Systems asks the hardware
// one), plus two more per --steering-log line. Closed windows draw no body at all (ui.h),
// so the default flight layout pays none of this. So a
// Scratch is worth doing before the
// row counts grow. #594 landed the other half of that ask: ONE AttitudeAuthority per
// substep, answered once for the stick and every latched law together (#420).

#include <array>
#include <cmath>
#include <vector>

#include <glm/glm.hpp>

/* One actuator. `axis` is the line it levers torque along, SCALED by how much a
   unit of its own demand buys: for a wheel that is a unit vector along the drum's
   spin axis and the demand is N m; for #426's surface it is the mount-derived
   torque direction at N m per radian and the demand is radians of deflection.
   `limit` bounds that demand either way (a wheel winds either sense, so the
   direction is a line, not a sense). A zero-length or NaN axis buys nothing, and the
   negated compare is what makes that true -- it is the guard for a poisoned row, since
   a NaN axis must never be creditable. An infinite one does get through, and nothing
   authored can produce one, so there is no guard pretending to. */
struct ActuatorRow {
    glm::dvec3 axis = glm::dvec3(0.0);
    double limit = 0.0;
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
    and also why the old single isotropic number understated an oblique axis. */
inline double actuatorTorqueAbout(const std::vector<ActuatorRow> &rows,
                                 const glm::dvec3 &dir) {
    const double d2 = glm::dot(dir, dir);
    if(!(d2 > 0.0)) { return 0.0; }
    const glm::dvec3 u = dir / std::sqrt(d2);
    double t = 0.0;
    for(const ActuatorRow &r : rows) {
        if(!(glm::dot(r.axis, r.axis) > 0.0) || !(r.limit > 0.0)) { continue; }
        t += r.limit * std::fabs(glm::dot(r.axis, u));
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

inline std::array<double, 3> actuatorRpY(const std::vector<ActuatorRow> &rows,
                                         const glm::dmat3 &ax) {
    return { actuatorTorqueAbout(rows, ax[2]), actuatorTorqueAbout(rows, ax[0]),
             actuatorTorqueAbout(rows, ax[1]) };
}

/* Fit `demand` (N m) into what the rows can deliver: one scalar per row, inside
   +/- its limit, so that Sum s_k * axis_k lands as close to `demand` as the
   ratings allow. `setpoints` is resized to rows and answers what each actuator is
   asked to do -- a wheel's N m along its own axis (which is the number #579's
   momentum store wants), a fin's radians -- so the caller can bill the actuator
   rather than only the sum. Returns the DELIVERED vector: it is shorter than
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
inline glm::dvec3 actuatorSolve(const std::vector<ActuatorRow> &rows,
                                const glm::dvec3 &demand,
                                std::vector<double> &setpoints) {
    setpoints.assign(rows.size(), 0.0);
    /* A NaN demand asks for nothing, and it has to be answered here: the sweep's
       clamp reads `fabs(want) <= capacity`, which a NaN fails into the NEGATIVE
       limit, i.e. a poisoned demand would command full torque against itself. */
    if(!std::isfinite(demand.x) || !std::isfinite(demand.y)
    || !std::isfinite(demand.z)) { return glm::dvec3(0.0); }
    struct Line {
        glm::dvec3 dir = glm::dvec3(0.0);
        double capacity = 0.0;
    };
    std::vector<Line> lines;
    /* Per row: which line it fell on, whether it points along or against that
       line's chosen sense, and how much of the line's capacity is its share. */
    std::vector<int> lineOf(rows.size(), -1);
    std::vector<double> sense(rows.size(), 0.0), share(rows.size(), 0.0), mag(rows.size(), 0.0);
    for(size_t k = 0; k < rows.size(); k++) {
        const double m2 = glm::dot(rows[k].axis, rows[k].axis);
        if(!(m2 > 0.0) || !(rows[k].limit > 0.0)) { continue; }
        const double m = std::sqrt(m2);
        const glm::dvec3 dir = rows[k].axis / m;
        int hit = -1;
        for(size_t j = 0; j < lines.size(); j++) {
            /* Same LINE either way, hence the fabs: two drums mounted head to tail
               are one actuator line with twice the rating. The tolerance is for
               axes that agree to within rounding, not for "nearly parallel", which
               is a real geometry with two nearly-independent lines. */
            if(std::fabs(glm::dot(dir, lines[j].dir)) > 1.0 - 1e-9) { hit = (int)j; break; }
        }
        if(hit < 0) {
            lines.push_back({ dir, 0.0 });
            hit = (int)lines.size() - 1;
        }
        const double w = rows[k].limit * m;
        lines[hit].capacity += w;
        lineOf[k] = hit;
        sense[k] = (glm::dot(dir, lines[hit].dir) >= 0.0) ? 1.0 : -1.0;
        share[k] = w;
        mag[k] = m;
    }
    if(lines.empty()) { return glm::dvec3(0.0); }

    std::vector<double> q(lines.size(), 0.0);   // N m each line is currently carrying
    for(int sweep = 0; sweep < 32; sweep++) {
        double moved = 0.0;
        for(size_t j = 0; j < lines.size(); j++) {
            /* This line's best contribution given every OTHER line's: the residual
               it is free to answer, projected on its own direction, clamped to its
               capacity. */
            glm::dvec3 rest(0.0);
            for(size_t i = 0; i < lines.size(); i++) {
                if(i != j) { rest += q[i] * lines[i].dir; }
            }
            const double want = glm::dot(demand - rest, lines[j].dir);
            const double take = (std::fabs(want) <= lines[j].capacity)
                              ? want : (want > 0.0 ? lines[j].capacity : -lines[j].capacity);
            moved = std::fmax(moved, std::fabs(take - q[j]));
            q[j] = take;
        }
        if(moved <= 1e-12 * (glm::length(demand) + 1.0)) { break; }
    }
    glm::dvec3 delivered(0.0);
    for(size_t j = 0; j < lines.size(); j++) { delivered += q[j] * lines[j].dir; }
    for(size_t k = 0; k < rows.size(); k++) {
        const int j = lineOf[k];
        if(j < 0) { continue; }      // a dead row is asked to do nothing
        const double along = q[j] * (share[k] / lines[j].capacity);
        setpoints[k] = sense[k] * along / mag[k];
    }
    return delivered;
}
