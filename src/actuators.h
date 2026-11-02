#pragma once

// actuators.h -- turning a DEMANDED attitude torque into per-actuator setpoints
// the actuators can actually deliver (glm + vector only, so tests/ can pin it
// without the render/physics chain, like gimbal.h and drag.h).
//
// #576 is why this exists. A reaction wheel used to be a rating with no
// direction: the capability model credited the summed wheel rating in full about
// whichever axis it was asked for, so a hull whose only drum sits along the stack
// -- which is every shipped hull, capsules and reaction wheels are both mounted
// stacked -- bought full pitch and yaw from a wheel that can only roll it. The
// honest answer asks two questions over ONE set of rows, so the budget a law
// plans against and the torque the actuators answer with cannot drift apart:
//
//   actuatorTorqueAbout()  what these actuators can bring to bear ABOUT an axis
//                          (the capability the slew law's alpha budgets with);
//   actuatorSolve()        what they actually deliver against a demanded vector,
//                          and what each one's share of it is.
//
// #426's fin ring is the same row with a different effect vector -- a surface's
// torque-per-unit-deflection and its max_deflection as the limit -- so the wheel
// case is that solver's small-n test bed before the ring exists.
//
// Not the same problem as rcs.h's nozzle allocator, which solves force AND
// torque over ONE-WAY nozzles (nnls, c in [0,1]). This one is torque-only and
// two-sided -- a drum winds either way -- so its bound is a box, not a sign.
// Worth knowing both exist before a third demand shape shows up and someone
// merges them. It also carries no Scratch like rcs::Scratch yet: today's rows are
// the handful of wheels on a hull, not a nozzle ring, so these vectors are the
// whole cost, and #593's ask (one AttitudeAuthority per substep instead of one
// per law) is what keeps that cost off the hot path.

#include <cmath>
#include <vector>

#include <glm/glm.hpp>

/* One actuator. `axis` is the line it levers torque along, SCALED by how much a
   unit of its own demand buys: for a wheel that is a unit vector along the drum's
   spin axis and the demand is N m; for #426's surface it is the mount-derived
   torque direction at N m per radian and the demand is radians of deflection.
   `limit` bounds that demand either way (a wheel winds either sense, so the
   direction is a line, not a sense). A non-finite or zero-length axis or a
   non-positive limit contributes nothing -- which is also the NaN guard, since a
   NaN axis must never be creditable. */
struct ActuatorRow {
    glm::dvec3 axis = glm::dvec3(0.0);
    double limit = 0.0;
};

/* The N m these actuators can bring to bear ABOUT `dir` (any length; the
   direction is what counts, and a zero or NaN direction buys nothing). A drum
    leaning 90 deg off the asked axis contributes nothing about it, which is the
    whole content of #576, and two drums on one line buy that line's capacity
    once -- the same arithmetic actuatorSolve distributes over. */
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
   hang and costs a few hundred flops for a handful of rows -- it runs per substep
   per law, not per part. */
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
