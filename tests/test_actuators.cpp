//
// Actuator allocation (src/actuators.h, header-only pure math): the capability
// row #576 gives a reaction wheel (a rating AND the line it levers), and the
// solver that turns a demanded attitude torque into per-actuator setpoints
// bounded by their ratings. Pinned without Bullet/GL so the arithmetic stands on
// its own -- and because #426's fin ring is the same row with a different effect
// vector, the non-unit rows here are that solver's small-n bed.
//
//   actuatorTorqueAbout: a drum buys rating * |cos(angle to the asked axis)|, so a
//                wheel 90 deg off an axis buys NOTHING about it; two drums on one
//                line buy the sum; a NaN or zero axis / a non-positive limit buys
//                nothing; the asked axis's length is irrelevant.
//   actuatorSolve: parallel rows (either sense) share one line's capacity, the
//                delivered vector never exceeds what the lines can carry, an
//                uncovered axis buys nothing instead of a pretend torque, the
//                per-row setpoints reconstruct the delivered vector and stay
//                inside their limits, and dead rows are asked to do nothing.
//                OBLIQUE lines (two drums 30 deg apart) are pinned separately:
//                that is the only shape where the coordinate descent does real
//                work, and where a solver that skipped the residual would bill
//                the same demand to both drums.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>

#include "actuators.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                 \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_failures++;                                                     \
            printf("FAIL: %s\n", msg);                                        \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, rel_tol, msg)                            \
    do {                                                                      \
        g_checks++;                                                           \
        const double a_ = (actual), e_ = (expected);                          \
        if(!(std::fabs(a_ - e_) <= rel_tol * std::fabs(e_) + 1e-12)) {        \
            g_failures++;                                                     \
            printf("FAIL: %s -- got %.6g, want %.6g\n", msg, a_, e_);         \
        }                                                                     \
    } while (0)

/* sum(setpoints[k] * rows[k].axis) -- what the actuators were actually asked to
   produce, which must equal what actuatorSolve says it delivered. */
static glm::dvec3 reconstruct(const std::vector<ActuatorRow> &rows,
                              const std::vector<double> &sp) {
    glm::dvec3 t(0.0);
    for(size_t k = 0; k < rows.size(); k++) { t += sp[k] * rows[k].axis; }
    return t;
}

static bool withinLimits(const std::vector<ActuatorRow> &rows,
                         const std::vector<double> &sp) {
    for(size_t k = 0; k < rows.size(); k++) {
        if(!(std::fabs(sp[k]) <= rows[k].limit + 1e-9)) { return false; }
    }
    return true;
}

/* --- the capability row: a wheel has a line, not just a rating ------------- */
static void test_capability_is_about_an_axis() {
    printf("== what a drum can bring to bear ABOUT an axis ==\n");
    const std::vector<ActuatorRow> one = { { glm::dvec3(0, 0, 1), 300.0 } };
    CHECK_NEAR(actuatorTorqueAbout(one, glm::dvec3(0, 0, 1)), 300.0, 1e-12,
               "a drum along the asked axis buys its full rating");
    CHECK_NEAR(actuatorTorqueAbout(one, glm::dvec3(1, 0, 0)), 0.0, 1e-12,
               "the same drum buys NOTHING about an axis it is 90 deg off (#576)");
    CHECK_NEAR(actuatorTorqueAbout(one, glm::dvec3(0, 0, -1)), 300.0, 1e-12,
               "a line, not a sense: asking the other way still buys the rating");
    CHECK_NEAR(actuatorTorqueAbout(one, glm::dvec3(0, 0, 2)), 300.0, 1e-12,
               "the asked vector's length is irrelevant");
    const double c60 = std::cos(std::acos(0.5));   // 60 deg from +Z
    CHECK_NEAR(actuatorTorqueAbout(one, glm::dvec3(std::sin(std::acos(0.5)), 0, c60)),
               300.0 * 0.5, 1e-12, "60 deg off buys half the rating");

    /* A real 3-axis cluster of equal drums: about a body diagonal each drum is at
       the same angle, and the sum is what the window should print. */
    const std::vector<ActuatorRow> cluster = {
        { glm::dvec3(1, 0, 0), 100.0 }, { glm::dvec3(0, 1, 0), 100.0 },
        { glm::dvec3(0, 0, 1), 100.0 },
    };
    const double inv3 = 1.0 / std::sqrt(3.0);
    CHECK_NEAR(actuatorTorqueAbout(cluster, glm::dvec3(inv3, inv3, inv3)),
               300.0 * inv3, 1e-12, "three drums about the body diagonal");
    CHECK_NEAR(actuatorTorqueAbout(cluster, glm::dvec3(1, 1, 0)), 200.0 / std::sqrt(2.0),
               1e-12, "two of the three, about a face diagonal");

    /* Two drums on one line: the line's capacity, once. */
    const std::vector<ActuatorRow> pair = { { glm::dvec3(0, 0, 1), 200.0 },
                                           { glm::dvec3(0, 0, -1), 50.0 } };
    CHECK_NEAR(actuatorTorqueAbout(pair, glm::dvec3(0, 0, 1)), 250.0, 1e-12,
               "two drums on one line buy the sum, head-to-tail included");

    /* Dead rows contribute nothing -- and a NaN axis must not be creditable. */
    const double nan = std::nan("");
    const std::vector<ActuatorRow> junk = { { glm::dvec3(nan, 0, 0), 500.0 },
                                            { glm::dvec3(0, 0, 0), 500.0 },
                                            { glm::dvec3(0, 0, 1), 0.0 },
                                            { glm::dvec3(0, 0, 1), -5.0 } };
    CHECK_TRUE(actuatorTorqueAbout(junk, glm::dvec3(0, 0, 1)) == 0.0,
               "a NaN axis, a zero axis and a non-positive limit all buy nothing");
    CHECK_TRUE(actuatorTorqueAbout({}, glm::dvec3(0, 0, 1)) == 0.0,
               "no actuator, no authority");
    CHECK_TRUE(actuatorTorqueAbout(cluster, glm::dvec3(0, 0, 0)) == 0.0,
               "a zero asked axis asks for nothing");
    CHECK_TRUE(!(actuatorTorqueAbout(cluster, glm::dvec3(nan, 0, 0)) > 0.0),
               "a NaN asked axis asks for nothing");
}

/* --- the solver: one line, and an axis the hull has no drum for ------------ */
static void test_one_line() {
    printf("== one drum: it carries its own line and nothing else ==\n");
    const std::vector<ActuatorRow> one = { { glm::dvec3(0, 0, 1), 300.0 } };
    std::vector<double> sp;
    const double nan = std::nan("");

    glm::dvec3 got = actuatorSolve(one, glm::dvec3(0, 0, 300), sp);
    CHECK_NEAR(got.z, 300.0, 1e-12, "demanded and within rating: delivered");
    CHECK_NEAR(sp[0], 300.0, 1e-12, "the drum is asked for all of it");

    got = actuatorSolve(one, glm::dvec3(0, 0, 900), sp);
    CHECK_NEAR(got.z, 300.0, 1e-12, "past the rating the drum still only has 300");
    CHECK_NEAR(sp[0], 300.0, 1e-12, "and its setpoint sits at the limit");

    got = actuatorSolve(one, glm::dvec3(100, 0, 100), sp);
    CHECK_NEAR(glm::length(got), 100.0, 1e-12, "the demand's other 100 N m is not pretend");
    CHECK_NEAR(got.x, 0.0, 1e-12, "an uncovered axis delivers nothing");
    CHECK_NEAR(got.z, 100.0, 1e-12, "the covered part still delivers");
    CHECK_TRUE(withinLimits(one, sp), "setpoints stay inside the rating");
    CHECK_NEAR(glm::length(reconstruct(one, sp) - got), 0.0, 1e-12,
               "the setpoints reconstruct exactly what the solve claims");

    got = actuatorSolve(one, glm::dvec3(0, 0, -500), sp);
    CHECK_NEAR(got.z, -300.0, 1e-12, "the other sense clamps the same way");
    CHECK_NEAR(sp[0], -300.0, 1e-12, "a drum winds either way");

    got = actuatorSolve(one, glm::dvec3(0, 0, 0), sp);
    CHECK_NEAR(glm::length(got), 0.0, 1e-12, "no demand, no delivery");
    CHECK_NEAR(sp[0], 0.0, 1e-12, "and nothing asked of the drum");

    got = actuatorSolve(one, glm::dvec3(nan, 0, 0), sp);
    CHECK_TRUE(!(glm::length(got) > 0.0), "a NaN demand delivers nothing rather than NaN");
    CHECK_TRUE(!(sp[0] != 0.0), "and asks nothing of the drum");

    std::vector<double> sp2;
    const std::vector<ActuatorRow> dead = { { glm::dvec3(0, 0, 0), 10.0 } };
    got = actuatorSolve(dead, glm::dvec3(0, 0, 5), sp2);
    CHECK_NEAR(glm::length(got), 0.0, 1e-12, "a dead row delivers nothing");
    CHECK_NEAR(sp2[0], 0.0, 1e-12, "and is asked to do nothing");
}

/* --- two drums on one line, and a cluster --------------------------------- */
static void test_lines_and_clusters() {
    printf("== one line shared, and a hull that covers all three ==\n");
    const std::vector<ActuatorRow> pair = { { glm::dvec3(0, 0, 1), 200.0 },
                                           { glm::dvec3(0, 0, -1), 50.0 } };
    std::vector<double> sp;
    glm::dvec3 got = actuatorSolve(pair, glm::dvec3(0, 0, 100), sp);
    CHECK_NEAR(got.z, 100.0, 1e-12, "the pair carries an inside-rating demand");
    CHECK_NEAR(sp[0], 200.0 / 250.0 * 100.0, 1e-12,
               "the bigger drum takes the bigger share of the line");
    CHECK_NEAR(sp[1], -50.0 / 250.0 * 100.0, 1e-12,
               "the head-to-tail drum works against its own +Z to push the same way");

    got = actuatorSolve(pair, glm::dvec3(0, 0, 1000), sp);
    CHECK_NEAR(got.z, 250.0, 1e-12, "the line's capacity is the sum of the ratings");
    CHECK_TRUE(withinLimits(pair, sp), "and neither drum is asked past its rating");
    CHECK_NEAR(glm::length(reconstruct(pair, sp) - got), 0.0, 1e-12,
               "the shares add back up to the delivered torque");

    const std::vector<ActuatorRow> cluster = {
        { glm::dvec3(1, 0, 0), 100.0 }, { glm::dvec3(0, 1, 0), 200.0 },
        { glm::dvec3(0, 0, 1), 300.0 },
    };
    got = actuatorSolve(cluster, glm::dvec3(50, 150, 250), sp);
    CHECK_NEAR(glm::length(got - glm::dvec3(50, 150, 250)), 0.0, 1e-12,
               "a cluster inside every rating delivers the whole demand");
    CHECK_NEAR(sp[1], 150.0, 1e-12, "one setpoint per drum, about its own axis");

    got = actuatorSolve(cluster, glm::dvec3(50, 900, 250), sp);
    CHECK_NEAR(glm::length(got - glm::dvec3(50, 200, 250)), 0.0, 1e-12,
               "one axis past its rating clamps alone; the others still answer");
    CHECK_NEAR(sp[1], 200.0, 1e-12, "that drum sits at its limit");
    CHECK_TRUE(withinLimits(cluster, sp), "no drum is over-billed to cover another");

    /* A cluster asked about an axis no drum points at still gets its projections:
       this is the shape #576 leaves a stack-only hull WITHOUT, so it is the pin
       that says the fix is about geometry, not about a missing constant. */
    const std::vector<ActuatorRow> stack_only = { { glm::dvec3(0, 0, 1), 100.0 } };
    got = actuatorSolve(stack_only, glm::dvec3(1, 1, 1), sp);
    CHECK_NEAR(glm::length(got - glm::dvec3(0, 0, 1)), 0.0, 1e-12,
               "a stack-only hull cannot pitch or yaw at all");
}

/* --- a row whose effect is not a unit vector: #426's fin shape ------------- */
static void test_effectiveness_rows() {
    printf("== a row scaled by its effectiveness (the fin-ring shape) ==\n");
    /* 2 N m per radian of travel, 0.35 rad of travel: the line's capacity is
       0.7 N m and the setpoint is radians, not N m. */
    const std::vector<ActuatorRow> fin = { { glm::dvec3(0, 0, 2), 0.35 } };
    std::vector<double> sp;
    CHECK_NEAR(actuatorTorqueAbout(fin, glm::dvec3(0, 0, 1)), 0.7, 1e-12,
               "capability = effectiveness x travel");
    const glm::dvec3 got = actuatorSolve(fin, glm::dvec3(0, 0, 5), sp);
    CHECK_NEAR(got.z, 0.7, 1e-12, "past full travel the surface delivers no more torque");
    CHECK_NEAR(sp[0], 0.35, 1e-12, "and its setpoint is the travel, in radians");

    /* Two fins on one line share by effectiveness, which is what a ring of
       identical fins must do to add authority without fighting. */
    const std::vector<ActuatorRow> pair = { { glm::dvec3(0, 0, 2), 0.35 },
                                           { glm::dvec3(0, 0, 1), 0.35 } };
    const glm::dvec3 g2 = actuatorSolve(pair, glm::dvec3(0, 0, 1.05), sp);
    CHECK_NEAR(g2.z, 1.05, 1e-12, "two fins on one line add up");
    CHECK_NEAR(sp[0] * 2.0 + sp[1] * 1.0, 1.05, 1e-12, "and add up consistently");
    CHECK_TRUE(withinLimits(pair, sp), "neither past its max_deflection");
}

/* #426's RING: four identical fins cruciform around the tail, each a row of
   q*S*cl * (ri x normal) N m per radian, each bounded by its own travel. The rows
   here are the numbers drag.h::surfaceTorquePerRadian produces for a ring at radius
   1 m, 4 m behind the COM, at q*S*cl = 0.5 -- the geometry is pinned there, so what
   this pins is what the SOLVER does with a ring: the symmetric pair's off-axis halves
   cancel, the ring's authority is different about each axis (a build answer, not a
   knob), and losing one fin is visible instead of silent. */
static void test_a_fin_ring_splits_and_a_lost_fin_shows() {
    printf("== a four-fin ring through the solver (#426) ==\n");
    const std::vector<ActuatorRow> ring = {
        { glm::dvec3( 2.0,  0.0, 0.5), 0.35 },   // +X fin,  normal +Y
        { glm::dvec3(-2.0,  0.0, 0.5), 0.35 },   // -X fin,  normal -Y
        { glm::dvec3( 0.0,  2.0, 0.5), 0.35 },   // +Y fin,  normal -X
        { glm::dvec3( 0.0, -2.0, 0.5), 0.35 },   // -Y fin,  normal +X
    };
    /* What the ring is worth, per axis: every fin levers the +Z line by 0.5 N m per
       radian and has 0.35 rad of travel, and only the two X fins lever X. So the
       capability is asymmetric by construction -- which is the whole point of making
       surfaces rows: the hull's authority is what the build says it is. */
    CHECK_NEAR(actuatorTorqueAbout(ring, glm::dvec3(0, 0, 1)), 4 * 0.35 * 0.5, 1e-12,
               "four fins x travel x their share about the roll line");
    CHECK_NEAR(actuatorTorqueAbout(ring, glm::dvec3(1, 0, 0)), 2 * 0.35 * 2.0, 1e-12,
               "only the two X-levered fins buy pitch");
    std::vector<double> sp;
    /* A demand far past the ring's reach along its covered axis: every fin goes to
       its stop, the delivered vector is the ring's SUM (so the paired off-axis halves
       cancel exactly), and no fin is asked to travel past max_deflection. */
    glm::dvec3 got = actuatorSolve(ring, glm::dvec3(0, 0, 4.0), sp);
    CHECK_NEAR(got.z, 4 * 0.35 * 0.5, 1e-9, "the ring delivers its sum and no more");
    CHECK_NEAR(got.x, 0.0, 1e-9, "ring symmetry cancels the off-axis halves");
    CHECK_NEAR(got.y, 0.0, 1e-9, "on both of them");
    CHECK_TRUE(withinLimits(ring, sp), "every fin inside its own travel stop");
    for(size_t k = 0; k < sp.size(); k++) {
        CHECK_NEAR(sp[k], 0.35, 1e-9, "an out-of-reach demand puts each fin at its stop");
    }
    CHECK_NEAR(glm::length(reconstruct(ring, sp) - got), 0.0, 1e-9,
               "and the per-fin deflections add back up to the delivered torque");

    /* Lose the -X fin: the capability drops by a quarter AND the delivered torque grows
       a sideways component the full ring cancelled. Today that asymmetry is what a
       ring of fins does silently -- the ring's off-axis torques only cancel by
       symmetry, and nothing in the model knows the symmetry is gone. */
    const std::vector<ActuatorRow> broken = { ring[0], ring[2], ring[3] };
    CHECK_NEAR(actuatorTorqueAbout(broken, glm::dvec3(0, 0, 1)), 3 * 0.35 * 0.5, 1e-12,
               "a lost fin is a quarter less authority, not a rumour");
    got = actuatorSolve(broken, glm::dvec3(0, 0, 4.0), sp);
    CHECK_NEAR(got.z, 3 * 0.35 * 0.5, 1e-9, "and the solver stops at the new sum");
    CHECK_NEAR(got.x, 0.35 * 2.0, 1e-9, "with the sideways component the lost pair used to cancel");
    CHECK_NEAR(glm::length(reconstruct(broken, sp) - got), 0.0, 1e-9,
               "still consistent fin by fin");
}

/* --- OBLIQUE LINES: what the descent is for -------------------------------
   Every other row set here is one line or a set of mutually orthogonal lines,
   and on those a single sweep that forgets the other lines' contribution is
   already exact. This is the shape where it is not: two drums 30 deg apart, so
   each line's own projection of the demand OVERLAPS the other's. A solver that
   took dot(demand, lineDir) per line instead of the residual would answer 100 N m
   along X with (175, 43.3) -- 75% over the demand, and a sideways torque nobody
   asked for. #426's fin ring is oblique by construction, so this is the ring's
   arithmetic pinned before the ring exists. */
static void test_oblique_lines_split_once() {
    printf("== two lines 30 deg apart: no double-credit, no sideways leak ==\n");
    const double c30 = std::cos(std::acos(-1.0) / 6.0), s30 = 0.5;
    const std::vector<ActuatorRow> pair = { { glm::dvec3(1, 0, 0), 100.0 },
                                           { glm::dvec3(c30, s30, 0), 100.0 } };
    std::vector<double> sp;
    /* Along one drum's own axis, inside its rating: that drum answers, the other
       is not asked to lean on it. The exact answer, so 0 tolerance. */
    glm::dvec3 got = actuatorSolve(pair, glm::dvec3(100, 0, 0), sp);
    CHECK_NEAR(glm::length(got - glm::dvec3(100, 0, 0)), 0.0, 1e-12,
               "an oblique neighbour adds nothing to a demand one line already covers");
    CHECK_NEAR(sp[1], 0.0, 1e-9, "and the neighbour is asked to do nothing");
    CHECK_NEAR(glm::length(reconstruct(pair, sp) - got), 0.0, 1e-9,
               "the setpoints still add up to the delivered vector");

    /* Past both ratings along X: both lines go to their limits (the exact
       constrained optimum), the delivered vector is shorter than the demand, and
       it never exceeds what the two ratings can carry. */
    got = actuatorSolve(pair, glm::dvec3(250, 0, 0), sp);
    CHECK_NEAR(got.x, 100.0 + 100.0 * c30, 1e-6, "both lines at their limit, along X");
    CHECK_NEAR(got.y, 100.0 * s30, 1e-6, "and the sideways component a leaning drum buys");
    CHECK_NEAR(sp[0], 100.0, 1e-6, "line 1 at its rating");
    CHECK_NEAR(sp[1], 100.0, 1e-6, "line 2 at its rating");
    CHECK_TRUE(withinLimits(pair, sp), "neither over-billed");
    CHECK_TRUE(glm::length(got) <= 250.0 + 1e-9, "delivered never exceeds the demand");
    CHECK_NEAR(actuatorTorqueAbout(pair, glm::dvec3(1, 0, 0)), 100.0 + 100.0 * c30, 1e-6,
               "the budget about X is the same two cosines the solve spent");

    /* The same pair about 15 deg off X: the budget is the sum of cosines, which
     * is the ceiling -- and the solve, asked for more than that, lands on the
     * same pair of limits. */
    const double a15 = std::acos(-1.0) / 12.0;
    const glm::dvec3 dir15(std::cos(a15), std::sin(a15), 0.0);
    CHECK_NEAR(actuatorTorqueAbout(pair, dir15),
               100.0 * std::cos(a15) + 100.0 * std::cos(std::acos(-1.0) / 6.0 - a15),
               1e-6, "about an oblique axis, each drum brings its own cos");
}

int main() {
    test_capability_is_about_an_axis();
    test_one_line();
    test_lines_and_clusters();
    test_oblique_lines_split_once();
    test_effectiveness_rows();
    test_a_fin_ring_splits_and_a_lost_fin_shows();
    printf("test_actuators: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
