//
// RCS nozzle allocator (src/rcs.h, header-only pure math, #428): one-way
// nozzles with rated thrust, allocated so the demanded force happens and
// unwanted spin is cancelled when free. Pinned without Bullet/GL (like
// test_gimbal, test_drag).
//
//   one-way:      a nozzle pushes along its dir; it can never pull.
//   position:     force lands at the pod, so an off-center pod also spins
//                 the ship -- the allocator keeps the commanded FORCE
//                 direction (torqueWeight small) rather than buying
//                 spin-reduction with force.
//   cancellation: a symmetric pair cancels torque exactly (zero-cost).
//   cone/rating:  unreachable demand (missing face) or over-rating comes
//                 back as an honest shortfall, never free thrust.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>

#include "rcs.h"

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

static bool near(double a, double b, double tol) { return std::fabs(a - b) < tol; }

/* Reconstruct (force, torque) from a solution, the way the caller applies it. */
static void achieved(const std::vector<RcsNozzle> &noz, const std::vector<double> &c,
                     glm::dvec3 &F, glm::dvec3 &T) {
    F = glm::dvec3(0.0); T = glm::dvec3(0.0);
    for(size_t i = 0; i < noz.size(); i++) {
        const double N = c[i] * noz[i].thrust;
        F += noz[i].dir * N;
        T += glm::cross(noz[i].arm, noz[i].dir) * N;
    }
}

static RcsNozzle nz(glm::dvec3 dir, glm::dvec3 arm, double thrust) {
    RcsNozzle n; n.dir = glm::normalize(dir); n.arm = arm; n.thrust = thrust;
    return n;
}

static void test_one_way() {
    printf("== rcsAllocate: nozzles push, never pull ==\n");
    const std::vector<RcsNozzle> noz = { nz(glm::dvec3(1,0,0), glm::dvec3(0,0,0), 100.0) };
    const std::vector<double> fwd = rcsAllocate(noz, glm::dvec3(80,0,0));
    CHECK_TRUE(fwd[0] > 0.7 && fwd[0] <= 1.0, "demand along the nozzle fires it");
    const std::vector<double> back = rcsAllocate(noz, glm::dvec3(-80,0,0));
    CHECK_TRUE(back[0] < 1e-6, "demand against a one-way nozzle stays shut");
    glm::dvec3 F, T;
    achieved(noz, back, F, T);
    CHECK_TRUE(glm::length(F) < 1e-6, "shut nozzle delivers no force");
}

static void test_offcenter_spins_but_moves() {
    printf("== rcsAllocate: off-center pod keeps the commanded force ==\n");
    /* A pod off to the right (+X) with one +Z nozzle: pushing "forward"
       translates AND spins (+X x +Z = ... torque about -Y... whatever it is,
       it is nonzero and the FORCE still lands on +Z). */
    const std::vector<RcsNozzle> noz = { nz(glm::dvec3(0,0,1), glm::dvec3(1,0,0), 200.0) };
    const std::vector<double> c = rcsAllocate(noz, glm::dvec3(0,0,200));
    glm::dvec3 F, T;
    achieved(noz, c, F, T);
    CHECK_TRUE(near(F.z, 200.0, 1e-6), "force demand met despite the spin");
    CHECK_TRUE(glm::length(T) > 1.0, "off-center pod torques the ship (the #428 point)");
    CHECK_TRUE(near(F.x, 0.0, 1e-6) && near(F.y, 0.0, 1e-6), "no invented lateral force");
}

static void test_pair_cancels() {
    printf("== rcsAllocate: a symmetric pair cancels spin for free ==\n");
    const std::vector<RcsNozzle> noz = {
        nz(glm::dvec3(0,0,1), glm::dvec3( 1,0,0), 200.0),
        nz(glm::dvec3(0,0,1), glm::dvec3(-1,0,0), 200.0),
    };
    const std::vector<double> c = rcsAllocate(noz, glm::dvec3(0,0,200));
    CHECK_TRUE(c[0] > 0.1 && c[1] > 0.1, "both boxes fire (share the work)");
    glm::dvec3 F, T;
    achieved(noz, c, F, T);
    CHECK_TRUE(near(F.z, 200.0, 1e-6), "pair delivers the full force");
    CHECK_TRUE(glm::length(T) < 1e-6, "pair cancels the torque exactly");
}

static void test_missing_face_is_honest() {
    printf("== rcsAllocate: no nozzle means no thrust (the honest cone) ==\n");
    /* Allocator view: dir = the FORCE a nozzle delivers. A real 5-bell box
       omits its mount BELL, so the force it lacks points AWAY from the
       mount; this stand-in box just omits a -Z nozzle -- what's pinned is
       "no thrust outside the installed nozzles' cone", by whatever
       geometry the part happens to have. */
    std::vector<RcsNozzle> box;
    const glm::dvec3 dirs[5] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1}};
    for(const glm::dvec3 &d : dirs) { box.push_back(nz(d, glm::dvec3(0,0,0), 60.0)); }
    const std::vector<double> c = rcsAllocate(box, glm::dvec3(0,0,-100));
    glm::dvec3 F, T;
    achieved(box, c, F, T);
    CHECK_TRUE(glm::length(F) < 1e-6, "no thrust into the mount face");
    /* A diagonal inside the cone IS reachable. */
    const std::vector<double> cd = rcsAllocate(box, glm::dvec3(60,60,0));
    achieved(box, cd, F, T);
    CHECK_TRUE(near(F.x, 60.0, 1e-6) && near(F.y, 60.0, 1e-6), "in-cone diagonal delivered");
    CHECK_TRUE(cd[1] < 1e-6 && cd[3] < 1e-6, "unused nozzles stay shut");
}

static void test_rating_clamp() {
    printf("== rcsAllocate: clipped c is a shortfall, not free thrust ==\n");
    const std::vector<RcsNozzle> noz = {
        nz(glm::dvec3(0,0,1), glm::dvec3( 1,0,0), 60.0),
        nz(glm::dvec3(0,0,1), glm::dvec3(-1,0,0), 60.0),
    };
    const std::vector<double> c = rcsAllocate(noz, glm::dvec3(0,0,300));
    for(size_t i = 0; i < c.size(); i++) { CHECK_TRUE(c[i] <= 1.0 + 1e-9, "never over rating"); }
    glm::dvec3 F, T;
    achieved(noz, c, F, T);
    CHECK_TRUE(near(F.z, 120.0, 1e-6), "achieved caps at the sum of ratings");
}

/* The B1 pin (adversarial review): an unbounded solve that concentrates on
   ONE nozzle and post-hoc clips loses the excess thrust instead of handing
   it to idle nozzles. Two pods, DIFFERENT arms (symmetric splits pass by
   luck; this is the common asymmetric case) plus an opposed nozzle that
   must not be dragged in as a torque trim against the demand. */
static void test_bound_redistributes() {
    printf("== rcsAllocate: over-rating spills to the idle nozzles ==\n");
    const std::vector<RcsNozzle> noz = {
        nz(glm::dvec3(0,0,1),  glm::dvec3(1,0,0), 200.0),
        nz(glm::dvec3(0,0,1),  glm::dvec3(2,0,0), 200.0),
        nz(glm::dvec3(0,0,-1), glm::dvec3(1,0,0), 200.0),
    };
    const std::vector<double> c = rcsAllocate(noz, glm::dvec3(0,0,400));
    CHECK_TRUE(c[0] > 0.99, "first +Z pod at rating");
    CHECK_TRUE(c[1] > 0.99, "the over-rated demand spills to the second +Z pod");
    CHECK_TRUE(c[2] < 1e-6, "the opposed nozzle is not a torque trim against demand");
    glm::dvec3 F, T;
    achieved(noz, c, F, T);
    CHECK_TRUE(near(F.z, 400.0, 1e-6), "the ship's full rated force is delivered");
    /* And one nozzle's worth of demand never needs two firing. */
    const std::vector<double> c1 = rcsAllocate(noz, glm::dvec3(0,0,200));
    int firing = 0;
    for(size_t i = 0; i < c1.size(); i++) { if(c1[i] > 1e-6) { firing++; } }
    CHECK_TRUE(firing >= 1 && firing <= 2, "sub-rating demand stays efficient");
    achieved(noz, c1, F, T);
    CHECK_TRUE(near(F.z, 200.0, 1e-6), "sub-rating demand fully delivered");
}

static void test_legacy_pod_is_omnidirectional() {
    printf("== rcsAllocate: legacy 6-dir pod hits any demand within rating ==\n");
    /* Six dirs along the part axes at one arm, like the synthesized legacy
       pod. Any direction must land exactly (opposite pairs: one fires). */
    std::vector<RcsNozzle> pod;
    const glm::dvec3 arms = glm::dvec3(0.5, 0.2, -0.3);
    const glm::dvec3 dirs[6] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for(const glm::dvec3 &d : dirs) { pod.push_back(nz(d, arms, 200.0)); }
    const glm::dvec3 Fdem(30.0, -40.0, 120.0);
    const std::vector<double> c = rcsAllocate(pod, Fdem);
    glm::dvec3 F, T;
    achieved(pod, c, F, T);
    CHECK_TRUE(glm::length(F - Fdem) < 1e-6, "6-dir pod meets an arbitrary demand");
    int firing = 0;
    for(size_t i = 0; i < c.size(); i++) { if(c[i] > 1e-9) { firing++; } }
    CHECK_TRUE(firing == 3, "one nozzle per used axis (no opposed waste-firing)");
    /* The pod is off-center (arms != 0), so spin comes along; that is the
       honest physics, and the demanded force still lands. */
    CHECK_TRUE(glm::length(T) > 1.0, "off-center legacy pod also spins the ship");
}

static void test_zero_demand_and_empty() {
    printf("== rcsAllocate: empty inputs are quiet ==\n");
    const std::vector<RcsNozzle> noz = { nz(glm::dvec3(0,0,1), glm::dvec3(0,0,0), 60.0) };
    const std::vector<double> c = rcsAllocate(noz, glm::dvec3(0,0,0));
    CHECK_TRUE(c.size() == 1 && c[0] < 1e-9, "zero demand -> nothing fires");
    const std::vector<RcsNozzle> none;
    CHECK_TRUE(rcsAllocate(none, glm::dvec3(0,0,100)).empty(), "no nozzles -> empty solution");
}

/* The game's hot path (Vehicle::applyRcsForce) keeps ONE Scratch across
   substeps and nozzle sets. A reused buffer must never carry state between
   solves: same inputs -> bit-identical answers, and shrinking to a smaller
   nozzle set (stale capacity) must not leak old columns. */
static bool same(const std::vector<double> &a, const std::vector<double> &b) {
    if(a.size() != b.size()) { return false; }
    for(size_t i = 0; i < a.size(); i++) { if(!(a[i] == b[i])) { return false; } }
    return true;
}

static void test_scratch_reuse() {
    printf("== rcsAllocateInto: one Scratch, reused, stays honest ==\n");
    const std::vector<RcsNozzle> big = {
        nz(glm::dvec3(0,0,1),  glm::dvec3(1,0,0.5),  200.0),
        nz(glm::dvec3(0,0,-1), glm::dvec3(-1,0,0.5), 200.0),
        nz(glm::dvec3(1,0,0),  glm::dvec3(0,0.5,0),  150.0),
        nz(glm::dvec3(-1,0,0), glm::dvec3(0,-0.5,0), 150.0),
        nz(glm::dvec3(0,1,0),  glm::dvec3(0.5,0,0),  100.0),
    };
    rcs::Scratch s;
    const glm::dvec3 F(30, -20, 100);
    rcsAllocateInto(big, F, s);
    CHECK_TRUE(same(s.c, rcsAllocate(big, F)), "reused Scratch == fresh solve");
    const double first = s.c[0];
    rcsAllocateInto(big, F, s);  // repeat: no state drifted into the second solve
    CHECK_TRUE(same(s.c, rcsAllocate(big, F)) && s.c[0] == first,
               "same Scratch twice -> identical answers");
    /* Over-rating: the pin-and-resolve passes also run through the Scratch. */
    rcsAllocateInto(big, glm::dvec3(0,0,900), s);
    CHECK_TRUE(same(s.c, rcsAllocate(big, glm::dvec3(0,0,900))),
               "pinned passes reuse the Scratch honestly");
    /* Shrink: 5 nozzles' capacity then 1 nozzle -- old columns must not leak. */
    const std::vector<RcsNozzle> small = { nz(glm::dvec3(0,0,1), glm::dvec3(0,0,0), 60.0) };
    rcsAllocateInto(small, glm::dvec3(0,0,40), s);
    CHECK_TRUE(same(s.c, rcsAllocate(small, glm::dvec3(0,0,40))),
               "small solve after big keeps only its own columns");
    /* And back to empty: the solution empties, no residue of the last. */
    const std::vector<RcsNozzle> none;
    rcsAllocateInto(none, glm::dvec3(0,0,100), s);
    CHECK_TRUE(s.c.empty(), "no nozzles -> empty solution in a warm Scratch");
}

/* Goldens from the PRE-scratch allocator (nested-vector rcs.h, printed at
   17 significant digits from the pre-rewrite source). They pin the flat
   rewrite as a bit-exact translation -- including the LS ridge residuals
   (0.9999999975 != 1.0) an "improved" solver would silently tidy away. */
static void test_golden_answers() {
    printf("== rcsAllocate: pre-rewrite goldens ==\n");
    struct GCase {
        std::vector<RcsNozzle> noz;
        glm::dvec3 F;
        std::vector<double> want;
    };
    const std::vector<RcsNozzle> five = {
        nz(glm::dvec3(0,0,1),  glm::dvec3(1,0,0.5),  200.0),
        nz(glm::dvec3(0,0,-1), glm::dvec3(-1,0,0.5), 200.0),
        nz(glm::dvec3(1,0,0),  glm::dvec3(0,0.5,0),  150.0),
        nz(glm::dvec3(-1,0,0), glm::dvec3(0,-0.5,0), 150.0),
        nz(glm::dvec3(0,1,0),  glm::dvec3(0.5,0,0),  100.0),
    };
    const GCase cases[] = {
        { { nz(glm::dvec3(1,0,0), glm::dvec3(0,0,0), 100.0) }, glm::dvec3(80,0,0),
          { 0.80000000000000004 } },
        { { nz(glm::dvec3(0,0,1), glm::dvec3(1,0,0), 200.0),
            nz(glm::dvec3(0,0,-1), glm::dvec3(-1,0,0), 200.0) }, glm::dvec3(0,0,200),
          { 0.99999999750000002, 0 } },
        { five, glm::dvec3(30,-20,100),
          { 0.49999999900000003, 0, 0.1999999999, 0, 0 } },
        { five, glm::dvec3(0,0,900), { 1, 0, 0, 0, 0 } },
    };
    for(const GCase &gc : cases) {
        CHECK_TRUE(same(rcsAllocate(gc.noz, gc.F), gc.want),
                   "answer matches the pre-rewrite golden bit-for-bit");
    }
}

int main() {
    printf("=== rcs.h: nozzle allocator ===\n\n");
    test_one_way();
    printf("\n");
    test_offcenter_spins_but_moves();
    printf("\n");
    test_pair_cancels();
    printf("\n");
    test_missing_face_is_honest();
    printf("\n");
    test_rating_clamp();
    printf("\n");
    test_bound_redistributes();
    printf("\n");
    test_legacy_pod_is_omnidirectional();
    printf("\n");
    test_zero_demand_and_empty();
    printf("\n");
    test_scratch_reuse();
    printf("\n");
    test_golden_answers();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
