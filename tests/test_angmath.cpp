//
// src/angmath.h (header-only pure math): the one acos(clamp) spelling, pinned so
// its job stays narrow and the NaN law stays written down (#622).
//
//   acosUnit clamps for FP DRIFT and nothing else: acosUnit of a cosine that landed
//   one ulp past 1 is 0 deg, not NaN, and one past -1 is 180 deg, not NaN. An
//   unclamped acos at those inputs poisons a window ("-nan deg") or a rotation.
//
//   It is NOT a guard, and this pins that too: a NaN cosine comes back NaN -- never
//   0, never pi, never the confident end of acos' range. glm::clamp is
//   min(max(x,lo),hi) and std::clamp is the same comparison shape; every relational
//   test against NaN is false, so it walks through. (An infinity IS caught, because
//   those comparisons really do hold -- the contrast is the point.)
//
//   And the trap the door guards exist for: one comparison after acosUnit turns a
//   NaN into the most confident number in the range. std::max(0.0, NaN) is 0.0, so
//   cos(max(0.0, NaN)) is 1.0 -- "all of your thrust, along nothing"
//   (availableAccelAlong answers that at its own door, pinned in test_thrust.cpp).
//   Pinned here so nobody deletes the door guard as redundant.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

#include "angmath.h"

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

/* --- what the clamp is FOR: fp drift, one ulp at a time --------------------- */
static void test_drift_and_range() {
    printf("== a cosine just past the valid range ==\n");
    const double one_up = std::nextafter(1.0, 2.0);    // 1 + 1 ulp: a real dot result
    const double one_dn = std::nextafter(-1.0, -2.0);
    CHECK_TRUE(one_up > 1.0, "the bed input really is out of acos' domain");
    CHECK_TRUE(one_dn < -1.0, "and its mirror is too");
    CHECK_TRUE(std::isnan(std::acos(one_up)),
               "unclamped, that input is a NaN -- which is the bug being clamped");
    CHECK_NEAR(acosUnit(one_up), 0.0, 0.0, "one ulp past 1 reads 0 deg");
    CHECK_NEAR(acosUnit(one_dn), std::numbers::pi, 0.0, "one ulp past -1 reads 180 deg");
    CHECK_NEAR(acosUnit(2.0), 0.0, 0.0, "so does a cosine that is simply out of range");
    CHECK_NEAR(acosUnit(-2.0), std::numbers::pi, 0.0, "and its mirror");
    /* An infinity is out of range and comparisons DO hold on it, so it clamps. The
       contrast with NaN below is the whole of #622. */
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_NEAR(acosUnit(inf), 0.0, 0.0, "an infinite cosine clamps to 0 deg");
    CHECK_NEAR(acosUnit(-inf), std::numbers::pi, 0.0, "and to 180 on the other side");
}

/* --- the function is the one it claims to be --------------------------------- */
static void test_the_angle_itself() {
    printf("== the angle, not its complement or its sine ==\n");
    const double pi = std::numbers::pi;
    CHECK_NEAR(acosUnit(1.0), 0.0, 0.0, "cos 1 -> 0");
    CHECK_NEAR(acosUnit(0.0), pi / 2.0, 1e-12, "cos 0 -> 90 deg");
    CHECK_NEAR(acosUnit(0.5), pi / 3.0, 1e-12, "cos 0.5 -> 60 deg");
    CHECK_NEAR(acosUnit(-0.5), 2.0 * pi / 3.0, 1e-12, "cos -0.5 -> 120 deg");
    CHECK_NEAR(acosUnit(-1.0), pi, 0.0, "cos -1 -> 180 deg");
    for(const double x : {0.3, -0.7, 0.999999}) {
        CHECK_NEAR(std::cos(acosUnit(x)), x, 1e-12, "cos(acosUnit(x)) round-trips in range");
    }
    /* Monotone: a smaller cosine is a bigger angle, so a sign slip is visible. */
    CHECK_TRUE(acosUnit(0.9) < acosUnit(0.1), "the angle rises as the cosine falls");
    CHECK_TRUE(acosUnit(0.1) < acosUnit(-0.1), "and keeps rising across zero");
    CHECK_TRUE(acosUnit(-0.1) < acosUnit(-0.9), "and keeps rising into the negatives");
}

/* --- what the clamp is NOT: a guard ----------------------------------------- */
static void test_a_nan_walks_through() {
    printf("== the clamp cannot see a NaN ==\n");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_TRUE(std::isnan(acosUnit(nan)),
               "a NaN cosine stays a NaN: the clamp is not a guard");
    CHECK_TRUE(std::isnan(acosUnit(nan + 1.0)), "arithmetic does not rescue it either");
    /* The snapping shape angmath.h warns about, pinned so the warning names
       something real: IEEE fmax/fmin return the NON-NaN operand, so THOSE would
       put a NaN at an end of acos' range. A hand-written (c < -1 ? -1 : ...)
       clamp does not -- NaN fails both comparisons -- and the mutation battery
       shows the pin set cannot tell that spelling from std::clamp, correctly. */
    CHECK_NEAR(std::fmax(nan, -1.0), -1.0, 0.0, "std::fmax would rescue a NaN, which is why it stays out of acosUnit");
    CHECK_TRUE(!std::isnan(acosUnit(0.0)), "the finite neighbour of that NaN is fine");
}

/* --- and one step later, the NaN becomes confidence -------------------------- */
static void test_the_laundering_step() {
    printf("== why the door guard is not redundant ==\n");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    /* availableAccelAlong()'s shape: cos(max(0, off - cone)). std::max keeps its
       first argument unless the second is strictly greater, and nothing is greater
       than NaN, so the NaN angle becomes 0 rad, i.e. "the ask is dead ahead". */
    CHECK_NEAR(std::cos(std::max(0.0, acosUnit(nan) - 0.0)), 1.0, 0.0,
               "std::max(0.0, NaN) is 0.0, so the share reads 1.0: full thrust along nothing");
    CHECK_TRUE(!(std::fabs(nan) > 1.0),
               "and a range gate waves a NaN through too: fabs(NaN) > 1 is false");
    CHECK_TRUE(!(acosUnit(nan) < 1e-9),
               "so the fail-closed spellings are !(a < b) / isfinite, at the door");
}

int main() {
    test_drift_and_range();
    test_the_angle_itself();
    test_a_nan_walks_through();
    test_the_laundering_step();
    printf("test_angmath: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
