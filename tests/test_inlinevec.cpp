//
// InlineVec (src/inlinevec.h): the small-buffer vector #668 landed so the
// attitude authority rows stop round-tripping through the heap. What the type
// has to keep, and what breaks if it does not:
//
//   the common hull never allocates    the first N elements live INSIDE the
//                object -- pinned by data() pointing into the object itself, which is the
//                observable form of "no malloc happened" without an allocator counter.
//                This is the whole point of the type: AttitudeAuthority is returned by
//                value, so a buffer it owns dies with every call.
//   a big hull is still correct        overflow past N moves to the heap and every
//                element survives in order. No cap, no truncation -- actuator rows are
//                physics, and silently dropping one would change how a hull steers.
//   refill is free                     clear() keeps the buffer, so the per-substep
//                rebuild (#649's law) lands in memory that already exists.
//   value semantics hold               copy does not alias the source (inline OR heap),
//                move leaves the source empty, and the pair survives being returned by
//                value out of attitudeAuthorityRows().
//   the STL consumers still work       actuateAttitude concatenates the two row sets
//                into a std::vector through begin()/end(), so the iterator pair has to
//                feed a real vector insert().
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test: rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "actuators.h"    // ActuatorRow: the row this container was built for
#include "inlinevec.h"

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

/* "No allocation" is an internal claim, so pin its observable cause: the storage
   is inside the object. sizeof catches a slot_ that moved out of the object. */
template<size_t N>
static bool inside(const InlineVec<ActuatorRow, N> &v) {
    const char *base = reinterpret_cast<const char *>(&v);
    const char *p = reinterpret_cast<const char *>(v.data());
    return p >= base && p < base + sizeof(v);
}

static void test_the_common_case_stays_inside_the_object() {
    InlineVec<ActuatorRow, 8> v;
    CHECK_TRUE(v.empty(), "a fresh InlineVec holds nothing");
    CHECK_TRUE(v.capacity() == 8, "its capacity starts at the inline room");
    for(int k = 0; k < 8; k++) { v.push_back({ glm::dvec3(0.0, 0.0, 1.0), double(k) }); }
    CHECK_TRUE(v.size() == 8, "push_back reaches the inline limit exactly");
    CHECK_TRUE(inside(v), "and never left the object");
    CHECK_TRUE(v.capacity() == 8, "still no heap capacity");
    CHECK_NEAR(v[3].limit, 3.0, 0.0, "indexing reads the row that was pushed");
}

static void test_overflow_is_an_answer_not_a_cliff() {
    InlineVec<ActuatorRow, 4> v;
    for(int k = 0; k < 11; k++) { v.push_back({ glm::dvec3(1.0, 0.0, 0.0), double(k) }); }
    CHECK_TRUE(v.size() == 11, "the rows past the inline room are kept, not dropped");
    CHECK_TRUE(v.capacity() > 4, "and the vector grew rather than truncating");
    bool ordered = true;
    for(int k = 0; k < 11; k++) {
        if(v[size_t(k)].limit != double(k) || v[size_t(k)].axis.x != 1.0) { ordered = false; }
    }
    CHECK_TRUE(ordered, "every row survived the move to the heap, in order");
    InlineVec<ActuatorRow, 4> big;
    big.reserve(40);
    CHECK_TRUE(big.capacity() == 40, "reserve() above the inline room asks for exactly that");
    CHECK_TRUE(!inside(big), "and that buffer is on the heap");
}

static void test_refilling_a_row_set_costs_nothing() {
    InlineVec<ActuatorRow, 8> v;
    for(int k = 0; k < 5; k++) { v.push_back({ glm::dvec3(0.0, 1.0, 0.0), double(k) }); }
    const ActuatorRow *before = v.data();
    v.clear();
    CHECK_TRUE(v.empty() && v.capacity() == 8, "clear() drops the count, not the buffer");
    CHECK_TRUE(v.data() == before, "clear() did not move the storage");
    for(int k = 0; k < 6; k++) { v.push_back({ glm::dvec3(0.0, 1.0, 0.0), double(k) }); }
    CHECK_TRUE(v.data() == before, "the rebuild lands in the SAME memory (#668's point)");
    CHECK_NEAR(v[5].limit, 5.0, 0.0, "and the new rows are the ones that are there");
}

static void test_it_is_still_a_value() {
    InlineVec<ActuatorRow, 3> a;
    for(int k = 0; k < 2; k++) { a.push_back({ glm::dvec3(0.0, 0.0, 1.0), 10.0 + k }); }
    InlineVec<ActuatorRow, 3> b(a);          // inline copy
    CHECK_TRUE(b.size() == 2 && b.data() != a.data(), "a copy owns its own inline slots");
    b[0].limit = 99.0;
    CHECK_NEAR(a[0].limit, 10.0, 0.0, "mutating the copy cannot reach the original");

    InlineVec<ActuatorRow, 3> c;
    for(int k = 0; k < 9; k++) { c.push_back({ glm::dvec3(1.0, 0.0, 0.0), double(k) }); }
    InlineVec<ActuatorRow, 3> d(c);          // heap copy
    CHECK_TRUE(d.size() == 9 && d.data() != c.data(), "a heap copy is a separate buffer");
    d[8].limit = -1.0;
    CHECK_NEAR(c[8].limit, 8.0, 0.0, "and the original keeps its own answer");
    c = d;                                   // heap assign over a heap buffer
    CHECK_TRUE(c.size() == 9 && c[8].limit == -1.0, "assignment copies the rows across");

    InlineVec<ActuatorRow, 3> moved(std::move(d));
    CHECK_TRUE(moved.size() == 9 && d.empty(), "a move takes the rows and leaves nothing");
    CHECK_NEAR(moved[0].limit, 0.0, 0.0, "intact");
    d.push_back({ glm::dvec3(0.0, 1.0, 0.0), 7.0 });
    CHECK_TRUE(d.size() == 1 && moved.size() == 9, "the moved-from vector is usable again");
}

static void test_the_stl_consumers_still_read_it() {
    InlineVec<ActuatorRow, 4> wheels, surfaces;
    wheels.push_back({ glm::dvec3(0.0, 0.0, 1.0), 200.0 });
    wheels.push_back({ glm::dvec3(1.0, 0.0, 0.0), 200.0 });
    surfaces.push_back({ glm::dvec3(0.0, 1.0, 0.0), 50.0 });
    /* actuateAttitude concatenates the two row sets into a std::vector and hands
       that to actuatorSolve, so the iterator pair has to feed insert(). */
    std::vector<ActuatorRow> rows;
    rows.reserve(wheels.size() + surfaces.size());
    rows.insert(rows.end(), wheels.begin(), wheels.end());
    rows.insert(rows.end(), surfaces.begin(), surfaces.end());
    CHECK_TRUE(rows.size() == 3, "both row sets reached the concatenated vector");
    CHECK_NEAR(actuatorTorqueAbout(rows, glm::dvec3(0.0, 0.0, 1.0)), 200.0, 1e-12,
               "the solver sees the wheel row about its own axis");
    /* And the same helper asked of the InlineVec directly: actuators.h takes any
       row container, which is how the authority rows reach it without a copy. */
    CHECK_NEAR(actuatorTorqueAbout(wheels, glm::dvec3(1.0, 0.0, 0.0)), 200.0, 1e-12,
               "actuatorTorqueAbout reads the inline rows in place");
    double sum = 0.0;
    for(const ActuatorRow &r : surfaces) { sum += r.limit; }
    CHECK_NEAR(sum, 50.0, 0.0, "range-for walks exactly the live rows");
}

int main() {
    test_the_common_case_stays_inside_the_object();
    test_overflow_is_an_answer_not_a_cliff();
    test_refilling_a_row_set_costs_nothing();
    test_it_is_still_a_value();
    test_the_stl_consumers_still_read_it();
    printf("test_inlinevec: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
