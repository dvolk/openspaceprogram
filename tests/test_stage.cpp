// Headless test for the pure stage-split bookkeeping (computeStageSplit in
// src/shipdef.cpp): which parts survive vs. drop, which constraints must be
// removed, and the survivors' links remapped into the compressed index space.
// GL/Bullet-free, so it links only against src/shipdef.cpp.
//
// Runs from the repo root:
//   make test   (or: ./test_stage)

#include "shipdef.h"

#include <cstdio>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static bool expect_throw(const std::function<void()> &fn) {
    try { fn(); }
    catch(const std::runtime_error &) { return true; }
    return false;
}

static bool vec_eq(const std::vector<size_t> &a, const std::vector<size_t> &b) {
    if(a.size() != b.size()) { return false; }
    for(size_t i = 0; i < a.size(); i++) { if(a[i] != b[i]) { return false; } }
    return true;
}
static bool links_eq(const std::vector<std::pair<size_t,size_t>> &a,
                     const std::vector<std::pair<size_t,size_t>> &b) {
    if(a.size() != b.size()) { return false; }
    for(size_t i = 0; i < a.size(); i++) {
        if(a[i].first != b[i].first || a[i].second != b[i].second) { return false; }
    }
    return true;
}

/* The common case: a 2-stage rocket, root at the top (part 0).
   parts: 0=capsule(2) 1=wheel(2) 2=tank(1) 3=engine(1)
   links: (0,1) (1,2) (2,3)
   drop the active (lowest) stage = {2,3}. */
static void test_two_stage_rocket() {
    const size_t N = 4;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)1),
        std::make_pair((size_t)1, (size_t)2),
        std::make_pair((size_t)2, (size_t)3) };
    std::vector<bool> drop = { false, false, true, true };

    StageSplit r = computeStageSplit(N, links, drop);

    CHECK(vec_eq(r.keptParts,     std::vector<size_t>{ 0, 1 }));
    CHECK(vec_eq(r.droppedParts,  std::vector<size_t>{ 2, 3 }));
    CHECK((std::vector<long>(r.newIndexOf) == std::vector<long>{ 0, 1, -1, -1 }));
    /* (1,2) is the stage interface; (2,3) is inside the dropped set -- both
       touch a deleted body, so both must be removed. (0,1) survives. */
    CHECK(vec_eq(r.cutConstraints, std::vector<size_t>{ 1, 2 }));
    CHECK(links_eq(r.keptLinks, std::vector<std::pair<size_t,size_t>>{
        std::make_pair((size_t)0, (size_t)1) }));
}

/* Dropping nothing is a no-op: everything kept, links unchanged. */
static void test_no_drop() {
    const size_t N = 4;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)1),
        std::make_pair((size_t)1, (size_t)2),
        std::make_pair((size_t)2, (size_t)3) };
    std::vector<bool> drop = { false, false, false, false };

    StageSplit r = computeStageSplit(N, links, drop);

    CHECK(vec_eq(r.keptParts,    std::vector<size_t>{ 0, 1, 2, 3 }));
    CHECK(vec_eq(r.droppedParts, std::vector<size_t>{}));
    CHECK((std::vector<long>(r.newIndexOf) == std::vector<long>{ 0, 1, 2, 3 }));
    CHECK(vec_eq(r.cutConstraints, std::vector<size_t>{}));
    CHECK(links_eq(r.keptLinks, links));
}

/* Multiple radial boosters: the drop set is several disconnected subtrees,
   each welded to the core. All four links touch a dropped part -> all cut.
   parts: 0=core(2) 1=bA_tank(1) 2=bA_engine(1) 3=bB_tank(1) 4=bB_engine(1)
   links: (0,1) (1,2) (0,3) (3,4)   drop = {1,2,3,4} */
static void test_radial_boosters() {
    const size_t N = 5;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)1),
        std::make_pair((size_t)1, (size_t)2),
        std::make_pair((size_t)0, (size_t)3),
        std::make_pair((size_t)3, (size_t)4) };
    std::vector<bool> drop = { false, true, true, true, true };

    StageSplit r = computeStageSplit(N, links, drop);

    CHECK(vec_eq(r.keptParts,    std::vector<size_t>{ 0 }));
    CHECK(vec_eq(r.droppedParts, std::vector<size_t>{ 1, 2, 3, 4 }));
    CHECK((std::vector<long>(r.newIndexOf) == std::vector<long>{ 0, -1, -1, -1, -1 }));
    CHECK(vec_eq(r.cutConstraints, std::vector<size_t>{ 0, 1, 2, 3 }));
    CHECK(r.keptLinks.empty());
}

/* Dropping a middle part of a 3-chain: both adjacent links are cut, and the
   two survivors are remapped to adjacent new indices (0 and 1).
   parts: 0=A 1=B 2=C    links: (0,1) (1,2)    drop = {1} */
static void test_middle_drop() {
    const size_t N = 3;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)1),
        std::make_pair((size_t)1, (size_t)2) };
    std::vector<bool> drop = { false, true, false };

    StageSplit r = computeStageSplit(N, links, drop);

    CHECK(vec_eq(r.keptParts,    std::vector<size_t>{ 0, 2 }));
    CHECK(vec_eq(r.droppedParts, std::vector<size_t>{ 1 }));
    CHECK((std::vector<long>(r.newIndexOf) == std::vector<long>{ 0, -1, 1 }));
    CHECK(vec_eq(r.cutConstraints, std::vector<size_t>{ 0, 1 }));
    CHECK(r.keptLinks.empty());
}

/* A kept link whose endpoints are non-adjacent after compression must be
   remapped correctly.
   parts: 0=A 1=B 2=C 3=D    links: (0,3)    drop = {1,2}
   A and D survive -> new indices 0 and 1; the (0,3) link becomes (0,1). */
static void test_link_remap() {
    const size_t N = 4;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)3) };
    std::vector<bool> drop = { false, true, true, false };

    StageSplit r = computeStageSplit(N, links, drop);

    CHECK(vec_eq(r.keptParts,    std::vector<size_t>{ 0, 3 }));
    CHECK(vec_eq(r.droppedParts, std::vector<size_t>{ 1, 2 }));
    CHECK((std::vector<long>(r.newIndexOf) == std::vector<long>{ 0, -1, -1, 1 }));
    CHECK(vec_eq(r.cutConstraints, std::vector<size_t>{}));
    CHECK(links_eq(r.keptLinks, std::vector<std::pair<size_t,size_t>>{
        std::make_pair((size_t)0, (size_t)1) }));
}

static void test_error_paths() {
    const size_t N = 3;
    std::vector<std::pair<size_t,size_t>> links = {
        std::make_pair((size_t)0, (size_t)1) };
    /* drop.size() mismatch -> throw */
    CHECK(expect_throw([&](){
        computeStageSplit(N, links, std::vector<bool>{ false, true });
    }));
    /* link endpoint out of range -> throw */
    std::vector<std::pair<size_t,size_t>> bad = {
        std::make_pair((size_t)0, (size_t)99) };
    CHECK(expect_throw([&](){
        computeStageSplit(N, bad, std::vector<bool>{ false, false, false });
    }));
}

int main() {
    test_two_stage_rocket();
    test_no_drop();
    test_radial_boosters();
    test_middle_drop();
    test_link_remap();
    test_error_paths();

    if(failures == 0) { printf("test_stage: all passed\n"); return 0; }
    printf("test_stage: %d FAILURES\n", failures);
    return 1;
}
