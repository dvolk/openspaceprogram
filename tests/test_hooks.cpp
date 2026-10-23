// test_hooks: the pure rules the headless action hooks used to smuggle.
// Runs from the repo root (needs no res/):
//   make test   (or: ./test_hooks)
//
// Three invariants that e2e could only reach through a hook that built a state
// the UI refuses to:
//
//   seatplan.h pruneSeatPlan   -- a seat plan entry whose part left the build
//        is dropped (issue #288). vabLaunch calls this before it reads the
//        plan, so a detach and a launch in the same frame cannot see a stale
//        entry (was case 244 / --vab-launch + --vab-detach).
//
//   seatplan.h seatPlanProblem -- one person cannot sit in two seats, and a
//        named person must be standing on the pad. The window greys the double
//        seat, so this is the backstop for a plan that goes stale (was case
//        237 / --vab-assign).
//
//   transferplanner.h eraseShipTargetRows / findTargetRow -- a removed vessel
//        leaves the target list at once, and the selection is an IDENTITY that
//        is resolved against that list rather than a slot in it: a body and a
//        vessel may share a name, and retiring one must not retarget the other
//        (was case 243 / --select-target; issue #392).
//
//   question.h dropPendingQuestion / answerPendingQuestion -- a teardown drops
//        the card (was case 258 / --switch-system), and the card closes before
//        onAnswer runs so the callback may open the next one (was case 265 /
//        --recover, whose callback re-found its subject by name).

#include "seatplan.h"
#include "question.h"
#include "transferplanner.h"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

// Distinct addresses, never dereferenced: the row helpers only compare them.
static Vehicle *fakeV(int n) { return reinterpret_cast<Vehicle *>(0x1000 + n); }
static TerrainBody *fakeB(int n) { return reinterpret_cast<TerrainBody *>(0x2000 + n); }
static const char *names[] = { "a", "b", "c" };

static void test_prune_drops_stale() {
    std::vector<SeatAssign> plan = {
        { "capsule_1", 0, 1 },   // stays
        { "gone", 0, 2 },        // dropped
        { "capsule_1", 1, 0 },   // stays (empty seat is a choice)
    };
    // capsule_1 has two seats; "gone" is not in the build at all.
    auto hasSeat = [](const std::string &id, int slot) {
        return id == "capsule_1" && slot >= 0 && slot < 2;
    };
    auto nameOf = [](uint64_t id) { return id == 2 ? std::string("Ace") : std::string(); };
    const int dropped = pruneSeatPlan(plan, hasSeat, nameOf);
    CHECK(dropped == 1);
    CHECK(plan.size() == 2);
    CHECK(plan[0].partId == "capsule_1" && plan[0].slot == 0 && plan[0].profileId == 1);
    CHECK(plan[1].partId == "capsule_1" && plan[1].slot == 1 && plan[1].profileId == 0);
}

static void test_prune_keeps_valid() {
    std::vector<SeatAssign> plan = { { "cap", 0, 7 } };
    auto hasSeat = [](const std::string &, int) { return true; };
    auto nameOf = [](uint64_t) { return std::string("Bram"); };
    CHECK(pruneSeatPlan(plan, hasSeat, nameOf) == 0);
    CHECK(plan.size() == 1);
}

static void test_problem_double_seat() {
    // One person, two seats: the dropdown greys this, so the launch refuses.
    std::vector<SeatAssign> plan = {
        { "cap", 0, 1 },
        { "cap", 1, 1 },
    };
    auto standing = [](uint64_t) { return true; };
    auto nameOf = [](uint64_t) { return std::string("Ace"); };
    CHECK(seatPlanProblem(plan, standing, nameOf) == "'Ace' cannot sit in two seats");
}

static void test_problem_not_at_pad() {
    std::vector<SeatAssign> plan = { { "cap", 0, 9 } };
    auto standing = [](uint64_t) { return false; };
    auto nameOf = [](uint64_t) { return std::string("Cyd"); };
    CHECK(seatPlanProblem(plan, standing, nameOf) == "'Cyd' is not at the launch pad");
}

static void test_problem_empty_seat_and_ok() {
    std::vector<SeatAssign> plan = {
        { "cap", 0, 0 },   // chosen empty: names nobody, always fine
        { "cap", 1, 3 },
    };
    auto standing = [](uint64_t id) { return id == 3; };
    auto nameOf = [](uint64_t) { return std::string(); };
    CHECK(seatPlanProblem(plan, standing, nameOf).empty());
}

static void test_erase_rows() {
    std::vector<TransferPlanner::XferTarget> rows = {
        { names[0], nullptr, fakeV(1) },
        { names[1], nullptr, fakeV(2) },
        { names[2], nullptr, fakeV(3) },
    };
    eraseShipTargetRows(rows, fakeV(2));
    CHECK(rows.size() == 2);
    CHECK(rows[0].ship == fakeV(1));
    CHECK(rows[1].ship == fakeV(3));
    /* A vessel that is not in the list leaves it alone. Erasing is all the
       caller has to do: the selection is an IDENTITY resolved against this
       list, so it does not slide onto whichever row moved up (#392). */
    eraseShipTargetRows(rows, fakeV(9));
    CHECK(rows.size() == 2);
    // Clearing the last one empties the list.
    eraseShipTargetRows(rows, fakeV(1));
    eraseShipTargetRows(rows, fakeV(3));
    CHECK(rows.empty());
}

static void test_target_identity() {
    /* A body and a vessel can share a name and the combo lists a row for each,
       so the KIND is part of the identity (#392). The name pointer is the
       object's own string, so these two rows are two different targets. */
    std::vector<TransferPlanner::XferTarget> rows = {
        { names[0], fakeB(1), nullptr },   // the body "a"
        { names[0], nullptr, fakeV(1) },   // a vessel also called "a"
    };
    const TargetRef bodyRef{names[0], false};
    const TargetRef shipRef{names[0], true};
    CHECK(findTargetRow(rows, bodyRef) == &rows[0]);
    CHECK(findTargetRow(rows, shipRef) == &rows[1]);
    CHECK(!targetMatches(rows[0], shipRef));
    CHECK(!targetMatches(rows[1], bodyRef));
    // No target and target-gone are the same answer, so callers have one check.
    CHECK(findTargetRow(rows, TargetRef{}) == nullptr);
    CHECK(findTargetRow(rows, TargetRef{"zz", false}) == nullptr);
    /* Retiring the vessel: its row goes, the body's stays. This is the pair
       Game::dropVehicleRefs relies on -- the ship selection stops resolving (so
       dropStaleTarget leaves NO selection rather than the row that slid up,
       which here would be the moon), while a body selection is untouched. */
    eraseShipTargetRows(rows, fakeV(1));
    CHECK(rows.size() == 1);
    CHECK(findTargetRow(rows, shipRef) == nullptr);
    CHECK(findTargetRow(rows, bodyRef) == &rows[0]);
}

static void test_drop_question() {
    auto q = std::make_unique<Question>();
    q->text = "Clear the pad?\nSecond line";
    q->labels = { "Clear the pad", "Cancel" };
    bool called = false;
    q->onAnswer = [&](int) { called = true; };
    dropPendingQuestion(q);
    CHECK(q == nullptr);
    CHECK(!called);   // dropped, not answered
}

static void test_answer_closes_first() {
    // The callback must see the card already gone, so it may ask() again.
    std::unique_ptr<Question> q = std::make_unique<Question>();
    q->text = "Clear the pad?";
    q->labels = { "Clear the pad", "Cancel" };
    std::string found;
    q->onAnswer = [&](int choice) {
        CHECK(q == nullptr);         // closed BEFORE onAnswer
        CHECK(choice == 0);
        found = "ran";
        // The real invariant of closing first: a callback may open the NEXT
        // card (the pad-clear flow does), and answerPendingQuestion must not
        // wipe it on the way out.
        q = std::make_unique<Question>();
        q->text = "Next?";
        q->labels = { "Yes" };
    };
    answerPendingQuestion(q, 0);
    CHECK(found == "ran");
    CHECK(q != nullptr && q->text == "Next?");   // the new card survived
}

static void test_answer_noop_without_card() {
    std::unique_ptr<Question> q;
    answerPendingQuestion(q, 0);   // must not crash
    CHECK(q == nullptr);
}

int main() {
    test_prune_drops_stale();
    test_prune_keeps_valid();
    test_problem_double_seat();
    test_problem_not_at_pad();
    test_problem_empty_seat_and_ok();
    test_erase_rows();
    test_target_identity();
    test_drop_question();
    test_answer_closes_first();
    test_answer_noop_without_card();
    if(failures == 0) { printf("test_hooks: OK\n"); }
    return failures == 0 ? 0 : 1;
}
