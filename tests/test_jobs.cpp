// test_jobs: the background JobRunner (src/job.cpp).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_jobs.cpp src/job.cpp -o test_jobs && ./test_jobs)
//
// Pins the worker / main-thread handoff: the job BODY runs off the calling
// thread, its returned continuation runs on the thread that calls poll(),
// jobs land in the order they were posted, a throwing body does not kill the
// worker, and busy() / poll() report the right state (including the running
// job's label for the "working on it" indicator).
#include "job.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <functional>
#include <stdexcept>   // std::runtime_error (the throwing-body case)
#include <string>
#include <thread>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static long tid() {
    return (long)std::hash<std::thread::id>{}(std::this_thread::get_id());
}

// poll() until the runner is idle (or ~2 s), so a lost job fails loudly
// instead of hanging the test.
static void pump(JobRunner &jr) {
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(2);
    while(jr.busy() && std::chrono::steady_clock::now() < deadline) {
        jr.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    jr.poll();   // final drain
}

int main() {
    const long main_id = tid();

    // =========================================================================
    // 1. Body off-thread, apply on the poll thread, result published.
    // =========================================================================
    {
        JobRunner jr;
        int body_ran = 0, applied = 0, result = 0;
        long body_id = -1, apply_id = -1;
        jr.post("job", [&]() -> std::function<void()> {
            body_ran = 1;
            body_id = tid();
            const int work = 42;   // the "pure" result computed off-thread
            // `work` is a body-local: capture it BY VALUE so the continuation
            // (which runs later, on the poll thread) does not read a dangling
            // reference. result/applied/apply_id are main()-locals that outlive
            // both, so they stay by reference.
            return [work, &result, &applied, &apply_id]() {
                applied = 1;
                apply_id = tid();
                result = work;
            };
        });
        CHECK(jr.busy());
        pump(jr);
        CHECK(!jr.busy());
        CHECK(body_ran == 1);
        CHECK(applied == 1);
        CHECK(result == 42);
        CHECK(body_id != -1 && body_id != main_id);  // body ran off the main thread
        CHECK(apply_id == main_id);                  // apply ran on the poll thread
    }

    // =========================================================================
    // 2. Ordering: two jobs land in the order they were posted.
    // =========================================================================
    {
        JobRunner jr;
        int order[2] = { -1, -1 };
        int seq = 0;
        jr.post("A", [&]() -> std::function<void()> {
            return [&]() { order[seq++] = 1; };
        });
        jr.post("B", [&]() -> std::function<void()> {
            return [&]() { order[seq++] = 2; };
        });
        pump(jr);
        CHECK(!jr.busy());
        CHECK(order[0] == 1 && order[1] == 2);
    }

    // =========================================================================
    // 3. A job with no continuation still completes (busy clears).
    // =========================================================================
    {
        JobRunner jr;
        int body_ran = 0;
        jr.post("no-apply", [&]() -> std::function<void()> {
            body_ran = 1;
            return std::function<void()>();   // empty = no main-thread apply
        });
        pump(jr);
        CHECK(!jr.busy());
        CHECK(body_ran == 1);
    }

    // =========================================================================
    // 4. A throwing body does not kill the worker: the job no-ops and the
    //    worker keeps serving subsequent jobs.
    // =========================================================================
    {
        JobRunner jr;
        int threw = 0, after = 0;
        jr.post("throw", [&]() -> std::function<void()> {
            threw = 1;
            throw std::runtime_error("boom");
            // Unreachable: satisfies -Wreturn-type (the body always throws).
            return std::function<void()>();
        });
        pump(jr);
        CHECK(!jr.busy());
        CHECK(threw == 1);
        // Worker still alive: a follow-up job completes.
        jr.post("after", [&]() -> std::function<void()> {
            return [&]() { after = 1; };
        });
        pump(jr);
        CHECK(!jr.busy());
        CHECK(after == 1);
    }

    // =========================================================================
    // 5. join() is idempotent: a job posted before join() still lands, and
    //    a second join() (the destructor does one too -- a double
    //    std::thread::join() would be UB) is a no-op.
    // =========================================================================
    {
        JobRunner jr;
        int done = 0;
        jr.post("drain", [&]() -> std::function<void()> {
            return [&]() { done = 1; };
        });
        jr.join();
        jr.poll();   // apply the drained job's continuation
        jr.join();   // the dtor joins again on scope exit
        CHECK(done == 1);
    }

    // =========================================================================
    // 6. poll() reports the running job's label while busy and "" when idle.
    //    The body blocks on `release` so it is GUARANTEED in flight when the
    //    test polls (no race on observing the label).
    // =========================================================================
    {
        JobRunner jr;
        CHECK(jr.poll() == std::string());   // idle -> empty label
        std::atomic<bool> release{false};
        std::atomic<bool> in_body{false};
        jr.post("MyLabel", [&]() -> std::function<void()> {
            in_body = true;
            while(!release.load()) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
            return []() {};
        });
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::seconds(2);
        while(!in_body.load() && std::chrono::steady_clock::now() < deadline) {
            jr.poll();
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        CHECK(in_body.load());
        CHECK(jr.busy());
        CHECK(jr.poll() == std::string("MyLabel"));   // label while in flight
        release.store(true);
        pump(jr);
        CHECK(!jr.busy());
        CHECK(jr.poll() == std::string());   // empty once idle
    }

    // =========================================================================
    // 7. abort() drops the queued jobs (does NOT drain them).
    //
    // Each body is slow (a short sleep) so a full drain of 20 would take
    // ~1 s. abort() returns after the in-flight one (if any) and discards the
    // rest: far fewer bodies ever run, and the call returns well before a
    // drain would. join() (the tested drain contract, case 5) is unchanged.
    // =========================================================================
    {
        JobRunner jr;
        std::atomic<int> ran{0};
        for(int i = 0; i < 20; i++) {
            jr.post("slow", [&]() -> std::function<void()> {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                ran++;
                return []() {};
            });
        }
        const auto t0 = std::chrono::steady_clock::now();
        jr.abort();
        const long dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        // A drain would be ~1000 ms and run all 20; abort() must be far
        // quicker and drop the queue (only the in-flight body, if any, runs).
        // dt_ms is the robust signal (a drain can't return this fast); the ran
        // bound just proves at least one job was dropped (< 20 = not all ran)
        // -- kept generous so a descheduled post loop can't false-fail it.
        CHECK(dt_ms < 500);
        CHECK(ran.load() < 20);
    }

    // =========================================================================
    // 8. restart() resurrects a stopped runner. abort() is terminal for the
    //    worker thread (a joined std::thread cannot run again), so a runner is
    //    otherwise single-use. The in-process system switch aborts the old
    //    system's terrain stream and then posts the new one's, so restart()
    //    must reset the stop latch and spawn a fresh worker.
    // =========================================================================
    {
        JobRunner jr;
        int first = 0;
        jr.post("old-system", [&]() -> std::function<void()> {
            return [&]() { first = 1; };
        });
        pump(jr);
        CHECK(first == 1);
        jr.abort();          // the worker thread exits (terminal)
        jr.restart();        // the linchpin: the worker is back
        int second = 0;
        jr.post("new-system", [&]() -> std::function<void()> {
            return [&]() { second = 1; };
        });
        pump(jr);
        CHECK(!jr.busy());
        CHECK(second == 1);  // the post-abort job ran on the restarted worker
    }

    // =========================================================================
    // 9. restart() discards pending completions (done_) -- a job that
    //    finished (body done, apply not yet run) before abort() must NOT run
    //    its apply after restart(). This is the branch the switch path hits:
    //    the old system's terrain finished in the worker, and the switch
    //    aborts before the main loop polls (so done_ is non-empty at abort).
    // =========================================================================
    {
        JobRunner jr;
        int old_apply = 0;
        std::atomic<bool> body_done{ false };
        jr.post("finished-not-polled", [&]() -> std::function<void()> {
            // Set the flag just before returning: when the test sees it, the
            // body has finished and its apply is parked in done_ (the branch
            // under test -- abort() must discard it, not run it).
            body_done = true;
            return [&]() { old_apply = 1; };
        });
        // Spin (with a deadline) until the body has finished -- this proves
        // the completion is in done_, unlike a fixed sleep that could expire
        // before the worker picks the job up.
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::seconds(2);
        while(!body_done && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        CHECK(body_done);   // the body finished; its apply is in done_
        jr.abort();         // discards done_ + in_flight_ (the old_apply is dropped)
        jr.restart();
        int new_job = 0;
        jr.post("new-system", [&]() -> std::function<void()> {
            return [&]() { new_job = 1; };
        });
        pump(jr);
        CHECK(old_apply == 0);  // the discarded completion never ran
        CHECK(new_job == 1);    // the new job ran on the restarted worker
    }

    if(failures == 0) {
        printf("test_jobs: all checks passed\n");
        return 0;
    }
    printf("test_jobs: %d FAILURES\n", failures);
    return 1;
}
