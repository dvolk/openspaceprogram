// test_jobs: the background JobRunner (src/job.cpp).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++11 -I./src tests/test_jobs.cpp src/job.cpp -o test_jobs && ./test_jobs)
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
    // 5. poll() reports the running job's label while busy and "" when idle.
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

    if(failures == 0) {
        printf("test_jobs: all checks passed\n");
        return 0;
    }
    printf("test_jobs: %d FAILURES\n", failures);
    return 1;
}
