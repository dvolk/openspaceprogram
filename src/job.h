// job.h -- a single-worker background job runner for the main loop.
//
// Long computations (the porkchop grid and the surface map; terrain gen
// later) run on one worker thread instead of stalling the frame. The main
// loop calls JobRunner::poll() once per frame; poll() runs the finished
// jobs' MAIN-THREAD continuations (which publish the result into game state
// and poke the UI) and returns the label of the job still running or
// queued, for the "working on it" indicator.
//
// A job's BODY runs on the worker thread, so it must only touch its own
// snapshot of the inputs (pure math) -- not game state, GL or imgui. When
// it is done it RETURNS a small continuation; poll() runs that on the main
// thread. That split is what keeps it safe: the worker never reads or
// writes shared game state, and every write to it happens where the UI
// already does its work. To adopt it, snapshot the inputs on the main
// thread, post a body that does the pure work and returns an apply lambda,
// and let the apply publish the result.
#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class JobRunner {
public:
    JobRunner() : worker_(&JobRunner::run, this) {}

    JobRunner(const JobRunner &) = delete;
    JobRunner &operator=(const JobRunner &) = delete;

    // The worker is joined (draining any queued jobs) on destruction.
    ~JobRunner() { join(); }

    // Enqueue a job for the worker thread. `label` is what the UI shows
    // while it runs ("Porkchop grid", "Surface map", ...). `body` runs OFF
    // the main thread and RETURNS the main-thread continuation (an empty
    // std::function = none) that publishes the result. `body` must be safe
    // to run off-thread (pure math over a snapshot); the returned
    // continuation is what may touch game state / the UI.
    template <class Body>
    void post(const std::string &label, Body &&body) {
        std::function<std::function<void()>()> fn(std::forward<Body>(body));
        {
            std::lock_guard<std::mutex> lk(mu_);
            tasks_.push_back(Task{label, std::move(fn)});
            in_flight_++;
        }
        cv_.notify_one();
    }

    // Main thread, once per frame: run the finished jobs' continuations
    // (in order) and return the label of the job still running or queued
    // ("" when idle) -- the "working on it" indicator.
    std::string poll();

    // True while any posted job has not fully landed (queued, running, or
    // finished-but-not-yet-applied).
    bool busy() const;

    // Block until every posted job has finished (clean shutdown).
    // Idempotent: the destructor joins too, so callers that join first
    // (main.cpp does, before freeing the state a job body may hold) are
    // safe.
    void join();

private:
    void run();

    struct Task {
        std::string label;
        std::function<std::function<void()>()> body;
    };
    struct Done {
        std::function<void()> apply;
    };

    // State first, the worker thread LAST: members initialize in
    // DECLARATION order, and the std::thread ctor starts the thread
    // immediately. Declaring worker_ first let run() race this ctor --
    // it locked mu_ before mu_'s own ctor had run, over whatever bytes
    // were already in that stack slot. Usually harmless (fresh stack
    // pages are zeroed, which reads like a fresh mutex); rarely the
    // garbage reads as a robust mutex with a dead owner and
    // std::mutex::lock() throws system_error (EOWNERDEAD, "Owner died"),
    // terminating the process at load (TSAN: the ctor's mutex-init write
    // vs the worker's first lock, no happens-before between them).
    // With worker_ last, pthread_create is the final initialization step,
    // so the thread-start synchronization edge covers all the state above.
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Task> tasks_;
    std::deque<Done> done_;
    std::string current_ = "";   // the running job's label ("" when idle)
    int in_flight_ = 0;          // posted but not yet applied
    bool stop_ = false;
    std::thread worker_;
};
