// job.h -- a single-worker background job runner. A job's BODY runs
// off-thread (pure math over a snapshot) and RETURNS a main-thread
// continuation; poll() runs that on the main thread. The worker never
// touches shared game state.

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

    // Enqueue a job. `body` runs OFF the main thread and RETURNS the
    // main-thread continuation. `body` must be pure (snapshot inputs only).
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

    // Main thread, once per frame: run finished jobs' continuations and
    // return the label of the job still running ("" when idle).
    std::string poll();

    // True while any posted job has not fully landed.
    bool busy() const;

    // True once abort()/join() has asked the worker to stop. A long job body
    // should check this between its own steps: abort() drops QUEUED jobs but
    // still joins the in-flight one, so an uncancellable body stalls the
    // caller (load, system swap, exit) for its whole remaining runtime.
    // Safe to call from the worker.
    bool stopping() const;

    // Block until every posted job has finished. Idempotent (the dtor joins too).
    void join();

    // Discard QUEUED jobs and stop (wait only for the in-flight job so it
    // finishes reading its snapshot). Idempotent. Use at hard shutdown.
    void abort();

    // Recreate the worker after abort()/join(). A no-op while running.
    void restart();

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
    // DECLARATION order and the std::thread ctor starts immediately.
    // worker_ first let run() race this ctor (locking mu_ before its ctor
    // had run), which could throw EOWNERDEAD at load.
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Task> tasks_;
    std::deque<Done> done_;
    std::string current_ = "";   // the running job's label ("" when idle)
    int in_flight_ = 0;          // posted but not yet applied
    bool stop_ = false;
    std::thread worker_;
};
