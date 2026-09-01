// job.cpp -- the single-worker job runner (see job.h for the contract).
//
// One worker thread drains the task queue in order. Each task's body runs
// off-thread (pure math over a snapshot); when it finishes it has returned
// the main-thread continuation, which is parked in `done_` and run by the
// next JobRunner::poll() (the main loop). That ordering -- worker produces,
// main thread applies -- is the whole synchronization story: the two never
// touch the same data at once, and the mutex only guards the handoff
// deques and the status counters.
#include "job.h"

#include <exception>

void JobRunner::run() {
    for(;;) {
        Task t;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [&] { return stop_ || !tasks_.empty(); });
            if(stop_ && tasks_.empty()) { break; }
            t = std::move(tasks_.front());
            tasks_.pop_front();
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            current_ = t.label;
        }
        // The body runs off-thread; it must be pure (no game state / GL /
        // imgui) and returns the main-thread continuation. A throw is
        // caught so the worker keeps serving and the job simply reports no
        // result (its apply is empty, so nothing is published).
        std::function<void()> apply;
        try {
            apply = t.body();
        } catch(const std::exception &) {
            apply = nullptr;
        } catch(...) {
            apply = nullptr;
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            done_.push_back(Done{std::move(apply)});
        }
    }
}

std::string JobRunner::poll() {
    std::vector<std::function<void()>> applies;
    std::string label;
    {
        std::lock_guard<std::mutex> lk(mu_);
        for(Done &d : done_) {
            if(d.apply) { applies.push_back(std::move(d.apply)); }
        }
        const int n = (int)done_.size();
        done_.clear();
        in_flight_ -= n;   // every landed job (apply or not) counts
        if(in_flight_ > 0) { label = current_; }
    }
    // Run the continuations OUTSIDE the lock so an apply can safely do
    // anything, including posting another job (no re-entrant deadlock).
    for(std::function<void()> &a : applies) {
        a();
    }
    return label;
}

bool JobRunner::busy() const {
    std::lock_guard<std::mutex> lk(mu_);
    return in_flight_ > 0;
}

void JobRunner::join() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_.notify_one();
    worker_.join();
}
