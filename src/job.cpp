// job.cpp -- the job runner implementation (contract in job.h).

#include "job.h"

#include <cstdio>
#include <exception>

void JobRunner::run() {
    for(;;) {
        Task t;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [&] { return stop_ || !tasks_.empty(); });
            if(stop_ && tasks_.empty()) { break; }
            // pop_heap moves the front task to the back, hence the
            // move-and-pop-back rather than a pop_front.
            std::pop_heap(tasks_.begin(), tasks_.end(), UrgentFirst());
            t = std::move(tasks_.back());
            tasks_.pop_back();
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            current_ = t.label;
        }
        // Catch so the worker keeps serving; the job reports no result.
        std::function<void()> apply;
        try {
            apply = t.body();
        } catch(const std::exception &e) {
            // Say what threw (a silent swallow reads as a stuck job).
            printf("[job-throw] %s: %s\n", t.label.c_str(), e.what());
            apply = nullptr;
        } catch(...) {
            printf("[job-throw] %s: unknown exception\n", t.label.c_str());
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
    // Run the continuations OUTSIDE the lock so an apply can post another job.
    for(std::function<void()> &a : applies) {
        a();
    }
    return label;
}

bool JobRunner::busy() const {
    std::lock_guard<std::mutex> lk(mu_);
    return in_flight_ > 0;
}

bool JobRunner::stopping() const {
    std::lock_guard<std::mutex> lk(mu_);
    return stop_;
}

void JobRunner::join() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if(stop_) { return; }   // already joined (idempotent; the dtor joins)
        stop_ = true;
    }
    cv_.notify_one();
    worker_.join();
}

void JobRunner::abort() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if(stop_) { return; }   // already stopped (idempotent; the dtor joins)
        stop_ = true;
        tasks_.clear();         // drop the queued jobs
    }
    cv_.notify_one();
    worker_.join();
}

void JobRunner::restart() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if(!stop_) { return; }   // worker already live
        stop_ = false;
        tasks_.clear();          // abort/join left it empty; be safe
        done_.clear();           // drop the aborted stream's continuations
        current_.clear();
        in_flight_ = 0;
    }
    // The old worker is joined, so move-assigning a fresh thread is valid.
    worker_ = std::thread(&JobRunner::run, this);
}
