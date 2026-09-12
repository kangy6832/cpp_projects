#include "Worker_thread_h.hpp"

#include <optional>
#include <utility>

Worker_thread_h::Worker_thread_h(Queue_h& queue, std::string name) : 
    queue_(queue), name_(std::move(name)) {}

Worker_thread_h::~Worker_thread_h() {
    request_stop();
    join();
}

void Worker_thread_h::start() {
    bool excepted = false;
    if (!running_.compare_exchange_strong(excepted, true, std::memory_order_acq_rel)) {
        return;
    }
    stop_.store(false, std::memory_order_release);

    thread_ = std::thread( [this] {
        run();
        running_.store(false, std::memory_order_release);
    });
}

void Worker_thread_h::join() {
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

void Worker_thread_h::request_stop() noexcept {
    stop_.store(true, std::memory_order_release);
    queue_.close();
}

void Worker_thread_h::run() {
    while (!stop_.load(std::memory_order_acquire)) {
        std::optional<Queue_h::Task> task = queue_.wait_pop();
        if (!task) {
            break;
        }
        try {
            (*task) ();
            executed_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
            failed_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}