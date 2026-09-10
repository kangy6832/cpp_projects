#include "Producer_thread_h.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

namespace {
    constexpr auto kPushSlice = 50ms;
}

Producer_thread_h::Producer_thread_h(Queue_h& queue, Generator gen, std::string name) 
    : queue_(queue), gen_(gen), name_(std::move(name)) {}

Producer_thread_h::~Producer_thread_h() {
    request_stop();
    join();
}

void Producer_thread_h::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }

    stop_.store(false, std::memory_order_relaxed);

    thread_ = std::thread([this]  {
        run();
        running_.store(false, std::memory_order_release);
    });


}

void Producer_thread_h::request_stop() {
    stop_.store(true, std::memory_order_release);
}

void Producer_thread_h::join() {
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

void Producer_thread_h::run() {
    while (!stop_.load(std::memory_order_acquire)) {
        std::optional<Task> task;
        try {
            task = gen_;

        } catch (...) {
            failed_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        if (!task) {
            break;
        }

        auto result = queue_.wait_push_for(std::move(*task), kPushSlice);

        if (result == PushResult::Closed) {
            break;
        } 
        if (result == PushResult::Timeout) {
            continue;
        }

        produced_.fetch_add(1, std::memory_order_relaxed);
    }
}

