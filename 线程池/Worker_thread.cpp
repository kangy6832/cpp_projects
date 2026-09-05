#include "Worker_thread.hpp"

#include <optional>
#include <utility>

Worker_thread::Worker_thread(Queue& queue, std::string name)
    : queue_(queue), name_(std::move(name)) {}

Worker_thread::~Worker_thread() {
    request_stop();
    join();
}

void Worker_thread::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;  // 已在运行，避免重复启动
    }
    stop_.store(false, std::memory_order_release);

    thread_ = std::thread([this] {
        run();
        running_.store(false, std::memory_order_release);
    });
}

void Worker_thread::request_stop() noexcept {
    stop_.store(true, std::memory_order_release);
    // wait_pop 只在"队列关闭且排空"或"有新任务"时返回，
    // 因此必须关闭队列才能把阻塞中的工作线程唤醒（close 幂等，可重复调用）
    queue_.close();
}

void Worker_thread::join() {
    // 禁止自 join（会抛 system_error / 死锁）
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

void Worker_thread::run() {
    while (!stop_.load(std::memory_order_acquire)) {
        // 队列空时阻塞在此；返回 nullopt 表示队列已关闭且排空
        std::optional<Queue::Task> task = queue_.wait_pop();
        if (!task) {
            break;
        }

        // 任务在锁外执行；异常必须拦住，否则会终止整个线程
        try {
            (*task)();
            executed_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
            failed_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}
