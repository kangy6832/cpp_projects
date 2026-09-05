#include "Queue.hpp"

#include <utility>

Queue::Queue(std::size_t capacity) : capacity_(capacity) {}

// ---------------- 生产者侧 ----------------

PushResult Queue::wait_push(Task task) {
    std::unique_lock<std::mutex> lock(mtx_);
    not_full_.wait(lock, [this] { return closed_ || !full_locked(); });
    if (closed_) return PushResult::Closed;

    tasks_.push_back(std::move(task));
    lock.unlock();
    not_empty_.notify_one();
    return PushResult::Ok;
}

PushResult Queue::wait_push_for(Task task, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mtx_);

    if (closed_) return PushResult::Closed;

    if (full_locked()) {
        if (!not_full_.wait_for(lock, timeout, [this] { return closed_ || !full_locked(); })) {
            return PushResult::Timeout;   // 交给调用方去检查停止标志
        }
        if (closed_) return PushResult::Closed;
    }

    tasks_.push_back(std::move(task));
    lock.unlock();
    not_empty_.notify_one();
    return PushResult::Ok;
}

// ---------------- 消费者侧 ----------------

std::optional<Queue::Task> Queue::wait_pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    not_empty_.wait(lock, [this] { return closed_ || !tasks_.empty(); });

    if (tasks_.empty()) return std::nullopt;   // 已关闭且排空

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock.unlock();
    not_full_.notify_one();   // 腾出空位，唤醒被背压的生产者
    return task;
}

std::optional<Queue::Task> Queue::try_pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    if (tasks_.empty()) return std::nullopt;

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return task;
}

// ---------------- 状态 ----------------

void Queue::close() {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        closed_ = true;
    }
    // 两端都要唤醒，否则阻塞在 wait 上的线程永远醒不过来
    not_full_.notify_all();
    not_empty_.notify_all();
}

bool Queue::closed() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return closed_;
}

bool Queue::empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return tasks_.empty();
}

std::size_t Queue::size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return tasks_.size();
}

std::size_t Queue::capacity() const { return capacity_; }
