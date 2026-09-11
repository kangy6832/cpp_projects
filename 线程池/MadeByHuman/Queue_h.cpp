#include "Queue_h.hpp"
#include <utility>

Queue_h::Queue_h(std::size_t capacity) : capacity_(capacity) {}

PushResult Queue_h::wait_push(Task task) {
    std::unique_lock<std::mutex> lock_(mtx_);
    not_full_.wait(lock_, [this] {return closed_ || !full_locked(); });
    if (closed_) return PushResult::Closed;
    tasks_.push_back(std::move(task));
    lock_.unlock();
    not_empty_.notify_one();
    return PushResult::Ok;
}

// PushResult Queue_h::wait_push_for(Task task, std::chrono::milliseconds timeout) {
//     std::unique_lock<std::mutex> lock(mtx_);
//     if (closed_) return PushResult::Closed;

//     if (full_locked()) {
//         if (!not_full_.wait_for(lock, timeout, [this] {return closed_ || !full_locked(); })) {
//             return PushResult::Timeout;
//         }

//         if (closed_) {
//             return PushResult::Closed;
//         }
//     }

//     tasks_.push_back(std::move(task));
//     lock.unlock();
//     not_empty_.notify_one();
//     return PushResult::Ok;
// }

PushResult Queue_h::wait_push_for(Task task, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock_(mtx_);
    if (closed_) {
        return PushResult::Closed;
    }

    if (!full_locked()) {
        if (!not_full_.wait_for(lock_, timeout, [this] { return !full_locked() || closed_; })) {
            return PushResult::Timeout;
        }
        if (closed_) {
            return PushResult::Closed;
        }
    }

    tasks_.push_back(task);
    lock_.unlock();
    not_empty_.notify_one();

    return PushResult::Ok;
}

std::optional<Queue_h::Task> Queue_h::wait_pop() {
    std::unique_lock<std::mutex> lock_(mtx_);
    not_empty_.wait(lock_, [this] { return closed_ || !tasks_.empty(); });

    if (tasks_.empty()) return std::nullopt;

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock_.unlock();
    not_full_.notify_one();
    return task;
}

std::optional<Queue_h::Task> Queue_h::try_pop() {
    std::unique_lock<std::mutex> lock_(mtx_);
    if (tasks_.empty()) return std::nullopt;

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock_.unlock();
    not_full_.notify_one();
    return task;
}

void Queue_h::close() {
    std::lock_guard<std::mutex> lock_(mtx_); // 自动给 std::mutex 加锁， 并离开作用域时解锁， 作用域是 close 函数
    closed_ = true;

    not_empty_.notify_all();
    not_full_.notify_all();
}

bool Queue_h::closed() const {
    std::lock_guard<std::mutex> lock_(mtx_);
    return closed_;
}

bool Queue_h::empty() const {
    std::lock_guard<std::mutex> lock_(mtx_);
    return tasks_.empty();
}

std::size_t Queue_h::capacity() const {
    return capacity_;
}

std::size_t Queue_h::size() const {
    std::lock_guard<std::mutex> lock_(mtx_);
    return tasks_.size();
}