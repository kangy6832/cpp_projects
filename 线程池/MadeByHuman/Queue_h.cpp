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
    if (closed_) return PushResult::Closed;

    if (!full_locked()) {
        if (!not_full_.wait_for(lock_, timeout, [this] {return closed_ || !full_locked(); })) {
            return PushResult::Timeout;
        }

        if (closed_) {
            return PushResult::Closed;
        }
    }

    tasks_.push_back(std::move(task));
    lock_.unlock();
    not_empty_.notify_one();
    return PushResult::Ok;
}





std::optional<Queue_h::Task> Queue_h::wait_pop() {

}

std::optional<Queue_h::Task> Queue_h::try_pop() {

}

void Queue_h::close() {

}

void Queue_h::closed() const {

}

bool Queue_h::empty() const {

}

std::size_t Queue_h::capacity() const {

}

std::size_t Queue_h::size() const {

}