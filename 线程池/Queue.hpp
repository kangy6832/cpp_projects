#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>

// push 的结果：Ok 入队成功 / Closed 队列已关闭 / Timeout 等待超时
enum class PushResult { Ok, Closed, Timeout };

// 有界阻塞任务队列，生产者与消费者之间唯一的耦合点。
//
// 语义约定：
//   1. capacity == 0 表示无界（不做背压）
//   2. close() 后不再接收新任务（push 返回 Closed），但已入队的任务仍可被 pop 取走，
//      直到队列排空后 wait_pop 返回 nullopt —— 即优雅关闭语义
//   3. 队列满时 push 阻塞、队列空时 pop 阻塞，两端都通过条件变量唤醒
class Queue {
public:
    using Task = std::function<void()>;
    using value_type = Task;

    explicit Queue(std::size_t capacity = 0);

    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    // ---- 生产者侧 ----
    PushResult wait_push(Task task);                                    // 满则一直等
    PushResult wait_push_for(Task task, std::chrono::milliseconds t);   // 满则最多等 t

    // ---- 消费者侧 ----
    // 阻塞直到取到任务；队列已关闭且排空时返回 nullopt
    std::optional<Task> wait_pop();
    // 非阻塞取任务，队列空立即返回 nullopt
    std::optional<Task> try_pop();

    // ---- 状态 ----
    void close();
    bool closed() const;
    bool empty() const;
    std::size_t size() const;
    std::size_t capacity() const;

private:
    bool full_locked() const { return capacity_ > 0 && tasks_.size() >= capacity_; }

    mutable std::mutex mtx_;
    std::condition_variable not_full_;    // 唤醒被背压阻塞的生产者
    std::condition_variable not_empty_;   // 唤醒空闲的消费者
    std::deque<Task> tasks_;
    std::size_t capacity_;
    bool closed_ = false;
};
