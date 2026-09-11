#ifndef QUEUE_H
#define QUEUE_H

#include "chrono"
#include "condition_variable"
#include "cstddef"
#include "deque"
#include "functional"
#include "mutex"
#include "optional"

enum class PushResult { Ok, Timeout, Closed };

class Queue_h {
public:
    using Task = std::function<void()>;
    using value_type = Task;    

    explicit Queue_h(std::size_t capacity = 0);

    Queue_h(const Queue_h&) = delete;
    Queue_h operator=(const Queue_h&) = delete;

    // 生产者侧
    PushResult wait_push(Task task);
    PushResult wait_push_for(Task task, std::chrono::milliseconds timeout);

    // 消费者侧
    std::optional<Queue_h::Task> wait_pop();
    std::optional<Queue_h::Task> try_pop();

    // 状态查询与关闭
    void close();
    bool closed() const;
    bool empty() const;
    std::size_t size() const;
    std::size_t capacity() const;

private:
    bool full_locked() const {
        return capacity_ > 0 && tasks_.size() >= capacity_;
    }

    mutable std::mutex mtx_;
    std::deque<Task> tasks_;
    
    std::condition_variable not_full_;
    std::condition_variable not_empty_;

    std::size_t capacity_;
    bool closed_ = false;

};

#endif