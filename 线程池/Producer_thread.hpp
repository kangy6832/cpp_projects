#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "Queue.hpp"

// 生产者线程：独立线程运行，从数据源不断取任务并投递到共享 Queue。
//
// 设计要点：
//   1. 背压：wait_push_for 在队列满时阻塞，天然限流，不会把内存打爆
//   2. 停止：request_stop() 只置标志，阻塞中的 push 靠超时返回后检查标志退出
//   3. 数据源解耦：Generator 返回 nullopt 表示数据耗尽，线程自然结束
//   4. RAII：析构保证 join，绝不留下可 join 的 thread（否则 std::terminate）
//
// 若 Queue 额外提供 notify_producers()，可在 request_stop() 中调用它
// 实现零延迟唤醒（见 .cpp 中注释）。
class Producer_thread {
public:
    using Task = Queue::value_type;

    // 返回 nullopt 表示数据源耗尽；每次调用应产出一个任务
    using Generator = std::function<std::optional<Task>()>;

    Producer_thread(Queue& queue, Generator gen, std::string name = "producer");
    ~Producer_thread();

    Producer_thread(const Producer_thread&) = delete;
    Producer_thread& operator=(const Producer_thread&) = delete;

    void start();
    void request_stop() noexcept;   // 非阻塞，只置标志
    void join();

    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    std::uint64_t produced() const noexcept { return produced_.load(std::memory_order_relaxed); }
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_relaxed); }
    const std::string& name() const noexcept { return name_; }

private:
    void run();

    Queue& queue_;
    Generator gen_;
    std::string name_;

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> produced_{0};
    std::atomic<std::uint64_t> failed_{0};
};
