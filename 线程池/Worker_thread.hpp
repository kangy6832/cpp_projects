#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "Queue.hpp"

// 工作线程（消费者）：独立线程循环从 Queue 取任务并执行。
//
// 设计要点：
//   1. 唤醒依赖队列：空闲时阻塞在 Queue::wait_pop()，队列关闭且排空后返回 nullopt 退出
//   2. 异常隔离：单个任务抛异常只记 failed_ 计数，绝不杀死工作线程
//   3. 执行时不持锁：任务在锁外运行，否则线程池退化成串行
//   4. RAII：析构保证 join，绝不留下可 join 的 thread（否则 std::terminate）
//
// 注意：request_stop() 会调用 Queue::close()，共享同一 Queue 的所有线程都会因此停止。
class Worker_thread {
public:
    explicit Worker_thread(Queue& queue, std::string name = "worker");
    ~Worker_thread();

    Worker_thread(const Worker_thread&) = delete;
    Worker_thread& operator=(const Worker_thread&) = delete;

    void start();
    void request_stop() noexcept;   // 置标志并关闭队列以唤醒阻塞中的 wait_pop
    void join();

    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    std::uint64_t executed() const noexcept { return executed_.load(std::memory_order_relaxed); }
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_relaxed); }
    const std::string& name() const noexcept { return name_; }

private:
    void run();

    Queue& queue_;
    std::string name_;

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> executed_{0};
    std::atomic<std::uint64_t> failed_{0};
};
