#ifndef WORKER_THREAD_H_HPP
#define WORKER_THREAD_H_HPP

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "Queue_h.hpp"

class Worker_thread_h {
public:
    explicit Worker_thread_h(Queue_h& queue, std::string name = "worker");
    ~Worker_thread_h();
    Worker_thread_h(const Worker_thread_h&) = delete;
    Worker_thread_h& operator=(const Worker_thread_h&) = delete;

    void start();
    void request_stop() noexcept;
    void join();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    std::uint64_t executed() const noexcept { return executed_.load(std::memory_order_acquire); }
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_acquire); }
    const std::string& name() const noexcept { return name_; }

private:
    void run();

    Queue_h& queue_;
    std::string name_;
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> executed_{0};
    std::atomic<std::uint64_t> failed_{0};

}


#endif