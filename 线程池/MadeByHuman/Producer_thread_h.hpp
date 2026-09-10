#ifndef PRODUCER_THREAD_H
#define PRODUCER_THREAD_H

// ... (rest of the header file content)

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "Queue_h.hpp"

class Producer_thread_h {
public:
    using Task = Queue_h::value_type;

    using Generator = std::function<std::optional<Task>()>;

    Producer_thread_h(Queue_h& queue, Generator gen, std::string name = "producer_h");
    ~Producer_thread_h();

    Producer_thread_h(const Producer_thread_h&) = delete;
    Producer_thread_h& operator=(const Producer_thread_h&) = delete;


    void start();
    void request_stop();
    void join();


    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    std::uint64_t producted() const noexcept { return produced_.load(std::memory_order_acquire); }
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_acquire); }
    const std::string& name() const noexcept { return name_; }


private:
    void run();

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> produced_{0};
    std::atomic<std::uint64_t> failed_{0};

    Queue_h& queue_;
    Generator gen_;
    std::string name_;

};


#endif // PRODUCER_THREAD_H