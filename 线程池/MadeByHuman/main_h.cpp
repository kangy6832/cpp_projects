#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>

#include "Producer_thread_h.hpp"
#include "Queue_h.hpp"
#include "Worker_thread_h.hpp"

using namespace std::chrono_literals;

int main() {
    constexpr std::size_t kCapacity = 32;
    constexpr int kTaskCount = 200;
    constexpr int kWorkerCount = 4;
    constexpr int kProducerCount = 2;

    Queue_h queue(kCapacity);
    std::atomic<int> next_task{0};
    std::atomic<int> finished{0};
    std::mutex out_mtx;
    std::deque<Worker_thread_h> workers;
    for (int i = 0; i < kWorkerCount; ++i) {
        workers.emplace_back(queue, "worker-" + std::to_string(i));
        workers.back().start();
    }

    std::deque<Producer_thread_h> producers;
    for (int p = 0; p < kProducerCount; ++p) {
        Producer_thread_h::Generator gen = [&]() -> std::optional<Queue_h::value_type> {
            const int id = next_task.fetch_add(1, std::memory_order_relaxed);

            if (id >= kTaskCount) {
                return std::nullopt;
            }

            return [id, kTaskCount, &finished, &out_mtx] {
                std::this_thread::sleep_for(10ms);

                const int n = finished.fetch_add(1, std::memory_order_relaxed) + 1;
                if (n % 50 == 0) {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::cout << "[worker] 已完成" << n << " / " << kTaskCount << " 个任务\n";
                }
            };
        };

        producers.emplace_back(queue, std::move(gen), "producer-" + std::to_string(p));
        producers.back().start();
    }

    for (auto& p : producers) p.join();
    queue.close();
    for (auto& w : workers) w.join();

    std::uint64_t produced = 0;
    for (const auto& p : producers) produced += p.produced();

    std::uint64_t executed = 0, failed = 0;
    for (const auto& w : workers) {
        executed += w.executed();
        failed += w.failed();
    }

    std::cout << "\n===== 统计 =====\n"
            << "任务总数     : " << kTaskCount << '\n'
            << "生产者投递   : " << produced << '\n'
            << "消费者执行   : " << executed << '\n'
            << "任务执行失败 : " << failed << '\n'
            << "队列残留     : " << queue.size() << '\n'
            << "校验         : " << (produced == kTaskCount && executed == kTaskCount &&
                                    failed == 0 && queue.empty()
                                        ? "PASS（无丢失、无重复、无残留）"
                                        : "FAIL")
            << std::endl;

    return 0;
}