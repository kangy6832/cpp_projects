#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>

#include "Producer_thread.hpp"
#include "Queue.hpp"
#include "Worker_thread.hpp"

using namespace std::chrono_literals;

int main() {
    constexpr std::size_t kCapacity = 32;   // 有界队列，用于体现背压
    constexpr int kTaskCount = 200;         // 任务总数（多生产者共享）
    constexpr int kWorkerCount = 4;
    constexpr int kProducerCount = 2;

    Queue queue(kCapacity);

    std::atomic<int> next_task{0};
    std::atomic<int> finished{0};
    std::mutex out_mtx;

    // ---------------- 启动消费者 ----------------
    // 用 deque 而非 vector：Worker_thread 不可拷贝/移动，deque 原地构造不要求可移动
    std::deque<Worker_thread> workers;
    for (int i = 0; i < kWorkerCount; ++i) {
        workers.emplace_back(queue, "worker-" + std::to_string(i));
        workers.back().start();
    }

    // ---------------- 启动生产者 ----------------
    std::deque<Producer_thread> producers;
    for (int p = 0; p < kProducerCount; ++p) {
        Producer_thread::Generator gen = [&, p]() -> std::optional<Queue::value_type> {
            const int id = next_task.fetch_add(1, std::memory_order_relaxed);
            if (id >= kTaskCount) {
                return std::nullopt;   // 数据源耗尽，生产者自然结束
            }
            return [id, kTaskCount, &finished, &out_mtx] {
                std::this_thread::sleep_for(10ms);   // 模拟耗时任务
                const int n = finished.fetch_add(1, std::memory_order_relaxed) + 1;
                if (n % 50 == 0) {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::cout << "[worker] 已完成 " << n << " / " << kTaskCount << " 个任务\n";
                }
            };
        };
        producers.emplace_back(queue, std::move(gen), "producer-" + std::to_string(p));
        producers.back().start();
    }

    // ---------------- 关闭顺序（不能乱） ----------------
    for (auto& p : producers) p.join();   // 1. 生产者先停，保证不再有新任务
    queue.close();                        // 2. 关闭队列（不收新任务，存量继续消费）
    for (auto& w : workers) w.join();     // 3. 消费者排空队列后退出

    // ---------------- 统计校验 ----------------
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
