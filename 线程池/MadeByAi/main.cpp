// ============================================================================
//  main.cpp —— 演示程序：把三个组件串成一条完整的流水线
// ============================================================================
//
//  【整体架构一览】
//
//      ┌──────────────────┐                        ┌──────────────────┐
//      │ Producer_thread  │ ──┐                ┌──>│  Worker_thread   │
//      │   producer-0     │   │                │   │    worker-0      │
//      └──────────────────┘   │                │   └──────────────────┘
//                             ├──> ┌────────┐ <┤   ┌──────────────────┐
//      ┌──────────────────┐   │    │ Queue  │  ├──>│  Worker_thread   │
//      │ Producer_thread  │ ──┘    │容量 32 │  │   │    worker-1      │
//      │   producer-1     │        └────────┘  │   └──────────────────┘
//      └──────────────────┘                    │            ...
//                                              └──> ┌──────────────────┐
//           2 个生产者                               │  Worker_thread   │
//                                                   │    worker-3      │
//                                                   └──────────────────┘
//                                                        4 个消费者
//
//  【数据流】
//    1. 两个生产者共享同一个原子计数器 next_task，各自抢号生成任务 id
//    2. 任务被投递到容量 32 的有界队列（队列满则生产者阻塞 = 背压）
//    3. 四个消费者并发地从队列取任务并执行
//
//  【本文件最能学的两点】
//    ① 关闭顺序：生产者 join → 队列 close → 消费者 join（顺序绝不能乱）
//    ② 为什么用 std::deque 而不是 std::vector 装线程对象
//
//  ==========================================================================

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
    constexpr int kTaskCount = 200;         // 任务总数（被多个生产者共享）
    constexpr int kWorkerCount = 4;
    constexpr int kProducerCount = 2;

    Queue queue(kCapacity);

    // ---------------- 共享状态 ----------------
    // 【设计要点】多个生产者要避免生成重复的任务，最简单的办法是
    // 让它们共享一个原子计数器去"抢号"。
    // fetch_add 是原子的，两个线程绝不会拿到同一个 id ——
    // 这就保证了任务不重复，无需加锁。
    std::atomic<int> next_task{0};

    // finished 只用于打印进度，不影响业务逻辑
    std::atomic<int> finished{0};

    // 【踩坑】std::cout 不是线程安全的！
    // 多个线程同时 << 会让输出内容交错成一团乱码。
    // 必须用 mutex 保护（或用 C++20 的 std::osyncstream）。
    std::mutex out_mtx;

    // ---------------- 启动消费者 ----------------
    //
    // 【踩坑·编译器会卡住你】为什么用 std::deque 而不是 std::vector？
    //   Worker_thread 内含 std::thread 和引用成员，因此：
    //     · 禁用了拷贝构造
    //     · 也没有移动构造
    //   而 vector::emplace_back 在【编译期】就要求元素类型可移动
    //   （因为扩容时要搬运元素）—— 哪怕你提前 reserve 保证不会扩容，
    //   这个要求依然存在，于是直接编译失败。
    //   deque 是分段连续存储，emplace_back 原地构造，
    //   扩容时不需要移动已有元素，因此没有这个要求。
    //   （这也是为什么 deque 常被用来装"不可移动的大对象"）
    std::deque<Worker_thread> workers;
    for (int i = 0; i < kWorkerCount; ++i) {
        workers.emplace_back(queue, "worker-" + std::to_string(i));
        workers.back().start();
    }

    // ---------------- 启动生产者 ----------------
    std::deque<Producer_thread> producers;
    for (int p = 0; p < kProducerCount; ++p) {
        // 这就是 Generator：生产者与"业务"之间的解耦点。
        // 这里的数据源是"抢号生成 id"，换成读文件、收网络包，
        // Producer_thread 的代码一行都不用改。
        //
        // 【踩坑】lambda 捕获循环变量时，[&, p] 里 p 是【按值】捕获的
        // （写了名字就是拷贝）。如果图省事写 [&] 捕获 p 的引用，
        // 循环结束后 p 早已变成 kProducerCount，所有生产者拿到的都是同一个值。
        Producer_thread::Generator gen = [&, p]() -> std::optional<Queue::value_type> {
            const int id = next_task.fetch_add(1, std::memory_order_relaxed);

            // 返回 nullopt = 告诉生产者"数据没了，你可以下班了"
            if (id >= kTaskCount) {
                return std::nullopt;
            }

            // 每个任务是一个无参 lambda，捕获自己的 id。
            // 【注意】这里按【引用】捕获 finished / out_mtx：
            // 它们都是 main 的局部变量，生命周期必须长于任务的执行。
            // 下面 join 过所有线程之后 main 才返回，所以这里是安全的。
            // 若线程池的生命周期长于这些变量，就必须改成 shared_ptr 捕获。
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

    // ---------------- 关闭顺序（三步，顺序绝不能乱） ----------------

    // 第 1 步：先停生产者。
    // 必须保证"不会再有新任务进来"，后面关闭队列才有意义。
    // 如果顺序反了（先关队列），生产者会收到 Closed 而抛弃剩余任务。
    for (auto& p : producers) p.join();

    // 第 2 步：关闭队列。
    // 语义：不再接收新任务，但已入队的存量任务仍可被消费者取走。
    // 这一步同时通过 notify_all 唤醒所有阻塞/空闲的消费者。
    queue.close();

    // 第 3 步：等消费者把队列排空后自然退出。
    // 每个消费者取完最后一个任务后，wait_pop 返回 nullopt，循环结束。
    for (auto& w : workers) w.join();

    // ---------------- 统计与校验 ----------------
    // 把所有线程的计数器汇总，验证"不丢、不重、不残留"。
    // 这类校验是并发程序最重要的自测手段：
    // 并发 bug 往往不报错，只是悄悄丢任务，只能靠计数发现。
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
