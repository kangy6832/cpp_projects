// ============================================================================
//  线程池单元测试
// ============================================================================
//
//  运行方式：
//    cmake -B build && cmake --build build -j4
//    ctest --test-dir build --output-on-failure
//  或直接用 gtest 参数：
//    ./build/bin/threadpool_tests
//    ./build/bin/threadpool_tests --gtest_filter='QueueTest.*'
//    ./build/bin/threadpool_tests --gtest_repeat=100 --gtest_brief=1
//
//  ---------------------------------------------------------------------------
//  【为什么并发代码的测试要单独讲】
//  ---------------------------------------------------------------------------
//  普通代码是确定性的：同样的输入 → 同样的输出，测一次就够。
//  并发代码是【非确定性】的：线程调度顺序由操作系统决定，每次运行都可能不同。
//  于是会出现"flaky test"（抖动测试）—— 平时通过，偶尔失败，极难复现。
//
//  应对并发不确定性的三条铁律（本文件严格遵守）：
//
//   铁律一：断言只在主线程做
//      GTest 的 EXPECT_*/ASSERT_* 【不是】线程安全的，
//      多个线程同时断言会破坏内部状态，报出莫名其妙的错误。
//      正确做法：子线程里只写 std::atomic，全部 join 之后，
//      由主线程统一读 atomic 做断言。
//
//   铁律二：先 join，再断言
//      没 join 就断言 = 读到了执行过程中的中间状态，
//      必然导致随机失败（例如"预期 200，实际 137"）。
//
//   铁律三：测试必须有超时上限，且用限时接口
//      并发 bug 的典型表现是【死锁】—— 测试不会失败，只会永远挂着。
//      CI 里一个挂死的测试能拖垮整个流水线。
//      所以：代码里用 wait_push_for(50ms) 而不是无限等待的 wait_push；
//      ctest 自身也有默认 1500 秒的兜底超时。
//
//  【测试的分层思路】
//    · Queue 层        ：测数据结构本身的语义（顺序、有界、关闭）
//    · Producer 层     ：测生产者的行为（计数、异常、停止、背压）
//    · Worker 层       ：测消费者的行为（执行、异常隔离、退出）
//    · Integration 层  ：把三者串起来，验证端到端的正确性
//
//  ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "Producer_thread.hpp"
#include "Queue.hpp"
#include "Worker_thread.hpp"

using namespace std::chrono_literals;

// ============================================================================
//  Queue 层
// ============================================================================

// 测什么：基本的 FIFO 语义。
// 为什么需要：队列乱序会让"先来后到"的业务假设失效，是最基础的保证。
TEST(QueueTest, PreservesFIFOOrder) {
    Queue q(10);
    std::vector<int> out;
    for (int i = 0; i < 5; ++i) {
        q.wait_push([i, &out] { out.push_back(i); });
    }
    for (int i = 0; i < 5; ++i) {
        auto task = q.try_pop();
        ASSERT_TRUE(task.has_value());
        (*task)();
    }
    EXPECT_EQ(out, (std::vector<int>{0, 1, 2, 3, 4}));
}

// 测什么：有界性。队列满时 push 必须【超时失败】，而不是无限增长。
// 为什么重要：这是背压能生效的前提。如果队列悄悄扩容，
//            内存会在高负载下被吃光，且这种 bug 只在压测时才暴露。
TEST(QueueTest, BoundedCapacityBlocksPush) {
    Queue q(2);
    EXPECT_EQ(q.wait_push([] {}), PushResult::Ok);
    EXPECT_EQ(q.wait_push([] {}), PushResult::Ok);
    EXPECT_EQ(q.size(), 2u);

    // 第三个必须超时失败，否则说明容量限制失效
    EXPECT_EQ(q.wait_push_for([] {}, 50ms), PushResult::Timeout);

    // 腾出空位后应能立即入队（验证等待者确实被唤醒了）
    ASSERT_TRUE(q.try_pop().has_value());
    EXPECT_EQ(q.wait_push_for([] {}, 50ms), PushResult::Ok);
}

// 测什么：优雅关闭语义 —— close 后存量任务仍可被取完，取空后才返回 nullopt。
// 为什么重要：这是"不丢任务"的核心保证。
//            若实现写成 `if (closed_) return nullopt;`（先判关闭再取任务），
//            close() 那一刻队列里剩下的任务就全丢了，而这个测试会立刻抓到它。
TEST(QueueTest, CloseStillDrainsRemainingTasks) {
    Queue q(10);
    int executed = 0;
    for (int i = 0; i < 3; ++i) {
        q.wait_push([&executed] { ++executed; });
    }

    q.close();
    EXPECT_EQ(q.wait_push([] {}), PushResult::Closed);   // 不再接收新任务

    for (int i = 0; i < 3; ++i) {
        auto task = q.wait_pop();
        ASSERT_TRUE(task.has_value());
        (*task)();
    }
    EXPECT_EQ(executed, 3);
    EXPECT_FALSE(q.wait_pop().has_value());   // 排空后才退出
}

// 测什么：wait_pop 在队列空时应当【阻塞】，而不是立即返回 nullopt
//        （返回 nullopt 是"下班"信号，不能和"暂时没活"混淆）。
// 怎么测：起一个消费者线程，主线程先睡 30ms 确认它还没返回，再投递任务。
// 【注意】这里用了睡眠来制造时序，是并发测试的常见手段。
//        睡眠时长是"魔法数字"，理论上不够严谨，
//        但在这种"验证阻塞行为"的场景下是最实用的办法。
TEST(QueueTest, WaitPopBlocksUntilTaskArrives) {
    Queue q(4);
    std::optional<Queue::Task> popped;
    std::thread consumer([&] { popped = q.wait_pop(); });

    std::this_thread::sleep_for(30ms);
    EXPECT_FALSE(popped.has_value());   // 队列空，应当阻塞而非返回 nullopt

    q.wait_push([] {});
    consumer.join();
    EXPECT_TRUE(popped.has_value());
}

// ============================================================================
//  Producer_thread 层
// ============================================================================

// 测什么：生产者能正确产完所有任务并自然退出。
//
// 【重要·背压陷阱】这里必须用【无界队列】（capacity = 0）。
//   如果写成 Queue q(64) 而不起消费者：
//     队列填满 64 个 → 生产者被背压阻塞 → 永远推不完 100 个
//     → p.join() 永久挂死 → 测试超时。
//   这【不是】代码的 bug，而是背压的正确行为。
//   想测有界场景，就必须同时提供消费者（见下面 Backpressure 那个用例）。
TEST(ProducerTest, ProducesAllTasksThenExits) {
    Queue q(0);   // ← 无界，故意的
    std::atomic<int> next{0};

    // Generator 每次被问就发一个号，号发完返回 nullopt 表示数据源耗尽
    Producer_thread::Generator gen = [&]() -> std::optional<Queue::value_type> {
        const int id = next.fetch_add(1, std::memory_order_relaxed);
        if (id >= 100) return std::nullopt;
        return [] {};
    };

    Producer_thread p(q, gen, "p0");
    p.start();
    p.join();

    EXPECT_EQ(p.produced(), 100u);
    EXPECT_EQ(q.size(), 100u);
    EXPECT_FALSE(p.running());
}

// 测什么：数据源抛异常时，生产者要【吞掉并计数】，绝不能被异常打死。
// 为什么重要：如果异常从线程主函数逃逸 → std::terminate() → 整个进程崩溃。
//            真实场景里数据源读文件、查数据库都可能抛异常，必须容错。
TEST(ProducerTest, GeneratorExceptionIsCountedAndSkipped) {
    Queue q(0);
    std::atomic<int> calls{0};

    Producer_thread::Generator gen = [&]() -> std::optional<Queue::value_type> {
        const int n = calls.fetch_add(1, std::memory_order_relaxed) + 1;
        if (n % 2 == 1) throw std::runtime_error("bad record");   // 奇数次抛异常
        if (n >= 10) return std::nullopt;
        return [] {};
    };

    Producer_thread p(q, gen, "p1");
    p.start();
    p.join();

    EXPECT_EQ(p.failed(), 5u);    // 5 次异常被记录
    EXPECT_EQ(p.produced(), 4u);  // 其余 4 个任务正常产出
}

// 测什么：request_stop() 能让正在运行的生产者及时停下，且停下后不再投递。
//
// 【关键】数据源是【永不枯竭】的（永远返回任务），
//   所以只能靠 request_stop 停止 —— 这正好测到了停止机制本身。
// 【关键】必须同时起一个消费者：
//   否则队列（容量 4）瞬间填满，生产者被背压堵死，
//   测的就不是"停止机制"而是"背压"了。
TEST(ProducerTest, RequestStopInterruptsProducer) {
    Queue q(4);
    Worker_thread w(q, "w");
    w.start();

    Producer_thread::Generator gen = []() -> std::optional<Queue::value_type> {
        return [] { std::this_thread::sleep_for(1ms); };   // 永不枯竭
    };

    Producer_thread p(q, gen, "p2");
    p.start();
    std::this_thread::sleep_for(100ms);   // 让它跑一会儿

    p.request_stop();
    p.join();   // 最坏等待一个超时切片（50ms）

    const auto produced_at_stop = p.produced();
    EXPECT_GT(produced_at_stop, 0u);

    // 停止之后再等一会儿，确认计数不再增长
    std::this_thread::sleep_for(60ms);
    EXPECT_EQ(p.produced(), produced_at_stop);

    q.close();
    w.join();
}

// 测什么：背压 —— 无论跑多久，队列深度都【不会超过容量上限】。
// 怎么测：起一个采样线程，每 1ms 读一次 q.size()，记录历史最大值。
//         这是典型的"运行时不变量监控"：
//         不去断言某一瞬间的状态，而是断言"任意时刻都必须成立"的性质。
//
// 为什么这么设计：
//   · 生产者无限供给 + 消费者故意做慢（每个任务睡 2ms）
//     → 必然触发背压路径
//   · 容量只有 8，如果实现有 bug，很快就会被采样线程抓到
TEST(ProducerTest, BackpressureNeverExceedsCapacity) {
    constexpr std::size_t kCapacity = 8;
    Queue q(kCapacity);

    std::atomic<std::size_t> max_seen{0};
    std::atomic<bool> sampling{true};
    std::thread sampler([&] {
        while (sampling.load(std::memory_order_relaxed)) {
            max_seen.store(std::max(max_seen.load(std::memory_order_relaxed), q.size()),
                           std::memory_order_relaxed);
            std::this_thread::sleep_for(1ms);
        }
    });

    std::deque<Worker_thread> workers;
    for (int i = 0; i < 2; ++i) {
        workers.emplace_back(q, "w" + std::to_string(i));
        workers.back().start();
    }

    Producer_thread::Generator gen = []() -> std::optional<Queue::value_type> {
        return [] { std::this_thread::sleep_for(2ms); };   // 慢任务，逼出背压
    };

    Producer_thread p(q, gen, "p3");
    p.start();
    std::this_thread::sleep_for(300ms);   // 跑够久，让采样覆盖到各种时刻

    p.request_stop();
    p.join();
    q.close();
    for (auto& w : workers) w.join();

    sampling.store(false);
    sampler.join();

    EXPECT_LE(max_seen.load(), kCapacity);   // 队列深度绝不超过容量上限
}

// ============================================================================
//  Worker_thread 层
// ============================================================================

// 测什么：消费者能把队列里的任务全部执行完。
// 【注意】关闭流程：先 start，再 close()。
//   close 是让消费者退出的唯一方式（它唤醒 wait_pop 并给出"下班"信号）。
//   如果不 close，w.join() 会永久挂死。
TEST(WorkerTest, ExecutesAllQueuedTasks) {
    Queue q(0);
    std::atomic<int> counter{0};
    for (int i = 0; i < 10; ++i) {
        q.wait_push([&counter] { counter.fetch_add(1, std::memory_order_relaxed); });
    }

    Worker_thread w(q, "w");
    w.start();
    q.close();
    w.join();

    EXPECT_EQ(counter.load(), 10);
    EXPECT_EQ(w.executed(), 10u);
}

// 测什么：异常隔离 —— 一个任务抛异常不能杀死工作线程，
//        后面的任务必须照常执行。
//
// 为什么这是线程池的【核心健壮性】保证：
//   如果去掉 try/catch，第一个任务抛出的异常会传播出线程入口函数
//   → std::terminate() → 整个进程崩溃，后面 5 个任务根本没机会执行。
//   这个测试一旦失败，表现是【整个测试进程崩掉】，而不是某条 EXPECT 不过。
TEST(WorkerTest, SurvivesThrowingTask) {
    Queue q(0);
    std::atomic<int> ok{0};

    q.wait_push([] { throw std::runtime_error("boom"); });   // 坏任务
    for (int i = 0; i < 5; ++i) {
        q.wait_push([&ok] { ok.fetch_add(1, std::memory_order_relaxed); });
    }

    Worker_thread w(q, "w");
    w.start();
    q.close();
    w.join();

    EXPECT_EQ(ok.load(), 5);        // 坏任务之后的任务照常执行
    EXPECT_EQ(w.failed(), 1u);      // 失败被记录
    EXPECT_EQ(w.executed(), 5u);    // 只有成功的才计入 executed
}

// 测什么：空队列 + 已关闭 → 消费者应当【立即退出】而不是卡住。
// 为什么重要：这验证了 close() 的 notify_all 唤醒逻辑。
//            如果 close() 忘了唤醒等待在 wait_pop 上的线程，
//            这里会直接挂死（测试超时），是最有价值的"防挂死"测试之一。
TEST(WorkerTest, ExitsWhenQueueClosedAndEmpty) {
    Queue q(0);
    q.close();

    Worker_thread w(q, "w");
    w.start();
    w.join();   // 若唤醒逻辑有 bug，这一行会永远不返回

    EXPECT_FALSE(w.running());
    EXPECT_EQ(w.executed(), 0u);
}

// ============================================================================
//  集成测试
// ============================================================================

// 测什么：【最强的正确性约束】每个任务恰好被执行一次 —— 不丢、不重。
//
// 为什么这样设计：
//   · 光统计"总执行次数 == 2000"是不够的：
//     丢了任务 A 却把任务 B 执行了两遍，总数仍然对得上，但结果是错的。
//   · 所以给每个任务配一个【独立的计数器】counts[id]，
//     最后逐个断言 counts[i] == 1。这才真正锁死了"恰好一次"。
//
// 参数选择：
//   · 队列容量只有 16，而消费者有 4 个、生产者 3 个
//     → 队列会频繁被填满，强制走背压路径（覆盖到并发最难的一段逻辑）
//   · 2000 个任务足够多，能放大竞态出现的概率
TEST(IntegrationTest, EveryTaskExecutedExactlyOnce) {
    constexpr int kTotal = 2000;
    constexpr int kProducers = 3;
    constexpr int kWorkers = 4;

    Queue q(16);
    std::atomic<int> next_id{0};

    // 每个任务一个独立计数器。
    // 【注意】std::vector<std::atomic<int>> 默认构造后值是未初始化的，
    // 必须显式 store(0)，不能想当然以为它像 vector<int>(n) 那样会清零。
    std::vector<std::atomic<int>> counts(kTotal);
    for (auto& c : counts) c.store(0);

    std::deque<Worker_thread> workers;
    for (int i = 0; i < kWorkers; ++i) {
        workers.emplace_back(q, "w" + std::to_string(i));
        workers.back().start();
    }

    std::deque<Producer_thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        Producer_thread::Generator gen = [&]() -> std::optional<Queue::value_type> {
            const int id = next_id.fetch_add(1, std::memory_order_relaxed);
            if (id >= kTotal) return std::nullopt;
            return [id, &counts] { counts[id].fetch_add(1, std::memory_order_relaxed); };
        };
        producers.emplace_back(q, std::move(gen), "p" + std::to_string(p));
        producers.back().start();
    }

    // 标准关闭顺序：先停生产者 → 再关队列 → 最后收消费者
    for (auto& p : producers) p.join();
    q.close();
    for (auto& w : workers) w.join();

    // ---- 所有线程都 join 之后，才在主线程里做断言（铁律一 + 铁律二）----
    std::uint64_t produced = 0;
    for (const auto& p : producers) produced += p.produced();
    std::uint64_t executed = 0;
    for (const auto& w : workers) executed += w.executed();

    EXPECT_EQ(produced, kTotal);
    EXPECT_EQ(executed, kTotal);
    EXPECT_TRUE(q.empty());

    // 逐个校验"恰好一次"。
    // 用 ASSERT 而非 EXPECT：第一个不相等就停下来，
    // 否则 2000 个失败信息会把输出淹没。
    // << 后面的信息是 GTest 的"附加消息"，失败时会打印，方便定位是哪个 id。
    for (int i = 0; i < kTotal; ++i) {
        ASSERT_EQ(counts[i].load(), 1) << "任务 " << i << " 执行次数异常";
    }
}

// 测什么：RAII —— 不显式调用 request_stop/join，直接离开作用域，
//        析构函数必须保证线程被正确回收，而不是崩溃。
//
// 为什么重要：std::thread 在 joinable 状态下析构会调用 std::terminate()。
//            这是初学者最常见的崩溃原因。
//            这个测试的价值在于"它跑完没崩"就算通过（用 SUCCEED() 显式表达）。
TEST(IntegrationTest, DestructorJoinsThreadWithoutStop) {
    Queue q(0);
    {
        Producer_thread::Generator gen = []() -> std::optional<Queue::value_type> {
            return [] {};
        };
        Producer_thread p(q, gen, "p");
        p.start();
    }   // p 在此析构：自动 request_stop() + join()
    SUCCEED();
}
