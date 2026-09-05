// 线程池单元测试
//
// 运行：
//   cmake -B build -DBUILD_TESTING=ON && cmake --build build && ./build/bin/threadpool_tests
// 或直接：
//   g++ -std=c++17 -pthread *.cpp tests/test_threadpool.cpp -lgtest -lgtest_main -o tests_bin
//
// 并发测试的三条铁律：
//   1. 断言只在主线程做 —— GTest 的 EXPECT/ASSERT 不是线程安全的，子线程里只写 atomic
//   2. 先 join 再断言 —— 否则读到的是中间状态，测试会随机失败
//   3. 每个测试都要有超时上限 —— 并发 bug 表现为死锁，不能让 CI 无限挂起

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

// ==================================================================
// Queue
// ==================================================================

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

// 有界性：队列满时 push 必须超时失败，而不是无限增长
TEST(QueueTest, BoundedCapacityBlocksPush) {
    Queue q(2);
    EXPECT_EQ(q.wait_push([] {}), PushResult::Ok);
    EXPECT_EQ(q.wait_push([] {}), PushResult::Ok);
    EXPECT_EQ(q.size(), 2u);

    EXPECT_EQ(q.wait_push_for([] {}, 50ms), PushResult::Timeout);

    // 腾出空位后应能立即入队
    ASSERT_TRUE(q.try_pop().has_value());
    EXPECT_EQ(q.wait_push_for([] {}, 50ms), PushResult::Ok);
}

// 优雅关闭：close 后存量任务仍可被取走，取完才返回 nullopt
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

// ==================================================================
// Producer_thread
// ==================================================================

// 注意：这里用无界队列（capacity=0）。若用有界队列且没有消费者，
// 生产者会被背压永久阻塞 —— 那是背压的正确行为，但会让测试挂死。
TEST(ProducerTest, ProducesAllTasksThenExits) {
    Queue q(0);
    std::atomic<int> next{0};

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

    EXPECT_EQ(p.failed(), 5u);    // 异常被吞掉并计数，线程没死
    EXPECT_EQ(p.produced(), 4u);
}

// request_stop 后生产者必须及时退出，且不再投递
TEST(ProducerTest, RequestStopInterruptsProducer) {
    Queue q(4);
    std::atomic<int> consumed{0};
    Worker_thread w(q, "w");                       // 需要消费者，否则背压会堵死生产者
    w.start();

    // 永不枯竭的数据源
    Producer_thread::Generator gen = []() -> std::optional<Queue::value_type> {
        return [] { std::this_thread::sleep_for(1ms); };
    };

    Producer_thread p(q, gen, "p2");
    p.start();
    std::this_thread::sleep_for(100ms);

    p.request_stop();
    p.join();                                       // 最坏等待一个超时切片（50ms）

    const auto produced_at_stop = p.produced();
    EXPECT_GT(produced_at_stop, 0u);
    std::this_thread::sleep_for(60ms);
    EXPECT_EQ(p.produced(), produced_at_stop);      // 停止后不再增长

    q.close();
    w.join();
}

// 背压是本项目最核心的保证，必须单独验证
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
        return [] { std::this_thread::sleep_for(2ms); };   // 慢消费者，必然触发背压
    };

    Producer_thread p(q, gen, "p3");
    p.start();
    std::this_thread::sleep_for(300ms);

    p.request_stop();
    p.join();
    q.close();
    for (auto& w : workers) w.join();

    sampling.store(false);
    sampler.join();

    EXPECT_LE(max_seen.load(), kCapacity);   // 队列深度绝不超过容量上限
}

// ==================================================================
// Worker_thread
// ==================================================================

TEST(WorkerTest, ExecutesAllQueuedTasks) {
    Queue q(0);
    std::atomic<int> counter{0};
    for (int i = 0; i < 10; ++i) {
        q.wait_push([&counter] { counter.fetch_add(1, std::memory_order_relaxed); });
    }

    Worker_thread w(q, "w");
    w.start();
    q.close();          // 关闭队列，让 worker 排空后退出
    w.join();

    EXPECT_EQ(counter.load(), 10);
    EXPECT_EQ(w.executed(), 10u);
}

// 异常隔离：一个任务抛异常不能杀死工作线程
TEST(WorkerTest, SurvivesThrowingTask) {
    Queue q(0);
    std::atomic<int> ok{0};

    q.wait_push([] { throw std::runtime_error("boom"); });
    for (int i = 0; i < 5; ++i) {
        q.wait_push([&ok] { ok.fetch_add(1, std::memory_order_relaxed); });
    }

    Worker_thread w(q, "w");
    w.start();
    q.close();
    w.join();

    EXPECT_EQ(ok.load(), 5);        // 抛异常之后的任务照常执行
    EXPECT_EQ(w.failed(), 1u);
    EXPECT_EQ(w.executed(), 5u);
}

TEST(WorkerTest, ExitsWhenQueueClosedAndEmpty) {
    Queue q(0);
    q.close();

    Worker_thread w(q, "w");
    w.start();
    w.join();                        // 若 wait_pop 唤醒逻辑有 bug，这里会挂死

    EXPECT_FALSE(w.running());
    EXPECT_EQ(w.executed(), 0u);
}

// ==================================================================
// 集成：多生产者 + 多消费者
// ==================================================================

// 最强约束：每个任务恰好被执行一次（不丢、不重）
TEST(IntegrationTest, EveryTaskExecutedExactlyOnce) {
    constexpr int kTotal = 2000;
    constexpr int kProducers = 3;
    constexpr int kWorkers = 4;

    Queue q(16);                                  // 小容量，强制走到背压路径
    std::atomic<int> next_id{0};
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

    // 关闭顺序不可颠倒
    for (auto& p : producers) p.join();
    q.close();
    for (auto& w : workers) w.join();

    std::uint64_t produced = 0;
    for (const auto& p : producers) produced += p.produced();
    std::uint64_t executed = 0;
    for (const auto& w : workers) executed += w.executed();

    EXPECT_EQ(produced, kTotal);
    EXPECT_EQ(executed, kTotal);
    EXPECT_TRUE(q.empty());

    for (int i = 0; i < kTotal; ++i) {
        ASSERT_EQ(counts[i].load(), 1) << "任务 " << i << " 执行次数异常";
    }
}

//  destructor 必须保证线程被 join，否则 std::terminate
TEST(IntegrationTest, DestructorJoinsThreadWithoutStop) {
    Queue q(0);
    {
        Producer_thread::Generator gen = []() -> std::optional<Queue::value_type> {
            return [] {};
        };
        Producer_thread p(q, gen, "p");
        p.start();
    }   // 析构时自动 request_stop + join，不应崩溃
    SUCCEED();
}
