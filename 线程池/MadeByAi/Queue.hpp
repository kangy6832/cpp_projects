#pragma once

// ============================================================================
//  Queue.hpp —— 有界阻塞任务队列（线程池的心脏）
// ============================================================================
//
//  【一、它在整个框架中的位置】
//
//      Producer_thread ──wait_push──> ┌─────────┐ ──wait_pop──> Worker_thread
//           生产者线程                │  Queue  │                消费者线程
//      Producer_thread ──wait_push──> │ 任务缓冲 │ ──wait_pop──> Worker_thread
//                                     └─────────┘
//
//  Queue 是线程池里唯一被多个线程共享的数据，也是生产者与消费者之间
//  【唯一】的耦合点：
//    · 生产者不认识消费者，消费者也不认识生产者
//    · 双方只依赖 Queue 提供的接口
//  这就是"解耦"的价值：将来想把消费者换成"带优先级的调度器"
//  或"工作窃取式调度器"，只要 Queue 的接口不变，生产者一行都不用改。
//
//  【二、为什么需要"缓冲队列"这个中间层】
//   1. 削峰填谷：生产速度短期超过消费速度时，任务先堆在队列里，
//      消费者按自己的节奏消化，而不是直接拒绝生产者。
//   2. 速度解耦：生产者不必等消费者处理完，消费者不必等生产者生成完。
//   3. 批量化：消费者可以攒一批再处理，摊薄线程切换的开销。
//
//  【三、为什么是"有界"队列 —— 背压（Backpressure）】
//  这是本文件最重要的设计决策。
//    · 无界队列 + 快生产者 = 任务无限堆积 → 内存耗尽（OOM）
//    · 有界队列：队列满时【阻塞生产者】，让"忙不过来"这个信号
//      反向传导回去，形成负反馈，系统自动达到平衡。这叫背压。
//
//  ⚠️ 背压的代价：会引入"背压死锁"。
//     例如：起了一个生产者往容量 8 的队列里灌任务，却一个消费者都没起，
//     队列填满后生产者将永远阻塞 —— join() 直接挂死。
//     规避方法见 Producer_thread.cpp 注释，以及 tests/test_threadpool.cpp
//     中 ProducerTest.ProducesAllTasksThenExits 里的说明。
//
//  【四、类设计上的几个"为什么"】
//   · 用 std::deque 而非 std::queue：std::queue 只是 deque 的适配器，
//     多一层封装没有收益；deque 头尾插入删除都是 O(1) 且不会像 vector
//     那样在扩容时搬运元素。
//   · 用 std::function<void()> 表示任务：这叫【类型擦除】。
//     不管外面传进来的是 lambda、函数指针还是仿函数，统一变成
//     同一种类型的"无参无返回"可调用对象，队列才能存它们。
//     代价是一次间接调用（约几纳秒），换来的是极大的通用性。
//   · 删除拷贝构造/赋值：std::mutex 和 std::condition_variable
//     都不可拷贝，含它们的类天生就该禁止拷贝。
//
//  ==========================================================================

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>

// push 的三种结果。
// 为什么不用 bool？因为"失败"必须区分原因：
//   Closed  → 队列已关闭，是永久性失败，调用方应该结束循环
//   Timeout → 只是暂时没空位，调用方应该去检查停止标志，然后重试
// 用 bool 表达不了这个区别，调用方就只能瞎猜。
enum class PushResult { Ok, Closed, Timeout };

// 有界阻塞任务队列，生产者与消费者之间唯一的耦合点。
//
// 语义约定（三条，是整个系统正确性的基础）：
//   1. capacity == 0 表示无界（不做背压），用于测试或内存充足的场景
//   2. close() 后不再接收新任务（push 返回 Closed），但已入队的任务
//      仍可被 pop 取走，直到排空后 wait_pop 才返回 nullopt
//      —— 这就是【优雅关闭】（graceful shutdown），保证不丢任务
//   3. 队列满时 push 阻塞、队列空时 pop 阻塞，两端都靠条件变量唤醒
class Queue {
public:
    using Task = std::function<void()>;
    using value_type = Task;   // 提供 value_type，让使用方可以写出与容器泛型兼容的代码

    // 【作用】创建队列并指定容量上限。
    // 【参数】capacity == 0 表示无界（不做背压）；> 0 时队列满会阻塞生产者。
    explicit Queue(std::size_t capacity = 0);

    // 【作用】禁止拷贝本队列（两行分别禁止"拷贝构造"和"拷贝赋值"）。
    // 【踩坑】含 mutex / condition_variable 的类必须禁拷贝，
    // 否则默认的拷贝构造会试图拷贝一把锁，语义完全错误，且编译通常直接失败。
    Queue(const Queue&) = delete;             // 拷贝构造
    Queue& operator=(const Queue&) = delete;  // 拷贝赋值运算符

    // ---------------- 生产者侧 ----------------

    // 【作用】把一个任务放进队尾。
    // 【阻塞条件】队列满则一直阻塞，直到有空位或队列被关闭。
    // 【返回】Ok 入队成功 / Closed 队列已关闭（任务被拒绝）。
    PushResult wait_push(Task task);

    // 【作用】同上，但最多只等 timeout 毫秒。
    // 【返回】多一种 Timeout：暂时没空位，调用方应去检查停止标志后重试。
    // 队列满则最多等待 timeout。
    // 【设计要点】为什么必须有这个"限时"版本？
    //   因为生产者需要定期检查自己的停止标志。
    //   如果只有无限等待的 wait_push，生产者一旦被背压阻塞，
    //   就再也没机会看到 stop_ 标志，request_stop() 会彻底失效。
    //   限时等待让"阻塞"变成"可中断的阻塞"，这是优雅停止的前提。
    PushResult wait_push_for(Task task, std::chrono::milliseconds timeout);

    // ---------------- 消费者侧 ----------------

    // 【作用】从队头取出一个任务并把它从队列移除。
    // 【阻塞条件】队列空则阻塞，直到有新任务或队列被关闭。
    // 【返回】取到任务则返回它；队列【已关闭且排空】时返回 nullopt（表示"可以下班了"）。
    std::optional<Queue::Task> wait_pop();

    // 【作用】同 wait_pop，但【不阻塞】：队列空立即返回 nullopt。
    // 【用途】单元测试、以及需要在无任务时做其他事的场景。
    std::optional<Queue::Task> try_pop();

    // ---------------- 状态查询 ----------------
    // 【设计要点】下面这些查询函数都加锁（capacity 除外，它构造后不再变）。
    // 有人会觉得"读个 bool 何必加锁"，但：
    //   · 不加锁会构成数据竞争（data race），在 C++ 里是【未定义行为】，
    //     TSan 会直接报错，编译器优化也可能让读到过期的值
    //   · 所以对 mtx_ 加了 mutable，才能在 const 成员函数里加锁
    // 【作用】优雅关闭：不再接收新任务，唤醒所有等待中的生产者与消费者。
    //        已入队的存量任务仍可被取走，取空后 wait_pop 才返回 nullopt。
    void close();
    // 作用：查询队列是否已关闭
    bool closed() const;
    // 作用：查询队列当前是否为空
    bool empty() const;
    // 作用：查询队列中当前积压的任务个数
    std::size_t size() const;
    // 作用：返回构造时指定的容量上限（0 表示无界）
    std::size_t capacity() const;

private:
    // 判断队列是否已满。
    // 【命名约定】后缀 _locked 表示"调用方必须已持有 mtx_"，
    // 这是并发代码里非常有用的注释即文档的做法，能避免误用。
    // capacity_ == 0 时恒为 false，即"永远不满"= 无界。
    bool full_locked() const { return capacity_ > 0 && tasks_.size() >= capacity_; }

    // ---------------- 成员变量 ----------------

    mutable std::mutex mtx_;

    // 【设计要点·核心】为什么需要【两个】条件变量？
    //   初学者的直觉是"一个条件变量够了"，但那会导致唤醒错位：
    //     · 消费者取走任务后，要唤醒的是【被背压阻塞的生产者】
    //     · 生产者放入任务后，要唤醒的是【空闲等待的消费者】
    //   如果共用一个条件变量 + notify_one()，很可能把"生产者"唤醒了
    //   —— 而它等的条件（有空位）此刻根本不满足，于是又睡回去，
    //   真正该被唤醒的消费者却没被叫醒 → 任务滞留、吞吐骤降，甚至卡死。
    //   分成 not_full_ / not_empty_ 后，谁等什么条件、唤醒谁，
    //   语义清晰，绝不会唤醒错对象。
    std::condition_variable not_full_;    // 等待条件：队列有空位（生产者等它）
    std::condition_variable not_empty_;   // 等待条件：队列有任务（消费者等它）

    std::deque<Task> tasks_;
    std::size_t capacity_;
    bool closed_ = false;   // 受 mtx_ 保护，只在持锁时读写
};
