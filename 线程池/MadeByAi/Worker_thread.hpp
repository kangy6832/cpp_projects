#pragma once

// ============================================================================
//  Worker_thread.hpp —— 工作线程（消费者）
// ============================================================================
//
//  【一、在框架中的位置】
//
//      Producer_thread ──>  Queue  ──wait_pop──> ┌────────────────┐
//                                                │ Worker_thread  │ ← 本类
//      Producer_thread ──>  Queue  ──wait_pop──> │  （执行任务）   │
//                                                └────────────────┘
//
//  多个 Worker_thread 共享同一个 Queue，谁抢到任务谁执行 ——
//  这就是线程池最朴素的【工作共享】（work sharing）模型。
//
//  【二、它和 Producer_thread 的结构几乎对称】
//  两者都有 start / request_stop / join / running / 计数器 / RAII 析构，
//  但停止机制不同，原因值得对比着看：
//
//    · 生产者：需要"定期检查停止标志"，所以阻塞的 push 用【限时等待】
//    · 消费者：停止时必须【关闭队列】才能把它从 wait_pop 里唤醒
//
//  为什么消费者不能用生产者那套"超时切片"？
//  因为消费者的退出条件是"队列已关闭且排空"，
//  这个信号本来就由 close() 传递，用 notify_all 唤醒是最直接的表达；
//  而让每个消费者每 50ms 醒一次去查标志，纯属浪费。
//
//  【三、Worker_thread 只做调度，不做计算】
//  它并不知道任务具体在干什么 —— 任务内容被 std::function 类型擦除了。
//  这种"框架与业务分离"的设计，让线程池可以执行任何可调用对象。
//
//  ==========================================================================

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "Queue.hpp"

class Worker_thread {
public:
    // 【作用】创建对象：绑定要消费的队列（引用共享）和线程名（默认 "worker"）。
    // 【注意】构造函数【不会】启动线程 —— 必须另外调用 start() 才会开工。
    explicit Worker_thread(Queue& queue, std::string name = "worker");

    // 【作用】对象销毁时自动停止并回收线程（内部 request_stop() + join()）。
    // 【设计要点·RAII】理由同 Producer_thread：避免 std::thread 未 join 就析构导致 terminate。
    ~Worker_thread();

    // 【作用】禁止拷贝本类对象（两行分别禁止"拷贝构造"和"拷贝赋值"）。
    Worker_thread(const Worker_thread&) = delete;             // 拷贝构造
    Worker_thread& operator=(const Worker_thread&) = delete;  // 拷贝赋值运算符

    // 【作用】启动工作线程：创建 std::thread 执行私有的 run() 主循环，开始取任务执行。
    // 【特性】非阻塞：创建完线程立即返回；幂等：重复调用无效（CAS 保证只启动一次）。
    void start();

    // 【作用】请求停止：置停止标志 + 【关闭队列】，把阻塞在 wait_pop 上的自己唤醒。
    // 【特性】非阻塞：立即返回，不等待线程真的停下。
    // 【注意·副作用】本函数会调用 Queue::close()。
    // 由于 close() 是队列级别的全局操作，共享同一个 Queue 的
    // 【所有】生产者和其他消费者都会因此停止。
    // 这不是 bug —— 停止一个消费者通常就意味着整个流水线要停了。
    // 如果需要单独停止某个消费者，应该给它配独立的队列，
    // 或在 Queue 里增加"取消单个等待者"的能力。
    void request_stop() noexcept;

    // 【作用】阻塞当前线程，直到工作线程【真正】结束为止。
    // 【前提】在此之前必须已经 close() 过队列（或调用了 request_stop），
    //        否则工作线程会一直阻塞在 wait_pop 上，join() 将永远等不到返回。
    void join();

    // ---------------- 运行时观测 ----------------
    // 【亮点】这四个查询都是无锁的（atomic load），可在任意线程安全调用。
    // 作用：查询工作线程当前是否还在运行
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    // 作用：已【成功】执行完的任务数（抛异常的不计入）
    std::uint64_t executed() const noexcept { return executed_.load(std::memory_order_relaxed); }
    // 作用：执行时抛出异常的任务数（用于发现业务代码的故障）
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_relaxed); }
    // 作用：返回构造时指定的线程名（用于日志标识）
    const std::string& name() const noexcept { return name_; }

private:
    // 【作用】工作线程的【主循环】（私有，由 start() 创建的线程执行，外部不可直接调用）。
    //        循环两步：① wait_pop() 取一个任务 ② 在【锁外】执行该任务并计数。
    // 【退出条件】停止标志被置位 / 队列已关闭且排空（wait_pop 返回 nullopt）。
    void run();

    Queue& queue_;
    std::string name_;

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> executed_{0};   // 成功执行的任务数
    std::atomic<std::uint64_t> failed_{0};     // 抛异常的任务数
};
