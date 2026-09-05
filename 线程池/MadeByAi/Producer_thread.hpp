#pragma once

// ============================================================================
//  Producer_thread.hpp —— 生产者线程
// ============================================================================
//
//  【一、生产者在框架中的角色】
//
//      数据源（文件/网络/数据库）
//            │
//            ▼
//      Producer_thread  ──wait_push_for──>  Queue  ──wait_pop──>  Worker_thread
//       (本文件的类)
//
//  它做三件事：
//      1. 从"数据源"取出一条数据
//      2. 把它包装成一个任务（Task）
//      3. 投递到 Queue
//
//  【二、最关键的设计：用 Generator 把"数据源"抽象出去】
//
//       using Generator = std::function<std::optional<Task>()>;
//
//  生产者自己【完全不知道】任务从哪来 —— 是读文件、收网络包、
//  还是遍历内存数组，全由外部传进来的这个函数对象决定。
//  这是【依赖倒置 / 策略模式】：
//      · 好处一：生产者类可以复用在任何场景，不掺业务逻辑
//      · 好处二：写单元测试时，传一个生成假数据的 lambda 就行了，
//               不需要真的去读文件或连网络（可测试性）
//  约定：Generator 返回 nullopt 表示"数据源耗尽"，生产者到此自然结束。
//
//  【三、为什么生产者线程里不直接"干活"】
//  如果生产者边取数据边处理，那就退化成单线程了。
//  生产者只负责"搬运 + 投递"，重活交给 Worker_thread 并行做，
//  这样 N 个消费者才能真正跑满多核。
//
//  ==========================================================================

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "Queue.hpp"

class Producer_thread {
public:
    using Task = Queue::value_type;  // 任务

    // 数据源：每调用一次产出一个任务；返回 nullopt 表示数据已耗尽
    // std::function< > 可调用对象容器，可以存 lambda、函数指针、仿函数等
    // std::optional<Task> ：函数类型语法 ，返回类型(参数类型1, 参数类型2, ...)
    using Generator = std::function<std::optional<Task>()>;  // 任务分发函数

    // 【作用】创建对象：把外部传入的队列、数据源生成函数、线程名绑定到成员变量上。
    // 【注意】构造函数【不会】启动线程 —— 必须另外调用 start() 才会开工。
    // 【设计要点】Queue 以【引用】传入，表示"我不拥有它，只是借用"。
    // 调用方必须保证 Queue 的生命周期长于 Producer_thread，
    // 否则会出现悬垂引用。这是本类唯一的生命周期约束。
    Producer_thread(Queue& queue, Generator gen, std::string name = "producer");

    // 【作用】对象销毁时自动回收线程：先请求停止，再等待线程真正结束。
    // 【设计要点·RAII】析构函数自动 request_stop() + join()。
    // 为什么必须这样？见 .cpp 中析构函数的注释。
    ~Producer_thread();

    // 【作用】禁止拷贝本类对象（两行分别禁止"拷贝构造"和"拷贝赋值"）。
    // 【踩坑】持有 std::thread 的类必须禁拷贝：
    // std::thread 本身不可拷贝（只能移动），两个对象持有同一个线程
    // 会导致 double join，后果是未定义行为。
    Producer_thread(const Producer_thread&) = delete;             // 拷贝构造
    Producer_thread& operator=(const Producer_thread&) = delete;   // 拷贝赋值运算符



    
    // 【作用】启动生产者线程：内部创建 std::thread 去执行私有的 run() 主循环，
    //        之后生产者就开始不停取数据、投递任务。
    // 【特性】非阻塞：创建完线程立即返回，不会等线程跑完。
    // 【特性】幂等：重复调用无效（内部用 CAS 保证只启动一次）。
    // 【配套】request_stop() 请求停止，join() 等待真正结束。
    void start();

    // 【作用】请求停止：把原子标志置为 true，让生产者线程在下一个检查点自行退出。
    // 【特性】非阻塞：只是置标志，立即返回，不等待线程真的停下。
    // 【设计要点】为什么叫 request_stop 而不叫 stop？
    // 因为它是【异步】的：只是置一个标志，立刻返回，
    // 线程会在下一个检查点自行退出。真正的等待由 join() 完成。
    // 这个命名区分了"请求停止"和"等待停止"，是并发代码的常见约定。
    void request_stop() noexcept;

    // 【作用】阻塞当前线程，直到生产者线程【真正】结束为止。
    // 【典型用法】① request_stop() 之后再 join()；② 数据源会自然耗尽时，直接 join() 等它跑完。
    // 【安全】内部已判断 joinable 并防止"线程自己 join 自己"，重复调用无副作用。
    void join();

    // ---------------- 运行时观测 ----------------
    // 【亮点】这四个查询都是无锁的（atomic load），可以在任何时刻
    // 从别的线程安全调用，用于监控生产速率、判断线程是否存活。
    // 作用：查询生产者线程当前是否还在运行
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    // 作用：已【成功入队】的任务总数（只有入队成功才计数，用于校验数据有无丢失）
    std::uint64_t produced() const noexcept { return produced_.load(std::memory_order_relaxed); }
    // 作用：数据源抛异常被跳过的次数（用于发现数据源故障）
    std::uint64_t failed() const noexcept { return failed_.load(std::memory_order_relaxed); }
    // 作用：返回构造时指定的线程名（用于日志标识）
    const std::string& name() const noexcept { return name_; }

private:
    // 【作用】生产者线程的【主循环】（私有，由 start() 创建的线程执行，外部不可直接调用）。
    //        循环三步：① 调用 gen_() 取一个任务 ② wait_push_for 投递到队列 ③ 计数。
    // 【退出条件】停止标志被置位 / 数据源返回 nullopt / 队列被关闭。
    void run();

    Queue& queue_;
    Generator gen_;
    std::string name_;

    // 【设计要点】为什么这四个状态用 std::atomic 而不是普通变量？
    //   stop_    ：由【外部线程】写、生产者线程读 → 跨线程通信，
    //              普通 bool 会构成数据竞争（未定义行为）
    //   running_ ：由生产者线程写、外部读，同样跨线程
    //   produced_/failed_：生产者线程写、外部读
    // 用 atomic 后无需加锁，读写都是原子的，且自带内存序保证。
    //
    // 【内存序的选择（亮点）】
    //   · stop_ 用 release/acquire：stop_ 是个"开关"，
    //     释放-获取语义能保证开关前后的内存操作不会被重排乱序。
    //   · 计数器用 relaxed：它只是个统计数字，不用来同步其他数据，
    //     relaxed 最轻量（在 x86 上就是一条普通 mov）。
    //   不加区分地全用默认的 seq_cst（顺序一致）会白付性能代价。
    std::thread thread_;   // 声明 生产者线程对象
    std::atomic<bool> stop_{false};  // 生产者线程的停止标志，false = 继续运行，true = 请求停止
    std::atomic<bool> running_{false};  // 生产者线程的运行状态，false = 未运行，true = 正在运行
    std::atomic<std::uint64_t> produced_{0};  // 生产者线程已成功入队的任务总数
    std::atomic<std::uint64_t> failed_{0};  // 生产者线程在取数据源时抛异常被跳过的次数
};
