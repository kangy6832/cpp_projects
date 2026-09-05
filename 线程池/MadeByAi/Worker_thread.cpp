#include "Worker_thread.hpp"

#include <optional>
#include <utility>

// ============================================================================
//  Worker_thread.cpp —— 工作线程（消费者）的实现
// ============================================================================
//
//  【本文件有两个"生死攸关"的设计点】
//
//  1. 任务必须在【锁外】执行
//     如果拿着队列的锁去跑任务，同一时刻就只有一个消费者能干活，
//     线程池直接退化成单线程 —— 而这是完全看不出错误、
//     只会表现为"性能很差"的隐蔽 bug。
//
//  2. 任务抛出的异常必须在本文件内被拦住
//     异常一旦从线程主函数逃逸，就是 std::terminate()，进程崩溃。
//     线程池必须保证"一个坏任务不会拖垮整个池子"。
//
//  ==========================================================================

Worker_thread::Worker_thread(Queue& queue, std::string name)
    : queue_(queue), name_(std::move(name)) {}

Worker_thread::~Worker_thread() {
    request_stop();
    join();
}

void Worker_thread::start() {
    // 与 Producer_thread 相同：用 CAS 保证只能启动一次，避免重复 start
    // 覆盖 thread_ 导致旧线程无法 join（进而析构时 terminate）
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }
    stop_.store(false, std::memory_order_release);

    thread_ = std::thread([this] {
        run();
        running_.store(false, std::memory_order_release);
    });
}

void Worker_thread::request_stop() noexcept {
    stop_.store(true, std::memory_order_release);

    // 【关键设计】只置标志是不够的！
    // 消费者此刻很可能正阻塞在 queue_.wait_pop() 里，
    // 而 wait_pop 只在两种情况下返回：① 有新任务 ② 队列被关闭。
    // 如果不关闭队列，没有任何新任务到来时，它会一直睡到天荒地老，
    // request_stop() 就完全失效了。
    //
    // 所以这里必须 close()：唤醒它 → 它把存量任务取完 →
    // 下一次 wait_pop 返回 nullopt → 循环结束 → 线程退出。
    //
    // close() 是幂等的（重复调用无害），所以析构函数里再调一次也没问题。
    queue_.close();
}

void Worker_thread::join() {
    // 同 Producer_thread：先判 joinable，再防自 join
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

// ----------------------------------------------------------------------------
//  消费者主循环
// ----------------------------------------------------------------------------
void Worker_thread::run() {
    while (!stop_.load(std::memory_order_acquire)) {

        // ---- 第 1 步：取任务（队列空时阻塞在此，不占 CPU）----
        // 返回 nullopt 的唯一含义：队列已关闭【且】已排空 → 可以下班了
        std::optional<Queue::Task> task = queue_.wait_pop();
        if (!task) {
            break;
        }

        // ---- 第 2 步：执行任务 ----
        //
        // 【亮点·要点一】此时【不持有】队列的锁（锁在 wait_pop 返回时就释放了）。
        // 于是多个消费者可以真正并发执行任务，这才是线程池的意义所在。
        // 反例：如果这里持锁，多线程只是"看起来在跑"，实际全程串行。
        //
        // 【亮点·要点二】try/catch(...) 是必须的防线。
        // std::function 里包着的是用户代码，它可能抛任何异常。
        // 若不拦住，异常会一路传播出线程的入口函数，
        // C++ 标准规定此时调用 std::terminate() → 整个进程崩溃，
        // 其他所有正常任务全部陪葬。
        // 拦住之后，一个任务失败只影响它自己，线程池继续服务后续任务。
        try {
            (*task)();
            executed_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
            // 更完善的做法：把异常存进 std::exception_ptr，
            // 通过 future 交还给提交者（这是 std::packaged_task 的做法）。
            // 本项目只做计数，保证池子存活即可。
            failed_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}
