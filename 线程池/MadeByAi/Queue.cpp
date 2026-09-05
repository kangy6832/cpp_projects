#include "Queue.hpp"

#include <utility>

// ============================================================================
//  Queue.cpp —— 有界阻塞队列的实现
// ============================================================================
//
//  【贯穿全文件的两条铁律】
//
//  铁律一：条件变量的 wait 必须带【谓词】（判断条件的 lambda）
//     错误写法：if (queue.empty()) cv.wait(lock);
//     正确写法：cv.wait(lock, []{ return !queue.empty(); });
//
//     原因是【虚假唤醒】（spurious wakeup）：
//     在 POSIX/Linux 上，即使没有任何线程 notify，wait 也可能返回
//     （内核信号、实现细节都可能导致）。不带谓词的 wait 返回后
//     如果不重新检查条件，就会在队列为空时去取元素 → 崩溃或未定义行为。
//     带谓词的 wait 等价于内部自动 while 循环重查，天然免疫这个问题。
//
//  铁律二：改完共享状态后，一定要记得唤醒等待方
//     这是并发代码最常见的 bug：逻辑全对，就是少了一句 notify，
//     表现为"程序偶尔卡住不动"，极难排查。
//     本文件里只有两处需要 notify，都用注释标出来了。
//
//  ==========================================================================

Queue::Queue(std::size_t capacity) : capacity_(capacity) {}

// ----------------------------------------------------------------------------
//  生产者侧：入队
// ----------------------------------------------------------------------------

PushResult Queue::wait_push(Task task) {
    std::unique_lock<std::mutex> lock(mtx_);

    // 等待"有空位"或"队列已关闭"。
    // 【注意】用的是 while 语义的谓词 wait，不是 if！
    not_full_.wait(lock, [this] { return closed_ || !full_locked(); });

    // 【踩坑】唤醒后必须【重新判断】是不是因为"关闭"才被唤醒的。
    // 关闭时的 notify_all 会把所有生产者叫醒，此时不能入队，
    // 必须返回 Closed，否则任务被塞进一个已经停止的系统里，永远不会被执行。
    if (closed_) return PushResult::Closed;

    tasks_.push_back(std::move(task));

    // 【设计要点】先解锁，再通知。
    //   若持锁 notify，被唤醒的线程会立刻试图抢锁、抢不到又阻塞，
    //   白白多一次上下文切换（这叫 "hurry up and wait"）。
    //   解锁后再 notify 是标准优化。
    //   安全性：即使此刻有新线程开始等待，它也会先拿锁检查谓词，
    //   看到队列非空就不会睡下，所以不会丢失唤醒。
    lock.unlock();
    not_empty_.notify_one();   // ← 唤醒一个空闲的消费者
    return PushResult::Ok;
}

PushResult Queue::wait_push_for(Task task, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mtx_);

    // 快路径：已关闭，直接失败返回（不必等待）
    if (closed_) return PushResult::Closed;

    // 只有真的满了才需要等待；不满就直接入队，避免无谓的超时开销
    if (full_locked()) {
        // wait_for 返回 bool：谓词为 true（条件满足）→ true；超时 → false
        if (!not_full_.wait_for(lock, timeout, [this] { return closed_ || !full_locked(); })) {
            // 超时返回：这【不是】错误，而是给调用方的"检查点"。
            // 调用方（Producer_thread）会在此刻回去看一眼停止标志。
            return PushResult::Timeout;
        }
        // 被唤醒了，但仍要判断是被"关闭"唤醒的还是被"有空位"唤醒的
        if (closed_) return PushResult::Closed;
    }

    tasks_.push_back(std::move(task));
    lock.unlock();
    not_empty_.notify_one();
    return PushResult::Ok;
}

// ----------------------------------------------------------------------------
//  消费者侧：出队
// ----------------------------------------------------------------------------

std::optional<Queue::Task> Queue::wait_pop() {
    std::unique_lock<std::mutex> lock(mtx_);

    // 等待"有任务"或"队列已关闭"
    not_empty_.wait(lock, [this] { return closed_ || !tasks_.empty(); });

    // 【设计要点·优雅关闭的关键】
    // 被唤醒后不能直接返回 nullopt，必须先看看是不是还有存量任务。
    // 因为 close() 的语义是"不再收新的，但存量要处理完"，
    // 所以这里要先把队列取空，取空之后的下一次 wait_pop 才返回 nullopt。
    // 如果这里写成 `if (closed_) return nullopt;`，
    // close() 时队列里剩下的任务就全丢了。
    if (tasks_.empty()) return std::nullopt;

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock.unlock();

    // 【踩坑·高发】这一句极易遗漏！
    // 取走一个任务 = 腾出一个空位，必须唤醒被背压阻塞的生产者。
    // 漏掉它，生产者会永久卡在 wait_push 上，整个系统静默停摆。
    not_full_.notify_one();
    return task;
}

std::optional<Queue::Task> Queue::try_pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    if (tasks_.empty()) return std::nullopt;

    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return task;
}

// ----------------------------------------------------------------------------
//  状态查询与关闭
// ----------------------------------------------------------------------------

void Queue::close() {
    {
        // 用局部作用域把锁的生命周期限制到最小：
        // 只保护 closed_ 这一行写入，下面 notify 时早已释放锁。
        std::lock_guard<std::mutex> lock(mtx_);
        closed_ = true;
    }

    // 【设计要点】这里必须用 notify_all 而不是 notify_one：
    //   · 关闭是一个【全局事件】，要通知所有等待中的线程
    //   · 如果只 notify_one，只叫醒一个线程，其余的会永远睡下去
    //   · 生产者和消费者可能同时在等，所以两个条件变量都要通知
    not_full_.notify_all();    // 叫醒所有被背压堵住的生产者 → 它们会看到 Closed 而退出
    not_empty_.notify_all();   // 叫醒所有空闲的消费者 → 它们排空队列后会看到 nullopt 而退出
}

bool Queue::closed() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return closed_;
}

bool Queue::empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return tasks_.empty();
}

std::size_t Queue::size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return tasks_.size();
}

// capacity_ 是不可变的（构造后不再修改），所以无需加锁
std::size_t Queue::capacity() const { return capacity_; }
