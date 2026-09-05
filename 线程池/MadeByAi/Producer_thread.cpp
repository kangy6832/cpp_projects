#include "Producer_thread.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

// ============================================================================
//  Producer_thread.cpp —— 生产者线程的实现
// ============================================================================
//
//  【本文件最难的一个问题：如何让一个"阻塞中"的线程优雅退出？】
//
//  生产者会阻塞在 queue_.wait_push() 上（队列满，背压）。
//  此时外部调用 request_stop()，但线程正卡在系统调用里，
//  根本没机会去读 stop_ 标志 —— 这就僵住了。
//
//  业界有三种解法，本项目选了第二种：
//
//   解法一：让队列支持"取消等待"
//       queue_.close() 会 notify_all 唤醒所有等待者。
//       代价：close() 是全局的，会影响共享同一队列的所有线程。
//       （Worker_thread 用的就是这种，因为它正好需要队列关闭才退出）
//
//   解法二：【超时切片】（本项目采用）
//       把"无限等待"拆成"每次最多等 50ms"，超时后回到循环顶部
//       重新检查 stop_，没停就再等一轮。
//       代价：停止响应最多有 50ms 延迟。
//       优点：不影响其他线程，实现最简单，也最容易推理正确性。
//
//   解法三：专门的 notify 接口
//       Queue 提供 notify_producers()，只唤醒生产者，不影响消费者。
//       零延迟且精确，代价是 Queue 的 API 多一个方法。
//
//  这是一个典型的【工程权衡】：没有完美方案，只有最适合当前约束的方案。
//
//  ==========================================================================

// 单次等待的切片时长。
// 【权衡】调小 → 停止响应快，但空转唤醒频繁（CPU 占用略升）；
//         调大 → 停止响应慢。50ms 是个实践中比较平衡的值。
// 若将来需要更快的停止响应，可改用上面说的解法三。
namespace {
constexpr auto kPushSlice = 50ms;
}

Producer_thread::Producer_thread(Queue& queue, Generator gen, std::string name)
    : queue_(queue), gen_(std::move(gen)), name_(std::move(name)) {}

Producer_thread::~Producer_thread() {
    // 【踩坑·必记】析构函数里必须 join！
    //
    // C++ 标准规定：std::thread 析构时如果还处于 joinable 状态
    // （即线程还在跑且没被 join 或 detach），会直接调用 std::terminate()，
    // 【整个进程立即崩溃】——不是抛异常，来不及 catch，程序直接死。
    //
    // 这是初学者写线程类最常踩的坑。用 RAII 在析构里保证 join，
    // 调用方哪怕忘了关线程也不会崩。
    request_stop();
    join();
}

void Producer_thread::start() {
    // 【亮点】用 CAS（compare_exchange）做"只允许启动一次"的保护。
    // 为什么不能简单写 if (running_) return; ？
    //   因为这是【检查-再执行】（check-then-act）模式，
    //   两个线程同时调用 start() 时，可能都通过检查 → 启动两个线程
    //   → thread_ 被覆盖赋值，前一个线程永远无法 join → 析构时崩溃。
    // compare_exchange_strong 是原子的"比较并交换"，
    // 保证只有一个线程能把 false 改成 true，另一个直接返回。
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;   // 已经在运行了，忽略重复调用
    }
    stop_.store(false, std::memory_order_release);   // 允许被重新启动

    // 【设计要点】为什么用 lambda 包一层 run()，而不是直接传 &Producer_thread::run？
    //   因为要在 run() 结束后把 running_ 置回 false。
    //   如果直接传成员函数，线程结束时没人更新 running_，
    //   外部看到的 running() 就永远是 true 了。
    thread_ = std::thread([this] {
        run();
        running_.store(false, std::memory_order_release);
    });

    // 【踩坑】lambda 捕获了 this 指针，因此必须保证：
    // 对象在线程结束前不能被销毁。
    // 本类靠析构函数里的 join() 来保证这一点。
}

void Producer_thread::request_stop() noexcept {
    stop_.store(true, std::memory_order_release);  // 把stop_置为 true

    // 注意：这里【没有】去唤醒阻塞中的 push。
    // 生产者会在最多 kPushSlice（50ms）内因超时返回并看到本标志。
    // 若将来 Queue 增加 notify_producers()，在这里加一行调用即可实现零延迟唤醒。
}

void Producer_thread::join() {
    // 【踩坑】必须先判断 joinable()。
    //   · 对未启动的线程（thread_ 是默认构造的）调 join 会抛 system_error
    //   · 对已经 join 过的线程再 join，同样抛异常
    //
    // 【踩坑】还要防止"线程自己 join 自己"：
    //   如果这段代码恰好在被管理的那个线程里执行（比如析构函数被该线程调用），
    //   join 会等待自己 → 永久死锁。
    //   用 get_id() 与当前线程 id 比较来规避。
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

// ----------------------------------------------------------------------------
//  生产者主循环
// ----------------------------------------------------------------------------
void Producer_thread::run() {
    // 循环条件每次都重新读 stop_，这样 request_stop() 后本循环会尽快退出
    while (!stop_.load(std::memory_order_acquire)) {

        // ---- 第 1 步：从数据源取一个任务 ----
        std::optional<Task> task;
        try {
            task = gen_();

            // 【设计要点】这一行在【锁外】执行，非常重要。
            // gen_() 是用户代码，可能读文件、查数据库、等网络，
            // 耗时完全不可控。虽然它并不持有队列的锁，
            // 但把它放在循环最前面、与 push 明确分离，
            // 能保证将来给 Queue 加锁范围变化时不会误伤。
        } catch (...) {
            // 【设计要点·异常隔离】
            // 数据源抛异常时，绝不能让异常逃出 run() ——
            // 异常从线程主函数逃逸 = 直接调用 std::terminate()，
            // 整个进程崩溃，其他所有线程一起陪葬。
            //
            // 这里的选择是【记录并跳过】：偶发的坏数据不该终止整个生产者。
            // 如果你的业务要求"出错即停"，把 continue 改成 break 即可。
            failed_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // 返回 nullopt = 数据源耗尽，正常结束（不是错误）
        if (!task) {
            break;
        }

        // ---- 第 2 步：投递到队列 ----
        // 队列满时这里会阻塞 → 这就是【背压】：
        // 消费者来不及处理时，生产者自动慢下来，防止内存无限增长。
        auto result = queue_.wait_push_for(std::move(*task), kPushSlice);

        if (result == PushResult::Closed) {
            break;      // 队列已关闭，永久性失败，没必要再试
        }
        if (result == PushResult::Timeout) {
            continue;   // 只是暂时没空位，回到循环顶部重新检查 stop_
        }

        // ---- 第 3 步：计数 ----
        // 注意：只有【成功入队】才计数。
        // 这样 produced() 的语义才严格等于"已进入队列的任务数"，
        // 测试里才能用它精确校验"任务有无丢失"。
        produced_.fetch_add(1, std::memory_order_relaxed);
    }
}
