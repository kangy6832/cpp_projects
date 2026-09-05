#include "Producer_thread.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

// 阻塞等待的切片时长：既保证停止时能及时响应，又不至于空转太频繁
namespace {
constexpr auto kPushSlice = 50ms;
}

Producer_thread::Producer_thread(Queue& queue, Generator gen, std::string name)
    : queue_(queue), gen_(std::move(gen)), name_(std::move(name)) {}

Producer_thread::~Producer_thread() {
    request_stop();
    join();
}

void Producer_thread::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;  // 已在运行，避免重复启动
    }
    stop_.store(false, std::memory_order_release);

    thread_ = std::thread([this] {
        run();
        running_.store(false, std::memory_order_release);
    });
}

void Producer_thread::request_stop() noexcept {
    stop_.store(true, std::memory_order_release);
    // 若 Queue 提供了 notify_producers()，在此调用可实现零延迟唤醒；
    // 当前依赖 wait_push_for 的超时返回，最坏延迟 kPushSlice。
}

void Producer_thread::join() {
    // 禁止自 join（会抛 system_error / 死锁）
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

void Producer_thread::run() {
    while (!stop_.load(std::memory_order_acquire)) {
        // 1. 从数据源取一个任务
        std::optional<Task> task;
        try {
            task = gen_();
        } catch (...) {
            // 数据源偶发异常视为可恢复：跳过该条继续。
            // 若希望异常即终止生产者，把这里的 continue 改成 break。
            failed_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        if (!task) {
            break;  // 数据源耗尽，自然退出
        }

        // 2. 投递。队列满则阻塞（背压）；Closed 表示队列已关闭
        auto result = queue_.wait_push_for(std::move(*task), kPushSlice);
        if (result == PushResult::Closed) {
            break;
        }
        if (result == PushResult::Timeout) {
            continue;  // 回到循环头部重新检查 stop_
        }

        produced_.fetch_add(1, std::memory_order_relaxed);
    }
}
