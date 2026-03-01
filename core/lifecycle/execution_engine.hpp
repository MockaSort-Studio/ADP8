#ifndef CORE_LIFECYCLE_EXECUTION_ENGINE
#define CORE_LIFECYCLE_EXECUTION_ENGINE

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace core::lifecycle {

using namespace std::chrono_literals;

struct Job
{
    std::chrono::steady_clock::time_point time;
    std::function<void()> callback;

    // C++17: Using > for min-priority queue (earliest time first)
    bool operator>(const Job& other) const { return time > other.time; }
};

class ExecutionEngine final
{
  public:
    ExecutionEngine() = default;
    ~ExecutionEngine() { Stop(); }

    // Disable copy/move to prevent accidental thread duplication
    ExecutionEngine(const ExecutionEngine&) = delete;
    ExecutionEngine& operator=(const ExecutionEngine&) = delete;

    void Start();
    void Stop();

    void Schedule(std::chrono::milliseconds period, std::function<void()> func);

    [[nodiscard]] bool IsRunning() const { return running_.load(); }
    [[nodiscard]] int TaskQueueSize() const { return queue_.size(); }

  private:
    void Run();
    void ScheduleInternal(
        std::chrono::steady_clock::time_point time, std::function<void()> func);
    std::mutex mtx_;
    std::condition_variable cv_;
    std::priority_queue<Job, std::vector<Job>, std::greater<>> queue_;
    std::thread worker_;
    std::atomic<bool> running_ {false};
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_EXECUTION_ENGINE
