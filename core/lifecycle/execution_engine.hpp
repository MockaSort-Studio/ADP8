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

/// @brief A single scheduled work item: a deadline and a callback.
///        Ordered by time (earliest first) in the priority queue.
struct Job {
  std::chrono::steady_clock::time_point time;
  std::function<void()> callback;

  // C++17: Using > for min-priority queue (earliest time first)
  bool operator>(const Job& other) const { return time > other.time; }
};

/// @brief Single-threaded periodic task scheduler backed by a min-heap priority
/// queue.
///
/// Each registered function wraps into a self-rescheduling callback: after
/// execution it re-enqueues itself at @c now + period. The worker thread sleeps
/// until the next deadline using @c condition_variable::wait_until.
///
/// Exceptions from callbacks are caught and logged; the failed task is not
/// rescheduled. Non-copyable. Destruction automatically calls @c Stop().
class ExecutionEngine final {
 public:
  ExecutionEngine() = default;
  ~ExecutionEngine() { Stop(); }

  // Disable copy/move to prevent accidental thread duplication
  ExecutionEngine(const ExecutionEngine&) = delete;
  ExecutionEngine& operator=(const ExecutionEngine&) = delete;

  /// @brief Spawns the worker thread. No-op if already running.
  void Start();

  /// @brief Clears the queue, signals the worker to exit, and joins it.
  /// Idempotent.
  void Stop();

  /// @brief Registers a periodic task.
  ///
  /// First invocation fires at @c now + @p period. Re-schedules itself on every
  /// execution. Call after @c Start().
  ///
  /// @param period Execution interval.
  /// @param func   Callable invoked at each period.
  void Schedule(std::chrono::milliseconds period, std::function<void()> func);

  /// @return True if the worker thread is currently running.
  [[nodiscard]] bool IsRunning() const { return running_.load(); }
  [[nodiscard]] int TaskQueueSize() const { return queue_.size(); }

 private:
  void Run();
  void ScheduleInternal(std::chrono::steady_clock::time_point time,
                        std::function<void()> func);
  std::mutex mtx_;
  std::condition_variable cv_;
  std::priority_queue<Job, std::vector<Job>, std::greater<>> queue_;
  std::thread worker_;
  std::atomic<bool> running_{false};
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_EXECUTION_ENGINE
