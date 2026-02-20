#ifndef CORE_LIFECYCLE_EXECUTION_ENGINE
#define CORE_LIFECYCLE_EXECUTION_ENGINE

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
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

class ExecutionEngine
{
  public:
    ExecutionEngine() = default;
    ~ExecutionEngine() { Stop(); }

    // Disable copy/move to prevent accidental thread duplication
    ExecutionEngine(const ExecutionEngine&) = delete;
    ExecutionEngine& operator=(const ExecutionEngine&) = delete;

    void Start()
    {
        std::scoped_lock lock(mtx_);
        if (!worker_.joinable())
        {
            running_ = true;
            worker_ = std::thread(&ExecutionEngine::Run, this);
        }
    }

    void Stop()
    {
        {
            std::scoped_lock lock(mtx_);
            if (!running_)
                return;
            running_ = false;

            // C++17: Clear queue by swapping with an empty temporary
            std::priority_queue<Job, std::vector<Job>, std::greater<>> {}.swap(queue_);
        }
        cv_.notify_all();
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    // --- The Periodic Schedule (User Facing) ---
    void Schedule(std::chrono::milliseconds period, std::function<void()> func)
    {
        // We use a shared_ptr so the lambda can capture its own "intent" safely
        auto wrapper = std::make_shared<std::function<void()>>();

        *wrapper = [this, period, func = std::move(func), wrapper]()
        {
            std::invoke(func);  // Run user logic

            // Re-schedule itself for Xms from now
            this->Schedule(std::chrono::steady_clock::now() + period, *wrapper);
        };

        // Kick off the first run
        Schedule(std::chrono::steady_clock::now() + period, *wrapper);
    }

    // --- The Internal One-Shot Schedule ---
    void Schedule(std::chrono::steady_clock::time_point time, std::function<void()> func)
    {
        {
            std::scoped_lock lock(mtx_);
            if (!running_)
                return;
            queue_.push({time, std::move(func)});
        }
        cv_.notify_one();
    }

    [[nodiscard]] bool IsRunning() const { return running_; }
    [[nodiscard]] int TaskQueueSize() const { return queue_.size(); }

  private:
    void Run()
    {
        while (running_)
        {
            std::unique_lock<std::mutex> lock(mtx_);

            if (queue_.empty())
            {
                cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
            } else
            {
                auto next_run = queue_.top().time;
                cv_.wait_until(
                    lock,
                    next_run,
                    [this, next_run]
                    { return !running_ || queue_.top().time < next_run; });
            }

            if (!running_)
                break;

            if (!queue_.empty() && queue_.top().time <= std::chrono::steady_clock::now())
            {
                auto job = std::move(const_cast<Job&>(queue_.top()));
                queue_.pop();
                lock.unlock();

                if (job.callback)
                {
                    // if it fails it's not rescheduled
                    try
                    {
                        std::invoke(job.callback);
                    } catch (const std::exception& e)
                    {
                        std::cerr << "[ExecutionEngine] Task Exception: " << e.what()
                                  << std::endl;
                    }
                }
            }
        }
    }

    std::mutex mtx_;
    std::condition_variable cv_;
    std::priority_queue<Job, std::vector<Job>, std::greater<>> queue_;
    std::thread worker_;
    std::atomic<bool> running_ {false};
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_EXECUTION_ENGINE
