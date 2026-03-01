#include <future>
#include <map>
#include <numeric>
#include <gtest/gtest.h>

#include "core/lifecycle/execution_engine.hpp"

namespace core::lifecycle {
using namespace std::chrono_literals;

TEST(ExecutionEngineTest, PeriodicIntervalIntegrity) {
    core::lifecycle::ExecutionEngine engine;
    engine.Start();

    std::mutex mtx;
    std::vector<std::chrono::steady_clock::time_point> hits;
    const auto period = 10ms;
    const int required_samples = 10;

    // Capture the exact time of activation
    engine.Schedule(period, [&]() {
        std::lock_guard lock(mtx);
        hits.push_back(std::chrono::steady_clock::now());
    });

    // Wait until enough samples have been collected or time out (prevents infinite hang)
    auto start_wait = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard lock(mtx);
            if (hits.size() >= required_samples) break;
        }
        if (std::chrono::steady_clock::now() - start_wait > 500ms) break;
        std::this_thread::sleep_for(10ms);
    }

    engine.Stop();

    ASSERT_GE(hits.size(), 2) << "Engine didn't even pulse twice!";

    std::vector<double> deltas_ms;
    for (size_t i = 1; i < hits.size(); ++i) {
        std::chrono::duration<double, std::milli> diff = hits[i] - hits[i-1];
        deltas_ms.push_back(diff.count());
    }

    for (double dt : deltas_ms) {
        EXPECT_GE(dt, 9.5) << "Task triggered too fast!";
    }

    double avg_dt = std::accumulate(deltas_ms.begin(), deltas_ms.end(), 0.0) / deltas_ms.size();
    
    // Allow 20% overhead for noisy neighbors.
    EXPECT_NEAR(avg_dt, 10.0, 2.0) << "Average period is outside of acceptable jitter range.";
}

TEST(ExecutionEngineTest, StopBreaksPeriodicChain)
{
    ExecutionEngine engine;
    engine.Start();

    std::atomic<int> count {0};

    engine.Schedule(1ms, [&]() { count++; });

    std::this_thread::sleep_for(10ms);
    engine.Stop();

    int count_after_stop = count.load();

    // Wait a bit more to ensure no "ghost" tasks ran
    std::this_thread::sleep_for(20ms);

    EXPECT_EQ(count.load(), count_after_stop)
        << "Tasks continued executing after engine was stopped!";
}

TEST(ExecutionEngineTest, PeriodicTaskResilience)
{
    ExecutionEngine engine;
    engine.Start();
    engine.Schedule(5ms, [&]() { throw std::runtime_error("First run failure"); });

    std::this_thread::sleep_for(10ms);

    EXPECT_EQ(engine.TaskQueueSize(), 0);
}

}  // namespace core::lifecycle