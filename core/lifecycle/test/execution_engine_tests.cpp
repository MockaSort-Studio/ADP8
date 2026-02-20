#include <future>
#include <map>

#include <gtest/gtest.h>

#include "core/lifecycle/execution_engine.hpp"
namespace core::lifecycle {
using namespace std::chrono_literals;

TEST(ExecutionEngineTest, PeriodicExecutionCount)
{
    core::lifecycle::ExecutionEngine engine;
    engine.Start();

    std::atomic<int> count_5ms {0};
    std::atomic<int> count_20ms {0};

    std::function<void()> pulse_5ms = [&]() { count_5ms++; };
    std::function<void()> pulse_20ms = [&]() { count_20ms++; };

    // Kick them off
    engine.Schedule(5ms, pulse_5ms);
    engine.Schedule(20ms, pulse_20ms);

    // Wait for the window to close
    std::this_thread::sleep_for(100ms);
    engine.Stop();

    // EXPECTATIONS:
    // 100ms / 5ms = 20 executions
    // 100ms / 20ms = 5 executions
    // We allow for a small margin (e.g., +/- 1) because OS scheduling
    // isn't hard real-time, but it should be very close.
    EXPECT_GE(count_5ms.load(), 19);
    EXPECT_GE(count_20ms.load(), 4);
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