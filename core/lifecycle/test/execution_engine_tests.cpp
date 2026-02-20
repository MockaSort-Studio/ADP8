#include <future>
#include <map>

#include <gtest/gtest.h>

#include "core/lifecycle/execution_engine.hpp"  // Assuming your file name

namespace core::lifecycle {
using namespace std::chrono_literals;

TEST(ExecutionEngineTest, PeriodicExecutionCount)
{
    core::lifecycle::ExecutionEngine engine;
    engine.Start();

    std::atomic<int> count_5ms {0};
    std::atomic<int> count_20ms {0};

    // Define the 5ms heartbeat
    // Use a pointer/std::function wrapper so the lambda can capture itself
    std::function<void()> pulse_5ms = [&]() { count_5ms++; };

    // Define the 20ms heartbeat
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

    std::cout << "5ms task ran: " << count_5ms.load() << " times" << std::endl;
    std::cout << "20ms task ran: " << count_20ms.load() << " times" << std::endl;
}

TEST(ExecutionEngineTest, StopBreaksPeriodicChain)
{
    ExecutionEngine engine;
    engine.Start();

    std::atomic<int> count {0};

    // Schedule a very fast task
    engine.Schedule(1ms, [&]() { count++; });

    std::this_thread::sleep_for(10ms);
    engine.Stop();  // This clears the queue and sets running_ = false

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

    // Wait a bit more to ensure task run
    std::this_thread::sleep_for(10ms);
    // after failure task is popped forever
    EXPECT_EQ(engine.TaskQueueSize(), 0);
}

}  // namespace core::lifecycle