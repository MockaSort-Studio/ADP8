#include <csignal>
#include <stdexcept>

#include <gtest/gtest.h>

#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

TEST_F(LifecycleFixture, MultipleTasksBasicRun)
{
    manager_.Start();

    manager_.AddTask<MockTask1, 10>("TaskA");
    manager_.AddTask<MockTask2, 10>("TaskB");

    std::this_thread::sleep_for(55ms);

    manager_.Stop();

    // Mathematically: 55ms / 10ms = 5.5 pulses.
    EXPECT_GE(mock_task_1_count.load(), 3) << "TaskA under-performed!";
    EXPECT_GE(mock_task_2_count.load(), 3) << "TaskB under-performed!";

    // Also verify they are running roughly in sync
    EXPECT_NEAR(mock_task_1_count.load(), mock_task_2_count.load(), 1);
}
}  // namespace core::lifecycle