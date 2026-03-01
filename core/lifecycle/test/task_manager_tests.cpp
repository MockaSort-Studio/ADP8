#include <csignal>
#include <stdexcept>

#include <gtest/gtest.h>

#include "core/lifecycle/tasks_manager.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

namespace {
static std::atomic<int> mock_task_1_count {0};
static std::atomic<int> mock_task_2_count {0};

class MockTask1 : public TaskInterface
{
  public:
    using TaskInterface::TaskInterface;
    void ExecuteStep() override { mock_task_1_count++; }
};

class MockTask2 : public TaskInterface
{
  public:
    using TaskInterface::TaskInterface;
    void ExecuteStep() override { mock_task_2_count++; }
};

class TasksManagerFixture : public ::testing::TestWithParam<int>
{
  protected:
    void SetUp() override
    {
        detail::shutdown_requested.store(false);

        mock_task_1_count.store(0);
        mock_task_2_count.store(0);
    }

    void TearDown() override
    {
        if (killer_thread_.joinable())
        {
            killer_thread_.join();
        }
    }

    // simulating shutdown caused by OS signals
    void ScheduleShutdown(std::chrono::milliseconds delay, int sig_type)
    {
        if (sig_type != SIGTERM && sig_type != SIGINT)
        {
            throw std::runtime_error(
                "only SIGINT(2) and SIGTERM(15) suported at the moment");
        }

        killer_thread_ = std::thread(
            [delay, sig_type]()
            {
                std::this_thread::sleep_for(delay);
                std::raise(sig_type);
            });

        killer_thread_.detach();
    }

    core::lifecycle::TasksManager manager_;
    std::thread killer_thread_;
};
}  // namespace

TEST_F(TasksManagerFixture, MultipleTasksBasicRun)
{
    manager_.AddTask<MockTask1, 10>("TaskA");
    manager_.AddTask<MockTask2, 10>("TaskB");

    ScheduleShutdown(55ms, SIGINT);

    manager_.Run();

    // Mathematically: 55ms / 10ms = 5.5 pulses.
    EXPECT_GE(mock_task_1_count.load(), 3) << "TaskA under-performed!";
    EXPECT_GE(mock_task_2_count.load(), 3) << "TaskB under-performed!";

    // Also verify they are running roughly in sync
    EXPECT_NEAR(mock_task_1_count.load(), mock_task_2_count.load(), 1);
}

TEST_P(TasksManagerFixture, ShutdownTest)
{
    auto signal_type = GetParam();
    auto start = std::chrono::steady_clock::now();

    ScheduleShutdown(500ms, signal_type);
    manager_.Run();

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_GE(elapsed.count(), 450);
    EXPECT_LE(elapsed.count(), 1000);
}

INSTANTIATE_TEST_SUITE_P(
    TimingTests, TasksManagerFixture, ::testing::Values(SIGINT, SIGTERM));
}  // namespace core::lifecycle