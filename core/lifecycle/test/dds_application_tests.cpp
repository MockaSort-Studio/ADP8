#include <csignal>

#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

TEST_P(LifecycleFixture, ApplicationLifecycleTest)
{
    auto signal_type = GetParam();
    auto start = std::chrono::steady_clock::now();

    ScheduleShutdown(500ms, signal_type);
    app_.Run();

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_GE(elapsed.count(), 450);
    EXPECT_LE(elapsed.count(), 1000);
}

INSTANTIATE_TEST_SUITE_P(
    TimingTests, LifecycleFixture, ::testing::Values(SIGINT, SIGTERM));
}  // namespace core::lifecycle