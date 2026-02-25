#include <csignal>
#include <tuple>

#include "core/lifecycle/dds_task.hpp"
#include "core/lifecycle/task_interface.hpp"
#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

class MockDDSTask : public DDSTask<TestTopicSubsList, TestTopicPubsList>
{
  public:
    using DDSTask<TestTopicSubsList, TestTopicPubsList>::DDSTask;

  protected:
    void Execute() override {}
};
TEST_F(LifecycleFixture, SameTopicSpecializedWithDifferentQueue)
{
    using TestSubTrait = std::tuple_element_t<0, TestTopicSubsList>;
    using TestPubTrait = std::tuple_element_t<0, TestTopicPubsList>;

    EXPECT_EQ(TestSubTrait::kName, TestPubTrait::kName);
    EXPECT_EQ(TestSubTrait::kQueueSize, 2);
}

TEST_F(LifecycleFixture, DDSTaskTest) { MockDDSTask task("test_task"); }
}  // namespace core::lifecycle