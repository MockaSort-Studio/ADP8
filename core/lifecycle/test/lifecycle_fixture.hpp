#ifndef CORE_LIFECYCLE_TEST_LIFECYCLE_FIXTURE
#define CORE_LIFECYCLE_TEST_LIFECYCLE_FIXTURE
#include <gtest/gtest.h>

#include "core/lifecycle/dds_application.hpp"
#include "core/lifecycle/dds_task.hpp"
#include "core/support/utils/lookup_table.hpp"
#include "test_ports_subscriptions.hpp"
#include "test_ports_publications.hpp"

namespace core::lifecycle {
static std::atomic<int> mock_task_1_count {0};
static std::atomic<int> mock_task_2_count {0};

using DDSTaskTopicSubsList = gen::Subscriptions;
using DDSTaskTopicPubsList = gen::Publications;

using TestTopicSubsList = TopicList<
    communication::TopicSpec<TestPayloadPubSubType, gen::kSendFalseTopicName, 2>,
    communication::TopicSpec<TestPayloadPubSubType, gen::kSendTrueTopicName, 2>>;
using TestTopicPubsList = TopicList<
    communication::TopicSpec<TestPayloadPubSubType, gen::kSendFalseTopicName>,
    communication::TopicSpec<TestPayloadPubSubType, gen::kSendTrueTopicName>>;

using TestInputs = Inputs_t<TestTopicSubsList>;
using TestOutputs = Outputs_t<TestTopicPubsList>;

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
class SendFalseDDSTask : public DDSTask<DDSTaskTopicSubsList, DDSTaskTopicPubsList>
{
  public:
    using DDSTask<DDSTaskTopicSubsList, DDSTaskTopicPubsList>::DDSTask;

  protected:
    void Execute() override
    {
        auto in = GetInputSource<gen::kSendFalseTopicName>();
        auto out = GetOutputSink<gen::kSendTrueTopicName>();

        if (in.Empty())
        {
            return;
        }

        auto data = in[0].data;
        EXPECT_TRUE(data.ok());
        TestPayload response;
        response.ok(false);
        out.Push(std::move(response));
    }
};

class SendTrueDDSTask : public DDSTask<DDSTaskTopicPubsList, DDSTaskTopicSubsList>
{
  public:
    using DDSTask<DDSTaskTopicPubsList, DDSTaskTopicSubsList>::DDSTask;

  protected:
    void Execute() override
    {
        auto in = GetInputSource<gen::kSendTrueTopicName>();
        auto out = GetOutputSink<gen::kSendFalseTopicName>();

        if (in.Empty())
        {
            return;
        }

        auto data = in[0].data;
        EXPECT_FALSE(data.ok());
        TestPayload response;
        response.ok(true);
        out.Push(std::move(response));
    }
};
using TestApplicationConfig = core::utils::LookupTable<
    core::utils::TableItem<TaskSpec<SendTrueDDSTask, 10>>,
    core::utils::TableItem<TaskSpec<SendFalseDDSTask, 20>>>;

class LifecycleFixture : public ::testing::TestWithParam<int>
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
    core::lifecycle::DDSAPPlication<TestApplicationConfig> app_ {
        "test_dds_domain_participant"};
    std::thread killer_thread_;
};
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_TEST_LIFECYCLE_FIXTURE
