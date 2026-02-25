#include <tuple>

#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/data_endpoint.hpp"
#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

TEST_F(LifecycleFixture, SameTopicSpecializedWithDifferentQueue)
{
    using TestSubTrait = std::tuple_element_t<0, TestTopicSubsList>;
    using TestPubTrait = std::tuple_element_t<0, TestTopicPubsList>;

    EXPECT_EQ(TestSubTrait::kName, TestPubTrait::kName);
    EXPECT_EQ(TestSubTrait::kQueueSize, 2);
}

using TestSub = DataEndpoint<
    communication::TopicSpec<TestPayloadPubSubType, kDifferentTopicName>,
    DataDirection::In>;
using TestPub = DataEndpoint<
    communication::TopicSpec<TestPayloadPubSubType, kTestTopicName>,
    DataDirection::Out>;

TEST(DDSTaskTest, PublishTrueExpectFalse)
{
    auto pub = TestPub();
    SendFalseDDSTask task("test_task");
    auto sub = TestSub();

    TestPayload payload;
    payload.ok(true);
    pub.Push(std::move(payload));
    pub.Sync();

    std::this_thread::sleep_for(20ms);
    task.ExecuteStep();
    std::this_thread::sleep_for(20ms);
    sub.Sync();
    auto result = sub[0];

    EXPECT_FALSE(result.data.ok());
}
}  // namespace core::lifecycle