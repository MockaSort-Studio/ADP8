#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <gtest/gtest.h>

#include "core/communication/dds_context.hpp"
#include "TestPubSubTypes.hpp"

namespace core::communication {
namespace dds = eprosima::fastdds::dds;
class DDSContextTest : public ::testing::Test
{
  protected:
    void SetUp() override {}
    DDSContext context_ {"test_domain_participant"};
};
inline constexpr char kTestTopicName[] = "Topic";

TEST_F(DDSContextTest, SuccessfulInitialization)
{
    EXPECT_NO_THROW({
        DDSContext& ctx = DDSContextProvider::Get();
        auto participant = ctx.GetDomainParticipant();
        EXPECT_NE(participant, nullptr);
    });
}

TEST_F(DDSContextTest, ReturnsSameInstanceAddress)
{
    DDSContext& ref1 = DDSContextProvider::Get();
    DDSContext& ref2 = DDSContextProvider::Get();

    EXPECT_EQ(&ref1, &ref2) << "Provider should return the same memory address";
}

TEST_F(DDSContextTest, DomainParticipantNotNull)
{
    ASSERT_NE(context_.GetDomainParticipant(), nullptr);
}

TEST_F(DDSContextTest, GetTopicWithTopicSpecTest)
{
    auto topic_handle =
        context_.GetDDSTopic<TopicSpec<TestPayloadPubSubType, kTestTopicName>>();

    ASSERT_NE(topic_handle, nullptr);
    EXPECT_EQ(topic_handle->get_name(), "Topic");
}

TEST_F(DDSContextTest, GetTopicWithIdlTypeTest)
{
    const std::string name = "Topic";
    auto topic_handle = context_.GetDDSTopic<TestPayloadPubSubType>(name);

    ASSERT_NE(topic_handle, nullptr);
    EXPECT_EQ(topic_handle->get_name(), name);
}

TEST_F(DDSContextTest, GetTopicTestDuplicatedTypeRegistration)
{
    const std::string name = "Topic";
    auto topic_handle = context_.GetDDSTopic<TestPayloadPubSubType>(name);

    ASSERT_NE(topic_handle, nullptr);
    EXPECT_EQ(topic_handle->get_name(), name);

    const std::string duplicated_type_name = "DuplicatedTopic";

    auto duplicated_handle =
        context_.GetDDSTopic<TestPayloadPubSubType>(duplicated_type_name);

    ASSERT_NE(duplicated_handle, nullptr);
    EXPECT_EQ(duplicated_handle->get_name(), duplicated_type_name);
}

}  // namespace core::communication
