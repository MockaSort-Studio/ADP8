#include "core/communication/dds_topic.hpp"

#include <memory>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <gtest/gtest.h>

#include "TestPubSubTypes.hpp"

namespace {
namespace dds = eprosima::fastdds::dds;
class DDSTopicTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        auto factory = dds::DomainParticipantFactory::get_instance();
        participant = std::shared_ptr<dds::DomainParticipant>(
            factory->create_participant(0, dds::PARTICIPANT_QOS_DEFAULT),
            [factory](dds::DomainParticipant* p)
            {
                if (p)
                    factory->delete_participant(p);
            });
    }
    std::shared_ptr<dds::DomainParticipant> participant;
};

TEST_F(DDSTopicTest, TopicAcquisition)
{
    const std::string name = "Topic";
    auto topic_handle =
        std::make_unique<core::communication::DDSTopic<TestPayloadPubSubType>>(
            participant, name);

    ASSERT_NE(topic_handle->Get(), nullptr);
    EXPECT_EQ(topic_handle->Name(), name);

    auto* found = participant->lookup_topicdescription(name);
    EXPECT_EQ(found, topic_handle->Get());
}

TEST_F(DDSTopicTest, AutomaticRelease)
{
    const std::string name = "TransientTopic";

    {
        core::communication::DDSTopic<TestPayloadPubSubType> topic(participant, name);
        ASSERT_NE(participant->lookup_topicdescription(name), nullptr);
    }  // 'topic' goes out of scope here; custom deleter should trigger

    // Check if FastDDS participant no longer knows about this topic
    EXPECT_EQ(participant->lookup_topicdescription(name), nullptr);
}
}  // namespace
