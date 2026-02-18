#ifndef CORE_COMMUNICATION_DDS_TOPIC
#define CORE_COMMUNICATION_DDS_TOPIC
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace core::communication {

namespace dds = eprosima::fastdds::dds;

template <typename PubSubType>
class DDSTopic
{
    static_assert(
        std::is_base_of_v<dds::TopicDataType, PubSubType>,
        "PubSubType must be derived from eprosima::fastdds::dds::TopicDataType");

  public:
    using MsgType = typename PubSubType::type;

    DDSTopic(
        std::shared_ptr<dds::DomainParticipant> participant,
        const std::string& topic_name)
        : participant_(std::move(participant))
    {
        if (!participant_)
        {
            throw std::invalid_argument("Participant cannot be null");
        }

        dds::TypeSupport type_support(new PubSubType());
        type_support.register_type(participant_.get());

        auto* raw_topic = participant_->create_topic(
            topic_name, type_support.get_type_name(), dds::TOPIC_QOS_DEFAULT);

        if (!raw_topic)
        {
            throw std::runtime_error("FastDDS failed to create topic: " + topic_name);
        }

        topic_ = std::unique_ptr<dds::Topic, TopicDeleter>(
            raw_topic,
            [part = participant_](dds::Topic* t)
            {
                if (part && t)
                {
                    part->delete_topic(t);
                }
            });
    }

    DDSTopic(const DDSTopic&) = delete;
    DDSTopic& operator=(const DDSTopic&) = delete;

    DDSTopic(DDSTopic&&) = default;
    DDSTopic& operator=(DDSTopic&&) = default;

    [[nodiscard]] dds::Topic* Get() const noexcept { return topic_.get(); }
    [[nodiscard]] std::string Name() const noexcept { return topic_->get_name(); };

  private:
    using TopicDeleter = std::function<void(dds::Topic*)>;

    std::shared_ptr<dds::DomainParticipant> participant_;
    std::unique_ptr<dds::Topic, TopicDeleter> topic_;
};
}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_TOPIC
