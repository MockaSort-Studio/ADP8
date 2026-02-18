#ifndef CORE_COMMUNICATION_DDS_PUBLISHER
#define CORE_COMMUNICATION_DDS_PUBLISHER

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace core::communication {

namespace {
class PubListener : public eprosima::fastdds::dds::DataWriterListener
{
  public:
    PubListener() : matched_(0) {}

    ~PubListener() override {}

    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher matched." << std::endl;
        } else if (info.current_count_change == -1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher unmatched." << std::endl;
        } else
        {
            std::cout << info.current_count_change
                      << " is not a valid value for PublicationMatchedStatus current "
                         "count change."
                      << std::endl;
        }
    }

    std::atomic_int matched_;
};
}  // namespace

template <typename TopicType>
class DDSPublisher
{
  public:
    using DDSDataType = typename TopicType::type;
    using DDSTopic = eprosima::fastdds::dds::Topic;
    using DDSDataWriter = eprosima::fastdds::dds::DataWriter;
    using DDSPub = eprosima::fastdds::dds::Publisher;

    DDSPublisher(
        std::string topic_name,
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant)
        : participant_(participant)
    {
        // type registration on participant, I see it happening once (see comment below)
        // type_ = std::shared_ptr<eprosima::fastdds::dds::TypeSupport>(new TopicType());
        auto type = eprosima::fastdds::dds::TypeSupport(new TopicType());
        type.register_type(participant_.get());
        std::cout << "DioSerpente" << std::endl;
        topic_ = std::shared_ptr<DDSTopic>(participant_->create_topic(
            topic_name, type.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT));
        std::cout << "DioSerpente" << std::endl;

        if (topic_.get() == nullptr)
        {
            throw std::runtime_error("Failed to create topic");
        }

        // Create the Publisher
        pub_ = std::shared_ptr<DDSPub>(participant_->create_publisher(
            eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT, nullptr));

        if (pub_.get() == nullptr)
        {
            throw std::runtime_error("Failed to create Publisher");
        }

        // Create the DataWriter
        writer_ = std::shared_ptr<DDSDataWriter>(pub_->create_datawriter(
            topic_.get(), eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT, &listener_));

        if (writer_.get() == nullptr)
        {
            throw std::runtime_error("Failed to create DataWriter");
        }
    }

    virtual ~DDSPublisher()
    {
        if (writer_.get() != nullptr)
        {
            pub_->delete_datawriter(writer_.get());
        }
        if (pub_.get() != nullptr)
        {
            participant_->delete_publisher(pub_.get());
        }
        if (topic_.get() != nullptr)
        {
            participant_->delete_topic(topic_.get());
        }
    }

    bool Publish(const DDSDataType& payload)
    {
        if (listener_.matched_ > 0)
        {
            writer_->write(&payload);
            return true;
        }
        return false;
    }

  private:
    // I can see an object managing both of it and being shared by different
    // participant in the domain
    std::shared_ptr<DDSTopic> topic_;
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;

    std::shared_ptr<DDSDataWriter> writer_;
    std::shared_ptr<DDSPub> pub_;
    PubListener listener_;
};
}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_PUBLISHER
