#ifndef CORE_COMMUNICATION_DDS_SUBSCRIBER
#define CORE_COMMUNICATION_DDS_SUBSCRIBER
#include <chrono>
#include <cstdint>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
namespace core::communication {
namespace {

template <typename Type>
class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
  public:
    SubListener() : samples_(0) {}

    ~SubListener() override {}

    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            std::cout << "Subscriber matched." << std::endl;
        } else if (info.current_count_change == -1)
        {
            std::cout << "Subscriber unmatched." << std::endl;
        } else
        {
            std::cout << info.current_count_change
                      << " is not a valid value for SubscriptionMatchedStatus "
                         "current count change"
                      << std::endl;
        }
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override
    {
        std::thread::id this_id = std::this_thread::get_id();

        std::cout << "Thread ID: " << this_id << std::endl;
        eprosima::fastdds::dds::SampleInfo info;
        if (reader->take_next_sample(&hello_, &info) ==
            eprosima::fastdds::dds::RETCODE_OK)
        {
            if (info.valid_data)
            {
                samples_++;
                std::cout << "Message: " << hello_.message()
                          << " with index: " << hello_.index() << " RECEIVED."
                          << std::endl;
            }
        }
    }

    Type hello_;
    std::atomic_int samples_;
};
}  // namespace

template <typename TopicType>
class DDSSubscriber
{
  public:
    using DDSDataType = typename TopicType::type;
    using DDSTopic = eprosima::fastdds::dds::Topic;
    using DDSDataReader = eprosima::fastdds::dds::DataReader;
    using DDSSub = eprosima::fastdds::dds::Subscriber;
    using DDSListener = SubListener<DDSDataType>;

    DDSSubscriber(
        const std::string topic_name,
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
        sub_ = std::shared_ptr<DDSSub>(participant_->create_subscriber(
            eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT, nullptr));

        if (sub_.get() == nullptr)
        {
            throw std::runtime_error("Failed to create Subscriber");
        }

        // Create the DataWriter
        reader_ = std::shared_ptr<DDSDataReader>(sub_->create_datareader(
            topic_.get(),
            eprosima::fastdds::dds::DATAREADER_QOS_USE_TOPIC_QOS,
            &listener_));

        if (reader_.get() == nullptr)
        {
            throw std::runtime_error("Failed to create DataReader");
        }
    }
    ~DDSSubscriber()
    {
        if (reader_.get() != nullptr)
        {
            sub_->delete_datareader(reader_.get());
        }
        if (sub_.get() != nullptr)
        {
            participant_->delete_subscriber(sub_.get());
        }
        if (topic_.get() != nullptr)
        {
            participant_->delete_topic(topic_.get());
        }
    }
    // tobe changed
    uint32_t NumberOfMessagesReceived() { return listener_.samples_; }

  private:
    std::shared_ptr<DDSTopic> topic_;
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;

    std::shared_ptr<DDSDataReader> reader_;
    std::shared_ptr<DDSSub> sub_;
    DDSListener listener_;
};

}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_SUBSCRIBER
