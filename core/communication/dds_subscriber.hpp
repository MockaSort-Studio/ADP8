#ifndef CORE_COMMUNICATION_DDS_SUBSCRIBER
#define CORE_COMMUNICATION_DDS_SUBSCRIBER
#include <chrono>
#include <cstdint>
#include <optional>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "core/communication/size_constrained_queue.hpp"
namespace core::communication {
namespace dds = eprosima::fastdds::dds;

namespace {

template <typename Type, std::size_t QueueSize>
class SubListener : public dds::DataReaderListener
{
  public:
    SubListener() : samples_(0) {}

    ~SubListener() override {}
    void on_subscription_matched(
        dds::DataReader*, const dds::SubscriptionMatchedStatus& info) override
    {
        matched_count_ = info.current_count;
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

    void on_data_available(dds::DataReader* reader) override
    {
        std::thread::id this_id = std::this_thread::get_id();

        std::cout << "Thread ID: " << this_id << std::endl;
        dds::SampleInfo info;
        Type message;
        if (reader->take_next_sample(&message, &info) == dds::RETCODE_OK)
        {
            if (info.valid_data)
            {
                samples_++;
                message_queue_.Push(std::move(message));
            }
        }
    }

    // Change return type to std::optional to avoid default-constructing Type()
    std::optional<Type> GetSample()
    {
        auto sample = std::move(message_queue_.GetSample());
        if (sample)
        {
            return std::move(sample->data);
        }
        return std::nullopt;
    }

    Type hello_;
    std::atomic<int> samples_;
    std::atomic<int> matched_count_ {0};
    SizeConstrainedQueue<Type, QueueSize> message_queue_;
};
}  // namespace

template <typename TopicClass, std::size_t QueueSize = 1>
class DDSSubscriber
{
  public:
    using DDSDataType = typename TopicClass::MsgType;
    using DDSListener = SubListener<DDSDataType, QueueSize>;

    DDSSubscriber(
        std::shared_ptr<TopicClass> topic_handle,
        std::shared_ptr<dds::DomainParticipant> participant)
        : topic_handle_(topic_handle), participant_(participant)
    {
        if (!topic_handle_.get() || !participant_.get())
        {
            throw std::runtime_error(
                "Invalid Topic or Participant provided to Subscriber");
        }

        auto* raw_sub =
            participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT, nullptr);
        if (!raw_sub)
            throw std::runtime_error("Failed to create Subscriber");

        sub_ = std::unique_ptr<dds::Subscriber, std::function<void(dds::Subscriber*)>>(
            raw_sub,
            [part = participant_](dds::Subscriber* s)
            {
                if (part && s)
                    part->delete_subscriber(s);
            });

        auto* raw_reader = sub_->create_datareader(
            topic_handle_->Get(), dds::DATAREADER_QOS_DEFAULT, &listener_);

        if (!raw_reader)
            throw std::runtime_error("Failed to create DataReader");

        reader_ = std::unique_ptr<dds::DataReader, std::function<void(dds::DataReader*)>>(
            raw_reader,
            [s = sub_.get()](dds::DataReader* r)
            {
                if (s && r)
                    s->delete_datareader(r);
            });
    }

    ~DDSSubscriber() = default;

    [[nodiscard]] bool IsMatched() const { return listener_.matched_count_ > 0; }

    std::optional<DDSDataType> GetSample() { return std::move(listener_.GetSample()); }

  private:
    std::shared_ptr<TopicClass> topic_handle_;
    std::shared_ptr<dds::DomainParticipant> participant_;

    std::unique_ptr<dds::Subscriber, std::function<void(dds::Subscriber*)>> sub_;
    std::unique_ptr<dds::DataReader, std::function<void(dds::DataReader*)>> reader_;

    DDSListener listener_;
};

template <typename TopicType>
using DDSSubscriberPtr = std::unique_ptr<DDSSubscriber<TopicType>>;

}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_SUBSCRIBER
