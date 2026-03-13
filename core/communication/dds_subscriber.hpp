#ifndef CORE_COMMUNICATION_DDS_SUBSCRIBER
#define CORE_COMMUNICATION_DDS_SUBSCRIBER
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <optional>
#include <thread>

#include "core/communication/dds_context.hpp"
#include "core/support/utils/size_constrained_queue.hpp"
namespace core::communication {
namespace dds = eprosima::fastdds::dds;

namespace {

template <typename Type, std::size_t QueueSize>
class SubListener : public dds::DataReaderListener {
  using Queue = core::utils::SizeConstrainedQueue<Type, QueueSize>;

 public:
  SubListener() : samples_(0) {}

  ~SubListener() override {}
  void on_subscription_matched(
      dds::DataReader*, const dds::SubscriptionMatchedStatus& info) override {
    matched_count_ = info.current_count;
    if (info.current_count_change == 1) {
      std::cout << "Subscriber matched." << std::endl;
    } else if (info.current_count_change == -1) {
      std::cout << "Subscriber unmatched." << std::endl;
    } else {
      std::cout << info.current_count_change
                << " is not a valid value for SubscriptionMatchedStatus "
                   "current count change"
                << std::endl;
    }
  }

  void on_data_available(dds::DataReader* reader) override {
    dds::SampleInfo info;
    Type message;
    if (reader->take_next_sample(&message, &info) == dds::RETCODE_OK) {
      if (info.valid_data) {
        samples_++;
        message_queue_.Push(std::move(message));
      }
    }
  }

  // Change return type to std::optional to avoid default-constructing Type()
  std::optional<Type> GetSample() {
    auto sample = std::move(message_queue_.GetSample());
    if (sample) {
      return std::move(sample->data);
    }
    return std::nullopt;
  }

  inline void DrainQueue(Queue& other) noexcept {
    message_queue_.TransferTo(other);
  }

  std::atomic<int> samples_;
  std::atomic<int> matched_count_{0};

 private:
  Queue message_queue_;
};
}  // namespace

/// @brief Wraps a FastDDS DataReader for a single topic, with a fixed-size sample queue.
///
/// Incoming samples are pushed into a @c SizeConstrainedQueue by the listener callback.
/// Call @c DrainQueue() to transfer all buffered samples at once (used by
/// @c DataEndpoint::Sync()), or @c GetSample() to consume one sample at a time.
///
/// @tparam PubSubType FastDDS PubSubType. Must derive from @c TopicDataType.
/// @tparam QueueSize  Maximum number of samples buffered (default: 1).
template <typename PubSubType, std::size_t QueueSize = 1>
class DDSSubscriber {
  static_assert(
      std::is_base_of_v<dds::TopicDataType, PubSubType>,
      "PubSubType must be derived from eprosima::fastdds::dds::TopicDataType");

 public:
  using DDSDataType = typename PubSubType::type;
  using DDSListener = SubListener<DDSDataType, QueueSize>;

  DDSSubscriber() = default;

  /// @brief Creates the FastDDS Subscriber and DataReader for the given topic.
  ///        Retrieves the DomainParticipant from @c DDSContextProvider.
  /// @param topic_name DDS topic name.
  /// @throws std::runtime_error if Subscriber or DataReader creation fails.
  void Start(const std::string& topic_name) {
    // we store a pointer to participant for lifecycle mgmt of subscriber and
    // data reader
    auto& ctx = DDSContextProvider<>::Get();
    participant_ = ctx.GetDomainParticipant();

    auto* raw_sub =
        participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (!raw_sub) throw std::runtime_error("Failed to create Subscriber");

    sub_ =
        std::unique_ptr<dds::Subscriber, std::function<void(dds::Subscriber*)>>(
            raw_sub, [part = participant_](dds::Subscriber* s) {
              if (part && s) part->delete_subscriber(s);
            });

    auto* raw_reader =
        sub_->create_datareader(ctx.GetDDSTopic<PubSubType>(topic_name),
                                dds::DATAREADER_QOS_DEFAULT, &listener_);

    if (!raw_reader) throw std::runtime_error("Failed to create DataReader");

    reader_ =
        std::unique_ptr<dds::DataReader, std::function<void(dds::DataReader*)>>(
            raw_reader, [s = sub_.get()](dds::DataReader* r) {
              if (s && r) s->delete_datareader(r);
            });
  }
  ~DDSSubscriber() = default;

  /// @return True if at least one publisher is currently matched.
  [[nodiscard]] bool IsMatched() const { return listener_.matched_count_ > 0; }

  /// @brief Dequeues and returns the most recent sample.
  /// @return The sample, or @c std::nullopt if the queue is empty.
  std::optional<DDSDataType> GetSample() {
    return std::move(listener_.GetSample());
  }

  /// @brief Transfers all buffered samples from the listener queue into @p other.
  ///        Replaces the contents of @p other entirely. Used by @c DataEndpoint::Sync().
  inline void DrainQueue(
      utils::SizeConstrainedQueue<DDSDataType, QueueSize>& other) noexcept {
    listener_.DrainQueue(other);
  }

 private:
  std::shared_ptr<dds::DomainParticipant> participant_;

  std::unique_ptr<dds::Subscriber, std::function<void(dds::Subscriber*)>> sub_;
  std::unique_ptr<dds::DataReader, std::function<void(dds::DataReader*)>>
      reader_;

  DDSListener listener_;
};

template <typename TopicType>
using DDSSubscriberPtr = std::unique_ptr<DDSSubscriber<TopicType>>;

}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_SUBSCRIBER
