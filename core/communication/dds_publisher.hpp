#ifndef CORE_COMMUNICATION_DDS_PUBLISHER
#define CORE_COMMUNICATION_DDS_PUBLISHER

#include <memory>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "core/communication/dds_context.hpp"

namespace core::communication {

namespace dds = eprosima::fastdds::dds;

namespace {
class PubListener : public dds::DataWriterListener
{
  public:
    PubListener() : matched_(0) {}

    ~PubListener() override {}

    void on_publication_matched(
        dds::DataWriter*, const dds::PublicationMatchedStatus& info) override
    {
        matched_count_ = info.current_count;

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
    std::atomic<int> matched_count_ {0};
};
}  // namespace

template <typename PubSubType>
class DDSPublisher
{
    static_assert(
        std::is_base_of_v<dds::TopicDataType, PubSubType>,
        "PubSubType must be derived from eprosima::fastdds::dds::TopicDataType");

  public:
    using DDSDataType = typename PubSubType::type;

    DDSPublisher() = default;

    void Start(const std::string& topic_name)
    {
        // we store a pointer to participant for lifecycle mgmt of publisher and data
        // writer
        auto& ctx = DDSContextProvider::Get();
        participant_ = ctx.GetDomainParticipant();

        auto* raw_pub =
            participant_->create_publisher(dds::PUBLISHER_QOS_DEFAULT, nullptr);

        if (!raw_pub)
            throw std::runtime_error("Failed to create Publisher");

        pub_ = std::unique_ptr<dds::Publisher, std::function<void(dds::Publisher*)>>(
            raw_pub,
            [part = participant_](dds::Publisher* p)
            {
                if (part && p)
                    part->delete_publisher(p);
            });

        auto* raw_writer = pub_->create_datawriter(
            ctx.GetDDSTopic<PubSubType>(topic_name),
            dds::DATAWRITER_QOS_DEFAULT,
            &listener_);

        if (!raw_writer)
            throw std::runtime_error("Failed to create DataWriter");

        writer_ = std::unique_ptr<dds::DataWriter, std::function<void(dds::DataWriter*)>>(
            raw_writer,
            [p = pub_.get()](dds::DataWriter* w)
            {
                if (p && w)
                    p->delete_datawriter(w);
            });
    }

    bool Publish(const DDSDataType& payload)
    {
        if (listener_.matched_count_ > 0)
        {
            return writer_->write(const_cast<DDSDataType*>(&payload)) == dds::RETCODE_OK;
        }
        return false;
    }

    [[nodiscard]] bool IsMatched() const { return listener_.matched_count_ > 0; }

  private:
    std::shared_ptr<dds::DomainParticipant> participant_;

    std::unique_ptr<dds::Publisher, std::function<void(dds::Publisher*)>> pub_;
    std::unique_ptr<dds::DataWriter, std::function<void(dds::DataWriter*)>> writer_;

    PubListener listener_;
};
template <typename TopicType>
using DDSPublisherPtr = std::unique_ptr<DDSPublisher<TopicType>>;
}  // namespace core::communication
#endif  // CORE_COMMUNICATION_DDS_PUBLISHER
