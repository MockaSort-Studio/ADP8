#ifndef CORE_COMMUNICATION_DDS_CONTEXT
#define CORE_COMMUNICATION_DDS_CONTEXT

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace core::communication {

namespace dds = eprosima::fastdds::dds;

constexpr uint64_t Hash(const char* str)
{
    uint64_t h = 0xcbf29ce484222325;
    while (*str)
    {
        h = (h ^ static_cast<uint64_t>(*str++)) * 0x100000001b3;
    }
    return h;
}

template <typename T, const char* TopicName, size_t max_queue_size = 0>
struct TopicSpec
{
    using type = T;
    static constexpr uint64_t kHash {Hash(TopicName)};
    static constexpr const char* kName {TopicName};
    static constexpr size_t kQueueSize {max_queue_size};
};

template <typename T>
struct is_topic_spec : std::false_type
{
};

template <typename T, const char* TopicName, size_t Q>
struct is_topic_spec<TopicSpec<T, TopicName, Q>> : std::true_type
{
};

template <typename T>
inline constexpr bool is_topic_spec_v = is_topic_spec<T>::value;

class DDSContext
{
  public:
    DDSContext(const std::string& domain_participant_name)
    {
        dds::DomainParticipantQos participantQos;
        participantQos.name(domain_participant_name);

        auto* raw_part =
            dds::DomainParticipantFactory::get_instance()->create_participant(
                0, participantQos);

        if (!raw_part)
            throw std::runtime_error("FastDDS failed to create DomainParticipant");

        participant_ = std::shared_ptr<dds::DomainParticipant>(
            raw_part,
            [](dds::DomainParticipant* p)
            {
                if (p)
                {
                    dds::DomainParticipantFactory::get_instance()->delete_participant(p);
                }
            });
    }

    ~DDSContext() = default;

    template <typename Spec>
    dds::Topic* GetDDSTopic()
    {
        static_assert(is_topic_spec_v<Spec>, "Spec must be specialization of TopicSpec");

        return GetDDSTopic<typename Spec::type>(Spec::kName);
    }

    template <typename PubSubType>
    dds::Topic* GetDDSTopic(const std::string& topic_name)
    {
        static_assert(
            std::is_base_of_v<dds::TopicDataType, PubSubType>,
            "PubSubType must be derived from eprosima::fastdds::dds::TopicDataType");

        auto it = std::find_if(
            topics_.begin(),
            topics_.end(),
            [&](const TopicPtr& t) { return t->get_name() == topic_name; });

        if (it != topics_.end())
            return it->get();

        dds::TypeSupport type_support(new PubSubType());
        type_support.register_type(participant_.get());

        auto* raw_topic = participant_->create_topic(
            topic_name, type_support.get_type_name(), dds::TOPIC_QOS_DEFAULT);

        if (!raw_topic)
            throw std::runtime_error("Failed to create topic");

        topics_.emplace_back(
            raw_topic,
            [part = participant_](dds::Topic* t)
            {
                if (part && t)
                    part->delete_topic(t);
            });

        return raw_topic;
    }

    std::shared_ptr<dds::DomainParticipant> GetDomainParticipant()
    {
        return participant_;
    }

    DDSContext(const DDSContext&) = delete;
    DDSContext& operator=(const DDSContext&) = delete;

    DDSContext(DDSContext&&) = default;
    DDSContext& operator=(DDSContext&&) = default;

  private:
    using TopicDeleter = std::function<void(dds::Topic*)>;
    using TopicPtr = std::unique_ptr<dds::Topic, TopicDeleter>;
    std::shared_ptr<dds::DomainParticipant> participant_;
    std::vector<TopicPtr> topics_;
};

class DDSContextProvider
{
  public:
    DDSContextProvider(const DDSContextProvider&) = delete;
    DDSContextProvider& operator=(const DDSContextProvider&) = delete;

    static void SetName(std::string name) { name_ = std::move(name); }

    static DDSContext& Get()
    {
        using Deleter = std::function<void(DDSContext*)>;

        static std::unique_ptr<DDSContext, Deleter> instance = []()
        {
            return std::unique_ptr<DDSContext, Deleter>(
                new DDSContext(name_),
                [](DDSContext* ptr)
                {
                    if (ptr)
                    {
                        delete ptr;
                    }
                });
        }();

        return *instance;
    }

  private:
    DDSContextProvider() = default;

    static inline std::string name_ = "MockaMammT";
};
}  // namespace core::communication

#endif  // CORE_COMMUNICATION_DDS_CONTEXT
