#ifndef CORE_COMMUNICATION_TOPIC_SPEC
#define CORE_COMMUNICATION_TOPIC_SPEC

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <fastdds/dds/topic/TypeSupport.hpp>

namespace core::communication {

namespace dds = eprosima::fastdds::dds;

template <typename T, const char* TopicName, size_t max_queue_size = 1>
struct TopicSpec
{
    static_assert(
        std::is_base_of_v<dds::TopicDataType, T>,
        "T must be derived from eprosima::fastdds::dds::TopicDataType");

    using type = T;
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

}  // namespace core::communication

#endif  // CORE_COMMUNICATION_TOPIC_SPEC
