#ifndef CORE_COMMUNICATION_TOPIC_SPEC
#define CORE_COMMUNICATION_TOPIC_SPEC

#include <cstddef>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <type_traits>

namespace core::communication {

namespace dds = eprosima::fastdds::dds;

/// @brief Compile-time descriptor binding a FastDDS PubSubType to a named topic and queue depth.
///
/// Pass specializations of this struct as template arguments to @c DataEndpoint,
/// @c DDSPublisher, and @c DDSSubscriber. All three parameters are resolved at
/// compile time — no runtime overhead for topic lookup.
///
/// @tparam T              FastDDS PubSubType. Must derive from @c TopicDataType.
///                        The inner @c ::type alias gives the message payload type.
/// @tparam TopicName      Null-terminated string literal identifying the DDS topic.
/// @tparam max_queue_size Maximum samples buffered in the subscriber queue (default: 1).
template <typename T, const char* TopicName, size_t max_queue_size = 1>
struct TopicSpec {
  static_assert(std::is_base_of_v<dds::TopicDataType, T>,
                "T must be derived from eprosima::fastdds::dds::TopicDataType");

  using type = T;
  static constexpr const char* kName{TopicName};
  static constexpr size_t kQueueSize{max_queue_size};
};

/// @brief Detects whether @p T is a @c TopicSpec specialization.
/// @tparam T Type to test. Evaluates to @c std::true_type for any @c TopicSpec<...>.
template <typename T>
struct is_topic_spec : std::false_type {};

template <typename T, const char* TopicName, size_t Q>
struct is_topic_spec<TopicSpec<T, TopicName, Q>> : std::true_type {};

/// @brief True iff @p T is a @c TopicSpec specialization.
template <typename T>
inline constexpr bool is_topic_spec_v = is_topic_spec<T>::value;

}  // namespace core::communication

#endif  // CORE_COMMUNICATION_TOPIC_SPEC
