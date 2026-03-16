#ifndef CORE_COMMUNICATION_DDS_CONTEXT
#define CORE_COMMUNICATION_DDS_CONTEXT

#include <boost/core/demangle.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "core/communication/topic_spec.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"
namespace core::communication {

namespace dds = eprosima::fastdds::dds;

/// @brief Owns a FastDDS DomainParticipant and the topics registered to it.
///
/// Topics are created on first request and reused on subsequent calls with the
/// same name. Participant and topic lifetime is managed through RAII deleters.
/// Non-copyable; move-constructible.
class DDSContext {
 public:
  /// @brief Constructs the context and creates a DomainParticipant with the
  /// given name.
  /// @param domain_participant_name Name assigned to the FastDDS
  /// DomainParticipant.
  /// @throws std::runtime_error if participant creation fails.
  DDSContext(const std::string& domain_participant_name) {
    // Start from the factory's default QoS so that any XML profile loaded via
    // FASTRTPS_DEFAULT_PROFILES_FILE is honoured (e.g. custom transports for
    // simulation). A manually constructed DomainParticipantQos bypasses the
    // XML default profile entirely.
    dds::DomainParticipantQos participantQos;
    dds::DomainParticipantFactory::get_instance()->get_default_participant_qos(
        participantQos);
    participantQos.name(domain_participant_name);

    auto* raw_part =
        dds::DomainParticipantFactory::get_instance()->create_participant(
            0, participantQos);

    if (!raw_part)
      throw std::runtime_error("FastDDS failed to create DomainParticipant");

    participant_ = std::shared_ptr<dds::DomainParticipant>(
        raw_part, [](dds::DomainParticipant* p) {
          if (p) {
            dds::DomainParticipantFactory::get_instance()->delete_participant(
                p);
          }
        });
  }

  ~DDSContext() = default;

  /// @brief Get or create a topic from a @c TopicSpec.
  /// @tparam Spec A @c TopicSpec specialization. Derives topic name and
  /// PubSubType from it.
  /// @return Pointer to the FastDDS topic. Owned by this context.
  template <typename Spec>
  dds::Topic* GetDDSTopic() {
    static_assert(is_topic_spec_v<Spec>,
                  "Spec must be specialization of TopicSpec");

    return GetDDSTopic<typename Spec::type>(Spec::kName);
  }

  /// @brief Get or create a topic by PubSubType and name.
  ///
  /// Registers the type on first call, then creates the DDS topic.
  /// Returns the existing topic if one with @p topic_name already exists.
  ///
  /// @tparam PubSubType FastDDS PubSubType. Must derive from @c TopicDataType.
  /// @param  topic_name DDS topic name string.
  /// @return Pointer to the FastDDS topic. Owned by this context.
  /// @throws std::runtime_error if topic creation fails.
  template <typename PubSubType>
  dds::Topic* GetDDSTopic(const std::string& topic_name) {
    static_assert(std::is_base_of_v<dds::TopicDataType, PubSubType>,
                  "PubSubType must be derived from "
                  "eprosima::fastdds::dds::TopicDataType");

    auto it = std::find_if(
        topics_.begin(), topics_.end(),
        [&](const TopicPtr& t) { return t->get_name() == topic_name; });

    if (it != topics_.end()) return it->get();

    dds::TypeSupport type_support(new PubSubType());
    type_support.register_type(participant_.get());

    auto* raw_topic = participant_->create_topic(
        topic_name, type_support.get_type_name(), dds::TOPIC_QOS_DEFAULT);

    if (!raw_topic) throw std::runtime_error("Failed to create topic");

    topics_.emplace_back(raw_topic, [part = participant_](dds::Topic* t) {
      if (part && t) part->delete_topic(t);
    });

    return raw_topic;
  }

  /// @brief Returns the shared domain participant.
  ///        Callers may hold the returned @c shared_ptr to extend participant
  ///        lifetime beyond this context.
  std::shared_ptr<dds::DomainParticipant> GetDomainParticipant() {
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

/// @brief Tag type for the default (process-wide) @c DDSContext.
struct DefaultContext {};

/// @brief Singleton @c DDSContext keyed by tag type.
///
/// The participant name is derived at construction from the demangled type name
/// of @p Tag. Non-instantiable — all access is through @c Get().
template <typename Tag = DefaultContext>
class DDSContextProvider {
 public:
  DDSContextProvider(const DDSContextProvider&) = delete;
  DDSContextProvider& operator=(const DDSContextProvider&) = delete;

  /// @brief Returns the singleton @c DDSContext for @p Tag, constructing it on
  /// first call.
  static DDSContext& Get() {
    static DDSContext instance{boost::core::demangle(typeid(Tag).name())};
    return instance;
  }

 private:
  DDSContextProvider() = default;
};
}  // namespace core::communication

#endif  // CORE_COMMUNICATION_DDS_CONTEXT
