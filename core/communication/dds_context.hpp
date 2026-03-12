#ifndef CORE_COMMUNICATION_DDS_CONTEXT
#define CORE_COMMUNICATION_DDS_CONTEXT

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <boost/core/demangle.hpp>

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
  /// @brief Constructs the context and creates a DomainParticipant with the given name.
  /// @param domain_participant_name Name assigned to the FastDDS DomainParticipant.
  /// @throws std::runtime_error if participant creation fails.
  DDSContext(const std::string& domain_participant_name) {
    dds::DomainParticipantQos participantQos;
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
  /// @tparam Spec A @c TopicSpec specialization. Derives topic name and PubSubType from it.
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

/// @brief Primary template: singleton @c DDSContext keyed by tag type.
///
/// The participant name is derived at construction from the demangled type name
/// of @p Tag. No @c SetName() needed — context identity is fully encoded in
/// the type. Non-instantiable — all access is through @c Get().
template <typename Tag>
class DDSContextProvider {
 public:
  DDSContextProvider(const DDSContextProvider&) = delete;
  DDSContextProvider& operator=(const DDSContextProvider&) = delete;

  /// @brief Returns the singleton @c DDSContext for @p Tag, constructing it on first call.
  static DDSContext& Get() {
    static DDSContext instance{boost::core::demangle(typeid(Tag).name())};
    return instance;
  }

 private:
  DDSContextProvider() = default;
};

/// @brief @c void specialization: preserves the original @c SetName()/@c Get() singleton.
///
/// The instance is constructed on first call to @c Get(), using the name set
/// via @c SetName(). Call @c SetName() once at startup before any pub/sub is
/// created. Non-instantiable — all access is through static methods.
template <>
class DDSContextProvider<void> {
 public:
  DDSContextProvider(const DDSContextProvider&) = delete;
  DDSContextProvider& operator=(const DDSContextProvider&) = delete;

  /// @brief Sets the DomainParticipant name used when the singleton is first constructed.
  ///        Must be called before the first @c Get() invocation. No effect after construction.
  static void SetName(std::string name) { name_ = std::move(name); }

  /// @brief Returns the singleton @c DDSContext, constructing it on first call.
  static DDSContext& Get() {
    using Deleter = std::function<void(DDSContext*)>;

    static std::unique_ptr<DDSContext, Deleter> instance = []() {
      return std::unique_ptr<DDSContext, Deleter>(new DDSContext(name_),
                                                  [](DDSContext* ptr) {
                                                    if (ptr) {
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
