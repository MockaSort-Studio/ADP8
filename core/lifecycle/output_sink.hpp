#ifndef CORE_LIFECYCLE_OUTPUT_SINK
#define CORE_LIFECYCLE_OUTPUT_SINK

#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/data_endpoint.hpp"

namespace core::lifecycle {

/// @brief Write-only view over an output @c DataEndpoint.
///
/// Returned by @c DDSTask::GetOutputSink(). Accepts one sample per step via
/// @c Push(); the endpoint publishes it on the next @c Sync(). Does not own
/// the endpoint.
///
/// @tparam Spec @c TopicSpec specialization. Determines the data type.
template <typename Spec>
struct OutputSink {
  static_assert(communication::is_topic_spec_v<Spec>,
                "Spec must be specialization of TopicSpec");

  using T = typename Spec::type;
  DataEndpoint<Spec, DataDirection::Out>& endpoint;

  /// @brief Enqueues @p data for publishing on the next sync cycle.
  template <typename U>
  inline void Push(U&& data) {
    endpoint.Push(std::forward<U>(data));
  }
};

template <typename Spec>
OutputSink(DataEndpoint<Spec, DataDirection::Out>&) -> OutputSink<Spec>;
}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_OUTPUT_SINK
