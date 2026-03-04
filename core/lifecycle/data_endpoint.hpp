#ifndef CORE_LIFECYCLE_DATA_ENDPOINT
#define CORE_LIFECYCLE_DATA_ENDPOINT

#include "core/communication/dds_publisher.hpp"
#include "core/communication/dds_subscriber.hpp"
#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/type_traits.hpp"
#include "core/support/utils/size_constrained_queue.hpp"
// this should be moved in communication
namespace core::lifecycle {

/// @brief Direction tag for @c DataEndpoint. @c In = subscriber, @c Out = publisher.
enum class DataDirection { In, Out };

/// @brief Generic DDS endpoint: subscriber (@c In) or publisher (@c Out) for one topic.
///
/// Uses @c if constexpr to instantiate only the relevant DDS primitive — the other
/// member collapses to @c int. Owns the sample queue.
///
/// @c Sync() drives data transfer each step:
/// - @c In: drains samples from the DDS listener queue into the local buffer.
/// - @c Out: publishes the most recent queued sample if one is pending.
///
/// @tparam Spec A @c TopicSpec specialization defining type, name, and queue size.
/// @tparam D    @c DataDirection::In (subscriber) or @c DataDirection::Out (publisher).
template <typename Spec, DataDirection D>
class DataEndpoint {
  static_assert(communication::is_topic_spec_v<Spec>,
                "Spec must be specialization of TopicSpec");
  static_assert(D != DataDirection::Out || Spec::kQueueSize == 1,
                "Output Endpoints (Publishers) are currently restricted to a "
                "queue size of 1 "
                "to ensure latest-sample-only semantics.");

 public:
  static constexpr size_t kQueueSize = Spec::kQueueSize;
  static constexpr uint64_t kHash = Hash(Spec::kName);
  using T = typename Spec::type::type;

  DataEndpoint() {
    if constexpr (D == DataDirection::In) {
      sub_.Start(Spec::kName);
    } else {
      pub_.Start(Spec::kName);
    }
  }
  /// @brief Synchronizes the endpoint with DDS.
  ///        In: drains the listener queue into the local sample buffer.
  ///        Out: publishes the latest queued sample (if any).
  void Sync() noexcept {
    if constexpr (D == DataDirection::In) {
      sub_.DrainQueue(queue_);
    } else {
      if (!queue_.Empty()) {
        pub_.Publish(queue_[0].data);
      }
    }
  }
  /**
   * @brief Access samples in reverse-chronological order.
   * [0] is the Newest, [Size-1] is the Oldest.
   */
  inline const auto& operator[](size_t index) const noexcept {
    static_assert(D == DataDirection::In, "Cannot read from an Output Sink");
    return queue_[index];
  }

  /// @brief Enqueues @p data for publishing on the next @c Sync().
  ///        Only valid for @c DataDirection::Out endpoints.
  void Push(T&& data) {
    static_assert(D == DataDirection::Out, "Cannot push to an Input Source");
    queue_.Push(std::move(data));
  }

  [[nodiscard]] inline size_t Size() const noexcept { return queue_.Size(); }
  [[nodiscard]] inline bool Empty() const noexcept { return queue_.Empty(); }

  /**
   * @brief Access latest sample in queue.
   * This is thread safe.
   */
  std::optional<utils::Sample<T>> Get() {
    static_assert(D == DataDirection::In, "Cannot read from an Output Sink");
    return queue_.GetSample();
  }

 private:
  utils::SizeConstrainedQueue<T, kQueueSize> queue_;

  // Using std::conditional to only instantiate what we need
  std::conditional_t<
      D == DataDirection::In,
      communication::DDSSubscriber<typename Spec::type, kQueueSize>, int>
      sub_;

  std::conditional_t<D == DataDirection::Out,
                     communication::DDSPublisher<typename Spec::type>, int>
      pub_;
};

template <typename T>
struct Inputs;

template <typename... Specs>
struct Inputs<std::tuple<Specs...>> {
  using type = std::tuple<DataEndpoint<Specs, DataDirection::In>...>;
};

template <typename T>
using Inputs_t = typename Inputs<T>::type;

template <typename T>
struct Outputs;

template <typename... Specs>
struct Outputs<std::tuple<Specs...>> {
  using type = std::tuple<DataEndpoint<Specs, DataDirection::Out>...>;
};

template <typename T>
using Outputs_t = typename Outputs<T>::type;
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DATA_ENDPOINT
