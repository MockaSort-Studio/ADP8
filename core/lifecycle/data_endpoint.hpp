#ifndef CORE_LIFECYCLE_DATA_ENDPOINT
#define CORE_LIFECYCLE_DATA_ENDPOINT

#include "core/communication/dds_publisher.hpp"
#include "core/communication/dds_subscriber.hpp"
#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/type_traits.hpp"
#include "core/support/utils/size_constrained_queue.hpp"

namespace core::lifecycle {
enum class DataDirection
{
    In,
    Out
};
template <typename Spec, DataDirection D>
class DataEndpoint
{
    static_assert(
        communication::is_topic_spec_v<Spec>, "Spec must be specialization of TopicSpec");
    static_assert(
        D != DataDirection::Out || Spec::kQueueSize == 1,
        "Output Endpoints (Publishers) are currently restricted to a queue size of 1 "
        "to ensure latest-sample-only semantics.");

  public:
    static constexpr size_t kQueueSize = Spec::kQueueSize;
    static constexpr uint64_t kHash = Hash(Spec::kName);
    using T = typename Spec::type::type;

    DataEndpoint()
    {
        if constexpr (D == DataDirection::In)
        {
            sub_.Start(Spec::kName);
        } else
        {
            pub_.Start(Spec::kName);
        }
    }
    void Sync() noexcept
    {
        if constexpr (D == DataDirection::In)
        {
            sub_.DrainQueue(queue_);
        } else
        {
            if (!queue_.Empty())
            {
                pub_.Publish(queue_[0].data);
            }
        }
    }
    /**
     * @brief Access samples in reverse-chronological order.
     * [0] is the Newest, [Size-1] is the Oldest.
     */
    inline const auto& operator[](size_t index) const noexcept
    {
        static_assert(D == DataDirection::In, "Cannot read from an Output Sink");
        return queue_[index];
    }

    void Push(T&& data)
    {
        static_assert(D == DataDirection::Out, "Cannot push to an Input Source");
        queue_.Push(std::move(data));
    }

    [[nodiscard]] inline size_t Size() const noexcept { return queue_.Size(); }
    [[nodiscard]] inline bool Empty() const noexcept { return queue_.Empty(); }

    /**
     * @brief Access latest sample in queue.
     * This is thread safe.
     */
    std::optional<utils::Sample<T>> Get()
    {
        static_assert(D == DataDirection::In, "Cannot read from an Output Sink");
        return queue_.GetSample();
    }

  private:
    utils::SizeConstrainedQueue<T, kQueueSize> queue_;

    // Using std::conditional to only instantiate what we need
    std::conditional_t<
        D == DataDirection::In,
        communication::DDSSubscriber<typename Spec::type, kQueueSize>,
        int>
        sub_;

    std::conditional_t<
        D == DataDirection::Out,
        communication::DDSPublisher<typename Spec::type>,
        int>
        pub_;
};

template <typename T>
struct Inputs;

template <typename... Specs>
struct Inputs<std::tuple<Specs...>>
{
    using type = std::tuple<DataEndpoint<Specs, DataDirection::In>...>;
};

template <typename T>
using Inputs_t = typename Inputs<T>::type;

template <typename T>
struct Outputs;

template <typename... Specs>
struct Outputs<std::tuple<Specs...>>
{
    using type = std::tuple<DataEndpoint<Specs, DataDirection::Out>...>;
};

template <typename T>
using Outputs_t = typename Outputs<T>::type;
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DATA_ENDPOINT
