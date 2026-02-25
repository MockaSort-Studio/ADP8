#ifndef CORE_LIFECYCLE_INPUT
#define CORE_LIFECYCLE_INPUT

#include <sys/types.h>

#include "core/communication/dds_subscriber.hpp"
#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/type_traits.hpp"
#include "core/support/utils/size_constrained_queue.hpp"
namespace core::lifecycle {

template <typename Spec>
class Input
{
    static_assert(
        communication::is_topic_spec_v<Spec>, "Spec must be specialization of TopicSpec");

  public:
    static constexpr size_t kQueueSize = Spec::kQueueSize;
    static constexpr uint64_t kHash = Hash(Spec::kName);
    using Sub = communication::DDSSubscriber<typename Spec::type, kQueueSize>;
    using T = typename Sub::DDSDataType;

    Input() { sub_.Start(Spec::kName); }
    void Fill() {}
    // void Push(T&& msg) { queue_.Push(std::move(msg)); }

    std::optional<utils::Sample<T>> Get() { return queue_.GetSample(); }

  private:
    utils::SizeConstrainedQueue<T, kQueueSize> queue_;
    Sub sub_;
};

template <typename T>
struct Inputs;

template <typename... Specs>
struct Inputs<std::tuple<Specs...>>
{
    using type = std::tuple<Input<Specs>...>;
};

template <typename T>
using Inputs_t = typename Inputs<T>::type;
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_INPUT
