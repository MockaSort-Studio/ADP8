#ifndef CORE_LIFECYCLE_OUTPUT
#define CORE_LIFECYCLE_OUTPUT

#include "core/communication/dds_publisher.hpp"
#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/type_traits.hpp"

namespace core::lifecycle {

template <typename Spec>
class Output
{
    static_assert(
        communication::is_topic_spec_v<Spec>, "Spec must be specialization of TopicSpec");

  public:
    using Pub = communication::DDSPublisher<typename Spec::type>;
    using T = typename Pub::DDSDataType;
    static constexpr uint64_t kHash = Hash(Spec::kName);

    Output() { pub_.Start(Spec::kName); }
    void Flush() { pub_.Publish(last_val_); }
    void Set(T&& msg) { last_val_ = std::move(msg); }

  private:
    T last_val_;
    Pub pub_;
};

template <typename T>
struct Outputs;

template <typename... Specs>
struct Outputs<std::tuple<Specs...>>
{
    using type = std::tuple<Output<Specs>...>;
};

template <typename T>
using Outputs_t = typename Outputs<T>::type;

template <typename... Topics>
using TopicList = std::tuple<Topics...>;

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_OUTPUT
