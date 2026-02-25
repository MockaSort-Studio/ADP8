#ifndef CORE_LIFECYCLE_INPUT_SOURCE
#define CORE_LIFECYCLE_INPUT_SOURCE

#include "core/communication/topic_spec.hpp"
#include "core/lifecycle/data_endpoint.hpp"
#include "core/support/utils/size_constrained_queue.hpp"

namespace core::lifecycle {

/**
 * @brief Read-Only View for Input DataEndpoints.
 */
template <typename Spec>
struct InputSource
{
    static_assert(
        communication::is_topic_spec_v<Spec>, "Spec must be specialization of TopicSpec");

    using T = typename Spec::type::type;
    const DataEndpoint<Spec, DataDirection::In>& endpoint;

    [[nodiscard]] inline const utils::Sample<T>& operator[](size_t i) const noexcept
    {
        return endpoint[i];
    }

    [[nodiscard]] inline size_t Size() const noexcept { return endpoint.Size(); }
    [[nodiscard]] inline bool Empty() const noexcept { return endpoint.Empty(); }
};

template <typename Spec>
InputSource(const DataEndpoint<Spec, DataDirection::In>&) -> InputSource<Spec>;

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_INPUT_SOURCE
