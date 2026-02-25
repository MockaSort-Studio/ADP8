#ifndef CORE_LIFECYCLE_TYPE_TRAITS
#define CORE_LIFECYCLE_TYPE_TRAITS

#include <cstddef>
#include <cstdint>
#include <tuple>

namespace core::lifecycle {

constexpr uint64_t Hash(const char* str)
{
    uint64_t h = 0xcbf29ce484222325;
    while (*str)
    {
        h = (h ^ static_cast<uint64_t>(*str++)) * 0x100000001b3;
    }
    return h;
}

// Helper for finding the index of Input/Output by hash in a tuple
// Input core/lifecycle/input.hpp
// Output core/lifecycle/output.hpp
template <uint64_t Hash, typename Tuple, typename Seq>
struct find_hash_impl;

template <uint64_t Hash, typename Tuple, size_t... Is>
struct find_hash_impl<Hash, Tuple, std::index_sequence<Is...>>
{
    static constexpr size_t get_index()
    {
        auto found = (size_t)-1;
        std::ignore =
            (((std::tuple_element_t<Is, Tuple>::kHash == Hash) ? (found = Is, true)
                                                               : false) ||
             ...);
        return found;
    }
    static constexpr size_t value = get_index();
};

template <uint64_t TargetHash, typename Tuple>
struct find_by_hash
{
    static constexpr size_t value = find_hash_impl<
        TargetHash,
        Tuple,
        std::make_index_sequence<std::tuple_size_v<Tuple>>>::value;

    static_assert(value != (size_t)-1, "Topic Name not found in the provided Spec List!");
};

template <const char* TopicName, typename Tuple>
auto& get(Tuple& storage)
{
    using T = std::decay_t<Tuple>;

    constexpr size_t I = find_by_hash<Hash(TopicName), T>::value;

    return std::get<I>(storage);
}

template <typename... Topics>
using TopicList = std::tuple<Topics...>;

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_TYPE_TRAITS
