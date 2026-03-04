#ifndef CORE_LIFECYCLE_TYPE_TRAITS
#define CORE_LIFECYCLE_TYPE_TRAITS

#include <cstddef>
#include <cstdint>
#include <tuple>

namespace core::lifecycle {

/// @brief FNV-1a 64-bit hash of a null-terminated string.
///        Evaluated at compile time when passed a string literal.
/// @param str Null-terminated string to hash.
/// @return 64-bit FNV-1a hash value.
constexpr uint64_t Hash(const char* str) {
  uint64_t h = 0xcbf29ce484222325;
  while (*str) {
    h = (h ^ static_cast<uint64_t>(*str++)) * 0x100000001b3;
  }
  return h;
}

/// @brief Implementation detail: linear scan over a tuple's element types,
///        returning the index whose @c kHash equals @p Hash.
template <uint64_t Hash, typename Tuple, typename Seq>
struct find_hash_impl;

template <uint64_t Hash, typename Tuple, size_t... Is>
struct find_hash_impl<Hash, Tuple, std::index_sequence<Is...>> {
  static constexpr size_t get_index() {
    auto found = (size_t)-1;
    std::ignore =
        (((std::tuple_element_t<Is, Tuple>::kHash == Hash) ? (found = Is, true)
                                                           : false) ||
         ...);
    return found;
  }
  static constexpr size_t value = get_index();
};

/// @brief Finds the index of the @c DataEndpoint in @p Tuple whose @c kHash equals @p TargetHash.
///        Produces a compile error if not found.
/// @tparam TargetHash Hash value to search for. Use @c Hash(TopicName) to compute.
/// @tparam Tuple      @c std::tuple of @c DataEndpoint specializations.
template <uint64_t TargetHash, typename Tuple>
struct find_by_hash {
  static constexpr size_t value =
      find_hash_impl<TargetHash, Tuple,
                     std::make_index_sequence<std::tuple_size_v<Tuple>>>::value;

  static_assert(value != (size_t)-1,
                "Topic Name not found in the provided Spec List!");
};

/// @brief Tag-dispatched accessor: returns the @c DataEndpoint in @p storage
///        whose @c TopicSpec::kName matches @p TopicName.
///        Fails to compile if the name is not found.
/// @tparam TopicName Compile-time string literal matching a @c TopicSpec::kName.
template <const char* TopicName, typename Tuple>
auto& get(Tuple& storage) {
  using T = std::decay_t<Tuple>;

  constexpr size_t I = find_by_hash<Hash(TopicName), T>::value;

  return std::get<I>(storage);
}

/// @brief Typed list of @c TopicSpec entries.
///        Passed as @c SubscriptionSpecs / @c PublicationSpecs to @c DDSTask.
template <typename... Topics>
using TopicList = std::tuple<Topics...>;

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_TYPE_TRAITS
