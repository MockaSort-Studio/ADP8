#ifndef CORE_SUPPORT_UTILS_LOOKUP_TABLE
#define CORE_SUPPORT_UTILS_LOOKUP_TABLE

#include <tuple>
#include <type_traits>
#include <utility>

namespace core::utils {

template <typename T>
struct single_type_extractor;

template <typename T>
struct single_type_extractor<std::tuple<T>>
{
    using type = T;
};

template <typename... Ts>
struct single_type_extractor<std::tuple<Ts...>>
{
    using type = std::tuple<Ts...>;
};

template <typename KeyType, typename... ValueTypes>
struct TableItem
{
    using Key = KeyType;
    using Values = std::tuple<ValueTypes...>;

    template <typename Func>
    static constexpr void for_each_in_table_element(Func&& func)
    {
        std::apply(
            [&func](auto&&... args) { (func(std::forward<decltype(args)>(args)), ...); },
            Values {});
    }
};

template <typename... TableElements>
struct LookupTable
{
    template <typename TargetKey>
    static constexpr std::size_t count_key_v =
        (0 + ... + (std::is_same_v<TargetKey, typename TableElements::Key> ? 1 : 0));

    static_assert(
        ((count_key_v<typename TableElements::Key> == 1) && ...),
        "LookupTable Error: Duplicate keys detected! Keys must be unique.");
    // ----------------------------------------------

    template <typename Key>
    struct get_type
    {
        template <typename Element>
        using extract_match = std::conditional_t<
            std::is_same_v<typename Element::Key, Key>,
            typename Element::Values,
            std::tuple<>>;

        using matched_tuple =
            decltype(std::tuple_cat(std::declval<extract_match<TableElements>>()...));

        using type = typename single_type_extractor<matched_tuple>::type;
    };

    template <typename Func>
    static constexpr void for_each_element(Func&& func)
    {
        (func(TableElements {}), ...);
    }
};

template <typename T>
constexpr bool is_lookup_table_v = false;

template <typename... Args>
constexpr bool is_lookup_table_v<LookupTable<Args...>> = true;

}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_LOOKUP_TABLE
