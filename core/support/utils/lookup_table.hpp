#ifndef CORE_SUPPORT_UTILS_LOOKUP_TABLE
#define CORE_SUPPORT_UTILS_LOOKUP_TABLE

#include <stdexcept>
#include <tuple>

namespace core::utils {

template <bool...>
struct bool_pack;

template <std::size_t I, bool B, bool... Bs>
struct find_true : std::conditional_t<
                       B,
                       std::integral_constant<std::size_t, I>,
                       find_true<I + 1, Bs...>>
{
};

template <std::size_t I, bool B>
struct find_true<I, B> : std::integral_constant<
                             std::size_t,
                             B ? I : throw std::logic_error("No true value found")>
{
};

struct UnusedValue;

template <typename KeyType, typename... ValueTypes>
struct TableItem
{
    using Key = KeyType;
    using Values = std::tuple<ValueTypes...>;

    template <typename Func, std::size_t... Is>
    static constexpr void for_each_in_table_element_impl(
        Func&& func, std::index_sequence<Is...>)
    {
        (func(std::get<Is>(Values {})), ...);
    }

    template <typename Func>
    static constexpr void for_each_in_table_element(Func&& func)
    {
        for_each_in_table_element_impl(
            std::forward<Func>(func),
            std::make_index_sequence<std::tuple_size_v<Values>> {});
    }
};

template <typename T>
struct single_type_extractor;

// Specialization for tuple with one type
template <typename T>
struct single_type_extractor<std::tuple<T>>
{
    using type = T;
};

// General case for tuple
template <typename... Ts>
struct single_type_extractor<std::tuple<Ts...>>
{
    using type = std::tuple<Ts...>;
};

template <typename... TableElements>
struct LookupTable
{
    template <typename Key>
    struct get_type
    {
        template <typename Pair>
        struct is_key
        {
            static constexpr bool value = std::is_same_v<typename Pair::Key, Key>;
        };

        template <std::size_t... Is>
        static auto find_type(std::index_sequence<Is...>) ->
            typename single_type_extractor<decltype(std::tuple_cat(
                std::conditional_t<
                    is_key<std::tuple_element_t<Is, std::tuple<TableElements...>>>::value,
                    typename std::tuple_element_t<Is, std::tuple<TableElements...>>::
                        Values,
                    std::tuple<>>()...))>::type;

        using type = decltype(find_type(std::index_sequence_for<TableElements...> {}));
    };

    template <typename Func>
    static constexpr void for_each_element(Func&& func)
    {
        (func(TableElements {}), ...);
    }
};
}  // namespace core::utils

#endif  // CORE_SUPPORT_UTILS_LOOKUP_TABLE
