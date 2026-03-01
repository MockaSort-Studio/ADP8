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
    Values values;

    template <typename Func>
    static constexpr void for_each_in_table_element(Func&& func)
    {
        std::apply(
            [&func](auto&&... args) { (func(std::forward<decltype(args)>(args)), ...); },
            Values {});
    }
};

// Trait to find the specific TableItem base class for a given Key
template <typename TargetKey, typename... Items>
struct find_table_item;

template <typename TargetKey, typename First, typename... Rest>
struct find_table_item<TargetKey, First, Rest...> {
    using type = std::conditional_t<
        std::is_same_v<TargetKey, typename First::Key>,
        First,
        typename find_table_item<TargetKey, Rest...>::type
    >;
};

// Base case (if not found, this will cause a clean compile error on 'type')
template <typename TargetKey, typename Last>
struct find_table_item<TargetKey, Last> {
    using type = std::conditional_t<std::is_same_v<TargetKey, typename Last::Key>, Last, void>;
};

template <typename... TableElements>
struct LookupTable : public TableElements...
{
    // --- Your original consistency checks ---
    template <typename TargetKey>
    static constexpr std::size_t count_key_v =
        (0 + ... + (std::is_same_v<TargetKey, typename TableElements::Key> ? 1 : 0));

    static_assert(
        ((count_key_v<typename TableElements::Key> == 1) && ...),
        "LookupTable Error: Duplicate keys detected!");

template <typename DefaultsTuple>
    explicit constexpr LookupTable(DefaultsTuple&& defaults)
        : LookupTable(std::forward<DefaultsTuple>(defaults), 
                      std::index_sequence_for<TableElements...>{}) {}

    template <typename Key>
    using get_values_t = typename find_table_item<Key, TableElements...>::type::Values;

    template <typename TargetKey>
    const auto& GetValue() const {
        using ItemBase = typename find_table_item<TargetKey, TableElements...>::type;
        return static_cast<const ItemBase&>(*this).values;
    }

    template <typename Func>
    static constexpr void for_each(Func&& func) {
        (func(TableElements {}), ...);    
    }
    private:
    // Internal helper that unrolls the indices
    template <typename DefaultsTuple, std::size_t... Is>
    constexpr LookupTable(DefaultsTuple&& defaults, std::index_sequence<Is...>)
        : TableElements{ {std::get<Is>(defaults)} }... {}

};

template <typename T>
constexpr bool is_lookup_table_v = false;

template <typename... Args>
constexpr bool is_lookup_table_v<LookupTable<Args...>> = true;

}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_LOOKUP_TABLE
