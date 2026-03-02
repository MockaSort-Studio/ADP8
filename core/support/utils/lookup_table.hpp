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
    // candidate for pruning
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
struct find_table_item<TargetKey, First, Rest...>
{
    using type = std::conditional_t<
        std::is_same_v<TargetKey, typename First::Key>,
        First,
        typename find_table_item<TargetKey, Rest...>::type>;
};
template <typename TargetKey, typename Last>
struct find_table_item<TargetKey, Last>
{
    using type =
        std::conditional_t<std::is_same_v<TargetKey, typename Last::Key>, Last, void>;
};

// Below: Helpers to enable ordered independent (key-val) initialization of LookupTable
// using tag dispatching use it for complex cases/initialization
template <typename Tag, typename... ValueTypes>
struct Init
{
    using Key = Tag;
    std::tuple<ValueTypes...> values;

    constexpr Init(Tag, ValueTypes... args)
        : values(std::make_tuple(std::forward<ValueTypes>(args)...))
    {}
};

template <typename TargetTag, typename First, typename... Rest>
constexpr const auto& find_val(const First& first, const Rest&... rest)
{
    if constexpr (std::is_same_v<TargetTag, typename First::Key>)
    {
        return first.values;
    } else
    {
        return find_val<TargetTag>(rest...);
    }
}

template <typename Table, typename... Inits>
constexpr auto TableDefaults(const Inits&... inits)
{
    return Table::SetDefaults(inits...);
}

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

    explicit constexpr LookupTable() = default;

    // Init table with a tuple of tuple, use it for simple cases and keep in mind
    // that the order of default values must match the order of declaration for the table
    // items
    template <typename DefaultsTuple>
    explicit constexpr LookupTable(DefaultsTuple&& defaults)
        : LookupTable(
              std::forward<DefaultsTuple>(defaults),
              std::index_sequence_for<TableElements...> {})
    {}

    // Use this for complex cases, where the number of entry is > 10 and your types are
    // heavely using templates tag dispatchig saves us from the ::template syntax.
    template <typename... Inits>
    static constexpr auto SetDefaults(const Inits&... inits)
    {
        return std::make_tuple(find_val<typename TableElements::Key>(inits...)...);
    }
    template <typename Key>
    using get_values_t = typename find_table_item<Key, TableElements...>::type::Values;

    template <typename TargetKey>
    constexpr const auto& GetValue() const
    {
        using ItemBase = typename find_table_item<TargetKey, TableElements...>::type;
        return static_cast<const ItemBase&>(*this).values;
    }

    // Sometimes we end up in dependent template types
    // this is a small helper to enable tag dispatching and auto parameter deduction
    // so we avoid stupid syntax table.template GetValue<Tag>() oink oink 🐽
    template <typename Tag>
    constexpr auto& GetValue(Tag)
    {
        return GetValue<Tag>();
    }

    template <typename Tag>
    constexpr const auto& GetValue(Tag) const
    {
        return GetValue<Tag>();
    }

    template <typename Func>
    static constexpr void for_each(Func&& func)
    {
        (func(TableElements {}), ...);
    }

  private:
    // Internal helper that unrolls the indices
    template <typename DefaultsTuple, std::size_t... Is>
    constexpr LookupTable(DefaultsTuple&& defaults, std::index_sequence<Is...>)
        : TableElements {{std::get<Is>(defaults)}}...
    {}
};

template <typename T>
constexpr bool is_lookup_table_v = false;

template <typename... Args>
constexpr bool is_lookup_table_v<LookupTable<Args...>> = true;

}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_LOOKUP_TABLE
