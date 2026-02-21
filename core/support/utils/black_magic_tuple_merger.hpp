#ifndef CORE_SUPPORT_UTILS_BLACK_MAGIC_TUPLE_MERGER
#define CORE_SUPPORT_UTILS_BLACK_MAGIC_TUPLE_MERGER

#include <tuple>
#include <type_traits>

namespace core::utils {

template <typename T, typename... Us>
struct contains : std::disjunction<std::is_same<T, Us>...>
{
};

template <typename Input, typename Output>
struct filter_duplicates;

template <typename... Os>
struct filter_duplicates<std::tuple<>, std::tuple<Os...>>
{
    using type = std::tuple<Os...>;
};

template <typename T, typename... Is, typename... Os>
struct filter_duplicates<std::tuple<T, Is...>, std::tuple<Os...>>
{
    using type = typename std::conditional_t<
        contains<T, Os...>::value,
        filter_duplicates<std::tuple<Is...>, std::tuple<Os...>>,    // Skip T
        filter_duplicates<std::tuple<Is...>, std::tuple<Os..., T>>  // Add T
        >::type;
};

template <typename... Tuples>
struct BlackMagicTupleMerger
{
    static_assert(
        sizeof...(Tuples) > 1,
        "\n\n[KITEMMURT] Why are you wasting my time with one tuple? \n"
        "It's a pointless use of compilation time to merge one tuple! \n"
        "May a bolt of celestial fire cleave a parting through thy mane! \n");

    using flattened = decltype(std::tuple_cat(std::declval<Tuples>()...));

    using type = typename filter_duplicates<flattened, std::tuple<>>::type;
};
}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_BLACK_MAGIC_TUPLE_MERGER
