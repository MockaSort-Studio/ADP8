#ifndef CORE_LIFECYCLE_PARAMETERS_PROVIDER
#define CORE_LIFECYCLE_PARAMETERS_PROVIDER

#include <tuple>
#include <type_traits>
#include <utility>
namespace core::lifecycle {

namespace detail {
template <typename T>
constexpr auto collapse_tuple(T&& t) {
  constexpr std::size_t N = std::tuple_size_v<std::decay_t<T>>;

  if constexpr (N == 1) {
    return std::get<0>(std::forward<T>(t));
  } else {
    return std::apply(
        [](auto&&... args) {
          using FirstType = std::tuple_element_t<0, std::decay_t<T>>;
          return std::array<FirstType, N>{
              std::forward<decltype(args)>(args)...};
        },
        std::forward<T>(t));
  }
}
}  // namespace detail
/// @brief Provides typed read/write access to a @c LookupTable of parameters.
///
/// @p Params is a @c LookupTable type. @p ParamsDefaults is a tuple of default
/// value tuples, one per entry. Single-value entries are returned as the scalar
/// directly; multi-value entries collapse into a @c std::array.
///
/// @see core/lifecycle/test/parameters_provider_tests.cpp for usage examples.
///
/// @tparam Params         @c LookupTable type defining the parameter schema.
/// @tparam ParamsDefaults Tuple of default value tuples (order matches @p Params).
// proudly AI-generated, human-reviewed
template <typename Params, typename ParamsDefaults>
class ParametersProvider {
 public:
  /// @brief Returns the current value for @p ParameterTag.
  ///        Single-value entries return the scalar; multi-value entries return a @c std::array.
  /// @tparam ParameterTag Tag type identifying the parameter in @p Params.
  template <typename ParameterTag>
  constexpr auto GetParameterValue() const {
    return detail::collapse_tuple(parameters_.GetValue(ParameterTag{}));
  }

 protected:
  // Protected to allow derived classes (e.g. a parameter server) to update values.
  /// @brief Updates the value for @p ParameterTag.
  /// @tparam ParameterTag Tag type identifying the parameter in @p Params.
  /// @param  args         New values, forwarded into the parameter tuple.
  template <typename ParameterTag, typename... Args>
  void SetParameterValue(Args&&... args) {
    parameters_.GetValue(ParameterTag{}).values =
        std::make_tuple(std::forward<Args>(args)...);
  }

 private:
  Params parameters_{ParamsDefaults::values};
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_PARAMETERS_PROVIDER
