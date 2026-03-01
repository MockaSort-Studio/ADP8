#ifndef CORE_LIFECYCLE_PARAMETER_PROVIDER
#define CORE_LIFECYCLE_PARAMETER_PROVIDER

#include "core/support/utils/lookup_table.hpp"

namespace core::lifecycle {

namespace detail {
    template <typename T>
    constexpr auto collapse_tuple(T&& t) {
        constexpr std::size_t N = std::tuple_size_v<std::decay_t<T>>;
        
        if constexpr (N == 1) {
            return std::get<0>(std::forward<T>(t));
        } else {
            return std::apply([](auto&&... args) {
                using FirstType = std::tuple_element_t<0, std::decay_t<T>>;
                return std::array<FirstType, N>{std::forward<decltype(args)>(args)...};
            }, std::forward<T>(t));
        }
    }
}
//Check unit test for example usage 🐽
//core/lifecycle/test/parameters_provider_tests.cpp
template<typename Params, typename ParamsDefaults>
class ParametersProvider
{   
    public:

    template<typename ParameterTag>
    constexpr auto GetParameterValue() const
    {
        return detail::collapse_tuple(parameters_.GetValue(ParameterTag{}));
    }
    
    protected:
    //we don't want everyone to publically access set
    //setting it protected enable building parameter server which updates parameter on rpc
    template <typename ParameterTag, typename... Args>
    void SetParameterValue(Args&&... args) {
        parameters_.GetValue(ParameterTag{}).values = std::make_tuple(std::forward<Args>(args)...);
    }

    private:
    
    Params parameters_{ParamsDefaults::values};
};

}  // namespace core::lifecycle


#endif // CORE_LIFECYCLE_PARAMETER_PROVIDER
