#ifndef SUPPORT_VISIT_VISITOR_BASE
#define SUPPORT_VISIT_VISITOR_BASE

#include <stdexcept>
#include <string_view>
#include <type_traits>

namespace game::support
{

/**
 * \addtogroup support Support
 * \brief Supporting classes and functions
 * \ingroup support
 * @{
 */

/**
 * \addtogroup serialization Visitable and serialization utilities
 * \brief Supporting classes and functions for serializing, deserializing and visiting
 * objects
 * \ingroup support
 * @{
 */

namespace
{

template <typename T>
constexpr auto has_visit_nested(...) -> std::false_type;

template <typename T>
constexpr auto has_visit_nested(int)
    -> decltype(std::declval<T>().visit_nested(std::string_view {}), std::true_type {});

template <typename T>
constexpr auto has_exit_nested(...) -> std::false_type;

template <typename T>
constexpr auto has_exit_nested(int)
    -> decltype(std::declval<T>().exit_nested(std::string_view {}), std::true_type {});

}  // namespace

/**
 * @brief Checks if the derived visitor class has visit_nested member function.
 *
 * @tparam T The derived visitor class.
 * @return `true` if the derived visitor class has visit_nested member function,
 * `false` otherwise.
 */
template <typename T>
constexpr bool has_visit_nested_v = decltype(has_visit_nested<T>(0))::value;

/**
 * @brief Checks if the derived visitor class has exit_nested member function.
 *
 * @tparam T The derived visitor class.
 * @return `true` if the derived visitor class has exit_nested member function,
 * `false` otherwise.
 */
template <typename T>
constexpr bool has_exit_nested_v = decltype(has_exit_nested<T>(0))::value;

/**
 * @brief Base class for visitors. This class uses CRTP pattern to forward function
 * calls to the Derived class
 * @tparam Derived The derived visitor class.
 */
template <typename Derived>
class VisitorBase
{
    /**
     * @brief Visit a named value. This is called by the visitable everytime an element is
     * visited.
     * @tparam T The type of the value.
     * @param name The name of the visited element.
     * @param value The visited value.
     */
    template <typename T>
    void visit_base(std::string_view name, T& value)
    {
        derived_->visit(name, value);
    }

    template <typename T, typename... P>
    friend class Visitable;

    /**
     * @brief Enter a nested visitable object. This is called by the visitable everytime a
     * visitable object (or nested) is visited.
     * @param name The name of the nested object.
     * @return Implementation-defined.
     */
    void visit_nested_base(std::string_view name)
    {
        if constexpr (has_visit_nested_v<Derived>)
        {
            derived_->visit_nested(name);
        }
        else
        {
            throw std::runtime_error(
                "visit_nested is not a member function, please implement it in the "
                "derived class");
        }
    }

    /**
     * @brief Exit a nested visitable object. This is called by the visitable everytime a
     * visiting a visitable object (or nested) is over.
     * @param name The name of the nested object.
     * @return Implementation-defined.
     */
    void exit_nested_base(std::string_view name)
    {
        if constexpr (has_exit_nested_v<Derived>)
        {
            derived_->exit_nested(name);
        }
        else
        {
            throw std::runtime_error(
                "exit_nested is not a member function, please implement it in the "
                "derived class");
        }
    }
    Derived* derived_ = static_cast<Derived*>(this);
};

///@}
///@}

}  // namespace game::support
#endif // SUPPORT_VISIT_VISITOR_BASE
