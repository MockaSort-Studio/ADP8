#ifndef SUPPORT_VISIT_PROPERTY
#define SUPPORT_VISIT_PROPERTY

#include <string_view>
namespace sert::support {

#define BEGIN_VISITABLE(TypeName)               \
    inline static constexpr auto GetVisitable() \
    {                                           \
        using Type = TypeName;                  \
            auto v = sert::support::visitable<Type>(

#define ADD_PROPERTY_BEGIN(TypeName, PropertyName) \
    inline static constexpr auto GetVisitable()    \
    {                                              \
        using Type = TypeName;                     \
            auto v = sert::support::visitable<Type>(sert::support::property(#PropertyName, &Type::PropertyName)

#define ADD_PROPERTY(PropertyName) \
    , sert::support::property(#PropertyName, &Type::PropertyName)

#define ADD_PROPERTY_END(PropertyName)                            \
    , sert::support::property(#PropertyName, &Type::PropertyName) \
                );                                                \
    return v;                                                     \
    }

#define END_VISITABLE() \
                );      \
    return v;           \
    }

/**
 * \addtogroup support Support
 * \brief Supporting classes and functions
 * @{
 */

/**
 * \addtogroup serialization Visitable and serialization utilities
 * \brief Supporting classes and functions for serializing, deserializing and visiting
 * objects
 * \ingroup support
 * @{
 */

/**
 * @brief Represents a visitable property of a struct.
 *
 * This struct template allows for the creation of named ***public*** properties that are
 * members of a specific struct.
 *
 * @tparam Class The class that contains the member.
 * @tparam MemberType The type of the member.
 */
template <typename Class, typename MemberType>
struct Property
{
    /** @brief Alias for the member type. */
    using Type = MemberType;

    /**
     * @brief Constructs a Property object.
     *
     * @param name The name of the property.
     * @param member Pointer to the member of the class.
     */
    constexpr Property(const char* name, MemberType Class::* member)
        : name_ {name}, member_ {member}
    {}

    /**
     * @brief Gets the modifiable reference of the member described by a property for a
     * given instance.
     *
     * @param instance The instance of the class.
     * @return Type& Reference to the member value.
     */
    Type& Get(Class& instance) const { return instance.*member_; }

    /** The name of the property. This usually matches the member name */
    const char* name_;
    /** Pointer to the member of the class. */
    MemberType Class::* member_;
};

/**
 * @brief  Helper function to create a Property object.
 *
 * @tparam Class The class type that contains the member.
 * @tparam MemberType  The type of the member.
 * @param name The name of the property.
 * @param member Pointer to the member of the class.
 * @return constexpr Property<Class, MemberType> The created Property object.
 */
template <typename Class, typename MemberType>
constexpr inline auto property(const char* name, MemberType Class::* member)
{
    return Property<Class, MemberType> {name, member};
}

}  // namespace sert::support

/// @}
/// @}

#endif  // SUPPORT_VISIT_PROPERTY
