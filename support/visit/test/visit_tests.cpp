#include <gtest/gtest.h>

#include "support/visit/property.hpp"
#include "support/visit/visitable.hpp"
#include "support/visit/visitor_base.hpp"
#include "test_visitors.hpp"

namespace sert::test {

struct Mock
{
    int c {9};
    double d {6.5};
    std::string s {"cane"};

    ADD_PROPERTY_BEGIN(Mock, c)
    ADD_PROPERTY(d)
    ADD_PROPERTY_END(s)
};

struct MockNested
{
    int c_nested {1};
    double d_nested {1.2};
    std::string s_nested {"cagna"};

    Mock m {};

    ADD_PROPERTY_BEGIN(MockNested, c_nested)
    ADD_PROPERTY(d_nested)
    ADD_PROPERTY(s_nested)
    ADD_PROPERTY(m)  // ---|
    END_VISITABLE()  // ---|--->  equivalent to ADD_PROPERTY_END(m)
};

}  // namespace sert::test

TEST(Visitable, GivenMockNestedStructureType_MembersTraitAreCorrect)
{
    using namespace sert::test;

    MockNested m {};

    struct NotVisitable
    {
    };

    EXPECT_FALSE(sert::support::has_get_visitable<decltype(m.c_nested)>::value);
    EXPECT_TRUE(sert::support::has_get_visitable<decltype(m)>::value);
    EXPECT_TRUE(sert::support::has_get_visitable<decltype(m.m)>::value);

    EXPECT_FALSE(sert::support::has_exit_nested_v<SimpleVisitor>);
    EXPECT_FALSE(sert::support::has_exit_nested_v<SimpleVisitor>);
    EXPECT_TRUE(sert::support::has_visit_nested_v<NestedVisitor>);
    EXPECT_TRUE(sert::support::has_visit_nested_v<NestedVisitor>);
}

TEST(Visitable, GivenMockStructure_MembersAreVisited)
{
    using namespace sert::test;
    auto r = sert::support::is_i_visitable_v<decltype(Mock::GetVisitable())>;
    ASSERT_TRUE(r);

    Mock m {};
    SimpleVisitor visitor {};

    Mock::GetVisitable().accept(visitor, m);
    EXPECT_TRUE(visitor.c_result.first == "c" and visitor.c_result.second == m.c);
    EXPECT_TRUE(visitor.d_result.first == "d" and visitor.d_result.second == m.d);
    EXPECT_TRUE(visitor.s_result.first == "s" and visitor.s_result.second == m.s);

    m.c = 33;
    m.d = 33.3;
    m.s = "maiale";

    Mock::GetVisitable().accept(visitor, m);
    EXPECT_TRUE(visitor.c_result.first == "c" and visitor.c_result.second == m.c);
    EXPECT_TRUE(visitor.d_result.first == "d" and visitor.d_result.second == m.d);
    EXPECT_TRUE(visitor.s_result.first == "s" and visitor.s_result.second == m.s);
}

TEST(Visitable, GivenMockNestedStructure_MembersAreVisited)
{
    using namespace sert::test;

    MockNested m {};

    NestedVisitor visitor {};

    MockNested::GetVisitable().accept(visitor, m);

    ASSERT_EQ(visitor.enter_names.size(), 1);
    ASSERT_EQ(visitor.exit_names.size(), 1);
    ASSERT_TRUE(visitor.exit_names.back() == "m" and visitor.enter_names.back() == "m");

    EXPECT_EQ(visitor.c_result.first, "c");
    EXPECT_EQ(visitor.d_result.first, "d");
    EXPECT_EQ(visitor.s_result.first, "s");

    EXPECT_EQ(visitor.c_result.second, m.m.c);
    EXPECT_EQ(visitor.d_result.second, m.m.d);
    EXPECT_EQ(visitor.s_result.second, m.m.s);
}
