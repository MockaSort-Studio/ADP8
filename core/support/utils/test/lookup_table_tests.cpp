#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>

#include "core/support/utils/lookup_table.hpp"

namespace core::utils {
using TestTable = LookupTable<
    TableItem<int, std::string>,
    TableItem<float, double, bool, int>,
    TableItem<char, uint32_t>>;

TEST(LookupTableTest, GivenUniqueElementsCorrectTypeExtracted)
{
    // When
    using CorrectSingleItem = std::tuple_element_t<0, TestTable::get_values_t<int>>;

    // Then
    ASSERT_TRUE((std::is_same_v<std::string, CorrectSingleItem>));

    // When
    using CorrectMultipleItem = TestTable::get_values_t<float>;

    // Then
    ASSERT_TRUE((std::is_same_v<std::tuple<double, bool, int>, CorrectMultipleItem>));
}

TEST(LookupTableTest, RuntimeValueExtractiontest)
{
    auto tuple_int   = std::make_tuple(std::string("pippo")); 
    auto tuple_float = std::make_tuple(0.1, false, 1);
    auto tuple_char  = std::make_tuple(10000u);
    
    auto table_values = std::make_tuple(
        tuple_int,         
        tuple_float,
        tuple_char
    );

    auto table = TestTable(table_values);

    EXPECT_EQ(std::get<0>(table.GetValue<int>()), "pippo");
    EXPECT_EQ(table.GetValue<float>(), tuple_float);
    EXPECT_EQ(table.GetValue<char>(), tuple_char);
}
TEST(LookupTableTest, IsLookupTableTest)
{
    ASSERT_TRUE(is_lookup_table_v<TestTable>);

    using FakeTable = std::tuple<double, bool, int>;
    ASSERT_FALSE((is_lookup_table_v<FakeTable>));
}

TEST(TableElementTest, ForEachInTableElement)
{
    using Element = TableItem<int, double, const char*>;
    std::vector<std::string> results;

    ASSERT_TRUE((std::is_same_v<Element::Values, std::tuple<double, const char*>>));

    Element::for_each_in_table_element(
        [&results](const auto& value)
        {
            ASSERT_EQ(typeid(Element::Key).name(), typeid(int).name());
            results.emplace_back(typeid(value).name());
        });

    ASSERT_EQ(results.size(), 2);
    EXPECT_EQ(results[0], typeid(double).name());
    EXPECT_EQ(results[1], typeid(const char*).name());
}

TEST(LookupTableTest, ForEachElement)
{
    using Element1 = TableItem<int, double, const char*>;
    using Element2 = TableItem<float, uint32_t, std::string>;
    using Element3 = TableItem<bool, uint8_t>;

    using Table = LookupTable<Element1, Element2, Element3>;

    std::set<std::string> key_set;
    std::set<std::string> expected_key_set {
        typeid(int).name(), typeid(float).name(), typeid(bool).name()};
    std::vector<std::string> results;

    Table::for_each(
        [&results, &key_set](auto type)
        {
            using CurrentElement = decltype(type);
            key_set.insert(typeid(typename CurrentElement::Key).name());

            CurrentElement::for_each_in_table_element(
                [&results](const auto& value)
                { results.emplace_back(typeid(value).name()); });
        });

    ASSERT_EQ(results.size(), 5);
    ASSERT_EQ(key_set.size(), 3);
    EXPECT_EQ(key_set, expected_key_set);

    EXPECT_EQ(results[0], typeid(double).name());
    EXPECT_EQ(results[1], typeid(const char*).name());
    EXPECT_EQ(results[2], typeid(uint32_t).name());
    EXPECT_EQ(results[3], typeid(std::string).name());
    EXPECT_EQ(results[4], typeid(uint8_t).name());
}
}  // namespace core::utils