#include <gtest/gtest.h>

#include "core/support/utils/lookup_table.hpp"

namespace core::utils {

TEST(LookupTableTest, GivenTwoElementsCorrectOneAssociatedToTypeKey)
{
    // Given
    using TestTable = LookupTable<
        TableItem<int, std::string>,
        TableItem<float, double, bool, int>,
        TableItem<float, uint32_t>>;

    // When
    using CorrectSingleItem = TestTable::get_type<int>::type;

    // Then
    ASSERT_TRUE((std::is_same<std::string, CorrectSingleItem>::value));

    // When
    using CorrectMultipleItem = TestTable::get_type<float>::type;

    // Then
    ASSERT_TRUE(
        (std::is_same<std::tuple<double, bool, int, uint32_t>, CorrectMultipleItem>::
             value));
}

TEST(TableElementTest, ForEachInTableElement)
{
    using Element = TableItem<int, double, const char*>;
    std::vector<std::string> results;
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

TEST(LookupTableTest, ForEachElementForEachInTableElement)
{
    using Element1 = TableItem<int, double, const char*>;
    using Element2 = TableItem<float, uint32_t, std::string>;
    using Element3 = TableItem<bool, uint8_t>;
    using Table = LookupTable<Element1, Element2, Element3>;
    std::set<std::string> key_set;
    std::set<std::string> expected_key_set {
        typeid(int).name(), typeid(float).name(), typeid(bool).name()};
    std::vector<std::string> results;
    Table::for_each_element(
        [&results, &key_set](auto type)
        {
            key_set.insert(typeid(typename decltype(type)::Key).name());
            decltype(type)::for_each_in_table_element(
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